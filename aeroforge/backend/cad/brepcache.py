"""On-disk cache of BUILT designs (speed pass, 2026-08-28).

The same design was being built from scratch up to three times: once for the
one-piece preview STL (`build_design_solid`), once for the exploded preview
(`build_design_parts`) and once more for every export - a STEP click after
the previews had already been built paid the whole 3-10 minute CAD bill a
second time, queued behind the preview that was building the very same
parts. Nothing about a design changes after it is generated, so the first
job to build a design's parts (or its one-piece solid) writes them here as
binary BREP, and every later job that needs the same shapes LOADS them and
only tessellates / writes the file.

Why this cannot change the artifacts:

* the shapes are written with `BinTools` - binary, doubles exact, and WITH
  their triangulation - and written BEFORE the job that built them
  tessellates anything for its own output, so a loader sees exactly the
  in-memory state a fresh build hands the exporters (including whatever
  build-time triangulation `_tessellates_cleanly` left on faces, which
  `BRepMesh` consults). Measured on the default flying wing (identity run,
  2026-08-28): the one-piece STL is byte-identical from a loaded solid and
  from a fresh build - so the SOLID path (preview + STL export) is served
  from the cache. The PARTS path is NOT: the STEP written from loaded parts
  differed (607977 vs 608107 lines) and the wing/elevon per-part STLs
  differed, so `cadjobs._parts_for` builds fresh every time until the
  round-trip is proven faithful (see docs/PERF_NOTES.md);
* the key hashes the WHOLE design dict (minus its per-generate id and the
  prose) together with a digest of every `backend/cad/*.py` source, so an
  edit to any builder - or a different design - can never hit a stale
  entry.

Top level is stdlib only: `backend.api` asks `has_parts` / `has_solid` to
label a build's timing honestly (a cache hit is not an ETA data point for a
cold build), and it must not import cadquery. The OCC calls live inside the
functions that need them, which only workers call.

`AEROFORGE_BREP_CACHE=0` disables the cache (tests that want cold builds).
"""
from __future__ import annotations

import hashlib
import json
import os
import shutil
from pathlib import Path
from typing import Any

from ..paths import EXPORT_DIR

__all__ = ["CACHE_DIR", "design_key", "has_parts", "has_solid",
           "load_parts", "save_parts", "load_solid", "save_solid"]

CACHE_DIR = EXPORT_DIR / "brep"
_CAD_DIR = Path(__file__).resolve().parent
_CODE_DIGEST: str | None = None
_SKIP_KEYS = frozenset({"id", "guidance", "notes", "character"})


def enabled() -> bool:
    return os.environ.get("AEROFORGE_BREP_CACHE", "1") != "0"


def _code_digest() -> str:
    """sha256 over every CAD module's source, so a builder edit invalidates
    the whole cache. Computed once per process (the workers are
    long-lived and hold the code they imported anyway)."""
    global _CODE_DIGEST
    if _CODE_DIGEST is None:
        h = hashlib.sha256()
        for p in sorted(_CAD_DIR.glob("*.py")):
            h.update(p.name.encode())
            h.update(p.read_bytes())
        _CODE_DIGEST = h.hexdigest()[:16]
    return _CODE_DIGEST


def design_key(design: dict) -> str:
    """Content key of a design: everything the builders read, nothing that
    varies between two generates of the same aeroplane."""
    body = {k: v for k, v in design.items() if k not in _SKIP_KEYS}
    text = json.dumps(body, sort_keys=True, default=str, separators=(",", ":"))
    h = hashlib.sha256()
    h.update(_code_digest().encode())
    h.update(text.encode())
    return h.hexdigest()[:32]


def _dir(design: dict, kind: str) -> Path:
    return CACHE_DIR / f"{design_key(design)}.{kind}"


def _manifest_ok(d: Path) -> bool:
    return (d / "manifest.json").is_file()


def has_parts(design: dict) -> bool:
    return enabled() and _manifest_ok(_dir(design, "parts"))


def has_solid(design: dict) -> bool:
    return enabled() and _manifest_ok(_dir(design, "solid"))


# ---------------------------------------------------------------------------
# OCC side (workers only)
# ---------------------------------------------------------------------------

def _write_shape(shape, path: Path) -> None:
    from OCP.BinTools import BinTools

    ok = BinTools.Write_s(shape.wrapped, str(path))
    if ok is False:
        raise OSError(f"BinTools could not write {path}")


def _read_shape(path: Path):
    from OCP.BinTools import BinTools
    from OCP.TopoDS import TopoDS_Shape
    from cadquery.occ_impl.shapes import Shape

    s = TopoDS_Shape()
    if not BinTools.Read_s(s, str(path)) or s.IsNull():
        raise OSError(f"BinTools could not read {path}")
    return Shape.cast(s)


def _jsonable(obj: Any) -> Any:
    from .geometry import _jsonable as conv
    return conv(obj)


def _publish(tmp: Path, final: Path) -> None:
    """Atomic directory publish: a reader never sees a half-written entry."""
    if final.exists():
        shutil.rmtree(tmp, ignore_errors=True)
        return
    try:
        tmp.rename(final)
    except OSError:
        shutil.rmtree(tmp, ignore_errors=True)
        return
    _prune()


#: Keep the cache bounded: `exports/` has filled this disk before (28 GB free
#: on 2026-08-28; a parts set is ~50 MB). Oldest entries beyond either limit
#: go first; the one just written is always kept.
MAX_ENTRIES = 24
MAX_BYTES = 3 * 1024**3


def _prune() -> None:
    try:
        entries = [d for d in CACHE_DIR.iterdir()
                   if d.is_dir() and ".tmp" not in d.name]
    except OSError:
        return
    sized = []
    for d in entries:
        try:
            size = sum(f.stat().st_size for f in d.iterdir())
            sized.append((d.stat().st_mtime, size, d))
        except OSError:
            continue
    sized.sort()                                   # oldest first
    total = sum(s for _m, s, _d in sized)
    while sized and (len(sized) > MAX_ENTRIES or total > MAX_BYTES):
        _m, size, d = sized.pop(0)
        shutil.rmtree(d, ignore_errors=True)
        total -= size


def save_parts(design: dict, parts: dict, meta: dict) -> None:
    """Persist `build_design_parts` output. Call BEFORE tessellating any of
    the parts for the caller's own file (see the module docstring)."""
    if not enabled():
        return
    final = _dir(design, "parts")
    if final.exists():
        return
    tmp = final.with_name(final.name + f".tmp{os.getpid()}")
    shutil.rmtree(tmp, ignore_errors=True)
    tmp.mkdir(parents=True, exist_ok=True)
    try:
        names = list(parts)
        for i, name in enumerate(names):
            _write_shape(parts[name], tmp / f"{i:02d}_{name}.brep")
        (tmp / "manifest.json").write_text(json.dumps(
            {"kind": "parts", "names": names, "meta": _jsonable(meta)}))
    except Exception:
        shutil.rmtree(tmp, ignore_errors=True)
        return
    _publish(tmp, final)


def load_parts(design: dict) -> tuple[dict, dict] | None:
    """(parts, meta) as `build_design_parts` returned them, or None."""
    if not has_parts(design):
        return None
    d = _dir(design, "parts")
    try:
        man = json.loads((d / "manifest.json").read_text())
        parts = {name: _read_shape(d / f"{i:02d}_{name}.brep")
                 for i, name in enumerate(man["names"])}
        return parts, man["meta"]
    except Exception:
        return None


def save_solid(design: dict, solid, meta: dict) -> None:
    """Persist `build_design_solid` output (same before-tessellation rule)."""
    if not enabled():
        return
    final = _dir(design, "solid")
    if final.exists():
        return
    tmp = final.with_name(final.name + f".tmp{os.getpid()}")
    shutil.rmtree(tmp, ignore_errors=True)
    tmp.mkdir(parents=True, exist_ok=True)
    try:
        _write_shape(solid, tmp / "solid.brep")
        (tmp / "manifest.json").write_text(json.dumps(
            {"kind": "solid", "meta": _jsonable(meta)}))
    except Exception:
        shutil.rmtree(tmp, ignore_errors=True)
        return
    _publish(tmp, final)


def load_solid(design: dict) -> tuple[Any, dict] | None:
    if not has_solid(design):
        return None
    d = _dir(design, "solid")
    try:
        man = json.loads((d / "manifest.json").read_text())
        return _read_shape(d / "solid.brep"), man["meta"]
    except Exception:
        return None
