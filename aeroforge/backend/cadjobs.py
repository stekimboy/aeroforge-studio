"""CAD builds in WORKER PROCESSES, so previews and exports overlap.

Why processes: OpenCASCADE is not thread-safe and its Python bindings hold the
GIL, so the old in-request builds ran strictly one after another - five gallery
previews, each a full CAD build, in single file. Separate processes build them
genuinely concurrently on a design dict that is plain JSON data, and the
parent just serves the file the worker wrote.

What this module deliberately does NOT change: the build functions the workers
call are the exact ones the request thread used to call, with the exact same
tessellation tolerances - same design in, byte-identical artifact out. All
caching and per-design locking stays in `backend.api`; a job function is pure
"build this design's file(s) and report".

The module top level imports only the stdlib: worker processes are started
with the `spawn` method (the only one on Windows) and re-import this module,
and the parent imports it at boot - neither should pay for cadquery here. The
heavy imports happen inside the job functions, once per worker process, which
is why the pool is WARMED in the background at server startup (a cold worker
pays ~10-20 s of cadquery/neuralfoil import before its first build).

`AEROFORGE_CAD_WORKERS=0` forces every job inline in the calling thread
(the pre-pool behaviour); any other value overrides the worker count.
"""
from __future__ import annotations

import concurrent.futures as _cf
import multiprocessing as _mp
import os
import threading
from concurrent.futures.process import BrokenProcessPool
import json
from pathlib import Path

__all__ = ["run_job", "warm_pool", "job_preview_stl", "job_parts_previews",
           "job_export"]


def _default_workers() -> int:
    # CAD builds are CPU-bound and memory-hungry (~0.5-1 GB per worker):
    # two workers on this 4-core / 16 GB box, never more than three.
    cpus = os.cpu_count() or 2
    return max(1, min(3, cpus - 2)) if cpus > 2 else 1


MAX_WORKERS = int(os.environ.get("AEROFORGE_CAD_WORKERS",
                                 str(_default_workers())))

#: (linear, angular) tessellation of the PREVIEW meshes only. The exports'
#: tolerances live in `cad.exporters` and are not touched by this.
PREVIEW_TOL = (0.08, 0.2)

_POOL: _cf.ProcessPoolExecutor | None = None
_POOL_LOCK = threading.Lock()


def _pool() -> _cf.ProcessPoolExecutor:
    global _POOL
    with _POOL_LOCK:
        if _POOL is None:
            _POOL = _cf.ProcessPoolExecutor(
                max_workers=MAX_WORKERS,
                mp_context=_mp.get_context("spawn"))
        return _POOL


def _reset_pool() -> None:
    global _POOL
    with _POOL_LOCK:
        pool, _POOL = _POOL, None
    if pool is not None:
        pool.shutdown(wait=False, cancel_futures=True)


def run_job(fn, /, *args):
    """Run one CAD job, in a worker process when the pool is enabled.

    Blocking, like the in-process call it replaces - the caller already holds
    the per-design lock, so 'never build the same design twice concurrently'
    is preserved. If the pool is broken (a worker was killed, ran out of
    memory, ...) it is rebuilt once; if that fails too, the job runs inline in
    this thread so a pool problem can never take the feature down.
    """
    if MAX_WORKERS <= 0:
        return fn(*args)
    for _attempt in range(2):
        try:
            return _pool().submit(fn, *args).result()
        except BrokenProcessPool:
            _reset_pool()
    return fn(*args)


# ---------------------------------------------------------------------------
# warm-up
# ---------------------------------------------------------------------------

def _warm() -> int:
    """Pay the heavy import bill now, in the worker, so the first real build
    doesn't."""
    from .cad import exporters as _exporters          # noqa: F401
    return os.getpid()


def warm_pool() -> None:
    """Spawn every worker and run its imports, in the background.

    Called from a daemon thread at server startup: submitting MAX_WORKERS
    no-op jobs makes the executor spawn its full complement, and each new
    process runs `_warm`'s imports concurrently. Failures are swallowed - the
    pool will simply warm on first use instead.
    """
    if MAX_WORKERS <= 0:
        return
    try:
        futures = [_pool().submit(_warm) for _ in range(MAX_WORKERS)]
        for f in futures:
            f.result(timeout=300)
    except Exception:
        pass


def warm_pool_async() -> None:
    threading.Thread(target=warm_pool, name="cad-pool-warmup",
                     daemon=True).start()


# ---------------------------------------------------------------------------
# jobs (top-level functions: they must be picklable by name for the workers)
# ---------------------------------------------------------------------------

def job_preview_stl(design: dict, out_dir: str) -> dict:
    """Build one design's single-solid preview STL - same builder, same path,
    published atomically as before. PREVIEW_TOL is the viewer's tessellation
    (speed pass 2026-08-28: 0.05/0.12 wrote 18 MB and cost 6.4 s per preview
    against 5.5 MB / 1.8 s at 0.08/0.2 on the same solid; the exports keep
    their own tolerances untouched in `exporters`)."""
    from .cad.exporters import write_stl_verified

    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    solid, meta = _solid_for(design)
    tmp = out / f"_{design['id']}.partial.stl"
    write_stl_verified(solid, tmp, *PREVIEW_TOL)
    tmp.replace(out / f"{design['id']}.stl")
    return meta


def build_refusals(meta: dict) -> list[str]:
    """Every feature the build REFUSED, as one sentence each, read off the
    builder's meta the way the tests read it: any report entry with
    `ok: false` and a `reason` / `skipped`, plus the builder's own
    `warnings`. Generic across the type modules (their reports share the
    `{ok, reason}` shape but not the nesting)."""
    found: list[str] = []
    seen: set[str] = set()

    def _label(path: list[str]) -> str:
        keep = [k for k in path if k not in ("servos", "report", "conduits",
                                             "bays", "horns")]
        return " / ".join(keep) or "build"

    def _walk(o, path: list[str]) -> None:
        if isinstance(o, dict):
            if o.get("ok") is False:
                why = o.get("reason") or o.get("skipped") or o.get("error")
                if why:
                    msg = f"{_label(path)}: {why}"
                    if msg not in seen:
                        seen.add(msg)
                        found.append(msg)
            for k, v in o.items():
                if k in ("warnings",) and isinstance(v, list):
                    for w in v:
                        if isinstance(w, str) and w not in seen:
                            seen.add(w)
                            found.append(w)
                elif isinstance(v, (dict, list)):
                    _walk(v, path + [str(k)])
        elif isinstance(o, list):
            for v in o:
                if isinstance(v, (dict, list)):
                    _walk(v, path)
    _walk(meta or {}, [])
    return found[:12]


def job_parts_previews(design: dict, out_dir: str) -> list[dict]:
    """Build one STL per named part (the exploded preview). Same loop and
    file names as the old in-request `api._build_preview_parts`, at
    PREVIEW_TOL; returns the data the manifest needs."""
    from .cad.exporters import write_stl_verified

    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    did = design["id"]
    parts, meta = _parts_for(design)
    # Honest refusals (task 2, 2026-08-28): a feature the builder could not
    # make - a bay that would not carve, a servo pocket the section cannot
    # bury, a wire run that cannot stay inside the skin - used to be
    # visible only in the export's meta, which the preview discarded, so
    # the viewer showed a solid body and said nothing. They go to a
    # sidecar the manifest endpoint reads (no CAD import in api.py).
    try:
        (out / f"{did}__refusals.json").write_text(
            json.dumps(build_refusals(meta)), encoding="utf-8")
    except Exception:
        pass
    built: list[dict] = []
    for name, solid in parts.items():
        if name == "cg_marker":
            continue                     # the viewer draws its own CG marker
        stl = out / f"{did}__{name}.stl"
        if not (stl.exists() and stl.stat().st_size > 0):
            tmp = out / f"_{did}__{name}.partial.stl"
            try:
                write_stl_verified(solid, tmp, *PREVIEW_TOL)
            except Exception:
                continue
            tmp.replace(stl)
        built.append({"name": name, "volume_mm3": round(solid.Volume(), 1)})
    return built


def job_export(design: dict, fmt: str, progress_path: str | None = None
               ) -> str:
    """Build one export file (step / stl / stl_parts) with the unchanged
    exporter functions; returns the path as a string (picklable).

    `progress_path` names the per-job JSON the builders mark their stage
    boundaries into (`backend.progress`), which `/api/export/status` reads
    for the progress bar. None (tests, scripts) reports nothing."""
    from . import progress
    from .cad import exporters

    progress.begin(progress_path)
    try:
        if fmt == "step":
            parts, _meta = _parts_for(design)
            path = exporters.export_step(design, parts=parts)
        elif fmt == "stl_parts":
            parts, meta = _parts_for(design)
            path = exporters.export_stl_parts(design, parts=parts, meta=meta)
        else:
            solid, _meta = _solid_for(design)
            path = exporters.export_stl(design, solid=solid)
    finally:
        progress.end()
    return str(path)


# ---------------------------------------------------------------------------
# the BREP cache (backend.cad.brepcache): build once per design, per kind
# ---------------------------------------------------------------------------

def _parts_for(design: dict) -> tuple[dict, dict]:
    """`build_design_parts(design)`, served from the BREP cache when a
    previous job (exploded preview, STEP, parts zip) already built it. On a
    miss the fresh parts are cached BEFORE anything tessellates them, so a
    later loader sees exactly what the exporters see now."""
    from .cad.geometry import build_design_parts

    # PARTS ARE NOT SERVED FROM THE CACHE (2026-08-28 identity run on the
    # default swept wing): the one-piece STL from a loaded solid is
    # byte-identical to a direct build, but the STEP written from LOADED
    # parts differs (607977 vs 608107 lines) and four of the six per-part
    # STLs (wings, elevons) differ - the BinTools round-trip of the parts
    # compound is not faithful enough for the exporters. Until that is
    # proven fixed, STEP / parts-zip exports rebuild as they always did;
    # `brepcache.save_parts` / `load_parts` stay for the probe tools.
    return build_design_parts(design)


def _solid_for(design: dict) -> tuple[object, dict]:
    """`build_design_solid(design)`, cached the same way (one-piece preview
    and the STL export share it)."""
    from .cad import brepcache
    from .cad.geometry import build_design_solid

    hit = brepcache.load_solid(design)
    if hit is not None:
        return hit
    solid, meta = build_design_solid(design)
    brepcache.save_solid(design, solid, meta)
    return solid, meta
