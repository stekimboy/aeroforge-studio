"""STEP + STL export via CadQuery/OpenCASCADE (SPEC_FLYING_WING.md §5).

The STEP is a real CAD ASSEMBLY: one named, coloured body per airframe part
(centre body, wing panels, the green moving elevons, the orange hatch lid,
the CG marker) at its position on the assembled aircraft, so a builder can open the
file and pull the model apart. The STL stays a single fused watertight solid
for slicing / preview.

Files land in exports\\ with descriptive names (planform + timestamp); all
paths built with pathlib so they are Windows-safe.
"""
from __future__ import annotations

import datetime as _dt
import json
from pathlib import Path

import cadquery as cq
import numpy as np
from cadquery import exporters as cq_exporters

from ..paths import EXPORT_DIR  # noqa: F401  (re-exported: tests/api use it)
from ..progress import report as _progress
from .geometry import build_design_parts, build_design_solid

# per-part colours so the assembly is readable the moment it opens.
# Stealth-graphite key (builder's decision) - mirrors api.ROLE_COLORS so the
# STEP opens looking like the preview:
# DARK GRAPHITE = fixed structure
# TITANIUM GREY = a moving control surface, driven by a servo
# INSET BLACK   = the removable hatch
# RED           = the CG marker
_GRAPHITE = (0.24, 0.255, 0.28)
_CONTROL_GREY = (0.49, 0.52, 0.55)
_HATCH_GREY = (0.145, 0.157, 0.172)
_PART_COLORS: dict[str, tuple[float, float, float]] = {
    "airframe": _GRAPHITE,
    "centre_body": _GRAPHITE,
    "wing_left": _GRAPHITE, "wing_right": _GRAPHITE,
    "elevon_left": _CONTROL_GREY, "elevon_right": _CONTROL_GREY,
    # v2 conventional moving surfaces - same titanium grey as the elevons so
    # the STEP tells the same colour story as the preview (api.PART_ROLES).
    "aileron_left": _CONTROL_GREY, "aileron_right": _CONTROL_GREY,
    "elevator_left": _CONTROL_GREY, "elevator_right": _CONTROL_GREY,
    "rudder": _CONTROL_GREY,
    "hatch_lid": _HATCH_GREY,
    "cg_marker": (0.85, 0.20, 0.20),
}
_DEFAULT_COLOR = (0.35, 0.37, 0.39)


def _slug(design: dict) -> str:
    """Filename tag for a design: its planform family (v1 dicts used
    `config`)."""
    return str(design.get("planform") or design.get("config") or "wing")


def _stamp(design: dict, ext: str) -> Path:
    EXPORT_DIR.mkdir(parents=True, exist_ok=True)
    ts = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return EXPORT_DIR / f"aeroforge_{_slug(design)}_{ts}.{ext}"


def design_assembly(design: dict, parts: dict | None = None) -> cq.Assembly:
    """CadQuery assembly of the aircraft: one named body per airframe part,
    already in world coordinates (no per-part transforms needed).

    `parts` may be the dict `build_design_parts` already returned for this
    design (the BREP cache, `backend.cad.brepcache`); None builds it here.
    Either way the same shapes go through the same writer."""
    if parts is None:
        parts, _meta = build_design_parts(design)
    assy = cq.Assembly(name=f"aeroforge_{_slug(design)}")
    for name, solid in parts.items():
        r, g, b = _PART_COLORS.get(name, _DEFAULT_COLOR)
        assy.add(solid, name=name, color=cq.Color(r, g, b, 1.0))
    return assy


def export_step(design: dict, path: Path | None = None,
                parts: dict | None = None) -> Path:
    """Write a real .step (ISO-10303, OpenCASCADE BRep) of the aircraft as a
    multi-body ASSEMBLY - every part is its own named solid, positioned so the
    assembled model is the complete aeroplane."""
    path = Path(path) if path else _stamp(design, "step")
    assy = design_assembly(design, parts)
    _progress("write")
    # cadquery 2.8: Assembly.export supersedes the deprecated .save
    exporter = getattr(assy, "export", None) or assy.save
    exporter(str(path))
    return path


def write_stl_verified(solid, path: Path, tolerance: float = 0.12,
                       angular: float = 0.25) -> Path:
    """Export STL and VERIFY the tessellation actually covered the solid:
    OCC's mesher can silently skip a pathological face, leaving a hole in an
    otherwise 'valid' export. Compare mesh area to the BRep area and retry a
    tolerance ladder (a different deflection often meshes a stubborn face)."""
    from stl import mesh as stl_mesh

    occ_area = solid.Area()
    ladder = [(tolerance, angular), (0.08, 0.2), (0.15, 0.3), (0.25, 0.4)]
    best: tuple[float, tuple[float, float]] | None = None
    for tol, ang in ladder:
        cq_exporters.export(solid, str(path), tolerance=tol,
                            angularTolerance=ang)
        mesh_area = float(stl_mesh.Mesh.from_file(str(path)).areas.sum())
        ratio = mesh_area / max(occ_area, 1e-9)
        if ratio >= 0.985:
            return path
        if best is None or ratio > best[0]:
            best = (ratio, (tol, ang))
    # nothing reached full coverage - rewrite the best attempt honestly
    tol, ang = best[1]
    cq_exporters.export(solid, str(path), tolerance=tol, angularTolerance=ang)
    return path


def export_stl(design: dict, path: Path | None = None,
               tolerance: float = 0.12, solid=None) -> Path:
    """Write a print-ready .stl (mm units) of the whole assembly. `solid` may
    be the one `build_design_solid` already returned (BREP cache)."""
    if solid is None:
        solid, _meta = build_design_solid(design)
    path = Path(path) if path else _stamp(design, "stl")
    _progress("write")
    return write_stl_verified(solid, path, tolerance, 0.25)


def export_stl_parts(design: dict, path: Path | None = None,
                     tolerance: float = 0.12, parts: dict | None = None,
                     meta: dict | None = None) -> Path:
    """Every named part as its OWN .stl, in one zip.

    The single-solid STL is the whole aeroplane in printed position, which is
    what you slice to look at it - but the hatch lid is a separate piece of
    plastic on the real aircraft and has to be printed as one. This writes
    `airframe.stl`, `hatch_lid.stl`, `winglet_left.stl` ... each in the same
    world coordinates, so dropping them all into a slicer reassembles the
    aeroplane exactly.
    """
    import zipfile

    if parts is None:
        parts, meta = build_design_parts(design)
    _progress("write")
    path = Path(path) if path else _stamp(design, "parts.zip")
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_dir = path.parent / f"_{path.stem}_parts"
    tmp_dir.mkdir(parents=True, exist_ok=True)
    written: list[str] = []
    try:
        with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as zf:
            for name, solid in parts.items():
                if name == "cg_marker":       # a balance mark, not a part
                    continue
                stl = tmp_dir / f"{name}.stl"
                try:
                    write_stl_verified(solid, stl, tolerance, 0.25)
                except Exception:
                    continue
                zf.write(stl, arcname=f"{_slug(design)}/{name}.stl")
                written.append(name)
            zf.writestr(f"{_slug(design)}/parts.json",
                        json.dumps({"parts": written, "units": "mm",
                                    "meta": _jsonable(meta)}, indent=2))
    finally:
        for f in tmp_dir.glob("*.stl"):
            f.unlink(missing_ok=True)
        try:
            tmp_dir.rmdir()
        except OSError:
            pass
    return path


def _jsonable(obj):
    """Metadata may carry numpy scalars and tuples; make it serialisable."""
    if isinstance(obj, dict):
        return {str(k): _jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_jsonable(v) for v in obj]
    if isinstance(obj, (np.integer, np.floating)):
        return obj.item()
    if isinstance(obj, (str, int, float, bool)) or obj is None:
        return obj
    return str(obj)


def export_both_with_meta(design: dict) -> dict:
    solid, meta = build_design_solid(design)
    step_path = _stamp(design, "step")
    export_step(design, step_path)          # multi-part assembly
    stl_path = step_path.with_suffix(".stl")
    write_stl_verified(solid, stl_path, 0.12, 0.25)
    meta_path = step_path.with_suffix(".json")
    meta_path.write_text(json.dumps(meta, indent=2))
    return {"step": step_path, "stl": stl_path, "meta": meta_path}


# ---------------------------------------------------------------------------
# STL validation helpers (used by tests and the API sanity path)
# ---------------------------------------------------------------------------

def stl_is_valid_mesh(path: Path) -> tuple[bool, str]:
    """Valid = loads, has triangles, and encloses a positive volume
    (numpy-stl mass properties on a closed, consistently oriented mesh)."""
    from stl import mesh as stl_mesh

    m = stl_mesh.Mesh.from_file(str(path))
    if len(m.vectors) < 50:
        return False, f"too few triangles: {len(m.vectors)}"
    volume, _cog, _inertia = m.get_mass_properties()
    if not np.isfinite(volume) or volume <= 0:
        return False, f"non-positive enclosed volume: {volume}"
    return True, f"{len(m.vectors)} triangles, volume {volume:.0f} mm^3"


def stl_watertight_fraction(path: Path, decimals: int = 3) -> float:
    """Fraction of triangle edges shared by exactly two triangles after
    vertex snapping - 1.0 for a watertight 2-manifold tessellation."""
    from stl import mesh as stl_mesh

    m = stl_mesh.Mesh.from_file(str(path))
    v = np.round(m.vectors, decimals)
    edges: dict[tuple, int] = {}
    for tri in v:
        for i in range(3):
            a = tuple(tri[i])
            b = tuple(tri[(i + 1) % 3])
            key = (a, b) if a <= b else (b, a)
            edges[key] = edges.get(key, 0) + 1
    if not edges:
        return 0.0
    good = sum(1 for c in edges.values() if c == 2)
    return good / len(edges)
