"""Fin / stabilizer intrusion probe (work plan task 1, 2026-08-27):
no tail surface may stand inside the equipment-bay cavity of any type.

For every configuration this builds the CHEAP path of the type's own CAD
builder - the fuselage/wing loft, the tail surfaces exactly as the builder
lofts them, and `hatch.build_bay` with the builder's own request - through
the builder's probe seam (`_probe=` on `_build` / `_build_parts`), which
hands back the loose tail solids, the compartment solid the bay was cut
with and the airframe after the tail is fused. Nothing downstream of that
seam adds material to a fin: hinges, servos and wire runs only cut.

It then PROVES the result by point classification, never by flags: a grid
of points over each tail surface's overlap with the cavity's bounding box
is classified against the raw fin (is this fin material?), the cavity (is
this compartment air?) and the built airframe (is there material here in
the delivered geometry?). A point that is fin AND cavity AND still solid in
the airframe is an INTRUSION - a blade standing in the compartment. The
count, the estimated volume and the z band of the intruding material are
reported per surface.

Usage:
  .venv/Scripts/python.exe tools_probe_fin_intrusion.py [name-filter]
Prints `RESULT PASS|FAIL <json>` per configuration.
"""
from __future__ import annotations

import json
import math
import os
import sys
import time

os.environ.setdefault("AEROFORGE_HATCH_TRACE", "0")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from OCP.BRepClass3d import BRepClass3d_SolidClassifier   # noqa: E402
from OCP.gp import gp_Pnt                                 # noqa: E402
from OCP.TopAbs import TopAbs_IN, TopAbs_ON               # noqa: E402

from backend.physics.optimizer import optimize_design     # noqa: E402
from backend.physics import variants as _variants         # noqa: E402
from backend.schemas import GenerateRequest               # noqa: E402
from backend.cad import geometry as geo                   # noqa: E402

BOX_FW_DEFAULT = {"length_mm": 700, "width_mm": 1200, "height_mm": 300}
BOX_FW_675 = {"length_mm": 675, "width_mm": 675, "height_mm": 500}
BOX_CONV = {"length_mm": 900, "width_mm": 1400, "height_mm": 300}
BOX_TB = {"length_mm": 1300, "width_mm": 1800, "height_mm": 400}
BOX_MW = {"length_mm": 1200, "width_mm": 1400, "height_mm": 400}
BOX_DELTA = {"length_mm": 950, "width_mm": 1200, "height_mm": 300}

# grid pitch (mm) of the classification over the fin/cavity overlap box
DX, DY, DZ = 4.0, 1.5, 2.0
MAX_POINTS = 60000
INSET = 0.3

CONFIGS: list[dict] = []
for planform in ("swept", "bwb", "plank"):
    CONFIGS.append({"name": f"fw-{planform}-center_fin-default",
                    "airplane_type": "flying_wing", "planform": planform,
                    "vstab": "center_fin", "box": BOX_FW_DEFAULT,
                    "mission": "sport", "cruise": 16.0})
    CONFIGS.append({"name": f"fw-{planform}-center_fin-675",
                    "airplane_type": "flying_wing", "planform": planform,
                    "vstab": "center_fin", "box": BOX_FW_675,
                    "mission": "fpv_cruiser", "cruise": 18.0})
# bell carries no vertical surface; the sweep still runs it so a bell that
# grew one would be caught. Winglets and twin fins stand outboard of the bay
# by construction - one of each proves it.
CONFIGS.append({"name": "fw-bell-none-default", "airplane_type": "flying_wing",
                "planform": "bell", "vstab": None, "box": BOX_FW_DEFAULT,
                "mission": "sport", "cruise": 16.0})
CONFIGS.append({"name": "fw-swept-winglets-default",
                "airplane_type": "flying_wing", "planform": "swept",
                "vstab": None, "box": BOX_FW_DEFAULT, "mission": "sport",
                "cruise": 16.0})
CONFIGS.append({"name": "fw-swept-twin_fin-675", "airplane_type": "flying_wing",
                "planform": "swept", "vstab": "twin_fin", "box": BOX_FW_675,
                "mission": "fpv_cruiser", "cruise": 18.0})
for tail in ("conventional", "t_tail", "v_tail"):
    CONFIGS.append({"name": f"conv-{tail}-default", "airplane_type": "conventional",
                    "planform": None, "vstab": None, "box": BOX_CONV,
                    "mission": "sport", "cruise": 15.0, "tail_type": tail})
CONFIGS.append({"name": "twin_boom-default", "airplane_type": "twin_boom",
                "planform": None, "vstab": None, "box": BOX_TB,
                "mission": "fpv_cruiser", "cruise": 16.0})
CONFIGS.append({"name": "canard-default", "airplane_type": "canard",
                "planform": None, "vstab": None, "box": BOX_MW,
                "mission": "sport", "cruise": 15.0})
CONFIGS.append({"name": "tandem-default", "airplane_type": "tandem",
                "planform": None, "vstab": None, "box": BOX_MW,
                "mission": "sport", "cruise": 15.0})
CONFIGS.append({"name": "delta-default", "airplane_type": "delta",
                "planform": None, "vstab": None, "box": BOX_DELTA,
                "mission": "sport", "cruise": 15.0})


def design_for(cfg: dict) -> dict:
    body = {"airplane_type": cfg["airplane_type"], "mission": cfg["mission"],
            "cruise_speed_ms": cfg["cruise"], "payload_g": 0.0,
            "box": cfg["box"], "n_motors": 1, "build_method": "3d_printed",
            "material": "lw_pla",
            "mount_screws": 4, "mount_screw_d_mm": 3.2,
            "mount_bolt_circle_mm": 16.0}
    if cfg.get("planform"):
        body["planform"] = cfg["planform"]
    if cfg.get("vstab"):
        body["vstab"] = cfg["vstab"]
    if cfg.get("tail_type"):
        body["tail_type"] = cfg["tail_type"]
    req = GenerateRequest(**body)
    inp = req.to_optimizer_input()
    if cfg["airplane_type"] == "flying_wing":
        return optimize_design(inp)
    vs = _variants.generate_variants(inp)
    primary = next((v for v in vs if v.get("primary")), vs[0])
    d = primary["design"] if "design" in primary else primary
    if cfg.get("tail_type") and cfg["airplane_type"] == "conventional":
        # v3.1 gap (ARCHITECTURE.md): the conventional physics does not consume
        # the tail_type axis, while cad/conventional.py builds whatever
        # `geometry.tail.type` says. Stamp it so the t-tail / v-tail
        # GEOMETRY is what gets probed - the same way the CAD suites do.
        d["geometry"].setdefault("tail", {})["type"] = cfg["tail_type"]
        d.setdefault("config", {})["tail_type"] = cfg["tail_type"]
    return d


def build_pieces(design: dict) -> dict:
    """Run the type's builder up to its probe seam. Returns the seam dict:
    `airframe` (tail fused, bay cut), `fins` {name: raw solid}, `cavity`
    (compartment solid or None), `bay_mm`, `warnings`."""
    pr: dict = {}
    mod = geo._type_module(design)
    if mod is None:
        geo._build_parts(design, separate_parts=True, _probe=pr)
    else:
        mod._build(design, separate_parts=True, _probe=pr)
    if "airframe" not in pr:
        raise RuntimeError("builder returned without reaching the probe seam")
    return pr


def _inside(shape, x: float, y: float, z: float) -> bool:
    c = BRepClass3d_SolidClassifier(shape, gp_Pnt(x, y, z), 1e-4)
    return c.State() in (TopAbs_IN, TopAbs_ON)


def _bb(solid):
    b = solid.BoundingBox()
    return (b.xmin, b.xmax, b.ymin, b.ymax, b.zmin, b.zmax)


def classify_surface(name: str, fin, cavity, airframe,
                     airframe_pre=None) -> dict:
    """Grid-classify one tail surface against the cavity and the airframe."""
    fx0, fx1, fy0, fy1, fz0, fz1 = _bb(fin)
    cx0, cx1, cy0, cy1, cz0, cz1 = _bb(cavity)
    # sample strictly INSIDE the overlap box: a grid column landing on the
    # cavity's own wall face classifies ON (= inside) and reads the wall
    # as material in the void (measured: 17 points at x = cavity xmax
    # on the bwb, all solid before the fin fused)
    ox0, ox1 = max(fx0, cx0) + INSET, min(fx1, cx1) - INSET
    oy0, oy1 = max(fy0, cy0) + INSET, min(fy1, cy1) - INSET
    oz0, oz1 = max(fz0, cz0) + INSET, min(fz1, cz1) - INSET
    out = {"surface": name,
           "fin_bbox_mm": [round(v, 1) for v in (fx0, fx1, fy0, fy1, fz0, fz1)],
           "cavity_bbox_mm": [round(v, 1) for v in (cx0, cx1, cy0, cy1, cz0, cz1)]}
    if ox1 <= ox0 or oy1 <= oy0 or oz1 <= oz0:
        # no bounding-box overlap at all - the honest clearance, per axis
        gap = max(ox0 - ox1, oy0 - oy1, oz0 - oz1)
        out.update(overlap_box=False, bbox_clearance_mm=round(gap, 1),
                   raw_overlap_pts=0, intrusion_pts=0, points=0)
        return out
    nx = max(int(math.ceil((ox1 - ox0) / DX)) + 1, 2)
    ny = max(int(math.ceil((oy1 - oy0) / DY)) + 1, 2)
    nz = max(int(math.ceil((oz1 - oz0) / DZ)) + 1, 2)
    scale = 1.0
    if nx * ny * nz > MAX_POINTS:
        scale = (nx * ny * nz / MAX_POINTS) ** (1.0 / 3.0)
        nx, ny, nz = (max(int(n / scale), 2) for n in (nx, ny, nz))
    dx, dy, dz = ((ox1 - ox0) / (nx - 1), (oy1 - oy0) / (ny - 1),
                  (oz1 - oz0) / (nz - 1))
    fw, cw, aw = fin.wrapped, cavity.wrapped, airframe.wrapped
    pw = airframe_pre.wrapped if airframe_pre is not None else None
    raw = 0
    pre_solid = 0
    intr: list[tuple[float, float, float]] = []
    for i in range(nx):
        x = ox0 + i * dx
        for j in range(ny):
            y = oy0 + j * dy
            for k in range(nz):
                z = oz0 + k * dz
                if not _inside(fw, x, y, z):
                    continue
                if not _inside(cw, x, y, z):
                    continue
                raw += 1
                if _inside(aw, x, y, z):
                    # solid BEFORE the fin fused = the cavity solid
                    # and the carved void disagree here (a wall face,
                    # the magnet pad riser) - not fin material
                    if pw is not None and _inside(pw, x, y, z):
                        pre_solid += 1
                        continue
                    intr.append((x, y, z))
    cell = dx * dy * dz
    out.update(overlap_box=True,
               overlap_box_mm=[round(v, 1) for v in (ox0, ox1, oy0, oy1, oz0, oz1)],
               points=nx * ny * nz, grid_mm=[round(dx, 2), round(dy, 2), round(dz, 2)],
               raw_overlap_pts=raw, raw_overlap_cm3=round(raw * cell / 1000.0, 2),
               intrusion_pts=len(intr),
               cavity_solid_pre_fin_pts=(pre_solid if pw is not None
                                         else None),
               intrusion_cm3=round(len(intr) * cell / 1000.0, 2))
    if intr:
        xs = [p[0] for p in intr]
        zs = [p[2] for p in intr]
        ys = [p[1] for p in intr]
        out["intrusion_x_mm"] = [round(min(xs), 1), round(max(xs), 1)]
        out["intrusion_y_mm"] = [round(min(ys), 1), round(max(ys), 1)]
        out["intrusion_z_mm"] = [round(min(zs), 1), round(max(zs), 1)]
        # how far below the cavity's ceiling the deepest blade point stands
        # (the ceiling measured on the cavity solid along that column)
        deepest = min(intr, key=lambda p: p[2])
        zc = deepest[2]
        while zc < cz1 and _inside(cw, deepest[0], deepest[1], zc + 0.5):
            zc += 0.5
        out["deepest_below_roof_mm"] = round(zc - deepest[2], 1)
    return out


def probe(cfg: dict) -> dict:
    t0 = time.time()
    d = design_for(cfg)
    t1 = time.time()
    pr = build_pieces(d)
    t2 = time.time()
    g = d["geometry"]
    res = {"name": cfg["name"], "airplane_type": d.get("airplane_type"),
           "planform": d.get("planform"),
           "vstab": (g.get("vstab") or {}).get("type"),
           "tail_type": (d.get("config") or {}).get("tail_type"),
           "bay_ok": pr.get("cavity") is not None,
           "bay_rung": (pr.get("bay_mm") or {}).get("rung"),
           "bay_reason": (pr.get("bay_mm") or {}).get("reason"),
           "cavity_extended_mm": (pr.get("bay_mm") or {}).get("cavity_extended_mm"),
           "fins": list(pr.get("fins") or {}),
           "warnings": [w for w in (pr.get("warnings") or [])
                        if "root" in w or "fin" in w.lower()
                        or "stab" in w.lower()],
           "surfaces": []}
    cavity = pr.get("cavity")
    airframe = pr["airframe"]
    if cavity is not None:
        for nm, fin in (pr.get("fins") or {}).items():
            res["surfaces"].append(classify_surface(
                nm, fin, cavity, airframe, pr.get("airframe_pre")))
    res["intrusion_pts"] = sum(s["intrusion_pts"] for s in res["surfaces"])
    res["raw_overlap_pts"] = sum(s["raw_overlap_pts"] for s in res["surfaces"])
    res["ok"] = res["intrusion_pts"] == 0 and (
        cavity is not None or not (pr.get("fins") or {}))
    res["secs"] = {"design": round(t1 - t0), "build": round(t2 - t1),
                   "classify": round(time.time() - t2)}
    return res


def main() -> None:
    filt = sys.argv[1] if len(sys.argv) > 1 else ""
    for cfg in CONFIGS:
        if filt and filt not in cfg["name"]:
            continue
        try:
            out = probe(cfg)
        except Exception as exc:
            out = {"name": cfg["name"], "ok": False,
                   "reason": f"{type(exc).__name__}: {exc}"}
        verdict = ("PASS" if out.get("ok")
                   else ("NO-BAY" if out.get("bay_ok") is False else "FAIL"))
        print(f"RESULT {verdict:6s} "
              f"{json.dumps(out, default=str)}", flush=True)


if __name__ == "__main__":
    main()
