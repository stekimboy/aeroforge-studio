"""Configuration sweep for the rear cavity extension (user, 2026-08-27:
"make sure the inner hull ... smooth and continuous extension is fixed
across all designs and outputs").

For every configuration in the sweep this optimizes the design, lofts the
airframe, fuses the motor boss (the same steps `build_design_parts` takes
before the bay) and runs `hatch.build_bay` with the EXACT request the
build makes (`geometry.bay_request`). It then reports, per configuration,
whether the bay carved, how far the cavity extends past the hatch and the
station band - or the reason it refused. It skips hinges/servos/conduits,
so it is ~5x faster than a full build; the extension is what decides
whether the servo runs get the hull or the box galleries.

Usage:
  .venv/Scripts/python.exe tools_probe_cavity.py [name-filter]
"""
import json
import os
import sys
import time

os.environ.setdefault("AEROFORGE_HATCH_TRACE", "0")
sys.path.insert(0, ".")

from backend.physics.optimizer import optimize_design          # noqa: E402
from backend.schemas import GenerateRequest                     # noqa: E402
from backend.cad import geometry as geo                          # noqa: E402
from backend.cad import hatch                                    # noqa: E402

BOX_DEFAULT = {"length_mm": 700, "width_mm": 1200, "height_mm": 300}
BOX_USER = {"length_mm": 650, "width_mm": 650, "height_mm": 500}

CONFIGS = []
for planform in ("swept", "bwb", "plank", "bell"):
    for vstab in (None, "center_fin"):
        if planform == "bell" and vstab is not None:
            continue            # bell carries no vertical surface
        for tag, box, mission in (("default", BOX_DEFAULT, "sport"),
                                  ("user650", BOX_USER, "fpv_cruiser")):
            CONFIGS.append({
                "name": f"fw-{planform}-{vstab or 'defaultfin'}-{tag}",
                "airplane_type": "flying_wing", "planform": planform,
                "vstab": vstab, "box": box, "mission": mission,
                "cruise": 16.0 if tag == "default" else 18.0,
            })
# the 675 x 675 x 500 box (user, 2026-08-27): the bwb at this box failed
# EVERY rung with "the compartment pieces would not join" / "left the
# compartment SOLID" while the same planform at 650 carved at rung 4 - the
# core/below coincident-floor fuse failure fixed by hatch._CORE_FLOOR_LIFT_MM
# (task 2, 2026-08-28). Kept in the sweep so it cannot come back.
BOX_675 = {"length_mm": 675, "width_mm": 675, "height_mm": 500}
for vstab in (None, "center_fin"):
    CONFIGS.append({"name": f"fw-bwb-{vstab or 'defaultfin'}-user675",
                    "airplane_type": "flying_wing", "planform": "bwb",
                    "vstab": vstab, "box": BOX_675, "mission": "fpv_cruiser",
                    "cruise": 18.0})
for tag, box, mission in (("default", BOX_DEFAULT, "sport"),
                          ("user650", BOX_USER, "fpv_cruiser")):
    CONFIGS.append({"name": f"delta-{tag}", "airplane_type": "delta",
                    "planform": None, "vstab": None, "box": box,
                    "mission": mission, "cruise": 18.0})


def design_for(cfg):
    body = {"airplane_type": cfg["airplane_type"], "mission": cfg["mission"],
            "cruise_speed_ms": cfg["cruise"], "payload_g": 0.0,
            "box": cfg["box"], "n_motors": 1, "build_method": "3d_printed",
            "material": "lw_pla",
            "mount_screws": 4, "mount_screw_d_mm": 3.2,
            "mount_bolt_circle_mm": 16.0}
    if cfg["planform"]:
        body["planform"] = cfg["planform"]
    if cfg["vstab"]:
        body["vstab"] = cfg["vstab"]
    req = GenerateRequest(**body)
    if cfg["airplane_type"] == "delta":
        from backend.physics import variants
        vs = variants.generate_variants(req.to_optimizer_input())
        return vs[0]["design"] if isinstance(vs[0], dict) and "design" in vs[0] \
            else vs[0]
    return optimize_design(req.to_optimizer_input())


def probe(cfg):
    t0 = time.time()
    d = design_for(cfg)
    g = d["geometry"]
    wing = geo._wing_from_design(g)
    body = g.get("body") or {}
    airframe = geo._blended_airframe(wing)
    wall = max(float(g.get("wall_mm", 1.2)), 0.6)
    boss, _cutters, x_nac = geo._motor_mount(wing, g.get("motor_mount") or {},
                                             wall)
    if boss is not None:
        try:
            merged = geo._heal(airframe.fuse(boss))
            if merged.isValid() and len(merged.Solids()) == 1:
                airframe = merged
            else:
                boss, x_nac = None, 0.0
        except Exception:
            boss, x_nac = None, 0.0
    mm = g.get("motor_mount") or {}
    pusher = boss is not None and str(mm.get("type", "")) == "pusher"
    x_fin_limit = (x_nac - 4.0) if pusher else None
    req = geo.bay_request(g, wing, body, wall, boss, x_nac, x_fin_limit)
    res = hatch.build_bay(wing, airframe=airframe, magnets=True,
                          canopy=False, one_piece=False, **req)
    bm = dict(res.bay_mm or {})
    out = {
        "name": cfg["name"], "planform": d.get("planform"),
        "vstab": (g.get("vstab") or {}).get("type"),
        "span_mm": round(2 * wing.half, 0),
        "ok": bool(res.ok),
        "rung": bm.get("rung"),
        "hatch_x1_mm": bm.get("hatch_x1_mm"),
        "x1_mm": bm.get("x1_mm"),
        "extended_mm": bm.get("cavity_extended_mm"),
        "stations": len(bm.get("cavity_stations_mm") or []),
        "band_end": (bm.get("cavity_stations_mm") or [None])[-1],
        "request": {k: (round(v, 1) if isinstance(v, float) else v)
                    for k, v in req.items() if k != "cavity_guard"},
        "reason": bm.get("reason"), "tried": bm.get("tried"),
        "secs": round(time.time() - t0),
    }
    return out


def main():
    filt = sys.argv[1] if len(sys.argv) > 1 else ""
    for cfg in CONFIGS:
        if filt and filt not in cfg["name"]:
            continue
        try:
            out = probe(cfg)
        except Exception as exc:
            out = {"name": cfg["name"], "ok": False,
                   "reason": f"{type(exc).__name__}: {exc}"}
        verdict = ("EXT" if out.get("ok") and (out.get("extended_mm") or 0) >= 12
                   else ("BAY-ONLY" if out.get("ok") else "NO-BAY"))
        print(f"RESULT {verdict:8s} {json.dumps(out, default=str)}",
              flush=True)


if __name__ == "__main__":
    main()
