"""Validity sweep across the configuration axes the UI exposes
(work plan task 2, 2026-08-28).

Every type x every value of every axis it legally offers
(`config_defs.AIRPLANE_TYPES[...]["axes"]`, plus the flying wing's own
planform / vstab / mission choices), one axis varied at a time with the
others at the type's default, and the three UI boxes for the two main
types. Not every combination - the point is that each choice the sidebar
offers produces a buildable aircraft, and that a choice which cannot is
refused with a reason.

Per configuration, the CHEAPEST path that proves each property:

* physics    - the API's own `generate_variants` request returns a primary
               design, it is `feasible`, and its recorded totals fit the box;
* bay        - the builder's probe seam (`tools_probe_fin_intrusion.build_pieces`:
               loft + boss + `hatch.build_bay` + tail fused) carved the
               compartment (`cavity` present, rung recorded) and, on the
               tailless types that extend it, the cavity continues past the
               hatch (`cavity_extended_mm` >= 12);
* fins       - every tail surface the builder lofted is present in the
               built airframe: points along its mid-chord classify as
               MATERIAL in the airframe (a fuse that "succeeded" without its
               operand is the biplane failure mode);
* intrusion  - no fin material stands inside the cavity
               (`tools_probe_fin_intrusion.classify_surface`);
* full       - for the `--full` subset only: `build_design_solid` returns
               exactly one valid solid, its bbox stays inside the recorded
               totals + 2 mm, and no servo run fell back to a box gallery
               (`entry_mode` ending in `_gallery`).

Usage:
  .venv/Scripts/python.exe tools_probe_all.py [name-filter] [--full] [--list]
      [--from=<config name>]
Prints `RESULT PASS|FAIL <json>` per configuration and a summary table.
`--full` runs the full-build subset (FULL_BUILD names) instead of the
probe-level sweep. Run under `tools_cadlock.py` - one build at a time.
"""
from __future__ import annotations

import json
import os
import sys
import time
import traceback

os.environ.setdefault("AEROFORGE_HATCH_TRACE", "0")
os.environ.setdefault("AEROFORGE_CAD_WORKERS", "0")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from backend.physics.config_defs import AIRPLANE_TYPES      # noqa: E402
from backend.physics import variants as _variants           # noqa: E402
from backend.schemas import GenerateRequest                 # noqa: E402
import tools_probe_fin_intrusion as _fin                    # noqa: E402

BOXES = {
    "default": {"length_mm": 700, "width_mm": 1200, "height_mm": 300},
    "b650": {"length_mm": 650, "width_mm": 650, "height_mm": 500},
    "b675": {"length_mm": 675, "width_mm": 675, "height_mm": 500},
}
MISSIONS = ("sport", "fpv_cruiser", "thermal_floater", "park_flyer")
PLANFORMS = ("swept", "bwb", "plank", "bell")
VSTABS = ("winglets", "twin_fin", "center_fin")
CRUISE = {"flying_wing": 16.0, "conventional": 15.0, "delta": 18.0,
          "canard": 15.0, "tandem": 15.0, "twin_boom": 16.0}
# The UI's default box is the flying wing's; the other types' recommended
# minimum boxes (config_defs, task 5) are what their sidebar suggests.
def _type_box(t: str) -> dict:
    if t == "flying_wing":
        return BOXES["default"]
    rec = AIRPLANE_TYPES[t].get("recommended_min_box_mm")
    if rec:
        return {"length_mm": max(rec[0], 700), "width_mm": max(rec[1], 1200),
                "height_mm": max(rec[2], 300)}
    return BOXES["default"]


CONFIGS: list[dict] = []


def _add(name: str, t: str, **kw) -> None:
    cfg = {"name": name, "airplane_type": t, "mission": "sport",
           "cruise": CRUISE[t], "box": _type_box(t), "box_tag": "typedef"}
    cfg.update(kw)
    if any(c["name"] == name for c in CONFIGS):
        return
    CONFIGS.append(cfg)


for t, spec in AIRPLANE_TYPES.items():
    _add(f"{t}-default", t)
    ax = spec["axes"]
    for v in ax.get("motor_layout") or []:
        if len(ax["motor_layout"]) > 1:
            _add(f"{t}-motor_{v}", t, motor_layout=v)
    lo, hi = ax.get("n_motors") or [1, 1]
    for n in range(int(lo), int(hi) + 1):
        if hi > lo:
            _add(f"{t}-motors_{n}", t, n_motors=n)
    for v in ax.get("tail_type") or []:
        if len(ax["tail_type"]) > 1:
            _add(f"{t}-tail_{v}", t, tail_type=v)
    for v in ax.get("wing_position") or []:
        if len(ax["wing_position"]) > 1:
            _add(f"{t}-wing_{v}", t, wing_position=v)
    if t == "flying_wing":
        for p in PLANFORMS:
            _add(f"{t}-planform_{p}", t, planform=p)
        for v in VSTABS:
            _add(f"{t}-vstab_{v}", t, vstab=v)
    if t in ("flying_wing", "conventional"):
        for m in MISSIONS:
            _add(f"{t}-mission_{m}", t, mission=m)
        for tag, box in BOXES.items():
            _add(f"{t}-box_{tag}", t, box=box, box_tag=tag,
                 mission="fpv_cruiser" if tag != "default" else "sport",
                 cruise=18.0 if tag != "default" else CRUISE[t])
# the bug that opened this sweep: bwb + centre fin at the 675 box
_add("flying_wing-bwb-center_fin-b675", "flying_wing", planform="bwb",
     vstab="center_fin", box=BOXES["b675"], box_tag="b675",
     mission="fpv_cruiser", cruise=18.0)
_add("flying_wing-bwb-winglets-b675", "flying_wing", planform="bwb",
     box=BOXES["b675"], box_tag="b675", mission="fpv_cruiser", cruise=18.0)

# full-build subset, spread across the types
# (ordered by what each one proves end to end: the bwb-675 bay fix through
# the servo runs, the conventional fin fix through the bbox gate, then one
# of each remaining family; 15-35 min each, so the list is short)
FULL_BUILD = ("flying_wing-bwb-center_fin-b675", "conventional-default",
              "twin_boom-default", "delta-default", "canard-default",
              "tandem-default", "flying_wing-default",
              "conventional-tail_v_tail")


def request_for(cfg: dict) -> GenerateRequest:
    body = {"airplane_type": cfg["airplane_type"], "mission": cfg["mission"],
            "cruise_speed_ms": cfg["cruise"], "payload_g": 0.0,
            "box": cfg["box"], "n_motors": cfg.get("n_motors", 1),
            "build_method": "3d_printed", "material": "lw_pla",
            "mount_screws": 4, "mount_screw_d_mm": 3.2,
            "mount_bolt_circle_mm": 16.0}
    for k in ("planform", "vstab", "motor_layout", "tail_type",
              "wing_position"):
        if cfg.get(k):
            body[k] = cfg[k]
    return GenerateRequest(**body)


def design_for(cfg: dict) -> tuple[dict, list[dict]]:
    """The API's own path: `generate_variants`, primary first."""
    req = request_for(cfg)
    vs = _variants.generate_variants(req.to_optimizer_input())
    primary = next((v for v in vs if v.get("primary")), vs[0])
    d = primary["design"] if "design" in primary else primary
    if cfg.get("tail_type") and cfg["airplane_type"] == "conventional":
        # v3.1 gap (ARCHITECTURE.md): stamp the tail geometry the way the CAD
        # suites and tools_probe_fin_intrusion do
        d["geometry"].setdefault("tail", {})["type"] = cfg["tail_type"]
        d.setdefault("config", {})["tail_type"] = cfg["tail_type"]
    return d, vs


def physics_check(d: dict, cfg: dict) -> dict:
    g = d.get("geometry") or {}
    box = cfg["box"]
    span = float(g.get("span_m", 0.0)) * 1000.0
    length = float(g.get("length_total_m", 0.0)) * 1000.0
    height = float(g.get("height_total_m", 0.0)) * 1000.0
    fits = (span <= box["width_mm"] + 0.5 and length <= box["length_mm"] + 0.5
            and height <= box["height_mm"] + 0.5)
    bad = [c.get("message") or c.get("name") for c in (d.get("constraints") or [])
           if isinstance(c, dict) and not c.get("ok", True)]
    return {"feasible": bool(d.get("feasible")), "fits_box": fits,
            "span_mm": round(span, 1), "length_mm": round(length, 1),
            "height_mm": round(height, 1), "binding": bad[:3]}


def fins_present(pr: dict) -> dict:
    """Points along each raw fin's mid-chord must be MATERIAL in the built
    airframe (fused), sampled where the raw fin itself classifies IN."""
    out = {}
    aw = pr["airframe"].wrapped
    cav = pr["cavity"].wrapped if pr.get("cavity") is not None else None
    for nm, fin in (pr.get("fins") or {}).items():
        x0, x1, y0, y1, z0, z1 = _fin._bb(fin)
        fw = fin.wrapped
        tried = hit = trimmed = 0
        for fz in (0.25, 0.45, 0.65, 0.85):
            for fx in (0.35, 0.5, 0.65):
                x = x0 + fx * (x1 - x0)
                y = 0.5 * (y0 + y1)
                z = z0 + fz * (z1 - z0)
                # a swept blade: walk y across the fin's box until inside it
                for fy in (0.5, 0.35, 0.65, 0.2, 0.8):
                    yy = y0 + fy * (y1 - y0)
                    if _fin._inside(fw, x, yy, z):
                        y = yy
                        break
                else:
                    continue
                if cav is not None and (_fin._inside(cav, x, y, z)
                                        or _fin._inside(cav, x, y, z - 2.0)):
                    # the raw fin's buried root crossing the compartment
                    # (or the 1.2 mm bury band just above its roof): air
                    # there is the root TRIM working (task 1), not a
                    # missing fin - the intrusion check owns that region
                    trimmed += 1
                    continue
                tried += 1
                if _fin._inside(aw, x, y, z):
                    hit += 1
        out[nm] = {"sampled": tried, "in_airframe": hit,
                   "in_cavity_skipped": trimmed,
                   "fused": tried > 0 and hit >= max(1, int(0.8 * tried))}
    return out


def probe(cfg: dict, full: bool = False) -> dict:
    t0 = time.time()
    res: dict = {"name": cfg["name"], "airplane_type": cfg["airplane_type"],
                 "axis": {k: cfg[k] for k in ("planform", "vstab", "mission",
                                              "motor_layout", "n_motors",
                                              "tail_type", "wing_position",
                                              "box_tag") if k in cfg},
                 "checks": {}, "fail": []}
    d, _vs = design_for(cfg)
    res["design_id"] = d.get("id")
    res["planform"] = d.get("planform")
    ph = physics_check(d, cfg)
    res["checks"]["physics"] = ph
    if not ph["feasible"]:
        res["fail"].append("physics: infeasible - " + "; ".join(
            str(b) for b in ph["binding"]))
    if not ph["fits_box"]:
        res["fail"].append("physics: totals exceed the box")
    t1 = time.time()

    pr = _fin.build_pieces(d)
    bm = pr.get("bay_mm") or {}
    tailless_ext = cfg["airplane_type"] in ("flying_wing", "delta")
    bay = {"ok": pr.get("cavity") is not None, "rung": bm.get("rung"),
           "reason": bm.get("reason"),
           "cavity_extended_mm": bm.get("cavity_extended_mm"),
           "core_floor_lifted_mm": bm.get("core_floor_lifted_mm"),
           "mesh_round_tripped": bm.get("mesh_round_tripped"),
           "floor_raised_mm": bm.get("floor_raised_mm")}
    res["checks"]["bay"] = bay
    if not bay["ok"]:
        res["fail"].append(f"bay: not carved - {bay['reason']}")
    elif tailless_ext and (bay["cavity_extended_mm"] or 0) < 12:
        res["fail"].append("bay: cavity extension under 12 mm")
    fins = fins_present(pr)
    res["checks"]["fins"] = fins
    for nm, f in fins.items():
        if not f["fused"]:
            res["fail"].append(f"fins: {nm} not found in the built airframe")
    warn = [w for w in (pr.get("warnings") or [])
            if "fuse" in w.lower() or "fin" in w.lower()]
    if warn:
        res["checks"]["warnings"] = warn
    intr = 0
    if pr.get("cavity") is not None:
        surfaces = [_fin.classify_surface(nm, fin, pr["cavity"],
                                          pr["airframe"], pr.get("airframe_pre"))
                    for nm, fin in (pr.get("fins") or {}).items()]
        intr = sum(s["intrusion_pts"] for s in surfaces)
        res["checks"]["intrusion"] = {
            "pts": intr,
            "per_surface": {s["surface"]: s["intrusion_pts"] for s in surfaces}}
        if intr:
            res["fail"].append(f"intrusion: {intr} fin points in the cavity")
    t2 = time.time()

    if full:
        from backend.cad import geometry as geo
        solid, meta = geo.build_design_solid(d)
        bb = solid.BoundingBox()
        g = d["geometry"]
        span = float(g["span_m"]) * 1000.0
        length = float(g.get("length_total_m", 0.0)) * 1000.0
        height = float(g.get("height_total_m", 0.0)) * 1000.0
        modes: dict[str, str] = {}

        def _walk(o, path=""):
            if isinstance(o, dict):
                if "entry_mode" in o:
                    modes[path] = str(o["entry_mode"])
                for k, v in o.items():
                    _walk(v, f"{path}/{k}" if path else str(k))
            elif isinstance(o, list):
                for i, v in enumerate(o):
                    _walk(v, f"{path}[{i}]")
        _walk(meta)
        galleries = {k: v for k, v in modes.items() if v.endswith("_gallery")
                     or v == "gallery"}
        fb = {"solids": len(solid.Solids()), "valid": bool(solid.isValid()),
              "bbox_mm": [round(v, 1) for v in (bb.xmin, bb.xmax, bb.ymin,
                                                 bb.ymax, bb.zmin, bb.zmax)],
              "totals_mm": [round(length, 1), round(span, 1), round(height, 1)],
              "entry_modes": modes, "galleries": galleries,
              "warnings": list(meta.get("warnings") or [])[:6]}
        res["checks"]["full"] = fb
        if fb["solids"] != 1 or not fb["valid"]:
            res["fail"].append(f"full: {fb['solids']} solids, valid={fb['valid']}")
        if bb.xmin < -2.0 or bb.xmax > length + 2.0 or bb.ylen > span + 2.0 \
                or bb.zlen > height + 2.0:
            res["fail"].append("full: bbox exceeds the recorded totals + 2 mm")
        if galleries:
            res["fail"].append("full: servo run fell back to a gallery: "
                               + ", ".join(galleries))
    res["ok"] = not res["fail"]
    res["secs"] = {"design": round(t1 - t0), "probe": round(t2 - t1),
                   "full": round(time.time() - t2) if full else None}
    return res


def main() -> None:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    flags = {a for a in sys.argv[1:] if a.startswith("--")}
    filt = args[0] if args else ""
    full = "--full" in flags
    start = next((f[len("--from="):] for f in flags
                  if f.startswith("--from=")), None)
    names = FULL_BUILD if full else tuple(c["name"] for c in CONFIGS)
    todo = [c for c in CONFIGS if c["name"] in names
            and (not filt or filt in c["name"])]
    if start:
        # resume a sweep from a named configuration (inclusive)
        idx = next((i for i, c in enumerate(todo) if c["name"] == start), 0)
        todo = todo[idx:]
    if "--list" in flags:
        for c in todo:
            print(c["name"])
        return
    rows = []
    for cfg in todo:
        try:
            out = probe(cfg, full=full)
        except Exception as exc:
            out = {"name": cfg["name"], "ok": False,
                   "fail": [f"{type(exc).__name__}: {exc}"],
                   "trace": traceback.format_exc().splitlines()[-3:]}
        rows.append(out)
        print(f"RESULT {'PASS' if out.get('ok') else 'FAIL':4s} "
              f"{json.dumps(out, default=str)}", flush=True)
    print("\nSUMMARY")
    print(f"{'config':40s} {'verdict':7s} {'bay':22s} {'ext':>6s} {'fins':>5s} "
          f"{'intr':>4s} {'secs':>5s}  notes")
    for r in rows:
        ck = r.get("checks") or {}
        bay = ck.get("bay") or {}
        fins = ck.get("fins") or {}
        rung = (bay.get("rung") or bay.get("reason") or "-")
        ext = bay.get("cavity_extended_mm")
        secs = r.get("secs") or {}
        print(f"{r['name']:40s} {'PASS' if r.get('ok') else 'FAIL':7s} "
              f"{str(rung)[:22]:22s} {str(ext if ext is not None else '-'):>6s} "
              f"{str(len(fins)):>5s} "
              f"{str((ck.get('intrusion') or {}).get('pts', '-')):>4s} "
              f"{str(sum(v for v in secs.values() if v)):>5s}  "
              f"{'; '.join(r.get('fail') or [])[:90]}")
    n_ok = sum(1 for r in rows if r.get("ok"))
    print(f"\n{n_ok}/{len(rows)} PASS")


if __name__ == "__main__":
    main()
