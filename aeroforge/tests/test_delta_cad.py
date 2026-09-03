"""DELTA verification (v3): delta dicts ride the EXISTING flying-wing CAD
unmodified (V3_PLAN.md). Two variants - `delta_sport` (the 50-degree
reference character) and `delta_dart` (58-degree, thin, hot) - are generated
through `physics.delta.generate_delta_variants` and built through
`cad.geometry.build_design_parts`/`build_design_solid`, then held to the
standard tailless gates: one watertight solid, mesh >= 98.5%, nothing ahead
of the datum, bbox inside the recorded envelope, bay hollow-verified, elevons
hinged on captive pins, horn bores proven, servo pipes starting at the lead
grommet, and the belly-entry motor wiring applied.

Budget note: same opt-in `AEROFORGE_TEST_CAD_CACHE` BREP cache as
tests/test_twinboom.py (builds are ~6-7 minutes each; this environment kills
shells at ten). Unset ⇒ fresh builds (CI mode).
"""
import hashlib
import json
import math
import os
from pathlib import Path

import numpy as np
import pytest
from cadquery import Shape, Solid, Vector
from stl import mesh as stl_mesh

from backend.cad.exporters import (
    stl_is_valid_mesh, stl_watertight_fraction, write_stl_verified,
)
from backend.cad.geometry import (
    _elevon_hinge_line, _TIP_START, _wing_from_design, build_design_parts,
    build_design_solid,
)
from backend.physics.delta import generate_delta_variants

BOX = {"box_l": 0.95, "box_w": 1.2, "box_h": 0.3}


def _inp(**over):
    body = dict(mission="sport", v_cruise=15.0, v_stall_target=None,
                payload_kg=0.0, vstab="auto", n_motors=1, material="lw_pla",
                build_method="3d_printed", airfoil_override=None,
                ar_target=None, sm_override=None, **BOX)
    body.update(over)
    return body


# ---------------------------------------------------------------------------
# BREP cache plumbing (same contract as tests/test_twinboom.py)
# ---------------------------------------------------------------------------

def _cache_root():
    root = os.environ.get("AEROFORGE_TEST_CAD_CACHE")
    return Path(root) if root else None


def _key(name: str, design: dict) -> str:
    stable = {k: v for k, v in design.items() if k != "id"}  # id is a uuid
    blob = json.dumps(stable, sort_keys=True, default=str).encode()
    return f"{name}-{hashlib.sha1(blob).hexdigest()[:10]}"


def _load_brep(path: Path) -> Solid:
    from OCP.BRep import BRep_Builder
    from OCP.BRepTools import BRepTools
    from OCP.TopoDS import TopoDS_Shape
    sh = TopoDS_Shape()
    BRepTools.Read_s(sh, str(path), BRep_Builder())
    shape = Shape.cast(sh)
    return shape if isinstance(shape, Solid) else shape.Solids()[0]


def _save_brep(solid: Solid, path: Path) -> None:
    from OCP.BRepTools import BRepTools
    BRepTools.Write_s(solid.wrapped, str(path))


def _cached_parts(name: str, design: dict, builder):
    root = _cache_root()
    if root is None:
        return builder(design)
    d = root / _key(name, design)
    meta_p = d / "meta.json"
    if meta_p.exists():
        meta = json.loads(meta_p.read_text())
        parts = {p.stem: _load_brep(p) for p in d.glob("*.brep")}
        return parts, meta
    parts, meta = builder(design)
    d.mkdir(parents=True, exist_ok=True)
    for n, s in parts.items():
        _save_brep(s, d / f"{n}.brep")
    meta_p.write_text(json.dumps(meta, default=str))
    return parts, meta


def _cached_solid(name: str, design: dict, builder):
    root = _cache_root()
    if root is None:
        return builder(design)
    d = root / _key(name, design)
    meta_p = d / "meta.json"
    if meta_p.exists():
        return _load_brep(d / "solid.brep"), json.loads(meta_p.read_text())
    solid, meta = builder(design)
    d.mkdir(parents=True, exist_ok=True)
    _save_brep(solid, d / "solid.brep")
    meta_p.write_text(json.dumps(meta, default=str))
    return solid, meta


def _classify(solid, p):
    from OCP.BRepClass3d import BRepClass3d_SolidClassifier
    from OCP.gp import gp_Pnt
    from OCP.TopAbs import TopAbs_IN, TopAbs_ON
    c = BRepClass3d_SolidClassifier(solid.wrapped, gp_Pnt(p.x, p.y, p.z),
                                    1e-7)
    return c.State() in (TopAbs_IN, TopAbs_ON)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def variants():
    vs = generate_delta_variants(_inp())
    assert len(vs) == 5
    return {v["key"]: v["design"] for v in vs}


@pytest.fixture(scope="module")
def sport(variants):
    d = variants["delta_sport"]
    assert d["airplane_type"] == "delta" and d["planform"] == "delta"
    return d


@pytest.fixture(scope="module")
def dart(variants):
    d = variants["delta_dart"]
    assert d["airplane_type"] == "delta" and d["planform"] == "delta"
    return d


@pytest.fixture(scope="module")
def sport_parts(sport):
    return _cached_parts("delta_sport_parts", sport, build_design_parts)


@pytest.fixture(scope="module")
def dart_parts(dart):
    return _cached_parts("delta_dart_parts", dart, build_design_parts)


@pytest.fixture(scope="module")
def sport_solid(sport):
    return _cached_solid("delta_sport_solid", sport, build_design_solid)


@pytest.fixture(scope="module")
def dart_solid(dart):
    return _cached_solid("delta_dart_solid", dart, build_design_solid)


# ---------------------------------------------------------------------------
# Physics-side sanity (cheap)
# ---------------------------------------------------------------------------

def test_deltas_look_like_deltas(sport, dart):
    for d in (sport, dart):
        g = d["geometry"]
        assert 45.0 <= g["sweep_le_deg"] <= 60.0
        assert g["aspect_ratio"] <= 3.6
        assert g["root_chord_m"] / g["span_m"] >= 0.40
        assert (g.get("vstab") or {}).get("type") in ("center_fin",
                                                      "twin_fin")
        assert g["span_m"] <= BOX["box_w"] + 1e-6
        assert g["length_total_m"] <= BOX["box_l"] + 1e-6
        assert g["height_total_m"] <= BOX["box_h"] + 1e-6
    assert dart["geometry"]["sweep_le_deg"] > sport["geometry"][
        "sweep_le_deg"]


# ---------------------------------------------------------------------------
# The standard tailless gates, per variant
# ---------------------------------------------------------------------------

def _check_parts(parts, meta):
    body_names = set(parts)
    assert ("airframe" in body_names
            or {"centre_body", "wing_left", "wing_right"} <= body_names), (
        sorted(parts))
    for need in ("elevon_left", "elevon_right", "hatch_lid", "cg_marker"):
        assert need in parts, f"no {need}, got {sorted(parts)}"
    for name, solid in parts.items():
        assert solid.isValid(), name
        assert solid.Volume() > 0, name


def _airframe_of(parts):
    if "airframe" in parts:
        return [parts["airframe"]]
    return [parts[k] for k in ("centre_body", "wing_left", "wing_right")]


def _check_bay(parts, meta):
    bay = meta.get("bay") or {}
    assert bay, "no equipment bay was cut"
    zf, zc = bay.get("z_floor_aft_mm"), bay.get("z_ceil_aft_mm")
    assert zf is not None and zc is not None and zc - zf >= 6.0
    p = Vector(0.5 * (bay["x0_mm"] + bay["x1_mm"]), 0.0, 0.5 * (zf + zc))
    assert not any(_classify(s, p) for s in _airframe_of(parts)), (
        "bay probe is still material")


def _check_hinges_and_horns(design, parts, meta):
    reports = {k: v for k, v in (meta.get("hinges") or {}).items()
               if k.startswith("elevon")}
    assert reports, "no elevon hinge reports"
    wing = _wing_from_design(design["geometry"])
    el = design["geometry"]["elevons"]
    xc = max(0.45, min(0.90, 1.0 - float(el["chord_frac"])))
    inner = max(wing.fb, 0.10, float(el["inner_frac"]))
    outer = min(float(el["outer_frac"]), _TIP_START - 0.01)
    pinned = 0
    for name, info in reports.items():
        if info.get("mode") != "print_in_place":
            continue
        pinned += 1
        surface = parts[name]
        others = [s for n, s in parts.items()
                  if n != name and not n.startswith("cg_")]
        sgn = 1.0 if name.endswith("right") else -1.0
        p_in, p_out, _ti, _to = _elevon_hinge_line(wing, sgn, inner, outer,
                                                   xc)
        u = p_out - p_in
        u = u.multiply(1.0 / u.Length)
        for st in info["stations"]:
            base = p_in + u.multiply(st["station_mm"])
            assert any(_classify(s, base) for s in others), (
                f"{name} @ {st['station_mm']:.0f}: nothing on the axis")
            assert not _classify(surface, base), (
                f"{name} @ {st['station_mm']:.0f}: barrel bore solid")
    assert pinned >= 1, {k: v.get("mode") for k, v in reports.items()}

    horns = {k: v for k, v in ((meta.get("servos") or {}).get("horns")
                               or {}).items() if v.get("ok") is not False}
    assert horns, "no horns fused"
    for name, h in horns.items():
        assert len(h.get("holes") or []) == 1, name
        assert h.get("holes_cut") == [True], (
            f"{name}: bore not proven ({h.get('holes_cut')})")


def _check_wiring(design, meta):
    sv = meta.get("servos") or {}
    bays = {k: v for k, v in (sv.get("bays") or {}).items() if v.get("ok")}
    con = sv.get("conduits") or {}
    from backend.cad.servos import SERVO_CLEARANCE_MM, SERVO_SG90
    x_off = (0.5 * float(SERVO_SG90["body_len_mm"]) + SERVO_CLEARANCE_MM
             + 0.5 * float(SERVO_SG90["lead_stub_mm"]))
    # The pipe's pocket cap drifts AFT of the grommet with sweep - the route
    # holds a chord fraction over the start overshoot, "in line with the
    # wire, which leaves the grommet chordwise" (DECISIONS.md, measured
    # 1.11 mm at 26 deg). The fw suite's 2.5 mm bound was calibrated at
    # <= 26 deg; a 45-60 deg delta drifts further along the same line, so
    # the AFT allowance scales with tan(sweep) while the forward bound
    # stays hard - the old fold-back mouth (5 mm FORWARD) must keep failing.
    sweep = math.radians(float(design["geometry"]["sweep_le_deg"]))
    tol_aft = 2.5 + 6.0 * math.tan(sweep)
    assert bays, "no servo bay fitted on a delta reference design"
    for name, bay in bays.items():
        ci = con.get(f"servo_{name}") or {}
        assert ci.get("applied"), (
            f"servo_{name}: pipe not applied "
            f"({ci.get('skipped') or ci.get('why') or 'missing'})")
        path = ci.get("path_mm") or []
        if path and "x_centre" in bay:
            x_expect = float(bay["x_centre"]) + x_off
            drift = path[0][0] - x_expect
            assert -2.5 < drift < tol_aft, (
                f"servo_{name}: mouth x={path[0][0]:.2f}, grommet "
                f"x={x_expect:.2f} (drift {drift:+.2f}, allowed -2.5.."
                f"{tol_aft:.1f})")
    m = con.get("motor") or {}
    assert m.get("ok") is not False, m
    # builder's spec, round 5: ONE straight pipe whose belly breach IS the
    # entry - the separate vertical entry bore is gone
    assert m.get("straight") is True, "motor run must be a straight pipe"
    assert "motor_entry" not in con, (
        "the separate vertical entry bore is back - the motor wiring must "
        "be one straight pipe")
    assert (con.get("motor_run") or {}).get("applied"), (
        "straight motor run not applied")


def _check_solid(design, solid, meta, tmp_path, tag):
    assert solid.isValid()
    assert len(solid.Solids()) == 1
    g = design["geometry"]
    stl = write_stl_verified(solid, tmp_path / f"{tag}.stl")
    m = stl_mesh.Mesh.from_file(str(stl))
    v = m.vectors
    assert float(v[:, :, 0].min()) >= -0.25
    assert float(v[:, :, 0].max()) <= g["length_total_m"] * 1000 + 2.0
    assert float(np.ptp(v[:, :, 1])) <= g["span_m"] * 1000 + 2.0
    assert float(np.ptp(v[:, :, 2])) <= g["height_total_m"] * 1000 + 2.0
    assert float(m.areas.sum()) / solid.Area() >= 0.985
    ok, msg = stl_is_valid_mesh(stl)
    assert ok, msg
    assert stl_watertight_fraction(stl) >= 0.98


def test_sport_parts_present_and_valid(sport_parts):
    _check_parts(*sport_parts)


def test_sport_bay_hollow(sport_parts):
    _check_bay(*sport_parts)


def test_sport_elevons_hinged_horns_proven(sport, sport_parts):
    parts, meta = sport_parts
    _check_hinges_and_horns(sport, parts, meta)


def test_sport_wiring(sport, sport_parts):
    _check_wiring(sport, sport_parts[1])


def test_sport_one_solid_and_mesh(sport, sport_solid, tmp_path):
    solid, meta = sport_solid
    _check_solid(sport, solid, meta, tmp_path, "delta_sport")


def test_dart_parts_present_and_valid(dart_parts):
    _check_parts(*dart_parts)


def test_dart_bay_hollow(dart_parts):
    _check_bay(*dart_parts)


def test_dart_elevons_hinged_horns_proven(dart, dart_parts):
    parts, meta = dart_parts
    _check_hinges_and_horns(dart, parts, meta)


def test_dart_wiring(dart, dart_parts):
    _check_wiring(dart, dart_parts[1])


def test_dart_one_solid_and_mesh(dart, dart_solid, tmp_path):
    solid, meta = dart_solid
    _check_solid(dart, solid, meta, tmp_path, "delta_dart")
