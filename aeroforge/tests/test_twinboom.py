"""CAD build + export invariants for the TWIN-BOOM airplane path (v3).

The v2 conventional bar (tests/test_conventional.py) applied to
`airplane_type="twin_boom"` - one valid watertight solid, nothing ahead of
the nose datum, envelope honesty, mesh coverage, a real hollow bay with a
separable lid, print-in-place control surfaces, proven horn bores, servo
pipes starting at the lead grommet - PLUS the type's own items: boom socket
bores proven open full length, the pusher firewall drilled and proven, motor
leads running INSIDE the pod (no belly hole), and the prop-disc clearance
asserted on the built stations.

Budget note: a twin-boom CAD build takes ~9 minutes. Everything expensive is
module-scoped, and because this environment kills any shell command at ten
minutes, the fixtures support an opt-in cross-process cache: set
`AEROFORGE_TEST_CAD_CACHE=<dir>` to store/reuse built solids as BREP + a
meta JSON, keyed on a hash of the design dict (so a stale cache can never
mask a physics change). Unset, every run builds fresh - the CI mode.
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
    _elevon_hinge_line, _TIP_START, build_design_parts, build_design_solid,
)
from backend.physics.twinboom import optimize_twinboom

BOX = {"box_l": 1.3, "box_w": 1.8, "box_h": 0.4}


def _inp(**over):
    body = dict(mission="fpv_cruiser", v_cruise=16.0, v_stall_target=None,
                payload_kg=0.0, vstab="auto", n_motors=1, material="lw_pla",
                build_method="3d_printed", airfoil_override=None,
                ar_target=None, sm_override=None, **BOX)
    body.update(over)
    return body


# ---------------------------------------------------------------------------
# BREP cache plumbing (opt-in; see module docstring)
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


def _get_design() -> dict:
    return optimize_twinboom(_inp())


@pytest.fixture(scope="module")
def design():
    d = _get_design()
    assert d["airplane_type"] == "twin_boom"
    return d


@pytest.fixture(scope="module")
def built_parts(design):
    return _cached_parts("tb_parts", design, build_design_parts)


@pytest.fixture(scope="module")
def built(design):
    return _cached_solid("tb_solid", design, build_design_solid)


def _classify(solid, p):
    """True if point `p` is inside (or on) the material."""
    from OCP.BRepClass3d import BRepClass3d_SolidClassifier
    from OCP.gp import gp_Pnt
    from OCP.TopAbs import TopAbs_IN, TopAbs_ON
    c = BRepClass3d_SolidClassifier(solid.wrapped, gp_Pnt(p.x, p.y, p.z),
                                    1e-7)
    return c.State() in (TopAbs_IN, TopAbs_ON)


# ---------------------------------------------------------------------------
# Physics-side envelope honesty (cheap - no CAD)
# ---------------------------------------------------------------------------

def test_recorded_envelope_fits_the_user_box(design):
    g = design["geometry"]
    assert g["span_m"] <= BOX["box_w"] + 1e-6
    assert g["length_total_m"] <= BOX["box_l"] + 1e-6
    assert g["height_total_m"] <= BOX["box_h"] + 1e-6


def test_prop_clearance_guaranteed_and_asserted(design, built_parts):
    """Physics guarantees spacing >= 1.15 x prop D; the CAD asserts it again
    on the BUILT stations, fairing width included."""
    b = design["geometry"]["booms"]
    assert b["spacing_m"] >= 1.15 * b["prop_diameter_est_m"] - 1e-9
    assert b["spacing_m"] >= b["prop_diameter_est_m"] + 0.030 - 1e-9
    _parts, meta = built_parts
    bm = meta["booms_mm"]
    assert bm["prop_clearance_ok"], bm
    assert bm["inner_face_mm"] >= bm["prop_tip_mm"] + 2.0 - 1e-6


# ---------------------------------------------------------------------------
# Parts
# ---------------------------------------------------------------------------

def test_expected_parts_are_present_named_and_valid(built_parts):
    parts, meta = built_parts
    for need in ("airframe", "aileron_left", "aileron_right",
                 "elevator_left", "elevator_right",
                 "rudder_left", "rudder_right", "hatch_lid", "cg_marker"):
        assert need in parts, f"no {need}, got {sorted(parts)}"
    for name, solid in parts.items():
        assert solid.isValid(), name
        assert solid.Volume() > 0, name
    # assembled in place: no part past the recorded length (mesh-measured)
    aft = max(max(v.x for v in p.tessellate(0.2)[0]) for p in parts.values())
    assert aft <= meta["length_mm"] + 2.0


def test_pod_bay_is_hollow_and_verified(built_parts):
    """The nose bay is the type's whole point - and because a boolean can
    fail without failing, the void is re-proven here by classification on
    the built part, not taken from the report alone."""
    parts, meta = built_parts
    bay = meta.get("bay") or {}
    assert bay, "no equipment bay was cut"
    assert bay["volume_cm3"] > 5.0
    assert bay["box_l_mm"] >= 25.0 and bay["box_w_mm"] >= 16.0 \
        and bay["box_h_mm"] >= 6.0
    zf, zc = bay.get("z_floor_aft_mm"), bay.get("z_ceil_aft_mm")
    assert zf is not None and zc is not None and zc - zf >= 6.0
    x_probe = 0.5 * (bay["x0_mm"] + bay["x1_mm"])
    p = Vector(x_probe, 0.0, 0.5 * (zf + zc))
    assert not _classify(parts["airframe"], p), (
        "probe is still material - the bay cut silently did nothing")
    # the nose bay must end ahead of the wing (the wing owns the pod deck)
    assert bay["x1_mm"] <= meta["fuselage_mm"]["x_wing_le"] - 10.0


def test_hatch_lid_is_a_separate_tray(built_parts):
    parts, meta = built_parts
    lid = parts.get("hatch_lid")
    assert lid is not None
    assert lid.isValid() and len(lid.Solids()) == 1
    bb = lid.BoundingBox()
    assert bb.xlen > 20.0 and bb.ylen > 15.0, "lid is a sliver"
    assert lid.Volume() < 0.35 * bb.xlen * bb.ylen * bb.zlen  # tray, no plug


# ---------------------------------------------------------------------------
# Control surfaces
# ---------------------------------------------------------------------------

def test_ailerons_hang_on_captive_printed_pins(design, built_parts):
    from backend.cad.twinboom import _make_hosts

    parts, meta = built_parts
    reports = {k: v for k, v in (meta.get("hinges") or {}).items()
               if k.startswith("aileron")}
    assert reports, "no aileron hinge reports at all"

    g = design["geometry"]
    _pod, wing, _stab, _dims = _make_hosts(g)
    ail = g["ailerons"]
    xc = max(0.45, min(0.90, 1.0 - float(ail["chord_frac"])))
    inner = max(wing.fb, 0.10, float(ail["inner_frac"]))
    outer = min(float(ail["outer_frac"]), _TIP_START - 0.01)

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
        e1 = Vector(-u.y, u.x, 0.0)
        e1 = e1.multiply(1.0 / e1.Length)
        e2 = u.cross(e1)
        e2 = e2.multiply(1.0 / e2.Length)
        assert info.get("pin_printed_in_place"), name
        assert info["n_hinges_built"] >= 2, name
        for st in info["stations"]:
            base = p_in + u.multiply(st["station_mm"])
            r_pin = 0.5 * st["pin_dia_mm"]
            r_barrel = 0.5 * st["knuckle_od_mm"]
            assert any(_classify(s, base) for s in others), (
                f"{name} @ {st['station_mm']:.0f}: nothing on the axis")
            assert not _classify(surface, base), (
                f"{name} @ {st['station_mm']:.0f}: barrel bore is solid")
            r_wall = 0.5 * (r_pin + r_barrel) + 0.15
            for k in range(8):
                a = 2.0 * math.pi * k / 8.0
                p = (base + e1.multiply(r_wall * math.cos(a))
                     + e2.multiply(r_wall * math.sin(a)))
                assert _classify(surface, p), (
                    f"{name} @ {st['station_mm']:.0f}: barrel open at "
                    f"{math.degrees(a):.0f} deg")
    assert pinned >= 1, {k: v.get("mode") for k, v in reports.items()}


def test_tail_surfaces_report_their_hinge_mode(built_parts):
    _parts, meta = built_parts
    hr = meta.get("hinges") or {}
    for name in ("elevator_left", "elevator_right",
                 "rudder_left", "rudder_right"):
        info = hr.get(name)
        assert isinstance(info, dict) and info.get("mode") in (
            "print_in_place", "bevel_only", "none"), (
            f"{name}: no hinge report ({info})")


def test_every_horn_bore_is_one_proven_25mm_hole(built_parts):
    _parts, meta = built_parts
    horns = {k: v for k, v in ((meta.get("servos") or {}).get("horns")
                               or {}).items() if v.get("ok") is not False}
    assert horns, "no horns were fused at all"
    assert any(k.startswith("aileron") for k in horns), sorted(horns)
    assert any(k.startswith("rudder") for k in horns), sorted(horns)
    for name, h in horns.items():
        holes = h.get("holes") or []
        assert len(holes) == 1, f"{name}: {len(holes)} holes, want 1"
        assert abs(h.get("hole_d_mm", 0) - 2.5) < 0.01, name
        assert not h.get("feed_d_mm"), f"{name}: keyhole feed lobe is back"
        assert h.get("holes_cut") == [True], (
            f"{name}: bore not verified open ({h.get('holes_cut')})")


def test_the_servo_arm_and_aileron_horn_share_a_plane(built_parts):
    _parts, meta = built_parts
    sv = meta.get("servos") or {}
    bays = {k: v for k, v in (sv.get("bays") or {}).items() if v.get("ok")}
    horns = {k: v for k, v in (sv.get("horns") or {}).items()
             if k.startswith("aileron") and v.get("ok") is not False}
    if not bays:
        pytest.skip("no aileron servo bay fitted this section")
    for name, h in horns.items():
        if name not in bays:
            continue
        resid = h.get("align_residual_mm")
        assert resid is not None, f"{name}: horn not aligned to the arm"
        assert resid <= 2.0, f"{name}: {resid:.1f} mm off the arm plane"


def test_ailerons_get_servo_bays(built_parts):
    _parts, meta = built_parts
    bays = (meta.get("servos") or {}).get("bays") or {}
    assert set(bays) >= {"aileron_left", "aileron_right"}, sorted(bays)
    for name, b in bays.items():
        assert b.get("ok"), f"{name}: {b.get('reason')}"


# ---------------------------------------------------------------------------
# Wire runs
# ---------------------------------------------------------------------------

def test_aileron_wire_chain_reaches_the_nose_bay(built_parts):
    """ONE straight pipe per lead (builder's spec, round 5): mouth AT the
    grommet, descending through the wing root and pod into the void. The
    'operation whose absence matters' doctrine - the run applied and
    classified open end to end, or it reports itself dead."""
    _parts, meta = built_parts
    sv = meta.get("servos") or {}
    bays = {k: v for k, v in (sv.get("bays") or {}).items() if v.get("ok")}
    if not bays:
        pytest.skip("no servo bay fitted")
    con = sv.get("conduits") or {}
    from backend.cad.servos import SERVO_CLEARANCE_MM, SERVO_SG90
    x_grommet_off = (0.5 * float(SERVO_SG90["body_len_mm"])
                     + SERVO_CLEARANCE_MM
                     + 0.5 * float(SERVO_SG90["lead_stub_mm"]))
    for name, bay in bays.items():
        ci = con.get(f"servo_{name}") or {}
        assert ci.get("applied"), (
            f"servo_{name}: wire chain not applied "
            f"({ci.get('why') or ci.get('reason') or 'missing'})")
        assert ci.get("riser_open"), f"servo_{name}: riser blocked"
        assert ci.get("run_open"), f"servo_{name}: pod run blocked"
        path = ci.get("path_mm") or []
        if path and "x_centre" in bay:
            x_expect = float(bay["x_centre"]) + x_grommet_off
            assert abs(path[0][0] - x_expect) < 2.5, (
                f"servo_{name}: pipe mouth at x={path[0][0]:.2f} but the "
                f"lead grommet is at x={x_expect:.2f}")


def test_tail_snake_guides_are_cut_and_open(built_parts):
    """Elevator/rudder servos live in the pod bay; each gets a straight
    round 8.25 mm trumpeted guide out through the pod flank toward its boom
    - the snake continues externally along the boom (Skyhunter rigging)."""
    _parts, meta = built_parts
    pr = (meta.get("servos") or {}).get("pushrods") or {}
    for key in ("elevator", "rudder"):
        info = pr.get(key)
        assert isinstance(info, dict), f"no {key} snake guide at all"
        assert info.get("applied"), (
            f"{key} guide not applied: "
            f"{info.get('skipped') or info.get('why') or info.get('reason')}")
        assert info.get("route_open"), f"{key} guide bore is blocked"
        assert abs(info.get("d_mm", 0) - 8.25) < 0.01
        assert info.get("trumpeted") is True


def test_motor_leads_run_inside_the_pod_no_belly_hole(design, built_parts):
    """The physics dict says the leads run INSIDE the pod: one Ø8.25 pipe
    from behind the firewall into the bay void, proven open - and none of
    the flying-wing belly-entry machinery may appear on this type."""
    _parts, meta = built_parts
    con = (meta.get("servos") or {}).get("conduits") or {}
    m = con.get("motor")
    assert isinstance(m, dict), "no motor-lead pipe at all"
    assert m.get("applied"), m
    assert m.get("route_open"), "motor pipe reported cut but is blocked"
    assert abs(m.get("d_mm", 0) - 8.25) < 0.01
    # entry behind the firewall, not through the belly
    x_face = design["geometry"]["motor_mount"]["x_m"] * 1000.0
    assert m["entry_mm"][0] >= x_face - 1.0
    assert "motor_entry" not in con, "belly-entry hole on a twin-boom pod"


# ---------------------------------------------------------------------------
# Pusher mount + boom sockets
# ---------------------------------------------------------------------------

def test_pusher_mount_holes_are_drilled_and_proven(design, built_parts):
    parts, meta = built_parts
    mm = design["geometry"]["motor_mount"]
    assert mm["type"] == "pusher"
    cut = (meta.get("motor_mount") or {}).get("holes_cut")
    assert cut and all(cut), f"motor bores not verified open: {cut}"
    xf = mm["x_m"] * 1000.0
    yc, zc = mm["y_m"] * 1000.0, mm["z_m"] * 1000.0
    at_circle = 0
    for f in parts["airframe"].Faces():
        c = f.Center()
        if abs(c.x - xf) < 40:
            r = math.hypot(c.y - yc, c.z - zc)
            if abs(r - mm["bolt_circle_radius_mm"]) <= 0.5:
                at_circle += 1
    assert at_circle >= mm["n_screws"], (
        f"expected {mm['n_screws']} screw holes on the aft bolt circle, "
        f"found {at_circle} faces there")


def test_boom_sockets_are_bored_full_length_and_proven(design, built_parts):
    """The carbon tube IS the boom's stiffness; its socket must be open end
    to end. The build's own report is checked AND the bores are re-proven
    here by classification - a status code is never trusted."""
    parts, meta = built_parts
    sock = meta.get("sockets") or {}
    booms = sock.get("booms") or {}
    sect = design["geometry"]["booms"]["section_mm"]
    assert abs(sock.get("socket_id_mm", 0) - sect["socket_id_mm"]) < 1e-6
    for side in ("left", "right"):
        b = booms.get(side) or {}
        assert b.get("open"), f"{side} socket: {b}"
        assert b.get("stations_checked", 0) >= 7
        # independent re-proof on the shipped part, 5+ stations
        xs = np.linspace(b["x0_mm"] + 4.0, b["x1_mm"] - 4.0, 5)
        for x in xs:
            p = Vector(float(x), b["y_mm"], b["z_mm"])
            assert not _classify(parts["airframe"], p), (
                f"{side} socket solid at x={x:.0f} - the bore is not there")


# ---------------------------------------------------------------------------
# One-piece solid + export integrity
# ---------------------------------------------------------------------------

def test_onepiece_builds_exactly_one_valid_solid(built):
    solid, meta = built
    assert solid.isValid()
    assert len(solid.Solids()) == 1, (
        f"{len(solid.Solids())} disconnected solids - pod, wing, booms or "
        "tail did not fuse")
    assert meta["valid_solid"]
    assert solid.Volume() > 0


def test_onepiece_nothing_ahead_of_the_nose_datum(built, tmp_path):
    solid, _ = built
    stl = write_stl_verified(solid, tmp_path / "datum.stl")
    xmin = float(stl_mesh.Mesh.from_file(str(stl)).vectors[:, :, 0].min())
    assert xmin >= -0.25, f"geometry at x = {xmin:.3f} mm"


def test_onepiece_stays_inside_the_recorded_envelope(design, built,
                                                     tmp_path):
    solid, _ = built
    g = design["geometry"]
    v = stl_mesh.Mesh.from_file(
        str(write_stl_verified(solid, tmp_path / "env.stl"))).vectors
    assert float(v[:, :, 0].max()) <= g["length_total_m"] * 1000 + 2.0
    assert float(np.ptp(v[:, :, 1])) <= g["span_m"] * 1000 + 2.0
    assert float(np.ptp(v[:, :, 2])) <= g["height_total_m"] * 1000 + 2.0


def test_onepiece_tessellation_covers_the_whole_solid(built, tmp_path):
    solid, _ = built
    stl = write_stl_verified(solid, tmp_path / "cover.stl")
    mesh_area = float(stl_mesh.Mesh.from_file(str(stl)).areas.sum())
    assert mesh_area / solid.Area() >= 0.985, (
        f"mesh covers only {100 * mesh_area / solid.Area():.1f}% of skin")


def test_onepiece_stl_exports_valid_and_watertight(built, tmp_path):
    solid, _ = built
    stl = write_stl_verified(solid, tmp_path / "tb.stl")
    ok, msg = stl_is_valid_mesh(stl)
    assert ok, msg
    assert stl_watertight_fraction(stl) >= 0.98


def test_onepiece_boom_sockets_still_open(built):
    """The one-piece STL must carry the tube sockets too."""
    solid, meta = built
    booms = (meta.get("sockets") or {}).get("booms") or {}
    for side in ("left", "right"):
        b = booms.get(side) or {}
        assert b.get("open"), f"{side} socket: {b}"
        for x in np.linspace(b["x0_mm"] + 4.0, b["x1_mm"] - 4.0, 5):
            assert not _classify(solid, Vector(float(x), b["y_mm"],
                                               b["z_mm"])), (
                f"{side} socket solid at x={x:.0f} in the one-piece build")


def test_every_wire_run_is_one_straight_tube(built_parts):
    """Builder's spec (round 5, 2026-08-24): a straight rod must pass
    through every wire run end to end - no curved routes, no elbows. The
    recorded centreline of every applied run must be collinear to within
    0.05 mm."""
    from cadquery import Vector
    _parts, meta = built_parts
    con = (meta.get("servos") or {}).get("conduits") or {}
    checked = 0
    for key, ci in con.items():
        if not isinstance(ci, dict) or not ci.get("path_mm"):
            continue
        applied = ((con.get("motor_run") or {}).get("applied")
                   if key == "motor" else ci.get("applied"))
        if not applied:
            continue
        assert ci.get("straight") is True, (
            f"{key}: not built as a straight run")
        assert ci.get("trumpeted") is not True and \
                float(ci.get("mouth_flare") or 1.0) <= 1.0 + 1e-6, (
            f"{key}: the bore tapers (flare {ci.get('mouth_flare')}) - "
            "constant cross-section end to end (builder, round 6)")
        pts = [Vector(*p) for p in ci["path_mm"]]
        a, b = pts[0], pts[-1]
        ab = b - a
        length = ab.Length
        assert length > 1.0, key
        worst = 0.0
        for q in pts[1:-1]:
            worst = max(worst, (q - a).cross(ab).Length / length)
        assert worst < 0.05, (
            f"{key}: centreline bows {worst:.3f} mm off the straight "
            f"chord - a straight rod would bind")
        checked += 1
    if not checked:
        pytest.skip("no applied wire runs on this design")
