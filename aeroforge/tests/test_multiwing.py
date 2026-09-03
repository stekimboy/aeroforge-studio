"""CAD build + export invariants for the v3 MULTI-WING types: canard and
tandem (V3_PLAN.md wave 3a; biplane was removed by the builder 2026-08-21).

Mirrors the v2 conventional bar (tests/test_conventional.py) - exactly one
valid watertight solid (mesh >= 98.5% of BRep), nothing ahead of x = 0, bbox
inside the recorded envelope and the user box, expected parts present /
named / valid, a hollow bay proven by classification, hinged surfaces on
captive pins with the clearance table, horn bores proven open, servo pipes
applied with their mouths at the lead grommet, pushrod runs open, motor
bores proven - plus the per-type items: canard foreplane elevator + fused
tip fins, tandem's two wings at their recorded heights with the Quickie
control split.

These tests import `backend.cad.multiwing` DIRECTLY - the geometry.py
dispatch (and therefore STEP export / preview endpoints) is the integration
wave's job, so there are deliberately no exporter-dispatch tests here yet.

Budget note: one type is two full CAD builds (parts + one-piece), 8-12 min
each; physics is seconds. Every build is therefore cached on disk keyed by a
hash of the design dict (minus its per-run uuid) - the physics and the
builders are deterministic (the cadjobs byte-identity proof), so a cache hit
is the same geometry. Set AEROFORGE_MW_CACHE=0 to force cold builds, or
delete the cache dir (tempdir/aeroforge_mw_test_cache). Run per type while
iterating: -k canard / -k tandem.
"""
import hashlib
import json
import math
import os
import tempfile
from pathlib import Path

import numpy as np
import pytest
from cadquery import Solid, Vector
from stl import mesh as stl_mesh

from backend.cad import multiwing as mw
from backend.cad.exporters import (
    stl_is_valid_mesh, stl_watertight_fraction, write_stl_verified,
)
from backend.cad.geometry import _clamp, _clamp_aft, _elevon_hinge_line, \
    _TIP_START
from backend.physics.canard import generate_canard_variants
from backend.physics.tandem import generate_tandem_variants

# same reference box as tests/test_multiwing_physics.py.
# biplane was removed outright (builder, 2026-08-21) - its rows here, its
# fixtures and its two dedicated tests went with the type.
BOX = dict(box_l=1.2, box_w=1.4, box_h=0.4)
TYPES = ("canard", "tandem")
GEN = {"canard": generate_canard_variants,
       "tandem": generate_tandem_variants}
EXPECTED_PARTS = {
    "canard": {"airframe", "aileron_left", "aileron_right",
               "elevator_left", "elevator_right", "hatch_lid", "cg_marker"},
    "tandem": {"airframe", "aileron_left", "aileron_right",
               "elevator_left", "elevator_right", "rudder",
               "hatch_lid", "cg_marker"},
}
PUSHROD_KEYS = {"canard": {"elevator"},
                "tandem": {"elevator", "rudder"}}


def _input(airplane_type):
    return dict(airplane_type=airplane_type, planform=None, mission="sport",
                v_cruise=15.0, v_stall_target=None, payload_kg=0.0, **BOX,
                vstab=None, n_motors=1, material="lw_pla",
                build_method="3d_printed", airfoil_override=None,
                ar_target=None, sm_override=None, mount_screws=None,
                mount_screw_d_mm=None, mount_bolt_circle_mm=None)


# ---------------------------------------------------------------------------
# Disk cache for the built solids (see module docstring)
# ---------------------------------------------------------------------------

_CACHE_ON = os.environ.get("AEROFORGE_MW_CACHE", "1") != "0"
_CACHE = Path(os.environ.get("AEROFORGE_MW_CACHE_DIR")
              or Path(tempfile.gettempdir()) / "aeroforge_mw_test_cache")


def _key(design: dict) -> str:
    payload = {k: v for k, v in design.items() if k != "id"}
    return hashlib.sha1(json.dumps(payload, sort_keys=True,
                                   default=str).encode()).hexdigest()[:16]


def _cached_parts(design):
    key = _key(design)
    meta_p = _CACHE / f"{key}_parts_meta.json"
    if _CACHE_ON and meta_p.exists():
        meta = json.loads(meta_p.read_text())
        try:
            parts = {n: Solid.importBrep(str(_CACHE / f"{key}_part_{n}.brep"))
                     for n in meta["part_names"]}
            return parts, meta
        except Exception:
            pass
    parts, meta = mw.build_design_parts(design)
    if _CACHE_ON:
        _CACHE.mkdir(parents=True, exist_ok=True)
        for n, s in parts.items():
            s.exportBrep(str(_CACHE / f"{key}_part_{n}.brep"))
        meta_p.write_text(json.dumps(meta, default=str))
    return parts, meta


def _cached_solid(design):
    key = _key(design)
    meta_p = _CACHE / f"{key}_solid_meta.json"
    brep_p = _CACHE / f"{key}_solid.brep"
    if _CACHE_ON and meta_p.exists() and brep_p.exists():
        try:
            return Solid.importBrep(str(brep_p)), json.loads(
                meta_p.read_text())
        except Exception:
            pass
    solid, meta = mw.build_design_solid(design)
    if _CACHE_ON:
        _CACHE.mkdir(parents=True, exist_ok=True)
        solid.exportBrep(str(brep_p))
        meta_p.write_text(json.dumps(meta, default=str))
    return solid, meta


# ---------------------------------------------------------------------------
# Fixtures: one design per type - the PRIMARY character of the type's
# generate_*_variants, exactly what the app would lead the gallery with
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module", params=TYPES)
def tp(request):
    return request.param


@pytest.fixture(scope="module")
def design(tp):
    variants = GEN[tp](_input(tp))
    d = next(v for v in variants if v["primary"])["design"]
    assert d["airplane_type"] == tp
    return d


@pytest.fixture(scope="module")
def built_parts(design):
    return _cached_parts(design)


@pytest.fixture(scope="module")
def built_solid(design):
    return _cached_solid(design)


@pytest.fixture(scope="module")
def solid_stl(built_solid, tmp_path_factory, tp):
    solid, _meta = built_solid
    path = tmp_path_factory.mktemp(f"mw_{tp}") / f"{tp}.stl"
    return solid, write_stl_verified(solid, path)


def _classify(solid, p):
    """True if point `p` is inside (or on) the material."""
    from OCP.BRepClass3d import BRepClass3d_SolidClassifier
    from OCP.gp import gp_Pnt
    from OCP.TopAbs import TopAbs_IN, TopAbs_ON
    c = BRepClass3d_SolidClassifier(solid.wrapped, gp_Pnt(p.x, p.y, p.z),
                                    1e-7)
    return c.State() in (TopAbs_IN, TopAbs_ON)


# ---------------------------------------------------------------------------
# One aircraft, one watertight solid, honest envelope
# ---------------------------------------------------------------------------

def test_builds_exactly_one_valid_solid(built_solid):
    solid, meta = built_solid
    assert solid.isValid()
    assert len(solid.Solids()) == 1, (
        f"{len(solid.Solids())} disconnected solids - a wing, fin, strut "
        "or the foreplane did not fuse into the fuselage")
    assert meta["valid_solid"]
    assert solid.Volume() > 0


def test_nothing_reaches_ahead_of_the_nose_datum(solid_stl):
    """x = 0 is the datum every station is measured from. Measured on the
    exported mesh, not BoundingBox()."""
    _solid, stl = solid_stl
    xmin = float(stl_mesh.Mesh.from_file(str(stl)).vectors[:, :, 0].min())
    assert xmin >= -0.25, f"geometry at x = {xmin:.3f} mm"


def test_cad_stays_inside_the_recorded_envelope(design, solid_stl):
    _solid, stl = solid_stl
    g = design["geometry"]
    v = stl_mesh.Mesh.from_file(str(stl)).vectors
    assert float(v[:, :, 0].max()) <= g["length_total_m"] * 1000 + 2.0
    assert float(np.ptp(v[:, :, 1])) <= g["span_m"] * 1000 + 2.0
    assert float(np.ptp(v[:, :, 2])) <= g["height_total_m"] * 1000 + 2.0


def test_recorded_envelope_fits_the_user_box(design):
    g = design["geometry"]
    assert g["span_m"] <= BOX["box_w"] + 1e-9
    assert g["length_total_m"] <= BOX["box_l"] + 1e-9
    assert g["height_total_m"] <= BOX["box_h"] + 1e-9


def test_tessellation_covers_the_whole_solid(solid_stl):
    """OCC silently skips pathological boolean-trimmed faces - the mesh gate
    doctrine applies to every type."""
    solid, stl = solid_stl
    mesh_area = float(stl_mesh.Mesh.from_file(str(stl)).areas.sum())
    assert mesh_area / solid.Area() >= 0.985, (
        f"mesh covers only {100 * mesh_area / solid.Area():.1f}% of the skin")


def test_stl_exports_valid_and_watertight(solid_stl):
    _solid, stl = solid_stl
    ok, msg = stl_is_valid_mesh(stl)
    assert ok, msg
    assert stl_watertight_fraction(stl) >= 0.98


def test_the_one_piece_solid_stays_sealed(built_solid):
    solid, _ = built_solid
    assert len(solid.Solids()) == 1
    assert solid.isValid()


# ---------------------------------------------------------------------------
# Parts
# ---------------------------------------------------------------------------

def test_expected_parts_are_present_named_and_valid(built_parts, tp):
    parts, meta = built_parts
    assert set(parts) == EXPECTED_PARTS[tp], sorted(parts)
    for name, solid in parts.items():
        assert solid.isValid(), name
        assert solid.Volume() > 0, name
    aft = max(max(v.x for v in p.tessellate(0.2)[0]) for p in parts.values())
    assert aft <= meta["length_mm"] + 2.0


# ---------------------------------------------------------------------------
# Equipment bay + hatch
# ---------------------------------------------------------------------------

def test_fuselage_bay_is_hollow_and_verified(built_parts):
    """The bay must be real, MEASURED space - re-proven here by
    classification on the built part, never taken from the report alone."""
    parts, meta = built_parts
    bay = meta.get("bay") or {}
    assert bay, "no equipment bay was cut"
    assert bay["volume_cm3"] > 5.0
    # a receiver is ~25 x 16 x 6 mm; every type's bay should beat that
    assert bay["box_l_mm"] >= 25.0 and bay["box_w_mm"] >= 16.0 \
        and bay["box_h_mm"] >= 6.0
    zf, zc = bay.get("z_floor_aft_mm"), bay.get("z_ceil_aft_mm")
    assert zf is not None and zc is not None and zc - zf >= 6.0, (
        f"no measured void z band: {zf}..{zc}")
    x_probe = 0.5 * (bay["x0_mm"] + bay["x1_mm"])
    p = Vector(x_probe, 0.0, 0.5 * (zf + zc))
    assert not _classify(parts["airframe"], p), (
        f"probe at ({p.x:.0f}, 0, {p.z:.0f}) is still material - the bay "
        "cut silently did nothing")


def test_hatch_lid_is_a_separate_tray(built_parts):
    parts, meta = built_parts
    lid = parts.get("hatch_lid")
    assert lid is not None
    assert lid.isValid() and len(lid.Solids()) == 1
    bay = meta.get("bay") or {}
    bb = lid.BoundingBox()
    assert bb.xlen > 20.0 and bb.ylen > 15.0, "lid is a sliver"
    assert bb.ylen >= bay["hatch_w_mm"] - 1.0
    # a tray, not a plug
    assert lid.Volume() < 0.35 * bb.xlen * bb.ylen * bb.zlen


# ---------------------------------------------------------------------------
# Control surfaces: captive pins, clearances
# ---------------------------------------------------------------------------

def _aileron_hinge_line(design, sgn):
    """The aileron hinge line exactly as `_install_hardware` computes it -
    same hosts (make_hosts is what the builder itself calls), same clamps."""
    g = design["geometry"]
    hosts = mw.make_hosts(design)
    wing = hosts["main"]
    ail = g.get("ailerons") or {}
    inner = _clamp(float(ail.get("inner_frac", 0.55)),
                   max(wing.fb, 0.10), 0.85)
    outer = _clamp(float(ail.get("outer_frac", 0.95)),
                   inner + 0.10, _TIP_START - 0.01)
    xc = _clamp(1.0 - float(ail.get("chord_frac", 0.25)), 0.45, 0.90)
    return _elevon_hinge_line(wing, sgn, inner, outer, xc)


def test_ailerons_hang_on_captive_printed_pins(design, built_parts):
    """Same invariant as v1/v2: pin on the axis owned by the wing side, the
    surface's own bore open, and a closed barrel ring around the pin -
    solid / air / solid along every radius."""
    parts, meta = built_parts
    reports = {k: v for k, v in (meta.get("hinges") or {}).items()
               if k.startswith("aileron")}
    assert reports, "no aileron hinge reports at all"

    pinned = 0
    for name, info in reports.items():
        if info.get("mode") != "print_in_place":
            continue                    # bevel-only fallback: reported, legal
        pinned += 1
        surface = parts[name]
        others = [s for n, s in parts.items()
                  if n != name and not n.startswith("cg_")]
        sgn = 1.0 if name.endswith("right") else -1.0
        p_in, p_out, _ti, _to = _aileron_hinge_line(design, sgn)
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
                f"{name} @ {st['station_mm']:.0f}: nothing on the hinge axis")
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
    assert pinned >= 1, (
        "both ailerons fell back to bevel-only; reports: "
        f"{ {k: v.get('mode') for k, v in reports.items()} }")


def test_hinged_surfaces_report_their_mode(built_parts, tp):
    """Every separated surface must carry a hinge report - a surface with no
    report was never processed. The thin foreplane/stab/fin sections may
    legitimately fall back to bevel-only, and the report must say so (the
    documented fallback the canard elevator is allowed)."""
    _parts, meta = built_parts
    hr = meta.get("hinges") or {}
    need = ["elevator_left", "elevator_right"]
    if tp == "tandem":
        need.append("rudder")
    for name in need:
        info = hr.get(name)
        assert isinstance(info, dict) and info.get("mode") in (
            "print_in_place", "bevel_only", "none"), (
            f"{name}: no hinge report ({info})")


def test_every_horn_bore_is_one_proven_25mm_hole(built_parts):
    """ONE plain 2.5 mm hole per horn, existence-checked (the doctrine that
    caught horns shipping as solid triangles)."""
    _parts, meta = built_parts
    horns = {k: v for k, v in ((meta.get("servos") or {}).get("horns")
                               or {}).items() if v.get("ok") is not False}
    assert horns, "no horns were fused at all"
    assert any(k.startswith("aileron") for k in horns), sorted(horns)
    for name, h in horns.items():
        holes = h.get("holes") or []
        assert len(holes) == 1, f"{name}: {len(holes)} holes, want exactly 1"
        assert abs(h.get("hole_d_mm", 0) - 2.5) < 0.01, name
        assert not h.get("feed_d_mm"), f"{name}: keyhole feed lobe is back"
        assert h.get("holes_cut") == [True], (
            f"{name}: bore not verified open "
            f"(holes_cut={h.get('holes_cut')})")


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
        assert resid is not None, f"{name}: horn was not aligned to the arm"
        assert resid <= 2.0, (
            f"{name}: horn stands {resid:.1f} mm off the servo arm's plane")


def test_ailerons_get_servo_bays(built_parts):
    _parts, meta = built_parts
    bays = (meta.get("servos") or {}).get("bays") or {}
    assert set(bays) >= {"aileron_left", "aileron_right"}, sorted(bays)
    for name, b in bays.items():
        assert b.get("ok"), f"{name}: {b.get('reason')}"


# ---------------------------------------------------------------------------
# Wire runs
# ---------------------------------------------------------------------------

def test_every_fitted_wing_servo_gets_its_pipe(built_parts):
    """The 'operation whose absence matters' doctrine: the pipe must be
    APPLIED, its run must genuinely enter the bay void, and the mouth must
    open AT the measured lead grommet (no fold-back under the case)."""
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
            f"servo_{name}: wire pipe not applied "
            f"({ci.get('skipped') or ci.get('why') or 'missing'})")
        assert ci.get("into_bay_open"), (
            f"servo_{name}: the run into the bay is not open "
            f"({ci.get('leg_open_detail')}) - the lead walks up to the "
            "compartment and never enters")
        path = ci.get("path_mm") or []
        if path and "x_centre" in bay:
            x_expect = float(bay["x_centre"]) + x_grommet_off
            assert abs(path[0][0] - x_expect) < 2.5, (
                f"servo_{name}: pipe mouth at x={path[0][0]:.2f} but the "
                f"lead grommet is at x={x_expect:.2f}")


def test_pushrod_exit_runs_are_cut_and_open(built_parts, tp):
    """Fuselage-bay servos drive the tail/foreplane through straight round
    8.25 mm trumpeted guide pipes; each cut must be applied AND classified
    open. The canard/tandem elevator pipe exits FORWARD (the aft pattern
    mirrored); rudder pipes exit aft."""
    _parts, meta = built_parts
    pr = (meta.get("servos") or {}).get("pushrods") or {}
    for key in PUSHROD_KEYS[tp]:
        info = pr.get(key)
        assert isinstance(info, dict), f"no {key} pushrod run at all"
        assert info.get("applied"), (
            f"{key} pushrod pipe not applied: "
            f"{info.get('skipped') or info.get('why') or info.get('reason')}")
        assert info.get("route_open"), (
            f"{key} pushrod pipe reported applied but its bore is blocked: "
            f"{info.get('route_open_detail')}")
        assert abs(info.get("d_mm", 0) - 8.25) < 0.01
        assert info.get("trumpeted") is True
    if tp in ("canard", "tandem"):
        assert pr["elevator"].get("exit_side") == "forward"


# ---------------------------------------------------------------------------
# Motor mount
# ---------------------------------------------------------------------------

def test_motor_holes_are_drilled_and_proven(design, built_parts, tp):
    """Canard bolts a pusher to the flat AFT face; tandem a tractor to the
    nose. Every screw and shaft bore is existence-checked."""
    parts, meta = built_parts
    mm = design["geometry"]["motor_mount"]
    assert mm["type"] == ("pusher" if tp == "canard" else "tractor")
    assert mm["n_screws"] >= 2
    cut = (meta.get("motor_mount") or {}).get("holes_cut")
    assert cut and all(cut), f"motor bores not verified open: {cut}"
    xf, yc, zc = mm["x_m"] * 1000, mm["y_m"] * 1000, mm["z_m"] * 1000
    at_circle = 0
    for f in parts["airframe"].Faces():
        c = f.Center()
        if abs(c.x - xf) < 40:
            r = math.hypot(c.y - yc, c.z - zc)
            if abs(r - mm["bolt_circle_radius_mm"]) <= 0.5:
                at_circle += 1
    assert at_circle >= mm["n_screws"], (
        f"expected {mm['n_screws']} screw holes on the bolt circle, found "
        f"{at_circle} faces there")


# ---------------------------------------------------------------------------
# Per-type geometry
# ---------------------------------------------------------------------------

def test_canard_tip_fins_are_fused(design, built_parts, tp):
    """The main wing carries two FIXED tip fins (fused like flying-wing
    winglets, no rudder) - proven by classifying a point inside each fin's
    mid-span volume on the built airframe."""
    if tp != "canard":
        pytest.skip("canard only")
    parts, meta = built_parts
    fins_meta = meta.get("fins") or {}
    assert fins_meta.get("count") == 2, fins_meta
    g = design["geometry"]
    fins = g["fins"]
    hosts = mw.make_hosts(design)
    wing = hosts["main"]
    h_fin = fins["height_m"] * 1000
    c_root_f = fins["c_root_m"] * 1000
    c_tip_f = fins["c_tip_m"] * 1000
    x_le2, sweep_f = _clamp_aft(fins["x_le_m"] * 1000, c_root_f, h_fin, 30.0,
                                c_tip_f / c_root_f,
                                x_aft=g["length_total_m"] * 1000)
    for sgn in (1.0, -1.0):
        sec_t = wing.section(sgn * _TIP_START)
        y_fin = sec_t.le.y
        # a point at 40% of the fin's span, mid-chord - inside the panel
        f = 0.40
        c_f = c_root_f + (c_tip_f - c_root_f) * f
        xc_mid = _clamp((min(fins["x_le_m"] * 1000 + 0.5 * c_root_f,
                             sec_t.le.x + 0.85 * sec_t.chord)
                         - sec_t.le.x) / max(sec_t.chord, 1e-6), 0.05, 0.9)
        z_root = 0.5 * (wing.crown_z(sgn * _TIP_START, xc_mid)
                        + wing.keel_z(sgn * _TIP_START, xc_mid)) - 2.0
        p = Vector(x_le2 + f * h_fin * math.tan(math.radians(sweep_f))
                   + 0.45 * c_f, y_fin, z_root + f * h_fin)
        assert _classify(parts["airframe"], p), (
            f"no fin material at {p.x:.0f}, {p.y:.0f}, {p.z:.0f} "
            f"({'right' if sgn > 0 else 'left'} tip fin)")


def test_tandem_both_wings_at_their_recorded_heights(design, built_parts,
                                                     tp):
    """Front wing LOW, rear wing HIGH, each at its dict z (the Quickie
    arrangement) - probed inside both wings' mid-panel volume - and the
    control split is front elevator + rear ailerons + rudder."""
    if tp != "tandem":
        pytest.skip("tandem only")
    parts, _meta = built_parts
    g = design["geometry"]
    hosts = mw.make_hosts(design)
    for host, label in ((hosts["main"], "rear"), (hosts["fore"], "front")):
        f = 0.35
        sec = host.section(f)
        xc = 0.40
        p = Vector(sec.le.x + xc * sec.chord, sec.le.y,
                   0.5 * (host.crown_z(f, xc) + host.keel_z(f, xc)))
        assert _classify(parts["airframe"], p), (
            f"{label} wing: no material at ({p.x:.0f}, {p.y:.0f}, "
            f"{p.z:.0f})")
    # front wing genuinely LOW, rear genuinely HIGH (dict says so; assert
    # the relation the CAD placed them by)
    assert g["wing2"]["z_m"] < g["wing_z_m"]
    assert {"aileron_left", "aileron_right", "elevator_left",
            "elevator_right", "rudder"} <= set(parts)


# (test_biplane_upper_wing_on_struts was deleted with the biplane type -
# builder, 2026-08-21. The fuse-existence doctrine it exercised lives on in
# test_every_feature_actually_fused below, which runs on every type.)


# ---------------------------------------------------------------------------
# Regression: features that fused "successfully" but were not in the model
# ---------------------------------------------------------------------------

def test_every_feature_actually_fused(built_parts):
    """No feature may be reported fused and be absent.

    The biplane shipped with NO vertical fin: `base.fuse(fin)` returned one
    valid solid that simply did not contain the fin, the old gate accepted
    it on those two flags, and the failure only surfaced three steps later
    as a rudder split that took an 89 mm3 chip out of the tail cone. The
    builders now classify instead of trusting the flags
    (`geometry.fuse_feature`), and a feature that still cannot be fused says
    so in the warnings rather than vanishing quietly.
    """
    _parts, meta = built_parts
    missing = [w for w in (meta.get("warnings") or [])
               if "did not fuse into the airframe" in w]
    assert not missing, missing


def test_no_moving_surface_carries_a_fixed_fin(design, built_parts, tp):
    """A control surface must not reach up into a fixed vertical surface.

    The canard's aileron pocket is unbounded in z, and at the sized 0.95
    outer station it cut the tip fin's whole aft section free: the aileron
    came off carrying two thirds of a fin (z = 164.7 mm on a wing whose skin
    tops out at 20 mm). That is the fin-on-elevon fiasco (DECISIONS.md), and
    it is a flying-qualities bug, not a cosmetic one - the fin would flap
    with roll input.

    Checked against the surface's OWN host: measuring every surface against
    the main wing was meaningless on the biplane, whose elevator lives on a
    tail stab 46 mm above the lower wing.

    Only the UPWARD reach is bounded this tightly. Downward, a horned surface
    legitimately hangs its control horn `protrude_max_mm` (15 mm, the user's
    cap) below the skin, and only ONE of a split elevator pair is horned -
    the panels are joined by a wire at the bench, standard practice - so the
    pair's z-extents are asymmetric BY DESIGN. `elevator_right` reaching
    13.7 mm below the stab keel is that horn, not a pocket running into the
    tail cone; the allowance below is the horn cap plus the host's own
    section, and it is the direction the fin-on-elevon failure never takes
    (a fin stands UP).
    """
    parts, _meta = built_parts
    hosts = mw.make_hosts(design)
    # the host each moving surface is actually hinged to
    ail_host = hosts["main"]                      # every type: the roll wing
    elev_host = hosts.get("stab") or hosts["fore"]   # biplane tail / foreplane

    def band(host):
        crown = max(host.crown_z(f, xc)
                    for f in (0.0, 0.4, 0.7, 0.9) for xc in (0.2, 0.5, 0.8))
        keel = min(host.keel_z(f, xc)
                   for f in (0.0, 0.4, 0.7, 0.9) for xc in (0.2, 0.5, 0.8))
        return crown, keel, 1.5 * (crown - keel)

    for name, part in parts.items():
        if not name.startswith(("aileron", "elevator")):
            continue
        host = ail_host if name.startswith("aileron") else elev_host
        crown, keel, margin = band(host)
        bb = part.BoundingBox()
        assert bb.zmax <= crown + margin, (
            f"{name} reaches z={bb.zmax:.1f} against its host's crown of "
            f"{crown:.1f} (+{margin:.1f} allowed) - it is carrying a fixed "
            f"surface")
        # below: the host's section plus the control horn's own 15 mm cap
        horn = (_meta.get("servos") or {}).get("horns", {}).get(name) or {}
        drop = margin + float(horn.get("protrude_max_mm", 15.0))
        assert bb.zmin >= keel - drop, (
            f"{name} reaches down to z={bb.zmin:.1f} against its host's keel "
            f"of {keel:.1f} (-{drop:.1f} allowed, horn cap included)")


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
