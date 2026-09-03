"""CAD build + export invariants for the CONVENTIONAL airplane path (v2).

Mirrors the v1 flying-wing invariants of tests/test_cad_export.py - one valid
watertight solid, nothing ahead of the nose datum, envelope honesty, mesh
coverage, a real hollow bay with a separable lid, print-in-place control
surfaces, proven horn bores, and a wire pipe for every fitted servo - but
ONLY for `airplane_type="conventional"` designs. The flying-wing tests are
untouched; these run BESIDE them.

Budget note: a conventional CAD build takes minutes, so everything expensive
is module-scoped and there is exactly ONE design (sport mission, the same
900 x 1400 x 300 box the v1 suite uses).
"""
import math

import numpy as np
import pytest
from stl import mesh as stl_mesh

from backend.cad.exporters import (
    export_step, stl_is_valid_mesh, stl_watertight_fraction,
    write_stl_verified,
)
from backend.cad.geometry import (
    _elevon_hinge_line, _TIP_START, build_design_parts, build_design_solid,
)
from backend.physics.optimizer import optimize_design
from backend.schemas import GenerateRequest

BOX = {"length_mm": 900, "width_mm": 1400, "height_mm": 300}


def _input(**over):
    body = dict(mission="sport", cruise_speed_ms=15.0, payload_g=0.0,
                box=dict(BOX), n_motors=1, build_method="3d_printed",
                material="lw_pla", airplane_type="conventional")
    body.update(over)
    return GenerateRequest(**body).to_optimizer_input()


@pytest.fixture(scope="module")
def design():
    d = optimize_design(_input())
    assert d["airplane_type"] == "conventional"
    return d


@pytest.fixture(scope="module")
def built(design):
    return build_design_solid(design)


@pytest.fixture(scope="module")
def built_parts(design):
    return build_design_parts(design)


def _classify(solid, p):
    """True if point `p` is inside (or on) the material."""
    from OCP.BRepClass3d import BRepClass3d_SolidClassifier
    from OCP.gp import gp_Pnt
    from OCP.TopAbs import TopAbs_IN, TopAbs_ON
    c = BRepClass3d_SolidClassifier(solid.wrapped, gp_Pnt(p.x, p.y, p.z), 1e-7)
    return c.State() in (TopAbs_IN, TopAbs_ON)


# ---------------------------------------------------------------------------
# One aircraft, one solid
# ---------------------------------------------------------------------------

def test_builds_exactly_one_valid_solid(built):
    solid, meta = built
    assert solid.isValid()
    assert len(solid.Solids()) == 1, (
        f"{len(solid.Solids())} disconnected solids - the wing, a tail or "
        "the fin did not fuse into the fuselage")
    assert meta["valid_solid"]
    assert solid.Volume() > 0


def test_nothing_reaches_ahead_of_the_nose_datum(built, tmp_path):
    """x = 0 is the datum every station number is measured from; the tractor
    firewall face sits a few mm aft of it (spinner clearance). Measured on
    the exported mesh, not BoundingBox()."""
    solid, _ = built
    stl = write_stl_verified(solid, tmp_path / "datum.stl")
    xmin = float(stl_mesh.Mesh.from_file(str(stl)).vectors[:, :, 0].min())
    assert xmin >= -0.25, f"geometry at x = {xmin:.3f} mm"


def test_cad_stays_inside_the_recorded_envelope(design, built, tmp_path):
    solid, _ = built
    g = design["geometry"]
    v = stl_mesh.Mesh.from_file(
        str(write_stl_verified(solid, tmp_path / "env.stl"))).vectors
    assert float(v[:, :, 0].max()) <= g["length_total_m"] * 1000 + 2.0
    assert float(np.ptp(v[:, :, 1])) <= g["span_m"] * 1000 + 2.0
    assert float(np.ptp(v[:, :, 2])) <= g["height_total_m"] * 1000 + 2.0


def test_recorded_envelope_fits_the_user_box(design):
    g = design["geometry"]
    assert g["span_m"] * 1000 <= BOX["width_mm"] + 1e-6
    assert g["length_total_m"] * 1000 <= BOX["length_mm"] + 1e-6
    assert g["height_total_m"] * 1000 <= BOX["height_mm"] + 1e-6


def test_tessellation_covers_the_whole_solid(built, tmp_path):
    """OCC silently skips pathological boolean-trimmed faces - the mesh gate
    doctrine applies to the conventional airframe exactly as to the wing."""
    solid, _ = built
    stl = write_stl_verified(solid, tmp_path / "cover.stl")
    mesh_area = float(stl_mesh.Mesh.from_file(str(stl)).areas.sum())
    assert mesh_area / solid.Area() >= 0.985, (
        f"mesh covers only {100 * mesh_area / solid.Area():.1f}% of the skin")


# ---------------------------------------------------------------------------
# Parts
# ---------------------------------------------------------------------------

def test_expected_parts_are_present_named_and_valid(built_parts):
    parts, meta = built_parts
    for need in ("airframe", "aileron_left", "aileron_right", "rudder",
                 "hatch_lid", "cg_marker"):
        assert need in parts, f"no {need}, got {sorted(parts)}"
    assert any(n.startswith("elevator") for n in parts), sorted(parts)
    for name, solid in parts.items():
        assert solid.isValid(), name
        assert solid.Volume() > 0, name
    # assembled in place: no part past the recorded length (mesh-measured)
    aft = max(max(v.x for v in p.tessellate(0.2)[0]) for p in parts.values())
    assert aft <= meta["length_mm"] + 2.0


# ---------------------------------------------------------------------------
# Equipment bay + hatch
# ---------------------------------------------------------------------------

def test_fuselage_bay_is_hollow_and_verified(built_parts):
    """The bay must be real, MEASURED space - and because a boolean can fail
    without failing, the void is re-proven here by classification on the
    built part, not taken from the report alone."""
    parts, meta = built_parts
    bay = meta.get("bay") or {}
    assert bay, "no equipment bay was cut"
    assert bay["volume_cm3"] > 5.0, f"bay is only {bay['volume_cm3']:.1f} cm3"
    # a receiver is ~25 x 16 x 6 mm; the conventional bay should beat that
    assert bay["box_l_mm"] >= 25.0 and bay["box_w_mm"] >= 16.0 \
        and bay["box_h_mm"] >= 6.0
    # the classified z band was measured on the built solid
    zf, zc = bay.get("z_floor_aft_mm"), bay.get("z_ceil_aft_mm")
    assert zf is not None and zc is not None and zc - zf >= 6.0, (
        f"no measured void z band: {zf}..{zc}")
    from cadquery import Vector
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
    assert bb.ylen <= meta["fuselage_mm"]["width"] + 2.0
    # a tray, not a plug
    assert lid.Volume() < 0.35 * bb.xlen * bb.ylen * bb.zlen


def test_the_one_piece_solid_stays_sealed(built):
    solid, _ = built
    assert len(solid.Solids()) == 1
    assert solid.isValid()


# ---------------------------------------------------------------------------
# Control surfaces
# ---------------------------------------------------------------------------

def test_ailerons_hang_on_captive_printed_pins(design, built_parts):
    """Same invariant as the v1 elevons: pin on the axis owned by the wing
    side, the surface's own bore open, and a closed barrel ring around the
    pin - solid / air / solid along every radius."""
    from cadquery import Vector
    from backend.cad.conventional import _make_hosts

    parts, meta = built_parts
    reports = {k: v for k, v in (meta.get("hinges") or {}).items()
               if k.startswith("aileron")}
    assert reports, "no aileron hinge reports at all"

    g = design["geometry"]
    _fus, wing, _stab, _dims = _make_hosts(g)
    ail = g["ailerons"]
    xc = max(0.45, min(0.90, 1.0 - float(ail["chord_frac"])))
    inner = max(wing.fb, 0.10, float(ail["inner_frac"]))
    outer = min(float(ail["outer_frac"]), _TIP_START - 0.01)

    pinned = 0
    for name, info in reports.items():
        if info.get("mode") != "print_in_place":
            continue                    # bevel-only fallback: reported, legal
        pinned += 1
        surface = parts[name]
        others = [s for n, s in parts.items()
                  if n != name and not n.startswith("cg_")]
        sgn = 1.0 if name.endswith("right") else -1.0
        p_in, p_out, _t_in, _t_out = _elevon_hinge_line(
            wing, sgn, inner, outer, xc)
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
            r_pin, r_barrel = 0.5 * st["pin_dia_mm"], 0.5 * st["knuckle_od_mm"]
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
    # a 9-12% section at aileron depth must manage pins on at least one side
    assert pinned >= 1, (
        "both ailerons fell back to bevel-only - the wing section should "
        f"carry pins; reports: { {k: v.get('mode') for k, v in reports.items()} }")


def test_thin_tail_surfaces_report_their_hinge_mode(built_parts):
    """The stab/fin sections are thin; print-in-place may legitimately fall
    back to bevel-only THERE, but the report must say which - a surface with
    no hinge report at all was never processed."""
    _parts, meta = built_parts
    hr = meta.get("hinges") or {}
    for name in ("elevator_left", "elevator_right", "rudder"):
        info = hr.get(name)
        assert isinstance(info, dict) and info.get("mode") in (
            "print_in_place", "bevel_only", "none"), (
            f"{name}: no hinge report ({info})")


def test_every_horn_bore_is_one_proven_25mm_hole(built_parts):
    """ONE plain 2.5 mm hole per horn, existence-checked - the doctrine that
    caught horns shipping as solid triangles. Applies to the aileron horns
    and to the elevator/rudder horns alike."""
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
            f"{name}: bore not verified open (holes_cut={h.get('holes_cut')})")


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
    """The wing at 12% thickness has room for 9 g servos under both ailerons
    on this reference design - a refusal here is a real regression, not a
    thin-section property."""
    _parts, meta = built_parts
    bays = (meta.get("servos") or {}).get("bays") or {}
    assert set(bays) >= {"aileron_left", "aileron_right"}, sorted(bays)
    for name, b in bays.items():
        assert b.get("ok"), f"{name}: {b.get('reason')}"


# ---------------------------------------------------------------------------
# Wire runs
# ---------------------------------------------------------------------------

def test_every_fitted_wing_servo_gets_its_pipe(built_parts):
    """The 'operation whose absence matters' doctrine: the pipe cut must be
    APPLIED, not merely attempted."""
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
        # ...and the straight run must actually open into the bay void
        assert ci.get("riser_open"), (
            f"servo_{name}: the straight run into the bay is not open "
            f"({ci.get('route_open_detail')}) - the lead walks up to the "
            "compartment and never enters")
        assert ci.get("straight") is True, (
            f"servo_{name}: the lead run must be one straight tube")
        # round 6 (builder): the tube meets the bay side wall at exactly
        # 90 degrees - level, zero angle - and the bore is CONSTANT
        assert ci.get("perpendicular") is True, (
            f"servo_{name}: entry_mode={ci.get('entry_mode')} - the run "
            "must enter the bay wall square")
        ax = ci.get("axis_unit") or [0, 1, 0]
        assert abs(ax[0]) < 5e-3, (
            f"servo_{name}: axis {ax} is not square to the side wall "
            "in plan")
        slope = abs(float(ci.get("slope_deg") or 0.0))
        if str(ci.get("entry_mode") or "").endswith("min_slope"):
            # the recorded exception: no LEVEL line fits inside the skin
            # (dihedral drops the section along the run); the shallowest
            # feasible slope ships, named in level_note
            assert slope < 2.5 and ci.get("level_note"), (
                f"servo_{name}: min-slope run at {slope} deg unflagged")
        else:
            assert ci.get("level") is True and slope < 0.05, (
                f"servo_{name}: run slopes {slope} deg - the builder's "
                "spec is ZERO angle")
        assert ci.get("trumpeted") is not True and \
                float(ci.get("mouth_flare") or 1.0) <= 1.0 + 1e-6, (
            f"servo_{name}: the bore tapers (flare "
            f"{ci.get('mouth_flare')}) - the cross-section must be "
            "constant end to end")
        # round 7 (builder): the port is a wider OVAL for the wire bundle
        assert float(ci.get("w_mm") or ci.get("d_mm") or 0.0) >= 11.9, (
            f"servo_{name}: port width {ci.get('w_mm')} mm - the oval "
            "wire port is 12.0 wide (builder, round 7)")
        # ...and the pocket mouth opens AT the measured lead grommet
        # (builder's decision, v2 - no fold-back under the case)
        path = ci.get("path_mm") or []
        if path and "x_centre" in bay:
            # 2.5 mm: covers the chord-fraction drift over the pocket
            # overshoot (1.11 mm measured at 26 deg sweep); the old
            # fold-back mouth sat 5 mm forward and must keep failing.
            x_expect = float(bay["x_centre"]) + x_grommet_off
            assert abs(path[0][0] - x_expect) < 2.5, (
                f"servo_{name}: pipe mouth at x={path[0][0]:.2f} but the "
                f"lead grommet is at x={x_expect:.2f}")


def test_pushrod_exit_runs_are_cut_and_open(built_parts):
    """The elevator/rudder servos sit in the fuselage bay; each gets a
    straight round 8.25 mm trumpeted guide pipe aft through the tail cone.
    Both cuts must be applied AND the centreline classified open."""
    _parts, meta = built_parts
    pr = (meta.get("servos") or {}).get("pushrods") or {}
    for key in ("elevator", "rudder"):
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


# ---------------------------------------------------------------------------
# Motor mount
# ---------------------------------------------------------------------------

def test_motor_holes_are_drilled_and_proven(design, built_parts):
    """The solid nose IS the mount structure; every screw bore and the shaft
    bore are existence-checked by classification (a boolean can fail without
    failing)."""
    parts, meta = built_parts
    assert "motor_mount" not in parts
    mm = design["geometry"]["motor_mount"]
    assert mm["type"] == "tractor"
    assert mm["n_screws"] >= 2
    cut = (meta.get("motor_mount") or {}).get("holes_cut")
    assert cut and all(cut), f"motor bores not verified open: {cut}"
    # and the bolt-circle faces are really on the part
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
# Export integrity
# ---------------------------------------------------------------------------

def test_stl_exports_valid_and_watertight(built, tmp_path):
    solid, _ = built
    stl = write_stl_verified(solid, tmp_path / "conv.stl")
    ok, msg = stl_is_valid_mesh(stl)
    assert ok, msg
    assert stl_watertight_fraction(stl) >= 0.98


def test_step_exports_as_a_named_assembly(design):
    path = export_step(design)
    assert path.exists() and path.stat().st_size > 0
    low = path.read_text(errors="ignore").lower()
    for name in ("airframe", "aileron_left", "aileron_right", "rudder",
                 "hatch_lid"):
        assert name in low, f"STEP has no {name} body"


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
