"""CAD build + STEP/STL export for every flying-wing planform.

Enforces the geometry invariants of SPEC_FLYING_WING.md section 8, and the
topology requirement of section 5: the SURFACE is one continuous full-span
loft (not a pod with wings bolted to it), while the parts export divides that
same surface into bodies a CAD user can separate.
"""
import numpy as np
import pytest
from stl import mesh as stl_mesh

from backend.cad.exporters import (
    EXPORT_DIR, export_step, export_stl, stl_is_valid_mesh,
    stl_watertight_fraction, write_stl_verified,
)
from backend.cad.geometry import build_design_parts, build_design_solid
from backend.physics.config_defs import PLANFORMS
from backend.physics.optimizer import optimize_design
from backend.schemas import GenerateRequest

BOX = {"length_mm": 900, "width_mm": 1400, "height_mm": 300}


def _input(**over):
    body = dict(mission="sport", cruise_speed_ms=15.0, payload_g=0.0,
                box=dict(BOX), n_motors=1, build_method="3d_printed",
                material="lw_pla")
    body.update(over)
    return GenerateRequest(**body).to_optimizer_input()


@pytest.fixture(scope="module", params=list(PLANFORMS))
def design(request):
    return optimize_design(_input(planform=request.param))


@pytest.fixture(scope="module")
def built(design):
    return build_design_solid(design)


@pytest.fixture(scope="module")
def built_parts(design):
    return build_design_parts(design)


# ---------------------------------------------------------------------------
# One aircraft, one solid (spec 8.1)
# ---------------------------------------------------------------------------

def test_builds_exactly_one_valid_solid(design, built):
    solid, meta = built
    assert solid.isValid(), design["planform"]
    assert len(solid.Solids()) == 1, (
        f"{design['planform']}: {len(solid.Solids())} disconnected solids - "
        "something is not attached to the airframe")
    assert meta["valid_solid"]
    assert solid.Volume() > 0


def test_the_surface_is_one_loft_but_the_assembly_is_separable(design, built,
                                                              built_parts):
    """Two requirements that pull opposite ways, and both have to hold.

    The SURFACE is one continuous full-span loft - that is what stops the
    aircraft reading as parts glued together, and the one-piece STL keeps it
    that way. The ASSEMBLY has to come apart, because a builder opening the
    STEP in Fusion needs bodies they can export and print separately. So the
    parts export divides that same surface at the blend station; the pieces are
    coincident at the joint and reassemble into exactly the original shape.
    """
    parts, _meta = built_parts
    solid, _ = built
    assert len(solid.Solids()) == 1, "the one-piece STL must stay one solid"

    for gone in ("fuselage", "pod", "canopy"):
        assert gone not in parts, f"{design['planform']}: {gone!r} still built"
    assert {"centre_body", "wing_left", "wing_right"} <= set(parts), sorted(parts)

    # Measured on the TESSELLATED SURFACE, not on BoundingBox(). OCC's bounding
    # box for a B-spline is the box of the CONTROL POLYGON, which stands off the
    # surface it describes - on this blended body by about 3 mm a side. That is
    # not a gap in the model, it is the box being loose, and asking it whether
    # two parts touch gives an answer that has nothing to do with the geometry.
    def _y(name):
        verts, _tris = parts[name].tessellate(0.2)
        ys = np.array([v.y for v in verts])
        return ys.min(), ys.max()

    span = design["geometry"]["span_m"] * 1000
    extents = {n: _y(n) for n in ("centre_body", "wing_left", "wing_right")}
    total = sum(hi - lo for lo, hi in extents.values())
    assert total >= 0.97 * span, "the split pieces do not add back up to the span"
    # and they must meet exactly: each panel starts where the body ends
    cb_lo, cb_hi = extents["centre_body"]
    for side, name in ((1, "wing_right"), (-1, "wing_left")):
        w_lo, w_hi = extents[name]
        edge = cb_hi if side > 0 else cb_lo
        near = w_lo if side > 0 else w_hi
        assert abs(near - edge) < 0.5, (
            f"{name} does not meet the centre body: {near:.2f} vs {edge:.2f} mm")


def test_the_hatch_opens_into_the_bay(design, built_parts):
    """Removing the lid has to expose the compartment. A solid with a SEALED
    internal void reports two shells (an outer surface and an inner one); an
    equipment bay you can actually reach into reports one.

    This caught a real defect: the bay ceiling is set by the crown at its outer
    EDGE, where the section is far thinner than on the centreline, so a block
    of solid material sat between that ceiling and the crown and the lid was
    only shaving the skin off the top of it.
    """
    parts, _meta = built_parts
    body = parts.get("centre_body") or parts["airframe"]
    assert len(body.Shells()) == 1, (
        f"{design['planform']}: the equipment bay is a sealed void - taking "
        "the lid off does not open it")
    assert "hatch_lid" in parts, sorted(parts)


def test_the_bay_keeps_its_belly(design, built_parts):
    """The compartment floor must survive the bay cut in the PARTS build.

    2026-08-21: a swept sport shipped with a 224 x 64 mm hole clean through
    the belly ("just no wall at all, just a large box hole going through the
    design"). The floor is a keel + wall sliver between two spline lofts,
    OCC's cut tolerance-merged the two surfaces and deleted both, and every
    existing gate passed the result - one valid solid, meshes completely,
    void open, ONE shell (which is exactly what an open compartment reports,
    so the shell count cannot tell a lid aperture from a missing belly).
    `hatch._skin_breached` now rejects such a rung and the raised-floor
    retry rebuilds it; this test pins the outcome the way the builder sees
    it: a vertical ray down the bay's centreline must hit BELLY material
    below the body's mid-plane, not fly straight through to the seat.
    """
    parts, meta = built_parts
    body = parts.get("centre_body") or parts.get("airframe")
    bay = (meta.get("bay") or {})
    if body is None or "x0_mm" not in bay:
        pytest.skip("no carved bay on this design")
    verts, tris = body.tessellate(0.5)
    p = np.asarray([[v.x, v.y, v.z] for v in verts], dtype=float)
    t = p[np.asarray(tris, dtype=int)]
    x0, x1 = float(bay["x0_mm"]), float(bay["x1_mm"])
    for frac in (0.25, 0.50, 0.75):
        x = x0 + frac * (x1 - x0)
        a, b, c = t[:, 0], t[:, 1], t[:, 2]
        det = ((b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1])
               - (c[:, 0] - a[:, 0]) * (b[:, 1] - a[:, 1]))
        ok = np.abs(det) > 1e-12
        aa, bb, cc, dd = a[ok], b[ok], c[ok], det[ok]
        u = ((bb[:, 0] - aa[:, 0]) * (0.0 - aa[:, 1])
             - (bb[:, 1] - aa[:, 1]) * (x - aa[:, 0])) / dd
        v = ((x - aa[:, 0]) * (cc[:, 1] - aa[:, 1])
             - (0.0 - aa[:, 1]) * (cc[:, 0] - aa[:, 0])) / dd
        w = 1.0 - u - v
        hit = (u >= -1e-9) & (v >= -1e-9) & (w >= -1e-9)
        assert hit.any(), (
            f"{design['planform']}: no material at all on the bay "
            f"centreline at x = {x:.0f}")
        zmin = float((w[hit] * aa[hit][:, 2] + v[hit] * bb[hit][:, 2]
                      + u[hit] * cc[hit][:, 2]).min())
        assert zmin < 0.0, (
            f"{design['planform']}: the lowest surface on the bay "
            f"centreline at x = {x:.0f} is z = {zmin:.1f} - the belly under "
            f"the compartment is gone")


def test_nothing_reaches_ahead_of_the_nose_datum(design, built, tmp_path):
    """Every CG / station number the user reads is measured from x = 0.

    Measured on the exported MESH, not on `Shape.BoundingBox()`: for B-spline
    faces OCC builds the bounding box from the control polygon, so it reports
    up to ~0.3 mm outside the real surface and gives a false failure here.

    The datum is the CHORD-LINE nose. An airfoil's leading-edge radius makes
    the physical surface bulge a fraction of a millimetre ahead of that point,
    so the quarter-millimetre tolerance is the definition of the datum, not
    slack: it is an order of magnitude below a 3D printer's layer height and
    below the tessellation tolerance itself.
    """
    solid, _ = built
    stl = write_stl_verified(solid, tmp_path / "datum.stl")
    xmin = float(stl_mesh.Mesh.from_file(str(stl)).vectors[:, :, 0].min())
    assert xmin >= -0.25, f"{design['planform']}: geometry at x = {xmin:.3f} mm"


def test_cad_stays_inside_the_recorded_envelope(design, built, tmp_path):
    """Spec 8.2. If a part needs more room the optimizer must widen the
    budget - never silently clamp the part away from where it belongs.

    Measured on the exported MESH: `Shape.BoundingBox()` is built from the
    control polygon for B-spline faces and over-reports by millimetres (it
    called a 43.9 mm bell wing 49.9 mm tall).
    """
    solid, _ = built
    g = design["geometry"]
    v = stl_mesh.Mesh.from_file(
        str(write_stl_verified(solid, tmp_path / "env.stl"))).vectors
    name = design["planform"]
    assert float(v[:, :, 0].max()) <= g["length_total_m"] * 1000 + 2.0, name
    assert float(np.ptp(v[:, :, 1])) <= g["span_m"] * 1000 + 2.0, name
    assert float(np.ptp(v[:, :, 2])) <= g["height_total_m"] * 1000 + 2.0, name


def test_recorded_dimensions_are_the_real_ones(design, built, tmp_path):
    """The spec panel shows these as the aircraft's dimensions, so they have
    to be what it measures - not a conservative estimate. Anything looser than
    a few millimetres is a number the user cannot trust."""
    solid, _ = built
    g = design["geometry"]
    v = stl_mesh.Mesh.from_file(
        str(write_stl_verified(solid, tmp_path / "dims.stl"))).vectors
    for axis, key, label in ((0, "length_total_m", "length"),
                             (2, "height_total_m", "height")):
        built_mm = float(np.ptp(v[:, :, axis]))
        rec_mm = g[key] * 1000
        assert abs(built_mm - rec_mm) <= 8.0, (
            f"{design['planform']}: records {label} {rec_mm:.1f} mm but "
            f"measures {built_mm:.1f} mm")


def test_recorded_envelope_fits_the_user_box(design):
    g = design["geometry"]
    assert g["span_m"] * 1000 <= BOX["width_mm"] + 1e-6
    assert g["length_total_m"] * 1000 <= BOX["length_mm"] + 1e-6
    assert g["height_total_m"] * 1000 <= BOX["height_mm"] + 1e-6


# ---------------------------------------------------------------------------
# Export integrity
# ---------------------------------------------------------------------------

def test_stl_exports_and_is_a_valid_closed_mesh(design, tmp_path):
    stl = write_stl_verified(build_design_solid(design)[0],
                             tmp_path / f"{design['planform']}.stl")
    ok, msg = stl_is_valid_mesh(stl)
    assert ok, f"{design['planform']}: {msg}"
    assert stl_watertight_fraction(stl) >= 0.98, design["planform"]


def test_tessellation_covers_the_whole_solid(design, built, tmp_path):
    """OCC silently skips pathological boolean-trimmed faces, which once put a
    gaping hole in the exported skin while the BRep still reported valid."""
    solid, _ = built
    stl = write_stl_verified(solid, tmp_path / "cover.stl")
    mesh_area = float(stl_mesh.Mesh.from_file(str(stl)).areas.sum())
    assert mesh_area / solid.Area() >= 0.985, (
        f"{design['planform']}: mesh covers only "
        f"{100 * mesh_area / solid.Area():.1f}% of the solid surface")


def test_step_exports_as_a_named_assembly(design):
    path = export_step(design)
    assert path.exists() and path.stat().st_size > 0
    text = path.read_text(errors="ignore")
    low = text.lower()
    for name in ("centre_body", "wing_left", "wing_right"):
        assert name in low, f"STEP has no {name} body to pull apart"
    assert "canopy" not in low, "the canopy body should be gone"


def test_parts_are_named_valid_and_assembled_in_place(design, built_parts):
    parts, meta = built_parts
    assert parts, design["planform"]
    for name, solid in parts.items():
        assert isinstance(name, str) and name
        assert solid.isValid(), f"{design['planform']}/{name}"
        assert solid.Volume() > 0, f"{design['planform']}/{name}"
    # Assembled positions: no part reaches aft of the recorded length.
    #
    # Measured on the tessellated surface. OCC builds a B-spline's bounding box
    # from the CONTROL POLYGON, so it stands off the real surface - here by
    # enough to fail this by FIVE MICRONS against a 2 mm allowance, which says
    # nothing about the aeroplane. Every other envelope invariant in this file
    # is measured on the mesh for the same reason (DECISIONS.md, "OpenCASCADE
    # lofting lessons"); this one was the straggler.
    aft = max(max(v.x for v in p.tessellate(0.2)[0]) for p in parts.values())
    assert aft <= design["geometry"]["length_total_m"] * 1000 + 2.0, (
        f"{design['planform']}: a part reaches {aft:.1f} mm, past the recorded "
        f"length {design['geometry']['length_total_m'] * 1000:.1f} mm")


# ---------------------------------------------------------------------------
# Vertical surfaces per arrangement (spec 5 "Fin construction", 8.7)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("planform,vstab,carrier", [
    ("swept", "winglets", ("wing_left", "wing_right")),
    ("swept", "twin_fin", ("wing_left", "wing_right")),
    ("swept", "center_fin", ("centre_body",)),
    ("bwb", "center_fin", ("centre_body",)),
    ("plank", "center_fin", ("centre_body",)),
])
def test_vertical_surface_arrangements_build(planform, vstab, carrier):
    """A stationary fin is STRUCTURE, not a component.

    Nothing on a flying wing moves except the elevons, so a fin is fused into
    the surface it stands on rather than handed over as a loose body to glue
    on - a centre fin goes with the centre body, winglets with their wing
    panel. So the test is not "is there a `fin` part" (there must not be one),
    it is "did the fin's material end up in the piece that carries it".
    """
    d = optimize_design(_input(planform=planform, vstab=vstab))
    assert d["geometry"]["vstab"]["type"] == vstab
    parts, meta = build_design_parts(d)

    loose = [n for n in parts if "fin" in n or "winglet" in n]
    assert not loose, f"{planform}/{vstab}: fin handed over loose: {loose}"
    assert meta["vstab"]["type"] == vstab
    assert meta["fin_stations"], f"{planform}/{vstab}: no fin was placed"

    # the fin has to reach above the wing it stands on, in the part that owns it
    for name in carrier:
        assert name in parts, f"{planform}/{vstab}: no {name}, got {sorted(parts)}"
    tallest = max(parts[n].BoundingBox().zlen for n in carrier)
    bare = max(parts[n].BoundingBox().zlen
               for n in parts if n.startswith("elevon"))
    assert tallest > bare, (
        f"{planform}/{vstab}: {carrier} is no taller than a control surface - "
        "the fin did not fuse into it")

    solid, _ = build_design_solid(d)
    assert len(solid.Solids()) == 1, f"{planform}/{vstab}"


def test_bell_spanload_builds_with_no_vertical_surfaces():
    """Fins on a bell-spanload wing defeat the configuration - the tip-region
    induced thrust already gives proverse yaw."""
    d = optimize_design(_input(planform="bell"))
    parts, _ = build_design_parts(d)
    for name in parts:
        assert "fin" not in name and "winglet" not in name, name
    solid, _ = build_design_solid(d)
    assert len(solid.Solids()) == 1


# ---------------------------------------------------------------------------
# Hollow equipment bay, removable hatch lid, motor mount
# ---------------------------------------------------------------------------

def test_centre_body_is_hollow(design, built_parts):
    """The bay has to be real space you can put an FPV stack in, not a marking.

    Measured from what the builder actually cut (`meta["bay"]`) rather than
    from a net-volume comparison against the bare wing: the built airframe also
    carries the motor nacelle, which ADDS material, and on a thin bell wing
    that addition is the same order as the bay - so the difference says "barely
    hollowed" about a wing with a perfectly good compartment in it. (It did
    exactly that, which is why this measures the cut instead of the
    difference.)

    `box_*` is the largest RECTANGULAR box that fits inside the curved bay,
    which is the number that decides whether a battery goes in.
    """
    _parts, meta = built_parts
    bay = meta.get("bay") or {}
    assert bay, f"{design['planform']}: no equipment bay was cut"
    assert bay["volume_cm3"] > 5.0, (
        f"{design['planform']}: bay is only {bay['volume_cm3']:.1f} cm3")
    # A receiver is about 25 x 16 x 6 mm; anything that cannot swallow one is
    # not an equipment bay whatever its total volume says.
    assert bay["box_l_mm"] >= 25.0 and bay["box_w_mm"] >= 16.0 \
        and bay["box_h_mm"] >= 6.0, (
        f"{design['planform']}: usable box is only {bay['box_l_mm']:.0f} x "
        f"{bay['box_w_mm']:.0f} x {bay['box_h_mm']:.0f} mm")


def test_hatch_lid_is_a_separate_part(design, built_parts):
    """A canopy you can actually lift off: its own body in the parts export, a
    skin-thickness tray with a skirt, covering the bay.

    It is deliberately WIDER than the bay it covers - it overlaps onto the seat
    lip that is left standing inside the aperture rim, which is what stops it
    dropping through the hole. Sizing it to the bay width was the old build,
    and that lid was an 81 x 48 mm scrap of skin.
    """
    parts, meta = built_parts
    lid = parts.get("hatch_lid")
    assert lid is not None, f"{design['planform']}: no hatch_lid, got {sorted(parts)}"
    assert lid.isValid() and len(lid.Solids()) == 1
    bay = meta.get("bay") or {}
    bb = lid.BoundingBox()
    assert bb.xlen > 20.0 and bb.ylen > 20.0, "lid is a sliver"
    # covers the aperture it seats over, and stays inside the centre body
    assert bb.ylen >= bay["hatch_w_mm"] - 1.0
    assert bb.ylen <= 2.0 * meta["body_half_width_mm"] + 2.0
    # a tray, not a plug: its volume is far less than its bounding box
    assert lid.Volume() < 0.35 * bb.xlen * bb.ylen * bb.zlen


def test_the_one_piece_solid_keeps_the_lid_attached_and_scribed(design, built):
    """The single STL is the whole aeroplane, so the lid stays in place there
    and only its outline is scribed - two loose solids would not be one
    watertight model."""
    solid, _ = built
    assert len(solid.Solids()) == 1
    assert solid.isValid()


def _classify(solid, p):
    """True if point `p` is inside (or on) `solid`. Point classification, not a
    boolean - intersecting two full wing panels takes minutes."""
    from OCP.BRepClass3d import BRepClass3d_SolidClassifier
    from OCP.gp import gp_Pnt
    from OCP.TopAbs import TopAbs_IN, TopAbs_ON
    c = BRepClass3d_SolidClassifier(solid.wrapped, gp_Pnt(p.x, p.y, p.z), 1e-7)
    return c.State() in (TopAbs_IN, TopAbs_ON)


def test_control_surfaces_hang_on_a_captive_printed_pin(design, built_parts):
    """The hinge has to come off the bed WORKING - a rod already inside a ring.

    This is the invariant behind "print and place". The wing owns a pin lying
    on the hinge axis; the control surface owns a barrel around it with a
    radial gap. If the pin were missing the surface would simply fall off, and
    if the gap were missing the two would print as one lump - the joint has to
    read solid / air / solid along a radius, in every direction.

    Checked by classifying points rather than by intersecting solids: a boolean
    between two full wing panels takes minutes, and this takes milliseconds.
    """
    import math
    from cadquery import Vector
    from backend.cad.geometry import _elevon_hinge_line, _wing_from_design, _TIP_START

    parts, meta = built_parts
    reports = meta.get("hinges") or {}
    if not reports:
        pytest.skip(f"{design['planform']}: no separated control surfaces")

    g = design["geometry"]
    wing = _wing_from_design(g)
    el = g["elevons"]
    xc = max(0.45, min(0.90, 1.0 - float(el["chord_frac"])))

    for name, info in reports.items():
        if info.get("mode") != "print_in_place":
            continue                       # bevel-only fallback: no pin to find
        surface = parts[name]
        # every solid that is NOT this control surface is "the wing side"
        others = [s for n, s in parts.items()
                  if n != name and not n.startswith("cg_")]
        sgn = 1.0 if name.endswith("right") else -1.0
        p_in, p_out, _t_in, _t_out = _elevon_hinge_line(
            wing, sgn, max(wing.fb, 0.10, float(el["inner_frac"])),
            min(float(el["outer_frac"]), _TIP_START - 0.01), xc)
        u = p_out - p_in
        u = u.multiply(1.0 / u.Length)
        e1 = Vector(-u.y, u.x, 0.0)
        e1 = e1.multiply(1.0 / e1.Length)
        e2 = u.cross(e1)
        e2 = e2.multiply(1.0 / e2.Length)

        assert info.get("pin_printed_in_place"), f"{name}: pin is not printed in place"
        assert info["n_hinges_built"] >= 2, (
            f"{name}: {info['n_hinges_built']} hinge(s) - a surface hung on one "
            "point flutters at the free end")

        for st in info["stations"]:
            base = p_in + u.multiply(st["station_mm"])
            r_pin, r_barrel = 0.5 * st["pin_dia_mm"], 0.5 * st["knuckle_od_mm"]
            # 1. the pin: wing material sitting on the axis
            assert any(_classify(s, base) for s in others), (
                f"{name} @ {st['station_mm']:.0f} mm: nothing on the hinge axis "
                "- the surface would fall off the moment it is printed")
            # 2. and the surface must NOT be there, or they are fused solid
            assert not _classify(surface, base), (
                f"{name} @ {st['station_mm']:.0f} mm: the surface fills its own "
                "bore - this prints as one lump, not a hinge")
            # 3. a closed ring of surface material all the way round the pin
            r_wall = 0.5 * (r_pin + r_barrel) + 0.15
            for k in range(8):
                a = 2.0 * math.pi * k / 8.0
                p = (base + e1.multiply(r_wall * math.cos(a))
                     + e2.multiply(r_wall * math.sin(a)))
                assert _classify(surface, p), (
                    f"{name} @ {st['station_mm']:.0f} mm: barrel is open at "
                    f"{math.degrees(a):.0f} deg - the pin escapes, so this is "
                    "not a captive hinge")


def test_the_servo_arm_and_the_control_horn_share_a_plane(design, built_parts):
    """The pushrod is a straight wire, so both ends must stand in one plane.

    Put the horn at a station the servo arm cannot reach and the rod runs
    diagonally: part of every command becomes a side load on the servo's output
    bushing, up-throw and down-throw stop matching, and the horn hole wears
    oval until the slop turns into flutter. This is not trimmable, and it is
    not a tolerance to hold - the horn solves its own station from the arm's,
    so the offset is zero by construction. It regressed to 65 mm once, when the
    horn was pinned to mid-span and the arm fell where the case put it.
    """
    _parts, meta = built_parts
    sv = (meta.get("servos") or {})
    bays = {k: v for k, v in (sv.get("bays") or {}).items() if v.get("ok")}
    horns = {k: v for k, v in (sv.get("horns") or {}).items()
             if v.get("ok") is not False}
    if not bays or not horns:
        pytest.skip(f"{design['planform']}: no servo bay fitted this section")
    for name, h in horns.items():
        if name not in bays:
            continue
        resid = h.get("align_residual_mm")
        assert resid is not None, f"{name}: horn was not aligned to the arm"
        assert resid <= 2.0, (
            f"{design['planform']} {name}: horn stands {resid:.1f} mm off the "
            f"servo arm's plane - the pushrod cannot be straight")


def test_the_pushrod_hole_is_one_clean_25mm_bore(design, built_parts):
    """ONE plain 2.5 mm circular hole per horn, PROVEN cut.

    The builder's own spec, replacing the earlier keyhole (which read as two
    malformed circles on the print). And because a boolean can fail without
    failing - horns shipped with solid triangles while every step reported
    success - the meta must say the bore was verified open, not merely
    attempted."""
    _parts, meta = built_parts
    horns = {k: v for k, v in ((meta.get("servos") or {}).get("horns") or {}).items()
             if v.get("ok") is not False}
    if not horns:
        pytest.skip(f"{design['planform']}: no horns on this design")
    for name, h in horns.items():
        holes = h.get("holes") or []
        assert len(holes) == 1, f"{name}: {len(holes)} holes, want exactly 1"
        assert abs(h.get("hole_d_mm", 0) - 2.5) < 0.01, (
            f"{name}: hole is {h.get('hole_d_mm')} mm, spec is 2.5")
        assert not h.get("feed_d_mm"), (
            f"{name}: a feed lobe is back - the hole must be one plain circle")
        cut = h.get("holes_cut")
        assert cut == [True], (
            f"{name}: bore not verified open (holes_cut={cut}) - the boolean "
            f"failed and the builder would get a solid triangle")


def test_the_servo_lead_leaves_from_the_corner_the_wire_is_in(design,
                                                              built_parts):
    """The lead exits one END of the servo case, not its middle. A channel on
    the pocket centreline makes the wire double back on itself before it can
    enter the run."""
    _parts, meta = built_parts
    bays = {k: v for k, v in (((meta.get("servos") or {}).get("bays")) or {}).items()
            if v.get("ok")}
    if not bays:
        pytest.skip(f"{design['planform']}: no servo bay fitted this section")
    from backend.cad.servos import SERVO_CLEARANCE_MM, SERVO_SG90
    # AT the grommet (builder's decision, v2): its middle stands half a case
    # plus the pocket clearance plus half the 1.0 mm moulded stub past the
    # case centre - just OUTSIDE the case's own end face, inside the stub
    # relief the pocket carves for it.
    x_g = (0.5 * float(SERVO_SG90["body_len_mm"]) + SERVO_CLEARANCE_MM
           + 0.5 * float(SERVO_SG90["lead_stub_mm"]))
    for name, b in bays.items():
        off = abs(b.get("lead_off_centre_mm") or 0.0)
        assert abs(off - x_g) < 0.5, (
            f"{design['planform']} {name}: lead exit is {off:.2f} mm off the "
            f"pocket centre but the measured grommet is at {x_g:.2f} - the "
            f"wire would have to double back to reach the pipe")


def test_every_fitted_servo_gets_its_wire_pipe(design, built_parts):
    """A pocket without a pipe is a servo with nowhere to send its lead.

    The pipe cut must be APPLIED, not merely attempted: the flared-mouth
    regression (both servo runs silently refused on the 610 while every test
    stayed green) proved that nothing asserted the conduits actually exist -
    the same 'operation whose absence matters' doctrine as the bores and the
    bay cut."""
    _parts, meta = built_parts
    sv = meta.get("servos") or {}
    bays = {k: v for k, v in ((sv.get("bays")) or {}).items() if v.get("ok")}
    if not bays:
        pytest.skip(f"{design['planform']}: no servo bay fitted this section")
    con = sv.get("conduits") or {}
    from backend.cad.servos import SERVO_CLEARANCE_MM, SERVO_SG90
    x_grommet_off = (0.5 * float(SERVO_SG90["body_len_mm"])
                     + SERVO_CLEARANCE_MM
                     + 0.5 * float(SERVO_SG90["lead_stub_mm"]))
    for name, bay in bays.items():
        ci = con.get(f"servo_{name}") or {}
        assert ci.get("applied"), (
            f"{design['planform']} servo_{name}: wire pipe not applied "
            f"({ci.get('skipped') or ci.get('gate') or 'missing'}) - the "
            f"servo has a pocket but its lead has no way to the bay")
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
            # (dihedral/twist drops the section along the run); the
            # shallowest feasible slope ships, named in level_note
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
        # The pipe's pocket mouth opens AT the measured lead grommet (the
        # shaft-side end face, where the wire actually leaves the case), so
        # the wire feeds straight in - builder's decision, v2. A mouth that
        # drifts forward of the end face is the old 4 mm fold-back bug.
        path = ci.get("path_mm") or []
        if path and "x_centre" in bay:
            # 2.5 mm tolerance: the cap drifts aft of the grommet because the
            # route holds a CHORD FRACTION over the 6 mm pocket overshoot
            # (measured 1.11 mm on the 26-deg 610 wing); the old fold-back
            # bug sat 5 mm forward and must keep failing.
            x_expect = float(bay["x_centre"]) + x_grommet_off
            assert abs(path[0][0] - x_expect) < 2.5, (
                f"{design['planform']} servo_{name}: pipe mouth at "
                f"x={path[0][0]:.2f} but the lead grommet is at "
                f"x={x_expect:.2f} - the wire would have to double back")


def test_the_motor_leads_enter_the_belly_near_the_mount(design, built_parts):
    """The builder's spec (round 5): ONE straight 8.25 mm pipe from a hole
    through the BOTTOM skin near the motor into the equipment bay - no
    bends, and no separate vertical entry bore meeting an internal channel
    at an angle. Verified in the GEOMETRY: the skin breach must be air and
    the single cut must have passed the mesh gate."""
    from cadquery import Vector
    parts, meta = built_parts
    con = (meta.get("servos") or {}).get("conduits") or {}
    mi = con.get("motor")
    if not isinstance(mi, dict) or not mi.get("ok"):
        if not (meta.get("bay") or {}).get("x1_mm"):
            pytest.skip(f"{design['planform']}: no bay, so no wire run")
        pytest.fail(f"{design['planform']}: motor wiring refused: {mi}")
    assert abs(mi["entry_d_mm"] - 8.25) < 0.01
    assert mi.get("straight") is True, "motor run must be a straight pipe"
    cut = con.get("motor_run") or {}
    assert cut.get("applied"), (
        f"{design['planform']}: motor_run cut was not applied: {cut}")
    assert "motor_entry" not in con, (
        "the separate vertical entry bore is back - the motor wiring must "
        "be one straight pipe whose belly breach IS the entry")
    af = parts.get("centre_body") or parts.get("airframe")
    p = Vector(mi["entry_x_mm"], mi["entry_y_mm"], mi["entry_z_skin_mm"] + 1.0)
    assert not _classify(af, p), (
        f"{design['planform']}: the entry at x={mi['entry_x_mm']} did not "
        f"open the skin - 1 mm above the keel is still material")


def test_motor_mount_is_drilled_and_integral(design, built_parts):
    """Structure, not decoration: the boss is part of the airframe (not a
    loose part) and the screw holes are really there."""
    parts, _ = built_parts
    assert "motor_mount" not in parts, "the mount must be fused into the airframe"
    mm = design["geometry"]["motor_mount"]
    assert mm["n_screws"] >= 2
    assert 1.5 <= mm["screw_hole_d_mm"] <= 6.0
    assert 5.0 <= mm["bolt_circle_radius_mm"] <= 30.0
    assert mm["plate_radius_mm"] > mm["bolt_circle_radius_mm"]
    # the holes must be inside the plate, and the plate inside the aircraft
    assert mm["x_m"] * 1000 <= design["geometry"]["length_total_m"] * 1000 + 2.0


def test_a_custom_bolt_pattern_is_what_gets_drilled():
    """The user types the pattern off their motor's spec sheet, so the model
    has to come out with THOSE holes - not the auto-sized ones."""
    import math
    from backend.schemas import GenerateRequest
    req = GenerateRequest(planform="swept", box=dict(BOX), payload_g=150.0,
                          mount_screws=4, mount_screw_d_mm=2.7,
                          mount_bolt_circle_mm=11.31)
    d = optimize_design(req.to_optimizer_input())
    mm = d["geometry"]["motor_mount"]
    assert mm["n_screws"] == 4
    assert mm["screw_hole_d_mm"] == pytest.approx(2.7)
    assert mm["bolt_circle_radius_mm"] == pytest.approx(11.31)
    # a square pattern's bolt circle runs through its corners
    assert mm["pattern_side_mm"] == pytest.approx(11.31 * math.sqrt(2), rel=1e-6)
    assert mm["plate_radius_mm"] > mm["bolt_circle_radius_mm"]

    solid, _ = build_design_solid(d)
    assert len(solid.Solids()) == 1 and solid.isValid()
    xf = mm["x_m"] * 1000
    yc, zc = mm["y_m"] * 1000, mm["z_m"] * 1000
    at_bolt_circle = 0
    for f in solid.Faces():
        c = f.Center()
        if abs(c.x - xf) < 40:
            r = math.hypot(c.y - yc, c.z - zc)
            if abs(r - mm["bolt_circle_radius_mm"]) <= 0.5:
                at_bolt_circle += 1
    assert at_bolt_circle >= 4, (
        f"expected 4 screw holes on the {mm['bolt_circle_radius_mm']:.2f} mm "
        f"bolt circle, found {at_bolt_circle} faces there")


def test_parts_export_writes_one_stl_per_part(design, tmp_path):
    import zipfile
    from backend.cad.exporters import export_stl_parts
    z = export_stl_parts(design, tmp_path / "parts.zip")
    assert z.exists()
    with zipfile.ZipFile(z) as zf:
        names = [n.rsplit("/", 1)[-1] for n in zf.namelist()]
        assert "centre_body.stl" in names, names
        assert "wing_left.stl" in names and "wing_right.stl" in names, names
        assert "hatch_lid.stl" in names, names
        assert "parts.json" in names
        for n in zf.namelist():
            if n.endswith(".stl"):
                assert zf.getinfo(n).file_size > 1000, n


def test_exports_land_in_the_exports_directory(design):
    p = export_stl(design)
    assert p.exists()
    assert EXPORT_DIR in p.parents or p.parent == EXPORT_DIR


def test_every_wire_run_is_one_straight_tube(design, built_parts):
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


def test_rear_hollow_is_one_central_channel_not_two_boxes(design,
                                                          built_parts):
    """plan.md regression lock: when the servo grommets sit aft of the
    hatch bay, the rear must hold ONE central hollow (the bay continued
    aft), never the two rectangular side galleries. Checked by POINT
    CLASSIFICATION of the built centre body - the builder's own flags
    are not consulted for the verdict, only for where to look."""
    from OCP.BRepClass3d import BRepClass3d_SolidClassifier
    from OCP.gp import gp_Pnt
    from OCP.TopAbs import TopAbs_IN, TopAbs_ON

    parts, meta = built_parts
    bm = meta.get("bay") or {}
    ext_mm = float(bm.get("cavity_extended_mm") or 0.0)
    con = (meta.get("servos") or {}).get("conduits") or {}
    galleries = [k for k, v in con.items() if isinstance(v, dict)
                 and str(v.get("entry_mode", "")).endswith("_gallery")]
    assert not galleries, (
        f"servo run(s) {galleries} fell back to the rejected box galleries "
        f"(cavity extended {ext_mm:.0f} mm)")
    if ext_mm < 12.0:
        pytest.skip("grommets inside the hatch span - no extension needed")
    cs = bm.get("cavity_stations_mm") or []
    assert len(cs) >= 4, "extended cavity published no station band"
    body = parts.get("centre_body") or parts.get("airframe")
    assert body is not None, sorted(parts)
    shp = body.wrapped

    def solid_at(x, y, z):
        c = BRepClass3d_SolidClassifier(shp, gp_Pnt(x, y, z), 1e-4)
        return c.State() in (TopAbs_IN, TopAbs_ON)

    x_a = float(bm.get("hatch_x1_mm") or cs[0][0])
    x_b = float(cs[-1][0])
    # the column must reach from below the keel to above the CROWN: a
    # fixed -40..40 window never saw the roof of the deep bwb (crown at
    # z=+59 at the probe station on the 1400 box) and called its cavity
    # "missing" (2026-08-28)
    from backend.cad.geometry import _wing_from_design
    wing = _wing_from_design(design["geometry"])

    def zscan(x):
        xc = wing.xc_at(0.0, x)
        if 0.0 <= xc <= 1.0:
            lo = float(wing.keel_z(0.0, xc)) - 6.0
            hi = float(wing.crown_z(0.0, xc)) + 6.0
        else:
            lo, hi = -40.0, 40.0
        return [lo + i for i in range(int(hi - lo) + 1)]
    # `cavity_stations_mm` rows are [x, HALF-width, z_lo, z_hi] (hatch
    # publishes `s.hw`); the pre-round-12 channel reported a full
    # `width_mm`, and halving the half-width put these probes INSIDE the
    # cavity, where its own air read as a "gallery remnant" on every
    # planform (2026-08-28).
    half_w = max(float(r[1]) for r in cs)
    # the servo tubes are level pipes at constant (x, z) crossing every y
    # between the pocket and the cavity wall - a probe column that lands
    # on one sees the tube's bore, not a gallery; mask those cells
    tubes = []
    for k, v in con.items():
        if isinstance(v, dict) and k.startswith("servo_") \
                and v.get("applied") and v.get("path_mm"):
            pm = v["path_mm"]
            tubes.append((float(pm[0][0]), float(pm[0][2]),
                          0.5 * float(v.get("w_mm") or 12.0) + 1.0,
                          0.5 * float(v.get("h_mm") or 8.25) + 1.0))
    # box-remnant probe (reviewer finding #2): forward of the grommet
    # station (fracs .35/.6 - the tubes only cross at the channel's aft
    # end) there must be NO enclosed air out at gallery y offsets
    for frac in (0.35, 0.6):
        x = x_a + frac * (x_b - x_a)
        for y_probe in (half_w + 10.0, -(half_w + 10.0),
                        half_w + 22.0, -(half_w + 22.0)):
            zs = zscan(x)
            col = [solid_at(x, y_probe, z) for z in zs]
            in_tube = [any(abs(x - tx) <= rw and abs(z - tz) <= rh
                           for tx, tz, rw, rh in tubes) for z in zs]
            air = [i for i, s in enumerate(col)
                   if not s and not in_tube[i]
                   and any(col[:i]) and any(col[i:])]
            assert len(air) < 4, (
                f"{design['planform']} x={x:.0f}, y={y_probe:.0f}: "
                f"enclosed air pocket ({len(air)} cells) beside the "
                "channel - a box gallery remnant")
    for frac in (0.35, 0.6, 0.85):
        x = x_a + frac * (x_b - x_a)
        # find the channel air at the centreline: scan z, demand an
        # enclosed air run (solid below AND above)
        zs = zscan(x)
        col = [solid_at(x, 0.0, z) for z in zs]
        air = [i for i, s in enumerate(col)
               if not s and any(col[:i]) and any(col[i:])]
        assert air, (
            f"{design['planform']} x={x:.0f}: no enclosed air on the "
            "centreline - the channel is missing here")
        z_mid = zs[air[len(air) // 2]]
        # the box signature: air out at +/-(half_w + 8) with SOLID at
        # the centreline would mean side pockets instead - already
        # excluded by the centreline air above; also demand the channel
        # actually spans sideways like a channel, not a pinhole. The
        # width is the LOCAL one: the extension tapers toward its end
        # (the bwb's 92 mm hatch half-width is 40-odd mm at frac 0.85),
        # so the widest station's figure is the wrong yardstick here.
        hw_x = float(np.interp(x, [float(r[0]) for r in cs],
                               [float(r[1]) for r in cs]))
        assert not solid_at(x, 0.8 * hw_x, z_mid) or \
            not solid_at(x, -0.8 * hw_x, z_mid) or hw_x < 9.0, (
            f"{design['planform']} x={x:.0f}: centreline air but solid "
            f"at +/-{0.8 * hw_x:.0f} mm - narrower than the recorded "
            f"channel (local half-width {hw_x:.0f}); the hollow is not "
            "the one reported")
