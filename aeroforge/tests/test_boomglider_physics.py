"""Physics-side invariants of the v3 Wave-2b types - TWIN-BOOM and DELTA
(the GLIDER was removed by the builder 2026-08-21) - plus the config_axes transforms, and the one unforgivable regression:
the flying-wing planform registry must stay untouched at rest (delta is
registered only transiently inside a solve - see physics/delta.py docstring).

Bands cite RESEARCH_TYPES_V3.md by its section tags ([RT3 s.N] / quick-
reference rows). Structure mirrors tests/test_conventional_physics.py.
"""
import math

import pytest

from backend.physics import config_axes as CA
from backend.physics.config_defs import MISSIONS, PLANFORMS, SM_BAND
from backend.physics.delta import (
    DELTA_AR_BAND, DELTA_SM_TARGET_BAND, DELTA_SWEEP_BAND,
    generate_delta_variants,
)
from backend.physics.twinboom import (
    BOOM_SPACING_PROP_MULT_MIN, PROP_TIP_CLEARANCE_M, TB_AR_HARD,
    TB_LH_MAC_BAND, TB_VH_BAND, TB_VV_BAND, generate_twinboom_variants,
)


def _inp(**over):
    """Optimizer input shaped exactly like GenerateRequest.to_optimizer_input
    (the new types are not registered in the request schema yet - dispatch is
    the integration wave's job; the physics modules take the same dict)."""
    body = dict(mission="sport", v_cruise=15.0, v_stall_target=None,
                payload_kg=0.0, box_l=0.95, box_w=1.2, box_h=0.3,
                vstab="auto", n_motors=1, material="lw_pla",
                build_method="3d_printed", airfoil_override=None,
                ar_target=None, sm_override=None)
    body.update(over)
    return body


@pytest.fixture(scope="module")
def delta_variants():
    return generate_delta_variants(_inp())


@pytest.fixture(scope="module")
def tb_variants():
    return generate_twinboom_variants(_inp(
        mission="fpv_cruiser", v_cruise=16.0,
        box_l=1.3, box_w=1.8, box_h=0.4))


# ---------------------------------------------------------------------------
# Shared invariants (the v3 verification bar, per type)
# ---------------------------------------------------------------------------

def _check_five(variants, atype):
    assert len(variants) == 5
    ids = set()
    for v in variants:
        assert v["id"] and v["key"] and v["name"] and v["tagline"]
        assert v["airplane_type"] == atype
        assert v["design"]["airplane_type"] == atype
        assert len(v["traits"]) >= 3
        assert v["design"]["guidance"], v["key"]
        ids.add(v["id"])
    assert len(ids) == 5
    assert sum(v["primary"] for v in variants) == 1


def _check_physics(variants, box_l, box_w, box_h):
    """Box fit, CG ahead of NP, L = W (within 2% per the bar - it is exact by
    construction), speed margin - for every variant of the set."""
    from backend.physics.atmosphere import isa
    rho = isa(0.0).density_kgm3
    for v in variants:
        d = v["design"]
        assert d["feasible"], (v["key"], d["binding"])
        g, a, st = d["geometry"], d["aero"], d["stability"]
        assert g["span_m"] <= box_w + 1e-6, v["key"]
        assert g["length_total_m"] <= box_l + 1e-6, v["key"]
        assert g["height_total_m"] <= box_h + 1e-6, v["key"]
        assert st["x_cg_m"] < st["x_np_m"], v["key"]
        assert st["dcm_dalpha"] < 0, v["key"]
        lift = 0.5 * rho * a["v_cruise_ms"]**2 * g["area_m2"] * a["cl_cruise"]
        assert lift == pytest.approx(d["mass"]["weight_n"], rel=0.02), v["key"]
        sf = MISSIONS[d["mission"]].stall_factor
        assert a["v_cruise_ms"] >= sf * a["v_stall_ms"] * 0.999, v["key"]
        # nothing ahead of the nose datum
        for mo in g["motors"]:
            assert mo["x"] >= 0.0, v["key"]


# ---------------------------------------------------------------------------
# DELTA - through the REAL tailless optimizer ([RT3 s.1])
# ---------------------------------------------------------------------------

def test_delta_five_variants_through_tailless_machinery(delta_variants):
    _check_five(delta_variants, "delta")
    for v in delta_variants:
        d = v["design"]
        assert d["planform"] == "delta", v["key"]
        # evidence this really is the tailless machinery, not a copy: the
        # dict carries the flying-wing-only blocks and axes
        g, st = d["geometry"], d["stability"]
        assert "elevons" in g and "vstab" in g and "body" in g, v["key"]
        assert "proverse_yaw" in st and "elevon_trim_used" in st, v["key"]
        assert g["control_surfaces"] == ["elevon_left", "elevon_right"]


def test_delta_leaves_the_flying_wing_registry_untouched(delta_variants):
    """THE invariant of the delta implementation: PLANFORMS at rest is
    byte-identical to v2 (test_optimizer/test_api assert the same set; this
    proves generating deltas does not leak the transient registration)."""
    assert set(PLANFORMS) == {"swept", "bwb", "plank", "bell"}


def test_delta_physics_invariants(delta_variants):
    _check_physics(delta_variants, 0.95, 1.2, 0.3)


def test_delta_sm_in_tailless_band(delta_variants):
    """Tailless hard band 0.03-0.15; the untuned characters land in the
    [RT3 row 4] 0.06-0.12 target slice (delta CG practice 15-22% MAC is the
    same point expressed on the chord)."""
    for v in delta_variants:
        sm = v["design"]["stability"]["static_margin"]
        assert SM_BAND[0] - 1e-6 <= sm <= SM_BAND[1] + 1e-6, v["key"]
    lead = next(v for v in delta_variants if v["primary"])
    sm = lead["design"]["stability"]["static_margin"]
    assert DELTA_SM_TARGET_BAND[0] - 0.01 <= sm \
        <= DELTA_SM_TARGET_BAND[1] + 0.01, sm


def test_delta_sweep_and_ar_in_researched_bands(delta_variants):
    for v in delta_variants:
        g = v["design"]["geometry"]
        assert DELTA_SWEEP_BAND[0] - 1e-6 <= g["sweep_le_deg"] \
            <= DELTA_SWEEP_BAND[1] + 1e-6, v["key"]
        assert DELTA_AR_BAND[0] - 1e-6 <= g["aspect_ratio"] \
            <= DELTA_AR_BAND[1] + 0.05, (v["key"], g["aspect_ratio"])
        # the paper-dart proportion: root chord ~ half the span
        assert 0.40 <= g["root_chord_m"] / g["span_m"] <= 0.62, v["key"]


def test_delta_fin_area_in_band(delta_variants):
    """Every delta carries real vertical surface(s) - centre fin default,
    twin-fin (Stryker) legal, never none. Area: inside the machinery's own
    2-9% invariant with a >= 3% floor - the [MA-T] 4-9% band's floor is
    relaxed by the machinery's documented sweep-credit model (a 45-60 deg
    delta earns ~3.5-3.9%; see delta.py's DELTA_FIN_FRAC_BAND note)."""
    for v in delta_variants:
        g = v["design"]["geometry"]
        vs = g["vstab"]
        assert vs["type"] in ("center_fin", "twin_fin"), v["key"]
        assert vs["count"] >= 1, v["key"]
        frac = vs["area_total_m2"] / g["area_m2"]
        assert 0.03 <= frac <= 0.09, (v["key"], frac)


def test_delta_flies_a_reflexed_section_and_light_cruise_trim(delta_variants):
    """[RT3 s.1.2]: section Cm0 >= 0 (reflex preferred - the machinery
    requires reflex outright) and cruise trim <= 25% of the elevon throw."""
    for v in delta_variants:
        d = v["design"]
        assert d["aero"]["cm0_section_geometric"] > 0, v["key"]
        st = d["stability"]
        authority = st["elevon_authority_cm"]
        assert authority > 0, v["key"]
        trim_frac = abs(st["cm_trim_residual"]) / authority
        assert trim_frac <= 0.25, (v["key"], trim_frac)


# ---------------------------------------------------------------------------
# TWIN-BOOM ([RT3 s.5])
# ---------------------------------------------------------------------------

def test_twinboom_five_variants(tb_variants):
    _check_five(tb_variants, "twin_boom")


def test_twinboom_physics_invariants(tb_variants):
    _check_physics(tb_variants, 1.3, 1.8, 0.4)


def test_twinboom_boom_spacing_vs_prop(tb_variants):
    """[RT3 row 35]: spacing >= 1.15 x the estimated prop diameter AND
    >= D + 2 x 15 mm tip clearance; the estimate itself is recorded."""
    for v in tb_variants:
        b = v["design"]["geometry"]["booms"]
        prop_d = b["prop_diameter_est_m"]
        assert 0.15 <= prop_d <= 0.26, v["key"]     # 6-10 in class estimate
        assert b["spacing_m"] >= BOOM_SPACING_PROP_MULT_MIN * prop_d - 1e-9
        assert b["spacing_m"] >= prop_d + 2 * PROP_TIP_CLEARANCE_M - 1e-9
        assert b["spacing_over_prop_d"] == pytest.approx(
            b["spacing_m"] / prop_d)


def test_twinboom_tail_in_bands(tb_variants):
    """Tail arm 2.5-3.5 MAC (the booms' whole reason), V_H/V_V conventional
    bands with the vertical area split across two boom fins, stab spanning
    boom-to-boom ([RT3 rows 33-34])."""
    for v in tb_variants:
        d = v["design"]
        st, g = d["stability"], d["geometry"]
        arm = st["l_h_m"] / st["mac_m"]
        assert TB_LH_MAC_BAND[0] - 1e-6 <= arm <= TB_LH_MAC_BAND[1] + 1e-6, (
            v["key"], arm)
        assert TB_VH_BAND[0] - 1e-6 <= st["vh"] <= TB_VH_BAND[1] + 1e-6
        assert TB_VV_BAND[0] - 1e-6 <= st["vv"] <= TB_VV_BAND[1] + 1e-6
        t, b = g["tail"], g["booms"]
        assert t["arrangement"] == "h_tail_twin_boom"
        assert t["span_h_m"] == pytest.approx(b["spacing_m"])   # by constr.
        assert t["n_fins"] == 2
        assert t["fin_y_m"] == pytest.approx([-0.5 * b["spacing_m"],
                                              0.5 * b["spacing_m"]])
        assert TB_AR_HARD[0] - 1e-6 <= g["aspect_ratio"] \
            <= TB_AR_HARD[1] + 1e-6, v["key"]


def test_twinboom_geometry_blocks_for_cad(tb_variants):
    """V3_PLAN.md: geometry.booms {y_m, length_m, section_mm} + pod-fuselage
    block + pusher motor on the pod's aft face; boom stiffness proxy
    recorded and satisfied ([RT3 s.5.2])."""
    for v in tb_variants:
        g = v["design"]["geometry"]
        b = g["booms"]
        for key in ("y_m", "x_start_m", "length_m", "spacing_m",
                    "section_mm"):
            assert key in b, (v["key"], key)
        assert b["y_m"] == pytest.approx([-0.5 * b["spacing_m"],
                                          0.5 * b["spacing_m"]])
        sec = b["section_mm"]
        assert sec["tube_od_mm"] in (8.0, 10.0)
        assert sec["I_mm4"] >= sec["I_required_mm4"] - 1e-9, v["key"]
        # Ø8x1 reference: pi (8^4 - 6^4)/64 = 137.4 mm^4 per metre
        assert sec["I_required_mm4"] == pytest.approx(
            137.44467 * b["length_m"], rel=1e-3)
        fus = g["fuselage"]
        for key in ("length_m", "width_m", "height_m", "x_wing_le_m", "bay"):
            assert key in fus, (v["key"], key)
        # pod ~ 2 x MAC, ending at the wing root TE, motor pushing off it
        assert 1.5 <= fus["pod_len_mac"] <= 2.5, v["key"]
        mo = g["motors"]
        assert len(mo) == 1 and mo[0]["type"] == "pusher"
        assert mo[0]["x"] == pytest.approx(fus["length_m"])
        # the pusher prop spins between the booms, behind the pod
        assert fus["length_m"] < b["x_start_m"] + b["length_m"]


# ---------------------------------------------------------------------------
# The wing has to be able to bury its own aileron servo
# ---------------------------------------------------------------------------

def test_twinboom_servo_pocket_constants_match_the_cad():
    """physics/twinboom.py restates four of `cad/servos.py`'s numbers, because
    physics must not import backend.cad (cadquery would follow it into the
    server process - the whole reason paths.py exists). They must not drift:
    the chord floor is only as right as the pocket it is sized against.

    Same doctrine as tests/test_type_dispatch.py - a copy is allowed exactly
    when something asserts it is still a copy."""
    from backend.cad import servos as SV
    from backend.physics import twinboom as TB

    assert TB.SERVO_POCKET_DEPTH_MM == SV.SERVO_SG90["body_wid_mm"]
    assert TB.SERVO_POCKET_LEN_MM == SV.SERVO_SG90["ear_span_mm"]
    assert TB.SERVO_POCKET_CLEAR_MM == SV.SERVO_CLEARANCE_MM
    assert TB.SERVO_POCKET_SAFETY_MM == SV.ROOF_SAFETY_MM
    # the pocket's spanwise extent, as servo_bay computes it
    assert TB.SERVO_SPAN_MM == pytest.approx(
        SV.SERVO_SG90["total_h_mm"] + 2.0 * SV.SERVO_CLEARANCE_MM + 1.0)


def test_twinboom_wing_can_bury_its_aileron_servo(tb_variants):
    """Every twin-boom records what the aileron servo costs its planform, and
    the chord at the servo station clears the floor. Before this existed the
    `mapper` style shipped with no aileron servo pockets at all - the CAD
    refused them, said so in its warnings, and nothing upstream listened."""
    from backend.physics.twinboom import TB_AR_HARD

    for v in tb_variants:
        g = v["design"]["geometry"]
        sf = g.get("servo_chord_floor")
        assert sf is not None, v["key"]
        for key in ("chord_floor_m", "chord_at_servo_m", "station_frac",
                    "fits"):
            assert key in sf, (v["key"], key)
        # the floor may only be missed when the researched AR floor binds
        # first - never because nothing tried
        if not sf["fits"]:
            assert g["aspect_ratio"] <= TB_AR_HARD[0] + 1e-6, (
                f"{v['key']}: chord at the servo is "
                f"{sf['chord_at_servo_m'] * 1000:.1f} mm against a "
                f"{sf['chord_floor_m'] * 1000:.1f} mm floor, and AR "
                f"{g['aspect_ratio']:.2f} still has room to widen it")


def test_twinboom_chord_floor_is_monotonic_and_section_aware():
    """A cambered section holds a shallower STRAIGHT well than a symmetric one
    of the same thickness - the pocket is a box, so what it gets is
    min(crown) - max(keel) across its whole footprint, not peak thickness.
    The floor has to see that, or it under-sizes exactly the styles that fly
    NACA 4412."""
    from backend.physics.airfoils import naca4_coordinates
    from backend.physics.twinboom import (
        pocket_tilt_loss_mm, section_well_frac,
    )

    sym = naca4_coordinates("0012")
    cam = naca4_coordinates("4412")
    # a wider window can only make the well shallower
    for coords in (sym, cam):
        prev = 1.0
        for w in (0.10, 0.20, 0.30, 0.40):
            d = section_well_frac(coords, w)
            assert d <= prev + 1e-9
            prev = d
    # ... and camber costs depth at the same thickness
    assert section_well_frac(cam, 0.28) < section_well_frac(sym, 0.28)
    # tilt losses are first-order geometry and are not free
    assert pocket_tilt_loss_mm(0.0, 0.0) == pytest.approx(0.0)
    assert pocket_tilt_loss_mm(2.0, 1.2) > 1.5


# ---------------------------------------------------------------------------
# CONFIG AXES ([RT3 s.7-s.10])
# ---------------------------------------------------------------------------

def test_v_tail_equivalence_matches_the_research_rule():
    """[MA-T] rule: S = S_h + S_v at A = arctan sqrt(S_v/S_h); tan^2 split
    returns EXACTLY the required areas (the identity the rule is built on),
    NOT the sine/cosine projected areas - the documented undersizing trap."""
    s_h, s_v = 0.030, 0.012
    out = CA.apply_tail_type({"area_h_m2": s_h, "area_v_m2": s_v}, "v_tail")
    a_exp = math.degrees(math.atan(math.sqrt(s_v / s_h)))     # 32.31 deg
    assert out["dihedral_deg"] == pytest.approx(a_exp, abs=1e-9)
    assert 30.0 <= out["dihedral_deg"] <= 40.0                # [NACA823] cap
    assert out["area_total_m2"] == pytest.approx(s_h + s_v)
    assert out["area_panel_m2"] == pytest.approx(0.5 * (s_h + s_v))
    # the tan^2 split reproduces the required areas exactly
    assert out["area_h_eff_m2"] == pytest.approx(s_h, rel=1e-9)
    assert out["area_v_eff_m2"] == pytest.approx(s_v, rel=1e-9)
    # and is NOT the projected-area (sin/cos) split, which would undersize
    a = math.radians(out["dihedral_deg"])
    assert out["area_h_eff_m2"] < (s_h + s_v) * math.cos(a)
    assert any("projected" in n for n in out["notes"])
    assert out["ruddervator_throw_budget_deg"] == pytest.approx(25.0)


def test_v_tail_dihedral_clipped_into_band():
    """A ratio outside the 30-40 deg band is clipped ([NACA823] validity /
    RC practice) and both angles are recorded."""
    out = CA.apply_tail_type({"area_h_m2": 0.030, "area_v_m2": 0.003},
                             "v_tail")
    assert out["dihedral_deg"] == pytest.approx(30.0)
    assert out["dihedral_unclipped_deg"] < 30.0
    # effective areas are quoted at the CLIPPED angle
    s = out["area_total_m2"]
    a = math.radians(30.0)
    assert out["area_h_eff_m2"] == pytest.approx(s * math.cos(a) ** 2)
    assert out["area_v_eff_m2"] == pytest.approx(s * math.sin(a) ** 2)


def test_t_tail_and_conventional_transforms():
    tg = {"area_h_m2": 0.03, "area_v_m2": 0.012}
    t = CA.apply_tail_type(tg, "t_tail")
    assert t["fin_mass_factor"] == pytest.approx(1.4)        # [RT3 row 51]
    assert t["deep_stall_flag"] is True
    assert t["stab_z_mode"] == "fin_tip"
    assert any("8.25" in n for n in t["notes"])              # pipe doctrine
    c = CA.apply_tail_type(tg, "conventional")
    assert c["type"] == "conventional"
    assert c["area_h_m2"] == pytest.approx(0.03)             # pass-through
    with pytest.raises(ValueError):
        CA.apply_tail_type(tg, "cruciform")


def test_wing_position_ladder_returns_documented_values():
    """[RT3 rows 52-53]: geometric ladder high 1-2 / mid 2 / low 3 deg,
    effective increment +1.5 / 0 / -1.5 deg - summed effect ~constant."""
    high = CA.apply_wing_position("high")
    mid = CA.apply_wing_position("mid")
    low = CA.apply_wing_position("low")
    assert high["geometric_dihedral_deg"] == pytest.approx(1.5)
    assert high["geometric_dihedral_band_deg"] == [1.0, 2.0]
    assert high["effective_dihedral_increment_deg"] == pytest.approx(+1.5)
    assert mid["geometric_dihedral_deg"] == pytest.approx(2.0)
    assert mid["effective_dihedral_increment_deg"] == pytest.approx(0.0)
    assert low["geometric_dihedral_deg"] == pytest.approx(3.0)
    assert low["effective_dihedral_increment_deg"] == pytest.approx(-1.5)
    # the point of the ladder: net dihedral effect lands in the same place
    assert high["total_effective_dihedral_deg"] == pytest.approx(3.0)
    assert mid["total_effective_dihedral_deg"] == pytest.approx(2.0)
    assert low["total_effective_dihedral_deg"] == pytest.approx(1.5)
    # sweep cross-term recorded: 4 deg sweep ~ 1 deg dihedral at cruise CL
    assert high["sweep_equiv_deg_per_dihedral_deg"] == pytest.approx(4.0)
    with pytest.raises(ValueError):
        CA.apply_wing_position("shoulder")


def test_motor_layout_transforms():
    tr = CA.apply_motor_layout("tractor", 1)
    assert tr["efficiency_penalty"] == 0.0
    assert tr["thrustline"]["down_deg"] == pytest.approx(2.0)   # [ND] 2-3
    assert tr["thrustline"]["right_deg"] == pytest.approx(2.0)
    pod = CA.apply_motor_layout("pusher", 1, pusher_context="pod")
    assert pod["efficiency_penalty"] == pytest.approx(0.05)     # [RT3 row 45]
    tb = CA.apply_motor_layout("pusher", 1, pusher_context="twin_boom")
    assert tb["efficiency_penalty"] == pytest.approx(0.03)
    twin = CA.apply_motor_layout("tractor", 2)
    assert twin["nacelle_semispan_frac"] == pytest.approx(0.30)  # row 47
    assert twin["nacelle_semispan_frac_band"] == [0.25, 0.35]
    none = CA.apply_motor_layout("tractor", 0)
    assert none["thrustline"] is None
    with pytest.raises(ValueError):
        CA.apply_motor_layout("mid", 1)
    with pytest.raises(ValueError):
        CA.apply_motor_layout("tractor", 3)


def test_engine_out_check_math():
    """[RT3 row 48]: N_fin = q(1.2 Vs) S_v CL_v l_v vs one motor's static
    thrust ~ AUW/2 at the nacelle arm - both sides checked by hand."""
    kw = dict(mass_kg=2.0, v_stall_ms=8.0, rho=1.225, s_v_m2=0.03,
              l_v_m=0.6, span_m=1.8, nacelle_semispan_frac=0.30)
    out = CA.engine_out_check(**kw)
    v = 1.2 * 8.0
    n_fin = 0.5 * 1.225 * v * v * 0.03 * 0.8 * 0.6
    n_motor = 0.5 * 2.0 * 9.80665 * (0.30 * 0.9)
    assert out["check_speed_ms"] == pytest.approx(v)
    assert out["n_fin_nm"] == pytest.approx(n_fin)
    assert out["n_motor_nm"] == pytest.approx(n_motor)
    assert out["ok"] == (n_fin >= n_motor)
    assert out["margin"] == pytest.approx(n_fin / n_motor)
    # a big fin on a slow ship passes; a tiny fin on a heavy twin fails
    ok = CA.engine_out_check(**{**kw, "s_v_m2": 0.08, "mass_kg": 1.2})
    bad = CA.engine_out_check(**{**kw, "s_v_m2": 0.005, "mass_kg": 4.0})
    assert ok["ok"] is True
    assert bad["ok"] is False
    assert "differential" in bad["note"]
