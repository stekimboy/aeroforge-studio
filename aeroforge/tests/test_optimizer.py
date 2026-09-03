"""Flying-wing optimizer + full-pipeline physics sanity.

Checks the physics-side invariants of SPEC_FLYING_WING.md section 8 for every
planform family and every variant character. Tests drive the PUBLIC path
(`GenerateRequest.to_optimizer_input()` -> `optimize_design`) so they stay
honest about the contract rather than about internal keyword names.
"""
import pytest

from backend.physics.config_defs import MISSIONS, PLANFORMS
from backend.physics.optimizer import optimize_design
from backend.physics.variants import VARIANTS, generate_variants
from backend.schemas import GenerateRequest

ALL_PLANFORMS = list(PLANFORMS)

BOX = {"length_mm": 900, "width_mm": 1400, "height_mm": 300}


def _input(**over):
    """Optimizer input built the way the API builds it."""
    body = dict(mission="sport", cruise_speed_ms=15.0, payload_g=0.0,
                box=dict(BOX), n_motors=1, build_method="3d_printed",
                material="lw_pla")
    body.update(over)
    return GenerateRequest(**body).to_optimizer_input()


@pytest.fixture(scope="module")
def designs():
    return {p: optimize_design(_input(planform=p)) for p in ALL_PLANFORMS}


@pytest.fixture(scope="module")
def variant_set():
    return generate_variants(_input(planform=ALL_PLANFORMS[0]))


# ---------------------------------------------------------------------------
# The product is flying wings ONLY
# ---------------------------------------------------------------------------

def test_only_flying_wing_planforms_exist():
    assert set(PLANFORMS) == {"swept", "bwb", "plank", "bell"}
    assert set(MISSIONS) == {"sport", "fpv_cruiser", "thermal_floater",
                             "park_flyer"}


def test_no_tailed_aircraft_concepts_survive(designs):
    """Tails, canards, V-tails and booms are gone from the product - not
    hidden behind a flag, gone. A stray key means a code path survived."""
    banned = ("htail", "canard", "boom", "ruddervator", "elevator",
              "twin_boom", "t_tail")
    for name, d in designs.items():
        blob = repr(d).lower()
        for word in banned:
            assert word not in blob, f"{name}: '{word}' survives in the design"


def test_geometry_has_a_blended_body_not_a_fuselage(designs):
    """The centre body IS the wing (spec section 5), so there is no separate
    fuselage block in the geometry any more."""
    for name, d in designs.items():
        g = d["geometry"]
        assert "body" in g, name
        assert "fuselage" not in g, f"{name}: still carries a fuselage block"


def test_every_design_uses_a_reflexed_section(designs):
    """Spec 8.5: a tailless wing trims on positive Cm0 - a cambered non-reflex
    section cannot hold the nose up without a tail."""
    from backend.physics.airfoils import LIBRARY
    for name, d in designs.items():
        af = d["geometry"]["airfoil"]
        assert af in LIBRARY, f"{name}: unknown section {af!r}"
        assert LIBRARY[af].reflexed, f"{name}: {af} is not reflexed"


# ---------------------------------------------------------------------------
# Safety invariants (spec 8.3, 8.4)
# ---------------------------------------------------------------------------

def test_cg_ahead_of_np_and_margin_in_tailless_band(designs):
    for name, d in designs.items():
        st = d["stability"]
        assert st["x_cg_m"] < st["x_np_m"], name
        assert 0.03 <= st["static_margin"] <= 0.15, (
            f"{name}: SM {st['static_margin']:.3f} outside the tailless band")
        assert st["dcm_dalpha"] < 0, name


def test_stall_margin_and_lift_equals_weight(designs):
    from backend.physics.atmosphere import isa
    rho = isa(0.0).density_kgm3
    for name, d in designs.items():
        a, g = d["aero"], d["geometry"]
        sf = MISSIONS[d["mission"]].stall_factor
        assert a["v_cruise_ms"] >= sf * a["v_stall_ms"] * 0.999, name
        lift = 0.5 * rho * a["v_cruise_ms"] ** 2 * g["area_m2"] * a["cl_cruise"]
        assert lift == pytest.approx(d["mass"]["weight_n"], rel=1e-6), name


def test_fits_the_user_box(designs):
    for name, d in designs.items():
        g = d["geometry"]
        assert g["span_m"] <= BOX["width_mm"] / 1000 + 1e-6, name
        assert g["length_total_m"] <= BOX["length_mm"] / 1000 + 1e-6, name
        assert g["height_total_m"] <= BOX["height_mm"] / 1000 + 1e-6, name


# ---------------------------------------------------------------------------
# Proportion realism (spec 8.6) - "if it looks right, it flies right"
# ---------------------------------------------------------------------------

def test_wings_have_real_rc_proportions(designs):
    """The old generator produced AR 5-6 with a root chord ~0.20 x span, then
    bolted a pod on because the root was too thin to hold anything. Real
    FPV/sport wings are short and deep: Skywalker X5 is 1280 mm span with a
    717 mm root (0.56); SonicModell AR Wing is 900/482 (0.54)."""
    for name in ("swept", "bwb"):
        g = designs[name]["geometry"]
        frac = g["root_chord_m"] / g["span_m"]
        assert frac >= 0.30, (
            f"{name}: root chord is only {frac:.2f} of span - that is a "
            "glider planform, not a flying wing")
        assert 3.0 <= g["aspect_ratio"] <= 6.0, name


def test_centre_body_is_a_real_body(designs):
    """Spec 8.6: the body IS the wing, and it has to be deep enough to hold
    the pack or the design is fiction."""
    for name, d in designs.items():
        body = d["geometry"]["body"]
        assert body["chord_scale"] >= 1.10, name
        assert 1.5 <= body["depth_scale"] <= 3.5, name
        assert body["half_width_m"] > 0, name
        depth = (body["depth_scale"] * d["geometry"]["root_chord_m"]
                 * _thickness_ratio(d["geometry"]["airfoil"]))
        assert depth >= 0.028, (
            f"{name}: centre body only {depth * 1000:.0f} mm deep - nothing "
            "fits inside it")


def _thickness_ratio(airfoil_name):
    from backend.physics.airfoils import LIBRARY
    return LIBRARY[airfoil_name].thickness


def test_vertical_surfaces_are_flying_wing_sized(designs):
    """Spec 8.7. A tailless fin sits ~one root chord behind the CG, so sizing
    it from the tail-aft V_V band demands a sail (the old code produced 24% of
    wing area). Real wings carry 3-7%."""
    for name, d in designs.items():
        g = d["geometry"]
        vs = g["vstab"]
        if vs["type"] == "none":
            assert vs["area_total_m2"] == 0.0, name
            assert vs["count"] == 0, name
            continue
        frac = vs["area_total_m2"] / g["area_m2"]
        assert 0.02 <= frac <= 0.09, (
            f"{name}: vertical surfaces are {frac * 100:.1f}% of wing area")
        assert vs["height_m"] > 0, name
        # a surface taller than a third of the semi-span is a sail, whatever
        # the area bookkeeping says
        assert vs["height_m"] <= 0.33 * g["span_m"] / 2, (
            f"{name}: {vs['type']} is {vs['height_m'] * 1000:.0f} mm tall on a "
            f"{g['span_m'] * 1000:.0f} mm span")
        if vs["type"] == "center_fin":
            assert vs["height_m"] <= 0.60 * g["root_chord_m"], name


def test_a_single_centre_fin_is_offered_on_every_planform_that_can_take_one():
    """A single vertical stabilizer on the spine is a real arrangement on any
    wing that is allowed fins at all - it sits on a shorter arm than tip fins
    so it is sized larger, but it keeps the tips clean and survives a
    cartwheel. Only the bell spanload refuses fins, and for its own reason."""
    for name, pf in PLANFORMS.items():
        if pf.bell_spanload:
            assert pf.vstab_options == ("none",), name
        else:
            assert "center_fin" in pf.vstab_options, (
                f"{name} offers {pf.vstab_options} - no single centre fin")


@pytest.mark.parametrize("planform", ["swept", "bwb", "plank"])
def test_centre_fin_designs_are_sized_and_feasible(planform):
    d = optimize_design(_input(planform=planform, vstab="center_fin"))
    vs = d["geometry"]["vstab"]
    assert vs["type"] == "center_fin" and vs["count"] == 1
    frac = vs["area_total_m2"] / d["geometry"]["area_m2"]
    assert 0.02 <= frac <= 0.09, f"{planform}: fin is {frac * 100:.1f}% of wing"
    assert vs["height_m"] <= 0.60 * d["geometry"]["root_chord_m"], planform


def test_bell_spanload_has_no_fins_and_real_washout(designs):
    """The bell spanload's whole point is proverse yaw from tip-region induced
    thrust; it needs 8-13 deg of washout and NO vertical surfaces. Fins on a
    Horten defeat the configuration (NASA Prandtl-D)."""
    g = designs["bell"]["geometry"]
    assert g["vstab"]["type"] == "none"
    assert g["vstab"]["area_total_m2"] == 0.0
    assert 8.0 <= g["washout_deg"] <= 13.0, g["washout_deg"]
    assert designs["bell"]["stability"].get("bell_spanload") is True


def test_plank_is_actually_a_plank(designs):
    g = designs["plank"]["geometry"]
    assert g["sweep_le_deg"] <= 8.0
    assert g["taper"] >= 0.70


# ---------------------------------------------------------------------------
# Variants (spec 8.8)
# ---------------------------------------------------------------------------

def test_five_variants_always_returned(variant_set):
    assert len(variant_set) == len(VARIANTS) == 5
    for v in variant_set:
        assert v["id"] and v["name"] and v["design"]
        assert len(v["traits"]) >= 3


def test_variants_span_at_least_three_planform_families(variant_set):
    families = {v["design"]["planform"] for v in variant_set}
    assert len(families) >= 3, f"only {families}"


def test_every_variant_satisfies_the_safety_invariants(variant_set):
    for v in variant_set:
        d = v["design"]
        st, a, g = d["stability"], d["aero"], d["geometry"]
        assert st["x_cg_m"] < st["x_np_m"], v["key"]
        assert 0.03 <= st["static_margin"] <= 0.15, v["key"]
        assert st["dcm_dalpha"] < 0, v["key"]
        sf = MISSIONS[d["mission"]].stall_factor
        assert a["v_cruise_ms"] >= sf * a["v_stall_ms"] * 0.999, v["key"]
        assert g["span_m"] <= BOX["width_mm"] / 1000 + 1e-6, v["key"]
        assert g["length_total_m"] <= BOX["length_mm"] / 1000 + 1e-6, v["key"]
        assert g["height_total_m"] <= BOX["height_mm"] / 1000 + 1e-6, v["key"]


def test_variants_are_never_dropped_even_when_infeasible():
    vs = generate_variants(_input(
        planform="swept", cruise_speed_ms=28.0,
        box=dict(length_mm=260, width_mm=320, height_mm=110)))
    assert len(vs) == 5
    assert any(not v["design"]["feasible"] for v in vs)
    for v in vs:
        d = v["design"]
        assert d["binding"] if not d["feasible"] else True
        assert d["guidance"], v["key"]


def test_infeasible_designs_report_their_binding_constraints():
    """A heavy payload in a box with no room for the wing area to carry it:
    the wing loading and stall margin cannot both be met at any planform the
    box allows. (Note a SMALL box alone is not infeasible - a 300 mm wing at
    30 m/s is a perfectly real thing; it is the loading that breaks it.)"""
    d = optimize_design(_input(
        planform="bwb", cruise_speed_ms=12.0, payload_g=2000,
        box=dict(length_mm=300, width_mm=400, height_mm=120)))
    assert not d["feasible"]
    assert d["binding"]
    bad = [c for c in d["constraints"] if not c["ok"]]
    assert bad and all(c["message"] for c in bad)


# ---------------------------------------------------------------------------
# Sensitivity: the optimizer must respond to the mission, not ignore it
# ---------------------------------------------------------------------------

def test_faster_cruise_raises_wing_loading():
    slow = optimize_design(_input(planform="swept", cruise_speed_ms=11.0))
    fast = optimize_design(_input(planform="swept", cruise_speed_ms=22.0))
    assert (fast["aero"]["wing_loading_kgm2"]
            > slow["aero"]["wing_loading_kgm2"])


def test_ld_is_physically_plausible(designs):
    for name, d in designs.items():
        assert 5.0 <= d["aero"]["ld_cruise"] <= 24.0, name


# ---------------------------------------------------------------------------
# Handling qualities: the OTHER two axes
# ---------------------------------------------------------------------------

def test_every_design_is_directionally_and_laterally_stable(designs):
    """Longitudinal balance is not enough. A wing that trims perfectly can
    still hunt in yaw or roll further into its own sideslip."""
    for name, d in designs.items():
        st = d["stability"]
        assert st["cn_beta"] > 0, f"{name}: Cn_beta {st['cn_beta']:.4f} - hunts in yaw"
        assert st["cl_beta"] < 0, f"{name}: Cl_beta {st['cl_beta']:.4f} - spirally divergent"
        assert st["elevon_trim_used"] <= 0.75, name


def test_yaw_stiffness_is_split_between_fins_and_sweep(designs):
    """The split is the physics story: a plank's stiffness is nearly all fin,
    a finless bell's is all sweep."""
    plank, bell = designs["plank"]["stability"], designs["bell"]["stability"]
    assert plank["cn_beta_fin"] > 5 * plank["cn_beta_sweep"], (
        "an unswept plank's yaw stiffness must come from its fin")
    assert bell["cn_beta_fin"] == 0.0 and bell["cn_beta_sweep"] > 0, (
        "a finless bell wing's yaw stiffness can only come from sweep")


def test_bell_spanload_gives_proverse_yaw_and_the_others_do_not():
    """The reason a Horten needs no fin: elevons outboard of eta = 0.577 sit in
    upwash, so the down-going one LOSES drag and the nose swings into the turn
    (Prandtl 1933; NASA Prandtl-D). A conventional spanload does the opposite."""
    bell = optimize_design(_input(planform="bell"))
    swept = optimize_design(_input(planform="swept"))
    assert bell["stability"]["proverse_yaw"] > 0.02, "bell wing is not proverse"
    assert swept["stability"]["proverse_yaw"] < 0, "swept wing should be adverse"


def test_handling_notes_are_written_for_every_design(designs):
    for name, d in designs.items():
        notes = d["stability"]["handling_notes"]
        assert isinstance(notes, list) and notes, name
        for n in notes:
            assert isinstance(n, str) and len(n) > 30, name


def test_variant_tuning_falls_back_to_the_untuned_feasible_planform():
    """A character's tuning (fixed taper, narrowed AR band) can be what makes
    it infeasible in a small box while the same planform, untuned, fits.
    Measured on the user's 650 x 650 x 500 fpv box: the Swept Sport at
    taper 0.45 / AR >= 3.7 missed the SG90 servo floor and the UI showed a
    "closest feasible" wing, although the bare swept optimum is feasible
    with the servo buried. `generate_variants` now ships the feasible
    untuned solution for that character, with a note (task 2, 2026-08-28)."""
    from backend.physics import variants as _variants

    inp = _input(mission="fpv_cruiser", cruise_speed_ms=18.0,
                 box={"length_mm": 650, "width_mm": 650, "height_mm": 500})
    vs = _variants.generate_variants(inp)
    primary = next(v for v in vs if v.get("primary"))
    d = primary["design"]
    assert d["planform"] == "swept"
    assert d["feasible"], [c for c in d["constraints"] if not c["ok"]]
    assert all(c["ok"] for c in d["constraints"] if c["name"] == "servo_fit")
    assert any("untuned" in n for n in d["notes"])
