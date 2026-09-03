"""Physics-side invariants of the v2 CONVENTIONAL airplane type, plus the one
unforgivable regression: the flying-wing path must be bit-identical to v1.

Tests drive the PUBLIC path (`GenerateRequest.to_optimizer_input()` ->
`optimize_design` / `generate_variants`) exactly like test_optimizer.py, so
they stay honest about the contract. Bands cite RESEARCH_CONVENTIONAL.md.
"""
import pytest

from backend.physics.config_defs import (
    AIRPLANE_TYPES, CONV_LH_MAC_BAND, CONV_SM_BAND, CONV_STYLES,
    CONV_VH_BAND, CONV_VV_BAND, MISSIONS,
)
from backend.physics.optimizer import optimize_design
from backend.physics.variants import generate_variants
from backend.schemas import GenerateRequest

BOX = {"length_mm": 900, "width_mm": 1400, "height_mm": 300}


def _input(**over):
    """Optimizer input built the way the API builds it."""
    body = dict(airplane_type="conventional", mission="sport",
                cruise_speed_ms=15.0, payload_g=0.0, box=dict(BOX),
                n_motors=1, build_method="3d_printed", material="lw_pla")
    body.update(over)
    return GenerateRequest(**body).to_optimizer_input()


@pytest.fixture(scope="module")
def conv():
    """One conventional design per style, via the public dispatch."""
    out = {}
    for style in CONV_STYLES:
        inp = _input()
        inp["conv_style"] = style
        out[style] = optimize_design(inp)
    return out


@pytest.fixture(scope="module")
def conv_variants():
    return generate_variants(_input())


# ---------------------------------------------------------------------------
# Registry and schema
# ---------------------------------------------------------------------------

def test_airplane_type_registry():
    # v3 was eight families; biplane and glider were removed by the builder
    # (2026-08-21), leaving six
    assert set(AIRPLANE_TYPES) == {"flying_wing", "conventional", "delta",
                                   "canard", "tandem", "twin_boom"}
    for t in AIRPLANE_TYPES.values():
        assert t["label"] and t["description"]


def test_unknown_airplane_type_is_rejected():
    with pytest.raises(Exception):
        GenerateRequest(airplane_type="ornithopter", box=dict(BOX))


def test_removed_types_are_rejected():
    # removal means REMOVED: a request for either must fail exactly like any
    # other unknown type, not fall through to a dormant code path
    for gone in ("biplane", "glider"):
        with pytest.raises(Exception):
            GenerateRequest(airplane_type=gone, box=dict(BOX))


def test_design_dict_matches_v2_schema(conv):
    """V2_PLAN.md schema: airplane_type + geometry.fuselage/tail/ailerons."""
    for style, d in conv.items():
        assert d["airplane_type"] == "conventional", style
        g = d["geometry"]
        fus = g["fuselage"]
        for key in ("length_m", "width_m", "height_m", "x_wing_le_m", "bay"):
            assert key in fus, f"{style}: fuselage missing {key}"
        for key in ("bay_start_m", "bay_length_m", "bay_width_m",
                    "bay_wall_m"):
            assert key in fus["bay"], f"{style}: bay missing {key}"
        t = g["tail"]
        for key in ("arrangement", "x_le_h_m", "span_h_m", "c_root_h_m",
                    "c_tip_h_m", "area_h_m2", "V_H", "elevator_chord_frac",
                    "x_le_v_m", "height_v_m", "c_root_v_m", "c_tip_v_m",
                    "area_v_m2", "V_V", "rudder_chord_frac",
                    "incidence_h_deg"):
            assert key in t, f"{style}: tail missing {key}"
        for key in ("inner_frac", "outer_frac", "chord_frac"):
            assert key in g["ailerons"], style
        assert g["control_surfaces"] == ["aileron_left", "aileron_right",
                                         "elevator", "rudder"], style
        # single tractor motor on the nose; nothing ahead of the x = 0 datum
        assert len(g["motors"]) == 1 and g["motors"][0]["type"] == "tractor"
        assert 0.0 <= g["motors"][0]["x"] < 0.05, style


# ---------------------------------------------------------------------------
# Safety invariants (tailed bands)
# ---------------------------------------------------------------------------

def test_static_margin_in_tailed_band(conv):
    """[MIT] eq.(2): ideal SM +0.05..+0.15 - the TAILED band, roughly 2x the
    tailless trainer end (dossier s.3)."""
    for style, d in conv.items():
        st = d["stability"]
        assert st["x_cg_m"] < st["x_np_m"], style
        assert CONV_SM_BAND[0] - 1e-6 <= st["static_margin"] \
            <= CONV_SM_BAND[1] + 1e-6, (
            f"{style}: SM {st['static_margin']:.3f} outside the tailed band")
        assert st["dcm_dalpha"] < 0, style


def test_lift_equals_weight_and_stall_margin(conv):
    from backend.physics.atmosphere import isa
    rho = isa(0.0).density_kgm3
    for style, d in conv.items():
        a, g = d["aero"], d["geometry"]
        sf = MISSIONS[d["mission"]].stall_factor
        assert a["v_cruise_ms"] >= sf * a["v_stall_ms"] * 0.999, style
        lift = 0.5 * rho * a["v_cruise_ms"] ** 2 * g["area_m2"] * a["cl_cruise"]
        assert lift == pytest.approx(d["mass"]["weight_n"], rel=1e-6), style


def test_tail_volumes_and_arm_in_researched_bands(conv):
    """V_H 0.40-0.65, V_V 0.025-0.05, l_h = 2.5-3.5 x MAC
    (RESEARCH_CONVENTIONAL.md s.2: [MIT]/[RCG]/[SCHOLZ]/[ND])."""
    for style, d in conv.items():
        st = d["stability"]
        assert CONV_VH_BAND[0] - 1e-6 <= st["vh"] <= CONV_VH_BAND[1] + 1e-6, (
            f"{style}: V_H {st['vh']:.3f}")
        assert CONV_VV_BAND[0] - 1e-6 <= st["vv"] <= CONV_VV_BAND[1] + 1e-6, (
            f"{style}: V_V {st['vv']:.4f}")
        arm = st["l_h_m"] / st["mac_m"]
        assert CONV_LH_MAC_BAND[0] - 1e-6 <= arm \
            <= CONV_LH_MAC_BAND[1] + 1e-6, f"{style}: l_h/MAC {arm:.2f}"
        # the reported volumes match the built geometry
        g = d["geometry"]
        vh_geo = g["tail"]["area_h_m2"] * st["l_h_m"] / (
            g["area_m2"] * st["mac_m"])
        assert vh_geo == pytest.approx(st["vh"], rel=1e-6), style


def test_fits_the_user_box(conv):
    for style, d in conv.items():
        g = d["geometry"]
        assert g["span_m"] <= BOX["width_mm"] / 1000 + 1e-6, style
        assert g["length_total_m"] <= BOX["length_mm"] / 1000 + 1e-6, style
        assert g["height_total_m"] <= BOX["height_mm"] / 1000 + 1e-6, style


def test_fuselage_proportions_read_conventional(conv):
    """[ND]: fuselage ~3/4 span (real airframes 0.63-0.80); nothing ahead of
    the nose datum; the fuselage ends at the tail post."""
    for style, d in conv.items():
        g = d["geometry"]
        fus = g["fuselage"]
        frac = fus["length_m"] / g["span_m"]
        assert 0.60 <= frac <= 0.84, f"{style}: L_f/b {frac:.2f}"
        assert fus["x_wing_le_m"] > 0, style
        te = g["tail"]["x_le_h_m"] + g["tail"]["c_root_h_m"]
        assert te <= fus["length_m"] + 1e-6, style


def test_wing_uses_a_non_reflexed_section(conv):
    """The exact inverse of the flying-wing gate: the tail trims here, so the
    wing flies a plain cambered (or symmetric, aerobatic) section."""
    from backend.physics.airfoils import LIBRARY
    for style, d in conv.items():
        af = d["geometry"]["airfoil"]
        assert af in LIBRARY, style
        assert not LIBRARY[af].reflexed, f"{style}: {af} is reflexed"
        assert d["aero"]["cm0_section_geometric"] <= 1e-3, style
    # the cambered trainer section really is cambered (negative Cm_ac)
    assert conv["trainer"]["aero"]["cm0_section_geometric"] < -0.02


def test_control_surface_fractions_in_researched_bands(conv):
    """Dossier s.5: elevator 20-30% of stab, rudder 35-50% of fin, ailerons
    ~25% chord on the outboard panels."""
    for style, d in conv.items():
        t = d["geometry"]["tail"]
        ail = d["geometry"]["ailerons"]
        assert 0.20 <= t["elevator_chord_frac"] <= 0.30, style
        assert 0.35 <= t["rudder_chord_frac"] <= 0.50, style
        assert 0.20 <= ail["chord_frac"] <= 0.25, style
        assert ail["outer_frac"] <= 0.97 and ail["inner_frac"] >= 0.45, style


# ---------------------------------------------------------------------------
# Handling: TAILED thresholds, not the tailless calibration
# ---------------------------------------------------------------------------

def test_handling_with_tailed_thresholds(conv):
    for style, d in conv.items():
        st = d["stability"]
        # weathercock: a tailed model flies at Cn_beta 0.05-0.15 /rad - an
        # order of magnitude above the flying-wing numbers (handling.py note)
        assert st["cn_beta"] > 0.03, f"{style}: Cn_beta {st['cn_beta']:.4f}"
        assert st["cn_beta"] < 0.30, style          # sanity: per-radian scale
        assert st["cn_beta_fin"] > 0 > st["cn_beta_fus"], style
        assert st["cl_beta"] < 0, f"{style}: Cl_beta {st['cl_beta']:.4f}"
        assert st["elevator_trim_used"] <= 0.75, style
        assert st["handling_notes"], style


def test_trim_is_carried_by_the_stab_incidence(conv):
    """The stab incidence is set so the elevator is neutral in cruise
    (decalage, dossier s.4) - so there is no residual moment to hold."""
    for style, d in conv.items():
        st = d["stability"]
        tc = st["tail_contribution"]
        assert st["cm_trim_residual"] == 0.0, style
        assert -1.0 <= tc["decalage_deg"] <= 3.5, (
            f"{style}: decalage {tc['decalage_deg']:.1f} deg")
        assert 0.0 < tc["deps_dalpha"] < 0.7, style
        # the tail pushes the NP aft, the fuselage pulls it forward (Munk)
        assert tc["x_np_shift_tail_m"] > 0 > tc["x_np_shift_fus_m"], style


# ---------------------------------------------------------------------------
# Weights vs the research AUW bands
# ---------------------------------------------------------------------------

def test_mass_model_hits_the_1m_trainer_auw_band():
    """RESEARCH_CONVENTIONAL.md s.7: published 3D-printed conventionals at
    ~1000 mm span fly at 490-600 g LW-PLA / 730-900 g PLA (Eclipson Model A,
    3DLabPrint J-3 Cub). Checked at the Eclipson-like reference geometry:
    S = 0.167 m^2 (1 m span, AR 6), stab 17% / fin 7.6% of wing, 750 mm
    fuselage shell."""
    from backend.physics.weights import MATERIALS, estimate_mass_conventional
    ref = dict(wing_area=0.167, tail_area_h=0.028, tail_area_v=0.0127,
               fus_wetted_area=0.167, n_motors=1, n_servos=4, payload_kg=0.0)
    lw = estimate_mass_conventional(material=MATERIALS["lw_pla"], **ref)
    assert 0.49 <= lw.total <= 0.62, f"LW-PLA AUW {lw.total * 1000:.0f} g"
    # the lumped power system stays at the documented ~28% of AUW
    assert 0.24 <= lw.power_system / lw.total <= 0.32
    pla = estimate_mass_conventional(material=MATERIALS["pla"], **ref)
    assert 0.70 <= pla.total <= 0.95, f"PLA AUW {pla.total * 1000:.0f} g"


def test_optimized_trainer_auw_and_loading_are_plausible():
    """End to end: a trainer solved in a ~1 m box lands in the research AUW
    neighbourhood and the 25-60 g/dm^2 wing-loading band (dossier s.7)."""
    inp = _input(mission="fpv_cruiser",  # leads with the trainer style
                 cruise_speed_ms=13.0,
                 box=dict(length_mm=850, width_mm=1050, height_mm=300))
    d = optimize_design(inp)
    assert d["planform"] == "trainer"
    span = d["geometry"]["span_m"]
    assert 0.80 <= span <= 1.05
    # scale the 490-600 g @ 1 m band by span^2 (areal structure) with margin
    lo, hi = 0.42 * span**2, 0.72 * span**2
    auw = d["mass"]["total_kg"]
    assert lo <= auw <= hi, f"AUW {auw * 1000:.0f} g at {span * 1000:.0f} mm"
    assert 2.5 <= d["aero"]["wing_loading_kgm2"] <= 6.0


def test_ld_is_physically_plausible(conv):
    for style, d in conv.items():
        assert 5.0 <= d["aero"]["ld_cruise"] <= 24.0, style


# ---------------------------------------------------------------------------
# Variants and the "any" mode
# ---------------------------------------------------------------------------

def test_five_conventional_variants(conv_variants):
    assert len(conv_variants) == 5
    ids = set()
    for v in conv_variants:
        assert v["id"] and v["key"] and v["name"] and v["tagline"]
        assert v["design"]["airplane_type"] == "conventional"
        assert len(v["traits"]) >= 3
        assert v["design"]["guidance"], v["key"]
        ids.add(v["id"])
    assert len(ids) == 5
    assert sum(v["primary"] for v in conv_variants) == 1
    assert len({v["design"]["planform"] for v in conv_variants}) >= 3


def test_any_mode_spans_both_types():
    """v3: "any" competes ALL eight types and must display at least three
    (V3_PLAN.md spanning guarantee), every one a registered type."""
    vs = generate_variants(_input(airplane_type="any"))
    assert len(vs) == 5
    types = {v["design"]["airplane_type"] for v in vs}
    assert len(types) >= 3, types
    assert types <= set(AIRPLANE_TYPES), types
    assert sum(v["primary"] for v in vs) == 1
    for v in vs:
        assert v["airplane_type"] == v["design"]["airplane_type"]
        assert len(v["traits"]) >= 3


# ---------------------------------------------------------------------------
# THE regression: the flying-wing path is bit-identical to v1
# ---------------------------------------------------------------------------

def _strip_id(d: dict) -> dict:
    d = dict(d)
    d.pop("id", None)
    return d


def test_flying_wing_generate_is_unchanged_by_the_dispatch():
    """Same fixed inputs through the public path, once with the v2 default
    airplane_type and once with a v1-style input carrying no such key at all:
    the two design dicts must be EQUAL (ids aside - they are fresh uuids),
    and the flying-wing dict must not have grown any v2 keys."""
    req = GenerateRequest(planform="swept", mission="sport",
                          cruise_speed_ms=15.0, box=dict(BOX))
    inp = req.to_optimizer_input()
    assert inp["airplane_type"] == "flying_wing"    # the schema default
    d_v2 = optimize_design(inp)
    inp_v1 = {k: v for k, v in inp.items() if k != "airplane_type"}
    d_v1 = optimize_design(inp_v1)                  # the untouched v1 path
    assert _strip_id(d_v2) == _strip_id(d_v1)
    # no new required keys on a flying-wing design (V2_PLAN.md)
    assert "airplane_type" not in d_v2
    assert "fuselage" not in d_v2["geometry"]
    assert "tail" not in d_v2["geometry"]
    assert "ailerons" not in d_v2["geometry"]
    assert "tail_contribution" not in d_v2["stability"]


def test_flying_wing_variants_are_unchanged_by_the_dispatch():
    """generate_variants with the default airplane_type returns the same five
    flying-wing characters as a v1-style input (key fields compared - the
    solve is deterministic, only the ids are fresh)."""
    req = GenerateRequest(planform="plank", mission="park_flyer",
                          cruise_speed_ms=11.0, box=dict(BOX))
    inp = req.to_optimizer_input()
    inp_v1 = {k: v for k, v in inp.items() if k != "airplane_type"}
    vs_v2 = generate_variants(inp)
    vs_v1 = generate_variants(inp_v1)
    assert [v["key"] for v in vs_v2] == [v["key"] for v in vs_v1]
    for a, b in zip(vs_v2, vs_v1):
        assert a["primary"] == b["primary"]
        assert a["traits"] == b["traits"]
        ga, gb = a["design"]["geometry"], b["design"]["geometry"]
        assert ga == gb, a["key"]
        assert (a["design"]["stability"] == b["design"]["stability"]), a["key"]
        assert a["design"]["mass"] == b["design"]["mass"], a["key"]
        assert "airplane_type" not in a["design"], a["key"]
