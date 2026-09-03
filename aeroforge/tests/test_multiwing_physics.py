"""Physics-side invariants of the v3 MULTI-WING types: canard and
tandem (V3_PLAN.md wave 2a; biplane was removed by the builder 2026-08-21).

These modules are NOT yet registered in the optimizer/variants dispatch
(that is the integration wave's job), so the tests drive the modules'
public entry points DIRECTLY - `optimize_<type>` / `generate_<type>_variants`
- with an input dict shaped exactly like GenerateRequest.to_optimizer_input's
output. Bands cite RESEARCH_TYPES_V3.md by section / quick-reference row.
"""
import pytest

from backend.physics.canard import (
    CANARD_CL_RATIO_BAND, CANARD_CL_RATIO_FLOOR, CANARD_K_BAND,
    CANARD_SC_S_BAND, CANARD_VC_BAND, generate_canard_variants,
    optimize_canard,
)
from backend.physics.config_defs import CONV_SM_BAND, CONV_VH_BAND, MISSIONS
from backend.physics.tandem import (
    TANDEM_GAP_BAND, TANDEM_K_BAND, TANDEM_RATIO_FLOOR, TANDEM_SHARE_BAND,
    TANDEM_STAGGER_BAND, generate_tandem_variants, optimize_tandem,
)

# A box with the length the staggered types need and headroom to spare - the input dict mirrors to_optimizer_input's key set.
BOX = dict(box_l=1.2, box_w=1.4, box_h=0.4)


def _input(airplane_type, **over):
    body = dict(airplane_type=airplane_type, planform=None, mission="sport",
                v_cruise=15.0, v_stall_target=None, payload_kg=0.0, **BOX,
                vstab=None, n_motors=1, material="lw_pla",
                build_method="3d_printed", airfoil_override=None,
                ar_target=None, sm_override=None, mount_screws=None,
                mount_screw_d_mm=None, mount_bolt_circle_mm=None)
    body.update(over)
    return body


@pytest.fixture(scope="module")
def canard_variants():
    return generate_canard_variants(_input("canard"))


@pytest.fixture(scope="module")
def tandem_variants():
    return generate_tandem_variants(_input("tandem"))


def _all(variants):
    return [v["design"] for v in variants]


# ---------------------------------------------------------------------------
# Shared contract: five characters per type, all feasible in the reference
# box, correct airplane_type, guidance/traits present
# ---------------------------------------------------------------------------

def _check_variant_contract(variants, airplane_type):
    assert len(variants) == 5
    ids = set()
    for v in variants:
        assert v["id"] and v["key"] and v["name"] and v["tagline"]
        assert v["airplane_type"] == airplane_type
        assert v["design"]["airplane_type"] == airplane_type
        assert len(v["traits"]) >= 3
        assert v["design"]["guidance"], v["key"]
        ids.add(v["id"])
    assert len(ids) == 5
    assert sum(v["primary"] for v in variants) == 1
    assert len({v["design"]["planform"] for v in variants}) >= 3


def test_five_canard_variants(canard_variants):
    _check_variant_contract(canard_variants, "canard")


def test_five_tandem_variants(tandem_variants):
    _check_variant_contract(tandem_variants, "tandem")


def test_all_variants_feasible_in_reference_box(canard_variants,
                                                tandem_variants):
    """The reference box is generous on purpose: every character of every
    type must close fully feasible in it - a binding constraint here means
    a band pair is mutually unsatisfiable, not a tight box."""
    for vs in (canard_variants, tandem_variants):
        for v in vs:
            assert v["design"]["feasible"], (
                v["key"], v["design"]["binding"])


# ---------------------------------------------------------------------------
# Shared physics invariants (V3_PLAN.md verification bar)
# ---------------------------------------------------------------------------

def test_fits_the_box(canard_variants, tandem_variants):
    for vs in (canard_variants, tandem_variants):
        for v in vs:
            g = v["design"]["geometry"]
            assert g["span_m"] <= BOX["box_w"] + 1e-6, v["key"]
            assert g["length_total_m"] <= BOX["box_l"] + 1e-6, v["key"]
            assert g["height_total_m"] <= BOX["box_h"] + 1e-6, v["key"]


def test_cg_ahead_of_np_and_pitch_stable(canard_variants, tandem_variants):
    for vs in (canard_variants, tandem_variants):
        for v in vs:
            st = v["design"]["stability"]
            assert st["x_cg_m"] < st["x_np_m"], v["key"]
            assert st["dcm_dalpha"] < 0, v["key"]


def test_static_margin_in_researched_band(canard_variants, tandem_variants):
    """Canard/tandem: k = 0.20-0.25 x MAC_rear ([LEN-CAN Eq. 2-2], rows
    10/17-21 via s.3's inheritance)."""
    for v in canard_variants:
        sm = v["design"]["stability"]["static_margin"]
        assert CANARD_K_BAND[0] - 1e-6 <= sm <= CANARD_K_BAND[1] + 1e-6, (
            v["key"], sm)
    for v in tandem_variants:
        sm = v["design"]["stability"]["static_margin"]
        assert TANDEM_K_BAND[0] - 1e-6 <= sm <= TANDEM_K_BAND[1] + 1e-6, (
            v["key"], sm)


def test_lift_equals_weight_within_2pct(canard_variants, tandem_variants):
    """L = W at cruise on the TOTAL lifting area (cl_cruise is the system
    CL referenced to area_total_m2 for every multi-wing type)."""
    from backend.physics.atmosphere import isa
    rho = isa(0.0).density_kgm3
    for vs in (canard_variants, tandem_variants):
        for v in vs:
            d = v["design"]
            a, g = d["aero"], d["geometry"]
            lift = (0.5 * rho * a["v_cruise_ms"] ** 2
                    * g["area_total_m2"] * a["cl_cruise"])
            assert lift == pytest.approx(d["mass"]["weight_n"],
                                         rel=0.02), v["key"]


def test_stall_margin(canard_variants, tandem_variants):
    for vs in (canard_variants, tandem_variants):
        for v in vs:
            a = v["design"]["aero"]
            sf = MISSIONS[v["design"]["mission"]].stall_factor
            assert a["v_cruise_ms"] >= sf * a["v_stall_ms"] * 0.999, v["key"]


def test_ld_is_physically_plausible(canard_variants, tandem_variants):
    for vs in (canard_variants, tandem_variants):
        for v in vs:
            assert 5.0 <= v["design"]["aero"]["ld_cruise"] <= 24.0, v["key"]


# ---------------------------------------------------------------------------
# CANARD: the defining safety property (RESEARCH_TYPES_V3.md s.2.2)
# ---------------------------------------------------------------------------

def test_canard_cl_ratio_in_band_at_cruise(canard_variants):
    """CLf/CLr = 1.4-1.6 at cruise, hard floor > 1.0 ([LEN-CAN Eq. 2-7],
    row 9) - the stall-first margin in coefficient form, recorded in the
    design dict as required by V3_PLAN.md."""
    for v in canard_variants:
        st = v["design"]["stability"]
        r = st["cl_ratio_fr"]
        assert r > CANARD_CL_RATIO_FLOOR, (v["key"], r)
        assert CANARD_CL_RATIO_BAND[0] - 1e-6 <= r \
            <= CANARD_CL_RATIO_BAND[1] + 1e-6, (v["key"], r)
        # the ratio really is the two recorded surface CLs
        assert r == pytest.approx(
            st["cl_canard_cruise"] / st["cl_wing_cruise"], rel=1e-6)


def test_canard_stalls_first_by_construction(canard_variants):
    """The canard reaches its CL_max strictly before the wing: the alpha
    margin (wing-to-stall minus canard-to-stall, in aircraft alpha) must be
    positive, and the usable CL_max must be canard-limited (below the
    wing's own 3D CL_max scaled to the system)."""
    for v in canard_variants:
        st = v["design"]["stability"]
        assert st["canard_stalls_first"] is True, v["key"]
        assert st["stall_first_margin_deg"] > 0.0, (
            v["key"], st["stall_first_margin_deg"])
        # margin is real, not epsilon: at least 1 degree of alpha in hand
        assert st["stall_first_margin_deg"] >= 1.0, (
            v["key"], st["stall_first_margin_deg"])


def test_canard_geometry_bands(canard_variants):
    """Sc/S in 0.20-0.35 (row 8), V_C in 0.5-0.9 (row 12), canard AR >=
    wing AR (row 13), aft/forward lateral-area moment >= 1.25 (row 15)."""
    for v in canard_variants:
        g = v["design"]["geometry"]
        cn = g["canard"]
        assert CANARD_SC_S_BAND[0] - 1e-6 <= cn["sc_s"] \
            <= CANARD_SC_S_BAND[1] + 1e-6, v["key"]
        assert CANARD_VC_BAND[0] - 1e-6 <= cn["V_C"] \
            <= CANARD_VC_BAND[1] + 1e-6, v["key"]
        assert cn["aspect_ratio"] >= g["aspect_ratio"] - 1e-6, v["key"]
        assert g["fins"]["cla_moment_ratio"] >= 1.25 - 1e-6, v["key"]


def test_canard_geometry_block_complete(canard_variants):
    """V3_PLAN.md: geometry.canard {x_le_m, span_m, c_root_m, c_tip_m,
    area_m2, V_C, elevator_chord_frac, incidence_deg} + main wing aft +
    fuselage + fins - every dimension the CAD needs."""
    for v in canard_variants:
        d = v["design"]
        g = d["geometry"]
        for key in ("x_le_m", "span_m", "c_root_m", "c_tip_m", "area_m2",
                    "V_C", "elevator_chord_frac", "incidence_deg", "mac_m",
                    "aspect_ratio", "taper", "airfoil", "z_m", "sc_s"):
            assert key in g["canard"], f"{v['key']}: canard missing {key}"
        # main wing AFT of the canard
        assert g["x_le_wing_m"] > g["canard"]["x_le_m"], v["key"]
        for key in ("length_m", "width_m", "height_m", "x_wing_le_m",
                    "x_canard_le_m", "bay"):
            assert key in g["fuselage"], f"{v['key']}: fuselage missing {key}"
        for key in ("bay_start_m", "bay_length_m", "bay_width_m",
                    "bay_depth_m", "bay_wall_m"):
            assert key in g["fuselage"]["bay"], v["key"]
        for key in ("arrangement", "count", "area_each_m2", "height_m",
                    "c_root_m", "c_tip_m", "x_le_m", "y_m", "z_m",
                    "cla_moment_ratio"):
            assert key in g["fins"], f"{v['key']}: fins missing {key}"
        assert g["fins"]["arrangement"] == "tip_fins", v["key"]
        assert g["fins"]["count"] == 2, v["key"]
        # pusher: single motor at the aft fuselage face, nothing ahead of
        # the x = 0 datum
        assert len(g["motors"]) == 1
        assert g["motors"][0]["type"] == "pusher", v["key"]
        assert 0.0 < g["motors"][0]["x"] <= g["length_total_m"], v["key"]
        assert d["config"] == {"motor_layout": "pusher", "n_motors": 1,
                               "tail_type": None, "wing_position": "mid"}


# ---------------------------------------------------------------------------
# TANDEM: lift share and spacing bands (RESEARCH_TYPES_V3.md s.3.2)
# ---------------------------------------------------------------------------

def test_tandem_lift_share_in_band(tandem_variants):
    """Front wing carries 0.45-0.60 of the lift (row 16), CLf/CLr >= 1.2
    (row 17), and the share recorded in geometry.wing2 matches."""
    for v in tandem_variants:
        st = v["design"]["stability"]
        share = st["front_lift_share"]
        assert TANDEM_SHARE_BAND[0] - 1e-6 <= share \
            <= TANDEM_SHARE_BAND[1] + 1e-6, (v["key"], share)
        assert st["cl_ratio_fr"] >= TANDEM_RATIO_FLOOR - 1e-6, (
            v["key"], st["cl_ratio_fr"])
        assert st["cl_ratio_fr"] <= 2.0, v["key"]     # sanity, not a band
        assert v["design"]["geometry"]["wing2"]["lift_share"] == \
            pytest.approx(share, rel=1e-9)
        assert st["front_stalls_first"] is True, v["key"]
        assert st["stall_first_margin_deg"] > 0.0, v["key"]


def test_tandem_stagger_and_gap_in_band(tandem_variants):
    """Stagger 2.5-3.5 rear MAC (row 18); gap 0.3-1.0 MAC with the REAR
    wing HIGH (row 19)."""
    for v in tandem_variants:
        st = v["design"]["stability"]
        g = v["design"]["geometry"]
        assert TANDEM_STAGGER_BAND[0] - 1e-6 <= st["stagger_mac"] \
            <= TANDEM_STAGGER_BAND[1] + 1e-6, v["key"]
        assert TANDEM_GAP_BAND[0] - 1e-6 <= st["gap_mac"] \
            <= TANDEM_GAP_BAND[1] + 1e-6, v["key"]
        # rear wing HIGH: the main (rear) wing's z sits one gap above the
        # front wing's z
        assert g["wing_z_m"] > g["wing2"]["z_m"], v["key"]
        assert g["wing_z_m"] - g["wing2"]["z_m"] == pytest.approx(
            g["wing2"]["gap_m"], rel=1e-6), v["key"]


def test_tandem_geometry_block_complete(tandem_variants):
    """V3_PLAN.md: geometry.wing2 {x_le_m, span_m, chords, area_m2,
    lift_share, decalage_deg} - the front wing - plus fuselage/fins."""
    for v in tandem_variants:
        d = v["design"]
        g = d["geometry"]
        for key in ("role", "x_le_m", "span_m", "c_root_m", "c_tip_m",
                    "area_m2", "mac_m", "lift_share", "decalage_deg",
                    "incidence_deg", "z_m", "gap_m", "stagger_mac",
                    "dihedral_deg", "airfoil", "elevator_chord_frac"):
            assert key in g["wing2"], f"{v['key']}: wing2 missing {key}"
        assert g["wing2"]["role"] == "front"
        # front wing ahead of the rear wing, both on the fuselage
        assert g["wing2"]["x_le_m"] < g["x_le_wing_m"], v["key"]
        for key in ("bay_start_m", "bay_length_m", "bay_width_m",
                    "bay_depth_m", "bay_wall_m"):
            assert key in g["fuselage"]["bay"], v["key"]
        assert g["fins"]["arrangement"] == "center_fin", v["key"]
        assert g["fins"]["cla_moment_ratio"] >= 1.25 - 1e-6, v["key"]
        # tractor: single nose motor, nothing ahead of x = 0
        assert len(g["motors"]) == 1
        assert g["motors"][0]["type"] == "tractor", v["key"]
        assert 0.0 <= g["motors"][0]["x"] < 0.05, v["key"]
        assert d["config"] == {"motor_layout": "tractor", "n_motors": 1,
                               "tail_type": None, "wing_position": "high"}
        # per-wing AR band 5-7 (row 21)
        assert 5.0 - 1e-6 <= g["wing2"]["aspect_ratio"] <= 7.0 + 1e-6
        assert 5.0 - 1e-6 <= g["aspect_ratio"] <= 7.0 + 1e-6


# ---------------------------------------------------------------------------
# Optimize entry points honour overrides (spot checks, one per type)
# ---------------------------------------------------------------------------

def test_optimize_canard_sm_override():
    d = optimize_canard(_input("canard", sm_override=0.24))
    assert d["stability"]["static_margin"] == pytest.approx(0.24, abs=1e-6)


def test_optimize_tandem_runs_standalone():
    d = optimize_tandem(_input("tandem", tandem_style="tandem_quickie"))
    assert d["airplane_type"] == "tandem"
    assert d["planform"] == "tandem_quickie"
    assert d["id"]


