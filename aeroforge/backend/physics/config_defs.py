"""Planform-family and mission definitions for the flying-wing specialist
(SPEC_FLYING_WING.md sections 3.1 / 3.2).

AeroForge designs FLYING WINGS ONLY. There is no aircraft-configuration
dropdown any more: what the user chooses is a PLANFORM FAMILY (how the tailless
wing is laid out) and a MISSION (what it is for).

Every band in this file is a measured range from real airframes, not a guess -
sources are quoted at the definition. The optimizer treats the bands as the
flyable envelope it searches inside; a design that leaves one is reported as
binding, never silently clamped.
"""
from __future__ import annotations

from dataclasses import dataclass

# ---------------------------------------------------------------------------
# Tailless static-margin band.
#
# A tailless wing flies at a LOWER static margin than a tailed model: with no
# horizontal tail, dCm/dalpha comes entirely from sweep + washout + section
# reflex, and the pitch damping that a tail arm provides is absent, so a large
# static margin buys a very sluggish, trim-drag-heavy aeroplane instead of a
# forgiving one. Published RC flying-wing practice (Zagi / Ritewing / Skywalker
# X5 class CG ranges) sits at 5-10% of MAC, with 3% the twitchy limit and 15%
# the point where the elevon reflex needed for trim starts to cost real drag.
# ---------------------------------------------------------------------------
SM_BAND: tuple[float, float] = (0.03, 0.15)
SM_DEFAULT_BAND: tuple[float, float] = (0.05, 0.10)


@dataclass(frozen=True)
class PlanformDef:
    """One tailless planform family."""
    name: str
    label: str
    description: str
    sweep_band: tuple[float, float]           # LE sweep of the outboard panel [deg]
    taper_band: tuple[float, float]           # lambda = c_tip / c_wing_root
    ar_band: tuple[float, float]              # aspect ratio b^2/S
    washout_band: tuple[float, float]         # root-to-tip geometric twist [deg]
    body_depth_band: tuple[float, float]      # centre-body thickness / wing thickness
    root_chord_frac_band: tuple[float, float]  # wing root chord / span
    oswald_mult: float                        # span-efficiency multiplier
    vstab_options: tuple[str, ...]            # first entry is the default
    bell_spanload: bool
    blurb: str


PLANFORMS: dict[str, PlanformDef] = {
    # Skywalker X5 Pro (1280 mm span, 717 mm root length, 44 dm^2, AR 3.7),
    # SonicModell AR Wing Classic 900 (900/482 mm, AR ~4.0), Reptile S800.
    # Root chord 0.45-0.58 x span on those airframes INCLUDING the centre
    # body's leading-edge root extension; the trapezoidal WING root chord
    # (which is what root_chord_frac_band measures) is the 0.30-0.45 that is
    # left once the body's chord_scale is divided back out.
    "swept": PlanformDef(
        name="swept", label="Swept sport wing",
        description=(
            "The classic RC flying wing: moderate leading-edge sweep, a short "
            "deep centre section that carries the gear, elevons on the "
            "outboard panels and small tip fins. Skywalker X5 / AR Wing class."
        ),
        sweep_band=(18.0, 32.0),
        taper_band=(0.35, 0.60),
        ar_band=(3.5, 5.2),
        washout_band=(2.0, 5.0),
        body_depth_band=(1.8, 2.6),
        root_chord_frac_band=(0.30, 0.45),
        # Flying wings lose span efficiency to the washout and the reflexed
        # section's off-design loading; 0.88 is the usual tailless multiplier
        # on the Raymer straight-wing estimate.
        oswald_mult=0.88,
        vstab_options=("winglets", "twin_fin", "center_fin"),
        bell_spanload=False,
        blurb="Sharp, fast, easy to launch - the shape most RC wings are.",
    ),
    # Ritewing Drak class blended wing body: less sweep, more taper ratio, a
    # deep payload body that is itself a lifting surface.
    "bwb": PlanformDef(
        name="bwb", label="Blended wing body",
        description=(
            "A long-range cruiser: the centre body is deep and wide enough to "
            "be a real payload bay, and it blends into the wing instead of "
            "sitting on it. Big canted winglets, gentle sweep, lots of volume."
        ),
        sweep_band=(16.0, 28.0),
        taper_band=(0.42, 0.68),
        ar_band=(3.8, 5.6),
        washout_band=(2.0, 4.0),
        body_depth_band=(2.2, 3.2),
        root_chord_frac_band=(0.30, 0.44),
        # The deep centre body carries lift over its own span, which fills in
        # the middle of the spanload - slightly better than a plain swept wing.
        oswald_mult=0.90,
        # A deep centre body is the one place a single fin has real structure
        # to bolt to, so a centre fin belongs here as much as on a plank -
        # there was never a physical reason to leave it out.
        vstab_options=("winglets", "twin_fin", "center_fin"),
        bell_spanload=False,
        blurb="Volume and range: the wing IS the fuselage.",
    ),
    # PW-51, Zagi-plank, Hepperle plank designs: no sweep at all, so the
    # reflexed section does 100% of the trimming and one centre fin does 100%
    # of the directional stability.
    "plank": PlanformDef(
        name="plank", label="Plank",
        description=(
            "No sweep, near-constant chord, strong section reflex doing all "
            "the trimming and one centre fin doing all the yaw damping. Looks "
            "like a rectangular door, which is exactly right - planks are the "
            "most benign and most efficient tailless layout at low speed."
        ),
        sweep_band=(0.0, 8.0),
        taper_band=(0.70, 1.00),
        ar_band=(5.0, 8.0),
        washout_band=(0.0, 2.0),
        body_depth_band=(1.8, 2.4),
        root_chord_frac_band=(0.13, 0.26),
        # An unswept, near-rectangular tailless wing can approach the same
        # spanload as a conventional wing; only the reflex loading costs it.
        oswald_mult=0.92,
        vstab_options=("center_fin", "winglets"),
        bell_spanload=False,
        blurb="Slow, docile, efficient - the trainer of the tailless world.",
    ),
    # NASA Prandtl-D and the Horten sailplanes: a bell-shaped spanload needs a
    # large root-to-tip twist (Prandtl-D carries ~9 deg of washout), and the
    # induced THRUST in the tip region gives proverse yaw - which is why these
    # aircraft carry no vertical surfaces at all. Fitting fins to one defeats
    # the entire configuration.
    "bell": PlanformDef(
        name="bell", label="Bell-distribution (Horten)",
        description=(
            "A bell-shaped spanload wing: strongly tapered, well swept and "
            "carrying 8-13 degrees of washout so the tips are lightly loaded. "
            "The tip region then makes induced THRUST in a turn, giving "
            "proverse yaw - so it needs, and gets, no vertical surfaces."
        ),
        sweep_band=(20.0, 34.0),
        taper_band=(0.20, 0.42),
        ar_band=(6.0, 11.0),
        washout_band=(8.0, 13.0),
        body_depth_band=(1.8, 2.3),
        root_chord_frac_band=(0.12, 0.28),
        # A bell spanload deliberately trades ~10-15% more induced drag than
        # the elliptic optimum for proverse yaw and a lighter structure
        # (Prandtl / Horten): the span-efficiency multiplier reflects that.
        oswald_mult=0.80,
        vstab_options=("none",),
        bell_spanload=True,
        blurb="No fins, proverse yaw, and it thermals like a bird.",
    ),
}


@dataclass(frozen=True)
class MissionDef:
    """What the wing is for. Sets the target static margin, the wing-loading
    band the optimizer centres on, the stall-speed safety factor and the
    default cruise speed offered in the UI."""
    name: str
    label: str
    static_margin: float               # target SM, fraction of MAC
    wl_band: tuple[float, float]       # wing loading band [kg/m^2]
    stall_factor: float                # required V_cruise >= f * V_stall
    default_cruise_ms: float
    preferred_planforms: tuple[str, ...]  # UI ORDER ONLY - never a restriction
    blurb: str


MISSIONS: dict[str, MissionDef] = {
    # Wing loadings are measured from real foam/printed wings: an X5 at
    # 1.2-1.5 kg over 44 dm^2 is 2.7-3.4 kg/m^2; a 900 mm AR Wing at ~0.8 kg
    # over 20 dm^2 is ~4 kg/m^2; a loaded long-range FPV wing reaches 8-11.
    "sport": MissionDef(
        "sport", "Sport", 0.07, (2.5, 10.0), 1.30, 17.0,
        ("swept", "bwb", "plank", "bell"),
        "Quick, crisp, aerobatic - the everyday slope and field wing.",
    ),
    "fpv_cruiser": MissionDef(
        "fpv_cruiser", "FPV cruiser", 0.08, (3.0, 11.0), 1.30, 16.0,
        ("bwb", "swept", "plank", "bell"),
        "Steady camera platform with room for the gear and a long pack.",
    ),
    "thermal_floater": MissionDef(
        "thermal_floater", "Thermal floater", 0.06, (1.8, 5.0), 1.30, 11.0,
        ("bell", "plank", "swept", "bwb"),
        "Light, slow and efficient - stays up on very little.",
    ),
    "park_flyer": MissionDef(
        "park_flyer", "Park flyer", 0.09, (1.8, 5.5), 1.35, 11.0,
        ("plank", "swept", "bwb", "bell"),
        "Small field, low speed, forgiving - built to be relaunched all day.",
    ),
}


# ---------------------------------------------------------------------------
# Vertical-surface arrangements (spec 3.3)
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class VstabDef:
    name: str
    label: str
    count: int
    y_frac: float          # spanwise station, fraction of semi-span
    cant_deg: float        # outboard cant (0 = vertical)
    description: str


VSTABS: dict[str, VstabDef] = {
    "winglets": VstabDef(
        "winglets", "2 tip winglets", 2, 0.97, 12.0,
        "Flat trapezoidal fins at the tips, canted outboard - the standard "
        "swept-wing arrangement; they also fence the tip vortex.",
    ),
    "twin_fin": VstabDef(
        # X5-style: the fins sit inboard where the wing is still deep enough
        # to take the loads, at 0.50-0.65 of the semi-span (spec 3.3).
        "twin_fin", "2 inboard fins", 2, 0.58, 0.0,
        "Two vertical fins standing inboard of the tips, X5 style - out of "
        "the way on a hand launch and easy to bolt into a spar.",
    ),
    "center_fin": VstabDef(
        "center_fin", "1 centre fin", 1, 0.0, 0.0,
        "A single vertical stabilizer on the centre-body spine. The only "
        "workable choice on an unswept plank, which has no sweep to lean on "
        "for yaw stiffness, and a perfectly good one on any swept wing - it "
        "sits on a shorter arm than tip fins, so it is sized about 15% larger "
        "for the same effect, but it keeps the tips clean and survives a "
        "cartwheel landing far better than a winglet does.",
    ),
    "none": VstabDef(
        "none", "No vertical surfaces", 0, 0.0, 0.0,
        "Nothing at all: a bell-spanload wing gets its yaw stability and "
        "proverse yaw from the spanload itself. Adding fins defeats it.",
    ),
}


# =============================================================================
# v2 multi-type additions (V2_PLAN.md). This whole block is ADDITIVE: no v1
# definition changed, and the flying-wing path never reads these names.
# =============================================================================

# The airplane-type registry the API validates against and the UI lists.
# "any" is a REQUEST mode, not a type: it is resolved by physics.variants into
# concrete designs that each carry one of these keys.
#
# v3 "axes" (ADDITIVE, V3_PLAN.md): which configuration axes apply per type,
# transcribed from RESEARCH_TYPES_V3.md s.10.4 "Applicability matrix
# (types x axes)" - the authoritative per-type truth. Conventions:
#   - motor_layout / tail_type / wing_position are lists of the LEGAL values;
#     the FIRST entry is the type's researched default. A single-entry list
#     means the axis is FIXED for the type (no UI choice); an empty list means
#     the axis is meaningless (request None / omit).
#   - n_motors is a [min, max] band (0 was legal only for the since-removed
#     glider; every current type flies with at least one motor).
# The frontend renders a select only for an axis with a real choice; the
# request sends nothing for "Type default", so a bare request is byte-
# identical to v2 and each type's own module applies its researched default.
AIRPLANE_TYPES: dict[str, dict] = {
    "flying_wing": {
        "label": "Flying wing",
        "description": (
            "Tailless blended wing - one continuous lifting surface tip to "
            "tip, trimmed by a reflexed section and washout. The v1 product, "
            "unchanged."),
        # s.10.4: "pusher (fixed) | 1 | None | None" - no configurable axis;
        # the v1 panel gains nothing.
        "axes": {"motor_layout": ["pusher"], "n_motors": [1, 1],
                 "tail_type": [], "wing_position": []},
    },
    "conventional": {
        "label": "Conventional",
        "description": (
            "Wing + fuselage + horizontal and vertical stabilizers with a "
            "single tractor motor on the nose - the classic trainer/sport "
            "layout. The tail does the trimming, so the wing flies a plain "
            "cambered section."),
        # s.10.4: tractor default / pusher legal; 1 default / 2 legal (s.8
        # wing nacelles); all three tails (s.9); high default, mid/low legal
        # (s.10).
        "axes": {"motor_layout": ["tractor", "pusher"], "n_motors": [1, 2],
                 "tail_type": ["conventional", "t_tail", "v_tail"],
                 "wing_position": ["high", "mid", "low"]},
    },
    # v3 (V3_PLAN.md): six more families. Bands live in each type's own
    # physics module, cited to RESEARCH_TYPES_V3.md.
    "delta": {
        "label": "Delta",
        "description": (
            "Tailless triangle - 45-60 deg leading-edge sweep, low aspect "
            "ratio, elevons and a centre fin. Rides the flying-wing "
            "machinery at its own bands; docile at high alpha, fast in a "
            "straight line."),
        # s.10.4: pusher default (every s.1.1 reference airframe), tractor
        # legal; single motor (s.8); tailless - fins come from the planform.
        "axes": {"motor_layout": ["pusher", "tractor"], "n_motors": [1, 1],
                 "tail_type": [], "wing_position": []},
    },
    "canard": {
        "label": "Canard",
        "description": (
            "Foreplane first: a lifting canard ahead of the main wing, "
            "pusher motor aft, tip fins. The canard is loaded harder than "
            "the wing by design, so it always stalls first and the nose "
            "drops before the wing can let go."),
        # s.10.4: pusher default [LEN-CAN s.5]; "tractor -> see tandem", so
        # tractor is NOT offered here; single motor (s.8); no tail (tip
        # fins); wing_position read as the REAR wing's position, mid default.
        "axes": {"motor_layout": ["pusher"], "n_motors": [1, 1],
                 "tail_type": [],
                 "wing_position": ["mid", "high", "low"]},
    },
    "tandem": {
        "label": "Tandem wing",
        "description": (
            "Two comparable wings sharing the lift - front wing high-set "
            "and loaded harder (it stalls first), rear wing carrying the "
            "ailerons, fin and rudder aft. Quickie-class layout."),
        # s.10.4: tractor default [LEN-CAN s.6], pusher legal; single motor
        # (s.8); no stab (aft fin / tip fins); rear wing HIGH is fixed by
        # the s.3.2 gap rule (rear high / front low), so no choice.
        "axes": {"motor_layout": ["tractor", "pusher"], "n_motors": [1, 1],
                 "tail_type": [], "wing_position": ["high"]},
    },
    # "biplane" and "glider" were REMOVED from this registry by the builder
    # (2026-08-21: "lets remove biplane and the glider option completely").
    # Their physics modules and tests went with them; the biplane's CAD
    # builder remains dormant inside cad/multiwing.py (shared with canard/
    # tandem, whose machinery it is built from) but is unreachable - dispatch
    # is table-driven off THIS registry and test_type_dispatch asserts the
    # two cannot drift.
    "twin_boom": {
        "label": "Twin boom",
        "description": (
            "Pusher pod with two tail booms carrying an H-tail between "
            "them - the FPV / mapping workhorse layout. Clean nose for a "
            "camera, prop protected between the booms."),
        # s.10.4: pusher default (s.5), tractor-twins-on-booms legal; 1 or 2
        # motors; H-tail between booms default, inverted-V legal (mapped to
        # the schema's "v_tail"); high wing FIXED (s.5.1 practice).
        "axes": {"motor_layout": ["pusher", "tractor"], "n_motors": [1, 2],
                 "tail_type": ["conventional", "v_tail"],
                 "wing_position": ["high"]},
    },
}

# Recommended (NOT enforced) minimum size box per type, mm [length, width,
# height], shown next to the box inputs (work plan task 5). Each is
# derived from the hardware the CAD must bury, worked backwards through the
# type's own proportion rules; a smaller box is accepted and the optimizer
# then reports the honest constraint failure (`servo_fit`, `box_*`) itself.
# Shared inputs (cad/servos.py, cad/hatch.py, cad/conduits.py restated in
# physics/twinboom.py):
#   * SG90 pocket: 32.5 mm ear span + 2 x 0.25 clearance chordwise window,
#     12.40 + 0.25 + 0.15 safety + 1.2 wall = 14.0 mm straight well depth ->
#     `twinboom._chord_floor_mm`: RFX-9 reflexed 165 mm chord at zero tilt,
#     190 mm with an ordinary 2 deg / 2 deg dihedral+twist corner loss;
#     NACA 2412 128 / 147 mm.
#   * Wire pipes: 12.0 x 8.25 mm oval, constant section, so the skin the
#     pipe runs through must be >= 8.25 + 2 walls ~ 11 mm deep at its
#     shallowest station - satisfied wherever the servo well already is.
#   * Bay: hatch._MIN_BAY_LEN_MM 25 / _MIN_BAY_DEPTH_MM 10 + 2 walls, with
#     bay_len = 0.60 x centre chord (optimizer.BAY_LENGTH_FRAC) on the
#     tailless types and >= 130 mm long, 60 x 45 mm inside on the fuselage
#     types (conventional.BAY_LEN_MIN_M / FUS_WIDTH_MIN_M / FUS_HEIGHT_MIN_M).
RECOMMENDED_MIN_BOX_MM: dict[str, list[int]] = {
    # Servo arm station = elevon inner 0.35 + 0.10 + half footprint ~ 0.5 of
    # the semispan, where a swept wing (root 0.30 x span, taper ~0.45) is
    # ~0.22 x span deep in chord: 190 / 0.22 = 860 -> width 800 (the plank's
    # 0.9 taper gives the same 0.2 x span). Length: root chord 0.30 x 800 =
    # 240 plus the centre body's chord_scale nose extension and the pusher
    # mount behind the TE -> 400. Height: 11 % section on a ~320 mm centre
    # chord = 35 mm, bay depth_scale on top, winglets / centre fin <= 0.6 x
    # local chord -> 120.
    "flying_wing": [400, 800, 120],
    # 60 mm bay + 2 walls wide, 45 mm deep at 62 % of a 73 mm section, bay
    # >= 130 mm long from 8 % of the fuselage with the tail arm 3 MAC behind:
    # fuselage ~0.66 x span (span_target rule). Aileron servo at 0.55-0.60
    # semispan on a NACA 2412: AR 5, taper 0.8 -> chord there ~0.20 x span,
    # 147 / 0.20 = 735 -> width 800; length 0.66 x 800 = 530 -> 550; height:
    # 73 mm fuselage + fin (~0.15 x span) + dihedral rise -> 200.
    "conventional": [550, 800, 200],
    # AR 1.8-3.5, taper 0.10-0.30: chord at the arm station ~0.35 x span, so
    # 190 / 0.35 = 540 -> width 600. Root chord 0.42-0.60 x span -> length
    # 400 (the delta is the one type whose LENGTH, not width, is the tight
    # side of the box). Centre fin ~0.15 x span above a 40 mm root -> 150.
    "delta": [400, 600, 150],
    # Rear wing carries the ailerons at AR 5.5-7, taper ~0.85: chord at the
    # servo ~0.19 x span, 147 / 0.19 = 775 -> width 800. Fuselage 0.8 x
    # span (span_target rule) so the foreplane clears the wing -> 650.
    # Height: 73 mm pod + tip fins -> 180.
    "canard": [650, 800, 180],
    # Rear wing AR 5.0-7.0 with the ailerons at ~0.55 semispan -> width 800
    # as the canard. Fuselage 0.75 x span -> 600. Two wings stacked by the
    # gap rule (front low, rear high) plus the aft fin -> 220.
    "tandem": [600, 800, 220],
    # AR 7-10 hard band, taper 0.6-1.0: chord at the 0.50-0.58 aileron root
    # ~0.13 x span at the AR floor, 147 / 0.13 = 1130; evaluate_twinboom
    # lifts the chord to the servo floor itself, so 1100 is where it stops
    # having to. Pod + booms + H-tail = 0.62 x span -> 700. Height: 73 mm
    # pod + H-tail fins between the booms -> 200.
    "twin_boom": [700, 1100, 200],
}
for _name, _box in RECOMMENDED_MIN_BOX_MM.items():
    AIRPLANE_TYPES[_name]["recommended_min_box_mm"] = list(_box)
assert set(RECOMMENDED_MIN_BOX_MM) == set(AIRPLANE_TYPES),     "every AIRPLANE_TYPES entry needs a recommended_min_box_mm"


# ---------------------------------------------------------------------------
# Conventional (tailed) design bands - RESEARCH_CONVENTIONAL.md.
#
# Static margin: [MIT] eq.(2) gives the ideal band SM = +0.05..+0.15 of MAC;
# trainer practice ([ND] CG 25-33% chord, FT Simple Cub shipping at 22%) sits
# deliberately at/above the top of it. Contrast SM_BAND above: a TAILED model
# both tolerates and wants roughly twice the tailless trainer-end margin,
# because the tail's damping and elevator authority make a nose-heavy model
# docile rather than unflyable (RESEARCH_CONVENTIONAL.md section 3).
# ---------------------------------------------------------------------------
CONV_SM_BAND: tuple[float, float] = (0.05, 0.15)

# Horizontal tail volume V_H = S_h l_h / (S c_mac): [MIT] 0.30-0.60, [RCG]
# sport 0.50-0.80 / scale 0.35-0.60, [SCHOLZ]/Raymer homebuilt 0.50. The
# recommended app band from the dossier is 0.40-0.65 (section 2).
CONV_VH_BAND: tuple[float, float] = (0.40, 0.65)
# Vertical tail volume V_V = S_v l_v / (S b): [MIT] 0.02-0.05, [SCHOLZ]/Raymer
# GA single 0.04. Dossier recommendation 0.025-0.05, trainer 0.035-0.04.
# NOTE: this is the genuine tail-aft band. The flying-wing fin math above
# reports V_V far below it for its own documented reason - never share code
# between the two checks (RESEARCH_CONVENTIONAL.md section 2 note).
CONV_VV_BAND: tuple[float, float] = (0.025, 0.05)
# Tail arm: [ND] stab LE 2-3 wing chords behind the TE; with CG near 0.28c the
# CG -> stab quarter-chord arm is l_h = 2.5-3.5 x MAC (dossier section 2).
CONV_LH_MAC_BAND: tuple[float, float] = (2.5, 3.5)
# Stab area 15-20% of wing [ND]; hard limits widened to the sourced spread
# (V_H x arm combinations reach both edges).
CONV_SH_S_BAND: tuple[float, float] = (0.10, 0.24)
# Fuselage length ~75% of span [ND]; real airframes 0.63-0.80 (dossier
# section 6, derived from the section-1 table); hard band with tolerance.
CONV_FUS_LEN_FRAC_BAND: tuple[float, float] = (0.60, 0.84)
# Fuselage height ~10-15% of its length [ND].
CONV_FUS_HEIGHT_FRAC: float = 0.12


@dataclass(frozen=True)
class ConvStyleDef:
    """One conventional-airplane character (the tailed analogue of a planform
    family). Every number is a row of the coefficient quick-reference table at
    the end of RESEARCH_CONVENTIONAL.md; sources are in its sections 2-6."""
    name: str
    label: str
    description: str
    ar_band: tuple[float, float]       # [ND] span = 5-6 x chord; dossier s.4
    taper_band: tuple[float, float]    # dossier s.4 (from the s.1 airframes)
    dihedral_deg: float                # [ND]/[FMS] bands, dossier s.4
    wing_position: str                 # "high" | "low"
    washout_deg: float                 # [ND]: 3-5 deg trainers, less aerobatic
    vh: float                          # target V_H (dossier s.2)
    vv: float                          # target V_V (dossier s.2)
    sm_target: float                   # dossier s.3 mission targets
    lh_mac: float                      # tail arm / MAC target (dossier s.2)
    nose_len_mac: float                # [ND]: nose 1-1.5 chords ahead of LE
    fus_len_frac: float                # fuselage length / span target (s.6)
    elevator_frac: float               # elevator / S_h ([ND] 20-30%)
    rudder_frac: float                 # rudder / S_v ([ND] 1/3-1/2 of fin)
    aileron_inner_frac: float          # dossier s.5 recommended outboard set
    aileron_outer_frac: float
    aileron_chord_frac: float          # 25% of wing chord [ND]/Lennon
    airfoil: str                       # non-reflexed section (the tail trims)
    wl_band: tuple[float, float]       # wing loading [kg/m^2] (25-60 g/dm^2
    #                                    hard per dossier s.7; style slices)
    blurb: str


CONV_STYLES: dict[str, ConvStyleDef] = {
    # The [ND] proportion diagram made flesh: Apprentice / Eclipson Model A /
    # FT Simple Cub class. Every default is the "trainer" column of the
    # quick-reference table.
    "trainer": ConvStyleDef(
        name="trainer", label="High-wing trainer",
        description=(
            "The classic first aeroplane: high wing with visible dihedral, "
            "slab-sided fuselage about 3/4 of the span, a big stab a long "
            "way back and a nose-heavy CG. Apprentice / Eclipson Model A "
            "class proportions."),
        ar_band=(5.5, 7.0), taper_band=(0.70, 1.00),
        dihedral_deg=2.5, wing_position="high", washout_deg=3.0,
        vh=0.50, vv=0.038, sm_target=0.12, lh_mac=3.0,
        nose_len_mac=1.15, fus_len_frac=0.73,
        elevator_frac=0.25, rudder_frac=0.40,
        aileron_inner_frac=0.60, aileron_outer_frac=0.95,
        aileron_chord_frac=0.25,
        airfoil="NACA 2412", wl_band=(2.8, 4.5),
        blurb="Docile, self-recovering, lands at a walk - the first aeroplane.",
    ),
    "sport": ConvStyleDef(
        name="sport", label="Low-wing sport",
        description=(
            "A low-wing sport model: less dihedral, more taper, a slightly "
            "hotter tail volume and the CG a notch further aft than the "
            "trainer - quick but still honest."),
        ar_band=(5.0, 6.5), taper_band=(0.55, 0.80),
        dihedral_deg=2.0, wing_position="low", washout_deg=2.0,
        vh=0.55, vv=0.035, sm_target=0.10, lh_mac=3.0,
        nose_len_mac=1.20, fus_len_frac=0.72,
        elevator_frac=0.27, rudder_frac=0.45,
        aileron_inner_frac=0.55, aileron_outer_frac=0.95,
        aileron_chord_frac=0.25,
        airfoil="NACA 2412", wl_band=(3.2, 5.0),
        blurb="Crisper than the trainer without giving up the manners.",
    ),
    # [ND]: "symmetrical airfoils are intended for aerobatic models"; lower AR
    # for roll rate; the CG rides the bottom of the tailed band.
    "aerobatic": ConvStyleDef(
        name="aerobatic", label="Aerobat",
        description=(
            "Symmetric section, low aspect ratio, minimal dihedral and a "
            "light static margin: it flies the same upright or inverted and "
            "rolls on its own axis."),
        ar_band=(4.5, 6.0), taper_band=(0.50, 0.70),
        dihedral_deg=1.0, wing_position="low", washout_deg=0.5,
        vh=0.45, vv=0.030, sm_target=0.06, lh_mac=2.7,
        nose_len_mac=1.30, fus_len_frac=0.70,
        elevator_frac=0.30, rudder_frac=0.50,
        aileron_inner_frac=0.50, aileron_outer_frac=0.95,
        aileron_chord_frac=0.25,
        airfoil="NACA 0010", wl_band=(3.2, 5.2),
        blurb="Neutral, symmetric and eager - it goes exactly where pointed.",
    ),
    "floater": ConvStyleDef(
        name="floater", label="High-wing floater",
        description=(
            "A light, high-lift high-winger: big camber, generous dihedral "
            "and the lowest wing loading of the set - it climbs on a breath "
            "and lands at walking pace."),
        ar_band=(6.0, 7.0), taper_band=(0.75, 1.00),
        dihedral_deg=3.0, wing_position="high", washout_deg=3.0,
        vh=0.45, vv=0.032, sm_target=0.12, lh_mac=3.2,
        nose_len_mac=1.05, fus_len_frac=0.70,
        elevator_frac=0.25, rudder_frac=0.40,
        aileron_inner_frac=0.60, aileron_outer_frac=0.95,
        aileron_chord_frac=0.22,
        airfoil="NACA 4412", wl_band=(2.4, 3.8),
        blurb="Slow, floaty and forgiving - thermals and calm evenings.",
    ),
    "speed": ConvStyleDef(
        name="speed", label="Sport speedster",
        description=(
            "Thin low-camber section, short span, higher wing loading and a "
            "longer nose: it carries speed and punches wind, at the price of "
            "a faster approach."),
        ar_band=(4.5, 5.5), taper_band=(0.55, 0.75),
        dihedral_deg=1.5, wing_position="low", washout_deg=1.0,
        vh=0.50, vv=0.035, sm_target=0.08, lh_mac=2.8,
        nose_len_mac=1.25, fus_len_frac=0.70,
        elevator_frac=0.27, rudder_frac=0.45,
        aileron_inner_frac=0.55, aileron_outer_frac=0.95,
        aileron_chord_frac=0.25,
        airfoil="NACA 1410", wl_band=(3.5, 5.5),
        blurb="Wind penetration and pace - fly the approach with power on.",
    ),
}

# Which conventional style leads for each (reused) mission. Conventional
# missions ARE the existing MISSIONS - only the leading character differs.
CONV_STYLE_FOR_MISSION: dict[str, str] = {
    "sport": "sport",
    "fpv_cruiser": "trainer",
    "thermal_floater": "floater",
    "park_flyer": "trainer",
}


def resolve_vstab(planform: PlanformDef, choice: str | None) -> tuple[str, list[str]]:
    """Resolve a user vstab choice against what this planform allows.

    Returns (resolved_name, notes). A bell-spanload wing always resolves to
    "none" - vertical surfaces are forbidden on it, not merely discouraged.
    """
    notes: list[str] = []
    want = (choice or "auto").strip() or "auto"
    if planform.bell_spanload:
        if want not in ("auto", "none"):
            notes.append(
                f"A {planform.label} carries no vertical surfaces: the bell "
                "spanload's tip-region induced thrust is what gives it "
                "proverse yaw, and a fin would cancel it. Ignoring the "
                f"'{want}' request.")
        return "none", notes
    if want == "auto":
        return planform.vstab_options[0], notes
    if want not in planform.vstab_options:
        notes.append(
            f"'{VSTABS.get(want, VSTABS['winglets']).label}' is not available "
            f"on a {planform.label}; using "
            f"'{VSTABS[planform.vstab_options[0]].label}'.")
        return planform.vstab_options[0], notes
    return want, notes
