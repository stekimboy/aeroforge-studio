"""TWIN-BOOM pusher design solve + variant generation (V3_PLAN.md).

NEW in v3. Lives BESIDE the conventional path, same fence discipline: nothing
here is imported on any other type's run, and the frozen solvers are
IMPORTED, never copied - `stability_conv.solve_conventional` / `conv_handling`
do the longitudinal and lateral math on a TailSizing this module constructs
for the twin-boom's own tail geometry (stab spanning boom-to-boom, vertical
area split across two boom-mounted fins).

The airframe and why it exists ([RT3 s.5], Wikipedia twin-boom): "For a
single engine with a propeller in pusher configuration, a conventional tail
requires the propeller to be moved far aft... The twin-boom configuration
allows a much shorter and more efficient installation." So: a short pod
(nose payload/battery bay, PUSHER motor on the pod's aft face), two booms
carrying the tail at a full conventional arm, H-tail between the booms.

Every band cites RESEARCH_TYPES_V3.md s.5 (tag [RT3 s.5], quick-reference
rows 33-37); reference airframes: MyTwinDream 1800, ZOHD/SonicModell
Skyhunter 1800, RMRC Skyhunter.
"""
from __future__ import annotations

import math
import uuid
from dataclasses import dataclass
from functools import lru_cache

import numpy as np
from scipy import optimize as sciopt

from . import aero as A
from .airfoils import (
    Airfoil, fin_airfoil, naca4_coordinates, reflexed_coordinates,
)
from .atmosphere import isa, reynolds
from .config_defs import MISSIONS, MissionDef
from .conventional import CONV_AIRFOILS, STRUCT_CHORD_CENTROID, traits_generic
from .optimizer import (
    G, RE_MIN_GOOD, RE_MIN_HARD, _band, _band_center_pen, _motor_mount_spec,
)
from .stability import chords_from_area, mac_length, y_mac
from .stability_conv import (
    AR_V, FIN_SWEEP_DEG, TAPER_V, TailSizing, conv_handling,
    solve_conventional,
)
from .weights import (
    MATERIALS, PACK_PER_MOTOR, SERVO_MASS, estimate_mass_conventional,
)

# ---------------------------------------------------------------------------
# Research bands, transcribed ([RT3 s.5.2] / quick-reference rows 33-37)
# ---------------------------------------------------------------------------
# Tail arm 2.5-3.5 x MAC, default 3.0 - the conventional band applies
# verbatim; the booms exist to buy that arm without a rear fuselage.
TB_LH_MAC_BAND: tuple[float, float] = (2.5, 3.5)
# V_H 0.40-0.65 (default 0.50; the boom-to-boom stab picks up end-plate
# benefit from the fins, so it may sit at the BOTTOM of the band), V_V
# 0.025-0.05 (default 0.035) with the vertical area SPLIT half per
# boom-mounted fin ([MIT]/[SCHOLZ] coefficients, fins at the boom ends
# where l_v ~ l_h).
TB_VH_BAND: tuple[float, float] = (0.40, 0.65)
TB_VV_BAND: tuple[float, float] = (0.025, 0.05)
N_FINS = 2                       # one per boom, by definition
# Boom spacing >= 1.15 x prop diameter (derived rule, default 1.2), and
# never < prop D + 2 x 15 mm tip clearance - this project's clearance
# doctrine applied to a spinning tip (a flexing boom converges on the disk).
BOOM_SPACING_PROP_MULT_MIN = 1.15
BOOM_SPACING_PROP_MULT = 1.20
PROP_TIP_CLEARANCE_M = 0.015
# Prop-diameter estimate from the existing power-mass model's motor classes
# (optimizer.MOUNT_PATTERNS_MM: <=0.70 kg -> 2204-2306, <=1.60 kg ->
# 2212-2814, above -> 28xx-35xx). Those classes swing 6-7 / 8-9 / 10 inch
# props respectively (RC practice; [RT3 s.5.2]: "our class flies 8-10 in
# props, so boom spacing ~ 260-330 mm" - the Skyhunter's recommended props
# are 9-12 in [RMRC]). DOCUMENTED ESTIMATE, user-overridable via
# inp["prop_diameter_m"].
PROP_D_BY_MASS_M: tuple[tuple[float, float], ...] = (
    (0.70, 0.178),   # 7 in - 2204-2306 class
    (1.60, 0.229),   # 9 in - 2212-2814 class (the common FPV size)
    (1e9, 0.254),    # 10 in - 28xx-35xx class
)
# Wing: Skyhunter 1800 derives to AR 9.0 on a near-constant 200 mm chord
# ([RT3 s.5.1] derived; row 36: band 7-10, default 9).
TB_AR_HARD: tuple[float, float] = (7.0, 10.0)
# Wing loading 30-60 g/dm^2 (row 37: hold the conventional 30-50 band at
# this scale, top end for the payload mission); checked with the same
# x0.8 / x1.2 tolerance the conventional solver applies to its band.
TB_WL_HARD_KGM2: tuple[float, float] = (3.0, 6.0)
# Pod ~ 2 x MAC long, ending at/near the wing TE where the prop spins
# between the booms (derived from the s.5.1 airframes' proportions); CG
# anchor "1/3 of wing from leading edge" [RMRC].
POD_LEN_MAC = 2.0
POD_HEIGHT_MIN_M = 0.075        # bay >= 45 mm deep at 62% + walls (same
#                                 dossier s.6 minimum the conventional uses)
POD_HEIGHT_FRAC = 0.21          # of pod length - the stubby FPV-pod look
POD_WIDTH_MIN_M = 0.064         # bay >= 60 mm wide + 2 walls
POD_WIDTH_HEIGHT_FRAC = 0.85
POD_VOLUME_FILL = 0.55
# Boom stiffness proxy ([RT3 s.5.2] / V3_PLAN verification item): the boom
# section's second moment must not fall below a round carbon tube
# Ø8 x 1 mm wall equivalent PER METRE of boom - I_req = I_ref x L[m].
# I(tube) = pi (D^4 - d^4)/64: Ø8x1 -> 137.4 mm^4, Ø10x1 -> 289.8 mm^4.
# ([RMRC] kits ship tube booms; the printed boom is a fairing around a
# carbon-tube socket, so the tube IS the stiffness.)
BOOM_TUBES_MM: tuple[tuple[float, float], ...] = ((8.0, 1.0), (10.0, 1.0))
BOOM_I_REF_MM4_PER_M = math.pi * (8.0**4 - 6.0**4) / 64.0     # 137.4
BOOM_SOCKET_CLEARANCE_MM = 0.30   # printed socket over the tube diameter
BOOM_FAIRING_SIDE_MM = 14.0       # printed square fairing around the tube

# H-tail between the booms: rectangular stab spanning boom-to-boom (span =
# boom spacing BY CONSTRUCTION [RT3 s.5.2]); fins are the conventional
# trapezoid (stability_conv's AR_V / TAPER_V / FIN_SWEEP_DEG), half the
# vertical area each, root TE on the boom end.
BAY_START_FRAC_POD = 0.06
BAY_LEN_FRAC_POD = 0.52
BAY_LEN_MIN_M = 0.12
BAY_DEPTH_FRAC = 0.62

DIHEDRAL_DEG_TB = 2.0   # high wing, RC ladder 1-2 deg geometric
#                         (config_axes.WING_POS_GEOM_DIHEDRAL_DEG["high"];
#                         [RT3 s.10.1])
WASHOUT_DEG_TB = 2.0    # ordinary straight-wing tip-stall margin

# ---------------------------------------------------------------------------
# The wing has to be able to bury its own aileron servo
# ---------------------------------------------------------------------------
# A twin-boom is the type this bites: high aspect ratio on a modest span
# leaves a short chord, and the aileron servo lies on its SIDE inside the
# panel, so the pocket needs a straight prismatic well 12.4 mm deep running
# 32.5 mm along the chord - the servo's mounting ears are full case width and
# set the length. On a NACA 2412 that well is only ~0.89 of the section's
# maximum thickness, and on the cambered 4412 rather less, because the pocket
# is a BOX: what it gets is (lowest crown) minus (highest keel) across the
# whole footprint, not the peak thickness at one station.
#
# Before this floor existed the `mapper` style shipped with no aileron servo
# pockets at all - the CAD refused them and said so in the warnings, and
# nothing upstream was listening. Widening the chord here is the doctrine
# ARCHITECTURE.md states for exactly this case: if a part needs more room, widen the
# budget in the optimizer, never silently clamp the part.
#
# These four numbers are `backend.cad.servos`', restated because physics must
# not import backend.cad - cadquery would follow it into the server process,
# which is the whole reason `paths.py` exists. `tests/test_boomglider_physics`
# asserts they still equal the CAD's, so the two cannot drift apart.
SERVO_POCKET_DEPTH_MM = 12.40     # servos.SERVO_SG90["body_wid_mm"]
SERVO_POCKET_LEN_MM = 32.50       # servos.SERVO_SG90["ear_span_mm"]
SERVO_POCKET_CLEAR_MM = 0.25      # servos.SERVO_CLEARANCE_MM
SERVO_POCKET_SAFETY_MM = 0.15     # servos.ROOF_SAFETY_MM
# The servo straddles its station, so the chord is taken at the OUTBOARD end
# of its footprint (half of `total_h + 2 cl + 1`), and the deepest station the
# CAD will accept is just outboard of the aileron root (`f_min = inner+0.02`).
SERVO_SPAN_MM = 34.05             # total_h + 2 cl + 1, servos.servo_bay
SERVO_HALF_SPAN_MM = 0.5 * SERVO_SPAN_MM
SERVO_STATION_OFFSET = 0.02


def _upper_lower(coords: np.ndarray, xs: np.ndarray):
    """Interpolate a Selig loop (TE -> upper -> LE -> lower -> TE) on `xs`."""
    c = np.asarray(coords, dtype=float)
    n = (len(c) + 1) // 2
    up = c[:n][::-1]              # LE -> TE
    lo = c[n - 1:]                # LE -> TE
    return (np.interp(xs, up[:, 0], up[:, 1]),
            np.interp(xs, lo[:, 0], lo[:, 1]))


def section_well_frac(coords: np.ndarray, window: float) -> float:
    """Deepest STRAIGHT well, per unit chord, that fits in this section.

    `window` is the well's chordwise length as a fraction of chord. The well
    is a box, so its depth is min(upper) - max(lower) over the window, and the
    window is slid to the best position - which is what `cad.servos._sample`
    measures on the built wing.
    """
    w = float(min(max(window, 1e-3), 1.0))
    xs = _WELL_XS
    up, lo = _upper_lower_cached(coords)
    # Speed pass 2026-08-28: the 121 window positions are scanned as ONE
    # boolean matrix instead of a Python loop (the bisection above calls this
    # ~5 000 times per generate). Same xs grid, same mask rule, same min/max
    # per window - min and max are exact, so the result is bit-identical.
    c0 = np.linspace(0.5 * w, 1.0 - 0.5 * w, 121)
    m = np.abs(xs[None, :] - c0[:, None]) <= 0.5 * w + 1e-12
    rows = m.any(axis=1)
    if not rows.any():
        return 0.0
    up_min = np.where(m, up[None, :], np.inf).min(axis=1)
    lo_max = np.where(m, lo[None, :], -np.inf).max(axis=1)
    best = float((up_min - lo_max)[rows].max())
    return max(best, 0.0)


_WELL_XS = np.linspace(0.0, 1.0, 401)
_UL_CACHE: dict[tuple, tuple[np.ndarray, np.ndarray]] = {}


def _upper_lower_cached(coords: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """`_upper_lower(coords, _WELL_XS)`, memoized on the coordinate values
    (the same section is scanned at ~5 000 window widths per generate)."""
    c = np.asarray(coords, dtype=float)
    key = (c.shape, c.tobytes())
    hit = _UL_CACHE.get(key)
    if hit is None:
        if len(_UL_CACHE) > 64:
            _UL_CACHE.clear()
        hit = _UL_CACHE[key] = _upper_lower(c, _WELL_XS)
    return hit


def pocket_tilt_loss_mm(dihedral_deg: float, twist_deg: float) -> float:
    """Well depth lost because the pocket is a WORLD-VERTICAL box and the wing
    it sits in is tilted two ways.

    The pocket's roof, floor and end walls are all axis-aligned - that is what
    lets the servo drop straight in - so any tilt of the surrounding skin eats
    depth at the corners:

    * **dihedral** tilts the panel spanwise, so across the pocket's own span
      the lowest crown and the highest keel are at opposite ends: one
      `span x tan(gamma)`; and
    * **twist** (root incidence minus washout at the station) tilts the
      section chordwise, costing `window x tan(theta)` the same way.

    Both are first-order exact and both are worth more than a millimetre on an
    ordinary 2 deg / 2 deg wing, which is why an estimate built on the bare
    section overstates what the wing can hold.
    """
    span = SERVO_SPAN_MM
    win = SERVO_POCKET_LEN_MM + 2.0 * SERVO_POCKET_CLEAR_MM
    return (span * math.tan(math.radians(abs(float(dihedral_deg))))
            + win * math.tan(math.radians(abs(float(twist_deg)))))


@lru_cache(maxsize=256)
def _chord_floor_mm(kind: str, param: str, wall_mm: float,
                    extra_mm: float) -> float:
    """Bisection cached on the section, because the sweep calls this for every
    grid point and the scan itself is thousands of window positions."""
    coords = (naca4_coordinates(param) if kind == "naca4"
              else reflexed_coordinates(*(float(v) for v in param.split(","))))
    need = (SERVO_POCKET_DEPTH_MM + SERVO_POCKET_CLEAR_MM + float(wall_mm)
            + SERVO_POCKET_SAFETY_MM + float(extra_mm))
    win = SERVO_POCKET_LEN_MM + 2.0 * SERVO_POCKET_CLEAR_MM
    lo, hi = 20.0, 2000.0
    if section_well_frac(coords, win / hi) * hi < need:
        return hi                               # unreachable; report the cap
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if section_well_frac(coords, win / mid) * mid >= need:
            hi = mid
        else:
            lo = mid
    return hi


def servo_chord_floor_m(airfoil: Airfoil, wall_mm: float,
                        dihedral_deg: float = 0.0,
                        twist_deg: float = 0.0) -> float:
    """Smallest local chord [m] whose section can still hold the servo well.

    Monotonic in chord (a longer chord makes the fixed 33.0 mm footprint a
    smaller fraction of it, so the well deepens faster than linearly), so the
    bisection behind this is exact to the tolerance asked.
    """
    d = airfoil.defn
    extra = round(pocket_tilt_loss_mm(dihedral_deg, twist_deg), 2)
    return _chord_floor_mm(d.kind, str(d.param), float(wall_mm),
                           extra) / 1000.0


def boom_tube_mm(l_boom_m: float) -> dict:
    """Pick the carbon tube meeting the Ø8x1-per-metre stiffness rule and
    record the whole check so it is reproducible ([RT3 s.5.2])."""
    i_req = BOOM_I_REF_MM4_PER_M * max(l_boom_m, 0.0)
    for d_o, wall in BOOM_TUBES_MM:
        d_i = d_o - 2.0 * wall
        i_tube = math.pi * (d_o**4 - d_i**4) / 64.0
        if i_tube >= i_req - 1e-9:
            break
    return {
        "type": "carbon_tube_in_printed_fairing",
        "tube_od_mm": float(d_o),
        "tube_wall_mm": float(wall),
        "socket_id_mm": float(d_o + BOOM_SOCKET_CLEARANCE_MM),
        "fairing_mm": [BOOM_FAIRING_SIDE_MM, BOOM_FAIRING_SIDE_MM],
        "I_mm4": float(i_tube),
        "I_required_mm4": float(i_req),
        "rule": ("boom second moment >= Ø8x1 carbon equivalent per metre "
                 f"({BOOM_I_REF_MM4_PER_M:.1f} mm^4/m x {l_boom_m:.2f} m = "
                 f"{i_req:.1f} mm^4) [RT3 s.5.2]"),
    }


def estimate_prop_diameter_m(mass_kg: float, inp: dict) -> float:
    """Prop diameter from the power-mass model's motor class (documented
    estimate, see PROP_D_BY_MASS_M); overridable by the builder who knows
    the actual prop."""
    over = inp.get("prop_diameter_m")
    if over:
        return float(over)
    return next(d for m, d in PROP_D_BY_MASS_M if mass_kg <= m)


def boom_spacing_m(prop_d: float) -> float:
    """Boom spacing from the prop: >= 1.2 x D (hard floor 1.15 x D) and
    never < D + 2 x 15 mm tip clearance ([RT3 s.5.2])."""
    return max(BOOM_SPACING_PROP_MULT * prop_d,
               prop_d + 2.0 * PROP_TIP_CLEARANCE_M)


def size_twinboom_tail(*, area: float, span: float, mac: float, x_cg: float,
                       vh_target: float, vv_target: float, lh_mac: float,
                       spacing: float) -> TailSizing:
    """Size the twin-boom tail into stability_conv's own TailSizing container
    so the frozen solvers consume it unmodified.

    Stab: S_h from V_H = S_h l_h/(S c) at l_h = lh_mac x MAC; RECTANGULAR,
    spanning boom-to-boom (span_h = spacing by construction [RT3 s.5.2]), so
    its chord is S_h/spacing and its AC sits 0.25 c behind its LE. The fins
    end-plate both stab tips, which is what lets a low-geometric-AR stab
    behave like stability_conv's AR-3 assumption (Raymer's x1.55 end-plate
    factor on AR ~1.6-2 lands at 2.5-3.1) - conservative, documented, and
    the reason the V_H default sits at the band bottom.

    Fins: V_V = S_v l_v/(S b) at the arm the placed fin has, HALF the area
    per boom ([RT3 s.5.2]), each the conventional trapezoid (AR_V, TAPER_V,
    FIN_SWEEP_DEG imported from stability_conv), root TE on the tail post.
    TailSizing records PER-SURFACE heights/chords and TOTAL areas, exactly
    as the conventional single-fin case does.
    """
    l_h = lh_mac * mac
    s_h = vh_target * area * mac / max(l_h, 1e-6)
    span_h = spacing                                # by construction
    c_h = s_h / max(span_h, 1e-6)                   # rectangular stab
    x_ac_h = x_cg + l_h
    x_le_h = x_ac_h - 0.25 * c_h
    x_te_h = x_le_h + c_h

    l_v = l_h
    for _ in range(2):
        s_v = vv_target * area * span / max(l_v, 1e-6)
        s_each = s_v / N_FINS                       # split half per boom
        h_v = math.sqrt(AR_V * s_each)              # AR_V = h^2/S per fin
        c_root_v = 2.0 * s_each / (h_v * (1.0 + TAPER_V))
        c_tip_v = TAPER_V * c_root_v
        c_mac_v = mac_length(c_root_v, TAPER_V)
        x_le_v = x_te_h - c_root_v                  # root TE on the tail post
        x_ac_v = (x_le_v + 0.30 * c_root_v
                  + 0.35 * h_v * math.tan(math.radians(FIN_SWEEP_DEG)))
        l_v = max(x_ac_v - x_cg, 1e-3)
    vv = l_v * s_v / (area * span)
    vh = l_h * s_h / (area * mac)

    return TailSizing(
        s_h=float(s_h), span_h=float(span_h), c_root_h=float(c_h),
        c_tip_h=float(c_h), c_mac_h=float(c_h), x_le_h=float(x_le_h),
        x_ac_h=float(x_ac_h), l_h=float(l_h), vh=float(vh),
        s_v=float(s_v), h_v=float(h_v), c_root_v=float(c_root_v),
        c_tip_v=float(c_tip_v), c_mac_v=float(c_mac_v), x_le_v=float(x_le_v),
        x_ac_v=float(x_ac_v), l_v=float(l_v), vv=float(vv),
        x_te=float(max(x_te_h, x_le_v + c_root_v)),
    )


@dataclass(frozen=True)
class TBStyleDef:
    """One twin-boom character's researched numbers ([RT3 s.5] rows)."""
    name: str
    label: str
    description: str
    ar_band: tuple[float, float]
    taper_band: tuple[float, float]
    washout_deg: float
    vh: float
    vv: float
    sm_target: float
    lh_mac: float
    elevator_frac: float
    rudder_frac: float
    aileron_inner_frac: float
    aileron_outer_frac: float
    aileron_chord_frac: float
    airfoil: str
    wl_band: tuple[float, float]     # kg/m^2
    blurb: str


TB_STYLES: dict[str, TBStyleDef] = {
    # Skyhunter / MyTwinDream class: the type's reason to exist.
    "fpv": TBStyleDef(
        name="fpv", label="FPV twin-boom",
        description=(
            "The Skyhunter pattern: high wing near AR 9, a stubby pod with "
            "the whole nose free for camera and pack, pusher prop between "
            "the booms, H-tail on a full conventional arm."),
        ar_band=(8.0, 10.0), taper_band=(0.70, 1.00),
        washout_deg=WASHOUT_DEG_TB,
        vh=0.50, vv=0.035, sm_target=0.10, lh_mac=3.0,
        elevator_frac=0.28, rudder_frac=0.40,
        aileron_inner_frac=0.55, aileron_outer_frac=0.95,
        aileron_chord_frac=0.25,
        airfoil="NACA 2412", wl_band=(3.5, 5.0),
        blurb="Camera up front, prop out of the shot - the FPV workhorse.",
    ),
    "endurance": TBStyleDef(
        name="endurance", label="Endurance cruiser",
        description=(
            "The efficiency slice: aspect ratio at the top of the band, "
            "cambered floater section, lighter loading and a longer arm "
            "with a smaller tail - the mapping-loiter mission."),
        ar_band=(9.0, 10.0), taper_band=(0.65, 0.90),
        washout_deg=WASHOUT_DEG_TB,
        vh=0.45, vv=0.032, sm_target=0.08, lh_mac=3.3,
        elevator_frac=0.28, rudder_frac=0.40,
        aileron_inner_frac=0.55, aileron_outer_frac=0.95,
        aileron_chord_frac=0.24,
        airfoil="NACA 4412", wl_band=(3.0, 4.2),
        blurb="Slow burn: it stays up longer than the battery deserves.",
    ),
    "sport": TBStyleDef(
        name="sport", label="Sport twin-boom",
        description=(
            "Shorter span, hotter tail volume, more loading - the twin-boom "
            "layout flown for fun rather than footage."),
        ar_band=(7.0, 8.5), taper_band=(0.60, 0.85),
        washout_deg=1.5,
        vh=0.55, vv=0.038, sm_target=0.08, lh_mac=2.8,
        elevator_frac=0.30, rudder_frac=0.45,
        aileron_inner_frac=0.50, aileron_outer_frac=0.95,
        aileron_chord_frac=0.25,
        airfoil="NACA 2412", wl_band=(4.0, 5.5),
        blurb="The pusher sport model that shrugs off nose-first arrivals.",
    ),
    "mapper": TBStyleDef(
        name="mapper", label="Payload mapper",
        description=(
            "The MyTwinDream mission: maximum pod volume and the stable "
            "end of the CG band, so a camera payload rides level and the "
            "aircraft flies itself down a survey line."),
        ar_band=(8.0, 9.5), taper_band=(0.75, 1.00),
        washout_deg=WASHOUT_DEG_TB,
        vh=0.55, vv=0.040, sm_target=0.12, lh_mac=3.0,
        elevator_frac=0.28, rudder_frac=0.40,
        aileron_inner_frac=0.58, aileron_outer_frac=0.95,
        aileron_chord_frac=0.24,
        airfoil="NACA 4412", wl_band=(3.6, 5.6),
        blurb="A flying tripod with a long straight-and-level habit.",
    ),
    "trainer": TBStyleDef(
        name="trainer", label="Club twin-boom",
        description=(
            "The forgiving end: modest aspect ratio, big stab, nose-heavy "
            "CG - a first pusher that keeps the prop out of the grass and "
            "the fingers."),
        ar_band=(7.0, 9.0), taper_band=(0.75, 1.00),
        washout_deg=2.5,
        vh=0.55, vv=0.038, sm_target=0.12, lh_mac=3.0,
        elevator_frac=0.28, rudder_frac=0.42,
        aileron_inner_frac=0.58, aileron_outer_frac=0.95,
        aileron_chord_frac=0.24,
        airfoil="NACA 2412", wl_band=(3.0, 4.5),
        blurb="Hand-launch friendly and belly-lands on anything.",
    ),
}

N_MOTORS_TB = 1          # pod pusher by definition ([RT3 s.5]; the
#                          tractor-twins-on-booms option is a v3 config-axis
#                          follow-up, not this module's default physics)
N_SERVOS_TB = 4          # 2 ailerons + elevator + rudder


def _style_for(inp: dict) -> TBStyleDef:
    key = inp.get("tb_style")
    if key in TB_STYLES:
        return TB_STYLES[key]
    return TB_STYLES["fpv"]


def _tb_airfoil(style: TBStyleDef, inp: dict) -> Airfoil:
    override = inp.get("airfoil_override")
    if override in CONV_AIRFOILS:
        return Airfoil(override)
    return Airfoil(style.airfoil)


# ---------------------------------------------------------------------------
# Single full-physics evaluation of one candidate
# ---------------------------------------------------------------------------

def evaluate_twinboom(x: dict, inp: dict) -> dict:
    """Evaluate one candidate {span, ar, taper, lh} through the tailed
    pipeline with twin-boom geometry. All tail/handling math is
    stability_conv's, imported, fed a twin-boom TailSizing."""
    style = _style_for(inp)
    mission: MissionDef = MISSIONS[inp.get("mission") or "fpv_cruiser"]
    material = MATERIALS[inp["material"]]
    atm = isa(0.0)
    rho, mu = atm.density_kgm3, atm.viscosity_Pas
    wall_m = material.wall_mm / 1000.0

    # ---- search variables ---------------------------------------------------
    span = float(x["span"])
    ar_band = _band(inp, "ar_band_shift", style.ar_band, *TB_AR_HARD)
    ar = float(np.clip(x["ar"], *ar_band))
    taper = float(np.clip(x.get("taper", 0.5 * sum(style.taper_band)),
                          *style.taper_band))
    lh_mac = float(np.clip(x.get("lh", style.lh_mac), *TB_LH_MAC_BAND))

    airfoil = _tb_airfoil(style, inp)
    area = span**2 / ar                                  # S = b^2/AR
    c_root, c_tip = chords_from_area(span, area, taper)  # S = b (c_r+c_t)/2

    # ---- the wing must be able to bury its own aileron servo ----------------
    # Widen the chord until the section at the servo's OUTBOARD footprint edge
    # can hold the pocket, capped at the researched AR floor - a wing that is
    # still short there is left alone and the CAD refuses the pocket by name,
    # which is the honest outcome. Span and taper are untouched; only the
    # chord (and with it the area, and so the AR) moves.
    f_servo = min(style.aileron_inner_frac + SERVO_STATION_OFFSET
                  + SERVO_HALF_SPAN_MM / (500.0 * max(span, 1e-6)), 0.95)
    c_floor = servo_chord_floor_m(
        airfoil, material.wall_mm, dihedral_deg=DIHEDRAL_DEG_TB,
        twist_deg=style.washout_deg * f_servo)
    c_servo = c_root - (c_root - c_tip) * f_servo
    ar_servo_cap = ar
    if c_servo < c_floor - 1e-9:
        ar_servo_cap = max(TB_AR_HARD[0], ar * c_servo / c_floor)
        if ar_servo_cap < ar - 1e-9:
            ar = ar_servo_cap
            area = span**2 / ar
            c_root, c_tip = chords_from_area(span, area, taper)
            c_servo = c_root - (c_root - c_tip) * f_servo

    mac = mac_length(c_root, taper)
    y_bar = y_mac(span, taper)
    # pod ~ 2 x MAC ending at the wing root TE (derived, [RT3 s.5.2]): the
    # wing LE sits so nose + root chord = POD_LEN_MAC x MAC, nose >= 0.7 MAC
    x_w = max(POD_LEN_MAC * mac - c_root, 0.7 * mac)
    pod_len = x_w + c_root

    fin_af = fin_airfoil()
    tc_wing = airfoil.defn.thickness
    e_osw = A.oswald_efficiency(ar, 1.0)
    v = float(inp["v_cruise"])
    from .config_defs import CONV_SM_BAND        # tailed band applies
    sm_target = float(np.clip(inp.get("sm_override") or style.sm_target,
                              *CONV_SM_BAND))
    s = inp.get("wl_band_scale") or 1.0
    s_lo, s_hi = s if isinstance(s, (tuple, list)) else (s, s)
    wl_band = (style.wl_band[0] * float(s_lo), style.wl_band[1] * float(s_hi))
    payload = float(inp.get("payload_kg") or 0.0)
    washout = style.washout_deg

    # pod envelope (fixed by pod_len, not iterated)
    pod_h = max(POD_HEIGHT_FRAC * pod_len, POD_HEIGHT_MIN_M)
    pod_w = max(POD_WIDTH_MIN_M, POD_WIDTH_HEIGHT_FRAC * pod_h)
    vol_pod = POD_VOLUME_FILL * pod_len * pod_w * pod_h
    s_wet_pod = 0.90 * 2.0 * (pod_w + pod_h) * pod_len * 0.80

    # ---- coupled mass / aero / stability solve ------------------------------
    def run_pipeline(sm: float, mass0: float, ballast: float = 0.0):
        mass = mass0
        x_cg_est = x_w + 0.30 * mac      # CG anchor "1/3 of wing" [RMRC]
        stab = tail = drag = mass_bd = None
        cl_req = re_mac = v_stall = clmax3 = 0.0
        prop_d = spacing = l_boom = s_wet_booms = 0.0
        for _ in range(30):
            W = mass * G
            cl_req = A.required_cl(W, rho, v, area)       # L = W at cruise
            re_mac = reynolds(rho, v, mac, mu)            # Re = rho V c / mu
            clmax3 = A.cl_max_3d(airfoil.cl_max(re_mac), 0.0, washout)
            v_stall = A.stall_speed(W, rho, area, clmax3)

            prop_d = estimate_prop_diameter_m(mass, inp)
            spacing = boom_spacing_m(prop_d)

            for _ in range(3):
                tail = size_twinboom_tail(
                    area=area, span=span, mac=mac, x_cg=x_cg_est,
                    vh_target=style.vh, vv_target=style.vv, lh_mac=lh_mac,
                    spacing=spacing)
                stab = solve_conventional(
                    span=span, area=area, mac=mac, y_mac_m=y_bar,
                    x_le_wing=x_w, airfoil=airfoil, re_mac=re_mac, ar=ar,
                    e_oswald=e_osw, tail=tail, sm_target=sm,
                    cl_cruise=cl_req, fus_volume_m3=vol_pod,
                    sm_band=CONV_SM_BAND)
                if abs(stab.x_cg - x_cg_est) < 1e-5:
                    x_cg_est = stab.x_cg
                    break
                x_cg_est = stab.x_cg
            # booms run from the wing spar region to the tail post
            x_boom0 = x_w + 0.10 * c_root
            l_boom = max(tail.x_te - x_boom0, 0.05)
            s_wet_booms = (N_FINS * 4.0 * (BOOM_FAIRING_SIDE_MM / 1000.0)
                           * l_boom)

            drag = _build_drag_tb(cl_req, stab.cl_w_cruise, airfoil, re_mac,
                                  ar, e_osw, area, tail, pod_len, pod_w,
                                  pod_h, s_wet_pod, s_wet_booms, rho, v, mu)
            mass_bd = estimate_mass_conventional(
                material=material, wing_area=area, tail_area_h=tail.s_h,
                tail_area_v=tail.s_v,
                fus_wetted_area=s_wet_pod + s_wet_booms,
                n_motors=N_MOTORS_TB, n_servos=N_SERVOS_TB,
                payload_kg=payload)
            new_mass = mass_bd.total + ballast
            if abs(new_mass - mass) < 1e-4:
                mass = new_mass
                break
            mass = 0.5 * mass + 0.5 * new_mass
        W = mass * G
        cl_req = A.required_cl(W, rho, v, area)
        v_stall = A.stall_speed(W, rho, area, clmax3)
        return (mass, stab, tail, mass_bd, drag, cl_req, re_mac, v_stall,
                clmax3, prop_d, spacing, l_boom, s_wet_booms)

    # ---- battery placement (nose bay: the pod nose is FREE - the motor is
    # on the aft face, which is the type's whole point) -----------------------
    x_ballast = 0.015

    def bay_bounds() -> tuple[float, float, float, float]:
        x0 = max(BAY_START_FRAC_POD * pod_len, 0.020)
        length = max(BAY_LEN_FRAC_POD * pod_len, BAY_LEN_MIN_M)
        length = min(length, 0.80 * pod_len - x0)
        return x0, max(length, 0.05), x0, x0 + max(length, 0.05)

    def pack_station(mass, stab, tail, mass_bd, l_boom, s_wet_booms,
                     bay_lo, bay_len, ballast=0.0):
        """Moment balance sum(m_i x_i) + m_batt x_batt = m x_cg. The pusher
        motor mass sits at the pod's AFT face; the boom share of the shell
        mass at the boom midpoints; the battery slides in the nose bay."""
        m = mass_bd
        m_at_motors = min(PACK_PER_MOTOR * N_MOTORS_TB, 0.8 * m.power_system)
        m_batt = max(m.power_system - m_at_motors, 1e-6)
        x_wing = x_w + STRUCT_CHORD_CENTROID * mac      # Torenbeek 0.42c
        # split the body shell mass by wetted share: pod at 0.45 pod_len,
        # booms at their own midpoint
        wet_total = max(s_wet_pod + s_wet_booms, 1e-9)
        x_boom0 = x_w + 0.10 * c_root
        x_body = ((s_wet_pod * 0.45 * pod_len
                   + s_wet_booms * (x_boom0 + 0.5 * l_boom)) / wet_total)
        x_tail = ((tail.s_h * tail.x_ac_h + tail.s_v * tail.x_ac_v)
                  / max(tail.s_h + tail.s_v, 1e-9))
        moment = (
            m.structure_wing * x_wing
            + m.structure_body * x_body
            + m.structure_fins * x_tail
            + m_at_motors * pod_len                    # PUSHER: aft face
            + 2 * SERVO_MASS * (x_w + 0.75 * mac)      # aileron servos
            + 2 * SERVO_MASS * (bay_lo + 0.9 * bay_len)  # elev/rud servos
            + m.receiver_wiring * (bay_lo + 0.35 * bay_len)
            + m.payload * (bay_lo + 0.10 * bay_len)    # camera in the nose
            + ballast * x_ballast
        )
        x_batt = (mass * stab.x_cg - moment) / m_batt
        return x_batt, m_batt, moment

    def ballast_needed(mass, stab, m_batt, moment, ballast, batt_lo):
        moment_fixed = moment - ballast * x_ballast
        num = moment_fixed + m_batt * batt_lo - mass * stab.x_cg
        den = max(stab.x_cg - x_ballast, 1e-4)
        return max(num / den, 0.0)

    sm = sm_target
    ballast = 0.0
    mass_guess = 0.35 + 2.8 * area
    from .config_defs import CONV_SM_BAND as _SMB

    def solve_at(sm_v, mass0, ballast_v):
        r = run_pipeline(sm_v, mass0, ballast_v)
        bay_x0, bay_len, lo, hi = bay_bounds()
        xb_, mbatt_, mom_ = pack_station(r[0], r[1], r[2], r[3], r[11],
                                         r[12], lo, bay_len, ballast_v)
        return r, (bay_x0, bay_len, lo, hi), xb_, mbatt_, mom_

    res, bay, x_batt, m_batt, moment = solve_at(sm, mass_guess, 0.0)
    batt_lo, batt_hi = bay[2], bay[3]
    if inp.get("sm_override") is None:
        target = 0.5 * (batt_lo + batt_hi)
        sm_lo = max(_SMB[0], 0.75 * sm_target)
        for _ in range(8):
            if batt_lo <= x_batt <= batt_hi:
                break
            sm_new = float(np.clip(
                sm + (x_batt - target) * m_batt / max(res[0] * res[1].mac, 1e-9),
                sm_lo, _SMB[1]))
            if abs(sm_new - sm) < 1e-4:
                break
            sm = sm_new
            res, bay, x_batt, m_batt, moment = solve_at(sm, res[0], ballast)
            batt_lo, batt_hi = bay[2], bay[3]
    for _ in range(5):
        if x_batt >= batt_lo - 1e-9:
            break
        add = ballast_needed(res[0], res[1], m_batt, moment, ballast, batt_lo)
        if add <= 1e-5:
            break
        ballast = min(ballast + add, 0.35 * res[0])
        res, bay, x_batt, m_batt, moment = solve_at(sm, res[0], ballast)
        batt_lo, batt_hi = bay[2], bay[3]
        x_batt = max(x_batt, batt_lo)

    (mass, stab, tail, mass_bd, drag, cl_req, re_mac, v_stall, clmax3,
     prop_d, spacing, l_boom, s_wet_booms) = res
    bay_x0, bay_len = bay[0], bay[1]
    if ballast > 1e-5:
        mass_bd.components = dict(mass_bd.components)
        mass_bd.components["nose ballast"] = ballast
    W = mass * G
    boom_section = boom_tube_mm(l_boom)
    x_boom0 = x_w + 0.10 * c_root

    # ---- overall envelope ---------------------------------------------------
    # z = 0 on the pod mid-height / thrust line; high wing on the pod deck,
    # booms at wing level, fins standing on the booms.
    wing_z = 0.5 * pod_h
    tip_rise = 0.5 * span * math.tan(math.radians(DIHEDRAL_DEG_TB))
    z_top = max(wing_z + tail.h_v,               # fins on the boom line
                wing_z + tip_rise + 0.5 * tc_wing * c_tip)
    height_total = z_top + 0.5 * pod_h + 0.005
    length_total = tail.x_te + 0.005

    # ---- constraints --------------------------------------------------------
    wl = mass / area
    sf = mission.stall_factor
    cons: list[dict] = []

    def con(name, ok, value, limit, msg):
        cons.append({"name": name, "ok": bool(ok), "value": float(value),
                     "limit": float(limit), "message": msg})

    con("box_span", span <= inp["box_w"] + 1e-9, span, inp["box_w"],
        "Wingspan exceeds box width - reduce span or enlarge the box.")
    con("box_length", length_total <= inp["box_l"] + 1e-9, length_total,
        inp["box_l"],
        "Pod nose to tail post exceeds box length - the booms need their "
        "arm; give it more box or less span.")
    con("box_height", height_total <= inp["box_h"] + 1e-9, height_total,
        inp["box_h"], "Aircraft height (boom fins) exceeds box height.")
    if inp.get("v_stall_target"):
        con("stall_speed", v_stall <= inp["v_stall_target"] * 1.001, v_stall,
            inp["v_stall_target"],
            "Stall speed above your target - more wing area needed.")
    else:
        con("stall_margin", v >= sf * v_stall, v / max(v_stall, 0.1), sf,
            f"Cruise speed must be >= {sf:.2f} x stall speed.")
    con("wing_loading", wl_band[0] * 0.8 <= wl <= wl_band[1] * 1.2, wl,
        wl_band[1],
        f"Wing loading {wl * 10:.0f} g/dm2 outside the twin-boom "
        f"{style.name} band ({wl_band[0] * 10:.0f}-{wl_band[1] * 10:.0f} "
        "g/dm2; [RT3 row 37] holds the conventional 30-50 band at this "
        "scale).")
    con("static_margin",
        _SMB[0] - 1e-6 <= stab.static_margin <= _SMB[1] + 1e-6,
        stab.static_margin, sm_target,
        "Static margin outside the tailed band 0.05-0.15 ([MIT]).")
    con("pitch_stability", stab.dcm_dalpha < 0, stab.dcm_dalpha, 0.0,
        "dCm/dalpha must be negative (CG ahead of NP).")
    con("cambered_section",
        (not airfoil.defn.reflexed) and airfoil.cm0_geometric() <= 1e-3,
        airfoil.cm0_geometric(), 0.0,
        "A tailed twin-boom flies a plain cambered section; the tail trims.")
    con("vh_band", TB_VH_BAND[0] - 1e-6 <= stab.vh <= TB_VH_BAND[1] + 1e-6,
        stab.vh, TB_VH_BAND[1],
        f"V_H outside the {TB_VH_BAND[0]:.2f}-{TB_VH_BAND[1]:.2f} band "
        "([MIT]/[SCHOLZ] via [RT3 row 34]).")
    con("vv_band", TB_VV_BAND[0] - 1e-6 <= stab.vv <= TB_VV_BAND[1] + 1e-6,
        stab.vv, TB_VV_BAND[1],
        f"V_V outside the {TB_VV_BAND[0]:.3f}-{TB_VV_BAND[1]:.3f} band "
        "(split half per boom fin, [RT3 row 34]).")
    con("tail_arm",
        TB_LH_MAC_BAND[0] - 1e-6 <= stab.l_h / mac
        <= TB_LH_MAC_BAND[1] + 1e-6,
        stab.l_h / mac, TB_LH_MAC_BAND[1],
        "Tail arm outside the 2.5-3.5 MAC band the booms exist to buy "
        "([RT3 row 33]).")
    con("aspect_ratio",
        TB_AR_HARD[0] - 1e-6 <= ar <= TB_AR_HARD[1] + 1e-6, ar,
        TB_AR_HARD[1],
        "Wing AR outside the twin-boom 7-10 band (Skyhunter derives to 9, "
        "[RT3 row 36]).")
    con("boom_spacing",
        spacing >= BOOM_SPACING_PROP_MULT_MIN * prop_d - 1e-9
        and spacing >= prop_d + 2 * PROP_TIP_CLEARANCE_M - 1e-9,
        spacing / max(prop_d, 1e-9), BOOM_SPACING_PROP_MULT_MIN,
        f"Boom spacing must be >= 1.15 x the estimated prop diameter "
        f"({prop_d * 1000:.0f} mm) AND >= D + 2 x 15 mm tip clearance "
        "([RT3 row 35]).")
    con("boom_stiffness",
        boom_section["I_mm4"] >= boom_section["I_required_mm4"] - 1e-9,
        boom_section["I_mm4"], boom_section["I_required_mm4"],
        "Boom section falls below the Ø8x1 carbon-per-metre stiffness "
        "proxy ([RT3 s.5.2]) - a flexing boom converges on the prop disk.")
    con("reynolds", re_mac >= RE_MIN_HARD, re_mac, RE_MIN_HARD,
        "Chord too small - Reynolds number too low for reliable airfoil "
        "performance.")
    con("battery_position", batt_lo - 1e-9 <= x_batt <= batt_hi + 1e-9,
        x_batt, batt_hi,
        "The pack cannot sit inside the pod bay and still balance the "
        "aircraft - a longer pod or a lighter tail is needed.")
    con("nose_ballast", ballast <= 0.15 * mass + 1e-9,
        ballast / max(mass, 1e-9), 0.15,
        "This layout needs an unreasonable amount of nose ballast.")

    # ---- handling (tailed thresholds, imported solver) ----------------------
    hq = conv_handling(
        area=area, span=span, mac=mac, y_bar=y_bar, cl_cruise=cl_req,
        a_wing=stab.a_wing, a_total=stab.a_total,
        dihedral_deg=DIHEDRAL_DEG_TB, wing_position="high", tail=tail,
        vh=stab.vh, vv=stab.vv, l_v=stab.l_v, a_tail=stab.a_tail,
        fus_volume_m3=vol_pod, fus_height_m=pod_h,
        static_margin=stab.static_margin, cl_max=clmax3,
        elevator_frac=style.elevator_frac)
    con("directional_stability", hq.cn_beta > 0.03, hq.cn_beta, 0.03,
        "Directional stiffness below the tailed floor - more fin volume "
        "on the booms.")
    con("roll_stability", hq.cl_beta < 0.0, hq.cl_beta, 0.0,
        "No dihedral effect: 2 deg on a high wing is the fix.")
    con("elevator_authority", hq.elevator_trim_used <= 0.75,
        hq.elevator_trim_used, 0.75,
        "Pulling to CL_max eats most of the elevator travel.")

    feasible = all(c["ok"] for c in cons)

    # ---- cost ---------------------------------------------------------------
    ld_cruise = A.lift_to_drag(cl_req, drag.cd)
    sd_weight = float(inp.get("sd_weight") or 1.0)
    cost = sd_weight * 100.0 / max(ld_cruise, 1e-3)
    wl_pen_w = inp.get("wl_pen_weight")
    wl_pen_w = 1.0 if wl_pen_w is None else float(wl_pen_w)
    cost += 6.0 * wl_pen_w * _band_center_pen(wl, *wl_band)
    cost += 4.0 * _band_center_pen(ar, *style.ar_band)
    cost += 2.0 * _band_center_pen(taper, *style.taper_band)
    if re_mac < RE_MIN_GOOD:
        cost += 300.0 * ((RE_MIN_GOOD - re_mac) / RE_MIN_GOOD) ** 2
    span_frac = float(inp.get("span_pref_frac") or 0.88)
    span_target = span_frac * min(inp["box_w"], inp["box_l"] / 0.62, 2.6)
    if span < span_target:
        cost += 25.0 * ((span_target - span) / span_target) ** 2
    cost += 40.0 * (ballast / max(mass, 1e-9)) ** 2
    v_goal = float(inp.get("v_stall_goal") or 0.0)
    if v_goal > 0 and v_stall > v_goal:
        cost += 90.0 * ((v_stall - v_goal) / v_goal) ** 2
    for c in cons:
        if not c["ok"]:
            rel = abs(c["value"] - c["limit"]) / (abs(c["limit"]) + 1e-9)
            cost += 1500.0 + 3000.0 * min(rel, 2.0)

    # ---- assemble the design dict (V3_PLAN.md: conventional schema + the
    # geometry.booms block; tail arrangement names the twin-boom H-tail) -----
    motors = [{"x": float(pod_len), "y": 0.0, "z": 0.0, "type": "pusher"}]
    mount = _motor_mount_spec(inp, mass, motors, material.wall_mm)
    bay_w = max(pod_w - 2.0 * wall_m, 0.02)
    bay_d = BAY_DEPTH_FRAC * pod_h
    y_boom = 0.5 * spacing

    design = {
        "id": "",
        "airplane_type": "twin_boom",
        "planform": style.name,
        "planform_label": style.label,
        "mission": mission.name,
        "config": {
            "motor_layout": "pusher",       # type default (V3_PLAN.md)
            "n_motors": N_MOTORS_TB,
            "tail_type": "conventional",    # H-tail between booms default
            "wing_position": "high",        # fixed - [RT3 s.5.1] practice
        },
        "geometry": {
            "span_m": float(span),
            "area_m2": float(area),
            "aspect_ratio": float(ar),
            "taper": float(taper),
            "sweep_le_deg": 0.0,
            "dihedral_deg": float(DIHEDRAL_DEG_TB),
            "washout_deg": float(washout),
            "root_incidence_deg": float(stab.incidence_w_deg),
            "root_chord_m": float(c_root),
            "tip_chord_m": float(c_tip),
            "airfoil": airfoil.name,
            "fin_airfoil": fin_af.name,
            "wing_position": "high",
            "fuselage": {                      # the POD ([RT3 s.5.2])
                "length_m": float(pod_len),
                "width_m": float(pod_w),
                "height_m": float(pod_h),
                "x_wing_le_m": float(x_w),
                "wetted_area_m2": float(s_wet_pod),
                "pod_len_mac": float(pod_len / mac),
                "bay": {
                    "bay_start_m": float(bay_x0),
                    "bay_length_m": float(bay_len),
                    "bay_width_m": float(bay_w),
                    "bay_depth_m": float(bay_d),
                    "bay_wall_m": float(wall_m),
                },
            },
            "booms": {                          # V3_PLAN.md geometry block
                "y_m": [float(-y_boom), float(y_boom)],
                "x_start_m": float(x_boom0),
                "length_m": float(l_boom),
                "spacing_m": float(spacing),
                "prop_diameter_est_m": float(prop_d),
                "spacing_over_prop_d": float(spacing / max(prop_d, 1e-9)),
                "section_mm": boom_section,
            },
            "tail": {
                "arrangement": "h_tail_twin_boom",  # stab between the booms
                "x_le_h_m": float(tail.x_le_h),
                "span_h_m": float(tail.span_h),     # = boom spacing
                "c_root_h_m": float(tail.c_root_h),
                "c_tip_h_m": float(tail.c_tip_h),
                "area_h_m2": float(tail.s_h),
                "V_H": float(stab.vh),
                "elevator_chord_frac": float(style.elevator_frac),
                "x_le_v_m": float(tail.x_le_v),
                "height_v_m": float(tail.h_v),      # per fin
                "c_root_v_m": float(tail.c_root_v),
                "c_tip_v_m": float(tail.c_tip_v),
                "area_v_m2": float(tail.s_v),       # TOTAL, both fins
                "n_fins": N_FINS,
                "fin_y_m": [float(-y_boom), float(y_boom)],
                "V_V": float(stab.vv),
                "rudder_chord_frac": float(style.rudder_frac),
                "incidence_h_deg": float(stab.incidence_h_deg),
            },
            "ailerons": {
                "inner_frac": float(style.aileron_inner_frac),
                "outer_frac": float(style.aileron_outer_frac),
                "chord_frac": float(style.aileron_chord_frac),
            },
            # What the aileron servo cost the planform, so a widened chord is
            # never mistaken for the aerodynamic optimum (see the chord floor
            # near the top of this module).
            "servo_chord_floor": {
                "chord_floor_m": float(c_floor),
                "chord_at_servo_m": float(c_servo),
                "station_frac": float(f_servo),
                "ar_capped": bool(ar_servo_cap < float(np.clip(
                    x["ar"], *ar_band)) - 1e-9),
                "fits": bool(c_servo >= c_floor - 1e-9),
            },
            "motors": motors,
            "motor_mount": mount,
            "battery_x_m": float(np.clip(x_batt, batt_lo, batt_hi)),
            "battery_x_required_m": float(x_batt),
            "length_total_m": float(length_total),
            "height_total_m": float(height_total),
            "control_surfaces": ["aileron_left", "aileron_right",
                                 "elevator", "rudder"],
            "wall_mm": float(material.wall_mm),
            "build_method": inp["build_method"],
        },
        "aero": {
            "cl_cruise": float(cl_req),
            "cd0": float(drag.cd0), "cd_cruise": float(drag.cd),
            "cd0_breakdown": {"wing": drag.cd0_wing,
                              "fuselage": drag.cd0_fuselage,
                              "tail": drag.cd0_tail,
                              "interference": drag.cd0_interference},
            "ld_cruise": float(ld_cruise),
            "specific_drag": float(1.0 / ld_cruise if ld_cruise > 0 else 0.0),
            "oswald_e": float(e_osw), "re_mac": float(re_mac),
            "v_cruise_ms": float(v), "v_stall_ms": float(v_stall),
            "stall_margin": float(v / max(v_stall, 0.1)),
            "wing_loading_kgm2": float(wl),
            "cl_max_2d": float(airfoil.cl_max(re_mac)),
            "cl_max_3d": float(clmax3),
            "alpha_cruise_deg": float(stab.alpha_cruise_deg),
            "cm0_section": float(airfoil.cm0(re_mac)),
            "cm0_section_geometric": float(airfoil.cm0_geometric()),
            # pusher efficiency note ([RT3 s.7] / config_axes): guidance-
            # level, the app sizes no thrust - a clean twin-boom pusher
            # pays ~3% where a pod pusher pays ~5%
            "pusher_efficiency_penalty": 0.03,
        },
        "stability": {
            "mac_m": float(stab.mac), "y_mac_m": float(stab.y_mac),
            "x_le_mac_m": float(stab.x_le_mac),
            "x_ac_wing_m": float(stab.x_ac_wing),
            "x_np_m": float(stab.x_np), "x_cg_m": float(stab.x_cg),
            "static_margin": float(stab.static_margin),
            "cg_pct_mac": float(stab.cg_pct_mac),
            "cg_mm_from_nose": float(stab.x_cg * 1000.0),
            "vh": float(stab.vh), "vv": float(stab.vv),
            "s_h_m2": float(tail.s_h), "s_v_m2": float(tail.s_v),
            "l_h_m": float(stab.l_h), "l_v_m": float(stab.l_v),
            "fin_height_m": float(tail.h_v),
            "fin_chord_m": float(tail.c_mac_v),
            "x_fin_ac_m": float(tail.x_ac_v),
            "dcm_dalpha": float(stab.dcm_dalpha),
            "cm0_wing": float(stab.cm0_wing),
            "cm_trim_residual": float(stab.cm_residual),
            "tail_contribution": {
                "x_np_shift_tail_m": float(stab.x_np_shift_tail_m),
                "x_np_shift_fus_m": float(stab.x_np_shift_fus_m),
                "deps_dalpha": float(stab.deps_dalpha),
                "cl_h_cruise": float(stab.cl_h_cruise),
                "incidence_h_deg": float(stab.incidence_h_deg),
                "decalage_deg": float(stab.decalage_deg),
                "eta_h": 0.90,
            },
            "cn_beta": float(hq.cn_beta),
            "cn_beta_fin": float(hq.cn_beta_fin),
            "cn_beta_fus": float(hq.cn_beta_fus),
            "cl_beta": float(hq.cl_beta),
            "cl_beta_dihedral": float(hq.cl_beta_dihedral),
            "cl_beta_wing_pos": float(hq.cl_beta_wing_pos),
            "cl_beta_fin": float(hq.cl_beta_fin),
            "roll_yaw_ratio": float(hq.ratio),
            "spiral_b": float(hq.spiral_b),
            "elevator_authority_cm": float(hq.elevator_authority),
            "elevator_trim_used": float(hq.elevator_trim_used),
            "handling_notes": list(hq.notes),
            "washout_deg": float(washout),
            "notes": list(stab.notes),
        },
        "power_system": {
            "n_motors": N_MOTORS_TB,
            "pack_mass_kg": float(mass_bd.power_system),
            "motor_positions": [dict(mo) for mo in motors],
        },
        "mass": {
            "total_kg": float(mass),
            "weight_n": float(W),
            "nose_ballast_kg": float(ballast),
            "breakdown_kg": mass_bd.components,
        },
        "constraints": cons,
        "feasible": bool(feasible),
        "binding": [c["name"] for c in cons if not c["ok"]],
        "notes": list(stab.notes),
        "cost": float(cost),
    }
    design["notes"].append(
        f"Boom spacing {spacing * 1000:.0f} mm = "
        f"{spacing / prop_d:.2f} x the estimated {prop_d * 1000:.0f} mm "
        "prop (>= 1.15 hard, >= D + 30 mm tip clearance [RT3 row 35]); "
        "prop estimated from the power-mass motor class - override with "
        "prop_diameter_m if the real prop is known.")
    return design


def _build_drag_tb(cl_req, cl_w, airfoil, re_mac, ar, e_osw, area, tail,
                   pod_len, pod_w, pod_h, s_wet_pod, s_wet_booms, rho, v, mu):
    """CD0 build-up for a pod-and-twin-boom airframe.

    Pod by the wetted-area method with Raymer's fuselage form factor
    FF = 1 + 60/f^3 + f/400 (eq. 12.31) at the pod's own LOW fineness ratio
    - a 2-MAC pod is stubby and pays for it, which is the honest price of
    the layout. Booms as slender bodies at FF 1.10 (fineness > 15: the form
    term is negligible, the factor covers the fairing joints). Tail by the
    same thin-symmetric method as the conventional. Interference at 11% of
    the build-up: this airframe has MORE junctions than a conventional
    (wing-pod, two boom-wing, two fin-boom, two stab-fin).
    """
    cd0_wing = airfoil.cd_at_cl(cl_w, re_mac)
    re_f = reynolds(rho, v, max(pod_len, 0.05), mu)
    d_eq = math.sqrt(4.0 * pod_w * pod_h / math.pi)
    f = max(pod_len / max(d_eq, 1e-3), 2.0)
    ff_pod = 1.0 + 60.0 / f**3 + f / 400.0            # Raymer eq. 12.31
    cd0_pod = A.flat_plate_cf(max(re_f, 3e4)) * ff_pod * (s_wet_pod / area)
    cd0_booms = A.flat_plate_cf(max(re_f, 3e4)) * 1.10 * (s_wet_booms / area)
    re_t = reynolds(rho, v, max(tail.c_mac_h, 0.02), mu)
    cd0_tail = (A.flat_plate_cf(max(re_t, 3e4)) * 2.05
                * ((tail.s_h + tail.s_v) / area) * 1.25)
    subtotal = cd0_wing + cd0_pod + cd0_booms + cd0_tail
    cd0_int = 0.11 * subtotal
    cd0 = subtotal + cd0_int
    cdi = cl_req**2 / (math.pi * ar * e_osw)          # CDi = CL^2/(pi AR e)
    return A.DragBreakdown(cd0_wing, cd0_pod + cd0_booms, cd0_tail, cd0_int,
                           cd0, cdi, cd0 + cdi)


# ---------------------------------------------------------------------------
# Search driver (mirrors optimize_conventional's contract)
# ---------------------------------------------------------------------------

def optimize_twinboom(inp: dict) -> dict:
    """Sweep (span, AR, taper, tail arm) inside the twin-boom bands and the
    box, polish with Nelder-Mead, return the full design dict. Never an
    exception for an infeasible request - closest design returned flagged."""
    style = _style_for(inp)

    b_hi = min(float(inp["box_w"]), float(inp["box_l"]) / 0.62, 2.6)
    b_lo = min(max(0.60, 0.55 * b_hi), 0.92 * b_hi)

    ar_lo, ar_hi = _band(inp, "ar_band_shift", style.ar_band, *TB_AR_HARD)
    if inp.get("ar_target"):
        ar_lo = ar_hi = float(np.clip(inp["ar_target"], *TB_AR_HARD))
    tp_lo, tp_hi = style.taper_band
    if inp.get("taper_override") is not None:
        tp_lo = tp_hi = float(np.clip(inp["taper_override"],
                                      *style.taper_band))
    lh_lo, lh_hi = TB_LH_MAC_BAND

    spans = np.linspace(b_lo, b_hi, 5)
    ars = (np.linspace(ar_lo, ar_hi, 3) if ar_hi > ar_lo + 1e-6
           else np.array([ar_lo]))
    tapers = (np.linspace(tp_lo, tp_hi, 2) if tp_hi > tp_lo + 1e-6
              else np.array([tp_lo]))
    lhs = np.linspace(lh_lo, lh_hi, 3)

    grid = [{"span": float(b), "ar": float(a), "taper": float(t),
             "lh": float(lh)}
            for b in spans for a in ars for t in tapers for lh in lhs]
    for s in (inp.get("seeds") or []):
        try:
            grid.append({
                "span": float(np.clip(s["span"], 0.3, b_hi)),
                "ar": float(np.clip(s["ar"], ar_lo, ar_hi)),
                "taper": float(np.clip(s.get("taper", 0.5 * (tp_lo + tp_hi)),
                                       tp_lo, tp_hi)),
                "lh": float(np.clip(s.get("lh", style.lh_mac), lh_lo, lh_hi)),
            })
        except Exception:
            continue

    best = None
    for xd in grid:
        try:
            d = evaluate_twinboom(xd, inp)
        except Exception:
            continue
        if best is None or d["cost"] < best["cost"]:
            best = d
    if best is None:
        raise RuntimeError("optimizer could not evaluate any candidate design")

    g = best["geometry"]
    x0 = [g["span_m"], g["aspect_ratio"], g["taper"],
          best["stability"]["l_h_m"] / best["stability"]["mac_m"]]

    def unpack(xv):
        return {"span": float(np.clip(xv[0], 0.3, b_hi)),
                "ar": float(np.clip(xv[1], ar_lo, ar_hi)),
                "taper": float(np.clip(xv[2], tp_lo, tp_hi)),
                "lh": float(np.clip(xv[3], lh_lo, lh_hi))}

    def f(xv):
        try:
            return evaluate_twinboom(unpack(xv), inp)["cost"]
        except Exception:
            return 1e9

    try:
        r = sciopt.minimize(f, x0, method="Nelder-Mead",
                            options={"maxfev": 70, "xatol": 1e-3,
                                     "fatol": 0.4})
        polished = evaluate_twinboom(unpack(r.x), inp)
        if polished["cost"] <= best["cost"]:
            best = polished
    except Exception:
        pass

    best["id"] = uuid.uuid4().hex[:12]
    if not best["feasible"]:
        msgs = [c["message"] for c in best["constraints"] if not c["ok"]]
        best["notes"].append(
            "Design is the CLOSEST FEASIBLE result - binding constraint(s): "
            + "; ".join(msgs))
    return best


# ---------------------------------------------------------------------------
# Five twin-boom characters
# ---------------------------------------------------------------------------

TB_VARIANTS: list[dict] = [
    {"key": "tb_fpv", "name": "FPV Hauler", "style": "fpv",
     "tagline": "Skyhunter proportions: camera nose, pusher prop, H-tail "
                "on a real arm.",
     "knobs": {}},
    {"key": "tb_endurance", "name": "Endurance Cruiser", "style": "endurance",
     "tagline": "Top-of-band aspect ratio and a floater section - built to "
                "loiter.",
     "knobs": {"sd_weight": 1.8, "span_pref_frac": 0.95}},
    {"key": "tb_sport", "name": "Sport Pusher", "style": "sport",
     "tagline": "Shorter, hotter, heavier loading - the fun slice of the "
                "layout.",
     "knobs": {"span_pref_frac": 0.78, "v_cruise_mult": 1.15}},
    {"key": "tb_mapper", "name": "Payload Mapper", "style": "mapper",
     "tagline": "Maximum pod and the stable end of the CG band - a survey "
                "camera's aeroplane.",
     "knobs": {"span_pref_frac": 0.92}},
    {"key": "tb_trainer", "name": "Club Pusher", "style": "trainer",
     "tagline": "Forgiving bands and a prop that cannot bite the grass or "
                "the launcher's hand.",
     "knobs": {"v_stall_goal_frac": 0.9}},
]

_TB_PRIMARY = "tb_fpv"


def _tb_guidance(vd: dict, d: dict) -> list[dict]:
    g, a, st = d["geometry"], d["aero"], d["stability"]
    fus, t, booms = g["fuselage"], g["tail"], g["booms"]
    secs: list[dict] = []
    if not d.get("feasible", True):
        bad = [c["message"] for c in d["constraints"] if not c["ok"]]
        secs.append({"title": "Compromises - read this first",
                     "body": "\n".join(f"- {m}" for m in bad)})
    secs.append({"title": "CG, balance and rigging", "body": "\n".join([
        f"- Balance point: {st['cg_mm_from_nose']:.0f} mm from the pod nose "
        f"({st['cg_pct_mac']:.0f}% of the wing chord - the [RMRC] '1/3 of "
        "wing' anchor).",
        f"- The bay runs {fus['bay']['bay_start_m'] * 1000:.0f}-"
        f"{(fus['bay']['bay_start_m'] + fus['bay']['bay_length_m']) * 1000:.0f}"
        f" mm; the pack sits at {g['battery_x_m'] * 1000:.0f} mm - the whole "
        "nose is payload space because the motor lives on the aft face.",
        f"- Booms: {booms['spacing_m'] * 1000:.0f} mm apart "
        f"({booms['spacing_over_prop_d']:.2f} x the estimated "
        f"{booms['prop_diameter_est_m'] * 1000:.0f} mm prop), "
        f"{booms['length_m'] * 1000:.0f} mm long on "
        f"Ø{booms['section_mm']['tube_od_mm']:.0f} mm carbon tube "
        "(the [RT3 s.5.2] stiffness rule - a flexing boom converges on "
        "the disk).",
        f"- Rig the stab at {t['incidence_h_deg']:+.1f} deg against "
        f"{g['root_incidence_deg']:+.1f} deg of wing incidence; motor leads "
        "run INSIDE the pod to the bay - no belly hole on this type.",
    ])})
    secs.append({"title": f"Why this variant - {vd['name']}",
                 "body": "\n".join([
        f"- This is the {vd['name']}, a {d['planform_label'].lower()}: "
        f"{g['span_m'] * 1000:.0f} mm span at AR {g['aspect_ratio']:.1f}, "
        f"pod {fus['length_m'] * 1000:.0f} mm "
        f"({fus['pod_len_mac']:.1f} x MAC), tail V_H {t['V_H']:.2f} / "
        f"V_V {t['V_V']:.3f} split across two boom fins on a "
        f"{st['l_h_m'] / st['mac_m']:.1f}-chord arm.",
        f"- Numbers: {a['wing_loading_kgm2'] * 10:.0f} g/dm2 loading, "
        f"{a['v_stall_ms']:.1f} m/s stall vs {a['v_cruise_ms']:.1f} m/s "
        f"cruise ({a['stall_margin']:.2f}x), L/D {a['ld_cruise']:.1f} "
        "(a clean twin-boom pusher gives back ~3% where a pod pusher "
        "loses 5 [RT3 s.7]).",
    ])})
    return secs


def generate_twinboom_variants(inp: dict) -> list[dict]:
    """Five twin-boom characters - same contract as the other generators:
    always exactly five, one primary, infeasible characters flagged."""
    ctx: dict = {"seeds": []}
    designs: dict[str, dict] = {}

    def _run(vd: dict) -> dict:
        knobs = dict(vd["knobs"])
        v_mult = knobs.pop("v_cruise_mult", None)
        vs_goal = knobs.pop("v_stall_goal_frac", None)
        merged = {**inp, **knobs, "tb_style": vd["style"],
                  "seeds": ctx.get("seeds", [])}
        if v_mult:
            merged["v_cruise"] = inp["v_cruise"] * float(v_mult)
        ref = ctx.get("v_stall_ref", 0.0)
        if vs_goal and ref > 0:
            merged["v_stall_goal"] = float(vs_goal) * ref
        try:
            return optimize_twinboom(merged)
        except Exception:
            d = optimize_twinboom({**inp, "tb_style": vd["style"]})
            d["notes"].append(
                f"The {vd['name']} tuning could not be solved for these "
                "inputs; showing the untuned solution for this character.")
            return d

    def _seed(d: dict) -> dict:
        g = d["geometry"]
        return {"span": g["span_m"], "ar": g["aspect_ratio"],
                "taper": g["taper"],
                "lh": d["stability"]["l_h_m"] / d["stability"]["mac_m"]}

    lead_def = next(v for v in TB_VARIANTS if v["key"] == _TB_PRIMARY)
    lead = _run(lead_def)
    designs[_TB_PRIMARY] = lead
    ctx["v_stall_ref"] = lead["aero"]["v_stall_ms"]
    ctx["seeds"].append(_seed(lead))
    for vd in TB_VARIANTS:
        if vd["key"] == _TB_PRIMARY:
            continue
        d = _run(vd)
        designs[vd["key"]] = d
        ctx["seeds"].append(_seed(d))

    ordered = [designs[vd["key"]] for vd in TB_VARIANTS]
    traits = traits_generic(ordered)
    out: list[dict] = []
    for vd, d, tr in zip(TB_VARIANTS, ordered, traits):
        d["character"] = {"key": vd["key"], "name": vd["name"],
                          "tagline": vd["tagline"]}
        d["guidance"] = _tb_guidance(vd, d)
        out.append({
            "id": d["id"],
            "key": vd["key"],
            "name": vd["name"],
            "tagline": vd["tagline"],
            "planform": d["planform"],
            "airplane_type": "twin_boom",
            "primary": vd["key"] == _TB_PRIMARY,
            "traits": tr,
            "design": d,
        })
    return out
