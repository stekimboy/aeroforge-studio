"""CANARD airplane design solve + variant generation (V3_PLAN.md wave 2a).

NEW in v3. This module lives BESIDE the flying-wing and conventional paths and
is imported by neither: dispatch registration is the integration wave's job,
so everything here is callable standalone (`evaluate_canard`,
`optimize_canard`, `generate_canard_variants` mirror
physics/conventional.py's entry points exactly).

The airframe: a lifting FOREPLANE (the canard) on the nose, the MAIN WING aft,
a slab-sided fuselage between them whose gap IS the equipment bay, twin fixed
fins at the wingtips extended rearward, and a single PUSHER motor on the aft
fuselage face ("Most, but not all, canard aircraft employ pusher
arrangements", RESEARCH_TYPES_V3.md s.2.2 [LEN-CAN s.5]; s.10.4 matrix).
Nothing protrudes ahead of the x = 0 nose datum (v1 invariant).

The defining safety property - THE CANARD STALLS FIRST, ALWAYS - is carried
as the lift-coefficient ratio CLf/CLr = 1.4-1.6 with a hard floor > 1.0
("This ratio must be greater than 1 to satisfy stability requirements and is
typically on the order of 1.4 to 1.6", RESEARCH_TYPES_V3.md s.2.2
[LEN-CAN Eq. 2-7]), recorded in the design dict as the loading margin plus an
explicit stall-first alpha margin computed from both surfaces' CL_max.

Every band cites RESEARCH_TYPES_V3.md (section + quick-reference row); the
design dict mirrors the conventional dict's key names wherever the concept
matches and adds the `geometry.canard` / `geometry.fins` blocks of the
V3_PLAN.md schema plus the v3 `config` block. Objective is the SAME specific
drag 100/(L/D) family as the flying-wing / conventional optimizers, so an
"any"-mode comparison across types stays apples to apples.
"""
from __future__ import annotations

import math
import uuid
from dataclasses import dataclass, field

import numpy as np
from scipy import optimize as sciopt

from . import aero as A
from .airfoils import LIBRARY, Airfoil, AirfoilDef, fin_airfoil
from .atmosphere import isa, reynolds
from .config_defs import MISSIONS, MissionDef
from .conventional import (
    FUS_HEIGHT_MIN_M, FUS_VOLUME_FILL, FUS_WET_PERIM, FUS_WET_TAPER,
    FUS_WIDTH_HEIGHT_FRAC, FUS_WIDTH_MIN_M, STRUCT_CHORD_CENTROID,
    traits_generic,
)
from .optimizer import (
    G, RE_MIN_GOOD, RE_MIN_HARD, _band_center_pen, _motor_mount_spec,
)
from .stability import chords_from_area, lift_slope_3d, mac_length, y_mac
from .stability_conv import AR_V, ETA_V, K_MUNK, TAPER_V
from .weights import (
    CONV_FITTINGS_KG, CONV_FUS_AREAL, CONV_PUSHRODS_KG, CONV_TAIL_AREAL,
    CONV_WING_AREAL, MATERIALS, PACK_PER_MOTOR, RECEIVER_MASS, SERVO_MASS,
    WIRING_BASE, MassBreakdown, power_system_allowance,
)

# ---------------------------------------------------------------------------
# Canard bands - RESEARCH_TYPES_V3.md s.2.2, quick-reference rows 8-15.
# ---------------------------------------------------------------------------
# Canard/wing area ratio Sc/S: recommended 0.20-0.35, default 0.25 (row 8;
# spread 0.156 Long-EZ .. 0.50 Lennon ex.2 - above ~0.4 the design is a
# tandem and physics/tandem.py owns it).
CANARD_SC_S_BAND: tuple[float, float] = (0.20, 0.35)
# CLf/CLr at every trimmed AoA: hard floor > 1.0, typical 1.4-1.6
# (row 9, [LEN-CAN Eq. 2-7]). This IS the stall-first margin in
# coefficient form.
CANARD_CL_RATIO_BAND: tuple[float, float] = (1.4, 1.6)
CANARD_CL_RATIO_FLOOR: float = 1.0
# Static margin: CG ahead of NP by k = 0.20-0.25 x MAC_rear (row 10,
# [LEN-CAN s.2 Eq. 2-2]). Larger than the tailed 0.05-0.15 band because it is
# referenced to the REAR wing's MAC alone while the two-surface NP sits far
# forward of the rear wing's AC.
CANARD_K_BAND: tuple[float, float] = (0.20, 0.25)
# Rear-wing downwash efficiency for the portion of the wing directly behind
# the canard span: 0.8 (row 11, [LEN-CAN Eq. 2-1]) - applied SPAN-RESOLVED
# (only the covered fraction of the rear wing pays it, s.3.2 note).
REAR_EFF: float = 0.80
# Canard volume V_C = Sc l_c/(S MAC): 0.5-0.9, default 0.7 (row 12, derived
# from the two Lennon worked examples - lifting canards run an order of
# magnitude above tail-style control-canard coefficients).
CANARD_VC_BAND: tuple[float, float] = (0.50, 0.90)
# Canard AR 5-8 and >= wing AR (row 13, [LEN-CAN s.4]: higher AR -> steeper
# lift slope -> the canard reaches its stall AoA first). Wing AR 4.5-6 per
# the conventional bands (s.2.2).
CANARD_AR_C_BAND: tuple[float, float] = (5.0, 8.0)
CANARD_AR_W_LIMITS: tuple[float, float] = (4.5, 6.5)
# Elevator-on-canard: plain flap, chord 0.2 x canard chord (row 14); useful
# deflection knee ~40 deg, app throws +-20 default / +-30 max (s.2.2).
CANARD_ELEVATOR_FRAC: float = 0.20
CANARD_ELEVATOR_MAX_DEG: float = 20.0
# Aft/forward lateral-area MOMENT ratio >= 1.25 (row 15, [LEN-CAN s.1 item 4,
# s.5]: "the aft side area moment behind the CG needs to [be] about 25%
# greater than that of the front area moment"). Sized at 1.35 for margin.
CLA_RATIO_MIN: float = 1.25
CLA_RATIO_TARGET: float = 1.35
# Fin area 2-9% of wing area, expect the TOP of the band ("Canards generally
# have a small moment arm to VT, requiring larger area", s.2.2 [VT canards]).
FIN_FRAC_BAND: tuple[float, float] = (0.02, 0.09)
# AC-to-AC stagger in rear MACs. Derived from the Lennon worked examples via
# V_C: ex.1 lc = 2.2 MAC + lw ~ 0.85 MAC -> D ~ 3.0; the band brackets both
# examples with the V_C 0.5-0.9 window (s.2.2 row 12 derivation).
CANARD_D_MAC_BAND: tuple[float, float] = (2.6, 4.2)
# Wing-loading band on the TOTAL lifting area (S + Sc). RESEARCH_TYPES_V3.md
# bands no canard loading explicitly; derived from the conventional sport
# band (RESEARCH_CONVENTIONAL.md s.7, 25-60 g/dm2 hard) - a canard carries
# the same structure per lifted area as a tailed sport model.
CANARD_WL_BAND: tuple[float, float] = (2.6, 5.2)

N_MOTORS_CANARD = 1     # single pusher (s.10.4 matrix: canard twins have no
#                         reference practice at this scale, s.8)
N_SERVOS_CANARD = 3     # 2 rear-wing ailerons + 1 canard elevator

# The canard flies in clean freestream ahead of everything, so it takes the
# full dynamic pressure (contrast the 0.90 a fuselage-mounted tail gets in
# stability_conv.ETA_H).
ETA_CANARD: float = 1.0

# ---------------------------------------------------------------------------
# Non-reflexed thin sections for the FOREPLANE. Turbulent-friendly NACA
# geometry, never the most laminar polar (s.2.2 [VT canards slides 27-28]:
# the Long-EZ's laminar GU25 lost lift in rain and was replaced by the Roncz
# 1145). The DEFAULT pairing is near-symmetric on BOTH surfaces (NACA 0010
# wing / NACA 0009 canard) - a deliberate, measured departure from
# full-scale cambered-canard practice, for two coupled reasons at RC cruise
# CLs (~0.2-0.3):
#   1. A cambered wing's Cm_ac must be balanced by EXTRA canard lift, and at
#      low cruise CL that pushes CLf/CLr well above the 1.4-1.6 band
#      (measured here: NACA 1410's Cm0 = -0.024 alone adds ~+0.3 to the
#      ratio at CL ~ 0.19) - the trim moment budget that a full-scale canard
#      absorbs at CL ~ 0.8 swamps the band at model loadings.
#   2. Stall-first robustness wants CL_max PARITY between the surfaces: a
#      high-camber foreplane (Cl_max 1.14) paired with a symmetric wing
#      (0.90) nearly cancels the 1.5x loading margin's alpha advantage.
# The cambered 2409/4409 stay registered for override experiments.
# Registered through the existing analytic-NACA machinery, idempotently, and
# only ADDS non-reflexed entries the flying-wing picker can never select
# (same pattern as conventional.py's section registration).
# ---------------------------------------------------------------------------
_CANARD_SECTIONS = (
    AirfoilDef("NACA 0009", "naca4", "0009", 0.09,
               "Thin symmetric canard/foreplane section (turbulent-friendly)"),
    AirfoilDef("NACA 2409", "naca4", "2409", 0.09,
               "Thin cambered canard/foreplane section (turbulent-friendly)"),
    AirfoilDef("NACA 4409", "naca4", "4409", 0.09,
               "High-lift cambered foreplane section (turbulent-friendly)"),
)
for _d in _CANARD_SECTIONS:
    LIBRARY.setdefault(_d.name, _d)


# ---------------------------------------------------------------------------
# The Lennon two-surface longitudinal solution ([LEN-CAN] s.2), shared with
# physics/tandem.py: "The design principles in this document apply to any
# two-surface design" (RESEARCH_TYPES_V3.md s.3, [LEN-CAN s.6 item 7]).
# Coordinate frame: x AFT from the nose datum, metres.
# ---------------------------------------------------------------------------

@dataclass
class TwoSurfaceSolution:
    """Longitudinal solution of a front + rear lifting-surface pair."""
    x_np: float               # neutral point [m from nose]
    x_cg: float               # CG for the requested margin k
    k_margin: float           # static margin, fraction of REAR MAC
    l_f_arm: float            # x_cg - x_ac_front (front arm, > 0)
    l_r_arm: float            # x_ac_rear - x_cg (rear arm, > 0)
    cl_f: float               # front-surface cruise CL (its own area)
    cl_r: float               # rear-surface cruise CL (its own area)
    cl_ratio: float           # CLf/CLr - the stall-first loading margin
    cl_sys: float             # system CL on S_f + S_r
    a_f: float                # front 3D lift slope [1/rad]
    a_r: float                # rear isolated 3D lift slope [1/rad]
    a_r_eff: float            # rear slope after span-resolved downwash eff.
    a_total: float            # system dCL/dalpha on S_tot [1/rad]
    eff_rear: float           # span-resolved rear efficiency (1 -> untouched)
    frac_covered: float       # rear-span fraction behind the front span
    deps_dalpha: float        # downwash slope at the rear surface
    eps_rear_deg: float       # cruise downwash angle at the rear surface
    i_f_deg: float            # front incidence (fuselage datum level)
    i_r_deg: float            # rear incidence
    decalage_deg: float       # i_f - i_r (front-positive)
    cl_sys_max: float         # usable system CL_max (front stalls first)
    margin_f_deg: float       # alpha to front-surface stall
    margin_r_deg: float       # alpha to rear-surface stall
    stall_first_margin_deg: float  # margin_r - margin_f (> 0 = front first)
    dcm_dalpha: float         # about the CG, per rad, ref S_tot x MAC_rear
    x_np_shift_fus_m: float   # Munk fuselage pull (negative = forward)
    notes: list[str] = field(default_factory=list)


def solve_two_surface(
    *, s_f: float, s_r: float, span_f: float, span_r: float,
    x_ac_f: float, x_ac_r: float, a_f: float, a_r: float, ar_f: float,
    mac_f: float, mac_r: float, cm0_f: float, cm0_r: float,
    clmax_f3: float, clmax_r3: float,
    alpha0_f_deg: float, alpha0_r_deg: float,
    k_margin: float, w_over_q: float, fus_volume_m3: float,
) -> TwoSurfaceSolution:
    """Neutral point, CG, trim and stall-first margins of a two-surface pair.

    Relations (RESEARCH_TYPES_V3.md s.2.2 unless noted):
      rear efficiency   eff = 0.8 on the rear-span fraction behind the front
                        span, span-resolved ([LEN-CAN Eq. 2-1], s.3.2)
      neutral point     slope-weighted AC of both surfaces - the lift-slope
                        generalization of Lennon's N = Af L/(Af + eff Ar)
                        forward of the rear AC ([LEN-CAN Eq. 2-1]) - pulled
                        forward by Munk's slender-body fuselage moment
                        dCm/da = 2 (k2-k1) Vol/(S c) (Munk 1924, same term
                        stability_conv.solve_conventional carries)
      CG                x_cg = x_np - k MAC_rear, k = 0.20-0.25
                        ([LEN-CAN Eq. 2-2], row 10)
      cruise trim       Sf CLf + Sr CLr = W/q            (L = W)
                        Sf CLf lf - Sr CLr lr + M0 = 0   (sum M_cg = 0)
                        with M0 = Sf c_f Cm0_f + Sr c_r Cm0_r, solved
                        exactly (2x2 linear)
      incidences        each surface's incidence set so it flies its trim CL
                        with the fuselage datum level - decalage falls out,
                        which is how Lennon rigs it ([LEN-CAN s.4])
      stall-first       dCLf/da = a_f, dCLr/da = eff a_r; the surface with
                        the SMALLER (CLmax - CL)/slope stalls first
    """
    notes: list[str] = []
    s_tot = s_f + s_r
    # span-resolved rear-wing downwash efficiency ([LEN-CAN Eq. 2-1]; only
    # the covered fraction of the rear span pays the 0.8, s.3.2)
    frac_cov = float(np.clip(span_f / max(span_r, 1e-6), 0.0, 1.0))
    eff = 1.0 - frac_cov * (1.0 - REAR_EFF)
    a_r_eff = eff * a_r

    # slope-weighted neutral point of the pair ([LEN-CAN Eq. 2-1] with lift
    # slopes instead of bare areas; reduces to Lennon's form for a_f = a_r)
    w_f = a_f * s_f
    w_r = a_r_eff * s_r
    x_np_surf = (w_f * x_ac_f + w_r * x_ac_r) / max(w_f + w_r, 1e-9)
    a_total = (w_f + w_r) / s_tot
    # Munk slender-body fuselage moment, destabilising: pulls the NP forward
    # by dCm/da_fus * MAC_rear / a_total (Munk 1924, ref S_tot x MAC_rear)
    dcm_da_fus = 2.0 * K_MUNK * max(fus_volume_m3, 0.0) / (s_tot * mac_r)
    shift_fus = dcm_da_fus * mac_r / a_total
    x_np = x_np_surf - shift_fus

    x_cg = x_np - k_margin * mac_r        # k = (x_np - x_cg)/MAC_rear (row 10)
    l_f_arm = x_cg - x_ac_f               # front arm (canard lifts nose-up)
    l_r_arm = x_ac_r - x_cg
    d_ac = x_ac_r - x_ac_f
    if l_f_arm <= 1e-6:
        notes.append("CG fell behind the front surface's AC - the layout "
                     "cannot trim as a two-surface aircraft.")
        l_f_arm = 1e-6

    # ---- exact cruise trim (2x2 linear, sum L = W and sum M_cg = 0) --------
    # M0: both sections' Cm_ac terms, dimensional over q
    m0 = s_f * mac_f * cm0_f + s_r * mac_r * cm0_r
    sr_clr = (w_over_q * l_f_arm + m0) / max(d_ac, 1e-9)
    cl_r = sr_clr / max(s_r, 1e-9)
    cl_f = (w_over_q - sr_clr) / max(s_f, 1e-9)
    if cl_r < 0.01:
        notes.append("Rear wing nearly unloaded in cruise - the CG target "
                     "and section camber push all the lift onto the front "
                     "surface.")
        cl_r = max(cl_r, 0.01)
    cl_ratio = cl_f / max(cl_r, 1e-6)
    cl_sys = w_over_q / s_tot

    # ---- rigging: incidences with the fuselage datum level in cruise ------
    # front surface flies freestream: i_f = CLf/a_f + alpha_L0,f
    i_f = math.degrees(cl_f / max(a_f, 1e-6)) + alpha0_f_deg
    # far-field downwash of the front surface at the rear: eps = 2 CLf/(pi
    # AR_f) (elliptic, Nelson eq. 2.23 analogue), on the covered fraction
    eps = frac_cov * 2.0 * cl_f / (math.pi * max(ar_f, 1e-6))
    deps = frac_cov * 2.0 * a_f / (math.pi * max(ar_f, 1e-6))
    # rear surface: i_r = CLr/a_r + alpha_L0,r + eps (downwash eats alpha)
    i_r = math.degrees(cl_r / max(a_r, 1e-6)) + alpha0_r_deg + math.degrees(eps)
    decalage = i_f - i_r

    # ---- stall order (the defining safety property) ------------------------
    # per radian of AIRCRAFT alpha: front gains a_f, rear gains eff a_r
    margin_f = (clmax_f3 - cl_f) / max(a_f, 1e-6)
    margin_r = (clmax_r3 - cl_r) / max(a_r_eff, 1e-6)
    stall_first = math.degrees(margin_r - margin_f)
    # usable system CL_max: at front-surface stall the rear sits at
    # CLr + margin_f * eff a_r - the honest price of stall-first safety: the
    # rear wing NEVER reaches its own CL_max ([LEN-CAN s.1])
    cl_r_at_stall = cl_r + margin_f * a_r_eff
    cl_sys_max = (s_f * clmax_f3 + s_r * min(cl_r_at_stall, clmax_r3)) / s_tot

    # pitch stiffness about the CG (fuselage already inside x_np)
    dcm_da = -a_total * (x_np - x_cg) / mac_r

    return TwoSurfaceSolution(
        x_np=float(x_np), x_cg=float(x_cg), k_margin=float(k_margin),
        l_f_arm=float(l_f_arm), l_r_arm=float(l_r_arm),
        cl_f=float(cl_f), cl_r=float(cl_r), cl_ratio=float(cl_ratio),
        cl_sys=float(cl_sys), a_f=float(a_f), a_r=float(a_r),
        a_r_eff=float(a_r_eff), a_total=float(a_total),
        eff_rear=float(eff), frac_covered=float(frac_cov),
        deps_dalpha=float(deps), eps_rear_deg=float(math.degrees(eps)),
        i_f_deg=float(i_f), i_r_deg=float(i_r), decalage_deg=float(decalage),
        cl_sys_max=float(cl_sys_max),
        margin_f_deg=float(math.degrees(margin_f)),
        margin_r_deg=float(math.degrees(margin_r)),
        stall_first_margin_deg=float(stall_first),
        dcm_dalpha=float(dcm_da), x_np_shift_fus_m=float(-shift_fus),
        notes=notes,
    )


# ---------------------------------------------------------------------------
# Vertical surfaces by the lateral-area moment rule (shared with tandem.py)
# ---------------------------------------------------------------------------

@dataclass
class FinSizing:
    s_total: float           # both fins together [m^2]
    s_each: float
    h_fin: float             # height of one fin
    c_root: float
    c_tip: float
    c_mac: float
    x_le: float
    x_ac: float
    l_arm: float             # x_ac - x_cg
    cla_ratio: float         # aft/forward lateral-area moment ratio achieved
    vv: float                # V_V = S_v l_v/(S b), reported not gated


def size_fins_cla(*, fus_h: float, l_fus: float, x_cg: float,
                  x_le_anchor: float, s_ref: float, span_ref: float,
                  count: int, min_frac: float = FIN_FRAC_BAND[0],
                  ) -> FinSizing:
    """Size the vertical surfaces from the canard/tandem lateral-area rule.

    Aft lateral-area moment about the CG >= 1.25 x forward moment
    (RESEARCH_TYPES_V3.md s.2.2 row 15, [LEN-CAN s.1 item 4, s.5]); sized at
    CLA_RATIO_TARGET = 1.35 for margin. The fuselage side profile is taken as
    a fus_h-deep rectangle over its length (the same slab-sided shell the
    drag build-up prices), so its moments about the CG integrate to
    h x^2/2 on each side. The fins fill whatever the aft fuselage moment
    lacks; a floor of `min_frac` x S_ref keeps a fin on the airframe even
    when the fuselage alone would satisfy the rule (fin band 2-9% S, s.2.2).
    Fin planform follows the stability_conv conventions (AR_V, TAPER_V).
    """
    m_fwd = fus_h * x_cg**2 / 2.0
    m_aft_fus = fus_h * max(l_fus - x_cg, 0.0)**2 / 2.0
    s_total = max(min_frac * s_ref, 1e-4)
    x_ac = x_le_anchor
    for _ in range(3):     # area <-> own-geometry-arm fixed point, fast
        s_each = s_total / max(count, 1)
        h_fin = math.sqrt(AR_V * s_each)             # AR_V = h^2/S
        c_root = 2.0 * s_each / (h_fin * (1.0 + TAPER_V))
        c_tip = TAPER_V * c_root
        c_mac = mac_length(c_root, TAPER_V)
        x_le = x_le_anchor
        # low-AR raked fin: AC ~ 30% root chord + mid-height sweep (same
        # argument as stability_conv.size_tail's fin placement)
        x_ac = x_le + 0.30 * c_root + 0.35 * h_fin * math.tan(math.radians(30.0))
        l_arm = max(x_ac - x_cg, 0.05)
        need = (CLA_RATIO_TARGET * m_fwd - m_aft_fus) / l_arm
        s_total = max(need, min_frac * s_ref)
    l_arm = max(x_ac - x_cg, 0.05)
    cla = (m_aft_fus + s_total * l_arm) / max(m_fwd, 1e-9)
    vv = s_total * l_arm / (s_ref * span_ref)
    return FinSizing(
        s_total=float(s_total), s_each=float(s_total / max(count, 1)),
        h_fin=float(h_fin), c_root=float(c_root), c_tip=float(c_tip),
        c_mac=float(c_mac), x_le=float(x_le), x_ac=float(x_ac),
        l_arm=float(l_arm), cla_ratio=float(cla), vv=float(vv),
    )


# ---------------------------------------------------------------------------
# Canard styles (the five characters' band sets)
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class CanardStyleDef:
    """One canard character. Every band cites RESEARCH_TYPES_V3.md s.2.2."""
    name: str
    label: str
    description: str
    ar_w_band: tuple[float, float]    # wing AR 4.5-6 (conventional bands)
    ar_c: float                       # canard AR 5-8, >= wing AR (row 13)
    sc_s_band: tuple[float, float]    # inside CANARD_SC_S_BAND (row 8)
    d_mac_band: tuple[float, float]   # inside CANARD_D_MAC_BAND
    k_target: float                   # inside CANARD_K_BAND (row 10)
    taper_w: float
    taper_c: float
    dihedral_deg: float               # rear-wing dihedral (roll stability)
    airfoil_w: str                    # non-reflexed (the canard trims)
    airfoil_c: str                    # turbulent-friendly camber (s.2.2 note)
    wl_band: tuple[float, float]      # on TOTAL lifting area (derived band)
    blurb: str


CANARD_STYLES: dict[str, CanardStyleDef] = {
    # Long-EZ proportions scaled to the hobby box: modest foreplane, high-AR
    # canard, efficiency mission (s.2.1 reference table).
    "canard_cruiser": CanardStyleDef(
        name="canard_cruiser", label="EZ cruiser canard",
        description=(
            "Long-EZ proportions at park size: a modest high-aspect "
            "foreplane, the main wing well aft, pusher prop behind it all - "
            "the layout that cannot deep-stall because the nose drops first."),
        ar_w_band=(5.2, 6.0), ar_c=7.0, sc_s_band=(0.28, 0.35),
        d_mac_band=(3.2, 4.0), k_target=0.21, taper_w=0.65, taper_c=0.70,
        dihedral_deg=2.0, airfoil_w="NACA 0010", airfoil_c="NACA 0009",
        wl_band=(2.8, 4.6),
        blurb="The Rutan recipe: it runs out of nose before it runs out of "
              "wing.",
    ),
    "canard_sport": CanardStyleDef(
        name="canard_sport", label="Sport canard",
        description=(
            "Lower aspect ratio, a bigger foreplane and the CG at the brisk "
            "end of the band - quick and direct, still stall-proof by "
            "construction."),
        ar_w_band=(5.0, 6.0), ar_c=6.5, sc_s_band=(0.28, 0.35),
        d_mac_band=(3.2, 4.0), k_target=0.20, taper_w=0.60, taper_c=0.70,
        dihedral_deg=1.5, airfoil_w="NACA 0010", airfoil_c="NACA 0009",
        wl_band=(3.0, 5.2),
        blurb="Brisk and honest - the canard bobs, the wing never lets go.",
    ),
    "canard_floater": CanardStyleDef(
        name="canard_floater", label="Thermal canard",
        description=(
            "Slow and light: the biggest foreplane of the set, a gentle "
            "cruise and a deep static margin - it mushes where other "
            "floaters drop a tip."),
        ar_w_band=(5.2, 6.0), ar_c=8.0, sc_s_band=(0.28, 0.35),
        d_mac_band=(3.2, 4.0), k_target=0.21, taper_w=0.70, taper_c=0.75,
        dihedral_deg=2.5, airfoil_w="NACA 0010", airfoil_c="NACA 0009",
        wl_band=(2.6, 4.0),
        blurb="A stall-proof floater - the foreplane nods, nothing else "
              "happens.",
    ),
    # MyTwinDream-style mission on a canard airframe: the nose is CLEAN (no
    # motor, no prop) - the canard's gift to camera platforms.
    "canard_fpv": CanardStyleDef(
        name="canard_fpv", label="FPV canard",
        description=(
            "A camera hauler: pusher power, a completely clean nose for the "
            "lens, the bay between the surfaces swallowing the pack - and "
            "stall-proof, which is what you want a long way out."),
        ar_w_band=(5.2, 6.0), ar_c=7.0, sc_s_band=(0.28, 0.35),
        d_mac_band=(3.4, 4.2), k_target=0.22, taper_w=0.68, taper_c=0.72,
        dihedral_deg=2.0, airfoil_w="NACA 0010", airfoil_c="NACA 0009",
        wl_band=(3.0, 4.8),
        blurb="Clean nose for the camera, pusher power, no stall to bite.",
    ),
    "canard_speed": CanardStyleDef(
        name="canard_speed", label="Canard dart",
        description=(
            "Thin sections, a short span and the tight end of the stagger "
            "band: it carries speed like a pattern ship and still refuses "
            "to spin."),
        ar_w_band=(4.8, 5.8), ar_c=6.0, sc_s_band=(0.26, 0.34),
        d_mac_band=(3.0, 3.8), k_target=0.20, taper_w=0.55, taper_c=0.65,
        dihedral_deg=1.0, airfoil_w="NACA 0010", airfoil_c="NACA 0009",
        wl_band=(3.4, 5.2),
        blurb="Fast, flat and unspinnable - the dart that lands itself.",
    ),
}

CANARD_STYLE_FOR_MISSION: dict[str, str] = {
    "sport": "canard_sport", "fpv_cruiser": "canard_fpv",
    "thermal_floater": "canard_floater", "park_flyer": "canard_cruiser",
}


def _style_for(inp: dict) -> CanardStyleDef:
    key = inp.get("canard_style")
    if key in CANARD_STYLES:
        return CANARD_STYLES[key]
    mission = inp.get("mission") or "sport"
    return CANARD_STYLES[CANARD_STYLE_FOR_MISSION.get(mission,
                                                      "canard_cruiser")]


# ---------------------------------------------------------------------------
# Mass model (module-owned: weights.py is fenced). Reuses the calibrated
# conventional areal densities - a canard's wing and foreplane are plain
# thin-section panels exactly like a conventional wing, and the fuselage is
# the same slab-sided printed shell (RESEARCH_CONVENTIONAL.md s.7 anchors).
# ---------------------------------------------------------------------------

def estimate_mass_canard(
    *, material, wing_area: float, canard_area: float, fin_area: float,
    fus_wetted_area: float, n_motors: int, n_servos: int, payload_kg: float,
) -> MassBreakdown:
    """Mass of a canard airframe. The foreplane is wing-grade structure (it
    carries flight loads at 1.4-1.6x the wing's CL); fins are tail-grade."""
    m_wing = CONV_WING_AREAL[material.name] * max(wing_area, 0.0)
    m_canard = CONV_WING_AREAL[material.name] * max(canard_area, 0.0)
    m_fins = CONV_TAIL_AREAL[material.name] * max(fin_area, 0.0)
    m_fus = (CONV_FUS_AREAL[material.name] * max(fus_wetted_area, 0.0)
             + CONV_FITTINGS_KG)
    # one elevator pushrod run inside the nose (same allowance class as the
    # conventional tail pushrods)
    m_push = CONV_PUSHRODS_KG
    m_structure = m_wing + m_canard + m_fins + m_fus + m_push
    m_pack = power_system_allowance(n_motors, m_structure, payload_kg)
    m_servo = n_servos * SERVO_MASS
    m_rx = RECEIVER_MASS + WIRING_BASE
    total = m_structure + m_pack + m_servo + m_rx + payload_kg
    return MassBreakdown(
        structure_wing=m_wing + m_canard, structure_fins=m_fins,
        structure_body=m_fus + m_push, power_system=m_pack, servos=m_servo,
        receiver_wiring=m_rx, payload=payload_kg, total=total,
        components={
            "main wing": m_wing, "canard": m_canard,
            "tip fins": m_fins, "fuselage": m_fus + m_push,
            "power system (motor+ESC+pack)": m_pack,
            "servos": m_servo, "rx+wiring": m_rx, "payload": payload_kg,
        },
    )


# ---------------------------------------------------------------------------
# Single full-physics evaluation of one candidate
# ---------------------------------------------------------------------------

def evaluate_canard(x: dict, inp: dict) -> dict:
    """Evaluate one candidate {span, ar, sc_s, d_mac[, k]} through the whole
    canard pipeline. Returns the full V3_PLAN.md design dict with 'cost',
    'constraints' and 'feasible' filled in (same contract as
    conventional.evaluate_conventional)."""
    style = _style_for(inp)
    mission: MissionDef = MISSIONS[inp.get("mission") or "sport"]
    material = MATERIALS[inp["material"]]
    atm = isa(0.0)
    rho, mu = atm.density_kgm3, atm.viscosity_Pas
    wall_m = material.wall_mm / 1000.0
    v = float(inp["v_cruise"])
    q = 0.5 * rho * v**2                       # q = 0.5 rho V^2
    payload = float(inp.get("payload_kg") or 0.0)

    # ---- search variables, clipped to the researched bands -----------------
    span = float(x["span"])                                   # main wing span
    ar_w = float(np.clip(x["ar"], *style.ar_w_band))
    sc_s = float(np.clip(x.get("sc_s", 0.5 * sum(style.sc_s_band)),
                         *style.sc_s_band))
    d_mac = float(np.clip(x.get("d_mac", 0.5 * sum(style.d_mac_band)),
                          *style.d_mac_band))
    k = float(np.clip(x.get("k", inp.get("sm_override") or style.k_target),
                      *CANARD_K_BAND))

    # ---- fixed geometry of the candidate -----------------------------------
    area = span**2 / ar_w                                # S = b^2/AR
    c_root, c_tip = chords_from_area(span, area, style.taper_w)
    mac = mac_length(c_root, style.taper_w)
    y_bar = y_mac(span, style.taper_w)
    s_c = sc_s * area                                    # Sc = (Sc/S) S
    ar_c = max(style.ar_c, ar_w)                         # canard AR >= wing AR
    span_c = math.sqrt(ar_c * s_c)                       # b = sqrt(AR S)
    c_root_c, c_tip_c = chords_from_area(span_c, s_c, style.taper_c)
    mac_c = mac_length(c_root_c, style.taper_c)
    s_tot = area + s_c

    # stations: canard right on the nose (Rutan practice), wing AC one
    # stagger D aft of the canard AC, fuselage ending just past the wing TE
    # where the pusher motor face sits
    x_le_c = max(0.025, 0.45 * mac_c)      # nose bulkhead + elevator servo
    x_ac_c = x_le_c + 0.25 * mac_c         # thin-airfoil AC at c/4
    x_ac_w = x_ac_c + d_mac * mac
    x_le_w = x_ac_w - 0.25 * mac           # unswept wing LE
    l_f = x_le_w + c_root + 0.040          # aft fairing carries the mount
    fus_h = max(0.10 * l_f, FUS_HEIGHT_MIN_M)   # slender pusher pod
    fus_w = max(FUS_WIDTH_MIN_M, FUS_WIDTH_HEIGHT_FRAC * fus_h)
    s_wet_fus = FUS_WET_PERIM * 2.0 * (fus_w + fus_h) * l_f * FUS_WET_TAPER
    vol_fus = FUS_VOLUME_FILL * l_f * fus_w * fus_h

    airfoil_w = Airfoil(style.airfoil_w)
    airfoil_c = Airfoil(style.airfoil_c)
    fin_af = fin_airfoil()
    e_w = A.oswald_efficiency(ar_w, 1.0)    # plain planar wing, full Raymer
    e_c = A.oswald_efficiency(ar_c, 1.0)
    washout = 0.5                           # near-none: stall order is set by
    #                                         the CL ratio, not tip washout

    wl_band = style.wl_band
    sfac = inp.get("wl_band_scale") or 1.0
    s_lo, s_hi = sfac if isinstance(sfac, (tuple, list)) else (sfac, sfac)
    wl_band = (wl_band[0] * float(s_lo), wl_band[1] * float(s_hi))

    # ---- coupled mass / trim / fins / drag solve ---------------------------
    def run_pipeline(k_v: float, mass0: float, ballast: float = 0.0):
        mass = mass0
        two = fins = drag = mass_bd = None
        re_w = re_c = v_stall = 0.0
        for _ in range(30):
            W = mass * G
            re_w = reynolds(rho, v, mac, mu)          # Re = rho V c / mu
            re_c = reynolds(rho, v, mac_c, mu)
            clmax_w3 = A.cl_max_3d(airfoil_w.cl_max(re_w), 0.0, washout)
            clmax_c3 = A.cl_max_3d(airfoil_c.cl_max(re_c), 0.0, 0.0)
            a_w = lift_slope_3d(airfoil_w.lift_slope_2d(re_w), ar_w, e_w)
            a_c = lift_slope_3d(airfoil_c.lift_slope_2d(re_c), ar_c, e_c)
            two = solve_two_surface(
                s_f=s_c, s_r=area, span_f=span_c, span_r=span,
                x_ac_f=x_ac_c, x_ac_r=x_ac_w, a_f=a_c, a_r=a_w, ar_f=ar_c,
                mac_f=mac_c, mac_r=mac, cm0_f=airfoil_c.cm0(re_c),
                cm0_r=airfoil_w.cm0(re_w), clmax_f3=clmax_c3,
                clmax_r3=clmax_w3, alpha0_f_deg=airfoil_c.alpha_l0_deg(re_c),
                alpha0_r_deg=airfoil_w.alpha_l0_deg(re_w),
                k_margin=k_v, w_over_q=W / q, fus_volume_m3=vol_fus)
            # twin tip fins, extended rearward off the wingtips clear of the
            # prop circle ([LEN-CAN s.5]); sized by the lateral-area rule
            x_fin_anchor = x_le_w + c_tip - 0.45 * (0.35 * c_root)
            fins = size_fins_cla(
                fus_h=fus_h, l_fus=l_f, x_cg=two.x_cg,
                x_le_anchor=x_fin_anchor, s_ref=s_tot, span_ref=span,
                count=2)
            v_stall = A.stall_speed(W, rho, s_tot, two.cl_sys_max)
            drag = _build_drag_canard(
                two, airfoil_w, airfoil_c, re_w, re_c, area, s_c, s_tot,
                ar_w, ar_c, e_w, e_c, fins, l_f, fus_w, fus_h, s_wet_fus,
                rho, v, mu)
            mass_bd = estimate_mass_canard(
                material=material, wing_area=area, canard_area=s_c,
                fin_area=fins.s_total, fus_wetted_area=s_wet_fus,
                n_motors=N_MOTORS_CANARD, n_servos=N_SERVOS_CANARD,
                payload_kg=payload)
            new_mass = mass_bd.total + ballast
            if abs(new_mass - mass) < 1e-4:
                mass = new_mass
                break
            mass = 0.5 * mass + 0.5 * new_mass
        # final consistency pass: L = W exactly at the converged mass
        W = mass * G
        two = solve_two_surface(
            s_f=s_c, s_r=area, span_f=span_c, span_r=span,
            x_ac_f=x_ac_c, x_ac_r=x_ac_w,
            a_f=lift_slope_3d(airfoil_c.lift_slope_2d(re_c), ar_c, e_c),
            a_r=lift_slope_3d(airfoil_w.lift_slope_2d(re_w), ar_w, e_w),
            ar_f=ar_c, mac_f=mac_c, mac_r=mac, cm0_f=airfoil_c.cm0(re_c),
            cm0_r=airfoil_w.cm0(re_w),
            clmax_f3=A.cl_max_3d(airfoil_c.cl_max(re_c), 0.0, 0.0),
            clmax_r3=A.cl_max_3d(airfoil_w.cl_max(re_w), 0.0, washout),
            alpha0_f_deg=airfoil_c.alpha_l0_deg(re_c),
            alpha0_r_deg=airfoil_w.alpha_l0_deg(re_w),
            k_margin=k_v, w_over_q=W / q, fus_volume_m3=vol_fus)
        v_stall = A.stall_speed(W, rho, s_tot, two.cl_sys_max)
        return mass, two, fins, drag, mass_bd, re_w, re_c, v_stall

    # ---- battery placement (the builder's trim tool) -----------------------
    # the bay is the fuselage between the canard root TE and the wing LE -
    # the canard layout's natural equipment volume ([LEN-CAN s.5]: the CG
    # sits near the rear wing's LE, so the pack slides in this gap)
    bay_lo = x_le_c + c_root_c + 0.015
    bay_hi = x_le_w - 0.010
    bay_len = max(bay_hi - bay_lo, 0.01)
    x_ballast = 0.02                        # lead against the nose bulkhead

    def pack_station(mass, two, fins, mass_bd, ballast=0.0):
        """Moment balance sum(m_i x_i) + m_batt x_batt = m x_cg, solved for
        the battery station - identical doctrine to the conventional solver."""
        m = mass_bd
        c = m.components
        m_at_motors = min(PACK_PER_MOTOR * N_MOTORS_CANARD,
                          0.8 * m.power_system)
        m_batt = max(m.power_system - m_at_motors, 1e-6)
        moment = (
            c["main wing"] * (x_le_w + STRUCT_CHORD_CENTROID * mac)
            + c["canard"] * (x_le_c + STRUCT_CHORD_CENTROID * mac_c)
            + c["tip fins"] * fins.x_ac
            + c["fuselage"] * 0.45 * l_f
            + m_at_motors * (l_f - 0.02)               # PUSHER: motor aft
            + 2 * SERVO_MASS * (x_le_w + 0.75 * mac)   # aileron servos
            + 1 * SERVO_MASS * (x_le_c + c_root_c)     # canard elevator servo
            + m.receiver_wiring * (bay_lo + 0.35 * bay_len)
            + m.payload * (bay_lo + 0.10 * bay_len)
            + ballast * x_ballast
        )
        x_batt = (mass * two.x_cg - moment) / m_batt
        return x_batt, m_batt, moment

    def ballast_needed(mass, two, m_batt, moment, ballast):
        """Nose lead when the pack against the forward bay wall still leaves
        the CG too far aft (closed form, same as the conventional solver)."""
        moment_fixed = moment - ballast * x_ballast
        num = moment_fixed + m_batt * bay_lo - mass * two.x_cg
        den = max(two.x_cg - x_ballast, 1e-4)
        return max(num / den, 0.0)

    ballast = 0.0
    mass_guess = 0.30 + 2.6 * s_tot

    def solve_at(k_v, mass0, ballast_v):
        r = run_pipeline(k_v, mass0, ballast_v)
        xb, mb, mom = pack_station(r[0], r[1], r[2], r[4], ballast_v)
        return r, xb, mb, mom

    res, x_batt, m_batt, moment = solve_at(k, mass_guess, 0.0)
    if inp.get("sm_override") is None:
        # in-band lever: slide the margin (bounded) to put the pack inside
        # the bay, then make up the remainder with nose lead - the same
        # doctrine as the flying-wing and conventional solvers, with the
        # slide bounded to the canard k band (row 10)
        target = 0.5 * (bay_lo + bay_hi)
        for _ in range(8):
            if bay_lo <= x_batt <= bay_hi:
                break
            k_new = float(np.clip(
                k + (x_batt - target) * m_batt / max(res[0] * mac, 1e-9),
                *CANARD_K_BAND))
            if abs(k_new - k) < 1e-4:
                break
            k = k_new
            res, x_batt, m_batt, moment = solve_at(k, res[0], ballast)
    for _ in range(5):
        if x_batt >= bay_lo - 1e-9:
            break
        add = ballast_needed(res[0], res[1], m_batt, moment, ballast)
        if add <= 1e-5:
            break
        ballast = min(ballast + add, 0.35 * res[0])
        res, x_batt, m_batt, moment = solve_at(k, res[0], ballast)
        x_batt = max(x_batt, bay_lo)

    mass, two, fins, drag, mass_bd, re_w, re_c, v_stall = res
    if ballast > 1e-5:
        mass_bd.components = dict(mass_bd.components)
        mass_bd.components["nose ballast"] = ballast
    W = mass * G
    cl_sys = W / (q * s_tot)                 # L = W: CL = W/(q S_tot)

    # ---- lateral handling (canard calibration) ----------------------------
    # weathercock: fins at their volume arm vs the destabilising fuselage
    # Cn_beta_fin = eta_v V_V a_v (Etkin & Reid form, stability_conv doctrine)
    a_v = lift_slope_3d(2.0 * math.pi, AR_V, 0.85)
    cn_fin = ETA_V * fins.vv * a_v
    # Munk slender-body yaw, destabilising: Cn_beta_fus = -2 (k2-k1) V/(S b)
    cn_fus = -2.0 * K_MUNK * vol_fus / (s_tot * span)
    cn_beta = cn_fin + cn_fus
    # dihedral effect: strip theory Cl_beta = -a Gamma (y_bar/b) (same
    # integral the conventional handling carries), plus the tip fins riding
    # above the wing plane: Cl_beta_v = -eta (S_v z_v)/(S b) a_v
    gam = math.radians(style.dihedral_deg)
    cl_dih = -two.a_r * gam * (y_bar / span)
    z_fin = 0.40 * fins.h_fin
    cl_fin = -ETA_V * (fins.s_total * z_fin) / (s_tot * span) * a_v
    cl_beta = cl_dih + cl_fin

    # ---- elevator-on-canard authority --------------------------------------
    # Cm_de = eta_c (Sc lc)/(S_tot MAC_r) a_c tau (Nelson eq. 2.47 with the
    # FRONT arm; tau = plain-flap effectiveness of the 0.2c elevator, row 14)
    from .handling import flap_effectiveness
    tau = flap_effectiveness(CANARD_ELEVATOR_FRAC)
    cm_de = ETA_CANARD * (s_c * two.l_f_arm / (s_tot * mac)) * two.a_f * tau
    authority = cm_de * math.radians(CANARD_ELEVATOR_MAX_DEG)
    d_alpha = max(two.cl_sys_max - cl_sys, 0.0) / max(two.a_total, 1e-6)
    trim_used = (abs(two.dcm_dalpha) * d_alpha / authority
                 if authority > 1e-9 else float("inf"))

    # ---- envelope ----------------------------------------------------------
    tip_rise = 0.5 * span * math.tan(gam)
    height_total = fus_h + tip_rise + fins.h_fin + 0.005
    length_total = l_f + 0.005
    v_c = s_c * two.l_f_arm / (area * mac)   # V_C = Sc l_c/(S MAC) (row 12)
    wl = mass / s_tot
    sf = mission.stall_factor
    fin_frac = fins.s_total / s_tot
    ld_cruise = A.lift_to_drag(cl_sys, drag.cd)

    # ---- constraints -------------------------------------------------------
    cons: list[dict] = []

    def con(name, ok, value, limit, msg):
        cons.append({"name": name, "ok": bool(ok), "value": float(value),
                     "limit": float(limit), "message": msg})

    con("box_span", span <= inp["box_w"] + 1e-9, span, inp["box_w"],
        "Wingspan exceeds box width - reduce span or enlarge the box.")
    con("box_length", length_total <= inp["box_l"] + 1e-9, length_total,
        inp["box_l"],
        "Nose-to-motor-face length exceeds the box - a canard needs its "
        "stagger; give it more length or less span.")
    con("box_height", height_total <= inp["box_h"] + 1e-9, height_total,
        inp["box_h"], "Aircraft height (fins + dihedral) exceeds box height.")
    # ---- the aileron servo must fit (user decision 2026-08-26: "I need
    # servo's on all planes and designs") - ailerons ride the REAR/main
    # wing at inner_frac 0.55; same shared floor as the twin-boom.
    from .twinboom import SERVO_HALF_SPAN_MM, servo_chord_floor_m
    f_arm_srv = min(0.55 + 0.10
                    + (SERVO_HALF_SPAN_MM / 1000.0)
                    / max(0.5 * span, 1e-6), 0.95)
    c_arm_srv = float(c_root + (c_tip - c_root) * f_arm_srv)
    c_floor_srv = float(servo_chord_floor_m(
        airfoil_w, material.wall_mm,
        dihedral_deg=style.dihedral_deg, twist_deg=0.0))
    con("servo_fit", c_arm_srv >= c_floor_srv, c_arm_srv, c_floor_srv,
        "The main-wing section at the aileron-servo station cannot bury "
        "the measured SG90 pocket. More chord or less taper is required - "
        "every design carries its servos (user decision).")
    if inp.get("v_stall_target"):
        con("stall_speed", v_stall <= inp["v_stall_target"] * 1.001, v_stall,
            inp["v_stall_target"],
            "Stall speed above your target - more lifting area needed.")
    else:
        con("stall_margin", v >= sf * v_stall, v / max(v_stall, 0.1), sf,
            f"Cruise speed must be >= {sf:.2f} x stall speed.")
    con("wing_loading", wl_band[0] * 0.8 <= wl <= wl_band[1] * 1.2, wl,
        wl_band[1],
        f"Loading {wl:.1f} kg/m^2 (total lifting area) outside the "
        f"{style.label} band ({wl_band[0]:.1f}-{wl_band[1]:.1f}).")
    con("static_margin",
        CANARD_K_BAND[0] - 1e-6 <= two.k_margin <= CANARD_K_BAND[1] + 1e-6,
        two.k_margin, style.k_target,
        f"Static margin outside the canard band k = "
        f"{CANARD_K_BAND[0]:.2f}-{CANARD_K_BAND[1]:.2f} x MAC_rear "
        "([LEN-CAN Eq. 2-2], RESEARCH_TYPES_V3.md row 10).")
    con("pitch_stability", two.dcm_dalpha < 0, two.dcm_dalpha, 0.0,
        "dCm/dalpha must be negative (CG ahead of NP).")
    # THE defining safety property, twice: the coefficient floor and the
    # constructed stall order ([LEN-CAN Eq. 2-7], V3_PLAN.md verification bar)
    con("cl_ratio_floor", two.cl_ratio > CANARD_CL_RATIO_FLOOR,
        two.cl_ratio, CANARD_CL_RATIO_FLOOR,
        "CLf/CLr must exceed 1.0 - below it the WING stalls first and the "
        "canard layout's defining safety property is gone.")
    con("cl_ratio_band",
        CANARD_CL_RATIO_BAND[0] - 1e-6 <= two.cl_ratio
        <= CANARD_CL_RATIO_BAND[1] + 1e-6,
        two.cl_ratio, CANARD_CL_RATIO_BAND[1],
        f"CLf/CLr {two.cl_ratio:.2f} outside the 1.4-1.6 loading-margin "
        "band ([LEN-CAN Eq. 2-7], RESEARCH_TYPES_V3.md row 9).")
    con("canard_stalls_first", two.stall_first_margin_deg > 0.0,
        two.stall_first_margin_deg, 0.0,
        "The canard must reach its stall AoA before the wing - the margin "
        "went negative; more canard AR or less canard camber restores it.")
    con("sc_s_band",
        CANARD_SC_S_BAND[0] - 1e-6 <= sc_s <= CANARD_SC_S_BAND[1] + 1e-6,
        sc_s, CANARD_SC_S_BAND[1],
        "Canard/wing area ratio outside the 0.20-0.35 band (row 8).")
    con("vc_band",
        CANARD_VC_BAND[0] - 1e-6 <= v_c <= CANARD_VC_BAND[1] + 1e-6,
        v_c, CANARD_VC_BAND[1],
        f"Canard volume V_C {v_c:.2f} outside the 0.5-0.9 lifting-canard "
        "band (row 12, derived from the Lennon examples).")
    con("cla_ratio", fins.cla_ratio >= CLA_RATIO_MIN - 1e-6,
        fins.cla_ratio, CLA_RATIO_MIN,
        "Aft lateral-area moment must be >= 1.25 x the forward moment "
        "([LEN-CAN s.1 item 4], row 15) - more fin or a longer aft body.")
    con("fin_fraction",
        FIN_FRAC_BAND[0] - 1e-6 <= fin_frac <= FIN_FRAC_BAND[1] + 1e-6,
        fin_frac, FIN_FRAC_BAND[1],
        "Fin area outside the 2-9% of wing-area band (s.2.2 [VT canards] - "
        "canards run the TOP of it, but not past it).")
    con("directional_stability", cn_beta > 0.0, cn_beta, 0.0,
        "Weathercock instability: the fuselage side area overpowers the tip "
        "fins.")
    con("roll_stability", cl_beta < 0.0, cl_beta, 0.0,
        "No dihedral effect: a sideslip rolls it further into the slip.")
    con("elevator_authority", trim_used <= 0.75, trim_used, 0.75,
        "Pulling to the canard's stall eats most of the elevator travel - "
        "a shallower margin or more elevator is needed.")
    con("cambered_section",
        (not airfoil_w.defn.reflexed) and (not airfoil_c.defn.reflexed),
        airfoil_w.cm0_geometric(), 0.0,
        "Both canard surfaces fly plain (non-reflexed) sections - the "
        "foreplane does the trimming, the exact inverse of a flying wing.")
    con("reynolds", min(re_w, re_c) >= RE_MIN_HARD, min(re_w, re_c),
        RE_MIN_HARD,
        "Chord too small (the canard's usually) - Reynolds number too low "
        "for reliable airfoil performance.")
    con("battery_position", bay_lo - 1e-9 <= x_batt <= bay_hi + 1e-9,
        x_batt, bay_hi,
        "The pack cannot sit in the bay between canard and wing and still "
        "balance the aircraft - more stagger or a lighter tail end.")
    con("nose_ballast", ballast <= 0.15 * mass + 1e-9,
        ballast / max(mass, 1e-9), 0.15,
        "This layout needs an unreasonable amount of nose ballast - a "
        "pusher canard wants its pack well forward instead.")
    con("bay_length", bay_len >= 0.10, bay_len, 0.10,
        "The inter-surface bay is too short for a flight pack (>= 100 mm).")

    feasible = all(c["ok"] for c in cons)

    # ---- cost (same specific-drag objective family) ------------------------
    sd_weight = float(inp.get("sd_weight") or 1.0)
    cost = sd_weight * 100.0 / max(ld_cruise, 1e-3)
    wl_pen_w = inp.get("wl_pen_weight")
    wl_pen_w = 1.0 if wl_pen_w is None else float(wl_pen_w)
    cost += 6.0 * wl_pen_w * _band_center_pen(wl, *wl_band)
    cost += 4.0 * _band_center_pen(ar_w, *style.ar_w_band)
    cost += 3.0 * _band_center_pen(two.cl_ratio, *CANARD_CL_RATIO_BAND)
    cost += 2.0 * _band_center_pen(sc_s, *style.sc_s_band)
    cost += 2.0 * _band_center_pen(v_c, *CANARD_VC_BAND)
    if min(re_w, re_c) < RE_MIN_GOOD:
        cost += 300.0 * ((RE_MIN_GOOD - min(re_w, re_c)) / RE_MIN_GOOD) ** 2
    span_frac = float(inp.get("span_pref_frac") or 0.85)
    span_target = span_frac * min(inp["box_w"], inp["box_l"] / 0.8, 2.4)
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

    # ---- assemble the design dict (V3_PLAN.md schema) ----------------------
    motors = [{"x": float(l_f - 0.010), "y": 0.0, "z": 0.0, "type": "pusher"}]
    bay_w = max(fus_w - 2.0 * wall_m, 0.02)
    bay_d = 0.62 * fus_h

    notes = list(two.notes)
    if not (0.0 <= two.decalage_deg <= 3.5):
        notes.append(
            f"Decalage {two.decalage_deg:+.1f} deg is outside the +1..+3 deg "
            "canard rigging practice ([LEN-CAN s.4], RESEARCH_TYPES_V3.md "
            "s.2.2) - check the CG target against the section cambers.")

    design = {
        "id": "",
        "airplane_type": "canard",
        "planform": style.name,
        "planform_label": style.label,
        "mission": mission.name,
        # v3 config block, researched defaults (V3_PLAN.md; s.10.4 matrix)
        "config": {
            "motor_layout": "pusher",     # [LEN-CAN s.5]
            "n_motors": N_MOTORS_CANARD,
            "tail_type": None,            # tailless aft: tip fins instead
            "wing_position": "mid",       # rear wing mid default (s.10.4)
        },
        "geometry": {
            "span_m": float(span),
            "area_m2": float(area),                  # MAIN (rear) wing
            "area_total_m2": float(s_tot),           # wing + canard
            "aspect_ratio": float(ar_w),
            "taper": float(style.taper_w),
            "sweep_le_deg": 0.0,
            "dihedral_deg": float(style.dihedral_deg),
            "washout_deg": float(washout),
            "root_incidence_deg": float(two.i_r_deg),
            "root_chord_m": float(c_root),
            "tip_chord_m": float(c_tip),
            "airfoil": airfoil_w.name,
            "fin_airfoil": fin_af.name,
            "wing_position": "mid",
            "x_le_wing_m": float(x_le_w),
            "wing_z_m": 0.0,
            # V3_PLAN.md: geometry.canard {x_le_m, span_m, c_root_m, c_tip_m,
            # area_m2, V_C, elevator_chord_frac, incidence_deg}
            "canard": {
                "x_le_m": float(x_le_c),
                "span_m": float(span_c),
                "c_root_m": float(c_root_c),
                "c_tip_m": float(c_tip_c),
                "area_m2": float(s_c),
                "mac_m": float(mac_c),
                "aspect_ratio": float(ar_c),
                "taper": float(style.taper_c),
                "V_C": float(v_c),
                "elevator_chord_frac": float(CANARD_ELEVATOR_FRAC),
                "incidence_deg": float(two.i_f_deg),
                "airfoil": airfoil_c.name,
                "z_m": float(0.25 * fus_h),   # shoulder-mounted foreplane
                "sc_s": float(sc_s),
            },
            "fins": {
                "arrangement": "tip_fins",    # pusher canard: twin tip fins
                "count": 2,                   # clear of the prop [LEN-CAN s.5]
                "area_each_m2": float(fins.s_each),
                "area_total_m2": float(fins.s_total),
                "height_m": float(fins.h_fin),
                "c_root_m": float(fins.c_root),
                "c_tip_m": float(fins.c_tip),
                "x_le_m": float(fins.x_le),
                "x_ac_m": float(fins.x_ac),
                "y_m": float(0.5 * span),
                "z_m": float(tip_rise),
                "cla_moment_ratio": float(fins.cla_ratio),
            },
            "fuselage": {
                "length_m": float(l_f),
                "width_m": float(fus_w),
                "height_m": float(fus_h),
                "x_wing_le_m": float(x_le_w),
                "x_canard_le_m": float(x_le_c),
                "wetted_area_m2": float(s_wet_fus),
                "bay": {
                    "bay_start_m": float(bay_lo),
                    "bay_length_m": float(min(bay_len, 0.45 * l_f)),
                    "bay_width_m": float(bay_w),
                    "bay_depth_m": float(bay_d),
                    "bay_wall_m": float(wall_m),
                },
            },
            "ailerons": {          # rear-wing outboard set (conventional
                "inner_frac": 0.55,  # aileron practice; the canard elevator
                "outer_frac": 0.95,  # does pitch)
                "chord_frac": 0.25,
            },
            "motors": motors,
            "motor_mount": _motor_mount_spec(inp, mass, motors,
                                             material.wall_mm),
            "battery_x_m": float(np.clip(x_batt, bay_lo, bay_hi)),
            "battery_x_required_m": float(x_batt),
            "length_total_m": float(length_total),
            "height_total_m": float(height_total),
            "control_surfaces": ["aileron_left", "aileron_right",
                                 "elevator_canard"],
            "wall_mm": float(material.wall_mm),
            "build_method": inp["build_method"],
        },
        "aero": {
            "cl_cruise": float(cl_sys),
            "cd0": float(drag.cd0), "cd_cruise": float(drag.cd),
            "cd0_breakdown": {"wing": drag.cd0_wing,
                              "fuselage": drag.cd0_fuselage,
                              "tail": drag.cd0_tail,
                              "interference": drag.cd0_interference},
            "ld_cruise": float(ld_cruise),
            "specific_drag": float(1.0 / ld_cruise if ld_cruise > 0 else 0.0),
            "oswald_e": float(e_w), "re_mac": float(re_w),
            "re_canard": float(re_c),
            "v_cruise_ms": float(v), "v_stall_ms": float(v_stall),
            "stall_margin": float(v / max(v_stall, 0.1)),
            "wing_loading_kgm2": float(wl),
            "cl_max_2d": float(airfoil_w.cl_max(re_w)),
            "cl_max_3d": float(two.cl_sys_max),
            "alpha_cruise_deg": float(two.i_r_deg),
            "cm0_section": float(airfoil_w.cm0(re_w)),
            "cm0_section_geometric": float(airfoil_w.cm0_geometric()),
        },
        "stability": {
            "mac_m": float(mac), "y_mac_m": float(y_bar),
            "x_le_mac_m": float(x_le_w),
            "x_ac_wing_m": float(x_ac_w),
            "x_ac_canard_m": float(x_ac_c),
            "x_np_m": float(two.x_np), "x_cg_m": float(two.x_cg),
            "static_margin": float(two.k_margin),
            "static_margin_ref": "MAC_rear",   # k band, NOT the tailed band
            "cg_pct_mac": float(100.0 * (two.x_cg - x_le_w) / mac),
            "cg_mm_from_nose": float(two.x_cg * 1000.0),
            "dcm_dalpha": float(two.dcm_dalpha),
            "cm0_wing": float(airfoil_w.cm0(re_w)),
            "cm_trim_residual": 0.0,   # incidences trim it by construction
            # the canard loading margin - THE recorded safety property
            "cl_canard_cruise": float(two.cl_f),
            "cl_wing_cruise": float(two.cl_r),
            "cl_ratio_fr": float(two.cl_ratio),
            "cl_ratio_band": list(CANARD_CL_RATIO_BAND),
            "canard_stalls_first": bool(two.stall_first_margin_deg > 0),
            "stall_first_margin_deg": float(two.stall_first_margin_deg),
            "rear_wing_efficiency": float(two.eff_rear),
            "deps_dalpha": float(two.deps_dalpha),
            "decalage_deg": float(two.decalage_deg),
            "incidence_canard_deg": float(two.i_f_deg),
            "incidence_wing_deg": float(two.i_r_deg),
            "v_c": float(v_c),
            "l_canard_m": float(two.l_f_arm),
            "l_wing_m": float(two.l_r_arm),
            "x_np_shift_fus_m": float(two.x_np_shift_fus_m),
            "cn_beta": float(cn_beta),
            "cn_beta_fin": float(cn_fin),
            "cn_beta_fus": float(cn_fus),
            "cl_beta": float(cl_beta),
            "cl_beta_dihedral": float(cl_dih),
            "cl_beta_fin": float(cl_fin),
            "cla_moment_ratio": float(fins.cla_ratio),
            "elevator_authority_cm": float(authority),
            "elevator_trim_used": float(trim_used),
            "handling_notes": [
                f"Canard loading margin CLf/CLr = {two.cl_ratio:.2f} - the "
                f"foreplane stalls {two.stall_first_margin_deg:.1f} deg of "
                "alpha before the wing can.",
                f"Aft/forward lateral-area moment ratio "
                f"{fins.cla_ratio:.2f} (>= 1.25 required, [LEN-CAN s.1]).",
            ],
            "washout_deg": float(washout),
            "notes": notes,
        },
        "power_system": {
            "n_motors": N_MOTORS_CANARD,
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
        "notes": notes,
        "cost": float(cost),
    }
    return design


def _build_drag_canard(two, airfoil_w, airfoil_c, re_w, re_c, s_w, s_c,
                       s_tot, ar_w, ar_c, e_w, e_c, fins, l_f, fus_w, fus_h,
                       s_wet_fus, rho, v, mu):
    """CD0 build-up for a canard airframe, referenced to S_tot.

    Each lifting surface's profile drag at ITS OWN operating Cl (the canard
    flies 1.4-1.6x harder - the honest price of stall-first safety); fuselage
    by the wetted-area method with Raymer's form factor FF = 1 + 60/f^3 +
    f/400 (eq. 12.31); fins by the thin-symmetric wetted-area method;
    interference at 8% (canard root + wing root + two fin joints). Induced
    drag per surface: CDi = sum (S_i/S_tot) CL_i^2/(pi AR_i e_i) - the mutual
    induced term is already inside the 0.8 rear efficiency on the LIFT side
    (Lennon's simplification, [LEN-CAN Eq. 2-1])."""
    cd0_wing = (airfoil_w.cd_at_cl(two.cl_r, re_w) * (s_w / s_tot)
                + airfoil_c.cd_at_cl(two.cl_f, re_c) * (s_c / s_tot))
    re_f = reynolds(rho, v, max(l_f, 0.05), mu)
    d_eq = math.sqrt(4.0 * fus_w * fus_h / math.pi)
    f = max(l_f / max(d_eq, 1e-3), 2.0)
    ff_fus = 1.0 + 60.0 / f**3 + f / 400.0            # Raymer eq. 12.31
    cd0_fus = A.flat_plate_cf(max(re_f, 3e4)) * ff_fus * (s_wet_fus / s_tot)
    re_v = reynolds(rho, v, max(fins.c_mac, 0.02), mu)
    cd0_fin = (A.flat_plate_cf(max(re_v, 3e4)) * 2.05
               * (fins.s_total / s_tot) * 1.25)
    subtotal = cd0_wing + cd0_fus + cd0_fin
    cd0_int = 0.08 * subtotal
    cd0 = subtotal + cd0_int
    # CDi per surface (each sheds its own vortex system at its own CL)
    cdi = ((s_w / s_tot) * two.cl_r**2 / (math.pi * ar_w * e_w)
           + (s_c / s_tot) * two.cl_f**2 / (math.pi * ar_c * e_c))
    return A.DragBreakdown(cd0_wing, cd0_fus, cd0_fin, cd0_int, cd0, cdi,
                           cd0 + cdi)


# ---------------------------------------------------------------------------
# Search driver
# ---------------------------------------------------------------------------

def optimize_canard(inp: dict) -> dict:
    """Sweep (span, AR, Sc/S, stagger) inside the style bands and the box,
    keep the best design, polish it (Nelder-Mead, with the static margin k as
    a fifth polish variable) and return the full design dict. Mirrors
    optimize_conventional's contract: never an exception for an infeasible
    request - the closest design is returned with its binding constraints."""
    style = _style_for(inp)

    # span bounded by the box width directly and by the length through the
    # stagger: l_f ~ (0.45 + d_mac + 0.30) x MAC and MAC ~ span/AR
    b_hi = min(float(inp["box_w"]),
               float(inp["box_l"]) * 0.5 * sum(style.ar_w_band)
               / (0.5 * sum(style.d_mac_band) + 1.6), 2.4)
    b_lo = min(max(0.45, 0.55 * b_hi), 0.92 * b_hi)

    ar_lo, ar_hi = style.ar_w_band
    if inp.get("ar_target"):
        ar_lo = ar_hi = float(np.clip(inp["ar_target"], *CANARD_AR_W_LIMITS))
    sc_lo, sc_hi = style.sc_s_band
    d_lo, d_hi = style.d_mac_band

    spans = np.linspace(b_lo, b_hi, 4)
    ars = (np.linspace(ar_lo, ar_hi, 2) if ar_hi > ar_lo + 1e-6
           else np.array([ar_lo]))
    scs = np.linspace(sc_lo, sc_hi, 3)
    ds = np.linspace(d_lo, d_hi, 3)

    grid = [{"span": float(b), "ar": float(a), "sc_s": float(s),
             "d_mac": float(d)}
            for b in spans for a in ars for s in scs for d in ds]
    for s in (inp.get("seeds") or []):
        try:
            grid.append({
                "span": float(np.clip(s["span"], 0.3, b_hi)),
                "ar": float(np.clip(s["ar"], ar_lo, ar_hi)),
                "sc_s": float(np.clip(s.get("sc_s", 0.5 * (sc_lo + sc_hi)),
                                      sc_lo, sc_hi)),
                "d_mac": float(np.clip(s.get("d_mac", 0.5 * (d_lo + d_hi)),
                                       d_lo, d_hi)),
            })
        except Exception:
            continue

    best = None
    for xd in grid:
        try:
            d = evaluate_canard(xd, inp)
        except Exception:
            continue
        if best is None or d["cost"] < best["cost"]:
            best = d
    if best is None:
        raise RuntimeError("optimizer could not evaluate any candidate design")

    g = best["geometry"]
    x0 = [g["span_m"], g["aspect_ratio"], g["canard"]["sc_s"],
          (best["stability"]["x_ac_wing_m"]
           - best["stability"]["x_ac_canard_m"]) / best["stability"]["mac_m"],
          best["stability"]["static_margin"]]

    k_pin = inp.get("sm_override")   # a user margin stays pinned in polish

    def unpack(xv):
        return {"span": float(np.clip(xv[0], 0.3, b_hi)),
                "ar": float(np.clip(xv[1], ar_lo, ar_hi)),
                "sc_s": float(np.clip(xv[2], sc_lo, sc_hi)),
                "d_mac": float(np.clip(xv[3], d_lo, d_hi)),
                "k": float(np.clip(k_pin if k_pin is not None else xv[4],
                                   *CANARD_K_BAND))}

    def f(xv):
        try:
            return evaluate_canard(unpack(xv), inp)["cost"]
        except Exception:
            return 1e9

    try:
        r = sciopt.minimize(f, x0, method="Nelder-Mead",
                            options={"maxfev": 70, "xatol": 1e-3,
                                     "fatol": 0.4})
        polished = evaluate_canard(unpack(r.x), inp)
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
# Five canard characters
# ---------------------------------------------------------------------------

CANARD_VARIANTS: list[dict] = [
    {"key": "canard_cruiser", "name": "EZ Cruiser", "style": "canard_cruiser",
     "tagline": "Long-EZ proportions at park size - efficient and unstallable.",
     "knobs": {}},
    {"key": "canard_sport", "name": "Canard Sport", "style": "canard_sport",
     "tagline": "Bigger foreplane, brisker margin - quick but stall-proof.",
     "knobs": {}},
    {"key": "canard_floater", "name": "Thermal Canard",
     "style": "canard_floater",
     "tagline": "Slow and deep-margined - it mushes where others drop a tip.",
     "knobs": {"sd_weight": 1.6, "span_pref_frac": 0.92,
               "v_cruise_mult": 0.85}},
    {"key": "canard_fpv", "name": "FPV Canard", "style": "canard_fpv",
     "tagline": "Clean nose for the camera, pusher power, deep margin.",
     "knobs": {}},
    {"key": "canard_speed", "name": "Canard Dart", "style": "canard_speed",
     "tagline": "Thin, short-coupled and fast - and it still cannot spin.",
     "knobs": {"span_pref_frac": 0.72, "v_cruise_mult": 1.15}},
]


def _canard_guidance(vd: dict, d: dict) -> list[dict]:
    """Builder guidance sections - same rendering contract as guidance.py."""
    g, a, st = d["geometry"], d["aero"], d["stability"]
    cn, fus = g["canard"], g["fuselage"]
    secs: list[dict] = []
    if not d.get("feasible", True):
        bad = [c["message"] for c in d["constraints"] if not c["ok"]]
        secs.append({"title": "Compromises - read this first",
                     "body": "\n".join(f"- {m}" for m in bad)})
    secs.append({"title": "CG, balance and rigging", "body": "\n".join([
        f"- Balance point: {st['cg_mm_from_nose']:.0f} mm from the nose "
        f"({st['static_margin'] * 100:.0f}% of the wing chord AHEAD of the "
        f"neutral point - canards balance on the rear wing's MAC, "
        "[LEN-CAN]).",
        f"- Slide the battery to balance: the bay runs "
        f"{fus['bay']['bay_start_m'] * 1000:.0f}-"
        f"{(fus['bay']['bay_start_m'] + fus['bay']['bay_length_m']) * 1000:.0f}"
        f" mm from the nose; the design places the pack at "
        f"{g['battery_x_m'] * 1000:.0f} mm.",
        f"- Rig the canard at {cn['incidence_deg']:+.1f} deg and the wing "
        f"at {g['root_incidence_deg']:+.1f} deg "
        f"({st['decalage_deg']:+.1f} deg of decalage) so the elevator sits "
        "neutral in cruise ([LEN-CAN s.4]).",
        "- NEVER balance it aft of the marked CG: the stall-first margin is "
        "the whole point of a canard.",
    ])})
    secs.append({"title": "The stall-proof property", "body": "\n".join([
        f"- The foreplane flies at CL {st['cl_canard_cruise']:.2f} vs the "
        f"wing's {st['cl_wing_cruise']:.2f} - a loading margin of "
        f"{st['cl_ratio_fr']:.2f} (band 1.4-1.6, [LEN-CAN Eq. 2-7]).",
        f"- So the canard runs out of lift "
        f"{st['stall_first_margin_deg']:.1f} deg of alpha before the wing "
        "can: the nose bobs down, the wing keeps flying, nothing snaps.",
        "- The price: the wing never reaches its own CL_max, so the "
        "approach is a little faster than the wing area suggests.",
        "- Keep the foreplane CLEAN: a contaminated canard loses lift first "
        "(the Long-EZ rain lesson) - no gap tape ridges, no bugs, and the "
        "thin turbulent-friendly section stays.",
    ])})
    secs.append({"title": "Control surfaces and throws", "body": "\n".join([
        f"- Elevator: {cn['elevator_chord_frac'] * 100:.0f}% of the canard "
        "chord, full span; start at +-15 deg, max +-20 - the authority "
        "knee is ~40 deg and past it the canard just stalls sooner "
        "([LEN-CAN s.3-4]).",
        f"- Ailerons: {g['ailerons']['chord_frac'] * 100:.0f}% chord on the "
        "outer rear-wing panels; +-12 deg low rate.",
        "- No rudder: the twin tip fins are fixed and sized by the "
        "lateral-area moment rule (aft/forward >= 1.25, [LEN-CAN s.1]).",
    ])})
    secs.append({"title": f"Why this variant - {vd['name']}",
                 "body": "\n".join([
        f"- This is the {vd['name']}, a {d['planform_label'].lower()}: "
        f"{g['span_m'] * 1000:.0f} mm wing span with a "
        f"{cn['span_m'] * 1000:.0f} mm foreplane "
        f"(Sc/S {cn['sc_s']:.2f}), {fus['length_m'] * 1000:.0f} mm long, "
        f"pusher motor on the tail face.",
        f"- Canard volume V_C {cn['V_C']:.2f} (lifting-canard band 0.5-0.9) "
        f"on a {st['l_canard_m'] / st['mac_m']:.1f}-chord front arm.",
        f"- Numbers: {a['wing_loading_kgm2']:.1f} kg/m2 on the total "
        f"lifting area, {a['v_stall_ms']:.1f} m/s stall vs "
        f"{a['v_cruise_ms']:.1f} m/s cruise ({a['stall_margin']:.2f}x), "
        f"L/D {a['ld_cruise']:.1f}.",
    ])})
    return secs


def generate_canard_variants(inp: dict) -> list[dict]:
    """Five canard characters for one set of user requirements - the canard
    mirror of generate_conventional_variants: always exactly five, the
    mission's own style leads, infeasible characters returned flagged."""
    primary_key = CANARD_STYLE_FOR_MISSION.get(
        inp.get("mission") or "sport", "canard_cruiser")
    ctx: dict = {"seeds": []}
    designs: dict[str, dict] = {}

    def _run(vd: dict) -> dict:
        knobs = dict(vd["knobs"])
        v_mult = knobs.pop("v_cruise_mult", None)
        merged = {**inp, **knobs, "canard_style": vd["style"],
                  "seeds": ctx.get("seeds", [])}
        if v_mult:
            merged["v_cruise"] = inp["v_cruise"] * float(v_mult)
        ref = ctx.get("v_stall_ref", 0.0)
        if vd["key"] == "canard_floater" and ref > 0:
            merged["v_stall_goal"] = 0.85 * ref
        try:
            return optimize_canard(merged)
        except Exception:
            d = optimize_canard({**inp, "canard_style": vd["style"]})
            d["notes"].append(
                f"The {vd['name']} tuning could not be solved for these "
                "inputs; showing the untuned solution for this character.")
            return d

    def _seed(d: dict) -> dict:
        g = d["geometry"]
        return {"span": g["span_m"], "ar": g["aspect_ratio"],
                "sc_s": g["canard"]["sc_s"],
                "d_mac": ((d["stability"]["x_ac_wing_m"]
                           - d["stability"]["x_ac_canard_m"])
                          / d["stability"]["mac_m"])}

    lead_def = next(v for v in CANARD_VARIANTS if v["key"] == primary_key)
    lead = _run(lead_def)
    designs[primary_key] = lead
    ctx["v_stall_ref"] = lead["aero"]["v_stall_ms"]
    ctx["seeds"].append(_seed(lead))
    for vd in CANARD_VARIANTS:
        if vd["key"] == primary_key:
            continue
        d = _run(vd)
        designs[vd["key"]] = d
        ctx["seeds"].append(_seed(d))

    ordered = [designs[vd["key"]] for vd in CANARD_VARIANTS]
    traits = traits_generic(ordered)
    out: list[dict] = []
    for vd, d, tr in zip(CANARD_VARIANTS, ordered, traits):
        d["character"] = {"key": vd["key"], "name": vd["name"],
                          "tagline": vd["tagline"]}
        d["guidance"] = _canard_guidance(vd, d)
        out.append({
            "id": d["id"],
            "key": vd["key"],
            "name": vd["name"],
            "tagline": vd["tagline"],
            "planform": d["planform"],
            "airplane_type": "canard",
            "primary": vd["key"] == primary_key,
            "traits": tr,
            "design": d,
        })
    return out
