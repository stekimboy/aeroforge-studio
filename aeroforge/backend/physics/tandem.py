"""TANDEM-WING airplane design solve + variant generation (V3_PLAN.md 2a).

NEW in v3, beside the flying-wing / conventional / canard paths; dispatch
registration is the integration wave's job, so `evaluate_tandem`,
`optimize_tandem` and `generate_tandem_variants` are callable standalone
(same entry-point pattern as physics/conventional.py).

The airframe: two COMPARABLE wings - front wing LOW on the fuselage, rear
wing HIGH and 2.5-3.5 rear MACs behind it (the Quickie arrangement:
"the Quickie mounts the canard low and the main wing high",
RESEARCH_TYPES_V3.md s.3.2) - a single TRACTOR motor on the nose ("Tractor
designs ... [the] result is what is usually called a tandem-wing design",
[LEN-CAN s.6 item 7]; s.10.4 matrix), the equipment bay in the fuselage
between the wings, and one aft centre fin sized by the lateral-area moment
rule. Control allocation is the Quickie Q2 split: FULL-SPAN ELEVATOR on the
front wing, AILERONS on the rear wing (s.3.2) - pitch and roll servos in
different wings, which the wiring doctrine likes.

Tandem IS the canard math with the area ratio walked up: "The design
principles in this document apply to any two-surface design ... increase
[canard] area and maybe decrease the rear wing area" ([LEN-CAN s.6 item 7],
RESEARCH_TYPES_V3.md s.3) - so the longitudinal core (`solve_two_surface`)
and the lateral-area fin rule (`size_fins_cla`) are imported from
physics/canard.py rather than duplicated.

Every band cites RESEARCH_TYPES_V3.md s.3.2 (quick-reference rows 16-21);
objective is the same specific-drag 100/(L/D) family as every other type.
"""
from __future__ import annotations

import math
import uuid

import numpy as np
from scipy import optimize as sciopt

from . import aero as A
from .airfoils import Airfoil, fin_airfoil
from .atmosphere import isa, reynolds
from .canard import (
    CLA_RATIO_MIN, FIN_FRAC_BAND, REAR_EFF, estimate_mass_canard,
    size_fins_cla, solve_two_surface,
)
from .config_defs import MISSIONS, MissionDef
from .conventional import (
    FUS_HEIGHT_MIN_M, FUS_VOLUME_FILL, FUS_WET_PERIM, FUS_WET_TAPER,
    FUS_WIDTH_HEIGHT_FRAC, FUS_WIDTH_MIN_M, STRUCT_CHORD_CENTROID,
    traits_generic,
)
from .handling import flap_effectiveness
from .optimizer import (
    G, RE_MIN_GOOD, RE_MIN_HARD, _band_center_pen, _motor_mount_spec,
)
from .stability import chords_from_area, lift_slope_3d, mac_length, y_mac
from .stability_conv import AR_V, ETA_V, K_MUNK
from .weights import MATERIALS, PACK_PER_MOTOR, SERVO_MASS

# ---------------------------------------------------------------------------
# Tandem bands - RESEARCH_TYPES_V3.md s.3.2, quick-reference rows 16-21.
# ---------------------------------------------------------------------------
# Front-wing lift share 0.45-0.60 of total, default 0.50-0.55 (row 16;
# "each wing carries 40-60% of total lift/area", Cheng et al. 2018).
TANDEM_SHARE_BAND: tuple[float, float] = (0.45, 0.60)
# CLf/CLr >= 1.2, target 1.3 (row 17: Lennon's 1.4-1.6 applies at
# canard-like area ratios; as Sf/Sr -> 1 the achievable ratio shrinks,
# derived from [LEN-CAN Eq. 2-7] with D/L at the recorded CG).
TANDEM_RATIO_FLOOR: float = 1.2
TANDEM_RATIO_TARGET_BAND: tuple[float, float] = (1.2, 1.7)
# Stagger (AC-to-AC) 2.5-3.5 x rear MAC, default 3.0 (row 18; derived from
# Lennon ex.1 L/MAC_r = 3.1 and ex.2 = 3.0).
TANDEM_STAGGER_BAND: tuple[float, float] = (2.5, 3.5)
# Vertical gap 0.3-1.0 x rear MAC, REAR WING HIGH, default 0.5 (row 19;
# non-zero gap lifts the rear wing out of the front wake - "lift performance
# improves and stall is delayed significantly", Bath experiments).
TANDEM_GAP_BAND: tuple[float, float] = (0.3, 1.0)
TANDEM_GAP_DEFAULT: float = 0.5
# Decalage front-positive +0.5..+2 deg, default +1 (row 20). The app trims
# analytically - the equivalent statement "set incidences so CLf/CLr lands
# in band at cruise" (s.3.2) is the mechanism here, and the geometric
# decalage that falls out is REPORTED against the band, not forced: the
# rear wing's incidence carries the front wing's downwash angle on top of
# its own lift demand, which legitimately shrinks the front-positive rigging
# at model CLs.
TANDEM_DECALAGE_BAND: tuple[float, float] = (0.5, 2.0)
# Per-wing AR 5-7, Lennon's 6 front / 5 rear (row 21).
TANDEM_AR_BAND: tuple[float, float] = (5.0, 7.0)
# Static margin k of REAR MAC: the tandem inherits the canard band
# k = 0.20-0.25 ([LEN-CAN s.6 item 7]: same framework, RESEARCH row 10).
TANDEM_K_BAND: tuple[float, float] = (0.20, 0.25)
# Wing loading on TOTAL lifting area: derived from the conventional band
# (RESEARCH_CONVENTIONAL.md s.7) exactly as the canard module does - the
# dossier bands no tandem-specific loading.
TANDEM_WL_LIMITS: tuple[float, float] = (2.6, 5.2)

N_MOTORS_TANDEM = 1     # tractor single (s.10.4: no twin practice here, s.8)
N_SERVOS_TANDEM = 4     # 2 front elevator halves + 2 rear ailerons (s.3.2)
ELEVATOR_FRAC_TANDEM = 0.20     # plain-flap optimum 0.2c ([LEN-CAN s.3])
ELEVATOR_MAX_DEG_TANDEM = 20.0  # inside the 40-deg authority knee (s.2.2)


# ---------------------------------------------------------------------------
# Tandem styles. Section note: both wings fly NEAR-SYMMETRIC sections
# (front NACA 0009 / rear NACA 0010) for the same measured reason as the
# canard module - at model cruise CLs a cambered pair's Cm_ac terms push the
# front lift share past the 0.60 band edge (share shifts by
# -M0/((W/q) D) with M0 < 0 for camber) and erode the front-stalls-first
# CL_max parity. See canard.py's section-registration comment.
# ---------------------------------------------------------------------------

from dataclasses import dataclass


@dataclass(frozen=True)
class TandemStyleDef:
    """One tandem character. Bands cite RESEARCH_TYPES_V3.md s.3.2."""
    name: str
    label: str
    description: str
    ar_band: tuple[float, float]      # rear-wing AR, inside TANDEM_AR_BAND
    sf_sr_band: tuple[float, float]   # front/rear area ratio (tandem regime)
    d_mac_band: tuple[float, float]   # stagger, inside TANDEM_STAGGER_BAND
    gap_mac: float                    # vertical gap, inside TANDEM_GAP_BAND
    bf_frac: float                    # front span / rear span (Quickie ~0.9)
    k_target: float                   # inside TANDEM_K_BAND
    taper_f: float
    taper_r: float
    dihedral_f_deg: float             # front LOW wing: RC ladder ~3 (s.10.1)
    dihedral_r_deg: float             # rear HIGH wing: ladder ~1-2 (s.10.1)
    airfoil_f: str
    airfoil_r: str
    wl_band: tuple[float, float]      # on total lifting area (derived)
    blurb: str


TANDEM_STYLES: dict[str, TandemStyleDef] = {
    # The Rutan Quickie / QAC Q2 proportions at hobby scale (s.3.1 table).
    "tandem_quickie": TandemStyleDef(
        name="tandem_quickie", label="Quickie-style tandem",
        description=(
            "The Rutan Quickie recipe: front wing low, rear wing high, "
            "nearly equal spans, elevator up front and ailerons in back - "
            "compact, quick and stall-resistant."),
        ar_band=(5.2, 6.0), sf_sr_band=(0.75, 0.92), d_mac_band=(2.8, 3.5),
        gap_mac=0.5, bf_frac=0.93, k_target=0.20, taper_f=0.70, taper_r=0.70,
        dihedral_f_deg=3.0, dihedral_r_deg=1.5,
        airfoil_f="NACA 0009", airfoil_r="NACA 0010",
        wl_band=(3.0, 5.0),
        blurb="Two wings, no tail, all business - the Quickie look.",
    ),
    "tandem_cruiser": TandemStyleDef(
        name="tandem_cruiser", label="Tandem tourer",
        description=(
            "A steady cruiser: longer stagger for pitch damping, a deeper "
            "margin and the bay between the wings swallowing a long pack."),
        ar_band=(5.4, 6.2), sf_sr_band=(0.75, 0.90), d_mac_band=(3.0, 3.5),
        gap_mac=0.6, bf_frac=0.93, k_target=0.22, taper_f=0.72, taper_r=0.72,
        dihedral_f_deg=3.0, dihedral_r_deg=1.5,
        airfoil_f="NACA 0009", airfoil_r="NACA 0010",
        wl_band=(2.8, 4.6),
        blurb="Long-legged and level - the touring tandem.",
    ),
    "tandem_floater": TandemStyleDef(
        name="tandem_floater", label="Meadow tandem",
        description=(
            "Light loading, gentle cruise, the biggest gap of the set to "
            "keep the rear wing flying clean and slow."),
        ar_band=(5.6, 6.4), sf_sr_band=(0.78, 0.92), d_mac_band=(3.0, 3.5),
        gap_mac=0.8, bf_frac=0.94, k_target=0.21, taper_f=0.75, taper_r=0.75,
        dihedral_f_deg=3.0, dihedral_r_deg=2.0,
        airfoil_f="NACA 0009", airfoil_r="NACA 0010",
        wl_band=(2.6, 3.8),
        blurb="Slow, soft and settled - two wings sharing a breeze.",
    ),
    # The MyTwinDream mission on a tandem airframe: lots of lifting area in
    # a short box, the pack and payload low between the wings.
    "tandem_hauler": TandemStyleDef(
        name="tandem_hauler", label="Payload tandem",
        description=(
            "A load hauler: maximum lifting area inside the box (that is "
            "the tandem's whole trick), the heaviest loading of the set "
            "and the bay sized for a fat pack plus payload."),
        ar_band=(5.0, 5.8), sf_sr_band=(0.75, 0.90), d_mac_band=(2.8, 3.4),
        gap_mac=0.6, bf_frac=0.94, k_target=0.22, taper_f=0.75, taper_r=0.75,
        dihedral_f_deg=3.0, dihedral_r_deg=1.5,
        airfoil_f="NACA 0009", airfoil_r="NACA 0010",
        wl_band=(3.0, 5.0),
        blurb="Twice the wing in half the box - built to carry.",
    ),
    "tandem_speed": TandemStyleDef(
        name="tandem_speed", label="Twin dart",
        description=(
            "Short-coupled and thin: the tight end of the stagger band, "
            "less span, more loading - it goes, and both wings keep "
            "flying all the way down the speed range."),
        ar_band=(5.0, 5.8), sf_sr_band=(0.72, 0.88), d_mac_band=(2.5, 3.2),
        gap_mac=0.4, bf_frac=0.92, k_target=0.20, taper_f=0.65, taper_r=0.65,
        dihedral_f_deg=2.5, dihedral_r_deg=1.0,
        airfoil_f="NACA 0009", airfoil_r="NACA 0010",
        wl_band=(3.4, 5.2),
        blurb="Two short wings, one straight line - the fast tandem.",
    ),
}

TANDEM_STYLE_FOR_MISSION: dict[str, str] = {
    "sport": "tandem_quickie", "fpv_cruiser": "tandem_hauler",
    "thermal_floater": "tandem_floater", "park_flyer": "tandem_cruiser",
}


def _style_for(inp: dict) -> TandemStyleDef:
    key = inp.get("tandem_style")
    if key in TANDEM_STYLES:
        return TANDEM_STYLES[key]
    mission = inp.get("mission") or "sport"
    return TANDEM_STYLES[TANDEM_STYLE_FOR_MISSION.get(mission,
                                                      "tandem_quickie")]


# ---------------------------------------------------------------------------
# Single full-physics evaluation of one candidate
# ---------------------------------------------------------------------------

def evaluate_tandem(x: dict, inp: dict) -> dict:
    """Evaluate one candidate {span, ar, sf_sr, d_mac[, k]} through the whole
    tandem pipeline. Returns the full V3_PLAN.md design dict with 'cost',
    'constraints' and 'feasible' filled in."""
    style = _style_for(inp)
    mission: MissionDef = MISSIONS[inp.get("mission") or "sport"]
    material = MATERIALS[inp["material"]]
    atm = isa(0.0)
    rho, mu = atm.density_kgm3, atm.viscosity_Pas
    wall_m = material.wall_mm / 1000.0
    v = float(inp["v_cruise"])
    q = 0.5 * rho * v**2
    payload = float(inp.get("payload_kg") or 0.0)

    # ---- search variables, clipped to the researched bands -----------------
    span = float(x["span"])                              # REAR wing span
    ar_r = float(np.clip(x["ar"], *style.ar_band))
    sf_sr = float(np.clip(x.get("sf_sr", 0.5 * sum(style.sf_sr_band)),
                          *style.sf_sr_band))
    d_mac = float(np.clip(x.get("d_mac", 0.5 * sum(style.d_mac_band)),
                          *style.d_mac_band))
    k = float(np.clip(x.get("k", inp.get("sm_override") or style.k_target),
                      *TANDEM_K_BAND))

    # ---- fixed geometry of the candidate -----------------------------------
    s_r = span**2 / ar_r                                 # S = b^2/AR
    c_root_r, c_tip_r = chords_from_area(span, s_r, style.taper_r)
    mac_r = mac_length(c_root_r, style.taper_r)
    y_bar_r = y_mac(span, style.taper_r)
    span_f = style.bf_frac * span                        # Quickie: fronts run
    s_f = sf_sr * s_r                                    # slightly shorter
    ar_f = span_f**2 / s_f                               # AR = b^2/S
    c_root_f, c_tip_f = chords_from_area(span_f, s_f, style.taper_f)
    mac_f = mac_length(c_root_f, style.taper_f)
    y_bar_f = y_mac(span_f, style.taper_f)
    s_tot = s_f + s_r
    gap_mac = float(np.clip(style.gap_mac, *TANDEM_GAP_BAND))
    gap = gap_mac * mac_r                                # rear wing HIGH

    # stations: tractor motor face at x = 0, front wing just behind the
    # firewall, rear wing one stagger aft, fin on the tail cone
    x_le_f = 0.040 + 0.35 * mac_f          # motor + firewall + nose former
    x_ac_f = x_le_f + 0.25 * mac_f         # thin-airfoil AC
    x_ac_r = x_ac_f + d_mac * mac_r        # stagger, AC to AC (row 18)
    x_le_r = x_ac_r - 0.25 * mac_r
    l_f = x_le_r + c_root_r + 0.050        # tail cone carries the fin
    fus_h = max(0.10 * l_f, FUS_HEIGHT_MIN_M)
    fus_w = max(FUS_WIDTH_MIN_M, FUS_WIDTH_HEIGHT_FRAC * fus_h)
    s_wet_fus = FUS_WET_PERIM * 2.0 * (fus_w + fus_h) * l_f * FUS_WET_TAPER
    vol_fus = FUS_VOLUME_FILL * l_f * fus_w * fus_h
    z_f = -0.35 * fus_h                    # front wing LOW (Quickie, s.3.2)
    z_r = z_f + gap                        # rear wing HIGH by the gap

    airfoil_f = Airfoil(style.airfoil_f)
    airfoil_r = Airfoil(style.airfoil_r)
    fin_af = fin_airfoil()
    e_f = A.oswald_efficiency(ar_f, 1.0)
    e_r = A.oswald_efficiency(ar_r, 1.0)
    washout = 0.5

    wl_band = style.wl_band
    sfac = inp.get("wl_band_scale") or 1.0
    s_lo, s_hi = sfac if isinstance(sfac, (tuple, list)) else (sfac, sfac)
    wl_band = (wl_band[0] * float(s_lo), wl_band[1] * float(s_hi))

    # ---- coupled mass / trim / fin / drag solve ----------------------------
    def run_pipeline(k_v: float, mass0: float, ballast: float = 0.0):
        mass = mass0
        two = fins = drag = mass_bd = None
        re_f = re_r = v_stall = 0.0
        for _ in range(30):
            W = mass * G
            re_f = reynolds(rho, v, mac_f, mu)        # Re = rho V c / mu
            re_r = reynolds(rho, v, mac_r, mu)
            two = solve_two_surface(
                s_f=s_f, s_r=s_r, span_f=span_f, span_r=span,
                x_ac_f=x_ac_f, x_ac_r=x_ac_r,
                a_f=lift_slope_3d(airfoil_f.lift_slope_2d(re_f), ar_f, e_f),
                a_r=lift_slope_3d(airfoil_r.lift_slope_2d(re_r), ar_r, e_r),
                ar_f=ar_f, mac_f=mac_f, mac_r=mac_r,
                cm0_f=airfoil_f.cm0(re_f), cm0_r=airfoil_r.cm0(re_r),
                clmax_f3=A.cl_max_3d(airfoil_f.cl_max(re_f), 0.0, 0.0),
                clmax_r3=A.cl_max_3d(airfoil_r.cl_max(re_r), 0.0, washout),
                alpha0_f_deg=airfoil_f.alpha_l0_deg(re_f),
                alpha0_r_deg=airfoil_r.alpha_l0_deg(re_r),
                k_margin=k_v, w_over_q=W / q, fus_volume_m3=vol_fus)
            # single aft centre fin on the tail cone (tractor tandem takes
            # one aft fin, s.3.2), lateral-area moment rule (row 15)
            fins = size_fins_cla(
                fus_h=fus_h, l_fus=l_f, x_cg=two.x_cg,
                x_le_anchor=l_f - 0.105, s_ref=s_tot, span_ref=span,
                count=1)
            v_stall = A.stall_speed(W, rho, s_tot, two.cl_sys_max)
            drag = _build_drag_tandem(
                two, airfoil_f, airfoil_r, re_f, re_r, s_f, s_r, s_tot,
                ar_f, ar_r, e_f, e_r, fins, l_f, fus_w, fus_h, s_wet_fus,
                rho, v, mu)
            # mass: the canard estimator IS the tandem estimator - two
            # wing-grade lifting panels + fins + slab fuselage (canard.py's
            # module-owned model, same calibrated areal densities)
            mass_bd = estimate_mass_canard(
                material=material, wing_area=s_r, canard_area=s_f,
                fin_area=fins.s_total, fus_wetted_area=s_wet_fus,
                n_motors=N_MOTORS_TANDEM, n_servos=N_SERVOS_TANDEM,
                payload_kg=payload)
            new_mass = mass_bd.total + ballast
            if abs(new_mass - mass) < 1e-4:
                mass = new_mass
                break
            mass = 0.5 * mass + 0.5 * new_mass
        # final consistency pass: L = W exactly at the converged mass
        W = mass * G
        two = solve_two_surface(
            s_f=s_f, s_r=s_r, span_f=span_f, span_r=span,
            x_ac_f=x_ac_f, x_ac_r=x_ac_r,
            a_f=lift_slope_3d(airfoil_f.lift_slope_2d(re_f), ar_f, e_f),
            a_r=lift_slope_3d(airfoil_r.lift_slope_2d(re_r), ar_r, e_r),
            ar_f=ar_f, mac_f=mac_f, mac_r=mac_r,
            cm0_f=airfoil_f.cm0(re_f), cm0_r=airfoil_r.cm0(re_r),
            clmax_f3=A.cl_max_3d(airfoil_f.cl_max(re_f), 0.0, 0.0),
            clmax_r3=A.cl_max_3d(airfoil_r.cl_max(re_r), 0.0, washout),
            alpha0_f_deg=airfoil_f.alpha_l0_deg(re_f),
            alpha0_r_deg=airfoil_r.alpha_l0_deg(re_r),
            k_margin=k_v, w_over_q=W / q, fus_volume_m3=vol_fus)
        v_stall = A.stall_speed(W, rho, s_tot, two.cl_sys_max)
        return mass, two, fins, drag, mass_bd, re_f, re_r, v_stall

    # ---- battery placement: the bay between the wings ----------------------
    bay_lo = x_le_f + c_root_f + 0.015
    bay_hi = x_le_r - 0.010
    bay_len = max(bay_hi - bay_lo, 0.01)
    x_ballast = 0.02

    def pack_station(mass, two, fins, mass_bd, ballast=0.0):
        """Moment balance sum(m_i x_i) + m_batt x_batt = m x_cg (same
        doctrine as every other type's solver)."""
        m = mass_bd
        c = m.components
        m_at_motors = min(PACK_PER_MOTOR * N_MOTORS_TANDEM,
                          0.8 * m.power_system)
        m_batt = max(m.power_system - m_at_motors, 1e-6)
        moment = (
            c["main wing"] * (x_le_r + STRUCT_CHORD_CENTROID * mac_r)
            + c["canard"] * (x_le_f + STRUCT_CHORD_CENTROID * mac_f)
            + c["tip fins"] * fins.x_ac
            + c["fuselage"] * 0.45 * l_f
            + m_at_motors * 0.02                        # TRACTOR: nose
            + 2 * SERVO_MASS * (x_le_f + 0.75 * mac_f)  # front elev servos
            + 2 * SERVO_MASS * (x_le_r + 0.75 * mac_r)  # rear aileron servos
            + m.receiver_wiring * (bay_lo + 0.35 * bay_len)
            + m.payload * (bay_lo + 0.10 * bay_len)
            + ballast * x_ballast
        )
        x_batt = (mass * two.x_cg - moment) / m_batt
        return x_batt, m_batt, moment

    def ballast_needed(mass, two, m_batt, moment, ballast):
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
        target = 0.5 * (bay_lo + bay_hi)
        for _ in range(8):
            if bay_lo <= x_batt <= bay_hi:
                break
            k_new = float(np.clip(
                k + (x_batt - target) * m_batt / max(res[0] * mac_r, 1e-9),
                *TANDEM_K_BAND))
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

    mass, two, fins, drag, mass_bd, re_f, re_r, v_stall = res
    if ballast > 1e-5:
        mass_bd.components = dict(mass_bd.components)
        mass_bd.components["nose ballast"] = ballast
    W = mass * G
    cl_sys = W / (q * s_tot)
    # front lift share = Sf CLf / (W/q) - THE tandem band, row 16
    share_f = s_f * two.cl_f / max(W / q, 1e-9)

    # ---- lateral handling --------------------------------------------------
    a_v = lift_slope_3d(2.0 * math.pi, AR_V, 0.85)
    cn_fin = ETA_V * fins.vv * a_v
    cn_fus = -2.0 * K_MUNK * vol_fus / (s_tot * span)
    cn_beta = cn_fin + cn_fus
    # dihedral effect per wing (strip term, converted to the common
    # S_tot x b_rear reference) + wing-position equivalent dihedral: high
    # +1.5 deg / low -1.5 deg (RESEARCH_TYPES_V3.md s.10.1, row 52)
    gam_f = math.radians(style.dihedral_f_deg - 1.5)   # front LOW
    gam_r = math.radians(style.dihedral_r_deg + 1.5)   # rear HIGH
    cl_dih_f = (-two.a_f * gam_f * (y_bar_f / span_f)
                * (s_f * span_f) / (s_tot * span))
    cl_dih_r = (-two.a_r * gam_r * (y_bar_r / span)
                * (s_r * span) / (s_tot * span))
    z_v = 0.5 * fus_h + 0.45 * fins.h_fin
    cl_fin = -ETA_V * (fins.s_total * z_v) / (s_tot * span) * a_v
    cl_beta = cl_dih_f + cl_dih_r + cl_fin

    # ---- front-elevator authority ------------------------------------------
    # Cm_de = (Sf lf)/(S_tot MAC_r) a_f tau, full-span 0.2c plain flap on the
    # front wing (Quickie allocation, s.3.2; tau per handling.py)
    tau = flap_effectiveness(ELEVATOR_FRAC_TANDEM)
    cm_de = (s_f * two.l_f_arm / (s_tot * mac_r)) * two.a_f * tau
    authority = cm_de * math.radians(ELEVATOR_MAX_DEG_TANDEM)
    d_alpha = max(two.cl_sys_max - cl_sys, 0.0) / max(two.a_total, 1e-6)
    trim_used = (abs(two.dcm_dalpha) * d_alpha / authority
                 if authority > 1e-9 else float("inf"))

    # ---- envelope ----------------------------------------------------------
    tip_rise_r = 0.5 * span * math.tan(math.radians(style.dihedral_r_deg))
    z_top = max(0.5 * fus_h + fins.h_fin, z_r + tip_rise_r + 0.06 * c_root_r)
    height_total = z_top + 0.5 * fus_h + 0.005
    length_total = max(l_f, fins.x_le + fins.c_root) + 0.005
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
        "Rear wingspan exceeds box width - reduce span or enlarge the box.")
    con("box_length", length_total <= inp["box_l"] + 1e-9, length_total,
        inp["box_l"],
        "Nose-to-fin length exceeds the box - a tandem needs its stagger.")
    con("box_height", height_total <= inp["box_h"] + 1e-9, height_total,
        inp["box_h"],
        "Aircraft height (gap + fin) exceeds box height - the rear wing "
        "rides high by 0.3-1.0 MAC and that costs vertical room.")
    # ---- the aileron servo must fit (user decision 2026-08-26: "I need
    # servo's on all planes and designs") - the ailerons ride the REAR
    # wing at inner_frac 0.55; same shared floor as the twin-boom.
    from .twinboom import SERVO_HALF_SPAN_MM, servo_chord_floor_m
    f_arm_srv = min(0.55 + 0.10
                    + (SERVO_HALF_SPAN_MM / 1000.0)
                    / max(0.5 * span, 1e-6), 0.95)
    c_arm_srv = float(c_root_r + (c_tip_r - c_root_r) * f_arm_srv)
    c_floor_srv = float(servo_chord_floor_m(
        airfoil_r, material.wall_mm,
        dihedral_deg=style.dihedral_r_deg, twist_deg=0.0))
    con("servo_fit", c_arm_srv >= c_floor_srv, c_arm_srv, c_floor_srv,
        "The rear-wing section at the aileron-servo station cannot bury "
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
        TANDEM_K_BAND[0] - 1e-6 <= two.k_margin <= TANDEM_K_BAND[1] + 1e-6,
        two.k_margin, style.k_target,
        f"Static margin outside the two-surface band k = "
        f"{TANDEM_K_BAND[0]:.2f}-{TANDEM_K_BAND[1]:.2f} x MAC_rear "
        "([LEN-CAN Eq. 2-2] via s.3's framework inheritance).")
    con("pitch_stability", two.dcm_dalpha < 0, two.dcm_dalpha, 0.0,
        "dCm/dalpha must be negative (CG ahead of NP).")
    con("lift_share_band",
        TANDEM_SHARE_BAND[0] - 1e-6 <= share_f
        <= TANDEM_SHARE_BAND[1] + 1e-6,
        share_f, TANDEM_SHARE_BAND[1],
        f"Front-wing lift share {share_f:.2f} outside the 0.45-0.60 band "
        "(RESEARCH_TYPES_V3.md row 16, Cheng et al. 2018).")
    con("cl_ratio_floor", two.cl_ratio >= TANDEM_RATIO_FLOOR - 1e-6,
        two.cl_ratio, TANDEM_RATIO_FLOOR,
        "CLf/CLr must stay >= 1.2 (row 17) - the front wing loads first at "
        "every trimmed AoA or the tandem loses its stall manners.")
    con("front_stalls_first", two.stall_first_margin_deg > 0.0,
        two.stall_first_margin_deg, 0.0,
        "The front wing must reach its stall AoA before the rear wing.")
    con("stagger_band",
        TANDEM_STAGGER_BAND[0] - 1e-6 <= d_mac
        <= TANDEM_STAGGER_BAND[1] + 1e-6,
        d_mac, TANDEM_STAGGER_BAND[1],
        "Stagger outside the 2.5-3.5 rear-MAC band (row 18): shorter "
        "couples the wings too tightly, longer wastes fuselage.")
    con("gap_band",
        TANDEM_GAP_BAND[0] - 1e-6 <= gap_mac <= TANDEM_GAP_BAND[1] + 1e-6,
        gap_mac, TANDEM_GAP_BAND[1],
        "Vertical gap outside the 0.3-1.0 rear-MAC band (row 19) - the "
        "rear wing must ride clear of the front wake, rear HIGH.")
    con("per_wing_ar",
        TANDEM_AR_BAND[0] - 1e-6 <= min(ar_f, ar_r)
        and max(ar_f, ar_r) <= TANDEM_AR_BAND[1] + 1e-6,
        ar_f, TANDEM_AR_BAND[1],
        "Per-wing aspect ratio outside the 5-7 band (row 21).")
    con("cla_ratio", fins.cla_ratio >= CLA_RATIO_MIN - 1e-6,
        fins.cla_ratio, CLA_RATIO_MIN,
        "Aft lateral-area moment must be >= 1.25 x the forward moment "
        "([LEN-CAN s.1 item 4] via s.3.2).")
    con("fin_fraction",
        FIN_FRAC_BAND[0] - 1e-6 <= fin_frac <= FIN_FRAC_BAND[1] + 1e-6,
        fin_frac, FIN_FRAC_BAND[1],
        "Fin area outside the 2-9% of lifting-area band.")
    con("directional_stability", cn_beta > 0.0, cn_beta, 0.0,
        "Weathercock instability: fuselage side area overpowers the fin.")
    con("roll_stability", cl_beta < 0.0, cl_beta, 0.0,
        "No dihedral effect: a sideslip rolls it further into the slip.")
    con("elevator_authority", trim_used <= 0.75, trim_used, 0.75,
        "Pulling to the front wing's stall eats most of the elevator "
        "travel.")
    con("cambered_section",
        (not airfoil_f.defn.reflexed) and (not airfoil_r.defn.reflexed),
        airfoil_r.cm0_geometric(), 0.0,
        "Both tandem wings fly plain (non-reflexed) sections - the front "
        "wing does the trimming.")
    con("reynolds", min(re_f, re_r) >= RE_MIN_HARD, min(re_f, re_r),
        RE_MIN_HARD,
        "Chord too small - Reynolds number too low for reliable airfoil "
        "performance.")
    con("battery_position", bay_lo - 1e-9 <= x_batt <= bay_hi + 1e-9,
        x_batt, bay_hi,
        "The pack cannot sit in the inter-wing bay and still balance the "
        "aircraft.")
    con("nose_ballast", ballast <= 0.15 * mass + 1e-9,
        ballast / max(mass, 1e-9), 0.15,
        "This layout needs an unreasonable amount of nose ballast.")
    con("bay_length", bay_len >= 0.10, bay_len, 0.10,
        "The inter-wing bay is too short for a flight pack (>= 100 mm).")

    feasible = all(c["ok"] for c in cons)

    # ---- cost (same specific-drag objective family) ------------------------
    sd_weight = float(inp.get("sd_weight") or 1.0)
    cost = sd_weight * 100.0 / max(ld_cruise, 1e-3)
    wl_pen_w = inp.get("wl_pen_weight")
    wl_pen_w = 1.0 if wl_pen_w is None else float(wl_pen_w)
    cost += 6.0 * wl_pen_w * _band_center_pen(wl, *wl_band)
    cost += 4.0 * _band_center_pen(ar_r, *style.ar_band)
    cost += 5.0 * _band_center_pen(share_f, *TANDEM_SHARE_BAND)
    cost += 2.0 * _band_center_pen(two.cl_ratio, *TANDEM_RATIO_TARGET_BAND)
    cost += 2.0 * _band_center_pen(d_mac, *style.d_mac_band)
    if min(re_f, re_r) < RE_MIN_GOOD:
        cost += 300.0 * ((RE_MIN_GOOD - min(re_f, re_r)) / RE_MIN_GOOD) ** 2
    span_frac = float(inp.get("span_pref_frac") or 0.85)
    span_target = span_frac * min(inp["box_w"], inp["box_l"] / 0.75, 2.4)
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
    motors = [{"x": 0.005, "y": 0.0, "z": 0.0, "type": "tractor"}]
    bay_w = max(fus_w - 2.0 * wall_m, 0.02)
    bay_d = 0.62 * fus_h

    notes = list(two.notes)
    if not (TANDEM_DECALAGE_BAND[0] <= two.decalage_deg
            <= TANDEM_DECALAGE_BAND[1]):
        notes.append(
            f"Geometric decalage {two.decalage_deg:+.1f} deg sits outside "
            "the +0.5..+2 deg rigging band (row 20) - legitimate here: the "
            "rear wing's incidence carries the front wing's downwash angle, "
            "and the CL ratio (the band's real content, s.3.2) is "
            f"{two.cl_ratio:.2f}.")

    design = {
        "id": "",
        "airplane_type": "tandem",
        "planform": style.name,
        "planform_label": style.label,
        "mission": mission.name,
        # v3 config defaults (V3_PLAN.md; s.10.4 matrix)
        "config": {
            "motor_layout": "tractor",    # [LEN-CAN s.6 item 7]
            "n_motors": N_MOTORS_TANDEM,
            "tail_type": None,            # aft fin only, no stab
            "wing_position": "high",      # rear high / front low (s.3.2)
        },
        "geometry": {
            # main block = REAR wing (the stability reference surface)
            "span_m": float(span),
            "area_m2": float(s_r),
            "area_total_m2": float(s_tot),
            "aspect_ratio": float(ar_r),
            "taper": float(style.taper_r),
            "sweep_le_deg": 0.0,
            "dihedral_deg": float(style.dihedral_r_deg),
            "washout_deg": float(washout),
            "root_incidence_deg": float(two.i_r_deg),
            "root_chord_m": float(c_root_r),
            "tip_chord_m": float(c_tip_r),
            "airfoil": airfoil_r.name,
            "fin_airfoil": fin_af.name,
            "wing_position": "high",
            "x_le_wing_m": float(x_le_r),
            "wing_z_m": float(z_r),
            # V3_PLAN.md: geometry.wing2 {x_le_m, span_m, chords, area_m2,
            # lift_share, decalage_deg} - the FRONT wing
            "wing2": {
                "role": "front",
                "x_le_m": float(x_le_f),
                "span_m": float(span_f),
                "c_root_m": float(c_root_f),
                "c_tip_m": float(c_tip_f),
                "area_m2": float(s_f),
                "mac_m": float(mac_f),
                "aspect_ratio": float(ar_f),
                "taper": float(style.taper_f),
                "lift_share": float(share_f),
                "decalage_deg": float(two.decalage_deg),
                "incidence_deg": float(two.i_f_deg),
                "z_m": float(z_f),
                "gap_m": float(gap),
                "gap_mac": float(gap_mac),
                "stagger_mac": float(d_mac),
                "dihedral_deg": float(style.dihedral_f_deg),
                "airfoil": airfoil_f.name,
                "elevator_chord_frac": float(ELEVATOR_FRAC_TANDEM),
                "sf_sr": float(sf_sr),
            },
            "fins": {
                "arrangement": "center_fin",   # tractor tandem (s.3.2)
                "count": 1,
                "area_each_m2": float(fins.s_each),
                "area_total_m2": float(fins.s_total),
                "height_m": float(fins.h_fin),
                "c_root_m": float(fins.c_root),
                "c_tip_m": float(fins.c_tip),
                "x_le_m": float(fins.x_le),
                "x_ac_m": float(fins.x_ac),
                "y_m": 0.0,
                "z_m": float(0.5 * fus_h),
                "cla_moment_ratio": float(fins.cla_ratio),
            },
            "fuselage": {
                "length_m": float(l_f),
                "width_m": float(fus_w),
                "height_m": float(fus_h),
                "x_wing_le_m": float(x_le_r),
                "x_wing2_le_m": float(x_le_f),
                "wetted_area_m2": float(s_wet_fus),
                "bay": {
                    "bay_start_m": float(bay_lo),
                    "bay_length_m": float(min(bay_len, 0.45 * l_f)),
                    "bay_width_m": float(bay_w),
                    "bay_depth_m": float(bay_d),
                    "bay_wall_m": float(wall_m),
                },
            },
            "ailerons": {          # REAR wing outboard (Quickie split)
                "inner_frac": 0.55,
                "outer_frac": 0.95,
                "chord_frac": 0.25,
            },
            "motors": motors,
            "motor_mount": _motor_mount_spec(inp, mass, motors,
                                             material.wall_mm),
            "battery_x_m": float(np.clip(x_batt, bay_lo, bay_hi)),
            "battery_x_required_m": float(x_batt),
            "length_total_m": float(length_total),
            "height_total_m": float(height_total),
            "control_surfaces": ["elevator_left", "elevator_right",
                                 "aileron_left", "aileron_right"],
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
            "oswald_e": float(e_r), "re_mac": float(re_r),
            "re_front": float(re_f),
            "v_cruise_ms": float(v), "v_stall_ms": float(v_stall),
            "stall_margin": float(v / max(v_stall, 0.1)),
            "wing_loading_kgm2": float(wl),
            "cl_max_2d": float(airfoil_r.cl_max(re_r)),
            "cl_max_3d": float(two.cl_sys_max),
            "alpha_cruise_deg": float(two.i_r_deg),
            "cm0_section": float(airfoil_r.cm0(re_r)),
            "cm0_section_geometric": float(airfoil_r.cm0_geometric()),
        },
        "stability": {
            "mac_m": float(mac_r), "y_mac_m": float(y_bar_r),
            "x_le_mac_m": float(x_le_r),
            "x_ac_wing_m": float(x_ac_r),
            "x_ac_front_m": float(x_ac_f),
            "x_np_m": float(two.x_np), "x_cg_m": float(two.x_cg),
            "static_margin": float(two.k_margin),
            "static_margin_ref": "MAC_rear",
            "cg_pct_mac": float(100.0 * (two.x_cg - x_le_r) / mac_r),
            "cg_mm_from_nose": float(two.x_cg * 1000.0),
            "dcm_dalpha": float(two.dcm_dalpha),
            "cm0_wing": float(airfoil_r.cm0(re_r)),
            "cm_trim_residual": 0.0,
            "front_lift_share": float(share_f),
            "cl_front_cruise": float(two.cl_f),
            "cl_rear_cruise": float(two.cl_r),
            "cl_ratio_fr": float(two.cl_ratio),
            "front_stalls_first": bool(two.stall_first_margin_deg > 0),
            "stall_first_margin_deg": float(two.stall_first_margin_deg),
            "rear_wing_efficiency": float(two.eff_rear),
            "deps_dalpha": float(two.deps_dalpha),
            "decalage_deg": float(two.decalage_deg),
            "incidence_front_deg": float(two.i_f_deg),
            "incidence_rear_deg": float(two.i_r_deg),
            "stagger_mac": float(d_mac),
            "gap_mac": float(gap_mac),
            "l_front_m": float(two.l_f_arm),
            "l_rear_m": float(two.l_r_arm),
            "x_np_shift_fus_m": float(two.x_np_shift_fus_m),
            "cn_beta": float(cn_beta),
            "cn_beta_fin": float(cn_fin),
            "cn_beta_fus": float(cn_fus),
            "cl_beta": float(cl_beta),
            "cl_beta_dihedral": float(cl_dih_f + cl_dih_r),
            "cl_beta_fin": float(cl_fin),
            "cla_moment_ratio": float(fins.cla_ratio),
            "elevator_authority_cm": float(authority),
            "elevator_trim_used": float(trim_used),
            "handling_notes": [
                f"Front wing carries {share_f * 100:.0f}% of the lift at "
                f"CLf/CLr = {two.cl_ratio:.2f} - it stalls "
                f"{two.stall_first_margin_deg:.1f} deg of alpha before the "
                "rear wing can.",
                f"Rear wing rides {gap_mac:.1f} MAC above the front wake "
                "(Bath experiments: non-zero gap delays rear-wing stall).",
            ],
            "washout_deg": float(washout),
            "notes": notes,
        },
        "power_system": {
            "n_motors": N_MOTORS_TANDEM,
            "pack_mass_kg": float(mass_bd.power_system),
            "motor_positions": [dict(mo) for mo in motors],
        },
        "mass": {
            "total_kg": float(mass),
            "weight_n": float(W),
            "nose_ballast_kg": float(ballast),
            "breakdown_kg": {
                "front wing": mass_bd.components["canard"],
                "rear wing": mass_bd.components["main wing"],
                "fin": mass_bd.components["tip fins"],
                "fuselage": mass_bd.components["fuselage"],
                "power system (motor+ESC+pack)":
                    mass_bd.components["power system (motor+ESC+pack)"],
                "servos": mass_bd.components["servos"],
                "rx+wiring": mass_bd.components["rx+wiring"],
                "payload": mass_bd.components["payload"],
                **({"nose ballast": ballast} if ballast > 1e-5 else {}),
            },
        },
        "constraints": cons,
        "feasible": bool(feasible),
        "binding": [c["name"] for c in cons if not c["ok"]],
        "notes": notes,
        "cost": float(cost),
    }
    return design


def _build_drag_tandem(two, airfoil_f, airfoil_r, re_f, re_r, s_f, s_r,
                       s_tot, ar_f, ar_r, e_f, e_r, fins, l_f, fus_w, fus_h,
                       s_wet_fus, rho, v, mu):
    """CD0 build-up for a tandem airframe, referenced to S_tot.

    Same construction as the canard build-up (each wing's profile drag at
    its OWN operating Cl, Raymer fuselage form factor eq. 12.31, fin by the
    wetted-area method, 8% interference for the three real junctions);
    induced drag per wing CDi = sum (S_i/S_tot) CL_i^2/(pi AR_i e_i) with
    the mutual term inside the 0.8 rear efficiency (Lennon's
    simplification, [LEN-CAN Eq. 2-1])."""
    cd0_wing = (airfoil_r.cd_at_cl(two.cl_r, re_r) * (s_r / s_tot)
                + airfoil_f.cd_at_cl(two.cl_f, re_f) * (s_f / s_tot))
    re_fus = reynolds(rho, v, max(l_f, 0.05), mu)
    d_eq = math.sqrt(4.0 * fus_w * fus_h / math.pi)
    fr = max(l_f / max(d_eq, 1e-3), 2.0)
    ff_fus = 1.0 + 60.0 / fr**3 + fr / 400.0          # Raymer eq. 12.31
    cd0_fus = A.flat_plate_cf(max(re_fus, 3e4)) * ff_fus * (s_wet_fus / s_tot)
    re_v = reynolds(rho, v, max(fins.c_mac, 0.02), mu)
    cd0_fin = (A.flat_plate_cf(max(re_v, 3e4)) * 2.05
               * (fins.s_total / s_tot) * 1.25)
    subtotal = cd0_wing + cd0_fus + cd0_fin
    cd0_int = 0.08 * subtotal
    cd0 = subtotal + cd0_int
    cdi = ((s_r / s_tot) * two.cl_r**2 / (math.pi * ar_r * e_r)
           + (s_f / s_tot) * two.cl_f**2 / (math.pi * ar_f * e_f))
    return A.DragBreakdown(cd0_wing, cd0_fus, cd0_fin, cd0_int, cd0, cdi,
                           cd0 + cdi)


# ---------------------------------------------------------------------------
# Search driver
# ---------------------------------------------------------------------------

def optimize_tandem(inp: dict) -> dict:
    """Sweep (span, AR, Sf/Sr, stagger) inside the style bands and the box,
    polish with Nelder-Mead (k as fifth variable) and return the full design
    dict. Same contract as every other optimize_<type>."""
    style = _style_for(inp)

    b_hi = min(float(inp["box_w"]),
               float(inp["box_l"]) * 0.5 * sum(style.ar_band)
               / (0.5 * sum(style.d_mac_band) + 1.7), 2.4)
    b_lo = min(max(0.45, 0.55 * b_hi), 0.92 * b_hi)

    ar_lo, ar_hi = style.ar_band
    if inp.get("ar_target"):
        ar_lo = ar_hi = float(np.clip(inp["ar_target"], *TANDEM_AR_BAND))
    sf_lo, sf_hi = style.sf_sr_band
    d_lo, d_hi = style.d_mac_band

    spans = np.linspace(b_lo, b_hi, 4)
    ars = (np.linspace(ar_lo, ar_hi, 2) if ar_hi > ar_lo + 1e-6
           else np.array([ar_lo]))
    sfs = np.linspace(sf_lo, sf_hi, 3)
    ds = np.linspace(d_lo, d_hi, 3)

    grid = [{"span": float(b), "ar": float(a), "sf_sr": float(s),
             "d_mac": float(d)}
            for b in spans for a in ars for s in sfs for d in ds]
    for s in (inp.get("seeds") or []):
        try:
            grid.append({
                "span": float(np.clip(s["span"], 0.3, b_hi)),
                "ar": float(np.clip(s["ar"], ar_lo, ar_hi)),
                "sf_sr": float(np.clip(s.get("sf_sr", 0.5 * (sf_lo + sf_hi)),
                                       sf_lo, sf_hi)),
                "d_mac": float(np.clip(s.get("d_mac", 0.5 * (d_lo + d_hi)),
                                       d_lo, d_hi)),
            })
        except Exception:
            continue

    best = None
    for xd in grid:
        try:
            d = evaluate_tandem(xd, inp)
        except Exception:
            continue
        if best is None or d["cost"] < best["cost"]:
            best = d
    if best is None:
        raise RuntimeError("optimizer could not evaluate any candidate design")

    g = best["geometry"]
    x0 = [g["span_m"], g["aspect_ratio"], g["wing2"]["sf_sr"],
          best["stability"]["stagger_mac"],
          best["stability"]["static_margin"]]

    k_pin = inp.get("sm_override")   # a user margin stays pinned in polish

    def unpack(xv):
        return {"span": float(np.clip(xv[0], 0.3, b_hi)),
                "ar": float(np.clip(xv[1], ar_lo, ar_hi)),
                "sf_sr": float(np.clip(xv[2], sf_lo, sf_hi)),
                "d_mac": float(np.clip(xv[3], d_lo, d_hi)),
                "k": float(np.clip(k_pin if k_pin is not None else xv[4],
                                   *TANDEM_K_BAND))}

    def f(xv):
        try:
            return evaluate_tandem(unpack(xv), inp)["cost"]
        except Exception:
            return 1e9

    try:
        r = sciopt.minimize(f, x0, method="Nelder-Mead",
                            options={"maxfev": 70, "xatol": 1e-3,
                                     "fatol": 0.4})
        polished = evaluate_tandem(unpack(r.x), inp)
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
# Five tandem characters
# ---------------------------------------------------------------------------

TANDEM_VARIANTS: list[dict] = [
    {"key": "tandem_quickie", "name": "Quickie", "style": "tandem_quickie",
     "tagline": "Front wing low, rear wing high - the Rutan tandem look.",
     "knobs": {}},
    {"key": "tandem_cruiser", "name": "Tandem Tourer",
     "style": "tandem_cruiser",
     "tagline": "Longer stagger, deeper margin - the touring tandem.",
     "knobs": {}},
    {"key": "tandem_floater", "name": "Meadow Tandem",
     "style": "tandem_floater",
     "tagline": "Light loading and a big gap - slow, soft, settled.",
     "knobs": {"sd_weight": 1.6, "span_pref_frac": 0.92,
               "v_cruise_mult": 0.85}},
    {"key": "tandem_hauler", "name": "Payload Mule", "style": "tandem_hauler",
     "tagline": "Twice the wing in half the box - built to carry.",
     "knobs": {"wl_pen_weight": 0.6}},
    {"key": "tandem_speed", "name": "Twin Dart", "style": "tandem_speed",
     "tagline": "Short-coupled and thin - the fast tandem.",
     "knobs": {"span_pref_frac": 0.72, "v_cruise_mult": 1.15}},
]


def _tandem_guidance(vd: dict, d: dict) -> list[dict]:
    """Builder guidance sections - same rendering contract as guidance.py."""
    g, a, st = d["geometry"], d["aero"], d["stability"]
    w2, fus = g["wing2"], g["fuselage"]
    secs: list[dict] = []
    if not d.get("feasible", True):
        bad = [c["message"] for c in d["constraints"] if not c["ok"]]
        secs.append({"title": "Compromises - read this first",
                     "body": "\n".join(f"- {m}" for m in bad)})
    secs.append({"title": "CG, balance and rigging", "body": "\n".join([
        f"- Balance point: {st['cg_mm_from_nose']:.0f} mm from the nose - "
        f"between the wings, {st['static_margin'] * 100:.0f}% of the rear "
        "chord ahead of the neutral point (the two-surface k margin, "
        "[LEN-CAN]).",
        f"- Slide the battery to balance: the inter-wing bay runs "
        f"{fus['bay']['bay_start_m'] * 1000:.0f}-"
        f"{(fus['bay']['bay_start_m'] + fus['bay']['bay_length_m']) * 1000:.0f}"
        f" mm; the design places the pack at "
        f"{g['battery_x_m'] * 1000:.0f} mm.",
        f"- Rig the FRONT wing at {w2['incidence_deg']:+.1f} deg and the "
        f"rear at {g['root_incidence_deg']:+.1f} deg "
        f"({w2['decalage_deg']:+.1f} deg decalage) - the front wing must "
        "always work harder (CLf/CLr "
        f"{st['cl_ratio_fr']:.2f}, floor 1.2).",
        f"- The rear wing sits {w2['gap_m'] * 1000:.0f} mm HIGHER than the "
        "front one - that gap keeps it out of the front wake; do not "
        "flatten it in the build.",
    ])})
    secs.append({"title": "Control surfaces and throws", "body": "\n".join([
        f"- Elevator: full-span on the FRONT wing, "
        f"{w2['elevator_chord_frac'] * 100:.0f}% chord (plain-flap optimum); "
        "start at +-12 deg, max +-20.",
        f"- Ailerons: {g['ailerons']['chord_frac'] * 100:.0f}% chord on the "
        "outer REAR-wing panels; +-12 deg low rate. Pitch servos front, "
        "roll servos rear - separate wings, separate wire runs.",
        "- No rudder: the aft fin is fixed, sized by the lateral-area "
        "moment rule (aft/forward >= 1.25).",
    ])})
    secs.append({"title": f"Why this variant - {vd['name']}",
                 "body": "\n".join([
        f"- This is the {vd['name']}, a {d['planform_label'].lower()}: "
        f"rear wing {g['span_m'] * 1000:.0f} mm / front wing "
        f"{w2['span_m'] * 1000:.0f} mm, stagger "
        f"{st['stagger_mac']:.1f} rear chords, front share "
        f"{st['front_lift_share'] * 100:.0f}% of the lift.",
        f"- Numbers: {a['wing_loading_kgm2']:.1f} kg/m2 on the total "
        f"lifting area, {a['v_stall_ms']:.1f} m/s stall vs "
        f"{a['v_cruise_ms']:.1f} m/s cruise ({a['stall_margin']:.2f}x), "
        f"L/D {a['ld_cruise']:.1f}.",
        "- Two wings in one box means the most lifting area per millimetre "
        "of span in the app - that is the tandem's trade.",
    ])})
    return secs


def generate_tandem_variants(inp: dict) -> list[dict]:
    """Five tandem characters - same contract as every other
    generate_<type>_variants: exactly five, mission style leads, infeasible
    characters returned flagged."""
    primary_key = TANDEM_STYLE_FOR_MISSION.get(
        inp.get("mission") or "sport", "tandem_quickie")
    ctx: dict = {"seeds": []}
    designs: dict[str, dict] = {}

    def _run(vd: dict) -> dict:
        knobs = dict(vd["knobs"])
        v_mult = knobs.pop("v_cruise_mult", None)
        merged = {**inp, **knobs, "tandem_style": vd["style"],
                  "seeds": ctx.get("seeds", [])}
        if v_mult:
            merged["v_cruise"] = inp["v_cruise"] * float(v_mult)
        ref = ctx.get("v_stall_ref", 0.0)
        if vd["key"] == "tandem_floater" and ref > 0:
            merged["v_stall_goal"] = 0.85 * ref
        try:
            return optimize_tandem(merged)
        except Exception:
            d = optimize_tandem({**inp, "tandem_style": vd["style"]})
            d["notes"].append(
                f"The {vd['name']} tuning could not be solved for these "
                "inputs; showing the untuned solution for this character.")
            return d

    def _seed(d: dict) -> dict:
        g = d["geometry"]
        return {"span": g["span_m"], "ar": g["aspect_ratio"],
                "sf_sr": g["wing2"]["sf_sr"],
                "d_mac": d["stability"]["stagger_mac"]}

    lead_def = next(v for v in TANDEM_VARIANTS if v["key"] == primary_key)
    lead = _run(lead_def)
    designs[primary_key] = lead
    ctx["v_stall_ref"] = lead["aero"]["v_stall_ms"]
    ctx["seeds"].append(_seed(lead))
    for vd in TANDEM_VARIANTS:
        if vd["key"] == primary_key:
            continue
        d = _run(vd)
        designs[vd["key"]] = d
        ctx["seeds"].append(_seed(d))

    ordered = [designs[vd["key"]] for vd in TANDEM_VARIANTS]
    traits = traits_generic(ordered)
    out: list[dict] = []
    for vd, d, tr in zip(TANDEM_VARIANTS, ordered, traits):
        d["character"] = {"key": vd["key"], "name": vd["name"],
                          "tagline": vd["tagline"]}
        d["guidance"] = _tandem_guidance(vd, d)
        out.append({
            "id": d["id"],
            "key": vd["key"],
            "name": vd["name"],
            "tagline": vd["tagline"],
            "planform": d["planform"],
            "airplane_type": "tandem",
            "primary": vd["key"] == primary_key,
            "traits": tr,
            "design": d,
        })
    return out
