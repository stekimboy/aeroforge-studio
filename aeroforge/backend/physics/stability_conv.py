"""Tail-volume stability, trim and tail sizing for CONVENTIONAL airplanes.

NEW in v2 (V2_PLAN.md): this module is the tailed counterpart of
`stability.py` and never touches it - the flying-wing math lives there,
untouched. Coordinate frame is the same: x measured AFT from the nose datum
(x = 0 at the fuselage nose / motor face), metres; z up, y to starboard.

Key relations (each cited at point of use; numbers from
RESEARCH_CONVENTIONAL.md, referenced by its source tags [MIT]/[ND]/[SCHOLZ]):

  tail volumes        V_H = S_h l_h / (S c_mac) ;  V_V = S_v l_v / (S b) [MIT]
  downwash slope      deps/dalpha = 2 a_w / (pi AR)          (elliptic wing,
                      Nelson "Flight Stability and Automatic Control" eq. 2.23)
  neutral point       lift-slope-weighted aerodynamic centre of wing + tail
                      (the exact form of the classic
                      x_np = x_ac_w + eta_h V_H (a_h/a_w)(1 - deps/da) c_mac,
                      Nelson eq. 2.34), shifted forward by the fuselage's
                      Munk destabilising moment
  fuselage moment     dCm/dalpha_fus = 2 k (k2-k1) Vol / (S c_mac)  (Munk 1924
                      apparent-mass slender-body result; (k2-k1) ~ 0.9 at the
                      fineness ratios these fuselages have)
  trim                Cm_cg = Cm_ac,w + CL_w (x_cg - x_ac,w)/c
                                - eta_h V_H CL_h = 0
                      solved for CL_h, then the STAB INCIDENCE i_h is set so
                      the elevator is neutral in cruise (decalage, dossier s.4)
  static margin       SM = (x_np - x_cg)/MAC, band CONV_SM_BAND = 0.05-0.15
                      [MIT] eq. (2); trainer target 0.12, sport 0.10,
                      aerobatic 0.06 (dossier s.3)

Handling checks are the TAILED analogues of physics/handling.py. The
tailless-calibrated thresholds there explicitly do NOT apply here: a tailed
model runs Cn_beta ~ 0.05-0.15 per radian (that module's own calibration
note), an order of magnitude above a flying wing, because the fin works at a
genuine fuselage-length arm.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from .airfoils import Airfoil
from .config_defs import CONV_SM_BAND
from .handling import flap_effectiveness
from .stability import lift_slope_3d, mac_length

# ---------------------------------------------------------------------------
# Tail geometry conventions
# ---------------------------------------------------------------------------
AR_H = 3.0            # stab aspect ratio ~3 [ND]
TAPER_H = 0.70        # gentle stab taper (classic trainer stab planform)
AR_V = 1.5            # fin height^2 / area, hobby practice (fins are stubby)
TAPER_V = 0.60
FIN_SWEEP_DEG = 30.0  # fin LE sweep - the classic raked RC fin outline

# Dynamic-pressure ratio at the tail. A conventional tail sits behind the
# fuselage and in the wing wake, so 0.90 rather than the 0.95 a clean
# wing-mounted flying-wing fin gets (see handling.py's fin note; Nelson ch.2
# uses eta_h = 0.9 for a fuselage-mounted tail).
ETA_H = 0.90
ETA_V = 0.90

E_TAIL = 0.80         # span efficiency of the low-AR stab (Prandtl slope)
A0_TAIL = 2.0 * math.pi  # thin-airfoil 2D slope of the symmetric tail section

# Munk apparent-mass factor (k2 - k1) for fineness ratios 6-10 (Munk 1924,
# "The aerodynamic forces on airship hulls").
K_MUNK = 0.90

# Fin end-plate credit: the stab mounted at the fin root raises the fin's
# effective aspect ratio by ~1.55x (Raymer, "Aircraft Design", tail chapter).
FIN_ENDPLATE_AR_MULT = 1.55

# Elevator travel actually used for authority accounting: the dossier's
# recommended high-rate elevator throw is +-18 deg (s.5 throws table).
ELEVATOR_MAX_DEG = 18.0

# TAILED handling calibration (NOT the tailless numbers in handling.py):
# a conventional model runs Cn_beta 0.05-0.15 /rad (handling.py's own tailed
# reference; Etkin & Reid typical values). Below ~0.03 the fin volume is not
# doing its job even by RC standards.
CN_BETA_MIN_TAILED = 0.03
CN_BETA_GOOD_TAILED = 0.05
# |Cl_beta|/Cn_beta balance thresholds for a TAILED model (Etkin & Reid):
# far lower than the flying wing's 6.0, because the fin is supposed to win.
RATIO_DUTCH_ROLL_TAILED = 2.5
RATIO_SPIRAL_TAILED = 0.25

# High-wing "keel effect": the fuselage crossflow under a high wing acts like
# extra dihedral. RC practice records the spread as "high wing 1 deg, mid
# 2 deg, low 3 deg" of GEOMETRIC dihedral for the same net effect
# (RESEARCH_CONVENTIONAL.md s.4, RCU thread) - i.e. the high position is worth
# roughly +1.5 deg of equivalent dihedral over a low wing.
HIGH_WING_EQUIV_DIHEDRAL_DEG = 1.5

# [MIT] spiral-stability parameter B = (l_v/b)(Gamma_deg/C_L): > 5 spirally
# stable. Most RC models sit below it and are flown anyway, so it is reported
# as a note, not a hard gate.
SPIRAL_B_STABLE = 5.0


# ---------------------------------------------------------------------------
# Tail sizing from the volume coefficients
# ---------------------------------------------------------------------------

@dataclass
class TailSizing:
    """Both tail surfaces, placed on the fuselage. All x from the nose datum."""
    # horizontal stabilizer
    s_h: float
    span_h: float
    c_root_h: float
    c_tip_h: float
    c_mac_h: float
    x_le_h: float
    x_ac_h: float
    l_h: float          # arm used for the sizing (CG estimate -> stab c/4)
    vh: float           # volume actually built at that arm
    # vertical fin
    s_v: float
    h_v: float
    c_root_v: float
    c_tip_v: float
    c_mac_v: float
    x_le_v: float
    x_ac_v: float
    l_v: float
    vv: float
    x_te: float         # aft-most structure (the fuselage tail post)


def size_tail(*, area: float, span: float, mac: float, x_cg: float,
              vh_target: float, vv_target: float, lh_mac: float) -> TailSizing:
    """Size and place both tail surfaces from the volume-coefficient bands.

    S_h from V_H = S_h l_h/(S c) with l_h = lh_mac x MAC (CG -> stab
    quarter-chord, [MIT] definition; arm band 2.5-3.5 MAC per [ND], dossier
    s.2). The stab is an AR-3, taper-0.7 trapezoid [ND] with an unswept LE, so
    its MAC quarter-chord sits 0.25 c_mac_h behind its LE. The fin is sized
    from V_V = S_v l_v/(S b) at the arm its own placed geometry provides
    (l_v ~ l_h on a conventional tail, dossier s.2), with its root trailing
    edge on the tail post so rudder and elevator hinge lines meet there.
    """
    l_h = lh_mac * mac
    s_h = vh_target * area * mac / max(l_h, 1e-6)
    span_h = math.sqrt(AR_H * s_h)                      # AR = b^2/S
    c_root_h = 2.0 * s_h / (span_h * (1.0 + TAPER_H))   # S = b (c_r + c_t)/2
    c_tip_h = TAPER_H * c_root_h
    c_mac_h = mac_length(c_root_h, TAPER_H)
    x_ac_h = x_cg + l_h
    x_le_h = x_ac_h - 0.25 * c_mac_h                    # unswept stab LE
    x_te_h = x_le_h + c_root_h

    # fin: first pass at l_v = l_h, then re-derive the area at the arm the
    # placed fin actually has (one pass is enough - l_v moves by millimetres)
    l_v = l_h
    for _ in range(2):
        s_v = vv_target * area * span / max(l_v, 1e-6)
        h_v = math.sqrt(AR_V * s_v)                     # AR_V = h^2/S_v
        c_root_v = 2.0 * s_v / (h_v * (1.0 + TAPER_V))
        c_tip_v = TAPER_V * c_root_v
        c_mac_v = mac_length(c_root_v, TAPER_V)
        x_le_v = x_te_h - c_root_v                      # root TE on tail post
        # low-AR swept fin: AC near 30% of the root chord plus the sweep of
        # the mid-height station (same low-AR-aft-AC argument as the flying
        # wing's fin in stability.flying_wing_fin)
        x_ac_v = (x_le_v + 0.30 * c_root_v
                  + 0.35 * h_v * math.tan(math.radians(FIN_SWEEP_DEG)))
        l_v = max(x_ac_v - x_cg, 1e-3)
    vv = l_v * s_v / (area * span)
    vh = l_h * s_h / (area * mac)

    return TailSizing(
        s_h=float(s_h), span_h=float(span_h), c_root_h=float(c_root_h),
        c_tip_h=float(c_tip_h), c_mac_h=float(c_mac_h), x_le_h=float(x_le_h),
        x_ac_h=float(x_ac_h), l_h=float(l_h), vh=float(vh),
        s_v=float(s_v), h_v=float(h_v), c_root_v=float(c_root_v),
        c_tip_v=float(c_tip_v), c_mac_v=float(c_mac_v), x_le_v=float(x_le_v),
        x_ac_v=float(x_ac_v), l_v=float(l_v), vv=float(vv),
        x_te=float(max(x_te_h, x_le_v + c_root_v)),
    )


# ---------------------------------------------------------------------------
# Longitudinal: neutral point, CG, trim (stab incidence / decalage)
# ---------------------------------------------------------------------------

@dataclass
class ConvStability:
    mac: float
    y_mac: float
    x_le_mac: float          # = wing LE station (unswept LE)
    x_ac_wing: float
    x_np: float
    x_cg: float
    static_margin: float
    cg_pct_mac: float
    # tail contribution (the V2_PLAN.md "tail_contribution" fields)
    x_np_shift_tail_m: float   # how far the tail pushes the NP aft
    x_np_shift_fus_m: float    # how far the fuselage pulls it forward (Munk)
    deps_dalpha: float
    a_wing: float            # finite-wing lift slope [1/rad]
    a_tail: float
    a_total: float           # whole-aircraft dCL/dalpha [1/rad]
    cl_w_cruise: float
    cl_h_cruise: float
    # rigging
    incidence_w_deg: float   # wing incidence on the fuselage datum
    incidence_h_deg: float   # stab incidence (elevator neutral in cruise)
    decalage_deg: float      # i_w - i_h
    alpha_cruise_deg: float  # fuselage datum flies level: wing alpha = i_w
    cm0_wing: float          # section Cm_ac at cruise Re (negative: cambered)
    cm_residual: float       # 0 by construction - the stab incidence trims it
    dcm_dalpha: float
    l_h: float
    l_v: float
    vh: float
    vv: float
    notes: list[str] = field(default_factory=list)


def solve_conventional(
    *, span: float, area: float, mac: float, y_mac_m: float,
    x_le_wing: float, airfoil: Airfoil, re_mac: float, ar: float,
    e_oswald: float, tail: TailSizing, sm_target: float, cl_cruise: float,
    fus_volume_m3: float, sm_band: tuple[float, float] = CONV_SM_BAND,
) -> ConvStability:
    """Tailed longitudinal solution: NP with tail + fuselage terms, CG from
    the static-margin target, stab incidence from cruise trim.

    Unlike the flying wing there are two trim devices and neither is washout:
    the CG target picks the margin ([MIT] band via `sm_band`) and the STAB
    INCIDENCE absorbs the wing's pitching moment so the elevator sits neutral
    in cruise - which is exactly how a builder rigs decalage (dossier s.4:
    "pick decalage so cruise elevator deflection ~ 0").
    """
    a0 = airfoil.lift_slope_2d(re_mac)
    a_w = lift_slope_3d(a0, ar, e_oswald)          # Prandtl finite-wing slope
    a_h = lift_slope_3d(A0_TAIL, AR_H, E_TAIL)
    # elliptic-wing downwash slope at the tail (Nelson eq. 2.23)
    deps = 2.0 * a_w / (math.pi * ar)
    x_ac_w = x_le_wing + 0.25 * mac                # thin-airfoil wing AC
    sh_s = tail.s_h / max(area, 1e-9)
    k_tail = ETA_H * sh_s * a_h * (1.0 - deps)     # tail's share of dCL/da
    a_total = a_w + k_tail

    # Neutral point = lift-slope-weighted AC of wing + tail (exact form of
    # Nelson eq. 2.34; reduces to x_ac_w + eta V_H (a_h/a_w)(1-deps) c for a
    # short-coupled tail).
    x_np_wt = (a_w * x_ac_w + k_tail * tail.x_ac_h) / a_total
    # Munk slender-body fuselage moment: dCm/da_fus = 2 (k2-k1) Vol/(S c),
    # destabilising, i.e. it pulls the NP forward by dCm/da_fus * c / a_total.
    dcm_da_fus = 2.0 * K_MUNK * max(fus_volume_m3, 0.0) / (area * mac)
    shift_fus = dcm_da_fus * mac / a_total
    x_np = x_np_wt - shift_fus

    notes: list[str] = []
    sm = float(min(max(sm_target, sm_band[0]), sm_band[1]))
    if abs(sm - sm_target) > 1e-6:
        notes.append(
            f"Static-margin target {sm_target:.2f} clipped into the tailed "
            f"band {sm_band[0]:.2f}-{sm_band[1]:.2f} ([MIT] ideal SM).")
    x_cg = x_np - sm * mac                          # SM = (x_np - x_cg)/MAC

    l_h = tail.x_ac_h - x_cg
    l_v = tail.x_ac_v - x_cg
    vh = tail.s_h * l_h / (area * mac)              # V_H = S_h l_h/(S c) [MIT]
    vv = tail.s_v * l_v / (area * span)             # V_V = S_v l_v/(S b) [MIT]

    # ---- cruise trim --------------------------------------------------------
    # Cm_cg = Cm_ac,w + CL_w (x_cg - x_ac_w)/c - eta_h V_H CL_h = 0, and the
    # total lift is shared: CL = CL_w + eta_h (S_h/S) CL_h. Two-variable fixed
    # point, converges in a couple of passes (CL_h is small).
    cm0_w = airfoil.cm0(re_mac)                     # cambered: Cm_ac < 0
    cl_h = 0.0
    cl_w = cl_cruise
    for _ in range(4):
        cl_w = cl_cruise - ETA_H * sh_s * cl_h
        cl_h = (cm0_w + cl_w * (x_cg - x_ac_w) / mac) / (ETA_H * vh)
    # wing incidence with the fuselage datum level in cruise (alpha_fus = 0):
    # i_w = CL_w/a_w + alpha_L0 (per the finite-wing lift line)
    i_w = math.degrees(cl_w / a_w) + airfoil.alpha_l0_deg(re_mac)
    # downwash angle at the tail in cruise: eps = 2 CL_w/(pi AR) (elliptic)
    eps = 2.0 * cl_w / (math.pi * ar)
    # stab incidence for a neutral elevator: CL_h = a_h (i_h - eps)
    i_h = math.degrees(cl_h / a_h + eps)
    decalage = i_w - i_h
    # dossier s.4: decalage +1.5..+2.5 deg trainer, ~0.5-1 sport, 0 aerobatic;
    # outside 0..3 deg the rigging looks wrong on the bench even if it trims.
    if not (-0.5 <= decalage <= 3.0):
        notes.append(
            f"Decalage {decalage:+.1f} deg is outside the 0..+3 deg rigging "
            "range RC practice uses (RESEARCH_CONVENTIONAL.md s.4) - check "
            "the CG target against the section camber.")
    if cl_h > 0.05:
        notes.append(
            f"The stab LIFTS in cruise (CL_h {cl_h:+.2f}): the CG is far "
            "enough aft that the tail carries load upward - normal near the "
            "aft limit, but it steals stall margin from the tail.")

    # dCm/dalpha about the CG (fuselage already inside x_np)
    dcm_da = -a_total * (x_np - x_cg) / mac

    return ConvStability(
        mac=float(mac), y_mac=float(y_mac_m), x_le_mac=float(x_le_wing),
        x_ac_wing=float(x_ac_w), x_np=float(x_np), x_cg=float(x_cg),
        static_margin=float(sm),
        cg_pct_mac=float(100.0 * (x_cg - x_le_wing) / mac),
        x_np_shift_tail_m=float(x_np_wt - x_ac_w),
        x_np_shift_fus_m=float(-shift_fus),
        deps_dalpha=float(deps), a_wing=float(a_w), a_tail=float(a_h),
        a_total=float(a_total),
        cl_w_cruise=float(cl_w), cl_h_cruise=float(cl_h),
        incidence_w_deg=float(i_w), incidence_h_deg=float(i_h),
        decalage_deg=float(decalage), alpha_cruise_deg=float(i_w),
        cm0_wing=float(cm0_w), cm_residual=0.0, dcm_dalpha=float(dcm_da),
        l_h=float(l_h), l_v=float(l_v), vh=float(vh), vv=float(vv),
        notes=notes,
    )


# ---------------------------------------------------------------------------
# Lateral / directional handling - TAILED thresholds
# ---------------------------------------------------------------------------

@dataclass
class ConvHandling:
    cn_beta: float           # per radian, > 0 weathercock-stable
    cn_beta_fin: float
    cn_beta_fus: float       # Munk term, destabilising (negative)
    cl_beta: float           # per radian, < 0 roll-stable
    cl_beta_dihedral: float
    cl_beta_wing_pos: float  # high-wing keel-effect equivalent
    cl_beta_fin: float
    ratio: float             # |Cl_beta| / Cn_beta
    spiral_b: float          # [MIT] B = (l_v/b)(Gamma/CL); > 5 spirally stable
    elevator_authority: float  # Cm available at full elevator
    elevator_trim_used: float  # fraction spent pulling to CL_max
    notes: list[str] = field(default_factory=list)


def conv_handling(
    *, area: float, span: float, mac: float, y_bar: float, cl_cruise: float,
    a_wing: float, a_total: float, dihedral_deg: float, wing_position: str,
    tail: TailSizing, vh: float, vv: float, l_v: float, a_tail: float,
    fus_volume_m3: float, fus_height_m: float, static_margin: float,
    cl_max: float, elevator_frac: float,
) -> ConvHandling:
    """Directional / lateral stability and elevator authority, judged against
    TAILED bands. The tailless thresholds in physics/handling.py do NOT apply:
    here the fin works at a real fuselage arm and the model is expected to run
    Cn_beta 0.05-0.15 /rad (that module's own tailed calibration note)."""
    notes: list[str] = []

    # --- weathercock: fin at its volume arm vs the destabilising fuselage ----
    # Cn_beta_fin = eta_v V_V a_v (Etkin & Reid), with the fin's effective AR
    # raised by the stab end-plate (Raymer).
    a_v = lift_slope_3d(A0_TAIL, FIN_ENDPLATE_AR_MULT * AR_V, 0.85)
    cn_fin = ETA_V * vv * a_v
    # Munk slender-body yaw: Cn_beta_fus = -2 (k2-k1) Vol/(S b), destabilising
    cn_fus = -2.0 * K_MUNK * max(fus_volume_m3, 0.0) / (area * span)
    cn_beta = cn_fin + cn_fus

    # --- dihedral effect ------------------------------------------------------
    # Strip theory: a panel at dihedral Gamma in sideslip beta sees delta-alpha
    # = +-beta Gamma, so Cl_beta = -(2 a Gamma/(S b)) Int c y dy = -a Gamma
    # (y_bar/b) with y_bar the half-wing area centroid (same integral the
    # flying-wing strip model evaluates numerically in handling._strip_terms).
    gam = math.radians(dihedral_deg)
    cl_dih = -a_wing * gam * (y_bar / span)
    # high-wing keel effect as equivalent dihedral (dossier s.4 spread)
    gam_pos = math.radians(
        HIGH_WING_EQUIV_DIHEDRAL_DEG if wing_position == "high" else 0.0)
    cl_pos = -a_wing * gam_pos * (y_bar / span)
    # fin above the CG rolls against the slip: Cl_beta_v = -eta (S_v z_v)/(S b) a_v
    z_v = 0.5 * fus_height_m + 0.45 * tail.h_v
    cl_fin = -ETA_V * (tail.s_v * z_v) / (area * span) * a_v
    cl_beta = cl_dih + cl_pos + cl_fin
    ratio = abs(cl_beta) / cn_beta if cn_beta > 1e-9 else float("inf")

    # --- spiral parameter [MIT]: B = (l_v/b)(Gamma_deg/C_L) > 5 stable --------
    gam_eff_deg = dihedral_deg + math.degrees(gam_pos)
    spiral_b = (l_v / span) * (gam_eff_deg / max(cl_cruise, 0.05))

    # --- elevator authority ---------------------------------------------------
    # Cm_delta_e = -eta_h V_H a_h tau (Nelson eq. 2.47 with the plain-flap
    # effectiveness tau of the elevator's chord fraction); the trim demand is
    # what pulling from cruise CL to CL_max costs against dCm/dalpha.
    tau = flap_effectiveness(elevator_frac)
    cm_de = ETA_H * vh * a_tail * tau
    authority = cm_de * math.radians(ELEVATOR_MAX_DEG)
    d_alpha = max(cl_max - cl_cruise, 0.0) / max(a_total, 1e-6)
    cm_need = a_total * static_margin * d_alpha
    trim_used = cm_need / authority if authority > 1e-9 else float("inf")

    # --- readable assessment (tailed bands) ----------------------------------
    if cn_beta <= 0:
        notes.append(
            "Directionally UNSTABLE: the fuselage side area overpowers the "
            "fin. More fin area or a longer tail arm is required.")
    elif cn_beta < CN_BETA_GOOD_TAILED:
        notes.append(
            f"Directional stiffness Cn_beta {cn_beta:.3f}/rad is below the "
            f"{CN_BETA_GOOD_TAILED:.2f}-0.15 band a tailed model normally "
            "flies at - it will track, but lazily. A touch more fin volume "
            "would sharpen it.")
    else:
        notes.append(
            f"Directional stiffness Cn_beta {cn_beta:.3f}/rad (fin "
            f"{cn_fin:.3f}, fuselage {cn_fus:.3f}) - inside the 0.05-0.15 "
            "band of a well-mannered tailed model.")
    if cl_beta >= 0:
        notes.append(
            "No dihedral effect: a sideslip rolls it further into the slip. "
            "Add dihedral (2-3 deg is the high-wing trainer norm).")
    if math.isfinite(ratio) and ratio > RATIO_DUTCH_ROLL_TAILED:
        notes.append(
            f"Roll stability dominates yaw stiffness ({ratio:.1f}:1): "
            "Dutch-roll-prone for a tailed model - less dihedral or more fin "
            "settles it.")
    elif math.isfinite(ratio) and ratio < RATIO_SPIRAL_TAILED:
        notes.append(
            f"Yaw stiffness dominates roll stability ({ratio:.1f}:1): it "
            "will slowly tighten a bank on its own (spiral mode). Normal - "
            "keep a hand on it in long turns.")
    if spiral_b < SPIRAL_B_STABLE:
        notes.append(
            f"Spiral parameter B = {spiral_b:.1f} (< 5, [MIT]): mildly "
            "spirally divergent, like almost every aileron-equipped RC "
            "model - it needs a pilot in a sustained bank, nothing more.")
    if trim_used > 0.5:
        notes.append(
            f"Pulling to CL_max uses {trim_used * 100:.0f}% of the elevator "
            "travel - flare authority is thin. A shallower static margin or "
            "a bigger elevator fraction buys it back.")
    return ConvHandling(
        cn_beta=float(cn_beta), cn_beta_fin=float(cn_fin),
        cn_beta_fus=float(cn_fus), cl_beta=float(cl_beta),
        cl_beta_dihedral=float(cl_dih), cl_beta_wing_pos=float(cl_pos),
        cl_beta_fin=float(cl_fin), ratio=float(ratio),
        spiral_b=float(spiral_b), elevator_authority=float(authority),
        elevator_trim_used=float(trim_used), notes=notes,
    )
