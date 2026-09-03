"""Tailless longitudinal + directional stability (SPEC_FLYING_WING.md 3.3, 6).

There is no tail-aft or canard solver in AeroForge any more: a flying wing's
neutral point, trim and pitch stiffness all come from ONE surface, so they are
solved on a spanwise strip model of the actual blended planform - the same
planform the CAD lofts (spec section 5), body blend included.

Coordinate frame: x measured AFT from the nose datum (x = 0 at the forward-most
point of the centre body), in metres. z up, y to starboard.

Key relations (each cited at point of use):
  blended chord        c(f)    = c_wing(f) (1 + (chord_scale - 1) w(f))
  blended LE station   x_le(f) = f (b/2) tan(Lambda_LE) + nose_ext (1 - w(f))
  strip aerodynamic centre     x_ac(f) = x_le(f) + 0.25 c(f)   (thin-airfoil)
  mean aerodynamic chord       MAC   = (1/S_h) Int c^2 dy
  spanwise MAC station         y_MAC = (1/S_h) Int c y dy
  Prandtl finite-wing slope    a = a0 / (1 + a0/(pi AR e))
  neutral point (area-weighted aerodynamic centre of the strips)
                               x_np  = (1/S_h) Int c x_ac dy
  lift                         CL    = (1/S_h) Int c a (alpha + i(y) - a_l0) dy
  moment about the CG          Cm    = (1/(S_h MAC)) Int [c cl (x_cg - x_ac)
                                       + c^2 cm0] dy
  trim CG (Cm = 0, CL fixed)   x_cg  = (X1 - cm0_term MAC) / CL
                               X1    = (1/S_h) Int c cl x_ac dy
  static margin                SM    = (x_np - x_cg)/MAC ; dCm/dalpha < 0

Directional stability is NOT solved from the tail-aft volume coefficient - see
:func:`flying_wing_fin` for why, and DECISIONS.md for the full argument.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .airfoils import Airfoil


# ---------------------------------------------------------------------------
# Planform geometry (the blended full-span wing the CAD builds)
# ---------------------------------------------------------------------------

def chords_from_area(span: float, area: float, taper: float) -> tuple[float, float]:
    """Root/tip chord of the reference TRAPEZOID: S = b (c_r + c_t)/2."""
    c_r = 2.0 * area / (span * (1.0 + taper))
    return c_r, c_r * taper


def mac_length(c_root: float, taper: float) -> float:
    """Trapezoidal MAC = (2/3) c_r (1 + lam + lam^2)/(1 + lam).

    Used only for quick reference values; the solver integrates the real
    blended chord distribution instead.
    """
    return (2.0 / 3.0) * c_root * (1 + taper + taper**2) / (1 + taper)


def y_mac(span: float, taper: float) -> float:
    """Trapezoidal spanwise MAC station: y = (b/6)(1 + 2 lam)/(1 + lam)."""
    return (span / 6.0) * (1 + 2 * taper) / (1 + taper)


def x_le_mac(sweep_le_deg: float, span: float, taper: float) -> float:
    """Chordwise LE offset of the trapezoidal MAC: x = y_MAC tan(sweep)."""
    return y_mac(span, taper) * math.tan(math.radians(sweep_le_deg))


def lift_slope_3d(a0: float, ar: float, e: float) -> float:
    """Prandtl finite-wing lift-curve slope: a = a0/(1 + a0/(pi AR e))."""
    return a0 / (1.0 + a0 / (math.pi * ar * e))


def blend_weight(f: np.ndarray | float, fb: float) -> np.ndarray | float:
    """Centre-body blend weight w(f): 1 at the centreline, 0 outboard of the
    body half-width, with a smoothstep (C1-continuous) in between.

    This is the SAME function the CAD lofts with (spec section 5), so the
    physics and the geometry describe one aeroplane, not two.
        w(f) = 1 - (3 s^2 - 2 s^3),  s = clip(f/fb, 0, 1)
    """
    if fb <= 1e-9:
        return np.zeros_like(np.asarray(f, dtype=float))
    s = np.clip(np.asarray(f, dtype=float) / fb, 0.0, 1.0)
    return 1.0 - (3.0 * s**2 - 2.0 * s**3)


@dataclass
class BlendedPlanform:
    """Discretised half-planform of the blended full-span wing."""
    span: float
    c_root_wing: float          # trapezoidal wing root chord (body/wing joint)
    taper: float
    sweep_le_deg: float
    body_half_width_frac: float  # fb = body half-width / semi-span
    body_chord_scale: float      # centre chord / wing root chord

    f: np.ndarray = field(default_factory=lambda: np.zeros(0))
    y: np.ndarray = field(default_factory=lambda: np.zeros(0))
    dy: float = 0.0
    w: np.ndarray = field(default_factory=lambda: np.zeros(0))
    c: np.ndarray = field(default_factory=lambda: np.zeros(0))
    x_le: np.ndarray = field(default_factory=lambda: np.zeros(0))
    x_ac: np.ndarray = field(default_factory=lambda: np.zeros(0))

    # integrated quantities
    s_half: float = 0.0
    area: float = 0.0
    mac: float = 0.0
    y_mac: float = 0.0
    x_le_mac: float = 0.0
    x_np: float = 0.0
    nose_ext: float = 0.0
    c_centre: float = 0.0
    c_tip: float = 0.0
    x_centroid: float = 0.0     # area centroid (used for the mass balance)
    x_te_max: float = 0.0


def build_planform(*, span: float, c_root_wing: float, taper: float,
                   sweep_le_deg: float, body_half_width_frac: float = 0.0,
                   body_chord_scale: float = 1.0,
                   n_strips: int = 120) -> BlendedPlanform:
    """Discretise the blended half-wing exactly as spec section 5 defines it.

    The nose extension is IMPLIED by the body chord scale: the centre section's
    trailing edge stays on the wing root's trailing edge and the extra chord
    goes forward, which is what a real leading-edge root extension looks like.
        nose_ext = (chord_scale - 1) c_root_wing
    Stations are then shifted so the forward-most point (the body nose) is the
    x = 0 datum - no geometry may lie at x < 0.
    """
    half = 0.5 * span
    n = int(n_strips)
    f = (np.arange(n) + 0.5) / n
    y = f * half
    dy = half / n
    w = np.asarray(blend_weight(f, body_half_width_frac), dtype=float)

    c_wing = c_root_wing * (1.0 - (1.0 - taper) * f)     # linear taper
    c = c_wing * (1.0 + (body_chord_scale - 1.0) * w)
    nose_ext = (body_chord_scale - 1.0) * c_root_wing
    x_le = y * math.tan(math.radians(sweep_le_deg)) + nose_ext * (1.0 - w)
    x_ac = x_le + 0.25 * c                                # thin-airfoil theory

    s_half = float(np.sum(c) * dy)
    pf = BlendedPlanform(
        span=span, c_root_wing=c_root_wing, taper=taper,
        sweep_le_deg=sweep_le_deg,
        body_half_width_frac=body_half_width_frac,
        body_chord_scale=body_chord_scale,
        f=f, y=y, dy=dy, w=w, c=c, x_le=x_le, x_ac=x_ac,
    )
    pf.s_half = s_half
    pf.area = 2.0 * s_half
    pf.mac = float(np.sum(c**2) * dy) / s_half            # MAC = Int c^2 / S_h
    pf.y_mac = float(np.sum(c * y) * dy) / s_half
    pf.x_le_mac = float(np.sum(c * x_le) * dy) / s_half
    pf.x_np = float(np.sum(c * x_ac) * dy) / s_half       # area-weighted AC
    pf.nose_ext = nose_ext
    pf.c_centre = float(c_root_wing * body_chord_scale)
    pf.c_tip = float(c_root_wing * taper)
    # planform-area centroid: where the skin mass actually sits
    pf.x_centroid = float(np.sum(c * (x_le + 0.5 * c)) * dy) / s_half
    pf.x_te_max = float(np.max(x_le + c))
    return pf


def chord_at(pf: BlendedPlanform, f: float) -> float:
    """Local chord at spanwise fraction f (0 = centreline, 1 = tip)."""
    ff = float(np.clip(f, 0.0, 1.0))
    w = float(blend_weight(np.array([ff]), pf.body_half_width_frac)[0])
    c_wing = pf.c_root_wing * (1.0 - (1.0 - pf.taper) * ff)
    return c_wing * (1.0 + (pf.body_chord_scale - 1.0) * w)


def x_le_at(pf: BlendedPlanform, f: float) -> float:
    """Local leading-edge station at spanwise fraction f."""
    ff = float(np.clip(f, 0.0, 1.0))
    w = float(blend_weight(np.array([ff]), pf.body_half_width_frac)[0])
    return (ff * 0.5 * pf.span * math.tan(math.radians(pf.sweep_le_deg))
            + pf.nose_ext * (1.0 - w))


# ---------------------------------------------------------------------------
# Results container
# ---------------------------------------------------------------------------

@dataclass
class StabilityResult:
    mac: float
    y_mac: float
    x_le_mac: float          # from nose datum
    x_ac_wing: float         # quarter-MAC, from nose
    x_np: float              # neutral point, from nose
    x_cg: float              # CG, from nose
    static_margin: float     # (x_np - x_cg)/MAC
    cg_pct_mac: float        # CG position as % MAC aft of MAC LE
    area: float = 0.0        # integrated planform area (blend included)
    # vertical surfaces (sized by flying_wing_fin, built to the same numbers)
    vv: float = 0.0          # reported V_V = l_V S_V/(S b) - see the note there
    s_v: float = 0.0
    l_v: float = 0.0
    fin_height: float = 0.0
    fin_chord: float = 0.0
    x_fin_ac: float = 0.0
    # trim
    washout_deg: float = 0.0
    root_incidence_deg: float = 0.0
    alpha_cruise_deg: float = 0.0
    cm0_cg: float = 0.0      # section-reflex contribution to Cm about the CG
    cm_residual: float = 0.0  # Cm left for the elevons to trim (0 = exact trim)
    dcm_dalpha: float = 0.0  # must be < 0 for stability
    bell_spanload: bool = False
    notes: list[str] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Flying-wing vertical surfaces
# ---------------------------------------------------------------------------

# Total vertical area as a fraction of wing area, for a tailless wing.
# Unswept plank at one end, well-swept wing at the other (see the docstring
# of :func:`flying_wing_fin` for where these come from).
FW_SV_FRAC_UNSWEPT = 0.065
FW_SV_FRAC_SWEPT = 0.032
FW_SWEEP_FULL_CREDIT_DEG = 25.0
FW_CENTRE_FIN_PENALTY = 1.15   # one centre fin has a shorter arm than tip fins
FW_FIN_AR = 1.2                # height^2/area of one centre fin (real practice)
FW_TIPFIN_AR = 1.3
FW_FIN_TAPER = 0.55            # tip/root chord of a trapezoidal RC fin
FW_FIN_SWEEP_DEG = 26.0        # leading-edge sweep of the fin itself
# Invariant (spec 8.7): vertical surfaces 2-9% of wing area and no taller than
# 60% of the chord they stand on.
FW_SV_FRAC_MIN = 0.020
FW_SV_FRAC_MAX = 0.090
FW_FIN_HEIGHT_MAX_CHORD_FRAC = 0.60
FW_FIN_HEIGHT_MIN_CHORD_FRAC = 0.22
FW_FIN_HEIGHT_MAX_SEMISPAN_FRAC = 0.22


@dataclass
class FinSizing:
    vstab_type: str
    count: int
    area_total: float
    height: float            # per surface
    chord_mean: float
    root_chord: float
    tip_chord: float
    sweep_le_deg: float
    cant_deg: float
    y_frac: float
    x_ac: float
    l_v: float
    vv: float


def flying_wing_fin(
    *, pf: BlendedPlanform, area: float, x_cg: float, vstab_type: str,
    y_frac: float | None = None, cant_deg: float | None = None,
) -> FinSizing:
    """Vertical-surface sizing for a tailless wing, the way real ones are built.

    Why NOT the classic tail-volume coefficient: V_V = l_V S_V/(S b) with the
    0.02-0.05 band is a TAIL-AFT correlation, and it assumes a fuselage-length
    moment arm. A flying wing's fin sits about one root chord behind the CG, so
    l_V is ~0.05-0.08 m at model scale. Solving that band for S_V demands a fin
    of 20-25% of the WING area - a sail no flying wing has ever carried, and one
    that makes the model unbalanceable in yaw.

    What actually keeps a tailless wing straight is LEADING-EDGE SWEEP: in a
    sideslip the advancing panel presents more span-normal area and more profile
    drag than the retreating one, and that drag difference acts at the swept
    panel's own arm. Sweep is the primary directional stiffness and the fin only
    trims the remainder - which is why swept wings (Zagi / Ritewing / Skywalker
    X5 class) carry 3-4% of wing area in small tip fins while an unswept plank,
    with no sweep to lean on, needs 6-7% on a centre fin. Those measured
    proportions are what this function reproduces, interpolating on sweep
    between them; the resulting V_V (0.002-0.006) is REPORTED honestly rather
    than forced into a band that does not apply.

    A single centre fin is penalised ~15%: on a swept wing the tips sit well aft
    of the centre section, so tip fins act on a materially longer arm.

    `vstab_type` is one of "winglets" (tips, canted outboard), "twin_fin"
    (inboard at 0.50-0.65 of semi-span, vertical), "center_fin" (on the body
    spine) or "none" (bell spanload: everything is zero).
    """
    from .config_defs import VSTABS

    if vstab_type == "none":
        return FinSizing("none", 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0)

    vsd = VSTABS[vstab_type]
    yf = vsd.y_frac if y_frac is None else float(np.clip(y_frac, 0.0, 0.99))
    cant = vsd.cant_deg if cant_deg is None else float(cant_deg)
    n = max(vsd.count, 1)

    # --- area from measured tailless proportions, interpolated on sweep ------
    credit = min(max(pf.sweep_le_deg, 0.0) / FW_SWEEP_FULL_CREDIT_DEG, 1.0)
    frac = FW_SV_FRAC_UNSWEPT + (FW_SV_FRAC_SWEPT - FW_SV_FRAC_UNSWEPT) * credit
    if vstab_type == "center_fin":
        frac *= FW_CENTRE_FIN_PENALTY
    frac = float(np.clip(frac, FW_SV_FRAC_MIN, FW_SV_FRAC_MAX))
    s_total = frac * area
    s_each = s_total / n

    # --- proportion guards against the surface the fin stands on -------------
    # A fin has to look right against BOTH things it is measured against on a
    # real airframe: the chord it stands on and the span of the wing it steers.
    c_local = chord_at(pf, yf)
    half = 0.5 * pf.span
    ar_fin = FW_FIN_AR if n == 1 else FW_TIPFIN_AR
    h_hi = min(FW_FIN_HEIGHT_MAX_CHORD_FRAC * c_local,
               FW_FIN_HEIGHT_MAX_SEMISPAN_FRAC * half)
    h_lo = min(FW_FIN_HEIGHT_MIN_CHORD_FRAC * c_local, h_hi)
    h = float(np.clip(math.sqrt(ar_fin * s_each), h_lo, h_hi))
    # Height is the guarded quantity; AREA is preserved by letting the chord
    # grow, so the directional stiffness the sweep interpolation asked for is
    # actually built. If that would make the fin longer than the surface it is
    # bolted to, the area gives way instead.
    c_mean = s_each / h
    if c_mean > 0.90 * c_local:
        c_mean = 0.90 * c_local
        s_each = c_mean * h
        s_total = s_each * n

    c_root_fin = 2.0 * c_mean / (1.0 + FW_FIN_TAPER)   # trapezoid mean chord
    c_tip_fin = FW_FIN_TAPER * c_root_fin

    # --- station and moment arm ---------------------------------------------
    # The fin's own aerodynamic centre sits at ~40% of its mean chord aft of
    # its leading edge (low aspect ratio, so aft of the quarter-chord point),
    # and its LE is set so its trailing edge lands on the local wing TE.
    x_le_local = x_le_at(pf, yf)
    x_fin_te = x_le_local + c_local
    x_ac_fin = x_fin_te - c_mean + 0.40 * c_mean
    l_v = max(x_ac_fin - x_cg, 1e-3)
    vv = l_v * s_total / (area * pf.span)     # V_V = l_V S_V/(S b), reported

    return FinSizing(
        vstab_type=vstab_type, count=n, area_total=float(s_total),
        height=float(h), chord_mean=float(c_mean),
        root_chord=float(c_root_fin), tip_chord=float(c_tip_fin),
        sweep_le_deg=FW_FIN_SWEEP_DEG, cant_deg=float(cant), y_frac=float(yf),
        x_ac=float(x_ac_fin), l_v=float(l_v), vv=float(vv),
    )


# ---------------------------------------------------------------------------
# Flying wing: strip model for NP, trim (washout / CG), pitch stability
# ---------------------------------------------------------------------------

def _trim_state(pf: BlendedPlanform, a3d: float, a_l0: float,
                cl_cruise: float, eps: float) -> tuple[float, float, np.ndarray]:
    """Root alpha, lift-weighted AC station X1 and the strip cl at a given
    washout eps [rad].

    Local incidence is linear in the span fraction, i(f) = -eps f, so with the
    finite-wing slope applied per strip:
        CL = (1/S_h) Int c a (alpha - eps f - a_l0) dy
    which inverts to alpha = a_l0 + CL/a + eps <f>_c, <f>_c being the
    chord-weighted mean of f.
    """
    wgt = pf.c * pf.dy / pf.s_half
    f_bar = float(np.sum(wgt * pf.f))
    alpha = a_l0 + cl_cruise / a3d + eps * f_bar
    cl = a3d * (alpha - eps * pf.f - a_l0)
    x1 = float(np.sum(pf.c * pf.dy * cl * pf.x_ac)) / pf.s_half
    return alpha, x1, cl


def solve_flying_wing(
    *, pf: BlendedPlanform, airfoil: Airfoil, re_mac: float, ar: float,
    e_oswald: float, sm_target: float, cl_cruise: float,
    washout_band: tuple[float, float] = (0.0, 7.0),
    bell_spanload: bool = False,
    sm_band: tuple[float, float] = (0.03, 0.15),
    sm_hard: bool = False,
) -> StabilityResult:
    """Tailless longitudinal solution on the blended strip model.

    Two levers set the trim: the geometric WASHOUT (root-to-tip twist) and the
    CG position. They are not independent - for a fixed cruise CL, both the
    lift-weighted AC station X1 and therefore the trim CG are LINEAR in the
    washout, so the whole trim problem closes in one linear solve:

        x_cg_trim(eps) = (X1(eps) - cm0_term * MAC) / CL          (Cm_cg = 0)
        SM(eps)        = (x_np - x_cg_trim(eps)) / MAC

    Ordinary wings: solve `eps` so that the trim CG lands exactly at the target
    static margin, then clamp `eps` into the planform's washout band.

    Bell-spanload wings: the washout is NOT a free trim variable. A bell
    spanload needs a large root-to-tip twist in its own right (NASA Prandtl-D
    carries ~9 deg; the band is 8-13 deg), because that is what unloads the tips
    enough to produce the induced thrust that gives proverse yaw. So the washout
    is driven INTO the bell band first - as close to the value that also trims
    at the target static margin as the band allows - and the CG then follows
    from the trim equation.

    In both cases the resulting static margin is clamped into `sm_band`; if the
    clamp bites, the leftover moment is reported as `cm_residual` and trimmed
    with a small elevon offset, exactly as a builder would.

    `sm_hard=True` means the caller has asked for a specific CG (the user's
    static-margin override): the CG is then placed at exactly that margin and
    whatever moment is left over is reported, rather than letting the trim
    solution move it.
    """
    a0 = airfoil.lift_slope_2d(re_mac)
    a3d = lift_slope_3d(a0, ar, e_oswald)      # Prandtl finite-wing slope
    a_l0 = math.radians(airfoil.alpha_l0_deg(re_mac))
    cm0_sec = airfoil.cm0(re_mac)              # reflexed section: cm0 > 0

    mac, x_np = pf.mac, pf.x_np
    # section pitching-moment contribution: Cm0_wing = (1/(S_h MAC)) Int c^2 cm0
    cm0_term = cm0_sec * float(np.sum(pf.c**2) * pf.dy) / (pf.s_half * mac)

    def x_cg_trim(eps: float) -> float:
        _alpha, x1, _cl = _trim_state(pf, a3d, a_l0, cl_cruise, eps)
        return (x1 - cm0_term * mac) / max(cl_cruise, 1e-6)

    # x_cg_trim is linear in eps -> two evaluations give the exact inverse
    x0 = x_cg_trim(0.0)
    x1_ = x_cg_trim(0.1)
    slope = (x1_ - x0) / 0.1                   # d(x_cg_trim)/d(eps) [m/rad]
    x_cg_want = x_np - sm_target * mac         # SM = (x_np - x_cg)/MAC
    if abs(slope) > 1e-9:
        eps_trim = (x_cg_want - x0) / slope
    else:
        eps_trim = math.radians(0.5 * (washout_band[0] + washout_band[1]))

    lo, hi = (math.radians(washout_band[0]), math.radians(washout_band[1]))
    eps = float(np.clip(eps_trim, lo, hi))
    notes: list[str] = []
    if bell_spanload:
        # the band IS the design requirement here, not a guard rail
        notes.append(
            f"Bell spanload: washout held at {math.degrees(eps):.1f} deg "
            f"(band {washout_band[0]:.0f}-{washout_band[1]:.0f} deg) so the "
            "tips stay lightly loaded and make induced thrust - that is what "
            "gives this wing proverse yaw and lets it fly with no fins.")
    elif eps_trim > hi + 1e-6:
        notes.append(
            f"Trim wanted {math.degrees(eps_trim):.1f} deg of washout; held at "
            f"the {washout_band[1]:.0f} deg limit for this planform and the "
            "remainder taken on elevon reflex.")
    elif eps_trim < lo - 1e-6:
        notes.append(
            "Section reflex alone over-trims this planform; washout held at "
            f"{washout_band[0]:.0f} deg.")

    alpha, x1v, cl_strip = _trim_state(pf, a3d, a_l0, cl_cruise, eps)
    x_cg = (x1v - cm0_term * mac) / max(cl_cruise, 1e-6)
    sm = (x_np - x_cg) / mac
    cm_residual = 0.0
    if sm_hard:
        sm_used = float(np.clip(sm_target, sm_band[0], sm_band[1]))
        x_cg = x_np - sm_used * mac
        cm_residual = (x_cg * cl_cruise - x1v) / mac + cm0_term
        if abs(sm_used - sm) > 1e-4:
            notes.append(
                f"CG held at the requested {sm_used * 100:.0f}% MAC static "
                f"margin (the wing itself trims at {sm * 100:.0f}%); Cm "
                f"{cm_residual:+.4f} is taken on elevon trim.")
        sm = sm_used
    elif not (sm_band[0] <= sm <= sm_band[1]):
        sm_used = float(np.clip(sm, sm_band[0], sm_band[1]))
        x_cg = x_np - sm_used * mac
        # Cm about the clamped CG = [x_cg CL - X1]/MAC + cm0_term
        cm_residual = (x_cg * cl_cruise - x1v) / mac + cm0_term
        notes.append(
            f"Exact trim would put the CG at {sm * 100:.0f}% MAC static "
            f"margin; held at {sm_used * 100:.0f}% (the {sm_band[0] * 100:.0f}"
            f"-{sm_band[1] * 100:.0f}% tailless band) with Cm "
            f"{cm_residual:+.4f} taken on elevon trim.")
        sm = sm_used

    # dCm/dalpha = -a3d (x_np - x_cg)/MAC : negative whenever the CG is ahead
    # of the neutral point, which the SM clamp guarantees.
    dcm_da = -a3d * (x_np - x_cg) / mac

    res = StabilityResult(
        mac=mac, y_mac=pf.y_mac, x_le_mac=pf.x_le_mac,
        x_ac_wing=pf.x_le_mac + 0.25 * mac,
        x_np=x_np, x_cg=x_cg, static_margin=sm,
        cg_pct_mac=100.0 * (x_cg - pf.x_le_mac) / mac,
        area=pf.area,
        washout_deg=math.degrees(eps),
        # The root chord line IS the model datum on a flying wing (there is no
        # fuselage to reference incidence against), so the built-in root
        # incidence is zero and the twist runs 0 -> -washout out to the tip.
        # The angle the aircraft actually flies at in cruise is reported
        # separately as alpha_cruise_deg.
        root_incidence_deg=0.0,
        alpha_cruise_deg=math.degrees(alpha),
        cm0_cg=float(cm0_term), cm_residual=float(cm_residual),
        dcm_dalpha=float(dcm_da), bell_spanload=bool(bell_spanload),
        notes=notes,
    )
    if cm0_sec <= 0:
        res.notes.append(
            "WARNING: section Cm0 <= 0 - a flying wing needs a reflexed "
            "airfoil to trim with the CG ahead of the neutral point.")
    return res
