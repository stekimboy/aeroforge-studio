"""Lateral/directional stability and control authority - "how will it fly?"

Everything else in `physics/` answers whether the wing BALANCES: neutral point,
static margin, trim washout, stall margin. That is the longitudinal axis only,
and it is not what people mean when they ask whether an aeroplane handles well.
A tailless wing can be perfectly trimmed in pitch and still be unflyable: it
can wallow in yaw, drop a wing in a turn, or run out of elevon in the flare.

This module adds the other two axes, computed on the SAME spanwise strip model
the neutral point comes from rather than from remembered coefficients:

  Cn_beta   directional ("weathercock") stiffness - must be > 0 or the model
            hunts in yaw and will not track
  Cl_beta   dihedral effect - must be < 0 or a sideslip rolls it further in,
            which is spiral divergence
  |Cl_beta| / Cn_beta   the balance between them, which is what decides
            whether a design is Dutch-roll-prone or spirally divergent

MECHANISMS, and why a strip model gets them right for this configuration:

* SWEEP. In a sideslip the freestream meets the two panels at different
  effective sweep angles: the windward panel is effectively less swept, the
  leeward panel more. The velocity component normal to the quarter-chord line
  scales as cos(Lambda -/+ beta), so the local dynamic pressure scales as
  cos^2(Lambda -/+ beta) and

      d/dbeta [cos^2(Lambda -/+ beta)]|_0 = -/+ sin(2 Lambda)

  The windward panel therefore gains lift and drag, the leeward panel loses
  both. The lift difference rolls the model away from the sideslip (a
  stabilising, negative Cl_beta that grows with CL and sweep - this is why
  swept wings need little or no geometric dihedral) and the drag difference
  yaws it into the sideslip (a stabilising, positive Cn_beta). That second
  term is the one that lets a tailless wing fly with fins far smaller than a
  tail-volume calculation would demand - see stability.flying_wing_fin.

* GEOMETRIC DIHEDRAL. A panel with dihedral Gamma in a sideslip beta sees a
  local incidence change of +/- beta*Gamma, giving the classic strip-theory
  dihedral effect.

* VERTICAL SURFACES. A fin of area S_v with its aerodynamic centre l_v behind
  and z_v above the CG contributes
      Cn_beta_v = eta (S_v l_v)/(S b) a_v      (stabilising)
      Cl_beta_v = -eta (S_v z_v)/(S b) a_v     (stabilising when above the CG)
  eta is the dynamic-pressure ratio at the fin; on a tailless wing the fins sit
  in clean flow rather than behind a fuselage, so 0.95 rather than the 0.9 used
  for a tail behind a body.

CONTROL AUTHORITY. Elevons are the only pitch control a wing has, and unlike a
tailed model they are also the trim device, so their remaining travel is what
is left for the flare. Authority is estimated from the elevon's own area
moment about the CG with a plain-flap effectiveness.

References for the flap effectiveness and the fin terms: Etkin & Reid,
"Dynamics of Flight"; Nelson, "Flight Stability and Automatic Control".
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# Plain-flap (elevon) lift effectiveness: d(alpha_eff)/d(delta) as a function
# of the flap-chord ratio. The 1/2-power fit reproduces the standard
# thin-airfoil/flap curve to within a few percent over 0.15 < cf/c < 0.40,
# which covers every elevon this app generates.
def flap_effectiveness(chord_frac: float) -> float:
    cf = min(max(float(chord_frac), 0.02), 0.6)
    return float(np.clip(1.9 * math.sqrt(cf) - 1.05 * cf, 0.05, 0.85))


ETA_FIN = 0.95          # dynamic-pressure ratio at a wing-mounted fin
ELEVON_MAX_DEG = 20.0   # usable one-way travel before a plain flap stalls

# Calibration for TAILLESS aircraft. A tailed model runs Cn_beta 0.05-0.15
# per radian; a flying wing runs an order of magnitude lower and is flown
# that way quite happily, so judging one against tailed numbers just labels
# every design "weak". These thresholds are set where the handling actually
# changes character for a wing.
CN_BETA_LOW = 0.008        # below this it genuinely wanders in gusts
# |Cl_beta|/Cn_beta: wings are Dutch-roll-prone by nature (this is the
# familiar "tail wag"), so the warning belongs well above a tailed value.
RATIO_DUTCH_ROLL = 6.0
RATIO_SPIRAL = 0.8


@dataclass
class Handling:
    cn_beta: float          # per radian, > 0 is weathercock-stable
    cl_beta: float          # per radian, < 0 is roll-stable
    cn_beta_fin: float
    cn_beta_sweep: float
    cl_beta_sweep: float
    cl_beta_dihedral: float
    cl_beta_fin: float
    ratio: float            # |Cl_beta| / Cn_beta
    elevon_authority: float  # Cm available at full deflection
    elevon_trim_used: float  # fraction of that spent holding trim at CLmax
    # > 0 = proverse (nose swings INTO the turn on roll input, no rudder
    # co-ordination needed); < 0 = the usual adverse yaw
    proverse_yaw: float = 0.0
    notes: list[str] = field(default_factory=list)


def _strip_terms(pf, cl_local: np.ndarray, cd_local: np.ndarray,
                 a_local: float, sweep_deg: float, dihedral_deg: float,
                 area: float, span: float) -> tuple[float, float, float, float]:
    """Sweep and dihedral contributions from the half-wing strips.

    Returns (cl_beta_sweep, cn_beta_sweep, cl_beta_dihedral, _unused).
    Both half-wings are handled at once by symmetry: the windward panel gains
    what the leeward panel loses, so each effect is twice the half-wing
    integral of the perturbation acting at its own spanwise arm.
    """
    lam = math.radians(max(sweep_deg, 0.0))
    dsin = math.sin(2.0 * lam)                 # d/dbeta of cos^2(sweep)
    dy = pf.dy
    y = pf.y
    c = pf.c

    # Lift perturbation +/- c*cl*sin(2L) per unit beta, acting at arm y.
    # Rolling moment is negative (stabilising) for a positive sweep.
    roll_sweep = -dsin * float(np.sum(c * cl_local * y) * dy) * 2.0
    cl_beta_sweep = roll_sweep / (area * span)

    # Drag perturbation, same form, yawing the model INTO the sideslip.
    yaw_sweep = dsin * float(np.sum(c * cd_local * y) * dy) * 2.0
    cn_beta_sweep = yaw_sweep / (area * span)

    # Geometric dihedral: delta-alpha = +/- beta * Gamma on the two panels.
    gam = math.radians(dihedral_deg)
    roll_dih = -a_local * gam * float(np.sum(c * y) * dy) * 2.0
    cl_beta_dihedral = roll_dih / (area * span)
    return cl_beta_sweep, cn_beta_sweep, cl_beta_dihedral, 0.0


def handling_qualities(
    *, pf, area: float, span: float, cl_cruise: float, cd_cruise: float,
    a_wing: float, sweep_le_deg: float, dihedral_deg: float,
    fin_area: float, fin_arm: float, fin_height: float, fin_count: int,
    a_fin: float, mac: float, cl_max: float,
    elevon_inner: float, elevon_outer: float, elevon_chord_frac: float,
    cm_trim_residual: float = 0.0, bell_spanload: bool = False,
    cd0_profile: float = 0.012,
) -> Handling:
    """Full lateral/directional picture plus elevon authority."""
    notes: list[str] = []
    n = len(pf.c) if len(pf.c) else 1
    cl_local = np.full(n, cl_cruise)
    cd_local = np.full(n, max(cd_cruise, 1e-4))

    # --- yaw due to ROLL input: adverse or proverse -------------------------
    # This, not Cn_beta, is what the bell spanload buys. Deflecting an elevon
    # changes the local lift, and the induced drag that comes with it is
    #     d(cd_i) = alpha_i(y) * d(cl)
    # so the sign of the LOCAL INDUCED ANGLE where the elevons live decides
    # which way the nose goes. On an elliptical spanload alpha_i is downwash
    # everywhere, the down-going elevon gains drag, and the nose swings OUT of
    # the turn - adverse yaw, the reason tailed models need rudder co-ordination.
    #
    # A bell spanload's induced angle is quadratic, alpha_i ~ (1 - 3 eta^2),
    # reversing at eta = 1/sqrt(3) = 0.577. Elevons outboard of that station sit
    # in UPWASH: the down-going one loses drag and the nose swings INTO the
    # turn. That is proverse yaw, and it is the entire reason a Horten flies
    # co-ordinated turns with no fin and no rudder (Prandtl 1933; NASA
    # Prandtl-D). It is NOT weathercock stiffness - a bell wing is genuinely
    # soft in Cn_beta and is flown that way.
    fi_e, fo_e = sorted((float(elevon_inner), float(elevon_outer)))
    el_mask = (pf.f >= fi_e) & (pf.f <= fo_e)
    eta_all = np.clip(pf.y / max(0.5 * span, 1e-9), 0.0, 1.0)
    if bell_spanload:
        alpha_i_shape = 1.0 - 3.0 * eta_all ** 2
    else:
        alpha_i_shape = np.ones(n)          # elliptical: uniform downwash
    if np.any(el_mask):
        w_el = pf.c[el_mask]
        proverse = -float(np.sum(alpha_i_shape[el_mask] * w_el)
                          / max(np.sum(w_el), 1e-9))
    else:
        proverse = -1.0

    if bell_spanload and n > 1:
        # PRANDTL'S 1933 BELL SPANLOAD. Circulation follows (1 - eta^2)^(3/2),
        # and the induced angle it produces is QUADRATIC in span:
        #     alpha_i(eta) proportional to (1 - 3 eta^2)
        # which changes sign at eta = 1/sqrt(3) = 0.577. Outboard of that
        # station the wing sees local UPWASH, so its induced drag goes
        # negative - the tips produce induced THRUST. In a sideslip the
        # windward tip makes more of it than the leeward one, and the result
        # is a yawing moment INTO the turn: proverse yaw, the entire reason a
        # Horten needs no fin.
        #
        # A uniform-drag strip model cannot see any of that; it just reports
        # such a wing as directionally weak. Given AeroForge offers the bell
        # planform specifically for this property, the distribution has to be
        # modelled rather than averaged away.
        eta = np.clip(pf.y / max(0.5 * span, 1e-9), 0.0, 1.0)
        gamma = np.power(np.clip(1.0 - eta ** 2, 0.0, 1.0), 1.5)
        # scale the local Cl so the wing still carries the required total CL
        denom = float(np.sum(gamma * pf.c) * pf.dy)
        if denom > 1e-12:
            cl_local = cl_cruise * gamma * (0.5 * area) / denom
        # scale the induced angle so total induced drag matches the polar
        shape = 1.0 - 3.0 * eta ** 2
        cdi_shape = float(np.sum(pf.c * cl_local * shape) * pf.dy) * 2.0 / area
        cdi_total = max(cd_cruise - cd0_profile, 0.0)
        k = (cdi_total / cdi_shape) if abs(cdi_shape) > 1e-9 else 0.0
        cd_local = np.maximum(cd0_profile + k * cl_local * shape, -0.05)

    cl_b_sw, cn_b_sw, cl_b_dih, _ = _strip_terms(
        pf, cl_local, cd_local, a_wing, sweep_le_deg, dihedral_deg,
        area, span)

    # --- vertical surfaces -------------------------------------------------
    cn_b_fin = cl_b_fin = 0.0
    if fin_area > 0 and fin_count > 0:
        cn_b_fin = ETA_FIN * (fin_area * max(fin_arm, 1e-4)) / (area * span) * a_fin
        # a winglet's centre of area sits about 45% of its height above the
        # wing; a centre fin the same above the body's own spine
        z_v = 0.45 * max(fin_height, 0.0)
        cl_b_fin = -ETA_FIN * (fin_area * z_v) / (area * span) * a_fin

    cn_beta = cn_b_sw + cn_b_fin
    cl_beta = cl_b_sw + cl_b_dih + cl_b_fin
    ratio = abs(cl_beta) / cn_beta if cn_beta > 1e-9 else float("inf")

    # --- elevon authority --------------------------------------------------
    # Area moment of the elevon strip about the CG, times the flap
    # effectiveness, gives the pitching moment available at full travel.
    tau = flap_effectiveness(elevon_chord_frac)
    fi, fo = sorted((float(elevon_inner), float(elevon_outer)))
    mask = (pf.f >= fi) & (pf.f <= fo)
    if not np.any(mask):
        mask = pf.f >= 0.5
    arm = np.abs(pf.x_ac[mask] - (pf.x_np - 0.0)) + 0.25 * pf.c[mask]
    s_elevon = 2.0 * float(np.sum(pf.c[mask] * elevon_chord_frac) * pf.dy)
    moment = 2.0 * float(np.sum(pf.c[mask] * elevon_chord_frac * arm) * pf.dy)
    cm_per_rad = -a_wing * tau * moment / (area * mac)
    elevon_authority = abs(cm_per_rad) * math.radians(ELEVON_MAX_DEG)
    trim_used = (abs(cm_trim_residual) / elevon_authority
                 if elevon_authority > 1e-9 else float("inf"))

    # --- readable assessment ----------------------------------------------
    if cn_beta <= 0:
        notes.append(
            "Directionally UNSTABLE: nothing is holding the nose into the "
            "airflow, so the model will hunt and swap ends. Add sweep or fin "
            "area.")
    elif cn_beta < CN_BETA_LOW:
        notes.append(
            f"Low directional stiffness (Cn_beta {cn_beta:.3f}/rad, and "
            f"{'sweep' if cn_b_sw >= cn_b_fin else 'the fins'} doing most "
            "of it). It will fly, but it will wander in gusts and "
            "you will be flying the heading rather than letting it track "
            "itself. More fin area or more sweep is the fix.")
    else:
        notes.append(
            f"Directional stiffness Cn_beta {cn_beta:.3f}/rad "
            f"(fins {cn_b_fin:.3f}, sweep {cn_b_sw:.3f}) - normal for a "
            "tailless wing, which runs an order of magnitude below a tailed "
            "model and relies on sweep for much of it.")
    if cl_beta >= 0:
        notes.append(
            "No dihedral effect: a sideslip will roll it FURTHER into the "
            "slip (spiral divergence). Add dihedral or sweep.")
    if math.isfinite(ratio):
        if ratio > RATIO_DUTCH_ROLL:
            notes.append(
                f"Roll stability strongly dominates yaw stiffness "
                f"({ratio:.1f}:1) - even for a wing that is Dutch-roll "
                "territory: expect a wallowing tail-wag after a gust that "
                "takes a few cycles to damp. More fin area, or less "
                "dihedral, settles it.")
        elif ratio < RATIO_SPIRAL:
            notes.append(
                f"Yaw stiffness dominates roll stability ({ratio:.1f}:1): it "
                "tracks beautifully but will slowly tighten a turn on its own "
                "(spiral mode). Keep a hand on it in a long bank.")
    if proverse > 0.02:
        notes.append(
            "PROVERSE yaw: roll input swings the nose INTO the turn, so it "
            "turns co-ordinated on elevons alone. This is what the bell "
            "spanload is for, and it is why the design carries no fin - "
            "expect it to feel unusually tidy in a bank.")
    elif proverse < -0.02:
        notes.append(
            "Adverse yaw on roll input, as on any conventional spanload: "
            "the nose initially swings OUT of the turn. Nothing is wrong - "
            "it is why elevon differential (more up than down) is worth "
            "dialling in on the transmitter.")
    if trim_used > 0.5:
        notes.append(
            f"Trim eats {trim_used * 100:.0f}% of the elevon travel, leaving "
            "little for the flare. Move the CG aft inside the band or accept a "
            "faster arrival.")
    return Handling(
        cn_beta=float(cn_beta), cl_beta=float(cl_beta),
        cn_beta_fin=float(cn_b_fin), cn_beta_sweep=float(cn_b_sw),
        cl_beta_sweep=float(cl_b_sw), cl_beta_dihedral=float(cl_b_dih),
        cl_beta_fin=float(cl_b_fin), ratio=float(ratio),
        elevon_authority=float(elevon_authority),
        elevon_trim_used=float(trim_used), proverse_yaw=float(proverse),
        notes=notes,
    )
