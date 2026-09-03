"""Whole-aircraft aerodynamics (spec section 5.3).

Equations implemented (each cited where used):
  Lift:        L = 0.5 rho V^2 S CL
  Cruise trim: L = W  ->  CL_req = 2 W / (rho V^2 S)
  Drag polar:  CD = CD0 + CL^2 / (pi AR e)
  AR = b^2 / S
  Stall:       V_stall = sqrt(2 W / (rho S CL_max))
  Drag/thrust: D = 0.5 rho V^2 S CD ;  T_cruise = D
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .airfoils import Airfoil
from .atmosphere import reynolds


def aspect_ratio(span: float, area: float) -> float:
    """AR = b^2 / S."""
    return span**2 / area


def required_cl(weight_n: float, rho: float, v: float, area: float) -> float:
    """Cruise weight balance L = W: CL = 2W/(rho V^2 S)."""
    return 2.0 * weight_n / (rho * v**2 * area)


def oswald_efficiency(ar: float, mult: float = 1.0) -> float:
    """Raymer straight-wing estimate e = 1.78(1 - 0.045 AR^0.68) - 0.64,
    scaled by a config multiplier (flying wings lower), clamped to the
    spec's 0.6..0.85 practical band."""
    e = 1.78 * (1.0 - 0.045 * ar**0.68) - 0.64
    return max(0.60, min(0.85, e * mult))


def flat_plate_cf(re_l: float) -> float:
    """Fully turbulent flat-plate skin friction Cf = 0.074 / Re^0.2 (Schlichting)."""
    return 0.074 / max(re_l, 1e4) ** 0.2


def airfoil_form_factor(tc: float) -> float:
    """Airfoil/streamline-body form factor FF = 1 + 2.7 (t/c) + 100 (t/c)^4
    (Hoerner). Used as a RATIO here, to price the extra profile drag of the
    deepened centre section against the wing's own section."""
    return 1.0 + 2.7 * tc + 100.0 * tc**4


def blended_body_cd0(cd_section: float, tc_wing: float, tc_body_mean: float,
                     s_body: float, s_ref: float) -> float:
    """Extra parasite drag of the blended centre body.

    There is no fuselage on a flying wing: the centre body is the same surface
    as the wing, simply deeper and longer. So its drag is not a separate
    fuselage build-up but the wing's own section drag re-priced at the body's
    much larger thickness ratio, over the body's share of the planform:

        dCD0_body = Cd_section * (FF(t/c_body)/FF(t/c_wing) - 1) * S_body/S_ref

    With a 9% wing section and a 2.4x deepened body (t/c ~ 0.22) the ratio is
    ~1.4, i.e. the body region costs ~40% more profile drag than plain wing -
    the honest price of carrying the pack inside the aerofoil.
    """
    if s_body <= 0.0 or s_ref <= 0.0:
        return 0.0
    ratio = airfoil_form_factor(tc_body_mean) / airfoil_form_factor(tc_wing)
    return cd_section * max(ratio - 1.0, 0.0) * (s_body / s_ref)


@dataclass
class DragBreakdown:
    cd0_wing: float
    cd0_fuselage: float      # blended centre body (kept under the v1 key)
    cd0_tail: float          # vertical surfaces
    cd0_interference: float
    cd0: float
    cdi: float
    cd: float


def drag_polar(cl: float, ar: float, e: float, cd0: float) -> float:
    """CD = CD0 + CL^2/(pi AR e)  (parasite + induced)."""
    return cd0 + cl**2 / (math.pi * ar * e)


# A tailless wing has only the vertical-surface roots as junctions - there is
# no wing/fuselage fillet, no tailplane root and no boom joint - so the classic
# 8-10% interference allowance of a full build-up is reduced to 6%.
INTERFERENCE_FRACTION = 0.06


def build_drag(cl_cruise: float, airfoil: Airfoil, re_mac: float,
               ar: float, e: float,
               s_ref: float, fin_area_total: float,
               tc_wing: float, tc_body_mean: float, s_body: float,
               fin_chord: float,
               rho: float, v: float, mu: float) -> DragBreakdown:
    """CD0 build-up for a blended flying wing (spec 5.3, adapted).

    Terms: the wing's own section profile drag at the cruise Cl, the centre
    body's extra thickness drag, the vertical surfaces by the wetted-area
    method, and an interference allowance.
    """
    cd_section = airfoil.cd_at_cl(cl_cruise, re_mac)
    cd0_wing = cd_section
    cd0_body = blended_body_cd0(cd_section, tc_wing, tc_body_mean, s_body, s_ref)
    # vertical surfaces: thin symmetric section, both sides wetted,
    # CD0_fin = Cf(Re_fin) * 2.05 * S_v/S_ref  (wetted-area method, Raymer)
    # x1.3 for the thickness form factor of an 8% symmetric section.
    cd0_fin = 0.0
    if fin_area_total > 0.0 and s_ref > 0.0:
        re_fin = reynolds(rho, v, max(fin_chord, 0.02), mu)
        cd0_fin = (flat_plate_cf(max(re_fin, 3e4)) * 2.05
                   * (fin_area_total / s_ref) * 1.3)
    subtotal = cd0_wing + cd0_body + cd0_fin
    cd0_int = INTERFERENCE_FRACTION * subtotal
    cd0 = subtotal + cd0_int
    cdi = cl_cruise**2 / (math.pi * ar * e)     # CDi = CL^2/(pi AR e)
    return DragBreakdown(cd0_wing, cd0_body, cd0_fin, cd0_int, cd0, cdi,
                         cd0 + cdi)


def dynamic_pressure(rho: float, v: float) -> float:
    """q = 0.5 rho V^2."""
    return 0.5 * rho * v**2


def drag_force(rho: float, v: float, s: float, cd: float) -> float:
    """D = 0.5 rho V^2 S CD; in steady cruise required thrust T = D."""
    return dynamic_pressure(rho, v) * s * cd


def cl_max_3d(cl_max_2d: float, sweep_le_deg: float = 0.0,
              washout_deg: float = 0.0) -> float:
    """Finite-wing maximum lift.

    CL_max = 0.9 Cl_max,2D * cos(Lambda_c/4) * (1 - 0.02 * washout_deg)

    - 0.9 is Raymer's finite-wing factor for a moderate-aspect-ratio wing.
    - cos(Lambda_c/4) is the standard sweep correction: only the component of
      the flow normal to the quarter-chord line does the lifting.
    - washout deliberately keeps the tips below the root's angle of attack, so
      the outboard panels are not yet at Cl_max when the root stalls; ~2% of
      CL_max is lost per degree of twist at these aspect ratios. That is the
      price a bell-spanload wing pays for its docile tip behaviour.
    """
    # quarter-chord sweep is a few degrees less than LE sweep on a tapered
    # wing; using the LE value is the conservative side of the correction
    k_sweep = math.cos(math.radians(max(0.0, min(sweep_le_deg, 45.0))))
    k_twist = max(0.55, 1.0 - 0.02 * max(washout_deg, 0.0))
    return 0.9 * cl_max_2d * k_sweep * k_twist


def stall_speed(weight_n: float, rho: float, s: float, clmax_3d: float) -> float:
    """V_stall = sqrt(2 W / (rho S CL_max))."""
    return math.sqrt(2.0 * weight_n / (rho * s * clmax_3d))


def lift_to_drag(cl: float, cd: float) -> float:
    return cl / cd if cd > 0 else 0.0
