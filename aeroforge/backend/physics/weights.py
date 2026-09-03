"""Empty-weight estimation for foam / 3D-printed RC builds (spec section 5.6).

Structure is estimated from AREAL densities (kg per m^2 of surface planform /
wetted area) calibrated against typical published RC builds:
  - LW-PLA printed wing (single-wall shell + sparse internal structure at
    ~0.6 g/cc effective): ~1.9 kg/m^2 of wing planform
  - standard PLA (1.24 g/cc): ~1.9x heavier
  - foamboard (5 mm paper-faced foam + spars): ~0.9 kg/m^2
These are engineering estimates for a shelled, printable structure; the CAD
module uses matching wall thicknesses so the geometry and the weight model
describe the same build.

POWER-SYSTEM ALLOWANCE (replaces the old motor/ESC/battery sizing model)
-----------------------------------------------------------------------
AeroForge no longer specifies motors, ESCs, props or battery packs - the app
designs airframes, and the builder picks whatever power system they like. The
airframe physics still needs a realistic all-up mass though: stall speed and
wing loading are fiction without it.

So the motor(s) + ESC(s) + flight pack are carried as ONE documented lumped
allowance. Survey of typical electric RC models (park flyers through 2 m
cruisers) puts the complete power system at roughly 25-32% of all-up mass -
the pack alone is usually 18-25% and the motor/ESC/mount another 6-9%. The
allowance is written as

    pack = PACK_PER_MOTOR * n_motors + PACK_STRUCTURE_FRACTION * (structure
                                                                 + payload)

with a hard floor of PACK_MIN_KG (you cannot buy a usable motor + ESC + LiPo
for less than ~70 g at these sizes). The constants are chosen so that the
resulting pack fraction lands near 0.28 of AUW on typical designs:
a design whose structure + payload is m_s ends up at

    AUW = m_s + fixed + PACK_PER_MOTOR*n + PACK_STRUCTURE_FRACTION*(m_s)

and with PACK_STRUCTURE_FRACTION = 0.345 that is ~0.28 * AUW for the sport /
trainer sizes this app produces (see DECISIONS.md).

Because the allowance depends only on STRUCTURE and PAYLOAD - both of which
are pure functions of the geometry, not of the total mass - it is a trivially
stable fixed point of the mass-convergence loop in optimizer.evaluate_candidate
(it converges on the first pass and cannot oscillate).

Fixed-point iteration (spec 5.6): total weight -> required CL & drag ->
airframe/system masses -> new total weight, until converged.
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class MaterialDef:
    name: str
    label: str
    wing_areal: float      # kg/m^2 planform
    tail_areal: float      # kg/m^2 planform (vertical surfaces)
    fus_areal: float       # kg/m^2 wetted (kept: used for the body allowance)
    wall_mm: float         # print wall thickness used by CAD
    density_gcc: float


MATERIALS: dict[str, MaterialDef] = {
    "lw_pla": MaterialDef("lw_pla", "LW-PLA (~0.6 g/cc printed)", 1.9, 1.4, 1.3, 1.2, 0.6),
    "pla": MaterialDef("pla", "PLA (1.24 g/cc)", 3.6, 2.7, 2.5, 0.9, 1.24),
    "foam": MaterialDef("foam", "Foamboard", 0.9, 0.7, 0.8, 5.0, 0.035),
}

# fixed electronics masses [kg]
SERVO_MASS = 0.009
RECEIVER_MASS = 0.008
WIRING_BASE = 0.015

# lumped power-system allowance (motor(s) + ESC(s) + mounts + flight pack)
PACK_PER_MOTOR = 0.045          # kg of motor + ESC + mount per motor station
PACK_STRUCTURE_FRACTION = 0.345  # kg of pack per kg of structure + payload
PACK_MIN_KG = 0.070             # smallest credible motor+ESC+LiPo combination


# Bulk density of a hobby LiPo pack, measured: a 3S 2200 mAh pack is 137 g in a
# 105 x 34 x 24 mm envelope = 85.7 cm^3 -> 1.60 g/cc. Used to work out how deep
# the centre body has to be to actually swallow the flight pack.
PACK_BULK_DENSITY_KGM3 = 1600.0


def pack_volume_m3(pack_mass_kg: float) -> float:
    """Envelope volume a flight pack of this mass occupies [m^3]."""
    return max(pack_mass_kg, 0.0) / PACK_BULK_DENSITY_KGM3


def power_system_allowance(n_motors: int, structure_kg: float,
                           payload_kg: float) -> float:
    """Lumped motor + ESC + battery mass allowance [kg].

    See the module docstring: a converging allowance that lands the power
    system at roughly 25-32% of all-up mass for typical RC model sizes.
    """
    pack = (PACK_PER_MOTOR * max(int(n_motors), 1)
            + PACK_STRUCTURE_FRACTION * max(structure_kg + payload_kg, 0.0))
    return max(pack, PACK_MIN_KG)


# Hatch, bay formers and the pack retention on the centre body. A printed or
# foam access hatch with a latch at these sizes weighs 15-25 g; 18 g is used.
BODY_FITTINGS_KG = 0.018


def body_wetted_factor(tc_wing: float, tc_body: float) -> float:
    """How much more skin the deepened centre body needs than plain wing.

    Torenbeek's wetted-area estimate for a wing surface is
        S_wet / S = 1.977 + 0.52 (t/c)
    so a body region at t/c_body costs that ratio more skin - and therefore
    that ratio more structure mass - than wing at t/c_wing over the same
    planform area.
    """
    return (1.977 + 0.52 * max(tc_body, 0.0)) / (1.977 + 0.52 * max(tc_wing, 0.0))


@dataclass
class MassBreakdown:
    structure_wing: float
    structure_fins: float
    structure_body: float
    power_system: float
    servos: float
    receiver_wiring: float
    payload: float
    total: float
    components: dict[str, float] = field(default_factory=dict)


def estimate_mass(
    *, material: MaterialDef, wing_area: float, body_area: float,
    body_wet_factor: float, fin_area: float,
    n_motors: int, n_servos: int, payload_kg: float,
) -> MassBreakdown:
    """Mass of a blended flying wing.

    `wing_area` is the OUTBOARD planform area, `body_area` the blended centre
    body's planform area and `body_wet_factor` the extra-skin ratio from
    :func:`body_wetted_factor`. There is no fuselage term: on this airframe the
    body is wing, built the same way out of the same material.
    """
    m_wing = material.wing_areal * max(wing_area, 0.0)
    m_body = (material.wing_areal * max(body_area, 0.0) * max(body_wet_factor, 1.0)
              + BODY_FITTINGS_KG)
    m_fins = material.tail_areal * max(fin_area, 0.0)
    m_structure = m_wing + m_body + m_fins
    m_pack = power_system_allowance(n_motors, m_structure, payload_kg)
    m_servo = n_servos * SERVO_MASS
    m_rx = RECEIVER_MASS + WIRING_BASE
    total = m_structure + m_pack + m_servo + m_rx + payload_kg
    return MassBreakdown(
        structure_wing=m_wing, structure_fins=m_fins, structure_body=m_body,
        power_system=m_pack, servos=m_servo, receiver_wiring=m_rx,
        payload=payload_kg, total=total,
        components={
            "wing panels": m_wing,
            "centre body": m_body,
            "vertical surfaces": m_fins,
            "power system (motor+ESC+pack)": m_pack,
            "servos": m_servo, "rx+wiring": m_rx, "payload": payload_kg,
        },
    )


# =============================================================================
# v2 CONVENTIONAL airframe mass model (V2_PLAN.md). Everything below is
# ADDITIVE: the flying-wing mass math above is untouched, and the flying-wing
# path never calls into this section.
#
# Same material/areal approach as above, but with CONVENTIONAL areal densities
# calibrated against published 3D-printed conventionals rather than blended
# wings (RESEARCH_CONVENTIONAL.md section 7):
#   - Eclipson Model A (1000 mm trainer): 220 g LW-PLA total print for a
#     16 dm^2 wing + 760 mm fuselage + tail; AUW 490 g LW-PLA / 730 g PLA.
#   - 3DLabPrint Piper J-3 Cub (1068 mm): ~300 g LW-PLA print, ~600 g AUW.
#   - Fuselage budget 80-140 g LW-PLA at 1000-1100 mm span (35-45% of print
#     mass on those designs); tail group 30-60 g incl. pushrods.
# A conventional wing is a thin plain-section panel, not a 2x-deepened blended
# body carrying the whole aircraft's volume, so its areal density is well
# below the flying-wing figure; the PLA column is the LW-PLA one x ~1.9
# (ColorFabb foamed density 0.54 vs 1.24 g/cc, dossier section 7).
# =============================================================================

CONV_WING_AREAL: dict[str, float] = {   # kg per m^2 of wing planform
    "lw_pla": 0.95, "pla": 1.80, "foam": 0.75,
}
CONV_TAIL_AREAL: dict[str, float] = {   # kg per m^2 of stab + fin planform
    "lw_pla": 0.80, "pla": 1.50, "foam": 0.60,
}
CONV_FUS_AREAL: dict[str, float] = {    # kg per m^2 of fuselage WETTED area
    "lw_pla": 0.60, "pla": 1.15, "foam": 0.55,
}
# firewall, hatch, wing bolts/dowels and tail-post hardware on a printed
# conventional: engineering estimate, same class as BODY_FITTINGS_KG above
CONV_FITTINGS_KG = 0.025
# elevator + rudder pushrods/snakes to the tail (~30 g for 2 servos +
# pushrods per the dossier; the servos are counted separately below)
CONV_PUSHRODS_KG = 0.012


def estimate_mass_conventional(
    *, material: MaterialDef, wing_area: float, tail_area_h: float,
    tail_area_v: float, fus_wetted_area: float,
    n_motors: int, n_servos: int, payload_kg: float,
) -> MassBreakdown:
    """Mass of a conventional (wing + fuselage + tail) airframe.

    Reuses :class:`MassBreakdown` with the obvious mapping: the fuselage rides
    in `structure_body`, the stab + fin group in `structure_fins`. The lumped
    power-system allowance is the SAME documented ~28%-of-AUW model as the
    flying wing (module docstring) - the departure list keeps it for v2.
    """
    m_wing = CONV_WING_AREAL[material.name] * max(wing_area, 0.0)
    m_fus = (CONV_FUS_AREAL[material.name] * max(fus_wetted_area, 0.0)
             + CONV_FITTINGS_KG)
    m_tail = (CONV_TAIL_AREAL[material.name]
              * max(tail_area_h + tail_area_v, 0.0) + CONV_PUSHRODS_KG)
    m_structure = m_wing + m_fus + m_tail
    m_pack = power_system_allowance(n_motors, m_structure, payload_kg)
    m_servo = n_servos * SERVO_MASS
    m_rx = RECEIVER_MASS + WIRING_BASE
    total = m_structure + m_pack + m_servo + m_rx + payload_kg
    return MassBreakdown(
        structure_wing=m_wing, structure_fins=m_tail, structure_body=m_fus,
        power_system=m_pack, servos=m_servo, receiver_wiring=m_rx,
        payload=payload_kg, total=total,
        components={
            "wing panels": m_wing,
            "fuselage": m_fus,
            "tail surfaces": m_tail,
            "power system (motor+ESC+pack)": m_pack,
            "servos": m_servo, "rx+wiring": m_rx, "payload": payload_kg,
        },
    )
