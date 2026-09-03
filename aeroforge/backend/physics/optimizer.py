"""Constrained flying-wing optimizer (SPEC_FLYING_WING.md sections 4, 6, 8).

AeroForge designs tailless wings only, so the search space is the planform
itself: SPAN, ASPECT RATIO, LEADING-EDGE SWEEP and TAPER, each bounded by the
chosen planform family's measured band. There is no tail to size, no wing
position to solve and no fuselage to proportion - on this airframe the centre
BODY is simply the inboard part of the same surface, deeper and longer, and it
is sized to actually swallow the flight pack.

Objective: minimise SPECIFIC DRAG at cruise, D/W = 1/(L/D) - the power-free
honest efficiency measure (see DECISIONS.md) - plus soft terms that keep the
design inside the flyable bands and looking like a real flying wing.

Hard constraints (checked, reported and penalised):
  - fits the user's box in span / length / height
  - L = W at cruise (enforced by construction: CL_req from the lift equation)
  - V_cruise >= stall_factor * V_stall (or the user's stall target)
  - static margin inside the TAILLESS band 0.03-0.15 with dCm/dalpha < 0
  - PROPORTION REALISM: the wing root chord is a real fraction of the span
    (0.30-0.45 for swept/bwb) and the centre body is deep enough to hold the
    pack - this is what stops the optimizer producing a glider with a pod
    glued to it
  - vertical surfaces 2-9% of wing area, no taller than 60% of the chord they
    stand on (zero for a bell-spanload wing)
  - wing loading inside the mission band
  - Reynolds number high enough for the section
  - the pack can be placed inside the centre body's equipment bay

Search strategy: structured sweep over (span, AR, sweep, taper) with a full
physics evaluation at every point, then a Nelder-Mead polish of the penalised
objective (scipy). See DECISIONS.md for why this instead of end-to-end CasADi.

OPTIONAL VARIANT KNOBS (all default to no-ops, so a plain call is unaffected) -
used by physics.variants to generate the five design characters:
    planform        str     planform family override ("swept"/"bwb"/...)
    mission         str     mission override
    vstab           str     vertical-surface arrangement override
    ar_band_shift   float   added to both ends of the aspect-ratio sweep band
                            (or a (lo, hi) pair to move the ends separately)
    ar_target       float   pin the aspect ratio
    taper_override  float   pin the taper ratio (clipped to the planform band)
    sweep_override  float   pin the LE sweep (clipped to the planform band)
    sweep_shift_deg float   added to the middle of the planform's sweep band
    washout_shift_deg float added to both ends of the washout band
    wl_band_scale   float   multiplies the mission's wing-loading band
                            (or a (lo, hi) pair, to WIDEN rather than shift)
    wl_pen_weight   float   multiplies the wing-loading band-CENTERING cost
    sd_weight       float   multiplies the specific-drag (1/(L/D)) cost term
    span_pref_frac  float   fraction of the box width the envelope-utilisation
                            preference aims at (default 0.88)
    v_stall_goal    float   SOFT low-stall-speed goal [m/s]
    body_chord_scale float  centre-body chord scale (1.10-1.45)
    body_depth_mult float   multiplies the required centre-body depth
    body_width_mult float   multiplies the centre-body half-width
    canopy          bool    force a faired hatch on the spine
    airfoil_pref    str|list preferred reflexed section (or a candidate list -
                            the search then runs once per candidate)
    seeds           list    extra {span, ar, sweep, taper} starting points
"""
from __future__ import annotations

import math
import uuid

import numpy as np
from scipy import optimize as sciopt

from . import aero as A
from .airfoils import LIBRARY, Airfoil, fin_airfoil, pick_airfoil
from .atmosphere import isa, reynolds
from .config_defs import (
    MISSIONS, PLANFORMS, SM_BAND, VSTABS, MissionDef, PlanformDef, resolve_vstab,
)
from .handling import handling_qualities
from .stability import (
    build_planform, chord_at, flying_wing_fin, lift_slope_3d,
    solve_flying_wing, x_le_at,
)
from .weights import MATERIALS, estimate_mass, body_wetted_factor, pack_volume_m3

G = 9.80665
RE_MIN_GOOD = 1.5e5    # below this the airfoil polar quality drops fast
# Hard floor: below Re ~ 9e4 the reflexed sections in airfoils.LIBRARY lose
# their positive Cm0 (the aft camber sits inside the laminar separation bubble
# at that scale), so a tailless wing built there cannot trim at all.
RE_MIN_HARD = 9.0e4

N_STRIPS = 80          # spanwise strips of the half-wing


# ---------------------------------------------------------------------------
# Centre-body proportions
#
# The blend half-width is measured off real airframes as a fraction of the
# SEMI-SPAN: the Skywalker X5's centre section is visibly deepened over roughly
# the inner 110 mm of its 640 mm semi-span (0.17); a Ritewing Drak-class BWB
# carries its body over a quarter of the semi-span; a plank or a Horten keeps a
# narrow spine so the outboard panels stay pure wing.
# ---------------------------------------------------------------------------
BODY_HALF_WIDTH_FRAC = {"swept": 0.17, "bwb": 0.26, "plank": 0.13, "bell": 0.12}
# centre chord / wing root chord: how far the leading-edge root extension
# reaches forward. 1.10 is the invariant floor (spec 8.6); a BWB needs more.
BODY_CHORD_SCALE = {"swept": 1.22, "bwb": 1.38, "plank": 1.12, "bell": 1.14}
# how bluntly the centre leading edge is faired (0 = knife, 1 = full round)
BODY_NOSE_ROUND = {"swept": 0.60, "bwb": 0.85, "plank": 0.50, "bell": 0.45}
# share of the body's extra depth that goes UP rather than down: a moulded wing
# has a nearly flat keel and all the volume in the crown
BODY_CROWN_FRAC = {"swept": 0.65, "bwb": 0.72, "plank": 0.62, "bell": 0.60}
# usable equipment-bay length as a fraction of the centre chord (the rest is
# the LE fairing and the TE wedge, neither of which can hold anything)
BAY_LENGTH_FRAC = 0.60
BAY_START_FRAC = 0.04
# usable bay width as a fraction of the full body width (walls + the section
# rolling off toward the blend)
BAY_WIDTH_FRAC = 0.62
# spec 8.6: the body must be able to hold the pack at all
BODY_MIN_DEPTH_M = 0.028

# ---------------------------------------------------------------------------
# Motor mount
#
# AeroForge does not size power systems, but a wing has to have somewhere to
# BOLT the motor, and that hole pattern is a real, standardised dimension the
# builder has to print. RC outrunners come with a square 4-hole pattern on the
# bell mount; the three that cover essentially the whole model range are:
#
#     16 x 16 mm   2204-2306 class   (small sport wings, ~<700 g)
#     19 x 19 mm   2212-2814 class   (the common FPV-wing size)
#     25 x 25 mm   28xx-35xx class   (big/heavy wings)
#
# all on M3 screws. For a square pattern of side s the holes sit at
# (+-s/2, +-s/2), so the bolt-circle radius is R = s/sqrt(2) x ... = 0.7071 s.
# Defaults are chosen from all-up mass; every number is user-overridable,
# because the builder knows which motor is actually going on it.
# ---------------------------------------------------------------------------
MOUNT_PATTERNS_MM = ((0.70, 16.0), (1.60, 19.0), (1e9, 25.0))   # (mass<=, side)
MOUNT_SCREW_CLEARANCE_MM = 3.2      # M3 clearance
MOUNT_PLATE_MIN_MM = 4.0            # printed bulkhead thickness
SQRT2 = math.sqrt(2.0)


def _motor_mount_spec(inp: dict, mass_kg: float, motors: list[dict],
                      wall_mm: float) -> dict:
    """The bolt pattern the CAD drills and the spec panel prints."""
    side = next(s for m, s in MOUNT_PATTERNS_MM if mass_kg <= m)
    side = float(inp.get("mount_pattern_mm") or side)
    n_screws = int(inp.get("mount_screws") or 4)
    screw_d = float(inp.get("mount_screw_d_mm") or MOUNT_SCREW_CLEARANCE_MM)
    # a square pattern's bolt circle passes through its corners
    radius = float(inp.get("mount_bolt_circle_mm") or side / SQRT2)
    m0 = motors[0] if motors else {"x": 0.0, "y": 0.0, "z": 0.0,
                                   "type": "pusher"}
    plate = max(MOUNT_PLATE_MIN_MM, 2.2 * wall_mm, 0.9 * screw_d)
    return {
        # where the motor bolts on: the mounting FACE sits at the body's
        # trailing edge (pusher) or its nose (tractor), and the boss grows
        # INTO the body, so the mount never lengthens the aircraft
        "type": str(m0.get("type", "pusher")),
        "x_m": float(m0.get("x", 0.0)),
        "y_m": float(m0.get("y", 0.0)),
        "z_m": float(m0.get("z", 0.0)),
        "n_screws": n_screws,
        "screw_hole_d_mm": screw_d,
        "bolt_circle_radius_mm": radius,
        "pattern": "square" if n_screws == 4 else "circular",
        "pattern_side_mm": float(radius * SQRT2),
        # clearance for the shaft/prop boss (pusher) or the wire bundle
        "shaft_hole_d_mm": float(max(6.0, 0.55 * radius * 2.0)),
        "plate_thickness_mm": float(plate),
        # the plate has to reach past the outermost screw with real material
        "plate_radius_mm": float(radius + max(4.0, 1.6 * screw_d)),
    }

# Elevon layout per planform, as fractions of the semi-span and local chord.
# Swept wings hinge the outboard 60% of the span at ~25% chord (X5/AR Wing);
# a plank needs more chord because the reflex is doing the trimming and the
# elevon has to move against it; a bell wing keeps the elevons well outboard so
# they work in the lightly loaded tip region without upsetting the spanload.
ELEVONS = {
    "swept": {"inner_frac": 0.35, "outer_frac": 0.95, "chord_frac": 0.25},
    "bwb": {"inner_frac": 0.40, "outer_frac": 0.95, "chord_frac": 0.24},
    "plank": {"inner_frac": 0.30, "outer_frac": 0.95, "chord_frac": 0.28},
    "bell": {"inner_frac": 0.45, "outer_frac": 0.97, "chord_frac": 0.22},
}

# Clearance between an inboard fin and the start of the elevon, as a fraction
# of the semi-span. Enough that the fin's root chord is clear of the hinge line
# and a servo horn has somewhere to live.
FIN_ELEVON_CLEARANCE = 0.04


def elevon_layout(planform: str, fin) -> dict:
    """Elevon span/chord for this planform, moved clear of any inboard fin.

    A `twin_fin` wing carries its fins partway out the semi-span, and the
    default elevon band starts INBOARD of them - so the fin stands on top of
    the moving surface. In CAD the fin is fused into the wing and the elevon is
    then cut out from under it, which hands back a 115 mm tall, 236 cm^3
    "control surface" with a fin on it. It would deflect the fin with the
    elevon, which is not a thing any aircraft does.

    Real practice is unambiguous: on a Skywalker X5 the two fins sit inboard
    and the elevons run OUTBOARD of them, so the elevon starts outboard of the
    fin station plus a clearance. Reported here rather than clamped in the CAD,
    because the elevon's span is what sets its roll authority and the handling
    check has to see the same surface the builder gets.
    """
    el = dict(ELEVONS[planform])
    y_fin = float(getattr(fin, "y_frac", 0.0) or 0.0)
    if getattr(fin, "count", 0) and 0.0 < y_fin < 1.0 \
            and el["inner_frac"] < y_fin < el["outer_frac"]:
        inner = min(y_fin + FIN_ELEVON_CLEARANCE, el["outer_frac"] - 0.20)
        el["inner_frac"] = max(el["inner_frac"], inner)
    return el

# A plank has no sweep, so it has no sweep-derived roll/yaw coupling to keep it
# upright; a couple of degrees of dihedral is what real planks use instead.
# Every swept layout gets its roll stability from the sweep itself.
DIHEDRAL_DEG = {"swept": 0.0, "bwb": 0.0, "plank": 2.0, "bell": 0.0}


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------

def _band(inp: dict, key: str, base: tuple[float, float],
          lo_limit: float, hi_limit: float) -> tuple[float, float]:
    """Apply a scalar-or-pair shift knob to a band and clamp it."""
    shift = inp.get(key) or 0.0
    lo_s, hi_s = (shift if isinstance(shift, (tuple, list)) else (shift, shift))
    lo = max(lo_limit, base[0] + float(lo_s))
    hi = min(hi_limit, max(lo + 1e-6, base[1] + float(hi_s)))
    return lo, hi


def _wl_band(mission: MissionDef, inp: dict) -> tuple[float, float]:
    """Wing-loading band for this run, optionally scaled by a variant knob.
    A single number scales both ends; a (lo, hi) pair scales them separately
    (so a variant can be allowed a WIDER legal band, not just a shifted one)."""
    s = inp.get("wl_band_scale") or 1.0
    s_lo, s_hi = s if isinstance(s, (tuple, list)) else (s, s)
    return (mission.wl_band[0] * float(s_lo), mission.wl_band[1] * float(s_hi))


def _band_center_pen(val: float, lo: float, hi: float) -> float:
    mid, hw = 0.5 * (lo + hi), 0.5 * (hi - lo)
    return ((val - mid) / hw) ** 2 if hw > 0 else 0.0


def _reconcile_proportions(span: float, ar: float, taper: float,
                           fb: float, chord_scale: float,
                           pf_def: PlanformDef,
                           ar_band: tuple[float, float]) -> tuple[float, float, float]:
    """Make (AR, taper) produce a root chord that is a REAL fraction of the span.

    This is the fix for the "glider with a pod glued on" failure mode. For a
    linearly tapered wing the root chord is fully determined by the span, the
    aspect ratio and the taper:

        S = b^2/AR  and  S = b (c_r + c_t)/2   ->   c_r/b = 2/(AR (1 + lam))

    (with a small correction for the area the centre-body blend adds). So a
    root chord of 0.30-0.45 x span - the proportion every real sport/FPV wing
    has - is not a free choice on top of AR and taper, it IS a constraint
    linking them. Aspect ratio is moved first because the planform's AR band is
    the wider of the two; taper takes up whatever is left.

    Returns (ar, taper, c_root) with c_root measured on the actual blended
    planform.
    """
    f_lo, f_hi = pf_def.root_chord_frac_band
    t_lo, t_hi = pf_def.taper_band

    def c_root_of(ar_v: float, taper_v: float) -> float:
        # build with unit root chord: area scales linearly with c_root
        unit = build_planform(span=span, c_root_wing=1.0, taper=taper_v,
                              sweep_le_deg=0.0, body_half_width_frac=fb,
                              body_chord_scale=chord_scale, n_strips=40)
        return (span**2 / ar_v) / unit.area

    for _ in range(8):
        frac = c_root_of(ar, taper) / span
        if f_lo - 1e-6 <= frac <= f_hi + 1e-6:
            break
        target = float(np.clip(frac, f_lo, f_hi))
        # c_root ~ 1/AR at fixed taper
        ar = float(np.clip(ar * frac / target, ar_band[0], ar_band[1]))
        frac = c_root_of(ar, taper) / span
        if f_lo - 1e-6 <= frac <= f_hi + 1e-6:
            break
        # c_root ~ 1/(1 + lam) at fixed AR
        target = float(np.clip(frac, f_lo, f_hi))
        taper = float(np.clip((1.0 + taper) * frac / target - 1.0, t_lo, t_hi))
    return ar, taper, c_root_of(ar, taper)


def _motor_layout(n_motors: int, pf, c_centre: float, z_body: float,
                  planform: str) -> list[dict]:
    """Motor stations.

    A SWEPT tailless wing is a pusher: the prop lives at the centre trailing
    edge, behind the body, out of the way of a hand launch (Zagi / Ritewing /
    X5 and every wing derived from them).

    A PLANK is a TRACTOR. That is not a style choice - an unswept wing's
    neutral point sits barely aft of its own quarter-chord, so the CG has to be
    far forward of the structure, and hanging the heaviest single item on the
    trailing edge is exactly the wrong place for it. Electric plank conversions
    (PW-51 class, Hepperle's plank designs) put the motor on the nose for that
    reason, and it removes most of the nose ballast the layout would otherwise
    need.
    """
    tractor = (planform == "plank")
    half = 0.5 * pf.span
    if tractor:
        motors = [{"x": 0.005, "y": 0.0, "z": float(z_body), "type": "tractor"}]
    else:
        motors = [{"x": float(c_centre), "y": 0.0, "z": float(z_body),
                   "type": "pusher"}]
    for i in range(1, max(int(n_motors), 1)):
        side = 1 if i % 2 == 1 else -1
        f = 0.30 + 0.18 * ((i - 1) // 2)
        x = (x_le_at(pf, f) if tractor
             else x_le_at(pf, f) + chord_at(pf, f))
        motors.append({"x": float(x), "y": float(side * f * half), "z": 0.0,
                       "type": "tractor" if tractor else "pusher"})
    return motors[:max(int(n_motors), 1)]


# ---------------------------------------------------------------------------
# Single full-physics evaluation of one candidate planform
# ---------------------------------------------------------------------------

def evaluate_candidate(x: dict, inp: dict) -> dict:
    """Evaluate one candidate {span, ar, sweep, taper} through the whole
    pipeline. Returns the design dict of SPEC_FLYING_WING.md section 4 with
    'cost', 'constraints' and 'feasible' filled in."""
    pf_def: PlanformDef = PLANFORMS[inp.get("planform") or "swept"]
    mission: MissionDef = MISSIONS[inp.get("mission") or "sport"]
    material = MATERIALS[inp["material"]]
    atm = isa(0.0)
    rho, mu = atm.density_kgm3, atm.viscosity_Pas
    wall_m = material.wall_mm / 1000.0

    # ---- search variables, clipped to the planform's measured bands --------
    span = float(x["span"])
    ar_band = _band(inp, "ar_band_shift", pf_def.ar_band, 3.0, 14.0)
    ar = float(np.clip(x["ar"], *ar_band))
    sweep = float(np.clip(x.get("sweep", 0.5 * sum(pf_def.sweep_band)),
                          *pf_def.sweep_band))
    taper = float(np.clip(x.get("taper", 0.5 * sum(pf_def.taper_band)),
                          *pf_def.taper_band))

    # ---- centre-body proportions -------------------------------------------
    fb = float(np.clip(BODY_HALF_WIDTH_FRAC[pf_def.name]
                       * float(inp.get("body_width_mult") or 1.0), 0.08, 0.34))
    chord_scale = float(np.clip(
        inp.get("body_chord_scale") or BODY_CHORD_SCALE[pf_def.name], 1.10, 1.45))

    # PROPORTION REALISM: tie AR/taper to a real root-chord fraction (spec 8.6)
    ar, taper, c_root = _reconcile_proportions(
        span, ar, taper, fb, chord_scale, pf_def, ar_band)

    pf = build_planform(span=span, c_root_wing=c_root, taper=taper,
                        sweep_le_deg=sweep, body_half_width_frac=fb,
                        body_chord_scale=chord_scale, n_strips=N_STRIPS)
    area = pf.area                       # integrated, blend included
    ar_actual = span**2 / area           # AR = b^2/S
    c_centre = pf.c_centre
    c_tip = pf.c_tip
    root_chord_frac = c_root / span

    # body / outboard area split (the body region is where t/c is scaled up)
    body_mask = pf.f < fb
    s_body = 2.0 * float(np.sum(pf.c[body_mask])) * pf.dy
    s_outboard = max(area - s_body, 0.0)
    half_width_m = fb * 0.5 * span
    bay_len = BAY_LENGTH_FRAC * c_centre
    bay_w = BAY_WIDTH_FRAC * 2.0 * half_width_m

    # ---- section, mission targets, vertical surfaces ------------------------
    af_choice = inp.get("airfoil_override") or inp.get("airfoil_pref")
    if isinstance(af_choice, (list, tuple)):
        af_choice = af_choice[0] if af_choice else None
    airfoil = pick_airfoil(pf_def.name, mission.name, af_choice)
    tc_wing = airfoil.defn.thickness
    fin_af = fin_airfoil()

    v = float(inp["v_cruise"])
    sm_target = float(np.clip(inp.get("sm_override") or mission.static_margin,
                              *SM_BAND))
    wl_band = _wl_band(mission, inp)
    washout_band = _band(inp, "washout_shift_deg", pf_def.washout_band, 0.0, 15.0)
    vs, vstab_notes = resolve_vstab(pf_def, inp.get("vstab"))
    e_osw = A.oswald_efficiency(ar_actual, pf_def.oswald_mult)
    n_servos = 2                         # two elevons, one servo each
    n_motors = max(int(inp.get("n_motors") or 1), 1)
    payload = float(inp.get("payload_kg") or 0.0)

    # ---- coupled mass / aero / stability / body-depth solve -----------------
    def run_pipeline(sm: float, mass0: float, ballast: float = 0.0):
        from .weights import PACK_PER_MOTOR
        mass = mass0
        depth_scale = float(pf_def.body_depth_band[0])
        washout = 0.5 * sum(washout_band)
        stab = fin = drag = mass_bd = None
        cl_req = re_mac = v_stall = clmax3 = 0.0
        tc_body_mean = tc_wing
        for _ in range(30):
            W = mass * G
            cl_req = A.required_cl(W, rho, v, area)        # L = W at cruise
            re_mac = reynolds(rho, v, pf.mac, mu)          # Re = rho V c / mu
            clmax3 = A.cl_max_3d(airfoil.cl_max(re_mac), sweep, washout)
            v_stall = A.stall_speed(W, rho, area, clmax3)

            stab = solve_flying_wing(
                pf=pf, airfoil=airfoil, re_mac=re_mac, ar=ar_actual,
                e_oswald=e_osw, sm_target=sm, cl_cruise=cl_req,
                washout_band=washout_band,
                bell_spanload=pf_def.bell_spanload, sm_band=SM_BAND,
                sm_hard=inp.get("sm_override") is not None)
            washout = stab.washout_deg

            fin = flying_wing_fin(pf=pf, area=area, x_cg=stab.x_cg,
                                  vstab_type=vs)

            # centre-body depth: sized so the bay can ACTUALLY hold the pack
            # (spec 8.6). Volume of the flight pack from its bulk density, plus
            # the payload, spread over the usable bay footprint, plus two skins.
            m_at_motors = min(PACK_PER_MOTOR * n_motors,
                              0.8 * (mass_bd.power_system if mass_bd else 0.1))
            m_batt = max((mass_bd.power_system if mass_bd else 0.12)
                         - m_at_motors, 1e-6)
            v_needed = pack_volume_m3(m_batt) + payload / 700.0
            d_needed = (v_needed / max(bay_len * bay_w, 1e-6) + 2.0 * wall_m)
            d_needed = max(d_needed, BODY_MIN_DEPTH_M) * float(
                inp.get("body_depth_mult") or 1.0)
            # depth_scale multiplies the WING's thickness at the root
            depth_scale = float(np.clip(d_needed / max(tc_wing * c_root, 1e-6),
                                        *pf_def.body_depth_band))
            # area-weighted mean t/c over the blended body region
            wb = pf.c[body_mask]
            tc_body_mean = float(
                tc_wing * np.sum(wb * (1.0 + (depth_scale - 1.0) * pf.w[body_mask]))
                / max(np.sum(wb), 1e-9))

            drag = A.build_drag(
                cl_req, airfoil, re_mac, ar_actual, e_osw, area,
                fin.area_total, tc_wing, tc_body_mean, s_body,
                fin.chord_mean, rho, v, mu)

            mass_bd = estimate_mass(
                material=material, wing_area=s_outboard, body_area=s_body,
                body_wet_factor=body_wetted_factor(tc_wing, tc_body_mean),
                fin_area=fin.area_total, n_motors=n_motors,
                n_servos=n_servos, payload_kg=payload)
            new_mass = mass_bd.total + ballast
            if abs(new_mass - mass) < 1e-4:
                mass = new_mass
                break
            mass = 0.5 * mass + 0.5 * new_mass
        # final consistency pass so L = W holds to numerical precision
        W = mass * G
        cl_req = A.required_cl(W, rho, v, area)
        v_stall = A.stall_speed(W, rho, area, clmax3)
        return (mass, stab, fin, mass_bd, drag, cl_req, re_mac, v_stall,
                clmax3, depth_scale, tc_body_mean)

    # A wing's skin + spar mass is NOT spread evenly along the chord: the spar
    # and the D-box carry most of it and sit at 25-35% chord, while the
    # trailing edge is a thin wedge. The classic wing-mass chordwise centroid
    # (Torenbeek / Raymer) is ~0.42 c, which is what is used here.
    STRUCT_CHORD_CENTROID = 0.42
    x_ballast = 0.02 * c_centre       # lead sits against the nose bulkhead

    def pack_station(mass, stab, fin, mass_bd, motors, ballast=0.0):
        """Where the flight pack has to sit for the CG to land at x_cg.

        Moment balance:  sum(m_i x_i) + m_pack x_pack = m_total x_cg.
        The lumped power system splits into a fixed part at the motor
        station(s) and the flight battery, which is the part the builder
        actually slides fore and aft.
        """
        from .weights import PACK_PER_MOTOR
        m = mass_bd
        m_at_motors = min(PACK_PER_MOTOR * len(motors), 0.8 * m.power_system)
        m_batt = max(m.power_system - m_at_motors, 1e-6)

        def centroid(mask):
            cc = pf.c[mask]
            if cc.size == 0 or float(np.sum(cc)) <= 0:
                return STRUCT_CHORD_CENTROID * c_centre
            xx = pf.x_le[mask] + STRUCT_CHORD_CENTROID * cc
            return float(np.sum(cc * xx) / np.sum(cc))

        x_wing = centroid(~body_mask)
        x_body = centroid(body_mask)
        el = elevon_layout(pf_def.name, fin)
        f_serv = 0.5 * (el["inner_frac"] + el["outer_frac"])
        x_serv = x_le_at(pf, f_serv) + (1.0 - el["chord_frac"]) * chord_at(pf, f_serv)
        moment = (
            m.structure_wing * x_wing
            + m.structure_body * x_body
            + m.structure_fins * (fin.x_ac if fin.count else x_body)
            + sum(m_at_motors / len(motors) * mo["x"] for mo in motors)
            + m.servos * x_serv
            + m.receiver_wiring * (BAY_START_FRAC * c_centre + 0.35 * bay_len)
            + m.payload * (BAY_START_FRAC * c_centre + 0.10 * bay_len)
            + ballast * x_ballast
        )
        x_batt = (mass * stab.x_cg - moment) / m_batt
        return x_batt, m_batt, moment

    def ballast_needed(mass, stab, m_batt, moment, ballast):
        """Nose lead needed when the pack, jammed against the forward bulkhead,
        still cannot pull the CG far enough forward.

        This is what a builder physically does, and it is not free: the ballast
        is added to the all-up mass, so it raises the wing loading and the stall
        speed and the optimizer pays for it in the cost. Solving
            (m + dm) x_cg = moment_without_ballast + m_batt x_lo + dm x_ballast
        for the extra mass dm gives
            dm = (moment_fixed + m_batt x_lo - m x_cg) / (x_cg - x_ballast)
        """
        moment_fixed = moment - ballast * x_ballast
        num = moment_fixed + m_batt * batt_lo - mass * stab.x_cg
        den = max(stab.x_cg - x_ballast, 1e-4)
        return max(num / den, 0.0)

    # Outer solve: the pack must land inside the equipment bay. On a tailless
    # wing there is no wing position to slide, so the lever is the STATIC
    # MARGIN itself (inside its band): a shallower margin puts the CG further
    # aft, which lets the pack sit further aft too.
    sm = sm_target
    ballast = 0.0
    mass_guess = 0.35 + 3.2 * area
    batt_lo = BAY_START_FRAC * c_centre
    batt_hi = batt_lo + bay_len

    def solve_at(sm_v, mass0, ballast_v):
        r = run_pipeline(sm_v, mass0, ballast_v)
        (m_, st_, fin_, mb_, dr_, cl_, re_, vs_, clm_, ds_, tcb_) = r
        zb = (ds_ - 1.0) * tc_wing * c_centre * (BODY_CROWN_FRAC[pf_def.name] - 0.5)
        mo_ = _motor_layout(n_motors, pf, c_centre, zb, pf_def.name)
        xb_, mbatt_, mom_ = pack_station(m_, st_, fin_, mb_, mo_, ballast_v)
        return r, zb, mo_, xb_, mbatt_, mom_

    res, z_body, motors, x_batt, m_batt, moment = solve_at(sm, mass_guess, 0.0)
    if inp.get("sm_override") is None:
        target = 0.5 * (batt_lo + batt_hi)
        # The CG may slide aft of the mission's target margin to get the pack
        # inside the bay, but only by a quarter of it: past that the model's
        # pitch manners have been traded away for a few grams of lead, which is
        # the wrong way round. Beyond this floor the balance is made up with
        # nose ballast instead.
        sm_lo = max(SM_BAND[0], 0.75 * sm_target)
        for _ in range(8):
            if batt_lo <= x_batt <= batt_hi:
                break
            # d(x_batt)/d(SM) = -mass*MAC/m_batt   (x_cg = x_np - SM*MAC)
            sm_new = float(np.clip(
                sm + (x_batt - target) * m_batt / max(res[0] * res[1].mac, 1e-9),
                sm_lo, SM_BAND[1]))
            if abs(sm_new - sm) < 1e-4:
                break
            sm = sm_new
            res, z_body, motors, x_batt, m_batt, moment = solve_at(
                sm, res[0], ballast)
    # still nose-heavy-short? add lead, and pay for it in mass and cost
    for _ in range(5):
        if x_batt >= batt_lo - 1e-9:
            break
        add = ballast_needed(res[0], res[1], m_batt, moment, ballast)
        if add <= 1e-5:
            break
        ballast = min(ballast + add, 0.35 * res[0])
        res, z_body, motors, x_batt, m_batt, moment = solve_at(sm, res[0], ballast)
        x_batt = max(x_batt, batt_lo)

    (mass, stab, fin, mass_bd, drag, cl_req, re_mac, v_stall, clmax3,
     depth_scale, tc_body_mean) = res
    if ballast > 1e-5:
        mass_bd.components = dict(mass_bd.components)
        mass_bd.components["nose ballast"] = ballast
    W = mass * G

    # ---- overall envelope ---------------------------------------------------
    body_depth = depth_scale * tc_wing * c_centre     # ordinates scaled, chord too
    z_top_body = z_body + 0.5 * body_depth
    z_bot_body = z_body - 0.5 * body_depth
    if fin.count == 0:
        z_top = z_top_body
    elif vs == "center_fin":
        z_top = z_top_body + fin.height
    else:
        c_local_fin = chord_at(pf, fin.y_frac)
        z_top = max(z_top_body,
                    fin.height * math.cos(math.radians(fin.cant_deg))
                    + 0.5 * tc_wing * c_local_fin)
    height_total = (z_top - z_bot_body) + 0.005
    # aft-most point of the surface; the fins' trailing edges are flush with
    # the local wing TE by construction, so the wing sets the length
    length_total = pf.x_te_max + 0.012
    # dihedral raises the tips; account for it in the height budget
    dihedral = DIHEDRAL_DEG[pf_def.name]
    if dihedral > 0:
        height_total += 0.5 * span * math.tan(math.radians(dihedral))

    # ---- constraints --------------------------------------------------------
    wl = mass / area                                   # kg/m^2
    sf = mission.stall_factor
    fin_frac = fin.area_total / area
    c_fin_local = chord_at(pf, fin.y_frac) if fin.count else 1.0
    cons: list[dict] = []

    def con(name, ok, value, limit, msg):
        cons.append({"name": name, "ok": bool(ok), "value": float(value),
                     "limit": float(limit), "message": msg})

    con("box_span", span <= inp["box_w"] + 1e-9, span, inp["box_w"],
        "Wingspan exceeds box width - reduce span or enlarge the box.")
    con("box_length", length_total <= inp["box_l"] + 1e-9, length_total,
        inp["box_l"],
        "Root-to-tip length exceeds box length. A flying wing is deep: either "
        "give it more length, or accept less sweep / a higher aspect ratio.")
    con("box_height", height_total <= inp["box_h"] + 1e-9, height_total,
        inp["box_h"], "Aircraft height exceeds box height.")
    if inp.get("v_stall_target"):
        con("stall_speed", v_stall <= inp["v_stall_target"] * 1.001, v_stall,
            inp["v_stall_target"],
            "Stall speed above your target - more wing area needed (bigger box "
            "or slower cruise).")
    else:
        con("stall_margin", v >= sf * v_stall, v / max(v_stall, 0.1), sf,
            f"Cruise speed must be >= {sf:.2f} x stall speed.")
    con("wing_loading", wl_band[0] * 0.8 <= wl <= wl_band[1] * 1.2, wl,
        wl_band[1],
        f"Wing loading {wl:.1f} kg/m^2 outside the {mission.name} band "
        f"({wl_band[0]:.1f}-{wl_band[1]:.1f}).")
    con("static_margin", SM_BAND[0] <= stab.static_margin <= SM_BAND[1],
        stab.static_margin, sm_target,
        f"Static margin outside the tailless band "
        f"{SM_BAND[0]:.2f}-{SM_BAND[1]:.2f}.")
    con("pitch_stability", stab.dcm_dalpha < 0, stab.dcm_dalpha, 0.0,
        "dCm/dalpha must be negative (CG ahead of NP).")
    con("reflexed_section",
        airfoil.defn.reflexed and airfoil.cm0_geometric() > 0,
        airfoil.cm0_geometric(), 0.0,
        "A flying wing needs a reflexed section with positive Cm0.")
    # --- proportion realism (spec 8.6) --------------------------------------
    con("root_chord_frac",
        pf_def.root_chord_frac_band[0] - 1e-3 <= root_chord_frac
        <= pf_def.root_chord_frac_band[1] + 1e-3,
        root_chord_frac, pf_def.root_chord_frac_band[0],
        "Root chord is not a realistic fraction of the span - the centre "
        "section would be too thin to be the fuselage.")
    con("body_chord_scale", chord_scale >= 1.10 - 1e-9, chord_scale, 1.10,
        "Centre body must extend the root chord by at least 10%.")
    con("body_depth", depth_scale * tc_wing * c_root >= BODY_MIN_DEPTH_M - 1e-6,
        depth_scale * tc_wing * c_root, BODY_MIN_DEPTH_M,
        "Centre body is too shallow to hold a flight pack - a deeper section "
        "or a longer root chord is needed.")
    # --- vertical surfaces (spec 8.7) ---------------------------------------
    if pf_def.bell_spanload:
        con("bell_no_fins", fin.area_total == 0.0, fin.area_total, 0.0,
            "A bell-spanload wing must carry no vertical surfaces.")
    else:
        con("fin_area_frac", 0.02 <= fin_frac <= 0.09, fin_frac, 0.09,
            "Vertical surface area outside the 2-9% of wing area that real "
            "flying wings carry.")
        con("fin_height", fin.height <= 0.60 * c_fin_local + 1e-9, fin.height,
            0.60 * c_fin_local,
            "Fin is taller than 60% of the chord it stands on.")
    con("reynolds", re_mac >= RE_MIN_HARD, re_mac, RE_MIN_HARD,
        "Chord too small - Reynolds number too low for reliable airfoil "
        "performance.")
    con("battery_position", batt_lo - 1e-9 <= x_batt <= batt_hi + 1e-9, x_batt,
        batt_hi,
        "The pack cannot be placed inside the centre-body bay and still put "
        "the CG where the wing trims - the body needs to be longer.")
    # Lead in the nose is a legitimate fix - published plank and Zagi build
    # notes routinely call for 30-60 g of nose weight on a 400-600 g model,
    # i.e. 5-12% - but past 15% the layout is simply wrong: the aeroplane is
    # carrying more ballast than battery and every number suffers for it.
    con("nose_ballast", ballast <= 0.15 * mass + 1e-9, ballast / max(mass, 1e-9),
        0.15,
        "This layout needs an unreasonable amount of nose ballast to balance - "
        "a longer centre body or less sweep would move the CG target aft.")

    # ---- how it will actually HANDLE (the other two axes) -------------------
    # Everything above is longitudinal. A wing that balances perfectly in pitch
    # can still be unflyable: it can hunt in yaw, roll further into its own
    # sideslip, or run out of elevon in the flare. See physics/handling.py.
    a_w = lift_slope_3d(airfoil.lift_slope_2d(re_mac), ar_actual, e_osw)
    hq = handling_qualities(
        pf=pf, area=area, span=span, cl_cruise=cl_req, cd_cruise=drag.cd,
        a_wing=a_w, sweep_le_deg=sweep, dihedral_deg=dihedral,
        fin_area=fin.area_total, fin_arm=fin.l_v, fin_height=fin.height,
        fin_count=fin.count,
        a_fin=lift_slope_3d(2.0 * math.pi, 1.3, 0.85),
        mac=stab.mac, cl_max=clmax3,
        elevon_inner=elevon_layout(pf_def.name, fin)["inner_frac"],
        elevon_outer=elevon_layout(pf_def.name, fin)["outer_frac"],
        elevon_chord_frac=elevon_layout(pf_def.name, fin)["chord_frac"],
        cm_trim_residual=stab.cm_residual,
        bell_spanload=pf_def.bell_spanload, cd0_profile=drag.cd0)

    con("directional_stability", hq.cn_beta > 0.0, hq.cn_beta, 0.0,
        "Directionally unstable: nothing holds the nose into the airflow, so "
        "it will hunt in yaw and eventually swap ends. Needs more sweep or "
        "more fin area.")
    con("roll_stability", hq.cl_beta < 0.0, hq.cl_beta, 0.0,
        "No dihedral effect: a sideslip rolls it FURTHER into the slip, which "
        "is spiral divergence. Needs sweep or geometric dihedral.")
    con("elevon_authority", hq.elevon_trim_used <= 0.75,
        hq.elevon_trim_used, 0.75,
        "Holding trim uses most of the elevon travel, leaving nothing for the "
        "flare - the CG or the section reflex is fighting the wing.")

    # ---- the servo must fit (user decision 2026-08-26: "I need servo's on
    # all planes and designs") ------------------------------------------------
    # Same doctrine as the twin-boom: the OPTIMIZER makes room, the CAD never
    # silently ships a wing without its pockets. The pocket is a world-
    # vertical box, so what the section owes it is the straight-well depth
    # at the ARM station (elevon inner + 0.10, the outboard end of the
    # footprint), minus what dihedral and twist cost at the corners.
    # `twinboom.servo_chord_floor_m` restates the CAD's measured SG90
    # numbers without importing backend.cad.
    from .twinboom import SERVO_HALF_SPAN_MM, servo_chord_floor_m
    el_pre = elevon_layout(pf_def.name, fin)
    f_arm = min(float(el_pre["inner_frac"]) + 0.10
                + (SERVO_HALF_SPAN_MM / 1000.0) / max(0.5 * span, 1e-6),
                0.95)
    twist_arm = abs(float(stab.root_incidence_deg)
                    - float(stab.washout_deg) * f_arm)
    c_arm = float(chord_at(pf, f_arm))
    c_floor_srv = float(servo_chord_floor_m(airfoil, material.wall_mm,
                                            dihedral_deg=dihedral,
                                            twist_deg=twist_arm))
    con("servo_fit", c_arm >= c_floor_srv, c_arm, c_floor_srv,
        "The section at the aileron-servo station cannot bury the measured "
        "SG90 pocket (12.4 mm deep plus clearance and roof). More chord or "
        "less taper at that station is required - every design carries its "
        "servos (user decision).")

    feasible = all(c["ok"] for c in cons)

    # ---- cost ---------------------------------------------------------------
    # SPECIFIC DRAG at cruise, D/W = CD/CL = 1/(L/D), is the main term: the
    # fraction of its own weight the aircraft must push, and therefore exactly
    # what sets glide ratio, cruise thrust and range whatever power system the
    # builder bolts on. Size-normalised, so the optimizer is not biased toward
    # trivially small aircraft. x100 puts it in the 5-17 range of real designs,
    # comparable to the soft band terms below; constraint penalties are two
    # orders larger and always dominate.
    ld_cruise = A.lift_to_drag(cl_req, drag.cd)
    sd_weight = float(inp.get("sd_weight") or 1.0)
    cost = sd_weight * 100.0 / max(ld_cruise, 1e-3)
    wl_pen_w = inp.get("wl_pen_weight")
    wl_pen_w = 1.0 if wl_pen_w is None else float(wl_pen_w)
    cost += 6.0 * wl_pen_w * _band_center_pen(wl, *wl_band)
    # Proportion preference: the planform bands ARE the measured range of real
    # airframes of this family, so sitting in the middle of them is the class
    # consensus and moving to an edge should have to earn it in drag. This is
    # what makes the generated wing read as the aircraft it is meant to be
    # instead of a drag-optimal sliver.
    cost += 4.0 * _band_center_pen(root_chord_frac, *pf_def.root_chord_frac_band)
    cost += 3.0 * _band_center_pen(sweep, *pf_def.sweep_band)
    if re_mac < RE_MIN_GOOD:   # progressive small-chord penalty
        cost += 300.0 * ((RE_MIN_GOOD - re_mac) / RE_MIN_GOOD) ** 2
    span_frac = float(inp.get("span_pref_frac") or 0.88)
    span_target = span_frac * min(inp["box_w"], 2.6)
    if span < span_target:     # mild envelope-utilisation preference
        cost += 25.0 * ((span_target - span) / span_target) ** 2
    # Trim drag. A residual pitching moment has to be held on permanent elevon
    # deflection: with 25%-chord elevons over ~60% of the span, holding
    # Cm ~ 0.05 costs roughly 5 deg of elevon and 5-8% of the cruise L/D. The
    # weight below is set so that a 0.05 residual costs ~0.07 x the specific-
    # drag term, i.e. exactly that L/D penalty.
    cost += 300.0 * stab.cm_residual**2
    # dead weight in the nose: real, but always worth designing away
    cost += 40.0 * (ballast / max(mass, 1e-9)) ** 2
    v_goal = float(inp.get("v_stall_goal") or 0.0)
    if v_goal > 0 and v_stall > v_goal:
        cost += 90.0 * ((v_stall - v_goal) / v_goal) ** 2
    for c in cons:
        if not c["ok"]:
            rel = abs(c["value"] - c["limit"]) / (abs(c["limit"]) + 1e-9)
            cost += 1500.0 + 3000.0 * min(rel, 2.0)

    # ---- assemble the design dict (spec section 4) --------------------------
    el = elevon_layout(pf_def.name, fin)
    canopy = bool(inp.get("canopy")) if inp.get("canopy") is not None else (
        pf_def.name in ("bwb", "swept") and (payload > 0.0 or pf_def.name == "bwb"))
    vsd = VSTABS[vs]

    design = {
        "id": "",
        "planform": pf_def.name,
        "planform_label": pf_def.label,
        "mission": mission.name,
        "geometry": {
            "span_m": float(span),
            "area_m2": float(area),
            "aspect_ratio": float(ar_actual),
            "taper": float(taper),
            "sweep_le_deg": float(sweep),
            "dihedral_deg": float(dihedral),
            "washout_deg": float(stab.washout_deg),
            "root_incidence_deg": float(stab.root_incidence_deg),
            "root_chord_m": float(c_root),
            "tip_chord_m": float(c_tip),
            "airfoil": airfoil.name,
            "fin_airfoil": fin_af.name,
            "body": {
                "half_width_m": float(half_width_m),
                "depth_scale": float(depth_scale),
                "chord_scale": float(chord_scale),
                "nose_round": float(BODY_NOSE_ROUND[pf_def.name]),
                "crown_frac": float(BODY_CROWN_FRAC[pf_def.name]),
                "canopy": bool(canopy),
                "bay_length_m": float(bay_len),
                # the hollow equipment bay the CAD cuts, and the hatch that
                # opens it: usable inside dimensions, walls already deducted
                "bay_width_m": float(bay_w),
                "bay_start_m": float(BAY_START_FRAC * c_centre),
                "bay_wall_m": float(wall_m),
                # implied by chord_scale (the centre TE stays on the wing root
                # TE and the extra chord goes forward); published so the CAD
                # and the physics use the SAME leading-edge root extension
                "nose_ext_m": float(pf.nose_ext),
            },
            "vstab": {
                "type": vs,
                "count": int(fin.count),
                "label": vsd.label,
                "area_total_m2": float(fin.area_total),
                "height_m": float(fin.height),
                "root_chord_m": float(fin.root_chord),
                "tip_chord_m": float(fin.tip_chord),
                "sweep_le_deg": float(fin.sweep_le_deg),
                "cant_deg": float(fin.cant_deg),
                "y_frac": float(fin.y_frac),
            },
            "elevons": dict(el),
            "motors": motors,
            "motor_mount": _motor_mount_spec(inp, mass, motors,
                                             material.wall_mm),
            "battery_x_m": float(np.clip(x_batt, batt_lo, batt_hi)),
            "battery_x_required_m": float(x_batt),
            "length_total_m": float(length_total),
            "height_total_m": float(height_total),
            "control_surfaces": ["elevon_left", "elevon_right"],
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
            "cl_max_2d": float(airfoil.cl_max(re_mac)), "cl_max_3d": float(clmax3),
            "alpha_cruise_deg": float(stab.alpha_cruise_deg),
            "cm0_section": float(airfoil.cm0(re_mac)),
            "cm0_section_geometric": float(airfoil.cm0_geometric()),
        },
        "stability": {
            "mac_m": float(stab.mac), "y_mac_m": float(stab.y_mac),
            "x_le_mac_m": float(stab.x_le_mac),
            "x_ac_wing_m": float(stab.x_ac_wing),
            "x_np_m": float(stab.x_np), "x_cg_m": float(stab.x_cg),
            "static_margin": float(stab.static_margin),
            "cg_pct_mac": float(stab.cg_pct_mac),
            "cg_mm_from_nose": float(stab.x_cg * 1000.0),
            "vv": float(fin.vv), "s_v_m2": float(fin.area_total),
            "l_v_m": float(fin.l_v),
            "fin_height_m": float(fin.height), "fin_chord_m": float(fin.chord_mean),
            "x_fin_ac_m": float(fin.x_ac),
            "dcm_dalpha": float(stab.dcm_dalpha),
            "cm0_wing": float(stab.cm0_cg),
            "cm_trim_residual": float(stab.cm_residual),
            # lateral / directional axes and control authority - the part that
            # decides how it HANDLES rather than whether it balances
            "cn_beta": float(hq.cn_beta),
            "cn_beta_fin": float(hq.cn_beta_fin),
            "cn_beta_sweep": float(hq.cn_beta_sweep),
            "cl_beta": float(hq.cl_beta),
            "cl_beta_sweep": float(hq.cl_beta_sweep),
            "cl_beta_dihedral": float(hq.cl_beta_dihedral),
            "cl_beta_fin": float(hq.cl_beta_fin),
            "roll_yaw_ratio": float(hq.ratio),
            "elevon_authority_cm": float(hq.elevon_authority),
            "elevon_trim_used": float(hq.elevon_trim_used),
            "proverse_yaw": float(hq.proverse_yaw),
            "handling_notes": list(hq.notes),
            "washout_deg": float(stab.washout_deg),
            "bell_spanload": bool(stab.bell_spanload),
            "notes": list(stab.notes),
        },
        "power_system": {
            "n_motors": int(n_motors),
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
        "notes": list(stab.notes) + vstab_notes,
        "cost": float(cost),
    }
    return design


# ---------------------------------------------------------------------------
# Search driver
# ---------------------------------------------------------------------------

def _refine_envelope(design: dict, inp: dict) -> None:
    """Record the envelope of the shape the CAD will ACTUALLY build, and
    re-check the box constraints against it.

    The estimate used during the search is deliberately cheap and conservative
    - it is evaluated at every one of several hundred grid points. Once a
    design has been chosen it is worth a few milliseconds to replace it with
    the true extents: `length_total_m` and `height_total_m` are shown to the
    user as the aircraft's dimensions, and a plank that reports 209 mm tall
    while measuring 171 mm is a WRONG number, not a safe one. It also stops the
    optimizer throwing away designs that fit the user's box perfectly well.

    Deferred import on purpose: `physics` must not depend on `cad` at module
    level. The helper is pure geometry (no OpenCASCADE) and reads exactly the
    `geometry` keys the builder reads, so the two cannot drift apart.
    """
    try:
        from ..cad.geometry import airframe_extents_m
        ext = airframe_extents_m(design)
        length = float(ext["length_m"])
        height = float(ext["height_m"])
    except Exception:
        return                      # keep the conservative estimate
    if not (math.isfinite(length) and math.isfinite(height)
            and length > 0 and height > 0):
        return

    g = design["geometry"]
    g["length_total_m"], g["height_total_m"] = length, height
    limits = {"box_length": (length, inp["box_l"]), "box_height": (height, inp["box_h"])}
    for c in design["constraints"]:
        if c["name"] in limits:
            value, limit = limits[c["name"]]
            c["value"], c["ok"] = float(value), bool(value <= limit + 1e-9)
    design["feasible"] = all(c["ok"] for c in design["constraints"])
    design["binding"] = [c["name"] for c in design["constraints"] if not c["ok"]]


def optimize_design(inp: dict) -> dict:
    """Sweep the feasible region, keep the best design, polish it and return
    the full design dict. If nothing is feasible, the closest design is
    returned with its binding constraints listed (never an exception).

    `airfoil_pref` may be a LIST of candidate sections (entries may be None,
    meaning "the automatic pick"); the whole search then runs once per candidate
    and the cheapest result wins.
    """
    # v2 multi-type dispatch (V2_PLAN.md): anything that is not a flying wing
    # is solved in physics.conventional. The flying-wing path below is the v1
    # code, untouched - an absent/"flying_wing" airplane_type never enters
    # this branch and never imports the conventional module.
    atype = inp.get("airplane_type") or "flying_wing"
    if atype != "flying_wing":
        from .conventional import optimize_conventional
        if atype == "conventional":
            return optimize_conventional(inp)
        # "any": solve both types and keep the better (feasible-first, then
        # the shared specific-drag objective)
        fw = optimize_design({**inp, "airplane_type": "flying_wing"})
        conv = optimize_conventional(inp)
        return min((fw, conv),
                   key=lambda d: (not d["feasible"], d["cost"]))

    pref = inp.get("airfoil_pref")
    if isinstance(pref, (list, tuple)):
        pf_name = inp.get("planform") or "swept"
        ms_name = inp.get("mission") or "sport"
        best_any, seen = None, set()
        for cand in (list(pref) or [None]):
            resolved = pick_airfoil(
                pf_name, ms_name, inp.get("airfoil_override") or cand).name
            if resolved in seen:
                continue
            seen.add(resolved)
            try:
                d = optimize_design({**inp, "airfoil_pref": cand})
            except Exception:
                continue
            if best_any is None or d["cost"] < best_any["cost"]:
                best_any = d
        if best_any is None:
            raise RuntimeError("optimizer could not evaluate any candidate design")
        return best_any

    pf_def: PlanformDef = PLANFORMS[inp.get("planform") or "swept"]

    b_hi = min(float(inp["box_w"]), 2.6)
    b_lo = min(max(0.45, 0.55 * b_hi), 0.92 * b_hi)

    ar_lo, ar_hi = _band(inp, "ar_band_shift", pf_def.ar_band, 3.0, 14.0)
    if inp.get("ar_target"):
        ar_lo = ar_hi = float(np.clip(inp["ar_target"], 3.0, 14.0))

    sw_lo, sw_hi = pf_def.sweep_band
    if inp.get("sweep_override") is not None:
        sw_lo = sw_hi = float(np.clip(inp["sweep_override"], *pf_def.sweep_band))
    elif inp.get("sweep_shift_deg"):
        mid = float(np.clip(0.5 * (sw_lo + sw_hi) + float(inp["sweep_shift_deg"]),
                            sw_lo, sw_hi))
        sw_lo, sw_hi = max(sw_lo, mid - 3.0), min(sw_hi, mid + 3.0)

    tp_lo, tp_hi = pf_def.taper_band
    if inp.get("taper_override") is not None:
        tp_lo = tp_hi = float(np.clip(inp["taper_override"], *pf_def.taper_band))

    spans = np.linspace(b_lo, b_hi, 5)
    ars = (np.linspace(ar_lo, ar_hi, 4) if ar_hi > ar_lo + 1e-6
           else np.array([ar_lo]))
    sweeps = (np.linspace(sw_lo, sw_hi, 3) if sw_hi > sw_lo + 1e-6
              else np.array([sw_lo]))
    tapers = (np.linspace(tp_lo, tp_hi, 3) if tp_hi > tp_lo + 1e-6
              else np.array([tp_lo]))

    grid = [{"span": float(b), "ar": float(a), "sweep": float(s),
             "taper": float(t)}
            for b in spans for a in ars for s in sweeps for t in tapers]
    for s in (inp.get("seeds") or []):
        try:
            grid.append({
                "span": float(np.clip(s["span"], 0.3, b_hi)),
                "ar": float(np.clip(s["ar"], ar_lo, ar_hi)),
                "sweep": float(np.clip(s.get("sweep", 0.5 * (sw_lo + sw_hi)),
                                       sw_lo, sw_hi)),
                "taper": float(np.clip(s.get("taper", 0.5 * (tp_lo + tp_hi)),
                                       tp_lo, tp_hi)),
            })
        except Exception:
            continue

    best = None
    for xd in grid:
        try:
            d = evaluate_candidate(xd, inp)
        except Exception:
            continue
        if best is None or d["cost"] < best["cost"]:
            best = d

    if best is None:  # should not happen; absolute fallback
        raise RuntimeError("optimizer could not evaluate any candidate design")

    # Nelder-Mead polish on the penalised cost around the best grid point
    g = best["geometry"]
    x0 = [g["span_m"], g["aspect_ratio"], g["sweep_le_deg"], g["taper"]]

    def unpack(xv):
        return {"span": float(np.clip(xv[0], 0.3, b_hi)),
                "ar": float(np.clip(xv[1], ar_lo, ar_hi)),
                "sweep": float(np.clip(xv[2], sw_lo, sw_hi)),
                "taper": float(np.clip(xv[3], tp_lo, tp_hi))}

    def f(xv):
        try:
            return evaluate_candidate(unpack(xv), inp)["cost"]
        except Exception:
            return 1e9

    try:
        r = sciopt.minimize(f, x0, method="Nelder-Mead",
                            options={"maxfev": 90, "xatol": 1e-3, "fatol": 0.4})
        polished = evaluate_candidate(unpack(r.x), inp)
        if polished["cost"] <= best["cost"]:
            best = polished
    except Exception:
        pass

    best["id"] = uuid.uuid4().hex[:12]
    _refine_envelope(best, inp)
    if not best["feasible"]:
        msgs = [c["message"] for c in best["constraints"] if not c["ok"]]
        best["notes"].append(
            "Design is the CLOSEST FEASIBLE result - binding constraint(s): "
            + "; ".join(msgs))
    return best
