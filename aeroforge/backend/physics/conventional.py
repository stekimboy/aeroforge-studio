"""CONVENTIONAL airplane design solve + variant generation (V2_PLAN.md).

NEW in v2. This module lives BESIDE the flying-wing path, never inside it:
`optimizer.optimize_design` and `variants.generate_variants` dispatch here
when `airplane_type` says "conventional" (or "any"), and nothing in here is
imported on a flying-wing run.

The airframe: wing + fuselage + horizontal & vertical stabilizer, single
TRACTOR motor on the nose (motor face at the x = 0 datum - nothing protrudes
ahead of it, same invariant as v1). Search space is (span, AR, taper, tail
arm), each bounded by the chosen style's researched band; the coupled
weight / aero / stability / bay-sizing problem is re-solved at every point
exactly the way the flying-wing optimizer does it, and the objective is the
SAME specific drag 100/(L/D) plus soft proportion terms, so an "any"-mode
comparison across types is apples to apples.

Every band cites RESEARCH_CONVENTIONAL.md (tags [MIT]/[ND]/[SCHOLZ] etc.);
the design dict mirrors the flying-wing dict's key names wherever the concept
matches and adds the `geometry.fuselage` / `geometry.tail` /
`geometry.ailerons` blocks of the V2_PLAN.md schema.
"""
from __future__ import annotations

import math
import uuid

import numpy as np
from scipy import optimize as sciopt

from . import aero as A
from .airfoils import LIBRARY, Airfoil, AirfoilDef, fin_airfoil
from .atmosphere import isa, reynolds
from .config_defs import (
    CONV_FUS_HEIGHT_FRAC, CONV_FUS_LEN_FRAC_BAND, CONV_LH_MAC_BAND,
    CONV_SH_S_BAND, CONV_SM_BAND, CONV_STYLE_FOR_MISSION, CONV_STYLES,
    CONV_VH_BAND, CONV_VV_BAND, MISSIONS, ConvStyleDef, MissionDef,
)
from .optimizer import (
    G, RE_MIN_GOOD, RE_MIN_HARD, _band, _band_center_pen, _motor_mount_spec,
)
from .stability import chords_from_area, mac_length, y_mac
from .stability_conv import conv_handling, size_tail, solve_conventional
from .weights import MATERIALS, PACK_PER_MOTOR, estimate_mass_conventional

# ---------------------------------------------------------------------------
# Non-reflexed cambered wing sections for the tailed wing.
#
# The tail does the trimming here, so the wing flies a PLAIN cambered section
# (negative Cm_ac) - exactly what a flying wing must never do. The airfoil
# library already generates any NACA 4-digit section analytically (that is how
# the fin's NACA 0008 is built), so the conventional sections are registered
# through that existing machinery; no new airfoil code (task rule):
#   NACA 2412 - the archetypal trainer/sport section ([ND] uses it as the
#               default cambered example; Cessna-classic camber at 12%)
#   NACA 4412 - high-lift floater camber
#   NACA 0010 - symmetric aerobatic section ([ND]: "symmetrical airfoils are
#               intended for aerobatic models")
#   NACA 1410 - thin low-camber speed section
# Registration is idempotent and only ever ADDS non-reflexed entries, which
# the flying-wing picker (`pick_airfoil` requires .reflexed) can never select.
# ---------------------------------------------------------------------------
_CONV_SECTIONS = (
    AirfoilDef("NACA 2412", "naca4", "2412", 0.12,
               "Classic cambered trainer/sport section (tail-trimmed)"),
    AirfoilDef("NACA 4412", "naca4", "4412", 0.12,
               "High-lift cambered floater section (tail-trimmed)"),
    AirfoilDef("NACA 0010", "naca4", "0010", 0.10,
               "Symmetric aerobatic wing section"),
    AirfoilDef("NACA 1410", "naca4", "1410", 0.10,
               "Thin low-camber speed section (tail-trimmed)"),
)
for _d in _CONV_SECTIONS:
    LIBRARY.setdefault(_d.name, _d)
CONV_AIRFOILS: tuple[str, ...] = tuple(d.name for d in _CONV_SECTIONS)

N_MOTORS_CONV = 1        # single tractor by definition (V2_PLAN.md)
N_SERVOS_CONV = 4        # 2 ailerons + elevator + rudder (dossier s.5:
#                          FT Simple Cub "(4) 9 gram servos")

# ---------------------------------------------------------------------------
# Fuselage proportions (RESEARCH_CONVENTIONAL.md s.6)
# ---------------------------------------------------------------------------
FUS_WIDTH_MIN_M = 0.064      # bay >= 60 mm wide + 2 walls (dossier s.6)
FUS_WIDTH_HEIGHT_FRAC = 0.85  # slab-sided box, a little narrower than deep
FUS_HEIGHT_MIN_M = 0.073     # bay >= 45 mm deep at 62% of the section
# wetted-area estimate of the rounded slab-sided shell: perimeter x length,
# corners rounded (x0.90) and nose+tail closure taper (x0.78) - a standard
# quick fuselage wetted-area take-off
FUS_WET_PERIM = 0.90
FUS_WET_TAPER = 0.78
FUS_VOLUME_FILL = 0.55       # enclosed volume / bounding box (nose + tail cone)
BAY_START_FRAC_F = 0.08      # behind the motor bulkhead + firewall
BAY_LEN_FRAC_F = 0.30
BAY_LEN_MIN_M = 0.13         # dossier s.6: bay >= 130 mm long
BAY_DEPTH_FRAC = 0.62        # usable depth of the fuselage section
BAY_END_FRAC_MAX = 0.55      # the aft fuselage is tail boom, not bay

# wing structure chordwise centroid - same Torenbeek/Raymer 0.42c doctrine as
# the flying-wing optimizer (spar + D-box forward, TE a thin wedge)
STRUCT_CHORD_CENTROID = 0.42


def _style_for(inp: dict) -> ConvStyleDef:
    key = inp.get("conv_style")
    if key in CONV_STYLES:
        return CONV_STYLES[key]
    mission = inp.get("mission") or "sport"
    return CONV_STYLES[CONV_STYLE_FOR_MISSION.get(mission, "trainer")]


def _conv_airfoil(style: ConvStyleDef, inp: dict) -> Airfoil:
    """The style's section, or a user override IF it names a conventional
    (non-reflexed) section - a reflexed section on a tailed wing would fight
    the tail, so such an override is ignored, mirroring pick_airfoil."""
    override = inp.get("airfoil_override")
    if override in CONV_AIRFOILS:
        return Airfoil(override)
    return Airfoil(style.airfoil)


# ---------------------------------------------------------------------------
# Single full-physics evaluation of one candidate
# ---------------------------------------------------------------------------

def evaluate_conventional(x: dict, inp: dict) -> dict:
    """Evaluate one candidate {span, ar, taper, lh} through the whole tailed
    pipeline. Returns the full V2_PLAN.md design dict with 'cost',
    'constraints' and 'feasible' filled in."""
    style = _style_for(inp)
    mission: MissionDef = MISSIONS[inp.get("mission") or "sport"]
    material = MATERIALS[inp["material"]]
    atm = isa(0.0)
    rho, mu = atm.density_kgm3, atm.viscosity_Pas
    wall_m = material.wall_mm / 1000.0

    # ---- search variables, clipped to the style's researched bands ----------
    span = float(x["span"])
    ar_band = _band(inp, "ar_band_shift", style.ar_band, 3.5, 9.0)
    ar = float(np.clip(x["ar"], *ar_band))
    taper = float(np.clip(x.get("taper", 0.5 * sum(style.taper_band)),
                          *style.taper_band))
    lh_mac = float(np.clip(x.get("lh", style.lh_mac), *CONV_LH_MAC_BAND))

    area = span**2 / ar                                  # S = b^2/AR
    c_root, c_tip = chords_from_area(span, area, taper)  # S = b (c_r+c_t)/2
    mac = mac_length(c_root, taper)
    y_bar = y_mac(span, taper)                           # half-wing centroid
    x_w = style.nose_len_mac * mac       # [ND]: nose 1-1.5 chords ahead of LE

    airfoil = _conv_airfoil(style, inp)
    fin_af = fin_airfoil()
    tc_wing = airfoil.defn.thickness
    e_osw = A.oswald_efficiency(ar, 1.0)  # full Raymer estimate - no tailless
    #                                       penalty on a plain planar wing
    v = float(inp["v_cruise"])
    sm_target = float(np.clip(inp.get("sm_override") or style.sm_target,
                              *CONV_SM_BAND))
    s = inp.get("wl_band_scale") or 1.0
    s_lo, s_hi = s if isinstance(s, (tuple, list)) else (s, s)
    wl_band = (style.wl_band[0] * float(s_lo), style.wl_band[1] * float(s_hi))
    payload = float(inp.get("payload_kg") or 0.0)
    washout = style.washout_deg
    dihedral = style.dihedral_deg

    # ---- coupled mass / aero / stability / fuselage solve --------------------
    def run_pipeline(sm: float, mass0: float, ballast: float = 0.0):
        mass = mass0
        x_cg_est = x_w + 0.28 * mac      # [ND] trainer CG 25-33% chord
        l_f = style.fus_len_frac * span  # first-pass fuselage length
        stab = tail = drag = mass_bd = None
        cl_req = re_mac = v_stall = clmax3 = 0.0
        fus_h = fus_w = s_wet_fus = vol_fus = 0.0
        for _ in range(30):
            W = mass * G
            cl_req = A.required_cl(W, rho, v, area)       # L = W at cruise
            re_mac = reynolds(rho, v, mac, mu)            # Re = rho V c / mu
            clmax3 = A.cl_max_3d(airfoil.cl_max(re_mac), 0.0, washout)
            v_stall = A.stall_speed(W, rho, area, clmax3)

            # fuselage envelope: height 10-15% of length [ND], width from the
            # equipment-bay minimum (dossier s.6); volume feeds the Munk term
            fus_h = max(CONV_FUS_HEIGHT_FRAC * l_f, FUS_HEIGHT_MIN_M)
            fus_w = max(FUS_WIDTH_MIN_M, FUS_WIDTH_HEIGHT_FRAC * fus_h)
            vol_fus = FUS_VOLUME_FILL * l_f * fus_w * fus_h

            # tail sizing + NP/CG/trim, iterated for the CG <-> arm coupling
            for _ in range(3):
                tail = size_tail(area=area, span=span, mac=mac,
                                 x_cg=x_cg_est, vh_target=style.vh,
                                 vv_target=style.vv, lh_mac=lh_mac)
                stab = solve_conventional(
                    span=span, area=area, mac=mac, y_mac_m=y_bar,
                    x_le_wing=x_w, airfoil=airfoil, re_mac=re_mac, ar=ar,
                    e_oswald=e_osw, tail=tail, sm_target=sm,
                    cl_cruise=cl_req, fus_volume_m3=vol_fus,
                    sm_band=CONV_SM_BAND)
                if abs(stab.x_cg - x_cg_est) < 1e-5:
                    x_cg_est = stab.x_cg
                    break
                x_cg_est = stab.x_cg
            l_f = tail.x_te              # the fuselage ends at the tail post
            s_wet_fus = (FUS_WET_PERIM * 2.0 * (fus_w + fus_h)
                         * l_f * FUS_WET_TAPER)

            drag = _build_drag_conv(cl_req, stab.cl_w_cruise, airfoil, re_mac,
                                    ar, e_osw, area, tail, l_f, fus_w, fus_h,
                                    s_wet_fus, rho, v, mu)

            mass_bd = estimate_mass_conventional(
                material=material, wing_area=area, tail_area_h=tail.s_h,
                tail_area_v=tail.s_v, fus_wetted_area=s_wet_fus,
                n_motors=N_MOTORS_CONV, n_servos=N_SERVOS_CONV,
                payload_kg=payload)
            new_mass = mass_bd.total + ballast
            if abs(new_mass - mass) < 1e-4:
                mass = new_mass
                break
            mass = 0.5 * mass + 0.5 * new_mass
        # final consistency pass so L = W holds to numerical precision
        W = mass * G
        cl_req = A.required_cl(W, rho, v, area)
        v_stall = A.stall_speed(W, rho, area, clmax3)
        return (mass, stab, tail, mass_bd, drag, cl_req, re_mac, v_stall,
                clmax3, l_f, fus_h, fus_w, s_wet_fus, vol_fus)

    # ---- battery placement (the builder's trim tool, dossier s.6) -----------
    x_ballast = 0.02                     # lead sits against the nose bulkhead

    def bay_bounds(l_f: float) -> tuple[float, float, float, float]:
        x0 = max(BAY_START_FRAC_F * l_f, 0.030)
        length = max(BAY_LEN_FRAC_F * l_f, BAY_LEN_MIN_M)
        length = min(length, BAY_END_FRAC_MAX * l_f - x0)
        return x0, max(length, 0.05), x0, x0 + max(length, 0.05)

    def pack_station(mass, stab, tail, mass_bd, l_f, bay_lo, bay_len,
                     ballast=0.0):
        """Moment balance sum(m_i x_i) + m_batt x_batt = m x_cg, solved for
        the battery station - identical doctrine to the flying-wing solver."""
        m = mass_bd
        m_at_motors = min(PACK_PER_MOTOR * N_MOTORS_CONV, 0.8 * m.power_system)
        m_batt = max(m.power_system - m_at_motors, 1e-6)
        x_wing = x_w + STRUCT_CHORD_CENTROID * mac    # Torenbeek 0.42c
        x_fus = 0.45 * l_f          # shell centroid, slightly forward (nose
        #                             structure heavier than the tail cone)
        x_tail = ((tail.s_h * tail.x_ac_h + tail.s_v * tail.x_ac_v)
                  / max(tail.s_h + tail.s_v, 1e-9))
        # 2 aileron servos live in the wing; elevator/rudder servos sit at the
        # wing-root bay area with pushrods to the tail (V2_PLAN.md)
        from .weights import SERVO_MASS
        moment = (
            m.structure_wing * x_wing
            + m.structure_body * x_fus
            + m.structure_fins * x_tail
            + m_at_motors * 0.02                       # motor + ESC at nose
            + 2 * SERVO_MASS * (x_w + 0.75 * mac)      # aileron servos
            + 2 * SERVO_MASS * (bay_lo + 0.9 * bay_len)  # elev/rudder servos
            + m.receiver_wiring * (bay_lo + 0.35 * bay_len)
            + m.payload * (bay_lo + 0.10 * bay_len)
            + ballast * x_ballast
        )
        x_batt = (mass * stab.x_cg - moment) / m_batt
        return x_batt, m_batt, moment

    def ballast_needed(mass, stab, m_batt, moment, ballast, batt_lo):
        """Nose lead when the pack against the forward bulkhead still leaves
        the CG too far aft (same closed form as the flying-wing solver)."""
        moment_fixed = moment - ballast * x_ballast
        num = moment_fixed + m_batt * batt_lo - mass * stab.x_cg
        den = max(stab.x_cg - x_ballast, 1e-4)
        return max(num / den, 0.0)

    sm = sm_target
    ballast = 0.0
    mass_guess = 0.30 + 2.8 * area

    def solve_at(sm_v, mass0, ballast_v):
        r = run_pipeline(sm_v, mass0, ballast_v)
        (m_, st_, tl_, mb_, dr_, cl_, re_, vs_, clm_, lf_, fh_, fw_, sw_,
         vf_) = r
        bay_x0, bay_len, lo, hi = bay_bounds(lf_)
        xb_, mbatt_, mom_ = pack_station(m_, st_, tl_, mb_, lf_, lo, bay_len,
                                         ballast_v)
        return r, (bay_x0, bay_len, lo, hi), xb_, mbatt_, mom_

    res, bay, x_batt, m_batt, moment = solve_at(sm, mass_guess, 0.0)
    batt_lo, batt_hi = bay[2], bay[3]
    if inp.get("sm_override") is None:
        # On a conventional the wing position is fixed by the style, so the
        # in-band lever is the SAME one the flying wing uses: let the margin
        # slide (a bounded amount) to put the pack inside the bay, then make
        # up the remainder with nose lead. Slide floor mirrors v1: no more
        # than a quarter of the target given away.
        target = 0.5 * (batt_lo + batt_hi)
        sm_lo = max(CONV_SM_BAND[0], 0.75 * sm_target)
        for _ in range(8):
            if batt_lo <= x_batt <= batt_hi:
                break
            sm_new = float(np.clip(
                sm + (x_batt - target) * m_batt / max(res[0] * res[1].mac, 1e-9),
                sm_lo, CONV_SM_BAND[1]))
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
     l_f, fus_h, fus_w, s_wet_fus, vol_fus) = res
    bay_x0, bay_len = bay[0], bay[1]
    if ballast > 1e-5:
        mass_bd.components = dict(mass_bd.components)
        mass_bd.components["nose ballast"] = ballast
    W = mass * G

    # ---- overall envelope -----------------------------------------------------
    # z = 0 on the thrust line / fuselage mid-height. High wing sits ON the
    # deck, low wing on the keel; dihedral raises the tips.
    wing_z = 0.5 * fus_h if style.wing_position == "high" else -0.35 * fus_h
    tip_rise = 0.5 * span * math.tan(math.radians(dihedral))
    z_top = max(0.5 * fus_h + tail.h_v,           # fin standing on the deck
                wing_z + tip_rise + 0.5 * tc_wing * c_tip)
    height_total = z_top + 0.5 * fus_h + 0.005
    length_total = l_f + 0.005

    # ---- constraints -----------------------------------------------------------
    wl = mass / area
    sf = mission.stall_factor
    fus_len_frac = l_f / span
    sh_s = tail.s_h / area
    cons: list[dict] = []

    def con(name, ok, value, limit, msg):
        cons.append({"name": name, "ok": bool(ok), "value": float(value),
                     "limit": float(limit), "message": msg})

    con("box_span", span <= inp["box_w"] + 1e-9, span, inp["box_w"],
        "Wingspan exceeds box width - reduce span or enlarge the box.")
    con("box_length", length_total <= inp["box_l"] + 1e-9, length_total,
        inp["box_l"],
        "Fuselage (nose to tail post) exceeds box length - a conventional "
        "needs ~3/4 of its span in length; give it more box or less span.")
    con("box_height", height_total <= inp["box_h"] + 1e-9, height_total,
        inp["box_h"], "Aircraft height (fin + dihedral) exceeds box height.")
    # ---- the aileron servo must fit (user decision 2026-08-26: "I need
    # servo's on all planes and designs") - same doctrine as the twin-boom:
    # the OPTIMIZER makes room, the CAD never silently ships a wing without
    # its pockets. Shared floor from physics.twinboom (no backend.cad).
    from .twinboom import SERVO_HALF_SPAN_MM, servo_chord_floor_m
    f_arm = min(float(style.aileron_inner_frac) + 0.10
                + (SERVO_HALF_SPAN_MM / 1000.0) / max(0.5 * span, 1e-6),
                0.95)
    c_arm = float(c_root + (c_tip - c_root) * f_arm)
    c_floor_srv = float(servo_chord_floor_m(airfoil, material.wall_mm,
                                            dihedral_deg=dihedral,
                                            twist_deg=0.0))
    con("servo_fit", c_arm >= c_floor_srv, c_arm, c_floor_srv,
        "The wing section at the aileron-servo station cannot bury the "
        "measured SG90 pocket (12.4 mm deep plus clearance and roof). More "
        "chord or less taper is required - every design carries its servos "
        "(user decision).")
    if inp.get("v_stall_target"):
        con("stall_speed", v_stall <= inp["v_stall_target"] * 1.001, v_stall,
            inp["v_stall_target"],
            "Stall speed above your target - more wing area needed (bigger "
            "box or slower cruise).")
    else:
        con("stall_margin", v >= sf * v_stall, v / max(v_stall, 0.1), sf,
            f"Cruise speed must be >= {sf:.2f} x stall speed.")
    con("wing_loading", wl_band[0] * 0.8 <= wl <= wl_band[1] * 1.2, wl,
        wl_band[1],
        f"Wing loading {wl:.1f} kg/m^2 outside the {style.name} band "
        f"({wl_band[0]:.1f}-{wl_band[1]:.1f}).")
    con("static_margin",
        CONV_SM_BAND[0] - 1e-6 <= stab.static_margin <= CONV_SM_BAND[1] + 1e-6,
        stab.static_margin, sm_target,
        f"Static margin outside the tailed band "
        f"{CONV_SM_BAND[0]:.2f}-{CONV_SM_BAND[1]:.2f} ([MIT]).")
    con("pitch_stability", stab.dcm_dalpha < 0, stab.dcm_dalpha, 0.0,
        "dCm/dalpha must be negative (CG ahead of NP).")
    # the tail trims, so the wing section must NOT be reflexed - the exact
    # inverse of the flying-wing gate
    con("cambered_section",
        (not airfoil.defn.reflexed) and airfoil.cm0_geometric() <= 1e-3,
        airfoil.cm0_geometric(), 0.0,
        "A tailed wing flies a plain cambered (non-reflexed) section; the "
        "tail does the trimming.")
    # --- researched tail proportion bands (dossier s.2) ----------------------
    con("vh_band", CONV_VH_BAND[0] - 1e-6 <= stab.vh <= CONV_VH_BAND[1] + 1e-6,
        stab.vh, CONV_VH_BAND[1],
        f"Horizontal tail volume V_H outside the {CONV_VH_BAND[0]:.2f}-"
        f"{CONV_VH_BAND[1]:.2f} band ([MIT]/[RCG]/[SCHOLZ]).")
    con("vv_band", CONV_VV_BAND[0] - 1e-6 <= stab.vv <= CONV_VV_BAND[1] + 1e-6,
        stab.vv, CONV_VV_BAND[1],
        f"Vertical tail volume V_V outside the {CONV_VV_BAND[0]:.3f}-"
        f"{CONV_VV_BAND[1]:.3f} band ([MIT]/[SCHOLZ]).")
    con("tail_arm",
        CONV_LH_MAC_BAND[0] - 1e-6 <= stab.l_h / mac
        <= CONV_LH_MAC_BAND[1] + 1e-6,
        stab.l_h / mac, CONV_LH_MAC_BAND[1],
        "Tail arm outside the 2.5-3.5 MAC band ([ND]): shorter reads "
        "free-flight, longer reads pattern ship.")
    con("stab_area_frac",
        CONV_SH_S_BAND[0] - 1e-6 <= sh_s <= CONV_SH_S_BAND[1] + 1e-6,
        sh_s, CONV_SH_S_BAND[1],
        "Stab area is not a realistic fraction of the wing ([ND]: 15-20%).")
    con("fuselage_proportion",
        CONV_FUS_LEN_FRAC_BAND[0] - 1e-6 <= fus_len_frac
        <= CONV_FUS_LEN_FRAC_BAND[1] + 1e-6,
        fus_len_frac, CONV_FUS_LEN_FRAC_BAND[1],
        "Fuselage length is not ~3/4 of the span ([ND]; real airframes "
        "0.63-0.80) - the layout would not read as a conventional model.")
    con("reynolds", re_mac >= RE_MIN_HARD, re_mac, RE_MIN_HARD,
        "Chord too small - Reynolds number too low for reliable airfoil "
        "performance.")
    con("battery_position", batt_lo - 1e-9 <= x_batt <= batt_hi + 1e-9,
        x_batt, batt_hi,
        "The pack cannot sit inside the fuselage bay and still balance the "
        "aircraft - a longer nose or bay is needed.")
    con("nose_ballast", ballast <= 0.15 * mass + 1e-9,
        ballast / max(mass, 1e-9), 0.15,
        "This layout needs an unreasonable amount of nose ballast - a longer "
        "nose moment or a lighter tail would fix the balance properly.")

    # ---- handling: the other two axes, TAILED thresholds ---------------------
    hq = conv_handling(
        area=area, span=span, mac=mac, y_bar=y_bar, cl_cruise=cl_req,
        a_wing=stab.a_wing, a_total=stab.a_total, dihedral_deg=dihedral,
        wing_position=style.wing_position, tail=tail, vh=stab.vh, vv=stab.vv,
        l_v=stab.l_v, a_tail=stab.a_tail, fus_volume_m3=vol_fus,
        fus_height_m=fus_h, static_margin=stab.static_margin, cl_max=clmax3,
        elevator_frac=style.elevator_frac)

    con("directional_stability", hq.cn_beta > 0.03, hq.cn_beta, 0.03,
        "Directional stiffness below what a tailed model needs (Cn_beta "
        "0.05-0.15 /rad is the flown band) - more fin volume.")
    con("roll_stability", hq.cl_beta < 0.0, hq.cl_beta, 0.0,
        "No dihedral effect: a sideslip rolls it further into the slip. "
        "2-3 deg of dihedral is the fix.")
    con("elevator_authority", hq.elevator_trim_used <= 0.75,
        hq.elevator_trim_used, 0.75,
        "Pulling to CL_max eats most of the elevator travel - a shallower "
        "margin or a bigger elevator is needed.")

    feasible = all(c["ok"] for c in cons)

    # ---- cost (same objective family as the flying-wing optimizer, so an
    # "any"-mode comparison is meaningful) --------------------------------------
    ld_cruise = A.lift_to_drag(cl_req, drag.cd)
    sd_weight = float(inp.get("sd_weight") or 1.0)
    cost = sd_weight * 100.0 / max(ld_cruise, 1e-3)
    wl_pen_w = inp.get("wl_pen_weight")
    wl_pen_w = 1.0 if wl_pen_w is None else float(wl_pen_w)
    cost += 6.0 * wl_pen_w * _band_center_pen(wl, *wl_band)
    # the style bands ARE the researched proportions of the class - sitting in
    # the middle of them is what makes the model read as the aircraft it is
    cost += 4.0 * _band_center_pen(ar, *style.ar_band)
    cost += 2.0 * _band_center_pen(taper, *style.taper_band)
    cost += 2.0 * _band_center_pen(fus_len_frac, 0.63, 0.80)
    if re_mac < RE_MIN_GOOD:
        cost += 300.0 * ((RE_MIN_GOOD - re_mac) / RE_MIN_GOOD) ** 2
    span_frac = float(inp.get("span_pref_frac") or 0.88)
    span_target = span_frac * min(inp["box_w"], inp["box_l"] / 0.66, 2.6)
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

    # ---- assemble the design dict (V2_PLAN.md schema) -------------------------
    motors = [{"x": 0.005, "y": 0.0, "z": 0.0, "type": "tractor"}]
    bay_w = max(fus_w - 2.0 * wall_m, 0.02)
    bay_d = BAY_DEPTH_FRAC * fus_h

    design = {
        "id": "",
        "airplane_type": "conventional",
        "planform": style.name,
        "planform_label": style.label,
        "mission": mission.name,
        "geometry": {
            "span_m": float(span),
            "area_m2": float(area),
            "aspect_ratio": float(ar),
            "taper": float(taper),
            "sweep_le_deg": 0.0,          # straight LE (dossier s.4: no sweep)
            "dihedral_deg": float(dihedral),
            "washout_deg": float(washout),
            "root_incidence_deg": float(stab.incidence_w_deg),
            "root_chord_m": float(c_root),
            "tip_chord_m": float(c_tip),
            "airfoil": airfoil.name,
            "fin_airfoil": fin_af.name,
            "wing_position": style.wing_position,
            "fuselage": {
                "length_m": float(l_f),
                "width_m": float(fus_w),
                "height_m": float(fus_h),
                "x_wing_le_m": float(x_w),
                "wetted_area_m2": float(s_wet_fus),
                "bay": {
                    "bay_start_m": float(bay_x0),
                    "bay_length_m": float(bay_len),
                    "bay_width_m": float(bay_w),
                    "bay_depth_m": float(bay_d),
                    "bay_wall_m": float(wall_m),
                },
            },
            "tail": {
                "arrangement": "conventional",
                "x_le_h_m": float(tail.x_le_h),
                "span_h_m": float(tail.span_h),
                "c_root_h_m": float(tail.c_root_h),
                "c_tip_h_m": float(tail.c_tip_h),
                "area_h_m2": float(tail.s_h),
                "V_H": float(stab.vh),
                "elevator_chord_frac": float(style.elevator_frac),
                "x_le_v_m": float(tail.x_le_v),
                "height_v_m": float(tail.h_v),
                "c_root_v_m": float(tail.c_root_v),
                "c_tip_v_m": float(tail.c_tip_v),
                "area_v_m2": float(tail.s_v),
                "V_V": float(stab.vv),
                "rudder_chord_frac": float(style.rudder_frac),
                "incidence_h_deg": float(stab.incidence_h_deg),
            },
            "ailerons": {
                "inner_frac": float(style.aileron_inner_frac),
                "outer_frac": float(style.aileron_outer_frac),
                "chord_frac": float(style.aileron_chord_frac),
            },
            "motors": motors,
            "motor_mount": _motor_mount_spec(inp, mass, motors,
                                             material.wall_mm),
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
            "n_motors": N_MOTORS_CONV,
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
    return design


def _build_drag_conv(cl_req, cl_w, airfoil, re_mac, ar, e_osw, area, tail,
                     l_f, fus_w, fus_h, s_wet_fus, rho, v, mu):
    """CD0 build-up for a conventional airframe.

    Wing profile drag at the wing's own operating Cl; fuselage by the
    wetted-area method with Raymer's fuselage form factor
    FF = 1 + 60/f^3 + f/400 (f = fineness ratio, Raymer eq. 12.31); tail
    surfaces by the same thin-symmetric wetted-area method the flying-wing
    fins use; interference at the classic 8-10% of a full build-up (three
    real junctions: wing root and both tail roots) - contrast the flying
    wing's reduced 6%.
    """
    cd0_wing = airfoil.cd_at_cl(cl_w, re_mac)
    re_f = reynolds(rho, v, max(l_f, 0.05), mu)
    d_eq = math.sqrt(4.0 * fus_w * fus_h / math.pi)   # equivalent diameter
    f = max(l_f / max(d_eq, 1e-3), 2.0)
    ff_fus = 1.0 + 60.0 / f**3 + f / 400.0            # Raymer eq. 12.31
    cd0_fus = A.flat_plate_cf(max(re_f, 3e4)) * ff_fus * (s_wet_fus / area)
    re_t = reynolds(rho, v, max(tail.c_mac_h, 0.02), mu)
    cd0_tail = (A.flat_plate_cf(max(re_t, 3e4)) * 2.05
                * ((tail.s_h + tail.s_v) / area) * 1.25)
    subtotal = cd0_wing + cd0_fus + cd0_tail
    cd0_int = 0.09 * subtotal
    cd0 = subtotal + cd0_int
    cdi = cl_req**2 / (math.pi * ar * e_osw)          # CDi = CL^2/(pi AR e)
    return A.DragBreakdown(cd0_wing, cd0_fus, cd0_tail, cd0_int, cd0, cdi,
                           cd0 + cdi)


# ---------------------------------------------------------------------------
# Search driver
# ---------------------------------------------------------------------------

def optimize_conventional(inp: dict) -> dict:
    """Sweep (span, AR, taper, tail arm) inside the style bands and the box,
    keep the best design, polish it (Nelder-Mead) and return the full design
    dict. Mirrors optimizer.optimize_design's contract: never an exception
    for an infeasible request - the closest design is returned with its
    binding constraints listed."""
    style = _style_for(inp)

    # span is bounded by BOTH box faces: width directly, and length through
    # the ~0.66-0.84 fuselage-length fraction (the tail post must fit)
    b_hi = min(float(inp["box_w"]), float(inp["box_l"]) / 0.62, 2.6)
    b_lo = min(max(0.45, 0.55 * b_hi), 0.92 * b_hi)

    ar_lo, ar_hi = _band(inp, "ar_band_shift", style.ar_band, 3.5, 9.0)
    if inp.get("ar_target"):
        ar_lo = ar_hi = float(np.clip(inp["ar_target"], 3.5, 9.0))
    tp_lo, tp_hi = style.taper_band
    if inp.get("taper_override") is not None:
        tp_lo = tp_hi = float(np.clip(inp["taper_override"], *style.taper_band))
    lh_lo, lh_hi = CONV_LH_MAC_BAND

    spans = np.linspace(b_lo, b_hi, 5)
    ars = np.linspace(ar_lo, ar_hi, 3) if ar_hi > ar_lo + 1e-6 else np.array([ar_lo])
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
            d = evaluate_conventional(xd, inp)
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
            return evaluate_conventional(unpack(xv), inp)["cost"]
        except Exception:
            return 1e9

    try:
        r = sciopt.minimize(f, x0, method="Nelder-Mead",
                            options={"maxfev": 70, "xatol": 1e-3, "fatol": 0.4})
        polished = evaluate_conventional(unpack(r.x), inp)
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
# Five conventional characters + the "any" mode
# ---------------------------------------------------------------------------

CONV_VARIANTS: list[dict] = [
    {"key": "conv_trainer", "name": "Classic Trainer", "style": "trainer",
     "tagline": "High wing, big dihedral, nose-heavy CG - the first aeroplane.",
     "knobs": {}},
    {"key": "conv_sport", "name": "Sport Cruiser", "style": "sport",
     "tagline": "Low wing, more taper, a notch less margin - quick but honest.",
     "knobs": {}},
    {"key": "conv_aerobat", "name": "Aerobat", "style": "aerobatic",
     "tagline": "Symmetric section and a light CG - the same upright or inverted.",
     "knobs": {}},
    {"key": "conv_floater", "name": "High-Wing Floater", "style": "floater",
     "tagline": "Big camber, low loading - it lands at a walk.",
     "knobs": {"sd_weight": 1.6, "span_pref_frac": 0.95}},
    {"key": "conv_speed", "name": "Speed Runner", "style": "speed",
     "tagline": "Thin section, short span, heavier loading - it punches wind.",
     "knobs": {"span_pref_frac": 0.75, "v_cruise_mult": 1.20}},
]

_CONV_PRIMARY_FOR_MISSION = {
    "sport": "conv_sport", "fpv_cruiser": "conv_trainer",
    "thermal_floater": "conv_floater", "park_flyer": "conv_trainer",
}


def _minmax(values: list[float]) -> list[float]:
    """Min-max normalize to 0.05..1.0 (same convention as variants._minmax)."""
    lo, hi = min(values), max(values)
    if hi - lo < 1e-9:
        return [0.6] * len(values)
    return [0.05 + 0.95 * (v - lo) / (hi - lo) for v in values]


def _clamp(v: float) -> float:
    return max(0.05, min(1.0, v))


def traits_generic(designs: list[dict]) -> list[list[dict]]:
    """The four trait rows, computed from keys BOTH airplane types carry, so a
    mixed ("any"-mode) set normalizes as one comparison. Cn_beta is left out
    on purpose: a tailed model runs an order of magnitude above a wing and
    min-max mixing would just label every wing unstable (handling.py's
    calibration note); static margin is the honest shared axis."""
    sm = [d["stability"]["static_margin"] for d in designs]
    vc = [d["aero"]["v_cruise_ms"] for d in designs]
    wl = [d["aero"]["wing_loading_kgm2"] for d in designs]
    vs = [d["aero"]["v_stall_ms"] for d in designs]
    ld = [d["aero"]["ld_cruise"] for d in designs]
    n_sm, n_vc, n_wl = _minmax(sm), _minmax(vc), _minmax(wl)
    n_vs, n_ld = _minmax([-v for v in vs]), _minmax(ld)
    rows: list[list[dict]] = []
    for i, d in enumerate(designs):
        stab_detail = f"{sm[i] * 100:.0f}% static margin"
        vh = d["stability"].get("vh")
        if vh:
            stab_detail += f", V_H {vh:.2f}"
        rows.append([
            {"label": "Stability", "value": round(_clamp(n_sm[i]), 3),
             "detail": stab_detail},
            {"label": "Speed",
             "value": round(_clamp(0.6 * n_vc[i] + 0.4 * n_wl[i]), 3),
             "detail": f"{vc[i]:.1f} m/s cruise, {wl[i]:.1f} kg/m2"},
            {"label": "Slow flight", "value": round(_clamp(n_vs[i]), 3),
             "detail": f"{vs[i]:.1f} m/s stall"},
            {"label": "Efficiency", "value": round(_clamp(n_ld[i]), 3),
             "detail": f"L/D {ld[i]:.1f}"},
        ])
    return rows


def _conv_guidance(vd: dict, d: dict) -> list[dict]:
    """Builder guidance sections for a conventional design. Plain text, "- "
    bullets one per line - same rendering contract as physics/guidance.py."""
    g, a, st = d["geometry"], d["aero"], d["stability"]
    t, fus = g["tail"], g["fuselage"]
    tc = st["tail_contribution"]
    secs: list[dict] = []
    if not d.get("feasible", True):
        bad = [c["message"] for c in d["constraints"] if not c["ok"]]
        secs.append({"title": "Compromises - read this first", "body": "\n".join(
            f"- {m}" for m in bad)})
    secs.append({"title": "CG, balance and rigging", "body": "\n".join([
        f"- Balance point: {st['cg_mm_from_nose']:.0f} mm from the nose, "
        f"{st['cg_pct_mac']:.0f}% of the wing chord - a finger under each "
        f"wing panel at that station ([ND] trainer rule: 25-33% chord).",
        f"- Slide the battery to balance: the bay runs "
        f"{fus['bay']['bay_start_m'] * 1000:.0f}-"
        f"{(fus['bay']['bay_start_m'] + fus['bay']['bay_length_m']) * 1000:.0f}"
        f" mm from the nose; the design places the pack at "
        f"{g['battery_x_m'] * 1000:.0f} mm.",
        f"- Rig the stab at {t['incidence_h_deg']:+.1f} deg and the wing at "
        f"{g['root_incidence_deg']:+.1f} deg ({tc['decalage_deg']:+.1f} deg "
        f"of decalage) so the elevator sits neutral in cruise.",
    ])})
    secs.append({"title": "Control surfaces and throws", "body": "\n".join([
        f"- Elevator: {t['elevator_chord_frac'] * 100:.0f}% of the stab "
        f"chord; start at +-12 deg with 30% expo, +-18 deg high rate "
        f"(FT Simple Cub / [ND] throw practice).",
        f"- Ailerons: {g['ailerons']['chord_frac'] * 100:.0f}% chord over "
        f"the outer {100 * (g['ailerons']['outer_frac'] - g['ailerons']['inner_frac']):.0f}% "
        f"of each panel; +-12 deg low rate, 2:1 differential on a "
        f"flat-bottomed section.",
        f"- Rudder: {t['rudder_chord_frac'] * 100:.0f}% of the fin; give it "
        f"+-20 deg - rudders run bigger throws.",
    ])})
    lines = [
        f"- This is the {vd['name']}, a {d['planform_label'].lower()}: "
        f"{g['span_m'] * 1000:.0f} mm span, {fus['length_m'] * 1000:.0f} mm "
        f"fuselage ({fus['length_m'] / g['span_m']:.2f} of the span), aspect "
        f"ratio {g['aspect_ratio']:.1f}, {g['dihedral_deg']:.1f} deg of "
        f"dihedral on a {g['wing_position']} wing.",
        f"- Tail: V_H {t['V_H']:.2f} / V_V {t['V_V']:.3f} on a "
        f"{st['l_h_m'] / st['mac_m']:.1f}-chord arm - inside the "
        f"[MIT]/[ND] bands, which is why it tracks.",
        f"- Numbers: {a['wing_loading_kgm2']:.1f} kg/m2 loading, "
        f"{a['v_stall_ms']:.1f} m/s stall vs {a['v_cruise_ms']:.1f} m/s "
        f"cruise ({a['stall_margin']:.2f}x), L/D {a['ld_cruise']:.1f}, "
        f"static margin {st['static_margin'] * 100:.0f}% MAC.",
    ]
    secs.append({"title": f"Why this variant - {vd['name']}",
                 "body": "\n".join(lines)})
    return secs


def generate_conventional_variants(inp: dict) -> list[dict]:
    """Five conventional characters for one set of user requirements - the
    tailed mirror of variants.generate_variants: always exactly five, the
    mission's own style leads, infeasible characters are returned flagged."""
    primary_key = _CONV_PRIMARY_FOR_MISSION.get(
        inp.get("mission") or "sport", "conv_trainer")
    ctx: dict = {"seeds": []}
    designs: dict[str, dict] = {}

    def _run(vd: dict) -> dict:
        knobs = dict(vd["knobs"])
        v_mult = knobs.pop("v_cruise_mult", None)
        merged = {**inp, **knobs, "conv_style": vd["style"],
                  "seeds": ctx.get("seeds", [])}
        if v_mult:
            merged["v_cruise"] = inp["v_cruise"] * float(v_mult)
        ref = ctx.get("v_stall_ref", 0.0)
        if vd["key"] == "conv_floater" and ref > 0:
            merged["v_stall_goal"] = 0.85 * ref
        try:
            return optimize_conventional(merged)
        except Exception:
            d = optimize_conventional({**inp, "conv_style": vd["style"]})
            d["notes"].append(
                f"The {vd['name']} tuning could not be solved for these "
                "inputs; showing the untuned solution for this character.")
            return d

    def _seed(d: dict) -> dict:
        g = d["geometry"]
        return {"span": g["span_m"], "ar": g["aspect_ratio"],
                "taper": g["taper"],
                "lh": d["stability"]["l_h_m"] / d["stability"]["mac_m"]}

    lead_def = next(v for v in CONV_VARIANTS if v["key"] == primary_key)
    lead = _run(lead_def)
    designs[primary_key] = lead
    ctx["v_stall_ref"] = lead["aero"]["v_stall_ms"]
    ctx["seeds"].append(_seed(lead))
    for vd in CONV_VARIANTS:
        if vd["key"] == primary_key:
            continue
        d = _run(vd)
        designs[vd["key"]] = d
        ctx["seeds"].append(_seed(d))

    ordered = [designs[vd["key"]] for vd in CONV_VARIANTS]
    traits = traits_generic(ordered)
    out: list[dict] = []
    for vd, d, tr in zip(CONV_VARIANTS, ordered, traits):
        d["character"] = {"key": vd["key"], "name": vd["name"],
                          "tagline": vd["tagline"]}
        d["guidance"] = _conv_guidance(vd, d)
        out.append({
            "id": d["id"],
            "key": vd["key"],
            "name": vd["name"],
            "tagline": vd["tagline"],
            "planform": d["planform"],
            "airplane_type": "conventional",
            "primary": vd["key"] == primary_key,
            "traits": tr,
            "design": d,
        })
    return out


def generate_any_variants(inp: dict, fw_generate) -> list[dict]:
    """The "any" mode: run BOTH types' candidate sets, return the best five by
    the shared objective, guaranteed to span both types, each dict carrying
    its own airplane_type (V2_PLAN.md)."""
    fw = fw_generate({**inp, "airplane_type": "flying_wing"})
    for v in fw:
        v["airplane_type"] = "flying_wing"
        v["design"]["airplane_type"] = "flying_wing"
    conv = generate_conventional_variants({**inp, "airplane_type":
                                           "conventional"})
    pool = sorted(fw + conv,
                  key=lambda v: (not v["design"]["feasible"],
                                 v["design"]["cost"]))
    selected = pool[:5]
    # spanning guarantee: if the top five are all one type, the best candidate
    # of the missing type replaces the worst selected one
    types = {v["airplane_type"] for v in selected}
    if len(types) < 2:
        missing = ({"flying_wing", "conventional"} - types).pop()
        cand = next((v for v in pool if v["airplane_type"] == missing), None)
        if cand is not None:
            selected[-1] = cand
    # one primary: the best of the merged set
    for v in selected:
        v["primary"] = False
    selected[0]["primary"] = True
    # traits re-normalized across the DISPLAYED set, whatever set each design
    # was first scored inside
    traits = traits_generic([v["design"] for v in selected])
    for v, tr in zip(selected, traits):
        v["traits"] = tr
    return selected
