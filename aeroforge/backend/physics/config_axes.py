"""The four CONFIGURATION AXES as pure transforms (V3_PLAN.md).

v3 exposes four configuration choices that cut ACROSS airplane types:

    tail type       conventional | t_tail | v_tail
    wing position   high | mid | low
    motor layout    tractor | pusher
    motor count     0 | 1 | 2

Each axis is implemented here as a PURE function - dict in, dict out, no
module state, no imports from the frozen stability solvers (`stability.py` /
`stability_conv.py` never import this module and it never imports them:
V3_PLAN.md fence). Type modules and the integration wave consume these
transforms; every constant cites RESEARCH_TYPES_V3.md (section tags in
comments, e.g. [RT3 s.9] = RESEARCH_TYPES_V3.md section 9).
"""
from __future__ import annotations

import math

# ---------------------------------------------------------------------------
# Axis 1: tail type ([RT3 s.9], quick-reference rows 49-51)
# ---------------------------------------------------------------------------

# V-tail dihedral band, degrees from horizontal. Derived in [RT3 s.9] from the
# app's own V_H/V_V bands (S_v/S_h ~ 0.35-0.55 -> A = arctan sqrt(ratio) =
# 30.6-36.6 deg); NACA Report 823 (Purser & Campbell) verifies the simplified
# vee-tail theory "for dihedral angles up to about 40 deg", which caps the band.
V_TAIL_DIHEDRAL_BAND_DEG: tuple[float, float] = (30.0, 40.0)
V_TAIL_DIHEDRAL_DEFAULT_DEG: float = 35.0

# Ruddervator throw budget: each panel carries elevator AND rudder commands
# summed, so budget +-(elevator + rudder/2) ~ +-25 deg total [RT3 s.9].
RUDDERVATOR_THROW_BUDGET_DEG: float = 25.0

# T-tail: "T-tails can be aerodynamically cleaner but structurally heavier"
# (SKYbrary, T-tail) - the fin carries the stab's loads and mass, so the fin
# structure mass is factored x1.3-1.5; x1.4 is the derived mid-value
# ([RT3 s.9], quick-reference row 51).
T_TAIL_FIN_MASS_FACTOR: float = 1.4
T_TAIL_FIN_MASS_FACTOR_BAND: tuple[float, float] = (1.3, 1.5)


def apply_tail_type(tail_geom: dict, tail_type: str) -> dict:
    """Transform a conventional-tail geometry block to the requested tail type.

    `tail_geom` must carry at least `area_h_m2` and `area_v_m2` (the required
    horizontal and vertical areas the volume coefficients produced - V_H/V_V
    bands are properties of the AIRFRAME, not the tail shape, so they are
    unchanged across tail types [RT3 s.9]). The input dict is never mutated.

    "v_tail" - the actual sizing rule, Model Aviation "Tails for Models"
    [MA-T via RT3 s.9]: total V-tail area equals the SUM of the required
    areas, S = S_h + S_v, mounted at dihedral A = arctan(sqrt(S_v/S_h));
    the effective areas obey the tan^2 split, S_Heff = S cos^2 A and
    S_Veff = S sin^2 A. The documented undersizing trap: "The effective
    areas are not the projected areas (which would use the sine and cosine
    directly). If projected areas were used to size the V-tail, it would be
    too small" [MA-T]. Sized by the sum rule the V-tail "provides
    essentially the same stability characteristics as a conventional tail"
    - no extra fudge factor is applied.

    "t_tail" - stab atop the fin: fin structure mass x1.4 (derived
    mid-value [RT3 s.9]), stab mass moved to fin-tip height for CG/inertia
    bookkeeping, deep-stall guidance flag (SKYbrary: separated wing flow can
    blanket a T-tail's elevator at high alpha), and the elevator pushrod is
    routed UP the fin through the same round Ø8.25 pipe doctrine
    (V3_PLAN.md).

    "conventional" - pass-through with the type recorded.
    """
    s_h = float(tail_geom["area_h_m2"])
    s_v = float(tail_geom["area_v_m2"])
    out = dict(tail_geom)

    if tail_type == "v_tail":
        s_total = s_h + s_v                      # S = S_h + S_v [MA-T]
        ratio = s_v / max(s_h, 1e-9)
        dihedral = math.degrees(math.atan(math.sqrt(max(ratio, 0.0))))
        dihedral_clipped = min(max(dihedral, V_TAIL_DIHEDRAL_BAND_DEG[0]),
                               V_TAIL_DIHEDRAL_BAND_DEG[1])
        a_rad = math.radians(dihedral_clipped)
        # tan^2 split: the areas the mounted V actually provides [MA-T]
        s_h_eff = s_total * math.cos(a_rad) ** 2
        s_v_eff = s_total * math.sin(a_rad) ** 2
        notes = [
            "V-tail sized by the SUM rule S = S_h + S_v at A = "
            f"arctan sqrt(S_v/S_h) = {dihedral:.1f} deg "
            "(Model Aviation 'Tails for Models'; NACA Report 823 verifies the "
            "theory to ~40 deg of dihedral). Do NOT size from projected "
            "areas - that is the documented undersizing trap.",
            "Ruddervator mixing: elevator and rudder commands sum on the two "
            f"panels; budget +-{RUDDERVATOR_THROW_BUDGET_DEG:.0f} deg of "
            "total throw per panel (elevator + rudder/2) and expect a slight "
            "roll with yaw input (the documented V-tail coupling).",
        ]
        if abs(dihedral_clipped - dihedral) > 1e-9:
            notes.append(
                f"Geometric dihedral {dihedral:.1f} deg clipped into the "
                f"{V_TAIL_DIHEDRAL_BAND_DEG[0]:.0f}-"
                f"{V_TAIL_DIHEDRAL_BAND_DEG[1]:.0f} deg band ([NACA823] "
                "validity limit / RC practice floor); effective areas quoted "
                "at the clipped angle.")
        out.update({
            "type": "v_tail",
            "dihedral_deg": float(dihedral_clipped),
            "dihedral_unclipped_deg": float(dihedral),
            "area_total_m2": float(s_total),
            "area_panel_m2": float(0.5 * s_total),   # two panels
            "area_h_eff_m2": float(s_h_eff),
            "area_v_eff_m2": float(s_v_eff),
            "ruddervator_throw_budget_deg": RUDDERVATOR_THROW_BUDGET_DEG,
            "notes": notes,
        })
        return out

    if tail_type == "t_tail":
        out.update({
            "type": "t_tail",
            # the stab rides at the fin tip: its mass moment arm in z is the
            # fin height - the caller's CG/inertia bookkeeping must use it
            "stab_z_mode": "fin_tip",
            "fin_mass_factor": T_TAIL_FIN_MASS_FACTOR,
            "deep_stall_flag": True,
            "notes": [
                f"T-tail: fin structure mass x{T_TAIL_FIN_MASS_FACTOR:.1f} "
                "(reinforced fin + stab joint, derived mid-value of the "
                "x1.3-1.5 band; SKYbrary: 'aerodynamically cleaner but "
                "structurally heavier') and the stab mass is carried at "
                "fin-tip height in the CG/inertia bookkeeping.",
                "DEEP STALL: at high angle of attack the wing's separated "
                "flow can blanket the elevator (SKYbrary, Deep Stall) - "
                "avoid aft-CG + full-stall flight tests; recover with power "
                "and forward stick early.",
                "Elevator pushrod routes UP the fin through the standard "
                "round Ø8.25 mm pipe with trumpeted mouths (V3_PLAN.md - "
                "same wire/pushrod doctrine as every other run).",
                "What it buys: the stab is out of the prop wash and clear of "
                "grass on belly landings - the RC reasons T-tails exist on "
                "gliders.",
            ],
        })
        return out

    if tail_type == "conventional":
        out.update({"type": "conventional", "notes": []})
        return out

    raise ValueError(f"unknown tail_type {tail_type!r}; "
                     "options: conventional, t_tail, v_tail")


# ---------------------------------------------------------------------------
# Axis 2: wing position ([RT3 s.10], quick-reference rows 52-54)
# ---------------------------------------------------------------------------

# Effective-dihedral increment contributed by the wing position (fuselage
# crossflow / keel effect), in degrees of equivalent dihedral. The recorded
# spread is +-1 deg (RC ladder) to +-2.5 deg (Aerospaceweb "about 5 deg of
# effective dihedral" high-over-low); +-1.5 deg is the recommended value
# inside it [RT3 s.10.1, row 52].
WING_POS_EFF_DIHEDRAL_DEG: dict[str, float] = {
    "high": +1.5, "mid": 0.0, "low": -1.5,
}
# Geometric-dihedral defaults per position - the RC practice ladder "high
# wing 1 deg, mid 2 deg, low 3 deg" (RCU dihedral thread via [RT3 s.10.1],
# row 53), with the high position given its 1-2 deg band.
WING_POS_GEOM_DIHEDRAL_DEG: dict[str, float] = {
    "high": 1.5, "mid": 2.0, "low": 3.0,
}
WING_POS_GEOM_BAND_DEG: dict[str, tuple[float, float]] = {
    "high": (1.0, 2.0), "mid": (2.0, 2.0), "low": (3.0, 3.0),
}
# Sweep <-> dihedral cross-term: 2-3 deg sweep per 1 deg dihedral at high CL
# (RC Kavala) vs 10 deg per 1 deg at low CL (Aerospaceweb) - the spread
# exists because Cl_beta,sweep ~ CL tan(sweep). 4 deg of sweep per 1 deg of
# dihedral is the derived mid-value at this app's cruise CLs [RT3 s.10.1,
# row 54].
SWEEP_DEG_PER_DIHEDRAL_DEG: float = 4.0


def apply_wing_position(position: str) -> dict:
    """Wing position -> effective-dihedral increment + geometric default.

    Returns the geometric dihedral the position defaults to (the RC ladder
    [RT3 s.10.1]) and the effective-dihedral increment the position itself
    contributes to Cl_beta - so the SUMMED dihedral effect lands in the same
    place for every position: high 1.5 geo + 1.5 eff = mid 2.0 + 0 =
    low 3.0 - 1.5 (within half a degree, which is the point of the ladder).
    """
    if position not in WING_POS_EFF_DIHEDRAL_DEG:
        raise ValueError(f"unknown wing position {position!r}; "
                         "options: high, mid, low")
    eff = WING_POS_EFF_DIHEDRAL_DEG[position]
    geo = WING_POS_GEOM_DIHEDRAL_DEG[position]
    handling = {
        "high": ("Most stable: pendulum + fuselage keel effect; payload low "
                 "in the pod - the trainer/FPV default [RT3 s.10.2]."),
        "mid": ("Neutral roll coupling, cleanest interference drag - the "
                "aerobatic choice [RT3 s.10.2]."),
        "low": ("Least effective dihedral (hence the +3 deg geometric); "
                "belly-lander rule: no servo horn, hatch lip or conduit "
                "mouth on the lower wing surface inboard of 40% semi-span "
                "[RT3 s.10.3]."),
    }[position]
    return {
        "wing_position": position,
        "geometric_dihedral_deg": float(geo),
        "geometric_dihedral_band_deg": list(WING_POS_GEOM_BAND_DEG[position]),
        "effective_dihedral_increment_deg": float(eff),
        # total dihedral EFFECT the position + default rigging delivers
        "total_effective_dihedral_deg": float(geo + eff),
        "sweep_equiv_deg_per_dihedral_deg": SWEEP_DEG_PER_DIHEDRAL_DEG,
        "notes": [
            f"Wing position '{position}': {geo:.1f} deg geometric dihedral "
            f"(RC ladder high 1-2 / mid 2 / low 3 [RT3 s.10.1]) plus "
            f"{eff:+.1f} deg of position-derived effective dihedral "
            "(recommended value inside the recorded +-1..+-2.5 deg spread), "
            f"so the net dihedral effect is ~{geo + eff:.1f} deg-equivalent "
            "whichever position is chosen.",
            handling,
        ],
    }


# ---------------------------------------------------------------------------
# Axis 3+4: motor layout and count ([RT3 s.7, s.8], rows 45-48)
# ---------------------------------------------------------------------------

# Pusher efficiency penalty: the prop ingests the airframe wake; literature
# spread 2-15% depending on how much dirty air the disk swallows. Derived
# mid-band picks [RT3 s.7, row 45]: 5% for a pod pusher (disk right behind
# the body), 3% for a clean twin-boom pusher (disk mostly outside the pod
# wake). The app carries no thrust model, so this lands in guidance and
# L/D notes, never in the drag build-up.
PUSHER_EFF_PENALTY: dict[str, float] = {"pod": 0.05, "twin_boom": 0.03}
# Tractor thrustline: 2-3 deg of down + right thrust ([ND] p.6 via
# [RT3 s.7, row 46]); 2 deg is the default.
TRACTOR_DOWNTHRUST_DEG: float = 2.0
TRACTOR_RIGHTTHRUST_DEG: float = 2.0
# Twin nacelles: 25-35% semi-span, default 30% (NASA TM X-3207 at ~32%,
# NASA interference-drag study at ~35%; inboard of 25% the disk hits the
# pod clearance, outboard of 35% the engine-out yaw grows for nothing)
# [RT3 s.8, row 47].
NACELLE_SEMISPAN_FRAC_BAND: tuple[float, float] = (0.25, 0.35)
NACELLE_SEMISPAN_FRAC_DEFAULT: float = 0.30
# Engine-out check speed: 1.2 x V_stall (the RC Vmc analogue [RT3 s.8]).
ENGINE_OUT_SPEED_FACTOR: float = 1.2
# One motor's static thrust ~ AUW/2: RC twins are set up around unity
# thrust-to-weight split across two motors (derived rule [RT3 s.8, row 48]).
TWIN_STATIC_THRUST_AUW_FRAC: float = 0.5
# Usable fin/rudder side-force coefficient for the engine-out moment: a
# symmetric fin with a 40-50% rudder at full throw reaches CL ~ 0.8 before
# the surface stalls (plain-flap limit, Etkin & Reid class numbers).
FIN_CL_MAX_ENGINE_OUT: float = 0.8

G = 9.80665


def apply_motor_layout(motor_layout: str, n_motors: int, *,
                       pusher_context: str = "pod") -> dict:
    """Motor layout + count -> thrustline / efficiency / CG bookkeeping.

    Pure transform: returns the layout's documented consequences for the
    type module to fold into its own solve. `pusher_context` picks the
    pusher penalty row ("pod" 5% or "twin_boom" 3% [RT3 s.7]).
    """
    if motor_layout not in ("tractor", "pusher"):
        raise ValueError(f"unknown motor_layout {motor_layout!r}; "
                         "options: tractor, pusher")
    n = int(n_motors)
    if n not in (0, 1, 2):
        raise ValueError("n_motors must be 0, 1 or 2 (V3_PLAN.md; 0 is "
                         "legal only for the glider type)")
    out: dict = {
        "motor_layout": motor_layout,
        "n_motors": n,
        "notes": [],
    }
    if n == 0:
        out.update({
            "efficiency_penalty": 0.0,
            "thrustline": None,
        })
        out["notes"].append(
            "No motor: no mount, no motor wiring; the power-system mass "
            "becomes nose ballast + a smaller RX pack at the same station "
            "so the CG closure is unchanged (glider rule, [RT3 s.6.2]).")
        return out

    if motor_layout == "tractor":
        out.update({
            "efficiency_penalty": 0.0,
            "thrustline": {
                "down_deg": TRACTOR_DOWNTHRUST_DEG,
                "right_deg": TRACTOR_RIGHTTHRUST_DEG,
            },
        })
        out["notes"].append(
            f"Tractor thrustline: {TRACTOR_DOWNTHRUST_DEG:.0f} deg down / "
            f"{TRACTOR_RIGHTTHRUST_DEG:.0f} deg right thrust ([ND] 2-3 deg "
            "practice).")
        out["notes"].append(
            "Belly-lander prop rule: a tractor prop survives landings only "
            "via nose-up flare geometry - the tip circle must clear the "
            "belly line by >= 10 deg of rotation about the TE touch point "
            "(derived geometric rule, [RT3 s.7]).")
    else:
        penalty = PUSHER_EFF_PENALTY.get(pusher_context,
                                         PUSHER_EFF_PENALTY["pod"])
        out.update({
            "efficiency_penalty": float(penalty),
            "thrustline": {"rule": "through_cg_parallel_to_chord"},
        })
        out["notes"].append(
            f"Pusher: ~{penalty * 100:.0f}% cruise thrust/efficiency "
            "penalty from wake ingestion (literature spread 2-15%; derived "
            f"mid-band pick for a {pusher_context.replace('_', '-')} "
            "installation [RT3 s.7]). Guidance-level only - the app sizes "
            "no thrust.")
        out["notes"].append(
            "Pusher thrustline: pass the thrust axis through the vertical "
            "CG and keep it parallel to the (rear) wing chord, or power "
            "changes pitch the aircraft ([LEN-CAN] s.6 item 5); a high pod "
            "that cannot reach the CG line takes up-thrust instead - flag "
            "it, never silently accept it.")

    if n == 2:
        out.update({
            "nacelle_semispan_frac": NACELLE_SEMISPAN_FRAC_DEFAULT,
            "nacelle_semispan_frac_band": list(NACELLE_SEMISPAN_FRAC_BAND),
            # the lumped power mass splits to the two nacelle stations; the
            # CG moves AFT (nacelles sit at the wing, behind the nose
            # station a single motor occupies), so the battery slides
            # FORWARD in the bay to close the same CG [RT3 s.8]
            "mass_cg_rule": "power_mass_split_to_nacelles_battery_forward",
        })
        out["notes"].append(
            f"Twin: nacelles at {NACELLE_SEMISPAN_FRAC_DEFAULT * 100:.0f}% "
            "semi-span (band 25-35%, NASA TM X-3207 / interference-drag "
            "study [RT3 s.8]); the power mass splits to the nacelle "
            "stations, the CG moves aft, and the battery slides forward in "
            "the bay to close it. Roll inertia rises - expect slower roll "
            "starts.")
        out["notes"].append(
            "Engine-out: check the fin yaw moment at 1.2 x V_stall against "
            "one motor at the nacelle arm (engine_out_check); failing "
            "that, counter-rotating props + differential-thrust mixing are "
            "mandatory guidance (standard FPV twin practice [RT3 s.8]).")
    return out


def engine_out_check(*, mass_kg: float, v_stall_ms: float, rho: float,
                     s_v_m2: float, l_v_m: float, span_m: float,
                     nacelle_semispan_frac: float =
                     NACELLE_SEMISPAN_FRAC_DEFAULT) -> dict:
    """Twin-motor engine-out yaw check at 1.2 x V_stall ([RT3 s.8, row 48]).

    The RC failure case mirrors full-scale Vmc: one motor at full thrust at
    low airspeed. Derived rule (the app sizes no thrust): the fin + rudder
    must generate more yaw moment at 1.2 x V_stall than one motor at the
    nacelle arm produces at static thrust ~ AUW/2:

        N_fin   = q(1.2 V_s) S_v CL_v,max l_v      (fin at full rudder)
        N_motor = (0.5 m g) (f_nac b/2)            (one motor, static)

    ok requires N_fin >= N_motor; otherwise guidance mandates
    counter-rotating props and differential-thrust mixing.
    """
    v = ENGINE_OUT_SPEED_FACTOR * float(v_stall_ms)
    q = 0.5 * float(rho) * v * v                       # q = rho V^2 / 2
    n_fin = q * float(s_v_m2) * FIN_CL_MAX_ENGINE_OUT * float(l_v_m)
    y_nac = float(nacelle_semispan_frac) * 0.5 * float(span_m)
    thrust_one = TWIN_STATIC_THRUST_AUW_FRAC * float(mass_kg) * G
    n_motor = thrust_one * y_nac
    ok = bool(n_fin >= n_motor)
    return {
        "check_speed_ms": float(v),
        "n_fin_nm": float(n_fin),
        "n_motor_nm": float(n_motor),
        "margin": float(n_fin / max(n_motor, 1e-9)),
        "ok": ok,
        "note": ("Fin holds the engine-out yaw at 1.2 V_stall." if ok else
                 "Fin CANNOT hold one-motor-out yaw at 1.2 V_stall: "
                 "counter-rotating props and differential-thrust mixing "
                 "are mandatory (guidance rule, [RT3 s.8])."),
    }
