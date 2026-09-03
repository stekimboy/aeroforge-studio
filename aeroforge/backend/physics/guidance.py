"""Expert builder guidance for FLYING WINGS, generated from a finished design.

`build_guidance(design)` turns the numeric design dict (SPEC_FLYING_WING.md §4)
into a list of plain-text sections [{"title", "body"}] with the design's own
numbers interpolated - CG in mm and % MAC, elevon chords and throws, bay size,
spar station, approach speed. The frontend renders these verbatim: bodies are
plain text, bullet lines start with "- " and must each be ONE line, and blank
lines separate paragraphs.

AeroForge designs tailless AIRFRAMES only. There are no tails, canards, V-tails,
booms, rudders or elevators anywhere in this product, and no propulsion sizing:
motors survive as count + position, and the only power-system number quoted is
the lumped mass allowance the weight model assumed (physics/weights.py), because
the builder needs to know the budget their real hardware must fit inside.

Every lookup is guarded - a missing key must never raise, because a raising
`build_guidance` blanks the whole Design Notes panel.

`LEARN_ARTICLES` is the static knowledge base served by GET /api/learn.
"""
from __future__ import annotations

import math

# ---------------------------------------------------------------------------
# safe accessors / formatting helpers
# ---------------------------------------------------------------------------

_PLANFORM_LABELS = {
    "swept": "swept sport wing",
    "bwb": "blended wing body",
    "plank": "plank",
    "bell": "bell-distribution (Horten-type) wing",
}

_MISSION_LABELS = {
    "sport": "sport flyer",
    "fpv_cruiser": "FPV cruiser",
    "thermal_floater": "thermal floater",
    "park_flyer": "park flyer",
}

_VSTAB_LABELS = {
    "winglets": "tip winglets",
    "twin_fin": "inboard twin fins",
    "center_fin": "centre fin",
    "none": "no vertical surfaces",
}


def _bool(value) -> bool:
    return bool(value) if isinstance(value, (bool, int, float)) else False


def _num(value, default: float = 0.0) -> float:
    """float(value) if it is a finite real number, else `default`.

    Guards the whole module against None / NaN / strings leaking into text.
    """
    try:
        out = float(value)
    except (TypeError, ValueError):
        return float(default)
    if not math.isfinite(out):
        return float(default)
    return out


def _human(x: float) -> str:
    """Readable number for a constraint report - never scientific notation."""
    a = abs(x)
    if a >= 10000.0:
        return f"{x:,.0f}"
    if a >= 100.0:
        return f"{x:.0f}"
    if a >= 10.0:
        out = f"{x:.1f}"
    elif a >= 1.0:
        out = f"{x:.2f}"
    else:
        out = f"{x:.3f}"
    return out.rstrip("0").rstrip(".") if "." in out else out


def _sub(design, key: str) -> dict:
    """Fetch a sub-dict, always returning a dict."""
    val = design.get(key) if isinstance(design, dict) else None
    return val if isinstance(val, dict) else {}


def _text(value, default: str) -> str:
    if isinstance(value, str) and value.strip():
        return value.strip()
    return default


def _para(lines) -> str:
    """Join bullet/paragraph lines into one block (each entry stays one line)."""
    return "\n".join(str(x) for x in lines if x)


def _body(*blocks: str) -> str:
    return "\n\n".join(b for b in blocks if b)


def _thickness_ratio(airfoil: str) -> float:
    """t/c parsed out of the section name, 0.0 if it cannot be read.

    Handles the reflex library ("RFX-9 reflexed" -> 0.09, "RFX-11" -> 0.11) and
    NACA 4-digit names ("NACA 0008" -> 0.08) so the bay-depth arithmetic never
    invents a thickness.
    """
    name = (airfoil or "").upper()
    if "RFX" in name:
        digits = ""
        after = name.split("RFX", 1)[1]
        for ch in after:
            if ch.isdigit():
                digits += ch
            elif digits:
                break
        if digits:
            val = int(digits) / 100.0
            if 0.03 <= val <= 0.25:
                return val
    parts = [p for p in name.replace("-", " ").split() if p.isdigit()]
    for p in parts:
        if len(p) == 4:
            val = int(p[2:]) / 100.0
            if 0.03 <= val <= 0.25:
                return val
    return 0.0


def _smoothstep_blend(f: float, fb: float) -> float:
    """Body blend weight w(f): 1 at the centreline, 0 at/outboard of fb.

    Mirrors SPEC_FLYING_WING.md §5 so quoted chords match the CAD.
    """
    if fb <= 1e-9:
        return 0.0
    u = min(max(f / fb, 0.0), 1.0)
    return 1.0 - (3.0 * u * u - 2.0 * u * u * u)


def _chord_at(design: dict, f: float) -> float:
    """Local chord (m) at semi-span fraction f, including the centre-body blend."""
    geo = _sub(design, "geometry")
    body = _sub(geo, "body")
    c_root = _num(geo.get("root_chord_m"), 0.0)
    c_tip = _num(geo.get("tip_chord_m"), c_root)
    span = _num(geo.get("span_m"), 0.0)
    f = min(max(_num(f, 0.0), 0.0), 1.0)
    c_wing = c_root + (c_tip - c_root) * f
    semi = 0.5 * span
    fb = (_num(body.get("half_width_m"), 0.0) / semi) if semi > 1e-9 else 0.0
    scale = _num(body.get("chord_scale"), 1.0)
    return c_wing * (1.0 + (scale - 1.0) * _smoothstep_blend(f, fb))


# ---------------------------------------------------------------------------
# 1. design compromises (infeasible designs only)
# ---------------------------------------------------------------------------

_FIXES = {
    # envelope
    "box_span": "Widen the box, or accept a smaller wing that flies faster.",
    "box_length": "The centre body sets the length on a wing - a longer box, less body chord scale or less sweep buys it back.",
    "box_height": "Raise the box a little: the fins or winglets usually set this, and a bell design (no fins) sidesteps it entirely.",
    "box_fit": "The airframe does not fit the box you gave it - enlarge the box or reduce the span target.",
    # flight envelope
    "stall_speed": "A slower stall needs more wing area or less weight: a bigger box, or a lighter build method.",
    "stall_margin": "Cruise is too close to the stall - raise the cruise speed or give the wing more area.",
    "wing_loading": "Box size and cruise speed are fighting each other: enlarge the box, slow the cruise, or pick a mission that tolerates a firmer loading.",
    "cl_max": "The wing is being asked for more lift coefficient than a reflexed section can give - reflex costs CLmax, so add area rather than angle of attack.",
    # balance
    "static_margin": "Wings fly at 3-15% MAC. Move the pack, or let the optimizer use a slightly different sweep so the neutral point lands where the mass can balance it.",
    "pitch_stability": "The CG ended up at or behind the neutral point. On a tailless model that is not 'twitchy', it is unflyable - do not build this one.",
    "cg_range": "The mass cannot be arranged to reach the required balance point - shorten the centre body, move the pack bay forward, or reduce sweep so the neutral point comes forward.",
    "battery_position": "The flight pack cannot slide far enough forward to balance the wing. A longer equipment bay, a forward bay, or less nose-light structure fixes it.",
    # planform / proportions
    "reynolds": "The chords came out too small for the air to behave. A bigger box or a lower aspect-ratio target gives fatter, better-behaved chords.",
    "aspect_ratio": "The aspect ratio the box allows is outside this planform's band - change the box shape or pick a planform whose band fits.",
    "root_chord": "The centre chord is too short to BE the fuselage. Reduce the span target or accept more chord: a wing with a bolted-on pod is not what this app builds.",
    "body_depth": "The centre body is too shallow to hold a pack - more body depth scale, more root chord, or a thicker section.",
    "bay_depth": "The equipment bay is too shallow for a realistic pack. Deepen the body or lengthen the root chord.",
    "washout": "The twist needed to trim is outside the band for this planform - more sweep gives the washout more leverage, or move to a reflexed plank.",
    "vstab_area": "The vertical surfaces cannot be sized inside the box - lower fins with more chord, or a planform that uses winglets instead.",
    "trim": "The section reflex plus washout cannot trim this lift coefficient - more reflex, more washout, or a lower design CL (more area).",
}


def _sec_compromises(design: dict) -> dict:
    cons = design.get("constraints")
    cons = cons if isinstance(cons, list) else []
    failed = [c for c in cons if isinstance(c, dict) and not c.get("ok", True)]
    if not failed:
        names = design.get("binding")
        names = names if isinstance(names, list) else []
        failed = [{"name": str(n)} for n in names]

    intro = ("This is the closest the optimizer could get inside your limits, and "
             "it did NOT meet every requirement. Read this before cutting anything.")

    bullets = []
    for c in failed:
        name = _text(c.get("name"), "constraint")
        msg = _text(c.get("message"), "outside its allowed band")
        fix = _FIXES.get(name, "Relax the input that drives this constraint and regenerate.")
        got, lim = c.get("value"), c.get("limit")
        detail = ""
        if got is not None and lim is not None:
            g, l = _num(got, float("nan")), _num(lim, float("nan"))
            if math.isfinite(g) and math.isfinite(l):
                detail = f" (got {_human(g)}, needed {_human(l)})"
        label = name.replace("_", " ").capitalize()
        bullets.append(f"- {label}: {msg}{detail}. Fix: {fix}")

    closing = ("On a flying wing the two inputs that unlock almost everything are a "
               "bigger design box and a lower cruise speed - both give the wing area "
               "and chord to work with. Change one, regenerate, and watch this "
               "section shrink. If 'pitch stability' is in the list above, treat the "
               "design as a study model only: do not fly it.")
    return {"title": "Design compromises - read this first",
            "body": _body(intro, _para(bullets), closing)}


# ---------------------------------------------------------------------------
# 2. balance & CG - the dominant flying-wing topic
# ---------------------------------------------------------------------------

def _sec_balance(design: dict) -> dict:
    geo = _sub(design, "geometry")
    st = _sub(design, "stability")
    mass = _sub(design, "mass")
    pwr = _sub(design, "power_system")

    mac_m = _num(st.get("mac_m"), 0.0)
    x_le_mac = _num(st.get("x_le_mac_m"), 0.0)
    x_cg = _num(st.get("x_cg_m"), 0.0)
    cg_mm = _num(st.get("cg_mm_from_nose"), x_cg * 1000.0)
    cg_pct = _num(st.get("cg_pct_mac"), 0.0)
    sm = _num(st.get("static_margin"), 0.0)
    sm_pct = sm * 100.0
    x_np_mm = _num(st.get("x_np_m"), 0.0) * 1000.0
    y_mac_mm = _num(st.get("y_mac_m"), 0.0) * 1000.0
    cg_behind_le_mm = (x_cg - x_le_mac) * 1000.0
    np_behind_cg_mm = max(x_np_mm - cg_mm, 0.0)
    mac_mm = mac_m * 1000.0

    length_mm = _num(geo.get("length_total_m"), 0.0) * 1000.0
    batt_mm = _num(geo.get("battery_x_m"), 0.0) * 1000.0
    batt_req_mm = _num(geo.get("battery_x_required_m"), batt_mm / 1000.0) * 1000.0
    total_kg = _num(mass.get("total_kg"), 0.0)
    pack_kg = _num(pwr.get("pack_mass_kg"), 0.0)
    pack_pct = 100.0 * pack_kg / total_kg if total_kg > 1e-6 else 0.0

    bullets = [
        f"- Balance point: {cg_mm:.0f} mm behind the nose datum, measured along the "
        f"centreline. That is {cg_pct:.1f}% of the mean aerodynamic chord "
        f"(MAC = {mac_mm:.0f} mm), and it is the single number this whole design "
        "is built around.",
    ]
    if mac_mm > 1.0 and cg_behind_le_mm > 0.0:
        bullets.append(
            f"- On the wing itself the CG sits {cg_behind_le_mm:.0f} mm behind the "
            f"leading edge at the MAC station ({y_mac_mm:.0f} mm out from the "
            "centreline). Mark it there on BOTH panels, underside, with a pen line "
            "you can feel - a swept wing's CG line is not square across the model.")
    if length_mm > 1.0:
        bullets.append(
            f"- As a sanity check while building: the CG is {100.0 * cg_mm / length_mm:.0f}% "
            f"of the way back along the {length_mm:.0f} mm overall length. If your "
            "finished model balances a long way from that, something is heavier or "
            "further aft than the design assumed - find it before you fly.")
    if np_behind_cg_mm > 0.0:
        bullets.append(
            f"- The neutral point is at {x_np_mm:.0f} mm, i.e. only {np_behind_cg_mm:.0f} mm "
            f"behind the CG ({sm_pct:.1f}% MAC static margin). Everything aft of the "
            "CG line by that distance is where the model stops being an aircraft. "
            "Read that as your real build tolerance.")

    bullets.append(
        "- Why a wing is unforgiving: a tailed model damps pitch with a surface on a "
        "long arm, so an aft CG makes it twitchy and porpoise-y but usually still "
        "flyable home. A tailless model has almost no pitch damping - the only thing "
        "resisting a pitch rate is the wing itself, over a chord's length - so behind "
        "the neutral point it diverges faster than you can react. Tail-heavy on a "
        "wing is not 'sporty'. It is unflyable, and the crash happens in the first "
        "three seconds off the hand.")
    bullets.append(
        "- Worse, the recovery control is the wrong one: pulling up-elevon on a wing "
        "adds nose-up moment by REMOVING lift from the trailing edge. A wing that is "
        "diverging nose-up and being fought with up-elevon is also sinking. There is "
        "no elevator on a boom to save you.")
    bullets.append(
        "- Nose-heavy, by contrast, is merely expensive: permanent up-elevon trim, "
        "extra trim drag, a flatter glide and a fast landing. Always err forward. "
        "Maiden 3-5 mm AHEAD of the marked CG and walk it back a couple of "
        "millimetres per flight until the pitch response is crisp but not nervous.")

    if sm_pct < 4.0:
        sm_note = (f"- {sm_pct:.1f}% MAC is a genuinely lively margin, at the sharp end "
                   "of the 3-15% band tailless models use. It will feel immediate in "
                   "pitch and it will not tolerate a pack sliding 5 mm aft in flight. "
                   "Strap the battery properly and check the balance every session.")
    elif sm_pct < 8.0:
        sm_note = (f"- {sm_pct:.1f}% MAC is the normal sport-wing margin (wings live at "
                   "3-15%, well below the 10-20% a trainer with a tail would use, "
                   "because the reflex/washout that trims a wing costs lift and a big "
                   "margin would make it fly permanently nose-down and fast).")
    elif sm_pct <= 12.0:
        sm_note = (f"- {sm_pct:.1f}% MAC is a relaxed, camera-platform margin: steady "
                   "hands-off, a little more up-trim needed, and a slightly faster "
                   "trimmed cruise than a sharper wing of the same size.")
    else:
        sm_note = (f"- {sm_pct:.1f}% MAC is at the stable end of what a wing can use. "
                   "Expect noticeable up-elevon trim, a fast trimmed speed and a "
                   "hot landing. If it feels sluggish, move the pack back 3 mm at a "
                   "time - never more than that in one step.")
    bullets.append(sm_note)

    if batt_mm > 1.0:
        slop = abs(batt_mm - batt_req_mm)
        bullets.append(
            f"- The pack is your balance tool and the design puts it at {batt_mm:.0f} mm "
            "from the nose. Fit it on velcro AND a strap, mark its position on the bay "
            "floor, and slide it in 5 mm steps to trim the balance rather than adding "
            "lead.")
        if slop > 2.0:
            bullets.append(
                f"- Note the pack has to sit within about {slop:.0f} mm of that station "
                "to hold the balance - build the bay so the pack cannot creep aft under "
                "launch acceleration, which is exactly when it would do it.")
    if pack_kg > 0.0:
        bullets.append(
            f"- The {total_kg * 1000.0:.0f} g all-up weight assumes a {pack_kg * 1000.0:.0f} g "
            f"power system (motor, ESC, mount and flight pack together - about "
            f"{pack_pct:.0f}% of all-up weight, the normal share for an electric model "
            "this size). Weigh yours. Every 10 g over budget has to be balanced out "
            "and shows up as a faster stall and a harder launch.")
    bullets.append(
        "- Check the balance the day you fly, with the real pack in, hatches closed "
        "and any camera/VTX fitted - not on the bench with an empty bay. Fingertips "
        "under the two marks, or a CG stand: the model should sit level or a hair "
        "nose-down. Nose-UP on the fingers means stop.")
    return {"title": "Balance & CG - the number that decides everything",
            "body": _para(bullets)}


# ---------------------------------------------------------------------------
# 3. elevon setup
# ---------------------------------------------------------------------------

def _sec_elevons(design: dict) -> dict:
    geo = _sub(design, "geometry")
    ele = _sub(geo, "elevons")
    planform = _text(design.get("planform"), "swept")
    vstab = _sub(geo, "vstab")
    vtype = _text(vstab.get("type"), "winglets")

    span = _num(geo.get("span_m"), 0.0)
    semi_mm = 0.5 * span * 1000.0
    inner = min(max(_num(ele.get("inner_frac"), 0.35), 0.0), 1.0)
    outer = min(max(_num(ele.get("outer_frac"), 0.95), 0.0), 1.0)
    if outer <= inner:
        inner, outer = 0.35, 0.95
    cf = _num(ele.get("chord_frac"), 0.22)
    cf = min(max(cf, 0.05), 0.5)

    ele_span_mm = (outer - inner) * semi_mm
    c_in = _chord_at(design, inner)
    c_out = _chord_at(design, outer)
    c_mean = 0.5 * (c_in + c_out)
    e_in_mm = c_in * cf * 1000.0
    e_out_mm = c_out * cf * 1000.0
    e_mean_mm = c_mean * cf * 1000.0
    hinge_in_mm = c_in * (1.0 - cf) * 1000.0
    hinge_out_mm = c_out * (1.0 - cf) * 1000.0

    def mm_at(deg: float) -> float:
        return e_mean_mm * math.sin(math.radians(deg))

    lively = planform in ("swept",) or _text(design.get("mission"), "") == "sport"
    pitch_lo, pitch_hi = (8.0, 12.0) if lively else (7.0, 10.0)
    roll_lo, roll_hi = (12.0, 18.0) if lively else (10.0, 15.0)

    bullets = [
        f"- Geometry you are building to: the elevons run from {inner * 100:.0f}% to "
        f"{outer * 100:.0f}% of the semi-span - {ele_span_mm:.0f} mm of hinge line per "
        f"side - and take the aft {cf * 100:.0f}% of the local chord. That makes the "
        f"surface {e_in_mm:.0f} mm deep at its inboard end and {e_out_mm:.0f} mm at the "
        "outboard end.",
        f"- Cut the hinge line {hinge_in_mm:.0f} mm behind the leading edge inboard and "
        f"{hinge_out_mm:.0f} mm behind it outboard, i.e. a straight line between those "
        "two points. Bevel the elevon's leading edge so it can move up and down without "
        "binding, and seal the top of the hinge gap with tape - an open gap on a wing "
        "leaks pressure straight past the only control surface you have.",
        "- Transmitter: use the built-in DELTA / ELEVON mix (or a wing type of 'elevon'), "
        "with one servo per side on the two channels the mix expects. Do NOT set an "
        "elevon wing type AND add a manual mix on top - the classic double-mix mistake "
        "doubles the travel and halves the servo life.",
        "- Bench-check before every maiden, standing BEHIND the model looking forward: "
        "stick back = both elevons UP; stick right = right elevon UP and left elevon "
        "DOWN. Reversed elevon mixing is the number-one flying-wing maiden killer, and "
        "it fails in under a second because you have no time to think about it.",
        "- The fix table when it is wrong: pitch correct but roll reversed = swap the "
        "two servo leads (or invert the aileron term in the mix). Pitch reversed but "
        "roll correct = invert the elevator term in the mix. Both reversed = reverse "
        "both servo channels. Re-check after every change; it is easy to chase your "
        "own tail here.",
        f"- Throws for the maiden: about {pitch_lo:.0f}-{pitch_hi:.0f} deg of pitch "
        f"({mm_at(pitch_lo):.0f}-{mm_at(pitch_hi):.0f} mm at the elevon's mean "
        f"{e_mean_mm:.0f} mm chord) and {roll_lo:.0f}-{roll_hi:.0f} deg of roll "
        f"({mm_at(roll_lo):.0f}-{mm_at(roll_hi):.0f} mm). Wings need LESS pitch throw "
        "than a tailed model, not more - there is nothing damping the pitch rate, so "
        "over-throw turns straight into pilot-induced oscillation.",
        f"- Set the mix so pitch and roll together never exceed about 25 deg "
        f"({mm_at(25.0):.0f} mm) on one surface. Past that the flow separates over the "
        "deflected surface and the elevon simply stops working - on a wing that is "
        "your pitch control failing at exactly the moment you asked most of it.",
        "- Expo: 30% on pitch, 25-30% on roll for the first flights. Add rate later, "
        "once you have seen how it behaves; you cannot un-crash a first flight that "
        "was over-controlled.",
        "- Neutral means aligned with the SECTION, not horizontal. The elevon is the "
        "back of a reflexed airfoil, so its neutral position continues the upward "
        "curve of the trailing edge. Set that mechanically at the linkage with the "
        "sub-trims at zero, and keep both sides identical - a 1 mm mismatch is a "
        "permanent roll trim.",
        "- Reflex / up-trim: 1-2 mm of up-elevon is a normal starting trim and is "
        "aerodynamically the same thing as adding reflex to the section. If you end up "
        "needing much more than that to hold level flight, the model is nose-heavy or "
        "the trailing edge lost its reflex during sanding - move the pack back a few "
        "millimetres rather than living with the trim drag.",
        "- If it needs DOWN trim to stop it climbing, stop and re-check the balance on "
        "the ground. Down-trim on a wing means the CG is at or behind the neutral "
        "point, and the next gust settles the argument.",
    ]

    if planform == "bell":
        bullets.append(
            "- Do not add aileron differential to this one. The bell spanload already "
            "yaws the nose INTO the roll (proverse yaw); differential exists to cancel "
            "adverse yaw that this planform does not produce, so it would fight the "
            "configuration. Roll it and watch the nose - it should follow, not lag.")
    elif vtype == "none":
        bullets.append(
            "- With no vertical surfaces, watch the nose during a roll input on the "
            "first flight. If it swings the wrong way, add 20-30% aileron differential "
            "(more up-travel than down) at the mix and re-check.")
    else:
        bullets.append(
            "- A little aileron differential - 20-30% more up-travel than down - takes "
            "the adverse yaw out of the turn entry, which matters on a wing because "
            "the fins are small and there is no rudder to paper over it. Most "
            "transmitters have a differential setting inside the elevon mix.")

    bullets.append(
        "- Servos: buy SG90s. The pockets are cut as the exact inverse of a "
        "measured SG90 with 0.25 mm of clearance per face, so it is a snug drop-in "
        "and a taller-cased servo on the same mounting footprint may not seat. "
        "Both servos on the same battery-eliminator supply, leads secured inside the "
        "panel before you close it, and zero slop in the linkage - free play in an "
        "elevon linkage shows up in the air as a wing that will not hold a trim.")
    return {"title": "Elevon setup & mixing", "body": _para(bullets)}


# ---------------------------------------------------------------------------
# 4. planform notes
# ---------------------------------------------------------------------------

def _flavour_line(design: dict) -> str:
    aero = _sub(design, "aero")
    geo = _sub(design, "geometry")
    wl = _num(aero.get("wing_loading_kgm2"), 0.0)
    wl_oz = wl * 3.277
    ld = _num(aero.get("ld_cruise"), 0.0)
    span_mm = _num(geo.get("span_m"), 0.0) * 1000.0
    v = _num(aero.get("v_cruise_ms"), 0.0)
    if wl < 4.0:
        feel = ("floaty - lovely in calm evening air, pushed around by any real wind")
    elif wl < 7.0:
        feel = ("an easy loading: honest launches, gentle landings, still usable in a breeze")
    elif wl < 11.0:
        feel = ("a firm sport loading: it carries speed, penetrates wind, and needs a "
                "proper approach rather than a float-on")
    else:
        feel = ("heavily loaded: fast, wind-proof, and completely unforgiving of a slow "
                "or nose-high launch")
    mission = _MISSION_LABELS.get(_text(design.get("mission"), ""), "")
    lead = f"How this {mission} will feel" if mission else "How it will feel"
    return (f"- {lead}: {wl:.1f} kg/m2 wing loading (~{wl_oz:.0f} oz/ft2) over a "
            f"{span_mm:.0f} mm span, cruising around {v:.0f} m/s with a power-off "
            f"glide near {ld:.0f}:1 - {feel}.")


def _sec_planform(design: dict) -> dict:
    geo = _sub(design, "geometry")
    st = _sub(design, "stability")
    planform = _text(design.get("planform"), "swept")
    label = _text(design.get("planform_label"), _PLANFORM_LABELS.get(planform, "flying wing"))
    body = _sub(geo, "body")

    sweep = _num(geo.get("sweep_le_deg"), 0.0)
    taper = _num(geo.get("taper"), 0.0)
    washout = _num(geo.get("washout_deg"), 0.0)
    root_inc = _num(geo.get("root_incidence_deg"), 0.0)
    span_mm = _num(geo.get("span_m"), 0.0) * 1000.0
    c_root_mm = _num(geo.get("root_chord_m"), 0.0) * 1000.0
    c_tip_mm = _num(geo.get("tip_chord_m"), 0.0) * 1000.0
    c_body_mm = _chord_at(design, 0.0) * 1000.0
    dihedral = _num(geo.get("dihedral_deg"), 0.0)
    depth_scale = _num(body.get("depth_scale"), 1.0)
    chord_scale = _num(body.get("chord_scale"), 1.0)
    half_w_mm = _num(body.get("half_width_m"), 0.0) * 1000.0
    tip_inc = root_inc - washout
    sweep_shift_mm = 0.5 * span_mm * math.tan(math.radians(max(sweep, 0.0)))

    head = _flavour_line(design)
    bullets = [head]

    if planform == "swept":
        bullets += [
            f"- This is the classic swept sport wing: {sweep:.0f} deg of leading-edge "
            f"sweep, taper {taper:.2f}, {washout:.1f} deg of washout. The sweep is not "
            "styling. It puts the tips behind the CG so that twisting them nose-down "
            "gives the same nose-up moment a tail would, and it is also what keeps the "
            "model pointing where it is going.",
            f"- Build the twist accurately: root at {root_inc:.1f} deg, tip at "
            f"{tip_inc:.1f} deg, linear in between. Check it with an incidence gauge or "
            "a pair of straightedges at the root and tip before the skin goes on. "
            "Washout you cannot measure is washout you do not have.",
            f"- The tips of a swept, tapered wing (tip chord {c_tip_mm:.0f} mm against "
            f"{c_root_mm:.0f} mm at the wing root) are the part most likely to stall "
            "first, and a tip stall behind the CG is a snap roll. The washout above is "
            "the entire defence - never sand it out chasing a 'straight' trailing edge.",
            f"- Sweep supplies effective dihedral, which is why the design only asks for "
            f"{dihedral:.0f} deg of geometric dihedral. Do not add more: a swept wing "
            "with extra dihedral gets a lazy Dutch roll that nothing on the model damps.",
            f"- Over the semi-span the leading edge moves {sweep_shift_mm:.0f} mm aft. "
            "That is the number that governs your jig: build both panels on the same "
            "flat table against the same sweep line, and measure tip-to-tip diagonals "
            "before the glue cures.",
        ]
    elif planform == "bwb":
        bullets += [
            f"- A blended wing body: {sweep:.0f} deg of sweep, taper {taper:.2f}, and a "
            f"centre section {chord_scale:.2f}x the wing root chord and "
            f"{depth_scale:.2f}x its thickness. The body is not a pod bolted on top - "
            f"it is the same surface getting deeper and longer toward the centreline, "
            f"blending out {half_w_mm:.0f} mm from the centreline on each side.",
            f"- That centre section is {c_body_mm:.0f} mm long, and it is the whole "
            "reason this planform exists: the payload, pack and electronics live inside "
            "the lifting surface instead of hanging under it. Keep the blend fair - a "
            "step or a hollow where the body meets the panel is a separation line, and "
            "it will show up as an early root stall.",
            f"- The centre body makes lift, so it needs the same reflexed section and "
            f"the same {root_inc:.1f} deg incidence as the root. Do not flatten the "
            "body's underside 'so it sits on the bench' - that is a decambered centre "
            "section and it changes the trim.",
            f"- {washout:.1f} deg of washout with {sweep:.0f} deg of sweep does the "
            "trimming together with the section reflex. On a long-range cruiser also "
            "resist the urge to add payload aft: a camera or a second pack behind the "
            "CG line is the fastest way to turn a docile cruiser into an aft-CG "
            "problem.",
            "- Fly it for range, not for speed: hold the trimmed cruise, use shallow "
            "turns, and let the big blended centre carry the weight. Steep banked turns "
            "load a low-aspect-ratio wing heavily and the induced drag bill is brutal.",
        ]
    elif planform == "plank":
        bullets += [
            f"- A plank: {sweep:.0f} deg of sweep, taper {taper:.2f}, aspect ratio "
            f"{_num(geo.get('aspect_ratio'), 0.0):.1f}. It looks like a rectangular "
            "door, and that is correct - the whole point is a simple straight wing with "
            "no sweep at all.",
            "- With no sweep there is nothing behind the CG to trim with, so the "
            "reflexed section does 100% of the trimming. That makes the trailing edge "
            "the most important 20% of this airframe. Cut it accurately, do not sand it "
            "round, and do not let a warp creep in while the glue dries.",
            f"- Correspondingly there is only {washout:.1f} deg of washout, and on a "
            "plank washout barely helps trim at all (an unswept tip is not behind the "
            "CG, so twisting it does almost nothing for pitch). What washout you do "
            "build is there purely to keep the tips flying at the stall.",
            "- Planks reward accuracy and punish trim drag: because the section is "
            "doing all the work, a plank that needs a lot of up-elevon to fly level is "
            "dragging its trailing edge through the air for the whole flight. Get the "
            "balance right and it will glide beautifully; get it wrong and it just "
            "feels heavy.",
            f"- Straight wings do not weathervane on their own, so this one carries a "
            "vertical surface at the centre and needs it (see the next section). The "
            "high aspect ratio also makes the panels floppy - the spar matters more "
            "here than on a stubby swept wing.",
        ]
    elif planform == "bell":
        bullets += [
            f"- This is a bell-spanload wing in the Horten / NASA Prandtl-D tradition: "
            f"{sweep:.0f} deg of sweep, taper {taper:.2f}, aspect ratio "
            f"{_num(geo.get('aspect_ratio'), 0.0):.1f}, and - the defining number - "
            f"{washout:.1f} deg of washout from root to tip.",
            f"- Build the twist exactly: {root_inc:.1f} deg at the root falling linearly "
            f"to {tip_inc:.1f} deg at the tip. That is far more twist than a normal wing "
            "and it will look wrong on the bench. It is not wrong. It is the airframe.",
            "- What the twist buys: the lift distribution across the span is bell-shaped "
            "instead of elliptical, so the outer ~30% of each panel carries very little "
            "lift. In that region the local flow is tilted such that the section's force "
            "vector leans FORWARD - the tips produce induced thrust rather than induced "
            "drag.",
            "- Because the tips make thrust, rolling the model yaws it INTO the turn. "
            "That is proverse yaw, and it is the opposite of the adverse yaw every other "
            "aircraft fights. A bell wing therefore steers itself in yaw with the "
            "elevons alone, which is exactly why it needs no vertical surfaces at all.",
            "- So: no fins, no winglets, nothing. Adding a fin to a bell wing does not "
            "make it 'safer' - it adds a weathervaning surface that overrides the "
            "spanload's own yaw control, adds drag and mass at the worst place for it, "
            "and throws away the reason you built this planform instead of a swept "
            "sport wing.",
            "- The honest trade: Prandtl showed in 1933 that for a given lift and a "
            "given root bending moment - that is, a given structural weight - the "
            "bell-shaped load beats the elliptical one, roughly 11% less induced drag "
            "with about 22% more span. At a fixed span it is the elliptical load that "
            "wins. You are buying efficiency with span and with build accuracy.",
            "- Build accuracy is the whole game here. A degree of twist error at one "
            "tip unbalances the spanload, and the proverse-yaw behaviour degrades into "
            "an ordinary wing that also happens to have no fins. Jig both panels, "
            "measure the tip incidence against the root with a gauge, and do not "
            "'improve' the twist because the tips look like they are not doing anything.",
        ]
    else:
        bullets += [
            f"- Planform: {label}. {sweep:.0f} deg of leading-edge sweep, taper "
            f"{taper:.2f}, {washout:.1f} deg of washout, root chord {c_root_mm:.0f} mm "
            f"across a {span_mm:.0f} mm span.",
            f"- Build the twist to the drawing: {root_inc:.1f} deg at the root, "
            f"{tip_inc:.1f} deg at the tip, linear between. On any tailless model the "
            "twist and the section reflex are the trim system.",
        ]

    if _bool(st.get("bell_spanload")) and planform != "bell":
        bullets.append(
            "- This design is flagged as carrying a bell spanload, so treat the washout "
            "figure as structural: it is setting the lift distribution, not just the "
            "stall behaviour.")
    return {"title": f"Planform notes - {label}", "body": _para(bullets)}


# ---------------------------------------------------------------------------
# 5. vertical surfaces
# ---------------------------------------------------------------------------

def _sec_vstab(design: dict) -> dict:
    geo = _sub(design, "geometry")
    st = _sub(design, "stability")
    vs = _sub(geo, "vstab")

    vtype = _text(vs.get("type"), "winglets")
    label = _text(vs.get("label"), _VSTAB_LABELS.get(vtype, "vertical surfaces"))
    count = int(_num(vs.get("count"), 0.0))
    area = _num(vs.get("area_total_m2"), 0.0)
    wing_area = _num(geo.get("area_m2"), 0.0)
    area_pct = 100.0 * area / wing_area if wing_area > 1e-9 else 0.0
    h_mm = _num(vs.get("height_m"), _num(st.get("fin_height_m"), 0.0)) * 1000.0
    c_root_mm = _num(vs.get("root_chord_m"), _num(st.get("fin_chord_m"), 0.0)) * 1000.0
    c_tip_mm = _num(vs.get("tip_chord_m"), 0.0) * 1000.0
    cant = _num(vs.get("cant_deg"), 0.0)
    y_frac = _num(vs.get("y_frac"), 0.0)
    sweep = _num(geo.get("sweep_le_deg"), 0.0)
    span_mm = _num(geo.get("span_m"), 0.0) * 1000.0
    vv = _num(st.get("vv"), 0.0)
    fin_airfoil = _text(geo.get("fin_airfoil"), "a symmetric section")
    y_mm = y_frac * 0.5 * span_mm

    if vtype == "none" or count <= 0 or area <= 0.0:
        bullets = [
            "- This design has NO vertical surfaces, and that is deliberate. A "
            "bell-spanload wing generates proverse yaw from its own lift distribution: "
            "the lightly-loaded tips produce induced thrust, so a roll input drags the "
            "nose into the turn instead of away from it. The spanload is the yaw "
            "control system.",
            "- Do not add fins, winglets or a centre fin 'just in case'. A fin "
            "weathervanes the model into the relative wind and swamps the small, "
            "useful proverse yawing moment you built the twist to get. You would end "
            "up with a heavier, draggier wing that flies like a worse version of the "
            "swept sport planform.",
            f"- What holds the heading instead is {sweep:.0f} deg of leading-edge sweep "
            "plus the spanload: in a sideslip the advancing panel presents more span to "
            "the flow and makes more drag than the retreating one, which yaws the model "
            "straight again. That is a real restoring moment, just a gentler one than a "
            "fin gives.",
            "- Expect the yaw to feel loose compared with a finned wing, especially in "
            "gusts near the ground. It should wander slightly and self-centre, not "
            "fishtail. Persistent fishtailing means the twist is not symmetric between "
            "the panels - fix the build, not the fin count.",
            "- The reported vertical tail volume for this design is zero by "
            "construction. Any tool or checklist that flags that as an error is a "
            "tail-aft tool being pointed at a Horten.",
        ]
        return {"title": "Vertical surfaces - none, on purpose", "body": _para(bullets)}

    what = {
        "winglets": f"{count} tip winglets",
        "twin_fin": f"{count} inboard fins",
        "center_fin": "a single centre fin",
    }.get(vtype, label)

    bullets = [
        f"- This wing carries {what}: {area_pct:.1f}% of the wing area in total, "
        f"{h_mm:.0f} mm tall, {c_root_mm:.0f} mm root chord tapering to "
        f"{c_tip_mm:.0f} mm, in {fin_airfoil}. That 3-9% of wing area is the real "
        "flying-wing proportion, measured off airframes that fly.",
    ]

    bullets.append(
        f"- It is deliberately NOT sized from the 0.02-0.05 vertical tail-volume "
        f"coefficient you would use on a tail-aft model. On a tailless design the fin "
        f"sits about one root chord behind the CG instead of a fuselage length, so "
        f"feeding that short arm into V_V = (S_v x l_v) / (S_w x b) would demand a fin "
        f"a quarter the size of the wing - a sail nobody has ever flown. The reported "
        f"V_V here is {vv:.4f}: read it as a bookkeeping number, not a target.")
    if sweep >= 10.0:
        bullets.append(
            f"- Most of the directional work is really done by the {sweep:.0f} deg of "
            "leading-edge sweep. In a sideslip the advancing panel makes more drag than "
            "the retreating one and yaws the model straight again. The fins add damping "
            "and a crisper feel; they are not the only thing keeping it pointed "
            "forward. Do not scale them up to 'fix' V_V, and do not lose the sweep.")
    else:
        bullets.append(
            f"- With only {sweep:.0f} deg of sweep there is no sweep-drag effect to help "
            "here, so unlike a swept wing this planform genuinely does depend on its "
            "vertical surface for directional stability. That is why an unswept wing "
            "always carries a fin, and why its arm - how far behind the CG you can get "
            "it - matters more than its area.")

    if vtype == "winglets":
        bullets.append(
            f"- Winglets get their yaw arm from the sweep - at the tip they already sit "
            f"{0.5 * span_mm * math.tan(math.radians(max(sweep, 0.0))):.0f} mm aft of "
            "the centreline leading edge, which is most of the arm this airframe has. "
            "Set them with the trailing edge flush with the wing tip trailing edge.")
        if abs(cant) > 0.5:
            bullets.append(
                f"- They are canted {abs(cant):.0f} deg outboard. That is worth "
                "building accurately: cant keeps them out of the grass on a belly "
                "landing, reduces the rolling moment the side force makes about the CG "
                "(too much roll-from-yaw coupling on a wing gives a lazy Dutch roll), "
                "and works with the tip vortex rather than against it.")
        bullets.append(
            "- Winglets are also the tip's crash structure. Bury the root properly - a "
            "tab into a slot, or a carbon pin - and reinforce with tape or a glass "
            "strip. A winglet that peels off in a landing takes wing skin with it.")
    elif vtype == "twin_fin":
        bullets.append(
            f"- The fins sit at {y_frac * 100:.0f}% of the semi-span, about {y_mm:.0f} mm "
            "out from the centreline on each side - the Skywalker X5 layout. Inboard "
            "fins have a slightly shorter yaw arm than tip winglets, but they survive "
            "belly landings, they do not lever the wing tips in a ground strike, and "
            "they keep the tip structure light where bending loads are lowest.")
        bullets.append(
            "- Build them vertical and identical. A degree of toe-in on one fin is a "
            "permanent yaw trim you will be fighting with elevon differential for the "
            "life of the model. Check them square to the wing chord line with a set "
            "square before the glue sets.")
    else:
        bullets.append(
            "- A single centre fin on the body spine is the right answer for an "
            "unswept planform: with no sweep to give an outboard fin any arm, the only "
            "useful place left is behind the centre body, as far aft as the trailing "
            "edge allows. Put its trailing edge at or just behind the body trailing "
            "edge and take every millimetre of arm you can get.")
        bullets.append(
            "- Keep it in line with the centreline and square to the wing. It is also "
            "the natural mount for an antenna or a VTX - but keep the mass low and "
            "close to the CG, not out at the fin tip.")

    bullets.append(
        "- Root attachment matters more than fin area: the root has to be buried in "
        "the wing surface, not butt-glued to it. Slot the root in, or add a tab and a "
        "carbon pin. A fin joint that flexes gives you yaw slop that feels exactly "
        "like a loose elevon linkage in the air.")
    return {"title": f"Vertical surfaces - {label}", "body": _para(bullets)}


# ---------------------------------------------------------------------------
# 6. airfoil & reflex
# ---------------------------------------------------------------------------

def _sec_airfoil(design: dict) -> dict:
    geo = _sub(design, "geometry")
    aero = _sub(design, "aero")
    st = _sub(design, "stability")
    planform = _text(design.get("planform"), "swept")

    airfoil = _text(geo.get("airfoil"), "a reflexed section")
    tc = _thickness_ratio(airfoil)
    re_mac = _num(aero.get("re_mac"), 0.0)
    cl_cruise = _num(aero.get("cl_cruise"), 0.0)
    cl_max = _num(aero.get("cl_max_3d"), 0.0)
    alpha = _num(aero.get("alpha_cruise_deg"), 0.0)
    sm = _num(st.get("static_margin"), 0.0)
    washout = _num(geo.get("washout_deg"), 0.0)
    sweep = _num(geo.get("sweep_le_deg"), 0.0)
    mac_mm = _num(st.get("mac_m"), 0.0) * 1000.0
    cm0_req = sm * cl_cruise

    bullets = [
        f"- Section: {airfoil}"
        + (f", {tc * 100:.0f}% thick" if tc > 0 else "")
        + ". Its trailing edge curves back UP, which gives the section a positive "
          "(nose-up) pitching moment about its own aerodynamic centre. On a tailless "
          "aircraft that is not a style choice - it is the trim system.",
        "- The arithmetic behind that: about the CG, Cm = Cm0 - CL x (static margin). "
        "Hands-off trim means Cm = 0, so the wing trims at CL = Cm0 / static margin. "
        "With a normal cambered section Cm0 is NEGATIVE and there is no positive CL "
        "that satisfies it: the model simply tucks. Positive Cm0 is mandatory.",
        f"- For this design that works out at Cm0 of roughly {cm0_req:.3f} to trim at "
        f"the cruise CL of {cl_cruise:.2f} with a {sm * 100:.1f}% static margin - "
        "supplied by the section reflex"
        + (f" plus {washout:.1f} deg of washout working through {sweep:.0f} deg of sweep"
           if (washout > 0.5 and sweep > 5.0) else "")
        + ". Whatever the airframe does not supply, you make up with up-elevon trim, "
          "and that costs drag for the whole flight.",
    ]

    if sweep > 5.0 and washout > 0.5:
        bullets.append(
            f"- Reflex and washout are two routes to the same nose-up moment, and this "
            f"design uses both: {sweep:.0f} deg of sweep puts the tips behind the CG, "
            f"and {washout:.1f} deg of washout unloads them there. If you build less "
            "twist than the drawing you have quietly removed part of the trim system, "
            "and the model will need permanent up-elevon to compensate.")
    elif planform == "plank":
        bullets.append(
            "- With no sweep, washout cannot help trim here: the tips are alongside the "
            "CG, not behind it, so twisting them changes almost nothing in pitch. The "
            "reflex is doing all of it, alone. That is why a plank needs a stronger "
            "reflex than a swept wing and why its trailing edge is sacred.")

    bullets.append(
        "- Sanding is where reflex dies. The up-curve lives in the last 15-25% of the "
        "chord, exactly the part builders instinctively sand into a straight taper to "
        "get a 'sharp' trailing edge. Do that and Cm0 falls toward zero, the model "
        "needs more and more up-trim, and at the design CG it can end up unable to "
        "trim at all. Keep the TE shape, and leave it 0.8-1.5 mm thick rather than "
        "knife-edged.")
    bullets.append(
        "- Reflex is not free: bending the trailing edge up removes camber where it "
        "made the most lift, so a reflexed wing has a lower CLmax and lands faster "
        f"than a cambered one of the same area. This one works out at CLmax "
        f"{cl_max:.2f}, cruising at {alpha:.1f} deg angle of attack - that is why the "
        "answer to a high stall speed on a wing is area, not more camber.")

    if re_mac > 0.0:
        if re_mac < 1.0e5:
            re_note = ("genuinely low. Air behaves 'sticky' at this scale, the boundary "
                       "layer separates easily, and published full-scale polars simply "
                       "do not apply. Keep the surface smooth, keep the leading edge "
                       "accurate, and expect the stall a little earlier than the "
                       "numbers say")
        elif re_mac < 2.5e5:
            re_note = ("normal park-flyer territory: the section behaves reasonably but "
                       "still below wind-tunnel numbers, so leave margin in the stall "
                       "estimate and do not chase the last 1% of surface finish")
        else:
            re_note = ("comfortably high for a model - the section will behave close to "
                       "its published character and small surface imperfections matter "
                       "less")
        bullets.append(
            f"- Reynolds number at the {mac_mm:.0f} mm mean chord is about "
            f"{re_mac:,.0f}, which is {re_note}.")

    bullets.append(
        "- Whatever you do to one panel's trailing edge, do to the other, to the "
        "millimetre. Asymmetric reflex is a rolling moment that changes with speed, "
        "and no amount of trim fixes something that only appears when you accelerate.")
    return {"title": "Airfoil & reflex - why the trailing edge is sacred",
            "body": _para(bullets)}


# ---------------------------------------------------------------------------
# 7. structure & building
# ---------------------------------------------------------------------------

def _sec_structure(design: dict) -> dict:
    geo = _sub(design, "geometry")
    body = _sub(geo, "body")
    mass = _sub(design, "mass")

    method = _text(geo.get("build_method"), "3d_printed")
    wall = _num(geo.get("wall_mm"), 1.0)
    span_mm = _num(geo.get("span_m"), 0.0) * 1000.0
    sweep = _num(geo.get("sweep_le_deg"), 0.0)
    c_body_mm = _chord_at(design, 0.0) * 1000.0
    c_root_mm = _num(geo.get("root_chord_m"), 0.0) * 1000.0
    half_w_mm = _num(body.get("half_width_m"), 0.0) * 1000.0
    bay_mm = _num(body.get("bay_length_m"), 0.0) * 1000.0
    depth_scale = _num(body.get("depth_scale"), 1.0)
    tc = _thickness_ratio(_text(geo.get("airfoil"), ""))
    # centre-body depth = depth_scale x t/c x wing-root chord (SPEC §8 invariant 6)
    bay_depth_mm = depth_scale * tc * c_root_mm if tc > 0 else 0.0
    total_g = _num(mass.get("total_kg"), 0.0) * 1000.0
    canopy = _bool(body.get("canopy"))
    spar_x_mm = 0.28 * c_body_mm
    sweep_shift_mm = 0.5 * span_mm * math.tan(math.radians(max(sweep, 0.0)))

    bullets = [
        f"- Spar first, everything else after. The bending load peaks at the "
        f"centreline and there is no fuselage to react it into, so the spar must run "
        f"CONTINUOUSLY through the centre body from one panel to the other - not two "
        f"panel spars meeting at a glue joint. Aim for a carbon tube or rod covering "
        f"at least the middle 60% of the {span_mm:.0f} mm span.",
        f"- Put it at roughly 28% of the chord, about {spar_x_mm:.0f} mm behind the "
        "leading edge at the centreline. That is near the section's maximum thickness "
        "(deepest section = stiffest spar for the least depth of cut) and close to the "
        "quarter-chord where the lift acts, so it does not twist the wing as it bends.",
    ]
    if sweep > 5.0:
        bullets.append(
            f"- Watch out for the sweep when you lay the spar out: over the semi-span "
            f"the leading edge moves {sweep_shift_mm:.0f} mm aft, so a straight tube at "
            "a constant distance from the nose walks forward through the section and "
            "eventually runs out of the leading edge. Either angle the spar back to "
            "follow the quarter-chord line, or use a straight through-tube across the "
            "centre body with separate panel spars joined to it.")
    bullets.append(
        f"- Balance the panels before you fly them: weigh the left and right halves "
        "separately once they are finished with servos and hardware in. A model with "
        f"one panel 10 g heavier than the other rolls hands-off, and on a {total_g:.0f} g "
        "wing that is easy to do with glue alone.")

    bay_line = (f"- Equipment bay: {bay_mm:.0f} mm of usable length inside a centre body "
                f"{c_body_mm:.0f} mm long and {2 * half_w_mm:.0f} mm wide at the blend.")
    if bay_depth_mm > 0.0:
        bay_line += (f" The centre section is about {bay_depth_mm:.0f} mm deep there "
                     f"({depth_scale:.2f}x the wing's own thickness), which is what has "
                     "to swallow your pack.")
    bullets.append(bay_line)
    bullets.append(
        "- Measure your actual pack, receiver and (if fitted) camera/VTX against those "
        "numbers BEFORE you commit to the build. Discovering the pack is 4 mm too deep "
        "after the airframe is closed up means cutting a hole in a lifting surface, "
        "which is exactly the place you do not want one.")
    bullets.append(
        "- The pack is the single heaviest item and it sits in the middle of the "
        "structure, so give it a proper floor: a thin ply or printed tray spreading "
        "the load into the skins, velcro to stop it sliding and a strap to stop it "
        "leaving. Belly landings load that floor every single flight.")
    if canopy:
        bullets.append(
            "- The design includes a faired hatch on the spine. Make it a real "
            "structural lid - magnets or a tongue-and-latch, not tape - because on a "
            "blended body the top skin is carrying load, and a floppy hatch is a soft "
            "spot right over the spar.")

    if method == "foamboard":
        bullets += [
            f"- Foamboard build, {wall:.0f} mm paper-faced board. Score-and-fold is the "
            "technique that makes a wing out of a flat sheet: cut the top paper only, "
            "crack the foam, and fold. A wrapped leading edge (one skin folded over the "
            "nose) is stronger and truer than two edges butted together.",
            "- Hot glue everywhere, low-temp gun so the foam survives. Seal the exposed "
            "foam edges and any cut paper with tape or paint - bare paper wicks water "
            "out of wet grass and a soggy wing is a scrap wing.",
            "- Foamboard has almost no torsional stiffness on its own, and a wing lives "
            "on torsional stiffness: a panel that twists under load changes its own "
            "washout in flight. Close the section (top and bottom skins joined at the "
            "trailing edge), and run a strip of packing tape diagonally on both skins.",
        ]
    elif method in ("foam", "epp", "xps", "depron", "hot_wire", "foam_core"):
        bullets += [
            "- Hot-wire foam cores: cut both panels with the SAME pair of templates and "
            "the same wire speed, then mark left and right immediately. Cores cut on "
            "different days are cores with different washout.",
            "- Sheet or tape the skin: 40-50 g fibreglass and epoxy, or brown packing "
            "tape spanwise for a Zagi-style beater. The skin is the torsion box - a "
            "bare foam core twists visibly by hand, which is far too soft to hold twist "
            "in flight.",
            "- Bed the spar in a routed channel with epoxy or foam-safe glue, and pack "
            "the servo wells and bay walls with ply or printed inserts so the hardware "
            "is not just pressed into foam.",
        ]
    else:
        standard_pla = wall >= 1.0  # LW-PLA gets the thinner 0.9 mm wall
        bullets += [
            f"- 3D-printed build at {wall:.1f} mm wall. Print the skins with a SINGLE "
            "perimeter and 0-5% gyroid infill: the skin is the structure, and doubling "
            "perimeters roughly doubles wing weight for very little strength.",
            "- Print panels chord-vertical (leading or trailing edge up). Layer lines "
            "then run spanwise, along the bending load, and the airfoil surface comes "
            "out clean without support scars across the reflex.",
            "- Print the centre body in as few pieces as the bed allows and register "
            "the joints positively - sockets, tongues, or the spar tube itself - so "
            "glue is not doing the alignment. Epoxy beats CA on foaming filament; a "
            "strip of fibre tape over the underside of each joint is cheap insurance.",
            f"- Root chord is {c_root_mm:.0f} mm and the centre body {c_body_mm:.0f} mm, "
            "so check the sections against your bed size early and watch the slicer "
            "preview for thin trailing-edge sections - slow the perimeters there or the "
            "reflex prints ragged.",
        ]
        bullets.append(
            "- You are set up for standard PLA: easy to print, but roughly twice the "
            "weight of LW-PLA for the same structure. The weight model accounts for "
            "it; if you can get LW-PLA the same airframe becomes a noticeably better "
            "flyer." if standard_pla else
            "- LW-PLA foams as it prints: dial flow and temperature on a test wall "
            "first (target around 0.6 g/cc, usually 50-60% flow at 230-250 C) and "
            "print slowly. The weight saving this design assumes only appears when the "
            "foaming is actually dialled in.")

    bullets.append(
        "- Elevon servos go in the panel just ahead of the surface, output arm as "
        "close to the hinge line as the linkage allows. Recess them flush and cover "
        "them: a servo standing proud of a wing skin is both drag and the first thing "
        "to break in a cartwheel.")
    bullets.append(
        "- Bottom-mounting (X5 / AR Wing factory style) keeps the top surface clean; "
        "top-mounting survives rough-ground belly landings better. Pick one and do "
        "both sides the same way, with short stiff linkages and a solid horn - free "
        "play in an elevon linkage is indistinguishable in the air from a wing that "
        "will not hold trim.")
    bullets.append(
        "- Run and secure the servo leads before you close the panels. There is no "
        "fuselage to fish a wire through afterwards, and cutting a lifting surface "
        "open to find a lead is how good wings become average ones.")
    bullets.append(
        "- The exported model already carries the hardware: a form-fitting "
        "pocket in each lower surface shaped from a measured SG90 model at "
        "0.25 mm of clearance per face, a round 8.25 mm wire pipe from its "
        "lead corner straight to the equipment bay, and a horn printed as "
        "part of each elevon. The pocket holds the servo only - fit the arm "
        "after it drops in, pointing aft, and it swings clear below the "
        "skin.")
    bullets.append(
        "- The motor's three phase leads enter through an 8.25 mm round hole "
        "in the bottom skin near the mount, then run ENCLOSED through a round "
        "8.25 mm internal pipe all the way to the equipment bay - the same "
        "clean bore as the servo runs. Feed the leads in at the belly hole "
        "before closing anything up; only the short hop from the motor to "
        "the hole is outside.")
    bullets.append(
        "- Bend the pushrods from 1.0-1.2 mm (.040-.047 in) music wire. Each "
        "elevon carries a stubby rounded-triangle horn, 15 mm tall at most, "
        "set as far aft of the hinge as that height can drive without the "
        "linkage binding. It has one plain 2.5 mm hole near the tip; thread "
        "the bent end through and take the slack out with a tight Z-bend or "
        "an E/Z-style keeper, because 2.5 mm over 1.2 mm wire leaves play, "
        "and play in an elevon linkage flutters. The design notes give the "
        "servo-arm angle to set at neutral and the expected throws.")
    return {"title": "Structure & building", "body": _para(bullets)}


# ---------------------------------------------------------------------------
# 8. first flight
# ---------------------------------------------------------------------------

def _sec_first_flight(design: dict) -> dict:
    geo = _sub(design, "geometry")
    aero = _sub(design, "aero")
    st = _sub(design, "stability")

    v_stall = _num(aero.get("v_stall_ms"), 0.0)
    v_cruise = _num(aero.get("v_cruise_ms"), 0.0)
    v_launch = 1.3 * v_stall
    v_app_lo, v_app_hi = 1.2 * v_stall, 1.3 * v_stall
    cg_mm = _num(st.get("cg_mm_from_nose"), _num(st.get("x_cg_m"), 0.0) * 1000.0)
    motors = geo.get("motors")
    motors = motors if isinstance(motors, list) else []
    pusher = any(isinstance(m, dict) and m.get("type") == "pusher" for m in motors)
    n_motors = len(motors)
    vtype = _text(_sub(geo, "vstab").get("type"), "winglets")

    bullets = [
        "- Range check first, every time: transmitter in range-test mode, walk 30+ "
        "paces, confirm solid response on both elevons. Then failsafe: throttle to "
        "zero on signal loss, checked by switching the transmitter off with the model "
        "restrained.",
        f"- Confirm the balance on the day, with the flight pack in and everything "
        f"closed up: {cg_mm:.0f} mm from the nose, fingertips under the marks. This is "
        "the one check that is worth being late for.",
        "- Control directions, standing behind the model: stick back = BOTH elevons UP; "
        "stick right = right elevon UP, left DOWN. Say it out loud while you move the "
        "sticks. A reversed wing is on the ground before your brain finishes the word "
        "'reversed'.",
        "- Wait for a calm evening with a steady breeze straight down the field, and "
        "pick a big clear area with grass. Wings land flat and fast and they land "
        "wherever they run out of energy - do not maiden one on a strip lined with "
        "trees.",
    ]

    grip = ("- The launch grip: hold the centre body from underneath, at or just behind "
            f"the CG mark ({cg_mm:.0f} mm from the nose), model level and pointing into "
            "wind. Holding it ahead of the CG makes it rotate nose-up out of your hand; "
            "holding it well behind makes it dive.")
    bullets.append(grip)
    if pusher:
        bullets.append(
            "- The prop is a pusher behind the wing. Your launch hand and the prop disc "
            "occupy the same airspace if you are careless: grip well forward of the "
            "disc, keep your fingers along the centreline underneath, and throw with "
            "the model beside your head rather than sweeping past your body.")
    elif n_motors:
        bullets.append(
            f"- Check prop clearance and thrust alignment on all {n_motors} motor "
            "position(s) before the first launch, and keep your launch hand clear of "
            "the disc.")

    bullets += [
        "- Power up first, THEN throw. Full throttle, let the airframe load up for a "
        "beat, then a firm level javelin throw straight into wind - flat, or at most "
        "5 deg nose-up. Throw it hard: aim to release above the stall speed, which "
        f"here means getting it away at more than {v_launch:.0f} m/s.",
        "- Do not throw it nose-up. A tailless model launched slow and nose-high has "
        "almost no pitch damping and very little elevon authority at that speed; it "
        "will drop a tip and arrive before you have moved a stick. Level and fast, "
        "every time.",
        "- Hands off for the first second after release. The instinct to pull up is "
        "what kills maidens - let it accelerate, then ease into a shallow climb.",
        f"- Trim it hands-off at around half throttle and roughly {v_cruise:.0f} m/s "
        "before doing anything else. Small trim steps: on a wing, one click of pitch "
        "trim is a bigger change than you expect because both surfaces move together.",
        "- Climb to three mistakes high (50 m or so) before you explore anything, and "
        "keep the first turns to gentle 20-30 deg banks. Low-aspect-ratio wings bleed "
        "energy fast in a hard turn.",
    ]
    if vtype == "none":
        bullets.append(
            "- Watch the nose during your first deliberate roll input: on this "
            "planform it should swing INTO the turn (proverse yaw). If it consistently "
            "swings out, land and check the twist symmetry between panels.")

    bullets += [
        f"- Stall speed is about {v_stall:.1f} m/s. Explore it high: throttle back, "
        "ease the nose up, and learn what the mush feels like before you meet it at "
        "two metres. Recovery is the same as any aircraft - stick forward, unload, let "
        "it fly.",
        f"- Approach at {v_app_lo:.0f}-{v_app_hi:.0f} m/s (1.2-1.3x stall), long, flat "
        "and into wind, carrying a little power until the last moment. Wings do not "
        "float in on a big flare.",
        "- Keep the flare small. Up-elevon on a tailless model raises the nose by "
        "taking lift OFF the trailing edge, so a big last-second pull actually reduces "
        "total lift and drops the model onto the grass. Level attitude, let it slide "
        "on the belly, done.",
        "- Change one thing per flight. Two changes at once and the airplane - the "
        "only honest instrument you have - can no longer tell you which one worked.",
    ]
    return {"title": "First flight - launch, trim, land", "body": _para(bullets)}


# ---------------------------------------------------------------------------
# public entry point
# ---------------------------------------------------------------------------

def build_guidance(design: dict) -> list[dict]:
    """Build the expert-guidance sections for a completed flying-wing design.

    Never raises: any section that fails is dropped rather than blanking the
    whole Design Notes panel.
    """
    if not isinstance(design, dict):
        return []

    sections: list[dict] = []
    builders = []
    if not design.get("feasible", True):
        builders.append(_sec_compromises)
    builders += [
        _sec_balance,
        _sec_elevons,
        _sec_planform,
        _sec_vstab,
        _sec_airfoil,
        _sec_structure,
        _sec_first_flight,
    ]
    for fn in builders:
        try:
            sec = fn(design)
        except Exception:  # guidance must never take the panel down with it
            continue
        if sec and sec.get("body"):
            sections.append(sec)
    return sections


# ---------------------------------------------------------------------------
# Learn knowledge base (GET /api/learn)
# ---------------------------------------------------------------------------

LEARN_ARTICLES: list[dict] = [
    {
        "id": "reflex-and-cm0",
        "title": "Reflex, Cm0, and how a tailless wing trims",
        "summary": "Why a flying wing needs a positive pitching moment, where it comes from, and the one-line equation that sets its trimmed speed.",
        "body": (
            "A conventional aircraft trims because a surface far behind the CG pushes "
            "the nose where it needs to go. Delete that surface and the wing has to "
            "make its own nose-up moment. That is what reflex is for.\n\n"
            "A reflexed airfoil's camber line curves back UP over the last fifth of the "
            "chord. The result is a positive Cm0 - a nose-up pitching moment about the "
            "section's aerodynamic centre - where a normal cambered section has a "
            "negative (nose-down) one.\n\n"
            "The whole of tailless trim is in one line. About the CG:\n"
            "- Cm = Cm0 - CL x SM, where SM is the static margin as a fraction of MAC.\n"
            "- Hands-off trim means Cm = 0, so CL_trim = Cm0 / SM.\n\n"
            "Read what that says:\n"
            "- If Cm0 is negative there is no positive CL that trims. The model tucks "
            "under. This is why you cannot simply put a Clark-Y on a plank.\n"
            "- A bigger static margin means a LOWER trimmed CL, which means a faster "
            "trimmed speed. Nose-heavy wings fly fast and land hot - not because they "
            "are heavy, but because of that division.\n"
            "- More reflex raises the trimmed CL and lets you fly slower, at the cost "
            "of lift and drag (see below).\n\n"
            "Reflex is not free. Bending the trailing edge up removes camber exactly "
            "where it was making the most lift, so a reflexed section has a lower CLmax "
            "and more drag at high lift than a plain cambered one. A wing therefore "
            "lands faster than a tailed model of the same area and weight. The answer "
            "to a too-fast stall on a flying wing is more area, never more camber.\n\n"
            "Practical consequences at the bench:\n"
            "- Never sand the trailing edge into a straight symmetric taper. The reflex "
            "lives in the last 15-25% of the chord and that is exactly what gets sanded "
            "away in pursuit of a 'sharp' edge. Leave the TE 0.8-1.5 mm thick and keep "
            "its shape.\n"
            "- Up-elevon trim is aerodynamically identical to more reflex. One or two "
            "millimetres is normal. Needing a lot of it means the model is nose-heavy "
            "or the section lost its reflex.\n"
            "- Needing DOWN trim means the CG is at or behind the neutral point. Land "
            "and re-balance; do not trim your way around it.\n"
            "- Reflex must be identical on both panels. Asymmetric reflex is a rolling "
            "moment that changes with speed, and no trim setting fixes a problem that "
            "only appears when you accelerate."
        ),
    },
    {
        "id": "sweep-washout-trim",
        "title": "Sweep and washout: the other way to trim a tailless wing",
        "summary": "How sweeping a wing back and twisting the tips down does the same job as reflex, and why a plank cannot use the trick.",
        "body": (
            "There are two ways to get the nose-up moment a tailless aircraft needs. "
            "Reflex is one. Sweep plus washout is the other, and most sport wings use "
            "both together.\n\n"
            "Sweep the wing back and the outer panels end up BEHIND the CG. Twist those "
            "tips nose-down - washout - and they carry less lift than the root, or even "
            "push down. A down force behind the CG is a nose-up moment. The swept, "
            "washed-out tips are functionally a tailplane that happens to be part of the "
            "wing.\n\n"
            "That immediately explains the planform families:\n"
            "- Swept sport wings (20-30 deg leading-edge sweep, 2-5 deg washout) split "
            "the job between a mild reflex and the twist.\n"
            "- Planks have no sweep, so their tips are alongside the CG rather than "
            "behind it, and twisting them does almost nothing for pitch. A plank's "
            "section must therefore be strongly reflexed and do 100% of the trimming. "
            "What washout a plank carries is purely stall insurance.\n"
            "- Bell-spanload wings take the twist idea to its conclusion: 8-13 deg of "
            "washout, shaping the whole lift distribution rather than just trimming.\n\n"
            "Sweep does two more useful jobs:\n"
            "- Directional stability. In a sideslip the advancing panel presents more "
            "span to the flow and makes more drag than the retreating one, which yaws "
            "the model straight again. This, not the little fins, is what keeps a swept "
            "wing pointing forward.\n"
            "- Effective dihedral. A swept wing rolls level on its own in a sideslip, "
            "which is why swept wings are built with little or no geometric dihedral. "
            "Adding dihedral on top gives a lazy Dutch roll that nothing on the model "
            "damps out.\n\n"
            "And one trap: sweep plus taper loads the tips hard, so a swept wing wants "
            "to stall at the tips - behind the CG - which is a snap roll rather than a "
            "nose drop. Washout is the defence. Build it accurately with an incidence "
            "gauge, check it on both panels, and never sand it out chasing a "
            "straight-looking trailing edge. Washout you cannot measure is washout you "
            "do not have."
        ),
    },
    {
        "id": "cg-static-margin-tailless",
        "title": "CG and static margin on a flying wing",
        "summary": "Why 3-15% MAC, why nose-heavy is cheap and tail-heavy is fatal, and how to find and hold the balance point in practice.",
        "body": (
            "The neutral point is where the aircraft's aerodynamic forces balance. CG "
            "ahead of it and a gust that pitches the nose up produces a restoring "
            "nose-down moment; CG behind it and the same gust is amplified. Static "
            "margin is the gap between the two, as a percentage of the mean aerodynamic "
            "chord.\n\n"
            "Flying wings run LOWER static margins than tailed models - roughly 3-15% "
            "MAC, with 5-10% typical, against 10-20% for a trainer. That is not "
            "bravado. Static margin divides into Cm0 to set the trimmed CL, so a big "
            "margin on a wing means a very low trimmed lift coefficient: a permanently "
            "fast, nose-down, hot-landing aeroplane.\n\n"
            "The asymmetry that matters is what happens when you get it wrong.\n"
            "- Nose-heavy is expensive but survivable: permanent up-elevon trim, extra "
            "trim drag, a flat glide, a fast landing.\n"
            "- Tail-heavy is not survivable. A tailed model with an aft CG is twitchy "
            "and porpoises, because a tailplane on a long arm still damps the pitch "
            "rate. A tailless model has almost no pitch damping - the only thing "
            "resisting a pitch rate is the wing itself over a chord's length - so past "
            "the neutral point it diverges faster than a human can react.\n"
            "- The recovery control is the wrong one, too. Up-elevon makes nose-up "
            "moment by taking lift OFF the trailing edge, so a wing that is diverging "
            "nose-up and being fought with up-elevon is also sinking.\n\n"
            "Practice:\n"
            "- Mark the CG on the underside of both panels at the MAC station, not just "
            "on the centreline. On a swept wing the CG line is not square across the "
            "model, and a mark on one side only invites a twisted check.\n"
            "- Check the balance with the real flight pack in, hatches closed, camera "
            "and VTX fitted. An empty-bay bench check is worthless.\n"
            "- Level or a hair nose-down on the fingertips is right. Nose-up is stop.\n"
            "- The pack is the balance tool: it is the heaviest single item and it "
            "slides. Velcro AND a strap - a pack that creeps aft under launch "
            "acceleration moves the CG at the worst possible moment.\n"
            "- A wing has a short nose, so ballast there has a poor moment arm: it "
            "takes several times more lead than the same fix on a tailed model. Moving "
            "the pack is always the better answer.\n"
            "- Maiden 3-5 mm FORWARD of the marked CG and walk it back a couple of "
            "millimetres a flight until pitch feels crisp but not nervous. On a wing, "
            "'a few millimetres' really is the whole adjustment range."
        ),
    },
    {
        "id": "bell-spanload",
        "title": "The bell-shaped spanload and proverse yaw",
        "summary": "What 8-13 degrees of washout buys you: induced thrust at the tips, yaw that goes the right way, and an aircraft with no vertical surfaces at all.",
        "body": (
            "The elliptical lift distribution is the classic answer to minimum induced "
            "drag - for a given span. In 1933 Prandtl published the other answer: for a "
            "given lift and a given root bending moment, which is to say for a given "
            "structural weight, the optimum load is BELL-shaped, with roughly 11% less "
            "induced drag and about 22% more span than the elliptical solution. The "
            "Horten brothers built sailplanes on that idea; NASA's Prandtl-D research "
            "aircraft flew it again and measured what it does.\n\n"
            "To get a bell load you twist the wing hard - 8 to 13 degrees of washout "
            "from root to tip, far more than any conventional wing - so lift falls away "
            "smoothly to nothing well inboard of the tip.\n\n"
            "The payoff is not just drag. In the outer region of a bell-loaded wing the "
            "local induced flow tilts the section's force vector FORWARD: the tips "
            "produce induced thrust rather than induced drag. Now roll the aircraft. On "
            "any normal wing the down-going aileron adds lift and induced drag on the "
            "outer panel and the nose yaws AWAY from the turn - adverse yaw, which is "
            "what the rudder exists to cancel. On a bell-loaded wing the same input "
            "changes the induced thrust at the tips, and the nose swings INTO the turn. "
            "That is proverse yaw.\n\n"
            "Which is why a bell wing has no vertical surfaces at all. None. The "
            "spanload is the yaw control system, and a fin is a weathervane that "
            "overrides it: it forces the nose into the relative wind, swamps the small "
            "useful proverse moment, and adds drag and mass. Bolting fins onto a Horten "
            "does not make it safer, it makes it a heavier swept wing with too much "
            "twist.\n\n"
            "What it costs you:\n"
            "- Span. The efficiency claim is per unit of structural weight, not per "
            "unit of span. At a fixed span the elliptical load still wins.\n"
            "- Build accuracy. The twist IS the aircraft. A degree of error at one tip "
            "unbalances the spanload and the proverse behaviour quietly degrades into "
            "ordinary handling on an aircraft that has no fins to fall back on. Jig both "
            "panels, measure tip incidence against root with a gauge, and accept that "
            "the tips will look like they are doing nothing. They are doing the yaw.\n"
            "- Nerve. Expect the yaw axis to feel loose in gusts compared with a finned "
            "wing. Gentle wander that self-centres is normal; persistent fishtailing "
            "means asymmetric twist, and the fix is the build, not a fin."
        ),
    },
    {
        "id": "elevon-mixing",
        "title": "Elevon mixing, differential and throws",
        "summary": "Setting up the delta mix, the reversal troubleshooting table, how much throw a wing actually wants, and why less pitch throw is better.",
        "body": (
            "Two surfaces, two jobs. Both elevons up or down together is pitch; one up "
            "and one down is roll. The transmitter does the adding, via the delta / "
            "elevon wing type.\n\n"
            "Setup order that works:\n"
            "- Select the elevon or delta wing type and plug one servo into each of the "
            "two channels it expects. Do NOT then add a manual mix on top - the classic "
            "double-mix error doubles the travel and eats servos.\n"
            "- Zero all sub-trims. Set neutral MECHANICALLY at the linkage. On a "
            "reflexed wing 'neutral' means the elevon continues the section's upward "
            "trailing-edge curve, not that it looks horizontal.\n"
            "- Match both sides by eye and by ruler. A 1 mm mismatch is a permanent "
            "roll trim.\n"
            "- Then, and only then, set throws.\n\n"
            "The reversal check, standing BEHIND the model looking forward: stick back "
            "= both elevons up; stick right = right elevon up, left down. Get it wrong "
            "and the maiden is over in about a second, which is why this is the number "
            "one flying-wing maiden killer. The fix table:\n"
            "- Pitch right, roll reversed: swap the two servo leads, or invert the "
            "aileron term in the mix.\n"
            "- Pitch reversed, roll right: invert the elevator term in the mix.\n"
            "- Both reversed: reverse both servo channels.\n\n"
            "Throws. A wing wants LESS pitch throw than a tailed model, not more, "
            "because there is nothing damping the pitch rate and the surfaces are close "
            "to the CG:\n"
            "- Maiden rates: around 8-12 deg of pitch and 12-18 deg of roll, with 30% "
            "expo on pitch and 25-30% on roll.\n"
            "- Cap the COMBINED deflection at about 25 deg on one surface. Past that "
            "the flow separates over the deflected elevon and it simply stops working, "
            "at exactly the moment you were asking most of it.\n"
            "- Add rate later once you have seen how it behaves. You cannot un-crash an "
            "over-controlled first flight.\n\n"
            "Differential. On a finned wing, 20-30% more up-travel than down takes the "
            "adverse yaw out of turn entry - useful, because the fins are small and "
            "there is no rudder to paper over it. On a bell-spanload wing, do not: that "
            "planform already yaws into the turn, and differential would fight it.\n\n"
            "Mechanics matter as much as the mixing. Seal the top of the hinge gap with "
            "tape, bevel the elevon leading edge so nothing binds at full deflection, "
            "keep linkages short and stiff, and use a solid horn. Free play in an elevon "
            "linkage feels in the air exactly like a wing that will not hold a trim."
        ),
    },
    {
        "id": "vertical-surfaces",
        "title": "Vertical surfaces on a wing: winglets, twin fins, centre fin, or nothing",
        "summary": "Why a wing's fins look far too small next to the textbook tail-volume band, and how to choose between the four layouts.",
        "body": (
            "Point a tail-aft design tool at a flying wing and it will tell you the fin "
            "is far too small. It is wrong, and the reason is arm.\n\n"
            "The vertical tail volume coefficient is V_V = (S_v x l_v) / (S_w x b), and "
            "the working band for tail-aft models is 0.02-0.05. On a tailless model l_v "
            "is about ONE ROOT CHORD, not a fuselage length. Feed that short arm into "
            "the same formula and it demands a fin roughly a quarter the area of the "
            "wing - a sail nobody has ever flown. Real flying wings carry 3-9% of wing "
            "area in vertical surface and fly fine.\n\n"
            "They fly fine because the fins are not what is holding the heading. Sweep "
            "is: in a sideslip the advancing panel makes more drag than the retreating "
            "one and yaws the model straight again. The fins add damping and a crisper "
            "feel. Treat a low reported V_V as bookkeeping, do not scale the fins up to "
            "'fix' it, and above all do not lose the sweep.\n\n"
            "The four layouts:\n"
            "- Tip winglets. On a swept wing the tips are already a long way aft, so "
            "winglets get their arm from the sweep for free, and they do a little for "
            "the tip vortex. They are also the tip's crash structure, so bury the root "
            "properly. Canting them outboard keeps them out of the grass and reduces "
            "the rolling moment their side force makes about the CG - too much "
            "roll-from-yaw coupling gives a lazy Dutch roll.\n"
            "- Inboard twin fins, at 50-65% of semi-span (the Skywalker X5 layout). "
            "Slightly less arm than winglets, but they survive belly landings, they do "
            "not lever the tips in a ground strike, and they keep mass off the "
            "outboard structure. Build them vertical and identical - a degree of toe-in "
            "on one is a permanent yaw trim.\n"
            "- A single centre fin. The right answer on a plank: with no sweep an "
            "outboard fin has no arm at all, so the only useful place is behind the "
            "centre body. Take every millimetre of arm you can and put the trailing "
            "edge at or behind the body's.\n"
            "- Nothing at all. Correct on a bell-spanload wing, where the spanload "
            "itself provides yaw control and a fin would override it. See the "
            "bell-spanload article.\n\n"
            "Whatever you fit, the root joint matters more than the area. Slot or pin "
            "the root into the surface rather than butt-gluing it: a fin joint that "
            "flexes gives yaw slop that feels exactly like a loose elevon linkage."
        ),
    },
    {
        "id": "wing-loading-launch",
        "title": "Wing loading, stall speed and the hand launch",
        "summary": "The one number that predicts how a wing will behave, why reflexed sections land fast, and the launch technique that follows from both.",
        "body": (
            "Lift is L = 0.5 x rho x V^2 x S x CL. In level flight lift equals weight, "
            "so the slowest a model can fly is set by its wing loading (weight over "
            "area) and its maximum lift coefficient. Everything about how an airframe "
            "feels follows from those two.\n\n"
            "Working bands, in kg/m2 (multiply by about 3.3 for oz/ft2):\n"
            "- Under 4: floaty. Beautiful on a calm evening, helpless in wind.\n"
            "- 4-7: easy. Honest launches, gentle landings, still usable in a breeze.\n"
            "- 7-11: firm sport and FPV loading. Carries speed, penetrates wind, needs "
            "a real approach.\n"
            "- Over 11: fast and wind-proof, and completely unforgiving of a slow or "
            "nose-high launch.\n\n"
            "Flying wings sit at the higher end of whatever band they are in, for two "
            "reasons. Reflex costs CLmax, so the same area supports the same weight at "
            "a higher speed. And wings are usually built compact - a short deep "
            "planform with a low aspect ratio - which puts a lot of weight on a modest "
            "area. Plan for it rather than being surprised by it.\n\n"
            "The launch follows directly. There is no undercarriage and no runway: the "
            "aircraft has to leave your hand already flying, which means above the "
            "stall speed, which on a typical wing is a genuinely hard throw.\n"
            "- Full throttle first, let it load up for a beat, then throw.\n"
            "- Firm, level, straight into wind. Flat, or at most 5 degrees nose-up.\n"
            "- Grip the centre body underneath at or just behind the CG. Ahead of it "
            "and the model rotates nose-up out of your hand; well behind and it dives.\n"
            "- Mind the pusher prop: grip forward of the disc and throw with the model "
            "beside your head, not sweeping past your body.\n"
            "- Hands off for the first second. The instinct to pull up is what kills "
            "maidens; a wing thrown slow and nose-high has neither pitch damping nor "
            "elevon authority and will drop a tip.\n\n"
            "Landing is the same physics backwards. Approach at 1.2-1.3x stall, long, "
            "flat, into wind, with a little power on. Keep the flare small: up-elevon "
            "raises the nose by taking lift off the trailing edge, so a big last-second "
            "pull on a tailless model reduces total lift and drops it onto the grass. "
            "Level attitude, let it slide on the belly.\n\n"
            "And the scaling trap: double a design's size and the area goes up 4x while "
            "the weight goes up nearer 8x, so wing loading doubles. Never scale a wing "
            "without re-checking loading and stall speed."
        ),
    },
    {
        "id": "reynolds-model-scale",
        "title": "Reynolds number at model scale",
        "summary": "Why full-scale airfoil data lies to you at 200 mm chord, what changes below Re 100,000, and how it shapes a wing's design.",
        "body": (
            "Reynolds number is the ratio of inertial to viscous forces in the flow. At "
            "sea level a useful shortcut is Re = 68,500 x speed (m/s) x chord (m). A "
            "200 mm chord at 12 m/s is about Re 164,000. A full-scale sailplane wing is "
            "up in the millions. That gap is why published airfoil data can mislead you "
            "badly.\n\n"
            "As Re falls, the boundary layer stays laminar further back, and laminar "
            "layers separate from an adverse pressure gradient far more readily than "
            "turbulent ones. The practical effects:\n"
            "- Maximum lift coefficient drops, so the real stall arrives earlier than "
            "the tables promise.\n"
            "- Profile drag rises, sometimes sharply, thanks to laminar separation "
            "bubbles.\n"
            "- The stall gets more abrupt and less forgiving.\n"
            "- Thick, aggressively cambered sections suffer worst. Below roughly Re "
            "100,000, thick sections go mushy and thinner ones win.\n\n"
            "Rough bands for models:\n"
            "- Below 100,000: indoor and small park models. Air feels sticky. Keep "
            "sections thin, keep the surface smooth, keep the leading edge accurate, "
            "and treat every published polar with suspicion.\n"
            "- 100,000-250,000: most park and sport wings. Sections behave reasonably "
            "but still below wind-tunnel numbers. Leave margin in the stall estimate.\n"
            "- Above 250,000: larger or faster models. Behaviour is close to published "
            "character and small surface imperfections matter less.\n\n"
            "This shapes flying wings specifically. A wing is short-coupled and "
            "low-aspect-ratio, which means fat chords for its span - and fat chords "
            "mean a higher Reynolds number than a high-aspect-ratio glider of the same "
            "span. That is one of the quiet advantages of the configuration. The flip "
            "side is the tips: a heavily tapered wing has small tip chords, so the tips "
            "run at a much lower Re than the root and stall earlier than the geometry "
            "alone suggests. On a swept wing, where the tips are behind the CG, that is "
            "another argument for building the washout accurately.\n\n"
            "One more practical note: the chord in the formula is the LOCAL chord. "
            "Quoting Re at the mean aerodynamic chord is convention, not a promise "
            "about the whole wing."
        ),
    },
    {
        "id": "building-wings",
        "title": "Building wings: foam, foamboard and 3D printing",
        "summary": "Torsional stiffness is the thing that matters, spars must be continuous through the centre, and each material has an honest set of trade-offs.",
        "body": (
            "A flying wing has no fuselage to hide structure in, so the structure is "
            "the aerodynamic surface. Two rules follow from that and they apply to "
            "every material.\n\n"
            "First: the spar runs CONTINUOUSLY through the centre. Bending load peaks "
            "at the centreline and there is nothing else to react it into, so two panel "
            "spars meeting at a glue joint in the middle is exactly the failure the "
            "pull-out of a dive will find. Run a carbon tube or rod across at least the "
            "middle 60% of the span, at roughly 28% chord - near the section's maximum "
            "thickness, where the spar is deepest and stiffest, and close to the "
            "quarter-chord where the lift acts, so it does not twist the wing as it "
            "bends. On a swept wing remember the leading edge marches aft as you go "
            "outboard: a straight tube at constant distance from the nose walks forward "
            "through the section and eventually exits the leading edge, so either angle "
            "the spar to follow the quarter-chord line or use a through-tube in the "
            "centre with separate panel spars.\n\n"
            "Second: torsional stiffness matters as much as bending strength. A panel "
            "that twists under load changes its own washout in flight, and washout is "
            "part of the trim system. Closed sections and skins carrying shear are what "
            "give you torsional stiffness - an open channel or an unsheeted core is far "
            "too soft.\n\n"
            "Materials, honestly:\n"
            "- EPP / XPS hot-wire cores. The classic wing build. Cut both panels with "
            "the same templates in the same session or they will not match. Skin them - "
            "40-50 g glass and epoxy, or spanwise packing tape for a beater - because a "
            "bare core twists by hand. EPP is nearly indestructible and repairable with "
            "contact adhesive; XPS is lighter and stiffer but brittle.\n"
            "- Foamboard. Cheapest and fastest, and score-and-fold makes a wing from a "
            "flat sheet: cut the top paper only, crack the foam, fold. Wrap the leading "
            "edge in one skin rather than butting two. Its weakness is torsion, so "
            "close the section and run diagonal tape on both skins. Seal every exposed "
            "edge - wet grass turns bare paper soggy.\n"
            "- 3D printing. Perfect repeatable geometry, including reflex and twist you "
            "would otherwise have to jig, plus replacement parts on demand. Plastic is "
            "heavy per unit of strength, so success depends on single-perimeter skins "
            "with 0-5% infill, chord-vertical printing so layer lines run spanwise "
            "along the bending load, and a carbon spar doing the real work. LW-PLA "
            "roughly halves structure weight versus standard PLA but needs its flow and "
            "temperature dialled in on test pieces first.\n\n"
            "Details that decide whether a wing flies well:\n"
            "- Weigh the finished panels separately. A 10 g difference between sides is "
            "a hands-off roll, and glue alone will do that to you.\n"
            "- Give the pack a real floor - ply or a printed tray spreading load into "
            "the skins - plus velcro and a strap. Belly landings load it every flight, "
            "and a pack that creeps aft moves the CG at the worst moment.\n"
            "- Recess the elevon servos flush and run the leads before you close the "
            "panels. There is no fuselage to fish a wire through afterwards.\n"
            "- Seal the hinge line on top, bevel the elevon leading edge, and use short "
            "stiff linkages with solid horns."
        ),
    },
    {
        "id": "reference-airframes",
        "title": "Airframes worth studying: Zagi, X5, AR Wing, Drak, Horten and Prandtl-D",
        "summary": "Five production wings and one research programme, and the specific lesson each one teaches about proportion, structure and configuration.",
        "body": (
            "Every good flying wing design decision has already been made by somebody "
            "and flown a thousand times. These are the airframes worth measuring.\n\n"
            "- Zagi (slope-combat wing, about 1200 mm span, root chord near 430 mm, "
            "aspect ratio around 5). The archetype: EPP foam, tape skin, tape hinges, "
            "two elevon servos, tip fins, nothing else. The lesson is that a flying "
            "wing needs almost no structure to work if the skin carries shear and the "
            "material bounces. Zagis survive impacts that would scrap anything built "
            "properly, which is why generations of pilots learned wings on them.\n\n"
            "- Skywalker X5 (1280 mm span, 717 mm root length, 44 dm2 area, aspect "
            "ratio 3.7). Look at that root-length-to-span ratio: 0.56. The centre "
            "section IS the fuselage, deep enough to swallow a big pack, an FPV stack "
            "and a camera without any pod bolted on top. Its twin fins sit inboard "
            "rather than at the tips, so they survive belly landings and keep mass off "
            "the outboard structure. If a design of yours ends up with a slim root and "
            "a pod on top, the X5 is the counter-example.\n\n"
            "- SonicModell AR Wing Classic (900 mm span, 482 mm length, root/span "
            "again about 0.54). The same proportions one size down, with winglets "
            "instead of inboard fins. Small span plus a full FPV load means a high wing "
            "loading, and it flies accordingly: fast, stable, wind-proof, and needing a "
            "committed launch. A good reminder that shrinking a design does not shrink "
            "its stall speed.\n\n"
            "- Ritewing Drak class (long-range blended wing body, moderate sweep, deep "
            "centre body, large canted winglets). This is what the BWB planform is for: "
            "the payload lives inside a lifting centre section instead of hanging under "
            "it, and the aircraft cruises for a long time at a modest speed. The lesson "
            "is that the blend must be fair - the body is not a pod, it is the same "
            "surface getting deeper - and that payload wants to sit on the CG, not "
            "behind it.\n\n"
            "- Horten sailplanes and NASA Prandtl-D. The bell-spanload line: large "
            "root-to-tip twist, no vertical surfaces at all, and proverse yaw from "
            "induced thrust in the tip region. Prandtl-D flew the idea as a research "
            "programme and measured the yaw behaviour that Prandtl predicted in 1933 - "
            "roughly 11% less induced drag than an elliptical load for the same "
            "structural weight, at about 22% more span. The lesson for a builder is "
            "that this configuration is bought with build accuracy: the twist is the "
            "aircraft, and there are no fins to cover for an asymmetric panel.\n\n"
            "What they have in common is worth stating plainly: short and deep, not "
            "glider-like. Root chords of a third to a half of the span, aspect ratios "
            "of 3.5 to 5 for sport and FPV wings, reflexed sections around 9% thick, "
            "elevons over most of the trailing edge, and a single pusher at the centre. "
            "When a design starts drifting toward a slim high-aspect-ratio wing with "
            "something strapped on top, it has stopped being a flying wing."
        ),
    },
]
