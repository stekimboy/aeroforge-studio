"""Five flying-wing CHARACTERS for one set of user requirements
(SPEC_FLYING_WING.md section 7).

The optimizer solves a single constrained problem: "the best tailless wing that
fits this box and flies this mission". But "best" hides a family of genuine
engineering trade-offs, and on a flying wing the biggest of them is the
PLANFORM FAMILY itself: a swept sport wing, a blended wing body, a bell-spanload
Horten and a plank are four different answers to the same question, not four
paint schemes. So each character carries its own planform plus the knobs that
express its personality, and the optimizer re-solves the whole coupled
weight / aero / stability / body-sizing problem against them.

The user's hard box constraint and every safety constraint (CG ahead of NP, the
0.03-0.15 tailless static-margin band, stall margin, proportion realism, fin
proportions, Reynolds floor, pack placement) are applied identically to all
five; a character that cannot satisfy them is still returned, flagged
infeasible with its binding constraints, so the returned count is always 5 and
at least three distinct planform families are always represented.

Traits are computed FROM THE PHYSICS RESULTS and min-max normalised ACROSS the
generated set, so the bars always compare these five aircraft with each other.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

from .config_defs import PLANFORMS
from .guidance import build_guidance
from .optimizer import optimize_design

TRAIT_LABELS = ("Stability", "Speed", "Slow flight", "Efficiency")


@dataclass(frozen=True)
class VariantDef:
    key: str
    name: str
    tagline: str
    planform: str
    #: knob overrides merged into the optimizer input dict. `ctx` carries
    #: reference numbers from the primary solution (e.g. its stall speed).
    knobs: Callable[[dict, dict], dict]
    #: plain-language summary of what this character trades away and gains
    trade: str = ""


# ---------------------------------------------------------------------------
# the five characters
# ---------------------------------------------------------------------------

def _sport_swept(inp: dict, ctx: dict) -> dict:
    return {
        "planform": "swept",
        "vstab": "winglets",
        # AR ~4.2 and ~24 deg of LE sweep: the Skywalker X5 / AR Wing point of
        # the design space, which is where the whole class has converged.
        "ar_band_shift": (0.2, -0.7),          # 3.7 - 4.5
        "sweep_shift_deg": -1.0,               # band centre -> ~24 deg
        "taper_override": 0.45,
        "seeds": ctx.get("seeds", []),
    }


def _bwb_cruiser(inp: dict, ctx: dict) -> dict:
    return {
        "planform": "bwb",
        "vstab": "winglets",
        # the point of this one is VOLUME: the deepest body the band allows,
        # the longest leading-edge root extension, and the widest blend
        "body_chord_scale": 1.42,
        "body_depth_mult": 1.20,
        "body_width_mult": 1.12,
        "canopy": True,
        "sd_weight": 1.8,                      # it is a cruiser: glide matters
        "span_pref_frac": 0.95,
        "wl_band_scale": (0.85, 1.15),
        "seeds": ctx.get("seeds", []),
    }


def _bell_horten(inp: dict, ctx: dict) -> dict:
    knobs = {
        "planform": "bell",
        "vstab": "none",                       # forbidden on a bell spanload
        "sd_weight": 3.0,                      # pure "best glide in the box"
        "span_pref_frac": 0.98,                # every millimetre of span
        "wl_band_scale": (0.55, 0.95),
        "wl_pen_weight": 0.0,
        "airfoil_pref": ["RFX-9 reflexed", "RFX-7 reflexed"],
        "seeds": ctx.get("seeds", []),
    }
    ref = ctx.get("v_stall_ref", 0.0)
    if ref > 0:
        knobs["v_stall_goal"] = 0.85 * ref
    return knobs


def _plank_park(inp: dict, ctx: dict) -> dict:
    knobs = {
        "planform": "plank",
        "vstab": "center_fin",
        # a plank buys its slow flight with AREA, and span is capped by the
        # box, so the aspect ratio comes DOWN to get it
        "ar_band_shift": (-0.5, -1.5),         # 4.5 - 6.5
        "taper_override": 0.90,
        "wl_band_scale": (0.55, 0.95),
        "wl_pen_weight": 0.0,
        "span_pref_frac": 0.95,
        "sd_weight": 0.4,                      # glide is explicitly not the goal
        "seeds": ctx.get("seeds", []),
    }
    ref = ctx.get("v_stall_ref", 0.0)
    if ref > 0:
        # a soft goal, not a hard target: the structural mass floor makes a big
        # stall reduction physically unreachable at these sizes, and an
        # always-infeasible variant would be useless
        knobs["v_stall_goal"] = 0.75 * ref
    return knobs


def _speed_delta(inp: dict, ctx: dict) -> dict:
    return {
        "planform": "swept",
        "vstab": "twin_fin",                   # small inboard fins, X5 style
        "v_cruise": inp["v_cruise"] * 1.25,    # design point 25% faster
        "ar_band_shift": (-0.5, -1.2),         # 3.0 - 4.0: short and deep
        "sweep_shift_deg": 5.0,                # band centre -> ~30 deg
        "taper_override": 0.38,
        "airfoil_pref": "RFX-7 reflexed",      # thin section for speed
        "wl_band_scale": 1.20,                 # heavier loading penetrates wind
        "span_pref_frac": 0.72,                # a compact wing is the point
        "sm_override": 0.05,                   # lively in pitch
        "seeds": ctx.get("seeds", []),
    }


VARIANTS: list[VariantDef] = [
    VariantDef(
        "sport_swept", "Swept Sport",
        "The everyday flying wing - quick, tough, launches off one hand.",
        "swept", _sport_swept,
        "is the reference point: the proportions the whole RC flying-wing "
        "class has converged on, with nothing exaggerated in any direction",
    ),
    VariantDef(
        "bwb_cruiser", "Long-Range Cruiser",
        "A blended wing body with a real payload bay - range and camera work.",
        "bwb", _bwb_cruiser,
        "spends drag and launch weight on VOLUME: a deep blended body that "
        "swallows a big pack and a camera, and a long leading-edge root "
        "extension to carry it, at the price of a heavier, thicker aeroplane",
    ),
    VariantDef(
        "bell_horten", "Bell-Distribution Floater",
        "No fins at all - the spanload itself does the yaw work.",
        "bell", _bell_horten,
        "trades wing loading and pitch authority for pure efficiency and "
        "proverse yaw: a big root-to-tip twist unloads the tips so they make "
        "induced thrust in a turn, which is why it needs no vertical surfaces "
        "- but it is a slender, fragile, deliberate aeroplane to fly",
    ),
    VariantDef(
        "plank_park", "Plank Park Flyer",
        "Unswept, slow and forgiving - the smallest field will do.",
        "plank", _plank_park,
        "gives up speed and wind tolerance for the slowest possible flying: "
        "no sweep, near-constant chord and a big lightly loaded wing drop the "
        "stall speed hard, but with no sweep to lean on it needs a real centre "
        "fin and it gets pushed around on a breezy day",
    ),
    VariantDef(
        "speed_delta", "Speed Delta",
        "Short, sharply swept and thin - it carries speed and punches wind.",
        "swept", _speed_delta,
        "trades slow-flight manners and glide for pace: a low aspect ratio, "
        "30 degrees of sweep, a thin reflexed section and a heavier wing "
        "loading give crisp handling and wind penetration, at the price of a "
        "fast stall and an approach you have to fly properly",
    ),
]

VARIANT_BY_KEY = {v.key: v for v in VARIANTS}
PRIMARY_KEY = "sport_swept"
#: which character leads for each planform the user can ask for
PRIMARY_FOR_PLANFORM = {v.planform: v.key for v in reversed(VARIANTS)}
PRIMARY_FOR_PLANFORM["swept"] = "sport_swept"


def primary_key_for(planform: str | None) -> str:
    """The character that leads when the user asked for this planform."""
    return PRIMARY_FOR_PLANFORM.get(planform or "", PRIMARY_KEY)


def variant_catalog() -> list[dict]:
    """Catalog for GET /api/options."""
    return [{"key": v.key, "name": v.name, "tagline": v.tagline,
             "planform": v.planform,
             "planform_label": PLANFORMS[v.planform].label}
            for v in VARIANTS]


# ---------------------------------------------------------------------------
# trait scoring
# ---------------------------------------------------------------------------

def _minmax(values: list[float]) -> list[float]:
    """Min-max normalize to 0.05..1.0 so a bar is never empty and the best of
    the set is always full."""
    lo, hi = min(values), max(values)
    if hi - lo < 1e-9:
        return [0.6] * len(values)
    return [0.05 + 0.95 * (v - lo) / (hi - lo) for v in values]


def _clamp(v: float) -> float:
    return max(0.05, min(1.0, v))


def _directional_authority(d: dict) -> float:
    """Combined yaw stiffness, expressed in equivalent fin-area fraction.

    stability.flying_wing_fin's own model says leading-edge sweep substitutes
    for fin area (6.5% of wing area unswept, 3.2% at 25 deg and beyond), so the
    sweep credit is worth the 3.3% of area it saves. A bell-spanload wing gets
    its directional stability from the spanload instead, and scores on sweep
    alone - which is exactly the physical story.
    """
    geo = d["geometry"]
    area = max(geo["area_m2"], 1e-6)
    fin_frac = geo["vstab"]["area_total_m2"] / area
    sweep_credit = 0.033 * min(max(geo["sweep_le_deg"], 0.0) / 25.0, 1.0)
    return fin_frac + sweep_credit


def _traits_for_set(designs: list[dict]) -> list[list[dict]]:
    """Compute the 4 trait rows for every design, normalized across the set."""
    sm = [d["stability"]["static_margin"] for d in designs]
    da = [_directional_authority(d) for d in designs]
    vc = [d["aero"]["v_cruise_ms"] for d in designs]
    wl = [d["aero"]["wing_loading_kgm2"] for d in designs]
    vs = [d["aero"]["v_stall_ms"] for d in designs]
    ld = [d["aero"]["ld_cruise"] for d in designs]

    n_sm, n_da = _minmax(sm), _minmax(da)
    n_vc, n_wl = _minmax(vc), _minmax(wl)
    n_vs = _minmax([-v for v in vs])          # lower stall = higher score
    n_ld = _minmax(ld)

    rows: list[list[dict]] = []
    for i, d in enumerate(designs):
        geo = d["geometry"]
        fin_pct = 100.0 * geo["vstab"]["area_total_m2"] / max(geo["area_m2"], 1e-9)
        stab_detail = f"{sm[i] * 100:.0f}% static margin"
        if fin_pct > 0:
            stab_detail += f", {fin_pct:.1f}% fin area"
        else:
            stab_detail += ", no fins (bell spanload)"
        rows.append([
            {"label": "Stability",
             "value": round(_clamp(0.65 * n_sm[i] + 0.35 * n_da[i]), 3),
             "detail": stab_detail},
            {"label": "Speed",
             "value": round(_clamp(0.6 * n_vc[i] + 0.4 * n_wl[i]), 3),
             "detail": f"{vc[i]:.1f} m/s cruise, {wl[i]:.1f} kg/m2"},
            {"label": "Slow flight",
             "value": round(_clamp(n_vs[i]), 3),
             "detail": f"{vs[i]:.1f} m/s stall"},
            {"label": "Efficiency",
             "value": round(_clamp(n_ld[i]), 3),
             "detail": f"L/D {ld[i]:.1f}"},
        ])
    return rows


# ---------------------------------------------------------------------------
# per-variant guidance
# ---------------------------------------------------------------------------

def _why_section(vd: VariantDef, d: dict, ref: dict | None) -> dict:
    a, st, geo = d["aero"], d["stability"], d["geometry"]
    body = geo["body"]
    lines = [
        f"- This airframe is the {vd.name}, a {d['planform_label'].lower()}: "
        f"it {vd.trade}.",
        f"- The numbers it actually came out with: {geo['span_m'] * 1000:.0f} mm "
        f"span, {geo['root_chord_m'] * 1000:.0f} mm wing root chord "
        f"({geo['root_chord_m'] / geo['span_m']:.2f} of the span) growing to "
        f"{geo['root_chord_m'] * body['chord_scale'] * 1000:.0f} mm at the "
        f"centreline, aspect ratio {geo['aspect_ratio']:.1f}, "
        f"{geo['sweep_le_deg']:.0f} deg of leading-edge sweep, "
        f"{geo['washout_deg']:.1f} deg of washout, "
        f"{a['wing_loading_kgm2']:.1f} kg/m2 wing loading, "
        f"{a['v_stall_ms']:.1f} m/s stall against a {a['v_cruise_ms']:.1f} m/s "
        f"cruise ({a['stall_margin']:.2f}x stall margin), cruise L/D "
        f"{a['ld_cruise']:.1f}, static margin {st['static_margin'] * 100:.0f}% MAC.",
    ]
    if ref is not None and ref is not d:
        ra, rst = ref["aero"], ref["stability"]

        def pct(new, old):
            return (new - old) / old * 100.0 if old else 0.0

        lines.append(
            f"- Against the {ref.get('character', {}).get('name', 'lead')} "
            f"design from the same inputs: stall "
            f"{a['v_stall_ms'] - ra['v_stall_ms']:+.1f} m/s "
            f"({pct(a['v_stall_ms'], ra['v_stall_ms']):+.0f}%), cruise L/D "
            f"{a['ld_cruise'] - ra['ld_cruise']:+.1f}, wing loading "
            f"{pct(a['wing_loading_kgm2'], ra['wing_loading_kgm2']):+.0f}%, "
            f"static margin "
            f"{(st['static_margin'] - rst['static_margin']) * 100:+.0f} points "
            f"of MAC.")

    if vd.key == "sport_swept":
        lines.append(
            f"- In the air that reads as: throw it and fly. "
            f"{geo['sweep_le_deg']:.0f} deg of sweep does most of the yaw work, "
            f"the tip fins trim the rest, and at "
            f"{st['static_margin'] * 100:.0f}% MAC it tracks without feeling "
            "dead. Elevons are quick - set 30% expo the first time out.")
    elif vd.key == "bwb_cruiser":
        lines.append(
            f"- In the air that reads as: it goes somewhere. The centre body is "
            f"{body['depth_scale'] * 100:.0f}% of the wing's own thickness and "
            f"{body['bay_length_m'] * 1000:.0f} mm of usable bay, so the pack "
            f"and the camera live inside the aerofoil instead of in the "
            f"breeze; about {a['ld_cruise']:.0f}:1 glide with the power off.")
    elif vd.key == "bell_horten":
        lines.append(
            f"- In the air that reads as: it turns flat with no rudder, because "
            f"there is none. {geo['washout_deg']:.1f} deg of washout unloads the "
            "tips so hard that they make induced THRUST on the outside of a "
            "turn - roll in with elevon alone and the nose follows. Land it on "
            "grass; a slender wing with no fins does not enjoy tarmac.")
    elif vd.key == "plank_park":
        lines.append(
            f"- In the air that reads as: {a['v_stall_ms']:.1f} m/s stall is "
            "walking pace - hand launches are trivial and a school field is "
            f"enough. With no sweep, the centre fin "
            f"({geo['vstab']['height_m'] * 1000:.0f} mm tall) is doing all the "
            "yaw damping, so keep it; and fly it in calm air.")
    elif vd.key == "speed_delta":
        lines.append(
            f"- In the air that reads as: it wants to be flown, not watched. "
            f"{st['static_margin'] * 100:.0f}% MAC and a thin "
            f"{geo['airfoil']} section make it light in pitch and fast in a "
            f"dive; fly the approach at {1.3 * a['v_stall_ms']:.0f} m/s and "
            "keep the power on until it is over the grass.")

    if not d.get("feasible", True):
        lines.append(
            "- NOTE: this character could not meet every constraint inside your "
            "box (see the compromises section above). It is still the closest "
            "the physics could get, but read that section before building it.")
    return {"title": f"Why this variant - {vd.name}", "body": "\n".join(lines)}


# ---------------------------------------------------------------------------
# public entry point
# ---------------------------------------------------------------------------

# v3: one lazy resolver per airplane type, so an unused type costs nothing
# and the flying-wing path never imports any of them. Keys mirror
# config_defs.AIRPLANE_TYPES; "flying_wing" is the in-module path below.
def _resolve_generator(atype: str):
    if atype == "conventional":
        from .conventional import generate_conventional_variants as g
    elif atype == "delta":
        from .delta import generate_delta_variants as g
    elif atype == "canard":
        from .canard import generate_canard_variants as g
    elif atype == "tandem":
        from .tandem import generate_tandem_variants as g
    elif atype == "twin_boom":
        from .twinboom import generate_twinboom_variants as g
    else:
        # biplane and glider were removed outright (builder, 2026-08-21);
        # an unknown type falls through to None and the API's registry
        # validation is what actually rejects it.
        return None
    return g


_V3_TYPES = ("conventional", "delta", "canard", "tandem", "twin_boom")


def generate_any_variants_v3(inp: dict) -> list[dict]:
    """The v3 "any" mode: run EVERY type's candidate set and return the best
    five by the shared specific-drag objective, spanning at least THREE
    airplane types so the gallery is a genuine cross-configuration trade
    study (V3_PLAN.md), each dict carrying its own airplane_type.

    Cost: one full five-character solve per type. That is deliberate - the
    scores being compared must be the same numbers each type's own gallery
    shows, so nothing is re-derived or approximated here.
    """
    from .conventional import traits_generic

    pool: list[dict] = []
    fw = generate_variants({**inp, "airplane_type": "flying_wing"})
    for v in fw:
        v["airplane_type"] = "flying_wing"
        v["design"]["airplane_type"] = "flying_wing"
    pool.extend(fw)
    for atype in _V3_TYPES:
        try:
            vs = _resolve_generator(atype)({**inp, "airplane_type": atype})
        except Exception:               # a type that cannot close this box
            continue                    # simply doesn't compete
        for v in vs:
            v["airplane_type"] = atype
            v["design"]["airplane_type"] = atype
        pool.extend(vs)

    pool.sort(key=lambda v: (not v["design"].get("feasible", False),
                             v["design"].get("cost", 1e9)))
    selected = pool[:5]
    # spanning guarantee: at least three types on display. Swap the worst
    # selected entries for the best candidate of each missing type until the
    # spread is reached (or the pool runs out of types).
    have = {v["airplane_type"] for v in selected}
    if len(have) < 3:
        others = [v for v in pool[5:] if v["airplane_type"] not in have]
        slot = len(selected) - 1
        for cand in others:
            if len(have) >= 3 or slot < 1:
                break
            if cand["airplane_type"] in have:
                continue
            selected[slot] = cand
            have = {v["airplane_type"] for v in selected}
            slot -= 1
    for v in selected:
        v["primary"] = False
    selected[0]["primary"] = True
    traits = traits_generic([v["design"] for v in selected])
    for v, tr in zip(selected, traits):
        v["traits"] = tr
    return selected


def generate_variants(inp: dict) -> list[dict]:
    """Run the optimizer once per character and return 5 variant dicts.

    Always returns exactly len(VARIANTS) entries, in VARIANTS order. The
    character whose planform family the user asked for is flagged `primary`.
    A variant that comes out infeasible is returned anyway, carrying its
    binding constraints.

    v2 (V2_PLAN.md): dispatch on `airplane_type`; the flying-wing path below
    is the v1 code, untouched - an absent/"flying_wing" airplane_type never
    imports a new module. v3 (V3_PLAN.md): six more types, each in its own
    module, and "any" merges ALL types' candidate sets (see
    `generate_any_variants_v3`).
    """
    atype = inp.get("airplane_type") or "flying_wing"
    gen = _resolve_generator(atype)
    if gen is not None:
        return gen(inp)
    if atype == "any":
        return generate_any_variants_v3(inp)

    primary_key = primary_key_for(inp.get("planform"))
    ctx: dict = {"seeds": []}
    designs: dict[str, dict] = {}

    def _seed(d: dict) -> dict:
        g = d["geometry"]
        return {"span": g["span_m"], "ar": g["aspect_ratio"],
                "sweep": g["sweep_le_deg"], "taper": g["taper"]}

    # A character's `vstab` knob is only its DEFAULT look. If the user actually
    # picked an arrangement in the sidebar, that is a decision about the
    # aircraft they intend to build and it outranks the character - otherwise
    # choosing "centre fin" silently produced five sets of winglets.
    # `config_defs.resolve_vstab` still filters the result per planform, so a
    # bell-spanload wing ends up with none whatever anyone asked for.
    user_vstab = (inp.get("vstab") or "auto")

    def _run(vd: VariantDef) -> dict:
        knobs = vd.knobs(inp, ctx)
        merged = {**inp, **knobs}
        if user_vstab != "auto":
            merged["vstab"] = user_vstab
        try:
            d = optimize_design(merged)
            if not d.get("feasible", True):
                # The character's tuning (a fixed taper, a narrowed AR band)
                # can be what makes the design infeasible in a small box
                # while the same planform, untuned, fits: measured on the
                # user's 650 x 650 x 500 fpv box, the Swept Sport's taper
                # 0.45 / AR >= 3.7 left the servo station 12 mm short of the
                # SG90 floor and the UI showed "closest feasible" - although
                # the bare swept optimum (taper 0.41, AR 3.7) is fully
                # feasible with the servo buried. A feasible aircraft of the
                # character's planform beats an infeasible one in its exact
                # style, so fall back the same way the except path does.
                # A tuned design that is feasible is untouched (byte-identical
                # bare requests).
                bad = [c.get("name") for c in d.get("constraints", [])
                       if not c.get("ok", True)]
                alt = optimize_design({**inp, "planform": vd.planform,
                                       "vstab": (user_vstab if user_vstab
                                                 != "auto" else
                                                 knobs.get("vstab", "auto"))})
                if alt.get("feasible", False):
                    alt["notes"].append(
                        f"The {vd.name} tuning could not meet "
                        f"{', '.join(str(b) for b in bad)} inside this box; "
                        "showing the untuned feasible solution for this "
                        "character instead.")
                    return alt
            return d
        except Exception:
            # never drop a variant: fall back to this character's planform with
            # no other tuning, so the set is always complete
            d = optimize_design({**inp, "planform": vd.planform,
                                 "vstab": user_vstab})
            d["notes"].append(
                f"The {vd.name} tuning could not be solved for these inputs; "
                "showing the untuned solution for this character instead.")
            return d

    # the leading character first: its stall speed is the reference the slow
    # characters' explicit goals are expressed against
    lead_def = VARIANT_BY_KEY[primary_key]
    lead = _run(lead_def)
    designs[primary_key] = lead
    ctx["v_stall_ref"] = lead["aero"]["v_stall_ms"]
    ctx["seeds"].append(_seed(lead))

    for vd in VARIANTS:
        if vd.key == primary_key:
            continue
        d = _run(vd)
        designs[vd.key] = d
        ctx["seeds"].append(_seed(d))

    ordered = [designs[vd.key] for vd in VARIANTS]
    traits = _traits_for_set(ordered)

    out: list[dict] = []
    for vd, d, tr in zip(VARIANTS, ordered, traits):
        d["character"] = {"key": vd.key, "name": vd.name, "tagline": vd.tagline}
    for vd, d, tr in zip(VARIANTS, ordered, traits):
        try:
            d["guidance"] = build_guidance(d) + [_why_section(vd, d, lead)]
        except Exception:
            d["guidance"] = [_why_section(vd, d, lead)]
        out.append({
            "id": d["id"],
            "key": vd.key,
            "name": vd.name,
            "tagline": vd.tagline,
            "planform": vd.planform,
            "primary": vd.key == primary_key,
            "traits": tr,
            "design": d,
        })
    return out
