"""DELTA planform - a TAILLESS airplane type through the flying-wing machinery.

v3 (V3_PLAN.md): the delta "routes through the TAILLESS machinery as a new
planform family... Its dict looks like a flying-wing dict with
`planform: "delta"`" - elevons, reflexed section, strip-model NP/trim, the
whole three-axis handling check, and the entire flying-wing hardware doctrine,
unchanged.

THE INVARIANT THIS MODULE IS BUILT AROUND: the flying-wing surface must not
change. `tests/test_optimizer.py::test_only_flying_wing_planforms_exist` and
`tests/test_api.py::test_options_advertises_flying_wings_only` both assert
`set(PLANFORMS) == {"swept", "bwb", "plank", "bell"}`, and `/api/options`
serves PLANFORMS verbatim. So the delta PlanformDef is NOT registered in
`config_defs.PLANFORMS` at rest: it lives here, and `_delta_machinery()`
registers it (plus the per-planform lookup rows `optimizer.py` keys on the
planform name) ONLY for the duration of one solve, restoring every table in a
`finally`. At rest the process state is byte-identical to v2, the flying-wing
path never sees a "delta" key (it only ever looks up its own four names), and
`/api/options` between solves is unchanged. Single-process servers run solves
sequentially per request; the transient key is invisible to the fw path even
mid-solve because nothing in that path enumerates the tables.

Every band cites RESEARCH_TYPES_V3.md s.1 (tag [RT3 s.1]); reference
airframes: ParkZone F-27 Stryker, FT Mini Arrow, DW Hobby E0601.
"""
from __future__ import annotations

from contextlib import contextmanager

from .config_defs import PLANFORMS, PlanformDef
from .guidance import build_guidance
from .optimizer import optimize_design
from . import optimizer as _opt
from .variants import _traits_for_set

# ---------------------------------------------------------------------------
# Research bands, transcribed ([RT3 s.1.2] / quick-reference rows 1-7)
# ---------------------------------------------------------------------------
DELTA_SWEEP_BAND: tuple[float, float] = (45.0, 60.0)   # LE sweep, default 50
DELTA_SWEEP_DEFAULT: float = 50.0
# AR 1.8-3.5 (derived: pure triangle AR = 4/tan(sweep) -> 45 deg = 4.0,
# 60 deg = 2.3; RC deltas are cropped, pushing below the pure value). NOTE:
# the tailless machinery hard-clamps its AR sweep at 3.0 (optimizer._band's
# lo_limit), so the reachable slice through this path is 3.0-3.5 - the
# moderate-sweep end of the band. Recorded, not fought: the machinery's floor
# exists because the strip model + Oswald estimate degrade below AR 3.
DELTA_AR_BAND: tuple[float, float] = (1.8, 3.5)
# CG 15-22% MAC is the builder-measured VALIDATION anchor [RCU-DELTA]; the
# app computes NP itself, so the mechanism is the static margin: tailless
# hard band 0.03-0.15 (config_defs.SM_BAND), target 0.06-0.12 [RT3 row 4].
DELTA_CG_PCT_MAC_ANCHOR: tuple[float, float] = (15.0, 22.0)
DELTA_SM_TARGET_BAND: tuple[float, float] = (0.06, 0.12)
# Fin: free-flight ~4% of wing area, "RC needs a fin ~50% larger" -> ~6%;
# band 4-9% with a centre fin the default [MA-T via RT3 s.1.2, row 5].
# The machinery's own sweep-credit model (stability.flying_wing_fin: sweep
# substitutes for fin area, 6.5% unswept -> 3.2% at >= 25 deg) sizes a
# 45-60 deg delta's centre fin at ~3.5-3.9% - just under the MA-T floor,
# for the documented reason that MA-T's rule ignores sweep. Both figures are
# recorded; the hard gate stays the machinery's own 2-9% invariant.
DELTA_FIN_FRAC_BAND: tuple[float, float] = (0.04, 0.09)
DELTA_FIN_FRAC_DEFAULT: float = 0.06
FW_FIN_FRAC_HARD_BAND: tuple[float, float] = (0.02, 0.09)  # spec 8.7 gate
# Elevon chord 20-25% of local chord (plain-flap optimum ~0.2c,
# [LEN-CAN s.3 via RT3 s.1.2, row 6]); cruise trim must use <= 25% of the
# elevon throw (consistent with the existing 75%-of-travel handling gate).
DELTA_ELEVON_CHORD_FRAC: tuple[float, float] = (0.20, 0.25)
DELTA_TRIM_THROW_FRAC_MAX: float = 0.25
# Wing loading 20-45 g/dm^2 (= 2.0-4.5 kg/m^2), target 25-35 [RT3 row 7].
DELTA_WL_HARD_KGM2: tuple[float, float] = (2.0, 4.5)
DELTA_WL_TARGET_KGM2: tuple[float, float] = (2.5, 3.5)

# ---------------------------------------------------------------------------
# The delta PlanformDef - same dataclass the four v1 families use, so the
# tailless optimizer, strip model, fin sizing and CAD read it unmodified.
# ---------------------------------------------------------------------------
DELTA_PLANFORM = PlanformDef(
    name="delta", label="Delta",
    description=(
        "A tailless delta: 45-60 degrees of leading-edge sweep, a low aspect "
        "ratio, a cropped tip and elevons doing both pitch and roll. The "
        "sweep does most of the yaw work and a centre fin trims the rest - "
        "Stryker / FT Arrow class."),
    sweep_band=DELTA_SWEEP_BAND,                 # [RT3 s.1.2] 45-60, dflt 50
    # cropped-delta tip: taper ratio 0.10-0.30 [RT3 s.1.2 AR derivation]
    taper_band=(0.10, 0.30),
    ar_band=DELTA_AR_BAND,                       # machinery reaches 3.0-3.5
    # RC deltas trim mostly on section reflex + elevon; a few degrees of
    # washout keeps the cropped tip flying through the flare (same practice
    # band as the swept family's low end). The trim solver picks the value.
    washout_band=(0.0, 5.0),
    body_depth_band=(1.8, 2.6),                  # swept-family practice: the
    #                                              centre section swallows the
    #                                              pack the same way
    # AR ~3 + taper ~0.2 puts the root chord at ~half the span - that IS the
    # delta silhouette (pure triangle: c_r/b = tan(sweep)/2 = 0.50-0.87).
    root_chord_frac_band=(0.42, 0.60),
    # Low-AR delta: span efficiency suffers from the strong tip loading and
    # the reflex; between the swept wing's 0.88 and the bell's 0.80.
    oswald_mult=0.84,
    # centre fin default (V3_PLAN.md / [RT3 s.1.2]); the Stryker's twin-fin
    # layout maps to the existing twin_fin option. Bell-style zero-fin
    # layouts do NOT apply to deltas.
    vstab_options=("center_fin", "twin_fin"),
    bell_spanload=False,
    blurb="Sharp, fast, unmistakable - the paper-dart shape that flies.",
)

# optimizer.py keys its centre-body / elevon / dihedral tables on the
# planform NAME; these are the delta rows, registered transiently alongside
# the PlanformDef. Values follow the swept family (the delta's centre
# section carries the pack the same way) with the elevon chord from the
# delta's own researched band.
def _delta_aux() -> list[tuple[dict, str, object]]:
    return [
        (_opt.BODY_HALF_WIDTH_FRAC, "delta", 0.16),   # swept-family class
        (_opt.BODY_CHORD_SCALE, "delta", 1.16),       # short LERX: the delta
        #                                               nose is already deep
        (_opt.BODY_NOSE_ROUND, "delta", 0.55),
        (_opt.BODY_CROWN_FRAC, "delta", 0.64),
        # elevons: chord 0.20 of local chord ([LEN-CAN] plain-flap optimum,
        # [RT3 row 6]), spanning the TE outboard of the pusher prop cutout
        (_opt.ELEVONS, "delta", {"inner_frac": 0.32, "outer_frac": 0.95,
                                 "chord_frac": 0.20}),
        # sweep provides the roll stiffness; no geometric dihedral (every
        # [RT3 s.1.1] reference delta flies flat panels)
        (_opt.DIHEDRAL_DEG, "delta", 0.0),
    ]


@contextmanager
def _delta_machinery():
    """Register the delta planform in the tailless machinery's lookup tables
    for the duration of ONE solve, then restore them exactly.

    Only keys this context added are removed, and only if nothing else
    registered them first - so at rest `set(PLANFORMS)` is byte-identical to
    v2 and the guarded options/planform tests stay green (module docstring).
    """
    added: list[tuple[dict, str]] = []
    tables: list[tuple[dict, str, object]] = (
        [(PLANFORMS, "delta", DELTA_PLANFORM)] + _delta_aux())
    try:
        for table, key, value in tables:
            if key not in table:
                table[key] = value
                added.append((table, key))
        yield
    finally:
        for table, key in added:
            table.pop(key, None)


# ---------------------------------------------------------------------------
# Solve + variants (same entry-point shapes as physics/conventional.py)
# ---------------------------------------------------------------------------

def optimize_delta(inp: dict) -> dict:
    """One delta design through the REAL tailless optimizer.

    The input dict is the standard optimizer input; `planform` is forced to
    "delta" and `airplane_type` to "flying_wing" so `optimize_design` runs
    its own v1 tailless path (any other value would dispatch away from it).
    The returned dict is a flying-wing design dict with `planform: "delta"`
    (V3_PLAN.md) plus `airplane_type: "delta"` stamped for the v3 dispatch/
    CAD waves - the only key this module adds.
    """
    merged = {**inp, "planform": "delta", "airplane_type": "flying_wing"}
    with _delta_machinery():
        d = optimize_design(merged)
    d["airplane_type"] = "delta"
    return d


# Five delta characters. Knobs are the optimizer's own documented variant
# knobs (optimizer.py docstring); each character cites what it expresses.
DELTA_VARIANTS: list[dict] = [
    {"key": "delta_sport", "name": "Sport Delta",
     "tagline": "The 50-degree everyday dart - quick, tough, dead simple.",
     "knobs": {"vstab": "center_fin",
               # [RT3 s.1.2]: default 50 deg LE sweep
               "sweep_override": 50.0}},
    {"key": "delta_dart", "name": "Hot Dart",
     "tagline": "Near-60 sweep, thin section, heavier loading - it carries "
                "speed like nothing else this size.",
     "knobs": {"vstab": "center_fin",
               "sweep_override": 58.0,          # top of the [RT3] 45-60 band
               "taper_override": 0.15,
               "airfoil_pref": "RFX-7 reflexed",  # thin = the fast end
               "v_cruise_mult": 1.20,
               "wl_band_scale": 1.15,
               "sm_override": 0.06,             # lively end of the target band
               "span_pref_frac": 0.75}},
    {"key": "delta_park", "name": "Park Delta",
     "tagline": "45 degrees and lightly loaded - the FT-arrow end: slow, "
                "floaty, relaunchable all day.",
     "knobs": {"vstab": "center_fin",
               "sweep_override": 45.0,          # bottom of the band
               "taper_override": 0.28,
               "wl_band_scale": (0.55, 0.95),
               "wl_pen_weight": 0.0,
               "sd_weight": 0.6,
               "v_stall_goal_frac": 0.85}},
    {"key": "delta_fpv", "name": "FPV Delta",
     "tagline": "A deeper centre section with a real bay - camera and pack "
                "inside the dart.",
     "knobs": {"vstab": "center_fin",
               "sweep_override": 48.0,
               "body_chord_scale": 1.30,
               "body_depth_mult": 1.15,
               "body_width_mult": 1.10,
               "canopy": True,
               "sd_weight": 1.5,
               "span_pref_frac": 0.92}},
    {"key": "delta_stryker", "name": "Twin-Fin Delta",
     "tagline": "Stryker-style twin fins - the elevons keep the full "
                "outboard span.",
     "knobs": {"vstab": "twin_fin",             # [RT3 s.1.1]: the Stryker
               #                                  layout maps to twin_fin
               "sweep_override": 52.0,
               "taper_override": 0.25}},
]

_DELTA_PRIMARY = "delta_sport"


def _delta_guidance(vd: dict, d: dict) -> list[dict]:
    """Delta-specific 'why this variant' section, appended to the standard
    flying-wing guidance (build_guidance reads the same dict shape)."""
    a, st, geo = d["aero"], d["stability"], d["geometry"]
    lines = [
        f"- This is the {vd['name']}, a cropped delta: "
        f"{geo['span_m'] * 1000:.0f} mm span, {geo['sweep_le_deg']:.0f} deg "
        f"of leading-edge sweep (the [RT3] 45-60 band), aspect ratio "
        f"{geo['aspect_ratio']:.1f}, root chord "
        f"{geo['root_chord_m'] / geo['span_m']:.2f} of the span - the "
        "paper-dart proportion is the aerodynamics, not a style choice.",
        f"- Numbers: {a['wing_loading_kgm2']:.1f} kg/m2 loading, "
        f"{a['v_stall_ms']:.1f} m/s stall vs {a['v_cruise_ms']:.1f} m/s "
        f"cruise ({a['stall_margin']:.2f}x), L/D {a['ld_cruise']:.1f}, "
        f"static margin {st['static_margin'] * 100:.0f}% MAC (delta practice "
        "balances at 15-22% MAC - the same point expressed on the chord).",
        "- Deltas stall soft and mushy (vortex lift) but sink hard: fly the "
        "approach with a touch of power and do not flare early.",
    ]
    if not d.get("feasible", True):
        lines.append(
            "- NOTE: this character could not meet every constraint inside "
            "your box; it is the closest the physics could get - read the "
            "compromises section first.")
    return [{"title": f"Why this variant - {vd['name']}",
             "body": "\n".join(lines)}]


def generate_delta_variants(inp: dict) -> list[dict]:
    """Five delta characters through the real tailless machinery - the delta
    mirror of variants.generate_variants: always exactly five, one primary,
    infeasible characters returned flagged, traits normalised across the set
    (variants._traits_for_set works verbatim - these ARE tailless dicts)."""
    ctx: dict = {"seeds": []}
    designs: dict[str, dict] = {}

    def _run(vd: dict) -> dict:
        knobs = dict(vd["knobs"])
        v_mult = knobs.pop("v_cruise_mult", None)
        vs_goal = knobs.pop("v_stall_goal_frac", None)
        merged = {**inp, **knobs, "seeds": ctx.get("seeds", [])}
        if v_mult:
            merged["v_cruise"] = inp["v_cruise"] * float(v_mult)
        ref = ctx.get("v_stall_ref", 0.0)
        if vs_goal and ref > 0:
            merged["v_stall_goal"] = float(vs_goal) * ref
        try:
            return optimize_delta(merged)
        except Exception:
            d = optimize_delta(dict(inp))
            d["notes"].append(
                f"The {vd['name']} tuning could not be solved for these "
                "inputs; showing the untuned delta for this character.")
            return d

    def _seed(d: dict) -> dict:
        g = d["geometry"]
        return {"span": g["span_m"], "ar": g["aspect_ratio"],
                "sweep": g["sweep_le_deg"], "taper": g["taper"]}

    lead_def = next(v for v in DELTA_VARIANTS if v["key"] == _DELTA_PRIMARY)
    lead = _run(lead_def)
    designs[_DELTA_PRIMARY] = lead
    ctx["v_stall_ref"] = lead["aero"]["v_stall_ms"]
    ctx["seeds"].append(_seed(lead))
    for vd in DELTA_VARIANTS:
        if vd["key"] == _DELTA_PRIMARY:
            continue
        d = _run(vd)
        designs[vd["key"]] = d
        ctx["seeds"].append(_seed(d))

    ordered = [designs[vd["key"]] for vd in DELTA_VARIANTS]
    traits = _traits_for_set(ordered)
    out: list[dict] = []
    for vd, d, tr in zip(DELTA_VARIANTS, ordered, traits):
        d["character"] = {"key": vd["key"], "name": vd["name"],
                          "tagline": vd["tagline"]}
        try:
            d["guidance"] = build_guidance(d) + _delta_guidance(vd, d)
        except Exception:
            d["guidance"] = _delta_guidance(vd, d)
        out.append({
            "id": d["id"],
            "key": vd["key"],
            "name": vd["name"],
            "tagline": vd["tagline"],
            "planform": "delta",
            "airplane_type": "delta",
            "primary": vd["key"] == _DELTA_PRIMARY,
            "traits": tr,
            "design": d,
        })
    return out
