# AeroForge v3 — eight airplane types + configuration axes (working contract)

The user's brief (2026-08-16, verbatim in spirit): the type list must include
**conventional, flying wing, delta, canard, tandem wing, biplane, twin-boom,
and glider**, and the app must expose **configuration options** as separate
choices: **tractor vs pusher**, **single vs twin motor**, **V-tail vs
conventional tail vs T-tail**, and **high/mid/low wing**. "The current servo
and wiring management systems are good to go, keep these across all designs."

Non-negotiables inherited from v1/v2:
- `flying_wing` and `conventional` keep their current behavior BIT-IDENTICAL
  when the new config fields are absent or default. v2 is commit `46650f1`+.
- The hardware doctrine applies to EVERY type: measured SG90 inverse pockets
  at 0.25 mm, round Ø8.25 wire pipes with 1.6× trumpeted mouths that start AT
  the lead grommet, print-in-place captive-pin hinges, world-aligned horns
  (15 mm cap, one 2.5 mm bore), per-face clearances with sources, and every
  cut existence-checked by classification — never trusted from a status code.
- Every number shown comes from a real calculation; every band cites research.

## Design-dict schema extension

```
design["airplane_type"] =
    "flying_wing" | "conventional" | "delta" | "canard" | "tandem" |
    "biplane" | "twin_boom" | "glider"

design["config"] = {                      # NEW block, all keys optional
    "motor_layout": "tractor" | "pusher",  # default per type (twin_boom →
                                           # pusher, others → tractor)
    "n_motors": 0 | 1 | 2,                 # 0 legal only for glider;
                                           # 2 = wing-mounted nacelles
    "tail_type": "conventional" | "t_tail" | "v_tail" | None,
                                           # None for tailless types
    "wing_position": "high" | "mid" | "low" | None,
                                           # None where meaningless
}
```

Absent `config` ⇒ the type's researched default, which for flying_wing and
conventional must reproduce today's output exactly.

Per-type geometry additions (each mirrors the v2 `geometry.tail` pattern —
every dimension the CAD needs, recorded where the physics computed it):

- `delta` — routes through the TAILLESS machinery as a new planform family
  (low AR 1.8–3.5, LE sweep 45–60°, center fin default, elevons). Its dict
  looks like a flying-wing dict with `planform: "delta"`.
- `canard` — `geometry.canard {x_le_m, span_m, c_root_m, c_tip_m, area_m2,
  V_C, elevator_chord_frac, incidence_deg}`; main wing AFT; the canard must
  stall FIRST (loading margin recorded). Vertical fin(s) aft or winglets.
- `tandem` — `geometry.wing2 {x_le_m, span_m, chords, area_m2, lift_share,
  decalage_deg}`; both wings carry ailerons/elevators per research.
- `biplane` — `geometry.wing2 {gap_m, stagger_m, decalage_deg, ...}` plus
  interplane/cabane strut stations; Munk gap/interference factor in the aero.
- `twin_boom` — `geometry.booms {y_mm, length_m, section}`; tail carried
  BETWEEN the booms; pusher motor on the pod default.
- `glider` — conventional geometry with glider bands (AR 10–18, wing loading
  from the thermal end); `n_motors: 0` ⇒ no mount, no motor wiring, nose
  ballast provision recorded instead.
- `tail_type: "v_tail"` — `geometry.tail` gains `{type: "v_tail",
  dihedral_deg, ...}` sized by projected-area equivalence (Purser–Campbell),
  ruddervators mixed; `"t_tail"` puts the stab atop the fin (deep-stall note
  in guidance, elevator pushrod routed up the fin — same pipe doctrine).

## Module layout (fences)

- `physics/config_defs.py` — AIRPLANE_TYPES grows to 8; per-type band
  constants live in the TYPE's own module, registered here.
- NEW `physics/canard.py`, `physics/tandem.py`, `physics/biplane.py`,
  `physics/twinboom.py`, `physics/glider.py`; delta lands as bands in the
  tailless path (config_defs PLANFORMS + variants character). Each module:
  evaluate/optimize/variants entry points with the same signatures as
  `physics/conventional.py` (the template to copy).
- NEW `physics/config_axes.py` — the four axes: V-tail/T-tail sizing
  transforms, wing-position → effective dihedral / stability deltas, motor
  layout/count → mass + CG + mount stations. Consumed by type modules,
  never imported by `stability.py`/`stability_conv.py` (those stay frozen).
- `physics/optimizer.py` + `variants.py` — dispatch table keyed on
  airplane_type ("any" runs ALL types' candidates, returns the best five
  spanning ≥ 3 types). Existing branches untouched.
- CAD: NEW `cad/multiwing.py` (canard, tandem, biplane — fuselage + two
  lifting surfaces, reusing `cad/conventional.py`'s fuselage/wing builders
  as importable functions), NEW `cad/twinboom.py`; glider and the tail_type/
  wing_position/motor axes land INSIDE `cad/conventional.py` (they are
  parameter changes to its existing builders, not new topologies); delta
  rides the existing flying-wing CAD. Dispatch stays at `cad/geometry.py`'s
  two public entry points only.
- `frontend/` — type dropdown grows to 8; a CONFIGURATION row (motor layout,
  motor count, tail type, wing position) appears for the types where each
  axis applies; flying-wing panel stays byte-identical.

## Verification bar (every new type)

Same as v2's: one watertight solid (mesh ≥ 98.5% of BRep), nothing ahead of
x=0, bbox inside recorded envelope and the user's box, CG ahead of NP with
the type's researched static-margin band, L=W at cruise, V_cruise ≥
stall_factor × V_stall, five variants per generate, all hardware existence-
checked, tests per type in `tests/test_<type>.py`. Canard adds: the canard
reaches CL_max before the main wing (the defining safety property). Biplane
adds: gap/stagger inside the researched band. Twin-boom adds: boom stiffness
proxy recorded. Config axes: V-tail projected volumes match the researched
equivalence; twin-motor CG stays in band.

## Waves

1. research (web) → `RESEARCH_TYPES_V3.md`: the six new types + four axes,
   reference airframes and cited bands each.
2. physics A: canard + tandem + biplane. physics B: twin_boom + glider +
   delta bands + `config_axes.py`. (Disjoint files; config_defs registration
   done by integration.)
3. CAD A: `multiwing.py` (canard/tandem/biplane). CAD B: `twinboom.py` +
   glider/axes inside conventional. Delta: verify the tailless CAD holds at
   delta bands, fix only what breaks.
4. frontend: 8 types + configuration row + any-mode across all.
5. integration: dispatch registration, full verification, docs, server, push.
