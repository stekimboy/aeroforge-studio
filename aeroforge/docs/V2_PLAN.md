# AeroForge v2 — multi-type architecture (working contract)

The user's brief, verbatim in spirit: keep the flying-wing experience EXACTLY
as it is (tolerances, hinge placement, wire management — all of it), add back
other aircraft types starting with CONVENTIONAL (wing + fuselage + horizontal
and vertical stabilizers), put an aircraft-type dropdown at the FIRST step of
the UI that routes to a type-specific panel, and add an "Any" mode that takes
bare parameters and optimizes across types. Separately: make generation,
preview and export faster WITHOUT changing any physical design or algorithm.

v1 is tagged `v1` on github.com/stekimboy/aeroforge. Regressing the
flying-wing path is the one unforgivable failure mode of this project.

## Ground rules for every work track

- READ `docs/ARCHITECTURE.md` and
  `aeroforge/DECISIONS.md` FIRST. They are the distilled history of this
  project and every hard-won lesson (silent OCC boolean failures, existence
  checks, measured-not-assumed doctrine, user decisions that must not be
  "fixed"). The hardware doctrine applies to every aircraft type.
- Run from `aeroforge/` with `.venv/Scripts/python.exe` (never activate).
- The flying-wing code paths and tests must keep passing untouched. New types
  live BESIDE them, never inside them.
- Every geometric claim gets verified by probing the built solid
  (classification / mesh gates), never by trusting a status code.
- Windows PowerShell 5.1 has no `&&`. Tests:
  `.venv/Scripts/python.exe -m pytest tests/ -q -k "<filter>"`.

## Design-dict schema extension (the contract between physics, CAD, frontend)

Every design dict gains:

```
design["airplane_type"] = "flying_wing" | "conventional"   # default flying_wing
```

`flying_wing` designs are byte-identical to v1 — no new required keys.

`conventional` designs add to `design["geometry"]`:

```
"fuselage": {
    "length_m", "width_m", "height_m",     # outer envelope
    "x_wing_le_m",                         # wing LE station on the fuselage
    "bay": {...}                           # equipment bay, same shape as v1
},
"tail": {
    "arrangement": "conventional",         # (T-tail etc. later)
    "x_le_h_m", "span_h_m", "c_root_h_m", "c_tip_h_m", "area_h_m2",
    "V_H",                                 # horizontal tail volume coeff
    "elevator_chord_frac",
    "x_le_v_m", "height_v_m", "c_root_v_m", "c_tip_v_m", "area_v_m2",
    "V_V",                                 # vertical tail volume coeff
    "rudder_chord_frac",
    "incidence_h_deg",
},
"ailerons": {"inner_frac", "outer_frac", "chord_frac"}     # like v1 elevons
```

Stability block gains `"tail_contribution"` fields where relevant; the same
`x_cg_m`/`x_np_m`/static-margin keys keep their meanings (tailed band
0.05–0.15 typical; document the chosen band from research).

## Module layout

- `backend/physics/config_defs.py` — add `AIRPLANE_TYPES` registry and
  conventional planform/mission bands (from RESEARCH_CONVENTIONAL.md).
- `backend/physics/stability_conv.py` — NEW: tail-volume NP/trim/sizing for
  conventional. Do not modify `stability.py`'s flying-wing math.
- `backend/physics/optimizer.py` — dispatch on `airplane_type`; the
  flying-wing branch must remain bit-identical (same calls, same defaults).
- `backend/physics/variants.py` — five variants per generate. For
  `airplane_type="any"`: run both types' candidates, return the best five
  spanning BOTH types, each dict carrying its own `airplane_type`.
- `backend/cad/conventional.py` — NEW: fuselage loft + wing + tails,
  REUSING `hinges.py` (ailerons, elevator, rudder), `servos.py`
  (wing servos for ailerons; fuselage-mounted servos are v2.1 — for v2 the
  elevator/rudder servos sit in the wing-root/fuselage bay area with pipes),
  `conduits.py` (Ø8.25 round pipes with trumpeted mouths).
  Exposes `build_design_parts(design)` / `build_design_solid(design)`
  compatible outputs; `backend/cad/geometry.py` gains a thin dispatch at its
  public entry points ONLY.
- `backend/api.py` — accepts `airplane_type` in GenerateRequest, threads it
  through; preview/export machinery unchanged (it already works on parts).
- `frontend/` — `#airplane_type` dropdown ABOVE the preset row. Selecting
  `flying_wing` shows the EXACT current sidebar (do not restyle it);
  `conventional` shows an analogous sidebar (its own planform/mission/tail
  options); `any` shows only mission targets + size box + motors + structure.
  One shared stage/gallery/datapanel — they already render whatever design
  dicts arrive.

## Verification bar for the conventional path (mirror of v1's)

- One valid watertight solid; nothing ahead of x=0; bbox inside recorded
  envelope; mesh area >= 98.5% of BRep.
- CG ahead of NP with the researched static-margin band; L=W at cruise;
  V_cruise >= stall_factor * V_stall; fits the box.
- Control surfaces (ailerons, elevator, rudder) hinged print-in-place where
  section depth allows, with the v1 clearance table; servo pockets are the
  measured SG90 inverse at 0.25 mm; every wire run is a round 8.25 mm pipe
  with trumpeted mouths ending INSIDE the bay void; horns follow the v1 horn
  doctrine (15 mm stub, one 2.5 mm hole, world-aligned).
- Every cut existence-checked. Five variants returned. Tests added per
  feature in `tests/test_conventional.py` (new file; do not bloat
  test_cad_export.py).

## Performance pass (separate track; ALREADY CONSTRAINED)

Allowed: process-level parallelism for the five preview builds, caching
(server- and HTTP-level), lazy imports, frontend asset caching, faster
serialization. FORBIDDEN: changing mesh tolerances of exported artifacts,
optimizer iteration counts, physics constants, CAD geometry, or any numeric
output. The proof obligation: same design id in == byte-comparable design
dict out, and the test suite stays green.

## Wave plan

1. research (web) -> aeroforge/RESEARCH_CONVENTIONAL.md ; perf track (code)
2. physics: conventional config/stability/optimizer/variants + unit tests
3. CAD: conventional airframe + hardware integration + tests
4. frontend: type dropdown + conventional panel + any-mode panel
5. integration: full verification, DECISIONS/ARCHITECTURE.md updates, server, push
