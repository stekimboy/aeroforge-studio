# Changelog

All notable changes to AeroForge, newest first. Dates are the dates the work landed; the engineering reasoning behind each entry — measurements, failed attempts, why a number is what it is — lives in [`aeroforge/DECISIONS.md`](aeroforge/DECISIONS.md).

## Straight wire runs — 2026-08-24

- **Every wire run between hollow bays is ONE straight Ø8.25 mm tube of constant bore** (builder's decision, two rounds same day): a straight rod passes end to end, and the tapering trumpet mouths are retired from wire runs — the cross-section is identical at every station. Pushrod guide tubes are a separate feature and keep their flare.
- **Servo runs enter the bay wall at exactly 90° — level, zero angle.** No oblique fallbacks: when the carved void doesn't reach the grommet's height the bay itself is enlarged (side trench or floor well on the conventional, floor well or aft side gallery on the flying wing/delta, aft gallery(+well) on canard/tandem, pod gallery on the twin-boom — the forward-bay types where a square side entry is geometrically impossible and the run stays straight but angled), and a line that cannot stay inside the skin refuses honestly.
- The motor wiring is one straight belly-to-bay pipe; it may angle downward (it exits through the bottom), never bend. The separate vertical entry bore is gone — the belly breach IS the entry.
- **Ports are oval — 12.0 × 8.25 mm** (builder: "more room for the wires"), servo and motor alike, still one straight extrusion of constant cross-section; the feasibility check samples the true ellipse (full height on the centreline, tapering to the rims).
- **The rear cavity extends on every flying-wing/delta configuration** (user, 2026-08-27): centre-fin designs keep the cavity under the fin at full section and the fin's root is trimmed to follow the inner hull (`_FIN_ROOT_BURY_MM`), tractor nose bosses no longer cap it, and the extension survey uses the hatch's station spacing. Proven by `tools_probe_cavity.py` across 18 configurations; the rear-hollow test now fails on any gallery fallback.
- **Aft hatch magnet pad is a shelf on a roof riser, not a rib down the cavity** (user, 2026-08-27): with the cavity extended, the pad ends 12 mm past the aperture and hangs from a riser into the ceiling behind the opening, like the front one. `bay_mm` publishes `magnet_centres_mm` / `aperture_x1_mm` / `aft_pad`.
- **The rear cavity is a root principle, not an add-on** (builder): the extension is born inside `hatch.build_bay`'s own survey/plan/band construction — one native void with the hull's inner surfaces, ≥ 3 mm rear walls, hinge/TE-guarded taper, hatch and lid unchanged — and the wire runs aim at the published per-station cavity band. The interim `_aft_hollow` add-on cutter is retired.
- **Every flying-wing/delta design must bury its servos** (builder: "I need servos on all planes and designs"): the optimizer gained the `servo_fit` constraint — the section's straight-well depth at the arm station must hold the measured SG90, so small boxes reshape (more chord, less taper) instead of shipping servo-less. Same doctrine the twin-boom already had.
- **The rear hollow's finish matches the hull** (builder: the first cut looked "separate and blocky"): station wires are splined floor/roof through 15 keel/crown samples with straight side walls, lofted smooth, tail tapering shut — the same surface language as the front of the hull.
- **The rear hollow is the hull itself, continued** (builder, two rounds: "make it smooth and part of the inner hollow hull ... maximize the internal space for things like batteries and flight controllers"): on the flying wing/delta the cavity aft of the hatch bay is a station-wise inner offset of the skin — floor at keel+wall, roof following the crown's own curvature (prints like the hull), full bay width tapering with the body, ≥ 3 mm walls, hatch unchanged — with the added stowage volume reported per design. The servo tubes open straight into it, wires exposed inside. On the conventional the per-side floor wells merged into one full-width chamber with a ≥ 3 mm floor. An interim fitted teardrop channel was rejected by the builder as "a small tube"; the rectangular box galleries survive only as the last-resort fallback.
- New shared router `conduits.straight_conduit` (line feasibility across merged material bands, mid-run air-gap refusal, hinge guard); every applied run's centreline is classified open end-to-end after the cut, and all four CAD suites assert collinearity ≤ 0.05 mm, constant bore, and the oval width on the built artifact.

## v3.1 — 2026-08-19 → 2026-08-22

### Hardware
- **Servo is now the TowerPro SG90** (`REAL SG90.stl`, the builder's own measured model), replacing the HXT900. Same 9 g class, far easier to buy; pocket is the exact inverse at 0.25 mm per face, proven on all 1,614 vertices. Measured: 23.00 × 12.40 case, 24.75 case top, 29.05 gear-head top, 32.55 spline top, ears 32.50 × 2.70 at 18.90. The first file supplied was the wrong one and is superseded.
- **Horn swing slot 15 → 25 mm** chordwise (`HORN_SWING_ROOM_MM = 8.0`); the pocket's hinge-margin cap now accounts for the boss end reaching past the ear tips.
- **Hatch magnet pockets bored at exactly Ø8.15** with zero added fit; pad and lug widths scale with the bore.
- **Canopy magnet lugs actually fuse now.** They never did on the bwb: the lug box was too short to reach the centreline crown and the station-loft trim band missed the lid's true skin. A lug is now fused only after it provably shares volume with the canopy, with a recut against the lid itself as the fallback.

### Geometry integrity
- **Tail post widened to carry its surfaces.** On every conventional style and the tandem, fin and stab roots poked 0.5–1.2 mm out of the knife-edge tail cone ("vertical stabilizers hanging off"). The post now clears the thickest root by 1 mm per side; verified in built solids.
- **Bay floor can no longer be deleted by the cut.** A swept-sport variant shipped its centre-body part with a 224 × 64 mm hole through the belly — the 1.2 mm floor sliver sat inside OCC's tolerance-merge range and every existing gate passed the result. New `_skin_breached` gate, a retry ladder, and a raised-floor rebuild (+1.8 mm, only when the thin floor fails). Regression test `test_the_bay_keeps_its_belly` on all four planforms.
- **Twin-boom wing sized to bury its aileron servo.** `servo_chord_floor_m` computes the minimum chord whose section can hold the pocket (box well depth, not peak thickness, minus dihedral/twist losses) and the optimizer lifts the chord to meet it. The `mapper` style had been shipping with no aileron pockets at all.

### Product
- **`biplane` and `glider` removed** outright: registry, dispatch, physics modules, tests. Six types remain.
- **Eager preview warm:** `/api/generate` queues the primary's part previews and the gallery minis in the worker pool immediately, so the viewer's first fetch joins an in-flight build instead of starting one. Byte-identical artifacts; stale warms are cancelled by a newer generate.
- Removing two types cut "any"-mode physics time by ~2/7.

### Known gap recorded
- The tail-type / wing-position / motor-layout selects are accepted and ignored for `conventional` and `twin_boom` — the transforms were only ever wired into the removed glider.

## v3 — 2026-08-16 → 2026-08-18

- Eight aircraft types with four configuration axes, each band cited to `RESEARCH_TYPES_V3.md`: delta (rides the tailless machinery), canard, tandem, biplane, twin-boom, glider.
- Type dispatch became a table asserted by `test_type_dispatch` — `multiwing.py` had sat finished-but-unwired while every per-type suite passed.
- `fuse_feature`: a feature is believed only when it can be classified in the result. The biplane had shipped with no fin while every flag said otherwise.
- Control surfaces clear fixed fins against the pocket's real bounding plane, not the hinge line.
- Servo pocket top slot +1.5 mm (`BOSS_SEAT_EXTRA_MM`), from a test print — and the lesson that the axis matters more than the number.
- **CFD campaign** on SimScale: conventional, tandem and flying wing at 20 m/s, k-ω SST, surface-wrapped geometry; results and live projects in `CFD_results/`.

## v2 — 2026-08-12 → 2026-08-15

- Multi-type support returns: `conventional` beside the frozen v1 flying-wing path, plus an "any" request mode.
- Tailed stability is genuinely tailed math (tail-volume NP, Munk fuselage term, decalage trim).
- Elevator split into left/right panels around the tail post; rudder built in a rotated frame.
- CAD moved to a 2-worker process pool with a byte-identity proof; immutable artifact caching; selective gzip.
- Servo wire pipe starts *at* the lead grommet (from the printed part).

## v1 — 2026-07-29 → 2026-08-11

- Flying-wing design studio: four planform families, four missions, five variants per request.
- One continuous tip-to-tip loft; hollow centre body with a removable hatch; structural motor mount.
- Print-in-place hinges, measured servo pockets, fused control horns with a solved four-bar linkage, round internal wire runs with trumpeted mouths.
- Tessellation gate: a cut is kept only if the result still meshes completely (the "missing 73 mm of nose" bug).
- Handling checked on all three axes with tailless-calibrated thresholds; fins sized from flying-wing practice rather than the tail-aft V_V band.
- Specific-drag objective, Reynolds floor, real-RC proportion terms; `VALIDATION.md` auto-generated from real runs.
