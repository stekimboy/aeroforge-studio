# Performance pass — what changed and what was measured

Contract: `docs/V2_PLAN.md` "Performance pass". Constraint honoured throughout:
**no physical design, algorithm, tolerance or numeric output changed.** Same
design in ⇒ byte-identical artifact out (proven below).

## Changes

1. **CAD builds moved to worker processes** (`backend/cadjobs.py`).
   OpenCASCADE is not thread-safe and its bindings hold the GIL, so the old
   in-request builds effectively serialized: five gallery previews requested
   concurrently all crawled together on ~2 effective cores and none finished
   for over 11 minutes. Previews, exploded-part previews and exports now run
   in a 2-worker `ProcessPoolExecutor` (spawn, sized for this 4-core / 16 GB
   box; `AEROFORGE_CAD_WORKERS` overrides, `0` = old inline behaviour, and a
   broken pool falls back inline so the feature can never be down). The job
   functions call the exact same builders with the exact same tessellation
   tolerances (preview 0.05/0.12, parts 0.06/0.15, exports untouched); all
   caching and per-design lock semantics stayed in `backend/api.py` — the
   same design is still never built twice concurrently.
2. **Pool warm-up at boot** (`backend/main.py` lifespan → daemon thread):
   workers pay their cadquery/neuralfoil import bill (~5 s warm cache, more
   cold) while the user is still filling in the sidebar, not in front of
   their first preview.
3. **cadquery no longer imports in the server process** (`backend/paths.py`
   carries `EXPORT_DIR`; `backend/api.py` imports no CAD module). Boot import
   set: 5.4 s → 2.9 s (warm; the OCP/vtk chain is the dominant cold cost).
4. **Immutable HTTP caching for per-design artifacts** (`backend/api.py`):
   preview STLs, part STLs and `parts.json` are content-addressed by design
   id (fresh uuid4 per generate, never reused, never mutated) and now carry
   `Cache-Control: public, max-age=31536000, immutable` — the browser never
   re-downloads megabytes of STL on a gallery revisit. The app shell keeps
   its deliberate `no-store` (stale app.js against a newer API breaks the
   page — see main.py).
5. **Selective gzip** (`backend/main.py` `SelectiveGZip`): JSON and the app
   shell compress (a /api/generate response went 185,719 → 28,829 wire
   bytes, 6.4×); `.stl`/`.step`/`.zip` and `/api/export/file/*` bypass the
   compressor — on 127.0.0.1 gzipping a 19 MB STL is pure CPU stall.
6. **Gallery prefetch concurrency 2** (`frontend/viewer.js`
   `MiniPreviewPool`): the mini-preview queue loads two cards at a time to
   match the 2-worker pool (it was strictly one-at-a-time). Same states,
   same rendering, just a bounded-2 queue.

## Measured (this machine: 4 cores, 16 GB, warm OS cache)

Request: swept / sport / 900×1200×300 box (the test-suite SMALL_BOX).

| metric | before | after |
|---|---|---|
| server boot to /health | ~5.4 s import + uvicorn (proxy¹) | **3.7 s measured** (2.9 s import) |
| POST /api/generate (5 variants) | 16.8 s (unchanged code path) | 16.8 s |
| generate JSON on the wire | 185,719 B | **28,829 B** (gzip) |
| one preview STL, solo build | 167.0 s in-process | 140.3 s in a worker |
| five previews, 5 concurrent GETs | **0 of 5 finished at 690 s** (run killed)² | **first 219.5 s, all five 700.4 s** |
| preview re-fetch (server cache) | ~0.1 s + full re-download on browser revisit | 0.11 s, and the browser caches it forever (immutable) |

¹ The pre-change baseline server run was lost mid-measurement; boot is
compared via the import-set proxy (`import backend.main` plus, for "before",
the `backend.cad.exporters` chain the old api.py pulled in).
² Same request, same 5-concurrent-GET pattern against the pre-change code:
the five in-thread builds interleaved on ~2 effective cores (846 CPU-s in the
first 7 min) and none had completed when the run was terminated at ~11.5 min.
The after-run finishes all five inside that window and shows the first wing
at 3.7 min.

## Determinism proof

- The 610 mm calibration design (`design_swept.json`) built through the OLD
  path (in-process `build_design_solid` + `write_stl_verified(0.05, 0.12)`)
  and through the NEW path (spawned CAD worker):
  `sha256 aa66591ca4df0e81cad535e8a592e975a11c1dc1334d78c97bc4ef5391a14d54`,
  13,112,684 bytes — **byte-identical**.
- /api/generate before vs after the change, same request: all five design
  dicts **identical minus the uuid** (`id` differs per generate by design).
- Tests: `tests/test_optimizer.py` 27 passed (80 s); `tests/test_api.py`
  27 passed (33 min — it builds five previews and a STEP/STL export, and now
  exercises the worker pool end-to-end: lazy previews, concurrent preview
  requests, exports through worker processes).

## Deliberately NOT done (constraint violations)

- **Parallel `optimize_design` in /api/generate.** The five variant solves
  are NOT independent: each character's knobs receive `ctx["seeds"]`, which
  accumulates the geometry of every variant solved before it
  (`variants.generate_variants`). Running them in parallel changes what each
  optimizer sees and therefore the numeric output — forbidden. The 16.8 s
  physics stage stays sequential.
- **No gzip / re-encoding of exported artifacts**, no tessellation-tolerance
  changes anywhere, no CAD geometry edits, no optimizer/physics changes.
- **Deferring the neuralfoil import** out of boot: it would just move ~3 s
  onto the user's first /api/generate — cost moved, not removed.

## Speed pass 2 (2026-08-28, overnight task 4) - same design, same bytes

Constraint unchanged: no physics number, no geometry, no export tolerance
moved. Proof and numbers below; scripts from the profiling pass are summarised below.

### Where the time went (measured before)

* `generate` (UI-default request, 5 variants): 61 s wall alone. cProfile:
  a third of it in `airfoils.Airfoil`'s derived quantities - `alpha_for_cl`
  / `cd_at_cl` / `lift_slope_2d` / `cm0` re-did argmax -> slice -> argsort ->
  interp (and a `polyfit`) on the already-cached polar grid on EVERY call,
  ~30 000 times per generate; `polar()` rebuilt three arrays from tuples
  180 000 times; `twinboom.section_well_frac` (the servo-pocket chord floor
  the flying-wing optimizer evaluates too) scanned 121 window positions in a
  Python loop ~5 000 times (14 s).
* CAD: **cProfile is blind to cadquery's booleans** (multimethod dispatch:
  20 `cut` calls report 0.000 s; a 618 s `build_design_solid` profiled as
  38 s). That is why the old "160 s STEP" figure never matched the 15-35 min
  the server shows. `speed/timewrap.py` (wrappers on the builders and
  `Shape` methods) accounts for them; its table for the default wing lands
  in `speed/out/timeprof_*.txt` when the queued chain runs.
* Pipeline: one design was built up to THREE times - one-piece preview,
  exploded preview, and again for every export - and a STEP click queued in
  the 2-worker pool's FIFO behind the four gallery minis, whose warm threads
  all submitted at generate time. Observed live 2026-08-28 00:29-01:32:
  the user's STEP took 63 min end to end; both workers were on the SAME
  design's parts (preview warm, then the export rebuilt them).
* Preview tessellation on one solid: 0.05/0.12 -> 18.0 MB, 6.4 s;
  0.08/0.2 -> 5.5 MB, 1.8 s; 0.12/0.25 -> 3.1 MB, 1.3 s.

### Changes

1. `physics/airfoils.py`: `_polar_arrays` (read-only arrays per (airfoil,
   Re bucket)) and `_polar_branch` (sorted pre-stall branch, Cl_max, lift
   slope, cm0 - once per bucket). `physics/twinboom.py`:
   `section_well_frac` scans the 121 windows as one boolean matrix (min/max
   exact -> bit-identical, checked on 1200 (section, window) pairs against
   the loop) and memoizes the section interpolation.
   **generate 61 s -> 28 s (flying wing), 20 s -> 8 s (conventional);
   all five variants of both types identical** (full-precision JSON dump,
   ids stripped, before == after).
2. `cad/brepcache.py` (new) + `cadjobs._parts_for/_solid_for`: the first job
   to build a design's parts (or one-piece solid) writes them as binary BREP
   (BinTools, triangulation included, BEFORE its own tessellation so a
   loader sees the exact in-memory state the exporters see) and every later
   job loads instead of rebuilding - a STEP after the exploded preview is a
   load + write instead of a second 15-35 min build. Keyed by the design
   dict (minus id/prose) + a digest of `backend/cad/*.py`; bounded to 24
   entries / 3 GB; `AEROFORGE_BREP_CACHE=0` disables. `exporters` accept
   prebuilt shapes (`parts=`, `solid=`, `meta=`); signatures otherwise
   unchanged. `tests/test_brepcache.py`.
3. `api._warm_previews`: gallery minis build one at a time and never while
   an export is building (one worker stays free for the user's click);
   `_export_build` files cache-hit durations under `<type>:<fmt>:cached`
   so the ETA history only learns from cold builds.
4. `cadjobs.PREVIEW_TOL = (0.08, 0.2)` for the preview meshes only (was
   0.05/0.12 one-piece, 0.06/0.15 parts). Export tolerances untouched.

### Byte-identity proof (probe-level, loft + motor boss + 2 cuts, mesh gate)

Direct write vs. BinTools round trip, export tolerances:

* STL: sha256 `ed5a06a8...` on both paths, and on two independent direct
  builds in separate processes - identical.
* STEP: 174 676 lines; the cache path's body is byte-identical to a direct
  write (`0a704873...`). The ONLY variation seen anywhere is between two
  DIRECT writes (7 lines: the `FILE_NAME` timestamp and XCAF's
  address-dependent order of the two `STYLED_ITEM`/`COLOUR_RGB` records) -
  a pre-existing writer nondeterminism, present before this pass and
  independent of the cache; every geometry entity is identical. A second
  export inside one process also bumps `NEXT_ASSEMBLY_USAGE_OCCURRENCE`
  ids ('1','2' -> '3','4'), again pre-existing (workers are long-lived).

The full default-wing run (`speed/identity_run.py`: parts + solid build,
STEP / parts.zip / STL both ways, timing table) is queued under
`tools_cadlock.py` behind the validity sweeps; its report lands in
`speed/out/identity_flying_wing.txt`.

### Speed pass 2 - identity run results (2026-08-28 05:49-06:06, default swept wing, machine loaded)

| step | seconds |
|---|---|
| generate (5 variants, physics) | 18.5 (was 61 before the polar/well-scan caching) |
| build_design_parts | 514.9 |
| export_step direct / from loaded parts | 3.0 / 3.8 |
| export_stl_parts direct / from loaded parts | 3.5 / 2.3 |
| build_design_solid | 504.1 |
| export_stl direct / from loaded solid | 1.3 / 1.4 (3.2 MB) |
| preview tessellation 0.05/0.12 -> 0.08/0.2 -> 0.12/0.25 | 6.8 s 18.5 MB -> 1.8 s 5.1 MB -> 1.4 s 3.2 MB |

Identity: one-piece STL sha256 IDENTICAL direct vs loaded solid. STEP body
from loaded parts DIFFERENT (607977 vs 608107 lines); parts zip: wing_left,
wing_right, elevon_left, elevon_right STLs differ. Consequence: the parts
path is NOT served from the BREP cache (`cadjobs._parts_for`); the solid
path is. Open: make the parts round-trip faithful (suspects: BinTools
dropping shared sub-shape identity across the compound, or triangulation
state changing which faces BRepMesh re-meshes) and re-run
the identity-run script from the profiling pass.
