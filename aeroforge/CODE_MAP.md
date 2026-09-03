# CODE_MAP.md - what lives where, and where to edit for what

Companion to `docs/ARCHITECTURE.md` (invariants and doctrine); the
current state is in the README. Line numbers are as of commit 8a3101a (2026-08-28) and
drift; function names are the stable handles - `grep -n "def name"`.
Python: `.venv/bin/python` on macOS, `.venv/Scripts/python.exe` on Windows.

## 1. Request flow (the four dispatch seams)

```
frontend/app.js  buildRequest()/buildRequestAlt()  ->  POST /api/generate  (backend/api.py:386)
  -> physics/variants.generate_variants()  [seam 1: _TYPE_VARIANTS table -> per-type generate_*_variants]
     -> physics/optimizer.optimize_design()  [seam 2: airplane_type]  (flying wing + delta)
        physics/conventional.optimize_conventional(), canard.py, tandem.py, twinboom.py (their own evaluate/optimize)
  <- 5 design dicts (variants) returned at once; CAD is NOT built yet
GET /api/preview/{id}.stl (api.py:441)  -> cadjobs.job_preview_stl -> cad/geometry.build_design_solid  [seam 3: _TYPE_MODULES]
GET /api/preview/{id}/parts.json (api.py:276) -> cadjobs.job_parts_previews -> cad/geometry.build_design_parts [seam 4]
POST /api/export/start (api.py:559) -> cadjobs.job_export -> exporters.export_step / export_stl / export_stl_parts
GET /api/export/status/{id}/{fmt} (api.py:606) <- progress.status_from_file (stage/progress/eta)
```

`geometry._TYPE_MODULES` (geometry.py:3072): `conventional` -> `cad/conventional.py`,
`canard`+`tandem` -> `cad/multiwing.py`, `twin_boom` -> `cad/twinboom.py`,
`flying_wing`+`delta` -> geometry.py itself. **A type in
`config_defs.AIRPLANE_TYPES` without a dispatch entry silently builds as a
flying wing** - `tests/test_type_dispatch.py` asserts the tables agree.

## 2. Backend modules

### backend/api.py (620 lines) - HTTP only, never imports cadquery
| what | where |
|---|---|
| `/api/types`, `/api/options` (type registry + `recommended_min_box_mm`) | `_airplane_types()`, routes at 127/134 |
| generate: validates `schemas.GenerateRequest`, calls variants, stores `_DESIGNS[id]`, records `generate:<type>` timing, warms previews | route 386, `_warm_previews` (minis one at a time, never while an export builds) |
| preview routes (one-piece STL, parts.json with `refusals`, per-part STL) | 276-441 |
| export: `/export` (sync, scripts only), `/export/start` (background thread, `_EXPORT_JOBS[key]`), `/export/status`, `/export/file`, `/timing` | 540-620; `_export_build`, `_export_state` |
| caches: `_EXPORTS` one file per (design, format), per-design lock so preview and export never build twice at once | `_export_key`, `_REGISTRY` |

### backend/cadjobs.py (285) - the worker-process boundary
`run_job` (2-worker spawn pool, `AEROFORGE_CAD_WORKERS`, `0` = inline),
`job_preview_stl`, `job_parts_previews`, `job_export` (brackets the build
with `progress.begin/end`), `build_refusals(meta)` (every `ok: false` entry
+ builder `warnings` -> the "Built with refusals" banner - open item: split
informational warnings out), `_parts_for` (builds fresh - parts cache
bypassed, not byte-identical), `_solid_for` (BREP cache hit for the
one-piece solid), `PREVIEW_TOL = (0.08, 0.2)` preview-only tessellation.

### backend/progress.py (265) - export stages + ETA
`STAGES`/`STAGE_LABELS`, worker-side `begin/report/end` writing
`exports/progress/<id>_<fmt>.json`, `TimingStore` (rolling median of 5 per
kind in `exports/timing.json`, learned stage fractions),
`status_from_file` (ETA held while `queued`). Builders call
`_progress("loft"|"bay"|...)` at stage boundaries (no-op outside a job).

### backend/cad/brepcache.py (238) - binary BREP cache keyed on design + digest of cad/*.py
`save_solid/load_solid` used; `save_parts/load_parts` exist but are not
served (see `_parts_for`). `AEROFORGE_BREP_CACHE=0` disables.

### backend/cad/exporters.py (237)
`export_step` (CadQuery assembly with colours, `design_assembly`),
`export_stl` -> `write_stl_verified` (tolerance ladder, mesh-area gate
>= 98.5 % of BRep area), `export_stl_parts` (zip + manifest),
`stl_is_valid_mesh`, `stl_watertight_fraction`.

### backend/cad/geometry.py (3170) - the flying-wing / delta airframe and the orchestrator
| edit here for | function(s) |
|---|---|
| section shapes, the one continuous full-span loft, tip cap, twist | `_Section`, `_BlendedWing` (241), `_blended_airframe` (503), `_station_groups`, `_rounded_surface` |
| motor mount boss + screw holes | `_motor_mount` (724), `_NACELLE_EMBED` |
| elevon separation and hinge grooves | `_elevon_grooves`, `_separate_elevons` (985), `_apply_grooves` |
| healing / fusing / cut existence checks | `_heal`, `_fuse_all`, `fuse_feature` (1298, classification-verified fuse with the retry ladder), `_tessellates_cleanly` (649), `_point_in_solid`, `_witness_inside` |
| centre fin, winglets, twin fins (construction) | `_vertical_surfaces` (1494), `_fin_dims` |
| fin root vs the bay cavity | `trim_root_to_cavity` (1337), `_FIN_ROOT_BURY_MM = 1.2` |
| what the bay is asked to be (start/length/width, extension target, elevon guard) | `bay_request` (1372) - the EXACT request `hatch.build_bay` gets; the probe tools call it |
| the build order for the tailless types | `_build_parts` (1615): loft -> boss -> bay/hatch -> fins (trimmed) -> hinges -> servos -> conduits -> split; `_probe=` seam returns raw fins + cavity for the probes |
| servo pocket placement per elevon | `_install_servos` (2008) |
| motor wire pipe (belly hole -> cavity) | `_motor_entry_run` (2297): walks entry candidates from the mount toward the published cavity band `cavity_stations_mm`; uses `conduits.straight_conduit` |
| servo wire pipes (perpendicular, level, into the cavity band) | `_servo_run` (2769); `_void_z_band` |
| legacy aft hollow (pre-round-12 fallback) | `_aft_hollow` (2532) |
| STEP part split (centre_body / wing_left / wing_right) | `_split_wing_panels` (3024) |
| public entry points (signatures frozen) | `build_design_parts` (3091), `build_design_solid` (3135), `airframe_extents_m` |

### backend/cad/hatch.py (2073) - equipment bay + hatch + magnets (root principle: the cavity IS the hull)
| edit here for | function(s) / constants |
|---|---|
| bay survey along the centre body (stations, crown/keel band, half-width) | `_survey` (527), `_Station`, `_profile_band`, `_band`, `_offset_band` |
| cavity extension aft of the hatch, ceiling ramp, wall thickness, elevon/TE guard | `_attempt` (1189) `extend_to` handling, `_EXT_RAMP_MM = 30`, `_guarded_hw` (1161), `_MIN_BAY_DEPTH_MM = 10` |
| the ladder of rungs (narrower / shorter cavities on boolean failure) and the existence gates | `build_bay` (1945), `_reject`, `_one_valid_solid`, `tessellates_cleanly`, `_round_tripped`, `_CORE_FLOOR_LIFT_MM = 1.0` (coincident-floor retry) |
| hatch lid (shelled against a dropped copy), aperture, seat, skirt | `_build_lid` (804), `LID_CLEARANCE_MM`, `SEAT_CLEARANCE_MM`, `_APERTURE_INSET_MM` |
| magnets: bore Ø8.15 (`MAGNET_FIT_MM` stays 0), forward pad, aft shelf on a riser | `_magnets` (905), `_fit_magnets` (1072), `_AFT_RISER_LEN_MM = 12` |
| lid outline scribe on the one-piece solid | `_build_scribe` |
| what `bay_mm` publishes (`x0_mm/x1_mm`, `hatch_x1_mm`, `cavity_extended_mm`, `cavity_stations_mm`, `magnet_centres_mm`, `aft_pad`, `warnings`) | end of `_attempt` / `build_bay`; consumers: `geometry._servo_run`, `_motor_entry_run`, `trim_root_to_cavity`, the probe tools |

### backend/cad/conduits.py (2408) - wire runs
`straight_conduit` (1414): ONE straight constant-section oval pipe
12.0 x 8.25 (`PIPE_W_MM`, `OVAL_W_RATIO`), `_line_ok` (1310, must stay
inside the skin except the intended breach; `_material_bands`), `_Nacelle`
(597, the motor fairing as material), `cut_conduit`/`cut_conduits`
(existence-checked cuts), `route_is_open`/`route_connects` (classification
of the finished pipe), `mesh_audit`/`skin_audit`. `motor_conduit` /
`servo_conduit` (899/1078) are the older swept-channel builders kept for
the non-flying-wing modules.

### backend/cad/servos.py (2133) - SG90 pockets, horns, linkage
`servo_bay` (846, pocket = measured `REAL SG90.stl` inverse + 0.25 mm per
face, `arm_y_mm` places the CASE so the ARM lands on a plane),
`control_horn` (1512, rounded beveled triangle, one Ø2.5 hole, `protrude_max_mm`
15, `rake_max_deg` 50, `align_world_y` solves its own station), `linkage`
(1903, parallelogram four-bar, `arm_r_mm` = `r_design_mm` = 11.0),
`HORN_SWING_ROOM_MM = 8`, `SERVO_CLEARANCE_MM = 0.25`.

### backend/cad/hinges.py (1620) - print-in-place hinges
`print_in_place_hinges` (1270), `bevel_control_surface` (RC double bevel),
`_cove_tool`, `clear_surface_ends` (1.00 mm end clearance),
`_size_barrel`, `_station_tools`/`_batched_tools` (falls back to one hinge
at a time - that fallback's note currently shows as a refusal),
`check_deflection_clearance`, `HINGE_DEFAULTS` (0.50 hinge gap, 0.35 pin).

### backend/cad/conventional.py (2249) - wing + fuselage + tail
`_FusProfile` (superellipse fuselage), `_ConvWing`, `_FusBayHost` (357, the
fuselage bay host used by `hatch.build_bay`), `_split_rudder` (487, +90 deg
build frame), `_split_ruddervator` (V-tail), `_t_tail_fin_run`,
`_spar_channel`, `_ballast_pocket`, `_bay_expansion` (1060, side trench /
floor chamber so servo runs can be straight), `_aileron_run`,
`_install_hardware` (1300), `_make_hosts`, `_build` (1684, `_probe=` seam),
`_FIN_TE_INSET_MM = 2.5` (fin root TE off the tail-cone end cap - the fin
was silently missing without it). Part names: airframe, aileron_l/r,
elevator_l/r, rudder, hatch_lid, cg_marker.

### backend/cad/multiwing.py (2124) - canard + tandem (biplane builder dormant)
`_PusherFus`, `_wing_lead_run`, `_install_hardware` (618), `_bay_and_hatch`
(910, `out=` exposes the cavity for probes), `_front_elevator`,
`_build_canard` (1155) / `_build_tandem` (1437), `_BUILDERS`/`_HOSTS`
tables, `make_hosts`, `build_design_parts/solid`.

### backend/cad/twinboom.py (1146)
`_PodProfile`, `_aileron_chain_tb`, `_make_hosts` (393), `_install_hardware`
(503), `_bore_sockets`, `_build` (803, `_probe=` seam), constants
`BOOM_BURY_MM`, `FIN_BURY_MM`, `STAB_TIP_OVERLAP_MM`, `PROP_TIP_MARGIN_MM`.

### backend/physics/
| module | role / where to edit |
|---|---|
| `config_defs.py` (602) | `PLANFORMS`, `MISSIONS`, `CONV_STYLES`, `AIRPLANE_TYPES` (axes per type, `recommended_min_box_mm` + derivation comments), `RECOMMENDED_MIN_BOX_MM`, `resolve_vstab` |
| `optimizer.py` (1105) | flying-wing/delta: `evaluate_candidate` (334, the whole coupled evaluation incl. servo chord floor, body proportions, motor layout), `optimize_design` (970, sweep + Nelder-Mead), `_motor_mount_spec`, `elevon_layout`, body constants `BODY_*`, `BAY_*_FRAC` (what the CAD bay is asked to be starts here) |
| `variants.py` (562) | the five characters (`_sport_swept` ... `_speed_delta`), `_TYPE_VARIANTS` table, `generate_variants` (444), untuned-feasible fallback, "any" mode `generate_any_variants_v3` |
| `stability.py` (508) | strip model NP/trim/washout, `flying_wing_fin` (259, fin sizing from FW practice - do not scale by V_V), `solve_flying_wing` |
| `stability_conv.py`, `conventional.py` (983) | tailed NP (Nelson 2.34), `evaluate_conventional` (125), `optimize_conventional`, fuselage/bay fractions `FUS_*`, `BAY_*` |
| `canard.py`, `tandem.py`, `twinboom.py`, `delta.py` | per-type evaluate/optimize/variants; `twinboom.servo_chord_floor_m` / `section_well_frac` (servo pocket fit -> minimum chord; restates 4 numbers from cad/servos.py - `test_twinboom_servo_pocket_constants_match_the_cad`) |
| `config_axes.py` (387) | `apply_tail_type`, `apply_wing_position`, `apply_motor_layout`, `engine_out_check` - currently NO physics consumers (v3.1 gap) |
| `airfoils.py`, `aero.py`, `atmosphere.py`, `weights.py`, `handling.py`, `guidance.py` | polars (NeuralFoil + cached arrays `_polar_arrays`), drag build-up, ISA, mass, 3-axis handling checks, builder advice/Learn text |

### backend/schemas.py (174)
`GenerateRequest` (validation, `to_optimizer_input()`), `ExportRequest`,
`GenerateResponse`. Add a request field here first, then `buildRequest*` in
app.js, then the consumer.

## 3. Frontend (frontend/)
| edit here for | app.js |
|---|---|
| type switching, axis rows, planform/mission/style selects, presets, `/api/options` load | `syncAirplaneType` (475), `renderConfigRow`, `syncPlanform`, `syncMission`, `applyPreset`, `loadOptions` (593) |
| request assembly | `buildRequest` (733, flying wing), `buildRequestAlt` (784, other types) |
| spec panel, design notes, Learn overlay | `showSpecs` (837), `showGuidance`, `openLearn` |
| banners incl. "Built with refusals" and "Closest feasible wing" | `showBanner`, `showRefusals` (1378), `failBanner` |
| variant gallery + selection (loads preview STL, parts.json) | `renderVariants` (1430), `selectVariant` (1507) |
| generate flow | `generate` (1572) |
| dirty tracking, confirm modal, export guard, recommended box hint | `canonical`/`formIsDirty`/`refreshDirty` (1677-1690), `requestChanges`, `openConfirm`, `guardedGenerate` (1821), `guardedExport` (1846), `syncBoxReco` (1883) |
| progress bar + ETA | `refreshTiming`, `showProgress`/`hideProgress` (1904-1932) |
| export buttons | `doExport` (1971) |
`viewer.js`: `Viewer` (three.js stage, CG/NP/MAC/motor/box overlays,
`MARKERS`), `MiniPreviewPool` (shared-context gallery previews),
`loadGeometry`. `index.html`: three columns `#datapanel` / `#workspace` /
`#sidebar`, `#box_reco`, `#confirm_modal`, `#build_progress`. `style.css`:
dark-navy tokens; `#sidebar .note` outranks plain class selectors (add
`#sidebar` to the selector when styling sidebar text).

## 4. Tests and tools - what proves what
| claim | run |
|---|---|
| registry and dispatch tables agree (5 s, never builds) | `pytest tests/test_type_dispatch.py` |
| physics bands, stability, optimizer | `test_optimizer.py`, `test_stability.py`, `test_aero.py`, `test_atmosphere.py`, `test_conventional_physics.py`, `test_multiwing_physics.py`, `test_boomglider_physics.py` |
| full flying-wing CAD invariants (one solid, bbox, mesh area, hinge/servo/horn/pipe rules, rear hollow, motor belly entry) - ~45 min, use `-k` | `test_cad_export.py` (`-k "motor or straight_tube or rear_hollow"` = 16 tests, 30 s) |
| other types' CAD (full builds, 30 min each) | `test_conventional.py`, `test_multiwing.py`, `test_twinboom.py`, `test_delta_cad.py` |
| bay ladder carves + classifies as air | `test_bay_ladder.py` (2.5 min) |
| fins do not enter the cavity | `test_fin_intrusion.py`; sweep: `tools_probe_fin_intrusion.py` |
| cavity extension on every config | `tools_probe_cavity.py` (16 configs) |
| validity across every UI axis (52 configs, `--full` for 3 full builds) | `tools_probe_all.py`; `test_probe_all.py` |
| refusal plumbing, progress/ETA store, BREP cache, recommended box | `test_refusals.py`, `test_export_progress.py`, `test_brepcache.py`, `test_recommended_box.py` |
| a delivered STEP is right (classification of the file itself) | `tools_audit_magnet_pad.py <file.step>`, `tools_audit_rear_hollow.py` (slow); the centre-wall column scan and the belly-hole/pipe check used on `deliverables/final.step` were one-off scripts built on `tools_audit_magnet_pad.py column()` and are not in the repo |
| API end to end | `test_api.py` (builds CAD - slow) |
| one build at a time | wrap any build in `tools_cadlock.py <cmd>` |

Probe-level (loft + boss + bay + fins, ~3 min) CANNOT see hinge / servo /
wire-run refusals - only a full build's `refusals` sidecar
(`exports/previews/<id>__refusals.json`) or classification of the export can.

## 5. Recipes

- **Change a hardware clearance**: the constant lives in the module that owns
  the feature (`hinges.HINGE_DEFAULTS`, `servos.SERVO_CLEARANCE_MM`,
  `hatch.MAGNET_*`, `conduits.PIPE_W_MM`); cite the source in the comment;
  prove with the matching `test_cad_export.py -k` subset and DECISIONS.md.
- **Change how big the bay is**: physics first (`optimizer.BAY_*_FRAC`,
  `conventional.BAY_*`), then `geometry.bay_request`; never clamp in hatch.py.
- **Add a request field**: `schemas.GenerateRequest` -> `to_optimizer_input`
  -> consumer in physics -> `app.js buildRequest*` -> `index.html` control
  -> `labelFor` so the confirm dialog names it.
- **Add an aircraft type**: `config_defs.AIRPLANE_TYPES` + `variants._TYPE_VARIANTS`
  + `geometry._TYPE_MODULES` + a `test_type_dispatch` row + a probe config.
- **Debug a bay that will not carve**: `AEROFORGE_HATCH_TRACE=1` prints the
  ladder; `AEROFORGE_HATCH_DUMP=<dir>` writes the cutter pieces.
- **Prove an export**: import the STEP, classify columns (see
  `tools_audit_magnet_pad.py column()`), never trust `bay_mm` alone.
