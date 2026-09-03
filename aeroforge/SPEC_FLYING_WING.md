# AeroForge v2 — flying-wing specialist

**Binding contract for the rewrite.** AeroForge designs **flying wings only**.
Every other configuration (conventional tail-aft, V-tail, twin-boom, canard) is
removed — not hidden, removed: config defs, optimizer branches, CAD branches,
stability solvers, UI options, guidance text and tests.

---

## 1. The problem we are fixing

The old flying-wing output read as **separate objects stitched together**: an
ellipsoidal pod sitting on top of two wing lofts, with fins bolted on. Real RC
flying wings are not built that way and do not look that way. Two root causes:

**(a) Wrong proportions.** Real FPV/sport wings are *short and deep*, not
glider-like. Measured airframes:

| Aircraft | Span | Root length | Root/span | Wing area | AR |
|---|---|---|---|---|---|
| Skywalker X5 Pro | 1280 mm | 717 mm | 0.56 | 44 dm² | 3.7 |
| SonicModell AR Wing Classic | 900 mm | 482 mm | 0.54 | ~20 dm² | ~4.0 |
| Zagi-class slope wing | 1200 mm | ~430 mm | 0.36 | — | ~5 |

The optimizer was producing AR 5–6 with a root chord ~0.20 × span, then adding
a pod because the root was too thin to hold anything. **The centre chord must
be long enough to BE the fuselage.**

**(b) Wrong topology.** The body was a separate solid. On a real moulded wing
the centre section *is* the wing — the airfoil simply gets deeper and longer
toward the root, and the surface is continuous from tip to tip. The new CAD
must build the airframe as **one continuous full-span loft**, not a pod plus
two panels.

---

## 2. Real-world reference data (design inspiration, encoded)

- **Swept sport / FPV wing** (Skywalker X5, SonicModell AR Wing, Reptile S800):
  LE sweep 20–30°, taper 0.35–0.55, AR 3.5–5.0, root chord 0.45–0.58 × span,
  reflexed section ~9% thick, washout 2–5°, tip winglets or inboard twin fins,
  single pusher motor at the centre trailing edge.
- **Blended wing body / long-range cruiser** (Ritewing Drak class): LE sweep
  18–26°, taper 0.45–0.65, AR 4.0–5.5, a deep centre body (2.2–3.0 × the wing's
  own thickness ratio) carrying the payload bay, large canted winglets.
- **Plank** (PW-51, Zagi-plank): LE sweep 0–6°, taper 0.75–1.00, AR 5.0–8.0,
  strongly reflexed section doing all the trimming, little or no washout, one
  centre fin. Looks like a rectangular door — that is correct.
- **Bell-distribution / Horten** (NASA Prandtl-D, Horten sailplanes): LE sweep
  22–34°, taper 0.20–0.40, AR 6.0–11.0, **washout 8–13°** (the bell spanload
  needs a large root-to-tip twist), and **no vertical surfaces at all** — the
  tip-region induced thrust gives proverse yaw, which is the whole point of the
  configuration. Never put fins on this one.

Sources for the reference numbers are listed at the bottom of this file.

---

## 3. Domain model

### 3.1 Planform families (replaces `config`)

`backend/physics/config_defs.py` exposes `PLANFORMS: dict[str, PlanformDef]`:

| key | label | sweep band | taper band | AR band | washout band | vstab options (first = default) |
|---|---|---|---|---|---|---|
| `swept` | Swept sport wing | 18–32 | 0.35–0.60 | 3.5–5.2 | 2–5 | `winglets`, `twin_fin`, `center_fin` |
| `bwb` | Blended wing body | 16–28 | 0.42–0.68 | 3.8–5.6 | 2–4 | `winglets`, `twin_fin` |
| `plank` | Plank | 0–8 | 0.70–1.00 | 5.0–8.0 | 0–2 | `center_fin`, `winglets` |
| `bell` | Bell-distribution (Horten) | 20–34 | 0.20–0.42 | 6.0–11.0 | 8–13 | `none` |

`PlanformDef` fields: `name, label, description, sweep_band, taper_band,
ar_band, washout_band, body_depth_band, root_chord_frac_band, oswald_mult,
vstab_options, bell_spanload: bool, blurb`.

`bell_spanload=True` for `bell` only: washout is driven to the bell target
instead of the minimum trim value, and vertical surfaces are forbidden.

### 3.2 Missions (replaces `style`)

`MISSIONS: dict[str, MissionDef]` — `sport`, `fpv_cruiser`, `thermal_floater`,
`park_flyer`. Fields as the old `StyleDef` (`static_margin`, `wl_band`,
`stall_factor`, `default_cruise_ms`) plus `preferred_planforms: tuple[str,...]`
used only to order the UI, never to restrict.

Static-margin note: flying wings fly at **lower** static margin than tailed
models — the band is 0.03–0.15 with defaults 0.05–0.10, NOT the old 0.04–0.20.

### 3.3 Vertical surfaces

`winglets` (2, at the tips, canted 8–20° outboard) ·
`twin_fin` (2, inboard at 0.50–0.65 semi-span, X5 style, vertical) ·
`center_fin` (1, on the centre body spine) ·
`none` (bell spanload only).

Sizing stays in `stability.flying_wing_fin` (already correct: 3–7% of wing area
interpolated on sweep, guarded against chord and semi-span). Extend it with the
`twin_fin` inboard case and a `none` case returning zeros.

---

## 4. The design dict — THE CONTRACT

`optimize_design` returns exactly this. The CAD reads **only** `geometry` plus
`stability.x_cg_m`; the frontend reads `geometry`, `aero`, `stability`,
`mass`, `power_system`, `constraints`, `guidance`, `character`.

```python
{
  "id": str,
  "planform": "swept"|"bwb"|"plank"|"bell",
  "planform_label": str,
  "mission": "sport"|"fpv_cruiser"|"thermal_floater"|"park_flyer",

  "geometry": {
    # --- planform -----------------------------------------------------
    "span_m": float, "area_m2": float, "aspect_ratio": float,
    "taper": float,                  # c_tip / c_wing_root
    "sweep_le_deg": float,           # of the WING panel (outboard of the body)
    "dihedral_deg": float,           # 0-4; wings mostly use 0
    "washout_deg": float,            # root incidence minus tip incidence
    "root_incidence_deg": float,
    "root_chord_m": float,           # WING root chord (at the body/wing joint)
    "tip_chord_m": float,
    "airfoil": str,                  # reflexed section (RFX-*)
    "fin_airfoil": str,              # symmetric section for fins

    # --- blended centre body (NOT a separate pod) ---------------------
    # The body is the inboard part of the same surface: sections get deeper
    # and longer toward y = 0 and blend out by `half_width_m`.
    "body": {
      "half_width_m": float,         # spanwise half-extent of the blend
      "depth_scale": float,          # root thickness / wing thickness (1.8-3.2)
      "chord_scale": float,          # root chord / wing root chord (1.10-1.45)
      "nose_round": float,           # 0-1, how bluntly the centre LE is faired
      "crown_frac": float,           # 0.5-0.75 of the extra depth that goes UP
      "canopy": bool,                # faired hatch on the spine
      "bay_length_m": float,         # usable equipment bay length, informational
    },

    # --- vertical surfaces -------------------------------------------
    "vstab": {
      "type": "winglets"|"twin_fin"|"center_fin"|"none",
      "count": int, "label": str,
      "area_total_m2": float,        # 0.0 when type == "none"
      "height_m": float,             # per surface
      "root_chord_m": float, "tip_chord_m": float,
      "sweep_le_deg": float,         # 20-35 typical
      "cant_deg": float,             # outboard cant, 0 for vertical fins
      "y_frac": float,               # spanwise station, fraction of semi-span
    },

    # --- controls, power, envelope ------------------------------------
    "elevons": {"inner_frac": float, "outer_frac": float, "chord_frac": float},
    "motors": [{"x": float, "y": float, "z": float, "type": "pusher"|"tractor"}],
    "battery_x_m": float, "battery_x_required_m": float,
    "length_total_m": float,         # nose datum x=0 to aft-most point
    "height_total_m": float,         # belly to highest point
    "control_surfaces": ["elevon_left", "elevon_right"],
    "wall_mm": float, "build_method": str,
  },

  "aero":   {... unchanged from v1 ...},
  "stability": {... unchanged, plus "bell_spanload": bool,
                 "fin_height_m", "fin_chord_m", "x_fin_ac_m" ...},
  "power_system": {...}, "mass": {...},
  "constraints": [...], "feasible": bool, "binding": [...],
  "notes": [...], "cost": float,
  "guidance": [...], "character": {"key","name","tagline"},
}
```

**Datum:** x = 0 at the forward-most point of the aircraft (the centre body
nose). Everything (CG, NP, motors, stations) is measured aft from there. No
geometry may extend to x < 0.

---

## 5. CAD topology — one continuous surface

`backend/cad/geometry.py` must build the airframe as a **single full-span
loft** from left tip to right tip. There is no separate fuselage solid.

Section at spanwise station `y` (let `f = |y| / (span/2)`, `fb =
body.half_width_m / (span/2)`):

- **blend** `w(f) = smoothstep(1 → 0)` over `f ∈ [0, fb]`, 0 outboard.
- **chord** `c(f) = c_wing(f) · (1 + (body.chord_scale − 1)·w(f))`, where
  `c_wing(f)` is the linear taper `c_root → c_tip`.
- **thickness scale** `t(f) = 1 + (body.depth_scale − 1)·w(f)` applied to the
  airfoil ordinates (NOT to the chord).
- **LE x** `x_le(f) = f·(span/2)·tan(sweep) − nose_ext·w(f)`, so the centre
  body's nose reaches forward of the wing-root LE and the planform LE curves
  smoothly into it (a real BWB leading-edge root extension).
- **section z-centre** offset upward by `(t(f) − 1)·thickness·(crown_frac −
  0.5)`, so the body's crown rises more than its keel drops — as on a real
  moulded wing.
- **twist** linear washout from `root_incidence_deg` at `f = 0` to
  `root_incidence_deg − washout_deg` at `f = 1`.

Loft through ≥ 19 stations, clustered near `f = 0…fb` where curvature is
highest, in **axial segments sharing boundary wires** (the proven trick already
used for the old fuselage: it keeps OCC's through-sections from overshooting
and keeps every trimmed face simple enough for the mesher).

Named parts returned by `build_design_parts`:
`airframe` (the full-span blended wing — always present), then any of
`winglet_left` / `winglet_right` / `fin_left` / `fin_right` / `fin`,
plus `canopy` (optional) and `cg_marker`.

`build_design_solid` fuses the same parts, heals, cuts the elevon grooves, and
must return **exactly one valid solid**.

### Fin construction
Fins are separate bolted-on parts (correct — real wings bolt or slot them on),
but their roots must be buried in the surface they stand on so the fuse yields
one solid. Winglets: at the tip, TE flush with the wing tip TE, cant outboard.
Twin fins: at `y_frac` of the semi-span, vertical, root buried below the wing
skin. Centre fin: on the body spine, TE at or slightly behind the body TE.

---

## 6. Module responsibilities after the rewrite

| File | Responsibility |
|---|---|
| `physics/config_defs.py` | `PLANFORMS`, `MISSIONS` only. No tailed configs. |
| `physics/stability.py` | `solve_flying_wing` (strip model), `flying_wing_fin`, bell-spanload washout solve. `solve_tail_aft` and `solve_canard` **deleted**. |
| `physics/optimizer.py` | Search over (span, AR, sweep, taper). No tail volumes, no wing-position solve, no fuselage sizing — the body IS the wing. |
| `physics/variants.py` | Five flying-wing characters (§7). |
| `physics/airfoils.py` | Reflexed sections are the primary library; add 2 more reflex thicknesses. `pick_airfoil` takes (planform, mission, override). |
| `physics/guidance.py` | Wing-only guidance + Learn articles. |
| `cad/geometry.py` | §5. |
| `schemas.py`, `api.py` | `planform` + `mission` replace `config` + `style`. |
| `frontend/*` | Planform / mission / vstab selectors; no aircraft-type dropdown. |

## 7. The five variant characters

1. `sport_swept` — **Swept Sport** (`swept`, AR ~4.2, sweep ~24°, winglets)
2. `bwb_cruiser` — **Long-Range Cruiser** (`bwb`, deep body, big winglets, payload)
3. `bell_horten` — **Bell-Distribution Floater** (`bell`, no fins, high washout)
4. `plank_park` — **Plank Park Flyer** (`plank`, centre fin, slow)
5. `speed_delta` — **Speed Delta** (`swept`, AR ~3.4, sweep ~30°, small fins)

Traits stay min-max normalized across the set.

## 8. Invariants (tests must enforce)

1. Exactly one valid watertight solid per design; mesh area ≥ 98.5% of BRep.
2. Nothing at x < 0; CAD bbox ≤ recorded `span` / `length_total_m` /
   `height_total_m` + 2 mm, and the recorded totals ≤ the user's box.
3. CG ahead of NP, static margin 0.03–0.15, `dCm/dalpha < 0`.
4. `V_cruise ≥ stall_factor × V_stall`; L = W at cruise.
5. Reflexed section (positive `Cm0`) on every design.
6. **Proportion realism** (new — this is what "looks right" means):
   root chord ≥ 0.30 × span for `swept`/`bwb`; body `chord_scale` ≥ 1.10;
   the centre body must be deep enough to hold the pack
   (`body.depth_scale × t/c × root chord ≥ 28 mm`).
7. Vertical surfaces 2–9% of wing area, height ≤ 60% of the chord they stand
   on; `bell` designs have **zero** vertical area.
8. Five variants always returned, ≥ 3 distinct planform families among them.

## 9. Resolved during implementation

Points where this spec was under-specified or wrong, and what was actually
built. `DECISIONS.md` carries the full reasoning.

- **§4 `root_chord_m` is the trapezoidal WING root chord**, measured at the
  body/wing joint — *not* the centre-body length. The X5's quoted 717 mm on a
  1280 mm span (0.56) is the centre length including the leading-edge root
  extension; divide `body.chord_scale` back out and the wing root chord is the
  0.30–0.45 × span that `root_chord_frac_band` constrains. Invariant §8.6 reads
  the wing root chord.
- **§3.1 and §8.3 conflict for `bell`** — 8–13° of washout on a swept AR-6.5
  wing at cruise CL ≈ 0.2 trims above the 0.15 static-margin ceiling. The
  ceiling wins; the residual is carried on elevon trim and reported.
- **`body` also carries `nose_ext_m`** (derived: `(chord_scale − 1) ·
  root_chord`), so the CAD cannot diverge from the physics on how far forward
  the leading-edge root extension reaches.
- **`stability` dropped** `vh, s_h, l_h, vtail_dihedral_deg, s_vtail_total`
  (no horizontal surface exists) and **gained** `bell_spanload, fin_height_m,
  fin_chord_m, x_fin_ac_m, cm_trim_residual, cm0_wing, washout_deg, s_v_m2,
  l_v_m`. `mass` gained `nose_ballast_kg`.
- **`planform: "any"`** is a valid wire value (the sidebar default), normalised
  to `None` at the schema boundary.
- **The airfoil's physical leading edge sits 0.03–0.15 mm ahead of the chord
  line**, so a literal §5 `x_le` puts a sliver at x < 0. Both the physics
  planform and the CAD normalise the forward-most point to x = 0.

## Sources for the reference geometry

- Skywalker X5 Pro specification (1280 mm span, 717 mm length, 44 dm²) —
  https://www.uavmodel.com/products/skywalker-x5-pro-1280mm-uav-fixed-wing
- SonicModell AR Wing Classic 900 mm (900 mm span, 482 mm length) —
  https://www.getfpv.com/sonicmodell-ar-wing-classic-900mm-wingspan-epp-flying-wing-rc-airplane-kit-version.html
- Bell-shaped spanload, washout magnitude and proverse yaw —
  NASA Prandtl-D, https://ntrs.nasa.gov/api/citations/20210014683/downloads/H3284FINAL.pdf
