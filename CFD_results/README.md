# AeroForge CFD — Virtual Wind Tunnel Study

External aerodynamics of three AeroForge-generated RC airframes, run as full 3D cases on SimScale. The cropped, named renders (streamlines, surface pressure, force-convergence plots) are in [`../docs/media/`](../docs/media/); the raw full-screen captures are not in the repo, but the three SimScale projects are public and linked at the bottom, so every view can be reopened live.

<p align="center"><img src="../docs/media/flying_wing_centreline_streamlines.png" width="70%" alt="Flying wing, centreline streamlines"></p>

## Conditions

| Parameter | Value |
|---|---|
| Freestream velocity | 20 m/s |
| Angle of attack | 0° (relative to each STL's own x-datum — see note) |
| Fluid | Air, ρ = 1.196 kg/m³, ν = 1.529×10⁻⁵ m²/s |
| Dynamic pressure q | 239.2 Pa |
| Reynolds number | ≈ 2×10⁵ (chord-based) |
| Solver | Incompressible steady RANS, SIMPLE |
| Turbulence model | k-ω SST |
| Domain | 3.5 × 2.0 × 1.4 m (x −1→2.5, y ±1, z ±0.7) |
| Boundary conditions | Velocity inlet (Min X), pressure outlet 0 Pa (Max X), slip walls (sides/top/bottom), no-slip aircraft |
| Blockage ratio | < 1 % |
| Model scale | 1:1, 650 mm span, imported in mm |
| Iterations | 1500 (Flying Wing, Tandem), 1000 (Conventional); forces converged to 4 decimals over the last 100 |

## Results

Forces on the airframe, converged (mean of last 100 iterations = final value to 4 decimals).
Body axes: **Fx = drag** (streamwise), **Fy = side**, **Fz = lift**.

| Airframe | Lift Fz (N) | Drag Fx (N) | Side Fy (N) | L/D | Lift (gram-force) |
|---|---|---|---|---|---|
| Conventional | **+8.882** | **1.222** | 0.015 | **7.27** | 905 |
| Tandem Wing  | **+11.086** | **1.666** | 0.004 | **6.65** | 1130 |
| Flying Wing  | **−3.400** | **0.966** | 0.018 | — | −347 |

### Drag breakdown

| Airframe | Pressure (form) | Viscous (skin friction) | Viscous share |
|---|---|---|---|
| Conventional | 0.919 N | 0.302 N | 25 % |
| Tandem Wing  | 1.201 N | 0.464 N | 28 % |
| Flying Wing  | 0.649 N | 0.317 N | 33 % |

### Sanity checks

- **Side force ≈ 0** on all three (0.004–0.018 N against 1–1.7 N of drag) — the symmetric geometry produced a symmetric solution, as it must.
- **Stagnation pressure** peaks at +265 Pa (Conventional) and +245 Pa (Flying Wing) against a theoretical q of 239.2 Pa. Cp_max ≈ 1.0, exactly as expected.
- The Flying Wing's higher viscous share is consistent with an all-wing layout: more wetted area per unit frontal area, less form drag.

### Note on the Flying Wing's negative lift

This is a real result, not an error. AeroForge flying wings use **reflexed** airfoil sections plus washout so the aircraft can trim in pitch without a tail. That makes the STL's x-datum sit above the zero-lift line, so at 0° to the datum the wing produces downforce; its trim angle of attack is positive. The Conventional and Tandem have their wings mounted at positive incidence relative to the fuselage datum, so they lift at 0°.

**The three are therefore not directly comparable at a single datum angle.** A proper comparison needs an angle-of-attack sweep, re-running each case with the inlet velocity vector rotated. That sweep has not been run.

## Geometry preparation

The source STLs are print-ready models: 0.5 mm print-in-place hinge gaps, scribed hatch outlines, servo pockets, and razor trailing edges. A CFD surface mesher cannot resolve sub-millimetre features at any practical cell count — they appear as self-intersections and produce zero-volume sliver cells.

Two failure modes were hit and fixed:

1. **Meshing failed** on the Conventional (self-intersection at a 0.46 mm scribed hatch outline, x=122 mm) and the Flying Wing (razor trailing edge, x=289 mm).
2. **Solver diverged** on the Flying Wing and Tandem — each time at the *identical cell*, which is the signature of a degenerate cell rather than a numerical instability. Lowering relaxation only delayed it.

Both were resolved by applying a **surface wrap** to each model before extracting the flow volume, producing a clean watertight outer shell. The Conventional wrapped at resolution 7; the Flying Wing and Tandem needed resolution 5 to blunt the trailing edge enough to mesh. This slightly increases trailing-edge base drag but preserves the boundary-layer physics — a far smaller compromise than dropping the boundary layers would have been.

Final runs additionally used damped numerics (relaxation αp = 0.3, αU = 0.65, two non-orthogonal correctors, 1500 iterations).

## Post-processing recipe

The captures were taken with one repeatable setup per project, so the three airframes read on the same scale:

1. Open the project → `SIMULATIONS ▸ Incompressible ▸ Simulation runs` → the **green** run (red ones are the failed meshing / divergence attempts) → **Solution Fields**.
2. If the opaque domain box hides the aircraft: click the airframe → right-click → **Isolate selection** (twice if the first click grabs a box face) → **Clear selection**. `Parts Color` draws the flow region's *outer boundary* by default, not the aircraft.
3. `Parts Color` → Coloring = **Pressure**.
4. Pin the legends by double-clicking their end values: **Pressure −160 → +80 Pa**, **Velocity 14 → 25 m/s**. The auto ranges fit to single sliver cells at the wrapped trailing edge (Tandem: −3218 → +255 Pa) and render the whole airframe one flat colour.
5. `Particle Trace` at ~1.1 mm cylinder radius; `Pick Position` on a temporary cross-flow cutting plane through the aircraft's frontal section (then hide the plane) to seed a rake that actually wets the wing. Fewer, thinner lines read better than a dense sheet.
6. Note the cutting-plane orientation differs between projects: the centreline slice is normal **Y** in the Flying Wing project and **Z** in the Conventional and Tandem ones (different CAD modules, different body-axis conventions). Avoid the **Reset** button — it wipes the post-processing session.

## Files

| Path | Contents |
|---|---|
| `../docs/media/<airframe>_*.png` | Cropped renders: centreline / planform / iso streamlines, one-wing rakes, surface pressure (planform / front / side / iso), one set per airframe |
| `../docs/media/<airframe>_force_convergence.jpg` | All nine force components vs iteration, cropped to the chart |
| `../docs/crop_media.ps1` | The crop boxes used to make the renders from the raw SimScale captures (the raw captures themselves are not in the repo) |
| `../CFD_STLs/` | The three source STLs as exported by AeroForge |

## Live projects

All three SimScale projects are public — the full 3D result, mesh and solver logs can be opened in a browser:

- Conventional: https://www.simscale.com/workbench/?pid=5552874111724692651
- Tandem Wing: https://www.simscale.com/workbench/?pid=7351757414170498497
- Flying Wing: https://www.simscale.com/workbench/?pid=6116530240019432829
