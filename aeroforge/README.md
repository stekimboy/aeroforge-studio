# AeroForge — the application

This is the application-level guide: how to run it and what the sidebar does.
The project front page, the CFD study and the release log are one directory
up in the repository root `README.md`.

AeroForge designs RC aircraft. Give it an aircraft type, a mission, a cruise
speed and a hard size box; it solves the coupled weight ↔ aerodynamics ↔
stability problem, hands you five genuinely different aircraft to choose
between, renders the chosen one in 3D with CG / neutral-point / MAC markings,
and exports real CAD (STEP) and print-ready STL with the hardware already in
the part.

Every number in the UI comes from an actual calculation in `backend/physics/`.
Nothing is hard-coded, and nothing is a placeholder.

Six types are supported: `flying_wing` (the original and most mature path,
documented in depth below), `conventional`, `delta`, `canard`, `tandem` and
`twin_boom`. `SPEC_FLYING_WING.md` is the flying-wing design contract;
`RESEARCH_CONVENTIONAL.md` and `RESEARCH_TYPES_V3.md` hold the cited bands for
the others.

## Prerequisite (Windows 10/11, 64-bit)

Python **3.11 or 3.12** installed with **"Add python.exe to PATH"** checked
(from [python.org](https://www.python.org/downloads/), or
`winget install Python.Python.3.12`). Everything else is handled by the
launcher.

## How to run

**Double-click `run.bat`** in File Explorer. It will:
1. find Python (prefers 3.11, falls back to 3.12/3.13),
2. create the `.venv` virtual environment on first run,
3. install dependencies (first run only — takes a few minutes),
4. start the local server and open your browser at `http://127.0.0.1:8000`.

Close the minimized "AeroForge server" window to stop the app.

`run.ps1` is a PowerShell equivalent. If PowerShell blocks scripts, either run
`Set-ExecutionPolicy -Scope Process Bypass` first or just use `run.bat`.

## The four planform families

| Family | What it is | Real aircraft it comes from |
|---|---|---|
| **Swept sport wing** | 18–32° leading-edge sweep, taper 0.35–0.60, AR 3.5–5.2. Short and deep — the root chord *is* the fuselage. | Skywalker X5 (1280 mm span, 717 mm root), SonicModell AR Wing (900/482) |
| **Blended wing body** | 16–28° sweep, a deep blended centre body carrying a real payload bay, large canted winglets. | Ritewing Drak class |
| **Plank** | Almost unswept, nearly rectangular, AR 5–8. A strongly reflexed section does all the trimming. | PW-51, Zagi-plank |
| **Bell-distribution** | 20–34° sweep, strong taper, **8–13° of washout, and no vertical surfaces at all**. | Horten sailplanes, NASA Prandtl-D |

The bell spanload is worth understanding before you pick it: loading the tips
very lightly makes the tip region produce induced *thrust*, which yaws the wing
into its own turn (proverse yaw). That is why it needs no fins — and why adding
fins to one defeats the point.

## Using the sidebar

- **Preset** — load a ready-made wing, then press **Generate**.
- **Wing configuration** — planform family (or "Any", which lets the five
  variants span several shapes), mission (sport / FPV cruiser / thermal floater
  / park flyer), and the vertical surfaces that planform supports (wingtip
  winglets, inboard twin fins, a centre fin, or none).
- **Mission targets** — cruise speed (m/s ⇄ mph linked), optional stall-speed
  target, payload mass carried in the centre body.
- **Size box** — hard constraint in mm: the assembled wing must fit inside
  Length × Width × Height. The grey wireframe in the 3D view shows it, sitting
  on the ground under the model. Wings are *wide and short* — give it width.
- **Motors** — count only. AeroForge sizes airframes, not power systems; the
  red spheres in the 3D view mark where the motors go.
- **Structure** — 3D-printed (LW-PLA / PLA) or foamboard; drives the weight
  model and the recorded wall thickness.
- **Advanced** — reflexed-section override, aspect-ratio target, static-margin
  override (3–15% — a tailless wing flies at a lower margin than a tailed one).

Press **Generate** and you get **five wings**, not one: each is an independent
full solve against a different planform and a different set of targets, so they
differ in shape and in behaviour. Pick one from the gallery to load it into the
main view. If the request is infeasible (box too small for the cruise speed,
say) you still get the *closest feasible* design plus a banner naming the
binding constraint.

## What you get to build

The centre body is **hollow** — a real equipment bay with walls at the build
method's own thickness, its ceiling following the crown so the space is
usable. The spec panel gives you its length and width.

The top of that bay is a **removable hatch lid**. On the one-piece STL it stays
in place with its outline scribed as a line, because a two-piece STL would not
be one watertight model. In the STEP assembly and in **Export parts** it is its
own body, so you can print it separately.

There is a **structural motor mount** on the centre bulkhead: a boss whose
mounting face sits flush with the trailing edge (or the nose, on a plank) so it
never lengthens the aircraft, with the screw holes drilled straight through
boss, skin and bulkhead together. The spec panel prints everything you need to
check it against your motor — number of screw holes, hole diameter, bolt-circle
radius from the motor centre, the equivalent square pattern, centre bore and
plate thickness. Defaults come from the airframe's mass using the standard RC
outrunner patterns (16 × 16 mm for ~2204–2306, 19 × 19 for 2212–2814, 25 × 25
for 28xx and up, all on M3).

**Export STEP / Export STL / Export parts** write real CAD to `exports\`. STEP
is an assembly of named bodies; STL is one watertight solid in mm; **Export
parts** is a zip with one STL per part (`airframe.stl`, `hatch_lid.stl`,
`winglet_left.stl` …) plus a `parts.json` manifest — all in the same world
coordinates, so dropping them into a slicer reassembles the aeroplane exactly.

## How the physics works (implemented equations)

All in `backend/physics/`, cited in code comments:

- **Atmosphere** — ISA (`T = T0 − Lh`, `p = p0(T/T0)^{g/LR}`, `ρ = p/RT`),
  Sutherland viscosity, `Re = ρVc/μ`.
- **Airfoils** — a library of analytically-generated **reflexed** sections
  (positive `Cm0`, the property a tailless wing trims on) plus NACA 4-digit
  sections. Polars (Cl, Cd, Cm vs α, Re) from **NeuralFoil** (AeroSandbox) —
  real low-Reynolds predictions for the RC regime — with a thin-airfoil-theory
  + low-Re-correction fallback if it is unavailable.
- **Aerodynamics** — `L = ½ρV²S·CL`; cruise trim `L = W`; drag polar
  `CD = CD0 + CL²/(πAR·e)` with a component CD0 build-up; `AR = b²/S`;
  `V_stall = √(2W/(ρS·CLmax))` with the mission's stall margin enforced.
- **Stability** — a **spanwise strip model**: each strip contributes lift at
  its own local incidence and a pitching moment about its own aerodynamic
  centre, the neutral point is the area-weighted AC, the CG is placed at
  `x_NP − SM·MAC` (3–15% for tailless aircraft), and the washout is *solved* so
  the wing trims at cruise CL with `dCm/dα < 0`. Bell-spanload wings drive the
  washout to the bell target instead.
- **Vertical surfaces** — sized from real flying-wing practice (3–7% of wing
  area, interpolated on sweep), **not** from the tail-aft `V_V` band. On a
  tailless model the fin sits about one root chord behind the CG rather than a
  fuselage length, and solving the classic band on that arm demands a fin a
  quarter the size of the wing. Sweep does most of the directional work.
- **Weights** — areal-density structure model per build method, plus one
  documented lumped allowance for motor + ESC + pack (~28% of all-up mass).
- **Optimizer** — structured sweep over span / aspect ratio / sweep / taper +
  Nelder-Mead polish, minimising **specific drag** `D/W = 1/(L/D)` with
  hard-constraint penalties (box fit, stall margin, static margin, Reynolds
  floor) and proportion terms that hold the wing to real RC geometry.

`VALIDATION.md` holds an auto-generated physics audit table (one design per
planform family plus the five characters); `DECISIONS.md` records every
ambiguity resolution; `SPEC_FLYING_WING.md` is the design contract.

## The CAD is one continuous surface

A real moulded flying wing has no separate fuselage — the centre section *is*
the wing, its airfoil simply getting deeper and longer toward the root. So the
CAD lofts **one surface from tip to tip**, scaling section thickness and chord
up over the inboard blend and extending the leading edge forward at the centre.
There is no pod to bolt anything to, which is precisely why nothing looks
bolted on. Fins and winglets are separate parts (as they are on the real
aircraft) with their roots buried in the surface they stand on, so the whole
thing still fuses into exactly one watertight solid.

## Tests

```
.venv\Scripts\python.exe -m pytest tests -q
```

Covers ISA/Reynolds, reflexed-airfoil polars and thin-airfoil results, the drag
polar and stall equations, the strip model (NP, CG, static margin, trim
washout, sweep sensitivity), full-optimizer feasibility for all four planform
families, the proportion-realism checks that keep generated wings looking like
real ones, one-valid-solid / envelope / watertightness / mesh-coverage checks on
the CAD, STEP and STL export, and every API endpoint.

## Resolved dependency set (tested on Windows, Python 3.12)

AeroSandbox 4.2.10 · NeuralFoil 0.3.3 · cadquery 2.8.0 · casadi 3.7.2 ·
numpy 2.4.6 · scipy 1.18.0 · numpy-stl 4.0.0 · fastapi 0.141.1 ·
uvicorn 0.52.0 · pydantic 2.13.4 · pytest 9.1.1 · httpx 0.28.1
