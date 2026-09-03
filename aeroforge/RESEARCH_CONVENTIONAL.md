# RESEARCH_CONVENTIONAL.md — design-rules dossier for conventional RC airplanes

Scope: wing + fuselage + horizontal & vertical stabilizers, tractor prop, hobby scale
(600–1600 mm span), 3D-printed PLA/LW-PLA, 9 g servos. Every number carries its source.
Where sources disagree the spread is recorded (same convention as the `_spread` pattern in
`backend/cad/servos.py`). "Derived" marks a number computed here from cited inputs — usable,
but not itself a published figure.

Primary references used throughout:

- **[MIT]** Drela, "Basic Aircraft Design Rules", MIT 16.01/16.02 Lab 11 notes —
  https://web.mit.edu/16.unified/www/SPRING/systems/Lab_Notes/desrules.pdf
- **[ND]** "Trainer Design" course handout, Univ. of Notre Dame AME 40462 (classic RC
  trainer proportion diagram) — https://sites.nd.edu/ame40462/files/2018/03/TrainerScaling.pdf
- **[RCG]** "Tail Design and Sizing" (RCGroups design-guide attachment) —
  https://www.rcgroups.com/forums/showatt.php?attachmentid=4869670
- **[MA]** "Tails for Models", Model Aviation library —
  https://library.modelaviation.com/article/tails-models
- **[SCHOLZ]** Scholz et al., "Empennage sizing with the tail volume…", INCAS Bulletin 13(3)
  2021 (reproduces Raymer's *Aircraft Design* Table 6.4 tail-volume values) —
  https://www.fzt.haw-hamburg.de/pers/Scholz/Aero/AERO_PUB_INCAS_TailVolume_Vol13No3_2021.pdf

---

## 1. Reference airframes (real dimensions)

| Airframe | Span [mm] | Length [mm] | Wing area [dm²] | Mean chord [mm] (derived S/b) | AR (derived b²/S) | AUW [g] | Wing loading [g/dm²] | Source |
|---|---|---|---|---|---|---|---|---|
| E-flite Apprentice STS 1.5m (foam trainer) | 1499 | 1080 | 33.2 (515 in²) | 222 | 6.8 | 1390 | 41.9 (derived) | [HobbyTown/E-flite spec](https://www.hobbytown.com/eflite-apprentice-sts-1.5m-rtf-basic-smart-trainer-electric-airplane-1500mm-efl370001/p1465508) |
| HobbyZone AeroScout S 2 1.1m (foam trainer) | 1095 | 870 | n/p | — | — | 788–836 flying | — | [HobbyZone spec](https://www.hobbyzone.com/products/hobbyzone-aeroscout-s-2-1.1m-rtf-basic-hobby-zone) |
| FT Simple Cub (foam-board trainer) | 956 | — | n/p | ~203 (8 in constant chord, plans) | ~4.7 | 408 w/o battery; ~540 with 3S 1300 (derived) | — | [FliteTest spec](https://flitetest.co.za/products/ft-simple-cub-mkr2), [plans PDF](http://s3.amazonaws.com/plans.flitetest.com/FT_Simple_Cub_v1.0_Full-Size.pdf) |
| Eclipson Model A (3D-printed trainer) | 1000 | 760 | 16 | 160 | 6.6 (published) | 490 (LW-PLA) / 730 (PLA) | 30 / 45 (published) | [Eclipson](https://www.eclipson-airplanes.com/modela) |
| Eclipson Model A XL (3D-printed trainer) | 1400 | — | 30 | 214 | 6.6 (published) | 900 (LW-PLA) / 1400 (PLA) | 30 / 45 (published) | [Eclipson](https://www.eclipson-airplanes.com/modela14) |
| 3DLabPrint Piper J-3 Cub (3D-printed scale) | 1068 | 675 | 18.15 | 170 | 6.3 (derived) | ~600 with electronics (LW-PLA) | 31 (published) | [air-rc spec sheet](https://www.air-rc.com/aircraft/3D-LabPrint_Piper-J-3-Cub_j3cub), [3DLabPrint](https://3dlabprint.com/shop/piperj3cub/) |

Notes usable in code:

- Tail arm / stab / fin areas are not published by the foam-RTF manufacturers; size them
  from the volume-coefficient bands in §2, not by copying a spec sheet.
- CG anchors: FT Simple Cub CG = 45 mm from wing LE on a ~203 mm chord = **22% chord**
  ([FliteTest](https://flitetest.co.za/products/ft-simple-cub-mkr2)); [ND] general trainer rule
  CG = **25–33% of chord**.
- Length/span ratios from the table: Apprentice 0.72, AeroScout 0.79, Eclipson A 0.76,
  J-3 Cub 0.63 (all derived). See §6.

## 2. Tail volume coefficients and tail arm

**Horizontal tail volume** `V_H = (S_h × l_h) / (S × c_mac)`, l_h = CG → stab quarter-chord [MIT].

| Source | Category | V_H |
|---|---|---|
| [MIT] | any well-behaved model | **0.30 – 0.60** |
| [RCG] | powered sport / pattern | 0.50 – 0.80 |
| [RCG] | scale models | 0.35 – 0.60 |
| [RCG] | gliders | 0.30 – 0.50 |
| [SCHOLZ] (Raymer Table 6.4, full-scale) | homebuilt | 0.50 |
| [SCHOLZ] (Raymer Table 6.4, full-scale) | GA single engine | 0.70 |
| [SCHOLZ] (Raymer Table 6.4, full-scale) | sailplane | 0.50 |

Spread: the RC-specific sources overlap at **0.35–0.60**; sport/pattern practice runs hotter
(up to 0.8). **Recommended band for the app: V_H = 0.40–0.65, trainer default 0.50,
sport 0.55, aerobatic 0.45** (aerobats trade stability for response — bottom of band).

**Vertical tail volume** `V_V = (S_v × l_v) / (S × b)` [MIT].

| Source | Category | V_V |
|---|---|---|
| [MIT] | any well-behaved model | **0.02 – 0.05** |
| [SCHOLZ] (Raymer) | homebuilt / GA single | 0.04 |
| [SCHOLZ] (Raymer) | sailplane | 0.02 |
| [MA] | RC vs free-flight | RC needs a fin ~50% larger than FF practice (FF gas rule: fin ≈ 4% of wing area) |

**Recommended band for the app: V_V = 0.025–0.05, trainer default 0.035–0.04.**
(NOTE: this is the genuine tail-aft band — unlike the flying-wing path, where reported V_V is
legitimately far below it; see ARCHITECTURE.md invariants. Do not share code between the two checks.)

**Tail arm to MAC.** [ND]: horizontal stab LE sits **2–3 wing chords behind the wing TE**;
nose (prop plate) sits **1–1.5 chords ahead of the wing LE**. With CG near 0.28c, the CG→stab
quarter-chord arm comes out at **l_h ≈ 2.5–3.5 × MAC** (derived). Consistency check (derived):
stab area 15–20% of wing [ND] × arm 2.5–3.5 c ⇒ V_H = 0.38–0.70, matching the table above.
For V_V, the fin arm l_v ≈ l_h on a conventional tail, so S_v = V_V·S·b/l_v ⇒ with
V_V = 0.035, b/c = 6, l_v = 3c: **fin ≈ 7% of wing area** (derived; [ND]'s "fin = 33% of stab
area" with stab = 15–20% wing gives 5.0–6.6% — consistent).

## 3. Static margin (tailed band)

- [MIT], eq. (2): ideal **SM = +0.05 … +0.15** of MAC; NP is the hard aft CG limit;
  SM ≈ +0.4 shown as annoyingly over-stable (large trim changes with speed).
- [ND]: trainer CG at **25–33% of chord**; with a trainer's NP around 40–50% MAC (V_H ≈ 0.5
  via [MIT] eq. 5) that is an effective SM of roughly **0.10–0.20** (derived) — trainer
  practice deliberately sits at/above the top of the ideal band.
- FT Simple Cub ships at CG 22% chord ([FliteTest](https://flitetest.co.za/products/ft-simple-cub-mkr2)) — same nose-heavy trainer bias.

**Recommended band for the app: 0.05–0.15 (hard limits), mission targets: trainer 0.12,
sport 0.10, aerobatic 0.06.** Contrast with the flying-wing band already in the app
(0.03–0.15, VALIDATION.md): tailed models both tolerate and want roughly 2× the margin at
the trainer end because the tail's damping and elevator authority make a nose-heavy model
docile rather than unflyable; a wing at 15% margin is already mushing.

## 4. Wing proportions

**Aspect ratio** (span = 5–6 × chord for a trainer per [ND] ⇒ AR 5–6):

| Mission | AR band | Anchors |
|---|---|---|
| trainer | **5.5 – 7** | [ND] 5–6; Apprentice 6.8, Eclipson A/XL 6.6, J-3 Cub 6.3 (§1) |
| sport | 5 – 6.5 | [ND] band + FT Simple Cub 4.7 marks the stubby floor |
| aerobatic | 4.5 – 6 | lower AR for roll rate; symmetric section ([ND]: "symmetrical airfoils are intended for aerobatic models") |

**Taper:** trainers are constant-chord or nearly so (FT Simple Cub, AeroScout, Cub scale
outers); use **taper ratio 0.7–1.0 trainer, 0.5–0.8 sport/aerobatic**. (Spread note: no RC
source pins taper numerically; these bounds come from the §1 airframes' planforms and keep
tip Re healthy at 600 mm span.)

**Dihedral** [ND] + [FMS Hobby wing-design guide](https://www.fmshobby.com/blogs/news/rc-wing-design-guide):

- High-wing, rudder/elevator (no ailerons): **3–6°**
- High-wing with ailerons (trainer): **up to 3°** — use 2–3°
- Low-wing sport/aerobatic: ~1–3°; forum practice suggests "high wing 1°, mid 2°, low 3°"
  as a with-ailerons starting point ([RCU thread](https://www.rcuniverse.com/forum/scratch-building-aircraft-design-3d-cad-174/1769864-how-much-dihedral.html)) — spread recorded, recommend **2° low-wing sport**.
- Spiral-stability check available: B = (l_v/b)·(Υ°/C_L) > 5 stable, VvB = 0.10–0.20 for
  rudder-roll authority [MIT] — worth implementing in `stability_conv.py` since it uses
  numbers the app already has.

**Incidence / decalage:**

- [ND]: wing and stab incidence "may preliminarily be set at zero", 2–3° down/right thrust.
- RC practice: **+2° decalage** (wing incidence minus stab incidence) "is considered good"
  ([RCU aerodynamics forum](https://www.rcuniverse.com/forum/aerodynamics-76/14486-wing-stab-incidence.html); definition: [Wikipedia "Decalage"](https://en.wikipedia.org/wiki/Decalage)).
- Spread: 0° (ND, trimmed by test flight) to +2–3° (forum consensus for trainers with
  cambered sections). **Recommend: wing +1.5 to +2°, stab 0 to −0.5°, i.e. decalage
  +1.5 to +2.5° trainer; ~0.5–1° sport; 0° aerobatic (symmetric section).** The app trims
  analytically anyway — pick decalage so cruise elevator deflection ≈ 0.
- Washout 3–5° advisable for stall behavior on trainers [ND] (less on aerobats).

## 5. Control surface sizing and throws

**Ailerons** ([Lennon, *Basics of R/C Model Aircraft Design*](https://books.google.com/books/about/Basics_of_R_C_Model_Aircraft_Design.html?id=3V7tmlTYbR4C) as cited in search results; [RCPlaneDesigner](https://rcplanedesigner.com/wing/ailerons/); [ND]):

| Style | chord fraction | span coverage | Source |
|---|---|---|---|
| outboard (recommended) | 25% of wing chord | outer 20–25% of each semispan ([ND]: aileron length = 1/4 wingspan i.e. ~50% of semispan; Lennon: 35–40% of semispan) | [ND], Lennon |
| strip aileron | 1/8 (12.5%) of chord | ~80% of semispan | [ND], [RCPlaneDesigner] |
| minimum authority | total aileron area ≥ 5% of wing area | — | [RCPlaneDesigner](https://rcplanedesigner.com/wing/ailerons/) |

Spread on span coverage is real (25–50% of semispan); **recommend outer_frac 0.95,
inner_frac 0.55–0.60, chord_frac 0.25**, which lands mid-spread and clears the tip.

**Elevator:** area = **20–30% of stab area** [ND]; stab AR ≈ 3 [ND]. Full-span constant-chord
elevator ⇒ elevator_chord_frac ≈ 0.25–0.30 of stab chord.
**Rudder:** fin area = **33% of stab area**, rudder = **1/3 to 1/2 of the fin** [ND]
(i.e. rudder_chord_frac 0.35–0.50); [MA] free-flight practice + "RC needs ~50% more fin".

**Throws** (measured at TE unless noted):

| Surface | [ND] (trainer, mm at TE) | FT Simple Cub (deg) | Recommended app default |
|---|---|---|---|
| elevator | ±6 mm | 12° with 30% expo | ±12° low / ±18° high |
| ailerons | ±6 mm; differential 8 up / 4 down with flat-bottom wings | 12° with 30% expo | ±12° low, 2:1 differential option |
| rudder | ±10 mm | 12° | ±20° (rudder runs bigger throws; [ND]'s 10 mm on a small fin chord ≈ 15–20°) |

Sources: [ND] p.6; [FliteTest Simple Cub](https://flitetest.co.za/products/ft-simple-cub-mkr2).
Spread: FT states one 12° figure for all surfaces; classic practice gives rudder more.

**Horn / servo implications.** A 9 g servo (TowerPro SG90: 1.8 kg·cm @ 4.8 V, the pocket
already measured in `backend/cad/servos.py`; [ProtoSupplies SG90](https://protosupplies.com/product/servo-motor-micro-sg90/))
drives these surfaces with margin at trainer speeds — FT specifies exactly "(4) 9 gram
servos" for the Simple Cub and Eclipson specifies 4–6 servos on the Model A/XL (§1 sources).
Keep the v1 horn doctrine unchanged (one 2.5 mm hole, world-aligned arm/horn planes,
ratio ~1 four-bar); the only new requirement is *reach*: elevator and rudder servos sit at
the wing-root bay with long runs, so pushrod/snake length, not torque, is the constraint
(V2_PLAN.md already routes them through Ø8.25 mm pipes).

## 6. Fuselage proportions and the electronics bay

**Length:** fuselage ≈ **75% of wingspan** [ND]. Real airframes (§1, derived): 0.63–0.79,
so band **0.63–0.80 × span**, trainer default 0.72–0.75.

**Cross-section:** fuselage height ≈ **10–15% of its length** [ND] (e.g. 1000 mm span →
750 mm fuselage → 75–113 mm deep). Width is set by the equipment bay, below.

**Bay contents envelope** (battery + RX + ESC; the app carries no propulsion electronics
sizing, only the bay):

- 3S 1500 mAh: 68 × 34.5 × 27.5 mm ([Zeee](https://zeeebattery.com/products/zeee-3s-lipo-battery-1500mah-11-1v-120c-xt60)); 102 × 34 × 20 mm ([Turnigy](https://hobbyking.com/turnigy-1500mah-3s-20c-lipo-pack.html)) — same capacity, two form factors: plan for both.
- 3S 2200 mAh: 100–115 × 34–35 × 23–28 mm ([Voltz](https://www.cmldistribution.co.uk/product/VZ0422003S/voltz-2200mah-3s-11-1v-30c-lipo-battery), [Gens Ace](https://genstattu.com/gens-ace-g-tech-2200mah-3s-35c-11-1v-hardcase-lipo-battery-pack-with-iec2-plug/))
- v1 flying-wing bays for the same mission run **136–310 mm long** (VALIDATION.md,
  "Equipment bay" row), with `hatch.py` floors of 25 mm length / 16 mm width / 10 mm depth
  (`_MIN_BAY_*`, backend/cad/hatch.py:165-167).

**Recommended conventional bay: ≥ 130 × 60 × 45 mm (L×W×D), preferred 160–200 × 70–90 ×
50–60 mm** — battery + RX + ESC side by side with finger room, matching the 60–100 mm bay
widths v1 produces on the deep-body planforms. That sets minimum fuselage internal width
~60–90 mm + 2 walls at the wing station.

**Nose length vs tail arm (CG closure).** Motor at x = 0 (tractor). [ND]: nose = 1–1.5 ×
chord ahead of the wing LE; tail arm 2–3 × chord behind the TE. With the app's lumped power
system at ~28% of AUW (ARCHITECTURE.md departure list, kept for v2) concentrated at the nose,
moment balance about a 0.28c CG works out (derived): the short arm of the heavy power mass
against the long arm of the light tail (tail group ≈ 5–8% AUW, §7) is what lets the battery
float inside the bay for trim. Code rule: place the bay so the **battery's allowed travel
(±30–40 mm) sweeps CG ±3–4% MAC** — that is the builder's trim tool, exactly how RTF
trainers are balanced ([FliteTest CG spec](https://flitetest.co.za/products/ft-simple-cub-mkr2) is given as a point + "move battery to balance").

## 7. Weight model deltas vs the flying wing

Material facts: LW-PLA foams to ~**0.54 g/cm³** effective vs **1.2–1.24 g/cm³** PLA
([ColorFabb](https://colorfabb.com/blog/post/lightweight-3d-printing-filaments-for-rc-planes), [SainSmart](https://www.sainsmart.com/products/lightweight-pla)) — the app's existing
material table already encodes this; nothing changes for the wing panels.

New mass items for a conventional (all vs. a same-span flying wing):

- **Fuselage skin:** a 750 × ~80 × ~90 mm single-wall LW-PLA shell. Anchor: 3DLabPrint J-3
  Cub total print mass ≈ 300 g LW-PLA for the *whole* 1068 mm airframe ([3DLabPrint](https://3dlabprint.com/shop/piperj3cub/)); Eclipson Model A: 220 g LW-PLA total print
  ([Eclipson](https://www.eclipson-airplanes.com/modela)). Fuselage is roughly 35–45% of print mass on these
  designs (derived from part lists) ⇒ budget **80–140 g LW-PLA (× ~1.9 for PLA)** at
  1000–1100 mm span, scaling ~ (span)³ ⁄ weakly.
- **Tail group:** stab 15–20% + fin ~5–7% of wing area (§2) at skin-panel areal density ⇒
  ≈ **20–25% of the wing structure's mass ratio**, typically 30–60 g at 1000 mm. Plus 2
  extra 9 g servos + pushrods (~30 g). Tail group ≈ 5–8% AUW (derived).
- **Wing saves nothing:** same spar/skin logic as v1.

AUW bands per span class for 3D-printed conventionals (published take-off weights):

| Span class | LW-PLA AUW | PLA AUW | Anchors |
|---|---|---|---|
| ~1000 mm | **490–600 g** | 730–900 g | Eclipson Model A 490/730 g; J-3 Cub ~600 g w/ electronics (§1) |
| ~1400 mm | **900–1100 g** | 1400 g | Eclipson Model A XL 900/1400 g (§1) |
| ~1500 mm foam RTF for comparison | — | 1390 g (Apprentice, foam+plastic) | §1 |

Sanity cross-check from the wing-panel field test: instrumented 3-panel wing 600 g PLA vs
372 g LW-PLA vs 332 g hot-wire foam ([Fabbaloo field test](https://www.fabbaloo.com/news/3d-print-materials-count-a-field-test-of-lightweight-pla)) — LW-PLA lands within ~12% of foam,
so the foam RTF AUWs above are legitimate targets for the optimizer, not just floors.
Wing-loading targets: **30–45 g/dm²** (Eclipson publishes exactly this LW-PLA→PLA range, §1);
[ND] cautions ~60 g/dm² is the *upper* norm at 1500 mm and "definitely lower" for smaller
models. v1 flying wings fly 34–39 g/dm² (VALIDATION.md) — keep the same neighborhood.

## 8. What makes a conventional trainer "look right"

The [ND] proportion diagram *is* the recognizable trainer; encode it as soft targets the
optimizer is penalized for leaving:

- **Span = 5–6 × chord** — a fatter chord reads as toy, a thinner one as glider [ND].
- **Fuselage ≈ 3/4 of the span, height 10–15% of its length** [ND] — the slab-sided box with
  a rounded top deck that every Cub/Apprentice silhouette shares (§1 lengths: 0.63–0.79).
- **Nose 1–1.5 chords, stab 2–3 chords behind the TE** [ND] — a nose shorter than 1 chord
  with a long tail reads as free-flight; longer than 1.5 reads as pattern ship.
- **High wing sitting ON the fuselage with 2–3° dihedral and visible tip lift** (§4) —
  trainers wear their dihedral; a flat wing on a high-winger looks wrong even when V_V and
  spiral B close.
- **Stab visibly ~1/6–1/5 of the wing in area, fin about a third of the stab** [ND] — an
  undersized fin is the single most common "something's off" cue on homebuilt models.
- **Tractor spinner/cowl at x = 0, main gear at/just aft of CG (trike) or axle at wing LE
  (taildragger)** [ND] — gear placement is a stated ND rule, and the app draws motors.
- **CG marker at 25–33% chord** [ND] — where a builder's finger goes under each wing panel.

Same doctrine as v1: these are geometry checks, not vibes — each maps to a number above and
belongs in `tests/test_conventional.py` alongside the physics gates.

---

### Coefficient quick-reference (for `config_defs.py`)

| Quantity | Band (hard) | Trainer default | Sport | Aerobatic |
|---|---|---|---|---|
| V_H | 0.40–0.65 | 0.50 | 0.55 | 0.45 |
| V_V | 0.025–0.05 | 0.038 | 0.035 | 0.030 |
| Static margin | 0.05–0.15 | 0.12 | 0.10 | 0.06 |
| AR | 4.5–7 | 6.0 | 5.5 | 5.0 |
| Taper ratio | 0.5–1.0 | 0.85 | 0.70 | 0.60 |
| Dihedral [°] | 1–6 | 2.5 (high wing + ailerons) | 2.0 | 1.0 |
| Decalage [°] | 0–2.5 | +2.0 | +0.75 | 0 |
| l_h / MAC | 2.5–3.5 | 3.0 | 3.0 | 2.7 |
| S_h / S | 0.15–0.20 | 0.18 | 0.17 | 0.16 |
| S_v / S_h | ~0.33 (fin) | 0.33 | 0.33 | 0.35 |
| Elevator / S_h | 0.20–0.30 | 0.25 | 0.27 | 0.30 |
| Rudder / S_v | 0.35–0.50 | 0.40 | 0.45 | 0.50 |
| Aileron chord frac | 0.20–0.25 | 0.25 | 0.25 | 0.25 |
| Aileron span (semispan frac) | 0.25–0.50 outboard | 0.35 | 0.40 | 0.45 |
| Fuselage length / span | 0.63–0.80 | 0.73 | 0.72 | 0.70 |
| Wing loading [g/dm²] | 25–60 | 30–40 | 35–50 | 35–50 |

Sources for each row are in §2–§6 above; defaults sit inside every cited spread.
