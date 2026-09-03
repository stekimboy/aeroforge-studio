# RESEARCH_TYPES_V3.md — design-rules dossier for the six new airplane types + four configuration axes

Scope: the v3 type list beyond `flying_wing`/`conventional` — **delta, canard, tandem,
biplane, twin_boom, glider** — plus the four config axes (**tractor/pusher, motor count,
tail type, wing position**), hobby scale (600–1800 mm span), 3D-printed PLA/LW-PLA, 9 g
servos. Same conventions as `RESEARCH_CONVENTIONAL.md`: every number carries its source,
disagreements are recorded as spreads, and **"Derived"** marks a number computed here from
cited inputs — usable, but not itself a published figure. The final quick-reference table is
the transcription target for `physics/config_defs.py` and the per-type modules.

Primary references used throughout (in addition to those already in RESEARCH_CONVENTIONAL.md):

- **[MIT]** Drela, "Basic Aircraft Design Rules", MIT 16.01/16.02 Lab 11 notes —
  https://web.mit.edu/16.unified/www/SPRING/systems/Lab_Notes/desrules.pdf
- **[ND]** "Trainer Design", Univ. of Notre Dame AME 40462 handout —
  https://sites.nd.edu/ame40462/files/2018/03/TrainerScaling.pdf
- **[SCHOLZ]** Scholz et al., INCAS Bulletin 13(3) 2021 (reproduces Raymer Table 6.4) —
  https://www.fzt.haw-hamburg.de/pers/Scholz/Aero/AERO_PUB_INCAS_TailVolume_Vol13No3_2021.pdf
- **[RCG]** "Tail Design and Sizing" (RCGroups design-guide attachment) —
  https://www.rcgroups.com/forums/showatt.php?attachmentid=4869670
- **[LEN-CAN]** "Canard Design Notes" (19 pp., based on Lennon, *R/C Model Aircraft Design*,
  ch. 22 — equations credited to Lennon in the document itself) —
  https://rcaeronotes.wordpress.com/wp-content/uploads/2018/02/canard-design-notes-1-20-2017.pdf
- **[AEROM]** "Biplane Gap & Stagger", Aeromodeller March 1951, abstracted by M. Nassise —
  http://www.flyingacesclub.com/PFFT/BipeGapStagger.pdf
- **[KAV]** "Biplanes configuration", RC Kavala Acro Team (giant-scale biplane design article) —
  https://rckavalaacroteam.com/biplanes-configuration/
- **[MA-T]** "Tails for Models", Model Aviation library (V-tail math + FF/RC fin practice) —
  https://library.modelaviation.com/article/tails-models
- **[NACA823]** Purser & Campbell, NACA Report 823, "Experimental verification of a
  simplified vee-tail theory…" — https://www.abbottaerospace.com/downloads/naca-report-823-experimental-verification-of-a-simplified-vee-tail-theory-and-analysis-of-available-data-on-complete-models-with-vee-tails/
  (RC summary: "V-Tails for Aeromodels", FMSG Alling — https://www.fmsg-alling.de/wp-content/uploads/2013/09/V-Leitwerke.pdf)
- **[RCU-DELTA]** "Center of gravity for delta wing", RCUniverse aerodynamics forum
  (long-standing thread with builder-measured CGs) —
  https://www.rcuniverse.com/forum/aerodynamics-76/7571698-center-gravity-delta-wing.html
- **[AWEB]** Aerospaceweb, "Wing Twist and Dihedral" —
  https://aerospaceweb.org/question/dynamics/q0055.shtml

---

## 1. Delta (tailless, elevons)

Routes through the tailless machinery as a new planform family (V3_PLAN.md); the flying-wing
hardware doctrine and three-axis handling checks apply unchanged.

### 1.1 Reference airframes

| Airframe | Span [mm] | Length [mm] | AUW [g] | Motor | Source |
|---|---|---|---|---|---|
| ParkZone F-27C Stryker (delta foamie) | 940 | 690 | 620 | pusher, 480-class outrunner | AMain Hobbies product page PKZ4200 (listing since removed; no live link) |
| ParkZone F-27Q Stryker | 943 | — | 855 | pusher brushless | [Hearns Hobbies](https://www.hearnshobbies.com/blogs/old-hearns/71259717-parkzone-f-27q-stryker) |
| FT Mini Arrow (foam-board arrow/delta) | 775 (30.5 in) | — | 218 dry / 245 AUW | pusher, CG 44 mm (1.75 in) per plans | [FT plans PDF](https://s3.amazonaws.com/plans.flitetest.com/stonekap/FT%20Mini%20Arrow%20v1.1%20Tiled.pdf) |
| DW Hobby Zagi-style delta E0601 | 1000 | 365 | 320 flying | pusher, 2 × 9 g servos | [Amazon listing](https://www.amazon.com/Hobby-Electric-Wingspan-Tail-Pusher-Aeroplane/dp/B07CB82CKN) |

All four are **pushers with two 9 g elevon servos** — the type's natural config default (§7).

### 1.2 Bands

- **LE sweep 45–60°, default 50°.** "For any 'usual' delta of around 60 degrees L/E sweep…"
  [RCU-DELTA]; FT arrows/deltas and the Stryker sit visibly nearer 45°. Below ~45° the shape
  reads as a swept wing (v1 `swept` family), above 60° tip Re and low-speed handling die at
  this size.
- **AR 1.8–3.5, default 2.5.** Derived: a pure triangular delta has `AR = 4/tan(Λ_LE)`
  (from `S = b·c_r/2`, `c_r = (b/2)·tan Λ`), i.e. Λ = 45° → AR 4.0, Λ = 60° → AR 2.3;
  RC deltas are cropped (finite tip chord), which pushes AR below the pure-triangle value.
  The band brackets 45–60° sweep with realistic tip crops (taper ratio 0.10–0.30).
- **CG 15–22% MAC** (≡ static margin at the middle of the tailless band). Builder-measured
  practice [RCU-DELTA]: "Most delta's I researched … balanced between 15%–20% MAC" (flew
  best at 18%); an F-106 model at "approx 22% MAC"; 20–25% quoted for tailless deltas; the
  60°-sweep shortcut "balance at about 50% of the centre chord" is the same point expressed
  on the root chord. **App rule: keep the existing tailless SM band 0.03–0.15 hard, target
  0.06–0.12** — the app computes NP itself, so the %-MAC figures are validation anchors, not
  the mechanism.
- **Section / reflex.** RC deltas at AR ≤ 3 fly on flat-plate or symmetric sections trimmed
  by up-elevon (FT foam-board deltas are flat plates; the Stryker family is near-symmetric
  foam). At these ARs the elevon is a large fraction of total area, so trim authority is
  cheap; classic flap data: a 0.2 c plain flap at 10° gives ΔCL ≈ 0.15 and shifts the
  zero-lift angle 2° [LEN-CAN §3, Table 3-2] — inverted, a few degrees of reflex buys the
  positive Cm0 a tailless design needs. **App rule: section Cm0 ≥ 0 (symmetric allowed,
  reflex preferred), and cruise trim must use ≤ 25% of elevon throw** (consistent with the
  existing 75%-of-travel handling gate).
- **Vertical fin 4–9% of wing area, default center fin ≈ 6%.** Free-flight practice: fin ≈ 4%
  of wing area (gas), and "RC needs a fin ~50% larger" [MA-T → ≈ 6%]; the v1 flying-wing fin
  band (2–9% S, `stability.flying_wing_fin`) already brackets this. Deltas default to a
  center fin (V3_PLAN.md); the Stryker's twin-fin layout maps to the existing `twin_fin`
  option. Bell-style zero-fin layouts do NOT apply to deltas.
- **Elevon sizing: chord 20–25% of local chord, spanning the trailing edge outboard of the
  motor/prop cutout.** Plain-flap optimum ≈ 0.2 c — "a plain flap to cord ratio of 0.2 C is
  near optimum. Smaller … lower lift … larger ones don't improve lift but increase drag"
  [LEN-CAN §3]. Keep the v1 elevon/hinge/horn doctrine bit-for-bit.
- **Wing loading 20–45 g/dm², target 25–35.** Spread note: delta areas are rarely published;
  derived anchors — DW Hobby delta ≈ 320 g on ≈ 20 dm² (≈ 16 g/dm², floaty EPP end), v1
  flying wings fly 34–39 g/dm² (VALIDATION.md), Stryker-class deltas are visibly hotter than
  both. Deltas tolerate the high end via vortex lift but pay in sink rate; land the default
  in the middle.

## 2. Canard (foreplane + aft main wing)

`geometry.canard` block per V3_PLAN.md. The defining safety property: **the canard stalls
first, always** — it is a certification-style invariant, not a preference [LEN-CAN §1].

### 2.1 Reference airframes / designs

| Design | Span (wing/canard) | Areas (wing/canard) | Ratio Sc/S | Weight | Source |
|---|---|---|---|---|---|
| Rutan Long-EZ (full-scale anchor) | 26.1 ft / 11.8 ft | 81.99 ft² / 12.8 ft² | 0.156 | 1600 lb gross | [Wikipedia Long-EZ](https://en.wikipedia.org/wiki/Rutan_Long-EZ) |
| Rutan VariEze (full-scale anchor) | 22 ft 2.5 in | 53.6 ft² wing | — | canard airfoil GU25-5(11)8 | [Wikipedia VariEze](https://en.wikipedia.org/wiki/Rutan_VariEze), [VT canards review](https://archive.aoe.vt.edu/mason/Mason_f/canardsS03.pdf) |
| Lennon worked example 1 (RC) | 897 mm rear / 490 mm front | 249 / 62.2 in² | 0.25 | 28 oz (794 g) | [LEN-CAN Table 4-3] |
| Lennon worked example 2 (RC) | 691 mm rear / 516 mm front | 164 / 82 in² | 0.50 | 18 oz (510 g) | [LEN-CAN Table 4-4] |

Derived from the table: Long-EZ canard AR = 3.6²/1.19 m² ≈ **10.9** (high-AR foreplane);
Lennon's RC examples use canard AR 6 vs wing AR 5 and canard AR ~5 vs wing ~4.5 — RC canards
run lower AR than full-scale but still **at or above the wing's AR**.

### 2.2 Bands

- **Canard/wing area ratio Sc/S = 0.20–0.35, default 0.25.** Spread: 0.156 (Long-EZ) to 0.50
  (Lennon ex. 2, verging on tandem); Lennon's primary example uses 0.25. Below ~0.15 the
  canard's trim authority shrinks and elevator deflections grow; above ~0.4 the design is a
  tandem and should be handled by §3.
- **Lift-coefficient ratio CLf/CLr = 1.4–1.6, hard floor > 1.0.** "This ratio must be greater
  than 1 to satisfy stability requirements and is typically on the order of 1.4 to 1.6"
  [LEN-CAN Eq. 2-7]. This IS the stall-first margin in coefficient form: the canard flies
  40–60% harder than the wing at every trimmed AoA. **Record it in the design dict as the
  loading margin** (V3_PLAN.md requires it recorded).
- **Static margin: CG ahead of NP by k = 0.20–0.25 × MAC_rear.** "k is a static margin
  fraction, typically in the range of 0.2 to 0.25" [LEN-CAN §2, Eq. 2-2]. Note this is
  larger than the tailed band (0.05–0.15) because it is referenced to the rear wing's MAC
  alone while the NP of the two-surface system sits far forward of the rear wing's AC.
- **Rear-wing downwash efficiency 0.8** for the portion of the wing directly behind the
  canard span: "Assume eff = 0.8 for the portion of the rear wing directly behind the front
  wing span" [LEN-CAN Eq. 2-1]. NP relation: `N = Af·L/(Af + eff·Ar)` measured forward of
  the rear AC.
- **Canard volume coefficient V_C = Sc·l_c/(S·MAC): 0.55–0.85 (derived).** From the two
  Lennon examples: ex. 1 — Sc 62.2 in², canard arm L−D = 22−6.29 = 15.71 in, wing 249 in²,
  MAC 7.10 in → V_C = 0.55; ex. 2 — 82 in², 18−7.71 = 10.29 in, 164 in², 6.08 in → V_C =
  0.85. Lifting canards run an order of magnitude above tail-style "control canard"
  coefficients — do not import tailed V_H logic.
- **Canard AR 5–8, and ≥ wing AR** (Lennon uses 6 vs 5; higher AR → steeper lift slope →
  the canard reaches its stall AoA first, reinforcing criterion 1) [LEN-CAN §4]. Wing AR
  4.5–6 per the conventional bands.
- **Elevator-on-canard: plain flap, chord 0.2 × canard chord; useful deflection tops out
  ≈ 40°** ("the optimum deflection is about 40 degrees. Any greater deflection causes the
  front wing to stall at a lower AoA") [LEN-CAN §3–4]. App throws: ±20° default, ±30° max —
  well inside the 40° authority knee.
- **Incidence/decalage: canard +1 to +3° relative to the rear wing; rear-wing chord aligned
  with the thrust axis** ("By aligning the rear wing chord line with the thrust axis the
  lift force caused by the motor is removed") [LEN-CAN §4 pp. 12–14, §6 item 5]. The app
  trims analytically — pick incidence so cruise elevator ≈ 0 with CLf/CLr in band.
- **Vertical surfaces: aft lateral-area moment ≥ 1.25 × forward moment; CLA arm ratio
  LR1/LR2 ≈ 0.25; pusher canards use TWIN fins at/near the wingtips, clear of the prop
  circle** ("the aft side area moment behind the CG needs to about 25% greater than that of
  the front area moment"; "mount them at the wing tips where they can be extended rearward
  without affecting the ailerons") [LEN-CAN §1 item 4, §5]. Canards "generally have a small
  moment arm to VT, requiring larger area" [VT canards, slide 30] — expect fin area at the
  top of the flying-wing-style 2–9% S band.
- **Pusher default** ("Most, but not all, canard aircraft employ pusher arrangements. This is
  because the CG tends to be toward the rear, near the rear wing leading edge") [LEN-CAN §5].
  Canard airfoil note for guidance text: the Long-EZ's laminar GU25 canard lost lift in rain
  (contamination-triggered separation) and was replaced by the Roncz 1145 [VT canards,
  slides 27–28] — turbulent-friendly sections on the foreplane, never the most laminar polar.

## 3. Tandem wing (two comparable wings)

`geometry.wing2` per V3_PLAN.md. Lennon's framework covers it explicitly: "The design
principles in this document apply to any two-surface design … Tractor designs … increase
[canard] area and maybe decrease the rear wing area. The result is what is usually called a
tandem-wing design" [LEN-CAN §6 item 7] — i.e. tandem = the canard math with Sc/S → 0.4–1.0.

### 3.1 Reference airframes

| Airframe | Front/rear span | Notes | Source |
|---|---|---|---|
| QAC Quickie Q2 (full-scale anchor) | canard 15 ft / wing 16 ft 8 in | "full-span elevators are on the canard and the inboard ailerons are on the rear wing"; main gear on canard tips | [Wikipedia QAC Quickie Q2](https://en.wikipedia.org/wiki/QAC_Quickie_Q2) |
| Rutan Quickie (single-seat original) | comparable-span tandem | tandem layout definition | [Wikipedia Rutan Quickie](https://en.wikipedia.org/wiki/Rutan_Quickie) |
| Lennon example 2 re-read as tandem | 516 mm front / 691 mm rear | Sf/Sr = 0.50, CLf/CLr = 1.5 | [LEN-CAN Table 4-4] |

Academic anchors: tandem lift-share and spacing studies — Cheng et al. 2018 (lifting-line
prediction for tandem/multi-surface systems, notes each wing carries **40–60% of total
lift/area** in the tandem regime) — https://onlinelibrary.wiley.com/doi/10.1155/2018/3104902 ;
wing-spacing experiments (Bath) showing non-zero gap delays rear-wing stall and that optimal
gaps exist — https://purehost.bath.ac.uk/ws/files/119292318/Experiments_in_Fluids_Biplane_and_Tandem.pdf ;
"bigger … gap and stagger can produce better lift" (tandem UAV, morphing study) —
https://journals.sagepub.com/doi/full/10.1177/1687814017692290 .

### 3.2 Bands

- **Lift share: front wing 45–60% of total, default 0.50–0.55.** Each wing 40–60% of the
  combined lifting area/lift (Cheng 2018, above). The front-heavy bias is the same
  stall-first rule as §2: front CL > rear CL at all trimmed AoA (CLf/CLr ≥ 1.2 for tandems —
  Lennon's 1.4–1.6 applies at canard-like area ratios; as Sf/Sr → 1 the achievable ratio
  shrinks, so hold **≥ 1.2, target 1.3** [derived from LEN-CAN Eq. 2-7 with D/L at the
  recorded CG]).
- **Stagger (AC-to-AC distance) L = 2.5–3.5 × rear MAC, default 3.0.** Derived: Lennon ex. 1
  L/MAC_r = 22/7.10 = 3.1; ex. 2 = 18/6.08 = 3.0. Shorter couples the wings too tightly
  (rear wing deep in downwash), longer wastes fuselage.
- **Gap 0.3–1.0 × MAC, rear wing HIGH.** Non-zero vertical gap lifts the rear wing out of
  the front wake — "lift performance improves and stall is delayed significantly … with
  non-zero gap," with an optimum gap (Bath, above); the Quickie mounts the canard low and
  the main wing high for exactly this reason [Wikipedia QAC Quickie Q2]. Default 0.5 × MAC.
- **Decalage: front wing +0.5 to +2° relative to rear, default +1°.** The front-loads-first
  requirement expressed as rigging; equivalently set incidences so CLf/CLr lands in band at
  cruise [LEN-CAN §4 incidence discussion]. Keep the rear wing chord on the thrust axis
  (same rule as §2).
- **Per-wing AR 5–7** (Lennon's 6 front / 5 rear; both wings within the conventional band so
  the existing wing-panel CAD generalizes).
- **Rear-wing efficiency 0.8** behind the front span [LEN-CAN Eq. 2-1] — with comparable
  spans that is most of the rear wing; the physics module must apply it span-resolved.
- **Control allocation: full-span elevator on the FRONT wing, ailerons on the REAR wing**
  (Quickie Q2 practice, above). Elevator chord 0.2 c (plain-flap optimum [LEN-CAN §3]);
  rear ailerons per the conventional aileron bands (chord 25%, outboard). Elevons-on-both
  is a legal FT-style variant but the default is the Quickie split — it keeps pitch and roll
  servos in different wings, which the wiring doctrine likes.
- **Vertical surfaces:** same CLA rules as canard §2 (aft moment ≥ 1.25 × forward, LR1/LR2 ≈
  0.25) [LEN-CAN §5]; single aft fin works with a tractor motor; twin tip fins for pusher.

## 4. Biplane (two stacked wings + conventional tail)

`geometry.wing2 {gap_m, stagger_m, decalage_deg}` + strut stations per V3_PLAN.md.

### 4.1 Reference airframes

| Airframe | Span [mm] | Wing area [dm²] | AUW [g] | Wing loading [g/dm²] | Source |
|---|---|---|---|---|---|
| E-flite Pitts S-1S 850 mm (sport bipe) | 850 | 28.2 (437 in², both wings) | 1304 | 46 (derived) | [HobbyTown spec](https://www.hobbytown.com/eflite-pitts-s1s-pnp-electric-airplane-850mm-efl3575/p800192) |
| FT Baby Blender (foam-board bipe) | 610 | n/p | 397 (14 oz) w/o battery | — | [HobbyTown spec](https://www.hobbytown.com/flite-test-baby-blender-speed-build-electric-airplane-kit-610mm-flt-1003/p-xuhcsedqwuqbg4yz), [FT article](https://www.flitetest.com/articles/Baby_Blender_Biplane) |
| E-flite UMX Pitts S-1S (micro anchor) | 434 | n/p | 94 w/o battery | — | [HobbyTown spec](https://www.hobbytown.com/eflite-umx-pitts-s1s-bindnfly-electric-airplane-434mm-eflu15250/p1401529) |

Note the Pitts' 46 g/dm² on BOTH wings' area: bipes buy area, not lightness — a same-span
monoplane at that loading would carry half the area.

### 4.2 Bands

- **Gap/chord 0.8–1.25, default 1.0.** "For satisfactory flight efficiency a minimum figure
  is usually stated as gap = wing chord"; above ~1.5 c "each wing acts as a single monoplane
  wing with no inter-wing interference" [AEROM]. RCU practice thread: 1.25 × mean chord
  used; "some consider 0.7 as about the lowest gap should go"
  ([RCU "Biplane Wing Gap?"](https://www.rcuniverse.com/forum/aerodynamics-76/2019961-biplane-wing-gap.html)).
  Spread 0.7–1.5; the recommended band keeps prints stiff (short cabanes) without eating
  the ~10% lift interference penalty below 0.8.
- **Stagger 0 to +0.5 c (forward = upper wing ahead), default +0.25 c.** "Theoretically,
  forward stagger is best and the use of stagger enables the gap to be reduced for the same
  overall efficiency" [AEROM]; wind-tunnel practice: "positive stagger of 50% of the mean
  chord will increase maximum lift by about 5%"; "increasing stagger by 0.3 of chord will
  reduce induced drag by about the same degree as increasing gap by 0.1 chord" (stagger is
  worth ~⅓ of gap) [RCU gap thread, above]. Stagger also opens cockpit/bay access on the
  real airframes — and the hatch, for us.
- **Decalage 0 to +1°, default +0.5° (upper wing more incidence).** "Positive decalage is
  more usual with forward stagger. … the upper wing will reach its stalling angle before the
  lower one … the lower wing will act like a short-coupled tailplane"; but the effect "is
  small compared with tailplane power … not a worthwhile inclusion solely on this score"
  [AEROM]. Keep it small; the tail does the stabilizing.
- **Interference / effective lift (the Munk/Prandtl content, coded practically).** Theory:
  Prandtl's biplane theory (Munk's stagger theorem: total induced drag is independent of
  stagger for fixed lift split) says a biplane always has LESS induced drag than the
  same-span monoplane and the benefit grows with gap ([Purdue box-wing review, §Prandtl
  1924](https://docs.lib.purdue.edu/cgi/viewcontent.cgi?article=1253&context=jate)); but at
  model scale each wing's chord Re is what hurts: "a gap of greater than 1.5 times the chord
  can give almost 90% of the lift produced by a monoplane [of the same total area]" [RCU gap
  thread]; [AEROM] fig. 1 discussion makes the same Re point. **App rule (derived
  interpolation anchored on the cited 90% @ 1.5 c and "minimum gap 1.0 c" points): per-wing
  lift effectiveness factor ≈ 0.80 at g/c = 0.8, 0.85 at 1.0, 0.90 at 1.5** — apply to CL
  and to the induced-drag e-factor, and record it in the design dict (V3_PLAN's "Munk
  gap/interference factor in the aero").
- **Total-area sizing vs monoplane: +10–25% area for the same effective lift (derived** from
  the effectiveness factors above; sanity: the Pitts 850's 46 g/dm² would be a hot 55+ g/dm²
  monoplane at the same AUW and one wing's area).
- **Lower/upper area ratio ≥ 0.5** — below that "the layout is called a sesqui-plane"
  [AEROM]. Default: equal span/chord both wings, which is also the most efficient
  arrangement per [AEROM] ("most efficient wings are of equal area, span and chord, set at
  the same angle of incidence").
- **Per-wing AR 4–6.** Derived: Pitts 850 — each wing ≈ 14.1 dm², span 8.5 dm → AR ≈ 5.1;
  bipes accept lower AR because the two wings share the span's induced-drag burden.
- **Tail volumes:** "tail moment arm … three times the upper wing's mean aerodynamic chord";
  "horizontal tail area … 15 percent of the biplane's total wing area"; "elevator … 30
  percent of the horizontal tail"; "rudder area … no less than 50 percent of the total
  vertical tail area" [KAV]. Derived V_H = 0.15 × 3.0 = **0.45 on total wing area + mean
  MAC** — inside the conventional band, so `stability_conv`-style checks port directly if S
  means TOTAL biplane area. Fin per conventional V_V 0.025–0.05.
- **Dihedral 2–3°** [KAV], with [AEROM]'s note that bipes need "slightly less dihedral" than
  monoplanes — use 2° upper the default, 0–2° lower.
- **Struts/cabane:** interplane strut pair at ~60% semi-span + cabane struts at the fuselage
  sides (the Pitts/Baby Blender pattern, §4.1 sources). For print: struts are structural
  (they close the wing-box), so they are PARTS with sized cross-sections, not decoration;
  clearance doctrine applies at their wing sockets (0.25 mm/face, same source discipline as
  `servos.SERVO_CLEARANCE_MM`).
- **Wing loading 30–50 g/dm² on total area** (Pitts 46 at the sport end; FT foam bipes run
  well under; [KAV]'s 25 oz/ft² ≈ 76 g/dm² is giant-scale practice — recorded as the spread's
  far end, not for this size class).

## 5. Twin-boom (pusher pod + tail between booms)

`geometry.booms` per V3_PLAN.md; the type's raison d'être: "For a single engine with a
propeller in pusher configuration, a conventional tail requires the propeller to be moved far
aft … The twin-boom configuration allows a much shorter and more efficient installation"
([Wikipedia, Twin-boom aircraft](https://en.wikipedia.org/wiki/Twin-boom_aircraft)).

### 5.1 Reference airframes

| Airframe | Span [mm] | Length [mm] | Wing area [dm²] | Weight | Source |
|---|---|---|---|---|---|
| MyTwinDream (MFD) FPV twin-boom | 1800 | 1220 | n/p | 1100 g empty; MTOW quoted up to 4.5 kg+ | [FPV Model blog spec](https://fpvplane.blogspot.com/2015/07/mfd-mytwindream-1800mm-fpv-plane.html), [Banggood listing](https://www.banggood.com/My-Twin-Dream-MTD-1800mm-Wingspan-Twin-Motor-EPO-Aerial-Survey-FPV-Platform-Mapping-RC-Airplane-Kit-reviews-p1105054.html) |
| ZOHD/SonicModell Skyhunter 1800 | 1800 | 1400 | 36 | 3.0–3.5 kg max flying | [HobbyKing spec](https://hobbyking.com/en_us/skyhunter-fpv-uav-aircraft-platform-1800mm-kit.html); SonicModell product page (site unreachable, link dropped) |
| RMRC Skyhunter kit | 1800 | 1400 | 36 | CG "1/3 of wing from leading edge", props 9–12 in | [RMRC product page](https://www.readymaderc.com/products/details/rmrc-skyhunter-kit) |

Derived: Skyhunter mean chord = 36 dm²/18 dm = 200 mm, AR = 18²/36 = **9.0** — twin-boom FPV
wings run higher AR than sport trainers (efficiency mission).

### 5.2 Bands

- **Tail arm l_h = 2.5–3.5 × MAC, default 3.0** — the conventional band ([ND]: stab 2–3
  chords behind the TE; RESEARCH_CONVENTIONAL §2) applies verbatim; the booms exist to buy
  that arm without a rear fuselage. Derived check: Skyhunter overall length 1400 mm on a
  200 mm chord leaves ≈ 600–700 mm from a ~0.3 c CG to the stab quarter-chord ≈ 3.0–3.5 MAC.
- **Tail volumes: V_H 0.40–0.65 (default 0.50), V_V 0.025–0.05 (default 0.035), with the
  vertical area SPLIT half per boom-mounted fin** — same coefficients as conventional
  ([MIT], [SCHOLZ]; RESEARCH_CONVENTIONAL §2), fins at the boom ends where l_v ≈ l_h. The
  stab spanning boom-to-boom picks up end-plate benefit from the fins (inverted-U/H-tail),
  so the H-tail can sit at the BOTTOM of the V_H band; boom-mounted inverted-V is the other
  documented arrangement ([KDU inverted V-tail on booms paper](http://ir.kdu.ac.lk/bitstream/handle/345/3680/FOE%20Proceeding%20article%20228-234.pdf?sequence=1&isAllowed=y)).
  **App default: H-tail between booms** (Skyhunter/MTD practice); stab span = boom spacing
  by construction.
- **Boom spacing ≥ 1.15 × prop diameter (derived rule), and never < prop D + 2 × 15 mm tip
  clearance.** Anchors: the Skyhunter's recommended props are 9–12 in [RMRC], and a 13 × 8
  fits with "plenty of room" ([Innov8tive motor note, PDF](https://innov8tivedesigns.com/downloads/ZOHD-Skyhunter-1800.pdf));
  our class flies 8–10 in props, so boom spacing ≈ 260–330 mm. The 15 mm per-side floor is
  this project's clearance doctrine applied to a spinning tip (larger than any static
  clearance in the app — a flexing boom converges on the disk).
- **Pod: nose payload/battery bay + motor at the pod's aft face, pusher.** CG anchor: "1/3 of
  wing from leading edge" [RMRC]. Pod length ≈ 2 × MAC ahead of the wing TE (derived from
  the §5.1 airframes' proportions: pod ends at/near the wing TE where the prop spins between
  the booms). The bay doctrine (Ø8.25 pipes, trumpeted mouths, existence-checked cuts)
  carries over; motor leads now run INSIDE the pod, not through a belly hole.
- **Boom stiffness proxy (V3_PLAN verification item):** booms are the printed part most prone
  to flutter-by-flex. Rule (derived from printed-spar practice, §6): boom section second
  moment must not fall below a round carbon tube Ø8 × 1 mm wall equivalent per metre of
  boom; record `booms.section` with the chosen shape so the check is reproducible. (Foam
  airframes above use aluminum/carbon booms — [RMRC] kit ships tube booms.)
- **Wing loading 35–60 g/dm²** (derived: Skyhunter at 2.5–3.5 kg on 36 dm² = 69–97 g/dm² at
  MTOW — but that is a ballistic FPV hauler at 4S; at our 0.6–1.8 m/LW-PLA scale hold the
  conventional 30–50 band, top end allowed for the payload mission).

## 6. Glider / thermal sailplane

Conventional geometry with glider bands; `n_motors: 0` legal (V3_PLAN.md).

### 6.1 Reference airframes

| Airframe | Span [mm] | Wing area [dm²] | AR | AUW [g] | Wing loading [g/dm²] | Source |
|---|---|---|---|---|---|---|
| ParkZone Radian (foam e-glider) | 2000 | 35.5 (551 in²) | 11.3 (derived) | 980 | 27.6 (derived) | [Great Hobbies spec](https://www.greathobbies.com/productinfo/?prod_id=PKZ4775) |
| Carl Goldberg Gentle Lady (balsa 2 m) | 1988 | 42.8 (663 in²) | 9.2 (derived) | 623–709 | 14.6–16.6 (derived) | [MRS Hobby kit spec](https://mrshobby.com/goldberg-gentle-lady-glider-rc-model-kit/), [Outerzone plan](https://outerzone.co.uk/plan_details.asp?ID=5961) |
| Eclipson Apex (3D-printed 2.3 m) | 2300 | 36 | 14 (published) | 1300–1550 | 36–43 (published) | [Eclipson Apex](https://www.eclipson-airplanes.com/apex) |
| 3DLabPrint Swift S-1 (printed 1.38 m) | 1380 | n/p | — | ~524 (print 271 g) | — | [3DLabPrint Swift S-1](https://3dlabprint.com/shop/swift-s-1/) |

The spread between the Gentle Lady (15 g/dm²) and the printed Apex (36–43 g/dm²) is the
material talking: printed gliders cannot reach balsa floater loadings.

### 6.2 Bands

- **AR 9–15 for printed, hard band 8–18, default 11.** Anchors: Gentle Lady 9.2, Radian
  11.3, Apex 14 (§6.1). The 15–18 end belongs to composite moldies; a printed LW-PLA wing
  above AR ~15 at 2 m span exceeds any spar this app can print around (see spar note).
- **Wing loading: hard band 15–45 g/dm²; thermal target 18–30 (LW-PLA), 30–45 accepted for
  PLA.** Anchors: Gentle Lady 14.6–16.6 marks the balsa floor; Radian 27.6 is the foam
  standard; Eclipson publishes 36–43 for a printed glider (§6.1). The task's classic
  "thermal 15–35" band is real but its bottom half is unreachable in PLA — the optimizer
  must not chase it.
- **Tail volumes: V_H 0.35–0.55 (default 0.45), V_V 0.02–0.035 (default 0.025).** [RCG]
  gliders V_H 0.30–0.50; Raymer/[SCHOLZ] sailplane V_H 0.50, V_V 0.02. Long tail arms
  (l_h 3–4 × MAC) with SMALL surfaces is the sailplane signature — prefer stretching the arm
  over growing the stab (fuselage is cheap, tail mass is not: it multiplies nose ballast).
- **Static margin 0.05–0.15, default 0.10** (tailed band, RESEARCH_CONVENTIONAL §3 — nothing
  glider-specific changes it; floaters trim at the stable end).
- **Fuselage length 0.50–0.60 × span (derived: Radian 0.57, Gentle Lady 0.52, Apex 0.565)** —
  distinctly below the trainer's 0.63–0.80 band; the glider silhouette is a long wing on a
  slender pod-and-boom body. Height stays ≤ 10% of length (slender cross-section).
- **Spar/wing-bending (the printed-wing gate):** Eclipson stiffens the 2.3 m Apex with
  carbon tubes — "D10mm, D8mm, D4mm, and D8x8mm square" sections [Eclipson Apex spec];
  the EBW-160's whole printed airframe is 275 g at 1.6 m ([Eclipson EBW-160](https://www.eclipson-airplanes.com/ebw-160-rc)).
  **App rule: span > 1.4 m OR AR > 10 ⇒ the wing carries a round carbon-tube spar socket,
  Ø8–10 mm at the root, tapering allowed outboard; record it in the geometry block.** The
  LW-PLA skin carries torsion only — same doctrine as the v1 spar logic, one size up.
- **Polyhedral vs ailerons:** rudder/elevator thermal ships use polyhedral — "5 degrees from
  root to 3/5 of the semi-span, with an increase of 3 degrees from the polyhedral joint to
  the wingtip" ([Hooked-on-RC thermal gliding guide](https://www.hooked-on-rc-airplanes.com/thermal-gliding.html));
  the Radian and Gentle Lady are both polyhedral R/E designs (§6.1 sources). Aileron gliders
  (Apex: 6 servos, ailerons + flaps) fly flat-ish wings with 2–4° dihedral. **App rule:
  mission `thermal_floater` + no ailerons ⇒ polyhedral 5°/+3° at 60% semi-span; aileron
  variant ⇒ 3° straight dihedral** — and polyhedral is a geometry.wing feature the CAD must
  loft (two panel breaks per side).
- **Nose motor vs pure glider:** electric versions carry a nose outrunner + folding prop
  (Radian: "480 Outrunner, 960Kv" — [Great Hobbies spec]); a pure glider REPLACES that mass
  with nose ballast — the Gentle Lady electric conversion documents the swap
  ([Shlaes conversion notes](https://shlaes.com/Airplanes/GentleLady.htm)). **App rule:
  `n_motors: 0` ⇒ no mount, no motor wiring, and a recorded nose-ballast provision equal to
  the power-system mass the CG closure needs at the same station** (the app's lumped ~28%
  AUW power allowance becomes ballast + a smaller RX pack; the bay keeps its ±30–40 mm
  battery-travel trim rule from RESEARCH_CONVENTIONAL §6).

---

## 7. Config axis: tractor vs pusher

Applicable to every type except glider-with-`n_motors:0`; defaults per type in the matrix (§10.4).

- **Efficiency: a pusher prop ingests the airframe wake — thrust losses ~2–15% depending on
  how much dirty air the disk swallows** ([tractor/pusher wing-propeller interaction study](https://www.researchgate.net/publication/301902109_Comparison_of_wing-propeller_interaction_in_tractor_and_pusher_configuration);
  practical overview: [Airplane Academy comparison](https://airplaneacademy.com/pusher-vs-puller-propeller-aircraft-compared/)).
  **App model: 5% cruise thrust/efficiency penalty for a pod pusher, 3% for a clean
  twin-boom pusher** (disk mostly outside the pod wake) — derived mid-band picks; the app
  carries no thrust model, so this lands only in guidance text and L/D notes.
- **Thrustline:** tractor — 2–3° down/right thrust [ND, p. 6]. Pusher — thrust axis through
  the vertical CG and parallel to the (rear) wing chord, or power changes pitch the
  aircraft: "if at all possible maintain parallelism with the rear wing chord while passing
  the thrust axis through the CG" [LEN-CAN §6 item 5]. High-pod pushers that can't reach the
  CG line take up-thrust instead — flag in guidance, don't silently accept.
- **Prop clearance:** full-scale anchor — FAR §23.925 requires ≥ 7 in ground clearance
  (nosewheel) / 9 in (tailwheel) ([14 CFR 23.925 as of 2017-08-29, eCFR point-in-time](https://www.ecfr.gov/on/2017-08-29/title-14/chapter-I/subchapter-C/part-23/subpart-E/section-23.925); [2016 CFR PDF, govinfo](https://www.govinfo.gov/content/pkg/CFR-2016-title14-vol1/pdf/CFR-2016-title14-vol1-sec23-925.pdf)).
  The app's aircraft are belly-landers (no gear, v1 doctrine), so the RC translation is:
  **tractor props survive belly landings only via nose-up flare geometry — require the prop
  tip circle to clear the belly line by ≥ 10° of rotation about the TE touch point (derived
  geometric rule); pushers and twin-booms clear trivially, which is WHY the FPV class is
  pusher** (§5). Gliders: folding prop, no check.
- **Defaults per type:** tractor — conventional, biplane, glider (motorized), tandem
  ("Tractor designs … result is what is usually called a tandem-wing design" [LEN-CAN §6]);
  pusher — flying wing (v1), delta (every §1.1 reference airframe), canard [LEN-CAN §5],
  twin_boom ([Wikipedia twin-boom], §5).

## 8. Config axis: single vs twin motor

`n_motors: 2` = wing-mounted nacelles (V3_PLAN.md). Applies to conventional, twin_boom
(motors on booms' noses = tractor twins, or pod pusher single), glider (rarely; allowed),
biplane/tandem/canard/delta: single only (no reference practice for twins at this scale).

- **Nacelle spanwise placement: 25–35% semi-span, default 30%.** Full-scale anchors: test
  configurations at "approximately 32 percent semispan" ([NASA TM X-3207](https://ntrs.nasa.gov/api/citations/19750013210/downloads/19750013210.pdf))
  and "approximately 35% of the semispan" ([NASA interference-drag study](https://ntrs.nasa.gov/api/citations/20200002417/downloads/20200002417.pdf));
  inboard of ~25% the prop disk hits the fuselage/pod clearance, outboard of ~35% the
  engine-out yaw moment grows for nothing.
- **Asymmetric thrust (the RC Vmc note).** Full-scale certification defines a minimum
  control speed Vmc below which rudder cannot hold one-engine-out yaw; the RC failure case
  is the same — one motor at full thrust at low airspeed. **App rule (derived, since the
  app sizes no thrust): the fin+rudder sized by V_V must
  generate more yaw moment at 1.2 × V_stall than one motor at the nacelle arm produces at
  static thrust ≈ AUW/2 — checked with the app's recorded motor positions; failing that,
  guidance mandates counter-rotating props and differential-thrust mixing** (standard FPV
  twin practice — the MyTwinDream's selling point is exactly twin "redundancy in those times
  of unexpected motor/ESC loss" ([FPV Model blog](https://fpvplane.blogspot.com/2015/07/mfd-mytwindream-1800mm-fpv-plane.html))).
- **Mass/CG bookkeeping:** the lumped power-system mass (~28% AUW, ARCHITECTURE.md) splits into two
  nacelle stations at the wing (x ≈ wing LE at 30% semi-span, mirrored) instead of one nose
  station — the CG moves AFT, so the battery slides FORWARD in the bay to close the same CG;
  record both nacelle stations in `config` and let the §6-style battery-travel rule absorb
  the shift. Roll inertia rises (mass at 30% semi-span); guidance notes slower roll starts.
- **When twins make sense:** nose kept free for payload/camera (the MyTwinDream's mission,
  above) and prop-out redundancy; NOT for thrust at this scale — two small motors weigh and
  cost more than one right-sized one. Default `n_motors: 1` for every type; 0 legal only for
  glider (V3_PLAN.md).

## 9. Config axis: V-tail vs conventional vs T-tail

Applies to conventional, glider, twin_boom (as inverted-V on the booms); tailless types take
`tail_type: None`; biplane stays conventional (all §4.1 references).

- **V-tail sizing — the actual rule [MA-T]:** total V-tail area equals the SUM of the
  required horizontal and vertical areas, `S = S_v + S_h`, mounted at dihedral
  `A = arctan(sqrt(S_v/S_h))`; effective areas obey the tan² split — `S_Heff = S·cos²A`,
  `S_Veff = S·sin²A`. **The known underprediction trap, stated in the source:** "The
  effective areas are not the projected areas (which would use the sine and cosine
  directly). If projected areas were used to size the V-tail, it would be too small" [MA-T].
  The simplified theory is experimentally verified "for dihedral angles up to about 40°"
  [NACA823 via the FMSG Alling summary]. Sized this way the V-tail "provides essentially the
  same stability characteristics as a conventional tail" [MA-T] — no extra fudge factor.
- **Dihedral band: 30–40° from horizontal, default 35°.** Derived from the formula with this
  app's own bands: S_v/S_h = V_V·S·b/l_v ÷ V_H·S·c̄/l_h with l_v ≈ l_h gives S_v/S_h ≈
  0.35–0.55 for the §2-of-RESEARCH_CONVENTIONAL defaults → A = arctan√(0.35…0.55) =
  30.6–36.6°; [NACA823]'s 40° validity limit caps the band.
- **Ruddervator mixing:** elevator and rudder commands sum on the two surfaces (standard
  transmitter V-tail mix); each surface must carry BOTH throws, so budget total deflection
  ±(elevator + rudder/2) ≈ ±25° and note the coupling: yaw input rolls slightly (the
  Dutch-roll tuning trick of tip fins on the V panels is documented in [MA-T]). One 9 g
  servo per panel, same pocket doctrine.
- **T-tail:** stab atop the fin. Caveats, both cited: (1) **deep stall** — "a high angle of
  attack would likely place the wing separated airflow into the path of the horizontal
  surface of the tail … loss of elevator authority and … inability to recover"
  ([SKYbrary, Deep Stall](https://skybrary.aero/articles/deep-stall)); (2) **mass up the
  fin** — "T-tails can be aerodynamically cleaner but structurally heavier"
  ([SKYbrary, T-tail](https://skybrary.aero/articles/t-tail)). **App rules: fin structure
  mass factor ×1.4 (derived mid-value for the reinforced fin + joint), stab mass moved to
  fin-tip height in the CG/inertia bookkeeping, elevator pushrod routed UP the fin through
  the same Ø8.25 pipe doctrine (V3_PLAN.md), deep-stall note in guidance.** Benefits earning
  it: stab out of prop wash and clear of grass on belly landings — the RC reasons T-tails
  exist on gliders.
- **Volumes unchanged across tail types:** V_H/V_V bands are properties of the airframe, not
  the tail shape — V-tail converts them via the tan² rule, T-tail keeps them as-is.

## 10. Config axis: high / mid / low wing

Applies to conventional, glider (high/shoulder default), twin_boom (high default — §5
airframes); meaningless for flying wing/delta; biplane is its own answer; canard/tandem take
it as the REAR/main wing's position (front wing offset by the §2/§3 gap rules).

### 10.1 Effective dihedral per position

The spread is real and worth recording:

- Full-scale rough rule: "a high wing configuration can provide about 5° of effective
  dihedral over a low wing configuration" [AWEB] — i.e. ±2.5° about mid.
- RC practice ladder (with ailerons): "high wing 1°, mid 2°, low 3°" geometric dihedral
  ([RCU dihedral thread](https://www.rcuniverse.com/forum/scratch-building-aircraft-design-3d-cad-174/1769864-how-much-dihedral.html),
  already the basis of RESEARCH_CONVENTIONAL §4) — a ±1° geometric compensation about mid,
  the "classic ±1°".

The two are consistent once you note the RC ladder is what builders LEAVE IN after the
fuselage effect is accounted for at model Reynolds numbers and fat lifting fuselages.
**App rule: wing position contributes an effective-dihedral increment to `Cl_beta` —
high +1.5°, mid 0°, low −1.5° (recommended value inside the ±1…±2.5° spread), and the
geometric-dihedral default per position follows the RC ladder (high 1–2°, mid 2°, low 3°)**
so the summed dihedral effect lands in the same place for every position.

Sweep cross-term, for the record (already relevant to `config_axes` bookkeeping): "10° of
sweepback … provides about 1° of effective dihedral" [AWEB] vs "2 to 3 degrees of sweepback
are equivalent to 1 degree of dihedral" [KAV] — the spread exists because the sweep
contribution scales with CL (`Cl_beta,sweep ∝ CL·tanΛ`); at RC climb/thermal CLs the [KAV]
end applies, at cruise the [AWEB] end. Use 4° sweep ≡ 1° dihedral at the app's cruise CL
(derived mid-value); the tailless path's own strip model already handles its sweep properly.

### 10.2 Stability/handling per mission

- **High:** most stable (pendulum + fuselage effect), payload/battery low in the pod, the
  trainer/FPV default — every §5.1 and §6.1 airframe is high/shoulder wing.
- **Mid:** neutral roll coupling, cleanest interference drag, the aerobatic choice —
  symmetric-section sport designs.
- **Low:** least effective dihedral (hence the +3° geometric), fastest-looking, sport/warbird;
  needs the belly-landing caveat below.

### 10.3 Belly-lander practice (no landing gear, v1 doctrine)

RC models in this class land on grass without gear. High/shoulder wing keeps the pod flat on
its belly with the prop (pusher) or wing (tractor) clear; low wing lands ON the wing skin —
require: no servo horn, hatch lip, or conduit mouth on the lower wing surface inboard of 40%
semi-span for low-wing designs (derived rule; it is the printed-part version of why trainers
are high-wing — "high wing sitting ON the fuselage" is the recognizable-trainer cue,
RESEARCH_CONVENTIONAL §8). Skids/EVA pads are builder-side and out of scope.

### 10.4 Applicability matrix (types × axes)

| type | tractor/pusher | n_motors | tail_type | wing_position |
|---|---|---|---|---|
| flying_wing (v1) | pusher (fixed) | 1 | None | None |
| conventional (v2) | tractor default, pusher legal | 1 default, 2 legal | conventional / t_tail / v_tail | high default, mid/low legal |
| delta | pusher default (§1.1), tractor legal | 1 | None (fins via planform) | None |
| canard | pusher default [LEN-CAN §5], tractor→see tandem | 1 | None (tip fins) | rear wing mid default |
| tandem | tractor default [LEN-CAN §6], pusher legal | 1 | None (aft fin / tip fins §3) | rear high / front low (§3 gap) |
| biplane | tractor (fixed — §4.1 practice) | 1 | conventional only | N/A (two wings) |
| twin_boom | pusher default (§5), tractor-twins-on-booms legal | 1 or 2 | conventional-between-booms default, inverted-V legal | high (fixed — §5.1 practice) |
| glider | tractor (nose, folding) or none | 0 or 1 | conventional / t_tail / v_tail | high/shoulder default |

---

### Coefficient quick-reference (transcription target for `config_defs.py` and type modules)

| # | Quantity | Hard band | Default | Section |
|---|---|---|---|---|
| 1 | Delta LE sweep [°] | 45–60 | 50 | §1.2 |
| 2 | Delta AR | 1.8–3.5 | 2.5 | §1.2 |
| 3 | Delta CG [% MAC] (validation anchor) | 15–22 | 18 | §1.2 |
| 4 | Delta static margin (tailless band) | 0.03–0.15 | 0.06–0.12 target | §1.2 |
| 5 | Delta fin area [% S] | 4–9 | 6 (center fin) | §1.2 |
| 6 | Delta elevon chord [frac local chord] | 0.20–0.25 | 0.20 | §1.2 |
| 7 | Delta wing loading [g/dm²] | 20–45 | 30 | §1.2 |
| 8 | Canard area ratio Sc/S | 0.15–0.40 | 0.25 | §2.2 |
| 9 | Canard CL ratio CLf/CLr | > 1.0 hard | 1.4–1.6 | §2.2 |
| 10 | Canard static margin k [× MAC_rear] | 0.20–0.25 | 0.22 | §2.2 |
| 11 | Rear-wing efficiency behind canard/front wing | — | 0.8 | §2.2 / §3.2 |
| 12 | Canard volume V_C (derived) | 0.5–0.9 | 0.7 | §2.2 |
| 13 | Canard AR (≥ wing AR) | 5–8 | 6 | §2.2 |
| 14 | Canard elevator chord [frac canard chord] | 0.2 | 0.2 | §2.2 |
| 15 | Canard aft/fwd lateral-area moment ratio | ≥ 1.25 | 1.25 | §2.2 |
| 16 | Tandem front lift share | 0.45–0.60 | 0.52 | §3.2 |
| 17 | Tandem CLf/CLr | ≥ 1.2 | 1.3 | §3.2 |
| 18 | Tandem stagger L [× MAC_rear] | 2.5–3.5 | 3.0 | §3.2 |
| 19 | Tandem gap [× MAC], rear wing high | 0.3–1.0 | 0.5 | §3.2 |
| 20 | Tandem decalage [°, front +] | +0.5–+2 | +1.0 | §3.2 |
| 21 | Tandem/canard per-wing AR | 5–7 | 6 front / 5 rear | §3.2 |
| 22 | Biplane gap/chord | 0.8–1.25 | 1.0 | §4.2 |
| 23 | Biplane stagger [× chord, fwd +] | 0–0.5 | 0.25 | §4.2 |
| 24 | Biplane decalage [°, upper +] | 0–1 | 0.5 | §4.2 |
| 25 | Biplane per-wing lift effectiveness | 0.80–0.90 vs g/c 0.8–1.5 | 0.85 @ g/c 1.0 | §4.2 |
| 26 | Biplane total area vs monoplane | +10–25% | +15% | §4.2 |
| 27 | Biplane lower/upper area ratio | ≥ 0.5 | 1.0 | §4.2 |
| 28 | Biplane per-wing AR | 4–6 | 5 | §4.2 |
| 29 | Biplane tail arm [× upper MAC] | ~3 | 3.0 | §4.2 |
| 30 | Biplane S_h [% total S] / elevator [% S_h] | 15 / 30 | 15 / 30 | §4.2 |
| 31 | Biplane dihedral [°] | 0–3 | 2 upper | §4.2 |
| 32 | Biplane wing loading [g/dm², total area] | 30–50 | 38 | §4.2 |
| 33 | Twin-boom tail arm l_h [× MAC] | 2.5–3.5 | 3.0 | §5.2 |
| 34 | Twin-boom V_H / V_V (fins split per boom) | 0.40–0.65 / 0.025–0.05 | 0.50 / 0.035 | §5.2 |
| 35 | Boom spacing [× prop diameter] | ≥ 1.15 | 1.2 | §5.2 |
| 36 | Twin-boom wing AR | 7–10 | 9 | §5.1 (derived) |
| 37 | Twin-boom wing loading [g/dm²] | 30–60 | 40 | §5.2 |
| 38 | Glider AR | 8–18 | 11 (printed 9–15) | §6.2 |
| 39 | Glider wing loading [g/dm²] | 15–45 | 18–30 LW-PLA | §6.2 |
| 40 | Glider V_H / V_V | 0.35–0.55 / 0.02–0.035 | 0.45 / 0.025 | §6.2 |
| 41 | Glider static margin | 0.05–0.15 | 0.10 | §6.2 |
| 42 | Glider fuselage length [× span] | 0.50–0.60 | 0.55 | §6.2 |
| 43 | Glider spar rule | span > 1.4 m or AR > 10 ⇒ carbon Ø8–10 | — | §6.2 |
| 44 | Glider polyhedral (R/E ships) | 5° + 3° @ 60% semi-span | per mission | §6.2 |
| 45 | Pusher efficiency penalty | 2–15% (lit.) | 5% pod / 3% twin-boom | §7 |
| 46 | Tractor down/right thrust [°] | 2–3 | 2 | §7 |
| 47 | Nacelle spanwise position [% semi-span] | 25–35 | 30 | §8 |
| 48 | Twin engine-out check | fin yaw ≥ 1-motor moment @ 1.2 V_stall | — | §8 |
| 49 | V-tail dihedral [° from horizontal] | 30–40 | 35 | §9 |
| 50 | V-tail area rule | S = S_h + S_v, A = arctan√(S_v/S_h) | tan² split | §9 |
| 51 | T-tail fin mass factor | ×1.3–1.5 | ×1.4 | §9 |
| 52 | Wing-position dihedral increment [° eff.] | high +1…+2.5 / low −1…−2.5 | ±1.5 | §10.1 |
| 53 | Geometric dihedral per position [°] | high 1–2 / mid 2 / low 3 | ladder | §10.1 |
| 54 | Sweep ≡ dihedral cross-term | 2–3°/1° (high CL) … 10°/1° (low CL) | 4°/1° @ cruise | §10.1 |

Sources for every row are in §1–§10 above; each default sits inside its cited or derived
spread. Rows marked derived in the text (2, 12, 25, 26, 33, 35, 36, 42, 43, 45 in part,
48, 51, 52, 54) are computed from cited inputs and carry their derivations inline.
