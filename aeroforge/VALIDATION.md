# VALIDATION.md - flying-wing physics sanity summary

Automatically generated from real optimizer runs (defaults: sport mission,
16 m/s cruise, 900x1200x300 mm box, one motor, LW-PLA). Every number below is
the direct output of the physics pipeline in `backend/physics/` - nothing is
hand-entered. The same checks run automatically in `tests/`:

- CG ahead of NP, static margin inside the TAILLESS band 3-15% (a flying wing
  flies at a lower margin than a tailed model)
- V_cruise >= stall_factor x V_stall; L = W at cruise (lift equation check)
- Reflexed section (positive Cm0) on every design - a tailless wing has no
  tail to trim against
- Fits the size box in span / length / height
- Real RC proportions: root chord >= 0.30 x span for swept/BWB wings, and a
  centre body deep enough to hold the pack. Reference airframes: Skywalker X5
  Pro 1280 mm span / 717 mm root (0.56); SonicModell AR Wing 900 / 482 (0.54)
- Vertical surfaces 2-9% of wing area, sized from real flying-wing practice
  rather than the tail-aft V_V band; bell-spanload wings carry none at all
- One valid watertight solid per design; STL mesh area >= 98.5% of BRep

## Representative designs (one per planform family)

| Quantity | swept | bwb | plank | bell |
|---|---|---|---|---|
| Span [mm] | 1040 | 1078 | 1038 | 964 |
| Wing area [dm^2] | 30.9 | 30.6 | 18.3 | 11.8 |
| Aspect ratio | 3.50 | 3.80 | 5.88 | 7.88 |
| Root chord [mm] | 391 | 375 | 202 | 201 |
| Root chord / span | 0.38 | 0.35 | 0.19 | 0.21 |
| Tip chord [mm] | 189 | 157 | 147 | 40 |
| Taper ratio | 0.48 | 0.42 | 0.73 | 0.20 |
| LE sweep [deg] | 25.0 | 21.9 | 3.7 | 25.8 |
| Washout [deg] | 2.0 | 2.0 | 1.9 | 8.0 |
| Airfoil | RFX-7 reflexed | RFX-11 reflexed | RFX-9 reflexed | RFX-9 reflexed |
| Body depth scale | 1.80 | 2.20 | 1.80 | 1.80 |
| Body chord scale | 1.22 | 1.38 | 1.12 | 1.14 |
| Equipment bay [mm] | 286 | 310 | 136 | 138 |
| All-up mass [g] | 1065 | 1063 | 683 | 454 |
| Wing loading [kg/m^2] | 3.44 | 3.48 | 3.73 | 3.85 |
| CL cruise | 0.215 | 0.217 | 0.233 | 0.241 |
| L/D cruise | 14.6 | 12.6 | 13.2 | 13.7 |
| V stall [m/s] | 8.3 | 7.6 | 7.8 | 8.9 |
| Stall margin [x] | 1.92 | 2.09 | 2.06 | 1.79 |
| Re at MAC [k] | 344 | 344 | 195 | 156 |
| MAC [mm] | 314 | 314 | 178 | 142 |
| NP [mm from nose] | 259 | 280 | 83 | 150 |
| CG [mm from nose] | 223 | 233 | 74 | 129 |
| CG [%MAC] | 14 | 10 | 20 | 10 |
| Static margin | 0.112 | 0.148 | 0.053 | 0.150 |
| Vertical surfaces | winglets | winglets | center_fin | none |
| Vert. area [% wing] | 3.2 | 3.6 | 6.9 | 0.0 |
| Vert. height [mm] | 80 | 85 | 114 | 0 |
| Motor stations | 1 | 1 | 1 | 1 |
| Feasible | yes | yes | yes | yes |

## The five wing characters (same request)

Each column is a full independent solve of the same constrained problem with a different planform family and different targets, so the differences below are physics and shape, not styling.

| Quantity | Swept Sport | Long-Range Cruiser | Bell-Distribution Floater | Plank Park Flyer | Speed Delta |
|---|---|---|---|---|---|
| Planform | swept | bwb | bell | plank | swept |
| Span [mm] | 1091 | 1200 | 1200 | 1200 | 778 |
| Aspect ratio | 3.70 | 3.80 | 7.60 | 6.02 | 3.77 |
| Root chord [mm] | 397 | 411 | 248 | 208 | 291 |
| Taper ratio | 0.45 | 0.42 | 0.26 | 0.90 | 0.38 |
| LE sweep [deg] | 25.0 | 21.9 | 25.6 | 5.6 | 27.0 |
| Washout [deg] | 2.0 | 2.0 | 8.0 | 0.4 | 2.0 |
| Airfoil | RFX-7 reflexed | RFX-11 reflexed | RFX-9 reflexed | RFX-9 reflexed | RFX-7 reflexed |
| Vertical surfaces | winglets | winglets | none | center_fin | twin_fin |
| All-up mass [g] | 1098 | 1271 | 638 | 814 | 559 |
| Wing loading [kg/m^2] | 3.42 | 3.35 | 3.36 | 3.40 | 3.48 |
| V stall [m/s] | 8.29 | 7.46 | 8.32 | 7.34 | 8.46 |
| V cruise [m/s] | 16.0 | 16.0 | 16.0 | 16.0 | 20.0 |
| L/D cruise | 14.85 | 12.75 | 13.53 | 12.54 | 11.92 |
| Static margin | 0.115 | 0.150 | 0.150 | 0.053 | 0.050 |
| Feasible | yes | yes | yes | yes | yes |

Normalized traits (min-max across the five):

| Trait | Swept Sport | Long-Range Cruiser | Bell-Distribution Floater | Plank Park Flyer | Speed Delta |
|---|---|---|---|---|---|
| Stability | 0.71 (11% static margin, 3.2% fin area) | 0.93 (15% static margin, 3.6% fin area) | 0.67 (15% static margin, no fins (bell spanload)) | 0.40 (5% static margin, 6.6% fin area) | 0.31 (5% static margin, 3.2% fin area) |
| Speed | 0.23 (16.0 m/s cruise, 3.4 kg/m2) | 0.05 (16.0 m/s cruise, 3.4 kg/m2) | 0.08 (16.0 m/s cruise, 3.4 kg/m2) | 0.20 (16.0 m/s cruise, 3.4 kg/m2) | 1.00 (20.0 m/s cruise, 3.5 kg/m2) |
| Slow flight | 0.20 (8.3 m/s stall) | 0.90 (7.5 m/s stall) | 0.17 (8.3 m/s stall) | 1.00 (7.3 m/s stall) | 0.05 (8.5 m/s stall) |
| Efficiency | 1.00 (L/D 14.8) | 0.32 (L/D 12.7) | 0.57 (L/D 13.5) | 0.25 (L/D 12.5) | 0.05 (L/D 11.9) |

## Mission sensitivity (swept planform)

| Mission | Wing loading [kg/m^2] | V stall [m/s] | L/D |
|---|---|---|---|
| sport | 3.44 | 8.33 | 14.6 |
| fpv_cruiser | 3.44 | 7.83 | 14.0 |
| thermal_floater | 3.43 | 7.83 | 14.0 |
| park_flyer | 3.43 | 7.84 | 14.0 |

Cruise-speed sensitivity: wing loading at 11 m/s = 3.05 kg/m^2, at 22 m/s = 3.44 kg/m^2 - increases with cruise speed as expected.
