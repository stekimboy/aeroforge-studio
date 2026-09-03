"""Elevon servo pockets and fused control horns for the printed flying wing.

WHAT THIS MODULE IS FOR
-----------------------
An elevon that is hinged but not *driven* is decoration. This module supplies
the two halves of the drive that the airframe has to carry in its own
geometry:

  1. a **servo pocket** recessed into the LOWER wing surface, sized to a real
     9 g servo, blind against the upper skin, with a cable exit pointing at
     the root; and
  2. a **control horn** standing DOWN from the underside of the control
     surface, *fused into it* so the surface comes off the plate with its horn
     already on, carrying real pushrod holes.

What it deliberately does NOT build is the linkage. The user bends the
pushrod out of music wire by hand, so the job here is to put the servo output
and the horn holes where a hand-bent Z-bend rod can join them and the throws
come out right - not to model a rod.

It never imports ``backend.cad.geometry`` (that would be an import cycle),
nor ``hinges`` nor ``hatch``. The wing arrives by duck typing and only has to
expose:

    wing.section(f)      -> object with .le (Vector, mm), .chord (mm)
    wing.crown_z(f, xc)  -> UPPER surface z at span fraction f, chord frac xc
    wing.keel_z(f, xc)   -> LOWER surface z, same arguments
    wing.xc_at(f, x_mm)  -> chord fraction of an absolute x station
    wing.half            -> semi-span, mm
    wing.fb              -> body half-width / semi-span

`f` is SIGNED: +1 right tip, -1 left tip. All lengths in and out of this
module are MILLIMETRES, x aft from the nose datum, y spanwise, z up.


THE SERVO: TowerPro SG90, AND WHY THE NUMBERS DISAGREE
-------------------------------------------------------
The SG90 is the part this airframe is dimensioned around - the builder's own
call (2026-08-19): it is the same 9 g class as the HXT900 it replaces, drives
the same loads, and is *far* more widely available, which is the whole reason
for the swap. Its footprint is the "de facto industry footprint ... the one
most 3D-printable mounts on Thingiverse are dimensioned around" (Sunservo),
shared with the MG90S and the FS90. It is not one part from one factory, and
the sources do not agree:

* The SG90 datasheet family and the retailers quote the CASE as
  **22.8-23.1 x 12.2-12.5 x 26-29 mm**:
    https://handsontec.com/dataspecs/motor_fan/SG90-Servo.pdf
    https://www.espboards.dev/sensors/sg90/            (22.8 x 12.2 x 28.5)
    https://hackaday.io/page/2936-sg-90-hobby-servo    (23 x 12.2 x 29)
    https://protosupplies.com/product/servo-motor-micro-sg90/  (23 x 12 x 26)
* **With the mounting ears** the part is **32.2-32.5 mm** long and about
  **30-32 mm** tall to the top of the output boss:
    https://sunservo.com/blog/micro-servo-size-chart/   ("32.3 mm long and
        30 mm high ... needs a 23 x 12.2 mm rectangular pocket")
    https://protosupplies.com/product/servo-motor-micro-sg90/  ("32 mm with
        mounting ears", "32 mm with the rotating shaft attached")

The 21-22 mm "height" that some listings for this class quote is the plain
case without the output boss; the 26-29 mm figures include it. Both are honest
and they describe the same servo measured to different features, which is
exactly why this module stores a *feature-by-feature* dictionary instead of an
L x W x H triple.

The one dimension every source warns about is the ears. Hackaday's teardown -
the most careful public measurement of the family - says flatly that the
servos "have two 'ears' with mounting holes, but the sizes of the ears and
the holes vary greatly", and AliExpress's own SG90 wiki says the mounting
holes "are not standardized across all manufacturers" and to "account for an
additional 2-3 mm of clearance around the housing". **That is the reason this
module does not capture the ears.** A pocket that clamps a 32.2 mm ear span
is a pocket that rejects half the servos on the market.

The spline is 21 teeth / ~4.9 mm across (Sunservo; 4.99 measured), rising
straight out of the gear-head housing offset toward one end of the case,
6.20 mm from that end. Horn holes on the supplied
single arm are on a 2 mm pitch (Thingiverse "MG90/SG90 Servo Horn Arm
Assortment", crashworks3d, thing:4617038), the outermost about 11-12 mm from
the spline centre.


HOW A REAL FLYING WING CARRIES ITS ELEVON SERVO (and what that dictates here)
-----------------------------------------------------------------------------
* **It goes in the bottom skin, not the top.** The elevon horn is on the
  underside, so the pushrod, the arm and the servo all live below the wing
  where a linkage can be reached with the model upside down on the bench.
* **It lies on its side.** The builder's measured SG90 is 32.55 mm tall to the
  top of its spline and no sport wing is 32 mm thick where the elevons are, so
  the servo is laid with its 12.4 mm face up: 23.0 mm chordwise, 32.6 mm
  spanwise, **12.4 mm deep**, and the output shaft axis ends up SPANWISE. That
  is the orientation
  that makes the kinematics right as well as the packaging: the arm swings in the
  chordwise-vertical plane about a spanwise axis, exactly parallel to the
  elevon hinge axis, so arm and horn form a true planar four-bar and the
  pushrod stays in one plane through the whole throw. (Mounting the shaft
  vertically - the classic thick-wing aileron installation - puts the arm and
  the horn 90 degrees out of plane and needs 29 mm of section.)
* **The ears are not captured; the servo is bedded in.** Foam-wing practice
  is a rectangular pocket a shade bigger than the servo, the servo pushed up
  against the pocket roof and held with hot glue, silicone or double-sided
  tape. It survives crashes better than screws in foam and it tolerates the
  ear scatter above. This module follows that, and adds M2 pilot holes in the
  pocket roof *only when the section is deep enough to give the screw real
  material* - reported, never silently skipped.
* **The pocket is blind against the top skin.** Cutting through to the crown
  would put a 33 x 30 mm hole in the upper surface of the wing at 30% chord,
  which is both a structural and an aerodynamic own goal. `servo_bay` refuses
  to break the crown: it walks the servo INBOARD until the section is deep
  enough, and reports the move.
* **The cable goes to the root, and it leaves from a corner.** The lead exits
  the case at the end opposite the spline - not from the middle of the servo -
  and the spline is itself offset `spline_dx_mm` from its own end, so on the
  real moulding the lead is hard against the far corner. The servo is installed
  spline-outboard, which puts that corner at the FORWARD-INBOARD corner of the
  pocket, and the channel starts there. `cable_exit` is published in world
  coordinates for `backend.cad.conduits` to pick up.
* **The pocket holds the SERVO, not the arm's swing.** The arm is fitted after
  the servo drops in and works below the skin in free air, tangent to the lower
  surface - which is how a recessed foamie servo is actually rigged - so no
  swept volume is carved into the wing for it. What the pocket owes the arm is
  a number, not a cavity: `arm_free_deg` is how far it can swing before its tip
  climbs back above the keel line, and `servo_bay` warns if that is less than
  the throw the linkage needs. On the test wing it is 38 deg against the 20 deg
  an X5 flies on. Reserving the swing instead cost a well reaching 22 mm
  forward of the shaft, which drove the fit search inboard looking for a
  section deep enough to roof it.


PRINT CLEARANCE, BAMBU LAB H2D, 0.4 mm NOZZLE, PLA, DEFAULT PROFILE
--------------------------------------------------------------------
`SERVO_CLEARANCE_MM = 0.35` per face - a *drop-in* clearance fit, not a press
fit. The published windows for a Bambu-class machine put clearance fits at
0.2-0.3 mm and interference fits at 0.1-0.15 mm of overlap:
    https://www.raphaelgarcia.me/blog/2024/9/9/tolerances-and-fits-in-3d-printing-how-to-get-it-right-with-your-bambu-lab-x1c
    https://x3dstudios.com/blog/3d-printing-tolerances-guide
0.35 mm is deliberately at the top of that band, for three reasons that are
specific to this pocket:

  1. the object dropping in is an injection moulding whose own tolerance is
     the dominant error (the sources above disagree by 2 mm on the ear span
     alone), so the fit has to be forgiving, not tight;
  2. the pocket is **cut out of a doubly curved skin**, so its walls are not
     truly flat and the effective gap varies across the depth; and
  3. the wing prints with this pocket **facing the build plate**, so the
     pocket mouth sits in the first few layers and eats elephant-foot growth.
     PrusaSlicer's own guidance is that first-layer compensation "around
     0.2 mm usually work[s] well for the default 0.4 mm nozzle", and Bambu's
     0.4 mm profiles ship 0.15 mm - which means up to ~0.15 mm per side of
     unplanned growth right at the mouth if the profile's compensation is off.
        https://help.prusa3d.com/article/elephant-foot-compensation_114487

**Tree supports will get into this pocket, and that is fine.** The pocket
opens downward on a wing printed flat, so it is a genuine cavity over the
plate and Bambu Studio's tree supports will grow into it - the community
answer to "supports in screw holes" is that the slicer supports anything it
can see, and the fix is a support blocker or a geometry that does not need
support (https://forum.bambulab.com/t/supports-in-screw-holes/83637,
https://wiki.bambulab.com/en/software/bambu-studio/support). The design
consequence is not "avoid the supports", it is **make the supports trivially
removable**:

  * every wall of the pocket is vertical - no draft, no undercut, nothing for
    a support branch to key into;
  * the roof is **flat**, not crown-following, so it is one clean bridge and
    one clean support interface instead of a curved ceiling that tree tips
    have to chase;
  * the ears are not captured, so there is no 2.5 mm deep blind ear slot for
    supports to fossilise in - the single worst feature you can put in a
    downward-facing pocket;
  * the M2 pilot holes are 1.7 mm, well under the ~1.1 mm + line-width floor
    at which support can enter a feature at all (see `hinges` for that
    calculation), so they stay empty.


THE CONTROL HORN
----------------
Commercial nylon micro horns for foam and park-flyer models are 11-13 mm wide
at the base, stand **20 mm** tall, and carry **4 holes of 1.5 mm** on the
blade (Hobbypark 20 x 11 mm micro horn; WMYCONGCONG 21 x 11 mm 4-hole; uxcell
16 x 17 mm 3-hole at 1.2 mm). 1.2-1.5 mm music wire with a Z-bend or a 1.2 mm
clevis pin is what goes through them.
    https://www.amazon.com/Hobbypark-Control-Airplane-Remote-Electric/dp/B019I79ZG0
    https://www.amazon.com/Nylon-Control-Airplane-Remote-Electric/dp/B07JX96TK1
    https://www.amazon.com/uxcell-Control-16x17mm-Plastic-Airplane/dp/B07PVPBSXN

The geometry rule that actually matters is the one Model Aviation states for
setting throws: *"choose a servo arm hole that matches the distance of the
control arm hole to the surface hinge axis to achieve the recommended
throws"* - equal radii give a 1:1 ratio, so the surface angle equals the
servo angle.
    https://www.modelaviation.com/article/Flight-Control-Servo-Balancing
The second rule, from the same body of practice, is to keep the horn arm as
long as the airframe allows and the servo arm as short as it allows, because
"the closer you put the linkage to the hinge line the more play you build in"
(RCU, "post style control horn set up"):
    https://www.rcuniverse.com/forum/questions-answers-154/11637378-post-style-control-horn-set-up.html

So `control_horn` publishes the radius of every hole from the hinge axis, and
`linkage` solves the actual four-bar rather than assuming the 1:1. The default
`r_design_mm` (11.0) is set EQUAL to the bay's default `arm_r_mm`, which is the
outer hole of the arm supplied with a 9 g servo, so the built-in case is the
parallelogram and the ratio comes out 1.000 by construction. That lands the
hole about 12 mm below the skin - the size Du-Bro sells into this class as the
Micro Control Horn (12.7 mm).
    https://www.dubro.com/products/micro-control-horns

**The horn has to stand in the servo arm's plane.** This is the one linkage
error that cannot be trimmed out. HobbyKing's install guide is blunt about the
mechanism: servo arms "are not designed for side forces, and a side force may
twist or fracture the horn causing an inflight failure"
(https://hobbyking.com/blog/servos-control-horns); Model Aviation adds that
"an off-axis pushrod will eventually ream out the hole in the control horn and
create a sloppy connection"
(https://www.modelaviation.com/control-linkage-roundup), and Great Planes that
any resulting free play is "greatly magnified, possibly causing destructive
control surface flutter". Nobody publishes a numeric tolerance, because the
answer in CAD is to make the offset zero by construction: `servo_bay` takes
`arm_y_mm` and places the CASE so the ARM lands on a given plane, and
`control_horn` takes `align_world_y` and solves its own station to match. The
residual is reported, not assumed.

**The pushrod hole is ONE plain 2.5 mm circle, by the builder's own spec.**
The first cut of this design used a keyhole (1.5 mm running seat + 2.5 mm
feed lobe) to let a hand-bent Z thread the hole without buying the ~1 mm of
lash that a plain 2.5 mm hole leaves over 1.2 mm wire - lash being the
documented start of flutter (Model Aviation). On the print it read as two
malformed circles, and the builder asked for a single clean 2.5 mm hole per
horn instead: their linkage, their call, recorded here so the keyhole is not
"restored" as a fix. The lash mitigation moves to the bench - a tight Z-bend,
or Du-Bro's answer of an E/Z-style keeper at the horn
(https://www.dubro.com/products/micro-push-rod-system). The keyhole form
survives in `_keyhole` behind `feed_d_mm`/`feed_offset_mm` for anyone who
wants it back.

The horn is raked slightly forward below the skin. That is not styling: the
horn is rigidly part of the surface, so through a deflection it sweeps a
circle about the hinge axis, and the part of that sweep which can hit the WING
is the part near the root. A horn root sitting less than about half the local
section thickness aft of the hinge plane swings *up into the wing's lower
skin* at 30 degrees of up-elevon. `HORN_DEFAULTS["setback_mm"]` keeps the root
clear, and the blade then leans forward so the holes still finish close to
under the hinge axis, which is what keeps the up and down throws equal.
`control_horn` will *verify* this by rotation and intersection when it is
handed the wing panel (`clearance_against`), rather than trusting the sums.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np
from cadquery import Edge, Shape, Solid, Vector, Wire
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.gp import gp_Pnt
from OCP.TopAbs import TopAbs_IN, TopAbs_OUT

__all__ = [
    "SERVO_SG90",
    "SERVO_CLEARANCE_MM",
    "BOSS_SEAT_EXTRA_MM",
    "HORN_DEFAULTS",
    "ServoBay",
    "servo_bay",
    "control_horn",
    "linkage",
]


# ---------------------------------------------------------------------------
# The servo
# ---------------------------------------------------------------------------
#
# Every entry is (value, source). The values are the DESIGN envelope: where
# the sources disagree we take the largest credible number, because a pocket
# that is 0.5 mm too big is invisible and a pocket 0.5 mm too small is a
# reprint. `_spread` records the disagreement so it can be reported.

SERVO_SG90: dict[str, Any] = {
    "name": "TowerPro SG90 / 9 g micro servo (SG90 / MG90S footprint)",

    # -- case -------------------------------------------------------------
    # MEASURED triangle-exact off the builder's own SG90 model ("REAL
    # SG90.stl", 2026-08-19) - a model the builder DESIGNED THEMSELVES in
    # CAD from the physical part, not a download (it superseded an earlier
    # file they had supplied by mistake: "I gave you the wrong file earlier").
    # The Thingiverse HXT900 reference that preceded it has been removed from
    # the repository; the HXT900 numbers survive only in this history.
    # Sliced feature by feature the same way the HXT900 before it was; the
    # face planes are exact to 0.01 mm (case top 24.750, head top 29.050,
    # ears 17.550/20.250, spline top 32.550). Same doctrine, same clearances -
    # only the envelope moved. Against the datasheet family this part is
    # TALLER (32.55 to the spline top vs the quoted 26-29) and its case runs
    # 24.75 where the listings say ~22.7: the builder's own part wins over
    # the catalogue, exactly as it did for the HXT900's gear head.
    #
    # This model draws NO wire tab and no lead grommet - the case is a clean
    # box - so the lead numbers below stay from the previous evidence rather
    # than being zeroed to match a simplification.
    "body_len_mm": 23.00,         # X_s, the direction the ears stick out in
    "lead_stub_mm": 1.0,          # moulded lead grommet protruding past the
                                  # shaft-side END face, low on the case -
                                  # the pocket end wall must clear it. The
                                  # REAL SG90 model does not draw it, but the
                                  # physical part has one, and the wire pipe's
                                  # mouth is REQUIRED to open at it - so the
                                  # measured-elsewhere 1.0 stays rather than
                                  # collapsing to a modelling omission.
    "body_wid_mm": 12.40,         # Y_s, the THIN axis - this is the pocket depth
    "case_h_mm": 24.75,           # Z_s, bottom of case to top of case body
    "head_h_mm": 29.05,           # Z_s, top of the GEAR-HEAD housing - FULL
                                  # case width (12.40) and standing 4.30 mm
                                  # above the case top; the region a bare
                                  # FS90-style drawing misses
    "head_fwd_of_spline_mm": 9.40,  # X_s, head reach from the spline centre
                                    # toward the far (non-shaft) end
                                    # (measured 9.393, rounded OUTWARD - the
                                    # envelope always takes the larger
                                    # credible number)
    "head_aft_of_spline_mm": 6.20,  # X_s, head reach past the spline toward
                                    # the shaft-side end - flush with the
                                    # case end face on this part
    "total_h_mm": 32.55,          # Z_s, bottom of case to top of the spline

    # -- mounting ears ----------------------------------------------------
    "ear_span_mm": 32.50,         # tip to tip, X_s (tips at -4.75 / +27.75
                                  # about a 0..23.00 case)
    "ear_thk_mm": 2.70,           # along Z_s (faces at 17.550 and 20.250)
    "ear_z_mm": 18.90,            # Z_s of the ear mid-plane, from case bottom
    "ear_hole_pitch_mm": 27.75,   # hole centre to hole centre, X_s. This
                                  # model's ears are UN-BORED (solid plates -
                                  # ray-probed at ten stations), so the pitch
                                  # is set mid-ear on THIS part's measured
                                  # ears (centre of each 4.75 mm plate), where
                                  # the real part's holes sit. The pocket
                                  # itself spans the whole ear either way;
                                  # only the roof's M2 pilots use this.
    "ear_hole_d_mm": 2.20,        # takes the supplied M2 self-tapper

    # -- output -----------------------------------------------------------
    "spline_d_mm": 4.99,          # circle-fit d 4.993, rms 0.004
    "boss_d_mm": 4.99,            # this moulding has NO raised collar: the
                                  # spline rises straight out of the gear head,
                                  # so the boss and the spline are one diameter
    "spline_dx_mm": 6.20,         # X_s, spline centre (16.800, on its own
                                  # face plane) to the NEAR end of case
    "hub_d_mm": 8.5,              # a fitted plastic horn's hub
    "arm_z_mm": 30.05,            # Z_s of a fitted horn's mid-plane: the hub
                                  # rides on the spline (29.05 head top ->
                                  # 32.55 spline top), so the arm sits ~1 mm
                                  # above the head - the same construction the
                                  # HXT900 entry used, re-evaluated at this
                                  # part's head height
    "arm_thk_mm": 2.0,
    "arm_hole_pitch_mm": 2.0,     # holes along the supplied single arm
    "arm_r_max_mm": 11.5,         # outermost hole, spline centre to hole

    # -- lead -------------------------------------------------------------
    # No manufacturer draws the lead and neither does the REAL SG90 model, so
    # these come from the earlier evidence: calibrated measurement of
    # manufacturer product photography, corroborated by the wire tab on the
    # builder's superseded first STL (Z_s 4.40-5.50, mid 4.95). Clones still
    # move it - which is why the pocket's lead channel is open to the lower
    # surface instead of being a tunnel aimed at one spot.
    "lead_len_mm": 250.0,         # TowerPro SG90 and its clones. FS90 200 +/-5.
    "lead_w_mm": 3.5,             # 3 x 26 AWG bonded side by side
    "lead_h_mm": 1.3,
    "lead_exit": ("short END face on the SAME side as the output shaft "
                  "offset, low - about 5 mm above the case bottom, well "
                  "below the mounting-ear plane"),
    "lead_exit_z_mm": 4.95,       # Z_s of the lead centreline, from the bottom

    "mass_g": 9.0,

    "_sources": {
        # Primary: the builder's own SG90 model - triangle-exact slices of
        # this STL are where the case, head, ear, bore and spline numbers
        # above come from.
        "_reference_model": [
            r"REAL SG90.stl"],
        "body_len_mm": ["https://hackaday.io/page/2936-sg-90-hobby-servo",
                        "https://www.aliexpress.com/s/wiki-ssr/article/sg90-servo-dimensions-datasheet"],
        "body_wid_mm": ["https://www.espboards.dev/sensors/sg90/",
                        "https://www.aliexpress.com/s/wiki-ssr/article/sg90-servo-dimensions-datasheet"],
        "case_h_mm": ["https://protosupplies.com/product/servo-motor-micro-sg90/"],
        "total_h_mm": ["https://www.espboards.dev/sensors/sg90/",
                       "https://hackaday.io/page/2936-sg-90-hobby-servo",
                       "https://protosupplies.com/product/servo-motor-micro-sg90/"],
        "ear_span_mm": ["https://sunservo.com/blog/micro-servo-size-chart/",
                        "https://protosupplies.com/product/servo-motor-micro-sg90/"],
        "spline_d_mm": ["https://sunservo.com/blog/micro-servo-size-chart/"],
        "arm_hole_pitch_mm": ["https://www.thingiverse.com/thing:4617038"],
        "lead_len_mm": ["https://protosupplies.com/product/servo-motor-micro-sg90/",
                        "https://www.aliexpress.com/s/wiki-ssr/article/sg90-servo-dimensions-datasheet"],
        "mass_g": ["https://protosupplies.com/product/servo-motor-micro-sg90/",
                   "https://handsontec.com/dataspecs/motor_fan/SG90-Servo.pdf"],
        # The one dimensioned manufacturer drawing for this case.
        "_drawing": ["https://www.pololu.com/file/0J1435/FS90-specs.pdf"],
        "ear_hole_d_mm": ["https://www.auselectronicsdirect.com.au/assets/brochures/TA0132.pdf"],
        # No drawing shows the lead. These are calibrated measurements off
        # manufacturer product photography, cross-checked on three parts.
        "lead_exit": ["https://www.adafruit.com/product/169",
                      "https://www.adafruit.com/product/2442",
                      "https://cdn-shop.adafruit.com/product-files/5592/C17481_SG92R_datasheet.pdf"],
        "lead_w_mm": ["https://www.hansenhobbies.com/products/connectors/Connectors.pdf",
                      "https://www.componentshop.co.uk/3-way-futaba-light-duty-flat-servo-wire-26-awg.html"],
    },

    # Where the published numbers actually disagree, in mm, so a caller can
    # report it instead of pretending the part is dimensionally certain.
    "_spread": {
        "body_len_mm": (21.0, 23.2),
        "body_wid_mm": (11.4, 12.5),
        "case_h_mm": (22.0, 26.0),
        "total_h_mm": (26.0, 32.6),     # the builder's measured part is 32.55
        "ear_span_mm": (32.0, 32.6),
        "ear_hole_pitch_mm": (27.0, 28.5),   # "sizes of the ears ... vary greatly"
        "lead_exit_z_mm": (3.5, 6.0),        # measured across three parts
    },
}

# Per-face clearance between the servo and the printed pocket.
# 0.25, by the builder's decision (round 2, superseding their earlier 0.50):
# "re-reference the STL file ... add a 0.25mm clearance around that whole
# model and implement that inverse shape/cut it into the airplane so it fits
# exactly as shaped." The pocket regions ARE the measured servo features
# (case + lead stub, ears, gear head, spline/hub), each a vertical prism so
# the servo still drops in from below, each offset by exactly this value.
# 0.25 is below the 0.35 mm printed-XY separability band, so this is a snug
# press-in fit by design - the builder test prints and owns the call.
SERVO_CLEARANCE_MM = 0.25

# Extra room at the TOP of the servo - the spline/hub end, where the horn
# goes and the output actually turns - measured along Z_s, the axis the servo
# pushes in on. Builder's decision from a test print, twice: "the top of the
# servo ... needs to be increased by 1.5mm; the very top of the servo is not
# pushing in all the way", then, after the first attempt missed: "there is
# 3.4mm gap where the servo/motor movement is and I need it to be extended by
# 1.5mm."
#
# THE 3.4 mm IS THE FEATURE THIS NUMBER MOVES, and naming it here is the point
# of this comment. The pocket's top slot runs from the gear-head housing's top
# to the boss region's top, the only part of the pocket that is boss and
# nothing else - what a builder measures in Fusion when they look at "the gap
# where the servo spins". On the HXT900 envelope that was head_h + cl = 27.65
# to total_h + cl + 1.0 = 31.05, i.e. the 3.40 mm the builder measured, going
# to 4.90 mm. On the measured SG90 the same construction gives a
# total_h - head_h + 1.0 = 4.50 mm slot, going to 6.00 mm. The 1.5 mm is the
# DECISION and it carries across; the figures either side of it are derived
# and move with the servo.
#
# The first attempt at this applied 1.5 mm to the boss patch's DEPTH instead
# (`boss_up`, into the wing along Y_s). That is a different axis: it made the
# blind pocket deeper under the hub, which is not where the servo was binding,
# and the builder correctly reported no visible difference against the old
# models. Depth is how far the pocket reaches INTO the skin; this is how far
# it reaches ALONG the servo. Do not confuse them again - if the servo will
# not seat, the question is always "which axis is it binding on".
BOSS_SEAT_EXTRA_MM = 1.5

# CHORDWISE swing room in the boss patch, per side of the shaft, beyond the
# fitted horn's hub - the third axis of that same patch (X_s: along the case,
# which lies chordwise in the wing). The patch spans
# 2 x (hub/2 + clearance + this), so 8.0 puts it at exactly 25.0 mm with the
# measured 8.5 mm hub and 0.25 clearance. Builder's decision from the printed
# part (2026-08-20): "the upper portion of the servo that spans 15mm long
# that allows tolerance for the servo horn to move ... right now it restricts
# the servo movement too much. Lets make it 25mm" - at the old 3.0 (15.0 mm
# patch) the swinging arm/hub scuffed the patch's chordwise end walls. The
# extra reach is counted against the hinge margin in `servo_bay`'s aft cap,
# because the boss end now sits further aft than the ear tips.
HORN_SWING_ROOM_MM = 8.0

# Minimum skin left over a blind pocket. Two perimeters at a 0.4 mm nozzle is
# the accepted floor for a printed wall; the caller's `wall` normally exceeds
# this and wins.
MIN_ROOF_MM = 0.8

# The pilot holes are only drilled if the roof is at least this thick, so the
# screw has real material and does not simply burst through the crown.
MIN_SCREW_ROOF_MM = 4.5

# The crown is sampled on a grid, so the true minimum between samples is a
# little lower than the one that is measured. This is the headroom kept on top
# of `wall` so a station that passes the grid test still has a full-thickness
# roof everywhere. It costs about 0.01 of semi-span on a 566 mm wing.
ROOF_SAFETY_MM = 0.15


HORN_DEFAULTS: dict[str, Any] = {
    "blade_t_mm": 2.8,        # 7 lines at 0.4 mm; nylon horns are 1.5-2 mm but
                              # PLA in bending needs the extra. Printables'
                              # printed-horn models call for 2-2.5 mm at 100%
                              # infill, and warn that a thin blade splits along
                              # the layer lines.
    "gusset_t_mm": 5.6,       # flared root, blended onto the surface
    "gusset_h_mm": 3.2,       # how far down the flare reaches
    "setback_mm": 4.0,        # forward safety floor, aft of the hinge PLANE
    "base_len_mm": 14.0,      # root chord of the blade
    "te_margin_mm": 2.5,      # the base stops this short of the measured
                              # trailing-edge lip, which is too thin to bond to
    "protrude_max_mm": 15.0,  # the horn stands AT MOST this far below the
                              # skin - the builder's cap (round 4: "shorter/
                              # stubbier, 15 mm long max"), and Du-Bro's micro
                              # horn class agrees. This cap, via the rake
                              # limit, is also what sets how far aft the hole
                              # can ride: depth*tan(rake_max) - a 15 mm horn
                              # physically cannot drive a hole parked at the
                              # trailing edge without toggling the four-bar.
    "corner_r_mm": 2.5,       # fillet radius on the two base corners
    "bevel_mm": 0.6,          # chamfer on both blade faces - the "beveled"
                              # look the builder asked for, and it also kills
                              # the elephant's-foot flare when the horn prints
                              # face-down
    "hole_aft_mm": 3.2,       # FLOOR for the hole line aft of the hinge plane.
                              # The horn itself now anchors to the trailing
                              # edge (builder's spec, asked three times), so
                              # the real hole line is derived from the measured
                              # local reach, far aft of this.
    "hole_d_mm": 2.5,         # ONE plain 2.5 mm bore, per the builder's own
                              # spec. The keyhole this used to be (1.5 mm
                              # running seat + 2.5 mm feed lobe, against
                              # Z-bend lash) read as two malformed circles on
                              # the print and the builder asked for a single
                              # clean hole instead - their linkage, their
                              # call. 2.5 mm threads any hand-bent 1.0-1.2 mm
                              # wire easily; take the lash out with a tight
                              # Z-bend or an E/Z-style keeper.
    "feed_d_mm": 0.0,         # 0 = no feed lobe: the hole is one circle
    "feed_offset_mm": 0.0,
    "hole_edge_mm": 4.0,      # material around the hole. 2.6 left ~1.4 mm of
                              # rim beyond a 2.5 mm bore and the builder
                              # flagged it as break-off thin; 4.0 leaves
                              # 2.75 mm of wall on every side, and because the
                              # blade tip grows with it the hole also sits
                              # visibly higher in the triangle.
    "n_holes": 1,             # one hole per horn, per the builder's spec
    "hole_pitch_mm": 4.0,
    "r_design_mm": 11.0,      # radius (hinge axis -> middle hole) we aim for.
                              # Matched to `arm_r_mm` for a 1:1 parallelogram -
                              # Model Airplane News' rule that horn radius from
                              # the hinge line should equal arm radius from the
                              # spline. 11 mm is the outer hole of a 9 g arm
                              # (SG90 arm_r_max 11.5) and lands the hole
                              # ~12 mm below the skin, which is the Du-Bro
                              # Micro Control Horn size (12.7 mm) for this
                              # class.
    "rise_mm": 0.0,           # retained for API compatibility; the blade top
                              # is now set from the MEASURED local skin band
                              # (the trailing edge of a reflexed section curls
                              # up past the hinge-axis plane, so a fixed top
                              # at z=0 would miss the surface entirely there)
    "rake_max_deg": 50.0,     # hole depth grows so atan(aft/depth) never
                              # exceeds this. Measured on both cached test
                              # wings with the TE-anchored hole: 50 deg gives
                              # the best balanced throws (-18.5/+21.7 on the
                              # 610, -14.0/+20.1 on the 1200 for +/-45 of
                              # servo); at 55 the up-throw halves and by 60
                              # the four-bar toggles and it is gone entirely.
    "max_deflect_deg": 30.0,
}

# The rake of the MINIMUM horn (hole at `hole_aft_mm`, radius `r_design_mm`).
# Kept for reference only: the hole line now scales aft with the surface
# (`hole_aft_frac`), so the real rake is per-design and the arm's neutral
# clock is solved by `linkage`, which searches for the angle that balances
# up-throw against down-throw.
HORN_DEFAULTS_RAKE_DEG = math.degrees(math.asin(min(
    HORN_DEFAULTS["hole_aft_mm"] / HORN_DEFAULTS["r_design_mm"], 0.99)))


# ---------------------------------------------------------------------------
# Small helpers (deliberately local - this module imports no sibling)
# ---------------------------------------------------------------------------

_TESS_TOL = 0.6


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def _unit(v: Vector) -> Vector:
    n = v.Length
    return Vector(0.0, 0.0, 0.0) if n < 1e-12 else v.multiply(1.0 / n)


def heal(solid: Solid) -> Solid:
    """ShapeFix a boolean result; keep it only if it is still one valid solid.

    Same guard the rest of the CAD uses: OCC will call a spline-on-spline
    boolean valid and then refuse to mesh faces whose trim curves it cannot
    rebuild, and the exported STL has a hole in it.
    """
    try:
        from OCP.ShapeFix import ShapeFix_Shape

        sf = ShapeFix_Shape(solid.wrapped)
        sf.Perform()
        fixed = Shape.cast(sf.Shape())
        solids = fixed.Solids()
        if len(solids) == 1 and fixed.isValid():
            return solids[0]
    except Exception:
        pass
    return solid


def tess_ratio(solid: Solid, tol: float = _TESS_TOL) -> float:
    """Fraction of the BRep area OCC actually meshes (0.0 if it cannot)."""
    try:
        verts, tris = solid.tessellate(tol)
    except Exception:
        return 0.0
    if not tris:
        return 0.0
    p = np.asarray([[v.x, v.y, v.z] for v in verts], dtype=float)
    idx = np.asarray(tris, dtype=int)
    a = p[idx[:, 1]] - p[idx[:, 0]]
    b = p[idx[:, 2]] - p[idx[:, 0]]
    area = 0.5 * float(np.sum(np.linalg.norm(np.cross(a, b), axis=1)))
    ref = float(solid.Area())
    return area / ref if ref > 0 else 0.0


def _one_valid_solid(shape: Any) -> Solid | None:
    try:
        solids = shape.Solids()
    except Exception:
        return None
    if len(solids) != 1 or not shape.isValid():
        return None
    return solids[0]


def _box(x0: float, x1: float, y0: float, y1: float,
         z0: float, z1: float) -> Solid:
    """Axis-aligned box from two corners, in world mm."""
    return Solid.makeBox(abs(x1 - x0), abs(y1 - y0), abs(z1 - z0),
                         Vector(min(x0, x1), min(y0, y1), min(z0, z1)))


def _fuse(parts: list[Solid]) -> Solid:
    out = parts[0]
    for p in parts[1:]:
        out = out.fuse(p)
    return out


# ---------------------------------------------------------------------------
# Reading the wing
# ---------------------------------------------------------------------------

def _skin(wing: Any, x: float, y: float) -> tuple[float, float]:
    """(crown z, keel z) at a world (x, y). Clamped to the local chord, so a
    probe that runs off the leading or trailing edge returns the edge value
    rather than nonsense."""
    f = _clamp(y / float(wing.half), -0.999, 0.999)
    xc = float(wing.xc_at(f, x))
    return float(wing.crown_z(f, xc)), float(wing.keel_z(f, xc))


@dataclass
class _Region:
    """One rectangular patch of the pocket, and how much headroom it needs.

    The pocket is NOT one box. The case needs the full 12.5 mm; the ears are
    2.5 mm of the span and need it only over a 33 mm strip; the output boss
    and the fitted horn hub need about 11 mm over a 17 mm strip. Sampling the
    section over one bounding box would demand the deepest requirement
    everywhere and push the servo needlessly inboard - on the 566 mm test wing
    that difference is worth about 0.09 of semi-span.
    """
    name: str
    x0: float
    x1: float
    y0: float
    y1: float
    need_up: float     # headroom above the servo datum plane, mm
    on_floor: bool     # does the servo have material down at the datum plane?


def _sample(wing: Any, r: _Region, n: int = 7) -> tuple[float, float]:
    """(max keel, min crown) over a region."""
    kmax, cmin = -1e18, 1e18
    for x in np.linspace(r.x0, r.x1, n):
        for y in np.linspace(r.y0, r.y1, n):
            c, k = _skin(wing, float(x), float(y))
            kmax = max(kmax, k)
            cmin = min(cmin, c)
    return kmax, cmin


# ---------------------------------------------------------------------------
# The servo bay
# ---------------------------------------------------------------------------

@dataclass
class ServoBay:
    """Everything the airframe needs to know about one elevon servo.

    `cutter` is CUT from the wing panel; it opens on the lower surface and is
    blind against the upper skin. `retainers` are solids to FUSE (empty unless
    the section is deep enough to carry screw bosses - see `notes`).
    """
    ok: bool
    cutter: Solid | None = None
    retainers: list[Solid] = field(default_factory=list)

    # where the servo ends up
    origin: Vector = field(default_factory=lambda: Vector(0, 0, 0))
    axes: dict[str, Vector] = field(default_factory=dict)
    spline: Vector = field(default_factory=lambda: Vector(0, 0, 0))
    spline_axis: Vector = field(default_factory=lambda: Vector(0, 1, 0))
    arm_hole: Vector = field(default_factory=lambda: Vector(0, 0, 0))
    arm_r_mm: float = 0.0

    # what the wiring module consumes
    cable_exit: Vector = field(default_factory=lambda: Vector(0, 0, 0))
    cable_dir: Vector = field(default_factory=lambda: Vector(0, 1, 0))

    # where it ended up vs where it was asked for
    y_frac: float = 0.0
    y_frac_requested: float = 0.0
    moved_inboard: bool = False

    dims: dict[str, float] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    reason: str = ""

    def arm_point(self, angle_deg: float = 0.0,
                  radius: float | None = None) -> Vector:
        """The pushrod hole on the servo arm, world mm.

        `angle_deg` is measured from straight down, positive aft - the angle
        the arm is clocked to on the spline at neutral. `arm_hole` is the
        `angle_deg = 0` case. Set it equal to the horn's rake and the two
        links become parallel, which is what makes the throw symmetric.
        """
        r = self.arm_r_mm if radius is None else float(radius)
        a = math.radians(angle_deg)
        return Vector(self.dims["x_shaft"] + r * math.sin(a),
                      self.dims["y_spline_mm"] - self.spline_axis.y
                      * (SERVO_SG90["total_h_mm"] - SERVO_SG90["arm_z_mm"]),
                      self.dims["z_shaft"] - r * math.cos(a))


def _arm_reach(sv: dict, r_arm: float, neutral_deg: float,
               travel_deg: float) -> tuple[float, float]:
    """How far aft and forward of the spline the SWINGING ARM sweeps.

    Not `r * sin(travel)`. The arm is clocked to the horn's rake at neutral
    (see :func:`linkage`), so its extremes are `neutral +/- travel`, and it is
    a blade with width, so its outer corner leads the centreline by
    `atan(w/2r)`. Getting this wrong is worth a real interference: an arm
    clocked 21 degrees aft and swung another 45 reaches 11.2 mm aft of the
    spline, where the naive sum says 8.1 and the pocket wall is at 11.0.

    The pocket no longer reserves this volume - the arm swings outside the skin
    - so this is now a REPORTED number, used to work out how much of the swing
    is below the keel line and to warn when part of it is not.
    """
    over = 2.5                          # arm sticks past its outer hole
    hw = 0.5 * (float(sv["arm_thk_mm"]) + 1.5)
    r_env = math.hypot(r_arm + over, hw)
    psi = math.degrees(math.atan2(hw, r_arm + over))
    hi = math.radians(neutral_deg + travel_deg + psi)
    lo = math.radians(neutral_deg - travel_deg - psi)
    aft = max(0.0, min(r_env, r_env * math.sin(hi))) + 1.0
    fwd = max(0.0, min(r_env, -r_env * math.sin(lo))) + 1.0
    return aft, fwd


def _servo_regions(sv: dict, cl: float, x_c: float, y_in: float,
                   dir_out: float, x_shaft: float) -> list[_Region]:
    """The pocket, cut up into the patches that actually need depth.

    Local servo frame -> world:
        X_s (body length, 23 mm, ears stick out along it)  -> world x
        Y_s (body width, 12.5 mm, the THIN axis)           -> world z
        Z_s (case height, 29 mm, the OUTPUT SHAFT AXIS)    -> world y

    `y_in` is the world y of the servo's bottom face (the cable end) and
    `dir_out` is +1 or -1, the direction along y in which the spline lies.

    Three patches, and NO arm well. The pocket houses the SERVO - case, mounting
    ears, output boss - and stops there. The arm is fitted after the servo drops
    in and swings below the skin in free air, tangent to the lower surface, so
    carving its swept volume into the wing buys nothing and costs section: the
    well used to reach 22 mm forward of the shaft and forced the servo further
    inboard to find a station deep enough to roof it.
    """
    depth = sv["body_wid_mm"] + cl                      # full-depth patches
    # The boss patch clears the SPLINE and the fitted horn's hub with working
    # room. Not the arm's full swept volume - the blade works below the skin
    # in free air - but the hub itself sits at skin level and turns with
    # every command, so it needs room fore and aft or it scuffs the pocket
    # edge on the first input. How MUCH room is the builder's call, made from
    # the printed part: see HORN_SWING_ROOM_MM.
    boss_up = 0.5 * sv["body_wid_mm"] + 0.5 * sv["hub_d_mm"] + cl

    def yspan(t0: float, t1: float) -> tuple[float, float]:
        a = y_in + dir_out * t0
        b = y_in + dir_out * t1
        return (min(a, b), max(a, b))

    hl = 0.5 * sv["body_len_mm"] + cl
    stub = float(sv.get("lead_stub_mm", 0.0))
    el = 0.5 * sv["ear_span_mm"] + cl
    bl = 0.5 * sv["hub_d_mm"] + cl + HORN_SWING_ROOM_MM  # hub + swing room
    # the gear-head housing: FULL case width, standing above the case top at
    # the spline end. The builder's test print found this the hard way - the
    # old model only relieved a hub-sized, half-depth patch here, and the
    # head hit the pocket floor before the ears ever reached their recess.
    head_h = float(sv.get("head_h_mm", sv["case_h_mm"]))
    h_fwd = float(sv.get("head_fwd_of_spline_mm", 0.0))
    h_aft = float(sv.get("head_aft_of_spline_mm", 0.0))

    y_case = yspan(-cl, sv["case_h_mm"] + cl)
    y_ear = yspan(sv["ear_z_mm"] - 0.5 * sv["ear_thk_mm"] - cl,
                  sv["ear_z_mm"] + 0.5 * sv["ear_thk_mm"] + cl)
    y_head = yspan(sv["case_h_mm"] - 0.5, head_h + cl)
    y_boss = yspan(head_h - 0.5,
                   sv["total_h_mm"] + cl + 1.0 + BOSS_SEAT_EXTRA_MM)
    # the moulded lead grommet: its own small relief low on the shaft-side
    # end face, exactly where the STL puts it (Z_s ~2-6), rather than
    # padding the whole end wall - the inverse SHAPE, per the builder
    y_stub = yspan(1.0, 7.0)

    out = [
        _Region("case", x_c - hl, x_c + hl, *y_case, depth, True),
        _Region("ears", x_c - el, x_c + el, *y_ear, depth, True),
        _Region("boss", x_shaft - bl, x_shaft + bl, *y_boss, boss_up, False),
    ]
    if stub > 1e-6:
        out.append(_Region("stub", x_c + hl - 1.0, x_c + hl + stub + cl,
                           *y_stub, depth, True))
    if head_h > sv["case_h_mm"] + 1e-6 and (h_fwd + h_aft) > 1e-6:
        out.insert(2, _Region("head", x_shaft - h_fwd - cl,
                              x_shaft + h_aft + cl, *y_head, depth, True))
    return out


def servo_bay(wing: Any, *, y_frac: float, x_hinge: float, sgn: float = 1.0,
              wall: float = 1.2, arm_y_mm: float | None = None,
              params: dict | None = None) -> ServoBay:
    """A 9 g servo recessed into the LOWER surface, driving the elevon.

    Parameters
    ----------
    wing
        The duck-typed blended wing (see module docstring).
    y_frac
        Wanted station as a fraction of semi-span. Magnitude is used; `sgn`
        picks the side. If the section is too shallow there the servo is
        walked INBOARD until it fits and `moved_inboard` is set.
    x_hinge
        World x of the elevon hinge line at that station. The pocket is kept
        `te_margin_mm` forward of it, and the pushrod reach is reported
        against it.
    arm_y_mm
        Optional world y for the ARM's swing plane. The arm is not at the
        servo's mid-station - it sits on the spline, near one end of the case -
        so asking for a station centres the case and lets the arm fall where it
        may. The pushrod has to be straight, which means the arm and the
        control horn must share a plane, so the caller normally wants to place
        the ARM and let the case sit wherever that puts it. Given, this wins
        over `y_frac` (which then only seeds the depth search).
    sgn
        +1 right wing, -1 left wing.
    wall
        Skin thickness to leave over the pocket, mm.
    params
        ``clearance`` (SERVO_CLEARANCE_MM), ``servo`` (dict override),
        ``spline_outboard`` (True), ``arm_r_mm`` (11.0),
        ``arm_neutral_deg`` (0.0 - only drives the free-swing reporting; the
        clock that balances the throws is solved by :func:`linkage`),
        ``arm_travel_deg`` (45.0),
        ``te_margin_mm`` (10.0), ``xc_aft_limit`` (0.50), ``xc_min`` (0.16),
        ``cable_stub_mm`` (7.0), ``cable_w_mm`` (5.6), ``cable_h_mm`` (4.2),
        ``walk_step`` (0.01), ``f_min`` (auto: just outboard of the body),
        ``screw_bosses`` (True).

    Returns
    -------
    ServoBay - ``ok`` False (with ``reason``) if no station on this side can
    take the servo without breaking the upper skin.
    """
    p = dict(params or {})
    sv = dict(SERVO_SG90)
    sv.update(p.get("servo") or {})
    cl = float(p.get("clearance", SERVO_CLEARANCE_MM))
    wall = max(float(wall), MIN_ROOF_MM)
    sgn = 1.0 if sgn >= 0 else -1.0
    out_dir = sgn if bool(p.get("spline_outboard", True)) else -sgn
    # 1:1 with HORN_DEFAULTS["r_design_mm"]: equal radii make the four-bar a
    # parallelogram, so the elevon turns degree for degree with the servo at
    # every angle. 11 mm is the outer hole of the supplied 9 g arm.
    arm_r = float(p.get("arm_r_mm", HORN_DEFAULTS["r_design_mm"]))
    te_margin = float(p.get("te_margin_mm", 10.0))
    xc_aft = float(p.get("xc_aft_limit", 0.50))
    xc_min = float(p.get("xc_min", 0.16))
    step = float(p.get("walk_step", 0.01))
    half = float(wing.half)
    f_min = float(p.get("f_min", max(float(wing.fb) + 0.02, 0.10)))
    # 0 = the arm hangs straight down at neutral. The exact clock that
    # balances up-throw against down-throw depends on where the horn hole
    # lands and is solved by `linkage`; the pocket no longer reserves the
    # swing, so this only drives the free-swing reporting.
    arm_neutral = float(p.get("arm_neutral_deg", 0.0))
    arm_travel = float(p.get("arm_travel_deg", 45.0))
    arm_reach = _arm_reach(sv, arm_r, arm_neutral, arm_travel)

    # The arm sits on the spline, `arm_z_mm` up the case, so it is this far
    # OUTBOARD of the servo's mid-station. Placing the arm means offsetting the
    # case by exactly this.
    span_len = sv["total_h_mm"] + 2.0 * cl + 1.0
    arm_off = float(sv["arm_z_mm"]) - 0.5 * span_len

    f_req = abs(float(y_frac))
    if arm_y_mm is not None:
        # Solve for the CASE station that lands the arm on the asked-for plane.
        # y_arm = y_mid + out_dir*arm_off and y_mid = sgn*f*half, so
        # f = (y_arm - out_dir*arm_off) / (sgn*half). Dividing by `half` alone
        # gets the right wing right and mirrors the offset the wrong way on the
        # left, which is worth 15 mm of asymmetry between the two servos.
        f_req = abs((float(arm_y_mm) - out_dir * arm_off)
                    / (sgn * max(half, 1e-9)))
    warnings: list[str] = []
    notes: list[str] = []

    # ---- find a station and a chord position that the section can take ----
    best = None
    deepest = (0.0, 0.0, 0.0)      # (usable depth, f, xc) - for the refusal
    f = _clamp(f_req, f_min, 0.95)
    while f >= f_min - 1e-9:
        sec = wing.section(sgn * f)
        chord = float(sec.chord)
        le_x = float(sec.le.x)
        # The servo is centred on the station in span, so the pocket straddles
        # y = sgn*f*half. Spanwise extent is fixed by the servo; only the
        # chordwise position is free.
        y_mid = sgn * f * half
        y_in = y_mid - out_dir * 0.5 * span_len

        # aft limit: keep the pocket clear of the hinge line. The aft-most
        # feature is whichever reaches further from the pocket centre: the
        # ear tip, or the boss patch's aft end wall (the shaft sits aft of
        # centre and the 25 mm swing slot extends past it - at 15 mm the ears
        # always won and the half-ear-span shorthand was silently safe).
        x_aft_cap = x_hinge - te_margin
        aft_reach = max(
            0.5 * sv["ear_span_mm"] + cl,
            (0.5 * sv["body_len_mm"] - sv["spline_dx_mm"])       # the shaft
            + 0.5 * sv["hub_d_mm"] + cl + HORN_SWING_ROOM_MM,    # boss end
            0.5 * sv["body_len_mm"] + cl
            + float(sv.get("lead_stub_mm", 0.0)) + cl)           # stub end
        xc_hi = min(xc_aft, float(wing.xc_at(sgn * f, x_aft_cap - aft_reach)))
        xc = xc_hi
        while xc >= xc_min - 1e-9:
            x_c = le_x + xc * chord
            x_shaft = x_c + (0.5 * sv["body_len_mm"] - sv["spline_dx_mm"])
            regions = _servo_regions(sv, cl, x_c, y_in, out_dir, x_shaft)
            # datum plane = the servo's own lower face
            z_datum = max(_sample(wing, r)[0]
                          for r in regions if r.on_floor)
            need_roof = z_datum + max(r.need_up for r in regions)
            fits = True
            margin = 1e18
            for r in regions:
                _, cmin = _sample(wing, r)
                top = z_datum + r.need_up
                margin = min(margin, cmin - (top + wall))
                if cmin < top + wall + ROOF_SAFETY_MM:
                    fits = False
            usable = (sv["body_wid_mm"] + cl + wall + ROOF_SAFETY_MM
                      + margin) - (cl + wall + ROOF_SAFETY_MM)
            if usable > deepest[0]:
                deepest = (usable, f, xc)
            if fits:
                best = dict(f=f, xc=xc, x_c=x_c, x_shaft=x_shaft, y_in=y_in,
                            y_mid=y_mid, z_datum=z_datum, regions=regions,
                            chord=chord, le_x=le_x, roof=need_roof,
                            margin=margin)
                break
            xc -= 0.01
        if best is not None:
            break
        f -= step

    if best is None:
        return ServoBay(
            ok=False, y_frac_requested=sgn * f_req,
            reason=(f"a {sv['body_wid_mm']:.1f} mm deep servo plus "
                    f"{cl:.2f} mm clearance and a {wall:.1f} mm roof needs "
                    f"{sv['body_wid_mm'] + cl + wall:.1f} mm of section, and "
                    f"no station between {f_min:.2f} and {f_req:.2f} semi-span "
                    f"has it forward of the hinge line. The best is "
                    f"{deepest[0]:.1f} mm at {deepest[1]:.2f} semi-span, "
                    f"{deepest[2]:.2f} chord - fit a servo no more than "
                    f"{deepest[0]:.1f} mm across its thin axis (the 5 g class "
                    f"is about 8 mm) or deepen the section. The inboard end of "
                    f"that window is the control surface's own root: a servo "
                    f"further inboard than the surface it drives cannot reach "
                    f"the horn with a straight pushrod"),
            dims={"deepest_mm": deepest[0], "deepest_f": deepest[1],
                  "deepest_xc": deepest[2]},
            warnings=["servo bay refused: section too shallow everywhere"])

    f = float(best["f"])
    x_c = float(best["x_c"])
    x_shaft = float(best["x_shaft"])
    y_in = float(best["y_in"])
    z_datum = float(best["z_datum"])
    regions: list[_Region] = best["regions"]

    moved = abs(f - f_req) > 1e-6
    if moved:
        notes.append(
            f"section too shallow at {f_req:.3f} semi-span; servo walked "
            f"inboard to {f:.3f} ({abs(f_req - f) * half:.1f} mm)")

    # ---- build the cutter -------------------------------------------------
    z_bot = z_datum - 40.0                      # well clear below the skin
    parts: list[Solid] = []
    roofs: dict[str, float] = {}
    for r in regions:
        z_top = z_datum + r.need_up
        roofs[r.name] = z_top
        parts.append(_box(r.x0, r.x1, r.y0, r.y1, z_bot, z_top))

    z_roof = roofs["case"]

    # ---- cable exit and stub ---------------------------------------------
    # The lead does NOT leave from the middle of the servo. It leaves the case
    # through one short END face, low down - and on the SG90/FS90 case that end
    # is the one the output shaft is offset toward, not the far one. The shaft
    # here is aft of centre, so the lead comes out at the AFT-INBOARD corner of
    # the pocket, and the run starts there. Putting the mouth on the pocket's
    # centreline made the wire double back inside the pocket before it could
    # enter the channel.
    #
    # This is the one number on the part that no manufacturer draws: FEETECH,
    # TowerPro and AUS Electronics all omit the lead from their dimensioned
    # drawings, so the face and height come from calibrated measurement of
    # manufacturer product photography (Adafruit 169 / 2442, TowerPro SG92R),
    # and clones do move it. That is why the stub below is open to the lower
    # surface rather than a tunnel: the lead can leave from anywhere along the
    # pocket floor and still lie in the run.
    # The rectangular stub slot is GONE (builder feedback, round 4: "0 sharp
    # corners or edges leading up to it"): the round wire pipe now drives its
    # own smoothly trumpeted mouth deep into the pocket cavity, so a boxy
    # side-channel would only add the exact corners the builder is removing.
    # `cable_stub_mm` survives as an override for anyone who wants the old
    # open slot back; `cable_exit` still marks where the pipe should pick the
    # lead up (low, at the case's lead end).
    stub = float(p.get("cable_stub_mm", 0.0))
    cw = float(p.get("cable_w_mm", 8.5))
    ch = float(p.get("cable_h_mm", 8.5))
    # ON the grommet, not merely near it (builder's decision, v2: the pipe
    # must start "at the edge of the servo where the wire to control it is
    # ... directly starting there so that the wire can feed through that hole
    # rather than twisting it back along the bottom of the servo"). The pipe
    # centreline stands at the middle of the measured lead grommet - just
    # PAST the case's shaft-side end face, inside the stub relief the pocket
    # already carves for it - so the trumpeted mouth opens around the grommet
    # itself and the wire runs straight off it into the bore. The old default
    # tucked the mouth half a channel-width INSIDE the end face, a ~4 mm
    # fold-back the builder had to make under the case on the printed part.
    grommet = float(sv.get("lead_stub_mm", 0.0))
    x_lead_default = x_c + 0.5 * float(sv["body_len_mm"]) + cl + 0.5 * grommet
    x_lead = float(p.get("cable_x_mm", x_lead_default))
    y_case_in = y_in - out_dir * cl
    y_stub = y_case_in - out_dir * stub
    z_c0 = z_datum + 0.8                        # nominal channel floor
    z_c1 = z_c0 + ch
    if stub > 0.0:
        # Open to the lower surface, like the pocket it leaves. A JR plug is
        # ~8 x 6 mm and will not thread a 5.6 x 4.2 tunnel, and the X8's own
        # manual has the builder widening exactly this slot by hand for that
        # reason; an open channel with the lead laid in and covered afterwards
        # is what moulded foam wings actually do.
        parts.append(_box(x_lead - 0.5 * cw, x_lead + 0.5 * cw,
                          min(y_case_in, y_stub), max(y_case_in, y_stub),
                          z_bot, z_c1))
    cable_exit = Vector(x_lead, y_stub, 0.5 * (z_c0 + z_c1))
    cable_dir = Vector(0.0, -out_dir, 0.0)

    # ---- optional screw pilots into the roof ------------------------------
    _, crown_over_case = _sample(wing, regions[0])
    roof_thk = crown_over_case - z_roof
    screws = bool(p.get("screw_bosses", True))
    if screws and roof_thk >= MIN_SCREW_ROOF_MM:
        pitch = float(sv["ear_hole_pitch_mm"])
        y_ear = 0.5 * (regions[1].y0 + regions[1].y1)
        depth = min(roof_thk - 1.0, 6.0)
        for dx in (-0.5 * pitch, 0.5 * pitch):
            parts.append(Solid.makeCylinder(
                0.85, depth, Vector(x_c + dx, y_ear, z_roof - 0.2),
                Vector(0, 0, 1)))
        notes.append(
            f"M2 pilot holes (1.7 mm) drilled {depth:.1f} mm up into the "
            f"{roof_thk:.1f} mm roof at the {pitch:.0f} mm ear pitch - the "
            f"servo can be screwed up against the roof")
    else:
        notes.append(
            f"no screw pilots: only {roof_thk:.1f} mm of roof above the "
            f"pocket (need {MIN_SCREW_ROOF_MM:.1f}); bed the servo in with "
            f"silicone or double-sided tape, which is foam-wing practice "
            f"anyway")

    cutter = _fuse(parts)

    # ---- the output, in world coordinates ---------------------------------
    z_shaft = z_datum + 0.5 * sv["body_wid_mm"]
    y_arm = y_in + out_dir * float(sv["arm_z_mm"])
    y_spline = y_in + out_dir * float(sv["total_h_mm"])
    spline = Vector(x_shaft, y_spline, z_shaft)
    arm_hole = Vector(x_shaft, y_arm, z_shaft - arm_r)   # arm hanging straight down

    # How far the arm can swing before its tip climbs back above the keel line
    # and into wing material. The pocket no longer reserves that volume: the
    # arm is fitted after the servo drops in and works in free air below the
    # skin. Measured from straight down, so the usable travel either side of a
    # neutral clocked to `arm_neutral` is free_deg -/+ arm_neutral.
    _c = (z_shaft - z_datum) / max(arm_r, 1e-9)
    free_deg = math.degrees(math.acos(_clamp(_c, -1.0, 1.0)))
    free_aft = free_deg - arm_neutral
    free_fwd = free_deg + arm_neutral

    dims = {
        "arm_reach_aft_mm": arm_reach[0],
        "arm_reach_fwd_mm": arm_reach[1],
        "arm_neutral_deg": arm_neutral,
        "arm_travel_deg": arm_travel,
        "arm_free_deg": free_deg,
        "arm_free_aft_deg": free_aft,
        "arm_free_fwd_deg": free_fwd,
        "pocket_x_mm": (max(r.x1 for r in regions)
                        - min(r.x0 for r in regions)),
        "pocket_y_mm": max(r.y1 for r in regions) - min(r.y0 for r in regions),
        "pocket_ear_x_mm": regions[1].x1 - regions[1].x0,
        "pocket_depth_mm": z_roof - z_datum,
        "pocket_case_x_mm": regions[0].x1 - regions[0].x0,
        "pocket_case_y_mm": regions[0].y1 - regions[0].y0,
        # the boss/spline patch is deliberately shallower than the rest, and
        # being invisible here is how it went 1.65 mm under-cut for so long
        # (the servo bottomed out on it) - report what it was actually cut to
        "pocket_boss_depth_mm": next((r.need_up for r in regions
                                      if r.name == "boss"), None),
        "servo_x_mm": float(sv["body_len_mm"]),
        "servo_y_mm": float(sv["total_h_mm"]),
        "servo_z_mm": float(sv["body_wid_mm"]),
        "ear_span_mm": float(sv["ear_span_mm"]),
        "clearance_mm": cl,
        "roof_mm": roof_thk,
        "roof_wall_asked_mm": wall,
        "crown_margin_mm": float(best["margin"]),
        "section_thk_mm": crown_over_case - z_datum,
        "z_datum": z_datum,
        "z_roof": z_roof,
        "x_centre": x_c,
        "x_lead_mm": x_lead,
        "lead_off_centre_mm": x_lead - x_c,
        "cable_w_mm": cw,
        "cable_h_mm": ch,
        "xc_centre": float(best["xc"]),
        "x_shaft": x_shaft,
        "z_shaft": z_shaft,
        "arm_r_mm": arm_r,
        "arm_below_keel_mm": z_datum - (z_shaft - arm_r),
        "pushrod_reach_mm": abs(x_hinge - x_shaft),
        "y_mm": float(best["y_mid"]),
        "y_spline_mm": y_spline,
    }

    if dims["arm_below_keel_mm"] < 0.5:
        warnings.append(
            "the servo arm barely reaches below the skin - use a longer arm "
            "hole or a deeper horn")

    # The pocket holds the servo, not the arm's swept volume. Say how much
    # swing that leaves, and only complain if it is less than the aircraft
    # actually needs - an X5 flies on +/-20 deg of elevon, so at the 1:1 ratio
    # this linkage is built for, +/-20 deg of servo is the real requirement.
    need = float(p.get("arm_need_deg", 20.0))
    notes.append(
        f"no arm well is cut: the arm is fitted after the servo drops in and "
        f"swings below the skin in free air. It clears the keel line to "
        f"{free_deg:.0f} deg either side of straight down, which is "
        f"{min(free_aft, free_fwd):.0f} deg of travel from a neutral clocked "
        f"{arm_neutral:.0f} deg aft to match the horn.")
    if min(free_aft, free_fwd) < need:
        warnings.append(
            f"the arm has only {min(free_aft, free_fwd):.0f} deg of free swing "
            f"before it touches the wing, against the {need:.0f} deg this "
            f"linkage needs - move the horn hole closer to the hinge plane to "
            f"cut the rake, or use a shorter arm hole")

    return ServoBay(
        ok=True, cutter=cutter, retainers=[],
        origin=Vector(x_c, y_in, z_datum),
        axes={"x": Vector(1, 0, 0), "y": Vector(0, out_dir, 0),
              "z": Vector(0, 0, 1)},
        spline=spline, spline_axis=Vector(0.0, out_dir, 0.0),
        arm_hole=arm_hole, arm_r_mm=arm_r,
        cable_exit=cable_exit, cable_dir=cable_dir,
        y_frac=sgn * f, y_frac_requested=sgn * f_req, moved_inboard=moved,
        dims=dims, notes=notes, warnings=warnings)


# ---------------------------------------------------------------------------
# The control horn
# ---------------------------------------------------------------------------

def _horn_frame(p_in: Vector, p_out: Vector, station_frac: float):
    """(origin on the hinge axis, axis unit u, aft unit, down unit)."""
    d = p_out.sub(p_in)
    L = d.Length
    u = _unit(d)
    aft = Vector(-u.y, u.x, 0.0)
    if aft.Length < 1e-9:
        aft = Vector(1.0, 0.0, 0.0)
    aft = _unit(aft)
    if aft.x < 0.0:
        aft = aft.multiply(-1.0)
    o = p_in.add(u.multiply(_clamp(station_frac, 0.0, 1.0) * L))
    return o, u, aft, Vector(0.0, 0.0, -1.0), L


def _aft_reach(surface: Solid, o: Vector, aft: Vector) -> float:
    """How far aft of the hinge axis the surface's bounding box reaches."""
    try:
        bb = surface.BoundingBox()
    except Exception:
        return 0.0
    best = 0.0
    for x in (bb.xmin, bb.xmax):
        for y in (bb.ymin, bb.ymax):
            for z in (bb.zmin, bb.zmax):
                best = max(best, Vector(x, y, z).sub(o).dot(aft))
    return best


def _blade(o: Vector, u: Vector, aft: Vector, down: Vector,
           poly: list[tuple[float, float]], t: float) -> Solid:
    """Extrude a profile given in (aft, down) coordinates along the hinge
    axis, thickness `t`, centred on the station."""
    base = o.sub(u.multiply(0.5 * t))
    pts = [base.add(aft.multiply(s)).add(down.multiply(h)) for s, h in poly]
    wire = Wire.makePolygon(pts, close=True)
    return Solid.extrudeLinear(wire, [], u.multiply(t))


def _inside(solid: Solid, v: Vector) -> bool:
    """Point classification: is `v` strictly inside the material?

    Strictly: ON does not count. The classifier ray-casts, and along the
    razor-thin trailing edge a near-tangent ray can misreport - a cluster of
    phantom hits BEYOND the lip once formed a fake skin band at h=+5 that
    dragged the horn root 7 mm below the real surface (measured on the pytest
    swept fixture; the blade detached and the fuse failed). Callers that care
    confirm a hit with a second probe as well.
    """
    c = BRepClass3d_SolidClassifier(solid.wrapped)
    c.Perform(gp_Pnt(v.x, v.y, v.z), 1e-6)
    return c.State() == TopAbs_IN


def _local_reach(surface: Solid, o: Vector, aft: Vector, down: Vector,
                 half_t: float, s_max: float) -> float:
    """How far aft of the hinge axis the surface reaches AT THIS STATION.

    The bounding box (`_aft_reach`) includes the whole elevon, so on a swept
    wing it reports the aft-most corner of a station the horn is not at - park
    the base off that number and it overhangs the real trailing edge (worth
    30 mm on the 1.2 m wing). The walk goes BACKWARD, from beyond the box
    toward the hinge: past the lip there is guaranteed to be nothing, so the
    first material found IS the trailing edge - where a forward walk starting
    at the hinge can die instantly on a station that happens to sit on a
    hinge-knuckle bore (measured: the 1.2 m wing's horn station lands on the
    20% knuckle, whose pin clearance has no material at all near the axis,
    and the forward walk reported reach 0).
    """
    def band_hit(s: float) -> float | None:
        # expand from the axis plane outward; near the lip the thin band
        # hugs the local camber, within a few mm of the hinge-axis plane.
        # A hit only counts if a NEIGHBOURING probe confirms it - a lone IN
        # near the razor edge is as likely a classifier misfire as material.
        for dh in np.arange(0.0, half_t + 6.0 + 1e-6, 0.5):
            for h in ((dh,) if dh == 0.0 else (dh, -dh)):
                p = o.add(aft.multiply(float(s))).add(down.multiply(float(h)))
                if not _inside(surface, p):
                    continue
                for dh2, ds2 in ((0.3, 0.0), (-0.3, 0.0), (0.0, -0.7)):
                    q = (o.add(aft.multiply(float(s) + ds2))
                          .add(down.multiply(float(h) + dh2)))
                    if _inside(surface, q):
                        return float(h)
        return None

    first_in = None
    for s in np.arange(max(s_max, 8.0), 1.9, -2.0):
        if band_hit(float(s)) is not None:
            first_in = float(s)
            break
    if first_in is None:
        return 0.0
    lo, hi = first_in, first_in + 2.0
    for _ in range(5):
        mid = 0.5 * (lo + hi)
        if band_hit(mid) is not None:
            lo = mid
        else:
            hi = mid
    return lo


def _skin_band(surface: Solid, o: Vector, aft: Vector, down: Vector,
               s: float, half_t: float) -> tuple[float, float] | None:
    """(h_top, h_bot) of the material band at aft offset `s`, in DOWN mm.

    h_top is the crown side (smallest h, may be negative on a reflexed
    trailing edge, which curls UP past the hinge-axis plane), h_bot the keel.
    None if no material is found.
    """
    step = 0.35
    hs = np.arange(-(half_t + 6.0), half_t + 8.0 + 1e-6, step)
    states = [_inside(surface, o.add(aft.multiply(float(s)))
                               .add(down.multiply(float(h)))) for h in hs]
    best: tuple[float, float] | None = None
    run_start = None
    for h, st in zip(list(hs) + [hs[-1] + step], states + [False]):
        if st and run_start is None:
            run_start = h
        elif not st and run_start is not None:
            # runs shorter than 3 samples (~1 mm) are ignored: a couple of
            # phantom classifier hits must not be mistaken for skin
            if h - run_start >= 3 * step - 1e-9 and (
                    best is None or (h - run_start) > (best[1] - best[0])):
                best = (float(run_start), float(h - step))
            run_start = None
    return best


def _corner_fillet(A: tuple[float, float], P: tuple[float, float],
                   B: tuple[float, float], r: float):
    """Fillet the corner at P between rays P->A and P->B.

    Returns (T1, mid, T2): tangent point on PA, arc midpoint, tangent point
    on PB - or None when the corner is too tight for `r`."""
    ax, ay = A[0] - P[0], A[1] - P[1]
    bx, by = B[0] - P[0], B[1] - P[1]
    la, lb = math.hypot(ax, ay), math.hypot(bx, by)
    if la < 1e-9 or lb < 1e-9:
        return None
    ax, ay, bx, by = ax / la, ay / la, bx / lb, by / lb
    cosang = _clamp(ax * bx + ay * by, -0.999, 0.999)
    ang = math.acos(cosang)
    t_d = r / math.tan(0.5 * ang)
    if t_d > 0.45 * min(la, lb):        # shrink the fillet, keep the corner
        t_d = 0.45 * min(la, lb)
        r = t_d * math.tan(0.5 * ang)
    bisx, bisy = ax + bx, ay + by
    lbis = math.hypot(bisx, bisy)
    if lbis < 1e-9:
        return None
    bisx, bisy = bisx / lbis, bisy / lbis
    cx = P[0] + bisx * r / math.sin(0.5 * ang)
    cy = P[1] + bisy * r / math.sin(0.5 * ang)
    T1 = (P[0] + ax * t_d, P[1] + ay * t_d)
    T2 = (P[0] + bx * t_d, P[1] + by * t_d)
    mx, my = P[0] - cx, P[1] - cy
    lm = math.hypot(mx, my)
    mid = (cx + mx / lm * r, cy + my / lm * r)
    return T1, mid, T2


def _tangent_point(E: tuple[float, float], c: tuple[float, float], r: float,
                   side: float) -> tuple[float, float] | None:
    """Tangent point on circle (c, r) for the tangent line through external
    point E; `side` (+1/-1) picks which of the two tangents."""
    dx, dy = c[0] - E[0], c[1] - E[1]
    d = math.hypot(dx, dy)
    if d <= r + 1e-6:
        return None
    th = math.acos(r / d)               # angle at c between (c->E) and (c->T)
    ex, ey = -dx / d, -dy / d           # unit c -> E
    ca, sa = math.cos(side * th), math.sin(side * th)
    tx, ty = ex * ca - ey * sa, ex * sa + ey * ca
    return (c[0] + tx * r, c[1] + ty * r)


def _horn_blade(o: Vector, u: Vector, aft: Vector, down: Vector, *,
                base_start: float, base_end: float, h_root: float,
                hole_c: tuple[float, float], tip_r: float,
                corner_r: float, t: float, bevel: float
                ) -> tuple[Solid, bool]:
    """The rounded-triangle blade the builder asked for.

    Base along the surface underside from `base_start` to `base_end` (aft mm),
    tip a full circular lobe of radius `tip_r` centred EXACTLY on the pushrod
    hole - so the rim around the bore is `tip_r - bore_r` everywhere by
    construction, not by hoping a fillet lands right. The two flanks are the
    tangent lines from the base corners to that lobe (the shape is the convex
    hull of the base segment and the tip circle), the base corners carry
    `corner_r` fillets, and both faces get a `bevel` chamfer.

    Returns (solid, beveled) - the chamfer is cosmetic, so a failed chamfer
    falls back to the sharp-edged blade rather than failing the horn.
    """
    A = (base_start, h_root)
    B = (base_end, h_root)
    c = hole_c

    # flank tangent points: each corner has two tangents to the lobe; the
    # hull flank is the OUTER one - forward-most for the forward corner,
    # aft-most for the aft corner
    ta_c = [_tangent_point(A, c, tip_r, s) for s in (+1.0, -1.0)]
    tb_c = [_tangent_point(B, c, tip_r, s) for s in (+1.0, -1.0)]
    TA = min([t for t in ta_c if t], key=lambda q: q[0], default=None)
    TB = max([t for t in tb_c if t], key=lambda q: q[0], default=None)
    if TA is None or TB is None:
        # hole lobe swallows a base corner - degenerate; fall back to a plain
        # triangle blade via the polygon extruder
        poly = [(base_start, h_root), (base_end, h_root),
                (c[0] + tip_r, c[1] + tip_r), (c[0] - tip_r, c[1] + tip_r)]
        return _blade(o, u, aft, down, poly, t), False

    # base-corner fillets (corner between the base line and each flank)
    fA = _corner_fillet(B, A, TA, corner_r)
    fB = _corner_fillet(A, B, TB, corner_r)

    # tip arc midpoint: on the lobe, on the far side from the base
    mx, my = c[0] - 0.5 * (A[0] + B[0]), c[1] - h_root
    lm = math.hypot(mx, my)
    tip_mid = (c[0] + mx / lm * tip_r, c[1] + my / lm * tip_r)

    base_pt = o.sub(u.multiply(0.5 * t))

    def P(q: tuple[float, float]) -> Vector:
        return base_pt.add(aft.multiply(q[0])).add(down.multiply(q[1]))

    edges: list[Edge] = []

    def line(q1, q2):
        if math.hypot(q2[0] - q1[0], q2[1] - q1[1]) > 1e-6:
            edges.append(Edge.makeLine(P(q1), P(q2)))

    def arc(q1, qm, q2):
        edges.append(Edge.makeThreePointArc(P(q1), P(qm), P(q2)))

    # walk the outline: A-corner arc, flank down, tip lobe, flank up,
    # B-corner arc, base back to A
    a_end = A if fA is None else fA[0]      # on the base line
    a_flank = A if fA is None else fA[2]    # on the flank
    b_end = B if fB is None else fB[0]
    b_flank = B if fB is None else fB[2]
    if fA is not None:
        arc(fA[0], fA[1], fA[2])
    line(a_flank, TA)
    arc(TA, tip_mid, TB)
    line(TB, b_flank)
    if fB is not None:
        arc(fB[2], fB[1], fB[0])
    line(b_end, a_end)

    wire = Wire.assembleEdges(edges)
    blade = Solid.extrudeLinear(wire, [], u.multiply(t))

    if bevel > 1e-6:
        try:
            face_edges = []
            for f in blade.Faces():
                n = f.normalAt()
                if abs(n.dot(u)) > 0.99:
                    face_edges.extend(f.Edges())
            if face_edges:
                chamfered = blade.chamfer(bevel, None, face_edges)
                got = _one_valid_solid(chamfered)
                if got is not None:
                    return got, True
        except Exception:
            pass
    return blade, False


def _keyhole(o: Vector, u: Vector, aft: Vector, down: Vector,
             a_h: float, h: float, d_run: float, d_feed: float,
             off: float) -> Solid:
    """The cutter for one pushrod hole, bored along the hinge axis.

    With no feed lobe (`off` <= 0) this is ONE plain cylinder - the current
    default, per the builder's own spec of a single clean 2.5 mm circle. The
    keyhole form (a `d_feed` lobe offset radially outboard of a `d_run` seat,
    waist narrower than the wire) is kept for callers who want a captive
    running fit that still threads a hand-bent Z.
    """
    def bore(c: Vector, r: float) -> Solid:
        return Solid.makeCylinder(r, 20.0, c.sub(u.multiply(10.0)), u)

    seat_c = o.add(aft.multiply(a_h)).add(down.multiply(h))
    if off <= 0.0 or d_feed <= d_run:
        return bore(seat_c, 0.5 * d_run)
    feed_c = o.add(aft.multiply(a_h)).add(down.multiply(h + off))
    cut = bore(seat_c, 0.5 * d_run).fuse(bore(feed_c, 0.5 * d_feed))
    return _one_valid_solid(cut) or cut


def control_horn(surface: Solid, *, hinge_p_in: Vector, hinge_p_out: Vector,
                 station_frac: float, thickness: float,
                 align_world_y: float | None = None,
                 params: dict | None = None) -> tuple[Solid, dict]:
    """Fuse a pushrod horn onto the UNDERSIDE of a control surface.

    The horn is part of the surface - it prints with it, in one piece, with no
    screws and nothing to glue. It stands DOWN from the underside just aft of
    the hinge line and carries `n_holes` pushrod holes stacked along the blade
    so the builder can trade throw against precision at the field.

    Parameters
    ----------
    surface
        The control-surface solid (already cut free and hinged).
    hinge_p_in, hinge_p_out
        The hinge line's inboard and outboard ends, world mm - exactly what
        `geometry._elevon_hinge_line` returns.
    station_frac
        Where along that line the horn goes, 0 at `hinge_p_in`. Ignored when
        `align_world_y` is given.
    align_world_y
        World y of the SERVO ARM's swing plane. The pushrod is a straight piece
        of wire, so the horn has to stand in that same plane: put the horn at a
        station the arm is not at and the rod runs diagonally, which converts
        part of every command into a side load on the servo's output bushing
        and makes up-throw and down-throw unequal. Given, the horn is placed at
        whatever station puts its hole line on this plane, and the residual is
        reported as ``info["align_residual_mm"]``.
    thickness
        Local section thickness at that station, mm. Sets how far below the
        skin the blade has to reach before its holes clear the wing, and how
        far the root is set back.
    params
        Overrides for :data:`HORN_DEFAULTS`, plus:
          ``clearance_against`` (Solid|None)
              The WING panel. If given, the horn is rotated about the hinge
              axis through +/- ``max_deflect_deg`` and intersected with it; the
              worst interference volume is reported in ``info["foul"]``.
          ``n_check`` (13) - rotation samples for that sweep.

    Returns
    -------
    (surface_with_horn_fused, info)
    """
    p = dict(HORN_DEFAULTS)
    p.update(params or {})

    # Keep the blade off the surface's own end faces. The elevon is freed by
    # two cut planes and then trimmed back off them for running clearance, so
    # a horn parked at station_frac 0.02 can hang over an edge that is only a
    # couple of millimetres away.
    d_line = hinge_p_out.sub(hinge_p_in)
    L_line = max(d_line.Length, 1e-6)
    keep = (0.5 * float(p["gusset_t_mm"]) + 3.0) / L_line
    sf_req = float(station_frac)
    align_note = None
    if align_world_y is not None:
        # Put the horn in the servo arm's plane. The blade is extruded along
        # the hinge axis and the holes are bored along it, so the hole line's
        # world y is just the station's - solve the station directly.
        dy = d_line.y
        if abs(dy) > 1e-6:
            sf_req = (float(align_world_y) - hinge_p_in.y) / dy
        else:
            align_note = ("hinge line has no spanwise run; cannot align the "
                          "horn to the servo arm by station")
    sf = _clamp(sf_req, keep, 1.0 - keep)

    o, u, aft, down, L = _horn_frame(hinge_p_in, hinge_p_out, sf)
    half_t = 0.5 * float(thickness)

    # THE BLADE STANDS IN THE PUSHROD'S OWN PLANE (builder's decision):
    # extruded along WORLD y and bored along world y, so its faces are
    # perpendicular to the servo arm's swing plane instead of following the
    # hinge sweep - "make it straight relative to the wing, not at the same
    # sweep/hinge angle". The LEVER geometry (setback, rake, radius about
    # the hinge) is still computed perpendicular to the true hinge axis and
    # converted to chordwise mm with 1/cos(sweep); the world y of the hole
    # line is still exactly the station's, so the arm alignment is untouched.
    u_b = Vector(0.0, 1.0 if u.y >= 0.0 else -1.0, 0.0)
    aft_b = Vector(1.0, 0.0, 0.0)
    cosL = max(abs(aft.x), 0.5)      # aft is unit; aft.x = cos(hinge sweep)
    sx = 1.0 / cosL                  # perpendicular mm -> chordwise mm

    # Forward safety floor. A root point at PERPENDICULAR offset `s` and
    # depth `h` swings to s' = s cos(th) - h sin(th) at full up-elevon;
    # s >= h tan(th) keeps it aft of the hinge plane.
    th = math.radians(float(p["max_deflect_deg"]))
    s0 = (max(float(p["setback_mm"]), half_t * math.tan(th) + 0.6)) * sx

    # Material around the hole - this is also the tip lobe's radius, so the
    # rim is `edge_w - bore_r` everywhere around the bore by construction.
    edge = float(p["hole_edge_mm"])
    _d_feed = max(float(p.get("feed_d_mm", 0.0)), float(p["hole_d_mm"]))
    _f_off = (float(p.get("feed_offset_mm", 0.0))
              if _d_feed > float(p["hole_d_mm"]) else 0.0)
    edge_w = max(edge, 0.5 * _d_feed + 1.5)          # tip lobe radius

    # Where the surface actually ENDS at this station - measured by probing
    # the solid along the blade's own (chordwise) direction, because the
    # bounding box reports the aft-most corner of the WHOLE elevon, which on
    # a swept wing belongs to some other station.
    reach_bb = _aft_reach(surface, o, aft_b)
    reach = _local_reach(surface, o, aft_b, down, half_t,
                         s_max=reach_bb + 4.0)
    if reach < s0 + 8.0:                 # probe failed or surface is tiny
        reach = max(0.85 * reach_bb, s0 + 8.0)

    # A 15 mm STUB, AS FAR AFT AS A 15 mm STUB CAN WORK - the builder's
    # round-4 cap ("shorter/stubbier, 15 mm long max") supersedes the pure
    # trailing-edge anchor of round 3, because the two are physically
    # incompatible: the pushrod is near-chordwise, so the moment arm about
    # the hinge is the hole's DEPTH below the axis, and past `rake_max_deg`
    # of rake the four-bar toggles (measured: 50 deg is the sweet spot, 60
    # deg has no up-throw at all). A hole capped at ~11 mm of depth can
    # therefore ride at most depth*tan(50) aft before the linkage dies -
    # about 18 mm on the 610 wing - and the base follows the hole, not the
    # lip. On a surface short enough for the two to meet, the trailing-edge
    # anchor (minus `te_margin_mm` of knife edge) still governs.
    te_m = float(p["te_margin_mm"])
    prot = float(p.get("protrude_max_mm", 15.0))
    rake_cap = math.radians(float(p.get("rake_max_deg", 50.0)))
    h_cap = half_t + max(prot - edge_w, edge_w + 1.0)   # deepest usable hole
    base_end = max(min(reach - te_m,
                       h_cap * math.tan(rake_cap) * sx + edge_w),
                   s0 + 6.0)
    base_len = min(float(p["base_len_mm"]), base_end - s0)
    base_start = base_end - base_len

    # The hole rides at the tip of the lobe, tucked under the aft end of the
    # base - as far from the hinge as the height cap allows. `a_h` is the
    # hole line's CHORDWISE offset; the lever arm about the hinge uses its
    # perpendicular projection `a_perp`.
    a_h = max(float(p["hole_aft_mm"]), base_end - edge_w, s0 + 1.0)
    a_perp = a_h * cosL
    r_des = float(p["r_design_mm"])
    n = int(p["n_holes"])
    pitch = float(p["hole_pitch_mm"])
    # Hole depth: clears the skin, meets the r_design floor, grows so the
    # rake never passes `rake_max_deg`, and stays under the protrusion cap.
    h0 = max(math.sqrt(max(r_des * r_des - a_perp * a_perp, 0.0)),
             half_t + 2.5,
             a_perp / math.tan(rake_cap))
    h0 = min(h0, h_cap)
    depths = [h0 + i * pitch for i in range(n)]
    depths = [d for d in depths if d > half_t + 1.0]
    if not depths:
        depths = [half_t + 3.0]
    h_max = max(depths)
    h_min = min(depths)
    tip = h_max + edge_w

    # Blade top from the MEASURED skin band along the base. A reflexed
    # section's trailing edge curls UP past the hinge-axis plane, so the old
    # fixed top at z=0 would miss the surface entirely back here. The top is
    # pushed to just under the UPPER skin - at the trailing edge the root is
    # effectively a rib filling the section, which is exactly the bond a
    # lever hanging off a 1-2 mm skin needs.
    prof_warnings: list[str] = []
    bands = []
    for s_smp in np.linspace(base_start + 1.0, base_end - 1.0, 4):
        b = _skin_band(surface, o, aft_b, down, float(s_smp), half_t)
        if b is not None:
            bands.append(b)
    if bands:
        # a max() over samples is poisoned by ONE bad band, so throw away any
        # band whose top strays more than 4 mm from the median - the crown
        # line moves gently along a 14 mm base, phantom bands do not
        med = sorted(b[0] for b in bands)[len(bands) // 2]
        bands = [b for b in bands if abs(b[0] - med) <= 4.0] or bands
        h_top = max(b[0] for b in bands)     # deepest crown along the base
        h_bot = min(b[1] for b in bands)     # shallowest keel along the base
        h_root = min(h_top + 0.8, h_bot - 0.3)
        h_root = max(h_root, h_top + 0.4)
    else:
        h_root = 0.0
        prof_warnings.append(
            "could not probe the skin band under the horn base - the root "
            "was left at the hinge-axis plane and may not fully bond on a "
            "reflexed trailing edge")

    blade, beveled = _horn_blade(
        o, u_b, aft_b, down,
        base_start=base_start, base_end=base_end, h_root=h_root,
        hole_c=(a_h, h_max), tip_r=edge_w,
        corner_r=float(p["corner_r_mm"]), t=float(p["blade_t_mm"]),
        bevel=float(p.get("bevel_mm", 0.0)))

    # Flared root, so the bending load goes into the skin over a patch rather
    # than a line. Stops `gusset_h` down the blade.
    gh = float(p["gusset_h_mm"])
    gpoly = [
        (base_start, h_root),
        (base_end, h_root),
        (base_end - 0.35 * base_len, h_root + gh),
        (base_start + 0.25 * base_len, h_root + gh),
    ]
    gusset = _blade(o, u_b, aft_b, down, gpoly, float(p["gusset_t_mm"]))

    horn = heal(blade.fuse(gusset))

    d_run = float(p["hole_d_mm"])
    d_feed = max(float(p.get("feed_d_mm", 0.0)), d_run)
    f_off = float(p.get("feed_offset_mm", 0.0)) if d_feed > d_run else 0.0
    holes: list[Vector] = [o.add(aft_b.multiply(a_h)).add(down.multiply(h))
                           for h in depths]

    # Waist between lobe and seat: what the wire has to snap past.
    waist = 0.0
    if f_off > 1e-9 and f_off < 0.5 * (d_run + d_feed):
        r1, r2 = 0.5 * d_feed, 0.5 * d_run
        d1 = (f_off * f_off + r1 * r1 - r2 * r2) / (2.0 * f_off)
        waist = 2.0 * math.sqrt(max(r1 * r1 - d1 * d1, 0.0))

    info: dict[str, Any] = {
        "station_frac": sf,
        "station_frac_requested": float(station_frac),
        "origin": o,
        "axis": u,
        "aft": aft,
        "hinge_len_mm": L,
        "setback_mm": s0,
        "reach_mm": reach,
        "base_start_mm": base_start,
        "base_end_mm": base_end,
        "te_margin_mm": te_m,
        "protrude_max_mm": prot,
        "tip_below_skin_mm": round(tip - half_t, 1),
        "root_embed_h_mm": h_root,
        "beveled": beveled,
        "hole_aft_mm": a_h,
        "blade_t_mm": float(p["blade_t_mm"]),
        "base_len_mm": base_len,
        "tip_depth_mm": tip,
        "hole_d_mm": d_run,
        "feed_d_mm": d_feed if f_off > 0.0 else None,
        "feed_offset_mm": f_off,
        "keyhole_waist_mm": round(waist, 3),
        "holes": holes,
        "hole_depths_mm": depths,
        "hole_aft_perp_mm": a_perp,
        "blade_sweep_cos": cosL,
        "hole_radii_mm": [math.hypot(a_perp, h) for h in depths],
        "hole_rake_deg": [math.degrees(math.atan2(a_perp, h))
                          for h in depths],
        "depth_below_keel_mm": [h - half_t for h in depths],
        "section_thk_mm": float(thickness),
        "warnings": prof_warnings,
        "notes": [],
    }
    if h_min <= half_t + 1.2:
        info["warnings"].append(
            "innermost hole is barely clear of the skin - the pushrod will "
            "foul the surface")
    if align_note:
        info["warnings"].append(align_note)
    if align_world_y is not None:
        # What the alignment actually cost. The holes all share the station's
        # world y (they are bored along the axis and offset only in aft/down,
        # and `down` has no y), so one residual covers the whole horn.
        info["align_world_y"] = float(align_world_y)
        info["align_residual_mm"] = abs(o.y - float(align_world_y))
        info["align_clamped"] = abs(sf - sf_req) > 1e-9
        if info["align_residual_mm"] > 2.0:
            info["warnings"].append(
                f"horn sits {info['align_residual_mm']:.1f} mm off the servo "
                f"arm's plane - the servo arm's swing does not reach this far "
                f"along the hinge line, so the pushrod will run at an angle")
    if f_off > 0.0:
        info["notes"].append(
            f"pushrod holes are keyholes: thread the bend through the "
            f"{d_feed:.1f} mm lobe, then pull the rod outboard so the wire "
            f"snaps past the {waist:.2f} mm waist into the {d_run:.1f} mm "
            f"seat. Sized for 1.0-1.2 mm (.040-.047 in) music wire.")

    # ---- does it clear the wing through the whole throw? ------------------
    wing_solid = p.get("clearance_against")
    if wing_solid is not None:
        info["foul"] = _sweep_check(horn, o, u, wing_solid,
                                    float(p["max_deflect_deg"]),
                                    int(p.get("n_check", 13)))
        if info["foul"]["max_volume_mm3"] > 1.0:
            info["warnings"].append(
                f"horn fouls the wing by "
                f"{info['foul']['max_volume_mm3']:.1f} mm^3 at "
                f"{info['foul']['worst_deg']:+.1f} deg")

    fused = surface.fuse(horn)
    got = _one_valid_solid(heal(fused))
    if got is None:
        got = _one_valid_solid(fused)
    if got is None:
        # same doctrine as the bay cut: a boolean can fail for reasons that
        # live in cached TShape state, and fresh copies are the cheap retry
        try:
            fused = surface.copy().fuse(horn.copy())
            got = _one_valid_solid(heal(fused)) or _one_valid_solid(fused)
        except Exception:
            got = None
    if got is None:
        info["warnings"].append("horn fuse did not give one valid solid")
        return surface, info

    # ---- bore the keyholes LAST, and PROVE each one is open ---------------
    # The holes used to be bored into the standalone horn before the fuse, and
    # parts turned up in the field with the triangles solid: every step
    # reported success, but nothing ever checked that the finished part still
    # had its holes. Boring the fused solid means no later boolean can lose
    # them, and each cut is kept only if a point classification at the seat
    # and the lobe says the material is actually gone - with a fallback that
    # cuts the two cylinders separately, which is the more robust boolean.
    def _is_open(solid: Solid, v: Vector) -> bool:
        c = BRepClass3d_SolidClassifier(solid.wrapped)
        c.Perform(gp_Pnt(v.x, v.y, v.z), 1e-6)
        return c.State() == TopAbs_OUT

    def _bores(cc: Vector) -> list[Solid]:
        # bored along the BLADE's own axis (world y), matching its faces
        seat = Solid.makeCylinder(0.5 * d_run, 20.0,
                                  cc.sub(u_b.multiply(10.0)), u_b)
        if f_off <= 0.0:
            return [seat]
        fc = cc.add(down.multiply(f_off))
        return [seat, Solid.makeCylinder(0.5 * d_feed, 20.0,
                                         fc.sub(u_b.multiply(10.0)), u_b)]

    holes_cut: list[bool] = []
    for h, c in zip(depths, holes):
        probe = [c] + ([c.add(down.multiply(f_off))] if f_off > 0.0 else [])
        ok = False
        # one combined keyhole cut, then the two cylinders separately
        for cutters in ([_keyhole(o, u_b, aft_b, down, a_h, h, d_run, d_feed,
                                  f_off)], _bores(c)):
            trial = got
            good = True
            for cyl in cutters:
                nxt = _one_valid_solid(trial.cut(cyl))
                if nxt is None:
                    good = False
                    break
                trial = nxt
            if good and all(_is_open(trial, v) for v in probe):
                got = trial
                ok = True
                break
        holes_cut.append(ok)
        if not ok:
            info["warnings"].append(
                f"keyhole at depth {h:.1f} mm could not be bored cleanly - "
                f"drill it by hand at {d_run:.1f} mm")
    info["holes_cut"] = holes_cut

    info["horn"] = horn
    info["horn_volume_mm3"] = float(horn.Volume())
    return got, info


def _sweep_check(horn: Solid, o: Vector, u: Vector, wing: Solid,
                 max_deg: float, n: int) -> dict:
    """Rotate the horn about the hinge axis and intersect it with the wing.

    This is the only honest way to answer "does it foul": the horn is a
    tapered blade on a curved skin next to a coved trailing edge, and the
    closed-form answer is a guess.
    """
    a0 = o
    a1 = o.add(u)
    worst, worst_deg = 0.0, 0.0
    per: list[tuple[float, float]] = []
    for i in range(n):
        deg = -max_deg + 2.0 * max_deg * i / max(n - 1, 1)
        try:
            moved = horn.rotate(a0, a1, deg) if abs(deg) > 1e-9 else horn
            hit = moved.intersect(wing)
            vol = 0.0
            for s in hit.Solids():
                vol += float(s.Volume())
        except Exception:
            vol = float("nan")
        per.append((deg, vol))
        if vol == vol and vol > worst:
            worst, worst_deg = vol, deg
    return {"max_volume_mm3": worst, "worst_deg": worst_deg,
            "samples": per, "range_deg": max_deg}


# ---------------------------------------------------------------------------
# The linkage the builder is going to bend
# ---------------------------------------------------------------------------

def linkage(bay: ServoBay, horn_info: dict, *, hole: int = -1,
            servo_travel_deg: float = 45.0,
            arm_r_mm: float | None = None,
            arm_neutral_deg: float | None = None) -> dict:
    """Solve the planar four-bar the builder will actually get.

    Not "the radii are equal so it is 1:1". The horn's hole has to sit a few
    mm aft of the hinge axis or its root fouls the wing at full up-elevon, so
    the horn link is RAKED at neutral. Rake a link and the four-bar develops
    differential: on the test wing, an arm hanging straight down against a
    horn raked 28 degrees gave -42 / +60 degrees for the same +/-45 degrees of
    servo. The cure is free and is the one thing to get right at the bench:

        clock the servo arm to the SAME rake as the horn, and pick the arm
        hole at the SAME radius as the horn hole.

    The two links are then parallel and equal, the mechanism is a
    parallelogram, and the ratio is exactly 1:1 at every angle regardless of
    how long the pushrod is. That is the geometric content of Model Aviation's
    "choose a servo arm hole that matches the distance of the control arm hole
    to the surface hinge axis".

    With the horn hole pushed AFT toward the trailing edge (for leverage: the
    surface torque is rod force x hole radius), the horn radius exceeds any
    hole on a 9 g arm, so the perfect parallelogram is no longer on offer -
    the mechanism runs geared DOWN (ratio < 1: fewer surface degrees per servo
    degree, more torque per gram of servo). The residual freedom is the arm's
    neutral clock, and by default this function SEARCHES for the clock that
    balances up-throw against down-throw and reports it as
    ``arm_neutral_deg`` - set the arm to that angle at the bench. Override
    `arm_neutral_deg` / `arm_r_mm` to see what a mismatched build gives.

    `hole` indexes `horn_info["holes"]` (-1 = the outermost, longest horn arm,
    least play - which is what RC practice asks for).
    """
    if not bay.ok or not horn_info.get("holes"):
        return {"ok": False, "reason": "no bay or no horn holes"}

    o: Vector = horn_info["origin"]
    u: Vector = horn_info["axis"]
    aft: Vector = horn_info["aft"]
    down = Vector(0.0, 0.0, -1.0)

    hp: Vector = horn_info["holes"][hole]
    # horn hole in the (aft, down) plane of the hinge axis
    rel = hp.sub(o)
    hs = rel.dot(aft)
    hh = rel.dot(down)
    r_horn = math.hypot(hs, hh)
    phi0 = math.atan2(hs, hh)              # 0 = straight down, + = aft

    # servo arm, in the same 2-D frame, measured from the hinge axis origin
    a_rel = bay.spline.sub(o)
    As = a_rel.dot(aft)
    Ah = a_rel.dot(down)
    r_arm = float(arm_r_mm) if arm_r_mm is not None else min(r_horn, 11.0)

    def throws_for(d0: float) -> tuple[float, float, float]:
        """(down, up, rod) for an arm clocked to d0 at neutral."""
        px0 = As + r_arm * math.sin(d0)
        ph0 = Ah + r_arm * math.cos(d0)
        rd = math.hypot(px0 - hs, ph0 - hh)
        up = dn = 0.0
        for k in (-servo_travel_deg, -0.5 * servo_travel_deg,
                  0.5 * servo_travel_deg, servo_travel_deg):
            px = As + r_arm * math.sin(d0 + math.radians(k))
            ph = Ah + r_arm * math.cos(d0 + math.radians(k))
            lo, hi = phi0 - math.radians(75.0), phi0 + math.radians(75.0)
            flo = math.hypot(px - r_horn * math.sin(lo),
                             ph - r_horn * math.cos(lo)) - rd
            fhi = math.hypot(px - r_horn * math.sin(hi),
                             ph - r_horn * math.cos(hi)) - rd
            if flo * fhi > 0:
                continue
            for _ in range(60):
                mid = 0.5 * (lo + hi)
                fm = math.hypot(px - r_horn * math.sin(mid),
                                ph - r_horn * math.cos(mid)) - rd
                if flo * fm <= 0:
                    hi, fhi = mid, fm
                else:
                    lo, flo = mid, fm
            deg = math.degrees(0.5 * (lo + hi) - phi0)
            up = max(up, deg)
            dn = min(dn, deg)
        return dn, up, rd

    if arm_neutral_deg is not None:
        delta0 = math.radians(arm_neutral_deg)
    else:
        # Pick the arm's neutral clock so the throws come out SYMMETRIC and
        # BIG ENOUGH. With the horn hole under the hinge axis the answer is
        # the horn's own rake (the parallelogram); with the hole pushed aft
        # toward the trailing edge no clock is perfect, so search the whole
        # useful circle. The cost wants |up| = |down| first and penalises any
        # shortfall below the ~25 deg an elevon actually flies with - without
        # that term the most 'balanced' clock can be one that jams the rod
        # early and balances two uselessly small throws.
        need = 25.0
        best_d0, best_cost = phi0, None
        for kd in [math.degrees(phi0)] + list(np.arange(-80.0, 80.1, 4.0)):
            d0 = math.radians(kd)
            dn_, up_, _rd = throws_for(d0)
            cost = (abs(up_ + dn_)
                    + 3.0 * max(0.0, need - up_)
                    + 3.0 * max(0.0, need + dn_))
            if best_cost is None or cost < best_cost - 1e-9:
                best_cost, best_d0 = cost, d0
        delta0 = best_d0

    P0 = (As + r_arm * math.sin(delta0), Ah + r_arm * math.cos(delta0))
    H0 = (hs, hh)
    rod = math.hypot(P0[0] - H0[0], P0[1] - H0[1])
    # Spanwise offset between the arm's swing plane and the horn's. The solve
    # below is planar; that is EXACT for the parallelogram case (both ends
    # turn through the same angle, so the 3-D distance between them is
    # constant no matter how far apart the planes are) and a small-angle
    # approximation otherwise.
    skew = abs(bay.arm_point(math.degrees(delta0), r_arm).dot(u)
               - o.add(aft.multiply(hs)).add(down.multiply(hh)).dot(u))

    def horn_pt(phi: float) -> tuple[float, float]:
        return (r_horn * math.sin(phi), r_horn * math.cos(phi))

    def arm_pt(delta: float) -> tuple[float, float]:
        # arm rotates about the spline axis; delta > 0 swings the tip AFT
        return (As + r_arm * math.sin(delta0 + delta),
                Ah + r_arm * math.cos(delta0 + delta))

    def solve(delta: float) -> float | None:
        """surface angle phi (rad, relative to neutral) for servo angle."""
        px, ph = arm_pt(delta)

        def err(phi: float) -> float:
            hx, hz = horn_pt(phi)
            return math.hypot(px - hx, ph - hz) - rod

        lo, hi = phi0 - math.radians(75.0), phi0 + math.radians(75.0)
        flo, fhi = err(lo), err(hi)
        if flo * fhi > 0:
            return None
        for _ in range(80):
            mid = 0.5 * (lo + hi)
            fm = err(mid)
            if flo * fm <= 0:
                hi, fhi = mid, fm
            else:
                lo, flo = mid, fm
        return 0.5 * (lo + hi) - phi0

    up = dn = 0.0
    curve = []
    for k in range(-int(servo_travel_deg), int(servo_travel_deg) + 1):
        got = solve(math.radians(k))
        if got is None:
            continue
        deg = math.degrees(got)
        curve.append((float(k), deg))
        up = max(up, deg)
        dn = min(dn, deg)

    ratio = None
    for k, deg in curve:
        if abs(k - 10.0) < 1e-6:
            ratio = deg / 10.0

    # ---- the 3-D truth ----------------------------------------------------
    # The planar solve above assumes both ends turn in parallel planes. They do
    # not quite: the horn turns about the HINGE axis and the arm turns about the
    # servo's shaft axis, and on a swept wing those are a few degrees apart, so
    # a rod pinned at both ends has to stretch. `pushrod_skew_mm` measures the
    # offset along the hinge axis and therefore reads the sweep as misalignment
    # even when the two holes share a plane exactly. What actually decides
    # whether the linkage binds is how much the END-TO-END DISTANCE varies
    # through the throw: a rigid wire cannot change length, so any variation is
    # taken up as slop, as a bent rod, or as load on the servo bushing.
    def _rot(pt: Vector, axis_o: Vector, axis_u: Vector, deg: float) -> Vector:
        # Rodrigues, about a general axis
        a = math.radians(deg)
        v = pt.sub(axis_o)
        k = _unit(axis_u)
        return (axis_o.add(v.multiply(math.cos(a)))
                .add(k.cross(v).multiply(math.sin(a)))
                .add(k.multiply(k.dot(v) * (1.0 - math.cos(a)))))

    def _sense(pt: Vector, axis_o: Vector, axis_u: Vector) -> float:
        """+1 if a positive rotation about this axis carries `pt` AFT.

        Both 2-D solves above call aft positive, but which way that is about
        the 3-D axis flips between the two wings (the hinge axis points
        outboard, so it reverses, and so does the spline axis). Probing it beats
        a sign convention nobody can check by reading.
        """
        moved = _rot(pt, axis_o, axis_u, 1.0)
        return 1.0 if moved.sub(pt).dot(aft) >= 0.0 else -1.0

    arm0 = bay.arm_point(math.degrees(delta0), r_arm)
    axis_arm = bay.spline_axis
    s_arm = _sense(arm0, bay.spline, axis_arm)
    s_horn = _sense(hp, o, u)
    rod0 = arm0.sub(hp).Length
    stretch = 0.0
    worst_deg = 0.0
    for k, surf_deg in curve:
        a_pt = _rot(arm0, bay.spline, axis_arm, s_arm * k)
        h_pt = _rot(hp, o, u, s_horn * surf_deg)
        d = abs(a_pt.sub(h_pt).Length - rod0)
        if d > stretch:
            stretch, worst_deg = d, k
    return {
        "ok": True,
        "arm_r_mm": r_arm,
        "arm_neutral_deg": math.degrees(delta0),
        "arm_hole": bay.arm_point(math.degrees(delta0), r_arm),
        "horn_r_mm": r_horn,
        "horn_rake_deg": math.degrees(phi0),
        "parallelogram": (abs(r_arm - r_horn) < 1e-6
                          and abs(delta0 - phi0) < 1e-9),
        "pushrod_mm": rod,
        "pushrod_3d_mm": rod0,
        "pushrod_skew_mm": skew,
        "pushrod_skew_deg": math.degrees(math.atan2(skew, rod)),
        # what a rigid wire actually has to absorb, over the whole throw
        "pushrod_stretch_mm": stretch,
        "pushrod_stretch_at_deg": worst_deg,
        "hole_offset_mm": abs(hp.y - arm0.y),   # the alignment that matters
        "ratio_at_neutral": ratio,
        "surface_deg_for_travel": (dn, up),
        "servo_travel_deg": servo_travel_deg,
        "curve": curve,
    }
