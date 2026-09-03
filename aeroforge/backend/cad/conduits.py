"""Internal wire conduits: motor phase leads and elevon servo leads.

WHY THIS EXISTS
---------------
The airframe is one continuous solid loft with a single hollow equipment bay
in the centre body. Everything aft of the bay and everything outboard of it is
SOLID material - which is structurally what you want, but it leaves a builder
with nowhere to put the wiring. Taped to the outside, a motor lead is a 3 mm
trip wire across the whole aft centre section and a servo lead is a spanwise
one across the panel; on a 570 mm flying wing that is a real drag and a real
snag hazard, not a cosmetic one.

This module cuts the two channels a flying wing actually needs:

  1. MOTOR CONDUIT - motor-mount face -> equipment bay, for the three phase
     leads. Chordwise, on (or near) the centreline.
  2. SERVO CONDUIT - elevon servo bay -> equipment bay, one per side, for the
     3-conductor servo lead. Mostly spanwise, through the wing panel.

Nothing here imports `backend.cad.geometry`, `hinges`, `hatch` or `servos`;
`wing` is duck-typed (see :func:`motor_conduit`).


RESEARCH: WHAT ACTUALLY HAS TO FIT
----------------------------------
*Motor phase leads.* A 2204-2814 class outrunner ships with three silicone
leads. Measured stock:

  * 16 AWG silicone, 252 x 0.08 mm tinned strands: OD **3.0 mm +/- 0.1**
    https://bntechgo.com/bntechgo-16-gauge-silicone-wire-spool-brown-100-feet-ultra-flexible-high-temp-200-c-600v-16-awg-silicone-rubber-wire-with-252-strands-of-tinned-copper-wire-stranded-wire-for-model-battery/
  * 18 AWG silicone, 150 x 0.08 mm strands: OD **2.3 mm +/- 0.1**
    https://bntechgo.com/bntechgo-18-gauge-silicone-wire-spool-white-250-feet-ultra-flexible-high-temp-200-c-600v-18-awg-silicone-rubber-wire-with-150-strands-of-tinned-copper-wire-stranded-wire-for-model-battery/

  THREE equal circles of diameter d pack into a circumscribed circle of
  d * (1 + 2/sqrt(3)) = 2.1547 d.  So:
      3 x 18 AWG -> 4.96 mm      3 x 16 AWG -> 6.46 mm

  * 3.5 mm bullet connectors (the standard ESC/motor joint in this class) are
    ~12.5 mm long and are insulated with 5 mm / 3/8" ID heat shrink, giving a
    finished joint OD of about 5.0-5.5 mm.
    https://www.readymaderc.com/products/details/3-5mm-bullet-connectors-1-pair
    https://rchobbyexplosion.com/traxxas-bullet-connectors-male-3-5mm-w-heat-shrink-3342/

  Pulling three factory-terminated leads through means, at the worst section,
  one insulated bullet (5.5 mm) sharing the bore with two bare 16 AWG leads
  (3.0 mm). Smallest enclosing circle for radii 2.75/1.5/1.5 in contact is
  ~4.3 mm radius -> **8.6 mm**, +0.4 mm print clearance -> **9.0 mm**.

  VERDICT ON THE 6 mm SUGGESTION: 6 mm is NOT enough. It will not pass three
  16 AWG leads with connectors (needs 9 mm), and it will not even pass three
  BARE 16 AWG leads (needs 6.5 mm). It is enough only for three bare 18 AWG
  leads (4.96 mm) with the bullets cut off and re-fitted after threading.
  The nominal here is therefore 9.0 mm, degrading through 7.0 (three bare
  16 AWG + slack) and 5.5 (three bare 18 AWG) where the section is too
  shallow, and the tier actually achieved is reported.

*Servo lead (SG90 / generic 9 g).* The user's assumption needs one
correction: the standard RC servo lead IS usually a "ribbon" - but that word
means three SEPARATELY INSULATED round stranded conductors bonded side by
side in a flat strip, not a flat-flex ribbon cable. Twisted 3-core is the
other common form. So "3-conductor cable" is right; "not a ribbon" is not the
way the hobby uses the word.
    https://www.rchelicopterfun.com/rc-servo-connectors.html
    (gauges in use: 26 AWG small, 24 AWG medium, 22/20 AWG large;
     connector rated 3 A sustained.)

  * Conductor OD: **1.2 mm** at 26 AWG (30 x 0.08 mm strands)
    https://www.amazon.com/uxcell-Extension-Connectors-30-Cores-Futaba/dp/B07PK6XBSZ
  * Futaba J / JR housings accept AWG 24-26 with **max wire OD 1.5 mm**
    https://www.scondar.com/wire-to-wire/futaba-j-2-54mm-pitch-connectors/
  * So the flat lead envelope is 3 x 1.5 = **4.5 mm wide x 1.5 mm thick**
    worst case (22-24 AWG), 3.6 x 1.2 for a 9 g servo's 26 AWG lead.

  * THE CONNECTOR IS THE PROBLEM, as suspected. No RC vendor publishes the
    JR/Futaba housing envelope, but it is a 3-circuit 2.54 mm crimp housing -
    the same family as the Molex KK 254, whose 3-way housing 22-01-3037 is
    **8.18 x 4.82 mm** in section:
      https://uk.farnell.com/molex/22-01-3037/housing-female-3way/dp/1462838
    The JR/Futaba shell adds a polarising key and a wider wire entry, so the
    real envelope is about **8.2 mm wide x 6.2 mm tall x ~10 mm long**.
    Passing that needs a ~9.0 x 7.0 mm clear opening.

  MEASURED CONSEQUENCE: on the reference 567 mm swept wing the outer panel is
  10.8 mm deep at max thickness where an elevon servo lives (65% semi-span),
  11.4 mm on the BWB, 10.5 on the plank and 6.5 on the bell. After 1.2 mm of
  skin top and bottom, and after the 45 degree self-supporting roof, a
  connector-passing 9.0 x 7.0 opening needs 11.5 mm of section and does not
  fit on any of them. So:

  **The servo conduit carries the BARE CABLE, not the connector.** The
  builder threads the lead (or a pull string) through, and the JR/Futaba plug
  lives in the equipment bay where there is room for it. The channel is sized
  5.6 x 3.4 mm clear, which swallows a 4.5 x 1.5 mm flat lead plus a pull
  string with room to spare. The module still ATTEMPTS the 9.0 x 7.0
  connector-passing tier first and takes it wherever the section is deep
  enough (short inboard runs, BWB and plank planforms), and reports which
  tier it got.


RESEARCH: PRINTABILITY (Bambu Lab H2D, 0.4 mm nozzle, PLA, tree supports)
-------------------------------------------------------------------------
* Minimum horizontal (through-wall) hole diameter for FDM is quoted at
  **2.0 mm**, versus 1.0 mm for a vertical hole:
  https://www.pollen.am/design_guidelines_holes/
  Small holes are also the ones that close up in practice - Bambu users
  report 0.5-0.75 mm holes filling in about half the time even at half speed:
  https://forum.bambulab.com/t/looking-for-tips-to-print-small-holes-0-5-0-75mm/179015
  Every channel here is >= 4 mm, so diameter is not the limit; the ROOF is.

* A horizontal round hole's crown is an unsupported overhang. Overhangs past
  45 degrees generally want support, and the standard fix for a horizontal
  hole is a TEARDROP / keyhole cross-section which holds a 45 degree roof so
  the printer closes it with no support and no droop:
  https://3dprinterly.com/how-to-3d-print-holes-without-supports-is-it-possible/
  https://makerworld.com/en/models/2085522-teardrop-hole-tests
  Nophead measured the bridge over a plain 6 mm horizontal hole drooping
  ~0.15 mm, and showed the layer-sampling error that also shrinks such holes:
  https://hydraraptor.blogspot.com/2020/07/horiholes_36.html

* Tree supports WILL grow inside an internal cavity if the slicer can see it,
  and they are then impossible to remove. Bambu's own guidance and the
  community answer is to paint a support blocker over the region or drop a
  blocker volume into the hole - i.e. manual work per print:
  https://forum.bambulab.com/t/supports-in-screw-holes/83637
  https://wiki.bambulab.com/en/software/bambu-studio/support
  A self-supporting cross-section removes the need entirely, which is why
  every channel in this module is cut with a 45 degree gabled roof regardless
  of which way the part is finally laid on the plate. It costs nothing if the
  wing is printed span-vertical (the channels are then near-vertical anyway)
  and saves the print if it is laid flat.

The section used is therefore a "gabled teardrop": an elliptical belly of
half-width a and half-depth b, continued by two straight 45 degree flanks
that leave the ellipse exactly where its own tangent reaches 45 degrees (so
the profile is C1 there, no corner for the slicer to overhang on) and meet at
a ridge. Total height is b + sqrt(a^2 + b^2); for a circle (a = b = r) that
is the classic 2.414 r teardrop.


A NOTE ON THE EXISTING SHAFT BORE (not this module's doing)
-----------------------------------------------------------
`geometry._motor_mount` already drills a shaft bore down the nacelle axis -
17.6 mm diameter on the reference wing, running `lead + 0.85 * depth` forward
of the mounting face. Measured on the built solid, that bore's ceiling sits
ABOVE the local nacelle skin between roughly x = 217 and x = 229 mm: on the
centreline there the whole crown is missing, an open slot about 12 mm long
just behind the centre fin. It is present in the airframe with or without any
conduit (verified by comparing vertical crossing columns before and after
cutting - they are identical over that stretch), so it is not caused by, and
cannot be fixed from, this module.

It does matter here for two reasons. First, a motor conduit makes that long
bore unnecessary: with a proper 9 mm channel the shaft bore only has to clear
the motor's shaft boss, ~15 mm, which would close the slot. Second, it means
a skin measurement taken over the aft end of the motor conduit is really
measuring the shaft bore, not the conduit - which is why the verification
numbers for the motor channel are quoted forward of x = 216 mm.


A NOTE ON THE PART SPLIT
------------------------
`build_design_parts` hands the airframe over as `centre_body` + `wing_left` +
`wing_right`, split at the body/panel joint. A servo conduit crosses that
joint, so it comes out as a matching hole on each mating face - which is
exactly what a builder wants: thread the lead through the panel, out of its
root rib, and into the body. It does mean the two halves of the channel must
line up, so `servo_conduit` routes on a chord FRACTION rather than on a
constant x; the joint is at a single spanwise station and both sides evaluate
the same route there.


INVARIANTS THIS MODULE KEEPS
----------------------------
* A conduit never breaks the skin. Every station of every channel is checked
  against the real crown and keel of the local section, at the profile's own
  extreme points, with `wall` (>= 1.2 mm, three 0.42 mm extrusions) of skin
  left top and bottom. Where the section is too shallow the channel SHRINKS
  through its size tiers, and if the floor tier still does not fit the conduit
  is REPORTED AS SKIPPED rather than punched through.
* The route stays forward of the elevon hinge line, so it never crosses the
  hinge/knuckle region or ends up inside a separated elevon.
* The motor conduit stays clear of the mount's bolt circle.
* Cutting is gated on `cut_conduit`, which keeps a cut only if the result is
  ONE valid solid that OCC still meshes at >= 98.5% of its BRep area. That
  area ratio - not `isValid()` - is what catches OCC silently dropping a
  trimmed face, which is how a hole once appeared in the exported skin.
"""

from __future__ import annotations

import math
from typing import Any, Sequence

import numpy as np
from cadquery import Edge, Solid, Vector, Wire

__all__ = [
    "MOTOR_CONDUIT",
    "SERVO_CONDUIT",
    "PRINTABILITY",
    "teardrop_section",
    "teardrop_extents",
    "teardrop_wire",
    "motor_conduit",
    "servo_conduit",
    "straight_conduit",
    "cut_conduit",
    "cut_conduits",
    "tess_ratio",
    "tess_stats",
    "untriangulated_faces",
    "unmeshed_faces",
    "face_meshes_alone",
    "mesh_audit",
    "face_key",
    "Classifier",
    "MeshProbe",
    "point_is_inside",
    "path_vectors",
    "route_is_open",
    "route_connects",
    "skin_audit",
]


def path_vectors(info: dict) -> list[Vector]:
    """The centreline from a conduit's `info` dict, as Vectors."""
    return [Vector(*p) for p in info.get("path_mm", [])]


# ---------------------------------------------------------------------------
# Researched sizing
# ---------------------------------------------------------------------------

#: Skin left standing above and below a channel, mm. 1.2 mm is AeroForge's
#: default `wall_mm` and is three 0.42 mm extrusions from a 0.4 nozzle - the
#: thinnest run that still prints as a solid, non-porous shell.
MIN_SKIN_MM = 1.2

#: Absolute floor on a channel's clear width. Below the 2.0 mm horizontal-hole
#: minimum a channel is not reliably open at all, and below ~4 mm it will not
#: pass anything this aircraft carries, so there is no point cutting it.
MIN_CHANNEL_MM = 4.0

#: Slicers under-cut horizontal holes because they sample each layer at its
#: mid-height (Nophead, "horiholes"). Half a 0.2 mm layer of radius is added
#: back so the AS-PRINTED bore matches the design bore.
LAYER_COMP_MM = 0.10

# Builder's round 7 (2026-08-24): wire ports are OVAL - "it can be oval
# shape ish to make more room for the wires" - width/height ratio applied
# to every straight wire run (12.0 wide for the 8.25 bore). Still one
# straight extrusion of constant cross-section; pushrod guides untouched.
PIPE_W_MM = 12.0
OVAL_W_RATIO = PIPE_W_MM / 8.25


MOTOR_CONDUIT: dict[str, Any] = {
    "what": "three motor phase leads, motor-mount face to equipment bay",
    "section": "gabled teardrop, 45 deg roof, ridge up (+z)",
    # ---- what has to fit -------------------------------------------------
    "wire_od_mm": {"16awg_silicone": 3.0, "18awg_silicone": 2.3},
    "bundle_of_three_mm": {"16awg": 6.46, "18awg": 4.96},   # 2.1547 * d
    "bullet_joint_od_mm": 5.5,        # 3.5 mm bullet in 5 mm ID heat shrink
    "bullet_joint_len_mm": 12.5,
    # ---- the size actually cut --------------------------------------------
    # ONE ROUND PIPE (builder's decision, round 2): every wire-management
    # hole on the aircraft is the same clean circle, 8.25 mm diameter, motor
    # and servo alike - "no fancy shape, just a circle and circular cross
    # section". 8.25 passes three bulleted 16 AWG leads (bullet joint 5.5).
    # No smaller fallback: if the pipe does not fit, the run refuses and
    # names the numbers.
    "tiers_mm": (
        # (clear diameter, name, what it passes)
        (8.25, "pipe",
         "round 8.25 mm pipe - three 16 AWG leads with 3.5 mm bullets"),
    ),
    "nominal_d_mm": 9.0,
    "verdict_on_6mm": (
        "too small: 6 mm passes three bare 18 AWG leads only. Three bare "
        "16 AWG leads circumscribe 6.46 mm and a bullet joint alone is "
        "5.5 mm, so a connectorised 16 AWG motor needs 9.0 mm."
    ),
    "sources": [
        "https://bntechgo.com/bntechgo-16-gauge-silicone-wire-spool-brown-100-feet-ultra-flexible-high-temp-200-c-600v-16-awg-silicone-rubber-wire-with-252-strands-of-tinned-copper-wire-stranded-wire-for-model-battery/",
        "https://bntechgo.com/bntechgo-18-gauge-silicone-wire-spool-white-250-feet-ultra-flexible-high-temp-200-c-600v-18-awg-silicone-rubber-wire-with-150-strands-of-tinned-copper-wire-stranded-wire-for-model-battery/",
        "https://www.readymaderc.com/products/details/3-5mm-bullet-connectors-1-pair",
        "https://rchobbyexplosion.com/traxxas-bullet-connectors-male-3-5mm-w-heat-shrink-3342/",
    ],
}


SERVO_CONDUIT: dict[str, Any] = {
    "what": "one elevon servo lead, servo bay inboard to the equipment bay",
    "section": "gabled teardrop, wider than tall (the wing panel is shallow)",
    # ---- what has to fit -------------------------------------------------
    "cable_construction": (
        "THREE separately insulated stranded conductors bonded side by side "
        "in a flat strip (the hobby calls this a 'ribbon', but it is not a "
        "flat-flex ribbon cable) or twisted into a 3-core. Not a ribbon in "
        "the flat-flex sense."
    ),
    "conductor_awg": (26, 24, 22),
    "conductor_od_mm": {"26awg": 1.2, "max_for_jr_housing": 1.5},
    "cable_envelope_mm": (4.5, 1.5),      # 3 x 1.5 mm side by side, worst case
    "connector": "JR / Futaba J, 3 circuits at 2.54 mm pitch",
    "connector_envelope_mm": (8.2, 6.2, 10.0),   # w x h x l, see module doc
    "connector_passes": False,
    "connector_note": (
        "The channel is sized for the BARE CABLE. A JR/Futaba housing is "
        "~8.2 x 6.2 mm and needs a 9.0 x 7.0 mm opening; after 1.2 mm of "
        "skin top and bottom and a 45 deg self-supporting roof that does not "
        "fit in an outer panel much past 35% semi-span on a typical swept "
        "wing. Thread the bare lead (or a pull string) and leave the plug in "
        "the equipment bay. Where the section IS deep enough the router takes "
        "the connector tier automatically and says so in info['tier']."
    ),
    # ---- the size actually cut --------------------------------------------
    # ONE ROUND PIPE (builder's decision, round 2): the same clean 8.25 mm
    # circle as the motor run - "I want to just see a circular hole looking
    # through the entire servo wire management system". 8.25 passes the bare
    # lead easily and a JR/Futaba plug diagonally (housing 8.2 x 6.2, corner
    # to corner ~10.3 - thread the bare lead or a pull string; the plug still
    # lives in the equipment bay). No smaller fallback: if the pipe does not
    # fit, the run refuses and names the numbers.
    "tiers_mm": (
        #     (clear width, clear height, name, what it passes)
        (8.25, 8.25, "pipe",
         "round 8.25 mm pipe - flat 3-conductor lead + pull string"),
    ),
    "sources": [
        "https://www.rchelicopterfun.com/rc-servo-connectors.html",
        "https://www.scondar.com/wire-to-wire/futaba-j-2-54mm-pitch-connectors/",
        "https://uk.farnell.com/molex/22-01-3037/housing-female-3way/dp/1462838",
        "https://www.amazon.com/uxcell-Extension-Connectors-30-Cores-Futaba/dp/B07PK6XBSZ",
        "https://protosupplies.com/product/servo-motor-micro-sg90/",
    ],
}


PRINTABILITY: dict[str, Any] = {
    "printer": "Bambu Lab H2D, 0.4 mm nozzle, PLA, default profile",
    "min_horizontal_hole_mm": 2.0,
    "min_vertical_hole_mm": 1.0,
    "roof_angle_deg": 45.0,
    "layer_compensation_mm": LAYER_COMP_MM,
    "why_teardrop": (
        "A horizontal round hole's crown is an unsupported overhang; past "
        "45 deg the slicer wants support, and tree supports grown inside a "
        "closed channel cannot be removed. A 45 deg gabled roof closes the "
        "channel with no support anywhere."
    ),
    "sources": [
        "https://www.pollen.am/design_guidelines_holes/",
        "https://3dprinterly.com/how-to-3d-print-holes-without-supports-is-it-possible/",
        "https://hydraraptor.blogspot.com/2020/07/horiholes_36.html",
        "https://makerworld.com/en/models/2085522-teardrop-hole-tests",
        "https://forum.bambulab.com/t/supports-in-screw-holes/83637",
    ],
}


# ---------------------------------------------------------------------------
# small helpers
# ---------------------------------------------------------------------------


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


def _smoothstep(u: float) -> float:
    u = _clamp(u, 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


def _unit(v: Vector) -> Vector:
    n = v.Length
    return Vector(0, 0, 1) if n < 1e-12 else v.multiply(1.0 / n)


# ---------------------------------------------------------------------------
# cross-section
# ---------------------------------------------------------------------------


#: Segments around the rounded belly of a channel profile. 20 puts the chord
#: error at 0.04 mm on a 9 mm bore - under the layer compensation, and
#: invisible in a wire channel - while keeping the spline's pole count (and so
#: the cost of every surface-surface intersection in the boolean) low.
_N_ARC = 20


def teardrop_section(width: float, height: float | None = None, *,
                     roof_deg: float = 45.0,
                     n_arc: int = _N_ARC) -> list[tuple[float, float]]:
    """Self-supporting horizontal-channel profile, as (u, v) points.

    `u` is across the channel, `v` is UP (build direction). The profile is an
    elliptical belly of half-width ``a = width/2`` and half-depth
    ``b = height/2`` (default ``b = a``, i.e. a circle), continued by two
    straight flanks at `roof_deg` from horizontal that leave the ellipse
    exactly where the ellipse's own tangent already has that slope, and meet
    at a ridge on the centreline.

    Leaving at the matching-tangent point is the bit that matters: splice the
    flanks on at the widest point instead and there is a slope discontinuity
    right where the overhang starts, which is the corner the slicer droops
    off. Done this way the whole roof is at or below `roof_deg` everywhere.

    For a circle this reduces to the classic teardrop: ridge at
    ``r * sqrt(2)`` above centre, total height ``2.414 r``.

    Returns a closed CCW point list (first point is not repeated).
    """
    a = max(float(width), 0.2) * 0.5
    b = a if height is None else max(float(height), 0.2) * 0.5
    m = math.tan(math.radians(_clamp(roof_deg, 20.0, 75.0)))   # |dv/du|

    # tangency point on the ellipse where dv/du = -m
    #   ellipse: u = a cos t, v = b sin t ; dv/du = -(b/a) cot t = -m
    t_tan = math.atan2(b, a * m)              # in (0, pi/2)
    u0, v0 = a * math.cos(t_tan), b * math.sin(t_tan)
    ridge = v0 + m * u0

    pts: list[tuple[float, float]] = []
    # belly + flanks of the ellipse: from (+u0, +v0) clockwise round the
    # bottom to (-u0, +v0), i.e. t from t_tan down through -pi/2 to
    # -(pi - t_tan)... walk CCW instead: start at (-u0, v0) -> bottom -> +u0.
    t_start = math.pi - t_tan
    t_end = 2.0 * math.pi + t_tan
    for i in range(n_arc + 1):
        t = t_start + (t_end - t_start) * i / n_arc
        pts.append((a * math.cos(t), b * math.sin(t)))
    # up the right flank, over the ridge, down the left flank
    pts.append((0.0, ridge))
    return pts


def teardrop_extents(width: float, height: float | None = None, *,
                     roof_deg: float = 45.0) -> tuple[float, float, float]:
    """(half_width, drop_below_centre, rise_above_centre) for a profile."""
    a = max(float(width), 0.2) * 0.5
    b = a if height is None else max(float(height), 0.2) * 0.5
    m = math.tan(math.radians(_clamp(roof_deg, 20.0, 75.0)))
    t_tan = math.atan2(b, a * m)
    return a, b, b * math.sin(t_tan) + m * a * math.cos(t_tan)


def teardrop_wire(width: float, height: float | None, origin: Vector,
                  u_dir: Vector, v_dir: Vector, *,
                  roof_deg: float = 45.0, n_arc: int = _N_ARC,
                  spline: bool = True) -> Wire:
    """A :func:`teardrop_section` placed in 3D on the (u_dir, v_dir) plane."""
    pts2 = teardrop_section(width, height, roof_deg=roof_deg, n_arc=n_arc)
    pts3 = [origin + u_dir.multiply(u) + v_dir.multiply(v) for u, v in pts2]
    if spline:
        try:
            # smooth belly, crisp ridge: one spline round the rounded part and
            # two straight flanks, so the ridge stays a real 45 deg corner
            # instead of being rounded off by the interpolation.
            belly = Edge.makeSpline(pts3[:-1])
            right = Edge.makeLine(pts3[-2], pts3[-1])
            left = Edge.makeLine(pts3[-1], pts3[0])
            w = Wire.assembleEdges([belly, right, left])
            if w.IsClosed():
                return w
        except Exception:
            pass
    return Wire.makePolygon(pts3, close=True)


# ---------------------------------------------------------------------------
# sweeping a profile along a path
# ---------------------------------------------------------------------------


def _frames(path: Sequence[Vector],
            up: Vector = Vector(0, 0, 1)) -> list[tuple[Vector, Vector, Vector]]:
    """(tangent, u_dir, v_dir) at every path point.

    `v_dir` is world-up projected perpendicular to the tangent, so the ridge of
    the teardrop always points as close to +z as the path allows - which is
    what makes it self-supporting no matter which way the channel runs.
    """
    n = len(path)
    out = []
    for i in range(n):
        if i == 0:
            t = path[1] - path[0]
        elif i == n - 1:
            t = path[-1] - path[-2]
        else:
            t = path[i + 1] - path[i - 1]
        t = _unit(t)
        v = up - t.multiply(t.dot(up))
        if v.Length < 1e-6:                     # channel runs vertically
            v = Vector(1, 0, 0) - t.multiply(t.dot(Vector(1, 0, 0)))
        v = _unit(v)
        u = _unit(v.cross(t))                   # (u, v, t) right-handed
        out.append((t, u, v))
    return out


def _sweep(path: Sequence[Vector], sizes: Sequence[tuple[float, float]], *,
           roof_deg: float = 45.0, n_arc: int = _N_ARC,
           section: str = "teardrop") -> Solid:
    """Loft a teardrop of `sizes[i]` at every point of `path`.

    THROUGH-SECTIONS, not ruled, and this is a performance decision worth more
    than every other one in the module put together. A ruled loft makes one
    face per profile edge PER SEGMENT: 30 stations x 3 edges = 89 faces per
    conduit, 249 faces of BSpline across three of them, and cutting that from a
    236-face spline airframe took 41.6 s. Through-sections fits ONE B-spline
    surface per profile edge through all the stations - 5 faces per conduit,
    15 in total - and the same cut is a different order of magnitude.

    Nothing is given away for it: the surface still INTERPOLATES every station
    wire, so the geometry is exact at every station the router validated, and
    between them it bulges by 0.045% of volume (microns on a 6 mm channel)
    instead of chording slightly inside. Ruled remains as the fallback for the
    rare path where through-sections will not close.
    """
    frames = _frames(path)

    def build(spline: bool, ruled: bool) -> Solid:
        if section == "circle":
            # A true pipe: one circle per station, lofted. The builder's own
            # words: "equivalent to doing a 'pipe' or 'loft' function in
            # Fusion 360" - no ridge, no flats, a circular cross-section the
            # whole way through.
            wires = [Wire.assembleEdges(
                        [Edge.makeCircle(0.5 * w, p, t)])
                     for p, (t, u, v), (w, h) in zip(path, frames, sizes)]
        elif section == "oval":
            # Builder's round 7: a wider oval bore "to make more room for
            # the wires" - still ONE straight extrusion of constant
            # cross-section. Major axis rides the frame's horizontal u.
            wires = [Wire.assembleEdges(
                        [Edge.makeEllipse(0.5 * w, 0.5 * h, p, t, u)])
                     for p, (t, u, v), (w, h) in zip(path, frames, sizes)]
        else:
            wires = [teardrop_wire(w, h, p, u, v, roof_deg=roof_deg,
                                   n_arc=n_arc, spline=spline)
                     for p, (t, u, v), (w, h) in zip(path, frames, sizes)]
        return Solid.makeLoft(wires, ruled)

    for spline, ruled in ((True, False), (True, True), (False, True)):
        try:
            solid = build(spline, ruled)
        except Exception:
            continue
        if solid.isValid() and len(solid.Solids()) == 1 and solid.Volume() > 0:
            return solid
    return build(True, True)          # let the caller's own gate reject it


# ---------------------------------------------------------------------------
# how much room is there? (the "never break the skin" machinery)
# ---------------------------------------------------------------------------


class _Window:
    """The vertical band a channel centre may occupy at one station.

    Everything about staying inside the wing goes through here. For a station
    at (x, y) with a profile that drops `drop` below its centre and rises
    `rise` above it, the centre z must satisfy ``lo <= z <= hi`` where the
    limits come from the REAL crown and keel of the local section, sampled at
    the profile's own extreme chordwise/spanwise offsets - not just on the
    axis, because the section is a wedge and the widest part of the channel is
    the part that breaks out.
    """

    __slots__ = ("lo", "hi", "crown", "keel")

    def __init__(self, lo: float, hi: float, crown: float, keel: float):
        self.lo, self.hi, self.crown, self.keel = lo, hi, crown, keel

    @property
    def ok(self) -> bool:
        return self.hi >= self.lo


def _crown_keel(wing: Any, x: float, y: float) -> tuple[float, float]:
    """Upper and lower surface z at a world (x, y), or (nan, nan) off-wing."""
    half = float(wing.half)
    s = _clamp(y / max(half, 1e-6), -0.995, 0.995)
    xc = float(wing.xc_at(s, x))
    if not (0.015 <= xc <= 0.985):
        # at the leading-edge cusp or on the trailing edge the section has no
        # usable depth; treat it as unavailable rather than pretending.
        return float("nan"), float("nan")
    return float(wing.crown_z(s, xc)), float(wing.keel_z(s, xc))


class _Nacelle:
    """Model of the fused motor nacelle's own cross-section.

    `backend.cad.geometry._motor_mount` lofts the nacelle from the body's own
    section at `x_root` to a circle of `r_plate` at the mounting face, with a
    smoothstep blend and the body end pulled 7% inside the skin so the fuse is
    never tangent. That is a lot of material aft of the body's own trailing
    edge, and a motor conduit is unbuildable without it: on the reference wing
    the bare section is 5.6 mm deep at x = 240 mm while the nacelle there is
    over 30 mm.

    Reproducing the blend here (rather than importing geometry) keeps the
    module boundary the caller asked for. The coupling is one expression and
    is deliberately conservative - it takes the ELLIPSE inscribed in the
    blended section, which the real loft always contains - and every result is
    still gated on the mesh test, so an out-of-date model costs a conduit,
    never the airframe.
    """

    EMBED = 0.93           # geometry._NACELLE_EMBED

    def __init__(self, wing: Any, x_face: float, x_root: float, y_c: float,
                 z_c: float, r_plate: float):
        self.wing = wing
        self.x_face, self.x_root = x_face, x_root
        self.y_c, self.z_c, self.r = y_c, z_c, max(r_plate, 0.0)
        # geometry: a_root = clamp(2*r_plate, 1.2*b_root, 0.9*fb*half)
        self.a_root = max(min(2.0 * r_plate,
                              0.9 * float(wing.fb) * float(wing.half)),
                          r_plate)

    def _s(self, x: float) -> float:
        """Blend parameter: 0 at the body root, 1 at the motor face."""
        if abs(self.x_face - self.x_root) < 1e-6:
            return 1.0
        u = (x - self.x_root) / (self.x_face - self.x_root)
        return _smoothstep(u)

    def covers(self, x: float) -> bool:
        lo, hi = sorted((self.x_face, self.x_root))
        # past the mounting face there is no aircraft at all; treat it as
        # covered so a lead-in stub outside the solid does not constrain.
        return x >= lo - 1e-6 if hi == self.x_face else x <= hi + 1e-6

    def section(self, x: float) -> tuple[float, float, float]:
        """(z_centre, half_height, half_width) of the nacelle at station x."""
        s = _clamp(self._s(x), 0.0, 1.0)
        crown, keel = _crown_keel(self.wing, x, self.y_c)
        if math.isnan(crown):
            b_body, z_body, a_body = 0.0, self.z_c, 0.0
        else:
            b_body = self.EMBED * 0.5 * (crown - keel)
            z_body = 0.5 * (crown + keel)
            a_body = self.EMBED * self.a_root
        return (z_body + (self.z_c - z_body) * s,
                b_body + (self.r - b_body) * s,
                a_body + (self.r - a_body) * s)

    def band(self, p: Vector, u_dir: Vector, half_w: float, drop: float,
             rise: float, wall: float) -> tuple[float, float]:
        """Allowed centre-z band inside the nacelle for a channel at `p`.

        Intersected over the SAME five spanwise samples `_window` uses, so
        that the band the router trusts is exactly the band the skin
        measurement afterwards checks. Judging the nacelle on the channel's
        axis alone and then measuring at its shoulder is how the motor channel
        ended up 0.05 mm thinner over the skin than advertised.
        """
        lo, hi = -1e9, 1e9
        for k in (-1.0, -0.5, 0.0, 0.5, 1.0):
            q = p + u_dir.multiply(k * half_w)
            if not self.covers(q.x):
                return 1.0, -1.0
            z_c, b, a = self.section(q.x)
            dy = abs(q.y - self.y_c)
            if b <= wall or a <= 0.0 or dy >= a:
                return 1.0, -1.0
            h = b * math.sqrt(max(1.0 - (dy / a) ** 2, 0.0))
            if h <= wall:
                return 1.0, -1.0
            lo = max(lo, z_c - h + wall + drop)
            hi = min(hi, z_c + h - wall - rise)
        return lo, hi


def _window(wing: Any, p: Vector, u_dir: Vector, half_w: float,
            drop: float, rise: float, wall: float,
            extra: "_Nacelle | None" = None) -> _Window:
    """Allowed centre-z band at `p` for a profile `half_w` wide.

    Sampled at u = -half_w, -half_w/2, 0, +half_w/2, +half_w so a channel
    close to the leading or trailing edge is judged on its WIDEST point, not
    just on its axis - the section is a wedge, and it is the shoulder of the
    channel that breaks out first.
    """
    lo, hi = -1e9, 1e9
    crown_min, keel_max = 1e9, -1e9
    n_valid = 0
    for k in (-1.0, -0.5, 0.0, 0.5, 1.0):
        q = p + u_dir.multiply(k * half_w)
        crown, keel = _crown_keel(wing, q.x, q.y)
        if math.isnan(crown):
            continue          # off the wing (or on the LE/TE cusp)
        n_valid += 1
        crown_min = min(crown_min, crown)
        keel_max = max(keel_max, keel)
        lo = max(lo, keel + wall + drop)
        hi = min(hi, crown - wall - rise)
    if n_valid == 0:
        lo, hi = 1.0, -1.0                      # the wing offers nothing here
    if extra is not None:
        # A fused boss (the motor nacelle) supplies material where the bare
        # wing section has none. The two bands are ALTERNATIVES, not something
        # to merge - their hull would admit a z that is inside neither - so
        # take whichever genuinely offers more room.
        e_lo, e_hi = extra.band(p, u_dir, half_w, drop, rise, wall)
        if e_hi - e_lo > hi - lo:
            z_c, b, _a = extra.section(p.x)
            lo, hi = e_lo, e_hi
            crown_min, keel_max = z_c + b, z_c - b
    if hi < lo and n_valid == 0:
        # genuinely outside the aircraft (a lead-in stub in free air, or the
        # far end already inside the bay void): nothing to break through.
        return _Window(-1e9, 1e9, float("nan"), float("nan"))
    return _Window(lo, hi, crown_min, keel_max)


# ---------------------------------------------------------------------------
# the router: fit a channel of the largest tier that stays inside the skin
# ---------------------------------------------------------------------------


def _route(wing: Any, xy: list[tuple[float, float]],
           tiers: Sequence[tuple[float, float, str, str]],
           wall: float, z_pref: list[float], *,
           roof_deg: float = 45.0,
           nacelle: "_Nacelle | None" = None,
           smooth_passes: int = 6,
           flare: Sequence[float] | None = None,
           section: str = "teardrop") -> dict[str, Any]:
    """Pick a size and a centreline z for a channel following `xy`.

    `xy` is the plan-view route (world x, y) and `z_pref` the z the channel
    would like to hold at each station (typically the section mid-line, or the
    motor axis near the mount). Returns a dict with the chosen tier, the 3D
    path, per-station sizes, and the measured minimum skin thickness above and
    below - which is the number that proves the channel did not nearly break
    out.

    `flare` is an optional per-station size multiplier (>= 1.0), used to bell
    the mouths of a run so a wire never meets a restriction at an entry or an
    exit. It participates in the window fitting like everything else, so a
    flared station that would break the skin fails the tier honestly instead
    of breaking out.
    """
    n = len(xy)
    fl = ([1.0] * n if flare is None
          else [max(1.0, float(v)) for v in flare])
    worst: dict[str, Any] | None = None
    for width, height, name, note in tiers:
        w = width + 2.0 * LAYER_COMP_MM
        h = (height + 2.0 * LAYER_COMP_MM) if height else None
        # The flare is a WISH, the base tier is the CONTRACT: a trumpeted
        # mouth that would break the skin shrinks per-station toward the
        # base size instead of failing the whole tier - the window fitter
        # cannot see that the mouth opens into an already-carved pocket or
        # bay, so treating the flare as load-bearing refused every servo run
        # whose pocket sits in a section the trumpet cannot clear (measured:
        # the 610 lost both). Kept monotone toward each mouth so what
        # remains is still a smooth funnel.
        fl_t = list(fl)

        def _ext_at(i: int):
            if section == "circle":
                return (0.5 * w * fl_t[i],) * 3
            return teardrop_extents(w * fl_t[i], (h * fl_t[i]) if h else None,
                                    roof_deg=roof_deg)

        ext = [_ext_at(i) for i in range(n)]

        # The frames depend on z and the allowed z depends on the frames, so
        # iterate: two passes is enough to settle to well under 0.05 mm, and
        # it matters - judging the clearance on a provisional tangent and then
        # sweeping on the final one is how a channel ends up 0.06 mm thinner
        # over the skin than the router believed.
        z = list(z_pref)
        frames: list[tuple[Vector, Vector, Vector]] = []
        wins: list[_Window] = []
        fits = True
        for _outer in range(3):
            frames = _frames([Vector(x, y, zi)
                              for (x, y), zi in zip(xy, z)])
            for _shrink in range(12):
                wins = []
                for (x, y), (t, u, v), (half_w, drop, rise) in zip(xy, frames,
                                                                   ext):
                    # the profile is measured perpendicular to the path; a
                    # tilted run projects taller onto z: inflate by 1/cos.
                    sec = 1.0 / max(abs(v.z), 0.70)
                    wins.append(_window(wing, Vector(x, y, 0.0), u, half_w,
                                        drop * sec, rise * sec, wall,
                                        nacelle))
                bad = [i for i, wd in enumerate(wins) if not wd.ok]
                if not bad:
                    break
                shrunk = False
                for i in bad:
                    if fl_t[i] > 1.0 + 1e-6:
                        fl_t[i] = max(1.0, 0.8 * fl_t[i])
                        shrunk = True
                if not shrunk:
                    # a BASE-size station does not fit: the tier truly fails.
                    # Remember WHERE, so the caller can act on it.
                    i = min(bad, key=lambda j: wins[j].hi - wins[j].lo)
                    have = wins[i].crown - wins[i].keel
                    worst = {"tier_tried": name,
                             "at_xy": [round(xy[i][0], 1),
                                       round(xy[i][1], 1)],
                             "section_depth_mm": (None if math.isnan(have)
                                                  else round(have, 2)),
                             "needed_mm": round(ext[i][1] + ext[i][2]
                                                + 2.0 * wall, 2),
                             "short_by_mm": round(wins[i].lo - wins[i].hi,
                                                  2)}
                    fits = False
                    break
                # keep the funnel tangent-smooth: no station may flare wider
                # than its neighbour nearer the mouth
                mid = n // 2
                for i in range(1, mid + 1):
                    fl_t[i] = min(fl_t[i], fl_t[i - 1])
                for i in range(n - 2, mid - 1, -1):
                    fl_t[i] = min(fl_t[i], fl_t[i + 1])
                ext = [_ext_at(i) for i in range(n)]
            if not fits:
                break
            z = [_clamp(zp, wd.lo, wd.hi) for zp, wd in zip(z_pref, wins)]
            for _ in range(smooth_passes):       # gentle, then re-clamp
                zs = list(z)
                for i in range(1, n - 1):
                    zs[i] = 0.25 * z[i - 1] + 0.5 * z[i] + 0.25 * z[i + 1]
                z = [_clamp(zi, wd.lo, wd.hi) for zi, wd in zip(zs, wins)]
        if not fits:
            continue                             # this tier cannot fit

        path = [Vector(x, y, zi) for (x, y), zi in zip(xy, z)]
        # Measure what we actually left standing, at the profile's extreme
        # points. This is the number that proves the channel did not nearly
        # break out; anything at or below `wall` means the router used its
        # entire budget and the tier below should have been taken.
        skin_above, skin_below = 1e9, 1e9
        at_above = at_below = None
        for p, (t, u, vv), (half_w, drop, rise) in zip(path, frames, ext):
            sec = 1.0 / max(abs(vv.z), 0.70)
            for k in (-1.0, -0.5, 0.0, 0.5, 1.0):
                q = p + u.multiply(k * half_w)
                crown, keel = _crown_keel(wing, q.x, q.y)
                if math.isnan(crown):
                    crown, keel = -1e9, 1e9
                if nacelle is not None:
                    z_c, b, a = nacelle.section(q.x)
                    dy = abs(q.y - nacelle.y_c)
                    if nacelle.covers(q.x) and a > 0.0 and dy < a:
                        hh = b * math.sqrt(max(1.0 - (dy / a) ** 2, 0.0))
                        crown = max(crown, z_c + hh)
                        keel = min(keel, z_c - hh)
                if crown < -1e8:
                    continue                     # no material at this point
                a_gap = crown - (p.z + rise * sec)
                b_gap = (p.z - drop * sec) - keel
                if a_gap < skin_above:
                    skin_above, at_above = a_gap, (q.x, q.y)
                if b_gap < skin_below:
                    skin_below, at_below = b_gap, (q.x, q.y)
        return {
            "tier": name,
            "tier_note": note,
            "section": section,
            "width_mm": round(w, 3),
            "height_mm": round(h if h else w, 3),
            "total_height_mm": round(ext[n // 2][1] + ext[n // 2][2], 3),
            "mouth_flare": round(max(fl_t), 2),
            "path": path,
            "sizes": [(w * fl_t[i], (h * fl_t[i]) if h else w * fl_t[i])
                      for i in range(n)],
            "roof_deg": roof_deg,
            "min_skin_above_mm": round(skin_above, 3),
            "min_skin_below_mm": round(skin_below, 3),
            "min_skin_above_at_xy": (None if at_above is None
                                     else [round(c, 1) for c in at_above]),
            "min_skin_below_at_xy": (None if at_below is None
                                     else [round(c, 1) for c in at_below]),
            "wall_required_mm": round(wall, 3),
        }
    return {"tier": None, "path": [], "sizes": [], "tightest": worst}


# ---------------------------------------------------------------------------
# public: motor conduit
# ---------------------------------------------------------------------------


def motor_conduit(wing: Any, *, x_mount: float, y_mount: float, z_mount: float,
                  x_bay_aft: float, bay_half_width: float, wall: float,
                  params: dict | None = None) -> tuple[Solid | None, dict]:
    """Channel for the three motor phase leads: mount face -> equipment bay.

    Parameters
    ----------
    wing
        AeroForge's blended-wing section generator. Needs ``.half``, ``.tc``,
        ``.section(f)``, ``.crown_z(f, xc)``, ``.keel_z(f, xc)``,
        ``.xc_at(f, x)``. `f` is the SIGNED span fraction.
    x_mount, y_mount, z_mount
        Centre of the motor mounting face, world mm. For a pusher this is the
        aft face; a tractor is handled automatically (the run direction is
        taken from the sign of ``x_bay_aft - x_mount``).
    x_bay_aft
        World x of the bay VOID's near wall - the face the conduit has to
        pierce. The channel is driven `bay_overshoot` mm past it so the
        bulkhead is always broken through and the channel really opens into
        the compartment.
    bay_half_width
        Half width of the bay void, mm. A motor that is off the centreline is
        curved inboard until it is within 60% of this, so the conduit enters
        through the bay's aft bulkhead rather than skimming a side wall.
    wall
        Skin to leave standing, mm. Raised to :data:`MIN_SKIN_MM` if smaller.
    params
        Optional dict:
          ``plate_radius_mm``, ``plate_thickness_mm``
              Let the router know about the fused motor nacelle, so it can use
              the boss's material where the bare wing section is too thin.
              Take them straight from
              ``design['geometry']['motor_mount']``.
          ``bolt_circle_radius_mm``, ``screw_hole_d_mm``, ``n_screws``
              Bolt pattern to stay clear of; the channel is shrunk if it would
              graze a screw bore.
          ``z_end_mm`` (None)
              World z the channel should arrive at where it pierces the bay
              wall - the middle of the CAVITY's band there, not the middle of
              the wing section. On a deep centre body the two differ by 25+ mm
              and a channel aimed at the section mid punches through the
              bulkhead above the compartment's ceiling and dead-ends.
          ``tiers``  - override :data:`MOTOR_CONDUIT` ``tiers_mm``.
          ``face_lead_mm`` (3.0), ``bay_overshoot_mm`` (6.0),
          ``n_stations`` (24), ``roof_deg`` (45.0).

    Returns
    -------
    (cutter, info)
        `cutter` is a solid to subtract from the airframe - use
        :func:`cut_conduit`, which gates the result on the mesh-area test.
        It is ``None`` if no tier fits, in which case ``info['skipped']``
        explains why. `info` is all-millimetre.
    """
    p = dict(params or {})
    wall = max(float(wall), MIN_SKIN_MM)
    lead = float(p.get("face_lead_mm", 3.0))
    over = float(p.get("bay_overshoot_mm", 6.0))
    n = int(p.get("n_stations", 24))
    roof = float(p.get("roof_deg", 45.0))

    sgn = -1.0 if x_bay_aft < x_mount else 1.0      # -1 pusher, +1 tractor
    x0 = x_mount - sgn * lead                        # start OUTSIDE the face
    x1 = x_bay_aft + sgn * over                      # end INSIDE the bay
    if abs(x1 - x0) < 8.0:
        return None, {"skipped": "mount and bay are less than 8 mm apart",
                      "tier": None}

    # ---- size tiers, trimmed by the bolt circle --------------------------
    tiers = list(p.get("tiers") or
                 [(w, None, nm, note) for w, nm, note
                  in MOTOR_CONDUIT["tiers_mm"]])
    r_bolt = float(p.get("bolt_circle_radius_mm", 0.0) or 0.0)
    d_screw = float(p.get("screw_hole_d_mm", 0.0) or 0.0)
    screw_limit = None
    if r_bolt > 0.0:
        # the channel's half width plus the screw bore's radius must stay
        # inside the bolt circle, with 1 mm of material between them
        screw_limit = 2.0 * max(r_bolt - 0.5 * d_screw - 1.0, 0.0)
        tiers = [(min(w, screw_limit), h, nm, note)
                 for w, h, nm, note in tiers]
        tiers = [t for t in tiers if t[0] >= MIN_CHANNEL_MM]
        if not tiers:
            return None, {"skipped": "bolt circle leaves no room for a bore",
                          "tier": None, "screw_clear_width_mm": screw_limit}

    # ---- nacelle model ----------------------------------------------------
    nac = None
    r_plate = float(p.get("plate_radius_mm", 0.0) or 0.0)
    if r_plate > 0.0:
        t_plate = float(p.get("plate_thickness_mm", 4.0) or 4.0)
        # same expression geometry._motor_mount uses for the fairing length
        depth = max(2.2 * r_plate, 3.0 * t_plate, 6.0 * wall, 18.0)
        nac = _Nacelle(wing, x_mount, x_mount + sgn * depth,
                       y_mount, z_mount, r_plate)

    # ---- plan route -------------------------------------------------------
    y_end = _clamp(y_mount, -0.6 * bay_half_width, 0.6 * bay_half_width)
    z_end = p.get("z_end_mm")
    xy: list[tuple[float, float]] = []
    z_pref: list[float] = []
    for i in range(n):
        t = i / (n - 1)
        x = x0 + (x1 - x0) * t
        y = y_mount + (y_end - y_mount) * _smoothstep((t - 0.15) / 0.6)
        xy.append((x, y))
        crown, keel = _crown_keel(wing, x, y)
        z_mid = z_mount if math.isnan(crown) else 0.5 * (crown + keel)
        # hold the motor axis while the nacelle is carrying us, then fall
        # smoothly to the middle of the local wing section
        hold = 1.0 if nac is None else _smoothstep((nac._s(x) - 0.10) / 0.45)
        z_i = z_mount * hold + z_mid * (1.0 - hold)
        if z_end is not None:
            # ... and finally DIVE to the cavity's own band over the last 40%
            # of the run, so the mouth opens into the compartment's air rather
            # than into the bulkhead above its ceiling
            w = _smoothstep((t - 0.60) / 0.35)
            z_i = z_i * (1.0 - w) + float(z_end) * w
        z_pref.append(z_i)

    # A ROUND PIPE with smoothly TRUMPETED mouths, same as the servo runs
    # (builder, round 4): constant 8.25 mm circle through the run, widening
    # tangentially into each opening so there is no rim at the belly entry
    # or where the run breaks into the bay.
    _nn = len(xy)
    _sec = str(p.get("section", "circle"))
    _mf = float(p.get("mouth_flare", 1.6))
    _flare = [1.0 + (_mf - 1.0)
              * max(_smoothstep((0.15 - i / (_nn - 1)) / 0.15),
                    _smoothstep((i / (_nn - 1) - 0.85) / 0.15))
              for i in range(_nn)]
    res = _route(wing, xy, tiers, wall, z_pref, roof_deg=roof, nacelle=nac,
                 flare=_flare, section=_sec)
    info: dict[str, Any] = {
        "kind": "motor",
        "carries": MOTOR_CONDUIT["what"],
        "x_start_mm": round(x0, 2), "x_end_mm": round(x1, 2),
        "direction": "forward (pusher)" if sgn < 0 else "aft (tractor)",
        "length_mm": round(abs(x1 - x0), 2),
        "screw_clear_width_mm": (round(screw_limit, 2)
                                 if screw_limit is not None else None),
        "nacelle_modelled": nac is not None,
        "verdict_on_6mm": MOTOR_CONDUIT["verdict_on_6mm"],
        "sources": MOTOR_CONDUIT["sources"],
    }
    if res.get("tier") is None:
        info.update(tier=None, tightest=res.get("tightest"),
                    skipped="no tier fits inside the section with "
                            f"{wall:.2f} mm of skin")
        return None, info

    info.update({k: v for k, v in res.items() if k not in ("path", "sizes")})
    # the centreline, so the caller can verify the channel after cutting
    # (route_is_open / route_connects / skin_audit all take it)
    info["path_mm"] = [[round(q.x, 4), round(q.y, 4), round(q.z, 4)]
                       for q in res["path"]]
    info["entry_mm"] = [round(c, 2) for c in
                        (res["path"][0].x, res["path"][0].y, res["path"][0].z)]
    info["exit_mm"] = [round(c, 2) for c in
                       (res["path"][-1].x, res["path"][-1].y,
                        res["path"][-1].z)]
    try:
        cutter = _sweep(res["path"], res["sizes"], roof_deg=roof,
                        section=_sec)
    except Exception as exc:                         # pragma: no cover
        info.update(tier=None, skipped=f"loft failed: {exc}")
        return None, info
    if not cutter.isValid() or len(cutter.Solids()) != 1:
        info.update(tier=None, skipped="cutter lofted to an invalid solid")
        return None, info
    info["cutter_volume_mm3"] = round(cutter.Volume(), 1)
    return cutter, info


# ---------------------------------------------------------------------------
# public: servo conduit
# ---------------------------------------------------------------------------


def servo_conduit(wing: Any, *, start: Vector, end: Vector, wall: float,
                  params: dict | None = None) -> tuple[Solid | None, dict]:
    """Channel for one elevon servo lead: servo bay -> equipment bay.

    Parameters
    ----------
    start
        Where the lead leaves the servo pocket, world mm. Honoured exactly
        (the channel has to meet the pocket) except that its z is clamped into
        whatever band the section allows there.
    end
        Where the channel is to pierce the equipment-bay wall, world mm. The
        route is driven `bay_overshoot_mm` past it, into the void.
    wall
        Skin to leave, mm; raised to :data:`MIN_SKIN_MM` if smaller.
    params
        Optional dict:
          ``hinge_xc`` (0.72)
              Chord fraction of the elevon hinge line. The corridor is kept
              at least ``hinge_margin_xc`` forward of it so the channel never
              enters the hinge/knuckle region or a separated elevon.
          ``hinge_margin_xc`` (0.12), ``corridor_xc`` (0.32)
              `corridor_xc` is where the run wants to sit - 0.32 chord is the
              deepest part of a typical reflexed section, which is exactly
              where a shallow outer panel has room for a channel.
          ``tiers`` - override :data:`SERVO_CONDUIT` ``tiers_mm``.
          ``bay_overshoot_mm`` (5.0), ``start_overshoot_mm`` (2.0),
          ``n_stations`` (30), ``roof_deg`` (45.0).

    Returns
    -------
    (cutter, info) - see :func:`motor_conduit`.
    """
    p = dict(params or {})
    wall = max(float(wall), MIN_SKIN_MM)
    n = int(p.get("n_stations", 30))
    roof = float(p.get("roof_deg", 45.0))
    over = float(p.get("bay_overshoot_mm", 8.0))
    # RAISED 2.0 -> 6.0 (builder feedback: obstructions at the exits): the
    # run now drives deep INTO the pocket cavity, so its end cap is a face in
    # free air, never a ledge flush with the pocket wall.
    out = float(p.get("start_overshoot_mm", 6.0))
    hinge_xc = float(p.get("hinge_xc", 0.72))
    hinge_m = float(p.get("hinge_margin_xc", 0.12))
    xc_best = _clamp(float(p.get("corridor_xc", 0.32)),
                     0.12, max(hinge_xc - hinge_m, 0.15))

    half = float(wing.half)
    if abs(start.y - end.y) < 5.0:
        return None, {"skipped": "start and end are less than 5 mm apart "
                                 "spanwise", "tier": None}

    sgn = 1.0 if start.y >= 0.0 else -1.0
    f0 = _clamp(start.y / half, -0.995, 0.995)
    f1 = _clamp(end.y / half, -0.995, 0.995)
    xc0 = float(wing.xc_at(f0, start.x))
    xc1 = float(wing.xc_at(f1, end.x))
    warnings: list[str] = []
    if xc0 > hinge_xc - 0.02:
        warnings.append(
            f"servo cable exit is at {xc0:.2f} chord, at or behind the "
            f"{hinge_xc:.2f} hinge line - move the servo forward")

    # Push the run out past the servo pocket and INTO the bay so both ends
    # definitely break through.
    #
    # Overshooting into the bay is free - there is no material there to remove
    # - and it is the only defence against the caller aiming at the bay's
    # NOMINAL wall while the hatch module actually built a narrower one. On the
    # reference wing `body['bay_width_m']` says 59.7 mm but the bay that got
    # cut is 57.3 mm, so an endpoint computed from the nominal width sits
    # 2.4 mm OUTSIDE the real void; a short overshoot would leave the channel
    # dead-ending in the bay wall. Clamped so it can never cross the
    # centreline or swallow the whole run.
    y_a = start.y + sgn * out
    over = min(over, 0.45 * abs(start.y - end.y), max(abs(end.y) - 2.0, 0.0))
    y_b = end.y - sgn * over

    xy: list[tuple[float, float]] = []
    z_pref: list[float] = []
    for i in range(n):
        t = i / (n - 1)
        y = y_a + (y_b - y_a) * t
        f = _clamp(y / half, -0.995, 0.995)
        # quadratic Bezier on the CHORD FRACTION, not on x: the panel sweeps
        # and tapers, so a straight line in x would wander from the leading
        # edge at the root to the trailing edge at the tip. Holding a chord
        # fraction keeps the channel in the thick part of every section it
        # crosses, which is the whole trick to fitting one in an outer panel.
        ctrl = 2.0 * xc_best - 0.5 * (xc0 + xc1)     # Bezier passes xc_best
        xc = ((1 - t) ** 2 * xc0 + 2 * t * (1 - t) * ctrl + t ** 2 * xc1)
        xc = _clamp(xc, 0.08, hinge_xc - hinge_m
                    if i not in (0, n - 1) else 0.97)
        sec = wing.section(f)
        x = float(sec.le.x) + xc * float(sec.chord)
        xy.append((x, y))
        crown, keel = _crown_keel(wing, x, y)
        z_pref.append(0.5 * (crown + keel) if not math.isnan(crown)
                      else float(sec.le.z))
    # honour the caller's endpoint heights over the first/last 25% of the run
    for i in range(n):
        t = i / (n - 1)
        w0 = 1.0 - _smoothstep(t / 0.25)
        w1 = _smoothstep((t - 0.75) / 0.25)
        z_pref[i] = (z_pref[i] * (1.0 - w0 - w1)
                     + start.z * w0 + end.z * w1)

    tiers = list(p.get("tiers") or SERVO_CONDUIT["tiers_mm"])
    # A ROUND PIPE with smoothly TRUMPETED mouths (builder, round 4: "0 sharp
    # corners or edges leading up to it and the transition at all, make it
    # all smooth"): the bore is a constant 8.25 mm circle through the run and
    # widens tangentially (smoothstep loft - no edge anywhere on the wall)
    # over the last 15% into each opening, so where it breaks into the servo
    # pocket and the equipment bay the wire meets a funnel, not a rim.
    sec = str(p.get("section", "circle"))
    mf = float(p.get("mouth_flare", 1.6))
    flare = [1.0 + (mf - 1.0) * max(_smoothstep((0.15 - i / (n - 1)) / 0.15),
                                    _smoothstep((i / (n - 1) - 0.85) / 0.15))
             for i in range(n)]
    res = _route(wing, xy, tiers, wall, z_pref, roof_deg=roof, flare=flare,
                 section=sec)

    info: dict[str, Any] = {
        "kind": "servo",
        "side": "right" if sgn > 0 else "left",
        "carries": SERVO_CONDUIT["what"],
        "corridor_xc": round(xc_best, 3),
        "hinge_xc": hinge_xc,
        "xc_start": round(xc0, 3), "xc_end": round(xc1, 3),
        "span_run_mm": round(abs(y_b - y_a), 2),
        "bay_overshoot_mm": round(over, 2),
        "pocket_overshoot_mm": round(out, 2),
        "cable_envelope_mm": SERVO_CONDUIT["cable_envelope_mm"],
        "connector_envelope_mm": SERVO_CONDUIT["connector_envelope_mm"],
        "connector_note": SERVO_CONDUIT["connector_note"],
        "warnings": warnings,
        "sources": SERVO_CONDUIT["sources"],
    }
    if res.get("tier") is None:
        info.update(tier=None, tightest=res.get("tightest"),
                    skipped="no tier fits inside the panel with "
                            f"{wall:.2f} mm of skin")
        return None, info

    info.update({k: v for k, v in res.items() if k not in ("path", "sizes")})
    # the centreline, so the caller can verify the channel after cutting
    # (route_is_open / route_connects / skin_audit all take it)
    info["path_mm"] = [[round(q.x, 4), round(q.y, 4), round(q.z, 4)]
                       for q in res["path"]]
    info["passes_connector"] = res["tier"] in ("connector",
                                               "connector_edgewise")
    info["entry_mm"] = [round(c, 2) for c in
                        (res["path"][0].x, res["path"][0].y, res["path"][0].z)]
    info["exit_mm"] = [round(c, 2) for c in
                       (res["path"][-1].x, res["path"][-1].y,
                        res["path"][-1].z)]
    path_len = sum((res["path"][i + 1] - res["path"][i]).Length
                   for i in range(len(res["path"]) - 1))
    info["length_mm"] = round(path_len, 2)
    try:
        cutter = _sweep(res["path"], res["sizes"], roof_deg=roof,
                        section=sec)
    except Exception as exc:                         # pragma: no cover
        info.update(tier=None, skipped=f"loft failed: {exc}")
        return None, info
    if not cutter.isValid() or len(cutter.Solids()) != 1:
        info.update(tier=None, skipped="cutter lofted to an invalid solid")
        return None, info
    info["cutter_volume_mm3"] = round(cutter.Volume(), 1)
    return cutter, info


# ---------------------------------------------------------------------------
# public: straight conduit (builder's spec, round 5 - 2026-08-24)
# ---------------------------------------------------------------------------
#
# "I need them to be direct and straight ... one straight continuous
# extrusion (so if i wanted to stick a straight stick cylinder through, it
# would go right through without blockage, bending, or twists)."
#
# The curved routers above bend the pipe to follow the wing's thickness
# corridor. This one does the opposite: the centreline is a LINE, chosen so
# the whole bore stays inside the skin, and if no line fits the run refuses
# with numbers instead of bending. The only shaping allowed is the trumpeted
# mouth, which widens the bore coaxially and so never blocks a straight rod.


def _material_bands(hosts: Sequence[Any], nacelle: "_Nacelle | None",
                    x: float, y: float) -> list[tuple[float, float]]:
    """Merged (keel, crown) material intervals at plan point (x, y).

    `hosts` may mix the wing duck type (``half``/``xc_at``/``crown_z``/
    ``keel_z``) and the fuselage-profile duck type (``contains_plan`` +
    ``crown(x, y)``/``keel(x, y)``). A run that crosses from a wing panel
    into a fuselage sees one continuous band where the two solids are fused,
    which is exactly what merging overlapping intervals models - enforcing
    each host's own skin margin at an INTERIOR junction would refuse a run
    through perfectly solid material.
    """
    bands: list[tuple[float, float]] = []
    for h in hosts:
        if hasattr(h, "contains_plan"):          # fuselage / pod profile
            try:
                if h.contains_plan(x, y, 0.8):
                    k, c = float(h.keel(x, y)), float(h.crown(x, y))
                    if c - k > 0.05:
                        bands.append((k, c))
            except Exception:
                pass
        else:                                     # wing section generator
            c, k = _crown_keel(h, x, y)
            if not math.isnan(c) and c - k > 0.05:
                bands.append((k, c))
    if nacelle is not None and nacelle.covers(x):
        z_c, b, a = nacelle.section(x)
        dy = abs(y - nacelle.y_c)
        if a > 0.0 and dy < a and b > 0.05:
            hh = b * math.sqrt(max(1.0 - (dy / a) ** 2, 0.0))
            if hh > 0.05:
                bands.append((z_c - hh, z_c + hh))
    if not bands:
        return []
    bands.sort()
    merged: list[list[float]] = [list(bands[0])]
    for lo, hi in bands[1:]:
        if lo <= merged[-1][1] + 0.5:            # fused / touching solids
            merged[-1][1] = max(merged[-1][1], hi)
        else:
            merged.append([lo, hi])
    return [(lo, hi) for lo, hi in merged]


def _line_ok(hosts: Sequence[Any], nacelle: "_Nacelle | None",
             path: Sequence[Vector], radii: Sequence[float],
             wall: float, belly_prefix: bool = False
             ) -> tuple[bool, dict, list[float]]:
    """Does a straight bore of per-station `radii` stay inside the skin?

    Stations where no host has material (free air below the belly, or air
    inside an already-carved pocket or bay the analytic hosts cannot see)
    are skipped, exactly as `_route` skips its nan stations. Stations whose
    centre lies OUTSIDE every band while material exists above or below are
    also skipped - that is the run passing under/over the solid on its way
    in, and the mouth regions rely on it. What is never skipped is a centre
    INSIDE a band: there the full bore plus `wall` must fit that band.

    `belly_prefix` is for the motor run's deliberate keel breach: floor-side
    violations are tolerated over a contiguous prefix of the run (the entry
    mouth cutting through the belly skin); the prefix ends at the first
    station whose floor clearance is honest, after which any breach is a
    real one. The roof is never excused.

    Returns (ok, info, per-station shrink flags for the flare ladder).
    The info of a failing line names the tightest station.
    """
    n = len(path)
    frames = _frames(list(path))
    shrink = [1.0] * n
    worst: dict | None = None
    min_above, min_below = 1e9, 1e9
    at_above = at_below = None
    in_prefix = bool(belly_prefix)
    mat_any = [False] * n
    for i, (p, (t, u, v)) in enumerate(zip(path, frames)):
        sec = 1.0 / max(abs(v.z), 0.70)
        r_i = radii[i]
        rw, rh = (r_i if isinstance(r_i, tuple) else (r_i, r_i))
        floor_clean = True
        saw_material = False
        for k in (-1.0, -0.5, 0.0, 0.5, 1.0):
            # lateral reach is the HALF-WIDTH; the vertical clearance the
            # bore needs at that shoulder follows the ellipse (full height
            # on the centreline, tapering to the rim)
            need = rh * math.sqrt(max(0.0, 1.0 - k * k))
            q = p + u.multiply(k * rw)
            bands = _material_bands(hosts, nacelle, q.x, q.y)
            if not bands:
                continue
            mat_any[i] = True
            home = None
            for lo, hi in bands:
                if lo - 0.05 <= p.z <= hi + 0.05:
                    home = (lo, hi)
                    break
            if home is None:
                continue                       # passing outside the solid
            saw_material = True
            lo, hi = home
            a_gap = (hi - p.z) - need * sec
            b_gap = (p.z - lo) - need * sec
            if b_gap < wall:
                floor_clean = False
            if in_prefix and b_gap < wall:
                b_gap = wall                   # the entry mouth: excused
            if a_gap < wall or b_gap < wall:
                fail = {"at_xy": [round(q.x, 1), round(q.y, 1)],
                        "band_mm": [round(lo, 1), round(hi, 1)],
                        "need_mm": round(2.0 * (need * sec + wall), 2),
                        "have_mm": round(hi - lo, 2),
                        "short_by_mm": round(
                            wall - min(a_gap, b_gap), 2)}
                if worst is None or fail["short_by_mm"] > worst["short_by_mm"]:
                    worst = fail
                shrink[i] = 0.0
            if a_gap < min_above:
                min_above, at_above = a_gap, (round(q.x, 1), round(q.y, 1))
            if b_gap < min_below:
                min_below, at_below = b_gap, (round(q.x, 1), round(q.y, 1))
        if in_prefix and saw_material and floor_clean:
            in_prefix = False                  # the belly is behind us
    # A straight line may begin or end in air (a pocket, the void, the
    # belly stub) but it may not LEAVE the aircraft mid-run: a tube
    # through free air between two solids is not wire management, it is a
    # snag. Any material-free station strictly between the first and last
    # material stations refuses the line.
    if any(mat_any):
        first = mat_any.index(True)
        last = n - 1 - mat_any[::-1].index(True)
        for i in range(first + 1, last):
            if not mat_any[i]:
                worst = {"at_xy": [round(path[i].x, 1),
                                   round(path[i].y, 1)],
                         "gap": "the line leaves the aircraft mid-run "
                                "(free air between two solids)",
                         "short_by_mm": 99.0}
                break
    info = {"min_skin_above_mm": (round(min_above, 3)
                                  if at_above else None),
            "min_skin_below_mm": (round(min_below, 3)
                                  if at_below else None),
            "min_skin_above_at_xy": list(at_above) if at_above else None,
            "min_skin_below_at_xy": list(at_below) if at_below else None,
            "tightest": worst}
    return worst is None, info, shrink


def straight_conduit(hosts: Any, *, start: Vector,
                     end_xy: tuple[float, float],
                     end_z_band: tuple[float, float], wall: float,
                     params: dict | None = None) -> tuple[Solid | None, dict]:
    """ONE straight round pipe from `start` to a point over `end_xy`.

    The plan endpoint is fixed by the caller; the end HEIGHT is searched
    inside `end_z_band` (the measured void band, or a single value pinned by
    passing lo == hi) for a line the whole bore can ride without breaking
    the skin. The start is honoured exactly - it is the lead grommet, or a
    belly entry the caller has already committed to.

    Parameters in `params` (all optional):
      ``d_mm`` (8.25), ``mouth_flare`` (1.0 - the bore is CONSTANT end to
          end, builder's decision round 6; >1 would restore the retired
          trumpet), ``n_stations`` (25),
      ``start_overshoot_mm`` (6.0)  - driven back past the start, into the
          pocket cavity / free air, so the mouth cap sits in air;
      ``end_overshoot_mm`` (8.0)    - driven past the end, into the void;
      ``max_start_x_drift_mm``      - cap on how far the start overshoot may
          move the mouth chordwise (the grommet-position contract);
      ``prefer_z``                  - end z tried first (closest-first scan);
      ``nacelle``                   - a `_Nacelle`, extra material model;
      ``hinge_guard``               - {"hinge_xc", "margin_xc", "span_lo_mm",
          "span_hi_mm"}: stations whose |y| lies in the span band must stay
          forward of the hinge on hosts[0].

    Returns (cutter, info) like the other routers. `info["straight"]` is
    True and `info["path_mm"]` is collinear by construction - a straight
    rod passes end to end.
    """
    hosts = list(hosts) if isinstance(hosts, (list, tuple)) else [hosts]
    p = dict(params or {})
    wall = max(float(wall), MIN_SKIN_MM)
    d = float(p.get("d_mm", 8.25))
    # builder's round 7: the wire ports are OVAL - wider than tall, "to
    # make more room for the wires", still one straight constant section.
    # Width defaults to 12.0 for the 8.25 bore (the same ratio elsewhere).
    w = float(p.get("w_mm", d * OVAL_W_RATIO))
    w = max(w, d)
    mf = float(p.get("mouth_flare", 1.0))
    n = int(p.get("n_stations", 25))
    out = float(p.get("start_overshoot_mm", 6.0))
    over = float(p.get("end_overshoot_mm", 8.0))
    nacelle = p.get("nacelle")
    rw_base = 0.5 * w + LAYER_COMP_MM
    rh_base = 0.5 * d + LAYER_COMP_MM
    r_base = rh_base

    ex, ey = float(end_xy[0]), float(end_xy[1])
    z_lo, z_hi = (float(end_z_band[0]), float(end_z_band[1]))
    if z_hi < z_lo:
        z_lo, z_hi = z_hi, z_lo
    plan_len = math.hypot(ex - start.x, ey - start.y)
    if plan_len < 5.0:
        return None, {"skipped": "start and end are less than 5 mm apart "
                                 "in plan", "tier": None, "straight": True}

    oval = w > d + 1e-6
    info_base: dict[str, Any] = {
        "straight": True, "section": ("oval" if oval else "circle"),
        "d_mm": d, "h_mm": d, "w_mm": round(w, 3),
        "trumpeted": bool(mf > 1.0 + 1e-6), "mouth_flare": mf,
        "wall_required_mm": round(wall, 3),
    }

    # ---- hinge guard: independent of z, so check the plan line once -------
    guard = p.get("hinge_guard")
    if guard and hosts:
        h0 = hosts[0]
        hx = float(guard.get("hinge_xc", 0.72))
        hm = float(guard.get("margin_xc", 0.05))
        s_lo = float(guard.get("span_lo_mm", 0.0))
        s_hi = float(guard.get("span_hi_mm", 1e9))
        half = float(getattr(h0, "half", 1.0))
        for i in range(n):
            t = i / (n - 1)
            x = start.x + (ex - start.x) * t
            y = start.y + (ey - start.y) * t
            if not (s_lo <= abs(y) <= s_hi):
                continue
            f = _clamp(y / max(half, 1e-6), -0.995, 0.995)
            try:
                xc = float(h0.xc_at(f, x))
            except Exception:
                continue
            if 0.0 <= xc <= 1.0 and xc > hx - hm:
                return None, dict(
                    info_base, tier=None,
                    skipped=(f"straight line crosses the hinge corridor at "
                             f"y={y:.0f} mm (xc={xc:.2f}, hinge {hx:.2f}) - "
                             "no straight route without entering the "
                             "elevon/knuckle region"))

    # ---- scan candidate end heights, closest to the preference first ------
    prefer = float(p.get("prefer_z", 0.5 * (z_lo + z_hi)))
    if z_hi - z_lo < 1e-6:
        cands = [z_lo]
    else:
        grid = list(np.linspace(z_lo, z_hi, 15))
        cands = sorted(grid, key=lambda z: abs(z - prefer))

    chosen = None
    last_fail: dict | None = None
    for z_e in cands:
        end = Vector(ex, ey, float(z_e))
        u = _unit(end - start)
        # the start overshoot may not walk the mouth off the grommet
        o = out
        drift_cap = p.get("max_start_x_drift_mm")
        if drift_cap is not None and abs(u.x) > 1e-6:
            o = min(o, float(drift_cap) / abs(u.x))
        p0 = start - u.multiply(o)
        p1 = end + u.multiply(over)
        path = [p0 + (p1 - p0).multiply(i / (n - 1)) for i in range(n)]
        total = (p1 - p0).Length
        o_frac = o / max(total, 1e-6)
        v_frac = over / max(total, 1e-6)
        flare = []
        for i in range(n):
            t = i / (n - 1)
            # trumpet over the 15% of the CORE run nearest each mouth, plus
            # the overshoot stubs themselves
            f0 = _smoothstep((o_frac + 0.15 - t) / 0.15)
            f1 = _smoothstep((t - (1.0 - v_frac - 0.15)) / 0.15)
            flare.append(1.0 + (mf - 1.0) * max(f0, f1))
        radii = [(rw_base * fl, rh_base * fl) for fl in flare]
        belly = bool(p.get("belly_entry"))
        ok, linfo, shrink = _line_ok(hosts, nacelle, path, radii, wall,
                                     belly_prefix=belly)
        if not ok:
            # the flare is a wish, the base bore is the contract: retry the
            # failing stations at base size before rejecting the line
            radii2 = [(rw_base, rh_base) if s == 0.0 else r
                      for r, s in zip(radii, shrink)]
            if radii2 != radii:
                # keep the funnel monotone toward each mouth
                for i in range(1, n):
                    radii2[i] = (
                        min(radii2[i][0], max(radii2[i - 1][0], rw_base)),
                        min(radii2[i][1], max(radii2[i - 1][1], rh_base)))
                for i in range(n - 2, -1, -1):
                    radii2[i] = (
                        min(radii2[i][0], max(radii2[i + 1][0], rw_base)),
                        min(radii2[i][1], max(radii2[i + 1][1], rh_base)))
                ok, linfo, _ = _line_ok(hosts, nacelle, path, radii2, wall,
                                        belly_prefix=belly)
                radii = radii2
        if ok:
            chosen = (path, radii, linfo, float(z_e))
            break
        last_fail = linfo.get("tightest") or last_fail

    if chosen is None:
        return None, dict(info_base, tier=None, tightest=last_fail,
                          skipped=("no straight line from the start to the "
                                   "target wall fits inside the skin with "
                                   f"{wall:.2f} mm left standing"))

    path, radii, linfo, z_e = chosen
    sizes = [(2.0 * r[0], 2.0 * r[1]) for r in radii]
    try:
        cutter = _sweep(path, sizes,
                        section=("oval" if oval else "circle"))
    except Exception as exc:                          # pragma: no cover
        return None, dict(info_base, tier=None,
                          skipped=f"pipe loft failed: {exc}")
    if not cutter.isValid() or len(cutter.Solids()) != 1:
        return None, dict(info_base, tier=None,
                          skipped="pipe lofted to an invalid solid")

    u = _unit(path[-1] - path[0])
    info = dict(info_base)
    info.update({
        "tier": "pipe",
        "tier_note": ((f"one straight oval {w:g} x {d:g} mm pipe, "
                       if oval else f"one straight round {d:g} mm pipe, ")
                      + ("trumpeted mouths" if mf > 1.0 + 1e-6
                         else "constant bore")),
        "width_mm": round(2.0 * rw_base, 3),
        "height_mm": round(2.0 * rh_base, 3),
        "end_z_mm": round(z_e, 2),
        "path_mm": [[round(q.x, 4), round(q.y, 4), round(q.z, 4)]
                    for q in path],
        "entry_mm": [round(c, 2) for c in (path[0].x, path[0].y, path[0].z)],
        "exit_mm": [round(c, 2) for c in (path[-1].x, path[-1].y,
                                          path[-1].z)],
        "length_mm": round((path[-1] - path[0]).Length, 2),
        "axis_unit": [round(c, 4) for c in (u.x, u.y, u.z)],
        "slope_deg": round(math.degrees(math.asin(_clamp(u.z, -1.0, 1.0))),
                           1),
        "cutter_volume_mm3": round(cutter.Volume(), 1),
    })
    info.update({k: v for k, v in linfo.items() if k != "tightest"})
    return cutter, info


# ---------------------------------------------------------------------------
# cutting, verification and the fallback ladder
# ---------------------------------------------------------------------------


def tess_stats(solid: Solid, tol: float = 0.6) -> tuple[float, float]:
    """(meshed area / BRep area, enclosed volume of the mesh).

    The area ratio is the load-bearing check: `isValid()` is not enough and
    never has been, because OCC will report a boolean result valid and then
    silently skip faces whose trim curves it cannot tessellate, which is a
    hole in the exported skin. Only the AREA ratio sees a big face going
    missing - a per-face "did it triangulate" test does not, because the face
    is still there, just mostly empty.

    The mesh volume comes free from the same tessellation and is worth having:
    `Shape.Volume()` is NOT trustworthy on the assembled airframe (it has an
    inner shell for the equipment bay, and OCC's GProp misreports the
    enclosed volume by ~12% on it), whereas the divergence-theorem sum over
    the triangles is exactly what the exported STL encloses.
    """
    try:
        verts, tris = solid.tessellate(tol)
    except Exception:
        return 0.0, 0.0
    if not tris:
        return 0.0, 0.0
    p = np.asarray([[v.x, v.y, v.z] for v in verts], dtype=float)
    idx = np.asarray(tris, dtype=int)
    a, b, c = p[idx[:, 0]], p[idx[:, 1]], p[idx[:, 2]]
    cr = np.cross(b - a, c - a)
    area = 0.5 * float(np.sum(np.linalg.norm(cr, axis=1)))
    vol = float(np.sum(np.einsum("ij,ij->i", a, np.cross(b, c)))) / 6.0
    ref = float(solid.Area())
    return ((area / ref) if ref > 0.0 else 0.0), vol


def tess_ratio(solid: Solid, tol: float = 0.6) -> float:
    """Meshed area / BRep area. See :func:`tess_stats`."""
    return tess_stats(solid, tol)[0]


# ---------------------------------------------------------------------------
# the mesh gate
# ---------------------------------------------------------------------------
#
# TWO tests, and they catch different things - this is the lesson `hatch` also
# had to learn, and getting the balance wrong costs you real geometry.
#
#   * AREA RATIO (meshed area / BRep area). The load-bearing one. OCC can mesh
#     every face and still lose most of a BIG one, and that is what puts a hole
#     in the exported skin - it once cost 73 mm of nose. Kept STRICT, always.
#
#   * PER FACE ("did every face triangulate at all"). Catches a face dropped
#     whole. But on its own it false-fails, and it did: the servo conduits were
#     rejected on this airframe by a face that measured
#
#         area 0.00343 mm2 (1.9e-8 of the surface), BSpline, 2 edges,
#         bbox x[254.878, 254.888] y[-0.259, 0.259] z[3.261, 3.261],
#         uv u[0, 0.00002] - a 0.52 mm long, 0.01 mm wide degenerate strip
#
#     which is the POLE of the motor nacelle's loft, at the mounting face on
#     the centreline. It is nothing to do with a conduit: it is present on the
#     airframe BEFORE any conduit is cut. The servo channels simply happened to
#     be cut first and inherited the blame; the motor channel, cut last, passed
#     only because its own bore removes that pole.
#
# Two conclusions, both implemented below:
#
#   1. A face that fails the per-face test is INTERROGATED before the cut is
#      condemned: re-mesh that face on its own down a tolerance ladder. If it
#      meshes at a finer chord it was a tolerance artifact (this one meshes at
#      0.1 relative, and at 0.6 absolute). If it will not mesh at any
#      tolerance it is a genuine defect, tolerated only if negligible in both
#      absolute and relative area - and reported either way.
#
#   2. A cut is judged on what it ADDED. Faces that were already unmeshable
#      before the boolean are matched by position and reported as
#      `pre_existing`, never charged to the conduit. Convicting a channel of a
#      defect it did not create is how a perfectly good channel gets thrown
#      away - and how an afternoon gets spent looking in the wrong place.

#: Chord tolerances tried on a single suspect face, coarse to fine.
#: (deflection, relative). Relative scales the chord by the face's own size,
#: which is what a sliver needs; the absolute rungs catch the rest.
FACE_MESH_LADDER = ((0.30, True), (0.10, True), (0.05, False), (0.01, False))

#: A face that will not mesh at ANY tolerance is tolerated only if it is this
#: small. Above it, something real has been dropped and the cut is rejected.
MAX_UNMESHED_FACE_MM2 = 1.0

#: ...and only if all such faces together are this fraction of the surface.
MAX_UNMESHED_AREA_FRAC = 1.0e-4


def _clean_mesh(shape: Any) -> None:
    """Drop any cached triangulation so the next mesh really re-runs."""
    from OCP.BRepTools import BRepTools
    try:
        BRepTools.Clean_s(shape)
    except Exception:
        pass


def unmeshed_faces(solid: Solid, tol: float = 0.6,
                   relative: bool = True) -> list[Any]:
    """The faces OCC leaves without a triangulation at this chord tolerance."""
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopLoc import TopLoc_Location
    try:
        BRepMesh_IncrementalMesh(solid.wrapped, tol, relative)
    except Exception:
        return list(solid.Faces())
    bad = []
    for f in solid.Faces():
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(f.wrapped, loc)
        if tri is None or tri.NbTriangles() == 0:
            bad.append(f)
    return bad


def untriangulated_faces(solid: Solid, tol: float = 0.6) -> int:
    """How many faces OCC left without a triangulation (count only)."""
    return len(unmeshed_faces(solid, tol))


def face_meshes_alone(face: Any,
                      ladder: Sequence[tuple[float, bool]] = FACE_MESH_LADDER
                      ) -> tuple[bool, float | None]:
    """Will this face triangulate on its own if we ask more finely?

    Meshing the face in isolation is the cheap way to tell a TOLERANCE
    artifact (a legitimate sliver the coarse chord skipped) from a genuine
    defect (a face whose trim curves OCC cannot follow at any resolution).
    Returns (ok, tolerance that worked).
    """
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopLoc import TopLoc_Location
    for t, rel in ladder:
        try:
            _clean_mesh(face.wrapped)
            BRepMesh_IncrementalMesh(face.wrapped, t, rel)
            loc = TopLoc_Location()
            tri = BRep_Tool.Triangulation_s(face.wrapped, loc)
            if tri is not None and tri.NbTriangles() > 0:
                return True, t
        except Exception:
            continue
    return False, None


def face_key(face: Any, grid: float = 0.05) -> tuple[float, float, float]:
    """Position key for matching a face across a boolean.

    Face identity is not preserved through an OCC boolean, but a face the
    boolean never touched keeps its position exactly. Rounding the bounding-box
    centre to a coarse grid is enough to recognise it again, which is all the
    pre-existing-defect check needs.
    """
    bb = face.BoundingBox()
    return (round(0.5 * (bb.xmin + bb.xmax) / grid) * grid,
            round(0.5 * (bb.ymin + bb.ymax) / grid) * grid,
            round(0.5 * (bb.zmin + bb.zmax) / grid) * grid)


def mesh_audit(solid: Solid, tol: float = 0.6, *,
               area_ratio_min: float = 0.985,
               baseline_keys: "set | None" = None) -> dict[str, Any]:
    """Full verdict on whether a solid will export cleanly.

    Runs the strict area-ratio test, then interrogates any face that would not
    triangulate at `tol`. See the block comment above for why both are needed
    and why the per-face one alone is not trustworthy.

    `baseline_keys` are :func:`face_key` values of faces that were ALREADY
    unmeshable before whatever operation produced `solid`. Those are reported
    but never counted against it.

    The returned ``unmeshable_keys`` is what a caller passes back as the next
    call's `baseline_keys`.
    """
    ratio, vol = tess_stats(solid, tol)
    rep: dict[str, Any] = {
        "mesh_ratio": round(ratio, 5),
        "mesh_volume_mm3": round(vol, 1),
        "area_ratio_ok": ratio >= area_ratio_min,
    }
    # NOTE: after tess_stats, so the area figure is taken before the per-face
    # probing wipes and rewrites individual triangulations.
    bad = unmeshed_faces(solid, tol)
    rep["untriangulated_faces"] = len(bad)
    total_area = float(solid.Area()) or 1.0
    base = baseline_keys or set()
    rescued: list[float] = []
    stubborn: list[float] = []
    pre: list[float] = []
    keys: list[tuple] = []
    worst_box = None
    finest = None
    for f in bad:
        try:
            area = float(f.Area())
        except Exception:
            area = 0.0
        ok, at = face_meshes_alone(f)
        if ok:
            rescued.append(area)
            finest = at if finest is None else min(finest, at)
            continue
        k = face_key(f)
        keys.append(k)
        if k in base:
            pre.append(area)                  # was already broken; not ours
            continue
        stubborn.append(area)
        if worst_box is None or area >= max(stubborn):
            bb = f.BoundingBox()
            worst_box = [round(v, 3) for v in (bb.xmin, bb.xmax, bb.ymin,
                                               bb.ymax, bb.zmin, bb.zmax)]
    _clean_mesh(solid.wrapped)          # leave no half-written mesh behind
    rep["faces_rescued_by_finer_mesh"] = len(rescued)
    rep["rescued_area_mm2"] = round(sum(rescued), 6)
    rep["finest_tol_needed"] = finest
    rep["faces_pre_existing"] = len(pre)
    rep["pre_existing_area_mm2"] = round(sum(pre), 6)
    rep["faces_unmeshable"] = len(stubborn)
    rep["unmeshable_area_mm2"] = round(sum(stubborn), 6)
    rep["unmeshable_area_frac"] = round(sum(stubborn) / total_area, 9)
    rep["biggest_unmeshable_mm2"] = round(max(stubborn), 6) if stubborn else 0.0
    rep["biggest_unmeshable_bbox"] = worst_box
    rep["unmeshable_keys"] = keys
    rep["faces_ok"] = (
        rep["biggest_unmeshable_mm2"] <= MAX_UNMESHED_FACE_MM2
        and rep["unmeshable_area_frac"] <= MAX_UNMESHED_AREA_FRAC)
    rep["ok"] = bool(rep["area_ratio_ok"] and rep["faces_ok"])
    return rep


def _heal(solid: Solid) -> Solid:
    """ShapeFix pass, same as `geometry._heal`.

    Rebuilding the trim curves is often enough to turn the sliver at a conduit
    mouth into a face the mesher will accept. Kept only if it is still exactly
    one valid solid.
    """
    try:
        from cadquery import Shape
        from OCP.ShapeFix import ShapeFix_Shape
        sf = ShapeFix_Shape(solid.wrapped)
        sf.Perform()
        healed = Shape.cast(sf.Shape())
        solids = healed.Solids()
        if len(solids) == 1 and healed.isValid():
            return solids[0]
    except Exception:
        pass
    return solid


def _strip(shape: Any) -> Any:
    """A boolean-ready copy: no triangulation, no inherited sub-shape graph.

    THIS IS THE SINGLE BIGGEST COST CONTROL IN THE MODULE. Two OCC behaviours
    compound viciously here:

      * `tessellate()` writes a triangulation onto the shared TShape, and every
        subsequent boolean then drags that mesh through its own algorithms. The
        same cut measured 2.0 s on a clean solid against 49.0 s on a
        tessellated one.
      * a boolean RESULT keeps the sub-shape history of all its inputs, so the
        graph the next boolean has to walk grows with every operation. A plain
        `BRepBuilderAPI_Copy` severs it (13.0 s -> 3.8 s on one intersect
        elsewhere in this codebase).

    The airframe reaching `cut_conduits` has already been through the bay, the
    hatch, the elevon split and the servo pockets, so it arrives carrying both.
    Cleaning and copying it once costs a fraction of a second and pays for
    itself many times over.
    """
    try:
        _clean_mesh(shape.wrapped)
        return shape.copy(mesh=False)
    except Exception:
        return shape


def cut_conduit(solid: Solid, cutter: Solid | None, *,
                area_ratio_min: float = 0.985,
                tol: float = 0.6,
                measure_removal: bool = False,
                baseline_keys: "set | None" = None) -> tuple[Solid, dict]:
    """Subtract one conduit, keeping the cut only if the result is sound.

    Sound means: exactly one solid, ``isValid()``, meshed area
    >= `area_ratio_min` of BRep area (strict - this is the test that catches
    OCC silently dropping a face), and no face left untriangulated that is not
    demonstrably a harmless sliver (see :func:`mesh_audit`).

    If the raw cut fails only the per-face test, a ShapeFix pass is tried
    before giving up: rebuilding the trim curves usually turns the sliver at a
    conduit mouth into a face the mesher accepts.

    Returns ``(solid, report)`` - the ORIGINAL solid unchanged if the cut was
    rejected, so a bad conduit costs you the conduit, never the airframe.
    """
    rep: dict[str, Any] = {"applied": False}
    if cutter is None:
        rep["why"] = "no cutter"
        return solid, rep
    try:
        cut = _strip(solid).cut(cutter)
    except Exception as exc:
        rep["why"] = f"boolean failed: {exc}"
        return solid, rep
    solids = cut.Solids()
    rep["n_solids"] = len(solids)
    if len(solids) != 1:
        rep["why"] = f"cut produced {len(solids)} solids"
        # name what detached - which piece, where, how big - so a split
        # can be diagnosed from the report instead of rebuilt blind
        pieces = []
        for s in sorted(solids, key=lambda q: -abs(q.Volume()))[:4]:
            b = s.BoundingBox()
            pieces.append({
                "volume_mm3": round(abs(s.Volume()), 1),
                "bbox_mm": [round(b.xmin, 1), round(b.xmax, 1),
                            round(b.ymin, 1), round(b.ymax, 1),
                            round(b.zmin, 1), round(b.zmax, 1)],
            })
        rep["pieces"] = pieces
        return solid, rep
    cut = solids[0]
    rep["valid"] = bool(cut.isValid())
    if not rep["valid"]:
        # one heal before giving up: booleans against smooth-lofted
        # cavities can leave tolerance-inconsistent shells that ShapeFix
        # repairs outright (round 10: the smooth hull extension cut
        # valid geometry to an "invalid" solid). The mesh gate below
        # still has the final word.
        try:
            from cadquery.occ_impl.shapes import Shape
            from OCP.ShapeFix import ShapeFix_Shape
            fx = ShapeFix_Shape(cut.wrapped)
            fx.Perform()
            healed = Shape.cast(fx.Shape())
            hs = healed.Solids()
            if len(hs) == 1 and hs[0].isValid():
                cut = hs[0]
                rep["valid"] = True
                rep["healed_after_cut"] = True
        except Exception:
            pass
    if not rep["valid"]:
        rep["why"] = "cut is not a valid solid"
        return solid, rep

    audit = mesh_audit(cut, tol, area_ratio_min=area_ratio_min,
                       baseline_keys=baseline_keys)
    chosen, tried = cut, "cut"
    if not audit["ok"] and audit["area_ratio_ok"]:
        # only the per-face test is unhappy: try to heal the sliver away
        healed = _heal(cut)
        if healed is not cut and healed.isValid() \
                and len(healed.Solids()) == 1:
            a2 = mesh_audit(healed, tol, area_ratio_min=area_ratio_min,
                            baseline_keys=baseline_keys)
            if a2["ok"] or (a2["faces_unmeshable"] < audit["faces_unmeshable"]
                            and a2["area_ratio_ok"]):
                audit, chosen, tried = a2, healed, "healed"
    rep.update(audit)
    rep["path"] = tried
    if not audit["area_ratio_ok"]:
        rep["why"] = (f"mesh coverage {audit['mesh_ratio']:.4f} "
                      f"< {area_ratio_min}")
        return solid, rep
    if not audit["faces_ok"]:
        rep["why"] = (f"{audit['faces_unmeshable']} face(s) will not mesh at "
                      f"any tolerance, {audit['biggest_unmeshable_mm2']:.4f} "
                      f"mm2 biggest "
                      f"(limit {MAX_UNMESHED_FACE_MM2} mm2 and "
                      f"{MAX_UNMESHED_AREA_FRAC:g} of the surface)")
        return solid, rep
    rep["applied"] = True
    if measure_removal:
        # from the MESH, not from Shape.Volume() - see tess_stats
        rep["volume_removed_mm3"] = round(
            tess_stats(solid, tol)[1] - audit["mesh_volume_mm3"], 1)
    return chosen, rep


def _sequential(solid: Solid, named: list[tuple[str, Solid]],
                area_ratio_min: float, tol: float,
                measure_removal: bool) -> tuple[Solid, dict[str, dict]]:
    """Cut one at a time, each independently gated. The ATTRIBUTION path.

    Slow - a tessellation per conduit - so it only runs when the cheap
    all-at-once cut has failed and we need to know which channel is to blame.
    """
    reports: dict[str, dict] = {}
    for name, cutter in named:
        out, rep = cut_conduit(solid, cutter,
                               area_ratio_min=area_ratio_min, tol=tol,
                               measure_removal=measure_removal)
        if not rep.get("applied") and rep.get("faces_ok") is False:
            # Judge the cut on what it ADDED. Costs a whole extra tessellation,
            # so it is only ever reached after a failure.
            base = mesh_audit(solid, tol, area_ratio_min=area_ratio_min)
            keys = set(base.get("unmeshable_keys") or ())
            if keys:
                out, rep = cut_conduit(
                    solid, cutter, area_ratio_min=area_ratio_min, tol=tol,
                    measure_removal=measure_removal, baseline_keys=keys)
                rep["baseline_unmeshable_before_cut"] = len(keys)
                rep["baseline_area_mm2"] = base["unmeshable_area_mm2"]
        solid = out
        rep.pop("unmeshable_keys", None)          # noise in a report
        reports[name] = rep
    return solid, reports


def cut_conduits(solid: Solid, cutters: Sequence[tuple[str, Solid | None]], *,
                 area_ratio_min: float = 0.985,
                 tol: float = 0.6,
                 measure_removal: bool = False
                 ) -> tuple[Solid, dict[str, dict]]:
    """Subtract every conduit, then validate ONCE.

    WHY IT IS SHAPED LIKE THIS. The obvious implementation - cut a conduit,
    check it, cut the next into that result, check again - is correct and was
    costing 119.6 s of a 296 s aircraft build, 40% of the whole thing, while
    BUILDING all three cutters took 0.5 s. Almost none of that was the
    geometry. Two OCC behaviours did it (see :func:`_strip`): a validation
    tessellates the solid, and every later boolean then drags that mesh and the
    accumulated sub-shape history around with it. Conduit 3 was paying for the
    audits of conduits 1 and 2.

    So: `(S - a) - b - c` is `S - (a | b | c)`, and OCC will take all three
    tools in ONE `BRepAlgoAPI_Cut`. One boolean on a stripped copy, then one
    mesh audit on the finished solid. The happy path pays for exactly one
    tessellation instead of three, and no boolean ever sees a triangulation.

    Nothing is given up for it. Every acceptance criterion is unchanged - one
    valid solid, `area_ratio_min` on the meshed-area ratio, the per-face check
    with its finer-tolerance rescue, and pre-existing-face attribution. What is
    deferred is only the BLAME: if the combined cut fails, and only then, the
    conduits are re-cut one at a time (:func:`_sequential`) to find which one
    is at fault and drop just that one. A failure is rare and worth paying for;
    success is the common case and now costs almost nothing.
    """
    reports: dict[str, dict] = {n: {"applied": False, "why": "no cutter"}
                                for n, c in cutters if c is None}
    named = [(n, c) for n, c in cutters if c is not None]
    if not named:
        return solid, reports

    vol_before = tess_stats(solid, tol)[1] if measure_removal else None

    # ---- one boolean, all tools, on a stripped copy ----------------------
    combined: Solid | None = None
    why = ""
    try:
        cut = _strip(solid).cut(*[c for _n, c in named])
        solids = cut.Solids()
        if len(solids) != 1:
            why = f"combined cut produced {len(solids)} solids"
        elif not solids[0].isValid():
            why = "combined cut is not a valid solid"
        else:
            combined = solids[0]
    except Exception as exc:
        why = f"combined boolean failed: {exc}"

    # ---- one audit -------------------------------------------------------
    if combined is not None:
        audit = mesh_audit(combined, tol, area_ratio_min=area_ratio_min)
        chosen = combined
        path = "combined"
        if not audit["ok"] and audit["area_ratio_ok"]:
            healed = _heal(combined)
            if healed is not combined and healed.isValid() \
                    and len(healed.Solids()) == 1:
                a2 = mesh_audit(healed, tol, area_ratio_min=area_ratio_min)
                if a2["ok"]:
                    audit, chosen, path = a2, healed, "combined+healed"
        if audit["ok"]:
            shared = {k: v for k, v in audit.items() if k != "unmeshable_keys"}
            shared["path"] = path
            if measure_removal:
                shared["volume_removed_mm3"] = round(
                    (vol_before or 0.0) - audit["mesh_volume_mm3"], 1)
            for name, _c in named:
                reports[name] = dict(shared, applied=True, n_solids=1,
                                     valid=True, gate="combined")
            return chosen, reports
        why = (f"combined cut failed the mesh gate "
               f"(ratio {audit['mesh_ratio']}, "
               f"{audit['faces_unmeshable']} unmeshable face(s))")

    # ---- something is wrong: find out WHICH conduit ----------------------
    solid, reports_seq = _sequential(solid, named, area_ratio_min, tol,
                                     measure_removal)
    for name, rep in reports_seq.items():
        rep["gate"] = "sequential"
        rep["combined_attempt"] = why
    reports.update(reports_seq)
    return solid, reports


class Classifier:
    """Reusable point-in-solid test.

    ``BRepClass3d_SolidClassifier`` builds its search tree once per instance,
    so keeping one instance and calling ``Perform`` repeatedly makes a probe
    effectively free - which is what makes it practical to walk a whole route,
    and why this is used instead of intersecting a probe solid (seconds each).
    """

    def __init__(self, solid: Solid):
        from OCP.BRepClass3d import BRepClass3d_SolidClassifier
        self._c = BRepClass3d_SolidClassifier(solid.wrapped)

    def state(self, p: Vector, tol: float = 1e-4) -> int:
        from OCP.gp import gp_Pnt
        self._c.Perform(gp_Pnt(p.x, p.y, p.z), tol)
        return self._c.State()

    def inside(self, p: Vector, tol: float = 1e-4, strict: bool = True) -> bool:
        """Strictly IN the material (ON the boundary does not count)."""
        from OCP.TopAbs import TopAbs_IN, TopAbs_ON
        s = self.state(p, tol)
        return s == TopAbs_IN or (not strict and s == TopAbs_ON)


def point_is_inside(solid: Solid, p: Vector, tol: float = 1e-4,
                    strict: bool = True) -> bool:
    """Is `p` inside the material of `solid`? See :class:`Classifier`."""
    return Classifier(solid).inside(p, tol, strict)


def _resample(path: Sequence[Vector], n: int) -> list[tuple[float, Vector]]:
    """(arc length, point) at n+1 even steps along a polyline."""
    segs = [(path[i], path[i + 1]) for i in range(len(path) - 1)]
    lens = [(b - a).Length for a, b in segs]
    total = sum(lens)
    out = []
    for k in range(n + 1):
        s = total * k / n
        acc = 0.0
        for (a, b), L in zip(segs, lens):
            if acc + L >= s - 1e-9:
                u = 0.0 if L < 1e-9 else (s - acc) / L
                out.append((s, a + (b - a).multiply(_clamp(u, 0.0, 1.0))))
                break
            acc += L
        else:
            out.append((s, path[-1]))
    return out


def route_is_open(solid: Solid, path: Sequence[Vector], *,
                  n: int = 120, skip_ends_mm: float = 0.5) -> dict[str, Any]:
    """Walk a conduit's centreline and check it is void the whole way.

    After the cut every point on the route should classify OUT of the material
    - it is now a hole. Any point still classifying IN means the channel is
    blocked there: a boolean that silently did nothing, or a route that left
    the solid and came back. The first and last half-millimetre are skipped
    because the end caps put those probes exactly ON a face.
    """
    if len(path) < 2:
        return {"open": False, "why": "empty path"}
    cls = Classifier(solid)
    samples = _resample(path, n)
    total = samples[-1][0]
    blocked = [round(s, 2) for s, q in samples
               if skip_ends_mm <= s <= total - skip_ends_mm
               and cls.inside(q)]
    return {"open": not blocked, "n_probes": len(samples),
            "length_mm": round(total, 2),
            "blocked_at_mm": blocked[:12], "n_blocked": len(blocked)}


def route_connects(solid: Solid, path: Sequence[Vector],
                   targets: dict[str, Vector], *,
                   n: int = 200) -> dict[str, Any]:
    """Does the channel actually join the things it is supposed to join?

    `targets` are points that must be in the SAME void as the channel - the
    centre of the equipment bay, a point just off the motor mounting face.
    Each is walked to along a straight line from the nearest route point; the
    connection holds if no material is crossed on the way.
    """
    cls = Classifier(solid)
    out: dict[str, Any] = {}
    samples = [q for _s, q in _resample(path, n)]
    for name, tgt in targets.items():
        near = min(samples, key=lambda q: (q - tgt).Length)
        steps = max(int((tgt - near).Length / 0.5), 2)
        blocked = sum(1 for k in range(steps + 1)
                      if cls.inside(near + (tgt - near).multiply(k / steps)))
        out[name] = {"connected": blocked == 0,
                     "gap_mm": round((tgt - near).Length, 2),
                     "material_probes": blocked}
    return out


class MeshProbe:
    """Vertical ray casts against a solid's own triangulation.

    `BRepClass3d_SolidClassifier` is exact but costs ~100 ms a probe on an
    airframe made of spline lofts, which makes a dense audit (thousands of
    probes) impractical. Casting a VERTICAL ray at a triangle mesh reduces to
    a 2D point-in-triangle test plus one plane evaluation, which vectorises;
    the whole audit then takes well under a second. It also measures the
    surface that is actually EXPORTED, which for a skin-thickness number is
    the more honest target.
    """

    def __init__(self, solid: Solid, tol: float = 0.25):
        verts, tris = solid.tessellate(tol)
        p = np.asarray([[v.x, v.y, v.z] for v in verts], dtype=float)
        idx = np.asarray(tris, dtype=int)
        self.a, self.b, self.c = p[idx[:, 0]], p[idx[:, 1]], p[idx[:, 2]]
        self.n_tri = len(idx)
        # 2D edge functions, precomputed
        self._d = ((self.b[:, 1] - self.c[:, 1]) * (self.a[:, 0] - self.c[:, 0])
                   + (self.c[:, 0] - self.b[:, 0]) * (self.a[:, 1] - self.c[:, 1]))
        self._d = np.where(np.abs(self._d) < 1e-12, 1e-12, self._d)
        self.xmin = np.minimum(np.minimum(self.a[:, 0], self.b[:, 0]),
                               self.c[:, 0])
        self.xmax = np.maximum(np.maximum(self.a[:, 0], self.b[:, 0]),
                               self.c[:, 0])
        self.ymin = np.minimum(np.minimum(self.a[:, 1], self.b[:, 1]),
                               self.c[:, 1])
        self.ymax = np.maximum(np.maximum(self.a[:, 1], self.b[:, 1]),
                               self.c[:, 1])

    def _hits(self, x: float, y: float) -> np.ndarray:
        """z of every triangle the vertical line through (x, y) crosses."""
        m = ((self.xmin <= x) & (x <= self.xmax)
             & (self.ymin <= y) & (y <= self.ymax))
        if not m.any():
            return np.empty(0)
        a, b, c, d = self.a[m], self.b[m], self.c[m], self._d[m]
        l1 = ((b[:, 1] - c[:, 1]) * (x - c[:, 0])
              + (c[:, 0] - b[:, 0]) * (y - c[:, 1])) / d
        l2 = ((c[:, 1] - a[:, 1]) * (x - c[:, 0])
              + (a[:, 0] - c[:, 0]) * (y - c[:, 1])) / d
        l3 = 1.0 - l1 - l2
        inside = (l1 >= -1e-9) & (l2 >= -1e-9) & (l3 >= -1e-9)
        if not inside.any():
            return np.empty(0)
        return (l1[inside] * a[inside, 2] + l2[inside] * b[inside, 2]
                + l3[inside] * c[inside, 2])

    def column(self, x: float, y: float, merge: float = 1e-3) -> np.ndarray:
        """Sorted, de-duplicated z of every surface crossing at (x, y).

        Adjacent triangles sharing an edge the ray grazes report the same z
        twice; merging within `merge` keeps the crossing list a clean
        alternation of entering and leaving material.
        """
        z = np.sort(self._hits(x, y))
        if z.size < 2:
            return z
        return z[np.concatenate(([True], np.diff(z) > merge))]

    def skin_over(self, p: Vector) -> tuple[float, float]:
        """(material above, material below) for a point INSIDE a channel.

        Measured crossing-to-crossing, not from the probe point: going up from
        inside the channel the first surface met is the channel's own roof and
        the second is whatever is above it (the outer skin, or the floor of
        another void). Their separation is the material left standing.

        Measuring from the probe point instead is what made this read 0.000 mm
        on a perfectly good channel - the tessellated roof passes within a
        micron of the analytic roof point, so the ray "hit something
        immediately" and the channel looked like it had broken out.

        Returns ``(above, below, local_void_height)``. The third value says how
        tall the void the probe sits in actually is, which is how a caller
        tells "I am in the 5 mm channel" from "I am in the 38 mm equipment
        bay" without knowing anything about the bay. `above` and `below` are
        0 where there is no surface to measure to - the channel is not open
        there.
        """
        z = self.column(p.x, p.y)
        above = z[z > p.z + 1e-6]
        below = z[z < p.z - 1e-6][::-1]
        a = float(above[1] - above[0]) if above.size >= 2 else 0.0
        b = float(below[0] - below[1]) if below.size >= 2 else 0.0
        void = (float(above[0] - below[0])
                if above.size and below.size else 0.0)
        return a, b, void

    def inside(self, p: Vector) -> bool:
        """Odd number of surface crossings above the point -> in material.

        APPROXIMATE. OCC leaves T-junctions on every boolean, so the exported
        mesh is not perfectly manifold and parity can flip spuriously where a
        fused part (a centre fin, say) meets the skin. Use
        ``BRepClass3d_SolidClassifier`` (:class:`Classifier`) when the answer
        has to be right; this is for cheap bulk screening only.
        """
        z = self.column(p.x, p.y)
        return bool(int(np.count_nonzero(z > p.z + 1e-6)) % 2)

    def void_span(self, x: float, y: float) -> tuple[float, float] | None:
        """The tallest empty band on the vertical line at (x, y).

        Handy for finding a point that is definitely inside the equipment bay
        without knowing how the bay was built.
        """
        z = self.column(x, y)
        if z.size < 4:
            return None
        # crossings alternate enter/leave; the voids are the odd gaps
        best = None
        for i in range(1, z.size - 1, 2):
            gap = z[i + 1] - z[i]
            if best is None or gap > best[1] - best[0]:
                best = (float(z[i]), float(z[i + 1]))
        return best


def skin_audit(solid: Solid, path: Sequence[Vector],
               width: float, height: float | None, *,
               roof_deg: float = 45.0, n_stations: int = 80,
               tol: float = 0.25, end_skip_mm: float = 1.5,
               probe: "MeshProbe | None" = None) -> dict[str, Any]:
    """Measure, ON THE CUT SOLID, how much material is left over the channel.

    For every station the channel's own roof and floor points are cast
    straight up / straight down until they meet the next surface. That
    distance IS the remaining skin, measured on the real built shape - boss,
    fin, bay wall and all - rather than on the analytic section, which is the
    number that proves the channel did not nearly break out.

    A distance of 0 means the point is already outside the material: the
    channel HAS broken through there.

    The first and last `end_skip_mm` are excluded. A vertical ray at either
    end cap's own (x, y) lies IN the cap's plane for a spanwise run, so it
    grazes that face and returns a meaningless pile of near-coincident
    crossings - which shows up as a phantom breakthrough at exactly the two
    end stations and nowhere else.
    """
    a, drop, rise = teardrop_extents(width, height, roof_deg=roof_deg)
    mp = probe or MeshProbe(solid, tol)
    frames = _frames(list(path))
    samples = _resample(list(path), n_stations)

    ch_h = drop + rise
    best_a, best_b = 1e9, 1e9
    at_a = at_b = None
    breaches: list[tuple[float, float]] = []
    n_measured = n_other = 0
    total = samples[-1][0]
    for _s, p in samples:
        if _s < end_skip_mm or _s > total - end_skip_mm:
            n_other += 5
            continue
        i = min(range(len(path)), key=lambda j: (path[j] - p).Length)
        _t, u, _v = frames[i]
        for k in (-0.8, -0.4, 0.0, 0.4, 0.8):
            # Probe from the channel AXIS at this offset: the point is inside
            # the void, so the first crossing going up is the channel's own
            # roof and the second is the skin over it.
            q = p + u.multiply(k * a)
            ra, rb, void = mp.skin_over(q)
            if void > 1.7 * ch_h or void <= 0.0:
                # not in the channel: either inside the equipment bay (where
                # the route deliberately ends) or outside the aircraft (the
                # lead-in stub). Nothing to measure, and not a breach.
                n_other += 1
                continue
            if ra <= 0.0 or rb <= 0.0:
                breaches.append((round(p.x, 1), round(p.y, 1)))
                continue
            n_measured += 1
            if ra < best_a:
                best_a, at_a = ra, (round(p.x, 1), round(p.y, 1))
            if rb < best_b:
                best_b, at_b = rb, (round(p.x, 1), round(p.y, 1))
    return {"min_skin_above_mm": round(best_a, 3) if at_a else None,
            "min_skin_below_mm": round(best_b, 3) if at_b else None,
            "above_at_xy": list(at_a) if at_a else None,
            "below_at_xy": list(at_b) if at_b else None,
            "breakthroughs": len(breaches),
            "breakthrough_at_xy": breaches[:6],
            "probes_in_channel": n_measured,
            "probes_elsewhere": n_other,
            "end_skip_mm": end_skip_mm,
            "n_stations": len(samples), "mesh_tol_mm": tol}
