"""Print-in-place hinges for RC control surfaces.

Self-contained: this module never imports `backend.cad.geometry` (that would
be an import cycle). It takes plain CadQuery solids plus the geometry of the
hinge line and hands back modified solids.

WHAT A PRINT-IN-PLACE HINGE ACTUALLY IS
---------------------------------------
Two leaves, interleaved cylindrical knuckles on a common axis, and a pin that
is *part of one leaf* and passes through the barrel of the other with a real
radial gap, so the joint comes off the bed already articulating. There is no
separate wire to thread. That is the whole point, and it is what the previous
bored-knuckle-plus-music-wire arrangement in `geometry.py` was missing.

Layout used here, three knuckles (the minimum that works - 2 + 1):

        |<----------------- hinge_length ----------------->|
        [ wing knuckle ]:[ surface barrel ]:[ wing knuckle ]      : = axial gap
        =================== pin (wing) =====================      runs the lot

The pin belongs to the WING and is continuous end to end, so it is a single
printed rod in tension/shear rather than two stubs. The CONTROL SURFACE gets
the barrel: a tube whose bore is the pin diameter plus twice the radial
clearance. Wing and surface never share volume - the engagement is purely
topological (a rod threaded through a ring), which is why the pair separates
cleanly in the slicer and still cannot be pulled apart.

Each leaf is MORTISED where the other leaf's knuckle sits: a flat-bottomed
rectangular notch cut back from the hinge plane, not a cylindrical pocket. A
cylindrical pocket is tidier on paper but on a 4 mm section it leaves a 0.3 mm
sliver of skin over its crown, thinner than one extrusion, and the slicer
either drops it or prints stringy rubbish there. The mortise is also safe
under rotation: everything a leaf keeps at that span lies further from the
axis than the mortise is deep, and the mating knuckle never leaves a cylinder
of that radius, so the clearance holds at every deflection angle.

CLEARANCES, AND WHERE THE NUMBERS COME FROM
-------------------------------------------
* radial clearance pin -> bore: **0.35 mm per side** (0.70 mm on diameter).
  3D Printer Stuff calls 0.3 mm per side "the universal starting point" for
  print-in-place hinges at a 0.2 mm layer height; Snapmaker and ozFDM both put
  the working window at 0.2-0.3 mm for PLA, with 0.15 mm only on a very well
  calibrated machine and 0.4 mm+ "a very loose joint with lots of play". We
  take a shade ABOVE the top of that window for one specific reason: our
  barrel axis is horizontal (spanwise, parallel to the bed), so the crown of
  the bore is an unsupported bridge over the pin and the first bridged layer
  always sags a little into the gap, and the horizontal diameter of the
  annulus is a plain XY gap where Bambu's own 0.35 mm applies. A vertically
  printed pin could use 0.2 mm; this one cannot. It stops at 0.35 rather than
  going to 0.5 with the flat gaps because a pin touches its bore on a LINE,
  and because the surface can float 2 x this figure bodily in the joint -
  see `cove_running_clearance_mm`.
    - https://www.3dprinterstuff.com/workshop/design-print-in-place-hinges-that-work
    - https://www.snapmaker.com/blog/3d-printed-hinges/
    - https://ozfdm.com.au/blogs/knowledge-base-blogs-knowledge-base/print-in-place-designs-hinges-joints-and-mechanisms
  (PETG wants +0.1-0.15 mm on top of PLA values - ozFDM. Exposed as a
  parameter rather than baked in.)

* axial clearance between adjacent knuckle end faces: **0.50 mm**. 3D Printer
  Stuff's "0.2 mm clearance between adjacent knuckle faces" is a Z gap on a
  vertically printed hinge; ours is an XY gap between two flat annular faces
  of several square millimetres, which is area contact and takes the flat-face
  number (see PRINTING THE WHOLE AIRCRAFT below), not the pin number.

* clearance between a knuckle and the mortise cut for it: **0.50 mm**. Flat
  face again, and a running clearance on top of that.

* wall around the bore: **0.80 mm**, i.e. two perimeters at a 0.4 mm nozzle,
  the accepted absolute minimum for a printed wall - below that the slicer
  either skips the wall or leaves gaps worth 50-70% of the strength. 1.2 mm
  (three perimeters) is the recommendation for loaded walls, and we use 1.2 mm
  whenever the section is deep enough to pay for it.
    - https://layerx3d.in/blog/fdm-design-rules-wall-thickness-overhangs-bridging-tolerances
    - https://www.raise3d.com/blog/3d-printing-wall-thickness/

* knuckle end chamfers: **0.4 mm at 45 deg** (3D Printer Stuff: "0.3-0.5 mm
  45-degree cuts on edges"). Also the trick the OpenSCAD vertical-hinge
  library uses - the funnel-shaped knuckle profile - so the end faces meet on
  a line instead of a plane and break free with a thumb-push.

* knuckle length: **>= 1.4 x barrel OD, floor 3.0 mm**. 3D Printer Stuff wants
  each knuckle >= 4 mm tall and a pin at least 1.5 x its own diameter long;
  at our scale (a 4 mm thick outboard section) 4 mm per knuckle is the right
  order and the 1.5 x pin rule is satisfied by a wide margin because the pin
  runs the *whole* hinge, not one knuckle.

* hinge gap, wing TE to surface LE: **0.50 mm**. Not a print clearance - a
  running clearance, so the two never rub through the throw. Was 0.35 mm and
  welded on the plate; see "PRINTING THE WHOLE AIRCRAFT" below for why 0.35 mm
  is under the floor for two model walls facing each other in the XY plane.

* spanwise END clearance, surface end face to wing: **0.50 mm**. The elevon is
  freed by intersecting the airframe with a half-space AND a span box, so its
  inboard and outboard end faces are the cut planes themselves and start life
  coincident with the wing that surrounds them - measured as 0.0000 mm over 24
  contact solutions on a real optimized wing. The surface is trimmed back
  along the hinge axis at both ends to open that up. See `clear_surface_ends`.

PRINTING THE WHOLE AIRCRAFT ON A BAMBU LAB H2D
----------------------------------------------
The target case is the entire airframe on one plate, 0.4 mm nozzle, PLA,
default profile, tree supports. That fixes the orientation - the wing lies
flat, so the hinge axis is parallel to the bed and EVERY clearance in this
module except the crown of the bore is a gap between two walls printed side by
side in the same layers, i.e. an XY gap, not a Z gap. Z gaps can be a layer
height; XY gaps cannot.

* **What a free print-in-place joint actually needs.** The published design
  guides land in a narrow band and they are describing exactly this
  orientation (the classic print-in-place hinge lies flat on the plate):
  Snapmaker calls "0.2 mm to 0.3 mm between the pin and the barrel ... the
  sweet spot" for a well calibrated FDM machine; ozFDM puts the PLA window at
  0.2-0.3 mm with 0.15 mm only on a very well calibrated printer and 0.4 mm+
  "a very loose joint with lots of play"; 3D Printer Stuff calls 0.3 mm per
  side the universal starting point at a 0.2 mm layer height. Material tables
  quote the same thing as a joint total: "PLA ... 0.5 mm for rotating joints
  (hinges, bearings)" (UAVMODEL, 3DPut) against "0.3 mm for sliding joints".
  0.30 mm per side = 0.60 mm on diameter brackets that 0.5 mm figure and sits
  at the top of the per-side window, which is where we want to be with a
  bridged bore. Sources DISAGREE at the tight end - Bambu forum users report
  0.10-0.20 mm working for press-ish fits on an X1C - but those are fits that
  are meant to be snug and to be freed with a poke, not a control surface that
  has to swing for the life of the model, so we follow the design guides.
    - https://www.snapmaker.com/blog/3d-printed-hinges/
    - https://ozfdm.com.au/blogs/knowledge-base-blogs-knowledge-base/print-in-place-designs-hinges-joints-and-mechanisms
    - https://www.3dprinterstuff.com/workshop/design-print-in-place-hinges-that-work
    - https://blog.uavmodel.com/3d-printer-print-in-place-mechanism-design-clearance-tolerances-hinge-geometry-and-joint-guidelines-2026/
    - https://3dput.com/complete-guide-to-3d-printing-tolerances-and-fit-clearance-for-moving-parts-2/
    - https://forum.bambulab.com/t/how-do-you-use-tolerance-value-in-your-design-with-bl/123711

* **Die swell sets the floor.** A 0.4 mm nozzle lays a ~0.45 mm line in PLA
  (UAVMODEL), and a slicer will not even resolve a gap much under 0.05 mm at
  that nozzle (Bambu forum). Two model walls 0.20-0.35 mm apart in XY are
  inside one extrusion of each other and the walls touch as they are laid.

* **Bambu's own separability number is 0.35 mm.** Bambu Studio's default
  Support/object XY distance - the distance at which Bambu itself is willing
  to bet that two printed things come apart - is 0.35 mm, and that is for
  support material, which is deliberately under-extruded and weak. Two solid
  model perimeters need more. That is the direct argument for the leading-edge
  running gap going to 0.50 mm.
    - https://all3dp.com/2/bambu-studio-support-settings-simply-explained/

* **Elephant foot.** The first layer is squished against the bed and comes out
  wider than designed; PrusaSlicer's guidance is that "values around 0.2 mm
  usually work well for the default 0.4 mm nozzle" of compensation. Bambu's
  0.4 mm profiles ship 0.15 mm of elephant-foot compensation - but not every
  profile does (the 0.6 mm P1S profile shipped 0), so a designer cannot lean
  on it. A wing printed flat has its whole hinge line ON the first few layers,
  so budget ~0.15 mm of first-layer growth per side: 0.35 mm of design gap can
  be ~0.05 mm of real gap on layer one, which is where it welds. 0.35 (Bambu's
  own XY separability) + 0.15 (elephant foot) = 0.50 mm, and that is where
  `hinge_gap_mm` and `end_clearance_mm` come from.
    - https://help.prusa3d.com/article/elephant-foot-compensation_114487
    - https://github.com/bambulab/BambuStudio/issues/2621

* **Tree supports do not change the required gap, because they cannot get in.**
  Support material has to be extruded as at least one line (~0.42-0.45 mm
  wide) and it stands off the model by the Support/object XY distance on BOTH
  sides (0.35 mm default). The narrowest gap that can physically receive
  support is therefore about 0.35 + 0.42 + 0.35 = 1.1 mm. Every gap in this
  module - the 0.50 mm hinge gap, the 0.50 mm end clearance, the 0.30 mm
  radial annulus around the pin, the 0.35 mm axial gaps between knuckles - is
  well under that, so NO support material enters the joint or the barrel.
  There is nothing to pick out of a hinge afterwards, and the gap does not
  need to be opened to let a tool in. (It also means the bore crown bridges
  unsupported over the pin, which is the reason the radial clearance is taken
  at the top of the window rather than the middle.) Tree supports are
  additionally the wrong thing to trust inside a joint: Bambu Studio's tree
  supports are documented not to honour the top Z distance exactly, rounding
  it up to the layer height.
    - https://all3dp.com/2/bambu-studio-support-settings-simply-explained/
    - https://github.com/bambulab/BambuStudio/issues/4644
    - https://github.com/bambulab/BambuStudio/issues/1827

* **What RC practice asks for.** Full-size and model practice both want the
  hinge line gap SMALL and preferably sealed - "gaps at the hinge area of a
  control surface increase drag and lower the efficiency of the control
  surface", and builders tape or seal a gap of roughly 1/16 in (1.6 mm) on a
  conventionally hinged surface. 0.5 mm is a third of that and is already an
  aerodynamically tight, effectively sealed hinge line, so the printability
  floor is the binding constraint here, not the aerodynamics.
    - https://www.rcuniverse.com/forum/questions-answers-154/300643-how-do-i-seal-aileron-hinge-line.html
    - https://en.wikibooks.org/wiki/RC_Airplane/Control_surfaces_and_linkages

HOW A REAL CONTROL SURFACE IS HINGED AND BEVELLED
-------------------------------------------------
This is the part the old code skipped entirely, and it is why the elevon could
not move: a square-nosed surface hinged on its own centreline jams against the
wing trailing edge at about one degree.

Two RC practices exist:

1. **Radiused leading edge, pivot recessed to the centre of the radius**
   (Robart-style hinge points). The nose is a half-cylinder about the hinge
   axis, so it sweeps within its own envelope. Needs the wing TE hollowed to
   the same radius - and at model scale that radius is half the section
   thickness, so the cove would cut the wing trailing edge clean off. Not
   usable here.

2. **Double bevel / V-nose**, the standard for pinned and taped surfaces:
   "double bevels permit deflections of 45 degrees or more" (Model Aviation,
   *Hinging for IMAC and 3-D*). Small and light planes use a tape hinge with
   "the opposite side of the joining control surface edge angled to allow
   movement in both directions" (Wikibooks, *RC Airplane / Control surfaces*).
     - https://www.modelaviation.com/hinging-for-imac-and-3-d
     - https://en.wikibooks.org/wiki/RC_Airplane/Control_surfaces_and_linkages

   This is what we build. The geometry is exact, not a rule of thumb: put the
   apex of a symmetric wedge of half-angle `a` ON the hinge axis, and every
   point of the surface then lies at a polar angle <= `a` from the chord
   plane. Rotating the surface by `d` degrees leaves it at <= a + d, so it
   never crosses into the wing's half-space provided

       d <= 90 deg - a           (a = 45 deg  ->  +/- 45 deg of throw)

   The apex is truncated to a 0.4 mm flat (one nozzle width - a knife edge
   prints as a ragged line and is the first thing to snap) and set back by the
   0.50 mm hinge gap. The truncated corner sits at atan2(0.20, 0.50) = 21.8
   deg from the chord plane, well inside the 45 deg bevel face, so the bevel
   and not the corner sets the limit and the guaranteed throw is the full
   90 - 45 = **45 deg**. Opening the gap from 0.35 to 0.50 mm moved that
   corner from 29.7 deg to 21.8 deg, i.e. it bought margin and cost nothing;
   the module measures what is left and reports it as
   `info["clears_deflection_deg"]`.

* hinge count and spacing: Model Aviation's rule for giant scale is "a hinge
  every 4 inches on the ailerons"; scaled to a 300-600 mm panel that is two
  hinges, and real surfaces are hung *near the ends of their span*, not
  bunched in the middle, so the outboard end cannot flutter. Default stations
  are therefore 20% and 80% of the hinge line - `n_hinges=2`.

* deflection target: +/- 25 to 30 deg is the usual aileron/elevon travel
  (WARG control-surface sizing notes +/- 30 deg max, +/- 25 deg typically
  sufficient). We design to +/- 45 deg and verify by rotating the solid.

WHY A HINGE MOVES, AND WHEN IT GETS SKIPPED
-------------------------------------------
The barrel needs pin + 2 x wall + 2 x radial clearance of section depth. With
the minimum wall that is 1.0 + 1.6 + 0.7 = 3.3 mm of barrel, and the barrel
must sit inside the local section (`section_fill`), so anything thinner than
about 3.8 mm at that station cannot house a hinge at all.

A tapered wing is often below that at the outboard station a builder would
like to use. Rather than drop the hinge - which leaves a long, unsupported,
flutter-prone outboard end - the station WALKS toward the thick end in 4% of
span steps until the section can take a barrel, keeping 22% of span between
hinges so they stay spaced out. Only when there is nowhere left to walk is the
hinge skipped, and both the walk and the skip are reported in `info`. A
bell-planform wing is ~2.2 mm deep at its outboard hinge station and stays
under 3.8 mm over most of the outer panel, so it genuinely ends up with one
hinge - which is the honest answer, not a barrel shrunk until it fuses solid.

EVERY SOLID IS CHECKED BEFORE IT IS RETURNED
--------------------------------------------
`isValid()` and one solid are necessary but not sufficient: OCC will hand back
a valid shape it then refuses to mesh, which shows up as a hole in the STL. So
each result is meshed, the triangle area summed and compared with the BRep
area, and anything under 98.5% coverage is rejected in favour of a simpler
hinge (fewer knuckles, or bevel-only - which is still a flyable tape-hinged
surface). The finished pair is then intersected with each other and the shared
volume reported, so the print-in-place claim is measured rather than argued.

Both checks happen ONCE, on the finished parts, and against copies of them -
see `_finish_pair`. Meshing a shape hangs a triangulation on it that every
later boolean then drags along, which together with a boolean per knuckle is
what used to make this module take 83 seconds per control surface against
about 10 now. The note above `_cut_all` has the measurements.
"""

from __future__ import annotations

import math
from typing import Any, Sequence

import cadquery as cq
from cadquery import Solid, Vector

__all__ = [
    "HINGE_DEFAULTS",
    "hinge_axis_frame",
    "bevel_control_surface",
    "relieve_wing_trailing_edge",
    "clear_surface_ends",
    "print_in_place_hinges",
    "check_deflection_clearance",
    "mesh_coverage",
]


# ---------------------------------------------------------------------------
# Every clearance in one place, with the reason it has that value
# ---------------------------------------------------------------------------

HINGE_DEFAULTS: dict[str, float] = {
    # -- print clearances ---------------------------------------------------
    # Pin OD to bore ID, per side. Top of the 0.2-0.3 mm PLA window because
    # this barrel prints with its axis horizontal: the bore crown is a bridge
    # and the first bridged layer sags into the gap. (3D Printer Stuff,
    # Snapmaker, ozFDM.) PETG: add 0.10-0.15 mm.
    # RAISED 0.30 -> 0.35, and deliberately NOT to 0.5 like the flat gaps. Two
    # things pin this number from opposite sides:
    #   UP: the published per-side window is 0.2-0.3 mm and we sit just above
    #       it, because our barrel prints with its axis parallel to the bed -
    #       the crown of the bore is an unsupported bridge over the pin and the
    #       first bridged layer sags into the gap. 0.35 mm is also Bambu's own
    #       Support/object XY distance, which is the right number for the
    #       HORIZONTAL diameter of the annulus, where the two walls are laid
    #       down side by side in the same layers.
    #   DOWN: a pin/bore pair touches on a LINE, not over an area - that is the
    #       whole reason print-in-place hinges work at 0.3 mm while two flat
    #       faces at 0.3 mm weld solid, and it is the same reasoning as the
    #       45 deg knuckle chamfers below. AND the joint's own play is what
    #       every other clearance has to absorb: the surface can shift 2 x this
    #       value bodily across the axis, so a radial fit bigger than
    #       `cove_running_clearance_mm` would let the elevon nose rub the cove
    #       under its own slop. 0.4 mm+ per side is explicitly "a very loose
    #       joint with lots of play"; at 0.35 mm on a ~1.7-2.0 mm pin the pair
    #       is still a bearing rather than a rattle.
    # This is therefore the ONE dimension in the joint that is under 0.5 mm on
    # purpose, and it sets the measured wing-to-surface minimum distance.
    "radial_clearance_mm": 0.35,
    # Between adjacent knuckle end faces. RAISED 0.20 -> 0.50. 3D Printer
    # Stuff's 0.2 mm is a Z gap on a vertically printed hinge; ours is an XY
    # gap - the knuckles sit side by side along the span, laid down in the same
    # layers - and 0.20 mm is inside one die-swelled 0.45 mm extrusion. These
    # are also flat ANNULAR faces of several square millimetres each, i.e. area
    # contact, not the line contact the pin gets, so they take the flat-face
    # number (0.35 Bambu XY + 0.15 elephant foot) and not the pin number. The
    # 45 deg end chamfers still help. Costs 0.60 mm of hinge length per
    # station, which no station in the fleet notices.
    "axial_clearance_mm": 0.50,
    # Radial gap between one leaf's knuckle OD and the pocket cut for it in
    # the other leaf. RAISED 0.30 -> 0.50. This is not a print fit at all: the
    # mortise floor is the surface the knuckle SWEEPS PAST at every deflection
    # angle, so it is a running clearance and belongs with `hinge_gap_mm`, not
    # with `radial_clearance_mm`. Same XY-gap and elephant-foot argument as the
    # hinge gap. Costs 0.20 mm of extra mortise depth per side, chordwise,
    # where there is plenty of section.
    "body_clearance_mm": 0.50,
    # Wall around the bore. 0.80 mm = two perimeters at a 0.4 mm nozzle, the
    # accepted floor; 1.20 mm = three perimeters, used when the section can
    # afford it because this wall is the loaded one.
    "bore_wall_min_mm": 0.80,
    "bore_wall_pref_mm": 1.20,
    # 45 deg break on each knuckle end (3D Printer Stuff: 0.3-0.5 mm).
    "knuckle_chamfer_mm": 0.40,

    # -- barrel sizing ------------------------------------------------------
    # Fraction of the local section depth the barrel may occupy. Leaves a
    # sliver of skin over the crown of the knuckle instead of bursting
    # through it.
    "section_fill": 0.88,
    # A pin thinner than ~2.5 extrusion widths is a printed hair; a fatter one
    # than this is just mass on a model this size.
    "pin_dia_min_mm": 1.00,
    "pin_dia_max_mm": 2.60,
    # Hard cap on barrel OD - the complaint that started this module was 7 mm
    # knuckles on a 5 mm wing.
    "knuckle_od_max_mm": 5.00,
    # Knuckle length: >= 1.4 x OD, floored at 3 mm (3D Printer Stuff asks for
    # >= 4 mm knuckles; 1.4 x OD lands there for any barrel over ~2.9 mm).
    "knuckle_len_factor": 1.40,
    "knuckle_len_min_mm": 3.00,
    "knuckle_len_max_mm": 6.00,

    # -- control-surface geometry ------------------------------------------
    # Running gap, wing TE to surface nose. Nothing rubs through the throw -
    # and, on a flat-printed wing, nothing WELDS on the plate. RAISED
    # 0.35 -> 0.50: 0.35 mm is Bambu's own XY separability distance with no
    # allowance left for the ~0.15 mm of first-layer elephant foot that a wing
    # printed flat gets right along its hinge line. The cove radius on the
    # wing side is derived from this value (see `_cove_tool`), so the throw
    # does not shrink when the gap opens - it grows very slightly.
    "hinge_gap_mm": 0.50,
    # Spanwise clearance at the INBOARD and OUTBOARD end faces of the control
    # surface. The split that frees the elevon uses a span box, so those two
    # faces are the cut planes and are coincident with the wing that surrounds
    # them - 0.0000 mm, measured. The surface is trimmed back along the hinge
    # axis by this much at each end (`clear_surface_ends`); the WING keeps its
    # full span, so the gap the builder sees is the aerodynamic one and the
    # elevon simply gets 2 x this shorter overall.
    # DOUBLED 0.50 -> 1.00 by the builder's decision: the side gaps ONLY -
    # hinge gap and trailing edge stay put. These two faces are broad, flat
    # and parallel to the wing's (the worst welding geometry), and unlike the
    # hinge gap they do no aerodynamic sealing work, so opening them costs
    # nothing but 2 mm of elevon span. Hinge stations follow automatically
    # (they stand `end_clearance_mm + axial_clearance_mm` inboard of the trim
    # planes), so no knuckle can be orphaned by the wider trim.
    "end_clearance_mm": 1.00,
    # Half-angle of the double bevel. 45 deg -> +/- 45 deg of geometric throw
    # (Model Aviation: "double bevels permit deflections of 45 deg or more").
    "bevel_half_angle_deg": 45.0,
    # The V is truncated to this flat: a knife edge prints ragged and snaps.
    "apex_flat_mm": 0.40,
    # Extra radius on the wing-side cove beyond what the swung apex needs.
    # RAISED 0.25 -> 0.40. This gap only exists once the surface is deflected,
    # so it is not a printability number - but it has to be bigger than
    # `radial_clearance_mm`, because the pin can float that far off centre in
    # its bore and the whole surface goes with it. At 0.25 mm against a 0.35 mm
    # radial fit the elevon could rub its own cove under nothing but joint
    # slop; at 0.40 mm it cannot.
    "cove_running_clearance_mm": 0.40,

    # -- layout -------------------------------------------------------------
    # Hinges live at these fractions of the hinge-line span for n=2; in
    # general linspace(margin, 1-margin, n). Surfaces are hung near the ends.
    "station_edge_margin": 0.20,
    # If a station is too shallow to house a barrel, walk it toward the thick
    # end in these steps rather than dropping the hinge - which is what a
    # builder does with a tapered surface. Give up after `station_walk_max`
    # of the span.
    "station_walk_step": 0.04,
    "station_walk_max": 0.45,
    # Two hinges must stay at least this fraction of the span apart, or they
    # are not "spaced out" any more.
    "station_min_gap": 0.22,
    # Design throw the bevel is checked against.
    "design_deflection_deg": 45.0,

    # -- validation ---------------------------------------------------------
    # Meshed area / BRep area. Below this OCC is dropping faces and the STL
    # has a hole in it.
    "mesh_coverage_min": 0.985,
    "mesh_tolerance_mm": 0.05,
    # Ignore boolean crumbs below this when asking "do the two parts overlap".
    "overlap_tol_mm3": 0.50,
}

_MIN_SECTION_MM = (
    HINGE_DEFAULTS["pin_dia_min_mm"]
    + 2.0 * HINGE_DEFAULTS["bore_wall_min_mm"]
    + 2.0 * HINGE_DEFAULTS["radial_clearance_mm"]
) / HINGE_DEFAULTS["section_fill"]


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------

def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def _params(overrides: dict[str, float] | None) -> dict[str, float]:
    p = dict(HINGE_DEFAULTS)
    if overrides:
        p.update({k: v for k, v in overrides.items() if k in HINGE_DEFAULTS})
    return p


def _heal(solid: Solid) -> Solid:
    """Rebuild pathological boolean trims (ShapeFix). A spline-on-spline cut
    can leave faces the mesher silently refuses to tessellate - the exported
    STL then has a gaping hole even though the BRep reports valid. Keep the
    healed shape only if it is still one valid solid."""
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


# Angular deflection used with the linear one when meshing. 0.25 rad is what
# `exporters.write_stl_verified` ships the STL at, so the mesh measured here is
# the mesh the builder actually gets rather than a much finer one.
_MESH_ANGULAR = 0.25


def _meshed_area(shape, tol: float, angular: float = _MESH_ANGULAR):
    """Total triangulated area, meshed and summed inside OCC.

    A face OCC declines to mesh simply carries no triangulation and adds
    nothing to the total - which is exactly the failure this is looking for.

    Doing it this way instead of `Shape.tessellate()` plus a Python loop over
    the triangles is worth ~20x: a wing panel measured 36.6 s through CadQuery
    (26 s meshing at its 0.1 rad default, 10 s summing 513k triangles one
    `Vector.cross` at a time) against 1.8 s here, for the same coverage figure
    to five decimals. Returns None if anything in the chain is unavailable, so
    the caller can fall back.
    """
    try:
        from OCP.BRep import BRep_Tool
        from OCP.BRepGProp import BRepGProp_MeshProps
        from OCP.BRepMesh import BRepMesh_IncrementalMesh
        from OCP.TopAbs import TopAbs_FACE
        from OCP.TopExp import TopExp_Explorer
        from OCP.TopLoc import TopLoc_Location
        from OCP.TopoDS import TopoDS

        BRepMesh_IncrementalMesh(shape.wrapped, tol, False, angular, True)
        kind = BRepGProp_MeshProps.BRepGProp_MeshObjType_e.Sinert
        total = 0.0
        exp = TopExp_Explorer(shape.wrapped, TopAbs_FACE)
        while exp.More():
            face = TopoDS.Face_s(exp.Current())
            loc = TopLoc_Location()
            tri = BRep_Tool.Triangulation_s(face, loc)
            if tri is not None:
                mp = BRepGProp_MeshProps(kind)
                mp.Perform(tri, loc, face.Orientation())
                total += abs(mp.Mass())
            exp.Next()
        return total
    except Exception:
        return None


def mesh_coverage(shape, tol: float = 0.05) -> float:
    """Meshed triangle area / BRep area.

    OCC will happily return `isValid() == True` for a shape whose faces it
    then declines to tessellate; the only way to catch it is to mesh the thing
    and add the triangles up. 1.0 is perfect, < 0.985 means the STL will have
    a hole in it.

    Meshing WRITES the triangulation onto the shape, and every later boolean
    then drags it along - a panel-sized cut measured seconds clean against the
    better part of a minute tessellated. Anything that will be booleaned again
    must therefore be passed in as a `.copy()`.
    """
    try:
        area = shape.Area()
        if area <= 0.0:
            return 0.0
        total = _meshed_area(shape, tol)
        if total is None:                       # no OCP: sum it in Python
            verts, tris = shape.tessellate(tol, _MESH_ANGULAR)
            total = 0.0
            for a, b, c in tris:
                pa, pb, pc = verts[a], verts[b], verts[c]
                total += 0.5 * (pb.sub(pa)).cross(pc.sub(pa)).Length
        return total / area
    except Exception:
        return 0.0


def _one_solid(solid) -> Solid | None:
    try:
        if solid is None or not solid.isValid():
            return None
        solids = solid.Solids()
        if len(solids) != 1:
            return None
        return solids[0]
    except Exception:
        return None


def _sound(solid, p: dict[str, float]) -> bool:
    """One valid solid AND fully meshable.

    The mesh check runs on a COPY. `tessellate()` hangs a triangulation on the
    shape's TShape and every later boolean then drags it along - see the note
    on batching below - so a shape that is going to be cut again must never be
    tessellated in place.
    """
    s = _one_solid(solid)
    if s is None:
        return False
    return mesh_coverage(s.copy(), p["mesh_tolerance_mm"]) >= p["mesh_coverage_min"]


# ---------------------------------------------------------------------------
# Boolean plumbing - and why it batches
# ---------------------------------------------------------------------------
#
# Every boolean here runs against a full wing panel: one large B-spline skin.
# Two measured rules decide how long a hinge takes.
#
#  * BATCH. OCC pays for the ARGUMENT shape on every call and only marginally
#    for the tools, so cutting the panel once with eight tools costs a small
#    fraction of eight cuts with one tool each. Cuts commute with cuts and
#    fuses with fuses - (S - a) - b is exactly S - (a u b) - so as long as
#    every cut still happens before every fuse the batched result is the same
#    solid, not an approximation of it.
#  * NEVER TESSELLATE BEFORE THE LAST BOOLEAN. `tessellate()` writes a
#    triangulation onto the shared TShape and every subsequent boolean carries
#    it along; the same cut measured seconds on a clean solid and the better
#    part of a minute on a tessellated one. The mesh check therefore happens
#    ONCE, at the end, on a `.copy()` (copied without the triangulation), and
#    the solids handed back are clean - the caller goes on fusing parts onto
#    them and would otherwise pay the tax all over again.
#  * FEED EACH BOOLEAN A COPY. A boolean result keeps references to the
#    sub-shapes of everything that went into it, and the next boolean walks
#    all of it: fusing the knuckles into the freshly cut panel measured 4.96 s
#    straight and 1.62 s through `BRepBuilderAPI_Copy` first, for a
#    bit-identical solid (same volume, area, face count). `copy()` is a
#    fraction of a millisecond even on the whole panel, so it is always worth
#    it - and it protects the caller's solids from being edited underneath
#    them, since OCC's booleans are not non-destructive by default.

def _cut_all(solid: Solid, tools: Sequence[Solid]) -> Solid | None:
    """Every tool removed in ONE boolean, then healed.

    Returns the healed shape WITHOUT checking that it is a single solid: that
    check costs a full BRepCheck pass over a spline skin, and intermediate
    states do not need it - `_finish_pair` checks the finished parts once.
    """
    live = [t for t in tools if t is not None]
    if not live:
        return solid
    try:
        return _heal(solid.copy().cut(*live))
    except Exception:
        return None


def _fuse_all(solid: Solid, tools: Sequence[Solid]) -> Solid | None:
    """Every tool added in ONE boolean, then healed. See `_cut_all`."""
    live = [t for t in tools if t is not None]
    if not live:
        return solid
    try:
        return _heal(solid.copy().fuse(*live))
    except Exception:
        return None


def _cut(solid: Solid, tool: Solid) -> Solid | None:
    out = _cut_all(solid, [tool])
    return None if out is None else _one_solid(out)


def _fuse(solid: Solid, tool: Solid) -> Solid | None:
    out = _fuse_all(solid, [tool])
    return None if out is None else _one_solid(out)


def _prism(profile: Sequence[tuple[float, float]], origin: Vector,
           x_dir: Vector, normal: Vector, length: float) -> Solid:
    """Extrude a 2D polygon along `normal`. The polygon is given in the local
    frame (local x = `x_dir`, local y = normal x x_dir)."""
    plane = cq.Plane(origin=origin, xDir=x_dir, normal=normal)
    wp = cq.Workplane(plane).polyline(list(profile)).close().extrude(length)
    return wp.val()


def _extent_along(shape, origin: Vector, u: Vector,
                  span: float) -> tuple[float, float]:
    """How far the shape reaches along `u`, measured from `origin`, as
    (lo, hi) with a millimetre of margin - and always covering 0..span.

    Used to size the bevel tool. The bevel has to run past both ends of the
    hinge line over the whole surface, but no further: a tool much larger than
    its feature makes OCC test it against far more skin than it can possibly
    touch.
    """
    try:
        bb = shape.BoundingBox()
        ts = [(x - origin.x) * u.x + (y - origin.y) * u.y + (z - origin.z) * u.z
              for x in (bb.xmin, bb.xmax)
              for y in (bb.ymin, bb.ymax)
              for z in (bb.zmin, bb.zmax)]
        return min(min(ts) - 1.0, -1.0), max(max(ts) + 1.0, span + 1.0)
    except Exception:
        return -10.0, span + 10.0


def _thickness_pair(thickness) -> tuple[float, float]:
    if isinstance(thickness, (tuple, list)) and len(thickness) >= 2:
        return float(thickness[0]), float(thickness[1])
    t = float(thickness)
    return t, t


# ---------------------------------------------------------------------------
# The hinge frame
# ---------------------------------------------------------------------------

def hinge_axis_frame(p_in: Vector, p_out: Vector) -> tuple[Vector, Vector, float]:
    """(unit_axis, aft_normal, span) for the hinge line `p_in` -> `p_out`.

    `aft_normal` is the in-plan unit vector perpendicular to the hinge line
    pointing aft (+x in the AeroForge frame, which runs aft from the nose
    datum). Together with the axis it spans the plane the bevel is cut in.
    A degenerate line (span ~ 0) raises ValueError.
    """
    d = Vector(p_out.x - p_in.x, p_out.y - p_in.y, p_out.z - p_in.z)
    span = d.Length
    if span < 1e-6:
        raise ValueError("hinge line has zero length")
    u = d.multiply(1.0 / span)

    aft = Vector(-u.y, u.x, 0.0)
    if aft.Length < 1e-9:                      # hinge line runs fore-and-aft
        aft = Vector(1.0, 0.0, 0.0)
    aft = aft.multiply(1.0 / aft.Length)
    if aft.x < 0.0:
        aft = aft.multiply(-1.0)
    return u, aft, span


def _bevel_geometry(p: dict[str, float]) -> tuple[float, float, float, float]:
    """(setback, apex_half_flat, half_angle_rad, guaranteed_deflection_deg).

    The guaranteed throw is 90 deg minus the largest polar angle any point of
    the bevelled nose subtends at the hinge axis. For a clean V that is just
    the half-angle; truncating the apex to a flat and setting it back by the
    hinge gap pushes the corner to a slightly steeper angle, and that corner -
    not the bevel face - is what sets the limit.
    """
    s0 = p["hinge_gap_mm"]
    a_f = 0.5 * p["apex_flat_mm"]
    alpha = math.radians(_clamp(p["bevel_half_angle_deg"], 15.0, 75.0))
    phi_corner = math.degrees(math.atan2(a_f, s0)) if s0 > 1e-9 else 90.0
    phi_max = max(math.degrees(alpha), phi_corner)
    return s0, a_f, alpha, max(0.0, 90.0 - phi_max)


# ---------------------------------------------------------------------------
# Bevelling the moving side
# ---------------------------------------------------------------------------

def _bevel_tool(surface: Solid, p_in: Vector, p_out: Vector, thickness,
                p: dict[str, float]) -> Solid | None:
    """The prism whose removal leaves the double bevel.

    Split out from `bevel_control_surface` so the caller can put it in the
    same boolean as the hinge mortises and bores instead of paying for a
    separate pass over the surface.
    """
    u, aft, span = hinge_axis_frame(p_in, p_out)
    t_in, t_out = _thickness_pair(thickness)
    s0, a_f, alpha, _ = _bevel_geometry(p)

    # Reach: comfortably past the surface in both directions. The prism only
    # ever removes material, so being generous is free.
    reach = 4.0 * max(t_in, t_out, 4.0) + 10.0
    tan_a = math.tan(alpha)
    x_flare = s0 + (reach - a_f) / max(tan_a, 1e-6)

    # Region to REMOVE, in (aft, vertical): everything forward of the V.
    profile = [
        (-reach, reach),
        (x_flare, reach),
        (s0, a_f),
        (s0, -a_f),
        (x_flare, -reach),
        (-reach, -reach),
    ]
    # Run the bevel past both ends of the hinge line, over the surface's whole
    # span: the hinge line is only where the KNUCKLES go, and an un-bevelled
    # end binds on the wing and makes the hinge decorative. Just past the
    # surface, though - not the old bounding-box diagonal at each end, which
    # built a metre-long tool for a 380 mm feature.
    lo, hi = _extent_along(surface, p_in, u, span)
    try:
        return _prism(profile, p_in.add(u.multiply(lo)), aft, u, hi - lo)
    except Exception:
        return None


def bevel_control_surface(surface: Solid, p_in: Vector, p_out: Vector,
                          thickness, *, clearance: float | None = None,
                          params: dict[str, float] | None = None) -> Solid:
    """Cut the double bevel (V-nose) into the control surface leading edge.

    The surface arrives square-nosed, straight off the split that freed it
    from the wing, and in that state it binds on the wing trailing edge at
    about one degree of deflection. This gives it the standard RC double
    bevel: a symmetric wedge whose apex sits ON the hinge axis, truncated to a
    0.4 mm flat and set back by the hinge gap.

    DEFLECTION CLEARED: with the default 45 deg half-angle, 0.40 mm flat and
    0.50 mm gap, every point of the nose lies within 45.0 deg of the chord
    plane (the truncated corner is at atan2(0.20, 0.50) = 21.8 deg, inside the
    bevel face, so the face sets the limit), and the surface swings
    **+/- 45 deg** before any part of it crosses the hinge plane into the
    wing's half-space - comfortably past the +/- 25 to 30 deg an elevon is ever
    asked for. Widening the gap from 0.35 mm did not cost throw: the cove
    radius on the wing side is derived from the same gap, so the relief opens
    with it. Call `_bevel_geometry` (or read `info["clears_deflection_deg"]`)
    for the number under non-default parameters.

    `thickness` is the section depth at the hinge line - a float, or
    (inboard, outboard). `clearance` overrides the hinge gap.
    """
    p = _params(params)
    if clearance is not None:
        p["hinge_gap_mm"] = float(clearance)

    tool = _bevel_tool(surface, p_in, p_out, thickness, p)
    if tool is None:
        return surface
    out = _cut(surface, tool)
    return out if out is not None else surface


# ---------------------------------------------------------------------------
# Relieving the fixed side
# ---------------------------------------------------------------------------

def _cove_tool(p_in: Vector, p_out: Vector, thickness,
               p: dict[str, float]) -> Solid | None:
    """The cylinder (or cone, on a tapered section) whose removal leaves the
    coved trailing edge. Split out of `relieve_wing_trailing_edge` so it can
    be batched with the hinge mortises into one cut against the panel."""
    u, _aft, span = hinge_axis_frame(p_in, p_out)
    t_in, t_out = _thickness_pair(thickness)
    s0, a_f, _alpha, _ = _bevel_geometry(p)

    swung = math.hypot(s0, a_f)                 # apex corner radius
    need = swung + p["cove_running_clearance_mm"]
    # Never take more than 45% of the local depth or the TE is cut in two.
    r_in = _clamp(need, 0.2, max(0.45 * t_in, 0.2))
    r_out = _clamp(need, 0.2, max(0.45 * t_out, 0.2))

    pad = 5.0
    # Extrapolate the radii over the padded ends so the cove does not step.
    k = (r_out - r_in) / max(span, 1e-6)
    r0 = max(0.15, r_in - k * pad)
    r1 = max(0.15, r_out + k * pad)
    try:
        if abs(r1 - r0) < 1e-6:
            return Solid.makeCylinder(r0, span + 2.0 * pad,
                                      p_in.sub(u.multiply(pad)), u)
        return Solid.makeCone(r0, r1, span + 2.0 * pad,
                              p_in.sub(u.multiply(pad)), u)
    except Exception:
        return None


def relieve_wing_trailing_edge(wing: Solid, p_in: Vector, p_out: Vector,
                               thickness, *, clearance: float | None = None,
                               params: dict[str, float] | None = None) -> Solid:
    """Cove the wing trailing edge on the hinge line.

    The matching relief for `bevel_control_surface`. A cylindrical cove
    centred on the hinge axis - the classic coved TE that goes with a centre
    hinge - sized to swallow the swung path of the truncated bevel apex plus a
    running clearance.

    It is deliberately NOT a full half-thickness radius: a cove of radius
    t/2 would remove the entire trailing edge, because the wing's own section
    is only t deep there. Being rotationally symmetric about the hinge axis,
    the cove clears the surface at every deflection angle equally, so it never
    becomes the thing that limits the throw.

    `clearance` overrides the running clearance added to the cove radius.
    """
    p = _params(params)
    if clearance is not None:
        p["cove_running_clearance_mm"] = float(clearance)

    tool = _cove_tool(p_in, p_out, thickness, p)
    if tool is None:
        return wing
    out = _cut(wing, tool)
    return out if out is not None else wing


# ---------------------------------------------------------------------------
# Clearing the spanwise ends of the moving side
# ---------------------------------------------------------------------------

def _end_tools(surface: Solid, p_in: Vector, p_out: Vector,
               p: dict[str, float]) -> list[Solid]:
    """The two slabs whose removal stands the surface's END faces clear.

    Split out of `clear_surface_ends` for the same reason as `_bevel_tool` and
    `_cove_tool`: they ride along in the surface's existing single cut instead
    of buying a second full pass over it.

    The slabs are sized off the surface's own bounding box rather than made
    arbitrarily large - a tool much bigger than its feature makes OCC test it
    against far more skin than it can possibly touch.
    """
    e = float(p.get("end_clearance_mm", 0.0))
    if e <= 1e-6:
        return []
    u, aft, span = hinge_axis_frame(p_in, p_out)
    if span <= 4.0 * e:                          # nothing worth trimming
        return []

    # Chordwise and vertical reach of the surface, in the prism's own frame
    # (local x = aft). The vertical axis is taken symmetric because the local
    # y of the plane is u x aft, whose sign depends on the panel handedness.
    lo_a, hi_a = _extent_along(surface, p_in, aft, 0.0)
    lo_z, hi_z = _extent_along(surface, p_in, Vector(0.0, 0.0, 1.0), 0.0)
    vz = max(abs(lo_z), abs(hi_z), 1.0)
    profile = [(lo_a, -vz), (hi_a, -vz), (hi_a, vz), (lo_a, vz)]

    lo_u, hi_u = _extent_along(surface, p_in, u, span)
    out: list[Solid] = []
    try:
        # inboard: everything with axis coordinate < e
        out.append(_prism(profile, p_in.add(u.multiply(lo_u)), aft, u,
                          e - lo_u))
        # outboard: everything with axis coordinate > span - e
        out.append(_prism(profile, p_in.add(u.multiply(span - e)), aft, u,
                          hi_u - (span - e)))
    except Exception:
        return []
    return out


def clear_surface_ends(surface: Solid, p_in: Vector, p_out: Vector,
                       params: dict[str, float] | None = None) -> Solid:
    """Trim the control surface back along the hinge axis at BOTH ends.

    The split that frees an elevon intersects the airframe with a half-space
    AND a span box, so the surface's inboard and outboard end faces ARE the cut
    planes: they share a surface with the wing that surrounds them. Measured on
    a real optimized swept wing that is 0.0000 mm over two dozen contact
    solutions, and on the plate those two broad parallel faces fuse solid - the
    elevon comes off the printer as part of the wing.

    This takes `end_clearance_mm` (default 0.50 mm) off each end of the
    SURFACE, not off the wing. That is deliberate: the wing keeps its full span
    and its recorded geometry, and the gap the builder ends up looking at is
    the aerodynamic one at the ends of the elevon, which is where it belongs.
    The elevon loses 2 x `end_clearance_mm` of span - 1.0 mm on a ~370 mm
    surface, about 0.3%, far below anything the aero model resolves.

    Nothing here can orphan a hinge: `print_in_place_hinges` keeps every
    station's knuckles at least `end_clearance_mm + axial_clearance_mm` inboard
    of the trim planes, so a knuckle is always fully inside the mortise cut for
    it and the mortise is always fully on the trimmed surface.

    Returns the trimmed solid, or the input unchanged if the cut refuses.
    """
    p = _params(params)
    tools = _end_tools(surface, p_in, p_out, p)
    if not tools:
        return surface
    out = _one_solid(_cut_all(surface, tools))
    return out if out is not None else surface


# ---------------------------------------------------------------------------
# Barrel sizing
# ---------------------------------------------------------------------------

def _size_barrel(t_local: float, p: dict[str, float]) -> dict[str, float] | None:
    """Pin / bore / OD for a hinge in a section `t_local` deep, or None if the
    section cannot house one.

    Everything is driven from the local depth, not from a global constant:
    a wing tapers hard and a single knuckle size taken from the root bursts
    through the skin at the tip, while one taken from the tip is a hair.
    """
    od_cap = min(p["section_fill"] * t_local, p["knuckle_od_max_mm"])

    for wall in (p["bore_wall_pref_mm"], p["bore_wall_min_mm"]):
        pin = od_cap - 2.0 * wall - 2.0 * p["radial_clearance_mm"]
        if pin < p["pin_dia_min_mm"]:
            continue
        pin = min(pin, p["pin_dia_max_mm"])
        bore = pin + 2.0 * p["radial_clearance_mm"]
        od = bore + 2.0 * wall
        return {
            "pin_dia_mm": pin,
            "bore_dia_mm": bore,
            "knuckle_od_mm": od,
            "bore_wall_mm": wall,
        }
    return None


def _chamfered_knuckle(r: float, length: float, base: Vector, u: Vector,
                       chamfer: float) -> Solid:
    """Cylinder with a 45 deg break on each end, so neighbouring knuckles meet
    on a line rather than a plane and pop free with a thumb-push."""
    c = _clamp(chamfer, 0.0, 0.35 * length)
    if c < 0.05 or r - c < 0.25:
        return Solid.makeCylinder(r, length, base, u)
    try:
        a = Solid.makeCone(r - c, r, c, base, u)
        b = Solid.makeCylinder(r, length - 2.0 * c, base.add(u.multiply(c)), u)
        d = Solid.makeCone(r, r - c, c,
                           base.add(u.multiply(length - c)), u)
        return _heal(a.fuse(b).fuse(d))
    except Exception:
        return Solid.makeCylinder(r, length, base, u)


# ---------------------------------------------------------------------------
# The hinge itself
# ---------------------------------------------------------------------------

def _stations(n: int, span: float, margin: float) -> list[float]:
    n = max(0, int(n))
    if n == 0:
        return []
    if n == 1:
        return [0.5]
    m = _clamp(margin, 0.05, 0.45)
    return [m + (1.0 - 2.0 * m) * i / (n - 1) for i in range(n)]


def _walk_to_material(f_want: float, t_at, taken: list[dict[str, Any]],
                      p: dict[str, float]):
    """Find the station nearest `f_want` that can actually house a barrel.

    A tapered surface is often too shallow at the 20/80 stations a builder
    would like to use. Rather than drop the hinge - which leaves a long,
    flexible, flutter-prone overhang - walk it toward the THICK end until the
    section can take a barrel, keeping `station_min_gap` of span between
    hinges so they stay spaced out. Returns (fraction, sizes|None, moved).
    """
    step = max(0.01, p["station_walk_step"])
    reach = max(0.0, p["station_walk_max"])
    thick_end = 0.0 if t_at(0.0) >= t_at(1.0) else 1.0
    direction = 1.0 if thick_end > f_want else -1.0
    gap = p["station_min_gap"]

    n = int(reach / step) + 1
    for i in range(n):
        f = _clamp(f_want + direction * step * i, 0.05, 0.95)
        if any(abs(f - o["fraction"]) < gap for o in taken):
            continue
        sizes = _size_barrel(t_at(f), p)
        if sizes is not None:
            return f, sizes, i > 0
        if abs(f - thick_end) < 1e-9:
            break
    return f_want, None, False


def _mortise(p_in: Vector, u: Vector, aft: Vector, y0: float, length: float,
             depth: float, side: float, reach: float) -> Solid:
    """A rectangular notch for the other leaf's knuckle, cut back `depth` from
    the hinge plane over `length` of span.

    A plain cylindrical pocket looks tidier but leaves a sliver of skin over
    its crown - on a 4 mm section that sliver is 0.3 mm thick, thinner than
    one extrusion, so the slicer either drops it or prints stringy rubbish.
    A mortise takes the sliver away and leaves the flat-bottomed notch a piano
    hinge actually has.

    It is still safe under rotation: everything the leaf keeps at this span
    lies further than `depth` from the axis, and the mating knuckle never
    leaves a cylinder of radius `depth` about that axis, so the clearance
    holds at every deflection angle. `side` is +1 to notch the aft part
    (control surface) or -1 to notch the forward part (wing).
    """
    lo, hi = (-reach, depth) if side > 0 else (-depth, reach)
    profile = [(lo, -reach), (hi, -reach), (hi, reach), (lo, reach)]
    return _prism(profile, p_in.add(u.multiply(y0)), aft, u, length)


def _station_tools(st: dict[str, Any], u: Vector, aft: Vector, p_in: Vector,
                   p: dict[str, float]) -> dict[str, Any] | None:
    """Every primitive one hinge needs, in one place.

    Both build paths - the batched one and the careful one-station-at-a-time
    fallback - take their geometry from here, so there is a single definition
    of where the knuckles, the pin, the bore and the mortises sit.
    """
    r_kn = 0.5 * st["knuckle_od_mm"]
    r_bore = 0.5 * st["bore_dia_mm"]
    r_pin = 0.5 * st["pin_dia_mm"]
    r_pocket = r_kn + p["body_clearance_mm"]
    g = p["axial_clearance_mm"]
    kl = st["knuckle_length_mm"]
    y0 = st["hinge_start_mm"]
    total = st["hinge_length_mm"]

    base = lambda y: p_in.add(u.multiply(y))    # noqa: E731

    seg1 = (y0, kl)                              # wing
    seg2 = (y0 + kl + g, kl)                     # surface barrel
    seg3 = (y0 + 2.0 * (kl + g), kl)             # wing

    try:
        wing_kn = [
            _chamfered_knuckle(r_kn, seg1[1], base(seg1[0]), u,
                               p["knuckle_chamfer_mm"]),
            _chamfered_knuckle(r_kn, seg3[1], base(seg3[0]), u,
                               p["knuckle_chamfer_mm"]),
        ]
        surf_kn = _chamfered_knuckle(r_kn, seg2[1], base(seg2[0]), u,
                                     p["knuckle_chamfer_mm"])
        # Mortises for the other leaf's knuckle: cut back `r_pocket` from the
        # hinge plane, opened by the axial clearance at each end.
        # Every mortise is opened by the axial clearance at BOTH ends, so no
        # knuckle end face ever lands flush against the other leaf.
        reach = 6.0 * r_pocket + 20.0
        pocket_wing = [                       # notches in the SURFACE
            _mortise(p_in, u, aft, seg1[0] - g, seg1[1] + 2.0 * g, r_pocket,
                     +1.0, reach),
            _mortise(p_in, u, aft, seg3[0] - g, seg3[1] + 2.0 * g, r_pocket,
                     +1.0, reach),
        ]
        pocket_surf = _mortise(p_in, u, aft, seg2[0] - g, seg2[1] + 2.0 * g,
                               r_pocket, -1.0, reach)   # notch in the WING
        pin = Solid.makeCylinder(r_pin, total, base(y0), u)
        bore = Solid.makeCylinder(r_bore, total + 2.0, base(y0 - 1.0), u)
    except Exception:
        return None

    return {
        "wing_kn": wing_kn,
        "surf_kn": surf_kn,
        "pocket_wing": pocket_wing,
        "pocket_surf": pocket_surf,
        "pin": pin,
        "bore": bore,
        "start_mm": y0,
        "end_mm": y0 + total,
    }


def _batched_tools(t: dict[str, Any]) -> dict[str, list] | None:
    """One station's primitives folded into the batched build's tool lists.

    The order the tools are applied in is exactly the order `_build_one` uses
    - wing: cut the mortise, add the knuckles and pin; surface: cut the
    mortises, add the barrel, bore it - just gathered so each stage is a
    single boolean however many hinges there are.

    The only shape assembled early is the wing's leaf: its two knuckles and
    its pin are one printed piece, so they are fused HERE, where the shapes
    are a few millimetres across, instead of making the panel pay for three
    separate fuses. The fused leaf is the same solid to four decimal places
    and the finished panel comes out with the same volume, area and face
    count either way.
    """
    try:
        leaf = _heal(t["wing_kn"][0].fuse(t["pin"], t["wing_kn"][1]))
    except Exception:
        return None
    return {
        "wing_cut": [t["pocket_surf"]],
        "wing_fuse": [leaf],
        "surf_cut": list(t["pocket_wing"]),
        "surf_fuse": [t["surf_kn"]],
        "surf_bore": [t["bore"]],
    }


def _stations_collide(tools: list[dict[str, Any]],
                      p: dict[str, float]) -> bool:
    """Do two hinges reach into each other along the span?

    Batching reorders the work - all cuts, then all fuses - which is only the
    same solid while the stations are disjoint. They are, by construction
    (`station_min_gap` keeps them 22% of span apart), but a very short panel
    could in principle clamp two hinges into contact, and then a mortise for
    one would eat a knuckle of the other. Cheap to check, so check.
    """
    g = p["axial_clearance_mm"] + 0.5
    iv = sorted((t["start_mm"] - g, t["end_mm"] + g) for t in tools)
    return any(b[0] < a[1] for a, b in zip(iv, iv[1:]))


def _build_one(wing: Solid, surface: Solid, st: dict[str, Any],
               u: Vector, aft: Vector, p_in: Vector, p: dict[str, float]):
    """Apply one hinge, one boolean at a time. Returns (wing, surface) or None
    if any boolean in the chain refuses - the caller then drops this station
    rather than shipping a broken solid.

    This is the FALLBACK path: it costs one full pass over the panel per
    boolean and is only used when the batched build refuses, where being able
    to drop a single bad station is worth the time.
    """
    t = _station_tools(st, u, aft, p_in, p)
    if t is None:
        return None
    wing_kn, surf_kn = t["wing_kn"], t["surf_kn"]
    pocket_wing, pocket_surf = t["pocket_wing"], t["pocket_surf"]
    pin, bore = t["pin"], t["bore"]

    # --- wing: pocket for the surface's barrel, then its own knuckles + pin
    w = wing
    w = _cut(w, pocket_surf)
    if w is None:
        return None
    for kn in wing_kn:
        w = _fuse(w, kn)
        if w is None:
            return None
    w = _fuse(w, pin)
    if w is None:
        return None

    # --- surface: pockets for the wing's knuckles, its barrel, then the bore
    s = surface
    for pk in pocket_wing:
        s = _cut(s, pk)
        if s is None:
            return None
    s = _fuse(s, surf_kn)
    if s is None:
        return None
    s = _cut(s, bore)
    if s is None:
        return None

    return w, s


def _bevel_only(wing: Solid, surface: Solid, bevel_tool, cove_tool,
                p: dict[str, float], info: dict[str, Any],
                end_tools: Sequence[Solid] = ()):
    """Bevel, end trim and cove alone: a legitimate tape-hinged surface, and
    the state the careful path starts from when the batched build refuses.

    The end trim goes in the SAME cut as the bevel - it is the one clearance
    that must survive every fallback, because without it the elevon is welded
    to the wing at both ends whatever else happens.
    """
    w, s = wing, surface
    surf_cut = [t for t in ([bevel_tool] + list(end_tools)) if t is not None]
    if surf_cut:
        cand = _one_solid(_cut_all(surface, surf_cut))
        if cand is not None and _sound(cand, p):
            s = cand
        elif end_tools:                          # bevel may be the problem
            cand = _one_solid(_cut_all(surface, list(end_tools)))
            if cand is not None and _sound(cand, p):
                s = cand
                info["warnings"].append("bevel rejected (mesh/solid check); "
                                        "end clearance kept")
            else:
                info["warnings"].append(
                    "bevel + end clearance rejected (mesh/solid check)")
        else:
            info["warnings"].append("bevel rejected (mesh/solid check)")
    if cove_tool is not None:
        cand = _one_solid(_cut_all(wing, [cove_tool]))
        if cand is not None and _sound(cand, p):
            w = cand
        else:
            info["warnings"].append("TE relief rejected (mesh/solid check)")
    if s is not surface or w is not wing:
        info["mode"] = "bevel_only"
    return w, s


def _finish_pair(w, s, p: dict[str, float]):
    """The one place the finished parts are checked: one valid solid each, and
    a mesh that actually covers them.

    Both checks happen HERE and nowhere earlier. `isValid` costs a full
    BRepCheck pass over a spline skin and meshing costs more, but the real
    reason to do it once at the end is that meshing writes a triangulation
    onto the shape which every later boolean then drags along - so the mesh
    check runs on a `.copy()` and the solids that go back to the caller are
    left clean for the parts it still has to fuse on.

    Returns (wing, surface, coverage_wing, coverage_surface) or None.
    """
    if w is None or s is None:
        return None
    ws, ss = _one_solid(w), _one_solid(s)
    if ws is None or ss is None:
        return None
    # Hand the caller a copy: it is what carries no triangulation and none of
    # the sub-shape baggage of the booleans that built it, and the caller has
    # a whole airframe still to fuse these into.
    ws, ss = ws.copy(), ss.copy()
    cov_w = mesh_coverage(ws.copy(), p["mesh_tolerance_mm"])
    cov_s = mesh_coverage(ss.copy(), p["mesh_tolerance_mm"])
    if min(cov_w, cov_s) < p["mesh_coverage_min"]:
        return None
    return ws, ss, cov_w, cov_s


def print_in_place_hinges(wing: Solid, surface: Solid, p_in: Vector,
                          p_out: Vector, t_in: float, t_out: float, *,
                          n_hinges: int = 2,
                          params: dict[str, float] | None = None,
                          bevel: bool = True,
                          relieve: bool = True,
                          end_clear: bool = True):
    """Hang `surface` off `wing` on print-in-place hinges.

    Returns `(wing, surface, info)`.

    `p_in` / `p_out` are the ends of the hinge line, `t_in` / `t_out` the
    section depth there. `n_hinges` defaults to **2**, placed at 20% and 80%
    of the hinge line - real surfaces are hung near the ends of their span so
    the outboard end cannot flutter, not bunched in the middle.

    Each hinge is three knuckles: wing / surface / wing. The **pin is printed
    in place** as part of the wing and runs the full hinge length through the
    surface's barrel with `radial_clearance_mm` of gap, so the pair
    articulates straight off the bed with no wire to thread. Wing and surface
    share no volume at all - the engagement is a rod threaded through a ring -
    which `info["engagement"]` reports explicitly.

    The surface's spanwise END faces are trimmed back by `end_clearance_mm`
    (`end_clear=False` to skip). Straight off the split those faces are the cut
    planes themselves and are coincident with the wing - 0.0000 mm, measured -
    and on a printed airframe they weld. See `clear_surface_ends`.

    A station whose section is too shallow to house a barrel is SKIPPED, with
    the reason recorded in `info["skipped"]`, rather than shrunk into
    something that prints as one lump. If a boolean or the mesh check fails
    the module falls back: fewer hinges, then bevel-only (a legitimate
    tape-hinged surface), then the untouched inputs.
    """
    p = _params(params)
    info: dict[str, Any] = {
        "mode": "none",
        "n_hinges_requested": int(n_hinges),
        "n_hinges_built": 0,
        "stations": [],
        "skipped": [],
        "warnings": [],
        "radial_clearance_mm": p["radial_clearance_mm"],
        "axial_clearance_mm": p["axial_clearance_mm"],
        "body_clearance_mm": p["body_clearance_mm"],
        "hinge_gap_mm": p["hinge_gap_mm"],
        "end_clearance_mm": p["end_clearance_mm"] if end_clear else 0.0,
        "ends_trimmed": False,
        "bevel_half_angle_deg": p["bevel_half_angle_deg"],
        "apex_flat_mm": p["apex_flat_mm"],
        "min_section_for_hinge_mm": round(_MIN_SECTION_MM, 2),
        "pin_printed_in_place": False,
        "pin_owner": "wing",
        "barrel_owner": "surface",
    }

    try:
        u, aft, span = hinge_axis_frame(p_in, p_out)
    except ValueError:
        info["warnings"].append("degenerate hinge line")
        return wing, surface, info

    _s0, _af, _al, throw = _bevel_geometry(p)
    info["clears_deflection_deg"] = round(throw, 1)
    info["span_mm"] = round(span, 2)

    # ---- bevel + cove: this is what actually lets the surface move ---------
    # Built as TOOLS, not as finished cuts. The bevel, the cove, the mortises
    # and the bores all end up in one boolean per part below, which is the
    # difference between a hinge taking seconds and taking minutes.
    bevel_tool = _bevel_tool(surface, p_in, p_out, (t_in, t_out), p) \
        if bevel else None
    cove_tool = _cove_tool(p_in, p_out, (t_in, t_out), p) if relieve else None
    # The spanwise end trim. Rides in the same cut as the bevel; without it the
    # surface's end faces sit ON the wing's cut planes and the pair prints as
    # one lump however good the hinge is.
    end_tools = _end_tools(surface, p_in, p_out, p) if end_clear else []
    if bevel and bevel_tool is None:
        info["warnings"].append("bevel tool could not be built")
    if relieve and cove_tool is None:
        info["warnings"].append("TE relief tool could not be built")
    if end_clear and not end_tools:
        info["warnings"].append("end-clearance tools could not be built")
        info["end_clearance_mm"] = 0.0

    swung = math.hypot(_s0, _af)
    info["cove_radius_mm"] = round(
        min(swung + p["cove_running_clearance_mm"],
            0.45 * min(t_in, t_out)), 3)

    # ---- lay the stations out ---------------------------------------------
    # Thick end first, so the hinge that is certain to fit claims its place
    # before the marginal one goes looking for somewhere to sit.
    fracs = _stations(n_hinges, span, p["station_edge_margin"])
    thick_first = sorted(fracs, key=lambda f: -(t_in + (t_out - t_in) * f))

    def _t_at(f: float) -> float:
        return t_in + (t_out - t_in) * f

    plan: list[dict[str, Any]] = []
    for f_want in thick_first:
        f, sizes, moved = _walk_to_material(f_want, _t_at, plan, p)
        if sizes is None:
            info["skipped"].append({
                "fraction": round(f_want, 3),
                "station_mm": round(f_want * span, 1),
                "section_thickness_mm": round(_t_at(f_want), 2),
                "reason": (f"section {_t_at(f_want):.2f} mm < "
                           f"{_MIN_SECTION_MM:.2f} mm needed for the smallest "
                           f"printable barrel, and no station within "
                           f"{p['station_walk_max']:.0%} of span is deep "
                           f"enough while staying clear of the other hinge"),
            })
            continue
        t_local = _t_at(f)
        kl = _clamp(p["knuckle_len_factor"] * sizes["knuckle_od_mm"],
                    p["knuckle_len_min_mm"], p["knuckle_len_max_mm"])
        g_ax = p["axial_clearance_mm"]
        total = 3.0 * kl + 2.0 * g_ax
        # Keep every knuckle clear of the planes the surface is trimmed back
        # to, so the end trim can never cut a hinge in half. A knuckle inside
        # `edge` of an end would have its mortise (which opens by the axial
        # clearance at each end) run off the trimmed surface and the pin would
        # be left standing in air.
        edge = (p["end_clearance_mm"] if end_tools else 0.0) + max(0.5, g_ax)
        room = span - 2.0 * edge
        if total > min(0.9 * span, room):        # tiny panel: shrink to fit
            kl = max(2.0, (min(0.9 * span, room) - 2.0 * g_ax) / 3.0)
            total = 3.0 * kl + 2.0 * g_ax
        y0 = _clamp(f * span - 0.5 * total, edge,
                    max(edge, span - total - edge))
        st = dict(sizes)
        st.update({
            "fraction": round(f, 3),
            "station_mm": round(f * span, 1),
            "section_thickness_mm": round(t_local, 2),
            "knuckle_length_mm": round(kl, 2),
            "hinge_length_mm": round(total, 2),
            "hinge_start_mm": y0,
            "knuckles": 3,
        })
        if moved:
            st["moved_from_fraction"] = round(f_want, 3)
            info["warnings"].append(
                f"hinge walked inboard {f_want:.2f} -> {f:.2f}: the section "
                f"at {f_want:.2f} is only {_t_at(f_want):.2f} mm deep")
        plan.append(st)
    plan.sort(key=lambda d: d["fraction"])

    if not plan:
        info["warnings"].append("no station could house a barrel")
        w0, s0_ = _bevel_only(wing, surface, bevel_tool, cove_tool, p, info,
                              end_tools)
        info["ends_trimmed"] = bool(end_tools) and s0_ is not surface
        return w0, s0_, info

    # ---- build: the fast path ---------------------------------------------
    # Collect every tool for every station, then spend FIVE booleans however
    # many hinges there are: cut the wing and add its leaves, cut the surface,
    # add its barrels and bore them. The bevel and the cove ride along in the
    # cuts. The sequence is the one `_build_one` walks - all that changes is
    # that each stage happens once instead of once per station.
    batch: dict[str, list] = {"wing_cut": [cove_tool], "wing_fuse": [],
                              "surf_cut": [bevel_tool] + list(end_tools),
                              "surf_fuse": [], "surf_bore": []}
    tooled: list[dict[str, Any]] = []
    for st in plan:
        t = _station_tools(st, u, aft, p_in, p)
        got = None if t is None else _batched_tools(t)
        if got is None:
            tooled = []                      # the careful path will report it
            break
        for k, v in got.items():
            batch[k] += v
        tooled.append(t)

    built: list[dict[str, Any]] = []
    w = s = None
    cov_w = cov_s = 0.0
    if tooled and not _stations_collide(tooled, p):
        wc = _cut_all(wing, batch["wing_cut"])
        sc = _cut_all(surface, batch["surf_cut"])
        sf = None if sc is None else _fuse_all(sc, batch["surf_fuse"])
        done = _finish_pair(
            None if wc is None else _fuse_all(wc, batch["wing_fuse"]),
            None if sf is None else _cut_all(sf, batch["surf_bore"]),
            p)
        if done is not None:
            w, s, cov_w, cov_s = done
            built = list(plan)
            info["ends_trimmed"] = bool(end_tools)

    # ---- build: the careful path ------------------------------------------
    # Only reached when the batch above refused - a boolean failed, the result
    # was not one solid, or it would not mesh. Redo the work one station at a
    # time so a single bad hinge can be dropped instead of losing the lot,
    # then fall back to one hinge, then to bevel-only.
    if not built:
        if tooled:
            info["warnings"].append(
                "batched build rejected; rebuilding one hinge at a time")
        w0, s0_ = _bevel_only(wing, surface, bevel_tool, cove_tool, p, info,
                              end_tools)
        info["ends_trimmed"] = bool(end_tools) and s0_ is not surface
        w, s = w0, s0_
        for st in plan:
            got = _build_one(w, s, st, u, aft, p_in, p)
            if got is None:
                info["skipped"].append({
                    "fraction": st["fraction"],
                    "station_mm": st["station_mm"],
                    "section_thickness_mm": st["section_thickness_mm"],
                    "reason": "boolean chain failed",
                })
                continue
            w_try, s_try = got
            if not (_one_solid(w_try) and _one_solid(s_try)):
                info["skipped"].append({
                    "fraction": st["fraction"],
                    "station_mm": st["station_mm"],
                    "reason": "hinge split the part",
                })
                continue
            w, s = w_try, s_try
            built.append(st)

        if not built:
            info["warnings"].append("every hinge failed; bevel-only fallback")
            return w0, s0_, info

        # ---- the check that matters: will OCC actually mesh it ------------
        done = _finish_pair(w, s, p)
        if done is None:
            info["warnings"].append(
                f"mesh coverage below {p['mesh_coverage_min']}; "
                f"retrying with one hinge")
            if len(built) <= 1:
                return w0, s0_, info
            keep = [max(built, key=lambda d: d["section_thickness_mm"])]
            got = _build_one(w0, s0_, keep[0], u, aft, p_in, p)
            done = None if got is None else _finish_pair(got[0], got[1], p)
            if done is None:
                return w0, s0_, info
            built = keep
        w, s, cov_w, cov_s = done

    # ---- report -----------------------------------------------------------
    # The whole-part shared volume, measured on the actual solids - not argued
    # from the clearances. On copies, because an OCC boolean is free to edit
    # its arguments and these two are about to be handed back.
    overlap = 0.0
    try:
        inter = w.copy().intersect(s.copy())
        overlap = sum(x.Volume() for x in inter.Solids())
    except Exception:
        overlap = float("nan")

    for st in built:
        st.pop("hinge_start_mm", None)
    info["mode"] = "print_in_place"
    info["n_hinges_built"] = len(built)
    info["stations"] = built
    info["pin_printed_in_place"] = True
    info["pin_dia_mm"] = round(min(st["pin_dia_mm"] for st in built), 3)
    info["knuckle_od_mm"] = round(max(st["knuckle_od_mm"] for st in built), 3)
    info["engagement"] = {
        "kind": "pin threaded through barrel (no shared volume)",
        "pin_owner": "wing",
        "barrel_owner": "surface",
        "capture_length_mm": round(
            sum(st["knuckle_length_mm"] for st in built), 2),
        "diametral_fit_mm": round(2.0 * p["radial_clearance_mm"], 3),
        "shared_volume_mm3": round(overlap, 4),
        "parts_separate": bool(overlap <= p["overlap_tol_mm3"]),
    }
    info["mesh_coverage"] = {"wing": round(cov_w, 5),
                             "surface": round(cov_s, 5)}
    # Every designed wing-to-surface gap in one place, so a caller (or a
    # builder) can see at a glance what the tightest thing on the plate is
    # without re-deriving it from the clearances.
    #
    # `on_plate` are the gaps that exist at zero deflection and therefore
    # decide whether the pair comes off the printer articulating. Every flat
    # face-to-face pair in that list is 0.50 mm; the pin/bore annulus is
    # deliberately tighter, because a cylinder touches its bore on a line
    # rather than over an area and because its play is what the running
    # clearances have to absorb. It is the global minimum distance between the
    # two solids and it is meant to be.
    e_used = info["end_clearance_mm"] if info["ends_trimmed"] else 0.0
    on_plate = {
        "leading_edge_gap": round(p["hinge_gap_mm"], 3),
        "spanwise_end_gap": round(e_used, 3),
        "knuckle_to_mortise": round(p["body_clearance_mm"], 3),
        "knuckle_to_knuckle_axial": round(p["axial_clearance_mm"], 3),
        "pin_to_bore_radial": round(p["radial_clearance_mm"], 3),
    }
    info["clearances_mm"] = {
        "on_plate": on_plate,
        "running": {"cove_to_nose_radial":
                    round(p["cove_running_clearance_mm"], 3)},
    }
    flat = {k: v for k, v in on_plate.items()
            if k != "pin_to_bore_radial" and v > 0.0}
    info["min_flat_face_gap_mm"] = round(min(flat.values()), 3) if flat else 0.0
    info["min_designed_gap_mm"] = round(
        min(v for v in on_plate.values() if v > 0.0), 3)
    info["tightest_feature"] = min(
        ((v, k) for k, v in on_plate.items() if v > 0.0))[1]
    if e_used > 0.0:
        info["surface_span_after_trim_mm"] = round(span - 2.0 * e_used, 2)
    # Support material needs one line (~0.42 mm) plus the slicer's
    # support/object XY standoff on each side (Bambu default 0.35 mm), so
    # nothing under ~1.1 mm can be entered. Every gap above is well under it -
    # tree supports cannot get into the joint, and there is nothing to pick out
    # of the hinge after the print.
    info["min_support_enterable_gap_mm"] = 1.12
    info["support_enters_joint"] = bool(max(on_plate.values()) >= 1.12)
    # The longest run of hinge line with nothing holding it. A surface whose
    # outboard hinge had to walk inboard - or was skipped - has a long free
    # end, and that is where flutter starts. Surfaced so the caller can decide
    # to shorten the elevon or deepen the section rather than find out later.
    edges = ([0.0] + [st["station_mm"] for st in built] + [span])
    info["largest_unsupported_run_mm"] = round(
        max(b - a for a, b in zip(edges, edges[1:])), 1)
    return w, s, info


# ---------------------------------------------------------------------------
# Verification helper
# ---------------------------------------------------------------------------

def check_deflection_clearance(wing: Solid, surface: Solid, p_in: Vector,
                               p_out: Vector, angle_deg: float,
                               *, tol_mm3: float = 0.5) -> dict[str, Any]:
    """Rotate the control surface about the hinge axis and measure what it
    hits. This is a geometric test, not an assertion about the design intent:
    it rotates the actual solid and intersects it with the actual wing.

    Returns {"angle_deg", "volume_mm3", "clear"} - `clear` is True when the
    interference is under `tol_mm3` (the pin inside the barrel has clearance,
    so a working hinge measures ~0 at every angle).
    """
    try:
        moved = surface.copy().rotate(p_in, p_out, float(angle_deg))
        inter = wing.copy().intersect(moved)
        vol = sum(x.Volume() for x in inter.Solids())
    except Exception as exc:                     # OCC can refuse a null cut
        return {"angle_deg": float(angle_deg), "volume_mm3": 0.0,
                "clear": True, "note": f"empty intersection ({type(exc).__name__})"}
    return {"angle_deg": float(angle_deg), "volume_mm3": round(vol, 4),
            "clear": bool(vol <= tol_mm3)}
