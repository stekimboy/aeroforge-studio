"""Equipment bay and removable canopy for the flying-wing centre section.

WHAT THIS MODULE IS FOR
-----------------------
An FPV wing is a box with wings on it. The box has to swallow a pack, a stack,
a VTX and a receiver, and the lid has to come off with one hand at the field.
The previous implementation produced a lid that was, on a 553 mm wing, an
81 x 48 mm x 3.7 cm^3 scrap of skin: geometrically a hatch, practically
nothing. This module builds the compartment and the canopy as the two halves
of one joint and *reports the usable inside dimensions*, so the number a
builder cares about ("will my 105 x 34 x 25 mm pack go in?") is answerable.

It deliberately does NOT import ``backend.cad.geometry`` - that would be an
import cycle. The wing arrives by duck typing and only has to expose:

    wing.section(f)      -> object with .chord (mm) and .le (Vector, mm)
    wing.crown_z(f, xc)  -> upper-surface z at span fraction f, chord frac xc
    wing.keel_z(f, xc)   -> lower-surface z, same arguments
    wing.xc_at(f, x_mm)  -> chord fraction of an absolute x station
    wing.half            -> semi-span, mm
    wing.fb              -> body half-width / semi-span
    wing.tc              -> section thickness / chord

All lengths in and out of this module are MILLIMETRES.

HOW A REAL FPV WING DOES IT (and what that dictates here)
---------------------------------------------------------
* The canopy covers most of the centre pod, not a porthole in it. On the
  Skywalker X5 / X8 and the SonicModell AR Wing the hatch is essentially the
  whole top of the centre section, because that is the only way to get a pack
  in and to slide it fore/aft for balance. So the aperture here is the bay's
  own footprint, not an inset panel: `_APERTURE_INSET_MM` is a boolean-safety
  offset, nothing more.
* Retention on moulded wings is a bolt-down canopy (X5) or magnets (X8); the
  3D-printed convention is a latch or tongue at the front and magnets at the
  rear (Painless360's "R/C Plane/Wing Canopy hatch latch", Thingiverse 3624994).
  Magnet pockets (Ø`MAGNET_DIA_MM` x `MAGNET_H_MM`, cut at exactly the
  builder's own 8.15 mm bore) are cut into local pads on
  the seat and under the canopy, one at each end - see `_magnets`.
* Moulded hatches never sit in a straight-sided hole; they sit on a rebate.
  The opening has a lip standing proud inward, the canopy's skirt lands on it,
  and the lid finishes flush with the skin instead of dropping through. That
  lip is `ledge` here, and its top face is the skin offset down by exactly the
  skirt depth, so the mate is a full ring contact rather than a single point.
* The skirt depth is measured DOWN FROM THE SKIN, not to a flat plane. It has
  to be: over a 553 mm wing's centre section the crown climbs about 30 mm from
  the bay's aft end to its nose and falls another 20 mm from the spine to the
  bay's edge, so a canopy cut off at one flat height comes out 48 mm deep at
  the front and 5 mm at the back - a plug that fills the compartment it is
  supposed to open. Cutting the canopy against a copy of the skin dropped by
  `skirt_h` gives the constant-depth shell a vacuum-formed hatch actually is.

CLEARANCES (stated once, here, because they decide whether it works)
--------------------------------------------------------------------
* `LID_CLEARANCE_MM = 0.35` per side (0.70 mm across the joint). FDM
  clearance-fit guidance for "simple box lids" and other parts that must be
  taken on and off repeatedly is 0.3-0.5 mm (Snapmaker, "3D Printing
  Tolerances"); one extrusion width (~0.4 mm on a 0.4 mm nozzle) is the
  classic sliding-fit rule of thumb. 0.35 mm sits in both bands, and the
  engagement is deliberately long (the skirt is `4 x wall`, 8 mm typical, all
  the way round) because a long lip at 0.4 mm beats a short lip at 0.2 mm on a
  warped print.
* `SEAT_CLEARANCE_MM = 0.20` NORMAL TO THE SKIN: the seat is dropped below
  the nominal skirt bottom by `0.20 / cos(local slope)`, so the lid lands
  0.2 mm *recessed* rather than proud. A lid that stands proud is a drag device
  and it rocks; 0.2 mm absorbs the elephant-foot on the seat's first layer and
  the skirt's bottom layer. Measuring it along the normal rather than straight
  down matters wherever the crown is steep - a plain vertical 0.2 mm becomes a
  0.05 mm gap on a 75-degree slope, and the parts then touch.

OPENCASCADE RULES OBEYED HERE (each one is a defect somebody already paid for
- see DECISIONS.md)
-----------------------------------------------------------------------------
* A loft whose sections change WIDTH and HEIGHT at once twists and
  self-intersects. So the cavity is NOT lofted. It is the intersection of a
  vertical prism carrying the variable-width PLAN and a constant-width band
  lofted through the crown/keel profile in Z ONLY. Each shape varies in one
  thing; the boolean does the rest.
* Near-coincident and coincident faces make booleans misbehave: sometimes an
  empty result, sometimes a valid-looking solid that `isValid()` rejects. The
  compartment is therefore assembled as ONE cutter before it touches the
  airframe (its internal walls vanish inside the union), and the magnet lugs
  stop 0.6 mm INSIDE the canopy skin instead of exactly on it.
* `isValid()` does not mean OCC will tessellate the shape. Every cut is
  accepted only if the result still MESHES completely (`tess_ratio` >= 0.985),
  and `build_bay` falls back down a ladder of smaller bays, reporting which
  rung it used, rather than handing back something broken.
* A sealed internal void reports two shells; one you can reach into reports
  one. That is the test that the lid actually opens the bay, and it is
  asserted here rather than eyeballed.
* A cutter's cost is set by its FACE COUNT, and planar faces against big
  B-spline faces are the expensive case. The same cavity built as a 1202-face
  ruled loft took 187 s to cut out of one 553 mm airframe; built from 4-edge
  spline sections (6 faces) it takes about 3 s. Nothing here is ever a polygon
  loft. For the same reason a 142-face polygonal prism costs 46 s where the
  identical shape as a 3-face spline prism costs 2 s.
* MESH NOTHING UNTIL EVERY BOOLEAN IS DONE. `tessellate()` stores a
  triangulation on the shape, OCC booleans share sub-shapes with their
  operands, and a boolean against an already-tessellated shape is far slower:
  measured, the same cut took 2.0 s on a clean airframe and 49.0 s on the same
  airframe after one `tessellate()`. `Shape.copy()` drops the triangulation and
  restores full speed, which is why each rung of the ladder gets a fresh copy.
* Keeping "the largest solid" quietly keeps the WRONG one when a cutter splits
  a feature into a stump and a bigger offcut floating in free air. Cutters are
  sized so they cannot split what they are trimming.
"""

from __future__ import annotations

import math
import os
import time
from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import Any

import numpy as np
from cadquery import Edge, Shape, Solid, Vector, Wire
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.gp import gp_Pnt
from OCP.TopAbs import TopAbs_IN, TopAbs_OUT

# Set AEROFORGE_HATCH_TRACE=1 to print a per-stage timing breakdown. Booleans
# here are the slowest thing in the whole CAD build and their cost is wildly
# non-linear in the cutter's face count, so having the breakdown one env var
# away has already paid for itself several times.
_TRACE = bool(os.environ.get("AEROFORGE_HATCH_TRACE"))
_LAST_REJECT: list[str] = []
# root-principle rear extension: length of the ceiling's rise from the
# hatch seat's underside to the extension's roof (see `below_ceiling`)
_EXT_RAMP_MM = 30.0
# aft magnet pad when the cavity extends: shelf + riser length past the
# aperture's aft wall (user, 2026-08-27: no rib to the rear wall)
_AFT_RISER_LEN_MM = 12.0


@contextmanager
def _stage(name: str):
    if not _TRACE:
        yield
        return
    t = time.perf_counter()
    try:
        yield
    finally:
        print(f"    [hatch] {name:32s} {time.perf_counter() - t:7.2f}s",
              flush=True)

__all__ = [
    "BayResult", "build_bay", "tess_ratio", "tessellates_cleanly",
    "unmeshed_faces",
    "LID_CLEARANCE_MM", "SEAT_CLEARANCE_MM", "MAGNET_DIA_MM", "MAGNET_H_MM",
]

# ---------------------------------------------------------------------------
# Print clearances and fits - see the module docstring for the sources
# ---------------------------------------------------------------------------
LID_CLEARANCE_MM = 0.35      # lid to aperture wall, per side
SEAT_CLEARANCE_MM = 0.20     # seat dropped below the skirt bottom
MAGNET_DIA_MM = 8.15         # pocket bore, AS CUT - the builder's own spec
                             # (2026-08-20): "I plan to put magnets in it,
                             # can you change the diameter to be 8.15
                             # diameter exactly." 8.15 is the number the
                             # printer receives; whatever press allowance
                             # their magnet needs is already inside it, which
                             # is why MAGNET_FIT_MM is zero and must stay
                             # zero - adding a fit on top would betray the
                             # word "exactly". (Was 6.0 + 0.20 for the 6 x 3
                             # N35 disc, the RC hatch default.)
MAGNET_H_MM = 3.0            # pocket depth still takes a 3 mm disc - the
                             # builder specified only the diameter
MAGNET_FIT_MM = 0.0          # see MAGNET_DIA_MM: the 8.15 is the as-cut bore
MAGNET_GAP_MM = 0.40         # air gap between the two discs when seated

# Geometry / boolean-safety constants
# How much the compartment floor is thickened when the standard thin-floor
# bay cut fails (the "raised floor" retry in `_attempt`): the keel+wall
# floor is a ~1.2 mm sliver between two spline lofts, and on some designs it
# lands inside OCC's tolerance-merge range - the cut welds belly to floor
# and deletes both (the builder's 2026-08-21 "large box hole in the bottom
# of the fuselage"). 1.8 mm puts the floor at ~3 mm, comfortably past any
# loft face tolerance, at the cost of a compartment shallower by the same
# amount - only ever paid on a design whose thin floor has already failed
# every other cut strategy.
_FLOOR_RAISE_MM = 1.8
# How far the `core` chunk's floor is lifted off the `below` chunk's when the
# three compartment pieces will not join (the void-fuse retry in `_attempt`).
# `core` and `below` are two INDEPENDENT lofts sharing the same floor curve
# (keel + wall) but different ceilings, so their floor faces are the same
# surface with different parameterizations - and once the rear extension
# lengthens `below`, OCC's fuse can no longer recognise them as one face:
# measured on the bwb at the 675 box, core+below came back INVALID with
# LESS volume than `below` alone, and every rung of the ladder died on it.
# The core's floor never shapes the void - inside the hatch span `below`
# already owns everything under the seat - so lifting it by a millimetre
# leaves the union the same set and takes the coincidence away (measured:
# 0.5, 1.0 and 2.0 mm all fuse to one valid solid of the same volume).
_CORE_FLOOR_LIFT_MM = 1.0
_APERTURE_INSET_MM = 1.2     # aperture inside the cavity wall (non-coincident)
_SEAT_OVERLAP_MM = 1.2       # seat protruding inboard of the skirt's inner face
_MIN_SKIRT_MM = 4.5          # shallowest point of the canopy skirt
_TOP_CLEAR_MM = 18.0         # how far cutters reach above the crown

# What counts as a bay at all
_MIN_BAY_DEPTH_MM = 10.0
_MIN_BAY_LEN_MM = 25.0
_MIN_BAY_HALF_W_MM = 8.0
_DEPTH_FRACTION = 0.35       # bay may reach out to where the section is still
#                              35% as deep as the centreline - past that the
#                              wing is skin and spar, not compartment
# Sections in the compartment lofts. These set the cost of every boolean
# downstream - a loft through more sections is a higher-degree surface and
# every trim against it is dearer - but cutting them too far is a false
# economy, because a coarser cavity changes which faces OCC decides it cannot
# triangulate and the bay then drops down the fallback ladder, which costs far
# more than the sections saved. Measured, one-piece build, four planforms:
#
#     11 x 9   swept 14.8 s   micro 60.8 s (rung 3, bay 30% smaller)
#     13 x 11  swept 18.0 s   micro 18.6 s (rung 0)
#     15 x 13  swept 25.5 s   micro 24.9 s (rung 0)
#
# 13 x 11 reproduces the crown to ~0.1 mm, well inside the wall thickness, and
# lands on the first rung for three of the four planforms.
_N_STATIONS = 13
_N_PROFILE = 11              # y samples per station in the crown/keel band

# Mesh check. tol 0.6 mm is what the rest of the CAD uses and it returns the
# SAME coverage ratio as tol 0.3 on every shape measured here, at about a
# thirteenth of the cost (38 s -> 2.9 s on one airframe). A tighter mesh finds
# no extra missing faces; it just makes more triangles of the ones OCC already
# agreed to produce.
_TESS_TOL = 0.6
# Default area floor. Callers that are checking a whole airframe should pass
# the stricter `area_ratio_min` instead - see `tessellates_cleanly`.
_TESS_AREA_FLOOR = 0.95


# ---------------------------------------------------------------------------
# Result
# ---------------------------------------------------------------------------

@dataclass
class BayResult:
    """Everything the caller needs to hollow the wing and fit a canopy.

    `cutter` is the one to CUT from the airframe: it is the compartment, the
    aperture and the magnet pockets as a single solid, and it already leaves
    the seat lip behind by not claiming it. `cavity` (the sealed compartment
    alone) and `aperture` are exposed for inspection; cutting them separately
    is slower and hits the near-coincident-face problems the module docstring
    describes.

    `lid` is the canopy, its own body. `scribe` is the groove used on the
    one-piece body so the parting line shows without the canopy coming off.

    `ledge` is NOT fused onto anything and is None by default. It is the seat
    lip as a standalone solid, for looking at or rendering only, and it costs
    about 12 s to isolate - pass `seat_ring=True` if you actually want it.
    """
    cavity: Solid | None = None
    aperture: Solid | None = None
    lid: Solid | None = None
    ledge: Solid | None = None       # None unless build_bay(seat_ring=True)
    scribe: Solid | None = None      # None unless one_piece
    cutter: Solid | None = None
    bay_mm: dict = field(default_factory=dict)
    rung: str = ""
    ok: bool = False
    # only produced when `airframe` is supplied to build_bay()
    airframe: Solid | None = None            # hollowed, opened, seat fused
    airframe_onepiece: Solid | None = None   # hollowed + scribed, lid attached

    def __bool__(self) -> bool:
        return self.ok


# ---------------------------------------------------------------------------
# OCC helpers (local copies - this module must not import cad.geometry)
# ---------------------------------------------------------------------------

def heal(solid: Solid) -> Solid:
    """ShapeFix a boolean result. Spline-on-spline cuts leave faces whose trim
    curves the mesher silently refuses to tessellate; ShapeFix rebuilds them.
    Keep the healed shape only if it is still one valid solid."""
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
    """Fraction of the BRep area that OCC actually meshes.

    `isValid()` is not enough and never has been: OCC will report a boolean
    result valid and then skip faces it cannot tessellate, which shows up as a
    hole in the exported skin. Returns 0.0 if it cannot mesh at all.
    """
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


def unmeshed_faces(solid: Solid, tol: float = _TESS_TOL) -> tuple[int, int]:
    """(faces OCC refused to triangulate, total faces).

    This is the DIRECT form of the test the area ratio only approximates. The
    failure being guarded against is specific: OCC reports a boolean result
    valid and then silently skips faces whose trim curves it cannot mesh, and
    the exported STL has a hole in it. Ask each face whether it ended up with a
    triangulation and there is no approximation left to argue about.

    The area ratio on its own is a poor judge on small parts: a 6 mm magnet
    pocket meshed at a 0.6 mm chord tolerance is a coarse polygon and loses
    ~2% of the true cylinder area, which looks exactly like a missing face and
    is not one. It is kept as a loose second opinion, not as the verdict.
    """
    try:
        from OCP.BRep import BRep_Tool
        from OCP.BRepMesh import BRepMesh_IncrementalMesh
        from OCP.TopLoc import TopLoc_Location

        BRepMesh_IncrementalMesh(solid.wrapped, tol, True)
        faces = solid.Faces()
        bad = 0
        for f in faces:
            loc = TopLoc_Location()
            tri = BRep_Tool.Triangulation_s(f.wrapped, loc)
            if tri is None or tri.NbTriangles() == 0:
                bad += 1
        return bad, len(faces)
    except Exception:
        return 1, 0


def tessellates_cleanly(solid: Solid, tol: float = _TESS_TOL,
                        area_min: float = _TESS_AREA_FLOOR) -> bool:
    """Will this shape export as a complete mesh?

    BOTH tests, because each one misses what the other catches.

    * Every face must have been triangulated. This is the direct form of the
      classic failure - OCC calls a boolean result valid and then skips faces
      whose trim curves it cannot mesh.
    * The triangulated area must be within `area_min` of the BRep area. A face
      can be handed a triangulation that covers only PART of it, which the
      per-face test cannot see: an integrated build passed the per-face test at
      96.9% coverage while 51 mm of the aircraft was simply absent from the
      exported STL.

    `area_min` is deliberately a parameter. On a whole airframe the area ratio
    is a sharp instrument and 0.985 is the right bar. On a small canopy
    carrying 6 mm magnet pockets it is not: at a 0.6 mm chord tolerance a 6 mm
    cylinder becomes a coarse polygon and loses ~2% of its area to the
    approximation, which reads exactly like a missing face and is not one.
    """
    bad, total = unmeshed_faces(solid, tol)
    if total == 0 or bad:
        if _TRACE and total:
            print(f"    [hatch] {bad}/{total} faces were not triangulated",
                  flush=True)
        return False
    r = tess_ratio(solid, tol)
    if r < area_min:
        if _TRACE:
            print(f"    [hatch] every face meshed but only {r:.4f} of the "
                  f"area came out (want {area_min})", flush=True)
        return False
    return True


def _reject(why: str) -> None:
    """Say WHY a rung was thrown away. Every one of these is a real geometric
    condition, and being told "no clean result" six times tells you nothing."""
    if _TRACE:
        print(f"    [hatch] REJECT: {why}", flush=True)
    _LAST_REJECT.append(why)
    return None


def _one_valid_solid(shape: Any) -> Solid | None:
    try:
        solids = shape.Solids()
    except Exception:
        return None
    if len(solids) != 1 or not shape.isValid():
        return None
    return solids[0]


def _round_tripped(solid: Solid) -> Solid:
    """`solid` written to BRep and read back, which rebuilds it from its own
    serialized form and normalizes the per-face tolerances that come out of a
    loft. Local copy of `geometry._round_tripped` (this module must not
    import its orchestrator - the sibling rule); same measured rescue, used
    here for the bay cut whose floor is a 1-wall sliver over the keel."""
    import tempfile
    from pathlib import Path as _P
    from OCP.BRep import BRep_Builder
    from OCP.BRepTools import BRepTools
    from OCP.TopoDS import TopoDS_Shape

    with tempfile.TemporaryDirectory(prefix="aeroforge_bay_") as d:
        path = str(_P(d) / "operand.brep")
        BRepTools.Write_s(solid.wrapped, path)
        shape = TopoDS_Shape()
        BRepTools.Read_s(shape, path, BRep_Builder())
    out = Shape.cast(shape)
    solids = out.Solids()
    return solids[0] if len(solids) == 1 else out


def _biggest_solid(shape: Any, min_volume: float = 1.0) -> Solid | None:
    try:
        keep = [s for s in shape.Solids() if s.Volume() > min_volume]
    except Exception:
        return None
    return max(keep, key=lambda s: s.Volume()) if keep else None


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


# ---------------------------------------------------------------------------
# The plan outline
# ---------------------------------------------------------------------------
#
# The compartment's footprint is a stadium: two flanks that follow the width
# the wing can actually contain at that station, closed by elliptical end caps.
# It is stored as (xs, hws, r) rather than as a wire, so that "inset by d" is
# an exact operation on the family - shrink the x range by d, drop every half
# width by d, shrink the cap radius by d - instead of a 2D offset, which OCC
# does badly on splines.

@dataclass
class _Plan:
    xs: np.ndarray               # ascending x stations, mm
    hws: np.ndarray              # available half width at each station, mm
    r: float                     # end-cap radius in x, mm

    def inset(self, d: float) -> "_Plan | None":
        x0, x1 = self.xs[0] + d, self.xs[-1] - d
        if x1 - x0 < 4.0:
            return None
        nx = np.linspace(x0, x1, len(self.xs))
        nh = np.interp(nx, self.xs, self.hws) - d
        if float(np.min(nh)) < 1.0:
            return None
        return _Plan(nx, nh, max(self.r - d, 1.0))

    def wire(self, z: float, n_flank: int = 22, n_cap: int = 9) -> Wire:
        x0, x1 = float(self.xs[0]), float(self.xs[-1])
        r = min(self.r, 0.45 * (x1 - x0))
        hw_f = float(np.interp(x0 + r, self.xs, self.hws))
        hw_a = float(np.interp(x1 - r, self.xs, self.hws))
        pts: list[Vector] = []
        # forward cap: a = pi -> pi/2, x runs x0 -> x0+r, y runs 0 -> +hw_f
        for a in np.linspace(math.pi, 0.5 * math.pi, n_cap)[:-1]:
            pts.append(Vector(x0 + r + r * math.cos(a), hw_f * math.sin(a), z))
        # right flank
        for x in np.linspace(x0 + r, x1 - r, n_flank):
            pts.append(Vector(float(x),
                              float(np.interp(x, self.xs, self.hws)), z))
        # aft cap: a = pi/2 -> 0
        for a in np.linspace(0.5 * math.pi, 0.0, n_cap)[1:]:
            pts.append(Vector(x1 - r + r * math.cos(a), hw_a * math.sin(a), z))
        # mirror back along -y (skip the two on-axis endpoints)
        mirror = [Vector(p.x, -p.y, z) for p in reversed(pts[1:-1])]
        pts += mirror
        return Wire.assembleEdges([Edge.makeSpline(pts + [pts[0]])])

    def prism(self, z0: float, z1: float) -> Solid:
        return Solid.extrudeLinear(self.wire(z0), [],
                                   Vector(0, 0, max(z1 - z0, 0.5)))

    @property
    def length(self) -> float:
        return float(self.xs[-1] - self.xs[0])

    @property
    def hw_max(self) -> float:
        return float(np.max(self.hws))


# ---------------------------------------------------------------------------
# Surveying the wing: how much room is actually in there?
# ---------------------------------------------------------------------------

@dataclass
class _Station:
    x: float
    hw: float          # half width the structure allows, inside the walls
    z_lo: float        # bay floor at the bay's OUTER edge (keel + wall)
    z_hi: float        # bay ceiling at the bay's OUTER edge (crown - wall)
    crown_c: float     # crown on the centreline, for cutter heights
    prof_lo: np.ndarray | None = None   # floor across y, on the shared grid
    prof_hi: np.ndarray | None = None   # ceiling across y
    prof_crown: np.ndarray | None = None  # the skin itself, across y
    sec_theta: np.ndarray | None = None   # 1/cos(slope) of the skin, across y

    def at(self, ys: np.ndarray, y: float) -> tuple[float, float]:
        """(floor, ceiling) a distance |y| off the centreline."""
        if self.prof_lo is None or self.prof_hi is None:
            return self.z_lo, self.z_hi
        return (float(np.interp(abs(y), ys, self.prof_lo)),
                float(np.interp(abs(y), ys, self.prof_hi)))


def _survey(wing: Any, x0: float, x1: float, hw_cap: float,
            wall: float, n: int | None = None) -> list[_Station]:
    """Walk the bay's x range and ask, at each station, how wide and how deep
    the wing can honestly be hollowed.

    Width is per-station and NOT a single constant. On a swept wing the
    forward end of a nominal bay sits ahead of the outboard sections, so a
    constant width taken from the thinnest station collapses the compartment to
    a slot - that is precisely the bug this replaces. The depth is read at the
    bay's OUTER edge, because a rectangular compartment has to clear the crown
    and the keel at its widest point, not on the centreline.
    """
    half = max(float(wing.half), 1e-6)
    f_lim = min(0.98, (hw_cap + wall) / half)
    out: list[_Station] = []
    for x in np.linspace(x0, x1, int(n or _N_STATIONS)):
        x = float(x)
        sec0 = wing.section(0.0)
        if not (sec0.le.x + 0.02 * sec0.chord <= x
                <= sec0.le.x + 0.985 * sec0.chord):
            continue
        xc0 = wing.xc_at(0.0, x)
        d0 = wing.crown_z(0.0, xc0) - wing.keel_z(0.0, xc0)
        floor_d = max(_MIN_BAY_DEPTH_MM + 2.0 * wall, _DEPTH_FRACTION * d0)
        f_use = 0.0
        for f in np.linspace(f_lim, 0.0, 33):
            f = float(f)
            sec = wing.section(f)
            # the station has to be inside that section's chord with a margin,
            # or the "wing" over the bay is leading/trailing edge
            if not (sec.le.x + 0.05 * sec.chord <= x
                    <= sec.le.x + 0.95 * sec.chord):
                continue
            xc = wing.xc_at(f, x)
            if wing.crown_z(f, xc) - wing.keel_z(f, xc) >= floor_d:
                f_use = f
                break
        hw = min(hw_cap, max(f_use * half - wall, 0.0))
        if hw < _MIN_BAY_HALF_W_MM:
            continue
        # clear the crown and the keel over the WHOLE width, not just its edge
        f_edge = min(hw + wall, half) / half
        crown, keel = 1e9, -1e9
        for f in np.linspace(0.0, f_edge, 6):
            f = float(f)
            sec = wing.section(f)
            if not (sec.le.x <= x <= sec.le.x + sec.chord):
                continue
            xc = wing.xc_at(f, x)
            crown = min(crown, wing.crown_z(f, xc))
            keel = max(keel, wing.keel_z(f, xc))
        if crown > 1e8 or keel < -1e8:
            continue
        z_hi, z_lo = crown - wall, keel + wall
        if z_hi - z_lo < _MIN_BAY_DEPTH_MM:
            continue
        out.append(_Station(x, hw, z_lo, z_hi, wing.crown_z(0.0, xc0)))
    return out


def _longest_run(stations: list[_Station]) -> list[_Station]:
    """Stations arrive with gaps where the wing is too thin or too short. Keep
    the longest CONTIGUOUS block - a bay in two pieces is not a bay."""
    if not stations:
        return []
    step = 0.0
    if len(stations) > 1:
        step = min(b.x - a.x for a, b in zip(stations, stations[1:]))
    best: list[_Station] = []
    run: list[_Station] = [stations[0]]
    for prev, st in zip(stations, stations[1:]):
        if st.x - prev.x <= step * 1.6 + 1e-6:
            run.append(st)
        else:
            best, run = (run if len(run) > len(best) else best), [st]
    return run if len(run) > len(best) else best


def _profile_band(wing: Any, stations: list[_Station], band_hw: float,
                  wall: float) -> np.ndarray:
    """Fill in each station's crown/keel profile across the span.

    The ceiling FOLLOWS THE CROWN and the floor the keel in y as well as in x,
    so the compartment is the wing's own inside surface offset by one wall and
    not a flat-topped slot. A flat ceiling taken at the bay's outer edge throws
    away the deepest part of the centre section - on a 2.2x deepened body that
    is most of the volume.

    The profiles are forced MONOTONE outward (running min on the ceiling,
    running max on the floor). Outboard of the blend the section thins anyway,
    so this costs nothing real, and it guarantees the two chains never cross -
    which is what would otherwise turn a station wire into a figure of eight
    and the lofted band into a self-intersecting mess.
    """
    ys = np.linspace(0.0, band_hw, _N_PROFILE)
    half = max(float(wing.half), 1e-6)
    for st in stations:
        hi = np.empty(_N_PROFILE)
        lo = np.empty(_N_PROFILE)
        cr = np.empty(_N_PROFILE)
        c_hi, c_lo, c_cr, pinched = 1e9, -1e9, 1e9, False
        for i, y in enumerate(ys):
            f = min(float(y) / half, 1.0)
            sec = wing.section(f)
            if (not pinched and sec.le.x + 0.03 * sec.chord <= st.x
                    <= sec.le.x + 0.97 * sec.chord):
                xc = wing.xc_at(f, st.x)
                c_cr = min(c_cr, wing.crown_z(f, xc))
                c_hi = min(c_hi, c_cr - wall)
                c_lo = max(c_lo, wing.keel_z(f, xc) + wall)
            if c_hi - c_lo < 1.0:
                # ran out of section: hold a 1 mm sliver. It lies outside the
                # bay's own plan, so it never reaches the cut.
                mid = 0.5 * (c_hi + c_lo) if c_hi < 1e8 else st.z_lo
                c_hi, c_lo, pinched = mid + 0.5, mid - 0.5, True
                c_cr = min(c_cr, c_hi + wall)
            hi[i], lo[i], cr[i] = c_hi, c_lo, c_cr
        st.prof_hi, st.prof_lo, st.prof_crown = hi, lo, cr

    # How steep the skin is at each sample, as 1/cos(slope). The seat's
    # clearance is measured DOWN, but what has to clear is the gap along the
    # surface NORMAL, and those are the same thing only where the skin is
    # flat. On a deep BWB the crown falls ~120 mm from the spine to the bay's
    # edge; there a 0.2 mm vertical drop is a 0.05 mm normal gap, and the
    # canopy's skirt ends up rubbing on its own seat. Dividing the drop by
    # cos(slope) restores a true 0.2 mm normal to the surface. Capped at 6 so
    # a near-vertical patch cannot swallow the skirt's engagement.
    crowns = np.array([st.prof_crown for st in stations], dtype=float)
    sx = np.array([st.x for st in stations], dtype=float)
    if len(stations) >= 2 and len(ys) >= 2:
        gx = np.gradient(crowns, sx, axis=0)
        gy = np.gradient(crowns, ys, axis=1)
        sec = np.sqrt(1.0 + gx ** 2 + gy ** 2)
    else:
        sec = np.ones_like(crowns)
    for st, row in zip(stations, np.clip(sec, 1.0, 6.0)):
        st.sec_theta = row
    return ys


def _band_wire(x: float, ys: np.ndarray, lo: np.ndarray,
               hi: np.ndarray) -> Wire:
    """One station of a crown/keel band: FOUR edges, two splines and two short
    lines, not a polygon.

    This is not cosmetic. A cutter's cost against the airframe is set by its
    FACE COUNT, and planar faces meeting big B-spline faces are the expensive
    case: measured on one 553 mm wing, cutting a 1202-face polygonal band took
    187 s, the same shape as a 4-edge spline band takes about 5 s, and a plain
    3-face prism 2 s. Same geometry, two orders of magnitude.
    """
    bot = [Vector(x, float(-y), float(z)) for y, z in zip(ys[::-1], lo[::-1])]
    bot += [Vector(x, float(y), float(z)) for y, z in zip(ys, lo)][1:]
    top = [Vector(x, float(y), float(z)) for y, z in zip(ys[::-1], hi[::-1])]
    top += [Vector(x, float(-y), float(z)) for y, z in zip(ys, hi)][1:]
    return Wire.assembleEdges([
        Edge.makeSpline(bot),
        Edge.makeLine(bot[-1], top[0]),
        Edge.makeSpline(top),
        Edge.makeLine(top[-1], bot[0]),
    ])


def _band_depth(stations: list[_Station]) -> float:
    """How far the crown climbs across the bay - the yardstick for how deep a
    cutter has to be to swallow the whole thing."""
    hi = max(float(np.max(s.prof_crown)) for s in stations)
    lo = min(float(np.min(s.prof_crown)) for s in stations)
    return hi - lo


def _band(stations: list[_Station], ys: np.ndarray,
          lo: Any, hi: Any) -> Solid:
    """Loft a solid between two z profiles given per station.

    `lo` / `hi` are callables station -> array over `ys`. The upper profile is
    forced at least 1 mm above the lower one: where the section runs out the
    two would otherwise cross and the station wire would become a figure of
    eight.
    """
    wires = []
    for st in stations:
        a = np.asarray(lo(st), dtype=float)
        b = np.maximum(np.asarray(hi(st), dtype=float), a + 1.0)
        wires.append(_band_wire(st.x, ys, a, b))
    return Solid.makeLoft(wires, False)


def _offset_band(stations: list[_Station], ys: np.ndarray,
                 top_off: float, bot_off: float) -> Solid:
    """A solid whose TOP surface is the skin offset straight down by
    `top_off` and whose bottom is offset by `bot_off` (bot_off > top_off).

    This is what makes the canopy a canopy. Cut a plug of airframe with the
    band at `top_off = skirt_depth` and what is left is a shell of CONSTANT
    depth following the crown - the same thing a vacuum-formed hatch is. Slice
    the seat out of the band at `top_off = skirt_depth + clearance` and its top
    face is, by construction, the surface the canopy's skirt lands on.

    Doing it this way instead of with a flat plane matters more than it
    sounds: on a 2.2x deepened centre body the crown falls ~20 mm from the
    spine to the bay's edge and another 30 mm from nose to tail, so a flat seat
    gives a skirt 48 mm deep at the front and 5 mm deep at the back - a lid
    that is mostly a plug, filling the compartment it is supposed to open.

    Only Z varies between sections, which is the one thing an OCC loft can be
    trusted with.
    """
    return _band(stations, ys, lambda st: st.prof_crown - bot_off,
                 lambda st: st.prof_crown - top_off)


def _inscribed_box(stations: list[_Station], ys: np.ndarray, plan: "_Plan",
                   ) -> tuple[float, float, float]:
    """Largest rectangular box that fits inside the compartment - i.e. what a
    builder can actually slide in, which is the number they care about.

    Honest about both trades. Width costs depth, because the ceiling comes
    down as you go outboard, so a grid of candidate widths is swept and the
    best VOLUME wins. And the box has to fit inside the seat ring's inner
    outline, `plan`, whose rounded ends pull the usable length in well short of
    the cavity's overall length.
    """
    hw_max = plan.hw_max
    best = (0.0, 0.0, 0.0)
    best_v = 0.0
    for w in np.linspace(4.0, hw_max, 28):
        w = float(w)
        run: list[_Station] = []
        runs: list[list[_Station]] = []
        for st in stations:
            # the rounded end caps: the plan's own outline, not its x range
            if _plan_hw_at(plan, st.x) >= w:
                run.append(st)
            else:
                runs.append(run)
                run = []
        runs.append(run)
        for r in runs:
            if len(r) < 2:
                continue
            h = min(st.at(ys, w)[1] - st.at(ys, w)[0] for st in r)
            L = r[-1].x - r[0].x
            v = L * 2.0 * w * max(h, 0.0)
            if v > best_v:
                best_v, best = v, (L, 2.0 * w, max(h, 0.0))
    return best


def _plan_area(plan: "_Plan", n: int = 200) -> float:
    """Plan area enclosed by an outline, mm^2. Integrated off the same
    `_plan_hw_at` the wire is drawn from, so it matches the built shape."""
    xs = np.linspace(plan.xs[0], plan.xs[-1], n)
    hw = np.array([max(_plan_hw_at(plan, float(x)), 0.0) for x in xs])
    trapz = getattr(np, "trapezoid", None) or np.trapz
    return float(2.0 * trapz(hw, xs))


def _plan_hw_at(plan: "_Plan", x: float) -> float:
    """Half width of the plan OUTLINE at x, end caps included."""
    x0, x1 = float(plan.xs[0]), float(plan.xs[-1])
    if not (x0 <= x <= x1):
        return -1.0
    r = min(plan.r, 0.45 * (x1 - x0))
    d = min(x - x0, x1 - x)
    if d >= r:
        return float(np.interp(x, plan.xs, plan.hws))
    # inside an end cap the outline is the ellipse the wire actually draws
    x_cap = x0 + r if x - x0 < r else x1 - r
    hw_cap = float(np.interp(x_cap, plan.xs, plan.hws))
    return hw_cap * math.sqrt(max(1.0 - ((r - d) / r) ** 2, 0.0))


# ---------------------------------------------------------------------------
# The lid
# ---------------------------------------------------------------------------

def _build_lid(airframe: Solid, plan_lid: _Plan, plan_inner: _Plan,
               stations: list[_Station], ys: np.ndarray,
               z_lo: float, z_top: float, skirt_h: float, lid_t: float
               ) -> Solid | None:
    """The canopy, as a TRAY: top skin plus a skirt, open underneath.

    The top skin is not modelled - it is SELECTED OUT of the airframe, so it
    carries the crown's exact double curvature and the canopy sits flush by
    construction rather than by fitting. Nothing here is offset or shelled.

    Three shapes and two cuts:
      plug   = the airframe inside the lid's plan
      shell  = plug minus the skin offset down by `skirt_h` -> a constant-depth
               cap, which is what a moulded canopy is
      lid    = shell minus (the skin offset down by `lid_t`, clipped to the
               skirt's inner outline) -> top skin plus a skirt ring, open below
    """
    prism = plan_lid.prism(z_lo, z_top)
    try:
        plug = _biggest_solid(airframe.intersect(prism), 10.0)
    except Exception:
        return None
    if plug is None:
        return None
    # Deep enough to swallow the whole plug wherever the crown happens to be,
    # and no deeper: every millimetre of a cutter is boolean work.
    deep = _band_depth(stations) + 20.0
    try:
        # NEITHER of the canopy's two surfaces is taken from the sampled band,
        # and that is the whole reason it is trustworthy. The band only passes
        # THROUGH the sampled crown heights; between samples it rides above or
        # below the airframe's actual lofted skin by a few tenths. Where it
        # rides high it punches windows through the top skin, and where it
        # rides high across the rim it severs the skirt and leaves the canopy
        # in pieces - of which "keep the biggest" then keeps one. All of that
        # still comes back as a closed, valid, well-meshed solid that seats
        # correctly; only the volume gives it away, and it moved the canopy
        # between 29 and 64 cm^3 purely with the loft resolution.
        #
        # So both surfaces are the AIRFRAME ITSELF, dropped: the skin is
        # exactly `lid_t` thick and the skirt exactly `skirt_h` deep, on any
        # wing, at any resolution. The dropped copies are taken through
        # slightly different outlines so no two side walls are ever coincident.
        outer = plan_lid.inset(-1.5) or plan_lid
        low = _biggest_solid(
            airframe.intersect(outer.prism(z_lo - deep, z_top)), 10.0)
        if low is None:
            return None
        shell = _biggest_solid(
            plug.cut(low.translate(Vector(0, 0, -skirt_h))), 10.0)
        if shell is None:
            return None
        inner = plug.translate(Vector(0, 0, -lid_t)).intersect(
            plan_inner.prism(z_lo - deep, z_top))
        return _biggest_solid(heal(shell.cut(inner)), 10.0)  # noqa: E501
    except Exception:
        return None


# ---------------------------------------------------------------------------
# Magnet retention
# ---------------------------------------------------------------------------

def _pocket(x: float, y: float, z0: float, up: bool,
            over: float = 0.4) -> Solid:
    """A blind bore for one magnet, opening at z0.

    It deliberately overshoots the face it opens through by `over`. A cylinder
    whose end cap is exactly coplanar with the face it breaks is the tangent
    boolean that leaves a valid-looking solid invalid - and this one is fused
    into a cutter, so getting it wrong loses the whole bay.
    """
    d = MAGNET_DIA_MM + MAGNET_FIT_MM
    h = MAGNET_H_MM + 0.2 + over
    z = z0 - over if up else z0 - h + over
    return Solid.makeCylinder(0.5 * d, h, Vector(x, y, z), Vector(0, 0, 1))


@dataclass
class _Magnets:
    """Everything needed to fit a pair of discs, computed WITHOUT the canopy.

    Splitting it this way is what lets the canopy be built last. The airframe
    side (`pads`, `pockets`) is carved into the bay cutter; the canopy side
    (`lugs`, `bores`) is fused into the lid once the airframe cuts are known to
    be good. None of it needs the lid to exist, so a rung that is going to be
    rejected never pays for one.
    """
    pads: list[Solid]
    pockets: list[Solid]
    lugs: list[Solid]
    bores: list[Solid]
    # The same lugs BEFORE the skin band trims their tops - kept so that
    # `_fit_magnets`, which has the real canopy in hand, can recut a lug
    # against the lid itself when the band's idea of the skin turns out to
    # disagree with the lid's true surface (see there).
    lugs_tall: list[Solid] = field(default_factory=list)
    # (x, z_mag) of each disc, front then aft - published for the report
    centres: list[tuple[float, float]] = field(default_factory=list)


def _magnets(plan_bay: _Plan, plan_seat_in: _Plan,
             stations: list[_Station], ys: np.ndarray,
             skirt_h: float, lid_t: float, z_top: float,
             x_ap_end: float | None = None) -> "_Magnets | None":
    """Two magnet discs (Ø MAGNET_DIA_MM x MAGNET_H_MM), one at each end.

    The 3D-printed RC convention is a latch or tongue forward and magnets aft
    (Painless360's canopy hatch latch); magnets at both ends is the symmetric
    version of the same idea and needs no moving parts.

    Each disc needs a local pad - the seat ring is ~5 mm wide and will not
    house the magnet - and the pads go at the two ENDS, straddling the
    compartment's front and rear walls, so each is carried by a bulkhead. Not
    further in along the spine: the seat ring there is out at +-30 mm of y, so
    a pad on the centreline would be floating in the middle of the bay
    attached to nothing.
    """
    # The airframe pad is deliberately WIDER than the canopy's lug. Give them
    # the same width and their side faces are coplanar, which is both the
    # near-coincident-face hazard OCC is worst at and a joint whose real
    # clearance you cannot measure - the parts read as touching.
    #
    # Both scale WITH the bore, holding the original rims (pad 3.4 mm per
    # side, lug 1.4 mm - the 6.2 mm bore's 13.0 / 9.0). A fixed width here is
    # how a bore change would quietly thin the lug wall: at the builder's
    # 8.15 the old 9.0 mm lug would have kept 0.4 mm of wall per side.
    bore = MAGNET_DIA_MM + MAGNET_FIT_MM
    pad, lug_w = bore + 6.8, bore + 2.8
    x0, x1 = float(plan_bay.xs[0]), float(plan_bay.xs[-1])

    # WHERE THE LUGS GO. Not simply "at each end": the seat ring's outline
    # closes in toward the centreline at the fore and aft caps, so a lug of any
    # width put right at the tip reaches out into the ring and the canopy fouls
    # its own seat. Walk in from each end until the seat's INNER outline is
    # comfortably wider than the lug, and start it there.
    need = 0.5 * pad + 2.0

    def _walk(from_start: bool) -> float | None:
        grid = np.linspace(plan_seat_in.xs[0], plan_seat_in.xs[-1], 240)
        for x in (grid if from_start else grid[::-1]):
            if _plan_hw_at(plan_seat_in, float(x)) >= need:
                return float(x)
        return None

    xf, xa = _walk(True), _walk(False)
    if xf is None or xa is None or xa - xf < 2.5 * pad:
        if _TRACE:
            print("    [hatch] magnets: hatch too small to place lugs clear "
                  "of the seat", flush=True)
        return None
    # airframe pad: reaches back to the compartment's end wall, so each one is
    # a rib off a bulkhead rather than a block floating in the bay.
    # ...EXCEPT when the cavity continues aft of the hatch (the root-principle
    # extension): the compartment's end wall is then up to 200 mm behind the
    # lug, and a rib reaching it is the "long stick extending from the back
    # of the plane" the user rejected (2026-08-27). The aft pad then stops
    # just past the aperture and hangs from a short RISER into the roof
    # right behind the opening - a bracket protruding from the inner ceiling,
    # like the front one, not a spine down the cavity.
    riser: Solid | None = None
    if x_ap_end is not None and x_ap_end + _AFT_RISER_LEN_MM < x1:
        x_pad_end = x_ap_end + _AFT_RISER_LEN_MM
    else:
        x_pad_end = x1 + 3.0
    spots = [((x0 - 3.0, xf + pad), (xf, xf + pad)),
             ((xa - pad, x_pad_end), (xa - pad, xa))]
    out = _Magnets([], [], [], [])
    try:
        # The clip band has to reach ABOVE the tallest lug, or the lug is
        # sliced in two and the fuse hands back a floating block. And it stops
        # 0.6 mm INSIDE the lid's own skin rather than exactly on it: a lug
        # whose top face is coincident with the surface it fuses to is the
        # tangent boolean that leaves a valid-looking solid invalid.
        skin_cut = _offset_band(stations, ys, -(_TOP_CLEAR_MM + 12.0),
                                max(lid_t - 0.6, 0.4))
        # Clip the lug to the SEAT's inner outline pulled in another 0.8 mm,
        # not to the canopy's own outline: that keeps it clear of the seat ring
        # by construction, and it stops the lug's side face landing exactly on
        # the skirt's, which is the tangency that makes a fine-looking fuse
        # come back invalid.
        clip = plan_seat_in.inset(0.8) or plan_seat_in
        sx = np.array([s.x for s in stations])
        # the crown at the pad's OUTER EDGE, not on the spine: the pad is 13 mm
        # wide and the skin falls away in y as well as in x
        s_cr = np.array([float(np.interp(0.5 * pad, ys, s.prof_crown))
                         for s in stations])
        for (pa, pb), (la, lb) in spots:
            xm = 0.5 * (la + lb)
            st = min(stations, key=lambda t: abs(t.x - xm))
            assert st.prof_crown is not None and st.prof_lo is not None
            # The magnet faces are FLAT, the skin they hang under is CURVED
            # in both directions. Take the crown at its LOWEST anywhere over
            # the pad's footprint - across its length AND out at its edge - or
            # the canopy's skirt dips through the pad where the crown falls
            # away and the two parts jam instead of mating.
            z_mag = float(np.min(np.interp(
                np.linspace(pa, pb, 9), sx, s_cr))) - skirt_h
            if z_mag - 6.0 < float(np.max(st.prof_lo)):
                if _TRACE:
                    print("    [hatch] magnets: no room under the seat",
                          flush=True)
                return None
            if z_mag + MAGNET_H_MM + 1.4 > float(st.prof_crown[0]) - lid_t:
                if _TRACE:
                    print("    [hatch] magnets: skin too thin for the lug",
                          flush=True)
                return None
            out.pads.append(Solid.makeBox(
                pb - pa, pad, 5.0,
                Vector(pa, -0.5 * pad, z_mag - MAGNET_GAP_MM - 5.0)))
            if x_ap_end is not None and abs(pb - (x_ap_end + _AFT_RISER_LEN_MM)) < 1e-6:
                # the riser: from the shelf's underside up through the roof.
                # It starts 2 mm aft of the aperture's wall (clear of the
                # lid's seat by construction) and is carved into the cutter,
                # so it becomes material only where the cavity would have
                # been - it grows out of the ceiling wherever that is.
                z_shelf = z_mag - MAGNET_GAP_MM - 5.0
                out.pads.append(Solid.makeBox(
                    _AFT_RISER_LEN_MM - 2.0, pad, (z_top + 2.0) - z_shelf,
                    Vector(x_ap_end + 2.0, -0.5 * pad, z_shelf)))
            out.pockets.append(_pocket(xm, 0.0, z_mag - MAGNET_GAP_MM,
                                       up=False))
            # TALL ENOUGH TO REACH THE SKIN WHERE THE SKIN ACTUALLY IS.
            # z_mag is the crown MINIMUM over the pad footprint, measured out
            # at the pad's edge - but the canopy skin above the lug's own plan
            # area rides at the CENTRELINE crown, which on a domed centre body
            # sits several millimetres higher. The old fixed height
            # (skirt_h + 3.0) was "just tall enough to reach through the skin"
            # measured from the wrong datum: on the bwb's aft lug the raw box
            # topped out BELOW the lid's inner surface, the skin band never
            # even trimmed it, and the lug fused to nothing - which is how
            # every bwb shipped "airframe pads only". The box now runs to just
            # past the local centreline crown; `skin_cut`'s band reaches
            # TOP_CLEAR + 12 above the crown, so the taller box still cannot
            # leave a floating cap.
            z_hi = float(np.max(np.interp(np.linspace(la, lb, 7), sx,
                                          np.array([float(t.prof_crown[0])
                                                    for t in stations]))))
            h_lug = max(min(z_hi + 1.0, z_top + 1.5) - z_mag, skirt_h + 3.0)
            lug = Solid.makeBox(lb - la, lug_w, h_lug,
                                Vector(la, -0.5 * lug_w, z_mag))
            # clip the lug inside the seat's inset outline first, keeping the
            # UNTRIMMED version for `_fit_magnets`' recut fallback...
            lug = lug.intersect(clip.prism(z_mag - 1.0, z_top + 2.0))
            tall = _biggest_solid(lug, 5.0)
            if tall is None:
                if _TRACE:
                    print("    [hatch] magnets: lug came out empty", flush=True)
                return None
            # ...then trim its top under the lid's own skin
            lug = _biggest_solid(tall.cut(skin_cut), 5.0)
            if lug is None:
                if _TRACE:
                    print("    [hatch] magnets: lug came out empty", flush=True)
                return None
            out.lugs.append(lug)
            out.lugs_tall.append(tall)
            out.bores.append(_pocket(xm, 0.0, z_mag, up=True))
            out.centres.append((float(xm), float(z_mag)))
        return out
    except Exception as exc:
        if _TRACE:
            print(f"    [hatch] magnets: {type(exc).__name__}: {exc}",
                  flush=True)
        return None


def _fit_magnets(lid: Solid, mag: "_Magnets",
                 lid_t: float = 1.2) -> Solid | None:
    """Fuse the lugs into the canopy and bore them. None if it will not go.

    Both lugs in one fuse and both bores in one cut. OCC's BOPAlgo takes a
    list; feeding it one shape at a time makes it redo the whole
    surface-surface pass per shape, and healing in between doubles that again.

    A LUG IS ONLY FUSED AFTER IT PROVABLY SHARES VOLUME WITH THE CANOPY -
    the same existence doctrine as `fuse_feature`, applied before the boolean
    instead of after. The banded lug top rides a station-lofted crown that
    can sit a millimetre away from the lid's true BRep surface (the identical
    mismatch the canopy-volume check downstream guards against); fusing a lug
    that merely grazes the skin returns two solids and used to silently
    degrade every such build to "airframe pads only". When the overlap comes
    back trivial the lug's top is RECUT against the LID ITSELF raised by
    (lid_t - 0.4): the shell-against-a-dropped-copy trick the lid is built
    with, run in reverse, which lands the top (lid_t - 0.4) inside the real
    skin by construction rather than by interpolation.
    """
    def _overlap(a: Solid, b: Solid) -> float:
        try:
            parts = a.intersect(b).Solids()
            return float(sum(s.Volume() for s in parts)) if parts else 0.0
        except Exception:
            return 0.0

    MIN_OVERLAP = 20.0          # mm^3; a healthy joint measures ~100+
    try:
        chosen: list[Solid] = []
        for i, lug in enumerate(mag.lugs):
            if _overlap(lid, lug) < MIN_OVERLAP and i < len(mag.lugs_tall):
                raised = mag.lugs_tall[i].cut(
                    lid.translate(Vector(0.0, 0.0, max(lid_t - 0.4, 0.4))))
                deep = _biggest_solid(raised, 3.0)
                if deep is not None and _overlap(lid, deep) >= MIN_OVERLAP:
                    if _TRACE:
                        print(f"    [hatch] magnets: lug {i} recut against "
                              f"the lid itself (band missed the skin)",
                              flush=True)
                    lug = deep
            if _overlap(lid, lug) < MIN_OVERLAP:
                if _TRACE:
                    print(f"    [hatch] magnets: lug {i} shares no volume "
                          f"with the canopy", flush=True)
                return None
            chosen.append(lug)
        if chosen:
            lid = heal(lid.fuse(*chosen))
        if mag.bores:
            lid = heal(lid.cut(*mag.bores))
        return _one_valid_solid(lid)
    except Exception as exc:
        if _TRACE:
            print(f"    [hatch] magnets: canopy side failed "
                  f"({type(exc).__name__})", flush=True)
        return None


# ---------------------------------------------------------------------------
# One attempt at a bay
# ---------------------------------------------------------------------------

def _build_scribe(stations: list[_Station], ys: np.ndarray, plan_ap: _Plan,
                  wall: float) -> Solid | None:
    """The canopy's parting line, drawn on the one-piece body.

    A groove of CONSTANT depth following the skin, not a flat-bottomed slot.
    The crown climbs ~30 mm from the bay's aft end to its nose, so a flat cut
    deep enough to score the low end goes clean through at the high end -
    which does not scribe the canopy, it cuts it off.

    Built LAST, and only for a rung that has already passed everything else.
    It is decoration: it used to be computed for every attempt, including the
    ones that were about to be thrown away, and the cut alone is 6-9 s.
    """
    try:
        t = max(0.5, 0.30 * wall)
        outer = _Plan(plan_ap.xs, plan_ap.hws + t, plan_ap.r + t)
        skin = _offset_band(stations, ys, -1.0, 0.45 * wall)
        gb = skin.BoundingBox()
        ring = outer.prism(gb.zmin - 2.0, gb.zmax + 2.0).cut(
            plan_ap.prism(gb.zmin - 4.0, gb.zmax + 4.0))
        return _biggest_solid(ring.intersect(skin), 1.0)
    except Exception:
        return None



def _guarded_hw(wing: Any, x: float, hw: float, guard: dict) -> float:
    """Shrink a rear-extension station's half width until it clears the
    elevon-hinge corridor (inside the elevon's span band only) and the
    TE / motor-mount chord cap."""
    half = max(float(wing.half), 1e-6)
    hx = float(guard.get("hinge_xc", 1.0)) - 0.04
    s_lo = float(guard.get("span_lo_mm", 1e9))
    s_hi = float(guard.get("span_hi_mm", -1.0))
    xc_max = float(guard.get("xc_max", 0.88))
    for _ in range(24):
        ok = True
        for y in (0.0, 0.6 * hw, hw):
            f = min(y / half, 0.995)
            xc = float(wing.xc_at(f, x))
            if not (0.0 <= xc <= xc_max):
                ok = False
                break
            if s_lo <= y <= s_hi and xc > hx:
                ok = False
                break
        if ok:
            return hw
        hw *= 0.85
        if hw < _MIN_BAY_HALF_W_MM:
            return 0.0
    return 0.0


def _attempt(wing: Any, x0: float, x1: float, hw_cap: float, wall: float,
             airframe: Solid | None, magnets: bool, one_piece: bool,
             seat_ring: bool, area_ratio_min: float, canopy: bool,
             scribe_line: bool, extend_to: float | None = None,
             guard: dict | None = None,
             roof_cap: tuple | None = None) -> BayResult | None:
    stations = _longest_run(_survey(wing, x0, x1, hw_cap, wall))
    if len(stations) < 6:
        return None
    if stations[-1].x - stations[0].x < _MIN_BAY_LEN_MM:
        return None

    # ---- root-principle rear extension (user, rounds 8-10): the CAVITY
    # itself continues aft of the hatch, built by the SAME survey / plan /
    # band machinery as the front - one native void with the hull's own
    # inner surfaces, no add-on cutter. The aperture (and therefore the
    # lid) stays on the hatch span; the extension carries >= 3 mm walls.
    stations_hatch = stations
    ext: list[_Station] = []
    if extend_to is not None and len(stations) >= 2:
        wall_e = max(wall, 3.0)
        x_last = stations[-1].x
        step_x = max(stations[-1].x - stations[-2].x, 3.0)
        if float(extend_to) > x_last + 2.0 * step_x:
            # survey at the HATCH's own station spacing: a fixed count
            # spread over a 250 mm run spaced ~21 mm on the plank (hatch
            # 8 mm), tripped the contiguity check and ended the extension
            # after one station
            n_ext = max(_N_STATIONS,
                        int((float(extend_to) - x_last) / step_x) + 2)
            raw = _survey(wing, x_last + step_x, float(extend_to),
                          hw_cap, wall_e, n=n_ext)
            hw_lead = stations[-1].hw
            for st in raw:
                if st.x <= x_last + 1e-6:
                    continue
                if ext and st.x - ext[-1].x > 2.5 * step_x + 1e-6:
                    break                       # contiguous prefix only
                hw = min(st.hw, hw_lead + 2.0)
                if guard:
                    hw = _guarded_hw(wing, st.x, hw, guard)
                if hw < _MIN_BAY_HALF_W_MM:
                    break
                ext.append(_Station(st.x, hw, st.z_lo, st.z_hi,
                                    st.crown_c))
            stations = stations + ext

    xs = np.array([s.x for s in stations])
    hws = np.array([s.hw for s in stations])
    # a touch of smoothing: the width limit is found by a search and can step,
    # and a stepped plan spline is a boolean liability
    if len(hws) >= 5:
        k = np.array([0.25, 0.5, 0.25])
        hws = np.convolve(np.pad(hws, 1, mode="edge"), k, mode="valid")
    r = _clamp(0.45 * float(np.max(hws)), 3.0, 0.28 * float(xs[-1] - xs[0]))
    plan_bay = _Plan(xs, hws, r)

    # --- the compartment ---------------------------------------------------
    # NOT a loft through the bay's own corners. A prism carrying the
    # variable-width PLAN, intersected with a band that varies only in Z. One
    # shape, one degree of freedom each; a loft that changed width and height
    # at once would twist and self-intersect, and OCC would then hand back a
    # "cavity" that breaks the airframe into pieces.
    band_hw = plan_bay.hw_max + 4.0
    ys = _profile_band(wing, stations_hatch, band_hw, wall)
    if ext:
        _profile_band(wing, ext, band_hw, max(wall, 3.0))
        if roof_cap is not None:
            # something is buried in the crown aft of the hatch (a centre
            # fin's root): the cavity continues UNDER it with its roof held
            # >= 3 mm below that root, blending down over _EXT_RAMP_MM ahead
            # of it. Stations that no longer hold the minimum depth end the
            # extension there - honestly, the way the hull's own taper does.
            # The cap is a CENTRE STRIP, not the full width: the buried
            # root is a blade `y_cap` wide, so only that strip of the roof
            # dips under it (blended out over 6 mm each side) and the cavity
            # keeps its full depth beside the root - where the volume and
            # the servo runs are. A full-width cap left < 10 mm under a
            # 0.35 t root and ended every centre-fin extension at station 1.
            cap_x, cap_z = float(roof_cap[0]), float(roof_cap[1])
            y_cap = float(roof_cap[2]) if len(roof_cap) > 2 else 1e9
            w_y = 1.0 - np.clip((np.abs(ys) - y_cap) / 6.0, 0.0, 1.0)
            w_y = w_y * w_y * (3.0 - 2.0 * w_y)
            keep: list[_Station] = []
            for st in ext:
                lim_z = cap_z - max(wall, 3.0)
                if st.x < cap_x - _EXT_RAMP_MM:
                    keep.append(st)
                    continue
                t = _clamp((st.x - (cap_x - _EXT_RAMP_MM)) / _EXT_RAMP_MM,
                           0.0, 1.0)
                t = t * t * (3.0 - 2.0 * t)
                dip = np.maximum(st.prof_hi - lim_z, 0.0) * (t * w_y)
                lim = st.prof_hi - dip
                if float(np.max(lim - st.prof_lo)) < _MIN_BAY_DEPTH_MM:
                    break
                st.prof_hi = lim
                st.prof_crown = np.minimum(st.prof_crown,
                                           lim + max(wall, 3.0))
                st.z_hi = min(st.z_hi, float(np.interp(st.hw, ys, lim)))
                keep.append(st)
            if len(keep) < len(ext):
                stations = stations_hatch + keep
                ext = keep
                xs = np.array([s_.x for s_ in stations])
                hws = np.array([s_.hw for s_ in stations])
                if len(hws) >= 5:
                    k = np.array([0.25, 0.5, 0.25])
                    hws = np.convolve(np.pad(hws, 1, mode="edge"), k,
                                      mode="valid")
                r = _clamp(0.45 * float(np.max(hws)), 3.0,
                           0.28 * float(xs[-1] - xs[0]))
                plan_bay = _Plan(xs, hws, r)
    # ceiling/aperture numbers come from the HATCH span only - the rear
    # extension tapers and must not drag the lid's floor down with it
    z_lo = np.array([s.at(ys, s.hw)[0] for s in stations_hatch])
    z_hi = np.array([s.at(ys, s.hw)[1] for s in stations_hatch])
    z_ceiling = float(np.min(z_hi))          # lowest point of the bay ceiling
    z_top = float(max(np.max([s.crown_c for s in stations]),
                      np.max([np.max(s.prof_hi) for s in stations]))
                  ) + _TOP_CLEAR_MM
    z_bot = float(np.min([np.min(s.prof_lo) for s in stations]))

    lid_t = max(1.6, wall)
    skirt_t = max(1.2, 0.85 * wall)
    seat_w = LID_CLEARANCE_MM + skirt_t + _SEAT_OVERLAP_MM
    # Skirt depth, measured DOWN THE NORMAL of the skin rather than to a plane,
    # so it is the same all round. Deep enough to guide the lid in (the whole
    # point of a long engagement at a loose clearance) and shallow enough not
    # to eat the compartment.
    skirt_h = _clamp(4.0 * wall, _MIN_SKIRT_MM, 14.0)
    if skirt_h <= lid_t + 1.5:
        skirt_h = lid_t + 2.5
    # The aperture's floor sits below the LOWEST point of the ceiling, so
    # opening it always breaks through into the compartment no matter what the
    # crown is doing overhead. That is the difference between a lid that opens
    # the bay and a lid that peels a membrane off the top of it.
    z_ap = z_ceiling - max(2.0, wall)

    if ext:
        xs_h = np.array([s.x for s in stations_hatch])
        hws_h = np.array([s.hw for s in stations_hatch])
        if len(hws_h) >= 5:
            k_h = np.array([0.25, 0.5, 0.25])
            hws_h = np.convolve(np.pad(hws_h, 1, mode="edge"), k_h,
                                mode="valid")
        r_h = _clamp(0.45 * float(np.max(hws_h)), 3.0,
                     0.28 * float(xs_h[-1] - xs_h[0]))
        plan_hatch = _Plan(xs_h, hws_h, r_h)
    else:
        plan_hatch = plan_bay
    plan_ap = plan_hatch.inset(_APERTURE_INSET_MM)
    plan_seat_in = plan_ap.inset(seat_w) if plan_ap is not None else None
    if plan_ap is None or plan_seat_in is None:
        return _reject("no room left for a seat inside the bay")
    # Per-sample, following the skin's slope - see `_profile_band`.
    seat_thick = max(1.8, 0.9 * wall)

    def seat_top(st: _Station) -> np.ndarray:
        return skirt_h + SEAT_CLEARANCE_MM * st.sec_theta

    def seat_bot(st: _Station) -> np.ndarray:
        return seat_top(st) + seat_thick
    aperture = plan_ap.prism(z_ap, z_top)

    # THE VOID, AND WHY THE SEAT IS LEFT BEHIND RATHER THAN CUT OUT.
    #
    # The obvious construction is: hollow the whole compartment, then build a
    # thin shelf solid and put it back. It does not survive a real wing. The
    # shelf is a slab of constant VERTICAL thickness, and on a deep BWB the
    # crown falls ~120 mm from the spine to the bay's edge, so where the
    # surface is steep that slab is a sub-millimetre sliver - and slivers are
    # where OCC's spline booleans hand back solids of NEGATIVE volume, or of
    # more volume than either operand. Measured on a 1.1 m BWB: a seat that
    # should have been about 12 cm^3 came back as -248 cm^3.
    #
    # So the compartment is assembled from three CHUNKY pieces instead, and
    # the seat is simply the material none of them claim:
    #   core   - everything inside the seat's inner outline, full depth
    #   below  - everything inside the bay, up to the seat's underside
    #   groove - the aperture, from the seat's top face out through the skin
    # Nothing in that union is thinner than about 8 mm in any direction.
    # The ceiling of the `below` chunk. Inside the hatch span it is the
    # seat's underside. Aft of it (the root-principle extension) it rises to
    # the extension's own roof - which `_profile_band` put >= 3 mm under the
    # crown, never the 1-wall sliver: a cutter face one wall under a spline
    # skin is the near-tangent case OCC's boolean silently drops (the floor
    # needed `_FLOOR_RAISE_MM` for the same reason). The rise is a smoothstep
    # over _EXT_RAMP_MM, because a step between two adjacent stations makes
    # the smoothed loft overshoot through the top skin.
    x_h_end = stations_hatch[-1].x

    def below_ceiling(st: _Station) -> np.ndarray:
        under_seat = st.prof_crown - seat_bot(st)
        if st.x <= x_h_end + 1e-6:
            return under_seat
        t = _clamp((st.x - x_h_end) / _EXT_RAMP_MM, 0.0, 1.0)
        t = t * t * (3.0 - 2.0 * t)
        return (1.0 - t) * under_seat + t * np.minimum(st.prof_hi,
                                                        st.prof_crown - 3.0)

    core_lifted = False
    try:
        with _stage("compartment"):
            full = _band(stations, ys, lambda st: st.prof_lo,
                         lambda st: st.prof_hi)
            zl, zh = z_bot - 10.0, z_top
            cavity = _biggest_solid(
                plan_bay.prism(zl, zh).intersect(full), 100.0)
            core = plan_seat_in.prism(zl, zh).intersect(full)
            # `below` is ONE loft over the whole run. Inside the hatch span
            # its ceiling is the seat's underside; aft of the hatch (the
            # root-principle extension) it is the full inner crown, and the
            # loft's spline ramps between the two across the junction step.
            # A separate `aft` chunk was tried first and failed every rung:
            # it shared its floor and its plan walls EXACTLY with `below`,
            # and OCC's fuse will not join coincident spline faces.
            below = plan_bay.prism(zl, zh).intersect(
                _band(stations, ys, lambda st: st.prof_lo, below_ceiling))
            groove = plan_ap.prism(zl, zh).intersect(
                _band(stations, ys, lambda st: st.prof_crown - seat_top(st),
                      lambda st: st.prof_crown + _TOP_CLEAR_MM))
        if _TRACE:
            floor_min = float(np.min([np.min(s.prof_lo) for s in stations]))
            for nm, pc in (("cavity", cavity), ("core", core),
                           ("below", below), ("groove", groove)):
                try:
                    bb = pc.BoundingBox()
                    print(f"    [hatch] piece {nm:6}: z {bb.zmin:7.2f}.."
                          f"{bb.zmax:7.2f}  (floor band min {floor_min:.2f})",
                          flush=True)
                except Exception as exc:
                    print(f"    [hatch] piece {nm:6}: bbox failed {exc}",
                          flush=True)
        if os.environ.get("AEROFORGE_HATCH_DUMP"):
            # diagnostic: write the compartment pieces so a failing fuse can
            # be replayed offline (tools/diag scripts), never used in a build
            from OCP.BRepTools import BRepTools as _BT
            _dd = os.environ["AEROFORGE_HATCH_DUMP"]
            os.makedirs(_dd, exist_ok=True)
            for nm, pc in (("core", core), ("below", below),
                           ("groove", groove), ("cavity", cavity)):
                if pc is not None:
                    _BT.Write_s(pc.wrapped, os.path.join(_dd, f"{nm}.brep"))
        with _stage("void = core + below + groove"):
            # ONE boolean with all arguments, not chained ones. OCC's
            # BOPAlgo takes the whole argument list at once and pays for the
            # surface-surface work once; fusing a, then b, then healing in
            # between, made it do the job twice.
            void = _one_valid_solid(heal(core.fuse(below, groove)))
        if void is None:
            # Retry with the core's floor lifted off `below`'s - see
            # _CORE_FLOOR_LIFT_MM. Only reached when the coincident-floor
            # fuse has already failed, so a design that joins first time
            # keeps its byte-identical void.
            with _stage("void retry (core floor lifted)"):
                core_l = plan_seat_in.prism(zl, zh).intersect(
                    _band(stations, ys,
                          lambda st: st.prof_lo + _CORE_FLOOR_LIFT_MM,
                          lambda st: st.prof_hi))
                void = _one_valid_solid(heal(core_l.fuse(below, groove)))
                if void is not None:
                    core = core_l
                    core_lifted = True
    except Exception as exc:
        return _reject(f"the compartment pieces would not build: "
                       f"{type(exc).__name__}: {exc}")
    if cavity is None:
        return _reject("the compartment envelope did not intersect the plan")
    if void is None:
        return _reject("the compartment pieces would not join into one void")
    cutter = void

    box_l, box_w, box_h = _inscribed_box(stations, ys, plan_seat_in)
    depth_c = max(float(np.max([s.prof_hi[0] - s.prof_lo[0]
                                for s in stations])), 0.0)
    bay = {
        "length_mm": round(float(xs[-1] - xs[0]), 1),
        # The VOID's world-x extents. The ladder shrinks the cavity about its
        # centre, so the nominal `bay_start + bay_length` can sit well AFT of
        # the real aft bulkhead - a wire channel aimed at the nominal wall
        # then dead-ends inside solid material. Anything that has to pierce
        # the compartment must aim at these.
        "x0_mm": round(float(xs[0]), 1),
        "x1_mm": round(float(xs[-1]), 1),
        # NOTE: no z band is published from here. The planning profiles are
        # NOT the carved void (they read 25+ mm off it), so geometry measures
        # the built airframe directly (`_void_z_band`) and writes
        # `z_floor_aft_mm`/`z_ceil_aft_mm` into this dict itself.
        "width_mm": round(2.0 * float(np.max(hws)), 1),
        "depth_mm": round(depth_c, 1),
        "depth_at_edge_mm": round(float(np.max(z_hi - z_lo)), 1),
        "volume_cm3": round(cavity.Volume() / 1000.0, 1),
        "box_l_mm": round(box_l, 1),
        "box_w_mm": round(box_w, 1),
        "box_h_mm": round(box_h, 1),
        "box_volume_cm3": round(box_l * box_w * box_h / 1000.0, 1),
        "hatch_l_mm": round(plan_ap.length, 1),
        "hatch_w_mm": round(2.0 * plan_ap.hw_max, 1),
        "skirt_mm": round(skirt_h, 1),
        "clearance_mm": LID_CLEARANCE_MM,
        "seat_clearance_mm": SEAT_CLEARANCE_MM,
        "lid_skin_mm": round(lid_t, 2),
        "skirt_wall_mm": round(skirt_t, 2),
    }
    if core_lifted:
        bay["core_floor_lifted_mm"] = _CORE_FLOOR_LIFT_MM
    if ext:
        bay["hatch_x1_mm"] = round(float(stations_hatch[-1].x), 1)
        bay["cavity_extended_mm"] = round(
            float(xs[-1] - stations_hatch[-1].x), 1)
        bay["cavity_wall_aft_mm"] = round(max(wall, 3.0), 2)
    # per-station cavity band, published for the wire runs: the tube must
    # aim at the REAL void at ITS OWN station, not at the aft bulkhead's
    # numbers (the extension tapers)
    bay["cavity_stations_mm"] = [
        [round(s.x, 1), round(s.hw, 1),
         round(s.at(ys, 0.8 * s.hw)[0], 1),
         round(s.at(ys, 0.8 * s.hw)[1], 1)]
        for s in stations]
    res = BayResult(cavity=cavity, aperture=aperture, cutter=cutter,
                    bay_mm=bay, ok=True)

    if airframe is None:
        return res

    # --- everything below needs the airframe -------------------------------
    #
    # ORDER MATTERS FOR COST, NOT JUST FOR CORRECTNESS. Everything that can
    # reject this rung happens BEFORE the canopy is built, because the canopy
    # costs ~10 s and a rejected rung used to pay for it in full. The magnet
    # work is split for the same reason: the airframe-side pads need no lid, so
    # they can be carved into the cutter first and the canopy's lugs fused on
    # at the end.
    plan_lid = plan_ap.inset(LID_CLEARANCE_MM)
    plan_skirt_in = plan_ap.inset(LID_CLEARANCE_MM + skirt_t)
    if plan_lid is None or plan_skirt_in is None:
        return _reject("no room left for the canopy inside the aperture")

    mag: _Magnets | None = None
    if magnets:
        with _stage("magnet features"):
            mag = _magnets(plan_bay, plan_seat_in, stations, ys, skirt_h,
                           lid_t, z_top,
                           x_ap_end=(float(plan_ap.xs[-1]) if ext else None))
        bay["magnets"] = (f"2 x Ø{MAGNET_DIA_MM:g}x{MAGNET_H_MM:g} pockets, "
                          f"bore cut at exactly Ø{MAGNET_DIA_MM:g}"
                          if mag is not None
                          else "none (pads would not build cleanly)")
        if mag is not None:
            bay["magnet_centres_mm"] = [[round(x, 1), round(z, 1)]
                                        for x, z in mag.centres]
            bay["aperture_x1_mm"] = round(float(plan_ap.xs[-1]), 1)
            bay["aft_pad"] = ("shelf + riser into the roof behind the "
                              "aperture" if ext else "rib to the aft wall")
    else:
        bay["magnets"] = "none"

    # The magnet pads are CARVED OUT OF THE VOID, not fused onto the airframe
    # afterwards. Fusing anything onto a shape the size of the whole aircraft
    # cost 132 s of OCC time on one 553 mm wing, and produced a lump stuck to
    # the skin rather than part of it. Taking it out of the cutter costs about
    # a second and what comes back is the airframe's own material.
    if mag is not None:
        with _stage("carve pads into cutter"):
            carved: Solid | None = cutter
            try:
                # one cut and one fuse, not four of each - see _fit_magnets
                carved = heal(carved.cut(*mag.pads))
                carved = heal(carved.fuse(*mag.pockets))
                carved = _one_valid_solid(carved)
            except Exception:
                carved = None
        if carved is not None:
            cutter = carved
        else:
            # Retention is a convenience; the compartment is the product. If
            # the pads will not carve, drop them and keep the bay rather than
            # throwing the rung away and handing back a smaller aircraft.
            mag = None
            bay["magnets"] = "none (pads would not carve cleanly)"
            if _TRACE:
                print("    [hatch] magnets dropped (pads would not carve)",
                      flush=True)
    res.cutter = cutter

    # --- prove the cuts leave a buildable airframe -------------------------
    # EVERY boolean first, EVERY mesh check afterwards, and the order is not a
    # style choice. `tessellate()` writes a triangulation into the shape's
    # TShape, OCC booleans SHARE sub-shapes with their operands, and a boolean
    # against a shape whose faces already carry a mesh is dramatically slower:
    # measured on one 553 mm wing, the same cut took 2.0 s on a clean airframe
    # and 49.0 s once it had been tessellated. Checking the first cut before
    # making the second one turned a 60 s bay into a four-minute one.
    # A cut can FAIL WITHOUT FAILING: on a 1.2 m deep-crowned body,
    # `airframe.cut(void)` handed back a valid, single-shell, cleanly-meshing
    # solid in which the 249 x 95 x 55 mm compartment simply was not there -
    # the only material removed was a millimetres-thin lick along the floor.
    # Every existing gate (validity, shell count, mesh coverage) passed,
    # because none of them asks whether the compartment EXISTS. So ask
    # directly: points spread across the intended cavity must classify as
    # air in the cut result. Same doctrine as the horn-keyhole and mesh
    # gates: an OCC status code is not evidence that an operation did its job.
    def _void_absent(solid: Solid) -> bool:
        cl = BRepClass3d_SolidClassifier(solid.wrapped)
        for st in (stations[len(stations) // 5],
                   stations[len(stations) // 2],
                   stations[(4 * len(stations)) // 5]):
            lo, hi = float(st.prof_lo[0]), float(st.prof_hi[0])
            if hi - lo < 6.0:
                continue
            for f in (0.30, 0.60):
                cl.Perform(gp_Pnt(float(st.x), 0.0, lo + f * (hi - lo)), 1e-6)
                if cl.State() != TopAbs_OUT:
                    return True
        return False

    # The DUAL of `_void_absent`: the cut may open ONLY the aperture. The
    # cavity floor sits one wall above the keel by construction (prof_lo =
    # keel + wall in `_profile_band`), so the belly under the compartment is
    # a 1-wall sliver - exactly the near-tangent boolean OCC is worst at. On
    # the builder's 2026-08-21 swept-sport variant the cut ATE THAT FLOOR
    # WHOLE: a 224 x 64 mm hole clean through the belly, and every older
    # gate passed the result - it is one valid solid, it meshes completely,
    # the void probes read open, and ONE shell is what an open compartment
    # is supposed to be, so the shell count cannot tell a lid aperture from
    # a missing belly. Ask directly: points inside the floor skin must still
    # classify as MATERIAL. (The one-piece path was never exposed - its
    # sealed cavity means two shells, and its 2-shell gate catches a breach.)
    def _skin_breached(solid: Solid) -> bool:
        cl = BRepClass3d_SolidClassifier(solid.wrapped)
        for st in (stations[len(stations) // 5],
                   stations[len(stations) // 2],
                   stations[(4 * len(stations)) // 5]):
            lo, hi = float(st.prof_lo[0]), float(st.prof_hi[0])
            if hi - lo < 6.0:
                continue
            # halfway down the floor sliver the cavity leaves below prof_lo
            cl.Perform(gp_Pnt(float(st.x), 0.0, lo - 0.5 * wall), 1e-6)
            if cl.State() != TopAbs_IN:
                return True
        return False

    def _cut_ok(solid: Solid | None) -> bool:
        return (solid is not None and not _void_absent(solid)
                and not _skin_breached(solid))

    with _stage("airframe.cut(cutter)"):
        opened = _one_valid_solid(heal(airframe.cut(cutter)))
        if not _cut_ok(opened):
            # retry on FRESH operands - a copy drops every cached sub-shape
            # structure the failing boolean may have tripped on...
            opened = _one_valid_solid(heal(airframe.copy().cut(cutter.copy())))
        if not _cut_ok(opened):
            # ...then round-trip both operands through BRep (the biplane-fin
            # rescue, geometry._fuse_verified step 3)...
            with _stage("bay cut retry (BRep round-trip)"):
                opened = _one_valid_solid(heal(
                    _round_tripped(airframe).cut(_round_tripped(cutter))))
        if not _cut_ok(opened) and not core_lifted:
            # ...then the void rebuilt with the CORE'S FLOOR LIFTED off the
            # `below` chunk's (_CORE_FLOOR_LIFT_MM). The same coincident
            # floor that stops the three pieces joining (the bwb at the 675
            # box) can let them join and then poison the CUT: on the delta
            # at the sidebar box the fused void carved only the AFT half of
            # the compartment - x < 240 mm stayed solid through the fresh-
            # copy and round-trip retries, every rung down to rung 4 - while
            # `airframe.cut(below)` alone carved 6/6 probe points and
            # `airframe.cut(core)` alone only 4/6. With the core lifted 1 mm
            # the fused void cuts 6/6 first time (measured offline on the
            # dumped operands, one-op and fused alike). Same point set, no
            # bay shrink - so it comes before the raised-floor retry.
            with _stage("bay cut retry (core floor lifted)"):
                try:
                    core_l = plan_seat_in.prism(zl, zh).intersect(
                        _band(stations, ys,
                              lambda st: st.prof_lo + _CORE_FLOOR_LIFT_MM,
                              lambda st: st.prof_hi))
                    void_l = _one_valid_solid(heal(core_l.fuse(below,
                                                               groove)))
                    cut_l = void_l
                    if mag is not None and void_l is not None:
                        try:
                            cut_l = _one_valid_solid(heal(heal(
                                void_l.cut(*mag.pads)).fuse(*mag.pockets)))
                        except Exception:
                            cut_l = None
                        if cut_l is None:
                            cut_l = void_l
                            mag = None
                            bay["magnets"] = ("none (pads would not carve "
                                              "into the lifted-core bay)")
                    if cut_l is not None:
                        cand = _one_valid_solid(heal(
                            airframe.copy().cut(cut_l)))
                        if _cut_ok(cand):
                            opened = cand
                            cutter = cut_l
                            core = core_l
                            core_lifted = True
                            res.cutter = cutter
                            bay["core_floor_lifted_mm"] = _CORE_FLOOR_LIFT_MM
                except Exception:
                    pass
        if not _cut_ok(opened):
            # ...and the last resort RAISES THE FLOOR. The compartment floor
            # is `keel + wall` - a ~1.2 mm sliver between two spline lofts
            # whose own face tolerances approach that gap - and on the
            # builder's 2026-08-21 swept sport EVERY form of the cut (fused
            # void, fresh copies, BRep round-trip, the pieces one at a time)
            # welded the two surfaces together and the floor vanished: a
            # 224 x 64 mm hole straight through the belly, "just no wall at
            # all". Nothing is wrong with the cut REQUEST; the sliver is
            # inside OCC's tolerance-merge range, so no boolean strategy can
            # keep it. Thicken it instead: rebuild the void with the floor
            # raised and cut with that. Designs that never breach keep their
            # byte-identical thin-floor bays - this path only runs after the
            # thin floor has failed every other way, and the cost is a
            # compartment shallower by the raise.
            with _stage("bay cut retry (raised floor)"):
                try:
                    full_r = _band(stations, ys,
                                   lambda st: st.prof_lo + _FLOOR_RAISE_MM,
                                   lambda st: st.prof_hi)
                    core_r = plan_seat_in.prism(zl, zh).intersect(full_r)
                    below_r = plan_bay.prism(zl, zh).intersect(
                        _band(stations, ys,
                              lambda st: st.prof_lo + _FLOOR_RAISE_MM,
                              below_ceiling))
                    void_r = _one_valid_solid(heal(core_r.fuse(below_r,
                                                               groove)))
                    if void_r is None:
                        # the same coincident-floor failure as the first
                        # fuse, with the same cure (_CORE_FLOOR_LIFT_MM)
                        core_r = plan_seat_in.prism(zl, zh).intersect(
                            _band(stations, ys,
                                  lambda st: (st.prof_lo + _FLOOR_RAISE_MM
                                              + _CORE_FLOOR_LIFT_MM),
                                  lambda st: st.prof_hi))
                        void_r = _one_valid_solid(heal(
                            core_r.fuse(below_r, groove)))
                    cut_r = void_r
                    if mag is not None and void_r is not None:
                        try:
                            cut_r = _one_valid_solid(heal(heal(
                                void_r.cut(*mag.pads)).fuse(*mag.pockets)))
                        except Exception:
                            cut_r = None
                        if cut_r is None:
                            # retention is a convenience; the bay is the
                            # product
                            cut_r = void_r
                            mag = None
                            bay["magnets"] = ("none (pads would not carve "
                                              "into the raised-floor bay)")
                    if cut_r is not None:
                        opened = _one_valid_solid(heal(
                            airframe.copy().cut(cut_r)))
                        if _cut_ok(opened):
                            bay["floor_raised_mm"] = _FLOOR_RAISE_MM
                except Exception:
                    opened = None
    if opened is None:
        return _reject("cutting the bay did not leave one valid solid")
    if _void_absent(opened):
        if os.environ.get("AEROFORGE_HATCH_DUMP"):
            from OCP.BRepTools import BRepTools as _BT
            _dd = os.environ["AEROFORGE_HATCH_DUMP"]
            os.makedirs(_dd, exist_ok=True)
            _BT.Write_s(cutter.wrapped, os.path.join(_dd, "cutter.brep"))
            _BT.Write_s(airframe.wrapped, os.path.join(_dd, "airframe.brep"))
            _BT.Write_s(opened.wrapped, os.path.join(_dd, "opened_solid.brep"))
        return _reject("the bay cut left the compartment SOLID - the boolean "
                       "silently failed; trying the next rung")
    if _skin_breached(opened):
        return _reject("the bay cut broke through the BELLY - the "
                       "compartment floor is gone; trying the next rung")
    if len(opened.Shells()) != 1:
        # one shell == you can reach in. This is the test that lifting the lid
        # actually opens the compartment rather than revealing a membrane.
        return _reject(f"opened bay has {len(opened.Shells())} shells, want 1")

    hollow = None
    if one_piece:
        # The one-piece body is the single-solid export: the canopy stays
        # attached and the compartment is sealed, so it is hollowed with the
        # PLAIN compartment. A seat lip only means something to a canopy that
        # comes off, and leaving it out here removes a boolean that buys the
        # user nothing.
        with _stage("airframe.cut(one-piece)"):
            hollow = _one_valid_solid(heal(airframe.cut(cavity)))
            if hollow is not None and _void_absent(hollow):
                hollow = _one_valid_solid(
                    heal(airframe.copy().cut(cavity.copy())))
        if hollow is None:
            return _reject("one-piece cut did not leave one valid solid")
        if _void_absent(hollow):
            return _reject("the one-piece cut left the compartment solid")
        if len(hollow.Shells()) != 2:
            # a sealed internal void is TWO shells. If it is one, the cavity
            # broke out through the skin somewhere it should not have.
            return _reject(f"hollow has {len(hollow.Shells())} shells, want 2"
                           " - the cavity broke through the skin")

    with _stage("mesh checks"):
        if not tessellates_cleanly(opened, area_min=area_ratio_min):
            # One face the mesher refuses is not always a bad face: on the
            # bwb at the 675 box the cut result carried 1/41 untriangulated
            # faces in memory, and the SAME solid written to BRep and read
            # back meshed 41/41 at area ratio 1.000 - the round trip
            # normalizes the per-face tolerances the boolean left behind
            # (the same rescue `_round_tripped` already gives the operands).
            # Retry on the result before throwing the rung away; the void
            # and floor gates are re-asked on the rebuilt solid.
            with _stage("mesh retry (BRep round-trip of the result)"):
                cand = _one_valid_solid(_round_tripped(opened))
                if (cand is not None and _cut_ok(cand)
                        and len(cand.Shells()) == 1
                        and tessellates_cleanly(cand,
                                                area_min=area_ratio_min)):
                    opened = cand
                    bay["mesh_round_tripped"] = True
        if not tessellates_cleanly(opened, area_min=area_ratio_min):
            if os.environ.get("AEROFORGE_HATCH_DUMP"):
                from OCP.BRepTools import BRepTools as _BT
                _dd = os.environ["AEROFORGE_HATCH_DUMP"]
                os.makedirs(_dd, exist_ok=True)
                _BT.Write_s(opened.wrapped, os.path.join(_dd, "opened.brep"))
                _BT.Write_s(cutter.wrapped, os.path.join(_dd, "cutter.brep"))
                _BT.Write_s(airframe.wrapped,
                            os.path.join(_dd, "airframe.brep"))
            return _reject("the opened airframe would not mesh completely")
        if hollow is not None and not tessellates_cleanly(
                hollow, area_min=area_ratio_min):
            return _reject("the one-piece body would not mesh completely")

    onepiece = hollow
    scribe = None
    if hollow is not None and scribe_line:
        # The scribe is a separate cut, never fused into the cutter: it is a
        # GROOVE that stops short of the ceiling, so it is disjoint from the
        # cavity and their union is two solids, not one.
        #
        # And it is ADVISORY, which is why it is OFF by default. It draws
        # the canopy's parting line on the one-piece body and nothing else;
        # the groove is a long shallow spline-on-spline trim and OCC declines
        # to mesh 6-20% of the result on three of the four planforms measured,
        # so most of the time this is 15 s spent on a line that is then
        # discarded. Losing it is a cosmetic disappointment; paying for it on
        # every design is not.
        with _stage("scribe"):
            scribe = _build_scribe(stations, ys, plan_ap, wall)
        if scribe is not None:
            with _stage("scribe cut"):
                cand = _one_valid_solid(heal(hollow.cut(scribe)))
                if cand is not None and tessellates_cleanly(
                        cand, area_min=area_ratio_min):
                    onepiece = cand
                elif _TRACE:
                    print("    [hatch] scribe line dropped (would not mesh)",
                          flush=True)
    res.scribe = scribe

    # --- and only now, the canopy ------------------------------------------
    # From a FRESH COPY of the airframe. `copy()` drops the cached
    # triangulation the mesh checks just wrote onto shapes this one shares
    # faces with, and a boolean against a tessellated shape is ~25x slower.
    #
    # The airframe is identical whether or not the canopy is built - the pads
    # are already carved - so a caller who only wants the one-piece body can
    # skip this and get the same aircraft.
    lid = None
    lid_mesh = 0.0
    if canopy:
        z_plug = float(np.min([np.min(t.prof_crown) for t in stations])) \
            - skirt_h - 5.0
        with _stage("canopy"):
            lid = _build_lid(airframe.copy(), plan_lid, plan_skirt_in,
                             stations, ys, z_plug, z_top, skirt_h, lid_t)
        if lid is None:
            return _reject("the canopy would not build")
        if mag is not None:
            with _stage("canopy magnet lugs"):
                fitted = _fit_magnets(lid, mag, lid_t)
            if fitted is not None:
                lid = fitted
            else:
                # The airframe already carries its pads. Say so rather than
                # pretending the pair exists.
                bay["magnets"] = ("airframe pads only "
                                  "(canopy lugs would not fuse)")

        # IS THERE ACTUALLY A CANOPY THERE? The underside is cut with a band
        # interpolated through sampled crown heights, and between samples that
        # band can ride ABOVE the airframe's own lofted skin - which punches
        # windows straight through the top of the canopy. The result is still
        # one valid closed solid that meshes perfectly and seats correctly, so
        # every other test here passes it; only the volume gives it away.
        # A tray must weigh at least its own top skin.
        skin_min = _plan_area(plan_lid) * lid_t
        if lid.Volume() < 0.90 * skin_min:
            return _reject(
                f"canopy is {lid.Volume() / 1000:.1f} cm3 but its top skin "
                f"alone should be {skin_min / 1000:.1f} cm3 - the underside "
                f"cut broke through")

        with _stage("canopy mesh check"):
            # The canopy is small and carries 6 mm pockets, so it is meshed
            # FINE and judged on the per-face test with a loose area floor -
            # see `tessellates_cleanly`.
            if not tessellates_cleanly(lid, 0.2):
                return _reject("the canopy would not mesh completely")
            lid_mesh = tess_ratio(lid, 0.2)
        bay["lid_skin_min_cm3"] = round(skin_min / 1000.0, 2)

    # The seat as a standalone body, for a caller that wants to look at it or
    # render it. OPT-IN, because it is worth ~12 s and NOTHING consumes it:
    # `cutter` already puts the seat in the airframe, by not claiming it.
    ledge = None
    if seat_ring:
        with _stage("seat ring (informational)"):
            ledge = _biggest_solid(heal(cavity.cut(void)), 20.0)
            if ledge is not None and (not ledge.isValid()
                                      or ledge.Volume() <= 0):
                ledge = None

    if lid is not None:
        lid_bb = lid.BoundingBox()
        bb_vol = max(lid_bb.xlen * lid_bb.ylen * lid_bb.zlen, 1.0)
        bay["lid_volume_cm3"] = round(lid.Volume() / 1000.0, 2)
        bay["lid_bbox_mm"] = (round(lid_bb.xlen, 1), round(lid_bb.ylen, 1),
                              round(lid_bb.zlen, 1))
        bay["lid_fill_frac"] = round(lid.Volume() / bb_vol, 3)
        bay["lid_shells"] = len(lid.Shells())
        bay["lid_mesh_ratio"] = round(lid_mesh, 4)
    bay["mesh_ratio"] = round(tess_ratio(opened), 4)
    bay["area_ratio_min"] = area_ratio_min

    res.lid, res.ledge = lid, ledge
    res.airframe, res.airframe_onepiece = opened, onepiece
    return res


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------

# The fallback ladder. Each rung is (length factor, width factor). A bay that
# will not cut cleanly at full size usually will at 90%, and giving the user a
# slightly smaller compartment beats giving them an airframe with a hole in it.
_LADDER: tuple[tuple[float, float], ...] = (
    (1.00, 1.00), (1.00, 0.90), (0.94, 0.82), (0.88, 0.72),
    (0.80, 0.62), (0.72, 0.52),
)


def build_bay(wing: Any, *, bay_start: float, bay_length: float,
              bay_half_width: float, wall: float,
              x_max: float | None = None, shrink: float = 1.0,
              airframe: Solid | None = None,
              magnets: bool = True,
              one_piece: bool = True,
              seat_ring: bool = False,
              area_ratio_min: float = 0.985,
              canopy: bool = True,
              scribe_line: bool = False,
              cavity_extend_to: float | None = None,
              cavity_guard: dict | None = None,
              cavity_roof_cap: tuple | None = None) -> BayResult:
    """Build the equipment bay and its removable canopy. All lengths in mm.

    Parameters
    ----------
    wing
        Anything exposing `section`, `crown_z`, `keel_z`, `xc_at`, `half`,
        `fb`, `tc` - see the module docstring.
    bay_start, bay_length, bay_half_width
        The compartment the physics asked for, as an outer envelope. The bay
        is trimmed INTO this, never grown out of it.
    wall
        Printed wall thickness. Every face of the compartment is pulled in by
        this, and it sets the canopy's skin and skirt thickness.
    x_max
        Hard aft limit. A centre fin's root is buried in the body, so the bay
        has to stop short of it or the fin ends up standing inside the
        compartment.
    shrink
        Global scale on the half width before the ladder runs. 1.0 normally.
    airframe
        The airframe solid BEFORE any bay cut. Optional, but without it the
        canopy cannot be built (its top skin is selected out of the airframe,
        which is what makes it flush) and none of the cuts can be verified.
    magnets
        Cut pockets for a pair of magnet discs (Ø MAGNET_DIA_MM, exactly, x
        MAGNET_H_MM). Silently skipped if the pads do not build cleanly - the
        compartment matters, the retention is a convenience.
    one_piece
        Also produce `airframe_onepiece`, the sealed single-solid body with the
        canopy still attached and its outline scribed, for the STL export. It
        costs a second big boolean plus the scribe; pass False if only the
        separable parts are wanted.
    seat_ring
        Also isolate the seat lip as `BayResult.ledge`. OFF by default: it is
        an extra boolean against the whole compartment for a body nothing
        consumes - `cutter` already puts the seat in the airframe.
    canopy
        Build `BayResult.lid`. Pass False on a one-piece-only build: the
        airframe is IDENTICAL either way (the magnet pads are carved into the
        cutter, not derived from the canopy), so the two calls agree, and this
        one skips ~11 s of work whose only output is a part you are not
        exporting.
    scribe_line
        Cut the canopy's parting line into the one-piece body. OFF by default:
        it costs ~15 s and OCC refuses to mesh the result on most planforms, in
        which case it is discarded anyway.
    area_ratio_min
        How much of the airframe's BRep area must actually come out as
        triangles. Checked on the opened airframe AND on the one-piece body,
        every rung, alongside the per-face test. Both are needed: a face can
        be given a triangulation that covers only part of it, which the
        per-face test cannot see, and that shipped an airframe at 96.9%
        coverage with 51 mm of the aircraft missing from its STL. The canopy is
        judged separately and more loosely - see `tessellates_cleanly`.

    Returns
    -------
    BayResult - falsy if no rung of the ladder produced something buildable,
    in which case `bay_mm["reason"]` and `bay_mm["tried"]` say why.

    Notes
    -----
    Almost all the time is OCC boolean and meshing work. A rung is ordered so
    that everything able to reject it happens before the canopy is built, and
    the scribe line is only cut once a rung has otherwise passed - so a
    rejected rung is much cheaper than an accepted one. Set
    `AEROFORGE_HATCH_TRACE=1` for a per-stage breakdown and the reason each
    rung was rejected.
    """
    if bay_length <= 4.0 * wall or bay_half_width <= 2.0 * wall:
        return BayResult(bay_mm={"reason": "requested bay is smaller than its "
                                           "own walls"})
    x0 = float(bay_start) + wall
    x1 = float(bay_start) + float(bay_length) - wall
    if x_max is not None:
        x1 = min(x1, float(x_max))
        # `x_max` caps the HATCH only. The cavity may continue under
        # whatever imposed it (a centre fin's buried root) with its roof
        # held below it - see `cavity_roof_cap`. Capping the cavity here
        # is what sent every centre-fin design back to the box galleries.
    hw0 = max(float(bay_half_width) * float(shrink) - wall, 0.0)
    if x1 - x0 < _MIN_BAY_LEN_MM or hw0 < _MIN_BAY_HALF_W_MM:
        return BayResult(bay_mm={"reason": "no room between the walls"})

    xm = 0.5 * (x0 + x1)
    tried: list[str] = []
    _LAST_REJECT.clear()
    for i, (fl, fw) in enumerate(_LADDER):
        L = (x1 - x0) * fl
        a, b = xm - 0.5 * L, xm + 0.5 * L
        hw = hw0 * fw
        if b - a < _MIN_BAY_LEN_MM or hw < _MIN_BAY_HALF_W_MM:
            break
        name = f"rung {i} (len x{fl:.2f}, width x{fw:.2f})"
        try:
            # A FRESH COPY per rung. `copy()` drops the cached triangulation,
            # and a triangulated airframe makes every boolean ~25x slower - so
            # if a caller (or a previous rung's mesh check) has already meshed
            # it, this rung would crawl. Copying a 7-face spline solid is free
            # by comparison.
            base = airframe.copy() if airframe is not None else None
            res = _attempt(wing, a, b, hw, wall, base, magnets, one_piece,
                           seat_ring, area_ratio_min, canopy, scribe_line,
                           extend_to=cavity_extend_to, guard=cavity_guard,
                           roof_cap=cavity_roof_cap)
        except Exception as exc:            # never let a bay break the build
            tried.append(f"{name}: {type(exc).__name__}")
            continue
        if res is not None and res.ok:
            res.rung = name
            res.bay_mm["rung"] = name
            res.bay_mm["rungs_tried"] = i + 1
            return res
        tried.append(f"{name}: {_LAST_REJECT[-1] if _LAST_REJECT else 'no clean result'}")
    return BayResult(bay_mm={"reason": "no rung produced a buildable bay",
                             "tried": tried})
