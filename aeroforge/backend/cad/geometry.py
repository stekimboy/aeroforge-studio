"""Build the flying-wing airframe as real 3D solids (SPEC_FLYING_WING.md §5).

Frame: x aft from the nose datum (mm), y right, z up. All CAD is done in mm.

THE TOPOLOGY, in one sentence: the aircraft IS a wing. There is no fuselage,
no pod and no separate centre body - one continuous loft runs from the left
tip to the right tip and the airfoil simply gets *deeper* and *longer* as it
approaches y = 0, exactly the way a moulded RC wing (Skywalker X5, SonicModell
AR Wing, Ritewing Drak) is actually built. Everything that used to be a pod is
now the inboard part of the same surface:

    f    = |y| / (span/2)                     spanwise fraction
    fb   = body.half_width_m / (span/2)       THICKNESS blend width
    fb_p = 1.6 * fb                           PLANFORM blend width
    w    = smoothstep 1 -> 0 over [0, fb]     (thickness)
    w_p  = smoothstep 1 -> 0 over [0, fb_p]   (chord + leading edge)

    chord      c(f)  = c_wing(f) * (1 + (chord_scale - 1) * w_p)
    thickness  t(f)  = 1 + (depth_scale - 1) * w      (ordinates, NOT chord)
    LE         x(f)  = cubic Hermite from x = 0 at the centreline onto the
                       swept line `nose_ext + f*(b/2)*tan(sweep)` at f = fb_p
    crown      z(f) += (t(f) - 1) * (t/c * c(f)) * (crown_frac - 0.5)
    twist      root_incidence -> root_incidence - washout, linear in f

Two blend widths, not one, and that is deliberate: the body's VOLUME dies out
at the recorded half-width, but the planform root extension on every real
blended wing is longer and shallower than that. Squeeze the whole forward
extension into the body half-width and the nose comes out as a beak. Chord and
leading edge must share their width, though, because the trailing edge is
x_le + chord - blend them over different spans and the TE picks up a visible
W-shaped wobble across the centre section. `nose_ext` is the extra centre
chord, sent forward, and trimmed back if that would notch the centre TE ahead
of the wing-root TE.

CONSTRUCTION RULES that are not negotiable (see DECISIONS.md - each one is a
defect that was actually built before it was a rule):

- The full-span loft is built in FIVE axial segments (left tip / left panel /
  one centre segment running right through y = 0 / right panel / right tip)
  that SHARE their boundary wires. One loft across the whole span makes OCC's
  through-sections overshoot, and one full-length spline face trimmed by
  booleans is exactly the face the mesher silently refuses to tessellate -
  that produced a real hole in the skin.
- Segment boundaries go where the section is C1 (`fb_p`, `_TIP_START`), never
  at `fb` where the planform is still curving, and never at y = 0.
- Stations are EVENLY spaced inside a segment. OCC parameterises through-
  sections on the section index, so bunching stations makes the surface bulge
  between them; cluster by adding segments instead.
- `_heal` (ShapeFix) runs after every fuse and after every cut.
- Never `.clean()`: merging boolean split faces re-creates the untessellatable
  spline faces.
- Sections carry a finite trailing edge (`open_te`). Knife edges tessellate
  badly and cannot be built.
- Vertical surfaces are separate bolted-on parts (real wings bolt them on) but
  their roots are buried inside the skin so the fuse yields exactly one solid.

Public API (stable):
    build_design_parts(design) -> ({name: Solid}, meta)   # STEP assembly
    build_design_solid(design) -> (Solid, meta)           # STL / preview
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Callable

import numpy as np
from cadquery import Edge, Solid, Vector, Wire
from OCP.BRepBuilderAPI import BRepBuilderAPI_GTransform
from OCP.gp import gp_GTrsf

from ..progress import report as _progress

MM = 1000.0

# Station tuple: (chord_mm, twist_deg, leading-edge Vector). Kept as a 3-tuple
# for backward compatibility; the blended wing carries its thickness scale
# alongside (see _Section).
Station = tuple[float, float, Vector]


# ---------------------------------------------------------------------------
# Airfoil coordinates
# ---------------------------------------------------------------------------

try:  # the physics library is the source of truth for section shapes
    from ..physics.airfoils import LIBRARY as _AF_LIBRARY, coordinates_for as _af_coords
except Exception:  # pragma: no cover - only if physics is mid-rewrite
    _AF_LIBRARY, _af_coords = None, None


def _fallback_coords(n: int, thickness: float = 0.09,
                     reflex: bool = True) -> np.ndarray:
    """Analytic section used only if the airfoil library cannot be imported.

    Reflexed camber y_c = m x (1-x)(1 - r x) (positive Cm0, see airfoils.py)
    on the NACA 4-digit thickness distribution
    y_t = 5t(0.2969 sqrt(x) - 0.1260 x - 0.3516 x^2 + 0.2843 x^3 - 0.1036 x^4).
    """
    beta = np.linspace(0.0, math.pi, n)
    x = 0.5 * (1 - np.cos(beta))
    m, r = (0.025, 2.4) if reflex else (0.0, 0.0)
    yc = m * x * (1 - x) * (1 - r * x)
    dyc = m * ((1 - 2 * x) * (1 - r * x) - r * x * (1 - x))
    yt = 5 * thickness * (0.2969 * np.sqrt(x) - 0.1260 * x - 0.3516 * x ** 2
                          + 0.2843 * x ** 3 - 0.1036 * x ** 4)
    th = np.arctan(dyc)
    xu, yu = x - yt * np.sin(th), yc + yt * np.cos(th)
    xl, yl = x + yt * np.sin(th), yc - yt * np.cos(th)
    return np.stack([np.concatenate([xu[::-1], xl[1:]]),
                     np.concatenate([yu[::-1], yl[1:]])], axis=1)


def open_te(coords: np.ndarray, thickness_frac: float) -> np.ndarray:
    """Open a knife-edge section to a finite trailing-edge thickness.

    The generated sections close to a mathematical point at x/c = 1. Lofting
    through knife edges leaves a degenerate surface strip that the mesher
    tessellates badly (visible nicks along the trailing edge) and that no
    builder could produce anyway - a moulded or printed wing has 1-3 mm of
    trailing edge. The opening grows linearly from zero at the leading edge,
    so camber and thickness forward of ~50% chord are untouched.
    """
    if thickness_frac <= 0.0:
        return coords
    out = coords.copy()
    n = (len(coords) + 1) // 2
    half = 0.5 * thickness_frac
    out[:n, 1] += half * out[:n, 0]           # upper surface, TE -> LE
    out[n:, 1] -= half * out[n:, 0]           # lower surface, LE -> TE
    return out


def _airfoil_pts(name: str, n: int = 51) -> np.ndarray:
    """Closed section loop (Selig order TE -> upper -> LE -> lower -> TE)."""
    if _AF_LIBRARY is not None and name in _AF_LIBRARY:
        return _af_coords(_AF_LIBRARY[name], n)
    sym = bool(name) and ("0008" in name or "symmetric" in name.lower())
    return _fallback_coords(n, 0.08 if sym else 0.09, reflex=not sym)


# ---------------------------------------------------------------------------
# Section wires and lofted surfaces
# ---------------------------------------------------------------------------

def section_wire(coords: np.ndarray, chord_mm: float, twist_deg: float,
                 le: Vector, tdir: Vector, t_scale: float = 1.0) -> Wire:
    """Closed airfoil wire at leading-edge position `le`, chordwise +x,
    thickness along unit vector `tdir`. Twist rotates about the quarter-chord
    in the (x, tdir) plane (positive = leading edge up).

    `t_scale` multiplies the airfoil ORDINATES only, so a section can be made
    deeper without changing its chord - that is what turns the inboard wing
    into the centre body. Default 1.0 keeps every existing caller unchanged.
    """
    th = math.radians(twist_deg)
    ct, st = math.cos(th), math.sin(th)
    pts = []
    for xc, tc0 in coords:
        tc = tc0 * t_scale
        xr = (xc - 0.25)
        # rotate (xr, tc) by -twist about quarter chord: LE-up positive
        xrot = xr * ct + tc * st
        trot = -xr * st + tc * ct
        pts.append(Vector(
            le.x + (0.25 + xrot) * chord_mm,
            le.y + trot * chord_mm * tdir.y,
            le.z + trot * chord_mm * tdir.z,
        ))
    if (pts[0] - pts[-1]).Length > 1e-9:
        pts.append(pts[0])
    spline = Edge.makeSpline(pts[:-1] + [pts[0]])
    return Wire.assembleEdges([spline])


def loft_surface(sections: list[Wire], ruled: bool = True) -> Solid:
    return Solid.makeLoft(sections, ruled)


def _foil_surf_t(coords: np.ndarray, xc: float, upper: bool = True) -> float:
    """Interpolated surface ordinate t/c at chord fraction xc (Selig order:
    TE -> upper -> LE -> lower -> TE)."""
    n = (len(coords) + 1) // 2
    arr = coords[:n][::-1] if upper else coords[n - 1:]
    return float(np.interp(xc, arr[:, 0], arr[:, 1]))


def _max_thickness(coords: np.ndarray) -> float:
    """Section max thickness / chord (upper minus lower at the same x)."""
    xs = np.linspace(0.02, 0.98, 60)
    return float(max(_foil_surf_t(coords, x, True) - _foil_surf_t(coords, x, False)
                     for x in xs))


def _foil_point(chord: float, twist_deg: float, le: Vector, tdir: Vector,
                xc: float, tc: float, t_scale: float = 1.0) -> Vector:
    """3D position of airfoil point (xc, tc) using the section_wire mapping."""
    th = math.radians(twist_deg)
    ct, st = math.cos(th), math.sin(th)
    tc = tc * t_scale
    xr = xc - 0.25
    xrot = xr * ct + tc * st
    trot = -xr * st + tc * ct
    return Vector(le.x + (0.25 + xrot) * chord,
                  le.y + trot * chord * tdir.y,
                  le.z + trot * chord * tdir.z)


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


def _smoothstep(u: float) -> float:
    """C1 Hermite step, 0 at u=0 and 1 at u=1 with zero slope at both ends."""
    u = _clamp(u, 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


# ---------------------------------------------------------------------------
# The blended full-span wing
# ---------------------------------------------------------------------------

@dataclass
class _Section:
    """One spanwise station of the blended wing."""
    chord: float
    twist: float
    le: Vector
    t_scale: float

    def as_station(self) -> Station:
        return self.chord, self.twist, self.le


_TIP_START = 0.94     # span fraction where the rounded tip cap begins
_TIP_CAP = 0.38       # chord at the very tip, as a fraction of the tip chord
_TWIST_EPS = 0.035    # softens the |y| twist/dihedral kink at the centreline


@dataclass
class _BlendedWing:
    """Section generator for the one-piece airframe (SPEC §5).

    `s` is the SIGNED span fraction: -1 at the left tip, 0 at the centreline,
    +1 at the right tip. Everything the CAD needs - the loft wires, the fin
    mounting points, the hinge lines, the CG keel - is derived from
    :meth:`section`, so there is a single source of truth for the shape.
    """
    coords: np.ndarray
    half: float               # semi-span, mm
    c_root: float             # WING root chord (at the body/wing joint), mm
    c_tip: float
    sweep_deg: float
    dihedral_deg: float
    incidence: float
    washout: float
    fb: float                 # body half-width / semi-span
    depth_scale: float
    chord_scale: float
    nose_round: float
    crown_frac: float
    x_offset: float = 0.0
    tc: float = field(init=False)
    nose_ext: float = field(init=False)
    fb_p: float = field(init=False)
    tan_sweep: float = field(init=False)
    tan_dih: float = field(init=False)

    def __post_init__(self) -> None:
        self.tc = _max_thickness(self.coords)
        self.tan_sweep = math.tan(math.radians(self.sweep_deg))
        self.tan_dih = math.tan(math.radians(self.dihedral_deg))
        # TWO blend widths, and this is the whole trick to a fair surface:
        #  * `fb`   - the THICKNESS blend, straight from body.half_width_m.
        #             It says where the body's volume dies out.
        #  * `fb_p` - the PLANFORM blend, ~1.6x wider, shared by the chord
        #             scaling AND the leading-edge extension.
        # They must be shared, because the trailing edge is x_le + chord: give
        # the LE a wider blend than the chord and the TE picks up a visible
        # W-shaped wobble across the centre section. And the planform blend
        # has to be wider than the thickness blend, because a real root
        # extension is long and shallow - squeeze the whole forward extension
        # into the body half-width and the nose comes out as a beak.
        self.fb_p = _clamp(1.6 * self.fb, self.fb, 0.70)
        extra = max(self.chord_scale - 1.0, 0.0) * self.c_root
        self.nose_ext = self._fair_nose_ext(extra)

    def _fair_nose_ext(self, extra: float) -> float:
        """How far the centre leading edge reaches ahead of the swept wing LE.

        All of the extra centre chord goes forward by default, which leaves
        the trailing edge essentially straight. Where taper and sweep would
        still push the TE at some inboard station BEHIND the centre TE - a
        scallop, which reads as damage rather than design - back the extension
        off until the notch is under 2% of the centre chord."""
        if extra <= 0.0:
            return 0.0
        e = extra
        c0 = self.c_root * self.chord_scale
        for _ in range(8):
            worst, arg = 0.0, 0.0
            for i in range(1, 41):
                f = self.fb_p * i / 40.0
                over = self._x_te(f, e) - c0
                if over > worst:
                    worst, arg = over, f
            if worst <= 0.02 * c0:
                break
            relax = max(1.0 - self._w_p(arg), 0.08)
            e = max(e - worst / relax, 0.45 * extra)
        return e

    def _w_p(self, f: float) -> float:
        """Planform blend (chord + leading edge)."""
        if self.fb_p <= 1e-6:
            return 0.0
        return 1.0 - _smoothstep(min(f / self.fb_p, 1.0))

    def chord(self, f: float) -> float:
        """Chord at span fraction f: linear taper, scaled up by the planform
        blend so the centre section is long enough to BE the fuselage."""
        c_lin = self.c_root + (self.c_tip - self.c_root) * f
        return c_lin * (1.0 + (self.chord_scale - 1.0) * self._w_p(f))

    def _x_te(self, f: float, nose_ext: float) -> float:
        return self._x_le(f, nose_ext) + self.chord(f)

    # -- blend functions ----------------------------------------------------
    def w(self, f: float) -> float:
        """Body THICKNESS blend: 1 on the centreline, 0 outboard of the body."""
        if self.fb <= 1e-6:
            return 0.0
        return 1.0 - _smoothstep(min(f / self.fb, 1.0))

    def _x_le(self, f: float, nose_ext: float) -> float:
        """Leading-edge x at span fraction f (before the tip round-off).

        Outboard of the planform blend it is the plain swept line, offset aft
        by `nose_ext`. Inboard it is a cubic Hermite that meets that line with
        matching slope at f = fb_p and reaches x = 0 at the centreline - so
        the LE curves smoothly forward into the nose instead of meeting it in
        a dart-like point. `nose_round` sets the slope at the centreline: 1.0
        gives a round snout (zero slope, X5 / AR-Wing), 0.0 a sharp one.
        """
        straight = nose_ext + f * self.half * self.tan_sweep
        if self.fb_p <= 1e-6 or f >= self.fb_p:
            return straight
        u = f / self.fb_p
        p1 = nose_ext + self.fb_p * self.half * self.tan_sweep
        m1 = self.fb_p * self.half * self.tan_sweep
        m0 = (1.0 - _clamp(self.nose_round, 0.0, 1.0)) * 0.20 * p1
        u2, u3 = u * u, u * u * u
        return ((-2 * u3 + 3 * u2) * p1 + (u3 - u2) * m1
                + (u3 - 2 * u2 + u) * m0)

    def x_le_planform(self, f: float) -> float:
        return self._x_le(f, self.nose_ext) + self.x_offset

    def solve_nose_guard(self) -> None:
        """Push the whole aircraft aft by however far the centre section's
        spline bulges ahead of its own leading-edge point.

        The section wire is one B-spline through the coordinate list; around
        the near-cusp at the leading edge it overshoots the interpolated
        points by a few tens of microns. Small, but the nose datum is where
        every CG, neutral-point and station number the user sees is measured
        from, so `x >= 0` has to be true of the built surface, not just of the
        control points. Measured on the real curve, not guessed."""
        self.x_offset = 0.0
        try:
            edge = self.wire(0.0).Edges()[0]
            xs = [edge.positionAt(t, mode="parameter").x
                  for t in np.linspace(0.0, 1.0, 400)]
            # +10 um also absorbs the float32 rounding in a binary STL
            self.x_offset = max(0.0, -min(xs)) + 0.01
        except Exception:
            self.x_offset = 0.05

    def _g(self, f: float) -> float:
        """Smoothed |y|: equals f to within ~3% but has zero slope at f = 0, so
        the twist (and dihedral) reversal at the centreline is a small fillet
        instead of a crease down the middle of a moulded wing. Normalised so
        the tip still gets exactly the full washout."""
        e = _TWIST_EPS
        return (math.hypot(f, e) - e) / (math.hypot(1.0, e) - e)

    # -- the section --------------------------------------------------------
    def section(self, s: float) -> _Section:
        f = min(abs(s), 1.0)
        c = self.chord(f)
        t_body = 1.0 + (self.depth_scale - 1.0) * self.w(f)
        t_scale = t_body
        x_le = self.x_le_planform(f)
        g = self._g(f)
        twist = self.incidence - self.washout * g
        # Rounded moulded tip: over the last few percent of span the chord
        # follows an elliptical arc down to a cap while the LE sweeps aft and
        # the section thins, so the outline curves shut. The arc SCALES the
        # continuing taper rather than replacing it, and sqrt(1-u^2) has zero
        # slope at u = 0, so chord, LE and thickness all stay C1 where the
        # round-off starts - freeze the chord instead and the planform gets a
        # visible corner right where the tip begins.
        if f > _TIP_START:
            u = (f - _TIP_START) / (1.0 - _TIP_START)
            r = max(math.sqrt(max(1.0 - u * u, 0.0)), _TIP_CAP)
            x_le += 0.42 * c * (1.0 - r)
            c *= r
            t_scale *= r ** 0.55
        # The crown offset belongs to the BODY blend only - the tip round-off
        # must not drag the outboard sections off the wing plane.
        z_c = (t_body - 1.0) * (self.tc * c) * (self.crown_frac - 0.5)
        z_le = self.half * g * self.tan_dih + z_c
        return _Section(c, twist, Vector(x_le, s * self.half, z_le), t_scale)

    def station(self, s: float) -> Station:
        return self.section(s).as_station()

    def wire(self, s: float) -> Wire:
        sec = self.section(s)
        # finite trailing edge: ~0.4% of the local chord, never below 0.8 mm
        # (buildable) and never above 3 mm (still a trailing edge)
        te = _clamp(0.004 * sec.chord, 0.8, 3.0) / max(sec.chord, 1e-6)
        return section_wire(open_te(self.coords, te / max(sec.t_scale, 1e-6)),
                            sec.chord, sec.twist, sec.le,
                            Vector(0, 0, 1), sec.t_scale)

    # -- surface queries (fin roots, hinge lines, CG keel) ------------------
    def surf_point(self, s: float, xc: float, upper: bool = True) -> Vector:
        sec = self.section(s)
        t = _foil_surf_t(self.coords, _clamp(xc, 0.0, 1.0), upper)
        return _foil_point(sec.chord, sec.twist, sec.le, Vector(0, 0, 1),
                           _clamp(xc, 0.0, 1.0), t, sec.t_scale)

    def crown_z(self, s: float, xc: float) -> float:
        return self.surf_point(s, xc, True).z

    def keel_z(self, s: float, xc: float) -> float:
        return self.surf_point(s, xc, False).z

    def xc_at(self, s: float, x_mm: float) -> float:
        sec = self.section(s)
        return _clamp((x_mm - sec.le.x) / max(sec.chord, 1e-6), 0.0, 1.0)

    def extents(self) -> dict[str, float]:
        """Analytic bounding extents of the lofted surface (mm)."""
        xs, zs = [], []
        for i in range(81):
            s = i / 80.0
            for xc in np.linspace(0.0, 1.0, 17):
                p_u = self.surf_point(s, float(xc), True)
                p_l = self.surf_point(s, float(xc), False)
                xs += [p_u.x, p_l.x]
                zs += [p_u.z, p_l.z]
        return {"x_min": min(xs), "x_max": max(xs),
                "z_min": min(zs), "z_max": max(zs)}


# -- station distribution ----------------------------------------------------
# 43 distinct sections across the span, well over the 19 the spec asks for,
# and clustered where the curvature is: 23 of them across the blended centre
# section (which is where the body blend AND the leading-edge root extension
# both live), 6 more per side along the outboard panel, 5 closing each tip.
_TIP_U = (0.26, 0.52, 0.74, 0.90, 1.0)
_BODY_N = 11          # intervals per side across the blended centre section
_PANEL_N = 5          # intervals from the blend edge out to the tip cap


def _station_groups(fb: float, fb_p: float) -> list[list[float]]:
    """The axial loft segments, as lists of SIGNED span fractions. Adjacent
    groups share their boundary station exactly, which keeps every trimmed
    face simple enough for the mesher and stops OCC's through-sections
    overshooting across the whole span in one go.

    Three rules, each of which was a visible defect before it was one:

    1. The CENTRE segment runs right through the centreline, -fb_p to +fb_p,
       in a single loft. Split it at y = 0 and the two halves meet with
       mirrored - therefore mismatched - spanwise tangents: a hard crease
       straight down the spine of what should be a moulded surface.
    2. Segment boundaries sit at `fb_p` and `_TIP_START`, the two stations
       where the section is C1 (both blends have died out with zero slope and
       the tip round-off has not started). Putting a boundary at `fb`, where
       the planform blend is still curving, leaves the two lofts with
       genuinely different tangents and draws a panel line down each wing.
    3. Stations are EVENLY SPACED inside each segment; clustering comes from
       adding segments, never from bunching stations. OCC parameterises the
       through-sections surface on the section INDEX, not on distance, so a
       segment whose spacing grows 7x from end to end interpolates at wildly
       varying speed and bulges between sections - a graded outboard segment
       put the bell planform's skin 104 mm behind its own trailing edge.
    """
    right = [
        [fb_p + (_TIP_START - fb_p) * k / _PANEL_N
         for k in range(_PANEL_N + 1)],
        [_TIP_START] + [_TIP_START + (1.0 - _TIP_START) * u for u in _TIP_U],
    ]
    left = [[-s for s in reversed(grp)] for grp in right]
    half = [fb_p * k / _BODY_N for k in range(_BODY_N + 1)]
    centre = [-s for s in reversed(half)] + half[1:]
    return list(reversed(left)) + [centre] + right


def _blended_airframe(wing: _BlendedWing) -> Solid:
    """The one-piece airframe: left tip to right tip, no seam, no pod."""
    cache: dict[float, Wire] = {}

    def wire(s: float) -> Wire:
        key = round(s, 9)
        if key not in cache:
            cache[key] = wing.wire(s)
        return cache[key]

    solid: Solid | None = None
    for group in _station_groups(wing.fb, wing.fb_p):
        seg = Solid.makeLoft([wire(s) for s in group], False)
        solid = seg if solid is None else solid.fuse(seg)
    return _heal(solid)


# ---------------------------------------------------------------------------
# Vertical surfaces (winglets, twin fins, centre fin)
# ---------------------------------------------------------------------------

def _rounded_surface(
    *, airfoil: str, span_mm: float, c_root_mm: float, c_tip_mm: float,
    le_root: Vector, sweep_le_deg: float, dihedral_deg: float,
    twist_root_deg: float, twist_tip_deg: float,
    span_dir: Vector, tdir: Vector, mirror_y: bool = False,
    tip_start: float = 0.88, tip_cap: float = 0.26,
) -> tuple[Solid, Callable[[float], Station]]:
    """Smooth multi-section lofted panel with a rounded tip - used for every
    vertical surface. Two lofts sharing the section at `tip_start`, each with
    its own near-uniform station spacing: put the tightly spaced tip sections
    in the same loft as the widely spaced inboard ones and OCC's
    through-sections overshoots between them (see `_station_groups`)."""
    coords = _airfoil_pts(airfoil)
    tan_sw = math.tan(math.radians(sweep_le_deg))
    tan_dh = math.tan(math.radians(dihedral_deg))
    s = span_mm
    sd = Vector(span_dir.x, -span_dir.y if mirror_y else span_dir.y,
                span_dir.z)

    def station(f: float) -> Station:
        c_lin = c_root_mm + (c_tip_mm - c_root_mm) * f
        tw = twist_root_deg + (twist_tip_deg - twist_root_deg) * f
        le = Vector(le_root.x + f * s * tan_sw,
                    le_root.y + sd.y * f * s,
                    le_root.z + sd.z * f * s + abs(sd.y) * f * s * tan_dh)
        if f > tip_start:
            c_base = c_root_mm + (c_tip_mm - c_root_mm) * tip_start
            u = (f - tip_start) / max(1.0 - tip_start, 1e-9)
            c = max(c_base * math.sqrt(max(1.0 - u * u, 0.0)), tip_cap * c_base)
            return c, tw, le + Vector(0.40 * (c_base - c), 0, 0)
        return c_lin, tw, le

    def wire(f: float) -> Wire:
        c, tw, le = station(f)
        te = _clamp(0.005 * c, 0.7, 2.0) / max(c, 1e-6)
        return section_wire(open_te(coords, te), c, tw, le, tdir)

    fr_tip = 1.0 - tip_start
    inboard = [0.0, 0.45 * tip_start, tip_start]
    tip = [tip_start, tip_start + 0.45 * fr_tip, tip_start + 0.80 * fr_tip, 1.0]
    w_in = [wire(f) for f in inboard]
    w_tip = [wire(f) for f in tip]
    return Solid.makeLoft(w_in, False).fuse(Solid.makeLoft(w_tip, False)), station


def _clamp_aft(x_le: float, c_root: float, span_mm: float, sweep_deg: float,
               taper: float, x_aft: float,
               tip_start: float = 0.88) -> tuple[float, float]:
    """Shift a fin's LE forward / relax its sweep so no section's trailing edge
    extends past station x_aft (keeps the recorded overall length honest)."""
    x_le = min(x_le, x_aft - c_root)
    c90 = c_root * (1.0 + (taper - 1.0) * tip_start)
    tan_lim = (x_aft - x_le - c90) / max(tip_start * span_mm, 1.0)
    sw = min(sweep_deg, math.degrees(math.atan(max(tan_lim, 0.0))))
    return x_le, sw


# ---------------------------------------------------------------------------
# Canopy / hatch fairing and CG marker
# ---------------------------------------------------------------------------

# how far the hatch fairing stands proud of the crown, as a fraction of the
# centre body depth (kept small on purpose - see _canopy_fairing)
_CANOPY_RISE = 0.085
# Below this the hatch is a flush panel on a deep body, not a blister: there is
# nothing to see and the near-tangent boolean is a liability.
_CANOPY_MIN_RISE_MM = 2.5


def _ellipse_wire(x: float, a: float, b: float, zc: float = 0.0) -> Wire:
    a, b = max(a, 1.2), max(b, 1.2)
    pts = [Vector(x, a * math.cos(t), zc + b * math.sin(t))
           for t in np.linspace(0, 2 * math.pi, 33)[:-1]]
    return Wire.assembleEdges([Edge.makeSpline(pts + [pts[0]])])



# Angular deflection used with the linear one when meshing. This is what
# `exporters.write_stl_verified` ships the STL at (0.25 in its default, 0.12
# for previews) and what `hinges._MESH_ANGULAR` already measures against, so
# the mesh gated here is the mesh the builder actually gets. CadQuery's
# `Shape.tessellate()` quietly defaults to 0.1 rad - FINER than anything that
# is ever exported - and the gate was paying for a mesh nobody sees.
_MESH_ANGULAR = 0.25


def _meshed_area(shape, tol: float, angular: float = _MESH_ANGULAR):
    """Total triangulated area, meshed and summed inside OCC.

    A face OCC declines to mesh simply carries no triangulation and adds
    nothing to the total - which is exactly the failure this is looking for,
    so the coverage figure means the same thing it always did.

    The same primitive as `hinges._meshed_area`, which measured it at ~20x
    against `Shape.tessellate()` plus a Python loop (36.6 s -> 1.8 s on a wing
    panel) for the same coverage figure to five decimals. Returns None if any
    of the OCP chain is missing, so the caller can fall back.
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


def _tessellates_cleanly(solid: Solid, tol: float = 0.6,
                         min_ratio: float = 0.99) -> bool:
    """Does OCC actually mesh the WHOLE of this solid?

    `isValid()` is not enough and never has been: OCC will happily report a
    boolean result valid and then silently skip faces whose trim curves it
    cannot tessellate, which shows up as a hole in the exported skin. Cutting
    a groove is exactly the kind of operation that produces such a face, so a
    cut is only kept if the result still meshes completely.

    (The equipment bay used to be gated on this too. It still is - the check
    now lives in `backend.cad.hatch`, which uses a stricter per-face version:
    an area ratio false-fails on small parts, where losing one 6 mm pocket
    face costs 2% of the area and rejects a perfectly good canopy.)

    THE PROBE IS A COPY, AND THAT IS NOT AN OPTIMISATION DETAIL - IT IS THE
    WHOLE COST. `tessellate()`/`BRepMesh` WRITE the triangulation onto the
    shared TShape, and every later boolean drags it along; `hinges.py` (line
    ~588) measured the same cut at "seconds on a clean solid and the better
    part of a minute on a tessellated one". This gate runs mid-build - the
    caller does `airframe = cut` immediately afterwards and goes on cutting -
    so gating the solid itself taxed every remaining operation on the
    airframe. `Shape.copy()` copies without the triangulation and costs a
    fraction of a millisecond, so the solid that survives this check is clean.
    The verdict is unchanged: same geometry in, same coverage out.
    """
    try:
        ref = solid.Area()
        if ref <= 0:
            return False
        area = _meshed_area(solid.copy(), tol)
        if area is not None:
            return area / ref >= min_ratio
    except Exception:
        return False

    # Fallback: no OCP mesh-props chain. The original path, kept whole - it is
    # the definition this gate is measured against.
    try:
        verts, tris = solid.copy().tessellate(tol, _MESH_ANGULAR)
    except Exception:
        return False
    if not tris:
        return False
    p = np.asarray([[v.x, v.y, v.z] for v in verts], dtype=float)
    idx = np.asarray(tris, dtype=int)
    a = p[idx[:, 1]] - p[idx[:, 0]]
    b = p[idx[:, 2]] - p[idx[:, 0]]
    area = 0.5 * float(np.sum(np.linalg.norm(np.cross(a, b), axis=1)))
    ref = solid.Area()
    return ref > 0 and area / ref >= min_ratio


# ---------------------------------------------------------------------------
# Motor mount
# ---------------------------------------------------------------------------





# How far inside the body the nacelle's root section sits (fraction of the
# local half-section). Strictly less than 1 so the fuse is never tangent.
_NACELLE_EMBED = 0.93


def _ellipse_yz(x: float, a: float, b: float, yc: float, zc: float,
                n_pts: int = 45) -> Wire:
    """Closed ellipse in the plane x = const."""
    a, b = max(a, 0.6), max(b, 0.6)
    pts = [Vector(x, yc + a * math.cos(t), zc + b * math.sin(t))
           for t in np.linspace(0.0, 2.0 * math.pi, n_pts)[:-1]]
    return Wire.assembleEdges([Edge.makeSpline(pts + [pts[0]])])


def _motor_mount(wing: "_BlendedWing", spec: dict,
                 wall: float) -> tuple[Solid | None, list[Solid], float]:
    """Structural motor nacelle + its screw holes.

    THE PROBLEM THIS SOLVES: a flying wing's centre section is thin at the
    trailing edge - 9 mm deep on a 400 mm chord is typical - while the bolt
    circle for even a small outrunner needs a face ~37 mm across. Sitting a
    disc of that size on the trailing edge leaves most of it hanging in free
    air, reads as a part glued on afterwards, and collides with anything else
    that lives on the centreline (the fin, most obviously).

    What real pusher wings do instead is carry a faired NACELLE: the centre
    body keeps its depth aft and swells smoothly into the motor face. So the
    boss here is a loft that starts as the body's OWN local cross-section -
    entirely inside the skin, which is why the fuse leaves no seam - and
    morphs over a shallow taper into the circular mounting face. The mounting
    face still sits flush with the trailing edge (or the nose, on a tractor),
    so the mount never lengthens the aircraft.

    Returns (nacelle, cutters, x_root) where `x_root` is where the fairing
    leaves the body - anything else on the centreline has to clear it.
    """
    if not spec:
        return None, [], 0.0
    tractor = str(spec.get("type", "pusher")) == "tractor"
    x_face = float(spec.get("x_m", 0.0)) * MM
    y_c = float(spec.get("y_m", 0.0)) * MM
    z_c = float(spec.get("z_m", 0.0)) * MM
    r_plate = float(spec.get("plate_radius_mm", 16.0))
    t_plate = float(spec.get("plate_thickness_mm", 4.0))
    r_bolt = float(spec.get("bolt_circle_radius_mm", 13.4))
    d_screw = float(spec.get("screw_hole_d_mm", 3.2))
    d_shaft = float(spec.get("shaft_hole_d_mm", 8.0))
    n = max(int(spec.get("n_screws", 4)), 2)

    sgn = 1.0 if tractor else -1.0            # the fairing grows INTO the body
    # Long enough that the fairing is a shallow cone, not a step. The taper
    # angle is what decides whether this reads as part of the aeroplane.
    depth = max(2.2 * r_plate, 3.0 * t_plate, 6.0 * wall, 18.0)
    x_root = x_face + sgn * depth

    # the section the fairing grows out of, measured on the body itself
    xc_root = wing.xc_at(0.0, x_root)
    crown = wing.crown_z(0.0, xc_root)
    keel = wing.keel_z(0.0, xc_root)
    b_root = max(0.5 * (crown - keel), 1.0)
    z_root = 0.5 * (crown + keel)
    a_root = max(min(2.0 * r_plate, 0.9 * wing.fb * wing.half), 1.2 * b_root)

    # The first section is sampled from the BODY'S OWN SURFACE, not from an
    # ellipse approximating it. An ellipse is close but not equal to an airfoil
    # section, and the mismatch showed up as a crease all round the joint - the
    # thing that made the mount read as bolted-on rather than grown-out-of.
    # Sampling the real crown and keel means the fairing starts as the body and
    # ends as the motor face, with nothing to see in between.
    #
    # More sections than the taper strictly needs, too: with only a handful the
    # loft's own curvature between them is what you notice.
    n_pts = 49
    wires = []
    for u in np.linspace(0.0, 1.0, 11):
        s = u * u * (3.0 - 2.0 * u)                    # smoothstep
        x = x_root - sgn * u * depth
        pts = []
        for t in np.linspace(0.0, 2.0 * math.pi, n_pts)[:-1]:
            ct, st = math.cos(t), math.sin(t)
            # the body's own outline at this station, over the fairing's width
            y_b = y_c + a_root * ct
            f_b = _clamp(abs(y_b) / max(wing.half, 1e-6), 0.0, 0.95)
            xc_b = _clamp(wing.xc_at(f_b, x), 0.0, 1.0)
            z_b = (wing.crown_z(f_b, xc_b) if st >= 0.0
                   else wing.keel_z(f_b, xc_b))
            z_b = z_root + (z_b - z_root) * abs(st)
            # Pull the root section slightly INSIDE the body. Sampled exactly
            # on the surface it would be tangent to it, and a tangent fuse is
            # how OCC produces a compound instead of one solid (the aircraft
            # then exports as several disconnected pieces). Inside by a few
            # percent gives the boolean real material to work with, and the
            # fairing simply stays hidden until the body thins down to it.
            y_b = y_c + (y_b - y_c) * _NACELLE_EMBED
            z_b = z_root + (z_b - z_root) * _NACELLE_EMBED
            # the circular motor face
            y_f, z_f = y_c + r_plate * ct, z_c + r_plate * st
            pts.append(Vector(x, y_b + (y_f - y_b) * s,
                              z_b + (z_f - z_b) * s))
        wires.append(Wire.assembleEdges([Edge.makeSpline(pts + [pts[0]])]))
    if sgn > 0:
        wires.reverse()                                 # keep x increasing
    boss = Solid.makeLoft(wires, False)

    cutters: list[Solid] = []
    axis = Vector(sgn, 0, 0)
    lead = 6.0                    # start outside the face for a clean cut

    # A MOUNT SCREW IS 8-10 mm LONG. Drilling the bolt pattern as a long
    # cylinder does not just waste material - it bores straight on through
    # whatever else is in line, and on a centre-fin layout what is in line is
    # the FIN ROOT. That produced exactly the reported symptom: a hole and a
    # gap opening up behind the fin, tens of millimetres from the mount.
    # The bore now stops inside the bulkhead it is meant to thread into.
    screw_depth = _clamp(3.0 * t_plate, 9.0, 16.0)
    if n == 4:
        # the standard square pattern: holes on the corners of a square whose
        # diagonal is the bolt circle
        s = r_bolt / math.sqrt(2.0)
        offsets = [(s, s), (s, -s), (-s, s), (-s, -s)]
    else:
        offsets = [(r_bolt * math.cos(2 * math.pi * i / n),
                    r_bolt * math.sin(2 * math.pi * i / n)) for i in range(n)]
    for dy, dz in offsets:
        cutters.append(Solid.makeCylinder(
            0.5 * d_screw, lead + screw_depth,
            Vector(x_face - sgn * lead, y_c + dy, z_c + dz), axis))

    # THE CENTRE BORE ONLY HAS TO SWALLOW THE MOTOR'S SHAFT BOSS.
    #
    # It used to run `0.85 x depth` forward "for the wires", which is the same
    # mistake the screw holes made and with a much fatter cylinder: a ~17 mm
    # bore driven most of the length of the nacelle. The nacelle is faired into
    # the body, so the crown DROPS as the bore runs forward, and past a certain
    # station the bore's ceiling stands above the local skin - it stops being a
    # bore and becomes an open trough. Measured on a 500x610x350 swept wing
    # with a centre fin, that opened a ~12 mm slot along the spine just behind
    # the fin: the reported "random hole and gap behind the tail rudder".
    #
    # The wires no longer need it either - `cad.conduits` runs them from the
    # mount face into the equipment bay through a properly routed, skin-checked
    # channel. So the bore is now only as deep as the boss it clears, and it is
    # additionally clipped to stay inside the local section.
    shaft_depth = _clamp(2.0 * t_plate, 8.0, 15.0)
    r_shaft = 0.5 * d_shaft
    x_tip = x_face + sgn * shaft_depth
    for f in np.linspace(0.0, 1.0, 9):          # walk it back until it is buried
        x_end = x_face + sgn * shaft_depth * (1.0 - f)
        if abs(x_end - x_face) < 2.0:
            shaft_depth = 0.0
            break
        xc = wing.xc_at(0.0, x_end) if wing is not None else 0.5
        if not (0.0 <= xc <= 1.0):
            continue
        z_hi = wing.crown_z(0.0, xc) if wing is not None else z_c + r_shaft + 99
        z_lo = wing.keel_z(0.0, xc) if wing is not None else z_c - r_shaft - 99
        if z_c + r_shaft <= z_hi - 0.8 and z_c - r_shaft >= z_lo + 0.8:
            shaft_depth = shaft_depth * (1.0 - f)
            break
    if shaft_depth >= 2.0:
        cutters.append(Solid.makeCylinder(
            r_shaft, lead + shaft_depth,
            Vector(x_face - sgn * lead, y_c, z_c), axis))
    return boss, cutters, x_root


def _cg_marker(x_cg: float, z_bottom: float, scale: float = 1.0) -> Solid:
    """Visible balance-point marker: a small keel cone at the CG station. It
    reaches well up into the skin so it always fuses, and protrudes only a
    couple of millimetres so it cannot blow the height budget."""
    r = _clamp(5.0 * scale, 3.0, 8.0)
    out = _clamp(2.5 * scale, 1.5, 4.0)
    return Solid.makeCone(r, 0.8, out + 9.0, Vector(x_cg, 0, z_bottom - out),
                          Vector(0, 0, 1))


# ---------------------------------------------------------------------------
# Hinge grooves (control-surface split lines)
# ---------------------------------------------------------------------------

def _hinge_groove(airfoil: str, s1: Station, s2: Station, tdir: Vector,
                  xc: float = 0.72, width: float = 1.1,
                  t1: float = 1.0, t2: float = 1.0) -> Solid:
    """Thin spanwise groove cutter along the hinge line at chord fraction xc
    on the TOP surface: ~38% of the local thickness deep, `width` mm wide,
    lofted between the two end stations so it follows taper/twist/dihedral."""
    coords = _airfoil_pts(airfoil)
    tu = _foil_surf_t(coords, xc, upper=True)
    tl = _foil_surf_t(coords, xc, upper=False)
    rects = []
    for (c, tw, le), ts in ((s1, t1), (s2, t2)):
        p_up = _foil_point(c, tw, le, tdir, xc, tu, ts)
        depth = 0.38 * (tu - tl) * c * ts
        lo = p_up - tdir * depth
        hi = p_up + tdir * 3.0
        pts = [lo + Vector(-width / 2, 0, 0), lo + Vector(width / 2, 0, 0),
               hi + Vector(width / 2, 0, 0), hi + Vector(-width / 2, 0, 0)]
        rects.append(Wire.makePolygon(pts, close=True))
    return Solid.makeLoft(rects, True)


def _elevon_grooves(wing: _BlendedWing, airfoil: str, inner: float,
                    outer: float, chord_frac: float) -> list[Solid]:
    """Hinge grooves along the elevon trailing edges, one cutter per spanwise
    sub-segment per side so the groove follows the planform where the chord is
    still changing (inboard of the panel it is not a straight line)."""
    xc = _clamp(1.0 - chord_frac, 0.45, 0.90)
    # never start a hinge line inside the centre body: the groove is cut to a
    # fraction of the LOCAL thickness, and inboard of `fb` that is body depth,
    # not wing depth - it would gouge the equipment bay open
    inner = _clamp(inner, wing.fb, 0.90)
    outer = _clamp(outer, inner + 0.05, _TIP_START - 0.005)
    edges = np.linspace(inner, outer, 4)
    out: list[Solid] = []
    for sgn in (1.0, -1.0):
        for a, b in zip(edges[:-1], edges[1:]):
            sa, sb = wing.section(sgn * float(a)), wing.section(sgn * float(b))
            width = _clamp(0.006 * sa.chord, 1.0, 2.2)
            try:
                out.append(_hinge_groove(
                    airfoil, sa.as_station(), sb.as_station(), Vector(0, 0, 1),
                    xc=xc, width=width, t1=sa.t_scale, t2=sb.t_scale))
            except Exception:
                continue
    return out




# ---------------------------------------------------------------------------
# Separated elevons on printed pin hinges
# ---------------------------------------------------------------------------

# Every print clearance now lives in `backend.cad.hinges.HINGE_DEFAULTS`,
# each one carrying the published source it came from. This module only makes
# the split; the hinge module decides how the two halves clear each other.


def _rot_to(u: Vector) -> tuple[Vector, float]:
    """Rotation (axis, degrees) taking +Z onto the unit vector `u`."""
    z = Vector(0, 0, 1)
    d = max(-1.0, min(1.0, z.dot(u)))
    cr = z.cross(u)
    if cr.Length < 1e-9:
        return Vector(1, 0, 0), (0.0 if d > 0 else 180.0)
    return cr, math.degrees(math.acos(d))


def _slab(origin: Vector, u: Vector, t0: float, t1: float,
          reach: float) -> Solid:
    """Half-open box between distances t0..t1 along `u` from `origin`,
    unbounded (to `reach`) in the other two directions."""
    box = Solid.makeBox(2.0 * reach, 2.0 * reach, max(t1 - t0, 0.01),
                        Vector(-reach, -reach, 0.0))
    axis, ang = _rot_to(u)
    box = box.rotate(Vector(0, 0, 0), axis, ang)
    return box.translate(origin + u.multiply(t0))


def _elevon_hinge_line(wing: "_BlendedWing", sgn: float, inner: float,
                       outer: float, xc: float):
    """(inboard point, outboard point, inboard thickness, outboard thickness).
    line. Real elevon hinge lines are straight even when the planform is not:
    that is what lets one wire pin the whole surface."""
    pts, thick = [], []
    for f in (inner, outer):
        sec = wing.section(sgn * f)
        x = sec.le.x + xc * sec.chord
        xcl = wing.xc_at(sgn * f, x)
        crown, keel = wing.crown_z(sgn * f, xcl), wing.keel_z(sgn * f, xcl)
        pts.append(Vector(x, sec.le.y, 0.5 * (crown + keel)))
        thick.append(crown - keel)
    return pts[0], pts[1], thick[0], thick[1]


def _separate_elevons(airframe: Solid, wing: "_BlendedWing", inner: float,
                      outer: float, chord_frac: float):
    """Cut the elevons free of the wing and hang them on print-in-place hinges.

    Returns (airframe_without_elevons, {name: elevon_solid}, {name: info}).

    The split itself happens here - intersect the airframe with the half-space
    aft of the hinge plane to get the surface, cut the same half-space away to
    get the wing - and it is made on the hinge plane EXACTLY, with no gap. All
    of the clearance is created afterwards by `backend.cad.hinges`, which puts
    the standard RC double bevel on the surface nose (apex on the axis, set
    back by the running gap) and the matching cove in the wing trailing edge.
    That pairing, not a parallel gap, is what lets a centre-hinged surface
    actually deflect; a square nose binds on the trailing edge at about 1 deg.

    The hinges themselves are two per surface at 20% / 80% of the hinge line,
    each three knuckles with the pin PRINTED IN PLACE inside the surface's
    barrel - nothing to thread after the print. See `hinges.HINGE_DEFAULTS`
    for every clearance and the source it came from.
    """
    from . import hinges as _h

    xc = _clamp(1.0 - chord_frac, 0.45, 0.90)
    inner = _clamp(inner, max(wing.fb, 0.10), 0.85)
    outer = _clamp(outer, inner + 0.10, _TIP_START - 0.01)
    reach = 4.0 * wing.half
    out: dict[str, Solid] = {}
    reports: dict[str, dict] = {}

    for sgn, name in ((1.0, "elevon_right"), (-1.0, "elevon_left")):
        try:
            p_in, p_out, t_in, t_out = _elevon_hinge_line(
                wing, sgn, inner, outer, xc)
        except Exception:
            continue
        d = p_out - p_in
        span = d.Length
        if span < 25.0:
            continue
        u = d.multiply(1.0 / span)

        # aft direction in plan, perpendicular to the hinge line
        aft = Vector(-u.y, u.x, 0.0)
        if aft.Length < 1e-9:
            aft = Vector(1.0, 0.0, 0.0)
        aft = aft.multiply(1.0 / aft.Length)
        if aft.x < 0:
            aft = aft.multiply(-1.0)

        try:
            span_box = _slab(p_in, u, 0.0, span, reach)
            pocket = _slab(p_in, aft, 0.0, reach, reach).intersect(span_box)
            elevon = _heal(airframe.intersect(pocket))
            keep = [s for s in elevon.Solids() if s.Volume() > 50.0]
            if not keep:
                continue
            elevon = max(keep, key=lambda s: s.Volume())
            trimmed = _heal(airframe.cut(pocket))
            if not trimmed.isValid() or len(trimmed.Solids()) != 1:
                continue
        except Exception:
            continue

        try:
            trimmed, elevon, info = _h.print_in_place_hinges(
                trimmed, elevon, p_in, p_out, t_in, t_out, n_hinges=2)
        except Exception as exc:                     # never lose the split
            info = {"mode": "none", "warnings": [f"hinge module failed: {exc}"]}

        keep = [s for s in elevon.Solids() if s.Volume() > 50.0]
        if not keep or not trimmed.isValid() or len(trimmed.Solids()) != 1:
            continue
        out[name] = max(keep, key=lambda s: s.Volume())
        airframe = trimmed
        reports[name] = info
    return airframe, out, reports


# ---------------------------------------------------------------------------
# Boolean plumbing
# ---------------------------------------------------------------------------

def _heal(solid: Solid) -> Solid:
    """Repair pathological boolean-trimmed faces. Spline-on-spline fuses can
    leave faces whose trim curves the mesher silently refuses to tessellate -
    the exported STL then has a gaping hole even though the BRep reports
    valid. ShapeFix rebuilds those trims. Keep the healed shape only if it is
    still one valid solid."""
    try:
        from cadquery import Shape
        from OCP.ShapeFix import ShapeFix_Shape

        sf = ShapeFix_Shape(solid.wrapped)
        sf.Perform()
        healed = Shape.cast(sf.Shape())
        healed_solids = healed.Solids()
        if len(healed_solids) == 1 and healed.isValid():
            return healed_solids[0]
    except Exception:
        pass
    return solid


def _fuse_all(parts: list[Solid]) -> Solid:
    solid = parts[0]
    for p in parts[1:]:
        try:
            solid = solid.fuse(p)
        except Exception:
            continue  # drop a decorative part rather than fail the build
    # NOTE: no .clean() here - merging the boolean split faces produces spline
    # faces that OCC's mesher fails to tessellate (missing skin in the
    # exported STL). The extra split edges are cosmetic only.
    return _heal(solid)


# How far a verified fuse's result may fall short of the addition's bounding
# box before the feature counts as missing. Booleans legitimately trim a
# fraction of a millimetre off a root that pierces the host; losing a whole
# millimetre of a fin's extent does not happen unless the part is gone.
_FUSE_BBOX_TOL_MM = 1.0


def _point_in_solid(solid: Solid, p: Vector, tol: float = 1e-7) -> bool:
    """True when `p` is inside or on `solid` (OCC point classification)."""
    try:
        from OCP.BRepClass3d import BRepClass3d_SolidClassifier
        from OCP.gp import gp_Pnt
        from OCP.TopAbs import TopAbs_IN, TopAbs_ON

        cl = BRepClass3d_SolidClassifier(solid.wrapped,
                                         gp_Pnt(p.x, p.y, p.z), tol)
        return cl.State() in (TopAbs_IN, TopAbs_ON)
    except Exception:
        return False


def _witness_inside(solid: Solid) -> Vector | None:
    """A point provably INSIDE `solid`, for use as a classification witness.

    The centre of mass is right for a wing loft or a fin, but falls outside a
    curved or shelled body, so fall back to a coarse grid over the bounding
    box. None only if nothing lands inside (a degenerate shape).
    """
    try:
        c = solid.Center()
        if _point_in_solid(solid, c):
            return c
    except Exception:
        pass
    try:
        bb = solid.BoundingBox()
    except Exception:
        return None
    for fx in (0.5, 0.35, 0.65, 0.2, 0.8):
        for fy in (0.5, 0.35, 0.65):
            for fz in (0.5, 0.35, 0.65, 0.25):
                p = Vector(bb.xmin + fx * (bb.xmax - bb.xmin),
                           bb.ymin + fy * (bb.ymax - bb.ymin),
                           bb.zmin + fz * (bb.zmax - bb.zmin))
                if _point_in_solid(solid, p):
                    return p
    return None


def _fuse_verified(base: Solid, add: Solid) -> Solid | None:
    """Fuse and PROVE the addition survived, or return None.

    `isValid()` and "exactly one solid" are not enough. On the v3 biplane the
    fin fused with both flags true while the result simply did not contain
    the fin: the tail then had no vertical surface at all, the rudder split
    took a chip out of the tail cone instead, and its horn fell off that
    chip - three "successful" steps downstream of a boolean that had quietly
    thrown the part away. Same doctrine as the tessellation gate and the
    horn-bore classification: an operation whose absence matters gets its own
    existence check.

    The check is GEOMETRIC, never volumetric: `Volume()` on these spline-
    heavy solids is computed to a loose default tolerance and disagrees with
    itself by tens of thousands of mm3 on shapes that classify perfectly, so
    a volume test would fail good fuses and pass bad ones. Instead the result
    must contain the addition's bounding box and a witness point taken inside
    each operand.

    The retry ladder exists because this boolean is tolerance-sensitive: the
    SAME operands that fail in-build succeed after a BRep round-trip (which
    rewrites tolerances), so healing the operands - the in-memory equivalent
    - is tried before giving up, then a fuzzy-valued boolean.
    """
    w_add = _witness_inside(add)
    w_base = _witness_inside(base)
    try:
        bb_a = add.BoundingBox()
    except Exception:
        return None

    def ok(res: Solid | None) -> bool:
        if res is None:
            return False
        try:
            if not (res.isValid() and len(res.Solids()) == 1):
                return False
            bb = res.BoundingBox()
            # the addition's extents must fit inside the result's, or a piece
            # of it is missing (the biplane fin: add reached z=161.8, the
            # "successful" fuse stopped at z=51.4)
            if (bb.xmin > bb_a.xmin + _FUSE_BBOX_TOL_MM
                    or bb.xmax < bb_a.xmax - _FUSE_BBOX_TOL_MM
                    or bb.ymin > bb_a.ymin + _FUSE_BBOX_TOL_MM
                    or bb.ymax < bb_a.ymax - _FUSE_BBOX_TOL_MM
                    or bb.zmin > bb_a.zmin + _FUSE_BBOX_TOL_MM
                    or bb.zmax < bb_a.zmax - _FUSE_BBOX_TOL_MM):
                return False
            for w in (w_add, w_base):
                if w is not None and not _point_in_solid(res, w):
                    return False
        except Exception:
            return False
        return True

    def attempt(fn):
        try:
            res = fn()
        except Exception:
            return None
        return res if ok(res) else None

    # 1. what the builders have always done - the only rung that runs when
    #    the boolean behaves, so the happy path costs one bbox and two point
    #    classifications on top of the fuse itself
    got = attempt(lambda: _heal(base.fuse(add)))
    if got is not None:
        return got
    # 2. drop the cached tessellation from both operands (cheap, in memory)
    got = attempt(lambda: _heal(_cleaned(base).fuse(_cleaned(add))))
    if got is not None:
        return got
    # 3. rebuild both operands through BRep. Measured on the biplane fin:
    #    the in-memory fuse drops the fin, the SAME operands round-tripped
    #    fuse correctly, and round-tripping the base alone does not help -
    #    it is the freshly lofted addition that carries the bad state.
    got = attempt(lambda: _heal(_round_tripped(base).fuse(_round_tripped(add))))
    if got is not None:
        return got
    # 4. fuzzy-valued boolean, coarsening
    for fuzz in (1e-3, 1e-2):
        got = attempt(lambda f=fuzz: _heal(_fuse_fuzzy(base, add, f)))
        if got is not None:
            return got
    return None


def _cleaned(solid: Solid) -> Solid:
    """A copy of `solid` with its cached triangulation and polygons dropped.

    A freshly lofted surface carries the mesh left behind by whatever last
    tessellated it, and OCC's boolean consults that cache. Dropping it is the
    cheap, in-memory half of what a BRep round-trip does.
    """
    from cadquery import Shape
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Copy
    from OCP.BRepTools import BRepTools

    shape = BRepBuilderAPI_Copy(solid.wrapped).Shape()
    BRepTools.Clean_s(shape)
    return Shape.cast(shape)


def _round_tripped(solid: Solid) -> Solid:
    """`solid` written to BRep and read back, which rebuilds it from its own
    serialized form and normalizes the per-face tolerances that come out of a
    loft. Slower than `_cleaned` (it goes through a temp file), and the only
    thing measured to rescue the biplane's fin fuse - see `_fuse_verified`.
    """
    import tempfile
    from pathlib import Path
    from cadquery import Shape
    from OCP.BRep import BRep_Builder
    from OCP.BRepTools import BRepTools
    from OCP.TopoDS import TopoDS_Shape

    with tempfile.TemporaryDirectory(prefix="aeroforge_fuse_") as d:
        path = str(Path(d) / "operand.brep")
        BRepTools.Write_s(solid.wrapped, path)
        shape = TopoDS_Shape()
        BRepTools.Read_s(shape, path, BRep_Builder())
    out = Shape.cast(shape)
    solids = out.Solids()
    return solids[0] if len(solids) == 1 else out


def _fuse_fuzzy(base: Solid, add: Solid, fuzz: float) -> Solid | None:
    """`base ∪ add` with an explicit boolean fuzzy value (mm)."""
    from cadquery import Shape
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse
    from OCP.TopTools import TopTools_ListOfShape

    args, tools = TopTools_ListOfShape(), TopTools_ListOfShape()
    args.Append(base.wrapped)
    tools.Append(add.wrapped)
    op = BRepAlgoAPI_Fuse()
    op.SetArguments(args)
    op.SetTools(tools)
    op.SetFuzzyValue(float(fuzz))
    op.SetRunParallel(False)
    op.Build()
    if not op.IsDone():
        return None
    shape = Shape.cast(op.Shape())
    solids = shape.Solids()
    return solids[0] if len(solids) == 1 else None


def fuse_feature(base: Solid, add: Solid, what: str,
                 warnings: list[str]) -> tuple[Solid, bool]:
    """Fuse a named feature into the airframe, verified (`_fuse_verified`).

    On failure keep `base`, warn by name, and say so - callers that COUNT
    fused features (tip fins, struts) need the flag, not an inference from
    the warning list. Shared by every type's builder so the gate cannot be
    fixed in one module and missed in another.
    """
    got = _fuse_verified(base, add)
    if got is not None:
        return got, True
    warnings.append(f"{what} did not fuse into the airframe - it is MISSING "
                    f"from this model, not merely unhealed")
    return base, False


def _apply_grooves(solid: Solid, cutters: list[Solid]) -> Solid:
    """Cut hinge grooves out of one part, keeping the part if a cut fails or
    would break it up (grooves are cosmetic split lines, never structural)."""
    for cutter in cutters:
        try:
            cut = _heal(solid.cut(cutter))
        except Exception:
            continue
        if len(cut.Solids()) == 1 and cut.isValid():
            solid = cut.Solids()[0]
    return solid


# ---------------------------------------------------------------------------
# Whole-aircraft assembly
# ---------------------------------------------------------------------------

# how deep a centre fin's trimmed root sits inside the cavity roof (the
# roof is >= 3 mm aft of the hatch); see the fin fuse in build_design_parts
_FIN_ROOT_BURY_MM = 1.2


def trim_root_to_cavity(fin_solid: Solid, cavity: Solid, warnings: list[str],
                        *, what: str = "fin",
                        bury_mm: float = _FIN_ROOT_BURY_MM,
                        keep_min_frac: float = 0.15) -> Solid:
    """Trim a vertical surface's buried root to the cavity roof.

    The root is cut with the cavity lifted by `bury_mm` so the blade sits
    that deep inside the >= 3 mm wall along its whole chord - bonded
    everywhere, never standing in the compartment. Accepted only if the
    result is valid, one solid and keeps more than `keep_min_frac` of the
    fin's volume (the buried blade of a deep body can be MORE than half of
    it - the 675 mm fpv wing: root 30 mm down, fin 40 mm up - so the floor
    is 15 %, enough to catch a cut that ate the fin, not one that did its
    job). A rejected trim is named in `warnings`: it means a wall down the
    compartment. Shared by every type's builder (work plan task 1).
    """
    try:
        # two cuts: the cavity lifted by the bury depth takes the roof band,
        # the cavity itself takes what the lift leaves where the FLOOR rises
        # (the tapering rear extension: measured on the 675 mm swept fpv
        # wing, a 1 mm foot sliver at x 219-228 stood on the cavity floor
        # after the lifted cut alone - tools_probe_fin_intrusion.py)
        trimmed = _heal(fin_solid.cut(cavity.translate(Vector(0.0, 0.0,
                                                              bury_mm))))
        trimmed = _heal(trimmed.cut(cavity))
        if (trimmed.isValid() and len(trimmed.Solids()) == 1
                and trimmed.Volume() > keep_min_frac * fin_solid.Volume()):
            return trimmed
        warnings.append(f"{what} root NOT trimmed to the cavity roof - "
                        "it stands in the compartment")
    except Exception as exc:
        warnings.append(f"{what} root trim failed: {type(exc).__name__}")
    return fin_solid


def bay_request(g: dict, wing: "_BlendedWing", body: dict, wall: float,
                boss: Solid | None, x_nacelle_root: float,
                x_fin_limit: float | None) -> dict:
    """Everything `hatch.build_bay` needs for THIS design, in one place.

    Shared by `build_design_parts` and the configuration sweep probe
    (`tools_probe_cavity.py`), so a sweep audits exactly the request the
    build makes. Returns the keyword arguments: bay_start/length/half_width
    (mm), wall, x_max (hatch cap), cavity_extend_to, cavity_guard and
    cavity_roof_cap.

    * `x_max` caps the HATCH ahead of a centre fin's buried root.
    * The CAVITY is not capped by the fin: it continues under the root
      with its roof held below it (`cavity_roof_cap`, the same 0.35 t
      root depth the fin builder buries to). Capping the cavity there is
      what sent every centre-fin design back to the box galleries.
    * `cavity_extend_to` reaches 200 mm past the nominal bay, never into
      a pusher nacelle / motor-boss root.
    * `cavity_guard` keeps the extension out of the elevon corridor and
      the trailing-edge margin.
    """
    x_max = None
    roof_cap: tuple | None = None
    if str((g.get("vstab") or {}).get("type", "")) == "center_fin":
        try:
            _h, cr_f, _ct = _fin_dims(g.get("vstab") or {},
                                      float(g.get("area_m2", 0.0)) * MM * MM)
            sec0 = wing.section(0.0)
            cr_f = min(cr_f, 0.55 * sec0.chord)
            x_te_fin = sec0.le.x + sec0.chord - max(2.0, 0.03 * cr_f)
            if x_fin_limit is not None:
                x_te_fin = min(x_te_fin, x_fin_limit)
            x_max = x_te_fin - cr_f - 5.0
            x_le_f = x_te_fin - cr_f
            xc_lo = wing.xc_at(0.0, x_le_f)
            z_root = (min(wing.crown_z(0.0, xc_lo),
                          wing.crown_z(0.0, min(xc_lo + 0.25, 1.0)))
                      - 0.35 * wing.tc * sec0.chord * sec0.t_scale)
            # No roof cap: the fin's ROOT is trimmed to the cavity roof
            # when it is fused (build_design_parts), so the cavity keeps its
            # full section under the fin. (A centre-strip dip was tried and
            # filled the middle of a deep body's cavity - the fin root there
            # reaches 30 mm down - leaving two side channels.)
            del x_le_f, z_root
            roof_cap = None
        except Exception:
            x_max, roof_cap = None, None
    el_g = g.get("elevons") or {}
    xc_g = _clamp(1.0 - float(el_g.get("chord_frac", 0.25)), 0.45, 0.90)
    guard = {
        "hinge_xc": xc_g,
        "span_lo_mm": float(el_g.get("inner_frac", 0.35)) * wing.half,
        "span_hi_mm": float(el_g.get("outer_frac", 0.95)) * wing.half,
        "xc_max": 0.88,
    }
    ext_to = (float(body.get("bay_start_m", 0.0))
              + float(body.get("bay_length_m", 0.0))) * MM + 200.0
    mm_spec = g.get("motor_mount") or {}
    if (boss is not None and x_nacelle_root > 0.0
            and str(mm_spec.get("type", "")) == "pusher"):
        # a PUSHER nacelle root sits on the tail; never carve into it. On a
        # tractor the station is the nose boss - ahead of the bay - and
        # capping on it killed the plank's whole extension at 45 mm.
        ext_to = min(ext_to, float(x_nacelle_root) - 6.0)
    return {
        "bay_start": float(body.get("bay_start_m", 0.0)) * MM,
        "bay_length": float(body.get("bay_length_m", 0.0)) * MM,
        "bay_half_width": 0.5 * float(body.get("bay_width_m", 0.0)) * MM,
        "wall": wall,
        "x_max": x_max,
        "cavity_extend_to": ext_to,
        "cavity_guard": guard,
        "cavity_roof_cap": roof_cap,
    }


def _wing_from_design(g: dict) -> _BlendedWing:
    """Read SPEC §4 `geometry` into the section generator (§5)."""
    span = float(g["span_m"]) * MM
    half = max(span / 2.0, 1.0)
    body = g.get("body") or {}
    c_root = float(g["root_chord_m"]) * MM
    c_tip = float(g.get("tip_chord_m")
                  or c_root * float(g.get("taper", 0.5))) * MM
    hw = float(body.get("half_width_m", 0.14 * span / MM)) * MM
    wing = _BlendedWing(
        coords=_airfoil_pts(g.get("airfoil", "RFX-9 reflexed")),
        half=half,
        c_root=c_root,
        c_tip=max(c_tip, 0.06 * c_root),
        sweep_deg=float(g.get("sweep_le_deg", 0.0)),
        dihedral_deg=float(g.get("dihedral_deg", 0.0)),
        incidence=float(g.get("root_incidence_deg", 0.0)),
        washout=float(g.get("washout_deg", 0.0)),
        fb=_clamp(hw / half, 0.06, 0.60),
        depth_scale=_clamp(float(body.get("depth_scale", 1.0)), 1.0, 4.0),
        chord_scale=_clamp(float(body.get("chord_scale", 1.0)), 1.0, 1.8),
        nose_round=float(body.get("nose_round", 0.6)),
        crown_frac=_clamp(float(body.get("crown_frac", 0.62)), 0.5, 0.85),
    )
    wing.solve_nose_guard()
    return wing


def _fin_dims(vs: dict, s_wing_mm2: float) -> tuple[float, float, float]:
    """(height, root chord, tip chord) per surface, mm. Uses what the physics
    sized; falls back to area + a sensible fin aspect ratio."""
    n = max(int(vs.get("count", 2) or 2), 1)
    h = float(vs.get("height_m", 0.0) or 0.0) * MM
    cr = float(vs.get("root_chord_m", 0.0) or 0.0) * MM
    ct = float(vs.get("tip_chord_m", 0.0) or 0.0) * MM
    if h <= 0.0 or cr <= 0.0:
        area = float(vs.get("area_total_m2", 0.0) or 0.0) * MM * MM
        area = max(area, 0.02 * s_wing_mm2)
        h = math.sqrt(1.4 * area / n)
        cr = 1.30 * (area / n) / h
        ct = 0.62 * cr
    if ct <= 0.0:
        ct = 0.62 * cr
    return h, cr, ct


def _vertical_surfaces(g: dict, wing: "_BlendedWing", fin_af: str,
                       span: float, x_fin_limit: float | None,
                       x_aft: float
                       ) -> tuple[dict[str, Solid], dict[str, float]]:
    """The fixed vertical surfaces of the tailless types, as the loose
    solids they are before the fuse: `{name: solid}` in fusing order plus
    `{name: span_fraction}` of where each stands. Split out of
    `_build_parts` verbatim (2026-08-27) so the fin-intrusion probe can
    build EXACTLY the fins the airframe gets and classify them against
    the bay cavity; the build calls it at the same point it always did.
    """
    parts: dict[str, Solid] = {}
    # ---- vertical surfaces (SPEC §5 "Fin construction") -------------------
    _progress("fins")
    vs = g.get("vstab") or {}
    vs_type = str(vs.get("type", "none"))
    s_wing = float(g.get("area_m2", 0.0)) * MM * MM or (wing.c_root * span)
    fin_placed: dict[str, float] = {}

    if vs_type in ("winglets", "twin_fin", "center_fin"):
        h_fin, cr_fin, ct_fin = _fin_dims(vs, s_wing)
        sweep_fin = float(vs.get("sweep_le_deg", 26.0) or 26.0)

    if vs_type == "winglets":
        # Canted tip fins. The root sits as far outboard as it can while the
        # CANTED TIP still lands inside the recorded span - on a real wing the
        # span IS measured over the winglets, so this is the honest placement
        # rather than a part hanging outside the user's box.
        cant = _clamp(float(vs.get("cant_deg", 12.0) or 0.0), 0.0, 35.0)
        ca, sa = math.cos(math.radians(cant)), math.sin(math.radians(cant))
        f_root, span_f = 0.965, h_fin
        for _ in range(4):   # the fin's own span depends on where it is rooted
            z_lo = wing.keel_z(f_root, 0.55) + 1.0
            z_hi = wing.crown_z(f_root, 0.55) + h_fin
            span_f = max((z_hi - z_lo) / max(ca, 0.2), 0.5 * h_fin)
            f_new = _clamp((wing.half - span_f * sa - 0.8) / wing.half,
                           0.72, 0.965)
            if abs(f_new - f_root) < 2e-4:
                break
            f_root = f_new
        for sgn, name in ((1.0, "winglet_right"), (-1.0, "winglet_left")):
            sec = wing.section(sgn * f_root)
            cr = min(cr_fin, 0.95 * sec.chord)
            z_lo = wing.keel_z(sgn * f_root, 0.55) + 1.0
            # trailing edge flush with the wing tip trailing edge
            x_le = sec.le.x + sec.chord - cr
            x_le, sw = _clamp_aft(x_le, cr, span_f, sweep_fin, ct_fin / cr,
                                  x_aft)
            solid, _ = _rounded_surface(
                airfoil=fin_af, span_mm=span_f, c_root_mm=cr,
                c_tip_mm=min(ct_fin, 0.85 * cr),
                le_root=Vector(x_le, sec.le.y, z_lo),
                sweep_le_deg=sw, dihedral_deg=0.0,
                twist_root_deg=0.0, twist_tip_deg=0.0,
                span_dir=Vector(0, sgn * sa, ca),
                tdir=Vector(0, ca, -sgn * sa))
            parts[name] = solid
            fin_placed[name] = sgn * f_root
    elif vs_type == "twin_fin":
        # X5-style inboard fins: vertical, standing on the wing at y_frac of
        # the semi-span. The root reaches down to just inside the lower skin,
        # so the fin passes through the full section and always fuses.
        yf = _clamp(float(vs.get("y_frac", 0.58) or 0.58), 0.20, 0.88)
        for sgn, name in ((1.0, "fin_right"), (-1.0, "fin_left")):
            sec = wing.section(sgn * yf)
            cr = min(cr_fin, 0.92 * sec.chord)
            # centre the fin on the local chord, biased aft for yaw damping
            x_le = sec.le.x + 0.98 * sec.chord - cr
            xc_mid = wing.xc_at(sgn * yf, x_le + 0.5 * cr)
            z_lo = wing.keel_z(sgn * yf, xc_mid) + 1.0
            z_hi = wing.crown_z(sgn * yf, xc_mid) + h_fin
            span_f = max(z_hi - z_lo, 0.5 * h_fin)
            x_le, sw = _clamp_aft(x_le, cr, span_f, sweep_fin, ct_fin / cr,
                                  x_aft)
            solid, _ = _rounded_surface(
                airfoil=fin_af, span_mm=span_f, c_root_mm=cr,
                c_tip_mm=min(ct_fin, 0.85 * cr),
                le_root=Vector(x_le, sec.le.y, z_lo),
                sweep_le_deg=sw, dihedral_deg=0.0,
                twist_root_deg=0.0, twist_tip_deg=0.0,
                span_dir=Vector(0, 0, 1), tdir=Vector(0, 1, 0))
            parts[name] = solid
            fin_placed[name] = sgn * yf
    elif vs_type == "center_fin":
        # Centre fin on the body spine, trailing edge just AHEAD of the body
        # TE. The inset is not cosmetic: putting the fin's TE exactly on the
        # wing's leaves two near-coincident trailing-edge faces a couple of
        # tenths apart, and OCC then computes an empty intersection - the fin
        # comes out as a second, disconnected solid even though the two shapes
        # visibly overlap. Real wings inset the fin anyway; the trailing edge
        # belongs to the elevon.
        sec = wing.section(0.0)
        cr = min(cr_fin, 0.55 * sec.chord)
        inset = max(2.0, 0.03 * cr)
        x_te = sec.le.x + sec.chord - inset
        if x_fin_limit is not None:
            # slide the whole fin forward of the motor nacelle / prop disc
            x_te = min(x_te, x_fin_limit)
        x_le = x_te - cr
        xc_lo = wing.xc_at(0.0, x_le)
        z_lo = min(wing.crown_z(0.0, xc_lo),
                   wing.crown_z(0.0, min(xc_lo + 0.25, 1.0))) \
            - 0.35 * wing.tc * sec.chord * sec.t_scale
        z_hi = wing.crown_z(0.0, min(xc_lo + 0.15, 1.0)) + h_fin
        span_f = max(z_hi - z_lo, 0.5 * h_fin)
        x_le, sw = _clamp_aft(x_le, cr, span_f, sweep_fin, ct_fin / cr, x_aft)
        solid, _ = _rounded_surface(
            airfoil=fin_af, span_mm=span_f, c_root_mm=cr,
            c_tip_mm=min(ct_fin, 0.80 * cr),
            le_root=Vector(x_le, 0.0, z_lo),
            sweep_le_deg=sw, dihedral_deg=0.0,
            twist_root_deg=0.0, twist_tip_deg=0.0,
            span_dir=Vector(0, 0, 1), tdir=Vector(0, 1, 0))
        parts["fin"] = solid
        fin_placed["fin"] = 0.0
    # "none": bell-spanload designs carry NO vertical surface at all - the
    # tip-region induced thrust gives proverse yaw, which is the whole point
    # of the configuration. Never add one here.
    return parts, fin_placed


def _build_parts(design: dict, separate_parts: bool = True,
                 _probe: dict | None = None) -> tuple[
        dict[str, Solid], list[tuple[str, Solid]], dict[str, Any]]:
    """Core builder: every airframe part as its own solid, in the world
    coordinates it occupies on the assembled aircraft, plus the hinge-groove
    cutters (tagged with the part they belong to) and the metadata dict.

    Parts are inserted in fusing order (airframe, fins, lid, marker) -
    :func:`build_design_solid` unions them in exactly that order."""
    g = design["geometry"]
    wing = _wing_from_design(g)
    body = g.get("body") or {}
    fin_af = g.get("fin_airfoil", "NACA 0008")
    span = 2.0 * wing.half
    ext = wing.extents()

    parts: dict[str, Solid] = {}
    grooves: list[tuple[str, Solid]] = []
    hinge_report: dict[str, dict] = {}
    servo_report: dict = {}

    # ---- the aircraft: ONE continuous loft, left tip to right tip ---------
    _progress("loft")
    airframe = _blended_airframe(wing)
    wall = max(float(g.get("wall_mm", 1.2)), 0.6)

    # ---- structural motor bulkhead ----------------------------------------
    # Fused BEFORE the bay is cut and BEFORE the holes are drilled, so the
    # boss, the skin and the bay's bulkhead end up as one piece of material
    # and the screw holes pass cleanly through all of it.
    mount_cutters: list[Solid] = []
    mount_spec = g.get("motor_mount") or {}
    boss, mount_cutters, x_nacelle_root = _motor_mount(wing, mount_spec, wall)
    if boss is not None:
        ok = False
        try:
            # Validate before AND after: a nacelle loft that self-intersects
            # fuses into a compound rather than raising, and the aircraft then
            # quietly exports as several disconnected solids.
            if boss.isValid() and len(boss.Solids()) == 1 and boss.Volume() > 0:
                merged = _heal(airframe.fuse(boss))
                if merged.isValid() and len(merged.Solids()) == 1:
                    airframe = merged
                    ok = True
        except Exception:
            ok = False
        if not ok:
            mount_cutters, x_nacelle_root = [], 0.0
            boss = None
    # Anything else living on the centreline has to clear the nacelle. A centre
    # fin whose trailing edge overlaps it is not just a modelling collision -
    # a fin sitting in the prop disc is a real design error.
    pusher_nacelle = (boss is not None
                      and str(mount_spec.get("type", "")) == "pusher")
    x_fin_limit = (x_nacelle_root - 4.0) if pusher_nacelle else None

    # ---- hollow equipment bay + hatch --------------------------------------
    _progress("bay")
    # The bay is hollowed either way. What differs is the LID: the parts
    # export gets it as its own body (print it separately, it is a real hatch),
    # while the one-piece STL keeps it attached and only scribes its outline -
    # a two-piece STL would not be one watertight solid.
    # Retry ladder on the bay width. A cavity that comes too close to the skin
    # produces a shape OCC reports as invalid or splits into several solids -
    # a narrower bay is always better than a broken airframe.
    lid: Solid | None = None
    bay_built = False
    # A centre fin is buried deep in the body to carry its load, so if the bay
    # runs back under it the fin's root stands inside the compartment - which
    # is exactly as useless as it sounds. Work out where the fin starts BEFORE
    # cutting the bay and stop the bay short of it.
    bay_req = bay_request(g, wing, body, wall, boss, x_nacelle_root,
                          x_fin_limit)
    bay_x_max = bay_req.get("x_max")

    # `backend.cad.hatch` owns the whole compartment: it cuts the void, opens
    # the aperture through the ceiling, leaves the seat lip standing and lifts
    # the canopy out of the airframe's own skin so it sits flush.
    #
    # It runs its own ladder and gates every rung on BOTH mesh tests, so there
    # is no retry ladder around it here. The two tests catch different things
    # and it needs both: a per-face test asks "did OCC triangulate every face",
    # which is the right question for a 12 cm^3 canopy where one small pocket
    # face is 2% of the area and an area ratio would false-fail it. But OCC can
    # also mesh every face and still lose most of a BIG one, and that is what
    # puts a hole in the exported skin - it once cost 73 mm of nose, and the
    # area ratio is the only thing that sees it. Wrapping a second ladder round
    # the outside of the module's own made the cost quadratic (288 s a wing);
    # `area_ratio_min` inside does the same job per rung.
    #
    # Only ever ask for the ONE output this path needs. The canopy is ~9 s of
    # boolean work that the single-solid STL throws away, and the airframe is
    # identical either way - the magnet pads are carved into the cutter, not
    # derived from the lid.
    from . import hatch as _hatch

    bay_mm: dict = {}
    try:
        # root principle (user, rounds 8-10): the inner hull cavity itself
        # continues aft of the hatch, born inside build_bay's own
        # survey/plan/band machinery - not an add-on cutter. Everything
        # the request needs is computed by `bay_request` (shared with the
        # sweep probe, so what is audited is what is built).
        bay = _hatch.build_bay(
            wing, airframe=airframe, magnets=True,
            canopy=separate_parts, one_piece=not separate_parts,
            **bay_req)
    except Exception as exc:
        bay = None
        bay_mm = {"ok": False, "reason": f"{type(exc).__name__}: {exc}"}

    if bay is not None and not bay.ok:
        # a bay that refused must SAY so - the first root-principle build
        # shipped a solid centre body and nothing in the report showed it
        bay_mm = {"ok": False, **dict(bay.bay_mm or {})}
    if bay is not None and bay.ok:
        built = bay.airframe if separate_parts else bay.airframe_onepiece
        if built is not None and built.isValid() and len(built.Solids()) == 1:
            airframe = built
            bay_built = True
            bay_mm = dict(bay.bay_mm or {})
            if separate_parts and bay.lid is not None:
                lid = bay.lid
            # MEASURE the void's z band at its aft wall, off the airframe the
            # cut actually produced. Nothing else can be trusted for this:
            # the hatch's planning profiles put the ceiling at +25 where the
            # carved air starts at -13, and its `cavity` solid is the
            # compartment CONCEPT (it reaches up through the hatch aperture),
            # not the hole in this airframe. Classifying the built solid is
            # the ground truth by construction.
            zb = _void_z_band(airframe, wing, bay_mm)
            if zb is not None:
                bay_mm["z_floor_aft_mm"], bay_mm["z_ceil_aft_mm"] = zb

    # screw holes last, so they go through boss + skin + bulkhead together
    for cutter in mount_cutters:
        try:
            drilled = _heal(airframe.cut(cutter))
            if drilled.isValid() and len(drilled.Solids()) == 1:
                airframe = drilled
        except Exception:
            continue

    parts["airframe"] = airframe
    if lid is not None:
        parts["hatch_lid"] = lid

    # Aft limit for the fins. The recorded length is the budget, but never let
    # a rounding difference against the physics squash a fin the model needs -
    # the built surface's own trailing edge is the floor.
    x_aft = max(float(g.get("length_total_m", 0.0)) * MM, ext["x_max"])

    # ---- vertical surfaces (SPEC §5 "Fin construction") -------------------
    vs = g.get("vstab") or {}          # meta below still names the type
    vs_type = str(vs.get("type", "none"))
    fins, fin_placed = _vertical_surfaces(g, wing, fin_af, span,
                                          x_fin_limit, x_aft)
    parts.update(fins)

    # NO separate canopy blister. The hatch lid IS the top of the centre body,
    # so a second fairing on the same spine was both redundant and confusing -
    # it showed up as an extra body in the STEP export that corresponded to
    # nothing a builder would make.

    # ---- fixed surfaces become part of the structure -----------------------
    # Nothing on a flying wing's vertical surfaces moves: there is no rudder,
    # the elevons do all the work. A stationary fin is therefore structure, not
    # a component, so it is fused into the surface it stands on instead of
    # being handed over as a loose body to glue on. (The split into
    # centre_body / wing panels then puts each fin with the piece it belongs
    # to: a centre fin with the body, winglets with their wing.)
    for fin_name in ("fin", "fin_left", "fin_right",
                     "winglet_left", "winglet_right"):
        fin_solid = parts.pop(fin_name, None)
        if fin_solid is None:
            continue
        if fin_name == "fin" and bay_built and bay is not None \
                and bay.cavity is not None:
            # THE CENTRE FIN'S ROOT FOLLOWS THE INNER HULL. The fin is built
            # with a straight root buried 0.35 t under the crown so that,
            # with the crown falling aft, its whole chord stays inside the
            # skin - on a deep centre body that is a blade reaching 30 mm
            # down, nearly to the floor. With the cavity continuing under
            # the fin (round 14) that blade would stand in the compartment
            # as a centre wall: two channels either side, the very
            # "two boxes" look the user rejected. So the root is trimmed to
            # the cavity's own roof lifted by _FIN_ROOT_BURY_MM: it sits
            # that deep in the >= 3 mm roof along its entire chord, bonds
            # everywhere, and never intrudes. Only the centre fin - winglets
            # and twin fins stand outboard of the bay.
            # The buried blade can be MORE than half the fin's volume on a
            # deep body (the 675 mm fpv wing: root 30 mm down, fin 40 mm
            # up), so the sanity floor is 15 % - enough to catch a cut that
            # ate the fin, not one that did its job. A rejected trim is
            # named in the warnings: it means a wall down the compartment.
            fin_solid = trim_root_to_cavity(fin_solid, bay.cavity,
                                            bay_mm.setdefault("warnings", []),
                                            what="centre fin")
        try:
            merged = _heal(parts["airframe"].fuse(fin_solid))
            if merged.isValid() and len(merged.Solids()) == 1:
                parts["airframe"] = merged
            else:
                parts[fin_name] = fin_solid      # keep it rather than lose it
        except Exception:
            parts[fin_name] = fin_solid

    if _probe is not None:
        # probe seam (tools_probe_fin_intrusion.py): hand back the
        # pieces exactly as built and stop here - nothing downstream
        # adds material to a tail surface, it only cuts
        _probe.update(
            airframe=parts["airframe"], fins=fins,
            airframe_pre=(bay.airframe if bay_built and bay is not None
                          else None),
            cavity=(bay.cavity if bay_built and bay is not None
                    else None),
            bay_mm=bay_mm, warnings=list(bay_mm.get("warnings") or []))
        return parts, grooves, {"bay": bay_mm, "probe": True}

    # ---- elevon hinge grooves ---------------------------------------------
    _progress("hinges")
    el = g.get("elevons") or {}
    if el:
        el_in = float(el.get("inner_frac", 0.35))
        el_out = float(el.get("outer_frac", 0.95))
        el_c = float(el.get("chord_frac", 0.28))
        if separate_parts:
            # Real moving surfaces: cut free of the wing, hinged on a printed
            # pin joint, so they can actually be driven by a servo.
            airframe, elevons, hinge_report = _separate_elevons(
                parts["airframe"], wing, el_in, el_out, el_c)
            parts["airframe"] = airframe
            parts.update(elevons)
            # Servo bays in the lower skin, horns fused onto the surfaces they
            # drive, and the wire runs that feed them. Only in the parts build:
            # the one-piece STL has no separate elevon to hang a horn on.
            _progress("servos")
            airframe, elevons2, servo_report = _install_servos(
                parts["airframe"], wing, elevons, el_in, el_out, el_c, wall,
                mount_spec, body, bay_mm)
            parts["airframe"] = airframe
            parts.update(elevons2)
        else:
            # the one-piece STL keeps them attached and only scribes the line
            for cutter in _elevon_grooves(
                    wing, g.get("airfoil", "RFX-9 reflexed"),
                    el_in, el_out, el_c):
                grooves.append(("airframe", cutter))
            # ...but it still gets its servo bays and its wire runs
            _progress("servos")
            airframe, _none, servo_report = _install_servos(
                parts["airframe"], wing, {}, el_in, el_out, el_c, wall,
                mount_spec, body, bay_mm)
            parts["airframe"] = airframe

    # ---- CG marker ---------------------------------------------------------
    st = design.get("stability", {})
    x_cg = float(st.get("x_cg_m", 0.0)) * MM
    if x_cg <= 0.0:
        x_cg = wing.section(0.0).le.x + 0.30 * wing.section(0.0).chord
    z_bot = wing.keel_z(0.0, wing.xc_at(0.0, x_cg))
    parts["cg_marker"] = _cg_marker(x_cg, z_bot + 1.0,
                                    scale=wing.half / 550.0)

    meta = {
        "units": "mm",
        "planform": design.get("planform", design.get("config", "swept")),
        "x_cg_mm": float(st.get("x_cg_m", 0.0)) * MM,
        "x_np_mm": float(st.get("x_np_m", 0.0)) * MM,
        "mac_mm": float(st.get("mac_m", 0.0)) * MM,
        "x_le_mac_mm": float(st.get("x_le_mac_m", 0.0)) * MM,
        "y_mac_mm": float(st.get("y_mac_m", 0.0)) * MM,
        "cg_pct_mac": st.get("cg_pct_mac", 0.0),
        "static_margin": st.get("static_margin", 0.0),
        "span_mm": span,
        "length_mm": float(g.get("length_total_m", 0.0)) * MM,
        "height_mm": float(g.get("height_total_m", 0.0)) * MM,
        "body_half_width_mm": wing.fb * wing.half,
        "root_chord_mm": wing.section(0.0).chord,
        "body_depth_mm": wing.tc * wing.section(0.0).chord
        * wing.section(0.0).t_scale,
        "surface_length_mm": ext["x_max"],
        "control_surfaces": g.get("control_surfaces", []),
        "elevons": el,
        "vstab": {"type": vs_type, **{k: v for k, v in vs.items()
                                      if k != "type"}},
        "fin_stations": fin_placed,
        "wall_mm": g.get("wall_mm", 1.2),
        "hinges": hinge_report,
        "servos": servo_report,
        "bay": bay_mm,
        "valid_solid": all(bool(p.isValid()) for p in parts.values()),
    }
    return parts, grooves, meta


def airframe_extents_m(design: dict) -> dict[str, float]:
    """Envelope the CAD is going to occupy, in METRES, without building it.

    Cheap (a few ms - no OpenCASCADE at all), so the optimizer can record
    `length_total_m` and `height_total_m` from the shape it is actually asking
    for instead of estimating them. Keys: `length_m` (nose datum to aft-most
    point), `height_m` (lowest to highest point), `z_min_m`, `z_max_m`.

    Reads the same `geometry` keys as the builder, so the two cannot drift.
    """
    g = design["geometry"]
    wing = _wing_from_design(g)
    ext = wing.extents()
    z_lo, z_hi = ext["z_min"], ext["z_max"]

    vs = g.get("vstab") or {}
    vs_type = str(vs.get("type", "none"))
    if vs_type in ("winglets", "twin_fin", "center_fin"):
        # Each fin type stands on a DIFFERENT chordwise station, and the crown
        # falls away steeply toward the trailing edge - so the top has to be
        # computed the way the builder computes it, not from one nominal
        # station. Evaluating them all at 0.35 c (near max thickness, i.e. the
        # highest crown there is) over-reported the plank by 13 mm.
        # `test_recorded_dimensions_are_the_real_ones` pins these together.
        s_wing = float(g.get("area_m2", 0.0)) * MM * MM or (
            wing.c_root * 2.0 * wing.half)
        h_fin, cr_fin, _ct = _fin_dims(vs, s_wing)
        if vs_type == "center_fin":
            sec = wing.section(0.0)
            cr = min(cr_fin, 0.55 * sec.chord)
            x_te_fin = sec.le.x + sec.chord - max(2.0, 0.03 * cr)
            mm_spec = g.get("motor_mount") or {}
            if str(mm_spec.get("type", "")) == "pusher":
                # The BUILDER slides a centre fin forward of the pusher
                # nacelle root (prop-disc clearance), where the crown is
                # HIGHER and the root band digs deeper - so the fin's top
                # and root must be evaluated at the SLID station or the
                # recorded height under-reports by several mm. Found on the
                # v3 delta (whose default is exactly centre fin + pusher,
                # measured 5.4 mm short); latent for any fw design flown in
                # that combination. Tractor centre fins (the v1 plank) are
                # untouched - their numbers do not change.
                r_plate = float(mm_spec.get("plate_radius_mm", 16.0))
                t_plate = float(mm_spec.get("plate_thickness_mm", 4.0))
                wall_e = max(float(g.get("wall_mm", 1.2)), 0.6)
                depth_n = max(2.2 * r_plate, 3.0 * t_plate, 6.0 * wall_e,
                              18.0)
                x_root_n = float(mm_spec.get("x_m", 0.0)) * MM - depth_n
                x_te_fin = min(x_te_fin, x_root_n - 4.0)
                xc_lo = wing.xc_at(0.0, x_te_fin - cr)
                z_lo_fin = (min(wing.crown_z(0.0, xc_lo),
                                wing.crown_z(0.0, min(xc_lo + 0.25, 1.0)))
                            - 0.35 * wing.tc * sec.chord * sec.t_scale)
                z_lo = min(z_lo, z_lo_fin)
            else:
                xc_lo = wing.xc_at(0.0, sec.le.x + sec.chord - cr)
            z_hi = max(z_hi,
                       wing.crown_z(0.0, min(xc_lo + 0.15, 1.0)) + h_fin)
        elif vs_type == "twin_fin":
            yf = _clamp(float(vs.get("y_frac", 0.58) or 0.58), 0.20, 0.88)
            sec = wing.section(yf)
            cr = min(cr_fin, 0.92 * sec.chord)
            xc_mid = wing.xc_at(yf, sec.le.x + 0.98 * sec.chord - 0.5 * cr)
            z_hi = max(z_hi, wing.crown_z(yf, xc_mid) + h_fin)
        else:                                   # winglets, canted at the tip
            z_hi = max(z_hi, wing.crown_z(0.965, 0.55) + h_fin)

    # The motor mount is a real boss on a real bulkhead. On a deep body it is
    # buried; on a thin one (a bell wing is only ~44 mm deep) a mount ring big
    # enough to take the bolt circle genuinely stands proud, exactly as it does
    # on the real aircraft - so it belongs in the recorded height rather than
    # being quietly ignored.
    mm = g.get("motor_mount") or {}
    if mm:
        z_m = float(mm.get("z_m", 0.0)) * MM
        r_plate = float(mm.get("plate_radius_mm", 0.0))
        if r_plate > 0:
            z_hi = max(z_hi, z_m + r_plate)
            z_lo = min(z_lo, z_m - r_plate)
    # The CG keel marker only counts if it actually reaches below the skin -
    # on a deep centre body it does not protrude at all.
    z_lo = min(z_lo, z_lo + 1.0 - _clamp(2.5 * wing.half / 550.0, 1.5, 4.0))
    # The sampler above is exact on the SECTION model, but the lofted B-spline
    # passes through those sections and bulges a little between them - up to
    # ~2.3 mm on a BWB, whose leading-edge root extension is the sharpest
    # spanwise curvature on any of the planforms. That overshoot is a property
    # of the surface, not of the sampling, so it cannot be resolved by taking
    # more stations; it has to be carried as an allowance. Length only: the
    # height already comes out conservative because the fin tops are computed
    # from the crown the fin is rooted on.
    length = ext["x_max"] + max(3.0, 0.006 * ext["x_max"])
    return {"length_m": length / MM,
            "height_m": (z_hi - z_lo) / MM,
            "z_min_m": z_lo / MM, "z_max_m": z_hi / MM}


def _install_servos(airframe: Solid, wing: "_BlendedWing",
                    elevons: dict, inner: float, outer: float,
                    chord_frac: float, wall: float,
                    mount_spec: dict, body: dict, bay_mm: dict):
    """Recess the elevon servos, fuse a horn onto each surface, run the wires.

    Returns (airframe, {name: surface_with_horn}, report).

    The servo goes into the LOWER skin, shaft axis spanwise, so its arm and the
    control horn turn about parallel axes and the linkage is a true planar
    four-bar. The horn is fused into the control surface, so it prints as one
    green part and there is nothing to glue on. The builder bends the pushrod.

    Everything here is best-effort and individually gated: a wing too shallow
    for a 9 g servo keeps its skin and simply reports why, because a bay cut
    through the crown is worse than no bay at all. Same for the wire runs.
    """
    from . import conduits as _cd
    from . import servos as _sv

    report: dict = {"bays": {}, "horns": {}, "conduits": {}}
    out: dict[str, Solid] = {}
    xc = _clamp(1.0 - chord_frac, 0.45, 0.90)
    inner = _clamp(inner, max(wing.fb, 0.10), 0.85)
    outer = _clamp(outer, inner + 0.10, _TIP_START - 0.01)
    # Where the ARM goes, not where the case goes. The pushrod is a straight
    # wire, so the arm and the horn have to stand in one plane; the arm sits on
    # the spline near one end of the case, so asking for a case station and
    # letting the arm fall where it may is what put them 65 mm apart. Pick the
    # arm station - just inboard of mid on the elevon's inner half, which is
    # where a foamie puts it - and let `servo_bay` place the case to suit.
    y_arm_frac = _clamp(inner + 0.10, wing.fb + 0.06, outer - 0.10)

    conduit_cutters: list[tuple[str, Solid | None]] = []
    bay_cutters: list[tuple[str, object]] = []
    for sgn, name in ((1.0, "elevon_right"), (-1.0, "elevon_left")):
        # The bay and the wire run are cuts in the AIRFRAME, so they happen
        # whether or not this build separates the control surfaces - otherwise
        # the one-piece STL would come out with nowhere to put a servo and no
        # way to route a wire. Only the horn needs a surface to stand on.
        surface = elevons.get(name)
        try:
            p_in, p_out, t_in, t_out = _elevon_hinge_line(
                wing, sgn, inner, outer, xc)
        except Exception:
            continue

        # ---- the bay ------------------------------------------------------
        x_hinge = 0.5 * (p_in.x + p_out.x)
        try:
            bay = _sv.servo_bay(wing, y_frac=y_arm_frac, x_hinge=x_hinge,
                                sgn=sgn, wall=wall,
                                arm_y_mm=sgn * y_arm_frac * wing.half,
                                # The servo may NOT walk inboard past the
                                # surface it drives. `servo_bay` walks inboard
                                # looking for section deep enough to roof the
                                # pocket, and on a thin outer panel it will
                                # happily walk clean out from under the elevon
                                # - a bell wing put the arm 32 mm inboard of
                                # the elevon root, where no straight pushrod
                                # can reach the horn. A pocket that cannot
                                # drive its own surface is not a pocket; better
                                # to refuse and say the section is too thin.
                                params={"f_min": inner + 0.02})
        except Exception as exc:
            bay = None
            report["bays"][name] = {"ok": False, "reason": str(exc)}
        if bay is not None and bay.ok and bay.cutter is not None:
            # Collected, not cut yet. Cutting one bay and mesh-checking it
            # before cutting the other is the expensive mistake: `tessellate`
            # writes a triangulation onto the shared TShape and the next
            # boolean then drags it along - the same cut runs ~25x slower on a
            # tessellated solid. Both bays go in one boolean, checked once.
            bay_cutters.append((name, bay))
        elif bay is not None:
            report["bays"][name] = {"ok": False,
                                    "reason": bay.reason or "no room"}

        # ---- the horn, fused into the surface it drives --------------------
        if surface is None:
            continue                    # one-piece build: nothing to hang it on
        # Stand the horn in the servo arm's plane. If the bay was refused there
        # is no arm to line up with, so it falls back to mid-span.
        align_y = (bay.arm_hole.y if bay is not None and bay.ok else None)
        # The setback and the hole depths are both scaled by the section
        # thickness where the horn actually lands, so interpolate it there
        # rather than handing over the mid-span value.
        sf_h = 0.5 if align_y is None else _clamp(
            (align_y - p_in.y) / ((p_out.y - p_in.y) or 1.0), 0.0, 1.0)
        try:
            horned, hinfo = _sv.control_horn(
                surface, hinge_p_in=p_in, hinge_p_out=p_out,
                station_frac=0.5, align_world_y=align_y,
                thickness=t_in + sf_h * (t_out - t_in))
            if horned.isValid() and len(horned.Solids()) == 1:
                out[name] = horned
                report["horns"][name] = hinfo
                # the four-bar the builder will actually rig: throws, ratio,
                # rod length, and the arm clock that balances the throws
                if bay is not None and bay.ok and hinfo.get("holes"):
                    try:
                        report.setdefault("linkage", {})[name] = \
                            _sv.linkage(bay, hinfo)
                    except Exception:
                        pass
            else:
                out[name] = surface
                report["horns"][name] = {"ok": False,
                                         "reason": "horn broke the surface"}
        except Exception as exc:
            out[name] = surface
            report["horns"][name] = {"ok": False, "reason": str(exc)}

    # ---- every servo bay in ONE boolean, validated once --------------------
    if bay_cutters:
        tool = bay_cutters[0][1].cutter
        for _n, b in bay_cutters[1:]:
            try:
                tool = tool.fuse(b.cutter)
            except Exception:
                tool = None
                break
        cut = None
        if tool is not None:
            try:
                cut = _heal(airframe.cut(tool))
            except Exception:
                cut = None
        ok = (cut is not None and cut.isValid() and len(cut.Solids()) == 1
              and _tessellates_cleanly(cut, tol=0.3, min_ratio=0.985))
        if ok:
            airframe = cut
        # rounds 8-10 became a ROOT PRINCIPLE: the cavity's rear extension
        # is born inside hatch.build_bay itself (one native void, the
        # hull's own inner surfaces). The runs read the published
        # per-station cavity band; the old _aft_hollow add-on is retired.
        for nm, b in bay_cutters:
            if ok:
                report["bays"][nm] = {
                    "ok": True, "y_frac": b.y_frac,
                    "moved_inboard": b.moved_inboard, **(b.dims or {})}
                if b.cable_exit is not None:
                    c_srv, i_srv = _servo_run(
                        _cd, wing, b, wall, body, bay_mm,
                        hinge_xc=xc,
                        span_mm=(inner * wing.half,
                                 outer * wing.half))
                    conduit_cutters.append((f"servo_{nm}", c_srv))
                    if i_srv:
                        report["conduits"][f"servo_{nm}"] = i_srv
            else:
                report["bays"][nm] = {
                    "ok": False,
                    "reason": "bay cut would not mesh; skin left intact"}

    # ---- the motor wiring: belly entry near the mount + internal run -------
    # Builder's spec, third iteration and now explicit: a 6.5 mm hole through
    # the BOTTOM skin close to the motor, leading INSIDE the body and running
    # enclosed until it opens into the equipment bay. Cut with the rest and
    # mesh-gated as one.
    try:
        cutters, info = _motor_entry_run(_cd, wing, bay_mm, mount_spec, wall)
        conduit_cutters.extend(cutters)
        report["conduits"]["motor"] = info
    except Exception as exc:
        report["conduits"]["motor"] = {"ok": False, "reason": str(exc)}

    if conduit_cutters:
        try:
            airframe, cinfo = _cd.cut_conduits(airframe, conduit_cutters)
            # MERGE the cut verdicts into the routing info - replacing it
            # wholesale threw away the route (`path_mm`), which is the only
            # thing that lets anything downstream verify the channel actually
            # arrives where it claims to.
            for k, v in cinfo.items():
                prev = report["conduits"].get(k)
                if isinstance(prev, dict) and isinstance(v, dict):
                    prev.update(v)
                else:
                    report["conduits"][k] = v
        except Exception as exc:
            report["conduits"]["error"] = str(exc)
        # existence: a straight run's centreline must classify open end
        # to end - a cut can report applied and still dead-end in a
        # bulkhead above the compartment
        for k, ci in list(report["conduits"].items()):
            if not isinstance(ci, dict) or not ci.get("path_mm"):
                continue
            applied = (bool((report["conduits"].get("motor_run") or {})
                            .get("applied")) if k == "motor"
                       else bool(ci.get("applied")))
            if not applied:
                continue
            try:
                ro = _cd.route_is_open(airframe, _cd.path_vectors(ci))
                ci["route_open"] = bool(ro.get("open"))
                if not ro.get("open"):
                    ci["route_open_detail"] = ro
                    ci["applied"] = False
                    if k == "motor":
                        ci["ok"] = False
                        mr = report["conduits"].get("motor_run")
                        if isinstance(mr, dict):
                            mr["applied"] = False
                    ci["why"] = ("cut reported applied but the bore is "
                                 "blocked")
            except Exception as exc:
                ci["route_open"] = False
                ci["route_open_detail"] = str(exc)
    return airframe, out, _jsonable(report)


def _jsonable(obj):
    """Strip CadQuery objects out of a report so it can go over the API.

    The servo and conduit modules report real geometry - cable exits, spline
    positions, route waypoints - as `Vector`s, which FastAPI cannot serialise.
    They are genuinely useful numbers, so they are converted rather than
    dropped."""
    if isinstance(obj, Vector):
        return [round(obj.x, 3), round(obj.y, 3), round(obj.z, 3)]
    if isinstance(obj, dict):
        return {str(k): _jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_jsonable(v) for v in obj]
    if isinstance(obj, (str, bool, int, float, type(None))):
        return obj
    if isinstance(obj, np.generic):
        return obj.item()
    if isinstance(obj, np.ndarray):
        return _jsonable(obj.tolist())
    return str(obj)


def _void_z_band(airframe: Solid, wing: "_BlendedWing",
                 bay_mm: dict) -> tuple[float, float] | None:
    """(floor, ceiling) of the carved bay void at its aft wall, world mm.

    Classify a z ladder through the BUILT airframe at the void's aft end on
    the centreline: the contiguous OUT band inside the skin is the
    compartment's air. Measured on the finished solid because every model of
    it lied - the hatch's planning profiles put the ceiling at +25 where the
    carved air starts at -13, and its `cavity` solid is the compartment
    concept (reaching up through the hatch aperture), not the hole in this
    airframe.
    """
    try:
        from OCP.BRepClass3d import BRepClass3d_SolidClassifier
        from OCP.gp import gp_Pnt
        from OCP.TopAbs import TopAbs_OUT

        x1 = float((bay_mm or {}).get("x1_mm", 0.0))
        if airframe is None or x1 <= 0.0:
            return None
        x = x1 - 3.0
        xc = wing.xc_at(0.0, x)
        if not (0.0 <= xc <= 1.0):
            return None
        z_hi = float(wing.crown_z(0.0, xc)) - 1.0
        z_lo = float(wing.keel_z(0.0, xc)) + 1.0
        if z_hi - z_lo < 8.0:
            return None
        cl = BRepClass3d_SolidClassifier(airframe.wrapped)
        air = []
        for z in np.linspace(z_lo, z_hi, 60):
            cl.Perform(gp_Pnt(x, 0.0, float(z)), 1e-6)
            if cl.State() == TopAbs_OUT:
                air.append(float(z))
        if len(air) < 3:
            return None
        # the LONGEST contiguous run, so a stray open feature (a scribe, a
        # magnet pocket) cannot masquerade as the compartment
        step = (z_hi - z_lo) / 59.0
        runs, cur = [], [air[0]]
        for z in air[1:]:
            if z - cur[-1] <= 1.8 * step:
                cur.append(z)
            else:
                runs.append(cur)
                cur = [z]
        runs.append(cur)
        best = max(runs, key=len)
        if len(best) < 3:
            return None
        return best[0], best[-1]
    except Exception:
        return None


def _motor_entry_run(_cd, wing: "_BlendedWing", bay_mm: dict,
                     mount_spec: dict, wall: float, port_d: float = 8.25):
    """The motor wiring, builder's spec round 5: ONE straight round pipe.

    It breaks through the BOTTOM skin near the motor mount and runs in a
    single straight extrusion - angled, never bent - until it opens into
    the equipment bay void. A straight rod fed in at the belly hole comes
    out inside the compartment; the old vertical entry bore meeting a
    curved internal channel at an angle is gone.

    Rounds 12-14 made the rear cavity the hull itself: `bay_mm["x1_mm"]`
    is now the END of the tapered rear extension, not a box bulkhead, and
    `cavity_stations_mm` is the per-station band of the void that really
    exists. The pipe aims at that band the way the servo runs do - at the
    real void, at its own station - and enters it through the extension's
    end wall / floor corner. The entry walk keeps its one rule: the belly
    station's OWN wing section must hold the bore (the motor nacelle is
    structure and is never breached - a line grazing its faired underside
    at 24 deg opens a ~50 mm slot along it, measured on the 675 swept
    wing). What changed is where the walk stops: the old fixed standoff
    short of `x1_mm` was written for a bulkhead the line had to pierce,
    and on the hull cavity it left only the thin tail (52-64 mm of section
    under 14 mm deep, on the user's 675 design and the UI default) and
    refused every design with a rear extension. Now the walk runs right up
    to the end wall, so the hole lands at the aft-most station that can
    carry the pipe - the same station the pre-extension build chose.

    Returns ([(name, cutter)], info). Cut via `cut_conduits`, mesh-gated.
    """
    bm = bay_mm or {}
    if bm.get("x1_mm") is None:
        return [], {"ok": False,
                    "reason": "no bay void - nowhere for the wires to go"}
    x_bay0 = float(bm.get("x0_mm", bm["x1_mm"]))
    x_bay1 = float(bm["x1_mm"])
    x_mount = float(mount_spec.get("x_m", 0.0)) * MM
    y0 = float(mount_spec.get("y_m", 0.0)) * MM
    z_m = float(mount_spec.get("z_m", 0.0)) * MM

    pusher = x_mount >= 0.5 * (x_bay0 + x_bay1)
    step = -1.0 if pusher else 1.0               # walk direction, toward bay
    x_wall = x_bay1 if pusher else x_bay0
    wall_p = max(wall, 2.0)
    r_h = 0.5 * float(port_d) + 0.6              # bore + a hair of air
    r_w = 0.5 * float(port_d) * float(getattr(_cd, "OVAL_W_RATIO", 1.0)) + 0.6

    # The nacelle, as MATERIAL the line check may ride through (the same
    # model `conduits.motor_conduit` builds; it is inscribed in the real
    # loft, so it only ever under-reports). It never hosts the entry.
    nac = None
    r_plate = float(mount_spec.get("plate_radius_mm", 0.0) or 0.0)
    if r_plate > 0.0 and hasattr(_cd, "_Nacelle"):
        t_plate = float(mount_spec.get("plate_thickness_mm", 4.0) or 4.0)
        depth_n = max(2.2 * r_plate, 3.0 * t_plate, 6.0 * wall, 18.0)
        try:
            nac = _cd._Nacelle(wing, x_mount, x_mount + step * depth_n,
                               y0, z_m, r_plate)
        except Exception:
            nac = None

    f0 = _clamp(y0 / max(wing.half, 1e-6), -0.995, 0.995)

    def belly_z(x: float):
        """Lowest skin at (x, y0): the wing keel, or the nacelle's underside
        where the fairing hangs below it. None off the plan."""
        xc = wing.xc_at(f0, x)
        if not (0.0 <= xc <= 1.0):
            return None
        z = float(wing.keel_z(f0, xc))
        if nac is not None and nac.covers(x):
            try:
                zc_n, hh_n, a_n = nac.section(x)
                if a_n > 0.0 and hh_n > 0.05 and abs(y0 - nac.y_c) < a_n:
                    z = min(z, zc_n - hh_n * math.sqrt(
                        max(1.0 - ((y0 - nac.y_c) / a_n) ** 2, 0.0)))
            except Exception:
                pass
        return z

    # ---- where the pipe must arrive: the void that actually exists -------
    # (x_t, y_t, (z_lo, z_hi), z_pref) targets, nearest the end wall first:
    # the nearest gives the shortest pipe and the steepest, most compact
    # belly breach; a deeper one is a shallower line when the section just
    # aft of the end wall will not carry the steep one.
    targets: list[tuple[float, float, tuple[float, float], float]] = []
    for r in (bm.get("cavity_stations_mm") or []):
        try:
            x_t, hw_t, zl_t, zh_t = (float(v) for v in r[:4])
        except Exception:
            continue
        if pusher and x_t > x_bay1 - 4.0:
            continue
        if not pusher and x_t < x_bay0 + 4.0:
            continue
        if zh_t - zl_t < 2.0 * r_h + 2.0 or hw_t < r_w + 2.0:
            continue
        y_t = _clamp(y0, -(hw_t - r_w - 2.0), hw_t - r_w - 2.0)
        zb = (zl_t + r_h + 1.0, zh_t - r_h - 1.0)
        targets.append((x_t, y_t, zb, 0.5 * (zb[0] + zb[1])))
    targets.sort(key=(lambda t: -t[0]) if pusher else (lambda t: t[0]))
    targets = targets[:8]
    legacy = not targets
    if legacy:
        # no per-station band (a box bay): the far end is 6 mm inside the
        # void past its aft bulkhead, in the MEASURED height band there
        hw_bay = 0.5 * float(bm.get("width_mm", 40.0))
        y_end = _clamp(y0, -0.6 * hw_bay, 0.6 * hw_bay)
        x_end = x_wall + step * 6.0
        _zf, _zc = bm.get("z_floor_aft_mm"), bm.get("z_ceil_aft_mm")
        if _zf is not None and _zc is not None \
                and float(_zc) - float(_zf) > 12.0:
            z_band = (float(_zf) + 5.0, float(_zc) - 5.0)
            z_pref = float(_zc) - 6.0
        elif _zf is not None and _zc is not None:
            zm = 0.5 * (float(_zf) + float(_zc))
            z_band, z_pref = (zm - 1.5, zm + 1.5), zm
        else:
            xcw = wing.xc_at(0.0, x_end)
            zm = 0.5 * (float(wing.crown_z(0.0, xcw))
                        + float(wing.keel_z(0.0, xcw)))
            z_band, z_pref = (zm - 3.0, zm + 3.0), zm
        targets = [(x_end, y_end, z_band, z_pref)]

    # ---- candidate belly stations: walk from the mount toward the bay ------
    # A station qualifies when the WING section there can hold the pipe
    # with its walls; the straight router is the authority on whether the
    # LINE from it really fits, and a refused candidate hands over to the
    # next. On a hull cavity the walk runs up to (a hair over) the end wall
    # - there is no bulkhead to stand off from; on a box bay the old
    # standoff keeps the line piercing the bulkhead squarely.
    need = float(port_d) + 2.0 * wall_p + 2.0
    gap = abs(x_mount - x_wall)
    standoff = (min(15.0, max(5.0, 0.35 * gap)) if legacy else -6.0)
    dx0 = min(8.0, max(4.0, 0.30 * gap))
    dx_max = max(90.0, gap - 12.0) if legacy else gap + 6.5
    candidates: list[tuple[float, float]] = []
    for dx in np.arange(dx0, dx_max, 3.0):
        x = x_mount + step * dx
        if (pusher and x <= x_wall + standoff) or \
                (not pusher and x >= x_wall - standoff):
            break                                # ran into the bay itself
        xc = wing.xc_at(0.0, x)
        if not (0.0 <= xc <= 1.0):
            continue
        crown, keel = float(wing.crown_z(0.0, xc)), float(wing.keel_z(0.0, xc))
        if crown - keel >= need:
            candidates.append((x, keel))
    if not candidates:
        return [], {"ok": False,
                    "reason": (
                        f"no usable entry station: mount x={x_mount:.0f}, "
                        f"{'pusher' if pusher else 'tractor'} toward the "
                        f"{'bay wall' if legacy else 'cavity end'} "
                        f"x={x_wall:.0f} (gap {gap:.0f} mm, standoff "
                        f"{standoff:.0f}, first station {dx0:.0f}), needs "
                        f"{need:.1f} mm of section depth")}

    run, info, target = None, {}, None
    last_info: dict | None = None
    x_entry = candidates[0][0]
    n_tried = 0
    for x_entry, z_keel in candidates[:14]:
        zb_e = belly_z(x_entry)
        z_start = (min(z_keel, zb_e) if zb_e is not None else z_keel) - 6.0
        start = Vector(x_entry, y0, z_start)     # free air below the skin
        for x_t, y_t, zb, z_pref in targets:
            if abs(x_t - x_entry) < 6.0:
                continue
            run, info = _cd.straight_conduit(
                [wing], start=start, end_xy=(x_t, y_t),
                end_z_band=zb, wall=wall_p,
                params={"start_overshoot_mm": 2.0, "end_overshoot_mm": 4.0,
                        "prefer_z": z_pref, "belly_entry": True,
                        "d_mm": float(port_d), "nacelle": nac})
            n_tried += 1
            if run is not None:
                target = (x_t, y_t)
                break
            last_info = info
        if run is not None:
            break

    info = dict(info or last_info or {})
    # where the axis actually crosses the belly skin - the hole the builder
    # feeds the leads into
    x_skin, z_skin = x_entry, None
    if run is not None:
        pts = _cd.path_vectors(info)
        for a, b in zip(pts, pts[1:]):
            ka, kb = belly_z(a.x), belly_z(b.x)
            if ka is None or kb is None:
                continue
            da, db = a.z - ka, b.z - kb
            if da <= 0.0 <= db or db <= 0.0 <= da:
                t = abs(da) / max(abs(da) + abs(db), 1e-9)
                x_skin = a.x + (b.x - a.x) * t
                z_skin = ka + (kb - ka) * t
                break
    if z_skin is None:
        zs = belly_z(x_skin)
        z_skin = zs if zs is not None else 0.0

    toward = "forward" if pusher else "aft"
    info.update({
        "ok": run is not None,
        "kind": "motor",
        "what": ("one straight pipe: belly entry near the motor -> "
                 "equipment bay, no bends"),
        "entry_d_mm": float(port_d),
        "entry_x_mm": round(x_skin, 1),
        "entry_y_mm": round(y0, 1),
        "entry_z_skin_mm": round(z_skin, 1),
        "entry_aft_of_bay_mm": round(x_skin - x_bay1, 1),
        "mount_to_entry_mm": round(abs(x_mount - x_skin), 1),
        "cavity_end_x_mm": round(x_wall, 1),
        "arrives_via": ("the bay's aft bulkhead" if legacy else
                        "the rear extension's end wall / floor"),
        "candidates_tried": n_tried,
        "note": ("feed the three motor leads into the belly hole "
                 f"{abs(x_mount - x_skin):.0f} mm {toward} of the mount; "
                 "the pipe runs dead straight into the equipment bay - a "
                 "straight rod passes end to end"),
    })
    if target is not None:
        info["target_x_mm"] = round(target[0], 1)
        info["target_y_mm"] = round(target[1], 1)
    if run is None and not info.get("reason"):
        info["reason"] = (
            f"no straight line from any belly station between x="
            f"{candidates[0][0]:.0f} and x={candidates[min(13, len(candidates) - 1)][0]:.0f} "
            f"reaches the cavity band inside the skin: "
            f"{info.get('skipped') or 'refused'}")
    return ([("motor_run", run)] if run is not None else []), info


def _aft_hollow(_cd, wing, bay_mm: dict, wall: float,
                exits: list, hinge_xc: float,
                span_mm: tuple[float, float] = (0.0, 1e9)):
    """ONE smooth aft continuation of the equipment-bay hollow (round 8).

    The builder rejected the per-run rectangular galleries: "you can just
    expand that and make it smooth and part of the inner hollow
    hull/fuseleage ... push the hollow part to the back of the plane
    (that does not mean extend the hatch) ... keep the wall thickness
    substantial like 2-3 maybe 4mm ... so the wires can fit better and be
    exposed within the main fueseleage inside." So the CAVITY itself
    continues aft of the hatch bay as one gabled-teardrop channel - the
    house's self-supporting print section - following the keel line,
    sized to swallow every servo tube mouth, with >= 3 mm of skin all
    round. The hatch and its lid are untouched; this is enclosed hull
    volume, and the wires live exposed inside it.

    Returns (cutter | None, info, override): `override` hands
    `_servo_run` the extended void so the tube opens straight into the
    main hollow - no separate chamber at all.
    """
    info: dict = {"kind": "aft_hollow"}
    try:
        x1 = float(bay_mm["x1_mm"])
        w_bay = float(bay_mm.get("width_mm") or 0.0)
        if w_bay < 24.0:
            info.update(ok=False, skipped="bay too narrow to continue aft")
            return None, info, None
        x_need = max(float(v.x) for v in exits) + 12.0
        if x_need <= x1 - 8.0:
            info.update(ok=False, skipped="not needed: grommets in bay")
            return None, info, None
        if x_need - x1 > 140.0:
            info.update(ok=False, skipped=(
                f"extension of {x_need - x1:.0f} mm would gut the body"))
            return None, info, None
        wall_e = max(float(wall), 3.0)     # builder: substantial walls
        from cadquery import Edge, Wire
        a_bay = 0.5 * (w_bay - 2.0)
        x_stop_cap = x1 + 200.0

        # Round 9 (builder): not a routed channel - the HULL ITSELF
        # continues aft. Each station is the section's own inner offset
        # (floor keel+wall, roof crown-wall), full bay width where the
        # skin allows, tapering with the hull. Maximizes the internal
        # volume for batteries / flight controllers, and the roof
        # follows the crown's own curvature, so it prints exactly like
        # the hull does.
        # near the hatch rim the roof must not rise above the BAY's own
        # ceiling: a crown-offset roof there undercuts the rim lip and
        # chips it off (measured: a 602 mm3 fragment at x 276-286,
        # z 18.6-23.6). The cap releases aft of the rim.
        zc_bay = bay_mm.get("z_ceil_aft_mm")

        def station(x: float, a_try: float):
            # roof RAMP, not a step: pure bay-ceiling cap at the hatch
            # rim, blending to the full crown offset over 20 mm. A step
            # left the smooth loft overshooting around it and slicing a
            # 0.3 mm sliver off the rim (measured, 96.8 mm3 at x=295.5);
            # the ramp is also simply the smoother shape.
            if zc_bay is not None:
                t_r = _clamp((x - (x1 + 6.0)) / 20.0, 0.0, 1.0)
                cap0 = float(zc_bay) - 0.5
            else:
                t_r, cap0 = 1.0, None
            a = a_try
            for _ in range(18):
                ys_s = [a * (-1.0 + 2.0 * i / 14.0) for i in range(15)]
                bot, top, ok = [], [], True
                for yv in ys_s:
                    f = _clamp(yv / max(wing.half, 1e-6), -0.995, 0.995)
                    xcq = float(wing.xc_at(f, x))
                    if not (0.0 <= xcq <= 0.88):
                        ok = False
                        break
                    if xcq > hinge_xc - 0.04 and \
                            span_mm[0] <= abs(yv) <= span_mm[1]:
                        ok = False
                        break
                    k = float(wing.keel_z(f, xcq)) + wall_e
                    c = float(wing.crown_z(f, xcq)) - wall_e
                    if cap0 is not None:
                        c = min(c, cap0 + t_r * max(0.0, c - cap0))
                    if c - k < 8.0:
                        ok = False
                        break
                    bot.append((yv, k))
                    top.append((yv, c))
                if ok:
                    return True, a, bot, top
                a *= 0.85
                if a < 9.0:
                    return False, a, None, None
            return False, a, None, None

        def smooth_wire(x: float, bot, top):
            """SMOOTH station wire (builder: 'not smooth and continuous
            ... seems separate and blocky'): splined floor and roof that
            follow the keel/crown, straight side walls - the same
            surface language as the hull itself. Polygon fallback."""
            pb = [Vector(x, yv, zv) for yv, zv in bot]
            pt = [Vector(x, yv, zv) for yv, zv in reversed(top)]
            try:
                eb = Edge.makeSpline(pb)
                er = Edge.makeLine(pb[-1], pt[0])
                et = Edge.makeSpline(pt)
                el = Edge.makeLine(pt[-1], pb[0])
                w = Wire.assembleEdges([eb, er, et, el])
                if w.IsClosed():
                    return w
            except Exception:
                pass
            return Wire.makePolygon(pb + pt, close=True)

        xs: list[float] = []
        wires = []
        widths: list[float] = []
        area_mm2: list[float] = []
        stations_pts: list[tuple] = []
        x = x1 - 10.0
        a_prev = a_bay
        while x <= x_stop_cap:
            ok_s, a_fit, bot, top = station(x, min(a_prev * 1.05, a_bay))
            if not ok_s:
                break
            try:
                wires.append(smooth_wire(x, bot, top))
            except Exception:
                break
            xs.append(x)
            widths.append(2.0 * a_fit)
            area_mm2.append(sum(t[1] - b[1] for t, b in zip(top, bot))
                            / 15.0 * 2.0 * a_fit)
            stations_pts.append((bot, top))
            a_prev = a_fit
            x += 6.0
        # taper the tail shut instead of a flat end wall: two shrinking
        # closure stations scaled about the last profile's centre
        if len(xs) >= 3:
            bot_l, top_l = stations_pts[-1]
            z_mid = 0.5 * (sum(z for _y, z in bot_l) / len(bot_l)
                           + sum(z for _y, z in top_l) / len(top_l))
            for dx, s in ((6.0, 0.55), (10.0, 0.22)):
                xq = xs[-1] + dx
                bot_s = [(yv * s, z_mid + (zv - z_mid) * s)
                         for yv, zv in bot_l]
                top_s = [(yv * s, z_mid + (zv - z_mid) * s)
                         for yv, zv in top_l]
                try:
                    wires.append(smooth_wire(xq, bot_s, top_s))
                except Exception:
                    break
        if len(xs) < 3 or xs[-1] <= x1 + 6.0:
            info.update(ok=False, skipped=(
                "hull too thin aft of the bay to continue the hollow "
                f"(reached x={xs[-1] if xs else x1:.0f})"))
            return None, info, None
        # RULED loft of SPLINED sections. The "blocky" look was the
        # 9-point polygon cross-sections, not the lengthwise strips:
        # splined floor/roof make each section smooth, and at 6 mm
        # stations the ruled strips deviate by micrometres - below a
        # print layer. Through-sections was tried three ways (plain,
        # BRep-round-tripped, ShapeFix-healed) and every cut came back
        # unmeshable or invalid; ruled booleans are the ones this body
        # provably accepts (round 9).
        try:
            cutter = Solid.makeLoft(wires, True)
        except Exception:
            try:
                cutter = Solid.makeLoft(wires, False)
            except Exception as exc:
                info.update(ok=False,
                            skipped=f"hull-extension loft failed: {exc}")
                return None, info, None
        if not cutter.isValid() or len(cutter.Solids()) != 1:
            info.update(ok=False,
                        skipped="hull extension lofted invalid")
            return None, info, None
        # the freshly lofted cutter carries the bad state (the house's
        # measured lesson from fuse_feature/hatch): round-trip it
        # through BRep so its per-face tolerances are normalized before
        # the boolean - the smooth loft cut to an INVALID solid without
        # this (measured, round 10)
        try:
            cutter = _round_tripped(cutter)
        except Exception:
            pass
        vol_cm3 = sum(a * 10.0 for a in area_mm2) / 1000.0
        reached = xs[-1] >= x_need - 2.0
        info.update({
            "ok": True,
            "what": ("the inner hull itself continued aft: a smooth "
                     "station-wise inner offset of the skin (floor "
                     "keel+wall, roof crown-wall, walls >= "
                     f"{wall_e:.1f} mm) maximizing internal volume for "
                     "battery/FC stowage; the servo tubes open "
                     "straight into it"),
            "x_mm": [round(xs[0], 1), round(xs[-1], 1)],
            "width_start_mm": round(widths[0], 1),
            "width_end_mm": round(widths[-1], 1),
            "wall_mm": round(wall_e, 2),
            "beyond_bay_mm": round(xs[-1] - x1, 1),
            "approx_added_volume_cm3": round(vol_cm3),
            "reaches_grommets": bool(reached),
        })
        if not reached:
            info["note"] = (f"extension ends at x={xs[-1]:.0f}, short "
                            f"of the grommet station "
                            f"{x_need - 12.0:.0f} - the runs will "
                            "report their own verdict")

        def a_at(xq: float) -> float:
            if xq <= xs[0]:
                return 0.5 * widths[0]
            if xq >= xs[-1]:
                return 0.5 * widths[-1]
            for i in range(len(xs) - 1):
                if xs[i] <= xq <= xs[i + 1]:
                    t = (xq - xs[i]) / max(xs[i + 1] - xs[i], 1e-9)
                    return 0.5 * (widths[i]
                                  + t * (widths[i + 1] - widths[i]))
            return 0.5 * widths[-1]

        def band_at(xq: float, yq: float) -> tuple[float, float]:
            f = _clamp(yq / max(wing.half, 1e-6), -0.995, 0.995)
            xcq = _clamp(float(wing.xc_at(f, xq)), 0.0, 1.0)
            return (float(wing.keel_z(f, xcq)) + wall_e + 1.0,
                    float(wing.crown_z(f, xcq)) - wall_e - 1.0)

        override = {"x1_mm": xs[-1] - 2.0, "a_at": a_at,
                    "band_at": band_at}
        return cutter, info, override
    except Exception as exc:
        info.update(ok=False, reason=str(exc))
        return None, info, None


def _servo_run(_cd, wing, bay, wall, body, bay_mm, hinge_xc: float = 0.72,
               span_mm: tuple[float, float] = (0.0, 1e9)):
    """ONE straight pipe carrying a servo lead inboard to the equipment bay.

    Builder's spec (rounds 5 and 6): every wire run is a single straight
    round tube of CONSTANT bore - a straight rod pushed in at the pocket
    mouth must come out inside the bay. Round 6 hardened the entry: the
    tube meets the bay's side wall at exactly 90 degrees - constant x AND
    constant z, level, zero angle. When the carved void floor sits above
    the grommet's height a floor WELL is sunk from the void to the tube's
    mouth (bay air, not a bend), mirroring the conventional's remedy.
    There is NO oblique fallback: a pocket whose level perpendicular line
    cannot stay inside the skin refuses honestly - never bends, never
    tilts.
    """
    try:
        bm = bay_mm or {}
        if bm.get("x0_mm") is not None and bm.get("x1_mm") is not None:
            x0, x1 = float(bm["x0_mm"]), float(bm["x1_mm"])
        else:
            x0 = float(body.get("bay_start_m", 0.0)) * MM
            x1 = x0 + float(body.get("bay_length_m", 0.0)) * MM
        w_bay = float(bm.get("width_mm")
                      or float(body.get("bay_width_m", 0.0)) * MM)
        start = bay.cable_exit
        y_in = max(0.5 * w_bay - 6.0, 2.0)          # 6 mm inside the void
        y_end = math.copysign(y_in, start.y or 1.0)
        zf, zc = bm.get("z_floor_aft_mm"), bm.get("z_ceil_aft_mm")
        r_m = 0.5 * 8.25 + 0.6                       # bore + a hair of air
        if zf is not None and zc is not None \
                and float(zc) - float(zf) > 2.0 * r_m:
            void = (float(zf), float(zc))
        else:
            # no measured band: fall back to the section mid at the wall
            f_wall = _clamp((0.5 * w_bay + wall) / max(wing.half, 1e-6),
                            0.0, 0.95)
            f_wall = math.copysign(f_wall, y_end)
            xcw = wing.xc_at(f_wall, 0.5 * (x0 + x1))
            zm = 0.5 * (wing.crown_z(f_wall, xcw)
                        + wing.keel_z(f_wall, xcw))
            void = (zm - 4.0 - r_m, zm + 4.0 + r_m)
        # root principle: the bay publishes its per-station cavity band
        # (the extension tapers, so the aft bulkhead's numbers are wrong
        # for a mid-cavity grommet). Aim at the REAL void at the tube's
        # own station.
        chan = False
        cs = bm.get("cavity_stations_mm")
        if cs and len(cs) >= 2:
            xs_c = [float(r[0]) for r in cs]
            if xs_c[0] + 4.0 <= start.x <= xs_c[-1] - 6.0:
                hw_x = zl_x = zh_x = None
                for i in range(len(xs_c) - 1):
                    if xs_c[i] <= start.x <= xs_c[i + 1]:
                        t = ((start.x - xs_c[i])
                             / max(xs_c[i + 1] - xs_c[i], 1e-9))
                        hw_x = (float(cs[i][1])
                                + t * (float(cs[i + 1][1])
                                       - float(cs[i][1])))
                        zl_x = (float(cs[i][2])
                                + t * (float(cs[i + 1][2])
                                       - float(cs[i][2])))
                        zh_x = (float(cs[i][3])
                                + t * (float(cs[i + 1][3])
                                       - float(cs[i][3])))
                        break
                if hw_x is not None and zh_x - zl_x > 2.0 * r_m + 2.0 \
                        and hw_x > r_m + 4.0:
                    chan = True
                    x1 = float(xs_c[-1])
                    void = (zl_x + 1.0, zh_x - 1.0)
                    y_end = math.copysign(min(y_in, hw_x - r_m - 2.0),
                                          start.y or 1.0)
        guard = {"hinge_xc": float(hinge_xc), "margin_xc": 0.04,
                 "span_lo_mm": float(span_mm[0]),
                 "span_hi_mm": float(span_mm[1])}
        base = {"hinge_guard": guard, "start_overshoot_mm": 6.0,
                "end_overshoot_mm": 8.0, "max_start_x_drift_mm": 2.2}

        # round 6 (builder): the tube is square to the bay's side wall in
        # plan - constant x - and LEVEL whenever the skin allows; the
        # oblique fallbacks are gone. When the void floor sits above the
        # grommet a floor WELL is sunk to the tube's own height (bay air,
        # not a bend), and when the grommet lies beyond the bay's x
        # footprint entirely (swept wings: the pocket chases the elevon
        # hinge aft) the void is EXTENDED along its side wall - a side
        # GALLERY, more hollowed-out bay - so the square entry still
        # opens into bay air. Only when no LEVEL line exists does the run
        # take the SHALLOWEST feasible slope, recorded and flagged. A
        # line that cannot stay inside the skin refuses honestly.
        in_x = x0 + 8.0 <= start.x <= x1 - 8.0
        need_gallery = not in_x
        GALLERY_MAX_MM = 140.0
        if need_gallery:
            reach = max(x0 + 8.0 - start.x, start.x - (x1 - 8.0))
            if reach > GALLERY_MAX_MM:
                return None, {"ok": False, "skipped": (
                    f"grommet x={start.x:.1f} is {reach:.0f} mm beyond "
                    f"the bay span [{x0:.1f}, {x1:.1f}] - a side gallery "
                    f"longer than {GALLERY_MAX_MM:.0f} mm would gut the "
                    "body")}

        cands: list[tuple] = []
        if need_gallery:
            cands.append(("perpendicular_gallery",
                          (start.z, start.z), "gallery"))
            cands.append(("gallery_min_slope",
                          (start.z - 12.0, start.z + 12.0), "gallery"))
        elif void[0] + r_m <= start.z <= void[1] - r_m:
            cands.append(("perpendicular", (start.z, start.z), None))
        elif not chan and void[0] - 30.0 <= start.z < void[0] + r_m:
            cands.append(("floor_well", (start.z, start.z), "well"))
        if not need_gallery:
            cands.append(("perpendicular_min_slope",
                          (void[0] + r_m, void[1] - r_m), None))
            if not chan and start.z < void[0] + r_m:
                cands.append(("floor_well_min_slope",
                              (void[0] - 30.0, void[0] + r_m), "well"))

        last = None
        for mode, zband, ext in cands:
            cutter, info = _cd.straight_conduit(
                [wing], start=start, end_xy=(start.x, y_end),
                end_z_band=zband, wall=wall,
                params=dict(base, prefer_z=start.z))
            info["kind"] = "servo"
            info["entry_mode"] = mode
            info["perpendicular"] = True   # square to the wall in plan
            slope = float(info.get("slope_deg") or 0.0)
            info["level"] = abs(slope) < 0.05
            if mode.endswith("min_slope") and cutter is not None \
                    and not info["level"]:
                info["level_note"] = (
                    "no LEVEL line fits inside the skin; shallowest "
                    f"feasible slope {slope:g} deg shipped instead")
            if cutter is None:
                last = info
                continue
            if ext == "well":
                z_e = float(info.get("end_z_mm", start.z))
                side_s = 1.0 if (y_end or 1.0) >= 0 else -1.0
                # round 7 (builder): a generous chamber, not a narrow
                # shaft - the tube's arrival merges into the bay hollow
                ya, yb = sorted((y_end - side_s * 18.0,
                                 y_end + side_s * 4.0))
                z_b = z_e - r_m - 1.0
                keel_hi, ok_floor = -1e9, True
                for xq in (start.x - 14.0, start.x + 14.0):
                    for yq in (ya, yb):
                        f = _clamp(yq / max(wing.half, 1e-6),
                                   -0.995, 0.995)
                        xcq = float(wing.xc_at(f, xq))
                        if not (0.0 <= xcq <= 1.0):
                            ok_floor = False
                            break
                        keel_hi = max(keel_hi,
                                      float(wing.keel_z(f, xcq)))
                if not ok_floor or z_b < keel_hi + max(wall, 3.0) + 0.3:
                    info["skipped"] = (
                        f"floor well bottom z={z_b:.1f} would thin the "
                        f"keel (max {keel_hi:.1f}) below {wall:.1f} mm")
                    last = info
                    continue
                well = Solid.makeBox(28.0, yb - ya,
                                     (void[0] + 4.0) - z_b,
                                     Vector(start.x - 14.0, ya, z_b))
                try:
                    cutter = cutter.fuse(well)
                except Exception as exc:
                    info["skipped"] = f"floor well fuse failed: {exc}"
                    last = info
                    continue
                info["bay_expansion"] = {
                    "ok": True, "kind": "floor_well",
                    "x_mm": [round(start.x - 14.0, 1),
                             round(start.x + 14.0, 1)],
                    "y_mm": [round(ya, 1), round(yb, 1)],
                    "z_mm": [round(z_b, 1), round(void[0] + 4.0, 1)],
                    "lowered_floor_by_mm": round(void[0] - z_b, 1),
                    "what": ("floor well sunk from the bay void to the "
                             "tube's mouth - the local enlargement that "
                             "keeps the run square to the wall")}
            elif ext == "gallery":
                z_e = float(info.get("end_z_mm", start.z))
                side_s = 1.0 if (y_end or 1.0) >= 0 else -1.0
                # round 7 (builder): a generous chamber, not a narrow
                # shaft - the tube's arrival merges into the bay hollow
                ya, yb = sorted((y_end - side_s * 18.0,
                                 y_end + side_s * 4.0))
                z_lo_g = min(start.z, z_e) - r_m - 1.0
                z_hi_g = max(max(start.z, z_e) + r_m + 1.0,
                             min(void[0] + 4.0, void[1] - 2.0))
                if start.x > x1 - 8.0:
                    xg_a, xg_b = x1 - 6.0, start.x + 10.0
                else:
                    xg_a, xg_b = start.x - 10.0, x0 + 6.0
                hx = float(base["hinge_guard"]["hinge_xc"]) - 0.04
                s_lo = float(base["hinge_guard"]["span_lo_mm"])
                s_hi = float(base["hinge_guard"]["span_hi_mm"])
                ok_g, why = True, ""
                for xq in (xg_a, 0.5 * (xg_a + xg_b), xg_b):
                    for yq in (ya, yb):
                        f = _clamp(yq / max(wing.half, 1e-6),
                                   -0.995, 0.995)
                        xcq = float(wing.xc_at(f, xq))
                        if not (0.0 <= xcq <= 1.0):
                            ok_g, why = False, (
                                f"leaves the plan at x={xq:.0f}, "
                                f"y={yq:.0f}")
                            break
                        if s_lo <= abs(yq) <= s_hi and xcq > hx:
                            ok_g, why = False, (
                                f"crosses the hinge corridor at "
                                f"y={yq:.0f} (xc={xcq:.2f})")
                            break
                        crown = float(wing.crown_z(f, xcq))
                        keel = float(wing.keel_z(f, xcq))
                        if z_hi_g > crown - wall - 0.3 or \
                                z_lo_g < keel + wall + 0.3:
                            ok_g, why = False, (
                                f"would thin the skin at x={xq:.0f}, "
                                f"y={yq:.0f} (band [{keel:.1f}, "
                                f"{crown:.1f}])")
                            break
                    if not ok_g:
                        break
                if not ok_g:
                    info["skipped"] = f"side gallery {why}"
                    last = info
                    continue
                gal = Solid.makeBox(xg_b - xg_a, yb - ya,
                                    z_hi_g - z_lo_g,
                                    Vector(xg_a, ya, z_lo_g))
                try:
                    cutter = cutter.fuse(gal)
                except Exception as exc:
                    info["skipped"] = f"side gallery fuse failed: {exc}"
                    last = info
                    continue
                info["bay_expansion"] = {
                    "ok": True, "kind": "side_gallery",
                    "x_mm": [round(xg_a, 1), round(xg_b, 1)],
                    "y_mm": [round(ya, 1), round(yb, 1)],
                    "z_mm": [round(z_lo_g, 1), round(z_hi_g, 1)],
                    "beyond_bay_mm": round(
                        max(0.0, xg_b - x1, x0 - xg_a), 1),
                    "what": ("bay extended along its side wall to meet "
                             "the square tube - the hollowed-out "
                             "enlargement that keeps the entry at 90 "
                             "degrees on a swept wing")}
            return cutter, info
        return None, (last or {"ok": False, "skipped": "no candidates"})
    except Exception as exc:
        return None, {"ok": False, "reason": str(exc)}


def _split_wing_panels(airframe: Solid, y_split: float,
                       reach: float) -> dict[str, Solid] | None:
    """Cut the full-span loft into `centre_body` + `wing_left` + `wing_right`.

    The aerodynamic surface is deliberately ONE continuous loft - that is what
    stops the aircraft looking like parts glued together, and the single-solid
    STL keeps it that way. But a CAD assembly has the opposite requirement: a
    builder opening the STEP in Fusion needs separable bodies to export, edit
    and print, and a plug-in wing joint is how a real wing of this size is
    transported anyway.

    So the parts export cuts the same surface at the spanwise station where the
    centre body blends out. The three pieces are coincident at the joint, so
    reassembling them reproduces the original surface exactly - nothing is
    approximated, only divided.
    """
    if y_split <= 1.0:
        return None
    big = 4.0 * reach
    try:
        right_box = Solid.makeBox(big, big, big,
                                  Vector(-0.5 * big, y_split, -0.5 * big))
        left_box = Solid.makeBox(big, big, big,
                                 Vector(-0.5 * big, -y_split - big, -0.5 * big))
        right = _heal(airframe.intersect(right_box))
        left = _heal(airframe.intersect(left_box))
        centre = _heal(airframe.cut(right_box).cut(left_box))
    except Exception:
        return None
    out = {"centre_body": centre, "wing_right": right, "wing_left": left}
    for solid in out.values():
        if not solid.isValid() or len(solid.Solids()) != 1 or solid.Volume() <= 0:
            return None
    return out


# v2/v3 type dispatch. This is ONE of the four seams where airplane_type is
# allowed to branch (ARCHITECTURE.md); both public entry points below resolve
# through it so they can never drift apart.
#   conventional -> cad.conventional
#   canard, tandem -> cad.multiwing  (fuselage + two lifting surfaces)
#   twin_boom -> cad.twinboom
#   delta, flying_wing -> the tailless path in THIS module, unmodified
#                         (V3_PLAN.md: a delta is a flying-wing dict whose
#                          planform is "delta")
# biplane and glider were removed from the registry (builder, 2026-08-21)
# and their rows here went with them - test_type_dispatch asserts this table
# and AIRPLANE_TYPES cannot drift apart in either direction.
_TYPE_MODULES = {
    "conventional": "conventional",
    "canard": "multiwing",
    "tandem": "multiwing",
    "twin_boom": "twinboom",
}


def _type_module(design: dict):
    """The sibling CAD module that owns this design, or None for the
    flying-wing/delta path built in this module."""
    import importlib

    name = _TYPE_MODULES.get(str(design.get("airplane_type", "flying_wing")))
    if name is None:
        return None
    return importlib.import_module(f".{name}", __package__)


def build_design_parts(design: dict) -> tuple[dict[str, Solid], dict[str, Any]]:
    """The aircraft as SEPARATE NAMED PARTS (mm), each in the world position
    it occupies on the assembled plane - drop them all into one CAD scene and
    you get the complete aeroplane.

    Names: `centre_body`, `wing_left` and `wing_right` (the full-span surface
    divided at the blend station - see `_split_wing_panels`), then any of
    `winglet_left` / `winglet_right` / `fin_left` / `fin_right` / `fin`, plus
    `hatch_lid` and `cg_marker`. Fin roots overlap the surface they stand on,
    which is exactly what an assembly of real bolted-on parts looks like.

    If the split cannot be made cleanly the whole surface is returned as a
    single `airframe` body rather than something broken.
    """
    mod = _type_module(design)
    if mod is not None:
        return mod.build_design_parts(design)
    parts, grooves, meta = _build_parts(design)
    _progress("split")
    out: dict[str, Solid] = {}
    for name, solid in parts.items():
        cutters = [c for target, c in grooves if target == name]
        out[name] = _apply_grooves(solid, cutters) if cutters else solid

    # Divide the surface into bodies a CAD user can actually separate.
    g = design["geometry"]
    body = g.get("body") or {}
    span_mm = float(g.get("span_m", 0.0)) * MM
    y_split = max(float(body.get("half_width_m", 0.0)) * MM * 1.6,
                  0.18 * 0.5 * span_mm)
    airframe = out.get("airframe")
    if airframe is not None and span_mm > 0:
        panels = _split_wing_panels(airframe, y_split, span_mm)
        if panels is not None:
            del out["airframe"]
            # keep the assembly ordered nose-outward for readability
            merged = dict(panels)
            merged.update(out)
            out = merged
    meta["valid_solid"] = all(bool(p.isValid()) for p in out.values())
    meta["part_names"] = list(out)
    return out, meta


def build_design_solid(design: dict) -> tuple[Solid, dict[str, Any]]:
    """Construct the full aircraft solid (mm) - ONE fused, healed, watertight
    body for STL / preview. Returns (solid, metadata).

    Same geometry as :func:`build_design_parts`: the named parts are unioned in
    build order, healed, and the elevon hinge grooves are then cut from the
    whole (cuts can re-break face trims, so `_heal` runs again after)."""
    mod = _type_module(design)
    if mod is not None:
        return mod.build_design_solid(design)
    parts, grooves, meta = _build_parts(design, separate_parts=False)
    _progress("fuse")
    solid = _fuse_all(list(parts.values()))

    if grooves:
        for _target, groove in grooves:
            try:
                cut = _heal(solid.cut(groove))
            except Exception:
                continue
            # A groove is a cosmetic scribe line. It is never worth keeping one
            # that leaves a face OCC will not tessellate: that shows up as a
            # HOLE in the exported skin, and `isValid()` does not catch it.
            if (len(cut.Solids()) == 1 and cut.isValid()
                    and _tessellates_cleanly(cut, tol=0.3, min_ratio=0.985)):
                solid = cut
        solid = _heal(solid)

    bb = solid.BoundingBox()
    meta["valid_solid"] = bool(solid.isValid())
    meta["bbox_mm"] = {"x": (bb.xmin, bb.xmax), "y": (bb.ymin, bb.ymax),
                       "z": (bb.zmin, bb.zmax)}
    meta["built_length_mm"] = bb.xmax - bb.xmin
    meta["built_span_mm"] = bb.ymax - bb.ymin
    meta["built_height_mm"] = bb.zmax - bb.zmin
    return solid, meta
