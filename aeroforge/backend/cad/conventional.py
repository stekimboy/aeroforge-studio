"""CAD builder for CONVENTIONAL designs (v2): wing + fuselage + tails.

Lives BESIDE the flying-wing path (V2_PLAN.md): `geometry.build_design_parts`
/ `build_design_solid` dispatch here when `design["airplane_type"] ==
"conventional"` and nothing in this module is imported on a flying-wing run.

Frame: x aft from the nose datum (mm), y right, z up, z = 0 on the thrust
line / fuselage mid-height (matches `physics.conventional`'s envelope math).
All CAD in mm.

TOPOLOGY. The fuselage is a smooth three-segment loft of superellipse
(rounded-rectangle) sections: a blunt nose cap that IS the firewall (the
tractor motor bolts straight to the flat front face - the solid nose is the
mount structure, so there is no separate nacelle to fair in), a constant bay
section, and a tail cone tapering to a near-knife vertical trailing edge with
the classic upswept belly and near-straight top deck. The wing is one
continuous tip-to-tip loft (`geometry._BlendedWing` with the body blends
turned off - straight LE, linear taper, smoothed dihedral), mounted high or
low per the design dict. The horizontal stab is the same kind of loft buried
in the tail post; the fin is a `_rounded_surface` panel rooted below the deck.
Everything fuses into ONE valid solid under the same `_heal` / mesh-gate
discipline as v1.

HARDWARE - all of it REUSED from the v1 modules, none re-implemented:
  * ailerons: cut free by `geometry._separate_elevons` (double bevel + cove +
    print-in-place captive-pin hinges via `hinges.py`), wing servo pockets via
    `servos.servo_bay`, horns via `servos.control_horn` (world-aligned to the
    arm plane), `servos.linkage` reports, and ONE STRAIGHT round trumpeted
    8.25 mm pipe per servo into the fuselage bay via
    `conduits.straight_conduit` (builder's spec, round 5) - perpendicular
    to the bay's side wall where the geometry allows, with a local bay
    enlargement (`_bay_expansion`) when that is what perpendicularity
    needs, straight-oblique as the last resort. A straight rod passes end
    to end; the old corridor + vertical riser is gone.
  * elevator: split into LEFT and RIGHT panels (the fin root and tail post own
    the centreline - a one-piece elevator would slice them), hinged the same
    way where the thin stab section allows (hinges.py refuses gracefully to
    bevel-only and says so). The RIGHT panel carries the horn; the builder
    joins the halves with a torsion wire, which is standard practice.
  * rudder: cut from the fin on a vertical hinge and hinged/horned IN A
    ROTATED FRAME - the hinge and horn machinery assumes an in-plan spanwise
    hinge line, so the airframe+rudder pair is rotated +90 deg about the x
    axis (world (x,y,z) -> build (x,-z,y)), built, and rotated back. The horn
    comes out on the LEFT side with a vertical bore, which is exactly what a
    rudder horn is.
  * elevator/rudder servos sit IN THE FUSELAGE BAY (v2 contract - no fuselage
    pockets this round): two straight round 8.25 mm trumpeted PIPES run from
    inside the measured bay void aft through the tail cone toward each horn,
    cut through `conduits.cut_conduits` (mesh-gated) and existence-checked
    with `conduits.route_is_open`.

EQUIPMENT BAY. `hatch.build_bay` is reused unchanged: the fuselage is
presented to it through the same duck-type protocol the wing satisfies
(`_FusBayHost`). On a low/mid wing the bay floor is RAISED over the wing
carry-through (keel guard = wing crown + margin, ramped in x) so hollowing
the fuselage can never sever the wing spar region; on a high wing the bay is
clamped to end ahead of the wing LE instead (the aperture opens through the
top deck, which the wing owns there). The lid is a separate part in the parts
build and stays attached (scribed) in the one-piece build, exactly like v1.

DOCTRINE (every rule inherited from DECISIONS.md, none relaxed):
  * every cut whose absence matters gets its own existence check - the bay
    void, every horn bore, every wire pipe, the motor screw holes;
  * measured, not assumed - pipe ends aim at the void the hatch actually
    carved (`bay_mm["x0_mm"]/["x1_mm"]` + a classified z band), never at the
    nominal envelope;
  * every boolean that matters is tessellation-gated;
  * nothing ahead of x = 0 (the flat firewall face sits at the motor-mount
    station, a few mm aft of the datum - the gap is the spinner clearance).
"""
from __future__ import annotations

import math
from typing import Any, Callable

import numpy as np
from cadquery import Edge, Solid, Vector, Wire

from . import conduits as _cd
from . import hatch as _hatch
from . import servos as _sv
from ..progress import report as _progress
from .geometry import (
    MM,
    _airfoil_pts,
    _apply_grooves,
    _BlendedWing,
    _blended_airframe,
    _cg_marker,
    _clamp,
    _clamp_aft,
    _elevon_grooves,
    _elevon_hinge_line,
    _foil_surf_t,
    _fuse_all,
    _heal,
    _jsonable,
    _max_thickness,
    _rounded_surface,
    _Section,
    _separate_elevons,
    _slab,
    _smoothstep,
    _tessellates_cleanly,
    _TIP_START,
    _TIP_U,
    _void_z_band,
    fuse_feature as _fuse_feature,
)

# The conventional wing sections (NACA 2412 etc.) are registered into the
# airfoil LIBRARY by the physics module. A design dict can arrive here without
# the physics ever having been imported in this process (a stored dict, a bare
# CAD test), and then `_airfoil_pts` would silently fall back to a REFLEXED
# analytic section - the exact opposite of what a tailed wing flies. Importing
# the physics module makes the registration unconditional.
try:  # pragma: no cover - only fails if physics is mid-rewrite
    from ..physics import conventional as _register_conv_sections  # noqa: F401
except Exception:
    pass

PIPE_D_MM = 8.25          # every wire run is one round pipe (builder's spec)
PIPE_FLARE = 1.6          # trumpeted mouths, same figure as conduits.py
SPINNER_NOTE = ("the flat nose face is the firewall; the gap between it and "
                "x = 0 is the spinner clearance")


# ---------------------------------------------------------------------------
# Fuselage: profiles, sections, loft
# ---------------------------------------------------------------------------

_SE_N = 2.6               # superellipse exponent: rounded-rectangle sections


class _FusProfile:
    """The fuselage's outer skin as three analytic profiles of x.

    a(x)   half width,  t(x) top,  b(x) bottom (centreline values, mm).
    Sections are superellipses (exponent `_SE_N`), so the body reads as the
    slab-sided rounded box every Cub / Apprentice silhouette shares
    (RESEARCH_CONVENTIONAL.md s.6/s.8). Nose cap and tail cone blend with
    smoothsteps so every segment boundary is C1 - the same rule the wing loft
    obeys (DECISIONS.md, "Segment boundaries must land where the section is
    C1").
    """

    def __init__(self, *, l_f: float, w: float, h: float, x_nose: float,
                 r_plate: float) -> None:
        self.l_f = float(l_f)
        self.x_nose = float(x_nose)
        self.W = 0.5 * float(w)
        self.T = 0.5 * float(h)
        self.B = -0.5 * float(h)
        # The firewall face must carry the motor plate with margin, but can
        # never be wider than the body it caps.
        self.a0 = _clamp(max(0.58 * self.W, r_plate + 3.0), 6.0, 0.92 * self.W)
        self.b0 = _clamp(max(0.55 * self.T, r_plate + 3.0), 6.0, 0.92 * self.T)
        self.x_full = x_nose + _clamp(0.15 * l_f, 40.0, 0.30 * l_f)
        # tail cone: starts aft of the constant bay section; the wing TE gets
        # its own guarantee in _build (the cone start is pushed aft of it)
        self.x_cone = _clamp(0.52 * l_f, 0.45 * l_f, 0.70 * l_f)
        # near-straight top deck, upswept belly, near-knife vertical TE
        self.post_top = self.T - 0.10 * (self.T - self.B)
        self.post_bot = self.post_top - 0.26 * (self.T - self.B)
        self.w_post = max(0.9, 0.010 * w)

    def set_cone_start(self, x: float) -> None:
        self.x_cone = _clamp(x, 0.45 * self.l_f, 0.72 * self.l_f)
        if self.x_cone < self.x_full + 10.0:
            self.x_cone = self.x_full + 10.0

    def blend(self, x: float) -> tuple[float, float, float]:
        """(half width, top z, bottom z) at station x, centreline."""
        x = _clamp(x, self.x_nose, self.l_f)
        if x <= self.x_full:
            s = _smoothstep((x - self.x_nose)
                            / max(self.x_full - self.x_nose, 1e-6))
            return (self.a0 + (self.W - self.a0) * s,
                    self.b0 + (self.T - self.b0) * s,
                    -self.b0 + (self.B + self.b0) * s)
        if x < self.x_cone:
            return self.W, self.T, self.B
        s = _smoothstep((x - self.x_cone) / max(self.l_f - self.x_cone, 1e-6))
        return (self.W + (self.w_post - self.W) * s,
                self.T + (self.post_top - self.T) * s,
                self.B + (self.post_bot - self.B) * s)

    def hw(self, x: float) -> float:
        return self.blend(x)[0]

    def top(self, x: float) -> float:
        return self.blend(x)[1]

    def _band(self, x: float, y: float) -> tuple[float, float]:
        """(crown, keel) of the superellipse section at world (x, y). Probes
        beyond the local half width are clamped to just inside the section
        edge (the `servos._skin` convention: edge value, never nonsense)."""
        a, t, b = self.blend(x)
        zc, bh = 0.5 * (t + b), 0.5 * (t - b)
        u = min(abs(y) / max(a, 1e-6), 0.985)
        f = (1.0 - u ** _SE_N) ** (1.0 / _SE_N)
        return zc + bh * f, zc - bh * f

    def crown(self, x: float, y: float) -> float:
        return self._band(x, y)[0]

    def keel(self, x: float, y: float) -> float:
        return self._band(x, y)[1]

    def contains_plan(self, x: float, y: float, inset: float = 0.8) -> bool:
        if not (self.x_nose + 1.0 <= x <= self.l_f - 1.0):
            return False
        return abs(y) <= self.hw(x) - inset

    def wire(self, x: float) -> Wire:
        """One closed superellipse section wire (a single spline - a polygon
        here would multiply the face count of every boolean downstream)."""
        a, t, b = self.blend(x)
        zc, bh = 0.5 * (t + b), max(0.5 * (t - b), 0.6)
        a = max(a, 0.6)
        e = 2.0 / _SE_N
        pts = []
        for th in np.linspace(0.0, 2.0 * math.pi, 49)[:-1]:
            cy, sz = math.cos(th), math.sin(th)
            pts.append(Vector(
                x,
                a * math.copysign(abs(cy) ** e, cy),
                zc + bh * math.copysign(abs(sz) ** e, sz)))
        return Wire.assembleEdges([Edge.makeSpline(pts + [pts[0]])])

    def solid(self) -> Solid:
        """The fuselage loft: three axial segments sharing boundary wires,
        evenly spaced stations inside each (the OCC through-sections rules
        from DECISIONS.md apply to this loft exactly as to the wing's)."""
        cache: dict[float, Wire] = {}

        def w(x: float) -> Wire:
            k = round(x, 6)
            if k not in cache:
                cache[k] = self.wire(x)
            return cache[k]

        segs = [
            np.linspace(self.x_nose, self.x_full, 7),
            np.linspace(self.x_full, self.x_cone, 3),
            np.linspace(self.x_cone, self.l_f, 9),
        ]
        solid: Solid | None = None
        for xs in segs:
            seg = Solid.makeLoft([w(float(x)) for x in xs], False)
            solid = seg if solid is None else solid.fuse(seg)
        return _heal(solid)


# ---------------------------------------------------------------------------
# Lifting surfaces positioned in world coordinates
# ---------------------------------------------------------------------------

class _ConvWing(_BlendedWing):
    """A plain tapered lifting surface (wing or stab) placed in the world.

    `_BlendedWing` with the body blends inert (depth_scale = chord_scale = 1)
    is exactly a straight-LE linearly tapered panel with smoothed dihedral and
    twist, a rounded moulded tip, and the full duck-type protocol every
    hardware module expects (`section` / `crown_z` / `keel_z` / `xc_at` /
    `half` / `fb` / `tc`). This subclass only positions it: `x_le_root` goes
    through the parent's own `x_offset`, and the vertical mount is applied in
    `section()`, the single source every other query derives from.
    """

    def __init__(self, *, x_le_root: float = 0.0, z_mount: float = 0.0,
                 **kw: Any) -> None:
        super().__init__(**kw)
        self._z_mount = float(z_mount)
        self.x_offset = float(x_le_root)

    def section(self, s: float) -> _Section:
        sec = super().section(s)
        return _Section(sec.chord, sec.twist,
                        Vector(sec.le.x, sec.le.y, sec.le.z + self._z_mount),
                        sec.t_scale)


class _PolyWing(_ConvWing):
    """POLYHEDRAL wing (v3 glider axis): the dihedral law becomes piecewise -
    `inner_deg` from the root out to `break_frac` of the semi-span,
    `outer_deg` beyond it - which is the researched thermal-ship rigging
    ("5 degrees from root to 3/5 of the semi-span, +3 from the joint to the
    tip", [RT3 row 44] via physics.glider.POLY_*).

    The parent is constructed with `dihedral_deg = 0` and this class adds the
    polyhedral z on top of `section()` - the single source every hardware
    query derives from, so crown/keel/hinge lines all see the breaks. The
    inner panel keeps the parent's smoothed-|y| centreline treatment (no
    crease down a moulded centre section); the break itself is a real C0
    joint and gets its own loft-segment boundary in `_poly_airframe`.
    """

    def __init__(self, *, poly: dict, **kw: Any) -> None:
        kw["dihedral_deg"] = 0.0
        super().__init__(**kw)
        self.poly = dict(poly)
        self.poly_break = _clamp(float(poly.get("break_frac", 0.60)),
                                 0.20, 0.90)
        self._tan_in = math.tan(math.radians(float(poly.get("inner_deg",
                                                            5.0))))
        self._tan_out = math.tan(math.radians(float(poly.get("outer_deg",
                                                             8.0))))

    def _poly_z(self, f: float) -> float:
        z = self.half * self._g(min(f, self.poly_break)) * self._tan_in
        if f > self.poly_break:
            z += self.half * (f - self.poly_break) * self._tan_out
        return z

    def section(self, s: float) -> _Section:
        sec = super().section(s)
        f = min(abs(s), 1.0)
        return _Section(sec.chord, sec.twist,
                        Vector(sec.le.x, sec.le.y,
                               sec.le.z + self._poly_z(f)),
                        sec.t_scale)


def _poly_airframe(wing: _PolyWing) -> Solid:
    """The full-span loft for a polyhedral wing. Same construction rules as
    `geometry._blended_airframe` (shared boundary wires, evenly spaced
    stations inside each segment) with ONE extra segment boundary per side at
    the polyhedral break - a C0 kink lofted through mid-segment would be
    smoothed into a bulge (the OCC through-sections lesson, DECISIONS.md)."""
    cache: dict[float, Any] = {}

    def wire(s: float):
        key = round(s, 9)
        if key not in cache:
            cache[key] = wing.wire(s)
        return cache[key]

    fbp, brk = wing.fb_p, wing.poly_break
    brk = _clamp(brk, fbp + 0.05, _TIP_START - 0.05)
    right = [
        [fbp + (brk - fbp) * k / 4.0 for k in range(5)],
        [brk + (_TIP_START - brk) * k / 3.0 for k in range(4)],
        [_TIP_START] + [_TIP_START + (1.0 - _TIP_START) * u for u in _TIP_U],
    ]
    left = [[-s for s in reversed(grp)] for grp in right]
    half = [fbp * k / 11.0 for k in range(12)]
    centre = [-s for s in reversed(half)] + half[1:]
    groups = list(reversed(left)) + [centre] + right
    solid: Solid | None = None
    for grp in groups:
        seg = Solid.makeLoft([wire(float(s)) for s in grp], False)
        solid = seg if solid is None else solid.fuse(seg)
    return _heal(solid)


class _FusBayHost:
    """The FUSELAGE presented through the wing duck-type protocol, so
    `hatch.build_bay`, `geometry._void_z_band` and the motor-hole probes can
    treat it exactly like the flying-wing centre body.

    Mapping: span fraction f -> y = f * half (half is the aircraft's real
    semi-span, so the fuselage lives at small |f|); "chord" is the fuselage
    length, so xc_at/x round-trip exactly.

    `keel_guard(x, y)` (optional) RAISES the reported keel - it is how the bay
    floor is kept above the wing carry-through on a low/mid wing. The guard is
    part of the honest survey: the hatch believes the section is shallower
    there and sizes the compartment accordingly, instead of a cut being
    clamped after the fact.
    """

    def __init__(self, fus: _FusProfile, half: float,
                 keel_guard: Callable[[float, float], float] | None = None
                 ) -> None:
        self.fus = fus
        self.half = float(half)
        self.fb = fus.W / max(self.half, 1e-6)
        self._chord = fus.l_f - fus.x_nose
        self.tc = (fus.T - fus.B) / max(self._chord, 1e-6)
        self._guard = keel_guard

    def section(self, f: float) -> _Section:
        return _Section(self._chord, 0.0,
                        Vector(self.fus.x_nose, f * self.half, 0.0), 1.0)

    def xc_at(self, f: float, x_mm: float) -> float:
        return _clamp((x_mm - self.fus.x_nose) / max(self._chord, 1e-6),
                      0.0, 1.0)

    def _xy(self, f: float, xc: float) -> tuple[float, float]:
        return (self.fus.x_nose + _clamp(xc, 0.0, 1.0) * self._chord,
                f * self.half)

    def crown_z(self, f: float, xc: float) -> float:
        x, y = self._xy(f, xc)
        return self.fus.crown(x, y)

    def keel_z(self, f: float, xc: float) -> float:
        x, y = self._xy(f, xc)
        k = self.fus.keel(x, y)
        if self._guard is not None:
            g = self._guard(x, y)
            if g > k:
                k = g
        return k


# ---------------------------------------------------------------------------
# Motor mount: the solid nose IS the structure - drill it and prove it
# ---------------------------------------------------------------------------

def _mount_cutters(spec: dict) -> tuple[list[tuple[Solid, Vector]], dict]:
    """Screw-hole and shaft-bore cutters for the tractor nose, each paired
    with a probe point that must classify as AIR after the cut (the same
    bore-existence doctrine as the horn keyholes - a boolean can fail without
    failing)."""
    if not spec:
        return [], {}
    x_face = float(spec.get("x_m", 0.005)) * MM
    y_c = float(spec.get("y_m", 0.0)) * MM
    z_c = float(spec.get("z_m", 0.0)) * MM
    r_bolt = float(spec.get("bolt_circle_radius_mm", 13.4))
    d_screw = float(spec.get("screw_hole_d_mm", 3.2))
    d_shaft = float(spec.get("shaft_hole_d_mm", 8.0))
    t_plate = float(spec.get("plate_thickness_mm", 4.0))
    n = max(int(spec.get("n_screws", 4)), 2)

    lead = 6.0
    screw_depth = _clamp(3.0 * t_plate, 9.0, 16.0)
    if n == 4:  # square pattern: holes on the corners, diagonal = bolt circle
        s = r_bolt / math.sqrt(2.0)
        offsets = [(s, s), (s, -s), (-s, s), (-s, -s)]
    else:
        offsets = [(r_bolt * math.cos(2 * math.pi * i / n),
                    r_bolt * math.sin(2 * math.pi * i / n)) for i in range(n)]
    axis = Vector(1, 0, 0)
    cutters: list[tuple[Solid, Vector]] = []
    for dy, dz in offsets:
        cutters.append((
            Solid.makeCylinder(0.5 * d_screw, lead + screw_depth,
                               Vector(x_face - lead, y_c + dy, z_c + dz), axis),
            Vector(x_face + 1.5, y_c + dy, z_c + dz)))
    shaft_depth = _clamp(2.0 * t_plate, 8.0, 15.0)
    cutters.append((
        Solid.makeCylinder(0.5 * d_shaft, lead + shaft_depth,
                           Vector(x_face - lead, y_c, z_c), axis),
        Vector(x_face + 1.5, y_c, z_c)))
    info = {"x_face_mm": x_face, "n_screws": n,
            "screw_depth_mm": round(screw_depth, 1),
            "shaft_depth_mm": round(shaft_depth, 1),
            "note": SPINNER_NOTE}
    return cutters, info


def _is_air(solid: Solid, p: Vector) -> bool:
    """Strict point-in-air classification (OUT only)."""
    try:
        from OCP.BRepClass3d import BRepClass3d_SolidClassifier
        from OCP.gp import gp_Pnt
        from OCP.TopAbs import TopAbs_OUT
        c = BRepClass3d_SolidClassifier(solid.wrapped)
        c.Perform(gp_Pnt(p.x, p.y, p.z), 1e-6)
        return c.State() == TopAbs_OUT
    except Exception:
        return False


# ---------------------------------------------------------------------------
# The rudder: split, hinge and horn in a rotated frame
# ---------------------------------------------------------------------------

# Fin root trailing edge kept ahead of the tail cone's end cap (mm): an edge
# lying ON the end face makes OCC's fuse return two disconnected solids and
# the fin goes MISSING (see the fin placement in _build).
_FIN_TE_INSET_MM = 2.5

_RUD_ROT_DEG = 90.0        # world (x, y, z) -> build (x, -z, y)


def _rot_pt(p: Vector, deg: float) -> Vector:
    a = math.radians(deg)
    return Vector(p.x, p.y * math.cos(a) - p.z * math.sin(a),
                  p.y * math.sin(a) + p.z * math.cos(a))


def _split_rudder(airframe: Solid, fin_station, span_fin: float,
                  z_lo: float, z_hi: float, rudder_frac: float,
                  z_fin_root: float, t_frac: float, *,
                  y_fin: float = 0.0, y_guard: float | None = None
                  ) -> tuple[Solid, Solid | None, dict]:
    """Cut the rudder free of the fin (vertical raked hinge), hang it on
    print-in-place hinges and fuse its horn - all in a frame rotated +90 deg
    about x, because the hinge/horn machinery reads "spanwise" off the plan
    projection of the hinge line and a vertical line has none.

    Returns (airframe, rudder | None, report). The horn lands on the LEFT
    side (-y) with a vertical bore, which is what a rudder horn is; the
    matching pushrod pipe exits the left of the tail cone.

    v3 additive parameters (defaults reproduce v2 byte-identically - with
    `y_fin=0.0` no translate is performed and no extra boolean runs):
      * `y_fin` - the fin's spanwise station. A twin-boom fin stands on a
        boom at y = +-y_boom, not on the centreline; the airframe+rudder
        pair is translated to y=0 before the rotate-hinge-horn dance and
        translated back after, so the machinery sees exactly the frame the
        centreline rudder proved out.
      * `y_guard` - half-width of a y band around `y_fin` the freeing cut is
        confined to. The v2 pocket is unbounded in y (fine for one centre
        fin); with two fins the unbounded pocket would carve BOTH, so the
        twin-boom passes a guard that protects the far fin.
    """
    from . import hinges as _h

    rep: dict[str, Any] = {"ok": False}
    xc_h = _clamp(1.0 - float(rudder_frac), 0.40, 0.85)

    def at_z(z: float) -> tuple[float, float]:
        f = _clamp((z - z_fin_root) / max(span_fin, 1e-6), 0.0, 1.0)
        c, _tw, le = fin_station(f)
        return le.x + xc_h * c, c

    x_lo, c_lo = at_z(z_lo)
    x_hi, c_hi = at_z(z_hi)
    p_lo, p_hi = Vector(x_lo, y_fin, z_lo), Vector(x_hi, y_fin, z_hi)
    d = p_hi - p_lo
    span = d.Length
    if span < 25.0:
        rep["reason"] = f"rudder hinge span {span:.0f} mm is too short"
        return airframe, None, rep
    u = d.multiply(1.0 / span)
    aft_n = Vector(u.z, 0.0, -u.x)          # perpendicular in x-z, aft-facing
    if aft_n.x < 0:
        aft_n = aft_n.multiply(-1.0)
    reach = 4.0 * span

    try:
        span_box = _slab(p_lo, u, 0.0, span, reach)
        pocket = _slab(p_lo, aft_n, 0.0, reach, reach).intersect(span_box)
        if y_guard is not None:
            band = Solid.makeBox(4.0 * reach, 2.0 * float(y_guard),
                                 4.0 * reach,
                                 Vector(-2.0 * reach,
                                        y_fin - float(y_guard),
                                        -2.0 * reach))
            pocket = pocket.intersect(band)
        rudder = _heal(airframe.intersect(pocket))
        keep = [s for s in rudder.Solids() if s.Volume() > 50.0]
        if not keep:
            rep["reason"] = "rudder split produced no material"
            return airframe, None, rep
        rudder = max(keep, key=lambda s: s.Volume())
        trimmed = _heal(airframe.cut(pocket))
        if not trimmed.isValid() or len(trimmed.Solids()) != 1:
            rep["reason"] = "freeing the rudder broke the airframe"
            return airframe, None, rep
    except Exception as exc:
        rep["reason"] = f"rudder split failed: {exc}"
        return airframe, None, rep

    t_lo, t_hi = t_frac * c_lo, t_frac * c_hi

    # ---- rotate, hinge, horn, rotate back ---------------------------------
    # (an off-centre fin is first translated onto the centreline, so the
    # machinery sees exactly the frame the v2 centre rudder proved out)
    org, ax = Vector(0, 0, 0), Vector(1, 0, 0)
    p_lo0 = Vector(p_lo.x, 0.0, p_lo.z) if y_fin else p_lo
    p_hi0 = Vector(p_hi.x, 0.0, p_hi.z) if y_fin else p_hi
    try:
        if y_fin:
            trimmed = trimmed.translate(Vector(0.0, -y_fin, 0.0))
            rudder = rudder.translate(Vector(0.0, -y_fin, 0.0))
        r_air = trimmed.rotate(org, ax, _RUD_ROT_DEG)
        r_rud = rudder.rotate(org, ax, _RUD_ROT_DEG)
        rp_lo = _rot_pt(p_lo0, _RUD_ROT_DEG)
        rp_hi = _rot_pt(p_hi0, _RUD_ROT_DEG)
    except Exception as exc:
        rep["reason"] = f"rotation failed: {exc}"
        return airframe, None, rep

    try:
        r_air, r_rud, hinfo = _h.print_in_place_hinges(
            r_air, r_rud, rp_lo, rp_hi, t_lo, t_hi, n_hinges=2)
    except Exception as exc:                          # never lose the split
        hinfo = {"mode": "none", "warnings": [f"hinge module failed: {exc}"]}

    horn_info: dict = {}
    try:
        horned, horn_info = _sv.control_horn(
            r_rud, hinge_p_in=rp_lo, hinge_p_out=rp_hi,
            station_frac=0.12, thickness=t_lo + 0.12 * (t_hi - t_lo))
        if horned.isValid() and len(horned.Solids()) == 1:
            r_rud = horned
        else:
            horn_info = {"ok": False, "reason": "horn broke the rudder"}
    except Exception as exc:
        horn_info = {"ok": False, "reason": str(exc)}
    horn_info.setdefault(
        "frame", "rudder build frame: world (x,y,z) = build (x, z, -y); "
                 "the horn stands on the LEFT (-y) side, bore vertical")

    try:
        trimmed2 = r_air.rotate(org, ax, -_RUD_ROT_DEG)
        rudder2 = r_rud.rotate(org, ax, -_RUD_ROT_DEG)
        if y_fin:
            trimmed2 = trimmed2.translate(Vector(0.0, y_fin, 0.0))
            rudder2 = rudder2.translate(Vector(0.0, y_fin, 0.0))
    except Exception as exc:
        rep["reason"] = f"rotate-back failed: {exc}"
        return airframe, None, rep
    keep = [s for s in rudder2.Solids() if s.Volume() > 50.0]
    if not keep or not trimmed2.isValid() or len(trimmed2.Solids()) != 1:
        rep["reason"] = "hinged rudder did not survive as one solid"
        return airframe, None, rep

    rep.update(ok=True, hinges=hinfo, horn=horn_info,
               hinge_p_lo=p_lo, hinge_p_hi=p_hi,
               hinge_xc=xc_h, span_mm=round(span, 1))
    return trimmed2, max(keep, key=lambda s: s.Volume()), rep


# ---------------------------------------------------------------------------
# v3 glider hardware: spar channel + nose-ballast pocket ([RT3 s.6.2])
# ---------------------------------------------------------------------------

LEAD_DENSITY_KGM3 = 11340.0       # cast lead - the ballast the pocket holds
SPAR_XC = 0.32                    # spar station: ~max thickness of the section
SPAR_SOCKET_CLEARANCE_MM = 0.30   # printed socket over the tube (same figure
#                                   as the twin-boom's carbon-tube socket)


def _spar_channel(airframe: Solid, wing: _ConvWing, spar: dict,
                  wall: float) -> tuple[Solid, dict]:
    """Bore the round carbon-spar socket through the wing sections ([RT3
    row 43]: span > 1.4 m or AR > 10 puts the bending load in a carbon tube;
    the LW-PLA skin carries torsion only).

    One STRAIGHT channel per wing panel: a polyhedral wing kinks at its
    break, so the spar does too - that joint is where a real vee-joiner
    lives - and the two inner segments cross the centreline so their bores
    meet in a vee, which is the joiner pocket. The outer end walks inboard
    until the section swallows Ø + skin, and the achieved reach is recorded
    rather than a thinner channel being shipped. Every segment is
    existence-checked by classification at five stations - `isValid()` never
    proves a cut did its job (DECISIONS.md doctrine).
    """
    d = float(spar.get("d_root_mm", 8.0)) + SPAR_SOCKET_CLEARANCE_MM
    r = 0.5 * d
    info: dict[str, Any] = {"applied": False, "d_mm": round(d, 2),
                            "xc": SPAR_XC, "segments": []}

    def centre(s: float) -> Vector:
        sec = wing.section(s)
        x = sec.le.x + SPAR_XC * sec.chord
        xcl = wing.xc_at(s, x)
        return Vector(x, sec.le.y, 0.5 * (wing.crown_z(s, xcl)
                                          + wing.keel_z(s, xcl)))

    def depth(f: float) -> float:
        sec = wing.section(f)
        xcl = wing.xc_at(f, sec.le.x + SPAR_XC * sec.chord)
        return wing.crown_z(f, xcl) - wing.keel_z(f, xcl)

    need = d + 2.0 * max(wall, 1.0)
    f_out = 0.90
    while f_out > 0.20 and depth(f_out) < need:
        f_out -= 0.02
    if f_out <= 0.20:
        info["reason"] = (f"no station out to 20% semi-span is deep enough "
                          f"for the Ø{d:.1f} mm spar socket plus "
                          f"{max(wall, 1.0):.1f} mm of skin per face")
        return airframe, info

    brk = getattr(wing, "poly_break", None)
    if brk is not None and brk < f_out - 0.08:
        bounds = [(-0.02, float(brk)), (float(brk), f_out)]
    else:
        bounds = [(-0.02, f_out)]

    cutters: list[Solid] = []
    probes: list[Vector] = []
    for sgn in (1.0, -1.0):
        for f0, f1 in bounds:
            p0, p1 = centre(sgn * f0), centre(sgn * f1)
            axis = p1 - p0
            length = axis.Length
            if length < 20.0:
                continue
            cutters.append(Solid.makeCylinder(
                r, length, p0, axis.multiply(1.0 / length)))
            for t in (0.10, 0.30, 0.50, 0.70, 0.90):
                probes.append(p0 + axis.multiply(t))
            info["segments"].append({
                "side": "right" if sgn > 0 else "left",
                "f0": round(f0, 3), "f1": round(f1, 3),
                "p0_mm": [round(c, 1) for c in (p0.x, p0.y, p0.z)],
                "p1_mm": [round(c, 1) for c in (p1.x, p1.y, p1.z)]})
    if not cutters:
        info["reason"] = "spar segments degenerate"
        return airframe, info

    tool = cutters[0]
    for c in cutters[1:]:
        try:
            tool = tool.fuse(c)
        except Exception:
            info["reason"] = "spar cutter fuse failed"
            return airframe, info
    cut = None
    for attempt in range(2):
        try:
            base = airframe.copy() if attempt else airframe
            t2 = tool.copy() if attempt else tool
            trial = _heal(base.cut(t2))
            if (trial.isValid() and len(trial.Solids()) == 1
                    and _tessellates_cleanly(trial, tol=0.3,
                                             min_ratio=0.985)):
                cut = trial
                break
        except Exception:
            continue
    if cut is None:
        info["reason"] = "spar cut rejected (mesh gate / validity)"
        return airframe, info
    open_ = [bool(_is_air(cut, p)) for p in probes]
    if not all(open_):
        info["reason"] = (f"spar bore not fully open "
                          f"({open_.count(False)}/{len(open_)} probes solid)")
        info["probes"] = open_
        return airframe, info
    info.update(applied=True, f_out=round(f_out, 3),
                probes_open=len(open_),
                reach_mm=round(f_out * wing.half, 1))
    if brk is not None:
        info["note"] = ("polyhedral: the spar kinks at the break - fit a "
                       "vee joiner in the crossed centre pocket")
    return cut, info


def _ballast_pocket(airframe: Solid, fus: _FusProfile, bay_mm: dict,
                    provision_kg: float, wall: float
                    ) -> tuple[Solid, dict]:
    """A cylindrical LEAD pocket bored forward from the bay void's front
    wall into the solid nose ([RT3 s.6.2]: with `n_motors: 0` the motor's
    mass becomes nose ballast at the same station - the Gentle Lady
    conversion in reverse). Sized at cast-lead density for the recorded
    provision, opens INTO the bay so the builder can load and trim, and is
    existence-checked by classification like every cut whose absence
    matters."""
    info: dict[str, Any] = {"applied": False,
                            "provision_kg": round(float(provision_kg), 4)}
    bm = bay_mm or {}
    if bm.get("x0_mm") is None:
        info["reason"] = "no bay void to open the pocket into"
        return airframe, info
    x0 = float(bm["x0_mm"])
    # the void's z band at its FORWARD wall, measured on the built solid
    zk = fus.keel(x0 + 4.0, 0.0) + 1.0
    zc = fus.crown(x0 + 4.0, 0.0) - 1.0
    air = [float(z) for z in np.linspace(zk, zc, 40)
           if _is_air(airframe, Vector(x0 + 4.0, 0.0, float(z)))]
    if len(air) < 4:
        info["reason"] = "could not measure the void at its forward wall"
        return airframe, info
    z_lo, z_hi = min(air), max(air)

    vol_mm3 = float(provision_kg) / LEAD_DENSITY_KGM3 * 1e9
    x_min = fus.x_nose + 6.0
    r = min(11.0, 0.45 * (z_hi - z_lo))
    length = 0.0
    for _ in range(4):
        length = _clamp(vol_mm3 / max(math.pi * r * r, 1e-6),
                        8.0, x0 - x_min)
        x_far = x0 - length
        a_far, t_far, b_far = fus.blend(x_far)
        z_p = _clamp(0.5 * (z_lo + z_hi), b_far + r + wall + 1.0,
                     t_far - r - wall - 1.0)
        r_new = min(11.0, 0.45 * (z_hi - z_lo),
                    a_far - wall - 1.0,
                    t_far - z_p - wall - 1.0, z_p - b_far - wall - 1.0)
        if r_new <= 2.0:
            info["reason"] = (f"nose too slender for a lead pocket "
                             f"(radius would be {r_new:.1f} mm)")
            return airframe, info
        if abs(r_new - r) < 0.2:
            r = r_new
            break
        r = r_new
    capacity_kg = math.pi * r * r * length * LEAD_DENSITY_KGM3 / 1e9

    bore = Solid.makeCylinder(r, length + 8.0,
                              Vector(x0 - length, 0.0, z_p), Vector(1, 0, 0))
    cut = None
    for attempt in range(2):
        try:
            base = airframe.copy() if attempt else airframe
            tool = bore.copy() if attempt else bore
            trial = _heal(base.cut(tool))
            if (trial.isValid() and len(trial.Solids()) == 1
                    and _tessellates_cleanly(trial, tol=0.3,
                                             min_ratio=0.985)):
                cut = trial
                break
        except Exception:
            continue
    if cut is None:
        info["reason"] = "ballast pocket cut rejected (mesh gate / validity)"
        return airframe, info
    probe = Vector(x0 - 0.5 * length, 0.0, z_p)
    if not _is_air(cut, probe):
        info["reason"] = "ballast pocket bore did not verify open"
        return airframe, info
    info.update(applied=True, d_mm=round(2.0 * r, 1),
                length_mm=round(length, 1), z_mm=round(z_p, 1),
                x0_mm=round(x0 - length, 1), x1_mm=round(x0, 1),
                capacity_kg=round(capacity_kg, 4),
                density_kgm3=LEAD_DENSITY_KGM3)
    if capacity_kg < 0.98 * float(provision_kg):
        info["short_kg"] = round(float(provision_kg) - capacity_kg, 4)
        info["note"] = ("pocket holds less than the full provision - the "
                        "remainder rides in the bay against the front wall")
    return cut, info


# ---------------------------------------------------------------------------
# v3 tail-type axes: V-tail ruddervators + the T-tail fin pushrod run
# ---------------------------------------------------------------------------

def _split_ruddervator(airframe: Solid, panel_station, span_p: float,
                       frac: float, sgn: float, dihedral_deg: float,
                       t_frac: float) -> tuple[Solid, Solid | None, dict]:
    """Free one V-tail panel's RUDDERVATOR, hinge and horn it with the
    standard machinery - the `_split_rudder` pattern generalised from 90 deg
    to the panel's own dihedral: rotate the airframe+surface pair about x by
    -sgn*A so the panel lies flat, run the proven in-plan hinge/horn code,
    rotate back. The freeing pocket is bounded along the panel NORMAL so it
    cannot nick the opposite panel where the vee closes at the root."""
    from . import hinges as _h

    rep: dict[str, Any] = {"ok": False}
    A = float(dihedral_deg)
    ca, sa = math.cos(math.radians(A)), math.sin(math.radians(A))
    xc_h = _clamp(1.0 - float(frac), 0.40, 0.85)
    n = Vector(0.0, -sa, sgn * ca)              # panel normal (unit)

    def at_f(f: float) -> tuple[Vector, float]:
        c, _tw, le = panel_station(f)
        return Vector(le.x + xc_h * c, le.y, le.z), c

    p_lo, c_lo = at_f(0.14)
    p_hi, c_hi = at_f(0.88)
    d = p_hi - p_lo
    span = d.Length
    if span < 25.0:
        rep["reason"] = f"ruddervator hinge span {span:.0f} mm is too short"
        return airframe, None, rep
    u = d.multiply(1.0 / span)
    aft_n = u.cross(n)
    if aft_n.x < 0:
        aft_n = aft_n.multiply(-1.0)
    reach = 4.0 * span
    band = min(10.0, 0.8 * t_frac * c_lo + 6.0)

    try:
        span_box = _slab(p_lo, u, 0.0, span, reach)
        pocket = _slab(p_lo, aft_n, 0.0, reach, reach).intersect(span_box)
        pocket = pocket.intersect(_slab(p_lo, n, -band, band, reach))
        surf = _heal(airframe.intersect(pocket))
        keep = [s for s in surf.Solids() if s.Volume() > 50.0]
        if not keep:
            rep["reason"] = "ruddervator split produced no material"
            return airframe, None, rep
        surf = max(keep, key=lambda s: s.Volume())
        trimmed = _heal(airframe.cut(pocket))
        if not trimmed.isValid() or len(trimmed.Solids()) != 1:
            rep["reason"] = "freeing the ruddervator broke the airframe"
            return airframe, None, rep
    except Exception as exc:
        rep["reason"] = f"ruddervator split failed: {exc}"
        return airframe, None, rep

    t_lo, t_hi = t_frac * c_lo, t_frac * c_hi
    rot = -sgn * A                               # panel -> horizontal
    org, ax = Vector(0, 0, 0), Vector(1, 0, 0)
    try:
        r_air = trimmed.rotate(org, ax, rot)
        r_srf = surf.rotate(org, ax, rot)
        rp_lo = _rot_pt(p_lo, rot)
        rp_hi = _rot_pt(p_hi, rot)
    except Exception as exc:
        rep["reason"] = f"rotation failed: {exc}"
        return airframe, None, rep

    try:
        r_air, r_srf, hinfo = _h.print_in_place_hinges(
            r_air, r_srf, rp_lo, rp_hi, t_lo, t_hi, n_hinges=2)
    except Exception as exc:                     # never lose the split
        hinfo = {"mode": "none", "warnings": [f"hinge module failed: {exc}"]}

    horn_info: dict = {}
    try:
        horned, horn_info = _sv.control_horn(
            r_srf, hinge_p_in=rp_lo, hinge_p_out=rp_hi,
            station_frac=0.40, thickness=t_lo + 0.40 * (t_hi - t_lo))
        if horned.isValid() and len(horned.Solids()) == 1:
            r_srf = horned
        else:
            horn_info = {"ok": False, "reason": "horn broke the ruddervator"}
    except Exception as exc:
        horn_info = {"ok": False, "reason": str(exc)}
    horn_info.setdefault(
        "frame", f"ruddervator build frame: panel rotated {rot:+.0f} deg "
                 "about x onto the horizontal; the horn stands on the "
                 "panel's lower face")

    try:
        trimmed2 = r_air.rotate(org, ax, -rot)
        surf2 = r_srf.rotate(org, ax, -rot)
    except Exception as exc:
        rep["reason"] = f"rotate-back failed: {exc}"
        return airframe, None, rep
    keep = [s for s in surf2.Solids() if s.Volume() > 50.0]
    if not keep or not trimmed2.isValid() or len(trimmed2.Solids()) != 1:
        rep["reason"] = "hinged ruddervator did not survive as one solid"
        return airframe, None, rep

    rep.update(ok=True, hinges=hinfo, horn=horn_info,
               hinge_p_lo=p_lo, hinge_p_hi=p_hi,
               hinge_xc=xc_h, span_mm=round(span, 1),
               dihedral_deg=A)
    return trimmed2, max(keep, key=lambda s: s.Volume()), rep


def _t_tail_fin_run(airframe: Solid, fin_station, span_fin: float,
                    z_fin_root: float, z_stab: float, stab_half: float,
                    fus: _FusProfile, wall: float
                    ) -> tuple[Solid, dict]:
    """The T-tail elevator pushrod's vertical leg UP THE FIN ([RT3 s.9] /
    V3_PLAN.md: same Ø8.25 trumpeted pipe doctrine), with the honest
    consequence drawn: a thin symmetric fin is SHALLOWER than the pipe, so
    the run brings its own material - a fused FAIRING tube (pipe + a wall
    each side, swept along the same path) standing proud of both fin faces,
    which is exactly the pushrod fairing real T-tail gliders wear. The
    bottom mouth opens in free air at the fin base (the rod arrives there
    externally from the horizontal tail-cone pipe - the established 'rod
    continues externally' doctrine); the top mouth opens just below the
    stab, by the elevator horn. The bore is cut through the mesh-gated
    conduit machinery and classified open end to end."""
    info: dict[str, Any] = {"applied": False, "d_mm": PIPE_D_MM,
                            "trumpeted": True}
    f_lo = _clamp((fus.post_top + 2.0 - z_fin_root) / max(span_fin, 1e-6),
                  0.02, 0.40)
    f_hi = _clamp((z_stab - stab_half - 5.0 - z_fin_root)
                  / max(span_fin, 1e-6), f_lo + 0.25, 0.97)

    def pt(f: float) -> Vector:
        c, _tw, le = fin_station(f)
        return Vector(le.x + 0.30 * c, 0.0, le.z)

    B, T = pt(f_lo), pt(f_hi)
    u = T - B
    length = u.Length
    if length < 30.0:
        info["reason"] = f"fin run is only {length:.0f} mm - no room"
        return airframe, info
    u = u.multiply(1.0 / length)
    B_over = B - u.multiply(6.0)                 # mouth breaks the fairing
    T_over = T + u.multiply(2.0)

    n = 13
    path = [B_over + (T_over - B_over).multiply(i / (n - 1))
            for i in range(n)]
    pipe_sizes, fair_sizes = [], []
    wall_f = max(wall, 1.2)
    for i in range(n):
        t = i / (n - 1)
        fl = 1.0 + (PIPE_FLARE - 1.0) * max(
            _smoothstep((0.15 - t) / 0.15), _smoothstep((t - 0.85) / 0.15))
        pipe_sizes.append((PIPE_D_MM * fl, PIPE_D_MM * fl))
        df = PIPE_D_MM * fl + 2.0 * wall_f
        fair_sizes.append((df, df))
    try:
        fairing = _cd._sweep(path, fair_sizes, section="circle")
        cutter = _cd._sweep(path, pipe_sizes, section="circle")
    except Exception as exc:
        info["reason"] = f"fin-run loft failed: {exc}"
        return airframe, info
    if not (fairing.isValid() and cutter.isValid()):
        info["reason"] = "fin-run loft invalid"
        return airframe, info

    try:
        merged = _heal(airframe.fuse(fairing))
        if not (merged.isValid() and len(merged.Solids()) == 1):
            info["reason"] = "fairing did not fuse into one solid"
            return airframe, info
    except Exception as exc:
        info["reason"] = f"fairing fuse failed: {exc}"
        return airframe, info

    try:
        cut_air, cinfo = _cd.cut_conduits(merged, [("t_tail_run", cutter)])
        applied = bool((cinfo.get("t_tail_run") or {}).get("applied"))
    except Exception as exc:
        info["reason"] = f"fin-run cut failed: {exc}"
        return airframe, info
    if not applied:
        info["reason"] = "fin-run cut rejected by the mesh gate"
        info["detail"] = cinfo.get("t_tail_run")
        return airframe, info
    try:
        open_ = _cd.route_is_open(cut_air, path)
    except Exception as exc:
        open_ = {"open": False, "error": str(exc)}
    if not open_.get("open"):
        info["reason"] = "fin-run bore reported cut but is blocked"
        info["detail"] = open_
        return airframe, info
    info.update(applied=True, route_open=True,
                bottom_mm=[round(c, 1) for c in (B_over.x, B_over.y,
                                                 B_over.z)],
                top_mm=[round(c, 1) for c in (T_over.x, T_over.y, T_over.z)],
                length_mm=round(length, 1),
                fairing_d_mm=round(PIPE_D_MM + 2 * wall_f, 1),
                note=("rod arrives externally from the tail-cone pipe, "
                      "enters the fin fairing at the base mouth and exits "
                      "just below the stab by the elevator horn"))
    return cut_air, info


# ---------------------------------------------------------------------------
# Wire runs
# ---------------------------------------------------------------------------

def _straight_pipe(start: Vector, end: Vector, d: float = PIPE_D_MM,
                   n: int = 13) -> tuple[Solid | None, list[Vector], dict]:
    """One straight round pipe with trumpeted mouths (smoothstep flare to
    `PIPE_FLARE` x over the last 15% at each end), lofted with the same
    `conduits._sweep(section='circle')` every other wire run uses."""
    path = [start + (end - start).multiply(i / (n - 1)) for i in range(n)]
    sizes = []
    for i in range(n):
        t = i / (n - 1)
        fl = 1.0 + (PIPE_FLARE - 1.0) * max(
            _smoothstep((0.15 - t) / 0.15), _smoothstep((t - 0.85) / 0.15))
        sizes.append((d * fl, d * fl))
    info = {"d_mm": d, "trumpeted": True,
            "entry_mm": [round(c, 2) for c in (start.x, start.y, start.z)],
            "exit_mm": [round(c, 2) for c in (end.x, end.y, end.z)],
            "length_mm": round((end - start).Length, 1)}
    try:
        cutter = _cd._sweep(path, sizes, section="circle")
    except Exception as exc:
        return None, path, dict(info, skipped=f"pipe loft failed: {exc}")
    if not cutter.isValid() or len(cutter.Solids()) != 1:
        return None, path, dict(info, skipped="pipe lofted invalid")
    return cutter, path, info


def _bay_expansion(fus: _FusProfile, bay_mm: dict, x_at: float, side: float,
                   z_need_lo: float, z_need_hi: float,
                   wall: float) -> tuple[Solid | None, dict]:
    """A local enlargement of the carved bay void so a PERPENDICULAR wire
    tube can open into real air - the builder's own remedy (2026-08-24:
    "hollowing out part of the fuselage more to expand its size so
    perpendicularity is possible"). A narrow trench beside the side wall
    the tube pierces, overlapping the main void so the two are one air
    space; refused whenever it would thin the fuselage skin below `wall`.
    """
    bm = bay_mm or {}
    zf, zc = bm.get("z_floor_aft_mm"), bm.get("z_ceil_aft_mm")
    if zf is None or zc is None:
        return None, {"ok": False,
                      "reason": "no measured void band to expand"}
    zf, zc = float(zf), float(zc)
    x0, x1 = float(bm["x0_mm"]), float(bm["x1_mm"])
    w_bay = float(bm.get("width_mm", 0.0))
    xa = max(x0 + 2.0, x_at - 9.0)
    xb = min(x1 - 2.0, x_at + 9.0)
    if xb - xa < 10.0:
        return None, {"ok": False, "reason": "no chordwise room to expand"}
    y_out = max(0.5 * w_bay - 2.0, 4.0)          # just inside the side wall
    y_in = max(y_out - 22.0, 2.0)
    if z_need_hi > zc - 1.0:                      # grow the ceiling
        z_bot, z_top = zc - 4.0, z_need_hi
    elif z_need_lo < zf + 1.0:                    # grow the floor
        z_bot, z_top = z_need_lo, zf + 4.0
    else:
        return None, {"ok": False,
                      "reason": "void already reaches the tube"}
    for x in (xa, 0.5 * (xa + xb), xb):
        for y in (side * y_in, side * y_out):
            if not fus.contains_plan(x, y, 0.5):
                return None, {"ok": False, "reason": (
                    f"expansion leaves the fuselage plan at x={x:.0f}, "
                    f"y={y:.0f}")}
            crown, keel = float(fus.crown(x, y)), float(fus.keel(x, y))
            if z_top > crown - wall - 0.3 or z_bot < keel + wall + 0.3:
                return None, {"ok": False, "reason": (
                    f"expanding the void to z=[{z_bot:.1f}, {z_top:.1f}] "
                    f"would thin the skin below {wall:.1f} mm at "
                    f"x={x:.0f}, y={y:.0f}")}
    ya, yb = sorted((side * y_in, side * y_out))
    box = Solid.makeBox(xb - xa, yb - ya, z_top - z_bot,
                        Vector(xa, ya, z_bot))
    return box, {
        "ok": True, "x_mm": [round(xa, 1), round(xb, 1)],
        "y_mm": [round(ya, 1), round(yb, 1)],
        "z_mm": [round(z_bot, 1), round(z_top, 1)],
        "raised_ceiling_by_mm": round(max(0.0, z_top - zc), 1),
        "lowered_floor_by_mm": round(max(0.0, zf - z_bot), 1),
        "what": ("local bay enlargement beside the side wall so the "
                 "perpendicular wire tube opens into the void")}


def _aileron_run(wing: _ConvWing, fus: _FusProfile, bay: "_sv.ServoBay",
                 wall: float, bay_mm: dict, hinge_xc: float = 0.75,
                 span_mm: tuple[float, float] = (0.0, 1e9)
                 ) -> tuple[list[tuple[str, Solid]], Vector | None,
                            list["Vector"] | None, dict]:
    """ONE straight pipe carrying one aileron servo lead to the FUSELAGE
    bay.

    Builder's spec (rounds 5 and 6, 2026-08-24): every wire run between
    hollow bays is a single straight round tube of CONSTANT bore - a
    straight rod pushed in at the pocket mouth must come out inside the
    bay. Round 6 hardened the entry: the tube meets the bay's side wall
    at exactly 90 degrees - constant x AND constant z, level, zero angle.
    When the carved void does not reach the grommet's height the bay is
    locally ENLARGED so the perpendicular tube still opens into air (a
    side trench, or a floor WELL sunk to the tube's own height); there is
    NO oblique fallback any more - a wing whose level perpendicular line
    cannot stay inside the skin refuses honestly. The old corridor +
    vertical riser (a 90-degree turn the wire had to make) is gone.

    Returns (named cutters, void-probe point, centreline, info).
    """
    bm = bay_mm or {}
    if bm.get("x0_mm") is None or bm.get("x1_mm") is None:
        return [], None, None, {"ok": False, "reason": (
            "no bay void - nowhere for the lead to go")}
    x0, x1 = float(bm["x0_mm"]), float(bm["x1_mm"])
    w_bay = float(bm.get("width_mm", 0.0))
    if w_bay < 18.0:
        return [], None, None, {"ok": False,
                                "reason": f"bay is only {w_bay:.0f} mm wide"}
    start = bay.cable_exit
    side = 1.0 if (start.y or 1.0) >= 0 else -1.0
    y_end = side * _clamp(0.5 * w_bay - 8.0, 4.0, 26.0)
    zf, zc = bm.get("z_floor_aft_mm"), bm.get("z_ceil_aft_mm")
    r_m = 0.5 * PIPE_D_MM + 0.6
    have_band = (zf is not None and zc is not None
                 and float(zc) - float(zf) > 2.0 * r_m)
    if have_band:
        void = (float(zf), float(zc))
    else:
        x_mid = _clamp(start.x, x0 + 8.0, x1 - 8.0)
        zm = 0.5 * (fus.crown(x_mid, 0.0) + fus.keel(x_mid, 0.0))
        void = (zm - 4.0 - r_m, zm + 4.0 + r_m)
    guard = {"hinge_xc": float(hinge_xc), "margin_xc": 0.04,
             "span_lo_mm": float(span_mm[0]),
             "span_hi_mm": float(span_mm[1])}
    base = {"hinge_guard": guard, "start_overshoot_mm": 6.0,
            "end_overshoot_mm": 8.0, "max_start_x_drift_mm": 2.2}
    in_x = x0 + 8.0 <= start.x <= x1 - 8.0

    cands: list[tuple] = []
    expand_refused: dict | None = None
    if in_x:
        if void[0] + r_m <= start.z <= void[1] - r_m:
            cands.append(("perpendicular", (start.x, y_end),
                          (start.z, start.z), start.z, None))
        elif have_band:
            box, xinfo = _bay_expansion(fus, bm, start.x, side,
                                        start.z - r_m - 0.7,
                                        start.z + r_m + 0.7, wall)
            if box is not None:
                cands.append(("perpendicular_expanded", (start.x, y_end),
                              (start.z, start.z), start.z, (box, xinfo)))
            else:
                expand_refused = xinfo
        if have_band and void[0] - 30.0 <= start.z < void[0] + r_m \
                and start.z - r_m - 1.0 >= (float(fus.keel(start.x, 0.0))
                                            + max(wall, 3.0) + 0.3):
            # The void may sit wholly ABOVE the wing (low/mid wing: the
            # bay floor is raised over the carry-through), and then no
            # level line can enter it. The builder's own remedy: hollow
            # the bay DOWN locally. Round 6: the tube stays LEVEL at the
            # grommet's own height - zero angle - and the floor WELL is
            # sunk from the void to meet its mouth. The well is bay air,
            # so a straight rod pushed through the tube emerges inside
            # the compartment.
            cands.append(("floor_well", (start.x, y_end),
                          (start.z, start.z), start.z, "well"))
        # LAST RESORT, still square in plan (round 6): when no LEVEL line
        # exists - a low wing's dihedral drops the section toward the
        # root faster than the skin allows (measured: the level line was
        # refused at 1.20 mm standing on the trainer) - take the
        # SHALLOWEST feasible slope rather than shipping no wiring at
        # all. prefer_z is the grommet's own height and the router scans
        # closest-first, so the slope that ships is the minimum feasible
        # one; recorded in `level_note`, never silent.
        cands.append(("perpendicular_min_slope", (start.x, y_end),
                      (void[0] + r_m, void[1] - r_m), start.z, None))
        if have_band and start.z < void[0] + r_m:
            z_hi_w = void[0] + r_m
            z_lo_w = max(void[0] - 30.0,
                         float(fus.keel(start.x, 0.0))
                         + max(wall, 3.0) + r_m + 1.0)
            if z_hi_w > z_lo_w:
                cands.append(("floor_well_min_slope", (start.x, y_end),
                              (z_lo_w, z_hi_w), start.z, "well"))

    last: dict | None = None
    for mode, exy, zband, prefer, ext in cands:
        pipe, info = _cd.straight_conduit(
            [wing, fus], start=start, end_xy=exy, end_z_band=zband,
            wall=wall, params=dict(base, prefer_z=prefer))
        info["kind"] = "servo"
        info["shape"] = "straight"
        info["entry_mode"] = mode
        # round 6: every candidate is square to the wall in plan; LEVEL
        # whenever the skin allows, else the recorded minimum slope
        info["perpendicular"] = True
        _slope = float(info.get("slope_deg") or 0.0)
        info["level"] = abs(_slope) < 0.05
        if mode.endswith("min_slope") and not info["level"]:
            info["level_note"] = (
                "no LEVEL line fits inside the skin; shallowest feasible "
                f"slope {_slope:g} deg shipped instead")
        if expand_refused is not None:
            info["bay_expansion_refused"] = expand_refused
        if pipe is None:
            last = info
            continue
        cutters: list[tuple[str, Solid]] = []
        well_probe = None
        if ext == "well":
            zf_v = float(bm["z_floor_aft_mm"])
            z_e = float(info["end_z_mm"])
            x_w, y_w = exy
            # rounds 7-8 (builder): ONE generous full-width chamber that
            # is simply part of the hollow - not a per-side pocket, no
            # boxy right-angle shaft for the wire to negotiate
            yspan = abs(y_w) + 4.0
            ya, yb = -yspan, yspan
            z_b = z_e - r_m - 1.0
            ok_plan = all(fus.contains_plan(x, y, 0.5)
                          for x in (x_w - 14.0, x_w + 14.0)
                          for y in (ya, yb))
            if not ok_plan:
                info["skipped"] = "floor well would leave the fuselage plan"
                last = info
                continue
            well = Solid.makeBox(28.0, yb - ya, (zf_v + 4.0) - z_b,
                                 Vector(x_w - 14.0, ya, z_b))
            cutters.append(("baylift", well))
            info["bay_expansion"] = {
                "ok": True, "kind": "floor_well",
                "x_mm": [round(x_w - 14.0, 1), round(x_w + 14.0, 1)],
                "y_mm": [round(ya, 1), round(yb, 1)],
                "z_mm": [round(z_b, 1), round(zf_v + 4.0, 1)],
                "lowered_floor_by_mm": round(zf_v - z_b, 1),
                "what": ("vertical well hollowed down from the bay floor "
                         "to the tube mouth - the local enlargement that "
                         "makes a straight run possible on a low/mid "
                         "wing")}
            well_probe = Vector(x_w, y_w, zf_v - 2.0)
        elif ext is not None:
            cutters.append(("baylift", ext[0]))
            info["bay_expansion"] = ext[1]
        cutters.append(("pipe", pipe))
        path = _cd.path_vectors(info)
        u = path[-1] - path[0]
        length = u.Length
        u = u.multiply(1.0 / max(length, 1e-9))
        if well_probe is not None:
            # a point in the well's former material, opened only by the cut
            probe = well_probe
        else:
            # probe INSIDE the side wall's former material, on the
            # centreline - a point only the cut can have opened
            y_probe = side * (0.5 * w_bay + 1.5)
            if abs(u.y) > 1e-6:
                t_p = _clamp((y_probe - path[0].y) / u.y, 2.0, length - 2.0)
            else:
                t_p = length - 6.0
            probe = path[0] + u.multiply(t_p)
        return cutters, probe, path, info
    return [], None, None, (last or {"ok": False, "skipped": (
        "no perpendicular entry: the grommet lies outside the bay's x "
        "span" if not in_x else
        "no level perpendicular line reaches the void")})


# ---------------------------------------------------------------------------
# Servo / hardware installation
# ---------------------------------------------------------------------------

def _install_hardware(airframe: Solid, wing: _ConvWing, fus: _FusProfile,
                      surfaces: dict[str, Solid], ail: dict, wall: float,
                      bay_mm: dict, pushrod_aims: dict[str, dict],
                      separate_parts: bool,
                      servo_params: dict | None = None
                      ) -> tuple[Solid, dict[str, Solid], dict]:
    """Aileron servo pockets + horns + linkage, every wire pipe, and the two
    tail pushrod exit runs. Mirrors `geometry._install_servos`' structure:
    collect cutters, one boolean per family, mesh-gate once, verify existence.

    v3: `ail` may be EMPTY (a rudder/elevator glider has no ailerons - the
    wing gets no servo pockets at all, [RT3 s.6.2]); `servo_params` lets the
    caller constrain the pocket search (e.g. `xc_min` aft of a spar
    channel). v2 callers pass a non-empty `ail` and no params - unchanged.
    """
    report: dict = {"bays": {}, "horns": {}, "conduits": {}, "pushrods": {}}
    out: dict[str, Solid] = {}
    riser_probes: dict[str, tuple[Vector, list[Vector] | None]] = {}

    inner = _clamp(float(ail.get("inner_frac", 0.55)),
                   max(wing.fb, 0.10), 0.85)
    outer = _clamp(float(ail.get("outer_frac", 0.95)),
                   inner + 0.10, _TIP_START - 0.01)
    xc = _clamp(1.0 - float(ail.get("chord_frac", 0.25)), 0.45, 0.90)
    y_arm_frac = _clamp(inner + 0.10, wing.fb + 0.06, outer - 0.10)

    bay_cutters: list[tuple[str, Any]] = []
    conduit_cutters: list[tuple[str, Solid | None]] = []
    pipe_paths: dict[str, list[Vector]] = {}

    sides = (((1.0, "aileron_right"), (-1.0, "aileron_left"))
             if ail else ())
    for sgn, name in sides:
        surface = surfaces.get(name)
        try:
            p_in, p_out, t_in, t_out = _elevon_hinge_line(
                wing, sgn, inner, outer, xc)
        except Exception:
            continue
        x_hinge = 0.5 * (p_in.x + p_out.x)
        try:
            bay = _sv.servo_bay(wing, y_frac=y_arm_frac, x_hinge=x_hinge,
                                sgn=sgn, wall=wall,
                                arm_y_mm=sgn * y_arm_frac * wing.half,
                                params={"f_min": inner + 0.02,
                                        **(servo_params or {})})
        except Exception as exc:
            bay = None
            report["bays"][name] = {"ok": False, "reason": str(exc)}
        if bay is not None and bay.ok and bay.cutter is not None:
            bay_cutters.append((name, bay))
        elif bay is not None:
            report["bays"][name] = {"ok": False,
                                    "reason": bay.reason or "no room"}

        if surface is None:
            continue                       # one-piece build: nothing separate
        align_y = bay.arm_hole.y if (bay is not None and bay.ok) else None
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

    # ---- both servo pockets in ONE boolean, validated once -----------------
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
        for nm, b in bay_cutters:
            if ok:
                report["bays"][nm] = {
                    "ok": True, "y_frac": b.y_frac,
                    "moved_inboard": b.moved_inboard, **(b.dims or {})}
                if b.cable_exit is not None:
                    run_cutters, probe, run_path, i_srv = _aileron_run(
                        wing, fus, b, wall, bay_mm,
                        hinge_xc=xc, span_mm=(inner * wing.half,
                                              outer * wing.half))
                    for kind, c in run_cutters:
                        conduit_cutters.append((f"{kind}_{nm}", c))
                    if probe is not None:
                        riser_probes[nm] = (probe, run_path)
                    if i_srv:
                        report["conduits"][f"servo_{nm}"] = i_srv
            else:
                report["bays"][nm] = {
                    "ok": False,
                    "reason": "bay cut would not mesh; skin left intact"}

    # ---- the two tail pushrod exit runs ------------------------------------
    bm = bay_mm or {}
    if bm.get("x0_mm") is not None and bm.get("x1_mm") is not None:
        x0, x1 = float(bm["x0_mm"]), float(bm["x1_mm"])
        x_s = _clamp(x1 - 12.0, x0 + 6.0, x1 - 6.0)
        zf, zc_ = bm.get("z_floor_aft_mm"), bm.get("z_ceil_aft_mm")
        if zf is not None and zc_ is not None and float(zc_) - float(zf) > 12.0:
            z_s = float(zf) + _clamp(8.0, 4.0,
                                     0.5 * (float(zc_) - float(zf)))
        else:
            z_s = 0.5 * (fus.crown(x_s, 0.0) + fus.keel(x_s, 0.0))
        for key, aim in pushrod_aims.items():
            side = float(aim["side"])              # +1 right, -1 left
            start = Vector(x_s, side * 7.0, z_s)
            target = Vector(*aim["target"])
            x_stop = float(aim["x_stop"])
            t_end = _clamp((x_stop - start.x)
                           / max(target.x - start.x, 1e-6), 0.35, 1.0)
            end = start + (target - start).multiply(t_end)
            cutter, path, info = _straight_pipe(start, end)
            info.update(kind="pushrod", drives=key,
                        note=("straight guide pipe from the fuselage bay "
                              "through the tail cone; the rod continues "
                              "externally to the horn"),
                        target_mm=[round(target.x, 1), round(target.y, 1),
                                   round(target.z, 1)])
            conduit_cutters.append((f"pushrod_{key}", cutter))
            pipe_paths[f"pushrod_{key}"] = path
            report["pushrods"][key] = info
    else:
        for key in pushrod_aims:
            report["pushrods"][key] = {
                "ok": False, "applied": False,
                "reason": "no bay void - the tail servos have nowhere to sit"}

    # ---- cut everything at once, mesh-gated, then prove the pipes ----------
    if conduit_cutters:
        try:
            airframe, cinfo = _cd.cut_conduits(airframe, conduit_cutters)
            for k, v in cinfo.items():
                if k.startswith("pushrod_"):
                    key = k[len("pushrod_"):]
                    prev = report["pushrods"].get(key)
                    if isinstance(prev, dict) and isinstance(v, dict):
                        prev.update(v)
                    else:
                        report["pushrods"][key] = v
                elif k.startswith("pipe_") or k.startswith("baylift_"):
                    kind, nm2 = k.split("_", 1)
                    ci = report["conduits"].setdefault(f"servo_{nm2}", {})
                    if kind == "pipe" and isinstance(v, dict):
                        ci.update(v)
                    elif isinstance(v, dict):
                        be = ci.setdefault("bay_expansion", {})
                        be["applied"] = bool(v.get("applied"))
                        if not v.get("applied"):
                            be["why"] = v.get("skipped") or v.get("why")
                else:
                    prev = report["conduits"].get(k)
                    if isinstance(prev, dict) and isinstance(v, dict):
                        prev.update(v)
                    else:
                        report["conduits"][k] = v
        except Exception as exc:
            report["conduits"]["error"] = str(exc)
        # existence check on every wing lead: the straight bore must be
        # air through the wall it pierces AND open along its whole
        # centreline - a run that walks up to the compartment and never
        # enters is the documented failure this check exists for
        for nm, (probe, run_path) in riser_probes.items():
            ci = report["conduits"].get(f"servo_{nm}")
            if not isinstance(ci, dict):
                continue
            open_ = bool(ci.get("applied")) and _is_air(airframe, probe)
            if open_ and run_path:
                try:
                    ro = _cd.route_is_open(airframe, run_path)
                    open_ = bool(ro.get("open"))
                    if not open_:
                        ci["route_open_detail"] = ro
                except Exception as exc:
                    open_ = False
                    ci["route_open_detail"] = str(exc)
            ci["into_bay_open"] = bool(open_)
            ci["riser_open"] = bool(open_)      # legacy key, same truth
            ci["route_open"] = bool(open_)
            if not open_:
                ci["applied"] = False
                ci["why"] = ("straight run into the bay is blocked - the "
                             "lead walks up to the compartment and never "
                             "enters")
        # existence check on every pushrod run: the centreline must be air
        for k, path in pipe_paths.items():
            key = k[len("pushrod_"):]
            rep = report["pushrods"].get(key)
            if not isinstance(rep, dict) or not rep.get("applied"):
                continue
            try:
                open_ = _cd.route_is_open(airframe, path)
                rep["route_open"] = bool(open_.get("open"))
                if not open_.get("open"):
                    rep["route_open_detail"] = open_
                    rep["applied"] = False
                    rep["why"] = "cut reported applied but the bore is blocked"
            except Exception as exc:
                rep["route_open"] = False
                rep["route_open_detail"] = str(exc)

    return airframe, out, report


# ---------------------------------------------------------------------------
# Core build
# ---------------------------------------------------------------------------

def _make_hosts(g: dict) -> tuple[_FusProfile, _ConvWing, _ConvWing, dict]:
    """(fuselage profile, wing host, stab host, derived dims) from a design's
    `geometry` dict. One source of truth: the builder AND the tests both read
    hinge lines and skin bands off these hosts, so they cannot drift."""
    fusd = g.get("fuselage") or {}
    taild = g.get("tail") or {}

    wall = max(float(g.get("wall_mm", 1.2)), 0.6)
    span = float(g["span_m"]) * MM
    half = max(0.5 * span, 1.0)
    c_root = float(g["root_chord_m"]) * MM
    c_tip = float(g.get("tip_chord_m") or c_root * 0.7) * MM
    wing_pos = str(g.get("wing_position", "high"))
    l_f = float(fusd.get("length_m", 0.7 * span / MM)) * MM
    fw = float(fusd.get("width_m", 0.07)) * MM
    fh = float(fusd.get("height_m", 0.09)) * MM
    x_w = float(fusd.get("x_wing_le_m", 0.2)) * MM
    mount_spec = g.get("motor_mount") or {}
    r_plate = float(mount_spec.get("plate_radius_mm", 16.0))
    x_nose = _clamp(float(mount_spec.get("x_m", 0.005)) * MM, 2.0, 15.0)
    length_total = float(g.get("length_total_m", l_f / MM + 0.005)) * MM

    # z = 0 on the thrust line / fuselage mid-height; the wing mount mirrors
    # the physics envelope formula exactly (physics.conventional, envelope).
    # v3 wing-position axis: "mid" sits ON the thrust line ([RT3 s.10] - the
    # aerobatic neutral position); v2 dicts only ever carry high/low.
    if wing_pos == "high":
        wing_z = 0.5 * fh
    elif wing_pos == "mid":
        wing_z = 0.0
    else:
        wing_z = -0.35 * fh

    fus = _FusProfile(l_f=l_f, w=fw, h=fh, x_nose=x_nose, r_plate=r_plate)
    fus.set_cone_start(max(0.52 * l_f, x_w + c_root + 6.0))

    wing_kw = dict(
        coords=_airfoil_pts(g.get("airfoil", "NACA 2412")),
        half=half, c_root=c_root, c_tip=max(c_tip, 0.06 * c_root),
        sweep_deg=float(g.get("sweep_le_deg", 0.0)),
        dihedral_deg=float(g.get("dihedral_deg", 0.0)),
        incidence=float(g.get("root_incidence_deg", 0.0)),
        washout=float(g.get("washout_deg", 0.0)),
        fb=_clamp((0.5 * fw) / half, 0.04, 0.50),
        depth_scale=1.0, chord_scale=1.0, nose_round=0.6, crown_frac=0.62,
        x_le_root=x_w, z_mount=wing_z)
    polyd = g.get("polyhedral")
    if polyd:
        # v3 glider axis: the loft gains a dihedral break ([RT3 row 44])
        wing: _ConvWing = _PolyWing(poly=polyd, **wing_kw)
    else:
        wing = _ConvWing(**wing_kw)

    fin_af = g.get("fin_airfoil", "NACA 0008")
    span_h = float(taild.get("span_h_m", 0.3 * span / MM)) * MM
    c_root_h = float(taild.get("c_root_h_m", 0.1)) * MM
    tail_type = str(taild.get("type") or taild.get("arrangement")
                    or "conventional")
    z_stab = 0.5 * (fus.post_top + fus.post_bot)   # buried in the tail post
    x_le_h_in = float(taild.get("x_le_h_m", 0.8 * l_f / MM)) * MM
    t_dims: dict = {}
    if tail_type == "t_tail":
        # v3 tail axis: the stab rides ATOP the fin ([RT3 s.9]). Fin numbers
        # here are the SAME formulas `_build` uses for the conventional fin,
        # so host and build cannot drift; the stab is dropped so its crown
        # sits at the fin tip - the recorded height stays honest.
        h_v = float(taild.get("height_v_m", 0.1)) * MM
        c_root_v = float(taild.get("c_root_v_m", 0.09)) * MM
        c_tip_v = float(taild.get("c_tip_v_m", 0.06)) * MM
        x_le_v = float(taild.get("x_le_v_m", 0.85 * l_f / MM)) * MM
        # The physics puts the fin root TE exactly on the fuselage's aft end
        # (both derive from tail.x_te), and in this branch the fin fuses
        # into the BARE cone (the stab rides the fin tip, so nothing else
        # bridges the joint) - a coincident end face makes the boolean
        # return two disconnected solids (DECISIONS.md, the centre-fin /
        # twin-boom lesson). Inset the fin root TE 2.5 mm ahead of the cone
        # end so every meeting is a real intersection.
        x_le_v = min(x_le_v, l_f - c_root_v - _FIN_TE_INSET_MM)
        z_fin_tip = 0.5 * fh + h_v
        z_fin_root = fus.post_top - 12.0
        span_fin = max(z_fin_tip - z_fin_root, 0.6 * h_v)
        x_le_v2, sweep_v = _clamp_aft(x_le_v, c_root_v, span_fin, 30.0,
                                      c_tip_v / max(c_root_v, 1e-6),
                                      x_aft=length_total)
        stab_half = 0.5 * _max_thickness(_airfoil_pts(fin_af)) * c_root_h
        z_stab = z_fin_tip - max(0.9 * stab_half, 3.0)
        x_tip_le = (x_le_v2
                    + 0.88 * span_fin * math.tan(math.radians(sweep_v)))
        # the stab must actually straddle the fin tip chord
        x_le_h_in = _clamp(x_le_h_in, x_tip_le - 0.35 * c_root_h,
                           x_tip_le + 0.5 * c_tip_v)
        t_dims = dict(z_fin_root=z_fin_root, span_fin=span_fin,
                      x_le_v2=x_le_v2, sweep_v=sweep_v,
                      z_fin_tip=z_fin_tip, x_tip_le=x_tip_le,
                      stab_half=stab_half)
    stab = _ConvWing(
        coords=_airfoil_pts(fin_af),
        half=max(0.5 * span_h, 1.0),
        c_root=c_root_h,
        c_tip=float(taild.get("c_tip_h_m", 0.07)) * MM,
        sweep_deg=0.0, dihedral_deg=0.0,
        incidence=float(taild.get("incidence_h_deg", 0.0)),
        washout=0.0, fb=0.06, depth_scale=1.0, chord_scale=1.0,
        nose_round=0.6, crown_frac=0.62,
        x_le_root=x_le_h_in,
        z_mount=z_stab)

    # ---- THE TAIL POST IS AS WIDE AS THE SURFACES IT CARRIES ---------------
    # Audit finding (2026-08-21, the builder's "vertical stabilizers are
    # hanging off slightly from the rest of the body"): `w_post` was a pure
    # styling knife edge, max(0.9, 1% of body width) ~ 1 mm half-width, while
    # the fin and stab roots fly 8% sections on 60-90 mm chords - half
    # thicknesses up to ~2.8 mm. Every conventional design's tail roots
    # therefore poked 0.5-1.2 mm out of the cone's FLANKS: the root is buried
    # 12 mm DOWN, so the protruding part reads as a thin slab standing proud
    # of each side of the cone. The post now clears the thickest root it
    # carries by 1.0 mm of shoulder per side; the cone still tapers (capped
    # at 55% of the full body half-width) and the recorded envelope is
    # untouched because the post is always narrower than the body.
    t_over_c = _max_thickness(_airfoil_pts(fin_af))
    roots_mm = [float(taild.get("c_root_v_m", 0.09)) * MM]
    if tail_type == "conventional":
        roots_mm.append(c_root_h)          # stab straddles the post
    if tail_type == "v_tail":
        # the panel root chord, derived exactly as `_build`'s v_tail branch
        # derives it (dict values win; the fallback is the same formula)
        area_p = float(taild.get("area_panel_m2") or 0.5 * (
            float(taild.get("area_h_m2", 0.010))
            + float(taild.get("area_v_m2", 0.006)))) * MM * MM
        ar_p = float(taild.get("panel_ar", 3.2))
        taper_p = float(taild.get("panel_taper", 0.65))
        span_p = (float(taild.get("panel_span_m", 0.0)) * MM
                  or math.sqrt(ar_p * area_p))
        roots_mm.append(float(taild.get("c_root_panel_m", 0.0)) * MM
                        or 2.0 * area_p / (span_p * (1.0 + taper_p)))
    need_half = 0.5 * t_over_c * max(roots_mm) + 1.0
    fus.w_post = _clamp(max(fus.w_post, need_half), 0.9, 0.55 * fus.W)

    dims = dict(wall=wall, span=span, half=half, c_root=c_root,
                wing_pos=wing_pos, l_f=l_f, fw=fw, fh=fh, x_w=x_w,
                wing_te=x_w + c_root, x_nose=x_nose, wing_z=wing_z,
                z_stab=z_stab, span_h=span_h, fin_af=fin_af,
                mount_spec=mount_spec, length_total=length_total,
                tail_type=tail_type, t_tail=t_dims)
    return fus, wing, stab, dims


def _build(design: dict, separate_parts: bool = True,
           _probe: dict | None = None
           ) -> tuple[dict[str, Solid], list[tuple[str, Solid]],
                      dict[str, Any]]:
    g = design["geometry"]
    fusd = g.get("fuselage") or {}
    taild = g.get("tail") or {}
    aild = g.get("ailerons") or {}
    warnings: list[str] = []

    fus, wing, stab, dims = _make_hosts(g)
    wall, half, span = dims["wall"], dims["half"], dims["span"]
    c_root, wing_pos = dims["c_root"], dims["wing_pos"]
    l_f, fw, fh = dims["l_f"], dims["fw"], dims["fh"]
    x_w, wing_te, x_nose = dims["x_w"], dims["wing_te"], dims["x_nose"]
    wing_z, z_stab, span_h = dims["wing_z"], dims["z_stab"], dims["span_h"]
    fin_af, mount_spec = dims["fin_af"], dims["mount_spec"]
    length_total = dims["length_total"]
    # v3 additive branches, all keyed on dict content v2 dicts never carry
    tail_type = dims.get("tail_type", "conventional")
    has_ail = bool(aild)
    cfg = design.get("config") or {}
    n_motors = cfg.get("n_motors")
    if n_motors is None:
        n_motors = 1 if mount_spec else 0

    parts: dict[str, Solid] = {}
    grooves: list[tuple[str, Solid]] = []
    hinge_report: dict = {}
    servo_report: dict = {}

    _progress("loft")
    # ---- airframe: fuselage + wing + stab + fin, one solid ------------------
    airframe = fus.solid()

    def _fuse_in(base: Solid, add: Solid, what: str) -> Solid:
        """Verified feature fuse - see `geometry.fuse_feature`. A boolean can
        return one valid solid that does not contain the part it was asked to
        add (found on the v3 biplane's fin); the gate classifies rather than
        trusting the flags."""
        return _fuse_feature(base, add, what, warnings)[0]

    wing_loft = (_poly_airframe(wing) if isinstance(wing, _PolyWing)
                 else _blended_airframe(wing))
    airframe = _fuse_in(airframe, wing_loft, "wing")
    raw_tail: dict[str, Solid] = {}    # the loose tail solids (probe seam)
    if tail_type == "conventional":
        raw_tail["stab"] = _blended_airframe(stab)
        airframe = _fuse_in(airframe, raw_tail["stab"], "stabilizer")

    _progress("fins")
    # fin: rooted below the local deck, tip capped at the recorded height
    h_v = float(taild.get("height_v_m", 0.1)) * MM
    c_root_v = float(taild.get("c_root_v_m", 0.09)) * MM
    c_tip_v = float(taild.get("c_tip_v_m", 0.06)) * MM
    x_le_v = float(taild.get("x_le_v_m", 0.85 * l_f / MM)) * MM
    z_fin_tip = 0.5 * fh + h_v            # = the physics z_top (fin governs)
    z_fin_root = fus.post_top - 12.0      # buried into the cone / post
    span_fin = max(z_fin_tip - z_fin_root, 0.6 * h_v)
    # The fin root's trailing edge must NOT land on the tail cone's end cap.
    # `_clamp_aft` puts the root TE exactly at x_aft, and when the physics
    # asks for a fin further aft than the fuselage allows that is the cone
    # end itself: the root plane's aft edge then lies ON the end face and
    # OCC's fuse hands back two disconnected solids - measured on the
    # 700 x 1200 x 300 sport trainer (fin TE 668.00, cone end 668.03): the
    # "fin did not fuse" warning fired on EVERY conventional configuration
    # of that box, and the same fin shifted 0.1 mm either way fused first
    # time. The t-tail host already insets 2.5 mm for exactly this reason;
    # every tail type gets it (task 2, 2026-08-28).
    x_le_v = min(x_le_v, l_f - c_root_v - _FIN_TE_INSET_MM)
    x_le_v2, sweep_v = _clamp_aft(x_le_v, c_root_v, span_fin, 30.0,
                                  c_tip_v / max(c_root_v, 1e-6),
                                  x_aft=length_total)
    if tail_type == "t_tail":
        # the hosts inset the fin root TE ahead of the cone end (see
        # _make_hosts) - build from the same numbers so they cannot drift
        td_t = dims.get("t_tail") or {}
        x_le_v2 = float(td_t.get("x_le_v2", x_le_v2))
        sweep_v = float(td_t.get("sweep_v", sweep_v))
    vt: dict = {}
    panel_stations: dict[str, Any] = {}
    if tail_type == "v_tail":
        # v3 tail axis ([RT3 s.9] via config_axes.apply_tail_type): two
        # raked panels replace stab + fin. Panel dims come from the dict
        # where given, else are derived from the panel area at AR 3.2 /
        # taper 0.65 and recorded - the CAD's input contract for the axis.
        A_vt = _clamp(float(taild.get("dihedral_deg", 35.0)), 25.0, 45.0)
        area_p = float(taild.get("area_panel_m2") or 0.5 * (
            float(taild.get("area_h_m2", 0.010))
            + float(taild.get("area_v_m2", 0.006)))) * MM * MM
        ar_p = float(taild.get("panel_ar", 3.2))
        taper_p = float(taild.get("panel_taper", 0.65))
        span_p = (float(taild.get("panel_span_m", 0.0)) * MM
                  or math.sqrt(ar_p * area_p))
        c_root_p = (float(taild.get("c_root_panel_m", 0.0)) * MM
                    or 2.0 * area_p / (span_p * (1.0 + taper_p)))
        c_tip_p = taper_p * c_root_p
        x_le_p = float(taild.get("x_le_h_m", 0.8 * l_f / MM)) * MM
        z_root_p = fus.post_top - 8.0
        x_le_p2, sweep_p = _clamp_aft(x_le_p, c_root_p, span_p, 18.0,
                                      c_tip_p / max(c_root_p, 1e-6),
                                      x_aft=length_total)
        ca_v = math.cos(math.radians(A_vt))
        sa_v = math.sin(math.radians(A_vt))
        for sgn_p, nm_p in ((1.0, "right"), (-1.0, "left")):
            p_solid, p_station = _rounded_surface(
                airfoil=fin_af, span_mm=span_p, c_root_mm=c_root_p,
                c_tip_mm=c_tip_p,
                le_root=Vector(x_le_p2, 0.0, z_root_p),
                sweep_le_deg=sweep_p, dihedral_deg=0.0,
                twist_root_deg=0.0, twist_tip_deg=0.0,
                span_dir=Vector(0, sgn_p * ca_v, sa_v),
                tdir=Vector(0, -sa_v, sgn_p * ca_v))
            raw_tail[f"vtail_{nm_p}"] = p_solid
            airframe = _fuse_in(airframe, p_solid, f"{nm_p} v-tail panel")
            panel_stations[nm_p] = p_station
        fin_solid, fin_station = None, None
        vt = {"dihedral_deg": A_vt, "span_panel_mm": round(span_p, 1),
              "c_root_mm": round(c_root_p, 1), "c_tip_mm": round(c_tip_p, 1),
              "x_le_mm": round(x_le_p2, 1), "sweep_deg": round(sweep_p, 1),
              "z_root_mm": round(z_root_p, 1),
              "z_tip_mm": round(z_root_p + span_p * sa_v, 1)}
    else:
        fin_solid, fin_station = _rounded_surface(
            airfoil=fin_af, span_mm=span_fin, c_root_mm=c_root_v,
            c_tip_mm=c_tip_v, le_root=Vector(x_le_v2, 0.0, z_fin_root),
            sweep_le_deg=sweep_v, dihedral_deg=0.0,
            twist_root_deg=0.0, twist_tip_deg=0.0,
            span_dir=Vector(0, 0, 1), tdir=Vector(0, 1, 0))
        raw_tail["fin"] = fin_solid
        airframe = _fuse_in(airframe, fin_solid, "fin")
        if tail_type == "t_tail":
            # the stab fuses AFTER the fin it stands on (fused before, its
            # tips would hang in free air and the fuse would disconnect)
            raw_tail["stab"] = _blended_airframe(stab)
            airframe = _fuse_in(airframe, raw_tail["stab"],
                                "stabilizer (atop the fin)")

    # ---- v3 t_tail: the elevator pushrod's fin run (fairing + Ø8.25) ------
    t_tail_run: dict = {}
    if tail_type == "t_tail" and fin_station is not None:
        stab_half = float(dims.get("t_tail", {}).get("stab_half", 4.0))
        airframe, t_tail_run = _t_tail_fin_run(
            airframe, fin_station, span_fin, z_fin_root, z_stab,
            stab_half, fus, wall)
        if not t_tail_run.get("applied"):
            warnings.append("t-tail fin pushrod run not cut: "
                            + str(t_tail_run.get("reason")))

    _progress("bay")
    # ---- equipment bay + hatch ----------------------------------------------
    bayd = fusd.get("bay") or {}
    bay_start = float(bayd.get("bay_start_m", 0.08 * l_f / MM)) * MM
    bay_len = float(bayd.get("bay_length_m", 0.3 * l_f / MM)) * MM
    bay_hw = 0.5 * float(bayd.get("bay_width_m", (fw - 2.4) / MM)) * MM

    keel_guard: Callable[[float, float], float] | None = None
    bay_x_max: float | None = None
    if wing_pos == "high":
        # the wing owns the top deck over its chord - the aperture cannot open
        # there, so the bay stops ahead of the wing LE
        bay_x_max = x_w - 12.0
    else:
        # low/mid wing: the wing passes through the fuselage belly. The bay
        # floor is RAISED above the wing carry-through (crown + 2 mm), ramped
        # in x so the hatch survey sees a smooth honest section.
        def keel_guard(x: float, y: float,
                       _x0: float = x_w, _x1: float = wing_te) -> float:
            r_in = _smoothstep((x - (_x0 - 18.0)) / 18.0)
            r_out = _smoothstep(((_x1 + 18.0) - x) / 18.0)
            ramp = min(r_in, r_out)
            if ramp <= 0.0:
                return -1e9
            f = _clamp(y / half, -0.98, 0.98)
            xw = _clamp(x, _x0 + 0.5, _x1 - 0.5)
            protect = wing.crown_z(f, wing.xc_at(f, xw)) + 2.0
            base = fus.keel(x, y)
            return base + ramp * max(protect - base, 0.0)

    bay_host = _FusBayHost(fus, half, keel_guard)
    bay_mm: dict = {}
    lid: Solid | None = None
    try:
        bay = _hatch.build_bay(
            bay_host, bay_start=bay_start, bay_length=bay_len,
            bay_half_width=bay_hw, wall=wall, x_max=bay_x_max,
            airframe=airframe, magnets=True,
            canopy=separate_parts, one_piece=not separate_parts)
    except Exception as exc:
        bay = None
        warnings.append(f"bay module failed: {exc}")
    if bay is not None and bay.ok:
        built = bay.airframe if separate_parts else bay.airframe_onepiece
        if built is not None and built.isValid() and len(built.Solids()) == 1:
            airframe = built
            bay_mm = dict(bay.bay_mm or {})
            if separate_parts and bay.lid is not None:
                lid = bay.lid
            zb = _void_z_band(airframe, bay_host, bay_mm)
            if zb is not None:
                bay_mm["z_floor_aft_mm"], bay_mm["z_ceil_aft_mm"] = zb
    elif bay is not None:
        warnings.append("no equipment bay could be cut: "
                        + str((bay.bay_mm or {}).get("reason")))
    if _probe is not None:
        # probe seam (tools_probe_fin_intrusion.py): hand back the
        # pieces exactly as built and stop here - nothing downstream
        # adds material to a tail surface, it only cuts
        _probe.update(airframe=airframe, fins=raw_tail,
                      cavity=(bay.cavity if bay is not None and bay.ok
                              else None),
                      bay_mm=bay_mm, warnings=list(warnings))
        return {"airframe": airframe}, grooves, {"bay": bay_mm,
                                                 "probe": True}

    # ---- motor holes: drilled into the solid nose, each PROVEN open ---------
    cutters, mount_info = _mount_cutters(mount_spec)
    holes_open: list[bool] = []
    for cyl, probe in cutters:
        try:
            drilled = _heal(airframe.cut(cyl))
            if drilled.isValid() and len(drilled.Solids()) == 1:
                airframe = drilled
        except Exception:
            pass
        holes_open.append(_is_air(airframe, probe))
    if cutters:
        mount_info["holes_cut"] = holes_open
        if not all(holes_open):
            warnings.append(
                "one or more motor-mount bores did not verify open - drill "
                f"the {mount_spec.get('screw_hole_d_mm', 3.2)} mm holes by "
                "hand at the marked bolt circle")

    # ---- v3 glider: carbon-spar channel through the wing sections ----------
    spar_info: dict = {}
    if g.get("spar"):
        airframe, spar_info = _spar_channel(airframe, wing, g["spar"], wall)
        if not spar_info.get("applied"):
            warnings.append("spar channel not cut: "
                            + str(spar_info.get("reason")))

    # ---- v3 glider: nose-ballast pocket (pure glider, n_motors = 0) --------
    ballast_info: dict = {}
    provision = float(g.get("nose_ballast_provision_kg") or 0.0)
    if provision > 0.0 and int(n_motors) == 0:
        airframe, ballast_info = _ballast_pocket(airframe, fus, bay_mm,
                                                 provision, wall)
        if not ballast_info.get("applied"):
            warnings.append("nose-ballast pocket not cut: "
                            + str(ballast_info.get("reason")))

    _progress("hinges")
    # ---- control surfaces ----------------------------------------------------
    ail_in = float(aild.get("inner_frac", 0.55))
    ail_out = float(aild.get("outer_frac", 0.95))
    ail_c = float(aild.get("chord_frac", 0.25))
    elev_c = float(taild.get("elevator_chord_frac", 0.27))
    rud_c = float(taild.get("rudder_chord_frac", 0.45))
    rudder_rep: dict = {}

    ruddervators: dict[str, Solid] = {}
    rv_reps: dict[str, dict] = {}
    if separate_parts:
        # ailerons (outer wing) - absent on a rudder/elevator glider
        if has_ail:
            airframe, cut_surfs, rep = _separate_elevons(
                airframe, wing, ail_in, ail_out, ail_c)
            surfaces = {k.replace("elevon", "aileron"): v
                        for k, v in cut_surfs.items()}
            hinge_report.update({k.replace("elevon", "aileron"): v
                                 for k, v in rep.items()})
        else:
            surfaces = {}

        if tail_type == "v_tail":
            # ruddervators replace elevator + rudder, one per panel
            elevators, rudder = {}, None
            coords_fin = _airfoil_pts(fin_af)
            rv_c = float(taild.get("ruddervator_chord_frac")
                         or taild.get("elevator_chord_frac") or 0.30)
            xc_h_v = _clamp(1.0 - rv_c, 0.40, 0.85)
            t_frac_p = (_foil_surf_t(coords_fin, xc_h_v, True)
                        - _foil_surf_t(coords_fin, xc_h_v, False))
            for sgn_p, nm_p in ((1.0, "right"), (-1.0, "left")):
                airframe, rv, rv_rep = _split_ruddervator(
                    airframe, panel_stations[nm_p], vt["span_panel_mm"],
                    rv_c, sgn_p, vt["dihedral_deg"], t_frac_p)
                rv_reps[nm_p] = rv_rep
                if rv_rep.get("ok") and rv is not None:
                    ruddervators[f"ruddervator_{nm_p}"] = rv
                    hinge_report[f"ruddervator_{nm_p}"] = \
                        rv_rep.get("hinges", {})
                else:
                    warnings.append(f"{nm_p} ruddervator not separated: "
                                    + str(rv_rep.get("reason")))
        else:
            # elevator: LEFT + RIGHT panels, inboard edge clear of the post
            airframe, cut_elev, rep_e = _separate_elevons(
                airframe, stab, 0.10, 0.95, elev_c)
            elevators = {k.replace("elevon", "elevator"): v
                         for k, v in cut_elev.items()}
            hinge_report.update({k.replace("elevon", "elevator"): v
                                 for k, v in rep_e.items()})

            # rudder: above the stab and the cone's top deck (t_tail: the
            # stab rides at the fin TIP, so the rudder spans the fin below)
            if tail_type == "t_tail":
                stab_half_t = float(dims.get("t_tail", {}).get("stab_half",
                                                               4.0))
                z_r_lo = fus.post_top + 3.0
                z_r_hi = min(z_fin_root + 0.87 * span_fin,
                             z_stab - stab_half_t - 4.0)
            else:
                z_r_lo = max(z_stab + 0.55 * stab.tc * stab.c_root * 1.2,
                             fus.post_top) + 3.0
                z_r_hi = z_fin_root + 0.87 * span_fin
            coords_fin = _airfoil_pts(fin_af)
            xc_h_r = _clamp(1.0 - rud_c, 0.40, 0.85)
            t_frac_r = (_foil_surf_t(coords_fin, xc_h_r, True)
                        - _foil_surf_t(coords_fin, xc_h_r, False))
            airframe, rudder, rudder_rep = _split_rudder(
                airframe, fin_station, span_fin, z_r_lo, z_r_hi, rud_c,
                z_fin_root, t_frac_r)
            if rudder_rep.get("ok"):
                hinge_report["rudder"] = rudder_rep.get("hinges", {})
            else:
                warnings.append("rudder not separated: "
                                + str(rudder_rep.get("reason")))

        # elevator horn on the RIGHT panel (the left is joined with a torsion
        # wire at the bench - standard split-elevator practice)
        elev_horn: dict = {}
        if "elevator_right" in elevators:
            try:
                p_in, p_out, t_in, t_out = _elevon_hinge_line(
                    stab, 1.0, 0.10, min(0.95, _TIP_START - 0.01),
                    _clamp(1.0 - elev_c, 0.45, 0.90))
                horned, elev_horn = _sv.control_horn(
                    elevators["elevator_right"], hinge_p_in=p_in,
                    hinge_p_out=p_out, station_frac=0.45,
                    thickness=t_in + 0.45 * (t_out - t_in))
                if horned.isValid() and len(horned.Solids()) == 1:
                    elevators["elevator_right"] = horned
                else:
                    elev_horn = {"ok": False,
                                 "reason": "horn broke the elevator"}
            except Exception as exc:
                elev_horn = {"ok": False, "reason": str(exc)}
            elev_horn.setdefault(
                "note", "the left panel is joined with a torsion wire; one "
                        "pushrod drives both halves")
    else:
        surfaces, elevators, rudder = {}, {}, None
        # the one-piece STL keeps the surfaces attached, hinge lines scribed
        if has_ail:
            for cutter in _elevon_grooves(wing, g.get("airfoil",
                                                      "NACA 2412"),
                                          ail_in, ail_out, ail_c):
                grooves.append(("airframe", cutter))
        if tail_type != "v_tail":
            for cutter in _elevon_grooves(stab, fin_af, 0.10, 0.95, elev_c):
                grooves.append(("airframe", cutter))

    # ---- pushrod aim points ---------------------------------------------------
    pushrod_aims: dict[str, dict] = {}
    pushrod_pre_fail: dict[str, dict] = {}
    x_le_h = float(taild.get("x_le_h_m", 0.8 * l_f / MM)) * MM
    if tail_type == "v_tail":
        # one guide pipe per ruddervator, aimed under its panel's horn line
        ca_v = math.cos(math.radians(vt["dihedral_deg"]))
        sa_v = math.sin(math.radians(vt["dihedral_deg"]))
        rv_c2 = float(taild.get("ruddervator_chord_frac")
                      or taild.get("elevator_chord_frac") or 0.30)
        for sgn_p, nm_p in ((1.0, "right"), (-1.0, "left")):
            st_fn = panel_stations.get(nm_p)
            if st_fn is None:
                continue
            c_p, _twp, le_p = st_fn(0.40)
            pushrod_aims[f"ruddervator_{nm_p}"] = {
                "side": sgn_p,
                "target": (le_p.x + _clamp(1.0 - rv_c2, 0.4, 0.85) * c_p
                           + 12.0,
                           sgn_p * 0.40 * vt["span_panel_mm"] * ca_v,
                           le_p.z - 6.0),
                "x_stop": vt["x_le_mm"] - 25.0}
    else:
        try:
            p_in_e, p_out_e, _ti, _to = _elevon_hinge_line(
                stab, 1.0, 0.10, min(0.95, _TIP_START - 0.01),
                _clamp(1.0 - elev_c, 0.45, 0.90))
            y_horn_e = p_in_e.y + 0.45 * (p_out_e.y - p_in_e.y)
            z_h_e = 0.5 * (p_in_e.z + p_out_e.z)
            if tail_type == "t_tail" and t_tail_run.get("applied"):
                # the horizontal pipe hands over to the fin run: aim at the
                # fin-run's bottom mouth instead of the (fin-top) horn
                bm_t = t_tail_run["bottom_mm"]
                pushrod_aims["elevator"] = {
                    "side": +1.0,
                    "target": (bm_t[0], 10.0, bm_t[2]),
                    "x_stop": x_le_v2 - 25.0}
            else:
                pushrod_aims["elevator"] = {
                    "side": +1.0,
                    "target": (0.5 * (p_in_e.x + p_out_e.x) + 14.0, y_horn_e,
                               z_h_e - 10.0),
                    "x_stop": x_le_h - 25.0}
        except Exception as exc:
            pushrod_pre_fail["elevator"] = {"ok": False, "applied": False,
                                            "reason": str(exc)}
        if separate_parts and rudder_rep.get("ok"):
            p_lo_r = rudder_rep["hinge_p_lo"]
            pushrod_aims["rudder"] = {
                "side": -1.0,
                "target": (p_lo_r.x + 10.0, -14.0, p_lo_r.z - 8.0),
                "x_stop": x_le_v2 - 25.0}
        elif not separate_parts:
            # the one-piece build still gets the guide pipe: aim at the
            # nominal rudder-horn station on the fin
            z_r_lo1 = (fus.post_top + 9.0 if tail_type == "t_tail"
                       else max(z_stab + 6.0, fus.post_top) + 3.0)
            f_lo1 = _clamp((z_r_lo1 - z_fin_root) / max(span_fin, 1e-6),
                           0.0, 1.0)
            c1, _tw1, le1 = fin_station(f_lo1)
            pushrod_aims["rudder"] = {
                "side": -1.0,
                "target": (le1.x + _clamp(1.0 - rud_c, 0.4, 0.85) * c1
                           + 10.0,
                           -14.0, z_r_lo1 - 8.0),
                "x_stop": x_le_v2 - 25.0}

    # ---- servos, horns, wire pipes -------------------------------------------
    servo_params = None
    if spar_info.get("applied") and has_ail:
        # keep the servo pocket AFT of the spar channel: case and tube must
        # not share the section (real ships mount the servo behind the spar)
        c_mid = wing.section(0.65).chord
        servo_params = {"xc_min": max(0.16, SPAR_XC
                                      + (0.5 * float(spar_info["d_mm"])
                                         + 2.0) / max(c_mid, 1.0))}
    _progress("servos")
    airframe, horned_ail, servo_report = _install_hardware(
        airframe, wing, fus, surfaces, aild, wall, bay_mm, pushrod_aims,
        separate_parts, servo_params=servo_params)
    for key, fail in pushrod_pre_fail.items():
        servo_report.setdefault("pushrods", {}).setdefault(key, fail)
    if separate_parts:
        surfaces.update(horned_ail)
        if "elevator_right" in elevators:
            servo_report.setdefault("horns", {})["elevator_right"] = elev_horn
        if rudder is not None and rudder_rep.get("ok"):
            servo_report.setdefault("horns", {})["rudder"] = \
                rudder_rep.get("horn", {})
        for nm_p, rv_rep in rv_reps.items():
            if rv_rep.get("ok"):
                servo_report.setdefault("horns", {})[
                    f"ruddervator_{nm_p}"] = rv_rep.get("horn", {})

    parts["airframe"] = airframe
    if lid is not None:
        parts["hatch_lid"] = lid
    if separate_parts:
        parts.update(surfaces)
        parts.update(elevators)
        parts.update(ruddervators)
        if rudder is not None:
            parts["rudder"] = rudder

    # ---- CG marker -------------------------------------------------------------
    st = design.get("stability", {})
    x_cg = float(st.get("x_cg_m", 0.0)) * MM
    if x_cg <= 0.0:
        x_cg = x_w + 0.30 * c_root
    parts["cg_marker"] = _cg_marker(x_cg, fus.keel(x_cg, 0.0) + 1.0,
                                    scale=half / 550.0)

    meta = {
        "units": "mm",
        "airplane_type": str(design.get("airplane_type", "conventional")),
        "planform": design.get("planform", "trainer"),
        "wing_position": wing_pos,
        "tail_type": tail_type,
        "x_cg_mm": float(st.get("x_cg_m", 0.0)) * MM,
        "x_np_mm": float(st.get("x_np_m", 0.0)) * MM,
        "mac_mm": float(st.get("mac_m", 0.0)) * MM,
        "x_le_mac_mm": float(st.get("x_le_mac_m", 0.0)) * MM,
        "y_mac_mm": float(st.get("y_mac_m", 0.0)) * MM,
        "cg_pct_mac": st.get("cg_pct_mac", 0.0),
        "static_margin": st.get("static_margin", 0.0),
        "span_mm": span,
        "length_mm": length_total,
        "height_mm": float(g.get("height_total_m", 0.0)) * MM,
        "root_chord_mm": c_root,
        "fuselage_mm": {"length": l_f, "width": fw, "height": fh,
                        "x_wing_le": x_w, "x_nose": x_nose,
                        "wing_z": wing_z, "z_stab": z_stab},
        "tail_mm": {"x_le_h": x_le_h, "span_h": span_h,
                    "x_le_v": x_le_v2, "fin_span": span_fin,
                    "fin_sweep_deg": sweep_v, "z_fin_root": z_fin_root},
        "control_surfaces": g.get("control_surfaces", []),
        "ailerons": aild,
        "motor_mount": _jsonable(mount_info),
        "wall_mm": g.get("wall_mm", 1.2),
        "hinges": _jsonable(hinge_report),
        "servos": _jsonable(servo_report),
        "bay": _jsonable(bay_mm),
        "warnings": warnings,
        "valid_solid": all(bool(p.isValid()) for p in parts.values()),
    }
    # v3 blocks: present only when the design carries the feature
    if spar_info:
        meta["spar"] = _jsonable(spar_info)
    if ballast_info:
        meta["ballast_pocket"] = _jsonable(ballast_info)
    if t_tail_run:
        meta["t_tail_run"] = _jsonable(t_tail_run)
    if vt:
        meta["v_tail"] = _jsonable(vt)
    if isinstance(wing, _PolyWing):
        meta["polyhedral"] = _jsonable(wing.poly)
    return parts, grooves, meta


# ---------------------------------------------------------------------------
# Public API (geometry.py dispatches here - keep both signatures stable)
# ---------------------------------------------------------------------------

def build_design_parts(design: dict) -> tuple[dict[str, Solid], dict[str, Any]]:
    """The conventional aircraft as SEPARATE NAMED PARTS (mm), each in world
    position: `airframe`, `aileron_left/right`, `elevator_left/right`,
    `rudder`, `hatch_lid`, `cg_marker`."""
    parts, grooves, meta = _build(design, separate_parts=True)
    out: dict[str, Solid] = {}
    for name, solid in parts.items():
        cutters = [c for target, c in grooves if target == name]
        out[name] = _apply_grooves(solid, cutters) if cutters else solid
    meta["valid_solid"] = all(bool(p.isValid()) for p in out.values())
    meta["part_names"] = list(out)
    return out, meta


def build_design_solid(design: dict) -> tuple[Solid, dict[str, Any]]:
    """ONE fused, healed, watertight solid for STL / preview. Control
    surfaces stay attached with their hinge lines scribed; the hatch lid
    stays attached with its outline scribed - v1's exact contract."""
    parts, grooves, meta = _build(design, separate_parts=False)
    _progress("fuse")
    solid = _fuse_all(list(parts.values()))
    for _target, groove in grooves:
        try:
            cut = _heal(solid.cut(groove))
        except Exception:
            continue
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
