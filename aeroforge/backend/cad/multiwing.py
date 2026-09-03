"""CAD builder for the v3 MULTI-WING types: CANARD, TANDEM and BIPLANE.

Lives BESIDE the flying-wing and conventional paths (V3_PLAN.md wave 3a):
`geometry.build_design_parts` / `build_design_solid` will dispatch here when
`design["airplane_type"]` is one of ("canard", "tandem", "biplane") - the
dispatch itself is the integration wave's job, so both public entry points
are callable standalone and mirror `cad.conventional`'s contract exactly:
parts dict of named Solids in world mm + meta dict, or one fused/healed/
scribed watertight solid + meta.

Frame: x aft from the nose datum (mm), y right, z up, z = 0 on the thrust
line / fuselage mid-height (matches every physics module's envelope math).

TOPOLOGY, per type (each mirrors its physics module's layout dict exactly):

  * CANARD  - lifting foreplane on the nose (`geometry.canard`, shoulder
    height), MAIN wing aft at mid height, slab-sided fuselage between them
    whose gap IS the equipment bay, fixed TIP FINS on the main wing extended
    rearward (`geometry.fins`, arrangement "tip_fins" - fused like
    flying-wing winglets, no rudder: yaw is fin area, [LEN-CAN s.5]), and a
    single PUSHER motor bolted to the fuselage's flat AFT face. The aft face
    is widened to carry the plate (`_PusherFus`) and the loft ENDS at the
    mount station so the face the motor bolts to is real - the recorded
    length beyond it is spinner clearance, exactly like the tractor nose gap.
  * TANDEM  - front wing LOW through the fuselage belly, rear wing HIGH
    (`geometry.wing2` role "front" carries the front wing; the main block is
    the REAR wing - the Quickie arrangement, RESEARCH_TYPES_V3.md s.3.2),
    tractor nose motor, centre fin + RUDDER aft (the conventional module's
    rotated-frame machinery, reused). Where the high rear wing's root rides
    above the fuselage crown a wing-saddle PYLON is lofted between them -
    without it the two skins meet tangentially and the fuse degenerates
    (DECISIONS.md, "Near-coincident faces make a boolean silently produce
    NOTHING").
  * BIPLANE - lower wing on the fuselage keel, upper wing (`geometry.wing2`
    role "upper") one gap above and half a stagger ahead, carried on FOUR
    cabane struts over the fuselage and ONE INTERPLANE PAIR PER SIDE at the
    recorded `geometry.struts` stations - streamline-section printed struts,
    fused (they close the wing box, s.4.2). Conventional tail via the same
    stab/fin/elevator/rudder pattern as cad/conventional.py. Ailerons on the
    LOWER wing only; the upper wing is clean.

CONTROL SURFACES. Ailerons ride the main wing host with the full v1 servo
doctrine. The canard foreplane / tandem front wing carry a split LEFT+RIGHT
elevator (the fuselage owns the centreline) hinged with the captive-pin
machinery where the section depth allows, with hinges.py's documented
bevel-only fallback where it does not - exactly conventional.py's elevator
contract. Their freeing cuts CANNOT use `geometry._separate_elevons`
unmodified: its freeing slabs are unbounded normal to the plan, and on a
two-surface aircraft the slab aft of the FRONT surface's hinge plane passes
straight through the REAR wing (and on a biplane, the slab behind the lower
wing's hinge grazes the upper wing's trailing edge - measured 0.6 mm at
stagger 0.25 c). `_separate_surfaces_bounded` is therefore a minimal
reimplementation of that function's split loop with an axis-aligned bounding
box intersected into the pocket; editing geometry.py is fenced off for this
wave, and the hinge/bevel/cove machinery itself is reused untouched.

WIRE RUNS. Hardware doctrine verbatim (ARCHITECTURE.md): measured SG90 inverse
pockets at 0.25 mm, every run one round 8.25 mm pipe with 1.6x trumpeted
mouths starting AT the lead grommet, world-aligned horns with one proven
2.5 mm bore, every cut existence-checked by classification. The aileron lead
run is ONE STRAIGHT pipe either way (builder's spec, round 5), aimed by
where the bay sits:

  * bay beside/under the wing root chord: conventional.py's straight
    `_aileron_run` is delegated to unchanged (perpendicular side-wall
    entry, local bay enlargement when that is what perpendicularity needs,
    straight-oblique fallback);
  * bay AHEAD of the wing (the reference designs - the bay lives between
    the two lifting surfaces): one straight pipe angled forward from the
    lead grommet through the wing root and the intervening bulkhead(s)
    into the MEASURED bay void (`bay_mm["x0_mm"]/["x1_mm"]` + the
    classified z band - never the nominal envelope), and a probe INSIDE
    the pierced wall proves the lead actually enters (`into_bay_open`).

The elevator servo sits IN the fuselage bay and its pushrod guide pipe exits
FORWARD through the bay's front bulkhead toward the foreplane horn - the
conventional module's aft pushrod-exit pattern, mirrored; the tandem/biplane
rudder (and biplane elevator) keep the aft-exit original.

EQUIPMENT BAY. `hatch.build_bay` unchanged through the same `_FusBayHost`
duck type conventional.py uses. Type-specific honesty about the aperture:
the biplane bay is clamped to end ahead of the FORWARD cabane strut (a strut
footed on the hatch lid would come off with it), and every bay already ends
ahead of the aft lifting surface per its physics dict.
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
from .conventional import (
    PIPE_D_MM,
    _aileron_run,
    _ConvWing,
    _FusBayHost,
    _FusProfile,
    _is_air,
    _mount_cutters,
    _split_rudder,
    _straight_pipe,
)
from .geometry import (
    MM,
    _airfoil_pts,
    _apply_grooves,
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
    _slab,
    _tessellates_cleanly,
    _TIP_START,
    _void_z_band,
    fuse_feature as _fuse_feature,
)

# The NACA sections these types fly (0009/0010/2412/...) are registered into
# the airfoil LIBRARY by their physics modules. A design dict can arrive here
# without the physics ever having been imported in this process, and
# `_airfoil_pts` would then silently fall back to a REFLEXED analytic section
# - the exact opposite of a tailed/two-surface wing. Same guard as
# cad/conventional.py.
try:  # pragma: no cover - only fails if the physics is mid-rewrite
    from ..physics import canard as _reg_can  # noqa: F401
    from ..physics import tandem as _reg_tan  # noqa: F401
except Exception:
    pass

_BIG = 1.0e5          # "unbounded" extent for axis-aligned bounding boxes

# Spanwise structure a moving surface leaves between its end face and the
# foot of a FIXED vertical surface standing on the same wing: the 1.0 mm
# spanwise end clearance every control surface already carries
# (`hinges.HINGE_DEFAULTS["end_clearance_mm"]`) plus 3 mm of wing to carry
# the fin's root moment. Smaller than this and the cut runs into the fin's
# own footprint, which is how a tip fin ends up hinged to an aileron.
_FIN_SURFACE_GAP_MM = 4.0


def _outer_frac_clear_of(wing, sgn: float, inner: float, outer: float,
                         xc: float, y_face_mm: float,
                         x_fwd_mm: float, x_aft_mm: float,
                         gap_mm: float = _FIN_SURFACE_GAP_MM) -> float:
    """The largest outboard station <= `outer` at which the elevon freeing
    pocket misses a fixed surface whose plan footprint is
    `y_face_mm` (inboard face) over x in [x_fwd_mm, x_aft_mm].

    A SPANWISE STATION IS NOT A CLEARANCE. `_separate_elevons` bounds the
    pocket with a plane whose normal is the HINGE LINE direction, so the
    pocket's outboard edge runs aft along `aft = perp(u)` rather than along
    x: when the hinge line (75% chord, not the leading edge) is swept
    FORWARD - which a tapered wing's rear spar line usually is, measured
    -4.4 deg on canard_sport even with a swept-back LE - that edge rakes
    OUTBOARD as it goes aft. Pulling the station in to `y_fin - foot - gap`
    therefore still overshot the fin by 11.8 mm at the trailing edge and the
    aileron came off carrying the whole fin (z = 164.7 mm on a 23.8 mm
    crown). The fin is cleared against the pocket's real boundary instead:
    every corner of the footprint must sit outboard of the plane through
    p_out, by `gap_mm` measured along the plane normal.

    Bisected rather than solved because `_elevon_hinge_line` is the only
    thing that knows where the hinge line actually lands (it walks the
    section geometry), and it is pure arithmetic - no booleans, no cost.
    """
    from .geometry import _elevon_hinge_line

    lo = inner + 0.10
    if outer <= lo:
        return outer

    def clears(frac: float) -> bool:
        try:
            p_in, p_out, _t0, _t1 = _elevon_hinge_line(wing, sgn, inner,
                                                       frac, xc)
        except Exception:
            return False
        d = p_out - p_in
        if d.Length < 1e-9:
            return False
        u = d.multiply(1.0 / d.Length)
        # worst footprint corner = the one least far along u
        return min(u.x * x + u.y * (sgn * y_face_mm)
                   for x in (x_fwd_mm, x_aft_mm)) \
            >= u.x * p_out.x + u.y * p_out.y + gap_mm

    if clears(outer):
        return outer
    if not clears(lo):
        return lo
    for _ in range(40):                      # ~1e-12 on a 0-1 fraction
        mid = 0.5 * (lo + outer)
        if clears(mid):
            lo = mid
        else:
            outer = mid
    return lo

# Biplane strut proportions. The physics records ONE station per strut group
# (`interplane_x_m` at 0.35 MAC, `cabane_x_m` likewise on the upper wing);
# real interplane/cabane struts come as fore/aft PAIRS that close the bay
# into a truss (every s.4.1 reference airframe), so the pair straddles the
# recorded station by +-STRUT_PAIR_HALF_FRAC of the MAC, clamped so both feet
# stay on their wing's chord and clear of the aileron hinge line.
STRUT_PAIR_HALF_FRAC = 0.16
STRUT_PRINT_NOTE = (
    "struts are streamline (elliptical) sections, chord along x, ~12 x 5 mm; "
    "printed as part of the airframe they are near-vertical columns on a "
    "nose-up/tail-up build and clean bridges are impossible wings-flat - "
    "print the biplane cell standing on its nose or tail, or slice the "
    "struts with supports")


# ---------------------------------------------------------------------------
# Pusher fuselage: blunt aft face that IS the motor mount
# ---------------------------------------------------------------------------

class _PusherFus(_FusProfile):
    """`_FusProfile` with the tail cone ending in a flat structural aft face
    sized for the pusher motor plate, instead of the tractor profile's
    near-knife vertical post. The nose keeps a plain blunt cap (no firewall
    there - the motor is aft). Construct it with `l_f` equal to the MOUNT
    face station: the loft ends where the motor bolts on, and the recorded
    total length beyond it is the spinner clearance (the exact mirror of the
    tractor nose gap, conventional.SPINNER_NOTE)."""

    def __init__(self, *, l_f: float, w: float, h: float, x_nose: float,
                 r_plate: float) -> None:
        # nose sized for bluntness only (r_plate=8 keeps a0/b0 modest)
        super().__init__(l_f=l_f, w=w, h=h, x_nose=x_nose, r_plate=8.0)
        a_aft = _clamp(r_plate + 3.0, 8.0, 0.92 * self.W)
        b_aft = _clamp(r_plate + 3.0, 8.0, 0.92 * self.T)
        self.w_post = a_aft
        self.post_top = b_aft         # centred on z = 0, the thrust line
        self.post_bot = -b_aft


def _mount_cutters_pusher(spec: dict, l_build: float
                          ) -> tuple[list[tuple[Solid, Vector]], dict]:
    """Screw/shaft bores for the PUSHER aft face: conventional's
    `_mount_cutters` mirrored to drill FORWARD (-x) from the flat aft face at
    `spec["x_m"]`, each bore paired with a probe point that must classify as
    AIR after the cut (the horn-keyhole doctrine)."""
    if not spec:
        return [], {}
    x_face = float(spec.get("x_m", 0.0)) * MM
    y_c = float(spec.get("y_m", 0.0)) * MM
    z_c = float(spec.get("z_m", 0.0)) * MM
    r_bolt = float(spec.get("bolt_circle_radius_mm", 13.4))
    d_screw = float(spec.get("screw_hole_d_mm", 3.2))
    d_shaft = float(spec.get("shaft_hole_d_mm", 8.0))
    t_plate = float(spec.get("plate_thickness_mm", 4.0))
    n = max(int(spec.get("n_screws", 4)), 2)

    lead = 6.0 + max(l_build - x_face, 0.0)   # start clear beyond the face
    screw_depth = _clamp(3.0 * t_plate, 9.0, 16.0)
    if n == 4:
        s = r_bolt / math.sqrt(2.0)
        offsets = [(s, s), (s, -s), (-s, s), (-s, -s)]
    else:
        offsets = [(r_bolt * math.cos(2 * math.pi * i / n),
                    r_bolt * math.sin(2 * math.pi * i / n)) for i in range(n)]
    axis = Vector(-1, 0, 0)
    cutters: list[tuple[Solid, Vector]] = []
    for dy, dz in offsets:
        cutters.append((
            Solid.makeCylinder(0.5 * d_screw, lead + screw_depth,
                               Vector(x_face + lead, y_c + dy, z_c + dz),
                               axis),
            Vector(x_face - 1.5, y_c + dy, z_c + dz)))
    shaft_depth = _clamp(2.0 * t_plate, 8.0, 15.0)
    cutters.append((
        Solid.makeCylinder(0.5 * d_shaft, lead + shaft_depth,
                           Vector(x_face + lead, y_c, z_c), axis),
        Vector(x_face - 1.5, y_c, z_c)))
    info = {"x_face_mm": x_face, "n_screws": n,
            "screw_depth_mm": round(screw_depth, 1),
            "shaft_depth_mm": round(shaft_depth, 1),
            "note": "the flat AFT face is the pusher firewall; the recorded "
                    "length beyond it is the spinner clearance"}
    return cutters, info


# ---------------------------------------------------------------------------
# Bounded control-surface separation (two-surface aircraft need it)
# ---------------------------------------------------------------------------

def _bound_box(x_lo: float, x_hi: float, z_lo: float, z_hi: float) -> Solid:
    return Solid.makeBox(x_hi - x_lo, 2.0 * _BIG, z_hi - z_lo,
                         Vector(x_lo, -_BIG, z_lo))


def _separate_surfaces_bounded(
        airframe: Solid, host: Any, inner: float, outer: float,
        chord_frac: float, *, x_aft_max: float | None = None,
        z_max: float | None = None
) -> tuple[Solid, dict[str, Solid], dict[str, dict]]:
    """`geometry._separate_elevons`, with the freeing pocket intersected into
    an axis-aligned bound so it cannot amputate the OTHER lifting surface.

    Reimplemented rather than imported (with this comment saying why): the
    original's `_slab` pockets are unbounded normal to the plan, which is
    fine on a one-wing aircraft and catastrophic on a stacked/staggered one -
    the pocket aft of a canard/tandem FRONT surface's hinge plane passes
    through the rear wing, and the pocket behind a biplane's lower-wing
    hinge line grazes the upper wing's trailing edge (0.6 mm margin at
    stagger 0.25 c). geometry.py is fenced for this wave, so the split loop
    is copied here minimally; the double-bevel/cove/captive-pin machinery
    (`hinges.print_in_place_hinges`) is reused untouched, including its
    documented bevel-only fallback on thin sections.

    `x_aft_max` caps the pocket's aft reach (front-surface elevators);
    `z_max` caps it upward (biplane lower-wing ailerons under the upper
    wing). Names stay `elevon_left`/`elevon_right`; callers rename.
    """
    from . import hinges as _h

    xc = _clamp(1.0 - chord_frac, 0.45, 0.90)
    inner = _clamp(inner, max(host.fb, 0.10), 0.85)
    outer = _clamp(outer, inner + 0.10, _TIP_START - 0.01)
    reach = 4.0 * host.half
    bound = _bound_box(-_BIG, x_aft_max if x_aft_max is not None else _BIG,
                       -_BIG, z_max if z_max is not None else _BIG)
    out: dict[str, Solid] = {}
    reports: dict[str, dict] = {}

    for sgn, name in ((1.0, "elevon_right"), (-1.0, "elevon_left")):
        try:
            p_in, p_out, t_in, t_out = _elevon_hinge_line(
                host, sgn, inner, outer, xc)
        except Exception:
            continue
        d = p_out - p_in
        span = d.Length
        if span < 25.0:
            continue
        u = d.multiply(1.0 / span)
        aft = Vector(-u.y, u.x, 0.0)
        if aft.Length < 1e-9:
            aft = Vector(1.0, 0.0, 0.0)
        aft = aft.multiply(1.0 / aft.Length)
        if aft.x < 0:
            aft = aft.multiply(-1.0)

        try:
            span_box = _slab(p_in, u, 0.0, span, reach)
            pocket = (_slab(p_in, aft, 0.0, reach, reach)
                      .intersect(span_box).intersect(bound))
            surface = _heal(airframe.intersect(pocket))
            keep = [s for s in surface.Solids() if s.Volume() > 50.0]
            if not keep:
                continue
            surface = max(keep, key=lambda s: s.Volume())
            trimmed = _heal(airframe.cut(pocket))
            if not trimmed.isValid() or len(trimmed.Solids()) != 1:
                continue
        except Exception:
            continue

        try:
            trimmed, surface, info = _h.print_in_place_hinges(
                trimmed, surface, p_in, p_out, t_in, t_out, n_hinges=2)
        except Exception as exc:                     # never lose the split
            info = {"mode": "none",
                    "warnings": [f"hinge module failed: {exc}"]}

        keep = [s for s in surface.Solids() if s.Volume() > 50.0]
        if not keep or not trimmed.isValid() or len(trimmed.Solids()) != 1:
            continue
        out[name] = max(keep, key=lambda s: s.Volume())
        airframe = trimmed
        reports[name] = info
    return airframe, out, reports


# ---------------------------------------------------------------------------
# Biplane struts
# ---------------------------------------------------------------------------

def _ellipse_xy(xc: float, yc: float, z: float, a: float, b: float,
                n: int = 25) -> Wire:
    """Closed streamline (elliptical) section in the x-y plane at height z:
    chord 2a along x, thickness 2b along y."""
    pts = [Vector(xc + a * math.cos(t), yc + b * math.sin(t), z)
           for t in np.linspace(0.0, 2.0 * math.pi, n)[:-1]]
    return Wire.assembleEdges([Edge.makeSpline(pts + [pts[0]])])


def _strut(x_bot: float, y_bot: float, z_bot: float,
           x_top: float, y_top: float, z_top: float,
           chord: float, thick: float) -> Solid:
    """One streamline strut lofted between two horizontal sections. Both end
    stations must already be INSIDE their parent solids (the caller buries
    them a few mm) so the fuse leaves no tangent seam."""
    a, b = 0.5 * chord, 0.5 * thick
    return Solid.makeLoft([_ellipse_xy(x_bot, y_bot, z_bot, a, b),
                           _ellipse_xy(x_top, y_top, z_top, a, b)], False)


# ---------------------------------------------------------------------------
# Wing lead run: pocket -> equipment bay, whatever their relative layout
# ---------------------------------------------------------------------------

def _wing_lead_run(wing: _ConvWing, bay: "_sv.ServoBay", wall: float,
                   bay_mm: dict, fus: _FusProfile,
                   hinge_xc: float = 0.75,
                   span_mm: tuple[float, float] = (0.0, 1e9)
                   ) -> tuple[list[tuple[str, Solid]],
                              Vector | None, list[Vector] | None, dict]:
    """ONE straight pipe carrying one aileron servo lead to the fuselage
    bay, whatever the relative layout.

    Builder's spec (round 5, 2026-08-24): the corridor+riser and
    corridor+leg chains - each with a turn the wire had to make - are
    gone. A straight rod fed in at the pocket mouth exits inside the void.

    * bay void beside/under the wing root: conventional's straight
      `_aileron_run` (perpendicular side-wall entry, local bay enlargement
      when that is what perpendicularity needs, straight-oblique
      fallback);
    * bay void AHEAD of the wing (canard / tandem reference layouts): one
      straight pipe angled forward through the wing root and the
      intervening bulkheads, piercing the void's aft wall.

    Either way the probe point sits in the pierced wall's former material
    so `_install_hardware` can prove the lead actually enters the
    compartment.
    """
    bm = bay_mm or {}
    if bm.get("x0_mm") is None or bm.get("x1_mm") is None:
        return [], None, None, {"ok": False,
                                "reason": "no bay void - nowhere for the "
                                          "lead"}
    x0, x1 = float(bm["x0_mm"]), float(bm["x1_mm"])
    w_bay = float(bm.get("width_mm", 0.0))
    if w_bay < 18.0:
        return [], None, None, {"ok": False,
                                "reason": f"bay is only {w_bay:.0f} mm wide"}

    sec_r = wing.section(0.02)
    x_wing_lo = sec_r.le.x + 0.10 * sec_r.chord
    if x1 - 4.0 > x_wing_lo:
        # the void reaches beside/under the wing root: side-wall entry
        return _aileron_run(wing, fus, bay, wall, bay_mm,
                            hinge_xc=hinge_xc, span_mm=span_mm)

    # ---- bay wholly ahead of the wing: one straight pipe, angled forward --
    start = bay.cable_exit
    side = 1.0 if (start.y or 1.0) >= 0 else -1.0
    zf, zc = bm.get("z_floor_aft_mm"), bm.get("z_ceil_aft_mm")
    if zf is None or zc is None or float(zc) - float(zf) < 10.0:
        return [], None, None, {"ok": False,
                                "reason": "no measured void z band to aim "
                                          "the run at"}
    zf, zc = float(zf), float(zc)
    r_m = 0.5 * PIPE_D_MM + 0.6
    guard = {"hinge_xc": float(hinge_xc), "margin_xc": 0.04,
             "span_lo_mm": float(span_mm[0]),
             "span_hi_mm": float(span_mm[1])}
    base = {"hinge_guard": guard, "start_overshoot_mm": 6.0,
            "end_overshoot_mm": 4.0, "max_start_x_drift_mm": 2.2}

    # candidate 1: DIRECT into the void's aft bulkhead. Usually refused
    # honestly - the line either violates the wing corridor or leaves the
    # aircraft between the wing LE and the fuselage (the router's air-gap
    # check) - but where it fits it needs no extra hollowing.
    y_v = side * min(_clamp(0.5 * w_bay - 8.0, 4.0, 26.0), 0.60 * fus.W)
    zband = (zf + r_m, zc - r_m)
    pipe, info = _cd.straight_conduit(
        [wing, fus], start=start, end_xy=(x1 - 8.0, y_v),
        end_z_band=zband, wall=wall,
        params=dict(base, end_overshoot_mm=8.0,
                    prefer_z=_clamp(start.z, zband[0], zband[1])))
    info["kind"] = "servo"
    info["shape"] = "straight"
    info["entry_mode"] = "direct_forward"
    info["perpendicular"] = False
    if pipe is not None:
        path = _cd.path_vectors(info)
        u = path[-1] - path[0]
        length = u.Length
        u = u.multiply(1.0 / max(length, 1e-9))
        if abs(u.x) > 1e-6:
            t_p = _clamp(((x1 + 3.0) - path[0].x) / u.x, 2.0, length - 2.0)
        else:
            t_p = length - 6.0
        probe = path[0] + u.multiply(t_p)
        return [("pipe", pipe)], probe, path, info
    direct_refusal = {"skipped": info.get("skipped"),
                      "tightest": info.get("tightest")}

    # candidate 2: end the tube at the WING ROOT over the fuselage (the aim
    # ladder starts in the thick corridor - an aim near the LE dies in the
    # knife-thin nose sections), then hollow the fuselage out further so
    # the bay extends aft to meet it: a GALLERY forward into the void, and
    # a WELL down from the tube mouth when the tube (a high wing) ends
    # above the void band. The rod emerges from the straight tube into bay
    # air; the wire never bends inside a channel.
    sec_r2 = wing.section(0.02)
    x_le2 = float(sec_r2.le.x)
    c_r2 = float(sec_r2.chord)
    aims = [start.x, x_le2 + 0.40 * c_r2, x_le2 + 0.30 * c_r2,
            x_le2 + 0.20 * c_r2]
    y_e = side * _clamp(0.35 * fus.W, 6.0, 0.5 * fus.W - 4.0)
    pipe, x_e = None, 0.0
    for x_try in aims:
        x_try = min(float(x_try), fus.l_f - 12.0)
        f_e = _clamp(y_e / wing.half, -0.98, 0.98)
        xc_e = float(wing.xc_at(f_e, x_try))
        if not (0.02 <= xc_e <= 0.9):
            continue
        wk = float(wing.keel_z(f_e, xc_e))
        wc = float(wing.crown_z(f_e, xc_e))
        pipe, info = _cd.straight_conduit(
            [wing, fus], start=start, end_xy=(x_try, y_e),
            end_z_band=(wk - 1.0, wc - wall - r_m), wall=wall,
            params=dict(base, prefer_z=0.5 * (wk + wc)))
        x_e = x_try
        if pipe is not None:
            break
    info = dict(info or {})
    info["kind"] = "servo"
    info["shape"] = "straight"
    info["entry_mode"] = "gallery_forward"
    info["perpendicular"] = False
    info["direct_refusal"] = direct_refusal
    if pipe is None:
        info.setdefault("skipped",
                        "no straight line reaches the wing root either")
        return [], None, None, info
    z_e = float(info["end_z_mm"])

    # the hollowed extension: gallery (and well when the tube ends above
    # the void band), each cap checked against the analytic skins
    ya, yb = sorted((y_e - side * 9.0, y_e + side * 7.0))
    for x in (x1 - 6.0, x_e + 7.0):
        for y in (ya, yb):
            if not fus.contains_plan(x, y, 0.5):
                info["skipped"] = ("bay extension leaves the fuselage plan "
                                   f"at x={x:.0f}, y={y:.0f}")
                return [], None, None, info
    exp = None
    if zf + 2.0 <= z_e <= zc - 2.0:
        g_lo = max(zf + 2.0, z_e - 7.0)
        g_hi = min(zc - 2.0, z_e + 7.0)
        exp = Solid.makeBox((x_e + 7.0) - (x1 - 6.0), yb - ya, g_hi - g_lo,
                            Vector(x1 - 6.0, ya, g_lo))
        kind = "gallery"
        well_z = None
    else:
        crown_min = 1e9
        for x in (x_e - 7.0, x_e + 7.0):
            for y in (ya, yb):
                f = _clamp(y / wing.half, -0.98, 0.98)
                xcx = float(wing.xc_at(f, x))
                if 0.015 <= xcx <= 0.985:
                    crown_min = min(crown_min,
                                    float(wing.crown_z(f, xcx)))
                crown_min = min(crown_min, max(float(fus.crown(x, y)),
                                               crown_min))
        z_top_well = z_e + r_m + 1.0
        if z_e > zc - 2.0 and z_top_well > crown_min - wall - 0.3:
            info["skipped"] = (f"well top z={z_top_well:.1f} would thin "
                               f"the crown (min {crown_min:.1f}) below "
                               f"{wall:.1f} mm")
            return [], None, None, info
        if z_e > zc - 2.0:
            w_lo, w_hi = zc - 6.0, z_top_well
        else:
            keel_max = float(fus.keel(x_e, y_e))
            w_lo = z_e - r_m - 1.0
            if w_lo < keel_max + wall + 0.3:
                info["skipped"] = (f"well bottom z={w_lo:.1f} would thin "
                                   "the keel")
                return [], None, None, info
            w_hi = zf + 6.0
        well = Solid.makeBox(14.0, yb - ya, w_hi - w_lo,
                             Vector(x_e - 7.0, ya, w_lo))
        g_lo, g_hi = (zc - 12.0, zc - 2.0) if z_e > zc else (zf + 2.0,
                                                            zf + 12.0)
        gallery = Solid.makeBox((x_e + 7.0) - (x1 - 6.0), yb - ya,
                                g_hi - g_lo, Vector(x1 - 6.0, ya, g_lo))
        exp = gallery.fuse(well)
        kind = "gallery+well"
        well_z = [round(w_lo, 1), round(w_hi, 1)]
    info["bay_expansion"] = {
        "ok": True, "kind": kind,
        "x_mm": [round(x1 - 6.0, 1), round(x_e + 7.0, 1)],
        "y_mm": [round(ya, 1), round(yb, 1)],
        "well_z_mm": well_z,
        "what": ("the bay extended aft through the fuselage to meet the "
                 "straight tube - the hollowed-out enlargement that makes "
                 "a straight run possible with the void ahead of the "
                 "wing")}
    path = _cd.path_vectors(info)
    # probe in the extension's former material, opened only by these cuts
    probe = Vector(x1 + 3.0, y_e, _clamp(z_e, zf + 4.0, zc - 4.0))
    return [("pipe", pipe), ("baylift", exp)], probe, path, info


# ---------------------------------------------------------------------------
# Hardware installation (adapted from cad.conventional._install_hardware)
# ---------------------------------------------------------------------------

def _install_hardware(airframe: Solid, wing: _ConvWing, fus: _FusProfile,
                      surfaces: dict[str, Solid], ail: dict, wall: float,
                      bay_mm: dict, pushrod_aims: dict[str, dict],
                      separate_parts: bool
                      ) -> tuple[Solid, dict[str, Solid], dict]:
    """Aileron servo pockets + horns + linkage, every wire pipe, and the
    fuselage-servo pushrod exit runs. Adapted from conventional.py's
    `_install_hardware` (reimplementing is deliberate - that function bakes
    in the aft-only pushrod start and the riser-only lead run, and its
    surface is owned by a concurrent wave): the differences are the
    layout-aware `_wing_lead_run` and per-aim pushrod start ends (`"from":
    "front"` starts the guide pipe at the bay's FORWARD wall for the
    canard/tandem elevator, mirroring the aft pattern).
    """
    report: dict = {"bays": {}, "horns": {}, "conduits": {}, "pushrods": {}}
    out: dict[str, Solid] = {}
    void_probes: dict[str, Vector] = {}

    inner = _clamp(float(ail.get("inner_frac", 0.55)),
                   max(wing.fb, 0.10), 0.85)
    outer = _clamp(float(ail.get("outer_frac", 0.95)),
                   inner + 0.10, _TIP_START - 0.01)
    xc = _clamp(1.0 - float(ail.get("chord_frac", 0.25)), 0.45, 0.90)
    y_arm_frac = _clamp(inner + 0.10, wing.fb + 0.06, outer - 0.10)

    bay_cutters: list[tuple[str, Any]] = []
    conduit_cutters: list[tuple[str, Solid | None]] = []
    pipe_paths: dict[str, list[Vector]] = {}
    leg_paths: dict[str, list[Vector]] = {}

    for sgn, name in ((1.0, "aileron_right"), (-1.0, "aileron_left")):
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
                                params={"f_min": inner + 0.02})
        except Exception as exc:
            bay = None
            report["bays"][name] = {"ok": False, "reason": str(exc)}
        if bay is not None and bay.ok and bay.cutter is not None:
            bay_cutters.append((name, bay))
        elif bay is not None:
            report["bays"][name] = {"ok": False,
                                    "reason": bay.reason or "no room"}

        if surface is None:
            continue                       # one-piece build
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

    # ---- both servo pockets in ONE boolean, mesh-gated once ----------------
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
                    cutters, probe, leg_path, i_srv = _wing_lead_run(
                        wing, b, wall, bay_mm, fus,
                        hinge_xc=xc, span_mm=(inner * wing.half,
                                              outer * wing.half))
                    for kind, c in cutters:
                        conduit_cutters.append((f"{kind}_{nm}", c))
                    if probe is not None:
                        void_probes[nm] = probe
                    if leg_path is not None:
                        leg_paths[nm] = leg_path
                    if i_srv:
                        report["conduits"][f"servo_{nm}"] = i_srv
            else:
                report["bays"][nm] = {
                    "ok": False,
                    "reason": "bay cut would not mesh; skin left intact"}

    # ---- fuselage-servo pushrod exit runs ----------------------------------
    bm = bay_mm or {}
    if bm.get("x0_mm") is not None and bm.get("x1_mm") is not None:
        x0, x1 = float(bm["x0_mm"]), float(bm["x1_mm"])
        zf, zc_ = bm.get("z_floor_aft_mm"), bm.get("z_ceil_aft_mm")
        if zf is not None and zc_ is not None and float(zc_) - float(zf) > 12.0:
            z_s = float(zf) + _clamp(8.0, 4.0,
                                     0.5 * (float(zc_) - float(zf)))
        else:
            z_s = 0.5 * (fus.crown(0.5 * (x0 + x1), 0.0)
                         + fus.keel(0.5 * (x0 + x1), 0.0))
        for key, aim in pushrod_aims.items():
            side = float(aim["side"])              # +1 right, -1 left
            if aim.get("from") == "front":         # forward exit (canard)
                x_s = _clamp(x0 + 12.0, x0 + 6.0, x1 - 6.0)
            else:                                  # aft exit (conventional)
                x_s = _clamp(x1 - 12.0, x0 + 6.0, x1 - 6.0)
            start = Vector(x_s, side * 7.0, z_s)
            target = Vector(*aim["target"])
            x_stop = float(aim["x_stop"])
            t_end = _clamp((x_stop - start.x)
                           / ((target.x - start.x) or 1e-6), 0.35, 1.0)
            end = start + (target - start).multiply(t_end)
            cutter, path, info = _straight_pipe(start, end)
            info.update(kind="pushrod", drives=key,
                        exit_side="forward" if aim.get("from") == "front"
                        else "aft",
                        note=("straight guide pipe from the fuselage bay "
                              "through the intervening structure; the rod "
                              "continues externally to the horn"),
                        target_mm=[round(target.x, 1), round(target.y, 1),
                                   round(target.z, 1)])
            conduit_cutters.append((f"pushrod_{key}", cutter))
            pipe_paths[f"pushrod_{key}"] = path
            report["pushrods"][key] = info
    else:
        for key in pushrod_aims:
            report["pushrods"][key] = {
                "ok": False, "applied": False,
                "reason": "no bay void - the fuselage servos have nowhere "
                          "to sit"}

    # ---- cut everything at once, mesh-gated, then prove the cuts -----------
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
                    kind, nm = k.split("_", 1)
                    ci = report["conduits"].setdefault(f"servo_{nm}", {})
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

        # existence check: the lead must actually REACH the void - a probe
        # in the bulkhead the run pierces must be air, and where the run has
        # a straight leg its whole centreline must classify open
        for nm, probe in void_probes.items():
            ci = report["conduits"].get(f"servo_{nm}")
            if not isinstance(ci, dict):
                continue
            open_ = bool(ci.get("applied")) and _is_air(airframe, probe)
            if open_ and nm in leg_paths:
                try:
                    ro = _cd.route_is_open(airframe, leg_paths[nm])
                    open_ = bool(ro.get("open"))
                    if not open_:
                        ci["leg_open_detail"] = ro
                except Exception as exc:
                    open_ = False
                    ci["leg_open_detail"] = str(exc)
            ci["into_bay_open"] = bool(open_)
            # legacy key so shared tooling reads the same truth
            ci["riser_open"] = bool(open_)
            if not open_:
                ci["applied"] = False
                ci["why"] = ("run into the bay is blocked - the lead walks "
                             "up to the compartment and never enters")
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
                    rep["why"] = ("cut reported applied but the bore is "
                                  "blocked")
            except Exception as exc:
                rep["route_open"] = False
                rep["route_open_detail"] = str(exc)

    return airframe, out, report


# ---------------------------------------------------------------------------
# Shared build pieces
# ---------------------------------------------------------------------------

def _fuse_in(base: Solid, add: Solid, what: str,
             warnings: list[str]) -> tuple[Solid, bool]:
    """Fuse `add` into `base`, VERIFIED - see `geometry.fuse_feature`.

    This used to accept any result that was one valid solid, which is how
    the biplane shipped with no vertical fin at all: the boolean returned a
    valid single solid that simply did not contain the fin.
    """
    return _fuse_feature(base, add, what, warnings)


def _wing_host(g: dict, *, coords_name: str, half: float, c_root: float,
               c_tip: float, sweep: float, dihedral: float, incidence: float,
               washout: float, fb: float, x_le: float, z: float) -> _ConvWing:
    return _ConvWing(
        coords=_airfoil_pts(coords_name), half=half, c_root=c_root,
        c_tip=max(c_tip, 0.06 * c_root), sweep_deg=sweep,
        dihedral_deg=dihedral, incidence=incidence, washout=washout,
        fb=_clamp(fb, 0.04, 0.50), depth_scale=1.0, chord_scale=1.0,
        nose_round=0.6, crown_frac=0.62, x_le_root=x_le, z_mount=z)


def _blended(wing: _ConvWing) -> Solid:
    from .geometry import _blended_airframe
    return _blended_airframe(wing)


# The bay floor is held BAY_FLOOR_LIFT_MM above the fuselage keel. This is a
# boolean-survival figure, not a styling one, and it was measured: with the
# cavity floor at keel + wall (1.2 mm) the floor and the skin are two
# near-parallel B-spline bands a millimetre apart over the whole compartment,
# and OCC's fuse/cut on the superellipse fuselage silently degenerates - on
# the canard reference body `airframe.cut(void)` removed 150 cm3 of a
# 2693 cm3 compartment and `intersect` returned EMPTY for shapes overlapping
# by litres (the DECISIONS.md "boolean silently produces nothing" class, at
# compartment scale). Translating the identical void up 5 mm made the same
# booleans exact, so the floor is surveyed 4 mm off the keel: every rung then
# lands first-try on all three reference fuselages (canard rung 3 at 62 s vs
# every-rung-failed; tandem/biplane rung 0 vs rung 5/timeout). The 4 mm strip
# below the floor is structure, not waste - it is the battery tray's plinth.
BAY_FLOOR_LIFT_MM = 4.0
# The canard's physics records a bay that spans the whole canard-to-wing gap
# (537 mm on the reference design). A canopy that long fails to build on any
# rung, so the aperture is capped and centred to keep the recorded battery
# station inside; the hatch may only trim INTO what it is asked for, never
# grow it, so the cap must be applied by the caller.
BAY_LEN_CAP_MM = 380.0


def _bay_and_hatch(airframe: Solid, fus: _FusProfile, half: float,
                   bayd: dict, wall: float, separate_parts: bool,
                   warnings: list[str], *, x_max: float | None = None,
                   keel_guard: Callable[[float, float], float] | None = None,
                   battery_x_mm: float | None = None,
                   out: dict | None = None
                   ) -> tuple[Solid, dict, Solid | None, _FusBayHost]:
    """Equipment bay + hatch through the same duck type conventional uses.
    Returns (airframe, bay_mm with the measured void band, lid, host).

    Callers pass the BARE fuselage solid (before any lifting surface fuses):
    every type's bay region is disjoint from every wing/fin/strut root by
    construction (`x_max` clamps guarantee it), and hollowing a 5-face body
    is both faster and far more robust than hollowing the fused spline soup.
    """
    l_f = fus.l_f
    bay_start = float(bayd.get("bay_start_m", 0.08 * l_f / MM)) * MM
    bay_len = float(bayd.get("bay_length_m", 0.3 * l_f / MM)) * MM
    if bay_len > BAY_LEN_CAP_MM:
        lo_min = bay_start
        lo_max = bay_start + bay_len - BAY_LEN_CAP_MM
        centre = (battery_x_mm - 0.55 * BAY_LEN_CAP_MM
                  if battery_x_mm is not None else lo_min)
        bay_start = _clamp(centre, lo_min, lo_max)
        bay_len = BAY_LEN_CAP_MM
    bay_hw = 0.5 * float(bayd.get("bay_width_m",
                                  (2.0 * fus.W - 2.4) / MM)) * MM

    def lifted(x: float, y: float) -> float:
        k = fus.keel(x, y) + BAY_FLOOR_LIFT_MM
        if keel_guard is not None:
            g2 = keel_guard(x, y)
            if g2 > k:
                k = g2
        return k

    host = _FusBayHost(fus, half, lifted)
    bay_mm: dict = {}
    lid: Solid | None = None
    try:
        bay = _hatch.build_bay(
            host, bay_start=bay_start, bay_length=bay_len,
            bay_half_width=bay_hw, wall=wall, x_max=x_max,
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
            zb = _void_z_band(airframe, host, bay_mm)
            if zb is not None:
                bay_mm["z_floor_aft_mm"], bay_mm["z_ceil_aft_mm"] = zb
    elif bay is not None:
        warnings.append("no equipment bay could be cut: "
                        + str((bay.bay_mm or {}).get("reason")))
    if out is not None:
        # probe seam: the compartment solid the cut was made with
        out["cavity"] = bay.cavity if bay is not None and bay.ok else None
    return airframe, bay_mm, lid, host


def _drill_motor(airframe: Solid, mount_spec: dict, warnings: list[str],
                 l_build: float) -> tuple[Solid, dict]:
    """Drill the motor screw/shaft bores - tractor into the nose face,
    pusher forward from the aft face - and PROVE each open."""
    if str(mount_spec.get("type", "tractor")) == "pusher":
        cutters, mount_info = _mount_cutters_pusher(mount_spec, l_build)
    else:
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
    return airframe, mount_info


def _front_elevator(airframe: Solid, front: _ConvWing, elev_frac: float,
                    hinge_report: dict, warnings: list[str],
                    separate_parts: bool, x_aft_max: float
                    ) -> tuple[Solid, dict[str, Solid], dict]:
    """Split the LEFT+RIGHT elevator panels off a FRONT lifting surface
    (canard foreplane / tandem front wing), hinge them, and put the horn on
    the RIGHT panel (left joined with a torsion wire at the bench - standard
    split-elevator practice, same note as conventional's elevator)."""
    inner, outer = 0.14, 0.95
    airframe, cut, rep = _separate_surfaces_bounded(
        airframe, front, inner, outer, elev_frac, x_aft_max=x_aft_max)
    elevators = {k.replace("elevon", "elevator"): v for k, v in cut.items()}
    hinge_report.update({k.replace("elevon", "elevator"): v
                         for k, v in rep.items()})
    horn: dict = {}
    if "elevator_right" in elevators:
        try:
            xc_h = _clamp(1.0 - elev_frac, 0.45, 0.90)
            inner_c = _clamp(inner, max(front.fb, 0.10), 0.85)
            outer_c = _clamp(outer, inner_c + 0.10, _TIP_START - 0.01)
            p_in, p_out, t_in, t_out = _elevon_hinge_line(
                front, 1.0, inner_c, outer_c, xc_h)
            # horn near the INBOARD end: the pushrod comes forward from the
            # centreline bay, so an outboard horn would run the rod
            # diagonally across half the span
            horned, horn = _sv.control_horn(
                elevators["elevator_right"], hinge_p_in=p_in,
                hinge_p_out=p_out, station_frac=0.15,
                thickness=t_in + 0.15 * (t_out - t_in))
            if horned.isValid() and len(horned.Solids()) == 1:
                elevators["elevator_right"] = horned
            else:
                horn = {"ok": False, "reason": "horn broke the elevator"}
        except Exception as exc:
            horn = {"ok": False, "reason": str(exc)}
        horn.setdefault(
            "note", "the left panel is joined with a torsion wire; one "
                    "pushrod drives both halves")
    else:
        warnings.append("front-surface elevator was not separated")
    return airframe, elevators, horn


def _front_elevator_aim(front: _ConvWing, elev_frac: float,
                        x_le_front: float, c_root_front: float) -> dict:
    """Pushrod aim for the front-surface elevator: the pipe leaves the bay's
    FORWARD wall and stops just aft of the surface's trailing edge; the rod
    continues externally to the inboard horn."""
    xc_h = _clamp(1.0 - elev_frac, 0.45, 0.90)
    inner_c = _clamp(0.14, max(front.fb, 0.10), 0.85)
    outer_c = _clamp(0.95, inner_c + 0.10, _TIP_START - 0.01)
    p_in, p_out, _ti, _to = _elevon_hinge_line(
        front, 1.0, inner_c, outer_c, xc_h)
    y_horn = p_in.y + 0.15 * (p_out.y - p_in.y)
    z_h = p_in.z + 0.15 * (p_out.z - p_in.z)
    return {
        "side": +1.0, "from": "front",
        "target": (0.5 * (p_in.x + p_out.x) + 10.0, y_horn, z_h - 8.0),
        "x_stop": x_le_front + c_root_front + 18.0}


def _finish(design: dict, parts: dict[str, Solid], fus: _FusProfile,
            half: float, meta_extra: dict, hinge_report: dict,
            servo_report: dict, bay_mm: dict, mount_info: dict,
            warnings: list[str]) -> dict:
    g = design["geometry"]
    st = design.get("stability", {})
    x_cg = float(st.get("x_cg_m", 0.0)) * MM
    if x_cg <= 0.0:
        x_cg = 0.45 * fus.l_f
    parts["cg_marker"] = _cg_marker(x_cg, fus.keel(x_cg, 0.0) + 1.0,
                                   scale=half / 550.0)
    meta = {
        "units": "mm",
        "airplane_type": design.get("airplane_type"),
        "planform": design.get("planform"),
        "x_cg_mm": float(st.get("x_cg_m", 0.0)) * MM,
        "x_np_mm": float(st.get("x_np_m", 0.0)) * MM,
        "mac_mm": float(st.get("mac_m", 0.0)) * MM,
        "x_le_mac_mm": float(st.get("x_le_mac_m", 0.0)) * MM,
        "y_mac_mm": float(st.get("y_mac_m", 0.0)) * MM,
        "cg_pct_mac": st.get("cg_pct_mac", 0.0),
        "static_margin": st.get("static_margin", 0.0),
        "span_mm": float(g["span_m"]) * MM,
        "length_mm": float(g.get("length_total_m", fus.l_f / MM)) * MM,
        "height_mm": float(g.get("height_total_m", 0.0)) * MM,
        "root_chord_mm": float(g["root_chord_m"]) * MM,
        "fuselage_mm": {"length": fus.l_f, "width": 2.0 * fus.W,
                        "height": 2.0 * fus.T, "x_nose": fus.x_nose},
        "control_surfaces": g.get("control_surfaces", []),
        "ailerons": g.get("ailerons", {}),
        "motor_mount": _jsonable(mount_info),
        "wall_mm": g.get("wall_mm", 1.2),
        "hinges": _jsonable(hinge_report),
        "servos": _jsonable(servo_report),
        "bay": _jsonable(bay_mm),
        "warnings": warnings,
        "valid_solid": all(bool(p.isValid()) for p in parts.values()),
    }
    meta.update(_jsonable(meta_extra))
    return meta


# ---------------------------------------------------------------------------
# CANARD
# ---------------------------------------------------------------------------

def _hosts_canard(design: dict) -> dict[str, Any]:
    g = design["geometry"]
    cnd = g["canard"]
    fusd = g.get("fuselage") or {}
    span = float(g["span_m"]) * MM
    half = max(0.5 * span, 1.0)
    c_root = float(g["root_chord_m"]) * MM
    c_tip = float(g.get("tip_chord_m") or c_root * 0.7) * MM
    fw = float(fusd.get("width_m", 0.08)) * MM
    fh = float(fusd.get("height_m", 0.10)) * MM
    l_f = float(fusd.get("length_m", 0.9 * span / MM)) * MM
    x_w = float(g.get("x_le_wing_m", 0.7 * l_f / MM)) * MM
    mount_spec = g.get("motor_mount") or {}
    x_mount = float(mount_spec.get("x_m", (l_f - 10.0) / MM)) * MM

    # PUSHER: the loft ends AT the mount face - the flat aft face is the
    # firewall, the recorded length beyond it is spinner clearance
    fus = _PusherFus(l_f=min(x_mount, l_f), w=fw, h=fh, x_nose=2.0,
                     r_plate=float(mount_spec.get("plate_radius_mm", 16.0)))
    fus.set_cone_start(max(0.52 * fus.l_f, x_w + 0.55 * c_root))

    wing = _wing_host(
        g, coords_name=g.get("airfoil", "NACA 0010"), half=half,
        c_root=c_root, c_tip=c_tip,
        sweep=float(g.get("sweep_le_deg", 0.0)),
        dihedral=float(g.get("dihedral_deg", 0.0)),
        incidence=float(g.get("root_incidence_deg", 0.0)),
        washout=float(g.get("washout_deg", 0.0)),
        fb=(0.5 * fw) / half, x_le=x_w,
        z=float(g.get("wing_z_m", 0.0)) * MM)

    half_c = max(0.5 * float(cnd["span_m"]) * MM, 1.0)
    canard = _wing_host(
        g, coords_name=cnd.get("airfoil", "NACA 0009"), half=half_c,
        c_root=float(cnd["c_root_m"]) * MM,
        c_tip=float(cnd["c_tip_m"]) * MM,
        sweep=0.0, dihedral=0.0,
        incidence=float(cnd.get("incidence_deg", 0.0)),
        washout=0.0, fb=(0.5 * fw) / half_c,
        x_le=float(cnd["x_le_m"]) * MM,
        z=float(cnd.get("z_m", 0.25 * fh / MM)) * MM)
    return {"fus": fus, "main": wing, "fore": canard}


def _build_canard(design: dict, separate_parts: bool,
                  _probe: dict | None = None
                  ) -> tuple[dict[str, Solid], list[tuple[str, Solid]],
                             dict[str, Any]]:
    g = design["geometry"]
    cnd = g["canard"]
    fins = g.get("fins") or {}
    fusd = g.get("fuselage") or {}
    aild = g.get("ailerons") or {}
    warnings: list[str] = []
    hinge_report: dict = {}
    grooves: list[tuple[str, Solid]] = []

    wall = max(float(g.get("wall_mm", 1.2)), 0.6)
    span = float(g["span_m"]) * MM
    half = max(0.5 * span, 1.0)
    c_root = float(g["root_chord_m"]) * MM
    fw = float(fusd.get("width_m", 0.08)) * MM
    fh = float(fusd.get("height_m", 0.10)) * MM
    l_f = float(fusd.get("length_m", 0.9 * span / MM)) * MM
    x_w = float(g.get("x_le_wing_m", 0.7 * l_f / MM)) * MM
    mount_spec = g.get("motor_mount") or {}
    length_total = float(g.get("length_total_m", (l_f + 5.0) / MM)) * MM
    half_c = max(0.5 * float(cnd["span_m"]) * MM, 1.0)
    x_le_c = float(cnd["x_le_m"]) * MM
    c_root_c = float(cnd["c_root_m"]) * MM

    hosts = make_hosts(design)
    fus, wing, canard = hosts["fus"], hosts["main"], hosts["fore"]

    _progress("bay")
    # ---- bay + hatch FIRST, on the bare fuselage (see _bay_and_hatch) -----
    airframe = fus.solid()
    _bay_out: dict = {}
    raw_tail: dict[str, Solid] = {}    # the loose tail solids (probe seam)
    airframe, bay_mm, lid, _host = _bay_and_hatch(
        airframe, fus, half, fusd.get("bay") or {}, wall, separate_parts,
        warnings, x_max=x_w - 12.0,
        battery_x_mm=float(g.get("battery_x_m", 0.0)) * MM or None,
        out=_bay_out)

    _progress("loft")
    # ---- airframe: fuselage + wing + foreplane + tip fins ------------------
    airframe, _ok = _fuse_in(airframe, _blended(wing), "main wing", warnings)
    airframe, _ok = _fuse_in(airframe, _blended(canard), "canard foreplane",
                             warnings)

    _progress("fins")
    # tip fins: fixed, fused, rooted in the wing tip and extended rearward
    # ([LEN-CAN s.5] - clear of the prop circle). The foot stands at the
    # rounded-tip start station so it lands on full-chord material.
    fin_af = g.get("fin_airfoil", "NACA 0008")
    h_fin = float(fins.get("height_m", 0.10)) * MM
    c_root_f = float(fins.get("c_root_m", 0.09)) * MM
    c_tip_f = float(fins.get("c_tip_m", 0.06)) * MM
    x_le_f = float(fins.get("x_le_m", (x_w + c_root) / MM)) * MM
    f_fin = _TIP_START
    fins_info: dict = {"count": 0}
    for sgn in (1.0, -1.0):
        sec_t = wing.section(sgn * f_fin)
        y_fin = sec_t.le.y
        # bury the foot: root plane 2 mm under the local band mid at the
        # thick part of the shared x window
        xc_mid = _clamp((min(x_le_f + 0.5 * c_root_f,
                             sec_t.le.x + 0.85 * sec_t.chord)
                         - sec_t.le.x) / max(sec_t.chord, 1e-6), 0.05, 0.9)
        z_mid = 0.5 * (wing.crown_z(sgn * f_fin, xc_mid)
                       + wing.keel_z(sgn * f_fin, xc_mid))
        z_root = z_mid - 2.0
        x_le2, sweep_f = _clamp_aft(x_le_f, c_root_f, h_fin, 30.0,
                                    c_tip_f / max(c_root_f, 1e-6),
                                    x_aft=length_total)
        # the fin's plan footprint, which the aileron pocket must miss at
        # EVERY height (that pocket is unbounded in z): most forward at the
        # root leading edge, most aft at the swept tip's trailing edge
        fin_x_fwd = x_le2
        fin_x_aft = max(x_le2 + c_root_f,
                        x_le2 + h_fin * math.tan(math.radians(sweep_f))
                        + c_tip_f)
        fin_solid, _st = _rounded_surface(
            airfoil=fin_af, span_mm=h_fin, c_root_mm=c_root_f,
            c_tip_mm=c_tip_f, le_root=Vector(x_le2, y_fin, z_root),
            sweep_le_deg=sweep_f, dihedral_deg=0.0,
            twist_root_deg=0.0, twist_tip_deg=0.0,
            span_dir=Vector(0, 0, 1), tdir=Vector(0, 1, 0))
        raw_tail[f"tip_fin_{'right' if sgn > 0 else 'left'}"] = fin_solid
        airframe, fused_ok = _fuse_in(
            airframe, fin_solid,
            f"tip fin ({'right' if sgn > 0 else 'left'})", warnings)
        if fused_ok:
            fins_info["count"] += 1
    fins_info.update(arrangement="tip_fins", fused=fins_info["count"] == 2,
                     x_le_mm=round(x_le_f, 1), height_mm=round(h_fin, 1),
                     y_mm=round(abs(y_fin), 1))

    if _probe is not None:
        # probe seam (tools_probe_fin_intrusion.py): hand back the
        # pieces exactly as built and stop here - nothing downstream
        # adds material to a tail surface, it only cuts
        _probe.update(airframe=airframe, fins=raw_tail,
                      cavity=_bay_out.get("cavity"),
                      bay_mm=bay_mm, warnings=list(warnings))
        return {"airframe": airframe}, grooves, {"bay": bay_mm,
                                                 "probe": True}

    # The fin's spanwise footprint on the wing skin, which the AILERON must
    # stay clear of (see the control-surface section below).
    _fin_coords = _airfoil_pts(fin_af)
    t_frac_f = (_foil_surf_t(_fin_coords, 0.30, True)
                - _foil_surf_t(_fin_coords, 0.30, False))
    fin_foot_half = 0.5 * t_frac_f * c_root_f
    fin_y_face = abs(y_fin) - fin_foot_half

    # ---- pusher motor bores, proven open -----------------------------------
    airframe, mount_info = _drill_motor(airframe, mount_spec, warnings,
                                        fus.l_f)

    _progress("hinges")
    # ---- control surfaces --------------------------------------------------
    elev_frac = float(cnd.get("elevator_chord_frac", 0.20))
    ail_in = float(aild.get("inner_frac", 0.55))
    ail_out = float(aild.get("outer_frac", 0.95))
    ail_c = float(aild.get("chord_frac", 0.25))
    elev_horn: dict = {}

    # The ailerons must stop INBOARD of the tip fins, in BOTH build modes -
    # the split path cuts at this station and the one-piece path scribes at
    # it, and a scribe line that disagrees with the cut is a lie on the
    # model. Unbounded AFT is safe here (nothing sits behind this wing), but
    # the freeing pocket is unbounded in Z as well and the tip fins stand on
    # this very wing: at the sized 0.95 outer station the pocket reached
    # straight up through the fin and cut its whole aft section free, so the
    # "aileron" came off the build carrying two thirds of a fin (measured on
    # canard_sport - the part reached z = 164.7 mm on a wing whose skin tops
    # out at 20 mm, 108,907 mm3 against an elevator's 22,449). A moving
    # surface that carries a fixed vertical surface is the fin-on-elevon
    # fiasco (DECISIONS.md) in a new place. Ending the aileron inboard of the
    # fin is also simply where a tip-finned aeroplane puts it; the station it
    # actually got is recorded rather than silently standing in for the sized
    # one.
    if fins_info["count"]:
        # match `_separate_elevons`' own clamps so the station solved here is
        # the station the pocket is actually built at
        _xc = _clamp(1.0 - ail_c, 0.45, 0.90)
        _in = _clamp(ail_in, max(wing.fb, 0.10), 0.85)
        ail_out_clear = _outer_frac_clear_of(
            wing, 1.0, _in, _clamp(ail_out, _in + 0.10, _TIP_START - 0.01),
            _xc, fin_y_face, fin_x_fwd, fin_x_aft)
        if ail_out_clear < ail_out - 1e-6:
            warnings.append(
                f"aileron outer station pulled in from {ail_out:.3f} to "
                f"{ail_out_clear:.3f} semispan to clear the tip fin foot at "
                f"{abs(y_fin):.0f} mm (+/-{fin_foot_half:.1f} mm + "
                f"{_FIN_SURFACE_GAP_MM:.0f} mm), solved against the freeing "
                f"pocket's raked outboard edge rather than its hinge-line "
                f"station")
            ail_out = ail_out_clear

    if separate_parts:
        from .geometry import _separate_elevons
        airframe, cut_surfs, rep = _separate_elevons(
            airframe, wing, ail_in, ail_out, ail_c)
        surfaces = {k.replace("elevon", "aileron"): v
                    for k, v in cut_surfs.items()}
        hinge_report.update({k.replace("elevon", "aileron"): v
                             for k, v in rep.items()})
        # foreplane elevator: bounded aft so the pocket can never reach the
        # main wing at the same z
        airframe, elevators, elev_horn = _front_elevator(
            airframe, canard, elev_frac, hinge_report, warnings,
            separate_parts, x_aft_max=x_le_c + c_root_c + 30.0)
    else:
        surfaces, elevators = {}, {}
        for cutter in _elevon_grooves(wing, g.get("airfoil", "NACA 0010"),
                                      ail_in, ail_out, ail_c):
            grooves.append(("airframe", cutter))
        for cutter in _elevon_grooves(canard, cnd.get("airfoil", "NACA 0009"),
                                      0.14, 0.95, elev_frac):
            grooves.append(("airframe", cutter))

    # ---- pushrod aim: elevator servo in the bay, pipe exits FORWARD --------
    pushrod_aims: dict[str, dict] = {}
    try:
        pushrod_aims["elevator"] = _front_elevator_aim(
            canard, elev_frac, x_le_c, c_root_c)
    except Exception as exc:
        warnings.append(f"elevator pushrod aim failed: {exc}")

    _progress("servos")
    airframe, horned_ail, servo_report = _install_hardware(
        airframe, wing, fus, surfaces, aild, wall, bay_mm, pushrod_aims,
        separate_parts)
    parts: dict[str, Solid] = {"airframe": airframe}
    if separate_parts:
        surfaces.update(horned_ail)
        parts.update(surfaces)
        parts.update(elevators)
        if "elevator_right" in elevators:
            servo_report.setdefault("horns", {})["elevator_right"] = elev_horn
    if lid is not None:
        parts["hatch_lid"] = lid

    meta_extra = {
        "wing_position": g.get("wing_position", "mid"),
        "canard_mm": {"x_le": x_le_c, "span": 2.0 * half_c,
                      "c_root": c_root_c,
                      "z": float(cnd.get("z_m", 0.0)) * MM,
                      "elevator_chord_frac": elev_frac},
        "fins": fins_info,
        "tail_mm": {},          # tailless aft: tip fins carry the yaw area
        # what the ailerons were actually CUT at, which is not the sized
        # `ailerons` block when the tip fin pushed the outer station in
        "ailerons_built": {"inner_frac": round(ail_in, 4),
                           "outer_frac": round(ail_out, 4),
                           "chord_frac": round(ail_c, 4)},
    }
    meta = _finish(design, parts, fus, half, meta_extra, hinge_report,
                   servo_report, bay_mm, mount_info, warnings)
    return parts, grooves, meta


# ---------------------------------------------------------------------------
# TANDEM
# ---------------------------------------------------------------------------

def _hosts_tandem(design: dict) -> dict[str, Any]:
    g = design["geometry"]
    w2 = g["wing2"]                      # role "front"
    fusd = g.get("fuselage") or {}
    span = float(g["span_m"]) * MM
    half = max(0.5 * span, 1.0)
    c_root_r = float(g["root_chord_m"]) * MM
    fw = float(fusd.get("width_m", 0.07)) * MM
    fh = float(fusd.get("height_m", 0.09)) * MM
    l_f = float(fusd.get("length_m", 0.9 * span / MM)) * MM
    x_le_r = float(g.get("x_le_wing_m", 0.7 * l_f / MM)) * MM
    mount_spec = g.get("motor_mount") or {}
    x_nose = _clamp(float(mount_spec.get("x_m", 0.005)) * MM, 2.0, 15.0)
    bayd = fusd.get("bay") or {}
    bay_end = (float(bayd.get("bay_start_m", 0.0))
               + float(bayd.get("bay_length_m", 0.0))) * MM

    fus = _FusProfile(l_f=l_f, w=fw, h=fh, x_nose=x_nose,
                      r_plate=float(mount_spec.get("plate_radius_mm", 16.0)))
    # constant section must cover the whole bay; the cone then carries the
    # high rear wing and the fin
    fus.set_cone_start(max(0.52 * l_f, bay_end + 8.0))
    # THE TAIL POST IS AS WIDE AS THE FIN IT CARRIES (audit 2026-08-21, the
    # builder's "vertical stabilizers hanging off slightly from the body" -
    # same defect and same rule as conventional._make_hosts): the default
    # knife-edge post is ~1 mm half-wide while the aft fin's 8% root section
    # runs ~3 mm half-thick, so the buried root poked out of the cone flanks.
    # 1.0 mm of shoulder per side, capped so the cone still tapers.
    find = g.get("fins") or {}
    c_root_fin = float(find.get("c_root_m", 0.08)) * MM
    t_over_c = _max_thickness(_airfoil_pts(g.get("fin_airfoil", "NACA 0008")))
    fus.w_post = _clamp(max(fus.w_post, 0.5 * t_over_c * c_root_fin + 1.0),
                        0.9, 0.55 * fus.W)

    rear = _wing_host(
        g, coords_name=g.get("airfoil", "NACA 0010"), half=half,
        c_root=c_root_r, c_tip=float(g.get("tip_chord_m",
                                           0.7 * c_root_r / MM)) * MM,
        sweep=float(g.get("sweep_le_deg", 0.0)),
        dihedral=float(g.get("dihedral_deg", 0.0)),
        incidence=float(g.get("root_incidence_deg", 0.0)),
        washout=float(g.get("washout_deg", 0.0)),
        fb=(0.5 * fw) / half, x_le=x_le_r,
        z=float(g.get("wing_z_m", 0.5 * fh / MM)) * MM)

    half_f = max(0.5 * float(w2["span_m"]) * MM, 1.0)
    front = _wing_host(
        g, coords_name=w2.get("airfoil", "NACA 0009"), half=half_f,
        c_root=float(w2["c_root_m"]) * MM,
        c_tip=float(w2["c_tip_m"]) * MM,
        sweep=0.0, dihedral=float(w2.get("dihedral_deg", 0.0)),
        incidence=float(w2.get("incidence_deg", 0.0)), washout=0.0,
        fb=(0.5 * fw) / half_f, x_le=float(w2["x_le_m"]) * MM,
        z=float(w2.get("z_m", -0.35 * fh / MM)) * MM)
    return {"fus": fus, "main": rear, "fore": front}


def _build_tandem(design: dict, separate_parts: bool,
                  _probe: dict | None = None
                  ) -> tuple[dict[str, Solid], list[tuple[str, Solid]],
                             dict[str, Any]]:
    g = design["geometry"]
    w2 = g["wing2"]                      # role "front"
    fins = g.get("fins") or {}
    fusd = g.get("fuselage") or {}
    aild = g.get("ailerons") or {}
    warnings: list[str] = []
    hinge_report: dict = {}
    grooves: list[tuple[str, Solid]] = []

    wall = max(float(g.get("wall_mm", 1.2)), 0.6)
    span = float(g["span_m"]) * MM
    half = max(0.5 * span, 1.0)
    c_root_r = float(g["root_chord_m"]) * MM
    fw = float(fusd.get("width_m", 0.07)) * MM
    fh = float(fusd.get("height_m", 0.09)) * MM
    l_f = float(fusd.get("length_m", 0.9 * span / MM)) * MM
    x_le_r = float(g.get("x_le_wing_m", 0.7 * l_f / MM)) * MM
    z_r = float(g.get("wing_z_m", 0.5 * fh / MM)) * MM
    mount_spec = g.get("motor_mount") or {}
    x_nose = _clamp(float(mount_spec.get("x_m", 0.005)) * MM, 2.0, 15.0)
    length_total = float(g.get("length_total_m", (l_f + 5.0) / MM)) * MM
    bayd = fusd.get("bay") or {}
    bay_end = (float(bayd.get("bay_start_m", 0.0))
               + float(bayd.get("bay_length_m", 0.0))) * MM

    half_f = max(0.5 * float(w2["span_m"]) * MM, 1.0)
    x_le_f = float(w2["x_le_m"]) * MM
    c_root_f = float(w2["c_root_m"]) * MM

    hosts = make_hosts(design)
    fus, rear, front = hosts["fus"], hosts["main"], hosts["fore"]

    _progress("bay")
    # ---- bay + hatch FIRST, on the bare fuselage (see _bay_and_hatch) -----
    airframe = fus.solid()
    _bay_out: dict = {}
    raw_tail: dict[str, Solid] = {}    # the loose tail solids (probe seam)
    airframe, bay_mm, lid, _host = _bay_and_hatch(
        airframe, fus, half, bayd, wall, separate_parts, warnings,
        x_max=x_le_r - 12.0,
        battery_x_mm=float(g.get("battery_x_m", 0.0)) * MM or None,
        out=_bay_out)

    _progress("loft")
    # ---- airframe ----------------------------------------------------------
    airframe, _ok = _fuse_in(airframe, _blended(front), "front wing",
                             warnings)

    # the HIGH rear wing may ride above the fuselage crown on the tail cone:
    # a wing-saddle pylon guarantees real overlap instead of a tangent seam
    pylon_info: dict = {"built": False}
    xc0, xc1 = 0.15, 0.70
    x0p = x_le_r + xc0 * c_root_r
    x1p = x_le_r + xc1 * c_root_r
    keel_max = max(rear.keel_z(0.0, xc)
                   for xc in np.linspace(xc0, xc1, 9))
    crown_min = min(fus.crown(x, 0.0) for x in np.linspace(x0p, x1p, 9))
    if keel_max > crown_min - 3.0:
        y_hw = max(min(10.0, 0.75 * min(fus.hw(x)
                                        for x in np.linspace(x0p, x1p, 9))),
                   5.0)
        z_top = min(rear.crown_z(0.0, xc)
                    for xc in np.linspace(xc0, xc1, 9)) - 1.5
        z_bot = crown_min - 3.0
        if z_top > z_bot + 2.0:
            pylon = Solid.makeBox(x1p - x0p, 2.0 * y_hw, z_top - z_bot,
                                  Vector(x0p, -y_hw, z_bot))
            airframe, pyl_ok = _fuse_in(airframe, pylon, "rear-wing pylon",
                                        warnings)
            pylon_info = {"built": bool(pyl_ok),
                          "x_mm": [round(x0p, 1), round(x1p, 1)],
                          "half_width_mm": round(y_hw, 1),
                          "z_mm": [round(z_bot, 1), round(z_top, 1)]}
    airframe, _ok = _fuse_in(airframe, _blended(rear), "rear wing", warnings)

    _progress("fins")
    # centre fin, rooted below the deck (conventional's pattern)
    fin_af = g.get("fin_airfoil", "NACA 0008")
    h_fin = float(fins.get("height_m", 0.10)) * MM
    c_root_v = float(fins.get("c_root_m", 0.08)) * MM
    c_tip_v = float(fins.get("c_tip_m", 0.05)) * MM
    x_le_v = float(fins.get("x_le_m", 0.88 * l_f / MM)) * MM
    z_fin_tip = 0.5 * fh + h_fin
    z_fin_root = fus.post_top - 12.0
    span_fin = max(z_fin_tip - z_fin_root, 0.6 * h_fin)
    x_le_v2, sweep_v = _clamp_aft(x_le_v, c_root_v, span_fin, 30.0,
                                  c_tip_v / max(c_root_v, 1e-6),
                                  x_aft=length_total)
    fin_solid, fin_station = _rounded_surface(
        airfoil=fin_af, span_mm=span_fin, c_root_mm=c_root_v,
        c_tip_mm=c_tip_v, le_root=Vector(x_le_v2, 0.0, z_fin_root),
        sweep_le_deg=sweep_v, dihedral_deg=0.0,
        twist_root_deg=0.0, twist_tip_deg=0.0,
        span_dir=Vector(0, 0, 1), tdir=Vector(0, 1, 0))
    raw_tail["fin"] = fin_solid
    airframe, _ok = _fuse_in(airframe, fin_solid, "fin", warnings)

    if _probe is not None:
        # probe seam (tools_probe_fin_intrusion.py): hand back the
        # pieces exactly as built and stop here - nothing downstream
        # adds material to a tail surface, it only cuts
        _probe.update(airframe=airframe, fins=raw_tail,
                      cavity=_bay_out.get("cavity"),
                      bay_mm=bay_mm, warnings=list(warnings))
        return {"airframe": airframe}, grooves, {"bay": bay_mm,
                                                 "probe": True}

    # ---- tractor motor bores -----------------------------------------------
    airframe, mount_info = _drill_motor(airframe, mount_spec, warnings, l_f)

    _progress("hinges")
    # ---- control surfaces --------------------------------------------------
    elev_frac = float(w2.get("elevator_chord_frac", 0.20))
    ail_in = float(aild.get("inner_frac", 0.55))
    ail_out = float(aild.get("outer_frac", 0.95))
    ail_c = float(aild.get("chord_frac", 0.25))
    rud_c = float(fins.get("rudder_chord_frac", 0.45))
    elev_horn: dict = {}
    rudder_rep: dict = {}
    rudder: Solid | None = None

    if separate_parts:
        # ailerons off the REAR wing (Quickie split: pitch front, roll rear)
        from .geometry import _separate_elevons
        airframe, cut_surfs, rep = _separate_elevons(
            airframe, rear, ail_in, ail_out, ail_c)
        surfaces = {k.replace("elevon", "aileron"): v
                    for k, v in cut_surfs.items()}
        hinge_report.update({k.replace("elevon", "aileron"): v
                             for k, v in rep.items()})
        # front-wing elevator: bounded aft of the front surface
        airframe, elevators, elev_horn = _front_elevator(
            airframe, front, elev_frac, hinge_report, warnings,
            separate_parts, x_aft_max=x_le_f + c_root_f + 30.0)

        # rudder in the rotated frame; the hinge must start ABOVE the high
        # rear wing's root crown or the freeing cut slices the wing
        wing_crown = max(rear.crown_z(0.0, xc)
                         for xc in np.linspace(0.05, 0.95, 12))
        z_r_lo = max(fus.post_top, wing_crown) + 3.0
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
    else:
        surfaces, elevators = {}, {}
        for cutter in _elevon_grooves(rear, g.get("airfoil", "NACA 0010"),
                                      ail_in, ail_out, ail_c):
            grooves.append(("airframe", cutter))
        for cutter in _elevon_grooves(front, w2.get("airfoil", "NACA 0009"),
                                      0.14, 0.95, elev_frac):
            grooves.append(("airframe", cutter))

    # ---- pushrod aims ------------------------------------------------------
    pushrod_aims: dict[str, dict] = {}
    try:
        pushrod_aims["elevator"] = _front_elevator_aim(
            front, elev_frac, x_le_f, c_root_f)
    except Exception as exc:
        warnings.append(f"elevator pushrod aim failed: {exc}")
    if separate_parts and rudder_rep.get("ok"):
        p_lo_r = rudder_rep["hinge_p_lo"]
        pushrod_aims["rudder"] = {
            "side": -1.0,
            "target": (p_lo_r.x + 10.0, -14.0, p_lo_r.z - 8.0),
            "x_stop": x_le_v2 - 25.0}
    elif not separate_parts:
        z_r_lo1 = fus.post_top + 6.0
        f_lo1 = _clamp((z_r_lo1 - z_fin_root) / max(span_fin, 1e-6),
                       0.0, 1.0)
        c1, _tw1, le1 = fin_station(f_lo1)
        pushrod_aims["rudder"] = {
            "side": -1.0,
            "target": (le1.x + _clamp(1.0 - rud_c, 0.4, 0.85) * c1 + 10.0,
                       -14.0, z_r_lo1 - 8.0),
            "x_stop": x_le_v2 - 25.0}

    _progress("servos")
    airframe, horned_ail, servo_report = _install_hardware(
        airframe, rear, fus, surfaces, aild, wall, bay_mm, pushrod_aims,
        separate_parts)
    parts: dict[str, Solid] = {"airframe": airframe}
    if separate_parts:
        surfaces.update(horned_ail)
        parts.update(surfaces)
        parts.update(elevators)
        if rudder is not None and rudder_rep.get("ok"):
            parts["rudder"] = rudder
            servo_report.setdefault("horns", {})["rudder"] = \
                rudder_rep.get("horn", {})
        if "elevator_right" in elevators:
            servo_report.setdefault("horns", {})["elevator_right"] = elev_horn
    if lid is not None:
        parts["hatch_lid"] = lid

    meta_extra = {
        "wing_position": g.get("wing_position", "high"),
        "wing2_mm": {"role": "front", "x_le": x_le_f, "span": 2.0 * half_f,
                     "c_root": c_root_f,
                     "z": float(w2.get("z_m", 0.0)) * MM,
                     "gap": float(w2.get("gap_m", 0.0)) * MM,
                     "elevator_chord_frac": elev_frac},
        "fins": {"arrangement": "center_fin", "x_le_mm": round(x_le_v2, 1),
                 "span_mm": round(span_fin, 1),
                 "sweep_deg": round(sweep_v, 1),
                 "z_root_mm": round(z_fin_root, 1)},
        "pylon": pylon_info,
    }
    meta = _finish(design, parts, fus, half, meta_extra, hinge_report,
                   servo_report, bay_mm, mount_info, warnings)
    return parts, grooves, meta


# ---------------------------------------------------------------------------
# BIPLANE
# ---------------------------------------------------------------------------

def _hosts_biplane(design: dict) -> dict[str, Any]:
    g = design["geometry"]
    w2 = g["wing2"]                      # role "upper"
    taild = g.get("tail") or {}
    fusd = g.get("fuselage") or {}
    span = float(g["span_m"]) * MM
    half = max(0.5 * span, 1.0)
    c_root = float(g["root_chord_m"]) * MM
    fw = float(fusd.get("width_m", 0.08)) * MM
    fh = float(fusd.get("height_m", 0.09)) * MM
    l_f = float(fusd.get("length_m", 0.9 * span / MM)) * MM
    x_wl = float(g.get("x_le_wing_m", 0.25 * l_f / MM)) * MM
    z_l = float(g.get("wing_z_m", -0.35 * fh / MM)) * MM
    x_wu = float(w2.get("x_le_m", x_wl / MM)) * MM
    z_u = float(w2.get("z_m", (z_l + 150.0) / MM)) * MM
    mount_spec = g.get("motor_mount") or {}
    x_nose = _clamp(float(mount_spec.get("x_m", 0.005)) * MM, 2.0, 15.0)

    fus = _FusProfile(l_f=l_f, w=fw, h=fh, x_nose=x_nose,
                      r_plate=float(mount_spec.get("plate_radius_mm", 16.0)))
    fus.set_cone_start(max(0.52 * l_f, x_wl + c_root + 6.0))

    lower = _wing_host(
        g, coords_name=g.get("airfoil", "NACA 2412"), half=half,
        c_root=c_root, c_tip=float(g.get("tip_chord_m",
                                         0.75 * c_root / MM)) * MM,
        sweep=float(g.get("sweep_le_deg", 0.0)),
        dihedral=float(g.get("dihedral_deg", 1.0)),
        incidence=float(g.get("root_incidence_deg", 0.0)),
        washout=float(g.get("washout_deg", 0.0)),
        fb=(0.5 * fw) / half, x_le=x_wl, z=z_l)

    upper = _wing_host(
        g, coords_name=w2.get("airfoil", g.get("airfoil", "NACA 2412")),
        half=max(0.5 * float(w2["span_m"]) * MM, 1.0),
        c_root=float(w2["c_root_m"]) * MM,
        c_tip=float(w2["c_tip_m"]) * MM, sweep=0.0,
        dihedral=float(w2.get("dihedral_deg", 2.0)),
        incidence=float(w2.get("incidence_deg", 0.0)), washout=0.0,
        fb=(0.5 * fw) / half, x_le=x_wu, z=z_u)

    stab = _wing_host(
        g, coords_name=g.get("fin_airfoil", "NACA 0008"),
        half=max(0.5 * float(taild.get("span_h_m", 0.4 * span / MM)) * MM,
                 1.0),
        c_root=float(taild.get("c_root_h_m", 0.1)) * MM,
        c_tip=float(taild.get("c_tip_h_m", 0.07)) * MM, sweep=0.0,
        dihedral=0.0, incidence=float(taild.get("incidence_h_deg", 0.0)),
        washout=0.0, fb=0.06,
        x_le=float(taild.get("x_le_h_m", 0.85 * l_f / MM)) * MM,
        z=0.5 * (fus.post_top + fus.post_bot))
    return {"fus": fus, "main": lower, "upper": upper, "stab": stab}


def _build_biplane(design: dict, separate_parts: bool
                   ) -> tuple[dict[str, Solid], list[tuple[str, Solid]],
                              dict[str, Any]]:
    g = design["geometry"]
    w2 = g["wing2"]                      # role "upper"
    strd = g.get("struts") or {}
    taild = g.get("tail") or {}
    fusd = g.get("fuselage") or {}
    aild = g.get("ailerons") or {}
    warnings: list[str] = []
    hinge_report: dict = {}
    grooves: list[tuple[str, Solid]] = []

    wall = max(float(g.get("wall_mm", 1.2)), 0.6)
    span = float(g["span_m"]) * MM
    half = max(0.5 * span, 1.0)
    c_root = float(g["root_chord_m"]) * MM
    fw = float(fusd.get("width_m", 0.08)) * MM
    fh = float(fusd.get("height_m", 0.09)) * MM
    l_f = float(fusd.get("length_m", 0.9 * span / MM)) * MM
    x_wl = float(g.get("x_le_wing_m", 0.25 * l_f / MM)) * MM
    z_l = float(g.get("wing_z_m", -0.35 * fh / MM)) * MM
    x_wu = float(w2.get("x_le_m", x_wl / MM)) * MM
    z_u = float(w2.get("z_m", (z_l + 150.0) / MM)) * MM
    gap = float(w2.get("gap_m", (z_u - z_l) / MM)) * MM
    mount_spec = g.get("motor_mount") or {}
    x_nose = _clamp(float(mount_spec.get("x_m", 0.005)) * MM, 2.0, 15.0)
    length_total = float(g.get("length_total_m", (l_f + 5.0) / MM)) * MM
    mac = float(w2.get("mac_m", c_root * 0.9 / MM)) * MM

    hosts = make_hosts(design)
    fus, lower, upper, stab = (hosts["fus"], hosts["main"], hosts["upper"],
                               hosts["stab"])

    # ---- strut stations (pure arithmetic - needed before the bay so the
    # aperture can be clamped ahead of the forward cabane) -------------------
    strut_c = float(strd.get("section_chord_m", 0.012)) * MM
    strut_t = float(strd.get("section_thickness_m", 0.005)) * MM
    y_ip = abs(float((strd.get("interplane_y_m") or [0.6 * half / MM])[-1])
               ) * MM
    x_ip = float(strd.get("interplane_x_m", (x_wl + 0.35 * mac) / MM)) * MM
    x_cab = float(strd.get("cabane_x_m", (x_wu + 0.35 * mac) / MM)) * MM
    # cabanes lean in from the fuselage top corners: the recorded +-0.5 w
    # station is the side EDGE of the superellipse where crown == keel, so
    # the feet stand at 0.40 w where the crown still has real material
    y_cab = min(abs(float((strd.get("cabane_y_m") or [0.5 * fw / MM])[-1])
                    ) * MM, 0.40 * fw)
    dx_pair = _clamp(STRUT_PAIR_HALF_FRAC * mac, 12.0, 30.0)
    stagger = x_wl - x_wu                 # upper wing ahead: positive

    struts: list[tuple[str, Solid]] = []
    strut_stations: list[dict] = []
    ail_hinge_x = None
    try:
        # keep interplane feet clear of the aileron hinge line
        xc_a = _clamp(1.0 - float(aild.get("chord_frac", 0.25)), 0.45, 0.90)
        f_ip = _clamp(y_ip / half, 0.05, 0.93)
        sec_ip = lower.section(f_ip)
        ail_hinge_x = sec_ip.le.x + xc_a * sec_ip.chord
    except Exception:
        pass
    for sgn in (1.0, -1.0):
        for dx, tag in ((-dx_pair, "fwd"), (dx_pair, "aft")):
            # interplane pair: lower wing top -> upper wing bottom, both
            # feet clamped onto their own chord and clear of the aileron
            # hinge line (a strut footed on a moving surface is the
            # fin-on-elevon fiasco, DECISIONS.md)
            f_l = _clamp(sgn * y_ip / half, -0.93, 0.93)
            f_u = _clamp(sgn * y_ip / upper.half, -0.93, 0.93)
            sec_l, sec_u = lower.section(f_l), upper.section(f_u)
            x_lo = x_ip + dx
            if ail_hinge_x is not None:
                x_lo = min(x_lo, ail_hinge_x - 0.5 * strut_c - 6.0)
            x_lo = _clamp(x_lo, sec_l.le.x + 0.12 * sec_l.chord,
                          sec_l.le.x + 0.80 * sec_l.chord)
            x_up = _clamp(x_lo - stagger,
                          sec_u.le.x + 0.12 * sec_u.chord,
                          sec_u.le.x + 0.85 * sec_u.chord)
            z_bot = lower.crown_z(f_l, lower.xc_at(f_l, x_lo)) - 2.5
            z_top = upper.keel_z(f_u, upper.xc_at(f_u, x_up)) + 2.5
            struts.append((f"interplane_{tag}",
                           _strut(x_lo, sgn * y_ip, z_bot,
                                  x_up, sgn * y_ip, z_top,
                                  strut_c, strut_t)))
            strut_stations.append({"kind": "interplane", "side": sgn,
                                   "x_mm": round(x_lo, 1),
                                   "y_mm": round(sgn * y_ip, 1)})
            # cabane pair: fuselage crown -> upper wing bottom. The FRONT
            # cabane stands AT the recorded station (0.35 MAC - the front
            # spar) and the rear one a pair-spacing aft, rather than
            # straddling it: a straddled pair puts the forward foot 23 mm
            # ahead of the spar, which is exactly where the hatch aperture
            # ends, and a strut footed on the lid comes off with it.
            f_cu = _clamp(sgn * y_cab / upper.half, -0.93, 0.93)
            sec_cu = upper.section(f_cu)
            x_c = _clamp(x_cab + dx + dx_pair,
                         sec_cu.le.x + 0.12 * sec_cu.chord,
                         sec_cu.le.x + 0.85 * sec_cu.chord)
            z_cb = fus.crown(x_c, sgn * y_cab) - 2.5
            z_ct = upper.keel_z(f_cu, upper.xc_at(f_cu, x_c)) + 2.5
            struts.append((f"cabane_{tag}",
                           _strut(x_c, sgn * y_cab, z_cb,
                                  x_c, sgn * y_cab, z_ct,
                                  strut_c, strut_t)))
            strut_stations.append({"kind": "cabane", "side": sgn,
                                   "x_mm": round(x_c, 1),
                                   "y_mm": round(sgn * y_cab, 1)})

    _progress("bay")
    # ---- bay + hatch FIRST, on the bare fuselage, clamped ahead of the
    # FORWARD cabane strut (a strut footed on the hatch lid would come off
    # with it) ---------------------------------------------------------------
    x_cab_fwd = min(st["x_mm"] for st in strut_stations
                    if st["kind"] == "cabane") if strut_stations else x_cab
    bay_x_max = min(x_cab_fwd - 0.5 * strut_c - 6.0, x_wl - 12.0)
    airframe = fus.solid()
    airframe, bay_mm, lid, _host = _bay_and_hatch(
        airframe, fus, half, fusd.get("bay") or {}, wall, separate_parts,
        warnings, x_max=bay_x_max,
        battery_x_mm=float(g.get("battery_x_m", 0.0)) * MM or None)

    _progress("loft")
    # ---- airframe: fuselage + lower wing + tail ----------------------------
    airframe, _ok = _fuse_in(airframe, _blended(lower), "lower wing",
                             warnings)

    fin_af = g.get("fin_airfoil", "NACA 0008")
    span_h = float(taild.get("span_h_m", 0.4 * span / MM)) * MM
    z_stab = 0.5 * (fus.post_top + fus.post_bot)
    airframe, _ok = _fuse_in(airframe, _blended(stab), "stabilizer",
                             warnings)

    h_v = float(taild.get("height_v_m", 0.1)) * MM
    c_root_v = float(taild.get("c_root_v_m", 0.09)) * MM
    c_tip_v = float(taild.get("c_tip_v_m", 0.06)) * MM
    x_le_v = float(taild.get("x_le_v_m", 0.88 * l_f / MM)) * MM
    z_fin_tip = 0.5 * fh + h_v
    z_fin_root = fus.post_top - 12.0
    span_fin = max(z_fin_tip - z_fin_root, 0.6 * h_v)
    x_le_v2, sweep_v = _clamp_aft(x_le_v, c_root_v, span_fin, 30.0,
                                  c_tip_v / max(c_root_v, 1e-6),
                                  x_aft=length_total)
    fin_solid, fin_station = _rounded_surface(
        airfoil=fin_af, span_mm=span_fin, c_root_mm=c_root_v,
        c_tip_mm=c_tip_v, le_root=Vector(x_le_v2, 0.0, z_fin_root),
        sweep_le_deg=sweep_v, dihedral_deg=0.0,
        twist_root_deg=0.0, twist_tip_deg=0.0,
        span_dir=Vector(0, 0, 1), tdir=Vector(0, 1, 0))
    airframe, _ok = _fuse_in(airframe, fin_solid, "fin", warnings)

    # ---- tractor motor bores -----------------------------------------------
    airframe, mount_info = _drill_motor(airframe, mount_spec, warnings, l_f)

    _progress("hinges")
    # ---- control surfaces --------------------------------------------------
    ail_in = float(aild.get("inner_frac", 0.55))
    ail_out = float(aild.get("outer_frac", 0.95))
    ail_c = float(aild.get("chord_frac", 0.25))
    elev_c = float(taild.get("elevator_chord_frac", 0.30))
    rud_c = float(taild.get("rudder_chord_frac", 0.45))
    rudder_rep: dict = {}
    rudder: Solid | None = None
    elev_horn: dict = {}

    if separate_parts:
        # ailerons on the LOWER wing only, pocket capped below the upper
        # wing (the freeing slab grazes the upper TE at stagger 0.25 c)
        airframe, cut_surfs, rep = _separate_surfaces_bounded(
            airframe, lower, ail_in, ail_out, ail_c,
            z_max=z_l + 0.45 * gap)
        surfaces = {k.replace("elevon", "aileron"): v
                    for k, v in cut_surfs.items()}
        hinge_report.update({k.replace("elevon", "aileron"): v
                             for k, v in rep.items()})

        # elevator: LEFT + RIGHT panels off the stab (conventional pattern)
        from .geometry import _separate_elevons
        airframe, cut_elev, rep_e = _separate_elevons(
            airframe, stab, 0.10, 0.95, elev_c)
        elevators = {k.replace("elevon", "elevator"): v
                     for k, v in cut_elev.items()}
        hinge_report.update({k.replace("elevon", "elevator"): v
                             for k, v in rep_e.items()})

        # rudder in the rotated frame
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

        # elevator horn on the RIGHT panel
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
        surfaces, elevators = {}, {}
        for cutter in _elevon_grooves(lower, g.get("airfoil", "NACA 2412"),
                                      ail_in, ail_out, ail_c):
            grooves.append(("airframe", cutter))
        for cutter in _elevon_grooves(stab, fin_af, 0.10, 0.95, elev_c):
            grooves.append(("airframe", cutter))

    # ---- upper wing + struts: fused as one truss ---------------------------
    # One fuse for the whole cell: base + upper wing + all struts (the upper
    # wing alone would be a second disconnected solid - the struts join it).
    # Deliberately fused AFTER the control-surface separations (they run
    # ~40% faster against the pre-cell solid and cannot touch the upper
    # wing) and BEFORE `_install_hardware` - the interplane feet stand
    # within millimetres of the aileron lead's spanwise corridor, so every
    # conduit cut and open-route existence check must see the FINAL solid,
    # or a fused foot could silently pinch a bore the checks had already
    # blessed (the exact class of loss the doctrine exists for).
    cell = [airframe, _blended(upper)] + [s for _n, s in struts]
    fused = _fuse_all(cell)
    n_struts_built = 0
    if fused.isValid() and len(fused.Solids()) == 1:
        airframe = fused
        n_struts_built = len(struts)
    else:
        warnings.append("upper wing + struts did not fuse into one solid")

    strut_info = {"n_planned": len(struts), "n_built": n_struts_built,
                  "n_total_recorded": int(strd.get("n_total", 8)),
                  "section_mm": [round(strut_c, 1), round(strut_t, 1)],
                  "stations": strut_stations,
                  "print_note": STRUT_PRINT_NOTE}

    # ---- pushrod aims (both aft, conventional's originals) -----------------
    pushrod_aims: dict[str, dict] = {}
    x_le_h = float(taild.get("x_le_h_m", 0.85 * l_f / MM)) * MM
    try:
        p_in_e, p_out_e, _ti, _to = _elevon_hinge_line(
            stab, 1.0, 0.10, min(0.95, _TIP_START - 0.01),
            _clamp(1.0 - elev_c, 0.45, 0.90))
        y_horn_e = p_in_e.y + 0.45 * (p_out_e.y - p_in_e.y)
        z_h_e = 0.5 * (p_in_e.z + p_out_e.z)
        pushrod_aims["elevator"] = {
            "side": +1.0,
            "target": (0.5 * (p_in_e.x + p_out_e.x) + 14.0, y_horn_e,
                       z_h_e - 10.0),
            "x_stop": x_le_h - 25.0}
    except Exception as exc:
        warnings.append(f"elevator pushrod aim failed: {exc}")
    if separate_parts and rudder_rep.get("ok"):
        p_lo_r = rudder_rep["hinge_p_lo"]
        pushrod_aims["rudder"] = {
            "side": -1.0,
            "target": (p_lo_r.x + 10.0, -14.0, p_lo_r.z - 8.0),
            "x_stop": x_le_v2 - 25.0}
    elif not separate_parts:
        z_r_lo1 = max(z_stab + 6.0, fus.post_top) + 3.0
        f_lo1 = _clamp((z_r_lo1 - z_fin_root) / max(span_fin, 1e-6),
                       0.0, 1.0)
        c1, _tw1, le1 = fin_station(f_lo1)
        pushrod_aims["rudder"] = {
            "side": -1.0,
            "target": (le1.x + _clamp(1.0 - rud_c, 0.4, 0.85) * c1 + 10.0,
                       -14.0, z_r_lo1 - 8.0),
            "x_stop": x_le_v2 - 25.0}

    _progress("servos")
    airframe, horned_ail, servo_report = _install_hardware(
        airframe, lower, fus, surfaces, aild, wall, bay_mm, pushrod_aims,
        separate_parts)
    parts: dict[str, Solid] = {"airframe": airframe}
    if separate_parts:
        surfaces.update(horned_ail)
        parts.update(surfaces)
        parts.update(elevators)
        if rudder is not None and rudder_rep.get("ok"):
            parts["rudder"] = rudder
            servo_report.setdefault("horns", {})["rudder"] = \
                rudder_rep.get("horn", {})
        if "elevator_right" in elevators:
            servo_report.setdefault("horns", {})["elevator_right"] = elev_horn
    if lid is not None:
        parts["hatch_lid"] = lid

    meta_extra = {
        "wing_position": "biplane",
        "wing2_mm": {"role": "upper", "x_le": x_wu, "z": z_u,
                     "gap": gap,
                     "stagger": float(w2.get("stagger_m", 0.0)) * MM},
        "struts": strut_info,
        "tail_mm": {"x_le_h": x_le_h, "span_h": span_h,
                    "x_le_v": x_le_v2, "fin_span": span_fin,
                    "fin_sweep_deg": sweep_v, "z_fin_root": z_fin_root},
    }
    meta = _finish(design, parts, fus, half, meta_extra, hinge_report,
                   servo_report, bay_mm, mount_info, warnings)
    return parts, grooves, meta


# ---------------------------------------------------------------------------
# Public API (same contract as cad.conventional - keep both signatures stable)
# ---------------------------------------------------------------------------

_BUILDERS = {
    "canard": _build_canard,
    "tandem": _build_tandem,
    "biplane": _build_biplane,
}

_HOSTS = {
    "canard": _hosts_canard,
    "tandem": _hosts_tandem,
    "biplane": _hosts_biplane,
}


def make_hosts(design: dict) -> dict[str, Any]:
    """The geometry hosts the builder itself works from, keyed by role:
    always `fus` (fuselage profile) and `main` (the aileron-carrying wing),
    plus `fore` (canard foreplane / tandem front wing) or `upper` + `stab`
    (biplane). The builders call THIS, so a test that reads hinge lines or
    skin bands off these hosts cannot drift from the built part - the same
    one-source-of-truth contract as `conventional._make_hosts`."""
    t = str(design.get("airplane_type", ""))
    if t not in _HOSTS:
        raise ValueError(f"cad.multiwing has no hosts for {t!r}")
    return _HOSTS[t](design)


def _build(design: dict, separate_parts: bool = True,
           _probe: dict | None = None
           ) -> tuple[dict[str, Solid], list[tuple[str, Solid]],
                      dict[str, Any]]:
    t = str(design.get("airplane_type", ""))
    if t not in _BUILDERS:
        raise ValueError(f"cad.multiwing cannot build airplane_type={t!r} "
                         f"(knows {sorted(_BUILDERS)})")
    if _probe is not None:
        return _BUILDERS[t](design, separate_parts, _probe=_probe)
    return _BUILDERS[t](design, separate_parts)


def build_design_parts(design: dict) -> tuple[dict[str, Solid],
                                              dict[str, Any]]:
    """The aircraft as SEPARATE NAMED PARTS (mm), each in world position:
    `airframe` (fuselage + all fixed lifting surfaces + fins/struts fused),
    `aileron_left/right`, `elevator_left/right`, `rudder` (tandem/biplane),
    `hatch_lid`, `cg_marker`."""
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
