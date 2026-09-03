"""CAD builder for TWIN-BOOM pusher designs (v3): pod + wing + booms + H-tail.

Lives BESIDE the conventional path exactly the way conventional lives beside
the flying wing (V3_PLAN.md): `geometry.build_design_parts` /
`build_design_solid` dispatch here when `design["airplane_type"] ==
"twin_boom"` and nothing in this module is imported on any other type's run.

Frame: x aft from the nose datum (mm), y right, z up, z = 0 on the pod
mid-height / thrust line (matches `physics.twinboom`'s envelope math).
All CAD in mm.

TOPOLOGY. A short POD (rounded nose, constant bay section, aft blend to the
flat FIREWALL face that carries the pusher motor - the mirror of the
conventional tractor nose, reusing its superellipse profile machinery), a
high wing on the pod deck, two round printed BOOM FAIRINGS at the physics'
`geometry.booms.y_m` stations with the carbon-tube socket (`socket_id_mm`)
bored the full length - the tube IS the boom's stiffness ([RT3 s.5.2]), the
print is a fairing around it - an H-TAIL: one stab spanning boom to boom,
its tips buried in the fairings, and one fin per boom. The prop spins between
the booms behind the pod; physics guarantees the spacing clears the disk and
this module asserts it again on the built stations.

HARDWARE - all of it REUSED from the v1/v2 modules, none re-implemented:
  * ailerons: `geometry._separate_elevons` + `hinges.py` captive pins, wing
    servo pockets via `servos.servo_bay`, world-aligned horns via
    `servos.control_horn`, `servos.linkage` reports. Each servo lead runs
    in ONE STRAIGHT round Ø8.25 pipe (builder's spec, round 5): a single
    straight extrusion from the lead grommet - a straight rod passes end
    to end; angled is allowed, bent is not. The nose bay sits AHEAD of a
    shoulder wing on a narrow pod, so on most designs no straight line
    reaches the void directly; the tube then ends over the pod in the
    wing's thick corridor (~30% root chord, where the old riser stood)
    and the pod is hollowed out further - a WELL under the tube mouth and
    a GALLERY forward under the deck - so the compartment extends aft to
    meet the tube and the rod emerges into bay air.
  * elevator: LEFT + RIGHT panels split from the stab by the same
    `_separate_elevons` machinery (joined with a torsion wire at the bench,
    same as conventional); the RIGHT panel carries the horn.
  * rudders: one per boom fin, cut/hinged/horned by conventional's
    `_split_rudder` with its v3 `y_fin`/`y_guard` parameters - the pair is
    translated onto the centreline, rotated, hinged in the proven frame and
    put back.
  * elevator/rudder servos sit IN THE POD BAY (physics puts their mass
    there); each gets a straight round Ø8.25 trumpeted SNAKE GUIDE from
    inside the bay void out through the pod flank toward its boom root - the
    pushrod/snake continues externally along the boom to the tail horn,
    which is exactly how the Skyhunter class is rigged. Both cuts are
    mesh-gated and their bores classified open.
  * motor leads run INSIDE the pod (physics dict rule - NO belly hole on
    this type): one straight round Ø8.25 trumpeted pipe from behind the
    firewall (co-axial with the shaft bore the leads enter through) forward
    into the measured bay void, cut through `conduits.cut_conduits` and
    proven with `conduits.route_is_open`.

EQUIPMENT BAY. `hatch.build_bay` unchanged, presented the pod through
conventional's `_FusBayHost` duck type. High wing: the bay is clamped to end
ahead of the wing LE (the wing owns the pod deck over its chord), which is
also what keeps the whole nose free for the pack and camera.

BOOM SOCKETS. Each fairing is bored `socket_id_mm` end to end, LAST (so no
later boolean can lose the bore), and every bore is existence-checked by
classification at seven stations - `isValid()` says a shape is well-formed,
never that a cut did its job (DECISIONS.md, four proven instances).
"""
from __future__ import annotations

import math
from typing import Any

import numpy as np
from cadquery import Solid, Vector

from . import conduits as _cd
from . import hatch as _hatch
from . import servos as _sv
from ..progress import report as _progress
from .conventional import (
    PIPE_D_MM,
    _ConvWing,
    _FusBayHost,
    _FusProfile,
    _is_air,
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
    _rounded_surface,
    _Section,
    _separate_elevons,
    _smoothstep,
    _tessellates_cleanly,
    _TIP_START,
    _void_z_band,
    fuse_feature as _fuse_feature,
)

BOOM_BURY_MM = 2.5        # how far the fairing crown is buried into the wing
FIN_BURY_MM = 4.0         # how far a fin root reaches down into its boom
STAB_TIP_OVERLAP_MM = 4.0  # stab semi-span past the boom centreline
PROP_TIP_MARGIN_MM = 2.0  # fairing inner face must clear the disk by this


# ---------------------------------------------------------------------------
# The pod: conventional's fuselage profile machinery, mirrored for a pusher
# ---------------------------------------------------------------------------

class _PodProfile(_FusProfile):
    """Pusher pod as three analytic superellipse profiles of x: a rounded
    NOSE cap, a constant bay section, and an aft blend down to the flat
    FIREWALL face - the parent's tractor logic mirrored end for end. The
    parent's `wire`/`solid`/`crown`/`keel` machinery is inherited unchanged;
    only the blend law differs, so the loft discipline (three axial segments
    sharing boundary wires, evenly spaced stations) is exactly the proven
    one."""

    def __init__(self, *, l_f: float, w: float, h: float,
                 r_plate: float) -> None:
        super().__init__(l_f=l_f, w=w, h=h, x_nose=1.0, r_plate=r_plate)
        # nose tip: a small rounded face (the parent a0/b0 pair stays the
        # FIREWALL face and is used by the aft blend below)
        self.a_n = max(0.30 * self.W, 2.5)
        self.b_n = max(0.30 * self.T, 2.5)
        # short nose cap keeps the constant bay section long - the nose IS
        # the payload volume on this type
        self.x_full = self.x_nose + _clamp(0.22 * l_f, 24.0, 0.30 * l_f)
        self.x_cone = l_f - _clamp(0.30 * l_f, max(1.4 * r_plate, 24.0),
                                   0.42 * l_f)
        if self.x_cone < self.x_full + 10.0:
            self.x_cone = self.x_full + 10.0

    def blend(self, x: float) -> tuple[float, float, float]:
        x = _clamp(x, self.x_nose, self.l_f)
        if x <= self.x_full:
            s = _smoothstep((x - self.x_nose)
                            / max(self.x_full - self.x_nose, 1e-6))
            return (self.a_n + (self.W - self.a_n) * s,
                    self.b_n + (self.T - self.b_n) * s,
                    -self.b_n + (self.B + self.b_n) * s)
        if x < self.x_cone:
            return self.W, self.T, self.B
        s = _smoothstep((x - self.x_cone) / max(self.l_f - self.x_cone, 1e-6))
        return (self.W + (self.a0 - self.W) * s,
                self.T + (self.b0 - self.T) * s,
                self.B + (-self.b0 - self.B) * s)


def _mount_cutters_pusher(spec: dict) -> tuple[list[tuple[Solid, Vector]],
                                               dict]:
    """Screw-hole and shaft-bore cutters for the PUSHER firewall (the pod's
    flat aft face), each paired with a probe point that must classify as AIR
    after the cut - conventional's `_mount_cutters` with the axis reversed.
    The shaft bore doubles as the motor-lead pass-through into the pod."""
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

    lead = 6.0
    screw_depth = _clamp(3.0 * t_plate, 9.0, 16.0)
    if n == 4:  # square pattern: holes on the corners, diagonal = bolt circle
        s = r_bolt / math.sqrt(2.0)
        offsets = [(s, s), (s, -s), (-s, s), (-s, -s)]
    else:
        offsets = [(r_bolt * math.cos(2 * math.pi * i / n),
                    r_bolt * math.sin(2 * math.pi * i / n)) for i in range(n)]
    axis = Vector(-1, 0, 0)                     # drilled from behind the face
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
            "note": ("the pod's flat aft face is the pusher firewall; the "
                     "motor leads enter through the shaft bore and run "
                     "enclosed to the bay - no belly hole on this type")}
    return cutters, info


# ---------------------------------------------------------------------------
# Aileron wiring: ONE straight pipe, grommet -> nose bay (no turns)
# ---------------------------------------------------------------------------

def _aileron_chain_tb(wing: _ConvWing, pod: _PodProfile,
                      bay: "_sv.ServoBay", wall: float, bay_mm: dict,
                      hinge_xc: float = 0.75,
                      span_mm: tuple[float, float] = (0.0, 1e9)
                      ) -> tuple[list[tuple[str, Solid]], dict]:
    """ONE straight pipe carrying one aileron lead to the NOSE bay.

    Builder's spec (round 5, 2026-08-24): the old three-piece chain
    (in-wing corridor, vertical wing-saddle riser, forward pod run) had
    two right-angle turns; the lead now runs in a single straight
    extrusion from the lead grommet - a straight rod passes end to end.

    Two candidates, tried in order:

    1. DIRECT: the tube descends through the wing root and pod interior
       and pierces the void's aft bulkhead. On most twin-booms this is
       geometrically impossible - the nose bay sits wholly ahead of a
       shoulder wing on a narrow pod, so any line steep enough to reach
       the void either breaks the wing skin or leaves the aircraft
       between the wing LE and the pod (both measured) - but where it
       fits, it is the cleanest.
    2. GALLERY: the tube stays in the wing's own corridor and ends over
       the pod just aft of the wing leading edge; the pod is then
       hollowed out further - the builder's own remedy - with a WELL
       under the tube mouth and a GALLERY running forward under the deck
       into the void, so the compartment simply extends aft to meet the
       straight tube. The rod emerges into bay air; the wire never bends
       inside a channel.

    Returns ([(name, cutter)], info). The probe point and the run path
    are recorded so the caller can classify everything open after the
    cut.
    """
    bm = bay_mm or {}
    if bm.get("x0_mm") is None or bm.get("x1_mm") is None:
        return [], {"ok": False,
                    "reason": "no bay void - nowhere for the lead to go"}
    x1 = float(bm["x1_mm"])
    zf, zc_ = bm.get("z_floor_aft_mm"), bm.get("z_ceil_aft_mm")
    if zf is None or zc_ is None or float(zc_) - float(zf) < 10.0:
        return [], {"ok": False,
                    "reason": "no measured void z band to aim the run at"}
    zf, zc_ = float(zf), float(zc_)
    r_m = 0.5 * PIPE_D_MM + 0.6
    start = bay.cable_exit
    side = 1.0 if (start.y or 1.0) >= 0 else -1.0
    guard = {"hinge_xc": float(hinge_xc), "margin_xc": 0.04,
             "span_lo_mm": float(span_mm[0]),
             "span_hi_mm": float(span_mm[1])}
    base = {"hinge_guard": guard, "start_overshoot_mm": 6.0,
            "end_overshoot_mm": 8.0, "max_start_x_drift_mm": 2.2}

    # ---- candidate 1: direct into the void's aft bulkhead -----------------
    z_lo = zf + max(r_m, 5.0)
    z_hi = zc_ - max(r_m, 5.0)
    if z_hi < z_lo:
        z_lo = z_hi = 0.5 * (zf + zc_)
    y_v = side * _clamp(0.5 * float(bm.get("width_mm", 30.0)) - 8.0,
                        3.0, 24.0)
    pipe, info = _cd.straight_conduit(
        [wing, pod], start=start, end_xy=(x1 - 8.0, y_v),
        end_z_band=(z_lo, z_hi), wall=wall,
        params=dict(base, prefer_z=0.5 * (z_lo + z_hi)))
    info["kind"] = "servo"
    info["shape"] = "straight"
    info["entry_mode"] = "direct"
    if pipe is not None:
        path = _cd.path_vectors(info)
        u = path[-1] - path[0]
        length = u.Length
        u = u.multiply(1.0 / max(length, 1e-9))
        if abs(u.x) > 1e-6:
            t_p = _clamp(((x1 + 3.0) - path[0].x) / u.x, 2.0, length - 2.0)
        else:
            t_p = length - 6.0
        info["_riser_probe"] = path[0] + u.multiply(t_p)
        info["_run_path"] = path
        return [("wing", pipe)], info
    direct_refusal = {"skipped": info.get("skipped"),
                      "tightest": info.get("tightest")}

    # ---- candidate 2: end over the pod, extend the bay aft to meet it -----
    # The aim ladder starts at 30% of the root chord - the wing's thickest
    # corridor, and where the old riser stood - because a tube aimed near
    # the LEADING EDGE dies in the knife-thin nose sections (measured:
    # 7.1 mm of section against the 10.9 the bore needs).
    sec_r = wing.section(0.02)
    x_le = float(sec_r.le.x)
    c_r = float(sec_r.chord)
    aims = [x_le + 0.30 * c_r, x_le + 0.40 * c_r, x_le + 0.20 * c_r,
            min(start.x, pod.x_cone - 4.0)]
    pipe, info, x_e, y_e = None, {}, 0.0, 0.0
    for x_try in aims:
        y_try = side * _clamp(0.35 * pod.W, 6.0,
                              max(pod.hw(x_try) - 9.0, 6.0))
        f_e = _clamp(y_try / wing.half, -0.98, 0.98)
        xc_e = float(wing.xc_at(f_e, x_try))
        if not (0.02 <= xc_e <= 0.9):
            continue
        wk = float(wing.keel_z(f_e, xc_e))
        wc = float(wing.crown_z(f_e, xc_e))
        pipe, info = _cd.straight_conduit(
            [wing, pod], start=start, end_xy=(x_try, y_try),
            end_z_band=(wk - 1.0, wc - wall - r_m), wall=wall,
            params=dict(base, prefer_z=0.5 * (wk + wc),
                        end_overshoot_mm=4.0))
        x_e, y_e = x_try, y_try
        if pipe is not None:
            break
    info = dict(info or {})
    info["kind"] = "servo"
    info["shape"] = "straight"
    info["entry_mode"] = "gallery"
    info["direct_refusal"] = direct_refusal
    if pipe is None:
        info.setdefault("skipped",
                        "no wing section over the pod to end the tube in")
        return [], info
    z_e = float(info["end_z_mm"])

    # the WELL under the tube mouth (interior: wing above, pod below) and
    # the GALLERY forward under the deck into the void. Each cap is checked
    # against the analytic skins before anything is cut.
    yw_a, yw_b = sorted((y_e - side * 9.0, y_e + side * 7.0))
    xw_a = x_e - 7.0
    xw_b = x_e + 7.0
    wc_min = 1e9
    for x in (xw_a, xw_b):
        for y in (yw_a, yw_b):
            f = _clamp(y / wing.half, -0.98, 0.98)
            xcx = float(wing.xc_at(f, x))
            if 0.015 <= xcx <= 0.985:
                wc_min = min(wc_min, float(wing.crown_z(f, xcx)))
    z_top_well = z_e + r_m + 1.0
    if z_top_well > wc_min - wall - 0.3:
        info["skipped"] = (f"well top z={z_top_well:.1f} would thin the "
                          f"wing crown (min {wc_min:.1f}) below "
                          f"{wall:.1f} mm")
        return [], info

    xg_a, xg_b = x1 - 6.0, xw_a + 4.0
    xg_b = max(xg_b, xw_a + 4.0)
    gal_top = 1e9
    for x in (xg_a, 0.5 * (xg_a + xg_b), xg_b):
        for y in (yw_a, yw_b):
            if not pod.contains_plan(x, y, 0.5):
                info["skipped"] = ("gallery leaves the pod plan at "
                                   f"x={x:.0f}, y={y:.0f}")
                return [], info
            gal_top = min(gal_top, float(pod.crown(x, y)) - wall - 0.3)
    g_bot = zf + 2.0
    if gal_top < g_bot + 8.0:
        info["skipped"] = (f"gallery band [{g_bot:.1f}, {gal_top:.1f}] "
                          "is too shallow under the deck")
        return [], info

    well = Solid.makeBox(xw_b - xw_a, yw_b - yw_a,
                         z_top_well - (gal_top - 6.0),
                         Vector(xw_a, yw_a, gal_top - 6.0))
    gallery = Solid.makeBox(xg_b - xg_a, yw_b - yw_a, gal_top - g_bot,
                            Vector(xg_a, yw_a, g_bot))
    info["bay_expansion"] = {
        "ok": True, "kind": "pod_gallery",
        "well_x_mm": [round(xw_a, 1), round(xw_b, 1)],
        "gallery_x_mm": [round(xg_a, 1), round(xg_b, 1)],
        "y_mm": [round(yw_a, 1), round(yw_b, 1)],
        "well_z_mm": [round(gal_top - 6.0, 1), round(z_top_well, 1)],
        "gallery_z_mm": [round(g_bot, 1), round(gal_top, 1)],
        "what": ("the nose bay extended aft under the deck to meet the "
                 "straight tube - the hollowed-out enlargement that makes "
                 "a straight run possible on this layout")}
    path = _cd.path_vectors(info)
    # probe in the gallery's former material, opened only by these cuts
    info["_riser_probe"] = Vector(x1 + 3.0, y_e,
                                  0.5 * (g_bot + gal_top))
    info["_run_path"] = path
    return [("wing", pipe), ("well", well), ("gal", gallery)], info


# ---------------------------------------------------------------------------
# Hosts
# ---------------------------------------------------------------------------

def _make_hosts(g: dict) -> tuple[_PodProfile, _ConvWing, _ConvWing, dict]:
    """(pod profile, wing host, stab host, derived dims) from a twin-boom
    design's `geometry` dict. One source of truth: the builder AND the tests
    read hinge lines and skin bands off these hosts."""
    fusd = g.get("fuselage") or {}
    taild = g.get("tail") or {}
    boomd = g.get("booms") or {}
    sect = boomd.get("section_mm") or {}

    wall = max(float(g.get("wall_mm", 1.2)), 0.6)
    span = float(g["span_m"]) * MM
    half = max(0.5 * span, 1.0)
    c_root = float(g["root_chord_m"]) * MM
    c_tip = float(g.get("tip_chord_m") or c_root * 0.8) * MM
    l_f = float(fusd.get("length_m", 0.25)) * MM
    fw = float(fusd.get("width_m", 0.065)) * MM
    fh = float(fusd.get("height_m", 0.075)) * MM
    x_w = float(fusd.get("x_wing_le_m", 0.5 * l_f / MM)) * MM
    mount_spec = g.get("motor_mount") or {}
    r_plate = float(mount_spec.get("plate_radius_mm", 16.0))
    length_total = float(g.get("length_total_m", l_f / MM + 0.3)) * MM

    # z = 0 on the pod mid-height / thrust line; high wing on the pod deck
    # (physics.twinboom: wing_z = 0.5 * pod_h, fixed - [RT3 s.5.1] practice)
    wing_z = 0.5 * fh

    pod = _PodProfile(l_f=l_f, w=fw, h=fh, r_plate=r_plate)

    wing = _ConvWing(
        coords=_airfoil_pts(g.get("airfoil", "NACA 2412")),
        half=half, c_root=c_root, c_tip=max(c_tip, 0.06 * c_root),
        sweep_deg=float(g.get("sweep_le_deg", 0.0)),
        dihedral_deg=float(g.get("dihedral_deg", 2.0)),
        incidence=float(g.get("root_incidence_deg", 0.0)),
        washout=float(g.get("washout_deg", 0.0)),
        fb=_clamp((0.5 * fw) / half, 0.04, 0.50),
        depth_scale=1.0, chord_scale=1.0, nose_round=0.6, crown_frac=0.62,
        x_le_root=x_w, z_mount=wing_z)

    # boom stations. The physics ends the boom exactly at the tail's
    # trailing edge, which puts the boom's flat end face COINCIDENT with the
    # fin root TE and the stab TE - and a near-coincident boolean silently
    # produces nothing (DECISIONS.md: the centre-fin empty-intersection
    # lesson, reproduced here on the first build: the fin fuse returned two
    # disconnected solids). The built boom therefore runs 3.5 mm past the
    # nominal end (inside the physics' own 5 mm length margin) and the fin
    # root TE is inset 2.5 mm ahead of the nominal end, so every meeting is
    # a real intersection.
    y_boom = abs(float((boomd.get("y_m") or [0.09])[-1])) * MM
    x_boom0 = float(boomd.get("x_start_m", (x_w + 0.1 * c_root) / MM)) * MM
    l_boom_nom = float(boomd.get("length_m", 0.4)) * MM
    l_boom = l_boom_nom + 3.5
    r_fair = 0.5 * float((sect.get("fairing_mm") or [14.0, 14.0])[0])
    socket_id = float(sect.get("socket_id_mm", 8.3))
    # the fairing crown is buried into the wing's lower skin at the boom
    # station so the boom and the wing fuse into one piece of material
    f_b = _clamp(y_boom / half, 0.0, 0.95)
    keel_b = wing.keel_z(f_b, 0.30)
    z_boom = keel_b + BOOM_BURY_MM - r_fair

    # H-tail: rectangular stab spanning boom to boom, tips buried in the
    # fairings; z on the boom centreline
    fin_af = g.get("fin_airfoil", "NACA 0008")
    half_s = y_boom + STAB_TIP_OVERLAP_MM
    stab = _ConvWing(
        coords=_airfoil_pts(fin_af),
        half=half_s,
        c_root=float(taild.get("c_root_h_m", 0.12)) * MM,
        c_tip=float(taild.get("c_tip_h_m",
                              taild.get("c_root_h_m", 0.12))) * MM,
        sweep_deg=0.0, dihedral_deg=0.0,
        incidence=float(taild.get("incidence_h_deg", 0.0)),
        washout=0.0, fb=0.06, depth_scale=1.0, chord_scale=1.0,
        nose_round=0.6, crown_frac=0.62,
        x_le_root=float(taild.get("x_le_h_m", 0.75 * length_total / MM)) * MM,
        z_mount=z_boom)

    # fins: rooted in the booms, tips at the physics height anchor
    h_v = float(taild.get("height_v_m", 0.1)) * MM
    z_fin_root = z_boom - FIN_BURY_MM
    z_fin_tip = wing_z + h_v                    # = the physics z_top anchor
    span_fin = max(z_fin_tip - z_fin_root, 0.6 * h_v)
    c_root_v = float(taild.get("c_root_v_m", 0.09)) * MM
    c_tip_v = float(taild.get("c_tip_v_m", 0.055)) * MM
    x_le_v = float(taild.get("x_le_v_m", 0.8 * length_total / MM)) * MM
    # inset the fin root TE ahead of the boom's nominal end face (see the
    # boom-overhang note above - coincident TEs make empty booleans)
    x_le_v = min(x_le_v, x_boom0 + l_boom_nom - c_root_v - 2.5)
    x_le_v2, sweep_v = _clamp_aft(x_le_v, c_root_v, span_fin, 30.0,
                                  c_tip_v / max(c_root_v, 1e-6),
                                  x_aft=length_total)

    dims = dict(wall=wall, span=span, half=half, c_root=c_root, l_f=l_f,
                fw=fw, fh=fh, x_w=x_w, wing_z=wing_z, mount_spec=mount_spec,
                length_total=length_total, y_boom=y_boom, x_boom0=x_boom0,
                l_boom=l_boom, l_boom_nom=l_boom_nom, r_fair=r_fair,
                socket_id=socket_id,
                z_boom=z_boom, fin_af=fin_af, h_v=h_v,
                z_fin_root=z_fin_root, span_fin=span_fin,
                c_root_v=c_root_v, c_tip_v=c_tip_v, x_le_v2=x_le_v2,
                sweep_v=sweep_v, half_s=half_s,
                prop_d=float(boomd.get("prop_diameter_est_m", 0.2)) * MM,
                spacing=float(boomd.get("spacing_m", 2 * y_boom / MM)) * MM)
    return pod, wing, stab, dims


# ---------------------------------------------------------------------------
# Aileron servo installation (mirrors conventional._install_hardware)
# ---------------------------------------------------------------------------

def _install_hardware(airframe: Solid, wing: _ConvWing, pod: _PodProfile,
                      surfaces: dict[str, Solid], ail: dict, wall: float,
                      bay_mm: dict, snake_aims: dict[str, dict]
                      ) -> tuple[Solid, dict[str, Solid], dict]:
    """Aileron servo pockets + horns + linkage, the straight wire runs,
    the two tail snake guides and the motor-lead pipe. Same structure as
    conventional's `_install_hardware`: collect cutters, one boolean per
    family, mesh-gate once, verify existence."""
    report: dict = {"bays": {}, "horns": {}, "conduits": {}, "pushrods": {}}
    out: dict[str, Solid] = {}
    riser_probes: dict[str, Vector] = {}
    run_paths: dict[str, list[Vector]] = {}
    snake_paths: dict[str, list[Vector]] = {}

    inner = _clamp(float(ail.get("inner_frac", 0.55)),
                   max(wing.fb, 0.10), 0.85)
    outer = _clamp(float(ail.get("outer_frac", 0.95)),
                   inner + 0.10, _TIP_START - 0.01)
    xc = _clamp(1.0 - float(ail.get("chord_frac", 0.25)), 0.45, 0.90)
    y_arm_frac = _clamp(inner + 0.10, wing.fb + 0.06, outer - 0.10)

    bay_cutters: list[tuple[str, Any]] = []
    conduit_cutters: list[tuple[str, Solid | None]] = []

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
                    chain, i_srv = _aileron_chain_tb(
                        wing, pod, b, wall, bay_mm, hinge_xc=xc,
                        span_mm=(inner * wing.half, outer * wing.half))
                    probe = i_srv.pop("_riser_probe", None)
                    path = i_srv.pop("_run_path", None)
                    for piece, cutter in chain:
                        conduit_cutters.append((f"servo_{nm}_{piece}",
                                                cutter))
                    if probe is not None:
                        riser_probes[nm] = probe
                    if path:
                        run_paths[nm] = path
                    report["conduits"][f"servo_{nm}"] = i_srv
            else:
                report["bays"][nm] = {
                    "ok": False,
                    "reason": "bay cut would not mesh; skin left intact"}

    # ---- tail snake guides + the motor-lead pipe ---------------------------
    bm = bay_mm or {}
    if bm.get("x0_mm") is not None and bm.get("x1_mm") is not None:
        x0, x1 = float(bm["x0_mm"]), float(bm["x1_mm"])
        x_s = _clamp(x1 - 12.0, x0 + 6.0, x1 - 6.0)
        zf, zc_ = bm.get("z_floor_aft_mm"), bm.get("z_ceil_aft_mm")
        if zf is not None and zc_ is not None and float(zc_) - float(zf) > 12.0:
            z_s = float(zf) + _clamp(8.0, 4.0,
                                     0.5 * (float(zc_) - float(zf)))
            z_run = _clamp(0.5 * (float(zf) + float(zc_)),
                           float(zf) + 6.0, float(zc_) - 6.0)
        else:
            z_s = 0.0
            z_run = 0.0
        for key, aim in snake_aims.items():
            if key.startswith("_"):
                continue
            side = float(aim["side"])
            start = Vector(x_s, side * 7.0, z_s)
            end = Vector(*aim["target"])
            cutter, path, info = _straight_pipe(start, end)
            info.update(kind="snake_guide", drives=key,
                        note=("straight guide pipe from the pod bay out "
                              "through the pod flank toward the boom root; "
                              "the pushrod/snake continues externally along "
                              "the boom to the tail horn - Skyhunter-class "
                              "rigging"))
            conduit_cutters.append((f"pushrod_{key}", cutter))
            snake_paths[key] = path
            report["pushrods"][key] = info

        # the motor-lead pipe: behind the firewall -> forward into the void
        mx = float((snake_aims.get("_motor") or {}).get("x_face_mm", 0.0))
        if mx > 0.0:
            m_start = Vector(mx + 4.0, 0.0, 0.0)
            m_end = Vector(x1 - 8.0, 0.0, z_run)
            m_cut, m_path, m_info = _straight_pipe(m_start, m_end)
            m_info["what"] = ("motor-lead pipe INSIDE the pod: enters "
                              "through the firewall shaft bore, opens into "
                              "the bay void - no belly hole on this type")
            conduit_cutters.append(("motor", m_cut))
            snake_paths["_motor"] = m_path
            report["conduits"]["motor"] = m_info
    else:
        for key in snake_aims:
            if key.startswith("_"):
                continue
            report["pushrods"][key] = {
                "ok": False, "applied": False,
                "reason": "no bay void - the tail servos have nowhere to sit"}

    # ---- cut everything at once, mesh-gated, then prove the bores ----------
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
                elif k.startswith("servo_") and k.count("_") >= 2:
                    base = k.rsplit("_", 1)[0]      # servo_<name>
                    piece = k.rsplit("_", 1)[1]
                    prev = report["conduits"].setdefault(base, {})
                    if isinstance(v, dict):
                        prev.setdefault("pieces", {})[piece] = v
                        if piece == "wing":
                            prev.update({kk: vv for kk, vv in v.items()
                                         if kk not in prev})
                else:
                    prev = report["conduits"].get(k)
                    if isinstance(prev, dict) and isinstance(v, dict):
                        prev.update(v)
                    else:
                        report["conduits"][k] = v
        except Exception as exc:
            report["conduits"]["error"] = str(exc)

        # existence: the straight bore must be air through the bulkhead it
        # pierces AND open along its whole centreline
        for nm, probe in riser_probes.items():
            ci = report["conduits"].get(f"servo_{nm}")
            if not isinstance(ci, dict):
                continue
            pieces = ci.get("pieces") or {}
            applied = bool((pieces.get("wing") or {}).get("applied"))
            open_wall = applied and _is_air(airframe, probe)
            open_run = False
            if applied and nm in run_paths:
                try:
                    r = _cd.route_is_open(airframe, run_paths[nm])
                    open_run = bool(r.get("open"))
                except Exception:
                    open_run = False
            ci["applied"] = bool(applied)
            # legacy keys, one straight run: both report the same bore
            ci["riser_open"] = bool(open_wall)
            ci["run_open"] = bool(open_run)
            ci["into_bay_open"] = bool(open_wall and open_run)
            if not (applied and open_wall and open_run):
                ci["applied"] = False
                ci["why"] = ("straight run blocked - the lead cannot "
                             "reach the bay")
        for key, path in snake_paths.items():
            rep = (report["conduits"].get("motor") if key == "_motor"
                   else report["pushrods"].get(key))
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
# Boom socket bores: cut LAST, each PROVEN open end to end
# ---------------------------------------------------------------------------

def _bore_sockets(airframe: Solid, dims: dict,
                  warnings: list[str]) -> tuple[Solid, dict]:
    """Bore the carbon-tube socket through each boom fairing, full length,
    and prove every bore open by classification at seven stations. Bored
    LAST so no downstream boolean can lose them; `.copy()` retry on failure
    (the bay-cut lesson)."""
    r = 0.5 * float(dims["socket_id"])
    x0 = dims["x_boom0"] - 6.0
    length = dims["l_boom"] + 12.0
    z_b = dims["z_boom"]
    out: dict = {"socket_id_mm": dims["socket_id"], "booms": {}}
    for sgn, name in ((1.0, "right"), (-1.0, "left")):
        y_b = sgn * dims["y_boom"]
        cyl = Solid.makeCylinder(r, length, Vector(x0, y_b, z_b),
                                 Vector(1, 0, 0))
        cut = None
        for attempt in range(2):
            try:
                base = airframe.copy() if attempt else airframe
                tool = cyl.copy() if attempt else cyl
                trial = _heal(base.cut(tool))
                if (trial.isValid() and len(trial.Solids()) == 1
                        and _tessellates_cleanly(trial, tol=0.3,
                                                 min_ratio=0.985)):
                    cut = trial
                    break
            except Exception:
                continue
        probes = []
        if cut is not None:
            xs = np.linspace(dims["x_boom0"] + 3.0,
                             dims["x_boom0"] + dims["l_boom"] - 3.0, 7)
            probes = [bool(_is_air(cut, Vector(float(x), y_b, z_b)))
                      for x in xs]
        if cut is not None and all(probes) and len(probes) >= 7:
            airframe = cut
            out["booms"][name] = {
                "open": True, "stations_checked": len(probes),
                "x0_mm": round(dims["x_boom0"], 1),
                "x1_mm": round(dims["x_boom0"] + dims["l_boom"], 1),
                "y_mm": round(y_b, 1), "z_mm": round(z_b, 1)}
        else:
            out["booms"][name] = {"open": False, "probes": probes}
            warnings.append(
                f"{name} boom socket bore did not verify open - drill the "
                f"Ø{dims['socket_id']:.1f} mm tube socket by hand along the "
                "fairing axis")
    return airframe, out


# ---------------------------------------------------------------------------
# Core build
# ---------------------------------------------------------------------------

def _build(design: dict, separate_parts: bool = True,
           _probe: dict | None = None
           ) -> tuple[dict[str, Solid], list[tuple[str, Solid]],
                      dict[str, Any]]:
    g = design["geometry"]
    fusd = g.get("fuselage") or {}
    taild = g.get("tail") or {}
    aild = g.get("ailerons") or {}
    warnings: list[str] = []

    pod, wing, stab, dims = _make_hosts(g)
    wall, half = dims["wall"], dims["half"]
    l_f, fw, fh, x_w = dims["l_f"], dims["fw"], dims["fh"], dims["x_w"]
    y_boom, x_boom0, l_boom = dims["y_boom"], dims["x_boom0"], dims["l_boom"]
    r_fair, z_boom = dims["r_fair"], dims["z_boom"]
    z_fin_root, span_fin = dims["z_fin_root"], dims["span_fin"]
    fin_af, mount_spec = dims["fin_af"], dims["mount_spec"]
    length_total, half_s = dims["length_total"], dims["half_s"]

    parts: dict[str, Solid] = {}
    grooves: list[tuple[str, Solid]] = []
    hinge_report: dict = {}
    servo_report: dict = {}

    # ---- prop clearance: guaranteed by physics, asserted anyway ------------
    prop_r = 0.5 * dims["prop_d"]
    inner_face = y_boom - r_fair
    prop_ok = (dims["spacing"] >= 1.15 * dims["prop_d"] - 1e-6
               and inner_face >= prop_r + PROP_TIP_MARGIN_MM - 1e-6)
    if not prop_ok:
        warnings.append(
            f"prop disc clearance FAILED on the built stations: fairing "
            f"inner face at {inner_face:.1f} mm vs prop tip at "
            f"{prop_r:.1f} mm (+{PROP_TIP_MARGIN_MM:.0f} margin) - the "
            "physics guarantee did not survive the fairing width")

    _progress("loft")
    # ---- airframe: pod + wing + booms + stab + fins, one solid -------------
    airframe = pod.solid()

    def _fuse_in(base: Solid, add: Solid, what: str) -> Solid:
        """Verified feature fuse - see `geometry.fuse_feature`. A boom or fin
        can vanish into a boolean that reports one valid solid; the gate
        classifies rather than trusting the flags."""
        return _fuse_feature(base, add, what, warnings)[0]

    from .geometry import _blended_airframe
    airframe = _fuse_in(airframe, _blended_airframe(wing), "wing")
    for sgn, nm in ((1.0, "right boom"), (-1.0, "left boom")):
        boom = Solid.makeCylinder(r_fair, l_boom,
                                  Vector(x_boom0, sgn * y_boom, z_boom),
                                  Vector(1, 0, 0))
        airframe = _fuse_in(airframe, boom, nm)
    raw_tail: dict[str, Solid] = {}    # the loose tail solids (probe seam)
    raw_tail["stab"] = _blended_airframe(stab)
    airframe = _fuse_in(airframe, raw_tail["stab"], "stabilizer")

    _progress("fins")
    fin_stations = {}
    for sgn, nm in ((1.0, "right"), (-1.0, "left")):
        fin_solid, fin_station = _rounded_surface(
            airfoil=fin_af, span_mm=span_fin, c_root_mm=dims["c_root_v"],
            c_tip_mm=dims["c_tip_v"],
            le_root=Vector(dims["x_le_v2"], sgn * y_boom, z_fin_root),
            sweep_le_deg=dims["sweep_v"], dihedral_deg=0.0,
            twist_root_deg=0.0, twist_tip_deg=0.0,
            span_dir=Vector(0, 0, 1), tdir=Vector(0, 1, 0))
        raw_tail[f"fin_{nm}"] = fin_solid
        airframe = _fuse_in(airframe, fin_solid, f"{nm} fin")
        fin_stations[nm] = fin_station

    _progress("bay")
    # ---- equipment bay + hatch (nose bay - the type's whole point) ---------
    bayd = fusd.get("bay") or {}
    bay_start = float(bayd.get("bay_start_m", 0.06 * l_f / MM)) * MM
    bay_len = float(bayd.get("bay_length_m", 0.5 * l_f / MM)) * MM
    bay_hw = 0.5 * float(bayd.get("bay_width_m", (fw - 2.4) / MM)) * MM
    bay_x_max = x_w - 12.0        # high wing owns the deck over its chord

    bay_host = _FusBayHost(pod, half)
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

    # ---- pusher motor holes: drilled into the firewall, each PROVEN open ---
    cutters, mount_info = _mount_cutters_pusher(mount_spec)
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

    _progress("hinges")
    # ---- control surfaces --------------------------------------------------
    elev_c = float(taild.get("elevator_chord_frac", 0.28))
    rud_c = float(taild.get("rudder_chord_frac", 0.40))
    # the elevator's outer ends stop clear of the boom fairings
    outer_e = _clamp((y_boom - r_fair - 3.0) / half_s, 0.30,
                     _TIP_START - 0.01)
    stab_crown = stab.crown_z(0.3, 0.4)
    rudder_reps: dict[str, dict] = {}

    if separate_parts:
        # ailerons (outer wing)
        airframe, cut_surfs, rep = _separate_elevons(
            airframe, wing, float(aild.get("inner_frac", 0.55)),
            float(aild.get("outer_frac", 0.95)),
            float(aild.get("chord_frac", 0.25)))
        surfaces = {k.replace("elevon", "aileron"): v
                    for k, v in cut_surfs.items()}
        hinge_report.update({k.replace("elevon", "aileron"): v
                             for k, v in rep.items()})

        # elevator: LEFT + RIGHT panels between the booms
        airframe, cut_elev, rep_e = _separate_elevons(
            airframe, stab, 0.10, outer_e, elev_c)
        elevators = {k.replace("elevon", "elevator"): v
                     for k, v in cut_elev.items()}
        hinge_report.update({k.replace("elevon", "elevator"): v
                             for k, v in rep_e.items()})

        # rudders: one per boom fin, in the proven rotated frame
        coords_fin = _airfoil_pts(fin_af)
        xc_h_r = _clamp(1.0 - rud_c, 0.40, 0.85)
        t_frac_r = (_foil_surf_t(coords_fin, xc_h_r, True)
                    - _foil_surf_t(coords_fin, xc_h_r, False))
        z_r_lo = max(z_boom + r_fair + 2.0, stab_crown + 4.0)
        z_r_hi = z_fin_root + 0.87 * span_fin
        rudders: dict[str, Solid] = {}
        for sgn, nm in ((1.0, "right"), (-1.0, "left")):
            airframe, rud, r_rep = _split_rudder(
                airframe, fin_stations[nm], span_fin, z_r_lo, z_r_hi,
                rud_c, z_fin_root, t_frac_r,
                y_fin=sgn * y_boom, y_guard=r_fair + 6.0)
            rudder_reps[nm] = r_rep
            if r_rep.get("ok") and rud is not None:
                rudders[f"rudder_{nm}"] = rud
                hinge_report[f"rudder_{nm}"] = r_rep.get("hinges", {})
            else:
                warnings.append(f"{nm} rudder not separated: "
                                + str(r_rep.get("reason")))

        # elevator horn on the RIGHT panel (left joined with torsion wire)
        elev_horn: dict = {}
        if "elevator_right" in elevators:
            try:
                p_in, p_out, t_in, t_out = _elevon_hinge_line(
                    stab, 1.0, 0.10, outer_e,
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
        surfaces, elevators, rudders = {}, {}, {}
        for cutter in _elevon_grooves(wing, g.get("airfoil", "NACA 2412"),
                                      float(aild.get("inner_frac", 0.55)),
                                      float(aild.get("outer_frac", 0.95)),
                                      float(aild.get("chord_frac", 0.25))):
            grooves.append(("airframe", cutter))
        for cutter in _elevon_grooves(stab, fin_af, 0.10, outer_e, elev_c):
            grooves.append(("airframe", cutter))

    # ---- snake-guide aims (tail servos live in the pod bay) ----------------
    z_exit = _clamp(0.30 * z_boom, 2.0, 14.0)
    snake_aims = {
        "elevator": {"side": +1.0,
                     "target": (x_w + 0.55 * dims["c_root"],
                                0.5 * fw + 26.0, z_exit)},
        "rudder": {"side": -1.0,
                   "target": (x_w + 0.55 * dims["c_root"],
                              -(0.5 * fw + 26.0), z_exit)},
        "_motor": {"x_face_mm": float(mount_spec.get("x_m", 0.0)) * MM},
    }

    # ---- servos, horns, wire chains ----------------------------------------
    _progress("servos")
    airframe, horned_ail, servo_report = _install_hardware(
        airframe, wing, pod, surfaces, aild, wall, bay_mm, snake_aims)
    if separate_parts:
        surfaces.update(horned_ail)
        if "elevator_right" in elevators:
            servo_report.setdefault("horns", {})["elevator_right"] = elev_horn
        for nm, r_rep in rudder_reps.items():
            if r_rep.get("ok"):
                servo_report.setdefault("horns", {})[f"rudder_{nm}"] = \
                    r_rep.get("horn", {})

    # ---- boom socket bores: LAST, proven -----------------------------------
    airframe, socket_report = _bore_sockets(airframe, dims, warnings)

    parts["airframe"] = airframe
    if lid is not None:
        parts["hatch_lid"] = lid
    if separate_parts:
        parts.update(surfaces)
        parts.update(elevators)
        parts.update(rudders)

    # ---- CG marker ---------------------------------------------------------
    st = design.get("stability", {})
    x_cg = float(st.get("x_cg_m", 0.0)) * MM
    if x_cg <= 0.0:
        x_cg = x_w + 0.30 * dims["c_root"]
    parts["cg_marker"] = _cg_marker(x_cg, pod.keel(x_cg, 0.0) + 1.0,
                                    scale=half / 550.0)

    meta = {
        "units": "mm",
        "airplane_type": "twin_boom",
        "planform": design.get("planform", "fpv"),
        "x_cg_mm": float(st.get("x_cg_m", 0.0)) * MM,
        "x_np_mm": float(st.get("x_np_m", 0.0)) * MM,
        "mac_mm": float(st.get("mac_m", 0.0)) * MM,
        "x_le_mac_mm": float(st.get("x_le_mac_m", 0.0)) * MM,
        "y_mac_mm": float(st.get("y_mac_m", 0.0)) * MM,
        "cg_pct_mac": st.get("cg_pct_mac", 0.0),
        "static_margin": st.get("static_margin", 0.0),
        "span_mm": dims["span"],
        "length_mm": length_total,
        "height_mm": float(g.get("height_total_m", 0.0)) * MM,
        "root_chord_mm": dims["c_root"],
        "fuselage_mm": {"length": l_f, "width": fw, "height": fh,
                        "x_wing_le": x_w, "wing_z": dims["wing_z"]},
        "booms_mm": {"y": y_boom, "x0": x_boom0, "length": l_boom,
                     "fairing_r": r_fair, "z": z_boom,
                     "spacing": dims["spacing"],
                     "prop_diameter": dims["prop_d"],
                     "prop_clearance_ok": bool(prop_ok),
                     "inner_face_mm": round(inner_face, 1),
                     "prop_tip_mm": round(prop_r, 1)},
        "tail_mm": {"x_le_h": stab.x_offset, "span_h": 2.0 * half_s,
                    "x_le_v": dims["x_le_v2"], "fin_span": span_fin,
                    "fin_sweep_deg": dims["sweep_v"],
                    "z_fin_root": z_fin_root, "z_boom": z_boom},
        "sockets": _jsonable(socket_report),
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
    return parts, grooves, meta


# ---------------------------------------------------------------------------
# Public API (geometry.py dispatches here - keep both signatures stable)
# ---------------------------------------------------------------------------

def build_design_parts(design: dict) -> tuple[dict[str, Solid], dict[str, Any]]:
    """The twin-boom aircraft as SEPARATE NAMED PARTS (mm), each in world
    position: `airframe`, `aileron_left/right`, `elevator_left/right`,
    `rudder_left/right`, `hatch_lid`, `cg_marker`."""
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
