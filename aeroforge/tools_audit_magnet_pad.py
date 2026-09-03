"""Audit the aft hatch-magnet pad on an exported STEP (user, 2026-08-27).

The user's complaint: "a long stick extending from the back of the plane
to support that hole/magnet slot". The pad must be a SHORT shelf hanging
from the roof just behind the hatch opening - like the front one - never
a rib running down the cavity extension to its rear wall.

The check reads the delivered geometry, not builder flags: it scans
vertical columns beside the centreline (y = Y_SCAN, clear of the bores) through the centre body and
reports the material z-segments per station. Verdict:
  * PASS: aft of the shelf (x > x_pad_end + margin) every column shows
    only floor skin + roof skin - nothing hanging in the cavity - until
    the cavity ends; and at the magnet station the pad is attached to
    material above it (the shelf reaches the ceiling through its riser).
  * FAIL: a "stick" - material between floor and roof on >= 3
    consecutive stations aft of the shelf.

Usage:
  .venv/Scripts/python.exe tools_audit_magnet_pad.py <file.step> [hatch_lid.stl]
(the aperture's aft wall is read off the lid - the STEP's own lid solid,
 or the preview STL when given)
"""
import sys
import time

from cadquery import importers
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.gp import gp_Pnt
from OCP.TopAbs import TopAbs_IN, TopAbs_ON

Z_STEP = 1.0
X_STEP = 6.0
# scan line: OFF the centreline, because the magnet bores (Ø8.15) are ON it
# and a column through a bore sees the pad as a sliver; 5.5 mm is inside the
# ~15 mm pad and its riser, outside the bore
Y_SCAN = 5.5


def is_solid_at(shape, x, y, z):
    c = BRepClass3d_SolidClassifier(shape, gp_Pnt(x, y, z), 1e-4)
    return c.State() in (TopAbs_IN, TopAbs_ON)


def column(shape, x, z_lo, z_hi, y=Y_SCAN):
    """Material z-segments [(z0, z1), ...] along one vertical line."""
    segs, start, z = [], None, z_lo
    while z <= z_hi + 1e-9:
        m = is_solid_at(shape, x, y, z)
        if m and start is None:
            start = z
        if not m and start is not None:
            segs.append((round(start, 1), round(z - Z_STEP, 1)))
            start = None
        z += Z_STEP
    if start is not None:
        segs.append((round(start, 1), round(z_hi, 1)))
    return segs


def lid_extent(path):
    """x-range of the hatch lid from its preview STL (binary or ASCII)."""
    import struct
    import numpy as np
    b = open(path, "rb").read()
    if b[:5] == b"solid" and b"facet" in b[:400]:
        v = [ln.split()[1:] for ln in b.decode(errors="ignore").splitlines()
             if ln.strip().startswith("vertex")]
        arr = np.array(v, float)
    else:
        n = struct.unpack("<I", b[80:84])[0]
        a = np.frombuffer(b[84:84 + n * 50],
                          dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"),
                                          ("a", "<u2")]))
        arr = a["v"].reshape(-1, 3).astype(float)
    return float(arr[:, 0].min()), float(arr[:, 0].max())


def hollow_gap(segs, min_gap=10.0):
    """Largest air gap between two material segments (0 if none)."""
    best = 0.0
    for (a0, a1), (b0, b1) in zip(segs, segs[1:]):
        best = max(best, b0 - a1)
    return best if best >= min_gap else 0.0


def main():
    path = sys.argv[1]
    lid_path = sys.argv[2] if len(sys.argv) > 2 else None
    t0 = time.time()
    wp = importers.importStep(path)
    solids = wp.solids().vals()
    cands = []
    for s in solids:
        bb = s.BoundingBox()
        if bb.ymin < 0.0 < bb.ymax and bb.xlen > 100.0:
            cands.append((bb.xlen * bb.ylen * bb.zlen, s, bb))
    cands.sort(key=lambda t: -t[0])
    if not cands:
        print("AUDIT ERROR: no centre-body candidate solid")
        sys.exit(2)
    _, body, bb = cands[0]
    print(f"centre body: x [{bb.xmin:.0f},{bb.xmax:.0f}] z [{bb.zmin:.0f},"
          f"{bb.zmax:.0f}]  ({time.time()-t0:.0f}s to import)", flush=True)
    shp = body.wrapped

    if lid_path is None:
        # the lid is the STEP's own hatch_lid solid: the one whose x-range
        # lies inside the body and whose y-range is narrower than the body's
        lids = [s for s in solids if s is not body and s.BoundingBox().xlen > 60
                and abs(s.BoundingBox().ymin + s.BoundingBox().ymax) < 5.0
                and s.BoundingBox().ylen < 0.5 * bb.ylen]
        if not lids:
            print("AUDIT ERROR: no hatch lid solid found (pass its STL)")
            sys.exit(2)
        lb = max(lids, key=lambda s: s.BoundingBox().xlen).BoundingBox()
        lid_x0, lid_x1 = lb.xmin, lb.xmax
    else:
        lid_x0, lid_x1 = lid_extent(lid_path)
    x_ap_end = lid_x1 + 1.2            # aperture wall = lid + clearance
    x_pad_end = x_ap_end + 12.0        # _AFT_RISER_LEN_MM
    print(f"lid x {lid_x0:.1f}..{lid_x1:.1f}; aperture aft wall ~{x_ap_end:.1f}; "
          f"riser expected {x_ap_end + 2:.1f}..{x_pad_end:.1f}", flush=True)

    rows = []
    x = lid_x1 - 40.0
    while x < bb.xmax - 2.0:
        segs = column(shp, x, bb.zmin - 1.0, bb.zmax + 1.0)
        rows.append((x, segs))
        x += X_STEP
    for x, segs in rows:
        gap = hollow_gap(segs)
        hang = gap > 0 and len(segs) >= 3
        tag = "hang" if hang else ("hollow" if gap > 0 else "solid")
        print(f"  x={x:6.1f} {tag:6s} {segs}", flush=True)

    # the cavity extension: hollow columns aft of the aperture
    ext = [(x, s) for x, s in rows if x > x_pad_end + 6.0 and hollow_gap(s) > 0]
    if not ext:
        print("AUDIT N/A: no cavity extension aft of the hatch on this design "
              "(centre-fin cap or refusal) - the old rib rule applies")
        sys.exit(3)
    x_ext_end = max(x for x, _s in ext)
    print(f"cavity extension aft of the pad: to x={x_ext_end:.0f}", flush=True)
    run, worst = 0, 0
    for x, s in ext:
        run = run + 1 if len(s) >= 3 else 0
        worst = max(worst, run)
    stick = worst >= 2

    # the riser: a thick top segment (riser + roof as one) just behind the
    # aperture wall, at fine x resolution
    attached, thick = False, 0.0
    xr = x_ap_end + 2.5
    while xr <= x_pad_end - 0.5:
        segs = column(shp, xr, bb.zmin - 1.0, bb.zmax + 1.0)
        if segs:
            t = segs[-1][1] - segs[-1][0]
            thick = max(thick, t)
            if t >= 8.0:
                attached = True
        xr += 1.5
    print(f"riser: thickest roof-side material behind the aperture "
          f"{thick:.1f} mm -> attached={attached}")
    if stick:
        print("AUDIT FAIL: a rib/stick runs aft of the magnet shelf")
        sys.exit(1)
    if not attached:
        print("AUDIT FAIL: no riser - the aft shelf does not reach the roof")
        sys.exit(1)
    print("AUDIT PASS: short aft shelf on a roof riser, nothing hanging aft")


if __name__ == "__main__":
    main()
