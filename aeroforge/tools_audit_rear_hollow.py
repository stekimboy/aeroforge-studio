"""Cross-section auditor for the rear hollow (plan.md, step 2).

Classifies a y-z point grid against the ACTUAL solid at stations along
the rear body and reports, per station, every enclosed interior air
region. Verdicts:

  * TWO-BOXES signature (FAIL): >= 2 enclosed regions parked at +/-y
    with NO region covering the centreline.
  * CHANNEL signature (PASS): an enclosed region containing y ~ 0 on
    several consecutive stations, its dims printed.

Usage:
  .venv/Scripts/python.exe tools_audit_rear_hollow.py <file.step> [label]

Works on the exported STEP itself - no builder flags are consulted.
"""
import sys
import time

from cadquery import importers
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.gp import gp_Pnt
from OCP.TopAbs import TopAbs_IN, TopAbs_ON

Y_MAX = 62.0     # scan half-width, mm (covers bay + channel, avoids
                 # the wing-panel servo tubes far outboard)
Y_STEP = 2.0
Z_STEP = 1.2


def is_solid_at(shape, x, y, z):
    c = BRepClass3d_SolidClassifier(shape, gp_Pnt(x, y, z), 1e-4)
    return c.State() in (TopAbs_IN, TopAbs_ON)


def station_regions(shape, x, z_lo, z_hi):
    """Enclosed interior air cells at one x station -> connected regions."""
    ys = []
    y = -Y_MAX
    while y <= Y_MAX + 1e-9:
        ys.append(round(y, 3))
        y += Y_STEP
    zs = []
    z = z_lo
    while z <= z_hi + 1e-9:
        zs.append(round(z, 3))
        z += Z_STEP
    cells = set()
    for yi, yv in enumerate(ys):
        col = [is_solid_at(shape, x, yv, zv) for zv in zs]
        # enclosed air: a False run with True somewhere below AND above
        i = 0
        n = len(col)
        while i < n:
            if not col[i]:
                j = i
                while j < n and not col[j]:
                    j += 1
                below = any(col[:i])
                above = any(col[j:])
                if below and above:
                    for k in range(i, j):
                        cells.add((yi, k))
                i = j
            else:
                i += 1
    # flood fill (4-neighbour)
    regions = []
    seen = set()
    for cell in cells:
        if cell in seen:
            continue
        stack, comp = [cell], []
        seen.add(cell)
        while stack:
            cy, cz = stack.pop()
            comp.append((cy, cz))
            for dy, dz in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                q = (cy + dy, cz + dz)
                if q in cells and q not in seen:
                    seen.add(q)
                    stack.append(q)
        y_vals = [ys[c[0]] for c in comp]
        z_vals = [zs[c[1]] for c in comp]
        regions.append({
            "cells": len(comp),
            "y": (min(y_vals), max(y_vals)),
            "z": (min(z_vals), max(z_vals)),
            # CELL-based, not bbox-based: a U-shaped region straddling
            # the centreline in bbox but solid AT the centreline must
            # not read as central (reviewer finding #3)
            "covers_y0": any(abs(v) <= 2.0 for v in y_vals),
        })
    regions.sort(key=lambda r: -r["cells"])
    return [r for r in regions if r["cells"] >= 6]     # ignore slivers


def main():
    path = sys.argv[1]
    label = sys.argv[2] if len(sys.argv) > 2 else path
    t0 = time.time()
    print(f"importing {path} ...", flush=True)
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
    print(f"centre body: x [{bb.xmin:.0f},{bb.xmax:.0f}] "
          f"y [{bb.ymin:.0f},{bb.ymax:.0f}] z [{bb.zmin:.0f},{bb.zmax:.0f}]"
          f"  ({time.time()-t0:.0f}s to import)", flush=True)
    shp = body.wrapped

    L = bb.xmax - bb.xmin
    boxes_hits, channel_hits = [], []
    zs_probe = []
    zp = bb.zmin + 0.6
    while zp <= bb.zmax - 0.6:
        zs_probe.append(zp)
        zp += Z_STEP
    x = bb.xmin + 0.45 * L
    while x <= bb.xmin + 0.985 * L:
        # the hatch opening: the lid is a separate assembly part, so at
        # the bay the centreline reads ROOFLESS air. "Enclosed" cells at
        # +/-y there are the open bay's own shoulders under the intact
        # crown edge - not boxes (reviewer finding #7). Skip the
        # box-judgement on such stations.
        def top_solid(yv):
            t = None
            for z in zs_probe:
                if is_solid_at(shp, x, yv, z):
                    t = z
            return t

        top0 = top_solid(0.0)
        flanks = [v for v in (top_solid(40.0), top_solid(-40.0),
                              top_solid(58.0), top_solid(-58.0))
                  if v is not None]
        # the hatch opening: the centreline's roof (the lid) is a
        # separate assembly part, so the highest solid at y=0 sits far
        # below the intact flank crown. (Air above the crown is outside
        # the body and must NOT count as "roofless" - first attempt's
        # bug: it skipped every station.)
        open_top = (top0 is not None and flanks
                    and top0 < max(flanks) - 6.0)
        if open_top:
            print(f"x={x:7.1f}: hatch opening (roofless centreline) - "
                  "box judgement skipped", flush=True)
            x += 6.0
            continue
        regs = station_regions(shp, x, bb.zmin + 0.6, bb.zmax - 0.6)
        if regs:
            desc = "; ".join(
                f"y[{r['y'][0]:.0f},{r['y'][1]:.0f}] "
                f"z[{r['z'][0]:.1f},{r['z'][1]:.1f}]"
                f"{' <y0>' if r['covers_y0'] else ''}"
                for r in regs[:4])
            print(f"x={x:7.1f}: {len(regs)} region(s): {desc}", flush=True)
            offset = [r for r in regs
                      if not r["covers_y0"]
                      and abs(0.5 * (r["y"][0] + r["y"][1])) > 8.0]
            central = [r for r in regs if r["covers_y0"]]
            # reviewer findings #1/#2: ANY persistent offset pocket is
            # a box - one-sided counts, and coexisting with a central
            # region does not excuse it. (A servo tube crossing shows
            # at <= 2 adjacent stations only; boxes persist.)
            if offset:
                boxes_hits.append(x)
            if central:
                channel_hits.append((x, central[0]))
        x += 6.0

    print("\n================ VERDICT ================")
    persistent = [x for x in boxes_hits
                  if sum(1 for v in boxes_hits if abs(v - x) <= 13.0) >= 3]
    if persistent:
        print(f"FAIL - BOX/pocket signature (persistent offset air) at "
              f"x = {', '.join(f'{v:.0f}' for v in sorted(set(persistent)))}")
        sys.exit(1)
    if boxes_hits:
        print(f"note: transient offset air at x = "
              f"{', '.join(f'{v:.0f}' for v in boxes_hits)} (<= 2 "
              "stations - consistent with a servo tube crossing, not "
              "a box)")
    if len(channel_hits) >= 3:
        xs = [c[0] for c in channel_hits]
        r = channel_hits[len(channel_hits) // 2][1]
        print(f"PASS [{label}] - one central hollow from x~{min(xs):.0f} "
              f"to x~{max(xs):.0f}; mid-station "
              f"y [{r['y'][0]:.0f},{r['y'][1]:.0f}] "
              f"z [{r['z'][0]:.1f},{r['z'][1]:.1f}]")
        sys.exit(0)
    print("INCONCLUSIVE - no enclosed rear air found (no channel, no "
          "boxes). If the design needed a rear run, that is a FAIL.")
    sys.exit(3)


if __name__ == "__main__":
    main()
