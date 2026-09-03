"""The bay ladder on the configuration that broke it (task 2, 2026-08-28).

bwb planform, fpv_cruiser, 18 m/s, centre fin, the user's 675 x 675 x 500
box: `hatch.build_bay` reported "the compartment pieces would not join into
one void" / "the bay cut left the compartment SOLID" on EVERY rung and the
design shipped with a solid centre body, while the same planform at the
650 box carved at rung 4.

Root cause (measured on the dumped pieces): `core` and `below` are two
independent lofts sharing the same floor curve; once the rear extension
lengthens `below`, OCC's fuse cannot recognise the two floor faces as one -
core+below came back INVALID with less volume than `below` alone. Lifting
the core's floor 1 mm (`hatch._CORE_FLOOR_LIFT_MM`, retry-only) takes the
coincidence away; the cut result then carried one untriangulated face that
a BRep round-trip of the result meshes completely (`mesh_round_tripped`).

Probe-level (loft + boss + bay, no hinges/servos/conduits): ~4 min.
Proven in the geometry, not in flags: points along the published cavity
band classify as AIR in the opened airframe, including the extension.
"""
import pytest
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.gp import gp_Pnt
from OCP.TopAbs import TopAbs_OUT

from backend.cad import geometry as geo
from backend.cad import hatch
from backend.physics.optimizer import optimize_design
from backend.schemas import GenerateRequest

BOX_675 = {"length_mm": 675, "width_mm": 675, "height_mm": 500}


def _bay_for(vstab: str):
    body = {"airplane_type": "flying_wing", "mission": "fpv_cruiser",
            "cruise_speed_ms": 18.0, "payload_g": 0.0, "box": BOX_675,
            "n_motors": 1, "build_method": "3d_printed", "material": "lw_pla",
            "mount_screws": 4, "mount_screw_d_mm": 3.2,
            "mount_bolt_circle_mm": 16.0, "planform": "bwb", "vstab": vstab}
    d = optimize_design(GenerateRequest(**body).to_optimizer_input())
    g = d["geometry"]
    wing = geo._wing_from_design(g)
    airframe = geo._blended_airframe(wing)
    wall = max(float(g.get("wall_mm", 1.2)), 0.6)
    boss, _c, x_nac = geo._motor_mount(wing, g.get("motor_mount") or {}, wall)
    if boss is not None:
        merged = geo._heal(airframe.fuse(boss))
        if merged.isValid() and len(merged.Solids()) == 1:
            airframe = merged
        else:
            boss, x_nac = None, 0.0
    mm = g.get("motor_mount") or {}
    pusher = boss is not None and str(mm.get("type", "")) == "pusher"
    req = geo.bay_request(g, wing, g.get("body") or {}, wall, boss, x_nac,
                          (x_nac - 4.0) if pusher else None)
    res = hatch.build_bay(wing, airframe=airframe, magnets=True,
                          canopy=False, one_piece=False, **req)
    return d, res


@pytest.fixture(scope="module")
def bwb675():
    return _bay_for("center_fin")


def test_bwb_675_centre_fin_bay_carves_with_extension(bwb675):
    d, res = bwb675
    bm = dict(res.bay_mm or {})
    assert d["planform"] == "bwb"
    assert res.ok, f"bay refused: {bm.get('reason')} tried={bm.get('tried')}"
    assert bm.get("rung") == "rung 0 (len x1.00, width x1.00)", bm.get("rung")
    assert (bm.get("cavity_extended_mm") or 0.0) >= 12.0, bm
    assert bm.get("hatch_x1_mm") and bm["x1_mm"] > bm["hatch_x1_mm"] + 12.0


def test_bwb_675_cavity_is_air_in_the_opened_airframe(bwb675):
    _d, res = bwb675
    bm = dict(res.bay_mm or {})
    cl = BRepClass3d_SolidClassifier(res.airframe.wrapped)
    solid_pts = []
    band = bm["cavity_stations_mm"]
    assert len(band) >= 8
    for x, hw, zlo, zhi in band[1:-1]:
        if zhi - zlo < 6.0:
            continue
        for y in (0.0, 0.5 * hw, -0.5 * hw):
            z = 0.5 * (zlo + zhi)
            cl.Perform(gp_Pnt(float(x), float(y), float(z)), 1e-6)
            if cl.State() != TopAbs_OUT:
                solid_pts.append((round(x, 1), round(y, 1), round(z, 1)))
    assert not solid_pts, (
        f"{len(solid_pts)} points of the published cavity band are still "
        f"material in the opened airframe: {solid_pts[:6]}")
    # the extension specifically: stations past the hatch must be air too
    aft = [row for row in band if row[0] > bm["hatch_x1_mm"] + 2.0]
    assert len(aft) >= 3, band
