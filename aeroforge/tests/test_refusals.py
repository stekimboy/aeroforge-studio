"""Honest refusals reach the viewer (task 2, 2026-08-28).

A feature the CAD build could not make - a bay that would not carve, a
servo pocket the section cannot bury, a wire run that cannot stay inside
the skin - used to live only in the export meta, which the preview job
discarded, so the 3D view showed a solid body and said nothing.
`cadjobs.build_refusals` reads them off the builder's meta generically
(`{ok: false, reason|skipped}` entries plus `warnings` lists), the preview
job writes them beside the part STLs, and `parts.json` carries them as
`refusals` for the banner. No CAD is built here.
"""
from backend import cadjobs


def test_build_refusals_names_every_refused_feature():
    meta = {
        "bay": {"ok": False, "reason": "no rung produced a buildable bay",
                "tried": ["rung 0: x"]},
        "servos": {"bays": {"left": {"ok": False,
                                     "reason": "section too thin"},
                            "right": {"ok": True, "depth_mm": 12.4}},
                   "conduits": {"motor": {"ok": False,
                                          "skipped": "no bay void"}}},
        "warnings": ["fin did not fuse"],
        "hinges": {"warnings": ["hinge module failed: x"]},
    }
    out = cadjobs.build_refusals(meta)
    assert out == ["bay: no rung produced a buildable bay",
                   "left: section too thin",
                   "motor: no bay void",
                   "fin did not fuse",
                   "hinge module failed: x"]


def test_build_refusals_is_empty_on_a_clean_build():
    meta = {"bay": {"ok": True, "rung": "rung 0"},
            "servos": {"bays": {"left": {"ok": True}}},
            "warnings": [], "valid_solid": True}
    assert cadjobs.build_refusals(meta) == []
    assert cadjobs.build_refusals({}) == []
    assert cadjobs.build_refusals(None) == []


def test_parts_manifest_carries_refusals(monkeypatch):
    """`/api/preview/{id}/parts.json` includes the sidecar's list."""
    from fastapi.testclient import TestClient
    from backend import api
    from backend.main import app

    did = "deadbeef0001"
    api._DESIGNS[did] = {"id": did, "airplane_type": "flying_wing",
                         "geometry": {}}
    api._PART_META[did] = [{"name": "airframe", "role": "structure",
                            "label": "airframe", "color": "#888",
                            "role_label": "Structure",
                            "url": f"/api/preview/{did}/part/airframe.stl",
                            "volume_mm3": 1.0}]
    api._PART_REFUSALS[did] = ["bay: no rung produced a buildable bay"]
    try:
        r = TestClient(app).get(f"/api/preview/{did}/parts.json")
        assert r.status_code == 200
        body = r.json()
        assert body["refusals"] == ["bay: no rung produced a buildable bay"]
        assert body["parts"][0]["name"] == "airframe"
    finally:
        api._DESIGNS.pop(did, None)
        api._PART_META.pop(did, None)
        api._PART_REFUSALS.pop(did, None)
