"""The on-disk BREP cache (backend.cad.brepcache, speed pass 2026-08-28).

Cheap by design: box/sphere stand-ins for the parts, no airframe build. The
byte-identity of real exports through the cache is proven by the speed-pass
run recorded in docs/PERF_NOTES.md, not here.
"""
import cadquery as cq
import pytest

from backend.cad import brepcache as bc


@pytest.fixture
def cache_dir(tmp_path, monkeypatch):
    monkeypatch.setattr(bc, "CACHE_DIR", tmp_path / "brep")
    monkeypatch.delenv("AEROFORGE_BREP_CACHE", raising=False)
    return tmp_path / "brep"


DESIGN = {"id": "abc123", "airplane_type": "flying_wing",
          "geometry": {"span_m": 1.2, "sweep_le_deg": 22.0},
          "stability": {"static_margin": 0.08},
          "notes": ["a note"], "guidance": [{"title": "x"}]}


def test_key_ignores_per_generate_fields_but_not_geometry():
    k = bc.design_key(DESIGN)
    assert k == bc.design_key({**DESIGN, "id": "other", "notes": [],
                               "guidance": []})
    assert k != bc.design_key({**DESIGN, "geometry": {"span_m": 1.3}})
    assert k != bc.design_key({**DESIGN, "stability": {"static_margin": 0.1}})


def test_parts_round_trip_preserves_order_names_and_meta(cache_dir):
    box = cq.Workplane().box(10, 20, 30).val()
    sph = cq.Workplane().sphere(5).val()
    parts = {"airframe": box, "hatch_lid": sph}
    assert not bc.has_parts(DESIGN)
    assert bc.load_parts(DESIGN) is None
    bc.save_parts(DESIGN, parts, {"bay": {"x0_mm": 12.5, "band": (1, 2)}})
    assert bc.has_parts(DESIGN)
    loaded, meta = bc.load_parts(DESIGN)
    assert list(loaded) == ["airframe", "hatch_lid"]
    assert meta == {"bay": {"x0_mm": 12.5, "band": [1, 2]}}
    for name in parts:
        assert loaded[name].isValid()
        assert abs(loaded[name].Volume() - parts[name].Volume()) < 1e-6
    # the one-piece solid is a separate entry
    assert not bc.has_solid(DESIGN)
    bc.save_solid(DESIGN, box, {})
    solid, _ = bc.load_solid(DESIGN)
    assert abs(solid.Volume() - box.Volume()) < 1e-6


def test_disabled_by_env(cache_dir, monkeypatch):
    monkeypatch.setenv("AEROFORGE_BREP_CACHE", "0")
    bc.save_solid(DESIGN, cq.Workplane().box(1, 1, 1).val(), {})
    assert not bc.has_solid(DESIGN)
    assert bc.load_solid(DESIGN) is None


def test_prune_keeps_the_newest_entries(cache_dir, monkeypatch):
    monkeypatch.setattr(bc, "MAX_ENTRIES", 3)
    box = cq.Workplane().box(1, 1, 1).val()
    for i in range(6):
        bc.save_solid({**DESIGN, "geometry": {"i": i}}, box, {})
    kept = [d for d in cache_dir.iterdir() if d.is_dir()]
    assert len(kept) == 3
    assert bc.has_solid({**DESIGN, "geometry": {"i": 5}})
    assert not bc.has_solid({**DESIGN, "geometry": {"i": 0}})
