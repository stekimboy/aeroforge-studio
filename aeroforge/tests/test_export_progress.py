"""Export progress bar + ETA plumbing (`backend.progress`), without CAD.

The bar is proportional to REAL time - each stage's share is learned from
the last runs - and the ETA is the median of those runs; without history
the bar still steps per stage and the ETA is honestly None. The API status
shape is checked by monkeypatching the export job so no worker builds.
"""
from __future__ import annotations

import json

import pytest

from backend import progress as P


# ---------------------------------------------------------------------------
# worker-side marks
# ---------------------------------------------------------------------------

def test_report_is_a_noop_without_a_job(tmp_path):
    P.end()                                   # make sure nothing is active
    P.report("loft")                          # must not raise / write
    assert list(tmp_path.iterdir()) == []


def test_marks_round_trip_and_stage_durations(tmp_path):
    pfile = tmp_path / "job.json"
    P.begin(pfile)
    try:
        P.report("loft")
        P.report("bay")
    finally:
        P.end()
    marks = P.read_marks(pfile)
    assert [m["stage"] for m in marks] == ["queued", "loft", "bay", "done"]
    assert all(marks[i]["t"] <= marks[i + 1]["t"]
               for i in range(len(marks) - 1))
    # durations: fixed timestamps, "done" closes the last stage
    fixed = [{"stage": "queued", "t": 0.0}, {"stage": "loft", "t": 2.0},
             {"stage": "bay", "t": 5.0}, {"stage": "done", "t": 9.0}]
    d = P.stage_durations(fixed)
    assert d == {"queued": 2.0, "loft": 3.0, "bay": 4.0}
    # once the sink is closed, reporting is a no-op again
    P.report("servos")
    assert [m["stage"] for m in P.read_marks(pfile)][-1] == "done"


# ---------------------------------------------------------------------------
# timing store
# ---------------------------------------------------------------------------

def test_history_median_of_last_five_and_persistence(tmp_path):
    store = P.TimingStore(tmp_path / "timing.json")
    assert store.expected_s("flying_wing:stl") is None
    for d in (100, 400, 200, 800, 300, 500, 900):
        store.record("flying_wing:stl", d)
    # rolling window: only the last 5 (200, 800, 300, 500, 900) -> median 500
    assert store.expected_s("flying_wing:stl") == 500
    assert len(store.runs("flying_wing:stl")) == 5
    # persisted, reloaded by a fresh store
    again = P.TimingStore(tmp_path / "timing.json")
    assert again.expected_s("flying_wing:stl") == 500
    assert again.summary()["flying_wing:stl"] == {"expected_s": 500, "n": 5}
    raw = json.loads((tmp_path / "timing.json").read_text())
    assert set(raw) == {"flying_wing:stl"}


def test_stage_fractions_are_learned_from_history(tmp_path):
    store = P.TimingStore(tmp_path / "timing.json")
    assert store.stage_fractions("k") is None
    store.record("k", 100, {"loft": 10, "bay": 60, "write": 30})
    store.record("k", 120, {"loft": 12, "bay": 78, "write": 30})
    store.record("k", 90, {"loft": 8, "bay": 52, "write": 30})
    f = store.stage_fractions("k")
    assert set(f) == {"loft", "bay", "write"}
    assert abs(sum(f.values()) - 1.0) < 1e-9
    # medians 10 / 60 / 30 -> 0.1 / 0.6 / 0.3: the bar is proportional to
    # time, NOT one third per stage
    assert f["loft"] == pytest.approx(0.1)
    assert f["bay"] == pytest.approx(0.6)
    assert f["write"] == pytest.approx(0.3)


# ---------------------------------------------------------------------------
# live status
# ---------------------------------------------------------------------------

def _marks(tmp_path, marks):
    p = tmp_path / "job.json"
    p.write_text(json.dumps({"marks": marks}))
    return p


def test_status_without_history_steps_by_stage_with_unknown_eta(tmp_path):
    p = _marks(tmp_path, [{"stage": "queued", "t": 0}, {"stage": "loft", "t": 1},
                          {"stage": "bay", "t": 3}])
    st = P.status_from_file(p, "flying_wing:stl", None, 0.0, now=10.0)
    assert st["stage"] == "bay"
    assert st["stage_label"] == P.STAGE_LABELS["bay"]
    assert st["eta_s"] is None and st["expected_s"] is None
    assert 0.0 < st["progress"] < 1.0
    later = P.status_from_file(
        _marks(tmp_path, [{"stage": "queued", "t": 0}, {"stage": "loft", "t": 1},
                          {"stage": "bay", "t": 3}, {"stage": "write", "t": 8}]),
        "flying_wing:stl", None, 0.0, now=10.0)
    assert later["progress"] > st["progress"]
    assert later["elapsed_s"] == 10.0


def test_status_with_history_is_time_proportional(tmp_path):
    store = P.TimingStore(tmp_path / "timing.json")
    for _ in range(3):
        store.record("flying_wing:stl", 100,
                     {"queued": 0, "loft": 10, "bay": 60, "write": 30})
    # 20 s in: 10 s of loft done (0.10) + 10 of the bay's 60 (0.60 * 1/6)
    p = _marks(tmp_path, [{"stage": "queued", "t": 0}, {"stage": "loft", "t": 0},
                          {"stage": "bay", "t": 10}])
    st = P.status_from_file(p, "flying_wing:stl", store, 0.0, now=20.0)
    assert st["progress"] == pytest.approx(0.20, abs=0.01)
    assert st["eta_s"] == pytest.approx(80.0, abs=0.5)
    assert st["expected_s"] == 100
    # a slow bay stalls at its own boundary rather than lying about "write"
    st2 = P.status_from_file(p, "flying_wing:stl", store, 0.0, now=95.0)
    assert st2["progress"] <= 0.70 + 1e-6
    assert st2["stage"] == "bay"
    # overdue: the ETA is re-estimated from the bar and never negative
    st3 = P.status_from_file(p, "flying_wing:stl", store, 0.0, now=150.0)
    assert st3["eta_s"] is not None and st3["eta_s"] >= 0
    # done -> full bar
    p2 = _marks(tmp_path, [{"stage": "queued", "t": 0}, {"stage": "done", "t": 99}])
    assert P.status_from_file(p2, "flying_wing:stl", store, 0.0,
                              now=100.0)["progress"] == 1.0


def test_status_with_no_progress_file_is_queued(tmp_path):
    st = P.status_from_file(tmp_path / "missing.json", "x:stl", None, 0.0,
                            now=1.0)
    assert st["stage"] == "queued" and st["progress"] == 0.0


# ---------------------------------------------------------------------------
# API shape (export job monkeypatched: no CAD)
# ---------------------------------------------------------------------------

def test_export_status_shape_and_timing_endpoint(tmp_path, monkeypatch):
    import threading
    import time

    from fastapi.testclient import TestClient

    from backend import api, cadjobs
    from backend.main import app

    monkeypatch.setattr(api, "_TIMING", P.TimingStore(tmp_path / "timing.json"))
    monkeypatch.setattr(api, "PROGRESS_DIR", tmp_path / "progress")
    monkeypatch.setattr(api, "_EXPORTS", {})
    monkeypatch.setattr(api, "_EXPORT_JOBS", {})
    api._DESIGNS["fake-design"] = {"id": "fake-design",
                                   "airplane_type": "flying_wing"}
    release = threading.Event()

    def fake_export(design, fmt, progress_path=None):
        P.begin(progress_path)
        P.report("loft")
        release.wait(10)
        P.report("write")
        out = tmp_path / "fake.stl"
        out.write_bytes(b"solid fake\nendsolid fake\n")
        P.end()
        return str(out)

    monkeypatch.setattr(cadjobs, "run_job", lambda fn, *a: fake_export(*a))

    client = TestClient(app, raise_server_exceptions=False)
    r = client.post("/api/export/start",
                    json={"design_id": "fake-design", "format": "stl"})
    assert r.status_code == 200
    st = r.json()
    assert st["status"] == "building"
    for key in ("stage", "stage_label", "progress", "started_at", "eta_s",
                "elapsed_s"):
        assert key in st, key
    assert st["eta_s"] is None                      # no history yet
    # the worker has reported "loft" by the time a poll arrives
    for _ in range(50):
        st = client.get("/api/export/status/fake-design/stl").json()
        if st.get("stage") == "loft":
            break
        time.sleep(0.05)
    assert st["status"] == "building" and st["stage"] == "loft"
    assert 0.0 <= st["progress"] < 1.0
    release.set()
    for _ in range(100):
        st = client.get("/api/export/status/fake-design/stl").json()
        if st["status"] == "ready":
            break
        time.sleep(0.05)
    assert st["status"] == "ready"
    assert st["url"].endswith("/fake-design/stl") and st["bytes"] > 0
    # the finished run was recorded, with its stage seconds
    runs = api._TIMING.runs("flying_wing:stl")
    assert len(runs) == 1 and set(runs[0]["stages"]) >= {"loft", "write"}
    t = client.get("/api/timing").json()
    assert "flying_wing:stl" in t["kinds"]
    assert t["kinds"]["flying_wing:stl"]["n"] == 1
    assert t["stages"] == P.STAGES
    # the progress file is cleaned up after the build
    assert not list((tmp_path / "progress").glob("*.json"))
    api._DESIGNS.pop("fake-design", None)
