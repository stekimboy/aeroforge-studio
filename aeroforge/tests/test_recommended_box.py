"""Task 5 of the 2026-08-28 work plan: the advisory minimum size box per type.

Registry presence + shape, the derivation's floor against the hardware it is
derived from, and the two API routes that carry it. No CAD is built - the
TestClient never touches /api/generate or a preview.
"""
from __future__ import annotations

import pytest
from fastapi.testclient import TestClient

from backend.main import app
from backend.physics.config_defs import AIRPLANE_TYPES, RECOMMENDED_MIN_BOX_MM
from backend.physics.twinboom import SERVO_POCKET_LEN_MM, SERVO_SPAN_MM


def test_every_type_has_a_recommended_box():
    assert set(RECOMMENDED_MIN_BOX_MM) == set(AIRPLANE_TYPES)
    for name, t in AIRPLANE_TYPES.items():
        box = t["recommended_min_box_mm"]
        assert isinstance(box, list) and len(box) == 3, name
        assert all(isinstance(v, int) and v > 0 for v in box), name
        assert box == RECOMMENDED_MIN_BOX_MM[name]


def test_recommended_boxes_clear_the_hardware_they_are_derived_from():
    # A box no wider than one servo footprint, or shorter than the pocket's
    # chordwise window plus a bay, could never be a "clean build" hint.
    for name, (length, width, height) in RECOMMENDED_MIN_BOX_MM.items():
        assert width >= 10 * SERVO_SPAN_MM, name          # two servo bays + body
        assert length >= 6 * SERVO_POCKET_LEN_MM, name    # pocket + bay + tail
        assert 100 <= height <= 300, name                 # RC proportions
        assert width >= length, name                      # span is the wide side


@pytest.fixture(scope="module")
def client():
    return TestClient(app)


def _check_types_payload(rows):
    assert {r["name"] for r in rows} == set(AIRPLANE_TYPES)
    for r in rows:
        assert r["recommended_min_box_mm"] == RECOMMENDED_MIN_BOX_MM[r["name"]]
        for key in ("label", "description", "axes"):
            assert key in r


def test_api_types_carries_the_box(client):
    res = client.get("/api/types")
    assert res.status_code == 200
    body = res.json()
    assert set(body) == {"airplane_types"}
    _check_types_payload(body["airplane_types"])


def test_api_options_carries_the_same_box(client):
    res = client.get("/api/options")
    assert res.status_code == 200
    rows = res.json()["airplane_types"]
    _check_types_payload(rows)
    assert rows == client.get("/api/types").json()["airplane_types"]
