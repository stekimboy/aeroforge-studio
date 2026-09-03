"""No tail surface stands inside the equipment-bay cavity (the 2026-08-28 work plan
task 1). Runs `tools_probe_fin_intrusion.py`'s cheap path - the builder up
to its probe seam (loft + tail + bay, no hinges/servos/conduits) - and
classifies fin material against the compartment solid and the delivered
airframe. An intruding point is fin AND cavity AND still solid.

Two configurations, ~2-4 min total: the flying-wing centre fin (the one
that was proven to stand in the compartment as a centre wall before its
root was trimmed to the cavity roof) and the default conventional.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import tools_probe_fin_intrusion as probe   # noqa: E402

_BY_NAME = {c["name"]: c for c in probe.CONFIGS}


@pytest.mark.parametrize("name", ["fw-swept-center_fin-default",
                                  "conv-conventional-default"])
def test_no_tail_surface_intrudes_into_the_bay(name):
    res = probe.probe(_BY_NAME[name])
    assert res["bay_ok"], f"{name}: no bay cavity was carved: {res}"
    assert res["fins"], f"{name}: the builder produced no tail surfaces"
    bad = [s for s in res["surfaces"] if s["intrusion_pts"]]
    assert not bad, (f"{name}: tail material inside the cavity void: "
                     f"{[(s['surface'], s['intrusion_pts'], s.get('intrusion_z_mm')) for s in bad]}")
    assert not any("NOT trimmed" in w for w in res["warnings"]), res["warnings"]
    assert res["ok"], res
