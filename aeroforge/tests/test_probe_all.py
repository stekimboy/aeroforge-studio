"""The validity sweep's own checks on the configurations that failed it
(work plan task 2, 2026-08-28). `tools_probe_all.py` runs every UI
axis one at a time; this keeps the two root-cause fixes it produced under
test at probe level (loft + tail + bay through the builder's probe seam,
no hinges/servos/conduits, ~3 min each):

* conventional at the sidebar's 700 x 1200 x 300 box - the fin root TE
  landed on the tail cone's end cap and the fin went MISSING from every
  configuration of that box (`conventional._FIN_TE_INSET_MM`);
* delta at the sidebar box - the fused void carved only the aft half of the
  compartment and the ladder fell to rung 4 (`hatch._CORE_FLOOR_LIFT_MM`
  cut retry);
* the bwb + centre fin at the 675 box is covered by tests/test_bay_ladder.py.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import tools_probe_all as sweep   # noqa: E402

_BY_NAME = {c["name"]: c for c in sweep.CONFIGS}


@pytest.mark.parametrize("name", ["conventional-default", "delta-default"])
def test_axis_configuration_builds_its_bay_and_tail(name):
    res = sweep.probe(_BY_NAME[name])
    ck = res["checks"]
    if name == "delta-default":
        # the delta's fused void carved only the aft half of the compartment
        # and fell to rung 4 (bay 20 % shorter, 38 % narrower, 14 min) until
        # the lifted-core cut retry (hatch._CORE_FLOOR_LIFT_MM): rung 0 now
        assert ck["bay"]["rung"].startswith("rung 0"), ck["bay"]
        assert (ck["bay"]["cavity_extended_mm"] or 0) >= 12.0, ck["bay"]
    assert ck["physics"]["feasible"], ck["physics"]
    assert ck["physics"]["fits_box"], ck["physics"]
    assert ck["bay"]["ok"], ck["bay"]
    for fin, f in ck["fins"].items():
        assert f["fused"], f"{name}: {fin} is missing from the built airframe: {f}"
    assert not ck.get("warnings"), ck.get("warnings")
    assert ck["intrusion"]["pts"] == 0, ck["intrusion"]
    assert res["ok"], res["fail"]
