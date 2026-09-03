"""v3 integration: EVERY registered airplane type reaches the CAD module
that owns it, through the two public entry points and nothing else.

The per-type CAD suites (test_multiwing.py, test_twinboom.py,
test_glider_cad.py, test_delta_cad.py) import their builder module DIRECTLY,
so a type whose module is finished but never wired into
`cad/geometry.py` passes every one of them and still exports a wrong shape
from the server - exactly the gap this file closes. `canard`, `tandem` and
`biplane` were in that state when the integration wave picked them up.

These tests are CHEAP on purpose: they resolve the dispatch, they never
build. A build costs 6-12 minutes and is already covered per type.
"""
import pytest

from backend.cad import geometry
from backend.physics.config_defs import AIRPLANE_TYPES

# The module that owns each type's topology (V3_PLAN.md "Module layout").
# `None` = the tailless path inside geometry.py itself: a delta IS a
# flying-wing dict whose planform is "delta", and rides that CAD unmodified.
# biplane and glider were removed from the registry outright (builder,
# 2026-08-21); this table shrank with it, and the parity tests below are what
# guarantee neither can quietly come back on one side only.
OWNER = {
    "flying_wing": None,
    "delta": None,
    "conventional": "backend.cad.conventional",
    "canard": "backend.cad.multiwing",
    "tandem": "backend.cad.multiwing",
    "twin_boom": "backend.cad.twinboom",
}


def test_every_registered_type_has_a_declared_owner():
    """A type added to the registry without a CAD owner would silently build
    as a flying wing - the failure mode this file exists to prevent."""
    assert set(AIRPLANE_TYPES) == set(OWNER), (
        f"registry {sorted(AIRPLANE_TYPES)} vs dispatch {sorted(OWNER)}")


@pytest.mark.parametrize("atype", sorted(OWNER))
def test_type_resolves_to_its_owning_module(atype):
    mod = geometry._type_module({"airplane_type": atype})
    want = OWNER[atype]
    if want is None:
        assert mod is None, (
            f"{atype} must build through the tailless path in geometry.py, "
            f"got {mod.__name__}")
    else:
        assert mod is not None, f"{atype} falls through to the flying wing"
        assert mod.__name__ == want, (
            f"{atype} -> {mod.__name__}, expected {want}")


@pytest.mark.parametrize("atype", sorted(OWNER))
def test_owning_module_exposes_both_entry_points(atype):
    """Both public builders must exist with the same names on every owner -
    `build_design_parts` for the STEP assembly, `build_design_solid` for the
    one-piece STL/preview (ARCHITECTURE.md: keep both signatures stable)."""
    mod = geometry._type_module({"airplane_type": atype}) or geometry
    for fn in ("build_design_parts", "build_design_solid"):
        assert callable(getattr(mod, fn, None)), f"{mod.__name__}.{fn}"


def test_absent_airplane_type_is_the_v1_flying_wing():
    """A v1 design dict carries no airplane_type at all and must keep
    building through the untouched tailless path."""
    assert geometry._type_module({}) is None
    assert geometry._type_module({"airplane_type": None}) is None
