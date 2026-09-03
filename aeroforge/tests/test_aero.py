"""Aerodynamics equations + airfoil polars (spec 5.2, 5.3)."""
import math

import numpy as np
import pytest

from backend.physics import aero as A
from backend.physics.airfoils import (
    Airfoil, LIBRARY, naca4_coordinates, thin_airfoil_props, _naca4_camber,
    _reflexed_camber,
)
from backend.physics.atmosphere import isa


def test_naca4_geometry():
    c = naca4_coordinates("2412")
    x, y = c[:, 0], c[:, 1]
    assert abs(x.max() - 1.0) < 1e-6 and abs(x.min()) < 1e-6
    # 12% thickness: max upper-lower gap close to 0.12
    assert 0.10 < (y.max() - y.min()) < 0.145


def test_thin_airfoil_naca2412():
    # Anderson: NACA 2412 has alpha_L0 ~ -2 deg and Cm_c/4 ~ -0.05
    a_l0, cm = thin_airfoil_props(lambda x: _naca4_camber(x, 0.02, 0.4)[1])
    assert -math.radians(3.0) < a_l0 < -math.radians(1.2)
    assert -0.08 < cm < -0.03


def test_reflexed_section_has_positive_cm0():
    # flying-wing requirement: reflex -> Cm0 > 0 (spec 5.2)
    _, cm = thin_airfoil_props(lambda x: _reflexed_camber(x, 0.025, 2.4)[1])
    assert cm > 0.0
    af = Airfoil("RFX-9 reflexed")
    assert af.cm0(2.0e5) > 0.0


def test_polars_are_physical():
    # the section library is reflexed sections now - a tailless wing has no
    # tail to trim a conventional cambered profile against
    af = Airfoil("RFX-9 reflexed")
    for re in (8e4, 2e5, 5e5):
        clmax = af.cl_max(re)
        assert 0.9 < clmax < 1.9
        cd = af.cd_at_cl(0.5, re)
        assert 0.005 < cd < 0.06
    # low-Re penalty: Cl_max degrades and Cd grows at low Re (spec 5.2)
    assert af.cl_max(6e4) <= af.cl_max(4e5) + 0.05
    assert af.cd_at_cl(0.6, 6e4) > af.cd_at_cl(0.6, 4e5)


def test_lift_equation_and_required_cl():
    # L = 0.5 rho V^2 S CL ; at CL_req the lift equals weight
    rho = isa(0.0).density_kgm3
    W, v, s = 15.0, 12.0, 0.25
    cl = A.required_cl(W, rho, v, s)
    lift = 0.5 * rho * v**2 * s * cl
    assert abs(lift - W) < 1e-9


def test_drag_polar_and_oswald():
    # CD = CD0 + CL^2/(pi AR e)
    cd = A.drag_polar(0.6, 6.0, 0.8, 0.03)
    assert abs(cd - (0.03 + 0.36 / (math.pi * 6.0 * 0.8))) < 1e-12
    e = A.oswald_efficiency(6.0)
    assert 0.6 <= e <= 0.85


def test_stall_speed_formula():
    # V_stall = sqrt(2W/(rho S CLmax))
    rho = isa(0.0).density_kgm3
    vs = A.stall_speed(15.0, rho, 0.25, 1.2)
    assert abs(vs - math.sqrt(2 * 15.0 / (rho * 0.25 * 1.2))) < 1e-12


def test_aspect_ratio():
    assert A.aspect_ratio(1.2, 0.24) == pytest.approx(6.0)


def test_reflexed_sections_are_the_primary_library():
    """AeroForge designs flying wings, so the section library has to offer a
    real choice of reflexed profiles - thin for speed, deep for a blended
    centre body - not one token section."""
    reflexed = [d for d in LIBRARY.values() if d.reflexed]
    assert len(reflexed) >= 3, [d.name for d in LIBRARY.values()]
    thicknesses = sorted(d.thickness for d in reflexed)
    assert thicknesses[0] < thicknesses[-1], "all the same thickness"
    for d in reflexed:
        assert 0.05 <= d.thickness <= 0.16, d.name
        assert Airfoil(d.name).cm0(2.0e5) > 0.0, d.name
