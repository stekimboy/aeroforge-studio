"""ISA / Sutherland / Reynolds sanity (spec 5.1)."""
import math

from backend.physics.atmosphere import isa, reynolds


def test_sea_level_density():
    a = isa(0.0)
    assert abs(a.density_kgm3 - 1.225) < 0.001  # ISA sea level
    assert abs(a.temperature_K - 288.15) < 1e-9


def test_sutherland_viscosity_15C():
    a = isa(0.0)
    assert abs(a.viscosity_Pas - 1.81e-5) < 0.03e-5  # ~1.81e-5 Pa s at 15 C


def test_density_falls_with_altitude():
    assert isa(1500.0).density_kgm3 < isa(0.0).density_kgm3


def test_reynolds_number():
    # Re = rho V c / mu : RC-typical wing (0.2 m chord at 12 m/s, sea level)
    a = isa(0.0)
    re = reynolds(a.density_kgm3, 12.0, 0.2, a.viscosity_Pas)
    assert 1.5e5 < re < 1.8e5
    assert reynolds(a.density_kgm3, 0.0, 0.2, a.viscosity_Pas) == 0.0
