"""International Standard Atmosphere (ISA) + Reynolds number.

Equations (spec section 5.1):
  ISA troposphere:  T = T0 - L*h,  p = p0 * (T/T0)^(g/(L*R)),  rho = p/(R*T)
  Sutherland's law: mu = mu_ref * (T/T_ref)^1.5 * (T_ref + S) / (T + S)
  Reynolds number:  Re = rho * V * c / mu
"""
from __future__ import annotations

import math
from dataclasses import dataclass

# ISA sea-level constants
T0 = 288.15        # K (15 degC)
P0 = 101325.0      # Pa
L_RATE = 0.0065    # K/m temperature lapse rate
R_AIR = 287.058    # J/(kg K) specific gas constant of air
G0 = 9.80665       # m/s^2

# Sutherland constants for air
MU_REF = 1.716e-5  # Pa s at T_ref
T_REF = 273.15     # K
S_SUTH = 110.4     # K


@dataclass(frozen=True)
class Atmosphere:
    altitude_m: float
    temperature_K: float
    pressure_Pa: float
    density_kgm3: float
    viscosity_Pas: float


def isa(altitude_m: float = 0.0) -> Atmosphere:
    """ISA state in the troposphere (valid to 11 km, far above any RC field)."""
    h = max(0.0, min(altitude_m, 11000.0))
    # T = T0 - L*h
    T = T0 - L_RATE * h
    # p = p0 * (T/T0)^(g/(L*R))   (hydrostatic + ideal gas, constant lapse rate)
    p = P0 * (T / T0) ** (G0 / (L_RATE * R_AIR))
    # rho = p / (R*T)   (ideal gas law)
    rho = p / (R_AIR * T)
    # Sutherland's law: mu = mu_ref*(T/T_ref)^1.5*(T_ref+S)/(T+S)
    mu = MU_REF * (T / T_REF) ** 1.5 * (T_REF + S_SUTH) / (T + S_SUTH)
    return Atmosphere(h, T, p, rho, mu)


def reynolds(rho: float, V: float, chord: float, mu: float) -> float:
    """Re = rho * V * c / mu  (c = mean aerodynamic chord for wing-level Re)."""
    if chord <= 0 or V <= 0:
        return 0.0
    return rho * V * chord / mu


# quick sanity: sea level should give ~1.225 kg/m^3 and ~1.81e-5 Pa s at 15 C
if __name__ == "__main__":
    a = isa(0.0)
    print(a)
    assert abs(a.density_kgm3 - 1.225) < 0.001
    assert abs(a.viscosity_Pas - 1.81e-5) < 0.02e-5
