"""Airfoil geometry generation + polars (Cl, Cd, Cm vs alpha, Re).

Spec section 5.2 / SPEC_FLYING_WING.md section 6.

AeroForge designs flying wings only, so the PRIMARY library is a family of
analytically generated REFLEXED sections (positive Cm0, which is what lets a
tailless wing trim with the CG ahead of the neutral point). Three reflex
thicknesses cover the range the app produces:

    RFX-7   7% thick, light camber   - speed / high wing loading
    RFX-9   9% thick, medium camber  - the general-purpose section
    RFX-11 11% thick, more camber    - deep blended centre bodies

A single symmetric NACA 0008 stays in the library as the section for vertical
surfaces (winglets, fins). NACA 4-digit generation is retained because that is
how the symmetric fin section is produced.

Geometry: NACA 4-digit sections generated analytically (Abbott & von Doenhoff
"Theory of Wing Sections" equations), plus the analytically generated REFLEXED
sections for the wing itself.

Polars: primary engine is NeuralFoil (AeroSandbox) evaluated on the generated
coordinates - real low-Reynolds viscous predictions, exactly the RC regime
(Re ~ 5e4..5e5). If NeuralFoil is unavailable, an analytic fallback based on
thin-airfoil theory (Anderson, "Fundamentals of Aerodynamics") plus empirical
low-Re corrections is used with the same interface.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from functools import lru_cache

import numpy as np

try:  # primary polar engine
    import neuralfoil as nf

    HAVE_NEURALFOIL = True
except Exception:  # pragma: no cover - exercised only when install failed
    HAVE_NEURALFOIL = False


# ----------------------------------------------------------------------------
# Geometry generation
# ----------------------------------------------------------------------------

def _naca4_camber(x: np.ndarray, m: float, p: float) -> tuple[np.ndarray, np.ndarray]:
    """NACA 4-digit camber line y_c(x) and slope dyc/dx (Abbott & von Doenhoff):
       y_c = m/p^2 (2 p x - x^2)            for x < p
       y_c = m/(1-p)^2 ((1-2p) + 2 p x - x^2) for x >= p
    """
    yc = np.where(
        x < p,
        m / max(p, 1e-9) ** 2 * (2 * p * x - x**2),
        m / (1 - p) ** 2 * ((1 - 2 * p) + 2 * p * x - x**2),
    )
    dyc = np.where(
        x < p,
        2 * m / max(p, 1e-9) ** 2 * (p - x),
        2 * m / (1 - p) ** 2 * (p - x),
    )
    if m == 0:
        yc = np.zeros_like(x)
        dyc = np.zeros_like(x)
    return yc, dyc


def _naca_thickness(x: np.ndarray, t: float) -> np.ndarray:
    """NACA 4-digit thickness distribution (closed trailing edge variant):
       y_t = 5t (0.2969 sqrt(x) - 0.1260 x - 0.3516 x^2 + 0.2843 x^3 - 0.1036 x^4)
    """
    return 5 * t * (
        0.2969 * np.sqrt(x) - 0.1260 * x - 0.3516 * x**2 + 0.2843 * x**3 - 0.1036 * x**4
    )


def _reflexed_camber(x: np.ndarray, m: float, r: float) -> tuple[np.ndarray, np.ndarray]:
    """Analytic reflexed camber line for tailless aircraft:
       y_c = m * x (1 - x) (1 - r x)
    The cubic term (r > 1) pulls the aft camber line below the chord, which by
    thin-airfoil theory (Cm_c/4 = pi/4 (A2 - A1)) yields a POSITIVE Cm0 -
    the property a flying wing needs to trim with CG ahead of the NP.
    """
    yc = m * x * (1 - x) * (1 - r * x)
    dyc = m * ((1 - 2 * x) * (1 - r * x) + x * (1 - x) * (-r))
    return yc, dyc


def _assemble(x: np.ndarray, yc: np.ndarray, dyc: np.ndarray, yt: np.ndarray) -> np.ndarray:
    """Combine camber line + thickness normal to camber into a closed loop of
    coordinates running TE -> upper -> LE -> lower -> TE (Selig ordering)."""
    theta = np.arctan(dyc)
    xu = x - yt * np.sin(theta)
    yu = yc + yt * np.cos(theta)
    xl = x + yt * np.sin(theta)
    yl = yc - yt * np.cos(theta)
    xs = np.concatenate([xu[::-1], xl[1:]])
    ys = np.concatenate([yu[::-1], yl[1:]])
    return np.stack([xs, ys], axis=1)


def _cosine_x(n: int = 81) -> np.ndarray:
    """Cosine spacing clusters points at LE/TE where curvature is high."""
    beta = np.linspace(0.0, math.pi, n)
    return 0.5 * (1 - np.cos(beta))


def naca4_coordinates(code: str, n: int = 81) -> np.ndarray:
    """Generate NACA 4-digit coordinates, e.g. naca4_coordinates('2412')."""
    code = code.strip()
    if len(code) != 4 or not code.isdigit():
        raise ValueError(f"invalid NACA 4-digit code: {code!r}")
    m = int(code[0]) / 100.0
    p = int(code[1]) / 10.0
    t = int(code[2:]) / 100.0
    x = _cosine_x(n)
    yc, dyc = _naca4_camber(x, m, p if p > 0 else 0.3)
    yt = _naca_thickness(x, t)
    return _assemble(x, yc, dyc, yt)


def reflexed_coordinates(m: float = 0.20, r: float = 1.45, t: float = 0.09, n: int = 81) -> np.ndarray:
    """Generate the reflexed flying-wing section (see _reflexed_camber)."""
    x = _cosine_x(n)
    yc, dyc = _reflexed_camber(x, m, r)
    yt = _naca_thickness(x, t)
    return _assemble(x, yc, dyc, yt)


# ----------------------------------------------------------------------------
# Thin-airfoil theory quantities (used by the fallback engine and for Cm0
# cross-checks). Anderson, "Fundamentals of Aerodynamics":
#   x = c/2 (1 - cos theta)
#   alpha_L0 = -(1/pi) Int_0^pi dyc/dx (cos th - 1) dth
#   A1 = (2/pi) Int dyc/dx cos th dth ;  A2 = (2/pi) Int dyc/dx cos 2th dth
#   Cm_c/4 = (pi/4)(A2 - A1)
# ----------------------------------------------------------------------------

def thin_airfoil_props(camber_fn) -> tuple[float, float]:
    """Return (alpha_L0 [rad], Cm_c/4) for a camber-line slope function dyc/dx(x)."""
    theta = np.linspace(1e-4, math.pi - 1e-4, 400)
    x = 0.5 * (1 - np.cos(theta))
    dyc = camber_fn(x)
    trapz = getattr(np, "trapezoid", None) or np.trapz  # numpy 2.x renamed it
    alpha_l0 = -(1.0 / math.pi) * trapz(dyc * (np.cos(theta) - 1.0), theta)
    a1 = (2.0 / math.pi) * trapz(dyc * np.cos(theta), theta)
    a2 = (2.0 / math.pi) * trapz(dyc * np.cos(2 * theta), theta)
    cm_c4 = (math.pi / 4.0) * (a2 - a1)
    return float(alpha_l0), float(cm_c4)


# ----------------------------------------------------------------------------
# Airfoil model with polars
# ----------------------------------------------------------------------------

@dataclass
class AirfoilDef:
    name: str
    kind: str            # 'naca4' or 'reflexed'
    param: str           # naca code, or 'm,r,t' for reflexed
    thickness: float
    description: str
    reflexed: bool = False


# The reflexed wing sections, all generated analytically (see DECISIONS.md:
# NACA 4-digit sections cannot produce reflex, so the camber line is built from
# y_c = m x (1-x) (1 - r x), whose cubic term gives Cm_c/4 > 0).
#
# Camber-line parameters are TUNED against the polar engine, not guessed:
#   r ~ 1.45 puts the camber-line crossing at x = 1/r = 0.69 c, so the section
#   carries ~2.4% positive camber around 0.30 c and ~0.6% of reflex droop
#   around 0.87 c - the shape of a real tailless section (MH-60 / EH class).
#   m is then set so the VISCOUS Cm0 lands at +0.008..+0.022 over the Reynolds
#   range these models actually fly at (1.5e5-4.5e5). That is the value that puts
#   the trim CG at 6-12% of MAC ahead of the neutral point, i.e. inside the
#   tailless static-margin band, with a couple of degrees of washout left over.
#   The GEOMETRIC reflex (inviscid Cm_c/4 ~ +0.04) is positive at every
#   Reynolds number; the viscous value falls off below Re ~ 1.3e5 because the
#   aft camber then sits inside the laminar separation bubble, and that is
#   reported honestly rather than hidden.
#
# The three thicknesses are chosen from what real tailless models use:
#   - 7%  : thin, low drag at high wing loading; the fast/small-chord end
#           (Zagi-class speed wings, delta-planform sport wings).
#   - 9%  : the general-purpose reflexed thickness (Skywalker X5 / AR Wing
#           class sections sit right here).
#   - 11% : needed when the centre body must swallow a pack and a camera - a
#           deeper section keeps the body's chord scale (and therefore its
#           length) sane on a blended wing body.
LIBRARY: dict[str, AirfoilDef] = {
    "RFX-7 reflexed": AirfoilDef("RFX-7 reflexed", "reflexed", "0.17,1.45,0.07", 0.07,
                                 "Thin reflexed section - speed and high wing loading",
                                 reflexed=True),
    "RFX-9 reflexed": AirfoilDef("RFX-9 reflexed", "reflexed", "0.20,1.45,0.09", 0.09,
                                 "General-purpose reflexed section for flying wings",
                                 reflexed=True),
    "RFX-11 reflexed": AirfoilDef("RFX-11 reflexed", "reflexed", "0.21,1.45,0.11", 0.11,
                                  "Deep reflexed section - blended centre bodies",
                                  reflexed=True),
    "NACA 0008": AirfoilDef("NACA 0008", "naca4", "0008", 0.08,
                            "Symmetric section for winglets and fins"),
}

#: sections that may be used for the WING of a flying wing (reflex required)
WING_AIRFOILS: tuple[str, ...] = tuple(
    n for n, d in LIBRARY.items() if d.reflexed)
#: the symmetric section every vertical surface is built from
FIN_AIRFOIL = "NACA 0008"


def coordinates_for(defn: AirfoilDef, n: int = 81) -> np.ndarray:
    if defn.kind == "naca4":
        return naca4_coordinates(defn.param, n)
    m, r, t = (float(v) for v in defn.param.split(","))
    return reflexed_coordinates(m, r, t, n)


def _camber_slope_fn(defn: AirfoilDef):
    if defn.kind == "naca4":
        m = int(defn.param[0]) / 100.0
        p = int(defn.param[1]) / 10.0
        return lambda x: _naca4_camber(x, m, p if p > 0 else 0.3)[1]
    m, r, _t = (float(v) for v in defn.param.split(","))
    return lambda x: _reflexed_camber(x, m, r)[1]


ALPHA_GRID = np.arange(-6.0, 18.01, 0.75)  # deg


def _re_bucket(Re: float) -> int:
    """Bucket Re geometrically (~12% bins) so polar grids can be cached."""
    Re = max(2e4, min(Re, 2e6))
    return int(round(math.log(Re) / math.log(1.12)))


def _bucket_re(bucket: int) -> float:
    return math.exp(bucket * math.log(1.12))


@lru_cache(maxsize=256)
def _polar_grid(name: str, bucket: int) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
    """(CL, CD, CM) arrays over ALPHA_GRID at the bucketed Re, cached."""
    defn = LIBRARY[name]
    Re = _bucket_re(bucket)
    if HAVE_NEURALFOIL:
        coords = coordinates_for(defn, 121)
        out = nf.get_aero_from_coordinates(
            coordinates=coords, alpha=ALPHA_GRID, Re=Re, model_size="large"
        )
        cl = np.asarray(out["CL"], dtype=float)
        cd = np.asarray(out["CD"], dtype=float)
        cm = np.asarray(out["CM"], dtype=float)
    else:
        cl, cd, cm = _analytic_polar(defn, Re)
    return tuple(cl), tuple(cd), tuple(cm)


def _analytic_polar(defn: AirfoilDef, Re: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Fallback polar from thin-airfoil theory + empirical low-Re corrections.

    - Lift: Cl = 2 pi (alpha - alpha_L0), capped by Cl_max(Re).
    - Cl_max(Re): low-Re degradation ~ (Re/3e5)^0.12 on a camber/thickness-
      dependent base (McCormick-style low-Re trend; RC regime penalty).
    - Drag: Cd = Cd_min(Re) + k (Cl - Cl_minD)^2 with Cd_min from fully
      turbulent flat-plate skin friction Cf = 0.074 Re^-0.2 (Schlichting)
      x thickness form factor (1 + 2.7 t/c + 100 (t/c)^4)  (Hoerner).
    - Cm: constant Cm_c/4 from thin-airfoil theory.
    """
    alpha_l0, cm0 = thin_airfoil_props(_camber_slope_fn(defn))
    a = np.radians(ALPHA_GRID)
    cl_lin = 2 * math.pi * (a - alpha_l0)
    m_camber = abs(alpha_l0) * 10.0
    base_clmax = 1.15 + 2.2 * m_camber  # ~1.15 symmetric -> ~1.5 cambered
    clmax = base_clmax * min(1.15, max(0.62, (Re / 3.0e5) ** 0.12))
    cl = np.clip(cl_lin, -0.8 * clmax, clmax)
    # post-stall droop
    stalled = cl_lin > clmax
    cl[stalled] = clmax - 0.35 * (cl_lin[stalled] - clmax)
    tc = defn.thickness
    cf = 0.074 / Re**0.2
    cd_min = 2 * cf * (1 + 2.7 * tc + 100 * tc**4)
    cl_mind = 2 * math.pi * (-alpha_l0) * 0.5
    cd = cd_min + 0.011 * (cl - cl_mind) ** 2
    cd[stalled] += 0.05 * (cl_lin[stalled] - clmax) ** 2
    cm = np.full_like(cl, cm0)
    return cl, cd, cm


class Airfoil:
    """Polar interface: cl/cd/cm as functions of (alpha_deg, Re)."""

    def __init__(self, name: str):
        if name not in LIBRARY:
            raise KeyError(f"unknown airfoil {name!r}; options: {list(LIBRARY)}")
        self.name = name
        self.defn = LIBRARY[name]

    # -- raw polars ---------------------------------------------------------
    def polar(self, Re: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        cl, cd, cm = _polar_arrays(self.name, _re_bucket(Re))
        return ALPHA_GRID, cl, cd, cm

    def cl(self, alpha_deg: float, Re: float) -> float:
        a, cl, _, _ = self.polar(Re)
        return float(np.interp(alpha_deg, a, cl))

    def cd(self, alpha_deg: float, Re: float) -> float:
        a, _, cd, _ = self.polar(Re)
        return float(np.interp(alpha_deg, a, cd))

    def cm(self, alpha_deg: float, Re: float) -> float:
        a, _, _, cm = self.polar(Re)
        return float(np.interp(alpha_deg, a, cm))

    # -- derived quantities used by the design pipeline ---------------------
    def cl_max(self, Re: float) -> float:
        return _polar_branch(self.name, _re_bucket(Re)).cl_max_grid

    def cd_at_cl(self, cl_target: float, Re: float) -> float:
        """Profile drag at a given operating Cl (pre-stall branch)."""
        b = _polar_branch(self.name, _re_bucket(Re))
        return float(np.interp(np.clip(cl_target, b.cl_lo, b.cl_hi),
                               b.cl_sorted, b.cd_sorted))

    def alpha_for_cl(self, cl_target: float, Re: float) -> float:
        b = _polar_branch(self.name, _re_bucket(Re))
        return float(np.interp(np.clip(cl_target, b.cl_lo, b.cl_hi),
                               b.cl_sorted, b.a_sorted))

    def cm0(self, Re: float) -> float:
        """Viscous pitching moment about c/4 at zero lift (sign matters)."""
        return _polar_branch(self.name, _re_bucket(Re)).cm0

    def cm0_geometric(self) -> float:
        """Reflex measured on the CAMBER LINE alone: thin-airfoil Cm_c/4.

        This is the Reynolds-independent statement "this section is reflexed".
        The viscous cm0() above falls below it at low Reynolds number because
        the aft camber sits inside the separation bubble there, but the shape
        itself is unchanged - so this is the quantity that decides whether a
        section is admissible for a flying wing.
        """
        return thin_airfoil_props(_camber_slope_fn(self.defn))[1]

    def alpha_l0_deg(self, Re: float) -> float:
        return self.alpha_for_cl(0.0, Re)

    def lift_slope_2d(self, Re: float) -> float:
        """dCl/dalpha [1/rad] from the linear part of the polar."""
        return _polar_branch(self.name, _re_bucket(Re)).a0


class _PolarBranch:
    """Everything the derived polar quantities need, computed ONCE per
    (airfoil, Re bucket) - see :func:`_polar_branch`."""
    __slots__ = ("cl_sorted", "a_sorted", "cd_sorted", "cl_lo", "cl_hi",
                 "cl_max_grid", "a0", "cm0")


@lru_cache(maxsize=256)
def _polar_arrays(name: str, bucket: int) -> tuple[np.ndarray, np.ndarray,
                                                   np.ndarray]:
    """The cached grid as READ-ONLY numpy arrays (speed pass 2026-08-28:
    `polar()` was called ~180k times per generate and rebuilt three arrays
    from tuples each time). Callers never mutate them - the flags enforce
    it."""
    cl, cd, cm = _polar_grid(name, bucket)
    out = (np.array(cl), np.array(cd), np.array(cm))
    for arr in out:
        arr.setflags(write=False)
    return out


@lru_cache(maxsize=256)
def _polar_branch(name: str, bucket: int) -> _PolarBranch:
    """Pre-stall branch of the polar, sorted by Cl, plus the scalar
    quantities derived from it (Cl_max, lift slope, cm0).

    These are pure functions of the cached grid, so memoizing them changes
    nothing numerically: the arrays and the arithmetic are exactly the ones
    the per-call code used (argmax -> slice -> argsort -> interp; polyfit on
    the -2..6 deg window), just done once instead of ~30 000 times per
    generate (measured: a third of the optimizer's time was here)."""
    cl, cd, _cm = _polar_arrays(name, bucket)
    a = ALPHA_GRID
    imax = int(np.argmax(cl))
    cl_b, a_b, cd_b = cl[: imax + 1], a[: imax + 1], cd[: imax + 1]
    order = np.argsort(cl_b)
    b = _PolarBranch()
    b.cl_sorted, b.a_sorted, b.cd_sorted = cl_b[order], a_b[order], cd_b[order]
    b.cl_lo, b.cl_hi = cl_b.min(), cl_b.max()
    b.cl_max_grid = float(np.max(cl))
    lo, hi = np.searchsorted(a, -2.0), np.searchsorted(a, 6.0)
    b.a0 = float(np.polyfit(np.radians(a[lo:hi]), cl[lo:hi], 1)[0])
    a_l0 = float(np.interp(np.clip(0.0, b.cl_lo, b.cl_hi), b.cl_sorted,
                           b.a_sorted))
    b.cm0 = float(np.interp(a_l0, a, _cm))
    return b


def pick_airfoil(planform: str, mission: str, override: str | None = None) -> Airfoil:
    """Choose the wing section for a (planform, mission) pair.

    A flying wing ALWAYS gets a reflexed, positive-Cm0 section: without it
    there is nothing to balance the nose-down moment of a cambered wing and the
    aircraft cannot be trimmed with the CG ahead of the neutral point. An
    override naming a non-reflexed section is therefore ignored rather than
    obeyed (the caller reports it).

    The automatic pick is by thickness need:
      - a blended wing body has to swallow the payload inside the centre
        section, so it starts from the deepest reflexed section;
      - a sport wing runs at the highest wing loading and the highest cruise
        speed of the set, where the thin section's lower profile drag wins;
      - everything else takes the general-purpose 9% section.
    """
    if override and override.strip().lower() not in ("", "auto"):
        name = override.strip()
        if name in LIBRARY and LIBRARY[name].reflexed:
            return Airfoil(name)
        # fall through to the automatic pick: reflex is not optional here
    if planform == "bwb":
        return Airfoil("RFX-11 reflexed")
    if planform == "swept" and mission == "sport":
        return Airfoil("RFX-7 reflexed")
    return Airfoil("RFX-9 reflexed")


def fin_airfoil() -> Airfoil:
    """Symmetric section for winglets / fins (no camber: a vertical surface
    must make zero side force at zero sideslip)."""
    return Airfoil(FIN_AIRFOIL)
