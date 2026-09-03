"""Tailless stability: MAC geometry, the spanwise strip model, CG placement.

Only flying-wing solvers exist now - `solve_tail_aft` and `solve_canard` were
deleted with the tailed configurations.
"""
import math

import pytest

from backend.physics import stability as S
from backend.physics.airfoils import Airfoil
from backend.physics.stability import (
    build_planform, chords_from_area, flying_wing_fin, mac_length,
    solve_flying_wing, x_le_mac, y_mac,
)

REFLEX = "RFX-9 reflexed"


def test_tailed_solvers_are_gone():
    """The product is flying wings only; leaving the old solvers importable
    invites a tailed code path to creep back in."""
    assert not hasattr(S, "solve_tail_aft")
    assert not hasattr(S, "solve_canard")


# ---------------------------------------------------------------------------
# Planform geometry (pure functions, closed-form checks)
# ---------------------------------------------------------------------------

def test_mac_of_a_rectangular_wing_is_its_chord():
    assert mac_length(0.2, 1.0) == pytest.approx(0.2)


def test_mac_matches_the_taper_formula():
    # MAC = (2/3) c_r (1 + lam + lam^2)/(1 + lam)
    assert mac_length(0.3, 0.5) == pytest.approx((2 / 3) * 0.3 * 1.75 / 1.5)


def test_chords_reproduce_the_area():
    c_r, c_t = chords_from_area(1.2, 0.30, 0.45)
    assert 1.2 * (c_r + c_t) / 2 == pytest.approx(0.30)   # S = b(c_r+c_t)/2
    assert c_t == pytest.approx(0.45 * c_r)


def test_y_mac_and_le_sweep_offset():
    b, lam, sweep = 1.2, 0.5, 25.0
    ym = y_mac(b, lam)
    assert ym == pytest.approx((b / 6) * (1 + 2 * lam) / (1 + lam))
    assert x_le_mac(sweep, b, lam) == pytest.approx(
        ym * math.tan(math.radians(sweep)))


# ---------------------------------------------------------------------------
# Strip model
# ---------------------------------------------------------------------------

def _planform(**over):
    kw = dict(span=1.10, c_root_wing=0.40, taper=0.45, sweep_le_deg=24.0,
              body_half_width_frac=0.17, body_chord_scale=1.22)
    kw.update(over)
    return build_planform(**kw)


def _wing(pf=None, **over):
    kw = dict(pf=pf if pf is not None else _planform(), airfoil=Airfoil(REFLEX),
              re_mac=2.0e5, ar=4.0, e_oswald=0.75, sm_target=0.08,
              cl_cruise=0.45)
    kw.update(over)
    return solve_flying_wing(**kw)


def test_the_blended_planform_starts_at_the_nose_datum():
    """The centre body's leading-edge root extension IS the nose, so the
    forward-most station has to be x = 0 - every CG number references it."""
    pf = _planform()
    # strips are sampled at cell centres, so the first station sits a fraction
    # of a millimetre aft of the true nose - but nothing may be ahead of it
    assert 0.0 <= min(pf.x_le) < 0.003


def test_the_body_blend_deepens_the_centre_chord_only():
    """Spec section 5: chord is scaled up over the inboard blend and untouched
    outboard, which is what makes the body part of the wing."""
    pf = _planform()
    from backend.physics.stability import chord_at
    root, mid, tip = chord_at(pf, 0.0), chord_at(pf, 0.5), chord_at(pf, 0.98)
    assert root > mid > tip
    plain = build_planform(span=1.10, c_root_wing=0.40, taper=0.45,
                           sweep_le_deg=24.0, body_half_width_frac=0.0,
                           body_chord_scale=1.0)
    assert chord_at(pf, 0.0) > chord_at(plain, 0.0)      # body adds chord
    assert chord_at(pf, 0.6) == pytest.approx(chord_at(plain, 0.6), rel=1e-6)


def test_cg_is_placed_ahead_of_the_neutral_point():
    r = _wing()
    assert r.x_cg < r.x_np
    assert r.static_margin == pytest.approx(0.08, abs=1e-6)
    assert r.dcm_dalpha < 0            # statically stable in pitch


def test_sweep_moves_the_neutral_point_aft():
    """A swept wing's outboard area sits further back, so the area-weighted
    aerodynamic centre moves aft - this is what lets a tailless wing be
    balanced at all."""
    aft = _wing(pf=_planform(sweep_le_deg=30.0))
    fwd = _wing(pf=_planform(sweep_le_deg=14.0))
    assert aft.x_np > fwd.x_np
    assert aft.x_cg > fwd.x_cg          # and the CG has to follow it back


def test_trim_solution_is_a_real_washout_value():
    r = _wing()
    assert 0.0 <= r.washout_deg <= 15.0
    assert math.isfinite(r.root_incidence_deg)


# ---------------------------------------------------------------------------
# Vertical surfaces
# ---------------------------------------------------------------------------

def test_fin_sizing_follows_real_flying_wing_proportions():
    """Not the tail-aft V_V band: on this arm that band demands a fin a
    quarter the size of the wing."""
    pf = _planform()
    r = _wing(pf=pf)
    fin = flying_wing_fin(pf=pf, area=0.30, x_cg=r.x_cg, vstab_type="winglets")
    assert 0.02 <= fin.area_total / 0.30 <= 0.09
    assert fin.count == 2
    assert fin.vv < 0.018          # legitimately far below the tail-aft band


def test_an_unswept_plank_needs_more_fin_than_a_swept_wing():
    """Sweep is the primary directional stiffness; with none to lean on, the
    fin has to do the whole job."""
    swept = _planform(sweep_le_deg=28.0)
    plank = _planform(sweep_le_deg=2.0, taper=0.85)
    a = flying_wing_fin(pf=swept, area=0.30, x_cg=_wing(pf=swept).x_cg,
                        vstab_type="center_fin")
    b = flying_wing_fin(pf=plank, area=0.30, x_cg=_wing(pf=plank).x_cg,
                        vstab_type="center_fin")
    assert b.area_total > a.area_total


def test_no_vertical_surface_means_exactly_zero_area():
    pf = _planform()
    fin = flying_wing_fin(pf=pf, area=0.30, x_cg=_wing(pf=pf).x_cg,
                          vstab_type="none")
    assert fin.area_total == 0.0 and fin.count == 0 and fin.height == 0.0


def test_reflexed_section_carries_positive_cm0():
    """The whole tailless trim argument rests on Cm0 > 0."""
    assert Airfoil(REFLEX).cm0(2.0e5) > 0
