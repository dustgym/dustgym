"""Planner weight coupling: a loaded haul costs more -- the regolith in the drum drives BOTH the slip
(heavier -> more slip -> more 1/(1-slip) drive energy) AND the gravity climb (the load's m*g*h)."""
from planet_browser import autonomy as AUT
from planet_browser import mission_planner as MP


def test_planner_slip_is_weight_and_slope_coupled():
    # the planner's per-leg slip now comes from the conserved ladder: rises with slope AND with weight.
    assert MP.slip_alpha_to_slip(0.0) < MP.slip_alpha_to_slip(30.0)                      # steeper -> more slip
    s_empty = MP.slip_alpha_to_slip(20.0, payload_kg=0.0)
    s_loaded = MP.slip_alpha_to_slip(20.0, payload_kg=K_drum())
    assert 0.0 < s_empty < s_loaded < 1.0                                                # heavier -> more slip
    assert MP.slip_alpha_to_slip(45.0) > 0.9                                             # entraps near ~45 deg


def K_drum():
    return MP.DRUM_KG


def test_uphill_haul_costs_more_when_loaded(monkeypatch):
    m = MP.demo_mission()
    b = AUT.initial_belief(m, 1)
    monkeypatch.setattr(MP, "haul_elevation_gain_m", lambda *a, **k: 2.0)                # force a 2 m climb
    g = MP.body_gravity("moon")
    site = (b.x + 10.0, b.y)
    light = AUT.execute_leg(b, {"site": site, "mass": 0.0}, dem=object(), g=g)["true_energy_J"]
    heavy = AUT.execute_leg(b, {"site": site, "mass": 30.0}, dem=object(), g=g)["true_energy_J"]
    assert heavy > light
    # at least the load's gravity climb (slip adds a bit more on top)
    assert heavy - light >= 30.0 * g * 2.0 * 0.99


def test_loaded_costs_more_even_on_flat_via_slip(monkeypatch):
    m = MP.demo_mission()
    b = AUT.initial_belief(m, 1)
    monkeypatch.setattr(MP, "haul_elevation_gain_m", lambda *a, **k: 0.0)                # flat: no gravity term
    g = MP.body_gravity("moon")
    site = (b.x + 10.0, b.y)
    light = AUT.execute_leg(b, {"site": site, "mass": 0.0}, dem=object(), g=g)["true_energy_J"]
    heavy = AUT.execute_leg(b, {"site": site, "mass": 30.0}, dem=object(), g=g)["true_energy_J"]
    assert heavy >= light                                                                # weight still adds slip
