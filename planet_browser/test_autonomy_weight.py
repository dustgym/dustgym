"""Planner weight coupling: the uphill drive energy charges the regolith being hauled, not just the
dry rover -- so a loaded haul up a grade costs the load's m*g*h (path-dependent energy)."""
import math

from planet_browser import autonomy as AUT
from planet_browser import mission_planner as MP


def test_uphill_haul_charges_the_hauled_mass(monkeypatch):
    m = MP.demo_mission()
    b = AUT.initial_belief(m, 1)
    monkeypatch.setattr(MP, "haul_elevation_gain_m", lambda *a, **k: 2.0)   # force a 2 m climb, isolate the mass term
    g = MP.body_gravity("moon")
    site = (b.x + 10.0, b.y)                                                # 10 m away -> finite drive + slope
    light = AUT.execute_leg(b, {"site": site, "mass": 0.0}, dem=object(), g=g)["true_energy_J"]
    heavy = AUT.execute_leg(b, {"site": site, "mass": 30.0}, dem=object(), g=g)["true_energy_J"]
    assert heavy > light                                                    # hauling 30 kg uphill costs more
    assert math.isclose(heavy - light, 30.0 * g * 2.0, rel_tol=1e-6)        # exactly the load's m*g*h


def test_no_extra_charge_when_flat_or_empty(monkeypatch):
    m = MP.demo_mission()
    b = AUT.initial_belief(m, 1)
    g = MP.body_gravity("moon")
    site = (b.x + 10.0, b.y)
    monkeypatch.setattr(MP, "haul_elevation_gain_m", lambda *a, **k: 0.0)   # flat: no climb -> mass irrelevant
    flat_light = AUT.execute_leg(b, {"site": site, "mass": 0.0}, dem=object(), g=g)["true_energy_J"]
    flat_heavy = AUT.execute_leg(b, {"site": site, "mass": 30.0}, dem=object(), g=g)["true_energy_J"]
    assert math.isclose(flat_light, flat_heavy, rel_tol=1e-9)               # no grade -> no gravity-work delta
