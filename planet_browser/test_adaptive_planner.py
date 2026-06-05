"""The self-learned slip-energy model, deployed to price a real mission (no synthetic data)."""
from __future__ import annotations

from . import adaptive_planner as ADP
from . import autonomy as AUT
from . import mission_planner as MP


def _mission():
    return MP.mission_from_dict({"name": "p", "body": "moon", "charger": [0, 0], "orders": [
        {"action": "cut_pad", "kind": "cut", "x": 5, "y": 5, "footprint_m2": 40, "depth_m": 0.3},
        {"action": "fill_low", "kind": "fill", "x": 12, "y": 8, "footprint_m2": 40, "depth_m": 0.3}]})


def test_learned_pricing_tracks_executed_and_does_not_over_inflate():
    dem = MP.load_haworth_dem()
    o = MP.flattest_anchor(dem)
    cl = AUT.run_closed_loop(_mission(), dem=dem, dem_origin=o)
    p = ADP.price_mission(cl["legs"], ADP.learned_model())
    assert p["learned_J"] >= p["naive_J"] - 1e-6                  # drive inflation is non-negative
    assert abs(p["learned_J"] - p["actual_J"]) <= 0.05 * p["actual_J"] + 1.0   # tracks the executed truth (no over-inflation)


def test_dig_energy_does_not_inflate():
    # a pure-dig leg (no drive) must NOT be inflated by the slip model; a pure-drive leg inflates by inflation(slope)
    model = ADP.learned_model()
    legs = [{"nominal_J": 1000.0, "dig_e": 1000.0, "slope_deg": 20.0, "true_J": 1000.0},
            {"nominal_J": 1000.0, "dig_e": 0.0, "slope_deg": 20.0, "true_J": 1300.0}]
    p = ADP.price_mission(legs, model)
    assert abs(p["learned_J"] - (1000.0 + 1000.0 * model.predict(20.0))) < 1.0   # dig flat, drive inflated
