"""TDD for the closed-loop autonomy estimator (P12) — the AutoNav "OD" analog.

A recursive belief-state estimator with uncertainty: `predict` is the dead-reckoning/process step
(uncertainty GROWS with distance/energy spent), `update_*` fuse a measurement via a scalar Kalman update
(uncertainty SHRINKS). The loop replans against this ESTIMATE, not assumed-perfect state. Measurements in
these tests come from the real drum-sensor uncertainty model + a real conserved-authority cut — not fabricated.
"""

from __future__ import annotations

import math

import mission_planner as MP


def _mission():
    return MP.mission_from_dict({"name": "a", "body": "moon", "charger": [0, 0], "orders": [
        {"action": "cut", "kind": "cut", "x": 40, "y": 30, "footprint_m2": 36, "depth_m": 0.04},
        {"action": "fill", "kind": "fill", "x": 44, "y": 44, "footprint_m2": 14, "depth_m": 0.10}]})


def test_initial_belief_starts_at_charger_full_and_known():
    import autonomy as A
    b = A.initial_belief(_mission(), tasks_total=2)
    assert (b.x, b.y) == (0.0, 0.0)
    assert math.isclose(b.soc_frac(), 1.0) and b.drum_kg == 0.0
    assert b.tasks_done == 0 and b.tasks_total == 2
    assert b.pos_sigma_m >= 0.0 and b.energy_sigma_J == 0.0


def test_kf_update_reduces_variance_and_weights_by_precision():
    import autonomy as A
    mu, var = A._kf_update(10.0, 4.0, 20.0, 4.0)              # equal precision -> midpoint, variance halves
    assert math.isclose(mu, 15.0) and var < 4.0
    mu2, var2 = A._kf_update(10.0, 100.0, 20.0, 1.0)          # trust the precise measurement
    assert mu2 > 19.0 and var2 < 1.0
    assert A._kf_update(5.0, math.inf, 7.0, 2.0) == (7.0, 2.0)  # no prior -> take the measurement


def test_predict_grows_uncertainty_and_moves_state():
    import autonomy as A
    b = A.initial_belief(_mission(), 2)
    b2 = A.predict(b, moved_to=(200.0, 0.0), drive_m=200.0, energy_spent_J=0.6e6)
    assert (b2.x, b2.y) == (200.0, 0.0)
    assert b2.pos_sigma_m > b.pos_sigma_m                     # odometry drift grows pose uncertainty
    assert b2.energy_J < b.energy_J and b2.energy_sigma_J > 0.0   # spent energy + model uncertainty


def test_drum_measurement_shrinks_uncertainty_and_brackets_truth():
    import autonomy as A
    import numpy as np
    from terrain_authority.column_state import ColumnState
    # true drum mass from a REAL cut of real-density regolith (no fabricated value)
    cs = ColumnState(width=10, height=10, cell_m=0.5, mass_areal=np.full((10, 10), 1920.0 * 10.0))
    mask = np.zeros((10, 10), bool); mask[5, 5] = True
    true_kg = cs.cut_to_inventory(mask, 0.05 * 1920.0)
    assert true_kg > 0.0
    reading_sigma = MP.RM.FDC_MPE_HALF_FULL * true_kg         # real published sensor uncertainty (2.56%)
    b = A.initial_belief(_mission(), 2)
    b = A.predict(b, drum_delta_kg=true_kg, drum_process_sigma_kg=MP.DRUM_KG)  # process: large drum uncertainty
    s0 = b.drum_sigma_kg
    b = A.update_drum(b, reading_kg=true_kg, reading_sigma_kg=reading_sigma)
    assert b.drum_sigma_kg < s0                               # measurement shrinks uncertainty
    assert abs(b.drum_kg - true_kg) <= 2.0 * b.drum_sigma_kg  # estimate brackets truth (AutoNav consistency)


def test_pose_fix_shrinks_position_uncertainty():
    import autonomy as A
    b = A.initial_belief(_mission(), 2)
    b = A.predict(b, moved_to=(300.0, 0.0), drive_m=300.0)    # pose uncertainty grew with distance
    s0 = b.pos_sigma_m
    b = A.update_pose(b, fix_xy=(298.0, 1.0), fix_sigma_m=1.0)  # a 1 m pose fix (e.g. landmark/map match)
    assert b.pos_sigma_m < s0 and abs(b.x - 298.0) < s0        # fix pulls the estimate + shrinks sigma


# ---- executor + controller: the closed loop (plan -> execute -> sense -> estimate -> replan) -----
def _spread():
    return MP.mission_from_dict({"name": "c", "body": "moon", "charger": [0, 0], "orders": [
        {"action": "cut A", "kind": "cut", "x": 120, "y": 0, "footprint_m2": 40, "depth_m": 0.05},
        {"action": "cut B", "kind": "cut", "x": -110, "y": 10, "footprint_m2": 40, "depth_m": 0.05},
        {"action": "fill C", "kind": "fill", "x": 0, "y": 130, "footprint_m2": 16, "depth_m": 0.05},
        {"action": "fill D", "kind": "fill", "x": 140, "y": 30, "footprint_m2": 16, "depth_m": 0.05}]})


def test_execute_leg_truth_is_at_least_the_nominal_plan():
    import autonomy as A
    dem = MP.load_haworth_dem(); o = MP.flattest_anchor(dem)
    m = _spread()
    trips, _, _, _ = MP._build_trips(m, dem, o, 25.0)
    b = A.initial_belief(m, len(trips))
    leg = trips[0]
    t = A.execute_leg(b, leg, dem=dem, dem_origin=o, body="moon")
    nom = A.nominal_leg_energy_J((b.x, b.y), leg)
    assert t["drive_m"] > 0.0
    assert t["true_energy_J"] >= nom - 1e-6                    # slip + gravity climb only ADD to the flat plan
    assert 0.0 <= t["slip"] < 1.0


def test_closed_loop_completes_and_manages_the_battery():
    import autonomy as A
    dem = MP.load_haworth_dem(); o = MP.flattest_anchor(dem)
    r = A.run_closed_loop(_spread(), dem=dem, dem_origin=o, algorithm="nearest", objective="time")
    assert r["completed"] is True
    assert r["belief"].tasks_done == r["belief"].tasks_total == r["n_trips"]
    assert r["belief"].energy_J >= 0.0                         # never depleted — recharges before reserve
    assert r["recharges"] >= 1                                 # the loop actually managed the battery
    assert all(-1e-9 <= L["soc"] <= 1.0001 for L in r["legs"])


def test_true_drain_never_below_nominal_and_uncertainty_grows():
    # AutoNav model-vs-truth: the slip-adjusted truth is never cheaper than the flat nominal plan, and the
    # estimate carries growing uncertainty (the loop replans against the estimate, not assumed-perfect state).
    import autonomy as A
    dem = MP.load_haworth_dem(); o = MP.flattest_anchor(dem)
    r = A.run_closed_loop(_spread(), dem=dem, dem_origin=o)
    tot_true = sum(L["true_J"] for L in r["legs"])
    tot_nom = sum(L["nominal_J"] for L in r["legs"])
    assert tot_true >= tot_nom - 1e-6
    # pose uncertainty grows monotonically with travel (dead-reckoning, never reset); energy sigma is
    # legitimately reset to 0 by a full recharge, so it's not a reliable end-of-run signal.
    assert r["belief"].pos_sigma_m > 0.0
    assert max(L["energy_sigma_J"] for L in r["legs"]) > 0.0    # energy uncertainty WAS carried in the loop


def test_perception_in_the_loop_bounds_pose_uncertainty():
    # with a per-leg map/landmark pose fix, the dead-reckoning drift is BOUNDED (vs growing without it),
    # and the result stays below the dig-ready gate. Perception is now folded into the loop.
    import autonomy as A
    dem = MP.load_haworth_dem()
    o = MP.flattest_anchor(dem)
    off = A.run_closed_loop(_spread(), dem=dem, dem_origin=o)                      # perception OFF (dead-reckoning)
    on = A.run_closed_loop(_spread(), dem=dem, dem_origin=o,                       # perception ON
                           perception_sigma_m=0.10, dig_sigma_gate_m=0.20)
    assert on["belief"].pos_sigma_m < off["belief"].pos_sigma_m                    # fixes bound the drift
    assert on["belief"].pos_sigma_m <= 0.20 + 1e-6                                 # below the dig-ready gate
    assert on["perception_fixes"] >= 1 and on["observe_more"] >= 0
    assert on["completed"] is True
