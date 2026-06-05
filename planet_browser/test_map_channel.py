"""Tests for the in-loop map-channel reward (P6 / LAC section 10), the onboard-observability tier.

Real planner missions; the worksite + coverage are computed from the conserved order frame (no synthetic
data). The dense reconstruction RMSE is the separate, gated render/COLMAP tier (not exercised here).
"""
from __future__ import annotations

from . import map_channel as MC
from . import mission_planner as MP


def _mission():
    return MP.mission_from_dict({"name": "m", "body": "moon", "charger": [0, 0], "orders": [
        {"action": "cut", "kind": "cut", "x": 0, "y": 0, "footprint_m2": 36, "depth_m": 0.1},
        {"action": "fill", "kind": "fill", "x": 30, "y": 0, "footprint_m2": 36, "depth_m": 0.1}]})


def test_coverage_grows_with_stations_and_bounds():
    m = _mission()
    one = MC.map_channel_score(m, [(0.0, 0.0)])
    more = MC.map_channel_score(m, [(0.0, 0.0), (15.0, 0.0), (30.0, 0.0)])
    assert 0.0 <= one["coverage"] <= 1.0 and 0.0 <= more["coverage"] <= 1.0
    assert more["coverage"] > one["coverage"]                         # observing from more stations sees more
    # residual map uncertainty sits between the onboard-observed sigma and the unobserved prior
    assert MC.ONBOARD_STEREO_SIGMA_M <= more["mean_uncertainty_m"] <= MC.PRIOR_SIGMA_M
    assert more["dense_rmse_available"] is False                      # the dense RMSE is the gated tier


def test_blanketing_the_worksite_reaches_full_coverage():
    m = _mission()
    x0, y0, x1, y1 = MC.worksite_bbox(m)
    grid = [(x, y) for x in range(int(x0), int(x1) + 1, 4) for y in range(int(y0), int(y1) + 1, 4)]
    full = MC.map_channel_score(m, grid)
    assert full["coverage"] > 0.99 and full["mean_uncertainty_m"] == MC.ONBOARD_STEREO_SIGMA_M


def test_local_coverage_gate_signal():
    # a site the rover has stood on is well-covered locally; a far, unvisited site is not
    assert MC.local_coverage([(0.0, 0.0)], (0.0, 0.0)) > MC.COVERAGE_DIG_GATE
    assert MC.local_coverage([(0.0, 0.0)], (100.0, 100.0)) == 0.0
