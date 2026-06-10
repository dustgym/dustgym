"""Tests for the slope-aware route planner over a real Haworth DEM crop (no synthetic terrain)."""
from __future__ import annotations

import os

import pytest

from flight import load_crop
from route import plan_route, slope_deg, snap_to_navigable

_SCENE = os.path.join(os.path.dirname(__file__), "..", "..", "samples", "lunar_dem", "haworth_10km_5m")
_HAVE_SCENE = os.path.isdir(_SCENE)
pytestmark = pytest.mark.skipif(not _HAVE_SCENE, reason="Haworth sample not present")


def _crop(win=60):
    # the navigable rim-crest plateau (see CONTRACT.md / run_demo defaults)
    return load_crop(_SCENE, 720, 1800, win, win)


def test_route_reaches_goal_on_navigable_terrain():
    crop = _crop()
    sl = slope_deg(crop.heightmap, crop.cell_m)
    start = snap_to_navigable(sl, (10, 10), 18.0)
    goal = snap_to_navigable(sl, (50, 50), 18.0)
    wps = plan_route(crop.heightmap, crop.cell_m, start, goal, max_slope_deg=18.0, n_waypoints=5)
    assert wps, "expected a navigable route across the rim-crest plateau"
    # last waypoint is the goal cell
    assert (round(wps[-1][0]), round(wps[-1][1])) == goal
    # every waypoint is in bounds and on a navigable (<= cap) cell
    H, W = crop.heightmap.shape
    for r, c in wps:
        assert 0 <= r < H and 0 <= c < W
        assert sl[int(round(r)), int(round(c))] <= 18.0 + 1e-6


def test_snap_returns_navigable_cell():
    crop = _crop()
    sl = slope_deg(crop.heightmap, crop.cell_m)
    r, c = snap_to_navigable(sl, (0, 0), 8.0)
    assert sl[r, c] <= 8.0 + 1e-6
