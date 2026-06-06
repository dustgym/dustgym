"""Tests for map-relative localization (the scan-to-DEM 'overlay', P15 step 1).

Registers an observed elevation patch SENSED FROM THE CONSERVED-TRUTH DEM and recovers a perturbed pose --
no synthetic terrain (the real Haworth DEM is both the map and the truth the patch is sensed from). The flat
region uses a controlled zero patch to assert the ambiguity (low confidence) case.
"""
from __future__ import annotations

import numpy as np

from . import localization as LOC
from . import mission_planner as MP


def _textured_cell(Z, half):
    # a cell with real relief in its neighbourhood (so the match is well-conditioned), away from the edge
    H, W = Z.shape
    sub = Z[half + 5:H - half - 5, half + 5:W - half - 5]
    rr, cc = np.unravel_index(int(np.argmax(np.abs(np.gradient(sub)[0]))), sub.shape)
    return (rr + half + 5, cc + half + 5)


def test_recovers_a_perturbed_pose_on_real_haworth():
    dem = MP.load_haworth_dem(); Z, _cell = dem
    half = 6
    true_rc = _textured_cell(Z, half)
    observed = LOC.patch_at(Z, true_rc, half)                       # the rover senses the true terrain here
    guess_rc = (true_rc[0] + 3, true_rc[1] - 2)                     # ...but its belief has drifted
    out = LOC.register_to_dem(observed, dem, guess_rc, search_radius_cells=5)
    assert out["shift_cells"] == (-3, 2)                            # the correction recovers the drift exactly
    assert out["corrected_rc"] == true_rc                          # -> back to the true cell
    assert out["residual_rmse_m"] < 1e-6                           # an exact shape match
    assert out["confidence"] > 0.5                                 # textured terrain -> a sharp, confident peak


def test_drift_beyond_search_radius_is_not_fully_recovered():
    dem = MP.load_haworth_dem(); Z, _cell = dem
    half = 6
    true_rc = _textured_cell(Z, half)
    observed = LOC.patch_at(Z, true_rc, half)
    guess_rc = (true_rc[0] + 20, true_rc[1])                        # drift exceeds the +/-5 search window
    out = LOC.register_to_dem(observed, dem, guess_rc, search_radius_cells=5)
    assert abs(out["shift_cells"][0]) <= 5                          # bounded by the search radius (honest limit)
    assert out["corrected_rc"] != true_rc                          # cannot jump 20 cells -> not fully recovered


def test_flat_region_is_ambiguous_low_confidence():
    Z = np.zeros((40, 40), dtype=float)                            # a featureless flat map -> no shape to match
    observed = LOC.patch_at(Z, (20, 20), 6)
    out = LOC.register_to_dem(observed, (Z, 5.0), (22, 18), search_radius_cells=5)
    assert out["confidence"] == 0.0                               # ambiguous: every shift matches equally
