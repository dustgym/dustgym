"""Tests for the section-10 map-channel observed-map PRODUCER (obs_map_producer).

Pure-python runner (`python3 test_obs_map_producer.py`) that is ALSO pytest-discoverable, mirroring
test_score_map.py. The geometry/gridding is asserted by a REAL-DEM round-trip (real Haworth heights
-> world points via the terrain.gd inversion -> back onto the grid == identity) and a small median/
mask identity; the full stereo pipeline is asserted on a REAL Godot front-stereo drive egress when
one is present. The drive egress is render output (not committed), so those tests SKIP when it is
absent -- the same skip-if-no-artifact convention the rest of the bridge uses. No fabricated data.

CC0-1.0 (see ../../LICENSE).
"""

from __future__ import annotations

import os

import numpy as np
import pytest

pytest.importorskip("cv2")   # T4 ROS-bridge: obs_map_producer needs OpenCV (container-only) -> skip on the bare CI runner

import obs_map_producer as omp  # noqa: E402
from score_map import score_map  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.normpath(os.path.join(_HERE, "..", ".."))
_BUNDLE = os.path.join(_REPO, "samples", "lunar_dem", "haworth_10km_5m")
_SCENE = os.path.join(_REPO, "samples", "crater_boulders")
_DRIVE = os.path.join(_REPO, "godot_sidecar", "out", "cam", "crater_boulders", "drive")


def _stations():
    if not os.path.isdir(_DRIVE):
        return []
    return [os.path.join(_DRIVE, d) for d in sorted(os.listdir(_DRIVE))
            if os.path.isfile(os.path.join(_DRIVE, d, "sensors.json"))]


def _skip_if_no_drive():
    if not _stations():
        import pytest
        pytest.skip("no front-stereo drive egress (Godot render output, not committed)")


# --------------------------------------------------------------- pure geometry (always runs)
def test_quat_to_R_identity_and_known_rotation():
    assert np.allclose(omp.quat_xyzw_to_R([0, 0, 0, 1]), np.eye(3))
    # +90 deg about Y (xyzw): in a right-handed Y-up frame this sends +X -> -Z.
    R = omp.quat_xyzw_to_R([0, np.sin(np.pi / 4), 0, np.cos(np.pi / 4)])
    assert np.allclose(R @ np.array([1.0, 0.0, 0.0]), [0.0, 0.0, -1.0], atol=1e-9)


def test_grid_roundtrip_on_real_dem():
    # REAL Haworth heights -> world points (terrain.gd:164 mapping) -> grid_to_heightfield == identity.
    n = 48
    Z = np.fromfile(os.path.join(_BUNDLE, "heightmap.rf32"), dtype="<f4").reshape(2000, 2000)
    Z = Z[:n, :n].astype(np.float64)
    grid = {"width": n, "height": n, "cell_m": 0.5, "x0": 0.0, "y0": 0.0}
    rows, cols = np.mgrid[0:n, 0:n]
    gx = grid["x0"] + (cols + 0.5) * grid["cell_m"]   # cell centers
    gz = grid["y0"] + (rows + 0.5) * grid["cell_m"]
    pts = np.stack([gx.ravel(), Z.ravel(), gz.ravel()], axis=1)
    obs, mask = omp.grid_to_heightfield(pts, grid)
    assert mask.all()                                  # every cell received its point
    assert np.allclose(obs, Z, atol=1e-9)              # exact round-trip of the height inversion


def test_grid_medians_duplicates_and_masks_unobserved():
    grid = {"width": 4, "height": 4, "cell_m": 1.0, "x0": 0.0, "y0": 0.0}
    # three points in cell (row 1, col 2): heights 1,2,9 -> median 2; all other cells unobserved.
    pts = np.array([[2.5, 1.0, 1.5], [2.5, 2.0, 1.5], [2.5, 9.0, 1.5]])
    obs, mask = omp.grid_to_heightfield(pts, grid)
    assert mask.sum() == 1 and mask[1, 2]
    assert obs[1, 2] == 2.0


def test_out_of_bounds_points_are_dropped():
    grid = {"width": 4, "height": 4, "cell_m": 1.0, "x0": 0.0, "y0": 0.0}
    pts = np.array([[-1.0, 5.0, 0.5], [10.0, 5.0, 0.5]])  # both outside [0,4) m
    obs, mask = omp.grid_to_heightfield(pts, grid)
    assert mask.sum() == 0


def test_identity_through_scorer_is_perfect():
    grid = omp.grid_from_metadata(os.path.join(_SCENE, "metadata.json"))
    truth = omp.load_truth_heightmap(_SCENE, grid)
    r = score_map(truth, truth, tol_m=0.10)
    assert r["map_rmse_m"] == 0.0 and r["map_cell_pass_frac"] == 1.0


# --------------------------------------------------- full stereo pipeline (needs a render egress)
def test_producer_recovers_ground_and_coverage_grows_on_real_render():
    _skip_if_no_drive()
    grid = omp.grid_from_metadata(os.path.join(_SCENE, "metadata.json"))
    truth = omp.load_truth_heightmap(_SCENE, grid)
    st = _stations()
    _, m1 = omp.produce_observed_map_multi(st[:1], grid)
    obsN, mN = omp.produce_observed_map_multi(st, grid)
    assert 0 < int(m1.sum()) < int(mN.sum())                       # coverage grows as the rover drives
    # ground plane recovered: observed median within 0.15 m of truth over the covered cells
    assert abs(float(np.median(obsN[mN])) - float(np.median(truth[mN]))) < 0.15
    sc = score_map(obsN, truth, tol_m=0.10, valid_mask=mN)         # feeds the scorer with finite metrics
    assert np.isfinite(sc["map_rmse_m"]) and 0.0 <= sc["map_cell_pass_frac"] <= 1.0


# ----------------------------------------------------------- Uncertainty layer (per-cell height sigma)
def test_uncertainty_grid_pure():
    grid = {"width": 4, "height": 4, "cell_m": 1.0, "x0": 0.0, "y0": 0.0}
    # cell (1,2): three identical heights -> zero scatter -> sigma 0; cell (0,0): one point -> prior.
    pts = np.array([[2.5, 1.0, 1.5], [2.5, 1.0, 1.5], [2.5, 1.0, 1.5], [0.5, 5.0, 0.5]])
    obs, sigma, count, mask = omp.grid_to_heightfield_uncertainty(pts, grid)
    assert count[1, 2] == 3 and count[0, 0] == 1
    assert sigma[1, 2] == 0.0                          # zero scatter -> zero standard error
    assert sigma[0, 0] == omp.PRIOR_SIGMA_M            # single view -> prior floor
    assert np.isinf(sigma[3, 3])                       # unobserved cell
    assert int(mask.sum()) == 2


def test_dig_ready_subset_of_observed():
    grid = {"width": 3, "height": 3, "cell_m": 1.0, "x0": 0.0, "y0": 0.0}
    pts = np.array([[0.5, 0.0, 0.5]] * 4 + [[2.5, 0.0, 2.5]])  # 4-view cell + 1-view cell
    _, sigma, _, mask = omp.grid_to_heightfield_uncertainty(pts, grid)
    ready = omp.dig_ready_mask(sigma, mask, tol_m=0.10)
    assert int(ready.sum()) <= int(mask.sum())
    assert ready[0, 0] and not ready[2, 2]             # confident cell ready; single-view (prior 0.30) not


def test_uncertainty_falls_with_more_views_on_real_render():
    _skip_if_no_drive()
    grid = omp.grid_from_metadata(os.path.join(_SCENE, "metadata.json"))
    st = _stations()
    _, _, c1, _ = omp.produce_uncertainty_map(st[:1], grid)
    _, sN, cN, mN = omp.produce_uncertainty_map(st, grid)
    assert int(cN.sum()) > int(c1.sum())                           # more driving -> more observations
    multi = cN >= 2
    assert multi.any() and bool(np.isfinite(sN[multi]).all())      # multi-view cells have finite sigma
    assert float(np.median(sN[multi])) < omp.PRIOR_SIGMA_M         # and lower uncertainty than the prior


if __name__ == "__main__":                                         # pure-python runner, no pytest needed
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"ok  {name}")
            except BaseException as e:                              # noqa: BLE001 -- runner reports skips too
                if type(e).__name__ == "Skipped":
                    print(f"skip {name}: {e}")
                else:
                    raise
    print("obs_map_producer: all checks passed")
