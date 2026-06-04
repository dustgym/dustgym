"""STRUCTURAL tests for the Lane-C MAP-channel scorer (report-only -- NO threshold gate).

Mirror the test_score_pose.py convention: a pure-python runner (`python3 test_score_map.py`) that is
ALSO pytest-discoverable.  They assert the scorer's identities on REAL Haworth DEM data (truth-vs-truth
is perfect; a real lower-resolution reconstruction scores worse; tolerance/mask move the metrics the
right way) and the rock-F1 detection identities -- they do NOT assert any acceptance bar (none exists).

The OBSERVED map is a REAL lower-fidelity version of the real DEM (block-mean downsample then upsample --
a real reconstruction, like a thumbnail; subsampling real data, never fabricated).

CC0-1.0 (see ../../LICENSE).
"""

from __future__ import annotations

import os

import numpy as np

import eval_schema as es
import score_map as sm

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.normpath(os.path.join(_HERE, "..", ".."))
_BUNDLE = os.path.join(_REPO, "samples", "lunar_dem", "haworth_10km_5m")


def _haworth_crop(n=64):
    """A real Haworth height crop [m] (top-left n x n of the real LOLA 5 m DEM)."""
    Z = np.fromfile(os.path.join(_BUNDLE, "heightmap.rf32"), dtype="<f4").reshape(2000, 2000)
    return Z[:n, :n].astype(np.float64)


def _coarsen(Z, block):
    """Real lower-resolution reconstruction: block-mean downsample then upsample back to shape."""
    n = Z.shape[0] - Z.shape[0] % block
    Zc = Z[:n, :n]
    small = Zc.reshape(n // block, block, n // block, block).mean(axis=(1, 3))
    return np.kron(small, np.ones((block, block)))


def test_identity_is_perfect():
    Z = _haworth_crop()
    r = sm.score_map(Z, Z, tol_m=0.10)
    assert r["map_rmse_m"] == 0.0 and r["map_cell_pass_frac"] == 1.0


def test_coarser_reconstruction_scores_worse_monotonically():
    Z = _haworth_crop()
    o2, o4, o8 = _coarsen(Z, 2), _coarsen(Z, 4), _coarsen(Z, 8)
    # tol_m = 1.0 is a sensible cell-pass band for a 5 m polar DEM (per-cell relief ~0.45 m median);
    # at 0.10 m almost nothing passes and the tiny tail is not informative.
    r2 = sm.score_map(o2, Z, tol_m=1.0)
    r4 = sm.score_map(o4, Z, tol_m=1.0)
    r8 = sm.score_map(o8, Z, tol_m=1.0)
    assert 0.0 < r2["map_rmse_m"] < r4["map_rmse_m"] < r8["map_rmse_m"]   # coarser -> larger RMSE
    assert r2["map_cell_pass_frac"] > r4["map_cell_pass_frac"] > r8["map_cell_pass_frac"]
    assert all(0.0 <= r["map_cell_pass_frac"] <= 1.0 for r in (r2, r4, r8))


def test_tolerance_and_mask_move_metrics_correctly():
    Z = _haworth_crop()
    obs = _coarsen(Z, 4)
    loose = sm.score_map(obs, Z, tol_m=0.50)["map_cell_pass_frac"]
    tight = sm.score_map(obs, Z, tol_m=0.05)["map_cell_pass_frac"]
    assert loose >= tight                                                # looser tolerance passes more cells
    # masking out the worst-error cells (observed unobserved/occluded) raises pass_frac
    err = np.abs(obs - Z)
    keep = err <= np.median(err)                                         # keep the better half (a valid_mask)
    masked = sm.score_map(obs, Z, tol_m=0.05, valid_mask=keep)["map_cell_pass_frac"]
    assert masked >= tight


def test_rock_f1_detection_identities():
    truth = [(1.0, 1.0), (5.0, 5.0), (9.0, 2.0)]
    assert sm.score_map(np.zeros((2, 2)), np.zeros((2, 2)), truth_rocks=truth,
                        observed_rocks=truth, rock_match_m=0.5)["rock_f1"] == 1.0          # perfect
    half = sm.score_map(np.zeros((2, 2)), np.zeros((2, 2)), truth_rocks=truth,
                        observed_rocks=[(1.0, 1.0)], rock_match_m=0.5)["rock_f1"]           # recall 1/3
    assert abs(half - (2 * 1.0 * (1 / 3) / (1.0 + 1 / 3))) < 1e-9
    # spurious detections cut precision below 1
    spur = sm.score_map(np.zeros((2, 2)), np.zeros((2, 2)), truth_rocks=truth,
                        observed_rocks=truth + [(50.0, 50.0)], rock_match_m=0.5)["rock_f1"]
    assert spur < 1.0
    assert sm.score_map(np.zeros((2, 2)), np.zeros((2, 2)))["rock_f1"] is None             # no rock lists


def test_attach_map_metrics_leaves_pose_channel_untouched():
    Z = _haworth_crop()
    base = es.Scorecard(pose_rmse_trans_mm=12.7, pose_rmse_yaw_deg=1.5, ate_mm=11.0, n_frames=80)
    out = sm.attach_map_metrics(base, _coarsen(Z, 4), Z, tol_m=0.10)
    assert out.pose_rmse_trans_mm == 12.7 and out.n_frames == 80          # pose channel preserved
    assert out.map_rmse_m is not None and out.map_cell_pass_frac is not None
    assert base.map_rmse_m is None                                        # original untouched (copy)


def test_harness_run_map_emits_non_null_map_metrics():
    # PRD P6: the harness wires the map channel as the 2nd eval channel; on a real DEM pair it emits
    # non-null map metrics, with the pose channel zeroed (the two are reported side by side, never summed).
    import eval_harness as eh
    Z = _haworth_crop()
    report = eh.run_map(Z, _coarsen(Z, 4), tol_m=1.0)
    sc = report["scorecard"]
    assert report["mode"] == "map" and report["report_only"] is True
    assert sc["map_rmse_m"] is not None and sc["map_cell_pass_frac"] is not None
    assert sc["pose_rmse_trans_mm"] == 0.0 and sc["n_frames"] == 0          # pose channel zeroed, not summed


if __name__ == "__main__":                                               # pure-python runner, no pytest needed
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn(); print(f"ok  {name}")
    print("score_map: all checks passed")
