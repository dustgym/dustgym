"""STRUCTURAL tests for the Lane-C pose scorer (report-only -- NO threshold gate).

These mirror the test_frames.py convention: a pure-python runner (`python3 test_score_pose.py`)
that is ALSO pytest-discoverable, no pytest required.  They assert the scorer RUNS, emits a
VALID `Scorecard`, and that `n_frames` equals the count of non-null truth samples -- they do
NOT assert any error threshold (there is no acceptance bar in this repo; this lane reports
numbers only).

CC0-1.0 (see ../../LICENSE).
"""

from __future__ import annotations

import os
import sys

import eval_schema as es
import score_pose
import synthetic_feed as sf

# Resolve samples/ relative to the repo root (../../ from scripts/ros2_bridge/).
_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.normpath(os.path.join(_HERE, "..", ".."))


def _scene_dir(name: str) -> str:
    return os.path.join(_REPO, "samples", name)


def _nonnull_truth_count(scene_dir: str) -> int:
    paths = sf.scene_frame_paths(scene_dir)
    frames = es.load_scene_frames(paths)
    return sum(1 for f in frames if f.get("rover_rc") is not None)


def test_scorecard_is_valid_and_nframes_matches():
    """A synthetic run emits a valid Scorecard whose n_frames == non-null truth-sample count.

    Runs against whichever of the two trajectory scenes are populated in this checkout (the
    motion frames are gitignored/regenerable); skips a scene only if it is absent so the test
    stays green on a bookend-only checkout.  Structural only -- no error threshold asserted.
    """
    checked = 0
    for name in ("tread_track_4wheel", "tread_track"):
        scene_dir = _scene_dir(name)
        if not sf.scene_frame_paths(scene_dir):
            continue
        checked += 1
        truth, estimate = sf.synthesize(scene_dir, sf.NoiseConfig())
        card = score_pose.score_trajectory(truth, estimate)

        # Round-trips through the contract's JSON schema (valid Scorecard).
        card2 = es.Scorecard.from_json(card.to_json())
        assert card2.to_dict() == card.to_dict(), f"{name}: Scorecard JSON round-trip mismatch"

        # n_frames == number of non-null truth samples (frame_index match is exact + complete).
        expect = _nonnull_truth_count(scene_dir)
        assert card.n_frames == expect, (
            f"{name}: n_frames {card.n_frames} != non-null truth count {expect}")
        assert card.n_frames == len(truth), (
            f"{name}: n_frames {card.n_frames} != lifted truth len {len(truth)}")

        # Non-negative, finite metrics (report-only: value, not pass/fail, is the contract).
        for field in ("pose_rmse_trans_mm", "pose_rmse_yaw_deg", "ate_mm"):
            v = getattr(card, field)
            assert v >= 0.0 and v == v, f"{name}: {field} not finite/non-negative ({v})"

    assert checked > 0, "no trajectory scene populated; regenerate via terrain_authority.scenes"


def test_zero_noise_is_zero_error():
    """With zero injected noise the estimate IS truth, so all trajectory errors are exactly 0.

    Pins the metric definitions (a truth-vs-truth comparison must read 0), independent of any
    acceptance bar.  Uses whichever scene is populated.
    """
    scene_dir = None
    for name in ("tread_track_4wheel", "tread_track"):
        d = _scene_dir(name)
        if sf.scene_frame_paths(d):
            scene_dir = d
            break
    assert scene_dir is not None, "no trajectory scene populated"

    cfg = sf.NoiseConfig(sigma_trans_m=0.0, sigma_yaw_rad=0.0)
    truth, estimate = sf.synthesize(scene_dir, cfg)
    card = score_pose.score_trajectory(truth, estimate)
    assert card.n_frames == len(truth) and card.n_frames > 0
    assert abs(card.pose_rmse_trans_mm) < 1e-9, card.pose_rmse_trans_mm
    assert abs(card.pose_rmse_yaw_deg) < 1e-9, card.pose_rmse_yaw_deg
    assert abs(card.ate_mm) < 1e-9, card.ate_mm


def test_apriltag_channel_mirrors_compare_pose():
    """The apriltag scorer reproduces compare_pose.py's mm + geodesic-deg definitions.

    Identity orientation vs a 90-deg-about-Z orientation -> 90 deg; a 0.3 m offset -> 300 mm.
    Confirms the channel is wired to the frozen compare_pose.rotation_error_deg and stays
    separate from the trajectory Scorecard.
    """
    res = score_pose.score_apriltag(
        detected_pos_m=[0.3, 0.0, 0.0], detected_quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        truth_pos_m=[0.0, 0.0, 0.0],
        truth_quat_xyzw=[0.0, 0.0, 0.7071067811865476, 0.7071067811865476],  # +90 deg about Z
    )
    assert abs(res["apriltag_trans_err_mm"] - 300.0) < 1e-6, res["apriltag_trans_err_mm"]
    assert abs(res["apriltag_rot_err_deg"] - 90.0) < 1e-6, res["apriltag_rot_err_deg"]


def test_quantization_floor_is_surfaced():
    """The ~20 mm rover_rc quantization floor is exposed as a constant (surfaced, not asserted)."""
    assert abs(score_pose.QUANTIZATION_FLOOR_MM - 20.0) < 1e-9


def _run_all():
    tests = [
        ("scorecard_is_valid_and_nframes_matches", test_scorecard_is_valid_and_nframes_matches),
        ("zero_noise_is_zero_error", test_zero_noise_is_zero_error),
        ("apriltag_channel_mirrors_compare_pose", test_apriltag_channel_mirrors_compare_pose),
        ("quantization_floor_is_surfaced", test_quantization_floor_is_surfaced),
    ]
    failures = 0
    for name, fn in tests:
        try:
            fn()
            print(f"PASS  test_{name}")
        except Exception as exc:  # noqa: BLE001  (test harness wants to keep going)
            failures += 1
            print(f"FAIL  test_{name}: {exc}")
    if failures:
        print(f"\n{failures} test(s) FAILED")
        return 1
    print(f"\nAll {len(tests)} structural scorer tests passed.")
    return 0


if __name__ == "__main__":
    sys.exit(_run_all())
