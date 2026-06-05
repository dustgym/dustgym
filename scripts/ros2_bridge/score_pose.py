"""Two INDEPENDENT pose scorers for the Lane-C evaluation (report-only, no CI gate).

This module COMPUTES and REPORTS error metrics; it emits NO pass/fail and asserts NO acceptance
threshold (none exist in the repo -- inventing one would be portfolio-fraudulent).  Two
*separate* truth channels are scored and ALWAYS reported side by side; they are NEVER summed,
averaged, or folded into a single number (see eval_schema module docstring):

  (a) TRAJECTORY channel  -> `score_trajectory(...)` -> a `Scorecard`:
        pose_rmse_trans_mm  raw translation RMSE over frame_index-matched samples (mm)
        pose_rmse_yaw_deg   yaw-only rotation RMSE (deg); the synthetic channel has no
                            roll/pitch truth, so yaw is the only meaningful rotation metric
        ate_mm              Absolute Trajectory Error (trans-only, mm), reported as the
                            Umeyama-similarity-ALIGNED RMSE per the TUM RGB-D ATE convention
                            (Sturm et al., "A Benchmark for the Evaluation of RGB-D SLAM
                            Systems," IROS 2012).  For the synthetic same-frame estimate the
                            estimate IS truth+noise in the same world frame, so the optimal
                            alignment is ~identity and ate_mm coincides with the raw RMSE;
                            we still run the alignment so the field is the canonical aligned
                            ATE the moment real (drifted/mis-framed) M2 data flows in.
        n_frames            count of matched samples scored == count of non-null truth samples.

  (b) APRILTAG single-pose channel -> `score_apriltag(...)` -> a plain dict (lives OUTSIDE the
        trajectory Scorecard).  MIRRORS compare_pose.py exactly: translation error * 1000 mm
        (Euclidean) and the geodesic `rotation_error_deg` (full quaternion).  We REUSE
        compare_pose.rotation_error_deg rather than re-deriving it, so the two stay aligned and
        compare_pose.py is never edited.

ASSOCIATION (synthetic): exact match on `frame_index` -- dependency-free and exact for the
synthetic stream where estimate frame indices are copied from truth.  The future live M2 path
will instead need a nearest-`t_s` association (estimate timestamps will not equal truth
indices); a clean hook for that is left in `match_by_frame_index` / `_nearest_t_s_hook` but the
live path is intentionally NOT implemented here.

QUANTIZATION FLOOR: truth is lifted from integer rover_rc cells at cell_m == 0.02 m
(terrain_authority/constants.py CELL_M), so trans/ATE cannot resolve below ~20 mm.  This is
surfaced by the harness, not asserted (report-only).

Pure stdlib + numpy.  No ROS/rclpy import; safe on the bare host .venv.

CC0-1.0 (see ../../LICENSE).
"""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np

import eval_schema as es


def _import_compare_pose_rotation():
    """Import `rotation_error_deg` from the FROZEN compare_pose.py without editing it.

    compare_pose.py is a ROS node: it does `from rclpy.node import Node` under a try/except,
    but its module-level `class ComparePose(Node)` references `Node` unconditionally, so a bare
    host import (no rclpy) raises `NameError: Node` at class-definition time -- before we can
    reach `rotation_error_deg`.  Rather than touch the frozen file, we install lightweight stub
    modules for `rclpy` + the message packages it pulls so its import path succeeds on the bare
    .venv.  `rotation_error_deg` is pure-numpy and stub-independent; this just lets us CALL the
    frozen reference (no duplication, no edit) so the apriltag channel stays bit-aligned with
    compare_pose.py.
    """
    import sys as _sys
    import types as _types

    installed: list[str] = []
    needed = {
        "rclpy": ["Node"],          # not real attrs; placeholder modules with a Node symbol
        "rclpy.node": ["Node"],
        "geometry_msgs": [],
        "geometry_msgs.msg": ["PoseStamped"],
        "tf2_msgs": [],
        "tf2_msgs.msg": ["TFMessage"],
    }
    try:
        for mod_name, attrs in needed.items():
            if mod_name not in _sys.modules:
                stub = _types.ModuleType(mod_name)
                for a in attrs:
                    # `Node` must be a usable base class for `class ComparePose(Node)`.
                    setattr(stub, a, type(a, (object,), {}))
                _sys.modules[mod_name] = stub
                installed.append(mod_name)
        from compare_pose import rotation_error_deg as _red
        return _red
    finally:
        # Leave the import space exactly as we found it (only remove stubs WE installed).
        for mod_name in installed:
            _sys.modules.pop(mod_name, None)


# AprilTag-channel rotation math is the frozen compare_pose.py reference; we CALL it, never
# duplicate or edit it, so the two channels report identically-defined geodesic error.
rotation_error_deg = _import_compare_pose_rotation()

# rover_rc grid resolution -> hard synthetic resolution floor (mm).  Sourced from
# terrain_authority/constants.py CELL_M == 0.02 m; surfaced (not asserted) because report-only.
CELL_M = 0.02
QUANTIZATION_FLOOR_MM = CELL_M * 1000.0  # 20.0 mm


# --- association -------------------------------------------------------------------------

def match_by_frame_index(
    truth: Sequence[es.TrajectorySample],
    estimate: Sequence[es.TrajectorySample],
) -> list[tuple[es.TrajectorySample, es.TrajectorySample]]:
    """Pair truth and estimate samples by exact `frame_index` (the --synthetic association).

    Dependency-free and exact: the synthetic feed copies frame_index from truth, so every
    truth frame has exactly one estimate partner.  Returned in truth order.  Frames present on
    only one side are dropped (no fabricated pairs).
    """
    est_by_idx = {s.frame_index: s for s in estimate}
    pairs: list[tuple[es.TrajectorySample, es.TrajectorySample]] = []
    for t in truth:
        e = est_by_idx.get(t.frame_index)
        if e is not None:
            pairs.append((t, e))
    return pairs


def _nearest_t_s_hook(*_args, **_kwargs):
    """RESERVED hook for the live M2 path: nearest-`t_s` association.

    M2 estimate timestamps will NOT equal truth frame indices, so the live scorer will need to
    pair each estimate to its nearest-in-time truth sample (within a tolerance) instead of the
    exact frame_index match used for --synthetic.  Intentionally NOT implemented in this lane
    (synthetic-only); left here as a documented seam so the live path slots in without
    restructuring the scorer.
    """
    raise NotImplementedError(
        "nearest-t_s association is the live M2 path; --synthetic uses exact frame_index "
        "matching (match_by_frame_index)"
    )


# --- (a) trajectory channel --------------------------------------------------------------

def _umeyama_alignment_2d(
    src_xz: np.ndarray, dst_xz: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Optimal rigid (rotation+translation, no scale) alignment of src onto dst in the x-z plane.

    Umeyama (1991), as used by the TUM RGB-D ATE benchmark (Sturm 2012): the ATE is computed on
    the estimated trajectory AFTER the least-squares similarity transform that best maps it onto
    the ground truth, so a constant frame offset / global yaw is not charged as trajectory error.
    We restrict to the 2-D ground plane (x, z) because the synthetic channel has no y-truth, and
    we omit the scale factor (rigid only) -- pose trajectories are metric, not up-to-scale.

    Returns (R 2x2, t 2,) such that dst ~= R @ src + t.  Degenerate (<2 points or zero-variance)
    inputs fall back to identity rotation + mean offset, which is exact for the synthetic case.
    """
    src = np.asarray(src_xz, dtype=np.float64)
    dst = np.asarray(dst_xz, dtype=np.float64)
    n = src.shape[0]
    if n == 0:
        return np.eye(2), np.zeros(2)
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    sc = src - mu_s
    dc = dst - mu_d
    if n < 2 or np.allclose(sc, 0.0):
        return np.eye(2), mu_d - mu_s
    cov = (dc.T @ sc) / n
    u, _s, vt = np.linalg.svd(cov)
    d = np.array([1.0, np.sign(np.linalg.det(u @ vt))])
    r = u @ np.diag(d) @ vt
    t = mu_d - r @ mu_s
    return r, t


def score_trajectory(
    truth: Sequence[es.TrajectorySample],
    estimate: Sequence[es.TrajectorySample],
) -> es.Scorecard:
    """Score the synthetic trajectory channel -> a `Scorecard` (mm / deg).

    * pose_rmse_trans_mm: RMSE of the raw per-frame Euclidean translation error (metres->mm).
    * ate_mm: Umeyama-aligned trans RMSE (TUM RGB-D ATE convention; see module docstring).  For
      same-frame synthetic data this coincides with the raw RMSE (alignment ~identity).
    * pose_rmse_yaw_deg: RMSE of the per-frame yaw error, each error wrapped to (-pi, pi] so a
      359-vs-1-degree pair scores ~2 deg, not ~358.
    * n_frames: number of frame_index-matched pairs (== non-null truth sample count).

    Returns a zeroed Scorecard with n_frames == 0 when there is nothing to match (still a valid,
    reportable Scorecard -- report-only, no gate).
    """
    pairs = match_by_frame_index(truth, estimate)
    n = len(pairs)
    if n == 0:
        return es.Scorecard(
            pose_rmse_trans_mm=0.0, pose_rmse_yaw_deg=0.0, ate_mm=0.0, n_frames=0)

    truth_xz = np.array([[t.position_m[0], t.position_m[2]] for t, _ in pairs])
    est_xz = np.array([[e.position_m[0], e.position_m[2]] for _, e in pairs])

    # Raw translation RMSE (mm).  Full 3-D norm, but y is the constant ground plane on both
    # sides so it contributes 0; computing in 3-D keeps the metric honest if y-truth ever lands.
    truth_xyz = np.array([list(t.position_m) for t, _ in pairs], dtype=np.float64)
    est_xyz = np.array([list(e.position_m) for _, e in pairs], dtype=np.float64)
    trans_err_m = np.linalg.norm(est_xyz - truth_xyz, axis=1)
    pose_rmse_trans_mm = float(np.sqrt(np.mean(trans_err_m ** 2)) * 1000.0)

    # Umeyama-aligned ATE (mm): best rigid map of estimate onto truth, then RMSE of residuals.
    r, t = _umeyama_alignment_2d(est_xz, truth_xz)
    est_aligned = (r @ est_xz.T).T + t
    ate_err_m = np.linalg.norm(truth_xz - est_aligned, axis=1)
    ate_mm = float(np.sqrt(np.mean(ate_err_m ** 2)) * 1000.0)

    # Yaw RMSE (deg), each residual wrapped to (-pi, pi].
    yaw_errs = []
    for t_s, e_s in pairs:
        dy = es.quat_xyzw_to_yaw(e_s.quaternion_xyzw) - es.quat_xyzw_to_yaw(t_s.quaternion_xyzw)
        dy = math.atan2(math.sin(dy), math.cos(dy))  # wrap to (-pi, pi]
        yaw_errs.append(dy)
    yaw_errs = np.asarray(yaw_errs, dtype=np.float64)
    pose_rmse_yaw_deg = float(np.degrees(np.sqrt(np.mean(yaw_errs ** 2))))

    return es.Scorecard(
        pose_rmse_trans_mm=pose_rmse_trans_mm,
        pose_rmse_yaw_deg=pose_rmse_yaw_deg,
        ate_mm=ate_mm,
        n_frames=n,
    )


# --- (b) apriltag single-pose channel ----------------------------------------------------

def score_apriltag(
    detected_pos_m: Sequence[float],
    detected_quat_xyzw: Sequence[float],
    truth_pos_m: Sequence[float],
    truth_quat_xyzw: Sequence[float],
) -> dict:
    """Score one camera->tag pose, MIRRORING compare_pose.py (`_maybe_report`).

    translation error: Euclidean norm of (detected - truth) position, * 1000 -> mm (identical to
    compare_pose's `terr * 1000`).  rotation error: the frozen `compare_pose.rotation_error_deg`
    geodesic angle (deg) -- CALLED, not re-derived.  Returned as a standalone dict; this metric
    lives OUTSIDE the trajectory Scorecard and is never merged with it.
    """
    dt = np.asarray(detected_pos_m, dtype=np.float64)
    tt = np.asarray(truth_pos_m, dtype=np.float64)
    trans_err_mm = float(np.linalg.norm(dt - tt) * 1000.0)
    rot_err_deg = rotation_error_deg(detected_quat_xyzw, truth_quat_xyzw)
    return {
        "apriltag_trans_err_mm": trans_err_mm,
        "apriltag_rot_err_deg": rot_err_deg,
    }
