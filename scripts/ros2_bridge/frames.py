"""REP-103 frame conversion: the single Godot -> ROS seam (sensor_bridge_contract.md §3).

This module is the *only* place the Godot(Y-up) <-> ROS(Z-up, REP-103) coordinate trap
(INTERFACE.md §3, ipex-terrain-sim-spec.md §11) is solved.  `sensors.json` is 100%
Godot-native; `bag_writer.py` imports these functions and converts exactly once, on the way
into the rosbag.  A silent sign-flip here is the classic cause of plausible-but-wrong SLAM,
so the two normative point maps below are pinned and unit-tested in `test_frames.py`.

Conventions (contract §3):
  Godot world   : right-handed, +X right, +Y up,   +Z toward viewer (camera looks -Z).
  ROS world map : right-handed, +X forward, +Y left, +Z up (REP-103).
  ROS cam optical: +Z forward (into scene), +X right, +Y down.

Normative point maps:
  1. World  Y-up -> Z-up      (a -90 deg rotation about X):  (x,y,z)_ros  = ( x, -z,  y).
  2. Godot-cam -> ROS-optical (a 180 deg rotation about X):  (x,y,z)_opt = ( x, -y, -z).

Pure numpy; no ROS / rclpy dependency, so it runs on the host too.

Quaternions are XYZW order throughout (matching `sensors.json` `quaternion_xyzw` and ROS
`geometry_msgs/Quaternion` field order x,y,z,w).  Rotation matrices act on column vectors:
`v_out = R @ v_in`.

CC0-1.0 (see ../../LICENSE).
"""

from __future__ import annotations

import numpy as np

# --- Normative change-of-basis matrices (contract §3) -----------------------------------

# Map 1: Godot world (Y-up) -> ROS world (Z-up, REP-103).  R_W @ [x,y,z] = [x,-z,y].
R_WORLD_G2R = np.array(
    [[1.0, 0.0, 0.0],
     [0.0, 0.0, -1.0],
     [0.0, 1.0, 0.0]],
    dtype=np.float64,
)

# Map 2: Godot camera axes -> ROS optical axes.  R_OPT @ [x,y,z] = [x,-y,-z].
R_CAM_G2R = np.array(
    [[1.0, 0.0, 0.0],
     [0.0, -1.0, 0.0],
     [0.0, 0.0, -1.0]],
    dtype=np.float64,
)

# --- Lander frame -> apriltag tag-frame relabel (contract §1) ----------------------------
#
# `apriltag_ros` (christianrauch 3.x) reports a tag-frame whose ORIGIN is the tag centre but
# whose AXES follow the pose-estimator's convention -- and that convention differs between the
# two estimators it ships.  We run the **`pnp`** estimator (tags_36h11.yaml
# `pose_estimation_method: "pnp"`), i.e. raw `cv::solvePnP` against the object points
# {(-s/2,-s/2,0),(+s/2,-s/2,0),(+s/2,+s/2,0),(-s/2,+s/2,0)}; it does NOT apply the
# "swap x/y, invert z" fix-up that the `homography` estimator does.  EMPIRICALLY (verified by
# the M1 integration: a near-fronto-parallel tag reads q_xyzw=[0.998,0.001,0.007,-0.062] in the
# optical frame, i.e. ~R_x(180 deg)), this build's detector reports the tag axes as:
#   tag +X = image-right = optical +X ;  tag +Y = image-UP = optical -Y ;
#   tag +Z = OUT of the tag toward the camera = optical -Z.
# (So +Z is the OUTWARD normal here, not "into the tag" -- that latter wording applies to the
# `homography` estimator, which we are not using.)
#
# Our `lander` placement frame (contract §1) uses +X = the tag's OUTWARD normal (toward the
# rover), +Y = up.  The two frames share an origin (pose_in_lander identity) but differ by a
# FIXED rotation independent of the camera viewpoint, so identity pose_in_lander is NOT enough:
# the tag *orientation* must be relabelled into the detector's tag convention before comparison.
#
# Derivation from the actual axis definitions (sidecar.gd `_build_lander` + the QuadMesh, then
# pinned to the detector reading above):
#   * Detector tag +Z = outward normal toward the camera = lander +X.
#   * Detector tag +Y = image-up.  The QuadMesh (printed bitmap) is yawed +90 deg about lander
#     +Y; its rendered "up" runs along lander +Z, so tag +Y = lander +Z.
#   * Detector tag +X = image-right = lander +Y (right-handed completion).
# Columns of R_LANDER_TAG are therefore the detector tag axes expressed in lander coords:
#   tag +X = lander +Y ; tag +Y = lander +Z ; tag +Z = lander +X.
# This is a proper rotation (det=+1) -- a 120 deg cyclic axis-permutation -- applied by
# right-multiplying the tag's OWN-FRAME transform in bag_writer._compute_truth.  Because it is a
# constant own-frame relabel it corrects ALL camera viewpoints, not just the fronto-parallel one
# (an oblique view is exact too -- see test_frames.test_e); it is the principled replacement for
# the historical ~120 deg / q=[.5,.5,-.5,.5] orientation error.
R_LANDER_TAG = np.array(
    [[0.0, 0.0, 1.0],   # tag axes as columns in lander coords:
     [1.0, 0.0, 0.0],   #   col0 = tag+X = lander +Y
     [0.0, 1.0, 0.0]],  #   col1 = tag+Y = lander +Z ;  col2 = tag+Z = lander +X
    dtype=np.float64,
)


# --- Quaternion <-> matrix helpers (XYZW order) -----------------------------------------

def quat_xyzw_to_matrix(q: np.ndarray) -> np.ndarray:
    """Unit quaternion (x,y,z,w) -> 3x3 rotation matrix (column-vector convention)."""
    x, y, z, w = (float(c) for c in q)
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        raise ValueError("zero-norm quaternion")
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array(
        [[1.0 - (yy + zz), xy - wz, xz + wy],
         [xy + wz, 1.0 - (xx + zz), yz - wx],
         [xz - wy, yz + wx, 1.0 - (xx + yy)]],
        dtype=np.float64,
    )


def matrix_to_quat_xyzw(m: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix -> unit quaternion (x,y,z,w).  Numerically stable branch form."""
    m = np.asarray(m, dtype=np.float64)
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=np.float64)
    q /= np.linalg.norm(q)
    # Canonicalize sign (q and -q are the same rotation) -> w >= 0.
    if q[3] < 0.0:
        q = -q
    return q


# --- 4x4 homogeneous-transform helpers --------------------------------------------------

def make_transform(position: np.ndarray, quat_xyzw: np.ndarray) -> np.ndarray:
    """Build a 4x4 homogeneous transform from a position and an (x,y,z,w) quaternion."""
    t = np.eye(4, dtype=np.float64)
    t[:3, :3] = quat_xyzw_to_matrix(np.asarray(quat_xyzw, dtype=np.float64))
    t[:3, 3] = np.asarray(position, dtype=np.float64)
    return t


def transform_to_pos_quat(t: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Decompose a 4x4 transform into (position xyz, quaternion xyzw)."""
    t = np.asarray(t, dtype=np.float64)
    return t[:3, 3].copy(), matrix_to_quat_xyzw(t[:3, :3])


# --- Point maps (contract §3, normative) ------------------------------------------------

def godot_world_point_to_ros(p: np.ndarray) -> np.ndarray:
    """World position: Godot (Y-up) -> ROS map (Z-up).  (x,y,z) -> (x,-z,y)."""
    return R_WORLD_G2R @ np.asarray(p, dtype=np.float64)


def godot_cam_point_to_optical(p: np.ndarray) -> np.ndarray:
    """A point expressed in Godot camera axes -> the same point in ROS optical axes.

    (x,y,z) -> (x,-y,-z).
    """
    return R_CAM_G2R @ np.asarray(p, dtype=np.float64)


# --- Pose maps (orientation-aware; the part that bites in SLAM) --------------------------

def godot_world_pose_to_ros(position: np.ndarray, quat_xyzw: np.ndarray):
    """Convert a *world-attached* Godot pose (e.g. base_link, lander) to the ROS map frame.

    A pose is a change-of-basis from a body frame to the Godot world.  Re-expressing it in
    the ROS world re-bases BOTH endpoints by the world map (Map 1):
        R_ros = R_WORLD_G2R @ R_godot @ R_WORLD_G2R^T
        t_ros = R_WORLD_G2R @ t_godot
    The body frame itself is also a Godot-convention frame here (rover/lander axes follow the
    same Y-up rule as the world), so the same world map conjugates it -- exactly what keeps a
    forward-facing rover forward-facing in ROS.
    Returns (position_ros xyz, quaternion_ros xyzw).
    """
    r_g = quat_xyzw_to_matrix(np.asarray(quat_xyzw, dtype=np.float64))
    r_ros = R_WORLD_G2R @ r_g @ R_WORLD_G2R.T
    t_ros = R_WORLD_G2R @ np.asarray(position, dtype=np.float64)
    return t_ros, matrix_to_quat_xyzw(r_ros)


def godot_world_cam_pose_to_ros_optical(position: np.ndarray, quat_xyzw: np.ndarray):
    """Convert a Godot *camera* world-pose into a ROS map->optical pose.

    The camera origin is a world point (re-based by Map 1).  The camera *orientation* needs
    BOTH maps: re-base the world side by Map 1 and re-base the camera-axis side by Map 2:
        R_opt = R_WORLD_G2R @ R_godot @ R_CAM_G2R^T
        t_opt = R_WORLD_G2R @ t_godot
    i.e. the resulting rotation takes a vector in ROS optical axes to the ROS map frame.
    This is the orientation that, composed against the lander pose, yields the camera->tag
    truth in the optical frame (bag_writer.py).
    Returns (position_ros xyz, quaternion_optical xyzw).
    """
    r_g = quat_xyzw_to_matrix(np.asarray(quat_xyzw, dtype=np.float64))
    r_opt = R_WORLD_G2R @ r_g @ R_CAM_G2R.T
    t_opt = R_WORLD_G2R @ np.asarray(position, dtype=np.float64)
    return t_opt, matrix_to_quat_xyzw(r_opt)


def godot_cam_extrinsic_to_ros_optical(position: np.ndarray, quat_xyzw: np.ndarray):
    """Convert a camera extrinsic expressed *in base_link* (Godot axes) to ROS axes.

    base_link is itself converted by Map 1, and the camera-axis side by Map 2:
        R = R_WORLD_G2R @ R_godot @ R_CAM_G2R^T
        t = R_WORLD_G2R @ t_godot
    Used for the /tf_static base_link->*_optical edges.
    Returns (position xyz, quaternion xyzw).
    """
    r_g = quat_xyzw_to_matrix(np.asarray(quat_xyzw, dtype=np.float64))
    r = R_WORLD_G2R @ r_g @ R_CAM_G2R.T
    t = R_WORLD_G2R @ np.asarray(position, dtype=np.float64)
    return t, matrix_to_quat_xyzw(r)
