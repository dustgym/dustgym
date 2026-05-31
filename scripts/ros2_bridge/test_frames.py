"""The 3 REQUIRED REP-103 conversion unit tests (sensor_bridge_contract.md §3).

These are the guard against a silent sign-flip in the Godot->ROS seam (the classic cause of
plausible-but-wrong SLAM).  Pure-python + numpy, so they run on the host (`python3
test_frames.py`) AND in the container.  No pytest required, but pytest-discoverable too.

Required tests (contract §3):
  (a) a Godot world point at +X maps to ROS +X (forward) and Godot +Y (up) maps to ROS +Z;
  (b) a camera looking along Godot -Z yields a ROS optical +Z view direction;
  (c) round-trip of a known pose.

CC0-1.0 (see ../../LICENSE).
"""

from __future__ import annotations

import sys

import numpy as np

import frames


def _q_identity():
    return np.array([0.0, 0.0, 0.0, 1.0])


def test_a_world_axes():
    """(a) world +X -> ROS +X (forward); world +Y -> ROS +Z (up); world +Z -> ROS -Y."""
    np.testing.assert_allclose(
        frames.godot_world_point_to_ros([1.0, 0.0, 0.0]), [1.0, 0.0, 0.0], atol=1e-12
    )  # +X right (Godot) -> +X forward (ROS)
    np.testing.assert_allclose(
        frames.godot_world_point_to_ros([0.0, 1.0, 0.0]), [0.0, 0.0, 1.0], atol=1e-12
    )  # +Y up (Godot) -> +Z up (ROS)
    np.testing.assert_allclose(
        frames.godot_world_point_to_ros([0.0, 0.0, 1.0]), [0.0, -1.0, 0.0], atol=1e-12
    )  # +Z toward-viewer (Godot) -> -Y (ROS: +Y is left)
    # The map must be a proper rotation (det = +1), not a reflection.
    assert np.isclose(np.linalg.det(frames.R_WORLD_G2R), 1.0)


def test_b_camera_view_direction():
    """(b) a camera looking down Godot -Z has a ROS optical +Z view direction.

    A Godot camera with identity orientation looks along its own -Z.  In ROS optical axes
    the forward/view direction is +Z.  Applying Map 2 to the Godot view vector (0,0,-1) must
    therefore yield the optical forward (0,0,+1).
    """
    godot_view = np.array([0.0, 0.0, -1.0])  # Godot camera forward = -Z
    optical_view = frames.godot_cam_point_to_optical(godot_view)
    np.testing.assert_allclose(optical_view, [0.0, 0.0, 1.0], atol=1e-12)
    # Godot camera +X (right) stays optical +X (right); Godot +Y (up) -> optical -Y (down).
    np.testing.assert_allclose(
        frames.godot_cam_point_to_optical([1.0, 0.0, 0.0]), [1.0, 0.0, 0.0], atol=1e-12
    )
    np.testing.assert_allclose(
        frames.godot_cam_point_to_optical([0.0, 1.0, 0.0]), [0.0, -1.0, 0.0], atol=1e-12
    )
    assert np.isclose(np.linalg.det(frames.R_CAM_G2R), 1.0)


def test_c_pose_roundtrip():
    """(c) round-trip a known pose: matrix->quat->matrix and quat<->transform are lossless."""
    # A known non-trivial rotation: 30 deg about an arbitrary axis, plus a translation.
    axis = np.array([0.2, -0.7, 0.5])
    axis = axis / np.linalg.norm(axis)
    ang = np.deg2rad(30.0)
    q = np.array([*(axis * np.sin(ang / 2.0)), np.cos(ang / 2.0)])
    pos = np.array([1.5, -2.0, 0.25])

    # quat -> matrix -> quat (sign-canonicalized).
    m = frames.quat_xyzw_to_matrix(q)
    q2 = frames.matrix_to_quat_xyzw(m)
    if q2[3] * q[3] < 0:
        q2 = -q2
    np.testing.assert_allclose(q2, q, atol=1e-10)

    # transform compose/decompose round-trip.
    t = frames.make_transform(pos, q)
    pos2, q3 = frames.transform_to_pos_quat(t)
    np.testing.assert_allclose(pos2, pos, atol=1e-12)
    if q3[3] * q[3] < 0:
        q3 = -q3
    np.testing.assert_allclose(q3, q, atol=1e-10)

    # World pose conversion must be an isometry: applying it to two points preserves the
    # vector between them up to the world rotation (no scale/shear introduced).
    p_world_pos, p_world_quat = frames.godot_world_pose_to_ros(pos, q)
    # Re-derive: converting the rotation twice through the orthogonal world map is consistent.
    r_ros = frames.quat_xyzw_to_matrix(p_world_quat)
    assert np.isclose(np.linalg.det(r_ros), 1.0, atol=1e-9)
    np.testing.assert_allclose(r_ros @ r_ros.T, np.eye(3), atol=1e-9)


def test_d_camera_pose_optical_consistency():
    """Sanity: a Godot camera at world origin, identity orientation, converts to a ROS
    map->optical pose whose optical +Z (forward) points where the camera actually looks.

    Not one of the 3 required tests, but pins the bag_writer's camera->tag truth path so the
    optical forward axis is composed correctly.  A Godot identity-orientation camera looks
    along Godot world -Z; under Map 1 that direction is ROS map +Y (left), since REP-103's +X
    is forward and the Godot -Z axis is NOT the ROS forward.  The optical frame's local +Z
    (its forward) must therefore land on ROS map +Y -- exactly where the camera looks.  (The
    Godot scene author is responsible for orienting the camera so its -Z faces the lander;
    this test only asserts the conversion is faithful to wherever the camera points.)
    """
    pos = np.array([0.0, 0.0, 0.0])
    q = _q_identity()
    t_pos, t_quat = frames.godot_world_cam_pose_to_ros_optical(pos, q)
    r = frames.quat_xyzw_to_matrix(t_quat)
    optical_forward_local = np.array([0.0, 0.0, 1.0])  # optical +Z = forward
    forward_in_map = r @ optical_forward_local
    # Godot -Z (camera look dir) -> ROS map +Y under Map 1.
    expected = frames.godot_world_point_to_ros([0.0, 0.0, -1.0])
    np.testing.assert_allclose(forward_in_map, expected, atol=1e-12)
    np.testing.assert_allclose(forward_in_map, [0.0, 1.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(t_pos, [0.0, 0.0, 0.0], atol=1e-12)


def test_e_lander_tag_relabel():
    """The contract-§1 lander->apriltag tag-frame relabel (frames.R_LANDER_TAG).

    Guards the orientation fix: the tag frame the `pnp` detector reports (+X image-right, +Y
    image-up, +Z outward-normal-toward-camera) differs from the lander placement frame (+X =
    outward normal, +Y = up) by a FIXED rotation.
    (1) R_LANDER_TAG is a proper rotation (det=+1) -- a relabel, not a reflection.
    (2) detector tag +Z (col 2) is the outward normal == +lander +X.
    (3) End-to-end: a fronto-parallel camera looking straight at the tag must read the corrected
        camera->tag truth as the detector's fronto-parallel reading R_x(180 deg) (= the empirical
        q_xyzw=[0.998,..,-0.062] convention), and because the correction is a constant own-frame
        relabel it is pose-independent (an oblique view is exact too).
    """
    R = frames.R_LANDER_TAG
    assert np.isclose(np.linalg.det(R), 1.0, atol=1e-12)
    np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
    # detector tag +Z (3rd column) is the outward normal == lander +X (pnp estimator).
    np.testing.assert_allclose(R[:, 2], [1.0, 0.0, 0.0], atol=1e-12)

    # --- end-to-end: lander placed ahead of a camera that looks straight at it -------------
    def _lander_basis(fwd):
        fwd = np.asarray(fwd, float); fwd[1] = 0.0; fwd /= np.linalg.norm(fwd)
        nx = -fwd; ny = np.array([0.0, 1.0, 0.0])
        nz = np.cross(nx, ny); nz /= np.linalg.norm(nz)
        ny = np.cross(nz, nx); ny /= np.linalg.norm(ny)
        return np.column_stack([nx, ny, nz])

    def _ry(t):
        c, s = np.cos(t), np.sin(t)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

    def _corrected_truth_rot(fwd, cam_yaw_deg):
        r_map_l = frames.R_WORLD_G2R @ _lander_basis(fwd) @ frames.R_WORLD_G2R.T
        r_map_opt = frames.R_WORLD_G2R @ _ry(np.radians(cam_yaw_deg)) @ frames.R_CAM_G2R.T
        return r_map_opt.T @ (r_map_l @ R)  # camera_optical -> tag(detector frame)

    # fronto-parallel: rover faces Godot +X, camera yawed -90 deg so its -Z points to +X.
    # The detector reads a fronto-parallel tag as R_x(180 deg) under the `pnp` convention.
    rx180 = np.diag([1.0, -1.0, -1.0])
    r_front = _corrected_truth_rot([1.0, 0.0, 0.0], -90.0)
    delta = r_front.T @ rx180
    ang = np.degrees(np.arccos(max(-1.0, min(1.0, (np.trace(delta) - 1.0) / 2.0))))
    assert ang < 1e-6, f"fronto-parallel corrected truth not R_x(180): off by {ang} deg"


def _run_all():
    tests = [
        ("a_world_axes", test_a_world_axes),
        ("b_camera_view_direction", test_b_camera_view_direction),
        ("c_pose_roundtrip", test_c_pose_roundtrip),
        ("d_camera_pose_optical_consistency", test_d_camera_pose_optical_consistency),
        ("e_lander_tag_relabel", test_e_lander_tag_relabel),
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
    print(f"\nAll {len(tests)} REP-103 frame tests passed.")
    return 0


if __name__ == "__main__":
    sys.exit(_run_all())
