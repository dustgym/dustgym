#!/usr/bin/env python3
"""Back the rover's MAP pose out of a detected AprilTag face (demo_spiral_contract §3).

This is the inverse of `bag_writer._compute_truth`.  `_compute_truth` goes
*forward* -- given the truth poses it composes the camera(optical)->tag transform a
detector would observe:

    T_map_tag     = T_map_lander @ (T_lander_tag @ R_LANDER_TAG_relabel)
    T_map_optical = inv(...) ...      # built from the camera's world pose
    T_optical_tag = inv(T_map_optical) @ T_map_tag                # <- what the detector reports

DEMO-LOCALIZE goes the *other way*: the spiral demo has a FIXED lander (constant
`T_map_lander`, demo_spiral_contract §0/§2) and a KNOWN per-face `pose_in_lander`, and the
detector hands us `T_optical_tag` per visible face.  We recover the rover's `map->base_link`:

    T_map_baselink = T_map_lander @ lander_T_tag @ inv(T_optical_tag) @ inv(base_link_T_optical)

derived by chaining  T_map_optical = T_map_baselink @ base_link_T_optical  and
                     T_map_tag     = T_map_optical  @ T_optical_tag
                                   = T_map_lander   @ lander_T_tag
into     T_map_baselink @ base_link_T_optical @ T_optical_tag = T_map_lander @ lander_T_tag
and right-isolating `T_map_baselink`.  ``lander_T_tag`` is the tag's frame expressed in the
(ROS-converted) lander frame INCLUDING the §1.1 per-face relabel -- the detector reports the
tag axes in the `apriltag_ros` `pnp` convention, so the same `R_LANDER_TAG` / `R_face` that
`_compute_truth` right-multiplies onto `T_lander_tag` MUST be folded in here, or the recovered
orientation is off by that fixed ~120 deg axis-permutation.  Use :func:`lander_T_tag_from_pose`
(front face, M1-invariant) to build it from `sensors.json lander.apriltags[id].pose_in_lander`.

Detection itself is NOT rebuilt here: `fiducial_overlay.py` (`apriltag.detect` +
`cv2.solvePnP IPPE_SQUARE`) already produces the camera->tag pose per detected face; this
module is the pure-numpy transform math + the multi-face fuse that `fiducial_overlay` lacks
(it reports only camera->tag for id 0).  The container path calls the detector and feeds each
face's `T_optical_tag` into :func:`rover_pose_from_tag`; the math here is host-testable with
synthetic inputs (numpy only -- no cv2 / rclpy), cross-checked against `_compute_truth`'s
convention in the self-test below.

Quaternions are XYZW order throughout (matching `frames.py`, `sensors.json` and ROS).

CC0-1.0 (see ../../LICENSE).
"""

from __future__ import annotations

import numpy as np

import frames


# --- lander->tag builder (folds in the §1.1 detector-axis relabel) ----------------------

def lander_T_tag_from_pose(pose_in_lander, *, relabel=None):
    """Build the (ROS-frame) lander->tag transform from a `pose_in_lander` sub-pose.

    `pose_in_lander` is the contract-§2.2 dict ``{position_m, quaternion_xyzw}`` from
    `sensors.json lander.apriltags[id]` -- a body-frame transform inside the lander's own
    (Godot) axes.  It is re-expressed in the ROS lander frame by the SAME world map
    `bag_writer._compute_truth` uses (`frames.godot_world_pose_to_ros`), then the tag's OWN
    frame is relabelled into the detector's `pnp` convention by right-multiplying ``relabel``
    (default `frames.R_LANDER_TAG`, the front-face / M1-invariant relabel -- §1.1
    ``R_face(id=0) == R_LANDER_TAG``).  The TRANSLATION (tag centre == face origin) is
    untouched, exactly as in `_compute_truth`.

    Pass a per-face ``R_face`` (3x3, det=+1) for the non-front bundle faces (ids 1..3) when
    the demo localizes against a side face.  Returns a 4x4 transform.
    """
    if relabel is None:
        relabel = frames.R_LANDER_TAG
    tpos, tquat = frames.godot_world_pose_to_ros(
        pose_in_lander["position_m"], pose_in_lander["quaternion_xyzw"]
    )
    t_lander_tag = frames.make_transform(tpos, tquat)
    t_relabel = np.eye(4, dtype=np.float64)
    t_relabel[:3, :3] = np.asarray(relabel, dtype=np.float64)
    return t_lander_tag @ t_relabel


def base_link_T_optical_from_extrinsic(extrinsic_in_base_link):
    """Build the (ROS-frame) base_link->optical transform from a camera extrinsic.

    `extrinsic_in_base_link` is the contract-§2.2 dict for `sensors.json
    cameras[left].extrinsic_in_base_link` (Godot axes); convert it via
    `frames.godot_cam_extrinsic_to_ros_optical` -- the SAME helper `bag_writer.write_frame`
    uses for the /tf_static base_link->*_optical edge -- so the optical axis side matches the
    detector's optical frame.  Returns a 4x4 transform.
    """
    epos, equat = frames.godot_cam_extrinsic_to_ros_optical(
        extrinsic_in_base_link["position_m"],
        extrinsic_in_base_link["quaternion_xyzw"],
    )
    return frames.make_transform(epos, equat)


# --- the inverse: one face -> rover map pose --------------------------------------------

def rover_pose_from_tag(T_optical_tag, *, base_link_T_optical, T_map_lander, lander_T_tag):
    """Back the rover's map pose out of ONE detected face (demo_spiral_contract §3).

        T_map_baselink = T_map_lander @ lander_T_tag @ inv(T_optical_tag)
                                      @ inv(base_link_T_optical)

    All four arguments are 4x4 ROS-frame homogeneous transforms (column-vector convention,
    ``v_out = T @ v_in``):
      * ``T_optical_tag``     -- detector output: tag pose in the LEFT optical frame
                                 (what `_compute_truth` forward-computes / `fiducial_overlay`
                                 solvePnP reports).
      * ``base_link_T_optical`` -- camera extrinsic, ROS axes
                                 (see :func:`base_link_T_optical_from_extrinsic`).
      * ``T_map_lander``      -- the FIXED lander map pose (constant for the whole spiral, §0).
      * ``lander_T_tag``      -- tag pose in the ROS lander frame, INCLUDING the §1.1 detector
                                 relabel (see :func:`lander_T_tag_from_pose`).
    Returns ``(pos, quat_xyzw)`` of map->base_link (the rover's map pose).
    """
    T_optical_tag = np.asarray(T_optical_tag, dtype=np.float64)
    T_map_baselink = (
        np.asarray(T_map_lander, dtype=np.float64)
        @ np.asarray(lander_T_tag, dtype=np.float64)
        @ np.linalg.inv(T_optical_tag)
        @ np.linalg.inv(np.asarray(base_link_T_optical, dtype=np.float64))
    )
    return frames.transform_to_pos_quat(T_map_baselink)


# --- the fuse: combine the per-face rover estimates this frame --------------------------

def _quat_angle_deg(q_a, q_b):
    """Geodesic angle (deg) between two unit XYZW quaternions, sign-invariant (q == -q).

    Uses ``2*atan2(||q_err_vec||, |q_err_w|)`` rather than ``2*arccos(|dot|)``: arccos loses
    catastrophic precision near identity (its derivative blows up at 1.0), so it bottoms out at
    ~1e-3 deg for machine-equal quaternions; atan2 stays accurate all the way down to ~0.
    """
    qa = np.asarray(q_a, dtype=np.float64)
    qb = np.asarray(q_b, dtype=np.float64)
    # Relative quaternion q_err = qa^-1 * qb (qa unit => conjugate is the inverse), XYZW.
    ax, ay, az, aw = qa
    bx, by, bz, bw = qb
    ew = aw * bw + ax * bx + ay * by + az * bz
    ex = aw * bx - ax * bw - ay * bz + az * by
    ey = aw * by + ax * bz - ay * bw - az * bx
    ez = aw * bz - ax * by + ay * bx - az * bw
    vec = float(np.hypot(np.hypot(ex, ey), ez))
    return float(np.degrees(2.0 * np.arctan2(vec, abs(ew))))


def fuse_faces(per_face_poses):
    """Fuse the per-face rover-map-pose estimates of ONE frame; report agreement spread.

    ``per_face_poses`` is an iterable of ``(pos, quat_xyzw)`` -- one rover-pose estimate per
    detected face, each already backed out by :func:`rover_pose_from_tag`.  Each face is an
    INDEPENDENT estimate of the SAME rover pose, so they should agree; the disagreement is the
    headline diagnostic (a tight cluster => trustworthy localization, a wide one => a bad face
    detection / wrong-id association is dragging the fix).

    Fusion:
      * position -- arithmetic mean of the per-face translations.
      * orientation -- the per-face quaternions are sign-aligned to the first (q and -q are the
        same rotation) and averaged, then renormalized.  For the near-identical estimates this
        demo produces, the linear quaternion mean is within machine precision of the proper
        chordal/Karcher mean and avoids dragging in scipy; with a single face it is exact.

    Returns ``(pos, quat_xyzw, spread)`` where ``spread`` is a dict of agreement metrics:
      * ``n_faces``          -- number of estimates fused.
      * ``pos_spread_mm``    -- max pairwise translation disagreement, in millimetres
                                (0.0 for a single face).
      * ``rot_spread_deg``   -- max pairwise orientation angle, in degrees (0.0 for one face).
      * ``pos_std_mm``       -- per-axis RMS of the translations about the mean, in mm.
    """
    poses = list(per_face_poses)
    if not poses:
        raise ValueError("fuse_faces: no per-face poses to fuse")

    positions = np.array([np.asarray(p, dtype=np.float64) for p, _ in poses])
    quats = np.array([np.asarray(q, dtype=np.float64) for _, q in poses])
    quats = quats / np.linalg.norm(quats, axis=1, keepdims=True)

    # Sign-align every quaternion to the first so the linear mean does not cancel q vs -q.
    ref = quats[0]
    signs = np.sign(quats @ ref)
    signs[signs == 0.0] = 1.0
    quats_aligned = quats * signs[:, None]

    pos_mean = positions.mean(axis=0)
    quat_mean = quats_aligned.mean(axis=0)
    quat_mean = quat_mean / np.linalg.norm(quat_mean)
    quat_mean = frames.matrix_to_quat_xyzw(frames.quat_xyzw_to_matrix(quat_mean))  # canonicalize w>=0

    n = len(poses)
    pos_spread_mm = 0.0
    rot_spread_deg = 0.0
    for i in range(n):
        for j in range(i + 1, n):
            pos_spread_mm = max(
                pos_spread_mm,
                float(np.linalg.norm(positions[i] - positions[j])) * 1000.0,
            )
            rot_spread_deg = max(rot_spread_deg, _quat_angle_deg(quats[i], quats[j]))
    pos_std_mm = float(np.sqrt(((positions - pos_mean) ** 2).sum(axis=1).mean())) * 1000.0

    spread = {
        "n_faces": n,
        "pos_spread_mm": pos_spread_mm,
        "rot_spread_deg": rot_spread_deg,
        "pos_std_mm": pos_std_mm,
    }
    return pos_mean, quat_mean, spread


# --- self-test: closed round-trip vs the _compute_truth convention ----------------------

def _self_test() -> int:
    """Forward-compute the detector observation from a KNOWN rover pose, feed it back through
    :func:`rover_pose_from_tag`, and prove we recover the original pose to ~1e-9.

    The forward model REUSES `bag_writer._compute_truth`'s exact composition order so this
    cross-checks the inverse against the frozen truth convention (orientation included):
        T_map_optical = T_map_baselink @ base_link_T_optical
        T_map_tag     = T_map_lander   @ lander_T_tag
        T_optical_tag = inv(T_map_optical) @ T_map_tag        # what the detector reports
    All transforms are constructed DIRECTLY in the ROS frame (the inverse math is frame-
    agnostic), then `lander_T_tag` / `base_link_T_optical` are ALSO built from Godot-native
    `sensors.json`-shaped sub-poses via the frames.py helpers to exercise the real ingest path
    and confirm the §1.1 relabel round-trips.
    """
    rng = np.random.default_rng(20260531)
    ok = True

    def rand_quat():
        q = rng.standard_normal(4)
        return q / np.linalg.norm(q)

    # 1) A KNOWN rover map pose (the thing we must recover), a FIXED lander map pose,
    #    and a camera extrinsic + per-face pose_in_lander built from Godot-native sub-poses.
    rover_pos = np.array([3.4, -1.2, 0.15], dtype=np.float64)
    rover_quat = rand_quat()
    T_map_baselink_true = frames.make_transform(rover_pos, rover_quat)

    lander_pos = np.array([0.0, 0.0, 0.0], dtype=np.float64)   # fixed scene-centre lander
    lander_quat = rand_quat()
    T_map_lander = frames.make_transform(lander_pos, lander_quat)

    # base_link_T_optical from a sensors.json-shaped extrinsic_in_base_link (Godot axes ->
    # ROS via the same helper bag_writer uses); use the M1 fixture's left-cam extrinsic.
    extrinsic_in_base_link = {
        "position_m": [-0.05, 0.7, 0.0],
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
    }
    base_link_T_optical = base_link_T_optical_from_extrinsic(extrinsic_in_base_link)

    # lander_T_tag from the front-face identity pose_in_lander (M1-invariant) -> folds in
    # frames.R_LANDER_TAG, matching _compute_truth.
    pose_in_lander_front = {
        "position_m": [0.0, 0.0, 0.0],
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
    }
    lander_T_tag = lander_T_tag_from_pose(pose_in_lander_front)

    # 2) FORWARD: compute the camera(optical)->tag the detector would report, in the EXACT
    #    composition order of bag_writer._compute_truth.
    T_map_optical = T_map_baselink_true @ base_link_T_optical
    T_map_tag = T_map_lander @ lander_T_tag
    T_optical_tag = np.linalg.inv(T_map_optical) @ T_map_tag

    # 3) INVERSE: recover the rover pose from that single observation.
    pos_rec, quat_rec = rover_pose_from_tag(
        T_optical_tag,
        base_link_T_optical=base_link_T_optical,
        T_map_lander=T_map_lander,
        lander_T_tag=lander_T_tag,
    )
    pos_err = float(np.linalg.norm(pos_rec - rover_pos))
    rot_err = _quat_angle_deg(quat_rec, rover_quat)
    print(f"[round-trip] single-face recover: pos_err={pos_err:.3e} m  "
          f"rot_err={rot_err:.3e} deg")
    if pos_err > 1e-9 or rot_err > 1e-7:
        print("  FAIL: round-trip did not recover the known rover pose")
        ok = False

    # 3b) Guard: lander_T_tag_from_pose(front) must fold in EXACTLY frames.R_LANDER_TAG
    #     (§1.1 R_face(id=0) == R_LANDER_TAG -> M1 reading unchanged).
    relabel_back = lander_T_tag[:3, :3]  # identity pose_in_lander -> rotation IS the relabel
    # godot_world_pose_to_ros conjugates an identity quat to identity, so the residual rotation
    # is exactly R_LANDER_TAG; verify.
    if not np.allclose(relabel_back, frames.R_LANDER_TAG, atol=1e-12):
        print("  FAIL: front-face lander_T_tag does not reduce to R_LANDER_TAG")
        ok = False
    else:
        print("[relabel] front-face lander_T_tag == frames.R_LANDER_TAG (M1-invariant): OK")

    # 4) Multi-face fuse: simulate 3 faces all observing the SAME rover pose. Build 3 distinct
    #    per-face pose_in_lander sub-poses (different origins + a proper-rotation R_face each),
    #    forward-compute each detector observation, back each out, and fuse. They are EXACT
    #    independent estimates of one rover pose, so the spread must be ~0.
    # R_face permutations (proper rotations, det=+1) standing in for the side-face relabels.
    R_faces = [
        frames.R_LANDER_TAG,                       # front (id 0)
        frames.R_LANDER_TAG @ frames.R_LANDER_TAG,  # a different proper rotation (id 1)
        frames.R_LANDER_TAG.T,                      # its inverse (id 2)
    ]
    face_origins = [
        [0.0, 0.0, 0.0],
        [0.30, 0.0, 0.10],
        [-0.20, 0.05, -0.15],
    ]
    per_face = []
    for R_face, origin in zip(R_faces, face_origins):
        pil = {"position_m": origin, "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]}
        l_T_t = lander_T_tag_from_pose(pil, relabel=R_face)
        T_map_tag_f = T_map_lander @ l_T_t
        T_opt_tag_f = np.linalg.inv(T_map_optical) @ T_map_tag_f
        per_face.append(rover_pose_from_tag(
            T_opt_tag_f,
            base_link_T_optical=base_link_T_optical,
            T_map_lander=T_map_lander,
            lander_T_tag=l_T_t,
        ))
    fused_pos, fused_quat, spread = fuse_faces(per_face)
    fpos_err = float(np.linalg.norm(fused_pos - rover_pos))
    frot_err = _quat_angle_deg(fused_quat, rover_quat)
    print(f"[fuse] {spread['n_faces']} faces -> pos_err={fpos_err:.3e} m  "
          f"rot_err={frot_err:.3e} deg  "
          f"pos_spread_mm={spread['pos_spread_mm']:.3e}  "
          f"rot_spread_deg={spread['rot_spread_deg']:.3e}")
    if fpos_err > 1e-9 or frot_err > 1e-7:
        print("  FAIL: fused pose does not match the known rover pose")
        ok = False
    if spread["pos_spread_mm"] > 1e-6 or spread["rot_spread_deg"] > 1e-6:
        print("  FAIL: consistent faces should report ~0 spread")
        ok = False

    # 4b) Single-face fuse is exact and reports 0 spread.
    s_pos, s_quat, s_spread = fuse_faces([per_face[0]])
    if (s_spread["n_faces"] != 1 or s_spread["pos_spread_mm"] != 0.0
            or s_spread["rot_spread_deg"] != 0.0):
        print("  FAIL: single-face fuse must report n=1 and 0 spread")
        ok = False
    else:
        print("[fuse] single-face fuse: n=1, 0 spread: OK")

    # 5) Negative control: a DELIBERATELY WRONG face (wrong R_face relabel for its observation)
    #    must produce a DISAGREEING estimate -> non-trivial spread (proves the metric is real,
    #    not a constant 0).
    bad_pil = {"position_m": [0.30, 0.0, 0.10], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]}
    l_T_t_obs = lander_T_tag_from_pose(bad_pil, relabel=R_faces[1])       # what was observed
    T_opt_tag_bad = np.linalg.inv(T_map_optical) @ (T_map_lander @ l_T_t_obs)
    bad_est = rover_pose_from_tag(
        T_opt_tag_bad,
        base_link_T_optical=base_link_T_optical,
        T_map_lander=T_map_lander,
        lander_T_tag=lander_T_tag_from_pose(bad_pil, relabel=R_faces[0]),  # WRONG relabel used
    )
    _, _, bad_spread = fuse_faces([per_face[0], bad_est])
    print(f"[fuse] negative control (mismatched relabel): "
          f"rot_spread_deg={bad_spread['rot_spread_deg']:.3f}  "
          f"pos_spread_mm={bad_spread['pos_spread_mm']:.3f}")
    if bad_spread["rot_spread_deg"] < 1.0 and bad_spread["pos_spread_mm"] < 1.0:
        print("  FAIL: a wrong-relabel face should disagree (non-trivial spread)")
        ok = False

    print("ALL SELF-TESTS PASSED" if ok else "SELF-TEST FAILURES ABOVE")
    return 0 if ok else 1


if __name__ == "__main__":
    import sys

    sys.exit(_self_test())
