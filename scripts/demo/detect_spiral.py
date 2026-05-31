#!/usr/bin/env python3
"""Container-side detector for the spiral demo: per-frame AprilTag detect -> rover map pose.

Runs INSIDE the ROS2 container (foss_ipex/ros2_bridge:jazzy; cv2 + the `apriltag` binding).
For each out/cam/<run>/<NNN>/ frame it:
  1. detects all tag36h11 faces in front_left.png (apriltag.detect),
  2. solvePnP (IPPE_SQUARE) each face -> camera(optical)->tag pose,
  3. backs out the rover map pose per face via rover_localize.rover_pose_from_tag
     (FIXED lander T_map_lander + the face's known pose_in_lander), and fuses the faces,
  4. writes <NNN>/detect.json = {detected_faces, n_faces, rover_est_map, spread, range_m,
     per_face_err_vs_truth, occluded} -- the exact shape scripts/demo/demo_spiral.py ingests.

The detected rover pose is in the ROS map frame; demo_spiral.py (host) compares it to the
sub-cell truth (sensors.json rover{} via frames.godot_world_pose_to_ros) -> trans_mm/rot_deg.
per_face_err_vs_truth is a DIAGNOSTIC (uses truth only to report, never to filter) so we can
see whether the side-face relabel is correct (low err on all ids) or only id0 is trustworthy.

Run:  docker run --rm -v <repo>:/data foss_ipex/ros2_bridge:jazzy \
          bash -lc 'cd /data && python3 scripts/demo/detect_spiral.py --seq-dir godot_sidecar/out/cam/haworth_spiral_lit'
"""
import argparse
import json
import os
import sys

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "ros2_bridge"))
import frames  # noqa: E402
import rover_localize as rl  # noqa: E402
from apriltag import apriltag  # noqa: E402


def _corners(det):
    for k in ("lb-rb-rt-lt", "corners"):
        if k in det:
            return np.array(det[k], dtype=np.float64)
    raise KeyError("no corner field in detection: %s" % list(det.keys()))


# cv2.solvePnP's tag frame (obj corners lb-rb-rt-lt, +Y up / +Z toward camera) is rotated
# 180deg about X from the apriltag_ros `pnp` convention that frames.R_LANDER_TAG / the
# rover_localize chain expect (+Y down / +Z away). VERIFIED via _diag_pose.py: rel = inv(
# truth-forward T_optical_tag) @ solvePnP T_optical_tag == diag(1,-1,-1) to detection noise.
# Right-multiply (tag-frame side) to re-express solvePnP's pose in the detector convention.
_R_X180 = np.diag([1.0, -1.0, -1.0, 1.0])


def _T_from_rvec_tvec(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
    return T @ _R_X180


def _frame_dirs(seq_dir, cam="front_left"):
    out = []
    for name in sorted(os.listdir(seq_dir)):
        d = os.path.join(seq_dir, name)
        if os.path.isdir(d) and os.path.exists(os.path.join(d, "sensors.json")) \
                and os.path.exists(os.path.join(d, f"{cam}.png")):
            out.append((name, d))
    return out


def detect_frame(d, family="tag36h11", cam="front_left"):
    sensors = json.load(open(os.path.join(d, "sensors.json")))
    lander = sensors["lander"]
    rover = sensors["rover"]
    # FIXED lander map pose (ROS) + truth rover map pose (ROS).
    T_map_lander = frames.make_transform(
        *frames.godot_world_pose_to_ros(lander["position_m"], lander["quaternion_xyzw"]))
    truth_pos, truth_quat = frames.godot_world_pose_to_ros(
        rover["position_m"], rover["quaternion_xyzw"])
    range_m = float(np.linalg.norm(np.array(rover["position_m"]) - np.array(lander["position_m"])))

    # Fiducial camera (default front_left stereo; --cam left_mono for the travel-tangent runs where
    # the SIDE mono acquires the lander). Mono solvePnP works per-camera off its own intrinsics +
    # extrinsic_in_base_link, so any named camera resolves the same rover_pose_from_tag chain.
    cam_rec = next(c for c in sensors["cameras"] if c["name"] == cam)
    intr = cam_rec["intrinsics"]
    fx, fy, cx, cy = (float(intr[k]) for k in ("fx", "fy", "cx", "cy"))
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    base_link_T_optical = rl.base_link_T_optical_from_extrinsic(cam_rec["extrinsic_in_base_link"])
    by_id = {int(a["id"]): a for a in lander.get("apriltags", [])}

    gray = cv2.cvtColor(cv2.imread(os.path.join(d, f"{cam}.png"), cv2.IMREAD_COLOR),
                        cv2.COLOR_BGR2GRAY)
    dets = apriltag(family).detect(gray)

    per_face, faces, per_face_err = [], [], []
    for det in dets:
        tid = int(det.get("id", -1))
        a = by_id.get(tid)
        if a is None:
            continue
        h = float(a["size_m"]) / 2.0
        obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
        ok, rvec, tvec = cv2.solvePnP(obj, _corners(det), K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            continue
        T_opt_tag = _T_from_rvec_tvec(rvec, tvec)
        lander_T_tag = rl.lander_T_tag_from_pose(a["pose_in_lander"])
        pos, quat = rl.rover_pose_from_tag(
            T_opt_tag, base_link_T_optical=base_link_T_optical,
            T_map_lander=T_map_lander, lander_T_tag=lander_T_tag)
        per_face.append((pos, quat))
        faces.append(tid)
        per_face_err.append({
            "id": tid,
            "trans_mm": float(np.linalg.norm(pos - np.array(truth_pos)) * 1000.0),
            "rot_deg": rl._quat_angle_deg(quat, truth_quat),
        })

    rec = {"detected_faces": faces, "n_faces": len(faces), "range_m": round(range_m, 4),
           "occluded": False, "per_face_err_vs_truth": per_face_err}
    if per_face:
        fpos, fquat, spread = rl.fuse_faces(per_face)
        rec["rover_est_map"] = {"position_m": [float(v) for v in fpos],
                                "quaternion_xyzw": [float(v) for v in fquat]}
        rec["spread"] = spread
    else:
        rec["rover_est_map"] = None
        rec["spread"] = {}
    return rec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seq-dir", required=True)
    ap.add_argument("--family", default="tag36h11")
    ap.add_argument("--cam", default="front_left",
                    help="fiducial camera name (front_left | left_mono | right_mono | ...)")
    a = ap.parse_args()
    frame_dirs = _frame_dirs(a.seq_dir, a.cam)
    n_det = 0
    for name, d in frame_dirs:
        rec = detect_frame(d, a.family, a.cam)
        json.dump(rec, open(os.path.join(d, "detect.json"), "w"), indent=2)
        det = rec["rover_est_map"] is not None
        n_det += int(det)
        e = ""
        if det and rec["per_face_err_vs_truth"]:
            best = min(rec["per_face_err_vs_truth"], key=lambda x: x["trans_mm"])
            e = f" est trans_err={best['trans_mm']:.1f}mm rot_err={best['rot_deg']:.2f}deg (id{best['id']})"
        print(f"{name}: range={rec['range_m']:.2f}m faces={rec['detected_faces']}{e}")
    print(f"detect_spiral: {n_det}/{len(frame_dirs)} frames localized in {a.seq_dir}")


if __name__ == "__main__":
    main()
