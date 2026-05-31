#!/usr/bin/env python3
"""Diagnose the rover-map back-out frame flip: compare solvePnP's T_optical_tag to the
truth-forward-computed T_optical_tag (which round-trips by construction). Their relative
transform is the exact convention bug. Run in the container on one frame dir."""
import json
import os
import sys

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "ros2_bridge"))
import frames
import rover_localize as rl
from apriltag import apriltag


def _corners(det):
    for k in ("lb-rb-rt-lt", "corners"):
        if k in det:
            return np.array(det[k], dtype=np.float64)
    raise KeyError(list(det.keys()))


def main():
    d = sys.argv[1]
    sensors = json.load(open(os.path.join(d, "sensors.json")))
    lander, rover = sensors["lander"], sensors["rover"]
    T_map_lander = frames.make_transform(
        *frames.godot_world_pose_to_ros(lander["position_m"], lander["quaternion_xyzw"]))
    truth_pos, truth_quat = frames.godot_world_pose_to_ros(rover["position_m"], rover["quaternion_xyzw"])
    T_map_baselink_truth = frames.make_transform(truth_pos, truth_quat)
    left = next(c for c in sensors["cameras"] if c["name"] == sensors["stereo"]["left"])
    intr = left["intrinsics"]
    fx, fy, cx, cy = (float(intr[k]) for k in ("fx", "fy", "cx", "cy"))
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    base_link_T_optical = rl.base_link_T_optical_from_extrinsic(left["extrinsic_in_base_link"])
    T_map_optical_truth = T_map_baselink_truth @ base_link_T_optical
    by_id = {int(a["id"]): a for a in lander["apriltags"]}

    gray = cv2.cvtColor(cv2.imread(os.path.join(d, "front_left.png"), cv2.IMREAD_COLOR), cv2.COLOR_BGR2GRAY)
    dets = apriltag("tag36h11").detect(gray)
    print(f"frame {d}: truth rover pos={np.round(truth_pos,3)} quat={np.round(truth_quat,3)}  dets={[int(x.get('id')) for x in dets]}")
    np.set_printoptions(precision=4, suppress=True)
    for det in dets:
        tid = int(det.get("id", -1))
        a = by_id.get(tid)
        if a is None:
            continue
        h = float(a["size_m"]) / 2.0
        obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
        ok, rvec, tvec = cv2.solvePnP(obj, _corners(det), K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        Rm, _ = cv2.Rodrigues(rvec)
        Ts = np.eye(4); Ts[:3, :3] = Rm; Ts[:3, 3] = tvec.reshape(3)   # solvePnP tag->optical
        lander_T_tag = rl.lander_T_tag_from_pose(a["pose_in_lander"])
        T_map_tag = T_map_lander @ lander_T_tag
        Tt = np.linalg.inv(T_map_optical_truth) @ T_map_tag            # truth-forward tag->optical
        rel = np.linalg.inv(Tt) @ Ts                                   # the convention bug
        # recovered poses
        ps, qs = rl.rover_pose_from_tag(Ts, base_link_T_optical=base_link_T_optical, T_map_lander=T_map_lander, lander_T_tag=lander_T_tag)
        pt, qt = rl.rover_pose_from_tag(Tt, base_link_T_optical=base_link_T_optical, T_map_lander=T_map_lander, lander_T_tag=lander_T_tag)
        es = float(np.linalg.norm(ps - np.array(truth_pos)) * 1000)
        et = float(np.linalg.norm(pt - np.array(truth_pos)) * 1000)
        print(f"\n--- id{tid} ---")
        print(f"  Ts (solvePnP) t={np.round(Ts[:3,3],3)}")
        print(f"  Tt (truth)    t={np.round(Tt[:3,3],3)}")
        print(f"  rel = inv(Tt)@Ts  R=\n{rel[:3,:3]}\n  rel t={np.round(rel[:3,3],4)}")
        print(f"  recovered-from-solvePnP trans_err={es:.1f}mm | recovered-from-TRUTH trans_err={et:.3f}mm (must be ~0)")


if __name__ == "__main__":
    main()
