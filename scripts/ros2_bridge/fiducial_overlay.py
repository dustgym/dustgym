#!/usr/bin/env python3
"""Highlight the detected AprilTag on a front-left frame (sensor_bridge_contract §1).

Runs the apriltag detector on `front_left.png`, draws the detected tag quad + center + id +
decision-margin, and (if intrinsics + tag size are available) the solvePnP pose axes + range.
Output is a single annotated PNG — the "fiducial highlighting" visual.

Run inside the container (has cv2 + the apriltag binding):
    python3 fiducial_overlay.py --in /data/out/cam/flat_compact/000 --out bags/fid_flat.png
"""
import argparse
import json
import os

import cv2
import numpy as np
from apriltag import apriltag


def _corners(det):
    # The apriltag binding exposes the 4 corners as 'lb-rb-rt-lt'; fall back to 'corners'.
    for k in ("lb-rb-rt-lt", "corners"):
        if k in det:
            return np.array(det[k], dtype=np.float32)
    raise KeyError("no corner field in detection: %s" % list(det.keys()))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--in", dest="in_dir", required=True)
    ap.add_argument("--out", dest="out_path", required=True)
    ap.add_argument("--family", default="tag36h11")
    args = ap.parse_args()

    sensors = json.load(open(os.path.join(args.in_dir, "sensors.json")))
    left_cam = next(c for c in sensors["cameras"] if c["name"] == sensors["stereo"]["left"])
    intr = left_cam["intrinsics"]
    fx, fy, cx, cy = (float(intr[k]) for k in ("fx", "fy", "cx", "cy"))
    tag_size = float(sensors["lander"]["apriltag"]["size_m"])

    img = cv2.imread(os.path.join(args.in_dir, "front_left.png"), cv2.IMREAD_COLOR)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    dets = apriltag(args.family).detect(gray)
    print(f"fiducial_overlay: detected {len(dets)} tag(s): {[d.get('id') for d in dets]}")

    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    for d in dets:
        c = _corners(d)
        cv2.polylines(img, [c.astype(np.int32)], True, (0, 255, 0), 3)
        ctr = tuple(np.array(d["center"], dtype=int))
        cv2.circle(img, ctr, 5, (0, 0, 255), -1)
        label = f"{args.family}:{d.get('id')}"
        cv2.putText(img, label, (ctr[0] + 8, ctr[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        # Pose axes + range via solvePnP (object corners in the tag plane, order lb-rb-rt-lt).
        h = tag_size / 2.0
        obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
        ok, rvec, tvec = cv2.solvePnP(obj, c.astype(np.float64), K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if ok:
            axes = np.float64([[h, 0, 0], [0, h, 0], [0, 0, h], [0, 0, 0]])
            pts, _ = cv2.projectPoints(axes, rvec, tvec, K, None)
            pts = pts.reshape(-1, 2).astype(int)
            o = tuple(pts[3])
            cv2.line(img, o, tuple(pts[0]), (0, 0, 255), 3)   # X red
            cv2.line(img, o, tuple(pts[1]), (0, 255, 0), 3)   # Y green
            cv2.line(img, o, tuple(pts[2]), (255, 0, 0), 3)   # Z blue
            rng = float(np.linalg.norm(tvec))
            cv2.putText(img, f"range {rng:.2f} m", (ctr[0] + 8, ctr[1] + 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    os.makedirs(os.path.dirname(os.path.abspath(args.out_path)), exist_ok=True)
    cv2.imwrite(args.out_path, img)
    print(f"fiducial_overlay: wrote {args.out_path}")


if __name__ == "__main__":
    main()
