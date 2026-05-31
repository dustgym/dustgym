#!/usr/bin/env python3
"""Stereo depth map from a front-stereo `out/cam/<scene>/<NNN>/` egress (sensor_bridge_contract §2).

Reads `front_left.png` + `front_right.png` + `sensors.json` (for fx + the stereo baseline), runs
OpenCV StereoSGBM to get disparity, converts to metric depth `Z = fx * baseline / disparity`, and
writes a colorized depth PNG (near = warm, far = cool; invalid = black). Rectified-pinhole input
(M1 distortion OFF), so no rectification step is needed.

Run inside the container (has cv2):
    python3 depth_map.py --in /data/out/cam/boulder_field/000 --out bags/depth_boulders.png
"""
import argparse
import json
import os

import cv2
import numpy as np


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--in", dest="in_dir", required=True, help="out/cam/<scene>/<NNN>/ dir")
    ap.add_argument("--out", dest="out_path", required=True, help="output colorized depth PNG")
    ap.add_argument("--max-depth", type=float, default=6.0, help="depth (m) mapped to the far end of the ramp")
    args = ap.parse_args()

    sensors = json.load(open(os.path.join(args.in_dir, "sensors.json")))
    left_cam = next(c for c in sensors["cameras"] if c["name"] == sensors["stereo"]["left"])
    fx = float(left_cam["intrinsics"]["fx"])
    baseline = float(sensors["stereo"]["baseline_m"])

    left = cv2.imread(os.path.join(args.in_dir, "front_left.png"), cv2.IMREAD_GRAYSCALE)
    right = cv2.imread(os.path.join(args.in_dir, "front_right.png"), cv2.IMREAD_GRAYSCALE)
    if left is None or right is None:
        raise SystemExit("could not read front_left.png / front_right.png")

    # SGBM tuned for ~1-6 m at 0.1 m baseline, 1280-wide (disparity ~40 px @ 2.3 m, ~90 px @ 1 m).
    # Tuned for low-contrast, low-texture lunar regolith: small block (fine grit is ~6 px at a
    # few m, so big windows wash it out), permissive uniqueness (the speckle contrast is modest),
    # generous speckle filter to drop isolated mismatches. Passive stereo on uniform regolith is
    # inherently sparse; these recover what's recoverable without inventing matches.
    num_disp = 160  # multiple of 16
    block = 5
    sgbm = cv2.StereoSGBM_create(
        minDisparity=0, numDisparities=num_disp, blockSize=block,
        P1=8 * block * block, P2=32 * block * block,
        disp12MaxDiff=2, uniquenessRatio=5, speckleWindowSize=80, speckleRange=4,
        preFilterCap=31, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )
    disp = sgbm.compute(left, right).astype(np.float32) / 16.0  # SGBM returns fixed-point ×16

    valid = disp > 0.5
    depth = np.zeros_like(disp)
    depth[valid] = fx * baseline / disp[valid]

    # Colorize: 0..max_depth -> 255..0 (near warm) via TURBO; invalid -> black.
    norm = np.clip(depth / args.max_depth, 0.0, 1.0)
    vis = cv2.applyColorMap(((1.0 - norm) * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    vis[~valid] = (0, 0, 0)

    os.makedirs(os.path.dirname(os.path.abspath(args.out_path)), exist_ok=True)
    cv2.imwrite(args.out_path, vis)
    dv = depth[valid]
    print(f"depth_map: fx={fx:.1f} baseline={baseline:.3f}  valid={valid.mean()*100:.1f}%  "
          f"depth[min/med/max]={dv.min():.2f}/{np.median(dv):.2f}/{dv.max():.2f} m -> {args.out_path}")


if __name__ == "__main__":
    main()
