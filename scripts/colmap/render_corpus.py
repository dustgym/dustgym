#!/usr/bin/env python3
"""Render a static-scene multi-view image corpus for offline COLMAP photogrammetry.

A controlled camera arc orbiting the scene at a fixed look-at, recording the KNOWN pose of every
frame (so the COLMAP reconstruction, which is up to a similarity transform, can be Umeyama-aligned
back to the world frame for scoring against the conserved truth). Renders the STATIC terrain+clasts
only -- no rover, no lander -- so nothing moves between frames (COLMAP needs a rigid scene).

This is the GROUND tier of the two-tier perception story: offline COLMAP over the rover's image
corpus, the way GMRO already builds maps. Render once per BRDF (hapke|lambert) for the A/B that
shows the non-Lambertian regolith BRDF degrading multi-view photoconsistency.

Usage (from anywhere; render.sh self-locates):
    python3 render_corpus.py --brdf hapke   --out-name corpus_hapke
    python3 render_corpus.py --brdf lambert --out-name corpus_lambert
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess

_HERE = os.path.dirname(os.path.abspath(__file__))
_SIDE = os.path.normpath(os.path.join(_HERE, "..", "..", "godot_sidecar"))
_RENDER = os.path.join(_SIDE, "render.sh")


def arc_poses(cx, cz, radius, height, az0_deg, az1_deg, n):
    """n camera poses on an arc at `height`, `radius` from (cx,cz), all looking at (cx,0,cz)."""
    poses = []
    for i in range(n):
        a = math.radians(az0_deg + (az1_deg - az0_deg) * i / (n - 1))
        px = cx + radius * math.cos(a)
        pz = cz + radius * math.sin(a)
        poses.append({"pos": [px, height, pz], "target": [cx, 0.0, cz]})
    return poses


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--scene", default="../samples/crater_boulders", help="scene dir (relative to godot_sidecar)")
    ap.add_argument("--brdf", choices=["hapke", "lambert"], default="hapke")
    ap.add_argument("--out-name", required=True, help="out/<name>/ subdir for the corpus")
    ap.add_argument("--n", type=int, default=18)
    ap.add_argument("--radius", type=float, default=5.0)
    ap.add_argument("--height", type=float, default=2.6)
    ap.add_argument("--size", default="1024x768")
    args = ap.parse_args()

    cx, cz = 2.56, 2.56               # crater_boulders patch center (5.12 m, cell 0.02)
    poses = arc_poses(cx, cz, args.radius, args.height, 20.0, 160.0, args.n)
    out_rel = args.out_name           # bare -> lands in godot_sidecar/out/<name>/
    out_abs = os.path.join(_SIDE, "out", args.out_name)
    os.makedirs(out_abs, exist_ok=True)

    w, h = (int(x) for x in args.size.split("x"))
    fov_v = 55.0                      # sidecar single-frame Camera3D default (vertical, keep-height)
    fy = (h / 2) / math.tan(math.radians(fov_v / 2))
    intr = {"model": "PINHOLE", "w": w, "h": h, "fx": fy, "fy": fy, "cx": w / 2, "cz": h / 2}

    manifest = {"scene": args.scene, "brdf": args.brdf, "intrinsics": intr, "frames": []}
    for i, p in enumerate(poses):
        name = f"f{i:02d}.png"
        pose_arg = ",".join(f"{v:.5f}" for v in (p["pos"] + p["target"]))
        cmd = [_RENDER, os.path.join(_SIDE, "sidecar.tscn"), "--",
               "--scene", args.scene, "--layers", "terrain,clasts",
               "--brdf", args.brdf, "--pose", pose_arg, "--size", args.size,
               "--out", f"{out_rel}/{name}"]
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=200)
        ok = os.path.isfile(os.path.join(out_abs, name))
        manifest["frames"].append({"name": name, "pos": p["pos"], "target": p["target"], "ok": ok})
        print(f"  {name}  az-step {i+1}/{args.n}  {'OK' if ok else 'FAIL: ' + r.stderr[-120:]}")

    with open(os.path.join(out_abs, "poses.json"), "w") as f:
        json.dump(manifest, f, indent=2)
    n_ok = sum(1 for fr in manifest["frames"] if fr["ok"])
    print(f"corpus {args.out_name}: {n_ok}/{args.n} frames -> {out_abs}")


if __name__ == "__main__":
    main()
