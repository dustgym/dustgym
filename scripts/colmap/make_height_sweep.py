#!/usr/bin/env python3
"""COLMAP camera-height sweep: how does ground-tier reconstruction degrade toward grazing?

Runs the COLMAP map-channel producer on Hapke corpora rendered at several camera heights (the arc held
fixed, only the height dropped from elevated toward the rover's grazing eye-level), scores each against
the conserved truth, and plots reconstruction quality (registered images, coverage, map RMSE) vs camera
height. The honest test of whether the ground COLMAP tier holds up on the rover's real grazing capture,
not just an idealized elevated arc.

Usage:
    <venv>/bin/python make_height_sweep.py --corpus-base <godot_sidecar/out> --scene <samples/scene> \
        --out <figures dir>
"""
from __future__ import annotations

import argparse
import json
import os
import tempfile

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from colmap_map_channel import colmap_observed_map  # noqa: E402

# (camera height [m], corpus subdir). corpus_hapke is the elevated 2.6 m arc from the A/B.
CORPORA = [(0.5, "corpus_h05"), (1.0, "corpus_h10"), (1.5, "corpus_h15"), (2.6, "corpus_hapke")]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--corpus-base", required=True, help="dir holding the corpus_* subdirs")
    ap.add_argument("--scene", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rows = []
    for h, sub in CORPORA:
        cdir = os.path.join(args.corpus_base, sub)
        if not os.path.isfile(os.path.join(cdir, "poses.json")):
            print(f"  height {h}: MISSING {cdir}")
            continue
        work = tempfile.mkdtemp(prefix=f"sweep_{sub}_")
        _, _, _, m = colmap_observed_map(cdir, args.scene, work)
        m["height_m"] = h
        rows.append(m)
        print(f"  height {h:.1f} m: {json.dumps(m)}")

    rows.sort(key=lambda r: r["height_m"])
    hs = [r["height_m"] for r in rows]
    reg = [r.get("registered", 0) for r in rows]
    cov = [r.get("coverage", 0.0) * 100 for r in rows]
    rmse = [r.get("map_rmse_m", float("nan")) for r in rows]

    fig, ax = plt.subplots(1, 3, figsize=(13, 4.0))
    ax[0].plot(hs, reg, "o-", color="C0")
    ax[0].set_title("Images registered (of 18)")
    ax[0].set_ylim(0, 19)
    ax[1].plot(hs, cov, "s-", color="C2")
    ax[1].set_title("Map coverage [%]")
    ax[2].plot(hs, rmse, "^-", color="C3")
    ax[2].set_title("Map RMSE vs truth [m]")
    for a in ax:
        a.set_xlabel("camera height [m]  (grazing <- -> elevated)")
        a.grid(alpha=0.3)
    fig.suptitle("Ground-tier COLMAP vs camera height: reconstruction degrades toward the rover's "
                 "grazing eye-level (real Godot render, crater_boulders)", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(os.path.join(args.out, "colmap_height_sweep.png"), dpi=130)
    plt.close(fig)
    with open(os.path.join(args.out, "colmap_height_sweep.json"), "w") as f:
        json.dump(rows, f, indent=2)
    print(f"wrote sweep figure + metrics to {args.out}")


if __name__ == "__main__":
    main()
