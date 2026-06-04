#!/usr/bin/env python3
"""Hapke vs Lambert COLMAP A/B: the BRDF-breaks-multi-view-photoconsistency result.

Runs the ground-tier COLMAP producer (`colmap_map_channel.colmap_observed_map`) on a Hapke corpus and
a Lambert corpus of the SAME scene, compares the reconstructions (registered images, 3-D points, mean
reprojection error, map coverage and RMSE vs the conserved truth), and writes a figure:
truth | Hapke observed | Lambert observed. The non-Lambertian regolith BRDF (Hapke, the physically
sourced default) is expected to reconstruct WORSE than the Lambert baseline, exactly as on real lunar
imagery -- which is the point: the simulator reproduces the real failure, and only it has the ground
truth to quantify it.

Usage:
    <venv>/bin/python make_colmap_ab.py --hapke <corpus_hapke> --lambert <corpus_lambert> \
        --scene <samples/crater_boulders> --out <figures dir>
"""
from __future__ import annotations

import argparse
import json
import os
import tempfile

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from colmap_map_channel import colmap_observed_map  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--hapke", required=True)
    ap.add_argument("--lambert", required=True)
    ap.add_argument("--scene", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    results = {}
    panels = {}
    truth_ref = None
    for label, corpus in (("hapke", args.hapke), ("lambert", args.lambert)):
        work = tempfile.mkdtemp(prefix=f"colmap_{label}_")
        obs, mask, truth, m = colmap_observed_map(corpus, args.scene, work)
        results[label] = m
        truth_ref = truth
        panels[label] = (obs, mask)
        print(f"{label:8s} {json.dumps(m)}")

    vmin, vmax = float(truth_ref.min()), float(truth_ref.max())
    fig, ax = plt.subplots(1, 3, figsize=(13, 4.4))
    im = ax[0].imshow(truth_ref, cmap="terrain", vmin=vmin, vmax=vmax)
    ax[0].set_title("Truth terrain (conserved authority)")
    fig.colorbar(im, ax=ax[0], fraction=0.046, label="elevation [m]")
    for k, label in enumerate(("hapke", "lambert")):
        obs, mask = panels[label]
        m = results[label]
        a = ax[k + 1]
        a.set_facecolor("0.92")
        if mask is not None and mask.any():
            H, W = mask.shape
            rows, cols = np.nonzero(mask)
            sck = a.scatter(cols, rows, c=obs[mask], cmap="terrain", vmin=vmin, vmax=vmax,
                            s=5, marker="s", linewidths=0)
            a.set_xlim(0, W)
            a.set_ylim(H, 0)
            a.set_aspect("equal")
            fig.colorbar(sck, ax=a, fraction=0.046, label="elevation [m]")
        a.set_title(f"COLMAP observed ({label})\n"
                    f"{m.get('registered',0)}/{m.get('n_images','?')} imgs, "
                    f"{m.get('n_points3D',0)} pts\n"
                    f"reproj {m.get('mean_reproj_px',float('nan')):.2f}px  "
                    f"cov {m.get('coverage',0)*100:.1f}%  RMSE {m.get('map_rmse_m',float('nan')):.3f} m")
    for a in ax:
        a.set_xticks([])
        a.set_yticks([])
    fig.suptitle("Ground-tier COLMAP map channel, Hapke vs Lambert BRDF "
                 "(real Godot multi-view render, crater_boulders)", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(os.path.join(args.out, "colmap_hapke_vs_lambert.png"), dpi=130)
    plt.close(fig)
    with open(os.path.join(args.out, "colmap_ab_metrics.json"), "w") as f:
        json.dump(results, f, indent=2)
    print(f"wrote figure + metrics to {args.out}")


if __name__ == "__main__":
    main()
