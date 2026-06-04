#!/usr/bin/env python3
"""Generate the section-10 MAP-CHANNEL validation figures from a REAL Godot front-stereo drive.

Three figures, all from real render output + the conserved-authority truth (no synthetic data):
  1. observed_vs_truth.png   -- truth heightfield | accumulated observed map | per-cell error.
  2. levers.png              -- the two RMSE levers measured on the drive (depth cap; #stations).
  3. vslam_features.png      -- ORB keypoints on a real rover-cam frame (visual-SLAM feature density).

Usage:
    python3 make_map_channel_figures.py --drive <dir of station egresses> --scene <samples/scene> \
        --frame <a front_left.png> --out <figures dir>
"""
from __future__ import annotations

import argparse
import os

import cv2
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

import obs_map_producer as omp  # noqa: E402
from score_map import score_map  # noqa: E402


def _stations(drive):
    return [os.path.join(drive, d) for d in sorted(os.listdir(drive))
            if os.path.isfile(os.path.join(drive, d, "sensors.json"))]


def fig_observed_vs_truth(stations, scene, out):
    grid = omp.grid_from_metadata(os.path.join(scene, "metadata.json"))
    truth = omp.load_truth_heightmap(scene, grid)
    obs, mask = omp.produce_observed_map_multi(stations, grid)
    sc = score_map(obs, truth, tol_m=0.10, valid_mask=mask)
    err = np.where(mask, obs - truth, np.nan)
    obs_show = np.where(mask, obs, np.nan)
    vmin, vmax = float(truth.min()), float(truth.max())

    fig, ax = plt.subplots(1, 3, figsize=(13, 4.4))
    im0 = ax[0].imshow(truth, cmap="terrain", vmin=vmin, vmax=vmax)
    ax[0].set_title("Truth terrain (conserved authority)\nheightfield at time t")
    fig.colorbar(im0, ax=ax[0], fraction=0.046, label="elevation [m]")
    im1 = ax[1].imshow(obs_show, cmap="terrain", vmin=vmin, vmax=vmax)
    ax[1].set_title(f"Observed map (rover stereo, {len(stations)}-station drive)\n"
                    f"coverage {mask.mean()*100:.1f}%  (grey = unobserved)")
    ax[1].set_facecolor("0.85")
    fig.colorbar(im1, ax=ax[1], fraction=0.046, label="elevation [m]")
    im2 = ax[2].imshow(err, cmap="coolwarm", vmin=-0.4, vmax=0.4)
    ax[2].set_title(f"Observed - truth error\nRMSE {sc['map_rmse_m']:.3f} m   "
                    f"1σ {np.nanstd(err):.3f} m")
    fig.colorbar(im2, ax=ax[2], fraction=0.046, label="error [m]")
    for a in ax:
        a.set_xticks([])
        a.set_yticks([])
    fig.suptitle("Section 10 map channel: onboard rover-stereo observed map vs conserved truth "
                 "(real Godot render, crater_boulders)", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(os.path.join(out, "observed_vs_truth.png"), dpi=130)
    plt.close(fig)
    return sc, mask.mean()


def fig_levers(stations, scene, out):
    grid = omp.grid_from_metadata(os.path.join(scene, "metadata.json"))
    truth = omp.load_truth_heightmap(scene, grid)

    caps = [1.0, 1.5, 2.0, 3.0, 4.0]
    cap_rmse, cap_cov = [], []
    for cap in caps:
        pts = [omp.collect_world_points(d, max_depth_m=cap) for d in stations]
        pts = [p for p in pts if p.size]
        obs, mask = omp.grid_to_heightfield(np.concatenate(pts), grid)
        e = obs[mask] - truth[mask]
        cap_rmse.append(float(np.sqrt((e * e).mean())))
        cap_cov.append(float(mask.mean() * 100))

    ns = list(range(1, len(stations) + 1))
    n_cov = []
    for n in ns:
        _, m = omp.produce_observed_map_multi(stations[:n], grid)
        n_cov.append(float(m.mean() * 100))

    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
    a0 = ax[0]
    a0b = a0.twinx()
    a0.plot(caps, cap_rmse, "o-", color="C3", label="RMSE")
    a0b.plot(caps, cap_cov, "s--", color="C0", label="coverage")
    a0.set_xlabel("near-field depth cap [m]")
    a0.set_ylabel("map RMSE [m]", color="C3")
    a0b.set_ylabel("coverage [%]", color="C0")
    a0.set_title("Lever 1: depth cap trades accuracy vs coverage\n"
                 "(stereo error grows as Z²/f·baseline)")
    a0.grid(alpha=0.3)
    ax[1].plot(ns, n_cov, "s-", color="C0")
    ax[1].set_xlabel("stations driven (accumulated)")
    ax[1].set_ylabel("coverage [%]", color="C0")
    ax[1].set_title("Lever 2: coverage grows as the rover drives\n"
                    "(map-by-driving, the LAC approach)")
    ax[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "levers.png"), dpi=130)
    plt.close(fig)


def fig_vslam_features(frame_path, out):
    img = cv2.imread(frame_path, cv2.IMREAD_GRAYSCALE)
    orb = cv2.ORB_create(nfeatures=1500, fastThreshold=12)
    kp = orb.detect(img, None)
    lit = img > 8
    in_shadow = sum(1 for k in kp if not lit[int(k.pt[1]), int(k.pt[0])])
    vis = cv2.drawKeypoints(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), kp, None,
                            color=(0, 255, 0), flags=0)
    fig, ax = plt.subplots(figsize=(7.2, 5.4))
    ax.imshow(cv2.cvtColor(vis, cv2.COLOR_BGR2RGB))
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title(f"Visual-SLAM feature density on the real rover camera\n"
                 f"{len(kp)} ORB features  ({len(kp)/max(lit.sum(),1)*1e6:.0f}/lit-Mpx)  "
                 f"{in_shadow} in shadow (illumination-locked)")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "vslam_features.png"), dpi=130)
    plt.close(fig)


def fig_uncertainty(stations, scene, out):
    """The world model's Uncertainty layer: observed height | per-cell height sigma | dig-ready gate."""
    grid = omp.grid_from_metadata(os.path.join(scene, "metadata.json"))
    obs, sigma, count, mask = omp.produce_uncertainty_map(stations, grid)
    ready = omp.dig_ready_mask(sigma, mask, tol_m=0.10)
    obs_show = np.where(mask, obs, np.nan)
    sig_show = np.where(mask, np.minimum(sigma, omp.PRIOR_SIGMA_M), np.nan)

    fig, ax = plt.subplots(1, 3, figsize=(13, 4.4))
    im0 = ax[0].imshow(obs_show, cmap="terrain")
    ax[0].set_title("Observed height (rover stereo)")
    ax[0].set_facecolor("0.85")
    fig.colorbar(im0, ax=ax[0], fraction=0.046, label="elevation [m]")
    im1 = ax[1].imshow(sig_show, cmap="magma_r", vmin=0.0, vmax=omp.PRIOR_SIGMA_M)
    ax[1].set_title("Per-cell height uncertainty (1σ)\nstd-error of the mean; single-view = prior 0.30 m")
    ax[1].set_facecolor("0.85")
    fig.colorbar(im1, ax=ax[1], fraction=0.046, label="sigma [m]")
    # green = dig-ready (observed + sigma < 0.10 m), red = observed but uncertain, grey = unobserved
    gate = np.full(mask.shape + (3,), 0.85)
    gate[mask & ~ready] = (0.85, 0.3, 0.3)
    gate[ready] = (0.2, 0.7, 0.3)
    ax[2].imshow(gate)
    ax[2].set_title(f"Dig-ready gate (σ < 0.10 m)\n"
                    f"green {int(ready.sum())} ready · red {int((mask & ~ready).sum())} observe-more")
    for a in ax:
        a.set_xticks([])
        a.set_yticks([])
    fig.suptitle("World model Uncertainty layer: gate digging on per-cell confidence "
                 "(real rover-stereo render, crater_boulders)", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(os.path.join(out, "uncertainty_layer.png"), dpi=130)
    plt.close(fig)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--drive", required=True)
    ap.add_argument("--scene", required=True)
    ap.add_argument("--frame", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    st = _stations(args.drive)
    sc, cov = fig_observed_vs_truth(st, args.scene, args.out)
    fig_levers(st, args.scene, args.out)
    fig_vslam_features(args.frame, args.out)
    fig_uncertainty(st, args.scene, args.out)
    print(f"wrote 4 figures to {args.out}  "
          f"(coverage {cov*100:.1f}%, RMSE {sc['map_rmse_m']:.3f} m)")
