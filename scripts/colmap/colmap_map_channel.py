#!/usr/bin/env python3
"""GROUND-tier map-channel producer: offline COLMAP SfM over an image corpus -> observed heightfield.

The high-accuracy counterpart to the onboard rover-stereo producer (`obs_map_producer.py`). It runs
pycolmap incremental SfM over a multi-view corpus (rendered by `render_corpus.py`, with known camera
poses), then -- because SfM is solved only up to a similarity transform -- Umeyama-aligns the recovered
camera centers to the KNOWN render poses to put the sparse point cloud in the world (Godot) frame, grids
it to an observed heightfield, and scores it against the conserved truth with `score_map`.

Run twice (a Hapke corpus and a Lambert corpus) for the A/B that shows the non-Lambertian regolith BRDF
degrading multi-view photoconsistency. Needs pycolmap (in the runtime venv); no Docker.

Usage:
    <venv>/bin/python colmap_map_channel.py --corpus <out/corpus_hapke> --scene <samples/crater_boulders> \
        [--work <tmp colmap workdir>]
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
import tempfile

import numpy as np
import pycolmap

# reuse the gridding + truth loader + scorer from the onboard producer / scorer
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ros2_bridge"))
import obs_map_producer as omp  # noqa: E402
from score_map import score_map  # noqa: E402


def umeyama(src: np.ndarray, dst: np.ndarray):
    """Similarity transform (scale s, rotation R, translation t) mapping src -> dst (Nx3 each).

    Returns (s, R, t) minimizing ||dst - (s R src + t)||. Umeyama 1991."""
    mu_s = src.mean(0)
    mu_d = dst.mean(0)
    src_c = src - mu_s
    dst_c = dst - mu_d
    cov = dst_c.T @ src_c / src.shape[0]
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    var_s = (src_c ** 2).sum() / src.shape[0]
    s = float((D * np.diag(S)).sum() / var_s)
    t = mu_d - s * R @ mu_s
    return s, R, t


def run_sfm(corpus_dir: str, work_dir: str):
    """pycolmap incremental SfM over corpus_dir/*.png -> the largest Reconstruction (or None)."""
    db = os.path.join(work_dir, "database.db")
    img_dir = corpus_dir
    out_dir = os.path.join(work_dir, "sparse")
    os.makedirs(out_dir, exist_ok=True)
    pycolmap.extract_features(db, img_dir, camera_mode=pycolmap.CameraMode.SINGLE)
    pycolmap.match_exhaustive(db)
    recs = pycolmap.incremental_mapping(db, img_dir, out_dir)
    if not recs:
        return None
    return max(recs.values(), key=lambda r: r.num_reg_images())


def _camera_center(image) -> np.ndarray:
    """World-frame camera center from a pycolmap Image (handles API variants)."""
    try:
        return np.asarray(image.projection_center(), dtype=float)
    except AttributeError:
        cfw = image.cam_from_world          # world -> cam (Rigid3d)
        R = np.asarray(cfw.rotation.matrix())
        t = np.asarray(cfw.translation)
        return -R.T @ t


def colmap_observed_map(corpus_dir: str, scene_dir: str, work_dir: str):
    """Full ground-tier pipeline -> (observed, mask, truth, metrics dict)."""
    manifest = json.load(open(os.path.join(corpus_dir, "poses.json")))
    known = {fr["name"]: np.asarray(fr["pos"], dtype=float) for fr in manifest["frames"]}

    rec = run_sfm(corpus_dir, work_dir)
    grid = omp.grid_from_metadata(os.path.join(scene_dir, "metadata.json"))
    truth = omp.load_truth_heightmap(scene_dir, grid)
    if rec is None or rec.num_reg_images() < 3:
        return None, None, truth, {"registered": 0 if rec is None else rec.num_reg_images()}

    # match recovered camera centers (COLMAP frame) to the known world centers, by image name
    col_c, wld_c = [], []
    for img in rec.images.values():
        if img.name in known:
            col_c.append(_camera_center(img))
            wld_c.append(known[img.name])
    col_c = np.asarray(col_c)
    wld_c = np.asarray(wld_c)
    s, R, t = umeyama(col_c, wld_c)
    align_rmse = float(np.sqrt(((wld_c - (s * (col_c @ R.T) + t)) ** 2).sum(1).mean()))

    # transform the sparse 3-D points COLMAP -> world (Godot) frame
    pts = np.asarray([p.xyz for p in rec.points3D.values()], dtype=float)
    pts_world = s * (pts @ R.T) + t
    obs, mask = omp.grid_to_heightfield(pts_world, grid)
    sc = score_map(obs, truth, tol_m=0.10, valid_mask=mask)
    metrics = {
        "registered": rec.num_reg_images(),
        "n_images": len(manifest["frames"]),
        "n_points3D": len(pts),
        "mean_reproj_px": float(rec.compute_mean_reprojection_error()),
        "align_rmse_m": align_rmse,
        "coverage": float(mask.mean()),
        "map_rmse_m": sc["map_rmse_m"],
        "cell_pass_frac": sc["map_cell_pass_frac"],
    }
    return obs, mask, truth, metrics


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--corpus", required=True, help="corpus dir (images + poses.json)")
    ap.add_argument("--scene", required=True, help="samples/<scene>/")
    ap.add_argument("--work", default=None, help="colmap work dir (default: a temp dir)")
    args = ap.parse_args()
    work = args.work or tempfile.mkdtemp(prefix="colmap_")
    made = args.work is None
    try:
        _, _, _, m = colmap_observed_map(args.corpus, args.scene, work)
        print(json.dumps(m, indent=2))
    finally:
        if made:
            shutil.rmtree(work, ignore_errors=True)


if __name__ == "__main__":
    main()
