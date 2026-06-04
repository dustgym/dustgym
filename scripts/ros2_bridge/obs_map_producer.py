#!/usr/bin/env python3
"""Section 10 MAP-CHANNEL observed-map PRODUCER: rover front-stereo egress -> observed heightfield.

This is the producer half of the LAC map channel; `score_map.py` is the scorer. It reads a
front-stereo `out/cam/<scene>/<NNN>/` egress (front_left.png, front_right.png, sensors.json),
rectifies the pair with the EXACT known relative camera pose, runs SGBM, back-projects every
valid pixel into the authority world frame, and grids the points to an observed heightfield on
the scene grid plus a valid_mask of covered cells. The output feeds `score_map(observed, truth)`.

Rectification matters: the front stereo baseline runs along the rover-local Z, which lands on the
IMAGE-VERTICAL axis, so naive horizontal SGBM matches noise. Both camera world poses are in
sensors.json, so we compute the left->right relative pose, convert it to the optical frame, and
let cv2.stereoRectify horizontalize the epipolar lines.

Honest scope (validated on the crater_boulders render): the producer recovers the GROUND PLANE
(observed median within ~0.08 m of truth) and COVERAGE grows as the rover drives (2.6% -> 16.4%
over an 8-station traverse), but passive stereo at the rover's ~0.15 m eye-height has ~0.3 m (1
sigma) height precision. The rover-scale sample scenes have only cm-scale relief (std ~0.05 m),
which is BELOW that floor -- so the producer resolves coverage and ground level, not the
micro-relief that governs trafficability. That floor is a real perception limit (it motivates
active sensing and the conserved-physics ground truth), not a defect; the producer never
fabricates a height for an unobserved cell.

Frame inversion (terrain.gd:164): a terrain vertex (col,row,height) sits at Godot world
(x0+col*cell, height, y0+row*cell), so a Godot point (gx,gy,gz) is cell row=(gz-y0)/cell,
col=(gx-x0)/cell, elevation gy. Requires cv2 (host + runtime venv have 4.13). No synthetic data.
"""
from __future__ import annotations

import json
import os

import cv2
import numpy as np

# Far stereo on a ~5 m patch from a 0.07 m baseline is unreliable; cap it so far mismatches do
# not inject spurious far-and-high points into the observed map.
MAX_DEPTH_M = 4.0

# Godot camera-node <-> optical basis change (flip Y,Z). Same conversion for every camera.
_C = np.diag([1.0, -1.0, -1.0])


def quat_xyzw_to_R(q) -> np.ndarray:
    """Unit quaternion (x,y,z,w) -> 3x3 rotation matrix (columns = rotated basis axes)."""
    x, y, z, w = (float(v) for v in q)
    n = (x * x + y * y + z * z + w * w) ** 0.5
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def collect_world_points(egress_dir: str, max_depth_m: float = MAX_DEPTH_M) -> np.ndarray:
    """One front-stereo egress -> Nx3 back-projected world points (Godot frame), via proper
    stereo rectification using the exact known relative pose between front_left and front_right."""
    s = json.load(open(os.path.join(egress_dir, "sensors.json")))
    Lc = next(c for c in s["cameras"] if c["name"] == s["stereo"]["left"])
    Rc = next(c for c in s["cameras"] if c["name"] == s["stereo"]["right"])
    intr = Lc["intrinsics"]
    K = np.array([[intr["fx"], 0, intr["cx"]], [0, intr["fy"], intr["cy"]], [0, 0, 1.0]])
    D = np.zeros(5)  # plumb_bob, all-zero (rectified pinhole render)
    RL = quat_xyzw_to_R(Lc["pose_in_world"]["quaternion_xyzw"])
    tL = np.asarray(Lc["pose_in_world"]["position_m"])
    RR = quat_xyzw_to_R(Rc["pose_in_world"]["quaternion_xyzw"])
    tR = np.asarray(Rc["pose_in_world"]["position_m"])
    # relative left->right in the left node frame, converted to the optical frame
    R_rel = _C @ (RL.T @ RR) @ _C
    T_rel = _C @ (RL.T @ (tR - tL))
    left = cv2.imread(os.path.join(egress_dir, "front_left.png"), cv2.IMREAD_GRAYSCALE)
    right = cv2.imread(os.path.join(egress_dir, "front_right.png"), cv2.IMREAD_GRAYSCALE)
    if left is None or right is None:
        raise SystemExit("could not read front_left.png / front_right.png in " + egress_dir)
    h, w = left.shape
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(K, D, K, D, (w, h), R_rel, T_rel,
                                                flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
    m1x, m1y = cv2.initUndistortRectifyMap(K, D, R1, P1, (w, h), cv2.CV_32FC1)
    m2x, m2y = cv2.initUndistortRectifyMap(K, D, R2, P2, (w, h), cv2.CV_32FC1)
    lr = cv2.remap(left, m1x, m1y, cv2.INTER_LINEAR)
    rr = cv2.remap(right, m2x, m2y, cv2.INTER_LINEAR)
    blk = 5
    sg = cv2.StereoSGBM_create(
        minDisparity=0, numDisparities=160, blockSize=blk, P1=8 * blk * blk, P2=32 * blk * blk,
        disp12MaxDiff=2, uniquenessRatio=5, speckleWindowSize=80, speckleRange=4,
        preFilterCap=31, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
    disp = sg.compute(lr, rr).astype(np.float32) / 16.0
    pts3 = cv2.reprojectImageTo3D(disp, Q)  # rectified-left optical frame
    valid = (disp > 0.5) & np.isfinite(pts3).all(2) & (pts3[:, :, 2] > 0) & (pts3[:, :, 2] <= max_depth_m)
    pr = pts3[valid]            # N x 3 rectified-left optical
    pl_opt = pr @ R1           # R1: left->rectified, so left = R1^T @ p  =>  p @ R1
    pl_node = pl_opt @ _C.T    # optical -> node
    return pl_node @ RL.T + tL  # node -> world (Godot frame)


def grid_to_heightfield(points_world: np.ndarray, grid: dict, agg: str = "median"):
    """Grid Godot-world points to an observed heightfield on the authority grid.

    `grid` = {width, height, cell_m, x0, y0}. Returns (obs_height HxW float, valid_mask HxW bool).
    A Godot point (gx,gy,gz) -> cell row=(gz-y0)/cell, col=(gx-x0)/cell, elevation gy.
    """
    W = int(grid["width"])
    H = int(grid["height"])
    cell = float(grid["cell_m"])
    x0 = float(grid["x0"])
    y0 = float(grid["y0"])
    gx, gy, gz = points_world[:, 0], points_world[:, 1], points_world[:, 2]
    col = np.floor((gx - x0) / cell).astype(np.int64)
    row = np.floor((gz - y0) / cell).astype(np.int64)
    inb = (col >= 0) & (col < W) & (row >= 0) & (row < H)
    row, col, gy = row[inb], col[inb], gy[inb]
    obs = np.full((H, W), np.nan, dtype=np.float64)
    if row.size == 0:
        return np.zeros((H, W)), np.zeros((H, W), dtype=bool)
    flat = row * W + col
    order = np.argsort(flat, kind="stable")
    flat_s, gy_s = flat[order], gy[order]
    uniq, starts = np.unique(flat_s, return_index=True)
    ends = np.r_[starts[1:], flat_s.size]
    reducer = np.median if agg == "median" else np.max
    for cellflat, a, b in zip(uniq, starts, ends):
        r, c = divmod(int(cellflat), W)
        obs[r, c] = reducer(gy_s[a:b])
    valid_mask = ~np.isnan(obs)
    return np.where(valid_mask, obs, 0.0), valid_mask


# Single-view cells get this prior height-sigma (the measured passive-stereo 1-sigma floor at the
# rover's grazing eye-height); multi-view cells get the empirical standard error of the mean instead.
PRIOR_SIGMA_M = 0.30


def grid_to_heightfield_uncertainty(points_world: np.ndarray, grid: dict, agg: str = "median"):
    """Like grid_to_heightfield, but also returns a per-cell height UNCERTAINTY field + observation count.

    Returns (obs HxW, sigma HxW, count HxW int, mask HxW bool). For a cell with n>=2 back-projected
    points, sigma is the standard error of the mean height (std/sqrt(n)) -- the uncertainty of the
    cell's height ESTIMATE, which falls as more views accumulate. A cell with one point gets the
    PRIOR_SIGMA_M floor; unobserved cells are masked (sigma left at inf). This is the world model's
    Uncertainty layer per-cell height_uncertainty[x,y]; the planner gates digging on it.
    """
    W = int(grid["width"])
    H = int(grid["height"])
    cell = float(grid["cell_m"])
    x0 = float(grid["x0"])
    y0 = float(grid["y0"])
    gx, gy, gz = points_world[:, 0], points_world[:, 1], points_world[:, 2]
    col = np.floor((gx - x0) / cell).astype(np.int64)
    row = np.floor((gz - y0) / cell).astype(np.int64)
    inb = (col >= 0) & (col < W) & (row >= 0) & (row < H)
    row, col, gy = row[inb], col[inb], gy[inb]
    obs = np.zeros((H, W))
    sigma = np.full((H, W), np.inf)
    count = np.zeros((H, W), dtype=np.int64)
    if row.size == 0:
        return obs, sigma, count, np.zeros((H, W), dtype=bool)
    flat = row * W + col
    order = np.argsort(flat, kind="stable")
    flat_s, gy_s = flat[order], gy[order]
    uniq, starts = np.unique(flat_s, return_index=True)
    ends = np.r_[starts[1:], flat_s.size]
    reducer = np.median if agg == "median" else np.max
    for cellflat, a, b in zip(uniq, starts, ends):
        r, c = divmod(int(cellflat), W)
        vals = gy_s[a:b]
        n = vals.size
        obs[r, c] = reducer(vals)
        count[r, c] = n
        sigma[r, c] = float(np.std(vals) / np.sqrt(n)) if n >= 2 else PRIOR_SIGMA_M
    mask = count > 0
    return obs, sigma, count, mask


def produce_uncertainty_map(egress_dirs, grid: dict):
    """Accumulate stations -> (obs, sigma, count, mask): the observed heightfield + per-cell height
    uncertainty + observation count. The Uncertainty layer of the world model."""
    pts = [collect_world_points(d) for d in egress_dirs]
    pts = [p for p in pts if p.size]
    if not pts:
        W, H = int(grid["width"]), int(grid["height"])
        return (np.zeros((H, W)), np.full((H, W), np.inf), np.zeros((H, W), dtype=np.int64),
                np.zeros((H, W), dtype=bool))
    return grid_to_heightfield_uncertainty(np.concatenate(pts, axis=0), grid)


def dig_ready_mask(sigma: np.ndarray, mask: np.ndarray, tol_m: float = 0.10):
    """Cells confident enough to act on: observed AND height-sigma below tol. The 'need more
    observations before digging' gate -- its complement (observed-but-uncertain, or unobserved) is
    where the planner should look before committing."""
    return mask & (sigma <= tol_m)


def grid_from_metadata(meta_path: str) -> dict:
    """Build the grid dict from a scene metadata.json (INTERFACE.md layout)."""
    m = json.load(open(meta_path))
    g = m["grid"]
    wb = m["world_bounds_m"]
    return {"width": int(g["width"]), "height": int(g["height"]),
            "cell_m": float(g["cell_m"]), "x0": float(wb["x0"]), "y0": float(wb["y0"])}


def load_truth_heightmap(scene_dir: str, grid: dict) -> np.ndarray:
    """Read the scene heightmap.rf32 (row-major C float32) as the truth-at-t heightfield."""
    h = np.fromfile(os.path.join(scene_dir, "heightmap.rf32"), dtype="<f4")
    return h.reshape(int(grid["height"]), int(grid["width"])).astype(np.float64)


def produce_observed_map(egress_dir: str, grid: dict):
    """Single station: egress -> rectified stereo -> back-project -> observed heightfield + mask."""
    return grid_to_heightfield(collect_world_points(egress_dir), grid)


def produce_observed_map_multi(egress_dirs, grid: dict):
    """Accumulate many front-stereo stations (a driven path) into ONE observed heightfield + mask.

    The genuine LAC map-by-driving approach: a single rover stereo pose sees only a thin grazing
    swath, so coverage grows as the rover drives and each station's back-projected points are
    merged on the shared grid (per-cell median over all points that landed there)."""
    pts = [collect_world_points(d) for d in egress_dirs]
    pts = [p for p in pts if p.size]
    if not pts:
        W, H = int(grid["width"]), int(grid["height"])
        return np.zeros((H, W)), np.zeros((H, W), dtype=bool)
    return grid_to_heightfield(np.concatenate(pts, axis=0), grid)


if __name__ == "__main__":
    import argparse

    from score_map import score_map

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--drive", required=True, help="dir of station subdirs (each a front-stereo egress)")
    ap.add_argument("--scene", required=True, help="samples/<scene>/ (metadata.json + heightmap.rf32)")
    args = ap.parse_args()
    grid = grid_from_metadata(os.path.join(args.scene, "metadata.json"))
    truth = load_truth_heightmap(args.scene, grid)
    stations = [os.path.join(args.drive, d) for d in sorted(os.listdir(args.drive))
                if os.path.isfile(os.path.join(args.drive, d, "sensors.json"))]
    obs, mask = produce_observed_map_multi(stations, grid)
    sc = score_map(obs, truth, tol_m=0.10, valid_mask=mask)
    err = obs[mask] - truth[mask]
    print(f"stations={len(stations)}  coverage={mask.mean()*100:.1f}%  "
          f"map_rmse_m={sc['map_rmse_m']:.3f}  cell_pass={sc['map_cell_pass_frac']*100:.1f}%  "
          f"ground_recovery(obs_median={np.median(obs[mask]):+.3f} vs truth={np.median(truth[mask]):+.3f})  "
          f"precision_1sigma={err.std():.3f} m")
