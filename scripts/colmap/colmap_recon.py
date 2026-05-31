#!/usr/bin/env python3
"""foss_ipex COLMAP lane (M2b) — offline SfM/MVS reconstruction + recovered poses.

This is the "best-achievable reconstruction" benchmark that complements the online
rtabmap SLAM lane (M2-slam). Where rtabmap estimates poses *causally* (one frame at a
time, loop-closing as it goes), COLMAP gets to see ALL the images at once and solve a
global bundle adjustment — so its recovered trajectory is an upper bound on what a
feature-based estimator can do on this imagery. Scoring BOTH against the same Godot
ground truth (lane C, via the frozen eval_schema.TrajectorySample stream) shows how much
of rtabmap's error is "hard" (the regolith just doesn't have enough matchable structure)
versus "online" (causal/real-time cost). That contrast is the pipeline-visibility story.

A BIG advantage over real-world COLMAP: the Godot renderer KNOWS the camera intrinsics
exactly (sensor_bridge_contract §2.2 — fx=fy=(dim/2)/tan(fov/2), cx,cy centred, zero
distortion). So we feed COLMAP a fixed PINHOLE camera with those exact params
(`--ImageReader.camera_model PINHOLE --ImageReader.single_camera 1
--ImageReader.camera_params "fx,fy,cx,cy"`) and forbid it from refining them. Real SfM has
to self-calibrate; here the only unknowns are the poses + structure.

TWO SUBCOMMANDS (one HOST-side, one CONTAINER-side)
---------------------------------------------------
  render-arc   HOST: drive the symlinked Godot (.tools via godot_sidecar/render_layers.sh)
               to render N overlapping views arcing around a textured scene, writing
               views/NNN.png + ONE sensors.json carrying per-view camera intrinsics +
               ground-truth pose_in_world (Godot frame, contract §2.2 cameras[]). This is
               a SELF-VERIFICATION capture helper: the real input is the M2-egress moving
               sequence (Wave-2). Kept in this file (not a new script) to stay lane-confined.

  recon        CONTAINER (graffitytech/colmap, --gpus all): feature_extractor (known
               PINHOLE intrinsics) -> exhaustive_matcher -> mapper (sparse SfM) ->
               [optional --dense: image_undistorter -> patch_match_stereo -> stereo_fusion]
               -> parse the sparse model -> Sim3/Umeyama-align the recovered camera centres
               to the ground-truth camera pose_in_world from sensors.json -> emit an aligned
               eval_schema.TrajectorySample list (frame='map') so lane C can score COLMAP as
               an INDEPENDENT pose estimate alongside rtabmap.

FRAMES. sensors.json poses are 100% Godot-frame (contract §3 — the REP-103 conversion is
C1/frames.py's job, NOT ours). We align COLMAP (an arbitrary SfM gauge) to the GODOT
camera centres, so the emitted samples are in the *same Godot world gauge* as the truth.
We label frame='map' to match the TrajectorySample channel the scorer consumes; we do NOT
apply REP-103 here (that stays frozen in frames.py). Lane C compares COLMAP-vs-truth and
rtabmap-vs-truth in whatever single frame it standardises on — both estimates and the truth
go through the same conversion, so a consistent-but-unconverted gauge here is correct: the
alignment is truth-relative.

eval_schema.py is the FROZEN L0 seam, imported read-only. We add its dir to sys.path so
this file can run both on the host .venv and inside the container (where the repo is mounted
at /work). numpy is the only third-party dep (in the image; also in the host .venv).

CC0-1.0 (see ../../LICENSE).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
from dataclasses import dataclass
from typing import Optional

import numpy as np

# --- frozen L0 seam: eval_schema.TrajectorySample (read-only import) --------------------
# This file lives at scripts/colmap/; eval_schema.py lives at scripts/ros2_bridge/. Add
# that dir to sys.path so the import works on the host .venv AND inside the container
# (repo mounted at /work, this file run as /work/scripts/colmap/colmap_recon.py).
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROS2_BRIDGE = os.path.normpath(os.path.join(_HERE, "..", "ros2_bridge"))
if _ROS2_BRIDGE not in sys.path:
    sys.path.insert(0, _ROS2_BRIDGE)
from eval_schema import TrajectorySample  # noqa: E402  (frozen seam)


# =======================================================================================
# render-arc  (HOST side: drive the symlinked Godot)
# =======================================================================================

# The single-frame --pose render path (sidecar.gd::_setup_camera) builds a Camera3D with
# fov=55.0 and Godot's DEFAULT keep_aspect == KEEP_HEIGHT, so 55deg is the VERTICAL fov.
# Intrinsics therefore key off HEIGHT: fy = (h/2)/tan(fov_v/2), fx = fy (square pixels),
# cx = w/2, cy = h/2, zero distortion (rectified pinhole). This mirrors the contract §2.2
# intrinsics rule (camera_rig.intrinsics) but with the vertical-fov convention of the
# default camera, not the front-stereo rig's KEEP_WIDTH horizontal-fov convention.
SIDECAR_POSE_FOV_DEG = 55.0  # sidecar.gd::_setup_camera _cam.fov (KEEP_HEIGHT -> vertical)


def intrinsics_from_vertical_fov(fov_v_deg: float, w: int, h: int) -> dict:
    """Pinhole intrinsics for the default --pose camera (KEEP_HEIGHT, vertical fov)."""
    fy = (float(h) * 0.5) / math.tan(math.radians(fov_v_deg) * 0.5)
    return {
        "model": "pinhole",
        "fx": fy,  # square pixels
        "fy": fy,
        "cx": float(w) * 0.5,
        "cy": float(h) * 0.5,
        "distortion_model": "plumb_bob",
        "D": [0, 0, 0, 0, 0],
    }


def _godot_look_at_quat_xyzw(pos: np.ndarray, target: np.ndarray) -> list[float]:
    """Replicate Godot Camera3D.look_at_from_position(pos, target, UP=+Y) basis as a
    quaternion (XYZW, Godot frame). A Godot camera looks down its local -Z, +Y up, +X right.

    Godot's look_at builds: -Z = normalize(target - pos) [forward]; then with up=+Y,
        X (right) = normalize(up x (-forward_dir))  ... using its right-handed convention.
    We construct the same basis columns (X, Y, Z) and convert to a quaternion. This is the
    camera optical origin orientation in the Godot world frame (contract pose_in_world).
    """
    up = np.array([0.0, 1.0, 0.0])
    fwd = target - pos
    n = np.linalg.norm(fwd)
    if n < 1e-9:
        fwd = np.array([0.0, 0.0, -1.0])
    else:
        fwd = fwd / n
    z = -fwd  # camera back (local +Z points away from the look direction)
    x = np.cross(up, z)
    nx = np.linalg.norm(x)
    if nx < 1e-9:  # looking straight up/down: pick an arbitrary stable right axis
        x = np.cross(np.array([0.0, 0.0, 1.0]), z)
        nx = np.linalg.norm(x)
    x = x / nx
    y = np.cross(z, x)
    R = np.column_stack([x, y, z])  # basis columns (X, Y, Z) in world
    return _rot_to_quat_xyzw(R)


def _rot_to_quat_xyzw(R: np.ndarray) -> list[float]:
    """3x3 rotation matrix -> unit quaternion [x, y, z, w] (Shepperd's method)."""
    m00, m11, m22 = R[0, 0], R[1, 1], R[2, 2]
    tr = m00 + m11 + m22
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    return (q / np.linalg.norm(q)).tolist()


def cmd_render_arc(args: argparse.Namespace) -> int:
    """Render N overlapping views arcing around a scene centre + emit a matching
    sensors.json (per-view intrinsics + ground-truth camera pose_in_world, Godot frame).

    The arc is a partial ring at fixed radius + height, all cameras looking at the same
    surface centre, so consecutive views share large image overlap (good wide-baseline SfM
    coverage on the boulders) while still giving real translation parallax for pose recovery.
    """
    repo = os.path.normpath(os.path.join(_HERE, "..", ".."))
    scene_dir = os.path.join(repo, "samples", args.scene)
    meta_path = os.path.join(scene_dir, "metadata.json")
    if not os.path.isfile(meta_path):
        print(f"render-arc: no scene metadata at {meta_path}", file=sys.stderr)
        return 2
    with open(meta_path, "r", encoding="utf-8") as fh:
        meta = json.load(fh)
    wb = meta["world_bounds_m"]
    cx = 0.5 * (float(wb["x0"]) + float(wb["x1"]))
    cz = 0.5 * (float(wb["y0"]) + float(wb["y1"]))
    # Aim a touch below mean ground so the boulder field fills the lower frame (its relief
    # is the SfM signal); height_range_m gives a sensible target height.
    hr = meta.get("height_range_m", [0.0, 0.0])
    target_y = float(hr[0])
    centre = np.array([cx, target_y, cz])

    out_dir = os.path.abspath(args.out_dir)
    views_dir = out_dir
    os.makedirs(views_dir, exist_ok=True)

    render_sh = os.path.join(repo, "godot_sidecar", "render_layers.sh")
    intr = intrinsics_from_vertical_fov(SIDECAR_POSE_FOV_DEG, args.width, args.height)

    cameras = []
    n = args.views
    # Sweep an arc of `arc_deg` total, centred so the views look across the textured field.
    arc0 = math.radians(args.arc_start_deg)
    arc1 = math.radians(args.arc_start_deg + args.arc_deg)
    for i in range(n):
        frac = i / max(n - 1, 1)
        theta = arc0 + frac * (arc1 - arc0)
        px = cx + args.radius_m * math.cos(theta)
        pz = cz + args.radius_m * math.sin(theta)
        py = args.height_m
        pos = np.array([px, py, pz])
        name = f"{i:03d}"
        png_abs = os.path.join(views_dir, f"{name}.png")
        cmd = [
            render_sh, "--",
            "--scene", scene_dir,
            "--layers", "terrain,clasts",
            "--pose", f"{px:.5f},{py:.5f},{pz:.5f},{cx:.5f},{target_y:.5f},{cz:.5f}",
            "--size", f"{args.width}x{args.height}",
            "--sun-elev", str(args.sun_elev),
            "--sun-azim", str(args.sun_azim),
            "--out", png_abs,
        ]
        print(f"render-arc: view {name} pos=({px:.2f},{py:.2f},{pz:.2f})")
        r = subprocess.run(cmd, cwd=os.path.join(repo, "godot_sidecar"),
                           capture_output=True, text=True)
        if r.returncode != 0 or not os.path.isfile(png_abs):
            sys.stderr.write(r.stdout[-2000:] + "\n" + r.stderr[-2000:] + "\n")
            print(f"render-arc: render failed for view {name}", file=sys.stderr)
            return 3
        q = _godot_look_at_quat_xyzw(pos, centre)
        cameras.append({
            "name": name,
            "frame_id": f"view_{name}_optical",
            "image": f"{name}.png",
            "width": args.width,
            "height": args.height,
            "intrinsics": dict(intr),
            "pose_in_world": {"position_m": [px, py, pz], "quaternion_xyzw": q},
        })

    sensors = {
        "schema_version": "sensor_bridge/1.1",
        "scene": meta.get("scene_name", args.scene),
        "frame_index": 0,
        "frame_convention": "godot",
        "sun": {"elevation_deg": args.sun_elev, "azimuth_deg": args.sun_azim,
                "time_delta_s": 0.0},
        "capture": {
            "kind": "colmap_arc",
            "radius_m": args.radius_m, "height_m": args.height_m,
            "arc_start_deg": args.arc_start_deg, "arc_deg": args.arc_deg,
            "note": "M2b self-verification arc; real input is the M2-egress moving "
                    "sequence (Wave-2). Each cameras[] entry is one VIEW (a separate "
                    "rendered frame), not a simultaneous rig.",
        },
        "cameras": cameras,
    }
    sj = os.path.join(views_dir, "sensors.json")
    with open(sj, "w", encoding="utf-8") as fh:
        json.dump(sensors, fh, indent=2)
    print(f"render-arc: wrote {len(cameras)} views + {sj}")
    return 0


# =======================================================================================
# recon  (CONTAINER side: COLMAP SfM/MVS + Umeyama align)
# =======================================================================================

def _run(cmd: list[str], log: str) -> None:
    print(f"\n$ {' '.join(cmd)}")
    r = subprocess.run(cmd, capture_output=True, text=True)
    sys.stdout.write(r.stdout)
    if r.returncode != 0:
        sys.stderr.write(r.stderr)
        raise RuntimeError(f"{log}: exit {r.returncode}")
    # COLMAP writes useful stats to stderr too.
    sys.stdout.write(r.stderr)


@dataclass
class ColmapImage:
    name: str
    qw: float
    qx: float
    qy: float
    qz: float
    tx: float
    ty: float
    tz: float

    def center(self) -> np.ndarray:
        """Camera centre C in the COLMAP world: x_cam = R*X + t  =>  C = -R^T t."""
        R = _quat_wxyz_to_rot(self.qw, self.qx, self.qy, self.qz)
        t = np.array([self.tx, self.ty, self.tz])
        return -R.T @ t


def _quat_wxyz_to_rot(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if n < 1e-12:
        return np.eye(3)
    qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def parse_images_txt(path: str) -> dict[str, ColmapImage]:
    """Parse a COLMAP model_converter TXT images.txt -> {image_name: ColmapImage}.

    Format: two lines per registered image; the first is
        IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME
    the second is the 2D-point list (skipped). Comment lines start with '#'.
    """
    out: dict[str, ColmapImage] = {}
    with open(path, "r", encoding="utf-8") as fh:
        lines = [ln for ln in fh.read().splitlines()]
    i = 0
    while i < len(lines):
        ln = lines[i].strip()
        if not ln or ln.startswith("#"):
            i += 1
            continue
        parts = ln.split()
        # IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME
        if len(parts) >= 10:
            name = parts[9]
            out[name] = ColmapImage(
                name=name,
                qw=float(parts[1]), qx=float(parts[2]),
                qy=float(parts[3]), qz=float(parts[4]),
                tx=float(parts[5]), ty=float(parts[6]), tz=float(parts[7]),
            )
        i += 2  # skip the points2D line that follows each image header
    return out


def count_points3d_txt(path: str) -> int:
    n = 0
    with open(path, "r", encoding="utf-8") as fh:
        for ln in fh:
            s = ln.strip()
            if s and not s.startswith("#"):
                n += 1
    return n


def umeyama_sim3(src: np.ndarray, dst: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:
    """Least-squares Sim(3) (similarity) alignment src -> dst (Umeyama 1991).

    src, dst are (N,3). Returns (scale s, rotation R (3x3), translation t (3,)) such that
        dst ~= s * R @ src + t.
    COLMAP's reconstruction is only defined up to a similarity transform (gauge freedom);
    aligning the recovered camera centres to the metric Godot ground-truth centres recovers
    that gauge so the residual is the real pose error.
    """
    assert src.shape == dst.shape and src.shape[1] == 3
    n = src.shape[0]
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    sc = src - mu_s
    dc = dst - mu_d
    cov = (dc.T @ sc) / n
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1.0
    R = U @ S @ Vt
    var_s = (sc ** 2).sum() / n
    s = float(np.trace(np.diag(D) @ S) / var_s) if var_s > 1e-12 else 1.0
    t = mu_d - s * R @ mu_s
    return s, R, t


def cmd_recon(args: argparse.Namespace) -> int:
    images_dir = os.path.abspath(args.images)
    sensors_path = args.sensors or os.path.join(images_dir, "sensors.json")
    work = os.path.abspath(args.work)
    os.makedirs(work, exist_ok=True)
    db_path = os.path.join(work, "database.db")
    sparse_dir = os.path.join(work, "sparse")
    os.makedirs(sparse_dir, exist_ok=True)

    with open(sensors_path, "r", encoding="utf-8") as fh:
        sensors = json.load(fh)
    cams = sensors["cameras"]
    intr = cams[0]["intrinsics"]  # constant across views (contract §2.5)
    fx, fy = float(intr["fx"]), float(intr["fy"])
    cx, cy = float(intr["cx"]), float(intr["cy"])
    cam_params = f"{fx},{fy},{cx},{cy}"
    print(f"recon: feeding COLMAP EXACT known PINHOLE intrinsics {cam_params}")

    colmap = args.colmap_bin

    # (a) feature extraction with the EXACT known intrinsics, single shared camera, GPU.
    _run([
        colmap, "feature_extractor",
        "--database_path", db_path,
        "--image_path", images_dir,
        "--ImageReader.camera_model", "PINHOLE",
        "--ImageReader.single_camera", "1",
        "--ImageReader.camera_params", cam_params,
        "--SiftExtraction.use_gpu", "1",
    ], "feature_extractor")

    # (b) exhaustive matching (small N views) on the GPU.
    _run([
        colmap, "exhaustive_matcher",
        "--database_path", db_path,
        "--SiftMatching.use_gpu", "1",
    ], "exhaustive_matcher")

    # (b cont.) sparse SfM. Critically forbid refining the (known-exact) intrinsics.
    _run([
        colmap, "mapper",
        "--database_path", db_path,
        "--image_path", images_dir,
        "--output_path", sparse_dir,
        "--Mapper.ba_refine_focal_length", "0",
        "--Mapper.ba_refine_principal_point", "0",
        "--Mapper.ba_refine_extra_params", "0",
    ], "mapper")

    # mapper writes one or more models under sparse/0, sparse/1, ... pick the largest.
    model_dirs = sorted(
        d for d in os.listdir(sparse_dir) if os.path.isdir(os.path.join(sparse_dir, d))
    )
    if not model_dirs:
        print("recon: mapper produced NO model — reconstruction failed "
              "(insufficient matched structure on the regolith).", file=sys.stderr)
        return 4
    best = None
    best_n = -1
    for d in model_dirs:
        m = os.path.join(sparse_dir, d)
        txt = os.path.join(m, "images.txt")
        if not os.path.isfile(txt):
            _run([colmap, "model_converter", "--input_path", m,
                  "--output_path", m, "--output_type", "TXT"], "model_converter")
        imgs = parse_images_txt(os.path.join(m, "images.txt"))
        if len(imgs) > best_n:
            best_n, best = len(imgs), m
    model = best
    print(f"recon: selected sparse model {model} ({best_n} registered images)")

    # model_analyzer prints registered images / points / mean reprojection error.
    print("\n--- colmap model_analyzer ---")
    r = subprocess.run([colmap, "model_analyzer", "--path", model],
                       capture_output=True, text=True)
    analyzer = (r.stdout or "") + (r.stderr or "")
    sys.stdout.write(analyzer)
    mean_reproj = _grep_float(analyzer, "Mean reprojection error")
    n_points = count_points3d_txt(os.path.join(model, "points3D.txt"))
    colmap_imgs = parse_images_txt(os.path.join(model, "images.txt"))
    n_reg = len(colmap_imgs)

    # (c) OPTIONAL dense MVS, gated behind --dense.
    if args.dense:
        dense_dir = os.path.join(work, "dense")
        os.makedirs(dense_dir, exist_ok=True)
        _run([colmap, "image_undistorter",
              "--image_path", images_dir, "--input_path", model,
              "--output_path", dense_dir, "--output_type", "COLMAP"], "image_undistorter")
        _run([colmap, "patch_match_stereo",
              "--workspace_path", dense_dir, "--workspace_format", "COLMAP",
              "--PatchMatchStereo.geom_consistency", "1"], "patch_match_stereo")
        _run([colmap, "stereo_fusion",
              "--workspace_path", dense_dir, "--workspace_format", "COLMAP",
              "--input_type", "geometric",
              "--output_path", os.path.join(dense_dir, "fused.ply")], "stereo_fusion")
        print(f"recon: dense fused cloud -> {os.path.join(dense_dir, 'fused.ply')}")

    # (d) align recovered camera centres -> ground-truth camera pose_in_world (Godot frame).
    gt_by_name: dict[str, np.ndarray] = {}
    gt_quat_by_name: dict[str, list] = {}
    for c in cams:
        img_name = c["image"]
        p = c["pose_in_world"]["position_m"]
        gt_by_name[img_name] = np.array([float(p[0]), float(p[1]), float(p[2])])
        gt_quat_by_name[img_name] = [float(x) for x in c["pose_in_world"]["quaternion_xyzw"]]

    matched = [(n, colmap_imgs[n].center(), gt_by_name[n])
               for n in colmap_imgs if n in gt_by_name]
    matched.sort(key=lambda e: e[0])
    align = None
    samples: list[dict] = []
    if len(matched) >= 3:
        src = np.array([m[1] for m in matched])
        dst = np.array([m[2] for m in matched])
        s, R, t = umeyama_sim3(src, dst)
        aligned = (s * (R @ src.T)).T + t
        resid = np.linalg.norm(aligned - dst, axis=1)
        align = {
            "scale": s,
            "n_aligned": len(matched),
            "ate_rmse_m": float(math.sqrt(float((resid ** 2).mean()))),
            "ate_max_m": float(resid.max()),
        }
        print(f"\nrecon: Umeyama Sim3 align over {len(matched)} cams: "
              f"scale={s:.4f} ATE_rmse={align['ate_rmse_m']*1000:.1f}mm "
              f"ATE_max={align['ate_max_m']*1000:.1f}mm")
        # Emit one TrajectorySample per registered+matched view, in the Godot map gauge.
        for idx, (name, _c, _gt) in enumerate(matched):
            stem = name.rsplit(".", 1)[0]
            frame_index = int(stem) if stem.isdigit() else idx
            pos = aligned[idx].tolist()
            # Recovered orientation, rotated into the aligned (truth) gauge. COLMAP image
            # qvec is world->cam; camera orientation in world is R_wc = R_cw^T. We then
            # left-apply the Umeyama rotation R to land in the truth gauge.
            ci = colmap_imgs[name]
            R_cw = _quat_wxyz_to_rot(ci.qw, ci.qx, ci.qy, ci.qz)
            R_wc = R_cw.T
            R_aligned = R @ R_wc
            quat = _rot_to_quat_xyzw(R_aligned)
            ts = TrajectorySample(
                frame_index=frame_index,
                t_s=float(idx),
                position_m=pos,
                quaternion_xyzw=quat,
                frame="map",
            )
            samples.append(ts.to_dict())
    else:
        print(f"\nrecon: only {len(matched)} registered view(s) matched to truth — "
              "cannot Sim3-align (need >=3). Emitting no trajectory (honest partial "
              "reconstruction on regolith).", file=sys.stderr)

    report = {
        "scene": sensors.get("scene"),
        "n_input_views": len(cams),
        "n_registered": n_reg,
        "n_points3d": n_points,
        "mean_reproj_error_px": mean_reproj,
        "dense": bool(args.dense),
        "alignment": align,
        "trajectory_frame": "map",
        "trajectory": samples,
        "note": "COLMAP offline SfM (global BA) — upper-bound pose estimate vs the causal "
                "rtabmap lane. Poses in the Godot world gauge aligned to ground-truth "
                "camera centres (Umeyama Sim3); REP-103 stays frozen in frames.py.",
    }
    out_json = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out_json) or ".", exist_ok=True)
    with open(out_json, "w", encoding="utf-8") as fh:
        json.dump(report, fh, indent=2)
    print(f"\nrecon: registered={n_reg}/{len(cams)} points3d={n_points} "
          f"mean_reproj={mean_reproj} -> {out_json}")
    return 0


def _grep_float(text: str, key: str) -> Optional[float]:
    """Pull the numeric value that follows `key` on its line.

    COLMAP log lines are prefixed with a glog timestamp ("I20260531 03:29:22.260799 ..")
    whose colons/digits would poison a naive left-to-right scan — so we slice the line at
    `key` and parse only the tail after it (e.g. "...error: 0.298141px" -> 0.298141).
    """
    for ln in text.splitlines():
        idx = ln.find(key)
        if idx < 0:
            continue
        tail = ln[idx + len(key):].replace(":", " ").replace("px", " ")
        for tok in tail.split():
            try:
                return float(tok)
            except ValueError:
                continue
    return None


# =======================================================================================
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    ra = sub.add_parser("render-arc", help="HOST: render an overlapping arc of views + sensors.json")
    ra.add_argument("--scene", default="boulder_field",
                    help="scene dir under samples/ (default: boulder_field — gritty SfM features)")
    ra.add_argument("--out-dir", default="out/colmap/views", dest="out_dir")
    ra.add_argument("--views", type=int, default=12)
    ra.add_argument("--radius-m", type=float, default=2.6, dest="radius_m")
    ra.add_argument("--height-m", type=float, default=1.1, dest="height_m")
    ra.add_argument("--arc-start-deg", type=float, default=200.0, dest="arc_start_deg")
    ra.add_argument("--arc-deg", type=float, default=140.0, dest="arc_deg")
    ra.add_argument("--width", type=int, default=1024)
    ra.add_argument("--height", type=int, default=768)
    ra.add_argument("--sun-elev", type=float, default=24.0, dest="sun_elev")
    ra.add_argument("--sun-azim", type=float, default=215.0, dest="sun_azim")
    ra.set_defaults(func=cmd_render_arc)

    rc = sub.add_parser("recon", help="CONTAINER: COLMAP SfM/MVS + Umeyama align -> TrajectorySample")
    rc.add_argument("--images", required=True, help="dir of NNN.png views")
    rc.add_argument("--sensors", default=None, help="sensors.json (default: <images>/sensors.json)")
    rc.add_argument("--work", default="out/colmap/recon", help="COLMAP working dir (db, sparse, dense)")
    rc.add_argument("--out", default="out/colmap/recon/colmap_trajectory.json")
    rc.add_argument("--dense", action="store_true", help="also run dense MVS (image_undistorter -> patch_match_stereo -> stereo_fusion)")
    rc.add_argument("--colmap-bin", default="colmap", dest="colmap_bin")
    rc.set_defaults(func=cmd_recon)

    args = ap.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
