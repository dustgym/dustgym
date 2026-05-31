# COLMAP offline SfM/MVS lane (M2b)

The **"best-achievable reconstruction + recovered poses"** benchmark that complements the
online [rtabmap SLAM lane](../ros2_bridge/README.md). Where rtabmap estimates the rover
trajectory **causally** (frame by frame, loop-closing online), COLMAP gets to see **all**
the images at once and solve a global bundle adjustment, so its recovered poses are an
*upper bound* on what a feature-based estimator can do on this imagery. Scoring **both**
against the same Godot ground truth (lane C, via the frozen
[`eval_schema.TrajectorySample`](../ros2_bridge/eval_schema.py) stream) separates "hard"
error (the regolith just doesn't have enough matchable structure) from "online" cost
(real-time / causal). That contrast is the pipeline-visibility story.

A big advantage over real-world COLMAP: the Godot renderer **knows the camera intrinsics
exactly** (`sensor_bridge_contract` §2.2 — `fx=fy=(dim/2)/tan(fov/2)`, centred principal
point, zero distortion). We feed COLMAP a fixed `PINHOLE` camera with those exact params
and forbid bundle adjustment from refining them, so the only unknowns are poses + structure.

## Contents

| File | Role |
|---|---|
| `colmap.Dockerfile` | `FROM graffitytech/colmap:3.12.2-cuda12.8.1-devel-ubuntu24.04` (driver-matched CUDA 12.8.1; `--gpus all`, **no** `NVIDIA_DISABLE_REQUIRE`) + `python3` + `numpy` for the wrapper. |
| `colmap_recon.py` | Two subcommands: `render-arc` (HOST: drive the symlinked Godot to render an overlapping arc of views + a matching `sensors.json`) and `recon` (CONTAINER: COLMAP SfM/MVS + Umeyama Sim3 align → `TrajectorySample` JSON). |

## Build

```bash
# from the repo root (the build context is scripts/colmap/)
docker build -f scripts/colmap/colmap.Dockerfile -t fossipex/colmap:m2b scripts/colmap/

# smoke test (versions + GPU visibility):
docker run --rm --gpus all fossipex/colmap:m2b
docker run --rm --gpus all fossipex/colmap:m2b nvidia-smi --query-gpu=name --format=csv,noheader
```

The base layer is already pulled, so the build is fast (only the `python3`/`numpy` apt layer).

## Run

### 1. Capture a multi-view set (HOST — uses the symlinked Godot in `.tools`)

`render-arc` drives `godot_sidecar/render_layers.sh --pose ...` at N viewpoints arcing
around a textured scene, all looking at the same surface centre (large overlap + real
translation parallax). It writes `NNN.png` + ONE `sensors.json` carrying per-view
intrinsics and **ground-truth camera `pose_in_world`** (Godot frame, contract §2.2).

```bash
python3 scripts/colmap/colmap_recon.py render-arc \
    --scene boulder_field \        # gritty terrain -> SfM features (also try crater_boulders)
    --views 12 --radius-m 2.6 --height-m 1.1 \
    --arc-start-deg 200 --arc-deg 140 --sun-elev 24 \
    --out-dir out/colmap/views
```

> The default camera (sidecar `--pose` path) is `fov=55°` with Godot's default
> `keep_aspect=KEEP_HEIGHT`, i.e. 55° is the **vertical** FOV — so intrinsics key off image
> **height**: `fy=fx=(h/2)/tan(fov_v/2)`, `cx=w/2`, `cy=h/2`. (The front-stereo *rig*
> `camera_rig.gd` uses `KEEP_WIDTH`/horizontal-FOV instead; do not confuse the two.) This
> arc capture is **self-verification** — the real input is the M2-egress moving sequence
> (`out/cam/<scene>/<NNN>/`, contract §2.5), wired in Wave-2.

### 2. Reconstruct + recover poses (CONTAINER — `--gpus all`)

```bash
docker run --rm --gpus all \
    --user "$(id -u):$(id -g)" \          # so artifacts are host-owned, not root
    -e QT_QPA_PLATFORM=offscreen -e HOME=/tmp \
    -v "$PWD":/work -w /work \
    fossipex/colmap:m2b \
    python3 scripts/colmap/colmap_recon.py recon \
        --images out/colmap/views \
        --work   out/colmap/recon \
        --out    out/colmap/recon/colmap_trajectory.json
        # --dense   # also run image_undistorter -> patch_match_stereo -> stereo_fusion (fused.ply)
```

Pipeline: `feature_extractor` (exact `PINHOLE` intrinsics, `single_camera`, `use_gpu 1`)
→ `exhaustive_matcher` (`use_gpu 1`) → `mapper` (sparse SfM, intrinsics **not** refined) →
optional dense MVS → parse the sparse model → **Umeyama Sim3** align the recovered camera
centres to the ground-truth camera centres → emit the aligned `TrajectorySample` list.

### Output: `colmap_trajectory.json`

```jsonc
{
  "scene": "boulder_field",
  "n_input_views": 12, "n_registered": 12, "n_points3d": 3528,
  "mean_reproj_error_px": 0.296,
  "dense": false,
  "alignment": { "scale": 0.437, "n_aligned": 12,
                 "ate_rmse_m": 0.0057, "ate_max_m": 0.0136 },
  "trajectory_frame": "map",
  "trajectory": [ { "frame_index": 0, "t_s": 0.0,
                    "position_m": [..], "quaternion_xyzw": [..], "frame": "map" }, ... ]
}
```

`trajectory[]` items are `eval_schema.TrajectorySample` dicts (`frame='map'`) ready for the
lane-C scorer to consume as an **independent** estimate alongside rtabmap's `/slam/odom`.

## Frames

`sensors.json` poses are 100% **Godot-frame** (contract §3 — the REP-103 conversion is
`frames.py`'s job, **not** ours). We align COLMAP's arbitrary SfM gauge to the **Godot**
camera centres, so the emitted samples are in the same truth-relative gauge as the ground
truth; we label `frame='map'` to match the `TrajectorySample` channel. We do **not** apply
REP-103 here. Lane C runs both estimates and the truth through the same conversion, so a
consistent-but-unconverted gauge here is correct (the alignment is truth-relative).

## Verified result (this lane's self-check, 2026-05-30)

12-view arc of `samples/boulder_field`, RTX 4090, `--gpus all`:
SIFT **GPU** extractor + matcher (CUDA, not CPU fallback) · **12/12** images registered ·
**3528** 3D points · **mean reprojection error 0.296 px** · Umeyama Sim3 scale 0.437,
**ATE rmse 5.7 mm / max 13.6 mm** · `--dense` MVS fuses a **143 k-point** cloud.

The clean 12/12 registration here reflects the *favourable* arc capture (moderate sun,
high overlap, boulders for parallax). Be honest in the writeup: at a **grazing** sun
(elev ≤ 7°) the boulder shadows move with the viewpoint and self-shadowed regolith goes
featureless, so partial reconstruction (dropped images, a split model) is expected — that
is an **on-narrative finding** about regolith terramechanics imagery, not a tool failure.
Full validation against the moving M2-egress sequence is Wave-2.

## Licensing

This lane's source is **CC0-1.0** (repo `LICENSE`). The Docker base image carries COLMAP's
own **BSD-3-Clause** licence and the bundled CUDA runtime's NVIDIA EULA — those govern the
*image*, not this repo's source. See `THIRD_PARTY.md`.
