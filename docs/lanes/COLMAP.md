# Lane: COLMAP offline SfM/MVS (M2b)

**Branch** `lane/colmap` (off L0 `e2b0994`). **Owned files** (edit only these):
`scripts/colmap/colmap.Dockerfile`, `scripts/colmap/colmap_recon.py`,
`scripts/colmap/README.md`, `docs/lanes/COLMAP.md` (this file).

## What this lane is

The offline **best-achievable reconstruction + recovered poses** benchmark that complements
the online rtabmap SLAM lane. COLMAP sees all images at once (global BA) → its trajectory is
an *upper bound* on feature-based pose recovery on this imagery. Lane C scores it as an
**independent** `eval_schema.TrajectorySample` estimate next to rtabmap's `/slam/odom`,
separating "hard" error (regolith lacks matchable structure) from "online/causal" cost.

Key advantage exploited: the Godot renderer **knows intrinsics exactly**, so COLMAP gets a
fixed `PINHOLE` camera (`fx,fy,cx,cy` from `sensors.json`) with BA intrinsic-refinement OFF —
only poses + structure are solved.

## Run recipe (reproduce this lane's self-verification)

```bash
# 0. build the image (base graffitytech/colmap already pulled -> fast)
docker build -f scripts/colmap/colmap.Dockerfile -t fossipex/colmap:m2b scripts/colmap/

# 1. HOST: render a 12-view overlapping arc of a gritty scene + sensors.json
#    (uses the symlinked Godot in .tools via godot_sidecar/render_layers.sh --pose)
python3 scripts/colmap/colmap_recon.py render-arc \
    --scene boulder_field --views 12 --radius-m 2.6 --height-m 1.1 \
    --arc-start-deg 200 --arc-deg 140 --sun-elev 24 \
    --out-dir out/colmap/views

# 2. CONTAINER (--gpus all): COLMAP SfM (+ optional --dense MVS) + Umeyama align
docker run --rm --gpus all --user "$(id -u):$(id -g)" \
    -e QT_QPA_PLATFORM=offscreen -e HOME=/tmp \
    -v "$PWD":/work -w /work \
    fossipex/colmap:m2b \
    python3 scripts/colmap/colmap_recon.py recon \
        --images out/colmap/views --work out/colmap/recon \
        --out out/colmap/recon/colmap_trajectory.json   # [--dense]
```

`recon` runs `feature_extractor` (exact PINHOLE intrinsics, `single_camera`, `use_gpu 1`) →
`exhaustive_matcher` (`use_gpu 1`) → `mapper` (intrinsics NOT refined) → [optional
`image_undistorter`→`patch_match_stereo`→`stereo_fusion`] → parse sparse model →
Umeyama Sim3 align recovered camera centres → emit `TrajectorySample[]` (`frame='map'`).

## Verified (2026-05-30, RTX 4090, CUDA 12.8 driver)

- **GPU, not CPU fallback**: logs show `Creating SIFT GPU feature extractor` +
  `Creating SIFT GPU feature matcher`; base image's `nvidia-smi` sees the 4090 under `--gpus all`.
- **Sparse model**: 12/12 images registered · 3528 points · 10488 observations ·
  **mean reprojection error 0.296 px**.
- **Aligned trajectory**: Umeyama Sim3 scale 0.437 over 12 cams · **ATE rmse 5.7 mm /
  max 13.6 mm** · 12 `TrajectorySample`s emitted.
- **`--dense`** branch: image_undistorter → patch_match_stereo (geometric, CUDA) →
  stereo_fusion → **143 k-point** `fused.ply`.

### Honest finding (on-narrative, not a failure)

This clean 12/12 reflects a *favourable* capture: moderate sun (elev 24°), high overlap,
boulders for parallax. At a **grazing** sun (elev ≤ 7°, the lunar perception hazard) boulder
shadows move with the viewpoint and self-shadowed regolith goes featureless → expect dropped
images / a split sparse model. `recon` handles this gracefully: it picks the largest sub-model
and, if fewer than 3 views align to truth, emits **no** trajectory rather than a bogus Sim3.

## Frames

All `sensors.json` poses are Godot-frame; the REP-103 conversion stays **frozen in
`frames.py`** (L0 seam) — this lane does NOT convert. Alignment is truth-relative (COLMAP gauge
→ Godot camera centres), and samples are labelled `frame='map'` to match the scorer channel.

## Scope / next

This lane delivers the offline benchmark + a self-verification arc capture. **Wave-2**: run
`recon` against the moving **M2-egress** sequence (`out/cam/<scene>/<NNN>/`, contract §2.5) —
the per-frame `sensors.json` already carries the real monotonic `frame_index` + per-frame
camera `pose_in_world`, which `recon` consumes unchanged (it reads `cameras[].image` +
`pose_in_world` per view; a moving sequence is just N single-camera frames to it).

## Integration seams for the orchestrator

- **Frozen-file imports only**: `colmap_recon.py` imports `eval_schema.TrajectorySample`
  (read-only) by adding `scripts/ros2_bridge/` to `sys.path`. No frozen file edited.
- **Root `out/` not git-ignored**: scratch lands in repo-root `out/colmap/` (NOT
  `godot_sidecar/out/`, which `.gitignore` already covers). This lane's commit `git add`s
  ONLY its owned files, so nothing under `out/` is committed — but the orchestrator may want
  a root-level `out/` entry in `.gitignore` (a frozen-ish file, hence flagged not edited here).
- **Lane C handoff**: feed `colmap_trajectory.json["trajectory"]` (a `TrajectorySample[]`)
  into the lane-C scorer as the COLMAP estimate; report it side-by-side with rtabmap,
  never summed (eval_schema docstring: two independent channels).
