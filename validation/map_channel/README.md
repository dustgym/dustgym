# Section 10 map channel: onboard rover-stereo observed map vs conserved truth

Validation artifacts for the LAC-style map channel. The **producer**
(`scripts/ros2_bridge/obs_map_producer.py`) feeds the existing **scorer**
(`scripts/ros2_bridge/score_map.py`): it rectifies the rover front-stereo pair with the exact known
camera extrinsics, runs SGBM, back-projects to the authority world frame, and grids the points to an
observed heightfield, which `score_map` compares against the conserved ground-truth terrain.

All figures are from a REAL Godot render of `crater_boulders` (an 8-station drive) plus the conserved
authority truth. No synthetic data.

## Figures

- **observed_vs_truth.png** - truth heightfield, the accumulated observed map (16.4 percent coverage),
  and the per-cell error (RMSE 0.32 m).
- **levers.png** - the two RMSE levers. Lever 1, the near-field depth cap, trades accuracy against
  coverage (0.12 m RMSE at a 1 m cap, 0.32 m at 4 m, because stereo error grows as Z squared over
  f times baseline). Lever 2, coverage grows from 2.6 to 16.4 percent as the rover drives and the
  stations accumulate (map-by-driving).
- **vslam_features.png** - ORB feature density on a real rover camera. The render is feature-rich;
  the features that land in shadow are illumination-locked (they move as the sun sweeps).

## Honest finding

Passive rover stereo at the rover's ~0.15 m grazing eye-height has about 0.3 m (1 sigma) height
precision. The rover-scale sample scenes have only ~0.05 m relief, below that floor, so the producer
recovers the ground plane and the coverage (which grows with driving) but not the centimetre
micro-relief that governs trafficability. That floor is a real perception limit, not a defect: it
motivates active sensing and the ground COLMAP reference (`scripts/colmap/`), which this simulator
uniquely grades against ground truth.

The two perception tiers are complementary: onboard rover stereo is cheap and real-time-plausible
but noisy; ground COLMAP is offline and high-accuracy. The simulator feeds and scores both against
the same conserved truth.

## Ground tier: COLMAP scored vs truth (2026-06-04)

`scripts/colmap/` runs the ground tier with pycolmap (no Docker): `render_corpus.py` renders a
known-pose multi-view corpus of the static scene, `colmap_map_channel.py` runs incremental SfM and
Umeyama-aligns the recovered camera centers to the known render poses (alignment RMSE 6 mm) to put
the sparse point cloud in the world frame, then `score_map` compares it to the conserved truth:
**18/18 images registered, 0.48 px reprojection, map RMSE 0.04 m, 97 percent cell-pass** (sparse SfM,
about 3 percent coverage; dense MVS would fill it). Compared to the onboard tier's 0.32 m, the ground
tier is roughly an order of magnitude more accurate, as expected.

- **colmap_hapke_vs_lambert.png** - the BRDF A/B (`make_colmap_ab.py`). The physically-correct Hapke
  BRDF gives COLMAP about 33 percent fewer 3-D points and 30 percent less coverage than the idealized
  Lambert baseline, at higher reprojection error. The non-Lambertian regolith reflectance costs
  multi-view correspondences, exactly as on real lunar imagery; only the simulator has the ground
  truth to quantify it. Regenerate with `make_colmap_ab.py --hapke <corpus> --lambert <corpus>`.
- **colmap_ab_metrics.json** - the per-BRDF metrics behind the figure.
- **colmap_height_sweep.png** + **.json** - the camera-height sweep (`make_height_sweep.py`). The ground
  tier collapses toward the rover's grazing eye-level: 18 of 18 images register at elevated and mid
  heights, 12 of 18 at 1.0 m, and only 2 of 18 at 0.5 m. Near-horizontal views of a near-flat surface
  share too few features; accuracy stays near 4 cm where it reconstructs, but registration and coverage
  fall off. This is the honest answer to "does ground COLMAP hold up on the rover's real grazing capture."

## Uncertainty layer (world model)

- **uncertainty_layer.png** - the per-cell height uncertainty and the dig-ready gate
  (`obs_map_producer.grid_to_heightfield_uncertainty` + `dig_ready_mask`). Each observed cell carries a
  height sigma (the standard error of the mean, which falls as more views accumulate; single-view cells
  get a 0.30 m prior), and the planner gates digging on it: green cells are confident enough to act on,
  red cells are observed but uncertain (observe more first), grey are unobserved. This is the world
  model's Uncertainty layer; see `docs/world_model.md` for the full five-layer mapping.

## Regenerate

```
python3 scripts/ros2_bridge/make_map_channel_figures.py \
    --drive <front-stereo egress dir> --scene samples/crater_boulders \
    --frame <a front_left.png> --out validation/map_channel
```

The figures need a Godot front-stereo drive egress (render output, not committed). The producer's
unit tests run without it (`test_obs_map_producer.py`: 5 pass on the real DEM round-trip plus the
geometry identities, 1 integration test skips when no egress is present).
