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

## Regenerate

```
python3 scripts/ros2_bridge/make_map_channel_figures.py \
    --drive <front-stereo egress dir> --scene samples/crater_boulders \
    --frame <a front_left.png> --out validation/map_channel
```

The figures need a Godot front-stereo drive egress (render output, not committed). The producer's
unit tests run without it (`test_obs_map_producer.py`: 5 pass on the real DEM round-trip plus the
geometry identities, 1 integration test skips when no egress is present).
