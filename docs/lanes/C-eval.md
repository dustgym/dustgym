# Lane C-eval — two-channel pose scorer (synthetic, report-only)

Scores estimated trajectories/poses against ground truth on **two independent
channels that are never summed**:

1. **Trajectory channel** → `eval_schema.Scorecard`: `pose_rmse_trans_mm`,
   yaw-only `pose_rmse_yaw_deg`, and a **Umeyama-aligned `ate_mm`** (TUM RGB-D /
   Sturm 2012 convention; coincides with raw RMSE on same-frame synthetic data).
2. **AprilTag single-pose channel** → mirrors `compare_pose.py`
   (`translation·1000` mm + geodesic `rotation_error_deg`), reported separately.

**Report-only: no pass/fail, no acceptance threshold.** The scorer emits numbers;
a verdict bar is a portfolio call left unset (see `project_slam_parallel_plan`).

## Run

```bash
# from repo root, host .venv (pure stdlib + numpy; zero ROS/container dep)
# regenerate the gitignored sample trajectories first for full-length runs:
./.venv/bin/python -m terrain_authority.scenes

./.venv/bin/python scripts/ros2_bridge/eval_harness.py --scene samples/tread_track_4wheel
./.venv/bin/python scripts/ros2_bridge/eval_harness.py --scene samples/tread_track --out card.json
# knobs: --sigma-trans-m, --sigma-yaw-rad (seeded rng(0)); --synthetic is the default
```

Trajectory **Scorecard JSON** goes to stdout / `--out`; the AprilTag block and the
**~20 mm `rover_rc` quantization floor** (`CELL_M=0.02`) are surfaced on stderr.

## Outputs / notes
- Reference Scorecards: `tread_track_4wheel` n=18 → 13.6 mm / 0.78° / 13.6 mm;
  `tread_track` n=31 → 14.3 / 0.89 / 14.1. Zero-noise → exactly 0.
- Structural tests: `scripts/ros2_bridge/test_score_pose.py` (4, no threshold gate).
- Frozen `eval_schema.py` / `compare_pose.py` are imported unmodified
  (`compare_pose` via a self-cleaning ROS-stub shim so it loads on the bare host).
- Live M2 path (nearest-`t_s` association) is a documented hook, not yet implemented.
