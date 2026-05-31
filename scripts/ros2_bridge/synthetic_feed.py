"""Synthetic estimate-stream generator for the pose/map scorer (Lane C, report-only).

Lifts GROUND TRUTH from a scene's per-frame metadata via the frozen
`eval_schema` contract (`load_scene_frames` -> `lift_trajectory`), then injects SEEDED
zero-mean Gaussian noise (plus an optional constant bias) to synthesise an *estimate*
`TrajectorySample` stream.  This is the dependency-free stand-in for the real M2 SLAM
egress: it lets `score_pose.py` + `eval_harness.py` run green on the bare host .venv
(pure stdlib + numpy) with ZERO ROS / container dependency, BEFORE M2 exists.

The truth lift itself (rover_rc -> metric, persisted-vs-delta heading, null-frame skip) is
the contract's job and lives entirely in `eval_schema`; this module ONLY perturbs the lifted
truth so a non-trivial Scorecard can be produced and reported.

UNITS: metres in (TrajectorySample.position_m), metres of noise injected here; the *scorers*
convert to mm for the Scorecard (mm out).  Noise sigmas/bias are therefore specified in metres
and radians on the CLI.

QUANTIZATION FLOOR (see eval_schema module docstring + terrain_authority/constants.py CELL_M):
the truth positions are lifted from integer `rover_rc` grid cells at cell_m == 0.02 m, so the
truth itself is quantized to ~20 mm.  Injecting sub-20 mm noise is fine for exercising the
pipeline, but the *reported* trans/ATE figures cannot be interpreted below that floor -- it is
surfaced (not asserted) by the harness, because this lane is REPORT-ONLY.

DETERMINISM: a fixed default seed (numpy default_rng(0)) makes every run byte-reproducible; the
default sigmas are small but NON-ZERO so the emitted Scorecard is non-degenerate.

CC0-1.0 (see ../../LICENSE).
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

import eval_schema as es

# Default seed: deterministic, byte-reproducible synthetic stream (NumPy Generator API,
# default_rng(0) -- the conventional reproducible-fixture seed used across this repo's tests).
DEFAULT_SEED = 0

# Small NON-ZERO default perturbations so a non-degenerate Scorecard is produced without a
# CLI flag.  Chosen at roughly half the rover_rc quantization cell (cell_m == 0.02 m == 20 mm;
# terrain_authority/constants.py CELL_M) so the synthetic estimate sits on the order of the
# data's own resolution floor -- a realistic, not-flattering, exercise of the metrics.
DEFAULT_SIGMA_TRANS_M = 0.010   # 10 mm per-axis translation sigma (~half a 20 mm cell)
DEFAULT_SIGMA_YAW_RAD = 0.0175  # ~1.0 deg yaw sigma


@dataclass
class NoiseConfig:
    """Seeded Gaussian-noise + constant-bias parameters for the synthetic estimate.

    All fields are in SI (metres / radians).  Translation noise is injected per map axis
    (x and z; y stays the ground plane at 0).  Yaw noise is injected on the +Z yaw angle and
    re-encoded to the XYZW quaternion via the contract's `yaw_to_quat_xyzw`.
    """

    sigma_trans_m: float = DEFAULT_SIGMA_TRANS_M
    sigma_yaw_rad: float = DEFAULT_SIGMA_YAW_RAD
    bias_x_m: float = 0.0
    bias_z_m: float = 0.0
    bias_yaw_rad: float = 0.0
    seed: int = DEFAULT_SEED


def scene_frame_paths(scene_dir: str) -> list[str]:
    """Ordered list of per-frame metadata.json paths under a scene dir (samples/<scene>/).

    Sorted by the tNNN frame-dir name so frame order is the temporal order.  Frames whose
    `rover_rc` is null (e.g. t000 pre-placement) are KEPT here and skipped later by the
    contract's `lift_trajectory`; we do not pre-filter so the lift owns the null rule.
    """
    pattern = os.path.join(scene_dir, "t*", "metadata.json")
    return sorted(glob.glob(pattern))


def load_truth(scene_dir: str) -> list[es.TrajectorySample]:
    """Lift the scene's ground-truth trajectory via the frozen eval_schema contract.

    No noise here -- this is pure truth (rover_rc -> metric, persisted-or-delta heading,
    null-frame skip), exactly as M2's live truth channel will be lifted.
    """
    paths = scene_frame_paths(scene_dir)
    if not paths:
        raise FileNotFoundError(
            f"no per-frame metadata.json under {scene_dir}/t*/ -- regenerate the scene "
            f"(python -m terrain_authority.scenes) or point --scene at a populated samples dir"
        )
    frames = es.load_scene_frames(paths)
    return es.lift_trajectory(frames)


def inject_noise(
    truth: Sequence[es.TrajectorySample], cfg: NoiseConfig
) -> list[es.TrajectorySample]:
    """Return an ESTIMATE stream = truth + seeded Gaussian noise (+ optional constant bias).

    * translation: independent zero-mean Gaussian (sigma_trans_m) on the x and z map axes,
      plus the constant (bias_x_m, bias_z_m) offset; y stays 0 (ground plane, no z-truth).
    * yaw: zero-mean Gaussian (sigma_yaw_rad) + bias_yaw_rad on the +Z yaw, re-encoded to a
      unit XYZW quaternion through the contract's `yaw_to_quat_xyzw`.

    frame_index / t_s are COPIED from truth unchanged so the scorer can match exactly on
    frame_index (the dependency-free synthetic association).
    """
    rng = np.random.default_rng(cfg.seed)
    out: list[es.TrajectorySample] = []
    for s in truth:
        tx, ty, tz = (float(c) for c in s.position_m)
        nx = tx + cfg.bias_x_m + float(rng.normal(0.0, cfg.sigma_trans_m))
        nz = tz + cfg.bias_z_m + float(rng.normal(0.0, cfg.sigma_trans_m))
        truth_yaw = es.quat_xyzw_to_yaw(s.quaternion_xyzw)
        est_yaw = truth_yaw + cfg.bias_yaw_rad + float(rng.normal(0.0, cfg.sigma_yaw_rad))
        out.append(
            es.TrajectorySample(
                frame_index=s.frame_index,
                t_s=s.t_s,
                position_m=[nx, ty, nz],
                quaternion_xyzw=es.yaw_to_quat_xyzw(est_yaw),
                frame=s.frame,
            )
        )
    return out


def synthesize(scene_dir: str, cfg: Optional[NoiseConfig] = None):
    """Convenience: load truth and produce (truth, estimate) sample lists for a scene dir."""
    cfg = cfg or NoiseConfig()
    truth = load_truth(scene_dir)
    estimate = inject_noise(truth, cfg)
    return truth, estimate


def _build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--scene", required=True,
        help="path to a scene dir under samples/ (e.g. samples/tread_track_4wheel)")
    ap.add_argument("--seed", type=int, default=DEFAULT_SEED,
                    help=f"numpy default_rng seed (default {DEFAULT_SEED}, reproducible)")
    ap.add_argument("--sigma-trans-m", type=float, default=DEFAULT_SIGMA_TRANS_M,
                    help=f"per-axis translation noise sigma in m (default {DEFAULT_SIGMA_TRANS_M})")
    ap.add_argument("--sigma-yaw-rad", type=float, default=DEFAULT_SIGMA_YAW_RAD,
                    help=f"yaw noise sigma in rad (default {DEFAULT_SIGMA_YAW_RAD})")
    ap.add_argument("--bias-x-m", type=float, default=0.0,
                    help="constant x-axis translation bias in m (default 0)")
    ap.add_argument("--bias-z-m", type=float, default=0.0,
                    help="constant z-axis translation bias in m (default 0)")
    ap.add_argument("--bias-yaw-rad", type=float, default=0.0,
                    help="constant yaw bias in rad (default 0)")
    ap.add_argument(
        "--emit", choices=["estimate", "truth", "both"], default="estimate",
        help="which stream to print as JSONL (default estimate)")
    return ap


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _build_arg_parser().parse_args(argv)
    cfg = NoiseConfig(
        sigma_trans_m=args.sigma_trans_m,
        sigma_yaw_rad=args.sigma_yaw_rad,
        bias_x_m=args.bias_x_m,
        bias_z_m=args.bias_z_m,
        bias_yaw_rad=args.bias_yaw_rad,
        seed=args.seed,
    )
    truth, estimate = synthesize(args.scene, cfg)
    if args.emit in ("truth", "both"):
        for s in truth:
            print(json.dumps({"stream": "truth", **s.to_dict()}))
    if args.emit in ("estimate", "both"):
        for s in estimate:
            print(json.dumps({"stream": "estimate", **s.to_dict()}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
