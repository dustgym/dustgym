"""CLI orchestrator for the Lane-C pose/map scorer -- REPORT-ONLY (no CI gate, no pass/fail).

Wires the synthetic feed -> the two independent scorers and emits a side-by-side report:
  * the TRAJECTORY-channel `Scorecard` (pose_rmse_trans_mm / pose_rmse_yaw_deg / ate_mm /
    n_frames), the canonical JSON object, and
  * the standalone APRILTAG single-pose metrics (apriltag_trans_err_mm / apriltag_rot_err_deg),
    reported ALONGSIDE the Scorecard, NEVER summed into it.

`--synthetic` is the DEFAULT (and currently the only) mode: it generates a seeded estimate from
a scene's lifted ground truth and scores it on the bare host .venv with ZERO ROS / container
dependency.  The live M2 mode is intentionally unimplemented (a documented seam in score_pose).

This harness reports numbers; it does NOT emit pass/fail and does NOT assert any acceptance
threshold.  None exist in the repo, and inventing one would be portfolio-fraudulent.

QUANTIZATION FLOOR is surfaced (not asserted): truth is lifted from integer rover_rc cells at
cell_m == 0.02 m (terrain_authority/constants.py CELL_M), so the reported trans/ATE figures
cannot resolve below ~20 mm.  The report carries that floor so the reader interprets the
numbers correctly.

Output: the trajectory Scorecard JSON to stdout (and/or --out file); the apriltag block and the
floor note go to stderr (so stdout stays a clean, machine-parseable Scorecard).

CC0-1.0 (see ../../LICENSE).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Optional, Sequence

import numpy as np

import eval_schema as es
import score_map as sm
import score_pose
import synthetic_feed as sf


def run_synthetic(scene_dir: str, cfg: sf.NoiseConfig) -> dict:
    """Generate the seeded synthetic estimate for `scene_dir` and score the trajectory channel.

    Returns a report dict carrying the trajectory Scorecard plus side-by-side context
    (scene, association mode, the surfaced quantization floor).  No apriltag pair is available
    on a pure trajectory scene, so the apriltag block is reported as null context here and is
    exercised separately by `run_apriltag` / the structural test.
    """
    truth, estimate = sf.synthesize(scene_dir, cfg)
    scorecard = score_pose.score_trajectory(truth, estimate)
    return {
        "scene": scene_dir,
        "mode": "synthetic",
        "association": "frame_index (exact; --synthetic)",
        "report_only": True,  # this lane COMPUTES + REPORTS; it asserts no acceptance bar
        "quantization_floor_mm": score_pose.QUANTIZATION_FLOOR_MM,
        "trajectory": scorecard.to_dict(),
        "n_truth_samples": len(truth),
    }


def _load_heightfield(path: str) -> np.ndarray:
    """Load a heightfield [m] from a .npy file or a scene bundle dir (metadata.json + heightmap.rf32).

    This is the REAL-data ingress for the map channel: the observed map comes from a producer or a real
    DEM file you supply -- nothing is synthesised here.
    """
    if os.path.isdir(path):
        with open(os.path.join(path, "metadata.json"), "r", encoding="utf-8") as fh:
            g = json.load(fh)["grid"]
        z = np.fromfile(os.path.join(path, "heightmap.rf32"), dtype="<f4")
        return z.reshape(int(g["height"]), int(g["width"])).astype(np.float64)
    if path.endswith(".npy"):
        return np.load(path).astype(np.float64)
    raise ValueError(f"unrecognised heightfield path (want a .npy file or a bundle dir): {path}")


def run_map(
    truth, observed, *, tol_m: float = 0.10, valid_mask=None,
    truth_rocks=None, observed_rocks=None,
) -> dict:
    """Score the MAP channel for a SUPPLIED observed-vs-truth heightfield pair (the producer-independent
    entry point: you bring a real observed map -- from a producer or a real DEM -- and this harness
    fabricates none).  Report-only.  The returned Scorecard zeroes the pose channel (a different channel)
    and carries the non-null map metrics; the two are never summed.
    """
    metrics = sm.score_map(observed, truth, tol_m=tol_m, valid_mask=valid_mask,
                           truth_rocks=truth_rocks, observed_rocks=observed_rocks)
    sc = es.Scorecard(pose_rmse_trans_mm=0.0, pose_rmse_yaw_deg=0.0, ate_mm=0.0, n_frames=0,
                      map_rmse_m=metrics["map_rmse_m"], map_cell_pass_frac=metrics["map_cell_pass_frac"],
                      rock_f1=metrics["rock_f1"])
    return {"mode": "map", "report_only": True, "tol_m": float(tol_m),
            "shape": list(np.asarray(truth).shape), "scorecard": sc.to_dict()}


def _print_map_report(report: dict, out_path: Optional[str]) -> None:
    """Emit the map-channel Scorecard JSON to stdout (+ --out); context to stderr (mirrors pose report)."""
    sc_json = json.dumps(report["scorecard"], indent=2)
    print(sc_json)
    if out_path:
        with open(out_path, "w", encoding="utf-8") as fh:
            fh.write(sc_json + "\n")
    sc = report["scorecard"]
    print("", file=sys.stderr)
    print("=== Lane-C MAP scorer (REPORT-ONLY; no pass/fail, no threshold) ===", file=sys.stderr)
    print(f"  shape       : {report['shape']}  | tol_m: {report['tol_m']}", file=sys.stderr)
    print("  --- map channel (observed vs truth-at-t; never summed with pose) ---", file=sys.stderr)
    print(f"    map_rmse_m         : {sc['map_rmse_m']}", file=sys.stderr)
    print(f"    map_cell_pass_frac : {sc['map_cell_pass_frac']}", file=sys.stderr)
    print(f"    rock_f1            : {sc['rock_f1']}", file=sys.stderr)
    print("  NOTE: this scores a SUPPLIED observed map; the live producer (stereo-depth/SLAM -> "
          "heightfield) needs the Godot/sensor track and is not in this repo.", file=sys.stderr)
    print("==================================================================", file=sys.stderr)


def _print_report(report: dict, out_path: Optional[str]) -> None:
    """Emit the canonical trajectory Scorecard JSON to stdout (+ --out); context to stderr."""
    scorecard_json = json.dumps(report["trajectory"], indent=2)
    print(scorecard_json)
    if out_path:
        with open(out_path, "w", encoding="utf-8") as fh:
            fh.write(scorecard_json + "\n")

    # Side-by-side context + the surfaced (not asserted) quantization floor -> stderr.
    print("", file=sys.stderr)
    print("=== Lane-C pose scorer (REPORT-ONLY; no pass/fail, no threshold) ===",
          file=sys.stderr)
    print(f"  scene       : {report['scene']}", file=sys.stderr)
    print(f"  mode        : {report['mode']}  | association: {report['association']}",
          file=sys.stderr)
    print(f"  n_frames    : {report['trajectory']['n_frames']} "
          f"(of {report['n_truth_samples']} non-null truth samples)", file=sys.stderr)
    print("  --- trajectory channel (rover_rc; trans + yaw only) ---", file=sys.stderr)
    print(f"    pose_rmse_trans_mm : {report['trajectory']['pose_rmse_trans_mm']:.3f}",
          file=sys.stderr)
    print(f"    pose_rmse_yaw_deg  : {report['trajectory']['pose_rmse_yaw_deg']:.3f}",
          file=sys.stderr)
    print(f"    ate_mm (Umeyama)   : {report['trajectory']['ate_mm']:.3f}", file=sys.stderr)
    print("  --- apriltag single-pose channel (separate; never summed) ---", file=sys.stderr)
    if report.get("apriltag") is not None:
        print(f"    apriltag_trans_err_mm : {report['apriltag']['apriltag_trans_err_mm']:.3f}",
              file=sys.stderr)
        print(f"    apriltag_rot_err_deg  : {report['apriltag']['apriltag_rot_err_deg']:.3f}",
              file=sys.stderr)
    else:
        print("    (no camera->tag pair on this trajectory scene; see compare_pose.py / "
              "the structural test for the apriltag channel)", file=sys.stderr)
    print(f"  NOTE: synthetic resolution floor ~{report['quantization_floor_mm']:.1f} mm "
          f"(rover_rc integer cells @ cell_m=0.02 m); trans/ATE below this are not resolvable.",
          file=sys.stderr)
    print("====================================================================",
          file=sys.stderr)


def _build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--synthetic", action="store_true", default=True,
        help="synthetic mode (DEFAULT and currently only mode): seeded estimate vs lifted truth")
    ap.add_argument(
        "--scene", default=None,
        help="scene dir under samples/ (e.g. samples/tread_track_4wheel) -- pose channel")
    ap.add_argument(
        "--map-truth", default=None,
        help="MAP channel: true-terrain heightfield (.npy or a bundle dir); pair with --map-observed")
    ap.add_argument(
        "--map-observed", default=None,
        help="MAP channel: observed/reconstructed heightfield (.npy or a bundle dir)")
    ap.add_argument("--map-tol-m", type=float, default=0.10,
                    help="MAP channel cell-pass tolerance in m (default 0.10)")
    ap.add_argument("--out", default=None,
                    help="optional path to also write the trajectory Scorecard JSON")
    ap.add_argument("--seed", type=int, default=sf.DEFAULT_SEED,
                    help=f"numpy default_rng seed (default {sf.DEFAULT_SEED}, reproducible)")
    ap.add_argument("--sigma-trans-m", type=float, default=sf.DEFAULT_SIGMA_TRANS_M,
                    help=f"per-axis translation noise sigma in m (default {sf.DEFAULT_SIGMA_TRANS_M})")
    ap.add_argument("--sigma-yaw-rad", type=float, default=sf.DEFAULT_SIGMA_YAW_RAD,
                    help=f"yaw noise sigma in rad (default {sf.DEFAULT_SIGMA_YAW_RAD})")
    ap.add_argument("--bias-x-m", type=float, default=0.0, help="constant x bias in m")
    ap.add_argument("--bias-z-m", type=float, default=0.0, help="constant z bias in m")
    ap.add_argument("--bias-yaw-rad", type=float, default=0.0, help="constant yaw bias in rad")
    return ap


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _build_arg_parser().parse_args(argv)
    if args.map_truth or args.map_observed:                  # MAP channel: score a supplied DEM pair
        if not (args.map_truth and args.map_observed):
            print("error: --map-truth and --map-observed must be given together", file=sys.stderr)
            return 2
        report = run_map(_load_heightfield(args.map_truth), _load_heightfield(args.map_observed),
                         tol_m=args.map_tol_m)
        _print_map_report(report, args.out)
        return 0
    if not args.scene:
        print("error: --scene is required for the pose channel (or use --map-truth/--map-observed)",
              file=sys.stderr)
        return 2
    cfg = sf.NoiseConfig(
        sigma_trans_m=args.sigma_trans_m,
        sigma_yaw_rad=args.sigma_yaw_rad,
        bias_x_m=args.bias_x_m,
        bias_z_m=args.bias_z_m,
        bias_yaw_rad=args.bias_yaw_rad,
        seed=args.seed,
    )
    report = run_synthetic(args.scene, cfg)
    _print_report(report, args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
