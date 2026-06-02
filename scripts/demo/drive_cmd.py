#!/usr/bin/env python3
"""drive_cmd.py — closed-loop, twist-driven rover demo (Phase 3, 2026-06-01).

Drives the Tier-2 rover by cmd_vel TWIST commands instead of a precomputed path,
so a ROS node / Nav2 / RL policy can steer it and slip closes the loop. Two modes:

  scripted:  --twists twists.json     # [[v, omega], ...]  deterministic run
  live:      --cmd-vel cmd_vel.json --steps N
             poll the twist file each step (a controller rewrites it between steps)

Writes per-frame pose sidecars + telemetry.json + the final scene fields to --out.
Leaves the open-loop drive_spiral.py untouched. Run from the repo root:
    python scripts/demo/drive_cmd.py --twists t.json --out out/drive_cmd
    python scripts/demo/drive_cmd.py --slope-deg 55 --twists t.json   # watch it stall
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import numpy as np  # noqa: E402

from terrain_authority import drive, io_fields  # noqa: E402
from terrain_authority.column_state import ColumnState  # noqa: E402


def build_scene(grid: int, cell_m: float, slope_deg: float) -> ColumnState:
    cs = ColumnState(width=grid, height=grid, cell_m=cell_m)
    if slope_deg:
        cols = np.arange(grid)[None, :].repeat(grid, axis=0).astype(np.float64)
        cs.datum = math.tan(math.radians(slope_deg)) * cols * cell_m  # ramp along +col
    return cs


def _write_pose(out_dir: str, frame: int, telem: dict) -> None:
    with open(os.path.join(out_dir, f"pose_{frame:04d}.json"), "w") as fh:
        json.dump(telem, fh, indent=2)


def main() -> None:
    ap = argparse.ArgumentParser(description="closed-loop twist-driven rover demo")
    ap.add_argument("--grid", type=int, default=96)
    ap.add_argument("--cell-m", type=float, default=0.02)
    ap.add_argument("--slope-deg", type=float, default=0.0)
    ap.add_argument("--dt", type=float, default=0.1)
    ap.add_argument("--start", type=float, nargs=2, default=None, metavar=("ROW", "COL"))
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--payload-kg", type=float, default=0.0)
    ap.add_argument("--twists", help="JSON file [[v,omega], ...] (scripted mode)")
    ap.add_argument("--cmd-vel", help="JSON twist file polled each step (live mode)")
    ap.add_argument("--steps", type=int, default=40, help="steps in live mode")
    ap.add_argument("--out", default="out/drive_cmd")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    cs = build_scene(args.grid, args.cell_m, args.slope_deg)
    start = tuple(args.start) if args.start else (args.grid / 2.0, args.grid / 2.0)

    if args.twists:
        with open(args.twists) as fh:
            twists = [tuple(t) for t in json.load(fh)]
        res = drive.closed_loop_drive(cs, start, args.yaw, twists, dt=args.dt,
                                      payload_kg=args.payload_kg)
        for telem in res["steps"]:
            _write_pose(args.out, telem["frame"], telem)
    elif args.cmd_vel:
        rc, yaw = start, args.yaw
        steps = []
        for i in range(args.steps):
            v, omega = drive.poll_cmd_vel(args.cmd_vel)   # a controller rewrites this between steps
            rc, yaw, telem = drive.drive_step(cs, rc, yaw, v, omega, dt=args.dt,
                                              payload_kg=args.payload_kg)
            telem["frame"] = i
            _write_pose(args.out, i, telem)
            steps.append(telem)
        res = {
            "steps": steps, "final_rc": [rc[0], rc[1]], "final_yaw": yaw,
            "commanded_dist_m": sum(abs(s["v_cmd"]) * args.dt for s in steps),
            "achieved_dist_m": sum(abs(s["v_achieved"]) * args.dt for s in steps),
            "any_entrapped": any(s["entrapped"] for s in steps),
        }
    else:
        ap.error("provide --twists FILE (scripted) or --cmd-vel FILE (live)")

    with open(os.path.join(args.out, "telemetry.json"), "w") as fh:
        json.dump(res, fh, indent=2)
    meta = {
        "schema_version": "1.0",
        "scene_name": "drive_cmd",
        "producer": "terrain_authority drive_cmd (closed loop, slip feedback)",
        "grid": {"width": args.grid, "height": args.grid, "cell_m": args.cell_m,
                 "order": "row-major-C"},
        "gravity_m_s2": 1.62,
        "final_pose": {"rc": res["final_rc"], "yaw": res["final_yaw"]},
    }
    io_fields.save_scene(os.path.join(args.out, "final_scene"), cs.fields_dict(), meta)
    print(f"drive_cmd: {len(res['steps'])} steps -> {args.out}  "
          f"commanded={res['commanded_dist_m']:.3f} m  achieved={res['achieved_dist_m']:.3f} m  "
          f"entrapped={res['any_entrapped']}")


if __name__ == "__main__":
    main()
