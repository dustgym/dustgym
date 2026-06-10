"""UDP ground-station entry point (container / cross-host).

Runs the real GroundStation over a UdpLink against the in-container CCSDS bridge: plans a slope-aware
route over the Haworth crop, commands it waypoint-by-waypoint as CCSDS Space Packets, receives the
telemetry, and writes the trajectory artifact. Start/goal default to the rover executive's defaults so
the planned route is consistent with where the rover boots.

    python ground_station_main.py --bridge-host 127.0.0.1
"""
from __future__ import annotations

import argparse
import os
import time

from flight import load_crop
from ground_station import GroundStation
from link import UdpLink
from route import plan_route, slope_deg, snap_to_navigable


def _repo_root() -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def main() -> int:
    ap = argparse.ArgumentParser(description="UDP CCSDS ground station for the rover nav stack")
    ap.add_argument("--scene", default="samples/lunar_dem/haworth_10km_5m")
    ap.add_argument("--r0", type=int, default=720)
    ap.add_argument("--c0", type=int, default=1800)
    ap.add_argument("--win", type=int, default=160)
    ap.add_argument("--start", default="47,56")
    ap.add_argument("--goal", default="120,120")
    ap.add_argument("--max-slope-deg", type=float, default=18.0)
    ap.add_argument("--waypoints", type=int, default=6)
    ap.add_argument("--v-max", type=float, default=0.3)
    ap.add_argument("--local-port", type=int, default=52000)
    ap.add_argument("--bridge-host", default="127.0.0.1")
    ap.add_argument("--bridge-port", type=int, default=52001)
    ap.add_argument("--light-time-s", type=float, default=1.28)
    ap.add_argument("--out", default="out/ccsds_nav")
    args = ap.parse_args()

    scene = args.scene if os.path.isabs(args.scene) else os.path.join(_repo_root(), args.scene)
    crop = load_crop(scene, args.r0, args.c0, args.win, args.win)
    sl = slope_deg(crop.heightmap, crop.cell_m)
    sr, sc = (int(x) for x in args.start.split(","))
    gr, gc = (int(x) for x in args.goal.split(","))
    start = snap_to_navigable(sl, (sr, sc), args.max_slope_deg)
    goal = snap_to_navigable(sl, (gr, gc), args.max_slope_deg)
    waypoints = plan_route(crop.heightmap, crop.cell_m, start, goal,
                           max_slope_deg=args.max_slope_deg, n_waypoints=args.waypoints)
    if not waypoints:
        print("[ground] no navigable route; aborting")
        return 1

    link = UdpLink(("0.0.0.0", args.local_port),
                   (args.bridge_host, args.bridge_port), light_time_s=args.light_time_s)
    # generous per-leg timeout: round-trip light time + a leg's worth of execution
    ground = GroundStation(link, crop, out_dir=args.out, v_max=args.v_max, goal_radius_cells=1.0,
                           recv_timeout=2 * args.light_time_s + 120.0)
    print(f"[ground] UDP {args.local_port} <-> bridge {args.bridge_host}:{args.bridge_port}  "
          f"light_time={args.light_time_s}s  route {len(waypoints)} wps")
    time.sleep(1.0)  # let the bridge/executive come up
    summary = ground.run_mission(waypoints)
    link.close()
    print("[ground] summary:", summary)
    return 0 if summary["legs_reached"] > 0 else 2   # surface a broken stack as a non-zero exit


if __name__ == "__main__":
    raise SystemExit(main())
