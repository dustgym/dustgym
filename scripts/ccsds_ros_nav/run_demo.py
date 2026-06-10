"""End-to-end nav demo (no ROS): ground station <-> CCSDS loopback link <-> rover executive.

Wires the real pieces together in one process so the whole loop is verifiable in the .venv (and CI):
the ground station plans a slope-aware route over a crop of the real LOLA Haworth DEM, commands it
waypoint-by-waypoint over real CCSDS Space Packets, and the onboard executive drives the conserved
terramechanics authority while streaming back pose/slip/sinkage telemetry. The container build reuses
these exact objects over a UDP link + rclpy nodes.

    python scripts/ccsds_ros_nav/run_demo.py            # default Haworth-rim-crest traverse (Moon)
    python scripts/ccsds_ros_nav/run_demo.py --quick    # smaller window / shorter route (fast)
"""
from __future__ import annotations

import argparse
import os
import threading

from flight import FlightModel, load_crop
from ground_station import GroundStation
from link import loopback_pair
from route import plan_route, snap_to_navigable, slope_deg

# The navigable Haworth-rim-crest plateau found by scanning the 2000x2000 DEM for the flattest window
# (mean slope ~5.8 deg, 81% < 8 deg; see CONTRACT.md / the build log).
DEFAULT_SCENE = "samples/lunar_dem/haworth_10km_5m"
DEFAULT_R0, DEFAULT_C0 = 720, 1800


def _repo_root() -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def main() -> int:
    ap = argparse.ArgumentParser(description="CCSDS/ROS-style rover nav demo on the Haworth DEM")
    ap.add_argument("--scene", default=DEFAULT_SCENE)
    ap.add_argument("--r0", type=int, default=DEFAULT_R0)
    ap.add_argument("--c0", type=int, default=DEFAULT_C0)
    ap.add_argument("--win", type=int, default=160, help="crop size in cells (square)")
    ap.add_argument("--body", default="moon")
    ap.add_argument("--start", default="40,40", help="crop-local start row,col")
    ap.add_argument("--goal", default="120,120", help="crop-local goal row,col")
    ap.add_argument("--v-max", type=float, default=0.3)
    ap.add_argument("--dt", type=float, default=0.2)
    ap.add_argument("--max-slope-deg", type=float, default=18.0)
    ap.add_argument("--waypoints", type=int, default=6)
    ap.add_argument("--out", default="out/ccsds_nav")
    ap.add_argument("--quick", action="store_true", help="small window + short route for a fast smoke run")
    args = ap.parse_args()

    if args.quick:
        args.win, args.start, args.goal, args.waypoints = 80, "20,20", "60,60", 4

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
        print(f"[demo] no navigable route from {start} to {goal} (try a different crop/slope cap)")
        return 1

    print(f"[demo] body={args.body} crop {args.win}x{args.win} @ {crop.cell_m:.0f} m/cell "
          f"(r0={crop.r0},c0={crop.c0}); start {start} -> goal {goal}")
    print(f"[demo] route: {len(waypoints)} waypoints {[(round(r),round(c)) for r,c in waypoints]}")

    flight = FlightModel(crop=crop, start_rc=(float(start[0]), float(start[1])),
                         body=args.body, dt=args.dt, v_max_default=args.v_max)
    ground_link, flight_link = loopback_pair()
    server = threading.Thread(target=flight.serve, args=(flight_link,),
                              kwargs={"expect_legs": len(waypoints)}, daemon=True)
    server.start()

    ground = GroundStation(ground_link, crop, out_dir=args.out, v_max=args.v_max,
                           goal_radius_cells=1.0)
    summary = ground.run_mission(waypoints)
    server.join(timeout=10.0)

    print("\n[demo] === mission summary ===")
    for k, v in summary.items():
        print(f"  {k:18} {v}")
    drift = flight.mass_drift()
    print(f"  mass_drift_rel     {drift:.2e}  ({'CONSERVED' if drift < 1e-9 else 'CHECK'})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
