"""Ground side: a minimal-but-real mission-control commander (move-and-wait).

Not a stub — it speaks the real CCSDS link: for each waypoint it sends a GoTo telecommand, then blocks
until that leg's TLM_LEG summary returns, collecting the TLM_POSE samples in between (the move-and-wait
cadence the Earth-Moon light-time budget forces). At the end it writes the demonstration artifacts:
a trajectory over the real Haworth hillshade, the slip/sinkage/SOC time series, and a telemetry log.
"""
from __future__ import annotations

import csv
import json
import os
import time

import numpy as np

import messages
from flight import CropContext


def waypoints_from_plan_ir(ir: dict, crop: CropContext) -> list[tuple[float, float]]:
    """Convert a planet_browser Plan IR's GoTo actions (world metres) to crop-local grid (row, col).

    GoTo ``to`` is [x, y] in the plan's local frame; we map via the crop world bounds:
    col = (x - world_x0)/cell_m, row = (world_y0 - y)/cell_m  (row increases as world Y decreases).
    """
    cm = crop.cell_m
    out: list[tuple[float, float]] = []
    for a in ir.get("actions", []):
        if a.get("op") == "GoTo":
            x, y = a["to"]
            out.append(((crop.world_y0 - y) / cm, (x - crop.world_x0) / cm))
    return out


class GroundStation:
    """Sends waypoint telecommands over a CCSDS link and logs the returned telemetry."""

    def __init__(self, link, crop: CropContext, *, out_dir: str = "out/ccsds_nav",
                 v_max: float = 0.3, goal_radius_cells: float = 1.0, recv_timeout: float = 30.0) -> None:
        self.link = link
        self.crop = crop
        self.out_dir = out_dir
        self.v_max = v_max
        self.goal_radius_cells = goal_radius_cells
        self.recv_timeout = recv_timeout
        self._t0 = time.monotonic()
        self.uplink_count = 0
        self.downlink_count = 0

    def _met(self) -> float:
        return time.monotonic() - self._t0

    def _run_leg(self, leg_id: int, wp: tuple[float, float]) -> "tuple[messages.Leg | None, list[messages.Pose]]":
        cmd = messages.GoTo(leg_id=leg_id, goal_row=float(wp[0]), goal_col=float(wp[1]),
                            v_max_mps=self.v_max, goal_radius_cells=self.goal_radius_cells)
        self.link.send(messages.encode(cmd, seq_count=leg_id, met=self._met()))
        self.uplink_count += 1
        poses: list[messages.Pose] = []
        while True:
            pkt = self.link.recv(timeout=self.recv_timeout)
            if pkt is None:
                return None, poses                          # link silent -> abort leg
            self.downlink_count += 1
            msg = messages.decode(pkt)
            if isinstance(msg, messages.Pose):
                poses.append(msg)
            elif isinstance(msg, messages.Leg):
                return msg, poses

    def run_mission(self, waypoints: list[tuple[float, float]]) -> dict:
        """Drive the rover through ``waypoints`` (grid row,col), one move-and-wait leg each."""
        legs: list[messages.Leg] = []
        all_poses: list[messages.Pose] = []
        for leg_id, wp in enumerate(waypoints):
            leg, poses = self._run_leg(leg_id, wp)
            all_poses.extend(poses)
            if leg is None:
                print(f"[ground] leg {leg_id} -> NO TELEMETRY (link timeout)")
                break
            legs.append(leg)
            print(f"[ground] leg {leg_id} -> {messages.LEG_STATUS_NAME.get(leg.status, leg.status)}  "
                  f"cmd {leg.commanded_dist_m:6.1f} m / ach {leg.achieved_dist_m:6.1f} m  "
                  f"E {leg.energy_J/1e3:7.1f} kJ  pos ({leg.final_row:.1f},{leg.final_col:.1f})")
            if leg.status not in (messages.LEG_REACHED,):
                break                                        # stop the mission on a non-nominal leg
        summary = self._summarize(legs, all_poses)
        self._write_artifacts(legs, all_poses, summary)
        return summary

    def _summarize(self, legs: list[messages.Leg], poses: list[messages.Pose]) -> dict:
        reached = sum(1 for L in legs if L.status == messages.LEG_REACHED)
        return {
            "legs": len(legs),
            "legs_reached": reached,
            "commanded_dist_m": sum(L.commanded_dist_m for L in legs),
            "achieved_dist_m": sum(L.achieved_dist_m for L in legs),
            "energy_J": sum(L.energy_J for L in legs),
            "final_soc": poses[-1].soc if poses else 1.0,
            "max_slip": max((p.slip for p in poses), default=0.0),
            "max_sinkage_m": max((p.sinkage_m for p in poses), default=0.0),
            "mass_kg_first": legs[0].mass_kg if legs else 0.0,
            "mass_kg_last": legs[-1].mass_kg if legs else 0.0,
            "uplink_packets": self.uplink_count,
            "downlink_packets": self.downlink_count,
            "pose_samples": len(poses),
        }

    def _write_artifacts(self, legs, poses, summary) -> None:
        os.makedirs(self.out_dir, exist_ok=True)
        with open(os.path.join(self.out_dir, "telemetry.json"), "w") as fh:
            json.dump({
                "summary": summary,
                "legs": [vars(L) for L in legs],
                "poses": [vars(p) for p in poses],
            }, fh, indent=2)
        with open(os.path.join(self.out_dir, "telemetry.csv"), "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["leg_id", "row", "col", "yaw_rad", "v_achieved_mps", "slip",
                        "sinkage_m", "slope_rad", "soc", "entrapped"])
            for p in poses:
                w.writerow([p.leg_id, f"{p.row:.4f}", f"{p.col:.4f}", f"{p.yaw_rad:.4f}",
                            f"{p.v_achieved_mps:.4f}", f"{p.slip:.4f}", f"{p.sinkage_m:.5f}",
                            f"{p.slope_rad:.4f}", f"{p.soc:.4f}", int(p.entrapped)])
        self._plot(poses, summary)

    def _plot(self, poses, summary) -> None:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        hm = self.crop.heightmap
        gy, gx = np.gradient(hm, self.crop.cell_m)
        # simple hillshade with a grazing sun (az 215, el 12 deg) for lunar-polar feel
        az, el = np.radians(215.0), np.radians(12.0)
        aspect = np.arctan2(-gx, gy)
        slope = np.arctan(np.hypot(gx, gy))
        shade = (np.sin(el) * np.cos(slope)
                 + np.cos(el) * np.sin(slope) * np.cos(az - aspect))
        shade = np.clip(shade, 0, 1)

        fig, (axm, axt) = plt.subplots(1, 2, figsize=(15, 6))
        axm.imshow(shade, cmap="gray", origin="upper")
        if poses:
            cols = [p.col for p in poses]
            rows = [p.row for p in poses]
            sc = axm.scatter(cols, rows, c=[p.slip for p in poses], cmap="plasma",
                             s=6, vmin=0.0, vmax=max(0.05, summary["max_slip"]))
            axm.plot(cols, rows, "-", color="cyan", lw=0.6, alpha=0.6)
            axm.plot(cols[0], rows[0], "o", color="lime", ms=9, label="start")
            axm.plot(cols[-1], rows[-1], "*", color="red", ms=14, label="end")
            fig.colorbar(sc, ax=axm, label="slip ratio", fraction=0.046)
        axm.set_title(f"Rover traverse on Haworth crop @ {self.crop.cell_m:.0f} m/cell "
                      f"(r0={self.crop.r0}, c0={self.crop.c0})")
        axm.set_xlabel("col"); axm.set_ylabel("row"); axm.legend(loc="upper right")

        if poses:
            idx = range(len(poses))
            axt.plot(idx, [p.slip for p in poses], label="slip")
            axt.plot(idx, [p.sinkage_m * 100 for p in poses], label="sinkage [cm]")
            axt.plot(idx, [p.soc for p in poses], label="SOC")
            axt.plot(idx, [abs(p.slope_rad) for p in poses], label="|slope| [rad]")
        axt.set_title("Downlinked telemetry"); axt.set_xlabel("pose sample"); axt.legend()
        axt.grid(True, alpha=0.3)

        fig.suptitle(
            f"CCSDS/ROS nav demo — {summary['legs_reached']}/{summary['legs']} legs reached, "
            f"{summary['achieved_dist_m']:.0f} m driven, "
            f"{summary['uplink_packets']} TC / {summary['downlink_packets']} TM packets, "
            f"final SOC {summary['final_soc']*100:.0f}%")
        fig.tight_layout()
        out = os.path.join(self.out_dir, "traverse.png")
        fig.savefig(out, dpi=120)
        plt.close(fig)
        print(f"[ground] wrote {out}")
