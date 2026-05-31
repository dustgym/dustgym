"""Evaluation schema for the SLAM/pose scorer (sensor_bridge_contract.md v1.1, eval_schema).

Pure-stdlib dataclass/JSON module (dataclasses, json, math) -- NO ROS / rclpy / numpy
imports.  Lane C's `synthetic_feed.py` + `eval_harness.py` build against this BEFORE the
real M2 egress exists; M2 later feeds the same `TrajectorySample` stream into the live
scorer.  Keeping it ros-free means the schema + the rover_rc->metric lift rule are unit
testable on the bare host .venv.

TWO INDEPENDENT VALIDATION PATHS (NEVER summed into one number)
---------------------------------------------------------------
This module's `Scorecard` carries fields from two *separate* truth channels.  They measure
different things on different data and must be reported side by side, never added or averaged:

  (1) POSE / ATE from `tread_track_4wheel` `rover_rc` (the synthetic-trajectory channel).
      Truth poses are lifted from each frame's integer `rover_rc` grid cell + the persisted
      per-frame `wheel_tracks[*].heading_rad`; the *estimate* is `truth + noise` injected by
      lane C's synthetic feed.  TRANSLATION + YAW ONLY -- there is NO roll/pitch truth in the
      terrain frames (the surrogate moves a 2-D footprint over a heightmap), so the only
      meaningful rotation metric here is `pose_rmse_yaw_deg` (yaw about the map +Z axis).
      `pose_rmse_trans_mm` and `ate_mm` (Absolute Trajectory Error, trans-only) also live on
      this channel.

  (2) APRILTAG-RELATIVE POSE from an `out/cam` fixture (the live 12.7 mm / 7.15 deg M1
      reading -- see compare_pose.py / bag_writer._compute_truth).  This is a standalone,
      single-pose camera->tag measurement with a *full* quaternion truth.  Full-quaternion
      (geodesic) rotation RMSE is ONLY meaningful on THIS channel (or on real M2 data); it is
      NOT representable in the yaw-only synthetic channel above.

So: `pose_rmse_yaw_deg` is the synthetic-trajectory rotation metric (yaw only); a full
rotation RMSE belongs to the apriltag channel / real M2 and is reported separately by the
apriltag scorer, not mixed into the trajectory Scorecard.

ROVER_RC -> METRIC LIFT RULE
----------------------------
`rover_rc` is `[row, col]` integer grid indices into the scene's row-major-C field (verified
against samples/tread_track_4wheel/t018/metadata.json: grid.order == "row-major-C").  Using
the scene metadata.json `grid.cell_m` and `world_bounds_m`:

    world_x_m = world_bounds_m["x0"] + col * cell_m
    world_z_m = world_bounds_m["y0"] + row * cell_m
    world_y_m = 0.0            # ground plane; no z-truth in the 2-D terrain surrogate

(The contract states "world x = col*cell_m, z = row*cell_m"; we add the world_bounds_m
origin offsets, which are 0.0 for the sample scenes, so the bare-multiply contract form is
recovered exactly.)  This places the sample on the Godot map plane; the resulting
`TrajectorySample.frame` is labelled 'map' to match the ROS map-frame topic the scorer
consumes -- but NOTE this module performs NO REP-103 axis conversion: the Godot->ROS seam
stays C1's job (frames.py).  The lift is a planar grid->metric mapping only.

HEADING / YAW SOURCE (persisted-first, delta-fallback)
------------------------------------------------------
Heading is read from the *persisted* per-frame metadata where present.  VERIFIED:
`tread_track_4wheel` frames carry it as `wheel_tracks[<wheel>].heading_rad` (radians, REP-103
CCW about +Z; all four wheels agree per frame).  Plain `tread_track` frames have NO
`wheel_tracks` block, so heading there falls back to the rover_rc-delta heading
(atan2 between consecutive cells).  The persisted value is authoritative and is preferred
whenever available.

QUANTIZATION NOTE
-----------------
`rover_rc` is stored as integer grid indices, so the lifted position is quantized to the
grid: resolution is capped at ~`cell_m` (~0.02 m == 20 mm for the sample scenes).  Sub-cell
trajectory error below ~one cell is therefore not resolvable from `rover_rc` alone; pose RMSE
/ ATE figures at or under ~`cell_m` reflect this quantization floor, not estimator skill.

NULL ROVER_RC
-------------
`rover_rc` is `null` on pre-motion frames (e.g. t000) -- the rover has not been placed yet.
Such frames are SKIPPED by the lift (no truth sample emitted); the delta-heading fallback
likewise skips them and uses the first frame with two consecutive non-null cells.

CC0-1.0 (see ../../LICENSE).
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any, Mapping, Optional, Sequence


# --- TrajectorySample -------------------------------------------------------------------

@dataclass
class TrajectorySample:
    """One pose on a trajectory, in the ROS map frame (REP-103 labelling).

    `quaternion_xyzw` is XYZW order (matching sensors.json `quaternion_xyzw` and ROS
    geometry_msgs/Quaternion field order).  For the synthetic rover_rc channel only YAW is
    populated (rotation about map +Z); roll/pitch are 0 because no such truth exists.
    """

    frame_index: int
    t_s: float
    position_m: Sequence[float]            # [x, y, z]
    quaternion_xyzw: Sequence[float]       # [x, y, z, w]
    frame: str = "map"

    def to_dict(self) -> dict[str, Any]:
        return {
            "frame_index": int(self.frame_index),
            "t_s": float(self.t_s),
            "position_m": [float(c) for c in self.position_m],
            "quaternion_xyzw": [float(c) for c in self.quaternion_xyzw],
            "frame": str(self.frame),
        }

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "TrajectorySample":
        return cls(
            frame_index=int(d["frame_index"]),
            t_s=float(d["t_s"]),
            position_m=[float(c) for c in d["position_m"]],
            quaternion_xyzw=[float(c) for c in d["quaternion_xyzw"]],
            frame=str(d.get("frame", "map")),
        )

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=False)

    @classmethod
    def from_json(cls, s: str) -> "TrajectorySample":
        return cls.from_dict(json.loads(s))


# --- Scorecard --------------------------------------------------------------------------

@dataclass
class Scorecard:
    """Aggregate evaluation result.

    Channel (1) synthetic trajectory (rover_rc):
        pose_rmse_trans_mm  -- translation RMSE over all matched samples (mm).
        pose_rmse_yaw_deg   -- YAW-ONLY rotation RMSE (deg); see module docstring.
        ate_mm              -- Absolute Trajectory Error, translation only (mm).
        n_frames            -- number of matched (non-null) trajectory samples scored.
    Channel-independent optional metrics (None when not evaluated; never summed with above):
        map_rmse_m          -- reconstructed-vs-truth map height/geometry RMSE (m).
        map_cell_pass_frac  -- fraction of map cells within tolerance.
        rock_f1             -- boulder/rock detection F1.
    """

    pose_rmse_trans_mm: float
    pose_rmse_yaw_deg: float
    ate_mm: float
    n_frames: int
    map_rmse_m: Optional[float] = None
    map_cell_pass_frac: Optional[float] = None
    rock_f1: Optional[float] = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "pose_rmse_trans_mm": float(self.pose_rmse_trans_mm),
            "pose_rmse_yaw_deg": float(self.pose_rmse_yaw_deg),
            "ate_mm": float(self.ate_mm),
            "n_frames": int(self.n_frames),
            "map_rmse_m": None if self.map_rmse_m is None else float(self.map_rmse_m),
            "map_cell_pass_frac": (
                None if self.map_cell_pass_frac is None else float(self.map_cell_pass_frac)
            ),
            "rock_f1": None if self.rock_f1 is None else float(self.rock_f1),
        }

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "Scorecard":
        def _opt(v: Any) -> Optional[float]:
            return None if v is None else float(v)

        return cls(
            pose_rmse_trans_mm=float(d["pose_rmse_trans_mm"]),
            pose_rmse_yaw_deg=float(d["pose_rmse_yaw_deg"]),
            ate_mm=float(d["ate_mm"]),
            n_frames=int(d["n_frames"]),
            map_rmse_m=_opt(d.get("map_rmse_m")),
            map_cell_pass_frac=_opt(d.get("map_cell_pass_frac")),
            rock_f1=_opt(d.get("rock_f1")),
        )

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=False)

    @classmethod
    def from_json(cls, s: str) -> "Scorecard":
        return cls.from_dict(json.loads(s))


# --- rover_rc -> metric lift -------------------------------------------------------------

def yaw_to_quat_xyzw(yaw_rad: float) -> list[float]:
    """Yaw (rad, CCW about map +Z, REP-103) -> unit quaternion XYZW.

    Roll/pitch are 0 (no such truth in the 2-D terrain surrogate), so this is a pure
    rotation about +Z: q = [0, 0, sin(yaw/2), cos(yaw/2)].
    """
    h = 0.5 * float(yaw_rad)
    return [0.0, 0.0, math.sin(h), math.cos(h)]


def quat_xyzw_to_yaw(q: Sequence[float]) -> float:
    """Inverse of `yaw_to_quat_xyzw`: extract the +Z yaw (rad) from an XYZW quaternion.

    Uses the standard atan2 yaw extraction so it is robust to small roll/pitch noise.
    """
    x, y, z, w = (float(c) for c in q)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _grid_cell_m(metadata: Mapping[str, Any]) -> float:
    return float(metadata["grid"]["cell_m"])


def _world_origin_m(metadata: Mapping[str, Any]) -> tuple[float, float]:
    wb = metadata.get("world_bounds_m", {})
    return float(wb.get("x0", 0.0)), float(wb.get("y0", 0.0))


def rover_rc_to_world_xz(
    rover_rc: Sequence[int], metadata: Mapping[str, Any]
) -> tuple[float, float]:
    """Lift integer `rover_rc` = [row, col] to Godot map-plane (world_x_m, world_z_m).

        world_x_m = x0 + col * cell_m
        world_z_m = y0 + row * cell_m

    (Contract form `x = col*cell_m, z = row*cell_m` with the world_bounds_m origin added;
    origins are 0.0 for the sample scenes.)  No REP-103 conversion -- that is frames.py's job.
    """
    row, col = int(rover_rc[0]), int(rover_rc[1])
    cell_m = _grid_cell_m(metadata)
    x0, y0 = _world_origin_m(metadata)
    return x0 + col * cell_m, y0 + row * cell_m


def persisted_heading_rad(metadata: Mapping[str, Any]) -> Optional[float]:
    """Return the persisted per-frame heading (rad) if the scene stores one, else None.

    VERIFIED: `tread_track_4wheel` carries it under `wheel_tracks[<wheel>].heading_rad`
    (all wheels agree per frame).  Plain `tread_track` has no `wheel_tracks` -> None
    (caller then uses the rover_rc-delta fallback).
    """
    wheel_tracks = metadata.get("wheel_tracks")
    if isinstance(wheel_tracks, Mapping):
        for track in wheel_tracks.values():
            if isinstance(track, Mapping) and "heading_rad" in track:
                return float(track["heading_rad"])
    # Tolerate a hypothetical future top-level heading field too.
    for key in ("heading_rad", "heading"):
        if key in metadata and metadata[key] is not None:
            return float(metadata[key])
    return None


def lift_trajectory(
    frames: Sequence[Mapping[str, Any]],
    *,
    t_step_s: float = 1.0,
    t0_s: float = 0.0,
) -> list[TrajectorySample]:
    """Lift a sequence of scene metadata dicts to map-frame `TrajectorySample`s.

    Truth construction per the contract rover_rc->metric rule:
      * SKIP any frame whose `rover_rc` is null (e.g. t000, pre-placement).
      * position from `rover_rc_to_world_xz` (y=0, the ground plane).
      * yaw: PERSISTED `wheel_tracks[*].heading_rad` if present; otherwise the rover_rc-delta
        heading atan2(world_z_next-world_z_prev, world_x_next-world_x_prev) between this and
        the previous emitted sample (the first emitted sample, lacking a predecessor, reuses
        the next available delta; if it is the only sample, yaw=0).
      * frame_index from the frame's persisted `frame_index` when present, else its index in
        the surviving (non-null) sequence.
      * t_s = t0_s + emitted_index * t_step_s (the synthetic cadence; M2 may override).

    Note the integer-rover_rc quantization caps spatial resolution at ~cell_m (see module
    docstring); delta-headings on adjacent cells are likewise quantization-limited.
    """
    # First pass: keep only frames with a non-null rover_rc, carrying their metadata.
    kept: list[Mapping[str, Any]] = [
        m for m in frames if m.get("rover_rc") is not None
    ]
    if not kept:
        return []

    # Pre-compute world positions for delta-heading fallback.
    positions_xz: list[tuple[float, float]] = [
        rover_rc_to_world_xz(m["rover_rc"], m) for m in kept
    ]

    def _delta_heading(i: int) -> float:
        # Heading from displacement between consecutive kept cells.
        if len(positions_xz) < 2:
            return 0.0
        if i == 0:
            ax, az = positions_xz[0]
            bx, bz = positions_xz[1]
        else:
            ax, az = positions_xz[i - 1]
            bx, bz = positions_xz[i]
        dx, dz = bx - ax, bz - az
        if dx == 0.0 and dz == 0.0:
            # No motion this step: reuse the previous sample's heading if any, else 0.
            return _delta_heading(i - 1) if i > 0 else 0.0
        return math.atan2(dz, dx)

    samples: list[TrajectorySample] = []
    for i, m in enumerate(kept):
        wx, wz = positions_xz[i]
        yaw = persisted_heading_rad(m)
        if yaw is None:
            yaw = _delta_heading(i)
        frame_index = int(m["frame_index"]) if "frame_index" in m else i
        samples.append(
            TrajectorySample(
                frame_index=frame_index,
                t_s=t0_s + i * t_step_s,
                position_m=[wx, 0.0, wz],
                quaternion_xyzw=yaw_to_quat_xyzw(yaw),
                frame="map",
            )
        )
    return samples


def load_scene_frames(metadata_paths: Sequence[str]) -> list[dict[str, Any]]:
    """Convenience loader: read a list of per-frame metadata.json paths in order.

    Each path is a scene-frame metadata.json (e.g. samples/<scene>/tNNN/metadata.json).
    Returned dicts are passed straight to `lift_trajectory`.
    """
    out: list[dict[str, Any]] = []
    for p in metadata_paths:
        with open(p, "r", encoding="utf-8") as fh:
            out.append(json.load(fh))
    return out
