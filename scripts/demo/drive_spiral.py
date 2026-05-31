#!/usr/bin/env python3
"""Drive the rover along the spiral: kinematic terrain conform + 4-wheel track carve.

The honest "rover pose PRODUCER" step of the spiral demo, sitting between
``build_spiral_scene.py`` (the terrain) and the Godot render. It is the surrogate a real
Chrono::Vehicle + SCM solver would later replace behind the frozen INTERFACE seam (README
§4 #2-3); here it is GEOMETRY/STATE-ACCURATE, NOT FORCE-ACCURATE (terrain_authority/rover.py
header; spec §9) — no contact forces, no slip-sinkage (that is the deferred Chrono job).

For each frame of the SAME Archimedean spiral depart_spiral.gd / instrument_spiral.py walk
(TURNS=5, R0=30, R_GROWTH=36 cells, 80 frames) it:
  1. heads the rover along the TRAVEL TANGENT (front toward the next waypoint), NOT at the
     lander — the lander is then acquired by a SIDE mono (recorded per-frame as fiducial_cam).
  2. seats the rover on its 4 wheel contacts via rover.conform_pose: bilinear DEM heights,
     RIDING OVER clasts (the boulders are Python-authored in metadata.clasts), least-squares
     plane fit -> the resting surface-normal (tilt) + seat height. This is what stops the
     rover clipping the terrain / passing through boulders (README §4).
  3. carves FOUR separate compacting ruts (rover.four_wheel_pass) into a ColumnState rebuilt
     from the base scene — density up, mass conserved, height sinks; the per-wheel contact
     polylines become the §5.2 ``wheel_tracks`` metadata the renderer bakes into cleat detail.

Writes a SIBLING ``<scene>_driven`` scene (base stays pristine + the carve is idempotent):
  * the rutted scene (rasters + metadata + wheel_tracks + clasts copied through), and
  * ``rover_pose.json``: per-frame {rc, yaw_rad (Godot rover yaw), up (Godot-world normal),
    z_m, pitch_deg, roll_deg, fiducial_cam} consumed by --rover-pose in depart_spiral.gd /
    topdown_spiral.gd (the top-down trail accumulates by slicing wheel_tracks per frame).

YAW CONVENTIONS (load-bearing; spiral_path.py header). TWO distinct headings:
  * heading_field = atan2(drow, dcol)  -> the §5.2 wheel-cleat travel-heading fed to
    wheel_contact_points / four_wheel_pass / conform_pose.
  * yaw_godot     = -heading_field = look_at_yaw(rc[i], rc[i+1]) -> the Godot rover yaw
    (Basis(UP,yaw); sidecar._heading_yaw convention, atan2(-dz,dx)) emitted to rover_pose.json.

Run: .venv/bin/python scripts/demo/drive_spiral.py [--scene <dir>] [--out <dir>]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
sys.path.insert(0, os.path.join(_ROOT, "scripts", "demo"))

from terrain_authority import rover
from terrain_authority.column_state import ColumnState
from terrain_authority.io_fields import load_scene, save_scene
import spiral_path

# Spiral params MUST match depart_spiral.gd / instrument_spiral.py (R0=30, R_GROWTH=36, TURNS=5, 80 frames).
TURNS = 5
FRAMES = 80
R0_CELLS = 30.0
R_GROWTH_CELLS = 36.0

WHEEL_WIDTH_M = 0.18         # IPEx contact-patch width (asce-es-2024); rover.four_wheel_pass default
COMPACTION = 0.12            # fractional density increase per pass (rover.wheel_pass default)


def _travel_heading_field(rc_seq, i):
    """§5.2 field travel-heading atan2(drow,dcol): front toward the NEXT waypoint.

    Last frame reuses the previous segment's direction. Returns 0.0 for a degenerate
    (zero-length) step, mirroring spiral_path.look_at_yaw's no-travel fallback.
    """
    n = len(rc_seq)
    if i < n - 1:
        a, b = rc_seq[i], rc_seq[i + 1]
    else:
        a, b = rc_seq[i - 1], rc_seq[i]
    drow = float(b[0] - a[0])
    dcol = float(b[1] - a[1])
    if abs(drow) < 1e-9 and abs(dcol) < 1e-9:
        return 0.0
    return math.atan2(drow, dcol)


def _fiducial_side(rx, rz, cx, cz, yaw_godot):
    """Which side mono frames the fixed-center lander given a travel-tangent heading.

    Rover right (world x,z) under Basis(UP,yaw_godot) is -Z_body = (-sin yaw, -cos yaw).
    Positive projection of the rover->lander vector onto that is the lander on the right.
    """
    rdir = (-math.sin(yaw_godot), -math.cos(yaw_godot))   # rover +right in world (x,z)
    to = (cx - rx, cz - rz)
    return "right_mono" if (to[0] * rdir[0] + to[1] * rdir[1]) >= 0.0 else "left_mono"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default=os.path.join(_ROOT, "godot_sidecar", "out", "scenes", "haworth_spiral"))
    ap.add_argument("--out", default="", help="output (driven) scene dir; default <scene>_driven")
    ap.add_argument("--frames", type=int, default=FRAMES)
    ap.add_argument("--wheel-width-m", type=float, default=WHEEL_WIDTH_M)
    ap.add_argument("--compaction", type=float, default=COMPACTION)
    ap.add_argument("--smooth", type=int, default=5,
                    help="box-average window (frames, odd) for the conform tilt; 1=off")
    a = ap.parse_args()

    scene_dir = a.scene.rstrip("/")
    out_dir = a.out.rstrip("/") if a.out else (scene_dir + "_driven")

    fields, meta = load_scene(scene_dir)
    H, W = fields["heightmap"].shape
    cell_m = float(meta["grid"]["cell_m"])
    wb = meta["world_bounds_m"]
    x0, y0 = float(wb["x0"]), float(wb["y0"])
    clasts = meta.get("clasts", [])

    base_h = np.asarray(fields["heightmap"], dtype=np.float64)   # PRE-carve terrain for conform
    center_rc = ((H - 1) * 0.5, (W - 1) * 0.5)
    cx = x0 + center_rc[1] * cell_m
    cz = y0 + center_rc[0] * cell_m

    rc_seq = spiral_path.spiral_rc(center_rc, a.frames, turns=TURNS, r0_cells=R0_CELLS,
                                   r_growth_cells=R_GROWTH_CELLS, cell_m=cell_m)

    # --- per-frame conform pose + the (center_rc, heading_field) carve sequence ----------
    records = []
    poses_field = []
    heading_seq = []
    for i, rc in enumerate(rc_seq):
        hf = _travel_heading_field(rc_seq, i)
        yaw_godot = -hf                                   # == look_at_yaw(rc[i], rc[i+1])
        conf = rover.conform_pose(base_h, rc, hf, cell_m=cell_m, world_x0=x0, world_y0=y0,
                                  clasts=clasts)
        rx = x0 + rc[1] * cell_m
        rz = y0 + rc[0] * cell_m
        records.append({
            "frame": i,
            "rc": [float(rc[0]), float(rc[1])],
            "yaw_rad": float(yaw_godot),
            "up": conf["up"],
            "z_m": conf["z_m"],
            "pitch_deg": round(math.degrees(conf["pitch_rad"]), 3),
            "roll_deg": round(math.degrees(conf["roll_rad"]), 3),
            "fiducial_cam": _fiducial_side(rx, rz, cx, cz, yaw_godot),
        })
        poses_field.append((rc, hf))
        heading_seq.append(hf)

    # --- temporal smoothing of the conform tilt (box window) ------------------------------
    # The fixed spiral is NOT obstacle-aware, so at ~1 m frame spacing it occasionally crosses
    # a boulder a planner would route around -> a 1-frame lurch. A short box-average of the
    # up-normal approximates the rigid chassis RAMPING over short-wavelength features instead of
    # teleporting onto them (geometry smoothing, NOT dynamics; the carve below is unaffected --
    # ruts stay where the wheels actually were). --smooth 1 disables it.
    if a.smooth >= 3:
        ups = np.array([r["up"] for r in records], dtype=np.float64)
        half = a.smooth // 2
        for i, r in enumerate(records):
            v = ups[max(0, i - half):min(len(records), i + half + 1)].mean(axis=0)
            v /= np.linalg.norm(v)
            r["up"] = [float(v[0]), float(v[1]), float(v[2])]
            av, bv = -v[0] / v[1], -v[2] / v[1]        # plane gradient from the smoothed normal
            ch2, sh2 = math.cos(heading_seq[i]), math.sin(heading_seq[i])
            r["pitch_deg"] = round(math.degrees(math.atan(av * ch2 + bv * sh2)), 3)
            r["roll_deg"] = round(math.degrees(math.atan(-av * sh2 + bv * ch2)), 3)

    # --- carve FOUR ruts into a ColumnState rebuilt from the base scene (mass conserved) --
    mass = np.asarray(fields["mass_areal"], dtype=np.float64)
    dens = np.asarray(fields["density"], dtype=np.float64)
    cs = ColumnState(width=W, height=H, cell_m=cell_m,
                     mass_areal=mass.copy(), density=dens.copy(),
                     state_label=np.asarray(fields["state_label"], dtype=np.uint8).copy(),
                     disturbance=np.asarray(fields["disturbance"], dtype=np.float64).copy(),
                     datum=(base_h - mass / dens))
    assert float(np.max(np.abs(cs.derive_height() - base_h))) < 1e-3, "ColumnState rebuild != base heightmap"
    m0 = cs.total_mass()
    polylines = rover.four_wheel_pass(cs, poses_field, wheel_width_m=a.wheel_width_m,
                                      compaction=a.compaction)
    m1 = cs.total_mass()
    drived_h = cs.derive_height()
    sank = float(np.min(drived_h - base_h))               # most-sunk rut depth (negative)

    wheel_tracks = rover.build_wheel_tracks_meta(polylines, float(heading_seq[-1]),
                                                 cell_m=cell_m, width_m=a.wheel_width_m)

    # --- write the driven scene (rutted rasters + metadata + wheel_tracks) ----------------
    out_meta = dict(meta)
    out_meta["scene_name"] = os.path.basename(out_dir)
    out_meta["wheel_tracks"] = wheel_tracks
    out_meta["height_range_m"] = [round(float(drived_h.min()), 4), round(float(drived_h.max()), 4)]
    out_meta["producer"] = (meta.get("producer", "") +
                            " | drive_spiral.py: kinematic conform + 4-wheel carve "
                            f"({a.frames} frames, geometry-accurate not force-accurate, spec §9)")
    save_scene(out_dir, cs.fields_dict(), out_meta)

    pose_doc = {
        "scene": os.path.basename(out_dir),
        "frames": len(records),
        "convention": "yaw_rad=Godot rover yaw (Basis(UP,yaw)); up=Godot world surface normal (y-up)",
        "params": {"turns": TURNS, "r0_cells": R0_CELLS, "r_growth_cells": R_GROWTH_CELLS, "cell_m": cell_m},
        "records": records,
    }
    pose_path = os.path.join(out_dir, "rover_pose.json")
    json.dump(pose_doc, open(pose_path, "w"))

    tilt = np.array([math.hypot(r["pitch_deg"], r["roll_deg"]) for r in records])  # total tilt magnitude
    n_right = sum(1 for r in records if r["fiducial_cam"] == "right_mono")
    print(f"drive_spiral: {len(records)} frames -> {out_dir}")
    print(f"  conform tilt magnitude: median {np.median(tilt):.2f} / p90 {np.percentile(tilt,90):.2f} "
          f"/ max {tilt.max():.2f} deg (kinematic, geometry-accurate; DEM ~3deg + occasional clast bumps)")
    print(f"  4-wheel carve: rut depth {sank*100:.2f} cm; mass drift {abs(m1-m0)/m0:.2e} (conserved)")
    print(f"  fiducial side: {n_right} right_mono / {len(records)-n_right} left_mono frames")
    print(f"  + rover_pose.json -> {pose_path}")


if __name__ == "__main__":
    main()
