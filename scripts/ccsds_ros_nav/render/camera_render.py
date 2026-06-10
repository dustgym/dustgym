"""Shared sun-aware camera render — the rover's 'camera' on the host GPU.

Publishes a crop of the Haworth DEM as an INTERFACE.md scene with a given rover pose + Sun position,
then drives the FROZEN Godot ``--cameras-seq`` egress (xvfb + Vulkan) to render the moving front/rear
stereo + side/drum cameras and the per-frame sensors.json (the COLMAP feed). Reused by the batch
``render_egress.py`` and the live HITL console. No sidecar seam is edited; the Sun azimuth/elevation
(from the mission clock) flow in via the sidecar's ``--sun-azim``/``--sun-elev`` flags.
"""
from __future__ import annotations

import copy
import glob
import json
import os
import shutil
import subprocess
import sys

import numpy as np

_PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # scripts/ccsds_ros_nav
_REPO = os.path.abspath(os.path.join(_PKG, "..", ".."))
for _p in (_PKG, _REPO):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from terrain_authority import io_fields


def publish_scene(crop, scene_dir: str, scene_name: str, rover_rc: tuple[int, int],
                  haworth_dir: str, *, sun_az: float | None = None, sun_el: float | None = None) -> None:
    """Write the crop as an INTERFACE.md scene with the rover endpoint authored as rover_rc."""
    _base, meta = io_fields.load_scene(haworth_dir)                  # faithful metadata template
    cm = crop.cell_m
    win_h, win_w = crop.heightmap.shape
    md = copy.deepcopy(meta)
    md["grid"] = {"width": int(win_w), "height": int(win_h), "cell_m": float(cm)}
    # LOCAL RENDER FRAME (critical). Godot's terrain mesh is float32; at Haworth's absolute world
    # offset (~47 km / ~100 km) AND ~2820 m elevation the vertices + screen-space-derivative normals
    # lose precision and the terrain renders BLACK while the rover (standard material) still lights —
    # the exact failure worksite_roam.py already hit and fixed. So re-origin to a local frame: world
    # origin (0,0) and a subtracted height datum, so every rendered coordinate is small (0..~800 m).
    # This is a pure translation — relative geometry, slopes, and the (directional) Sun are unchanged;
    # rover/lander placement is grid-relative (world_min + rc*cell) so it follows automatically.
    hmin = float(np.min(crop.heightmap))
    hmax = float(np.max(crop.heightmap))
    md["world_bounds_m"] = {"x0": 0.0, "y0": 0.0,
                            "x1": (win_w - 1) * cm, "y1": (win_h - 1) * cm}
    md["height_range_m"] = [0.0, (hmax - hmin) if hmax > hmin else 1.0]
    md["render_height_datum_m"] = hmin                # the absolute datum subtracted (traceability)
    md["scene_name"] = scene_name
    md["rover_rc"] = [int(rover_rc[0]), int(rover_rc[1])]
    if sun_az is not None and sun_el is not None:
        md["sun"] = {"azimuth_deg": float(sun_az), "elevation_deg": float(sun_el)}
    fields = {
        "heightmap": (crop.heightmap.astype(np.float64) - hmin),       # local height datum
        "mass_areal": crop.fields["mass_areal"],
        "density": crop.fields["density"],
        "disturbance": crop.fields["disturbance"],
        "state_label": crop.fields["state_label"],
    }
    io_fields.save_scene(scene_dir, fields, md)


def run_cameras_seq(repo_root: str, scene_dir: str, *, frames: int, size: str,
                    sun_az: float, sun_el: float, timeout: float = 180.0) -> tuple[int, str]:
    """Invoke the frozen --cameras-seq sidecar on the host GPU. Returns (returncode, combined log)."""
    rl = os.path.join(repo_root, "godot_sidecar", "render_layers.sh")
    cmd = [rl, "--", "--scene", os.path.abspath(scene_dir), "--cameras-seq", "--stride", str(frames),
           "--sun-azim", f"{sun_az:.2f}", "--sun-elev", f"{sun_el:.2f}",
           "--layers", "terrain,clasts,rover", "--size", size]
    proc = subprocess.run(cmd, cwd=os.path.join(repo_root, "godot_sidecar"),
                          capture_output=True, text=True, timeout=timeout)
    return proc.returncode, (proc.stdout[-3000:] + "\n" + proc.stderr[-2000:])


def clear_cam_root(repo_root: str, scene_name: str) -> None:
    """Remove any prior render of this scene so stale higher-numbered frame dirs are never re-collected."""
    shutil.rmtree(os.path.join(repo_root, "godot_sidecar", "out", "cam", scene_name), ignore_errors=True)


def collect_frames(repo_root: str, scene_name: str) -> list[dict]:
    """Read the rendered frames: one record per (frame, camera) with the PNG path + sensors.json pose."""
    cam_root = os.path.join(repo_root, "godot_sidecar", "out", "cam", scene_name)
    out: list[dict] = []
    for fdir in sorted(glob.glob(os.path.join(cam_root, "[0-9][0-9][0-9]"))):
        nnn = os.path.basename(fdir)
        sj = os.path.join(fdir, "sensors.json")
        if not os.path.exists(sj):
            continue
        with open(sj) as fh:
            sensors = json.load(fh)
        for cam in sensors.get("cameras", []):
            png = os.path.join(fdir, cam["image"])
            if os.path.exists(png):
                out.append({"frame": int(nnn), "camera": cam["name"], "path": png,
                            "width": int(cam["width"]), "height": int(cam["height"])})
    return out


def render_single(repo_root: str, crop, rover_rc: tuple[int, int], *, yaw_flight: float,
                  sun_az: float, sun_el: float, scene_name: str, haworth_dir: str,
                  lander_rc: tuple[int, int] | None = None, size: str = "640x480",
                  cam_pitch_deg: float = 0.0, timeout: float = 120.0) -> dict:
    """Render ONE frame of all cameras at the rover's actual pose + heading (single-frame --cameras).

    The lander is placed at a FIXED world cell (``lander_rc``) so it is a stationary landmark the rover
    drives away from — the camera view then moves WITH the rover instead of being pinned to a lander
    glued ahead. ``yaw_flight`` is the sim heading (forward = (sin, cos) in (row, col)); the sidecar's
    +X yaw is its negation.

    ``cam_pitch_deg`` tilts the FRONT stereo pair DOWN toward the ground (camera_rig.gd ``--cam-pitch``).
    The stereo module sits only ~8 cm above the surface, so a level (0°) gaze stares at the black lunar
    sky — passive-stereo / COLMAP needs textured GROUND in frame. A positive pitch aims it at the terrain.
    """
    scene_dir = os.path.join(repo_root, "out", "ccsds_nav", "scenes", scene_name)
    publish_scene(crop, scene_dir, scene_name, rover_rc, haworth_dir, sun_az=sun_az, sun_el=sun_el)
    clear_cam_root(repo_root, scene_name)                    # drop stale frames from any prior render
    rl = os.path.join(repo_root, "godot_sidecar", "render_layers.sh")
    cmd = [rl, "--", "--scene", os.path.abspath(scene_dir), "--cameras",
           "--rover-rc", f"{int(rover_rc[0])},{int(rover_rc[1])}",
           "--rover-yaw", f"{-float(yaw_flight):.5f}",
           "--sun-azim", f"{sun_az:.2f}", "--sun-elev", f"{sun_el:.2f}",
           "--layers", "terrain,clasts,rover", "--size", size]
    if cam_pitch_deg > 1e-6:
        cmd += ["--cam-pitch", f"{float(cam_pitch_deg):.2f}"]
    if lander_rc is not None:
        cmd += ["--lander-rc", f"{int(lander_rc[0])},{int(lander_rc[1])}"]
    proc = subprocess.run(cmd, cwd=os.path.join(repo_root, "godot_sidecar"),
                          capture_output=True, text=True, timeout=timeout)
    if proc.returncode != 0:
        return {"ok": False, "log": proc.stdout[-2000:] + "\n" + proc.stderr[-1500:], "frames": []}
    return {"ok": True, "frames": collect_frames(repo_root, scene_name), "scene_name": scene_name}


def render_chase(repo_root: str, crop, rover_rc: tuple[int, int], *, yaw_flight: float,
                 sun_az: float, sun_el: float, scene_name: str, haworth_dir: str,
                 lander_rc: tuple[int, int] | None = None, size: str = "960x720",
                 timeout: float = 120.0) -> dict:
    """Render ONE trailing third-person 'coach' view of the rover (sidecar ``--chase-cam``).

    An EXTERNAL vantage (behind+above+side, looking at the rover) — the sim manager's view, not a
    rover sensor: it never feeds COLMAP, so it is rendered separately at a higher resolution and is
    opt-in in the console ('cheating' for an operator). Returns ``{"ok", "path"}`` (the chase PNG).
    """
    scene_dir = os.path.join(repo_root, "out", "ccsds_nav", "scenes", scene_name)
    publish_scene(crop, scene_dir, scene_name, rover_rc, haworth_dir, sun_az=sun_az, sun_el=sun_el)
    clear_cam_root(repo_root, scene_name)
    rl = os.path.join(repo_root, "godot_sidecar", "render_layers.sh")
    cmd = [rl, "--", "--scene", os.path.abspath(scene_dir), "--cameras", "--chase-cam",
           "--rover-rc", f"{int(rover_rc[0])},{int(rover_rc[1])}",
           "--rover-yaw", f"{-float(yaw_flight):.5f}",
           "--sun-azim", f"{sun_az:.2f}", "--sun-elev", f"{sun_el:.2f}",
           "--layers", "terrain,clasts,rover", "--size", size]
    if lander_rc is not None:
        cmd += ["--lander-rc", f"{int(lander_rc[0])},{int(lander_rc[1])}"]
    proc = subprocess.run(cmd, cwd=os.path.join(repo_root, "godot_sidecar"),
                          capture_output=True, text=True, timeout=timeout)
    png = os.path.join(repo_root, "godot_sidecar", "out", "cam", scene_name, "000", "chase.png")
    if proc.returncode != 0 or not os.path.exists(png):
        return {"ok": False, "log": proc.stdout[-2000:] + "\n" + proc.stderr[-1500:]}
    return {"ok": True, "path": png, "scene_name": scene_name}


def render_capture(repo_root: str, crop, rover_rc: tuple[int, int], *, sun_az: float, sun_el: float,
                   scene_name: str, haworth_dir: str, frames: int = 3, size: str = "640x480",
                   timeout: float = 180.0) -> dict:
    """Publish the scene at (rover_rc, sun), render the camera sequence, return the collected frames."""
    scene_dir = os.path.join(repo_root, "out", "ccsds_nav", "scenes", scene_name)
    publish_scene(crop, scene_dir, scene_name, rover_rc, haworth_dir, sun_az=sun_az, sun_el=sun_el)
    clear_cam_root(repo_root, scene_name)                    # drop stale frames from any prior render
    rc, log = run_cameras_seq(repo_root, scene_dir, frames=frames, size=size,
                              sun_az=sun_az, sun_el=sun_el, timeout=timeout)
    if rc != 0:
        return {"ok": False, "log": log, "frames": []}
    return {"ok": True, "frames": collect_frames(repo_root, scene_name), "scene_name": scene_name}
