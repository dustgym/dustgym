#!/usr/bin/env python3
"""Spiral-demo instrumentation panels as accumulating GIFs (John's viz battery, part 1).

Builds, for a rendered+detected spiral run (out/cam/<run>/<NNN>/{sensors,detect}.json + the
scene's resource.json):
  - position_slam.gif : top-down (ROS ground plane) TRUTH rover path vs AprilTag-derived (SLAM)
    estimate, accumulating 0..N. Lander fixed at centre; the 4 quadrants are shaded by which
    tag face should be visible from that bearing; diagonal lines mark the quadrant boundaries;
    each estimate's error is the line to its truth point. Detection failures show truth only.
  - resource.gif      : 2 cm-corridor resident memory (MB) + quadtree active-leaf count,
    accumulating, with the O(area) dense-2cm reference the corridor avoids.
And a static failure_breakdown.png comparing LIT vs UNLIT detection outcomes.

Host-only (numpy + matplotlib Agg + Pillow); reuses frames.py (godot->ROS) + the merged data.
"""
from __future__ import annotations
import argparse, glob, io, json, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge
from PIL import Image

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(_ROOT, "scripts", "ros2_bridge"))
import frames  # noqa: E402

FACE_COLORS = {0: "#1f77b4", 1: "#ff7f0e", 2: "#2ca02c", 3: "#d62728"}  # id0..3
# Lander face outward normals in Godot world (lander_bundle FACES): id0 +X, id1 +Z, id2 -X, id3 -Z.
FACE_NORMALS_GODOT = {0: (1, 0, 0), 1: (0, 0, 1), 2: (-1, 0, 0), 3: (0, 0, -1)}


def _load_run(run_dir):
    """Return (lander_xy, [per-frame dict]) in the ROS ground plane (x,y)."""
    frame_dirs = sorted(d for d in glob.glob(os.path.join(run_dir, "*")) if os.path.isdir(d))
    lander_xy = None
    rows = []
    for d in frame_dirs:
        sp = os.path.join(d, "sensors.json")
        if not os.path.exists(sp):
            continue
        s = json.load(open(sp))
        tpos, _ = frames.godot_world_pose_to_ros(s["rover"]["position_m"], s["rover"]["quaternion_xyzw"])
        if lander_xy is None:
            lpos, _ = frames.godot_world_pose_to_ros(s["lander"]["position_m"], s["lander"]["quaternion_xyzw"])
            lander_xy = (float(lpos[0]), float(lpos[1]))
        det = {}
        dp = os.path.join(d, "detect.json")
        if os.path.exists(dp):
            det = json.load(open(dp))
        est = det.get("rover_est_map")
        est_xy = (float(est["position_m"][0]), float(est["position_m"][1])) if est else None
        rows.append({
            "truth_xy": (float(tpos[0]), float(tpos[1])),
            "est_xy": est_xy,
            "range_m": det.get("range_m"),
            "faces": det.get("detected_faces", []),
            "err_mm": (min((x["trans_mm"] for x in det.get("per_face_err_vs_truth", [])), default=None)),
        })
    return lander_xy, rows


def _face_bearings_ros():
    """ROS ground-plane bearing (rad) of each face's outward normal."""
    out = {}
    for fid, n in FACE_NORMALS_GODOT.items():
        d = frames.R_WORLD_G2R @ np.array(n, dtype=float)
        out[fid] = float(np.arctan2(d[1], d[0]))
    return out


def _fig_to_pil(fig):
    buf = io.BytesIO(); fig.savefig(buf, format="png", dpi=90, bbox_inches="tight"); buf.seek(0)
    im = Image.open(buf).convert("RGB"); plt.close(fig); return im


def _save_gif(frames_pil, path):
    if not frames_pil:
        return
    dur = [120] * (len(frames_pil) - 1) + [2000]
    frames_pil[0].save(path, save_all=True, append_images=frames_pil[1:], duration=dur, loop=0, optimize=True)
    print(f"  wrote {path} ({len(frames_pil)} frames)")


def position_axes(lander_xy, rows):
    """Precompute the FIXED axes + face bearings shared by every per-frame still, so the
    standalone GIF and the composite tiles use identical framing. Returns (xlim, ylim, R, bearings)."""
    txy = np.array([r["truth_xy"] for r in rows])
    pad = 8.0
    xlim = (min(txy[:, 0].min(), lander_xy[0]) - pad, max(txy[:, 0].max(), lander_xy[0]) + pad)
    ylim = (min(txy[:, 1].min(), lander_xy[1]) - pad, max(txy[:, 1].max(), lander_xy[1]) + pad)
    R = float(np.hypot(xlim[1] - xlim[0], ylim[1] - ylim[0]))
    return xlim, ylim, R, _face_bearings_ros()


def position_frame(lander_xy, rows, N, xlim, ylim, R, bearings, title):
    """One accumulating position+SLAM still (frames 0..N-1) as a PIL.Image."""
    fig, ax = plt.subplots(figsize=(6.2, 6.0))
    # quadrant wedges by visible face (bearing +/-45deg) + boundary diagonals
    for fid, b in bearings.items():
        ax.add_patch(Wedge(lander_xy, R, np.degrees(b) - 45, np.degrees(b) + 45,
                           facecolor=FACE_COLORS[fid], alpha=0.08, edgecolor="none", zorder=0))
        ax.plot([lander_xy[0], lander_xy[0] + R * np.cos(b + np.pi / 4)],
                [lander_xy[1], lander_xy[1] + R * np.sin(b + np.pi / 4)],
                color="0.7", lw=0.6, zorder=1)
    sub = rows[:N]
    tx = [r["truth_xy"][0] for r in sub]; ty = [r["truth_xy"][1] for r in sub]
    ax.plot(tx, ty, "-", color="0.35", lw=1.3, zorder=3, label="truth path")
    ax.plot(tx[-1], ty[-1], "o", color="black", ms=7, zorder=6)
    for r in sub:
        if r["est_xy"]:
            ex, ey = r["est_xy"]; t = r["truth_xy"]
            c = FACE_COLORS.get((r["faces"] or [0])[0], "0.5")
            ax.plot([t[0], ex], [t[1], ey], "-", color=c, lw=0.7, alpha=0.6, zorder=4)
            ax.plot(ex, ey, "x", color=c, ms=6, mew=1.6, zorder=5)
    ax.plot(*lander_xy, "*", color="gold", ms=22, mec="black", mew=1.0, zorder=7, label="lander (fixed)")
    ax.set_xlim(xlim); ax.set_ylim(ylim); ax.set_aspect("equal")
    last = sub[-1]
    rng = last["range_m"]
    rng_s = f"{rng:.0f} m" if rng is not None else "?"
    st = (f"DET face {last['faces']}  est_err {last['err_mm']:.0f} mm" if last["est_xy"]
          else "NO DETECTION (shadow / range / occlusion)")
    ax.set_title(f"{title}\nframe {N-1}/{len(rows)-1}  range {rng_s}  |  {st}", fontsize=9)
    ax.set_xlabel("map x (m)"); ax.set_ylabel("map y (m)")
    ax.legend(loc="upper right", fontsize=7)
    return _fig_to_pil(fig)


def build_position_gif(lander_xy, rows, out_path, title):
    xlim, ylim, R, bearings = position_axes(lander_xy, rows)
    pil = [position_frame(lander_xy, rows, N, xlim, ylim, R, bearings, title)
           for N in range(1, len(rows) + 1)]
    _save_gif(pil, out_path)


def resource_frame(Rj, N):
    """One accumulating resource still (frames 0..N-1) as a PIL.Image. Rj = parsed resource.json."""
    rec = Rj["records"]
    mem = [r["resident_mem_mb"] for r in rec]
    act = [r["qt_active_leaves"] for r in rec]; rng = [r["range_m"] for r in rec]
    fig, ax1 = plt.subplots(figsize=(6.6, 4.2)); ax2 = ax1.twinx()
    ax1.plot(rng[:N], mem[:N], "-o", color="#d62728", ms=3, label="resident 2cm corridor (MB)")
    ax1.axhline(Rj["total_2cm_GB_if_dense"] * 1000, color="0.5", ls="--", lw=1,
                label=f"dense 2cm whole patch = {Rj['total_2cm_GB_if_dense']} GB")
    ax2.plot(rng[:N], act[:N], "-s", color="#1f77b4", ms=3, label="quadtree active fine leaves")
    ax1.set_yscale("log"); ax1.set_ylim(1, Rj["total_2cm_GB_if_dense"] * 1000 * 2)
    ax1.set_xlim(min(rng) - 2, max(rng) + 2)
    ax1.set_xlabel("rover->lander range (m)")
    ax1.set_ylabel("resident map memory (MB, log)", color="#d62728")
    ax2.set_ylabel("quadtree active fine leaves", color="#1f77b4")
    ax2.set_ylim(0, max(act) * 1.3 + 1)
    ax1.set_title(f"Map resource usage (frame {N-1}/{len(rec)-1})  |  peak {max(mem):.0f} MB "
                  f"vs {Rj['total_2cm_GB_if_dense']} GB dense  -- O(corridor) not O(area)", fontsize=9)
    h1, l1 = ax1.get_legend_handles_labels(); h2, l2 = ax2.get_legend_handles_labels()
    ax1.legend(h1 + h2, l1 + l2, loc="center right", fontsize=7)
    return _fig_to_pil(fig)


def build_resource_gif(resource_path, out_path):
    Rj = json.load(open(resource_path))
    pil = [resource_frame(Rj, N) for N in range(1, len(Rj["records"]) + 1)]
    _save_gif(pil, out_path)


def _failure_fig(runs):
    """LIT vs UNLIT detection-outcome bars (stacked) -- the illumination A/B at a glance.
    Returns the matplotlib Figure (caller decides savefig vs _fig_to_pil)."""
    fig, axes = plt.subplots(2, 1, figsize=(7.0, 5.2), sharex=True)
    axes = np.atleast_1d(axes)
    for ax, (name, rows) in zip(axes, runs.items()):
        det = sum(1 for r in rows if r["est_xy"])
        ndet = len(rows) - det
        # split non-detections by range band (proxy: far = out_of_range)
        far = sum(1 for r in rows if not r["est_xy"] and (r["range_m"] or 0) > 60)
        near = ndet - far
        ax.barh([0], [det], color="#2ca02c", label=f"detected ({det})")
        ax.barh([0], [near], left=[det], color="#ff7f0e", label=f"no face / shadow ({near})")
        ax.barh([0], [far], left=[det + near], color="#7f7f7f", label=f"out of range >60m ({far})")
        ax.set_title(f"{name}: {det}/{len(rows)} localized", fontsize=10)
        ax.set_yticks([]); ax.set_xlim(0, len(rows)); ax.legend(loc="lower right", fontsize=8)
    axes[-1].set_xlabel("frames")
    fig.suptitle("AprilTag detection outcome: LIT vs UNLIT (illumination A/B)", fontsize=11)
    return fig


def build_failure_breakdown(runs, out_path):
    fig = _failure_fig(runs)
    fig.savefig(out_path, dpi=110, bbox_inches="tight"); plt.close(fig)
    print(f"  wrote {out_path}")


def failure_breakdown_pil(runs):
    """The LIT-vs-UNLIT failure breakdown as a PIL.Image (static; pasted into every composite frame)."""
    return _fig_to_pil(_failure_fig(runs))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam-root", default=os.path.join(_ROOT, "godot_sidecar", "out", "cam"))
    ap.add_argument("--scene", default=os.path.join(_ROOT, "godot_sidecar", "out", "scenes", "haworth_spiral"))
    ap.add_argument("--out-dir", default=os.path.join(_ROOT, "godot_sidecar", "out", "panels"))
    a = ap.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)
    runs = {}
    for name in ("lit", "unlit"):
        rd = os.path.join(a.cam_root, f"haworth_spiral_{name}")
        if not os.path.isdir(rd):
            print(f"(skip {name}: {rd} absent)"); continue
        lxy, rows = _load_run(rd); runs[name] = rows
        print(f"{name}: {len(rows)} frames, {sum(1 for r in rows if r['est_xy'])} detected")
        build_position_gif(lxy, rows, os.path.join(a.out_dir, f"position_slam_{name}.gif"),
                            f"Spiral: truth vs AprilTag-SLAM ({name})")
    rp = os.path.join(a.scene, "resource.json")
    if os.path.exists(rp):
        build_resource_gif(rp, os.path.join(a.out_dir, "resource.gif"))
    if runs:
        build_failure_breakdown(runs, os.path.join(a.out_dir, "failure_breakdown.png"))
