#!/usr/bin/env python3
"""Deliverable — INTERACTION-KEYED QUADTREE showpiece (headless matplotlib).

The headline architecture-thesis demo (spec §4: "the tree manages SPACE, not physics; LOD
and space-management are keyed to INTERACTION"). It is the upgrade of README §4 row #4 from
"quadtree visualized, not memory-managing" to the tree actually MANAGING SPACE as the rover
drives: leaves near the rover PROMOTE to the finest level (fine/active) while distant
regions stay COARSE, and the fine cluster MOVES with the rover frame by frame.

Pure CONSUMER of the FROZEN state-field contract (INTERFACE.md §1/§5/§5.1/§7): every raster
read goes through ``terrain_authority.io_fields.load_scene`` (never raw bytes), and the
per-frame quadtree is read from the OPTIONAL additive metadata keys (INTERFACE.md §5.1:
``active_leaves`` / ``quadtree_nodes`` / ``touched_leaves`` / ``rover_rc``). It reuses the
``tread_track`` driven-rover series, so the quadtree follows the SAME rover that lays the
VIRGIN->TREAD trail — one coherent story.

Each frame fuses FOUR layers (one image), in the groundtruth_viz / tread_track idiom:
  (a) QUADTREE subdivision: coarse leaf boxes drawn thin/cool far from the rover; fine
      ``active_leaves`` boxes drawn hot, clustered on the rover (the promotion following it);
      the cumulative ``touched_leaves`` trail faintly outlined behind the rover.
  (b) ROVER position/footprint: a marker + contact disc at ``rover_rc``.
  (c) VIRGIN (grey) -> TREAD (orange) state_label segmentation, as a tinted hillshade base.
  (d) the COMPACTION / height channel: the grazing-sun hillshade shows the rut depression
      (height = datum + mass/density; denser column thins, so the track sinks).

Outputs (viz/out/):
  1. quadtree_demo.gif            animated: the fine LOD cluster tracking the rover.
  2. quadtree_demo_filmstrip.png  a few key frames side by side.

Run:  /home/john/Development/foss_ipex/.venv/bin/python viz/quadtree_demo.py
"""

from __future__ import annotations

import os
import sys

import matplotlib
matplotlib.use("Agg")  # headless: write PNGs directly (INTERFACE.md preview note)
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.patches import Patch

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)
from terrain_authority.io_fields import load_scene  # noqa: E402
from terrain_authority import constants as K  # noqa: E402

# Reuse the EXACT grazing-sun hillshade + series helpers used by the cave-in / tread-track
# showpieces so all portfolio pieces are visually consistent (spec §5.1/§8).
from viz.variety_panel import hillshade_rgb, _load_series, _pick_frames  # noqa: E402

SAMPLES = os.path.join(_REPO_ROOT, "samples")
OUT = os.path.join(_REPO_ROOT, "viz", "out")

# TREAD overlay tint (warm orange), matching tread_track.py so the two demos read alike.
TREAD_RGB = np.array([0.95, 0.55, 0.18])
VERT_EXAG = 18.0  # cm-scale rut on a metre patch; exaggerate as in tread_track.py

# Quadtree wireframe palette: active=hot (the LOD following the rover), coarse=cool/thin
# (the far-field that stays coarse), touched=faint amber (the promote-only trail behind).
ACTIVE_EDGE = "#ff3b1f"
COARSE_EDGE = "#3a6ea5"
TOUCHED_EDGE = "#f0a030"
ROVER_RGB = "#19e0ff"


def _fused_rgb(fields: dict, cell_m: float) -> np.ndarray:
    """Grazing-sun hillshade of the heightmap (the rut/compaction channel) with the
    VIRGIN->TREAD state-label tinted in. (H, W, 3) float RGB, origin='lower'."""
    h = fields["heightmap"]
    sl = fields["state_label"]
    rgb = hillshade_rgb(h, cell_m, vert_exag=VERT_EXAG, cmap="bone")[..., :3].copy()
    tread = (sl == K.STATE_TREAD) | (sl == K.STATE_COMPACTED_BERM)
    if tread.any():
        w = np.clip(0.35 + 0.55 * fields["disturbance"], 0.0, 0.9)[tread][:, None]
        rgb[tread] = (1.0 - w) * rgb[tread] + w * TREAD_RGB[None, :]
    return rgb


def _box_segments(boxes, cell_m: float):
    """[r0,c0,r1,c1] half-open cell boxes -> LineCollection segments in WORLD metres.

    x = col*cell_m, z = row*cell_m (INTERFACE.md §3). Each box is its 4 edges as segments.
    """
    segs = []
    for (r0, c0, r1, c1) in boxes:
        x0, x1 = c0 * cell_m, c1 * cell_m
        z0, z1 = r0 * cell_m, r1 * cell_m
        segs.append([(x0, z0), (x1, z0)])
        segs.append([(x1, z0), (x1, z1)])
        segs.append([(x1, z1), (x0, z1)])
        segs.append([(x0, z1), (x0, z0)])
    return segs


def _draw_frame(ax, fields: dict, meta: dict, cell_m: float, width: int, height: int,
                frame_idx: int, n_frames: int) -> None:
    """Draw one fused quadtree-demo frame onto ``ax`` (shared by filmstrip + gif)."""
    rgb = _fused_rgb(fields, cell_m)
    extent = [0, width * cell_m, 0, height * cell_m]
    ax.imshow(rgb, origin="lower", extent=extent, interpolation="nearest")

    # --- quadtree layers from the OPTIONAL additive metadata (INTERFACE.md §5.1) --------
    active = meta.get("active_leaves", [])
    touched = meta.get("touched_leaves", [])
    nodes = meta.get("quadtree_nodes", [])
    coarse = [[n["row0"], n["col0"], n["row0"] + n["size"], n["col0"] + n["size"]]
              for n in nodes if n.get("leaf") and n["size"] > meta.get(
                  "quadtree_lod", {}).get("min_leaf", 8)]

    # touched trail first (faint, behind), then coarse far-field, then the hot active set.
    if touched:
        ax.add_collection(LineCollection(_box_segments(touched, cell_m),
                                         colors=TOUCHED_EDGE, linewidths=0.5, alpha=0.45))
    if coarse:
        ax.add_collection(LineCollection(_box_segments(coarse, cell_m),
                                         colors=COARSE_EDGE, linewidths=0.7, alpha=0.65))
    if active:
        ax.add_collection(LineCollection(_box_segments(active, cell_m),
                                         colors=ACTIVE_EDGE, linewidths=1.3, alpha=0.95))

    # --- rover footprint (the interaction the LOD is keyed to) --------------------------
    rover = meta.get("rover_rc")
    qlod = meta.get("quadtree_lod", {})
    if rover is not None:
        rx, rz = rover[1] * cell_m, rover[0] * cell_m
        rad_m = float(qlod.get("footprint_radius_cells", 5.5)) * cell_m
        circ = plt.Circle((rx, rz), rad_m, fill=False, edgecolor=ROVER_RGB,
                          linewidth=1.6, alpha=0.95)
        ax.add_patch(circ)
        ax.plot([rx], [rz], marker="o", color=ROVER_RGB, markersize=4,
                markeredgecolor="#003844", markeredgewidth=0.5)

    ax.set_xlim(0, width * cell_m)
    ax.set_ylim(0, height * cell_m)

    sl = fields["state_label"]
    n_tread = int(((sl == K.STATE_TREAD) | (sl == K.STATE_COMPACTED_BERM)).sum())
    if frame_idx == 0:
        cap = "t000  pristine\nno rover yet -> 1 coarse ROOT leaf\n0 TREAD cells"
    else:
        pct = 100.0 * frame_idx / (n_frames - 1)
        cap = (f"t{frame_idx:03d}  drive {pct:.0f}%\n"
               f"{len(active)} fine/active leaves on rover\n{n_tread} TREAD cells")
    ax.set_title(cap, fontsize=8.5)
    ax.set_xticks([])
    ax.set_yticks([])


def build_filmstrip(series_dir: str, out_path: str, n_frames: int = 6) -> None:
    """Horizontal filmstrip: the fine LOD cluster promoting + tracking the rover."""
    parent, frame_dirs, _ = _load_series(series_dir)
    grid = parent["grid"]
    cell_m, width, height = grid["cell_m"], grid["width"], grid["height"]
    chosen = _pick_frames(frame_dirs, n_frames)

    n = len(chosen)
    fig, axes = plt.subplots(1, n, figsize=(2.7 * n, 3.6), dpi=140)
    if n == 1:
        axes = [axes]
    fig.suptitle(
        "Interaction-keyed quadtree: the tree MANAGES SPACE keyed to the rover (spec 4). "
        "Fine/active leaves (red) promote + cluster on the rover (cyan) and MOVE with it; "
        "far regions stay coarse (blue).\nVIRGIN->TREAD trail (orange) + grazing-sun rut "
        "(height = datum + mass/density) follow the SAME drive (tread_track series).",
        fontsize=10.5, fontweight="bold", y=1.08)

    for ax, fd in zip(axes, chosen):
        fields, meta = load_scene(os.path.join(series_dir, fd))
        _draw_frame(ax, fields, meta, cell_m, width, height,
                    int(fd[1:]), len(frame_dirs))

    qlod = parent.get("quadtree_lod", {})
    legend = [
        Patch(facecolor="none", edgecolor=ACTIVE_EDGE, label="active leaf (fine/min_leaf)"),
        Patch(facecolor="none", edgecolor=COARSE_EDGE, label="coarse leaf (far-field LOD)"),
        Patch(facecolor="none", edgecolor=TOUCHED_EDGE, label="touched history (trail)"),
        plt.Line2D([0], [0], marker="o", color=ROVER_RGB, lw=0, markersize=7,
                   label="rover footprint"),
        Patch(facecolor=TREAD_RGB, edgecolor="none", label="TREAD state_label"),
    ]
    fig.legend(handles=legend, loc="lower center", ncol=5, fontsize=8.5, framealpha=0.9,
               bbox_to_anchor=(0.5, -0.03))
    fig.text(0.5, 0.015,
             f"promotion: subdivide while box-dist(rover,node) - {qlod.get('footprint_radius_cells', 5.5)}"
             f" < {qlod.get('refine_factor', 0.5)}*size  down to min_leaf="
             f"{qlod.get('min_leaf', 8)} cells (quadtree.py; INTERFACE.md 5.1, ADDITIVE/ignorable)",
             ha="center", fontsize=8, style="italic")
    fig.tight_layout(rect=[0, 0.07, 1, 0.92])
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out_path}  ({os.path.getsize(out_path) // 1024} KB)")


def build_gif(series_dir: str, out_path: str, n_frames: int = 32) -> None:
    """Animated GIF: each frame rendered with matplotlib then stacked via pillow."""
    try:
        from PIL import Image
    except Exception as exc:  # pillow optional -> skip silently
        print(f"skipping GIF (pillow unavailable: {exc})")
        return

    parent, frame_dirs, _ = _load_series(series_dir)
    grid = parent["grid"]
    cell_m, width, height = grid["cell_m"], grid["width"], grid["height"]
    chosen = _pick_frames(frame_dirs, n_frames)

    pil_frames = []
    for fd in chosen:
        fields, meta = load_scene(os.path.join(series_dir, fd))
        fig, ax = plt.subplots(figsize=(5.0, 5.4), dpi=110)
        _draw_frame(ax, fields, meta, cell_m, width, height, int(fd[1:]), len(frame_dirs))
        fig.tight_layout()
        fig.canvas.draw()
        # RGBA buffer -> RGB uint8. Deterministic (no AA randomness) so bytes are stable.
        buf = np.asarray(fig.canvas.buffer_rgba())[..., :3].copy()
        pil_frames.append(Image.fromarray(buf))
        plt.close(fig)

    durations = [150] * (len(pil_frames) - 1) + [1600]
    pil_frames[0].save(out_path, save_all=True, append_images=pil_frames[1:],
                       duration=durations, loop=0, optimize=True)
    print(f"wrote {out_path}  ({os.path.getsize(out_path) // 1024} KB)")


def main() -> None:
    os.makedirs(OUT, exist_ok=True)
    series = os.path.join(SAMPLES, "tread_track")
    build_filmstrip(series, os.path.join(OUT, "quadtree_demo_filmstrip.png"), n_frames=6)
    build_gif(series, os.path.join(OUT, "quadtree_demo.gif"), n_frames=32)


if __name__ == "__main__":
    main()
