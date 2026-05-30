#!/usr/bin/env python3
"""Deliverable — driven-rover TREAD-TRACK showpiece (headless matplotlib).

The analogue of the cave-in showpiece (viz/variety_panel.py build_caveins_*), for the
headline "path-dependent terrain change" capability (README §4 row #3, §5 bullet 2): a
rover drives a 2-segment path across the field and lays down a compaction tread trail over
time. The producer (terrain_authority.scenes.build_tread_track) saves the trail being laid
as a TIME SERIES of full contract scenes under samples/tread_track/tNNN/; this script is a
pure consumer of the FROZEN state-field contract (INTERFACE.md §1/§5/§7) — all raster I/O
goes through io_fields.load_scene, never raw bytes.

Outputs (viz/out/):
  1. tread_track.gif         animated: the track being laid down, frame by frame.
  2. tread_track_filmstrip.png  a few key frames side by side.

Each frame is drawn TWO ways fused into one image so BOTH effects of a wheel pass read at
a glance:
  * GRAZING-SUN HILLSHADE (matplotlib LightSource, altdeg ~= 7 deg — the polar low-sun band,
    spec §5.1/§8) shows the RUT relief: the compacted column thins (height = datum +
    mass/density, mass untouched) so the track sinks slightly.
  * STATE-LABEL OVERLAY tints TREAD cells, so the VIRGIN -> TREAD segmentation along the
    wheel path is unambiguous even where the cm-scale rut would be subtle.

Run:  /home/john/Development/foss_ipex/.venv/bin/python viz/tread_track.py
"""

from __future__ import annotations

import os
import sys

import matplotlib
matplotlib.use("Agg")  # headless: write PNGs directly (INTERFACE.md preview note)
import matplotlib.pyplot as plt
import numpy as np

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)
from terrain_authority.io_fields import load_scene  # noqa: E402
from terrain_authority import constants as K  # noqa: E402

# Reuse the EXACT grazing-sun hillshade used by the cave-in showpiece so the two
# portfolio pieces are visually consistent (spec §5.1/§8).
from viz.variety_panel import hillshade_rgb, _load_series, _pick_frames  # noqa: E402

SAMPLES = os.path.join(_REPO_ROOT, "samples")
OUT = os.path.join(_REPO_ROOT, "viz", "out")

#: TREAD overlay tint (warm orange) blended onto compacted cells. Chosen to pop against the
#: cool grey "bone" hillshade of undisturbed regolith.
TREAD_RGB = np.array([0.95, 0.55, 0.18])
#: Vertical exaggeration: the rut is only cm-scale on a metre patch, invisible at a true
#: 7-deg grazing angle, so we exaggerate relief (same rationale as variety_panel).
VERT_EXAG = 18.0


def _fused_rgb(fields: dict, cell_m: float) -> np.ndarray:
    """Grazing-sun hillshade of the heightmap with the TREAD state-label tinted in.

    Returns an (H, W, 3) float RGB in [0,1], origin='lower' (row 0 at the bottom), matching
    every other imshow(origin='lower') in this repo.
    """
    h = fields["heightmap"]
    sl = fields["state_label"]
    rgb = hillshade_rgb(h, cell_m, vert_exag=VERT_EXAG, cmap="bone")[..., :3].copy()

    # Blend the TREAD tint by disturbance weight on tread cells so the freshly-worked,
    # most-disturbed part of the rut glows strongest (disturbance is the optics driver,
    # INTERFACE.md §4). COMPACTED_BERM (driving over spoil) would tint too, if present.
    tread = (sl == K.STATE_TREAD) | (sl == K.STATE_COMPACTED_BERM)
    if tread.any():
        w = np.clip(0.35 + 0.55 * fields["disturbance"], 0.0, 0.9)[tread][:, None]
        rgb[tread] = (1.0 - w) * rgb[tread] + w * TREAD_RGB[None, :]
    return rgb


def _frame_caption(fields: dict, frame_idx: int, n_frames: int) -> str:
    """Per-frame caption: how far the drive has progressed + the live track footprint."""
    sl = fields["state_label"]
    n_tread = int(((sl == K.STATE_TREAD) | (sl == K.STATE_COMPACTED_BERM)).sum())
    pct = 100.0 * frame_idx / (n_frames - 1)
    if frame_idx == 0:
        return "t000  pristine\n(pre-drive, 0 TREAD cells)"
    return f"t{frame_idx:03d}  drive {pct:.0f}%\n{n_tread} TREAD cells"


def build_tread_filmstrip(series_dir: str, out_path: str, n_frames: int = 6) -> None:
    """Horizontal filmstrip of the tread track being laid (a few key frames)."""
    parent, frame_dirs, _ = _load_series(series_dir)
    cell_m = parent["grid"]["cell_m"]
    chosen = _pick_frames(frame_dirs, n_frames)

    rgbs, caps = [], []
    for fd in chosen:
        fields, _ = load_scene(os.path.join(series_dir, fd))
        rgbs.append(_fused_rgb(fields, cell_m))
        caps.append(_frame_caption(fields, int(fd[1:]), len(frame_dirs)))

    ts = parent.get("time_series", {})
    mass_kg = ts.get("mass_conserved_kg")
    drift_kg = ts.get("mass_drift_kg", 0.0)
    if mass_kg is not None:
        mass_line = (f"mass conserved {mass_kg:.0f} kg (drift {drift_kg:.3g} kg) across the "
                     f"track  -  wheel_pass is pure compaction, not removal (spec 6)")
    else:
        mass_line = "mass conserved across the track  -  pure compaction (spec 6)"

    n = len(rgbs)
    fig, axes = plt.subplots(1, n, figsize=(2.5 * n, 3.4), dpi=140)
    if n == 1:
        axes = [axes]
    fig.suptitle(
        "Driven-rover tread track: a wheel footprint advancing along a 2-segment path lays a "
        "VIRGIN->TREAD compaction trail\n(grazing lunar sun alt=7 deg shows the rut; orange "
        "tint = TREAD state-label; path-dependent terrain change, README 4/5)",
        fontsize=10.5, fontweight="bold", y=1.05)

    for ax, rgb, cap in zip(axes, rgbs, caps):
        ax.imshow(rgb, origin="lower")
        ax.set_title(cap, fontsize=9)
        ax.set_xticks([])
        ax.set_yticks([])

    fig.text(0.5, -0.04,
             mass_line + "    |    density rises toward RHO_DEEP=1920 kg/m^3; the denser "
             "column thins so the rut sinks (height = datum + mass/density, spec 5.3/6)",
             ha="center", fontsize=8.5, style="italic")
    fig.tight_layout(rect=[0, 0.02, 1, 0.94])
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out_path}  ({os.path.getsize(out_path) // 1024} KB)")


def build_tread_gif(series_dir: str, out_path: str, n_frames: int = 32) -> None:
    """Animated GIF of the tread track being laid (optional; pillow). Denser sampling than
    the filmstrip so the wheel advancing along the path reads as motion."""
    try:
        from PIL import Image
    except Exception as exc:  # pillow optional -> skip silently
        print(f"skipping GIF (pillow unavailable: {exc})")
        return

    parent, frame_dirs, _ = _load_series(series_dir)
    cell_m = parent["grid"]["cell_m"]
    chosen = _pick_frames(frame_dirs, n_frames)

    pil_frames = []
    for fd in chosen:
        fields, _ = load_scene(os.path.join(series_dir, fd))
        rgb = _fused_rgb(fields, cell_m)
        # flipud so the saved GIF row order matches imshow(origin='lower') everywhere else.
        arr = (np.flipud(rgb) * 255).astype(np.uint8)
        pil_frames.append(Image.fromarray(arr))

    # Hold the finished track a beat longer so the laid-down trail is legible.
    durations = [130] * (len(pil_frames) - 1) + [1400]
    pil_frames[0].save(out_path, save_all=True, append_images=pil_frames[1:],
                       duration=durations, loop=0, optimize=True)
    print(f"wrote {out_path}  ({os.path.getsize(out_path) // 1024} KB)")


def main() -> None:
    os.makedirs(OUT, exist_ok=True)
    series = os.path.join(SAMPLES, "tread_track")
    build_tread_filmstrip(series, os.path.join(OUT, "tread_track_filmstrip.png"), n_frames=6)
    build_tread_gif(series, os.path.join(OUT, "tread_track.gif"), n_frames=32)


if __name__ == "__main__":
    main()
