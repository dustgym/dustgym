#!/usr/bin/env python3
"""Assemble the quadtree fly-through GIF from the Godot --sequence PNG frames.

The Godot sidecar's `--sequence` mode writes godot_sidecar/out/quadtree_flythrough_NNN.png
(one per driven frame, 1920x1080). This is the recurring portfolio upload — the one demo that
shows the whole pipeline evolving each pass. There is intentionally no ImageMagick dependency;
this mirrors the PIL gif-assembly already used by viz/quadtree_demo.py so the step is in-repo,
deterministic, and uses only the vendored pillow.

Usage (from repo root, after rendering the frames):
    .venv/bin/python scripts/assemble_flythrough_gif.py
"""
import glob
import os

from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, os.pardir, "godot_sidecar", "out")

# Downscale to half the 1080p render (matches the committed GIF size, keeps the imgur upload small).
GIF_SIZE = (960, 540)
FRAME_MS = 100          # ~10 fps playback
HOLD_LAST_MS = 1200     # pause on the final frame before looping


def main() -> None:
    pattern = os.path.join(OUT, "quadtree_flythrough_*.png")
    paths = sorted(glob.glob(pattern))
    if not paths:
        raise SystemExit(f"no frames matched {pattern} — render the --sequence first")

    frames = [Image.open(p).convert("RGB").resize(GIF_SIZE, Image.LANCZOS) for p in paths]
    durations = [FRAME_MS] * (len(frames) - 1) + [HOLD_LAST_MS]

    out_path = os.path.join(OUT, "quadtree_flythrough.gif")
    frames[0].save(out_path, save_all=True, append_images=frames[1:],
                   duration=durations, loop=0, optimize=True)
    print(f"wrote {out_path}  ({len(frames)} frames, {os.path.getsize(out_path) // 1024} KB)")


if __name__ == "__main__":
    main()
