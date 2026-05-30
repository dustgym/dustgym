"""D1a terminal hex visualizer for state-field scenes.

    python -m terrain_authority.hexviz <scene_dir> [--field heightmap|state_label|disturbance]

Loads a scene via io_fields, downsamples the chosen field to ~64x32, and prints one hex
digit 0-f per cell mapped from the field's relative value, with a min/max legend. This is
the dependency-free "can I see the terrain in a terminal" check that complements the Godot
render path (INTERFACE.md §1: previews are not in the Godot hot path).
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

from .io_fields import load_scene

_HEX = "0123456789abcdef"


def _downsample(field: np.ndarray, out_w: int, out_h: int) -> np.ndarray:
    """Block-mean downsample to (out_h, out_w) by averaging row/col bins."""
    h, w = field.shape
    out_h = min(out_h, h)
    out_w = min(out_w, w)
    row_idx = (np.linspace(0, h, out_h + 1)).astype(int)
    col_idx = (np.linspace(0, w, out_w + 1)).astype(int)
    out = np.zeros((out_h, out_w), dtype=np.float64)
    for i in range(out_h):
        for j in range(out_w):
            block = field[row_idx[i]:row_idx[i + 1], col_idx[j]:col_idx[j + 1]]
            out[i, j] = block.mean() if block.size else 0.0
    return out


def hex_render(field: np.ndarray, out_w: int = 64, out_h: int = 32) -> tuple[str, float, float]:
    """Return (text, vmin, vmax). One hex digit per cell by relative value.

    Rows printed top-down with the HIGHEST world +Z row first so the terminal picture is
    visually upright (origin[0,0] is min corner per INTERFACE.md §3).
    """
    ds = _downsample(field.astype(np.float64), out_w, out_h)
    vmin = float(ds.min())
    vmax = float(ds.max())
    span = vmax - vmin
    if span <= 0:
        norm = np.zeros_like(ds)
    else:
        norm = (ds - vmin) / span
    levels = np.clip((norm * 15.0).round().astype(int), 0, 15)
    lines = []
    for i in range(levels.shape[0] - 1, -1, -1):  # flip so +Z points up on screen
        lines.append("".join(_HEX[v] for v in levels[i]))
    return "\n".join(lines), vmin, vmax


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Terminal hex visualizer for state-field scenes.")
    ap.add_argument("scene_dir", help="scene directory containing metadata.json + rasters")
    ap.add_argument("--field", default="heightmap",
                    choices=["heightmap", "state_label", "disturbance", "density", "mass_areal"],
                    help="which field to render (default heightmap)")
    ap.add_argument("--width", type=int, default=64)
    ap.add_argument("--height", type=int, default=32)
    args = ap.parse_args(argv)

    fields, meta = load_scene(args.scene_dir)
    if args.field not in fields:
        print(f"field '{args.field}' not present in {args.scene_dir}", file=sys.stderr)
        return 2

    field = fields[args.field]
    text, vmin, vmax = hex_render(field, args.width, args.height)

    units = {
        "heightmap": "m", "density": "kg/m^3", "mass_areal": "kg/m^2",
        "disturbance": "[0,1]", "state_label": "enum 0..4",
    }[args.field]
    print(f"# scene: {meta.get('scene_name','?')}  field: {args.field}  "
          f"grid {meta['grid']['width']}x{meta['grid']['height']} "
          f"cell={meta['grid']['cell_m']}m  (+Z up on screen, +X right)")
    print(text)
    print(f"# legend 0..f maps min..max:  min={vmin:.5g} {units}   max={vmax:.5g} {units}")
    if args.field == "state_label":
        print("# enum: 0 VIRGIN 1 TREAD 2 EXCAVATED 3 SPOIL 4 COMPACTED_BERM "
              "(hex digit is RELATIVE: it maps this scene's min..max label to 0..f)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
