"""Crop a PGDA LOLA 30 km raw tile to a 10 km @ 5 m window (pure PIL + numpy).

Lane A helper (docs/dem_terrain_contract.md §1; eval §4.3). Reads the raw
``.vendor/lola_raw/Haworth_final_adj_5mpp_surf.tif`` (5960x5960, 29.8 km square),
finds the MAX-RELIEF 10 km @ 5 m window (2000x2000), and emits:
  * a ``float32`` numpy array of the cropped surface (height above sphere [m]); and
  * a provenance JSON carrying the same-frame world offsets + citation.

This is a SAME-FRAME pixel-window slice — the product is already south-polar
stereographic (IAU_2015:30135), so NO reprojection happens (eval §4.3). The window
is chosen to maximise rim-to-floor relief (p98-p2), which for Haworth lands at the
top-left corner (the deepest PSR floor + crater rim). The output array + json feed
``scripts/build_from_dem.py`` and, ultimately, the committable sample scene.

    python scripts/crop_lola_tile.py \
        [--src .vendor/lola_raw/Haworth_final_adj_5mpp_surf.tif] \
        [--extent-m 10000] [--out-dir .vendor/lola_raw] [--stride 200]

The raw tile and the numpy/json outputs live under .vendor/ (gitignored); the
COMMITTABLE deliverable is produced by build_from_dem.py under samples/lunar_dem/.
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # noqa: E402

from terrain_authority import dem_import as di  # noqa: E402

DEFAULT_SRC = ".vendor/lola_raw/Haworth_final_adj_5mpp_surf.tif"
CITATION = "Barker et al. 2021 (PSS 203,105119); Mazarico et al. 2011 (Icarus 211)"


def find_max_relief_window(Z: np.ndarray, n: int, stride: int = 200) -> tuple[int, int, float]:
    """Coarse grid-search for the (row0, col0) of the n x n window of max relief.

    Relief is the robust p98-p2 spread (rim-to-floor, NoData/outlier-resistant). The
    stride trades search cost for placement granularity; the default 200 px (1 km) is
    enough to land on the deep-PSR + rim window. Returns ``(row0, col0, relief_m)``.
    NoData (NaN) windows are skipped (a 10 km crop must be fully finite).
    """
    H, W = Z.shape
    if n > H or n > W:
        raise ValueError(f"window {n} larger than raster {H}x{W}")
    best = None
    for r0 in range(0, H - n + 1, stride):
        for c0 in range(0, W - n + 1, stride):
            sub = Z[r0:r0 + n, c0:c0 + n]
            if not np.isfinite(sub).all():
                continue
            p98, p2 = np.percentile(sub, [98, 2])
            relief = float(p98 - p2)
            if best is None or relief > best[2]:
                best = (r0, c0, relief)
    if best is None:
        raise ValueError("no fully-finite window found (all candidates contain NoData)")
    return best


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Crop a LOLA raw tile to a 10 km @ 5 m window.")
    ap.add_argument("--src", default=DEFAULT_SRC)
    ap.add_argument("--extent-m", type=float, default=10000.0)
    ap.add_argument("--out-dir", default=".vendor/lola_raw")
    ap.add_argument("--stride", type=int, default=200)
    ap.add_argument("--name", default="haworth_10km_5m")
    args = ap.parse_args(argv)

    Z, affine, meta = di.load_lola_geotiff(args.src)
    px = affine.px
    n = int(round(args.extent_m / px))
    print(f"loaded {meta['shape'][0]}x{meta['shape'][1]} @ {px} m; window {n}x{n} "
          f"({args.extent_m/1000:.1f} km); searching max relief (stride {args.stride})...")

    r0, c0, relief = find_max_relief_window(Z, n, stride=args.stride)
    # Window center in world coords, then crop via the contract slice (re-derives offsets).
    cx, cy = affine.xy(r0 + (n - 1) / 2.0, c0 + (n - 1) / 2.0)
    Z_crop, affine_crop = di.crop_square(Z, affine, (float(cx), float(cy)), args.extent_m)

    os.makedirs(args.out_dir, exist_ok=True)
    npy_path = os.path.join(args.out_dir, f"{args.name}.npy")
    json_path = os.path.join(args.out_dir, f"{args.name}.json")
    np.save(npy_path, Z_crop)

    x1 = affine_crop.x0 + (n - 1) * px   # last-pixel-center X
    y1 = affine_crop.y0                   # top-row Y (first row, max Y)
    y_bottom = affine_crop.y0 - (n - 1) * px
    provenance = {
        "source": "PGDA LOLA_5mpp Haworth_final_adj_5mpp_surf.tif (Product 78)",
        "frame": meta["frame"],
        "z_semantics": meta["z_semantics"],
        "cell_m": px,
        "n": n,
        "extent_km": [args.extent_m / 1000.0, args.extent_m / 1000.0],
        "window_row0_col0": [r0, c0],
        "world_x0_m": affine_crop.x0,                  # first-pixel-center X (col 0)
        "world_y1_m": affine_crop.y0,                  # first-row Y (row 0, max Y)
        "world_x1_m": float(x1),
        "world_y0_m": float(y_bottom),                 # bottom-row Y (min Y)
        "local_datum_offset_m": float(np.mean(Z_crop)),
        "relief_p98_p2_m": relief,
        "z_min_m": float(Z_crop.min()),
        "z_max_m": float(Z_crop.max()),
        "citation": CITATION,
    }
    with open(json_path, "w") as fh:
        json.dump(provenance, fh, indent=2)

    print(f"  window row0,col0=({r0},{c0}) relief(p98-p2)={relief:.1f} m")
    print(f"  world origin (x0,y1)=({affine_crop.x0},{affine_crop.y0}) m")
    print(f"  wrote {npy_path}  ({Z_crop.shape} {Z_crop.dtype})")
    print(f"  wrote {json_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
