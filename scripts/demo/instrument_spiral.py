#!/usr/bin/env python3
"""Per-step pipeline-resource instrumentation for the spiral demo (the 'resource usage' panel).

Drives the demand-driven corridor LOD (TileMosaic, 2 cm fine tiles) AND the interaction-keyed
QuadtreeTracker along the SAME spiral path depart_spiral renders, and records, per frame:
  - resident_tiles      : TileMosaic.resident_count (bounded fine-tile set around the live pose)
  - resident_cells/MB   : TileMosaic.resident_memory_cells() -> bytes (5 base fields: 4x f64 + 1x u8)
  - quadtree cells-by-size + active-leaf count (cells-by-depth) from QuadtreeTracker.step
  - total_2cm_cells_if_dense : the O(area) cost the corridor AVOIDS (the headline contrast)
Writes <scene>/resource.json (a list of per-frame records) for the resource GIF panel + the
unlit-top-down quadtree overlay. Pure host python (numpy); reuses the merged Wave-2 seams.

Run: .venv/bin/python scripts/demo/instrument_spiral.py [--scene godot_sidecar/out/scenes/haworth_spiral]
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
sys.path.insert(0, os.path.join(_ROOT, "scripts", "demo"))

from terrain_authority import constants as K
from terrain_authority import dem_import, dem_io, dem_overlay, tiles_mosaic
from terrain_authority.io_fields import load_scene
from terrain_authority.quadtree import QuadtreeTracker, quadtree_pad_pow2
import spiral_path

# Spiral params MUST match depart_spiral.gd (R0_CELLS=30, R_GROWTH_CELLS=36, TURNS=5, 16/lap).
TURNS = 5
FRAMES = 80
R0_CELLS = 30.0
R_GROWTH_CELLS = 36.0
FINE_CELL_M = 0.02            # the 2 cm corridor target
ENSURE_RADIUS_M = 6.0         # fine-refine radius around the live pose
BYTES_PER_CELL = 4 * 8 + 1    # 4 float64 fields + 1 uint8 state_label (the 5 BASE_FIELD_NAMES)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default=os.path.join(_ROOT, "godot_sidecar", "out", "scenes", "haworth_spiral"))
    ap.add_argument("--radius-m", type=float, default=ENSURE_RADIUS_M)
    a = ap.parse_args()

    base, meta = load_scene(a.scene)
    H, W = base["heightmap"].shape
    grid_cell = float(meta["grid"]["cell_m"])
    wb = meta["world_bounds_m"]
    world_x0, world_y0 = float(wb["x0"]), float(wb["y0"])
    mantle_m = float(meta.get("regolith_model", {}).get("mantle_thickness_m", K.Z_T))

    # Re-derive datum + ChaSTE density (build_from_dem's datum-supply path) for the base reader.
    rho = dem_import.polar_mantle_density_fn(mantle_m).rho_bar
    height = np.asarray(base["heightmap"], dtype=np.float64)
    base_fields = {
        "mass_areal": np.full((H, W), mantle_m * rho),
        "density": np.full((H, W), rho),
        "datum": height - mantle_m,
        "state_label": np.asarray(base.get("state_label", np.zeros((H, W), np.uint8)), np.uint8),
        "disturbance": np.asarray(base.get("disturbance", np.zeros((H, W))), np.float64),
    }
    reader = dem_io.ArrayBaseReader(base_fields, base_cell_m=grid_cell,
                                    world_x0=world_x0, world_y0=world_y0)
    overlay_params = dict(dem_overlay.DEFAULT_OVERLAY_PARAMS)
    feature_fn = dem_overlay.make_crater_feature_fn(dem_effres_m=15.0, d_min_m=1.0)
    mosaic = tiles_mosaic.TileMosaic(reader, grid_cell, FINE_CELL_M,
                                     tile_base_cells=8, max_resident_tiles=16, world_seed=1234,
                                     overlay_params=overlay_params, feature_fn=feature_fn)
    k = int(mosaic.k)
    field = quadtree_pad_pow2(max(W, H))
    tracker = QuadtreeTracker(field_size=field, min_leaf=8)

    center_rc = ((H - 1) * 0.5, (W - 1) * 0.5)
    center_xy = (world_x0 + center_rc[1] * grid_cell, world_y0 + center_rc[0] * grid_cell)
    rc_seq = spiral_path.spiral_rc(center_rc, FRAMES, turns=TURNS, r0_cells=R0_CELLS,
                                   r_growth_cells=R_GROWTH_CELLS, cell_m=grid_cell)
    total_2cm_dense = (W * grid_cell / FINE_CELL_M) * (H * grid_cell / FINE_CELL_M)  # O(area) the corridor avoids

    records = []
    qt_records = []                    # per-frame leaf geometry for the --topdown-spiral overlay
    peak_tiles = 0
    for i, rc in enumerate(rc_seq):
        row, col = float(rc[0]), float(rc[1])
        rover_xy = (world_x0 + col * grid_cell, world_y0 + row * grid_cell)
        mosaic.ensure_fine(rover_xy, radius_m=a.radius_m)          # demand-refine 2cm corridor (LRU-bounded)
        qt = tracker.step((row, col))                              # interaction-keyed quadtree this step
        by_size: dict[int, int] = {}
        for node in qt.nodes:
            if node.get("leaf"):
                s = int(node["size"]); by_size[s] = by_size.get(s, 0) + 1
        resident_cells = int(mosaic.resident_memory_cells())
        peak_tiles = max(peak_tiles, mosaic.resident_count)
        rng = float(np.hypot(rover_xy[0] - center_xy[0], rover_xy[1] - center_xy[1]))
        records.append({
            "frame": i, "range_m": round(rng, 3),
            "resident_tiles": mosaic.resident_count,
            "resident_2cm_cells": resident_cells,
            "resident_mem_mb": round(resident_cells * BYTES_PER_CELL / 1e6, 3),
            "qt_active_leaves": len(qt.active_leaves),
            "qt_cells_by_size": {str(s): n for s, n in sorted(by_size.items())},
            "qt_finest_cells": int(len(qt.active_leaves) * qt.min_leaf * qt.min_leaf),
        })
        # Full per-frame leaf GEOMETRY (not just the summary counts) for the in-engine
        # top-down overlay: terrain.gd::_build_quadtree_overlay consumes exactly these
        # {level,row0,col0,size,leaf} nodes + [r0,c0,r1,c1] active_leaves + lod.min_leaf.
        qt_records.append({
            "frame": i,
            "nodes": qt.nodes,
            "active_leaves": [[int(b[0]), int(b[1]), int(b[2]), int(b[3])] for b in qt.active_leaves],
            "lod": {"min_leaf": int(qt.min_leaf), "field_size": int(qt.field_size)},
        })

    out = {
        "scene": os.path.basename(a.scene), "frames": len(records), "fine_cell_m": FINE_CELL_M,
        "base_cell_m": grid_cell, "refine_k": k, "tile_base_cells": 8, "max_resident_tiles": 16,
        "peak_resident_tiles": peak_tiles,
        "total_2cm_cells_if_dense": int(total_2cm_dense),
        "total_2cm_GB_if_dense": round(total_2cm_dense * BYTES_PER_CELL / 1e9, 2),
        "records": records,
    }
    path = os.path.join(a.scene, "resource.json")
    json.dump(out, open(path, "w"), indent=1)
    qt_path = os.path.join(a.scene, "qt_leaves.json")
    json.dump(qt_records, open(qt_path, "w"))     # fed to --topdown-spiral via --qt-leaves
    peak_mb = max(r["resident_mem_mb"] for r in records)
    print(f"instrument_spiral: {len(records)} steps -> {path}")
    print(f"  + {len(qt_records)} per-frame quadtree-leaf records -> {qt_path}")
    print(f"  peak resident: {peak_tiles} tiles, {peak_mb:.1f} MB (2cm corridor, k={k})")
    print(f"  vs dense 2cm over the whole {W*grid_cell:.0f}m patch: {out['total_2cm_GB_if_dense']} GB "
          f"({out['total_2cm_cells_if_dense']:,} cells) -- the O(area) cost the corridor avoids")
    print(f"  range {records[0]['range_m']:.1f} -> {records[-1]['range_m']:.1f} m; "
          f"qt active leaves/step ~ {np.median([r['qt_active_leaves'] for r in records]):.0f}")


if __name__ == "__main__":
    main()
