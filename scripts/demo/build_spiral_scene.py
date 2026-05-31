#!/usr/bin/env python3
"""Build a renderable 2cm-class scene over a REAL Haworth DEM patch for the spiral demo.

Pipeline (faithful to the merged Wave-2 seams, NOT a procgen-from-scratch patch):
  1. load the committed real-LOLA Haworth scene (5 m base, samples/lunar_dem/haworth_10km_5m/).
  2. crop an NB x NB base-cell window (default 4x4 = 20 m) chosen for moderate relief, and
     re-derive datum = heightmap - mantle + the sourced ChaSTE density (dem_import.polar_mantle_density_fn),
     so the base ColumnState carries the REAL Haworth tilt + the polar density.
  3. dem_overlay.overlay_residual(base, k) -> fine fields: mean-preserving smooth-interp of the
     real base + bounded fbm_global detail + sub-DEM craters (make_crater_feature_fn), all
     conservation-correct (coarsen(fine)==base). heightmap = derive_height of the overlaid fine fields.
  4. sample a Golombek boulder field (procgen.sample_boulders) and surface-snap each clast to the
     fine heightmap (the build_crater_boulders recipe) -> metadata.clasts (the Godot clast layer).
  5. write a renderable scene dir (io_fields.save_scene + INTERFACE.md metadata).

Defaults: NB=4 (20 m), k=100 -> 400 x 400 @ 0.05 m. The depart_spiral spiral (r0=20, +70 cells/turn,
2 turns -> 160-cell radius) fits with margin AND reaches ~8 m, where the 0.15 m tag drops below the
apriltag detection floor at 1024 px -> the out-of-range failure mode appears in-patch.

Usage:  .venv/bin/python scripts/demo/build_spiral_scene.py [--out <dir>] [--nb 4] [--k 100] [--seed 1234]
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

# repo root on path (terrain_authority is a package there)
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from terrain_authority import constants as K
from terrain_authority import dem_import as di
from terrain_authority import dem_overlay as ov
from terrain_authority.column_state import ColumnState
from terrain_authority import procgen
from terrain_authority.io_fields import save_scene, write_hillshade_png


def _pick_window(H_base: np.ndarray, nb: int, *, target_relief_m: float = 2.5):
    """Pick an nb x nb base-cell window whose height peak-to-peak is closest to target_relief_m
    (a moderate ~7deg/20m tilt: visible relief, not a cliff, lander not on a steep slope)."""
    h = H_base.shape[0]
    w = H_base.shape[1]
    best = None
    best_err = 1e30
    # coarse stride scan over interior corners
    stride = max(1, (h - nb) // 60)
    for r0 in range(0, h - nb, stride):
        for c0 in range(0, w - nb, stride):
            block = H_base[r0:r0 + nb, c0:c0 + nb]
            ptp = float(block.max() - block.min())
            err = abs(ptp - target_relief_m)
            if err < best_err and np.isfinite(block).all():
                best_err = err
                best = (r0, c0, ptp)
    return best


def build(out_dir: str, *, nb: int = 4, k: int = 100, seed: int = 1234,
          boulder_k: float = 0.03, max_clasts: int = 70, scene_name: str = "haworth_spiral") -> dict:
    src = os.path.join(_ROOT, "samples", "lunar_dem", "haworth_10km_5m")
    from terrain_authority.io_fields import load_scene
    fields, meta = load_scene(src)
    H_base = np.asarray(fields["heightmap"], dtype=np.float64)
    base_cell = float(meta.get("base_cell_m", meta["grid"]["cell_m"]))
    mantle_m = float(meta.get("regolith_model", {}).get("mantle_thickness_m", K.Z_T))

    r0, c0, ptp = _pick_window(H_base, nb, target_relief_m=8.0)
    win = H_base[r0:r0 + nb, c0:c0 + nb].copy()
    fine_cell = base_cell / k
    print(f"window base[{r0}:{r0+nb}, {c0}:{c0+nb}] relief_p2p={ptp:.3f} m  base_cell={base_cell} mantle={mantle_m}")

    # --- base ColumnState over the window: real datum + sourced ChaSTE density ----
    density_fn = di.polar_mantle_density_fn(mantle_m)
    X = np.zeros((nb, nb)); Y = np.zeros((nb, nb))   # density_fn ignores X,Y (constant ChaSTE bulk)
    rho = density_fn(X, Y)
    datum = win - mantle_m
    mass_areal = np.full((nb, nb), mantle_m, dtype=np.float64) * rho
    cs_base = ColumnState(width=nb, height=nb, cell_m=base_cell,
                          mass_areal=mass_areal, density=rho, datum=datum)
    # sanity: the base derive_height reproduces the real window
    assert np.max(np.abs(cs_base.derive_height() - win)) < 1e-3

    # --- conservation-grade fine overlay (real tilt + fbm + sub-DEM craters) ------
    feat = ov.make_crater_feature_fn(dem_effres_m=15.0, d_min_m=1.0)
    fine = ov.overlay_residual(cs_base, k, 0.0, 0.0, world_seed=seed, feature_fn=feat)
    datum_f = np.asarray(fine["datum"], dtype=np.float64)
    mass_f = np.asarray(fine["mass_areal"], dtype=np.float64)
    dens_f = np.asarray(fine["density"], dtype=np.float64)
    state_f = np.asarray(fine["state_label"], dtype=np.uint8)
    dist_f = np.asarray(fine["disturbance"], dtype=np.float64)
    height_f = datum_f + mass_f / dens_f
    W = height_f.shape[1]; Hh = height_f.shape[0]
    assert (W, Hh) == (nb * k, nb * k)
    print(f"fine grid {W}x{Hh} @ {fine_cell} m  patch={W*fine_cell:.2f} m  "
          f"height[min/med/max]={height_f.min():.3f}/{np.median(height_f):.3f}/{height_f.max():.3f}  "
          f"density={float(np.unique(dens_f)[0]):.1f}")

    # --- Golombek boulder field, surface-snapped (build_crater_boulders recipe) ----
    # Sparse, TAG-SCALE+ rocks only: the demo wants a believable scatter that occasionally
    # OCCLUDES the lander tag from a rover angle, NOT a pebble carpet that buries the rover.
    # Sample only >=0.1 m rocks, keep the largest `max_clasts`, and clear a radius around the
    # lander/rover-start so the rig isn't spawned inside a boulder.
    cx_m = (W * 0.5) * fine_cell
    cz_m = (Hh * 0.5) * fine_cell
    raw = procgen.sample_boulders(W, Hh, fine_cell, k=boulder_k, seed=seed + 7,
                                  d_min_m=0.10, d_max_m=0.80)
    raw = [c for c in raw if c["radius_m"] >= 0.08
           and np.hypot(c["center_m"][0] - cx_m, c["center_m"][2] - cz_m) > 0.9]  # keep lander clear
    raw.sort(key=lambda c: c["radius_m"], reverse=True)
    raw = raw[:max_clasts]
    clasts: list[dict] = []
    for c in raw:
        x, _y, z = c["center_m"]
        col = min(W - 1, max(0, int(round(x / fine_cell))))
        row = min(Hh - 1, max(0, int(round(z / fine_cell))))
        rad = c["radius_m"]; buried = c["buried_frac"]
        c["center_m"] = [round(x, 4),
                         round(float(height_f[row, col]) + rad * (1.0 - 2.0 * buried), 4),
                         round(z, 4)]
        c["id"] = len(clasts)
        clasts.append(c)
    rsz = [c["radius_m"] for c in clasts]
    print(f"boulders: {len(clasts)} clasts (Golombek k={boulder_k}, q={K.golombek_q(boulder_k):.3f}, "
          f"radius {min(rsz):.2f}-{max(rsz):.2f} m)" if clasts else "boulders: 0")

    # --- metadata (INTERFACE.md; local frame so Godot float32 stays precise) -------
    x1 = round(W * fine_cell, 4); y1 = round(Hh * fine_cell, 4)
    out_meta = {
        "schema_version": "1.0",
        "scene_name": scene_name,
        "producer": "scripts/demo/build_spiral_scene.py (real Haworth DEM patch + Wave-2 overlay)",
        "grid": {"width": W, "height": Hh, "cell_m": fine_cell, "order": "row-major-C"},
        "world_bounds_m": {"x0": 0.0, "y0": 0.0, "x1": x1, "y1": y1},
        "gravity_m_s2": K.g,
        "fields": {
            "heightmap": {"file": "heightmap.rf32", "dtype": "<f4", "units": "m"},
            "mass_areal": {"file": "mass_areal.rf32", "dtype": "<f4", "units": "kg/m^2"},
            "density": {"file": "density.rf32", "dtype": "<f4", "units": "kg/m^3"},
            "disturbance": {"file": "disturbance.rf32", "dtype": "<f4", "units": "1 (normalized)"},
            "state_label": {"file": "state_label.r8", "dtype": "u1", "enum": K.STATE_NAMES},
        },
        "height_range_m": [round(float(height_f.min()), 4), round(float(height_f.max()), 4)],
        "clasts": clasts,
        "active_zone": {"min_rc": [0, 0], "max_rc": [Hh, W]},
        "quadtree": [{"level": 0, "row0": 0, "col0": 0, "size": max(W, Hh), "label": "ROOT"}],
        "regolith_model": {
            "uniform_mantle": True,
            "mantle_thickness_m": mantle_m,
            "mantle_areal_kg_m2": round(float(mantle_m * np.unique(dens_f)[0]), 4),
            "surface_density": round(float(np.unique(dens_f)[0]), 4),
            "source": "[CALIB] ChaSTE polar two-layer depth-integrated bulk (Durga Prasad 2026)",
        },
        "dem_provenance": {
            "source": "PGDA LOLA 5 m South-Pole DEM (Product 78), Haworth; window refined via Wave-2 overlay",
            "base_window_rc": [r0, c0, r0 + nb, c0 + nb],
            "base_cell_m": base_cell, "fine_cell_m": fine_cell, "refine_k": k,
            "citation": "Barker et al. 2021 PSS 203:105119; Mazarico et al. 2011 Icarus 211:1066",
        },
        "notes": (f"Spiral-demo scene: real Haworth {nb}x{nb} base-cell window refined k={k} "
                  f"({W}x{Hh} @ {fine_cell} m, {W*fine_cell:.0f} m patch) + conservation-grade "
                  f"fbm+crater overlay + {len(clasts)} Golombek clasts surface-snapped."),
    }

    out_fields = {
        "heightmap": height_f.astype(np.float32),
        "mass_areal": mass_f.astype(np.float32),
        "density": dens_f.astype(np.float32),
        "disturbance": dist_f.astype(np.float32),
        "state_label": state_f.astype(np.uint8),
    }
    os.makedirs(out_dir, exist_ok=True)
    save_scene(out_dir, out_fields, out_meta)
    try:
        write_hillshade_png(height_f, os.path.join(out_dir, "preview_hillshade.png"),
                            cell_m=fine_cell, altdeg=K.SUN_ELEVATION_DEG_POLAR,
                            title=f"{scene_name} ({W}x{Hh} @ {fine_cell} m)")
    except Exception as e:  # preview is non-essential
        print(f"(hillshade preview skipped: {e})")
    print(f"WROTE scene -> {out_dir}  ({W}x{Hh}, {len(clasts)} clasts)")
    return out_meta


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(_ROOT, "godot_sidecar", "out", "scenes", "haworth_spiral"))
    ap.add_argument("--nb", type=int, default=4)
    ap.add_argument("--k", type=int, default=100)
    ap.add_argument("--seed", type=int, default=1234)
    ap.add_argument("--boulder-k", type=float, default=0.03)
    ap.add_argument("--max-clasts", type=int, default=70)
    a = ap.parse_args()
    build(a.out, nb=a.nb, k=a.k, seed=a.seed, boulder_k=a.boulder_k, max_clasts=a.max_clasts)
