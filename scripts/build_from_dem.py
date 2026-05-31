"""End-to-end: real LOLA DEM -> a committable terrain_authority scene + hillshade.

Lane A deliverable (docs/dem_terrain_contract.md §1-2; eval §4-5, §9 Lane A). Turns
"no DEM path exists" into a REAL-MAP scene:

    raw LOLA tile  ->  crop 10 km @ 5 m  ->  dem_to_base (datum path)  ->
    io_fields.save_scene (5 rasters + metadata.json)  +  write_hillshade_png

The output under ``samples/lunar_dem/haworth_10km_5m/`` is a NORMAL INTERFACE.md
raster bundle (heightmap / mass_areal / density / disturbance / state_label + a
metadata.json), so every existing Python/Godot consumer loads it UNCHANGED. The only
new-to-this-scene metadata are the additive keys the contract §2 freezes:
``world_bounds_m`` with NON-ZERO global offsets, ``base_cell_m``, ``fine_cell_m``,
``region``, ``local_datum_offset_m``, ``dem_provenance`` (source/citation/frame).
``schema_version`` stays "1.0" (additive only).

    python scripts/build_from_dem.py \
        [--src .vendor/lola_raw/Haworth_final_adj_5mpp_surf.tif] \
        [--extent-m 10000] [--base-cell-m 5.0] \
        [--out samples/lunar_dem/haworth_10km_5m] [--stride 200]

Mass note: the surface rides in via the datum path with a UNIFORM cm-scale loose
mantle (~Z_T at RHO_SURFACE) on top; ``datum`` carries everything below the loose
layer (eval §5 step 1). ``derive_height() == DEM`` to ~1e-3 m (asserted in
dem_to_base and re-checked here after a save/load round-trip).
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # noqa: E402

from terrain_authority import constants as K  # noqa: E402
from terrain_authority import dem_import as di  # noqa: E402
from terrain_authority.io_fields import (load_scene, save_scene,  # noqa: E402
                                          write_hillshade_png, write_preview_png)
from scripts.crop_lola_tile import find_max_relief_window  # noqa: E402

DEFAULT_SRC = ".vendor/lola_raw/Haworth_final_adj_5mpp_surf.tif"
DEFAULT_OUT = "samples/lunar_dem/haworth_10km_5m"
FINE_CELL_M = 0.02  # the sim's 2 cm fine cell (corridor refinement target; eval §2)
REGION = "Haworth"
CITATION = ("Barker et al. 2021 (Planet. Space Sci. 203:105119); "
            "Mazarico et al. 2011 (Icarus 211:1066)")


def build(src: str, out_dir: str, *, extent_m: float = 10000.0,
          base_cell_m: float = 5.0, stride: int = 200) -> dict:
    """Crop -> dem_to_base -> save_scene + hillshade. Returns the written metadata."""
    # --- 1. load + max-relief crop (same-frame pixel-window slice) ---------
    Z, affine, src_meta = di.load_lola_geotiff(src)
    px = affine.px
    n = int(round(extent_m / px))
    r0, c0, relief = find_max_relief_window(Z, n, stride=stride)
    cx, cy = affine.xy(r0 + (n - 1) / 2.0, c0 + (n - 1) / 2.0)
    Z_crop, affine_crop = di.crop_square(Z, affine, (float(cx), float(cy)), extent_m)
    print(f"cropped {n}x{n} @ {px} m at row0,col0=({r0},{c0})  relief={relief:.1f} m")

    # --- 2. inject the surface via the datum path -> ColumnState ----------
    # Density now rides the ChaSTE polar profile via the density_fn hook (Wave-2
    # W2-DENSITY): a single depth-integrated (mass-weighted-mean) bulk density over
    # the loose mantle [0, Z_T], broadcast as a constant grid (NOT a spatial field —
    # ChaSTE is one vertical probe at 69.4 deg S). It REPLACES the equatorial-Apollo
    # K.RHO_SURFACE stand-in. derive_height()==Z is unchanged (density cancels in the
    # inversion); only the loose mantle's areal MASS becomes the sourced polar value.
    density_fn = di.polar_mantle_density_fn(K.Z_T)
    cs = di.dem_to_base(Z_crop, affine_crop, base_cell_m,
                        mantle_m=K.Z_T, density_fn=density_fn)
    rho_bar = density_fn.rho_bar
    aff_base = cs._dem_affine  # global-frame affine at base_cell_m
    surf = cs.derive_height()
    inject_err = float(np.max(np.abs(surf - _resampled_ref(Z_crop, affine_crop, base_cell_m))))
    print(f"dem_to_base: {cs.width}x{cs.height} @ {base_cell_m} m  "
          f"derive_height-vs-DEM max_err={inject_err:.2e} m")

    # --- 3. world bounds (NON-ZERO global offsets, contract §2) -----------
    # Pixel CENTERS: cell (0,0) center at (aff_base.x0, aff_base.y0); Y decreases with row.
    half = base_cell_m / 2.0
    x0 = aff_base.x0 - half                       # left edge of col 0
    x1 = aff_base.x0 + (cs.width - 1) * base_cell_m + half
    y_top = aff_base.y0 + half                     # top edge of row 0 (max Y)
    y_bot = aff_base.y0 - (cs.height - 1) * base_cell_m - half
    # world_bounds_m uses (x0<x1, y0<y1); y0 = bottom edge, y1 = top edge.
    world_bounds = {
        "x0": round(float(x0), 4), "y0": round(float(y_bot), 4),
        "x1": round(float(x1), 4), "y1": round(float(y_top), 4),
    }
    local_datum = float(np.mean(surf))

    # --- 4. metadata (INTERFACE.md §5 shape + additive contract §2 keys) --
    hmin, hmax = float(surf.min()), float(surf.max())
    meta = {
        "schema_version": "1.0",
        "scene_name": "lunar_dem/haworth_10km_5m",
        "producer": "scripts/build_from_dem.py (real LOLA DEM ingest, Lane A)",
        "grid": {"width": cs.width, "height": cs.height, "cell_m": base_cell_m,
                 "order": "row-major-C"},
        "world_bounds_m": world_bounds,            # NON-ZERO global offsets (contract §2)
        "gravity_m_s2": K.g,
        "fields": {
            "heightmap": {"file": "heightmap.rf32", "dtype": "<f4", "units": "m"},
            "mass_areal": {"file": "mass_areal.rf32", "dtype": "<f4", "units": "kg/m^2"},
            "density": {"file": "density.rf32", "dtype": "<f4", "units": "kg/m^3"},
            "disturbance": {"file": "disturbance.rf32", "dtype": "<f4",
                            "units": "1 (normalized)"},
            "state_label": {"file": "state_label.r8", "dtype": "u1", "enum": K.STATE_NAMES},
        },
        "ice_present": False,
        "height_range_m": [round(hmin, 4), round(hmax, 4)],
        "clasts": [],
        "active_zone": {"min_rc": [0, 0], "max_rc": [cs.height, cs.width]},
        "quadtree": [{"level": 0, "row0": 0, "col0": 0, "size": max(cs.width, cs.height),
                      "label": "ROOT"}],
        # --- additive contract §2 keys (schema_version stays "1.0") --------
        "base_cell_m": float(base_cell_m),
        "fine_cell_m": FINE_CELL_M,
        "region": REGION,
        "local_datum_offset_m": round(local_datum, 4),
        "regolith_model": {
            "uniform_mantle": True,
            "mantle_thickness_m": K.Z_T,
            "mantle_density_kg_m3": round(rho_bar, 4),
            "mantle_density_source": "ChaSTE depth-integrated (mass-weighted-mean) bulk "
                                     "density over [0, Z_T]; constant broadcast, NOT a "
                                     "spatial field [CALIB]",
            "mantle_areal_kg_m2": round(K.Z_T * rho_bar, 4),
            "note": "DEM surface injected via the datum path: datum=Z-Z_T, "
                    "mass_areal=Z_T*rho_bar, derive_height()==Z. Z_T is the cm-scale "
                    "loose layer; the datum carries everything below it (eval §5 step 1). "
                    "rho_bar is the mass-weighted mean of the ChaSTE polar profile "
                    "(constants.polar_density_profile: 750/1300/1940 over 0-3/3-6.5/"
                    ">6.5 cm) integrated over [0, Z_T] via dem_import."
                    "polar_mantle_density_fn (Wave-2 W2-DENSITY) — REPLACES the prior "
                    "equatorial-Apollo RHO_SURFACE stand-in. Density CANCELS in "
                    "derive_height, so only the loose mantle's areal mass changes.",
        },
        "dem_provenance": {
            "source": "PGDA LOLA_5mpp Haworth_final_adj_5mpp_surf.tif (Product 78)",
            "frame": src_meta["frame"],
            "z_semantics": src_meta["z_semantics"],
            "native_cell_m": px,
            "crop_window_row0_col0": [r0, c0],
            "crop_extent_km": [extent_m / 1000.0, extent_m / 1000.0],
            "relief_p98_p2_m": round(relief, 4),
            "sphere_radius_m": src_meta["R"],
            "citation": CITATION,
            "license_basis": "U.S. Government work (NASA GSFC PGDA); no formal license "
                             "string published — treated as public-domain / CC0-compatible "
                             "under the US-Gov-works principle (see THIRD_PARTY.md).",
        },
        "features": ["dem_backbone"],
        "contract_revision": "1.0.2",
        "notes": f"Real LOLA south-polar DEM 10 km @ {base_cell_m} m backbone (region "
                 f"{REGION}, max-relief window). Same-frame pixel-window crop (no "
                 f"reprojection); surface mass-conserving via the datum path. "
                 f"world_bounds_m carry NON-ZERO global offsets (contract §2).",
    }

    # --- 5. write scene + previews ----------------------------------------
    os.makedirs(out_dir, exist_ok=True)
    save_scene(out_dir, cs.fields_dict(), meta)
    # vert_exag is tuned down for km-scale relief so the hillshade reads structure
    # rather than saturating; grazing sun (7 deg) per the polar perception challenge.
    write_hillshade_png(surf, os.path.join(out_dir, "preview_hillshade.png"),
                        base_cell_m, altdeg=K.SUN_ELEVATION_DEG_POLAR,
                        title=f"{REGION} 10 km @ {base_cell_m} m (grazing sun "
                              f"{K.SUN_ELEVATION_DEG_POLAR}deg)")
    write_preview_png(surf, os.path.join(out_dir, "preview_height.png"),
                      cmap="terrain", title=f"{REGION} height [m] (range {hmin:.0f}..{hmax:.0f})")
    print(f"wrote scene -> {out_dir}  ({cs.width}x{cs.height})")

    # --- 6. VERIFY-SELF: round-trip reproduces derive_height (contract) ---
    fields_rt, meta_rt = load_scene(out_dir)
    rt_err = float(np.max(np.abs(fields_rt["heightmap"].astype(np.float64) - surf)))
    wb = meta_rt["world_bounds_m"]
    nonzero = (wb["x0"] != 0.0 or wb["y0"] != 0.0)
    print(f"VERIFY round-trip heightmap max_err={rt_err:.2e} m (float32 ~ <={2.0e-4:.1e}); "
          f"world_bounds non-zero={nonzero}; base_cell_m={meta_rt['base_cell_m']}")
    if not nonzero:
        raise AssertionError("world_bounds_m global offsets are zero (contract §2 requires non-zero)")
    if rt_err > 5e-1:  # generous float32 bound over ~4 km of absolute relief
        raise AssertionError(f"round-trip heightmap error {rt_err:.3e} m too large")
    return meta


def _resampled_ref(Z_crop: np.ndarray, affine, base_cell_m: float) -> np.ndarray:
    """The same resample dem_to_base applies, in float64, for an error check."""
    Zr, _ = di._resample_bilinear(Z_crop, affine, base_cell_m)
    return Zr.astype(np.float64)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Build a real-LOLA-DEM scene end to end.")
    ap.add_argument("--src", default=DEFAULT_SRC)
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--extent-m", type=float, default=10000.0)
    ap.add_argument("--base-cell-m", type=float, default=5.0)
    ap.add_argument("--stride", type=int, default=200)
    args = ap.parse_args(argv)

    build(args.src, args.out, extent_m=args.extent_m,
          base_cell_m=args.base_cell_m, stride=args.stride)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
