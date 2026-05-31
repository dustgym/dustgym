# L0 contract — real-DEM 10 km terrain + demand-driven corridor LOD

*Frozen 2026-05-31. The contracts-first seam for the DEM-terrain thrust (`lunar_dem_10km_eval.md`). Wave-1 lanes A/B/C confine to disjoint owned files and POPULATE these signatures; they do NOT restructure them. No behaviour change at L0 — skeletons import cleanly and raise `NotImplementedError` (or no-op) until their lane fills them. Schema additions are ADDITIVE; `schema_version` stays 1.0.*

## 0. Binding decisions (John, 2026-05-31)
- **Region:** Haworth (`Haworth_final_adj_5mpp_surf.tif`, validated: 5960², 29.8 km, Z = height-above-1737400 m-sphere in metres, −1643…+2842 m).
- **Extent:** full 10 km @ 5 m coarse base + 2 cm fine **generated at runtime around the rover's LIVE pose** — **NOT a precomputed trail.** The rover may explore ANY portion of the map.
- **Commit:** the CC0 PGDA crop (US-Gov-works principle) + provenance in `THIRD_PARTY.md`.
- **Build:** all three Wave-1 lanes in parallel.

The runtime/demand-driven decision has three hard consequences the contracts below enforce:
1. **Demand-driven refine** — fine tiles are materialized by `QuadtreeTracker` around the current rover cell each step, not batch-precomputed.
2. **Coordinate-hashed determinism (load-bearing)** — generation MUST be a pure function of GLOBAL coordinate, so exploring (or re-visiting) any patch yields byte-identical terrain a precompute would have produced (spec §10).
3. **Bounded/evictable resident set** — free roaming would otherwise accumulate the whole explored area at 2 cm; the resident fine set is an LRU window around the rover, paged to disk. Storage is O(resident window), not O(total explored).

## 1. Ingest — pure PIL + numpy (NO GDAL / rasterio / pip)  — Lane A
`terrain_authority/dem_import.py`
```python
def load_lola_geotiff(path) -> tuple[np.ndarray, Affine, dict]:
    """Read a PGDA LOLA *_surf.tif via PIL (mode 'F'); parse GeoTIFF tags
    33550 ModelPixelScale, 33922 ModelTiepoint, 34735/6 GeoKeys WITHOUT GDAL.
    Returns (Z float32 [m above sphere], affine, meta{px,tiepoint,R,nodata})."""
def crop_square(Z, affine, center_xy_m, extent_m) -> tuple[np.ndarray, Affine]:
    """Same-frame pixel-window slice (NO reprojection — product is already
    south-polar-stereographic). Pixel-registered (GMT): (0,0) = first-pixel CENTER."""
def dem_to_base(Z_crop, affine, base_cell_m, *, mantle_m, density_fn) -> ColumnState:
    """Resample to base_cell_m (numpy/scipy), inject the surface via the datum path:
    datum = Z - mantle_m ; mass_areal = mantle_m * density ; derive_height()==Z.
    mantle_m is the CM-SCALE loose layer (~Z_T), NOT the m-scale regolith column."""
```
Affine: `X(col)=X0+col*px ; Y(row)=Y0-row*px`. No `Z-1737400` (Z is already height-above-sphere).

## 2. Additive per-tile metadata (all lanes; `_base_metadata` extension)  — Lane C owns the writer
```jsonc
"world_bounds_m": { "x0": <global m>, "y0": <global m>, "x1":…, "y1":… },  // NON-zero global offsets
"base_cell_m": 5.0, "fine_cell_m": 0.02, "region": "Haworth",
"local_datum_offset_m": <tile-mean, hygiene>, "dem_provenance": { source, citation, frame }
```
`schema_version` stays "1.0" (additive only). `world_bounds_m.{x0,y0}` are 0.0 today (`scenes.py:64`) — this is the only change to existing scenes' metadata semantics and is backward-compatible (0.0 = origin tile).

## 3. Coordinate-hashed seed (load-bearing for explore-anywhere)  — Lane C
`terrain_authority/procgen_seed.py`
```python
def coord_seed(global_x_m, global_y_m, octave, base_cell_class, *, world_seed=0) -> int:
    """64-bit seed = stable hash of QUANTIZED global coords + octave + resolution class.
    Same world point -> same value regardless of tile/render order or base_cell_m;
    re-runs bit-identical (spec §10). Replaces fbm's scalar default_rng(seed)."""
def fbm_global(world_x0, world_y0, n, cell_m, *, H, nu0, world_seed=0) -> np.ndarray:
    """Variance/deviogram-anchored fbm (NOT the [0,1] min-max renorm) sampled on the
    GLOBAL lattice so adjacent tiles agree at seams. H may be a scale-ramp callable."""
```

## 4. Procgen-overlay hook (the one conservation-critical new piece)  — Lane C signature, Lane B generators
`terrain_authority/dem_overlay.py` (or `refinement.extract_tiles` hook)
```python
def overlay_residual(base_tile, k, world_x0, world_y0, *, params, world_seed) -> dict:
    """1) mean-preservingly SMOOTH-interpolate the base across base-cell centers
       (kills the piecewise-constant np.repeat plateau -> anti-alias);
    2) ADD bounded procgen residual (fbm_global + sub-DEM craters + boulders),
       ZERO-MEAN PER BASE CELL so coarsen(tile)==base survives (INTERFACE.md §5.3);
    3) cross-base-cell features: generate on the UNION of overlapping k×k blocks,
       subtract each base cell's own mean only within its sub-block (continuous boundary)."""
```

## 5. Demand-driven, bounded, streamed corridor  — Lane C
`terrain_authority/tiles_mosaic.py`, `terrain_authority/dem_io.py`
```python
class TileMosaic:           # global-frame tiled base; windowed/memmap read (io_fields stays frozen)
    def base_window(self, bbox) -> dict: ...        # only the active base pages in RAM
    def ensure_fine(self, rover_xy, radius_m) -> list[Tile]:   # demand-driven refine around LIVE pose
    def evict(self, keep_bbox) -> None: ...         # LRU page-out beyond the resident window
def quadtree_pad_pow2(n) -> int:                    # 2000->2048, 10000->16384 (quadtree.py helper)
```
`io_fields` .rf32 contract is FROZEN — the mosaic is a layer ON TOP (per-tile files), never a change to `save_scene`/`load_scene`.

## 6. Sourced procgen parameters  — Lane B
`terrain_authority/constants.py` (honest-tagged block) + `terrain_authority/procgen_csfd.py`.
Tags `[FIXED]/[CALIB]/[UNKNOWN]/[prior-applied-to-pole]` per `lunar_dem_10km_eval.md` §6. Lane B may NOT label anything "sourced" until the caveat tags are in. `procgen_csfd.py` = Poisson per log-D bin, `min(production@T=3.5Gyr, Xiao&Werner equilibrium)`, `LDEM_EFFRES`/slope de-confliction, calls `carve_crater`. fbm: fix gain AND replace the `procgen.py:74-77` [0,1] renorm with variance-anchored scaling.

## 7. Numeric acceptance test (makes "sourced" falsifiable)  — Wave-2
`coarsen(fine)==base` bit-exact; deviogram@100 m within ±15 % of the DEM `_slp`/Product-90 anchor; RMS-slope-vs-baseline match; >1 m boulder density in range; crater count/log-D ≤ equilibrium cap.

## 8. Wave-2 integration seams (frozen 2026-05-31)
*The Wave-1 generators exist but are disconnected; Wave-2 is the adapter glue + the acceptance harness. Two confirmed signature MISMATCHES and one frozen-format trap make this real adapter code, not just calls. Lanes own DISJOINT files; `constants.py` is single-owner (W2-ILLUM) to avoid a merge conflict. The acceptance anchor `_slp.tif` is fetched + co-registration-verified (the `_surf` re-crop at the scene window is byte-identical to the committed heightmap; the same window on `_slp` gives the slope reference: median 16.4°, RMS 16.8° at 5 m over the scene window).*

- **W2-CRATERS** — `terrain_authority/dem_overlay.py`:
  `def make_crater_feature_fn(*, dem_effres_m, d_min_m=1.0, age_gyr=K.NEUKUM_SURFACE_AGE_GYR, base_cell_class=0) -> Callable` returning the frozen `feature_fn(residual_h, world_x0, world_y0, cell_m, *, params, world_seed)->array`. Bridges the mismatch: `populate_craters` carves IN-PLACE on a `ColumnState` (`procgen_csfd.py:69`) vs `feature_fn` takes a residual array — wrap `residual_h` as a transient `ColumnState`, derive a per-tile seed via `coord_seed(world_x0,world_y0,octave,base_cell_class,world_seed)` (explore-anywhere determinism, §3), call `populate_craters`, return carved height. Does NOT zero-mean (the existing `_apply_feature_hook` does, AFTER). Imports `procgen_csfd`/`procgen_seed` read-only.
- **W2-DENSITY** — `terrain_authority/dem_import.py` + `scripts/build_from_dem.py`:
  `def polar_mantle_density_fn(mantle_m=K.Z_T) -> Callable[[X,Y],array]` — resolves the axis mismatch (`dem_to_base` calls `density_fn(X,Y)` world-coords; `polar_density_profile(depth_m)` takes DEPTH). Returns a closure yielding the **depth-integrated ChaSTE bulk density over [0, mantle_m]** as a constant grid (it is a single mass-weighted-mean scalar broadcast, NOT a spatial field — name it honestly). `dem_to_base` signature UNCHANGED (the `density_fn` hook is filled). Wire into `build_from_dem.py` replacing hardcoded `density=K.RHO_SURFACE`.
  *Note:* `derive_height()==Z` holds for ANY density (it cancels: `datum=Z-mantle_m`, `mass=mantle_m*ρ`, `height=datum+mass/ρ=Z`) — so the acceptance check on density is the **range assertion** `ρ∈[RHO_SURFACE_POLAR, RHO_BULK_POLAR_10CM]`, not the height round-trip.
- **W2-ILLUM** — `terrain_authority/illumination.py` (NEW) + `terrain_authority/constants.py` (additive):
  `def horizon_clip(heightmap, cell_m, sun_az_deg, sun_el_deg) -> bool-mask` (per-pixel local-horizon ray-march along the sun azimuth — replaces the flat-plane `elev>0` stand-in) AND `def psr_gate(illuminated_mask, *, t_psr_k=K.T_PSR_K) -> mask` (finally CONSUMES the dead `T_PSR_K=110.0`). Self-tagged **terrain-derived horizon, NOT a Product-69 ingest** (no Product-69 reader/data on disk). Also feeds the demo's per-face shadow attribution (§4 of `demo_spiral_contract.md`).
- **W2-VARIANCE** — `terrain_authority/dem_stats.py` (NEW) + `scripts/dem_acceptance.py` (NEW):
  `def deviogram(field, cell_m, baselines_m) -> dict` + `def rms_slope_vs_baseline(field, cell_m, baselines_m) -> dict` (no such measurement exists today). **Anchor = the real `_slp.tif`**: crop `.vendor/lola_raw/Haworth_final_adj_5mpp_slp.tif` to the committed scene window (use `dem_import.crop_square` with the metadata `world_bounds_m` center+extent — verified co-registered), measure at the resolved/100 m baselines, **commit a compact anchor JSON** (not the 16 MB raster). `dem_acceptance.py` runs the falsifiable test (§7) with explicit per-criterion pass/fail booleans; calibrates `fbm_nu0` and PASSES it to `build_from_dem` (does NOT edit `dem_overlay.py:58` — lane-confine).
- **W2-SCENES (serial join, after the 4 merge)** — `terrain_authority/scenes.py`:
  `def build_from_dem(scene_dir, *, region='haworth', radius_m=30.0, with_craters=True, fbm_nu0=None, world_seed=0) -> (fields, meta)`. Connects the disconnected corridor stack to the builder (scenes.py has zero DEM imports today). **Datum re-supply trap:** `load_scene` omits `datum` (frozen `io_fields._FIELD_SPEC`) but `ArrayBaseReader` requires it (`dem_io.py:78`) — re-derive `datum = heightmap - mantle_m` (in-RAM at 2000²) or `write_base_rasters`+`MemmapBaseReader` (for the streaming 10 km base). Forward `feature_fn`=W2-CRATERS + calibrated `fbm_nu0`. Hook into `main()` alongside the 9 legacy builders WITHOUT touching them. Writes non-zero `world_bounds_m` via `tiles_mosaic.write_dem_base_metadata`. Runs the §7 acceptance test end-to-end.
