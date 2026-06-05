# Real lunar DEM terrain at 10 km scale — feasibility & integration evaluation

*Evaluation written 2026-05-31. Grounded against the live repo (`terrain_authority/`, INTERFACE.md, `docs/lac_reimplementation_eval.md` §4.3/§8) and a 13-agent recon + web-research workflow (5 codebase readers, 5 source-verification agents, synthesis, an adversarial sourcing audit, and a revision pass). Every external claim is cited inline; numbers taken from secondary sources / abstracts rather than the primary PDF are flagged, and parameters carry the repo's `[FIXED]/[CALIB]/[UNKNOWN]/[prior-applied-to-pole]` honesty tags. Repo claims below were re-read against the live files for this doc.*

> **Scope note.** This is an architecture/feasibility evaluation, not a build. It answers: *can the foss_ipex sim be driven over a real ~10 km lunar south-polar tile, where does the data come from (and may we redistribute it under CC0), how do we fill the gap from the DEM's effective resolution down to the 2 cm sim cell with procgen whose every parameter is sourced, and what is the contracts-first build path?* It extends `lac_reimplementation_eval.md`, which already names PGDA Product 78 as the DEM plan-of-record (§4.3) and establishes the O(path) storage argument (§8).

> **Validation addendum (2026-05-31).** The Haworth Product-78 tile was downloaded and probed end-to-end, correcting several assumptions below: (1) **Haworth IS in the Product-78 5 m set** — `Haworth_final_adj_5mpp_surf.tif`, 5960×5960, **29.8 km square**, real tiepoint X0=−52900 / Y0=105400 (the provisional gazetteer center in §10 is moot). (2) **Z is height-above-sphere in metres, NOT an absolute radius** (Haworth range −1643…+2842 m) → `derive_height` consumes Z **directly, no `Z − 1737400` subtraction** (revises §4.2/§5-step-0). (3) At these magnitudes `float32` resolves ~0.3 mm, so the per-tile datum offset (§5-step-5) is good hygiene, **not** a precision necessity for this data. (4) The tile is **uncompressed classic TIFF, `float32`, single-band, PIL-readable** with `ModelPixelScale`/`ModelTiepoint`/`GeoKey` tags parseable directly — so ingest is **pure PIL + numpy, no GDAL / rasterio / `pip install`** (this environment has none, and a same-frame 10 km crop needs **no reprojection** — just a pixel-window slice, revising the `gdalwarp` recipe in §4.3). (5) **Slope ships inside Product 78** as `_slp.tif` (+ `_toterr.tif`, `_slperr.tif`), so some §6 anchors are available without Product 90. (6) PGDA README states **no license string** (confirms the US-Gov-works principle framing in §4.4); citation is Barker et al. 2021 (PSS 203, 105119) + Mazarico et al. 2011 (Icarus 211). A 10 km @ 5 m crop (2000², 16 MB, 2702 m rim-to-floor relief, 100% finite) was produced and hillshaded via the repo's own `io_fields.write_hillshade_png`.

---

## 1. Verdict

**Feasible, and it rides the existing `terrain_authority` machinery almost unchanged.** The pieces that do the hard part already exist and were verified against the live files: the mass-conserving column model with a surface-injection seam (`column_state.set_height_via_mass`), the conservation-grade resolution bridge (`refinement.refine_field`/`coarsen_field`/`k_factor`, bit-exact for any positive integer `k` including k=250), the rover-centric quadtree LOD, and the procgen primitives (`fbm`, `carve_crater`, `sample_boulders` on a Golombek rock SFD). What is genuinely **new work** is narrow and identifiable: a DEM-ingest module, a tiled/windowed storage layer (the `io_fields` path is whole-array today), a procgen-overlay hook with the conservation + anti-aliasing constraints, a crater-population generator, a coordinate-hashed seed primitive, and a horizon/illumination consumer. The headline risk is not engineering — it is **discipline**: several of the procgen-bounding parameters are mare-derived, equatorial, or transcribed-from-secondary-source, and must stay honestly tagged or the artifact violates its own "every parameter sourced" rule.

---

## 2. The core tension — this is a resolution bridge, not a bigger map

A 10 km square at the sim's `CELL_M = 0.02 m` (`scenes.py:42`) is **500,000 × 500,000 = 2.5 × 10¹¹ cells**. That is categorically impossible to hold dense:

| Layout | Cells/side | Cells | One `float32` field (disk) | 5-raster bundle (disk) | One `float64` field (RAM) | All 4 `float64` + `u8` (RAM) |
|---|---|---|---|---|---|---|
| **10 km @ 2 cm (flat)** | 500,000 | 2.5 × 10¹¹ | 1.0 TB | **4.25 TB** | 2.0 TB | **~8.0 TB** |
| 10 km @ 1 m | 10,000 | 1.0 × 10⁸ | 0.4 GB | 1.70 GB | 0.8 GB | ~3.3 GB |
| 10 km @ 5 m | 2,000 | 4.0 × 10⁶ | 16 MB | 67 MB | 32 MB | ~0.13 GB |

(`ColumnState` carries `mass_areal`/`density`/`datum`/`disturbance` as `float64` + `state_label` `uint8`; on-disk is 4×`float32` + 1×`uint8` = 17 B/cell per `io_fields`.) The flat 10 km @ 2 cm bundle is ~3,725× an 8192² tile — not a hardware problem, a **wrong-data-structure** problem.

Meanwhile the *real* data is coarse where it matters: a raw LOLA polar 5 m DEM is **~90 % interpolated**, with a median *effective* resolution of ~15 m for the Haworth/Connecting-Ridge group up to ~35 m at Amundsen Rim ([Barker et al. 2023](https://pgda.gsfc.nasa.gov/products/90); read the per-pixel `LDEM_EFFRES` layer for the actual tile — the ~15 m figure is a group median, **not** a Haworth-specific measurement). So procgen must synthesize roughly **3–4 decades** of scale (≈15–35 m down to 2 cm; a ~750× linear fan-out from the trustworthy band).

**The answer (already the `lac_reimplementation_eval.md` §8 result):** keep the whole 10 km world only as a **coarse DEM-backed base** (16 MB at 5 m — trivial, always resident), and materialize 2 cm fine tiles **only in the rover's touched corridor**. Cost scales **O(path length), not O(area²)**: ~80 MiB for a 300 m drive versus ~8 GiB uniform. The quadtree and the mass-conserving refine/coarsen operators are exactly the mechanism; this work makes them the *storage* authority, not just a render LOD.

---

## 3. What already exists and transfers (re-verified against the live files)

| Capability | Where | Verified behaviour |
|---|---|---|
| **Surface-injection seam** | `column_state.py:98-104, 126-134` | `derive_height() = datum + mass_areal/density`; `set_height_via_mass(target)` sets `mass_areal = max(target − datum, 0)·density`. This is the "author a surface, back it out to conserved mass" path a DEM rides in on. |
| **Datum-carries-topography idiom** | `scripts/build_crater_boulders_worked.py:81-88` (`_rebase_uniform_mantle`) | Lays the surface in `datum` and a **single uniform** loose mantle in `mass_areal`. Reused for the DEM backbone (with the mantle made the cm-scale loose layer — see §5). |
| **Conservation-grade LOD bridge** | `refinement.py:90-118` (`k_factor`), `:155-198` (`refine_field`), `:205+` (`coarsen_field`) | `k_factor` accepts **any positive integer k** within an IEEE-754 tolerance gate (k=250 for 5 m→2 cm, k=50 for 1 m→2 cm both pass). `refine_field` is piecewise-constant `np.repeat` block-copy; `mass_areal` is intensive so it is *copied* not divided → total mass identical. `coarsen(refine(x)) == x` is bit-exact (`tests.py:test_refine_coarsen_roundtrip`). |
| **Rover-centric quadtree** | `quadtree.py:106+` (`build_quadtree`, pow2 + multiple-of-`min_leaf` gate `:134-139`), `QuadtreeTracker` | Distance-graded promotion keeps only the rover neighbourhood dense. |
| **Procgen primitives** | `procgen.py` | `fbm` value-noise (`:55`), `carve_crater` (`:131`, mass-consistent via `set_height_via_mass`), `sample_boulders` (`:196`) inverting the Golombek area-SFD to a Poisson count distribution. |
| **Golombek rock SFD** | `constants.py:156-169` | `golombek_q(k) = 1.79 + 0.152/k`; `F_k(D) = k·exp(−q·D)`, k = total fractional area. Cited to `rock-size-freq_abstract.txt` (Golombek et al. 2003). **Model is correct as-is.** |

The decoupling that makes this additive (producer writes `mass_areal`/`density`/`datum`; height is always re-derived, never stored) is the same seam `lac_reimplementation_eval.md` §6 leans on. **No frozen contract needs to break** — the new metadata is additive (`schema_version` stays 1.0).

---

## 4. Data acquisition

### 4.1 Products
- **Heightmap basis — PGDA Product 78** (LOLA 5 m/px south-pole per-site DEM GeoTIFF, e.g. `Haworth_*.tif`). The redistributable backbone; it is the plan-of-record in `lac_reimplementation_eval.md` §4.3.
- **Per-tile statistical anchors — PGDA Product 90** COG layers: `LDRM_H` (Hurst), `LDRM_RMSD`/`LDRM_ROUGH` (RMS height deviation at 100/200/400/800/1600 m baselines), `LDRM_SPSLP` (spectral slope), `LDEM_EFFRES` (effective resolution), `LDSM` (slope), `LPSR` (PSR mask). These turn the procgen fill from eyeballed to **anchored at the actual tile**. ⚠ Product 90 native scale is 10–240 m/px — **coarser than the 5 m heightmap** — so co-register/upsample onto the heightmap grid and read the roughness anchor at the **100 m baseline where both overlap**, not at 5 m.
- **Illumination — PGDA Product 69** (avg-illumination over the 18.6-yr precession cycle, PSR & Earth-visibility masks, and nested per-pixel **horizon maps** 20 m→480 m out to 310 km). Drives PSR placement and clips realized sun elevation below the local horizon.

### 4.2 Projection & datum
- **Horizontal:** south polar stereographic, **IAU_2015:30135** — PROJ4 `+proj=stere +lat_0=-90 +lon_0=0 +k=1 +x_0=0 +y_0=0 +R=1737400 +units=m +no_defs` (the modern equivalent of the PGDA MOON_ME/DE421 frame). A 10 km tile = center ± 5000 projected metres. All three products share this frame, so they co-register trivially.
- **Vertical (must not be skipped):** LOLA Z is a **radius relative to the 1737400 m sphere**, not a metre height. Define sim elevation = `LOLA_radius − 1737400 m`, confirm the reference from the product header/README, and store the **tile-mean elevation as a local datum offset in metadata** so the on-disk `float32` heightmap keeps cm precision over km of absolute relief (a single absolute-elevation `float32` over 10 km loses 2 cm in the mantissa).

### 4.3 Crop + co-register (recipe)
```bash
# 1. Read the TRUE projected center from the header (do NOT trust the gazetteer lat/lon):
gdalinfo Haworth_surf.tif          # -> projected X0,Y0 center; verify the tile spans >= 10 km
# 2. Crop a 10 km square, continuous DEM with bilinear:
gdalwarp -t_srs '+proj=stere +lat_0=-90 +lon_0=0 +k=1 +x_0=0 +y_0=0 +R=1737400 +units=m +no_defs' \
  -te $((X0-5000)) $((Y0-5000)) $((X0+5000)) $((Y0+5000)) -tr 5 5 -r bilinear -of COG \
  Haworth_surf.tif haworth_10km_5m.tif
# 3. Categorical layers (LPSR/PSR mask) use -r near; continuous (slope, roughness) -r cubic.
# 4. Co-register the Product-90 roughness layers onto the SAME -te/-tr grid (upsamples them).
```
Ingest reads the COG via `rasterio` (or `osgeo.gdal` — mind the `libgdal.so.37` soname PyChrono::Vehicle pins per `docs/chrono_bringup_log.md`), resamples to `base_cell_m`, and writes the surface into `ColumnState.datum` via the `_rebase_uniform_mantle` idiom. **`save_scene`/`load_scene` are whole-array** (`io_fields.py:85`, no memmap/windowing) so the 10 km base must be written as a **tiled mosaic**, never one raster.

### 4.4 Licensing — split, and it gates what may be committed

| Source | Commit to CC0 repo? | Note |
|---|---|---|
| **PGDA Products 78 / 90 / 69** | **Yes (as a principle)** | NASA-GSFC US-Government works → public domain under the general US-Gov-works principle. PGDA publishes **no formal license string**, so state it as that principle, not as a published CC0 license (defensible, low legal risk for a portfolio repo). Record courtesy citations (Barker et al. 2021/2023, Mazarico et al. 2011) + dataset DOIs in `THIRD_PARTY.md`. |
| **USGS down-selected Artemis III nav grids** (doi 10.5066/P1MEQ6UK) | **Yes (explicit CC0-1.0)** | But they ship in ACC/LTM/LPS projections ≠ IAU_2015:30135 — a committed reprojected derivative must record its reprojection provenance in `THIRD_PARTY.md`. |
| **2026 Shape-from-Shading 5 m DEM** (Bertone et al. 2026, doi 10.3847/PSJ/ae5b70; Zenodo doi 10.5281/zenodo.17954508) | **Only segregated** | **CC-BY-4.0, not CC0.** Either keep reference-only (download script, nothing committed) or place under a clearly-marked CC-BY-4.0 subfolder with an attribution NOTICE. **Never relabel as CC0.** |
| **Neukum coefficient vector** (cross-checked vs MintonGroup/cratermaker) | **Numbers only, no code** | cratermaker is **GPL-3.0** (verified 2026-05-31). GPL is copyleft → **no cratermaker code may be copied** into this CC0 repo. Only the numeric coefficients are reused — uncopyrightable scientific facts, cited to Neukum/Ivanov/Hartmann 2001 by author/year. (Verified no cratermaker code is vendored/copied.) |

---

## 5. The conservation-grade DEM → ColumnState bridge

The DEM becomes mass-conserving state through the existing author-surface-then-back-out path. Six steps, in order:

0. **Vertical datum.** Convert LOLA radius → metres-above-sphere (`Z − 1737400`); confirm the product reference matches what `derive_height` assumes (a plain metre height).
1. **DEM backbone → `datum`.** After resampling to `base_cell_m`, set `datum = DEM_surface_m − mantle_thickness`, set `density` to the polar profile (§6), put the loose mantle in `mass_areal = mantle_thickness · density`. Then `derive_height() = datum + mass_areal/density` returns `DEM_surface`. **Mass-budget note:** `mantle_thickness` here is the **cm-scale loose layer** (≈ `Z_T`), *not* the 10–15 m regolith column — the datum carries everything below the loose layer. State this explicitly; the height-to-mass inversion rests on it.
2. **Procgen detail on top.** Author `target = derive_height() + detail_residual` and call `set_height_via_mass(target)`; this only re-partitions mass at fixed density/datum (conservative). `carve_crater`/`sample_boulders` already do this.
3. **Resolution-bridge invariant (load-bearing).** The normative rule (INTERFACE.md §5.3) is `base == coarsen(fine_tile)`. So the procgen residual must be **zero-mean per base cell**. **Cross-base-cell features** (a crater/boulder spanning cells) must **not** be zero-meaned independently per cell (that creates an internal-boundary step) — generate the feature on the *union* of overlapping k×k blocks, then subtract each base cell's own mean only within its sub-block. Each base-cell mean is unchanged (so `base == coarsen` survives per cell) while the feature stays continuous across the boundary.
4. **Anti-aliasing seam.** `refine_field` is piecewise-constant block-copy, so a refined base cell is a **flat k×k plateau**; adding zero-mean fbm on flat plateaus aliases at every base-cell boundary. Before adding the residual, **smoothly (and mean-preservingly) interpolate** the base across base-cell centers (bilinear/bicubic) so the low-frequency content is continuous, and ensure the fbm's lowest synthesized octave **overlaps** the DEM's highest resolved frequency (no spectral gap). The `gdalwarp` bilinear smooths the *raster*; the *in-engine* refine path is piecewise-constant, so the overlay hook must do this itself.
5. **Float32 precision over 10 km.** Per-tile local datum offset (subtract tile-mean, store in metadata) — see §4.2.
6. **Determinism / seam continuity.** A scalar `np.random.default_rng(seed)` (`procgen.py:63`) cannot make adjacent corridor tiles agree at seams or stay stable across `base_cell_m`. Replace with a **coordinate-hash seed** = `hash(global_x, global_y, octave, base_cell_class)`, with the fbm lattice indexed in the **global frame** (requires the per-tile `world_bounds_m` offsets that are 0.0 today, `scenes.py:64`). Then the same world point yields the same value regardless of which tile renders it, re-runs are bit-identical (spec §10), and 5 m vs 1 m bases agree. **This is a contract-level design resolved at L0, not internal to any one lane.**

---

## 6. Procgen bounded by sourced parameters

The binding rule applied honestly: a parameter is "sourced" only with a real primary citation; everything else is tagged `[CALIB]` (a calibration choice), `[prior-applied-to-pole]` (a global/equatorial/mare value used at the pole because no polar in-situ measurement exists), or `[UNKNOWN]`.

| Quantity | Model / value | Source | Tag |
|---|---|---|---|
| **Crater production** | Neukum polynomial `log₁₀ N_cum`; surface age committed **T = 3.5 Gyr**; valid 10 m–1000 km, **do not extrapolate below ~10 m** | Ivanov/Neukum/Hartmann 2001 (Space Sci. Rev. 96:55). Coeff vector cross-checked vs MintonGroup/cratermaker (**GPL-3.0** → numbers-as-facts only, no code; primary PDF absent so not directly verified). The "8.25e-4 vs 8.38e-4" worry was a mislabel: a0→**8.173e-4**@1 km is the production constant, 8.38e-4 is the *chronology* linear coeff, 8.25e-4 is the a10 shape coeff — a ~2.47% poly-vs-chronology gap remains, within model uncertainty (sub-10 m band is governed by the equilibrium cap) | `[CALIB]` |
| **Small-crater equilibrium cap** | `n_eq(>D) ≈ 0.084·D⁻²` /m² (~5.5 % of geometric saturation) for highland/polar; `min(production, equilibrium)` in the sub-DEM band | Xiao & Werner 2015 (JGR doi 10.1002/2015JE004860, the 1–10 % band); Minton et al. 2019 (Icarus, arXiv:1902.07746) is the **Apollo-15 mare fit `0.0336·D⁻²` = lower bound on a highland surface** | `[CALIB]` |
| **Crater synthesis cutoff** | `D_min = m · DEM_effective_px`, `m ≈ 2–3`; synthesize only **below** the DEM effective resolution; de-conflict against craters the DEM already resolves using `LDEM_EFFRES` | `LDEM_EFFRES` per-pixel layer (Barker 2023) is the sourced **input**; the 2–3× multiplier is an engineering Nyquist heuristic | `[CALIB]` |
| **Fresh crater depth/diameter** | `d/D ≈ 0.196` above 400 m; **drops to 0.11–0.17 below 400 m** (~0.13 at 20–50 m) | Repo `CRATER_DEPTH_DIAMETER_RATIO=0.2` (`constants.py:178`, Pike 1977, valid >400 m); Stöffler 2006 (RiMG 60), Stopar 2017 (Icarus) for the sub-400 m band | `[FIXED]` >400 m / `[CALIB]` below |
| **Fresh crater rim height** | `h_rim/D ≈ 0.036` → rim/depth ≈ 0.18 | Repo `CRATER_RIM_HEIGHT_FRAC=0.2` (`constants.py:182`, currently "Geometric approximation"); now sourced to Stöffler 2006 | `[FIXED]` |
| **Crater ejecta** | radial thickness `∝ (r/R)⁻³`, continuous ejecta to **2.3–2.7 R** from center | McGetchin 1973 (EPSL 20), Settle & Head 1977, Melosh 1989 | `[FIXED]` |
| **Boulder SFD** | Golombek `F_k(D)=k·exp(−q(k)·D)`, `q=1.79+0.152/k`; **make k spatial** (background `k≈0.001–0.01`, ramp to 0.05–0.40 only in fresh ejecta) | Repo `constants.py:164-169` + `sample_boulders` (**correct as-is**); Golombek & Rapp 1997 (doi 10.1029/96JE03319); Bandfield 2011 (Diviner background <1 %) | `[FIXED]` model / `[CALIB]` spatial k |
| **Boulder density / caps / clustering** | ~1800/km² Shackleton rim, ~3000 Connecting Ridge base, up to ~60,000 in clusters vs ~7 outside; cluster toward ejecta/rims/steep slopes | Bernhardt/Boazman 2022 (PSJ), Watkins 2019 (JGR), Bickel & Kring 2020 (Icarus). **Several figures from abstracts/snippets — primary PDFs unverified.** Cross-check vs **USGS LROC NAC Boulder Database v1** | `[CALIB]` |
| **Roughness Hurst (resolved band ≥~30 m)** | `H ≈ 0.95` (south pole, highland-like, **not** maria 0.76); fbm `gain = lacunarity⁻ᴴ` | Rosenburg et al. 2011 (JGR doi 10.1029/2010JE003716), Barker et al. 2025 (PSJ doi 10.3847/PSJ/adbc9d) | `[CALIB]` |
| **Roughness Hurst (cm / rover band)** | `H ≈ 0.5–0.7`; **H is scale-dependent** — a single fixed gain is wrong at one end | Helfenstein & Shepard 1999 (Icarus 141, Apollo close-up, **equatorial**) | `[prior-applied-to-pole]` |
| **Terminal RMS slope @ 2 cm** | ~20° (envelope 15–35°) at mm–cm scale | Helfenstein & Shepard 1999; Bandfield et al. 2015 (Diviner, **global**) | `[prior-applied-to-pole]` |
| **Absolute roughness anchor** | anchor on the **Product-78 `_slp.tif` slope sibling** (co-registered to `_surf.tif`, 5960², slope in degrees, median ~10.3° on this tile) at the resolved baselines; seed fbm variance so synthesized roughness-vs-baseline matches. *Downloaded + cropped to the scene window (CC0/US-Gov, same provenance as `_surf`).* Product 90 `LDRM_RMSD` at the 100 m baseline remains an optional future cross-check (not on disk). | Product 78 (Barker 2021) — **per-tile DATA, auditable** | data |
| **Polar regolith density profile** | ChaSTE two-layer: **750** kg/m³ @0–3 cm, **1300** @3–6.5 cm, **~1940** @0–10 cm avg | Durga Prasad et al. 2026 (ApJ, doi 10.3847/1538-4357/ae5228); Mathew 2025 (Sci Rep, doi 10.1038/s41598-025-91866-4). **At 69.4°S, ~20° from the pole.** | `[CALIB]` |
| **Regolith column thickness** | ~10–15 m (highland), distinct from the cm-scale `Z_T` loose-layer transition | Bart/Fa methods; site-specific bound from Product 90 | `[ASSUMPTION]` |
| **Boulder buried fraction** | repo `U(0.1, 0.7)` (`procgen.py:246`) | Ruesch & Woehler 2021 gives only a **qualitative** age-monotonic direction — **no numeric distribution exists** | `[UNKNOWN]` |
| **PSR / horizon clip** | `illum==0` → cold trap (gates optional ice, `W_ICE_MAX≤0.056`); horizon map clips realized sun below local horizon | Repo `constants.py:147-153`; Product 69 (Mazarico et al. 2011, Icarus 211) | `[FIXED]` + data |

**Repo refinements this surfaced** (each a sourced correction, not a guess):
- **fbm spectral fidelity.** Default `gain=0.5` (`procgen.py:56`) implies H=1.0 (too smooth/correlated vs target H≈0.95), *and* — more importantly — the **min-max-to-[0,1] renorm at `procgen.py:74-77`** is a realization-dependent nonlinear rescale that **destroys the PSD slope** the Hurst derivation assumes. Fixing the gain is **necessary but not sufficient**; the renorm must be replaced by a variance/deviogram-anchored scaling (`ν₀` from Product 90 `LDRM_RMSD`). Because H is scale-dependent, a single fixed gain is wrong at one end — H(baseline) must ramp.
- **Crater d/D below 400 m.** The flat `0.2` is too deep for the sub-400 m craters procgen actually adds (Stopar 2017: 0.11–0.17).
- **Crater ejecta law.** *Correction to an earlier draft:* the existing ejecta is **not** "backwards." Reading `procgen.py:172-176`, `ej_t` is **1.0 at the rim and 0 at the outer edge**, so thickness is thickest at the rim and thins outward — the correct direction. The legitimate refinement is that it uses a **quadratic ramp keyed to the outer edge** rather than the empirical McGetchin `(r/R)⁻³` law, and `CRATER_EJECTA_EXTENT_RADII=2.0` (`constants.py:186`) sits at the **low edge** of the observed 2.3–2.7 R.
- **Regolith constants.** `RHO_SURFACE=1300`/`Z_T=0.12` are the Apollo-equatorial profile; the ChaSTE polar two-layer profile differs and `Z_T` (cm loose-layer transition) is being conflated with the m-scale regolith column. **ChaSTE's ~1940 kg/m³ @10 cm does *not* "confirm" repo `RHO_DEEP=1920` @~100 cm — different depths.**

---

## 7. The new work (the honest gaps)

| New / extended | Owns | Why |
|---|---|---|
| `dem_import.py` (**new**) | DEM read, vertical-datum reconcile, resample, emit base `ColumnState` via the datum path | No DEM/GeoTIFF ingestion exists anywhere (grep-confirmed) — this is the greenfield piece. |
| `tiles_mosaic.py` + windowed/memmap reader (**new**) | global-frame tiled layout, streamed tile emission, bounded active-only refine | `io_fields` is whole-array (`io_fields.py:85`); a 10 km base cannot be one raster. |
| `procgen_seed.py` (**new**) | coordinate-hash global-frame seed | seam continuity + cross-resolution stability + spec-§10 determinism (see §5 step 6). |
| `procgen_csfd.py` (**new**) | crater-population generator (Poisson per log-D bin, `min(production, equilibrium)`, `LDEM_EFFRES` de-confliction) | no CSFD sampler exists — only single-crater `carve_crater`. |
| `refinement.py` overlay hook (**extend**) | apply procgen residual with zero-mean-per-base-cell + cross-cell handling + mean-preserving smooth interpolation | the one new conservation-critical piece (§5 steps 3–4). |
| `quadtree.py` padding helper (**extend**) | pad 10 km base to next pow2 (2000→2048 / 10000→16384); windowed `leaves_cover_field` | `leaves_cover_field` allocates a full `field_size²` raster — must be windowed on the hot path. |
| `illumination.py` (**new**) | Product 69 PSR gate + horizon clip | the flat `EL_MAX=7°` (`constants.py:46`) is an upper bound; a depression goes dark even at 5–7°. |
| `constants.py` sourced block (**extend**) | the §6 parameters with honest tags + citations | each row tagged `[FIXED]/[CALIB]/[UNKNOWN]/[prior-applied-to-pole]`. |

---

## 8. Numeric acceptance test (makes "sourced" falsifiable)

Without pass/fail criteria the "sourced" claim is unfalsifiable. A synthesized tile **passes** iff:
1. `coarsen(fine_tile) == base_tile` bit-exact (mass conservation + INTERFACE.md §5.3);
2. the synthesized deviogram `ν(dx)` at the 100 m baseline is within tolerance (e.g. ±15 %) of the Product-90 `LDRM_RMSD` value for that tile;
3. the synthesized RMS-slope-vs-baseline curve matches the Product-90 multi-baseline values where they overlap;
4. the synthesized >1 m boulder areal density is within the Bernhardt-2022 / USGS-LROC-NAC-Boulder-Database range for the region context (background vs ejecta);
5. the synthesized crater count per log-D bin ≤ the Xiao & Werner equilibrium cap.

---

## 9. Contracts-first build plan (the established parallel-worktree workflow)

**L0 — Contracts (on `main`, frozen seams, no behaviour change).** Freeze: (a) `dem_import` signature incl. vertical-datum handling; (b) additive per-tile metadata (`world_bounds_m.{x0,y0}` global offsets + local datum offset + `base_cell_m`/`fine_cell_m`/`region_rc`; `schema_version` stays 1.0); (c) the procgen-overlay hook signature incl. the zero-mean + cross-cell + mean-preserving-smoothing contract; (d) the **coordinate-hashed seed contract** (`procgen_seed`); (e) the windowed-base-reader signature; (f) the §8 acceptance-test spec. Add `CITATIONS.md` rows for every §6 source (tagged). *The seed-determinism and anti-aliasing designs are resolved here — they are contract-level, not lane-internal.*

**Wave-1 (parallel isolated worktrees, disjoint owned files; John reviews, maintainer merges):**
- **Lane A — DEM ingest + projection + datum** → `dem_import.py`, `scripts/build_from_dem.py`, `scripts/crop_lola_tile.sh`. Produces the committable CC0 Haworth 10 km @ 5 m sample tile.
- **Lane B — Sourced procgen + crater population** → `constants.py` (honest-tagged block), `procgen.py` (gain fix **and** renorm replacement), `procgen_csfd.py`. **May not label anything "sourced" until the prior/caveat tags are in.**
- **Lane C — Tiled mosaic + windowed I/O + quadtree-10km + seed** → `tiles_mosaic.py`, `dem_io.py`, `procgen_seed.py`, `quadtree.py` padding helper.

**Wave-2 — Integration + illumination (after review):** wire the overlay hook into `extract_tiles` (Lane B generators applied zero-mean + cross-cell + anti-aliased, seeded by Lane C in the global frame), add `scenes.build_from_dem`, add `illumination.py`, run the §8 acceptance test end-to-end.

---

## 10. Open decisions (for John)

1. **Region** — recommend **Haworth** (only candidate present in all four datasets: 2024 nine-region down-select, 2026 SfS, USGS CC0 nav grids, Product 78; a PSR cold-trap matching GMRO interest). Honest caveat: its low effective resolution gives the **widest synthesis band** — maximizes pipeline *visibility* but also the *synthesized-vs-measured* fraction. Demonstration tile, not highest-fidelity. Replace the gazetteer center with the `gdalinfo`-read projected X/Y before cropping.
2. **In-sim extent** — recommend **full 10 km @ 5 m coarse base + 2 cm corridor-only** (the O(path) route; flat 10 km @ 2 cm is impossible).
3. **Commit a tile vs reference-only** — recommend **commit the CC0 PGDA LOLA 10 km @ 5 m tile (~67 MB) + USGS nav grid**, keep the CC-BY SfS DEM reference-only (or segregated). Never relabel SfS as CC0; verify cratermaker's license before committing the Neukum vector.
4. **Build now vs design-only** — recommend **build L0 contracts + Wave-1 Lane A now** (turns "no DEM path exists" into a real-map scene, the `lac_reimplementation_eval.md` Phase-3 headline, lowest-risk, unblocks the rest); run Lanes B/C in parallel once L0 seams freeze; defer Wave-2 to post-review.

---

## 11. Risks

- **Streaming/windowed I/O is unavoidable new work** — until a tiled mosaic + windowed reader exist, a naïve `load_scene` of a 10 km @ 2 cm raster attempts a multi-TB allocation.
- **Vertical-datum mistake** — treating raw LOLA radius as a metre height is wrong by ~1.7×10⁶ m and breaks conservation/precision. Read the reference from the header.
- **Float32 vertical precision** — without the per-tile local datum offset, the `saved == derive_height()` invariant can fail on `float32` round-trip at 10 km scale.
- **Over-cratering / over-rocking** — Neukum below ~10 m, the mare equilibrium fit as a highland cap, or uniform Golombek k=0.05–0.40 would over-populate a polar background (Diviner <1 %). Cap at the Xiao & Werner highland band; drive k spatially from the ejecta mask.
- **Presenting derived/mare/equatorial values as "sourced"** — the central honesty risk. The equilibrium cap (mare), cm-scale H/RMS-slope (equatorial), Neukum vector (transcribed), `D_min` multiplier (heuristic), and `buried_frac` (no numeric source) **must** stay tagged on the parameter, not buried in prose.
- **GDAL/rasterio vs the PyChrono `libgdal.so.37` pin** (`chrono_bringup_log.md`) — isolate the ingest dependency or build against the same libgdal.
- **Quadtree per-frame full rebuild + monotonic touched set** — a long 10 km drive could refine the entire trail at once and rebuild a depth-14 tree every step; needs bounded active-only refine + incremental update before real long traverses.
- **Polar sub-cm statistics are extrapolations** — cm-scale Hurst, terminal RMS slope, and the equilibrium cap come from Apollo equatorial / mare data; all flagged `[prior-applied-to-pole]`/`[CALIB]`.

---

## 12. Sources & new citations needed

**Datasets:** PGDA [Product 78](https://pgda.gsfc.nasa.gov/products/78) (LOLA 5 m S-pole), [Product 90](https://pgda.gsfc.nasa.gov/products/90) (roughness/Hurst/effres COGs), [Product 69](https://pgda.gsfc.nasa.gov/products/69) (illumination/horizon); USGS down-selected Artemis III nav grids (doi 10.5066/P1MEQ6UK, CC0); 2026 SfS DEMs (Bertone et al., doi 10.3847/PSJ/ae5b70; Zenodo 10.5281/zenodo.17954508, **CC-BY-4.0**).

**New citations to add to `papers/CITATIONS.md` (tagged):** Ivanov/Neukum/Hartmann 2001 (production, *primary table unverified — via cratermaker*); Minton et al. 2019 (mare equilibrium, *lower bound*); Xiao & Werner 2015 (highland equilibrium band); Stöffler 2006, Stopar 2017 (d/D), Pike 1977 (repo, add row); McGetchin 1973 / Settle & Head 1977 / Melosh 1989 (ejecta); Golombek & Rapp 1997 (boulder SFD origin); Bandfield 2011 (Diviner background); Bernhardt/Boazman 2022, Watkins 2019, Bickel & Kring 2020 (boulder densities, *secondary-sourced*) + USGS LROC NAC Boulder DB v1 (validation); Rosenburg 2011, Barker 2025 (Hurst); Helfenstein & Shepard 1999, Bandfield 2015 (*equatorial/global priors*); Durga Prasad 2026 / Mathew 2025 (ChaSTE); Ruesch & Woehler 2021 (*qualitative only*); Mazarico 2011 (illumination).

*Caveat: per the audit, the Neukum coefficient vector, the boulder per-region densities, and the cm-scale roughness priors are the least-verified inputs and are tagged accordingly; the feasibility verdict and the code-rides-on argument were verified against the live repo and do not depend on them.*
