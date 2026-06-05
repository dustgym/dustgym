---
title: "Render fidelity"
nav_order: 9
---

# Render-fidelity & variable-resolution spec — per-wheel tracks, drum teeth marks, 1 cm experiments

*Status: DESIGN SPEC (not yet implemented). Written 2026-05-30. Extends [`lac_reimplementation_eval.md`](lac_reimplementation_eval.md) §8 (scaling) and the [`INTERFACE.md`](../INTERFACE.md) contract (the metadata additions land there as v1.0.2 §5.2/§5.3). Producer/consumer code is specified here, not built.*

> **Goal in one line:** resolve **four separate wheel tracks** and **drum/excavation teeth marks** to photo-fidelity, by (a) optionally refining the terrain grid to **~1 cm in the active corridor** (toggleable off for speed), (b) carrying a little **extra metadata** that orients track/teeth detail, and (c) **faking sub-cm texture in shaders** (detail normals + parallax + anti-aliasing) rather than simulating it.

---

## 0. Scope & non-goals

**In scope:** a variable-resolution terrain grid keyed to the existing interaction quadtree; an additive metadata contract for per-wheel tracks and drum dig swaths; a Godot Forward+ detail-shading pipeline; the mass-conservation invariants the new operators must hold; acceptance tests; a phased build order.

**Non-goals:** simulating sub-cm geometry (grouser cleats, individual teeth) in the heightfield or in Chrono SCM — that resolution is meaningless to the physics and is handled entirely by shading. Bit-exact match to LAC's (undocumented) regolith model — we match behaviourally (see eval §2). Changing any REQUIRED raster, dtype, endianness, or existing metadata key — every change here is **additive and optional**; a v1.0/v1.0.1 consumer renders exactly as today.

**Portfolio framing:** photoreal detail is the **sensor-model** layer (what the LAC mono-camera sees). It *coexists with* — does not replace — the false-color/state explanatory layers (`falsecolor_height`/`falsecolor_state`). Pipeline-visibility stays the headline; this adds "…and it also produces a believable camera frame."

---

## 1. The resolution picture (why this is the right lever)

Measured against the live `ColumnState`/quadtree:

- Field today: **256×256 @ 2 cm = 5.12 m**; quadtree has 5 levels, finest leaf 16 cm — but **16 cm is LOD granularity, not the resolution floor**; the storage grid is uniform **2 cm** (each finest leaf = 8×8 = 64 columns).
- Render mesh already oversamples: `ACTIVE_VERTS_PER_SIDE=192` over the ~1.6 m rover window ≈ **8.4 mm/vertex** (≈2.4× finer than the 2 cm data). So the mesh can already *carry* sub-cm shading; what it lacks is (a) finer *data* in the corridor and (b) detail normals.
- Rover track geometry (from `sidecar.gd` `WHEEL_ORIGINS`): **track gauge 0.57 m, wheelbase 0.40 m, wheel radius ~0.18 m, contact band ~0.18 m wide.**

| feature | size | cells @ 2 cm | cells @ 1 cm | verdict |
|---|---|---|---|---|
| L↔R track separation | 0.57 m | 28 | 57 | separable at 2 cm already |
| single wheel band | ~0.18 m | 9 | 18 | band resolved at 2 cm; **crisp at 1 cm** |
| rut cross-section detail | ~2–5 cm | 1–2 | 2–5 | needs **1 cm** to read as a profile |
| grouser cleat pitch | ~1–3 cm | <1 | 1–3 | **shader only** (below useful grid) |
| drum teeth pitch | ~1–3 cm | <1 | 1–3 | **shader only** |

**Conclusion:** two/four tracks resolve at 2 cm; *crisp* tracks + rut profiles want **1 cm in the corridor**; cleats/teeth are **always shader detail**, never grid geometry.

---

## 2. Variable-resolution grid

### 2.1 Two modes
- **Mode A — global `cell_m` override (already supported).** Author a whole scene at `cell_m=0.01` (or 0.005). No contract change. This is the simplest way to *experiment with 1 cm*; cost is uniform and quadratic (a 5.12 m patch at 1 cm = 512² = 8.6 MB RAM vs 2.1 MB at 2 cm; 0.5 cm = 34.6 MB). Use for small experiment patches.
- **Mode B — corridor refinement (new).** Keep a coarse base over the whole field; store/render **fine tiles only where the rover has interacted** (the quadtree `active`/`touched` set). Cost scales with **path length, not area** (eval §8): a 1.6 m active window at 1 cm = 160² = 25,600 cells = **0.85 MB**; at 0.5 cm = **3.4 MB**. Use for mission-scale drives.

### 2.2 The refinement policy + speed toggle
A single config block (mirrored into `metadata.json` §5.3) drives both producer and consumer:

```jsonc
"refinement": {
  "enabled": true,            // false -> uniform base_cell_m everywhere (current behaviour, fast)
  "base_cell_m": 0.02,        // coarse base resolution
  "fine_cell_m": 0.01,        // active-corridor resolution (experiment knob; 0.01 / 0.005)
  "refine_where": "touched",  // "active" (live window only) | "touched" (whole trail) | "none"
  "fine_min_leaf": 4          // quadtree may subdivide past the base min_leaf to mark fine tiles
}
```

- **`enabled:false`** is the speed escape hatch: producer emits only the base rasters at `base_cell_m`, consumer renders uniform — identical to v1.0. **This MUST be a no-op-equivalent path.**
- **`refine_where:"active"`** refines only the live window (cheapest, but the trail behind the rover coarsens after it leaves — fine for "watch it drive," loses the persistent crisp trail). **`"touched"`** keeps the whole driven corridor fine (the photo-fidelity trail; bounded by corridor area, saturates over repeat passes — eval §8).
- **"a few extra quadtree levels"** is exactly `fine_min_leaf < min_leaf`: the interaction quadtree subdivides past today's `min_leaf=8` (16 cm) down to e.g. `4` (8 cm) or backs the leaf with a finer-than-base tile. The quadtree's job stays *space management*; the new part is that a refined leaf is **stored at `fine_cell_m`**.

### 2.3 Storage layout (additive)
Base rasters are unchanged and always present (so old consumers work). Refinement adds an OPTIONAL `tiles/` subdir; each tile is a normal INTERFACE.md raster bundle at `fine_cell_m`, covering a base-cell-aligned region:

```
samples/<scene>/<tNNN>/
  metadata.json
  heightmap.rf32 …            # BASE rasters @ base_cell_m  (REQUIRED, unchanged)
  tiles/                      # OPTIONAL refinement
    tile_0007/
      heightmap.rf32 …        # same 5 rasters, @ fine_cell_m
```
described by §5.3 `tiles[]`. A consumer that ignores `tiles/` renders the base (coarser in the corridor, still correct).

### 2.4 Mass conservation across resolution (THE invariant — must not break)
`mass_areal` is the conserved quantity (INTERFACE.md §4). `datum` here is the per-cell elevation offset already present in `ColumnState` (`derive_height() = datum + mass_areal/density`); it defaults to 0 and is **not** a separate on-disk raster — the `heightmap` raster already bakes it in, so the frozen §4 form `height = mass_areal/density` is just the `datum=0` case. The refine factor **`k = base_cell_m / fine_cell_m` MUST be a positive integer**; a tile is a base-cell-aligned k×k block. Both operators below must be mass-exact and preserve `height = datum + mass_areal/density`:

- **refine** (coarse cell → k×k fine cells): copy `mass_areal`, `density`, `datum`, `state_label`, `disturbance` to each child verbatim (piecewise-constant). `mass_areal` is *intensive* (kg/m²) so it is **copied, not divided**; total mass `Σ mass_areal·cell_area` is identical (each child `cell_area` is `1/k²` of the parent, ×k² children) and `height` is unchanged. **Drift = 0** (verified bit-exact in exact-rational arithmetic).
- **coarsen** (k×k equal-area fine cells → one coarse cell) — each rule chosen so total mass *and* the area-mean surface height are exact:
  - `mass_areal_coarse = mean(mass_areal_fine)` — conserves total mass exactly (area-weighted mean = simple mean for equal-area children).
  - `density_coarse = mass_areal_coarse / mean(thickness_fine)`, `thickness = mass_areal/density`. Expanded: `mean(mass_areal_fine) / mean(mass_areal_fine/density_fine)` = the **mass-weighted harmonic mean** of the child densities, chosen precisely so `mass_areal_coarse/density_coarse = mean(thickness_fine)` ⇒ coarse height = area-mean of child heights. **Do NOT** read it as `Σ(mass·ρ)/Σmass` — that would violate the height invariant.
  - **Zero-mass branch:** if `mean(thickness_fine)==0` (all children empty ⇒ `mass_areal_coarse==0`), set `density_coarse = mean(density_fine)` to avoid `0/0`; then `height_coarse = datum_coarse`. Precondition: `density_fine > 0` everywhere (VIRGIN cells carry the regolith bulk density, never 0).
  - `datum_coarse = mean(datum_fine)` — **required** so area-mean-height holds for non-uniform datum.
  - `state_label_coarse` = highest-priority child label, priority **EXCAVATED > SPOIL > COMPACTED_BERM > TREAD > VIRGIN** ("most-worked / most-salient wins" — built structures and excavation evidence outrank a plain rut). A total order ⇒ deterministic, tie-free, associative across multi-level coarsening (verified by brute force over all 5⁴ child combinations). **Not** statistical mode (ill-defined on ties, and it can erase excavation evidence).
  - `disturbance_coarse = mean(disturbance_fine)` — intentional areal average, in-range, round-trips trivially since refine copies. (`max` is the alternative if a shader under-reads worked-ness downstream; we default to mean.)
- **base↔tile consistency invariant:** for any base cell overlapped by a tile, the base raster value **equals `coarsen()` of that tile's fine cells**. The producer keeps them consistent; tests assert it. (Equivalently: the base raster is always a valid mass-conserving down-sample of the finest data present.)

> Implementation note: keep the working grid in float64 (as `ColumnState` already does) and only down-cast `<f4` at save, so coarsen/refine round-trips are not polluted by float32 quantization (the existing on-disk ~1e-7 caveat still applies to storage, not to the in-memory invariant).

### 2.5 Memory & perf
| config | corridor | RAM (in-mem, 33 B/cell) |
|---|---|---|
| uniform 2 cm, 5.12 m | — | 2.1 MB |
| uniform 1 cm, 5.12 m (Mode A) | — | 8.6 MB |
| base 2 cm + 1 cm active window (Mode B) | 1.6 m | base + **0.85 MB** |
| base 2 cm + 0.5 cm active window | 1.6 m | base + **3.4 MB** |
| 300 m multi-trip, base 8 cm + 1 cm touched band (~3 m) | 300 m | ~**80 MB** (vs ~8 GB uniform-fine) |

The **toggle** (`enabled:false` or `refine_where:"none"`) is the "selectively disable for execution speed" path — it drops straight back to uniform base resolution.

---

## 3. Metadata extension (INTERFACE.md v1.0.2 — additive, optional)

Normative schema lands in INTERFACE.md §5.2 (tracks/marks) and §5.3 (refinement/tiles). Summary of the new OPTIONAL per-frame keys:

```jsonc
"wheel_tracks": {                         // four wheels, this frame
  "LF": { "points": [[r,c], ...], "heading_rad": 1.57, "slip": 0.05, "width_m": 0.18 },
  "RF": { ... }, "LB": { ... }, "RB": { ... }
},
"drum_marks": [                           // zero or more active drums, this frame
  { "drum": "front", "swath": [[r,c], ...], "depth_m": 0.03,
    "width_m": 0.20, "teeth_count": 8, "teeth_pitch_m": 0.025, "phase": 0.0 }
],
"refinement": { ... },                    // §2.2 block (also on the parent metadata)
"tiles": [ { "id": 7, "region_rc": [r0,c0,r1,c1], "cell_m": 0.01, "dir": "tiles/tile_0007" } ]
```

- `points`/`swath` are `[row,col]` in **base** cells, same indexing as rasters (§2/§3). `heading_rad` is travel direction in field space (0 = +col/+X, +π/2 = +row/+Z) — it orients the cleat/teeth pattern. `slip` ∈ [0,1] modulates smearing. The renderer derives a **track-direction + phase field** from these (consumer-side; not a new raster).
- All keys OPTIONAL; consumers MAY ignore → behaviour identical to v1.0.1.

---

## 4. Shader detail pipeline (Godot 4.6 Forward+)

Three additions, cheapest first. **Headless rendering** uses `xvfb-run -a -s "-screen 0 1024x768x24" <godot> --rendering-driver vulkan --rendering-method forward_plus` (NOT `--headless`, which expands to the headless display driver and disables *all* rendering). This requires a **real Vulkan-capable GPU on the render host** (verified here: RTX 4090, `/dev/dri/renderD128`); on a GPU-less CI runner Forward+ Vulkan falls back to a software ICD (lavapipe), which is non-conformant and may crash — so renders must run on the GPU box. All techniques below are confirmed against the Godot 4.6 docs and headless-renderable on this host.

### 4.1 Anti-aliasing (the cheapest single win — no metadata, no data change)
`project.godot` currently sets **no AA**. In the INI file these go under the existing `[rendering]` section with the `rendering/` prefix dropped (the prefix = the section name):
```ini
[rendering]
anti_aliasing/quality/msaa_3d=2          ; 0/1/2/3 = off/2x/4x/8x  ->  2 = 4x MSAA
anti_aliasing/quality/screen_space_aa=2  ; 0/1/2 = off/FXAA/SMAA  ->  SMAA (added 4.4) is sharper than FXAA for still sensor frames; FXAA(1) blurs
anti_aliasing/quality/use_taa=false      ; TAA needs motion vectors + multi-frame convergence -> sequences ONLY, useless for single stills (Forward+ only)
scaling_3d/mode=0                        ; Bilinear — REQUIRED for scale>1.0 supersampling
scaling_3d/scale=1.5                     ; 1.5 = 2.25x SSAA. Bilinear-mode only; do NOT combine with FSR1/FSR2 (FSR2 is a temporal upscaler, same multi-frame caveat as TAA)
```
(Or in code: `ProjectSettings.set_setting("rendering/anti_aliasing/quality/msaa_3d", 2)` …) MSAA 4× + SMAA + 1.5× render-scale supersampling are all **single-still-frame-safe** and directly tame the grazing-sun hard-shadow stair-stepping that dominates the current look. **Do this first.**

### 4.2 `terrain.gdshader` detail pass (extends the existing fragment shader)
The current shader outputs `ALBEDO`/`ROUGHNESS` only, keyed by `state_label`/`disturbance`/`density`. Add:
1. **`NORMAL` / `NORMAL_MAP`** output (the shader is currently flat-normal). 
2. **Base regolith granularity:** a tiling/triplanar high-frequency detail-normal so the surface isn't glassy at the 8 mm mesh scale, strength scaled by `disturbance`.
3. **Per-wheel cleat marks** where `state_label==TREAD`: a transverse-ridge normal (and optional shallow POM) whose orientation = the local **track heading** sampled from the baked track-direction field (§3), periodic at the grouser pitch, strength × `disturbance`.
4. **Drum teeth marks** where `state_label∈{EXCAVATED,SPOIL}`: a periodic scoop/teeth normal + **parallax occlusion mapping** for apparent depth, oriented by the drum-swath travel direction, period = `teeth_pitch_m`, phase = `phase`.

> **POM in Godot 4.6:** the *built-in* POM is `StandardMaterial3D.heightmap_deep_parallax` (multi-step raymarch: `heightmap_enabled` + `heightmap_deep_parallax`, `heightmap_scale`, `heightmap_min_layers`/`max_layers`). A **custom** `terrain.gdshader` has no POM keyword — port the layered raymarch by hand; cost grows with layer count, so cap layers and gate POM to near tiles.

Detail strength fades with camera distance (mip/`UV` scaling) so far tiles don't shimmer.

### 4.3 The baked track-direction + phase field (consumer-side)
The GDScript loader (`state_fields.gd`) bakes `wheel_tracks`/`drum_marks` into a small RG(+B) texture over the active window: **R,G = unit travel direction, B = phase/accumulator**. The shader samples it to orient and phase the cleat/teeth patterns. This keeps the *contract* additive-metadata-only — no new on-disk raster; the direction field is a derived render asset.

### 4.4 Alternative: decals
For sparse, sharp marks (a single deep teeth gouge), Godot `Decal` nodes stamped along the swath are an option instead of in-shader POM. In-shader is preferred for the continuous track corridor (no decal-count blowup); decals are a fallback for hero excavation marks. **Constraints:** `Decal` uses **fixed** rendering — no custom shader, only albedo/normal/ORM/emission slots, Forward+/Mobile only — and decals share the Forward+ **512 clustered-element budget** with lights and reflection probes, so bound the per-frame decal count.

---

## 5. Producer-side changes (specified, not built)

- **`rover.py` → 4-wheel stamping.** `wheel_pass` currently sweeps **one** disc footprint. Replace with per-wheel contact: from rover pose (center + heading) compute the 4 ground-contact points via `WHEEL_ORIGINS` rotated by heading, stamp a separate compaction patch at each (each → its own TREAD rut, mass-conserving as today). Emit `wheel_tracks` metadata (the 4 contact polylines + heading + slip).
- **Drum dig events.** When a drum is lowered + spinning over a cell band, emit an `EXCAVATED`/`SPOIL` swath (the existing cut/dump mass transfer already exists in `column_state`) plus the `drum_marks` metadata (swath + depth + teeth params + phase).
- **Refinement driver.** Wire the quadtree's `active`/`touched` leaves to allocate/keep fine tiles per the `refinement` policy; coarsen tiles that fall out of `refine_where` (Mode B `"active"`). All transitions use the §2.4 mass-exact operators.

---

## 6. Conservation invariants & acceptance tests (extend `terrain_authority/tests.py`)

New checks (must pass alongside the current 10/10):
1. **refine/coarsen round-trip:** `coarsen(refine(cell)) == cell` exactly; total mass drift 0; `height==datum+mass/density` max-err 0.
2. **base↔tile consistency:** for a refined scene, every base cell over a tile equals `coarsen(tile children)` (mass + area-mean height).
2b. **zero-mass coarsen:** an all-empty (`mass_areal=0`) coarse cell coarsens to a finite `density = mean(density_fine)` and `height == datum` — no NaN/inf (zero-mass branch).
2c. **non-uniform datum:** with varied per-cell `datum`, coarse `height == area-mean(child heights)` and `datum_coarse == mean(datum_fine)`.
2d. **non-integer k rejected:** a `base_cell_m/fine_cell_m` that is not a positive integer is rejected/validated, not silently truncated.
3. **toggle equivalence:** `refinement.enabled=false` produces byte-identical base rasters to the current uniform pipeline (no-op-equivalent).
4. **4-wheel separability:** after a straight drive, exactly two TREAD bands at ~0.57 m gauge (28 cells @ 2 cm); after a turn, four distinct arcs.
5. **mass under 4-wheel pass:** total mass unchanged (density-only compaction, as the current single-pass test).

---

## 7. Phased build order

1. **AA + base detail normal** (§4.1, §4.2.1–2) — no contract/data change, immediate visual lift. *Cheap win.*
2. **4-wheel stamping + `wheel_tracks` metadata + cleat shader** (§5, §4.2.3, §4.3) — the headline "four separate compacting tracks."
3. **Variable-resolution Mode B** (§2) — the 1 cm corridor + the mass-exact operators + tests (§6.1–3). Toggleable; default `enabled:false` until validated.
4. **Drum dig + teeth marks** (§5, §4.2.4) — depends on a drum-control story (excavation is disabled in LAC's mapping year, so this is foss_ipex-distinctive, not LAC-required).

Default the whole refinement subsystem **off** so the existing scenes/tests are untouched until each piece is validated.

---

## 8. Open params to pin (before building)

- `fine_cell_m` default (1 cm vs 0.5 cm) and `base_cell_m` for mission scale (8 cm?).
- Grouser cleat pitch + depth (EZ-RASSOR wheel; cite `asce-es-2024-isru-pilot-excavator-wheel-testing.pdf`).
- Drum `teeth_count` / `teeth_pitch_m` / scoop depth (RASSOR drum; cite `2021-ASCEND-Mass-Inference-RASSOR.pdf`).
- Exact wheel **contact width** (the 0.18 m default vs the rendered mesh).
- Whether TAA (needs motion vectors over a sequence) or MSAA+SSAA is the better headless default.

---

## 9. Photometry — Hapke / Lommel–Seeliger BRDF (IMPLEMENTED)

The terrain and clast shaders own diffuse lighting via a custom `light()` instead of Godot's
`diffuse_lambert`. Lambert (∝ cos *i*) is wrong for an airless granular surface and **most wrong at
the grazing/low-sun angles the IPEx polar mission cares about** (it crushes low-incidence slopes to
black). We use the **Hapke IMSA** (isotropic multiple-scattering approximation), built up from the
Lommel–Seeliger single-scattering core so each physical term stays legible:

```
r(i,e,g) = (w/4π) · μ₀/(μ₀+μ) · [ (1 + B(g))·P(g) + H(μ₀)H(μ) − 1 ]
```

- **μ₀/(μ₀+μ)** — Lommel–Seeliger single-scattering geometry (Hapke 1981). The headline change over
  Lambert: cancels most limb/terminator darkening → the near-uniform-brightness disk the Moon
  actually is, instead of a Lambert sphere driven to black at a 5° grazing sun.
- **P(g)** — 2-term Henyey–Greenstein phase fn; lunar mare is net **backscattering**. `b=0.26, c=0.08`
  (Sato et al. 2014, LROC-derived global Hapke maps @643 nm).
- **B(g)** — shadow-hiding opposition surge `B₀/(1+tan(g/2)/h)`, `B₀=1.0, h=0.06` (Hapke 2002).
- **H(x)** — Chandrasekhar isotropic multiple scattering, Hapke 1981 rational approx; small on the
  dark Moon but included.
- **w** = the per-texel `ALBEDO` (macro mottle + state tints + cut-depth), so all prior appearance
  work flows in as a spatially/chromatically varying single-scattering albedo (standard Hapke usage).

Implementation: `terrain.gdshader`, `terrain_farfield.gdshader`, `clast.gdshader` (one shared
`light()`); params in `state_fields.gd` (literature defaults + optional per-scene `photometric_model`
override, e.g. highlands vs mare); `--brdf lambert|hapke` toggles the same pipeline for the A/B.
Additive & default-on (every scene gets correct airless photometry; no INTERFACE/physics change).
`hapke_gain=1.4` is a documented radiance calibration (lands the nominal scene at the prior
Lambert mid-tone), not a look-knob. Sources are cited by author/year in `papers/CITATIONS.md`.

**Deferred (flagged, not yet built): macroscopic-roughness shadowing** `S(i,e,g; θ̄)` (Hapke 1984,
mean slope θ̄ ≈ 20–25° for the Moon). It primarily corrects the BRDF near the terminator/limb — i.e.
the grazing-sun regime — so it is the natural next photometry refinement. We omit it for now because
the detail-normal pass (§4.2.2) + real cast shadows already carry sub-mesh roughness shadowing
*geometrically*; the statistical θ̄ term is a second-order correction on top of that.
