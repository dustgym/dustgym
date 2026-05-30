# foss_ipex — State-Field Interface Contract (v1.0, FROZEN; v1.0.1 + v1.0.2 additive notes)

This is the **decoupling seam** of the architecture (spec §2, §4). The physics authority
*produces* a directory of state fields; renderers and visualizers *consume* it. Producer and
consumer never share memory or types — only this on-disk format. That is what lets the
NumPy surrogate stand in for Project Chrono today and be swapped for a real Chrono::Vehicle +
SCM producer later with zero consumer changes.

> **Weekend-slice producer:** a pure NumPy/SciPy analytical Tier-2 surrogate
> (Bekker/Janosi/Wong-Reece geometry, not force-accurate; spec §3, §9 "Robot design context").
> **Production producer:** Project Chrono (spec §2, §11). Both emit *this exact format*.
> Papered-over, by design — cite spec §2 (authority model) and §4 (physics↔render interface).

---

## 1. Directory layout (one directory = one scene snapshot at time t)

```
samples/<scene_name>/
  metadata.json        # REQUIRED sidecar — describes everything below
  heightmap.rf32       # REQUIRED float32 surface elevation (m)
  mass_areal.rf32      # REQUIRED float32 areal mass (kg/m^2) — THE conserved invariant
  density.rf32         # REQUIRED float32 current bulk density (kg/m^3)
  disturbance.rf32     # REQUIRED float32 normalized cumulative disturbance [0,1]
  state_label.r8       # REQUIRED uint8 enum {0..4}
  ice.rf32             # OPTIONAL float32 ice/volatile mass fraction [0, ~0.06]
  preview_*.png        # OPTIONAL human-inspection previews (not consumed by Godot)
```

A *time series* (e.g. the cave-in relaxation sweep) is `samples/<scene>/t000/`, `t001/`, …,
each a full snapshot. Frame cadence is documented in the parent `metadata.json`.

## 2. Raster encoding (all `.rf32` / `.r8`)

- **Layout:** row-major (C order), no header, no padding. `width * height` elements.
- **`.rf32`:** little-endian IEEE-754 float32 (`numpy dtype '<f4'`).
- **`.r8`:** unsigned 8-bit (`numpy dtype 'u1'`).
- **Indexing:** element `k = row * width + col`. `row` increases +Z, `col` increases +X.
- Producer writes `arr.astype('<f4').tofile(path)` (already row-major from NumPy C arrays).
- Godot reads `FileAccess.get_buffer()` → `Image.create_from_data(width, height, false,
  Image.FORMAT_RF, bytes)` for float, `FORMAT_R8` for uint8. **No EXR, no PNG decode** in the
  hot path — raw bytes only. (Dependency-free on both ends; this is deliberate.)

## 3. Coordinate & frame conventions  (spec §11 — the Y-up/Z-up TF trap)

**Field space (canonical, what the rasters store):**
- `index[row, col]` → world `x = col * cell_m`, `z = row * cell_m`, `height = value` (up).
- Origin `index[0,0]` is the world min corner given by `world_bounds_m.{x0,y0}` (y0 ≡ z0).

**Godot mapping (Y-up):** `godot.x = x`, `godot.y = height`, `godot.z = z`. Direct.

**ROS mapping (Z-up, REP-103):** deferred to the ROS2 bridge (out of weekend scope). When
built: `ros.x = x`, `ros.y = -z` (or per chosen handedness), `ros.z = height`. Documented here
so the trap is named, not so it is solved this weekend. Cite spec §11.

## 4. Data-model semantics  (spec §5.3, §6)

- **`mass_areal` is the conserved invariant.** Everything else derives from or modifies it.
- **`heightmap` is DERIVED, never authored independently:** `height = mass_areal / density`
  (areal mass [kg/m²] ÷ bulk density [kg/m³] = column thickness [m], added to a datum).
  Producers MUST compute it this way; the conservation test (spec §10) asserts it.
- **`state_label` enum:** `0 VIRGIN, 1 TREAD, 2 EXCAVATED, 3 SPOIL, 4 COMPACTED_BERM` (spec §6).
- **`disturbance`** ∈ [0,1]: normalized "how worked is this cell" — max-sinkage-ever or
  pass-count proxy. Drives the shader's fresh-cut albedo/roughness; no physics reads it back.
- **`density`** in **SI kg/m³** (spec §5 quotes g/cm³: 1.30 g/cm³ = **1300** kg/m³,
  1.92 → **1920**). The contract is SI everywhere to kill unit ambiguity.

## 5. `metadata.json` schema  (v1.0)

```json
{
  "schema_version": "1.0",
  "scene_name": "crater_caveins",
  "producer": "terrain_authority (NumPy Tier-2 surrogate)",
  "grid": { "width": 256, "height": 256, "cell_m": 0.02, "order": "row-major-C" },
  "world_bounds_m": { "x0": 0.0, "y0": 0.0, "x1": 5.12, "y1": 5.12 },
  "gravity_m_s2": 1.62,
  "fields": {
    "heightmap":   { "file": "heightmap.rf32",   "dtype": "<f4", "units": "m" },
    "mass_areal":  { "file": "mass_areal.rf32",  "dtype": "<f4", "units": "kg/m^2" },
    "density":     { "file": "density.rf32",     "dtype": "<f4", "units": "kg/m^3" },
    "disturbance": { "file": "disturbance.rf32", "dtype": "<f4", "units": "1 (normalized)" },
    "state_label": { "file": "state_label.r8",   "dtype": "u1",  "enum": ["VIRGIN","TREAD","EXCAVATED","SPOIL","COMPACTED_BERM"] }
  },
  "ice_present": false,
  "height_range_m": [ -0.4, 0.15 ],
  "clasts": [
    { "id": 0, "center_m": [1.2, 0.05, 3.4], "radius_m": 0.08, "shape": "sphere", "buried_frac": 0.3 }
  ],
  "active_zone": { "min_rc": [64, 64], "max_rc": [192, 192] },
  "quadtree": [
    { "level": 0, "row0": 0, "col0": 0, "size": 256, "label": "ROOT" },
    { "level": 2, "row0": 64, "col0": 64, "size": 64, "label": "ACTIVE" }
  ],
  "notes": "free text; e.g. sun elevation, scene description"
}
```

- `clasts[].center_m` is world `[x, height_up, z]` (Godot-ready order).
- `quadtree[]` exists to drive D1b wireframes: each entry is a node box `[row0,col0]` of
  `size` cells at LOD `level`. Far-field leaves render as a single low-res plane; the
  `ACTIVE` node(s) render as fine cuboids. (Spec §4: the tree manages *space*, not physics.)
- `active_zone` is the fine-solve window (spec §4 "Under wheels/drums").

### 5.1 OPTIONAL additive keys — interaction-keyed quadtree (v1.0.1)

> **ADDITIVE, backward-compatible (contract still v1.0 for the rasters).** These are NEW
> OPTIONAL `metadata.json` keys. They appear today on the **driven-rover `tread_track`**
> time-series frames (per-frame) to realize spec §4's headline thesis — *the tree manages
> SPACE, keyed to interaction*: as the rover drives, quadtree leaves near it promote to the
> finest level (fine/active) while distant regions stay coarse. **Consumers MAY ignore all
> of these** and lose nothing; no raster, dtype, endianness, or existing metadata key
> changes. The static `quadtree[]` D1b key above is untouched and still present.

```jsonc
{
  "rover_rc": [128, 133],            // rover footprint CENTER [row,col] this frame, or null (pre-drive)
  "active_leaves": [[112,112,120,120], ...],   // FINE (min_leaf) leaf boxes under the rover NOW
  "touched_leaves": [[...], ...],    // cumulative min_leaf cells the rover has EVER activated (trail)
  "quadtree_nodes": [                // the full per-frame subdivision (coarse far, fine near)
    { "level": 0, "row0": 0, "col0": 0, "size": 256, "leaf": false },
    { "level": 5, "row0": 120, "col0": 128, "size": 8, "leaf": true }
  ],
  "quadtree_lod": {                  // the deterministic promotion knobs (also on the parent)
    "min_leaf": 8, "refine_factor": 0.5, "footprint_radius_cells": 5.5, "field_size": 256
  }
}
```

- **Box convention:** every box is `[r0, c0, r1, c1]` **half-open** in cells (rows `r0..r1-1`,
  cols `c0..c1-1`), same row-major indexing as the rasters (§2/§3). World corners are
  `x = col*cell_m`, `z = row*cell_m`.
- **`active_leaves`** are the `min_leaf`-size (finest) leaves under the current rover
  footprint — the live LOD hot-set that **promotes as the rover approaches and evicts as it
  leaves** (count is bounded; see `terrain_authority.tests`).
- **`touched_leaves`** is the promote-only cumulative history (the refined trail left behind,
  mirroring the VIRGIN→TREAD segmentation).
- **`quadtree_nodes[]`** tiles the field exactly once via its `leaf:true` nodes (no gaps,
  no overlap). It is a *finer-grained, interaction-driven* sibling of the static `quadtree[]`
  D1b key, NOT a replacement.
- **Promotion rule** (deterministic, pure NumPy; `terrain_authority/quadtree.py`): subdivide
  a node iff `size > min_leaf` AND box-distance(rover, node) − `footprint_radius_cells`
  `< refine_factor * size`. Distance-graded LOD: fine near the rover, coarse far.
- The parent `tread_track/metadata.json` advertises this under `quadtree_lod.per_frame_keys`.

### 5.2 OPTIONAL additive keys — per-wheel tracks & excavation marks (v1.0.2)

> **ADDITIVE, backward-compatible** (rasters unchanged; `schema_version` stays `"1.0"`). NEW
> OPTIONAL per-frame `metadata.json` keys that let a consumer orient and phase **track and
> teeth-mark detail in the shader** without resolving it in the heightfield (design:
> `docs/render_fidelity_spec.md`). **Consumers MAY ignore all of these** and render exactly as
> v1.0.1. No raster, dtype, endianness, or existing key changes.

```jsonc
{
  "wheel_tracks": {                        // the four wheels' ground contact THIS frame
    "LF": { "points": [[r,c], ...],        // contact-center polyline, [row,col] in BASE cells
            "heading_rad": 1.5708,         // travel dir in field space: 0=+col/+X, +pi/2=+row/+Z
            "slip": 0.05,                  // [0,1] slip ratio (modulates smear), OPTIONAL
            "width_m": 0.18 },             // contact band width
    "RF": { ... }, "LB": { ... }, "RB": { ... }
  },
  "drum_marks": [                          // zero or more ACTIVE excavation drums this frame
    { "drum": "front",                     // "front" | "back"
      "swath": [[r,c], ...],               // dug-band centerline, [row,col] in BASE cells
      "depth_m": 0.03, "width_m": 0.20,
      "teeth_count": 8, "teeth_pitch_m": 0.025, "phase": 0.0 }  // periodic teeth params
  ]
}
```

- `points`/`swath` use the same `[row,col]` row-major indexing and `[r0,c0,r1,c1]` half-open box
  convention as §2/§3/§5.1. `heading_rad` orients the (transverse) cleat/teeth ridge pattern.
- These pair with `state_label` (TREAD → cleat marks; EXCAVATED/SPOIL → teeth marks) and
  `disturbance` (detail strength), which already exist — the new data only adds **direction +
  phase + periodicity**. The renderer bakes them into a derived track-direction field
  (consumer-side; **not** a new on-disk raster).
- **Per-frame, not cumulative:** `points`/`swath` are the contact samples for THIS frame only;
  reconstruct a trail by concatenating frames. A single point is allowed (point contact); ≥2
  points define orientation in addition to `heading_rad`. These are PER-FRAME keys (on
  `tNNN/metadata.json`).
- **Units:** all `*_m` fields (`width_m`, `depth_m`, `teeth_pitch_m`) are **SI metres in world
  space** (§4), NOT cells; only `points`/`swath` are `[row,col]` base cells (convert via `cell_m`).

### 5.3 OPTIONAL additive keys — variable-resolution refinement / tiles (v1.0.2)

> **ADDITIVE, backward-compatible.** The BASE rasters at `grid.cell_m` are still REQUIRED and
> fully describe the scene. Refinement adds OPTIONAL finer-resolution **tiles** over the
> rover-interacted corridor (design + mass-conservation operators: `docs/render_fidelity_spec.md`
> §2). A consumer that ignores `tiles[]`/`tiles/` renders the base (coarser in the corridor,
> still correct). When `refinement.enabled=false` the scene is uniform base resolution =
> identical to v1.0.

```jsonc
{
  "refinement": {
    "enabled": true,
    "base_cell_m": 0.02, "fine_cell_m": 0.01,
    "refine_where": "touched",     // "active" | "touched" | "none"
    "fine_min_leaf": 4
  },
  "tiles": [                        // each tile is a normal raster bundle at fine_cell_m
    { "id": 7,
      "region_rc": [120, 128, 136, 144],   // base-cell-aligned region [r0,c0,r1,c1] half-open
      "cell_m": 0.01,
      "dir": "tiles/tile_0007" }            // subdir holding heightmap.rf32 etc. @ cell_m
  ]
}
```

- **Storage:** `samples/<scene>/<tNNN>/tiles/tile_<id>/` holds the same 5 REQUIRED rasters at
  `cell_m`. `region_rc` is in BASE cells and MUST be base-cell-aligned (an integer block).
  `k = base_cell_m / cell_m` MUST be a positive integer; the tile raster dims MUST equal
  `(r1-r0)*k × (c1-c0)*k`.
- **Tile set:** `id` is unique within a frame; tile `region_rc` regions are pairwise DISJOINT
  (no base cell under more than one tile); gaps are allowed (uncovered base cells render at base
  resolution). Producers MUST NOT emit overlapping tiles; a consumer that finds overlap MAY
  render either and SHOULD warn.
- **Robustness:** a consumer that finds a tile violating base-alignment, the dimension relation,
  or integer-`k` MUST ignore that tile and fall back to the base rasters for its region (never
  crash) — the base rasters always fully describe the scene, so dropping a bad tile is safe.
- **Relation to §5.1:** the refined region is the union of the §5.1 leaves selected by
  `refine_where` — `"active"` → `active_leaves`, `"touched"` → `touched_leaves`,
  `"none"`/`enabled:false` → no tiles. `fine_min_leaf` is the leaf size tiles are stored at (may
  be `< quadtree_lod.min_leaf`); `tiles[]` is thus the refined subset of the §5.1 leaf set.
- **Placement:** `tiles[]` is a PER-FRAME key (on `tNNN/metadata.json`); `refinement` appears on
  BOTH the parent `metadata.json` (scene-level policy) and per-frame as emitted.
- **Discoverability (optional):** a producer MAY add an informational top-level
  `"contract_revision": "1.0.2"` and/or `"features": [...]`; these are ignorable and do NOT change
  `schema_version` (still `"1.0"`). Consumers MUST feature-detect by key presence, not by
  `contract_revision`.
- **CONSERVATION INVARIANT (normative):** for any base cell overlapped by a tile, the base
  raster value MUST equal the **mass-conserving coarsen()** of that tile's fine cells:
  `mass_areal_base = mean(mass_areal_fine)`; `density_base = mass_areal_base /
  mean(mass_areal_fine/density_fine)` (the mass-weighted harmonic mean — chosen so base height =
  area-mean of child heights; **NOT** `Σ(mass·ρ)/Σmass`, which would break the height invariant);
  `datum_base = mean(datum_fine)`; `state_label_base` = highest-priority child label by
  **EXCAVATED > SPOIL > COMPACTED_BERM > TREAD > VIRGIN**; `disturbance_base = mean`. I.e. the
  base raster is always a valid mass-conserving down-sample of the finest data present. The full
  refine/coarsen operators (drift 0, `height = datum + mass_areal/density` preserved, zero-mass
  branch, integer-`k` precondition) are specified in `docs/render_fidelity_spec.md` §2.4 and
  asserted in `terrain_authority/tests.py`.

## 6. Producer & consumer responsibilities

| Producer (terrain_authority / Chrono) MUST | Consumer (Godot / native viz) MAY ASSUME |
|---|---|
| Write all REQUIRED fields, same `width×height` | All rasters share grid dims from metadata |
| Keep `height == mass_areal/density` (assert) | `heightmap` is authoritative for geometry |
| Keep Σ`mass_areal·cell_area` + inventory const | `disturbance∈[0,1]`, `state_label∈{0..4}` |
| Emit `metadata.json` first / atomically | Read metadata before opening rasters |
| Use SI units throughout | Units are SI; convert at the shader if needed |

## 7. Shared Python helper

`terrain_authority/io_fields.py` provides `save_scene(dir, fields, metadata)` and
`load_scene(dir) -> (fields, metadata)` implementing this contract. **All Python consumers
import these; they do not re-implement raw I/O.** Godot implements its own loader in GDScript
(`state_fields.gd`) against this same spec.

---

*Contract frozen 2026-05-30 (v1.0). v1.0.1 (2026-05-30): ADDITIVE only — added the OPTIONAL
interaction-keyed quadtree keys in §5.1 (`rover_rc`, `active_leaves`, `touched_leaves`,
`quadtree_nodes`, `quadtree_lod`) on the `tread_track` frames. No raster, dtype, endianness,
or existing metadata key changed; `schema_version` stays `"1.0"` because the on-disk raster
contract is unchanged and consumers may ignore the new keys. Bump `schema_version` only on a
BREAKING change.*

*v1.0.2 (2026-05-30): ADDITIVE only — added OPTIONAL §5.2 (`wheel_tracks`, `drum_marks` for
shader-driven per-wheel track & drum teeth-mark detail) and §5.3 (`refinement`, `tiles[]` for
variable-resolution corridor refinement with a normative mass-conserving base↔tile invariant).
Base rasters, dtypes, endianness, and all existing keys are unchanged; `tiles/` sub-bundles are
themselves standard v1.0 raster bundles. Consumers may ignore every v1.0.2 key and render
identically to v1.0.1; `schema_version` stays `"1.0"`. Design + operators:
`docs/render_fidelity_spec.md`.*
