# foss_ipex — State-Field Interface Contract (v1.0, FROZEN)

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

*Contract frozen 2026-05-30. Bump `schema_version` on any breaking change.*
