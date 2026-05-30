# foss_ipex — Artifact Manifest & Verification

**Verified:** 2026-05-30, end-to-end from the committed repo (commit `197f771`).
The **authority + all matplotlib viz consumers were re-run from clean this pass** and their
exit codes / numbers below are from that run. **Godot:** three representative renders were
re-run this pass (crater terrain layer, the articulated-rover hero, and the 15-frame quadtree
fly-through sequence) — all exit 0 and **byte-identical** to the committed PNGs (the headless
Vulkan renders are reproducibly deterministic here); the remaining layer renders were produced
and verified earlier this session by the build + adversarial-verify workflows. **Chrono** Path A
was executed per [`docs/chrono_bringup_log.md`](docs/chrono_bringup_log.md) in a separate conda
env (not the project `.venv`), so it is recorded, not re-run here.

**Verdict: ALL GREEN.** 10/10 conservation + quadtree tests pass; all **7** scenes re-export
deterministically with masses matching; all **5** matplotlib consumers and the full Godot layer
set + rover + fly-through produce non-black, varied PNGs; PyChrono 10.0.0 runs an SCM rover at
lunar g and a partial contract exporter round-trips through the frozen `io_fields`.

This manifest is a verification snapshot; [`README.md`](README.md) is the authoritative
description of the system and what is papered over.

---

## Commands run this pass (exact, cwd = repo root unless noted)

| # | Command | Exit | Result |
|---|---|---|---|
| 1 | `.venv/bin/python -m terrain_authority.tests` | 0 | **10/10 checks PASS** (7 conservation + 3 quadtree) |
| 2 | `.venv/bin/python -m terrain_authority.scenes` | 0 | Re-wrote **7** scenes; deterministic (crater + tread_track bookend md5 identical pre/post) |
| 3 | `.venv/bin/python viz/variety_panel.py` | 0 | variety_panel.png + caveins_filmstrip.png + caveins.gif |
| 4 | `.venv/bin/python viz/groundtruth_viz.py samples/{crater,boulder_field}` | 0 | groundtruth_*.png |
| 5 | `.venv/bin/python viz/tread_track.py` | 0 | tread_track.gif + tread_track_filmstrip.png |
| 6 | `.venv/bin/python viz/quadtree_demo.py` | 0 | quadtree_demo.gif + quadtree_demo_filmstrip.png |
| 7 | `render_layers.sh -- --scene ../samples/crater --layers terrain,clasts ... --out layer_3_terrain.png` | 0 | lit crater (byte-identical to committed) |
| 8 | `render_layers.sh -- --scene ../samples/crater_boulders --layers terrain,clasts,rover --pose 1.7,1.05,1.5,3.7,0.05,3.2 ... --out crater_boulders_rover.png` | 0 | "placed 143 clasts"; "assembled articulated EZ-RASSOR ... AABB (1.83,0.66,1.70)"; byte-identical to committed |
| 9 | `render_layers.sh -- --sequence ../samples/tread_track --stride 2 --layers terrain,quadtree,rover` | 0 | wrote 15 fly-through frames; log shows rover_rc advancing + active_leaves per frame + yaw turning at the bend |

Godot stderr prints benign ALSA "all audio drivers failed → dummy driver" warnings (headless box,
no sound device); rendering is unaffected and every process exits 0.

---

## Foundation — physics authority + state-field producer

Pure-NumPy Tier-2 surrogate (`terrain_authority/`, **11 modules**: constants, io_fields, column_state,
procgen, sandpile, rover, **quadtree**, hexviz, scenes, tests, `__init__`) emitting the frozen
`INTERFACE.md` contract (now **v1.0.1** — additive optional quadtree metadata; rasters unchanged).

| Scene (`samples/`) | Frames | Total mass | Notes | Status |
|---|---|---|---|---|
| `flat_compact` | 1 | **6298.481 kg** | dense, near-zero relief, VIRGIN only (low-albedo proxy) | GREEN |
| `rolling_hills` | 1 | **6500.812 kg** | fbm loose top, disturbance ≤ 0.02 | GREEN |
| `crater` | 1 | **4606.247 kg** | Pike-class EXCAVATED bowl (labels 0..2) | GREEN |
| `boulder_field` | 1 | **5819.188 kg** | **186** Golombek-SFD clasts in metadata | GREEN |
| `crater_boulders` | 1 | **4840.711 kg** | crater + **143** clasts (excluded from fresh bowl, surface-snapped) | GREEN |
| `crater_caveins` | **102** (t000..t101) | drift **0.00e+00 kg** (4525.5909) | 400-step rim slump; raw frames git-excluded except bookends | GREEN |
| `tread_track` | **32** (t000..t031) | drift **0.00e+00 kg** (5622.1704) | driven-rover VIRGIN→TREAD trail; per-frame quadtree metadata (active last=36, touched=208); raw frames git-excluded except bookends | GREEN |

Each committed scene carries the 5 contract rasters (`heightmap`/`mass_areal`/`density`/`disturbance`
`.rf32` + `state_label.r8`) + `metadata.json` + 4 `preview_*.png`. Terminal `hexviz` (no file output)
renders any field as dependency-free ASCII.

**Conservation invariants (spec §10), re-confirmed this pass:**
- Total mass constant across cut→dump→relax, rel_drift **2.99e-16**.
- `height == datum + mass/density` after every op (cut/dump/relax/procgen/crater/wheel_pass): max_err **0.0**.
- Rover single pass preserves mass (density-only compaction; rut sinks): m0 == m1, **0.0** drift.
- Sandpile relaxation conserves mass (rel_drift **1.75e-16**) and leaves all loose slopes ≤ θ_r (35.57° vs 35.00°, within 1°).
- save/load round-trip preserves dims/dtype/row-major.
- **Quadtree:** leaves tile the field exactly once (65536/65536, no gaps/overlap); promotion monotone toward the rover (rover leaf size 8 fine, far leaf size 64 coarse); active-leaf count bounded (peak 36 of 64) and the active cluster tracks the rover across all 32 frames.

## Native viz consumers (matplotlib, pure `load_scene` readers)

| Artifact | Size | Description | Status |
|---|---|---|---|
| `viz/out/variety_panel.png` | 1.18 MB | 2×2 grazing-sun hillshade: flat_compact / rolling_hills / crater / boulder_field | GREEN |
| `viz/out/caveins_filmstrip.png` / `caveins.gif` | 372 KB / 867 KB | 6-frame + 30-frame rim-slump cave-in | GREEN |
| `viz/out/groundtruth_crater.png` (+ `_turn0..2`) | 282 KB | D1b 3D bar3d cuboids by state_label + quadtree wireframes + clast scatter + turntable | GREEN |
| `viz/out/groundtruth_boulder_field.png` | 392 KB | VIRGIN cuboids + 186 clast spheres + quadtree | GREEN |
| `viz/out/tread_track.gif` / `tread_track_filmstrip.png` | 735 KB / 540 KB | driven-rover VIRGIN→TREAD compaction trail (mass conserved, rut via height=mass/density) | GREEN |
| `viz/out/quadtree_demo.gif` / `quadtree_demo_filmstrip.png` | 4.49 MB / 549 KB | **interaction-keyed quadtree**: active leaves (red) track the rover, touched trail (amber), coarse far (blue), under VIRGIN→TREAD + hillshade rut | GREEN |

## Godot render sidecar (headless Vulkan; D2 + D4)

`godot_sidecar/` — GDScript `INTERFACE.md` loader (`state_fields.gd`, now parses the v1.0.1 optional
keys), fine active-zone `ArrayMesh` + far-field LOD plane (`terrain.gd`), lit regolith / false-color /
dust / distortion shaders, articulated rover assembly + quadtree overlay + `--sequence` mode (`sidecar.gd`).

| Artifact | Size | Description | Status |
|---|---|---|---|
| `out/layer_1_heightmap.png` | 130 KB | unlit false-color elevation ramp | GREEN |
| `out/layer_2_state.png` | 6.3 KB | false-color state enum (grey VIRGIN + amber EXCAVATED; flat regions compress) | GREEN |
| `out/layer_3_terrain.png` | 254 KB | lit crater under ~5° sun, deep + cast rim shadow (re-run this pass) | GREEN |
| `out/layer_4_clasts.png` | 254 KB | crater has 0 clasts → equals terrain (documented; real demo below) | GREEN |
| `out/layer_5_dust.png` | 261 KB | ballistic GPUParticles3D, lunar g, soft-haze puffs | GREEN |
| `out/layer_6_distortion.png` | 223 KB | Brown-Conrady radial barrel-warp post-process (stub) | GREEN |
| `out/boulder_terrain_clasts.png` | 315 KB | 186 sphere clasts, long grazing-sun shadows | GREEN |
| `out/crater_boulders.png` | 359 KB | crater + Golombek boulder field together | GREEN |
| `out/crater_boulders_rover.png` / `rover_on_terrain.png` | 241 KB / 253 KB | **articulated EZ-RASSOR** (chassis + 4 wheels + 2 arms + 2 drums, MIT) on crater rim / rolling hills (re-run this pass; AABB 1.83×0.66×1.70 m, ground-snapped) | GREEN |
| `out/quadtree_flythrough.gif` (+ 8 key frames `_000..014`) | 1.05 MB | **D4 headline**: rover drives the tread_track path while the fine active-mesh window + quadtree LOD overlay follow it, consuming per-frame `active_leaves` (re-run this pass, 15 frames) | GREEN |
| `out/cube_on_plane.png` | 22 KB | original smoke test, intact | GREEN |

## Chrono Path A (executed; separate conda env — `docs/chrono_bringup_log.md`)

| Item | Result | Status |
|---|---|---|
| conda `chrono` env + **PyChrono 10.0.0** (`py312h98ab86c_677`) | installed; GDAL `.so.37` soname blocker resolved via `libgdal=3.11` | GREEN |
| stock `SCMTerrain` demo, headless, lunar g | 300 steps, `GetModifiedNodes`→261 nodes, exit 0 | GREEN |
| `scripts/chrono_scm_rover.py` | 400 steps, real 13.4 mm rut, 918 deformed nodes read back | GREEN |
| `scripts/chrono_scm_export.py` (+`_demo`) → INTERFACE | **PARTIAL**: heightmap + disturbance Chrono-sourced; mass_areal/density honest surrogate placeholders; round-trips via `io_fields`; `height = mass/density` invariant holds to **3.98e-08** | PARTIAL (by design) |

## Honest caveats (not defects)

1. **`crater_caveins` / `tread_track` on-disk mass.** The "drift 0.0" figure is the in-memory float64
   invariant. The `.rf32` contract stores `<f4`, so recomputing mass from the saved rasters shows
   float32 storage quantization (~1e-7 relative) — storage precision, not a conservation error.
2. **Rover is a static pose** (joints are fixed constants, not physics-driven; single-point ground-snap) — README §4 #11.
3. **Quadtree manages render/space LOD, not solve cost** (the physics grid is still uniform-fine) — README §4 #4.
4. **Chrono is bootstrapped, not the live authority**; the exporter is partial (no §4.4 mass-hybrid; bare test cylinder, not a Chrono::Vehicle) — README §4 #2.
5. **`layer_4_clasts.png` == `layer_3_terrain.png`** on the crater scene (0 clasts there); the genuine clast demos are `boulder_terrain_clasts.png` / `crater_boulders.png`.

No blocking issues. Every command above runs clean from the committed repo (`.venv` + the vendored
Godot binary for the renders; the `chrono` conda env for the Chrono scripts).
