# foss_ipex — Artifact Manifest & Verification

**Verified:** 2026-05-30, end-to-end from the committed repo (commit `245b363`).
The **authority + all matplotlib viz consumers were re-run from clean this pass** and their
exit codes / numbers below are from that run. **Godot:** the full reference render set was
**re-rendered this pass under the v1.0.2 render-fidelity pipeline** (4× MSAA + SMAA + 1.5×
SSAA, detail-normal regolith shading) and committed — all exit 0, headless Vulkan renders are
reproducibly deterministic here. **Chrono** Path A was executed per
[`docs/chrono_bringup_log.md`](docs/chrono_bringup_log.md) in a separate conda env (not the
project `.venv`), so it is recorded, not re-run here.

**Verdict: ALL GREEN.** 18/18 conservation + quadtree + variable-resolution tests pass; all
**9** scenes re-export deterministically with masses matching; all **5** matplotlib consumers
and the full Godot render set (six diagnostic layers + rover + the new 4-wheel / excavation
fidelity renders + the 1 cm trailing-cam fly-through) produce non-black, varied PNGs;
PyChrono 10.0.0 runs an SCM rover at lunar g and a partial contract exporter round-trips
through the frozen `io_fields`.

> **Note on the v1.0.2 render-fidelity upgrade (commits `c8ef9a8`, `245b363`).** Anti-aliasing
> (4× MSAA + SMAA + 1.5× SSAA) and the detail-normal/cleat/teeth shader change *every* rendered
> frame's bytes, so the reference renders below were **regenerated under the new pipeline this
> pass** — they are no longer byte-identical to the pre-v1.0.2 captures (that is the intended
> quality lift, not a regression).

This manifest is a verification snapshot; [`README.md`](README.md) is the authoritative
description of the system and what is papered over.

---

## Commands run this pass (exact, cwd = repo root unless noted)

| # | Command | Exit | Result |
|---|---|---|---|
| 1 | `.venv/bin/python -m terrain_authority.tests` | 0 | **18/18 checks PASS** (7 conservation + 3 quadtree + 8 variable-resolution/4-wheel §6) |
| 2 | `.venv/bin/python -m terrain_authority.scenes` | 0 | Re-wrote **9** scenes; deterministic (existing-scene bookend md5 identical pre/post) |
| 3 | `.venv/bin/python scripts/build_flythrough_1cm.py` | 0 | global-1 cm (512² @ 0.01 m) 4-wheel fly-through showcase scene; mass drift 0 |
| 4 | `.venv/bin/python viz/variety_panel.py` | 0 | variety_panel.png + caveins_filmstrip.png + caveins.gif |
| 5 | `.venv/bin/python viz/groundtruth_viz.py samples/{crater,boulder_field}` | 0 | groundtruth_*.png |
| 6 | `.venv/bin/python viz/tread_track.py` | 0 | tread_track.gif + tread_track_filmstrip.png |
| 7 | `.venv/bin/python viz/quadtree_demo.py` | 0 | quadtree_demo.gif + quadtree_demo_filmstrip.png |
| 8 | `render_layers.sh -- --scene ../samples/crater --layers {heightmap,state,terrain,terrain+clasts,terrain+dust,terrain+distortion} --pose 2.56,3.0,6.4,2.56,-0.1,2.56 --out layer_{1..6}_*.png` | 0 | six diagnostic layers, AA pipeline |
| 9 | `render_layers.sh -- --scene ../samples/crater_boulders --layers terrain,clasts,rover --pose 1.7,1.05,1.5,3.7,0.05,3.2 --out crater_boulders_rover.png` | 0 | "placed 143 clasts"; articulated EZ-RASSOR ground-snapped on the rim |
| 10 | `render_layers.sh -- --scene ../samples/tread_track_4wheel/t018 --layers terrain[,rover] --out tread_track_4wheel*_fidelity.png` | 0 | four-wheel cleated tracks + 1 cm corridor; detail/teeth shading |
| 11 | `render_layers.sh -- --scene ../samples/excavation_marks/t001 --layers terrain --out excavation_marks_fidelity.png` | 0 | drum trench: EXCAVATED cut + raised SPOIL lip + teeth-textured floor |
| 12 | `render_layers.sh -- --sequence ../samples/tread_track_4wheel_1cm --stride 1 --size 1920x1080 --layers terrain,quadtree,rover` | 0 | 24-frame 1080p **trailing chase-cam** fly-through; rover drives **nose-first**; quadtree wireframe depth-tested onto the terrain (rover occludes it) |

Godot stderr prints a benign `ERR_CANT_OPEN` (headless audio/driver probe) and ALSA
"all audio drivers failed → dummy driver" warnings (headless box, no sound device); rendering
is unaffected and every process exits 0.

---

## Foundation — physics authority + state-field producer

Pure-NumPy Tier-2 surrogate (`terrain_authority/`, **12 modules**: constants, io_fields,
column_state, procgen, sandpile, rover, quadtree, **refinement**, hexviz, scenes, tests,
`__init__`) emitting the frozen `INTERFACE.md` contract (now **v1.0.2** — additive optional
§5.2 per-wheel `wheel_tracks`/`drum_marks` + §5.3 variable-resolution `refinement`/`tiles`;
all rasters/dtypes/keys unchanged, `schema_version` stays `"1.0"`).

| Scene (`samples/`) | Frames | Total mass | Notes | Status |
|---|---|---|---|---|
| `flat_compact` | 1 | **6298.481 kg** | dense, near-zero relief, VIRGIN only (low-albedo proxy) | GREEN |
| `rolling_hills` | 1 | **6500.812 kg** | fbm loose top, disturbance ≤ 0.02 | GREEN |
| `crater` | 1 | **4606.247 kg** | Pike-class EXCAVATED bowl (labels 0..2) | GREEN |
| `boulder_field` | 1 | **5819.188 kg** | **186** Golombek-SFD clasts in metadata | GREEN |
| `crater_boulders` | 1 | **4840.711 kg** | crater + **143** clasts (excluded from fresh bowl, surface-snapped) | GREEN |
| `crater_caveins` | **102** (t000..t101) | drift **0.00e+00 kg** (4525.5909) | 400-step rim slump; raw frames git-excluded except bookends | GREEN |
| `tread_track` | **32** (t000..t031) | drift **0.00e+00 kg** (5622.1704) | driven-rover (single disc) VIRGIN→TREAD trail; per-frame quadtree metadata (active last=36, touched=208); bookends only | GREEN |
| `tread_track_4wheel` | **19** (t000..t018) | drift **0.00e+00 kg** (5521.7930) | **four separate** mass-conserving ruts (LF/RF/LB/RB) + §5.2 `wheel_tracks`; §5.3 `refinement` + **164** fine 1 cm `tiles` over the touched corridor at t018; bookends only (tile rasters git-excluded, descriptors in t018/metadata.json) | GREEN |
| `excavation_marks` | **2** (t000..t001) | drift **0.00e+00 kg** (6298.4813) | drum dig: **30.198 kg** EXCAVATED + dumped SPOIL (bulking: cut −0.04 m / spoil +0.059 m), §5.2 `drum_marks` on t001 | GREEN |

A 10th, script-generated **showcase** scene `tread_track_4wheel_1cm` (global 1 cm Mode A,
512² @ 0.01 m = 5.12 m; `scripts/build_flythrough_1cm.py`, NOT a canonical `scenes.py`
sample) backs the 1 cm fly-through; mass drift 0, bookends `t000`/`t024` committed (motion
frames git-excluded).

Each committed scene carries the 5 contract rasters (`heightmap`/`mass_areal`/`density`/
`disturbance` `.rf32` + `state_label.r8`) + `metadata.json` + `preview_*.png`. Terminal
`hexviz` (no file output) renders any field as dependency-free ASCII.

**Conservation + resolution invariants (spec §10, render_fidelity_spec.md §6), re-confirmed this pass — 18/18:**
- Total mass constant across cut→dump→relax, rel_drift **2.99e-16**.
- `height == datum + mass/density` after every op (cut/dump/relax/procgen/crater/wheel_pass): max_err **0.0**.
- Rover single pass preserves mass (density-only compaction; rut sinks): m0 == m1, **0.0** drift.
- Sandpile relaxation conserves mass (rel_drift **1.75e-16**) and leaves all loose slopes ≤ θ_r (35.57° vs 35.00°, within 1°).
- save/load round-trip preserves dims/dtype/row-major.
- **Quadtree:** leaves tile the field exactly once (65536/65536, no gaps/overlap); promotion monotone toward the rover (rover leaf 8 fine, far leaf 64 coarse); active-leaf count bounded (peak 36 of 64), cluster tracks the rover across all 32 frames.
- **§6.1 refine/coarsen round-trip exact for k ∈ {2,3,5,8}** (incl. the spec's k=8 mission config): field max-err, mass-copy, height-err all **0.0** — the operators copy homogeneous blocks verbatim so the round-trip is bit-exact for every integer k (not just k=2/4).
- **§6.2 base↔tile consistency:** every base cell over a tile == `coarsen()` of its fine cells (mass + area-mean height), max-err **0.0**; **§6.2b** zero-mass coarsen → finite density, height==datum, no NaN/inf; **§6.2c** non-uniform datum → coarse height == area-mean(child h) (err 2.78e-17); **§6.2d** non-integer / non-positive k rejected.
- **§6.3 toggle equivalence:** `refinement.enabled=false` build is byte-identical to the plain uniform base rasters.
- **§6.4 4-wheel separability:** straight drive → exactly 2 TREAD bands at ~gauge; turn → 4 distinct clusters. **§6.5** 4-wheel pass preserves mass (m0 == m1 = 328.729177, density-only).

## Native viz consumers (matplotlib, pure `load_scene` readers)

| Artifact | Size | Description | Status |
|---|---|---|---|
| `viz/out/variety_panel.png` | 1.21 MB | 2×2 grazing-sun hillshade: flat_compact / rolling_hills / crater / boulder_field | GREEN |
| `viz/out/caveins_filmstrip.png` / `caveins.gif` | 372 KB / 867 KB | 6-frame + 30-frame rim-slump cave-in | GREEN |
| `viz/out/groundtruth_crater.png` (+ `_turn0..2`) | 282 KB | D1b 3D bar3d cuboids by state_label + quadtree wireframes + clast scatter + turntable | GREEN |
| `viz/out/groundtruth_boulder_field.png` | 392 KB | VIRGIN cuboids + 186 clast spheres + quadtree | GREEN |
| `viz/out/tread_track.gif` / `tread_track_filmstrip.png` | 735 KB / 540 KB | driven-rover VIRGIN→TREAD compaction trail (mass conserved, rut via height=mass/density) | GREEN |
| `viz/out/quadtree_demo.gif` / `quadtree_demo_filmstrip.png` | 4.49 MB / 549 KB | **interaction-keyed quadtree**: active leaves (red) track the rover, touched trail (amber), coarse far (blue), under VIRGIN→TREAD + hillshade rut | GREEN |

## Godot render sidecar (headless Vulkan; D2 + D4 + v1.0.2 render fidelity)

`godot_sidecar/` — GDScript `INTERFACE.md` loader (`state_fields.gd`, parses the v1.0.1
quadtree keys **and** the v1.0.2 §5.2/§5.3 keys, baking `wheel_tracks`/`drum_marks` into a
track-direction field `tex_track_dir`), fine active-zone `ArrayMesh` + far-field LOD plane
(`terrain.gd`), and the **v1.0.2 fidelity shader** `terrain.gdshader` (lit regolith + detail-
normal granularity + per-wheel cleat ridges on TREAD + drum teeth ridges + capped parallax on
EXCAVATED/SPOIL, oriented by the baked track field). AA is set in `project.godot`
(`msaa_3d=2` 4×, `screen_space_aa=2` SMAA, `scaling_3d` Bilinear 1.5× SSAA). Plus the
false-color / dust / distortion shaders, articulated rover assembly, quadtree overlay
(now **depth-tested** so it sits on the terrain and the rover occludes it), and the
trailing chase-cam `--sequence` mode.

| Artifact | Size | Description | Status |
|---|---|---|---|
| `out/layer_1_heightmap.png` | 132 KB | unlit false-color elevation ramp | GREEN |
| `out/layer_2_state.png` | 13 KB | false-color state enum (grey VIRGIN + amber EXCAVATED) | GREEN |
| `out/layer_3_terrain.png` | 234 KB | lit crater under ~5° sun, deep + cast rim shadow, AA + detail-normal granularity | GREEN |
| `out/layer_4_clasts.png` | 234 KB | crater has 0 clasts → equals terrain (documented; real demo below) | GREEN |
| `out/layer_5_dust.png` | 240 KB | terrain + ballistic GPUParticles3D, lunar g, soft-haze puffs | GREEN |
| `out/layer_6_distortion.png` | 205 KB | terrain + Brown-Conrady radial barrel-warp post-process (stub) | GREEN |
| `out/boulder_terrain_clasts.png` | 298 KB | 186 sphere clasts, long grazing-sun shadows | GREEN |
| `out/crater_boulders.png` | 256 KB | crater + Golombek boulder field together | GREEN |
| `out/crater_boulders_rover.png` / `rover_on_terrain.png` | 237 KB / 239 KB | **articulated EZ-RASSOR** (chassis + 4 wheels + 2 arms + 2 drums, MIT) on the crater rim / rolling hills (AABB ~1.8×0.66×1.7 m, ground-snapped) | GREEN |
| `out/tread_track_4wheel_fidelity.png` / `_rover_fidelity.png` | 221 KB / 242 KB | **v1.0.2 headline**: four separate cleated tread ruts + 1 cm corridor; rover trailing the track | GREEN |
| `out/excavation_marks_fidelity.png` | 134 KB | **v1.0.2 headline**: drum trench — EXCAVATED cut, raised SPOIL lip, teeth-textured floor | GREEN |
| `out/quadtree_flythrough.gif` | 4.0 MB | **D4 headline (regenerated)**: global 1 cm, **1920×1080**, **trailing 3/4 chase cam**, 24 frames. Rover drives the path **nose-first** (heading-yaw fix) while the fine active-mesh window + the **depth-tested** quadtree LOD overlay follow it; AA + detail-normal + 4-wheel cleated track. Per-frame PNGs + 512² motion rasters git-excluded (regenerable via cmd 3 + cmd 12) | GREEN |
| `out/cube_on_plane.png` | 28 KB | original smoke test, intact (AA) | GREEN |

## Chrono Path A (executed; separate conda env — `docs/chrono_bringup_log.md`)

| Item | Result | Status |
|---|---|---|
| conda `chrono` env + **PyChrono 10.0.0** (`py312h98ab86c_677`) | installed; GDAL `.so.37` soname blocker resolved via `libgdal=3.11` | GREEN |
| stock `SCMTerrain` demo, headless, lunar g | 300 steps, `GetModifiedNodes`→261 nodes, exit 0 | GREEN |
| `scripts/chrono_scm_rover.py` | 400 steps, real 13.4 mm rut, 918 deformed nodes read back | GREEN |
| `scripts/chrono_scm_export.py` (+`_demo`) → INTERFACE | **PARTIAL**: heightmap + disturbance Chrono-sourced; mass_areal/density honest surrogate placeholders; round-trips via `io_fields`; `height = mass/density` invariant holds to **3.98e-08** | PARTIAL (by design) |

## Honest caveats (not defects)

1. **`crater_caveins` / `tread_track*` on-disk mass.** The "drift 0.0" figure is the in-memory
   float64 invariant. The `.rf32` contract stores `<f4`, so recomputing mass from the saved
   rasters shows float32 storage quantization (~1e-7 relative) — storage precision, not a
   conservation error. (The in-memory refine/coarsen round-trip is bit-exact; §6.1.)
2. **Rover is a static pose** (joints are fixed constants, not physics-driven; single-point
   ground-snap) — README §4 #11. In the `--sequence` fly-through it is placed at `rover_rc` and
   yawed along the path heading (forward = local +X; yaw = `atan2(-dz, dx)`).
3. **`tread_track_4wheel`'s four visually-separate ruts** are clearest on a pivot/sharp turn; a
   gentle drive sweeps the fore/aft wheels into two merged bands (physically correct).
4. **Quadtree manages render/space LOD, not solve cost** (the physics grid is still uniform-fine);
   the §5.3 `tiles[]` carry genuinely finer (1 cm) corridor data, but the Godot mesh does not yet
   build a finer corridor mesh from them (shader detail only) — README §4 #4 / a noted follow-up.
5. **Chrono is bootstrapped, not the live authority**; the exporter is partial (no §4.4
   mass-hybrid; bare test cylinder, not a Chrono::Vehicle) — README §4 #2.
6. **`layer_4_clasts.png` == `layer_3_terrain.png`** on the crater scene (0 clasts there); the
   genuine clast demos are `boulder_terrain_clasts.png` / `crater_boulders.png`.

No blocking issues. Every command above runs clean from the committed repo (`.venv` + the
vendored Godot binary for the renders; the `chrono` conda env for the Chrono scripts).
