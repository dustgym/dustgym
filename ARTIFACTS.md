# foss_ipex — Artifact Manifest & Verification

**Verified:** 2026-05-30, end-to-end from the committed repo (this commit — the **M1
camera→ROS2 sensor-bridge milestone**; head `aa058a1`/`573e126` carried the bridge, this pass
re-renders every reference still under the current pipeline and adds the bridge to the manifest).
The **authority + all matplotlib viz consumers were re-run from clean this pass** and their
exit codes / numbers below are from that run. **Godot:** the full reference render set was
**re-rendered this pass under the CURRENT render pipeline** — Hapke / Lommel-Seeliger BRDF
(terrain *and* clasts), angular faceted procgen boulders, 4× MSAA + SMAA + 1.5× SSAA,
detail-normal regolith shading — and committed; all exit 0, headless Vulkan renders are
reproducibly deterministic here. **ROS2 bridge:** the M1 camera→ROS2→AprilTag pose chain ran
end-to-end (real Godot render → `apriltag_ros` detects tag36h11 id 0 → camera→tag pose-vs-truth
**12.7 mm / 7.15°** on `flat_compact`); the depth + fiducial-overlay deliverables in
`out/ros/` were regenerated this pass. **Chrono** Path A was executed per
[`docs/chrono_bringup_log.md`](docs/chrono_bringup_log.md) in a separate conda env (not the
project `.venv`), so it is recorded, not re-run here.

**Verdict: ALL GREEN.** 18/18 conservation + quadtree + variable-resolution tests pass; all
**9** scenes re-export deterministically with masses matching; all **5** matplotlib consumers
and the full Godot render set (six diagnostic layers + boulder/clast demos + rover + the
4-wheel / excavation fidelity stills + the 1 cm trailing-cam fly-through) produce non-black,
varied PNGs; the **M1 sensor bridge** writes a schema-valid `sensors.json` + front-stereo PNGs,
its REP-103 frame seam passes 5/5 unit tests, and `apriltag_ros` detects id 0 with a real
cm/deg pose-vs-truth reading; PyChrono 10.0.0 runs an SCM rover at lunar g and a partial
contract exporter round-trips through the frozen `io_fields`.

This manifest is a verification snapshot; [`README.md`](README.md) is the authoritative
description of the system and what is papered over, and
[`docs/spec_coverage.md`](docs/spec_coverage.md) is the §-by-§ scorecard against the master spec.

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
| 8 | `render_layers.sh -- --scene ../samples/crater --layers L --pose 2.56,3.0,6.4,2.56,-0.1,2.56 --size 1024x768 --out layer_{1..6}_*.png` (one call per `L`∈{heightmap,state,terrain,clasts,dust,distortion}) | 0 | six diagnostic layers, **Hapke-lit** crater; AA + detail-normal pipeline. `layer_4_clasts` == `layer_3_terrain` byte-for-byte (crater has 0 clasts — documented) |
| 9 | `render_layers.sh -- --scene ../samples/boulder_field --layers terrain,clasts --size 1024x768 --out boulder_terrain_clasts.png` | 0 | "placed 186 clasts" — **angular faceted** Golombek boulders, long grazing-sun shadows on Hapke regolith |
| 10 | `render_layers.sh -- --scene ../samples/crater_boulders --layers terrain,clasts --size 1024x768 --out crater_boulders.png` | 0 | "placed 143 clasts" — Pike-class bowl ringed by the angular boulder field |
| 11 | `render_layers.sh -- --scene ../samples/crater_boulders --layers terrain,clasts,rover --pose 1.7,1.05,1.5,3.7,0.05,3.2 --size 1024x768 --out crater_boulders_rover.png` | 0 | "placed 143 clasts" + articulated EZ-RASSOR ground-snapped on the rim (AABB 1.83×0.66×1.70 m) |
| 12 | `render_layers.sh -- --scene ../samples/rolling_hills --layers terrain,rover --size 1024x768 --out rover_on_terrain.png` | 0 | articulated EZ-RASSOR on rolling terrain (AABB 1.83×0.66×1.70 m, root-snapped) |
| 13 | `render_layers.sh -- --scene ../samples/tread_track_4wheel/t018 --layers terrain[,rover] --size 1024x768 --out tread_track_4wheel*_fidelity.png` | 0 | four-wheel cleated tracks + 1 cm corridor; detail/teeth shading; terrain + rover-trailing variants |
| 14 | `render_layers.sh -- --scene ../samples/excavation_marks/t001 --layers terrain --size 1024x768 --out excavation_marks_fidelity.png` | 0 | drum trench: EXCAVATED cut + raised SPOIL lip + teeth-textured floor |
| 15 | `render_layers.sh -- --sequence ../samples/tread_track_4wheel_1cm --stride 1 --size 1920x1080 --layers terrain,quadtree,rover` | (deferred) | 24-frame 1080p **trailing chase-cam** fly-through — **NOT re-rendered this pass** (already Hapke, regenerated recently; the committed `quadtree_flythrough.gif` is current) |
| 16 | `render_layers.sh -- --scene <s> --cameras --layers terrain,clasts,rover …` | 0 | M1 front-stereo egress: `out/cam/<s>/000/{front_left,front_right}.png` + schema-valid `sensors.json` + procedural AprilTag lander (gitignored render output) |
| 17 | docker: `docker compose build` → `foss_ipex/ros2_bridge:jazzy`; `python3 test_frames.py` (5/5); `bag_writer.py` → rosbag2 MCAP; `ros2 launch apriltag_bringup.launch.py` + `compare_pose.py` | 0 | bridge integration: REP-103 frames green, `apriltag_ros` detects tag36h11 id 0, camera→tag **12.7 mm / 7.15°** vs truth on `flat_compact` (bags under `scripts/ros2_bridge/bags/`, gitignored) |
| 18 | `depth_map.py --in out/cam/<s>/000 --out …` + `fiducial_overlay.py --in out/cam/<s>/000 --out …` (in the container) | 0 | the `out/ros/` deliverables: StereoSGBM colorized metric depth + AprilTag fiducial highlight at 3 distances/angles (all detect id 0) |

The §3 REP-103 frame unit tests were also confirmed green on the repo `.venv` (pure-python/numpy):
`.venv/bin/python scripts/ros2_bridge/test_frames.py` → **5/5 PASS** (the 3 contract-required maps
+ optical sanity + the lander→tag relabel). Godot stderr prints a benign `ERR_CANT_OPEN`
(headless audio/driver probe) and ALSA "all audio drivers failed → dummy driver" warnings
(headless box, no sound device); rendering is unaffected and every render process exits 0.

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
| `boulder_field` | 1 | **5819.188 kg** | **186** Golombek-SFD clasts in metadata (render angular/faceted) | GREEN |
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

## Godot render sidecar (headless Vulkan; D2 + D4 + Hapke BRDF + angular clasts + M1 sensor egress)

`godot_sidecar/` — GDScript `INTERFACE.md` loader (`state_fields.gd`, parses the v1.0.1
quadtree keys **and** the v1.0.2 §5.2/§5.3 keys, baking `wheel_tracks`/`drum_marks` into a
track-direction field `tex_track_dir`), fine active-zone `ArrayMesh` + far-field LOD plane
(`terrain.gd`), and the fidelity shaders. The terrain shader `terrain.gdshader` is now lit by a
**Hapke IMSA / Lommel-Seeliger BRDF** (2-term Henyey-Greenstein phase + shadow-hiding opposition
surge + Chandrasekhar multiple-scattering H-function; Sato 2014 / Hapke 2002 — replaces Lambert)
over detail-normal granularity, per-wheel cleat ridges on TREAD, drum teeth ridges, and capped
parallax on EXCAVATED/SPOIL (oriented by the baked track field). Boulders use the **angular
faceted** clast shader `clast.gdshader` (conchoidal-fracture ridged + terraced relief, Tsuchiyama
2022 triaxial axial ratios — no longer smooth spheres), also Hapke-lit. AA is set in
`project.godot` (`msaa_3d=2` 4×, `screen_space_aa=2` SMAA, `scaling_3d` Bilinear 1.5× SSAA).
Plus the false-color / dust / distortion shaders, articulated rover assembly, quadtree overlay
(depth-tested so it sits on the terrain and the rover occludes it), the trailing chase-cam
`--sequence` mode, and the **M1 `--cameras` sensor egress** (front-stereo `camera_rig.gd` +
procedural-AprilTag lander `apriltag_gen.gd`).

| Artifact | Size | Description | Status |
|---|---|---|---|
| `out/layer_1_heightmap.png` | 133 KB | unlit false-color elevation ramp | GREEN |
| `out/layer_2_state.png` | 13 KB | false-color state enum (grey VIRGIN + amber EXCAVATED) | GREEN |
| `out/layer_3_terrain.png` | 192 KB | **Hapke/Lommel-Seeliger-lit** crater under ~5° sun, deep + cast rim shadow, AA + detail-normal granularity | GREEN |
| `out/layer_4_clasts.png` | 192 KB | crater has 0 clasts → **byte-identical to `layer_3_terrain.png`** (md5-confirmed; documented, real clast demos below) | GREEN |
| `out/layer_5_dust.png` | 198 KB | terrain + ballistic GPUParticles3D, lunar g, soft-haze puffs | GREEN |
| `out/layer_6_distortion.png` | 171 KB | terrain + Brown-Conrady radial barrel-warp post-process (stub) | GREEN |
| `out/boulder_terrain_clasts.png` | 250 KB | **186 angular/faceted clasts** (not spheres) on Hapke regolith, long grazing-sun shadows | GREEN |
| `out/crater_boulders.png` | 207 KB | Pike-class bowl (lit rim, black interior) ringed by the angular Golombek boulder field, Hapke-lit | GREEN |
| `out/crater_boulders_rover.png` / `rover_on_terrain.png` | 198 KB / 218 KB | **articulated EZ-RASSOR** (chassis + 4 wheels + 2 arms + 2 drums, MIT) on the crater rim / rolling hills (AABB 1.83×0.66×1.70 m, ground-snapped); Hapke terrain + angular clasts | GREEN |
| `out/tread_track_4wheel_fidelity.png` / `_rover_fidelity.png` | 204 KB / 210 KB | four separate cleated tread ruts + 1 cm corridor on Hapke regolith; rover trailing the track | GREEN |
| `out/excavation_marks_fidelity.png` | 136 KB | drum trench — EXCAVATED cut, raised SPOIL lip, teeth-textured floor, Hapke-lit | GREEN |
| `out/quadtree_flythrough.gif` | 4.5 MB | **D4 headline (already Hapke; NOT re-rendered this pass)**: global 1 cm, **1920×1080**, **trailing 3/4 chase cam**, 24 frames. Rover drives the path **nose-first** while the fine active-mesh window + the depth-tested quadtree LOD overlay follow it; AA + detail-normal + 4-wheel cleated track. Per-frame PNGs + 512² motion rasters git-excluded (regenerable via cmd 3 + cmd 15) | GREEN |
| `out/cube_on_plane.png` | 28 KB | original smoke test, intact (AA) | GREEN |

## M1 camera → ROS2 sensor bridge (the second frozen seam — `docs/sensor_bridge_contract.md`)

A NEW Godot-camera → ROS2 → AprilTag pose chain — the M1 *"basic comms established"* milestone.
The renderer (producer) and the ROS2 container (consumer) **never share memory or types**; they
agree only on the on-disk `sensors.json` + PNG contract (the dev-time analogue of `INTERFACE.md`),
so the camera-rig, boulder, and ROS-container tracks were built in parallel worktrees and merged
clean.

**G1 — Godot egress (`--cameras` mode):** `sidecar.gd --cameras` mirrors the proven multi-camera
SubViewport capture to write `out/cam/<scene>/000/{front_left,front_right}.png` + a schema-valid
`sensors.json` (`sensor_bridge/1.0`: real calibrated pinhole intrinsics `fx=fy=(w/2)/tan(fov/2)`,
cx, cy from the Godot FOV; 0.100 m stereo baseline `[CALIB]`; all poses Godot-native, Y-up).
`camera_rig.gd` is the declarative front-stereo extrinsics table + builder (mast-mounted pair on
the rover root, forward = local +X toward the lander). `apriltag_gen.gd` procedurally generates
the canonical **tag36h11 id-0** bitmap (the exact AprilRobotics codebook pixels, baked offline)
and renders it **UNLIT** (a matte fiducial, not Hapke regolith) on the lander's rover-facing face.
The `out/cam/` egress is git-ignored render output (regenerable from the committed `.gd`).

**C1 — ROS2 container (`scripts/ros2_bridge/`):**

| file | purpose |
|---|---|
| `Dockerfile` | image `foss_ipex/ros2_bridge:jazzy` on **`osrf/ros:jazzy-desktop`** + `apriltag_ros` + `stereo_image_proc` + `rosbag2-storage-mcap` + the pure-python `rosbags` writer |
| `docker-compose.yml` | mounts `../../godot_sidecar/out` (ro) + `bags/` (rw) |
| `frames.py` + `test_frames.py` | the **REP-103 Godot Y-up → ROS Z-up** seam, solved in ONE place (world Y-up→Z-up; cam→optical; positions + orientations) + the 5 guard unit tests (**5/5 PASS** this pass) — the classic silent-sign-flip SLAM bug, named→solved |
| `bag_writer.py` | `out/cam/.../NNN/` → rosbag2 **MCAP**, applying the §3 conversion once + computing camera→tag truth (`/lander/apriltag_truth`) |
| `apriltag_bringup.launch.py` | launches `apriltag_node` on `/front_left` (M2 stereo/SLAM stubbed) |
| `compare_pose.py` | joins detected `/tf` tag pose with `/lander/apriltag_truth`, prints translation (mm) + rotation (deg) error |
| `tags_36h11.yaml` | detector config: family `36h11`, id 0, size 0.150 m, `pnp` estimator |
| `depth_map.py` | StereoSGBM disparity → metric depth `Z = fx·baseline/disp`, colorized PNG (produces `out/ros/depth_*.png`) |
| `fiducial_overlay.py` | runs the detector on `front_left.png`, draws the detected tag quad + id + solvePnP pose axes + range (produces `out/ros/fid_*.png`) |
| `fixtures/` | hand-authored `sensors.json` + tag-bearing PNGs (the contract §2.4 stand-in; lets C1 build green before G1 lands) |

**Verified result (the spec §10 pose-error channel's first real reading):** a real Godot
`--cameras` render → `apriltag_ros` **detects tag36h11 id 0** → `compare_pose.py` reports
camera→tag pose-vs-truth of **12.7 mm / 7.15°** on `flat_compact` (also 13.3 mm / 5.10° on
crater_boulders; 29.9 mm / 1.46° on an oblique view). The ~5–7° residual on head-on views is the
real near-fronto-parallel PnP ambiguity, not a bridge bug. Landed in commit `573e126`
(tag-frame-convention fix: rotation 124.6° → ~7°) / merge `aa058a1`. Bags written under
`scripts/ros2_bridge/bags/` (gitignored).

### ROS output artifacts (`out/ros/` — committed deliverables)

| Artifact | Size | Description | Status |
|---|---|---|---|
| `out/ros/depth_boulders.png` | 49 KB | StereoSGBM metric depth on the boulder field, TURBO-colorized (near=warm, far=cool); **sparse (black) where the low-texture regolith can't stereo-match** — honest, not hidden | GREEN |
| `out/ros/depth_crater_drop.png` | 31 KB | StereoSGBM metric depth of the crater-drop view; same sparse-where-untextured behavior | GREEN |
| `out/ros/fid_close.png` | 63 KB | AprilTag fiducial highlight at close range — detects id 0, green quad + center + `tag36h11:0` label + solvePnP pose axes + `range 1.42 m` | GREEN |
| `out/ros/fid_oblique.png` | 48 KB | fiducial highlight at an oblique angle — detects id 0 with pose axes + range | GREEN |
| `out/ros/fid_far.png` | 34 KB | fiducial highlight at far range — detects id 0 with pose axes + range | GREEN |
| `out/ros/crater_drop_hero.png` | 263 KB | hero render: the articulated EZ-RASSOR dropped into a crater bowl, grazing-sun shadows on Hapke regolith, AprilTag lander glinting in the distance | GREEN |

## Spiral demo battery — the "larger, longer" Haworth visualization (commits `c560981` → `54e1ef6` → `3a3aef3`)

*Added 2026-05-31 from the spiral-demo commit chain. The battery was executed at build time — the
detection counts and resident-memory figures below are from those runs — and is recorded here **from
the commits, not re-executed for this manifest update**. Render egress (`out/cam/`, `out/scenes/`) and
the built `_driven` scene are gitignored (regenerable); the `out/panels/` GIFs/PNGs are the committed
deliverables.*

The capstone demo stacks the whole pipeline on a **real Haworth DEM window** (~220 m): a fixed-center
LM-class lander carries a **2.5 m four-face AprilTag bundle** (`lander_bundle.gd`, scaled 0.15→2.5 m off
`BODY_SIZE`/`TAG_PROUD_M`/`TAG_SIZE_M` so `size_m` + `pose_in_lander` stay parametric — ~100 m
theoretical range at the ideal 1024 px / 74° pinhole), and a rover **spirals out 80 frames** (16/lap ×
5 laps, 15→~105 m) localizing off the bundle. The failures — range, grazing-sun shadow, self/terrain
occlusion — are the data point.

**Rover-physics pass (`3a3aef3`).** Replaces the single-point ground-snap with a **kinematic 4-wheel
terrain conform** (`rover.conform_pose`: a macro-slope stencil that stays stable on the coarse 0.5 m DEM
+ capped clast ride-over → the rover seats on its four wheels and tilts pitch/roll with the surface).
Heading is **travel-tangent** (front faces the next spiral waypoint, not the lander); the **side mono**
(`left_mono`) acquires the fiducial while the front stereo looks along travel. `drive_spiral.py`
(producer) emits a per-frame `rover_pose.json` + carves **four separate mass-conserving ruts** into a
`<scene>_driven` heightmap; the compaction trail accumulates as **polyline markup** on both top-downs
(the 2 cm grouser cleats are sub-pixel at any rover+origin-in-frame zoom, so they can't render as
in-engine terrain features). Still **kinematic** — no contact forces / slip (Chrono::Vehicle+SCM remains
the deferred producer swap), joints still fixed constants. `terrain_authority.tests` adds a 19th check
(`test_conform_pose_flat_ramp_clast`: flat→upright, ramp→pitch=atan(slope), clast→capped ride-over tilt)
→ **19/19**.

**Top-down render mode (`54e1ef6`).** A net-new near-overhead pass frames the whole 220 m patch: a
**LIT** variant (grazing sun 7° / 135° az recovered from `sensors.json`, `--exposure 6` → genuine relief
+ long boulder/crater shadows) and an **UNLIT** variant (Lambert, shadows-off, spherical clasts, the
in-engine **quadtree-LOD overlay** fed per-frame from `qt_leaves.json` — the fine LOD cells track the
rover down the corridor). Terrain-cull fix: a perspective near-overhead camera + `_uncull_terrain()`
extra cull margin defeats the vertex-shader-displacement AABB mismatch that frustum-culled the far-field
plane (terrain brightness 11 → 138).

**Detection / localization.** `detect_spiral.py` runs container-side detect+PnP (IPPE_SQUARE) →
`rover_localize` back-out per face, with the solvePnP↔apriltag_ros 180°-about-X convention fix
(`_R_X180`; established by `_diag_pose.py`). Side-mono detection on the travel-tangent run: **21/80 lit,
47/80 unlit** — matches the old aimed front-stereo (`c560981`: **22/80 lit, 45/80 unlit**), confirming
the "face travel, glimpse the lander with the side cam" honesty holds the localization rate.

**Resource record.** `instrument_spiral.py` drives a 2 cm-corridor `TileMosaic` + `QuadtreeTracker`
along the same path → `resource.json`: an **O(corridor) resident ~21 MB** record vs **3.99 GB** for a
dense 2 cm tiling of the whole patch — the concrete §4 LOD payoff made legible.

| Committed deliverable (`out/panels/`) | Size | Description | Status |
|---|---|---|---|
| `failure_breakdown.png` | 34 KB | LIT 22/80 vs UNLIT 45/80 AprilTag-localization rate — the illumination A/B at 100 m scale, attributed by range / shadow / occlusion | GREEN |
| `position_slam_lit.gif` / `position_slam_unlit.gif` | 599 KB / 728 KB | truth vs AprilTag SLAM, lander-centered, quadrants shaded by visible face | GREEN |
| `resource.gif` | 328 KB | ~21 MB resident corridor record vs 3.99 GB dense-2 cm | GREEN |
| `composite_2x2.gif` | 37.1 MB | lit/unlit top-down + position-SLAM + resource, synced on the 80-frame spiral | GREEN |
| `composite_3x2.gif` | 42.6 MB | + side-mono rover-cam + stacked LIT-vs-UNLIT failure | GREEN |

Honesty rails preserved across the battery: idealized **noiseless pinhole** (errors are the
geometric/subpixel floor + PnP fronto-parallel flips at range, **not** distortion-inclusive); the rover
pose is **kinematic, not force-accurate**; the `_driven` scene + `out/cam/` egress are gitignored
(regenerable from the committed `.gd`/`.py`).

## Chrono Path A (executed; separate conda env — `docs/chrono_bringup_log.md`)

| Item | Result | Status |
|---|---|---|
| conda `chrono` env + **PyChrono 10.0.0** (`py312h98ab86c_677`) | installed; GDAL `.so.37` soname blocker resolved via `libgdal=3.11` | GREEN |
| stock `SCMTerrain` demo, headless, lunar g | 300 steps, `GetModifiedNodes`→261 nodes, exit 0 | GREEN |
| `scripts/chrono_scm_rover.py` | 400 steps, real 13.4 mm rut, 918 deformed nodes read back | GREEN |
| `scripts/chrono_scm_export.py` (+`_demo`) → INTERFACE | **PARTIAL**: heightmap + disturbance Chrono-sourced; mass_areal/density honest surrogate placeholders; round-trips via `io_fields`; `height = mass/density` invariant holds to **3.98e-08** | PARTIAL (by design) |

## Docs / reference

| Doc | Purpose |
|---|---|
| [`README.md`](README.md) | authoritative system description + what is papered over (§4 honest-accounting table) |
| [`INTERFACE.md`](INTERFACE.md) | the frozen physics↔render on-disk contract (v1.0.2) |
| [`docs/sensor_bridge_contract.md`](docs/sensor_bridge_contract.md) | the FROZEN G1↔C1 camera-egress ↔ ROS2 seam (AprilTag spec, `sensors.json` schema, REP-103 conversion) — the dev-time analogue of `INTERFACE.md` for the M1 bridge |
| [`docs/spec_coverage.md`](docs/spec_coverage.md) | the §1–§12 scorecard against `ipex-terrain-sim-spec.md` (✅/🟡/🔵/⬜ by section, evidence by file:line/commit/artifact) |
| [`docs/render_fidelity_spec.md`](docs/render_fidelity_spec.md) | the render-fidelity spec (§9 Hapke BRDF derivation) |
| [`docs/chrono_bringup_log.md`](docs/chrono_bringup_log.md) | the PyChrono Path A bring-up log (env, SCM rover, partial exporter) |

## Honest caveats (not defects)

1. **`crater_caveins` / `tread_track*` on-disk mass.** The "drift 0.0" figure is the in-memory
   float64 invariant. The `.rf32` contract stores `<f4`, so recomputing mass from the saved
   rasters shows float32 storage quantization (~1e-7 relative) — storage precision, not a
   conservation error. (The in-memory refine/coarsen round-trip is bit-exact; §6.1.)
2. **Rover joints are static** (fixed constants, not physics-driven) — README §4 #11. In the
   `--sequence` fly-through it is placed at `rover_rc` and yawed along the path heading (forward =
   local +X; yaw = `atan2(-dz, dx)`). The spiral demo (`3a3aef3`) **upgrades the body placement** from
   single-point ground-snap to a **kinematic 4-wheel terrain conform** (`rover.conform_pose` seats it on
   its four wheels and tilts pitch/roll with the surface + capped clast ride-over, travel-tangent
   heading) — but it stays **kinematic**: no contact forces, no slip-sinkage, and the joints are still
   fixed constants. Chrono::Vehicle+SCM is the deferred producer swap.
3. **`tread_track_4wheel`'s four visually-separate ruts** are clearest on a pivot/sharp turn; a
   gentle drive sweeps the fore/aft wheels into two merged bands (physically correct).
4. **Quadtree manages render/space LOD, not solve cost** (the physics grid is still uniform-fine);
   the §5.3 `tiles[]` carry genuinely finer (1 cm) corridor data, but the Godot mesh does not yet
   build a finer corridor mesh from them (shader detail only) — README §4 #4 / a noted follow-up.
5. **Chrono is bootstrapped, not the live authority**; the exporter is partial (no §4.4
   mass-hybrid; bare test cylinder, not a Chrono::Vehicle) — README §4 #2.
6. **`layer_4_clasts.png` == `layer_3_terrain.png`** on the crater scene (0 clasts there; this
   pass confirmed they are byte-identical by md5); the genuine clast demos are
   `boulder_terrain_clasts.png` / `crater_boulders.png`.
7. **Stereo depth is honestly sparse.** `out/ros/depth_*.png` is black wherever the low-texture
   lunar regolith gives StereoSGBM no features to match — that is the real airless-regolith
   stereo limitation surfaced, not a render bug. The §10 *map* (observed-vs-true-terrain) channel
   and a scoring harness are still unbuilt (`docs/spec_coverage.md` §10).
8. **Camera distortion stays a stub** for M1 (rectified pinhole, `D=[0,0,0,0,0]`); the AprilTag
   uses the `pnp` estimator and the lander origin == tag center (identity `pose_in_lander`), with
   the fixed lander→tag axis-relabel applied once in `bag_writer._compute_truth`
   (`sensor_bridge_contract.md` §1). The ~5–7° head-on rotation residual is PnP near-fronto-parallel
   ambiguity, not a frame error.

No blocking issues. Every render/verification command above runs clean from the committed repo
(`.venv` + the vendored Godot binary for the renders; the `foss_ipex/ros2_bridge:jazzy` container
for the bridge; the `chrono` conda env for the Chrono scripts).
