# foss_ipex — Sensor-Faithful Lunar Terrain Simulation (Weekend Vertical Slice)

A **calibrated, sensor-faithful lunar terrain simulator** for IPEx perception and human-in-the-loop
(HITL) autonomy work — judged at the **camera output**, not the force vector (spec §1). The pass/fail
question is *"does the perception stack see what it would see on the testbed and at the pole?"*, so the
product is exposed-sublayer albedo, rut/shadow geometry, and ballistic dust placement, not contact-force
accuracy. This repository is a **weekend vertical slice** that proves the whole pipeline end to end:
procedural terrain → state-field authority → headless renderer → diagnostic views. It targets **Tier 2**
(spec §3): coupled, semi-empirical analytical terramechanics (Bekker/Janosi/Wong-Reece geometry,
heightfield carving, rocks as rigid bodies, dust driven by disturbance) — closing the path-dependent loop
*without* a live granular solver. The physics authority here is a **pure-NumPy analytical surrogate
standing in for Project Chrono** behind a **frozen on-disk contract** ([`INTERFACE.md`](INTERFACE.md)):
the producer writes a directory of raw state-field rasters, consumers only ever read that directory, and
neither shares memory or types with the other. **Chrono (Chrono::Vehicle + SCM deformable terrain) is a
drop-in replacement for the producer with zero consumer changes — that decoupling seam *is* the
architecture** (spec §2 authority model, §4 physics↔render interface).

---

## 1. Pipeline (BUILT vs STUBBED)

Adapted from spec §2. `[BUILT]` ships in this slice; `[SURROGATE]` is the NumPy stand-in for the eventual
Chrono authority (same on-disk contract); `[STUB]`/`[TODO]` is named-but-not-implemented.

```
  ┌─────────────────────────┐   state fields    ┌──────────────────────┐  synthetic   ┌──────────────────┐
  │   PHYSICS AUTHORITY      │  (.rf32 / .r8     │       Godot          │  imagery +   │  Robot / ROS2    │
  │                          │   raster dir +    │   (RENDER + SENSOR    │  sensor data │  debug env       │
  │ terrain_authority/       │   metadata.json)  │    MODEL)            │ ───────────▶ │  SLAM stack      │
  │  • procgen     [BUILT]   │ ────────────────▶ │ godot_sidecar/        │              │                  │
  │  • sandpile CA [BUILT]   │   INTERFACE.md    │  • lunar lighting [B] │              │ • sensor noise   │
  │  • rover pass  [BUILT]   │   (FROZEN v1.0)   │  • dust shaders   [B] │              │   ............[TODO]
  │  • column mass [BUILT]   │                   │  • far-field LOD  [B] │              │ • SLAM/mapping   │
  │  ──────────────────────  │                   │  • cam intrinsics [STUB-projection]  │   ............[TODO]
  │  Chrono::Vehicle + SCM    │                  │  • dirty lens     [STUB-Brown-Conrady]└────────┬─────────┘
  │  + clasts as rigid bodies │                  └──────────────────────┘                        │
  │                [DROP-IN, SURROGATE today]                                                     │
  └────────┬──────────────────┘                                                                  │
           │  ground truth (true pose, true terrain at time t)  [viz/groundtruth_viz.py = BUILT]  │
           └───────────────────────────────────────────────────────────────────────────────────-┘
                                       EVALUATION  (two-channel: pose + map)  [TODO]
```

- **BUILT:** the full NumPy Tier-2 authority (mass-conserving column model, procgen, sandpile relaxation,
  single-pass rover), the frozen state-field I/O contract, the Godot render-only sidecar with toggleable
  diagnostic layers and a far-field LOD demo, and four diagnostic/portfolio viewers.
- **SURROGATE (decoupled, swappable):** the authority is analytical geometry, not Chrono. It emits the
  *exact* `INTERFACE.md` format Chrono will emit, so the swap is producer-side only.
- **STUB / TODO:** camera custom-projection + Brown-Conrady distortion (a render-only barrel-warp demo
  exists, not a calibrated intrinsic model), the downstream Robot/ROS2 + SLAM env, and the two-channel
  evaluation (SLAM pose vs. true pose; observed map vs. true terrain at time t, spec §2/§10).

---

## 2. What's here (by deliverable)

Run everything with the in-repo venv: `.venv/bin/python …`. Outputs already live under `viz/out/` and
`godot_sidecar/out/`.

### D1a — terminal hex ground-truth viz (dependency-free)
ASCII hex map of any state field; the "can I see the terrain in a terminal" check that needs nothing but
NumPy. Reads the frozen contract via `terrain_authority.io_fields.load_scene`.

```bash
.venv/bin/python -m terrain_authority.hexviz samples/crater --field state_label
.venv/bin/python -m terrain_authority.hexviz samples/rolling_hills --field heightmap
```
Output: text to stdout (downsampled ~64×32, +Z up on screen, min/max legend; for `state_label`, prints the
`0 VIRGIN … 4 COMPACTED_BERM` enum key).

### D1b — native matplotlib 3D ground-truth viz
The spatial state the robot's SLAM would have to reconstruct (spec §2 "true terrain at time t"). Three
world-coordinate layers: active-zone columns as `bar3d` cuboids colored by `state_label`, quadtree
non-leaf nodes as blue wireframe boxes (spec §4: the tree manages *space*, not physics), and clasts as red
spheres sized by radius.

```bash
.venv/bin/python viz/groundtruth_viz.py samples/crater --turntable
.venv/bin/python viz/groundtruth_viz.py samples/boulder_field
```
Outputs: [`viz/out/groundtruth_crater.png`](viz/out/groundtruth_crater.png),
[`viz/out/groundtruth_boulder_field.png`](viz/out/groundtruth_boulder_field.png),
turntable sweep [`viz/out/groundtruth_crater_turn0.png`](viz/out/groundtruth_crater_turn0.png),
[`turn1`](viz/out/groundtruth_crater_turn1.png), [`turn2`](viz/out/groundtruth_crater_turn2.png).

### D2 — Chrono→Godot state-field handoff + far-field LOD
GDScript consumer of `INTERFACE.md` (`godot_sidecar/state_fields.gd`): raw `FileAccess.get_buffer` →
`Image.create_from_data` with `FORMAT_RF`/`FORMAT_R8`, no EXR/PNG in the hot path. Builds a fine
active-zone `ArrayMesh` sampling the authoritative heightmap per vertex, plus a single low-subdivision
far-field plane displaced from a decimated tile in a vertex shader — the spec §4 LOD efficiency demo
(physics resolution ≠ render resolution; the two clocks need not match). Exercised by the D4 layers below.

### D3 — procgen variety panel + cave-in showpiece
Portfolio slides, pure consumers of the frozen contract (`viz/variety_panel.py`).

```bash
.venv/bin/python viz/variety_panel.py
```
Outputs:
- [`viz/out/variety_panel.png`](viz/out/variety_panel.png) — 2×2 grazing-sun hillshade: `flat_compact`
  (dense ρ≈1920), `rolling_hills` (loose fbm ρ≈1300), `crater` (Pike-class bowl D=2.4 m), `boulder_field`
  (186 Golombek-SFD clasts as rigid-body refs, not carved into mass; spec §6).
- [`viz/out/caveins_filmstrip.png`](viz/out/caveins_filmstrip.png) — 6-frame sandpile cave-in: an
  over-piled rim ridge slumps from +1.90 m to repose at +0.25 m. Mass is conserved: the in-memory
  float64 invariant is exact to ~3e-16 (`terrain_authority.tests`); the few-kg figure on the
  filmstrip is float32 raster-**storage** quantization (it recomputes mass from the saved `.rf32`
  rasters across 100 frames) — shown deliberately, to be honest about the on-disk contract.
- [`viz/out/caveins.gif`](viz/out/caveins.gif) — 30-frame animated version of the same slump.

### D4 — layer-toggle Godot render sidecar (headless)
Compositor that renders lunar-lit views with toggleable diagnostic layers (`heightmap`, `state`,
`terrain`, `clasts`, `rover`, `dust`, `distortion`). Single ~5° hard sun, ambient/SSIL/SDFGI/glow off, near-black
background — Godot's strong suit and exactly IPEx's grazing-angle perception challenge (spec §8).

```bash
cd godot_sidecar
./render_layers.sh -- --scene ../samples/crater --layers terrain,clasts \
    --pose 2.56,3.0,6.4,2.56,-0.1,2.56 --size 1024x768 --out layer_3_terrain.png
```
Outputs (all 1024×768 unless noted):
- [`godot_sidecar/out/layer_1_heightmap.png`](godot_sidecar/out/layer_1_heightmap.png) — false-color
  elevation ramp (floor blue, rim red).
- [`godot_sidecar/out/layer_2_state.png`](godot_sidecar/out/layer_2_state.png) — VIRGIN grey + EXCAVATED
  amber (only labels 0,2 present; flat enum regions compress to ~6 KB but are correct).
- [`godot_sidecar/out/layer_3_terrain.png`](godot_sidecar/out/layer_3_terrain.png) — lit PBR regolith,
  deep crater + cast rim shadow (the spec §8 perception hazard).
- [`godot_sidecar/out/layer_4_clasts.png`](godot_sidecar/out/layer_4_clasts.png) — crater has 0 clasts, so
  this equals terrain (the genuine clast demo is below).
- [`godot_sidecar/out/layer_5_dust.png`](godot_sidecar/out/layer_5_dust.png) — ballistic GPUParticles3D
  lofted from the most-disturbed cells, lunar g=1.62, no drag (render-only, spec §8). Soft
  radial-alpha puffs that accumulate into haze (not hard-edged sprites).
- [`godot_sidecar/out/layer_6_distortion.png`](godot_sidecar/out/layer_6_distortion.png) — Brown-Conrady
  radial barrel-warp post-process (stub, see §5).
- [`godot_sidecar/out/boulder_terrain_clasts.png`](godot_sidecar/out/boulder_terrain_clasts.png) —
  **headline render**: 186 sphere clasts on lit regolith, each casting a long grazing-sun shadow.
- [`godot_sidecar/out/crater_boulders.png`](godot_sidecar/out/crater_boulders.png) — **craters +
  boulders together**: a Pike-class bowl (lit rim, black interior — the spec §8 grazing-light hazard)
  ringed by a Golombek boulder field (clasts excluded from the fresh bowl, surface-snapped).
- [`godot_sidecar/out/crater_boulders_rover.png`](godot_sidecar/out/crater_boulders_rover.png) /
  [`rover_on_terrain.png`](godot_sidecar/out/rover_on_terrain.png) — the **real EZ-RASSOR chassis**
  (`base_unit`, MIT, vendored — see [`THIRD_PARTY.md`](THIRD_PARTY.md)) loaded at runtime via
  `GLTFDocument`, surface-snapped at a crater rim / on rolling terrain. Chassis only (no
  wheels/drums yet — see §4 #11).

> Terrain meshing note: the active-zone mesh samples the heightmap **bilinearly** (not nearest), and
> the far-field LOD plane recomputes per-vertex normals from the height gradient — without these the
> crater wall terraces and bands under the 5° sun, and the far field shades flat. Both are in
> `terrain.gd` / `terrain_farfield.gdshader`.

### Sample scenes (`samples/`, the frozen-contract corpus)
`flat_compact`, `rolling_hills`, `crater`, `boulder_field`, and `crater_caveins` (102-frame
`t000…t101` time series). All 256×256 at 2 cm/cell. See each `metadata.json` for parameters and citations.

---

## 3. Quickstart

Prereqs are vendored in-repo — **do not `pip install` anything**:
- Python venv: `.venv/` (numpy 2.4, scipy 1.17, matplotlib 3.10, pillow 12).
- Godot binary: `.tools/godot/Godot_v4.6.3-stable_linux.x86_64`.
- Headless render wrapper: `godot_sidecar/render.sh` (runs Godot under `xvfb` + the NVIDIA Vulkan ICD on
  the RTX 4090; `--headless` is *not* used because it disables the real driver and `frame_post_draw`).

Regenerate the authority corpus, tests, and all viewer outputs:

```bash
# 0) authority self-tests (mass conservation + height=mass/density + repose envelope)
.venv/bin/python -m terrain_authority.tests

# 1) (re)build the sample scenes
.venv/bin/python -m terrain_authority.scenes        # writes samples/*/

# 2) native viewers (D1a / D1b / D3)
.venv/bin/python -m terrain_authority.hexviz samples/crater --field state_label
.venv/bin/python viz/groundtruth_viz.py samples/crater --turntable
.venv/bin/python viz/groundtruth_viz.py samples/boulder_field
.venv/bin/python viz/variety_panel.py

# 3) Godot render sidecar (D2 + D4) — six diagnostic layers + headline boulder render
cd godot_sidecar
for L in heightmap state terrain clasts dust distortion; do
  ./render_layers.sh -- --scene ../samples/crater --layers $L \
      --pose 2.56,3.0,6.4,2.56,-0.1,2.56 --size 1024x768 --out layer_$L.png
done
./render_layers.sh -- --scene ../samples/boulder_field --layers terrain,clasts \
    --size 1024x768 --out boulder_terrain_clasts.png

# real EZ-RASSOR chassis (one-time: convert the vendored MIT mesh DAE -> glb)
../.venv/bin/python ../scripts/convert_rover_mesh.py
./render_layers.sh -- --scene ../samples/crater_boulders --layers terrain,clasts,rover \
    --pose 1.9,2.5,6.7,2.7,-0.1,2.56 --out crater_boulders_rover.png
```

`terrain_authority.tests` is authoritative: 7/7 checks pass (total mass constant across cut→dump→relax,
rel drift 3e-16; `height == datum + mass/density` after every op, max err 0.0; sandpile relaxation conserves
mass and leaves all loose slopes ≤ θ_r within 1°; non-square save/load round-trip preserves
dims/dtype/row-major).

---

## 4. What's papered over (and the citations)

Honest accounting. Every shortcut is deliberate, scoped to Tier 2, and cited to a spec section and the
paper that anchors the eventual fix. Cite by filename in `papers/` (do not bulk-open the PDFs).

| # | Shortcut (what's simplified) | Why it's fine for Tier 2 / this slice | Spec § | Citation |
|---|---|---|---|---|
| 1 | **Earth/Apollo-era Bekker moduli, no 1g→⅙g correction.** `k_φ` and sinkage exponent `n` use classic Mitchell/Costes fits; the low-g drop is *not* applied, so the surrogate under-predicts lunar sinkage. | Geometry/state-accurate is the bar (forces are engineered small, §9); the correction is a calibration step, not a structural change. Flagged `[CALIB]` in `constants.py`. | §5.2, §10 | `lyasko2010.pdf` |
| 2 | **No PyChrono / SCM deformable terrain.** Authority is a pure-NumPy analytical surrogate. | The frozen `INTERFACE.md` contract makes Chrono a drop-in producer swap with zero consumer changes — the whole point of §2's single-authority + decoupled-render design. | §2, §4 | `ascend24-ipex-trl-5-design-overview.pdf` |
| 3 | **Single-pass rover, geometry-only.** Compaction + rut sink + TREAD relabel; multi-pass "paving" emerges by re-applying. **No slip-sinkage / runaway entrapment.** | Static bearing sinkage self-limits in ⅙ g; slip-sinkage runaway (the Spirit failure) is exactly what a real Chrono::Vehicle slip solver would surface — named, not faked. | §6, §9 | `asce-es-2024-isru-pilot-excavator-wheel-testing.pdf`, `lyasko2010.pdf` |
| 4 | **Quadtree is visualized, not memory-managing.** Nodes drive D1b wireframes and the far-field-vs-active LOD split, but there's no live hot-region promotion/eviction. | The tree's job in this slice is to *show* that space-management and LOD are keyed to interaction (§4); the active grid is the substrate, and it's uniform-fine as required. | §4 | — |
| 5 | **Ballistic dust is render-only.** GPUParticles3D lofted from disturbed cells, lunar g, no drag — never enters the mass balance. | This is the spec's explicit instruction: no atmosphere → ballistic, not suspended; the lens/coating budget is µg–g against kg, negligible for conservation. Dust lives entirely in the render/sensor layer. | §8 | `2021-ASCEND-Mass-Inference-RASSOR.pdf` (gentle counter-rotating-drum excavation) |
| 6 | **Camera distortion is a stub.** A Brown-Conrady radial barrel-warp post-process exists, but there's no calibrated custom projection matrix or radial-tangential intrinsic fit. | Demonstrates the post-chain attaches to the real 3D frame; the calibrated intrinsics are a known few-hundred-line follow-on (CARLA gives it natively; here you own it). | §8 | — |
| 7 | **No SLAM / no two-channel evaluation.** No observed-map-vs-true-terrain or SLAM-pose-vs-true-pose scoring yet. | The ground-truth *producer* side is what this slice proves; D1b renders the exact "true terrain at time t" that evaluation will score against. | §2, §10 | — |
| 8 | **No ROS2 bridge; Y-up/Z-up TF trap named, not solved.** `INTERFACE.md` §3 documents the Godot Y-up ↔ ROS Z-up (REP-103) mapping but defers it. | Both bridge options (compiled module vs. rosbridge) are third-party/low-bus-factor; out of weekend scope by charter. The trap is named so it isn't a silent bug later. | §11 | — |
| 9 | **Clasts are metadata refs, not carved into mass.** Golombek-SFD boulder field lives in `metadata.clasts`; uncovered clasts become Chrono rigid bodies, not regolith. | Spec §6 explicitly: "rocks are not a soil problem" — rigid-body contact is Chrono-native; don't let rocks drag the design toward DEM. | §6 | `rock-size-freq_abstract.txt` (Golombek 2003) |
| 10 | **Ice/volatile field optional and inert.** Schema slot exists; no sublimation/frost optics or regime-flag switching modeled. | PSR effects are throttled hard (insulating regolith, sub-mm desiccated lag crust → no dramatic venting) and are an *optics* effect, gated on a charter PSR flag. | §5.2, §8 | `geosciences-15-00207-v3.pdf`, `FULLTEXT01.pdf` |
| 11 | **Rover mesh is the EZ-RASSOR `base_unit` chassis only** — static, no wheels/drums/arms assembled, no URDF kinematic tree, no articulation. | A real RASSOR silhouette on the terrain proves the asset/import path; full kinematic assembly from the EZ-RASSOR xacro (4 wheels + 2 counter-rotating bucket drums) and articulation is a known follow-on. | §2, §11 | `docs/ezrassor_assets.md` |

---

## 5. Where this goes — publishable directions

Each builds directly on a seam this slice already exposes:

- **Reduced-g Bekker recalibration against a Chrono::GPU DEM oracle.** Run DEM offline on a few
  representative cuts/wheel passes and fit the Tier-2 `k_φ`/`n`/`c` against it, applying the 1g→⅙g
  correction now flagged `[CALIB]`. DEM never enters the live loop (spec §10). → `lyasko2010.pdf`.
- **Path-dependent perception-failure benchmark.** Drive the closed loop to produce the failure class an
  open-loop generator structurally cannot: self-generated dust cloud, uncovered-rock deceptive shadow,
  berm-that-slumps. Score SLAM/map degradation against the D1b ground truth (spec §2.1).
- **Slip-sinkage runaway early-warning.** Add the slip solver (θ_m = (c₁+c₂·s)·θ_f), then train/operate a
  HITL early-warning on the rim-below-grade + bow-wave + sharp-rut signature — the Spirit failure mode
  operators most need to catch (spec §6).
- **Sandpile-CA-vs-DEM repose calibration.** The reduced-gravity effect on granular flow is genuinely
  unsettled; calibrate the CA's θ_r and cohesion/metastability knobs against DEM avalanche tests across the
  30–47° envelope (spec §7). The `crater_caveins` time series is the ready-made test bed.
- **PSR frost / desiccated-lag optics.** Activate the ice field: bright-frost → dark desiccated-lag albedo
  transient and frost re-condensation on cold shadowed cut walls — both path-dependent scene changes that
  break SLAM, gated on the PSR charter flag (spec §8). → `geosciences-15-00207-v3.pdf`, `FULLTEXT01.pdf`.

---

## 6. Provenance & license

**Independent personal project — not government work.** foss_ipex is the independent open-source
work of **John McCardle**, a private citizen, on his own time. **17 USC §105 does not apply.** The
IP note in [`ipex-terrain-sim-spec.md`](ipex-terrain-sim-spec.md) — *"intended as U.S. Government
work — public domain by default"* — describes the **aspirational deployment context** (what this
would be if adopted by NASA KSC's GMRO lab), **not this repository's legal status.**

**Dedicated to the public domain — [CC0 1.0 Universal](LICENSE).** In the spirit of that §105
framing, an independent build is released the way the government work *would* be: a public-domain
dedication. Use it for anything, no attribution required.

**Dependencies & vendored assets** are all permissive and kept license-clean for public release
(per the spec IP note):
- **Engines** (not redistributed here): Project Chrono (BSD-3), Godot 4.6 (MIT).
- **EZ-RASSOR rover mesh** (`godot_sidecar/assets/rover_base.glb`) — MIT, © UCF / Florida Space
  Institute / NASA; converted from the vendored `.vendor/` DAE, MIT notice carried in
  [`THIRD_PARTY.md`](THIRD_PARTY.md). CC0 covers only original foss_ipex code; the mesh keeps its
  own license.
- **Excluded on license grounds:** EZ-RASSOR `extra_models/` props (rocks, lander, ISRU plant) are
  third-party re-hosted art (clara.io / SketchUp Warehouse) with **no stated license** — they would
  compromise public release. Clasts are generated procedurally (Golombek SFD) instead.

See [`docs/chrono_integration.md`](docs/chrono_integration.md) and
[`docs/ezrassor_assets.md`](docs/ezrassor_assets.md) for the integration research behind these.

---

*Authority contract frozen 2026-05-30 (`INTERFACE.md` v1.0). Spec: [`ipex-terrain-sim-spec.md`](ipex-terrain-sim-spec.md).*
