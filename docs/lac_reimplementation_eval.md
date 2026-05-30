# Reimplementing the Lunar Autonomy Challenge on the foss_ipex stack — evaluation

*Evaluation written 2026-05-30. Grounded against the live repo (INTERFACE.md v1.0.1, README, `docs/chrono_bringup_log.md`) and a 13-agent web research sweep of the LAC documentation, the kit/Leaderboard source mirrored in team forks, the NASA/PGDA datasets, and three arXiv solution papers. Every external claim is cited inline; numbers sourced from team forks rather than JHU-APL directly are flagged.*

> **Scope note.** This is an architecture/feasibility evaluation, not a build. It exists to answer: *should* foss_ipex adopt the Lunar Autonomy Challenge (LAC) as its external benchmark, what transfers for free, what has to be built, and what we may legally reuse under the repo's CC0-1.0 release.

---

## 1. Verdict

**Worth doing — it is the single best-fit external benchmark foss_ipex could adopt — but as a multi-phase build, not a weekend slice.** LAC is a NASA / JHU-APL / Caterpillar / Embodied-AI competition whose simulated rover is a **digital twin of NASA's ISRU Pilot Excavator (IPEx)** ([NASA STMD](https://www.nasa.gov/directorates/stmd/top-prize-awarded-in-lunar-autonomy-challenge-to-virtually-map-moons-surface/)), and foss_ipex is a portfolio piece aimed squarely at the **IPEx team at NASA KSC's GMRO lab**. Adopting LAC's objectives, parameters, and agent API reframes foss_ipex from "a lunar terrain demo" into **"an open, CC0 reimplementation of the exact mapping mission the IPEx-twin competition runs, on a FOSS Chrono+Godot stack instead of the closed CARLA+Unreal one"** — a framing directly legible to the hiring lab.

The terrain authority and the rover transfer cleanly. The **mission, sensor streams, agent runtime, and scoring do not exist yet** and are the real work.

---

## 2. Correcting the working assumption (this matters, and it helps us)

The premise going in was *"LAC has zero terramechanics or terrain interaction besides collision."* The research refines that — and the refinement is in our favor:

- **LAC terrain *is* deformable.** Wheels sink **2–3 cm** and leave visible tracks; the regolith is a deformable surface ([Embodied-AI overview](https://lunar-autonomy-challenge.jhuapl.edu/Challenge-Documentation/Embodied-AI/)). So it is **not** collision-only.
- **But the deformation is cosmetic, not conservation-grade.** Ground-truth elevation is computed *after* deformation, yet "deformation has minimal impact averaged over a cell"; **rocks are immovable**; there is **no mass conservation, no stratigraphy, no excavation**. The IPEx bucket-drums exist on the twin but the **excavation task is disabled** in the 2024–25 mapping-only edition. The regolith model itself is undocumented (it is CARLA/Unreal terrain, not a published physics model).
- **LAC's *vehicle dynamics* are Project Chrono.** The organizers chose foss_ipex's physics engine ([2024 Guidebook](https://lac.jhuapl.edu/Challenge-Information/2024_Lunar-Autonomy-Challenge-Guidebook.pdf)). Godot simply replaces Unreal's render layer. **Do not frame Chrono as a substitution that needs defending — it is the same engine the competition uses.**

**Net:** LAC is fundamentally an *autonomy + mapping* challenge (estimate per-cell elevation + a boolean rock flag) over a surface with *token* deformation. foss_ipex's distinctive contribution is precisely the layer LAC omits: **mass-conserving, excavation-grade terramechanics with stratigraphy and berm-building.** The original instinct was right in spirit; the honest phrasing is *"cosmetic deformation"* → *"conservation-grade terramechanics,"* not *"no terramechanics"* → *"terramechanics."*

---

## 3. What LAC actually is (fact-dense)

**World.** Two dev environments, `Moon_Map_01` / `Moon_Map_02`, each **40 m × 40 m** with a **27 m × 27 m mapping area** centered on a static lander, plus a 6.5 m buffer ([getting_started](https://lunar-autonomy-challenge.jhuapl.edu/Challenge-Documentation/Embodied-AI/getting_started/)). Terrain derives from **LRO/LOLA South-Pole DEMs** (≈5 m/px local) with procedurally added rocks and deformable regolith ([Stanford SLAM paper](https://arxiv.org/html/2603.17229v1)). Lighting is the headline perception hazard: **very low sun elevation**, airless hard-edged shadows, overexposed highlights ([2603.17232](https://arxiv.org/html/2603.17232v1)). Sun and Earth are **dynamic** over the mission (a `--static-sky` replay flag freezes them). Lunar gravity **1.6220 m/s²** ([Stanford `params.py`](https://raw.githubusercontent.com/Stanford-NavLab/lunar_autonomy_challenge/HEAD/lac/params.py)).

**Rover (IPEx twin).** 4-wheel **differential / skid-steer**, no suspension, ~30 cm-diameter wheels, front & rear articulating arms with counter-rotating bucket drums (excavation disabled this edition). Sim caps: **max linear 0.48 m/s, max angular 4.13 rad/s** ([ipex_technical_details](https://lunar-autonomy-challenge.jhuapl.edu/Challenge-Documentation/Embodied-AI/ipex_technical_details/)); real-IPEx nominal 30 cm/s, 15° slopes, 7.5 cm obstacles ([NTRS 20240008162](https://ntrs.nasa.gov/citations/20240008162)).

**Sensors.** **8 monochrome cameras** (front+back stereo pairs, two side, one per arm), each with a co-located LED; **IMU at 20 Hz**, images at **10 Hz**, sim tick **20 Hz**. Max resolution **2448×2048**, ideal **pinhole / square pixels / zero distortion**, **70° (1.22173 rad) horizontal FOV**, **stereo baseline 0.162 m** (team-derived — see open questions). Hardware allows only 4 cameras live at once (advisory in sim). No GNSS, no LiDAR; wheel state via `get_linear_speed` / `get_angular_speed`.

**Mission & constraints.** Build two maps over the 27×27 m area — **elevation** (avg cell height) and **boolean rock-present** — on a **180×180 grid of 15 cm cells = 32,400 cells** ([metrics](https://lunar-autonomy-challenge.jhuapl.edu/Challenge-Documentation/Embodied-AI/metrics/)). Manage a **283 Wh battery** (5 Wh floor → termination) per a published Watt table (camera 3 W, light 9.8 W, wheels 40–60 W, compute 8 W…); recharge at the lander (full-from-empty = 2 h mission time). Mission clock **24 h finals** (1 h qualifier; 9×9 m qualifier area). Off-nominal termination on out-of-power, out-of-time, **out-of-bounds (±19.5 m)**, or **blocked (<0.1 m/s for >5 min)** ([constants.py](https://raw.githubusercontent.com/alex-tanton/LunarAutonomyChallenge/main/Leaderboard/leaderboard/utils/constants.py)).

**Scoring (1000 pts).** Geometric map **300** (1 pt per cell with |Δh| ≤ **50 mm**, ÷ N_truth × 300 — a hard threshold, *not* RMSE); Rock map **300** (**F1** = 2TP/(2TP+FP+FN)); Mapping productivity **250** (target **1350 cells/hr** = 32,400/24 h); Localization **150** (bonus for mapping *without* the lander's AprilTag 36h11 fiducials) ([metrics](https://lunar-autonomy-challenge.jhuapl.edu/Challenge-Documentation/Embodied-AI/metrics/)). Winners: Stanford NAV Lab 1st (~3.8–6.1 cm localization RMSE, total ~827.5), MIT MAPLE 2nd, CMU 3rd ([2603.17232](https://arxiv.org/abs/2603.17232)).

---

## 4. The three reuse targets — and a redistribution-safe path for each

These are the artifacts worth extracting. The constraint that shapes all three: **the LAC simulator package, the `Moon_Map_01/02` environments, and the IPEx digital-twin assets are gated and carry no stated license → treat as NON-redistributable.** We reuse *facts and numbers* (freely reimplementable) and *public-domain source data*, never APL's bundled assets, under our CC0 release.

### 4.1 The IPEx rover twin → rebuild geometry, don't vendor assets
The kit ships `docs/geometry.json` with **exact body-frame extrinsics** for all 8 cameras, the LEDs, the IMU, the wheels (0.32 m dia), the arms, and the lander fiducials (mirrored in MIT team forks: [Stanford `geometry.json`](https://raw.githubusercontent.com/Stanford-NavLab/lunar_autonomy_challenge/main/docs/geometry.json)). Real-IPEx specs are public via [NTRS 20240008162](https://ntrs.nasa.gov/citations/20240008162). **Path:** drop the *numbers* onto the existing articulated **EZ-RASSOR** (already vendored MIT, RASSOR-class IPEx cousin); do not bundle the JHU-APL twin meshes or the `geometry.json` file itself until its license is verified (the numbers are facts we may use freely regardless).

### 4.2 The control interface → mirror `AutonomousAgent`, run winning agents unmodified
This is the strongest finding. `AutonomousAgent` is **only loosely coupled to CARLA** — its sole CARLA dependencies are value types (`carla.VehicleVelocityControl`, the `carla.SensorPosition` enum, `carla.Transform`/`carla.Location`, `carla.RadiatorCoverState`) ([autonomous_agent.py](https://raw.githubusercontent.com/alex-tanton/LunarAutonomyChallenge/main/Leaderboard/leaderboard/autoagents/autonomous_agent.py)). A **thin `carla` shim** (a few dozen lines) plus a run-loop lets the **MIT-licensed Stanford (1st) and MIT MAPLE (2nd) agents run *unmodified* against the foss_ipex backend.** See §7. *That is the headline portfolio demo: real competition-winning autonomy code driving an independent open Chrono+Godot sim.*

### 4.3 The example map → can't take theirs, rebuild a comparable one from the same public DEM
LAC's `Moon_Map_01/02` are non-redistributable. But LAC's terrain *derives from* public-domain **LOLA 5 m/px South-Pole DEMs** ([PGDA Product 78](https://pgda.gsfc.nasa.gov/products/78) — U.S. Government public domain, free GeoTIFF, no registration). **Path:** import a PGDA south-pole DEM as the heightmap basis, add procgen rocks at the Golombek SFD we already use, and document it as *our* choice of a comparable region — never a claim of bit-fidelity to APL's map. This gives "a comparable simulation" legally and honestly.

> The kit zip is itself a **public ~4 GB S3 download** (`lac-content.s3.us-west-2.amazonaws.com/LunarAutonomyChallenge.zip`, HTTP 200, no auth — per the code sweep). Useful to fetch **locally** to read the authoritative `mission_weather.py` sun model, exact `geometry.json`, battery start-charge, and map semantics — but its license is **unstated (EULA inside the bundle)**, so keep it local/gitignored like `papers/` and extract only numbers; do not re-host any of it.

---

## 5. Component mapping — HAVE / PARTIAL / BUILD in foss_ipex

| LAC component | Status | Gap note |
|---|---|---|
| **Sim world (deformable terrain + procgen rocks)** | **HAVE** | foss_ipex has procgen craters + Golombek-SFD boulders + sandpile relaxation + a mass-conserving deformable column model; LAC's deformable regolith + immovable rocks map directly. Scale differs (§8). |
| **South-pole DEM terrain basis** | **PARTIAL** | Procgen today; LAC uses LOLA 5 m/px. PGDA GeoTIFFs are CC0-safe but no DEM-import path exists yet. |
| **Rover model (IPEx twin)** | **PARTIAL** | Articulated EZ-RASSOR assembled from MIT meshes, but **static pose** (joints not physics-driven); no differential-steer dynamics. |
| **Deformable-terrain physics authority** | **PARTIAL** | PyChrono 10.0.0 + `SCMTerrain` runs at lunar g and cut a 13 mm rut; LAC's reference dynamics are *also Chrono*. No Chrono::Vehicle 4-wheel model yet (bare cylinder). |
| **8 mono cameras (intrinsics/extrinsics)** | **BUILD** | Godot renders lit/false-color/dust views, but no calibrated camera — distortion is a render-only stub; no pinhole-K projection, no 8-camera rig. |
| **IMU (20 Hz, lunar g, RHCS)** | **BUILD** | None today; LAC's is documented finite-difference accel + gyro at rover origin. |
| **Agent API (`AutonomousAgent` shim)** | **BUILD** | Nothing today; well-isolated API → small shim is the core new runtime (§7). |
| **Map output contract (elevation + rock, 180×180)** | **PARTIAL** | foss_ipex *produces* heightmap rasters via INTERFACE.md; LAC needs a `(180,180,4)` estimated-map export. Close; different scale/units (§6). |
| **Scoring / eval harness** | **BUILD** | None (README marks two-channel evaluation TODO). LAC formulas are public, pure-NumPy. |
| **Localization landmarks (lander + AprilTags + boulders)** | **BUILD** | No lander, no fiducials. Boulder fields exist (usable as LunarLoc-style landmarks). |
| **Energy / power model (283 Wh + Watt table)** | **BUILD** | Zero battery/power constants in repo (grep-confirmed). Fully specified by LAC docs. |
| **Lighting / sun-angle** | **PARTIAL** | Godot already does a single ~5° hard sun, near-black ambient — *exactly* LAC's grazing regime. Missing az/el parameterization + dynamic sun. |

---

## 6. What transfers directly

- **Objectives & parameters, verbatim.** 27×27 m area, 180×180 grid, 15 cm cell, 50 mm tolerance, F1 rock scoring, 1350 cells/hr target, 1.6220 m/s² gravity, 0.48 m/s & 4.13 rad/s caps, 283 Wh battery, ±19.5 m bounds, 0.1 m/s blocked floor — all published; copy as constants.
- **Datasets.** LOLA 5 m/px South-Pole DEMs ([PGDA 78](https://pgda.gsfc.nasa.gov/products/78)) feed the heightmap; the MIT-licensed **LunarLoc** traverses ([lunarloc-data](https://github.com/mit-acl/lunarloc-data)) and **lac-data** `.lac` utilities ([lac-data](https://github.com/Robaire/lac-data)) are ready-made perception/SLAM validation fixtures (note imagery-rights caveat in §9).
- **The rover.** EZ-RASSOR is already vendored — same RASSOR lineage as IPEx. Geometry/DOF + the body-frame extrinsics from `geometry.json` drop into the existing kinematic assembly.
- **The map-grid contract — the cleanest transfer.** LAC's `GeometricMap` is a pure-NumPy `(180,180,4)` array `[x, y, height, rock_flag]` with `-inf` for uncompleted cells ([map_utils.py](https://raw.githubusercontent.com/Stanford-NavLab/lunar_autonomy_challenge/HEAD/lac/mapping/map_utils.py)). foss_ipex's INTERFACE.md already speaks raw row-major float32 rasters + `metadata.json` + `io_fields.save_scene/load_scene`. The LAC *ground-truth* map ≈ our `heightmap.rf32` + a rock raster derived from `metadata.clasts[]`; the LAC *estimated* map is a new consumer-side raster of the same shape. The decoupling seam was built for exactly this producer/consumer split — adding `estimated_height.rf32` / `rock.r8` (or a `(180,180,4)` export view) is **additive, not a contract break**. Two real differences to reconcile: **grid scale** (256×256 @ 2 cm = 5.12 m vs 180×180 @ 15 cm = 27 m) and **cell semantics** (LAC stores per-cell *average* height including rock tops, no terrain-beneath-rock).

---

## 7. The agent-API shim — the killer demo

**Mirror it; do not redesign it.** The harness calls the agent as `__call__(mission_time, vehicle_status, input_data) → (velocity_control, components_control)`, clamping velocity to the documented caps. The whole reuse surface:

- A `carla` Python module providing the ~5 value types + `SensorPosition` enum (a few dozen lines).
- The `AutonomousAgent` base class with `get_entry_point()` and the five overridable methods (`setup` / `sensors` / `use_fiducials` / `run_step` / `finalize`).
- An `input_data` builder: `{'Grayscale': {SensorPosition.X: np.uint8 HxW}}` from the Godot camera outputs at 10 Hz (+ a training-only `'Semantic'` channel).
- Getters/setters wired to Chrono/sim state: `get_imu_data`, `get_transform` (training only / `None` in eval), `get_linear_speed`/`get_angular_speed`, `get_mission_time`, `get_current_power`, `get_initial_position`, `get_initial_lander_position`, `set_front_arm_angle`/`set_back_arm_angle`, `set_light_state`, `set_camera_state`, `set_radiator_cover_state`, `mission_complete`, and `get_geometric_map` returning the `(180,180,4)` object.

**Payoff:** the **Stanford NAV Lab (1st)** and **MIT MAPLE (2nd)** agents — both MIT-licensed `AutonomousAgent` subclasses — run unmodified. MAPLE even ships `test/mocks/mock_agent.py`, `mock_carla_transform.py`, `mock_geometric_map.py` ([MAPLE](https://github.com/Robaire/MAPLE)) — a ready-made template for the shim. One nuance: LAC's `run_step(input_data)` drops upstream-CARLA's `timestamp` arg, and images arrive every other tick (10 Hz on a 20 Hz sim).

---

## 8. Scaling beyond the fixed 27×27 m map (where foss_ipex pulls ahead)

LAC's map is **fixed at 27×27 m**. foss_ipex's differentiator — and the reason it carries an interaction-keyed quadtree — is a **mission-representative drive**: hundreds of meters, multiple passes, deforming terrain, moving sun. The memory math (measured against the live `ColumnState`, not estimated):

- Current solve grid is **uniform fine**: 256×256 @ 2 cm = 5.12 m square, **33 B/cell in-RAM** (mass_areal/density/disturbance/datum f64 + state_label u1), **17 B/cell on disk** (the 5 rasters) → ~0.079 MiB/m² RAM, ~0.040 MiB/m² disk.
- At that fixed resolution, cost grows with **area²**: 27×27 m = 57 MiB RAM / 30 MiB disk per frame; **300×300 m = 6.9 GiB RAM / 3.6 GiB disk per frame** (and the current quadtree needs a power-of-two square, rounding 300 m up to 8.25 GiB/frame) — times a multi-hundred-frame mission, infeasible.
- **The quadtree is render/space-LOD only today; it does not shrink storage** (ARTIFACTS caveat #3). To realize the win it must become the *storage authority*: a coarse base over the whole world + fine tiles only on the **touched** corridor. Memory then scales **O(path length), not O(area²)**: a 300 m multi-trip mission ≈ ~80 MiB (≈71 MiB fine corridor + ~7 MiB coarse base) vs 8.25 GiB uniform — **~100× smaller**.
- Two realism axes are nearly free: **multiple trips** revisit the same fine cells (touched set saturates → memory flat across trips; passes just deepen deformation in-place), and **sun angle** is a pure render/lighting parameter that never touches column storage.

The required new work is **mass-conserving LOD operators** — *refine* (coarse→fine: split `mass_areal`, density preserved, height re-derives) and *coarsen* (fine→coarse: mass = Σ, density = mass-weighted mean) — holding the §10 invariant across LOD boundaries. The quadtree currently moves *zero* mass; making it conserve mass across LOD transitions is itself a strong GMRO-facing demonstration (and removes the power-of-two-square constraint, since only touched tiles are allocated). Solve-cost bounding (running Chrono/SCM only on active tiles) is the follow-on.

> **Design spec:** the variable-resolution corridor refinement (1 cm tiles, toggleable for speed), the additive INTERFACE.md v1.0.2 metadata for per-wheel tracks + drum teeth marks, and the Godot detail-shading pipeline are specified in [`render_fidelity_spec.md`](render_fidelity_spec.md).

---

## 9. Licensing & redistribution

Keep John's code **CC0-1.0** (matches existing `LICENSE`). Per asset:

| Asset / source | Vendor? | Note |
|---|---|---|
| **LAC sim binary + `Moon_Map_01/02` + IPEx twin assets** | **NO — reference only** | Public ~4 GB S3 download but **license UNSTATED** (EULA inside bundle). Keep local/gitignored; extract numbers only. Reimplement terrain from public LOLA. |
| **LOLA 5 m/px South-Pole DEMs (PGDA 78)** | **YES** | U.S. Gov public domain; CC0-safe. The redistributable terrain basis. Cite Barker/Mazarico et al. |
| **Stanford-NavLab agent** | **YES (MIT)** | Reference architecture + extrinsics + intrinsics model. |
| **MIT MAPLE agent + mocks** | **YES (MIT)** | Agent-mock template; runs against our shim. |
| **lac-data / lunarloc-data utilities + CSVs** | **YES (MIT wrapper)** | But rendered `.lac` imagery derives from APL's gated sim — upstream rights unstated; **do not re-host frames under CC0**, vendor only format utilities / our own captures. |
| **`docs/geometry.json` (extrinsics)** | **CAUTION** | Mirrored in MIT forks but inherited from the kit; verify before bundling the *file*. The *numbers* are reimplementable freely. |
| **EZ-RASSOR rover mesh** | **already vendored (MIT)** | `THIRD_PARTY.md`; `extra_models/` props correctly excluded (no stated license). Build the lander procedurally. |
| **AprilTag 36h11 textures** | **YES** | Tag family is open; generate tags ourselves rather than copy kit PNGs. |
| **arXiv papers (2603.17232 / .17229, 2506.16940, 2509.12367)** | **cite only** | Architecture/number references. |

---

## 10. Phased plan

**Phase 0 — Portfolio slice (smallest end-to-end LAC loop).** EZ-RASSOR on a 27×27 m foss_ipex scene under the existing grazing sun; **1–2 calibrated pinhole cameras** render monochrome frames; a **minimal `AutonomousAgent` shim** runs a trivial agent (fixed arc, stub map); a **NumPy scoring stub** computes the 50 mm geometric score + rock F1 against the D1b ground truth. *Reuses:* the whole terrain authority, `io_fields`, the Godot sidecar + lighting + EZ-RASSOR assembly + `--sequence`. *New:* one `Camera3D` + projection matrix, the `carla` shim + base class, the `(180,180,4)` map view, the three scoring formulas. *Scale:* regenerate a scene at 27 m extent (procgen + quadtree already parameterize on `cell_m`). **Headline: a LAC scoring loop closing on an open stack.**

**Phase 1 — Full sensor + agent fidelity.** All 8 cameras at exact `geometry.json` extrinsics + arm-camera articulation; IMU at 20 Hz; the complete getter/setter API; `input_data` at the 10 Hz/20 Hz cadence. *Goal:* run the **MIT MAPLE / Stanford** agent unmodified.

**Phase 2 — Chrono physics producer + mission economy.** Chrono::Vehicle 4-wheel differential-steer rover on `SCMTerrain` (the bringup-log hybrid), targeting 2–3 cm sinkage; add the **283 Wh energy model**, **lander + AprilTags + recharge**, the **24 h mission clock**, the four termination conditions. *Reuses:* the live PyChrono env, the partial SCM exporter, the frozen contract (zero consumer changes).

**Phase 3 — Localization + dynamic world + DEM + scaling.** Lander/boulder localization reference; dynamic sun; LOLA-DEM-sourced terrain; the storage-backing quadtree from §8 (mass-conserving refine/coarsen) for the hundreds-of-meters multi-pass drive; ROS2 egress (already planned). Validate against LunarLoc traverses + the Stanford RMSE envelope.

---

## 11. Open questions — verify before committing to numbers

- **Sun angle.** Exact `sun_altitude_angle`/`azimuth` and per-mission motion rate are **not public** (docs say only "very low inclination"). Read `mission_weather.py` from the local kit zip to settle it.
- **Camera FOV.** 70° (1.22173 rad) is **team-derived** (Stanford `params.py`), not stated by JHU-APL. Strong-but-unconfirmed; other intrinsics are closed-form once FOV + resolution are fixed.
- **Battery details.** 283 Wh + Watt table came via a text proxy of timing-out pages; **starting charge** and strict-linearity of recharge unconfirmed; per-frame vs continuous Wh accounting unspecified.
- **Steering model.** Docs say *differential/skid-steer*; the full-stack paper describes an *Ackermann-radius* action space ([2509.12367](https://arxiv.org/html/2509.12367v1)) — likely faked Ackermann on a skid chassis. Confirm before fixing the Chrono::Vehicle model.
- **Map / coordinate exactness.** Global-frame origin (lander vs map-center vs robot-start) and the exact sub-cell rock rasterization (5 cm rocks via 16 rays/cell per [geometric_map.py](https://raw.githubusercontent.com/alex-tanton/LunarAutonomyChallenge/main/Leaderboard/leaderboard/agents/geometric_map.py)) must match for scoring parity.
- **LAC terramechanics is not Chrono-bit-matchable.** Its *dynamics* are Chrono but its *regolith deformation* model is undocumented (CARLA/Unreal). We can only **behaviorally** match (target 2–3 cm sinkage), never reproduce LAC's exact rut.
- **Year-2 (LAC_26) drift.** Whether the 2025-26 edition changed the API, scoring, or map size vs the 2024-25 constants here is unconfirmed.

---

## 12. Sources

Documentation: [Embodied-AI overview](https://lunar-autonomy-challenge.jhuapl.edu/Challenge-Documentation/Embodied-AI/) · [getting_started](https://lunar-autonomy-challenge.jhuapl.edu/Challenge-Documentation/Embodied-AI/getting_started/) · [api_reference](https://lunar-autonomy-challenge.jhuapl.edu/Challenge-Documentation/Embodied-AI/api_reference/) · [metrics](https://lunar-autonomy-challenge.jhuapl.edu/Challenge-Documentation/Embodied-AI/metrics/) · [ipex_technical_details](https://lunar-autonomy-challenge.jhuapl.edu/Challenge-Documentation/Embodied-AI/ipex_technical_details/) · [2024 Guidebook (PDF)](https://lac.jhuapl.edu/Challenge-Information/2024_Lunar-Autonomy-Challenge-Guidebook.pdf)
NASA: [STMD top-prize](https://www.nasa.gov/directorates/stmd/top-prize-awarded-in-lunar-autonomy-challenge-to-virtually-map-moons-surface/) · [ISRU Pilot Excavator](https://www.nasa.gov/isru-pilot-excavator/) · [IPEx TRL-5 (NTRS 20240008162)](https://ntrs.nasa.gov/citations/20240008162)
Datasets: [PGDA Product 78 — LOLA 5 m/px South Pole](https://pgda.gsfc.nasa.gov/products/78) · [PDS LOLA](https://pds-geosciences.wustl.edu/missions/lro/lola.htm)
Code: [official kit zip (S3)](https://lac-content.s3.us-west-2.amazonaws.com/LunarAutonomyChallenge.zip) · [alex-tanton/LunarAutonomyChallenge](https://github.com/alex-tanton/LunarAutonomyChallenge) · [Stanford-NavLab](https://github.com/Stanford-NavLab/lunar_autonomy_challenge) · [Robaire/MAPLE](https://github.com/Robaire/MAPLE) · [Robaire/lac-data](https://github.com/Robaire/lac-data) · [mit-acl/lunarloc-data](https://github.com/mit-acl/lunarloc-data)
Papers: [Full-stack nav/mapping/planning (2603.17232)](https://arxiv.org/abs/2603.17232) · [Visual SLAM w/ DEM anchoring (2603.17229)](https://arxiv.org/abs/2603.17229) · [LunarLoc (2506.16940)](https://arxiv.org/abs/2506.16940) · [Ackermann action space (2509.12367)](https://arxiv.org/html/2509.12367v1)

*Caveat: the JHU-APL doc pages time out on direct fetch; their facts are corroborated from WebSearch digests + the team forks/papers. Numbers from team forks (FOV, some sensor specs) are flagged above as team-derived.*
