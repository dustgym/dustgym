---
title: "Spec coverage"
nav_order: 5
---

# Spec-coverage scorecard — `ipex-terrain-sim-spec.md` §1–§12 vs. what's built

*Status: living audit (snapshot 2026-05-31, repo head `3a3aef3`). This is the honest "what does the
slice actually cover?" pass against the master spec — the §-by-§ companion to [`README.md`](../README.md)
§4 (what's papered over) and [`ARTIFACTS.md`](../ARTIFACTS.md) (what re-runs green). Verdicts are graded
against the SPEC's bar, not the slice's ambition; evidence is by `file:line` / commit / artifact so a
reviewer can check each claim without taking my word for it.*

**Legend.** ✅ Accomplished · 🟡 Partial (real but incomplete) · 🔵 Surrogate-or-stub (stands in for the
eventual thing behind a frozen seam) · ⬜ Left out (named, not built).

---

## Section-by-section

| § | Spec topic | Status | Evidence (file:line / commit / artifact) | Open items |
|---|---|---|---|---|
| **1** | Purpose / framing: sensor-faithful, judged at camera output | ✅ | The whole product is built to this thesis — exposed-sublayer albedo (`terrain.gdshader:67-77,223-230`), grazing-sun shadows, ballistic dust placement, *not* force accuracy. README §1 restates it; the closed-loop path-dependence payoff is realized by the driven-rover tread trail (`viz/out/tread_track.gif`) + uncovered-clast shadows (`out/crater_boulders.png`). | None structural. The remaining failure-class demos (self-made dust cloud, slumping berm seen by SLAM) await §10's map channel. |
| **2** | Single-authority dynamics, decoupled render, downstream eval | 🟡 | Architecture is real and the seam is *demonstrated*: NumPy authority → frozen `INTERFACE.md` on-disk contract → Godot consumer (`godot_sidecar/state_fields.gd`), and Chrono drop-in proven end-to-end (`scripts/chrono_scm_export.py`, `docs/chrono_bringup_log.md` §E). The **second** seam (Godot → ROS2) is now also live and frozen (`docs/sensor_bridge_contract.md`). | Live authority is still the surrogate for SOIL, not Chrono (§5 below). **Update (2026-06): PyChrono 10.0.0 installed (conda-forge core+fea+robot) and a LIVE Chrono rigid-body producer runs** — `scripts/chrono_clast_producer.py` settles rigid clasts on the surface under lunar g (free-fall exact, clasts rest at z≈radius), the §4.4 hybrid's rigid-body half. The SCM deformable-terrain soil oracle still needs `pychrono.vehicle` (NOT in the conda build → source build). Eval is one channel of two (§10). |
| **3** | Fidelity tiers — target **Tier 2** | ✅ | Tier-2 by construction: analytical geometry + heightfield carving + rocks-as-rigid-refs + disturbance-driven dust, no live granular solver. README §1, spec §3. Tier-3 DEM correctly scoped *offline-only* and not attempted in the loop. | Tier-3 DEM oracle for calibration (§10) not run. |
| **4** | Spatial rep: stacked heightfield, interaction-keyed quadtree, uniform solve grid | ✅ (LOD) / 🟡 (compute) | Stacked-heightfield column model (`terrain_authority/column_state.py`), interaction-keyed quadtree that promotes under the rover and evicts behind (`quadtree.py`; `viz/out/quadtree_demo.gif`; per-frame `active_leaves` metadata, INTERFACE v1.0.1). Variable-resolution tiles + refine/coarsen operators added (`refinement.py`, INTERFACE v1.0.2 §5.3). Physics↔render texture-field handoff is the `.rf32`/`.r8` contract. The LOD payoff is now **demonstrated at mission scale**: the spiral demo's `instrument_spiral.py` records an **O(corridor) resident ~21 MB** 2 cm record vs **3.99 GB** for a dense 2 cm tiling of the whole 220 m patch (`resource.gif`, commit `c560981`), and the top-down UNLIT run renders the live quadtree-LOD overlay tracking the rover down the corridor (`topdown_spiral.gd` + per-frame `qt_leaves.json`, commit `54e1ef6`). | Quadtree buys **render/LOD legibility, not solve cost** — the 256² grid is solved uniformly every step (README §4 #4; ARTIFACTS caveat #4). The §5.3 fine `tiles[]` carry 1 cm data but the Godot mesh doesn't yet build a finer corridor mesh from them (shader-detail only). Local SDF/voxel escape hatch: not built (correctly, not needed yet). |
| **5.1** | Fixed constants + per-scenario site config | ✅ | All fixed constants present and SI-converted with `[FIXED]` tags: `g=1.62`, `G_s=3.1`, `S=1361`, polar sun band (`constants.py:25-46`). Site config (sun elevation, crater geometry) drives the hillshade/render. | PSR/cold-trap flag exists as a constant (`T_PSR_K`, `W_ICE_MAX`) but gates nothing live (see §5.2/§8). |
| **5.2** | Calibration unknowns: Bekker moduli, slip coeffs, repose, swell, ice | 🔵 **decorative** | The Bekker/Janosi parameters are **defined but not read by the authority** — grep-confirmed: `K_PHI`, `K_C`, `N_SINKAGE`, `COHESION`, `K_SHEAR`, `SLIP_C1/C2` appear only in `constants.py` (and the separate Chrono bootstrap `scripts/chrono_scm_rover.py:113-115`), never in `rover.py`/`column_state.py`. Compaction is a hardcoded `compaction=0.12` default (`rover.py:50,74,149,183`), **not** a pressure-sinkage solve. The 1g→⅙g correction is explicitly **not applied** (`constants.py:84-105`, flagged `[CALIB]`). Repose `THETA_R` and swell `SWELL_FACTOR` **are** live (sandpile + spoil density). | Make the moduli load-bearing (a real Wong-Reece/Bekker sinkage), then apply the low-g correction against a DEM oracle (README §5; `lyasko2010.pdf`). |
| **5.3** | Per-column dynamic state: mass-conserved, derived height, labels | ✅ | Areal mass is the conserved invariant; `z = datum + mass/(area·ρ)` is **never stored independently** (`column_state.py`). State enum VIRGIN/TREAD/EXCAVATED/SPOIL/COMPACTED_BERM present (`constants.py:191-196`). Conservation + `height==mass/ρ` assert green: rel-drift 2.99e-16, max height-err 0.0 (`terrain_authority.tests`, 18/18; ARTIFACTS cmd 1). | Embedded-clast refs are metadata only (correct per §6). Exposure-time/temp + maturity albedo are render-side, partial. |
| **6** | State transitions; two sinkage modes; rocks as rigid bodies | 🟡 | TREAD compaction (multi-pass paving emerges by re-applying), EXCAVATED cut→inventory, SPOIL dump, COMPACTED_BERM, sandpile relaxation each tick — all built (`rover.py`, `column_state.py`, `sandpile.py`; 4-wheel separable ruts §6.4/6.5). Rocks-as-rigid-refs (not soil) honored (`metadata.clasts`). The spiral demo now exercises this end-to-end as a **driven** rover: `drive_spiral.py` carves **four separate mass-conserving ruts** per frame into a `<scene>_driven` heightmap with an **accumulating** compaction trail, and the body **kinematically conforms** to the terrain (4-wheel plane-fit pitch/roll + capped clast ride-over, `rover.conform_pose`, unit-tested `test_conform_pose_flat_ramp_clast` → 19/19; commit `3a3aef3`). | **Slip-sinkage / runaway entrapment is NOT modeled** — only *static* bearing sinkage via geometry. `slip` is carried as an optional render-direction hint in `build_wheel_tracks_meta` (`rover.py:195-223`), not a sinkage solver; the comment is explicit (`rover.py:17-18`). The Spirit-mode failure + recovery maneuvers — the failure HITL operators most need — are named, not built (README §4 #3). |
| **7** | Berm building, bulking, sandpile collapse | ✅ | Bulking closes in **mass** (cut dense → dump loose at `RHO_SPOIL`, taller per kg; `constants.py:129-137`). Sandpile CA produces avalanches/repose with cohesion metastability (`sandpile.py`; `viz/out/caveins.gif`, the +1.90 m→+0.25 m rim slump). Relaxation conserves mass (rel-drift 1.75e-16) and leaves slopes ≤ θ_r within 1° (tests). | Berm-to-spec *scoring* (charter-gated, §12 Q2) not built — would need a deposition-quality metric, not just locomotion. Reduced-g repose-angle calibration vs. DEM is open (README §5). |
| **8** | Dust, volatiles, optics, camera intrinsics, lunar lighting | 🟡 (mixed) | **Lighting + BRDF: ✅ and above spec** — Hapke IMSA / Lommel-Seeliger photometry, sourced (Sato 2014, Hapke 2002), replaces Lambert (`terrain.gdshader:79-102`; `docs/render_fidelity_spec.md` §9). Cut-depth exposed-sublayer albedo, sourced not eyeballed (commit `2a96386`). **Dust: 🔵** render-only ballistic GPUParticles3D, lunar g, no drag, never in mass balance (`out/layer_5_dust.png`; README §4 #5) — exactly the spec instruction. **Camera intrinsics: 🟡** the M1 rig now emits a **real calibrated pinhole** `fx=fy=(w/2)/tan(fov/2), cx, cy` from the Godot FOV (`camera_rig.gd:79-88`, `sensors.json`) — so intrinsics-as-metadata WORKS; only the **distortion** stays a stub (Brown-Conrady radial barrel-warp, k1/k2 render post only, no calibrated `plumb_bob` fit; `distortion.gdshader:2-28`). | **Volatile/PSR optics ⬜:** no sublimation, no frost albedo transient, no frost re-condensation (the path-dependent SLAM-breakers). Calibrated radial-tangential distortion fit deferred. |
| **9** | Regolith domain notes (intuition corrections) | ✅ | Encoded as constants + behavior: fine D₅₀ (`D50=70e-6`), depth-density gradient (`RHO_SURFACE 1300` over `RHO_DEEP 1920`, `Z_T=0.12`), cohesion as interlocking (`COHESION`, crisp-walled ruts in render), regional/polar wide-envelope flagged `[UNKNOWN]`. Boulders render **angular/faceted** (conchoidal-fracture facets, `clast.gdshader:25-42,118`), sourced to Tsuchiyama 2022 triaxial ratios (`papers/CITATIONS.md:67`) — a regolith-accuracy lift over rounded pebbles. | "Forces engineered small → geometry-accuracy suffices" is the design hinge; honored. No gap. |
| **10** | Validation: conservation asserts, DEM calibration oracle, determinism, two-channel eval | 🟡 | Conservation invariants **assert green** (18/18; ARTIFACTS cmd 1) — mass constant, height-derived, save/load round-trip, quadtree tiling. Determinism: scenes re-export with identical bookend md5 (ARTIFACTS cmd 2). **Two-channel eval — first channel now LIVE:** the SLAM/pose channel reads a real number — camera→tag pose-vs-truth on a real Godot render, **12.7 mm / 7.15°** on `flat_compact` (also 13.3 mm/5.10° crater_boulders; 29.9 mm/1.46° on an oblique view), commit `573e126`. The pose channel is now **demonstrated at 100 m mission scale** across an 80-frame Haworth spiral with a real **illumination A/B** and per-frame failure attribution (range / shadow / occlusion): aimed front-stereo localizes **22/80 lit vs 45/80 unlit** (`failure_breakdown.png`, commit `c560981`), and the travel-tangent side-mono variant **21/80 lit, 47/80 unlit** (commit `3a3aef3`) — the localization rate holds when the rover faces travel and only glimpses the lander with the side cam. | **Map channel 🟡 scorer now built:** `scripts/ros2_bridge/score_map.py` computes the §10 perceived-map-vs-truth metrics (`map_rmse_m` + `map_cell_pass_frac` + `rock_f1`), wired into `eval_harness.py` as the 2nd channel beside pose (`run_map` + `--map-truth/--map-observed` CLI; `test_score_map.py` 6 tests; real Haworth identity 0 m/1.0, block-4 reconstruction 1.70 m/0.427). The **observed-map PRODUCER** (stereo-depth/SLAM → heightfield) is still ⬜ — it needs the Godot/sensor render track, so the scorer fabricates no observed map (you supply one). **DEM calibration oracle ⬜:** never run; the §5.2 moduli have nothing to fit against yet. |
| **11** | Candidate tooling + integration frictions | 🟡 | Godot 4.6 Forward+ (MIT) render+sensor: ✅. Project Chrono (BSD-3) **bootstrapped** — PyChrono 10.0.0 runs `SCMTerrain` rover at lunar g, partial exporter round-trips the frozen contract (`docs/chrono_bringup_log.md` §C/D/E). **ROS2 bridge now WORKS** — containerized `scripts/ros2_bridge/`: `bag_writer.py`→rosbag2 MCAP, `apriltag_ros` detects id 0, `compare_pose.py` prints pose error; REP-103 Godot Y-up↔ROS Z-up seam **solved in one place** with 3+ guard unit tests (`frames.py`, `test_frames.py`; `sensor_bridge_contract.md` §3). The Y-up/Z-up TF trap moved named→solved. | No URDF/SDF import (rover assembled from hand-transcribed xacro kinematics, README §4 #11). No `ros2_control`/Nav2 hooks. Chrono::Vehicle (vs. the bare SCM test cylinder) not modeled. |
| **12** | Open questions / charter dependencies | ⬜ (by design) | These are *charter* decisions, not build items — surfaced honestly, not resolved: excavation-forces-in-scope (Q1, → Tier 2 default), berm-to-spec scoring (Q2), PSR site (Q3, gates the whole ice regime), ROS2 bridge choice (Q4 — M1 picked the looser file/bag seam over a compiled module), calibration simulant/dataset (Q5). | All five are correctly left to the charter; the slice picks defensible defaults and names the dependency. |

---

## Above spec (build is *ahead* of the master spec here)

- **Hapke / Lommel-Seeliger BRDF.** Spec §8 only *gestured* at "lunar lighting — Godot's strong suit"; the
  build ships a sourced **Hapke IMSA** photometric model (2-term Henyey-Greenstein phase, shadow-hiding
  opposition surge, Chandrasekhar multiple-scattering H-function), tied to Sato et al. 2014 LROC-derived
  parameters and Hapke 2002 (`terrain.gdshader:79-102`; `docs/render_fidelity_spec.md` §9). This is real
  airless-regolith photometry, not a Lambert+ramp stand-in.
- **Angular/faceted clasts.** Boulders render with conchoidal-fracture faceting and Tsuchiyama-2022 triaxial
  axial ratios (`clast.gdshader`, `papers/CITATIONS.md:67`) — beyond anything the spec required.
- **The whole M1 camera→ROS2→SLAM bridge.** The spec lists ROS2 as *candidate tooling* (§11) with the
  bridge choice an open charter question (§12 Q4); a working containerized bridge that closes the §10 pose
  channel on a real render is **beyond the weekend slice's stated scope** — it is the next milestone, landed
  early.
- **A mission-scale failure-attribution demo.** The spiral battery (`c560981`→`54e1ef6`→`3a3aef3`) runs the
  full pipeline on a real 220 m Haworth window — an 80-frame spiral egress from a fixed-center LM-class
  lander, a kinematically-conforming driven rover leaving accumulating mass-conserving ruts, container-side
  AprilTag PnP localization, a top-down LIT/UNLIT render pass, an O(corridor) ~21 MB-vs-3.99 GB resource
  record, and synced composite GIFs. The spec asks for conservation asserts + a pose reading; this packages
  them into a legible *why-the-loop-matters* artifact (range/shadow/occlusion failure attribution) the spec
  never required.

## Doc-lag worth flagging (code state ahead of the prose docs)

- The **12.7 mm / 7.15°** real-render pose reading lives only in **commit `573e126`'s body** and the live
  bag (`scripts/ros2_bridge/bags/m1_final/`). The committed `scripts/ros2_bridge/README.md` still narrates
  only the *fixture* placeholder (2286 mm / 120°, "expected to drop to cm/deg"); `ARTIFACTS.md` now carries
  both the bridge reading and the spiral-battery mission-scale demo, but top-level `README.md` still predates
  the bridge. The §10 pose channel is genuinely better than the headline prose currently claims — surprising
  on the *upside*, the opposite of the usual drift.

---

## Top open items, ranked by GMRO-polar portfolio value

1. **PSR / volatile optics (§5.2/§8).** Highest polar relevance and currently the emptiest slot: the ice
   field is a wired schema slot (it already gates the CEMENTED regime in `sandpile`, `column_state.py:205-207`)
   but **no scene populates it** (`scenes.py:881` passes `ice=None`) and there is no sublimation/frost-albedo
   /re-condensation optics. Bright-frost→desiccated-lag transient + frost on cold shadowed cut walls are the
   path-dependent SLAM-breakers the IPEx ISRU regime is *about*. → `geosciences-15-00207-v3.pdf`, `FULLTEXT01.pdf`.
2. **Close §10's second (map) channel + a scoring harness.** The pose channel is live (now demonstrated at
   100 m mission scale with a lit/unlit localization-failure A/B, commits `c560981`/`3a3aef3`); the observed-map-vs-
   true-terrain-at-time-t channel — the actual LAC-style mapping objective — does not exist. This converts the
   slice from "produces ground truth" to "scores perception against it," and reuses the live `/lander/apriltag_truth`
   seam plus the D1b "true terrain at time t" renders.
3. **Slip-sinkage runaway + recovery maneuvers (§6).** The Spirit-mode entrapment is the failure HITL
   operators most need to recognize early, is purely path-dependent, and is the clearest demo of *why* a
   closed loop beats a procedural generator. Needs θ_m=(c₁+c₂s)·θ_f wired into a real sinkage solve.
   → `lyasko2010.pdf`, `asce-es-2024-isru-pilot-excavator-wheel-testing.pdf`.
4. **Live Chrono authority + DEM recalibration (§2/§5.2/§10).** Promote the bootstrapped PyChrono SCM run to
   the live producer (Chrono::Vehicle, mass-hybrid exporter), and run the offline Chrono::GPU DEM oracle to
   make the currently-**decorative** Bekker moduli load-bearing with the 1g→⅙g correction applied.
   → `lyasko2010.pdf`, `ascend24-ipex-trl-5-design-overview.pdf`, `docs/chrono_bringup_log.md`.
