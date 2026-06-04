# World model for lunar terrain transformation

For a robot that *reshapes* terrain rather than driving through it, the world model is the central
representation tying perception, physics, planning, and control together. The robot is not reaching a
waypoint; it is transforming the surface from a current state into a desired state. This document maps
that five-layer world-model idea onto what foss_ipex / dustgym already implements, and states the one
real architectural decision: which parts to compute exactly and which to learn.

## The shift this project is built around

The state transition is not "robot moves through terrain," it is "robot reshapes terrain into
infrastructure." That is the project's founding design choice: a conserved physics authority *mutates*
the terrain, and the learned or symbolic components only *command* it. Mass conservation is enforced by
construction, so the agent can never move mass that is not there or spend energy it does not have, and
the terrain-matching reward cannot be gamed.

## The five layers, mapped to the repo

| Layer | Specification | Implementation | Status |
|---|---|---|---|
| **Geometry** | height / slope / roughness / traversability; current vs target surface; earthwork volume | `column_state` heightmap, `slope_deg_map`, real LOLA Haworth DEM ingest; `mission_planner` and `structures.py` compute target minus current as cut and fill volumes | real |
| **Material** | per-cell density, cohesion, friction, bearing, compaction | per-cell `density.rf32` and `state_label` are real fields; `material.py` now derives per-cell friction and cohesion from the density field (sourced spec ranges) plus the trafficability maps (cut difficulty, slip susceptibility). The Bekker `k_phi` inside the solver is still global | real (added 2026-06-04) |
| **Physics** | slip, sinkage, traction, excavation force, power; the transition `S(t+1) = f(S, Action)` | the Tier-2 authority: load-bearing Bekker sinkage, the slip ladder, conserved cut / haul / dump, IPEx energy at lunar gravity. The transition is conserved and exact | real, exact |
| **Task** | target height map; cut, fill, transport | `mission_planner`, `structures.py` (8 composite structures, volume-balanced), `terrain_target_env` reward `R = -||H_cur - H_target||` | real |
| **Uncertainty** | terrain / material / localization confidence; per-cell `height_uncertainty[x,y]` | `autonomy.py` Belief and Kalman estimator (pose, energy, drum-fill sigma); per-cell terrain height sigma from the map channel (`obs_map_producer.grid_to_heightfield_uncertainty` + `dig_ready_mask`) | real (added 2026-06-04) |

### The state transition is computed, not predicted

The core function `S(t+1) = f(S, Action)` is the conserved authority itself. When the model is asked
what a 15 cm cut over a 1.5 m pass does, it returns the removed volume (mass-conserved), the energy
(grounded in the IPEx specs), and the slip risk (from the slip ladder) by *computing* them, not by
predicting them with a learned network. The prediction the world model needs is already exact.

### Physics file

The physics file is the simulator configuration, and it lives as Python single-source:
`terrain_authority/constants.py` (SI constants, each tagged `[FIXED]`, `[CALIB]`, or `[UNKNOWN]`),
`ipex_specs.py` (the rover mass, drum, drive, and battery numbers from the IPEx ASCEND 2024 paper), and
`bodies.py` (per-body gravity and regolith, so the same model runs on the Moon, Mars, or an asteroid).

### Skill library

The taxonomy in `building_taxonomy.md` is the reusable skill library, and tasks compose from it:

- `drive_to_pose` -> `step_pose` / `drive.py`; `estimate_regolith` -> `rassor_mass_model` (drum-current
  mass inference, R^2 ~= 0.99); `cut_pass` -> `drum_pass`; `transport_spoil` and `dump_spoil` -> haul plus
  `fill_toward`; `compact_surface` -> `four_wheel_pass`; `inspect_surface` -> the map channel;
  `avoid_hazard` -> `slope_costmap` plus least-cost routing; `recover_slip` -> `slip.py`;
  `verify_grade` -> `score_map` / `terrain_rmse`.
- `Flatten Area = Inspect + Cut + Transport + Compact + Verify`, and `Build Berm = Cut + Transport +
  Dump + Compact + Verify`, composed by `skill_env` and the scheduler. This is the compositional
  generalization the design calls for.

Honest finding from the RL work: at grounded energy ratios a single-task skill is physics-bounded, so a
greedy planner or model-based search already solves it. The headroom for learning is the multi-objective
scheduling layer (build A and B and C under one battery), which is where the scheduler earns its keep.

## The one architectural decision: hybrid, not monolithic learned

The natural temptation is a single learned latent world model that predicts future terrain. For this
robot that is the wrong place to learn, and the project has the evidence:

- **Dynamics: keep it conserved.** We have the exact, sub-millisecond authority. Model-based search in
  the true model beats model-free RL and beats a learned model for planning (beam search reaches the
  optimum at 24 legs versus PPO at 27 and greedy at 28; a search-distilled policy matches the optimum).
  Learning the dynamics would only add error, and a learned dynamics model is hackable by the policy,
  while a conserved one is not.
- **Perception: this is where a learned model earns its keep.** We do not have a cheap exact model of
  what the cameras will see. The Hapke render is hundreds of milliseconds per frame, shadows shift, dust
  and occlusion matter. A JEPA or RSSM that predicts future *observations* (not terrain state) is the
  right learned target, and it serves active perception: drive to the viewpoint that most reduces map
  uncertainty per joule. That ties directly into the Uncertainty layer and the map channel.

So the world model here is **a conserved physics model for dynamics (exact, unhackable) plus a thin
learned model for perception (appearance and observation, for imagination over the expensive render)**.

## Evidence from the map channel (2026-06-04)

The perception layers are now measured, not asserted:

- **Two tiers, both scored against the conserved truth.** Onboard rover stereo is cheap and real-time but
  noisy (RMSE 0.32 m, coverage grows to 16 percent over an 8-station drive). Ground COLMAP is offline and
  accurate (RMSE 0.04 m, 97 percent cell-pass, cameras aligned to truth within 6 mm).
- **Geometry matters for the ground tier.** A camera-height sweep shows COLMAP collapsing toward the
  rover's grazing eye-level: 18 of 18 images register at elevated and mid heights, 12 of 18 at 1.0 m, and
  only 2 of 18 at 0.5 m. Near-horizontal views of a near-flat surface share too few features. Accuracy
  stays near 4 cm where it reconstructs; registration and coverage are what fall off.
- **Photometry matters for SfM.** The physically correct Hapke BRDF gives COLMAP about a third fewer
  points and 30 percent less coverage than an idealized Lambert render, the non-Lambertian regolith
  costing multi-view correspondences exactly as on real lunar imagery.
- **The Uncertainty layer gates action.** `grid_to_heightfield_uncertainty` produces a per-cell height
  sigma (the standard error of the mean, which falls as more views accumulate; single-view cells get a
  prior), and `dig_ready_mask` flags cells confident enough to dig versus cells to observe more first.

Figures: `validation/map_channel/`.

## Status and next builds

Built: all five layers (Geometry, Material, Physics, Task, Uncertainty); the conserved transition; the
physics file; the skill library and its composition; both perception tiers scored against truth.
`material.py` derives per-cell friction and cohesion (plus cut-difficulty and slip-susceptibility maps)
from the conserved density field across the sourced spec ranges.

Next, in order: (1) thread per-cell Material INTO the sinkage/slip solver, so it changes the dynamics and
not just the prediction (the solver still reads the global Bekker `k_phi`); (2) close the
selection-to-render loop, so picking a map area in the browser crops a DEM window, renders it in Godot,
and feeds the observed map back to the planner (the pieces exist but are not yet one pipeline); (3) dense
MVS to fill the ground-tier coverage (CUDA-gated today); (4) the learned perception model (JEPA or RSSM
over the cheap headless rollouts) for active perception, the one genuinely learned component this
architecture calls for.
