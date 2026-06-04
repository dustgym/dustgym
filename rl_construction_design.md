# Hierarchical RL + world model for autonomous lunar earthmoving (foss_ipex)

**Status:** design / research direction (2026-06-02). Grounded in the actual sim (Phases 1-4 +
RL env), not a generic spec. Distinguishes what is reachable on the existing Tier-2 authority *now*
from what is gated on Tier-3 forces, the §10 map channel, render throughput, or a world model.

---

## 0. Framing: four levels, and where we are

| Level | Objective | Problem class | Status in sim |
|---|---|---|---|
| 0 | reach a waypoint | navigation | done (drive loop, Phase 3) |
| 1 | reach it despite slip | terramechanics-aware nav | done (slip ladder + closed loop, Phases 2-3; PPO/CEM solve it) |
| 2 | change the terrain | manipulation / excavation | **mechanically supported (Tier-2 cut/fill), not yet trained** |
| 3 | terraform geometry to spec | construction planning | the target; hierarchical + goal-conditioned |

The leap is Level 1 → 2 → 3: from *following a pose* to *transforming `H_current` into `H_target`*.
That is a different objective and the reason a bigger architecture (skills, goal-conditioning, world
model) finally earns its keep (a linear policy already saturates Level 1).

---

## 1. The non-negotiable invariant (this is what keeps it physical)

**Learned components issue actions / skill parameters. The physics authority is the only thing that
mutates terrain.** The policy/planner never writes the DEM. The authority
(`column_state` + `rover` + `slip`) applies the action and enforces, by construction:
- **mass conservation** (`cut_to_inventory` → `drum_inventory` → `dump_from_inventory`; density-only
  edits; height re-derives),
- **slip / sinkage coupling** (Phase 2-3: achieved motion = f(commanded, slope, soil)),
- **determinism** (replayable rollouts).

Consequence: the terrain-matching reward below **cannot be hacked**. The rover cannot conjure mass to
fill a hole; it must cut it from somewhere else and haul it. The physics invariant *is* the constraint
that makes the construction reward well-posed. No learned module may bypass the authority.

---

## 2. What the sim already provides (map skills → existing code)

| Capability | Code | Fidelity |
|---|---|---|
| Slip-aware locomotion | `rover.step_pose` + `drive.drive_step` + `slip.py` | geometry/state-accurate; slip from slope (Tier-2) |
| Cut (excavate) | `rover.drum_pass` → `cut_to_inventory` | mass-conserving, geometric. **NOT force-accurate** (Tier-3 gap) |
| Carry payload | `ColumnState.drum_inventory` (kg) | exact |
| Dump (deposit) | `dump_from_inventory` (SPOIL at loose density, bulking) | mass-conserving |
| Grade / compact | `four_wheel_pass` (TREAD/COMPACTED_BERM) + sandpile relax | mass-conserving |
| Entrapment + recovery | `slip.slip_sinkage_equilibrium` (runaway / back-off) | Tier-2 |
| Conserved DEM state | `ColumnState` (mass_areal invariant; height derived) | exact |

**Two honesty caveats that shape the whole program:**
1. **Excavation is geometric, not force-accurate.** The drum cut moves mass conservatively but models no
   dig torque / reaction. Learned earthmoving skills will be *geometry- and state-accurate* (the spec's
   own thesis), **not** force-accurate, until Tier-3 (Chrono::GPU DEM, FIX-1/2, euclid) lands. This is
   fine for construction-geometry autonomy; it is not fine for actuator-load or throughput claims.
2. **Perception (camera) is gated.** Shadow/sun-based terrain inference (your idea, §9) is genuinely
   supported by the sensor-faithful renderer (Hapke BRDF, grazing sun), but training on rendered images
   needs render-in-the-loop throughput (the 725 ms PNG-egress bottleneck) and the unbuilt §10 map
   channel. Until then the perception track is offline / low-rate.

---

## 3. Skill library (the vocabulary, not a monolith)

Train **primitives**, compose them with a planner. Each skill = its own goal-conditioned policy with a
termination condition, so it is reusable and separately benchmarkable.

| Skill | Action | Termination | Reward (control-level) | Sim support now |
|---|---|---|---|---|
| `TraverseTo(x,y)` | twist | at target / stuck | progress − slip − energy | **built** (Phase 3) |
| `FollowPath(path)` | twist | path end | cross-track err − slip | built |
| `RecoverFromEntrapment` | twist (back-off/reverse) | slip < thresh | regained mobility | slip model built (Phase 2) |
| `Excavate(area, depth)` | drive + drum-engage | drum full / depth met | mass cut toward target − spill | Tier-2 cut built; force = Tier-3 |
| `Haul(src, dst)` | twist (laden) | at dst | progress − slip(laden) − spill | built (payload weight feeds slip) |
| `Dump(loc, vol)` | drive + drum-release | inventory empty / vol met | placed-mass accuracy | built |
| `Grade(area, slope)` | wheel passes | H-error < tol | −\|\|H−H_target\|\|, smoothness | compaction + sandpile built |
| `BuildBerm(polyline,h)` | compose cut→haul→dump→grade | berm-to-spec | −\|\|H−H_target\|\| over polygon | composition of the above |
| `FillHole(area)` | compose | flat to tol | −\|\|H−H_target\|\| | composition |

Note: every skill's *physics* is already in the authority; what is learned is the **control policy**
that drives the rover to make the conserved terrain edit happen in the right place.

---

## 4. State representation (goal-conditioned)

Current obs (RoverSimEnv): `local_heightmap (5x5)`, pose, slip, sinkage, dist-to-goal. Extend to:

```
obs = {
    local_heightmap,     # current DEM patch (CNN input once 2D)
    target_heightmap,    # the goal terrain patch  <-- the critical addition
    rover_pose,          # row, col, yaw
    bucket_fill,         # = ColumnState.drum_inventory (kg)  [already exists]
    slip_ratio,          # from slip model
    terrain_type,        # = state_label enum (VIRGIN/TREAD/EXCAVATED/SPOIL/COMPACTED_BERM)
    sun_azimuth/elev,    # site config (drives shadows, §9)
    mission_clock,
}
```

`target_heightmap` turns the problem from "dig here / dump there" (hand-scripted) into **terrain
matching** (learned). `bucket_fill` and `terrain_type` already exist in `ColumnState`; the addition is
the goal channel + (for the perception track) the camera/shadow features.

---

## 5. Terrain-matching reward (the key shift)

Per-skill and per-mission reward on terrain *state*, not actions:

```
R_match = -|| H_current - H_target ||      (over the work polygon)
```

with shaping terms: −slip-events, −energy (∫ load·distance), −time, +completion. Because the authority
conserves mass, this is **well-posed and unhackable** (fill must be cut from elsewhere). Long-horizon
credit assignment (hundreds of steps before the berm matches spec) is exactly where dense shaping +
a **world model** (§8) pay rent. A clean dense surrogate: reward the *reduction* in H-error each step
(potential-based shaping, preserves the optimal policy).

This reward is the §10 "observed-map vs true-terrain" channel's *control-side twin*: here `H_current`
is ground truth (control reward); the perception version compares the rover's *observed* map to truth.

---

## 6. Hierarchy (what is symbolic vs learned vs model-based)

```
Mission Planner      (symbolic: "build a 5 m landing pad here")        <- search / rules
  -> Task Planner    (symbolic/optimization: decompose to skills + regions)
    -> Skill Selector(learned or scripted options-policy: which skill next)
      -> Skill Policy(LEARNED RL: the primitive controllers, §3)
        -> Low-level Controller (drive_step: twist -> conserved physics)
```

Recommended split: **low-level skills learned** (RL, the hard part — slip-aware control of cut/haul/
dump/grade); **skill-selector** starts *scripted/symbolic* (skills are reusable primitives, so a planner
can sequence them) and is later replaced by a learned options policy / HRL; **task & mission planners
symbolic** (classical planning over a terrain-state goal). This is the options framework / feudal HRL,
and it matches how DARPA RACER and manipulation stacks are built: learn skills, plan over them.

---

## 7. Curriculum (mapped to reachable-now vs gated)

| Stage | Task | Reachable on Tier-2 now? |
|---|---|---|
| 1 | waypoint | done |
| 2 | slip-aware waypoint (don't get stuck) | done (PPO/CEM) |
| 3 | move one pile (cut → dump) | **yes** (drum cut/dump built); needs target reward + train |
| 4 | fill a hole to flat | yes (composition + H_target reward) |
| 5 | flatten a construction pad | yes |
| 6 | build a berm to spec | yes (geometric); force-fidelity = Tier-3 |
| 7 | arbitrary `H_current → H_target` | yes (general earthmoving policy) |
| 8 | multi-skill mission (pad + road + berm) | needs the planner (§6) |

**Do not start at berms.** Stages 3-5 on the existing Tier-2 cut/fill are the reachable, fundable core
and a legitimate result on their own (geometric earthmoving autonomy with conserved mass + slip).

---

## 8. World model / JEPA (where it actually pays rent)

Not for Level 1 (linear solves it). It earns its keep at Levels 2-3:
- **Long-horizon credit assignment.** Grading is hundreds of steps before terrain matches spec. A latent
  dynamics model lets the planner *imagine* "cut here / dump there → predicted H" before acting, instead
  of learning from sparse end-of-episode reward.
- **Sample efficiency for the expensive perception track** (§9). When each env step is a camera render,
  model-free PPO is infeasible; a world model (Dreamer-style: train in imagination) is the standard fix.

**Physics-based vs learned split (the hybrid that respects §1):** keep the *terramechanics authority
physics-based* (mass, slip, conserved DEM). Learn only the **latent predictive model** used for
*planning*: encoder (DEM and/or camera → latent) + dynamics (latent_t, skill → latent_{t+1}). The
authority remains ground truth for execution; the world model is the planner's imagination, not the
simulator. JEPA fits as the **encoder / latent-prediction** front-end (your clean lewm lineage): predict
future *latent terrain representation* under a skill, rather than pixels. Dreamer/RSSM is the recurrent
alternative for the dynamics core. Recommended sequencing (your instinct is right): a **self-supervised
encoder (CNN/JEPA) on the sensor stream first**, then latent dynamics on top.

---

## 9. Lunar perception: shadows + solar geometry (your strongest idea)

The renderer is sensor-faithful (Hapke/Lommel-Seeliger BRDF, 0-7 deg grazing polar sun, hard shadows),
so **shadow length + sun azimuth + slope are tightly coupled and learnable**: an encoder can infer
crater depth / berm height / obstacle shape from shadow geometry, exactly as Apollo crews estimated
relief visually. For a polar rover where GNSS does not exist, this is more useful than pose priors:

```
obs_perception = { image, shadow_vectors, sun_azimuth, sun_elevation, mission_clock }
```

This is the bridge to the §10 map channel: shadow-derived relief → observed DEM → `score_map` →
perception reward. **Prerequisites (honest):** render-in-the-loop throughput (the 725 ms egress
bottleneck → stream textures / in-memory frames) + the observed-map producer (does not exist yet). So
the shadow/perception track is real and high-value but is the *second* track, gated behind the
control-skill track.

---

## 10. Multi-objective + what is NOT modeled

Optimize: mission completion, energy (∫load·distance, available), traversal time (steps), slip-avoidance
(slip events, available), construction quality (H-error, available). **Not modeled, do not claim:**
tool/drum wear (no wear model), thermal/power budget over the lunar day (sun drives optics not power),
actuator loads (Tier-3). Flag these as out-of-scope reward terms, not silently zero.

---

## 11. Evaluation benchmarks

Per skill: TraverseTo (success rate, slip events, energy); Excavate (mass-cut accuracy, spill);
Haul/Dump (placed-mass error); Grade (final H-RMSE, % cells within tolerance). Long-horizon: berm-to-
spec (H-error over polygon, mass moved vs minimum, time, energy), pad flatness, road traversability.
Determinism + domain-randomization generalization (held-out slopes/soils/scenes) are first-class
metrics (the env already supports seeded DR over the sourced envelopes).

---

## 12. Honest prerequisites + reachable-now build order

**Reachable on the existing Tier-2 authority, no new fidelity needed:**
1. Goal-conditioned env: add `target_heightmap` to obs + the `R_match` reward (potential-based shaping).
2. Train **Stage 3-5 skills** (move-pile, fill-hole, flatten-pad) with PPO/SAC — *now* a CNN on the
   2D `local/target heightmap` is justified (the input is finally spatial).
3. A scripted **skill-selector** to compose trained skills into a berm (Stage 6), then a learned options
   policy (Stage 7-8).

**Gated (sequence them after the core):**
- Force-accurate excavation (drum torque/throughput): **Tier-3, FIX-1/2 euclid oracle.**
- Perception/shadow track (camera obs, §9): **render throughput + §10 map channel + observed-map producer.**
- World model (§8): most valuable once the perception track makes the env expensive; build the
  JEPA/CNN encoder first, then latent dynamics.

**Recommended first concrete step:** a `TerrainTargetEnv` (goal-conditioned, 2D heightmap obs + target,
`R_match`) + train the **flatten-a-pad** skill (Stage 5) with a CNN policy. It is the smallest thing that
(a) needs more than a linear policy, (b) exercises the conserved cut/fill loop end-to-end as *learned*
behavior, and (c) is a legitimate standalone result. Everything else composes from there.

---

*Architectural through-line: learned components command, the conserved physics authority executes;
skills are the vocabulary; the terrain-matching reward + mass conservation make construction well-posed;
the world model + perception/shadow track are the second-stage unlocks, sequenced behind the reachable
Tier-2 skill library.*

---

## M3 results + the dig-dominance finding (2026-06-02) — where RL/ML planning actually helps

Built the M3 resource layer on `SkillMacroEnv` (drum capacity, energy/battery budget, travel + dig cost)
and added a **Discrete** cell-selection action (classify-which-cell, the learnable form;
coordinate-regression was not). Grounded the energy model in real IPEx data
(`terrain_authority/ipex_specs.py`, Schuler ASCEND 2024 + the 12S/30 Ah/~44 V pack): drive 40 W → **135 J/m**,
dig 48 W → **4151 J/kg**, pack **1332 Wh**.

**Honest empirical finding.** Across continuous-PPO, continuous-BC, discrete-BC, and discrete-PPO, a
learned policy **ties or loses to the greedy planner** on the flatten task, and several configs sit at or
below random. Two robust causes, neither a tooling failure:

1. **The skill-macro abstraction makes greedy strong.** Each macro moves the selected cell *monotonically
   toward target*, so even random cell selection makes progress and roughly matches a planner. Cloning the
   greedy "nearest above-target cell" expert is also brittle — among many near-equidistant cells the
   argmin tie-break is arbitrary, so BC train accuracy caps near 50% (contradictory labels for
   near-identical observations). This is a brittle-expert artifact, not an unlearnable task.

2. **At grounded energy ratios the cost is DIG-DOMINATED.** Excavation is ~4151 J/kg vs ~2.7 J/cell of
   travel, and **dig energy is fixed by the conserved mass a target requires** (mass conservation). So the
   energy budget is mostly a *feasibility floor*, not an optimization lever — "plan to save energy" has
   almost no headroom on a single mass-moving objective, because the dominant term is set by the target
   geometry, not the planner's choices.

**Consequence for the roadmap.** Single-objective RL planning is largely solved by greedy once the macro
abstraction is in place; chasing a learner-beats-greedy result there is chasing a metric. The genuine
RL/search headroom is the **multi-objective scheduling layer** — *build pad A, wall B, road C with one
rover, one drum, a shared battery/time budget, and precedence constraints*. That is a vehicle-routing /
job-shop-flavored problem where ordering and allocation (not per-cell servicing) dominate, and it is
exactly the **SimCity-Space planner** the vision calls for ("select what I want built where and execute").
That is the recommended next construct, ahead of more single-task policy tuning.

**Prerequisite surfaced:** the realistic cut-haul-fill primitive (borrow → haul → build) needs a per-cell
deficit-aware deposit in the authority (DEFERRED_FIXES **FIX-4**); the current even-spread + spoil-bulking
`dump_from_inventory` overshoots, so a haul env can't converge yet. `build_berm` is likewise infeasible in
the region-restricted macro (no borrow source). Fix FIX-4, then the multi-objective layer has a feasible
material-moving substrate to schedule over.

---

## M4 results — RL/ML planning IS viable on the multi-objective layer (2026-06-02)

FIX-4 landed (`column_state.deposit_field` / `fill_toward`, PR #4), giving the conserved material-moving
substrate. Built `terrain_authority/scheduler_env.py`: several separated build sites + borrow pits, one
rover, one drum. **Each action is one strategic trip-leg** (`Discrete` over regions): load the drum at a
borrow pit, or dump it toward a build site via `fill_toward`. Because dig energy is conserved-mass-fixed,
what ordering controls is **makespan** — so the binding budget is leg count, and the horizon collapses to
the scheduling decision. Layout is **randomized per episode** (positions vary, counts fixed → stable
action space); mass conserved, energy grounded and tracked.

**Result on held-out randomized layouts** (`scripts/demo/train_scheduler.py`, PPO 200k steps):

| policy | success | avg legs |
|---|---|---|
| greedy nearest-batch (strong baseline) | 100% | 28 |
| random | ~3% | (budget) |
| **PPO (learned, generalizes)** | **100%** | **27** |

A learned policy solves randomized multi-site cut-haul-fill scheduling, **matches/edges the hand-coded
planner, and beats random by ~97 points.** This is the headroom that was absent on flatten: the leg
ORDER changes the outcome, so planning (search or learned) is genuinely valuable. Tests
(`test_scheduler_env.py`, 6, numpy-only) pin mass conservation, planner-beats-random under budget,
no-overshoot fills, layout randomization, determinism.

**Closing the viability question honestly:** single-objective construction is solved by greedy (the macro
abstraction + conserved physics make it well-posed and dig-dominated → no learning headroom). The
multi-objective **scheduling** layer is where learning earns its keep, and a learned scheduler is viable
there — generalizing across layouts and matching a strong planner. That is the SimCity-Space planner's core
loop: *load a map → pick what to build where → a learned/searched scheduler executes it under conserved
physics + a resource budget.*

**Is the learned MLP the best policy? No — model-based search + distillation is (M4++, corrected).**
A first attempt added an analytic `min_legs_lower_bound` and concluded "PPO is near-optimal" — that was
**wrong** and has been removed. The bound counted `ceil(full_demand/drum)` dumps per site, but success is to
TOLERANCE (each cell within 1 cm), which needs ~4 dumps/site not 5, so the "bound" (28) was actually
*above* what is achievable. A model-based **beam search** — using the exact, deterministic, sub-ms authority
as its own simulator — finds **24 legs** (validated: sites to spec, mass conserved), below greedy (28) and
model-free PPO (27). Corrected picture on held-out layouts:

| policy | legs | |
|---|---|---|
| greedy heuristic | 28 | hand-coded |
| PPO (model-free MLP) | 27 | beats greedy, **not** optimal |
| beam search (model-based) | 24 | the optimum |
| **search-distilled (SAME MLP)** | **24** | optimum, no search at inference (BC loss ≈ 0) |

**CNN vs DNN is the wrong axis; algorithm class is the lever.** The 27 → 24 gain came from the TRAINING
SIGNAL (a model-based search teacher), not the network — it is the *same* MLP PPO uses. A CNN / transformer /
GNN over the same compact feature obs would not beat 24: model-free RL is the bottleneck, not capacity.
Because the conserved authority is an exact cheap simulator, model-based search dominates model-free RL, and
the best learned policy distills that search (the AlphaZero pattern). We do **not** need to *learn* a world
model for planning — we already have the true one; a learned world model only pays off where the true model
is expensive (the camera/sensor perception track, where rendering is costly).

**When the other architectures DO matter:** CNN when the obs becomes the raw spatial heightmap/map (vision —
perceiving terrain/shadows directly); attention / pointer-net / GNN when the number of regions (build orders)
is variable (the SimCity-Space goal — the current MLP is fixed to a region count); recurrence/transformer for
long-horizon memory. Each is dictated by the OBS representation + task structure, not by wanting "more power."

**Where a strict learned ≫ greedy margin also lives (beyond single-fleet):** *multi-vehicle* coordination —
K rovers, makespan = max over rovers, so the win is PARALLELISM + conflict avoidance (independent-greedy
rovers collide on the same job and serialize). That is a different mechanism, independent of dig-dominance.
Plus precedence (clear before build) and a per-charge battery forcing charger returns. (Note: a *probed,
reverted* capacity-stranding regime did NOT create single-fleet headroom — leg-count is allocation-
insensitive: assignment moves only the dig-dominated-negligible travel term. The headroom is in drum
BATCHING/packing, which search captures and the distilled policy learns.)
