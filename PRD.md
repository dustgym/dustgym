# PRD — dustgym: Lunar Construction Planner + Mission-Control Report

**Date:** 2026-06-04 · **Status:** living (v5, production-grade + single-software + autonomous-planning-limits reframe; see `docs/autonomous_planning_review.md`). **Software:**
there is one software, the **dustgym** monorepo (flat layout: `terrain_authority/` core + `dustgym/` package
+ `planet_browser/` product, all at the repo root); the former `roversim` dev tree is deprecated and folded
in. The conserved Tier-2 terramechanics core originates with **John McCardle** (CC0 provenance); dustgym is
the single CC0 software going forward. **Related:** `docs/architecture_review.md` (the production-readiness
review this version answers), `docs/world_model.md`, `building_taxonomy.md`, `planet_browser/mission_planner.py`
(planner + report), `planet_browser/server.py` (the API). **Legend:** ✅ done · 🟡 partial · ⬜ to build · ⛔
gated (render throughput / external oracle). **Priority:** P0 core-now · P1 next · P2 later · P3 research-bet.

**This version's intent (new):** make dustgym a **production-grade system**, not a research artifact. The
science core is correct, honest, fast, and well-tested (296 tests, conserved + sub-ms); the gap is the
operational shell. The production requirements are first-class here (Section 6, N9-N18), not a deferred
appendix. A six-agent architectural review (`docs/architecture_review.md`) graded the system **pre-production**
and set the roadmap this PRD now encodes.

---

## 1. Product

**This is construction-planning software, not a thesis.** Load a Moon or Mars map, place and sequence
build orders (cut a pad, fill a crater, grade a road), and an ML/optimizer plans and optimizes the
sequence under real physics, battery, and time constraints, then emits a **2-3 page mission-control
report**: coordinates, ordered actions, speed and battery-draw graphs, route and material-flow map,
energy/time/mass totals. **SimCity-style today** (the human places intent, the planner sequences and
optimizes it), **eventually autonomous**. Underneath sits a validated, conserved Tier-2 lunar
terramechanics core (the science): mass-conserving cut/fill, slip-sinkage, real-DEM maps, and an
env_checker-clean RL substrate, so the plan the report prescribes is physically feasible by
construction (it can never move mass that isn't there, or spend energy it doesn't have).

**The output IS the product.** The deliverable a user takes away is the report, not a score. The
planner's value is in **scheduling and sequencing** (cut-fill balancing, route order, battery-aware
recharge), which is where single-rover RL/search earns its keep; single-task routing is physics-bounded
and a greedy planner already solves it.

**Two faces kept strictly separate:** (a) the validated headless authority + planner core (the
science), (b) the visual front-end (planet browser, build-order queue, the report). The project also
exposes a reusable Gymnasium suite (`dustgym`) and an authorable benchmark, but the headline product is
the planner and its report.

## 2. The layer stack (build bottom-up; each area maps to a layer)

```
L8  3D APPLICATION / visual program  (load map UI · click-to-place build orders · top-down + 3D views ·
                                       multi-agent viz · telemetry/leaderboard · HITL)            [area M]
L7  Challenge / Mission system  (declarative challenge + BuildOrder/Mission grammar + scored runner)[J]
L6  Structures + Planner + Resources  (composite structures, mission/task planner, energy/battery)  [I,K]
L5  Construction skill library  (the taxonomy verbs, RL-learned)                                    [H]
L4  RL environment  (gymnasium env, training)                                                       [G]
L3  Sensor model / rendering  (Hapke 3D, camera, AprilTag)                                          [F]
L2  Scale / LOD  (quadtree, tiling, streaming)                                                      [E]
L1  Map  (procedural generation + REAL lunar DEM loading)                                        [C,D]
L0  Physics authority  (conserved Tier-2 terramechanics, slip, cut/fill — agents only command)      [A,B]
        + cross-cutting: World model [L]   ·   Perception/camera [F/⛔]   ·   Non-functional [N]
```

## 3. Goals / Non-goals
**Goals:** conserved, calibrated, sensor-faithful Tier-2 physics; **load real lunar maps** + procedural
maps; a skill library composed into structures by an RL/ML+symbolic planner under physics/energy/time
budgets; an authorable, scored, reproducible benchmark; a **3D application** to load/select/build/watch.
**Non-goals:** full granular-DEM at map scale (Tier-3 = offline oracle); flight-certified autonomy
(target = ground HITL/benchmark); arcade feel over fidelity; tool-wear/thermal-power physics.

## 4. Users
RL/autonomy researcher · benchmark/mission author · HITL operator-training user · GMRO/reviewer
(checks conservation + honesty) · **app user** (loads a lunar map, places build orders, watches it build).

## 5. Functional requirements (by area → layer → status)

### A. Physics core (L0)
| ID | P | Requirement | Status |
|---|---|---|---|
| A1 | P0 | Mass-conserving column model; height derived | ✅ `column_state` (drift 2.99e-16) |
| A2 | P0 | Load-bearing Bekker sinkage | ✅ `terramechanics` |
| A3 | P0 | Slip-sinkage + runaway/recovery | ✅ `slip.py` |
| A4 | P0 | Cut/haul/dump/compact (conserved earthmoving) | ✅ `drum_pass`/`dump`/`four_wheel_pass` |
| A5 | P1 | Reduced-gravity (Lyasko) magnitude fit | ⛔ FIX-1/2 euclid oracle |
| A6 | P2 | Force-accurate excavation (drum torque) | ⛔ Tier-3 (no granular DEM) |

### B. Mobility / closed loop (L0)
| B1 | P0 | Diff-drive integrator `step_pose` | ✅ |
| B2 | P0 | Closed loop w/ slip feedback (cmd_vel) | ✅ `drive.py` + `poll_cmd_vel` |
| B3 | P1 | Clast ride-over in loop | ✅ |

### C. Procedural map generation (L1)
| C1 | P0 | Craters/boulders/fbm calibrated to real stats | ✅ `procgen*` |
| C2 | P0 | Domain randomization from sourced envelopes | ✅ `domain_randomize` |
| C3 | P1 | One-seed composite map (multi-feature) | 🟡 generators exist; unified builder partial |

### D. **Load lunar maps (real DEM)** (L1) — first-class
| ID | P | Requirement | Status |
|---|---|---|---|
| D1 | P0 | Ingest real PGDA LOLA polar DEM (GeoTIFF) → map | ✅ `dem_import.load_lola_geotiff` + `build_from_dem` (south-polar stereographic, no reprojection) |
| D2 | P0 | A committed, loadable real lunar tile | ✅ `samples/lunar_dem/haworth_10km_5m` (2000², 10 km @ 5 m, real relief −96..+2842 m) |
| D3 | P0 | Crop/select a region + resample to work cell | ✅ `crop_square`/`dem_to_base` (window + base-cell) |
| D4 | P1 | General lunar products (other tiles/projections, equatorial → reprojection) | ✅ `dem_import.reproject_cylindrical` (pyproj: cylindrical lat/lon → local aeqd metres); real LOLA `ldem_4` equatorial patch ingests + relief round-trips + is planner-usable. GeoTIFF/COG `/vsicurl` = follow-on |
| D5 | P1 | Tile/stream a large map (don't hold km-scale in RAM) | ✅ `read_dem_window` (seek per row, exactly window bytes) + `flattest_anchor_streamed` (tile scan): 9.4 MB vs 160 MB peak, same result. Full live wiring (server reads windows) = follow-on |
| D6 | P1 | Raw-tile acquisition path (PGDA fetch / vendored input) | 🟡 vendored `.vendor/lola_raw/` (gitignored); doc + fetch helper ⬜ |
| D7 | P2 | Interactive region/tile selection (in the 3D app) | ⬜ (area M) |

### E. Scale / LOD (L2)
| E1 | P0 | Interaction-keyed quadtree LOD | ✅ `quadtree.py` (21MB vs 4GB demo) |
| E2 | P2 | Multi-site / multi-agent active regions | ⬜ |

### F. Sensor model / rendering (L3)
| F1 | P0 | Sensor-faithful Hapke render (grazing sun, shadows) | ✅ Godot `godot_sidecar` |
| F2 | P0 | AprilTag pose-vs-truth | ✅ 12.7mm/7.15° |
| F3 | P1 | Render-in-loop throughput (camera-RL) | ⛔ 725ms PNG egress |
| F4r| P2 | Calibrated lens distortion | ⬜ Brown-Conrady stub |

### G. RL environment & training (L4)
| G1 | P0 | Gymnasium env (passes env_checker) | ✅ `rover_env.py` |
| G2 | P0 | Goal-conditioned construction env (`H_target`) | ✅ `terrain_target_env.py` (drive + drum cut/dump) |
| G3 | P0 | Honest control reward + domain randomization | ✅ |
| G4 | P0 | Trainable (real RL converges) | ✅ PPO 0→100%; CEM 60→100% |

### H. Construction skill library — taxonomy verbs (L5)
| ID | P | Skill | Status |
|---|---|---|---|
| H1 | P0 | TraverseTo / FollowPath / Recover | ✅ physics; trainable |
| H2 | P1 | Grade / Compact (flatten primitives) | 🟡 physics ✅; **policy = M2 next** |
| H3 | P1 | Excavate / Haul / Dump | 🟡 physics ✅; policy ⬜ |
| H4 | P2 | BermBuild / FillHole (composite skills) | ⬜ |
| H5 | P2 | Sinter / Melt (fuse pad/road, the lunar concrete analog) | 🟡 authority ✅ (`column_state.sinter`, tested, mass-conserving) + WorkSite seam present but **GATED OFF** (`SINTER_ENABLED=False`): energy/density are [CALIB], not IPEx-grounded (IPEx has no sinter tool) |

### I. Structures + planner — taxonomy nouns (L6)
| I1 | P1 | Composite structures (Pad/Road/Berm/SolarPad/Foundation/BorrowPit/CraterFill) | ⬜ defined in taxonomy; decomposition+specs |
| I2 | P1 | Mission/Task planner (structure→skills + mass routing source↔sink + schedule) | 🟡 `scheduler_env.py` (trip-leg scheduling; beam-search 24 legs, PPO 27, greedy 28); structure-decomposition front-end ⬜ |
| I3 | P2 | Learned skill-selector (HRL options) | ⬜ |
| I4 | P0 | **Sequence optimizer** (cut-fill balancing source→sink + route order + battery-aware mid-task recharge) | ✅ `mission_planner.balance` + `_build_trips`/`_simulate`/`plan_and_simulate` |
| I13 | P1 | **Pluggable algorithm × objective** — run different path-planning/optimization algorithms, optimize/sort by any metric (duration, energy, power, distance, recharges, mass) incl. **multi-objective**; multi-vehicle off-by-default seam | ✅ **7 algorithms** `optimize_sequence`: heuristics **nearest/greedy/two_opt/or_opt/lk** (sim-scored), **exact** **brute** (≤7 perms) + **Held-Karp** DP (exact driving tour, ≤16, SOP-aware), and **auto** (dispatch: brute ≤7 · **Held-Karp-seed → LK-polish** 8-16 · LK >16 — "solved in sequence"). **Multi-objective**: `parse_objective` accepts a name, a `name:w,..` weighted spec (reference-normalized), or a dict. `compare_algorithms` ranks best-first + flags the **Pareto** frontier (non-dominated over time/energy/distance/charges). `/plan`+`/compare` take `algorithm`/`objective`/`precedence`; browser has algorithm + objective (incl. "balanced") + precedence inputs and a Compare table (★ best, ⬩ Pareto). Live (10 trips): auto **7.03 km** < lk 7.09 < held_karp 7.36 < nearest 7.50; Held-Karp = exact driving tour (verified vs full enumeration). **Multi-vehicle gated** (`vehicles=1`; `>1` raises → `terrain_authority/scheduler_env.py`). 15 tests. Multi-path coordination = future. |
| I5 | P0 | **Mission-control report** (2-3pp PDF: trip table, route+material-flow map, battery%/speed vs time, per-trip + cumulative energy/mass, material balance) + markdown | ✅ `mission_planner.report` (`planet_browser/reports/`) |
| I6 | P0 | **Terrain-aware siting** — read the DEM at each order's footprint (slope); reject sites above a buildability threshold | ✅ **LIVE** — `validate_plan(dem=, dem_origin=, max_slope_deg=)` + `load_haworth_dem`/`slope_deg_map`: real Haworth gate (flat 0.0° feasible, crater wall 69.8° rejected); **wired into `/plan` for Moon** via M11 anchor (cached DEM, graceful fallback). `test_mission_planner` slope + live-server tests |
| I7 | P0 | **Bulking-correct balance** — balance by MASS with the in-situ→spoil swell (cut ρ_deep ≈1920 → fill ρ_spoil ≈1300, ~1.5× volume), not by volume | ✅ **both layers**: `structures.py` `SWELL=RHO_DEEP/RHO_SPOIL` (≈1.48, single-source, loose fill bulks +48%) **and** `mission_planner` mass model (cut @ρ_bank, fill @ρ_loose) so the planner no longer reports a phantom deficit on bulked structures; mass exact, `test_structures` mass-balance tests |
| I8 | P0 | **Plan validation on the conserved authority** — execute the plan through `column_state` for real, mass-exact feasibility, not the abstract footprint estimate | ✅ `mission_planner.validate_plan` rasterizes orders onto a `ColumnState`, runs cuts→drum→fills; returns feasible / mass_conserved (drift 0.0) / executed-vs-planned kg; flags too-deep cuts (datum floor); `test_mission_planner` validate tests. (On a flat scene now; real-DEM siting = I6) |
| I9 | P1 | **Precedence / dependency DAG** — order build steps by dependency (grade road before haul on it; dig borrow before the berm it feeds; level pad before its berm), not spatial TSP alone | ✅ `Mission.precedence` (before→after action pairs) → `trip_precedence` lifts to trip constraints → **every** sequencer respects them (eligible-set for nearest/greedy, valid-permutation filter for brute, **SOP-aware Held-Karp** masking, topology-valid moves for 2-opt/Or-opt/LK); `/plan`+`/compare` accept `precedence`, browser has a precedence field; `test_precedence_is_respected_by_every_algorithm` |
| I10 | P1 | **Hazard-aware routing + slope/slip energy** — route hauls on a DEM costmap (avoid craters/steep/PSR), with slope- and slip-aware leg energy, not straight lines at flat 135 J/m | ✅ `slope_costmap` (cost = 1 + slip·tan θ; impassable > traverse cap) + `route_least_cost` (8-conn Dijkstra) + `routed_distance`; wired into `plan_and_simulate`/`run`/`/plan` for Moon (real Haworth, cached DEM); totals carry `routed_haul`/`blocked_legs`/`haul_detour_frac`; report + browser show the detour. Live: spread hauls +4.5% around hazards; 4 routing tests. **Slope energy: exact gravity lift DONE** — `haul_elevation_gain_m` + `body_gravity` add `mass·g·Δh` (real-DEM Δh) per uphill haul to the energy/battery/time, `totals.lift_energy_J`, surfaced in the report (live: 0.14 MJ; Mars no-DEM 0); exactness test. **Slip IS coupled into haul drive energy** via a single `[CALIB]` slip-vs-slope shape (`slip_alpha_to_slip`, a `1/(1−slip)` haul multiplier), **not** the full conserved `slip.py` ladder; the `[CALIB]` shape is the remaining ceiling (was previously mis-stated as "deferred") |
| I11 | P1 | **Per-structure acceptance** — verify flatness RMSE / berm profile / bearing vs spec (taxonomy §3), and enforce angle-of-repose + compaction so fills hold | ⬜ |
| I12 | P2 | **Robust plan / uncertainty bands** — confidence on energy/time/feasibility vs DEM error, [UNKNOWN] soil, slip variance, and the drum-fill ± | ⬜ (drum-fill ± is the only uncertainty modeled) |

### J. Challenge / Mission system (L7)
| J1 | P0 | Declarative `Challenge` schema (+JSON) | ✅ `challenge.py` |
| J2 | P0 | Deterministic `realize(seed)→map+target` generator | ✅ |
| J3 | P0 | `run(agent,challenge)→Scorecard` runner | ✅ `challenge_runner.py` |
| J4 | P1 | **BuildOrder/Mission grammar** (multi-structure: select what/where + global budget) | ⬜ extends J1 |
| J5 | P1 | Curriculum/difficulty tiers + held-out-seed generalization | 🟡 tier + reset(seed); full ladder ⬜ |

### K. Resources / constraints (L6)
| K1 | P0 | Mass budget (conserved) | ✅ `total_mass()` |
| K2 | P0 | **Energy / battery model** (capacity, draw per skill·load·dist·dig, recharge) | ✅ grounded in real IPEx (`ipex_specs.py`, NTRS 20240008162): drive 135 J/m, dig 4151 J/kg, 4.79 MJ pack; recharge/reserve [CALIB]. **+ exact gravity-lift** `mass·g·Δh` for uphill hauls (real-DEM Δh, per-body g; `totals.lift_energy_J`) |
| K10 | P1 | **Endurance / per-sortie range** ("true distance before recharge") — slope+slip-adjusted, DEM reach, body-correct timescale, ConOps reconciliation | ✅ `single_charge_range_m` ([135 J/m × 1/(1−slip) + rover_mass·g·sinθ]) + `reachable_radius_on_dem` (Dijkstra drive-energy field over the slope+slip costmap) + `body_timescale` (per-body synodic day/daylight/sun-window) + `endurance(mission, dem=)`. Surfaced in `/plan`, PDF + markdown report, browser. **Grounded:** 32.1 km flat / 26.2 km slope+slip @ Haworth 17° median; whole 10 km tile reachable for ~37% of pack. **Per-body timescale [corrected]:** Moon 1 day ≈ 29.5 Earth-days (354 h daylight) → a 30 h sortie fits ~7× in the ~9–11-day sun window (NOT window-bound); Mars 1 sol ≈ 24.7 h → the same sortie spans ~2.4 sols. **ConOps [SCHULER24]:** 70 km + 5–10 t over 11 days → drive ~2 packs vs dig ~4–9 packs → **drums dominate** (recharged daily). Key physics: rolling+slip dominate, gravity-climb minor in lunar g. 5 tests |
| K3 | P1 | Time / mission clock (+ sun window) | ✅ steps; sun coupling 🟡 |
| K4 | P1 | Slip-risk / entrapment budget | ✅ |
| K5 | P2 | Tool/drum wear | ⬜ not modeled (flag, don't score) |
| K6 | P1 | **Drum-mass inference + arm-lift energy** (know drum fill from motor current; no load cell) | ✅ grounded `rassor_mass_model.py` (ICE-RASSOR, NTRS 20210022781): linear AR/FDC/EDC + MPE fill-uncertainty + gravity-work arm-lift; coefficients fit-from-data (not fabricated) |
| K7 | P1 | **Drum-fill sensing observable + offload autonomy** | ✅ `DrumSensor` (forward `freespin_drum_current_a` + calibrated-on-conserved-signal inverse + `should_offload`) with a **toggleable seeded noise** (`noise_frac=0` off by default, deterministic). Wired into `worksite_env`/`scheduler_env` (optional `drum_sensor` → sensed drum-fill obs, default off = non-breaking), the planner report (`drum_cycles` + sensed-fill note), and the web (`server.py POST /sense` + the browser DRUM SENSOR widget with a noise checkbox). `test_drum_sensing.py` + `/sense` tests |
| K8 | P1 | **Realistic surface power** — at a PSR (Haworth) there is NO sun to charge from; power is a lander/tower budget, with IPEx thermal derating (−35/+40 °C, FIX-5) and the 14-day day/night cycle | ⬜ **wrong for the work site:** charging is a flat `[CALIB]` 700 W at (0,0) |
| K9 | P2 | **Operational windows** — sun / thermal / comms windows coupled to the mission clock (drive/dig/charge gated by availability) | ⬜ clock exists; no window coupling |

### Autonomous-planning limits — the ceilings on the I/K planner (stated explicitly; `docs/autonomous_planning_review.md`)
The planner solves single-rover, cut-fill-balanced, recharge-coupled routing genuinely — but its autonomy is
action-level, single-vehicle, open-loop-replan, and silently capped. The hard ceilings (the PRD previously
stated capabilities without these):

| ID | Limit |
|---|---|
| AL1 | **Exactness ceiling.** `brute` is exact on the chosen objective only ≤7 trips; Held-Karp is exact on *driving distance only* ≤16 (assumes dig dominates, order-independent); above 16, `auto` degrades to unbounded local search **with no quality bound and no user-facing warning**. |
| AL2 | **Infeasible-precedence cliff.** A cyclic / unsatisfiable SOP DAG makes `brute` raise and Held-Karp return a silently "successful" **0-trip plan**; there is no acyclicity/feasibility precheck. |
| AL3 | **Objective grammar can't express real constraints** — no deadline/time-window/makespan (K9), no soft constraints, no risk term; it optimizes an unconstrained-in-time world. |
| AL4 | **Action-level, not goal-level instruction.** The user enumerates every cut/fill + depth; the goal-level `Challenge.objective`+tolerance schema is disconnected from the product `Mission` (no "build a pad to ±2 cm, you sequence it"). |
| AL5 | **Footprints are scalar areas → axis-aligned squares** (a 15×2 m road becomes a 5.48 m square); no shape/orientation/corridor/polygon. `budget`/`scoring`/`priority`/`keepout` are **silently dropped** by `mission_from_dict` (the J4 grammar gap). |
| AL6 | **No as-built acceptance (I11 ⬜).** `validate_plan` checks mass conservation + the **center cell's** slope on a **flat synthetic mantle**, never the built structure vs its spec (flatness-RMSE / berm profile / repose). |
| AL7 | **Closed-loop autonomy = open-loop replan over a self-simulator.** It executes its own energy model (not telemetry / not perception), **battery is the only replan trigger**, there is **no fault detection or handling**, and pose σ runs open to ~11.5 m by dead reckoning without the (Godot-gated) perception fix. |

### MV. Multi-vehicle planning (L6/L7) — DESIGNED, UNBUILT (new area; `docs/autonomous_planning_review.md` §2)
There is **zero** multi-vehicle planning today (both the planner and `scheduler_env.py` are one-rover /
one-drum). The conserved per-cell authority gives fleet **mass/energy conservation for free** but has **no
multi-body dynamics** — collision can only ever be a planning constraint, not a simulated event. Staged design:

| ID | P | Requirement | Status |
|---|---|---|---|
| MV1 | P2 | **Fleet API** — `Mission` carries N rovers (count, per-rover start/charger, capability vector); `mission_from_dict` + `/plan` accept `rovers`; removes the `vehicles=1` raise | ⬜ |
| MV2 | P2 | **Task allocation** — `allocate(mission, rovers)` above `optimize_sequence`: sequential-greedy / regret-insertion bidding (bid = marginal `_simulate` cost via the real scorer), MILP/VRP exact oracle at ≤3 rovers; per-rover subsets reuse the existing sequencing pipeline | ⬜ |
| MV3 | P2 | **Spatial + temporal deconfliction** — prioritized planning over a cell/corridor reservation table (reuses `routed_distance` paths) + shared work-site time-windows; CBS fallback. Collision = scheduling constraint, not physics | ⬜ |
| MV4 | P2 | **Shared-resource scheduling** — charger as a queued single/k-server (a rover waits → wait = real makespan; fixes K8); borrow-pit / drum / ISRU-plant as locked decrementing resources in a fleet `_simulate` | ⬜ |
| MV5 | P2 | **Coordinated replan** — `run_closed_loop` extended to N shared-world `Belief`s; re-clear allocation (MV2) on recharge / model-error / pit-empty (AutoNav market re-clearing) | ⬜ |
| MV6 | P3 | **Heterogeneous fleet** — per-rover capability vector (drum cap, dig rate, drive speed, battery, tools) replacing the global `ipex_specs` singletons in the fleet simulator | ⬜ |
| MV7 | P2 | **Validity gate** — fleet mass conservation (free, one `ColumnState`) asserted; no double-claim on pits; charger-queue makespan accounted; a **2-rover EXACT baseline (extend `beam_search` to fleet state) required before any "learned ≫ greedy multi-vehicle" claim ships** | ⬜ |

Tractability: **2 rovers** exact-VRP-oracle viable (the only regime to validate learned-vs-exact) · **5**
auction + prioritized planning + queued charger (the charger queue becomes the dominant makespan term) ·
**20** auction + prioritized planning only (resource contention, not physics, caps useful fleet size).

### L. World model (cross-cutting, sample efficiency / planning)

**Full mapping in [`docs/world_model.md`](world_model.md)** (2026-06-04): the 5-layer world model for terrain
*transformation* (the robot reshapes terrain, it does not just drive through it) mapped onto the repo, with
the core design call. **Conserved physics for DYNAMICS** (exact, sub-ms, unhackable; model-based search
already beats model-free RL and a learned model for planning, the M4 finding) **+ a thin LEARNED model only
for PERCEPTION** (predict observations under the expensive render, for active "look before you dig"). NOT a
monolithic learned latent world model. The five layers and their status:

| Layer | Status |
|---|---|
| **Geometry** | ✅ `column_state` heightmap + slope + real LOLA DEM; planner cut/fill = target − current |
| **Material** | ✅ `material.py` (2026-06-04): per-cell friction + cohesion from the conserved density field across sourced spec ranges, + cut-difficulty + slip-susceptibility maps; `validation/map_channel/material_layer.png`. ✅ **THREADED into the solver** — `drive.drive_step(material=True)` overrides cohesion/phi from the rover's local cell (`material.cell_strength`); loose 0.199 vs compacted 0.058 slip on a 21.8° grade; default byte-identical; `test_material.py` (5). ⬜ per-cell `k_phi` sinkage too (cohesion/phi done; sinkage still uses the density-stiffening factor) |
| **Physics** | ✅ the Tier-2 authority: Bekker sinkage + slip ladder + IPEx energy at lunar g; `S(t+1)=f(S,Action)` is conserved + exact (removed-volume/energy/slip computed, not predicted) |
| **Task** | ✅ `mission_planner` + `structures.py` + `terrain_target_env` reward R=−‖H_cur−H_target‖ |
| **Uncertainty** | ✅ `autonomy.py` Belief/Kalman (pose/energy/drum σ) + per-cell terrain σ + `dig_ready_mask` (2026-06-04) |

| L1 | P3 | Learned encoder (CNN/JEPA) on DEM/sensor for the PERCEPTION branch (not dynamics) | ⬜ lewm lineage |
| L2 | P3 | Latent dynamics for imagination-planning | ⬜ DEPRIORITIZED — the conserved model is exact + unhackable; learn perception, not dynamics |

### M. **3D application / visual program** (L8) — the full software with visuals
| ID | P | Requirement | Status |
|---|---|---|---|
| M1 | P0 | Render primitives exist (Godot 3D + top-down DEM) | ✅ Godot renders + matplotlib top-down |
| M2 | P1 | **Interactive viewer** (load a map, pan/zoom/tilt) | 🟡 `planet_browser/index.html` (CesiumJS + NASA Trek WMTS; body dropdown Moon/Mars; pan/zoom/tilt; coord entry+load); sim-coupled 3D camera ⬜ |
| M3 | P1 | **Map-load UI** (pick real tile by body+coord → into the planner) | 🟡 real-tile select via Trek + coord-load ✅; procedural-seed + push-into-sim ⬜ |
| M4 | P1 | **Build-order authoring UI** (place footprints on the map = a Mission) | ✅ build-order panel (live mass·weight·energy·drum·dig-hr estimate via body g + bodies.json) + persistent **queue** (add/list/reorder/delete + from-pad/berm) wired to the planner (P1/S7) |
| M5 | P1 | **Execute + watch** (run planner → return the report; animate rovers / live terrain mutation) | ✅ PDF round-trip (P1/S7) + **top-down execution animation** (P5): `build_timeline` → `/plan` `timeline`; browser ▶ Execute view animates the rover along the route with a telemetry HUD (battery sawtooth / phase / position / mass), headless-render verified. Live terrain *mutation* during playback still ⬜. + ✅ **plan → render loop** (`scripts/plan_render_pipeline.py`, 2026-06-04): plan a flatten on a real scene (conserved cut→drum→fill) → write the worked AFTER bundle → render BEFORE/AFTER in Godot + quantify the earthwork (cut/fill volumes). The offline before/after terrain-mutation visual + the select-area→render loop CORE; browser `/render` endpoint (pick→crop DEM window→render) + perception feedback ⬜. 2 conservation tests; `validation/plan_render/`; see `docs/world_model.md` |
| M6 | P1 | Telemetry / scorecard / leaderboard overlay (mass, energy, time, slip, quality) | ⬜ |
| M7 | P2 | Multi-agent visualization | ⬜ |
| M8 | P2 | HITL controls (supervise / override / re-task) | ⬜ |
| M9 | P1 | **Web API + drum-sensor widget** (`server.py` `/plan` + `/sense`; browser build-queue + DRUM SENSOR readout with noise toggle) | ✅ P1/S7 + drum-sensing wired |
| M10 | P1 | **Mission persistence** — save / load / version a build project (mission JSON), not in-memory only | ⬜ |
| M11 | P1 | **Coordinate rigor** — a real site frame anchored to the globe lat/lon pick, with a lat/lon ↔ local-meters transform (today the queue uses ad-hoc `x,y` unrelated to the picked coord) | 🟡 **v0** — `flattest_anchor(dem)` auto-selects the flattest buildable Haworth region; `dem_origin` anchors the order local frame to it so the I6 slope gate fires on real terrain (`/plan` Moon path). **Remaining:** the true lat/lon→polar-stereo→DEM-cell projection from the *user's* globe pick (bundle has `world_bounds_m` + IAU south-polar-stereo frame) |
> **Engine recommendation (2026-06-02): web-first, not Electron.** A **React + three.js/react-three-fiber
> frontend + FastAPI backend** runs the Python sim and serves the existing on-disk state-field seam
> (HTTP/WebSocket) — zero-install/shareable (key for a benchmark + demos), reuses Aaron's FastAPI+React
> stack, keeps physics in Python. For high-fidelity 3D / the sensor view, embed **Godot (web-exported)**,
> which already does the Hapke render → likely **hybrid: web UI/authoring/leaderboard + Godot 3D view**,
> both consuming the seam. **Electron is the weaker choice** (per-OS packaging friction, no upside for a
> shareable research tool); reconsider only if heavy local-FS/offline desktop use is later required.

## 6. Non-functional (N)
| N1 | P0 | Mass conservation by construction (agents command; authority mutates) | ✅ |
| N2 | P0 | Determinism / replayability (seeded; no wall-clock RNG in dynamics) | ✅ |
| N3 | P0 | No synthetic/stub data; honesty tags ([CALIB]/[UNKNOWN]) | ✅ |
| N4 | P1 | Headless step perf (sub-ms authority step) | ✅ |
| N5 | P1 | License-clean core (numpy-only); heavy deps (SB3/torch/Godot) optional/gated | ✅ |
| N6 | P0 | Tests exist (regression coverage) | 🟡 **296 pytest** (`terrain_authority` 210 + `planet_browser` 86, + 26 in `scripts/`); all 10 registered `Dust/*` IDs pass strict env_checker locally. **But the gate is local-only** — see N9 (no CI runs them yet) |
| N7 | **P1** | **Production server** — ASGI (FastAPI/uvicorn): concurrency, request size/time limits, graceful shutdown, configurable host/port/workers. Today the server is a multi-threaded stdlib `ThreadingHTTPServer` with **thread-unsafe per-request report generation** (global pyplot) + no auth/limits | ⬜ |
| N8 | **P1** | **API hardening** — input size caps + path-traversal guards (✅ on `/reports/`,`/dem`) + auth on mutating routes + CORS policy + `pip-audit`; robust error handling | 🟡 input validation + traversal guards exist; auth/limits/CORS ⬜ |
| **N9** | **P0** | **CI gate** — a `ci.yml` runs on push/PR: `ruff check` + `pytest` (3.10-3.12 matrix) + strict env_checker (warnings-as-errors) on all 10 `Dust/*` IDs; pytest markers gate the GPU/Godot/COLMAP/Chrono tiers; merge blocked on green; publish `needs:` CI. **Today the only workflow is publish-only** | ⬜ |
| **N10** | **P1** | **Structured logging + observability** — `logging` (not the 360 `print()`); per-module loggers; server emits request + error logs (id/route/duration/outcome) + `/healthz` + `/metrics` | 🟡 server access-logging + previously-silent failure paths now route through `logging` (`planet_browser.server`, `$DUSTGYM_LOG_LEVEL`); TDD'd. CLI/self-test `print()`s are correct stdout. `/healthz` + `/metrics` land with the ASGI server (N7/N8) |
| **N11** | **P0** | **Code-quality tooling, committed + enforced** — `[tool.ruff]` + `[tool.mypy]` + `[tool.pytest.ini_options]` in `pyproject.toml`, `.pre-commit-config.yaml`, `py.typed`; wired into N9 | ⬜ |
| **N12** | **P1** | **Dependency hygiene** — a committed lockfile, version ceilings (esp. a tested `gymnasium` range), pinned `[rl]` extras, reproducible-install check (build wheel → fresh-venv import) in CI | 🟡 version ceilings added (numpy<3, scipy<2, gymnasium<2, torch<3, sb3<3) + `tomli` declared for the TOML overlay on py<3.11; a committed lockfile + the wheel→fresh-venv CI check remain |
| **N13** | **P1** | **Packaging completeness** — the published artifact contains the full advertised product (`planet_browser` + a server entry point), no synthetic-default registered envs, tests excluded from the wheel; or the wheel scope is documented | ⬜ `planet_browser` absent from wheel; 51 `sys.path` hacks |
| **N14** | **P1** | **Runtime invariant enforcement + input validation** — conservation / non-negativity / finite-state checkable at runtime (CI-gated, not `assert`); public physics + env constructors validate dims/cell-size/positive-density | ⬜ test-only today |
| **N15** | **P2** | **Externalized config (12-factor)** — env-overridable host/port/report-dir/DEM-bundle/`[CALIB]` knobs | 🟡 constants + ipex_specs overlay shipped (`config.py`, `CONFIG.md`, `config.describe()`); host/port/report-dir env-wiring lands with the ASGI server (N7/N8) |
| **N16** | **P1** | **Release process + versioning** — SemVer, `CHANGELOG.md`, `dustgym.__version__`, documented bump→tag→publish flow (replaces the old upstream-PR model) | ⬜ |
| **N17** | **P2** | **Deployment + ops doc** — `docs/deployment.md` + a server container image + env-var config + the Godot/`/render` optional-dependency toggle | ⬜ |
| **N18** | **P2** | **Reproducibility baselines** — checksummed data fixtures; golden-file regression on planner totals (energy/mass/distance); AprilTag 12.7 mm + map-channel RMSE tracked as regression baselines | ⬜ |

### O. Configurability — every constant, body, vehicle, and setting easily adjustable (expands N15)
Principle: **no physical constant, per-body parameter, vehicle/rover spec, or operational setting should
require a source edit to change** — all adjustable through one documented mechanism, with the `.py` files
remaining the default source of truth (the "everything stays .py" decision).

Current state (honest):
- ✅ **Terramechanics moduli** are runtime-adjustable — `TerramechanicsParams` (JSON-serializable;
  `from_constants` / `from_json` / `to_json` / `scm_oracle`; solver functions take the moduli as kwargs).
- ✅ **Per-body params + body selection** — `bodies.params_for_body(name)` + the `Body` registry (Moon/Mars/
  Ceres/Bennu/Phobos/Earth); add a body by adding a `Body`.
- ✅ **The 73 `constants.py` values, 22 `ipex_specs.py` vehicle specs, and the planner `[CALIB]` knobs**
  are now overridable through the env/file overlay (below) — the `.py` files stay the provenance-tagged
  defaults and derived values recompute from the overridden base.
- ✅ **Env-var / config-file overlay** — `terrain_authority/config.py` (`DUSTGYM_<KEY>` env + `DUSTGYM_CONFIG`
  TOML, env wins), applied at the end of `constants.py` / `ipex_specs.py`; reference in `CONFIG.md`.

Requirements:
| ID | P | Requirement | Status |
|---|---|---|---|
| O1 | P1 | **Config overlay** — one mechanism (`DUSTGYM_CONFIG=<file.toml>` + `DUSTGYM_<KEY>` env vars) loaded at startup that overrides the `.py` defaults for constants / vehicle specs / planner knobs / body selection; the `.py` stays the default source. Derived values (e.g. `ipex_specs.J_PER_M`) must recompute from the overridden base (so the overlay applies before import-time derivation, or derivations become lazy). | ✅ `config.py`: `DUSTGYM_<KEY>` env + `DUSTGYM_CONFIG` TOML; overlay applied then derived recompute |
| O2 | P1 | **Config reference (`CONFIG.md`)** — every adjustable key listed with its default, units, `[FIXED]/[CALIB]/[UNKNOWN]` tag, and the override name. | ✅ `CONFIG.md` |
| O3 | P2 | **Wired into the product** — `dustgym-serve --config <file>` + the env overlay honored by the planner/server and the envs. | 🟡 env/TOML overlay honored at import (server/planner/envs read the overridden constants); the `--config` flag is the remaining bit |
| O4 | P2 | **Per-vehicle config** — a vehicle/rover spec object (not global `ipex_specs` singletons) so different rovers carry different specs (ties to MV6 heterogeneous fleet). | ⬜ |
| O5 | P2 | **Settings discoverability** — `dustgym.config.describe()` (or similar) dumps the active config (every key, value, source: default/env/file) so the running configuration is inspectable. | ✅ `config.describe()` |

Sequencing: O1/O2 slot into build-sequence Phase 2 alongside N15 (externalized config); O4 lands with MV6.

## 7. KPIs
Benchmark: # authored challenges/missions; agent score vs baseline; train→held-out generalization gap.
Physics: mass drift (≤1e-9); sinkage RMS vs oracle (≤20%, post FIX-1/2). Autonomy: per-skill success;
pad/berm H-RMSE; energy/time/slip per task. Maps: load any PGDA polar tile; region-select latency.
App: load→place→execute→score round-trip time; reproducibility (seed→identical run).

## 8. Plan (by deliverable, in build order)

Each stage names its **Deliverable** (what ships), **Files** (touched / NEW, so the blast radius is
explicit), **Adds** (what is new), and **Tests** (what verifies it). "Shipped" is the record of done
stages; "Forward plan" is the live work. Multivehicle is deferred until explicitly requested.

> **Historical note:** the Shipped table below records the build history; the per-stage `PR #N` tags and
> the "tests at the time" counts are **historical waypoints from before the single-repo consolidation** (when
> the core was upstreamed to a separate fork). They are not the current state. Current state: one repo, 296
> tests. This history will migrate to `CHANGELOG.md` (N16).

### 8.0 Optimized build sequence (the authoritative ordering)

All outstanding work, sequenced so each phase **unblocks or de-risks** the next. The **critical path** to a
production, multi-vehicle planner is marked ★; the science track runs in parallel. Each item links its area
IDs (above) and the source review (`docs/architecture_review.md`, `docs/autonomous_planning_review.md`). The
Shipped/Forward backlog below is the detail this sequence orders.

**Phase 0 — Foundation (now; cheap; guards everything). ★**
- ★ **N9 CI gate** + **N11 quality config** (`ci.yml`: ruff + pytest 3.10-3.12 matrix + strict env_checker;
  commit `[tool.ruff]`/`[tool.mypy]`/`[tool.pytest]`, `py.typed`, pre-commit). Do first: every later phase
  regression-guards through it, and it is the cheapest production win.
- **AL2 fix** — guard the infeasible-precedence cliff (precheck the DAG; fail loud, not a silent 0-trip
  "success"). **AL1 fix** — emit a warning/flag when `auto` degrades past the exact caps. Real correctness bugs.

**Phase 1 — Package boundary (the one real structural "before-production" fix). ★**
- ★ **Restructure step 1+2 + P11e** — make `planet_browser` a real package (`__init__.py`, relative imports),
  **delete the dead `_ROVERSIM` resolver + the sys.path hacks**, fix sample-data paths, add `planet_browser` to
  the wheel + a `dustgym-serve` console entry point (**N13**). Completes the roversim purge.
- *Why now:* the ASGI server (Phase 2) and multi-vehicle (Phase 4) both live in `planet_browser`; a production
  server on a non-package that self-mutates `sys.path` and probes a dead `roversim/` sibling fights you the
  whole way. ~1 focused pass (see §15).

**Phase 2 — Operational shell (production hardening). ★**
- **N14** runtime invariants + input validation · **N10** structured logging + observability · **N15**
  externalized config · **N12** dependency hygiene (lockfile + ceilings).
- ★ **N7/N8 — ASGI server** (FastAPI/uvicorn): Pydantic request/response models (the API contract + input
  limits), auth on mutating routes, CORS, thread-safe OO-matplotlib report generation, `reports/` TTL. Needs
  Phase 1. The ASGI move resolves the DoS, thread-safety, observability, and contract findings at once.

**Phase 3 — Planning correctness + expressiveness (the autonomous-planning gaps).**
- **J4 / AL4-AL5** — goal-level Mission grammar (build-to-spec) + non-square footprints
  (disk/rect/corridor/polygon) + stop silently dropping `budget`/`priority`/`keepout`; wire
  `structures.decompose` as the goal→orders front-end (unifies the Challenge + Mission schemas).
- **I11 / AL6** — as-built acceptance (flatness-RMSE / berm-profile / repose) on the conserved authority; gate
  the **whole footprint** on the **real DEM**, not the center cell on a flat mantle.
- **M11** — the lat/lon→DEM-cell projection from the globe pick; multi-site (**E2**).
- *Why now:* makes single-vehicle planning trustworthy + expressive, and goals/acceptance are prerequisites for
  multi-vehicle (you allocate goals; you verify builds).

**Phase 4 — Multi-vehicle (headline feature; needs Phases 1-3). ★**
- ★ **MV1 fleet API → MV2 allocation → MV7 2-rover EXACT baseline** (validate learned-vs-exact before any
  claim) **→ MV3 deconfliction → MV4 shared-resource scheduling (fixes K8) → MV5 coordinated replan → MV6
  heterogeneous fleet.** Allocation is the only genuinely-new algorithm; per-rover sequencing reuses the
  existing exact-capped pipeline.

**Parallel science track (mostly independent; partly host-gated) — runs alongside Phases 2-4.**
- **P6** map-channel reward + a CI regression gate (Hapke<Lambert, real-DEM block RMSE) + tiny committed real
  render fixtures; **F3 render throughput** (the keystone unblock for camera-in-the-loop). **P7** Chrono live
  producer / SCM oracle (host-gated). **Autonomy: AL7 fault handling** + perception-in-loop + in-loop terrain
  mutation (prerequisites for fleet FDIR / MV5).

**Phase 5 — Release + ops.**
- **N16** CHANGELOG + SemVer + release flow · **N17** deployment doc + container · **N18** reproducibility
  baselines (golden-file planner totals; AprilTag/map-channel) · **M10** mission persistence.

**History note:** a one-time `git filter-repo` history rewrite WAS performed (owner-directed) to scrub
AI-assistant co-author trailers from all commit messages; `main` was force-pushed and the stale merged
branches deleted. This was a deliberate, backed-up, solo-repo exception to the earlier "no rewrite" stance
(§15), justified by the explicit scrub request and the absence of open PRs / external clones.

> **Critical path (★):** N9 CI → package-ify `planet_browser` → ASGI server → multi-vehicle. Everything else
> (the AL correctness fixes, the operational-shell sub-items, the J4/I11 expressiveness, the science track, the
> src/ hygiene) rides in parallel or as incremental hygiene the test suite protects.

### Shipped
| Stage | Deliverable | Key files | Tests at the time |
|---|---|---|---|
| S0 | physics + closed loop + RL substrate | `column_state`/`terramechanics`/`slip`/`drive`/`rover_env` (PR #1 merged) | 58→87 pytest + 19/19 legacy |
| S1 | challenge platform | `challenge.py`/`terrain_target_env.py`/`challenge_runner.py` (PR #3) | +27 |
| S2 | construction skills + the "greedy solves it" finding | `skill_env.py` | 89 |
| S3 | scheduler + grounded IPEx energy + per-cell deposit | `scheduler_env.py`, `ipex_specs.py`, `column_state.deposit_field`/`fill_toward` (FIX-4, PR #4); beam 24 vs greedy 28 / PPO 27 | 114 |
| S4 | multi-planet bodies + `dustgym` suite | `bodies.py`/`registration.py`; bodies sysrev | 124 |
| S5 | **product layer: planner + report + browser** | `planet_browser/mission_planner.py` (balance + optimize + 3pp report), `index.html` (Cesium browser + live estimate), `gen_bodies_json.py`→`bodies.json`; sinter = conserved primitive + WorkSite seam + planner order, **GATED OFF** | gate + deposit 9/9 |
| S6 | **config consolidation (single source = .py)** | planner imports `terrain_authority`; one `SINTER_ENABLED` in `constants.py`; `[CALIB]` knobs in `ipex_specs.py`; browser reads `bodies.json` `_ipex` | 168 pytest |
| S7 | **browser → plan → report round-trip (P1)** | `planet_browser/index.html` (build QUEUE add/list/reorder/delete + "Plan" → opens the PDF + the missing `bodies.json` fetch), NEW `server.py` (stdlib http; serves front-end + `POST /plan` + `/reports/`), `mission_planner.mission_from_dict` + `run(stem=)` | NEW `test_mission_planner.py` 13/13 (incl. real-socket `/plan` + PDF fetch); ruff-F clean; live curl drive (58 KB PDF, sinter→400) |
| S8 | **drum-mass sensing + offload autonomy** (ICE-RASSOR, areas K6/K7) | `rassor_mass_model.py` (`DrumSensor` + toggleable seeded noise, `freespin_drum_current_a`, `should_offload`; NTRS 20210022781), sinter primitive + gate, `worksite_env`/`scheduler_env` optional `drum_sensor` (landed pre-consolidation) | `test_rassor_mass_model` + `test_drum_sensing`; 190 pytest (historical) |
| S9 | **product/UI overhaul + release prep** | Earth render fix (Esri WebMercator), single-sidebar redesign + professional palette, imagery **layer selector** (Mars MOLA shaded-relief), terramechanics **ⓘ** info button (per-body), responsive layout, **Haworth work-area DEM inset** (`server.py /dem` + auto-show on Moon); `AGENTS.md` | Playwright-snapshot verified; 190 + 15 (historical) |
| S10 | **author by structure (P2)** | NEW `planet_browser/structures.py` (8 taxonomy templates → **volume-balanced** cut/fill orders) + `server.py POST /structure` + `index.html` structure picker → build queue | NEW `test_structures.py` 8/8 + 2 `/structure` endpoint tests (TDD red→green); ruff-F clean; UI snapshot-verified |

### Forward plan

**P1 — Browser → plan → report round-trip — ✅ SHIPPED (S7), see above.** Built TDD-first (13 tests red→green), lint-clean (ruff F), driven live over HTTP (served front-end + `POST /plan` returns a real 58 KB mission-control PDF; sinter order refused 400; inline JS `node --check` clean). The build-order queue (add/list/reorder/delete + a "from pad/berm" convenience) posts to a stdlib `server.py` that calls `mission_planner` and returns the report URL the browser opens. The dangling `PHY` (never-fetched `bodies.json`) was fixed in the same pass. **Visual note:** the Cesium globe render needs a real GPU browser (validated at HTTP/DOM/JS layers here, not pixels).

**P2 — Author by structure — ✅ SHIPPED (S10).** `structures.py` has 8 taxonomy templates (Landing Pad,
Solar Pad, Habitat Foundation, Haul Road, Blast Berm, Borrow Pit, Crater Fill, Trench) that decompose a
placed structure into **volume-balanced** cut/fill orders (density-invariant: a fill consumes exactly its
paired cut). Exposed via `server.py POST /structure` and an `index.html` structure picker that adds the
orders to the build queue. TDD-first (8 `structures.py` + 2 endpoint tests red→green), ruff-F clean,
UI-snapshot verified. NEXT forward: P4 (map generality) / P5 (execute+watch) on the product side; P6 (map
channel) / P7 (Chrono) on the science side; P3 (sinter un-gate) when its numbers are grounded.

**P3 — Ground sinter, then un-gate [P2]. 🟡 GROUNDED (2026-06-03); gate intentionally kept off (sourced physics).** Constants are now LITERATURE-SOURCED (no `[CALIB]`): `RHO_SINTERED`=2300 (microwave-sintered 2.23–2.34 g/cm³, Lin et al. J. Eur. Ceram. Soc. 2024; SPS to 2.90), `SINTER_ENERGY_J_PER_KG`=0.92 MJ/kg is the thermodynamic floor (sensible heat: c_p 0.8–1.0 J/g/K Hemingway et al. 1973 Apollo × ΔT~1075 K to ~1100 °C sinter temp, Tsubaki et al. ACS Omega 2024), + a documented `SINTER_PROCESS_ENERGY_J_PER_KG_MEASURED`=69 MJ/kg (measured microwave). **The research RESOLVED the gate the other way:** sinter stays OFF for the IPEx baseline for *sourced* reasons — IPEx is a drum excavator with **no sinter tool**, and even the floor is ~0.2× the pack/kg while the measured process energy is **~14–20× the whole 4.79 MJ pack per kilogram** (energetically incompatible). Un-gate only for a deliberate sinter-EQUIPPED, externally-powered variant. `test_sinter_constants_are_sourced` (provenance: no `[CALIB]`, refs present).
- **Deliverable:** flipping `SINTER_ENABLED=True` becomes legitimate; sinter is a usable action and a report leg.
- **Files:** `constants.py` (`RHO_SINTERED`, `SINTER_ENERGY_J_PER_KG` re-sourced + citation), `ipex_specs.py` (`SINTER_HEAD_POWER_W` sourced), flip `SINTER_ENABLED`; re-add the sinter order to the demo; regen `bodies.json`. (`worksite` + planner already wired.)
- **Adds:** a sourced sinter energy/density model + provenance; the un-gate.
- **Tests:** the gate test flips to assert the enabled path fuses + conserves mass + the report carries a sinter leg; a provenance test that the sinter constants are sourced (no `[CALIB]`).

**P4 — Map generality + scale [P1]. ✅ DONE (TDD) 2026-06-03 (deps installed: `pyproj`; real LOLA `ldem_4`).**
- **Deliverable:** ✅ tile/stream km-scale maps without holding them in RAM; ✅ load non-polar / other DEM products (reproject to the local metric grid).
- **Files:** ✅ `mission_planner.py` streaming — `dem_grid_info` (metadata only), `read_dem_window(r0,c0,h,w[,bundle_dir])` (seek-per-row, exactly h·w·4 bytes I/O, full 2000² never materialised), `flattest_anchor_streamed` (tile-by-tile scan w/ halo). ✅ NEW `dem_import.py` — `reproject_cylindrical` (pyproj: body geographic → local azimuthal-equidistant metres, bilinear resample), `load_cylindrical_fixture`, `ingest_to_bundle` (writes the sim `metadata.json`+`heightmap.rf32` so the readers consume it). ✅ `requirements.txt` (numpy/matplotlib/scipy/pyproj). ✅ `fixtures/ldem4_equator_*` (tiny REAL equatorial LOLA `ldem_4` patch, 0–20°N/0–20°E, 6266 m relief). **Follow-on:** a browser body/map picker + server reading windows end-to-end (still caches the full DEM); GeoTIFF/COG `/vsicurl` ingest.
- **Adds:** ✅ windowed DEM reader + streamed flat-site finder + non-polar (cylindrical) reprojection ingest.
- **Tests:** ✅ `test_*` (5): `dem_grid_info` no-load; `read_dem_window` bit-exact vs full-load crop (incl. far-corner random access); tracemalloc memory ceiling; `flattest_anchor_streamed` buildable site under a memory ceiling; **non-polar ingest relief round-trips** (real LOLA cylindrical → local metric, ≥95% relief at 2 km sampling, rf32 bundle round-trip). Verified: streamed flattest = **exact same site** as in-RAM (4115, 6915 @ 0.39°) at **9.4 MB peak vs 160 MB** (17×); ingested LOLA equatorial map is planner-usable (slope/flattest/route on a non-polar real DEM, was Haworth-only).

**P5 — Execute + watch [P2]. ✅ DONE (TDD) 2026-06-03.**
- **Deliverable:** after planning, animate the route top-down with a telemetry overlay (battery / phase / position / mass). ✅ (live terrain *mutation* during playback deferred).
- **Files:** `mission_planner.py` (`build_timeline` + rover positions added to the sim `tl`), `server.py` (`/plan` returns `timeline`), `index.html` (`#execview` canvas + ▶ Execute button + telemetry HUD + `runExecution`/`execDraw`/`execExtent`).
- **Adds:** the animatable timeline + a top-down execution canvas + telemetry HUD.
- **Tests:** `test_build_timeline_is_animatable` (contiguous monotonic time, starts/ends at charger, battery sawtooth, mass monotonic, recharges present), `test_build_timeline_routes_with_dem`, `test_plan_endpoint_returns_animatable_timeline`. Verified live: timeline 70.3 h / 13 frames / 2 recharges; execution canvas headless-rendered (rover at the dig site, route polylines, cut/fill footprints).

**P6 — LAC map channel [P1]. 🟢 SCORER + ONBOARD-STEREO PRODUCER BUILT (TDD) 2026-06-04 (Godot render track live on the GPU).**
- **Deliverable:** the §10 perceived-map-vs-truth objective: an observed/reconstructed elevation map scored against the true terrain at time t (the LAC-style mapping metric, and the keystone RL reward / nav costmap). The `map_rmse_m` / `map_cell_pass_frac` / `rock_f1` slots are no longer producerless — **the scorer is built**; the always-`None` in the *synthetic* harness now reflects only the missing live producer.
- **Files:** ✅ NEW `scripts/ros2_bridge/score_map.py` (`map_height_metrics` rmse + cell-pass-frac, `rock_f1` greedy-match, `score_map`, `attach_map_metrics`); ✅ `eval_harness.py` wired (`run_map` + `--map-truth/--map-observed/--map-tol-m` CLI, `.npy`/bundle-dir loader) as the 2nd channel beside pose (pose zeroed, never summed); `eval_schema.py` slots reused unchanged. ✅ NEW `scripts/ros2_bridge/obs_map_producer.py` (the ONBOARD observed-map producer): rectifies the rover front-stereo pair with the exact known camera extrinsics (`cv2.stereoRectify`), runs SGBM, back-projects to the authority world frame, and grids to an observed heightfield + valid_mask that feeds `score_map`. The Godot render track is LIVE on the RTX 3090 (2026-06-04), so the producer runs on real renders, not a supplied array. **Honest finding:** passive rover stereo at the ~0.15 m grazing eye-height has ~0.3 m (1σ) height precision; the rover-scale scenes' ~0.05 m relief is below that floor, so it recovers the ground plane + coverage (grows 2.6→16.4% over an 8-station drive) but not the cm micro-relief that governs trafficability. Validation figures + finding in `validation/map_channel/`.
- **Adds:** ✅ the map scorer + the second eval channel + a producer-independent real-DEM ingress + ✅ the ONBOARD rover-stereo observed-map producer + ✅ the GROUND COLMAP producer now SCORED vs truth (2026-06-04, pycolmap, no Docker): `scripts/colmap/render_corpus.py` renders a known-pose multi-view corpus, `colmap_map_channel.py` runs SfM and Umeyama-aligns the recovered camera centers to the known poses (align RMSE 6 mm) to put the sparse cloud in world frame, then `score_map` vs truth → **18/18 images, 0.48 px reproj, map RMSE 0.04 m, 97% cell-pass** (sparse SfM, ~3% coverage; dense MVS would fill it). **BRDF A/B** (`make_colmap_ab.py`): the physically-correct Hapke gives COLMAP ~33% fewer 3-D points and ~30% less coverage than the idealized Lambert baseline, at higher reprojection error — the non-Lambertian regolith costing multi-view correspondences, exactly as on real lunar imagery. Onboard (cheap, real-time, 0.32 m) and ground COLMAP (offline, 0.04 m) are complementary tiers; the sim grades both against the same conserved truth. + ✅ **GRAZING height-sweep** (`make_height_sweep.py`, 2026-06-04): the ground tier COLLAPSES toward the rover's grazing eye-level — 18/18 imgs register at elevated/mid height, 12/18 at 1.0 m, only **2/18 at 0.5 m** (near-horizontal views of a near-flat surface share too few features; accuracy stays ~4 cm where it reconstructs, registration+coverage fall off). + ✅ **UNCERTAINTY layer** (`obs_map_producer.grid_to_heightfield_uncertainty` + `dig_ready_mask`): per-cell height σ (std-error of the mean, falls with views; single-view = 0.30 m prior) → a dig-ready gate (green ready / red observe-more / grey unobserved). Mapped in `docs/world_model.md` (the 5-layer world model: Geometry/Material/Physics/Task/Uncertainty, conserved-dynamics + learned-perception hybrid). ⬜ dense MVS (CUDA-gated); spatially-varying Material fields; learned perception WM. Figures in `validation/map_channel/`.
- **Tests:** ✅ `test_score_map.py` (6) — identity perfect; a REAL coarsened reconstruction scores worse monotonically (block 2<4<8); tolerance/valid-mask move the metrics correctly; rock-F1 detection identities; `attach`/harness `run_map` emit non-null map metrics with the pose channel preserved. Live CLI verified on the real Haworth bundle: identity rmse 0/pass 1.0; block-4 reconstruction rmse 1.70 m/pass 0.427. ✅ NEW `test_obs_map_producer.py` (6): 5 pass — quaternion identity + real-Haworth grid round-trip + median/mask + out-of-bounds drop + identity-through-scorer; 1 integration test (real render egress) skips when no egress is present.

**P7 — Chrono live producer + oracle calibration [P2]. 🟡 RIGID-BODY PRODUCER DONE (2026-06-03); SCM oracle still source-gated.** PyChrono **10.0.0** installed via a bootstrapped **micromamba** (conda-forge, `/tmp/chrono-env`) — core + fea + robot. `pychrono.vehicle` (where SCM deformable terrain lives) is **NOT in the conda-forge build** → the soil-sinkage oracle needs a source build of Chrono-with-vehicle.
- **Deliverable:** replace the one-shot Chrono STUB with a live producer ✅ (rigid-body half); SCM oracle calibration of the Bekker moduli (FIX-1/FIX-2) ⬜ (needs vehicle module).
- **Files:** ✅ NEW `scripts/chrono_clast_producer.py` — the §4.4 hybrid's **rigid-body authority**: a real `ChSystemSMC` multibody solve (BULLET collision, lunar gravity) that settles rigid clasts on the surface and exports rest poses for the numpy surrogate to consume. ✅ `scripts/test_chrono_clast_producer.py` (pychrono-guarded: skips on the venv, passes under chrono-env). `scripts/chrono_scm_export.py` (the SCM exporter) stays as-is — it needs `pychrono.vehicle`.
- **Adds:** ✅ a live, validated Chrono rigid-body producer. ⬜ the SCM-calibrated moduli (FIX-1/FIX-2). **FIX-1 (K_PHI) is meanwhile resolved by the literature-sourced NASA LTV lunar Bekker values** (bodies sysrev: k_c 1400 / k_phi 820000 / n 1.0 / c 170) — the SCM run would re-derive the same Bekker form.
- **Tests:** ✅ free-fall matches analytic `t=√(2h/|g|)` to **0.00–0.01%** at lunar+earth g (ratio 2.461 = √(g_e/g_m)); 5 rigid clasts **settle on the surface** under lunar gravity (rest z ≈ radius, KE 5.6 J → 0.005 J) — run-verified under chrono-env. ⬜ sinkage RMS vs SCM oracle ≤ 20% (needs vehicle module).

**P8 — Realistic, authority-validated planning (the biggest realism jump) [P0].**
- **Deliverable:** make the plan terrain-aware and physically verified instead of an abstract footprint estimate. Reads the DEM for slope-feasible siting (I6), balances by mass with bulking (I7), and **validates/costs the plan by executing it on the conserved authority** (`column_state`/`scheduler_env`) for real mass-exact energy/time/feasibility incl. soil-dependent dig (I8); routing becomes slope/slip- and hazard-aware (I10).
- **Files:** `planet_browser/mission_planner.py` (read DEM, mass+bulking balance, call the authority), `terrain_authority/dem_import` (footprint slope/roughness query), reuse `column_state`/`slip`/`scheduler_env`.
- **Adds:** terrain query + bulking balance + an authority-validation pass + slope/slip/hazard routing.
- **Tests:** infeasible-slope site is rejected; cut↔fill mass-balances WITH bulking; authority-validated energy/time matches the executed sim within tolerance; a hazard is routed around. (Realises I6/I7/I8/I10.)
- **Progress:** **I7 ✅** (bulking-correct mass balance) · **I8 ✅** (`validate_plan` executes the plan on the conserved `ColumnState`; **fill-feasibility fixed** — a fill is infeasible only when the shared drum runs dry short of the analytic plan, not when a sub-grid footprint under-covers the 0.5 m cells; genuine under-supply still flagged) · **I6 ✅ LIVE** (real-Haworth slope gate wired into `/plan` for Moon) · **M11 🟡 v0** (`flattest_anchor` + `dem_origin` anchor the order frame to the auto-selected flattest Haworth region; full lat/lon-pick projection remains). · **I10 ✅** (hazard-aware haul routing — slope costmap + 8-connected Dijkstra least-cost path; wired into the planner/report/browser; honest grid-consistent detour metric; `slip.py` leg-energy coupling deferred). **P8 COMPLETE** (I6/I7/I8 + M11 v0 + I10; only the deferred `slip.py` energy coupling and the full M11 projection remain as polish). Next: **P6** (LAC map channel — biggest science gap) or **P5** (execute+watch animation).

**P9 — Precedence + acceptance + robustness [P1].**
- **Deliverable:** dependency-ordered build sequence (I9), per-structure acceptance verification (flatness RMSE / berm profile / bearing + repose-stable fills, I11), and uncertainty bands on the plan (I12).
- **Files:** `mission_planner.py` (precedence DAG, acceptance metrics, robustness pass), `structures.py` (per-structure acceptance spec), `building_taxonomy.md` (§3 acceptance).
- **Adds:** precedence graph + acceptance scorer + confidence bands.
- **Tests:** a precedence violation is reordered/flagged; a built structure passes/fails its acceptance metric; the report carries ± bands.

**P10 — Realistic polar power + operational windows [P1]. 🟡 POWER-SOURCE MODEL DONE (TDD) 2026-06-04.**
- **Deliverable:** PSR-correct power ✅ (no sun at Haworth → lander/tower budget; sunlit → duty-limited solar; thermal derating). Full mission-clock window-gating of drive/dig/charge (K9) ⬜.
- **Files:** ✅ `mission_planner.py` — `power_regime(mission, kind=, charge_power_w=, temp_c=)` (PSR `psr_tower` = anytime/duty 1.0 vs `sunlit_solar` = duty `daylight_h/solar_day_h`, effective_charge_w), `thermal_derate(temp_c)` (cold Li-ion fraction, FIX-5 qual context); wired into `endurance().power` + the report "Power:" line. Fixes the flagged-wrong "flat 700 W solar at a PSR" (the recharge model IS the tower — correct for a PSR; calling it solar was the error). **Remaining:** simulate the duty-limited solar recharge against the mission clock (night-parking), per-site illumination (K9).
- **Tests:** ✅ 3 — PSR tower duty 1.0/anytime vs sunlit-solar duty<1/daylight-only; thermal derate cold<1 floored 0.5; endurance carries the power regime. ⬜ window-gated charge-blocked-outside-window.

**P11 — Production hardening [P0, the new headline program]. Realizes N9-N18 + M10/M11.** This is the work
that takes dustgym from pre-production to production-grade (see `docs/architecture_review.md`). Built in
phases, foundation-first:
- **P11a — CI + quality gate (P0, N9/N11/N14).** `ci.yml` (ruff + pytest matrix + strict env_checker, gated
  test markers, branch protection, publish-needs-CI); commit `[tool.ruff]`/`[tool.mypy]`/`[tool.pytest]` +
  `.pre-commit` + `py.typed`; runtime invariant guard (`conserves_mass`/`check_invariants`) + public-constructor
  validation. **Tests:** CI is green and required; an invariant guard catches an injected mass leak; an invalid
  `ColumnState` is rejected.
- **P11b — Observability + config + deps (P1, N10/N12/N15).** `logging` replacing library/server `print`;
  request + error logging + `/healthz`/`/metrics`; env-overridable config; a lockfile + version ceilings +
  `pip-audit`. **Tests:** the server logs a request id + outcome; a bad config is rejected; a fresh-venv install
  reproduces from the lock.
- **P11c — ASGI server + API hardening (P1, N7/N8/N13).** FastAPI/uvicorn with Pydantic request/response
  models (the API contract), input size/time limits, auth on mutating routes, CORS policy, thread/async-safe
  OO-matplotlib report generation, `reports/` TTL/quota; package `planet_browser` + a server entry point in the
  wheel and delete the `sys.path` hacks. **Tests:** oversized body → 413; unauthenticated mutate → 401;
  concurrent `/plan` produce uncorrupted reports; `pip install` exposes the server entry point.
- **P11d — Persistence + geodesy + release (P1/P2, M10/M11/N16/N18).** Mission save/load/version (SQLite or
  files); the lat/lon↔local-meters transform driving the plan from the globe pick; `CHANGELOG.md` + SemVer +
  `__version__`; golden-file regression on planner totals + AprilTag/map-channel baselines. **Tests:** a mission
  round-trips through save/load; lat/lon↔meters is invertible; planner totals match the golden file.
- **P11e — roversim purge (P0, mechanical).** Remove every `roversim` path, the dual-tree resolver, the
  user-facing multi-vehicle raise string, and PR references from code + docs + this PRD's history. There is
  one software.

**P12 — Closed-loop autonomy (the AutoNav model) [P1]. 🟡 STARTED (TDD) 2026-06-04.**
- **Design basis:** DS1 AutoNav (Riedel/Bhaskaran, JPL) runs sense→**estimate(+covariance)**→plan→execute→re-estimate→**replan** onboard. We already own PLAN (`mission_planner`: algorithms×objectives, precedence = AutoNav's "legal" check, model-based self-simulation), EXECUTE (`terrain_authority` `drive_step`/worksite + the conserved authority), and one real observable (drum motor-current sensing). The gaps are the **ESTIMATE** half (state + uncertainty) and the **closed-loop replan**. Loop runs in the conserved-authority sim first (AutoNav's self-simulation), real telemetry later.
- **Deliverable:** `autonomy/` layer — `belief` (estimated pose / energy SoC / drum fill / task ledger, each with 1-σ), `estimator` (recursive Kalman/Bayesian fusion; predict grows uncertainty, update_* shrinks it), `executor` (steps the authority through a leg), `controller` (plan→execute→sense→estimate→replan + fault protection).
- **Status:** ✅ **estimator + belief** — `autonomy.py` (`Belief`, `initial_belief`, `_kf_update`, `predict`, `update_drum`/`update_pose`/`update_energy`); measurements grounded in the real drum-sensor uncertainty (FDC ±2.56%) + a real conserved-authority cut. ✅ **executor + controller (the closed loop)** — `nominal_leg_energy_J` (flat plan), `execute_leg` (slip+slope-adjusted TRUE telemetry from the real DEM), `run_closed_loop` = plan→execute→estimate(predict+grow σ)→**replan/recharge against the estimate**; reserve-aware closed-loop battery management; pose σ grows by dead-reckoning, energy σ by model error. Runs in the conserved-model sim first (AutoNav self-simulation). `test_autonomy.py` (**8**): KF identities, predict grows σ, drum measurement shrinks σ + brackets truth within 2σ, pose fix shrinks σ; `execute_leg` truth ≥ nominal; loop completes + recharges + bounds SoC; true ≥ nominal + σ carried. Live: 2-trip Moon plan completes, 2 recharges + 1 replan, pose σ → 11.5 m. **Honest finding:** on dig-dominated missions slip barely moves the total (dig ≫ drive — the endurance result), so the dominant model-error to track is the drum-fill ± (estimator handles) + dig variance; slip bites on traverse-heavy plans and once it's wired into the haul (#1). ⬜ fault protection · ⬜ wire perception (P6 producer, Godot-gated) · ⬜ terrain mutation on the authority during the loop (validate_plan has the machinery).
- **Builds on:** I12 (uncertainty bands) — the estimator IS the uncertainty foundation; the conserved closed drive loop + scheduler + beam-search as the execute/model substrate; P6 map channel as the perception input. #2 (power model) folds in as "estimate energy/battery state with uncertainty and replan against it."

**Multi-vehicle (corrected 2026-06-04):** the software has **zero multi-vehicle planning** today — both the
product planner and `scheduler_env.py` are strictly one-rover / one-drum. The earlier "the scheduler shows
learned ≫ greedy in multi-vehicle" note was **overstated**: that result (`beam_search` 24 legs vs greedy 28 /
PPO 27) is **single-rover** trip-leg makespan ordering, not parallelism or conflict. Multi-vehicle is now a
designed-but-unbuilt area (MV1-MV7 below); the `vehicles=1` gate is a deliberate seam, not an architectural
dead-end. A 2-rover EXACT baseline is required before any "learned ≫ greedy multi-vehicle" claim ships.

> Dependency: **P1 (round-trip) + P2 (structure authoring) have landed** — the product *flow* is real.
> **P8 is the keystone for realism**: today the plan is an abstract footprint+TSP estimate decoupled from
> the DEM and the conserved authority, so P8 (terrain-aware + authority-validated, with bulking + slope/slip
> + hazard routing) is what makes the numbers physically true, and P9 (precedence/acceptance/robustness)
> builds on it. P10 (polar power) and P3 (sinter) are independent grounding; P11 is production hardening;
> P5 (animation) and P6/P7 (map-channel/Chrono science depth) extend the rest.

## 9. Delivery / release
**Single software, single repo (`dustgym/dustgym`).** There is no upstream fork and no cross-repo PR model
anymore; the former `roversim` history is folded in (the Tier-2 core remains CC0, John McCardle's provenance).
Delivery is a standard release flow (N16): land work on a branch → green CI (N9) → merge to `main` → bump
`pyproject.toml` version + `CHANGELOG.md` → tag `vX.Y.Z` → the publish workflow ships to PyPI. Current state:
`main` is clean, 296 tests pass locally, the package builds (`dustgym 0.1.0`); the production-grade release gate
(N9-N16) is the work this PRD version prioritizes. dustgym is **not yet on PyPI** (pre-release).

## 10. Dependencies & risks
Tier-3 forces (A5/A6) + camera RL (F3, perception) gated on euclid oracle / render throughput. Energy/
battery (K2) is a new resource model (not physics). Scale (D5/E2) + the 3D app (M) are real engineering.
Over-claim risk: keep "real physics" = conserved Tier-2 (N3); keep app shell ≠ benchmark core (§1).

## 11. Open questions
v1 Mission grammar scope (which structures first: Pad+Road+Berm?). 3D-app engine (Godot vs web viewer — the
M-section recommendation is React + three.js + FastAPI). Battery model fidelity (constant draw vs
actuator-resolved). Single-agent v1 vs multi-agent schema now. (Resolved: there is one software, one repo —
the old "where RL/app code lives" question is closed; general map ingest now supports polar + reprojected
equatorial.)

## 12. Out of scope (v1)
Flight autonomy; granular DEM at scale; tool-wear/thermal-power; camera-RL/perception track; multi-agent;
the full game UX (M7/M8). Post-v1 or research-bet.

## 13. Visual artifacts (evidence)
`docs/foss_ipex_weekend.pdf` (deck); `live_run.png` (real-terrain drive, slip); `policy_training.png` +
`ppo_training.png` (CEM/PPO learning); `challenges.png` (the 3 M1 challenges: map→target→agent);
`simcityspace_concept.png` (**site plan over the REAL LOLA Haworth DEM** — the L8 map view, concept overlay).

## 14. Status rollup
**Done/shippable (M0+M1):** conserved Tier-2 physics + closed loop + sensor render + env_checker-clean RL
env (PPO 0→100%) + the challenge platform (schema/generator/runner/leaderboard) + **real lunar-map loading
for the polar LOLA family** (the 10 km Haworth tile loads and renders). **Next layer (M2):** train the
construction skills so structures actually build. **Then (M3):** structures + planner + Mission grammar +
the battery resource model — that completes "select what/where → execute under physics+battery." **Then
(M4/M5):** map-loading generality + scale, and the 3D application (the visual program with load/select/
build/watch). Externally gated: Tier-3 forces, camera-scale perception.

**Product layer now built (the 2026-06-03 reframe, standalone in `planet_browser/`):** the **planet
browser** (CesiumJS + NASA Solar System Treks WMTS; Moon/Mars body dropdown; pan/zoom/tilt; coord-load;
build-order panel with a live mass/weight/energy estimate driven by the chosen body's gravity and
`bodies.json`), and the **sequence optimizer + mission-control report** (`mission_planner.py`:
cut-fill balancing, nearest-neighbour route order, battery-aware mid-task recharge, grounded in
`ipex_specs`; emits a 3-page PDF + markdown). Multi-planet terramechanics is sourced (`bodies.py` /
`bodies.json`, Moon + Mars first-class). Scheduling is where RL/search pays off: beam-search reaches 24
legs vs greedy 28 / PPO 27. **Sinter** is a real conserved authority primitive + WorkSite seam action
but is **GATED OFF** (`SINTER_ENABLED=False`) until its [CALIB] energy/density are IPEx-grounded.
**P1 SHIPPED (S7):** the browser now has a persistent build-order queue and a local `server.py /plan`
endpoint, so place → queue → optimize → report is one round-trip in the UI (TDD 13/13, lint-clean,
driven live). **Next:** the optimized build sequence (§8.0) — Phase 0 (CI + the AL correctness fixes), then Phase 1 (the
`planet_browser` package boundary), then the operational shell, then multi-vehicle (now scoped as the MV area,
not parked).

## 15. Restructure & rebase decision (2026-06-04)

**Restructure: YES, targeted, medium-urgency (not a teardown).** The repo is already half-packaged —
`terrain_authority` (the engine) and `dustgym` (the Gym shim) are clean, declared, installable packages with
an **acyclic** dependency tree. The one real structural defect: **`planet_browser` (the product) is not a
package** (no `__init__.py`), is **excluded from the wheel**, imports its own siblings by bare name, and reaches
the engine via `sys.path` + the dead `_ROVERSIM` resolver that still probes a non-existent `roversim/` sibling.
That blocks installing/deploying the server, so fix the package boundary **before** the ASGI server and
multi-vehicle (build-sequence Phase 1).
- **Target:** one installable `src/dustgym/` package with `engine` (was `terrain_authority`) + `planner` (was
  `planet_browser`) subpackages + a `dustgym-serve` console entry point. The ~65 `sys.path` hacks across 43
  files **evaporate** once `pip install -e .` is the workflow; tests move to a top-level `tests/`.
- **Order:** package-ify `planet_browser` + delete `_ROVERSIM` (Phase 1, blocking) → wheel + entry point →
  *then later, incremental* the `src/` rename + `tests/` move + scripts/viz hack removal (pure hygiene, the
  296-test suite protects it). Only the `planet_browser` package-ification is "before production."

**Rebase: NO.** 39 commits, **linear PR-merge history, no roversim history folded in** (a clean
re-origination — the first commits already say "dustgym"). The large tracked binaries (two 40 MB demo GIFs,
the 15 MB `.rf32` real-DEM layers) are a **deliberate, `.gitignore`-documented** deliverable policy — each
appears in exactly one commit with zero churn, so `.git` is flat, not bloated-by-rewrite. A history rewrite
reclaims ~120 MB but rewrites every SHA on the shared `dustgym/dustgym` remote, breaking every collaborator
clone and open PR. **Risk ≫ reward.** *Future option, not now:* move the two demo GIFs to Git LFS or
out-of-repo hosting **going forward** (`.gitignore` + a new commit), not a history rewrite.
