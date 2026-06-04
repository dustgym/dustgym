# Building taxonomy — planned lunar construction (foss_ipex)

**Status:** design (2026-06-02). The ontology under `rl_construction_design.md` (skills/hierarchy) and
`platform_architecture.md` (platform). Defines the *vocabulary of building*: primitive skills (verbs),
composite structures (nouns), the resource/constraint budget, the planning hierarchy, and the
build-order grammar for the target workflow:

> **load a lunar map → select what to build where → it executes with fidelity (physics + battery + time
> constraints) using RL/ML to plan.**

Each row is tagged with its status against the actual sim: ✅ in code · 🟡 partial · ⬜ to build · ⛔ gated.
**Fidelity rail:** "execute with fidelity" = conserved, slip-aware, **geometry/state-accurate Tier-2**
(mass conserved, sinkage/slip coupled), NOT Tier-3 force-accurate (drum torque/throughput = euclid oracle,
deferred). **Authority rail:** learned/planner components only *command*; the physics authority mutates
terrain, so mass conservation holds by construction.

---

## 0. The workflow this serves (and where each step stands)

| Step | What it is | Maps to | Status |
|---|---|---|---|
| **Load map** | pick a lunar surface (procedural seed or real LOLA tile) | `challenge.realize` / `dem_import` | ✅ (single tile); km-scale tiling ⬜ |
| **Select what/where** | place build orders on the map (structure + footprint + spec) | **BuildOrder / Mission grammar (§5)** — extends the M1 `Challenge` schema | 🟡 single-objective ✅; multi-structure mission ⬜ |
| **Plan** | decompose structures → tasks → skills; route mass (borrow→fill); schedule under budget | **Hierarchy (§4)**, RL/ML + symbolic | ⬜ planner not built |
| **Execute** | run skill policies on the conserved physics, under constraints | `TerrainTargetEnv` + skills (§2) | 🟡 env ✅; trained skills ⬜ (M2) |
| **Score** | mass moved, energy, time, slip, build-to-spec quality | `challenge_runner.Scorecard` + resource model (§1) | 🟡 terrain/time/slip ✅; energy/battery ⬜ |

---

## 1. Resources & constraints (the budget the planner optimizes)

| Resource | Meaning | In sim? | How measured / model |
|---|---|---|---|
| **Regolith mass** | conserved material; fill must be cut from somewhere | ✅ | `total_mass()` invariant; cut→drum→dump |
| **Energy / battery** | capacity, draw per skill, solar recharge in sun windows | ⬜ **NEW** | model: `E -= k_drive·load·dist + k_dig·mass_cut + idle`; `E += solar(sun_elev)·dt` when lit; capacity cap. Partly derivable now (∫load·dist); full battery (capacity + recharge) is an addable resource subsystem. |
| **Time / mission clock** | step budget; couples to sun window | ✅ | env steps; `max_time_steps` |
| **Slip-risk / entrapment** | path-dependent stall hazard | ✅ | `slip.slip_sinkage_equilibrium`; slip events |
| **Tool / drum wear** | actuator degradation | ⬜ | not modeled — flag, do not score |
| **Thermal / PSR exposure** | shadow/cold-trap dwell limits | 🟡 | sun geometry ✅; thermal model ⬜ |

> Battery is the key *missing* constraint for "battery power." It is a clean resource layer to add (a
> scalar `E` with draw/recharge tied to the already-known load, distance, dig-mass, and sun elevation),
> not a physics change. Until then, energy is approximated by `∫ load·distance` + dig-mass.

---

## 2. Primitive skills (the verbs) — reusable, RL-learned, physics-executed

Each: observation / action / termination / reward / success — and its mapping to existing code.

| Skill | Action | Terminate when | Reward | Success metric | Sim mapping | Status |
|---|---|---|---|---|---|---|
| **TraverseTo(x,y)** | twist | at target / stuck | progress − slip − energy | reached, low slip | `step_pose`+`drive_step` | ✅ built, trainable |
| **FollowPath(path)** | twist | path end | −cross-track − slip | path tracked | drive loop | ✅ |
| **RecoverFromEntrapment** | twist (back-off/reverse) | slip<thr | regained mobility | mobility restored | slip model | ✅ physics |
| **Excavate(area,depth)** | drive+drum-cut | drum full / depth met | mass-cut toward plan − spill | cut mass vs target | `drum_pass`/`cut_to_inventory` | 🟡 Tier-2 (force=Tier-3 ⛔) |
| **Haul(src,dst)** | twist (laden) | at dst | progress − slip(laden) | payload delivered | `drum_inventory` + drive | ✅ mechanism |
| **Dump(loc,vol)** | drive+drum-release | inventory empty/vol | placed-mass accuracy | deposited vol | `dump_from_inventory` | ✅ |
| **Grade(area,slope)** | wheel passes | H-err<tol | −‖H−H_target‖, smoothness | RMSE, slope | `four_wheel_pass`+sandpile | 🟡 physics ✅, skill ⬜ |
| **Compact(area)** | wheel passes | density≥target | density gain | bearing/density | `four_wheel_pass` (TREAD/BERM) | ✅ physics |
| **BermBuild(polyline,h)** | compose cut→haul→dump→grade | berm-to-spec | −‖H−H_target‖ over poly | ridge height/profile | composition | ⬜ skill |
| **FillHole(area)** | compose | flat to tol | −‖H−H_target‖ | filled to grade | composition | ⬜ skill |

(Primitives' *physics* all exist in the authority; what's learned per skill is the **control policy**.)

---

## 3. Composite structures (the nouns) — planned, decomposed into skills

A structure = a target terrain spec (`H_target` / corridor / ring) + a skill plan + an acceptance metric.
"Build a berm" becomes a **planning** problem over the §2 vocabulary, not a new learned policy.

| Structure | Footprint | Target spec | Skill decomposition | Acceptance | Status |
|---|---|---|---|---|---|
| **LandingPad** | disk/rect | flat to ±tol + compacted | Excavate(high)→Haul→Dump(low)→Grade→Compact | flatness RMSE, bearing | ⬜ (flatten skill = M2) |
| **HaulRoad** | corridor A→B | graded, slope ≤ max, compacted | FollowPath + Grade + Compact along corridor | slope, traversability | ⬜ |
| **BlastWall / Berm** | ring/line around asset | ridge height H | BermBuild (cut borrow → dump ring → grade) | profile vs spec | ⬜ |
| **SolarFarmPad** | rect on sun-facing slope | flat, low obstruction, fixed tilt | Grade + boulder-clear + Compact | flatness, sun exposure | ⬜ |
| **HabitatFoundation** | rect | flat + compacted (+ trench) | Excavate→Grade→Compact (+FillHole) | flatness, bearing | ⬜ |
| **BorrowPit** | rect | excavated to depth (mass *source*) | Excavate→Haul-out | mass yield, stability | ⬜ |
| **CraterFill** | crater | filled to grade | Haul borrow → Dump → Grade | grade RMSE | ⬜ |

> Mass routing is explicit: BorrowPit/Excavate **produce** mass; Berm/Fill/Pad **consume** it. The task
> planner pairs sources↔sinks so the conserved cut/fill loop closes across structures.

**Implemented (P2, 2026-06-03):** these structures are generated by `planet_browser/structures.py`
(`decompose(name, x, y, **params)` → volume-balanced cut/fill order dicts; a fill consumes exactly its
paired cut, density-invariant) and exposed via `server.py POST /structure` + the browser structure picker.
Eight templates: Landing Pad, Solar Pad, Habitat Foundation, Haul Road, Blast Berm, Borrow Pit, Crater
Fill, Trench. TDD: `planet_browser/test_structures.py`.

---

## 4. Planning hierarchy (symbolic vs learned vs model-based)

```
Mission Planner   build orders -> ordered structures under a global budget   [SYMBOLIC: search/rules]
  -> Task Planner  structure -> regions + skill sequence + source/sink mass routing  [SYMBOLIC/optimization]
    -> Skill Selector  pick next skill + parameters                          [LEARNED options-policy / scripted]
      -> Skill Policy  the primitive controllers (§2)                        [LEARNED RL]
        -> Low-Level Controller  twist+drum -> conserved physics             [drive_step / drum]
```
- **Symbolic** top (mission/task) — interpretable plans, resource feasibility, mass routing.
- **Learned (RL)** for the skills (hard slip-aware control) and, later, the skill selector.
- **Model-based (world model)** for *planning under uncertainty*: a JEPA/RSSM latent model imagines
  "cut-here / dump-there → predicted terrain + slip + energy" so the task planner can search build
  sequences cheaply, and for sample-efficiency on the expensive perception track. Physics stays the
  ground-truth executor; the world model is the planner's imagination, not the simulator.

---

## 5. Build-order grammar (the "select what/where" artifact)

Extends the M1 `Challenge` schema to multi-structure missions:

```jsonc
BuildOrder { structure: "LandingPad"|"HaulRoad"|... , footprint: polygon|disk|corridor,
             params: {target_height?, slope_max?, ridge_height?, depth?, tolerance_m},
             priority: int, keepout: [poly] }
Mission    { map: MapSpec(seed|lola), orders: [BuildOrder...],
             budget: {energy, time_steps, max_slip_events},
             scoring: {w_quality, w_energy, w_time, w_slip} }
```
Selecting on the map = appending a `BuildOrder` (type + footprint + spec). A `Mission` (map + ordered
orders + budget) is what the planner consumes and the runner scores — the authorable, reproducible unit
(deterministic from the map seed). This is the multi-structure generalization of M1's single `Challenge`.

---

## 6. Status rollup & build path

- **Have:** conserved Tier-2 physics + slip; single-objective challenge authoring + goal-conditioned env
  + scored runner (M1); a trainable RL env; the skills' *physics*.
- **Next (M2):** train the primitive skills (Grade/Excavate/Haul/Dump/BermBuild) so structures become
  *plannable* — start with `flatten_pad` (LandingPad core).
- **Then:** the **planner** (structure→skills + mass routing + scheduling) and the **BuildOrder/Mission
  grammar (§5)**; the **energy/battery resource model (§1)** as a discrete add; multi-agent; the game shell.
- **Gated:** Tier-3 force fidelity (euclid oracle); camera/perception (render throughput).

**Bottom line for the target UX:** "load map → select what/where → execute under physics+battery" =
M1 authoring (done) + the §5 Mission grammar (small add) + M2 trained skills + a planner + the §1 battery
model. The physics-fidelity executor and the scoring already exist; the missing pieces are the learned
skills, the planner, and the battery resource layer — all reachable on the conserved Tier-2 authority.

## 7. Terrestrial-technique gap analysis → lunar additions (2026-06-03 sysrev)

A systematic gap-check of terrestrial earthmoving / heavy-civil against §2–§3 (web-sourced; see refs).
Lunar physics that drives every rating: no liquid water (→ no concrete/asphalt/wet compaction/Proctor),
hard vacuum + solar/microwave (→ **sintering** replaces hydraulic binders), 1/6 g (lower breakout *and*
reaction-mass/traction → digging is reaction-limited; pile repose + sinkage shift), no rain/wind/frost
(→ drainage/culverts/crowning N/A), abrasive electrostatic regolith, plume-ejecta + PSR drivers.

**TOP additions (genuinely new, lunar-justified, priority order):**
1. **Sinter/Melt** (NEW primitive) — fuse regolith into a hard pad/road/wall via solar/microwave/laser.
   *The* lunar substitute for concrete/asphalt; underpins landing pads, roads, dust fixation, cast walls.
   **Biggest single gap.** (MMPACT/MASON, microwave/laser-melt paving.)
2. **Force-controlled digging + Sensing-while-digging** (NEW control+perception layer) — admittance dig
   control + proprioceptive geotech estimation. Directly fixes the review's "terramechanics decorative /
   Bekker-slip moduli unread" finding; prerequisite for credible cut forces under variable regolith at 1/6 g.
3. **Cut-fill balancing + Spoil/transport allocation** (NEW planning layer) — site-level mass-routing to
   minimize haul energy. Exactly CraterGrader's optimization-based transport planner; the "brain" the
   map-channel reward is missing. (Partly = the mission_planner sequencer.)
4. **Lift-based compacted fill** + a *method* param on **Compact** (vibratory) — build berms/pads/footings
   in controlled, individually-compacted layers (strength-aware), dry. Makes BuildBerm/FillHole lift-aware.
5. **Trenching + Backfill** (primitives) and **Trenched-utility / Foundation-footing** (composites) —
   vertical-walled cuts, constrained fill around emplaced objects; buried power/data/thermal + habitat footings.
6. **Ripping/Scarifying** + **Rock-breaking / Boulder-clearing** + **Clearing** — pre-loosen the dense
   layer/duricrust and fragment/remove boulders so the drum-bucket can engage; the true "step one" of site prep.
7. **Stockpiling** (managed reusable pile vs terminal Dump) + **Benching/terracing** + **Slope-stability
   monitoring** — staged material handling + stable slope build/observe.
8. **As-built verification + Layout/staking** (survey/verification layer) — register design→terrain and
   certify finished tolerance. The AprilTag pose-vs-truth demo is the seed; extend from localization to surface-spec.

**Param-variants (not new verbs):** Screed/fine-grade (Grade precision mode), Dozing/Scraping (Haul
blade/spread modes), Slope-shaping (Grade-to-slope), Over-excavation (Excavate+FillHole).

**Explicitly SKIP on the Moon (honest N/A):** crowning, drainage, culverts (no runoff); moisture
conditioning / Proctor (no water); sheepsfoot/padfoot kneading (no plastic clay); dynamic drop-weight
compaction (1/6 g + lift cost); grubbing (no biota); wet binders — Portland concrete, asphalt (→ sintering
+ dry in-situ binders: sulfur/geopolymer/biopolymer).

Refs: CraterGrader (arXiv 2311.01697); microwave sintering landing pads (NTRS 20205010871); MMPACT/MSCC
(NTRS 20230014145); laser-melt paving (PMC10570301); autonomous construction with in-situ boulders
(Frontiers 2024); NASA MASON; material classification via proprioceptive force (Autom. in Constr. 2020);
plume/ISRU Metzger (arXiv 2104.06248). Full per-item HIGH/MED/LOW/N-A analysis in the session research.

---

## 8. NASA ICE-RASSOR grounding: action vocabulary + drum-mass inference (2026-06-03)

NASA KSC's flagship excavator RASSOR and its autonomy project **ICE-RASSOR** (Intelligent Capabilities
Enhanced RASSOR, IR&TD) anchor our verbs to a real flight-lineage system (public-domain U.S. Government).

**Action vocabulary (NTRS 20210021455):** the ICE-RASSOR autonomy loop is **excavate → scoop → haul →
dump → process**. Mapping to §2: *excavate* + *scoop* = our **Excavate** (bucket-drum dig + ingest);
*haul* / *dump* are ours; **process** is a NEW sink we lacked — deliver the load to an **ISRU processing
plant** (resource extraction), distinct from a construction *fill* or a terminal *Dump* to spoil. The
high-level autonomy trigger the paper states: "navigate to a processing plant to offload the collected
regolith when the drums are full" -- i.e. the planner must know drum fill to sequence haul-to-process.

**Drum-mass inference (NTRS 20210022781) -> `terrain_authority/rassor_mass_model.py`.** The 2020/2021
RASSOR had **no load cell**: drum regolith mass was inferred from existing motor telemetry (arm/drum
position, velocity, current, voltage, pose), so no new hardware and no new failure modes. Three validated
linear structures: **Arm-Raise** (mass ∝ integrated arm-motor power during a raise; R² 0.996/0.974 --
this is gravity work, which we now ground from first principles, gravity-aware via `bodies.py`),
**Free-spinning Drum Current** (at constant drum speed, mass ∝ steady non-digging average drum current;
R² 0.989/0.985; best hardware accuracy, **MPE 7.40% over range, 2.56% when > half full**; NN-augmented
to remove velocity dependence and flight-integrated), and **Excavation Drum Current** (mass/cycle ∝
aggregated dig current; R² 0.76). We take the **structure + validated quality + the realistic drum-fill
KNOWLEDGE UNCERTAINTY** (`drum_mass_uncertainty_frac`: 2.56% when > 20 kg else 7.40%), so the autonomy
layer plans against imperfect fill knowledge instead of exact mass; the un-published, RASSOR/1-g-specific
fit coefficients are NOT fabricated (`LinearMassModel.fit` calibrates from data). Adds the missing
**arm-raise lift energy** (`arm_raise_lift_energy_j`, m·g·h/η) to the energy budget -- small vs dig but
gravity-scaling, so it matters across bodies. [CALIB]: arm lift height/efficiency.

**Sensing observable + autonomy (WIRED, on the conserved authority):** `freespin_drum_current_a(mass)`
emits the drum-motor-current OBSERVABLE from the conserved true drum mass (the forward FDC structure,
`I = baseline + slope·mass`, [CALIB] from Fig 7/8); `LinearMassModel.fit` calibrates the inverse on our
OWN conserved drum signal (cut real regolith into `ColumnState.drum_inventory` → R²≈1.0, slope ~31 kg/A,
matching Fig 8); and `should_offload(inferred, capacity)` is the **autonomy trigger** that fires
haul-to-**process** when the UPPER confidence bound reaches the ~30 kg/cycle capacity, using the paper's
measured 2.56% (>half full) / 7.40% uncertainty as the safety margin — so it stops before overflow, and
knows its fill best exactly when the decision matters. The whole observable + inference + offload is packaged as a `DrumSensor`
with a **toggleable seeded noise** (`noise_frac=0` = OFF/deterministic by default; turn on for RL
robustness, off whenever wanted), and is **wired everywhere**: the RL envs (`worksite_env`/`scheduler_env`
take an optional `drum_sensor` → the drum-fill observation becomes the sensed value, default off =
non-breaking), the **planner report** (`mission_planner` adds `drum_cycles` + the sensed-fill note), and
the **web** (`server.py POST /sense` + the browser DRUM SENSOR widget with a noise checkbox). Tests:
`test_rassor_mass_model.py` (11) + `test_drum_sensing.py` (10, incl. sim-coupled + env wiring) + `/sense`
API tests. Future Work (per paper): lunar/low-g recalibration, online/transfer learning, autoencoders for
actuator wear — aligns with our domain-randomization + world-model track.
