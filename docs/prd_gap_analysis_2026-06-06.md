# DustGym PRD Gap Analysis

**Audit date:** 2026-06-06
**PRD:** `PRD.md`, dated 2026-06-04, living v5
**Repository commit:** `047331250cf443498c25b5bead4bed167668752c`

## Summary

The PRD is no longer an accurate statement of current product status. It contains both
stale pessimistic rows, where code has landed but the row still says unbuilt, and more
serious optimistic rows, where a requirement is marked complete despite an incomplete
integration or a demonstrated correctness failure.

Current assessment across the PRD's 136 requirement/limit rows:

| Current assessment | Count |
|---|---:|
| Substantially satisfied | 47 |
| Partial, at risk, or incompletely integrated | 57 |
| Not implemented | 20 |
| Externally gated | 4 |
| Failing the stated acceptance intent | 4 |
| Open planning limits, accurately documented | 4 |

The project is a capable research simulator and source-run planning application. It
does not currently meet the PRD's “production-grade system” intent. The primary
reasons are:

1. Invalid physical inputs are accepted.
2. The configured repository test suite cannot collect.
3. Multi-vehicle outputs disagree across totals, timeline, autonomy, and Plan IR.
4. The installed server product has incomplete dependencies, assets, and writable
   storage assumptions.
5. Vehicle selection affects geometry/rendering but not end-to-end vehicle physics.
6. The live-autonomy claims describe a self-simulator, not a telemetry-driven loop.

## Repository Delta

The tracked tree is still at the commit above. The worktree contains one untracked
review artifact:

```text
docs/architecture_review_2026-06-06_full.md
```

Consequently, the PRD's statement that `main` is clean is false for the present
worktree, although no tracked source file has been modified.

The repository has 562 tracked files: 187 Python files, 20 Godot scripts/shaders, one
browser application, two shell scripts, six YAML files, and two Dockerfiles.

## Status Legend

- **PASS**: the requirement is substantially implemented and locally evidenced.
- **PARTIAL**: useful implementation exists, but scope, integration, validation, or
  reliability is incomplete.
- **MISSING**: no implementation satisfies the requirement.
- **GATED**: implementation depends on an explicitly unavailable external oracle or
  throughput capability.
- **FAIL**: the PRD marks or implies completion, but current behavior violates the
  requirement's acceptance intent.
- **LIMIT**: an acknowledged planning limitation rather than a completion claim.

## Complete Requirement Diff

### A. Physics Core

| ID | PRD | Current | Delta |
|---|---|---|---|
| A1 | Done | **PARTIAL** | Conserved representation is real, but `ColumnState` accepts malformed arrays, NaNs, negative inventory, and negative transfer quantities. Invariants are optional rather than boundary-enforced. |
| A2 | Done | **PASS** | Bekker load-bearing implementation and tests exist. |
| A3 | Done | **PASS** | Slip/sinkage equilibrium and entrapment behavior exist. |
| A4 | Done | **PASS** | Valid cut/dump/compact operations conserve mass; invalid-input protection belongs to A1/N14. |
| A5 | Gated | **GATED** | Reduced-gravity magnitude oracle remains unavailable. |
| A6 | Gated | **GATED** | Force-accurate excavation still requires a granular/SCM authority. |

### B. Mobility

| ID | PRD | Current | Delta |
|---|---|---|---|
| B1 | Done | **PASS** | Differential-drive integration is implemented and tested. |
| B2 | Done | **PASS** | Slip-adjusted simulated `cmd_vel` loop exists; real-time product I/O remains R14. |
| B3 | Done | **PASS** | Clast contact can feed the pose/drive loop. |
| B4 | Done | **PASS** | Static stability and tip termination work for the configured geometry. End-to-end per-vehicle contact geometry remains O4. |
| B5 | Done | **PASS** | Truth-map drop-off masking and global route avoidance exist. Sensor discovery remains R7/R8. |

### C. Procedural Maps

| ID | PRD | Current | Delta |
|---|---|---|---|
| C1 | Done | **PASS** | Crater, boulder, and FBM generators with sourced envelopes exist. |
| C2 | Done | **PASS** | Seeded domain randomization exists. |
| C3 | Partial | **PARTIAL** | Feature generators exist; there is still no single authoritative composite-map builder across product paths. |

### D. Real DEM Loading

| ID | PRD | Current | Delta |
|---|---|---|---|
| D1 | Done | **PASS** | Polar LOLA ingest is implemented. |
| D2 | Done | **PASS** | The Haworth bundle is committed and loadable. |
| D3 | Done | **PASS** | Crop and resampling paths exist. |
| D4 | Done | **PASS** | Cylindrical-to-local reprojection exists; it requires the planner extra. |
| D5 | Done | **PARTIAL** | Windowed readers and streamed anchor search exist, but the live server still loads/caches the full Haworth DEM. |
| D6 | Partial | **PARTIAL** | Vendored-input convention exists; acquisition helper and durable source workflow do not. |
| D7 | Missing | **MISSING** | No interactive product-level tile/region selection into the simulation bundle. |

### E. Scale and LOD

| ID | PRD | Current | Delta |
|---|---|---|---|
| E1 | Done | **PASS** | Interaction-keyed quadtree code and tests exist. |
| E2 | Missing | **MISSING** | No coordinated multi-site active-region manager. |
| E3 | Done | **PASS** | Runtime tiled mosaic assembly exists. |

### F. Sensors and Rendering

| ID | PRD | Current | Delta |
|---|---|---|---|
| F1 | Done | **PASS** | Godot Hapke/shadow rendering is real and headless-render verified. |
| F2 | Done | **PARTIAL** | Pose-vs-truth scoring and AprilTag evidence exist, but the production live association/localization path is absent. |
| F3 | Gated | **GATED** | Render egress remains too slow for camera-in-the-loop RL. |
| F4r | Missing | **MISSING** | The shader explicitly remains a radial-distortion stub, not calibrated Brown-Conrady. |

### G. RL Environment and Training

| ID | PRD | Current | Delta |
|---|---|---|---|
| G1 | Done | **PASS** | Gymnasium environments and checker tests exist. CI does not explicitly enforce the full stated checker contract. |
| G2 | Done | **PASS** | Goal-conditioned terrain construction environment exists. |
| G3 | Done | **PASS** | Control reward and seeded randomization are implemented. |
| G4 | Done | **PASS** | PPO/CEM training code and evidence exist, though training convergence is not a CI regression gate. |
| G4b | Done | **PARTIAL** | Tip and entrapment signals are in `RoverSimEnv`; hole avoidance is a planner truth-map feature, not a learned sensor-driven avoidance signal. |
| G5 | Done | **PASS** | Active-perception environment and tests exist. |
| G6 | Done | **PARTIAL** | The learned model post-prices simulated autonomy legs. It does not change route selection or authoritative `/plan` totals as “re-prices routes, wired into `/plan`” implies. |

### H. Construction Skills

| ID | PRD | Current | Delta |
|---|---|---|---|
| H1 | Done | **PARTIAL** | Traverse physics exists, but there is no production FollowPath tracker or robust recovery executive. |
| H2 | Partial | **PARTIAL** | Grade/compact physics exists; reusable trained policy is absent. |
| H3 | Partial | **PARTIAL** | Earthmoving physics exists; a reusable policy/executive skill is absent. |
| H4 | Missing | **MISSING** | No composite BermBuild/FillHole skill implementation. |
| H5 | Partial | **PARTIAL** | Conserved sinter authority exists but is intentionally disabled for the baseline vehicle. |

### I. Structures and Planner

| ID | PRD | Current | Delta |
|---|---|---|---|
| I1 | Missing | **PARTIAL** | PRD is stale pessimistically: eight structure templates now decompose into orders. Shape/spec semantics and complete acceptance remain missing. |
| I2 | Partial | **PARTIAL** | Scheduling and order decomposition exist, but no unified goal-to-skills task planner. |
| I3 | Missing | **MISSING** | No HRL option/skill selector. |
| I4 | Done | **PASS** | Single-vehicle balancing, ordering, battery simulation, and recharge scheduling exist. |
| I13 | Done | **PASS** | Algorithm/objective selection, weighted objectives, compare, and Pareto marking exist. Fleet correctness is assessed under MV. |
| I5 | Done | **PASS** | Source-run PDF and markdown report generation works. Installed-product storage/dependencies remain N13. |
| I6 | Done | **PARTIAL** | Real DEM siting works when the asset is available; the server silently falls back to flat terrain when it is missing. |
| I7 | Done | **PASS** | Valid missions balance bank and loose material by mass. |
| I8 | Done | **PARTIAL** | Authority validation exists, but it is recomputed separately from the actual optimized/fleet plan and inherits weak physical input validation. |
| I9 | Done | **PASS** | All single-vehicle sequencers enforce precedence; fleet precedence is explicitly refused. |
| I10 | Done | **PARTIAL** | Hazard routing and slope/slip energy exist, but an unreachable route falls back to the unsafe straight line and only sets a warning counter. |
| I11 | Done | **PARTIAL** | Flatness acceptance exists. Berm profile, bearing, repose, and compaction enforcement remain; the row should never be marked complete while stating these omissions. |
| I12 | Missing | **PARTIAL** | Belief uncertainties provide a foundation, but the planner/report still lacks robust energy/time/feasibility bands. |

### J. Mission System

| ID | PRD | Current | Delta |
|---|---|---|---|
| J1 | Done | **PASS** | Declarative challenge schema exists. |
| J2 | Done | **PASS** | Seeded realization exists. |
| J3 | Done | **PASS** | Runner and scorecard exist. |
| J4 | Missing | **PARTIAL** | Product `Mission` and structure templates exist, but goal-level specs, budgets, priorities, scoring, and general footprint geometry do not. |
| J5 | Partial | **PARTIAL** | Difficulty/reset support exists; a complete held-out curriculum ladder does not. |

### K. Resources and Constraints

The PRD uses `K10` twice for unrelated requirements. This must be corrected before the
PRD can serve as a stable traceability document.

| ID | PRD | Current | Delta |
|---|---|---|---|
| K1 | Done | **PASS** | Conserved mass accounting exists for valid states. |
| K2 | Done | **PASS** | A useful battery/energy/recharge model exists, with disclosed calibration limits. |
| K10a | Done | **PARTIAL** | Endurance analysis exists, but still uses global vehicle constants and does not enforce operational windows. |
| K3 | Done/partial | **PARTIAL** | Time accounting exists; sun/thermal/comms availability is not coupled to execution. |
| K4 | Done | **PASS** | Slip/entrapment risk is represented in core mobility and planning. |
| K5 | Missing | **MISSING** | Wear remains unmodeled. |
| K6 | Done | **PASS** | Drum inference and arm-lift model exist. |
| K7 | Done | **PASS** | Drum sensing and offload threshold are integrated into env/server/UI paths. |
| K8 | Missing | **PARTIAL** | PRD is stale pessimistically: `power_regime()` and thermal derating are implemented and tested, but not coupled to the mission clock. |
| K9 | Missing | **MISSING** | No execution gating by sun, thermal, or communications windows. |
| K10b | Done | **PARTIAL** | Payload affects several slip/lift calculations, but dry mass/contact geometry and planner values remain global, so coupling is not “everywhere.” |
| K11 | Partial | **PARTIAL** | Survival draw can be enabled; terrain-dependent dig energy and drivetrain efficiency remain unresolved. |

### Autonomous-Planning Limits

| ID | Current | Delta |
|---|---|---|
| AL1 | **PARTIAL** | Warning behavior is fixed; exactness remains capped and no bound exists above the cap. |
| AL2 | **PARTIAL** | Main planning entry rejects cycles; lower public optimizer paths can still return an empty plan. |
| AL3 | **LIMIT** | Deadlines, windows, risk, and soft constraints remain unsupported. |
| AL4 | **LIMIT** | Product input remains action-level rather than goal-level. |
| AL5 | **LIMIT** | Footprints remain scalar-area squares; rich mission fields are incomplete. Keep-outs are no longer silently dropped, so that sentence is partly stale. |
| AL6 | **PARTIAL** | Flatness was added; the remaining acceptance requirements keep this unresolved. |
| AL7 | **LIMIT** | The description remains accurate: the loop is a self-simulator with no fault-handling/live perception authority. |

### MV. Multi-Vehicle

| ID | PRD | Current | Delta |
|---|---|---|---|
| MV1 | Done | **PARTIAL** | Fleet totals and allocation exist, but `/plan` returns a single-rover timeline, validation, endurance, and autonomy beside fleet totals. |
| MV2 | Done | **PASS** | Site-exclusive LPT allocation exists. |
| MV3 | Partial | **PARTIAL** | Same-site overlap detection exists; path and charger conflicts do not. |
| MV4 | Missing | **MISSING** | No charger/pit/resource queue. |
| MV5 | Missing | **MISSING** | No coordinated multi-belief replan. |
| MV6 | Missing | **PARTIAL** | PRD is stale pessimistically: vehicle registries/capabilities exist, but a fleet still has one mission-wide type and shared global physics. |
| MV7 | Partial | **PARTIAL** | Makespan/conflict fields exist, but there is no exact fleet oracle and Plan IR currently emits incorrect cross-vehicle route distances. |

### L. World Model

The five narrative layers should be converted into normal requirement IDs. Current
assessment: geometry and task representations are useful; material, uncertainty, and
physics integration are partial because consumers do not share one authoritative
state/plan artifact.

| ID | PRD | Current | Delta |
|---|---|---|---|
| L1 | Missing | **MISSING** | No learned perception encoder. |
| L2 | Missing/deprioritized | **MISSING** | No latent dynamics, deliberately deprioritized. |

### M. Visual Application

| ID | PRD | Current | Delta |
|---|---|---|---|
| M1 | Done | **PASS** | Godot and top-down render primitives work. |
| M1b | Done | **PASS** | Cockpit panes and a browser evaluation harness exist. |
| M2 | Partial | **PARTIAL** | Cesium map interaction exists; simulation-coupled 3D interaction does not. |
| M3 | Partial | **PARTIAL** | Globe coordinates feed Haworth siting; general map/tile/procedural loading does not. |
| M4 | Done | **PARTIAL** | Queue/structure authoring works, but footprints are not drawn or shaped on the map and remain scalar squares. |
| M5 | Done | **PARTIAL** | Single-rover forecast playback and offline render exist. Fleet playback is wrong/single-rover; live mutation is absent; render failures can be swallowed. |
| M6 | Missing | **PARTIAL** | A metrics/forecast pane exists, but no integrated scorecard or leaderboard product. |
| M7 | Missing | **MISSING** | No multi-rover visualization. |
| M8 | Missing | **MISSING** | No supervise/override/re-task controls. |
| M9 | Done | **PASS** | Source-run `/plan`, `/sense`, and widget integration exist. |
| M10 | Done | **PARTIAL** | Profiles persist, but there is no schema version/diff/migration; writes are non-atomic and target the package directory. |
| M11 | Done | **PARTIAL** | Haworth Moon projection exists; missing `pyproj` silently ignores a selected site, and other body/map frames are not generalized. |

### R. Robotics Autonomy

| ID | PRD | Current | Delta |
|---|---|---|---|
| R1 | Done | **PASS** | Differential-drive kinematics exist. |
| R2 | Done | **PARTIAL** | State fields exist, but publication is non-atomic and cross-language schema validation is incomplete. |
| R3 | Done | **PASS** | Grid global routing exists, subject to the unsafe no-route fallback in I10. |
| R4 | Missing | **MISSING** | No continuous/sampling planner. |
| R5 | Missing | **MISSING** | No trajectory profiler or path tracker. |
| R6 | Partial | **PARTIAL** | Scalar belief math exists; no live map-relative localization or SLAM. The current loop fuses simulated truth. |
| R7 | Partial | **PARTIAL** | Static keep-outs/drop-offs exist; no reactive dynamic-obstacle layer. |
| R15 | Done | **PASS** | Static stability exists for configured geometry. |
| R8 | Missing/gated | **MISSING** | No ICP/deep detector producing dynamic obstacles. |
| R9 | Partial/gated | **PARTIAL** | Stereo/depth producers exist, but lens calibration/live qualification remain incomplete. |
| R10 | Missing/gated | **GATED** | Force/impedance excavation still depends on the unavailable SCM/physical path. |
| R11 | Done | **PASS** | RL/search/model-based methods are represented. |
| R12 | Missing | **MISSING** | No arm FK/IK/Jacobian model. |
| R13 | Partial | **PARTIAL** | Versioned Plan IR exists, but there is no executive and fleet route expectations are currently incorrect. |
| R14 | Missing | **MISSING** | A polled JSON seam is not a streaming command/telemetry API. |

### N. Non-Functional

| ID | PRD | Current | Delta |
|---|---|---|---|
| N1 | Done | **PARTIAL** | Valid operations conserve mass, but malformed states and negative transfers are accepted at public boundaries. |
| N2 | Done | **PASS** | Core dynamics and plan IDs are deterministic for the same inputs. |
| N3 | Done | **FAIL** | Default synthetic environments, synthetic-only evaluation, Chrono/export stubs, and a distortion stub directly contradict “no synthetic/stub data.” |
| N4 | Done | **PASS** | Core step performance evidence exists. |
| N5 | Done | **PARTIAL** | Core separation is reasonable, but license/dependency cleanliness is not artifact-audited and the base package includes Gymnasium/SciPy. |
| N6 | Done | **FAIL** | Core coverage is high, but the configured suite fails collection because `trimesh` is undeclared. The stated test count is stale/ambiguous. |
| N7 | Done | **PARTIAL** | FastAPI exists, but there is no request execution timeout, worker option, production storage model, or reliable base-installed entry point. |
| N8 | Done | **PARTIAL** | Several controls exist, but negative physical inputs pass, chunked bodies bypass the early size check, auth is optional, CORS defaults `*`, and audit is absent from CI. |
| N9 | Done | **FAIL** | CI excludes `scripts`, does not explicitly run strict env-checker/warnings-as-errors, and can publish while the configured suite is uncollectable. |
| N10 | Partial | **PARTIAL** | Access logs, health, and JSON metrics exist; request IDs, structured records, Prometheus output, and broad module logging do not. |
| N11 | Partial | **PARTIAL** | Ruff/mypy/pytest config exists; no pre-commit config, large mypy ignore ratchet, and scripts are outside the actual CI test command. |
| N12 | Partial | **PARTIAL** | Version ceilings exist; no lock, reproducible fresh-install test, SBOM, or project-resolved audit. |
| N13 | Done | **FAIL** | The wheel has the package and entry point, but ships 61 test modules, includes synthetic-default envs, omits render/assets/scripts, and the entry point's dependencies are optional/incomplete. |
| N14 | Missing | **PARTIAL** | PRD is stale pessimistically: guards now exist, but constructor/state/mutation validation is incomplete and not automatically enforced. |
| N15 | Partial | **PARTIAL** | Constant/server overlays exist; report/profile directories and several operational settings remain fixed. |
| N16 | Missing | **MISSING** | No `CHANGELOG.md`, exported `__version__`, or complete release/version policy. |
| N17 | Missing | **MISSING** | ROS has a container, but there is no server deployment document/image or render dependency deployment profile. |
| N18 | Missing | **PARTIAL** | Evidence fixtures exist, but no fixture checksums or planner/AprilTag/map golden regression gate. |

### O. Configurability

| ID | PRD | Current | Delta |
|---|---|---|---|
| O1 | Done | **PARTIAL** | Numeric constants are overlaid, but body/default selections, vehicle objects, product storage, and all planner settings are not covered by one validated schema. Unknown keys are silently ignored. |
| O2 | Done | **PASS** | `CONFIG.md` documents the implemented overlay; it should stop claiming every tunable is covered. |
| O3 | Partial | **PARTIAL** | Import-time env/TOML use works; no `--config` CLI option. |
| O4 | Partial | **PARTIAL** | Registries, capability gating, geometry, and render selection exist. Drive/planner mass, wheel contact, battery, drum, and energy remain global. |
| O5 | Done | **PASS** | Active overlay state is inspectable through `config.describe()` and `/config`. |

## Delivery-Plan Diff

| Stage | Current assessment | Required correction |
|---|---|---|
| P1 | Delivered, docs stale | Replace the “stdlib server” description with FastAPI and current tests. |
| P2 | Delivered with model limits | Describe templates as mass-balanced scalar-area decompositions, not general structure geometry. |
| P3 | Intentionally gated | Keep sinter disabled for baseline IPEx; define a separate powered tool/vehicle variant before enabling. |
| P4 | Library complete, product partial | Wire windowed DEM access into server/UI and make projection dependencies part of the product extra. |
| P5 | Single-rover partial | Rebuild timeline/playback from one fleet-aware plan artifact and add terrain mutation if still required. |
| P6 | Research prototype partial | Separate truth-derived observability from reconstructed-map metrics; calibrate uncertainty and add live-data acceptance. |
| P7 | Partial/gated | Rigid clast producer exists; SCM oracle and excavation remain unavailable. |
| P8 | Partial, not complete | Remove flat/straight-line silent fallbacks and validate the same optimized plan that is reported/executed. |
| P9 | Partial | Precedence and pad flatness exist; robust bands, berm/bearing, repose, and compaction acceptance remain. |
| P10 | Partial | Power regime exists; couple availability and thermal state to mission-clock execution. |
| P11a | Partial/failing | Repair full test collection, enforce configured scope, complete invariant boundaries, and add pre-commit only after gates are truthful. |
| P11b | Partial | Add lock/audit/fresh-install checks, request IDs, structured metrics, and validated config. |
| P11c | Partial | Fix dependencies/storage/timeouts/concurrency, remove source-relative script reliance, and use one plan result. |
| P11d | Partial | Profiles and geodesy exist; add version/migration, release files, and golden baselines. |
| P11e | Partial | There are still 51 `sys.path` insertions and stale source-layout/roversim text. Preserve provenance links but remove dead operational guidance. |
| P12 | Simulation-only partial | Rename as simulated closed-loop evaluation until real telemetry, perception, fault handling, and authority mutation are wired. |
| P13 | Partial and currently incorrect for fleets | Fix Plan IR per-vehicle positions, add an executive/ROS lowering, then add streaming/replan APIs. |
| P14 | Missing | Implement planner/tracker only after the plan/execution contract is stable. |
| P15 | Missing beyond scalar belief | Build scan-to-map registration and ESKF against timestamped observations, not simulated truth fixes. |
| P16 | Missing | Add a local planner consuming dynamic obstacles. |
| P17 | Producer research exists; deliverable missing | Add obstacle detection and dynamic keep-out publication. |
| P18 | Gated | Keep explicitly gated until a force/contact authority is available. |
| P19 | Missing | Defer unless tooling requires precise end-effector placement. |

## PRD Defects

The PRD itself needs repair before it can be used as the production roadmap:

1. **Duplicate identifier:** `K10` means both endurance and weight coupling.
2. **Checkmarks with declared omissions:** I11, N7, N8, N13, M5, and other rows say
   complete while their own text lists missing acceptance criteria.
3. **Contradictory package status:** Section 15 says `planet_browser` has no
   `__init__.py` and is excluded from the wheel; both are now false.
4. **Contradictory server status:** P1/S7 still describes a stdlib server; the current
   server is FastAPI.
5. **Contradictory scope:** Section 12 says multi-agent is out of scope while MV is a
   built headline area.
6. **Stale release claims:** “701 tests pass,” “clean main,” and “tests excluded from
   wheel” are false or no longer well-defined.
7. **Stale phase ordering:** Phase 1 is written as pending even though package creation,
   wheel inclusion, and entry point have landed; Phase 2 is partly implemented.
8. **Unstable evidence:** Exact live metrics and test counts are embedded in prose
   without machine-readable evidence tied to a commit/artifact hash.
9. **Ambiguous completion:** “Implemented,” “integrated,” “validated,” “hardware
   qualified,” and “production ready” are repeatedly collapsed into one checkmark.
10. **Conflicting historical sections:** Status rollup, optimized sequence, forward
    plan, release section, and restructure decision describe different repository eras.

## Recommended PRD Model

Replace the single status icon with four independent fields:

| Field | Meaning |
|---|---|
| Implementation | Code exists for the requirement. |
| Integration | The product path actually uses it. |
| Verification | Automated acceptance test proves the stated behavior. |
| Qualification | Real data/hardware/deployment evidence exists where required. |

Each row should also contain:

- one owner;
- one source module/API;
- one acceptance test ID;
- one evidence artifact tied to a commit;
- explicit blocked dependencies;
- a last-verified date.

Generated status should come from a small checked-in requirement manifest rather than
being manually duplicated across Sections 5, 8, 9, 14, and 15.

## Recommended Work Order

### P0: Restore truthful release gates

1. Reject non-finite/negative physical inputs and enforce complete `ColumnState`
   construction/mutation invariants.
2. Declare `trimesh` appropriately and make CI run the configured suite.
3. Add a fresh-wheel smoke test for `dustgym-serve`, all registered envs, and planner
   imports.
4. Correct N3/N6/N9/N13 and the stale release/test claims in the PRD immediately.

### P0: Establish one authoritative plan

1. Introduce an immutable `PlanResult` containing allocation, trips, per-vehicle
   timelines, validation state, energy ledger, and acceptance results.
2. Generate totals, reports, Plan IR, autonomy inputs, and browser playback only from
   that result.
3. Fix Plan IR with `prev_by_vehicle`; add route, time, and energy conservation tests
   per vehicle.
4. Treat unreachable terrain as infeasible instead of falling back to a straight line.

### P1: Make the installed product real

1. Define coherent `planner`, `server`, `render`, and `dev` extras; the server extra
   must include all import-time planner dependencies.
2. Move reports/profiles/cache to a configurable application-data directory.
3. Package or version-download required terrain/render assets.
4. Add request execution limits, bounded queues, atomic persistence, and a supported
   multi-worker storage/locking model.

### P1: Complete vehicle and mission contracts

1. Pass a typed `VehicleModel` through contact geometry, mass, battery, drum, drive,
   terramechanics, planner simulation, and Plan IR.
2. Replace scalar footprints with typed rectangle/circle/corridor/polygon geometry.
3. Unify `Challenge`, structures, and product `Mission` around goal-level acceptance,
   budgets, priorities, and dependencies.

### P1/P2: Build autonomy only on validated seams

1. Separate `SimulatedExecutive` from live execution interfaces.
2. Make state-field publication atomic and schema-validated across Python, Godot, and
   ROS.
3. Add timestamped observation association, scan-to-DEM registration, and ESKF before
   claiming live localization.
4. Add continuous/path-tracking and reactive planners only after command, telemetry,
   and dynamic-obstacle contracts exist.

## Final Assessment

The PRD's strategic direction is mostly sound: authority-first physics, a planning
product, explicit autonomy gaps, and production hardening are the correct themes. Its
status accounting is not sound enough to manage release decisions.

The immediate project goal should not be another feature area. It should be to make
the existing product paths agree on one validated physical plan, make the configured
quality gate truthful, and make the installed artifact behave like the advertised
server product. Once those are complete, the PRD can credibly move from research
capability tracking to production delivery tracking.
