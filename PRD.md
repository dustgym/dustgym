# PRD: DustGym Lunar Construction and Solar-Terrain Autonomy

**Version:** 6.0
**Date:** 2026-06-06
**Status:** Current product and architecture baseline
**Baseline commit:** `047331250cf443498c25b5bead4bed167668752c`

## 1. Purpose

DustGym is a lunar construction-planning and digital-twin product for an IPEx/RASSOR-lineage
excavator. It must:

1. load real or generated terrain;
2. author construction goals and constraints;
3. produce a physically valid, energy-aware plan;
4. simulate and visualize execution;
5. emit a mission-control report and machine-consumable plan;
6. support a progression from simulated autonomy to sensor-driven navigation and execution.

The next product expansion is **solar-terrain autonomy**: use terrain shape, terrain changes,
low-sun illumination, shadow geometry, camera/LED selection, and articulated-arm posture to improve
mapping, localization, route safety, and construction execution.

This PRD replaces the June 4 v5 document. Historical stage narratives remain available through Git
history. Current status is based on:

- [`docs/architecture_review_2026-06-06_full.md`](docs/architecture_review_2026-06-06_full.md)
- [`docs/prd_gap_analysis_2026-06-06.md`](docs/prd_gap_analysis_2026-06-06.md)
- the current repository and locally executed verification described by those reviews.

The conserved terramechanics authority retains John McCardle's CC0 provenance. DustGym's product,
planner, Gymnasium, vehicle, perception, and visualization layers build on that authority.

## 2. Source Discipline

Every physical or operational claim must carry one of these evidence classes:

| Tag | Meaning |
|---|---|
| `[SPEC]` | Directly stated by an authoritative NASA, LAC, standards, or peer-reviewed source. |
| `[MEASURED]` | Measured by DustGym or a cited experiment with reproducible conditions. |
| `[CALIB]` | Calibrated model value with a documented data source and fitting procedure. |
| `[ASSUMPTION]` | Deliberate engineering assumption exposed through configuration. |
| `[PROPOSED]` | New behavior or algorithm that must be validated before capability claims. |
| `[UNKNOWN]` | Required parameter or behavior for which no defensible value is available. |

### 2.1 New references

**[NAVLAB26]** A. Dai et al., *Full Stack Navigation, Mapping, and Planning for the Lunar
Autonomy Challenge*, arXiv:2603.17232v1, March 18, 2026. Local review copy:
`/home/aaron/Downloads/2603.17232v1.pdf`. Publication page:
`https://arxiv.org/abs/2603.17232`.

This paper provides a validated LAC simulator reference architecture:

- semantic segmentation;
- SuperPoint + LightGlue feature matching;
- stereo visual odometry using triangulation and `solvePnPRansac`;
- GTSAM pose-graph optimization and loop closure;
- median-cell terrain mapping and majority-vote rock mapping;
- overlapping-loop/outward-spiral coverage planning;
- constant-curvature local arc sampling;
- reverse-and-replan recovery when progress collapses.

Its reported localization RMSE was approximately `0.038-0.067 m` across documented presets and
seeds. Those results are a benchmark, not evidence that DustGym currently achieves them.

**[IPEx-DT-REF]** *IPEx Rover: Architectural Review & Digital-Twin / World-Model Reference*.
Local working reference:
`/home/aaron/Downloads/IPEx_Rover_Architecture_DigitalTwin_Reference.md`.

This is a secondary synthesis. Statements marked `[SPEC]` in that document must still be traced to
its listed NASA/LAC source before becoming fixed model constants. Statements marked `[EST]` there
are treated as `[PROPOSED]` or `[ASSUMPTION]` here.

### 2.2 Solar-navigation claim boundary

The NavLab paper establishes robust navigation under variable lunar lighting. It does **not**
establish shadow-azimuth heading, arm-controlled solar observation, or Meerkat solar navigation.
Those are proposed DustGym research/product requirements derived from the IPEx/LAC platform
capabilities and south-pole lighting environment.

## 3. Status Model

A single checkmark is not sufficient. Every requirement carries four independent states:

| Column | Meaning |
|---|---|
| `I` | Implementation exists. |
| `X` | Integrated into the advertised product path. |
| `V` | Automated acceptance verifies the stated behavior. |
| `Q` | Qualified with representative external data, hardware, or deployment evidence. |

Values:

- `D`: done for the stated scope;
- `P`: partial;
- `N`: not done;
- `G`: externally gated;
- `NA`: qualification does not apply.

A requirement is release-ready only when its required columns are `D`. Research prototypes may
have `I=D` while `X`, `V`, or `Q` remain partial.

## 4. Current Product Truth

### 4.1 Working foundations

- Conserved mass-per-area terrain authority with derived height.
- Bekker/slip/sinkage mobility and mass-conserving earthmoving.
- Seeded Gymnasium environments and high Python source coverage.
- Real Haworth LOLA terrain bundle and non-polar reprojection library.
- Structure templates, cut/fill balancing, multiple sequence optimizers, precedence, and reports.
- Godot terrain/rover rendering and a browser planning cockpit.
- Versioned Plan IR and a simulated belief/replan loop.
- Initial fleet allocation and parallel makespan calculation.

### 4.2 Release blockers

| ID | Blocker | Required exit |
|---|---|---|
| RB-01 | Negative/non-finite physical values and malformed authority state are accepted. | Shared domain validation at every public boundary; mutation invariants enforced. |
| RB-02 | The configured test suite cannot collect because `trimesh` is undeclared; CI excludes that path. | Declared dependency/marker policy and CI running the configured suite. |
| RB-03 | Fleet totals, timeline, autonomy, validation, and UI do not represent one plan. | One immutable fleet-aware `PlanResult` consumed by all outputs. |
| RB-04 | Multi-vehicle Plan IR leaks position between vehicles. | Per-vehicle state ledger and route/energy tests. |
| RB-05 | Vehicle selection does not drive end-to-end mass, contact, energy, and capacity. | Typed `VehicleModel` threaded through authority, planner, Plan IR, and rendering. |
| RB-06 | The installed server has incomplete dependencies, assets, and writable storage assumptions. | Fresh-wheel server smoke test with externalized data directories and explicit asset mode. |

No production-grade release may be declared while any `RB-*` item is open.

## 5. Product Modes

DustGym supports four explicitly distinct modes:

| Mode | Purpose | Truth boundary |
|---|---|---|
| `PLAN` | Author and optimize a construction mission. | Model-based forecast over validated terrain and vehicle data. |
| `SIMULATE` | Execute against the conserved authority and synthetic/rendered sensors. | Simulation only; no live-hardware claim. |
| `EVALUATE` | Score maps, localization, plans, and acceptance against known truth. | Test/benchmark mode; truth access allowed. |
| `OPERATE` | Consume real telemetry and issue commands. | Future mode; unavailable until command, timing, safety, and fault requirements pass. |

The API and reports must label the active mode. Simulated truth must never be presented as a live
measurement.

## 6. Target Architecture

```text
L7  Product and operations
    browser / API / reports / profiles / deployment / observability

L6  Mission and fleet planning
    goals / structures / PlanResult / resources / acceptance / Plan IR

L5  Navigation and execution
    coverage planner / local planner / tracker / recovery / executive

L4  Perception and localization
    camera policy / segmentation / stereo VO / SLAM / map / solar factors

L3  Vehicle and posture digital twin
    VehicleModel / arms / drums / per-drum load / CG / support polygon / Meerkat

L2  Terrain, illumination, and world state
    conserved terrain / rocks / uncertainty / sun vector / shadows / mutable illumination

L1  Physical authority
    terramechanics / mobility / excavation / energy / thermal / power

L0  Contracts
    units / schemas / time / frames / provenance / invariant enforcement
```

### 6.1 Authoritative artifacts

The architecture must have these single-source runtime artifacts:

1. `WorldState`: terrain, material, rocks, illumination, uncertainty, time, and frame metadata.
2. `VehicleState`: pose, velocity, arm angles, per-drum fill, battery, thermal/dust state, and health.
3. `VehicleModel`: geometry, mass properties, contact, capacity, actuators, sensors, and power.
4. `BeliefState`: estimated state and covariance, separate from simulator truth.
5. `PlanResult`: fleet allocation, routes, actions, timeline, resources, acceptance, and provenance.
6. `ExecutionEvent`: command, observation, acknowledgement, fault, replan, and state-transition record.

Reports, Plan IR, playback, validation, and autonomy must be views over these artifacts, not
independent recomputations.

## 7. Requirements

### 7.1 Contracts and Conserved Authority

| ID | P | Requirement and acceptance | I | X | V | Q |
|---|---|---|---|---|---|---|
| CT-01 | P0 | All public numeric inputs enforce units, finiteness, and physical domains. Negative depth/mass and NaN/Inf are rejected. | P | P | P | NA |
| CT-02 | P0 | `ColumnState` validates dimensions, array shapes, dtypes/domains, density, labels, disturbance, datum, ice, and inventory at construction. | P | N | P | NA |
| CT-03 | P0 | Every authority mutation is transactional, conserves mass when required, and leaves all invariants valid. | P | P | P | NA |
| CT-04 | P0 | Scene publication writes verified rasters atomically and metadata last as the commit marker. | N | N | N | NA |
| CT-05 | P0 | Python, Godot, and ROS share a versioned schema with strict required-field, frame, dtype, and range validation. | P | P | P | NA |
| CT-06 | P0 | Production contract checks use explicit exceptions, never removable `assert` statements. | P | P | N | NA |
| CT-07 | P1 | Every artifact records source commit, configuration, mode, seed, schema version, and input hashes. | P | P | N | NA |

### 7.2 Terrain, Material, and Illumination

| ID | P | Requirement and acceptance | I | X | V | Q |
|---|---|---|---|---|---|---|
| TW-01 | P0 | Load and crop real polar LOLA terrain; fail explicitly if a requested real asset is unavailable. | D | P | D | P |
| TW-02 | P1 | Reproject supported non-polar products into a documented local metric frame. | D | P | D | P |
| TW-03 | P1 | Product paths use windowed/tiled terrain access rather than loading the full map by default. | D | N | P | NA |
| TW-04 | P1 | One seeded composite generator combines craters, rocks, material, and illumination parameters. | P | P | P | NA |
| TW-05 | P1 | `WorldState` carries per-cell material, traversability, observed/unobserved state, and calibrated uncertainty. | P | P | P | P |
| TW-06 | P1 | Add a site/time sun vector `s(t)` in the local world frame using a documented ephemeris interface. | N | N | N | N |
| TW-07 | P1 | Compute terrain horizon, direct illumination, cast-shadow mask, incidence angle, and overexposure risk from terrain plus `s(t)`. | P | N | P | P |
| TW-08 | P1 | Recompute affected illumination and navigation layers after excavation changes terrain. No stale pre-build shadow map may remain authoritative. | N | N | N | NA |
| TW-09 | P2 | Model camera LED contribution separately from solar illumination, including configurable intensity and pose. | P | N | N | N |
| TW-10 | P2 | Track dust/optical degradation as a state affecting image quality and maintenance decisions. `[PROPOSED]` | N | N | N | N |

### 7.3 Vehicle, Arms, Drums, and Stability

| ID | P | Requirement and acceptance | I | X | V | Q |
|---|---|---|---|---|---|---|
| VT-01 | P0 | A typed `VehicleModel` supplies mass, gauge, wheelbase, wheel/contact geometry, CG, battery, drum capacity, speed, energy, sensors, and render assets. | P | P | P | P |
| VT-02 | P0 | Selecting a vehicle changes all applicable authority/planner numbers; cross-vehicle tests assert expected differences. | N | N | N | N |
| VT-03 | P1 | Model front and rear arm joint state, limits, velocity, brake state, and energy. Exact geometry must come from authoritative LAC/IPEx data. | N | N | N | G |
| VT-04 | P1 | Track four drums and per-drum fill rather than one global inventory for IPEx mode. | N | N | N | P |
| VT-05 | P1 | Compute dynamic CG from chassis, arm pose, drum pose, and fill mass. `[SPEC/PROPOSED model]` | N | N | N | G |
| VT-06 | P1 | Compute posture-dependent support polygon and static stability margin each step. | P | N | P | G |
| VT-07 | P1 | Nominal excavation requires balanced front/rear counter-rotation; asymmetric digging exposes reaction, traction, yaw, and pitch risk. | N | N | N | P |
| VT-08 | P1 | Drum fill-rate supports the sourced bridging behavior: effective collection need not increase monotonically beyond approximately half scoop depth. | N | N | N | P |
| VT-09 | P2 | Arm/drum force and torque model distinguishes horizontal reaction, vertical fill-dependent load, cutting torque, and internal tumble. | N | N | N | G |
| VT-10 | P1 | Posture-dependent camera extrinsics are derived from vehicle and arm state for every image. | N | N | N | G |

### 7.4 Meerkat and Excavator-Arm Maneuvers

The maneuver vocabulary is sourced from LAC/IPEx/RASSOR capabilities through
`[IPEx-DT-REF]`; exact geometry and transition limits remain qualification inputs.

| ID | P | Requirement and acceptance | I | X | V | Q |
|---|---|---|---|---|---|---|
| AM-01 | P1 | Implement an explicit posture state machine: `TRANSIT`, `DIG`, `DUMP_Z`, `MEERKAT`, `DRUM_WALK`, `IRON_CROSS`, `SELF_RIGHT`, and `BRAKED_HOLD`. | N | N | N | G |
| AM-02 | P1 | Every transition has preconditions for slope, arm range, drum load, support contacts, stability margin, and collision clearance. | N | N | N | G |
| AM-03 | P1 | `MEERKAT` raises the camera vantage by lowering arms under the chassis; motion is speed-limited and rejected when stability margin is inadequate. | N | N | N | G |
| AM-04 | P1 | Differential front/rear arm pose may be used as a controlled camera-pitch action only after kinematic and stability validation. `[PROPOSED]` | N | N | N | G |
| AM-05 | P2 | `DRUM_WALK` supports bounded slow translation while raised and records contact/slip/energy separately from wheel drive. | N | N | N | G |
| AM-06 | P2 | `IRON_CROSS` permits wheel-cleaning/recovery only under explicit raised-posture safety limits. | N | N | N | G |
| AM-07 | P2 | `SELF_RIGHT` is a fault-recovery plan with transient stability/contact checks; it is not available as an unconstrained action. | N | N | N | G |
| AM-08 | P1 | Arm brake allows a validated posture hold with zero or modeled holding power; transition energy remains charged. | N | N | N | G |
| AM-09 | P1 | The planner may choose Meerkat only when predicted information gain or recovery value exceeds time, energy, and risk cost. `[PROPOSED]` | N | N | N | N |

### 7.5 Perception, Mapping, and Localization

The target spine follows the modular pattern demonstrated by `[NAVLAB26]`. Equivalent components are
allowed if they meet the acceptance criteria.

| ID | P | Requirement and acceptance | I | X | V | Q |
|---|---|---|---|---|---|---|
| PM-01 | P0 | Time-synchronize camera, IMU, command, arm, and truth/evaluation streams using explicit clocks and frame IDs. | P | N | P | N |
| PM-02 | P1 | Support the documented IPEx/LAC camera set and a maximum active-camera budget; camera activation/resolution has compute and energy cost. | P | N | N | G |
| PM-03 | P1 | Segment at least ground, rock, lander, fiducial, and sky from grayscale images without truth masks in evaluation mode. | N | N | N | N |
| PM-04 | P1 | Detect/match illumination-robust features and expose confidence/inlier statistics. `[NAVLAB26 reference: SuperPoint + LightGlue]` | N | N | N | N |
| PM-05 | P0 | Stereo VO triangulates landmarks, maintains persistent tracks, and estimates relative SE(3) pose with robust outlier rejection. | P | N | P | N |
| PM-06 | P0 | Fuse VO/IMU and validated absolute factors in a recursive estimator or factor graph with covariance. | P | N | P | N |
| PM-07 | P0 | Loop closures are candidate-gated, geometrically verified, and auditable; false closures must not silently enter the graph. | N | N | N | N |
| PM-08 | P1 | Produce a local/world elevation map using robust per-cell aggregation and a rock occupancy/probability map. | P | P | P | P |
| PM-09 | P1 | Track observed coverage, effective sample support, uncertainty floor, and correlation; dense pixels from one view are not treated as independent evidence. | P | P | P | N |
| PM-10 | P1 | Benchmark on a fixed LAC-style suite: localization RMSE, 5 cm height-cell pass fraction, rock F1, coverage, runtime, and failure count across seeds/light/rocks. | P | N | P | N |
| PM-11 | P1 | Target benchmark: demonstrate repeatable centimeter-scale localization comparable to the `0.038-0.067 m` `[NAVLAB26]` reference before claiming parity. | N | N | N | N |
| PM-12 | P1 | Truth pose and semantic masks are development/evaluation-only and structurally unavailable to operational estimator code. | N | N | N | NA |

### 7.6 Solar-Terrain Navigation

Solar-terrain navigation is the use of known/estimated solar geometry and terrain-induced
illumination as navigation evidence and an active-perception control variable. It is distinct from
solar power scheduling.

| ID | P | Requirement and acceptance | I | X | V | Q |
|---|---|---|---|---|---|---|
| SN-01 | P1 | Derive expected shadow azimuth from `s(t)` and local terrain/objects. `[PROPOSED]` | N | N | N | N |
| SN-02 | P1 | Detect reliable shadow vectors while rejecting rover/LED shadows, saturation, ambiguous penumbra, and texture edges. `[PROPOSED]` | N | N | N | N |
| SN-03 | P1 | Fuse accepted shadow evidence as a weak yaw factor with covariance; never as an unqualified absolute heading. `[PROPOSED]` | N | N | N | N |
| SN-04 | P1 | Re-evaluate shadow factors when terrain is excavated, the sun vector changes, or the observation viewpoint changes. | N | N | N | NA |
| SN-05 | P1 | Add illumination-aware route cost: visibility, saturation, shadow hazard, map uncertainty, energy, slope, and construction constraints remain separate inspectable terms. | N | N | N | N |
| SN-06 | P1 | Choose camera direction and exposure to avoid low-sun washout while preserving useful stereo overlap. | N | N | N | N |
| SN-07 | P1 | Choose camera subset and LED intensity to illuminate hard shadows within the active-camera and power budgets. | N | N | N | G |
| SN-08 | P1 | Permit arm-angle selection for near-field downward mapping or horizon/sun-grazing views using posture-dependent extrinsics. `[PROPOSED]` | N | N | N | G |
| SN-09 | P1 | Permit a Meerkat observation action for multi-height parallax and shadow/rock disambiguation when stability guards pass. `[PROPOSED]` | N | N | N | G |
| SN-10 | P1 | The active-perception objective maximizes expected localization/map information per joule and second, with stability risk as a hard constraint. | P | N | P | N |
| SN-11 | P1 | Low/high posture observations must be associated to the same world features through the current arm/camera transforms. | N | N | N | G |
| SN-12 | P1 | Solar-navigation claims require ablations against VO/SLAM without solar factors across multiple sun angles, terrains, terrain-change states, and seeds. | N | N | N | N |
| SN-13 | P1 | Acceptance target `[PROPOSED]`: improve median yaw/pose error or feature-track survival by a preregistered margin without increasing tip events; report energy/time overhead. | N | N | N | N |

### 7.7 Navigation, Planning, and Recovery

| ID | P | Requirement and acceptance | I | X | V | Q |
|---|---|---|---|---|---|---|
| NV-01 | P0 | Global routing rejects unreachable goals; it never substitutes an unsafe straight line. | P | P | P | NA |
| NV-02 | P1 | Coverage routes promote map coverage and deliberate re-observation/loop closure. `[NAVLAB26 reference: overlapping loops/outward spiral]` | P | N | P | N |
| NV-03 | P1 | A local planner samples dynamically feasible short-horizon trajectories and rejects rock/terrain collisions. `[NAVLAB26 reference: constant-curvature arcs]` | N | N | N | N |
| NV-04 | P1 | A path tracker converts trajectories into bounded commands and reports expected speed/progress. | N | N | N | N |
| NV-05 | P1 | Reactive obstacle observations update dynamic keep-outs and trigger local/global replan. | N | N | N | N |
| NV-06 | P1 | Backup recovery triggers on progress ratio, duration, and planner failure; initial benchmark uses the `[NAVLAB26]` less-than-25%-for-2-to-3-second rule as a configurable reference. | N | N | N | N |
| NV-07 | P1 | Recovery distinguishes collision/obstacle blockage from expected slope/slip slowdown to avoid false reverse maneuvers. | N | N | N | N |
| NV-08 | P1 | Tip, entrapment, localization divergence, low energy, thermal violation, and actuator faults are explicit fault classes. | P | N | P | N |
| NV-09 | P1 | An executive monitors action preconditions, command acknowledgements, belief covariance, and acceptance state, then pauses/replans/fails safely. | N | N | N | N |
| NV-10 | P0 | Plan IR maintains independent position, energy, time, and action state per vehicle. | P | P | N | NA |
| NV-11 | P1 | ROS lowering emits paths, motion commands, arm/drum goals, observation actions, and replan events from Plan IR. | N | N | N | N |
| NV-12 | P1 | Live command/telemetry uses a versioned streaming API with timestamps, sequence numbers, backpressure, and safe-stop semantics. | N | N | N | N |

### 7.8 Construction Mission Planning

| ID | P | Requirement and acceptance | I | X | V | Q |
|---|---|---|---|---|---|---|
| CP-01 | P0 | One immutable `PlanResult` is produced once and consumed by totals, report, validation, timeline, Plan IR, autonomy, and UI. | N | N | N | NA |
| CP-02 | P0 | Balance bank cut and loose fill by mass with drum/capacity constraints. | D | D | D | P |
| CP-03 | P0 | Execute/validate the selected optimized plan on the conserved authority and real terrain. | P | P | P | P |
| CP-04 | P1 | Goal grammar supports typed structures, tolerances, budgets, priorities, deadlines, dependencies, and keep-outs. | P | P | P | NA |
| CP-05 | P1 | Footprints support rectangle, circle, corridor, and polygon with orientation; scalar-area squares are legacy input only. | N | N | N | NA |
| CP-06 | P1 | Acceptance includes pad flatness, berm profile, bearing/compaction, repose stability, mass, time, and energy. | P | P | P | P |
| CP-07 | P1 | Plan uncertainty carries DEM, material, slip, dig-rate, drum-fill, localization, and power-window uncertainty into feasibility/time/energy bands. | P | N | P | N |
| CP-08 | P1 | Planner objectives support hard constraints and risk terms, not only unconstrained weighted metrics. | N | N | N | NA |
| CP-09 | P1 | Construction actions mutate `WorldState`; routing, illumination, observability, and acceptance consume the updated terrain. | P | N | P | NA |
| CP-10 | P1 | Sinter remains unavailable for baseline IPEx; enabling it requires a distinct tool/power model and capability-qualified vehicle. | D | D | D | P |

### 7.9 Energy, Thermal, Power, and Operations

| ID | P | Requirement and acceptance | I | X | V | Q |
|---|---|---|---|---|---|---|
| EP-01 | P0 | Energy ledger includes drive, slope/slip, payload, dig, arm/drum motion, observation, LEDs, compute, idle/heater, and recharge losses where modeled. | P | P | P | P |
| EP-02 | P1 | Dig energy depends on material/density/ice or is explicitly marked constant-model uncertainty. | N | N | N | N |
| EP-03 | P1 | Distinguish PSR lander/tower power from sunlit solar power. | D | P | D | P |
| EP-04 | P1 | Mission clock enforces power, illumination, thermal, and communications windows on actions/recharge. | N | N | N | N |
| EP-05 | P1 | Thermal derating and heater/survival demand affect usable battery and action availability. | P | P | P | N |
| EP-06 | P1 | Meerkat/arm posture and camera/LED policies include transition and dwell energy. | N | N | N | G |
| EP-07 | P2 | Dust accumulation affects optics, joints, thermal surfaces, and maintenance actions. | N | N | N | N |
| EP-08 | P1 | Endurance and reports use the selected `VehicleModel`, not global IPEx constants. | N | N | N | N |

### 7.10 Fleet Planning

| ID | P | Requirement and acceptance | I | X | V | Q |
|---|---|---|---|---|---|---|
| FL-01 | P0 | Fleet allocation, simulation, validation, timeline, Plan IR, and playback share one `PlanResult`. | P | N | N | NA |
| FL-02 | P1 | Detect and resolve route, site, and temporal conflicts rather than only same-site overlap. | P | N | P | NA |
| FL-03 | P1 | Model charger, pit, dump, observation vantage, and constrained corridor as shared resources. | N | N | N | NA |
| FL-04 | P1 | Maintain one belief/health/resource state per rover and coordinate replans. | N | N | N | N |
| FL-05 | P2 | Support heterogeneous vehicle capability and physics vectors. | P | N | P | N |
| FL-06 | P1 | Validate two-rover plans against an exact small-problem oracle before learned/heuristic superiority claims. | N | N | N | NA |
| FL-07 | P1 | Solar/Meerkat observation sites are reservable fleet resources so rovers do not occlude or collide during raised observations. `[PROPOSED]` | N | N | N | N |

### 7.11 Product, Packaging, and Operations

| ID | P | Requirement and acceptance | I | X | V | Q |
|---|---|---|---|---|---|---|
| PO-01 | P0 | `dustgym-serve` works after a fresh wheel install with one documented product extra. | P | P | N | N |
| PO-02 | P0 | Reports, profiles, caches, and renders use configurable application-data directories and atomic writes. | N | N | N | NA |
| PO-03 | P0 | CI installs declared dependencies and runs the configured suite across supported Python versions. | P | P | N | NA |
| PO-04 | P0 | CI separately gates Python core, scripts, Godot, browser, package smoke, and hardware-gated tiers. | P | N | P | NA |
| PO-05 | P1 | Commit a dependency lock, build an SBOM, scan resolved artifacts, and run a fresh-install test. | P | N | N | NA |
| PO-06 | P1 | Server enforces streamed body limits, execution timeouts, bounded concurrency, auth policy, and deployment-safe CORS. | P | P | P | N |
| PO-07 | P1 | Structured logs include request/event ID, mode, plan ID, route, duration, outcome, and error class. | P | P | P | N |
| PO-08 | P1 | Metrics are bounded and exportable in a standard operations format. | P | P | P | N |
| PO-09 | P1 | Mission/profile schemas are versioned and migratable. | P | P | P | NA |
| PO-10 | P1 | UI distinguishes forecast, simulation truth, estimator belief, and live telemetry. | P | P | P | NA |
| PO-11 | P1 | Fleet playback renders every rover and its independent telemetry. | N | N | N | NA |
| PO-12 | P1 | Solar view displays sun vector, illumination/shadow layers, active cameras/LEDs, arm posture, and evidence accepted/rejected by localization. | N | N | N | N |
| PO-13 | P1 | Add `CHANGELOG.md`, exported `__version__`, SemVer policy, and release evidence manifest. | N | N | N | NA |
| PO-14 | P1 | Provide deployment documentation and a supported server image; optional Godot/ROS capabilities are explicit profiles. | N | N | N | N |

## 8. User Workflows

### 8.1 Construction planning

1. Select body, terrain product, site, vehicle, tools, and operating mode.
2. Place typed structures/footprints and constraints.
3. Validate terrain, power, vehicle, and mission inputs.
4. Produce one `PlanResult`.
5. Review routes, resources, uncertainty, acceptance, and infeasibility.
6. Export report and Plan IR.
7. Simulate against the authority and compare actual simulation results to forecast.

### 8.2 Mapping/navigation evaluation

1. Select a benchmark scene, sun condition, rocks, spawn, and seed.
2. Run without truth access in the estimator/planner process.
3. Execute coverage and local navigation with recovery.
4. Score pose, terrain height, rock map, coverage, energy, and failures against held-out truth.
5. Run ablations for loop closure, solar factors, active camera policy, and Meerkat observations.

### 8.3 Solar-terrain observation

1. Predict useful illumination and expected shadow direction from terrain plus `s(t)`.
2. Evaluate visibility, saturation, feature support, map uncertainty, stability, energy, and time.
3. Choose transit, arm-angle observation, LED-assisted observation, or guarded Meerkat observation.
4. Update posture-dependent camera transforms.
5. Acquire synchronized images/IMU/arm state.
6. Accept or reject shadow heading evidence using residual and covariance gates.
7. Update belief/map and replan.
8. After earthmoving, recompute terrain illumination before the next observation decision.

## 9. Verification Strategy

### 9.1 Test tiers

| Tier | Runs in standard CI | Purpose |
|---|---|---|
| T0 | Yes | Unit/domain/invariant tests. |
| T1 | Yes | Cross-module plan, vehicle, terrain, and API integration. |
| T2 | Yes | Fresh-wheel install, browser syntax, headless Godot parser/render smoke. |
| T3 | Scheduled/artifact runner | Nav/perception benchmark over fixed rendered datasets. |
| T4 | Hardware/external environment | ROS, physical/test-site, Chrono SCM, calibrated cameras/arms. |

### 9.2 Required benchmark matrix

The autonomy benchmark must vary:

- terrain seed and real-terrain crop;
- rock distribution;
- initial pose;
- sun azimuth/elevation and exposure;
- unchanged versus excavated terrain;
- low versus Meerkat observation posture;
- LEDs off/on;
- fiducials available/disabled;
- nominal versus degraded camera/feature conditions.

Every result records configuration and source hashes.

### 9.3 Solar-navigation acceptance

No solar-navigation capability claim is allowed until:

1. the sun vector and frame transform are independently verified;
2. shadow factors are rejected when inconsistent with terrain/viewpoint;
3. an ablation demonstrates benefit or clearly bounded no-benefit conditions;
4. terrain mutation invalidates/recomputes affected shadow predictions;
5. posture/camera transforms are sourced and tested;
6. Meerkat transitions maintain a positive configured stability margin;
7. energy/time/risk overhead is reported.

## 10. Roadmap

### Phase 0: Truthful baseline and release gates

**Exit:** all `RB-*` issues closed.

- Complete physical input/state validation.
- Fix the configured suite and CI scope.
- Introduce `PlanResult`; fix fleet Plan IR/timeline/autonomy consistency.
- Repair installed-server dependencies, assets, and storage.
- Correct documentation claims and remove stale source-layout guidance.

### Phase 1: Vehicle and posture twin

**Exit:** one vehicle/arm/drum state drives physics, rendering, planning, and sensors.

- Complete `VehicleModel`.
- Import authoritative IPEx/LAC arm/camera geometry.
- Add arm joints, four drum inventories, dynamic CG, support polygon, and posture transforms.
- Implement guarded posture state machine through Meerkat and braked hold first.
- Keep drum-walk, iron-cross, and self-right behind qualification gates.

### Phase 2: LAC-derived navigation spine

**Exit:** repeatable sensor-only mapping/navigation benchmark without truth leakage.

- Time synchronization and strict frames.
- Segmentation and robust stereo feature/VO pipeline.
- Covariant estimator/factor graph and loop closure.
- Height/rock map generation.
- Coverage routes, local arc planner, tracker, and backup recovery.
- Benchmark against the `[NAVLAB26]` architecture and metrics.

### Phase 3: Solar-terrain active perception

**Exit:** validated solar evidence and arm/Meerkat observation decisions improve or safely preserve
navigation performance under defined low-sun conditions.

- Sun-vector service and mutable terrain illumination.
- Shadow extraction and weak yaw-factor fusion.
- Illumination-aware route/camera/exposure/LED policy.
- Posture-dependent views and multi-height association.
- Guarded Meerkat observation planner.
- Full ablation across sun, terrain change, posture, and seed.

### Phase 4: Construction under changing terrain

**Exit:** planning, navigation, perception, and acceptance consume the same mutated world.

- Rich footprint/goal grammar.
- Authority execution of the selected plan.
- Terrain-dependent routing and illumination updates after each work action.
- Complete structure acceptance and uncertainty bands.
- Tool/arm/drum actions in Plan IR and executive.

### Phase 5: Fleet and operational product

**Exit:** deployable, observable product with coordinated fleets and supported live I/O.

- Shared-resource fleet scheduling and coordinated replan.
- Fleet visualization and telemetry.
- Streaming command/event API and ROS lowering.
- Deployment image/docs, locks/SBOM/audit, versioned persistence, and release process.

## 11. KPIs

### Physics and construction

- Mass drift: `<= 1e-9` relative for conserved operations.
- Invalid-state acceptance: zero in public constructors/mutations.
- Plan/simulation mass and action ledger: exact within declared numeric tolerance.
- Pad flatness and other structure acceptance: per mission specification.

### Navigation and mapping

- Localization RMSE across benchmark seeds and lighting.
- Height-map RMSE and fraction of evaluated cells within `0.05 m`.
- Rock precision, recall, and F1.
- Coverage and uncertainty versus energy/time.
- Loop-closure acceptance/rejection and catastrophic-failure count.
- Local-planner collision count, stuck time, and successful recoveries.

### Solar-terrain autonomy

- Shadow-factor yaw residual and accepted-factor precision.
- Feature-track survival under low-sun/saturation conditions.
- Pose/map improvement versus no-solar ablation.
- Information gain per joule/second for transit, arm-angle, LED, and Meerkat observations.
- Stability margin and tip/fault count during posture transitions.
- Correct illumination invalidation after terrain mutation.

### Product and operations

- Fresh-wheel install and server startup.
- Full configured CI suite pass.
- Plan response consistency across totals/report/timeline/IR/playback.
- Bounded request latency, queue depth, error rate, and artifact storage.
- Reproducibility from manifest, lock, input hashes, and seed.

## 12. Non-Goals

- Flight certification.
- Full granular DEM at map scale.
- General-purpose manipulation, grasping, humanoid, or legged control.
- Fabricated arm, camera, force, or power constants.
- Claiming real closed-loop autonomy from simulator truth.
- Treating proposed solar/shadow methods as proven before ablation and qualification.

Force-controlled excavation and high-energy sintering remain gated research/tool variants. Meerkat
observation is in scope; unconstrained stunt-like motion is not.

## 13. Open Decisions and Required Data

1. Exact IPEx/LAC arm pivot geometry, limits, speed, brake behavior, and lift travel.
2. Exact camera intrinsics/extrinsics, including arm-mounted camera transforms.
3. Chassis, arm, wheel, empty-drum, and fill mass properties.
4. IPEx-scale drum geometry, scoop opening, and per-drum capacity.
5. Actuator power/efficiency and posture transition energy.
6. Authoritative sun-vector/ephemeris library and site/time frame convention.
7. Solar camera response: exposure, saturation, LED photometry, and dust degradation.
8. Whether NavLab components are adopted directly, reimplemented, or used only as benchmark baselines.
9. Preregistered improvement threshold for solar-navigation and Meerkat ablations.
10. First operational target: simulator-only LAC parity, terrestrial test site, or rover hardware.

No `[UNKNOWN]` item may be replaced by an undocumented guessed constant.

## 14. Legacy Crosswalk

| v5 area/stage | v6 destination |
|---|---|
| A, N1, N14 | Contracts/authority (`CT-*`) |
| B, R1, R3-R7, R14, P13-P16 | Navigation/execution (`NV-*`) |
| C, D, E, L | Terrain/world (`TW-*`) |
| F, P6, R6, R8-R9, P15/P17 | Perception/mapping (`PM-*`) |
| New solar-navigation work | Solar-terrain navigation (`SN-*`) |
| K6, R12, P19 and IPEx arm work | Vehicle/arm/posture (`VT-*`, `AM-*`) |
| H, I, J, P8-P10 | Construction planning (`CP-*`) |
| K2-K11 | Energy/power (`EP-*`) |
| MV | Fleet (`FL-*`) |
| M, N7-N18, O, P11 | Product/operations (`PO-*`) |
| P12 | Split between `PM-*`, `NV-*`, and simulated mode |
| P18 | Gated vehicle/force work (`VT-09`) |

## 15. Definition of Done

A requirement is done only when:

1. implementation is merged;
2. the advertised product path consumes it;
3. acceptance tests exercise success and failure behavior;
4. representative qualification evidence exists when required;
5. documentation states limitations and provenance;
6. no contradictory status remains elsewhere in the repository.

The PRD is the current requirement source. Historical test counts, screenshots, and stage narratives
must live in release/evidence records rather than being duplicated as present-tense product status.
