# DustGym Full Architectural Review

**Review date:** 2026-06-06
**Reviewed commit:** `047331250cf443498c25b5bead4bed167668752c`
**Repository:** `/mnt/projects/foss_ipex/dustgym`

## Executive verdict

DustGym has a credible simulation core, unusually strong source-level test coverage, a
real Godot rendering path, and several useful explicit contracts. It is not yet a
coherent end-to-end rover execution system. The main architectural problem is that
planning, simulation, validation, autonomy, rendering, and API response generation
recompute related state through separate paths instead of consuming one authoritative
plan artifact. This already produces contradictory fleet results.

The current commit also fails its configured test suite, accepts negative physical
quantities, emits incorrect multi-vehicle Plan IR distances, and advertises selectable
vehicle physics that are only partially wired. Those are release-blocking correctness
issues.

This review is complete for the static repository scope defined below. “100%” does not
mean mathematical proof or qualification of integrations that require unavailable
hardware and external systems. It means every tracked file was inventoried, every
tracked code category was reviewed or mechanically checked, all locally runnable
quality gates were exercised, and unverified boundaries are explicitly listed.

## Scope and method

The reviewed tree contains 562 tracked files:

| Category | Count |
|---|---:|
| Python | 187 |
| Godot GDScript | 13 |
| Godot shaders | 7 |
| Browser HTML/JavaScript | 1 |
| Shell | 2 |
| YAML | 6 |
| Dockerfiles | 2 |

The review covered:

- `terrain_authority`: state authority, dynamics, environments, terrain I/O, vehicle
  geometry, registration, invariants, and package interfaces.
- `planet_browser`: planner, API, Plan IR, autonomy, localization, perception, reports,
  browser UI, generated body data, and persistence.
- `godot_sidecar`: scene import, CLI, rendering, state-field consumption, shaders, and
  output handling.
- `scripts`: mesh generation, rendering pipeline, ROS 2 bridge, evaluation, Chrono
  integration, and tests.
- Build metadata, wheels/sdists, CI and publishing workflows, Docker configuration,
  security posture, documentation claims, generated artifacts, and repository layout.

Methods included Python compilation, Ruff, mypy, pytest/coverage, package builds,
`twine check`, shell syntax checks, JavaScript syntax checks, Godot editor import, an
actual headless Godot render, targeted API/domain probes, and focused source inspection.

## Release-blocking findings

### F-01: Negative physical quantities are accepted

**Severity:** High
**Area:** API, mission model, terrain authority

`planet_browser/server.py:199-206` permits order depths down to `-100`. The public
dataclass/API path in `planet_browser/mission_planner.py:108-202` does not consistently
enforce positive, finite footprint, depth, or coordinate values.

A live `/plan` request with a cut depth of `-0.1 m` returned HTTP 200 and a successful
plan containing negative cut mass. Negative mass is also accepted by
`ColumnState.cut_to_inventory()` and `dump_from_inventory()` in
`terrain_authority/column_state.py:173-212`, allowing operations to reverse their
physical meaning.

`ColumnState.__post_init__()` at `terrain_authority/column_state.py:78-83` validates
grid dimensions but not array shape, finite values, positive density, state domains, or
nonnegative drum inventory. Direct construction accepted a wrong-shaped mass array,
all-NaN density, and negative inventory.

**Impact:** Invalid requests and direct API calls can create physically impossible
plans and state. Invariants are optional diagnostics rather than enforced boundaries.

**Required change:** Introduce shared finite-and-positive domain validators at every
public construction/mutation boundary. Make state construction reject shape/domain
violations and make mutations transactional with invariant checks.

### F-02: The configured test suite is red while CI excludes the failure

**Severity:** High
**Area:** CI, dependency management

`pyproject.toml:67-69` configures pytest to collect `terrain_authority`,
`planet_browser`, and `scripts`. `scripts/test_gen_ipex_mesh.py:12` imports `trimesh`,
as does the production generator at `scripts/gen_ipex_mesh.py:25`, but `trimesh` is not
declared in `pyproject.toml`.

`python -m pytest scripts -q` fails during collection with
`ModuleNotFoundError: No module named 'trimesh'`.

Both `.github/workflows/ci.yml:31-32` and
`.github/workflows/publish-dustgym.yml:32-33` explicitly run only
`pytest terrain_authority planet_browser`, bypassing the configured `scripts` test
path. A release can therefore pass and publish while the repository’s declared test
suite cannot collect.

**Required change:** Declare a mesh-generation/test extra and install it in CI, or move
the test behind an explicit marker with a documented gate. CI should invoke the
configured suite instead of maintaining a narrower duplicated path list.

### F-03: Multi-vehicle API responses combine incompatible single- and fleet plans

**Severity:** High
**Area:** API architecture, fleet planning

`planet_browser/server.py:464-473` passes `vehicles` to `run()` and `plan_ir()`, but
calls `validate_plan()`, `build_timeline()`, endurance, and autonomy paths without the
fleet count.

A two-vehicle request returned:

- fleet totals and Plan IR tagged with two vehicles;
- a `16,523.8 s` fleet makespan;
- a single-rover timeline lasting `33,047.6 s`;
- no vehicle identity in timeline frames;
- a single-rover autonomy/perception result.

The browser playback around `planet_browser/index.html:821` also renders one rover
marker.

The README’s “space-time deconfliction” wording overstates the implementation.
`planet_browser/mission_planner.py:741-752` explicitly excludes shared-charger
contention, continuous path collision avoidance, and cross-vehicle precedence.

**Required change:** Build one immutable `PlanResult` containing trips, allocation,
timeline, validation inputs, Plan IR, and derived summaries. All API products must be
views over that same result. Fleet identity must be present throughout the timeline,
autonomy, validation, and UI.

### F-04: Multi-vehicle Plan IR computes routes from the wrong vehicle position

**Severity:** High
**Area:** Plan IR, execution contract

`planet_browser/mission_planner.py:874-905` uses one `prev` position while iterating
trips flattened across all vehicles. It does not maintain a position per vehicle.

In a verified two-vehicle plan, vehicle 1’s first `GoTo` was emitted as `121.07 m`
because it started from vehicle 0’s last destination. Its actual charger-to-site
distance was `40.31 m`.

Existing coverage at `planet_browser/test_mission_planner.py:716-720` verifies only
that both vehicle IDs appear, not that each route has correct geometry, duration, or
energy.

**Impact:** An executive consuming the Plan IR receives incorrect per-action
expectations and tolerance baselines.

**Required change:** Track `prev_by_vehicle`, explicitly model start/recharge
locations, and add per-vehicle route-ledger tests.

### F-05: Vehicle selection is not wired through vehicle physics

**Severity:** High
**Area:** vehicle model, dynamics, planner claims

`terrain_authority/rover_env.py:111-120` uses the selected vehicle primarily for
stability geometry. Drive energy and traction still use global defaults in
`terrain_authority/drive.py:49-83`; contact geometry still uses global rover geometry
in `terrain_authority/rover.py:188`; planner mass, battery, drum, and drive-energy
calculations remain global constants.

Targeted runs with IPEx and EZ-RASSOR produced identical planner totals and identical
telemetry on a 5-degree slope.

This contradicts the current commit message’s “per-vehicle physics, wired through every
stage” claim. `PRD.md:327` partly acknowledges that planner results still do not change.

**Required change:** Pass a typed `VehicleModel` through dynamics, contact geometry,
terramechanics, energy, capacity, planner simulation, and Plan IR. Add cross-vehicle
tests that assert expected numerical differences.

### F-06: The installed server product has an invalid dependency and filesystem model

**Severity:** High
**Area:** packaging, deployment

`pyproject.toml:46-48` always installs the `dustgym-serve` console entry point, while
FastAPI/uvicorn and planner dependencies are optional at `pyproject.toml:31-34`.
`planet_browser/mission_planner.py:31-34` imports matplotlib at module import time, so
the server extra alone is also insufficient.

The server writes reports and profiles inside its installed package directory
(`planet_browser/server.py:68-70`). The planner also writes there. This fails in normal
read-only system/site-package deployments.

The wheel excludes source-only scripts and large terrain/render assets used by
`planet_browser/server.py:58-66`. Rendering therefore degrades to 503, while Moon DEM
loading can silently fall back to flat terrain at `planet_browser/server.py:117-129`.

The wheel unexpectedly contains 61 test modules despite `PRD.md:298` claiming tests
are excluded.

**Required change:** Define a coherent `server` product extra including planner
dependencies, move mutable data to an explicit application-data directory, package or
download versioned runtime assets, and make missing real terrain an explicit mode or
error. Exclude tests intentionally if that remains the product policy.

## Major architectural findings

### F-07: Planning is recomputed across loosely coupled product paths

**Severity:** Medium-High

`planet_browser/mission_planner.py` is approximately 1,780 lines and combines domain
parsing, terrain loading, path optimization, battery simulation, validation, timeline
generation, Plan IR lowering, and report rendering. `post_plan()` invokes several of
these paths independently.

The result is excessive CPU cost and contract drift: fleet allocation, sequencing,
timing, and energy need not be the same plan across totals, validation, autonomy,
timeline, and Plan IR.

**Recommendation:** Split domain input, terrain context, optimization, simulation, and
presentation into explicit layers. The optimizer should produce one versioned plan
model consumed by all downstream adapters.

### F-08: The autonomy layer is a simulator, not a closed execution loop

**Severity:** Medium-High

`planet_browser/autonomy.py:119-151` simulates legs and explicitly omits return-to-site
drive energy. At `planet_browser/autonomy.py:203-207`, the localization update fuses
the simulated true pose directly. `planet_browser/map_channel.py:9-17` derives
observability from truth and station distance rather than a reconstructed live map.

`scripts/ros2_bridge/eval_harness.py:9-14` defaults to synthetic data and implements
only that mode. Live timestamp association in `scripts/ros2_bridge/score_pose.py:125-137`
raises `NotImplementedError`.

**Impact:** “Closed loop,” “executable,” and autonomy acceptance language should not be
interpreted as flight/field execution readiness.

**Recommendation:** Separate `SimulatedExecutive` from a real executive interface.
Require timestamped sensor observations, estimator outputs, command acknowledgements,
fault states, and a complete energy ledger before calling the path closed-loop.

### F-09: Scene publication is not atomic and readers validate too little

**Severity:** Medium-High

`terrain_authority/io_fields.py:12-13` and `:58-66` write metadata before raster data.
That makes metadata visible before the referenced state is complete, the reverse of a
safe commit-marker protocol. Files are not atomically renamed and have no checksums.

`load_scene()` at `terrain_authority/io_fields.py:69-87` accepts missing fields and does
not enforce schema version, ordering, dtype, dimensions, finiteness, or physical value
domains. `godot_sidecar/state_fields.gd:120-188` performs useful parse/grid checks but
also omits several contract-level validations.

**Recommendation:** Write all rasters to temporary files, validate/checksum them,
atomically rename them, and publish metadata last. Centralize a strict schema validator
used by Python, Godot, and ROS adapters.

### F-10: ROS boundary checks disappear under optimized Python

**Severity:** Medium-High

`scripts/ros2_bridge/bag_writer.py:57-75`, `:238-245`, and `:320-323` use `assert` for
PNG, schema, frame, and dimension validation. Python removes these checks under `-O`.
A direct optimized-mode probe accepted an invalid schema version.

Production assertions also exist in `terrain_authority/worksite.py:81,121` and
`terrain_authority/refinement.py:524`.

**Recommendation:** Replace input/contract assertions with explicit typed exceptions.
Reserve assertions for impossible internal states.

### F-11: Render failures can be reported as success

**Severity:** Medium-High

`scripts/plan_render_pipeline.py:79-95` ignores subprocess return status and stderr,
then returns the expected output path without verifying it exists. A probe using
`/bin/false` returned a nonexistent output path without raising.

The pipeline uses fixed intermediate paths at
`scripts/plan_render_pipeline.py:169-170`, allowing stale output reuse and collisions.
The server lock is process-local and does not protect multi-worker deployments.

`godot_sidecar/sidecar.gd:266-270` similarly ignores the boolean result of `save_png`
for one render path and can print success before exiting zero.

**Recommendation:** Use per-request temporary directories, check every return code and
output, capture stderr, and atomically publish only verified artifacts.

### F-12: Generated configuration has two authorities and import-time side effects

**Severity:** Medium

`planet_browser/gen_bodies_json.py:19-68` executes generation and writes
`bodies.json` during import. Importing the module can mutate an installation or fail in
a read-only package.

The planner imports the Python body registry but reads generated JSON at runtime in
`planet_browser/mission_planner.py:92-106`, allowing the generated copy to diverge from
the claimed single source of truth.

**Recommendation:** Make generation an explicit build command with a `main` guard.
Runtime Python should consume the registry directly; generated JSON should be a tested
browser artifact with a source hash/version.

### F-13: Server resource controls are deployment-unsafe

**Severity:** Medium

Authentication is disabled when `DUSTGYM_API_KEY` is absent
(`planet_browser/server.py:255-263`) and CORS defaults to `*` at `:269-274`. This is
reasonable for loopback development but unsafe if launched on `0.0.0.0`, as suggested
by `AGENTS.md:24-27`.

The request-size middleware at `planet_browser/server.py:277-288` trusts
`Content-Length`; chunked or missing-length requests bypass the early guard. CPU-heavy
planning/rendering has no rate, concurrency, or execution-time limits. Profiles are
unbounded and written non-atomically. Locks and fixed paths are process-local.

**Recommendation:** Separate dev and deployment profiles. Stream-enforce request
limits, require auth for non-loopback binding, use bounded worker queues, move state to
external storage, and define retention for every artifact type.

### F-14: Dependency and container builds are not reproducible

**Severity:** Medium

There is no project lockfile. Runtime and optional dependency ranges are broad.
`scripts/ros2_bridge/Dockerfile` uses an unpinned ROS base tag, unpinned apt packages,
and a ranged pip dependency. The image is not built by CI and does not define a
non-root runtime user.

A trustworthy project vulnerability inventory cannot be produced without a resolved
project environment. A scan of the shared workstation found vulnerable packages, but
that result cannot be attributed to DustGym and is not a substitute for an audited
lock.

**Recommendation:** Generate platform-specific locked environments, pin container base
digests, produce an SBOM, scan the resolved artifact in CI, and define an update policy.

## Correctness and scientific-model findings

### F-15: Stereo uncertainty can become unjustifiably confident

**Severity:** Medium

`scripts/ros2_bridge/obs_map_producer.py:136-174` treats dense pixels from one stereo
observation as independent samples and divides by `sqrt(n)`. Correlated pixels can
therefore drive reported uncertainty arbitrarily low. `dig_ready_mask()` at `:189-193`
can mark a cell ready based on that optimistic estimate.

This is an inference from the estimator implementation, not a calibration result.

**Recommendation:** Use an effective sample count/correlation model, preserve a
calibrated sensor/model floor, and validate coverage thresholds against held-out
physical data.

### F-16: Empty valid disparity sets are not handled

**Severity:** Medium-Low

`scripts/ros2_bridge/depth_map.py:63-65` computes min/median/max over valid disparity
values without handling an empty set, causing an exception after partial work.

### F-17: Godot CLI argument and output contracts are weak

**Severity:** Medium-Low

`godot_sidecar/sidecar.gd:471-573` increments argument indices without consistently
checking that a value follows. Unknown arguments warn rather than fail, allowing typos
to produce unintended defaults. Absolute output paths are unrestricted by
`sidecar.gd:575-579`.

### F-18: Plan IR is descriptive rather than fully executable

**Severity:** Medium

The IR emits typed work actions but represents recharge only as a battery precondition;
there is no positional recharge action, command protocol, acknowledgement model,
retry/fault transition, or ROS lowering in the repository. The schema is useful, but
“machine-executable” overstates the implemented integration.

### F-19: Browser runtime depends on unpinned third-party scripts

**Severity:** Medium-Low

`planet_browser/index.html:7-8` loads Cesium from `unpkg.com` without subresource
integrity or a content-security policy. This adds network availability and supply-chain
risk to mission-control startup. The page also uses several `innerHTML` sinks; current
inputs are mostly generated/local, but a CSP and safer DOM construction would reduce
future exposure.

## Repository and maintainability findings

### F-20: Large generated artifacts are committed

**Severity:** Medium

Tracked generated outputs under `godot_sidecar/out`, `viz/out`, `out`, and
`validation` total roughly 109 MB. The tracked working content is roughly 194 MB and
the Git pack is about 371 MB.

This increases clone, checkout, backup, and history maintenance costs. Validation
evidence should be versioned as release artifacts or in object storage with manifests,
not accumulated in the primary source history.

### F-21: Multiple divergent repository copies create authority risk

**Severity:** Medium

The surrounding workspace contains active copies named `dustgym`, `roversim`,
`dustgym_repo`, and a root `planet_browser`, at different commits. This is outside the
reviewed repository’s code but is an operational architecture risk: fixes and evidence
can easily target the wrong tree.

**Recommendation:** Declare one canonical repository/remote, archive obsolete copies,
and make reports include commit and dirty-state provenance.

### F-22: Type and coverage gates omit important surfaces

**Severity:** Medium

The existing coverage data reports about 96.2% for `terrain_authority` and
`planet_browser`, which is strong. It excludes scripts, Godot, shaders, and browser
JavaScript. The mypy configuration at `pyproject.toml:104-125` ignores errors in a
large set of core modules, including state and environment code.

The numerical coverage percentage is therefore not whole-product coverage, and a
green mypy run is a partial ratchet rather than end-to-end type correctness.

### F-23: Central modules carry too many responsibilities

**Severity:** Medium

High-complexity files include:

- `planet_browser/mission_planner.py`: planning, simulation, terrain, validation,
  reports, and Plan IR.
- `godot_sidecar/sidecar.gd`: CLI parsing, scene construction, import, camera,
  rendering, and output.
- `terrain_authority/scenes.py`: broad scene generation and configuration.
- `planet_browser/server.py`: API, persistence, rendering orchestration, and static
  delivery.
- `planet_browser/index.html`: UI, state, networking, calculations, and rendering in
  one file.

These are not defects by size alone, but current cross-feature regressions show that
the responsibility boundaries are no longer containing change.

## Documentation and claim audit

The following claims do not match the reviewed commit:

| Claim | Actual implementation |
|---|---|
| `AGENTS.md:17`: FastAPI-free stdlib server | Server is FastAPI/uvicorn. |
| `AGENTS.md:4-5` and `docs/index.md:75`: no synthetic data/stubs | Default synthetic worksite, synthetic-only eval harness, Chrono placeholder exporter, shader stub, and test import stubs exist. |
| README multi-vehicle space-time deconfliction | Only site-exclusive allocation is checked; shared charger, path collisions, and cross-vehicle precedence are excluded. |
| Current commit: per-vehicle physics wired through every stage | Vehicle-specific dynamics and planner constants are not wired end to end. |
| `PRD.md:298`: tests excluded from wheel | The built wheel contains 61 test modules. |
| `PRD.md:294`: CI/markers gate all tiers | CI explicitly runs only two Python package paths. |
| `PRD.md:604`: stated passing test count | The configured current suite fails collection. |
| `docs/spec_coverage.md`: current coverage snapshot | It references an older commit and misses newer map-channel work. |

The documentation set also contains mutually inconsistent older architectural reviews.
Architecture status should be generated from one current capability matrix tied to a
commit, with “implemented,” “simulated,” “stubbed,” “hardware-gated,” and “validated”
as distinct states.

## Architectural strengths

The review found substantial foundations worth preserving:

- The terrain-authority/consumer split is clear and generally respected.
- Mass-per-area state is an appropriate conserved representation.
- Seeded dynamics and controlled fixtures support reproducibility.
- Core Python source coverage is high.
- Pydantic request bounds, path sanitization, and capability gating exist.
- Plan IR is versioned, typed, deterministic, and carries useful expectations.
- Body/soil/vehicle registries are moving toward explicit configuration.
- Godot imports and renders the real project successfully in headless mode.
- Terrain provenance and limitations are often documented directly in code.
- Planner optimality labels distinguish exact and heuristic methods in several paths.

These strengths make the project repairable without a rewrite. The priority is to make
the existing contracts authoritative and consistent rather than add more features.

## Recommended remediation order

### P0: Before another release

1. Reject non-finite/nonpositive physical inputs and enforce state invariants.
2. Fix `prev_by_vehicle` in Plan IR and add geometry/energy ledger tests.
3. Replace repeated `/plan` computations with one authoritative plan result.
4. Restore a collectable configured test suite and make CI run it.
5. Correct package extras, mutable storage, and real-terrain failure behavior.
6. Reword vehicle/fleet/autonomy claims to match implemented scope.

### P1: Architecture stabilization

1. Introduce typed `MissionInput`, `TerrainContext`, `VehicleModel`, `PlanResult`, and
   `ExecutionResult` boundaries.
2. Make scene publication atomic and schema validation shared.
3. Replace production assertions with explicit validation errors.
4. Harden render subprocess and output contracts.
5. Separate simulated autonomy from live integration interfaces.
6. Lock dependencies and build/test the ROS container in CI.

### P2: Product maturity

1. Implement fleet charger/path coordination and per-vehicle UI playback.
2. Complete timestamped localization/map integration and ROS command lowering.
3. Calibrate perception uncertainty against physical datasets.
4. Split central modules after stable contracts are in place.
5. Move generated evidence out of Git history and consolidate repository copies.

## Verification record

| Check | Result |
|---|---|
| Git state at review start | Clean, commit recorded above |
| Python compilation | Passed |
| Ruff | Passed |
| Mypy configured scope | Passed, with documented ignored modules |
| Configured pytest scope | Failed collection: undeclared `trimesh` |
| Core CI pytest scope | 716 collected; 715 passed locally, 1 failed because this workstation lacks `pyproj` (included by the declared `dev` extra) |
| Existing Python coverage | Approximately 96.2% for the two configured source packages |
| Wheel and sdist build | Passed |
| `twine check` | Passed |
| Wheel content inspection | 126 files, including 61 test modules |
| Shell syntax | Passed for all tracked shell scripts |
| Browser inline JavaScript syntax | Passed with Node |
| Godot editor import | Passed with Godot 4.6.3 |
| Godot headless render | Passed; real PNG produced |
| Multi-vehicle Plan IR probe | Failed correctness: cross-vehicle position leakage |
| Negative-depth API probe | Invalid physical request accepted as success |
| Optimized ROS schema probe | Invalid schema accepted because assertions were removed |
| Render failure probe | Failed correctness: nonexistent result returned without error |

## Unverified and hardware-gated boundaries

The following were inspected statically but cannot be represented as end-to-end
validated by this workstation review:

- ROS 2 bag production and playback in the declared container.
- Physical camera/stereo calibration and live timestamp synchronization.
- COLMAP reconstruction on a representative real mission dataset.
- Chrono/SCM soil coupling and comparison against physical soil tests.
- Actual rover command execution, fault recovery, and communication loss.
- Multi-process production deployment behind a reverse proxy.
- Cross-platform package installation on every supported Python/OS combination.
- Long-duration memory, disk-retention, and concurrency testing.

These are explicit qualification tasks, not evidence that the corresponding path is
broken. They must remain labeled unverified until exercised in controlled environments.

## Final assessment

DustGym is currently best classified as a strong research simulation and planning
prototype with a partially integrated visualization and API product. It is not ready
to be treated as a physically authoritative, fleet-capable, installable rover execution
stack.

The highest-value architectural move is to establish one validated plan/state contract
and force every consumer to use it. That change addresses the fleet contradictions,
reduces repeated computation, makes vehicle-specific physics testable, and provides a
stable boundary for ROS, Godot, reports, and browser playback.
