# dustgym — full architectural deep-code review (2026-06-05)

Method: 8 parallel review agents, each deep-reading every class/def in its subsystem and **run-verifying**
findings (executing tests, the live server, mypy) rather than asserting. Scope: all of `terrain_authority/`
+ `planet_browser/` source, the workflows + packaging, the PRD, and a forward mission-readiness analysis.
Baseline confirmed during the review: **701 tests pass, mypy clean (50 files), coverage 95.7%, ruff-F clean.**

No CRITICAL defects, no synthetic-data / stub / `NotImplementedError` / fake-return violations anywhere.
The conserved-physics core, the terrain/DEM pipeline, the RL envs, and the server are all genuinely
trustworthy + honestly tagged. The real findings are below, severity-ranked, with `file:line`.

## Real bugs (verified)

> **Resolution (2026-06-05, same day):** all five verified bugs below (2 HIGH, 2 MAJOR, 1 MED) are now
> **FIXED** with regression tests (706 tests pass, coverage 95.7%, mypy + ruff-F clean). Each is annotated
> ✅ FIXED inline. The MINORs and the mission-readiness gaps further down remain open.

### HIGH — the mission-control report silently mis-reports its headline numbers
- **Cut-only missions plan ZERO dig energy/time/trips.** `mission_planner.balance()` (≈:226) emits flows
  only for *fills*; a cut with no paired fill produces no trip, so excavation is invisible to the
  sequencer/simulator. Verified: a borrow-pit + solar-pad mission reports `n_trips=0, energy=0.0 MJ,
  time=0.0 h` while `cut_kg=25.6 t`. **4 of 8 `structures.py` templates are cut-only** (borrow_pit,
  haul_road, solar_pad, trench), so those missions under-report the *dominant* dig cost.
  ✅ **FIXED:** `balance` emits surplus (un-routed) cut mass as `(cut, None)` spoil-flows (symmetric to
  import flows); `_build_trips` turns each into a `kind="dig"` trip carrying the dig energy/time. The
  spoil-disposal haul to a dump is left as a disclosed unmodeled term (no spoil-site coordinate to
  fabricate). Live: a borrow pit now reports 99.7 MJ / 24 t spoil (was 0). Test
  `test_cut_only_mission_plans_the_dominant_dig_cost`.
- **`_simulate` `distance_m` excludes the haul-shuttle distance** (≈:416). It sums only inter-site
  `drive` legs; `haul_m` feeds time/energy but not `distance_m`. Verified: a plan with `haul_m=5300 m`
  reports `distance_m=600 m` (~9× under-report) and the **`distance` objective optimizes a quantity that
  omits the largest driving term.** ✅ **FIXED:** `distance_m = drive_m + Σ haul_m`. Live: a mixed pad+berm
  mission now reports 4833 m (incl. a 2533 m shuttle). Test `test_distance_m_includes_the_haul_shuttle`.

### MAJOR
- **`column_state.loose_mask` OR-logic** (≈:302) `not_paved | soft` flags any TREAD/COMPACTED_BERM cell
  below mid-density (1610) as LOOSE, so a *fresh single rut* slumps like virgin spoil under sandpile
  relaxation — contradicting the docstring ("TREAD/COMPACTED_BERM hold their slope"). Verified: 59% of
  TREAD cells flagged LOOSE after one pass. Masked by a thin test (`test_sandpile.py:65` only checks a
  fresh all-virgin scene). ✅ **FIXED:** `not_paved & soft` (confirmed: EXCAVATED retains ~RHO_SURFACE so
  it stays loose; TREAD/COMPACTED_BERM hold regardless of density; dense SINTERED now also correctly holds,
  which OR floated loose). Docstring corrected. Test `test_loose_mask_compacted_cells_hold_even_when_low_density`.
- **`registration.ROVER_BODIES` is a hardcoded list** (:28), not derived from `BODIES` by
  `bekker_regime`. Adding a gravity-loaded body per the one-entry-extensibility promise silently creates
  no `Dust/RoverDrive-<Body>-v0` ID, and the parametrized test derives *from* `ROVER_BODIES` so it can't
  catch it. ✅ **FIXED:** `ROVER_BODIES = [k for k,b in BODIES.items() if b.bekker_regime == "gravity-loaded"]`
  (derives to moon/mars/ceres/earth; bennu/phobos excluded). Test
  `test_rover_bodies_are_derived_from_the_registry_not_hardcoded`.

### MED
- **`_held_karp` returns `[]` on cyclic precedence** (≈:532), dropping all trips, whereas every other
  sequencer raises `ValueError`. `plan_and_simulate` guards via `_precedence_is_feasible`, but
  `optimize_sequence` is public (called by `autonomy.run_closed_loop`). ✅ **FIXED:** raises `ValueError` on
  `endj == -1`. Test `test_held_karp_raises_on_cyclic_precedence`.

### MINOR (selected; full list in the agent reports)
- `slip.developed_thrust` returns a tiny negative for slip < ~2e-9 (unreachable from the solver, but
  the public contract is "≥0"); `z_entrap_m` default ignores its own `contact_width_m` kwarg.
- `config.apply` lets an `int` constant become a `float` when overridden via a float-string (`N_WHEELS=4.0`).
- `rassor_mass_model.py:202` `g: float = None` implicit-Optional (masked by the mypy ratchet).
- `SINTER_HEAD_POWER_W` tagged `[SOURCED]` in `ipex_specs.py:98` but `[CALIB]` in `mission_planner.py:62` — reconcile.
- Server hardening (a `0.0.0.0`-capable service): `/dem` lacks an `isfile` guard (500 not 404 in a wheel);
  `_access_log` `by_route` metric is keyed on the attacker-controlled raw path (unbounded-dict DoS); no
  request-body size cap; API-key compare isn't constant-time; `GET /reports|/profiles` are unauthenticated
  even when `DUSTGYM_API_KEY` is set; `/structure` params are unvalidated. Path-traversal IS closed on all
  three file routes; CORS default is safe. None are critical; all are second-tier hardening.
- Dead/cosmetic: `SWELL_FACTOR` is dead (self-disclosed); stale docstrings (`material.py`, `carve_crater`
  "MASS-CONSISTENT" → "height-identity-consistent"); duplicate planner section header; `_plan_stem` local
  re-imports of json/re.
- Disclosed discretization artifacts (not bugs): even-window half-pixel crop offset (`dem_import`),
  oblique thin-occluder shadow miss (`illumination`).

## PRD vs code — the drift is stale-PESSIMISTIC (code is ahead of the PRD; no true overclaims)
- **N9 (CI gate)** marked ⬜ but `.github/workflows/ci.yml` runs ruff-F + mypy + pytest/coverage on a
  3.10-3.13 matrix; publish is gated on it. → done.
- **N13 (wheel packaging)** marked ⬜ but the wheel ships `planet_browser` + the `dustgym-serve` entry
  point. → done (residual: ~45 `sys.path` inserts).
- **AL1 / AL2** listed as open Phase-0 fixes but both are in code (`mission_planner.py:669` degradation
  `warnings.warn`; `:653` `_precedence_is_feasible` cyclic precheck raises). → done.
- **Test count "296"** (cited ~6×) is actually **701**.
- **Undocumented, implemented + tested:** the self-learning slip-energy loop (`self_optimizing.py` +
  `adaptive_planner.py`), the active-perception RL env (`active_perception_env.py`), the runtime tiled-LOD
  mosaic (`tiles_mosaic.py`). The PRD should track these.
- Stale memory note (not PRD): the "P6 map-channel = always-None slots / biggest gap" framing is stale —
  the scorer + onboard-stereo + COLMAP producers exist; the gap is **wiring the reward into the loop**.

## Mission-readiness — what it needs to actually conduct the mission
**P0 (blocks "conducts a real mission"):**
1. **Close the LAC §10 map-channel reward into the planning/autonomy loop** — the scorer + producers exist
   but the observed-map-vs-truth signal isn't fed back; "the robot perceives the scene it reshapes" is
   unrealized. The keystone. BUILD (assembly + the in-loop reward; dense MVS is CUDA-gated).
2. **As-built acceptance on the real DEM** — `validate_plan` gates siting on the real DEM but executes the
   mass/feasibility check on a *flat mantle*, so it can certify "mass conserved + site buildable" but NOT
   "pad built to ±2 cm." A construction planner must verify what it built. BUILD.
3. **Idle / heater / survival continuous power** — `_simulate` accumulates energy only over active
   drive/dig/lift legs; over a multi-day sortie survival load is plausibly dominant, so every energy /
   charge / endurance figure is an optimistic lower bound. BUILD (magnitude DATA-GATED → `[ASSUMPTION]`).

**P1:** multi-vehicle MV1-7 (the headline feature, currently `vehicles!=1: raise`); goal-level Mission
grammar + non-square footprints (J4/AL4-5, today a scalar→square footprint); op-window/deadline gating
(K8/K9/AL3); fault handling (AL7); drivetrain efficiency η (DATA-GATED `[ASSUMPTION]`).

**P2 / host- or data-gated:** live Chrono SCM oracle + Tier-3 forces (PyChrono-with-vehicle host),
render-in-loop throughput (GPU), AprilTag re-verify (container), N16/N17/N18 release/container/golden-file
baselines.

**Bottom line:** a genuinely trustworthy **single-rover** construction-planning core in a now-largely
production-grade shell — it loads a real lunar DEM, sites/sequences/balances/routes/validates build orders
on conserved physics with grounded IPEx energy, and emits the report; that flow is real, green, honest. It
is not yet "software that conducts a real mission" because the perception loop isn't closed, the planner
can't verify what it built, and the energy ledger omits the likely-dominant survival power — and
multi-vehicle is zero. Highest-leverage next builds: (1) close the map-channel reward, (2) as-built
acceptance on the real surface, (3) the energy-completeness pass.
