# dustgym architectural review — production-readiness (2026-06-04)

A six-agent architectural review of the single dustgym software (the conserved-physics core, the RL/Gymnasium
suite + packaging, the planner + web/API, the perception/render seams, the cross-cutting production
engineering, and the docs/PRD). Every finding is evidence-backed at `file:line`; the suites were run and the
hot paths benchmarked. There is only this software now; `roversim` is deprecated.

## Verdict

**Tier: high-quality research / pre-production code. Not yet production.** The scientific core is unusually
disciplined for research software: 296 tests green (terrain_authority 210 + planet_browser 86, + 26 in
scripts), deterministic seeded dynamics, mass conservation bit-exact (0.0 drift over 300 steps), sub-ms
authority step (0.391 ms verified), honest `[FIXED]/[CALIB]/[UNKNOWN]` tags with citations, no synthetic-data
shortcuts, clean license hygiene (CC0 core + attributed MIT mesh). The gap to production-grade is **not the
science and not the architecture — it is the operational shell**: no test/lint/type CI, no structured
logging, no externalized config, floor-only deps with no lockfile, a multi-threaded stdlib server with a
thread-safety bug and no auth/limits, the planner product absent from the wheel, and deprecated roversim
references throughout the PRD and docs. None of these are research gaps; they are a focused hardening
program.

## Findings by severity (consolidated)

### CRITICAL
- **No test/lint/type CI.** The only workflow (`.github/workflows/publish-dustgym.yml`) builds and publishes
  to PyPI without ever running `pytest`/`ruff`. A release can ship red. The numpy core + cv2 producer + schema
  validators all run GPU-free on an Ubuntu runner; the Godot/COLMAP/Chrono legs already skip-if-no-artifact.
  → a `ci.yml` running pytest (3.10-3.12 matrix) + ruff + env_checker, with publish gated on green.
- **Server: unbounded request body (DoS) + thread-unsafe report generation.** `server.py` reads
  `Content-Length` with no cap (negative or huge → memory exhaustion), and per-request PDF generation drives
  the global matplotlib pyplot interface under a `ThreadingHTTPServer` — concurrent `/plan` requests corrupt
  each other's figures, and content-hashed stems make two identical missions write the same path concurrently.
  → cap the body; move report generation to the matplotlib OO API + a lock/atomic-rename; (both fall out of
  the ASGI migration).

### HIGH
- **No structured logging.** 360 `print()`, 0 `logging` across the codebase; the server's `log_message` is a
  no-op and core fields silently degrade to `null` on any exception (`_autonomy_perception`) — the server is
  effectively unobservable. → a `logging` config, per-module loggers, request + error logging.
- **No externalized config.** 0 `os.environ`/`getenv`; all ports/paths/`[CALIB]` magnitudes are hardcoded
  literals + 43 argparse CLIs. → an env-overridable config layer (host/port/report-dir/DEM-bundle/CALIB knobs).
- **Dependency pinning is floor-only, no lockfile, no ceilings.** `gymnasium>=0.29` (the most breakage-prone
  dep) has no upper bound; `torch` unpinned. → a committed lockfile + version ceilings + a tested gymnasium range.
- **Packaging: the planner/server product is not in the wheel.** `packages = ["terrain_authority","dustgym"]`
  excludes `planet_browser/` (the mission-control product), which is also why there are 51 `sys.path.insert`
  hacks. → add `planet_browser` + a server console entry point, or scope the wheel to the gym suite and say so.
- **No auth on the server**, `0.0.0.0` bind is a first-class option, and `/render` shells out to Godot twice
  per call unauthenticated. → token-gate mutating routes when bound off-localhost; rate-limit `/render`.
- **Conservation + invariants are enforced only in tests, never at runtime.** The headline guarantee has zero
  runtime guard (the only non-test `assert`s are stripped under `python -O`). → a `check_invariants()` /
  `conserves_mass()` guard (not bare `assert`), CI-gated.
- **`Dust/WorkSite-v0` ships synthetic terrain in its registered default.** It is the only registered env
  whose default terrain is `rng.random` bumps (the documented results use a real Haworth bundle `pip install`
  cannot supply). → register it on procgen/real terrain or gate the synthetic path behind a non-default kwarg.
- **Seam validation is asymmetric** — the consumer of the sensor-bridge seam validates `schema_version` +
  `frame_convention`; the producers (`obs_map_producer`, `colmap_*`) read the same JSON unguarded and fail with
  a deep `KeyError`/`StopIteration`. → one shared validating reader + committed JSON Schemas + a fixture test.

### MEDIUM (selected)
- **Public physics constructors do not validate input** (`ColumnState(8,8,-0.02)` accepted; `density=0` →
  silent `inf`). → validate dims/cell-size/positive-density in `__post_init__`.
- **Renderer far-plane hardcoded 100 m** silently clips the 10 km tile to black. → a `--far` flag + a
  scene-extent-vs-far warning.
- **Findings live only in figure scripts**, not regression tests (the Hapke<Lambert 33% result). → pin as a
  monotone-inequality test.
- **Training scripts hardcode `/mnt/...` and `/tmp` paths**; only 2 of 6 persist a checkpoint.
- **No `[tool.ruff]`/`[tool.mypy]`/pytest config committed**; 113 `noqa`, no `py.typed`, no pre-commit.
- **`reports/` grows unbounded and is HTTP-readable by basename** (no TTL/quota/scoping).
- **`ActivePerceptionEnv`** returns a 4-tuple without gym (siblings return 5) and calls `reset()` in `__init__`.
- **Generated render artifacts tracked in git** (`out/`, `godot_sidecar/out/*.png`).
- **`slip.developed_thrust`** returns a tiny negative near zero slip (latent sign bug below the 1e-6 floor).
- **`SWELL_FACTOR=1.2`** is dead but its docstring claims it is load-bearing.

### Strengths worth preserving (do not "fix")
- The conserved-authority layering, the honesty tags, determinism, and the no-synthetic-data discipline.
- Error handling is deliberate graceful-degradation (no bare `except`, no `eval`/`exec`/`pickle`/`shell=True`);
  `subprocess` uses list-args + timeouts. Path-traversal on `/reports/` and `/dem` is correctly blocked.
- The planner is a clean strategy pattern (7 sequencers, SOP-precedence-correct across all of them) with a
  real authority-validation pass, not a stub. The two on-disk seams are rigorously specified.

## Production-readiness roadmap (ordered)

1. **CI gate (P0).** `ci.yml`: ruff + pytest matrix (3.10-3.12) + strict env_checker on all 10 `Dust/*` IDs;
   register pytest markers for the gated tiers (`gpu`/`godot`/`colmap`/`chrono`); branch protection; publish
   `needs:` green CI.
2. **Quality config (P0).** Commit `[tool.ruff]` + `[tool.mypy]` + `[tool.pytest.ini_options]`, `.pre-commit`,
   `py.typed`; ratchet the 113 `noqa` down.
3. **Runtime invariants + input validation (P1).** `check_invariants()`/`conserves_mass()`; validate the public
   physics + env constructors.
4. **Structured logging + config (P1).** `logging` everywhere the library/server currently `print`s; an
   env-overridable config layer; request + error logging on the server.
5. **Dependency hygiene (P1).** Lockfile + ceilings + a tested gymnasium range; `pip-audit` in CI.
6. **ASGI server + hardening (P1).** FastAPI/uvicorn replacing the stdlib server: Pydantic request/response
   models (= the API contract + input limits), auth on mutating routes, CORS policy, `/healthz` + `/metrics`,
   thread/async-safe OO-matplotlib report generation, `reports/` TTL/quota.
7. **Packaging completeness (P1).** Add `planet_browser` + server entry point to the wheel; delete the 51
   `sys.path` hacks; exclude tests from the wheel; expose `__version__`.
8. **Seam + render hardening (P1).** Shared validating seam reader + JSON Schemas + fixture tests; renderer
   `--far` + scale-guard; pin the science findings as regression tests; commit tiny real render fixtures so the
   skip-gated producer/COLMAP/AprilTag tests run in CI.
9. **Repo + release hygiene (P2).** gitignore generated render outputs; `CHANGELOG.md`; SemVer release flow;
   golden-file regression on planner totals + the AprilTag/map-channel baselines.
10. **roversim purge (P0, mechanical).** Remove every `roversim` path, dual-tree resolver, user-facing string
    (`mission_planner.py` multi-vehicle raise message ships to users), and PR reference from code, PRD, AGENTS,
    and docs. There is only this software.

## Docs gaps for production
Missing: `CHANGELOG.md`, `CONTRIBUTING.md`, `docs/api.md` (the HTTP contract), `docs/architecture.md`
(consolidated, flat-monorepo-correct), `docs/deployment.md`, `SECURITY.md`. `AGENTS.md` is the most
roversim-saturated file (wrong `cd roversim`, "190 passing", PR #7) and needs a rewrite to the 296-test
single-software reality.

*Per-subsystem detail (the six full reviews) is on file. This document + the updated PRD are the actionable
synthesis.*
