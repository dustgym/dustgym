# AGENTS.md — orientation for LLM agents working on foss_ipex

This file is the fast on-ramp for an AI agent. Read it first, then `PRD.md` (deep status / deliverable
stages) and the per-component READMEs. Honesty conventions here are **binding**: no synthetic data, no stubs, no
fabricated values; `[CALIB]`/`[UNKNOWN]`/`MEASURED` tags mark provenance and must stay accurate.

## What this is
A **lunar (and Mars/Earth) construction planner** on a sensor-faithful terramechanics core, lineage NASA
**IPEx / Lunar Autonomy Challenge**. John McCardle owns the simulator core; Aaron hosts/extends. Two
components, two granularity tiers:

Single repo, flat layout: `github.com/dustgym/dustgym`. Two components:

| Component | Path | What it is |
|---|---|---|
| **terrain_authority** (core) | `terrain_authority/` | Conserved Tier-2 terramechanics authority + RL envs + the **dustgym** Gymnasium suite + drum-mass sensing. NumPy-only hot path. |
| **planet_browser** (product) | `planet_browser/` | The SimCity-style build planner: CesiumJS globe + build-order queue → `mission_planner` → 2-3pp mission-control PDF. FastAPI-free (stdlib `http.server`); installed as a package (entry point `dustgym-serve`). |

**Two tiers:** the globe = *where to build* (planetary nav + coordinate pick, ~100 m imagery); the sim's
Haworth 5 m LOLA DEM = *how to build* (meter-scale). They are intentionally separate.

## Run it (from the repo root, with the runtime venv)
```bash
VENV=/mnt/projects/07_runtime_system/venv/bin/python
# the product (web app): then open the printed URL  (or use the `dustgym-serve` entry point)
PYTHONPATH=. $VENV -m planet_browser.server --host 0.0.0.0 --port 8770
# headless plan: writes reports/<stem>.pdf + .md
PYTHONPATH=. $VENV -m planet_browser.mission_planner
# the full test suite (MUST stay green before any commit/push)
PYTHONPATH=. $VENV -m pytest terrain_authority planet_browser -q
# the dustgym Gymnasium suite
PYTHONPATH=. $VENV -c "import dustgym, gymnasium as gym; print(gym.make('Dust/Scheduler-v0'))"
```
**Do NOT** launch a headless Chrome/Cesium WebGL screenshot casually — it can hang; if you must, use
Playwright with `--use-angle=swiftshader --enable-unsafe-swiftshader`, a hard `timeout`, and reap chrome.

## Architecture + the frozen seams (do not violate)
```
Project Chrono (physics authority, STUB live)  ->  Godot (render+sensor, separate proj)  ->  ROS2 (perception/SLAM eval)
        conserved NumPy Tier-2 authority is the working producer today
```
- **Single physics authority.** Mass is conserved *by construction*: agents/controllers COMMAND, the
  authority MUTATES the terrain. Never let a learned component write terrain directly.
- **Seam 1**: state fields on disk (heightmap/density/disturbance) — `INTERFACE.md`.
- **Seam 2**: `sensors.json` + PNGs for ROS2 — `docs/sensor_bridge_contract.md`.

## Key modules (terrain_authority/)
- `column_state.py` — the conserved column model (cut/`deposit_field`/`fill_toward`/`sinter`; mass-exact).
- `terramechanics.py` + `slip.py` — Bekker pressure-sinkage + slip-sinkage ladder (load-bearing via `physical=True`).
- `drive.py` / `rover.py` — closed-loop unicycle + 4-wheel pass; per-body gravity threaded (`g=`).
- `bodies.py` / `registration.py` — per-planet constants (sysrev-sourced) + the `Dust/*` env IDs.
- `rover_env`/`terrain_target_env`/`skill_env`/`scheduler_env`/`worksite_env` — the RL envs.
- `ipex_specs.py` — real IPEx energy/battery (NTRS 20240008162). `rassor_mass_model.py` — drum-mass sensing (NTRS 20210022781).

## Sinter is GATED
`column_state.sinter` + `WorkSite.sinter` are real + tested, but **gated off** via the single
`constants.SINTER_ENABLED` (energy/density are `[CALIB]`, not IPEx-grounded). Flip one line to enable.

## Web API (planet_browser/server.py)
- `POST /plan` {name, body, charger, orders[{action,kind:cut|fill,x,y,footprint_m2,depth_m}]} → report PDF URL + totals.
- `POST /sense` {true_mass_kg, noise_frac, capacity_kg} → drum-fill inference + offload decision.
- See `planet_browser/README.md` for the contract.

## Current state
Single repo `github.com/dustgym/dustgym` (flat layout; the old `roversim/` split is gone). CI runs the
suite on every push. dustgym is **not yet on PyPI** (publish is owner-gated). Forward plan + deep status
live in `PRD.md` (§8 deliverable stages).
