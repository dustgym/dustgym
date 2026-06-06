# dustgym

[![CI](https://github.com/dustgym/dustgym/actions/workflows/ci.yml/badge.svg)](https://github.com/dustgym/dustgym/actions/workflows/ci.yml)
[![License: CC0-1.0](https://img.shields.io/badge/license-CC0--1.0-lightgrey.svg)](LICENSE)
[![Python](https://img.shields.io/badge/python-3.10%2B-blue.svg)](pyproject.toml)
[![Docs](https://img.shields.io/badge/docs-dustgym.github.io-informational)](https://dustgym.github.io/dustgym/)

**A conserved-physics world model, autonomous planner, and Gymnasium suite for off-world surface
construction.** A robot that *reshapes* regolith (excavate, grade, berm, fill) on an airless rocky
surface, on a mass-conserving terramechanics authority parameterized per planetary body (Moon / Mars /
Ceres / Bennu / Phobos / Earth). *Dust* = the regolith every airless surface shares. Lineage: NASA IPEx
(ISRU Pilot Excavator) and the Lunar Autonomy Challenge.

The design call, in one line: **conserved physics for the dynamics (exact, unhackable) plus a learned
model only for the expensive perception branch.** Where most of the field learns everything at
fleet scale and wraps a safety case around it, dustgym makes the dynamics provable and spends its
learning budget only where observation is genuinely expensive. See [`docs/related_work.md`](docs/related_work.md)
for where this lands in NASA autonomy, lunar mining, world models, autonomous driving, and SLAM.

---

## Install

```bash
pip install dustgym        # deps: numpy, scipy, gymnasium; extras: [rl] = torch + stable-baselines3
```

Or from source (editable), which also exposes the conserved authority directly:

```bash
git clone https://github.com/dustgym/dustgym && cd dustgym
pip install -e .[rl]
python -c "from terrain_authority import world_model; print(world_model.describe())"
```

The installed package is `terrain_authority` (the conserved authority + world model) and `dustgym`
(the Gymnasium registration shim). The mission planner and web UI live alongside in `planet_browser/`
(run from source).

---

## The five-layer world model

For a robot that transforms terrain, the world model ties perception, physics, planning, and control
together; the state transition is "robot reshapes terrain into infrastructure," not "robot reaches a
waypoint." Full design: [`docs/world_model.md`](docs/world_model.md).

| Layer | What it is | Where |
|---|---|---|
| **Geometry** | height / slope / roughness; current vs target surface; earthwork volume | `column_state`, real LOLA Haworth DEM ingest, `mission_planner` |
| **Material** | per-cell density → friction / cohesion / bearing; cut-difficulty + slip maps | `material.py` (threaded into the solver) |
| **Physics** | the conserved transition `S(t+1)=f(S,a)`: Bekker sinkage, slip ladder, mass-exact cut/haul/dump, IPEx energy at lunar g | the Tier-2 authority (exact) |
| **Task** | target heightmap; cut / fill / transport; 8 volume-balanced structures | `mission_planner`, `structures.py`, `terrain_target_env` |
| **Uncertainty** | terrain / material / localization confidence; per-cell height sigma | `autonomy.py` (Kalman), the map channel |

The transition is **computed, not predicted** — when asked what a 15 cm cut over a 1.5 m pass does, the
authority returns removed volume (mass-conserved), energy (grounded in IPEx specs), and slip risk (the
slip ladder) by computing them. Mass conservation is enforced by construction, so the terrain-matching
reward cannot be gamed.

---

## Gymnasium suite (all pass `gymnasium.utils.env_checker`)

```python
import dustgym                       # registers the Dust/* envs on import
import gymnasium as gym
env = gym.make("Dust/RoverDrive-Mars-v0")    # per-body physics (gravity + Lyasko-corrected regolith)
obs, info = env.reset(seed=0)
obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
```

| ID | task | action / obs |
|----|------|------|
| `Dust/RoverDrive-v0` · `-Moon`/`-Mars`/`-Ceres`/`-Earth-v0` | closed-loop drive over terramechanics (slip, sinkage) | `Box(2)` / `Box(32)` |
| `Dust/Construct-v0` | cut/fill to a target heightmap (terrain-matching reward) | `Box(3)` / `Box(104)` |
| `Dust/SkillMacro-v0` | skill-macro construction: pick a cell + cut/dump toward target | `Discrete(128)` / `Box(68)` |
| `Dust/Scheduler-v0` | multi-objective scheduling: borrow pits → build sites, one rover/drum | `Discrete(5)` / `Box(12)` |
| `Dust/WorkSite-v0` | RL controller over the streaming WorkSite seam (flatten pad + build berm) | `Discrete(2)` / `Box(4)` |
| `Dust/ActivePerception-v0` | next-best-view mapping: drive to reduce per-cell uncertainty per joule | `Discrete` / `Box` |

The authority is exact, deterministic, mass-conserving, and sub-millisecond — both the simulator *and*
the reward source — so learned/searched policies only **command** while the authority **mutates**.

**Honest RL findings (where learning earns its keep).** Single-objective construction is
physics-bounded: at grounded IPEx energy ratios a greedy or model-based planner already solves
flatten/berm, and PPO ties or loses. RL/ML planning earns its keep in the **multi-objective scheduling**
layer (build A+B+C under one battery, with precedence) — there a search-distilled policy reaches the
24-leg optimum that greedy (28) and PPO (27) miss. And active perception is **submodular**, so greedy
next-best-view ties multi-step beam (the classic 1−1/e guarantee); the learned model's real value is the
*expensive-observation* regime (the Godot render), not multi-step routing.

---

## Mission planner + web UI (`planet_browser/`)

Build-planning software: place build orders on a real map, an optimizer sequences them under
physics + battery + time, and you get a 2–3 page mission-control report.

```bash
python -m planet_browser.server              # FastAPI/uvicorn ASGI (or: dustgym-serve)
# open the printed URL; CesiumJS globe (NASA Trek Moon/Mars tiles), build queue, Plan mission -> PDF
```

A **tabbed view-pane switcher** runs the cockpit: **Plan** (globe + build queue), **Perception** (Godot
render frame), **Metrics** (top-down execution playback + telemetry HUD), **Report** (the mission-control
PDF embedded), plus read-only **Validation** (the `validation/` figures), **API** (Swagger `/docs`),
**Server** (`/healthz` + `/metrics`), and **Config** (`/config` overlay + docs) panes for engineers/devs.

- `/plan` — POST a build queue → cut/fill-balanced orders (cut-only excavation included) → hazard- and
  **keep-out**-routed haul → battery-aware mid-task recharge → PDF + markdown report, plus an **autonomy**
  block (closed-loop pose/energy estimate), a **perception** block (the in-loop map-channel coverage), and
  an **as-built acceptance** check (level-pad flatness on the real terrain). `vehicles` > 1 plans a
  **multi-vehicle fleet** (site-exclusive allocation, parallel makespan, space-time deconfliction).
- `/render` — crop a Haworth window, plan a flatten, render BEFORE/AFTER in Godot, return the earthwork.
- `/sense` — the ICE-RASSOR drum-mass inference observable (motor-current → inferred fill → offload
  trigger), with a toggleable seeded noise model grounded in the published 2.56%/7.40% accuracy.
- `/figures` · `/figure/{key}` · `/config` — engineer/dev pane data (validation figures, runtime config).

The energy model is grounded in real IPEx data (Schuler et al., *IPEx TRL-5 Design Overview*, ASCEND
2024; 12S / ~44 V / 30 Ah pack: drive 135 J/m, dig 4151 J/kg, pack 4.79 MJ) — all provenance-tagged,
no fabricated values.

---

## Self-optimizing pipeline

The pipeline closes on itself: **execute → observe the model-vs-truth gap → learn → re-plan**. The
planner's energy model is naively flat (135 J/m, slip = 0); the conserved `drive_step` shows the true
per-leg energy is slope-inflated (slip robs progress). `terrain_authority/self_optimizing.py` drives
over varied slopes, fits a generalizing `inflation(slope)` regression online, and held-out prediction
error collapses from ~20% to <1% on slopes it never trained on. The learned model then re-prices any
route (`planet_browser/adaptive_planner.py`, wired into `/plan`) — a flat and a steep route of equal
distance look identical to the naive planner, but the learned one routes around the steep grade. Only
the inflation regression is learned; the dynamics stay conserved.

---

## Perception: the two-tier map channel + Godot render

Both perception tiers are measured against the conserved truth (figures in `validation/map_channel/`):

- **Onboard rover stereo** — cheap, real-time, noisy (RMSE ~0.32 m; coverage grows to ~16% over an
  8-station drive). A camera-height sweep shows ground SfM collapsing toward the rover's grazing
  eye-level (18/18 images register elevated, only 2/18 at 0.5 m eye-height).
- **Ground COLMAP SfM** — offline, accurate (RMSE ~0.04 m, 97% cell-pass, cameras aligned to truth
  within ~6 mm). A Hapke-vs-Lambert A/B shows the physically-correct non-Lambertian regolith costs
  COLMAP ~33% of its points — a quantified non-Lambertian-MVS finding.

The Godot render/sensor track runs headless on an RTX 3090 (`xvfb-run --rendering-driver vulkan`,
Godot 4.6.3), producing the LAC 8-camera rig + AprilTag bridge frames with a sourced Hapke/Lommel-Seeliger
BRDF. The AprilTag pose channel reads 12.7 mm / 7.15° on `flat_compact`.

---

## Per-planet constants

`terrain_authority/bodies.py` carries literature-sourced surface/regolith mechanics for genuine
habitat/mining targets, every value tagged MEASURED / ESTIMATED / UNKNOWN (nothing fabricated). Full
systematic review with citations: [`docs/bodies_sysrev.md`](docs/bodies_sysrev.md). Gravity is exact
per body and drives wheel load + the Lyasko-corrected Bekker moduli. **Bennu/Phobos are microgravity**,
where the gravity-loaded Bekker model is out of regime (cohesion/granular dynamics dominate) — those
emit a warning and are reachable via `gym.make("Dust/RoverDrive-v0", body="bennu")`.

---

## Documentation

| Doc | What it is |
|---|---|
| [`docs/world_model.md`](docs/world_model.md) | The five-layer world model and the conserved-vs-learned design decision |
| [`docs/related_work.md`](docs/related_work.md) | Where dustgym lands across NASA autonomy, lunar mining, world models, AD, SLAM |
| [`docs/bodies_sysrev.md`](docs/bodies_sysrev.md) | Sourced per-planet terramechanics constants |
| [`PRD.md`](PRD.md) | The layered product requirements (L0 physics → L8 3D app), every requirement → status |
| `validation/` | Figures: planner, RL, nav, map channel, autonomy, plan->render, active perception, self-optimizing |

**Docs site:** <https://dustgym.github.io/dustgym/> &middot;
**Contributing:** [CONTRIBUTING.md](CONTRIBUTING.md) &middot;
**Security:** [SECURITY.md](SECURITY.md) &middot;
**Cite:** [CITATION.cff](CITATION.cff)

---

## Attribution & license

The `terrain_authority` terramechanics authority and the streaming `WorkSite` model are by
**John McCardle** ([jmccardle/roversim](https://github.com/jmccardle/roversim), CC0). dustgym adds the
Gymnasium suite, the per-planet `Body` registry, the world model, the mission planner + web UI, the map
channel + render integration, and the self-optimizing pipeline. Released **CC0** (public domain),
matching upstream.
