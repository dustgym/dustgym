---
title: Home
nav_order: 1
---

# dustgym

**A conserved-physics world model, autonomous planner, and Gymnasium suite for off-world surface
construction.** A robot that *reshapes* regolith — excavate, grade, berm, fill — on an airless rocky
surface, on a mass-conserving terramechanics authority parameterized per planetary body (Moon, Mars,
Ceres, Bennu, Phobos, Earth). *Dust* is the regolith every airless surface shares. Lineage: NASA IPEx
(ISRU Pilot Excavator) and the Lunar Autonomy Challenge.

The design call, in one line: **conserved physics for the dynamics (exact, unhackable) plus a learned
model only for the expensive perception branch.** The dynamics are provable; the learning budget is
spent only where observation is genuinely expensive.

[View on GitHub](https://github.com/dustgym/dustgym){: .btn .btn-primary }
[Get started](#quickstart){: .btn }

---

## Quickstart

```bash
pip install dustgym            # numpy, scipy, gymnasium; extras: [rl] = torch + stable-baselines3
```

```python
import dustgym                 # registers the Dust/* envs on import
import gymnasium as gym
env = gym.make("Dust/RoverDrive-Mars-v0")     # per-body physics (gravity + Lyasko-corrected regolith)
obs, info = env.reset(seed=0)
obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
```

From source (also exposes the conserved authority and the mission planner directly):

```bash
git clone https://github.com/dustgym/dustgym && cd dustgym
pip install -e .[rl]
PYTHONPATH=. python -m planet_browser.server      # the SimCity-style mission planner + web UI
```

---

## Documentation

| Doc | What it is |
|---|---|
| [The modelled vehicle — IPEx]({{ site.baseurl }}/vehicle_ipex) | The ISRU Pilot Excavator (RASSOR is the precursor), grounded in the six NASA IPEx papers; the digital-twin architecture and the excavation gap dustgym fills |
| [The five-layer world model]({{ site.baseurl }}/world_model) | Geometry / Material / Physics / Task / Uncertainty, and the conserved-vs-learned design decision |
| [Related work]({{ site.baseurl }}/related_work) | Where dustgym lands across NASA autonomy, lunar mining, world models, autonomous driving, SLAM |
| [Per-planet constants (systematic review)]({{ site.baseurl }}/bodies_sysrev) | Literature-sourced terramechanics per body, every value tagged MEASURED / ESTIMATED / UNKNOWN |
| [Spec coverage scorecard]({{ site.baseurl }}/spec_coverage) | Section-by-section: what is built / partial / surrogate / left out, with file:line evidence |
| [Architecture review]({{ site.baseurl }}/architecture_review) | Production-grade architecture assessment |
| [Autonomous planning review]({{ site.baseurl }}/autonomous_planning_review) | Single- and multi-vehicle planning limits |
| [Sensor-bridge contract]({{ site.baseurl }}/sensor_bridge_contract) | Seam 2: the Godot → ROS2 `sensors.json` + PNG contract |
| [Render fidelity spec]({{ site.baseurl }}/render_fidelity_spec) | The Godot render / sensor-model fidelity targets |

Repository-root references (rendered on GitHub):
[Product requirements (`PRD.md`)](https://github.com/dustgym/dustgym/blob/main/PRD.md) ·
[Master technical spec](https://github.com/dustgym/dustgym/blob/main/ipex-terrain-sim-spec.md) ·
[Building taxonomy](https://github.com/dustgym/dustgym/blob/main/building_taxonomy.md) ·
[Contributing](https://github.com/dustgym/dustgym/blob/main/CONTRIBUTING.md) ·
[Security policy](https://github.com/dustgym/dustgym/blob/main/SECURITY.md)

---

## Why it is trustworthy

The terramechanics authority is exact, deterministic, mass-conserving, and sub-millisecond — it is both
the simulator *and* the reward source, so learned or searched policies only **command** while the
authority **mutates**. Every physical constant carries its source and a provenance tag (`MEASURED` /
`ESTIMATED` / `[CALIB]` / `[UNKNOWN]`); there is no synthetic data anywhere in the figures, tests, or
validation. The energy model is grounded in real IPEx data (Schuler et al., *IPEx TRL-5 Design
Overview*, ASCEND 2024).

Released into the **public domain (CC0 1.0)**. The `terrain_authority` terramechanics core and the
streaming `WorkSite` model are by **John McCardle**; dustgym adds the Gymnasium suite, the per-planet
`Body` registry, the world model, the mission planner + web UI, the map channel + render integration,
and the self-optimizing pipeline.
