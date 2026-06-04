# dustgym

**Gymnasium environments for off-world surface vehicles + autonomous construction**, on a
mass-conserving terramechanics authority parameterized per planetary body (Moon / Mars / Ceres /
Bennu / Phobos / Earth). *Dust* = the regolith every airless rocky surface shares. Lineage: NASA
IPEx (ISRU Pilot Excavator) and the Lunar Autonomy Challenge.

```bash
pip install dustgym        # deps: numpy, scipy, gymnasium; extras: [rl] = torch + stable-baselines3
```

```python
import dustgym             # registers the Dust/* envs on import (or: gymnasium.register_envs(dustgym))
import gymnasium as gym

env = gym.make("Dust/RoverDrive-Mars-v0")   # per-body physics (gravity + Lyasko-corrected regolith)
env = gym.make("Dust/Scheduler-v0")         # multi-objective construction scheduling
obs, info = env.reset(seed=0)
obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
```

## Environments (all pass `gymnasium.utils.env_checker`)

| ID | task | action / obs |
|----|------|------|
| `Dust/RoverDrive-v0` · `-Moon`/`-Mars`/`-Ceres`/`-Earth-v0` | closed-loop drive over terramechanics (slip, sinkage) | `Box(2)` / `Box(32)` |
| `Dust/Construct-v0` | cut/fill to a target heightmap (terrain-matching reward) | `Box(3)` / `Box(104)` |
| `Dust/SkillMacro-v0` | skill-macro construction: pick a cell + cut/dump toward target | `Discrete(128)` / `Box(68)` |
| `Dust/Scheduler-v0` | multi-objective scheduling: borrow pits → build sites, one rover/drum | `Discrete(5)` / `Box(12)` |
| `Dust/WorkSite-v0` | RL controller over the streaming WorkSite seam (flatten pad + build berm) | `Discrete(2)` / `Box(4)` |

The physics authority is exact, deterministic, mass-conserving, and sub-millisecond — it is both the
simulator *and* the reward source, so learned/searched policies only **command** while the authority
**mutates** the terrain (rewards are unhackable by construction).

## Per-planet constants

`terrain_authority/bodies.py` carries literature-sourced surface/regolith mechanics for genuine
habitat/mining targets, every value tagged MEASURED / ESTIMATED / UNKNOWN (nothing fabricated). Full
systematic review with citations: [`docs/bodies_sysrev.md`](docs/bodies_sysrev.md). Gravity is exact
per body and drives wheel load + the Lyasko-corrected Bekker moduli. **Bennu/Phobos are microgravity**,
where the gravity-loaded Bekker model is out of regime (cohesion/granular dynamics dominate) — those
emit a warning and are reachable via `gym.make("Dust/RoverDrive-v0", body="bennu")`.

## Attribution & license

The `terrain_authority` terramechanics authority and the streaming `WorkSite` model are by
**John McCardle** ([jmccardle/roversim](https://github.com/jmccardle/roversim), CC0). dustgym adds the
Gymnasium suite, the per-planet `Body` registry, and the RL/scheduling layer. Released **CC0** (public
domain), matching upstream.
