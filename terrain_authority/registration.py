"""registration.py — register the dustgym envs as a Gymnasium suite (namespace ``Dust``).

Makes the envs discoverable through ``gymnasium.make("Dust/<Env>-v0")``. Importing the package
registers them (the documented Gymnasium third-party pattern)::

    import dustgym                  # or: import terrain_authority  -- either registers Dust/*
    import gymnasium as gym
    env = gym.make("Dust/RoverDrive-Mars-v0")     # per-body physics (gravity + regolith)
    env = gym.make("Dust/Scheduler-v0")           # body-neutral (mass-conserving construction)

Per-body IDs: the DRIVE env depends on gravity (weight = m*g) and the Lyasko-corrected regolith, so it
is registered per body (``RoverDrive-{Moon,Mars,Earth}-v0``) and accepts ``gym.make(..., body="ceres")``
for any body in ``terrain_authority.bodies.BODIES``. The CONSTRUCTION/scheduling envs are mass-conserving
(cut/fill is gravity-invariant in this model), so they are registered body-neutral -- a "Mars Scheduler"
would be identical, so it is not faked.

(The ``[project.entry-points."gymnasium.envs"]`` hook in pyproject.toml is a forward-compatible plugin
entry; current Gymnasium needs the import above. ``gymnasium.register_envs(dustgym)`` is the explicit,
lint-friendly equivalent.) Each ID is constructible with NO user arguments; any constructor arg can be
overridden via ``gym.make(id, **kw)``. register_envs() is a no-op without gymnasium (bare-numpy core
stays importable).
"""
from __future__ import annotations

# DRIVE env registered per body for the GRAVITY-LOADED bodies (Bekker model valid). Bennu/Phobos are
# microgravity (Bekker out of regime) -> NOT pre-registered as drive IDs; reachable via
# gym.make("Dust/RoverDrive-v0", body="bennu") which emits an out-of-regime warning. (bodies_sysrev.md)
# DERIVED from the registry so adding one gravity-loaded Body (the one-entry extensibility promise) auto-
# creates its Dust/RoverDrive-<Body>-v0 ID; microgravity bodies are excluded by construction.
from .bodies import BODIES  # noqa: E402

ROVER_BODIES = [k for k, b in BODIES.items() if b.bekker_regime == "gravity-loaded"]

ENV_IDS = (
    ["Dust/RoverDrive-v0"]
    + [f"Dust/RoverDrive-{b.capitalize()}-v0" for b in ROVER_BODIES]
    + ["Dust/Construct-v0", "Dust/SkillMacro-v0", "Dust/Scheduler-v0", "Dust/WorkSite-v0",
       "Dust/ActivePerception-v0"]
)

_REGISTERED = False


def _default_challenge():
    """A small, self-contained flatten-a-pad challenge used as the default for the construction envs."""
    from . import challenge as ch
    return ch.Challenge(
        id="dust_default", name="Flatten a construction pad", difficulty_tier=2,
        map=ch.MapSpec(seed=0, base="bumps", grid=48, roughness_m=0.004),
        objective=ch.Objective(type="flatten_pad", region=(16, 16, 32, 32), tolerance_m=0.01),
        constraints=ch.Constraints(max_time_steps=400),
    )


def _scheduler_kwargs():
    from . import ipex_specs as ix
    return dict(
        grid=64, cell_m=0.5,
        borrows=[(4, 4, 12, 12), (52, 52, 60, 60)],
        builds=[(10, 40, 14, 44), (40, 10, 44, 14), (44, 44, 48, 48)],
        fill_delta_m=0.10, mound_height_m=0.30, drum_capacity_kg=120.0, max_legs=40,
        travel_cost_per_cell=ix.drive_energy_per_m() * 0.5,   # grounded: 135 J/m * cell_m
        dig_cost_per_kg=ix.dig_energy_per_kg(),               # grounded: 4151 J/kg
        randomize=True,
    )


def register_envs():
    """Register the Dust/* environments with Gymnasium. Idempotent; no-op without gymnasium."""
    global _REGISTERED
    if _REGISTERED:
        return
    try:
        from gymnasium.envs.registration import register, registry
    except Exception:
        return
    dc = _default_challenge()
    rover_ep = "terrain_authority.rover_env:RoverSimEnv"
    specs = [
        # DRIVE: default (Moon) + per-body. (id, "module:Class", kwargs, max_episode_steps)
        ("Dust/RoverDrive-v0", rover_ep, {"body": "moon"}, 2000),
    ]
    specs += [(f"Dust/RoverDrive-{b.capitalize()}-v0", rover_ep, {"body": b}, 2000)
              for b in ROVER_BODIES]
    # CONSTRUCTION: body-neutral (mass-conserving -> gravity-invariant)
    specs += [
        ("Dust/Construct-v0", "terrain_authority.terrain_target_env:TerrainTargetEnv",
         {"challenge": dc}, None),
        ("Dust/SkillMacro-v0", "terrain_authority.skill_env:SkillMacroEnv",
         {"challenge": dc, "discrete_cells": 8}, None),
        ("Dust/Scheduler-v0", "terrain_authority.scheduler_env:SchedulerEnv",
         _scheduler_kwargs(), None),
        # RL controller over John's WorkSite execution seam (flatten/dump + drum ledger)
        ("Dust/WorkSite-v0", "terrain_authority.worksite_env:WorkSiteConstructEnv", {}, None),
        # PERCEPTION: active next-best-view mapping (the map channel / Uncertainty layer as the reward)
        ("Dust/ActivePerception-v0", "terrain_authority.active_perception_env:ActivePerceptionEnv", {}, 400),
    ]
    for env_id, entry_point, kwargs, max_steps in specs:
        if env_id in registry:
            continue
        register(id=env_id, entry_point=entry_point, kwargs=kwargs, max_episode_steps=max_steps)
    _REGISTERED = True
