"""dustgym — a Gymnasium suite for off-world surface vehicles + autonomous construction.

Reinforcement-learning environments on a mass-conserving terramechanics authority, parameterized
per planetary body (gravity + regolith): the Moon, Mars, Earth, and more. "Dust" = the regolith
that every airless/rocky surface shares (IPEx / Lunar Autonomy Challenge lineage).

Importing this package REGISTERS the ``Dust/*`` environments with Gymnasium (the documented
third-party pattern), so::

    import dustgym                                  # registers Dust/* on import
    import gymnasium as gym
    env = gym.make("Dust/RoverDrive-Mars-v0")       # per-body physics (gravity + regolith)
    env = gym.make("Dust/Scheduler-v0")             # body-neutral (mass-conserving construction)

The environments and the physics authority live in ``terrain_authority``; this package is the thin
Gymnasium-facing layer. Per-body constants live in ``terrain_authority.bodies``.
"""
from __future__ import annotations

from terrain_authority.bodies import BODIES
from terrain_authority.registration import ENV_IDS, register_envs

register_envs()

__all__ = ["register_envs", "ENV_IDS", "BODIES"]
