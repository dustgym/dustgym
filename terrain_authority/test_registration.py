"""Tests for the Dust/* Gymnasium suite registration + per-body physics.

Verifies every registered ID is gym.make-able with no args, round-trips reset/step under the
gym.make wrappers (OrderEnforcing / PassiveEnvChecker / TimeLimit), passes the official env_checker,
and that the per-body drive env actually changes its physics with gravity. Skipped automatically
where gymnasium is not installed (the bare-numpy core).
"""
from __future__ import annotations

import pytest

gym = pytest.importorskip("gymnasium")

import dustgym  # noqa: E402,F401  -- importing the suite registers Dust/* on import
from terrain_authority.bodies import BODIES, params_for_body  # noqa: E402
from terrain_authority.registration import ENV_IDS  # noqa: E402


@pytest.mark.parametrize("env_id", ENV_IDS)
def test_registered(env_id):
    from gymnasium.envs.registration import registry
    assert env_id in registry


@pytest.mark.parametrize("env_id", ENV_IDS)
def test_gym_make_roundtrip(env_id):
    env = gym.make(env_id)                      # no args -> defaults make it constructible
    obs, info = env.reset(seed=0)
    assert env.observation_space.contains(obs), env_id
    obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
    assert isinstance(reward, float)
    env.close()


@pytest.mark.parametrize("env_id", ENV_IDS)
def test_check_env(env_id):
    from gymnasium.utils.env_checker import check_env
    check_env(gym.make(env_id).unwrapped, skip_render_check=True)


def test_override_kwargs():
    # any constructor arg can still be overridden through gym.make
    env = gym.make("Dust/Scheduler-v0", max_legs=25)
    assert env.unwrapped.max_legs == 25
    env.close()


def test_per_body_gravity_differs():
    # the per-body drive env carries the body's surface gravity into the physics
    moon = gym.make("Dust/RoverDrive-Moon-v0").unwrapped
    mars = gym.make("Dust/RoverDrive-Mars-v0").unwrapped
    earth = gym.make("Dust/RoverDrive-Earth-v0").unwrapped
    assert moon.g == BODIES["moon"].g < mars.g == BODIES["mars"].g < earth.g == BODIES["earth"].g
    # and the regolith params are gravity-corrected (Earth = identity baseline; Moon = reduced)
    assert moon.params_base.k_phi < earth.params_base.k_phi


def test_body_kwarg_any_body():
    # any body in the registry is reachable through the body= kwarg (even unregistered IDs)
    env = gym.make("Dust/RoverDrive-v0", body="ceres").unwrapped
    assert env.g == BODIES["ceres"].g
    env.close()
