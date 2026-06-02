"""Tests for the RL environment (rover_env.RoverSimEnv) — Phase 4.

Host-runnable + pytest-discoverable; runs WITHOUT gymnasium (the env is
gymnasium-optional). Validates the Gymnasium API surface (reset/step 5-tuple,
obs shape/dtype), the control reward (flat reaches goal with positive return;
steep slope -> entrapment terminal), determinism, truncation, domain
randomization, mass conservation, and action clipping. A separate gym.Env-path
smoke runs only where gymnasium is installed (see session note).
"""
from __future__ import annotations

import math

import numpy as np

from . import rover_env
from .rover_env import RoverSimEnv


def test_reset_obs_shape_and_dtype():
    env = RoverSimEnv()
    obs, info = env.reset(seed=0)
    assert obs.shape == (env.obs_dim,)
    assert obs.dtype == np.float32
    assert "slope_deg" in info


def test_step_returns_gym_5tuple():
    env = RoverSimEnv()
    env.reset(seed=0)
    obs, reward, terminated, truncated, info = env.step([1.0, 0.0])
    assert obs.shape == (env.obs_dim,)
    assert isinstance(reward, float)
    assert isinstance(terminated, bool) and isinstance(truncated, bool)
    assert "telem" in info


def test_flat_reaches_goal_positive_return():
    env = RoverSimEnv(slope_deg=0.0, start_col=16, goal_col=60, max_steps=200)
    env.reset(seed=0)
    total = 0.0
    reached = terminated = False
    for _ in range(200):
        _, r, terminated, truncated, info = env.step([1.0, 0.0])   # full forward
        total += r
        if terminated or truncated:
            reached = info["reached_goal"]
            break
    assert reached and terminated
    assert total > 0.0


def test_steep_slope_entrapment_terminal():
    env = RoverSimEnv(slope_deg=55.0, start_col=16, goal_col=80, max_steps=50)
    env.reset(seed=0)
    _, r, terminated, _, info = env.step([1.0, 0.0])
    assert terminated and info["telem"]["entrapped"]
    assert not info["reached_goal"]
    assert r < 0.0                       # entrapment penalty dominates


def test_determinism_same_seed_and_actions():
    actions = [[1.0, 0.1]] * 10

    def run():
        env = RoverSimEnv()
        env.reset(seed=7)
        traj = []
        for a in actions:
            o, r, te, tr, _ = env.step(a)
            traj.append((tuple(o.tolist()), r, te, tr))
            if te or tr:
                break
        return traj

    assert run() == run()


def test_truncation_horizon():
    env = RoverSimEnv(slope_deg=0.0, start_col=16, goal_col=95, max_steps=3)
    env.reset(seed=0)
    terminated = truncated = False
    for _ in range(3):
        _, _, terminated, truncated, _ = env.step([0.0, 0.0])   # no motion -> never reach goal
    assert truncated and not terminated


def test_domain_randomization_varies_slope():
    env = RoverSimEnv(randomize=True, slope_max_deg=40.0)
    _, i0 = env.reset(seed=1)
    _, i1 = env.reset(seed=2)
    assert i0["slope_deg"] != i1["slope_deg"]


def test_episode_mass_conserved():
    env = RoverSimEnv(slope_deg=5.0)
    env.reset(seed=0)
    m0 = env.cs.total_mass()
    for _ in range(20):
        _, _, te, tr, _ = env.step([1.0, 0.05])
        if te or tr:
            break
    assert math.isclose(env.cs.total_mass(), m0, rel_tol=1e-9)


def test_action_out_of_range_clipped():
    env = RoverSimEnv()
    env.reset(seed=0)
    obs, _, _, _, _ = env.step([100.0, -100.0])   # clipped to [1,-1], must not crash
    assert np.all(np.isfinite(obs))


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn()
        print(f"[PASS] {fn.__name__}")
    print(f"\n{len(fns)}/{len(fns)} rover_env checks passed (gymnasium present: {rover_env.HAS_GYM}).")


if __name__ == "__main__":
    _run_all()
