"""Tests for CEM policy training (cem.py) — Phase 5 capstone.

Host-runnable + pytest-discoverable; pure numpy (no gymnasium / RL lib). Fast
settings. Validates the linear policy bounds, that CEM improves and beats a random
policy, and determinism.
"""
from __future__ import annotations

import numpy as np

from . import cem
from .rover_env import RoverSimEnv


def test_linear_policy_action_bounds():
    pol = cem.LinearPolicy(obs_dim=32, act_dim=2)
    rng = np.random.default_rng(0)
    for _ in range(20):
        a = pol.act(rng.standard_normal(pol.n_params) * 5.0, rng.standard_normal(32))
        assert a.shape == (2,)
        assert np.all(a >= -1.0) and np.all(a <= 1.0)   # tanh-squashed


def test_cem_improves_and_beats_random():
    # flat, deterministic env: optimal (orient + full forward) is learnable; random flails.
    env = RoverSimEnv(grid=32, slope_deg=0.0, start_col=6, goal_col=26, max_steps=40,
                      randomize=False)
    res = cem.train_cem(env, iters=6, pop=16, eval_seeds=(0,), rng_seed=0)
    base = cem.random_baseline(env, res["policy"], (0,))
    assert res["history"][-1] >= res["history"][0]          # best score is non-decreasing
    assert res["final"]["mean_return"] > base["mean_return"]  # trained beats random
    assert res["final"]["reached_rate"] > 0.0                 # trained reaches the goal


def test_cem_deterministic():
    env = RoverSimEnv(grid=32, slope_deg=0.0, start_col=6, goal_col=24, max_steps=30,
                      randomize=False)
    a = cem.train_cem(env, iters=4, pop=10, eval_seeds=(0,), rng_seed=7)
    b = cem.train_cem(env, iters=4, pop=10, eval_seeds=(0,), rng_seed=7)
    assert np.array_equal(a["best_theta"], b["best_theta"])
    assert a["history"] == b["history"]


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn()
        print(f"[PASS] {fn.__name__}")
    print(f"\n{len(fns)}/{len(fns)} cem checks passed.")


if __name__ == "__main__":
    _run_all()
