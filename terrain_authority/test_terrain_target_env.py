"""Tests for the goal-conditioned construction env (terrain_target_env.py) — M1 / F4.

Host-runnable + pytest-discoverable; runs without gymnasium. Covers obs shape, the
3-D action (drive + drum cut/dump), mass conservation under excavation, determinism,
success detection, and the traverse objective.
"""
from __future__ import annotations

import math

import numpy as np

from . import challenge as ch
from .terrain_target_env import TerrainTargetEnv


def _flatten(seed=2, tol=0.01, max_steps=40, grid=48):
    return ch.Challenge(id="f", name="flatten", difficulty_tier=2,
                        map=ch.MapSpec(seed=seed, base="bumps", grid=grid),
                        objective=ch.Objective(type="flatten_pad", region=(16, 16, 32, 32),
                                               tolerance_m=tol),
                        constraints=ch.Constraints(max_time_steps=max_steps))


def test_reset_obs_shape_and_dtype():
    env = TerrainTargetEnv(_flatten())
    obs, info = env.reset(seed=0)
    assert obs.shape == (env.obs_dim,)
    assert obs.dtype == np.float32
    assert "rmse" in info


def test_step_returns_5tuple():
    env = TerrainTargetEnv(_flatten())
    env.reset(seed=0)
    obs, r, term, trunc, info = env.step([0.2, 0.0, 0.0])
    assert obs.shape == (env.obs_dim,)
    assert isinstance(r, float) and isinstance(term, bool) and isinstance(trunc, bool)


def test_cut_moves_mass_to_drum():
    env = TerrainTargetEnv(_flatten())
    env.reset(seed=0)
    inv0 = env.cs.drum_inventory
    env.step([0.0, 0.0, 1.0])          # engage cut
    assert env.cs.drum_inventory > inv0


def test_drum_cut_then_dump_conserves_mass():
    env = TerrainTargetEnv(_flatten())
    env.reset(seed=0)
    m0 = env.cs.total_mass()           # grid + drum inventory
    env.step([0.0, 0.0, 1.0])          # cut
    env.step([0.0, 0.0, -1.0])         # dump
    assert math.isclose(env.cs.total_mass(), m0, rel_tol=1e-9)


def test_episode_mass_conserved():
    env = TerrainTargetEnv(_flatten())
    env.reset(seed=0)
    m0 = env.cs.total_mass()
    for k in range(20):
        _, _, te, tr, _ = env.step([0.3, 0.05, 1.0 if k % 3 else -1.0])
        if te or tr:
            break
    assert math.isclose(env.cs.total_mass(), m0, rel_tol=1e-9)


def test_determinism_same_seed_actions():
    actions = [[0.3, 0.1, 0.0], [0.2, 0.0, 1.0], [0.1, -0.1, -1.0]]

    def run():
        env = TerrainTargetEnv(_flatten())
        env.reset(seed=5)
        out = []
        for a in actions:
            o, r, te, tr, _ = env.step(a)
            out.append((tuple(round(x, 6) for x in o.tolist()), round(r, 6), te, tr))
            if te or tr:
                break
        return out

    assert run() == run()


def test_success_when_tolerance_huge():
    env = TerrainTargetEnv(_flatten(tol=100.0))   # already within tolerance
    env.reset(seed=0)
    _, _, term, _, info = env.step([0.0, 0.0, 0.0])
    assert term and info["success"]


def test_traverse_reaches_goal():
    c = ch.Challenge(id="t", name="traverse", difficulty_tier=1,
                     map=ch.MapSpec(seed=1, base="flat", grid=48),
                     objective=ch.Objective(type="traverse", region=(0, 0, 48, 48), goal_rc=(24, 40)),
                     constraints=ch.Constraints(max_time_steps=120))
    env = TerrainTargetEnv(c)
    env.reset(seed=0)
    reached = False
    for _ in range(120):
        _, _, term, trunc, info = env.step([1.0, 0.0, 0.0])   # full forward toward +col goal
        if term:
            reached = info["success"]
            break
        if trunc:
            break
    assert reached


def test_action_out_of_range_clipped():
    env = TerrainTargetEnv(_flatten())
    env.reset(seed=0)
    obs, _, _, _, _ = env.step([99.0, -99.0, 99.0])
    assert np.all(np.isfinite(obs))


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn()
        print(f"[PASS] {fn.__name__}")
    print(f"\n{len(fns)}/{len(fns)} terrain_target_env checks passed.")


if __name__ == "__main__":
    _run_all()
