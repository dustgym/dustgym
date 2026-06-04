#!/usr/bin/env python3
"""M4 viability demo: a LEARNED scheduler on randomized multi-site construction layouts vs the greedy
nearest-batch heuristic, random, and the model-based search optimum.

This is the env where learning earns its keep (the M3 finding): single-objective energy is
dig-dominated (conserved-mass-fixed) so routing has no headroom, but with several separated build
sites served by one rover/one drum the leg ORDER controls makespan. Action = one trip-leg (Discrete
over regions: load the drum at a borrow pit, or dump it toward a build site via fill_toward).

Observed on a 200k-step run (held-out randomized layouts, seed 0):
    greedy heuristic            success=100%  avg_legs=28
    random                      success~=3%
    PPO (model-free MLP)        success=100%  avg_legs=27   (beats greedy, but NOT optimal)
    beam search (model-based)   success=100%  avg_legs=24   (the optimum -- exact cheap simulator)

The MLP PPO policy is good but not optimal; the optimum is reached by model-based search and by a
search-DISTILLED policy (same MLP, taught the search's moves) -- see scripts/demo/distill_scheduler.py.
Architecture (CNN/DNN/transformer) is the wrong axis; algorithm class (model-free vs model-based) is.

Run with the runtime venv (gymnasium + torch + SB3):
    PYTHONPATH=<repo> /mnt/projects/07_runtime_system/venv/bin/python scripts/demo/train_scheduler.py
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import numpy as np

from terrain_authority import ipex_specs as ix
from terrain_authority.scheduler_env import (SchedulerEnv, beam_search_plan,
                                             greedy_nearest_schedule)


def make_cfg():
    return dict(
        grid=64, cell_m=0.5,
        borrows=[(4, 4, 12, 12), (52, 52, 60, 60)],
        builds=[(10, 40, 14, 44), (40, 10, 44, 14), (44, 44, 48, 48)],
        fill_delta_m=0.10, mound_height_m=0.30, drum_capacity_kg=120.0, max_legs=40,
        travel_cost_per_cell=ix.drive_energy_per_m() * 0.5,   # grounded: 135 J/m * cell_m
        dig_cost_per_kg=ix.dig_energy_per_kg(),               # grounded: 4151 J/kg
        randomize=True,
    )


def evaluate(make_env, policy, n=30, seed0=9000):
    succ, legs = [], []
    for i in range(n):
        env = make_env(); obs, _ = env.reset(seed=seed0 + i); done = False; info = {}
        while not done:
            obs, _, te, tr, info = env.step(policy(env, obs)); done = te or tr
        succ.append(bool(info["success"]))
        legs.append(info["legs"] if info["success"] else env.max_legs)
    return float(np.mean(succ)), float(np.mean(legs))


def eval_beam(make_env, n=8, seed0=9000, width=20):
    legs = []
    for i in range(n):
        env = make_env(); env.reset(seed=seed0 + i)
        plan = beam_search_plan(env, width=width)
        env2 = make_env(); env2.reset(seed=seed0 + i); info = {}
        for a in plan:
            _, _, te, tr, info = env2.step(a)
            if te or tr:
                break
        legs.append(info.get("legs", env.max_legs) if info.get("success") else env.max_legs)
    return float(np.mean(legs))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--timesteps", type=int, default=200_000)
    ap.add_argument("--eval-episodes", type=int, default=30)
    args = ap.parse_args()
    cfg = make_cfg()

    def make_env():
        return SchedulerEnv(**cfg)

    rng = np.random.default_rng(1)
    gs, gl = evaluate(make_env, lambda e, o: greedy_nearest_schedule(e), args.eval_episodes)
    rs, rl = evaluate(make_env, lambda e, o: rng.integers(e.n_region), args.eval_episodes)
    opt = eval_beam(make_env)
    print(f"greedy heuristic           success={gs:.0%}  avg_legs={gl:.1f}")
    print(f"random                     success={rs:.0%}  avg_legs={rl:.1f}")
    print(f"beam search (model-based)  optimum legs={opt:.1f}")

    from stable_baselines3 import PPO
    model = PPO("MlpPolicy", make_env(), n_steps=2048, batch_size=256, gamma=0.99,
                ent_coef=0.02, learning_rate=3e-4, verbose=0, seed=0, device="cpu")
    model.learn(total_timesteps=args.timesteps)
    ps, pl = evaluate(make_env, lambda e, o: int(model.predict(o, deterministic=True)[0]),
                      args.eval_episodes)
    print(f"PPO (model-free MLP)       success={ps:.0%}  avg_legs={pl:.1f}  ({args.timesteps} steps)")
    print(f"\nRL/ML planning is VIABLE on the multi-objective layer (PPO {pl:.0f} legs, {ps:.0%} success, "
          f"beats random by {(ps - rs) * 100:.0f} pts), but model-free PPO is NOT optimal: the optimum "
          f"is ~{opt:.0f} legs.\nThe optimum is reached by model-based search (we have an exact cheap "
          f"simulator) -- the best LEARNED policy distills that search (distill_scheduler.py).")


if __name__ == "__main__":
    main()
