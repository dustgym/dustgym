"""Tests for SchedulerEnv (M4: multi-objective construction scheduling).

The viability point: on a SINGLE objective the IPEx-grounded energy is dig-dominated (conserved-mass-
fixed) so routing has no headroom. On MULTIPLE separated sites served by one rover/one drum, the leg
ORDER controls makespan — so a planner finishes within the leg budget while random does not. The key
test (test_planner_beats_random_under_budget) pins that gap. Host-runnable + pytest; numpy only (no gym/
torch) — the learned-policy result lives in scripts/demo/train_scheduler.py.
"""
from __future__ import annotations

import math

import numpy as np

from terrain_authority.scheduler_env import SchedulerEnv, beam_search_plan, greedy_nearest_schedule

CFG = dict(grid=64, cell_m=0.5,
           borrows=[(4, 4, 12, 12), (52, 52, 60, 60)],
           builds=[(10, 40, 14, 44), (40, 10, 44, 14), (44, 44, 48, 48)],
           fill_delta_m=0.10, mound_height_m=0.30, drum_capacity_kg=120.0, max_legs=40,
           travel_cost_per_cell=67.0, dig_cost_per_kg=4151.0)


def mk(**kw):
    c = dict(CFG); c.update(kw)
    return SchedulerEnv(**c)


def test_mass_conserved():
    env = mk(); env.reset(seed=0); m0 = env.cs.total_mass()
    rng = np.random.default_rng(0)
    for _ in range(40):
        _, _, te, tr, _ = env.step(rng.integers(env.n_region))
        if te or tr:
            break
    assert math.isclose(env.cs.total_mass(), m0, rel_tol=1e-9)


def test_planner_solves_within_budget():
    env = mk(); env.reset(seed=0); done = False; info = {}
    while not done:
        _, _, te, tr, info = env.step(greedy_nearest_schedule(env)); done = te or tr
    assert info["success"]
    assert info["legs"] <= CFG["max_legs"]


def test_planner_beats_random_under_budget():
    """M4 viability: ordering matters -> nearest-batch planner solves every layout, random nearly never."""
    g, r = [], []
    rng = np.random.default_rng(0)
    for s in range(8):
        e = mk(randomize=True); e.reset(seed=s); done = False; info = {}
        while not done:
            _, _, te, tr, info = e.step(greedy_nearest_schedule(e)); done = te or tr
        g.append(bool(info["success"]))
        e = mk(randomize=True); e.reset(seed=s); done = False; info = {}
        while not done:
            _, _, te, tr, info = e.step(rng.integers(e.n_region)); done = te or tr
        r.append(bool(info["success"]))
    assert all(g), g                  # planner solves every randomized layout within budget
    assert sum(r) <= 1                # random almost never finishes in time


def test_model_based_search_beats_greedy():
    """Headroom check: greedy is NOT optimal. Beam search in the exact (deterministic, cheap) authority
    finds a strictly shorter valid schedule -> there is real headroom, and model-based planning captures
    it (the best learned policy distills this search; see scripts/demo/distill_scheduler.py)."""
    env = mk(); env.reset(seed=0); done = False; info = {}
    while not done:
        _, _, te, tr, info = env.step(greedy_nearest_schedule(env)); done = te or tr
    greedy_legs = info["legs"]
    # plan + replay the beam schedule on a fresh copy of the same instance
    env2 = mk(); env2.reset(seed=0)
    plan = beam_search_plan(env2, width=20)
    env3 = mk(); env3.reset(seed=0); info3 = {}
    for a in plan:
        _, _, te, tr, info3 = env3.step(a)
        if te or tr:
            break
    assert info3["success"]
    assert info3["legs"] < greedy_legs, (info3["legs"], greedy_legs)   # search strictly beats greedy
    for a0, b0, c0, d0 in env3.builds:                                  # and the solution is valid
        assert (env3.cs.derive_height()[a0:c0, b0:d0] - env3.target[a0:c0, b0:d0]).max() <= 1e-9


def test_no_overshoot_after_solve():
    """Sites are filled via fill_toward (FIX-4) -> no build cell ends above its target."""
    env = mk(); env.reset(seed=0); done = False
    while not done:
        _, _, te, tr, _ = env.step(greedy_nearest_schedule(env)); done = te or tr
    h = env.cs.derive_height()
    for a, b, c, d in env.builds:
        assert (h[a:c, b:d] - env.target[a:c, b:d]).max() <= 1e-9


def test_layout_randomization_keeps_counts():
    env = mk(randomize=True)
    env.reset(seed=1); r1 = list(env.regions)
    env.reset(seed=2); r2 = list(env.regions)
    assert r1 != r2                                   # positions differ
    assert env.n_region == 5 and env.n_borrow == 2    # counts (action space) stable


def test_obs_shape_and_determinism():
    env = mk(); o, _ = env.reset(seed=3)
    assert o.shape == (env.obs_dim,) == (2 * env.n_region + 2,)
    acts = [0, 2, 1, 3, 0, 4]

    def run():
        e = mk(); e.reset(seed=3); out = []
        for a in acts:
            _, rwd, te, tr, info = e.step(a); out.append((round(rwd, 6), round(info["deficit"], 6)))
            if te or tr:
                break
        return out
    assert run() == run()


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn(); print(f"[PASS] {fn.__name__}")
    print(f"\n{len(fns)}/{len(fns)} scheduler-env checks passed.")


if __name__ == "__main__":
    _run_all()
