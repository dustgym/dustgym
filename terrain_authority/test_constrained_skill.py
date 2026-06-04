"""Tests for the M3 resource model on SkillMacroEnv — drum capacity + energy/travel.

The point of M3: constraints make selection+ordering a real planning problem. Key test
(test_planner_beats_naive_under_budget): under a tight energy budget the capacity/travel-aware
planner SOLVES flatten while naive random FAILS — the inverse of the unconstrained case where
random tied/won. Host-runnable + pytest; no gymnasium needed.
"""
from __future__ import annotations

import math

import numpy as np

from terrain_authority import challenge as ch
from terrain_authority.skill_env import SkillMacroEnv, greedy_nearest_action


def mk(budget=70.0, seed=2, base="bumps", cap=15.0, tol=0.01, max_macros=400):
    c = ch.Challenge(id="m3", name="f", difficulty_tier=2,
                     map=ch.MapSpec(seed=seed, base=base, grid=44, roughness_m=0.004),
                     objective=ch.Objective(type="flatten_pad", region=(14, 14, 30, 30),
                                            tolerance_m=tol),
                     constraints=ch.Constraints(max_time_steps=max_macros))
    return SkillMacroEnv(c, drum_capacity_kg=cap, travel_cost_per_cell=1.0, energy_budget=budget)


def test_energy_decreases_with_travel():
    env = mk(budget=1e9); env.reset(seed=0); e0 = env._energy
    env.step([-1.0, -1.0, 1.0])              # drive to a far corner -> travel cost
    assert env._energy < e0


def test_out_of_energy_truncates():
    env = mk(budget=5.0); env.reset(seed=0)  # one far move exhausts it
    oo = False
    for _ in range(50):
        _, _, te, tr, info = env.step([1.0, 1.0, 1.0])
        if tr and info["out_of_energy"]:
            oo = True
        if te or tr:
            break
    assert oo


def test_capacity_caps_drum():
    env = mk(base="mound", cap=2.0, budget=1e9); env.reset(seed=0)
    rng = np.random.default_rng(0)
    drum_max = 0.0
    for _ in range(200):
        a = rng.uniform(-1, 1, 2)
        env.step([a[0], a[1], 1.0])          # cut-only across the region (lots of excess)
        drum_max = max(drum_max, env.cs.drum_inventory)
    assert env.cs.drum_inventory <= 2.0 + 1e-6      # never exceeds capacity
    assert drum_max >= 1.5                           # and the cap actually binds (drum fills)


def test_mass_conserved_constrained():
    env = mk(budget=1e9); env.reset(seed=0); m0 = env.cs.total_mass()
    rng = np.random.default_rng(0)
    for _ in range(40):
        _, _, te, tr, _ = env.step(rng.uniform(-1, 1, 3))
        if te or tr:
            break
    assert math.isclose(env.cs.total_mass(), m0, rel_tol=1e-9)


def test_planner_beats_naive_under_budget():
    """M3 viability: tight budget -> capacity/travel-aware planner SOLVES, naive random FAILS."""
    rng = np.random.default_rng(0)
    g, r = [], []
    for s in range(4):
        env = mk(budget=70.0); env.reset(seed=s); done = False; info = {}
        while not done:
            _, _, te, tr, info = env.step(greedy_nearest_action(env)); done = te or tr
        g.append(bool(info["success"]))
        env = mk(budget=70.0); env.reset(seed=s); done = False; info = {}
        while not done:
            _, _, te, tr, info = env.step(rng.uniform(-1, 1, 3)); done = te or tr
        r.append(bool(info["success"]))
    assert all(g), g                 # planner succeeds within budget on every seed
    assert sum(r) <= 1               # naive nearly always fails


def test_determinism():
    actions = [[0.3, 0.4, 1.0], [-0.5, 0.2, -1.0], [0.1, 0.9, 1.0]]

    def run():
        env = mk(); env.reset(seed=3); out = []
        for a in actions:
            _, rwd, te, tr, _ = env.step(a); out.append((round(rwd, 6), round(env._energy, 4)))
            if te or tr:
                break
        return out
    assert run() == run()


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn(); print(f"[PASS] {fn.__name__}")
    print(f"\n{len(fns)}/{len(fns)} constrained-skill checks passed.")


if __name__ == "__main__":
    _run_all()
