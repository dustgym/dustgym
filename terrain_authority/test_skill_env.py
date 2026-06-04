"""Tests for the skill-macro construction env (skill_env.py) — M2 pivot.

The action is state->skill->parameters: select a cell + cut/dump TOWARD target, executed
by the conserved authority. Key test: a GREEDY cell-selector now actually flattens (it
barely moved under raw drive+drum PPO), proving the macro action space is the right
abstraction. Host-runnable + pytest; no gymnasium needed.
"""
from __future__ import annotations

import math

import numpy as np

from . import challenge as ch
from . import skill_env as se
from .skill_env import SkillMacroEnv


def _flatten(grid=44, region=(14, 30, 30, 30), tol=0.01, max_macros=60, seed=2, base="bumps"):
    # region as (r0,c0,r1,c1); default a centered square
    region = (14, 14, 30, 30)
    return ch.Challenge(id="m2", name="flatten", difficulty_tier=2,
                        map=ch.MapSpec(seed=seed, base=base, grid=grid, roughness_m=0.004),
                        objective=ch.Objective(type="flatten_pad", region=region, tolerance_m=tol),
                        constraints=ch.Constraints(max_time_steps=max_macros))


def test_obs_shape_and_step_5tuple():
    env = SkillMacroEnv(_flatten())
    obs, info = env.reset(seed=0)
    assert obs.shape == (env.obs_dim,) and obs.dtype == np.float32
    obs, r, te, tr, info = env.step([0.5, 0.5, 1.0])
    assert obs.shape == (env.obs_dim,) and isinstance(r, float)
    assert isinstance(te, bool) and isinstance(tr, bool) and "rmse" in info


def test_mass_conserved_cut_then_dump():
    env = SkillMacroEnv(_flatten())
    env.reset(seed=0)
    m0 = env.cs.total_mass()
    env.step([0.2, 0.2, 1.0])      # cut toward target somewhere
    env.step([0.8, 0.8, -1.0])     # dump toward target elsewhere
    assert math.isclose(env.cs.total_mass(), m0, rel_tol=1e-9)


def test_greedy_cell_selector_solves_flatten():
    """THE pivot proof: on the SAME bumps-flatten task raw drive+drum PPO scored 0% success
    on (+6% RMSE), a greedy macro cell-selector SOLVES it to tolerance — the macro action
    space is the right abstraction. (On a hard 20cm mound it gets ~45% reduction vs raw's
    +8%; full solve there just needs more macros.) Mass conserved throughout."""
    env = SkillMacroEnv(_flatten(base="bumps", max_macros=120))
    _, info = env.reset(seed=0)
    rmse0 = info["rmse"]
    done = False
    while not done:
        _, _, te, tr, info = env.step(se.greedy_action(env))
        done = te or tr
    assert info["success"] and info["rmse"] <= env.tol, (rmse0, info["rmse"])   # SOLVED
    assert math.isclose(env.cs.total_mass(), env._m0, rel_tol=1e-9)             # conserved


def test_determinism():
    actions = [[0.3, 0.4, 1.0], [0.7, 0.2, -1.0], [0.5, 0.9, 1.0]]

    def run():
        env = SkillMacroEnv(_flatten()); env.reset(seed=3); out = []
        for a in actions:
            o, r, te, tr, _ = env.step(a); out.append((tuple(round(x, 6) for x in o.tolist()), round(r, 6)))
            if te or tr: break
        return out
    assert run() == run()


def test_action_clipped():
    env = SkillMacroEnv(_flatten()); env.reset(seed=0)
    o, _, _, _, _ = env.step([9.0, -9.0, 9.0])
    assert np.all(np.isfinite(o))


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn(); print(f"[PASS] {fn.__name__}")
    print(f"\n{len(fns)}/{len(fns)} skill_env checks passed.")


if __name__ == "__main__":
    _run_all()
