"""Tests for the active-perception env (next-best-view mapping; the map channel as the RL reward).

Bare-numpy core (observe reduces uncertainty; greedy next-best-view beats random; energy-bounded) plus
the gymnasium env_checker + registration when gymnasium is present. Real authority terrain, no synthetic.
"""
from __future__ import annotations

import numpy as np

from . import active_perception_env as ap


def _run(policy, *, seed=3, grid=20, charges=0.05, steps=120):
    env = ap.ActivePerceptionEnv(grid=grid, charges=charges, seed=seed)
    env.reset(seed=seed)
    rng = np.random.default_rng(seed)
    tot, info = 0.0, {}
    for _ in range(steps):
        if policy == "greedy":
            a = ap.greedy_action(env)
        elif policy == "beam":
            a = ap.beam_action(env, lookahead=5, beam_width=8)
        else:
            a = int(rng.integers(4))
        r = env.step(a)
        tot += r[1]
        info = r[-1]
        if (r[2] if len(r) == 4 else (r[2] or r[3])):
            break
    return info, tot


def test_observe_reduces_uncertainty():
    env = ap.ActivePerceptionEnv(grid=16, seed=1)
    s0 = float(env.sigma.mean())
    env.step(0)
    assert float(env.sigma.mean()) < s0                       # an observation reduces uncertainty


def test_greedy_beats_random_on_mapping():
    g, g_tot = _run("greedy")
    r, r_tot = _run("random")
    assert g["sigma_mean"] < r["sigma_mean"]                  # greedy maps to lower uncertainty
    assert g["map_rmse_m"] < r["map_rmse_m"]                  # and lower height RMSE
    assert g_tot > r_tot                                      # and higher info-per-joule reward


def test_beam_matches_greedy_submodular():
    # next-best-view info-gain is SUBMODULAR -> greedy is near-optimal (the 1-1/e guarantee), so multi-step
    # beam does NOT meaningfully beat it. The honest result: model-based search helps the scheduler (real
    # multi-step routing headroom), not active perception (a submodular coverage problem).
    g, _ = _run("greedy")
    b, _ = _run("beam")
    r, _ = _run("random")
    assert g["sigma_mean"] < r["sigma_mean"] and b["sigma_mean"] < r["sigma_mean"]   # both crush random
    assert abs(b["sigma_mean"] - g["sigma_mean"]) < 0.10                              # beam ~= greedy (no headroom)


def test_energy_bounded_and_terminates():
    env = ap.ActivePerceptionEnv(grid=16, charges=0.02, seed=2)
    env.reset(seed=2)
    done = False
    for _ in range(3000):
        r = env.step(ap.greedy_action(env))
        done = r[2] if len(r) == 4 else (r[2] or r[3])
        if done:
            break
    assert done                                               # finishes (mapped or out of energy)
    assert env.energy <= env.energy_budget                    # never gains energy


def test_passes_gym_env_checker_and_registers():
    import pytest
    gym = pytest.importorskip("gymnasium")
    from gymnasium.utils.env_checker import check_env
    check_env(ap.ActivePerceptionEnv(grid=12, seed=0), skip_render_check=True)
    from . import registration
    registration.register_envs()
    env = gym.make("Dust/ActivePerception-v0")
    obs, _ = env.reset(seed=0)
    assert obs.shape == env.observation_space.shape
    env.step(env.action_space.sample())
    env.close()


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"ok  {name}")
            except BaseException as e:                         # noqa: BLE001 -- report skips in the runner
                if type(e).__name__ == "Skipped":
                    print(f"skip {name}: {e}")
                else:
                    raise
    print("active_perception_env: all checks passed")
