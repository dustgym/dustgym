"""Cross-Entropy-Method policy training in RoverSimEnv (Phase 5 capstone, 2026-06-02).

Pure-numpy CEM optimizing a linear policy over the env observation -> twist action.
NO external RL library, so it trains on the bare interpreter. The point is to show
RoverSimEnv is TRAINABLE, not just runnable: a random linear policy flails (wrong
heading, truncates, low/negative return); a CEM-trained policy learns to orient and
drive to the goal while keeping slip down, scoring far higher across domain-randomized
terrains.

Policy: a(obs) = tanh(W @ [obs, 1]) in [-1, 1]^2, W shape (act_dim, obs_dim+1).
Deterministic given the CEM seed + the evaluation seeds (reproducible training).
"""
from __future__ import annotations

import numpy as np

from .rover_env import RoverSimEnv


class LinearPolicy:
    """Affine map observation -> tanh-squashed twist action (theta is a flat vector)."""

    def __init__(self, obs_dim: int, act_dim: int = 2):
        self.obs_dim = int(obs_dim)
        self.act_dim = int(act_dim)
        self.n_params = self.act_dim * (self.obs_dim + 1)

    def act(self, theta: np.ndarray, obs: np.ndarray) -> np.ndarray:
        W = np.asarray(theta, dtype=np.float64).reshape(self.act_dim, self.obs_dim + 1)
        x = np.concatenate([np.asarray(obs, dtype=np.float64), [1.0]])
        return np.tanh(W @ x)


def rollout(env: RoverSimEnv, policy: LinearPolicy, theta: np.ndarray, seed: int) -> dict:
    """Run one episode under the policy; return {return, reached, steps, entrapped}."""
    obs, _ = env.reset(seed=seed)
    total = 0.0
    info = {}
    steps = 0
    while True:
        obs, r, terminated, truncated, info = env.step(policy.act(theta, obs))
        total += r
        steps += 1
        if terminated or truncated:
            break
    return {"return": total, "reached": bool(info.get("reached_goal", False)),
            "steps": steps, "entrapped": bool(info["telem"]["entrapped"])}


def evaluate(env: RoverSimEnv, policy: LinearPolicy, theta: np.ndarray, seeds) -> dict:
    """Mean return / reached-rate over a fixed set of episode seeds (same terrains
    for every candidate, so scores are comparable)."""
    outs = [rollout(env, policy, theta, s) for s in seeds]
    return {"mean_return": float(np.mean([o["return"] for o in outs])),
            "reached_rate": float(np.mean([o["reached"] for o in outs]))}


def train_cem(env: RoverSimEnv, *, iters: int = 25, pop: int = 40, elite_frac: float = 0.2,
              sigma0: float = 1.0, eval_seeds=(0, 1, 2, 3), rng_seed: int = 0) -> dict:
    """Train a LinearPolicy with the cross-entropy method.

    Each iteration samples ``pop`` parameter vectors from N(mean, sigma^2), scores each
    by mean return over ``eval_seeds`` episodes, refits (mean, sigma) to the top
    ``elite_frac``. Deterministic given rng_seed + eval_seeds. Returns
    {best_theta, history (best mean_return per iter), policy, final}.
    """
    rng = np.random.default_rng(rng_seed)
    policy = LinearPolicy(env.obs_dim, env.action_dim)
    mean = np.zeros(policy.n_params)
    sigma = np.full(policy.n_params, float(sigma0))
    n_elite = max(2, int(pop * elite_frac))
    history = []
    best_theta = mean.copy()
    best_score = -np.inf
    for _ in range(iters):
        cand = mean[None, :] + sigma[None, :] * rng.standard_normal((pop, policy.n_params))
        scores = np.array([evaluate(env, policy, c, eval_seeds)["mean_return"] for c in cand])
        order = np.argsort(scores)
        elite = cand[order[-n_elite:]]
        mean = elite.mean(axis=0)
        sigma = elite.std(axis=0) + 1e-3
        it_best = float(scores.max())
        if it_best > best_score:
            best_score = it_best
            best_theta = cand[order[-1]].copy()
        history.append(best_score)
    return {"best_theta": best_theta, "history": history, "policy": policy,
            "final": evaluate(env, policy, best_theta, eval_seeds)}


def random_baseline(env: RoverSimEnv, policy: LinearPolicy, eval_seeds, *, rng_seed: int = 123) -> dict:
    """Mean return of a random linear policy (the untrained reference)."""
    rng = np.random.default_rng(rng_seed)
    theta = rng.standard_normal(policy.n_params)
    return evaluate(env, policy, theta, eval_seeds)


if __name__ == "__main__":   # quick demo training on a small env
    env = RoverSimEnv(grid=48, start_col=8, goal_col=40, max_steps=60,
                      randomize=True, slope_max_deg=22.0)
    res = train_cem(env, iters=20, pop=32, eval_seeds=(0, 1, 2, 3, 4))
    base = random_baseline(env, res["policy"], (0, 1, 2, 3, 4))
    print(f"random baseline : mean_return={base['mean_return']:.2f}  reached={base['reached_rate']:.0%}")
    print(f"CEM-trained     : mean_return={res['final']['mean_return']:.2f}  reached={res['final']['reached_rate']:.0%}")
    print(f"learning curve  : {[round(h, 1) for h in res['history']]}")
