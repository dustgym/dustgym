#!/usr/bin/env python3
"""Full Gymnasium + Stable-Baselines3 PPO training in RoverSimEnv (Phase 5, 2026-06-02).

The real deep-RL integration: RoverSimEnv is a gymnasium.Env (passes the official
env_checker), so any standard RL library drives it. Here SB3 PPO learns slip-aware
drive-to-goal across domain-randomized terrain. Run with an interpreter that has
gymnasium + stable-baselines3 + torch (the repo's runtime venv does):

    PYTHONPATH=<repo> <venv>/bin/python scripts/demo/train_ppo.py --timesteps 40000

SB3/torch are OPTIONAL run-time deps (imported here, not in the core package).
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import numpy as np  # noqa: E402

from terrain_authority.rover_env import RoverSimEnv  # noqa: E402


def make_env(**kw):
    return RoverSimEnv(grid=48, start_col=8, goal_col=40, goal_radius_cells=2.0,
                       max_steps=60, randomize=True, slope_max_deg=20.0, **kw)


def evaluate(model, n=24, seed0=10_000):
    env = make_env()
    rets, reached = [], []
    for i in range(n):
        obs, info = env.reset(seed=seed0 + i)
        total, done = 0.0, False
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, r, term, trunc, info = env.step(action)
            total += r
            done = term or trunc
        rets.append(total)
        reached.append(float(info["reached_goal"]))
    return float(np.mean(rets)), float(np.mean(reached))


def main():
    from stable_baselines3 import PPO
    from stable_baselines3.common.monitor import Monitor
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.utils import set_random_seed

    ap = argparse.ArgumentParser()
    ap.add_argument("--timesteps", type=int, default=40_000)
    ap.add_argument("--out", default="/tmp/ppo_rover")
    ap.add_argument("--viz", default="/mnt/projects/foss_ipex/ppo_training.png")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    set_random_seed(0)

    class RewardLog(BaseCallback):
        def __init__(self):
            super().__init__()
            self.ep_rewards = []
        def _on_step(self):
            for info in self.locals.get("infos", []):
                if "episode" in info:
                    self.ep_rewards.append(info["episode"]["r"])
            return True

    env = Monitor(make_env())
    model = PPO("MlpPolicy", env, seed=0, verbose=0, device="cpu",
                n_steps=512, batch_size=128, gamma=0.99, gae_lambda=0.95)

    base_ret, base_reach = evaluate(model)                 # untrained policy
    cb = RewardLog()
    model.learn(total_timesteps=args.timesteps, callback=cb, progress_bar=False)
    tr_ret, tr_reach = evaluate(model)
    model.save(os.path.join(args.out, "ppo_rover"))

    print(f"PPO untrained : return={base_ret:6.2f}  reached={base_reach:.0%}")
    print(f"PPO trained   : return={tr_ret:6.2f}  reached={tr_reach:.0%}  ({args.timesteps} timesteps, {len(cb.ep_rewards)} eps)")

    # learning curve (smoothed episode reward)
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        r = np.array(cb.ep_rewards, dtype=float)
        if r.size >= 5:
            k = max(1, r.size // 40)
            sm = np.convolve(r, np.ones(k) / k, mode="valid")
            plt.figure(figsize=(7, 4.2))
            plt.plot(r, alpha=0.25, color="C0", label="episode reward")
            plt.plot(np.arange(len(sm)) + k - 1, sm, color="C0", lw=2, label=f"smoothed (k={k})")
            plt.title(f"SB3 PPO on RoverSimEnv: reached {base_reach:.0%} -> {tr_reach:.0%}")
            plt.xlabel("training episode"); plt.ylabel("episode return"); plt.legend(fontsize=8)
            plt.tight_layout(); plt.savefig(args.viz, dpi=110)
            print(f"saved {args.viz}")
    except Exception as e:
        print("viz skipped:", e)


if __name__ == "__main__":
    main()
