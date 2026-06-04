#!/usr/bin/env python3
"""M2: train a construction skill (flatten a pad) with SB3 PPO on TerrainTargetEnv.

Run with the runtime venv (gymnasium + SB3 + torch):
  PYTHONPATH=<repo> <venv>/bin/python scripts/demo/train_construction.py --timesteps 120000
Measures trained-vs-baseline region H-RMSE + success. Honest: long-horizon, iterate.
"""
import argparse, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
import numpy as np
from terrain_authority import challenge as ch
from terrain_authority.terrain_target_env import TerrainTargetEnv

def make_challenge(grid=44, region=(14,14,30,30), tol=0.02, max_steps=220, seed=2, rough=0.004):
    return ch.Challenge(id="flatten_train", name="flatten", difficulty_tier=2,
        map=ch.MapSpec(seed=seed, base="mound", grid=grid, roughness_m=rough),
        objective=ch.Objective(type="flatten_pad", region=region, tolerance_m=tol),
        constraints=ch.Constraints(max_time_steps=max_steps))

def make_env(): return TerrainTargetEnv(make_challenge())

def evaluate(policy, n=12, seed0=5000):
    env = make_env(); rmses=[]; succ=[]; init=[]
    for i in range(n):
        obs, info = env.reset(seed=seed0+i); init.append(info["rmse"]); done=False
        while not done:
            a = policy.predict(obs, deterministic=True)[0] if hasattr(policy,"predict") else policy(obs)
            obs, r, te, tr, info = env.step(a); done = te or tr
        rmses.append(info["rmse"]); succ.append(float(info["success"]))
    return float(np.mean(init)), float(np.mean(rmses)), float(np.mean(succ))

def main():
    from stable_baselines3 import PPO
    from stable_baselines3.common.monitor import Monitor
    ap = argparse.ArgumentParser(); ap.add_argument("--timesteps", type=int, default=120000)
    ap.add_argument("--out", default="/tmp/m2"); args = ap.parse_args(); os.makedirs(args.out, exist_ok=True)
    noop = lambda o: [0.0,0.0,0.0]
    init0, base_rmse, base_succ = evaluate(noop)
    model = PPO("MlpPolicy", Monitor(make_env()), seed=0, verbose=0, device="cpu",
                n_steps=1024, batch_size=128, gamma=0.99, ent_coef=0.005, gae_lambda=0.95)
    model.learn(total_timesteps=args.timesteps, progress_bar=False)
    _, tr_rmse, tr_succ = evaluate(model)
    model.save(os.path.join(args.out, "flatten_ppo"))
    print(f"initial region RMSE (untouched): {init0:.4f} m")
    print(f"baseline (noop): RMSE={base_rmse:.4f}  success={base_succ:.0%}")
    print(f"PPO trained    : RMSE={tr_rmse:.4f}  success={tr_succ:.0%}  ({args.timesteps} steps)")
    impr = 100*(base_rmse - tr_rmse)/base_rmse if base_rmse else 0
    print(f"verdict: trained {'IMPROVES' if tr_rmse < base_rmse else 'DOES NOT IMPROVE'} on baseline (RMSE {impr:+.0f}%)")

if __name__ == "__main__":
    main()
