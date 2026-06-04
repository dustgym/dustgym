#!/usr/bin/env python3
"""M2: train PPO on the SKILL-MACRO env (select cell + cut/dump toward target).
PYTHONPATH=<repo> <venv>/bin/python scripts/demo/train_skill_macro.py --timesteps 80000"""
import argparse, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
import numpy as np
from terrain_authority import challenge as ch
from terrain_authority.skill_env import SkillMacroEnv

def mk():
    c = ch.Challenge(id="m2s", name="flatten", difficulty_tier=2,
        map=ch.MapSpec(seed=2, base="bumps", grid=44, roughness_m=0.004),
        objective=ch.Objective(type="flatten_pad", region=(14,14,30,30), tolerance_m=0.01),
        constraints=ch.Constraints(max_time_steps=120))
    return SkillMacroEnv(c)

def ev(policy, n=12, seed0=7000):
    env=mk(); rm=[]; sc=[]
    for i in range(n):
        o,info=env.reset(seed=seed0+i); done=False
        while not done:
            a = policy.predict(o,deterministic=True)[0] if hasattr(policy,"predict") else policy(env,o)
            o,r,te,tr,info=env.step(a); done=te or tr
        rm.append(info["rmse"]); sc.append(float(info["success"]))
    return float(np.mean(rm)), float(np.mean(sc))

def main():
    from stable_baselines3 import PPO
    from stable_baselines3.common.monitor import Monitor
    from stable_baselines3.common.utils import set_random_seed
    ap=argparse.ArgumentParser(); ap.add_argument("--timesteps",type=int,default=80000); args=ap.parse_args()
    set_random_seed(0)
    rng=np.random.default_rng(0)
    rand=lambda env,o: rng.uniform(-1,1,3)
    base_rmse, base_succ = ev(rand)
    model=PPO("MlpPolicy", Monitor(mk()), seed=0, verbose=0, device="cpu",
              n_steps=1024, batch_size=128, gamma=0.99, ent_coef=0.01)
    model.learn(total_timesteps=args.timesteps, progress_bar=False)
    tr_rmse, tr_succ = ev(model)
    print(f"random-macro baseline: RMSE={base_rmse:.4f}  success={base_succ:.0%}")
    print(f"PPO skill-macro      : RMSE={tr_rmse:.4f}  success={tr_succ:.0%}  ({args.timesteps} steps)")
if __name__=="__main__": main()
