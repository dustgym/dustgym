#!/usr/bin/env python3
"""Active-perception figure: greedy next-best-view vs random, on the map-channel reward.

Left: per-cell uncertainty (sigma_mean) falls faster per joule under greedy next-best-view than random.
Right: the greedy run's final uncertainty field (where the map is confident vs still needs observing).
Grounded: real authority fbm terrain, the measured stereo sigma + ipex drive energy. No synthetic data.
"""
from __future__ import annotations

import argparse
import os
import sys

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from terrain_authority.active_perception_env import ActivePerceptionEnv, greedy_action  # noqa: E402


def rollout(policy, *, seed=3, grid=24, charges=0.05, steps=160):
    env = ActivePerceptionEnv(grid=grid, charges=charges, seed=seed)
    env.reset(seed=seed)
    rng = np.random.default_rng(seed)
    spent, sig = [0.0], [float(env.sigma.mean())]
    for _ in range(steps):
        a = greedy_action(env) if policy == "greedy" else int(rng.integers(4))
        r = env.step(a)
        spent.append(env.energy_budget - env.energy)
        sig.append(r[-1]["sigma_mean"])
        if r[2] or r[3]:
            break
    return np.array(spent), np.array(sig), env


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    g_e, g_s, g_env = rollout("greedy")
    r_e, r_s, _ = rollout("random")

    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
    ax[0].plot(g_e / 1e3, g_s, "o-", ms=3, color="C2", label="greedy next-best-view")
    ax[0].plot(r_e / 1e3, r_s, "s-", ms=3, color="C3", label="random")
    ax[0].set_xlabel("energy spent driving [kJ]")
    ax[0].set_ylabel("mean per-cell uncertainty σ [m]")
    ax[0].set_title("Map uncertainty falls faster per joule\nunder active perception")
    ax[0].legend()
    ax[0].grid(alpha=0.3)
    im = ax[1].imshow(g_env.sigma, cmap="magma_r", vmin=0.0)
    ax[1].set_title("Greedy final uncertainty field\n(dark = mapped, bright = observe more)")
    ax[1].set_xticks([])
    ax[1].set_yticks([])
    fig.colorbar(im, ax=ax[1], fraction=0.046, label="σ [m]")
    fig.suptitle("Active perception: the map channel / Uncertainty layer as the RL reward "
                 "(real authority terrain, measured stereo σ + ipex energy)", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(args.out, dpi=130)
    plt.close(fig)
    print(f"wrote {args.out}  (greedy σ {g_s[-1]:.3f} vs random {r_s[-1]:.3f})")


if __name__ == "__main__":
    main()
