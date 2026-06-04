#!/usr/bin/env python3
"""Self-optimizing figure: the pipeline learns its own slip energy model from execution.

Left: held-out prediction error collapses as the model sees more executed slopes (self-learning +
generalization). Right: the learned inflation(slope) curve matches the executed truth -- so the planner
can predict per-leg energy on any grade and route around the expensive ones. Grounded in the conserved
drive_step (slip + Material) + ipex energy; only the inflation regression is learned. No synthetic data.
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
from terrain_authority import self_optimizing as so  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)

    train = [2, 5, 8, 11, 14, 17, 20, 23, 26, 29]
    test = [4, 10, 16, 22, 28]
    hist, model, truth = so.run_self_optimizing(train, test, seed=1)
    n = [h["n_obs"] for h in hist]
    mape = [h["held_out_mape"] * 100 for h in hist]

    slopes = np.linspace(0, 30, 60)
    learned = [model.predict(s) for s in slopes]
    true_inf = [so.execute_leg_energy(s)[1] / so.execute_leg_energy(s)[0] for s in slopes]

    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
    ax[0].plot(n, mape, "o-", color="C0")
    ax[0].set_yscale("log")
    ax[0].set_xlabel("executed legs observed")
    ax[0].set_ylabel("held-out prediction error [% MAPE]")
    ax[0].set_title("Self-learning: held-out energy error\ncollapses as the model sees more execution")
    ax[0].grid(alpha=0.3, which="both")
    ax[1].plot(slopes, true_inf, "-", color="0.5", lw=3, label="true (executed slip dynamics)")
    ax[1].plot(slopes, learned, "--", color="C3", lw=2, label="learned inflation(slope)")
    ax[1].plot(list(truth), [truth[s] for s in truth], "s", color="C2", ms=7, label="held-out test slopes")
    ax[1].set_xlabel("leg slope [deg]")
    ax[1].set_ylabel("energy inflation (true / flat)")
    ax[1].set_title("The learned model matches the executed truth\n(so the planner routes around steep grades)")
    ax[1].legend()
    ax[1].grid(alpha=0.3)
    fig.suptitle("Self-optimizing pipeline: learn the slip energy model from execution, then plan with it "
                 "(conserved drive_step + Material + ipex energy)", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(args.out, dpi=130)
    plt.close(fig)
    print(f"wrote {args.out}  (held-out MAPE {mape[0]:.1f}% -> {mape[-1]:.1f}%)")


if __name__ == "__main__":
    main()
