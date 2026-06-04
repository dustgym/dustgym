#!/usr/bin/env python3
"""Real-Haworth cut-haul-fill on a PHYSICS-grounded battery budget (answers "why N steps?").

Drives John's WorkSite on the real LOLA Haworth DEM: finds a flat work site, cuts a uniform layer off a
pad, hauls it through the drum ledger, and raises a mass-balanced berm -- with the episode bounded by
ENERGY from the IPEx battery (ipex_specs: 1332 Wh/charge), not a step count. Each flatten/dump spends
dig_energy_per_kg * mass + travel; the rover runs until the battery is spent. So the budget is joules, and
"how much can you build" falls out of the physics, not a tuned number.

Observed (12 flat Haworth sites): greedy solves 12/12, mass conserved, ~11 trip-legs, energy ~2.0 MJ =
~43% of a battery charge. The budget BINDS: success at >=0.5 charge; out_of_energy below ~0.43 charge.

Run (needs the committed Haworth DEM bundle under samples/):
    PYTHONPATH=<repo> python scripts/demo/worksite_haworth.py
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import numpy as np

from terrain_authority import ipex_specs as ix
from terrain_authority.worksite_env import WorkSiteConstructEnv, greedy_worksite

BUNDLE = "samples/lunar_dem/haworth_10km_5m"


def mk(charges, max_steps=200):
    return WorkSiteConstructEnv(bundle_dir=BUNDLE, fine_cell_m=0.1, flat_window=True,
                               cut_depth_m=0.05, work_cells=20, n_slices=6,
                               charges=charges, max_steps=max_steps)


def run(charges, seed):
    env = mk(charges); env.reset(seed=seed); m0 = env.ws.total_mass()
    budget = env.energy_budget_j; done = False; info = {}
    while not done:
        _, _, te, tr, info = env.step(greedy_worksite(env)); done = te or tr
    return info, budget - info["energy_j"], abs(env.ws.total_mass() - m0) < 1e-6


def main():
    if not os.path.isdir(BUNDLE):
        print(f"Haworth DEM bundle not found at {BUNDLE} (ships with the roversim repo). Skipping.")
        return
    print(f"IPEx battery: {ix.battery_energy_wh():.0f} Wh = {ix.battery_energy_j()/1e6:.2f} MJ/charge; "
          f"dig {ix.dig_energy_per_kg():.0f} J/kg, drive {ix.drive_energy_per_m():.0f} J/m")
    ok, legs, used = 0, [], []
    for s in range(12):
        info, e_used, mass_ok = run(1, s)
        ok += info["success"]; assert mass_ok
        if info["success"]:
            legs.append(info["steps"]); used.append(e_used)
    print(f"\nreal Haworth, flat sites, 1-charge budget: greedy {ok}/12 solved, mass conserved all")
    print(f"  ~{np.mean(legs):.0f} trip-legs, energy {np.mean(used)/1e6:.2f} MJ "
          f"({np.mean(used)/ix.battery_energy_j()*100:.0f}% of a charge)")
    print("\nthe BATTERY is the budget (not a step count):")
    for ch in (0.5, 0.3, 0.2):
        info, _, _ = run(ch, 0)
        print(f"  {ch:.1f} charge ({ch*ix.battery_energy_j()/1e6:.2f} MJ): "
              f"success={info['success']}  out_of_energy={info['out_of_energy']}  legs={info['steps']}")
    print("\n=> 'why N steps?' answered: the episode is bounded by joules from the IPEx pack; a berm costs "
          "~0.43 charge, and below that the rover runs out of battery mid-task.")


if __name__ == "__main__":
    main()
