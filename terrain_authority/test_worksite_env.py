"""Tests for WorkSiteConstructEnv — the RL controller over John's WorkSite seam (reconciliation).

Asserts the honest core: actions drive WorkSite.flatten/dump, mass is conserved through the GLOBAL
drum ledger (grid + inventory_kg), greedy batching solves a feasible instance, and the cut-vs-dump
ordering matters under a binding budget (greedy >> random). Numpy-only (no gymnasium) -> core suite.
"""
from __future__ import annotations

import math

import numpy as np

from terrain_authority.worksite_env import (WorkSiteConstructEnv, beam_worksite_plan,
                                            greedy_worksite)


def mk(**kw):
    return WorkSiteConstructEnv(**kw)


def test_mass_conserved_through_ledger():
    """grid_mass + inventory_kg is invariant across flatten/dump (WorkSite's contract, exercised by RL)."""
    env = mk(); env.reset(seed=0); m0 = env.ws.total_mass()
    rng = np.random.default_rng(0)
    for _ in range(env.max_steps):
        _, _, te, tr, _ = env.step(rng.integers(2))
        if te or tr:
            break
    assert math.isclose(env.ws.total_mass(), m0, rel_tol=1e-9)


def test_actions_drive_worksite_seam():
    """flatten action moves pad mass into the ledger; dump action moves it back onto the berm grid."""
    env = mk(); env.reset(seed=0)
    inv0 = env.ws.inventory_kg
    env.step(0)                                       # flatten a pad slice -> ledger grows
    assert env.ws.inventory_kg > inv0
    before_def = env._berm_deficit_kg()
    env.step(1)                                       # dump a berm slice -> deficit shrinks
    assert env._berm_deficit_kg() <= before_def


def test_greedy_solves_feasible_instance():
    env = mk(); env.reset(seed=0); done = False; info = {}
    while not done:
        _, _, te, tr, info = env.step(greedy_worksite(env)); done = te or tr
    assert info["success"], info


def test_greedy_beats_random_under_budget():
    """Ordering matters: batch flatten->dump (greedy) solves far more often than random within the budget."""
    def rate(pol, n=16, s0=100):
        out = []
        for i in range(n):
            e = mk(); e.reset(seed=s0 + i); done = False; info = {}
            while not done:
                _, _, te, tr, info = e.step(pol(e)); done = te or tr
            out.append(bool(info["success"]))
        return float(np.mean(out))
    rng = np.random.default_rng(1)
    g = rate(lambda e: greedy_worksite(e))
    r = rate(lambda e: rng.integers(2))
    assert g >= 0.8 and g >= r + 0.2, (g, r)         # greedy nearly always solves; random lags clearly


def test_haworth_balanced_cut_haul_fill_battery_bound():
    """Real LOLA Haworth, flat site, mass-balanced cut-haul-fill on a PHYSICS battery budget (not steps):
    greedy solves it, mass conserved, and the IPEx battery genuinely binds. Skips without the DEM bundle."""
    import os
    bundle = "samples/lunar_dem/haworth_10km_5m"
    if not os.path.isdir(bundle):
        import pytest
        pytest.skip("Haworth DEM bundle not present (in the repo's samples/, not the installed dustgym package)")

    def mkh(charges):
        return WorkSiteConstructEnv(bundle_dir=bundle, fine_cell_m=0.1, flat_window=True,
                                    cut_depth_m=0.05, work_cells=20, n_slices=6,
                                    charges=charges, max_steps=200)
    # solvable + mass-conserved on a comfortable (1-charge) budget
    env = mkh(1); env.reset(seed=0); m0 = env.ws.total_mass(); done = False; info = {}
    while not done:
        _, _, te, tr, info = env.step(greedy_worksite(env)); done = te or tr
    assert info["success"], info
    assert math.isclose(env.ws.total_mass(), m0, rel_tol=1e-9)
    # the BATTERY binds: a too-small charge runs the rover out of energy mid-task
    env = mkh(0.2); env.reset(seed=0); done = False; info = {}
    while not done:
        _, _, te, tr, info = env.step(greedy_worksite(env)); done = te or tr
    assert info["out_of_energy"] and not info["success"], info


def test_beam_plan_solves_and_conserves_mass():
    """Model-based beam search finds a valid success on a feasible instance, mass conserved via the ledger."""
    env = mk(); env.reset(seed=0); m0 = env.ws.total_mass()
    plan = beam_worksite_plan(env, width=12)
    assert plan, "beam found no success within budget on seed 0"
    env2 = mk(); env2.reset(seed=0); info = {}
    for a in plan:
        _, _, te, tr, info = env2.step(a)
        if te or tr:
            break
    assert info["success"]
    assert math.isclose(env2.ws.total_mass(), m0, rel_tol=1e-9)
