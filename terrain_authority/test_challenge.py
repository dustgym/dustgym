"""Tests for the challenge system (challenge.py) — M1 / H1-H3.

Host-runnable + pytest-discoverable. Covers the declarative schema (JSON round-trip),
the deterministic seed -> map/target generator, and the terrain-matching scorer.
"""
from __future__ import annotations

import json
import math

import numpy as np

from . import challenge as ch


def _sample(objective_type="flatten_pad", **kw):
    return ch.Challenge(
        id="t1", name="test", difficulty_tier=1,
        map=ch.MapSpec(seed=7, base="bumps", grid=48, cell_m=0.02, randomize_soil=True),
        objective=ch.Objective(type=objective_type, region=(16, 16, 32, 32),
                               goal_rc=(24, 40), target_delta_m=0.05, tolerance_m=0.01),
        **kw,
    )


# -- H1: schema + JSON round-trip --------------------------------------------

def test_challenge_json_roundtrip():
    c = _sample()
    back = ch.Challenge.from_dict(json.loads(c.to_json()))
    assert back == c
    # tuples survive (regions/goal come back as tuples, not lists)
    assert isinstance(back.objective.region, tuple)
    assert back.map.seed == 7


def test_challenge_rejects_unknown_objective():
    try:
        ch.realize(_sample(objective_type="teleport"))
    except ValueError as e:
        assert "teleport" in str(e)
    else:
        raise AssertionError("expected ValueError on unknown objective type")


# -- H2: deterministic generator ---------------------------------------------

def test_realize_deterministic():
    c = _sample()
    a = ch.realize(c)
    b = ch.realize(c)
    assert np.array_equal(a.cs.mass_areal, b.cs.mass_areal)
    assert np.array_equal(a.cs.datum, b.cs.datum)
    assert np.array_equal(a.target_height, b.target_height)
    assert a.params == b.params               # DR sampled identically from the seed


def test_realize_seed_changes_map():
    a = ch.realize(_sample())
    c2 = _sample()
    c2 = ch.Challenge.from_dict({**c2.to_dict(), "map": {**c2.map.to_dict(), "seed": 99}})
    b = ch.realize(c2)
    assert not np.array_equal(a.cs.datum, b.cs.datum) or not np.array_equal(
        a.target_height, b.target_height)


def test_realize_map_finite_and_positive_mass():
    inst = ch.realize(_sample())
    H = inst.cs.derive_height()
    assert np.all(np.isfinite(H))
    assert inst.cs.total_mass() > 0.0


# -- objective targets --------------------------------------------------------

def test_flatten_target_is_flat_in_region():
    inst = ch.realize(_sample(objective_type="flatten_pad"))
    r0, c0, r1, c1 = (16, 16, 32, 32)
    sub = inst.target_height[r0:r1, c0:c1]
    assert sub.std() < 1e-9                    # target is constant over the work region
    # and it equals the base region mean (mass-neutral flatten target)
    base = inst.base_height[r0:r1, c0:c1]
    assert math.isclose(sub.mean(), base.mean(), rel_tol=1e-6)


def test_berm_target_raised_in_region():
    inst = ch.realize(_sample(objective_type="build_berm"))
    r0, c0, r1, c1 = (16, 16, 32, 32)
    delta = inst.target_height[r0:r1, c0:c1] - inst.base_height[r0:r1, c0:c1]
    assert np.allclose(delta, 0.05, atol=1e-9)  # ridge of target_delta_m


def test_traverse_has_goal_no_target():
    inst = ch.realize(_sample(objective_type="traverse"))
    assert inst.target_height is None
    assert inst.goal_rc == (24, 40)


# -- scoring ------------------------------------------------------------------

def test_score_terrain_rmse_zero_on_match():
    inst = ch.realize(_sample(objective_type="flatten_pad"))
    rmse = ch.terrain_rmse(inst.target_height, inst.target_height, inst.objective.region)
    assert rmse == 0.0


def test_score_terrain_rmse_known():
    target = np.zeros((8, 8))
    achieved = np.zeros((8, 8))
    achieved[2:4, 2:4] = 0.1                    # 4 cells off by 0.1 inside a 4x4 region
    rmse = ch.terrain_rmse(achieved, target, (2, 2, 6, 6))
    # region is 4x4=16 cells, 4 of them at 0.1 -> rmse = sqrt(4*0.01/16) = 0.05
    assert math.isclose(rmse, 0.05, rel_tol=1e-9)


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn()
        print(f"[PASS] {fn.__name__}")
    print(f"\n{len(fns)}/{len(fns)} challenge checks passed.")


if __name__ == "__main__":
    _run_all()
