"""Tests for the self-optimizing pipeline: execute -> observe -> learn the slip energy model -> generalize.

Grounded in the conserved drive_step (slip + Material) + ipex energy; the only learned thing is the
inflation(slope) regression. No synthetic data.
"""
from __future__ import annotations

from . import self_optimizing as so

_TRAIN = [2, 6, 9, 12, 16, 19, 24, 28]
_TEST = [4, 11, 18, 26]


def test_inflation_rises_with_slope():
    def infl(s):
        fj, tj = so.execute_leg_energy(s)
        return tj / fj
    assert infl(0) < infl(10) < infl(20) < infl(28)          # slip + climb -> steeper costs more energy


def test_self_learning_reduces_held_out_error():
    hist, model, truth = so.run_self_optimizing(_TRAIN, _TEST, seed=1)
    early = hist[1]["held_out_mape"]                          # 2 obs: pre-fit, naive flat model
    late = hist[-1]["held_out_mape"]                          # all obs: learned
    assert late < early                                      # the model self-learns + generalizes
    assert late < 0.05                                       # held-out slopes predicted within 5%
    assert abs(model.predict(11) - truth[11]) / truth[11] < 0.05   # a specific held-out slope


def test_learned_model_optimizes_a_route_the_naive_cannot():
    _, model, _ = so.run_self_optimizing(_TRAIN, [10], seed=2)
    flat_route, steep_route = [2, 2, 2, 2], [24, 24, 24, 24]   # equal distance, very different grade
    assert so.route_energy(flat_route) == so.route_energy(steep_route)        # naive: slope-blind -> equal
    assert so.route_energy(steep_route, model) > so.route_energy(flat_route, model) * 1.2   # learned: steep costs more


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"ok  {name}")
    print("self_optimizing: all checks passed")
