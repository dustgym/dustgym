"""Tests for the challenge runner (challenge_runner.py) — M1 / H3.

Host-runnable + pytest-discoverable. Runs an agent (a callable obs->action) against a
challenge and returns a deterministic Scorecard. Covers well-formedness, determinism,
better-agent-scores-higher, the authored library, and Scorecard JSON.
"""
from __future__ import annotations

import json
import math

from . import challenge as ch
from . import challenge_runner as cr


def _traverse(max_steps=120, grid=48):
    return ch.Challenge(id="trav", name="t", difficulty_tier=1,
                        map=ch.MapSpec(seed=1, base="flat", grid=grid),
                        objective=ch.Objective(type="traverse", region=(0, 0, grid, grid),
                                               goal_rc=(grid // 2, grid - 8)),
                        constraints=ch.Constraints(max_time_steps=max_steps))


def FORWARD(obs):
    return [1.0, 0.0, 0.0]


def NOOP(obs):
    return [0.0, 0.0, 0.0]


def test_run_returns_scorecard():
    sc = cr.run(FORWARD, _traverse())
    assert sc.challenge_id == "trav" and sc.objective_type == "traverse"
    assert isinstance(sc.success, bool)
    assert math.isfinite(sc.score) and sc.steps >= 1


def test_run_deterministic():
    a = cr.run(FORWARD, _traverse())
    b = cr.run(FORWARD, _traverse())
    assert a.to_dict() == b.to_dict()


def test_better_agent_scores_higher():
    good = cr.run(FORWARD, _traverse())
    bad = cr.run(NOOP, _traverse())
    assert good.success and not bad.success
    assert good.score > bad.score


def test_run_authored_library():
    cards = [cr.run(FORWARD, c) for c in ch.authored_challenges()]
    assert len(cards) == 3
    for sc in cards:
        assert math.isfinite(sc.score) and sc.steps >= 1
        assert sc.objective_type in ch.OBJECTIVE_TYPES


def test_scorecard_json_roundtrip():
    sc = cr.run(FORWARD, _traverse())
    d = json.loads(json.dumps(sc.to_dict()))
    assert d["challenge_id"] == "trav" and "score" in d


def _run_all():
    fns = [v for k, v in sorted(globals().items())
           if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn()
        print(f"[PASS] {fn.__name__}")
    print(f"\n{len(fns)}/{len(fns)} challenge_runner checks passed.")


if __name__ == "__main__":
    _run_all()
