"""WP0.3 (RB-03) — one immutable PlanResult that totals / report / Plan IR / timeline are VIEWS of.

Asserts: plan() returns an immutable, provenance-stamped result; the legacy plan_and_simulate tuple is a
view of it; and the downstream consumers reuse a passed-in result instead of re-running the planner, so
they describe the SAME plan. No synthetic data — a real two-order moon mission on the conserved authority.
"""
from __future__ import annotations

import dataclasses

import pytest

from planet_browser import mission_planner as MP


def _mission(body="moon"):
    return MP.mission_from_dict({
        "name": "RB03", "body": body, "charger": [0, 0],
        "orders": [
            {"action": "Level pad", "kind": "cut", "x": 40, "y": 30, "footprint_m2": 36, "depth_m": 0.04},
            {"action": "Build berm", "kind": "fill", "x": 44, "y": 44, "footprint_m2": 14, "depth_m": 0.10},
        ],
    })


def test_plan_returns_immutable_result_with_provenance():
    r = MP.plan(_mission())
    assert isinstance(r, MP.PlanResult)
    with pytest.raises(dataclasses.FrozenInstanceError):       # frozen: cannot reassign the plan fields
        r.totals = {}
    p = r.provenance
    assert p["schema_version"] == MP.PLAN_RESULT_VERSION and p["mode"] == "PLAN"
    assert p["config"]["algorithm"] and "vehicles" in p["config"]
    assert len(p["input_sha256"]) == 64                        # a real content hash, not a placeholder


def test_plan_and_simulate_is_a_view_of_plan():
    m = _mission()
    legacy = MP.plan_and_simulate(m)                           # (trips, flows, per_trip, tl, totals)
    r = MP.plan(m)
    assert r.as_tuple()[0] == legacy[0]                        # same trips/order
    assert r.totals["time_s"] == legacy[4]["time_s"]
    assert r.totals["mass_kg"] == legacy[4]["mass_kg"]


def test_consumers_reuse_the_one_result_no_recompute():
    m = _mission()
    r = MP.plan(m)
    # timeline + Plan IR built FROM the shared result describe exactly that plan
    tl = MP.build_timeline(m, result=r)
    assert tl["duration_s"] == round(r.totals["time_s"], 3)    # timeline is a view of r.totals
    ir = MP.plan_ir(m, result=r)
    n_work = sum(1 for a in ir["actions"] if a["op"] in MP._IR_DIG_OPS)
    assert n_work == len(r.trips)                              # one work action per planned trip in r
    assert ir["expect"]["makespan_s"] == pytest.approx(r.totals["makespan_s"])   # IR headline == the one plan


def test_provenance_hash_is_deterministic_and_input_sensitive():
    h1 = MP.plan(_mission()).provenance["input_sha256"]
    h2 = MP.plan(_mission()).provenance["input_sha256"]        # same inputs -> same hash
    assert h1 == h2
    h3 = MP.plan(_mission(), objective="energy").provenance["input_sha256"]   # changed config -> different
    assert h3 != h1


def test_plan_ir_and_timeline_carry_the_plan_provenance():
    # CT-07: the machine-consumable Plan IR + the timeline playback both embed the one plan's provenance
    # (schema/mode/config + input hash), so any emitted artifact is traceable to exactly its inputs.
    m = _mission()
    r = MP.plan(m)
    ir = MP.plan_ir(m, result=r)
    tl = MP.build_timeline(m, result=r)
    assert ir["provenance"] == r.provenance
    assert tl["provenance"]["input_sha256"] == r.provenance["input_sha256"]
    assert ir["provenance"]["schema_version"] == MP.PLAN_RESULT_VERSION


def _fleet_mission():
    # four distinct sites so the site-exclusive allocator can split them across two rovers
    return MP.mission_from_dict({
        "name": "FLEET", "body": "moon", "charger": [0, 0],
        "orders": [
            {"action": "pad A", "kind": "cut", "x": 40, "y": 30, "footprint_m2": 30, "depth_m": 0.04},
            {"action": "pad B", "kind": "cut", "x": -35, "y": 25, "footprint_m2": 30, "depth_m": 0.04},
            {"action": "pad C", "kind": "cut", "x": 30, "y": -40, "footprint_m2": 30, "depth_m": 0.04},
            {"action": "pad D", "kind": "cut", "x": -30, "y": -30, "footprint_m2": 30, "depth_m": 0.04},
        ],
    })


def test_plan_ir_per_vehicle_no_position_leak():
    # RB-04: each rover's FIRST GoTo must measure from the charger, not from the previous rover's last
    # position. The old single-`prev` code leaked position across vehicles in the flat trip list.
    m = _fleet_mission()
    ir = MP.plan_ir(m, vehicles=2)
    assert ir["vehicles"] == 2
    charger = tuple(m.charger)
    first_goto = {}
    for a in ir["actions"]:
        if a["op"] == "GoTo" and a["vehicle"] not in first_goto:
            first_goto[a["vehicle"]] = a
    assert len(first_goto) >= 2                                # both rovers actually have trips
    for veh, a in first_goto.items():
        expected = round(MP._d(charger, tuple(a["to"])), 2)
        assert a["expect"]["distance_m"] == pytest.approx(expected, abs=0.02)   # starts at the charger


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn(); print(f"[PASS] {fn.__name__}")
    print(f"\n{len(fns)}/{len(fns)} plan_result checks passed.")


if __name__ == "__main__":
    _run_all()
