"""P2 (author by structure) tests, TDD-first. structures.py turns a named structure placed at a site
into mass-balanced cut/fill BuildOrder dicts (the mission_planner schema), so the UI can offer
"Landing Pad / Haul Road / Berm / ..." instead of raw cut/fill.

Invariants pinned here:
- balanced structures (pad+berm, foundation, berm-from-borrow, crater-fill) conserve VOLUME: total cut
  volume == total fill volume (density-invariant, so it holds on any body).
- source-only structures (borrow pit, road grade, solar pad, trench) are cut-only.
- every emitted order is a valid mission_planner order and plans without error.
Host-runnable + pytest; numpy-free.
"""
from __future__ import annotations

import math

from . import structures as ST


def _vol(orders, kind):
    return sum(o["footprint_m2"] * o["depth_m"] for o in orders if o["kind"] == kind)


def test_registry_has_the_taxonomy_structures():
    assert set(ST.STRUCTURES) >= {
        "landing_pad", "solar_pad", "habitat_foundation", "haul_road",
        "blast_berm", "borrow_pit", "crater_fill", "trench",
    }


def test_balanced_structures_conserve_mass_with_bulking():
    # I7: MASS is conserved, not volume. A cut removes BANK material (RHO_BANK) and the fill places it as
    # LOOSE spoil (RHO_LOOSE), which bulks to MORE volume. Pin cut_mass == fill_mass on the balanced pairs.
    for name in ("landing_pad", "habitat_foundation", "blast_berm", "crater_fill"):
        orders = ST.decompose(name, 40.0, 30.0)
        cut_v, fill_v = _vol(orders, "cut"), _vol(orders, "fill")
        assert cut_v > 0 and fill_v > 0, name
        assert math.isclose(cut_v * ST.RHO_BANK, fill_v * ST.RHO_LOOSE, rel_tol=1e-9), \
            f"{name}: cut {cut_v * ST.RHO_BANK:.1f} != fill {fill_v * ST.RHO_LOOSE:.1f} kg"


def test_loose_fill_bulks_above_bank_cut():
    # a cut->fill structure: the loose fill volume exceeds the bank cut volume by exactly SWELL (>1)
    orders = ST.decompose("landing_pad", 0.0, 0.0)
    cut_v, fill_v = _vol(orders, "cut"), _vol(orders, "fill")
    assert ST.SWELL > 1.0
    assert fill_v > cut_v and math.isclose(fill_v / cut_v, ST.SWELL, rel_tol=1e-9)


def test_source_or_sink_only_structures_are_cut_only():
    # a borrow pit / road grade / flat pad / trench is excavation only (surplus -> spoil via the planner)
    for name in ("borrow_pit", "haul_road", "solar_pad", "trench"):
        orders = ST.decompose(name, 0.0, 0.0)
        assert orders and all(o["kind"] == "cut" for o in orders), name


def test_orders_have_all_required_fields_and_positive_geometry():
    for o in ST.decompose("landing_pad", 5.0, 5.0):
        for k in ("action", "kind", "x", "y", "footprint_m2", "depth_m"):
            assert k in o, k
        assert o["kind"] in ("cut", "fill")
        assert o["footprint_m2"] > 0 and o["depth_m"] > 0


def test_paired_orders_are_not_colocated():
    # the cut source and the fill sink must be at different sites so the planner hauls between them
    orders = ST.decompose("blast_berm", 10.0, 10.0)
    cuts = [(o["x"], o["y"]) for o in orders if o["kind"] == "cut"]
    fills = [(o["x"], o["y"]) for o in orders if o["kind"] == "fill"]
    assert cuts and fills and all(c != f for c in cuts for f in fills)


def test_params_override_geometry():
    small = ST.decompose("solar_pad", 0.0, 0.0, side_m=4.0, cut_depth_m=0.02)
    big = ST.decompose("solar_pad", 0.0, 0.0, side_m=10.0, cut_depth_m=0.02)
    assert _vol(big, "cut") > _vol(small, "cut")


def test_decompose_rejects_unknown_structure():
    try:
        ST.decompose("death_star", 0.0, 0.0)
        assert False, "expected an error for an unknown structure"
    except (KeyError, ValueError):
        pass


def test_bulking_structure_balances_in_the_planner():
    # I7 (planner side): a bulking-correct structure must ALSO balance in mission_planner -- the planner
    # must excavate BANK density and place LOOSE density, else the bulked fill reads as a phantom deficit.
    from . import mission_planner as mp
    m = mp.mission_from_dict({"name": "x", "body": "moon", "charger": [0, 0],
                              "orders": ST.decompose("landing_pad", 40.0, 30.0)})
    *_, t = mp.plan_and_simulate(m)
    assert t["deficit_kg"] < 0.01 * t["fill_kg"], f"phantom deficit {t['deficit_kg']:.0f} kg"
    assert math.isclose(t["cut_kg"], t["fill_kg"], rel_tol=1e-6), f"cut {t['cut_kg']:.0f} != fill {t['fill_kg']:.0f}"


def test_orders_plan_through_mission_planner():
    # the emitted orders must be accepted by mission_planner + produce a real plan
    from . import mission_planner as mp
    orders = ST.decompose("landing_pad", 40.0, 30.0)
    m = mp.mission_from_dict({"name": "Pad", "body": "moon", "charger": [0, 0], "orders": orders})
    assert len(m.orders) == len(orders)
    *_, totals = mp.plan_and_simulate(m)
    assert totals["cut_kg"] > 0 and totals["fill_kg"] > 0


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn(); print(f"[PASS] {fn.__name__}")
    print(f"\n{len(fns)}/{len(fns)} structures checks passed.")


if __name__ == "__main__":
    _run_all()
