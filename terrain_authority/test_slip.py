"""Tests for the slip-sinkage ladder (slip.py) — Phase 2.

Host-runnable (``python -m terrain_authority.test_slip``) + pytest-discoverable.
Validates the QUALITATIVE physics (quantitative magnitudes are oracle-deferred,
DEFERRED_FIXES.md): the traction ceiling, Janosi-Hanamoto monotonicity, slip
inversion + entrapment, compaction-resistance growth, and the runaway / recovery
of the fixed-point slip-sinkage solve.
"""
from __future__ import annotations

import math

from . import constants as K
from . import slip


def _rover_weight_n():
    return K.ROVER_MASS_DRY_KG * K.g


# -- traction budget ---------------------------------------------------------

def test_traction_budget_increases_with_load():
    b1 = slip.traction_budget(10.0, contact_area_m2=0.018)
    b2 = slip.traction_budget(100.0, contact_area_m2=0.018)
    assert b2 > b1 > 0.0
    assert math.isclose(b1, K.COHESION * 0.018 + 10.0 * math.tan(K.PHI), rel_tol=1e-12)


# -- Janosi-Hanamoto developed thrust ----------------------------------------

def test_developed_thrust_monotone_below_ceiling():
    hmax = 50.0
    svals = [0.05, 0.1, 0.3, 0.6, 0.9]
    hs = [slip.developed_thrust(s, hmax, contact_len_m=0.10) for s in svals]
    assert all(hs[i] < hs[i + 1] for i in range(len(hs) - 1))   # monotone increasing
    assert all(0.0 < h < hmax for h in hs)                      # strictly below ceiling
    assert slip.developed_thrust(0.0, hmax, contact_len_m=0.10) == 0.0


def test_developed_thrust_asymptotes_to_ceiling():
    hmax = 50.0
    assert slip.developed_thrust(10.0, hmax, contact_len_m=0.10) > 0.9 * hmax


# -- slip inversion + entrapment ---------------------------------------------

def test_slip_for_demand_roundtrip():
    hmax = 50.0
    s, ent = slip.slip_for_demand(20.0, hmax, contact_len_m=0.10)
    assert not ent and 0.0 < s < 0.99
    assert math.isclose(slip.developed_thrust(s, hmax, contact_len_m=0.10), 20.0, rel_tol=1e-3)


def test_slip_for_demand_entrapment_when_demand_exceeds_budget():
    hmax = 50.0
    s, ent = slip.slip_for_demand(60.0, hmax, contact_len_m=0.10)
    assert ent and s >= 0.9


def test_slip_for_demand_zero():
    s, ent = slip.slip_for_demand(0.0, 50.0, contact_len_m=0.10)
    assert s == 0.0 and not ent


# -- compaction (motion) resistance ------------------------------------------

def test_compaction_resistance_grows_with_sinkage():
    r1 = slip.compaction_resistance(0.002, contact_width_m=0.18)
    r2 = slip.compaction_resistance(0.02, contact_width_m=0.18)
    assert r2 > r1 > 0.0
    assert slip.compaction_resistance(0.0, contact_width_m=0.18) == 0.0


# -- the runaway / recovery fixed-point solve --------------------------------

def test_equilibrium_flat_converges_low_slip():
    res = slip.slip_sinkage_equilibrium(_rover_weight_n(), 0.0)
    assert not res["entrapped"]
    assert res["slip"] < 0.2
    assert res["sinkage_m"] < 0.02


def test_equilibrium_gentle_slope_stable():
    res = slip.slip_sinkage_equilibrium(_rover_weight_n(), math.radians(15.0))
    assert not res["entrapped"]


def test_equilibrium_steep_slope_entraps():
    # past the friction angle the along-slope demand exceeds the traction budget
    res = slip.slip_sinkage_equilibrium(_rover_weight_n(), math.radians(55.0))
    assert res["entrapped"]


def test_equilibrium_recovery_by_backoff():
    steep = math.radians(55.0)
    hard = slip.slip_sinkage_equilibrium(_rover_weight_n(), steep, demand_frac=1.0)
    easy = slip.slip_sinkage_equilibrium(_rover_weight_n(), steep, demand_frac=0.3)
    assert hard["entrapped"]
    assert not easy["entrapped"]


def test_equilibrium_slip_monotone_in_slope():
    s10 = slip.slip_sinkage_equilibrium(_rover_weight_n(), math.radians(10.0))["slip"]
    s30 = slip.slip_sinkage_equilibrium(_rover_weight_n(), math.radians(30.0))["slip"]
    assert s30 > s10


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn()
        print(f"[PASS] {fn.__name__}")
    print(f"\n{len(fns)}/{len(fns)} slip checks passed.")


if __name__ == "__main__":
    _run_all()
