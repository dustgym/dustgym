"""Tests for ipex_specs.py — verify the energy model's derivations against the published
IPEx numbers (Schuler ASCEND 2024) + the stated 12S/30Ah pack. These check arithmetic on
REAL inputs, not synthetic data. Host-runnable + pytest.
"""
from __future__ import annotations

import math

from terrain_authority import ipex_specs as ix


def test_battery_energy():
    # 12S * 3.7 V * 30 Ah = 1332 Wh (~44 V pack) = 4.7952 MJ
    assert math.isclose(ix.battery_energy_wh(), 1332.0, rel_tol=1e-9)
    assert math.isclose(ix.battery_energy_j(), 1332.0 * 3600.0, rel_tol=1e-9)
    assert 43.0 <= ix.BATTERY_SERIES_CELLS * ix.LIION_NOMINAL_V_PER_CELL <= 45.0   # ~44 V


def test_drive_power_and_per_m():
    # 4 wheels * 0.063 N*m * (1530 RPM -> rad/s)
    omega = 1530.0 * 2 * math.pi / 60.0
    assert math.isclose(ix.drive_power_w(), 4 * 0.063 * omega, rel_tol=1e-9)
    assert 30.0 < ix.drive_power_w() < 60.0                 # ~40 W, sane for a 30 kg rover
    assert math.isclose(ix.drive_energy_per_m(), ix.drive_power_w() / 0.30, rel_tol=1e-9)


def test_dig_energy_per_kg():
    # 18.5 N*m at 25 RPM -> W; / (42 kg/hr -> kg/s)
    omega = 25.0 * 2 * math.pi / 60.0
    assert math.isclose(ix.dig_power_w(), 18.5 * omega, rel_tol=1e-9)
    assert math.isclose(ix.dig_energy_per_kg(),
                        ix.dig_power_w() / (42.0 / 3600.0), rel_tol=1e-9)
    assert ix.dig_energy_per_kg() > ix.drive_energy_per_m()   # digging a kg costs > driving a metre


def test_energy_model_units():
    m = ix.energy_model(cell_m=0.02)
    # per-cell travel cost = J/m * cell_m
    assert math.isclose(m["travel_cost_per_cell"], ix.drive_energy_per_m() * 0.02, rel_tol=1e-9)
    assert m["energy_budget"] == ix.battery_energy_j()        # default = full pack
    # a task allowance overrides the budget
    m2 = ix.energy_model(cell_m=0.02, allowance_factor=1.3, planner_cost_j=1000.0)
    assert math.isclose(m2["energy_budget"], 1300.0, rel_tol=1e-9)


def test_spec_record_roundtrips():
    import json
    rec = ix.spec_record()
    json.loads(json.dumps(rec))                               # JSON-serializable
    assert rec["published"]["scale_factor_rassor2_to_ipex"] == 0.7


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn(); print(f"[PASS] {fn.__name__}")
    print(f"\n{len(fns)}/{len(fns)} ipex_specs checks passed.")


if __name__ == "__main__":
    _run_all()
