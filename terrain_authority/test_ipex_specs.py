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


def test_published_geometry():
    # Flight IPEx wheel: 30.5 cm dia (Zhang et al. wheel testing; r=0.1524 m used in skid-steer Eq.1).
    assert math.isclose(ix.WHEEL_DIAMETER_M, 0.305, rel_tol=1e-9)
    assert math.isclose(ix.WHEEL_RADIUS_M, ix.WHEEL_DIAMETER_M / 2.0, rel_tol=1e-9)
    # Skid-steer kinematic track from wheel-testing Eq.1 (z = 0.5207 m on the RASSOR 2 test platform).
    assert math.isclose(ix.SKID_STEER_TRACK_M, 0.5207, rel_tol=1e-9)
    assert ix.SKID_STEER is True


def test_mobility_envelope():
    # ConOps: rocks up to 7.5 cm, nominal inclination up to 15 deg (Schuler TRL-5 mobility subsystem);
    # wheel slope test ran a 20 deg incline (Zhang wheel testing). Nominal < tested-capability.
    assert math.isclose(ix.OBSTACLE_HEIGHT_M, 0.075, rel_tol=1e-9)
    assert ix.NOMINAL_SLOPE_DEG == 15.0
    assert ix.SLOPE_TEST_DEG == 20.0
    assert ix.NOMINAL_SLOPE_DEG < ix.SLOPE_TEST_DEG


def test_drum_capacity():
    # Bucket-drum scaling (Schuler 2022), avg total regolith collected per drum.
    assert math.isclose(ix.DRUM_CAPACITY_KG["small"], 3.80, rel_tol=1e-9)
    assert math.isclose(ix.DRUM_CAPACITY_KG["medium"], 7.30, rel_tol=1e-9)
    assert math.isclose(ix.DRUM_CAPACITY_KG["large"], 24.98, rel_tol=1e-9)
    # RDS min success threshold 15 kg < the up-to-30 kg/cycle headline (Schuler TRL-5 RDS).
    assert ix.REGOLITH_MIN_THRESHOLD_KG == 15.0
    assert ix.REGOLITH_MIN_THRESHOLD_KG < ix.REGOLITH_PER_CYCLE_KG
    # operational dig: drum tangential velocity = 8.5x linear cut speed; cut depth <= 50% scoop.
    assert math.isclose(ix.TANGENTIAL_TO_CUT_RATIO, 8.5, rel_tol=1e-9)
    assert math.isclose(ix.MAX_CUT_DEPTH_FRAC, 0.50, rel_tol=1e-9)


def test_bp1_test_simulant_is_terrestrial_reference():
    # BP-1 is the TERRESTRIAL GMRO test-bed simulant (Earth-g bin), NOT the lunar surface the
    # terramechanics core models -- these are sourced reference values, not wired into the lunar physics.
    from terrain_authority import constants as K
    assert math.isclose(ix.BP1_BULK_DENSITY_KG_M3, 1750.0, rel_tol=1e-9)
    assert ix.BP1_BULK_DENSITY_KG_M3 != K.RHO_SURFACE         # distinct from the loose lunar surface density
    lo, hi = ix.BP1_SHEAR_STRENGTH_KPA
    assert lo == 27.0 and hi == 32.0 and lo < hi              # shear-vane range
    plo, phi = ix.BP1_PENETRATION_KPA
    assert plo == 206.0 and phi == 226.0 and plo < phi        # penetrometer range


def test_spec_record_roundtrips():
    import json
    rec = ix.spec_record()
    json.loads(json.dumps(rec))                               # JSON-serializable
    assert rec["published"]["scale_factor_rassor2_to_ipex"] == 0.7
    # the newly-sourced geometry/mobility/drum/simulant block is in the provenance record
    g = rec["geometry"]
    assert g["wheel_diameter_m"] == 0.305 and g["skid_steer_track_m"] == 0.5207
    assert rec["mobility"]["obstacle_height_m"] == 0.075
    assert rec["drum_capacity_kg"]["medium"] == 7.30
    assert rec["bp1_test_simulant"]["bulk_density_kg_m3"] == 1750.0
    # provenance lists the ASCE bucket-drum + wheel-testing sources, not just the TRL-5 overview
    src = " ".join(rec["sources"]).lower()
    assert "wheel" in src and "bucket drum" in src


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn(); print(f"[PASS] {fn.__name__}")
    print(f"\n{len(fns)}/{len(fns)} ipex_specs checks passed.")


if __name__ == "__main__":
    _run_all()
