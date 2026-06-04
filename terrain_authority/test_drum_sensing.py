"""Drum mass-sensing observable + offload autonomy, on the REAL conserved authority.

Closes the ICE-RASSOR loop (NTRS 20210022781) in our sim: cut real regolith into the conserved drum
(`ColumnState.cut_to_inventory` -> true `drum_inventory` kg), synthesize the free-spinning drum-current
OBSERVABLE from that mass (rassor_mass_model forward model), calibrate the linear FDC inference model on
those (current, true-mass) pairs, and run the autonomy trigger (offload when the drums read full, using
the paper's measured fill-knowledge uncertainty as the safety margin). No fabricated data: the masses are
real conserved-authority values; the current is a physical sensor model; the fit is calibrated, not
hard-coded. numpy (ColumnState); host-runnable + pytest.
"""
from __future__ import annotations

import numpy as np

from terrain_authority import rassor_mass_model as RM
from terrain_authority.column_state import ColumnState


def _drum_with_mass(target_kg):
    """Cut ~target_kg of REAL conserved regolith into a drum; return (ColumnState, true drum kg)."""
    cs = ColumnState(width=20, height=20, cell_m=0.05,
                     mass_areal=np.full((20, 20), 400.0))                   # deep mantle -> removable
    mask = np.zeros((cs.height, cs.width), bool)
    mask[5:15, 5:15] = True
    area = float(mask.sum()) * cs.cell_area
    cs.cut_to_inventory(mask, target_kg / area)                            # areal kg/m^2 so total ~ target
    return cs, cs.drum_inventory


def test_fdc_calibrates_on_conserved_sim_drum_signal():
    """Fit LinearMassModel on (synthesized current, REAL conserved drum mass) -> clean linear recovery,
    and the calibrated observable infers an unseen fill accurately."""
    masses, currents = [], []
    for target in (4.0, 8.0, 12.0, 16.0, 20.0, 24.0, 28.0, 32.0):
        _, true_mass = _drum_with_mass(target)
        masses.append(true_mass)
        currents.append(RM.freespin_drum_current_a(true_mass))
    model = RM.LinearMassModel.fit(currents, masses, source="conserved-sim FDC calibration")
    assert model.r2 > 0.999                                                # forward is linear -> near-perfect fit
    # infer an unseen fill from its current
    _, m18 = _drum_with_mass(18.0)
    est = model.predict(RM.freespin_drum_current_a(m18))
    assert abs(est - m18) < 0.5                                            # calibrated inference is accurate


def test_sense_infer_offload_loop_never_overflows():
    """Dig in increments on the real conserved drum; at each step sense current -> infer mass -> decide.
    The conservative trigger (paper uncertainty margin) must fire BEFORE the true mass exceeds capacity."""
    capacity = RM.REGOLITH_PER_CYCLE_KG                                    # ~30 kg/cycle (ipex_specs)
    # calibrate once on the conserved signal
    masses, currents = [], []
    for target in (5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0):
        _, tm = _drum_with_mass(target)
        masses.append(tm); currents.append(RM.freespin_drum_current_a(tm))
    model = RM.LinearMassModel.fit(currents, masses)

    cs = ColumnState(width=20, height=20, cell_m=0.05, mass_areal=np.full((20, 20), 600.0))
    mask = np.zeros((cs.height, cs.width), bool); mask[4:16, 4:16] = True
    area = float(mask.sum()) * cs.cell_area
    fired = False
    for _ in range(40):
        cs.cut_to_inventory(mask, 2.5 / area)                             # ingest ~2.5 kg/step
        true_mass = cs.drum_inventory
        current = RM.freespin_drum_current_a(true_mass)                   # the observable
        inferred = model.predict(current)                                 # what the rover "knows"
        decision = RM.should_offload(inferred, capacity)
        if decision.offload:
            fired = True
            assert true_mass <= capacity + 0.5                            # did not overflow (conservative margin)
            assert decision.uncertainty_frac == RM.FDC_MPE_HALF_FULL       # near full -> best-known regime (2.56%)
            break
    assert fired, "offload trigger never fired while digging past capacity"


def test_drum_sensor_noise_off_is_deterministic_and_accurate():
    s = RM.DrumSensor.calibrated([5, 10, 15, 20, 25, 30])   # noise_frac=0 by default -> OFF
    a = [s.observe(m) for m in (8.0, 18.0, 28.0)]
    b = [s.observe(m) for m in (8.0, 18.0, 28.0)]
    assert a == b                                            # deterministic + repeatable, noise OFF
    assert abs(s.observe(20.0) - 20.0) < 0.3                 # faithful (noiseless inference)


def test_drum_sensor_noise_on_is_seeded_and_reproducible():
    s = RM.DrumSensor.calibrated([5, 10, 15, 20, 25, 30], noise_frac=0.05, seed=7)
    r1 = s.observe(20.0)
    assert r1 != 20.0                                        # noise perturbs the reading
    s.reset_noise()                                          # same seed -> same stream
    assert s.observe(20.0) == r1                             # reproducible


def test_drum_sensor_noise_toggle_on_demand():
    s = RM.DrumSensor.calibrated([5, 10, 15, 20, 25, 30], noise_frac=0.08, seed=1)
    noisy = s.observe(18.0)
    s.noise_frac = 0.0                                       # turn noise OFF whenever wanted
    clean = s.observe(18.0)
    assert noisy != clean and abs(clean - 18.0) < 0.3        # off -> back to faithful


def test_drum_sensor_offload_uses_its_capacity():
    s = RM.DrumSensor.calibrated([5, 10, 15, 20, 25, 30], capacity_kg=30.0)
    assert s.offload(15.0).offload is False
    assert s.offload(30.0).offload is True


def test_worksite_env_drum_sensor_optional_and_toggleable():
    from terrain_authority.worksite_env import WorkSiteConstructEnv
    # default (no sensor) == explicit None -> non-breaking, obs identical
    o1, _ = WorkSiteConstructEnv(seed=0).reset(seed=0)
    o2, _ = WorkSiteConstructEnv(seed=0, drum_sensor=None).reset(seed=0)
    assert np.array_equal(o1, o2)
    # a NOISY sensor shifts the drum-fill obs component (index 2); turning noise OFF restores fidelity
    s = RM.DrumSensor.calibrated([2, 6, 10, 14, 18, 22, 26, 30], capacity_kg=30.0, noise_frac=0.15, seed=5)
    et = WorkSiteConstructEnv(seed=0); et.reset(seed=0); et.ws.inventory_kg = 18.0
    es = WorkSiteConstructEnv(seed=0, drum_sensor=s); es.reset(seed=0); es.ws.inventory_kg = 18.0
    assert es._obs()[2] != et._obs()[2]
    s.noise_frac = 0.0
    assert abs(es._obs()[2] - et._obs()[2]) < 1e-3


def test_scheduler_env_drum_sensor_optional():
    from terrain_authority.scheduler_env import SchedulerEnv
    kw = dict(borrows=[(2, 2, 10, 10)], builds=[(40, 40, 48, 48)])
    e1 = SchedulerEnv(**kw); e1.reset(seed=0)
    e2 = SchedulerEnv(drum_sensor=None, **kw); e2.reset(seed=0)
    assert np.array_equal(e1._obs(), e2._obs())                       # default off -> non-breaking
    s = RM.DrumSensor.calibrated([10, 30, 60, 90, 120], capacity_kg=120.0, noise_frac=0.2, seed=2)
    es = SchedulerEnv(drum_sensor=s, **kw); es.reset(seed=0); es.cs.drum_inventory = 60.0
    et = SchedulerEnv(**kw); et.reset(seed=0); et.cs.drum_inventory = 60.0
    assert es._obs()[-2] != et._obs()[-2]                             # drum obs (2nd-to-last) reflects sensing


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn(); print(f"[PASS] {fn.__name__}")
    print(f"\n{len(fns)}/{len(fns)} drum-sensing checks passed.")


if __name__ == "__main__":
    _run_all()
