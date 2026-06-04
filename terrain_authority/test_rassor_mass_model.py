"""Tests for rassor_mass_model -- the ICE-RASSOR drum-mass inference grounding (NTRS 20210022781).

Pins the published quality metrics, the gravity-work arm-raise energy (first-principles, g-aware), the
realistic drum-fill knowledge uncertainty, and the linear-model fit/predict/invert. Host-runnable +
pytest; pure Python (no numpy). The .fit() test uses a KNOWN MATHEMATICAL LINE (y=2x+1) to check the
least-squares estimator -- it is a numerical-method test, NOT fabricated RASSOR/regolith data.
"""
from __future__ import annotations

import math

from terrain_authority import rassor_mass_model as RM
from terrain_authority import constants as K


def test_published_metrics_verbatim():
    assert RM.AR_LINEAR_R2 == (0.996, 0.974)               # Fig 6 (front, rear)
    assert RM.FDC_LINEAR_R2 == (0.989, 0.985)              # Fig 8 (front, rear)
    assert math.isclose(RM.EDC_R2, 0.7601)                 # Fig 12
    assert math.isclose(RM.FDC_MPE_ALL, 0.07403)           # linear FDC, over range, excl. 2 outliers
    assert math.isclose(RM.FDC_MPE_WITH_OUTLIERS, 0.11842)
    assert math.isclose(RM.FDC_MPE_HALF_FULL, 0.02558)     # drum > ~half full (> 20 kg)


def test_drum_mass_uncertainty_bands_match_paper():
    assert RM.drum_mass_uncertainty_frac(25.0) == RM.FDC_MPE_HALF_FULL      # >half full -> 2.56%
    assert RM.drum_mass_uncertainty_frac(10.0) == RM.FDC_MPE_ALL            # below -> 7.40%
    assert RM.drum_mass_uncertainty_frac(10.0, include_outliers=True) == RM.FDC_MPE_WITH_OUTLIERS
    assert RM.drum_mass_uncertainty_frac(25.0) < RM.drum_mass_uncertainty_frac(10.0)  # fuller = better known


def test_arm_lift_energy_is_gravity_work_and_linear_in_mass():
    g = K.g                                                # the AR model (R^2=0.996) IS this gravity work
    e10 = RM.arm_raise_lift_energy_j(10.0, g)
    e20 = RM.arm_raise_lift_energy_j(20.0, g)
    assert math.isclose(e20, 2.0 * e10, rel_tol=1e-12)     # linear in mass
    assert math.isclose(e10, 10.0 * g * RM.ARM_LIFT_HEIGHT_M / RM.ARM_LIFT_EFFICIENCY, rel_tol=1e-12)
    assert RM.arm_raise_lift_energy_j(0.0, g) == 0.0


def test_arm_lift_energy_scales_with_gravity():
    m = 30.0
    e_moon = RM.arm_raise_lift_energy_j(m, 1.62)
    e_earth = RM.arm_raise_lift_energy_j(m, 9.81)
    assert math.isclose(e_earth / e_moon, 9.81 / 1.62, rel_tol=1e-12)       # gravity-aware (bodies.py g)


def test_arm_lift_energy_rejects_negative_mass():
    try:
        RM.arm_raise_lift_energy_j(-1.0, K.g)
        assert False, "expected ValueError on negative mass"
    except ValueError:
        pass


def test_linear_model_predict_and_invert():
    m = RM.LinearMassModel(slope=2.0, intercept=1.0)
    assert m.predict(3.0) == 7.0                           # mass = 2*feature + 1
    assert math.isclose(m.invert(7.0), 3.0, rel_tol=1e-12) # feature a given mass would produce


def test_linear_model_fit_recovers_known_line():
    # numerical-method test: a KNOWN line y = 2x + 1 (NOT RASSOR data) -> least squares recovers it
    xs = [1.0, 2.0, 3.0, 4.0, 5.0]
    ys = [2.0 * x + 1.0 for x in xs]
    fit = RM.LinearMassModel.fit(xs, ys, source="unit-test line")
    assert math.isclose(fit.slope, 2.0, abs_tol=1e-9)
    assert math.isclose(fit.intercept, 1.0, abs_tol=1e-9)
    assert math.isclose(fit.r2, 1.0, abs_tol=1e-9)


def test_linear_model_fit_rejects_bad_input():
    for bad in (lambda: RM.LinearMassModel.fit([1.0], [2.0]),                 # too few
                lambda: RM.LinearMassModel.fit([1.0, 2.0], [1.0]),            # mismatched lengths
                lambda: RM.LinearMassModel.fit([5.0, 5.0, 5.0], [1.0, 2.0, 3.0])):  # constant feature
        try:
            bad()
            assert False, "expected ValueError"
        except ValueError:
            pass


def test_freespin_current_forward_is_linear_in_mass():
    i0 = RM.freespin_drum_current_a(0.0)
    i10 = RM.freespin_drum_current_a(10.0)
    i20 = RM.freespin_drum_current_a(20.0)
    assert math.isclose(i0, RM.FDC_BASELINE_A, rel_tol=1e-12)              # empty drum -> baseline current
    assert math.isclose(i20 - i10, i10 - i0, rel_tol=1e-9)                 # constant slope (FDC linearity)
    assert i20 > i10 > i0                                                  # more mass -> more free-spin current


def test_freespin_current_gravity_scaling_is_opt_in():
    m = 20.0
    base = RM.freespin_drum_current_a(m)                                   # default: measured 1-g slope, no rescale
    moon = RM.freespin_drum_current_a(m, g=1.62)                           # opt-in gravity coupling
    rise_base = base - RM.FDC_BASELINE_A
    rise_moon = moon - RM.FDC_BASELINE_A
    assert math.isclose(rise_moon / rise_base, 1.62 / RM.EARTH_G, rel_tol=1e-9)  # rise scales with g (flagged assumption)


def test_should_offload_conservative_triggers_before_overflow():
    cap = 30.0
    # well below capacity: no offload either way
    assert RM.should_offload(15.0, cap).offload is False
    # just under capacity by point estimate, but the conservative upper bound has reached it
    d = RM.should_offload(29.5, cap)                                       # 29.5 * (1+2.56%) = 30.26 >= 30
    assert d.offload is True and d.upper_kg >= cap
    assert RM.should_offload(29.5, cap, conservative=False).offload is False  # point estimate < capacity
    # comfortably over: both fire
    assert RM.should_offload(31.0, cap).offload is True
    assert RM.should_offload(31.0, cap, conservative=False).offload is True


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn(); print(f"[PASS] {fn.__name__}")
    print(f"\n{len(fns)}/{len(fns)} rassor_mass_model checks passed.")


if __name__ == "__main__":
    _run_all()
