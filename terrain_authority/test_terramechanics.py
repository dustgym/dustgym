"""Tests for load-bearing Bekker pressure-sinkage (terramechanics.py).

Host-runnable (``python -m terrain_authority.test_terramechanics``) AND
pytest-discoverable, matching the repo's tests.py convention. Real-physics
assertions + an order-of-magnitude anchor to the committed Chrono SCM numbers
(docs/chrono_bringup_log.md). No synthetic data: the "oracle" values are the
SCM run's own soil parameters and reported sinkages.
"""
from __future__ import annotations

import math

from . import constants as K
from . import terramechanics as tm


# -- weight-on-wheels (sourced IPEx 30 kg-class mass) -------------------------

def test_static_wheel_load_lunar():
    """30 kg-class, 4 wheels, lunar g -> ~12.15 N/wheel dry, ~24.3 N laden."""
    dry = tm.static_wheel_load_n(0.0)
    laden = tm.static_wheel_load_n(K.DRUM_PAYLOAD_MAX_KG)
    assert math.isclose(dry, 30.0 * 1.62 / 4, rel_tol=1e-9), dry
    assert math.isclose(laden, 60.0 * 1.62 / 4, rel_tol=1e-9), laden
    assert laden > dry


# -- Bekker pressure-sinkage core --------------------------------------------

def test_sinkage_monotone_in_pressure():
    """More contact pressure -> more sinkage (strictly), and positive."""
    z1 = tm.bekker_pressure_sinkage(500.0, b_m=0.18)
    z2 = tm.bekker_pressure_sinkage(5000.0, b_m=0.18)
    assert z2 > z1 > 0.0, (z1, z2)


def test_sinkage_n1_linear_exact():
    """At n=1 with k_c=0, z = p / k_phi exactly, and linear in pressure."""
    z1 = tm.bekker_pressure_sinkage(1000.0, b_m=0.18, n=1.0, k_c=0.0, k_phi=2.0e5)
    z2 = tm.bekker_pressure_sinkage(2000.0, b_m=0.18, n=1.0, k_c=0.0, k_phi=2.0e5)
    assert math.isclose(z1, 1000.0 / 2.0e5, rel_tol=1e-12), z1
    assert math.isclose(z2, 2.0 * z1, rel_tol=1e-9), (z1, z2)


def test_zero_and_negative_load_no_sinkage():
    assert tm.bekker_pressure_sinkage(0.0, b_m=0.18) == 0.0
    assert tm.bekker_pressure_sinkage(-10.0, b_m=0.18) == 0.0
    assert tm.wheel_static_sinkage(0.0) == 0.0


# -- regime anchor: light rover -> sub-cm static bearing (spec §6) ------------

def test_lunar_static_bearing_subcm_both_param_sets():
    """The 30 kg-class per-wheel load gives SUB-CM static bearing sinkage under
    BOTH the constants.py moduli and the SCM oracle set — spec §6: "static
    bearing self-limits fast/shallow in 1/6 g (sub-cm to a few cm), benign."
    The SCM set (softer k_phi) predicts MORE sinkage than the spec moduli.
    """
    load = tm.static_wheel_load_n(0.0)  # ~12.15 N
    z_spec = tm.wheel_static_sinkage(load, contact_len_m=0.10, contact_width_m=0.18)
    z_scm = tm.wheel_static_sinkage(load, contact_len_m=0.10, contact_width_m=0.18,
                                    k_c=0.0, k_phi=0.2e6, n=1.0)
    assert 0.0 < z_spec < 0.01, z_spec
    assert 0.0 < z_scm < 0.02, z_scm
    assert z_scm > z_spec, (z_spec, z_scm)  # 4x softer k_phi -> deeper


def test_oracle_param_band_committed_numbers():
    """Order-of-magnitude anchor to committed SCM numbers (chrono_bringup_log.md:
    node sinkage ~8.7 mm, cylinder sink ~10 cm under the 25 kg moving cylinder).
    With the SCM soil set, Bekker over a plausible contact-pressure range lands in
    the committed band (1 mm .. 15 cm). Precise fit needs the euclid load-sweep
    (Phase 0.3); this is a sanity anchor, NOT a false-precision claim.
    """
    for pressure in (2000.0, 7000.0, 20000.0):
        z = tm.bekker_pressure_sinkage(pressure, b_m=0.12, k_c=0.0, k_phi=0.2e6, n=1.0)
        assert 1e-3 <= z <= 0.15, (pressure, z)


# -- mass-conserving sinkage -> density mapping ------------------------------

def test_sinkage_to_density_factor_mass_conserving():
    """density *= (1 + f) thins the column by exactly z, conserving areal mass."""
    rho = K.RHO_SURFACE
    t = 0.12
    mass_areal = rho * t
    z = 0.004
    f = tm.sinkage_to_density_factor(z, t)
    rho2 = rho * (1.0 + f)
    t2 = mass_areal / rho2  # thickness at conserved mass
    assert math.isclose(t - t2, z, rel_tol=1e-9), (t - t2, z)


def test_sinkage_factor_clamped_below_thickness():
    """A sinkage >= column thickness is clamped (cannot compact past zero)."""
    f = tm.sinkage_to_density_factor(0.20, 0.12)  # z > t
    rho2 = K.RHO_SURFACE * (1.0 + f)
    t2 = (K.RHO_SURFACE * 0.12) / rho2
    assert t2 > 0.0  # still a positive-thickness column


# -- multi-pass paving emerges from density stiffening -----------------------

def test_multipass_paving_diminishing_and_conserved():
    """Repeated passes at fixed load sink LESS each time (denser soil bears
    better) and conserve mass. Paving is EMERGENT from density stiffening, not a
    hardcoded constant.
    """
    load = tm.static_wheel_load_n(K.DRUM_PAYLOAD_MAX_KG)  # laden, clearer signal
    rho = K.RHO_SURFACE
    t = 0.12
    mass = rho * t
    sinks = []
    for _ in range(8):
        z = tm.wheel_static_sinkage(load, contact_len_m=0.10, contact_width_m=0.18,
                                    density=rho)
        f = tm.sinkage_to_density_factor(z, t)
        rho_new = min(rho * (1.0 + f), K.RHO_DEEP)
        t_new = mass / rho_new
        sinks.append(t - t_new)          # actual surface drop this pass
        assert math.isclose(rho_new * t_new, mass, rel_tol=1e-12)  # mass conserved
        assert rho_new >= rho             # compacting (monotone density)
        rho, t = rho_new, t_new
    # non-increasing sinkage, with a strict overall decrease (the paving effect)
    for i in range(1, len(sinks)):
        assert sinks[i] <= sinks[i - 1] + 1e-15, (i, sinks)
    assert sinks[-1] < sinks[0], sinks


# -- JSON config layer (TerramechanicsParams) --------------------------------

def test_params_default_from_constants():
    p = tm.TerramechanicsParams.from_constants()
    assert p.k_phi == K.K_PHI and p.k_c == K.K_C and p.n_sinkage == K.N_SINKAGE
    assert p.rover_mass_dry_kg == K.ROVER_MASS_DRY_KG


def test_params_json_roundtrip():
    """Override a field (domain-randomization style) and round-trip via JSON."""
    import json
    import os
    import tempfile
    base = tm.TerramechanicsParams.from_constants()
    p = tm.TerramechanicsParams.from_dict({**base.to_dict(), "k_phi": 2.0e5, "n_sinkage": 0.9})
    back = tm.TerramechanicsParams.from_dict(json.loads(p.to_json()))
    assert back == p
    assert back.k_phi == 2.0e5 and back.n_sinkage == 0.9
    fd, path = tempfile.mkstemp(suffix=".json")
    os.close(fd)
    try:
        p.to_json(path)
        assert tm.TerramechanicsParams.from_json(path) == p
    finally:
        os.remove(path)


def test_params_rejects_unknown_keys():
    try:
        tm.TerramechanicsParams.from_dict({"k_phi": 1.0, "bogus": 2.0})
    except ValueError as e:
        assert "bogus" in str(e)
    else:
        raise AssertionError("expected ValueError on unknown param key")


def test_scm_oracle_params_match_kwarg_path():
    """The .scm_oracle() params object reproduces the explicit-kwarg oracle call."""
    o = tm.TerramechanicsParams.scm_oracle()
    assert o.k_phi == 0.2e6 and o.k_c == 0.0 and o.n_sinkage == 1.0
    load = tm.static_wheel_load_n(0.0)
    z_obj = tm.wheel_static_sinkage(load, params=o, contact_len_m=0.10, contact_width_m=0.18)
    z_kw = tm.wheel_static_sinkage(load, contact_len_m=0.10, contact_width_m=0.18,
                                   k_c=0.0, k_phi=0.2e6, n=1.0)
    assert math.isclose(z_obj, z_kw, rel_tol=1e-12), (z_obj, z_kw)


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn()
        print(f"[PASS] {fn.__name__}")
    print(f"\n{len(fns)}/{len(fns)} terramechanics checks passed.")


if __name__ == "__main__":
    _run_all()
