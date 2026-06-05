"""Tests for load-bearing Bekker pressure-sinkage (terramechanics.py).

Host-runnable (``python -m terrain_authority.test_terramechanics``) AND
pytest-discoverable, matching the repo's tests.py convention. Real-physics
assertions + an order-of-magnitude anchor to the committed Chrono SCM numbers
(docs/chrono_bringup_log.md). No synthetic data: the "oracle" values are the
SCM run's own soil parameters and reported sinkages.
"""
from __future__ import annotations

import math

import numpy as np

from . import constants as K
from . import rover
from . import terramechanics as tm
from .column_state import ColumnState


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


# -- wiring into rover.four_wheel_pass (opt-in physical path) ------

def test_four_wheel_pass_default_path_unchanged():
    """physical=False (default) is byte-identical to the prior constant path."""
    poses = [((24.0, 14.0), 0.3), ((22.0, 30.0), 0.5)]
    a = ColumnState(width=48, height=48, cell_m=0.02)
    b = ColumnState(width=48, height=48, cell_m=0.02)
    rover.four_wheel_pass(a, poses)                                   # default
    rover.four_wheel_pass(b, poses, physical=False, compaction=0.12)  # explicit
    assert np.array_equal(a.density, b.density)
    assert np.array_equal(a.state_label, b.state_label)
    assert np.array_equal(a.disturbance, b.disturbance)


def test_four_wheel_pass_physical_mass_conserved():
    """The load-driven path conserves total mass (density-only edit) and compacts."""
    cs = ColumnState(width=64, height=64, cell_m=0.02)
    m0 = cs.total_mass()
    poses = [((32.0, 18.0), 0.0), ((32.0, 44.0), 0.0)]
    rover.four_wheel_pass(cs, poses, physical=True)
    assert math.isclose(cs.total_mass(), m0, rel_tol=1e-9), (cs.total_mass(), m0)
    assert cs.density.max() > K.RHO_SURFACE          # compaction happened
    assert cs.density.max() <= K.RHO_DEEP            # capped at the ceiling


def test_four_wheel_pass_physical_load_dependent():
    """Heavier per-wheel load -> deeper compaction (load-bearing, not a constant)."""
    poses = [((32.0, 32.0), 0.0)]
    light = ColumnState(width=64, height=64, cell_m=0.02)
    heavy = ColumnState(width=64, height=64, cell_m=0.02)
    rover.four_wheel_pass(light, poses, physical=True, loads=tm.static_wheel_load_n(0.0))
    rover.four_wheel_pass(heavy, poses, physical=True,
                          loads=tm.static_wheel_load_n(K.DRUM_PAYLOAD_MAX_KG))
    assert heavy.density.max() > light.density.max(), (light.density.max(), heavy.density.max())


def test_conform_pose_emits_normal_loads():
    """conform_pose returns per-wheel normal load; flat -> full weight, tilt -> less."""
    flat = np.zeros((32, 32))
    res = rover.conform_pose(flat, (16.0, 16.0), 0.0, cell_m=0.02)
    loads = res["normal_loads"]
    assert set(loads) == {"LF", "RF", "LB", "RB"}
    total = res["normal_load_total_n"]
    assert math.isclose(total, K.ROVER_MASS_DRY_KG * K.g, rel_tol=1e-6), total
    assert math.isclose(sum(loads.values()), total, rel_tol=1e-9)
    # a tilted surface reduces the normal component (cos(tilt) < 1) -> slip driver
    ramp = np.fromfunction(lambda r, c: 0.30 * c * 0.02, (32, 32))  # 30% grade in +col
    res_t = rover.conform_pose(ramp, (16.0, 16.0), 0.0, cell_m=0.02)
    assert res_t["normal_load_total_n"] < total


def test_conform_pose_payload_scales_load():
    """A drum payload increases weight-on-wheels."""
    flat = np.zeros((32, 32))
    dry = rover.conform_pose(flat, (16.0, 16.0), 0.0, cell_m=0.02)["normal_load_total_n"]
    laden = rover.conform_pose(flat, (16.0, 16.0), 0.0, cell_m=0.02,
                               payload_kg=K.DRUM_PAYLOAD_MAX_KG)["normal_load_total_n"]
    assert laden > dry
    assert math.isclose(laden, (K.ROVER_MASS_DRY_KG + K.DRUM_PAYLOAD_MAX_KG) * K.g, rel_tol=1e-6)


# -- Lyasko 1g->1/6g reduced-gravity correction -------------------

def test_lyasko_increases_sinkage():
    """The sourced NET truth: reduced gravity -> MORE sinkage under the same load."""
    earth = tm.TerramechanicsParams.from_constants()
    lunar = tm.TerramechanicsParams.lunar()
    load = tm.static_wheel_load_n(0.0)
    z_earth = tm.wheel_static_sinkage(load, params=earth)
    z_lunar = tm.wheel_static_sinkage(load, params=lunar)
    assert z_lunar > z_earth, (z_earth, z_lunar)


def test_lyasko_directions():
    """k_phi and cohesion decrease; k_c and phi unchanged (Lyasko: little change).
    n is left unchanged by default (the dimensional-units caveat in lyasko_reduce).
    """
    earth = tm.TerramechanicsParams.from_constants()
    lunar = tm.TerramechanicsParams.lunar()
    assert lunar.k_phi < earth.k_phi
    assert lunar.cohesion < earth.cohesion
    assert lunar.k_c == earth.k_c
    assert lunar.phi_rad == earth.phi_rad
    assert lunar.n_sinkage == earth.n_sinkage  # n_frac default 0 (deferred to oracle fit)


def test_lyasko_identity_at_earth_gravity():
    """At g = g_earth the correction is the identity (deficit 0)."""
    earth = tm.TerramechanicsParams.from_constants()
    same = tm.lyasko_reduce(earth, g=9.81, g_earth=9.81)
    assert same == earth


def test_lunar_params_json_roundtrip():
    """Lunar params are still a valid JSON-serializable config."""
    import json
    lunar = tm.TerramechanicsParams.lunar()
    back = tm.TerramechanicsParams.from_dict(json.loads(lunar.to_json()))
    assert back == lunar


# -- slip-sinkage multiplier + four_wheel_pass slip -----------

def test_slip_sinkage_multiplier_monotone():
    assert tm.slip_sinkage_multiplier(0.0) == 1.0
    assert tm.slip_sinkage_multiplier(0.5) > 1.0
    assert tm.slip_sinkage_multiplier(0.8) > tm.slip_sinkage_multiplier(0.5)


def test_physical_field_no_nan_on_bare_cells():
    """Bare / near-zero-thickness cells (EXCAVATED to firm layer on real scenes) give
    f=0, not NaN/inf. Regression for the divide-by-(thickness-z) found live."""
    density = np.array([K.RHO_SURFACE, K.RHO_DEEP, K.RHO_SURFACE], dtype=np.float64)
    mass_areal = np.array([0.0, 1e-6, K.RHO_SURFACE * 0.12], dtype=np.float64)  # bare, near-zero, normal
    f = tm.physical_compaction_field(density, mass_areal, tm.static_wheel_load_n(0.0))
    assert np.all(np.isfinite(f))
    assert f[0] == 0.0 and f[1] == 0.0      # cannot compact a (near-)zero-thickness column
    assert f[2] > 0.0                        # normal cell still compacts


def test_four_wheel_pass_slip_deepens_rut_mass_conserved():
    """A slipping wheel digs a deeper rut (more compaction) but conserves mass."""
    poses = [((32.0, 32.0), 0.0)]
    no_slip = ColumnState(width=64, height=64, cell_m=0.02)
    with_slip = ColumnState(width=64, height=64, cell_m=0.02)
    m0 = no_slip.total_mass()
    rover.four_wheel_pass(no_slip, poses, physical=True, slip=0.0)
    rover.four_wheel_pass(with_slip, poses, physical=True, slip=0.6)
    assert math.isclose(with_slip.total_mass(), m0, rel_tol=1e-9)
    assert with_slip.density.max() > no_slip.density.max()


# -- domain randomization within sourced envelopes -------------

def test_domain_randomize_within_envelopes():
    p = tm.domain_randomize(np.random.default_rng(0))
    assert 0.8 <= p.n_sinkage <= 1.0
    assert 0.2e6 <= p.k_phi <= 0.82e6
    assert 100.0 <= p.cohesion <= 1000.0
    assert 0.3 <= p.slip_c1 <= 0.5
    assert 0.2 <= p.slip_c2 <= 0.4
    assert p.k_c == tm.TerramechanicsParams.from_constants().k_c  # k_c held fixed


def test_domain_randomize_deterministic_seed():
    a = tm.domain_randomize(np.random.default_rng(42))
    b = tm.domain_randomize(np.random.default_rng(42))
    assert a == b


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn()
        print(f"[PASS] {fn.__name__}")
    print(f"\n{len(fns)}/{len(fns)} terramechanics checks passed.")


if __name__ == "__main__":
    _run_all()
