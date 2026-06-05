"""Tests for the per-planet Body registry (bodies.py), grounded in docs/bodies_sysrev.md.
Numpy-only (no gymnasium) -> runs in the core suite.

Pins the SOURCED constants and their honest regime flags: real gravities, the NASA-lunar Bekker moduli,
the Mars GRC-3 simulant moduli, microgravity flags for Bennu/Phobos, cohesion ordering across 6 orders
of magnitude, and that wheel load scales with body gravity.
"""
from __future__ import annotations

import math

import pytest

from terrain_authority import bodies as B
from terrain_authority import terramechanics as tm

TARGETS = ["moon", "mars", "ceres", "bennu", "phobos"]   # the 5 habitat/mining targets


def test_registry_has_targets_plus_earth():
    assert set(TARGETS).issubset(B.BODIES)
    assert "earth" in B.BODIES                            # validation body
    # science-only bodies are intentionally excluded
    assert "europa" not in B.BODIES and "titan" not in B.BODIES


def test_rover_bodies_are_derived_from_the_registry_not_hardcoded():
    # MAJOR (architecture review): the per-body DRIVE IDs must be DERIVED from BODIES by bekker_regime,
    # so adding one gravity-loaded Body auto-creates its Dust/RoverDrive-<Body>-v0 ID. A hardcoded list
    # silently drops new bodies, and microgravity bodies (Bekker out of regime) must be excluded.
    from terrain_authority.registration import ROVER_BODIES   # importable without gymnasium
    expected = [k for k, b in B.BODIES.items() if b.bekker_regime == "gravity-loaded"]
    assert ROVER_BODIES == expected
    assert "bennu" not in ROVER_BODIES and "phobos" not in ROVER_BODIES   # microgravity excluded
    assert {"moon", "mars", "ceres", "earth"}.issubset(set(ROVER_BODIES))


def test_known_gravities():
    assert B.BODIES["moon"].g == 1.62
    assert B.BODIES["mars"].g == 3.71
    assert B.BODIES["earth"].g == 9.81
    assert B.BODIES["bennu"].g < 1e-3 and B.BODIES["phobos"].g < 1e-2   # micro/milli-g


def test_moon_uses_sourced_nasa_bekker():
    # NASA LTV white paper lunar values (NTRS 20220010732), applied directly (no double Lyasko)
    p = B.params_for_body("moon")
    assert (p.k_c, p.k_phi, p.n_sinkage) == (1400.0, 820000.0, 1.0)
    assert p.cohesion == 170.0
    assert p.phi_rad == pytest.approx(math.radians(35.0))


def test_mars_uses_grc3_simulant_bekker():
    p = B.params_for_body("mars")
    assert (p.k_c, p.k_phi, p.n_sinkage) == (23200.0, 606700.0, 1.0)   # GRC-3, Oravec 2020
    assert p.cohesion == 1000.0


def test_microgravity_regime_flags():
    assert B.BODIES["bennu"].bekker_regime == "microgravity"
    assert B.BODIES["phobos"].bekker_regime == "microgravity"
    for n in ["moon", "mars", "ceres", "earth"]:
        assert B.BODIES[n].bekker_regime == "gravity-loaded"


def test_unknown_bekker_falls_back_to_lunar_analog():
    # Ceres/Bennu/Phobos have no sourced Bekker -> repo lunar baseline stands in (flagged in provenance)
    base = tm.TerramechanicsParams.from_constants()
    for n in ["ceres", "bennu", "phobos"]:
        assert B.BODIES[n].bekker is None
        assert B.params_for_body(n).k_phi == base.k_phi   # analog, not a fabricated body value


def test_cohesion_spans_orders_of_magnitude():
    # Bennu (~2 Pa) << Moon (170) << Mars (1000); the sysrev's headline spread
    assert B.BODIES["bennu"].cohesion_pa < B.BODIES["moon"].cohesion_pa < B.BODIES["mars"].cohesion_pa


def test_wheel_load_scales_with_gravity():
    moon = tm.static_wheel_load_n(g=B.BODIES["moon"].g)
    mars = tm.static_wheel_load_n(g=B.BODIES["mars"].g)
    assert mars == pytest.approx(moon * (B.BODIES["mars"].g / B.BODIES["moon"].g), rel=1e-9)


def test_provenance_present():
    # every body must carry a citation + confidence tag (no silent placeholders)
    for n, b in B.BODIES.items():
        assert b.provenance and b.confidence and b.role, n


def test_get_body_case_insensitive_and_errors():
    assert B.get_body("MARS").name == "mars"
    assert B.get_body(B.BODIES["moon"]).name == "moon"
    with pytest.raises(KeyError):
        B.get_body("pluto")
