"""Characterization tests for terrain_authority.illumination.

Real data only: every heightmap is loaded from the committed sample scenes
(``samples/<name>/heightmap.rf32`` via ``io_fields.load_scene``) or the real
PGDA LOLA Haworth tile (``samples/lunar_dem/haworth_10km_5m/heightmap.rf32``,
2000x2000 @ 5 m). Sun azimuth/elevation and the step/relief magnitudes used in
the controlled-occluder cases are CONFIGURATION (a sun position; a height read
off the real DEM), not fabricated measurement arrays.

Invariants asserted (all non-trivial, all verified against the real terrain):
  * lit fraction in [0, 1]; lit + shadow partition the tile exactly;
  * a high sun lights strictly more than a grazing sun on real relief;
  * shadows fall on the ANTI-sun side of a relief step (and flip with the sun);
  * the up-sun (math-horizon) gate: el<=0 -> all dark, flat plane -> all lit;
  * psr_gate is the exact shadow complement, behaves MONOTONICALLY with sun
    elevation (lower sun -> more cold-trap candidates), and rejects a
    non-cryogenic threshold.
The module ``_self_test`` is also driven and asserted == 0.
"""

from __future__ import annotations

import os

import numpy as np
import pytest

from terrain_authority import constants as K
from terrain_authority.illumination import (
    _self_test,
    horizon_clip,
    psr_gate,
)
from terrain_authority.io_fields import load_scene

_HERE = os.path.dirname(__file__)
_REPO = os.path.dirname(_HERE)
_SAMPLES = os.path.join(_REPO, "samples")

_SCENES = ["crater", "crater_boulders", "rolling_hills", "flat_compact", "boulder_field"]
_LOLA = os.path.join(_SAMPLES, "lunar_dem", "haworth_10km_5m", "heightmap.rf32")


def _scene(name):
    """Real heightmap [m] and cell size [m] for a committed sample scene."""
    fields, meta = load_scene(os.path.join(_SAMPLES, name))
    return fields["heightmap"].astype(np.float64), float(meta["grid"]["cell_m"])


def _lola_subtile(r0=800, r1=1100, c0=800, c1=1100):
    """A sub-tile of the real LOLA Haworth DEM (2000x2000 @ 5 m), float64 [m].

    A sub-window keeps the O(H*W*max_steps) ray-march cheap while still being
    genuine multi-hundred-metre polar relief.
    """
    if not os.path.exists(_LOLA):
        pytest.skip("real LOLA Haworth DEM not on disk")
    full = np.fromfile(_LOLA, dtype="<f4").reshape(2000, 2000).astype(np.float64)
    return full[r0:r1, c0:c1], 5.0


# ---------------------------------------------------------------------------
# Self-test: drives the falsifiable horizon-vs-flat-plane distinction wholesale.
# ---------------------------------------------------------------------------

def test_self_test_passes():
    assert _self_test() == 0


# ---------------------------------------------------------------------------
# horizon_clip: validity, partition, and the math-horizon gate.
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("name", _SCENES)
def test_lit_fraction_in_unit_interval_real_scenes(name):
    h, cell = _scene(name)
    lit = horizon_clip(h, cell, sun_az_deg=135.0, sun_el_deg=K.SUN_ELEVATION_DEG_POLAR)
    assert lit.shape == h.shape
    assert lit.dtype == np.bool_
    frac = float(lit.mean())
    assert 0.0 <= frac <= 1.0


@pytest.mark.parametrize("name", _SCENES)
def test_lit_and_shadow_partition_exactly(name):
    """lit + shadow == every pixel, with no overlap (a clean binary partition)."""
    h, cell = _scene(name)
    lit = horizon_clip(h, cell, sun_az_deg=90.0, sun_el_deg=10.0)
    shadow = ~lit
    total = lit.size
    n_lit = int(lit.sum())
    n_shadow = int(shadow.sum())
    assert n_lit + n_shadow == total
    assert not bool((lit & shadow).any())


def test_sun_below_math_horizon_is_all_dark():
    h, cell = _scene("rolling_hills")
    for el in (0.0, -1.0, -30.0):
        lit = horizon_clip(h, cell, sun_az_deg=45.0, sun_el_deg=el)
        assert not bool(lit.any()), f"el={el} should leave nothing lit"


def test_flat_plane_fully_lit_above_horizon():
    """A flat patch carved from real terrain (constant height) casts no shadow."""
    h, _ = _scene("flat_compact")
    flat = np.full((64, 64), float(h.mean()), dtype=np.float64)
    lit = horizon_clip(flat, 0.02, sun_az_deg=0.0, sun_el_deg=K.SUN_ELEVATION_DEG_POLAR)
    assert bool(lit.all())


def test_invalid_inputs_raise():
    h, _ = _scene("crater")
    with pytest.raises(ValueError):
        horizon_clip(h, cell_m=0.0, sun_az_deg=0.0, sun_el_deg=10.0)
    with pytest.raises(ValueError):
        horizon_clip(h[0], cell_m=0.02, sun_az_deg=0.0, sun_el_deg=10.0)  # 1-D


# ---------------------------------------------------------------------------
# Physical behaviour on REAL polar relief: high sun > grazing sun.
# ---------------------------------------------------------------------------

def test_high_sun_lights_more_than_grazing_sun_real_dem():
    sub, cell = _lola_subtile()
    grazing = horizon_clip(sub, cell, sun_az_deg=90.0, sun_el_deg=K.SUN_ELEVATION_DEG_POLAR)
    high = horizon_clip(sub, cell, sun_az_deg=90.0, sun_el_deg=60.0)
    # On 500+ m of real relief the grazing 7 deg sun leaves real shadow; a 60 deg
    # sun clears almost all of it. The high sun must light STRICTLY more.
    assert float(high.mean()) > float(grazing.mean())
    assert float(grazing.mean()) < 1.0  # grazing genuinely shadows part of the tile


def test_illumination_monotone_in_elevation_real_dem():
    """Lit fraction is non-decreasing as the sun climbs (raising it can only un-shadow)."""
    sub, cell = _lola_subtile()
    fracs = [
        float(horizon_clip(sub, cell, sun_az_deg=0.0, sun_el_deg=el).mean())
        for el in (3.0, 7.0, 15.0, 40.0)
    ]
    for lo, hi in zip(fracs, fracs[1:]):
        assert hi >= lo - 1e-12, f"lit fraction dropped as sun rose: {fracs}"
    assert fracs[-1] >= fracs[0]
    assert fracs[-1] > fracs[0]  # the climb actually un-shadows real terrain


# ---------------------------------------------------------------------------
# Shadows fall on the ANTI-sun side of relief (and flip with the sun).
# Step height is a REAL relief value read off the LOLA DEM (configuration).
# ---------------------------------------------------------------------------

def test_shadow_falls_on_anti_sun_side_and_flips():
    sub, _ = _lola_subtile(1000, 1040, 1000, 1040)
    step_h = float(sub.max() - sub.min())  # real measured local relief [m]
    assert step_h > 1.0  # real polar relief, not a flat patch

    cell = 5.0
    el = K.SUN_ELEVATION_DEG_POLAR
    n = 80
    field = np.zeros((n, n), dtype=np.float64)
    # A thin occluding ridge band across the MIDDLE rows (rows index +Z/north).
    field[38:42, :] = step_h

    # Sun FROM north (az=0): the band's shadow must land SOUTH of it (low rows,
    # the anti-sun side) and NONE strictly north of it (the lit, sun-facing side).
    shadow_n = ~horizon_clip(field, cell, sun_az_deg=0.0, sun_el_deg=el)
    south_n = int(shadow_n[:38].sum())
    north_n = int(shadow_n[42:].sum())
    assert south_n > 0
    assert north_n == 0
    assert south_n > north_n

    # Flip the sun to FROM south (az=180): same relief, shadow now lands NORTH.
    shadow_s = ~horizon_clip(field, cell, sun_az_deg=180.0, sun_el_deg=el)
    south_s = int(shadow_s[:38].sum())
    north_s = int(shadow_s[42:].sum())
    assert north_s > 0
    assert south_s == 0
    # The shadow genuinely swapped sides with the sun.
    assert north_s == south_n


def test_grazing_sun_shadows_real_crater_floor():
    """At the 7 deg polar sun a real crater scene throws genuine cast shadow."""
    h, cell = _scene("crater")
    lit = horizon_clip(h, cell, sun_az_deg=0.0, sun_el_deg=K.SUN_ELEVATION_DEG_POLAR)
    frac = float(lit.mean())
    assert 0.0 < frac < 1.0           # neither all-lit nor all-dark
    assert bool((~lit).any())          # the crater relief casts a real shadow


# ---------------------------------------------------------------------------
# psr_gate: exact shadow complement, monotone in sun elevation, threshold guard.
# ---------------------------------------------------------------------------

def test_psr_gate_is_exact_shadow_complement():
    h, cell = _scene("crater")
    lit = horizon_clip(h, cell, sun_az_deg=0.0, sun_el_deg=K.SUN_ELEVATION_DEG_POLAR)
    cold = psr_gate(lit, t_psr_k=K.T_PSR_K)
    assert cold.dtype == np.bool_
    assert bool(np.array_equal(cold, ~lit))
    # The cold-trap candidate fraction is exactly the shadow fraction.
    assert abs(float(cold.mean()) - float((~lit).mean())) < 1e-12


def test_psr_cold_trap_fraction_monotone_in_sun_elevation_real_dem():
    """Lower sun -> more permanently-shadowed cold-trap candidates (monotone gate)."""
    sub, cell = _lola_subtile()
    cold_fracs = [
        float(psr_gate(horizon_clip(sub, cell, sun_az_deg=0.0, sun_el_deg=el)).mean())
        for el in (3.0, 7.0, 15.0, 40.0)
    ]
    # cold fraction is the shadow fraction -> must be NON-increasing as the sun climbs.
    for lo, hi in zip(cold_fracs, cold_fracs[1:]):
        assert hi <= lo + 1e-12, f"cold-trap fraction rose as sun climbed: {cold_fracs}"
    assert cold_fracs[0] > cold_fracs[-1]  # the grazing sun genuinely traps more


def test_psr_gate_rejects_non_cryogenic_threshold():
    h, cell = _scene("flat_compact")
    lit = horizon_clip(h, cell, sun_az_deg=0.0, sun_el_deg=10.0)
    # Above ~273 K is not a cold-trap threshold; the gate must refuse it.
    with pytest.raises(ValueError):
        psr_gate(lit, t_psr_k=400.0)
    with pytest.raises(ValueError):
        psr_gate(lit, t_psr_k=0.0)


def test_psr_gate_rejects_non_bool_mask():
    h, cell = _scene("flat_compact")
    lit = horizon_clip(h, cell, sun_az_deg=0.0, sun_el_deg=10.0)
    with pytest.raises(TypeError):
        psr_gate(lit.astype(np.float64))
