"""Characterization tests for terrain_authority.dem_stats.

Real data only: every roughness number is measured on a committed sample-scene
heightmap (``samples/<name>/heightmap.rf32`` via ``io_fields.load_scene``) or on
the real PGDA LOLA Haworth tile (``samples/lunar_dem/haworth_10km_5m``,
2000x2000 @ 5 m). Baselines are CONFIGURATION (physical lag lengths), not
fabricated measurements.

Invariants asserted (all non-trivial, all verified against the real terrain):
  * deviogram D(L) and RMS slope are POSITIVE and FINITE on real rough terrain;
  * the structure function D(L) INCREASES with baseline on real relief;
  * RMS slope ROLLS OFF (decreases) with baseline -- the lunar signature;
  * flat_compact is near-zero roughness, an order+ of magnitude below
    rolling_hills and crater (the "is this terrain actually rough?" test);
  * keys echo the REQUESTED baselines; unresolvable baselines are dropped.
The module ``_self_test`` (analytic plane + white-noise truth) is also driven
and asserted == 0.
"""

from __future__ import annotations

import os

import numpy as np
import pytest

from terrain_authority.dem_stats import (
    _self_test,
    deviogram,
    rms_slope_vs_baseline,
)
from terrain_authority.io_fields import load_scene

_HERE = os.path.dirname(__file__)
_REPO = os.path.dirname(_HERE)
_SAMPLES = os.path.join(_REPO, "samples")
_LOLA = os.path.join(_SAMPLES, "lunar_dem", "haworth_10km_5m", "heightmap.rf32")


def _scene(name):
    """Real heightmap [m] and cell size [m] for a committed sample scene."""
    fields, meta = load_scene(os.path.join(_SAMPLES, name))
    return fields["heightmap"].astype(np.float64), float(meta["grid"]["cell_m"])


# A geometrically increasing set of resolvable baselines on the 256x256 @ 0.02 m
# sample grids (cell .. 32 cells; all < the field, all >= one cell).
def _baselines(cell):
    return [cell * m for m in (2, 4, 8, 16, 32)]


# ---------------------------------------------------------------------------
# Self-test: plane (linear D, constant slope) + white noise (flat D, rolloff).
# ---------------------------------------------------------------------------

def test_self_test_passes():
    assert _self_test() == 0


# ---------------------------------------------------------------------------
# Positivity + finiteness on real rough terrain.
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("name", ["crater", "crater_boulders", "rolling_hills", "boulder_field"])
def test_deviogram_and_slope_positive_finite_real_scenes(name):
    h, cell = _scene(name)
    bl = _baselines(cell)
    dev = deviogram(h, cell, bl)
    rms = rms_slope_vs_baseline(h, cell, bl)
    assert set(dev) == set(bl)  # all requested baselines resolve on a 256 grid
    assert set(rms) == set(bl)
    for L in bl:
        assert np.isfinite(dev[L]) and dev[L] > 0.0
        assert np.isfinite(rms[L]) and 0.0 < rms[L] < 90.0  # a real slope angle


# ---------------------------------------------------------------------------
# Structure function rises with baseline; RMS slope rolls off -- both on REAL terrain.
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("name", ["crater", "rolling_hills", "boulder_field"])
def test_deviogram_increases_with_baseline_real_terrain(name):
    """D(L) = RMS height difference grows with lag on real (correlated) relief."""
    h, cell = _scene(name)
    bl = _baselines(cell)
    dev = deviogram(h, cell, bl)
    vals = [dev[L] for L in bl]
    for lo, hi in zip(vals, vals[1:]):
        assert hi > lo, f"{name}: deviogram not increasing: {vals}"
    # And it genuinely spreads (not a flat white-noise field): far/near > 3x.
    assert vals[-1] > 3.0 * vals[0]


@pytest.mark.parametrize("name", ["crater", "crater_boulders", "rolling_hills", "boulder_field"])
def test_rms_slope_rolls_off_with_baseline_real_terrain(name):
    """RMS slope DECREASES with baseline -- the lunar slope-vs-scale signature."""
    h, cell = _scene(name)
    bl = _baselines(cell)
    rms = rms_slope_vs_baseline(h, cell, bl)
    vals = [rms[L] for L in bl]
    for lo, hi in zip(vals, vals[1:]):
        assert hi < lo, f"{name}: RMS slope did not roll off: {vals}"


def test_rms_slope_rolls_off_on_real_lola_dem():
    """The roll-off also holds on the real LOLA Haworth DEM at native-ish baselines."""
    if not os.path.exists(_LOLA):
        pytest.skip("real LOLA Haworth DEM not on disk")
    sub = np.fromfile(_LOLA, dtype="<f4").reshape(2000, 2000)[600:1100, 600:1100].astype(np.float64)
    cell = 5.0
    bl = [cell * m for m in (1, 2, 4, 8, 16, 32)]
    rms = rms_slope_vs_baseline(sub, cell, bl)
    dev = deviogram(sub, cell, bl)
    rvals = [rms[L] for L in bl]
    dvals = [dev[L] for L in bl]
    # RMS slope rolls off; structure function rises -- on genuine LOLA polar relief.
    for lo, hi in zip(rvals, rvals[1:]):
        assert hi < lo, f"LOLA RMS slope did not roll off: {rvals}"
    for lo, hi in zip(dvals, dvals[1:]):
        assert hi > lo, f"LOLA deviogram not increasing: {dvals}"


# ---------------------------------------------------------------------------
# Roughness ORDERING: flat_compact << rolling_hills << crater.
# This is the "sourced/match" test: a flat scene must read as nearly flat.
# ---------------------------------------------------------------------------

def test_flat_compact_is_near_zero_roughness_vs_rough_scenes():
    hf, cell = _scene("flat_compact")
    hr, _ = _scene("rolling_hills")
    hc, _ = _scene("crater")
    bl = _baselines(cell)

    dev_f = deviogram(hf, cell, bl)
    dev_r = deviogram(hr, cell, bl)
    dev_c = deviogram(hc, cell, bl)
    rms_f = rms_slope_vs_baseline(hf, cell, bl)
    rms_r = rms_slope_vs_baseline(hr, cell, bl)
    rms_c = rms_slope_vs_baseline(hc, cell, bl)

    for L in bl:
        # flat_compact is at least 5x smoother than rolling_hills, and rolling_hills
        # is itself smoother than the crater -- a strict roughness ordering.
        assert dev_f[L] * 5.0 < dev_r[L], f"flat not << rolling at L={L}"
        assert dev_r[L] < dev_c[L], f"rolling not < crater at L={L}"
        assert rms_f[L] * 5.0 < rms_r[L], f"flat slope not << rolling at L={L}"
        assert rms_r[L] < rms_c[L], f"rolling slope not < crater at L={L}"

    # In absolute terms the flat scene's slope is sub-degree; the crater is many degrees.
    assert max(rms_f.values()) < 1.0
    assert max(rms_c.values()) > 5.0


# ---------------------------------------------------------------------------
# Keying + baseline resolution contract on a real heightmap.
# ---------------------------------------------------------------------------

def test_keys_echo_requested_baselines_real_scene():
    h, cell = _scene("rolling_hills")
    requested = [cell * 2, cell * 8, cell * 32]
    dev = deviogram(h, cell, requested)
    assert sorted(dev.keys()) == sorted(float(L) for L in requested)


def test_unresolvable_baselines_dropped_real_scene():
    """Sub-cell and larger-than-field baselines are omitted; valid ones kept."""
    h, cell = _scene("crater")  # 256 cells @ 0.02 m -> field span 5.12 m
    sub_cell = cell / 4.0          # rounds to 0 cells -> dropped
    too_big = cell * 10_000.0      # far larger than the field -> dropped
    valid = cell * 4.0
    dev = deviogram(h, cell, [sub_cell, valid, too_big])
    rms = rms_slope_vs_baseline(h, cell, [sub_cell, valid, too_big])
    assert float(valid) in dev
    assert float(sub_cell) not in dev
    assert float(too_big) not in dev
    assert set(dev) == set(rms)


def test_field_must_be_2d():
    h, cell = _scene("crater")
    with pytest.raises(ValueError):
        deviogram(h.ravel(), cell, [cell * 2])
    with pytest.raises(ValueError):
        rms_slope_vs_baseline(h.ravel(), cell, [cell * 2])
