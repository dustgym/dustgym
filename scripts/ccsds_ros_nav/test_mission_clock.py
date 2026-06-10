"""Tests for the lunar mission clock + sun model (real Haworth crop for illumination)."""
from __future__ import annotations

import os

import pytest

import mission_clock as mc

_SCENE = os.path.join(os.path.dirname(__file__), "..", "..", "samples", "lunar_dem", "haworth_10km_5m")


def test_azimuth_sweeps_full_circle_per_synodic_month():
    az0, _ = mc.sun_az_el(0.0, az0_deg=10.0)
    az_half, _ = mc.sun_az_el(mc.SYNODIC_MONTH_S / 2, az0_deg=10.0)
    az_full, _ = mc.sun_az_el(mc.SYNODIC_MONTH_S, az0_deg=10.0)
    assert az0 == pytest.approx(10.0)
    assert az_half == pytest.approx(190.0)        # +180 deg at half a lunar day
    assert az_full == pytest.approx(10.0)         # back to start after one synodic month


def test_clock_advances_and_rebases_on_factor_change():
    t = [0.0]
    clk = mc.MissionClock(az0_deg=0.0, time_factor=100.0, now_fn=lambda: t[0])
    t[0] = 1.0
    assert clk.mission_time() == pytest.approx(100.0)     # 1 wall s x100
    clk.set_time_factor(1000.0)                            # rebase: mission time continuous
    assert clk.mission_time() == pytest.approx(100.0)
    t[0] = 2.0
    assert clk.mission_time() == pytest.approx(1100.0)     # +1 wall s x1000


@pytest.mark.skipif(not os.path.isdir(_SCENE), reason="Haworth sample not present")
def test_illuminated_start_and_sweep_change_lit_fraction():
    from flight import load_crop
    crop = load_crop(_SCENE, 720, 1800, 120, 120)
    az0, lf0 = mc.find_illuminated_start(crop.heightmap, crop.cell_m, el_deg=1.5)
    assert 0.0 <= lf0 <= 1.0 and lf0 > 0.3               # the rim-crest plateau is reasonably lit
    # the lit fraction must actually change as the sun sweeps (shadows move) — the informative property
    other = mc.lit_fraction(crop.heightmap, crop.cell_m, (az0 + 180) % 360, 1.5)
    assert abs(other - lf0) > 0.02
