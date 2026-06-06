"""Tests for the static tip-over stability model (stability.py) — the 'don't tip' criterion.

Pure geometry/mechanics (analytic ground truth, no synthetic data). Verifies the SSA = atan(half/cg)
identity, that wider/lower is more stable and taller/narrower tips earlier, and that the per-axis margin +
risk band behave on the modeled rover geometry (gauge 0.57 / wheelbase 0.40 -> pitch binds).
"""
from __future__ import annotations

import math

import pytest

from terrain_authority import constants as K
from terrain_authority import stability as ST
from terrain_authority.rover import WHEEL_BASE_M, WHEEL_GAUGE_M


def test_ssa_is_atan_half_over_cg():
    assert ST.ssa_deg(0.5, 0.5) == pytest.approx(45.0)                 # equal half-base and CG -> 45 deg
    assert ST.ssa_deg(1.0, 0.5) == pytest.approx(math.degrees(math.atan2(1.0, 0.5)))  # ~63.4
    assert ST.ssa_deg(0.0, 0.5) == 0.0                                 # zero support -> tips at any tilt
    assert ST.ssa_deg(0.5, 0.0) == 90.0                                # zero CG -> never tips (degenerate)


def test_wider_lower_is_more_stable_than_taller_narrower():
    wide_low = ST.ssa_deg(1.0, 0.3)
    tall_narrow = ST.ssa_deg(0.2, 0.6)
    assert wide_low > tall_narrow                                      # a low wide rover tolerates more tilt


def test_pitch_binds_on_the_modeled_geometry():
    # gauge 0.57 > wheelbase 0.40 -> the rover is wider than long -> the PITCH axis (shorter half-base) binds
    lim = ST.tip_tilt_limit_deg(gauge_m=WHEEL_GAUGE_M, wheelbase_m=WHEEL_BASE_M, cg_height_m=K.CG_HEIGHT_M)
    ssa_pitch = ST.ssa_deg(WHEEL_BASE_M / 2.0, K.CG_HEIGHT_M)
    ssa_roll = ST.ssa_deg(WHEEL_GAUGE_M / 2.0, K.CG_HEIGHT_M)
    assert ssa_pitch < ssa_roll                                        # pitch is the binding (smaller) SSA
    assert lim == pytest.approx(ssa_pitch)
    assert 25.0 < lim < 45.0                                           # ~33.7 deg; the default traverse cap (25) is below it


def test_stability_margin_and_risk_bands():
    geo = dict(gauge_m=WHEEL_GAUGE_M, wheelbase_m=WHEEL_BASE_M, cg_height_m=K.CG_HEIGHT_M)
    flat = ST.stability(0.0, 0.0, **geo)
    assert flat["risk"] == "ok" and flat["margin_deg"] == pytest.approx(min(flat["ssa_pitch_deg"], flat["ssa_roll_deg"]))
    # a pitch just past the pitch SSA -> tip, binding axis pitch, negative margin
    over = ST.stability(flat["ssa_pitch_deg"] + 1.0, 0.0, **geo)
    assert over["risk"] == "tip" and over["binding_axis"] == "pitch" and over["margin_deg"] < 0.0
    # a pitch in the warn band (>= 0.7*SSA, < SSA)
    warn = ST.stability(0.8 * flat["ssa_pitch_deg"], 0.0, **geo)
    assert warn["risk"] == "warn" and warn["margin_deg"] > 0.0
