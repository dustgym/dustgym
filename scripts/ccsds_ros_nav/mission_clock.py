"""Lunar mission clock + sun geometry for the Haworth site.

The operator console runs a mission clock that advances at a configurable acceleration (``time_factor``
sim-seconds per wall-second) and maps mission time to the Sun's position over the (south-polar) patch.
Accelerating the clock sweeps the Sun's azimuth, which walks terrain-cast shadows across Haworth — the
informative thing for planning a single lunar-day surface mission.

This is a deliberately SIMPLE analytic model, not a SPICE ephemeris:
  * Azimuth sweeps a full 360 deg per **synodic month** (29.530589 d) — the Sun circles the horizon once
    per lunar day. [SOURCED: synodic month 29.530589 d.]
  * Elevation is held at a low grazing value (polar sites see the Sun near the horizon; the Moon's
    obliquity to the ecliptic is only ~1.54 deg, so the sub-solar latitude — hence polar Sun elevation —
    stays within a couple of degrees). [SOURCED: lunar obliquity 1.54 deg; ASSUMPTION: constant grazing
    elevation over a single lunar day — true elevation libration is a follow-up.]

Tagged [ASSUMPTION]/[SOURCED] per the repo's provenance convention; do not read this as flight-grade
ephemeris. Lit/shadow itself is computed by the real terrain via ``illumination.horizon_clip``.
"""
from __future__ import annotations

import os
import sys
import time

import numpy as np

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from terrain_authority import illumination

#: Synodic (Sun-relative) lunar day. [SOURCED] 29.530589 mean solar days.
SYNODIC_MONTH_S = 29.530589 * 86400.0
#: Lunar obliquity to the ecliptic [deg]. [SOURCED] ~1.54 deg -> polar Sun stays near grazing.
LUNAR_OBLIQUITY_DEG = 1.54
#: Default grazing elevation for a south-polar patch [deg]. [ASSUMPTION] (see module docstring).
DEFAULT_SUN_EL_DEG = 1.5


def sun_az_el(mission_time_s: float, *, az0_deg: float = 0.0, el_deg: float = DEFAULT_SUN_EL_DEG,
              period_s: float = SYNODIC_MONTH_S) -> tuple[float, float]:
    """Sun (azimuth [deg, from north], elevation [deg]) at mission time. Azimuth = az0 at t=0."""
    az = (az0_deg + 360.0 * (mission_time_s / period_s)) % 360.0
    return az, float(el_deg)


def lit_fraction(heightmap: np.ndarray, cell_m: float, az_deg: float, el_deg: float) -> float:
    """Fraction of the patch directly lit under one Sun position (terrain cast-shadow, horizon_clip)."""
    return float(illumination.horizon_clip(heightmap, cell_m, az_deg, el_deg).mean())


def find_illuminated_start(heightmap: np.ndarray, cell_m: float, *,
                           el_deg: float = DEFAULT_SUN_EL_DEG, n_az: int = 72) -> tuple[float, float]:
    """Scan azimuth (n_az steps) for the best-lit Sun position; return (az0_deg, lit_fraction)."""
    best_az, best_lf = 0.0, -1.0
    for i in range(n_az):
        az = 360.0 * i / n_az
        lf = lit_fraction(heightmap, cell_m, az, el_deg)
        if lf > best_lf:
            best_az, best_lf = az, lf
    return best_az, best_lf


class MissionClock:
    """Wall-clock-driven mission time at a configurable acceleration, mapped to Sun geometry."""

    def __init__(self, *, az0_deg: float, el_deg: float = DEFAULT_SUN_EL_DEG,
                 time_factor: float = 1.0, period_s: float = SYNODIC_MONTH_S,
                 now_fn=time.monotonic) -> None:
        self.az0_deg = float(az0_deg)
        self.el_deg = float(el_deg)
        self.period_s = float(period_s)
        self.time_factor = float(time_factor)
        self._now = now_fn
        self._wall0 = now_fn()
        self._mission0 = 0.0

    def mission_time(self) -> float:
        """Mission-elapsed seconds (= wall-elapsed x time_factor since the last rate change)."""
        return self._mission0 + (self._now() - self._wall0) * self.time_factor

    def set_time_factor(self, factor: float) -> None:
        """Change the acceleration, rebasing so mission time stays continuous across the change."""
        self._mission0 = self.mission_time()
        self._wall0 = self._now()
        self.time_factor = float(factor)

    def lunar_day_fraction(self) -> float:
        """Mission time as a fraction of one lunar (synodic) day."""
        return (self.mission_time() / self.period_s) % 1.0

    def sun(self) -> tuple[float, float]:
        """Current Sun (azimuth, elevation) [deg]."""
        return sun_az_el(self.mission_time(), az0_deg=self.az0_deg, el_deg=self.el_deg,
                         period_s=self.period_s)
