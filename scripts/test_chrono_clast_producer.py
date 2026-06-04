"""Guarded test for the live Chrono rigid-body producer (P7).

Runs only where PyChrono is importable (the `/tmp/chrono-env` micromamba env, NOT the runtime venv) — it
is SKIPPED on the bare suite. The producer's own `__main__` self-check is the run-verified validation;
this makes the suite acknowledge the producer and re-checks the core physics where Chrono is present.

Run under the Chrono env:
    MAMBA_ROOT_PREFIX=/tmp/mamba LD_LIBRARY_PATH=/tmp/chrono-env/lib \
        /tmp/chrono-env/bin/python -m pytest scripts/test_chrono_clast_producer.py -q

CC0-1.0 (see ../LICENSE).
"""

from __future__ import annotations

import math
import os
import sys

import pytest

pytest.importorskip("pychrono")        # skip on the runtime venv (no pychrono); run under chrono-env

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import chrono_clast_producer as ccp     # noqa: E402


def test_free_fall_matches_analytic_at_lunar_and_earth_g():
    for g in (ccp.G_MOON, ccp.G_EARTH):
        t = ccp.free_fall_time(1.0, -g)
        assert abs(t - math.sqrt(2.0 / g)) / math.sqrt(2.0 / g) < 0.02      # exact-physics check


def test_clasts_settle_on_the_surface_under_lunar_gravity():
    clasts = [(0.0, 0.0, 0.12), (0.4, 0.0, 0.10), (-0.3, 0.2, 0.08)]
    r = ccp.settle_clasts(clasts, gravity_z=-ccp.G_MOON)
    assert r["final_ke_J"] < 1e-2 * r["drop_pe_J"]                          # dissipates to rest
    for c in r["rest"]:
        assert c["radius_m"] * 0.85 <= c["z"] <= c["radius_m"] * 1.10       # rests ON the surface
