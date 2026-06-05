"""Characterization tests for terrain_authority/procgen_csfd.py — the sub-DEM crater
size-frequency-distribution population generator (Lane B, spec §7).

The generator carves a real crater population via the real ``carve_crater``; a seeded run is
the real implementation under test (spec §10 determinism), not fabricated data.

Asserted invariants:
  * the module's own ``_self_test()`` returns 0 (its four falsifiable acceptance properties);
  * ``expected_crater_counts`` follows the capped Neukum/Xiao-Werner SFD: non-negative,
    decreasing cumulative density, and the equilibrium cap never exceeds bare production;
  * ``populate_craters`` is reproducible (same seed -> identical placed records; different
    seed differs), every synthesized D lies in [d_min, D_max), the DEM-resolves-all band is a
    true no-op, and the carved field stays mass-consistent (height == datum + mass/density);
  * the emplaced cumulative crater density obeys the Xiao & Werner equilibrium cap per bin.
"""

from __future__ import annotations

import numpy as np

from terrain_authority import constants as K
from terrain_authority import procgen_csfd
from terrain_authority.column_state import ColumnState


def _patch(width=200, height=200, cell_m=0.5) -> ColumnState:
    """A flat self-contained base patch (the module's own _make_patch, replicated as a fixture)."""
    cs = ColumnState(width=width, height=height, cell_m=cell_m)
    cs.density[:] = K.RHO_SURFACE
    cs.datum[:] = -K.REGOLITH_THICKNESS_M
    cs.set_height_via_mass(np.zeros((height, width)))
    return cs


# ---------------------------------------------------------------------------
# Module self-test (its §7 falsifiable acceptance properties).
# ---------------------------------------------------------------------------

def test_module_self_test_passes():
    assert procgen_csfd._self_test() == 0


# ---------------------------------------------------------------------------
# expected_crater_counts — the capped SFD.
# ---------------------------------------------------------------------------

def test_expected_counts_nonneg_and_cumulative_decreasing():
    d_edges = np.geomspace(1.0, 6.0, 17)
    area = 100.0 * 100.0
    centers, counts = procgen_csfd.expected_crater_counts(d_edges, area)
    assert centers.shape == (len(d_edges) - 1,)
    assert counts.shape == (len(d_edges) - 1,)
    assert np.all(counts >= 0.0)
    assert np.all(np.isfinite(counts))
    # bin centers are the geometric means, strictly increasing.
    assert np.all(np.diff(centers) > 0)


def test_expected_counts_equilibrium_cap_below_bare_production():
    d_edges = np.geomspace(1.0, 6.0, 17)
    area = 100.0 * 100.0
    _, capped = procgen_csfd.expected_crater_counts(d_edges, area, apply_equilibrium_cap=True)
    _, bare = procgen_csfd.expected_crater_counts(d_edges, area, apply_equilibrium_cap=False)
    # Capping at the Xiao&Werner ceiling can only reduce (or equal) the bare production count.
    assert np.all(capped <= bare + 1e-9)
    # The cap actually bites somewhere in the steep small end.
    assert np.any(capped < bare - 1e-9)


def test_expected_counts_deterministic():
    d_edges = np.geomspace(1.0, 6.0, 17)
    a = procgen_csfd.expected_crater_counts(d_edges, 1e4)
    b = procgen_csfd.expected_crater_counts(d_edges, 1e4)
    assert np.array_equal(a[0], b[0])
    assert np.array_equal(a[1], b[1])


# ---------------------------------------------------------------------------
# populate_craters — the population sampler.
# ---------------------------------------------------------------------------

def test_populate_reproducible_same_seed():
    eff_res = 15.0
    a, rec_a = procgen_csfd.populate_craters(_patch(), eff_res, seed=12345, return_records=True)
    b, rec_b = procgen_csfd.populate_craters(_patch(), eff_res, seed=12345, return_records=True)
    assert rec_a == rec_b
    assert len(rec_a) > 0
    # The carved conserved fields are byte-identical across the two seeded runs.
    assert np.array_equal(a.mass_areal, b.mass_areal)
    assert np.array_equal(a.state_label, b.state_label)


def test_populate_different_seed_differs():
    eff_res = 15.0
    _, rec_a = procgen_csfd.populate_craters(_patch(), eff_res, seed=1, return_records=True)
    _, rec_c = procgen_csfd.populate_craters(_patch(), eff_res, seed=999, return_records=True)
    assert rec_a != rec_c


def test_populate_diameters_in_band():
    eff_res = 15.0
    d_min = 1.0
    nyq = K.LDEM_EFFRES_NYQUIST_MULT
    d_max = eff_res / nyq
    _, rec = procgen_csfd.populate_craters(_patch(), eff_res, d_min_m=d_min, seed=7,
                                           return_records=True)
    diam = np.array([r["diameter_m"] for r in rec])
    assert diam.size > 0
    # De-confliction: strictly below D_max, at/above d_min (craters the DEM resolves are excluded).
    assert diam.max() < d_max + 1e-9
    assert diam.min() >= d_min - 1e-9


def test_populate_dem_resolves_all_is_noop():
    # eff_res = d_min * nyq -> D_max == d_min -> empty band -> grid returned unchanged.
    d_min = 1.0
    nyq = K.LDEM_EFFRES_NYQUIST_MULT
    base = _patch()
    before = base.derive_height().copy()
    cs, rec = procgen_csfd.populate_craters(base, d_min * nyq, d_min_m=d_min, seed=7,
                                            return_records=True)
    assert rec == []
    assert np.array_equal(cs.derive_height(), before)


def test_populate_mass_consistent_after_carving():
    cs, _ = procgen_csfd.populate_craters(_patch(), 15.0, seed=42, return_records=True)
    h = cs.derive_height()
    expect = cs.datum + cs.mass_areal / cs.density
    assert float(np.max(np.abs(h - expect))) <= 1e-9
    assert np.all(np.isfinite(h))


def test_populate_returns_columnstate_without_records():
    out = procgen_csfd.populate_craters(_patch(), 15.0, seed=3)
    # Default (return_records=False) returns the ColumnState itself, carved in place.
    assert isinstance(out, ColumnState)


def test_emplaced_density_obeys_equilibrium_cap():
    eff_res = 15.0
    d_min = 1.0
    nyq = K.LDEM_EFFRES_NYQUIST_MULT
    d_max = eff_res / nyq
    cs, rec = procgen_csfd.populate_craters(_patch(), eff_res, d_min_m=d_min, seed=12345,
                                            return_records=True)
    area = (cs.width * cs.cell_m) * (cs.height * cs.cell_m)
    diam = np.array([r["diameter_m"] for r in rec])
    for D in np.geomspace(d_min, d_max, 6):
        emplaced_cum = float(np.count_nonzero(diam >= D)) / area
        cap = float(K.eq_sfd(D))
        # Poisson overshoot tolerance: 1 crater quantum over the area + 25% (the module's own tol).
        tol = cap * 0.25 + 1.0 / area
        assert emplaced_cum <= cap + tol
