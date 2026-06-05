"""Characterization tests for terrain_authority/procgen.py (spec §4/§5/§9).

These exercise the REAL pure-NumPy procedural generators (fbm value noise, the rolling_hills
/ flat_compact archetypes, carve_crater, and the Golombek boulder sampler) at fixed seeds. A
seeded generator IS the real implementation under test — no synthetic data is fabricated; the
RNG draws are the deterministic output the spec promises (spec §10 determinism).

Asserted invariants (not trivia):
  * determinism — same seed -> byte-identical output; different seed differs;
  * output shapes / dtypes / finite value ranges;
  * fbm normalize modes ("minmax" -> [0,1]; "variance" -> zero-mean, RMS == target_rms);
  * archetypes preserve the conservation identity height == datum + mass_areal/density and
    the documented density envelopes / state labels;
  * carve_crater is mass-consistent (height re-derives) and produces a depressed bowl with a
    raised rim, labels the floor EXCAVATED, and obeys the requested d/D depth;
  * sample_boulders count scales with the requested Golombek area-coverage k (rocky > sparse)
    and emits clasts matching the INTERFACE §5 schema with diameters inside [d_min, d_max].
"""

from __future__ import annotations

import numpy as np
import pytest

from terrain_authority import constants as K
from terrain_authority import procgen
from terrain_authority.column_state import ColumnState, StateLabel


# ---------------------------------------------------------------------------
# fbm value noise.
# ---------------------------------------------------------------------------

def test_fbm_deterministic_byte_identical():
    a = procgen.fbm(64, 48, octaves=5, base_cells=4, seed=42)
    b = procgen.fbm(64, 48, octaves=5, base_cells=4, seed=42)
    # Byte-identical (determinism, spec §10).
    assert a.tobytes() == b.tobytes()
    assert np.array_equal(a, b)


def test_fbm_different_seed_differs():
    a = procgen.fbm(64, 64, seed=1)
    b = procgen.fbm(64, 64, seed=2)
    assert not np.array_equal(a, b)


def test_fbm_shape_dtype_and_minmax_range():
    h, w = 50, 70
    f = procgen.fbm(h, w, seed=3)
    assert f.shape == (h, w)
    assert f.dtype == np.float64
    assert np.all(np.isfinite(f))
    # Default "minmax" normalize -> renormalized to [0, 1].
    assert f.min() == pytest.approx(0.0, abs=1e-12)
    assert f.max() == pytest.approx(1.0, abs=1e-12)


def test_fbm_variance_mode_zero_mean_target_rms():
    target = 3.5
    f = procgen.fbm(80, 80, seed=5, normalize="variance", target_rms=target)
    assert np.all(np.isfinite(f))
    # Zero-mean field scaled to an exact target RMS deviation.
    assert float(f.mean()) == pytest.approx(0.0, abs=1e-9)
    rms = float(np.sqrt(np.mean(f ** 2)))
    assert rms == pytest.approx(target, rel=1e-9)


def test_fbm_variance_requires_target_rms():
    with pytest.raises(ValueError):
        procgen.fbm(16, 16, normalize="variance")


def test_fbm_unknown_normalize_raises():
    with pytest.raises(ValueError):
        procgen.fbm(16, 16, normalize="bogus")


# ---------------------------------------------------------------------------
# rolling_hills archetype.
# ---------------------------------------------------------------------------

def test_rolling_hills_deterministic_and_invariant():
    w, h, cm = 48, 48, 0.05
    a = procgen.rolling_hills(w, h, cm, seed=1)
    b = procgen.rolling_hills(w, h, cm, seed=1)
    # Determinism over the conserved fields.
    assert np.array_equal(a.mass_areal, b.mass_areal)
    assert np.array_equal(a.density, b.density)
    assert np.array_equal(a.disturbance, b.disturbance)

    assert a.width == w and a.height == h and a.cell_m == cm
    assert a.mass_areal.shape == (h, w)
    # Conservation identity (INTERFACE.md §4): height == datum + mass/density.
    expect = a.datum + a.mass_areal / a.density
    assert np.allclose(a.derive_height(), expect, atol=1e-12)
    assert np.all(np.isfinite(a.derive_height()))
    # Mass non-negative (set_height_via_mass clamps thickness >= 0).
    assert np.all(a.mass_areal >= 0.0)

    # Fluffy low-density envelope: clipped to [0.9*RHO_SURFACE, RHO_SURFACE] (spec §9).
    assert np.all(a.density >= 0.9 * K.RHO_SURFACE - 1e-9)
    assert np.all(a.density <= K.RHO_SURFACE + 1e-9)
    # All VIRGIN, undriven low disturbance.
    assert np.all(a.state_label == StateLabel.VIRGIN)
    assert np.all((a.disturbance >= 0.0) & (a.disturbance <= 0.02 + 1e-9))


def test_rolling_hills_amplitude_scales_relief():
    w, h, cm = 64, 64, 0.05
    lo = procgen.rolling_hills(w, h, cm, seed=7, amplitude_m=0.05)
    hi = procgen.rolling_hills(w, h, cm, seed=7, amplitude_m=0.5)
    rng_lo = np.ptp(lo.derive_height())
    rng_hi = np.ptp(hi.derive_height())
    # Larger amplitude -> larger surface relief at the same seed.
    assert rng_hi > rng_lo


# ---------------------------------------------------------------------------
# flat_compact archetype.
# ---------------------------------------------------------------------------

def test_flat_compact_dense_flat_undisturbed():
    w, h, cm = 40, 40, 0.05
    cs = procgen.flat_compact(w, h, cm, seed=2)
    # Compacted plate at the deep density.
    assert np.allclose(cs.density, K.RHO_DEEP)
    # Tiny micro-relief only (amplitude_m default 0.01) -> near-flat.
    assert np.ptp(cs.derive_height()) < 0.05
    assert np.all(cs.state_label == StateLabel.VIRGIN)
    assert np.all(cs.disturbance == 0.0)
    # Conservation identity holds.
    assert np.allclose(cs.derive_height(), cs.datum + cs.mass_areal / cs.density, atol=1e-12)


def test_flat_compact_deterministic():
    a = procgen.flat_compact(32, 32, 0.05, seed=9)
    b = procgen.flat_compact(32, 32, 0.05, seed=9)
    assert np.array_equal(a.mass_areal, b.mass_areal)
    assert np.array_equal(a.density, b.density)


# ---------------------------------------------------------------------------
# carve_crater.
# ---------------------------------------------------------------------------

def _flat_patch(w=120, h=120, cm=0.1) -> ColumnState:
    """A real flat base ColumnState with a deep datum so bowls never clamp to 0 mass."""
    cs = ColumnState(width=w, height=h, cell_m=cm)
    cs.density[:] = K.RHO_SURFACE
    cs.datum[:] = -K.REGOLITH_THICKNESS_M
    cs.set_height_via_mass(np.zeros((h, w)))
    return cs


def test_carve_crater_bowl_rim_and_mass_consistency():
    cs = _flat_patch()
    before = cs.derive_height().copy()
    diameter = 4.0  # m
    cm = cs.cell_m
    center = (cs.height // 2, cs.width // 2)
    procgen.carve_crater(cs, center, diameter)

    h = cs.derive_height()
    # Mass-consistency: height == datum + mass/density after carving (spec §6/§10).
    expect = cs.datum + cs.mass_areal / cs.density
    assert np.allclose(h, expect, atol=1e-9)
    assert np.all(np.isfinite(h))

    # Center is depressed below the original flat surface (a real bowl).
    r0, c0 = center
    assert h[r0, c0] < before[r0, c0] - 1e-3

    # Depth at center ~ depth_ratio * diameter (Pike-class).
    depth_center = before[r0, c0] - h[r0, c0]
    assert depth_center == pytest.approx(K.CRATER_DEPTH_DIAMETER_RATIO * diameter, rel=0.05)

    # A raised rim exists: somewhere along radius ~R the surface rises above the original.
    R = 0.5 * diameter
    rr = int(round(R / cm))
    rim_row = h[r0, c0 - rr - 1: c0 + rr + 2]
    assert rim_row.max() > before[r0, c0] + 1e-4

    # Floor is labelled EXCAVATED; disturbance bumped in the bowl.
    assert np.any(cs.state_label == StateLabel.EXCAVATED)
    assert cs.disturbance[r0, c0] > 0.0


def test_carve_crater_deterministic_and_far_field_untouched():
    a = _flat_patch()
    b = _flat_patch()
    procgen.carve_crater(a, (60, 60), 3.0)
    procgen.carve_crater(b, (60, 60), 3.0)
    assert np.array_equal(a.mass_areal, b.mass_areal)
    assert np.array_equal(a.state_label, b.state_label)
    # A corner far outside the ejecta extent is untouched.
    assert a.derive_height()[0, 0] == pytest.approx(_flat_patch().derive_height()[0, 0], abs=1e-12)


def test_carve_crater_mcgetchin_ejecta_mode_runs_and_conserves_identity():
    cs = _flat_patch()
    procgen.carve_crater(cs, (60, 60), 3.0, ejecta_mode="mcgetchin")
    assert np.allclose(cs.derive_height(), cs.datum + cs.mass_areal / cs.density, atol=1e-9)


def test_carve_crater_size_dependent_depth_shallower_small():
    # size_dependent only substitutes when depth_ratio is left at the legacy default.
    cs = _flat_patch()
    r0, c0 = 60, 60
    before = cs.derive_height()[r0, c0]
    small_d = 2.0  # < 400 m transition -> CRATER_DD_SMALL_NOMINAL (~0.13)
    procgen.carve_crater(cs, (r0, c0), small_d, size_dependent=True)
    depth_center = before - cs.derive_height()[r0, c0]
    assert depth_center == pytest.approx(K.crater_depth_ratio(small_d) * small_d, rel=0.05)
    # Shallower than the flat-0.2 law for a sub-400 m crater.
    assert K.crater_depth_ratio(small_d) < K.CRATER_DEPTH_DIAMETER_RATIO


def test_carve_crater_unknown_ejecta_mode_raises():
    cs = _flat_patch()
    with pytest.raises(ValueError):
        procgen.carve_crater(cs, (60, 60), 3.0, ejecta_mode="nope")


# ---------------------------------------------------------------------------
# sample_boulders (Golombek area SFD).
# ---------------------------------------------------------------------------

def test_sample_boulders_deterministic():
    a = procgen.sample_boulders(40, 40, 0.5, k=0.1, seed=7)
    b = procgen.sample_boulders(40, 40, 0.5, k=0.1, seed=7)
    assert a == b


def test_sample_boulders_count_scales_with_k():
    # Rocky (high k) yields more rocks than sparse (low k) at the same area/seed.
    sparse = procgen.sample_boulders(60, 60, 0.5, k=0.05, seed=7)
    rocky = procgen.sample_boulders(60, 60, 0.5, k=0.2, seed=7)
    assert len(rocky) > len(sparse)
    assert len(sparse) > 0


def test_sample_boulders_schema_and_diameter_band():
    d_min, d_max = 0.04, 0.6
    w, h, cm = 60, 60, 0.5
    clasts = procgen.sample_boulders(w, h, cm, k=0.2, d_min_m=d_min, d_max_m=d_max, seed=7)
    assert len(clasts) > 0
    ids = [c["id"] for c in clasts]
    # Unique ascending ids 0..n-1 (INTERFACE.md §5 clasts schema).
    assert ids == list(range(len(clasts)))
    Wm, Hm = w * cm, h * cm
    for c in clasts:
        assert set(c) >= {"id", "center_m", "radius_m", "shape", "buried_frac"}
        assert c["shape"] == "sphere"
        x, _y, z = c["center_m"]
        assert 0.0 <= x <= Wm
        assert 0.0 <= z <= Hm
        diameter = 2.0 * c["radius_m"]
        # Diameters are bin geometric-mean centers, strictly inside [d_min, d_max].
        assert d_min <= diameter <= d_max
        assert 0.1 <= c["buried_frac"] <= 0.7
