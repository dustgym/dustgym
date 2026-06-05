"""Characterization tests for terrain_authority/refinement.py — mass-exact variable-resolution
operators + tile extraction (spec §2.4, INTERFACE.md §5.3).

The base field under test is a REAL committed sample scene (samples/rolling_hills, a procgen
archetype written to disk), loaded back through the real io_fields.load_scene and reconstructed
into a ColumnState the way worksite.coarse_base_from_bundle does (datum = heightmap -
mass/density). No synthetic data: the rasters are the real generator output frozen on disk. The
real Haworth LOLA DEM bundle is used for a second base where available.

Asserted invariants (spec §2.4 / INTERFACE.md §5.3 CONSERVATION INVARIANT):
  * k_factor accepts integer ratios (incl. IEEE division noise) and rejects non-integers / <=0;
  * refine copies mass_areal (intensive) so total mass Σ mass·cell_area is conserved exactly,
    and dims become (H*k, W*k);
  * coarsen∘refine == base BIT-EXACT for every carried field at multiple k (drift 0);
  * coarsen halves dims, conserves total mass to ~1e-9, and reproduces base height to f32 tol;
  * state_label coarsens by the TOTAL priority order EXCAVATED > SPOIL > COMPACTED_BERM > TREAD
    > VIRGIN (verified by brute force over child combinations), and is associative across levels;
  * extract_tiles emits base-cell-aligned, disjoint, scan-ordered tiles whose coarsen() equals
    the base block they came from (base<->tile round-trip), and rejects overlaps / out-of-bounds.
"""

from __future__ import annotations

import itertools
import os

import numpy as np
import pytest

from terrain_authority import refinement as R
from terrain_authority.column_state import ColumnState, StateLabel
from terrain_authority.io_fields import load_scene

_HERE = os.path.dirname(os.path.abspath(__file__))
_SAMPLES = os.path.normpath(os.path.join(_HERE, "..", "samples"))


def _base_from_sample(name: str, crop: int | None = 24) -> ColumnState:
    """Load a real committed sample scene into a ColumnState (datum reconstructed), optionally
    cropped to a small block to keep refine/coarsen fast while staying REAL data."""
    fields, meta = load_scene(os.path.join(_SAMPLES, name))
    g = meta["grid"]
    mass = fields["mass_areal"].astype(np.float64)
    rho = fields["density"].astype(np.float64)
    datum = fields["heightmap"].astype(np.float64) - mass / rho
    if crop is not None:
        sl = (slice(0, crop), slice(0, crop))
        mass, rho, datum = mass[sl], rho[sl], datum[sl]
        state = fields["state_label"][sl].astype(np.uint8)
        dist = fields["disturbance"][sl].astype(np.float64)
        h, w = mass.shape
    else:
        state = fields["state_label"].astype(np.uint8)
        dist = fields["disturbance"].astype(np.float64)
        h, w = int(g["height"]), int(g["width"])
    return ColumnState(width=w, height=h, cell_m=float(g["cell_m"]),
                       mass_areal=mass, density=rho, state_label=state,
                       disturbance=dist, datum=datum)


@pytest.fixture(scope="module")
def base() -> ColumnState:
    return _base_from_sample("rolling_hills")


# ---------------------------------------------------------------------------
# k_factor validation (spec §6.2d).
# ---------------------------------------------------------------------------

def test_k_factor_exact_integer():
    assert R.k_factor(0.05, 0.01) == 5
    assert R.k_factor(0.08, 0.02) == 4
    assert R.k_factor(1.0, 1.0) == 1


def test_k_factor_absorbs_division_noise():
    # 0.02 / 0.01 lands at 1.999...8 in IEEE-754; must still accept as k=2.
    assert R.k_factor(0.02, 0.01) == 2
    # k=8 is the mission config (8 cm base / 1 cm touched band).
    assert R.k_factor(0.08, 0.01) == 8


def test_k_factor_rejects_noninteger():
    with pytest.raises(ValueError):
        R.k_factor(0.02, 0.012)  # 1.666...


def test_k_factor_rejects_nonpositive():
    with pytest.raises(ValueError):
        R.k_factor(0.0, 0.01)
    with pytest.raises(ValueError):
        R.k_factor(0.01, -0.01)


def test_k_factor_rejects_fine_coarser_than_base():
    with pytest.raises(ValueError):
        R.k_factor(0.01, 0.05)  # ratio < 1


# ---------------------------------------------------------------------------
# refine_field — copy semantics, dims, mass conservation.
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("k", [2, 3, 4])
def test_refine_dims_and_dtypes(base, k):
    fine = R.refine_field(base, k)
    h, w = base.mass_areal.shape
    for name in ("mass_areal", "density", "datum", "disturbance"):
        assert fine[name].shape == (h * k, w * k)
        assert fine[name].dtype == np.float64
    assert fine["state_label"].shape == (h * k, w * k)
    assert fine["state_label"].dtype == np.uint8


@pytest.mark.parametrize("k", [2, 3, 4])
def test_refine_conserves_total_mass(base, k):
    cell = base.cell_m
    fine_cell = cell / k
    base_mass = float(base.mass_areal.sum()) * cell * cell
    fine = R.refine_field(base, k)
    fine_mass = float(fine["mass_areal"].sum()) * fine_cell * fine_cell
    # mass_areal is INTENSIVE -> copied, not divided; total Σ mass·cell_area conserved.
    assert fine_mass == pytest.approx(base_mass, rel=1e-12)


def test_refine_is_verbatim_copy(base):
    k = 3
    fine = R.refine_field(base, k)
    # Each parent (r,c) maps to the k x k child block [r*k:(r+1)*k, c*k:(c+1)*k] verbatim.
    expanded = np.repeat(np.repeat(base.mass_areal, k, axis=0), k, axis=1)
    assert np.array_equal(fine["mass_areal"], expanded)


def test_refine_rejects_bad_k(base):
    with pytest.raises(ValueError):
        R.refine_field(base, 0)
    with pytest.raises(ValueError):
        R.refine_field(base, 1.5)


# ---------------------------------------------------------------------------
# coarsen_field — round-trip, mass, height, dims.
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("k", [2, 3, 4, 8])
def test_coarsen_refine_bit_exact(base, k):
    fine = R.refine_field(base, k)
    back = R.coarsen_field(fine, k)
    # coarsen(refine(x)) == x BIT-EXACT in float64 (spec §2.4 "Drift = 0").
    for name in ("mass_areal", "density", "datum", "disturbance"):
        assert np.array_equal(back[name], np.asarray(getattr(base, name), dtype=np.float64)), name
    assert np.array_equal(back["state_label"], base.state_label)


@pytest.mark.parametrize("k", [2, 4])
def test_coarsen_halves_dims_and_conserves_mass(base, k):
    fine = R.refine_field(base, k)
    fine_cell = base.cell_m / k
    coarse = R.coarsen_field(fine, k)
    hf, wf = fine["mass_areal"].shape
    assert coarse["mass_areal"].shape == (hf // k, wf // k)
    # Total mass conserved coarsening fine -> coarse (to ~1e-9 relative).
    fine_mass = float(fine["mass_areal"].sum()) * fine_cell * fine_cell
    coarse_mass = float(coarse["mass_areal"].sum()) * base.cell_m * base.cell_m
    assert coarse_mass == pytest.approx(fine_mass, rel=1e-9)


def test_coarsen_height_matches_base_to_f32_tol(base):
    k = 4
    fine = R.refine_field(base, k)
    coarse = R.coarsen_field(fine, k)
    base_h = base.derive_height()
    coarse_h = coarse["datum"] + coarse["mass_areal"] / coarse["density"]
    assert np.allclose(coarse_h, base_h, atol=1e-6)


def test_coarsen_rejects_indivisible_dims(base):
    # A (H*k+1) field is not divisible by k -> ValueError.
    fine = R.refine_field(base, 2)
    bad = {name: arr[:-1] for name, arr in fine.items()}  # odd row count
    with pytest.raises(ValueError):
        R.coarsen_field(bad, 2)


def test_coarsen_rejects_bad_k(base):
    fine = R.refine_field(base, 2)
    with pytest.raises(ValueError):
        R.coarsen_field(fine, 0)


# ---------------------------------------------------------------------------
# state_label total-order coarsening (associativity / priority, brute force).
# ---------------------------------------------------------------------------

def _coarsen_one_block(labels_2x2: list[int]) -> int:
    """Coarsen a single 2x2 state-label block via the real operator; return the winning label."""
    h, w, k = 1, 1, 2
    arr = np.array(labels_2x2, dtype=np.uint8).reshape(k, k)
    fields = {
        "mass_areal": np.ones((h * k, w * k)),
        "density": np.full((h * k, w * k), 1300.0),
        "datum": np.zeros((h * k, w * k)),
        "disturbance": np.zeros((h * k, w * k)),
        "state_label": arr,
    }
    out = R.coarsen_field(fields, k)
    return int(out["state_label"][0, 0])


def test_state_label_priority_total_order():
    labels = [int(StateLabel.EXCAVATED), int(StateLabel.SPOIL),
              int(StateLabel.COMPACTED_BERM), int(StateLabel.TREAD), int(StateLabel.VIRGIN)]
    priority = {int(StateLabel.EXCAVATED): 4, int(StateLabel.SPOIL): 3,
                int(StateLabel.COMPACTED_BERM): 2, int(StateLabel.TREAD): 1,
                int(StateLabel.VIRGIN): 0}
    # Brute force over all 5^4 child combinations of a 2x2 block: the winner is always the
    # highest-priority child by the TOTAL order (most-worked / most-salient wins).
    for combo in itertools.product(labels, repeat=4):
        won = _coarsen_one_block(list(combo))
        expected = max(combo, key=lambda lbl: priority[lbl])
        assert priority[won] == priority[max(combo, key=lambda lbl: priority[lbl])]
        assert won == expected


def test_state_label_associative_across_levels(base):
    # Coarsen by k=4 in one shot vs two k=2 levels -> identical state labels (associativity).
    fine = R.refine_field(base, 4)
    one_shot = R.coarsen_field(fine, 4)["state_label"]
    two_level = R.coarsen_field(R.coarsen_field(fine, 2), 2)["state_label"]
    assert np.array_equal(one_shot, two_level)


# ---------------------------------------------------------------------------
# Heterogeneous coarsen (genuine reduction, not refine-then-coarsen).
# ---------------------------------------------------------------------------

def test_coarsen_heterogeneous_mass_is_area_mean():
    # A genuine 2x2 block with distinct masses -> coarse mass_areal is the simple mean.
    k = 2
    mass = np.array([[1.0, 3.0], [5.0, 7.0]])
    fields = {
        "mass_areal": mass,
        "density": np.full((2, 2), 1300.0),
        "datum": np.zeros((2, 2)),
        "disturbance": np.array([[0.0, 0.2], [0.4, 0.6]]),
        "state_label": np.full((2, 2), int(StateLabel.VIRGIN), dtype=np.uint8),
    }
    out = R.coarsen_field(fields, k)
    assert out["mass_areal"][0, 0] == pytest.approx(mass.mean())
    assert out["disturbance"][0, 0] == pytest.approx(0.3)
    # Uniform density block -> coarse density equals that shared density exactly.
    assert out["density"][0, 0] == pytest.approx(1300.0)


def test_coarsen_zero_mass_block_height_equals_datum():
    # All-empty block (mass 0): density falls back to mean(density), height == datum (no NaN).
    k = 2
    fields = {
        "mass_areal": np.zeros((2, 2)),
        "density": np.full((2, 2), 1300.0),
        "datum": np.full((2, 2), -5.0),
        "disturbance": np.zeros((2, 2)),
        "state_label": np.full((2, 2), int(StateLabel.VIRGIN), dtype=np.uint8),
    }
    out = R.coarsen_field(fields, k)
    assert np.isfinite(out["density"][0, 0]) and out["density"][0, 0] > 0
    height = out["datum"][0, 0] + out["mass_areal"][0, 0] / out["density"][0, 0]
    assert height == pytest.approx(-5.0)


# ---------------------------------------------------------------------------
# extract_tiles — base<->tile round-trip, disjointness, ordering, bounds.
# ---------------------------------------------------------------------------

def test_extract_tiles_roundtrip_coarsen_equals_base_block(base):
    fine_cell = base.cell_m / 4  # k = 4
    k = R.k_factor(base.cell_m, fine_cell)
    boxes = [[0, 0, 4, 4], [4, 4, 8, 8]]
    tiles = R.extract_tiles(base, boxes, fine_cell)
    assert len(tiles) == 2
    for t in tiles:
        r0, c0, r1, c1 = t.region_rc
        # Fine dims == (r1-r0)*k x (c1-c0)*k.
        assert t.cs.height == (r1 - r0) * k and t.cs.width == (c1 - c0) * k
        assert t.cell_m == pytest.approx(fine_cell)
        # coarsen(tile) == the base block it came from (base<->tile consistency, §5.3).
        back = R.coarsen_field(t.fields, k)
        assert np.array_equal(back["mass_areal"], base.mass_areal[r0:r1, c0:c1])
        assert np.array_equal(back["state_label"], base.state_label[r0:r1, c0:c1])
        assert np.allclose(back["density"], base.density[r0:r1, c0:c1])
        assert np.allclose(back["datum"], base.datum[r0:r1, c0:c1])


def test_extract_tiles_scan_order_and_unique_ids(base):
    fine_cell = base.cell_m / 2
    # Provide out of scan order + a duplicate; expect dedup + (r0,c0) sort + 0..n-1 ids.
    boxes = [[4, 0, 6, 2], [0, 0, 2, 2], [0, 0, 2, 2]]
    tiles = R.extract_tiles(base, boxes, fine_cell)
    assert [t.id for t in tiles] == list(range(len(tiles)))
    starts = [(t.region_rc[0], t.region_rc[1]) for t in tiles]
    assert starts == sorted(starts)
    assert len(tiles) == 2  # duplicate dropped


def test_extract_tiles_rejects_overlap(base):
    fine_cell = base.cell_m / 2
    boxes = [[0, 0, 4, 4], [2, 2, 6, 6]]  # geometrically overlapping
    with pytest.raises(ValueError):
        R.extract_tiles(base, boxes, fine_cell)


def test_extract_tiles_rejects_out_of_bounds(base):
    fine_cell = base.cell_m / 2
    h = base.height
    boxes = [[0, 0, h + 2, 2]]  # exceeds the base grid
    with pytest.raises(ValueError):
        R.extract_tiles(base, boxes, fine_cell)


def test_extract_tiles_descriptor_shape(base):
    fine_cell = base.cell_m / 2
    tiles = R.extract_tiles(base, [[0, 0, 2, 2]], fine_cell)
    d = tiles[0].descriptor(dir_="tiles/0")
    assert d["id"] == 0
    assert d["region_rc"] == [0, 0, 2, 2]
    assert d["cell_m"] == pytest.approx(fine_cell)
    assert d["dir"] == "tiles/0"


def test_merge_leaf_boxes_dedups_and_drops_degenerate(base):
    boxes = [[0, 0, 2, 2], [0, 0, 2, 2], [3, 3, 3, 5], [1, 1, 3, 3]]
    out = R.merge_leaf_boxes_to_aligned_regions(boxes, base.cell_m, base.cell_m / 2)
    # Degenerate (r1<=r0) dropped, duplicate deduped, sorted by (r0,c0).
    assert out == [[0, 0, 2, 2], [1, 1, 3, 3]]


# ---------------------------------------------------------------------------
# A second REAL base: the committed Haworth LOLA DEM bundle (if present).
# ---------------------------------------------------------------------------

def test_haworth_dem_refine_coarsen_roundtrip():
    bundle = os.path.join(_SAMPLES, "lunar_dem", "haworth_10km_5m")
    if not os.path.exists(os.path.join(bundle, "metadata.json")):
        pytest.skip("Haworth DEM bundle not present")
    fields, meta = load_scene(bundle)
    g = meta["grid"]
    mass = fields["mass_areal"].astype(np.float64)[:16, :16]
    rho = fields["density"].astype(np.float64)[:16, :16]
    datum = fields["heightmap"].astype(np.float64)[:16, :16] - mass / rho
    base = ColumnState(width=16, height=16, cell_m=float(g["cell_m"]),
                       mass_areal=mass, density=rho,
                       state_label=fields["state_label"][:16, :16].astype(np.uint8),
                       disturbance=fields["disturbance"].astype(np.float64)[:16, :16],
                       datum=datum)
    k = 3
    back = R.coarsen_field(R.refine_field(base, k), k)
    assert np.array_equal(back["mass_areal"], base.mass_areal)
    assert np.array_equal(back["density"], base.density)
    # Mass conserved through refine on the real DEM block.
    fc = base.cell_m / k
    fine = R.refine_field(base, k)
    assert float(fine["mass_areal"].sum()) * fc * fc == pytest.approx(
        float(base.mass_areal.sum()) * base.cell_m * base.cell_m, rel=1e-12)
