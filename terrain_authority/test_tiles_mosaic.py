"""Characterization tests for ``terrain_authority.tiles_mosaic`` — the demand-driven, bounded,
evictable corridor LOD mosaic over a global-frame tiled base (L0 §0/§5).

The base reader is an ``ArrayBaseReader`` over a REAL LOLA Haworth base block built via
``dem_import.dem_to_base`` (same real-data idiom as ``test_dem_io.py`` / ``test_dem_overlay.py``).
No synthetic field is fabricated; the coarse base is the conserved authority's real output over
real lunar relief, and the fine tiles are produced by the real ``dem_overlay.overlay_residual``.

Tests cover the three hard L0 consequences the module enforces: (1) demand-driven refine around
a moving pose materializes fine tiles; (2) coordinate-hashed determinism makes a regenerated tile
byte-identical; (3) the resident set stays bounded under free roaming via LRU eviction. Plus the
additive metadata writer and the geometry helpers.
"""

from __future__ import annotations

import json
import os

import numpy as np
import pytest

from terrain_authority import dem_io
from terrain_authority.dem_import import Affine, crop_square, dem_to_base
from terrain_authority.dem_io import BASE_FIELD_NAMES, ArrayBaseReader
from terrain_authority.refinement import Tile
from terrain_authority.tiles_mosaic import (
    FineTile,
    TileMosaic,
    write_dem_base_metadata,
)

_DEM_DIR = os.path.join(
    os.path.dirname(__file__), os.pardir, "samples", "lunar_dem", "haworth_10km_5m",
)
_HEIGHTMAP = os.path.join(_DEM_DIR, "heightmap.rf32")
_METADATA = os.path.join(_DEM_DIR, "metadata.json")
_CELL_M = 5.0
_X0 = -52900.0
_Y0 = 105400.0


def _real_reader(crop_px: int = 40, *, base_cell_m: float = _CELL_M):
    """An ArrayBaseReader over a REAL LOLA Haworth base block, anchored at a global origin."""
    if not (os.path.exists(_HEIGHTMAP) and os.path.exists(_METADATA)):
        pytest.skip(f"real LOLA backbone absent: {_DEM_DIR}")
    with open(_METADATA) as fh:
        meta = json.load(fh)
    h = int(meta["grid"]["height"])
    w = int(meta["grid"]["width"])
    Z = np.fromfile(_HEIGHTMAP, dtype="<f4").reshape(h, w)
    affine = Affine(x0=_X0, y0=_Y0, px=_CELL_M)
    cx, cy = affine.xy(h // 2, w // 2)
    extent = crop_px * _CELL_M
    Z_crop, aff_crop = crop_square(Z, affine, (float(cx), float(cy)), extent)
    cs = dem_to_base(Z_crop, aff_crop, _CELL_M)
    fields = {n: getattr(cs, n) for n in BASE_FIELD_NAMES}
    return ArrayBaseReader(fields=fields, base_cell_m=base_cell_m,
                           world_x0=_X0, world_y0=_Y0)


@pytest.fixture(scope="module")
def reader():
    return _real_reader(40)


def _mosaic(reader, **kw):
    """A small fine-factor mosaic (k=4) so fine tiles stay tiny and fast."""
    params = dict(tile_base_cells=4, max_resident_tiles=6, world_seed=42)
    params.update(kw)
    return TileMosaic(reader, _CELL_M, 1.25, **params)  # k = 5/1.25 = 4


# --- additive metadata writer (L0 §2) -----------------------------------------------------

def test_write_dem_base_metadata_adds_keys_only():
    """The writer attaches the DEM/mosaic block additively, leaving pre-existing keys intact."""
    meta = {"schema_version": "1.0", "scene_name": "x", "grid": {"width": 8, "height": 8}}
    out = write_dem_base_metadata(
        meta, world_bounds_m={"x0": -52900.0, "y0": 105400.0, "x1": -52800.0, "y1": 105500.0},
        base_cell_m=5.0, fine_cell_m=0.02, region="Haworth")
    assert out is meta  # in-place + returned for chaining
    assert meta["grid"] == {"width": 8, "height": 8}  # untouched
    assert meta["base_cell_m"] == 5.0
    assert meta["fine_cell_m"] == 0.02
    assert meta["region"] == "Haworth"
    assert meta["world_bounds_m"]["x0"] == -52900.0
    # Default provenance is the validated Haworth/Product-78 block.
    assert "PGDA LOLA Product 78" in meta["dem_provenance"]["source"]
    # Still JSON-serializable (it is on-disk metadata).
    json.dumps(meta)


def test_write_dem_base_metadata_custom_provenance():
    meta = {"grid": {"width": 4, "height": 4}}
    prov = {"source": "S", "citation": "C", "frame": "F"}
    write_dem_base_metadata(
        meta, world_bounds_m={"x0": 0, "y0": 0, "x1": 1, "y1": 1},
        base_cell_m=5.0, fine_cell_m=1.0, region="R",
        local_datum_offset_m=12.5, dem_provenance=prov)
    assert meta["dem_provenance"] == prov
    assert meta["local_datum_offset_m"] == 12.5


# --- construction + geometry helpers ------------------------------------------------------

def test_mosaic_init_geometry(reader):
    m = _mosaic(reader)
    assert m.k == 4
    assert m.base_h == reader.height
    assert m.base_w == reader.width
    assert m.world_x0 == _X0 and m.world_y0 == _Y0
    # Tile grid covers the base (ceil division by tile_base_cells).
    import math
    assert m.n_tile_rows == math.ceil(reader.height / 4)
    assert m.n_tile_cols == math.ceil(reader.width / 4)
    assert m.resident_count == 0


def test_mosaic_rejects_bad_tile_size(reader):
    with pytest.raises(ValueError):
        TileMosaic(reader, _CELL_M, 1.25, tile_base_cells=0)


def test_base_window_returns_real_subrect(reader):
    m = _mosaic(reader)
    win = m.base_window((0, 0, 4, 4))
    for n in BASE_FIELD_NAMES:
        assert win[n].shape == (4, 4)
    # The window matches a direct read from the reader (it delegates to it).
    direct = reader.window((0, 0, 4, 4))
    assert np.array_equal(win["mass_areal"], direct["mass_areal"])


# --- demand-driven refine (L0 §0.1) -------------------------------------------------------

def test_ensure_fine_materializes_tiles(reader):
    """ensure_fine around a pose materializes resident fine tiles as refinement.Tile objects."""
    m = _mosaic(reader)
    tiles = m.ensure_fine((_X0 + 10.0, _Y0 + 10.0), radius_m=6.0)
    assert tiles, "no tiles materialized under the pose"
    assert all(isinstance(t, Tile) for t in tiles)
    assert m.resident_count > 0
    # Ascending ids in scan order.
    assert [t.id for t in tiles] == list(range(len(tiles)))
    # A fine tile is k x base_cells per side at the fine cell size.
    t0 = tiles[0]
    assert t0.cell_m == pytest.approx(1.25)
    assert t0.cs.mass_areal.shape[0] >= m.k  # at least one base cell refined


def test_ensure_fine_generation_count_and_reuse(reader):
    """First visit generates; a re-visit of a resident tile reuses (no new generation)."""
    m = _mosaic(reader)
    m.ensure_fine((_X0 + 10.0, _Y0 + 10.0), radius_m=4.0)
    g1 = m.generation_count
    assert g1 > 0
    # Same pose again: resident tiles are reused, generation count does not climb.
    m.ensure_fine((_X0 + 10.0, _Y0 + 10.0), radius_m=4.0)
    assert m.generation_count == g1


# --- coordinate-hashed determinism (L0 §0.2) ----------------------------------------------

def test_regenerated_tile_byte_identical(reader):
    """A tile generated twice (eviction/re-entry) is byte-identical (global-coord determinism)."""
    m = _mosaic(reader)
    t1 = m._generate_fine_tile(1, 1)
    t2 = m._generate_fine_tile(1, 1)
    assert isinstance(t1, FineTile)
    for n in dem_io.BASE_FIELD_NAMES:
        assert np.array_equal(t1.fields[n], t2.fields[n]), f"{n} not byte-identical on regen"
    # FineTile.as_tile adapts to a refinement.Tile carrying the same region.
    tile = t1.as_tile(0)
    assert isinstance(tile, Tile)
    assert tile.region_rc == t1.region_rc


# --- bounded / evictable resident set (L0 §0.3) -------------------------------------------

def test_resident_set_bounded_under_roaming(reader):
    """Driving across the base keeps the resident set within the LRU budget."""
    m = _mosaic(reader, max_resident_tiles=6)
    peak = 0
    span = (reader.width - 1) * _CELL_M
    for i in range(12):
        x = _X0 + (i / 11.0) * span
        y = _Y0 + (i / 11.0) * span
        tiles = m.ensure_fine((x, y), radius_m=3.0)
        assert tiles  # materialized at every step
        peak = max(peak, m.resident_count)
    assert peak <= 6
    assert m.resident_count <= 6


def test_evict_drops_tiles_outside_keep_box(reader):
    """evict pages out resident tiles whose base region is disjoint from the keep box."""
    m = _mosaic(reader, max_resident_tiles=64)  # large budget so nothing auto-evicts
    # Materialize a spread of tiles across the base.
    span = (reader.width - 1) * _CELL_M
    for i in range(6):
        x = _X0 + (i / 5.0) * span
        m.ensure_fine((x, _Y0 + 5.0), radius_m=3.0)
    before = m.resident_count
    assert before > 1
    # Keep only the bottom-left 4x4 base block; tiles fully outside it are evicted.
    m.evict((0, 0, 4, 4))
    after = m.resident_count
    assert after < before
    # Every surviving tile's base region overlaps the keep box.
    for key in m.resident_keys:
        r0, c0, r1, c1 = m._resident[key].region_rc
        assert not (r1 <= 0 or r0 >= 4 or c1 <= 0 or c0 >= 4)


def test_load_paged_none_without_page_dir(reader):
    """Without a page_dir, _load_paged short-circuits to None (drop-on-evict mode)."""
    m = _mosaic(reader)  # no page_dir
    assert m._load_paged(0, 0) is None


def test_touch_refreshes_existing_to_mru(reader):
    """Re-touching a resident tile moves it to MRU (LRU bookkeeping), keeping the count stable."""
    m = _mosaic(reader, max_resident_tiles=64)
    m.ensure_fine((_X0 + 8.0, _Y0 + 8.0), radius_m=2.0)
    keys_before = m.resident_keys
    assert keys_before
    count_before = m.resident_count
    first = keys_before[0]
    # Touch the first (LRU-front) tile again; it moves to the end, count unchanged.
    m._touch(first[0], first[1], m._resident[first])
    assert m.resident_count == count_before
    assert m.resident_keys[-1] == first


def test_resident_memory_cells_tracks_resident(reader):
    m = _mosaic(reader)
    assert m.resident_memory_cells() == 0
    m.ensure_fine((_X0 + 8.0, _Y0 + 8.0), radius_m=4.0)
    cells = m.resident_memory_cells()
    assert cells > 0
    # Equals the sum of the resident fine mass_areal sizes.
    expect = sum(t.fields["mass_areal"].size for t in m._resident.values())
    assert cells == expect


# --- paging round-trip (page_dir) ---------------------------------------------------------

def test_paging_roundtrip_loads_to_float32_precision(tmp_path, reader):
    """With page_dir set, an evicted tile is paged to disk and re-loads at float32 precision.

    Paging persists through the frozen ``.rf32``/``.r8`` byte format (float32 on disk), so a
    re-loaded float field matches the float64 in-RAM regeneration to the float32 round-off
    (state_label is uint8 -> exact). This is the real precision contract of the paging path:
    regeneration is the bit-identical source (the determinism guarantee, tested separately);
    paging is a load-vs-regenerate perf choice that costs one float32 cast.
    """
    page = str(tmp_path / "pages")
    m = _mosaic(reader, max_resident_tiles=2, page_dir=page)
    # Page tile (0,0) to disk through the real write path, then load it back.
    target = m._generate_fine_tile(0, 0)
    m._page_out((0, 0), target)
    loaded = m._load_paged(0, 0)
    assert loaded is not None
    # state_label is uint8 -> exact through the .r8 format.
    assert np.array_equal(loaded.fields["state_label"], target.fields["state_label"])
    # Float fields match to float32 precision (the disk format), recovering the same values.
    for n in ("mass_areal", "density", "datum", "disturbance"):
        assert np.allclose(loaded.fields[n], target.fields[n].astype(np.float32),
                           rtol=0, atol=0), n
        assert loaded.fields[n].shape == target.fields[n].shape
