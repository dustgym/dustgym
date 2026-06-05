"""Characterization tests for ``terrain_authority.dem_io`` (windowed/memmap base reader).

The base field dict driving these tests is built from the REAL LOLA Haworth backbone:
``dem_import.dem_to_base`` injects a crop of the committed real surface
(``samples/lunar_dem/haworth_10km_5m/heightmap.rf32``, real relief -96.6..+2842.2 m) into
a ``ColumnState``, whose five base fields (mass_areal / density / datum / state_label /
disturbance) become the reader's source. No synthetic field is fabricated; every value is
the conserved authority's real output over real lunar relief.

The tests characterize the streaming layer's invariants: windowing returns the exact
sub-rectangle as COPIES; ``ArrayBaseReader`` (in-RAM slice) and ``MemmapBaseReader`` (on
disk via numpy.memmap through ``write_base_rasters``) agree byte-for-byte; bbox clipping
follows the half-open contract; and the global metre origin of a window is placed by the
row-major (x grows with col, y grows with row) convention.
"""

from __future__ import annotations

import json
import os

import numpy as np
import pytest

from terrain_authority.dem_import import Affine, crop_square, dem_to_base
from terrain_authority.dem_io import (
    BASE_FIELD_NAMES,
    ArrayBaseReader,
    MemmapBaseReader,
    _clip_bbox,
    write_base_rasters,
)

_DEM_DIR = os.path.join(
    os.path.dirname(__file__), os.pardir,
    "samples", "lunar_dem", "haworth_10km_5m",
)
_HEIGHTMAP = os.path.join(_DEM_DIR, "heightmap.rf32")
_METADATA = os.path.join(_DEM_DIR, "metadata.json")
_CELL_M = 5.0
_X0_CENTER = -52900.0
_Y0_CENTER = 105400.0


def _real_base_fields(crop_px: int = 100) -> tuple[dict[str, np.ndarray], float]:
    """A REAL base field dict from the committed LOLA surface via ``dem_to_base``.

    Returns ``(fields, base_cell_m)`` where ``fields`` holds all five BASE_FIELD_NAMES
    (mass_areal/density/datum/disturbance float64, state_label uint8) for a ``crop_px`` x
    ``crop_px`` square of the real Haworth tile centred on the raster middle.
    """
    if not (os.path.exists(_HEIGHTMAP) and os.path.exists(_METADATA)):
        pytest.skip(f"real LOLA backbone absent: {_DEM_DIR}")
    with open(_METADATA) as fh:
        meta = json.load(fh)
    h = int(meta["grid"]["height"])
    w = int(meta["grid"]["width"])
    Z = np.fromfile(_HEIGHTMAP, dtype="<f4").reshape(h, w)
    affine = Affine(x0=_X0_CENTER, y0=_Y0_CENTER, px=_CELL_M)
    cx, cy = affine.xy(h // 2, w // 2)
    extent = crop_px * _CELL_M
    Z_crop, aff_crop = crop_square(Z, affine, (float(cx), float(cy)), extent)
    cs = dem_to_base(Z_crop, aff_crop, _CELL_M)
    fields = {n: getattr(cs, n) for n in BASE_FIELD_NAMES}
    return fields, cs.cell_m


# --- _clip_bbox: the half-open clipping contract ------------------------------------------

def test_clip_bbox_passthrough_and_clamp():
    assert _clip_bbox((2, 3, 5, 7), 10, 10) == (2, 3, 5, 7)
    # Out-of-range upper bounds clamp to the grid extent.
    assert _clip_bbox((-5, -5, 100, 100), 10, 12) == (0, 0, 10, 12)


def test_clip_bbox_empty_after_clip_raises():
    with pytest.raises(ValueError):
        _clip_bbox((5, 5, 5, 9), 10, 10)   # r1 == r0 -> empty
    with pytest.raises(ValueError):
        _clip_bbox((5, 5, 9, 5), 10, 10)   # c1 == c0 -> empty
    with pytest.raises(ValueError):
        _clip_bbox((20, 0, 30, 5), 10, 10)  # entirely below the grid -> empty after clamp


# --- ArrayBaseReader: in-RAM windowing over the real base ---------------------------------

def test_array_reader_post_init_dims_and_missing_field():
    fields, _ = _real_base_fields(64)
    rdr = ArrayBaseReader(fields=fields, base_cell_m=_CELL_M)
    assert (rdr.height, rdr.width) == (64, 64)
    # A base dict missing a required field is rejected at construction.
    broken = {k: v for k, v in fields.items() if k != "datum"}
    with pytest.raises(ValueError):
        ArrayBaseReader(fields=broken, base_cell_m=_CELL_M)


def test_array_reader_window_is_exact_subrectangle_and_a_copy():
    fields, _ = _real_base_fields(80)
    rdr = ArrayBaseReader(fields=fields, base_cell_m=_CELL_M)
    bbox = (10, 20, 40, 55)  # half-open
    win = rdr.window(bbox)
    r0, c0, r1, c1 = bbox
    for n in BASE_FIELD_NAMES:
        sub = win[n]
        assert sub.shape == (r1 - r0, c1 - c0)
        # Values equal the exact real sub-rectangle of the source field.
        assert np.array_equal(sub, fields[n][r0:r1, c0:c1])
        # dtype contract: state_label uint8, everything else float64.
        assert sub.dtype == (np.uint8 if n == "state_label" else np.float64)
        assert np.isfinite(sub).all()
    # It is a COPY: mutating the window does not alias the base.
    win["mass_areal"][0, 0] += 12345.0
    assert win["mass_areal"][0, 0] != fields["mass_areal"][10, 20]


def test_array_reader_window_clips_oversize_bbox():
    fields, _ = _real_base_fields(50)
    rdr = ArrayBaseReader(fields=fields, base_cell_m=_CELL_M)
    win = rdr.window((40, 40, 999, 999))  # clamps to (40,40,50,50) -> 10x10
    assert win["datum"].shape == (10, 10)
    assert np.array_equal(win["datum"], fields["datum"][40:50, 40:50])


def test_array_reader_window_origin_m_uses_row_col_offsets():
    fields, _ = _real_base_fields(40)
    rdr = ArrayBaseReader(fields=fields, base_cell_m=_CELL_M,
                          world_x0=1000.0, world_y0=-2000.0)
    x, y = rdr.window_origin_m((4, 6, 20, 25))
    # x grows with COLUMN (c0), y grows with ROW (r0).
    assert x == pytest.approx(1000.0 + 6 * _CELL_M)
    assert y == pytest.approx(-2000.0 + 4 * _CELL_M)


# --- write_base_rasters + MemmapBaseReader: the on-disk streaming path ---------------------

def test_write_base_rasters_missing_field_raises(tmp_path):
    fields, _ = _real_base_fields(16)
    bad = {k: v for k, v in fields.items() if k != "disturbance"}
    with pytest.raises(ValueError):
        write_base_rasters(str(tmp_path / "base"), bad)


def test_memmap_reader_round_trips_real_base_and_matches_array_reader(tmp_path):
    fields, cell = _real_base_fields(96)
    out_dir = str(tmp_path / "base")
    write_base_rasters(out_dir, fields)

    # Every expected per-field raster was written with the frozen extension convention.
    for n in BASE_FIELD_NAMES:
        ext = "r8" if n == "state_label" else "rf32"
        assert os.path.exists(os.path.join(out_dir, f"{n}.{ext}"))

    h, w = fields["mass_areal"].shape
    mm = MemmapBaseReader(dir_=out_dir, height=h, width=w, base_cell_m=cell)
    arr = ArrayBaseReader(fields=fields, base_cell_m=cell)

    bbox = (12, 8, 60, 70)
    win_mm = mm.window(bbox)
    win_arr = arr.window(bbox)
    r0, c0, r1, c1 = bbox
    for n in BASE_FIELD_NAMES:
        # Memmap window == array window == real source slice (byte-faithful round-trip).
        # The on-disk dtype is '<f4'/'u1' so float64 fields compare as float32 round-trips.
        assert win_mm[n].shape == (r1 - r0, c1 - c0)
        assert win_mm[n].dtype == (np.uint8 if n == "state_label" else np.float64)
        expect = fields[n][r0:r1, c0:c1]
        if n == "state_label":
            assert np.array_equal(win_mm[n], expect)
            assert np.array_equal(win_mm[n], win_arr[n])
        else:
            assert np.allclose(win_mm[n], expect.astype("<f4"), rtol=0, atol=0)
            # datum carries km-scale real relief: float32 storage is exact-enough; the
            # two readers agree to float32 precision.
            assert np.allclose(win_mm[n], win_arr[n].astype("<f4"))
        assert np.isfinite(win_mm[n]).all()


def test_memmap_reader_missing_file_raises(tmp_path):
    fields, _ = _real_base_fields(16)
    out_dir = str(tmp_path / "base")
    write_base_rasters(out_dir, fields)
    os.remove(os.path.join(out_dir, "density.rf32"))  # drop one field's raster
    mm = MemmapBaseReader(dir_=out_dir, height=16, width=16, base_cell_m=_CELL_M)
    with pytest.raises(FileNotFoundError):
        mm.window((0, 0, 8, 8))


def test_memmap_window_origin_m_matches_array_reader(tmp_path):
    fields, _ = _real_base_fields(16)
    out_dir = str(tmp_path / "base")
    write_base_rasters(out_dir, fields)
    mm = MemmapBaseReader(dir_=out_dir, height=16, width=16, base_cell_m=_CELL_M,
                          world_x0=500.0, world_y0=750.0)
    x, y = mm.window_origin_m((3, 5, 10, 12))
    assert x == pytest.approx(500.0 + 5 * _CELL_M)
    assert y == pytest.approx(750.0 + 3 * _CELL_M)
