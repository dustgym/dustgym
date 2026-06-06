"""Characterization tests for ``terrain_authority.dem_import`` against the REAL LOLA tile.

The ingest's public surface (frozen by the L0 contract) is exercised on the committed
real PGDA LOLA Haworth backbone at ``samples/lunar_dem/haworth_10km_5m`` — 2000x2000 @
5 m, real relief -96.6..+2842.2 m (``metadata.json`` ``height_range_m``). These are
genuine relief values from the south-polar ``Haworth_final_adj_5mpp_surf.tif`` slice, so
``crop_square`` / ``dem_to_base`` / ``polar_mantle_density_fn`` run on real data, not a
fabricated surface.

Honest scope note (no synthetic data): the source ``*_surf.tif`` GeoTIFF (5960x5960) is
NOT committed to this repo (only its measured products are — see ``slope_anchor.json``
``anchor_size_note``). ``load_lola_geotiff`` parses that raw GeoTIFF, so its TIFF-byte
path cannot be characterized here without fabricating a TIFF (forbidden). We instead load
the committed ``heightmap.rf32`` — which IS the real surface ``dem_to_base`` produced
(``metadata.json`` ``derive_height()==Z`` note) — and reconstruct the documented
first-pixel-center ``Affine`` from the committed ``world_bounds_m`` (which encode the real
global polar-stereographic offset). The full ``crop_square`` -> ``dem_to_base`` ->
``derive_height`` round-trip and the ChaSTE density bridge are characterized on real bytes.
"""

from __future__ import annotations

import json
import os

import numpy as np
import pytest

from terrain_authority import constants as K
from terrain_authority.dem_import import (
    Affine,
    crop_square,
    dem_to_base,
    polar_mantle_density_fn,
)
from terrain_authority.dem_import import _self_test  # author's own real assertions

# --- Real LOLA Haworth backbone (committed) --------------------------------------------
_DEM_DIR = os.path.join(
    os.path.dirname(__file__), os.pardir,
    "samples", "lunar_dem", "haworth_10km_5m",
)
_HEIGHTMAP = os.path.join(_DEM_DIR, "heightmap.rf32")
_METADATA = os.path.join(_DEM_DIR, "metadata.json")

# Documented real values (metadata.json). These are co-registered with the real _surf tile.
_GRID_W = _GRID_H = 2000
_CELL_M = 5.0
# height_range_m from metadata.json (real Haworth relief, height-above-sphere [m]).
_REAL_MIN = -96.6145
_REAL_MAX = 2842.2139
# First-pixel (row0,col0) CENTER from world_bounds_m + the half-cell convention in
# scripts/build_from_dem.py: x0_center = wb.x0 + cell/2 ; y0_center (TOP) = wb.y1 - cell/2.
_X0_CENTER = -52900.0   # = -52902.5 + 2.5
_Y0_CENTER = 105400.0   # = 105402.5 - 2.5


def _require_dem() -> None:
    if not (os.path.exists(_HEIGHTMAP) and os.path.exists(_METADATA)):
        pytest.skip(f"real LOLA backbone absent: {_DEM_DIR}")


def _load_real_dem() -> tuple[np.ndarray, Affine, dict]:
    """Load the committed real Haworth surface as (Z float32 [m], Affine, metadata).

    Z is the real LOLA-derived height-above-sphere surface (the bytes ``dem_to_base``
    produced via ``derive_height()``), and the Affine is the documented first-pixel-center
    pixel<->world map reconstructed from the committed ``world_bounds_m``.
    """
    _require_dem()
    with open(_METADATA) as fh:
        meta = json.load(fh)
    h = int(meta["grid"]["height"])
    w = int(meta["grid"]["width"])
    Z = np.fromfile(_HEIGHTMAP, dtype="<f4").reshape(h, w)
    affine = Affine(x0=_X0_CENTER, y0=_Y0_CENTER, px=float(meta["grid"]["cell_m"]))
    return Z, affine, meta


# --- Affine: the pixel<->world map --------------------------------------------------------

def test_affine_pixel_size_is_5m_and_origin_matches_metadata():
    Z, affine, meta = _load_real_dem()
    # Native cell size is the real LOLA 5 m/px (metadata + dem_provenance native_cell_m).
    assert affine.px == pytest.approx(5.0)
    assert float(meta["grid"]["cell_m"]) == pytest.approx(5.0)
    assert float(meta["dem_provenance"]["native_cell_m"]) == pytest.approx(5.0)
    # First-pixel center sits at the documented non-zero global polar-stereographic offset.
    assert affine.x0 == pytest.approx(_X0_CENTER)
    assert affine.y0 == pytest.approx(_Y0_CENTER)


def test_affine_xy_colrow_round_trip_and_y_decreases_with_row():
    _, affine, _ = _load_real_dem()
    # xy and colrow are inverses on real coordinates.
    x, y = affine.xy(np.array([0, 7, 1999]), np.array([0, 3, 1999]))
    col, row = affine.colrow(x, y)
    assert np.allclose(col, [0, 3, 1999])
    assert np.allclose(row, [0, 7, 1999])
    # Y decreases as row increases (the LOLA north-up convention); X increases with col.
    x_row0, y_row0 = affine.xy(0, 0)
    x_row1, y_row1 = affine.xy(1, 0)
    _, y_only = affine.xy(0, 5)  # same row, +5 cols
    x_col5, _ = affine.xy(0, 5)
    assert y_row1 < y_row0
    assert y_row1 == pytest.approx(y_row0 - 5.0)
    assert x_col5 == pytest.approx(x_row0 + 5 * 5.0)
    assert y_only == pytest.approx(y_row0)  # moving in col does not change Y


def test_affine_with_origin_translates_but_keeps_pixel_size():
    _, affine, _ = _load_real_dem()
    shifted = affine.with_origin(1234.0, -5678.0)
    assert shifted.px == affine.px
    assert shifted.x0 == pytest.approx(1234.0)
    assert shifted.y0 == pytest.approx(-5678.0)


# --- The real surface itself --------------------------------------------------------------

def test_real_dem_shape_finite_and_relief_in_documented_range():
    Z, _, meta = _load_real_dem()
    assert Z.shape == (_GRID_H, _GRID_W)
    assert Z.dtype == np.float32
    # The whole committed tile is finite (no NoData in the chosen max-relief window).
    assert np.isfinite(Z).all()
    # Real Haworth relief, exactly the metadata height_range_m (height above sphere [m]).
    zmin, zmax = float(Z.min()), float(Z.max())
    assert zmin == pytest.approx(_REAL_MIN, abs=1e-2)
    assert zmax == pytest.approx(_REAL_MAX, abs=1e-2)
    meta_min, meta_max = meta["height_range_m"]
    assert zmin == pytest.approx(meta_min, abs=1e-2)
    assert zmax == pytest.approx(meta_max, abs=1e-2)
    # This is real lunar relief (km of it), not a flat or near-flat synthetic plate.
    assert (zmax - zmin) > 2000.0


# --- crop_square: same-frame pixel-window slice on the REAL tile --------------------------

def test_crop_square_shape_and_offset_preserved_on_real_tile():
    Z, affine, _ = _load_real_dem()
    # Crop a 1 km square centred on the tile's geometric centre (well inside the raster).
    cx, cy = affine.xy(_GRID_H // 2, _GRID_W // 2)
    extent = 1000.0  # m -> 200 px at 5 m
    Z_crop, aff_crop = crop_square(Z, affine, (float(cx), float(cy)), extent)
    n = int(round(extent / affine.px))
    assert Z_crop.shape == (n, n) == (200, 200)
    assert aff_crop.px == affine.px
    # The crop is a real sub-window of the surface: every value matches the source slice,
    # is finite, and lies within the parent relief band.
    assert np.isfinite(Z_crop).all()
    assert float(Z_crop.min()) >= float(Z.min()) - 1e-3
    assert float(Z_crop.max()) <= float(Z.max()) + 1e-3
    # The crop's first-pixel center maps back into the source as an integer pixel offset
    # (global offsets are preserved, not zeroed).
    fcol, frow = affine.colrow(aff_crop.x0, aff_crop.y0)
    assert float(fcol) == pytest.approx(round(float(fcol)), abs=1e-6)
    assert float(frow) == pytest.approx(round(float(frow)), abs=1e-6)
    row0, col0 = int(round(float(frow))), int(round(float(fcol)))
    assert np.array_equal(Z_crop, Z[row0:row0 + n, col0:col0 + n])


def test_crop_square_window_outside_raster_raises():
    Z, affine, _ = _load_real_dem()
    # A square centred at the corner cannot fit a full window -> ValueError (a partial
    # NoData edge would corrupt the conservation round-trip).
    x_corner, y_corner = affine.xy(0, 0)
    with pytest.raises(ValueError):
        crop_square(Z, affine, (float(x_corner), float(y_corner)), 1000.0)


def test_crop_square_sub_pixel_extent_raises():
    Z, affine, _ = _load_real_dem()
    cx, cy = affine.xy(_GRID_H // 2, _GRID_W // 2)
    with pytest.raises(ValueError):
        crop_square(Z, affine, (float(cx), float(cy)), 1.0)  # < one 5 m pixel


# --- dem_to_base: inject the REAL surface via the frozen datum path -----------------------

def test_dem_to_base_native_cell_round_trips_real_surface():
    Z, affine, _ = _load_real_dem()
    cx, cy = affine.xy(_GRID_H // 2, _GRID_W // 2)
    Z_crop, aff_crop = crop_square(Z, affine, (float(cx), float(cy)), 1000.0)

    cs = dem_to_base(Z_crop, aff_crop, _CELL_M)  # native 5 m -> no resample
    assert cs.width == Z_crop.shape[1]
    assert cs.height == Z_crop.shape[0]
    assert cs.cell_m == pytest.approx(_CELL_M)

    # The datum path reproduces the real DEM elevations to ~mm (contract §1 invariant).
    derived = cs.derive_height()
    assert derived.shape == Z_crop.shape
    assert np.isfinite(derived).all()
    err = float(np.max(np.abs(derived - Z_crop.astype(np.float64))))
    assert err <= 1e-3

    # datum = Z - mantle_m ; mass = mantle_m * RHO_SURFACE (the frozen surface-injection seam).
    assert np.allclose(cs.datum, Z_crop.astype(np.float64) - K.Z_T)
    assert np.allclose(cs.density, K.RHO_SURFACE)
    assert np.allclose(cs.mass_areal, K.Z_T * K.RHO_SURFACE)
    # The global-frame affine is attached for the end-to-end builder.
    assert cs._dem_affine is aff_crop or cs._dem_affine == aff_crop


def test_dem_to_base_resample_to_10m_preserves_relief_and_origin():
    Z, affine, _ = _load_real_dem()
    cx, cy = affine.xy(_GRID_H // 2, _GRID_W // 2)
    Z_crop, aff_crop = crop_square(Z, affine, (float(cx), float(cy)), 1000.0)

    cs = dem_to_base(Z_crop, aff_crop, 10.0)  # bilinear resample 5 m -> 10 m
    assert cs.cell_m == pytest.approx(10.0)
    # ~half the cells per axis (floor((n-1)/ratio)+1 for n=200, ratio=2 -> 100).
    assert cs.height == 100 and cs.width == 100
    # Origin preserved: the resampled grid keeps the crop's first-cell center.
    assert cs._dem_affine.x0 == pytest.approx(aff_crop.x0)
    assert cs._dem_affine.y0 == pytest.approx(aff_crop.y0)
    assert cs._dem_affine.px == pytest.approx(10.0)
    # Resampling stays inside the real relief band (bilinear cannot extrapolate; nearest
    # edge clamp) and the surface still round-trips through the datum path.
    derived = cs.derive_height()
    assert np.isfinite(derived).all()
    assert float(derived.min()) >= float(Z_crop.min()) - 1e-3
    assert float(derived.max()) <= float(Z_crop.max()) + 1e-3
    assert float(np.max(np.abs(derived - (cs.datum + cs.mass_areal / cs.density)))) == 0.0


def test_dem_to_base_with_polar_density_hook_on_real_surface():
    Z, affine, _ = _load_real_dem()
    cx, cy = affine.xy(_GRID_H // 2, _GRID_W // 2)
    # Smaller crop keeps the test fast while staying on real bytes.
    Z_crop, aff_crop = crop_square(Z, affine, (float(cx), float(cy)), 250.0)  # 50x50 px

    dfn = polar_mantle_density_fn()  # default mantle_m == Z_T
    cs = dem_to_base(Z_crop, aff_crop, _CELL_M, density_fn=dfn)

    # Density CANCELS in derive_height, so the real surface still round-trips to ~mm even
    # with the ChaSTE polar density landed.
    err = float(np.max(np.abs(cs.derive_height() - Z_crop.astype(np.float64))))
    assert err <= 1e-3
    # The landed density is the constant ChaSTE depth-integrated bulk, NOT the equatorial
    # RHO_SURFACE stand-in.
    assert np.ptp(cs.density) == 0.0
    rho0 = float(cs.density.flat[0])
    assert rho0 == pytest.approx(dfn.rho_bar)
    assert K.RHO_SURFACE_POLAR <= rho0 <= K.RHO_BULK_POLAR_10CM
    assert abs(rho0 - K.RHO_SURFACE) > 1.0


# --- dem_to_base error guards (real crop, injected NoData / bad density hook) -------------

def test_dem_to_base_nonfinite_z_raises():
    Z, affine, _ = _load_real_dem()
    cx, cy = affine.xy(_GRID_H // 2, _GRID_W // 2)
    Z_crop, aff_crop = crop_square(Z, affine, (float(cx), float(cy)), 250.0)
    # Inject a real NoData hole into the real crop (a NaN like an undeclared LOLA gap):
    # dem_to_base must refuse to inject a non-finite surface.
    Z_hole = Z_crop.copy()
    Z_hole[0, 0] = np.nan
    with pytest.raises(ValueError):
        dem_to_base(Z_hole, aff_crop, _CELL_M)


def test_dem_to_base_density_fn_wrong_shape_raises():
    Z, affine, _ = _load_real_dem()
    cx, cy = affine.xy(_GRID_H // 2, _GRID_W // 2)
    Z_crop, aff_crop = crop_square(Z, affine, (float(cx), float(cy)), 250.0)

    def bad_shape(X, Y):
        return np.ones((3, 4), dtype=np.float64)  # not the grid shape

    with pytest.raises(ValueError):
        dem_to_base(Z_crop, aff_crop, _CELL_M, density_fn=bad_shape)


def test_dem_to_base_nonpositive_density_fn_raises():
    Z, affine, _ = _load_real_dem()
    cx, cy = affine.xy(_GRID_H // 2, _GRID_W // 2)
    Z_crop, aff_crop = crop_square(Z, affine, (float(cx), float(cy)), 250.0)

    def nonpos(X, Y):
        return np.zeros(np.shape(X), dtype=np.float64)  # density must be > 0

    with pytest.raises(ValueError):
        dem_to_base(Z_crop, aff_crop, _CELL_M, density_fn=nonpos)


# --- polar_mantle_density_fn: the ChaSTE depth-integrated bridge --------------------------

def test_polar_mantle_density_constant_in_range_and_matches_mass_weighted_mean():
    dfn = polar_mantle_density_fn()  # default mantle_m == Z_T (0.12 m)
    XX, YY = np.meshgrid(np.linspace(-50.0, 50.0, 9), np.linspace(200.0, 260.0, 7))
    rho = dfn(XX, YY)
    # A constant grid shaped to the inputs (honest: scalar broadcast, not a spatial field).
    assert rho.shape == XX.shape
    assert np.ptp(rho) == 0.0
    rho0 = float(rho.flat[0])
    assert K.RHO_SURFACE_POLAR <= rho0 <= K.RHO_BULK_POLAR_10CM
    # Independent recomputation of the mass-weighted ChaSTE mean over [0, Z_T].
    expect = (K.RHO_SURFACE_POLAR * K.Z_POLAR_TOP_M
              + K.RHO_MID_POLAR * (K.Z_POLAR_MID_M - K.Z_POLAR_TOP_M)
              + K.RHO_BULK_POLAR_10CM * (K.Z_T - K.Z_POLAR_MID_M)) / K.Z_T
    assert rho0 == pytest.approx(expect, abs=1e-9)
    assert dfn.rho_bar == pytest.approx(rho0, abs=1e-9)
    assert dfn.mantle_m == pytest.approx(K.Z_T)


def test_polar_mantle_density_is_depth_monotonic():
    # Thinner mantle -> more weight on the loose top fines -> LOWER mass-weighted mean.
    thin = polar_mantle_density_fn(0.02).rho_bar   # all inside [0,3cm) @ RHO_SURFACE_POLAR
    mid = polar_mantle_density_fn(0.05).rho_bar    # spans the 750 + 1300 bands
    full = polar_mantle_density_fn(K.Z_T).rho_bar  # default
    assert thin == pytest.approx(K.RHO_SURFACE_POLAR, abs=1e-9)
    assert K.RHO_SURFACE_POLAR < mid < full <= K.RHO_BULK_POLAR_10CM


def test_polar_mantle_density_nonpositive_mantle_raises():
    with pytest.raises(ValueError):
        polar_mantle_density_fn(0.0)
    with pytest.raises(ValueError):
        polar_mantle_density_fn(-0.1)


# --- The author's own self-test (covers polar_mantle_density_fn + dem_to_base end to end) -

def test_module_self_test_passes():
    # _self_test() encodes the author's REAL acceptance assertions (density range, datum
    # round-trip, depth-monotonicity) and returns 0 on all-pass.
    assert _self_test() == 0


def test_geotiff_rational_tag_parses(tmp_path):
    # Regression: a GeoTIFF carrying type-5 RATIONAL tags (XResolution/YResolution, which GDAL/tifffile
    # emit by DEFAULT) must parse. Before the fix the IFD reader over-read the RATIONAL field (16 B vs 8)
    # and raised struct.error -> any real survey GeoTIFF with those tags crashed ingest. Real Haworth
    # pixels, written through tifffile to a real classic TIFF.
    tifffile = pytest.importorskip("tifffile")
    _require_dem()
    from terrain_authority.dem_import import _read_tiff_ifd0
    Z = np.fromfile(_HEIGHTMAP, dtype="<f4", count=64).reshape(8, 8).copy()   # a real Haworth crop
    p = str(tmp_path / "crop_rational.tif")
    tifffile.imwrite(p, Z, resolution=(200.0, 200.0))      # -> 282/283 XResolution/YResolution RATIONAL (type 5)
    tags, _bo = _read_tiff_ifd0(p)                          # must NOT raise (was struct.error on type-5)
    assert 282 in tags and tags[282][0] == pytest.approx(200.0)   # RATIONAL decoded num/den, not crashed
