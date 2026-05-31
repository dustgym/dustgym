"""Real-DEM ingest — pure PIL + numpy + scipy (NO GDAL / rasterio / pip).

Lane A of the real-DEM 10 km thrust (docs/dem_terrain_contract.md §1,
docs/lunar_dem_10km_eval.md §4-5). Reads a PGDA LOLA south-polar 5 m/px
``*_surf.tif`` and lands its surface into the mass-conserving ``ColumnState`` via
the FROZEN datum path, so ``derive_height()`` reproduces the DEM to ~mm.

Why no GDAL: the validated Haworth tile (``Haworth_final_adj_5mpp_surf.tif``,
5960x5960, mode 'F' float32, uncompressed classic TIFF) is PIL-readable, and its
GeoTIFF tags (33550 ModelPixelScale, 33922 ModelTiepoint, 34735/34736 GeoKeys)
parse directly from the IFD. A same-frame 10 km crop is a pixel-window slice — the
product is ALREADY south-polar stereographic (IAU_2015:30135), so NO reprojection
is required (eval addendum §4.3). This environment has no gdal/rasterio/osgeo/pyproj
and may not pip install.

Vertical datum (load-bearing, eval addendum §4.2): the LOLA ``*_surf`` Z is a
HEIGHT-ABOVE-SPHERE in metres (Haworth range ~-1643..+2842 m), NOT an absolute
radius — so ``derive_height`` consumes Z DIRECTLY with NO ``Z - 1737400``
subtraction. We do not mutate Z; the float32 over a few km of relief resolves
sub-mm, so the per-tile local datum offset (metadata hygiene) is a downstream
convenience, not a precision necessity for this data.

The affine (pixel-registered / GMT "gridline" — (0,0) is the FIRST-PIXEL CENTER):

    X(col) = X0 + col * px        Y(row) = Y0 - row * px

with (X0, Y0) the ModelTiepoint mapping raster (0,0). Y decreases down rows.

Public surface (frozen by the L0 contract — signatures are NOT to be restructured):
    load_lola_geotiff(path)                  -> (Z float32 [m above sphere], Affine, meta)
    crop_square(Z, affine, center_xy_m, extent_m) -> (Z_crop, Affine)
    dem_to_base(Z_crop, affine, base_cell_m, *, mantle_m, density) -> ColumnState
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

import numpy as np

from . import constants as K
from .column_state import ColumnState

__all__ = ["Affine", "load_lola_geotiff", "crop_square", "dem_to_base"]


# ---------------------------------------------------------------------------
# Affine — the same-frame pixel<->world map (no rotation; polar-stereographic m).
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Affine:
    """Pixel-registered affine for a north-up, axis-aligned raster.

    ``X(col) = x0 + col*px`` ; ``Y(row) = y0 - row*px`` (Y decreases with row).
    ``(x0, y0)`` is the world coordinate of the FIRST-PIXEL CENTER (raster (0,0)),
    i.e. the GeoTIFF ModelTiepoint for a pixel-registered (GMT gridline) product.
    ``px`` is the cell size in metres (ModelPixelScale; assumed square for LOLA).
    """

    x0: float          # world X [m] of pixel (row=0, col=0) CENTER
    y0: float          # world Y [m] of pixel (row=0, col=0) CENTER
    px: float          # pixel size [m] (square)

    def xy(self, row, col):
        """World (X, Y) [m] of a pixel CENTER at (row, col)."""
        return self.x0 + np.asarray(col) * self.px, self.y0 - np.asarray(row) * self.px

    def colrow(self, x, y):
        """Fractional (col, row) of world (x, y) [m] (inverse of ``xy``)."""
        return (np.asarray(x) - self.x0) / self.px, (self.y0 - np.asarray(y)) / self.px

    def with_origin(self, x0: float, y0: float) -> "Affine":
        """A copy translated so its first-pixel CENTER is (x0, y0); px unchanged."""
        return Affine(x0=float(x0), y0=float(y0), px=self.px)


# ---------------------------------------------------------------------------
# 1. load_lola_geotiff — PIL read + hand-parsed GeoTIFF tags (no GDAL).
# ---------------------------------------------------------------------------

# GeoTIFF tag IDs (OGC GeoTIFF spec).
_TAG_MODEL_PIXEL_SCALE = 33550   # ModelPixelScaleTag  (double x3: sx, sy, sz)
_TAG_MODEL_TIEPOINT = 33922      # ModelTiepointTag    (double x6: i,j,k, X,Y,Z)
_TAG_GEO_KEY_DIRECTORY = 34735   # GeoKeyDirectoryTag  (short)
_TAG_GEO_DOUBLE_PARAMS = 34736   # GeoDoubleParamsTag  (double)
_TAG_GDAL_NODATA = 42113         # GDAL_NODATA         (ascii)

# TIFF field-type -> (struct code, byte size). Only the types we consume.
_TIFF_TYPE = {
    1: ("B", 1),   # BYTE
    2: ("s", 1),   # ASCII
    3: ("H", 2),   # SHORT
    4: ("I", 4),   # LONG
    5: ("II", 8),  # RATIONAL (num, den)
    11: ("f", 4),  # FLOAT
    12: ("d", 8),  # DOUBLE
}

# GeoKey IDs we read from the 34735 directory.
_GK_PROJ_LINEAR_UNITS = 3076
_GK_PROJ_STRAIGHT_VERT_LON = 3088
_GK_PROJ_NAT_ORIGIN_LAT = 3081
_GK_PROJ_SCALE_AT_NAT_ORIGIN = 3092
_GK_GEOG_SEMI_MAJOR_AXIS = 2057


def _read_tiff_ifd0(path) -> tuple[dict, str]:
    """Parse the FIRST IFD of a classic TIFF by hand; return ({tag: values}, byteorder).

    Values are returned as python tuples (one element per ``count``). Only the tags we
    need are materialized; pixel data is read separately via PIL. Supports both byte
    orders although LOLA products are little-endian ('II').
    """
    with open(path, "rb") as fh:
        header = fh.read(8)
        if header[:2] == b"II":
            bo = "<"
        elif header[:2] == b"MM":
            bo = ">"
        else:
            raise ValueError(f"{path}: not a TIFF (bad byte-order mark {header[:2]!r})")
        magic = struct.unpack(bo + "H", header[2:4])[0]
        if magic != 42:
            raise ValueError(f"{path}: not a classic TIFF (magic {magic}, BigTIFF unsupported)")
        ifd_off = struct.unpack(bo + "I", header[4:8])[0]

        fh.seek(ifd_off)
        n_entries = struct.unpack(bo + "H", fh.read(2))[0]
        raw = fh.read(n_entries * 12)

        tags: dict[int, tuple] = {}
        for i in range(n_entries):
            entry = raw[i * 12:(i + 1) * 12]
            tag, typ, count = struct.unpack(bo + "HHI", entry[:8])
            if typ not in _TIFF_TYPE:
                continue  # type we never consume
            code, size = _TIFF_TYPE[typ]
            total = size * (count * 2 if typ == 5 else count)
            value_field = entry[8:12]
            if total <= 4:
                blob = value_field[:total]
            else:
                off = struct.unpack(bo + "I", value_field)[0]
                fh.seek(off)
                blob = fh.read(total)
            tags[tag] = _decode(blob, typ, count, bo)
    return tags, bo


def _decode(blob: bytes, typ: int, count: int, bo: str):
    """Decode a TIFF tag value blob into a python tuple per its field type."""
    code, size = _TIFF_TYPE[typ]
    if typ == 2:  # ASCII (NUL-terminated)
        return (blob.split(b"\x00", 1)[0].decode("latin-1"),)
    if typ == 5:  # RATIONAL: pairs of LONGs
        nums = struct.unpack(bo + code * count, blob)
        return tuple(nums[2 * i] / nums[2 * i + 1] if nums[2 * i + 1] else float("nan")
                     for i in range(count))
    return struct.unpack(bo + code * count, blob)


def _parse_geokeys(directory: tuple, doubles: tuple) -> dict:
    """Parse the 34735 GeoKeyDirectory into {geo_key_id: value}.

    Layout (OGC GeoTIFF): the directory is a flat array of SHORTs in 4-tuples. The
    first tuple is a header ``(KeyDirVersion, KeyRev, MinorRev, NumberOfKeys)``; each
    following tuple is ``(KeyID, TIFFTagLocation, Count, Value_or_Offset)``. When
    ``TIFFTagLocation == 0`` the value is the literal short in ``Value_or_Offset``;
    when it points at 34736 (GeoDoubleParams) the value is ``doubles[offset]``.
    """
    out: dict[int, float] = {}
    if not directory or len(directory) < 4:
        return out
    n_keys = directory[3]
    for i in range(n_keys):
        base = 4 + i * 4
        if base + 3 >= len(directory):
            break
        key_id, loc, count, val = directory[base:base + 4]
        if loc == 0:
            out[key_id] = val
        elif loc == _TAG_GEO_DOUBLE_PARAMS and doubles and val < len(doubles):
            out[key_id] = doubles[val]
        # ASCII (34737) geokeys are descriptive only; skipped.
    return out


def load_lola_geotiff(path) -> tuple[np.ndarray, Affine, dict]:
    """Read a PGDA LOLA ``*_surf.tif`` via PIL; parse its GeoTIFF tags by hand.

    Returns ``(Z, affine, meta)`` where:
      * ``Z`` is a ``float32`` ndarray of HEIGHT-ABOVE-SPHERE in metres, shape
        (rows, cols), row 0 at the TOP (max Y). No ``Z - R`` subtraction is applied
        (LOLA ``*_surf`` Z is already a metre height, eval §4.2). NoData (if declared
        via GDAL_NODATA / a NaN) is left as-is — callers crop to finite windows.
      * ``affine`` maps pixel CENTERS to world metres (``X = x0 + col*px``,
        ``Y = y0 - row*px``); ``(x0, y0)`` is the ModelTiepoint (pixel-registered).
      * ``meta`` carries ``px``, ``tiepoint`` (x0, y0), ``R`` (sphere radius from the
        GeoKeys), ``nodata`` (or None), ``frame``, and the raster ``shape``.

    Pure PIL + numpy. The TIFF must be a classic (non-Big) TIFF in mode 'F'
    (single-band float32), which the PGDA Product-78 5 m tiles are.
    """
    from PIL import Image

    tags, _bo = _read_tiff_ifd0(path)

    scale = tags.get(_TAG_MODEL_PIXEL_SCALE)
    tie = tags.get(_TAG_MODEL_TIEPOINT)
    if scale is None or tie is None:
        raise ValueError(
            f"{path}: missing ModelPixelScale (33550) and/or ModelTiepoint (33922) — "
            "not a georeferenced GeoTIFF this ingest can place")
    px = float(scale[0])
    if len(scale) >= 2 and abs(scale[1] - px) > 1e-6 * max(px, 1.0):
        raise ValueError(f"{path}: non-square pixels {scale[:2]} unsupported (same-frame slice)")
    # ModelTiepoint: (i, j, k, X, Y, Z) maps raster (i, j) -> world (X, Y). For a
    # pixel-registered LOLA tile i=j=0 (first-pixel center).
    raster_i, raster_j = tie[0], tie[1]
    tie_x, tie_y = tie[3], tie[4]
    # If the tiepoint references a non-(0,0) pixel, back it out to the (0,0) origin.
    x0 = tie_x - raster_j * px
    y0 = tie_y + raster_i * px  # Y decreases with row, so origin Y is tie_y + i*px
    affine = Affine(x0=float(x0), y0=float(y0), px=px)

    geokeys = _parse_geokeys(tags.get(_TAG_GEO_KEY_DIRECTORY), tags.get(_TAG_GEO_DOUBLE_PARAMS))
    R = geokeys.get(_GK_GEOG_SEMI_MAJOR_AXIS)

    nodata = None
    nd_tag = tags.get(_TAG_GDAL_NODATA)
    if nd_tag:
        try:
            nodata = float(nd_tag[0])
        except (TypeError, ValueError):
            nodata = None

    im = Image.open(path)
    if im.mode != "F":
        raise ValueError(
            f"{path}: PIL mode {im.mode!r} (expected 'F' single-band float32). This ingest "
            "is for the LOLA *_surf.tif float32 product (eval §4.2).")
    Z = np.asarray(im, dtype=np.float32)

    meta = {
        "px": px,
        "tiepoint": [float(x0), float(y0)],
        "R": float(R) if R is not None else None,
        "nodata": nodata,
        "frame": "south polar stereographic, R=1737400 m sphere (IAU_2015:30135)",
        "z_semantics": "height above sphere [m] (NOT absolute radius; no Z-R subtraction)",
        "shape": [int(Z.shape[0]), int(Z.shape[1])],
        "source_path": str(path),
    }
    return Z, affine, meta


# ---------------------------------------------------------------------------
# 2. crop_square — same-frame pixel-window slice (NO reprojection).
# ---------------------------------------------------------------------------

def crop_square(Z: np.ndarray, affine: Affine, center_xy_m, extent_m: float
                ) -> tuple[np.ndarray, Affine]:
    """Slice a square ``extent_m`` x ``extent_m`` window centred on ``center_xy_m``.

    Pixel-registered, same-frame: the product is already south-polar stereographic,
    so this is a pure array slice with NO reprojection (eval §4.3). The window side
    is ``round(extent_m / px)`` pixels. The returned ``Affine`` is translated so its
    first-pixel CENTER is the world coord of the crop's (0,0) pixel — global offsets
    are preserved (this is where ``world_bounds_m`` non-zero offsets come from).

    The window is clamped to the raster; if the requested square does not fit fully
    inside the source a ``ValueError`` is raised (a partial NoData edge would corrupt
    the conservation round-trip). ``center_xy_m`` is ``(cx, cy)`` in world metres.
    """
    cx, cy = float(center_xy_m[0]), float(center_xy_m[1])
    px = affine.px
    n = int(round(extent_m / px))
    if n < 1:
        raise ValueError(f"crop_square: extent_m={extent_m} < one pixel ({px} m)")

    # Center pixel (fractional col,row), then the top-left of an n x n window around it.
    fcol, frow = affine.colrow(cx, cy)
    col0 = int(round(float(fcol) - (n - 1) / 2.0))
    row0 = int(round(float(frow) - (n - 1) / 2.0))

    H, W = Z.shape
    if row0 < 0 or col0 < 0 or row0 + n > H or col0 + n > W:
        raise ValueError(
            f"crop_square: {n}x{n} window at row0={row0},col0={col0} does not fit inside "
            f"the {H}x{W} raster (center=({cx},{cy}), extent={extent_m} m). A same-frame "
            "crop must lie fully inside the source — pick a center nearer the tile middle.")

    Z_crop = np.ascontiguousarray(Z[row0:row0 + n, col0:col0 + n])
    cx0, cy0 = affine.xy(row0, col0)
    affine_crop = affine.with_origin(float(cx0), float(cy0))
    return Z_crop, affine_crop


# ---------------------------------------------------------------------------
# 3. dem_to_base — inject the DEM surface via the FROZEN datum path.
# ---------------------------------------------------------------------------

def _resample_bilinear(Z: np.ndarray, affine: Affine, base_cell_m: float
                       ) -> tuple[np.ndarray, Affine]:
    """Resample ``Z`` (native ``affine.px``) to ``base_cell_m`` via scipy bilinear.

    At ``base_cell_m == affine.px`` this is a no-op (returns ``Z``, ``affine``). The
    resampled grid is pixel-registered on the SAME origin (first-cell center kept at
    ``(x0, y0)``) so global offsets stay exact. Bilinear (order=1) keeps the surface
    continuous; ``mode='nearest'`` clamps the edge so no NaN/extrapolation creeps in.
    """
    if abs(base_cell_m - affine.px) <= 1e-9 * max(base_cell_m, affine.px):
        return Z, affine

    from scipy.ndimage import map_coordinates

    ratio = base_cell_m / affine.px
    H, W = Z.shape
    # New cell centers, in source-pixel coordinates: cell j center sits at source index
    # j*ratio (cell 0 keeps the source first-pixel center -> origin preserved).
    n_rows = int(np.floor((H - 1) / ratio)) + 1
    n_cols = int(np.floor((W - 1) / ratio)) + 1
    rr = np.arange(n_rows) * ratio
    cc = np.arange(n_cols) * ratio
    grid_r, grid_c = np.meshgrid(rr, cc, indexing="ij")
    out = map_coordinates(Z.astype(np.float64), [grid_r, grid_c], order=1,
                          mode="nearest").astype(np.float32)
    affine_out = Affine(x0=affine.x0, y0=affine.y0, px=float(base_cell_m))
    return out, affine_out


def dem_to_base(Z_crop: np.ndarray, affine: Affine, base_cell_m: float, *,
                mantle_m: float = K.Z_T, density: float = K.RHO_SURFACE,
                density_fn=None) -> ColumnState:
    """Inject a DEM surface into a mass-conserving ``ColumnState`` via the datum path.

    The frozen surface-injection seam (column_state §, contract §1): author the
    surface into ``datum`` and a thin loose mantle into ``mass_areal`` so

        datum = Z - mantle_m ;  mass_areal = mantle_m * density
        derive_height() = datum + mass_areal/density == Z   (to ~1e-3 m)

    ``mantle_m`` is the CM-SCALE loose layer (~Z_T), NOT the metre-scale regolith
    column — the datum carries everything below the loose layer (eval §5 step 1).

    Z is consumed DIRECTLY (no ``Z - R`` subtraction; LOLA ``*_surf`` Z is already a
    metre height, eval §4.2). If ``base_cell_m`` differs from the native pixel size the
    DEM is bilinearly resampled (scipy) onto a pixel-registered grid on the same
    origin; at native 5 m it is a no-op slice.

    Density is UNIFORM by default (``constants.RHO_SURFACE``). A ``density_fn`` hook is
    accepted for Wave-2 (Lane B's polar profile); when given it is called as
    ``density_fn(X, Y)`` with the per-cell world-coordinate arrays [m] and must return a
    density array of the grid shape. Lane A depends on NO Lane B new constants.

    Note: ``ColumnState`` indexes ``[row, col]`` with world ``x = col*cell_m`` locally;
    the global offset lives in the scene metadata ``world_bounds_m`` (written by the
    caller / Lane C), so the grid here is the local frame and ``affine_out`` (returned
    via the ColumnState's attached ``_dem_affine``) carries the global placement.
    """
    Z_base, affine_out = _resample_bilinear(Z_crop, affine, base_cell_m)
    if not np.isfinite(Z_base).all():
        raise ValueError(
            "dem_to_base: non-finite Z in the resampled base (NoData in the crop?). "
            "Crop to a fully-finite window before injection.")

    h, w = Z_base.shape
    Z64 = Z_base.astype(np.float64)

    if density_fn is not None:
        rows = np.arange(h)
        cols = np.arange(w)
        gc, gr = np.meshgrid(cols, rows)  # (row, col) grids
        X, Y = affine_out.xy(gr, gc)
        rho = np.asarray(density_fn(X, Y), dtype=np.float64)
        if rho.shape != (h, w):
            raise ValueError(
                f"dem_to_base: density_fn returned shape {rho.shape}, expected {(h, w)}")
        if not np.all(rho > 0.0):
            raise ValueError("dem_to_base: density_fn returned a non-positive density")
    else:
        rho = np.full((h, w), float(density), dtype=np.float64)

    datum = Z64 - float(mantle_m)
    mass_areal = float(mantle_m) * rho

    cs = ColumnState(
        width=w, height=h, cell_m=float(base_cell_m),
        mass_areal=mass_areal,
        density=rho,
        datum=datum,
    )

    # Assert the surface round-trips through the datum path (contract §1).
    err = float(np.max(np.abs(cs.derive_height() - Z64)))
    if err > 1e-3:
        raise AssertionError(
            f"dem_to_base: derive_height() deviates from the DEM by {err:.3e} m (> 1e-3); "
            "the datum-path injection is broken")

    # Attach the global-frame affine so the end-to-end builder can write world_bounds_m
    # without re-deriving it (a plain attribute; ColumnState ignores extras).
    cs._dem_affine = affine_out  # type: ignore[attr-defined]
    return cs
