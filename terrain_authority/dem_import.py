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
is required here (eval addendum §4.3). (`pyproj` IS available in the product runtime
and `planet_browser/dem_import.py` uses it for the non-polar / equatorial reproject
path; this module deliberately stays GDAL/rasterio-free for the polar same-frame lane.)

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
from typing import Callable

import numpy as np

from . import constants as K
from .column_state import ColumnState

__all__ = ["Affine", "load_lola_geotiff", "crop_square", "dem_to_base",
           "polar_mantle_density_fn"]


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
            total = size * count   # size already covers a whole element (RATIONAL=8 B = the num+den pair)
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


# ---------------------------------------------------------------------------
# 4. polar_mantle_density_fn — fill dem_to_base's density_fn hook with the
#    depth-integrated ChaSTE bulk density (Wave-2 W2-DENSITY).
# ---------------------------------------------------------------------------

def polar_mantle_density_fn(mantle_m: float = K.Z_T
                            ) -> Callable[[np.ndarray, np.ndarray], np.ndarray]:
    """Build the ``density_fn(X, Y)`` hook for ``dem_to_base`` from the ChaSTE profile.

    Resolves an AXIS MISMATCH (contract §8, W2-DENSITY): ``dem_to_base`` calls its
    ``density_fn(X, Y)`` with per-cell WORLD-coordinate arrays [m], but
    ``constants.polar_density_profile`` takes DEPTH below the surface [m]. They live
    on different axes, so the profile cannot be handed in directly.

    The bridge is to collapse the depth axis: integrate the ChaSTE profile over the
    loose mantle ``[0, mantle_m]`` and divide by ``mantle_m`` to get the
    DEPTH-INTEGRATED (mass-weighted-mean) bulk density of that column — one scalar

        rho_bar = (1/mantle_m) * integral_0^mantle_m polar_density_profile(z) dz .

    Be honest about what this is: a SINGLE mass-weighted-mean scalar BROADCAST across
    the whole grid, NOT a spatial density field. The returned closure IGNORES X, Y and
    returns a constant grid of ``rho_bar`` shaped to the inputs. (A true spatial polar
    field would need lateral density data we do not have; ChaSTE is one vertical probe
    at 69.4 deg S.) The mass-weighting follows ``polar_density_profile``'s own piecewise
    constants — RHO_SURFACE_POLAR over [0, 3 cm), RHO_MID_POLAR over [3, 6.5 cm), and
    the RHO_BULK_POLAR_10CM stand-in beyond the ~6.5 cm ChaSTE band (constants §ChaSTE,
    eval §6 row). With the default ``mantle_m == Z_T == 0.12 m`` the deeper-than-6.5-cm
    remainder is the dominant slab, so ``rho_bar`` sits between RHO_MID_POLAR and
    RHO_BULK_POLAR_10CM (and always within [RHO_SURFACE_POLAR, RHO_BULK_POLAR_10CM]).

    Why it does not change the surface: in ``dem_to_base`` density CANCELS out of the
    height inversion (``datum = Z - mantle_m``, ``mass = mantle_m * rho``,
    ``height = datum + mass/rho == Z`` for any positive rho). So the loose mantle now
    carries a sourced polar AREAL MASS (``mantle_m * rho_bar`` kg/m^2) instead of the
    equatorial-Apollo ``RHO_SURFACE`` stand-in, with ``derive_height()`` untouched.
    """
    mantle_m = float(mantle_m)
    if mantle_m <= 0.0:
        raise ValueError(f"polar_mantle_density_fn: mantle_m={mantle_m} must be > 0")

    # Mass-weighted mean = (integral of the piecewise-constant profile over [0, mantle_m])
    # / mantle_m. Integrate analytically off polar_density_profile's own band edges/
    # values so the closure stays sourced to that function (no re-typed constants).
    edges = [0.0, K.Z_POLAR_TOP_M, K.Z_POLAR_MID_M, mantle_m]
    integral = 0.0
    for lo, hi in zip(edges, edges[1:]):
        lo = min(max(lo, 0.0), mantle_m)
        hi = min(max(hi, 0.0), mantle_m)
        if hi <= lo:
            continue
        # Evaluate the profile at the band MIDPOINT to pick its constant value.
        rho_band = float(K.polar_density_profile((lo + hi) / 2.0))
        integral += rho_band * (hi - lo)
    rho_bar = integral / mantle_m

    def density_fn(X: np.ndarray, Y: np.ndarray) -> np.ndarray:
        """Constant ChaSTE depth-integrated bulk density [kg/m^3], broadcast over the
        grid. IGNORES X, Y (this is a scalar, not a spatial field — see closure doc)."""
        return np.full(np.shape(X), rho_bar, dtype=np.float64)

    # Expose the scalar for honest logging/metadata (callers may read it; closures
    # are otherwise opaque). A plain attribute; nothing depends on it structurally.
    density_fn.rho_bar = rho_bar  # type: ignore[attr-defined]
    density_fn.mantle_m = mantle_m  # type: ignore[attr-defined]
    return density_fn


# ---------------------------------------------------------------------------
# Self-test (W2-DENSITY) — exercises polar_mantle_density_fn end to end.
#   python -m terrain_authority.dem_import
# ---------------------------------------------------------------------------

def _self_test() -> int:
    results: list[tuple[str, bool, str]] = []

    def check(name: str, ok: bool, detail: str = "") -> None:
        results.append((name, bool(ok), detail))
        print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))

    # 1. The returned density is CONSTANT and lies in the sourced ChaSTE range
    #    [RHO_SURFACE_POLAR, RHO_BULK_POLAR_10CM]. This is the REAL density acceptance
    #    check (the height round-trip below cannot see density — it cancels). -----------
    dfn = polar_mantle_density_fn()  # default mantle_m == K.Z_T
    Xq = np.linspace(-50.0, 50.0, 7)
    Yq = np.linspace(200.0, 260.0, 5)
    XX, YY = np.meshgrid(Xq, Yq)
    rho = dfn(XX, YY)
    is_const = bool(rho.shape == XX.shape and np.ptp(rho) == 0.0)
    rho0 = float(rho.flat[0])
    in_range = bool(K.RHO_SURFACE_POLAR <= rho0 <= K.RHO_BULK_POLAR_10CM)
    # Independent recomputation of the mass-weighted mean for Z_T (0.12 m): bands are
    # [0,3cm)@750, [3,6.5cm)@1300, [6.5,12cm)@1940.
    expect = (K.RHO_SURFACE_POLAR * K.Z_POLAR_TOP_M
              + K.RHO_MID_POLAR * (K.Z_POLAR_MID_M - K.Z_POLAR_TOP_M)
              + K.RHO_BULK_POLAR_10CM * (K.Z_T - K.Z_POLAR_MID_M)) / K.Z_T
    matches = abs(rho0 - expect) <= 1e-9
    check("density is a constant grid in [RHO_SURFACE_POLAR, RHO_BULK_POLAR_10CM] "
          "= mass-weighted ChaSTE mean",
          is_const and in_range and matches,
          f"rho_bar={rho0:.2f} expect={expect:.2f} "
          f"[{K.RHO_SURFACE_POLAR:.0f},{K.RHO_BULK_POLAR_10CM:.0f}] "
          f"const={is_const} closure_attr={dfn.rho_bar:.2f}")

    # 2. Feeding the hook through dem_to_base on a small synthetic DEM still satisfies
    #    derive_height() == Z within 1e-3 m. NOTE: density CANCELS in derive_height
    #    (datum=Z-mantle, mass=mantle*rho, height=datum+mass/rho=Z for ANY rho>0), so
    #    this is a ROUND-TRIP SANITY that the hook is plumbed correctly — the real
    #    density assertion is the range check in (1), not this. --------------------------
    aff = Affine(x0=-52900.0, y0=105400.0, px=5.0)  # real-tile-ish global origin
    rng = np.random.default_rng(0)
    Z = (np.linspace(-3.0, 4.0, 6)[:, None] + np.linspace(0.0, 2.0, 8)[None, :]
         + 0.1 * rng.standard_normal((6, 8))).astype(np.float32)
    cs = dem_to_base(Z, aff, 5.0, mantle_m=K.Z_T, density_fn=polar_mantle_density_fn())
    h = cs.derive_height()
    rt_err = float(np.max(np.abs(h - Z.astype(np.float64))))
    # The density field that actually landed in the ColumnState must be the constant.
    landed_const = bool(np.ptp(cs.density) == 0.0
                        and abs(float(cs.density.flat[0]) - rho0) <= 1e-9)
    # And it must be the POLAR value, not the equatorial RHO_SURFACE stand-in.
    differs_from_default = abs(rho0 - K.RHO_SURFACE) > 1.0
    check("dem_to_base with the hook: derive_height()==Z within 1e-3 m (density cancels) "
          "and the landed density is the polar constant",
          rt_err <= 1e-3 and landed_const and differs_from_default,
          f"rt_err={rt_err:.2e} m landed_rho={float(cs.density.flat[0]):.2f} "
          f"(RHO_SURFACE stand-in was {K.RHO_SURFACE:.0f})")

    # 3. Monotonicity / band-collapse robustness: a thinner mantle that stops inside the
    #    top fines must give a LOWER mass-weighted mean (more weight on the loose top),
    #    and one within only the top band returns exactly RHO_SURFACE_POLAR. ------------
    thin = polar_mantle_density_fn(0.02).rho_bar     # all inside the 0-3 cm @750 band
    mid = polar_mantle_density_fn(0.05).rho_bar      # spans 750 + 1300 bands
    monotone = bool(abs(thin - K.RHO_SURFACE_POLAR) <= 1e-9
                    and K.RHO_SURFACE_POLAR < mid < rho0 <= K.RHO_BULK_POLAR_10CM)
    check("mass-weighting is depth-correct: thinner mantle -> lower mean "
          "(top-only == RHO_SURFACE_POLAR)",
          monotone, f"thin(2cm)={thin:.1f} mid(5cm)={mid:.1f} full(Z_T)={rho0:.1f}")

    n_fail = sum(1 for _, ok, _ in results if not ok)
    print(f"\n{len(results) - n_fail}/{len(results)} checks passed.")
    return 1 if n_fail else 0


if __name__ == "__main__":
    raise SystemExit(_self_test())
