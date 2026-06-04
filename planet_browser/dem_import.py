"""P4: ingest a NON-POLAR DEM product (a cylindrical / equirectangular lat-lon heightfield) and reproject
it onto the sim's LOCAL METRIC grid, so the planner can plan on real maps beyond the polar-stereographic
Haworth bundle.

The reprojection is done with pyproj: source = the body's geographic CRS (a sphere of the given radius);
target = a LOCAL azimuthal-equidistant frame centred on the patch (so distances near the centre are true
metres). Heights are PRESERVED -- only the horizontal frame changes -- so relief round-trips within
bilinear-resampling tolerance.

REAL DATA ONLY. The bundled fixture (`fixtures/ldem4_equator_*`) is a tiny equatorial window of the real
LOLA `ldem_4` global DEM (PDS lrolol_1xxx, simple-cylindrical 4 px/deg, R=1737400 m), subsampled to a
fixture -- never synthetic terrain.
"""

from __future__ import annotations

import json
import os

import numpy as np
from pyproj import CRS, Transformer


def _geographic_crs(radius_m: float) -> CRS:
    """Body geographic CRS on a sphere of the given radius (lon/lat degrees)."""
    return CRS.from_proj4(f"+proj=longlat +R={radius_m} +no_defs")


def _local_aeqd_crs(lat0: float, lon0: float, radius_m: float) -> CRS:
    """Local azimuthal-equidistant frame (metres) centred on the patch -- true distances near centre."""
    return CRS.from_proj4(f"+proj=aeqd +lat_0={lat0} +lon_0={lon0} +R={radius_m} +units=m +no_defs")


def _bilinear(A: np.ndarray, frow: np.ndarray, fcol: np.ndarray) -> np.ndarray:
    """Bilinear sample of array A at fractional (row, col); clamped to bounds. Output never exceeds the
    local source range, so reprojected relief is <= source relief (heights are interpolated, not invented)."""
    H, W = A.shape
    fr = np.clip(frow, 0.0, H - 1.0)
    fc = np.clip(fcol, 0.0, W - 1.0)
    r0 = np.floor(fr).astype(int)
    c0 = np.floor(fc).astype(int)
    r1 = np.minimum(r0 + 1, H - 1)
    c1 = np.minimum(c0 + 1, W - 1)
    dr = fr - r0
    dc = fc - c0
    top = A[r0, c0] * (1 - dc) + A[r0, c1] * dc
    bot = A[r1, c0] * (1 - dc) + A[r1, c1] * dc
    return top * (1 - dr) + bot * dr


def reproject_cylindrical(heights_m, *, lat_top, lat_bottom, lon_left, lon_right, radius_m,
                          target_cell_m=None):
    """Reproject a cylindrical (equirectangular lat/lon) height patch [m] to a LOCAL metric grid centred on
    the patch (azimuthal-equidistant). Returns (Z_local [m], cell_m). The source grid is row 0 = lat_top
    .. row H-1 = lat_bottom, col 0 = lon_left .. col W-1 = lon_right (PDS convention)."""
    heights_m = np.asarray(heights_m, dtype=np.float64)
    H, W = heights_m.shape
    lat0 = 0.5 * (lat_top + lat_bottom)
    lon0 = 0.5 * (lon_left + lon_right)
    fwd = Transformer.from_crs(_geographic_crs(radius_m), _local_aeqd_crs(lat0, lon0, radius_m), always_xy=True)
    inv = Transformer.from_crs(_local_aeqd_crs(lat0, lon0, radius_m), _geographic_crs(radius_m), always_xy=True)
    # local metric extent = the projected patch corners
    cx, cy = fwd.transform([lon_left, lon_right, lon_left, lon_right],
                           [lat_top, lat_top, lat_bottom, lat_bottom])
    x0, x1, y0, y1 = min(cx), max(cx), min(cy), max(cy)
    if target_cell_m is None:                              # native-ish: patch metric span / source pixel count
        target_cell_m = max((x1 - x0) / max(1, W - 1), (y1 - y0) / max(1, H - 1))
    nx = max(2, int(round((x1 - x0) / target_cell_m)) + 1)
    ny = max(2, int(round((y1 - y0) / target_cell_m)) + 1)
    gx = x0 + np.arange(nx) * target_cell_m
    gy = y0 + np.arange(ny) * target_cell_m
    GX, GY = np.meshgrid(gx, gy)
    LON, LAT = inv.transform(GX, GY)                       # each local cell -> source lon/lat
    fcol = (LON - lon_left) / (lon_right - lon_left) * (W - 1)
    frow = (lat_top - LAT) / (lat_top - lat_bottom) * (H - 1)
    return _bilinear(heights_m, frow, fcol), float(target_cell_m)


def load_cylindrical_fixture(npy_path, json_path):
    """Load a cylindrical DEM patch fixture: returns (heights_m, geometry dict). heights = DN * scaling."""
    geom = json.load(open(json_path))
    dn = np.load(npy_path).astype(np.float64)
    return dn * float(geom["scaling_m_per_dn"]), geom


def ingest_to_bundle(heights_m, cell_m, out_dir, *, body="moon", source=""):
    """Write a sim bundle (metadata.json + heightmap.rf32) in the same format as the Haworth bundle, so
    `mission_planner.load_haworth_dem`-style readers / `read_dem_window` can consume the ingested map."""
    heights_m = np.asarray(heights_m, dtype=np.float64)
    os.makedirs(out_dir, exist_ok=True)
    H, W = heights_m.shape
    heights_m.astype("<f4").tofile(os.path.join(out_dir, "heightmap.rf32"))
    meta = {"grid": {"width": int(W), "height": int(H), "cell_m": float(cell_m), "order": "row-major-C"},
            "source": source, "body": body}
    with open(os.path.join(out_dir, "metadata.json"), "w") as f:
        json.dump(meta, f, indent=1)
    return out_dir
