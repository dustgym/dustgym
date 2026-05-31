"""Windowed / memmap base reader — the streaming layer ON TOP of the frozen ``io_fields``
(L0 contract §5; eval §7 "tiled/windowed storage layer"; risks §11).

WHY this exists. ``io_fields.save_scene``/``load_scene`` are WHOLE-ARRAY (``io_fields.py:85``
reads the full raster into RAM). A 10 km @ 5 m base is only 2000² (16 MB/field, fine), but the
point of the corridor design is to NEVER hold the whole world dense — and a 10 km @ 1 m base
(10000², 0.4 GB/field) or any larger product must be read in WINDOWS. This module provides a
windowed reader over a per-tile raster layout (or a single big ``.rf32`` via numpy memmap) so
"only the active base pages in RAM" (L0 §5). It is a strict ADD-ON: it NEVER calls or changes
``save_scene``/``load_scene`` and never alters the frozen ``.rf32`` byte contract — it reads the
same little-endian row-major float32 / uint8 bytes ``io_fields`` defines, just a slice at a time.

Two backends, same windowed interface:
  * ``MemmapBaseReader`` — one big per-field ``.rf32``/``.r8`` raster on disk, read via
    ``numpy.memmap`` so a ``window(bbox)`` materializes only the requested sub-rectangle.
  * ``ArrayBaseReader`` — an in-RAM base (e.g. the 16 MB 5 m base, or a synthetic test base);
    windowing is a plain slice. Useful when the whole coarse base IS small enough to stay
    resident (the eval's "16 MB at 5 m — trivial, always resident") but the FINE corridor must
    still stream.

Both return a field dict (mass_areal/density/datum/state_label/disturbance) for a base-cell
``bbox`` = ``(r0, c0, r1, c1)`` half-open, matching ``refinement``'s field-dict convention so a
window feeds straight into ``overlay_residual`` / ``refine_field``.

Pure NumPy + stdlib. Dependency-free; no GDAL/rasterio (none exist here, contract).
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import numpy as np

# The carried base fields, matching refinement._FIELD_NAMES order. heightmap is DERIVED and
# never stored as a base field here (column_state.derive_height owns it).
BASE_FIELD_NAMES = ("mass_areal", "density", "datum", "state_label", "disturbance")

# On-disk dtype per field (matches io_fields._FIELD_SPEC for the overlapping names; datum is a
# base-only field not in the frozen save_scene set, stored as '<f4' for consistency).
_BASE_DTYPE = {
    "mass_areal": "<f4",
    "density": "<f4",
    "datum": "<f4",
    "state_label": "u1",
    "disturbance": "<f4",
}


def _clip_bbox(bbox, h: int, w: int) -> tuple[int, int, int, int]:
    """Clip a half-open ``(r0, c0, r1, c1)`` to the [0,h)x[0,w) grid; raise if empty/invalid."""
    r0, c0, r1, c1 = (int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3]))
    r0 = max(0, min(r0, h))
    c0 = max(0, min(c0, w))
    r1 = max(0, min(r1, h))
    c1 = max(0, min(c1, w))
    if r1 <= r0 or c1 <= c0:
        raise ValueError(f"dem_io: bbox {tuple(bbox)} is empty after clipping to ({h},{w})")
    return r0, c0, r1, c1


@dataclass
class ArrayBaseReader:
    """Windowed reader over an IN-RAM base field dict (slice = window).

    The whole base is held (e.g. the trivially-small 5 m coarse base); ``window`` returns a
    COPY of the requested sub-rectangle so callers can mutate windows without aliasing the base.
    """

    fields: dict[str, np.ndarray]
    base_cell_m: float
    world_x0: float = 0.0   # GLOBAL metre coordinate of base cell (0,0)'s lower corner
    world_y0: float = 0.0

    def __post_init__(self) -> None:
        ma = self.fields["mass_areal"]
        self.height, self.width = int(ma.shape[0]), int(ma.shape[1])
        for n in BASE_FIELD_NAMES:
            if n not in self.fields:
                raise ValueError(f"ArrayBaseReader: missing base field '{n}'")

    def window(self, bbox) -> dict[str, np.ndarray]:
        """Return the (r0,c0,r1,c1) base-cell window as a field dict (COPIES; float64/uint8)."""
        r0, c0, r1, c1 = _clip_bbox(bbox, self.height, self.width)
        out: dict[str, np.ndarray] = {}
        for n in BASE_FIELD_NAMES:
            a = self.fields[n][r0:r1, c0:c1]
            out[n] = (np.array(a, dtype=np.uint8) if n == "state_label"
                      else np.array(a, dtype=np.float64))
        return out

    def window_origin_m(self, bbox) -> tuple[float, float]:
        """GLOBAL (x, y) metre coordinate of the window's (r0,c0) lower corner.

        x grows with COLUMN, y grows with ROW (INTERFACE.md §2 / column_state row-major-C).
        """
        r0, c0, _, _ = _clip_bbox(bbox, self.height, self.width)
        return (self.world_x0 + c0 * self.base_cell_m,
                self.world_y0 + r0 * self.base_cell_m)


@dataclass
class MemmapBaseReader:
    """Windowed reader over per-field ``.rf32``/``.r8`` rasters on disk via ``numpy.memmap``.

    ``dir_`` holds ``mass_areal.rf32``, ``density.rf32``, ``datum.rf32``, ``disturbance.rf32``,
    ``state_label.r8`` (same byte format as ``io_fields``; row-major C, little-endian). A
    ``window(bbox)`` memmaps each file and slices ONLY the sub-rectangle into RAM — the rest of
    the raster is never paged in (O(window), not O(field)). This is the streaming base for a 10 km
    base too large to hold dense.

    The byte layout is the FROZEN ``io_fields`` contract; this reader only opens an existing
    on-disk base in read mode and never writes through the frozen path.
    """

    dir_: str
    height: int
    width: int
    base_cell_m: float
    world_x0: float = 0.0
    world_y0: float = 0.0

    def _path(self, name: str) -> str:
        ext = "r8" if name == "state_label" else "rf32"
        return os.path.join(self.dir_, f"{name}.{ext}")

    def window(self, bbox) -> dict[str, np.ndarray]:
        """Memmap each field and slice the (r0,c0,r1,c1) base-cell window into RAM."""
        r0, c0, r1, c1 = _clip_bbox(bbox, self.height, self.width)
        out: dict[str, np.ndarray] = {}
        for n in BASE_FIELD_NAMES:
            path = self._path(n)
            if not os.path.exists(path):
                raise FileNotFoundError(f"MemmapBaseReader: missing {path}")
            mm = np.memmap(path, dtype=_BASE_DTYPE[n], mode="r",
                           shape=(self.height, self.width))
            sub = mm[r0:r1, c0:c1]
            # Copy the slice out of the memmap so the file handle can be released and callers
            # get a writable, contiguous array (the rest of the raster never materializes).
            out[n] = (np.array(sub, dtype=np.uint8) if n == "state_label"
                      else np.array(sub, dtype=np.float64))
            del mm
        return out

    def window_origin_m(self, bbox) -> tuple[float, float]:
        """GLOBAL (x, y) metre coordinate of the window's (r0,c0) lower corner."""
        r0, c0, _, _ = _clip_bbox(bbox, self.height, self.width)
        return (self.world_x0 + c0 * self.base_cell_m,
                self.world_y0 + r0 * self.base_cell_m)


def write_base_rasters(dir_: str, fields: dict[str, np.ndarray]) -> None:
    """Write a base field dict as per-field ``.rf32``/``.r8`` rasters for ``MemmapBaseReader``.

    Convenience for tests / ingest hand-off (Lane A's ``dem_to_base`` produces the ColumnState;
    a mosaic builder can persist its base tiles through this). Uses the SAME byte format as
    ``io_fields`` (row-major C, little-endian) WITHOUT calling the frozen ``save_scene`` (this is
    a per-field raster dump, not a scene snapshot, so it never touches the frozen seam).
    """
    os.makedirs(dir_, exist_ok=True)
    for n in BASE_FIELD_NAMES:
        if n not in fields:
            raise ValueError(f"write_base_rasters: missing field '{n}'")
        ext = "r8" if n == "state_label" else "rf32"
        arr = fields[n]
        dt = _BASE_DTYPE[n]
        arr.astype(dt).tofile(os.path.join(dir_, f"{n}.{ext}"))
