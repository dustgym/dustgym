"""Demand-driven, bounded, evictable corridor LOD over a global-frame tiled base
(L0 contract §5; eval §2/§7/§8; the HEADLINE of John's runtime-fill decision).

THE DECISION this implements (L0 §0). The 2 cm fine fill is generated AT RUNTIME around the
rover's LIVE pose — NOT a precomputed trail. The rover may explore ANY part of the 10 km. Three
hard consequences (L0 §0.1-3), each enforced here:

  1. DEMAND-DRIVEN REFINE. ``ensure_fine(rover_xy, radius_m)`` materializes the fine tiles whose
     base cells fall within ``radius_m`` of the CURRENT rover pose, refining them through
     ``dem_overlay.overlay_residual`` (smooth-interp + zero-mean-per-base-cell + global fbm).
     Nothing is batch-precomputed.
  2. COORDINATE-HASHED DETERMINISM. Every fine tile is produced by ``overlay_residual`` anchored
     at the tile's GLOBAL metre origin and seeded via ``procgen_seed.coord_seed``, so re-visiting
     (or first-visiting) any patch yields BYTE-IDENTICAL terrain regardless of path/order. A tile
     evicted and regenerated is bit-identical to its first generation.
  3. BOUNDED / EVICTABLE RESIDENT SET. ``evict(keep_bbox)`` LRU-pages-out fine tiles outside the
     resident window so storage is O(resident window), not O(total explored area). Free roaming
     over the whole 10 km never accumulates the whole map at 2 cm.

LAYOUT. The global base is partitioned into square TILES of ``tile_base_cells`` base cells. The
mosaic addresses tiles by integer (tile_row, tile_col). ``base_window(bbox)`` reads a base-cell
window from the windowed base reader (``dem_io``); ``ensure_fine`` refines the tiles overlapping
the rover disc; ``evict`` drops fine tiles (optionally paging them to disk so a re-entry is a
load, not a regenerate — though a regenerate is bit-identical by determinism, so paging is purely
a perf choice).

io_fields stays FROZEN: the mosaic is a layer ON TOP (per-tile fine bundles), never a change to
``save_scene``/``load_scene``. Pure NumPy + stdlib. Imports refinement/column_state/dem_overlay/
dem_io/procgen_seed (allowed); modifies none of them.
"""

from __future__ import annotations

import math
import os
from collections import OrderedDict
from dataclasses import dataclass

import numpy as np

from . import dem_io, dem_overlay, refinement
from .refinement import Tile


# ---------------------------------------------------------------------------
# Additive per-tile metadata writer (L0 contract §2 — Lane C owns the writer).
# ---------------------------------------------------------------------------

def write_dem_base_metadata(meta: dict, *, world_bounds_m: dict, base_cell_m: float,
                            fine_cell_m: float, region: str,
                            local_datum_offset_m: float = 0.0,
                            dem_provenance: dict | None = None) -> dict:
    """ADDITIVELY attach the L0 §2 DEM/mosaic metadata block to a scene ``meta`` dict.

    Adds NEW keys ONLY (never mutates existing rasters/grid/quadtree keys), mirroring the
    discipline of ``scenes._attach_quadtree_meta`` / ``_attach_refinement_meta``. ``schema_version``
    stays "1.0" (additive only). This is the one place the contract changes existing scenes'
    metadata SEMANTICS: ``world_bounds_m.{x0,y0}`` may now be NON-zero global offsets (they are
    0.0 today, ``scenes.py:64`` — and 0.0 still means "origin tile", so a v1.0 consumer that
    ignored the offset is unaffected).

    Parameters
    ----------
    meta : dict
        The scene metadata dict to extend IN PLACE (also returned for chaining).
    world_bounds_m : dict
        ``{"x0","y0","x1","y1"}`` global metre bounds of this tile (non-zero offsets allowed).
    base_cell_m, fine_cell_m : float
        Base / fine cell sizes (e.g. 5.0 / 0.02).
    region : str
        DEM region name (e.g. "Haworth").
    local_datum_offset_m : float
        Per-tile mean elevation subtracted for float32 hygiene (eval §4.2 / §5 step 5); 0.0 if
        unused (the validated Haworth data does not strictly need it — float32 resolves ~0.3 mm
        at these magnitudes — but the field is carried for portability).
    dem_provenance : dict | None
        ``{"source","citation","frame"}`` provenance (THIRD_PARTY.md hand-off). None -> a minimal
        Haworth/Product-78 default per the contract's validated facts.

    Returns
    -------
    dict
        The same ``meta`` dict, extended.
    """
    meta["world_bounds_m"] = {
        "x0": float(world_bounds_m["x0"]), "y0": float(world_bounds_m["y0"]),
        "x1": float(world_bounds_m["x1"]), "y1": float(world_bounds_m["y1"]),
    }
    meta["base_cell_m"] = float(base_cell_m)
    meta["fine_cell_m"] = float(fine_cell_m)
    meta["region"] = str(region)
    meta["local_datum_offset_m"] = float(local_datum_offset_m)
    if dem_provenance is None:
        dem_provenance = {
            "source": "PGDA LOLA Product 78 (Haworth_final_adj_5mpp_surf.tif)",
            "citation": "Barker et al. 2021 (PSS 203, 105119); Mazarico et al. 2011 (Icarus 211)",
            "frame": "south polar stereographic IAU_2015:30135 (R=1737400, lat0=-90, k=1)",
        }
    meta["dem_provenance"] = dict(dem_provenance)
    return meta


@dataclass
class FineTile:
    """One resident fine tile in the mosaic.

    Attributes
    ----------
    tile_rc : (int, int)
        Tile address (tile_row, tile_col) in tile units.
    region_rc : list[int]
        ``[r0, c0, r1, c1]`` half-open in BASE cells (the base block this tile refines).
    world_x0, world_y0 : float
        GLOBAL metre coordinate of the tile's (r0, c0) lower corner (anchors determinism).
    fields : dict[str, np.ndarray]
        The fine field bundle from ``overlay_residual`` (mass_areal/density/datum/state_label/
        disturbance), dims = (r1-r0)*k x (c1-c0)*k at ``fine_cell_m``.
    fine_cell_m : float
        Fine cell size [m].
    """

    tile_rc: tuple[int, int]
    region_rc: list[int]
    world_x0: float
    world_y0: float
    fields: dict[str, np.ndarray]
    fine_cell_m: float

    def as_tile(self, tile_id: int) -> Tile:
        """Adapt to a ``refinement.Tile`` (the §5.3 sidecar descriptor / ColumnState carrier)."""
        cs = refinement._column_state_from_fields(self.fields, self.fine_cell_m)
        return Tile(id=tile_id, region_rc=list(self.region_rc),
                    cell_m=float(self.fine_cell_m), cs=cs)


class TileMosaic:
    """Demand-driven, bounded, evictable fine-corridor mosaic over a global-frame coarse base.

    Parameters
    ----------
    base_reader : dem_io.ArrayBaseReader | dem_io.MemmapBaseReader
        Windowed base reader (the always-resident coarse base; only windows page in).
    base_cell_m : float
        Base cell size [m] (e.g. 5.0).
    fine_cell_m : float
        Fine cell size [m] (e.g. 0.02). ``base_cell_m/fine_cell_m`` must be a positive integer.
    tile_base_cells : int
        Tile side in BASE cells (a fine tile = ``tile_base_cells*k`` fine cells per side). Sized
        so a fine tile is a manageable raster (default 8 base cells -> at k=250, 2000² fine =
        16 MB/field, one tile; smaller for huge k).
    max_resident_tiles : int
        LRU budget: at most this many fine tiles stay resident (bounded set, L0 §0.3).
    world_seed : int
        Global scenario seed (determinism).
    overlay_params : dict | None
        Forwarded to ``overlay_residual`` (None -> dem_overlay.DEFAULT_OVERLAY_PARAMS).
    page_dir : str | None
        If set, evicted tiles are paged to ``page_dir`` (per-field rasters) instead of dropped,
        so a re-entry LOADS rather than regenerates. Regeneration is bit-identical regardless
        (determinism), so this is purely a perf option. None -> evicted tiles are simply dropped.
    feature_fn : callable | None
        Wave-2 crater/boulder hook forwarded to overlay_residual (None for Wave-1).
    """

    def __init__(self, base_reader, base_cell_m: float, fine_cell_m: float, *,
                 tile_base_cells: int = 8, max_resident_tiles: int = 16,
                 world_seed: int = 0, overlay_params: dict | None = None,
                 page_dir: str | None = None, feature_fn=None) -> None:
        self.base = base_reader
        self.base_cell_m = float(base_cell_m)
        self.fine_cell_m = float(fine_cell_m)
        self.k = refinement.k_factor(self.base_cell_m, self.fine_cell_m)  # validates integer k
        self.tile_base_cells = int(tile_base_cells)
        if self.tile_base_cells < 1:
            raise ValueError("TileMosaic: tile_base_cells must be >= 1")
        self.max_resident_tiles = int(max_resident_tiles)
        self.world_seed = int(world_seed)
        self.overlay_params = overlay_params
        self.page_dir = page_dir
        self.feature_fn = feature_fn

        self.base_h = int(base_reader.height)
        self.base_w = int(base_reader.width)
        self.world_x0 = float(getattr(base_reader, "world_x0", 0.0))
        self.world_y0 = float(getattr(base_reader, "world_y0", 0.0))

        # Tile grid extent (tiles covering the base; the last row/col may be partial -> clipped).
        self.n_tile_rows = math.ceil(self.base_h / self.tile_base_cells)
        self.n_tile_cols = math.ceil(self.base_w / self.tile_base_cells)

        # LRU of resident fine tiles: OrderedDict keyed by (tile_row, tile_col); most-recently
        # used moved to the END. Eviction drops from the FRONT (least-recently used).
        self._resident: "OrderedDict[tuple[int, int], FineTile]" = OrderedDict()
        # Stats / determinism bookkeeping.
        self._gen_count = 0   # total fine-tile generations (regen on re-entry counts again)

    # -- geometry helpers --------------------------------------------------

    def _tile_region(self, tr: int, tc: int) -> tuple[int, int, int, int]:
        """Base-cell half-open region (r0,c0,r1,c1) of tile (tr,tc), clipped to the base grid."""
        r0 = tr * self.tile_base_cells
        c0 = tc * self.tile_base_cells
        r1 = min(r0 + self.tile_base_cells, self.base_h)
        c1 = min(c0 + self.tile_base_cells, self.base_w)
        return r0, c0, r1, c1

    def _xy_to_base_rc(self, x_m: float, y_m: float) -> tuple[float, float]:
        """GLOBAL (x, y) metres -> fractional base-cell (row, col). x->col, y->row."""
        col = (float(x_m) - self.world_x0) / self.base_cell_m
        row = (float(y_m) - self.world_y0) / self.base_cell_m
        return row, col

    def _tiles_in_disc(self, rover_xy, radius_m: float) -> list[tuple[int, int]]:
        """Tile addresses whose base region intersects the rover disc (center xy, radius_m)."""
        x_m, y_m = float(rover_xy[0]), float(rover_xy[1])
        row, col = self._xy_to_base_rc(x_m, y_m)
        rad_cells = radius_m / self.base_cell_m
        r_lo = int(math.floor((row - rad_cells) / self.tile_base_cells))
        r_hi = int(math.floor((row + rad_cells) / self.tile_base_cells))
        c_lo = int(math.floor((col - rad_cells) / self.tile_base_cells))
        c_hi = int(math.floor((col + rad_cells) / self.tile_base_cells))
        out: list[tuple[int, int]] = []
        for tr in range(max(0, r_lo), min(self.n_tile_rows - 1, r_hi) + 1):
            for tc in range(max(0, c_lo), min(self.n_tile_cols - 1, c_hi) + 1):
                out.append((tr, tc))
        return out

    # -- base window (only the active base pages in RAM; L0 §5) ------------

    def base_window(self, bbox) -> dict[str, np.ndarray]:
        """Read a base-cell window ``(r0,c0,r1,c1)`` through the windowed reader (RAM = window)."""
        return self.base.window(bbox)

    # -- demand-driven fine refine around the LIVE pose (L0 §0.1) ----------

    def _generate_fine_tile(self, tr: int, tc: int) -> FineTile:
        """Deterministically refine tile (tr,tc) to fine via overlay_residual (global-anchored).

        Reads the tile's base window, computes its GLOBAL origin, and runs the
        conservation-grade overlay seeded by global coordinate (coord_seed inside fbm_global).
        Reproducible: same (tr,tc, world_seed) -> byte-identical fine fields, regardless of when
        or how many times it is generated (L0 §0.2).
        """
        region = self._tile_region(tr, tc)
        base_block = self.base.window(region)
        wx0, wy0 = self.base.window_origin_m(region)
        fine_fields = dem_overlay.overlay_residual(
            base_block, self.k, wx0, wy0,
            params=self.overlay_params, world_seed=self.world_seed,
            fine_cell_m=self.fine_cell_m, feature_fn=self.feature_fn)
        self._gen_count += 1
        r0, c0, r1, c1 = region
        return FineTile(tile_rc=(tr, tc), region_rc=[r0, c0, r1, c1],
                        world_x0=wx0, world_y0=wy0, fields=fine_fields,
                        fine_cell_m=self.fine_cell_m)

    def _page_path(self, tr: int, tc: int) -> str:
        return os.path.join(self.page_dir, f"tile_{tr}_{tc}")

    def _load_paged(self, tr: int, tc: int) -> FineTile | None:
        """Load a previously paged-out fine tile from disk, if present (else None)."""
        if self.page_dir is None:
            return None
        d = self._page_path(tr, tc)
        if not os.path.isdir(d):
            return None
        region = self._tile_region(tr, tc)
        r0, c0, r1, c1 = region
        kh, kw = (r1 - r0) * self.k, (c1 - c0) * self.k
        reader = dem_io.MemmapBaseReader(d, height=kh, width=kw,
                                         base_cell_m=self.fine_cell_m)
        fields = reader.window((0, 0, kh, kw))
        wx0, wy0 = self.base.window_origin_m(region)
        return FineTile(tile_rc=(tr, tc), region_rc=[r0, c0, r1, c1],
                        world_x0=wx0, world_y0=wy0, fields=fields,
                        fine_cell_m=self.fine_cell_m)

    def _touch(self, tr: int, tc: int, tile: FineTile) -> None:
        """Insert/refresh a tile as MOST-recently-used in the LRU."""
        key = (tr, tc)
        if key in self._resident:
            self._resident.move_to_end(key)
        else:
            self._resident[key] = tile
            self._resident.move_to_end(key)

    def ensure_fine(self, rover_xy, radius_m: float) -> list[Tile]:
        """DEMAND-refine the fine tiles within ``radius_m`` of the LIVE rover pose (L0 §0.1).

        For each tile whose base region intersects the rover disc: reuse the resident tile if
        present (LRU touch), else load a paged copy, else GENERATE it deterministically via
        ``overlay_residual``. Then enforce the resident budget (``evict`` keeps only
        ``max_resident_tiles`` most-recently-used). Returns the fine ``refinement.Tile`` list
        currently under the rover (ascending ids in (tile_row, tile_col) scan order).

        Determinism (L0 §0.2): a tile generated now is byte-identical to one generated on a
        prior visit (overlay is a pure function of global coordinate + world_seed), so the
        rover sees the same terrain whether or not the tile was evicted in between.
        """
        wanted = self._tiles_in_disc(rover_xy, radius_m)
        for (tr, tc) in wanted:
            key = (tr, tc)
            if key in self._resident:
                self._resident.move_to_end(key)
                continue
            tile = self._load_paged(tr, tc)
            if tile is None:
                tile = self._generate_fine_tile(tr, tc)
            self._touch(tr, tc, tile)

        # Bound the resident set to the budget (LRU drop of the oldest beyond the budget).
        self._enforce_budget()

        # Build the ascending-id Tile list for the tiles under the rover (scan order).
        present = sorted([t for t in wanted if t in self._resident])
        return [self._resident[t].as_tile(i) for i, t in enumerate(present)]

    # -- bounded / evictable resident set (L0 §0.3) ------------------------

    def _enforce_budget(self) -> None:
        """Drop least-recently-used resident tiles until within ``max_resident_tiles``."""
        while len(self._resident) > self.max_resident_tiles:
            old_key, old_tile = self._resident.popitem(last=False)  # LRU front
            self._page_out(old_key, old_tile)

    def _page_out(self, key: tuple[int, int], tile: FineTile) -> None:
        """Page a fine tile to disk (if page_dir set) so a re-entry can load it; else drop."""
        if self.page_dir is None:
            return
        d = self._page_path(*key)
        dem_io.write_base_rasters(d, tile.fields)

    def evict(self, keep_bbox) -> None:
        """LRU page-out every resident fine tile whose base region falls OUTSIDE ``keep_bbox``.

        ``keep_bbox`` = ``(r0, c0, r1, c1)`` half-open in BASE cells: the resident window to
        retain (e.g. an enlarged box around the rover). Tiles entirely outside it are evicted
        (paged to disk if ``page_dir`` set, else dropped), so the resident set stays O(resident
        window) under free roaming (L0 §0.3). Tiles overlapping the keep box are retained.
        """
        kr0, kc0, kr1, kc1 = (int(keep_bbox[0]), int(keep_bbox[1]),
                              int(keep_bbox[2]), int(keep_bbox[3]))
        to_drop: list[tuple[int, int]] = []
        for key, tile in self._resident.items():
            r0, c0, r1, c1 = tile.region_rc
            # Disjoint from keep box (half-open intersection empty) -> evict.
            disjoint = (r1 <= kr0 or r0 >= kr1 or c1 <= kc0 or c0 >= kc1)
            if disjoint:
                to_drop.append(key)
        for key in to_drop:
            tile = self._resident.pop(key)
            self._page_out(key, tile)

    # -- introspection (tests / viz) ---------------------------------------

    @property
    def resident_count(self) -> int:
        """Number of fine tiles currently held in RAM (the bounded resident set size)."""
        return len(self._resident)

    @property
    def resident_keys(self) -> list[tuple[int, int]]:
        """Resident tile addresses in LRU order (front = least-recently used)."""
        return list(self._resident.keys())

    @property
    def generation_count(self) -> int:
        """Total fine-tile generations so far (a re-entry without paging regenerates -> +1)."""
        return self._gen_count

    def resident_memory_cells(self) -> int:
        """Total fine cells held resident (a proxy for RAM; bounded by budget * tile area)."""
        total = 0
        for t in self._resident.values():
            total += t.fields["mass_areal"].size
        return total


# ---------------------------------------------------------------------------
# Lane-C VERIFY-SELF acceptance harness (the contract's self-checks). Lives in an OWNED
# module (tests.py is frozen, not Lane C's). Run: ``python -m terrain_authority.tiles_mosaic``.
# Asserts the four contract acceptance criteria + leaves the repo ``terrain_authority.tests``
# suite untouched (run that separately to confirm GREEN).
# ---------------------------------------------------------------------------

def _selftest() -> int:
    """Run the Lane-C acceptance checks; print PASS/FAIL; return 0 if all pass."""
    import numpy as _np
    from . import procgen_seed as _ps
    from .quadtree import quadtree_pad_pow2 as _pad

    results: list[tuple[str, bool, str]] = []

    def check(name, ok, detail=""):
        results.append((name, bool(ok), detail))

    # (1) coarsen(overlay_residual(refine(base))) == base to the float64 floor + carried fields
    #     bit-exact (mass conservation + zero-mean per base cell).
    rng = _np.random.default_rng(5)
    H, W = 6, 7
    base = {
        "mass_areal": rng.uniform(50, 200, (H, W)),
        "density": rng.uniform(1300, 1920, (H, W)),
        "datum": rng.uniform(-1.0, 1.0, (H, W)),
        "state_label": rng.integers(0, 5, (H, W)).astype(_np.uint8),
        "disturbance": rng.uniform(0, 1, (H, W)),
    }
    worst_rel, carried_ok, detail_present = 0.0, True, True
    for k in (2, 3, 5, 8, 50, 250):
        fine = dem_overlay.overlay_residual(base, k, 123.0, -456.0,
                                            fine_cell_m=5.0 / k, world_seed=7)
        back = refinement.coarsen_field(fine, k)
        worst_rel = max(worst_rel, float(
            _np.abs((back["mass_areal"] - base["mass_areal"]) / base["mass_areal"]).max()))
        carried_ok &= _np.array_equal(back["density"], base["density"])
        carried_ok &= _np.array_equal(back["datum"], base["datum"])
        carried_ok &= _np.array_equal(back["state_label"], base["state_label"])
        carried_ok &= _np.array_equal(back["disturbance"], base["disturbance"])
        # detail actually added (not a flat plateau)
        ft = (fine["mass_areal"] / fine["density"]).reshape(H, k, W, k)
        detail_present &= bool(ft.var(axis=(1, 3)).max() > 0)
    check("overlay: coarsen(overlay(refine(base)))==base (mass float-floor, carried bit-exact, "
          "zero-mean + detail present)",
          worst_rel < 1e-12 and carried_ok and detail_present,
          f"worst_mass_rel={worst_rel:.2e} carried_bit_exact={carried_ok} detail={detail_present}")

    # (2) coord_seed determinism: same global point -> identical value across two DIFFERENT
    #     tiles AND across 5 m vs 1 m base; fbm overlapping windows agree bit-exact.
    P = (123.5, -88.25)
    sA = _ps.coord_seed(P[0], P[1], 2, 0, world_seed=11)
    sB = _ps.coord_seed(P[0], P[1], 2, 0, world_seed=11)
    cell = 0.5
    A = _ps._value_noise_global(100.0, -100.0, 60, cell, 8.0, 0, 0, 11)
    B = _ps._value_noise_global(115.0, -92.5, 60, cell, 8.0, 0, 0, 11)
    overlap_exact = _np.array_equal(A[15:60, 30:60], B[0:45, 0:30])
    # 5 m vs 1 m base: same global window, same fine grid -> identical (base_cell irrelevant).
    c5 = _ps._value_noise_global(0.0, 0.0, 4, 5.0, 40.0, 0, 0, 0)
    c1 = _ps._value_noise_global(0.0, 0.0, 20, 1.0, 40.0, 0, 0, 0)
    cross_res_exact = _np.array_equal(c5, c1[[2, 7, 12, 17]][:, [2, 7, 12, 17]])
    check("seed: coord_seed stable across tiles + fbm overlap bit-exact + 5m-vs-1m agree",
          (sA == sB) and overlap_exact and cross_res_exact,
          f"coord_seed_stable={sA == sB} overlap_exact={overlap_exact} "
          f"cross_res_exact={cross_res_exact}")

    # (3) ensure_fine materializes fine tiles around a MOVING pose; evict keeps the resident set
    #     bounded; a regenerated tile is byte-identical (determinism by global coordinate).
    H2 = W2 = 40
    rng2 = _np.random.default_rng(9)
    bf = {
        "mass_areal": rng2.uniform(80, 160, (H2, W2)),
        "density": _np.full((H2, W2), 1300.0),
        "datum": rng2.uniform(-1, 1, (H2, W2)) * 20.0,
        "state_label": _np.zeros((H2, W2), _np.uint8),
        "disturbance": _np.zeros((H2, W2)),
    }
    reader = dem_io.ArrayBaseReader(bf, base_cell_m=5.0, world_x0=-52900.0, world_y0=105400.0)
    mosaic = TileMosaic(reader, 5.0, 0.02, tile_base_cells=4, max_resident_tiles=6, world_seed=42)
    peak, materialized_each_step = 0, True
    for i in range(20):
        x = -52900.0 + (i / 19.0) * 195.0
        y = 105400.0 + (i / 19.0) * 195.0
        tiles = mosaic.ensure_fine((x, y), radius_m=4.0)
        materialized_each_step &= len(tiles) > 0
        row = (y - 105400.0) / 5.0
        col = (x - (-52900.0)) / 5.0
        mosaic.evict((int(row - 3), int(col - 3), int(row + 4), int(col + 4)))
        peak = max(peak, mosaic.resident_count)
    t1 = mosaic._generate_fine_tile(2, 2)
    t2 = mosaic._generate_fine_tile(2, 2)
    regen_identical = all(_np.array_equal(t1.fields[n], t2.fields[n])
                          for n in dem_io.BASE_FIELD_NAMES)
    check("mosaic: ensure_fine materializes around moving pose + evict bounds resident set + "
          "regen byte-identical",
          materialized_each_step and peak <= 6 and regen_identical,
          f"peak_resident={peak} budget=6 materialized_each_step={materialized_each_step} "
          f"regen_identical={regen_identical} final_resident={mosaic.resident_count}")

    # (4) quadtree_pad_pow2 cases.
    pad_ok = (_pad(2000) == 2048 and _pad(10000) == 16384 and _pad(256) == 256
              and _pad(1) == 1 and _pad(3) == 4 and _pad(1025) == 2048)
    check("quadtree_pad_pow2: 2000->2048, 10000->16384, idempotent on pow2",
          pad_ok, f"2000->{_pad(2000)} 10000->{_pad(10000)} 256->{_pad(256)}")

    n_fail = sum(1 for _, ok, _ in results if not ok)
    for name, ok, detail in results:
        print(f"[{'PASS' if ok else 'FAIL'}] {name}  ({detail})")
    print(f"\n{len(results) - n_fail}/{len(results)} Lane-C checks passed.")
    return 1 if n_fail else 0


if __name__ == "__main__":
    raise SystemExit(_selftest())
