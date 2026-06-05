"""WorkSite — streaming coarse Haworth base + rover-following fine WORKED window.

The world model behind the scripted flatten -> haul -> berm demo (docs/worksite_contract.md),
shaped so an RL policy can drive the SAME .drive()/.flatten()/.dump()/.relax() seam later.
The controller is the only stub: the world is genuinely streaming-dynamic from day one.

Three layers (contract):
  * ``base``    : coarse Haworth ``ColumnState`` (5 m), READ-ONLY virgin authority.
  * ``mosaic``  : ``TileMosaic`` over the base — the streaming VIRGIN fine-terrain source
                  (DEM block + sourced procedural overlay; ``page_dir`` persists worked tiles).
  * ``fine``    : the rover-following WORKED fine ``ColumnState`` window — OUR OWN copy
                  (never the aliased ``Tile.cs``), the authoritative worked store.
  * ``inventory_kg`` : the GLOBAL drum ledger; carries mass across windows (dig here, dump there).

Mass invariant: ``fine.grid_mass() + inventory_kg`` is constant across flatten/dump/relax/
drive/compact (all mass-conserving or grid<->ledger). The coarse base is never mutated by
physics. See docs/worksite_contract.md for the documented deferred gaps (G1 cross-seam relax,
G2 worked-tile streaming, G3 Godot base-only render, G4 drum-arm pose, G5 berm pinning).
"""
from __future__ import annotations

import numpy as np

from . import constants as K
from . import rover as R
from . import drive as D
from .column_state import ColumnState, StateLabel
from .sandpile import Sandpile
from .io_fields import load_scene, save_scene
from .dem_io import ArrayBaseReader, BASE_FIELD_NAMES
from .tiles_mosaic import TileMosaic


# ---------------------------------------------------------------------------
# Flatten primitive — cut high spots to a target LEVEL into the drum (mass-conserving).
# drum_pass only cuts a UNIFORM depth; flattening an uneven pad to a plane needs a
# per-cell variable depth. Verified numerically exact (residual above target = 0.0).
# ---------------------------------------------------------------------------

def flatten_to_level(cs: ColumnState, mask: np.ndarray, target_m: float) -> tuple[float, np.ndarray]:
    """Cut every masked cell above ``target_m`` down TOWARD that level, routing the removed
    areal mass into ``cs.drum_inventory`` (mass-conserving).

    Returns ``(moved_kg, hi_mask)`` where ``hi_mask`` is the cells actually cut. Removes column
    thickness ``(height-target)`` at each cell's density -> height drops by that amount UNTIL the
    cell runs out of removable mass: ``cut_to_inventory`` clamps at the firm DEM datum, so a cell
    whose ``target`` is below its datum floors at the datum, not at ``target`` (only the loose
    mantle above datum is removable — gap G8). The achieved floor therefore traces the datum
    (terraced at the DEM resolution, G7), NOT a true plane. Cuts HIGH spots only; does NOT fill
    cells already below target (do a follow-up ``dump`` for a true level). Leaves ``state_label``
    untouched (caller relabels EXCAVATED).
    """
    h = cs.derive_height()
    hi = mask & (h > target_m)
    mass_per_cell = np.clip((h - target_m) * cs.density, 0.0, None)
    moved = cs.cut_to_inventory(hi, mass_per_cell)
    return moved, hi


# ---------------------------------------------------------------------------
# Loading the committed coarse Haworth base bundle -> a ColumnState (+ world origin).
# The bundle stores the DERIVED heightmap, NOT datum; reconstruct datum on load.
# ---------------------------------------------------------------------------

def coarse_base_from_bundle(bundle_dir: str) -> tuple[ColumnState, dict]:
    """Load a committed INTERFACE bundle (e.g. samples/lunar_dem/haworth_10km_5m) into a
    coarse ``ColumnState``. Returns ``(base, metadata)``. ``datum`` is reconstructed:
    ``datum = heightmap - mass_areal/density`` (the bundle stores heightmap, not datum)."""
    fields, meta = load_scene(bundle_dir)
    g = meta["grid"]
    datum = fields["heightmap"].astype(np.float64) - (
        fields["mass_areal"].astype(np.float64) / fields["density"].astype(np.float64))
    base = ColumnState(
        int(g["width"]), int(g["height"]), float(g["cell_m"]),
        mass_areal=fields["mass_areal"].astype(np.float64),
        density=fields["density"].astype(np.float64),
        state_label=fields["state_label"].astype(np.uint8),
        disturbance=fields["disturbance"].astype(np.float64),
        datum=datum,
    )
    assert np.allclose(base.derive_height(), fields["heightmap"], atol=1e-3), "datum round-trip"
    return base, meta


class WorkSite:
    """Streaming coarse base + rover-following fine worked window + global drum ledger.

    Construct via :meth:`from_haworth_bundle`, then :meth:`open_window` over a base cell, then
    drive the controller seam (:meth:`drive` / :meth:`flatten` / :meth:`dump` / :meth:`relax`
    / :meth:`compact_over`). :meth:`conservation_residual` must stay tiny throughout.
    """

    def __init__(self, base: ColumnState, *, world_x0: float, world_y0: float,
                 fine_cell_m: float = 0.05, tile_base_cells: int = 4,
                 world_seed: int = 0, page_dir: str | None = None,
                 smooth_datum: bool = False):
        self.base = base
        self.base_cell_m = float(base.cell_m)
        self.fine_cell_m = float(fine_cell_m)
        self.world_x0 = float(world_x0)
        self.world_y0 = float(world_y0)
        # G7 fix (gated, default OFF): replace the piecewise-constant np.repeat DEM datum (5 m
        # terraces with ~88 deg sub-cell cliffs) with a BILINEAR resample of the coarse base datum
        # at fine resolution. Datum carries NO mass (height = datum + mass_areal/density), so this is
        # conservation-NEUTRAL by construction — grid_mass() is identical to the bit with the flag on
        # or off (verified by direct equality), NOT because any test exercises this flag (the 60-test
        # suite touches neither WorkSite nor smooth_datum). It only removes the fake terrace cliffs
        # that saturate drive slip / pollute repose / stair-step the render. The procgen overlay lives
        # in mass_areal (set_height_via_mass), so it is preserved untouched. Bilinear sampling is a
        # pure function of GLOBAL fine-cell position -> seam-free across window crops. Default OFF
        # leaves the mosaic refine/coarsen roundtrip (open_window/slice path) bit-exact.
        self.smooth_datum = bool(smooth_datum)

        # The streaming VIRGIN fine source: a read-only reader over the coarse base
        # (needs `datum`, not heightmap) wrapped in a TileMosaic.
        base_fields = {
            "mass_areal": base.mass_areal, "density": base.density,
            "datum": base.datum, "state_label": base.state_label,
            "disturbance": base.disturbance,
        }
        assert set(BASE_FIELD_NAMES) <= set(base_fields), "base fields cover BASE_FIELD_NAMES"
        self.reader = ArrayBaseReader(base_fields, self.base_cell_m,
                                      world_x0=self.world_x0, world_y0=self.world_y0)
        self.mosaic = TileMosaic(self.reader, self.base_cell_m, self.fine_cell_m,
                                 tile_base_cells=tile_base_cells, world_seed=world_seed,
                                 page_dir=page_dir)

        # Tiling params for the streaming active window (mirror the mosaic's base-tile grid).
        self.tile_base_cells = int(tile_base_cells)
        self.k = int(round(self.base_cell_m / self.fine_cell_m))   # fine cells per base cell
        self._n_tile_rows = -(-base.height // self.tile_base_cells)  # ceil
        self._n_tile_cols = -(-base.width // self.tile_base_cells)

        # Worked window state (set in open_window / recenter).
        self.fine: ColumnState | None = None
        self.window_region_rc: list[int] | None = None      # [r0,c0,r1,c1] BASE cells
        self.window_world_origin: tuple[float, float] | None = None  # global (x,y) m of window (0,0)
        self.inventory_kg: float = 0.0                      # GLOBAL drum ledger [kg]
        self.peak_inventory_kg: float = 0.0
        self._baseline_mass: float | None = None            # single-window baseline (open_window path)

        # Streaming state (recenter path): a persistent WORKED store keyed by base-tile (tr,tc),
        # the set of tiles currently assembled into `fine`, and a baseline that GROWS as fresh
        # virgin terrain first enters the worked domain (so conservation stays sensitive across moves).
        self.worked_store: dict[tuple[int, int], dict] = {}
        self.active_blocks: set[tuple[int, int]] = set()
        self.active_origin_base_rc: tuple[int, int] | None = None
        self.seen_tiles: set[tuple[int, int]] = set()
        self._baseline_virgin_kg: float = 0.0
        self.recenters: int = 0

    # -- construction --------------------------------------------------------

    @classmethod
    def from_haworth_bundle(cls, bundle_dir: str, *, fine_cell_m: float = 0.05,
                            tile_base_cells: int = 4, world_seed: int = 0,
                            page_dir: str | None = None,
                            smooth_datum: bool = False) -> "WorkSite":
        """Load the committed coarse Haworth base and build the streaming mosaic over it.
        ``world_x0/world_y0`` come from the bundle ``world_bounds_m`` (global placement).
        ``smooth_datum`` enables the G7 bilinear-datum fix (see :meth:`__init__`)."""
        base, meta = coarse_base_from_bundle(bundle_dir)
        wb = meta.get("world_bounds_m", {"x0": 0.0, "y0": 0.0})
        return cls(base, world_x0=float(wb["x0"]), world_y0=float(wb["y0"]),
                   fine_cell_m=fine_cell_m, tile_base_cells=tile_base_cells,
                   world_seed=world_seed, page_dir=page_dir, smooth_datum=smooth_datum)

    # -- streaming window ----------------------------------------------------

    def base_rc_to_xy(self, base_rc: tuple[float, float]) -> tuple[float, float]:
        """Base (row,col) -> global metres (x,y). x grows with col, y grows with row
        (dem_io / column_state row-major-C convention)."""
        br, bc = base_rc
        return (self.world_x0 + bc * self.base_cell_m, self.world_y0 + br * self.base_cell_m)

    def open_window(self, base_rc: tuple[float, float], *, radius_m: float = 8.0) -> None:
        """Materialize the fine WORKED window covering a rover disc at base cell ``base_rc``.

        Pulls VIRGIN fine terrain from the mosaic (DEM block + sourced overlay) and COPIES it
        into our own ``ColumnState`` (so mutations are isolated — the returned ``Tile.cs`` may
        alias or copy depending on dtype/contiguity). SLICE: one contiguous tile that fully
        contains the work envelope, so the cross-seam relaxation gap (G1) never fires.
        """
        rover_xy = self.base_rc_to_xy(base_rc)
        tiles = self.mosaic.ensure_fine(rover_xy, radius_m)
        if not tiles:
            raise RuntimeError(f"no fine tile materialized at base_rc={base_rc} (xy={rover_xy})")
        # Pick the tile whose base region contains base_rc (else the first).
        br, bc = base_rc
        tile = next((t for t in tiles
                     if t.region_rc[0] <= br < t.region_rc[2] and t.region_rc[1] <= bc < t.region_rc[3]),
                    tiles[0])
        fcs = tile.cs
        self.fine = ColumnState(
            int(fcs.width), int(fcs.height), float(fcs.cell_m),
            mass_areal=np.array(fcs.mass_areal, dtype=np.float64),
            density=np.array(fcs.density, dtype=np.float64),
            state_label=np.array(fcs.state_label, dtype=np.uint8),
            disturbance=np.array(fcs.disturbance, dtype=np.float64),
            datum=np.array(fcs.datum, dtype=np.float64),
        )
        r0, c0, _r1, _c1 = tile.region_rc
        if self.smooth_datum:                              # G7 fix (gated): bilinear DEM datum
            self.fine.datum = self._bilinear_datum_block(r0 * self.k, c0 * self.k,
                                                         self.fine.height, self.fine.width)
        self.window_region_rc = list(tile.region_rc)
        self.window_world_origin = (self.world_x0 + c0 * self.base_cell_m,
                                    self.world_y0 + r0 * self.base_cell_m)
        # Do NOT zero the GLOBAL ledger here: drummed mass must CARRY across windows (dig in one
        # window, dump in another — the ledger's whole purpose; contract invariant #3). It is
        # initialized once in __init__. We re-anchor the conservation epoch to the new window
        # (baseline includes any carried ledger); full cross-window grid-mass accounting needs the
        # paged worked-tile store (gap G2), so the residual is sensitive WITHIN a window.
        self._baseline_mass = self.total_mass()

    # -- controller seam (scripted now, RL policy later — identical signature) ----

    def drive(self, twists, *, start_rc, start_yaw=0.0, dt: float = 0.1,
              params=None, payload_kg: float | None = None) -> dict:
        """Drive the rover through a sequence of ``(v, omega)`` twists over the fine window,
        laying slip-deepened ruts (closed_loop_drive: conform -> slip -> step -> four_wheel_pass
        physical). Density-only, mass-conserving. Returns the closed_loop_drive telemetry dict.

        ``payload_kg`` defaults to the LIVE drum fill (``self.inventory_kg``) so hauling a loaded
        drum sinks/slips more than an empty one (the path-dependent loop); pass a value to override."""
        if payload_kg is None:
            payload_kg = self.inventory_kg
        return D.closed_loop_drive(self._require_fine(), start_rc, start_yaw, list(twists),
                                   dt=dt, params=params, payload_kg=payload_kg)

    def flatten(self, mask: np.ndarray, target_m: float, *, relabel: bool = True) -> float:
        """Cut every cell in ``mask`` above ``target_m`` down to that level, into the GLOBAL
        ledger (mass-conserving). Relabels the cut cells EXCAVATED (+disturbance) by default.
        Returns kg moved into the drum."""
        cs = self._require_fine()
        moved, hi = flatten_to_level(cs, mask, target_m)
        self.inventory_kg += cs.drum_inventory
        cs.drum_inventory = 0.0                     # window drum is a transient register
        if relabel and hi.any():
            cs.state_label[hi] = np.uint8(StateLabel.EXCAVATED)
            cs.disturbance[hi] = np.clip(cs.disturbance[hi] + 0.4, 0.0, 1.0)
        self._note_inventory()
        return moved

    def dump(self, mask: np.ndarray, kg: float | None = None, *,
             spoil_density: float = K.RHO_SPOIL) -> float:
        """Deposit spoil from the GLOBAL ledger onto ``mask`` as bulked SPOIL. ``kg=None``
        dumps the whole ledger (clamped to what's available). Returns kg actually placed."""
        cs = self._require_fine()
        if not mask.any():
            return 0.0                              # empty mask: nothing placed, ledger untouched
        want = self.inventory_kg if kg is None else float(kg)
        want = max(0.0, min(want, self.inventory_kg))
        cs.drum_inventory = want                    # prime the transient register (a scratch copy)
        placed = cs.dump_from_inventory(mask, total_kg=want, spoil_density=spoil_density)
        cs.drum_inventory = 0.0                     # discard the register; real mass is in the ledger
        self.inventory_kg -= placed                 # ledger loses exactly what landed on the grid
        return placed

    def relax(self, *, max_steps: int = 400, capture: bool = False, capture_every: int = 4,
              theta_r: float = K.THETA_R, transfer_fraction: float = 0.6) -> tuple[int, list]:
        """Sandpile-relax the fine window to angle-of-repose (mass-conserving within the grid).
        Returns ``(steps, snapshots)``; ``snapshots`` are derived-height arrays when ``capture``,
        else ``[]``. Rebuilds the Sandpile each call (it caches cell_m at construction)."""
        sp = Sandpile(self._require_fine(), theta_r=theta_r, connectivity=8,
                      transfer_fraction=transfer_fraction)
        return sp.relax_to_rest(max_steps=max_steps, capture=capture, capture_every=capture_every)

    def compact_over(self, poses, *, physical: bool = True, params=None,
                     payload_kg: float | None = None) -> dict:
        """Drive the 4-wheel footprint over ``poses`` (list of ``(center_rc, heading_rad)``),
        compacting SPOIL -> COMPACTED_BERM and laying TREAD elsewhere (density-only, mass
        conserved). ``payload_kg`` defaults to the LIVE drum fill (``self.inventory_kg``) so a loaded
        rover presses harder (heavier firming). NOTE (G5): with ``physical=True`` (default) the Bekker
        pressure-sinkage at this rover's tiny EMPTY-drum static wheel load firms spoil only
        ~1300->~1304 kg/m^3 per pass (measured) — far below the 1610 pin threshold, so the berm is
        LABELLED COMPACTED_BERM but does not yet hold slope; a standing berm needs many passes /
        explicit firming (a full drum presses harder, but the IPEx mass is small either way). (The
        legacy constant path, ``physical=False``, would apply a fixed *1.12 -> ~1456; the demo does
        not use it.) Returns wheel polylines."""
        if payload_kg is None:
            payload_kg = self.inventory_kg
        return R.four_wheel_pass(self._require_fine(), list(poses), physical=physical,
                                 params=params, payload_kg=payload_kg)

    def sinter(self, mask: np.ndarray, *, sintered_density: float = K.RHO_SINTERED) -> float:
        """Fuse the masked fine-window cells into a hard SINTERED crust (the lunar concrete/road
        analog): density rises to ``sintered_density``, height drops, state -> SINTERED, mass exactly
        conserved (the grid-mass invariant holds, ledger untouched). Wraps the tested
        ``ColumnState.sinter`` authority primitive; the energy cost is the caller's
        (``constants.SINTER_ENERGY_J_PER_KG``), not modelled here.

        GATED OFF: refuses unless ``constants.SINTER_ENABLED`` is True (the single gate, shared with
        the planner). Sinter's energy and density are [CALIB] estimates, not IPEx-grounded (IPEx has
        no sinter tool), so it is exposed as a first-class controller action but is NOT runnable until
        those numbers are sourced. Returns kg fused when enabled."""
        if not K.SINTER_ENABLED:
            raise RuntimeError(
                "WorkSite.sinter is GATED OFF: its energy/density are [CALIB], not IPEx-grounded "
                "(IPEx has no sinter tool). Ground the model against a real source, then set "
                "terrain_authority.constants.SINTER_ENABLED = True to enable.")
        return self._require_fine().sinter(mask, sintered_density=sintered_density)

    # -- streaming active window (multi-window roam; G2 worked-tile paging) --------

    def _tile_region(self, tr: int, tc: int) -> tuple[int, int, int, int]:
        """Base-cell half-open region [r0,c0,r1,c1] of base-tile (tr,tc), clipped to the base."""
        r0, c0 = tr * self.tile_base_cells, tc * self.tile_base_cells
        return (r0, c0, min(r0 + self.tile_base_cells, self.base.height),
                min(c0 + self.tile_base_cells, self.base.width))

    def _bilinear_datum_block(self, fi0: int, fj0: int, h: int, w: int) -> np.ndarray:
        """Bilinear resample of the coarse base ``datum`` for a fine block starting at GLOBAL
        fine-cell index (fi0,fj0), size (h,w). Fine cell ``fi`` maps to base coordinate
        ``(fi+0.5)/k - 0.5`` (lands on base-cell-centre node ``br`` exactly only for odd ``k``; for
        even ``k`` — e.g. the demo's k=100 — the two central fine cells straddle the node), so within-tile
        plateaus become smooth ramps and the 5 m terrace cliffs vanish. Pure function of global
        index -> identical on either side of any window/tile seam (paging-safe, conservation-neutral
        — datum carries no mass)."""
        k = self.k
        D = self.base.datum
        Hb, Wb = D.shape
        ur = np.clip((np.arange(fi0, fi0 + h) + 0.5) / k - 0.5, 0.0, Hb - 1.0)
        uc = np.clip((np.arange(fj0, fj0 + w) + 0.5) / k - 0.5, 0.0, Wb - 1.0)
        r0 = np.clip(np.floor(ur).astype(int), 0, Hb - 2); tr = (ur - r0)[:, None]
        c0 = np.clip(np.floor(uc).astype(int), 0, Wb - 2); tc = (uc - c0)[None, :]
        d00 = D[np.ix_(r0, c0)]; d01 = D[np.ix_(r0, c0 + 1)]
        d10 = D[np.ix_(r0 + 1, c0)]; d11 = D[np.ix_(r0 + 1, c0 + 1)]
        top = d00 * (1.0 - tc) + d01 * tc
        bot = d10 * (1.0 - tc) + d11 * tc
        return top * (1.0 - tr) + bot * tr

    def _virgin_tile_fields(self, tr: int, tc: int) -> dict:
        """Fresh VIRGIN fine fields for base-tile (tr,tc) from the streaming mosaic (deterministic
        regen: DEM block + sourced overlay). float64/uint8 copies — our worked store owns them.
        With ``smooth_datum`` the piecewise-constant datum is replaced by the bilinear resample
        (G7 fix); mass_areal (which carries the procgen overlay) is untouched, so the height gains
        the smooth DEM ramp while keeping the sourced sub-cell texture, mass conserved exactly."""
        ft = self.mosaic._generate_fine_tile(tr, tc)
        out = {name: np.array(ft.fields[name],
                              dtype=(np.uint8 if name == "state_label" else np.float64))
               for name in BASE_FIELD_NAMES}
        if self.smooth_datum:
            r0, c0, _r1, _c1 = self._tile_region(tr, tc)
            h, w = out["datum"].shape
            out["datum"] = self._bilinear_datum_block(r0 * self.k, c0 * self.k, h, w)
        return out

    def _tile_grid_mass(self, fields: dict) -> float:
        return float(np.asarray(fields["mass_areal"]).sum()) * (self.fine_cell_m ** 2)

    def _commit_active(self) -> None:
        """Page the CURRENT active window's worked state back into the store, per base-tile."""
        if self.fine is None or self.active_origin_base_rc is None:
            return
        br0, bc0 = self.active_origin_base_rc
        k = self.k
        for (tr, tc) in self.active_blocks:
            r0, c0, r1, c1 = self._tile_region(tr, tc)
            rr0, cc0 = (r0 - br0) * k, (c0 - bc0) * k
            th, tw = (r1 - r0) * k, (c1 - c0) * k
            self.worked_store[(tr, tc)] = {
                name: np.array(getattr(self.fine, name)[rr0:rr0 + th, cc0:cc0 + tw])
                for name in BASE_FIELD_NAMES}

    def recenter(self, rover_xy: tuple[float, float], *, radius_tiles: int = 1) -> bool:
        """Slide the active fine window to cover the rover's base-tile + ``radius_tiles`` margin,
        paging the previous active window's WORKED state into the store and assembling the new
        window from worked-or-virgin tiles. The GLOBAL ledger carries unchanged. Returns True if
        the window moved. Conserves: store commit/load is lossless (float64); newly-seen virgin
        tiles extend the baseline by their virgin mass, so the residual stays sensitive across moves.
        This is the G2 worked-tile streaming; a berm/cut spanning the advance seam is the residual
        G1 gap (mitigate by sizing radius_tiles so each dump+relax stays inside one window).

        The streaming (``recenter``) and single-window (``open_window``) entry paths are mutually
        exclusive — ``open_window`` does not register the active-tile bookkeeping ``recenter`` pages
        from, so mixing them would silently DISCARD the opened window's worked state. Guarded here."""
        if self._baseline_mass is not None:
            raise RuntimeError(
                "recenter() (streaming) cannot follow open_window() (single-window slice) on the same "
                "WorkSite — the entry paths are mutually exclusive; construct a fresh WorkSite to stream.")
        x, y = rover_xy
        bc = int(round((x - self.world_x0) / self.base_cell_m))
        br = int(round((y - self.world_y0) / self.base_cell_m))
        ctr_tr, ctr_tc = br // self.tile_base_cells, bc // self.tile_base_cells
        new_tiles = [(tr, tc)
                     for tr in range(ctr_tr - radius_tiles, ctr_tr + radius_tiles + 1)
                     for tc in range(ctr_tc - radius_tiles, ctr_tc + radius_tiles + 1)
                     if 0 <= tr < self._n_tile_rows and 0 <= tc < self._n_tile_cols]
        new_set = set(new_tiles)
        if self.fine is not None and new_set == self.active_blocks:
            return False                                    # rover still inside the window

        self._commit_active()                               # page out current worked state
        regions = {t: self._tile_region(*t) for t in new_tiles}
        br0 = min(r[0] for r in regions.values()); bc0 = min(r[1] for r in regions.values())
        br1 = max(r[2] for r in regions.values()); bc1 = max(r[3] for r in regions.values())
        k = self.k
        H, W = (br1 - br0) * k, (bc1 - bc0) * k
        out = {name: np.zeros((H, W), dtype=(np.uint8 if name == "state_label" else np.float64))
               for name in BASE_FIELD_NAMES}
        for t, (r0, c0, r1, c1) in regions.items():
            tf = self.worked_store.get(t)
            if tf is None:                                  # first visit -> virgin, extend baseline
                tf = self._virgin_tile_fields(*t)
                self._baseline_virgin_kg += self._tile_grid_mass(tf)
                self.seen_tiles.add(t)
            rr0, cc0 = (r0 - br0) * k, (c0 - bc0) * k
            th, tw = (r1 - r0) * k, (c1 - c0) * k
            for name in BASE_FIELD_NAMES:
                out[name][rr0:rr0 + th, cc0:cc0 + tw] = tf[name]

        self.fine = ColumnState(W, H, self.fine_cell_m,
                                mass_areal=out["mass_areal"], density=out["density"],
                                state_label=out["state_label"], disturbance=out["disturbance"],
                                datum=out["datum"])
        self.active_blocks = new_set
        self.active_origin_base_rc = (br0, bc0)
        self.window_region_rc = [br0, bc0, br1, bc1]
        self.window_world_origin = (self.world_x0 + bc0 * self.base_cell_m,
                                    self.world_y0 + br0 * self.base_cell_m)
        self.recenters += 1
        return True

    def active_rc_for_xy(self, xy: tuple[float, float]) -> tuple[float, float]:
        """Global metres -> (row,col) in the CURRENT active fine window (for driving/masks)."""
        x, y = xy
        ox, oy = self.window_world_origin
        return ((y - oy) / self.fine_cell_m, (x - ox) / self.fine_cell_m)

    # -- invariant + IO ------------------------------------------------------

    def total_mass(self) -> float:
        """The conserved scalar: active-window grid mass + global drum ledger + the worked store
        (tiles paged out of the active window). Store tiles that are CURRENTLY active are excluded
        (their live state is in `fine`, the store copy is stale until the next commit)."""
        total = self.inventory_kg + (self.fine.grid_mass() if self.fine is not None else 0.0)
        total += sum(self._tile_grid_mass(f) for t, f in self.worked_store.items()
                     if t not in self.active_blocks)
        return total

    def conservation_residual(self) -> float:
        """|total_mass() - baseline|. Streaming path anchors on the cumulative virgin mass that
        has entered the worked domain (``_baseline_virgin_kg``); single-window path on
        ``_baseline_mass``. Must stay < 1e-6 * baseline.

        Gate on ``_baseline_virgin_kg`` (set on the FIRST recenter), NOT ``worked_store`` (populated
        only by the SECOND recenter's commit, so it lags one window behind). Gating on worked_store
        used to return a blind 0.0 throughout the first window — a false-negative in a conservation
        guard, even though the shipped roam never checked conservation that early."""
        if self._baseline_virgin_kg:                         # streaming entry (recenter)
            return abs(self.total_mass() - self._baseline_virgin_kg)
        if self._baseline_mass is None:
            return 0.0
        return abs(self.total_mass() - self._baseline_mass)  # single-window entry (open_window)

    @property
    def over_payload(self) -> bool:
        """Whether the ledger ever exceeded the sourced 30 kg/cycle drum envelope (not
        enforced anywhere in the repo; WorkSite flags it)."""
        return self.peak_inventory_kg > K.DRUM_PAYLOAD_MAX_KG

    def save_cs_bundle(self, cs: ColumnState, scene_dir: str, world_origin: tuple[float, float],
                       *, scene_name: str = "worksite", extra: dict | None = None) -> dict:
        """Save ANY worked ``ColumnState`` (the active window, or a corridor assembled by
        :meth:`assemble_region`) as an INTERFACE bundle Godot/matplotlib consume (heightmap.rf32
        baked — terrain.gd renders the base raster, not tiles[], per G3). Returns the metadata."""
        x0, z0 = world_origin
        h = cs.derive_height()
        meta = _fine_bundle_metadata(scene_name, cs.width, cs.height, cs.cell_m, x0, z0,
                                     height_range=[float(h.min()), float(h.max())], extra=extra)
        save_scene(scene_dir, cs.fields_dict(), meta)
        return meta

    def save_fine_bundle(self, scene_dir: str, *, scene_name: str = "worksite_fine",
                         extra: dict | None = None) -> dict:
        """Save the CURRENT active fine WORKED window as a renderable bundle (delegates to
        :meth:`save_cs_bundle` at the window's world origin)."""
        cs = self._require_fine()
        return self.save_cs_bundle(cs, scene_dir, self.window_world_origin or (0.0, 0.0),
                                   scene_name=scene_name, extra=extra)

    # -- corridor assembly (worked_store U live-active U virgin context) -----------

    def _active_tile_fields(self, tr: int, tc: int) -> dict:
        """LIVE worked fields of base-tile (tr,tc) sliced out of the CURRENT active window
        (un-committed) — mirrors :meth:`_commit_active`'s indexing."""
        br0, bc0 = self.active_origin_base_rc
        k = self.k
        r0, c0, r1, c1 = self._tile_region(tr, tc)
        rr0, cc0 = (r0 - br0) * k, (c0 - bc0) * k
        th, tw = (r1 - r0) * k, (c1 - c0) * k
        return {name: np.array(getattr(self.fine, name)[rr0:rr0 + th, cc0:cc0 + tw])
                for name in BASE_FIELD_NAMES}

    def visited_base_bbox(self) -> tuple[int, int, int, int]:
        """Base-cell bbox [r0,c0,r1,c1] over every tile ever visited (seen U active U stored)."""
        visited = self.seen_tiles | self.active_blocks | set(self.worked_store)
        if not visited:
            raise RuntimeError("no tiles visited yet (recenter/open_window first)")
        regions = [self._tile_region(*t) for t in visited]
        return (min(r[0] for r in regions), min(r[1] for r in regions),
                max(r[2] for r in regions), max(r[3] for r in regions))

    def visited_world_bbox(self) -> tuple[float, float, float, float]:
        """Global-metre bbox (x0,y0,x1,y1) of the worked corridor (for the site-context panel)."""
        br0, bc0, br1, bc1 = self.visited_base_bbox()
        return (self.world_x0 + bc0 * self.base_cell_m, self.world_y0 + br0 * self.base_cell_m,
                self.world_x0 + bc1 * self.base_cell_m, self.world_y0 + br1 * self.base_cell_m)

    def assemble_region(self, tiles: "set | None" = None, *, fill_virgin: bool = True
                        ) -> tuple[ColumnState, tuple[float, float]]:
        """Stitch the worked corridor into ONE fine ``ColumnState`` over the bbox of ``tiles``
        (default: every tile ever visited). For each base-tile in that bbox the source is, in
        order: the LIVE active window (un-committed work), else the paged worked store, else —
        when ``fill_virgin`` — a freshly regenerated VIRGIN tile (context between pad and berm).

        Returns ``(corridor_cs, world_origin_xy)``. READ-ONLY: does not touch ``worked_store`` /
        baseline / the active window. Allocates the full bbox (heavy for a long corridor); the
        renderer downsamples for display. Float64 throughout — lossless vs the live state."""
        visited = self.seen_tiles | self.active_blocks | set(self.worked_store)
        tileset = set(tiles) if tiles is not None else visited
        if not tileset:
            raise RuntimeError("assemble_region: nothing visited yet")
        regions = {t: self._tile_region(*t) for t in tileset}
        br0 = min(r[0] for r in regions.values()); bc0 = min(r[1] for r in regions.values())
        br1 = max(r[2] for r in regions.values()); bc1 = max(r[3] for r in regions.values())
        k = self.k
        H, W = (br1 - br0) * k, (bc1 - bc0) * k
        out = {name: np.zeros((H, W), dtype=(np.uint8 if name == "state_label" else np.float64))
               for name in BASE_FIELD_NAMES}
        tbc = self.tile_base_cells
        for tr in range(br0 // tbc, -(-br1 // tbc)):           # ceil on the high end
            for tc in range(bc0 // tbc, -(-bc1 // tbc)):
                t = (tr, tc)
                if t in self.active_blocks and self.fine is not None:
                    tf = self._active_tile_fields(tr, tc)
                elif t in self.worked_store:
                    tf = self.worked_store[t]
                elif fill_virgin:
                    tf = self._virgin_tile_fields(tr, tc)
                else:
                    continue
                r0, c0, r1, c1 = self._tile_region(tr, tc)
                rr0, cc0 = (r0 - br0) * k, (c0 - bc0) * k
                th, tw = (r1 - r0) * k, (c1 - c0) * k
                for name in BASE_FIELD_NAMES:
                    out[name][rr0:rr0 + th, cc0:cc0 + tw] = tf[name][:th, :tw]
        corridor = ColumnState(W, H, self.fine_cell_m,
                               mass_areal=out["mass_areal"], density=out["density"],
                               state_label=out["state_label"], disturbance=out["disturbance"],
                               datum=out["datum"])
        origin = (self.world_x0 + bc0 * self.base_cell_m, self.world_y0 + br0 * self.base_cell_m)
        return corridor, origin

    def snapshot(self) -> dict:
        """Capture the fine window's mutable state for one demo frame (deep copies)."""
        cs = self._require_fine()
        return {
            "height": cs.derive_height(),
            "mass_areal": np.array(cs.mass_areal),
            "density": np.array(cs.density),
            "state_label": np.array(cs.state_label),
            "disturbance": np.array(cs.disturbance),
            "inventory_kg": self.inventory_kg,
            "residual_kg": self.conservation_residual(),
        }

    # -- internals -----------------------------------------------------------

    def _require_fine(self) -> ColumnState:
        if self.fine is None:
            raise RuntimeError("open_window(...) must be called before working the site")
        return self.fine

    def _note_inventory(self) -> None:
        self.peak_inventory_kg = max(self.peak_inventory_kg, self.inventory_kg)


def _fine_bundle_metadata(scene_name: str, W: int, H: int, cell_m: float,
                          world_x0: float, world_z0: float, *,
                          height_range=None, active_zone=None, extra=None) -> dict:
    """Minimal VALID v1.0 INTERFACE metadata for a standalone fine work-zone bundle
    (the renderable artifact). ``save_scene`` hard-requires grid.width/grid.height; the rest
    is the contract surface Godot + the panels read."""
    x1 = round(world_x0 + W * cell_m, 4)
    z1 = round(world_z0 + H * cell_m, 4)
    meta = {
        "schema_version": "1.0",
        "scene_name": scene_name,
        "producer": "terrain_authority WorkSite (streaming fine window)",
        "grid": {"width": int(W), "height": int(H), "cell_m": float(cell_m), "order": "row-major-C"},
        "world_bounds_m": {"x0": round(world_x0, 4), "y0": round(world_z0, 4), "x1": x1, "y1": z1},
        "gravity_m_s2": K.g,
        "fields": {
            "heightmap":   {"file": "heightmap.rf32",   "dtype": "<f4", "units": "m"},
            "mass_areal":  {"file": "mass_areal.rf32",  "dtype": "<f4", "units": "kg/m^2"},
            "density":     {"file": "density.rf32",     "dtype": "<f4", "units": "kg/m^3"},
            "disturbance": {"file": "disturbance.rf32", "dtype": "<f4", "units": "1 (normalized)"},
            "state_label": {"file": "state_label.r8",   "dtype": "u1",  "enum": list(K.STATE_NAMES)},
        },
        "ice_present": False,
        "height_range_m": height_range or [0.0, 0.0],
        "clasts": [],
        "active_zone": active_zone or {"min_rc": [0, 0], "max_rc": [int(H), int(W)]},
        "quadtree": [{"level": 0, "row0": 0, "col0": 0, "size": int(max(W, H)), "label": "ROOT"}],
        "notes": "WorkSite fine work-zone (streaming window over the Haworth coarse base)",
    }
    if extra:
        meta.update(extra)
    return meta
