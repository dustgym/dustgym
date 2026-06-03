# WorkSite contract — streaming coarse-base + rover-following fine window

**Status:** draft for review (2026-06-02). Built against API ground-truth verified by
introspection (see `terrain_authority/{tiles_mosaic,refinement,dem_io,column_state,
sandpile,rover,drive,slip}.py`). This is the seam the scripted flatten→haul→berm demo
runs on *now*, shaped so an RL policy drives the **same** seam later.

## Why this exists

A >200 m Haworth site cannot be a uniform fine grid (10k² cells), and a ~0.3 m berm is
sub-cell at the 5 m DEM base — so the berm **physics has to live in a fine window**, while
the large site stays coarse. `WorkSite` is the world model that holds both and routes
cut/dump/sandpile/drive through the fine window with a **global** drum ledger, so the rover
can dig in one place and dump in another and mass carries across.

The **controller is the only stub**: today a scripted trajectory calls
`drive()/flatten()/dump()/relax()`; tomorrow an RL policy calls the identical methods. The
world model is genuinely streaming-dynamic from day one.

## Layered model

```
 controller (NOW: scripted twists+events │ LATER: RL policy.step) 
        │ identical calls
        ▼
 WorkSite.drive() / .flatten() / .dump() / .relax() / .compact_over()
        │
        ├── base   : coarse Haworth ColumnState (5 m), READ-ONLY virgin authority
        ├── mosaic : TileMosaic over the base — streaming VIRGIN fine-terrain source
        │            (+ page_dir persistence of worked tiles; LRU evict of unworked)
        ├── fine   : the rover-following WORKED fine ColumnState window
        │            (authoritative worked store — our own copy, not an aliased Tile.cs)
        └── ledger : inventory_kg — GLOBAL drum mass, carries across windows
```

## Interface (v0 — slice surface)

```python
class WorkSite:
    # --- construction -----------------------------------------------------
    @classmethod
    def from_haworth_bundle(cls, bundle_dir, *, fine_cell_m=0.05,
                            tile_base_cells=4, world_seed=0, page_dir=None) -> "WorkSite"
        # load committed coarse Haworth base (io_fields.load_scene; reconstruct datum),
        # wrap in dem_io.ArrayBaseReader, build TileMosaic (the streaming fine source).

    # --- streaming window -------------------------------------------------
    def open_window(self, base_rc, *, radius_m) -> None
        # materialize the rover-following fine WORKED window covering a rover disc,
        # pulling VIRGIN fine terrain from the mosaic and COPYING it into our own
        # ColumnState (avoids the ensure_fine alias/copy ambiguity). Records the
        # window's base-cell origin + global metre origin for placement.
        # SLICE: one-shot single tile covering the work envelope.
        # DEFERRED (streaming, see Gaps): multi-tile stitch + worked-tile paging +
        # cross-seam relaxation as the rover leaves the envelope.

    # --- controller seam (scripted now, RL later — identical signature) ---
    def drive(self, twists, *, start_rc, start_yaw, dt=0.1, params=None,
              payload_kg=0.0) -> dict          # closed_loop_drive over `fine`; lays slip-ruts
    def flatten(self, mask, target_m, *, relabel=True) -> float   # cut high spots -> ledger
    def dump(self, mask, kg=None, *, spoil_density=K.RHO_SPOIL) -> float  # ledger -> SPOIL
    def relax(self, *, max_steps=400, capture=False, capture_every=4,
              theta_r=K.THETA_R) -> tuple[int, list]   # sandpile `fine` to repose
    def compact_over(self, poses, *, physical=True) -> dict        # four_wheel_pass -> COMPACTED_BERM

    # --- invariant + IO ---------------------------------------------------
    def total_mass(self) -> float            # fine.grid_mass() + inventory_kg (the conserved scalar)
    def conservation_residual(self) -> float # |total_mass() - baseline| ; baseline set at open_window
    def save_fine_bundle(self, scene_dir) -> dict   # save_scene of `fine` (Godot reads heightmap.rf32)
    def snapshot(self) -> dict               # {height, mass_areal, density, state_label, inventory_kg}
```

Coordinates: physics works in the **fine window's local cell frame** (`row,col`,
`x=col*fine_cell_m`). `open_window` records `world_origin_m` + `base_rc` so the overhead
context panel and Godot bundle can be placed in the global Haworth frame. The fine bundle
is saved with its **own** local `world_bounds_m`; Godot renders its baked heightmap.

## Invariants (load-bearing)

1. **Mass conservation.** `fine.grid_mass() + inventory_kg` is invariant across
   `flatten/dump/relax/drive/compact_over` (all are mass-conserving or move grid↔ledger).
   `conservation_residual()` must stay < 1e-6·baseline. The coarse `base` is never mutated
   by physics — it is the virgin source only.
2. **Worked store is ours.** `fine` is a deep copy of the mosaic-sourced virgin terrain;
   we never rely on `ensure_fine`'s returned `Tile.cs` as the mutable store (it may alias
   or copy depending on dtype/contiguity — verified).
3. **Global ledger.** Per-window `drum_inventory` is a transient register (harvest→zero on
   cut; prime→reclaim on dump); the durable mass-in-drums lives in `inventory_kg`.
4. **Height is derived.** Never stored; `height = datum + mass_areal/density`. Bundles save
   the derived `heightmap`; the coarse base store (`dem_io`) saves `datum`.
5. **Drum payload envelope** (`DRUM_PAYLOAD_MAX_KG = 30 kg`) is **not enforced anywhere** in
   the repo — `WorkSite` enforces/flags it on `inventory_kg` itself.

## Documented deferred gaps (cite, don't hand-wave)

- **G1 — Cross-seam relaxation.** `Sandpile` operates on one `ColumnState`; a berm straddling
  a window/tile seam relaxes independently on each side (seam = invisible wall). *Slice
  mitigation:* the fine window fully contains the work envelope (+margin), so the gap never
  fires. *Streaming fix (deferred):* halo/ghost-cell exchange or a single window that grows
  with the worked region.
- **G2 — Worked-tile streaming.** Multi-tile streaming with `page_dir` persistence (worked
  tiles paged to float32 rasters, ~1e-5 rel mass loss; unworked tiles regenerate from base)
  is designed-for but not exercised by the slice (single window). The float32 paging breaks
  *exact* conservation by ~1e-5 — acceptable, flagged.
- **G3 — Godot fine detail.** `terrain.gd` renders only the base `heightmap.rf32`, not
  `tiles[]`. We bake the worked fine window into the bundle heightmap the camera reads;
  the coarse >200 m Haworth is overhead/context only, not the Godot far-field.
- **G4 — Drum-arm pose.** Sidecar forces both arms *up* (1.15 rad) in every camera mode; no
  per-render arm-down articulation without a sidecar code change. Camera panel shows the
  rover, not arms-mid-dig — captioned honestly.
- **G5 — Berm pinning.** A berm holds slope only when `label==COMPACTED_BERM` **and**
  `density≥1610`. With the **physical** Bekker path (`compact_over(physical=True)`, the default) at
  this 30 kg rover's tiny static wheel load, one pass firms spoil only ~1300→~1310 kg/m³ (measured:
  slice 1306–1315, roam 1302–1304) — nowhere near the pin, so the berm is labelled but does not hold
  slope. (The legacy *constant* path `physical=False` applies a fixed ×1.12 → ~1456; the demos don't
  use it.) Multi-pass / explicit firming required for a standing berm; otherwise it relaxes toward
  repose (the showpiece).
- **G6 — Emergent routing.** The bootstrap route is scripted. Emergent (RL-chosen) routing
  needs G1+G2 (true per-pose streaming). The seam is RL-ready; the policy is the missing part.
- **G7 — DEM terracing.** `refine_field` is piecewise-constant `np.repeat`, so a 5 m DEM
  refined to cm cells becomes 5 m terraces with near-vertical sub-cell cliffs. Consequences,
  all verified: (a) the **rendered** heightmap shows terraces; (b) **flatten** floors trace the
  terraced datum, not a plane; (c) **relax** can never satisfy a *window-wide* repose criterion
  (virgin terraces pin global max-loose-slope ~83° — the worked berm itself *does* reach repose,
  measured at θ_r). *Fix (scale-up):* bilinear-smooth the datum at fine res for the worked/rendered
  window, gated behind a param so default `coarsen(refine(x))==x` conservation stays bit-exact;
  sub-5 m relief is unrecoverable (Product-78 native 5 m), only the faked discontinuity is removed.
- **G8 — Datum-floor dig limit.** The fine window carries only the loose mantle (~`Z_T`=0.12 m)
  above a firm DEM datum; `cut_to_inventory` clamps removal at available mass, so you cannot
  excavate **below** datum in the current single-mantle model — a requested 0.30 m pad strips
  ~0.12 m to datum. Deeper pits need a deeper removable column (a density-profile model change),
  flagged for the scale-up, not silently papered over. The slice reports *achieved* drop, never
  the requested depth.

Ledger note: `open_window` does **not** zero the global ledger (drummed mass carries across
windows — invariant #3); it re-anchors the conservation epoch to the new window. Full
cross-window grid-mass accounting needs the G2 paged worked-tile store, so the residual is
sensitive *within* a window. `compact_over` relies on `four_wheel_pass` relabeling the **union**
of all four wheels against a single pre-pass SPOIL snapshot (fixed) so overlapping wheels can't
clobber fresh COMPACTED_BERM back to TREAD. Bulking (cut-dense/dump-loose → swell) is **not**
exercised on the uniform-1300 Haworth patch where `RHO_SPOIL==RHO_SURFACE`; the dump is iso-density.

## RL-readiness

`drive()` integrates one twist exactly as `RoverSimEnv.step` does (both call
`drive.drive_step`). Swapping the scripted driver for a policy is a controller swap, not a
world-model change. A `RoverSimEnv`-over-`WorkSite` adapter is step 4 of the build order
(prove the calls match; no training).
