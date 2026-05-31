"""Sample-scene builder/exporter.

    python -m terrain_authority.scenes

Builds and exports the weekend-slice sample scenes into <root>/samples/ on a 256x256 grid
at cell_m=0.02 (~5.12 m square patch, spec §4 active-zone 1-3 cm anchor). Every scene gets
a full metadata.json (INTERFACE.md §5): grid, world_bounds, gravity, fields, clasts,
active_zone, and quadtree (ROOT + at least one ACTIVE node over the interesting region) so
downstream D1b wireframes and the Godot loader work unchanged.

Scenes:
    flat_compact/    flat, dense, low-disturbance (low-albedo proxy).
    rolling_hills/   fbm fluffy hills, loose top.
    crater/          one Pike-class crater + ejecta.
    boulder_field/   rolling terrain + Golombek clasts in metadata (k=0.1).
    crater_caveins/  TIME SERIES t000..t0NN: a crater wall over-steepened by deposit(),
                     then relax_to_rest() snapshots — the cave-in showpiece.
    tread_track/     TIME SERIES t000..t0NN: a rover wheel footprint advanced along a
                     2-segment path, laying a VIRGIN->TREAD compaction trail incrementally
                     (path-dependent terrain change). Mass conserved (pure compaction).
"""

from __future__ import annotations

import os

import numpy as np

from . import constants as K
from . import procgen
from . import refinement
from .column_state import ColumnState
from .io_fields import save_scene, write_hillshade_png, write_preview_png
from .quadtree import QuadtreeTracker
# W2-SCENES (serial join): the DEM corridor stack. Imported here so build_from_dem can wire
# the four Wave-2 generators (density / craters / illumination / mosaic) WITHOUT touching the
# nine legacy builders below. A missing committed DEM scene degrades gracefully in main().
from . import dem_import, dem_io, dem_overlay, illumination, tiles_mosaic
from .io_fields import load_scene
from .rover import (build_drum_marks_meta, build_wheel_tracks_meta, drum_pass,
                    four_wheel_pass, straight_path, wheel_pass)
from .sandpile import Sandpile

# Grid (INTERFACE.md §5 example / spec §4 resolution anchors).
WIDTH = 256
HEIGHT = 256
CELL_M = 0.02  # 2 cm -> 5.12 m patch

# Interaction-keyed quadtree config for the driven-rover series (quadtree.py; spec §4).
# WIDTH==HEIGHT==256 is a power of two so the tree bottoms out cleanly at QT_MIN_LEAF.
QT_MIN_LEAF = 8                       # finest leaf side [cells] = 16 cm (wheel scale)
QT_REFINE_FACTOR = 0.5                # subdivide while box-dist < factor*node_size cells
QT_FOOTPRINT_RADIUS_CELLS = 5.5       # ~22 cm wheel contact half-width at 0.02 m/cell

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SAMPLES_DIR = os.path.join(ROOT, "samples")


def _base_metadata(scene_name: str, *, clasts=None, active_zone=None, quadtree=None,
                   notes: str = "", ice_present: bool = False,
                   height_range=None, extra=None) -> dict:
    x1 = WIDTH * CELL_M
    y1 = HEIGHT * CELL_M
    meta = {
        "schema_version": "1.0",
        "scene_name": scene_name,
        "producer": "terrain_authority (NumPy Tier-2 surrogate)",
        "grid": {"width": WIDTH, "height": HEIGHT, "cell_m": CELL_M, "order": "row-major-C"},
        "world_bounds_m": {"x0": 0.0, "y0": 0.0, "x1": round(x1, 4), "y1": round(y1, 4)},
        "gravity_m_s2": K.g,
        "fields": {
            "heightmap": {"file": "heightmap.rf32", "dtype": "<f4", "units": "m"},
            "mass_areal": {"file": "mass_areal.rf32", "dtype": "<f4", "units": "kg/m^2"},
            "density": {"file": "density.rf32", "dtype": "<f4", "units": "kg/m^3"},
            "disturbance": {"file": "disturbance.rf32", "dtype": "<f4", "units": "1 (normalized)"},
            "state_label": {"file": "state_label.r8", "dtype": "u1",
                            "enum": K.STATE_NAMES},
        },
        "ice_present": ice_present,
        "height_range_m": height_range if height_range is not None else [0.0, 0.0],
        "clasts": clasts if clasts is not None else [],
        "active_zone": active_zone if active_zone is not None
                       else {"min_rc": [64, 64], "max_rc": [192, 192]},
        "quadtree": quadtree if quadtree is not None else _default_quadtree(),
        "notes": notes,
    }
    if extra:
        meta.update(extra)
    return meta


def _default_quadtree(active_row0=64, active_col0=64, active_size=128):
    """ROOT + one ACTIVE node over the interesting region (INTERFACE.md §5)."""
    return [
        {"level": 0, "row0": 0, "col0": 0, "size": WIDTH, "label": "ROOT"},
        {"level": 1, "row0": active_row0, "col0": active_col0, "size": active_size,
         "label": "ACTIVE"},
    ]


def _attach_quadtree_meta(meta: dict, qt_result, rover_rc, touched_boxes) -> None:
    """ADDITIVELY attach the per-frame interaction-keyed quadtree state (INTERFACE.md §5.1).

    Adds NEW optional keys ONLY; never touches existing rasters or metadata keys (the static
    ``quadtree`` D1b key, fields, grid, ... are all left as-is). Consumers may ignore these:

      active_leaves   [[r0,c0,r1,c1],...]  fine (min_leaf) leaves under the CURRENT rover
                                           footprint (promote+evict; the live hot set).
      quadtree_nodes  [{level,row0,col0,size,leaf},...]  the full subdivision for this frame
                                           (coarse far, fine near — the LOD context).
      touched_leaves  [[r0,c0,r1,c1],...]  cumulative min_leaf cells the rover had activated
                                           AS OF THIS FRAME (promote-only history / the
                                           refined trail behind the rover; empty pre-drive).
      rover_rc        [row,col] or null    the rover footprint center this frame is keyed to.
      quadtree_lod    {min_leaf, refine_factor, footprint_radius_cells}  the promotion knobs.
    """
    meta["active_leaves"] = qt_result.boxes("active")
    meta["quadtree_nodes"] = qt_result.nodes
    meta["touched_leaves"] = touched_boxes
    meta["rover_rc"] = list(rover_rc) if rover_rc is not None else None
    meta["quadtree_lod"] = {
        "min_leaf": qt_result.min_leaf,
        "refine_factor": QT_REFINE_FACTOR,
        "footprint_radius_cells": QT_FOOTPRINT_RADIUS_CELLS,
        "field_size": qt_result.field_size,
    }


def _height_range(cs: ColumnState) -> list[float]:
    h = cs.derive_height()
    return [round(float(h.min()), 5), round(float(h.max()), 5)]


def _write_previews(scene_dir: str, cs: ColumnState, name: str) -> None:
    h = cs.derive_height()
    write_hillshade_png(h, os.path.join(scene_dir, "preview_hillshade.png"),
                        CELL_M, altdeg=K.SUN_ELEVATION_DEG_POLAR,
                        title=f"{name} hillshade (grazing sun {K.SUN_ELEVATION_DEG_POLAR}deg)")
    write_preview_png(h, os.path.join(scene_dir, "preview_height.png"),
                      cmap="terrain", title=f"{name} height [m]")
    write_preview_png(cs.state_label, os.path.join(scene_dir, "preview_state.png"),
                      cmap="tab10", title=f"{name} state_label")
    write_preview_png(cs.disturbance, os.path.join(scene_dir, "preview_disturbance.png"),
                      cmap="magma", title=f"{name} disturbance")


# ---------------------------------------------------------------------------
# Scene builders
# ---------------------------------------------------------------------------

def build_flat_compact() -> None:
    name = "flat_compact"
    cs = procgen.flat_compact(WIDTH, HEIGHT, CELL_M, seed=2)
    scene_dir = os.path.join(SAMPLES_DIR, name)
    meta = _base_metadata(
        name, height_range=_height_range(cs),
        notes="Flat dense compacted plate; low-albedo proxy via high compaction + low "
              "disturbance (spec §9, §8). Sun elevation 7deg (grazing).")
    save_scene(scene_dir, cs.fields_dict(), meta)
    _write_previews(scene_dir, cs, name)
    print(f"  wrote {name}  total_mass={cs.total_mass():.3f} kg")


def build_rolling_hills() -> None:
    name = "rolling_hills"
    cs = procgen.rolling_hills(WIDTH, HEIGHT, CELL_M, seed=11, amplitude_m=0.18)
    scene_dir = os.path.join(SAMPLES_DIR, name)
    meta = _base_metadata(
        name, height_range=_height_range(cs),
        notes="fbm fluffy rolling hills, low-density loose top (spec §9 loose-over-dense).")
    save_scene(scene_dir, cs.fields_dict(), meta)
    _write_previews(scene_dir, cs, name)
    print(f"  wrote {name}  total_mass={cs.total_mass():.3f} kg")


def build_crater() -> None:
    name = "crater"
    cs = procgen.rolling_hills(WIDTH, HEIGHT, CELL_M, seed=3, amplitude_m=0.05,
                               base_cells=2)
    diameter_m = 2.4  # ~half the patch
    procgen.carve_crater(cs, (HEIGHT // 2, WIDTH // 2), diameter_m)
    scene_dir = os.path.join(SAMPLES_DIR, name)
    # Active zone over the crater bowl.
    R_cells = int(0.5 * diameter_m / CELL_M)
    cr, cc = HEIGHT // 2, WIDTH // 2
    az = {"min_rc": [max(0, cr - R_cells), max(0, cc - R_cells)],
          "max_rc": [min(HEIGHT, cr + R_cells), min(WIDTH, cc + R_cells)]}
    qt = _default_quadtree(active_row0=max(0, cr - R_cells),
                           active_col0=max(0, cc - R_cells),
                           active_size=2 * R_cells)
    meta = _base_metadata(
        name, active_zone=az, quadtree=qt, height_range=_height_range(cs),
        notes=f"Single fresh simple (Pike-class) crater, D={diameter_m} m, depth/D="
              f"{K.CRATER_DEPTH_DIAMETER_RATIO}, rim + ejecta. Mass-consistent carve.")
    save_scene(scene_dir, cs.fields_dict(), meta)
    _write_previews(scene_dir, cs, name)
    print(f"  wrote {name}  total_mass={cs.total_mass():.3f} kg")


def build_boulder_field() -> None:
    name = "boulder_field"
    cs = procgen.rolling_hills(WIDTH, HEIGHT, CELL_M, seed=21, amplitude_m=0.12)
    clasts = procgen.sample_boulders(WIDTH, HEIGHT, CELL_M, k=0.1, seed=42)
    scene_dir = os.path.join(SAMPLES_DIR, name)
    meta = _base_metadata(
        name, clasts=clasts, height_range=_height_range(cs),
        notes=f"Rolling terrain + Golombek SFD clasts (k=0.1, q={K.golombek_q(0.1):.3f}); "
              f"{len(clasts)} clasts. rock-size-freq_abstract.txt. Clasts are metadata "
              f"refs (uncovered -> Chrono rigid bodies, spec §6); not carved into mass.")
    save_scene(scene_dir, cs.fields_dict(), meta)
    _write_previews(scene_dir, cs, name)
    print(f"  wrote {name}  total_mass={cs.total_mass():.3f} kg  clasts={len(clasts)}")


def build_crater_boulders() -> None:
    """A crater AND a Golombek boulder field in one scene (the GMRO 'craters + boulders' ask).

    Boulders are sampled from the same Golombek SFD as build_boulder_field, then (a) excluded
    from the fresh crater bowl (a freshly excavated bowl wouldn't have rocks resting in it) and
    (b) snapped to the local terrain surface so they sit on the regolith rather than floating at
    the y=0 reference the bare sampler uses.
    """
    name = "crater_boulders"
    cs = procgen.rolling_hills(WIDTH, HEIGHT, CELL_M, seed=17, amplitude_m=0.06, base_cells=3)
    diameter_m = 2.2
    cr, cc = HEIGHT // 2, WIDTH // 2
    procgen.carve_crater(cs, (cr, cc), diameter_m)

    R = 0.5 * diameter_m
    cx, cz = cc * CELL_M, cr * CELL_M
    h = cs.derive_height()
    raw = procgen.sample_boulders(WIDTH, HEIGHT, CELL_M, k=0.08, seed=71)
    clasts: list[dict] = []
    for c in raw:
        x, _y, z = c["center_m"]
        if np.hypot(x - cx, z - cz) < 0.95 * R:
            continue  # no boulders floating in the fresh bowl
        col = min(WIDTH - 1, max(0, int(round(x / CELL_M))))
        row = min(HEIGHT - 1, max(0, int(round(z / CELL_M))))
        rad = c["radius_m"]
        buried = c["buried_frac"]
        # Rest the partially-buried sphere on the surface: center = surface + r(1 - 2*buried).
        c["center_m"] = [round(x, 4), round(float(h[row, col]) + rad * (1.0 - 2.0 * buried), 4),
                         round(z, 4)]
        c["id"] = len(clasts)
        clasts.append(c)

    scene_dir = os.path.join(SAMPLES_DIR, name)
    R_cells = int(R / CELL_M)
    az = {"min_rc": [max(0, cr - R_cells), max(0, cc - R_cells)],
          "max_rc": [min(HEIGHT, cr + R_cells), min(WIDTH, cc + R_cells)]}
    qt = _default_quadtree(active_row0=max(0, cr - R_cells),
                           active_col0=max(0, cc - R_cells), active_size=2 * R_cells)
    meta = _base_metadata(
        name, clasts=clasts, active_zone=az, quadtree=qt, height_range=_height_range(cs),
        notes=f"Pike-class crater (D={diameter_m} m) + Golombek SFD boulder field "
              f"(k=0.08, q={K.golombek_q(0.08):.3f}); {len(clasts)} clasts, surface-snapped and "
              f"excluded from the fresh bowl. Craters + boulders together (spec §6, §9).")
    save_scene(scene_dir, cs.fields_dict(), meta)
    _write_previews(scene_dir, cs, name)
    print(f"  wrote {name}  total_mass={cs.total_mass():.3f} kg  clasts={len(clasts)}")


def build_crater_caveins() -> None:
    """TIME SERIES: over-steepen a crater rim, then relax to rest. The cave-in showpiece."""
    name = "crater_caveins"
    cs = procgen.rolling_hills(WIDTH, HEIGHT, CELL_M, seed=5, amplitude_m=0.04,
                               base_cells=2)
    diameter_m = 2.0
    cr, cc = HEIGHT // 2, WIDTH // 2
    procgen.carve_crater(cs, (cr, cc), diameter_m)

    # Over-steepen one wall: pile loose spoil on the inner-north rim so its slope into the
    # bowl far exceeds repose. deposit() raises grid mass; we record that as the t000
    # (pre-collapse) reference total so conservation across the relax is checkable.
    R_cells = int(0.5 * diameter_m / CELL_M)
    pile_r = cr - int(0.55 * R_cells)
    pile_c = cc
    sp = Sandpile(cs, theta_r=K.THETA_R, connectivity=8, transfer_fraction=0.6)
    # Add a tall, narrow loose ridge -> guaranteed over-repose -> avalanche into the bowl.
    sp.deposit(pile_r, pile_c, mass_kg=120.0, radius_cells=6)

    mass_before = cs.total_mass()

    # Relax to rest, capturing the cave-in frame by frame.
    steps, snaps = sp.relax_to_rest(max_steps=400, capture=True, capture_every=4)
    mass_after = cs.total_mass()

    scene_dir = os.path.join(SAMPLES_DIR, name)
    os.makedirs(scene_dir, exist_ok=True)

    # We export the relaxation as a series of full snapshots t000..t0NN. To keep each frame
    # a faithful ColumnState (mass/density consistent), re-run the relaxation determinist
    # capturing a full ColumnState clone per captured frame.
    frames = _replay_caveins(diameter_m, cr, cc)
    cadence = 4
    for i, frame_cs in enumerate(frames):
        tdir = os.path.join(scene_dir, f"t{i:03d}")
        az = {"min_rc": [max(0, cr - R_cells), max(0, cc - R_cells)],
              "max_rc": [min(HEIGHT, cr + R_cells), min(WIDTH, cc + R_cells)]}
        qt = _default_quadtree(active_row0=max(0, cr - R_cells),
                               active_col0=max(0, cc - R_cells),
                               active_size=2 * R_cells)
        meta = _base_metadata(
            name, active_zone=az, quadtree=qt, height_range=_height_range(frame_cs),
            notes=f"cave-in frame {i}/{len(frames)-1}; over-steepened crater rim "
                  f"relaxing to repose theta_r={np.rad2deg(K.THETA_R):.0f}deg (spec §7).")
        meta["frame_index"] = i
        save_scene(tdir, frame_cs.fields_dict(), meta)
    # hillshade + state previews for the first and last frame at the parent level.
    _write_previews(os.path.join(scene_dir, "t000"), frames[0], name + "_t000")
    _write_previews(os.path.join(scene_dir, f"t{len(frames)-1:03d}"), frames[-1],
                    name + f"_t{len(frames)-1:03d}")

    # Parent metadata documents the time-series cadence/count (INTERFACE.md §1).
    parent_meta = _base_metadata(
        name, height_range=_height_range(frames[-1]),
        notes="TIME SERIES (cave-in). A loose ridge piled on the inner crater rim with "
              "deposit() is relaxed to angle-of-repose by the sandpile CA (spec §7). "
              "Each tNNN/ is a full snapshot; mass conserved across the series.")
    parent_meta["time_series"] = {
        "frame_count": len(frames),
        "frame_cadence_steps": cadence,
        "frame_dirs": [f"t{i:03d}" for i in range(len(frames))],
        "mass_conserved_kg": round(mass_after, 6),
        "mass_drift_kg": round(abs(mass_after - mass_before), 9),
    }
    import json
    with open(os.path.join(scene_dir, "metadata.json"), "w") as fh:
        json.dump(parent_meta, fh, indent=2)
    print(f"  wrote {name}  frames={len(frames)}  steps={steps}  "
          f"mass_before={mass_before:.4f} mass_after={mass_after:.4f} kg "
          f"drift={abs(mass_after-mass_before):.2e} kg")


def build_tread_track() -> None:
    """TIME SERIES: a rover drives a 2-segment path, laying down a compaction tread trail.

    The headline "path-dependent terrain change" capability (README §4 row #3, §5 bullet 2):
    a wheel footprint is advanced along the path and ``rover.wheel_pass`` is applied
    INCREMENTALLY, one path-chunk per frame, so the track is laid down progressively over
    time. Each frame is a full contract scene (tNNN/). Over the series you watch, ALONG the
    wheel track only: VIRGIN -> TREAD relabel, density rising toward RHO_DEEP (compaction),
    the surface dipping slightly (the rut, because height = datum + mass/density and mass is
    untouched, so a denser column is thinner — spec §6), and a disturbance bump.

    MASS is CONSERVED across the whole track: wheel_pass is pure compaction (density-only
    redistribution capped at RHO_DEEP), it never removes or adds grid mass. The drum
    inventory is never touched here. So total_mass(first) == total_mass(last) to float64
    round-off (asserted/printed below; rover.py docstring + spec §6).
    """
    name = "tread_track"
    cr0, cc0, cr1, cc1, cr2, cc2 = _tread_path_endpoints()

    frames, mass_before, mass_after = _replay_tread_track()

    # Per-frame rover footprint center + the interaction-keyed quadtree that FOLLOWS it
    # (quadtree.py; spec §4). The QuadtreeTracker accumulates the "touched" history while
    # each frame's active set promotes/evicts with the rover. This is computed from the
    # SAME path/chunking the frames were laid with, so the fine LOD provably tracks the
    # same rover that lays the TREAD trail.
    positions = _tread_frame_positions()
    tracker = QuadtreeTracker(field_size=WIDTH, min_leaf=QT_MIN_LEAF,
                              refine_factor=QT_REFINE_FACTOR,
                              footprint_radius_cells=QT_FOOTPRINT_RADIUS_CELLS)
    # Step the tracker frame by frame, snapshotting the active set AND the cumulative
    # touched history AS OF THAT FRAME (touched grows monotonically: empty pre-drive,
    # full at the end). Snapshotting inside the loop is required — reading
    # tracker.touched_boxes() after the loop would give every frame the FINAL trail.
    qt_per_frame = []
    qt_touched_per_frame = []
    for pos in positions:
        qt_per_frame.append(tracker.step(pos))
        qt_touched_per_frame.append(tracker.touched_boxes())

    scene_dir = os.path.join(SAMPLES_DIR, name)
    os.makedirs(scene_dir, exist_ok=True)

    # Active zone tightly bounds the driven corridor so the downstream wireframe/Godot
    # loader focuses the fine-solve patch on the track (spec §4 "under wheels" patch).
    rmin = max(0, min(cr0, cr1, cr2) - 12)
    cmin = max(0, min(cc0, cc1, cc2) - 12)
    rmax = min(HEIGHT, max(cr0, cr1, cr2) + 12)
    cmax = min(WIDTH, max(cc0, cc1, cc2) + 12)
    az = {"min_rc": [rmin, cmin], "max_rc": [rmax, cmax]}
    qt = _default_quadtree(active_row0=rmin, active_col0=cmin,
                           active_size=max(rmax - rmin, cmax - cmin))

    for i, frame_cs in enumerate(frames):
        tdir = os.path.join(scene_dir, f"t{i:03d}")
        meta = _base_metadata(
            name, active_zone=az, quadtree=qt, height_range=_height_range(frame_cs),
            notes=f"tread-track frame {i}/{len(frames)-1}; rover wheel footprint advancing "
                  f"along a 2-segment path, laying VIRGIN->TREAD compaction (density up, "
                  f"rut sinks, disturbance bumped). Mass conserved — pure compaction "
                  f"(spec §6; rover.py).")
        meta["frame_index"] = i
        # ADDITIVE (INTERFACE.md v1.0.1): per-frame interaction-keyed quadtree state. The
        # existing static "quadtree" key (D1b wireframes) is untouched; these are NEW
        # optional keys consumers may ignore. boxes are [r0,c0,r1,c1] half-open cell boxes.
        _attach_quadtree_meta(meta, qt_per_frame[i], positions[i],
                              qt_touched_per_frame[i])
        save_scene(tdir, frame_cs.fields_dict(), meta)

    # First/last-frame previews at the parent level (mirrors crater_caveins).
    _write_previews(os.path.join(scene_dir, "t000"), frames[0], name + "_t000")
    last = len(frames) - 1
    _write_previews(os.path.join(scene_dir, f"t{last:03d}"), frames[-1],
                    name + f"_t{last:03d}")

    parent_meta = _base_metadata(
        name, active_zone=az, quadtree=qt, height_range=_height_range(frames[-1]),
        notes="TIME SERIES (driven-rover tread track). A wheel footprint is advanced along "
              "a 2-segment path and rover.wheel_pass is applied incrementally per frame, "
              "laying a VIRGIN->TREAD compaction trail (density up toward RHO_DEEP, rut "
              "sinks via height=datum+mass/density, disturbance bumped). Each tNNN/ is a "
              "full snapshot; mass conserved across the series (pure compaction, spec §6).")
    parent_meta["time_series"] = {
        "frame_count": len(frames),
        "frame_cadence_steps": 1,  # one path-chunk advance per frame
        "frame_dirs": [f"t{i:03d}" for i in range(len(frames))],
        "mass_conserved_kg": round(mass_after, 6),
        "mass_drift_kg": round(abs(mass_after - mass_before), 9),
    }
    # ADDITIVE (INTERFACE.md v1.0.1): advertise that each frame carries the per-frame
    # interaction-keyed quadtree (active_leaves / quadtree_nodes / touched_leaves / rover_rc).
    parent_meta["quadtree_lod"] = {
        "min_leaf": QT_MIN_LEAF, "refine_factor": QT_REFINE_FACTOR,
        "footprint_radius_cells": QT_FOOTPRINT_RADIUS_CELLS, "field_size": WIDTH,
        "per_frame_keys": ["active_leaves", "quadtree_nodes", "touched_leaves", "rover_rc"],
        "note": "interaction-keyed quadtree: leaves near the rover promote to min_leaf "
                "(fine/active), distant regions stay coarse (spec §4). Optional; ignorable.",
    }
    import json
    with open(os.path.join(scene_dir, "metadata.json"), "w") as fh:
        json.dump(parent_meta, fh, indent=2)
    n_active_last = len(qt_per_frame[-1].active_leaves)
    n_touched = len(tracker.touched_leaves())
    print(f"  wrote {name}  frames={len(frames)}  "
          f"mass_before={mass_before:.4f} mass_after={mass_after:.4f} kg "
          f"drift={abs(mass_after-mass_before):.2e} kg  "
          f"qt active(last)={n_active_last} touched(total)={n_touched}")


# ---------------------------------------------------------------------------
# NEW render-fidelity scenes (render_fidelity_spec.md §5/§6, INTERFACE.md §5.2/§5.3).
# These ADD to the existing seven scenes; they never touch the existing builders'
# outputs (HARD RULE 3). All metadata additions are ADDITIVE optional keys exactly
# like _attach_quadtree_meta (§5.1): new keys only, schema_version stays "1.0".
# ---------------------------------------------------------------------------

# Mode-B corridor-refinement knobs for the 4-wheel scene (spec §2.2 refinement block).
FINE_CELL_M = 0.01          # 1 cm active-corridor resolution (spec §2.2 experiment knob)
FINE_MIN_LEAF = 4           # quadtree may subdivide past min_leaf to mark fine tiles (§2.2)
N_4WHEEL_FRAMES = 18        # motion frames (+ pristine t000); modest, raw frames gitignored


def _attach_refinement_meta(meta: dict, *, enabled: bool, refine_where: str,
                            tiles_desc: list[dict] | None = None) -> None:
    """ADDITIVELY attach the §5.3 ``refinement`` policy block (and optional ``tiles[]``).

    Mirrors ``_attach_quadtree_meta``'s discipline: adds NEW optional keys ONLY (never
    touches existing rasters or metadata keys). A v1.0/v1.0.1 consumer that ignores these
    renders the base rasters exactly as today (spec §2.2; INTERFACE.md §5.3). The refinement
    factor ``k = base_cell_m/fine_cell_m`` is the validated positive integer (refinement.k_factor).
    """
    meta["refinement"] = {
        "enabled": bool(enabled),
        "base_cell_m": CELL_M,
        "fine_cell_m": FINE_CELL_M,
        "refine_where": refine_where,   # "touched" -> whole driven corridor stays fine (§2.2)
        "fine_min_leaf": FINE_MIN_LEAF,
    }
    if tiles_desc is not None:
        meta["tiles"] = tiles_desc      # PER-FRAME key (§5.3); each entry id/region_rc/cell_m/dir


def _heading_from_segment(p_prev, p_cur) -> float:
    """Travel heading [rad] from path point p_prev -> p_cur (INTERFACE.md §5.2 convention:
    0 = +col/+X, +pi/2 = +row/+Z). forward unit (drow,dcol)=(sin h, cos h) ⇒ h=atan2(drow,dcol)."""
    drow = float(p_cur[0] - p_prev[0])
    dcol = float(p_cur[1] - p_prev[1])
    if drow == 0.0 and dcol == 0.0:
        return 0.0
    return float(np.arctan2(drow, dcol))


def build_tread_track_4wheel() -> None:
    """TIME SERIES: a rover drives a 2-segment path laying FOUR separate compacting ruts.

    The render-fidelity headline (spec §5 "rover.py -> 4-wheel stamping", §4.2.3): unlike
    ``build_tread_track`` (one disc footprint), this drives with ``rover.four_wheel_pass`` so
    the IPEx wheel layout (gauge 0.57 m, wheelbase 0.40 m; asce-es-2024) lays LF/RF/LB/RB as
    four distinct VIRGIN->TREAD ruts. Mass is conserved per pass (density-only compaction; the
    column thins so each rut sinks via height=datum+mass/density — spec §6).

    Each frame carries, ADDITIVELY (consumers MAY ignore; schema_version stays "1.0"):
      - §5.2 ``wheel_tracks``: the four contact polylines + per-wheel heading + width_m, built
        by ``rover.build_wheel_tracks_meta`` so the shader can orient per-wheel cleat detail
        (§4.2.3) WITHOUT resolving cleats in the heightfield.
      - §5.3 ``refinement`` policy block (enabled=true, base 2 cm, fine 1 cm, refine_where
        "touched", fine_min_leaf 4) — the Mode-B corridor-refinement contract (spec §2.2).
    The FINAL frame additionally writes §5.3 ``tiles[]`` over the touched corridor: each tile
    is refined to 1 cm via ``refinement.extract_tiles`` and saved as a normal 5-raster bundle
    under ``tiles/tile_<id>/`` (the persistent crisp trail; base<->tile mass-consistent §2.4).
    """
    name = "tread_track_4wheel"
    # A 2-segment drive (row,col) waypoints, clear of the borders so the full 4-wheel
    # footprint (half-gauge ~14 cells, half-base ~10 cells) lands on-grid at every pose.
    cr0, cc0 = int(0.25 * HEIGHT), int(0.22 * WIDTH)
    cr1, cc1 = int(0.50 * HEIGHT), int(0.50 * WIDTH)
    cr2, cc2 = int(0.74 * HEIGHT), int(0.66 * WIDTH)

    path = straight_path(cr0, cc0, cr1, cc1, step_cells=1)
    path += straight_path(cr1, cc1, cr2, cc2, step_cells=1)[1:]
    chunks = np.array_split(np.arange(len(path)), N_4WHEEL_FRAMES)

    # Per-frame poses (center_rc, heading) for four_wheel_pass + the contact polylines we
    # accumulate for the §5.2 wheel_tracks metadata. Frame 0 is pristine (no rover).
    cs = procgen.rolling_hills(WIDTH, HEIGHT, CELL_M, seed=13, amplitude_m=0.1, base_cells=3)
    mass_before = cs.total_mass()
    frames: list[ColumnState] = [_clone(cs)]              # t000 pristine
    frame_polylines: list[dict | None] = [None]           # per-frame wheel_tracks polylines
    frame_headings: list[dict | None] = [None]
    frame_centers: list[tuple[int, int] | None] = [None]

    prev_pt = path[0]
    for chunk in chunks:
        if len(chunk) == 0:
            continue
        sub = [path[k] for k in chunk]
        # Build a pose per path sample in this chunk; heading from the local segment so the
        # cleat field orients to true travel direction (INTERFACE.md §5.2).
        poses: list[tuple[tuple[float, float], float]] = []
        p_prev = prev_pt
        for p in sub:
            poses.append(((float(p[0]), float(p[1])), _heading_from_segment(p_prev, p)))
            p_prev = p
        prev_pt = sub[-1]
        polylines = four_wheel_pass(cs, poses, wheel_width_m=0.18, compaction=0.14)
        # Per-wheel heading = the chunk's last-segment heading (one travel dir per frame).
        head = _heading_from_segment(sub[0] if len(sub) == 1 else sub[-2], sub[-1])
        frame_polylines.append(polylines)
        frame_headings.append({key: head for key in ("LF", "RF", "LB", "RB")})
        frame_centers.append(tuple(sub[-1]))
        frames.append(_clone(cs))
    mass_after = cs.total_mass()

    # Active zone + static D1b quadtree tightly bound the driven corridor (spec §4).
    rmin = max(0, min(cr0, cr1, cr2) - 16)
    cmin = max(0, min(cc0, cc1, cc2) - 16)
    rmax = min(HEIGHT, max(cr0, cr1, cr2) + 16)
    cmax = min(WIDTH, max(cc0, cc1, cc2) + 16)
    az = {"min_rc": [rmin, cmin], "max_rc": [rmax, cmax]}
    qt = _default_quadtree(active_row0=rmin, active_col0=cmin,
                           active_size=max(rmax - rmin, cmax - cmin))

    # Track the interaction quadtree (the touched corridor) frame by frame, so the final
    # frame's tiles cover exactly the driven trail (refine_where="touched", §2.2/§5.3).
    tracker = QuadtreeTracker(field_size=WIDTH, min_leaf=QT_MIN_LEAF,
                              refine_factor=QT_REFINE_FACTOR,
                              footprint_radius_cells=QT_FOOTPRINT_RADIUS_CELLS)
    qt_per_frame = []
    qt_touched_per_frame = []
    for center in frame_centers:
        qt_per_frame.append(tracker.step(center))
        qt_touched_per_frame.append(tracker.touched_boxes())

    scene_dir = os.path.join(SAMPLES_DIR, name)
    os.makedirs(scene_dir, exist_ok=True)
    last = len(frames) - 1
    final_tiles_desc: list[dict] = []

    for i, frame_cs in enumerate(frames):
        tdir = os.path.join(scene_dir, f"t{i:03d}")
        meta = _base_metadata(
            name, active_zone=az, quadtree=qt, height_range=_height_range(frame_cs),
            notes=f"4-wheel tread-track frame {i}/{last}; IPEx 4-wheel footprint "
                  f"(gauge {0.57} m, wheelbase {0.40} m) advancing along a 2-segment path, "
                  f"laying FOUR VIRGIN->TREAD ruts (density up, ruts sink, disturbance "
                  f"bumped). Mass conserved — pure compaction (spec §5, §6; rover.py).")
        meta["frame_index"] = i
        # ADDITIVE (INTERFACE.md v1.0.1 §5.1): per-frame interaction-keyed quadtree state.
        _attach_quadtree_meta(meta, qt_per_frame[i], frame_centers[i],
                              qt_touched_per_frame[i])
        # ADDITIVE (INTERFACE.md v1.0.2 §5.2): per-frame wheel_tracks (four contact polylines
        # + travel heading + contact width); built by rover so the shader can orient cleats.
        if frame_polylines[i] is not None:
            meta["wheel_tracks"] = build_wheel_tracks_meta(
                frame_polylines[i], frame_headings[i], cell_m=CELL_M, width_m=0.18)
        # ADDITIVE (INTERFACE.md v1.0.2 §5.3): the Mode-B refinement policy block on EVERY
        # frame; the FINAL frame additionally emits tiles[] + writes the fine tile bundles
        # under this frame's own tiles/ dir (tdir already names it; created on demand).
        if i == last:
            final_tiles_desc = _write_4wheel_tiles(tdir, frame_cs, qt_touched_per_frame[i])
            _attach_refinement_meta(meta, enabled=True, refine_where="touched",
                                    tiles_desc=final_tiles_desc)
        else:
            _attach_refinement_meta(meta, enabled=True, refine_where="touched")
        # ADDITIVE discoverability (ignorable; consumers feature-detect by key presence, not
        # this — INTERFACE.md §5.3). schema_version stays "1.0".
        meta["contract_revision"] = "1.0.2"
        meta["features"] = ["wheel_tracks", "refinement", "tiles"]
        save_scene(tdir, frame_cs.fields_dict(), meta)

    _write_previews(os.path.join(scene_dir, "t000"), frames[0], name + "_t000")
    _write_previews(os.path.join(scene_dir, f"t{last:03d}"), frames[-1],
                    name + f"_t{last:03d}")

    parent_meta = _base_metadata(
        name, active_zone=az, quadtree=qt, height_range=_height_range(frames[-1]),
        notes="TIME SERIES (driven-rover, FOUR wheels). rover.four_wheel_pass lays the IPEx "
              "LF/RF/LB/RB footprint as four separate VIRGIN->TREAD compaction ruts along a "
              "2-segment path (mass conserved, pure compaction; spec §5/§6). Per-frame §5.2 "
              "wheel_tracks orient cleat detail; §5.3 refinement (1 cm corridor) + tiles[] on "
              "the final frame back the crisp persistent trail. All additive; ignorable.")
    parent_meta["time_series"] = {
        "frame_count": len(frames),
        "frame_cadence_steps": 1,
        "frame_dirs": [f"t{i:03d}" for i in range(len(frames))],
        "mass_conserved_kg": round(mass_after, 6),
        "mass_drift_kg": round(abs(mass_after - mass_before), 9),
    }
    parent_meta["quadtree_lod"] = {
        "min_leaf": QT_MIN_LEAF, "refine_factor": QT_REFINE_FACTOR,
        "footprint_radius_cells": QT_FOOTPRINT_RADIUS_CELLS, "field_size": WIDTH,
        "per_frame_keys": ["active_leaves", "quadtree_nodes", "touched_leaves", "rover_rc",
                           "wheel_tracks", "refinement"],
        "note": "interaction-keyed quadtree + per-frame wheel_tracks; the final frame carries "
                "the §5.3 refinement tiles over the touched corridor. Optional; ignorable.",
    }
    # Scene-level refinement policy on the parent (spec §5.3 "refinement appears on BOTH the
    # parent and per-frame"); no tiles[] here (tiles are a per-frame key).
    _attach_refinement_meta(parent_meta, enabled=True, refine_where="touched")
    parent_meta["contract_revision"] = "1.0.2"
    parent_meta["features"] = ["wheel_tracks", "refinement", "tiles"]
    import json
    with open(os.path.join(scene_dir, "metadata.json"), "w") as fh:
        json.dump(parent_meta, fh, indent=2)
    print(f"  wrote {name}  frames={len(frames)}  "
          f"mass_before={mass_before:.4f} mass_after={mass_after:.4f} kg "
          f"drift={abs(mass_after - mass_before):.2e} kg  tiles(final)={len(final_tiles_desc)}")


def _write_4wheel_tiles(frame_dir: str, frame_cs: ColumnState,
                        touched_boxes: list[list[int]]) -> list[dict]:
    """Refine the touched corridor to FINE_CELL_M and write each tile bundle (§5.3).

    Uses ``refinement.extract_tiles`` to refine each base-cell-aligned touched leaf box (§5.1)
    to ``FINE_CELL_M`` (k = CELL_M/FINE_CELL_M = 2, a validated positive integer). Each tile is
    a normal INTERFACE.md raster bundle (the 5 REQUIRED rasters at the fine cell size) written
    to ``<frame_dir>/tiles/tile_<id>/`` via ``save_scene`` — a tile dir IS just a raster bundle.
    By construction ``coarsen(tile) == base block`` (base<->tile consistency, §2.4/§5.3,
    asserted in tests). Returns the §5.3 ``tiles[]`` descriptor list (id/region_rc/cell_m/dir).

    Tiles are written ONLY on the final frame (the complete, persistent crisp trail) to keep
    the raw frame set modest (raw frames are gitignored; the contract is the point). The
    ``dir`` in each descriptor is RELATIVE to the frame dir (``tiles/tile_<id>``), per §5.3.
    """
    if not touched_boxes:
        return []
    tiles = refinement.extract_tiles(frame_cs, touched_boxes, FINE_CELL_M)
    tiles_dir = os.path.join(frame_dir, "tiles")
    descs: list[dict] = []
    for t in tiles:
        rel_dir = f"tiles/tile_{t.id:04d}"
        tile_dir = os.path.join(tiles_dir, f"tile_{t.id:04d}")
        # Per-tile metadata: a self-contained fine raster bundle (INTERFACE.md §5.3 storage).
        tw, th = t.cs.width, t.cs.height
        x0 = t.region_rc[1] * CELL_M
        z0 = t.region_rc[0] * CELL_M
        tile_meta = {
            "schema_version": "1.0",
            "scene_name": f"tread_track_4wheel/tile_{t.id:04d}",
            "producer": "terrain_authority (NumPy Tier-2 surrogate)",
            "grid": {"width": tw, "height": th, "cell_m": t.cell_m, "order": "row-major-C"},
            "world_bounds_m": {"x0": round(x0, 4), "y0": round(z0, 4),
                               "x1": round(x0 + tw * t.cell_m, 4),
                               "y1": round(z0 + th * t.cell_m, 4)},
            "gravity_m_s2": K.g,
            "fields": {
                "heightmap": {"file": "heightmap.rf32", "dtype": "<f4", "units": "m"},
                "mass_areal": {"file": "mass_areal.rf32", "dtype": "<f4", "units": "kg/m^2"},
                "density": {"file": "density.rf32", "dtype": "<f4", "units": "kg/m^3"},
                "disturbance": {"file": "disturbance.rf32", "dtype": "<f4",
                                "units": "1 (normalized)"},
                "state_label": {"file": "state_label.r8", "dtype": "u1", "enum": K.STATE_NAMES},
            },
            "tile_of": "tread_track_4wheel",
            "region_rc": [int(v) for v in t.region_rc],  # BASE cells (§5.3)
            "refine_factor_k": refinement.k_factor(CELL_M, FINE_CELL_M),
            "notes": "§5.3 refinement tile: a base-cell-aligned k x k block refined to "
                     "fine_cell_m. coarsen(this) == the base block (base<->tile mass-consistent, "
                     "spec §2.4). Consumers MAY ignore tiles and render the base rasters.",
        }
        save_scene(tile_dir, t.cs.fields_dict(), tile_meta)
        descs.append(t.descriptor(rel_dir))
    return descs


def build_excavation_marks() -> None:
    """A short drum-dig demo: cut an EXCAVATED band, dump it as SPOIL; emit §5.2 drum_marks.

    The foss_ipex-distinctive excavation story (spec §5 "Drum dig events", §4.2.4; excavation
    is disabled in LAC's mapping year, so this is ours, not LAC-required). A counter-rotating
    RASSOR drum (2021-ASCEND-Mass-Inference-RASSOR.pdf) cuts a swath to ``depth_m`` into the
    drum inventory (EXCAVATED) and dumps it elsewhere as SPOIL (bulking: same mass, lower
    density -> more height; spec §7). Mass is conserved THROUGH the inventory (rover.drum_pass).

    Emits the §5.2 ``drum_marks`` metadata (swath + depth + teeth params + phase) so the shader
    can orient/phase the teeth normals + POM (§4.2.4) WITHOUT resolving teeth in the grid. A
    tiny two-frame series (t000 pristine -> t001 dug) bookends the change; all additive keys.
    """
    name = "excavation_marks"
    cs = procgen.flat_compact(WIDTH, HEIGHT, CELL_M, seed=2)
    mass_before = cs.total_mass()
    frame0 = _clone(cs)  # pristine

    # A straight dig swath across the middle, with the spoil dumped a short way off-axis.
    dig_r = HEIGHT // 2
    c_start, c_end = int(0.30 * WIDTH), int(0.62 * WIDTH)
    swath = straight_path(dig_r, c_start, dig_r, c_end, step_cells=1)
    dump_r = dig_r + 24  # ~48 cm off-axis spoil heap
    dump = straight_path(dump_r, c_start, dump_r, c_end, step_cells=1)

    depth_m = 0.04   # 4 cm cut (RASSOR scoop-depth scale; shader teeth ride this band)
    width_m = 0.20   # drum swath width (rover.DRUM defaults)
    moved_kg = drum_pass(cs, swath, depth_m=depth_m, width_m=width_m, dump_rc=dump)
    mass_after = cs.total_mass()
    frame1 = _clone(cs)

    # heading along the swath travel dir (+col/+X) = 0 rad (INTERFACE.md §5.2 convention).
    swath_heading = _heading_from_segment(swath[0], swath[-1])
    drum_entry = build_drum_marks_meta(swath, swath_heading, drum="front",
                                       depth_m=depth_m, width_m=width_m, cell_m=CELL_M)

    # Active zone bounds the dig + dump corridor.
    rmin = max(0, dig_r - 12)
    rmax = min(HEIGHT, dump_r + 12)
    cmin = max(0, c_start - 12)
    cmax = min(WIDTH, c_end + 12)
    az = {"min_rc": [rmin, cmin], "max_rc": [rmax, cmax]}
    qt = _default_quadtree(active_row0=rmin, active_col0=cmin,
                           active_size=max(rmax - rmin, cmax - cmin))

    scene_dir = os.path.join(SAMPLES_DIR, name)
    os.makedirs(scene_dir, exist_ok=True)
    frames = [frame0, frame1]
    for i, frame_cs in enumerate(frames):
        tdir = os.path.join(scene_dir, f"t{i:03d}")
        meta = _base_metadata(
            name, active_zone=az, quadtree=qt, height_range=_height_range(frame_cs),
            notes=f"excavation-marks frame {i}/{len(frames)-1}; RASSOR drum cuts an EXCAVATED "
                  f"swath (depth {depth_m} m) into the drum inventory and dumps it as SPOIL "
                  f"(bulking, spec §7). Mass conserved through the inventory (spec §5/§6; "
                  f"rover.drum_pass).")
        meta["frame_index"] = i
        # ADDITIVE (INTERFACE.md v1.0.2 §5.2): drum_marks on the DUG frame only (no active
        # drum on the pristine t000). The scene wraps the single rover entry in a list.
        if i == 1:
            meta["drum_marks"] = [drum_entry]
        meta["contract_revision"] = "1.0.2"
        meta["features"] = ["drum_marks"]
        save_scene(tdir, frame_cs.fields_dict(), meta)

    _write_previews(os.path.join(scene_dir, "t000"), frames[0], name + "_t000")
    _write_previews(os.path.join(scene_dir, "t001"), frames[-1], name + "_t001")

    parent_meta = _base_metadata(
        name, active_zone=az, quadtree=qt, height_range=_height_range(frames[-1]),
        notes="Drum-dig demo (t000 pristine -> t001 dug). rover.drum_pass cuts an EXCAVATED "
              "swath into the drum inventory and dumps SPOIL off-axis (bulking, spec §7); mass "
              "conserved through the inventory. Per-frame §5.2 drum_marks orient the shader "
              "teeth/POM detail (§4.2.4). All additive; ignorable.")
    parent_meta["time_series"] = {
        "frame_count": len(frames),
        "frame_cadence_steps": 1,
        "frame_dirs": [f"t{i:03d}" for i in range(len(frames))],
        "mass_conserved_kg": round(mass_after, 6),
        "mass_drift_kg": round(abs(mass_after - mass_before), 9),
        "drum_excavated_kg": round(moved_kg, 6),
    }
    parent_meta["contract_revision"] = "1.0.2"
    parent_meta["features"] = ["drum_marks"]
    import json
    with open(os.path.join(scene_dir, "metadata.json"), "w") as fh:
        json.dump(parent_meta, fh, indent=2)
    print(f"  wrote {name}  frames={len(frames)}  excavated={moved_kg:.3f} kg  "
          f"mass_before={mass_before:.4f} mass_after={mass_after:.4f} kg "
          f"drift={abs(mass_after - mass_before):.2e} kg")


def _tread_path_endpoints() -> tuple[int, int, int, int, int, int]:
    """The 2-segment drive path (row,col) waypoints: start -> mid bend -> end."""
    # Drive diagonally across the field with a gentle bend at the middle, staying clear of
    # the borders so the full wheel disc footprint lands on-grid.
    cr0, cc0 = int(0.22 * HEIGHT), int(0.18 * WIDTH)   # start (lower-left-ish)
    cr1, cc1 = int(0.50 * HEIGHT), int(0.52 * WIDTH)   # bend (center)
    cr2, cc2 = int(0.80 * HEIGHT), int(0.70 * WIDTH)   # end (upper-right-ish)
    return cr0, cc0, cr1, cc1, cr2, cc2


def _tread_frame_positions() -> list[tuple[int, int] | None]:
    """Per-frame rover footprint CENTER (row,col), aligned to the captured frames.

    Frame 0 is the pristine pre-drive surface -> None (no rover on the field yet). Frame i
    (1..n_motion) corresponds to the i-th path chunk having been driven, so the rover sits
    at the LAST cell of that chunk. This is the single source of truth for "where is the
    rover at frame i", reused by both the quadtree-per-frame metadata and the viz, so the
    quadtree provably follows the SAME rover as the tread trail (one coherent story).
    """
    cr0, cc0, cr1, cc1, cr2, cc2 = _tread_path_endpoints()
    path = straight_path(cr0, cc0, cr1, cc1, step_cells=1)
    path += straight_path(cr1, cc1, cr2, cc2, step_cells=1)[1:]
    chunks = np.array_split(np.arange(len(path)), 31)
    positions: list[tuple[int, int] | None] = [None]  # t000 pristine
    for chunk in chunks:
        if len(chunk) == 0:
            continue
        sub = [path[k] for k in chunk]
        positions.append(tuple(sub[-1]))
    return positions


def _replay_tread_track() -> tuple[list[ColumnState], float, float]:
    """Deterministically rebuild the tread track; return (frames, mass_before, mass_after).

    Builds the full (row,col) path, splits it into ~N_FRAMES contiguous chunks, and applies
    wheel_pass to one chunk per frame so the trail is laid progressively. A ColumnState
    clone is captured per frame (frame 0 is the pristine pre-drive surface). The chunking
    here is kept identical to ``_tread_frame_positions`` so the captured frames and the
    per-frame rover positions / quadtree stay in lockstep.
    """
    cs = procgen.rolling_hills(WIDTH, HEIGHT, CELL_M, seed=11, amplitude_m=0.12,
                               base_cells=3)
    mass_before = cs.total_mass()

    cr0, cc0, cr1, cc1, cr2, cc2 = _tread_path_endpoints()
    # Dense path (step_cells=1) so consecutive wheel discs overlap into a continuous rut.
    path = straight_path(cr0, cc0, cr1, cc1, step_cells=1)
    path += straight_path(cr1, cc1, cr2, cc2, step_cells=1)[1:]  # drop duplicate bend point

    n_motion = 31  # motion frames; + the pristine t000 -> 32 total (modest, gitignored raw)
    chunks = np.array_split(np.arange(len(path)), n_motion)

    frames: list[ColumnState] = [_clone(cs)]  # t000 = pristine, pre-drive
    for chunk in chunks:
        if len(chunk) == 0:
            continue
        sub = [path[k] for k in chunk]
        # Wider-than-default contact patch (~22 cm) and a firm per-pass compaction so the
        # VIRGIN->TREAD relabel + rut read clearly on the rolling-hills base.
        wheel_pass(cs, sub, wheel_width_m=0.22, compaction=0.16)
        frames.append(_clone(cs))

    mass_after = cs.total_mass()
    return frames, mass_before, mass_after


def _replay_caveins(diameter_m: float, cr: int, cc: int) -> list[ColumnState]:
    """Deterministically rebuild the cave-in and return a ColumnState clone per frame."""
    cs = procgen.rolling_hills(WIDTH, HEIGHT, CELL_M, seed=5, amplitude_m=0.04,
                               base_cells=2)
    procgen.carve_crater(cs, (cr, cc), diameter_m)
    R_cells = int(0.5 * diameter_m / CELL_M)
    sp = Sandpile(cs, theta_r=K.THETA_R, connectivity=8, transfer_fraction=0.6)
    sp.deposit(cr - int(0.55 * R_cells), cc, mass_kg=120.0, radius_cells=6)

    frames: list[ColumnState] = [_clone(cs)]
    cadence = 4
    for i in range(400):
        moved = sp.relax_step()
        if i % cadence == 0:
            frames.append(_clone(cs))
        if not moved:
            break
    frames.append(_clone(cs))  # final rest state
    return frames


def _clone(cs: ColumnState) -> ColumnState:
    out = ColumnState(width=cs.width, height=cs.height, cell_m=cs.cell_m,
                      mass_areal=cs.mass_areal.copy(), density=cs.density.copy(),
                      state_label=cs.state_label.copy(),
                      disturbance=cs.disturbance.copy(), datum=cs.datum.copy(),
                      ice=None if cs.ice is None else cs.ice.copy(),
                      drum_inventory=cs.drum_inventory)
    return out


# ---------------------------------------------------------------------------
# W2-SCENES (serial join) — connect the disconnected DEM corridor stack to the builder.
# This is ADDITIVE: it never touches the nine legacy builders above or their main() calls.
# The four Wave-2 generators are wired here (docs/dem_terrain_contract.md §8 "W2-SCENES"):
#   * W2-DENSITY  dem_import.polar_mantle_density_fn  -> sourced ChaSTE bulk density (+ datum re-supply)
#   * W2-CRATERS  dem_overlay.make_crater_feature_fn  -> sub-DEM crater population on fine tiles
#   * W2-ILLUM    illumination.horizon_clip           -> terrain-derived shadow mask (demo §4)
#   * W2-VARIANCE tiles_mosaic / dem_overlay fbm_nu0  -> calibrated fine-band roughness overlay
# The committed samples/ rasters are NOT regenerated; we re-derive the missing `datum` field
# in-RAM (the frozen io_fields._FIELD_SPEC omits it) so the streaming ArrayBaseReader is fed.
# ---------------------------------------------------------------------------

#: DEM effective resolution [m] for the Haworth-group base (PGDA Product-90 LDEM_EFFRES band;
#: the same ~15 m the procgen_csfd / dem_overlay self-tests use, Barker 2023). Craters at/above
#: this are ALREADY in the committed base heightmap; populate_craters synthesizes strictly below
#: it (de-confliction), so the crater overlay only adds SUB-DEM detail on fine corridor tiles.
DEM_EFFRES_M_HAWORTH = 15.0

#: Documented default residual variance [m^2] for the fine-band fbm overlay when the caller does
#: NOT pass a calibrated fbm_nu0. This is the [CALIB placeholder] dem_overlay.DEFAULT_OVERLAY_PARAMS
#: value (1.0e-4 m^2 -> ~1 cm RMS at the 2 cm fine cell). HONEST NOTE: the 100 m roughness is
#: carried by the REAL base (deviogram@100m == the _slp anchor by construction); fbm_nu0 calibrates
#: ONLY the sub-DEM / resolved fine band, which is where it is physically meaningful (the overlay
#: is zero-mean per base cell, so it adds NOTHING at or above the 5 m base lattice — see the
#: acceptance discussion in scripts/dem_acceptance.py / docs/dem_terrain_contract.md §7/§8).
DEFAULT_FBM_NU0_FINE = dem_overlay.DEFAULT_OVERLAY_PARAMS["fbm_nu0"]


def build_from_dem(scene_dir: str = "samples/lunar_dem/haworth_10km_5m", *,
                   region: str = "haworth", radius_m: float = 30.0,
                   with_craters: bool = True, fbm_nu0: float | None = None,
                   world_seed: int = 0) -> tuple[dict, dict]:
    """Build a loadable DEM-backed scene by wiring the four Wave-2 generators (contract §8).

    Connects the previously-disconnected corridor stack to a builder. Steps (contract §8 +
    the binding decisions in the W2-SCENES task brief):

      1. ``load_scene`` the committed real-LOLA base (heightmap + carried fields + metadata).
         Read the loose-mantle thickness from ``metadata.regolith_model.mantle_thickness_m``
         (fallback ``K.Z_T``).
      2. INJECT the sourced ChaSTE density WITHOUT regenerating the committed rasters:
         ``density = polar_mantle_density_fn(mantle_m)(X,Y)`` (a constant ChaSTE bulk grid), then
         RE-DERIVE ``datum = heightmap - mantle_m`` and ``mass_areal = mantle_m * density`` so
         ``derive_height() == heightmap`` stays exact (asserted ``max|err| <= 1e-3`` m). This both
         supplies the ``datum`` the frozen ``io_fields`` omits AND lands the sourced polar density.
      3. Build a ``dem_io.ArrayBaseReader`` over {mass_areal, density, datum, state_label,
         disturbance} with NON-ZERO ``world_x0/world_y0`` from ``metadata.world_bounds_m``, and a
         ``tiles_mosaic.TileMosaic`` over it.
      4. Forward ``feature_fn = make_crater_feature_fn(dem_effres_m=...)`` (when ``with_craters``)
         and the calibrated ``fbm_nu0`` into the overlay params used by the mosaic's fine tiles.
      5. Compute a terrain-derived illumination mask via ``illumination.horizon_clip`` and carry
         it in the returned meta (CLEARLY tagged terrain-derived, NOT a Product-69 ingest).
      6. Update ``meta`` via ``tiles_mosaic.write_dem_base_metadata`` (non-zero world_bounds,
         base/fine cell, region, dem_provenance, density source [CALIB] tag). schema_version 1.0.
      7. Return ``(fields, meta)``. ``fields`` IS the loadable base bundle whose
         ``derive_height() == heightmap`` (so the acceptance harness can measure the deviogram on
         the real surface); ``meta`` carries the corridor / illumination / density provenance.

    Returns
    -------
    (fields, meta) : (dict[str, np.ndarray], dict)
        ``fields`` has the 5 REQUIRED rasters (heightmap, mass_areal, density, disturbance,
        state_label) PLUS the re-derived ``datum`` — a full loadable scene whose surface is the
        real DEM. ``meta`` is the committed metadata extended (additively) with the corridor /
        density / illumination provenance.
    """
    # Resolve the scene dir relative to the repo root when a relative path is given.
    if not os.path.isabs(scene_dir):
        scene_dir = os.path.join(ROOT, scene_dir)

    # --- Step 1: load the committed real-LOLA base + metadata --------------------------
    base, meta = load_scene(scene_dir)
    if "heightmap" not in base:
        raise ValueError(f"build_from_dem: {scene_dir} has no heightmap.rf32 (not a DEM scene)")
    heightmap = np.asarray(base["heightmap"], dtype=np.float64)
    H, W = heightmap.shape

    grid = meta["grid"]
    base_cell_m = float(meta.get("base_cell_m", grid["cell_m"]))
    fine_cell_m = float(meta.get("fine_cell_m", CELL_M))  # 2 cm corridor (contract §0)

    # Loose-mantle thickness: the cm-scale loose layer the datum path injects (eval §5 step 1).
    mantle_m = float(meta.get("regolith_model", {}).get("mantle_thickness_m", K.Z_T))

    # --- Step 2: inject sourced ChaSTE density + RE-DERIVE datum (the §8 "datum re-supply trap").
    # polar_mantle_density_fn ignores X,Y (it is a single mass-weighted-mean scalar broadcast,
    # NOT a spatial field — honest per the dem_import closure doc). density CANCELS out of the
    # height inversion (datum=Z-mantle, mass=mantle*rho, height=datum+mass/rho==Z for any rho>0),
    # so this lands the sourced polar areal mass WITHOUT moving the surface.
    density_fn = dem_import.polar_mantle_density_fn(mantle_m)
    rho_bar = float(density_fn.rho_bar)  # the constant ChaSTE bulk density [kg/m^3]
    if not (K.RHO_SURFACE_POLAR <= rho_bar <= K.RHO_BULK_POLAR_10CM):
        raise AssertionError(
            f"build_from_dem: ChaSTE rho_bar={rho_bar} outside "
            f"[{K.RHO_SURFACE_POLAR}, {K.RHO_BULK_POLAR_10CM}] (density acceptance range)")
    density = np.full((H, W), rho_bar, dtype=np.float64)
    datum = heightmap - mantle_m
    mass_areal = np.full((H, W), mantle_m * rho_bar, dtype=np.float64)

    # Carried fields: keep the committed state_label / disturbance verbatim (defaults if absent).
    state_label = np.asarray(base.get("state_label", np.zeros((H, W), np.uint8)), dtype=np.uint8)
    disturbance = np.asarray(base.get("disturbance", np.zeros((H, W))), dtype=np.float64)

    # Assert the datum-path round-trip: derive_height() == committed heightmap (contract §8).
    derived = datum + mass_areal / density
    rt_err = float(np.max(np.abs(derived - heightmap)))
    if rt_err > 1e-3:
        raise AssertionError(
            f"build_from_dem: re-derived datum path deviates from the committed heightmap by "
            f"{rt_err:.3e} m (> 1e-3); the datum re-supply is broken")

    base_fields = {
        "mass_areal": mass_areal, "density": density, "datum": datum,
        "state_label": state_label, "disturbance": disturbance,
    }

    # --- Step 3: ArrayBaseReader (NON-ZERO global origin) + TileMosaic over it ----------
    wb = meta["world_bounds_m"]
    world_x0 = float(wb["x0"])
    world_y0 = float(wb["y0"])
    reader = dem_io.ArrayBaseReader(base_fields, base_cell_m=base_cell_m,
                                    world_x0=world_x0, world_y0=world_y0)

    # --- Step 4: crater feature_fn (sub-DEM) + calibrated fbm_nu0 into the overlay params.
    eff_res = float(meta.get("dem_effres_m",
                             meta.get("dem_provenance", {}).get("dem_effres_m",
                                                                DEM_EFFRES_M_HAWORTH)))
    nu0 = DEFAULT_FBM_NU0_FINE if fbm_nu0 is None else float(fbm_nu0)
    overlay_params = dict(dem_overlay.DEFAULT_OVERLAY_PARAMS)
    overlay_params["fbm_nu0"] = nu0

    feature_fn = None
    if with_craters:
        feature_fn = dem_overlay.make_crater_feature_fn(
            dem_effres_m=eff_res, d_min_m=1.0, base_cell_class=0)

    mosaic = tiles_mosaic.TileMosaic(
        reader, base_cell_m, fine_cell_m,
        tile_base_cells=8, max_resident_tiles=16, world_seed=int(world_seed),
        overlay_params=overlay_params, feature_fn=feature_fn)

    # Demand-refine the fine corridor around the scene center (a LIVE-pose stand-in). This
    # exercises the WHOLE wired stack (overlay_residual + crater feature_fn + fbm) and lets us
    # verify coarsen(fine tile) == base BIT-EXACT (conservation preserved even with craters).
    center_x = (world_x0 + float(wb["x1"])) / 2.0
    center_y = (world_y0 + float(wb["y1"])) / 2.0
    fine_tiles = mosaic.ensure_fine((center_x, center_y), radius_m=float(radius_m))

    # Conservation self-check on one materialized fine tile: coarsen(fine) == its base block.
    corridor_conservation = None
    if fine_tiles:
        t0 = fine_tiles[0]
        r0, c0, r1, c1 = t0.region_rc
        base_block = reader.window((r0, c0, r1, c1))
        # Coarsen the fine bundle through the 5 carried base fields (fields_dict() omits the
        # base-only `datum`; coarsen_field needs it — build the dict from the Tile's ColumnState).
        fine_bundle = {
            "mass_areal": t0.cs.mass_areal, "density": t0.cs.density, "datum": t0.cs.datum,
            "state_label": t0.cs.state_label, "disturbance": t0.cs.disturbance,
        }
        back = refinement.coarsen_field(fine_bundle, mosaic.k)
        scale = max(float(np.max(np.abs(base_block["mass_areal"]))), 1.0)
        mass_relerr = float(np.max(np.abs(back["mass_areal"] - base_block["mass_areal"]))) / scale
        datum_exact = bool(np.array_equal(back["datum"], base_block["datum"]))
        state_exact = bool(np.array_equal(back["state_label"], base_block["state_label"]))
        corridor_conservation = {
            "tile_region_rc": [int(v) for v in t0.region_rc],
            "k": int(mosaic.k),
            "mass_relerr": mass_relerr,
            "datum_bit_exact": datum_exact,
            "state_bit_exact": state_exact,
            "coarsen_equals_base": bool(mass_relerr <= 1e-12 and datum_exact and state_exact),
        }

    # --- Step 5: terrain-derived illumination mask (W2-ILLUM) for the demo's shadow attribution.
    # Grazing polar sun; azimuth defaulted to the hillshade-preview convention (light from +Z/N).
    sun_az = float(meta.get("sun_az_deg", 315.0))
    sun_el = float(K.SUN_ELEVATION_DEG_POLAR)
    illum_mask = illumination.horizon_clip(heightmap, base_cell_m, sun_az, sun_el)
    lit_fraction = float(illum_mask.mean())

    # --- Step 6: write the additive DEM/mosaic metadata block (non-zero world_bounds) -------
    tiles_mosaic.write_dem_base_metadata(
        meta,
        world_bounds_m=wb,
        base_cell_m=base_cell_m, fine_cell_m=fine_cell_m,
        region=str(meta.get("region", region)),
        local_datum_offset_m=float(meta.get("local_datum_offset_m", 0.0)),
        dem_provenance=meta.get("dem_provenance"))

    # ADDITIVE (schema_version stays "1.0"): the corridor / density / illumination provenance.
    meta["scene_name"] = meta.get("scene_name", "lunar_dem/haworth_10km_5m")
    meta["dem_corridor"] = {
        "fine_cell_m": fine_cell_m,
        "refine_factor_k": int(mosaic.k),
        "tile_base_cells": int(mosaic.tile_base_cells),
        "radius_m": float(radius_m),
        "world_seed": int(world_seed),
        "with_craters": bool(with_craters),
        "dem_effres_m": eff_res,
        "fine_tiles_materialized": len(fine_tiles),
        "demand_driven_note": "fine 2 cm tiles materialized at runtime around the LIVE pose "
                              "(contract §0); 100 m roughness is carried by the REAL base, the "
                              "overlay adds only sub-DEM detail (zero-mean per base cell).",
    }
    if corridor_conservation is not None:
        meta["dem_corridor"]["conservation_check"] = corridor_conservation
    # Sourced density provenance ([CALIB] ChaSTE; honest scalar-broadcast tag).
    meta["density_source"] = {
        "tag": "[CALIB]",
        "model": "ChaSTE depth-integrated bulk density over [0, mantle_m] (Durga Prasad 2026)",
        "rho_bar_kg_m3": round(rho_bar, 4),
        "mantle_m": mantle_m,
        "range_kg_m3": [K.RHO_SURFACE_POLAR, K.RHO_BULK_POLAR_10CM],
        "note": "single mass-weighted-mean scalar BROADCAST over the grid (NOT a spatial field, "
                "ChaSTE is one vertical probe at 69.4S); density cancels in derive_height so the "
                "DEM surface is untouched (dem_import.polar_mantle_density_fn).",
    }
    meta["fbm_nu0_fine"] = nu0
    # Illumination: CLEARLY tagged terrain-derived single-epoch local-horizon, NOT a Product-69.
    meta["illumination"] = {
        "tag": "terrain-derived",
        "method": "illumination.horizon_clip (per-pixel local-horizon ray-march)",
        "sun_az_deg": sun_az,
        "sun_el_deg": sun_el,
        "lit_fraction": round(lit_fraction, 6),
        "shadow_fraction": round(1.0 - lit_fraction, 6),
        "honesty": "single-epoch single-tile geometric horizon for ONE (az,el); NOT a PGDA "
                   "Product-69 illumination/PSR ingest (no Product-69 reader/data on disk). "
                   "Feeds the demo's per-face shadow attribution (demo_spiral_contract.md §4).",
    }
    meta["features"] = sorted(set(meta.get("features", []))
                              | {"dem_backbone", "dem_corridor", "density_chaste",
                                 "illumination_horizon"})

    # --- Step 7: assemble the returned loadable fields (5 required + re-derived datum) ------
    fields = {
        "heightmap": heightmap,
        "mass_areal": mass_areal,
        "density": density,
        "disturbance": disturbance,
        "state_label": state_label,
        "datum": datum,
    }
    return fields, meta


def build_dem_scene() -> None:
    """main() hook for build_from_dem — ALONGSIDE the nine legacy builders (degrades gracefully).

    A missing committed DEM scene (e.g. a fresh checkout without samples/lunar_dem/) is logged
    and skipped, NOT fatal — the nine legacy builders above are unaffected. We do NOT re-write
    the committed DEM rasters here (the build is in-RAM); we only report the wired stack so the
    serial-join is visible when running ``python -m terrain_authority.scenes``.
    """
    scene_dir = os.path.join(ROOT, "samples", "lunar_dem", "haworth_10km_5m")
    if not os.path.exists(os.path.join(scene_dir, "metadata.json")):
        print(f"  [build_from_dem] SKIP — no committed DEM scene at {scene_dir} "
              "(degrades gracefully; the nine legacy scenes are unaffected).")
        return
    try:
        fields, meta = build_from_dem(scene_dir)
    except Exception as exc:  # noqa: BLE001 — never let the DEM join break the legacy builders
        print(f"  [build_from_dem] SKIP — DEM build failed ({exc!r}); legacy scenes unaffected.")
        return
    h = fields["heightmap"]
    dc = meta.get("dem_corridor", {})
    cc = dc.get("conservation_check", {})
    print(f"  build_from_dem  region={meta.get('region')} grid={h.shape} "
          f"rho_bar={meta['density_source']['rho_bar_kg_m3']} kg/m^3  "
          f"fine_tiles={dc.get('fine_tiles_materialized')} "
          f"coarsen==base={cc.get('coarsen_equals_base')}  "
          f"lit={meta['illumination']['lit_fraction']:.3f}")


def main() -> int:
    os.makedirs(SAMPLES_DIR, exist_ok=True)
    print(f"Building sample scenes into {SAMPLES_DIR}")
    build_flat_compact()
    build_rolling_hills()
    build_crater()
    build_boulder_field()
    build_crater_boulders()
    build_crater_caveins()
    build_tread_track()
    # NEW render-fidelity scenes (spec §5; INTERFACE.md §5.2/§5.3). Additive; the seven
    # scenes above are byte-identical to before (HARD RULE 3).
    build_tread_track_4wheel()
    build_excavation_marks()
    # W2-SCENES (serial join): the DEM corridor builder, ALONGSIDE the nine legacy builders
    # above (which are byte-unchanged). Guarded so a missing committed DEM scene is non-fatal.
    build_dem_scene()
    print("done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
