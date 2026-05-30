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
from .column_state import ColumnState
from .io_fields import save_scene, write_hillshade_png, write_preview_png
from .quadtree import QuadtreeTracker
from .rover import straight_path, wheel_pass
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
    print("done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
