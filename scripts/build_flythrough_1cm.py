"""Generate the 1 cm global-resolution 4-wheel fly-through SHOWCASE scene (512x512 @ 0.01 m).

This is a *showcase deliverable* generator, not a canonical contract scene (those live in
``terrain_authority/scenes.py`` at 256x256 @ 0.02 m and are covered by the test suite). It
builds a driven IPEx four-wheel tread track over the full 5.12 m patch at GLOBAL 1 cm
resolution (Mode A, ``docs/render_fidelity_spec.md`` §2.1) so the Godot ``--sequence``
fly-through (``godot_sidecar/sidecar.gd``) renders genuinely 1 cm terrain with the render-
fidelity stack (4x MSAA + SMAA + 1.5x SSAA, detail-normal regolith, per-wheel cleat marks
oriented by the §5.2 ``wheel_tracks``) and the interaction-keyed quadtree LOD following the
rover.

Field 512 (power of two) x 0.01 m = 5.12 m: the SAME world extent as the 2 cm canonical
scenes, so the sidecar camera framing is unchanged and only the data/render resolution
doubles. The quadtree params are scaled to keep the SAME physical LOD behaviour as the 2 cm
scenes -- ``min_leaf`` 16 cells = 16 cm finest leaf, ``footprint_radius`` 11 cells = 11 cm
wheel half-width, ``refine_factor`` 0.5 -- i.e. an identical 5-level pyramid laid over
twice-as-fine data.

Each ``tNNN/`` frame is a full INTERFACE.md scene carrying the additive v1.0.1 per-frame
interaction-keyed quadtree keys (``rover_rc`` / ``active_leaves`` / ``quadtree_nodes`` /
``touched_leaves`` / ``quadtree_lod``) and the v1.0.2 §5.2 ``wheel_tracks`` (four contact
polylines + per-wheel heading). Mass is conserved -- ``four_wheel_pass`` is density-only
compaction. ``refinement``/``tiles`` are intentionally NOT emitted: the base is already 1 cm,
so corridor refinement is moot here (that Mode-B story is the 2 cm ``tread_track_4wheel``
scene). ``schema_version`` stays ``"1.0"``.

    python scripts/build_flythrough_1cm.py
then render the fly-through:
    cd godot_sidecar && ./render_layers.sh -- --sequence ../samples/tread_track_4wheel_1cm \
        --stride 2 --layers terrain,quadtree,rover
"""

from __future__ import annotations

import json
import os
import sys

import numpy as np

# Run standalone (python scripts/build_flythrough_1cm.py) by putting the repo root on the path.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from terrain_authority import constants as K  # noqa: E402
from terrain_authority import procgen
from terrain_authority.column_state import ColumnState
from terrain_authority.io_fields import save_scene, write_hillshade_png, write_preview_png
from terrain_authority.quadtree import QuadtreeTracker
from terrain_authority.rover import build_wheel_tracks_meta, four_wheel_pass

# --- grid (Mode A global 1 cm; 512 is a power of two so the quadtree bottoms out cleanly) ---
W = H = 512
CELL_M = 0.01                       # 1 cm -> 5.12 m patch (same extent as the 2 cm scenes)
N_FRAMES = 24                       # motion frames; + the pristine t000 -> 25 total

# Interaction-keyed quadtree, scaled to the SAME physical LOD as the 2 cm scenes.
QT_MIN_LEAF = 16                    # 16 cells @ 1 cm = 16 cm finest leaf (== 8 @ 2 cm)
QT_REFINE_FACTOR = 0.5
QT_FOOTPRINT_RADIUS_CELLS = 11.0    # 11 cells @ 1 cm = 11 cm wheel half-width (== 5.5 @ 2 cm)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(ROOT, "samples", "tread_track_4wheel_1cm")
NAME = "tread_track_4wheel_1cm"


def _waypoints() -> list[tuple[float, float]]:
    """Diagonal drive (row,col) waypoints: start -> mid bend -> end (same fractions as the
    2 cm tread scenes, so it reads as the same drive at finer resolution)."""
    return [(0.22 * H, 0.18 * W), (0.50 * H, 0.52 * W), (0.80 * H, 0.70 * W)]


def _segment(a: tuple[float, float], b: tuple[float, float]) -> list[tuple[float, float]]:
    (ra, ca), (rb, cb) = a, b
    n = int(max(abs(rb - ra), abs(cb - ca))) + 1
    return list(zip(np.linspace(ra, rb, n), np.linspace(ca, cb, n)))


def _dense_path() -> list[tuple[float, float]]:
    w = _waypoints()
    return _segment(w[0], w[1]) + _segment(w[1], w[2])[1:]


def _heading_at(path: list[tuple[float, float]], i: int) -> float:
    """Path-tangent heading in the §5.2 convention (0 = +col/+X, +pi/2 = +row/+Z).

    forward ~ (drow, dcol); heading = atan2(drow, dcol) so 0 faces +col and +pi/2 faces +row.
    Central difference where possible; one-sided at the ends.
    """
    j0 = max(0, i - 1)
    j1 = min(len(path) - 1, i + 1)
    drow = path[j1][0] - path[j0][0]
    dcol = path[j1][1] - path[j0][1]
    if abs(drow) < 1e-9 and abs(dcol) < 1e-9:
        return 0.0
    return float(np.arctan2(drow, dcol))


def _clone(cs: ColumnState) -> ColumnState:
    return ColumnState(width=cs.width, height=cs.height, cell_m=cs.cell_m,
                       mass_areal=cs.mass_areal.copy(), density=cs.density.copy(),
                       state_label=cs.state_label.copy(), disturbance=cs.disturbance.copy(),
                       datum=cs.datum.copy(),
                       ice=None if cs.ice is None else cs.ice.copy(),
                       drum_inventory=cs.drum_inventory)


def _height_range(cs: ColumnState) -> list[float]:
    h = cs.derive_height()
    return [round(float(h.min()), 5), round(float(h.max()), 5)]


def _meta(frame_cs: ColumnState, frame_idx: int, *, az: dict, qt_static: list,
          qt_result, rover_rc, touched_boxes, wheel_tracks) -> dict:
    """Full INTERFACE.md v1.0 metadata + additive v1.0.1 quadtree keys + v1.0.2 wheel_tracks."""
    x1 = round(W * CELL_M, 4)
    meta = {
        "schema_version": "1.0",
        "scene_name": NAME,
        "producer": "scripts/build_flythrough_1cm.py (NumPy Tier-2 surrogate, global 1 cm)",
        "grid": {"width": W, "height": H, "cell_m": CELL_M, "order": "row-major-C"},
        "world_bounds_m": {"x0": 0.0, "y0": 0.0, "x1": x1, "y1": x1},
        "gravity_m_s2": K.g,
        "fields": {
            "heightmap": {"file": "heightmap.rf32", "dtype": "<f4", "units": "m"},
            "mass_areal": {"file": "mass_areal.rf32", "dtype": "<f4", "units": "kg/m^2"},
            "density": {"file": "density.rf32", "dtype": "<f4", "units": "kg/m^3"},
            "disturbance": {"file": "disturbance.rf32", "dtype": "<f4", "units": "1 (normalized)"},
            "state_label": {"file": "state_label.r8", "dtype": "u1", "enum": K.STATE_NAMES},
        },
        "ice_present": False,
        "height_range_m": _height_range(frame_cs),
        "clasts": [],
        "active_zone": az,
        "quadtree": qt_static,
        "frame_index": frame_idx,
        "notes": f"GLOBAL 1 cm fly-through frame {frame_idx}; four-wheel drive over a 5.12 m "
                 f"patch at 0.01 m/cell. Mass-conserving compaction; render-fidelity shaders.",
    }
    # ADDITIVE v1.0.1 per-frame interaction-keyed quadtree (INTERFACE.md §5.1).
    meta["active_leaves"] = qt_result.boxes("active")
    meta["quadtree_nodes"] = qt_result.nodes
    meta["touched_leaves"] = touched_boxes
    meta["rover_rc"] = ([int(round(rover_rc[0])), int(round(rover_rc[1]))]
                        if rover_rc is not None else None)
    meta["quadtree_lod"] = {
        "min_leaf": qt_result.min_leaf, "refine_factor": QT_REFINE_FACTOR,
        "footprint_radius_cells": QT_FOOTPRINT_RADIUS_CELLS, "field_size": qt_result.field_size,
    }
    # ADDITIVE v1.0.2 §5.2 per-wheel tracks (orient the shader cleat detail).
    if wheel_tracks is not None:
        meta["wheel_tracks"] = wheel_tracks
        meta["features"] = ["wheel_tracks"]
    return meta


def main() -> int:
    os.makedirs(SCENE_DIR, exist_ok=True)
    print(f"Building {NAME} into {SCENE_DIR}  ({W}x{H} @ {CELL_M} m = {W*CELL_M:.2f} m, "
          f"{N_FRAMES} motion frames)")

    # Base terrain: rolling hills at 1 cm. base_cells doubled (6) vs the 2 cm scene (3) so the
    # physical hill wavelength matches at twice the resolution.
    cs = procgen.rolling_hills(W, H, CELL_M, seed=11, amplitude_m=0.12, base_cells=6)
    mass_before = cs.total_mass()

    path = _dense_path()
    chunks = [c for c in np.array_split(np.arange(len(path)), N_FRAMES) if len(c)]

    # Active zone / static quadtree bound the driven corridor (consumer fallback).
    rows = [p[0] for p in path]
    cols = [p[1] for p in path]
    rmin, rmax = max(0, int(min(rows)) - 24), min(H, int(max(rows)) + 24)
    cmin, cmax = max(0, int(min(cols)) - 24), min(W, int(max(cols)) + 24)
    az = {"min_rc": [rmin, cmin], "max_rc": [rmax, cmax]}
    qt_static = [
        {"level": 0, "row0": 0, "col0": 0, "size": W, "label": "ROOT"},
        {"level": 1, "row0": rmin, "col0": cmin,
         "size": max(rmax - rmin, cmax - cmin), "label": "ACTIVE"},
    ]

    tracker = QuadtreeTracker(field_size=W, min_leaf=QT_MIN_LEAF,
                              refine_factor=QT_REFINE_FACTOR,
                              footprint_radius_cells=QT_FOOTPRINT_RADIUS_CELLS)

    # Frame 0: pristine, pre-drive (no rover on the field).
    frames = [_clone(cs)]
    centers: list[tuple[float, float] | None] = [None]
    wheel_tracks_per_frame: list[dict | None] = [None]

    for chunk in chunks:
        sub = [path[k] for k in chunk]
        poses = [(sub[j], _heading_at(path, int(chunk[j]))) for j in range(len(sub))]
        polylines = four_wheel_pass(cs, poses, wheel_width_m=0.18, compaction=0.14)
        frames.append(_clone(cs))
        centers.append(tuple(sub[-1]))
        heading = _heading_at(path, int(chunk[-1]))
        wheel_tracks_per_frame.append(
            build_wheel_tracks_meta(polylines, heading, cell_m=CELL_M, width_m=0.18))
    mass_after = cs.total_mass()

    # Per-frame quadtree snapshots (active set follows the rover; touched grows monotonically).
    qt_per_frame, qt_touched = [], []
    for ctr in centers:
        qt_per_frame.append(tracker.step(ctr))
        qt_touched.append(tracker.touched_boxes())

    for i, frame_cs in enumerate(frames):
        tdir = os.path.join(SCENE_DIR, f"t{i:03d}")
        meta = _meta(frame_cs, i, az=az, qt_static=qt_static, qt_result=qt_per_frame[i],
                     rover_rc=centers[i], touched_boxes=qt_touched[i],
                     wheel_tracks=wheel_tracks_per_frame[i])
        save_scene(tdir, frame_cs.fields_dict(), meta)

    # Bookend previews (t000 + final) for human inspection.
    last = len(frames) - 1
    for idx in (0, last):
        write_hillshade_png(frames[idx].derive_height(),
                            os.path.join(SCENE_DIR, f"t{idx:03d}", "preview_hillshade.png"),
                            CELL_M, altdeg=K.SUN_ELEVATION_DEG_POLAR,
                            title=f"{NAME}_t{idx:03d} hillshade")
        write_preview_png(frames[idx].disturbance,
                          os.path.join(SCENE_DIR, f"t{idx:03d}", "preview_disturbance.png"),
                          cmap="magma", title=f"{NAME}_t{idx:03d} disturbance")
        write_preview_png(frames[idx].state_label,
                          os.path.join(SCENE_DIR, f"t{idx:03d}", "preview_state.png"),
                          cmap="tab10", title=f"{NAME}_t{idx:03d} state")

    # Parent metadata (time-series cadence + the per-frame-keys advertisement).
    parent = _meta(frames[-1], last, az=az, qt_static=qt_static, qt_result=qt_per_frame[-1],
                   rover_rc=centers[-1], touched_boxes=qt_touched[-1],
                   wheel_tracks=wheel_tracks_per_frame[-1])
    parent.pop("frame_index", None)
    parent["notes"] = ("SHOWCASE: global 1 cm (Mode A) four-wheel fly-through over a 5.12 m "
                       "patch (512x512 @ 0.01 m). Generated by scripts/build_flythrough_1cm.py; "
                       "NOT a canonical scenes.py sample. Mass conserved (density-only "
                       "compaction).")
    parent["time_series"] = {
        "frame_count": len(frames),
        "frame_cadence_steps": 1,
        "frame_dirs": [f"t{i:03d}" for i in range(len(frames))],
        "mass_conserved_kg": round(mass_after, 6),
        "mass_drift_kg": round(abs(mass_after - mass_before), 9),
    }
    parent["quadtree_lod"] = {
        "min_leaf": QT_MIN_LEAF, "refine_factor": QT_REFINE_FACTOR,
        "footprint_radius_cells": QT_FOOTPRINT_RADIUS_CELLS, "field_size": W,
        "per_frame_keys": ["active_leaves", "quadtree_nodes", "touched_leaves", "rover_rc",
                           "wheel_tracks"],
        "note": "global 1 cm Mode-A scene; interaction-keyed quadtree LOD + per-frame "
                "wheel_tracks. Optional; ignorable.",
    }
    parent["features"] = ["wheel_tracks"]
    with open(os.path.join(SCENE_DIR, "metadata.json"), "w") as fh:
        json.dump(parent, fh, indent=2)

    n_active = len(qt_per_frame[-1].active_leaves)
    print(f"  wrote {NAME}  frames={len(frames)}  "
          f"mass_before={mass_before:.4f} mass_after={mass_after:.4f} kg "
          f"drift={abs(mass_after - mass_before):.2e} kg  qt active(last)={n_active} "
          f"touched(total)={len(tracker.touched_leaves())}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
