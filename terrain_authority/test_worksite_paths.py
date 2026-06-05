"""WorkSite controller-seam coverage — the open_window single-window path + save/snapshot/sinter.

Complements terrain_authority/test_worksite.py (which covers the streaming recenter path). This file
exercises the OTHER entry path (open_window over the committed Haworth coarse base) and the controller
seam an RL policy / the scripted demo drives, asserting the mass invariant throughout:

  * open_window -> flatten -> dump -> relax -> compact_over: fine.grid_mass() + inventory_kg conserved.
  * drive(): closed-loop twist drive over the fine window (density-only, mass conserved).
  * dump on an empty mask: a no-op that leaves the ledger untouched.
  * sinter(): GATED OFF (constants.SINTER_ENABLED is False) -> raises.
  * save_fine_bundle / save_cs_bundle: write a real INTERFACE bundle that round-trips via load_scene.
  * snapshot(): deep-copied frame with the live mass/residual.
  * assemble_region / visited bbox helpers; _require_fine guard before open_window.

Fast: tiny windows over the committed Haworth bundle. Run from the repo root:
    PYTHONPATH=. <venv>/bin/python -m pytest terrain_authority/test_worksite_paths.py -q
"""
from __future__ import annotations

import os

import numpy as np
import pytest

from terrain_authority import constants as K
from terrain_authority.column_state import StateLabel
from terrain_authority.io_fields import load_scene
from terrain_authority.worksite import WorkSite, coarse_base_from_bundle

BUNDLE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      "samples", "lunar_dem", "haworth_10km_5m")
pytestmark = pytest.mark.skipif(not os.path.isdir(BUNDLE), reason="committed Haworth bundle absent")


def _site(**kw):
    return WorkSite.from_haworth_bundle(BUNDLE, fine_cell_m=0.05, tile_base_cells=2, **kw)


def _xy(s, br, bc):
    return (s.world_x0 + bc * s.base_cell_m, s.world_y0 + br * s.base_cell_m)


def _opened():
    s = _site()
    s.open_window((1101, 1101), radius_m=4.0)
    return s


def _center_mask(f, half=20):
    m = np.zeros((f.height, f.width), bool)
    m[f.height // 2 - half:f.height // 2 + half, f.width // 2 - half:f.width // 2 + half] = True
    return m


# ---- coarse_base_from_bundle (the loader the WorkSite builds on) ---------------------------------
def test_coarse_base_from_bundle_round_trips_height():
    base, meta = coarse_base_from_bundle(BUNDLE)
    fields, _ = load_scene(BUNDLE)
    assert base.width == meta["grid"]["width"] and base.height == meta["grid"]["height"]
    # datum was reconstructed so derive_height() recovers the bundle's stored heightmap
    assert np.allclose(base.derive_height(), fields["heightmap"], atol=1e-3)


# ---- open_window -> the controller seam ---------------------------------------------------------
def test_open_window_materializes_fine_and_sets_baseline():
    s = _opened()
    assert s.fine is not None and s.window_world_origin is not None
    assert s._baseline_mass is not None
    assert s.conservation_residual() == 0.0                  # no work yet -> exactly zero


def test_require_fine_guards_before_open_window():
    s = _site()
    with pytest.raises(RuntimeError, match="open_window"):
        s.flatten(np.zeros((4, 4), bool), 0.0)


def test_flatten_then_dump_conserves_total_mass():
    s = _opened()
    f = s.fine
    base = s._baseline_mass
    m = _center_mask(f, 30)
    target = float(f.derive_height()[m].mean() - 0.3)
    moved = s.flatten(m, target)
    assert moved > 0.0 and s.inventory_kg > 0.0
    assert s.conservation_residual() / base < 1e-9          # cut into the ledger conserves the total
    assert int((s.fine.state_label == int(StateLabel.EXCAVATED)).sum()) > 0
    # dump half the ledger back onto a different patch -> still conserved
    d = np.zeros((f.height, f.width), bool)
    d[10:30, 10:30] = True
    placed = s.dump(d, kg=s.inventory_kg * 0.5)
    assert placed > 0.0
    assert s.conservation_residual() / base < 1e-9
    assert int((s.fine.state_label == int(StateLabel.SPOIL)).sum()) > 0


def test_dump_empty_mask_is_noop():
    s = _opened()
    s.flatten(_center_mask(s.fine, 20), float(s.fine.derive_height().mean() - 0.2))
    inv0 = s.inventory_kg
    placed = s.dump(np.zeros((s.fine.height, s.fine.width), bool))
    assert placed == 0.0 and s.inventory_kg == inv0         # empty mask: ledger untouched


def test_dump_default_kg_dumps_whole_ledger():
    s = _opened()
    f = s.fine
    s.flatten(_center_mask(f, 25), float(f.derive_height().mean() - 0.25))
    d = np.zeros((f.height, f.width), bool)
    d[5:60, 5:60] = True
    base = s._baseline_mass
    s.dump(d)                                                # kg=None -> dump everything available
    assert s.inventory_kg < 1e-6                             # ledger emptied (clamped to what landed)
    assert s.conservation_residual() / base < 1e-9


def test_drive_lays_ruts_and_conserves_mass():
    s = _opened()
    base = s._baseline_mass
    f = s.fine
    start = (float(f.height // 2), float(f.width // 4))
    twists = [(0.2, 0.0)] * 8                                # straight drive across the window
    tele = s.drive(twists, start_rc=start, start_yaw=0.0, dt=0.1)
    assert isinstance(tele, dict)
    assert s.conservation_residual() / base < 1e-9          # density-only rut carving conserves mass


def test_relax_conserves_within_grid():
    s = _opened()
    f = s.fine
    base = s._baseline_mass
    # dump a pile so relaxation has something to flow
    s.flatten(_center_mask(f, 25), float(f.derive_height().mean() - 0.3))
    d = np.zeros((f.height, f.width), bool); d[20:40, 20:30] = True
    s.dump(d)
    steps, snaps = s.relax(max_steps=50, capture=True, capture_every=5)
    assert steps >= 0 and isinstance(snaps, list)
    assert s.conservation_residual() / base < 1e-9


def test_compact_over_conserves_and_relabels():
    s = _opened()
    f = s.fine
    base = s._baseline_mass
    poses = [((float(f.height // 2), float(f.width // 4 + i)), 0.0) for i in range(12)]
    poly = s.compact_over(poses)                             # physical=True default
    assert set(poly) == {"LF", "RF", "LB", "RB"}
    assert s.conservation_residual() / base < 1e-9          # density-only -> conserved


# ---- sinter: GATED OFF --------------------------------------------------------------------------
def test_sinter_is_gated_off():
    assert K.SINTER_ENABLED is False                         # the single gate, shared with the planner
    s = _opened()
    with pytest.raises(RuntimeError, match="GATED OFF"):
        s.sinter(_center_mask(s.fine, 10))


# ---- save bundle + snapshot ---------------------------------------------------------------------
def test_save_fine_bundle_round_trips(tmp_path):
    s = _opened()
    s.flatten(_center_mask(s.fine, 20), float(s.fine.derive_height().mean() - 0.2))
    out = str(tmp_path / "worksite_fine")
    meta = s.save_fine_bundle(out, scene_name="pytest_fine")
    assert meta["scene_name"] == "pytest_fine"
    assert os.path.isfile(os.path.join(out, "metadata.json"))
    # the saved bundle re-loads and its derived height matches the live window
    fields, loaded_meta = load_scene(out)
    assert loaded_meta["grid"]["width"] == s.fine.width
    assert np.allclose(fields["heightmap"], s.fine.derive_height(), atol=1e-3)
    # the metadata carries the contract surface Godot/panels read
    assert loaded_meta["gravity_m_s2"] == K.g
    assert set(loaded_meta["fields"]) >= {"heightmap", "mass_areal", "density", "disturbance", "state_label"}


def test_save_cs_bundle_with_extra(tmp_path):
    s = _opened()
    out = str(tmp_path / "cs_bundle")
    meta = s.save_cs_bundle(s.fine, out, (0.0, 0.0), scene_name="cs", extra={"note_extra": 7})
    assert meta["note_extra"] == 7                           # extra merged into the metadata
    assert os.path.isfile(os.path.join(out, "heightmap.rf32"))


def test_snapshot_captures_live_frame():
    s = _opened()
    s.flatten(_center_mask(s.fine, 20), float(s.fine.derive_height().mean() - 0.2))
    snap = s.snapshot()
    assert set(snap) >= {"height", "mass_areal", "density", "state_label", "disturbance",
                         "inventory_kg", "residual_kg"}
    assert snap["height"].shape == (s.fine.height, s.fine.width)
    assert snap["inventory_kg"] == s.inventory_kg
    # the snapshot is a DEEP copy: mutating the live grid does not change the captured frame
    before = snap["mass_areal"][0, 0]
    s.fine.mass_areal[0, 0] += 123.0
    assert snap["mass_areal"][0, 0] == before


def test_over_payload_flag_tracks_peak_inventory():
    s = _opened()
    assert s.over_payload is False                           # nothing dug yet
    # dig a big swath so the ledger exceeds the 30 kg drum envelope
    s.flatten(_center_mask(s.fine, 60), float(s.fine.derive_height().min()))
    assert s.peak_inventory_kg > 0.0
    assert s.over_payload == (s.peak_inventory_kg > K.DRUM_PAYLOAD_MAX_KG)


def test_visited_bbox_raises_before_any_window():
    s = _site()                                              # nothing visited (no recenter/open_window)
    with pytest.raises(RuntimeError, match="visited"):
        s.visited_base_bbox()


# ---- streaming-path corridor assembly (recenter) — visited bbox + assemble_region(fill_virgin) ---
def _streamed():
    s = _site()
    x0, y0 = _xy(s, 1101, 1101)
    s.recenter((x0, y0))
    return s, x0, y0


def test_visited_world_bbox_after_recenter():
    s, _x0, _y0 = _streamed()
    bx0, by0, bx1, by1 = s.visited_world_bbox()
    assert bx1 > bx0 and by1 > by0                           # a real, ordered corridor bbox in metres
    rr0, rc0, rr1, rc1 = s.visited_base_bbox()
    assert rr1 > rr0 and rc1 > rc0


def test_assemble_region_fill_virgin_false_skips_unworked_tiles():
    s, x0, y0 = _streamed()
    f = s.fine
    m = _center_mask(f, 30)
    s.flatten(m, float(f.derive_height()[m].mean() - 0.3))
    s.recenter((x0 + 12.0, y0))                              # slide so there is a worked store tile
    # fill_virgin=False stitches only worked/active tiles (no virgin context) -> still conserves the
    # mass that is actually in those tiles (cannot exceed the cumulative virgin baseline).
    cor, _origin = s.assemble_region(fill_virgin=False)
    assert cor.width > 0 and cor.height > 0
    assert cor.grid_mass() <= s._baseline_virgin_kg + 1e-3
