"""Physics-weight coupling: the terramechanics update with how much weight the rover carries.

As the drum fills with regolith the rover gets heavier, so the SAME drive / compaction sinks deeper and
slips more -- the path-dependent loop (excavate -> heavier -> more sinkage -> more slip). These lock in
that the LIVE drum mass (WorkSite.inventory_kg / ColumnState.drum_inventory) feeds the weight that drives
the Bekker pressure-sinkage solve and the slip ladder, not just the dry mass.
"""
import math
import os

import pytest

from terrain_authority import constants as K
from terrain_authority import drive as D
from terrain_authority import rover as R
from terrain_authority import terramechanics as tm
from terrain_authority.column_state import ColumnState

BUNDLE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      "samples", "lunar_dem", "haworth_10km_5m")
_have_bundle = os.path.isdir(BUNDLE)


def test_static_wheel_load_scales_with_payload():
    dry = tm.static_wheel_load_n(payload_kg=0.0)
    laden = tm.static_wheel_load_n(payload_kg=K.DRUM_PAYLOAD_MAX_KG)
    assert laden > dry
    assert math.isclose(laden, (K.ROVER_MASS_DRY_KG + K.DRUM_PAYLOAD_MAX_KG) * K.g / K.N_WHEELS, rel_tol=1e-9)


def test_four_wheel_pass_payload_deepens_compaction_mass_conserved():
    # same loose surface + same track; a heavier rover (full drum) firms the regolith more.
    poses = [((32.0, 32.0), 0.0)]
    empty = ColumnState(width=64, height=64, cell_m=0.02)
    laden = ColumnState(width=64, height=64, cell_m=0.02)
    m0 = empty.total_mass()
    R.four_wheel_pass(empty, poses, physical=True, payload_kg=0.0)
    R.four_wheel_pass(laden, poses, physical=True, payload_kg=K.DRUM_PAYLOAD_MAX_KG)
    assert math.isclose(empty.total_mass(), m0, rel_tol=1e-9)        # density-only edits ...
    assert math.isclose(laden.total_mass(), m0, rel_tol=1e-9)        # ... mass conserved either way
    assert laden.density.max() > empty.density.max()                 # heavier -> firmer


def test_drive_step_deeper_sinkage_when_loaded():
    # the drive physics reads weight: a full drum sinks deeper than an empty one over the same step.
    empty = ColumnState(width=96, height=96, cell_m=0.05)
    laden = ColumnState(width=96, height=96, cell_m=0.05)
    _, _, t_empty = D.drive_step(empty, (48.0, 48.0), 0.0, 0.2, 0.0, payload_kg=0.0)
    _, _, t_laden = D.drive_step(laden, (48.0, 48.0), 0.0, 0.2, 0.0, payload_kg=K.DRUM_PAYLOAD_MAX_KG)
    assert t_laden["sinkage_m"] > t_empty["sinkage_m"]


@pytest.mark.skipif(not _have_bundle, reason="Haworth bundle absent")
def test_worksite_drive_defaults_payload_to_live_drum(monkeypatch):
    # WorkSite.drive must default the haul weight to the LIVE drum fill, not 0.
    import terrain_authority.worksite as WS
    ws = WS.WorkSite.from_haworth_bundle(BUNDLE, fine_cell_m=0.05, tile_base_cells=2)
    ws.open_window((1101, 1101), radius_m=4.0)
    ws.inventory_kg = 23.0
    seen = {}
    real = WS.D.closed_loop_drive

    def spy(*a, **kw):
        seen.update(kw)
        return real(*a, **kw)

    monkeypatch.setattr(WS.D, "closed_loop_drive", spy)
    ws.drive([(0.1, 0.0)], start_rc=(ws.fine.height // 2, ws.fine.width // 4), start_yaw=0.0)
    assert seen.get("payload_kg") == 23.0                            # the live drum mass flowed into the drive


@pytest.mark.skipif(not _have_bundle, reason="Haworth bundle absent")
def test_worksite_compact_over_forwards_live_drum_payload(monkeypatch):
    # compacting with a loaded drum presses harder: the live drum mass reaches four_wheel_pass.
    import terrain_authority.worksite as WS
    ws = WS.WorkSite.from_haworth_bundle(BUNDLE, fine_cell_m=0.05, tile_base_cells=2)
    ws.open_window((1101, 1101), radius_m=4.0)
    ws.inventory_kg = 17.0
    seen = {}
    real = WS.R.four_wheel_pass

    def spy(cs, poses, **kw):
        seen.update(kw)
        return real(cs, poses, **kw)

    monkeypatch.setattr(WS.R, "four_wheel_pass", spy)
    ws.compact_over([((ws.fine.height // 2, ws.fine.width // 2), 0.0)])
    assert seen.get("payload_kg") == 17.0
