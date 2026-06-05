"""Regression tests for the streaming WorkSite (recenter / worked-store / G7 smooth_datum).

Covers the seam the worksite_roam.py demo runs on and the defects the 2026-06-02 adversarial
review surfaced: the conservation_residual() sensitivity within the first window, the
open_window<->recenter entry-path guard, G7 conservation-neutrality + seam-freeness, and that
assemble_region() stitches a conserving corridor. Fast (small windows over the committed Haworth).
"""
from __future__ import annotations

import os

import numpy as np
import pytest

from terrain_authority.column_state import StateLabel
from terrain_authority.worksite import WorkSite

BUNDLE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      "samples", "lunar_dem", "haworth_10km_5m")
pytestmark = pytest.mark.skipif(not os.path.isdir(BUNDLE), reason="committed Haworth bundle absent")


def _site(**kw):
    return WorkSite.from_haworth_bundle(BUNDLE, fine_cell_m=0.05, tile_base_cells=2, **kw)


def _xy(site, br, bc):
    return (site.world_x0 + bc * site.base_cell_m, site.world_y0 + br * site.base_cell_m)


def test_recenter_conserves_across_slides():
    """dig in window #1, slide, dump in window #2 — residual stays < 1e-6 the whole way."""
    s = _site()
    x0, y0 = _xy(s, 1101, 1101)
    s.recenter((x0, y0))
    base = s._baseline_virgin_kg
    f = s.fine; H, W = f.height, f.width
    m = np.zeros((H, W), bool); m[H // 2 - 60:H // 2 + 60, W // 2 - 60:W // 2 + 60] = True
    s.flatten(m, float(f.derive_height()[m].mean() - 0.3))
    assert s.conservation_residual() / base < 1e-6
    s.recenter((x0 + 12.0, y0))                              # slide > 1 tile -> real recenter
    assert s.conservation_residual() / s._baseline_virgin_kg < 1e-6
    f2 = s.fine; b = np.zeros((f2.height, f2.width), bool)
    b[f2.height // 2 - 30:f2.height // 2 + 30, f2.width // 2 - 20:f2.width // 2 + 20] = True
    s.dump(b, kg=s.inventory_kg * 0.5)
    assert s.conservation_residual() / s._baseline_virgin_kg < 1e-6


def test_residual_sensitive_in_first_window():
    """The review's finding [1]: conservation_residual() must DETECT a mass leak even inside the
    first window (before worked_store is populated by the 2nd recenter). Used to return blind 0.0."""
    s = _site()
    s.recenter(_xy(s, 1101, 1101))
    assert s.worked_store == {}                              # first window: store still empty
    assert s.conservation_residual() == 0.0                  # no work yet -> genuinely zero
    s.fine.mass_areal[0, 0] += 9999.0 / s.fine.cell_area     # inject a known 9999 kg leak
    assert abs(s.conservation_residual() - 9999.0) < 1e-3    # detected, not masked


def test_open_window_then_recenter_is_guarded():
    """The review's finding [2]: mixing the single-window (open_window) and streaming (recenter)
    entry paths silently discards worked state — now a hard error instead."""
    s = _site()
    s.open_window((1101, 1101), radius_m=4.0)
    with pytest.raises(RuntimeError, match="mutually exclusive"):
        s.recenter(_xy(s, 1101, 1101))


def test_smooth_datum_is_conservation_neutral():
    """G7: bilinear datum smoothing rewrites only datum (no mass) -> grid_mass identical to the bit."""
    a = _site(smooth_datum=False); a.recenter(_xy(a, 1101, 1101))
    b = _site(smooth_datum=True);  b.recenter(_xy(b, 1101, 1101))
    assert a.fine.grid_mass() == b.fine.grid_mass()          # exact, not approx
    assert a._baseline_virgin_kg == b._baseline_virgin_kg
    # and it actually removes the terrace cliffs
    da = np.abs(np.diff(a.fine.derive_height()[a.fine.height // 2]))
    db = np.abs(np.diff(b.fine.derive_height()[b.fine.height // 2]))
    assert db.max() < 0.1 < da.max()                          # smoothed << terraced


def test_smooth_datum_seam_free():
    """The smoothed datum is a pure function of global fine index -> two horizontally-adjacent virgin
    tiles are bit-exact slices of ONE continuous global bilinear field (no window-crop seam)."""
    s = _site(smooth_datum=True)
    left = s._virgin_tile_fields(200, 200)["datum"]
    right = s._virgin_tile_fields(200, 201)["datum"]
    r0, c0, _r1, _c1 = s._tile_region(200, 200)
    wl, wr = left.shape[1], right.shape[1]
    span = s._bilinear_datum_block(r0 * s.k, c0 * s.k, left.shape[0], wl + wr)  # both tiles in one block
    assert np.array_equal(span[:, :wl], left)               # left tile == its slice of the global field
    assert np.array_equal(span[:, wl:], right)              # right tile == the adjacent slice -> seam-free
    # and the join is continuous (one fine-cell step, no terrace jump) across the shared seam
    assert abs(float(left[s.k, -1] - right[s.k, 0])) < 0.05


def test_assemble_region_conserves():
    """The assembled corridor (worked store U live-active U virgin) holds the conserved total."""
    s = _site()
    x0, y0 = _xy(s, 1101, 1101)
    s.recenter((x0, y0))
    f = s.fine; m = np.zeros((f.height, f.width), bool)
    m[f.height // 2 - 40:f.height // 2 + 40, f.width // 2 - 40:f.width // 2 + 40] = True
    s.flatten(m, float(f.derive_height()[m].mean() - 0.3))
    s.recenter((x0 + 12.0, y0))
    cor, _origin = s.assemble_region()
    # corridor covers exactly the visited (contiguous) tiles -> grid + ledger == cumulative virgin
    assert abs(cor.grid_mass() + s.inventory_kg - s._baseline_virgin_kg) / s._baseline_virgin_kg < 1e-9
    assert int((cor.state_label == int(StateLabel.EXCAVATED)).sum()) > 0
