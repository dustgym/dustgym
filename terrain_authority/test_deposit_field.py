"""Tests for ColumnState.deposit_field / fill_toward (FIX-4: per-cell deficit-aware deposit).

The conserved authority had a per-cell CUT field (cut_to_inventory) but only an even-spread,
spoil-bulking DUMP (dump_from_inventory), which overshoots cells already near target on an uneven
deficit -> build/fill (haul, build_berm) could not converge. These tests pin the per-cell deposit:
no overshoot, mass conserved, drum-limited scaling, the volume-preserving height-rise identity, and
an end-to-end cut-haul-fill that reaches a raised target. Host-runnable + pytest; numpy only.
"""
from __future__ import annotations

import math

import numpy as np

from terrain_authority import constants as K
from terrain_authority.column_state import ColumnState, StateLabel


def _cs(grid=20, cell_m=0.05, mass_areal=None):
    kw = {}
    if mass_areal is not None:
        kw["mass_areal"] = np.full((grid, grid), float(mass_areal), dtype=np.float64)
    return ColumnState(width=grid, height=grid, cell_m=cell_m, **kw)


def test_height_rise_identity():
    """Depositing areal mass `a` raises a cell's height by exactly a/spoil_density (volume-preserving mix)."""
    cs = _cs(); cs.drum_inventory = 100.0
    cell = (5, 5); mask = np.zeros((cs.height, cs.width), bool); mask[cell] = True
    h0 = cs.derive_height()[cell]
    areal = 40.0                                            # kg/m^2 to place on the one cell
    cs.deposit_field(mask, np.where(mask, areal, 0.0))
    h1 = cs.derive_height()[cell]
    assert math.isclose(h1 - h0, areal / K.RHO_SPOIL, rel_tol=1e-9)


def test_fill_toward_no_overshoot():
    """fill_toward never raises any cell above its target, even with an UNEVEN deficit."""
    cs = _cs(); cs.drum_inventory = 500.0
    a, b, c, d = 4, 4, 12, 12
    mask = np.zeros((cs.height, cs.width), bool); mask[a:c, b:d] = True
    base = cs.derive_height()
    # uneven target: a ramp of deficits across the region (some cells need 6 cm, some 1 cm)
    target = base.copy()
    ramp = np.linspace(0.01, 0.06, c - a)[:, None] * np.ones((1, d - b))
    target[a:c, b:d] = base[a:c, b:d] + ramp
    for _ in range(40):
        if cs.drum_inventory <= 0:
            cs.drum_inventory = 500.0                       # keep supplied
        cs.fill_toward(mask, target, max_lift_m=0.02)
    h = cs.derive_height()
    over = (h[a:c, b:d] - target[a:c, b:d])
    assert over.max() <= 1e-9, over.max()                   # NO cell exceeds target
    assert np.abs(over).max() <= 1e-3                       # and it converged to target


def test_even_spread_overshoots_per_cell_does_not():
    """Regression contrast: dump_from_inventory (even spread) overshoots an uneven deficit; fill_toward does not."""
    a, b, c, d = 6, 6, 10, 10
    mask = np.zeros((20, 20), bool); mask[a:c, b:d] = True
    # one cell is already AT target (zero deficit); the rest need lifting
    def setup():
        cs = _cs(); cs.drum_inventory = 50.0
        base = cs.derive_height(); target = base.copy()
        target[a:c, b:d] = base[a:c, b:d] + 0.05
        target[a, b] = base[a, b]                           # this cell must NOT be raised
        return cs, target
    cs1, t1 = setup()
    cs1.dump_from_inventory(mask, 20.0)                     # even spread -> dumps onto the zero-deficit cell too
    assert cs1.derive_height()[a, b] > t1[a, b] + 1e-4      # overshoots the at-target cell
    cs2, t2 = setup()
    cs2.fill_toward(mask, t2, max_lift_m=0.05)              # per-cell -> places nothing on the at-target cell
    assert math.isclose(cs2.derive_height()[a, b], t2[a, b], abs_tol=1e-9)


def test_mass_conserved_cut_then_fill():
    """Cut excess into the drum, fill a deficit elsewhere from it: total (grid + drum) mass is conserved."""
    cs = _cs(); m0 = cs.total_mass()
    src = np.zeros((cs.height, cs.width), bool); src[2:6, 2:6] = True
    dst = np.zeros((cs.height, cs.width), bool); dst[14:18, 14:18] = True
    cs.cut_to_inventory(src, 30.0)                          # 30 kg/m^2 areal into drum
    cs.fill_toward(dst, cs.derive_height() + 0.05, max_lift_m=0.05)
    assert math.isclose(cs.total_mass(), m0, rel_tol=1e-12)


def test_drum_limited_scaling():
    """Requesting more than the drum holds places exactly the inventory, scaled across cells, drum -> 0."""
    cs = _cs(); cs.drum_inventory = 5.0
    mask = np.zeros((cs.height, cs.width), bool); mask[3:9, 3:9] = True
    placed = cs.deposit_field(mask, np.where(mask, 1000.0, 0.0))   # ask for far more than 5 kg
    assert math.isclose(placed, 5.0, rel_tol=1e-12)
    assert math.isclose(cs.drum_inventory, 0.0, abs_tol=1e-12)


def test_bare_cell_gets_spoil_density():
    cs = _cs(mass_areal=0.0); cs.drum_inventory = 10.0      # bare grid (mass_areal = 0)
    cell = (1, 1); mask = np.zeros((cs.height, cs.width), bool); mask[cell] = True
    cs.deposit_field(mask, np.where(mask, 20.0, 0.0))
    assert math.isclose(cs.density[cell], K.RHO_SPOIL, rel_tol=1e-9)
    assert cs.state_label[cell] == StateLabel.SPOIL


def test_haul_fill_converges_to_raised_target():
    """End-to-end: cut from a borrow region, haul (drum), fill a SEPARATED raised build pad to spec.
    This is the build_berm / cut-haul-fill primitive that even-spread dump could not solve."""
    cs = _cs(grid=24); m0 = cs.total_mass()
    borrow = np.zeros((cs.height, cs.width), bool); borrow[2:10, 2:10] = True
    build = np.zeros((cs.height, cs.width), bool); build[14:22, 14:22] = True
    base = cs.derive_height(); target = base.copy()
    target[14:22, 14:22] = base[14:22, 14:22] + 0.06        # raise the pad 6 cm
    for _ in range(60):
        # batch: top up the drum from borrow, then fill toward the pad target
        cs.cut_to_inventory(borrow, 8.0)                    # 8 kg/m^2 areal per step into drum
        cs.fill_toward(build, target, max_lift_m=0.02)
        if cs.derive_height()[14:22, 14:22].min() >= target[14:22, 14:22].min() - 1e-4:
            break
    h = cs.derive_height()
    assert h[14:22, 14:22].min() >= base[14:22, 14:22].min() + 0.06 - 2e-3   # pad reached spec
    assert (h[14:22, 14:22] - target[14:22, 14:22]).max() <= 1e-9            # no overshoot
    assert math.isclose(cs.total_mass(), m0, rel_tol=1e-12)                  # mass conserved


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn(); print(f"[PASS] {fn.__name__}")
    print(f"\n{len(fns)}/{len(fns)} deposit_field checks passed.")


if __name__ == "__main__":
    _run_all()
