"""N14: ColumnState public-constructor validation + runtime invariant guards (production reliability).

These make the conserved-state guarantees checkable at runtime (not only in the legacy assertion tests):
a malformed grid is rejected at construction, and the invariants / mass conservation can be asserted live.
"""
import numpy as np
import pytest

from .column_state import ColumnState


def test_constructor_rejects_nonpositive_dims_and_cell():
    with pytest.raises(ValueError):
        ColumnState(0, 0, 0.02)          # zero-size grid
    with pytest.raises(ValueError):
        ColumnState(8, 8, -0.02)         # negative cell size
    with pytest.raises(ValueError):
        ColumnState(8, 8, 0.0)           # zero cell size


def test_constructor_accepts_valid():
    cs = ColumnState(8, 8, 0.02)
    assert cs.width == 8 and cs.height == 8 and cs.cell_m == 0.02


def test_check_invariants_passes_on_fresh_state():
    ColumnState(16, 16, 0.02).check_invariants()        # no raise on a well-formed state


def test_check_invariants_catches_density_mass_and_nonfinite():
    cs = ColumnState(8, 8, 0.02)
    cs.density[0, 0] = 0.0
    with pytest.raises(ValueError, match="density"):
        cs.check_invariants()
    cs2 = ColumnState(8, 8, 0.02)
    cs2.mass_areal[0, 0] = -1.0
    with pytest.raises(ValueError, match="mass"):
        cs2.check_invariants()
    cs3 = ColumnState(8, 8, 0.02)
    cs3.mass_areal[0, 0] = np.inf
    with pytest.raises(ValueError):
        cs3.check_invariants()


def test_conserves_mass_guard():
    cs = ColumnState(8, 8, 0.02)
    with cs.conserves_mass():            # a no-op block conserves mass -> no raise
        pass
    with pytest.raises(ValueError, match="conserved"):
        with cs.conserves_mass():        # creating mass inside the block must raise
            cs.mass_areal += 1.0
