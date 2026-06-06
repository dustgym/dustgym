"""WP0.1 (RB-01 / CT-01 / CT-02 / CT-06) — domain validation at the public boundary.

Two layers: the reusable `validation` helpers, and `ColumnState` rejecting NaN/Inf/negative/malformed
arrays AT CONSTRUCTION (not only via an explicit invariant call). The "bad" inputs here are deliberately
invalid values used to prove rejection — not synthetic data standing in for real measurements; the valid
case uses ColumnState's own constants-derived defaults.
"""
from __future__ import annotations

import numpy as np
import pytest

from terrain_authority import validation as V
from terrain_authority.column_state import ColumnState


# ---- the reusable validators ---------------------------------------------------------------
def test_domain_error_is_a_value_error():
    assert issubclass(V.DomainError, ValueError)


def test_ensure_finite_rejects_nan_and_inf():
    V.ensure_finite(np.zeros((2, 2)), "ok")                       # passes
    for bad in (np.nan, np.inf, -np.inf):
        a = np.zeros((2, 2)); a[0, 0] = bad
        with pytest.raises(V.DomainError, match="non-finite"):
            V.ensure_finite(a, "x")


def test_ensure_nonneg_and_positive():
    V.ensure_nonneg(np.array([0.0, 1.0]), "ok")
    with pytest.raises(V.DomainError, match="negative"):
        V.ensure_nonneg(np.array([0.0, -1e-9]), "x")
    V.ensure_positive(np.array([1.0, 2.0]), "ok")
    with pytest.raises(V.DomainError, match="non-positive"):
        V.ensure_positive(np.array([1.0, 0.0]), "x")


def test_ensure_range_shape_kind_scalars():
    V.ensure_range(np.array([0.0, 0.5, 1.0]), 0.0, 1.0, "ok")
    with pytest.raises(V.DomainError, match="outside"):
        V.ensure_range(np.array([0.0, 1.5]), 0.0, 1.0, "x")
    with pytest.raises(V.DomainError, match="shape"):
        V.ensure_shape(np.zeros((2, 3)), (3, 2), "x")
    with pytest.raises(V.DomainError, match="dtype kind"):
        V.ensure_kind(np.zeros(3, dtype=np.uint8), "f", "x")      # int where float required
    assert V.ensure_positive_scalar(0.02, "cell_m") == 0.02
    for fn, bad in ((V.ensure_positive_scalar, 0.0), (V.ensure_finite_scalar, np.nan),
                    (V.ensure_nonneg_scalar, -1.0)):
        with pytest.raises(V.DomainError):
            fn(bad, "x")


# ---- ColumnState rejects bad state AT CONSTRUCTION (CT-02) ----------------------------------
def _good() -> ColumnState:
    return ColumnState(width=4, height=3, cell_m=0.02)                          # constants-derived valid defaults


def test_valid_construction_passes_and_check_invariants_clean():
    cs = _good()
    cs.check_invariants()                                          # no raise
    assert cs.mass_areal.shape == (3, 4) and np.all(cs.density > 0)


def test_construction_rejects_nan_density():
    d = np.full((3, 4), 1500.0); d[0, 0] = np.nan
    with pytest.raises(V.DomainError, match="density"):
        ColumnState(width=4, height=3, cell_m=0.02, density=d)


def test_construction_rejects_nonpositive_density():
    d = np.full((3, 4), 1500.0); d[1, 1] = 0.0
    with pytest.raises(V.DomainError, match="density"):
        ColumnState(width=4, height=3, cell_m=0.02, density=d)


def test_construction_rejects_negative_mass():
    m = np.full((3, 4), 100.0); m[2, 3] = -1e-6
    with pytest.raises(V.DomainError, match="mass_areal"):
        ColumnState(width=4, height=3, cell_m=0.02, mass_areal=m)


def test_construction_rejects_inf_datum_and_wrong_shape():
    dat = np.zeros((3, 4)); dat[0, 0] = np.inf
    with pytest.raises(V.DomainError, match="datum"):
        ColumnState(width=4, height=3, cell_m=0.02, datum=dat)
    with pytest.raises(V.DomainError, match="shape"):
        ColumnState(width=4, height=3, cell_m=0.02, mass_areal=np.zeros((2, 2)))


def test_construction_rejects_negative_drum_inventory():
    with pytest.raises(V.DomainError, match="drum_inventory"):
        ColumnState(width=4, height=3, cell_m=0.02, drum_inventory=-1.0)


def test_construction_rejects_negative_ice():
    ice = np.zeros((3, 4)); ice[0, 1] = -0.01
    with pytest.raises(V.DomainError, match="ice"):
        ColumnState(width=4, height=3, cell_m=0.02, ice=ice)


def test_validate_false_allows_signed_scratch_column_only_when_opted_in():
    # the ONE documented exception: a signed height-residual scratch (datum=0, density=1) the
    # crater overlay uses; mass_areal may be < 0 there. _validate=False permits it; the default rejects.
    resid = np.array([[0.1, -0.2], [0.3, -0.05]])
    cs = ColumnState(width=2, height=2, cell_m=0.02, mass_areal=resid,
                     density=np.ones((2, 2)), datum=np.zeros((2, 2)), _validate=False)
    assert np.any(cs.mass_areal < 0)                          # signed scratch allowed when opted in
    with pytest.raises(V.DomainError, match="mass_areal"):    # default boundary still strict
        ColumnState(width=2, height=2, cell_m=0.02, mass_areal=resid,
                    density=np.ones((2, 2)), datum=np.zeros((2, 2)))


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn(); print(f"[PASS] {fn.__name__}")
    print(f"\n{len(fns)}/{len(fns)} validation checks passed.")


if __name__ == "__main__":
    _run_all()
