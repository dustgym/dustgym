"""validation.py — shared domain validation for the conserved authority's public boundaries (PRD CT-01/06, RB-01).

One place for "reject non-finite/negative/out-of-domain physical values," raising EXPLICIT exceptions
(never a removable `assert`, CT-06) so the checks survive `python -O` and run in production/CI. Callers
at public boundaries (ColumnState construction + mutation, scene load, planner inputs) use these instead
of re-implementing ad-hoc checks.

`DomainError` is a `ValueError` subclass so existing `except ValueError` handlers and tests still catch
it, while new code can catch the narrower type.
"""
from __future__ import annotations

import numpy as np


class DomainError(ValueError):
    """A public input or state value violated a physical/structural domain (finiteness, sign, range, shape)."""


def ensure_finite(arr: np.ndarray, name: str) -> np.ndarray:
    """All elements finite (no NaN/Inf). Returns the array for chaining."""
    a = np.asarray(arr)
    if not np.all(np.isfinite(a)):
        n = int(np.sum(~np.isfinite(a)))
        raise DomainError(f"{name}: {n} non-finite (NaN/Inf) value(s) not allowed")
    return a


def ensure_nonneg(arr: np.ndarray, name: str) -> np.ndarray:
    """Finite and >= 0 everywhere (e.g. areal mass, ice fraction)."""
    a = ensure_finite(arr, name)
    if np.any(a < 0.0):
        raise DomainError(f"{name}: negative value(s) not allowed (min={float(a.min())!r})")
    return a


def ensure_positive(arr: np.ndarray, name: str) -> np.ndarray:
    """Finite and > 0 everywhere (e.g. density, so height = mass/density is defined)."""
    a = ensure_finite(arr, name)
    if np.any(a <= 0.0):
        raise DomainError(f"{name}: non-positive value(s) not allowed (min={float(a.min())!r})")
    return a


def ensure_range(arr: np.ndarray, lo: float, hi: float, name: str) -> np.ndarray:
    """Finite and within the closed interval [lo, hi] (e.g. disturbance in [0,1])."""
    a = ensure_finite(arr, name)
    if np.any(a < lo) or np.any(a > hi):
        raise DomainError(
            f"{name}: value(s) outside [{lo}, {hi}] (min={float(a.min())!r}, max={float(a.max())!r})")
    return a


def ensure_shape(arr: np.ndarray, shape: tuple, name: str) -> np.ndarray:
    """Exact array shape (e.g. every field is (height, width))."""
    a = np.asarray(arr)
    if a.shape != tuple(shape):
        raise DomainError(f"{name}: shape {a.shape} != expected {tuple(shape)}")
    return a


def ensure_kind(arr: np.ndarray, kinds: str, name: str) -> np.ndarray:
    """numpy dtype kind in `kinds` (e.g. 'fc' float/complex-free floats, 'iu' integer label).
    Lenient on width (float32 vs float64) so real loaded scenes pass; strict on category."""
    a = np.asarray(arr)
    if a.dtype.kind not in kinds:
        raise DomainError(f"{name}: dtype kind {a.dtype.kind!r} not in {kinds!r} (dtype={a.dtype})")
    return a


def ensure_finite_scalar(x: float, name: str) -> float:
    v = float(x)
    if not np.isfinite(v):
        raise DomainError(f"{name}: non-finite scalar {x!r}")
    return v


def ensure_positive_scalar(x: float, name: str) -> float:
    v = ensure_finite_scalar(x, name)
    if v <= 0.0:
        raise DomainError(f"{name}: must be > 0 (got {x!r})")
    return v


def ensure_nonneg_scalar(x: float, name: str) -> float:
    v = ensure_finite_scalar(x, name)
    if v < 0.0:
        raise DomainError(f"{name}: must be >= 0 (got {x!r})")
    return v
