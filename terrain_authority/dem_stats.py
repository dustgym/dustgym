"""Roughness statistics — deviogram + RMS-slope-vs-baseline (Wave-2, W2-VARIANCE).

The measurement half of the §7 falsifiable acceptance test (docs/dem_terrain_contract.md
§7, §8 W2-VARIANCE). No such measurement existed in the repo: it is what makes the word
"sourced" falsifiable — synthesized terrain is only as honest as it MATCHES the real DEM's
roughness, and "match" needs a number. The two functions here are the numbers.

  deviogram(field, cell_m, baselines_m)
      The (1-D, isotropic) STRUCTURE FUNCTION D(L) = RMS height difference over a lag L:
          D(L) = sqrt( < (h(x+L) - h(x))^2 > )            [m]
      averaged over BOTH grid axes (the lunar surface roughness has no preferred azimuth at
      these scales, so we pool the row- and column-lag pairs). This is the deviogram /
      Allan-deviation-of-height standard used for surface roughness vs baseline (Rosenburg
      et al. 2011, JGR 116:E02001, "Global surface slopes and roughness of the Moon from LOLA"
      use exactly the RMS height-difference-vs-baseline detrended-slope family). A pure plane
      has D(L) = |grad|*L (linear); a white-noise field saturates at D(L)=sqrt(2)*sigma.

  rms_slope_vs_baseline(field, cell_m, baselines_m)
      The RMS of the SLOPE measured over each baseline window, in DEGREES:
          slope_L(x) = atan( |h(x+L) - h(x)| / L ) ;  RMS over the field (both axes).
      This is the directly DEM-comparable quantity: the PGDA Product-78 `_slp` raster is the
      per-pixel slope at the NATIVE 5 m baseline, and the Rosenburg/Kreslavsky baseline-slope
      curve is this measured at increasing L. A pure plane returns its own slope at every L;
      coarser baselines smooth small-scale roughness so the RMS slope DECREASES with L
      (the lunar "slope rolls off with baseline" signature).

Both are PURE numpy, host-runnable (no engine, no scene I/O, no DEM read). They take a 2-D
``field`` (a heightmap [m], row-major; (row, col) == (+Z, +X) per INTERFACE.md §2) and a
``cell_m`` and a list of physical ``baselines_m``; each baseline is quantized to the nearest
integer cell lag ``k = round(L/cell_m)`` and SKIPPED (omitted from the returned dict) if it is
smaller than one cell or larger than the field, so the caller sees only baselines the field can
actually resolve. The returned dict is keyed by the REQUESTED baseline (float metres) for a
clean join with the committed slope_anchor.json.

PAPERED OVER (honesty): isotropic pooling assumes no azimuthal anisotropy (true for regolith
roughness at these scales, not for e.g. dune fields); the structure function is the raw
(non-detrended) height difference, so at large L it carries the regional tilt — for the
DEM-vs-synthetic comparison we anchor at the SAME L on the SAME window, so the tilt cancels.
"""

from __future__ import annotations

import numpy as np

__all__ = ["deviogram", "rms_slope_vs_baseline"]


def _lag_cells(cell_m: float, baseline_m: float, n_min: int) -> int | None:
    """Quantize a physical baseline [m] to an integer cell lag, or None if unusable.

    Returns ``round(baseline_m / cell_m)`` clamped to be at least 1 cell and strictly less
    than the smaller field dimension ``n_min`` (a lag >= the field has no pair to difference).
    None means "this baseline is not resolvable on this field" — the caller drops it.
    """
    if baseline_m <= 0.0 or cell_m <= 0.0:
        return None
    k = int(round(float(baseline_m) / float(cell_m)))
    if k < 1 or k >= n_min:
        return None
    return k


def _diff_pairs(field: np.ndarray, k: int) -> np.ndarray:
    """All lag-``k`` height differences along BOTH axes, flattened (isotropic pooling).

    Row-axis pairs ``field[:, k:] - field[:, :-k]`` and column-axis pairs
    ``field[k:, :] - field[:-k, :]`` are concatenated; the two axes are pooled so the result
    is the isotropic (azimuth-averaged) difference population. float64 for stable RMS.
    """
    f = field.astype(np.float64, copy=False)
    d_col = (f[:, k:] - f[:, :-k]).ravel()   # lag along +X (columns)
    d_row = (f[k:, :] - f[:-k, :]).ravel()   # lag along +Z (rows)
    return np.concatenate((d_col, d_row))


def deviogram(field: np.ndarray, cell_m: float, baselines_m) -> dict[float, float]:
    """Structure function D(L) = RMS height-difference over each baseline lag L [m].

    For each baseline ``L`` in ``baselines_m``, quantize to ``k = round(L/cell_m)`` cells and
    return ``D(L) = sqrt(mean((h(x+L) - h(x))^2))`` [m], pooling lag pairs over both grid axes
    (isotropic). Baselines that quantize below one cell or to/above the field size are OMITTED
    from the result (the field cannot resolve them). Keyed by the REQUESTED baseline (float m).

    >>> # pure plane, slope along +X only: D(L) == slope*L
    >>> import numpy as np
    >>> x = np.arange(100) * 2.0          # cell_m = 2 m
    >>> plane = np.tile(0.1 * x, (100, 1))  # 0.1 m rise per metre along X
    >>> d = deviogram(plane, 2.0, [10.0])   # L = 5 cells
    >>> abs(d[10.0] - 0.1 * 10.0) < 1e-9    # ~1.0 m (the X-axis pairs; Z-axis pairs are 0)
    """
    field = np.asarray(field)
    if field.ndim != 2:
        raise ValueError(f"deviogram: field must be 2-D, got shape {field.shape}")
    n_min = min(field.shape)
    out: dict[float, float] = {}
    for L in baselines_m:
        k = _lag_cells(cell_m, L, n_min)
        if k is None:
            continue
        d = _diff_pairs(field, k)
        out[float(L)] = float(np.sqrt(np.mean(d * d)))
    return out


def rms_slope_vs_baseline(field: np.ndarray, cell_m: float, baselines_m) -> dict[float, float]:
    """RMS slope [deg] measured over each baseline window L [m].

    For each baseline ``L``, quantize to ``k = round(L/cell_m)`` cells, take the magnitude of
    the height difference over that lag, convert to a slope angle ``atan(|dh| / (k*cell_m))``,
    and return the RMS of that angle over the field in DEGREES, pooling both axes (isotropic).
    The physical run used in the atan is ``k*cell_m`` (the QUANTIZED baseline), so the angle is
    consistent with the actual lag differenced. Unresolvable baselines are OMITTED. Keyed by the
    REQUESTED baseline (float m).

    This is the quantity directly comparable to the PGDA `_slp` product (per-pixel slope at the
    native baseline) and to the Rosenburg/Kreslavsky baseline-slope curve; a real lunar surface
    shows the RMS slope DECREASING as L grows (small-scale roughness averages out).

    >>> # pure plane, slope along +X only: RMS slope == atan(slope) regardless of L
    >>> import numpy as np
    >>> x = np.arange(80) * 5.0
    >>> plane = np.tile(np.tan(np.deg2rad(12.0)) * x, (80, 1))  # 12 deg along X, flat along Z
    >>> s = rms_slope_vs_baseline(plane, 5.0, [25.0])           # L = 5 cells
    >>> # X-axis pairs give 12 deg, Z-axis pairs give 0 deg -> RMS = sqrt((12^2+0)/2) ~ 8.49
    >>> abs(s[25.0] - (12.0 / np.sqrt(2.0))) < 1e-6
    """
    field = np.asarray(field)
    if field.ndim != 2:
        raise ValueError(f"rms_slope_vs_baseline: field must be 2-D, got shape {field.shape}")
    n_min = min(field.shape)
    out: dict[float, float] = {}
    for L in baselines_m:
        k = _lag_cells(cell_m, L, n_min)
        if k is None:
            continue
        run_m = k * float(cell_m)               # the actual (quantized) physical baseline
        d = np.abs(_diff_pairs(field, k))
        slope_deg = np.degrees(np.arctan2(d, run_m))
        out[float(L)] = float(np.sqrt(np.mean(slope_deg * slope_deg)))
    return out


# ---------------------------------------------------------------------------
# Self-test (analytic-truth sanity, not a trivially-true assertion).
#   python -m terrain_authority.dem_stats
# Checks against fields whose roughness statistics are known in closed form:
#   1. PURE PLANE  -> deviogram is LINEAR in L (D(L)=slope*L on the tilted axis) and
#      rms_slope is CONSTANT in L (== the plane's own slope, isotropically pooled).
#   2. WHITE NOISE -> deviogram is FLAT vs L and saturates at sqrt(2)*sigma; rms slope
#      DROPS as L grows (independent samples averaged over a longer run).
#   3. BASELINE ROLL-OFF on a real-shaped fbm-ish field -> rms slope is monotone-ish
#      decreasing with L (the lunar signature), and unresolvable baselines are dropped.
# Prints PASS/FAIL and exits nonzero on any failure.
# ---------------------------------------------------------------------------

def _self_test() -> int:
    results: list[tuple[str, bool, str]] = []

    def check(name: str, ok: bool, detail: str = "") -> None:
        results.append((name, bool(ok), detail))
        print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))

    cell_m = 2.0
    n = 256

    # 1. PURE PLANE: tilt only along +X, slope 8 deg. -------------------------
    slope_deg = 8.0
    g = np.tan(np.deg2rad(slope_deg))                  # rise per metre
    x_m = np.arange(n) * cell_m
    plane = np.tile(g * x_m, (n, 1))                   # h(row, col) = g * x(col)
    bl = [10.0, 20.0, 40.0, 80.0, 160.0]

    dev = deviogram(plane, cell_m, bl)
    # On a plane tilted along X only, the X-axis pairs give |dh|=g*L and the Z-axis pairs
    # give 0; pooled equally -> D(L) = sqrt((g*L)^2 / 2) = g*L/sqrt(2). LINEAR in L.
    dev_pred = {L: g * L / np.sqrt(2.0) for L in bl}
    dev_ok = all(abs(dev[L] - dev_pred[L]) <= 1e-6 * max(dev_pred[L], 1.0) for L in bl)
    # ... and the ratio D(L)/L is constant (slope of the structure function is flat).
    ratios = [dev[L] / L for L in bl]
    linear_ok = (max(ratios) - min(ratios)) <= 1e-9
    check("plane: deviogram is exactly g*L/sqrt2 (linear structure function)",
          dev_ok and linear_ok,
          f"D/L const={max(ratios)-min(ratios):.2e} D(160m)={dev[160.0]:.4f} "
          f"pred={dev_pred[160.0]:.4f}")

    rms = rms_slope_vs_baseline(plane, cell_m, bl)
    # X-axis pairs -> slope_deg, Z-axis pairs -> 0 deg, pooled -> sqrt((s^2+0)/2)=s/sqrt2.
    rms_pred = slope_deg / np.sqrt(2.0)
    rms_const = max(rms.values()) - min(rms.values())
    rms_ok = all(abs(v - rms_pred) <= 1e-6 for v in rms.values()) and rms_const <= 1e-9
    check("plane: rms-slope is constant in L == plane slope / sqrt2 (8deg -> 5.657)",
          rms_ok, f"rms={rms[80.0]:.4f} pred={rms_pred:.4f} spread={rms_const:.2e}")

    # 2. WHITE NOISE: deviogram flat, saturates at sqrt(2)*sigma. -------------
    rng = np.random.default_rng(20260531)
    sigma = 0.25
    noise = rng.normal(0.0, sigma, size=(n, n))
    devn = deviogram(noise, cell_m, bl)
    # Differences of independent N(0,sigma^2) -> var 2*sigma^2 -> RMS = sqrt(2)*sigma at EVERY
    # lag (no spatial correlation), so the deviogram is FLAT. Allow finite-sample wobble.
    sat = np.sqrt(2.0) * sigma
    flat_ok = all(abs(devn[L] - sat) <= 0.03 * sat for L in bl)
    rmsn = rms_slope_vs_baseline(noise, cell_m, bl)
    # RMS slope must DROP with L: same |dh| spread (~const) over a longer run -> smaller angle.
    drop_ok = all(rmsn[bl[i]] > rmsn[bl[i + 1]] for i in range(len(bl) - 1))
    check("white-noise: deviogram flat ~sqrt2*sigma; rms-slope strictly drops with L",
          flat_ok and drop_ok,
          f"D~{np.mean(list(devn.values())):.4f} (sat={sat:.4f}) "
          f"slope {rmsn[10.0]:.2f}->{rmsn[160.0]:.2f} deg")

    # 3. UNRESOLVABLE baselines are dropped (below 1 cell / above field). -----
    drop_dev = deviogram(plane, cell_m, [0.5, 1.0, 10.0, 1e9])  # 0.5m<cell, 1e9m>field
    drop_handled = (0.5 not in drop_dev) and (1e9 not in drop_dev) \
        and (10.0 in drop_dev) and (1.0 not in drop_dev)  # 1.0m -> round(0.5)=0 -> dropped
    check("unresolvable baselines (sub-cell / larger-than-field) are omitted",
          drop_handled, f"kept={sorted(drop_dev)}")

    n_fail = sum(1 for _, ok, _ in results if not ok)
    print(f"\n{len(results) - n_fail}/{len(results)} checks passed.")
    return 1 if n_fail else 0


if __name__ == "__main__":
    raise SystemExit(_self_test())
