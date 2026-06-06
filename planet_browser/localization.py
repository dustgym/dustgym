"""Map-relative localization — register an observed local elevation patch onto the PRIOR DEM (the SLAM
"overlay" / Gao Xiang *SLAM in Autonomous Driving* Ch10 fusion-positioning). docs/slam_pipeline_analysis.md.

dustgym already HAS the prior map (the LOLA DEM), so the rover localizes by matching what it senses to the
stored map -- not by building a map from scratch. This is the heightfield analog of ICP/NDT scan-to-map
registration: an integer-cell shift search that minimizes the MEAN-REMOVED SSD between the observed patch and
the DEM (mean-removed so an absolute datum/height offset doesn't bias it -- only terrain SHAPE matters).

Pure numpy; scored against the conserved-truth DEM (no gate). The corrected pose feeds the autonomy ESKF as
an independent map fix (P15 step 2). Sub-cell refinement + yaw recovery are the follow-on. The LIVE sensed
patch (Godot render -> stereo/COLMAP) is render/CUDA-gated; the registration math here is testable now by
sensing a patch from the truth and recovering a perturbed pose.
"""
from __future__ import annotations

import numpy as np


def patch_at(Z: np.ndarray, rc, half: int) -> np.ndarray:
    """Extract a (2*half+1)^2 elevation patch centred at cell rc, edge-clamped. Always returns the full
    shape (clamped at the map border)."""
    r0, c0 = int(round(rc[0])), int(round(rc[1]))
    H, W = Z.shape
    rows = np.clip(np.arange(r0 - half, r0 + half + 1), 0, H - 1)
    cols = np.clip(np.arange(c0 - half, c0 + half + 1), 0, W - 1)
    return Z[np.ix_(rows, cols)]


def register_to_dem(observed: np.ndarray, dem, guess_rc, *, search_radius_cells: int = 5) -> dict:
    """Register an observed elevation patch onto the prior DEM near a (drifted) guess pose.

    Searches integer cell shifts in [-R, R]^2 around `guess_rc`, scoring each by the mean-removed SSD
    between the DEM patch there and `observed`; the minimizing shift is the pose CORRECTION. Returns:
      corrected_rc   : guess_rc + best shift (the map-relative pose estimate)
      shift_cells    : the (dr, dc) correction
      residual_rmse_m: the matched height-shape RMSE (~0 for a clean match)
      confidence     : 1 - best_ssd/median_ssd, clamped [0,1] (sharply peaked = high; flat/ambiguous = ~0)
    """
    Z, _cell = dem
    half = observed.shape[0] // 2
    obs0 = observed - float(observed.mean())
    best_ssd, best = np.inf, (0, 0)
    ssds = []
    for dr in range(-search_radius_cells, search_radius_cells + 1):
        for dc in range(-search_radius_cells, search_radius_cells + 1):
            p = patch_at(Z, (guess_rc[0] + dr, guess_rc[1] + dc), half)
            if p.shape != observed.shape:
                continue
            res = (p - float(p.mean())) - obs0
            ssd = float(np.mean(res * res))
            ssds.append(ssd)
            if ssd < best_ssd:
                best_ssd, best = ssd, (dr, dc)
    med = float(np.median(ssds)) if ssds else 0.0
    confidence = (1.0 - best_ssd / med) if med > 1e-12 else 0.0
    return {
        "corrected_rc": (guess_rc[0] + best[0], guess_rc[1] + best[1]),
        "shift_cells": best,
        "residual_rmse_m": float(np.sqrt(max(0.0, best_ssd))),
        "confidence": float(max(0.0, min(1.0, confidence))),
    }
