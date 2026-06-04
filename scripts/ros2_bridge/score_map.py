"""Lane-C MAP channel: perceived-map-vs-truth scorer -- the §10 mapping metric (also the keystone
nav costmap / RL reward).  REPORT-ONLY (no CI gate, no pass/fail).

Computes the three `eval_schema.Scorecard` map-channel fields from an OBSERVED elevation map vs the
TRUE terrain at time t (the LAC-style objective; the terrain is time-varying because the rover
reshapes it, so truth is "truth AT t"):

    map_rmse_m         -- RMS height error over observed (valid) cells [m].
    map_cell_pass_frac -- fraction of valid cells within tol_m of truth (the LAC cell-pass metric).
    rock_f1            -- boulder-detection F1 (greedy nearest match within rock_match_m), or None.

These live on the SAME `Scorecard` as the pose channel but measure a DIFFERENT thing on DIFFERENT
data; per eval_schema they are reported side by side and NEVER summed/averaged with the pose metrics.

HONESTY -- this is the SCORER half of the map channel.  The live OBSERVED-MAP PRODUCER (stereo-depth
or SLAM egress -> reconstructed heightfield) needs the Godot/sensor render track, which is not present
in this repo, so the synthetic `eval_harness` leaves these metrics None until a producer exists.  This
module fabricates NO observed map: it scores an observed map you SUPPLY (a real producer's output, or a
real DEM pair via `eval_harness.run_map`).  Tests feed it a REAL lower-resolution reconstruction of the
real DEM (subsampled real data), never synthetic terrain.

CC0-1.0 (see ../../LICENSE).
"""

from __future__ import annotations

import dataclasses
import math
from typing import Optional, Sequence

import numpy as np

import eval_schema as es


def map_height_metrics(
    observed, truth, *, tol_m: float = 0.10, valid_mask=None
) -> tuple[Optional[float], Optional[float]]:
    """(map_rmse_m, map_cell_pass_frac) over the valid cells of an observed-vs-truth heightfield pair.

    `observed`/`truth` are 2-D height arrays (m), same shape.  `valid_mask` (bool, same shape) marks the
    cells the observation actually covers -- unobserved / occluded / shadowed cells are excluded so an
    incomplete map is not unfairly penalised on terrain it never saw.  Non-finite cells are dropped.
    Returns (None, None) when no valid cell remains (an empty observation is not scorable, not 0-error).
    """
    obs = np.asarray(observed, dtype=np.float64)
    tru = np.asarray(truth, dtype=np.float64)
    if obs.shape != tru.shape:
        raise ValueError(f"observed {obs.shape} and truth {tru.shape} must have the same shape")
    mask = np.ones(obs.shape, dtype=bool) if valid_mask is None else np.asarray(valid_mask, dtype=bool)
    mask = mask & np.isfinite(obs) & np.isfinite(tru)
    if not mask.any():
        return None, None
    err = obs[mask] - tru[mask]
    rmse = float(np.sqrt(np.mean(err * err)))
    pass_frac = float(np.mean(np.abs(err) <= tol_m))
    return rmse, pass_frac


def rock_f1(
    truth_rocks: Optional[Sequence[Sequence[float]]],
    observed_rocks: Optional[Sequence[Sequence[float]]],
    *,
    match_m: float = 1.0,
) -> Optional[float]:
    """Boulder-detection F1 by greedy nearest matching within `match_m` (positions are (x, y) in metres).

    Each observed rock claims the nearest still-unmatched truth rock within `match_m` (a true positive);
    leftover observations are false positives, leftover truths false negatives.  Returns None if either
    list is None (rock detection not evaluated); 1.0 when both are empty (nothing to find, nothing claimed).
    """
    if truth_rocks is None or observed_rocks is None:
        return None
    tr = [(float(p[0]), float(p[1])) for p in truth_rocks]
    ob = [(float(p[0]), float(p[1])) for p in observed_rocks]
    if not tr and not ob:
        return 1.0
    matched: set[int] = set()
    tp = 0
    for ox, oy in ob:
        best, best_d = None, match_m
        for i, (tx, ty) in enumerate(tr):
            if i in matched:
                continue
            d = math.hypot(ox - tx, oy - ty)
            if d <= best_d:
                best, best_d = i, d
        if best is not None:
            matched.add(best)
            tp += 1
    fp = len(ob) - tp
    fn = len(tr) - tp
    prec = tp / (tp + fp) if (tp + fp) else 0.0
    rec = tp / (tp + fn) if (tp + fn) else 0.0
    return float(2 * prec * rec / (prec + rec)) if (prec + rec) else 0.0


def score_map(
    observed, truth, *, tol_m: float = 0.10, valid_mask=None,
    truth_rocks=None, observed_rocks=None, rock_match_m: float = 1.0,
) -> dict:
    """The three map-channel metrics as a dict (the keys are the Scorecard field names)."""
    rmse, pass_frac = map_height_metrics(observed, truth, tol_m=tol_m, valid_mask=valid_mask)
    return {
        "map_rmse_m": rmse,
        "map_cell_pass_frac": pass_frac,
        "rock_f1": rock_f1(truth_rocks, observed_rocks, match_m=rock_match_m),
    }


def attach_map_metrics(scorecard: es.Scorecard, observed, truth, **kw) -> es.Scorecard:
    """Return a COPY of `scorecard` with the map-channel fields populated (pose channel left untouched)."""
    m = score_map(observed, truth, **kw)
    return dataclasses.replace(
        scorecard,
        map_rmse_m=m["map_rmse_m"],
        map_cell_pass_frac=m["map_cell_pass_frac"],
        rock_f1=m["rock_f1"],
    )
