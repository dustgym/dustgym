"""map_channel.py -- the LAC section 10 map-channel reward, closed into the planning/autonomy loop.

The Lunar Autonomy Challenge objective rewards building an ACCURATE ELEVATION MAP of the worksite, not
just reaching waypoints. There are two perception tiers (docs/world_model.md, validation/map_channel/):

  * the DENSE reconstruction tier (Godot render -> onboard-stereo / ground COLMAP MVS) gives a real
    observed heightfield with a measurable RMSE vs the conserved truth. It is host / CUDA / container
    gated, so it is NOT in this in-loop path (see scripts/ros2_bridge + scripts/colmap).
  * the cheap ONBOARD-OBSERVABILITY tier, here: computed from the conserved truth, it scores what the
    PLANNED ROUTE actually SEES -- the worksite COVERAGE and the residual per-cell map uncertainty as the
    rover visits stations. This is a real, sub-millisecond, in-loop reward that gates dig commitment.

Honesty: this scores OBSERVABILITY (which worksite cells the route brings within sensor range), not a
reconstructed heightfield. Observed cells carry the published onboard-stereo height sigma [CALIB ~0.32 m,
validation/map_channel]; unobserved cells carry a high prior. No fabricated heights -- the dense RMSE is
the gated tier above. "Closing the reward into the loop" = the route's coverage/uncertainty is reported
AND a low-coverage dig site triggers observe-more before the rover commits to digging there.
"""
from __future__ import annotations

import math

import numpy as np

ONBOARD_STEREO_SIGMA_M = 0.32   # [CALIB] onboard rover-stereo height RMSE (the cheap real-time tier)
PRIOR_SIGMA_M = 5.0             # an unobserved cell's prior height uncertainty (no map there yet)
SENSOR_RADIUS_M = 8.0          # [CALIB] usable onboard-stereo mapping radius per station
COVERAGE_DIG_GATE = 0.6        # observe-more before digging a site whose local coverage is below this


def worksite_bbox(mission, *, margin_m=10.0):
    """The worksite we score map coverage over: the bounding box of all order footprints + a margin."""
    xs = [o.x for o in mission.orders]
    ys = [o.y for o in mission.orders]
    if not xs:
        return (0.0, 0.0, 1.0, 1.0)
    return (min(xs) - margin_m, min(ys) - margin_m, max(xs) + margin_m, max(ys) + margin_m)


def _grid(bbox, cell_m):
    x0, y0, x1, y1 = bbox
    W = max(1, int(math.ceil((x1 - x0) / cell_m)))
    H = max(1, int(math.ceil((y1 - y0) / cell_m)))
    cx = x0 + (np.arange(W) + 0.5) * cell_m
    cy = y0 + (np.arange(H) + 0.5) * cell_m
    return np.meshgrid(cx, cy)   # XX, YY each (H, W)


def coverage_mask(bbox, cell_m, stations, sensor_radius_m):
    """Boolean (H, W): a cell is observed if it lies within sensor_radius of ANY visited station."""
    XX, YY = _grid(bbox, cell_m)
    obs = np.zeros(XX.shape, dtype=bool)
    for sx, sy in stations:
        obs |= ((XX - sx) ** 2 + (YY - sy) ** 2) <= sensor_radius_m ** 2
    return obs


def map_channel_score(mission, stations, *, cell_m=1.0, sensor_radius_m=SENSOR_RADIUS_M,
                      onboard_sigma_m=ONBOARD_STEREO_SIGMA_M, prior_sigma_m=PRIOR_SIGMA_M):
    """The in-loop map-channel reward for a route that visited `stations` (list of (x,y) in the order frame).
    Returns coverage (observed fraction of the worksite), mean residual map uncertainty, and the counts."""
    bbox = worksite_bbox(mission)
    obs = coverage_mask(bbox, cell_m, stations, sensor_radius_m)
    sigma = np.where(obs, onboard_sigma_m, prior_sigma_m)
    return {
        "coverage": float(obs.mean()),
        "observed_cells": int(obs.sum()), "total_cells": int(obs.size),
        "mean_uncertainty_m": float(sigma.mean()), "observed_uncertainty_m": float(onboard_sigma_m),
        "sensor_radius_m": float(sensor_radius_m), "cell_m": float(cell_m), "n_stations": len(stations),
        "dense_rmse_available": False,   # the dense COLMAP/render reconstruction RMSE is the gated tier
    }


def local_coverage(stations, site, *, radius_m=SENSOR_RADIUS_M, sensor_radius_m=SENSOR_RADIUS_M, cell_m=1.0):
    """Fraction of a disk of `radius_m` around `site` already observed by `stations` -- the signal the
    autonomy loop gates a dig on (don't commit to digging terrain you haven't mapped well enough yet)."""
    sx, sy = site
    bbox = (sx - radius_m, sy - radius_m, sx + radius_m, sy + radius_m)
    XX, YY = _grid(bbox, cell_m)
    in_disk = ((XX - sx) ** 2 + (YY - sy) ** 2) <= radius_m ** 2
    if not in_disk.any():
        return 1.0
    obs = coverage_mask(bbox, cell_m, stations, sensor_radius_m)
    return float(obs[in_disk].mean())
