"""A small slope-aware route planner over a DEM crop.

Dijkstra on the 8-connected grid with edge cost = metric length x (1 + penalty x mean slope), and steep
cells (> max_slope_deg) blocked. This is the ground station's job in a teleoperated mission: hand the
rover a sequence of reachable waypoints across navigable terrain rather than a blind straight line.
Pure NumPy + stdlib heapq (runs in CI).
"""
from __future__ import annotations

import heapq
import math

import numpy as np


def slope_deg(heightmap: np.ndarray, cell_m: float) -> np.ndarray:
    """Per-cell terrain slope magnitude [deg] from the DEM gradient."""
    gy, gx = np.gradient(np.asarray(heightmap, dtype=np.float64), cell_m)
    return np.degrees(np.arctan(np.hypot(gx, gy)))


def snap_to_navigable(slope: np.ndarray, rc: tuple[int, int], max_slope_deg: float) -> tuple[int, int]:
    """Return the nearest cell to ``rc`` whose slope is <= max_slope_deg."""
    ok = slope <= max_slope_deg
    if ok[rc[0], rc[1]]:
        return rc
    ys, xs = np.where(ok)
    if len(ys) == 0:
        return rc
    d = (ys - rc[0]) ** 2 + (xs - rc[1]) ** 2
    i = int(d.argmin())
    return int(ys[i]), int(xs[i])


def plan_route(heightmap: np.ndarray, cell_m: float, start_rc: tuple[int, int], goal_rc: tuple[int, int],
               *, max_slope_deg: float = 18.0, penalty: float = 0.6,
               n_waypoints: int = 6) -> list[tuple[float, float]]:
    """Plan a slope-penalized route from start to goal; return ~n_waypoints (row, col) cells (incl. goal).

    Returns an empty list if no navigable path exists. Start/goal are snapped to navigable cells first.
    """
    slope = slope_deg(heightmap, cell_m)
    H, W = slope.shape
    start = snap_to_navigable(slope, start_rc, max_slope_deg)
    goal = snap_to_navigable(slope, goal_rc, max_slope_deg)
    blocked = slope > max_slope_deg

    def idx(r: int, c: int) -> int:
        return r * W + c

    dist = np.full(H * W, math.inf)
    prev = np.full(H * W, -1, dtype=np.int64)
    dist[idx(*start)] = 0.0
    pq: list[tuple[float, int]] = [(0.0, idx(*start))]
    neigh = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    goal_i = idx(*goal)
    while pq:
        d, u = heapq.heappop(pq)
        if u == goal_i:
            break
        if d > dist[u]:
            continue
        ur, uc = divmod(u, W)
        for dr, dc in neigh:
            vr, vc = ur + dr, uc + dc
            if not (0 <= vr < H and 0 <= vc < W) or blocked[vr, vc]:
                continue
            step_m = cell_m * math.hypot(dr, dc)
            cost = step_m * (1.0 + penalty * 0.5 * (slope[ur, uc] + slope[vr, vc]))
            nd = d + cost
            vi = idx(vr, vc)
            if nd < dist[vi]:
                dist[vi] = nd
                prev[vi] = u
                heapq.heappush(pq, (nd, vi))

    if not math.isfinite(dist[goal_i]):
        return []
    path: list[tuple[int, int]] = []
    cur = goal_i
    while cur != -1:
        path.append((cur // W, cur % W))
        cur = int(prev[cur])
    path.reverse()
    if len(path) <= n_waypoints:
        return [(float(r), float(c)) for r, c in path[1:]]
    stride = max(1, (len(path) - 1) // n_waypoints)
    wps = [path[i] for i in range(stride, len(path), stride)]
    if wps[-1] != path[-1]:
        wps.append(path[-1])
    return [(float(r), float(c)) for r, c in wps]
