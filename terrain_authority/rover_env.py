"""RoverSimEnv — a Tier-2 reinforcement-learning environment (Phase 4, 2026-06-02).

Wraps the headless, deterministic, mass-conserving authority as a Gymnasium env so an
RL agent (or any controller) can learn to drive the rover. The per-step core is
``drive.drive_step`` (slope -> slip-sinkage -> achieved motion -> slip-deepened ruts),
so the env inherits the closed-loop slip physics from Phase 3.

GYMNASIUM IS OPTIONAL. The full RL logic (reset / step / observation / reward / done /
domain randomization) is pure NumPy and runs without gymnasium installed; when gymnasium
IS present, RoverSimEnv subclasses ``gymnasium.Env`` and exposes ``spaces.Box`` action /
observation spaces (and passes ``gymnasium.utils.env_checker``). This keeps the env
testable on the bare interpreter while remaining a first-class gym.Env where gym exists.

Task: drive the rover to a target column (``goal_col``) across terrain. Continuous action
= a normalized twist [v, omega] in [-1, 1]^2 (scaled to v_max / omega_max). Observation =
a local relative-height patch + proprioception (yaw, pitch, roll, last slip, last sinkage,
distance-to-goal). HONEST CONTROL REWARD from true state: progress toward goal minus a slip
penalty and a small time cost, with an entrapment terminal penalty and a goal bonus. The
reward reads GROUND-TRUTH state (this is a control env, not perception-in-the-loop; do not
confuse it with the unbuilt §10 map-channel perception reward).

Domain randomization (reset with randomize=True): samples slope + soil params from the
sourced [CALIB]/[UNKNOWN] envelopes (terramechanics.domain_randomize) — the tags are the
randomization spec (spec §7.5).
"""
from __future__ import annotations

import numpy as np

from . import constants as K
from . import drive
from . import rover
from . import terramechanics as tm
from .column_state import ColumnState

try:                                          # gymnasium is OPTIONAL
    import gymnasium as _gym
    from gymnasium import spaces as _spaces
    _HAS_GYM = True
    _BASE = _gym.Env
except Exception:                             # pragma: no cover - exercised only w/o gym
    _gym = None
    _spaces = None
    _HAS_GYM = False
    _BASE = object

HAS_GYM = _HAS_GYM


class RoverSimEnv(_BASE):
    """Gymnasium-style RL environment over the Tier-2 closed-loop authority.

    Args:
        grid, cell_m: scene dimensions. slope_deg: fixed ramp grade (ignored if randomize).
        start_col / goal_col: drive from start_col toward goal_col (+col). goal_radius_cells.
        v_max / omega_max: action scaling. dt: step time. max_steps: truncation horizon.
        payload_kg: drum payload (raises weight-on-wheels). params: TerramechanicsParams
        (None -> constants). randomize: sample slope+params from sourced envelopes each reset.
        patch: side length of the local height-patch observation (odd).
    """

    metadata = {"render_modes": []}

    def __init__(self, *, grid: int = 96, cell_m: float = 0.02, slope_deg: float = 0.0,
                 start_col: float = 16.0, goal_col: float = 80.0, goal_radius_cells: float = 2.0,
                 v_max: float = 0.3, omega_max: float = 1.0, dt: float = 0.1,
                 max_steps: int = 200, payload_kg: float = 0.0,
                 params: "tm.TerramechanicsParams | None" = None, body=None,
                 randomize: bool = False, slope_max_deg: float = 40.0, patch: int = 5):
        super().__init__()
        self.grid = int(grid)
        self.cell_m = float(cell_m)
        self.slope_deg = float(slope_deg)
        self.start_col = float(start_col)
        self.goal_col = float(goal_col)
        self.goal_radius = float(goal_radius_cells)
        self.v_max = float(v_max)
        self.omega_max = float(omega_max)
        self.dt = float(dt)
        self.max_steps = int(max_steps)
        self.payload_kg = float(payload_kg)
        # per-body physics: body sets gravity (weight = m*g) + Lyasko-corrected regolith (bodies.py).
        if body is not None:
            from . import bodies as _bodies
            _b = _bodies.get_body(body)
            self.body = _b.name
            self.g = _b.g
            self.params_base = params if params is not None else _bodies.params_for_body(_b)
            if _b.bekker_regime == "microgravity":         # honest: Bekker model is out of regime
                import warnings
                warnings.warn(
                    f"RoverSimEnv body={_b.name!r}: gravity is {_b.g:.1e} m/s^2 -- the gravity-loaded "
                    "Bekker pressure-sinkage model is OUT OF REGIME (microgravity, cohesion/granular "
                    "dynamics dominate). Results are a placeholder, not validated physics; use a "
                    "DEM/granular model. See terrain_authority.bodies / docs/bodies_sysrev.md.",
                    stacklevel=2)
        else:
            self.body = None
            self.g = K.g
            self.params_base = params or tm.TerramechanicsParams.from_constants()
        self.randomize = bool(randomize)
        self.slope_max_deg = float(slope_max_deg)
        self.patch = int(patch) | 1            # force odd
        self.obs_dim = self.patch * self.patch + 7
        self.action_dim = 2

        # runtime state (set in reset)
        self.params = self.params_base
        self.cs: ColumnState | None = None
        self.rc = (grid / 2.0, start_col)
        self.yaw = 0.0
        self._rng = np.random.default_rng(0)
        self._steps = 0
        self._last_slip = 0.0
        self._last_sinkage = 0.0

        if _HAS_GYM:
            self.action_space = _spaces.Box(-1.0, 1.0, shape=(self.action_dim,), dtype=np.float32)
            # Finite (generous) obs bounds: relative heights are ~metres, the proprioceptive
            # scalars are O(1) (sin/cos, pitch/roll, slip[0,1], sinkage, dist[0,1]). 1e3 cannot
            # clip any real observation and keeps env_checker / SB3 happy (no +/-inf bounds).
            hi = np.full(self.obs_dim, 1.0e3, dtype=np.float32)
            self.observation_space = _spaces.Box(-hi, hi, dtype=np.float32)

    # -- scene + geometry ----------------------------------------------------

    def _build_scene(self, slope_deg: float) -> ColumnState:
        cs = ColumnState(width=self.grid, height=self.grid, cell_m=self.cell_m)
        if slope_deg:
            cols = np.arange(self.grid)[None, :].repeat(self.grid, axis=0).astype(np.float64)
            cs.datum = np.tan(np.radians(slope_deg)) * cols * self.cell_m   # ramp along +col
        return cs

    def _goal_dist_cells(self) -> float:
        return max(0.0, self.goal_col - self.rc[1])

    def _obs(self) -> np.ndarray:
        h = self.cs.derive_height()
        r0 = int(round(self.rc[0]))
        c0 = int(round(self.rc[1]))
        half = self.patch // 2
        rows = np.clip(np.arange(r0 - half, r0 + half + 1), 0, self.grid - 1)
        cols = np.clip(np.arange(c0 - half, c0 + half + 1), 0, self.grid - 1)
        patch = h[np.ix_(rows, cols)]
        center = h[np.clip(r0, 0, self.grid - 1), np.clip(c0, 0, self.grid - 1)]
        rel = (patch - center).ravel()
        cf = rover.conform_pose(h, self.rc, self.yaw, cell_m=self.cell_m,
                                payload_kg=self.payload_kg)
        scal = np.array([
            np.sin(self.yaw), np.cos(self.yaw),
            cf["pitch_rad"], cf["roll_rad"],
            self._last_slip, self._last_sinkage,
            self._goal_dist_cells() / self.grid,
        ], dtype=np.float64)
        return np.concatenate([rel, scal]).astype(np.float32)

    # -- gym API -------------------------------------------------------------

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        if _HAS_GYM:
            super().reset(seed=seed)
        self._rng = np.random.default_rng(seed)
        slope_deg = self.slope_deg
        self.params = self.params_base
        if self.randomize:
            slope_deg = float(self._rng.uniform(0.0, self.slope_max_deg))
            self.params = tm.domain_randomize(self._rng, base=self.params_base)
        self.cs = self._build_scene(slope_deg)
        self.rc = (self.grid / 2.0, self.start_col)
        self.yaw = 0.0                         # facing +col, toward the goal
        self._steps = 0
        self._last_slip = 0.0
        self._last_sinkage = 0.0
        return self._obs(), {"slope_deg": slope_deg}

    def step(self, action):
        a = np.asarray(action, dtype=np.float64).reshape(-1)
        v = float(np.clip(a[0], -1.0, 1.0)) * self.v_max
        omega = float(np.clip(a[1], -1.0, 1.0)) * self.omega_max

        dist_prev = self._goal_dist_cells()
        self.rc, self.yaw, telem = drive.drive_step(
            self.cs, self.rc, self.yaw, v, omega, dt=self.dt, params=self.params,
            payload_kg=self.payload_kg, g=self.g)
        self._last_slip = telem["slip"]
        self._last_sinkage = telem["sinkage_m"]
        self._steps += 1

        dist_new = self._goal_dist_cells()
        progress = dist_prev - dist_new                       # cells advanced toward goal
        reward = progress - 0.1 * telem["slip"] - 0.001       # progress - slip penalty - time
        terminated = False
        if telem["entrapped"]:
            reward -= 1.0
            terminated = True
        elif dist_new <= self.goal_radius:
            reward += 1.0
            terminated = True
        truncated = self._steps >= self.max_steps
        info = {"telem": telem, "dist_cells": dist_new, "reached_goal": dist_new <= self.goal_radius}
        return self._obs(), float(reward), terminated, truncated, info
