"""TerrainTargetEnv — goal-conditioned construction RL env (M1 / F4).

A Gymnasium env configured by a `Challenge`. The agent commands a 3-D action
[v, omega, drum] (drive twist + drum engage: >0 cut, <0 dump); the conserved physics
authority mutates terrain (so mass conservation + slip are guarantees, not learned).
Observation = local height patch + target height patch + proprioception (yaw, slip,
sinkage, drum fill, progress). Reward is goal-conditioned and potential-based:

  flatten_pad / build_berm : R = (rmse_prev - rmse_new) * scale  (terrain matching),
  traverse                 : R = (goal_dist_prev - goal_dist_new) (cells),
  minus a small slip + time penalty, plus a success bonus.

Gymnasium-OPTIONAL (same pattern as RoverSimEnv): the core runs on bare numpy; it
subclasses gym.Env with spaces.Box when gymnasium is present.
"""
from __future__ import annotations

import dataclasses

import numpy as np

from . import challenge as chmod
from . import drive
from . import rover
from .column_state import StateLabel

try:
    import gymnasium as _gym
    from gymnasium import spaces as _spaces
    _HAS_GYM = True
    _BASE = _gym.Env
except Exception:                              # pragma: no cover
    _gym = None
    _spaces = None
    _HAS_GYM = False
    _BASE = object

HAS_GYM = _HAS_GYM


class TerrainTargetEnv(_BASE):
    metadata = {"render_modes": []}

    def __init__(self, challenge: chmod.Challenge, *, patch: int = 7, dt: float = 0.1,
                 v_max: float = 0.3, omega_max: float = 1.0,
                 cut_depth_m: float = 0.02, drum_half_cells: float = 5.0,
                 match_reward_scale: float = 20.0, goal_radius_cells: float = 2.0):
        super().__init__()
        self.challenge = challenge
        self.obj_type = challenge.objective.type
        self.region = challenge.objective.region
        self.tol = challenge.objective.tolerance_m
        self.grid = challenge.map.grid
        self.cell_m = challenge.map.cell_m
        self.max_steps = challenge.constraints.max_time_steps
        self.payload_limit = max(1e-6, challenge.constraints.payload_kg_limit)
        self.patch = int(patch) | 1
        self.dt = float(dt)
        self.v_max = float(v_max)
        self.omega_max = float(omega_max)
        self.cut_depth_m = float(cut_depth_m)
        self.drum_half = float(drum_half_cells)
        self.match_scale = float(match_reward_scale)
        self.goal_radius = float(goal_radius_cells)

        self.obs_dim = 2 * self.patch * self.patch + 6
        self.action_dim = 3

        # runtime
        self.inst = None
        self.cs = None
        self.rc = (self.grid / 2.0, 6.0)
        self.yaw = 0.0
        self._steps = 0
        self._last_slip = 0.0
        self._last_sinkage = 0.0
        self._progress = 0.0

        if _HAS_GYM:
            self.action_space = _spaces.Box(-1.0, 1.0, shape=(self.action_dim,), dtype=np.float32)
            hi = np.full(self.obs_dim, 1.0e3, dtype=np.float32)
            self.observation_space = _spaces.Box(-hi, hi, dtype=np.float32)

    # -- geometry helpers ----------------------------------------------------

    def _disc_mask(self, rc, half):
        r0, c0 = rc
        rr = np.arange(self.grid)[:, None] - r0
        cc = np.arange(self.grid)[None, :] - c0
        return (rr * rr + cc * cc) <= half * half

    def _patch(self, field, center_val):
        r0 = int(round(self.rc[0]))
        c0 = int(round(self.rc[1]))
        half = self.patch // 2
        rows = np.clip(np.arange(r0 - half, r0 + half + 1), 0, self.grid - 1)
        cols = np.clip(np.arange(c0 - half, c0 + half + 1), 0, self.grid - 1)
        return (field[np.ix_(rows, cols)] - center_val).ravel()

    def _region_rmse(self):
        return chmod.terrain_rmse(self.cs.derive_height(), self.inst.target_height, self.region)

    def _goal_dist(self):
        gr, gc = self.inst.goal_rc
        return float(np.hypot(self.rc[0] - gr, self.rc[1] - gc))

    def _progress_value(self):
        return self._goal_dist() if self.obj_type == "traverse" else self._region_rmse()

    def _obs(self):
        h = self.cs.derive_height()
        r0 = int(round(self.rc[0]))
        c0 = int(round(self.rc[1]))
        center = h[np.clip(r0, 0, self.grid - 1), np.clip(c0, 0, self.grid - 1)]
        local = self._patch(h, center)
        if self.inst.target_height is not None:
            target = self._patch(self.inst.target_height, center)
        else:
            target = np.zeros_like(local)        # traverse: no terrain target
        cf = rover.conform_pose(h, self.rc, self.yaw, cell_m=self.cell_m,
                                payload_kg=self.cs.drum_inventory)
        bucket = min(self.cs.drum_inventory / self.payload_limit, 1.0)
        prog = (self._goal_dist() / self.grid) if self.obj_type == "traverse" else self._region_rmse()
        scal = np.array([np.sin(self.yaw), np.cos(self.yaw), cf["pitch_rad"],
                         self._last_slip, bucket, prog], dtype=np.float64)
        return np.concatenate([local, target, scal]).astype(np.float32)

    # -- gym API -------------------------------------------------------------

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        if _HAS_GYM:
            super().reset(seed=seed)
        c = self.challenge
        if seed is not None:                     # per-episode map variation (held-out seeds)
            c = dataclasses.replace(c, map=dataclasses.replace(c.map, seed=seed))
        self.inst = chmod.realize(c)
        self.cs = self.inst.cs
        self.rc = (self.grid / 2.0, 6.0)
        self.yaw = 0.0
        self._steps = 0
        self._last_slip = 0.0
        self._last_sinkage = 0.0
        self._progress = self._progress_value()
        rmse = None if self.obj_type == "traverse" else self._region_rmse()
        return self._obs(), {"rmse": rmse, "goal_dist": (self._goal_dist() if self.obj_type == "traverse" else None)}

    def step(self, action):
        a = np.asarray(action, dtype=np.float64).reshape(-1)
        v = float(np.clip(a[0], -1.0, 1.0)) * self.v_max
        omega = float(np.clip(a[1], -1.0, 1.0)) * self.omega_max
        drum = float(np.clip(a[2], -1.0, 1.0))

        # 1) drive (slip-aware; laden weight = drum payload), authority carves ruts
        self.rc, self.yaw, telem = drive.drive_step(
            self.cs, self.rc, self.yaw, v, omega, dt=self.dt, params=self.inst.params,
            payload_kg=self.cs.drum_inventory)
        self._last_slip = telem["slip"]
        self._last_sinkage = telem["sinkage_m"]

        # 2) drum: cut (>+1/3) or dump (<-1/3) under the rover, mass-conserving
        if drum > 1.0 / 3.0:
            mask = self._disc_mask(self.rc, self.drum_half)
            if mask.any():
                self.cs.cut_to_inventory(mask, self.cut_depth_m * self.cs.density)
                self.cs.state_label[mask] = StateLabel.EXCAVATED
        elif drum < -1.0 / 3.0 and self.cs.drum_inventory > 0.0:
            mask = self._disc_mask(self.rc, self.drum_half)
            if mask.any():
                self.cs.dump_from_inventory(mask, self.cs.drum_inventory)

        # 3) reward (potential-based) + termination
        self._steps += 1
        new_prog = self._progress_value()
        if self.obj_type == "traverse":
            reward = (self._progress - new_prog) - 0.05 * telem["slip"] - 0.001
            success = new_prog <= self.goal_radius
        else:
            reward = (self._progress - new_prog) * self.match_scale - 0.05 * telem["slip"] - 0.001
            success = new_prog <= self.tol
        self._progress = new_prog
        terminated = bool(success)
        if success:
            reward += 1.0
        truncated = self._steps >= self.max_steps
        info = {"success": bool(success), "rmse": (None if self.obj_type == "traverse" else new_prog),
                "goal_dist": (new_prog if self.obj_type == "traverse" else None),
                "slip": telem["slip"], "drum_inventory": self.cs.drum_inventory}
        return self._obs(), float(reward), terminated, truncated, info
