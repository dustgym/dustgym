"""ActivePerceptionEnv: next-best-view mapping -- drive to reduce map uncertainty per joule.

The map channel / Uncertainty layer as the RL REWARD (the keystone the design docs name). The agent
explores a real conserved-authority fbm terrain; observing at a viewpoint fuses a RANGE-DEPENDENT stereo
observation into the per-cell belief (sigma shrinks toward the measured floor, growing with range like
Z^2 -- the height-sweep result), and the reward is the information gained per joule spent driving (the
ipex drive energy). This is the env a learned perception world model trains in: predict-then-act to map
efficiently. Grounded -- the perception noise + energy are measured, the terrain is the authority's own
generator; the gym layer is optional (bare-numpy core runs + is tested without gymnasium).
"""
from __future__ import annotations

import numpy as np

from . import ipex_specs as S
from . import procgen

try:                                          # gymnasium is OPTIONAL (bare-numpy core still runs/tests)
    import gymnasium as _gym
    from gymnasium import spaces as _spaces
    _HAS_GYM = True
    _BASE = _gym.Env
except Exception:                             # pragma: no cover - exercised only without gym
    _gym = _spaces = None
    _HAS_GYM = False
    _BASE = object

HAS_GYM = _HAS_GYM
OBS_SIGMA_FLOOR_M = 0.30                       # measured passive-stereo 1-sigma at the rover's grazing eye-height
PRIOR_SIGMA_M = 2.0                            # unobserved-cell prior height uncertainty
_MOVES = ((-1, 0), (1, 0), (0, 1), (0, -1))   # N, S, E, W


def _fuse_sigma(sigma, pos, rr, cc, sensor_r):
    """Range-dependent stereo fusion of one viewpoint: returns (new_sigma, kalman_gain). Pure, so the
    env mutates its state with it AND the beam planner simulates forward on copies with it."""
    dist = np.hypot(rr - pos[0], cc - pos[1])
    obs_sig = OBS_SIGMA_FLOOR_M * (1.0 + (dist / sensor_r) ** 2)   # stereo Z^2 range falloff
    var = sigma ** 2
    k = np.where(dist <= sensor_r, var / (var + obs_sig ** 2), 0.0)
    return np.sqrt(np.maximum(0.0, (1.0 - k) * var)), k


class ActivePerceptionEnv(_BASE):
    """Discrete-move next-best-view mapping. action in {0..3} = move N/S/E/W one cell + observe.
    obs = [flattened sigma field, pos_row/grid, pos_col/grid, energy_frac]. reward = info gain / joule."""

    metadata = {"render_modes": []}

    def __init__(self, *, grid: int = 24, cell_m: float = 2.0, sensor_radius_cells: float = 4.0,
                 relief_m: float = 3.0, charges: float = 0.25, seed: int = 0):
        super().__init__()
        self.grid = int(grid)
        self.cell_m = float(cell_m)
        self.sensor_r = float(sensor_radius_cells)
        self.relief_m = float(relief_m)
        self.drive_j_per_m = S.drive_power_w() / S.DRIVE_SPEED_MS
        self.energy_budget = float(charges) * S.battery_energy_j()
        self._seed0 = int(seed)
        self._rr, self._cc = np.mgrid[0:self.grid, 0:self.grid]
        if _HAS_GYM:
            self.action_space = _spaces.Discrete(4)
            n = self.grid * self.grid + 3
            self.observation_space = _spaces.Box(low=0.0, high=PRIOR_SIGMA_M, shape=(n,), dtype=np.float32)
        self.reset(seed=seed)

    def _gen_terrain(self, seed: int) -> np.ndarray:
        z = procgen.fbm(self.grid, self.grid, seed=int(seed))     # authority fbm, minmax-normalized [0,1]
        return (z - float(z.mean())) * self.relief_m

    def _observe(self) -> float:
        """Fuse a range-dependent stereo observation at the current pose; return summed sigma reduction."""
        before = float(self.sigma.sum())
        new_sigma, k = _fuse_sigma(self.sigma, self.pos, self._rr, self._cc, self.sensor_r)
        self.est = self.est + k * (self.truth - self.est)         # estimate -> truth (measured w/ noise)
        self.sigma = new_sigma
        return before - float(self.sigma.sum())

    def _obs(self) -> np.ndarray:
        pos = np.array([self.pos[0] / self.grid, self.pos[1] / self.grid,
                        max(0.0, self.energy) / self.energy_budget], dtype=np.float32)
        return np.concatenate([self.sigma.ravel().astype(np.float32), pos])

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        if _HAS_GYM:
            super().reset(seed=seed)
        s = self._seed0 if seed is None else int(seed)
        self.truth = self._gen_terrain(s)
        self.sigma = np.full((self.grid, self.grid), PRIOR_SIGMA_M)
        self.est = np.zeros((self.grid, self.grid))
        self.pos = (self.grid // 2, self.grid // 2)
        self.energy = self.energy_budget
        self._observe()                                            # an initial fix from the start pose
        obs = self._obs()
        return (obs, {}) if _HAS_GYM else obs

    def step(self, action):
        dr, dc = _MOVES[int(action) % 4]
        nr = min(max(self.pos[0] + dr, 0), self.grid - 1)
        nc = min(max(self.pos[1] + dc, 0), self.grid - 1)
        moved_m = (abs(nr - self.pos[0]) + abs(nc - self.pos[1])) * self.cell_m
        self.pos = (nr, nc)
        e = max(moved_m * self.drive_j_per_m, 1.0)                 # never free (a step still costs a little)
        self.energy -= e
        info_gain = self._observe()                               # uncertainty reduced at the new viewpoint
        reward = float(info_gain / e)                             # information per joule (the map-channel reward)
        mapped = float(self.sigma.mean()) < OBS_SIGMA_FLOOR_M * 1.25
        terminated = bool(mapped)
        truncated = bool(self.energy <= 0.0)
        info = {"sigma_mean": float(self.sigma.mean()), "energy_frac": max(0.0, self.energy) / self.energy_budget,
                "map_rmse_m": float(np.sqrt(np.mean((self.est - self.truth) ** 2)))}
        obs = self._obs()
        return (obs, reward, terminated, truncated, info) if _HAS_GYM else (obs, reward, terminated or truncated, info)


def greedy_action(env: ActivePerceptionEnv) -> int:
    """One-step next-best-view: the move whose new viewpoint reduces summed sigma the most per joule."""
    best_a, best = 0, -1.0
    for a, (dr, dc) in enumerate(_MOVES):
        nr = min(max(env.pos[0] + dr, 0), env.grid - 1)
        nc = min(max(env.pos[1] + dc, 0), env.grid - 1)
        dist = np.hypot(env._rr - nr, env._cc - nc)
        in_view = dist <= env.sensor_r
        obs_sig = OBS_SIGMA_FLOOR_M * (1.0 + (dist / env.sensor_r) ** 2)
        var = env.sigma ** 2
        gain = float((np.where(in_view, var - var * obs_sig ** 2 / (var + obs_sig ** 2), 0.0)).sum())
        moved_m = (abs(nr - env.pos[0]) + abs(nc - env.pos[1])) * env.cell_m
        per_j = gain / max(moved_m * env.drive_j_per_m, 1.0)
        if per_j > best:
            best, best_a = per_j, a
    return best_a


def beam_action(env: ActivePerceptionEnv, *, lookahead: int = 4, beam_width: int = 6) -> int:
    """Multi-step next-best-view: beam-search `lookahead` viewpoints ahead (the exact env is its own
    model), score by cumulative info-gain per joule, and return the FIRST action of the best sequence
    (receding horizon). lookahead=1 reduces to greedy; lookahead>1 routes ahead so it stops re-covering."""
    rr, cc, sr, grid, cm, jm = env._rr, env._cc, env.sensor_r, env.grid, env.cell_m, env.drive_j_per_m
    beam = [(0.0, env.sigma, env.pos, None)]                      # (cum score, sigma, pos, first action)
    for _ in range(max(1, lookahead)):
        cand = []
        for score, sigma, pos, first in beam:
            ssum = float(sigma.sum())
            for a, (dr, dc) in enumerate(_MOVES):
                nr = min(max(pos[0] + dr, 0), grid - 1)
                nc = min(max(pos[1] + dc, 0), grid - 1)
                e = max((abs(nr - pos[0]) + abs(nc - pos[1])) * cm * jm, 1.0)
                new_sigma, _ = _fuse_sigma(sigma, (nr, nc), rr, cc, sr)
                gain = ssum - float(new_sigma.sum())
                cand.append((score + gain / e, new_sigma, (nr, nc), a if first is None else first))
        cand.sort(key=lambda x: -x[0])
        beam = cand[:beam_width]
    return beam[0][3]
