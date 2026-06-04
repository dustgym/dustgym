"""SkillMacroEnv — skill-macro construction RL env (M2/M3, 2026-06-02).

`state -> skill -> parameters`: one action = one construction macro: SELECT a cell in the
work region + a mode, and the macro services that cell's disc TOWARD the target (cut excess
into the drum, or dump deficit from the drum) via the conserved authority (mass conservation
guaranteed). Collapses the long horizon to cell-selection decisions.

M3 RESOURCE MODEL (the constraints that make planning matter; default OFF -> unconstrained):
  - drum_capacity_kg : the drum can hold only so much -> must cut before you can dump.
  - energy_budget    : the rover travels to each selected cell; travel + dig cost energy;
                       the episode FAILS (truncates) when energy runs out -> routing/ordering
                       matter (a naive far-jumping selector exhausts the battery before it
                       finishes; a nearest/capacity-aware planner succeeds within budget).
Unconstrained (the defaults) the toward-target macro is ~closed-form (greedy/random solve it);
the resource layer is what turns selection+ordering into a real RL/ML planning problem.

Action (Box(3), [-1,1]): [row_frac, col_frac, mode] -> region cell + (mode>0 cut / <0 dump).
Observation: region target-error field (obs_k x obs_k) + [drum fill, energy frac, rover row/col].
Gymnasium-optional (bare-numpy core; gym.Env + spaces.Box when gymnasium present).
"""
from __future__ import annotations

import dataclasses

import numpy as np

from . import challenge as chmod
from .column_state import StateLabel

try:
    import gymnasium as _gym
    from gymnasium import spaces as _spaces
    _HAS_GYM = True
    _BASE = _gym.Env
except Exception:                              # pragma: no cover
    _gym = None; _spaces = None; _HAS_GYM = False; _BASE = object

HAS_GYM = _HAS_GYM


class SkillMacroEnv(_BASE):
    metadata = {"render_modes": []}

    def __init__(self, challenge, *, obs_k: int = 8, disc_cells: float = 2.0,
                 cut_per_macro_m: float = 0.04, match_scale: float = 50.0,
                 macro_cost: float = 0.01,
                 # --- M3 resource model (default OFF = unconstrained) ---
                 energy_budget: float = float("inf"),
                 drum_capacity_kg: float = float("inf"),
                 travel_cost_per_cell: float = 0.0,
                 dig_cost_per_kg: float = 0.0,
                 energy_penalty: float = 0.0,
                 discrete_cells: int = 0):     # 0=continuous Box; >0 = Discrete(dc*dc*2) cell+mode
        super().__init__()
        self.challenge = challenge
        self.obj = challenge.objective
        self.region = self.obj.region
        self.tol = self.obj.tolerance_m
        self.grid = challenge.map.grid
        self.cell_m = challenge.map.cell_m
        self.max_macros = challenge.constraints.max_time_steps
        self.obs_k = int(obs_k)
        self.disc = float(disc_cells)
        self.cut_per_macro_m = float(cut_per_macro_m)
        self.match_scale = float(match_scale)
        self.macro_cost = float(macro_cost)
        self.energy_budget = float(energy_budget)
        self.drum_capacity_kg = float(drum_capacity_kg)
        self.travel_cost_per_cell = float(travel_cost_per_cell)
        self.dig_cost_per_kg = float(dig_cost_per_kg)
        self.energy_penalty = float(energy_penalty)
        self.discrete_cells = int(discrete_cells)
        self.obs_dim = self.obs_k * self.obs_k + 4     # err field + [drum, energy, rover_r, rover_c]
        self.action_dim = 3

        self.inst = None; self.cs = None; self._steps = 0; self._rmse = 0.0; self._m0 = 0.0
        self.rc = None; self._energy = 0.0
        if _HAS_GYM:
            if self.discrete_cells:
                self.action_space = _spaces.Discrete(self.discrete_cells * self.discrete_cells * 2)
            else:
                self.action_space = _spaces.Box(-1.0, 1.0, shape=(self.action_dim,), dtype=np.float32)
            hi = np.full(self.obs_dim, 1.0e3, dtype=np.float32)
            self.observation_space = _spaces.Box(-hi, hi, dtype=np.float32)

    def _decode(self, action):
        """Return (rf, cf, mode) in [0,1]^2 x [-1,1] from a continuous Box or a Discrete index."""
        if self.discrete_cells:
            dc = self.discrete_cells
            idx = int(action) if np.isscalar(action) else int(np.asarray(action).ravel()[0])
            cell, mode_bit = idx // 2, idx % 2
            cr, cc = cell // dc, cell % dc
            return cr / max(1, dc - 1), cc / max(1, dc - 1), (1.0 if mode_bit == 0 else -1.0)
        a = np.asarray(action, dtype=np.float64).reshape(-1)
        return (float(np.clip(a[0], -1, 1)) + 1) / 2, (float(np.clip(a[1], -1, 1)) + 1) / 2, float(np.clip(a[2], -1, 1))

    # -- helpers -------------------------------------------------------------

    def _err(self):
        return self.cs.derive_height() - self.inst.target_height

    def _rmse_region(self):
        return chmod.terrain_rmse(self.cs.derive_height(), self.inst.target_height, self.region)

    def _disc_mask(self, rc):
        r0, c0 = rc
        rr = np.arange(self.grid)[:, None] - r0
        cc = np.arange(self.grid)[None, :] - c0
        m = (rr * rr + cc * cc) <= self.disc * self.disc
        reg = np.zeros_like(m)
        a, b, c, d = self.region
        reg[a:c, b:d] = True
        return m & reg

    def _region_center(self):
        a, b, c, d = self.region
        return ((a + c) / 2.0, (b + d) / 2.0)

    def _obs(self):
        err = self._err()
        a, b, c, d = self.region
        rows = np.linspace(a, c - 1, self.obs_k).round().astype(int)
        cols = np.linspace(b, d - 1, self.obs_k).round().astype(int)
        patch = err[np.ix_(rows, cols)].ravel()
        drum = self.cs.drum_inventory / self.drum_capacity_kg if np.isfinite(self.drum_capacity_kg) \
            else self.cs.drum_inventory / max(1.0, self.cs.grid_mass())
        e = (self._energy / self.energy_budget) if np.isfinite(self.energy_budget) else 1.0
        rf = (self.rc[0] - a) / max(1.0, c - a)
        cf = (self.rc[1] - b) / max(1.0, d - b)
        return np.concatenate([patch, [drum, e, rf, cf]]).astype(np.float32)

    # -- gym API -------------------------------------------------------------

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        if _HAS_GYM:
            super().reset(seed=seed)
        c = self.challenge
        if seed is not None:
            c = dataclasses.replace(c, map=dataclasses.replace(c.map, seed=seed))
        self.inst = chmod.realize(c)
        self.cs = self.inst.cs
        self._m0 = self.cs.total_mass()
        self._steps = 0
        self._rmse = self._rmse_region()
        self.rc = self._region_center()
        self._energy = self.energy_budget
        return self._obs(), {"rmse": self._rmse, "energy": self._energy}

    def step(self, action):
        rf, cf, mode = self._decode(action)
        r0, c0, r1, c1 = self.region
        tgt_rc = (r0 + rf * (r1 - 1 - r0), c0 + cf * (c1 - 1 - c0))

        # --- travel cost (rover drives to the selected cell) ---
        dist = float(np.hypot(tgt_rc[0] - self.rc[0], tgt_rc[1] - self.rc[1]))
        self.rc = tgt_rc
        energy_spent = self.travel_cost_per_cell * dist

        mask = self._disc_mask(tgt_rc)
        moved_kg = 0.0
        if mask.any():
            h = self.cs.derive_height()
            tgt = self.inst.target_height
            if mode > 1.0 / 3.0:                         # Excavate toward target (cut excess -> drum)
                excess = np.maximum(h - tgt, 0.0)
                mpc = np.minimum(excess, self.cut_per_macro_m) * self.cs.density
                room = self.drum_capacity_kg - self.cs.drum_inventory      # capacity gate
                if room > 0:
                    want = float(mpc[mask].sum()) * self.cs.cell_area
                    if want > room and want > 0:
                        mpc = mpc * (room / want)                          # scale to fit capacity
                    before = self.cs.drum_inventory
                    self.cs.cut_to_inventory(mask, mpc)
                    self.cs.state_label[mask] = StateLabel.EXCAVATED
                    moved_kg = self.cs.drum_inventory - before
            elif mode < -1.0 / 3.0 and self.cs.drum_inventory > 0.0:       # Dump toward target (drum -> deficit)
                deficit = np.minimum(np.maximum(tgt - h, 0.0), self.cut_per_macro_m)
                want_kg = float((deficit * self.cs.density)[mask].sum()) * self.cs.cell_area
                moved_kg = self.cs.dump_from_inventory(mask, min(want_kg, self.cs.drum_inventory))
        energy_spent += self.dig_cost_per_kg * moved_kg
        self._energy -= energy_spent

        self._steps += 1
        new = self._rmse_region()
        reward = (self._rmse - new) * self.match_scale - self.macro_cost - self.energy_penalty * energy_spent
        self._rmse = new
        success = new <= self.tol
        if success:
            reward += 1.0
        out_of_energy = self._energy <= 0.0
        terminated = bool(success)
        truncated = (self._steps >= self.max_macros) or out_of_energy
        info = {"success": success, "rmse": new, "drum": self.cs.drum_inventory,
                "energy": self._energy, "out_of_energy": out_of_energy, "rc": list(self.rc)}
        return self._obs(), float(reward), terminated, truncated, info


def greedy_action(env: SkillMacroEnv):
    """Unconstrained greedy: cut the highest above-target cell; else dump the lowest below-target."""
    err = env._err()
    r0, c0, r1, c1 = env.region
    sub = err[r0:r1, c0:c1]
    if sub.max() > env.tol:
        idx = np.unravel_index(np.argmax(sub), sub.shape); mode = 1.0
    else:
        idx = np.unravel_index(np.argmin(sub), sub.shape); mode = -1.0
    rf = idx[0] / max(1, r1 - 1 - r0); cf = idx[1] / max(1, c1 - 1 - c0)
    return [rf * 2 - 1, cf * 2 - 1, mode]


def _greedy_nearest_target(env: SkillMacroEnv):
    """Core M3 planner: pick the NEAREST above-target cell to cut (if drum has room) else the
    nearest below-target cell to dump. Returns (rf, cf in [0,1], mode in {+1 cut, -1 dump})."""
    err = env._err()
    r0, c0, r1, c1 = env.region
    rr, cc = np.mgrid[r0:r1, c0:c1]
    dist = np.hypot(rr - env.rc[0], cc - env.rc[1])
    room = (env.cs.drum_inventory < 0.9 * env.drum_capacity_kg) if np.isfinite(env.drum_capacity_kg) else True
    above = err[r0:r1, c0:c1] > env.tol
    below = err[r0:r1, c0:c1] < -env.tol
    if room and above.any():
        d = np.where(above, dist, np.inf); mode = 1.0
    elif env.cs.drum_inventory > 0.0 and below.any():
        d = np.where(below, dist, np.inf); mode = -1.0
    elif above.any():
        d = np.where(below, dist, np.inf) if below.any() else np.where(above, dist, np.inf)
        mode = -1.0 if (below.any() and env.cs.drum_inventory > 0) else 1.0
    else:
        d = np.where(below, dist, np.inf); mode = -1.0
    idx = np.unravel_index(np.argmin(d), d.shape)
    return idx[0] / max(1, r1 - 1 - r0), idx[1] / max(1, c1 - 1 - c0), mode


def greedy_nearest_action(env: SkillMacroEnv):
    """Capacity/travel-aware planner (M3), continuous action form."""
    rf, cf, mode = _greedy_nearest_target(env)
    return [rf * 2 - 1, cf * 2 - 1, mode]


def greedy_nearest_discrete(env: SkillMacroEnv):
    """The same planner as a Discrete(dc*dc*2) index (cell bucket + cut/dump) — the learnable form."""
    rf, cf, mode = _greedy_nearest_target(env)
    dc = env.discrete_cells
    cr = int(round(rf * (dc - 1))); cc = int(round(cf * (dc - 1)))
    return (cr * dc + cc) * 2 + (0 if mode > 0 else 1)
