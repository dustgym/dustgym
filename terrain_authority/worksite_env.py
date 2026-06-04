"""worksite_env.py — RL controller over John McCardle's WorkSite seam (reconciliation, 2026-06-03).

WorkSite (terrain_authority/worksite.py, PR #5) is the streaming execution engine: a coarse base +
rover-following fine window with a GLOBAL drum ledger, exposing `.flatten()/.dump()/.drive()/.relax()`
"shaped so an RL policy can drive the SAME seam later -- the controller is the only stub." This is that
controller: a Gymnasium env whose actions are WorkSite construction verbs, executed on the real window.

Task (cut-haul-fill, the genuine planning regime): flatten a bumpy PAD to a level (dig -> ledger) and
build a BERM elsewhere by dumping that material (ledger -> grid). The drum ledger couples them: you
cannot dump more than you have dug, so the binding decision is WHEN to switch cut->dump (batching), the
same finding as the standalone SchedulerEnv -- but now mass flows through John's conserved WorkSite ledger
(`inventory_kg`), not an ad-hoc env. Action = Discrete(2): 0 flatten the next pad slice, 1 dump the next
berm slice. Mass is conserved by WorkSite (grid + inventory_kg invariant).

Gymnasium-optional (bare-numpy core; gym.Env + spaces when present).
"""
from __future__ import annotations

import numpy as np

from . import constants as K

try:
    import gymnasium as _gym
    from gymnasium import spaces as _spaces
    _HAS_GYM = True
    _BASE = _gym.Env
except Exception:                              # pragma: no cover
    _gym = None; _spaces = None; _HAS_GYM = False; _BASE = object

HAS_GYM = _HAS_GYM


def _bumpy_base(n_base=8, base_cell_m=0.5, roughness_m=0.03, seed=0):
    """A small synthetic coarse base with surface bumps (so the pad has excess to cut)."""
    from .column_state import ColumnState
    cs = ColumnState(width=n_base, height=n_base, cell_m=base_cell_m)
    rng = np.random.default_rng(seed)
    cs.mass_areal += (rng.random((n_base, n_base)) * roughness_m) * cs.density   # bumps as mass
    return cs


class WorkSiteConstructEnv(_BASE):
    metadata = {"render_modes": []}

    def __init__(self, *, n_base=8, base_cell_m=0.5, fine_cell_m=0.1, roughness_m=0.15,
                 berm_delta_m=0.025, n_slices=6, max_steps=13, tol_frac=0.15,
                 match_scale=10.0, step_cost=0.05, seed=0,
                 bundle_dir=None, charges=None, work_cells=None, window_radius_m=8.0,
                 flat_window=False, flat_topk=24, cut_depth_m=None, drum_sensor=None):
        super().__init__()
        self.n_base = int(n_base); self.base_cell_m = float(base_cell_m)
        self.fine_cell_m = float(fine_cell_m); self.roughness_m = float(roughness_m)
        self.berm_delta_m = float(berm_delta_m); self.n_slices = int(n_slices)
        self.max_steps = int(max_steps); self.tol_frac = float(tol_frac)
        self.match_scale = float(match_scale); self.step_cost = float(step_cost)
        self._seed0 = int(seed)
        # Real-DEM mode + PHYSICS-GROUNDED budget (answers "why N steps?"): when `charges` is given the
        # episode is bounded by ENERGY from the IPEx battery (ipex_specs), not a step count -- each
        # flatten/dump spends dig_energy_per_kg * mass_moved + travel, and the rover runs until the
        # battery (1332 Wh/charge * charges) is exhausted. `bundle_dir` loads the real Haworth DEM;
        # `work_cells` sets a small centred work area so the task fits a realistic energy budget.
        self.bundle_dir = bundle_dir
        self.work_cells = int(work_cells) if work_cells else None
        self.window_radius_m = float(window_radius_m)
        from . import ipex_specs as _ix
        self.dig_J_per_kg = _ix.dig_energy_per_kg()        # grounded: 4151 J/kg
        self.travel_J_per_m = _ix.drive_energy_per_m()     # grounded: 135 J/m
        self.charges = charges
        self.energy_budget_j = (_ix.battery_energy_j() * float(charges)) if charges else float("inf")
        self._ws_cache = None                              # real-DEM WorkSite (loaded once, reused)
        self.flat_window = bool(flat_window); self.flat_topk = int(flat_topk)
        self.cut_depth_m = float(cut_depth_m) if cut_depth_m else None   # balanced cut-haul-fill depth
        # Optional drum-fill SENSING (rassor_mass_model.DrumSensor): when set, the drum-fill observation is
        # the motor-current INFERRED mass (optionally noisy) instead of the true ledger -> the policy plans
        # under realistic imperfect drum knowledge. None (default) = perfect knowledge (non-breaking).
        self.drum_sensor = drum_sensor
        self._flat_rc = None                               # cached flattest base-tile centres
        self._energy = float("inf"); self._pad_berm_dist_m = 0.0; self._last_region = None
        self.ws = None; self.fine = None
        self.pad_rows = None; self.berm_rows = None; self.pad_target = 0.0
        self.berm_target = None; self._pad_done = None; self._berm_done = None
        self._steps = 0
        self.obs_dim = 4
        if _HAS_GYM:
            self.action_space = _spaces.Discrete(2)            # 0 = flatten pad slice, 1 = dump berm slice
            hi = np.full(self.obs_dim, 1.0e3, dtype=np.float32)
            self.observation_space = _spaces.Box(-hi, hi, dtype=np.float32)

    # -- geometry -----------------------------------------------------------
    def _slice_mask(self, rows, k):
        m = np.zeros((self.fine.height, self.fine.width), bool)
        r = rows[k]
        m[r[0]:r[1], self._cwin[0]:self._cwin[1]] = True
        return m

    def _pad_excess_kg(self):
        h = self.fine.derive_height()
        tot = 0.0
        for (r0, r1) in self.pad_rows:
            tot += float(np.maximum(h[r0:r1, self._cwin[0]:self._cwin[1]] - self.pad_target, 0.0).sum())
        return tot

    def _berm_deficit_kg(self):
        h = self.fine.derive_height()
        tot = 0.0
        for (r0, r1) in self.berm_rows:
            tot += float(np.maximum(self.berm_target - h[r0:r1, self._cwin[0]:self._cwin[1]], 0.0).sum())
        return tot

    # -- gym API ------------------------------------------------------------
    def reset(self, *, seed=None, options=None):
        if _HAS_GYM:
            super().reset(seed=seed)
        s = self._seed0 if seed is None else int(seed)
        from .worksite import WorkSite
        if self.bundle_dir:                                # REAL Haworth DEM (loaded once, reused)
            if self._ws_cache is None:                    # G7 smooth_datum removes 5 m DEM terrace cliffs
                self._ws_cache = WorkSite.from_haworth_bundle(self.bundle_dir, fine_cell_m=self.fine_cell_m,
                                                              smooth_datum=True)
            self.ws = self._ws_cache
            self.ws.inventory_kg = 0.0                     # episodic reset of the global ledger
            rng = np.random.default_rng(s)
            if self.flat_window:                          # pick a (seeded) FLAT site -> solvable, no slip
                if self._flat_rc is None:
                    self._flat_rc = self._find_flat_windows()
                br, bc = self._flat_rc[int(rng.integers(len(self._flat_rc)))]
            else:
                m = 6
                br = int(rng.integers(m, self.ws.base.height - m))
                bc = int(rng.integers(m, self.ws.base.width - m))
            self.ws.open_window((br, bc), radius_m=self.window_radius_m)
        else:                                              # synthetic bumpy base (toy)
            base = _bumpy_base(self.n_base, self.base_cell_m, self.roughness_m, seed=s)
            self.ws = WorkSite(base, world_x0=0.0, world_y0=0.0, fine_cell_m=self.fine_cell_m)
            self.ws.open_window((self.n_base / 2.0, self.n_base / 2.0),
                                radius_m=self.base_cell_m * self.n_base)
        self.fine = self.ws.fine
        H, W = self.fine.height, self.fine.width
        if self.work_cells:                                # small centred work area (real DEM): pad above
            wc = min(self.work_cells, H // 2 - 1, W // 2 - 1)   # centre, berm below
            self._cwin = (W // 2 - wc // 2, W // 2 + wc // 2)
            pad_band = (H // 2 - wc, H // 2); berm_band = (H // 2, H // 2 + wc)
        else:                                              # full-window bands (toy)
            self._cwin = (W // 4, 3 * W // 4)
            pad_band = (H // 8, H // 2); berm_band = (H // 2, 7 * H // 8)
        self._pad_berm_dist_m = abs((pad_band[0] + pad_band[1]) - (berm_band[0] + berm_band[1])) / 2.0 \
            * self.fine_cell_m                             # pad<->berm haul distance [m]
        self.pad_rows = self._split(pad_band, self.n_slices)
        self.berm_rows = self._split(berm_band, self.n_slices)
        h = self.fine.derive_height()
        pad_h = h[pad_band[0]:pad_band[1], self._cwin[0]:self._cwin[1]]
        berm_h = h[berm_band[0]:berm_band[1], self._cwin[0]:self._cwin[1]]
        if self.cut_depth_m:
            # BALANCED cut-haul-fill (flat real-DEM site): cut a uniform `cut_depth` layer off the pad,
            # raise the berm by exactly the height that consumes it (mass-balanced, so it's solvable).
            self.pad_target = float(pad_h.mean()) - self.cut_depth_m
            cut_kg = float(np.maximum(pad_h - self.pad_target, 0.0).sum()) * K.RHO_SURFACE * self.fine_cell_m ** 2
            berm_area = float(berm_h.size) * self.fine_cell_m ** 2
            raise_m = 0.95 * cut_kg / (berm_area * K.RHO_SPOIL)    # 0.95: stay just under what the cut yields
            self.berm_target = float(berm_h.mean()) + raise_m
        else:
            self.pad_target = float(pad_h.min())                  # flatten the pad to its lowest (max export)
            # toy: raise above mean; real DEM (non-balanced): raise above local min by delta
            self.berm_target = float((berm_h.min() if self.bundle_dir else berm_h.mean()) + self.berm_delta_m)
        self._pad_done = [False] * self.n_slices
        self._berm_done = [False] * self.n_slices
        self._steps = 0
        self._pad0 = max(1e-9, self._pad_excess_kg())
        self._berm0 = max(1e-9, self._berm_deficit_kg())
        self._prev = self._pad_excess_kg() / self._pad0 + self._berm_deficit_kg() / self._berm0
        self._m0 = self.ws.total_mass()                            # grid + ledger invariant
        self._energy = self.energy_budget_j                        # physics budget (J); inf when step-capped
        self._last_region = None
        return self._obs(), {"inventory_kg": self.ws.inventory_kg, "energy_j": self._energy}

    def _find_flat_windows(self):
        """Top-K flattest base-tile centres (lowest local relief) -> solvable, non-slip work sites."""
        H = self.ws.base.derive_height(); tb = self.ws.tile_base_cells; nb = H.shape[0] // tb
        rel = np.full((nb, nb), np.inf)
        for i in range(2, nb - 2):
            for j in range(2, nb - 2):
                blk = H[i * tb:(i + 1) * tb, j * tb:(j + 1) * tb]
                rel[i, j] = blk.max() - blk.min()
        flat = np.argsort(rel.ravel())[:self.flat_topk]
        return [(int(idx // nb) * tb + tb // 2, int(idx % nb) * tb + tb // 2) for idx in flat]

    def _split(self, band, n):
        edges = np.linspace(band[0], band[1], n + 1).round().astype(int)
        return [(int(edges[i]), int(edges[i + 1])) for i in range(n)]

    def _obs(self):
        pad_left = self._pad_excess_kg() / self._pad0
        berm_left = self._berm_deficit_kg() / self._berm0
        inv_kg = (self.ws.inventory_kg if self.drum_sensor is None
                  else self.drum_sensor.observe(self.ws.inventory_kg))   # sensed drum fill (optional)
        inv = inv_kg / max(1.0, self._m0)
        return np.array([pad_left, berm_left, float(inv), 1.0 - self._steps / self.max_steps],
                        dtype=np.float32)

    def step(self, action):
        a = int(action) if np.isscalar(action) else int(np.asarray(action).ravel()[0])
        mass_moved = 0.0; region = "pad" if a == 0 else "berm"
        if a == 0:                                                 # flatten next undone pad slice
            k = next((i for i, d in enumerate(self._pad_done) if not d), None)
            if k is not None:
                mass_moved = self.ws.flatten(self._slice_mask(self.pad_rows, k), self.pad_target)
                self._pad_done[k] = True
        else:                                                      # dump next undone berm slice
            k = next((i for i, d in enumerate(self._berm_done) if not d), None)
            if k is not None:
                mask = self._slice_mask(self.berm_rows, k)
                # FIX-4 per-cell fill_toward (no overshoot) instead of WorkSite.dump's even-spread, which
                # cannot profile-fill an uneven real-DEM berm to tolerance -> drives the ledger directly.
                # (This is exactly the drop-in upgrade proposed for WorkSite.dump.)
                self.fine.drum_inventory = self.ws.inventory_kg            # prime the transient register
                mass_moved = self.fine.fill_toward(mask, self.berm_target)
                self.fine.drum_inventory = 0.0
                self.ws.inventory_kg -= mass_moved                        # ledger loses what landed
                # a slice is done once it's within tolerance -- including a slice that already had no
                # deficit (mass_moved==0); the old `mass_moved>0` guard left such slices forever-undone.
                if self._berm_deficit_slice(k) <= self.tol_frac * self._berm_slice0(k):
                    self._berm_done[k] = True
        # GROUNDED ENERGY: drum work (dig_J/kg * mass) + a haul leg (travel_J/m) when switching pad<->berm
        travel_m = self._pad_berm_dist_m if (self._last_region is not None and region != self._last_region) else 0.0
        self._last_region = region
        self._energy -= self.dig_J_per_kg * mass_moved + self.travel_J_per_m * travel_m
        self._steps += 1
        cur = self._pad_excess_kg() / self._pad0 + self._berm_deficit_kg() / self._berm0
        reward = (self._prev - cur) * self.match_scale - self.step_cost
        self._prev = cur
        success = (self._pad_excess_kg() <= self.tol_frac * self._pad0
                   and self._berm_deficit_kg() <= self.tol_frac * self._berm0)
        if success:
            reward += 5.0
        terminated = bool(success)
        out_of_energy = self._energy <= 0.0                        # battery exhausted (grounded budget)
        truncated = (self._steps >= self.max_steps) or out_of_energy
        info = {"success": success, "inventory_kg": self.ws.inventory_kg, "steps": self._steps,
                "energy_j": self._energy, "out_of_energy": out_of_energy,
                "pad_excess": self._pad_excess_kg(), "berm_deficit": self._berm_deficit_kg()}
        return self._obs(), float(reward), terminated, truncated, info

    def _berm_deficit_slice(self, k):
        h = self.fine.derive_height(); r0, r1 = self.berm_rows[k]
        return float(np.maximum(self.berm_target - h[r0:r1, self._cwin[0]:self._cwin[1]], 0.0).sum())

    def _berm_slice0(self, k):
        return max(1e-9, self._berm0 / self.n_slices)


def beam_worksite_plan(env: WorkSiteConstructEnv, width: int = 12):
    """Model-based planner over the WorkSite seam: beam-search the flatten/dump schedule from env's
    CURRENT state for the fewest-step success within the budget. Returns the action list to replay.

    The conserved WorkSite engine is exact + cheap (deepcopy ~0.2 ms), so search runs at inference. With
    the corrected mechanics + the tight default budget, beam and the greedy heuristic both solve ~100% on
    held-out instances while random ~53% and model-free PPO ~0% (no slack under the tight budget): the
    heuristic/search dominate (cf. the Dust/Scheduler finding that model-based search >= model-free). Pure
    numpy; eval/planning only (deep-copies env states)."""
    import copy
    beam = [(copy.deepcopy(env), False, False, [])]    # (env, done, success, path)
    best = None; best_path = []
    for _ in range(env.max_steps):
        cand = []
        for e, done, succ, path in beam:
            if done:
                cand.append((e, done, succ, path)); continue
            for a in (0, 1):
                ec = copy.deepcopy(e); _, _, te, tr, info = ec.step(a)
                cand.append((ec, te or tr, info["success"], path + [a]))
                if info["success"] and (best is None or info["steps"] < best):
                    best = info["steps"]; best_path = path + [a]
        cand.sort(key=lambda x: (0 if x[2] else 1,
                                 x[0]._pad_excess_kg() / x[0]._pad0 + x[0]._berm_deficit_kg() / x[0]._berm0))
        beam = cand[:width]
        if all(x[1] for x in beam):
            break
    return best_path


def greedy_worksite(env: WorkSiteConstructEnv):
    """Batch policy on the WorkSite seam: flatten pad slices to build the ledger, then dump into the berm.
    Flatten while the ledger can't yet cover the next berm slice; dump once it can."""
    if any(not d for d in env._pad_done):
        # need enough material for a berm slice? estimate one slice's kg
        h = env.fine.derive_height()
        k = next((i for i, d in enumerate(env._berm_done) if not d), None)
        if k is not None:
            mask = env._slice_mask(env.berm_rows, k)
            need = float(np.maximum(env.berm_target - h, 0.0)[mask].sum()) * K.RHO_SPOIL * env.fine_cell_m ** 2
            if env.ws.inventory_kg < need:
                return 0                                            # flatten more first
        else:
            return 0
    if any(not d for d in env._berm_done) and env.ws.inventory_kg > 0:
        return 1
    return 0 if any(not d for d in env._pad_done) else 1
