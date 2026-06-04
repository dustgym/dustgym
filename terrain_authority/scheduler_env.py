"""scheduler_env.py — multi-objective construction SCHEDULING env (M4, 2026-06-02).

Why this is the env where learning/search earns its keep (the M3 finding): with the IPEx-grounded
energy model the cost is DIG-DOMINATED (~4151 J/kg vs ~2.7 J/cell travel) and dig energy is fixed by
the conserved mass a target requires — so on a single objective there is ~no planning headroom. What
ordering/routing DOES control is MAKESPAN: how many round-trips the one rover/one drum makes to fill
several separated build sites from several borrow pits. So the binding budget here is trip-LEGS (time),
and each action is one strategic leg:

  action = Discrete(num_regions):
    - a BORROW region -> drive there, fill the drum from its excess (one load leg)
    - a BUILD  region -> drive there, fill_toward its target with the drum (one dump leg, FIX-4)

The conserved authority still MUTATES terrain (mass conservation guaranteed); the policy only chooses
the SEQUENCE of legs. A good scheduler batches full drum loads, sources each site from a pit that still
has material, skips satisfied sites / empty pits, and orders to cut deadheading — finishing all sites
within the leg budget. A naive/random scheduler wastes legs (visiting satisfied sites, empty pits,
partial loads) and runs out of budget. Mass-conserved, energy grounded (tracked), deterministic.

Observation (finite, gym-checked): per region [remaining-work frac, distance-from-rover frac] for all
regions (borrows then builds), then [drum fill frac, legs-remaining frac].
Gymnasium-optional (bare-numpy core; gym.Env + spaces when present).
"""
from __future__ import annotations

import numpy as np

try:
    import gymnasium as _gym
    from gymnasium import spaces as _spaces
    _HAS_GYM = True
    _BASE = _gym.Env
except Exception:                              # pragma: no cover
    _gym = None; _spaces = None; _HAS_GYM = False; _BASE = object

HAS_GYM = _HAS_GYM


class SchedulerEnv(_BASE):
    metadata = {"render_modes": []}

    def __init__(self, *, grid=64, cell_m=0.5, borrows, builds, fill_delta_m=0.10,
                 mound_height_m=0.30, tol_m=0.01, drum_capacity_kg=120.0, max_legs=40,
                 travel_cost_per_cell=1.0, dig_cost_per_kg=0.0, energy_budget=float("inf"),
                 match_scale=20.0, leg_cost=0.05, randomize=False,
                 borrow_size=8, build_size=4, drum_sensor=None):
        super().__init__()
        self.grid = int(grid); self.cell_m = float(cell_m)
        self.borrows = [tuple(b) for b in borrows]
        self.builds = [tuple(b) for b in builds]
        self.regions = self.borrows + self.builds
        self.n_borrow = len(self.borrows); self.n_region = len(self.regions)
        self.randomize = bool(randomize)
        self.borrow_size = int(borrow_size); self.build_size = int(build_size)
        self.fill_delta_m = float(fill_delta_m); self.mound_height_m = float(mound_height_m)
        self.tol = float(tol_m); self.drum_capacity_kg = float(drum_capacity_kg)
        self.max_legs = int(max_legs)
        self.travel_cost_per_cell = float(travel_cost_per_cell)
        self.dig_cost_per_kg = float(dig_cost_per_kg)
        self.energy_budget = float(energy_budget)
        self.match_scale = float(match_scale); self.leg_cost = float(leg_cost)
        self.drum_sensor = drum_sensor                     # optional DrumSensor: sensed (vs true) drum fill in obs
        self.cell_area = self.cell_m * self.cell_m
        self.obs_dim = 2 * self.n_region + 2

        self.cs = None; self.target = None; self.rc = None
        self._legs = 0; self._energy = 0.0; self._deficit0 = 0.0
        if _HAS_GYM:
            self.action_space = _spaces.Discrete(self.n_region)
            hi = np.full(self.obs_dim, 1.0e3, dtype=np.float32)
            self.observation_space = _spaces.Box(-hi, hi, dtype=np.float32)

    # -- geometry helpers ----------------------------------------------------
    def _mask(self, rect):
        a, b, c, d = rect
        m = np.zeros((self.grid, self.grid), bool); m[a:c, b:d] = True
        return m

    def _centroid(self, rect):
        a, b, c, d = rect
        return ((a + c - 1) / 2.0, (b + d - 1) / 2.0)

    def _build_deficit_total(self):
        h = self.cs.derive_height()
        tot = 0.0
        for rect in self.builds:
            a, b, c, d = rect
            tot += float(np.maximum(self.target[a:c, b:d] - h[a:c, b:d], 0.0).sum())
        return tot

    def _borrow_excess_total(self, rect):
        h = self.cs.derive_height(); a, b, c, d = rect
        return float(np.maximum(h[a:c, b:d] - self.target[a:c, b:d], 0.0).sum())

    def _build_deficit(self, rect):
        h = self.cs.derive_height(); a, b, c, d = rect
        return float(np.maximum(self.target[a:c, b:d] - h[a:c, b:d], 0.0).sum())

    def _sample_layout(self, rng):
        """Place n_borrow borrow squares + n_build build squares, non-overlapping, within a margin.
        Keeps region COUNTS fixed (so the action space is stable) and varies only positions/sizes-fixed."""
        placed = []
        sizes = [self.borrow_size] * self.n_borrow + [self.build_size] * (self.n_region - self.n_borrow)
        m = 2
        for s in sizes:
            for _ in range(200):
                a = int(rng.integers(m, self.grid - s - m)); b = int(rng.integers(m, self.grid - s - m))
                rect = (a, b, a + s, b + s)
                if all(not (rect[0] < q[2] + 1 and q[0] < rect[2] + 1 and
                            rect[1] < q[3] + 1 and q[1] < rect[3] + 1) for q in placed):
                    placed.append(rect); break
            else:
                placed.append((a, b, a + s, b + s))         # fallback: accept (rare at this density)
        return placed[:self.n_borrow], placed[self.n_borrow:]

    # -- map realization -----------------------------------------------------
    def reset(self, *, seed=None, options=None):
        if _HAS_GYM:
            super().reset(seed=seed)
        if self.randomize:
            rng = self.np_random if (_HAS_GYM and getattr(self, "np_random", None) is not None) \
                else np.random.default_rng(seed)
            self.borrows, self.builds = self._sample_layout(rng)
            self.regions = self.borrows + self.builds
        from .column_state import ColumnState
        cs = ColumnState(width=self.grid, height=self.grid, cell_m=self.cell_m)
        # borrow pits start as mounds (material to cut); builds start flat (deficit to fill).
        base = cs.derive_height().copy()
        self.target = base.copy()
        for rect in self.builds:
            a, b, c, d = rect
            self.target[a:c, b:d] = base[a:c, b:d] + self.fill_delta_m   # raise target -> deficit
        for rect in self.borrows:
            a, b, c, d = rect
            cs.mass_areal[a:c, b:d] += self.mound_height_m * cs.density[a:c, b:d]   # pile material
        self.cs = cs
        self.rc = self._centroid(self.builds[0])
        self._legs = 0
        self._energy = self.energy_budget
        self._deficit0 = max(1e-9, self._build_deficit_total())
        self._prev_def = self._deficit0
        return self._obs(), {"deficit": self._deficit0, "energy": self._energy}

    def _obs(self):
        feats = []
        for i, rect in enumerate(self.regions):
            if i < self.n_borrow:
                a, b, c, d = rect
                work = self._borrow_excess_total(rect) / max(1e-9, (c - a) * (d - b) * self.mound_height_m)
            else:
                work = self._build_deficit(rect) / max(1e-9, self._deficit0)
            dist = np.hypot(*(np.subtract(self._centroid(rect), self.rc))) / self.grid
            feats += [float(np.clip(work, 0, 1)), float(dist)]
        inv_kg = (self.cs.drum_inventory if self.drum_sensor is None
                  else self.drum_sensor.observe(self.cs.drum_inventory))   # sensed drum fill (optional)
        drum = inv_kg / self.drum_capacity_kg if np.isfinite(self.drum_capacity_kg) else 0.0
        legs_left = 1.0 - self._legs / self.max_legs
        return np.array(feats + [float(drum), float(legs_left)], dtype=np.float32)

    def step(self, action):
        a = int(action) if np.isscalar(action) else int(np.asarray(action).ravel()[0])
        a = max(0, min(self.n_region - 1, a))
        rect = self.regions[a]
        # drive to the region (one leg, travel cost)
        cen = self._centroid(rect)
        dist = float(np.hypot(cen[0] - self.rc[0], cen[1] - self.rc[1]))
        self.rc = cen
        self._energy -= self.travel_cost_per_cell * dist
        moved = 0.0
        h = self.cs.derive_height()
        if a < self.n_borrow:                              # LOAD: cut excess into the drum, up to room
            mask = self._mask(rect)
            excess = np.maximum(h - self.target, 0.0)
            mpc = excess * self.cs.density                  # areal kg to remove per cell
            room = self.drum_capacity_kg - self.cs.drum_inventory
            if room > 0:
                want = float(mpc[mask].sum()) * self.cell_area
                if want > room and want > 0:
                    mpc = mpc * (room / want)
                before = self.cs.drum_inventory
                self.cs.cut_to_inventory(mask, mpc)
                moved = self.cs.drum_inventory - before
        else:                                              # DUMP: fill_toward this site with the drum
            moved = self.cs.fill_toward(self._mask(rect), self.target)
        self._energy -= self.dig_cost_per_kg * moved
        self._legs += 1
        new_def = self._build_deficit_total()
        reward = (self._prev_def - new_def) / self._deficit0 * self.match_scale - self.leg_cost
        self._prev_def = new_def
        success = new_def <= self.tol * self._n_build_cells()
        if success:
            reward += 5.0
        out_of_energy = self._energy <= 0.0
        terminated = bool(success)
        truncated = (self._legs >= self.max_legs) or out_of_energy
        info = {"success": success, "deficit": new_def, "drum": self.cs.drum_inventory,
                "energy": self._energy, "legs": self._legs, "out_of_energy": out_of_energy}
        return self._obs(), float(reward), terminated, truncated, info

    def _n_build_cells(self):
        return sum((c - a) * (d - b) for a, b, c, d in self.builds)


def beam_search_plan(env: SchedulerEnv, width: int = 20, max_depth: int | None = None):
    """Model-based planner: use the exact, deterministic authority as its own simulator and beam-search
    for a near-optimal (fewest-leg) success from env's CURRENT state. Returns the action list to replay.

    Because the authority is exact and sub-ms, model-based search beats model-free RL here: it finds the
    true makespan optimum (e.g. 24 legs where the greedy heuristic uses 28 and model-free PPO 27). The
    best LEARNED policy is this search distilled into a net (AlphaZero pattern) -- see
    scripts/demo/distill_scheduler.py. Pure numpy; eval/planning only (deep-copies env states)."""
    import copy
    md = env.max_legs if max_depth is None else int(max_depth)
    beam = [(copy.deepcopy(env), 0, False, False, [])]
    best = None; best_path = []
    for _ in range(md):
        cand = []
        for e, legs, done, succ, path in beam:
            if done:
                cand.append((e, legs, done, succ, path)); continue
            for a in range(e.n_region):
                ec = copy.deepcopy(e); _, _, te, tr, info = ec.step(a)
                cand.append((ec, info["legs"], te or tr, info["success"], path + [a]))
                if info["success"] and (best is None or info["legs"] < best):
                    best = info["legs"]; best_path = path + [a]
        cand.sort(key=lambda x: (0 if x[3] else 1, x[0]._build_deficit_total(), x[1]))
        beam = cand[:width]
        if all(x[2] for x in beam):
            break
    return best_path


def greedy_nearest_schedule(env: SchedulerEnv):
    """Strong baseline: if the drum has room and any pit still has material, go to the NEAREST such pit;
    else if the drum has material and any site still needs fill, go to the NEAREST such site; else dump
    wherever still needed / load wherever available. Batches loads + sources by proximity."""
    room = env.cs.drum_inventory < 0.95 * env.drum_capacity_kg
    pits = [(i, r) for i, r in enumerate(env.borrows) if env._borrow_excess_total(r) > 1e-9]
    sites = [(env.n_borrow + j, r) for j, r in enumerate(env.builds) if env._build_deficit(r) > env.tol]
    def nearest(cands):
        return min(cands, key=lambda ir: np.hypot(*(np.subtract(env._centroid(ir[1]), env.rc))))[0]
    if room and pits and (env.cs.drum_inventory <= 0 or sites):
        # load when we have room and there is something to fill later (or drum empty)
        if env.cs.drum_inventory <= 0 or (sites and room):
            return nearest(pits)
    if env.cs.drum_inventory > 0 and sites:
        return nearest(sites)
    if pits:
        return nearest(pits)
    if sites:
        return nearest(sites)
    return 0
