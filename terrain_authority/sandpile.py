"""Sandpile cellular automaton — collapse / angle-of-repose relaxation (spec §7).

This is the showpiece mechanic: the grid-native answer to collapsing piles. Per spec §7:

    1. Dump adds mass to cells under the drum.
    2. A relaxation sweep checks each LOOSE cell's slope to its neighbors.
    3. Any cell exceeding the CRITICAL ANGLE topples EXCESS MASS downhill until all
       loose slopes are <= the angle of repose.

O(active cells) per tick, loose regions only. Produces avalanches, repose-angle slopes,
and slumping on overbuild/undercut (the cave-in time series in scenes.py).

CRITICAL DESIGN POINT (spec §7, §10): we topple **excess MASS, not height**. Mass is the
conserved invariant (INTERFACE.md §4); height is derived from mass/density. We compute the
height needed to bring a slope back to repose, convert that height delta to a MASS delta
at the donor cell's density, move that mass to the lower neighbor, and re-derive height.
The grid total mass is invariant by construction (mass only moves between cells).

Two knobs (spec §7): the repose angle theta_r (wide-envelope calibration unknown, see
constants.py / lyasko2010.pdf reduced-g granular flow is unsettled) and an optional
cohesion term that lets piles stand briefly steeper (metastability is itself a perception
hazard — a 'stable' berm that later slumps; spec §7).
"""

from __future__ import annotations

import numpy as np

from . import constants as K
from .column_state import ColumnState, loose_mask

# 4-connected (von Neumann) and 8-connected (Moore) neighbor offsets.
_NEIGHBORS_4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]
_NEIGHBORS_8 = _NEIGHBORS_4 + [(-1, -1), (-1, 1), (1, -1), (1, 1)]


class Sandpile:
    """Mass-conserving repose relaxation over a ColumnState.

    Operates on the column heights derived from mass_areal/density. ``relax_step`` does
    one stabilizing sweep; ``relax_to_rest`` iterates to quiescence, optionally capturing
    a snapshot per step so the cave-in TIME SERIES can be exported (scenes.py).
    """

    def __init__(self, cs: ColumnState, theta_r: float = K.THETA_R, *,
                 connectivity: int = 8, cohesion_steepening: float = 0.0,
                 transfer_fraction: float = 0.5):
        """
        theta_r: critical repose angle [rad] (spec §7 knob 1; wide envelope).
        connectivity: 4 or 8 neighbours.
        cohesion_steepening: extra angle [rad] piles may transiently hold before failing
            (spec §7 cohesion term -> metastability). Added to theta_r for the threshold.
        transfer_fraction: under-relaxation factor in (0,1]; <1 damps oscillation and
            yields smoother avalanche frames. Does not affect mass conservation.
        """
        self.cs = cs
        self.theta_r = theta_r
        self.neighbors = _NEIGHBORS_8 if connectivity == 8 else _NEIGHBORS_4
        self.cohesion_steepening = cohesion_steepening
        self.transfer_fraction = transfer_fraction
        # Per-neighbor horizontal run [m]: orthogonal = cell_m, diagonal = sqrt(2)*cell_m.
        self._runs = [np.hypot(dr, dc) * cs.cell_m for (dr, dc) in self.neighbors]

    # -- manual perturbation ----------------------------------------------

    def deposit(self, r: int, c: int, mass_kg: float, *, radius_cells: int = 0,
                spoil_density: float = K.RHO_SPOIL) -> None:
        """Add ``mass_kg`` (absolute) to cell (r,c) [or a small disc] as loose SPOIL.

        Manual perturbation hook (spec §7 step 1). Used by scenes.py to over-steepen a
        crater wall before relaxing it (the cave-in). Mass comes from "outside" the grid
        (caller's bookkeeping) — this raises grid mass by mass_kg, so callers wanting a
        closed budget should pull it from drum_inventory (see ColumnState.dump_*).
        """
        cs = self.cs
        if radius_cells <= 0:
            cells = [(r, c)]
        else:
            cells = []
            for dr in range(-radius_cells, radius_cells + 1):
                for dc in range(-radius_cells, radius_cells + 1):
                    if dr * dr + dc * dc <= radius_cells * radius_cells:
                        rr, cc = r + dr, c + dc
                        if 0 <= rr < cs.height and 0 <= cc < cs.width:
                            cells.append((rr, cc))
        per_cell_kg = mass_kg / len(cells)
        per_cell_areal = per_cell_kg / cs.cell_area
        for (rr, cc) in cells:
            old = cs.mass_areal[rr, cc]
            add = per_cell_areal
            denom = old / cs.density[rr, cc] + add / spoil_density
            cs.density[rr, cc] = (old + add) / denom if denom > 0 else spoil_density
            cs.mass_areal[rr, cc] = old + add
            from .column_state import StateLabel
            cs.state_label[rr, cc] = StateLabel.SPOIL

    # -- relaxation --------------------------------------------------------

    def _max_loose_slope(self) -> float:
        """Largest downhill slope angle [rad] among loose cells (for stopping)."""
        cs = self.cs
        height = cs.derive_height()
        mask = loose_mask(cs)
        max_ang = 0.0
        for (dr, dc), run in zip(self.neighbors, self._runs):
            h2 = np.roll(np.roll(height, -dr, axis=0), -dc, axis=1)
            valid = self._shift_valid(dr, dc) & mask
            drop = np.where(valid, height - h2, 0.0)
            ang = np.arctan2(np.maximum(drop, 0.0), run)
            m = float(ang.max()) if ang.size else 0.0
            max_ang = max(max_ang, m)
        return max_ang

    def _shift_valid(self, dr: int, dc: int) -> np.ndarray:
        """Bool mask of cells whose (dr,dc) neighbor is in-bounds (kills wrap-around)."""
        h, w = self.cs.height, self.cs.width
        valid = np.ones((h, w), dtype=bool)
        if dr > 0:
            valid[h - dr:, :] = False
        elif dr < 0:
            valid[:(-dr), :] = False
        if dc > 0:
            valid[:, w - dc:] = False
        elif dc < 0:
            valid[:, :(-dc)] = False
        return valid

    def relax_step(self) -> bool:
        """One stabilizing sweep. Returns True if any mass moved (not yet at rest).

        Two-pass, order-independent, strictly mass-conserving and convergent:

        Pass A (per direction): for each loose donor cell whose downhill slope to its
        (dr,dc) neighbor exceeds the threshold, the EXCESS HEIGHT above the repose plane
        is h_excess = drop - run*tan(threshold). The height a donor would shed to that one
        neighbor to JUST reach repose (meeting in the middle) is 0.5*h_excess. We record
        each cell's TOTAL desired outflow height (summed over all its over-repose downhill
        neighbors) and remember the per-neighbor shares.

        Pass B: scale every cell's outflow by transfer_fraction AND by a per-cell cap so a
        donor never sheds more than its own column height (no negative mass) and the total
        is split among its over-repose neighbors by share. Outflow is converted to MASS at
        the DONOR's density (dm = rho_donor * dh) and the SAME mass is added to each
        receiver. Because every gram removed from a donor is added to exactly one receiver,
        Σ mass is invariant to floating-point round-off — no renormalization needed.
        """
        cs = self.cs
        threshold = self.theta_r + self.cohesion_steepening
        tan_thr = np.tan(threshold)

        height = cs.derive_height()
        donor_density = cs.density
        mask = loose_mask(cs)

        # Pass A: per direction, the excess height of the donor above the repose plane of
        # its (dr,dc) neighbor (only when the donor is the HIGHER of the pair and loose).
        # ``excess_h`` is how far the donor sits above where repose would put it relative
        # to THAT neighbor. We use it both as the per-neighbor weight and to size the
        # donor's single allowed outflow so it cannot overshoot the gentlest active pair.
        per_dir_excess = []
        total_excess = np.zeros_like(height)        # Σ excess over active neighbors (weights)
        max_excess = np.zeros_like(height)          # largest single-neighbor excess
        moved = False
        for (dr, dc), run in zip(self.neighbors, self._runs):
            h2 = np.roll(np.roll(height, -dr, axis=0), -dc, axis=1)
            mask2 = np.roll(np.roll(mask, -dr, axis=0), -dc, axis=1)
            valid = self._shift_valid(dr, dc) & mask & mask2
            drop = height - h2
            excess_h = np.where(valid & (drop - run * tan_thr > 0),
                                drop - run * tan_thr, 0.0)
            per_dir_excess.append(excess_h)
            total_excess += excess_h
            max_excess = np.maximum(max_excess, excess_h)
            if excess_h.any():
                moved = True

        if not moved:
            return False

        # Pass B: each donor sheds ONE outflow budget (height), under-relaxed, split among
        # its over-repose neighbors in proportion to their excess. The budget targets the
        # average excess so donor and receivers converge to repose without the lone-peak
        # overshoot that 0.5*excess-per-neighbor caused. Capped at transfer_fraction of the
        # column so mass never goes negative.
        n_active = np.zeros_like(height)
        for ex in per_dir_excess:
            n_active += (ex > 0)
        with np.errstate(invalid="ignore", divide="ignore"):
            # mean excess across active neighbors; halve so donor & neighbors meet midway.
            mean_excess = np.where(n_active > 0, total_excess / n_active, 0.0)
            out_budget_h = self.transfer_fraction * 0.5 * mean_excess
        col_thick = cs.mass_areal / cs.density
        out_budget_h = np.minimum(out_budget_h, self.transfer_fraction * col_thick)

        # Per-neighbor share of the budget, weighted by that neighbor's excess.
        delta_mass = np.zeros_like(cs.mass_areal)
        for (dr, dc), ex in zip(self.neighbors, per_dir_excess):
            with np.errstate(invalid="ignore", divide="ignore"):
                frac = np.where(total_excess > 0, ex / total_excess, 0.0)
            move_h = out_budget_h * frac           # height leaving donor this direction
            dm = move_h * donor_density            # mass leaving donor (donor's density)
            delta_mass -= dm
            delta_mass += np.roll(np.roll(dm, dr, axis=0), dc, axis=1)  # to receiver cell

        cs.mass_areal += delta_mass
        # Belt-and-suspenders against sub-epsilon negatives from float round-off.
        np.maximum(cs.mass_areal, 0.0, out=cs.mass_areal)
        return moved

    def relax_to_rest(self, max_steps: int = 500, *, capture: bool = False,
                      capture_every: int = 1, eps_angle: float = np.deg2rad(0.5)):
        """Iterate relax_step until at rest (or max_steps). The cave-in driver.

        Returns (n_steps, snapshots). If ``capture``, snapshots is a list of derived
        heightmap copies every ``capture_every`` steps (a cave-in TIME SERIES). Reaching
        rest means no mass moved AND every loose slope <= theta_r (within eps_angle).
        """
        snapshots: list[np.ndarray] = []
        steps = 0
        if capture:
            snapshots.append(self.cs.derive_height().copy())
        rest_thresh = self.theta_r + self.cohesion_steepening + eps_angle
        for i in range(max_steps):
            moved = self.relax_step()
            steps += 1
            if capture and (i % capture_every == 0):
                snapshots.append(self.cs.derive_height().copy())
            # Rest = no cell moved this sweep, OR every loose slope is within eps of repose
            # (the conservative outflow budget converges to the repose plane asymptotically,
            # so we stop on the physically-meaningful slope criterion, not bit-exact zero).
            if not moved or self._max_loose_slope() <= rest_thresh:
                break
        if capture and (snapshots and not np.array_equal(snapshots[-1], self.cs.derive_height())):
            snapshots.append(self.cs.derive_height().copy())
        return steps, snapshots
