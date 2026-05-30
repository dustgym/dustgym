"""Minimal single wheel-pass rut carving (spec §6 TREAD transition, §9).

GEOMETRY/STATE-ACCURATE, NOT FORCE-ACCURATE. Per spec §9 ("Robot design context"): IPEx
uses counter-rotating bucket drums (RASSOR heritage; 2021-ASCEND-Mass-Inference-RASSOR.pdf)
that cancel horizontal excavation reaction forces because in 1/6 g there is too little
weight-on-wheels to anchor digging — "because forces are engineered small, the Tier-2
analytical layer need not be force-accurate to high precision to be useful — it must be
geometry- and state-accurate." asce-es-2024-isru-pilot-excavator-wheel-testing.pdf
characterizes the IPEx wheel on simulant; we reproduce the OBSERVABLE outcome of a pass
(a compacted, slightly sunken rut with bumped disturbance), not the contact mechanics.

A pass does the spec §6 VIRGIN/TREAD -> TREAD transition:
    density UP (compaction), height DOWN slightly (MASS PRESERVED — denser column is
    thinner), state_label -> TREAD, disturbance bumped. Multi-pass "paving" emerges by
    re-applying: each pass meets denser soil and compacts a little less (spec §6).

We do NOT model slip-sinkage / runaway entrapment here (spec §6 two sinkage modes) — that
is the path-dependent failure a full Chrono::Vehicle + slip-sinkage solver would surface.
"""

from __future__ import annotations

import numpy as np

from . import constants as K
from .column_state import ColumnState, StateLabel


def _wheel_mask(cs: ColumnState, center_rc: tuple[float, float], half_width_cells: float) -> np.ndarray:
    """Disc footprint of the contact patch around (row,col)."""
    r0, c0 = center_rc
    rows = np.arange(cs.height)[:, None] - r0
    cols = np.arange(cs.width)[None, :] - c0
    return (rows ** 2 + cols ** 2) <= half_width_cells ** 2


def wheel_pass(cs: ColumnState, path_rc: list[tuple[int, int]], *,
               wheel_width_m: float = 0.18, compaction: float = 0.12) -> ColumnState:
    """Carve a single rut along ``path_rc`` (list of (row,col)), in-place. MASS PRESERVED.

    wheel_width_m: contact-patch width (~10-20 cm, spec §4 resolution anchor; IPEx wheel,
        asce-es-2024). compaction: fractional density increase under the wheel per pass.

    Mechanism (spec §6): density *= (1+compaction) up to RHO_DEEP. Because mass_areal is
    untouched and height = mass/density, the column thins -> the rut sinks. Disturbance is
    bumped (drives fresh-cut albedo/roughness downstream, INTERFACE.md §4), and the cell
    is relabelled TREAD (or COMPACTED_BERM if it was SPOIL — driving over spoil compacts
    it into a real structure, spec §6).
    """
    half_w = max(0.5, 0.5 * wheel_width_m / cs.cell_m)  # half-width in cells

    touched = np.zeros((cs.height, cs.width), dtype=bool)
    for (r, c) in path_rc:
        m = _wheel_mask(cs, (r, c), half_w)
        touched |= m

    if not touched.any():
        return cs

    # Compaction: density up, capped at the deep/compacted ceiling. MASS UNCHANGED ->
    # height drops automatically via derive_height().
    cs.density[touched] = np.minimum(cs.density[touched] * (1.0 + compaction), K.RHO_DEEP)

    # State + disturbance. SPOIL -> COMPACTED_BERM (deliberate structure step, spec §6);
    # everything else -> TREAD.
    was_spoil = touched & (cs.state_label == StateLabel.SPOIL)
    cs.state_label[touched & ~was_spoil] = StateLabel.TREAD
    cs.state_label[was_spoil] = StateLabel.COMPACTED_BERM
    cs.disturbance[touched] = np.clip(cs.disturbance[touched] + 0.35, 0.0, 1.0)
    return cs


def straight_path(r0: int, c0: int, r1: int, c1: int, step_cells: int = 1) -> list[tuple[int, int]]:
    """Sample a straight (row,col) path between two cells (Bresenham-ish, dense)."""
    n = max(abs(r1 - r0), abs(c1 - c0)) // max(1, step_cells) + 1
    rs = np.linspace(r0, r1, n).round().astype(int)
    cs = np.linspace(c0, c1, n).round().astype(int)
    return list(zip(rs.tolist(), cs.tolist()))
