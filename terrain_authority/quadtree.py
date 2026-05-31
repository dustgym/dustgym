"""Interaction-keyed quadtree over the WIDTH x HEIGHT field (spec §4 thesis demo).

This is the headline architecture-thesis structure: **the tree manages SPACE, not
physics** (spec §4). As the rover drives, leaves near it PROMOTE to the finest level
(active / fine LOD) while distant regions stay COARSE. The uniform fine grid inside an
active leaf is the terramechanics solve substrate; the tree decides *where* that fine
solve is spent. LOD and space-management are *keyed to interaction* — the rover footprint
position (and, optionally, the disturbance field) is the only driver.

Pure NumPy, deterministic, dependency-free. No raster I/O here (that is io_fields, the
frozen seam); this module only computes node boxes from a rover position and emits them as
``[r0, c0, r1, c1]`` half-open cell boxes for the metadata sidecar and the viz consumer.

--------------------------------------------------------------------------------
PROMOTION RULE (precise, deterministic)
--------------------------------------------------------------------------------
The field side length must be a power of two (256 here) so the tree bottoms out cleanly at
``min_leaf`` (also a power of two, default 8). Starting from the ROOT (one node covering
the whole field), a node is **subdivided** into its four quadrants iff BOTH hold:

  1. its size is still above ``min_leaf`` (we never subdivide past the finest level), AND
  2. the rover is "close relative to the node's own scale": the Chebyshev (box) distance
     from the rover footprint to the node's box is **less than** ``refine_factor * size``
     cells, where ``size`` is the node's current side length.

Because the closeness threshold scales WITH the node size, the rule is *distance-graded*:
near the rover even small nodes pass the test and keep splitting down to ``min_leaf``
(fine/active), while far from the rover a node fails the test early and stops as a large
coarse leaf. This is the standard quadtree-LOD behaviour ("more detail where you look").

A leaf at exactly ``min_leaf`` size is FINE/ACTIVE. Any leaf larger than ``min_leaf`` is
COARSE. The full set of leaves (fine + coarse) tiles the field with no gaps or overlaps,
by construction (each subdivision partitions a node into four disjoint quadrants).

The footprint "distance" optionally uses a rover footprint RADIUS (the wheel contact disc,
spec §4 / rover.py), so a node touching the disc is at distance 0 and the cluster of fine
leaves is sized to the wheel, not to a single point.

--------------------------------------------------------------------------------
EVICTION / TOUCHED-HISTORY (documented choice)
--------------------------------------------------------------------------------
Two leaf sets are exposed and they answer the two halves of "keyed to interaction":

  * ``active_leaves``  — fine (min_leaf) leaves under the CURRENT rover footprint. These
    PROMOTE as the rover approaches and are EVICTED (coarsen back) as it leaves: the live
    hot-region working set, bounded in count (see ``max_active_leaves`` invariant in
    tests). This is the LOD-follows-interaction signal.
  * ``touched_leaves`` — every min_leaf-resolution cell box the rover footprint has EVER
    overlapped across the drive so far (active-history / "we have worked here"). This is
    persistent: it does NOT evict, so the refined TREAD trail stays recorded behind the
    rover. It mirrors the VIRGIN->TREAD segmentation the rover lays down.

So: the active set follows the rover (promote+evict); the touched set is the cumulative
trail (promote-only). The viz draws active leaves hot and the rest of the subdivision as
the LOD context, exactly the "space-management + LOD keyed to interaction" headline.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# Default finest leaf side [cells]. 8 cells @ 0.02 m/cell = 16 cm ~ the rover contact patch
# scale (spec §4 1-3 cm anchor / wheel ~10-22 cm), so the active cluster is wheel-sized.
DEFAULT_MIN_LEAF = 8

# Closeness threshold factor (see PROMOTION RULE). A node subdivides when the rover is
# within ``refine_factor * node_size`` cells of the node box. 1.0 = "within one node-width".
DEFAULT_REFINE_FACTOR = 1.0


def _is_pow2(n: int) -> bool:
    return n > 0 and (n & (n - 1)) == 0


# ---------------------------------------------------------------------------
# ADDITIVE helper (Lane C / L0 contract §5): pad a base side length to the next
# power of two so the 10 km @ 5 m base (2000 cells) or @ 1 m (10000 cells) can be
# fed to ``build_quadtree`` (which REQUIRES a power-of-two field_size, :134).
# This does NOT change build_quadtree or any existing function — it only computes
# the padded side a caller would build the tree over (the live grid stays a window
# inside that padded extent; cells past the real data are coarse leaves never
# refined). Pure / dependency-free.
# ---------------------------------------------------------------------------

def quadtree_pad_pow2(n: int) -> int:
    """Smallest power of two ``>= n`` (e.g. 2000 -> 2048, 10000 -> 16384, 256 -> 256).

    The interaction-keyed quadtree (``build_quadtree``) requires ``field_size`` to be a
    power of two so the tree bottoms out cleanly at ``min_leaf`` (:134-139). A real DEM base
    is sized to the data (2000 cells for 10 km @ 5 m, 10000 for @ 1 m), which is NOT a power
    of two, so a caller pads the tree extent up to ``quadtree_pad_pow2(side)`` and treats the
    live data as a window inside it (the padding cells stay coarse ROOT-side leaves that are
    never promoted because the rover never reaches them).

    Exact powers of two are returned unchanged (idempotent). Raises ``ValueError`` for
    ``n < 1`` (a non-positive side has no power-of-two cover).
    """
    if not isinstance(n, (int, np.integer)) or n < 1:
        raise ValueError(f"quadtree_pad_pow2: n must be a positive integer, got {n!r}")
    n = int(n)
    if _is_pow2(n):
        return n
    # Next power of two strictly above n: 1 << ceil(log2(n)). bit_length gives floor(log2)+1
    # for any n>0, which for a non-pow2 n is exactly ceil(log2(n)).
    return 1 << (n - 1).bit_length()


@dataclass
class QuadtreeResult:
    """Per-rover-position quadtree snapshot (all boxes are [r0, c0, r1, c1] half-open)."""

    leaves: list[tuple[int, int, int, int]]          # ALL leaves; tile the field, disjoint
    active_leaves: list[tuple[int, int, int, int]]   # fine (min_leaf) leaves under the rover
    coarse_leaves: list[tuple[int, int, int, int]]   # leaves larger than min_leaf
    nodes: list[dict]                                # every node (incl. internal) as a dict
    min_leaf: int
    field_size: int

    def boxes(self, which: str = "active") -> list[list[int]]:
        """Return [[r0,c0,r1,c1],...] for 'active' | 'coarse' | 'all' (JSON-friendly lists)."""
        src = {"active": self.active_leaves, "coarse": self.coarse_leaves,
               "all": self.leaves}[which]
        return [[int(r0), int(c0), int(r1), int(c1)] for (r0, c0, r1, c1) in src]


def _box_chebyshev_distance(r0: int, c0: int, r1: int, c1: int,
                            rr: float, rc: float) -> float:
    """Chebyshev (box / L-inf) distance in cells from point (rr,rc) to box [r0,c0,r1,c1).

    0 if the point is inside the (half-open) box. The box upper edge is r1-1 / c1-1.
    """
    dr = max(r0 - rr, rr - (r1 - 1), 0.0)
    dc = max(c0 - rc, rc - (c1 - 1), 0.0)
    return max(dr, dc)


def build_quadtree(field_size: int, rover_rc: tuple[float, float] | None, *,
                   min_leaf: int = DEFAULT_MIN_LEAF,
                   refine_factor: float = DEFAULT_REFINE_FACTOR,
                   footprint_radius_cells: float = 0.0) -> QuadtreeResult:
    """Build the interaction-keyed quadtree for one rover position. Pure / deterministic.

    Parameters
    ----------
    field_size : int
        Side length of the (square) field in cells. MUST be a power of two and a multiple
        of ``min_leaf`` so the tree bottoms out exactly at the finest level.
    rover_rc : (row, col) or None
        Rover footprint CENTER in cell coordinates. If None (e.g. the pristine pre-drive
        frame), no node is "close", so the whole field stays a single coarse ROOT leaf
        (nothing to refine yet) — itself a faithful "no interaction -> no fine LOD" state.
    min_leaf : int
        Finest leaf side [cells] (power of two).
    refine_factor : float
        Closeness factor in the promotion rule (subdivide while box-distance <
        refine_factor * node_size). Larger -> a wider fine cluster around the rover.
    footprint_radius_cells : float
        Optional wheel contact-disc radius [cells]; subtracted from the box distance so a
        node overlapping the disc is at distance 0 (the fine cluster is sized to the wheel).

    Returns
    -------
    QuadtreeResult with leaves / active_leaves / coarse_leaves / nodes.
    """
    if not _is_pow2(field_size):
        raise ValueError(f"field_size {field_size} must be a power of two")
    if not _is_pow2(min_leaf):
        raise ValueError(f"min_leaf {min_leaf} must be a power of two")
    if field_size % min_leaf != 0 or field_size < min_leaf:
        raise ValueError(f"field_size {field_size} must be a multiple of min_leaf {min_leaf}")

    leaves: list[tuple[int, int, int, int]] = []
    active: list[tuple[int, int, int, int]] = []
    coarse: list[tuple[int, int, int, int]] = []
    nodes: list[dict] = []

    if rover_rc is None:
        rr = rc = None
    else:
        rr, rc = float(rover_rc[0]), float(rover_rc[1])

    # Iterative DFS over a stack of (row0, col0, size, level). Deterministic order:
    # quadrants are always pushed/popped in the same sequence.
    stack: list[tuple[int, int, int, int]] = [(0, 0, field_size, 0)]
    while stack:
        r0, c0, size, level = stack.pop()
        r1, c1 = r0 + size, c0 + size

        # Decide whether to subdivide (PROMOTION RULE).
        subdivide = False
        if size > min_leaf and rr is not None:
            dist = _box_chebyshev_distance(r0, c0, r1, c1, rr, rc) - footprint_radius_cells
            if dist < refine_factor * size:
                subdivide = True

        nodes.append({"level": level, "row0": r0, "col0": c0, "size": size,
                      "leaf": not subdivide})

        if subdivide:
            half = size // 2
            # Push the 4 quadrants (reverse order so they pop in NW, NE, SW, SE order).
            stack.append((r0 + half, c0 + half, half, level + 1))  # SE
            stack.append((r0 + half, c0, half, level + 1))         # SW
            stack.append((r0, c0 + half, half, level + 1))         # NE
            stack.append((r0, c0, half, level + 1))                # NW
        else:
            box = (r0, c0, r1, c1)
            leaves.append(box)
            if size == min_leaf:
                active.append(box)
            else:
                coarse.append(box)

    # Deterministic sort of leaf lists by (row0, col0) so two runs produce byte-identical
    # JSON regardless of stack traversal nuances.
    key = lambda b: (b[0], b[1])
    leaves.sort(key=key)
    active.sort(key=key)
    coarse.sort(key=key)
    nodes.sort(key=lambda n: (n["level"], n["row0"], n["col0"]))

    return QuadtreeResult(leaves=leaves, active_leaves=active, coarse_leaves=coarse,
                          nodes=nodes, min_leaf=min_leaf, field_size=field_size)


# ---------------------------------------------------------------------------
# Driven-rover convenience: a persistent tracker that accumulates the touched
# history while the active set follows the rover (the two-set scheme above).
# ---------------------------------------------------------------------------

@dataclass
class QuadtreeTracker:
    """Stateful driver for a driven-rover series (one step per rover position).

    Holds the persistent ``touched`` history (the cumulative min_leaf cell boxes the rover
    footprint has overlapped). ``step`` rebuilds the per-frame subdivision (the active set
    that promotes/evicts with the rover) and folds the current active leaves into history.
    Deterministic: same sequence of positions -> same state.
    """

    field_size: int
    min_leaf: int = DEFAULT_MIN_LEAF
    refine_factor: float = DEFAULT_REFINE_FACTOR
    footprint_radius_cells: float = 0.0
    _touched: set[tuple[int, int, int, int]] = field(default_factory=set)

    def step(self, rover_rc: tuple[float, float] | None) -> QuadtreeResult:
        """Advance to ``rover_rc``; return this frame's QuadtreeResult and update history."""
        res = build_quadtree(self.field_size, rover_rc, min_leaf=self.min_leaf,
                             refine_factor=self.refine_factor,
                             footprint_radius_cells=self.footprint_radius_cells)
        for box in res.active_leaves:
            self._touched.add(box)
        return res

    def touched_leaves(self) -> list[tuple[int, int, int, int]]:
        """All min_leaf cell boxes the rover footprint has ever activated (sorted)."""
        return sorted(self._touched, key=lambda b: (b[0], b[1]))

    def touched_boxes(self) -> list[list[int]]:
        """touched_leaves as JSON-friendly [[r0,c0,r1,c1],...]."""
        return [[int(a), int(b), int(c), int(d)] for (a, b, c, d) in self.touched_leaves()]


def leaves_cover_field(res: QuadtreeResult) -> tuple[bool, int]:
    """Verify the leaf set tiles the field exactly once (no gaps, no overlaps).

    Returns (ok, n_cells_covered_exactly_once). Builds a coverage-count raster and checks it
    is all ones over the field.
    """
    cover = np.zeros((res.field_size, res.field_size), dtype=np.int32)
    for (r0, c0, r1, c1) in res.leaves:
        cover[r0:r1, c0:c1] += 1
    exactly_once = int(np.count_nonzero(cover == 1))
    ok = bool(np.all(cover == 1))
    return ok, exactly_once
