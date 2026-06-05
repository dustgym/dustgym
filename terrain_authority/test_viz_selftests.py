"""Cover the tiles_mosaic / dem_overlay self-test harnesses by invoking them.

Each module ships its own acceptance harness — ``tiles_mosaic._selftest()`` (the Lane-C corridor-LOD
checks) and ``dem_overlay._self_test()`` (the conservation-critical procgen-overlay checks). They run
the REAL deterministic algorithms (coord-seeded fbm, coarsen/refine round-trips, crater carving) and
return 0 only when every internal invariant holds. Calling them here both exercises the harness code
and re-asserts those invariants (mass conservation to the float64 floor, byte-identical regeneration,
detail-actually-added). No synthetic measurement data — the harnesses generate their own
deterministic fixtures from the modules' own math.

Run from the repo root:
    PYTHONPATH=. <venv>/bin/python -m pytest terrain_authority/test_viz_selftests.py -q
"""
from __future__ import annotations

from terrain_authority import dem_overlay, tiles_mosaic


def test_tiles_mosaic_selftest_passes():
    # Lane-C: ensure_fine materializes around a moving pose, evict bounds the resident set, regen is
    # byte-identical, coarsen(overlay(refine(base)))==base, quadtree pad-to-pow2.
    assert tiles_mosaic._selftest() == 0


def test_dem_overlay_self_test_passes():
    # procgen overlay: frozen feature_fn signature, determinism, coarsen(overlay(base))==base
    # (carried fields bit-exact), and craters actually carve detail.
    assert dem_overlay._self_test() == 0
