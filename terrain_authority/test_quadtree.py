"""Characterization tests for terrain_authority.quadtree (spec §4 thesis demo).

The interaction-keyed quadtree manages SPACE: as the rover drives, leaves near it promote
to the finest level (active/fine LOD) while distant regions stay coarse. It is pure spatial
logic — no measurement data: the field side length and a rover (row, col) position are
configuration, not fabricated measurements. We drive every test from a REAL committed scene's
grid (``samples/<name>/metadata.json`` -> 256x256, already a power of two) and a real rover
position inside that grid; the lone-path generator (``quadtree_pad_pow2``) is exercised on the
real DEM base side lengths it exists for (2000 cells @ 5 m, 10000 @ 1 m).

Asserted invariants (all non-trivial, all from the module's documented contract):

  * COVERAGE — the leaf set tiles the field EXACTLY once, no gaps/overlaps
    (``leaves_cover_field`` returns (True, field_size**2)). Holds with and without a rover.
  * POW2 PADDING — ``quadtree_pad_pow2`` returns the smallest power of two >= n (idempotent
    on exact powers; raises on n < 1); ``_is_pow2`` agrees.
  * FINER NEAR THE ROVER — active (min_leaf) leaves cluster near the rover; the nearest leaf
    to the rover is min_leaf-sized; mean box-distance of active leaves << that of coarse leaves.
  * DETERMINISM — same field_size + rover -> byte-identical leaves and nodes.
  * INPUT VALIDATION — non-pow2 field_size, non-pow2 min_leaf, and field_size < min_leaf raise.
  * TOUCHED HISTORY — the tracker's touched set is promote-only (never shrinks on revisit).
"""

from __future__ import annotations

import json
import os

import pytest

from .quadtree import (
    DEFAULT_MIN_LEAF,
    QuadtreeResult,
    QuadtreeTracker,
    _box_chebyshev_distance,
    _is_pow2,
    build_quadtree,
    leaves_cover_field,
    quadtree_pad_pow2,
)

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_REAL_SCENES = ["crater", "crater_boulders", "rolling_hills", "flat_compact"]


def _scene_grid(name: str) -> tuple[int, int, float]:
    """Read a real scene's grid (width, height, cell_m) from its metadata.json."""
    with open(os.path.join(_REPO, "samples", name, "metadata.json")) as fh:
        g = json.load(fh)["grid"]
    return int(g["width"]), int(g["height"]), float(g["cell_m"])


def _real_field_size(name: str = "crater") -> int:
    """The (square) committed-scene side length in cells — a real, power-of-two grid."""
    w, h, _ = _scene_grid(name)
    assert w == h, "committed scenes are square"
    return w


def _box_dist(box, rover) -> float:
    r0, c0, r1, c1 = box
    return _box_chebyshev_distance(r0, c0, r1, c1, rover[0], rover[1])


# ---------------------------------------------------------------------------
# Real scene grids are valid quadtree fields
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("name", _REAL_SCENES)
def test_real_scene_grid_is_power_of_two_square(name):
    """Every committed scene is a square power-of-two grid (a valid build_quadtree field)."""
    w, h, cell = _scene_grid(name)
    assert w == h
    assert _is_pow2(w)
    assert w % DEFAULT_MIN_LEAF == 0
    assert cell > 0


# ---------------------------------------------------------------------------
# pow2 padding helper (the lone real-DEM-side code path)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("n,expected", [
    (2000, 2048),     # 10 km @ 5 m real DEM base
    (10000, 16384),   # 10 km @ 1 m real DEM base
    (256, 256),       # committed scene side (idempotent)
    (1, 1),
    (8, 8),
    (9, 16),
    (1023, 1024),
    (1024, 1024),
])
def test_pad_pow2_smallest_power_of_two_at_or_above(n, expected):
    out = quadtree_pad_pow2(n)
    assert out == expected
    assert _is_pow2(out)
    assert out >= n
    # Smallest such: the previous power of two is strictly below n (for n > 1).
    if out > 1:
        assert out // 2 < n


def test_pad_pow2_is_idempotent_on_exact_powers():
    for p in (1, 2, 4, 8, 256, 1024, 16384):
        assert quadtree_pad_pow2(p) == p


@pytest.mark.parametrize("bad", [0, -1, -256])
def test_pad_pow2_rejects_non_positive(bad):
    with pytest.raises(ValueError):
        quadtree_pad_pow2(bad)


def test_is_pow2_matches_definition():
    assert [_is_pow2(n) for n in (1, 2, 3, 4, 8, 255, 256)] == \
        [True, True, False, True, True, False, True]
    assert not _is_pow2(0)
    assert not _is_pow2(-2)


# ---------------------------------------------------------------------------
# COVERAGE: leaves tile the field exactly once
# ---------------------------------------------------------------------------

def test_leaves_cover_field_with_rover():
    """With a rover present, the leaf set tiles the real 256x256 field exactly once."""
    fs = _real_field_size()
    res = build_quadtree(fs, (fs / 2.0, fs / 2.0), footprint_radius_cells=4.0)
    ok, n = leaves_cover_field(res)
    assert ok
    assert n == fs * fs
    # Total leaf area equals the field area (no gaps/overlaps), corroborating leaves_cover.
    area = sum((r1 - r0) * (c1 - c0) for (r0, c0, r1, c1) in res.leaves)
    assert area == fs * fs


def test_leaves_cover_field_without_rover_single_root_leaf():
    """rover_rc=None (no interaction) -> a single coarse ROOT leaf covering the field."""
    fs = _real_field_size()
    res = build_quadtree(fs, None)
    assert len(res.leaves) == 1
    assert res.active_leaves == []          # nothing promoted (no interaction -> no fine LOD)
    assert len(res.coarse_leaves) == 1
    assert res.leaves[0] == (0, 0, fs, fs)  # the whole field
    ok, n = leaves_cover_field(res)
    assert ok and n == fs * fs


@pytest.mark.parametrize("name", _REAL_SCENES)
def test_coverage_holds_across_real_scenes_and_positions(name):
    """Coverage is exact for every committed scene grid at several real rover positions."""
    fs = _real_field_size(name)
    for rover in [(0.0, 0.0), (fs / 4.0, fs / 3.0), (fs - 1.0, fs - 1.0), (fs / 2.0, fs / 2.0)]:
        res = build_quadtree(fs, rover)
        ok, n = leaves_cover_field(res)
        assert ok, f"{name} @ {rover}: leaves do not tile the field"
        assert n == fs * fs


def test_active_and_coarse_partition_all_leaves():
    """active_leaves and coarse_leaves are a disjoint partition of leaves (all + nothing else)."""
    fs = _real_field_size()
    res = build_quadtree(fs, (100.0, 80.0))
    assert set(res.active_leaves).isdisjoint(res.coarse_leaves)
    assert set(res.active_leaves) | set(res.coarse_leaves) == set(res.leaves)
    assert len(res.active_leaves) + len(res.coarse_leaves) == len(res.leaves)


# ---------------------------------------------------------------------------
# FINER NEAR THE ROVER (the LOD-follows-interaction headline)
# ---------------------------------------------------------------------------

def test_active_leaves_are_min_leaf_and_clustered_near_rover():
    """Active leaves are exactly min_leaf-sized and sit much closer to the rover (in box
    distance) than coarse leaves do — the "more detail where you look" property."""
    fs = _real_field_size()
    rover = (fs / 2.0, fs / 2.0)
    res = build_quadtree(fs, rover, min_leaf=DEFAULT_MIN_LEAF, footprint_radius_cells=4.0)

    assert res.active_leaves, "rover present -> some fine leaves must promote"
    # Every active leaf is exactly min_leaf on a side; every coarse leaf is larger.
    assert all((r1 - r0) == DEFAULT_MIN_LEAF and (c1 - c0) == DEFAULT_MIN_LEAF
               for (r0, c0, r1, c1) in res.active_leaves)
    assert all((r1 - r0) > DEFAULT_MIN_LEAF for (r0, c0, r1, c1) in res.coarse_leaves)

    mean_active = sum(_box_dist(b, rover) for b in res.active_leaves) / len(res.active_leaves)
    mean_coarse = sum(_box_dist(b, rover) for b in res.coarse_leaves) / len(res.coarse_leaves)
    assert mean_active < mean_coarse        # fine cluster is the near-field

    # The single leaf closest to the rover is min_leaf-sized (finest at the focus).
    nearest = min(res.leaves, key=lambda b: _box_dist(b, rover))
    assert (nearest[2] - nearest[0]) == DEFAULT_MIN_LEAF


def test_larger_refine_factor_grows_the_fine_cluster():
    """Increasing refine_factor (wider closeness threshold) yields MORE active leaves —
    the cluster of fine LOD around the rover gets larger (monotone in the knob)."""
    fs = _real_field_size()
    rover = (fs / 2.0, fs / 2.0)
    narrow = build_quadtree(fs, rover, refine_factor=0.5)
    wide = build_quadtree(fs, rover, refine_factor=2.0)
    assert len(wide.active_leaves) >= len(narrow.active_leaves)
    assert len(wide.active_leaves) > 0


def test_footprint_radius_extends_the_fine_cluster():
    """A wheel-disc footprint radius pulls more nodes to distance 0, so the fine cluster
    is at least as large as the point-footprint case (sized to the wheel, spec §4)."""
    fs = _real_field_size()
    rover = (fs / 2.0, fs / 2.0)
    point = build_quadtree(fs, rover, footprint_radius_cells=0.0)
    disc = build_quadtree(fs, rover, footprint_radius_cells=8.0)
    assert len(disc.active_leaves) >= len(point.active_leaves)


# ---------------------------------------------------------------------------
# DETERMINISM
# ---------------------------------------------------------------------------

def test_build_is_deterministic():
    """Same field_size + rover -> byte-identical leaf lists and node lists."""
    fs = _real_field_size()
    rover = (130.0, 99.0)
    a = build_quadtree(fs, rover)
    b = build_quadtree(fs, rover)
    assert a.leaves == b.leaves
    assert a.active_leaves == b.active_leaves
    assert a.coarse_leaves == b.coarse_leaves
    assert a.nodes == b.nodes


def test_leaves_are_sorted_by_position():
    """Leaf lists are sorted by (row0, col0) for byte-stable JSON across runs."""
    fs = _real_field_size()
    res = build_quadtree(fs, (fs / 2.0, fs / 2.0))
    keys = [(r0, c0) for (r0, c0, r1, c1) in res.leaves]
    assert keys == sorted(keys)


# ---------------------------------------------------------------------------
# QuadtreeResult.boxes() — JSON-friendly export
# ---------------------------------------------------------------------------

def test_boxes_returns_plain_int_lists():
    """boxes() emits [[r0,c0,r1,c1],...] of plain Python ints for the metadata sidecar."""
    fs = _real_field_size()
    res = build_quadtree(fs, (fs / 2.0, fs / 2.0))
    for which in ("active", "coarse", "all"):
        boxes = res.boxes(which)
        for box in boxes:
            assert len(box) == 4
            assert all(isinstance(v, int) for v in box)
    assert len(res.boxes("all")) == len(res.leaves)
    assert len(res.boxes("active")) == len(res.active_leaves)
    assert len(res.boxes("coarse")) == len(res.coarse_leaves)


# ---------------------------------------------------------------------------
# INPUT VALIDATION
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("bad_fs", [255, 200, 100, 3])
def test_non_pow2_field_size_raises(bad_fs):
    with pytest.raises(ValueError):
        build_quadtree(bad_fs, (1.0, 1.0))


def test_non_pow2_min_leaf_raises():
    with pytest.raises(ValueError):
        build_quadtree(256, (1.0, 1.0), min_leaf=6)


def test_field_size_smaller_than_min_leaf_raises():
    with pytest.raises(ValueError):
        build_quadtree(8, (1.0, 1.0), min_leaf=16)


def test_field_size_not_multiple_of_min_leaf_raises():
    # 16 is pow2 and 16 is pow2, but a non-multiple combination must still be rejected;
    # use field_size that is pow2 but not a multiple of a (pow2) min_leaf is impossible,
    # so exercise the explicit field_size < min_leaf branch from the same guard.
    with pytest.raises(ValueError):
        build_quadtree(4, (1.0, 1.0), min_leaf=8)


# ---------------------------------------------------------------------------
# _box_chebyshev_distance
# ---------------------------------------------------------------------------

def test_box_chebyshev_distance_zero_inside_box():
    """A point inside the half-open box is at distance 0; outside, it is the L-inf gap."""
    # box covers rows/cols [10, 18); upper edge cells are 17.
    assert _box_chebyshev_distance(10, 10, 18, 18, 12.0, 12.0) == 0.0
    assert _box_chebyshev_distance(10, 10, 18, 18, 17.0, 17.0) == 0.0
    # Two cells left of the box -> distance 2 (10 - 8).
    assert _box_chebyshev_distance(10, 10, 18, 18, 12.0, 8.0) == pytest.approx(2.0)
    # Diagonally below-right -> max of the two gaps.
    assert _box_chebyshev_distance(10, 10, 18, 18, 20.0, 25.0) == pytest.approx(8.0)


# ---------------------------------------------------------------------------
# QuadtreeTracker — touched history is promote-only
# ---------------------------------------------------------------------------

def test_tracker_touched_accumulates_along_a_path():
    """Stepping the tracker along a path accumulates touched min_leaf cells; each step's
    active leaves all end up in the touched set."""
    fs = _real_field_size()
    tr = QuadtreeTracker(field_size=fs, footprint_radius_cells=2.0)
    path = [(40.0, 40.0), (60.0, 40.0), (80.0, 40.0)]
    seen: set = set()
    for p in path:
        res = tr.step(p)
        seen |= set(res.active_leaves)
    touched = set(tr.touched_leaves())
    assert touched == seen
    assert len(touched) > 0


def test_tracker_touched_never_shrinks_on_revisit():
    """Driving back over an earlier position does NOT evict touched cells (promote-only
    cumulative trail — the refined TREAD trail stays recorded behind the rover)."""
    fs = _real_field_size()
    tr = QuadtreeTracker(field_size=fs, footprint_radius_cells=2.0)
    for p in [(40.0, 40.0), (60.0, 40.0), (80.0, 40.0)]:
        tr.step(p)
    after_forward = set(tr.touched_leaves())
    tr.step((40.0, 40.0))                       # revisit the start
    after_revisit = set(tr.touched_leaves())
    assert after_forward <= after_revisit       # superset (never lost a touched cell)


def test_tracker_step_matches_build_quadtree():
    """A tracker step's per-frame result equals a direct build_quadtree with the same args
    (the tracker only adds the persistent touched history on top)."""
    fs = _real_field_size()
    tr = QuadtreeTracker(field_size=fs, footprint_radius_cells=3.0, refine_factor=1.5)
    rover = (100.0, 100.0)
    res = tr.step(rover)
    direct = build_quadtree(fs, rover, min_leaf=DEFAULT_MIN_LEAF,
                            refine_factor=1.5, footprint_radius_cells=3.0)
    assert res.leaves == direct.leaves
    assert res.active_leaves == direct.active_leaves


def test_tracker_touched_boxes_are_plain_int_lists():
    """touched_boxes() emits JSON-friendly [[r0,c0,r1,c1],...] of plain ints, sorted."""
    fs = _real_field_size()
    tr = QuadtreeTracker(field_size=fs, footprint_radius_cells=2.0)
    tr.step((50.0, 50.0))
    boxes = tr.touched_boxes()
    assert boxes, "a step under the rover should touch at least one fine cell"
    for box in boxes:
        assert len(box) == 4 and all(isinstance(v, int) for v in box)
    keys = [(b[0], b[1]) for b in boxes]
    assert keys == sorted(keys)


def test_result_is_quadtreeresult_dataclass():
    """build_quadtree returns a QuadtreeResult carrying min_leaf and field_size."""
    fs = _real_field_size()
    res = build_quadtree(fs, (10.0, 10.0))
    assert isinstance(res, QuadtreeResult)
    assert res.field_size == fs
    assert res.min_leaf == DEFAULT_MIN_LEAF
    # Every node dict carries the documented keys.
    for node in res.nodes:
        assert set(node) == {"level", "row0", "col0", "size", "leaf"}
