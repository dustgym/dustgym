"""Spiral-departure trajectory generator (pure stdlib + math, host-runnable).

    python scripts/demo/spiral_path.py        # runs the self-test + prints the spiral

Lane DEMO-TRAJ (docs/demo_spiral_contract.md §1). Authors the per-frame
``rover_rc`` waypoints + heading for the "larger, longer" departure demo: an
AprilTag lander is FIXED at the scene CENTER (§0 binding decision) and the rover
drives an Archimedean spiral *outward* from it. The monotonically increasing range
is the mechanism that drives the wanted ``out_of_range`` failure mode (§0, §4) --
the tag eventually becomes too far to resolve.

Two outputs, both pure functions of their args (deterministic; no engine, no
numpy -- stdlib ``math`` only so this is trivially host-testable and matches the
"pure python (host)" line of the contract):

  * ``spiral_rc``     -- the [row,col] waypoint list (the repo's rover_rc channel,
                         [row,col]; see scenes.py:109/503/799).
  * ``look_at_yaw``   -- the per-frame heading that keeps the rover's +forward
                         (front stereo) pointed at the lander, in the SAME yaw
                         convention as the Godot rig so the tag stays in frustum.

YAW CONVENTION (load-bearing -- READ from source, not invented):
  godot_sidecar/sidecar.gd:357-378  ``_heading_yaw``:
      dx := b.y - a.y   # col delta -> +X
      dz := b.x - a.x   # row delta -> +Z
      return atan2(-dz, dx)   # point rover forward (+X) along travel
  godot_sidecar/capture_seq.gd:83-94 uses the identical (dx=col, dz=row,
      yaw=atan2(-dz, dx)) form, citing sidecar._heading_yaw as the convention.
  terrain_authority/scenes.py:473-480 ``_heading_from_segment`` is a DIFFERENT
      convention -- atan2(drow, dcol), the INTERFACE.md §5.2 travel-heading used
      to orient wheel cleats, NOT the Godot rover yaw. The contract §1 names
      ``atan2(-dz, dx)`` (the Godot rover yaw), so ``look_at_yaw`` matches the
      sidecar/capture_seq form, NOT _heading_from_segment. (The function the
      contract calls ``scenes._heading_yaw`` does not exist by that name; the
      live yaw convention lives in the .gd rig, which is what renders the demo.)

  To "look at" the center, the heading is computed on the vector FROM the rover
  TO the center (the direction +forward must point), i.e. the same atan2(-dz, dx)
  with (dx, dz) = (center_col - rover_col, center_row - rover_row).
"""

from __future__ import annotations

import math


def spiral_rc(center_rc, n_frames, *, turns, r0_cells, r_growth_cells, cell_m):
    """Archimedean spiral rover_rc waypoints about ``center_rc`` (the lander cell).

    r(θ) = r0_cells + r_growth_cells * θ/2π,  θ in [0, 2π*turns]   (cells)

    Sampled at ``n_frames`` evenly spaced θ. Because r(θ) is affine and strictly
    increasing in θ (for r_growth_cells > 0), the range to center is monotonically
    increasing along the path -- the rover progressively departs, which drives the
    ``out_of_range`` failure (docs/demo_spiral_contract.md §0, §4).

    Args:
        center_rc:        (row, col) lander cell, the spiral center.
        n_frames:         number of waypoints to emit (>= 1).
        turns:            number of full revolutions (θ spans [0, 2π*turns]).
        r0_cells:         starting radius at θ=0, in cells.
        r_growth_cells:   radius gained per full turn, in cells (the Archimedean
                          pitch: r grows r_growth_cells over each 2π of θ).
        cell_m:           cell size [m]; accepted for unit completeness so callers
                          carry one source of truth (scenes.CELL_M). The returned
                          rc are in CELL units (the rover_rc channel is cells), so
                          cell_m does not scale the output -- it is validated only.

    Returns:
        list[(row, col)] of length n_frames, floats (sub-cell; the demo authors a
        float pose, never the ~20 mm-quantized integer trajectory channel -- §0.4
        channel hygiene). center_rc + (Δrow, Δcol) with the spiral laid in the
        rc-plane: angle measured so +X is +col and +Z is +row (the rig's axis map,
        sidecar.gd:374-375), keeping it consistent with look_at_yaw.
    """
    if n_frames < 1:
        raise ValueError(f"n_frames must be >= 1, got {n_frames}")
    if cell_m <= 0.0:
        raise ValueError(f"cell_m must be > 0, got {cell_m}")
    if r_growth_cells < 0.0:
        # Negative growth would make range DECREASE -> violates the monotone-
        # departure contract; reject loudly rather than emit a misleading path.
        raise ValueError(f"r_growth_cells must be >= 0 (monotone departure), got {r_growth_cells}")

    crow, ccol = float(center_rc[0]), float(center_rc[1])
    theta_max = 2.0 * math.pi * turns

    out: list[tuple[float, float]] = []
    for i in range(n_frames):
        # Even θ sampling; guard the single-frame case (no division by zero).
        frac = 0.0 if n_frames == 1 else i / (n_frames - 1)
        theta = frac * theta_max
        r = r0_cells + r_growth_cells * theta / (2.0 * math.pi)
        # Axis map (sidecar.gd:374-375): +X <- +col, +Z <- +row. Place the spiral
        # in that plane so the angle θ is measured the same way look_at_yaw reads
        # it: dcol = r*cos θ along +X, drow = r*sin θ along +Z.
        dcol = r * math.cos(theta)
        drow = r * math.sin(theta)
        out.append((crow + drow, ccol + dcol))
    return out


def look_at_yaw(rover_rc, center_rc):
    """Heading [rad] that points the rover's +forward (front stereo) at the lander.

    SAME convention as godot_sidecar/sidecar.gd:357-378 ``_heading_yaw`` and
    capture_seq.gd:83-94: dx = col delta -> +X, dz = row delta -> +Z, and
    yaw = atan2(-dz, dx) aims the rover's local +X (front-stereo forward) along the
    given direction vector. Here the direction is FROM the rover TO the center, so
    the front stereo frames the fixed lander each step (docs/demo_spiral_contract.md
    §1, §0.2 fixed-center lander).

    Args:
        rover_rc:   (row, col) current rover cell.
        center_rc:  (row, col) lander cell to look at.

    Returns:
        yaw [rad] in (-π, π]. If the rover is exactly on the center (degenerate,
        zero range) returns 0.0, mirroring the rig's ``< 1e-6`` no-travel fallback.
    """
    dx = float(center_rc[1] - rover_rc[1])   # col delta -> +X (sidecar.gd:374)
    dz = float(center_rc[0] - rover_rc[0])   # row delta -> +Z (sidecar.gd:375)
    if abs(dx) < 1e-6 and abs(dz) < 1e-6:
        return 0.0
    return math.atan2(-dz, dx)               # point +forward at center (sidecar.gd:378)


# ---------------------------------------------------------------------------
# Self-test (host; no engine). Exercises the three contract guarantees:
#   1. range to center is strictly NON-decreasing along the path;
#   2. look_at_yaw points back at center (a +X step from center -> yaw == π);
#   3. determinism (same args -> byte-identical list).
# Then prints a small ASCII plot + coordinate dump of the spiral.
# ---------------------------------------------------------------------------
def _range_cells(rc, center_rc):
    return math.hypot(rc[0] - center_rc[0], rc[1] - center_rc[1])


def _self_test():
    center = (128.0, 128.0)
    n = 24
    kw = dict(turns=2.5, r0_cells=4.0, r_growth_cells=18.0, cell_m=0.02)
    path = spiral_rc(center, n, **kw)

    assert len(path) == n, f"expected {n} waypoints, got {len(path)}"

    # (1) Monotonically increasing (non-decreasing) range to center.
    ranges = [_range_cells(p, center) for p in path]
    bad = [(i, ranges[i - 1], ranges[i]) for i in range(1, n) if ranges[i] < ranges[i - 1] - 1e-9]
    assert not bad, f"range to center decreased at frames: {bad}"
    assert ranges[-1] > ranges[0], f"path did not depart: r0={ranges[0]:.3f} rN={ranges[-1]:.3f}"

    # (2) look_at_yaw points the rover's +forward back at the center. A rover one
    #     cell along +X from the center (rover_col > center_col, same row) must face
    #     -X (yaw = π) to look back at the lander.
    rover_plus_x = (center[0], center[1] + 1.0)
    yaw_x = look_at_yaw(rover_plus_x, center)
    assert abs(abs(yaw_x) - math.pi) < 1e-9, f"+X-of-center yaw expected ±π, got {yaw_x}"

    #     A rover one cell along +Z (+row) from center must face -Z. In the rig map
    #     +Z is +row and yaw = atan2(-dz, dx): direction to center is dz=-1, dx=0 ->
    #     atan2(1, 0) = +π/2.
    rover_plus_z = (center[0] + 1.0, center[1])
    yaw_z = look_at_yaw(rover_plus_z, center)
    assert abs(yaw_z - math.pi / 2.0) < 1e-9, f"+Z-of-center yaw expected +π/2, got {yaw_z}"

    #     And the yaw must actually frame the center: stepping the rover along its
    #     +forward (cos yaw -> +X/col, -sin yaw -> +Z/row, per sidecar.gd:354) from
    #     ANY waypoint must REDUCE the range to center (it is looking the right way).
    for p in path[1:]:
        yaw = look_at_yaw(p, center)
        fwd_col = math.cos(yaw)          # +X component (col)
        fwd_row = -math.sin(yaw)         # +Z component (row)  [Basis(UP,yaw): +X->(cos,0,-sin)]
        stepped = (p[0] + 0.5 * fwd_row, p[1] + 0.5 * fwd_col)
        assert _range_cells(stepped, center) < _range_cells(p, center) + 1e-9, (
            f"forward step did not approach center from {p} (yaw={yaw:.4f})")

    # (3) Determinism: identical args -> identical list (object-equal floats).
    again = spiral_rc(center, n, **kw)
    assert again == path, "spiral_rc is not deterministic for identical args"

    # Single-frame edge case must not divide by zero and must sit at r0.
    one = spiral_rc(center, 1, **kw)
    assert len(one) == 1 and abs(_range_cells(one[0], center) - kw["r0_cells"]) < 1e-9

    print("self-test PASS: monotone-departure + look_at_yaw(atan2(-dz,dx)) + determinism")
    print(f"  n_frames={n}  turns={kw['turns']}  r0={kw['r0_cells']}c  "
          f"growth={kw['r_growth_cells']}c/turn  cell_m={kw['cell_m']}")
    print(f"  range[cells]: first={ranges[0]:.2f}  last={ranges[-1]:.2f}  "
          f"(= {ranges[0]*kw['cell_m']:.3f} m -> {ranges[-1]*kw['cell_m']:.3f} m)")

    # --- coordinate dump (every 4th frame) ---
    print("\n  frame   row      col     range_c   range_m   yaw_deg")
    for i in range(0, n, 4):
        p = path[i]
        rng = ranges[i]
        yaw = look_at_yaw(p, center)
        print(f"  {i:>4}  {p[0]:7.2f} {p[1]:7.2f}  {rng:7.2f}  {rng*kw['cell_m']:7.3f}  "
              f"{math.degrees(yaw):7.1f}")

    # --- ASCII plot of the spiral (row down, col right; center marked '+') ---
    print("\n  spiral (row=down, col=right, '+'=lander/center, '0'=start, '*'=path):")
    rows = [p[0] for p in path]
    cols = [p[1] for p in path]
    rmin, rmax = min(rows + [center[0]]), max(rows + [center[0]])
    cmin, cmax = min(cols + [center[1]]), max(cols + [center[1]])
    W, H = 41, 21
    grid = [[" "] * W for _ in range(H)]

    def _cell(r, c):
        gy = 0 if rmax == rmin else int(round((r - rmin) / (rmax - rmin) * (H - 1)))
        gx = 0 if cmax == cmin else int(round((c - cmin) / (cmax - cmin) * (W - 1)))
        return max(0, min(H - 1, gy)), max(0, min(W - 1, gx))

    for i, p in enumerate(path):
        gy, gx = _cell(p[0], p[1])
        grid[gy][gx] = "0" if i == 0 else "*"
    cy, cx = _cell(center[0], center[1])
    grid[cy][cx] = "+"
    for line in grid:
        print("  " + "".join(line))


if __name__ == "__main__":
    _self_test()
