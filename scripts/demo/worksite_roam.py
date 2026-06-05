#!/usr/bin/env python3
"""worksite_roam.py — STREAMING roam over the real Haworth site (2026-06-02).

The headline deliverable: a rover ROAMS a >200 m real LOLA-DEM site, flattening a work
pad in one place and building a long straight berm line ELSEWHERE, while the fine
ColumnState window STREAMS along under it (recenter as it moves) and the GLOBAL drum
ledger carries the dug mass from the pad to the berm. The controller is the only stub —
scripted twists+events now via the same .recenter()/.drive()/.flatten()/.dump()/.relax()/
.compact_over() seam an RL policy will drive later (docs/worksite_contract.md).

Pipeline (all multi-window, conservation asserted after every stage):

    PHASE A  flatten a ~40x16 m pad toward a level, mantle-strip into the drum ledger,
             recentering as the dig face advances (G2 worked-tile paging exercised).
    PHASE B  haul: closed-loop drive the corridor pad->berm, laying slip-deepened TREAD,
             recentering each tile (the loaded drum carries the dug mass downrange).
    PHASE C  berm: drive the straight line, dump the ledger in segments (drum -> bulked
             SPOIL), relax each pile to repose (sandpile), compact the crest -> COMPACTED_BERM,
             recentering to advance (G1 mitigated: each dump+relax stays inside one window).

Outputs (under --out): the assembled worked-corridor + virgin baseline (corridor.npz),
the final berm window as a renderable bundle (berm_scene/, Godot rover-cam reads its baked
heightmap), telemetry.json, a build-up GIF, and the exact Godot rover-cam command. The 3x2
composite is built by worksite_composite.py from corridor.npz + the rendered rovercam.png.

Run from the repo root:
    python scripts/demo/worksite_roam.py --out out/worksite_roam
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import numpy as np  # noqa: E402

from terrain_authority import constants as K  # noqa: E402
from terrain_authority.column_state import ColumnState  # noqa: E402
from terrain_authority.sandpile import Sandpile  # noqa: E402
from terrain_authority.worksite import WorkSite  # noqa: E402

BUNDLE = "samples/lunar_dem/haworth_10km_5m"


# ---------------------------------------------------------------------------
# small world<->window helpers (the active window is axis-aligned with the world)
# ---------------------------------------------------------------------------

def active_rc_to_xy(site: WorkSite, rc) -> tuple[float, float]:
    ox, oy = site.window_world_origin
    r, c = rc
    return (ox + c * site.fine_cell_m, oy + r * site.fine_cell_m)


def mask_world_rect(site: WorkSite, x0, x1, y0, y1) -> np.ndarray:
    """Boolean mask over the CURRENT active window of all cells OVERLAPPING the world rectangle
    [x0,x1]x[y0,y1] (metres). floor() on the low edge + ceil() on the high edge give a conservative
    cover (a fringe cell whose centre lies just outside the rectangle is still included). Clipped to
    the window. Exact for cell-aligned rectangles, which is how the roam sizes its pad/berm masks."""
    fine = site.fine
    H, W = fine.height, fine.width
    (ra, ca) = site.active_rc_for_xy((x0, y0))
    (rb, cb) = site.active_rc_for_xy((x1, y1))
    r0, r1 = sorted((ra, rb)); c0, c1 = sorted((ca, cb))
    r0 = max(0, int(math.floor(r0))); r1 = min(H, int(math.ceil(r1)))
    c0 = max(0, int(math.floor(c0))); c1 = min(W, int(math.ceil(c1)))
    m = np.zeros((H, W), dtype=bool)
    if r1 > r0 and c1 > c0:
        m[r0:r1, c0:c1] = True
    return m


def tiles_covering(site: WorkSite, x0, x1, y0, y1) -> set:
    """The base-tile set (tr,tc) covering a world rectangle (for a FIXED corridor frame)."""
    bc0 = int((x0 - site.world_x0) // site.base_cell_m); bc1 = int((x1 - site.world_x0) // site.base_cell_m)
    br0 = int((y0 - site.world_y0) // site.base_cell_m); br1 = int((y1 - site.world_y0) // site.base_cell_m)
    tbc = site.tile_base_cells
    return {(tr, tc)
            for tr in range(br0 // tbc, br1 // tbc + 1)
            for tc in range(bc0 // tbc, bc1 // tbc + 1)
            if 0 <= tr < site._n_tile_rows and 0 <= tc < site._n_tile_cols}


def check(site: WorkSite, label: str, stages: list, baseline: float) -> None:
    res = site.conservation_residual()
    rel = res / baseline if baseline else 0.0
    print(f"  [{label:<14}] grid+ledger={site.total_mass():14.1f} kg  "
          f"ledger={site.inventory_kg:10.1f} kg  resid={res:.3e} (rel {rel:.2e})")
    stages.append({"stage": label, "total_mass": site.total_mass(),
                   "inventory_kg": site.inventory_kg, "residual_kg": res, "rel": rel,
                   "recenters": site.recenters})
    assert rel < 1e-6, f"conservation broke at {label}: rel={rel:.2e}"


# ---------------------------------------------------------------------------
# build-up GIF: each frame is the assembled corridor (FIXED tile frame) hillshaded,
# state-tinted, with the rover marker + a caption. Frames share origin/shape -> clean GIF.
# ---------------------------------------------------------------------------

def corridor_frame(cor, origin, x_s, y_s, caption: str, rover_xy, cell: float):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap, BoundaryNorm
    from io import BytesIO
    from PIL import Image

    ox, oy = origin
    h = cor.derive_height()
    state = cor.state_label
    # transverse crop to the corridor band (+-9 m of y_s) to keep the GIF legible
    r_lo = max(0, int((y_s - 9 - oy) / cell)); r_hi = min(cor.height, int((y_s + 9 - oy) / cell))
    h = h[r_lo:r_hi]; state = state[r_lo:r_hi]
    hs = _hillshade(h, cell)

    fig, ax = plt.subplots(2, 1, figsize=(11, 3.4), dpi=96)
    ext = [ox, ox + cor.width * cell, oy + r_lo * cell, oy + r_hi * cell]
    ax[0].imshow(hs, origin="lower", cmap="gray", extent=ext, aspect="equal", vmin=0, vmax=1)
    dh = h - np.median(h)
    ax[0].imshow(dh, origin="lower", cmap="terrain", extent=ext, aspect="equal",
                 alpha=0.45, vmin=-0.5, vmax=0.5)
    ax[0].plot([rover_xy[0]], [rover_xy[1]], marker="o", ms=7, mfc="#ff3b3b", mec="white", mew=1.0)
    ax[0].set_title(caption, fontsize=10, loc="left")
    ax[0].set_ylabel("y (m)", fontsize=8); ax[0].tick_params(labelsize=7)

    cols = ["#6b6b6b", "#c8a24a", "#7a3b2e", "#d98c3a", "#3a6ea5"]
    cmap = ListedColormap(cols); norm = BoundaryNorm(np.arange(-0.5, 5.5, 1), cmap.N)
    ax[1].imshow(state, origin="lower", cmap=cmap, norm=norm, extent=ext, aspect="equal")
    ax[1].plot([rover_xy[0]], [rover_xy[1]], marker="o", ms=7, mfc="white", mec="black", mew=1.0)
    ax[1].set_xlabel("x along corridor (m)", fontsize=8); ax[1].set_ylabel("y (m)", fontsize=8)
    ax[1].tick_params(labelsize=7)
    fig.tight_layout()
    buf = BytesIO(); fig.savefig(buf, format="png"); plt.close(fig)
    buf.seek(0)
    return Image.open(buf).convert("RGB")


def _hillshade(h: np.ndarray, cell: float, az_deg=315.0, alt_deg=35.0) -> np.ndarray:
    gy, gx = np.gradient(h, cell)
    slope = np.arctan(np.hypot(gx, gy))
    aspect = np.arctan2(-gx, gy)
    az = np.deg2rad(az_deg); alt = np.deg2rad(alt_deg)
    sh = (np.sin(alt) * np.cos(slope) +
          np.cos(alt) * np.sin(slope) * np.cos(az - aspect))
    return np.clip(sh, 0, 1)


# ---------------------------------------------------------------------------
# main roam
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(description="Streaming WorkSite roam: flatten pad + build berm line")
    ap.add_argument("--out", default="out/worksite_roam")
    ap.add_argument("--base-rc", type=int, nargs=2, default=[1101, 1101], metavar=("BR", "BC"),
                    help="coarse Haworth base cell where the roam starts (well inside the 2000^2 base)")
    ap.add_argument("--fine-cell-m", type=float, default=0.05)
    ap.add_argument("--tile-base-cells", type=int, default=2, help="base cells per fine tile (10 m @ 5 m base)")
    ap.add_argument("--pad-len-m", type=float, default=40.0)
    ap.add_argument("--pad-halfwidth-m", type=float, default=8.0)
    ap.add_argument("--haul-len-m", type=float, default=30.0)
    ap.add_argument("--berm-len-m", type=float, default=40.0)
    ap.add_argument("--berm-halfwidth-m", type=float, default=1.25)
    ap.add_argument("--dig-depth-m", type=float, default=0.30, help="requested pad cut (clamps to ~Z_T strip, G8)")
    ap.add_argument("--berm-segments", type=int, default=5)
    ap.add_argument("--relax-steps", type=int, default=160)
    ap.add_argument("--raw-terraces", action="store_true",
                    help="disable the G7 bilinear-datum fix (show the raw piecewise-constant DEM terraces)")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    page_dir = os.path.join(args.out, "tiles_page")

    # G7 fix ON by default for the roam (conservation-neutral; removes the fake 5 m terrace cliffs
    # that otherwise saturate drive slip + pollute the repose readout + stair-step the render).
    site = WorkSite.from_haworth_bundle(BUNDLE, fine_cell_m=args.fine_cell_m,
                                        tile_base_cells=args.tile_base_cells, page_dir=page_dir,
                                        smooth_datum=not args.raw_terraces)
    print(f"datum: {'RAW terraced DEM (G7 on display)' if args.raw_terraces else 'bilinear-smoothed (G7 fix; conservation-neutral)'}")
    cell = site.fine_cell_m

    # --- world geometry of the roam (a straight corridor along +x) ----------------------
    x_s = site.world_x0 + args.base_rc[1] * site.base_cell_m
    y_s = site.world_y0 + args.base_rc[0] * site.base_cell_m
    pad_x0, pad_x1 = x_s, x_s + args.pad_len_m
    haul_x1 = pad_x1 + args.haul_len_m
    berm_x0, berm_x1 = haul_x1, haul_x1 + args.berm_len_m
    corridor_tiles = tiles_covering(site, x_s - 6, berm_x1 + 6, y_s - 12, y_s + 12)
    print(f"ROAM corridor: x in [{0:.0f},{berm_x1 - x_s:.0f}] m (pad {args.pad_len_m:.0f} + haul "
          f"{args.haul_len_m:.0f} + berm {args.berm_len_m:.0f}), y={y_s:.1f}; "
          f"{len(corridor_tiles)} base-tiles @ {site.tile_base_cells * site.base_cell_m:.0f} m")
    print(f"window = {2 * 1 + 1}x{2 * 1 + 1} tiles = "
          f"{(2 + 1) * site.tile_base_cells * site.base_cell_m:.0f} m; fine cell {cell:.2f} m")

    stages: list = []
    frames: list = []                                    # PIL images for the simple overhead buildup.gif
    rover_xy = (pad_x0 + 2.0, y_s)
    site.recenter(rover_xy, radius_tiles=1)
    baseline = site._baseline_virgin_kg  # grows as tiles enter; rel residual is /this
    print("Mass-conservation ledger (active grid + worked store + global drum):")
    check(site, "open", stages, max(baseline, 1.0))

    # virgin corridor baseline (for the cross-section + GIF context), captured before any work
    virgin_cor, vorigin = site.assemble_region(tiles=corridor_tiles)
    virgin_h = virgin_cor.derive_height()
    yc_row = int((y_s - vorigin[1]) / cell)
    # representative pad level from the virgin DEM under the pad
    pc0 = max(0, int((pad_x0 - vorigin[0]) / cell)); pc1 = min(virgin_cor.width, int((pad_x1 - vorigin[0]) / cell))
    pad_band = virgin_h[max(0, yc_row - 80):yc_row + 80, pc0:pc1]
    target_level = float(pad_band.mean() - args.dig_depth_m)
    print(f"pad virgin height ~{pad_band.mean():.2f} m; flatten target_level={target_level:.2f} m "
          f"(requested -{args.dig_depth_m:.2f} m; mantle-strip clamps to ~{K.Z_T * 100:.0f} cm at datum, G8)")

    # --- per-step PROGRESS capture: each call snapshots the corridor (frame npz), the active window
    #     as a LOCAL render bundle + a follow-cam pose (manifest), and a simple overhead GIF frame.
    #     worksite_progress.py replays the manifest: Godot follow-cam + full 3x2 composite per frame.
    frames_dir = os.path.join(args.out, "frames")
    os.makedirs(frames_dir, exist_ok=True)
    manifest: list = []
    map_center = (0.5 * (x_s + berm_x1), y_s)            # fixed map crop centre (stable across the GIF)
    pad_moved = 0.0
    berm_repose: list = []

    def capture(phase_label: str, tag: str, rxy) -> None:
        idx = len(manifest)
        cor, (cox, coy) = site.assemble_region(tiles=corridor_tiles)
        np.savez_compressed(
            os.path.join(frames_dir, f"frame_{idx:02d}.npz"),
            height=cor.derive_height().astype(np.float32), state=cor.state_label.astype(np.uint8),
            density=cor.density.astype(np.float32), virgin_height=virgin_h.astype(np.float32),
            origin=np.array((cox, coy), float), cell=np.float64(cell), yc_row=np.int64(yc_row),
            x_s=np.float64(x_s), y_s=np.float64(y_s),
            pad_x=np.array([pad_x0 - x_s, pad_x1 - x_s]), haul_x=np.array([pad_x1 - x_s, berm_x0 - x_s]),
            berm_x=np.array([berm_x0 - x_s, berm_x1 - x_s]))
        fine = site.fine; hmin = float(fine.derive_height().min())
        lc = ColumnState(fine.width, fine.height, fine.cell_m,
                         mass_areal=np.array(fine.mass_areal), density=np.array(fine.density),
                         state_label=np.array(fine.state_label), disturbance=np.array(fine.disturbance),
                         datum=np.array(fine.datum) - hmin)
        bdir = os.path.join(frames_dir, f"bundle_{idx:02d}")
        site.save_cs_bundle(lc, bdir, (0.0, 0.0), scene_name=f"frame{idx}")
        rr = site.active_rc_for_xy(rxy)
        rrow = int(np.clip(round(rr[0]), 0, fine.height - 1)); rcol = int(np.clip(round(rr[1]), 0, fine.width - 1))
        rx, rz = rcol * cell, rrow * cell
        surf = float(lc.derive_height()[rrow, rcol])
        pose = [round(v, 2) for v in (rx - 5.0, surf + 4.0, rz - 5.0, rx + 2.0, surf - 0.5, rz + 2.0)]
        manifest.append({
            "idx": idx, "frame_label": phase_label, "tag": tag,
            "bundle": os.path.abspath(bdir), "rover_rc": [rrow, rcol], "pose": pose,
            "sun_elev": 35, "sun_azim": 215, "exposure": 2.2,
            "rover_xy": [rxy[0] - x_s, rxy[1] - y_s],
            "mass_point": [tag, float(pad_moved), float(site.inventory_kg)],
            "pad_moved_kg": float(pad_moved), "peak_inventory_kg": float(site.peak_inventory_kg),
            "recenters": int(site.recenters), "worst_rel_so_far": float(max((s["rel"] for s in stages), default=0.0)),
            "berm_repose": [float(v) for v in berm_repose], "worked_world_bbox": list(site.visited_world_bbox()),
        })
        frames.append(corridor_frame(cor, (cox, coy), x_s, y_s, phase_label, rxy, cell))

    capture("virgin Haworth corridor (pre-roam)", "virgin", rover_xy)

    # ---- PHASE A: flatten the pad, advancing the dig face (multi-window) ----------------
    print("\nPHASE A  flatten pad (mantle-strip -> drum, recentering as the face advances)")
    n_steps = max(2, int(round(args.pad_len_m / (site.tile_base_cells * site.base_cell_m))))
    for i in range(n_steps + 1):
        fx = pad_x0 + 2.0 + i * (args.pad_len_m - 4.0) / n_steps
        rover_xy = (fx, y_s)
        site.recenter(rover_xy, radius_tiles=1)
        # flatten the pad cells visible in THIS window toward the shared absolute level
        m = mask_world_rect(site, pad_x0, pad_x1, y_s - args.pad_halfwidth_m, y_s + args.pad_halfwidth_m)
        if m.any():
            pad_moved += site.flatten(m, target_level)
        if i in (0, n_steps // 2, n_steps):
            capture(f"PHASE A  flatten pad  (drum {site.inventory_kg/1000:.1f} t)", f"pad{i}", rover_xy)
    # achieved drop, measured against the virgin corridor over the pad
    final_pad, forigin = site.assemble_region(tiles=corridor_tiles)
    fph = final_pad.derive_height()
    pad_mask = np.zeros_like(fph, dtype=bool)
    pr0 = max(0, yc_row - int(args.pad_halfwidth_m / cell)); pr1 = yc_row + int(args.pad_halfwidth_m / cell)
    pad_mask[pr0:pr1, pc0:pc1] = True
    cut = pad_mask & (fph < virgin_h - 1e-4)
    achieved = float((virgin_h[cut] - fph[cut]).mean()) if cut.any() else 0.0
    datum_desc = "raw terraced" if args.raw_terraces else "bilinear-smoothed (G7 fix)"
    print(f"  pad: moved {pad_moved:.0f} kg into the drum; achieved mean drop {achieved:.3f} m over "
          f"{int(cut.sum())} cells (only ~{K.Z_T * 100:.0f} cm of loose mantle is removable above the "
          f"firm datum, G8; the pad floor follows the {datum_desc} DEM datum — the real regional grade, "
          f"not an absolute plane)")
    check(site, "flatten_pad", stages, baseline)
    # local regional grade under the berm (to read the repose-on-slope correctly below)
    bx0i = max(0, int((berm_x0 - vorigin[0]) / cell)); bx1i = min(virgin_h.shape[1], int((berm_x1 - vorigin[0]) / cell))
    seg_rise = virgin_h[yc_row, bx0i:bx1i]
    berm_grade = float(np.rad2deg(np.arctan(abs(np.polyfit(np.arange(seg_rise.size) * cell, seg_rise, 1)[0]))))
    print(f"  (berm site regional grade ~{berm_grade:.0f} deg; a pile at repose on it shows a downhill "
          f"face up to ~{np.rad2deg(K.THETA_R) + berm_grade:.0f} deg)")

    # ---- PHASE B: haul the loaded drum down the corridor (multi-window TREAD) -----------
    print("\nPHASE B  haul pad->berm (closed-loop drive, slip-deepened TREAD ruts)")
    haul_telemetry = []
    hx = pad_x1
    while hx < berm_x0 - 1.0:
        rover_xy = (hx, y_s)
        site.recenter(rover_xy, radius_tiles=1)
        start_rc = site.active_rc_for_xy((hx, y_s))
        res = site.drive([(0.6, 0.0)] * 18, start_rc=start_rc, start_yaw=0.0, dt=1.0,
                         payload_kg=min(site.inventory_kg, 30.0))
        haul_telemetry.append({"x": hx - x_s, "commanded_m": res["commanded_dist_m"],
                               "achieved_m": res["achieved_dist_m"], "entrapped": res["any_entrapped"]})
        end_xy = active_rc_to_xy(site, res["final_rc"])
        hx = max(end_xy[0], hx + 5.0)  # advance even if slip stalls the integrator
    rover_xy = (berm_x0, y_s)
    tot_cmd = sum(h["commanded_m"] for h in haul_telemetry); tot_ach = sum(h["achieved_m"] for h in haul_telemetry)
    print(f"  haul: {len(haul_telemetry)} window bursts; commanded {tot_cmd:.1f} m, achieved {tot_ach:.1f} m "
          f"(slip divergence is the closed loop; per-window because the fine window streams under the rover)")
    capture("PHASE B  haul (loaded drum downrange)", "haul", rover_xy)
    check(site, "haul", stages, baseline)

    # ---- PHASE C: build the straight berm line in segments (multi-window) ---------------
    print("\nPHASE C  build berm line (dump -> relax -> compact per segment, recentering to advance)")
    seg_len = args.berm_len_m / args.berm_segments
    for s in range(args.berm_segments):
        seg_x0 = berm_x0 + s * seg_len
        seg_x1 = seg_x0 + seg_len
        seg_cx = 0.5 * (seg_x0 + seg_x1)
        rover_xy = (seg_cx, y_s)
        site.recenter(rover_xy, radius_tiles=1)
        seg_mask = mask_world_rect(site, seg_x0, seg_x1,
                                   y_s - args.berm_halfwidth_m, y_s + args.berm_halfwidth_m)
        remaining_segs = args.berm_segments - s
        want = site.inventory_kg / remaining_segs
        placed = site.dump(seg_mask, kg=want)
        capture(f"PHASE C  berm seg {s + 1}/{args.berm_segments} dumped ({placed/1000:.1f} t SPOIL)",
                f"s{s + 1}d", rover_xy)
        # relax this pile toward repose (truncated at max_steps — the pile partially settles; window-wide
        # rest can't fire). Measure the BERM-LOCAL max loose slope to show how far it settled.
        steps, _ = site.relax(max_steps=args.relax_steps, capture=False)
        bslope = _berm_local_slope(site, seg_x0, seg_x1, y_s, args.berm_halfwidth_m + 2.0)
        berm_repose.append(np.rad2deg(bslope))
        # compact the crest -> COMPACTED_BERM along the segment centre line
        crest_r = int(round(site.active_rc_for_xy((seg_cx, y_s))[0]))
        c_lo = int(round(site.active_rc_for_xy((seg_x0, y_s))[1]))
        c_hi = int(round(site.active_rc_for_xy((seg_x1, y_s))[1]))
        poses = [((crest_r, c), 0.0) for c in range(c_lo, c_hi, 4)]
        if poses:
            site.compact_over(poses)
        capture(f"PHASE C  berm seg {s + 1}/{args.berm_segments} relaxed+compacted "
                f"(local {berm_repose[-1]:.0f} deg)", f"s{s + 1}c", rover_xy)
        converged = steps < args.relax_steps
        repose_on_grade = np.rad2deg(K.THETA_R) + berm_grade
        print(f"  seg {s + 1}/{args.berm_segments}: dumped {placed:.0f} kg, relaxed {steps}/{args.relax_steps} "
              f"sweeps ({'converged to rest' if converged else 'truncated — NOT yet at rest'}); berm-local max "
              f"loose slope {berm_repose[-1]:.1f} deg (flat repose {np.rad2deg(K.THETA_R):.0f} deg; full repose on "
              f"the {berm_grade:.0f} deg grade would be ~{repose_on_grade:.0f} deg -> pile partially settled, between the two)")
        check(site, f"berm_seg{s + 1}", stages, baseline)

    # ---- conservation verdict ----------------------------------------------------------
    worst = max(s["rel"] for s in stages)
    ok = worst < 1e-6
    print(f"\nCONSERVATION across the whole roam: worst relative residual = {worst:.2e}  ->  "
          f"{'PASS' if ok else 'FAIL'} (<1e-6) over {site.recenters} recenters")
    cycles = math.ceil(site.peak_inventory_kg / K.DRUM_PAYLOAD_MAX_KG)
    print(f"DRUM PAYLOAD: peak ledger {site.peak_inventory_kg:.0f} kg = ~{cycles} cycles of the "
          f"{K.DRUM_PAYLOAD_MAX_KG:.0f} kg/cycle drum (over_payload={site.over_payload}; the dig/haul/dump "
          f"is modeled as one continuous transfer — per-cycle batching is a scale-up item, not enforced)")

    # ---- save the final berm window as the Godot rover-cam bundle -----------------------
    # Two bundles: the GLOBAL one (record) and a LOCAL-frame one for rendering. Godot's terrain
    # mesh is built in float32, so at Haworth's ~47 km / ~100 km world offset the vertices lose
    # precision and the terrain renders black (documented). The render bundle therefore uses a
    # LOCAL frame: world origin (0,0) and a subtracted height datum, so all coords are small.
    berm_dir = os.path.join(args.out, "berm_scene")
    site.save_fine_bundle(berm_dir, scene_name="worksite_berm")
    fine = site.fine
    hmin = float(fine.derive_height().min())
    local_cs = ColumnState(fine.width, fine.height, fine.cell_m,
                           mass_areal=np.array(fine.mass_areal), density=np.array(fine.density),
                           state_label=np.array(fine.state_label), disturbance=np.array(fine.disturbance),
                           datum=np.array(fine.datum) - hmin)
    berm_local_dir = os.path.join(args.out, "berm_scene_local")
    site.save_cs_bundle(local_cs, berm_local_dir, (0.0, 0.0), scene_name="worksite_berm_local",
                        extra={"render_height_datum_m": hmin})
    # ---- assemble + persist the corridor for the composite ------------------------------
    final_cor, corigin = site.assemble_region(tiles=corridor_tiles)
    np.savez_compressed(
        os.path.join(args.out, "corridor.npz"),
        height=final_cor.derive_height().astype(np.float32),
        state=final_cor.state_label.astype(np.uint8),
        density=final_cor.density.astype(np.float32),
        virgin_height=virgin_h.astype(np.float32),
        origin=np.array(corigin, dtype=np.float64),
        cell=np.float64(cell),
        yc_row=np.int64(yc_row),
        x_s=np.float64(x_s), y_s=np.float64(y_s),
        pad_x=np.array([pad_x0 - x_s, pad_x1 - x_s]),
        haul_x=np.array([pad_x1 - x_s, berm_x0 - x_s]),
        berm_x=np.array([berm_x0 - x_s, berm_x1 - x_s]),
        worked_bbox=np.array(site.visited_world_bbox(), dtype=np.float64),
        berm_window_origin=np.array(site.window_world_origin, dtype=np.float64),
    )
    telem = {"stages": stages, "worst_rel_residual": worst, "conservation_pass": ok,
             "recenters": site.recenters, "peak_inventory_kg": site.peak_inventory_kg,
             "drum_cycles": cycles, "over_payload": site.over_payload,
             "pad_moved_kg": pad_moved, "pad_achieved_drop_m": achieved,
             "berm_local_repose_deg": berm_repose, "haul": haul_telemetry,
             "params": vars(args), "worked_world_bbox": list(site.visited_world_bbox())}
    with open(os.path.join(args.out, "telemetry.json"), "w") as fh:
        json.dump(telem, fh, indent=2)

    # ---- build-up GIF (simple overhead) + progress manifest (full 3x2 per step) --------
    gif_path = os.path.join(args.out, "buildup.gif")
    frames[0].save(gif_path, save_all=True, append_images=frames[1:], duration=900, loop=0)
    print(f"\nwrote: {gif_path} ({len(frames)} frames)")
    with open(os.path.join(frames_dir, "manifest.json"), "w") as fh:
        json.dump({"map_center": list(map_center), "map_zoom_km": 2.5, "map_bundle": BUNDLE,
                   "x_s": x_s, "y_s": y_s, "raw_terraces": bool(args.raw_terraces),
                   "worst_rel_residual": worst, "conservation_pass": ok,
                   "out": os.path.abspath(args.out), "frames": manifest}, fh, indent=2)
    print(f"wrote: {frames_dir}/manifest.json ({len(manifest)} per-step frames) — for the progress GIF run:\n"
          f"  python scripts/demo/worksite_progress.py --out {args.out}")

    # ---- the Godot rover-cam command (renders the baked berm heightmap, LOCAL frame) ---
    # Pose is in the LOCAL render frame (x=col*cell, z=row*cell, y=height-hmin). A raised
    # inspection sun (elev 35) lights the flat regolith — the grazing polar default leaves
    # horizontal lunar surfaces near-black (real Hapke photometry), so this is an INSPECTION
    # light for legibility, NOT PSR-realistic (captioned).
    cam_row = int(np.clip(round(site.active_rc_for_xy((berm_x1 - seg_len * 0.5, y_s))[0]), 0, fine.height - 1))
    cam_col = int(np.clip(round(site.active_rc_for_xy((berm_x1 - seg_len * 0.5, y_s))[1]), 0, fine.width - 1))
    rx, rz = cam_col * cell, cam_row * cell
    surf = float(local_cs.derive_height()[cam_row, cam_col])
    cam = (rx - 5.0, surf + 4.0, rz - 5.0)
    tgt = (rx + 2.0, surf - 0.5, rz + 2.0)
    out_abs = os.path.abspath(os.path.join(args.out, "rovercam.png"))
    scene_abs = os.path.abspath(berm_local_dir)
    cmd = (f"cd godot_sidecar && ./render_layers.sh -- "
           f"--scene {scene_abs} --layers terrain,clasts,rover "
           f"--rover-rc {cam_row},{cam_col} --size 960x720 "
           f"--pose {cam[0]:.2f},{cam[1]:.2f},{cam[2]:.2f},{tgt[0]:.2f},{tgt[1]:.2f},{tgt[2]:.2f} "
           f"--sun-elev 35 --sun-azim 215 --exposure 2.2 --out {out_abs}")
    with open(os.path.join(args.out, "godot_cmd.txt"), "w") as fh:
        fh.write(cmd + "\n")
    print("\nGodot rover-cam command (also saved to godot_cmd.txt; --out is absolute):")
    print("  " + cmd)
    print(f"\nThen:  python scripts/demo/worksite_composite.py --out {args.out}")
    print(f"wrote: {berm_dir}/  corridor.npz  telemetry.json")


def _berm_local_slope(site: WorkSite, x0: float, x1: float, yc: float, halfy: float) -> float:
    """Max loose-cell downhill slope [rad] in a berm-local sub-box of the active window."""
    fine = site.fine
    r0 = int(site.active_rc_for_xy((x0, yc - halfy))[0]); r1 = int(site.active_rc_for_xy((x1, yc + halfy))[0])
    c0 = int(site.active_rc_for_xy((x0, yc))[1]); c1 = int(site.active_rc_for_xy((x1, yc))[1])
    r0, r1 = sorted((r0, r1)); c0, c1 = sorted((c0, c1))
    r0 = max(0, r0); c0 = max(0, c0); r1 = min(fine.height, r1 + 1); c1 = min(fine.width, c1 + 1)
    sub = ColumnState(c1 - c0, r1 - r0, fine.cell_m,
                      mass_areal=np.array(fine.mass_areal[r0:r1, c0:c1]),
                      density=np.array(fine.density[r0:r1, c0:c1]),
                      state_label=np.array(fine.state_label[r0:r1, c0:c1]),
                      disturbance=np.array(fine.disturbance[r0:r1, c0:c1]),
                      datum=np.array(fine.datum[r0:r1, c0:c1]))
    return Sandpile(sub)._max_loose_slope()


if __name__ == "__main__":
    main()
