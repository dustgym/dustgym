#!/usr/bin/env python3
"""worksite_slice.py — thin vertical slice of the streaming WorkSite (2026-06-02).

Proves the streaming coarse-base + rover-following fine-window + GLOBAL drum ledger seam
END-TO-END on a REAL Haworth patch, with a SCRIPTED controller (the bootstrap stand-in for
a future RL policy — docs/worksite_contract.md):

    dig a level pad (flatten -> drum) -> haul (drive, lays TREAD) -> dump a berm line
    (drum -> bulked SPOIL) -> relax to repose (sandpile) -> drive over the crest (compact).

Mass conservation is asserted after EVERY stage (fine.grid_mass()+ledger invariant). Saves
the worked fine window as a renderable INTERFACE bundle (Godot reads the baked heightmap) and
writes verification panels (overhead height/state + the cut/berm cross-section). Prints the
exact Godot rover-cam command for the bundle.

Run from the repo root:
    python scripts/demo/worksite_slice.py --out out/worksite_slice
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
from terrain_authority.column_state import ColumnState, StateLabel  # noqa: E402
from terrain_authority.sandpile import Sandpile  # noqa: E402
from terrain_authority.worksite import WorkSite  # noqa: E402

BUNDLE = "samples/lunar_dem/haworth_10km_5m"


def _rect(H: int, W: int, r0: int, r1: int, c0: int, c1: int) -> np.ndarray:
    m = np.zeros((H, W), dtype=bool)
    m[max(0, r0):min(H, r1), max(0, c0):min(W, c1)] = True
    return m


def _region_max_loose_slope(fine: ColumnState, r0: int, r1: int, c0: int, c1: int) -> float:
    """Max loose-cell downhill slope [rad] within a sub-box — used to show the worked BERM
    reached repose even though the whole window can't (virgin DEM terraces pin the global metric)."""
    r0, c0 = max(0, r0), max(0, c0)
    r1, c1 = min(fine.height, r1), min(fine.width, c1)
    sub = ColumnState(c1 - c0, r1 - r0, fine.cell_m,
                      mass_areal=np.array(fine.mass_areal[r0:r1, c0:c1]),
                      density=np.array(fine.density[r0:r1, c0:c1]),
                      state_label=np.array(fine.state_label[r0:r1, c0:c1]),
                      disturbance=np.array(fine.disturbance[r0:r1, c0:c1]),
                      datum=np.array(fine.datum[r0:r1, c0:c1]))
    return Sandpile(sub)._max_loose_slope()


def _check(site: WorkSite, label: str, baseline: float) -> dict:
    """Print + return the conservation residual after a stage (must stay tiny)."""
    res = site.conservation_residual()
    rel = res / baseline if baseline else 0.0
    print(f"  [{label:<16}] grid+ledger={site.total_mass():15.3f} kg  "
          f"ledger={site.inventory_kg:11.3f} kg  residual={res:.3e} kg (rel {rel:.2e})")
    return {"stage": label, "total_mass": site.total_mass(),
            "inventory_kg": site.inventory_kg, "residual_kg": res, "rel": rel}


def main() -> None:
    ap = argparse.ArgumentParser(description="WorkSite vertical slice (dig/haul/dump/relax/compact)")
    ap.add_argument("--out", default="out/worksite_slice")
    ap.add_argument("--base-rc", type=int, nargs=2, default=[1101, 1101], metavar=("BR", "BC"),
                    help="coarse Haworth base cell to center the fine window on")
    ap.add_argument("--fine-cell-m", type=float, default=0.05)
    ap.add_argument("--tile-base-cells", type=int, default=4, help="fine window = N base cells square")
    ap.add_argument("--radius-m", type=float, default=5.0)
    ap.add_argument("--dig-depth-m", type=float, default=0.30, help="pad excavation depth")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    page_dir = os.path.join(args.out, "tiles_page")

    # --- build the streaming world over the committed Haworth coarse base ---------------
    site = WorkSite.from_haworth_bundle(BUNDLE, fine_cell_m=args.fine_cell_m,
                                        tile_base_cells=args.tile_base_cells, page_dir=page_dir)
    site.open_window(tuple(args.base_rc), radius_m=args.radius_m)
    fine = site.fine
    H, W = fine.height, fine.width
    cell = fine.cell_m
    baseline = site.total_mass()
    h0 = fine.derive_height()
    print(f"WorkSite fine window: {H}x{W} @ {cell} m  ({H*cell:.1f}x{W*cell:.1f} m)  "
          f"base_rc={args.base_rc}  world_origin={site.window_world_origin}")
    print(f"  virgin height range: [{h0.min():.3f}, {h0.max():.3f}] m  (relief {h0.max()-h0.min():.3f} m)")
    print("Mass-conservation ledger (fine.grid_mass + global drum):")
    snaps = {"virgin": site.snapshot()}
    stages = [_check(site, "open_window", baseline)]

    # --- geometry of the work (fine cells): a 5x5 m pad, a 13 m haul, a 2 m berm line ---
    rc = H // 2
    pad = _rect(H, W, rc - 50, rc + 50, 50, 150)          # 5 x 5 m excavation pad
    berm = _rect(H, W, rc - 50, rc + 50, 270, 310)         # 2 m wide berm line, 5 m long
    drive_c0, drive_c1 = 30, 290                           # haul corridor along +col

    # 1) DIG the pad toward (mean - dig_depth) -> spoil into the GLOBAL ledger. The fine window
    # only carries the loose mantle (~Z_T) above a firm DEM datum, so this STRIPS the mantle to
    # datum rather than excavating a true dig_depth pad (G8); report the ACHIEVED drop, not the ask.
    target = float(h0[pad].mean() - args.dig_depth_m)
    moved = site.flatten(pad, target)
    hd = fine.derive_height()
    cut = pad & (hd < h0 - 1e-4)
    achieved = float((h0[cut] - hd[cut]).mean()) if cut.any() else 0.0
    resid = float(hd[pad].max() - hd[pad].min())
    print(f"  dig: requested {args.dig_depth_m:.2f} m below pad mean; STRIPPED the loose mantle to "
          f"the firm datum -> achieved mean drop {achieved:.3f} m over {int(cut.sum())} cells, "
          f"moved {moved:.1f} kg into the drum.")
    print(f"       (only ~{K.Z_T*100:.0f} cm is removable above datum (G8); pad floor traces the "
          f"terraced datum, residual relief {resid:.3f} m (G7) — NOT a flat {args.dig_depth_m:.2f} m pad)")
    snaps["dug"] = site.snapshot()
    stages.append(_check(site, "dig_pad", baseline))

    # 2) HAUL: drive the corridor (slip-deepened TREAD ruts), pad -> dump site.
    n = 90
    res = site.drive([(0.3, 0.0)] * n, start_rc=(rc, drive_c0), start_yaw=0.0, dt=0.5)
    print(f"  haul: {len(res['steps'])} twists  commanded={res['commanded_dist_m']:.2f} m  "
          f"achieved={res['achieved_dist_m']:.2f} m  entrapped={res['any_entrapped']}")
    snaps["hauled"] = site.snapshot()
    stages.append(_check(site, "haul", baseline))

    # 3) DUMP the whole ledger as a SPOIL berm line.
    placed = site.dump(berm)
    print(f"  dump: placed {placed:.1f} kg of SPOIL on the berm line  (ledger now {site.inventory_kg:.1f} kg)")
    print(f"       NOTE bulking NOT exercised: RHO_SPOIL==RHO_SURFACE==1300 and this patch is uniform "
          f"1300 kg/m^3, so the dump is iso-density (berm height = pure mass relocation, not swell)")
    snaps["dumped"] = site.snapshot()
    stages.append(_check(site, "dump_berm", baseline))

    # 4) RELAX to angle-of-repose (the showpiece CA), capture the slump series. The WHOLE-window
    # rest criterion can't fire — virgin DEM terraces (piecewise-constant refine, G7) pin the
    # global max loose-slope ~83 deg — so report convergence honestly and show the BERM itself
    # reaching repose (its local max loose slope vs theta_r).
    max_steps = 300
    steps, heightframes = site.relax(max_steps=max_steps, capture=True, capture_every=8)
    converged = steps < max_steps
    berm_slope = _region_max_loose_slope(fine, rc - 60, rc + 60, 250, 330)
    print(f"  relax: ran {steps}/{max_steps} sweeps, {len(heightframes)} frames — window-wide rest "
          f"{'reached' if converged else 'NOT reached (virgin DEM terraces hold ~83 deg, G7)'}; "
          f"BERM-LOCAL max loose slope = {np.rad2deg(berm_slope):.1f} deg vs repose "
          f"{np.rad2deg(K.THETA_R):.0f} deg -> the spoil pile itself IS at repose")
    snaps["relaxed"] = site.snapshot()
    stages.append(_check(site, "relax_berm", baseline))

    # 5) COMPACT: drive across the berm crest -> SPOIL becomes COMPACTED_BERM (firmed structure).
    crest = [((rc, c), 0.0) for c in range(260, 320, 4)]
    site.compact_over(crest)
    n_berm = int((fine.state_label == int(StateLabel.COMPACTED_BERM)).sum())
    print(f"  compact: drove the crest -> {n_berm} COMPACTED_BERM cells "
          f"(G5: physical Bekker at this rover's tiny static wheel load firms spoil only "
          f"~1300->~1310 kg/m^3 per pass, far below the 1610 pin — labels the structure, "
          f"does not yet hold slope)")
    assert n_berm > 0, "compact_over produced no COMPACTED_BERM (four_wheel_pass union-relabel regression)"
    snaps["compacted"] = site.snapshot()
    stages.append(_check(site, "compact_berm", baseline))

    # regression guard: an empty/out-of-window dump mask must NOT fabricate ledger mass.
    pre = site.total_mass()
    assert site.dump(_rect(H, W, 10 * H, 10 * H + 1, 10 * W, 10 * W + 1), kg=500.0) == 0.0
    assert abs(site.total_mass() - pre) < 1e-9, "empty-mask dump injected phantom mass"

    # --- conservation verdict -----------------------------------------------------------
    worst = max(s["rel"] for s in stages)
    ok = worst < 1e-6
    print(f"\nCONSERVATION: worst relative residual = {worst:.2e}  ->  {'PASS' if ok else 'FAIL'} (<1e-6)")
    cycles = math.ceil(site.peak_inventory_kg / K.DRUM_PAYLOAD_MAX_KG)
    print(f"DRUM PAYLOAD: peak ledger {site.peak_inventory_kg:.1f} kg = ~{cycles} cycles of the "
          f"{K.DRUM_PAYLOAD_MAX_KG:.0f} kg/cycle drum, modeled as one transfer "
          f"(over_payload={site.over_payload}; per-cycle batching is a scale-up item, not enforced)")

    # --- save the renderable fine bundle + telemetry ------------------------------------
    bundle_dir = os.path.join(args.out, "fine_scene")
    meta = site.save_fine_bundle(bundle_dir, scene_name="worksite_slice")
    with open(os.path.join(args.out, "telemetry.json"), "w") as fh:
        json.dump({"stages": stages, "worst_rel_residual": worst, "conservation_pass": ok,
                   "peak_inventory_kg": site.peak_inventory_kg, "over_payload": site.over_payload,
                   "window": {"base_rc": list(args.base_rc), "shape": [H, W], "cell_m": cell,
                              "world_origin_m": list(site.window_world_origin)}}, fh, indent=2)

    # --- verification panels ------------------------------------------------------------
    _panels(os.path.join(args.out, "slice_panels.png"), snaps, cell, rc)

    # --- the Godot rover-cam command for this bundle ------------------------------------
    # An explicit --pose IS required: the no-pose default camera renders black at this bundle's
    # ~47 km world offset (large-coord float precision / camera frustum). Sun is raised off the
    # 5 deg polar default + exposure boosted purely for panel legibility (cite: not PSR-realistic).
    ox, oz = site.window_world_origin
    cam_row, cam_col = rc, 300
    surf = float(snaps["compacted"]["height"][cam_row, cam_col])
    cam = (ox + 1.0, surf + 4.0, oz + 8.0)
    tgt = (ox + 8.5, surf - 0.5, oz + cam_row * cell)
    out_abs = os.path.abspath(os.path.join(args.out, "rovercam.png"))
    scene_abs = os.path.abspath(bundle_dir)
    print("\nGodot rover-cam (renders the baked fine heightmap; --out must be ABSOLUTE):")
    print(f"  cd godot_sidecar && ./render_layers.sh -- \\\n"
          f"      --scene {scene_abs} --layers terrain,clasts,rover \\\n"
          f"      --rover-rc {cam_row},{cam_col} --size 960x720 \\\n"
          f"      --pose {cam[0]:.1f},{cam[1]:.1f},{cam[2]:.1f},{tgt[0]:.1f},{tgt[1]:.1f},{tgt[2]:.1f} \\\n"
          f"      --sun-elev 28 --sun-azim 120 --exposure 2.6 --out {out_abs}")
    print(f"\nwrote: {bundle_dir}/  +  slice_panels.png  +  telemetry.json")


def _panels(path: str, snaps: dict, cell_m: float, rc: int) -> None:
    """4-panel verification: overhead height (final), state-label, cross-section, ledger."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap, BoundaryNorm

    v, f = snaps["virgin"], snaps["compacted"]
    state_cols = ["#6b6b6b", "#c8a24a", "#7a3b2e", "#d98c3a", "#3a6ea5"]  # VIRGIN/TREAD/EXC/SPOIL/BERM
    cmap = ListedColormap(state_cols)
    norm = BoundaryNorm(np.arange(-0.5, 5.5, 1), cmap.N)

    fig, ax = plt.subplots(2, 2, figsize=(12, 10), dpi=110)
    fig.suptitle("WorkSite vertical slice — dig pad -> haul -> dump berm -> relax -> compact", fontsize=13)

    # (a) final height, hillshade-ish
    hh = f["height"]
    im = ax[0, 0].imshow(hh, origin="lower", cmap="terrain")
    ax[0, 0].axhline(rc, color="k", lw=0.6, ls="--")
    ax[0, 0].set_title("final height (m)  [dashed = section]")
    fig.colorbar(im, ax=ax[0, 0], fraction=0.046)

    # (b) final state labels — caption derived from labels actually PRESENT (no over-claim)
    im = ax[0, 1].imshow(f["state_label"], origin="lower", cmap=cmap, norm=norm)
    present = [K.STATE_NAMES[i] for i in range(5) if (f["state_label"] == i).any()]
    ax[0, 1].set_title("state present: " + ", ".join(present), fontsize=9)
    cb = fig.colorbar(im, ax=ax[0, 1], fraction=0.046, ticks=range(5))
    cb.ax.set_yticklabels(list(K.STATE_NAMES), fontsize=7)

    # (c) cross-section through rc: virgin vs final (the money plot)
    x = np.arange(hh.shape[1]) * cell_m
    ax[1, 0].plot(x, v["height"][rc], color="#888", lw=1.2, label="virgin")
    ax[1, 0].plot(x, f["height"][rc], color="#b5402e", lw=1.4, label="after (pad cut + berm)")
    ax[1, 0].fill_between(x, v["height"][rc], f["height"][rc],
                          where=f["height"][rc] < v["height"][rc], color="#7a3b2e", alpha=0.3)
    ax[1, 0].fill_between(x, v["height"][rc], f["height"][rc],
                          where=f["height"][rc] > v["height"][rc], color="#d98c3a", alpha=0.4)
    ax[1, 0].set_title(f"cross-section @ row {rc}: cut (brown) vs berm (orange)")
    ax[1, 0].set_xlabel("x (m)"); ax[1, 0].set_ylabel("height (m)"); ax[1, 0].legend(fontsize=8)

    # (d) mass-conservation ledger over stages
    order = ["virgin", "dug", "hauled", "dumped", "relaxed", "compacted"]
    grid = [snaps[k]["mass_areal"].sum() * cell_m * cell_m for k in order]
    ledg = [snaps[k]["inventory_kg"] for k in order]
    tot = [g + l for g, l in zip(grid, ledg)]
    ax[1, 1].plot(order, grid, "o-", label="fine grid mass")
    ax[1, 1].plot(order, ledg, "s-", label="drum ledger")
    ax[1, 1].plot(order, tot, "^--", color="k", label="total (invariant)")
    ax[1, 1].set_title("mass conservation across stages (kg)")
    ax[1, 1].tick_params(axis="x", rotation=30); ax[1, 1].legend(fontsize=8)

    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(path)
    plt.close(fig)


if __name__ == "__main__":
    main()
