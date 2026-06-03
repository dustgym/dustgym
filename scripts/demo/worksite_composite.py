#!/usr/bin/env python3
"""worksite_composite.py — the 3x2 headline composite for the streaming roam (2026-06-02).

render_composite() draws the 3x2 from a corridor-array dict + a meta dict + a Godot rovercam.
It is shared by BOTH the final still (this script's main, reading corridor.npz/telemetry.json)
and the per-step progress GIF (worksite_progress.py, which calls it once per captured frame):

  [ Godot rover-cam (inspection)  ][ overhead worked corridor ][ site context: 2.5 km Haworth crop ]
  [ centreline relief (+ range)   ][ mass flow dug→drum→berm  ][ berm transverse cross-section     ]

Run AFTER the Godot render of the final berm window:
    python scripts/demo/worksite_roam.py --out out/worksite_roam
    bash out/worksite_roam/godot_cmd.txt
    python scripts/demo/worksite_composite.py --out out/worksite_roam
"""
from __future__ import annotations

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import numpy as np  # noqa: E402

from terrain_authority import constants as K  # noqa: E402
from terrain_authority.io_fields import load_scene  # noqa: E402

BUNDLE = "samples/lunar_dem/haworth_10km_5m"
STATE_COLS = ["#6b6b6b", "#c8a24a", "#7a3b2e", "#d98c3a", "#3a6ea5"]  # VIRGIN/TREAD/EXC/SPOIL/BERM


def hillshade(h, cell, az_deg=315.0, alt_deg=40.0):
    gy, gx = np.gradient(np.asarray(h, float), cell)
    slope = np.arctan(np.hypot(gx, gy)); aspect = np.arctan2(-gx, gy)
    az = np.deg2rad(az_deg); alt = np.deg2rad(alt_deg)
    return np.clip(np.sin(alt) * np.cos(slope) + np.cos(alt) * np.sin(slope) * np.cos(az - aspect), 0, 1)


def load_corridor_npz(path):
    d = np.load(path)
    return {k: d[k] for k in d.files}


def fixed_ylims(arr, pad_moved_kg, peak_inventory_kg):
    """Final Y-axis ranges for panels (4) relief and (5) mass flow, computed ONCE from the final
    state so every GIF frame shares them (no per-frame autoscale bounce). Panel 4 spans the worked
    relief band; panel 5 spans 0..peak mass."""
    H = np.asarray(arr["height"], float); vh = np.asarray(arr["virgin_height"], float)
    cell = float(arr["cell"]); yc = int(arr["yc_row"])
    band = (H - vh)[max(0, yc - int(8.0 / cell)):yc + int(8.0 / cell)]
    rmin, rmax = float(band.min()), float(band.max())
    pad = 0.08 * max(rmax - rmin, 0.1)
    mmax = max(float(pad_moved_kg), float(peak_inventory_kg)) / 1000.0
    return (rmin - pad, rmax + pad), (-0.03 * mmax, mmax * 1.07)


def mass_timeline_from_stages(stages, pad_moved_kg):
    """(label, dug_kg, drum_kg) per stage. dug = cumulative mass lifted from the pad (the dig is a
    single 'flatten_pad' stage), drum = ledger now; placed-in-berm = dug - drum (derived in-panel)."""
    fi = next((i for i, s in enumerate(stages) if s["stage"] == "flatten_pad"), None)
    out = []
    for i, s in enumerate(stages):
        dug = pad_moved_kg if (fi is not None and i >= fi) else s["inventory_kg"]
        out.append([s["stage"], float(dug), float(s["inventory_kg"])])
    return out


# --- one cached coarse-Haworth crop (the site-context map) shared across all GIF frames ----
_MAP_CACHE = {}


def _map_crop(map_bundle, cx, cy, zoom_km):
    key = (map_bundle, round(cx, 1), round(cy, 1), zoom_km)
    if key in _MAP_CACHE:
        return _MAP_CACHE[key]
    cf, cm = load_scene(map_bundle)
    cH = cf["heightmap"].astype(np.float64); cb = float(cm["grid"]["cell_m"])
    wb = cm["world_bounds_m"]; cx0, cy0 = float(wb["x0"]), float(wb["y0"])
    half = zoom_km * 1000.0 / 2.0
    px0 = max(0, int((cx - half - cx0) / cb)); px1 = min(cH.shape[1], int((cx + half - cx0) / cb))
    py0 = max(0, int((cy - half - cy0) / cb)); py1 = min(cH.shape[0], int((cy + half - cy0) / cb))
    crop = cH[py0:py1, px0:px1]
    ext_km = [(px0 * cb + cx0 - cx) / 1000.0, (px1 * cb + cx0 - cx) / 1000.0,
              (py0 * cb + cy0 - cy) / 1000.0, (py1 * cb + cy0 - cy) / 1000.0]   # km, centred on corridor
    out = (hillshade(crop, cb), ext_km)
    _MAP_CACHE[key] = out
    return out


def render_composite(arr, meta, rovercam_path, map_bundle, out_path, *, map_zoom_km=2.5):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap, BoundaryNorm
    from matplotlib.patches import Rectangle
    import matplotlib.image as mpimg

    H = arr["height"]; vh = arr["virgin_height"]; state = arr["state"]
    cell = float(arr["cell"]); yc = int(arr["yc_row"])
    ox, oy = arr["origin"]; x_s = float(arr["x_s"]); y_s = float(arr["y_s"])
    pad_x = arr["pad_x"]; haul_x = arr["haul_x"]; berm_x = arr["berm_x"]
    x0_m = ox - x_s; y0_m = oy - y_s
    relief = H - vh
    cmap = ListedColormap(STATE_COLS); norm = BoundaryNorm(np.arange(-0.5, 5.5, 1), cmap.N)
    rover_xy = meta.get("rover_xy")                         # [x-x_s, y-y_s] or None

    # FIXED canvas + FIXED margins so every frame is pixel-identical (no bbox_inches='tight',
    # which would resize the figure whenever a title/footer's text length changed -> GIF jitter).
    fig = plt.figure(figsize=(19, 9.6), dpi=110)
    gs = fig.add_gridspec(2, 3, left=0.045, right=0.985, top=0.905, bottom=0.065,
                          hspace=0.32, wspace=0.22)

    # ---- (1) Godot rover-cam -------------------------------------------------------
    ax = fig.add_subplot(gs[0, 0])
    if rovercam_path and os.path.exists(rovercam_path):
        ax.imshow(mpimg.imread(rovercam_path))
        ax.set_title("(1) Godot rover-cam (inspection light) — following the rover", fontsize=11)
    else:
        ax.text(0.5, 0.5, "rovercam pending", ha="center", va="center", color="#a33")
        ax.set_title("(1) Godot rover-cam (pending)", fontsize=11)
    ax.set_xticks([]); ax.set_yticks([])

    # ---- (2) overhead worked corridor (state tint over hillshade) ------------------
    ax = fig.add_subplot(gs[0, 1])
    band = 9.0
    r_lo = max(0, int((y_s - band - oy) / cell)); r_hi = min(H.shape[0], int((y_s + band - oy) / cell))
    ext = [x0_m, x0_m + H.shape[1] * cell, (oy + r_lo * cell) - y_s, (oy + r_hi * cell) - y_s]
    ax.imshow(hillshade(H[r_lo:r_hi], cell), origin="lower", cmap="gray", extent=ext, aspect="auto", vmin=0, vmax=1)
    ax.imshow(state[r_lo:r_hi], origin="lower", cmap=cmap, norm=norm, extent=ext, aspect="auto", alpha=0.55)
    for (a, b), lab, col in [(pad_x, "pad", "#7a3b2e"), (haul_x, "haul", "#c8a24a"), (berm_x, "berm", "#3a6ea5")]:
        ax.add_patch(Rectangle((a, ext[2]), b - a, ext[3] - ext[2], fill=False, ec=col, lw=1.4, ls="--"))
        ax.text((a + b) / 2, ext[3] - 1.3, lab, color=col, ha="center", fontsize=9, weight="bold")
    if rover_xy is not None:
        ax.plot([rover_xy[0]], [rover_xy[1]], marker="o", ms=9, mfc="#ff2b2b", mec="white", mew=1.2)
    ax.set_title("(2) overhead worked corridor — dig pad → TREAD haul → berm line", fontsize=11)
    ax.set_xlabel("x along corridor (m)"); ax.set_ylabel("y (m)")

    # ---- (3) site context: 2.5 km Haworth crop + worked-corridor box ---------------
    ax = fig.add_subplot(gs[0, 2])
    bx = np.asarray(meta["worked_world_bbox"], float)       # (x0,y0,x1,y1) world m
    mc = meta.get("map_center")                             # fixed crop centre (stable across GIF) or None
    cxc, cyc = (float(mc[0]), float(mc[1])) if mc else (0.5 * (bx[0] + bx[2]), 0.5 * (bx[1] + bx[3]))
    chs, cext = _map_crop(map_bundle, cxc, cyc, map_zoom_km)
    ax.imshow(chs, origin="lower", cmap="gray", extent=cext, aspect="equal", vmin=0, vmax=1)
    bdx = (0.5 * (bx[0] + bx[2]) - cxc) / 1000.0; bdy = (0.5 * (bx[1] + bx[3]) - cyc) / 1000.0
    rw = max((bx[2] - bx[0]) / 1000.0, 0.02); rh = max((bx[3] - bx[1]) / 1000.0, 0.02)
    ax.add_patch(Rectangle((bdx - rw / 2, bdy - rh / 2), rw, rh, fill=False, ec="#ff2b2b", lw=2.0))
    ax.plot(bdx, bdy, "+", color="#ff2b2b", ms=10, mew=1.5)
    ax.annotate(f"worked corridor\n{bx[2]-bx[0]:.0f}×{bx[3]-bx[1]:.0f} m", (bdx, bdy),
                (cext[1] * 0.30, cext[3] * 0.62), color="#ff2b2b", fontsize=9,
                arrowprops=dict(arrowstyle="->", color="#ff2b2b"))
    ax.set_title(f"(3) site context: {map_zoom_km:g}×{map_zoom_km:g} km of real Haworth LOLA DEM (fine window streams the box)",
                 fontsize=9.5)
    ax.set_xlabel("x from corridor (km)"); ax.set_ylabel("y from corridor (km)")

    # ---- (4) centreline relief + transverse value-range band -----------------------
    ax = fig.add_subplot(gs[1, 0])
    xax = x0_m + np.arange(H.shape[1]) * cell
    grade_diff = float(vh[yc].max() - vh[yc].min())
    hb = int(8.0 / cell)
    bandr = relief[max(0, yc - hb):yc + hb]
    rmin = bandr.min(axis=0); rmax = bandr.max(axis=0)
    ax.fill_between(xax, rmin, rmax, color="#caa", alpha=0.45, label="range across corridor width (±8 m)")
    ax.axhline(0, color="#999", lw=0.8)
    ax.plot(xax, relief[yc], color="#b5402e", lw=1.3, label="centreline")
    for (a, b), col in [(pad_x, "#7a3b2e"), (berm_x, "#3a6ea5")]:
        ax.axvspan(a, b, color=col, alpha=0.07)
    ax.set_title(f"(4) worked relief = after − virgin  ({grade_diff:.0f} m regional grade differenced out)", fontsize=10)
    ax.set_xlabel("x along corridor (m)"); ax.set_ylabel("Δ height (m)"); ax.legend(fontsize=7.5, loc="upper left")
    if meta.get("ylim4"):
        ax.set_ylim(*meta["ylim4"])

    # ---- (5) mass flow: dug → carried in drum → placed in berm ---------------------
    ax = fig.add_subplot(gs[1, 1])
    tl = meta["mass_timeline"]                              # [[label, dug_kg, drum_kg], ...]
    labels = [t[0] for t in tl]
    dug = np.array([t[1] for t in tl]) / 1000.0
    drum = np.array([t[2] for t in tl]) / 1000.0
    placed = dug - drum
    ax.plot(labels, dug, "o-", color="#7a3b2e", ms=4, label="dug from pad (cumulative)")
    ax.plot(labels, drum, "s-", color="#c46210", ms=4, label="carried in drum (now)")
    ax.plot(labels, placed, "^-", color="#3a6ea5", ms=4, label="placed in berm (cumulative)")
    ax.fill_between(labels, 0, placed, color="#3a6ea5", alpha=0.12)
    ax.tick_params(axis="x", rotation=55, labelsize=7)
    ax.set_ylabel("mass (tonnes)"); ax.legend(fontsize=7.5, loc="upper left")
    ax.set_title("(5) mass flow: dug → carried in drum → placed in berm  (dug = drum + berm)", fontsize=10)
    if meta.get("ylim5"):
        ax.set_ylim(*meta["ylim5"])

    # ---- (6) berm transverse cross-section (the deposited pile) ---------------------
    ax = fig.add_subplot(gs[1, 2])
    bx_mid = 0.5 * (berm_x[0] + berm_x[1])
    col = int(np.clip((bx_mid - x0_m) / cell, 0, H.shape[1] - 1))
    yax = (np.arange(H.shape[0]) * cell + y0_m)             # transverse y rel. centreline
    rr0 = max(0, yc - int(3.5 / cell)); rr1 = min(H.shape[0], yc + int(3.5 / cell))
    vprof = vh[rr0:rr1, col]; fprof = H[rr0:rr1, col]; yy = yax[rr0:rr1]
    ax.plot(yy, vprof, color="#888", lw=1.1, label="virgin")
    ax.plot(yy, fprof, color="#b5402e", lw=1.4, label="after (berm pile)")
    ax.fill_between(yy, vprof, fprof, where=fprof > vprof, color="#d98c3a", alpha=0.55, label="deposited spoil")
    ax.fill_between(yy, vprof, fprof, where=fprof < vprof, color="#7a3b2e", alpha=0.4)
    comp = (state[rr0:rr1, col] == int(K.STATE_COMPACTED_BERM))
    if comp.any():
        ax.plot(yy[comp], fprof[comp], ".", color="#3affd0", ms=4, label="COMPACTED_BERM crest")
    pile = float((fprof - vprof).max())
    ax.set_title(f"(6) berm cross-section (transverse @ mid-berm): pile +{pile:.2f} m", fontsize=10)
    ax.set_xlabel("y across berm (m)"); ax.set_ylabel("height (m)"); ax.legend(fontsize=7, loc="upper left")

    # ---- title (no footer status-bar: a variable-width footer + tight bbox jittered the GIF) ---
    site_kind = "raw terraced DEM" if meta.get("raw_terraces") else "bilinear-smoothed DEM (G7 fix)"
    flabel = meta.get("frame_label")
    sup = ("WorkSite streaming roam on real Haworth LOLA DEM — flatten a pad, haul, build a berm line "
           f"  |  {site_kind}  |  mass-conserved to {meta['worst_rel_residual']:.0e}")
    if flabel:
        sup += f"\n{flabel}"
    fig.suptitle(sup, fontsize=12.5, y=0.985)
    fig.savefig(out_path)                                   # fixed-size canvas (NO bbox_inches='tight')
    plt.close(fig)
    return out_path


def main():
    ap = argparse.ArgumentParser(description="3x2 composite for the WorkSite streaming roam (final still)")
    ap.add_argument("--out", default="out/worksite_roam")
    ap.add_argument("--name", default="worksite_composite.png")
    ap.add_argument("--map-zoom-km", type=float, default=2.5)
    args = ap.parse_args()

    arr = load_corridor_npz(os.path.join(args.out, "corridor.npz"))
    with open(os.path.join(args.out, "telemetry.json")) as fh:
        tel = json.load(fh)
    meta = {
        "mass_timeline": mass_timeline_from_stages(tel["stages"], tel["pad_moved_kg"]),
        "pad_moved_kg": tel["pad_moved_kg"], "worst_rel_residual": tel["worst_rel_residual"],
        "conservation_pass": tel["conservation_pass"], "recenters": tel["recenters"],
        "peak_inventory_kg": tel["peak_inventory_kg"], "drum_cycles": tel["drum_cycles"],
        "berm_repose": tel["berm_local_repose_deg"], "raw_terraces": tel["params"].get("raw_terraces"),
        "worked_world_bbox": tel["worked_world_bbox"],
        "rover_xy": [float(arr["berm_x"][1]) - 4.0, float(arr["y_s"]) - float(arr["y_s"])],  # at berm end, centreline
    }
    meta["ylim4"], meta["ylim5"] = fixed_ylims(arr, tel["pad_moved_kg"], tel["peak_inventory_kg"])
    out = render_composite(arr, meta, os.path.join(args.out, "rovercam.png"), BUNDLE,
                           os.path.join(args.out, args.name), map_zoom_km=args.map_zoom_km)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
