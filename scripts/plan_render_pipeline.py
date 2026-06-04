#!/usr/bin/env python3
"""Plan -> render pipeline: visualize a flatten earthwork and how much regolith must move.

Loads a scene bundle (BEFORE) into the conserved ColumnState, plans a flatten of a central pad to a
level grade (cut the cells above target into the drum, fill the cells below from it -- mass-conserved),
writes the worked AFTER bundle, renders BEFORE and AFTER in Godot, and assembles a
before | after | earthwork figure with the cut and fill volumes. This is the planner's visual check:
render an approximation of what needs to happen and how much earth needs removal, then rerun (feedback)
with a different pad or target.

The same machinery accepts any INTERFACE.md scene bundle, including a DEM window cropped from a
user-selected map area (build_from_dem), so this is the core of the select-area -> plan -> render loop.

Usage:
    <venv>/bin/python plan_render_pipeline.py --scene samples/crater_boulders --out <work dir> [--pad-frac 0.5]
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.normpath(os.path.join(_HERE, ".."))
_SIDE = os.path.join(_REPO, "godot_sidecar")
_RENDER = os.path.join(_SIDE, "render.sh")
sys.path.insert(0, _REPO)
from terrain_authority.io_fields import save_scene  # noqa: E402
from terrain_authority.worksite import coarse_base_from_bundle  # noqa: E402


def plan_flatten(cs, pad_frac: float = 0.5) -> dict:
    """Flatten a central pad to its median level on the conserved ColumnState (cut above -> drum ->
    fill below). Mass-conserved. Returns the before/after heightfields, masks, and cut/fill volumes."""
    h0 = cs.derive_height().copy()
    H, W = cs.mass_areal.shape
    r0, r1 = int(H * (0.5 - pad_frac / 2)), int(H * (0.5 + pad_frac / 2))
    c0, c1 = int(W * (0.5 - pad_frac / 2)), int(W * (0.5 + pad_frac / 2))
    pad = np.zeros((H, W), dtype=bool)
    pad[r0:r1, c0:c1] = True
    target = float(np.median(h0[pad]))
    above = pad & (h0 > target)
    below = pad & (h0 < target)
    cut_areal = np.where(above, (h0 - target) * cs.density, 0.0)   # kg/m^2 to remove
    cut_kg = cs.cut_to_inventory(above, cut_areal)
    fill_kg = cs.fill_toward(below, target)
    h1 = cs.derive_height()
    area = cs.cell_m ** 2
    return {
        "pad": pad, "above": above, "below": below, "target": target,
        "cut_kg": cut_kg, "fill_kg": fill_kg, "drum_kg": float(cs.drum_inventory),
        "cut_vol_m3": float(np.where(above, h0 - target, 0.0).sum() * area),
        "fill_vol_m3": float(np.where(below, target - h0, 0.0).sum() * area),
        "h0": h0, "h1": h1,
    }


def write_bundle(cs, meta: dict, out_dir: str) -> str:
    """Write the (worked) ColumnState as a renderable INTERFACE.md bundle."""
    os.makedirs(out_dir, exist_ok=True)
    fields = {
        "heightmap": cs.derive_height().astype("<f4"),
        "mass_areal": cs.mass_areal.astype("<f4"),
        "density": cs.density.astype("<f4"),
        "disturbance": cs.disturbance.astype("<f4"),
        "state_label": cs.state_label.astype("u1"),
    }
    save_scene(out_dir, fields, meta)
    return out_dir


def render(scene_dir: str, out_name: str, layers: str = "terrain",
           sun_elev_deg: float | None = None, pose: str | None = None) -> str:
    """Render a scene bundle to godot_sidecar/out/<out_name> via the headless Godot sidecar.

    `sun_elev_deg` raises the sun for a lit planning/inspection view (the default 5 deg grazing sun
    leaves sloped terrain mostly shadowed); leave None for the sensor-faithful grazing render. `pose`
    is an explicit camera 'px,py,pz,tx,ty,tz' (the default auto-frame is tuned for ~5 m patches)."""
    os.makedirs(os.path.dirname(os.path.join(_SIDE, "out", out_name)), exist_ok=True)
    cmd = [_RENDER, os.path.join(_SIDE, "sidecar.tscn"), "--",
           "--scene", os.path.abspath(scene_dir), "--layers", layers,
           "--size", "1024x768", "--out", out_name]
    if pose is not None:
        cmd += ["--pose", pose]
    if sun_elev_deg is not None:
        cmd += ["--sun-elev", str(float(sun_elev_deg))]
    subprocess.run(cmd, capture_output=True, text=True, timeout=240)
    return os.path.join(_SIDE, "out", out_name)


def _make_figure(plan: dict, before_png: str, after_png: str, out_fig: str, title: str) -> str:
    """before | after | earthwork (signed cut+/fill- depth over the pad) with the cut/fill volumes."""
    import cv2
    ew = np.where(plan["pad"], plan["h0"] - plan["target"], np.nan)
    vmax = float(np.nanmax(np.abs(ew))) or 1e-3
    fig, ax = plt.subplots(1, 3, figsize=(14.0, 4.2))
    for axi, png, t in ((ax[0], before_png, "BEFORE (Godot)"), (ax[1], after_png, "AFTER plan (Godot)")):
        img = cv2.imread(png)
        if img is not None:
            axi.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        axi.set_title(t)
        axi.set_xticks([])
        axi.set_yticks([])
    im = ax[2].imshow(ew, cmap="RdBu_r", vmin=-vmax, vmax=vmax)
    ax[2].set_title(f"Earthwork: cut (red) {plan['cut_vol_m3']:.2f} m3 / fill (blue) {plan['fill_vol_m3']:.2f} m3")
    ax[2].set_xticks([])
    ax[2].set_yticks([])
    fig.colorbar(im, ax=ax[2], fraction=0.046, label="cut + / fill - depth [m]")
    fig.suptitle(title, fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(out_fig, dpi=130)
    plt.close(fig)
    return out_fig


def crop_window_columnstate(cs_full, u: float, v: float, win_cells: int, zoom: int):
    """Slice a win_cells window from a full ColumnState at the (u,v) fraction, resample by `zoom` (so a
    coarse DEM patch renders smoothly). A slice of the conserved state, so still mass-consistent."""
    from scipy.ndimage import zoom as ndzoom

    from terrain_authority.column_state import ColumnState
    H, W = cs_full.mass_areal.shape
    win_cells = min(win_cells, H, W)
    cr = int(np.clip(v, 0.0, 1.0) * (H - 1))
    cc = int(np.clip(u, 0.0, 1.0) * (W - 1))
    half = win_cells // 2
    r0 = int(np.clip(cr - half, 0, H - win_cells))
    c0 = int(np.clip(cc - half, 0, W - win_cells))
    sl = (slice(r0, r0 + win_cells), slice(c0, c0 + win_cells))
    datum = ndzoom(cs_full.datum[sl], zoom, order=1)
    datum = datum - float(datum.min())   # zero-base: Haworth carries absolute elevations (100s of m);
    #                                       the sidecar camera frames a ~0-relative patch (relief preserved)
    mass = ndzoom(cs_full.mass_areal[sl], zoom, order=1)
    dens = ndzoom(cs_full.density[sl], zoom, order=1)
    dist = ndzoom(cs_full.disturbance[sl], zoom, order=1)
    state = ndzoom(cs_full.state_label[sl].astype(float), zoom, order=0).astype("u1")
    hn, wn = datum.shape
    return ColumnState(wn, hn, cs_full.cell_m / zoom, mass_areal=mass, density=dens,
                       state_label=state, disturbance=dist, datum=datum)


def render_map_area(bundle_dir: str, u: float, v: float, out_dir: str,
                    *, win_cells: int = 10, zoom: int = 10, pad_frac: float = 0.5) -> dict:
    """Crop a window from a committed DEM bundle at the (u,v) fraction, plan a flatten, render
    BEFORE/AFTER in Godot, and return the image paths + earthwork volumes. The select-area -> render
    workhorse a browser /render endpoint drives. Cropping a small window also sidesteps the sidecar's
    100 m far-plane (a small selected window is exactly its scale)."""
    cs_full, meta = coarse_base_from_bundle(bundle_dir)
    cs = crop_window_columnstate(cs_full, u, v, win_cells, zoom)
    H, W = cs.mass_areal.shape
    win_meta = dict(meta)
    win_meta["scene_name"] = "map_window"
    win_meta["grid"] = {"width": W, "height": H, "cell_m": cs.cell_m, "order": "row-major-C"}
    win_meta["world_bounds_m"] = {"x0": 0.0, "y0": 0.0, "x1": W * cs.cell_m, "y1": H * cs.cell_m}
    os.makedirs(out_dir, exist_ok=True)
    before_dir = write_bundle(cs, win_meta, os.path.join(out_dir, "before_bundle"))
    plan = plan_flatten(cs, pad_frac=pad_frac)
    after_dir = write_bundle(cs, win_meta, os.path.join(out_dir, "after_bundle"))
    ext = W * cs.cell_m
    c = ext / 2.0
    pose = f"{c:.2f},{ext * 0.85:.2f},{c + ext * 0.6:.2f},{c:.2f},0,{c:.2f}"   # oblique bird's-eye
    before_png = render(before_dir, "plan_render/win_before.png", sun_elev_deg=25.0, pose=pose)
    after_png = render(after_dir, "plan_render/win_after.png", sun_elev_deg=25.0, pose=pose)
    fig = _make_figure(plan, before_png, after_png, os.path.join(out_dir, "plan_render.png"),
                       f"Select-area -> render: flatten at (u={u:.2f}, v={v:.2f}) on the real Haworth DEM")
    return {"before_png": before_png, "after_png": after_png, "figure": fig,
            "cut_vol_m3": plan["cut_vol_m3"], "fill_vol_m3": plan["fill_vol_m3"],
            "cut_kg": plan["cut_kg"], "fill_kg": plan["fill_kg"],
            "cell_m": cs.cell_m, "extent_m": W * cs.cell_m}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--scene", required=True, help="BEFORE scene bundle (samples/<scene> or a DEM window)")
    ap.add_argument("--out", required=True, help="work dir for the AFTER bundle + the figure")
    ap.add_argument("--pad-frac", type=float, default=0.5)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    cs, meta = coarse_base_from_bundle(args.scene)
    plan = plan_flatten(cs, pad_frac=args.pad_frac)
    after_dir = write_bundle(cs, meta, os.path.join(args.out, "after_bundle"))

    before_png = render(args.scene, "plan_render/before.png")
    after_png = render(after_dir, "plan_render/after.png")
    print(f"cut {plan['cut_vol_m3']:.3f} m3 ({plan['cut_kg']:.0f} kg) | "
          f"fill {plan['fill_vol_m3']:.3f} m3 ({plan['fill_kg']:.0f} kg) | "
          f"drum residual {plan['drum_kg']:.0f} kg | target {plan['target']:.3f} m")
    scene = os.path.basename(args.scene.rstrip("/"))
    out_fig = _make_figure(plan, before_png, after_png, os.path.join(args.out, "plan_render.png"),
                           f"Plan -> render: flatten a pad on {scene}, conserved cut/fill on the real surface")
    print(f"wrote {out_fig}")


if __name__ == "__main__":
    main()
