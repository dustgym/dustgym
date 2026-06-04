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


def render(scene_dir: str, out_name: str, layers: str = "terrain") -> str:
    """Render a scene bundle to godot_sidecar/out/<out_name> via the headless Godot sidecar."""
    os.makedirs(os.path.dirname(os.path.join(_SIDE, "out", out_name)), exist_ok=True)
    subprocess.run(
        [_RENDER, os.path.join(_SIDE, "sidecar.tscn"), "--",
         "--scene", os.path.abspath(scene_dir), "--layers", layers,
         "--size", "1024x768", "--out", out_name],
        capture_output=True, text=True, timeout=240)
    return os.path.join(_SIDE, "out", out_name)


def main():
    import cv2

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

    # before | after | earthwork (signed cut+/fill- depth over the pad)
    ew = np.where(plan["pad"], plan["h0"] - plan["target"], np.nan)
    vmax = float(np.nanmax(np.abs(ew))) or 1e-3
    fig, ax = plt.subplots(1, 3, figsize=(14.0, 4.2))
    for axi, png, title in ((ax[0], before_png, "BEFORE (Godot)"), (ax[1], after_png, "AFTER plan (Godot)")):
        img = cv2.imread(png)
        if img is not None:
            axi.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        axi.set_title(title)
        axi.set_xticks([])
        axi.set_yticks([])
    im = ax[2].imshow(ew, cmap="RdBu_r", vmin=-vmax, vmax=vmax)
    ax[2].set_title(f"Earthwork: cut (red) {plan['cut_vol_m3']:.2f} m3 / fill (blue) {plan['fill_vol_m3']:.2f} m3")
    ax[2].set_xticks([])
    ax[2].set_yticks([])
    fig.colorbar(im, ax=ax[2], fraction=0.046, label="cut + / fill - depth [m]")
    scene = os.path.basename(args.scene.rstrip("/"))
    fig.suptitle(f"Plan -> render: flatten a pad on {scene}, conserved cut/fill on the real surface", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    out_fig = os.path.join(args.out, "plan_render.png")
    fig.savefig(out_fig, dpi=130)
    plt.close(fig)
    print(f"wrote {out_fig}")


if __name__ == "__main__":
    main()
