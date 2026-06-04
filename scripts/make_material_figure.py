#!/usr/bin/env python3
"""Material-layer figure: per-cell regolith strength + trafficability from a scene's real density field.

Density | friction angle | cohesion | slip susceptibility, derived by terrain_authority.material from the
conserved density.rf32 (worked scenes vary, so the fields vary). No synthetic data.

Usage:
    <venv>/bin/python make_material_figure.py --scene samples/crater_boulders_worked --out <png>
"""
from __future__ import annotations

import argparse
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from terrain_authority import material as mat  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--scene", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    rho = mat.load_density(args.scene)
    f = mat.material_fields(rho)
    panels = [
        ("density [kg/m^3]", rho, "viridis"),
        ("friction angle [deg]", f["friction_deg"], "cividis"),
        ("cohesion [Pa]", f["cohesion_pa"], "magma"),
        ("slip susceptibility (loose = high)", f["slip_susceptibility"], "inferno"),
    ]
    fig, ax = plt.subplots(1, 4, figsize=(14.0, 3.9))
    for axi, (title, data, cmap) in zip(ax, panels):
        im = axi.imshow(data, cmap=cmap)
        axi.set_title(title)
        axi.set_xticks([])
        axi.set_yticks([])
        fig.colorbar(im, ax=axi, fraction=0.046)
    scene = os.path.basename(args.scene.rstrip("/"))
    fig.suptitle("World model Material layer: per-cell regolith strength from the conserved density field "
                 f"(real {scene})", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(args.out, dpi=130)
    print(f"wrote {args.out}  (friction {f['friction_deg'].min():.0f}-{f['friction_deg'].max():.0f} deg, "
          f"cohesion {f['cohesion_pa'].min():.0f}-{f['cohesion_pa'].max():.0f} Pa)")


if __name__ == "__main__":
    main()
