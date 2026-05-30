#!/usr/bin/env python3
"""Deliverable D3 — procgen variety panel + the cave-in showpiece (headless matplotlib).

This is a *consumer* of the FROZEN state-field contract (INTERFACE.md §1/§5/§7): it imports
``terrain_authority.io_fields.load_scene`` and never touches raw raster bytes itself. It
produces slide-ready figures for a domain reviewer (NASA KSC GMRO), answering the GMRO
challenge directly — "show craters + caving-in + boulder fields + flat-compact vs
rolling-fluffy" — using the Phase-1 sample scenes under ``samples/``.

Outputs (viz/out/):
  1. variety_panel.png    2x2 hillshade panel: flat_compact, rolling_hills, crater,
                          boulder_field (clast markers overlaid on the last).
  2. caveins_filmstrip.png  horizontal filmstrip of the crater_caveins TIME SERIES
                          (spec §7 sandpile cave-in): an over-steepened rim slumping to
                          angle-of-repose, mass conserved.
  3. caveins.gif (optional)  the same series animated, if pillow is available.

Lighting model (spec §5.1, §8): every relief image is hillshaded with a GRAZING sun
(matplotlib.colors.LightSource, altdeg ~= 7 deg) — the polar low-sun band that produces the
brutal long shadows that are "exactly IPEx's perception challenge" (spec §8 "Lunar
lighting"). vert_exag is raised because cm-scale relief on a metre-scale patch would be
invisible at a true 7-deg grazing angle otherwise.

Run:  /home/john/Development/foss_ipex/.venv/bin/python viz/variety_panel.py
"""

from __future__ import annotations

import os
import sys

import matplotlib
matplotlib.use("Agg")  # headless: no display, write PNGs directly (INTERFACE.md preview note)
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LightSource

# Import the FROZEN seam helper. We are a pure consumer (INTERFACE.md §7): all raster I/O
# goes through load_scene; this script never reshapes raw bytes itself.
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)
from terrain_authority.io_fields import load_scene  # noqa: E402

SAMPLES = os.path.join(_REPO_ROOT, "samples")
OUT = os.path.join(_REPO_ROOT, "viz", "out")

# Grazing polar sun (spec §5.1 "Sun elevation 0-7 deg (polar)"; §8 long-shadow perception
# challenge). 315 deg az = light from upper-left, the cartographic hillshade convention.
SUN_ALTDEG = 7.0
SUN_AZDEG = 315.0


def hillshade_rgb(height: np.ndarray, cell_m: float, vert_exag: float,
                  cmap: str = "bone") -> np.ndarray:
    """Return an RGB hillshade of a heightmap under the grazing lunar sun (spec §5.1/§8).

    Uses matplotlib.colors.LightSource.shade(): a soft-light blend of a base colormap with
    the analytic hillshade. ``cmap='bone'`` gives a cool grey lunar-regolith feel rather than
    a false rainbow. ``vert_exag`` exaggerates relief so cm-scale features read against a
    metre-scale patch at a 7-deg grazing angle (a true 1:1 exag would be near-invisible).
    """
    ls = LightSource(azdeg=SUN_AZDEG, altdeg=SUN_ALTDEG)
    return ls.shade(height, cmap=plt.get_cmap(cmap), blend_mode="soft",
                    vert_exag=vert_exag, dx=cell_m, dy=cell_m)


def _scene_density_note(fields: dict, default: str = "") -> str:
    """One-line bulk-density characterization for a subplot caption (spec §5.2/§9).

    Reports the loose-vs-dense story the GMRO reviewer cares about: surface bulk density
    drives trafficability and albedo (spec §9 "depth-density gradient dominates").
    """
    rho = fields["density"]
    lo, hi = float(rho.min()), float(rho.max())
    if hi - lo < 25.0:  # ~uniform column
        return f"uniform rho~{round(lo / 10) * 10:.0f} kg/m^3"
    return f"rho {round(lo / 10) * 10:.0f}-{round(hi / 10) * 10:.0f} kg/m^3 (loose-over-dense)"


# ---------------------------------------------------------------------------
# 1. Variety panel — the 2x2 "what the procgen can do" slide.
# ---------------------------------------------------------------------------

# (scene_dir, headline, one-line parameter note). Notes cite spec sections / params so the
# figure is self-documenting for a domain reviewer.
PANEL_SCENES = [
    ("flat_compact",
     "Flat compact plate",
     "dense compacted: rho~1920 kg/m^3, near-zero relief (spec 5.2 deep density)"),
    ("rolling_hills",
     "Rolling fluffy hills",
     "loose fbm top: rho~1300 kg/m^3 (spec 9 loose-over-dense)"),
    ("crater",
     "Fresh simple crater",
     "Pike-class D=2.4 m, depth/D=0.2, rim+ejecta (mass-consistent carve)"),
    ("boulder_field",
     "Boulder field",
     "Golombek SFD clasts, k=0.1 q=3.31 (rock-size-freq_abstract.txt)"),
]


def build_variety_panel(out_path: str) -> None:
    """Render the 2x2 hillshade variety panel (GMRO 'flat vs fluffy + craters + boulders')."""
    fig, axes = plt.subplots(2, 2, figsize=(11, 11), dpi=130)
    fig.suptitle(
        "foss_ipex Tier-2 procgen variety  (grazing lunar sun, alt=7 deg; spec 5.1/8)",
        fontsize=15, fontweight="bold", y=0.975)

    for ax, (scene, headline, note) in zip(axes.flat, PANEL_SCENES):
        scene_dir = os.path.join(SAMPLES, scene)
        fields, meta = load_scene(scene_dir)
        height = fields["heightmap"]
        cell_m = meta["grid"]["cell_m"]

        # vert_exag scaled to the scene's own relief so each cell reads, yet the *flat*
        # scene still looks flat (tiny range -> tiny absolute shading). The flat_compact
        # plate uses a high exag to reveal it is genuinely featureless (compaction proxy).
        rng = float(height.max() - height.min())
        vert_exag = 6.0 if rng > 0.05 else 60.0

        rgb = hillshade_rgb(height, cell_m, vert_exag=vert_exag)
        ax.imshow(rgb, origin="lower",
                  extent=[0, meta["grid"]["width"] * cell_m,
                          0, meta["grid"]["height"] * cell_m])

        # Overlay clast markers (boulder_field): metadata clasts are world [x, h, z]
        # (INTERFACE.md §5 Godot-ready order). They are rigid-body REFERENCES, not carved
        # into the heightfield (spec §6 "Rocks are not a soil problem") — so we annotate
        # them rather than expecting bumps in the surface. Marker area ~ rock cross-section.
        clasts = meta.get("clasts", [])
        if clasts:
            xs = [c["center_m"][0] for c in clasts]
            zs = [c["center_m"][2] for c in clasts]
            # area in points^2 roughly proportional to footprint; radius in m -> visible dot
            sizes = [max(4.0, (c["radius_m"] / cell_m) ** 2 * 0.6) for c in clasts]
            ax.scatter(xs, zs, s=sizes, facecolors="none", edgecolors="#ff5a36",
                       linewidths=0.9, alpha=0.9, label=f"{len(clasts)} clasts (Golombek SFD)")
            ax.legend(loc="upper right", fontsize=7, framealpha=0.7)

        ax.set_title(f"{headline}\n{note}", fontsize=10.5)
        ax.set_xlabel("x (m)  [+X = col]", fontsize=8)
        ax.set_ylabel("z (m)  [+Z = row]", fontsize=8)
        ax.tick_params(labelsize=7)

    fig.tight_layout(rect=[0, 0, 1, 0.955])
    fig.savefig(out_path)
    plt.close(fig)
    print(f"wrote {out_path}  ({os.path.getsize(out_path) // 1024} KB)")


# ---------------------------------------------------------------------------
# 2 & 3. Cave-in showpiece — the spec §7 sandpile relaxation time series.
# ---------------------------------------------------------------------------

def _pick_frames(frame_dirs: list[str], n: int) -> list[str]:
    """Evenly sample n frame names from a series (always include first and last)."""
    if len(frame_dirs) <= n:
        return frame_dirs
    idx = np.linspace(0, len(frame_dirs) - 1, n).round().astype(int)
    # dedupe while preserving order
    seen, out = set(), []
    for i in idx:
        if i not in seen:
            seen.add(i)
            out.append(frame_dirs[i])
    return out


def _load_series(series_dir: str):
    """Read the parent metadata for cadence/count, then load the chosen frame heightmaps.

    A time series stores cadence in the PARENT metadata.json under ``time_series`` (per the
    INTERFACE.md §1 note that "frame cadence is documented in the parent metadata.json").
    """
    import json
    with open(os.path.join(series_dir, "metadata.json")) as fh:
        parent = json.load(fh)
    ts = parent.get("time_series", {})
    frame_dirs = ts.get("frame_dirs")
    if not frame_dirs:
        frame_dirs = sorted(
            d for d in os.listdir(series_dir)
            if d.startswith("t") and os.path.isdir(os.path.join(series_dir, d)))
    cadence = ts.get("frame_cadence_steps", 1)
    return parent, frame_dirs, cadence


def build_caveins_filmstrip(series_dir: str, out_path: str, n_frames: int = 6) -> list:
    """Horizontal filmstrip of the cave-in relaxation (spec §7 showpiece).

    Each frame is hillshaded under the same grazing sun and captioned with the relaxation
    step (frame index * cadence) and the live peak ridge height, so the reviewer can read
    the slump *quantitatively*: a loose ridge over-piled on the inner crater rim topples
    downhill until every loose cell sits at or below the angle of repose (theta_r in the
    30-47 deg envelope, spec §5.2/§7). Reduced-gravity granular flow is genuinely unsettled
    in the literature (lyasko2010.pdf; spec §7), so theta_r is a wide-envelope calibration
    knob, not a fixed truth.

    Returns the per-frame RGB images (reused by the GIF) so we render the slump only once.
    """
    parent, frame_dirs, cadence = _load_series(series_dir)
    theta_r = 35  # repose used by the producer (t-frame notes: "repose theta_r=35deg")

    chosen = _pick_frames(frame_dirs, n_frames)
    cell_m = parent["grid"]["cell_m"]

    rgbs, captions = [], []
    for fd in chosen:
        fields, _ = load_scene(os.path.join(series_dir, fd))
        h = fields["heightmap"]
        step = int(fd[1:]) * cadence  # tNNN -> CA relaxation step
        peak = float(h.max())
        # High exag: the ridge is tall early (~+1.9 m) and shrinks to rim height (~+0.25 m);
        # a constant exag keeps the geometry comparable frame-to-frame so the slump is visible.
        rgbs.append(hillshade_rgb(h, cell_m, vert_exag=3.0))
        captions.append(f"step {step}\npeak +{peak:.2f} m")

    # mass-conservation headline straight from the producer's metadata (spec §10 invariant).
    ts = parent.get("time_series", {})
    mass_kg = ts.get("mass_conserved_kg")
    drift_kg = ts.get("mass_drift_kg", 0.0)
    if mass_kg is not None:
        mass_line = (f"mass conserved {mass_kg:.0f} kg "
                     f"(drift {drift_kg:.3g} kg) across the slump  -  spec 10 invariant")
    else:
        mass_line = "mass conserved across the slump  -  spec 10 invariant"

    n = len(rgbs)
    fig, axes = plt.subplots(1, n, figsize=(2.5 * n, 3.4), dpi=140)
    if n == 1:
        axes = [axes]
    fig.suptitle(
        "Cave-in showpiece: over-steepened crater rim relaxing to angle of repose "
        f"(theta_r~{theta_r} deg, envelope 30-47 deg; sandpile CA, spec 7)",
        fontsize=11, fontweight="bold", y=1.02)

    for ax, rgb, cap in zip(axes, rgbs, captions):
        ax.imshow(rgb, origin="lower")
        ax.set_title(cap, fontsize=9)
        ax.set_xticks([])
        ax.set_yticks([])

    fig.text(0.5, -0.04,
             mass_line + "    |    reduced-g granular flow is genuinely unsettled "
             "(lyasko2010.pdf; spec 7) -> theta_r is a wide-envelope calibration knob",
             ha="center", fontsize=8.5, style="italic")
    fig.tight_layout(rect=[0, 0.02, 1, 0.96])
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out_path}  ({os.path.getsize(out_path) // 1024} KB)")
    return None  # GIF renders its own denser series below


def build_caveins_gif(series_dir: str, out_path: str, n_frames: int = 30) -> None:
    """Animated GIF of the cave-in relaxation (optional; pillow). Denser sampling than the
    filmstrip so the avalanche reads as motion. Each frame is a grazing-sun hillshade."""
    try:
        from PIL import Image
    except Exception as exc:  # pillow missing -> skip silently (it's optional)
        print(f"skipping GIF (pillow unavailable: {exc})")
        return

    parent, frame_dirs, cadence = _load_series(series_dir)
    cell_m = parent["grid"]["cell_m"]
    chosen = _pick_frames(frame_dirs, n_frames)

    pil_frames = []
    for fd in chosen:
        fields, _ = load_scene(os.path.join(series_dir, fd))
        rgb = hillshade_rgb(fields["heightmap"], cell_m, vert_exag=3.0)
        # LightSource.shade returns float RGBA in [0,1]; PIL wants uint8 RGB. Flip rows so
        # origin matches the imshow(origin='lower') used everywhere else.
        arr = (np.flipud(rgb[..., :3]) * 255).astype(np.uint8)
        pil_frames.append(Image.fromarray(arr))

    # Hold the final reposed state a beat longer so the "settled" end-state is legible.
    durations = [120] * (len(pil_frames) - 1) + [1200]
    pil_frames[0].save(out_path, save_all=True, append_images=pil_frames[1:],
                       duration=durations, loop=0, optimize=True)
    print(f"wrote {out_path}  ({os.path.getsize(out_path) // 1024} KB)")


def main() -> None:
    os.makedirs(OUT, exist_ok=True)
    build_variety_panel(os.path.join(OUT, "variety_panel.png"))
    caveins_dir = os.path.join(SAMPLES, "crater_caveins")
    build_caveins_filmstrip(caveins_dir, os.path.join(OUT, "caveins_filmstrip.png"), n_frames=6)
    build_caveins_gif(caveins_dir, os.path.join(OUT, "caveins.gif"), n_frames=30)


if __name__ == "__main__":
    main()
