#!/usr/bin/env python3
"""worksite_progress.py — per-step 3x2 PROGRESS GIF for the streaming roam (2026-06-02).

Replays the manifest worksite_roam.py wrote (out/<run>/frames/manifest.json). For each captured
simulation step it: (1) renders the Godot rover-cam from that step's active-window LOCAL bundle with
a follow-cam pose (camera tracks the rover through pad→haul→berm), then (2) renders the full 3x2
composite for that step (worksite_composite.render_composite), with panels 2/4/5/6 showing the work
accumulating. Finally it stitches the per-step composites into out/<run>/progress.gif.

Run AFTER worksite_roam.py:
    python scripts/demo/worksite_roam.py --out out/worksite_roam
    python scripts/demo/worksite_progress.py --out out/worksite_roam
The Godot renders are the slow part (~15-20 s each); existing rovercam_NN.png are reused unless --force.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
sys.path.insert(0, _ROOT)
sys.path.insert(0, _HERE)

from PIL import Image  # noqa: E402

import worksite_composite as WC  # noqa: E402


def render_godot(bundle: str, rover_rc, pose, sun_elev, sun_azim, exposure, out_png: str,
                 *, size="960x720", timeout=240) -> bool:
    cmd = ["./render_layers.sh", "--", "--scene", bundle, "--layers", "terrain,clasts,rover",
           "--rover-rc", f"{int(rover_rc[0])},{int(rover_rc[1])}", "--size", size,
           "--pose", ",".join(f"{float(v):.2f}" for v in pose),
           "--sun-elev", str(sun_elev), "--sun-azim", str(sun_azim), "--exposure", str(exposure),
           "--out", os.path.abspath(out_png)]
    try:
        subprocess.run(cmd, cwd=os.path.join(_ROOT, "godot_sidecar"),
                       check=False, capture_output=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return False
    return os.path.exists(out_png)


def pad_to(im: Image.Image, w: int, h: int) -> Image.Image:
    canvas = Image.new("RGB", (w, h), "white")
    canvas.paste(im, ((w - im.width) // 2, (h - im.height) // 2))
    return canvas


def main() -> None:
    ap = argparse.ArgumentParser(description="per-step progress GIF for the WorkSite streaming roam")
    ap.add_argument("--out", default="out/worksite_roam")
    ap.add_argument("--force", action="store_true", help="re-render Godot frames even if cached")
    ap.add_argument("--gif-width", type=int, default=1500, help="downscale composite frames to this width")
    ap.add_argument("--duration-ms", type=int, default=1100)
    ap.add_argument("--no-godot", action="store_true", help="skip Godot (camera panel shows 'pending')")
    args = ap.parse_args()

    frames_dir = os.path.join(args.out, "frames")
    with open(os.path.join(frames_dir, "manifest.json")) as fh:
        man = json.load(fh)
    fr_list = man["frames"]
    print(f"progress: {len(fr_list)} steps; Godot follow-cam {'SKIPPED' if args.no_godot else 'per step'}")

    # fixed Y ranges for panels (4) and (5), computed ONCE from the FINAL step (no per-frame bounce)
    last = fr_list[-1]
    final_arr = WC.load_corridor_npz(os.path.join(frames_dir, f"frame_{last['idx']:02d}.npz"))
    ylim4, ylim5 = WC.fixed_ylims(final_arr, last["pad_moved_kg"], last["peak_inventory_kg"])

    mass_tl: list = []
    comp_paths: list = []
    for fr in fr_list:
        idx = fr["idx"]
        rovercam = os.path.join(frames_dir, f"rovercam_{idx:02d}.png")
        if not args.no_godot and (args.force or not os.path.exists(rovercam)):
            ok = render_godot(fr["bundle"], fr["rover_rc"], fr["pose"],
                              fr["sun_elev"], fr["sun_azim"], fr["exposure"], rovercam)
            print(f"  [{idx:02d}] godot {'ok' if ok else 'FAIL'}  {fr['frame_label']}")
        arr = WC.load_corridor_npz(os.path.join(frames_dir, f"frame_{idx:02d}.npz"))
        mass_tl.append(fr["mass_point"])
        worst = fr["worst_rel_so_far"] or 0.0
        peak = fr["peak_inventory_kg"]
        meta = {
            "mass_timeline": [list(p) for p in mass_tl],
            "pad_moved_kg": fr["pad_moved_kg"], "worst_rel_residual": worst or man["worst_rel_residual"],
            "conservation_pass": worst < 1e-6, "recenters": fr["recenters"],
            "peak_inventory_kg": peak, "drum_cycles": math.ceil(peak / 30.0) if peak > 0 else 0,
            "berm_repose": fr["berm_repose"] or None, "raw_terraces": man["raw_terraces"],
            "worked_world_bbox": fr["worked_world_bbox"], "rover_xy": fr["rover_xy"],
            "frame_label": f"step {idx + 1}/{len(fr_list)}: {fr['frame_label']}", "map_center": man["map_center"],
            "ylim4": ylim4, "ylim5": ylim5,
        }
        comp = os.path.join(frames_dir, f"composite_{idx:02d}.png")
        cam_arg = rovercam if (not args.no_godot and os.path.exists(rovercam)) else None
        WC.render_composite(arr, meta, cam_arg, man["map_bundle"], comp,
                            map_zoom_km=man.get("map_zoom_km", 2.5))
        comp_paths.append(comp)

    # stitch -> progress.gif (downscale + pad to a common canvas)
    imgs = []
    for p in comp_paths:
        im = Image.open(p).convert("RGB")
        if im.width > args.gif_width:
            im = im.resize((args.gif_width, round(im.height * args.gif_width / im.width)))
        imgs.append(im)
    w = max(i.width for i in imgs); h = max(i.height for i in imgs)
    imgs = [pad_to(i, w, h) for i in imgs]
    gif = os.path.join(args.out, "progress.gif")
    # hold the last frame a beat longer
    durations = [args.duration_ms] * (len(imgs) - 1) + [args.duration_ms * 3]
    imgs[0].save(gif, save_all=True, append_images=imgs[1:], duration=durations, loop=0)
    print(f"\nwrote {gif} ({len(imgs)} steps, {w}x{h})")


if __name__ == "__main__":
    main()
