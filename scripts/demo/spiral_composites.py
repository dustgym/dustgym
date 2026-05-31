#!/usr/bin/env python3
"""Composite viz-battery GIFs for the spiral demo (John's spec, part 2).

Tiles the per-frame panels + the net-new top-down Godot renders into two synced GIFs over the
80 spiral frames:
  - composite_2x2.gif : TL actual-lighting top-down | TR unlit top-down (quadtree overlay)
                        BL position+SLAM            | BR map resource usage      (one run, LIT)
  - composite_3x2.gif : TL lit top-down | TC unlit top-down       | TR unlit-tag rover-cam frame
                        BL position+SLAM | BC failure breakdown (LIT vs UNLIT) | BR resource
                        (the lit-vs-unlit illumination A/B)

REUSE: every panel is a per-frame still from spiral_panels.py (position_frame / resource_frame /
failure_breakdown_pil) -- the SAME code that makes the standalone GIFs -- so the composites stay
in lockstep with them. The top-down PNGs (out/cam/haworth_spiral_topdown_{lit,unlit}/<NNN>/topdown.png)
and the unlit rover-cam frame (out/cam/haworth_spiral_unlit/<NNN>/front_left.png) are loaded off disk.
A missing tile renders as a labeled placeholder (so a partial render still assembles for inspection).

Host-only (numpy + matplotlib Agg + Pillow). imageio is NOT available; GIFs via PIL save_all.

Run: .venv/bin/python scripts/demo/spiral_composites.py
"""
from __future__ import annotations
import argparse, json, os, sys
from PIL import Image, ImageDraw, ImageFont

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(_ROOT, "scripts", "demo"))
sys.path.insert(0, os.path.join(_ROOT, "scripts", "ros2_bridge"))
import spiral_panels as P  # noqa: E402  (reuses _load_run / *_frame / _save_gif / failure_breakdown_pil)

BANNER_H = 26


def _font(size=15):
    for name in ("DejaVuSans.ttf", "DejaVuSans-Bold.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except Exception:
            continue
    return ImageFont.load_default()


def _load_png(path, label):
    """Load a PNG as RGB, or a labeled gray placeholder if absent (partial-render friendly)."""
    if os.path.exists(path):
        return Image.open(path).convert("RGB")
    im = Image.new("RGB", (512, 512), (58, 58, 66))
    d = ImageDraw.Draw(im)
    d.text((18, 230), f"{label}\n(missing: {os.path.relpath(path, _ROOT)})", fill=(225, 185, 185), font=_font(15))
    return im


def _tile(im, cw, ch, caption=None):
    """Fit `im` (aspect-preserved) centered into a white cw x ch cell; optional top caption strip."""
    im = im.convert("RGB").copy()
    im.thumbnail((cw, ch), Image.LANCZOS)
    cell = Image.new("RGB", (cw, ch), "white")
    cell.paste(im, ((cw - im.width) // 2, (ch - im.height) // 2))
    if caption:
        d = ImageDraw.Draw(cell)
        d.rectangle([0, 0, cw, 21], fill=(20, 20, 30))
        d.text((5, 3), caption, fill=(240, 240, 250), font=_font(15))
    return cell


def _draw_legend(img, entries, pad=10, sw=18, fs=15):
    """Draw a compact swatch legend in the bottom-left of `img`. entries = [((r,g,b), label), ...]."""
    img = img.convert("RGB").copy()
    d = ImageDraw.Draw(img)
    f = _font(fs)
    line_h = max(sw, fs) + 6
    box_h = pad * 2 + line_h * len(entries)
    box_w = int(pad * 2 + sw + 8 + max(d.textlength(lbl, font=f) for _, lbl in entries))
    x0, y0 = pad, img.height - box_h - pad
    d.rectangle([x0, y0, x0 + box_w, y0 + box_h], fill=(0, 0, 0))
    for i, (col, lbl) in enumerate(entries):
        yy = y0 + pad + i * line_h
        d.rectangle([x0 + pad, yy, x0 + pad + sw, yy + sw], fill=col)
        d.text((x0 + pad + sw + 8, yy + 1), lbl, fill=(235, 235, 240), font=f)
    return img


# Quadtree LOD ramp (matches terrain.gd::_lod_color warm->cool) + the marker hues, for the legend.
_QT_LEGEND = [
    ((242, 130, 70), "fine LOD (rover corridor)"),
    ((63, 143, 216), "coarse far-field"),
    ((255, 30, 217), "rover"),
    ((235, 235, 235), "lander"),
]


def _grid(cells, cols, rows, cw, ch, banner):
    """Assemble (image, caption) cells row-major into a cols x rows grid with a top banner."""
    W = cols * cw
    H = rows * ch + BANNER_H
    canvas = Image.new("RGB", (W, H), (245, 245, 248))
    d = ImageDraw.Draw(canvas)
    d.rectangle([0, 0, W, BANNER_H], fill=(10, 10, 16))
    d.text((6, 5), banner, fill=(250, 250, 255), font=_font(16))
    for i, (im, cap) in enumerate(cells):
        r = i // cols
        c = i % cols
        canvas.paste(_tile(im, cw, ch, cap), (c * cw, BANNER_H + r * ch))
    return canvas


def build_composites(cam_root, scene_dir, out_dir, *, cell=512):
    os.makedirs(out_dir, exist_ok=True)

    # detect+truth rows for the two illumination runs (drives position+SLAM + failure breakdown)
    runs = {}
    lander_xy = {}
    for name in ("lit", "unlit"):
        rd = os.path.join(cam_root, f"haworth_spiral_{name}")
        if not os.path.isdir(rd):
            print(f"(warn: rover-cam run absent: {rd})")
            runs[name] = []
            lander_xy[name] = (0.0, 0.0)
            continue
        lxy, rows = P._load_run(rd)
        runs[name] = rows
        lander_xy[name] = lxy

    rp = os.path.join(scene_dir, "resource.json")
    if not os.path.exists(rp):
        print(f"(skip: {rp} absent -- run instrument_spiral.py first)")
        return
    Rj = json.load(open(rp))
    nrec = len(Rj["records"])

    n_lit = len(runs["lit"])
    n_unlit = len(runs["unlit"])
    N = min(x for x in (n_lit, n_unlit, nrec) if x > 0) if (n_lit or n_unlit) else nrec
    if N == 0:
        print("(skip: no frames found)")
        return
    print(f"composites: {N} frames (lit={n_lit}, unlit={n_unlit}, resource={nrec})")

    td_lit = os.path.join(cam_root, "haworth_spiral_topdown_lit")
    td_unlit = os.path.join(cam_root, "haworth_spiral_topdown_unlit")
    rovercam_unlit = os.path.join(cam_root, "haworth_spiral_unlit")
    for d in (td_lit, td_unlit):
        if not os.path.isdir(d):
            print(f"(warn: top-down run absent: {d} -- tiles will be placeholders)")

    # shared fixed axes for the position panel (LIT run feeds both composites' position+SLAM tile),
    # and the static lit-vs-unlit failure breakdown pasted into every 3x2 frame.
    axes_lit = P.position_axes(lander_xy["lit"], runs["lit"]) if n_lit else None
    fail_pil = P.failure_breakdown_pil(runs) if (n_lit and n_unlit) else None

    frames2, frames3 = [], []
    for idx in range(N):
        nnn = f"{idx:03d}"
        tdl = _load_png(os.path.join(td_lit, nnn, "topdown.png"), "top-down LIT")
        tdu = _draw_legend(_load_png(os.path.join(td_unlit, nnn, "topdown.png"), "top-down UNLIT"), _QT_LEGEND)
        rcu = _load_png(os.path.join(rovercam_unlit, nnn, "front_left.png"), "rover-cam UNLIT")
        pos = (P.position_frame(lander_xy["lit"], runs["lit"], idx + 1, *axes_lit,
                                "truth vs AprilTag-SLAM (lit)")
               if axes_lit else _load_png("", "position+SLAM"))
        res = P.resource_frame(Rj, min(idx + 1, nrec))

        cells2 = [
            (tdl, "top-down: 7° grazing sun (exposure-boosted overhead)"),
            (tdu, "top-down: unlit — demand-driven quadtree LOD"),
            (pos, None),
            (res, None),
        ]
        frames2.append(_grid(cells2, 2, 2, cell, cell,
                             f"foss_ipex spiral demo  |  frame {idx}/{N-1}  |  "
                             f"top-down (lit / unlit) + position+SLAM + map resource"))

        cells3 = [
            (tdl, "top-down: 7° grazing sun (exp-boosted)"),
            (tdu, "top-down: unlit — quadtree LOD"),
            (rcu, "rover-cam: unlit-tag front-left"),
            (pos, None),
            (fail_pil if fail_pil is not None else _load_png("", "failure breakdown"), None),
            (res, None),
        ]
        frames3.append(_grid(cells3, 3, 2, cell, cell,
                             f"foss_ipex spiral demo (lit vs unlit illumination A/B)  |  frame {idx}/{N-1}"))

    P._save_gif(frames2, os.path.join(out_dir, "composite_2x2.gif"))
    P._save_gif(frames3, os.path.join(out_dir, "composite_3x2.gif"))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam-root", default=os.path.join(_ROOT, "godot_sidecar", "out", "cam"))
    ap.add_argument("--scene", default=os.path.join(_ROOT, "godot_sidecar", "out", "scenes", "haworth_spiral"))
    ap.add_argument("--out-dir", default=os.path.join(_ROOT, "godot_sidecar", "out", "panels"))
    ap.add_argument("--cell", type=int, default=512)
    a = ap.parse_args()
    build_composites(a.cam_root, a.scene, a.out_dir, cell=a.cell)
