"""Batch Godot stereo imagery egress along a traverse + CFDP-style downlink (host GPU step).

Consumes a completed traverse (the ground station's telemetry.json) and, per leg, renders a moving
stereo sequence at the leg endpoint under the mission Sun (via the shared camera_render path), then
"downlinks" each frame as a real CCSDS Img packet. The live HITL console uses the same camera_render
module for on-demand capture. Follows the repo convention that the GPU render runs on the host.

    python scripts/ccsds_ros_nav/render/render_egress.py --telemetry out/ccsds_nav/telemetry.json
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import sys

_PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # scripts/ccsds_ros_nav
_REPO = os.path.abspath(os.path.join(_PKG, "..", ".."))
for _p in (_PKG, _REPO):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import camera_render
import messages
import mission_clock as mc
from flight import load_crop


def _leg_anchors(telemetry: dict) -> list[tuple[int, tuple[int, int]]]:
    """Per-leg (leg_id, endpoint rc) from the recorded poses."""
    last: dict[int, tuple[float, float]] = {}
    for p in telemetry["poses"]:
        last[int(p["leg_id"])] = (p["row"], p["col"])
    return [(lid, (int(round(rc[0])), int(round(rc[1])))) for lid, rc in sorted(last.items())]


def main() -> int:
    ap = argparse.ArgumentParser(description="Godot stereo egress + CFDP-style downlink for a traverse")
    ap.add_argument("--telemetry", default="out/ccsds_nav/telemetry.json")
    ap.add_argument("--scene", default="samples/lunar_dem/haworth_10km_5m")
    ap.add_argument("--r0", type=int, default=720)
    ap.add_argument("--c0", type=int, default=1800)
    ap.add_argument("--win", type=int, default=160)
    ap.add_argument("--frames-per-leg", type=int, default=6)
    ap.add_argument("--size", default="1024x768")
    ap.add_argument("--sun-el", type=float, default=8.0, help="sun elevation deg (grazing; raise for visibility)")
    ap.add_argument("--legs", default="all", help="'all' or comma-separated leg ids to render")
    ap.add_argument("--timeout", type=float, default=180.0)
    ap.add_argument("--out", default="out/ccsds_nav")
    args = ap.parse_args()

    haworth = args.scene if os.path.isabs(args.scene) else os.path.join(_REPO, args.scene)
    telem_path = args.telemetry if os.path.isabs(args.telemetry) else os.path.join(_REPO, args.telemetry)
    with open(telem_path) as fh:
        telemetry = json.load(fh)

    crop = load_crop(haworth, args.r0, args.c0, args.win, args.win)
    sun_az, lit = mc.find_illuminated_start(crop.heightmap, crop.cell_m, el_deg=args.sun_el)
    print(f"[render] sun az={sun_az:.0f} el={args.sun_el} ({lit*100:.0f}% lit start)")
    anchors = _leg_anchors(telemetry)
    if args.legs != "all":
        want = {int(x) for x in args.legs.split(",")}
        anchors = [a for a in anchors if a[0] in want]

    out = args.out if os.path.isabs(args.out) else os.path.join(_REPO, args.out)
    downlink_dir = os.path.join(out, "downlink")
    os.makedirs(downlink_dir, exist_ok=True)

    manifest: list[dict] = []
    img_packets = 0
    for leg_id, anchor in anchors:
        scene_name = f"ccsds_nav_leg{leg_id:02d}"
        cap = camera_render.render_capture(_REPO, crop, anchor, sun_az=sun_az, sun_el=args.sun_el,
                                           scene_name=scene_name, haworth_dir=haworth,
                                           frames=args.frames_per_leg, size=args.size, timeout=args.timeout)
        if not cap["ok"]:
            print(f"[render] leg {leg_id}: render failed; skipping\n{cap['log'][-600:]}")
            continue
        for fr in cap["frames"]:
            name = f"{scene_name}/{fr['frame']:03d}_{fr['camera']}.png"
            dst = os.path.join(downlink_dir, name.replace("/", "_"))
            shutil.copyfile(fr["path"], dst)
            size_bytes = os.path.getsize(dst)
            img = messages.Img(leg_id=leg_id, frame_index=fr["frame"], width=fr["width"],
                               height=fr["height"], size_bytes=size_bytes, name=name)
            pkt = messages.encode(img, seq_count=img_packets % 0x4000, met=float(fr["frame"]))
            assert messages.decode(pkt) == img                      # the packet really round-trips
            img_packets += 1
            manifest.append({"leg_id": leg_id, "frame": fr["frame"], "camera": fr["camera"],
                             "file": os.path.basename(dst), "size_bytes": size_bytes,
                             "apid": f"0x{messages.APID_TLM_IMG:03X}"})

    with open(os.path.join(out, "img_manifest.json"), "w") as fh:
        json.dump({"images": len(manifest), "img_packets": img_packets, "manifest": manifest}, fh, indent=2)
    print(f"\n[render] downlinked {len(manifest)} frames as {img_packets} CCSDS Img packets -> {downlink_dir}")
    return 0 if manifest else 1


if __name__ == "__main__":
    raise SystemExit(main())
