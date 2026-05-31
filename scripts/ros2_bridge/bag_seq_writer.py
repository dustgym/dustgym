#!/usr/bin/env python3
"""Loop an `out/cam/<scene>/000..NNN/` SEQUENCE into ONE rosbag2 (MCAP) -- contract §2.5.

This is the multi-frame outer driver for the M2-slam lane. It is the N>1 generalisation of
the frozen single-frame ``bag_writer.write_bag``: where ``write_bag`` opens a Writer, registers
the §2.3 connections, and calls ``write_frame`` for ONE capture dir, this driver

  * scans ``out/cam/<scene>/`` for the zero-padded ``<NNN>`` sub-dirs (000, 001, 002, ...),
  * opens ONE rosbag2 ``Writer`` (rosbags lib, MCAP),
  * calls :func:`bag_writer.register_connections` EXACTLY ONCE (one connection set per MCAP),
  * then calls :func:`bag_writer.write_frame` per frame with a MONOTONICALLY increasing bag
    time ``t_ns`` (and matching header ``sec``/``nanosec``),

producing a single, replayable MCAP. The frame cadence (default 10 Hz) sets the inter-frame
``t_ns`` step; rtabmap's stereo-odometry consumes the resulting image/camera_info stream in
order off ``ros2 bag play`` (see ``slam_bringup.launch.py``).

WHY rosbags (not rosbag2_py): the frozen reusable core ``bag_writer.write_frame`` /
``register_connections`` is written against the pure-python ``rosbags`` Writer API
(``writer.add_connection(...)`` -> ``Connection``; ``writer.write(conn, t_ns, ts.serialize_cdr(...))``)
so the bag can be WRITTEN with no rclpy/ROS runtime. This driver REUSES that core verbatim, so it
uses the same ``rosbags`` Writer -- it does NOT re-open a parallel ``rosbag2_py`` writer (that would
duplicate the conversion seam and the connection set). The MCAP it emits is read back by the ROS
runtime (``ros2 bag info/play``) identically. See ``docs/lanes/M2-slam.md``.

The Godot(Y-up) -> ROS(Z-up, REP-103) conversion still happens exactly ONCE per frame inside
``write_frame`` via ``frames`` -- this driver adds NO new conversion. Each ``<NNN>/sensors.json``
is a full v1.1 doc carrying that frame's real ``frame_index`` + per-frame rover ``pose_in_world``;
intrinsics / ``baseline_m`` / ``extrinsic_in_base_link`` are constant and read once from frame 000
(contract §2.5), so the connection set registered from frame 000 is valid for the whole sequence.

Usage (inside the container):
    python3 bag_seq_writer.py --scene-dir fixtures            # single-frame fixture as a 1-frame seq
    python3 bag_seq_writer.py --scene-dir /data/out/cam/_smoke --out bags/_smoke
    python3 bag_seq_writer.py --scene-dir /data/out/cam/<scene> --out bags/<scene> --rate-hz 10

CC0-1.0 (see ../../LICENSE).
"""

from __future__ import annotations

import argparse
import os
import re
import sys

# Reuse the FROZEN single-frame core verbatim -- do NOT reimplement the §2.3 mapping or the
# §3 REP-103 conversion here (bag_writer owns both; this file only drives the loop).
import bag_writer

# rosbags is container-only (same lazy-import discipline as bag_writer): the host can --help /
# AST-lint this file without the dep installed in the repo .venv.
try:
    from rosbags.rosbag2 import Writer
    from rosbags.typesys import Stores, get_typestore
    try:
        from rosbags.rosbag2.enums import StoragePlugin
    except Exception:  # older layouts expose it on Writer
        StoragePlugin = getattr(Writer, "StoragePlugin", None)
    _HAVE_ROSBAGS = True
except Exception:  # noqa: BLE001
    _HAVE_ROSBAGS = False


_NNN_RE = re.compile(r"^\d{3,}$")  # zero-padded >=3-digit frame dir (contract §2.5: from 000)


def discover_frames(scene_dir: str) -> list[tuple[int, str]]:
    """Return [(frame_index, abs_dir), ...] for every ``<NNN>/`` holding a sensors.json.

    Sorted by the INTEGER value of <NNN> (so 002 < 010), giving the natural monotonic order.
    Contract §2.5: <NNN> is zero-padded 3-digit from 000, monotonically +1; we tolerate >3
    digits and any start offset but require sensors.json to be present in each candidate dir.
    """
    if not os.path.isdir(scene_dir):
        raise FileNotFoundError(f"scene dir not found: {scene_dir}")
    frames_found: list[tuple[int, str]] = []
    for name in os.listdir(scene_dir):
        if not _NNN_RE.match(name):
            continue
        sub = os.path.join(scene_dir, name)
        if os.path.isfile(os.path.join(sub, "sensors.json")):
            frames_found.append((int(name), sub))
    frames_found.sort(key=lambda t: t[0])
    return frames_found


def write_sequence(scene_dir: str, out_dir: str, rate_hz: float = 10.0,
                   start_sec: int = 0, store: str = "ros2_jazzy") -> str:
    """Loop ``scene_dir/<NNN>/`` into ONE MCAP at ``out_dir`` with monotonic timestamps.

    ONE Writer, ONE ``register_connections`` (from frame 000 -- the rig is rigid, §2.5), and
    one ``write_frame`` per frame at ``t = start_sec + i / rate_hz``. Returns ``out_dir``.
    """
    if not _HAVE_ROSBAGS:
        raise RuntimeError(
            "rosbags not importable -- run bag_seq_writer.py INSIDE the container "
            "(it is not installed in the repo .venv by design)."
        )

    frames_found = discover_frames(scene_dir)
    if not frames_found:
        raise FileNotFoundError(
            f"no <NNN>/sensors.json frames under {scene_dir} "
            f"(expected zero-padded dirs 000, 001, ... per contract §2.5)"
        )

    store_enum = {
        "ros2_jazzy": getattr(Stores, "ROS2_JAZZY", None),
        "latest": getattr(Stores, "LATEST", None),
    }.get(store) or Stores.LATEST
    ts = get_typestore(store_enum)

    # Resolve the FRONT stereo pair + baseline ONCE from frame 000 (constant across the
    # sequence per §2.5). The connection set is registered from this same pair.
    first_idx, first_dir = frames_found[0]
    sensors0 = bag_writer._load_sensors(first_dir)
    left, right, baseline = bag_writer._resolve_stereo(sensors0)

    os.makedirs(os.path.dirname(out_dir) or ".", exist_ok=True)
    if os.path.exists(out_dir):
        raise FileExistsError(f"{out_dir} exists; remove it or choose another --out")

    step_ns = int(round(1e9 / rate_hz)) if rate_hz > 0 else 0
    base_ns = int(start_sec) * 1_000_000_000

    kwargs = {"version": 9}
    if StoragePlugin is not None:
        kwargs["storage_plugin"] = StoragePlugin.MCAP
    writer = Writer(out_dir, **kwargs)
    writer.open()
    try:
        # ONE connection set for the whole MCAP (contract / register_connections docstring).
        conns = bag_writer.register_connections(writer, ts, left, right)

        prev_t = -1
        for i, (frame_index, in_dir) in enumerate(frames_found):
            # Each frame carries its OWN sensors.json (per-frame rover pose); re-read it so the
            # moving state is honoured. Intrinsics/baseline are constant (validated below).
            sensors = bag_writer._load_sensors(in_dir)
            f_left, f_right, f_baseline = bag_writer._resolve_stereo(sensors)
            # Guard the §2.5 "constant across frames" invariant: the connection set was
            # registered from frame 000's cameras, so the per-frame topic names MUST match.
            if (f_left["name"], f_right["name"]) != (left["name"], right["name"]):
                raise ValueError(
                    f"{in_dir}: stereo pair "
                    f"({f_left['name']},{f_right['name']}) != frame-000 "
                    f"({left['name']},{right['name']}); a single MCAP needs one connection set"
                )
            if abs(f_baseline - baseline) > 1e-9:
                print(f"WARNING: {in_dir} baseline_m={f_baseline} != frame-000 {baseline} "
                      f"(§2.5 says baseline is constant); using per-frame value for P[3]",
                      file=sys.stderr)

            t_ns = base_ns + i * step_ns
            if t_ns <= prev_t:  # strictly monotonic -- the SLAM odom front-end relies on it
                t_ns = prev_t + 1
            prev_t = t_ns
            sec, nanosec = divmod(t_ns, 1_000_000_000)

            seq_idx = sensors.get("frame_index", frame_index)
            print(f"[{i:03d}] dir={os.path.basename(in_dir)} frame_index={seq_idx} "
                  f"t={t_ns / 1e9:.3f}s", flush=True)
            bag_writer.write_frame(
                writer, ts, conns, in_dir, sensors, f_left, f_right, f_baseline,
                t_ns, int(sec), int(nanosec),
            )
    finally:
        writer.close()

    print(f"wrote {len(frames_found)}-frame rosbag2 (MCAP) -> {out_dir} "
          f"(monotonic {base_ns / 1e9:.3f}s .. {prev_t / 1e9:.3f}s @ {rate_hz} Hz)")
    return out_dir


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene-dir", required=True,
                    help="dir holding the <NNN>/ frame sub-dirs "
                         "(e.g. /data/out/cam/<scene> or fixtures)")
    ap.add_argument("--out", dest="out_dir", default=None,
                    help="output rosbag2 dir (must not exist). "
                         "Default: bags/<basename(scene-dir)>_seq")
    ap.add_argument("--rate-hz", type=float, default=10.0,
                    help="inter-frame cadence for the monotonic bag timestamps (default 10)")
    ap.add_argument("--start-sec", type=int, default=0,
                    help="bag time of frame 0, whole seconds (default 0)")
    ap.add_argument("--store", default="ros2_jazzy",
                    help="rosbags typestore: ros2_jazzy|latest (default ros2_jazzy)")
    args = ap.parse_args()

    out_dir = args.out_dir
    if out_dir is None:
        base = os.path.basename(os.path.normpath(args.scene_dir)) or "scene"
        out_dir = os.path.join("bags", f"{base}_seq")

    write_sequence(args.scene_dir, out_dir, rate_hz=args.rate_hz,
                   start_sec=args.start_sec, store=args.store)


if __name__ == "__main__":
    main()
