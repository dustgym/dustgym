"""Turn an `out/cam/<scene>/<NNN>/` capture into a rosbag2 (MCAP) -- sensor_bridge_contract §2.3.

Reads a directory holding ``sensors.json`` + the two camera PNGs (the G1 egress, or the
``fixtures/000/`` stand-in) and writes a rosbag2 in **MCAP** format using the pure-Python
``rosbags`` library -- NO rclpy / ROS install needed to *write* (the conversion is the seam,
not the runtime).  Topics emitted (contract §2.3):

  /front_left/image_raw    sensor_msgs/Image       (left PNG, mono8 or rgb8)
  /front_left/camera_info  sensor_msgs/CameraInfo  (intrinsics; P[3]=0 on the left)
  /front_right/image_raw   sensor_msgs/Image
  /front_right/camera_info sensor_msgs/CameraInfo  (P[3] = -fx*baseline_m  <-- the stereo term)
  /tf                      tf2_msgs/TFMessage      (map -> base_link, the rover pose)
  /tf_static               tf2_msgs/TFMessage      (base_link -> *_optical; map -> lander)
  /lander/apriltag_truth   geometry_msgs/PoseStamped
                           = the computed camera->tag ground truth in the LEFT optical frame,
                             inv(T_map_leftoptical) @ T_map_lander, AFTER the §3 conversion.

The Godot(Y-up) -> ROS(Z-up, REP-103) conversion happens exactly ONCE, here, via ``frames``.
``sensors.json`` is 100% Godot-native (contract §3).

Usage (inside the container):
    python3 bag_writer.py --in fixtures/000 --out bags/fixture_000
    python3 bag_writer.py --in /data/out/cam/<scene>/000 --out bags/<scene>_000

CC0-1.0 (see ../../LICENSE).
"""

from __future__ import annotations

import argparse
import json
import os
import struct
import sys
import zlib

import numpy as np

import frames

# rosbags is container-only (do NOT pip-install into the repo .venv).  Import lazily so the
# host can still --help / lint this file without the dep.
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


# --- minimal PNG reader (stdlib): supports 8-bit grayscale (ct 0) & RGB/RGBA (ct 2/6) -------

def _read_png(path: str):
    """Return (width, height, channels, bytes) for an 8-bit PNG.  channels in {1,3}."""
    d = open(path, "rb").read()
    assert d[:8] == b"\x89PNG\r\n\x1a\n", f"{path}: not a PNG"
    i, w, h, ct = 8, 0, 0, 0
    idat = b""
    while i < len(d):
        ln = struct.unpack(">I", d[i:i + 4])[0]
        typ = d[i + 4:i + 8]
        body = d[i + 8:i + 8 + ln]
        i += 12 + ln
        if typ == b"IHDR":
            w, h, bitdepth, ct = struct.unpack(">IIBB", body[:10])
            assert bitdepth == 8, f"{path}: only 8-bit PNG supported (got {bitdepth})"
        elif typ == b"IDAT":
            idat += body
        elif typ == b"IEND":
            break
    samples = {0: 1, 2: 3, 6: 4}[ct]
    raw = zlib.decompress(idat)
    stride = 1 + w * samples
    bpp = samples
    prev = bytearray(w * samples)
    out = bytearray()

    def paeth(a, b, c):
        p = a + b - c
        pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
        return a if pa <= pb and pa <= pc else (b if pb <= pc else c)

    for r in range(h):
        f = raw[r * stride]
        line = bytearray(raw[r * stride + 1:r * stride + 1 + w * samples])
        for x in range(len(line)):
            a = line[x - bpp] if x >= bpp else 0
            b = prev[x]
            c = prev[x - bpp] if x >= bpp else 0
            if f == 1:
                line[x] = (line[x] + a) & 255
            elif f == 2:
                line[x] = (line[x] + b) & 255
            elif f == 3:
                line[x] = (line[x] + ((a + b) >> 1)) & 255
            elif f == 4:
                line[x] = (line[x] + paeth(a, b, c)) & 255
        prev = line
        out.extend(line)
    if ct == 6:  # drop alpha -> RGB
        arr = np.frombuffer(bytes(out), dtype=np.uint8).reshape(h, w, 4)[:, :, :3]
        return w, h, 3, arr.tobytes()
    return w, h, samples, bytes(out)


# --- message builders (typestore-bound) -------------------------------------------------

def _stamp(ts, sec, nanosec):
    return ts.types["builtin_interfaces/msg/Time"](sec=sec, nanosec=nanosec)


def _header(ts, sec, nanosec, frame_id):
    return ts.types["std_msgs/msg/Header"](
        stamp=_stamp(ts, sec, nanosec), frame_id=frame_id
    )


def _image_msg(ts, sec, nanosec, frame_id, w, h, channels, data):
    encoding = "mono8" if channels == 1 else "rgb8"
    return ts.types["sensor_msgs/msg/Image"](
        header=_header(ts, sec, nanosec, frame_id),
        height=h,
        width=w,
        encoding=encoding,
        is_bigendian=0,
        step=w * channels,
        data=np.frombuffer(data, dtype=np.uint8),
    )


def _camera_info(ts, sec, nanosec, frame_id, w, h, fx, fy, cx, cy, d_vec, p3):
    K = np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1], dtype=np.float64)
    R = np.eye(3, dtype=np.float64).reshape(9)
    # Projection: rectified pinhole; P[3] carries the stereo baseline term on the right cam.
    P = np.array([fx, 0, cx, p3, 0, fy, cy, 0, 0, 0, 1, 0], dtype=np.float64)
    return ts.types["sensor_msgs/msg/CameraInfo"](
        header=_header(ts, sec, nanosec, frame_id),
        height=h,
        width=w,
        distortion_model="plumb_bob",
        d=np.asarray(d_vec, dtype=np.float64),
        k=K,
        r=R,
        p=P,
        binning_x=0,
        binning_y=0,
        roi=ts.types["sensor_msgs/msg/RegionOfInterest"](
            x_offset=0, y_offset=0, height=0, width=0, do_rectify=False
        ),
    )


def _tf_msg(ts, transforms):
    return ts.types["tf2_msgs/msg/TFMessage"](transforms=transforms)


def _transform_stamped(ts, sec, nanosec, parent, child, pos, quat_xyzw):
    return ts.types["geometry_msgs/msg/TransformStamped"](
        header=_header(ts, sec, nanosec, parent),
        child_frame_id=child,
        transform=ts.types["geometry_msgs/msg/Transform"](
            translation=ts.types["geometry_msgs/msg/Vector3"](
                x=float(pos[0]), y=float(pos[1]), z=float(pos[2])
            ),
            rotation=ts.types["geometry_msgs/msg/Quaternion"](
                x=float(quat_xyzw[0]), y=float(quat_xyzw[1]),
                z=float(quat_xyzw[2]), w=float(quat_xyzw[3])
            ),
        ),
    )


def _pose_stamped(ts, sec, nanosec, frame_id, pos, quat_xyzw):
    return ts.types["geometry_msgs/msg/PoseStamped"](
        header=_header(ts, sec, nanosec, frame_id),
        pose=ts.types["geometry_msgs/msg/Pose"](
            position=ts.types["geometry_msgs/msg/Point"](
                x=float(pos[0]), y=float(pos[1]), z=float(pos[2])
            ),
            orientation=ts.types["geometry_msgs/msg/Quaternion"](
                x=float(quat_xyzw[0]), y=float(quat_xyzw[1]),
                z=float(quat_xyzw[2]), w=float(quat_xyzw[3])
            ),
        ),
    )


# --- conversion helpers -----------------------------------------------------------------

def _compute_truth(sensors, left_cam):
    """camera(left optical) -> tag ground truth = inv(T_map_optical) @ T_map_lander, in ROS.

    All inputs are Godot-frame; convert via §3 first.  The lander pose is converted as a
    world-attached pose (Map 1); the camera as a world camera pose -> optical (Map 1 + Map 2);
    the tag's pose_in_lander is identity for M1 (contract §1) so T_map_tag == T_map_lander.
    Returns (position xyz, quaternion xyzw) of the tag IN the left optical frame.
    """
    lp = sensors["lander"]
    lpos, lquat = frames.godot_world_pose_to_ros(
        lp["position_m"], lp["quaternion_xyzw"]
    )
    T_map_lander = frames.make_transform(lpos, lquat)

    # tag pose within lander (identity for M1, but honour the field generically).
    tag = lp["apriltag"]["pose_in_lander"]
    # pose_in_lander is a body-frame transform inside the lander's own (Godot) axes; the lander
    # frame after conversion is a ROS frame, so we convert this sub-pose by the same world map.
    tpos, tquat = frames.godot_world_pose_to_ros(
        tag["position_m"], tag["quaternion_xyzw"]
    )
    T_lander_tag = frames.make_transform(tpos, tquat)
    # Relabel the tag's OWN-FRAME axes from the contract-§1 lander convention (+X = outward
    # normal, +Y = up) into the apriltag_ros `pnp` detector convention (+X image-right,
    # +Y image-up, +Z outward-normal-toward-camera).  This is a FIXED rotation
    # (frames.R_LANDER_TAG) derived from the lander/QuadMesh axis definitions and pinned to the
    # detector's fronto-parallel reading, right-multiplied so it acts purely on the tag's own
    # frame -- pose_in_lander stays the contract's identity and the TRANSLATION (tag centre ==
    # lander origin) is untouched.  Without it /lander/apriltag_truth is off by a fixed ~120 deg
    # axis-permutation vs the detector (the historical q=[.5,.5,-.5,.5] / 124.6 deg error).
    T_relabel = np.eye(4, dtype=np.float64)
    T_relabel[:3, :3] = frames.R_LANDER_TAG
    T_lander_tag = T_lander_tag @ T_relabel
    T_map_tag = T_map_lander @ T_lander_tag

    cpos, cquat = frames.godot_world_cam_pose_to_ros_optical(
        left_cam["pose_in_world"]["position_m"],
        left_cam["pose_in_world"]["quaternion_xyzw"],
    )
    T_map_optical = frames.make_transform(cpos, cquat)
    T_optical_tag = np.linalg.inv(T_map_optical) @ T_map_tag
    return frames.transform_to_pos_quat(T_optical_tag)


def _load_sensors(in_dir: str):
    """Read + validate sensors.json from a capture dir; return the parsed dict."""
    sensors = json.load(open(os.path.join(in_dir, "sensors.json")))
    assert sensors["schema_version"].startswith("sensor_bridge/1."), \
        f"unexpected schema_version {sensors['schema_version']!r}"
    assert sensors.get("frame_convention") == "godot", \
        "sensors.json must be Godot-native (frame_convention=='godot')"
    return sensors


def _resolve_stereo(sensors):
    """Resolve the FRONT stereo pair BY NAME (contract §2.2: 'stereo' always carries it).

    Returns (left_cam, right_cam, baseline_m).  The stereo reader resolves cams via
    sensors['stereo']['left'/'right'] by name -- the front pair is always present per
    the frozen contract, so multi-frame egress can rely on it identically per frame.
    """
    cams = {c["name"]: c for c in sensors["cameras"]}
    left = cams[sensors["stereo"]["left"]]
    right = cams[sensors["stereo"]["right"]]
    baseline = float(sensors["stereo"]["baseline_m"])
    # sanity: baseline must match the metric L<->R separation (contract §2.2 rule).
    sep = np.linalg.norm(
        np.array(left["extrinsic_in_base_link"]["position_m"])
        - np.array(right["extrinsic_in_base_link"]["position_m"])
    )
    if abs(sep - baseline) > 1e-6:
        print(f"WARNING: baseline_m={baseline} != |L-R| extrinsic sep={sep:.6f}",
              file=sys.stderr)
    return left, right, baseline


def _msgtypes(ts):
    """The four __msgtype__ constants the topics use (resolved once per typestore)."""
    return {
        "IMG": ts.types["sensor_msgs/msg/Image"].__msgtype__,
        "CINFO": ts.types["sensor_msgs/msg/CameraInfo"].__msgtype__,
        "TFM": ts.types["tf2_msgs/msg/TFMessage"].__msgtype__,
        "POSE": ts.types["geometry_msgs/msg/PoseStamped"].__msgtype__,
    }


def register_connections(writer, ts, left, right):
    """Register the §2.3 topic connections on an ALREADY-OPEN Writer -- call ONCE.

    A single MCAP carries ONE connection set; when the per-frame core is looped over many
    frames on one open Writer (the future bag_seq_writer.py / M2-slam path) the topics are
    registered here exactly once, NOT per frame.  Returns the ``conns`` dict (topic -> id)
    that :func:`write_frame` then writes into.
    """
    mt = _msgtypes(ts)
    conns = {}

    def conn(topic, msgtype):
        conns[topic] = writer.add_connection(topic, msgtype, typestore=ts)

    for c in (left, right):
        conn(f"/{c['name']}/image_raw", mt["IMG"])
        conn(f"/{c['name']}/camera_info", mt["CINFO"])
    conn("/tf", mt["TFM"])
    conn("/tf_static", mt["TFM"])
    conn("/lander/apriltag_truth", mt["POSE"])
    return conns


def write_frame(writer, ts, conns, in_dir, sensors, left, right, baseline,
                t_ns, sec, nanosec):
    """Write ONE frame's §2.3 topics onto an ALREADY-OPEN Writer at bag time ``t_ns``.

    REUSABLE CORE.  Does NOT open/close the Writer and does NOT register connections or
    raise FileExistsError -- the caller owns the Writer lifecycle + connection registration
    (see :func:`register_connections`).  ``t_ns`` is the explicit bag write time (int ns);
    ``sec``/``nanosec`` populate the message header stamps.  A multi-frame caller advances
    ``t_ns`` (and the matching stamp) monotonically across frames on one open Writer => one
    MCAP, one connection set.

    The Godot(Y-up) -> ROS(Z-up, REP-103) conversion happens via ``frames`` exactly here.
    """
    mt = _msgtypes(ts)
    IMG, CINFO, TFM, POSE = mt["IMG"], mt["CINFO"], mt["TFM"], mt["POSE"]

    # --- images + camera_info (left P[3]=0, right P[3]=-fx*baseline) --------------------
    for c, is_right in ((left, False), (right, True)):
        w, h, ch, data = _read_png(os.path.join(in_dir, c["image"]))
        assert (w, h) == (c["width"], c["height"]), \
            f"{c['image']} dims {(w, h)} != sensors.json {(c['width'], c['height'])}"
        intr = c["intrinsics"]
        fx, fy, cx, cy = intr["fx"], intr["fy"], intr["cx"], intr["cy"]
        p3 = -fx * baseline if is_right else 0.0
        img = _image_msg(ts, sec, nanosec, c["frame_id"], w, h, ch, data)
        info = _camera_info(ts, sec, nanosec, c["frame_id"], w, h,
                            fx, fy, cx, cy, intr["D"], p3)
        writer.write(conns[f"/{c['name']}/image_raw"], t_ns,
                     ts.serialize_cdr(img, IMG))
        writer.write(conns[f"/{c['name']}/camera_info"], t_ns,
                     ts.serialize_cdr(info, CINFO))

    # --- /tf : map -> base_link (rover pose, converted) ---------------------------------
    rover = sensors["rover"]
    rpos, rquat = frames.godot_world_pose_to_ros(
        rover["position_m"], rover["quaternion_xyzw"]
    )
    tf_dyn = _tf_msg(ts, [_transform_stamped(
        ts, sec, nanosec, "map", rover["frame_id"], rpos, rquat)])
    writer.write(conns["/tf"], t_ns, ts.serialize_cdr(tf_dyn, TFM))

    # --- /tf_static : base_link -> *_optical, map -> lander -----------------------------
    static_tfs = []
    for c in (left, right):
        epos, equat = frames.godot_cam_extrinsic_to_ros_optical(
            c["extrinsic_in_base_link"]["position_m"],
            c["extrinsic_in_base_link"]["quaternion_xyzw"],
        )
        static_tfs.append(_transform_stamped(
            ts, sec, nanosec, rover["frame_id"], c["frame_id"], epos, equat))
    lpos, lquat = frames.godot_world_pose_to_ros(
        sensors["lander"]["position_m"], sensors["lander"]["quaternion_xyzw"]
    )
    static_tfs.append(_transform_stamped(
        ts, sec, nanosec, "map", sensors["lander"]["frame_id"], lpos, lquat))
    writer.write(conns["/tf_static"], t_ns,
                 ts.serialize_cdr(_tf_msg(ts, static_tfs), TFM))

    # --- /lander/apriltag_truth : camera(left optical) -> tag, computed ----------------
    truth_pos, truth_quat = _compute_truth(sensors, left)
    truth = _pose_stamped(ts, sec, nanosec, left["frame_id"], truth_pos, truth_quat)
    writer.write(conns["/lander/apriltag_truth"], t_ns,
                 ts.serialize_cdr(truth, POSE))
    print(f"apriltag_truth (left optical -> tag): pos={np.round(truth_pos, 4).tolist()} "
          f"quat_xyzw={np.round(truth_quat, 4).tolist()}")


def write_bag(in_dir: str, out_dir: str, sec: int = 0, nanosec: int = 0,
              store: str = "ros2_jazzy") -> str:
    if not _HAVE_ROSBAGS:
        raise RuntimeError(
            "rosbags not importable -- run bag_writer.py INSIDE the container "
            "(it is not installed in the repo .venv by design)."
        )
    sensors = _load_sensors(in_dir)

    store_enum = {
        "ros2_jazzy": getattr(Stores, "ROS2_JAZZY", None),
        "latest": getattr(Stores, "LATEST", None),
    }.get(store) or Stores.LATEST
    ts = get_typestore(store_enum)

    left, right, baseline = _resolve_stereo(sensors)

    os.makedirs(os.path.dirname(out_dir) or ".", exist_ok=True)
    if os.path.exists(out_dir):
        raise FileExistsError(f"{out_dir} exists; remove it or choose another --out")

    kwargs = {"version": 9}
    if StoragePlugin is not None:
        kwargs["storage_plugin"] = StoragePlugin.MCAP
    writer = Writer(out_dir, **kwargs)
    writer.open()
    try:
        conns = register_connections(writer, ts, left, right)
        t_ns = sec * 1_000_000_000 + nanosec
        write_frame(writer, ts, conns, in_dir, sensors, left, right, baseline,
                    t_ns, sec, nanosec)
    finally:
        writer.close()

    print(f"wrote rosbag2 (MCAP) -> {out_dir}")
    return out_dir


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--in", dest="in_dir", required=True,
                    help="input capture dir (sensors.json + 2 PNGs)")
    ap.add_argument("--out", dest="out_dir", required=True,
                    help="output rosbag2 dir (must not exist)")
    ap.add_argument("--store", default="ros2_jazzy",
                    help="rosbags typestore: ros2_jazzy|latest (default ros2_jazzy)")
    args = ap.parse_args()
    write_bag(args.in_dir, args.out_dir, store=args.store)


if __name__ == "__main__":
    main()
