"""Detected-vs-truth pose error node -- sensor_bridge_contract.md §5.2 (the spec §10 channel).

Subscribes to:
  * the apriltag detector's `/tf`  -- looks for the transform whose child_frame_id is the tag
    (default "tag36h11:0"), parent = the camera optical frame.  This is the DETECTED
    camera->tag pose.
  * `/lander/apriltag_truth` (geometry_msgs/PoseStamped) -- the bag_writer's computed
    camera->tag GROUND TRUTH in the same optical frame.

Prints translation error (m) and rotation error (deg) between detected and truth.  The number
this prints is the spec §10 pose-error channel's first real reading (contract §5).

Run in the container alongside the detector + `ros2 bag play` (see README).  Pure rclpy; needs
the ROS install (so it runs IN the container, not on the host / not in the repo .venv).

CC0-1.0 (see ../../LICENSE).
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from tf2_msgs.msg import TFMessage
    _HAVE_RCLPY = True
except Exception:  # noqa: BLE001
    _HAVE_RCLPY = False


def _quat_to_mat(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n
    return np.array(
        [[1 - (y * y + z * z) * s, (x * y - w * z) * s, (x * z + w * y) * s],
         [(x * y + w * z) * s, 1 - (x * x + z * z) * s, (y * z - w * x) * s],
         [(x * z - w * y) * s, (y * z + w * x) * s, 1 - (x * x + y * y) * s]],
        dtype=np.float64,
    )


def rotation_error_deg(qa, qb) -> float:
    """Geodesic angle (deg) between two orientations given as (x,y,z,w)."""
    ra = _quat_to_mat(*qa)
    rb = _quat_to_mat(*qb)
    r = ra.T @ rb
    cos = max(-1.0, min(1.0, (np.trace(r) - 1.0) / 2.0))
    return float(np.degrees(np.arccos(cos)))


class ComparePose(Node):
    def __init__(self, tag_frame: str, timeout_s: float):
        super().__init__("compare_pose")
        self.tag_frame = tag_frame
        self.detected = None
        self.truth = None
        self.create_subscription(TFMessage, "/tf", self._on_tf, 50)
        self.create_subscription(
            PoseStamped, "/lander/apriltag_truth", self._on_truth, 10)
        self._printed = False
        self.create_timer(timeout_s, self._on_timeout)
        self.get_logger().info(
            f"waiting for detection (/tf child '{tag_frame}') and /lander/apriltag_truth ...")

    def _on_truth(self, msg: "PoseStamped"):
        p, o = msg.pose.position, msg.pose.orientation
        self.truth = (np.array([p.x, p.y, p.z]), np.array([o.x, o.y, o.z, o.w]))
        self._maybe_report()

    def _on_tf(self, msg: "TFMessage"):
        for t in msg.transforms:
            if t.child_frame_id == self.tag_frame:
                tr, ro = t.transform.translation, t.transform.rotation
                self.detected = (np.array([tr.x, tr.y, tr.z]),
                                 np.array([ro.x, ro.y, ro.z, ro.w]))
                self.get_logger().info(
                    f"DETECTED tag '{self.tag_frame}' in frame '{t.header.frame_id}': "
                    f"t={np.round(self.detected[0], 4).tolist()}")
                self._maybe_report()

    def _maybe_report(self):
        if self._printed or self.detected is None or self.truth is None:
            return
        self._printed = True
        dt, dq = self.detected
        tt, tq = self.truth
        terr = float(np.linalg.norm(dt - tt))
        rerr = rotation_error_deg(dq, tq)
        print("\n=== camera->tag pose: DETECTED vs TRUTH ===")
        print(f"  detected t = {np.round(dt, 4).tolist()}  q = {np.round(dq, 4).tolist()}")
        print(f"  truth    t = {np.round(tt, 4).tolist()}  q = {np.round(tq, 4).tolist()}")
        print(f"  translation error = {terr * 1000:.1f} mm")
        print(f"  rotation    error = {rerr:.2f} deg")
        print("===========================================\n")

    def _on_timeout(self):
        if not self._printed:
            missing = []
            if self.detected is None:
                missing.append(f"detection (/tf child '{self.tag_frame}')")
            if self.truth is None:
                missing.append("/lander/apriltag_truth")
            self.get_logger().warn("timeout; still missing: " + ", ".join(missing))
        rclpy.shutdown()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tag-frame", default="tag36h11:0",
                    help="child_frame_id the detector publishes (default tag36h11:0)")
    ap.add_argument("--timeout", type=float, default=15.0,
                    help="seconds to wait for both inputs before giving up")
    args = ap.parse_args(argv)

    if not _HAVE_RCLPY:
        print("compare_pose.py needs rclpy -- run it INSIDE the container.", file=sys.stderr)
        return 1
    rclpy.init()
    node = ComparePose(args.tag_frame, args.timeout)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
