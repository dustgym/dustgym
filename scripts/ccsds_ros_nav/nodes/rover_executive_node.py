"""ROS 2 (rclpy) rover-executive node — the in-container binding of the FlightModel.

Subscribes to nav goals (the full CCSDS GoTo as JSON, from the bridge), runs the onboard pure-pursuit
controller, and publishes native ROS telemetry: ``/tf`` (map->base_link), ``/odom``, a decimated
``/rover/state`` (the CCSDS Pose + MET as JSON), and ``/rover/leg`` at leg completion. A wall-clock
timer advances the deterministic physics one ``dt`` step per fire; ``tick_period`` is decoupled from
``dt`` so the sim can run faster than real time (a 586 m traverse otherwise takes ~30 min at 0.2 s/tick).

rclpy + the message packages import at module load, so this is only imported inside the ROS container
(tests guard it with ``importorskip('rclpy')``).
"""
from __future__ import annotations

import json
import math
import os
import sys

_PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_REPO = os.path.abspath(os.path.join(_PKG, "..", ".."))
for _p in (_PKG, _REPO):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Float64, String
from tf2_ros import TransformBroadcaster

import messages
from flight import FlightModel, load_crop
from route import slope_deg, snap_to_navigable


def _quat_z(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class RoverExecutive(Node):
    def __init__(self) -> None:
        super().__init__("rover_executive")
        self.declare_parameter("scene", os.path.join(_REPO, "samples", "lunar_dem", "haworth_10km_5m"))
        self.declare_parameter("r0", 720)
        self.declare_parameter("c0", 1800)
        self.declare_parameter("win", 160)
        self.declare_parameter("body", "moon")
        self.declare_parameter("start_row", 47.0)
        self.declare_parameter("start_col", 56.0)
        self.declare_parameter("max_slope_deg", 18.0)
        self.declare_parameter("dt", 0.2)              # physics step [s]
        self.declare_parameter("time_factor", 1.0)     # sim seconds per wall second (timer = dt/time_factor)
        self.declare_parameter("v_max", 0.3)
        self.declare_parameter("goal_radius_cells", 1.0)
        self.declare_parameter("downlink_decim", 5)

        g = self.get_parameter
        crop = load_crop(g("scene").value, int(g("r0").value), int(g("c0").value),
                         int(g("win").value), int(g("win").value))
        self.cell_m = crop.cell_m
        # snap the boot pose to navigable terrain the SAME way the ground station plans from, so the
        # planned route and the rover's start agree without an extra handshake.
        sl = slope_deg(crop.heightmap, crop.cell_m)
        sr, sc = snap_to_navigable(sl, (int(g("start_row").value), int(g("start_col").value)),
                                   float(g("max_slope_deg").value))
        self.fm = FlightModel(crop=crop, start_rc=(float(sr), float(sc)),
                              body=g("body").value, dt=float(g("dt").value))
        self.v_max = float(g("v_max").value)
        self.radius = float(g("goal_radius_cells").value)
        self._decim = max(1, int(g("downlink_decim").value))

        self._goal: tuple[float, float] | None = None
        self._leg_id = 0
        self._commanded = self._achieved = self._energy0 = 0.0
        self._prev_yaw: float | None = None
        self._tick_count = 0

        self._dt = float(g("dt").value)
        self._time_factor = max(1e-3, float(g("time_factor").value))

        self.create_subscription(String, "/cmd/nav_goal", self._on_goal, 10)
        self.create_subscription(Empty, "/cmd/safe", self._on_safe, 10)
        self.create_subscription(Float64, "/sim/time_factor", self._on_time_factor, 10)
        self._pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self._pub_state = self.create_publisher(String, "/rover/state", 50)
        self._pub_leg = self.create_publisher(String, "/rover/leg", 10)
        self._tf = TransformBroadcaster(self)
        self._timer = self.create_timer(self._dt / self._time_factor, self._tick)
        self.get_logger().info(f"rover_executive up: body={g('body').value} crop {int(g('win').value)}@"
                               f"{self.cell_m}m start=({sr},{sc}) dt={self._dt} "
                               f"time_factor={self._time_factor} (timer {self._dt/self._time_factor:.3f}s)")

    def _on_time_factor(self, msg: Float64) -> None:
        tf = max(1e-3, float(msg.data))
        self._time_factor = tf
        self._timer.cancel()                              # retime the drive loop to the new acceleration
        self._timer = self.create_timer(self._dt / tf, self._tick)
        self.get_logger().info(f"time_factor -> {tf} (timer {self._dt/tf:.3f}s)")

    def _on_goal(self, msg: String) -> None:
        cmd = messages.GoTo(**json.loads(msg.data))
        self._goal = (float(cmd.goal_row), float(cmd.goal_col))
        self.v_max = float(cmd.v_max_mps) if cmd.v_max_mps > 0 else self.v_max
        self.radius = float(cmd.goal_radius_cells)        # honor the commanded arrival tolerance
        self._leg_id = int(cmd.leg_id)
        self._commanded = self._achieved = 0.0
        self._energy0 = self.fm.energy_j
        self._prev_yaw = None
        self._tick_count = 0
        self.get_logger().info(f"leg {self._leg_id}: GoTo ({self._goal[0]:.1f},{self._goal[1]:.1f}) "
                               f"r={self.radius}")

    def _on_safe(self, _msg: Empty) -> None:
        if self._goal is not None:
            self._publish_leg(messages.LEG_SAFED)
        self._goal = None
        self.get_logger().warn("SAFE: leg aborted")

    def _tick(self) -> None:
        if self._goal is None:
            return
        p, done, status, sc, sa = self.fm.step_toward(self._goal, self.v_max, self.radius,
                                                      leg_id=self._leg_id)
        if p is None:                                     # already within radius
            self._publish_leg(messages.LEG_REACHED)
            self._goal = None
            return
        self._commanded += sc
        self._achieved += sa
        self._tick_count += 1
        if done or (self._tick_count % self._decim == 0):
            self._publish_pose(p)
        if done:
            self._publish_leg(status if status is not None else messages.LEG_MAX_STEPS)
            self._goal = None

    def _publish_pose(self, p: messages.Pose) -> None:
        now = self.get_clock().now().to_msg()
        # grid -> REP-103 right-handed map frame: x=East(col), y=North(-row), z=up(height); yaw_ros=-yaw.
        x = p.col * self.cell_m
        y = -p.row * self.cell_m
        z = self.fm._height_at((p.row, p.col))
        qx, qy, qz, qw = _quat_z(-p.yaw_rad)

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = "map"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = x, y, z
        tf.transform.rotation.x, tf.transform.rotation.y = qx, qy
        tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
        self._tf.sendTransform(tf)

        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = "map"
        od.child_frame_id = "base_link"
        od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = x, y, z
        od.pose.pose.orientation.x, od.pose.pose.orientation.y = qx, qy
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = qz, qw
        od.twist.twist.linear.x = p.v_achieved_mps
        if self._prev_yaw is not None:                    # achieved yaw rate [rad/s] in the ROS frame
            dyaw = math.atan2(math.sin(p.yaw_rad - self._prev_yaw), math.cos(p.yaw_rad - self._prev_yaw))
            od.twist.twist.angular.z = -dyaw / self.fm.dt
        self._prev_yaw = p.yaw_rad
        self._pub_odom.publish(od)

        self._pub_state.publish(String(data=json.dumps({**vars(p), "met": self.fm.met})))

    def _publish_leg(self, status: int) -> None:
        leg = messages.Leg(leg_id=self._leg_id, status=status, commanded_dist_m=self._commanded,
                           achieved_dist_m=self._achieved, energy_J=self.fm.energy_j - self._energy0,
                           mass_kg=self.fm.cs.total_mass(), final_row=self.fm.rc[0], final_col=self.fm.rc[1])
        self._pub_leg.publish(String(data=json.dumps({**vars(leg), "met": self.fm.met})))
        self.get_logger().info(f"leg {leg.leg_id} -> {messages.LEG_STATUS_NAME.get(status, status)} "
                               f"ach {leg.achieved_dist_m:.1f} m")


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = RoverExecutive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
