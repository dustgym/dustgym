"""ROS 2 (rclpy) CCSDS<->ROS bridge node — the single translation place (mirrors frames.py discipline).

Uplink (ground->rover): poll the UDP CCSDS link; decode each Space Packet; republish GoTo as a native
``/cmd/nav_goal`` (geometry_msgs/PointStamped) and Safe as ``/cmd/safe`` (std_msgs/Empty) so standard
ROS tooling sees native goals.

Downlink (rover->ground): subscribe to the executive's ``/rover/state`` and ``/rover/leg`` (JSON), wrap
them back into CCSDS Pose/Leg packets, and send them over the UDP link (with its configured light-time
delay). All CCSDS<->semantic conversion lives here and nowhere else.
"""
from __future__ import annotations

import json
import os
import sys

_PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_REPO = os.path.abspath(os.path.join(_PKG, "..", ".."))
for _p in (_PKG, _REPO):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float64, String

import messages
from link import UdpLink


class CcsdsBridge(Node):
    def __init__(self) -> None:
        super().__init__("ccsds_bridge")
        self.declare_parameter("local_host", "0.0.0.0")
        self.declare_parameter("local_port", 52001)        # rover-side: receive TC, send TM
        self.declare_parameter("ground_host", "127.0.0.1")
        self.declare_parameter("ground_port", 52000)
        # Default 0: the HITL console owns the (adjustable) latency model so it is live-tunable ground-side.
        self.declare_parameter("light_time_s", 0.0)
        g = self.get_parameter
        self.link = UdpLink((g("local_host").value, int(g("local_port").value)),
                            (g("ground_host").value, int(g("ground_port").value)),
                            light_time_s=float(g("light_time_s").value))
        self._seq = 0
        # /cmd/nav_goal carries the FULL GoTo as JSON (lossless: leg_id, row, col, v_max, radius) — a
        # PointStamped (x,y,z) would drop goal_radius_cells and mislabel grid indices as REP-103 metres.
        self._pub_goal = self.create_publisher(String, "/cmd/nav_goal", 10)
        self._pub_safe = self.create_publisher(Empty, "/cmd/safe", 10)
        self._pub_tf = self.create_publisher(Float64, "/sim/time_factor", 10)
        self.create_subscription(String, "/rover/state", self._on_state, 50)
        self.create_subscription(String, "/rover/leg", self._on_leg, 10)
        self.create_timer(0.02, self._poll_uplink)          # 50 Hz uplink poll
        self.get_logger().info(f"ccsds_bridge up: TC<-:{g('local_port').value} TM->"
                               f"{g('ground_host').value}:{g('ground_port').value} "
                               f"light_time={g('light_time_s').value}s")

    # --- uplink: CCSDS -> ROS ---------------------------------------------------------------------
    def _poll_uplink(self) -> None:
        while True:
            pkt = self.link.recv(timeout=0.0)
            if pkt is None:
                return
            try:
                msg = messages.decode(pkt)
            except ValueError:
                self.get_logger().warn(f"dropping packet with unknown APID 0x{pkt.apid:03X}")
                continue
            if isinstance(msg, messages.GoTo):
                self._pub_goal.publish(String(data=json.dumps(vars(msg))))
            elif isinstance(msg, messages.Safe):
                self._pub_safe.publish(Empty())
            elif isinstance(msg, messages.SetSim):
                self._pub_tf.publish(Float64(data=float(msg.time_factor)))

    # --- downlink: ROS -> CCSDS -------------------------------------------------------------------
    def _on_state(self, s: String) -> None:
        d = json.loads(s.data)
        met = float(d.pop("met", 0.0))                     # MET rides the CCSDS secondary header, not the payload
        pose = messages.Pose(**d)
        self.link.send(messages.encode(pose, seq_count=self._next_seq(), met=met))

    def _on_leg(self, s: String) -> None:
        d = json.loads(s.data)
        met = float(d.pop("met", 0.0))
        leg = messages.Leg(**d)
        self.link.send(messages.encode(leg, seq_count=self._next_seq(), met=met))

    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0x3FFF
        return self._seq


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = CcsdsBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.link.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
