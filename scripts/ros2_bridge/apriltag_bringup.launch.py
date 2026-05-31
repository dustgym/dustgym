"""M1 AprilTag bringup: detect the lander's id-0 tag36h11 on the front-LEFT image.

sensor_bridge_contract.md §5.2: feed the bag's `/front_left/image_raw` + `/front_left/camera_info`
into apriltag_ros, which must DETECT id 0.  apriltag_ros subscribes to `image_rect` +
`camera_info`; for M1 the cameras are rectified pinhole (distortion OFF, contract §2.2), so
`image_raw` IS the rectified image and we remap `image_rect:=/front_left/image_raw` directly.

The detector publishes:
  /tf                 tf2_msgs/TFMessage          (map-less: <optical_frame> -> "tag36h11:0")
  /detections         apriltag_msgs/AprilTagDetectionArray

Run (in the container, after `ros2 bag play` of a bag from bag_writer.py -- see README):
    ros2 launch apriltag_bringup.launch.py

M2 (NOT M1; stubbed below, intentionally not launched): stereo_image_proc to produce a
disparity/point cloud from the front stereo pair, then rtabmap SLAM.  Left commented so it
does not block the M1 single-tag acceptance.

CC0-1.0 (see ../../LICENSE).
"""

from __future__ import annotations

import os

from launch import LaunchDescription
from launch_ros.actions import Node

_HERE = os.path.dirname(os.path.abspath(__file__))
_CFG = os.path.join(_HERE, "tags_36h11.yaml")


def generate_launch_description() -> LaunchDescription:
    apriltag = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag",
        output="screen",
        parameters=[_CFG],
        remappings=[
            ("image_rect", "/front_left/image_raw"),
            ("camera_info", "/front_left/camera_info"),
        ],
    )

    # --- M2 (do NOT enable for M1) -------------------------------------------------------
    # stereo = ComposableNodeContainer(
    #     name="stereo_container", namespace="", package="rclcpp_components",
    #     executable="component_container",
    #     composable_node_descriptions=[
    #         ComposableNode(package="stereo_image_proc",
    #                        plugin="stereo_image_proc::DisparityNode", ...),
    #     ])
    # rtabmap = Node(package="rtabmap_slam", executable="rtabmap", ...)

    return LaunchDescription([apriltag])
