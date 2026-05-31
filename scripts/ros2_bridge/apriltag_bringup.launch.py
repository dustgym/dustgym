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

M2 (un-stubbed below; gated OFF by default via the ``stereo`` launch-arg so M1 single-tag
acceptance is unchanged): a ``stereo_image_proc`` disparity container on the front pair, then
rtabmap stereo VISUAL ODOMETRY. The Godot images are pre-rectified pinhole (``camera_info.D ==
[0,0,0,0,0]``, contract §2.2), so the disparity node's rectification is a PASS-THROUGH and rtabmap
can consume ``image_raw`` directly as ``image_rect``. The full graph-SLAM + map-frame /slam/odom
pose lives in the dedicated ``slam_bringup.launch.py`` (contract §5); the stereo-odometry node here
is the shared VO front-end. Enable with ``ros2 launch apriltag_bringup.launch.py stereo:=true``.

CC0-1.0 (see ../../LICENSE).
"""

from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

_HERE = os.path.dirname(os.path.abspath(__file__))
_CFG = os.path.join(_HERE, "tags_36h11.yaml")


def generate_launch_description() -> LaunchDescription:
    # M2 is OFF by default: the AprilTag single-tag path (M1) stays the unconditional default.
    stereo = LaunchConfiguration("stereo")
    use_sim_time = LaunchConfiguration("use_sim_time")

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

    # --- M2 (un-stubbed; gated by the `stereo` arg, default false) -----------------------
    # stereo_image_proc DisparityNode on the front pair. Images are pre-rectified pinhole
    # (D=0, contract §2.2), so left/right "image_rect" IS the raw image (pass-through rectify).
    stereo_container = ComposableNodeContainer(
        name="stereo_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        condition=IfCondition(stereo),
        composable_node_descriptions=[
            ComposableNode(
                package="stereo_image_proc",
                plugin="stereo_image_proc::DisparityNode",
                name="disparity_node",
                parameters=[{"use_sim_time": use_sim_time, "approximate_sync": True}],
                remappings=[
                    ("left/image_rect", "/front_left/image_raw"),
                    ("left/camera_info", "/front_left/camera_info"),
                    ("right/image_rect", "/front_right/image_raw"),
                    ("right/camera_info", "/front_right/camera_info"),
                ],
            ),
            ComposableNode(
                package="stereo_image_proc",
                plugin="stereo_image_proc::PointCloudNode",
                name="point_cloud_node",
                parameters=[{"use_sim_time": use_sim_time, "approximate_sync": True}],
                remappings=[
                    ("left/image_rect_color", "/front_left/image_raw"),
                    ("left/camera_info", "/front_left/camera_info"),
                    ("right/camera_info", "/front_right/camera_info"),
                ],
            ),
        ],
    )

    # rtabmap stereo VISUAL ODOMETRY (the shared VO front-end; graph-SLAM lives in
    # slam_bringup.launch.py). Publishes nav_msgs/Odometry on /odom in the DRIFTING odom frame --
    # the loop-closed map-frame /slam/odom (contract §5) is slam_bringup's rtabmap node, not here.
    stereo_odometry = Node(
        package="rtabmap_odom",
        executable="stereo_odometry",
        name="stereo_odometry",
        output="screen",
        condition=IfCondition(stereo),
        parameters=[{
            "frame_id": "base_link",
            "odom_frame_id": "odom",
            "use_sim_time": use_sim_time,
            "approx_sync": True,
            "subscribe_stereo": True,
            "wait_imu_to_init": False,
        }],
        remappings=[
            ("left/image_rect", "/front_left/image_raw"),
            ("left/camera_info", "/front_left/camera_info"),
            ("right/image_rect", "/front_right/image_raw"),
            ("right/camera_info", "/front_right/camera_info"),
            ("odom", "/odom"),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("stereo", default_value="false",
                              description="enable the M2 stereo_image_proc + rtabmap stereo VO "
                                          "nodes (default false: M1 single-tag path only)"),
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="use the bag /clock (play with --clock)"),
        apriltag,
        stereo_container,
        stereo_odometry,
    ])
