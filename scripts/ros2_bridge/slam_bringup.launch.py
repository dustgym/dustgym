"""M2 stereo-SLAM bringup: rtabmap stereo VO + graph SLAM on the front pair -> map-frame pose.

sensor_bridge_contract.md §2.3 / §5 (the Workstream-C SLAM half). Feeds the bag's front stereo
stream into rtabmap to produce the loop-closed, globally-consistent pose the scorer (lane C)
asserts on:

  /slam/odom   nav_msgs/msg/Odometry   header.frame_id == "map"   child_frame_id == "base_link"

i.e. the loop-closed `map`-frame pose, NOT the drifting `odom` frame.

Pipeline (two rtabmap nodes; NO stereo_image_proc rectify pre-stage needed):

    /front_left/image_raw  + /front_left/camera_info  ┐
    /front_right/image_raw + /front_right/camera_info ┘─> rtabmap_odom/stereo_odometry
                                                              -> /odom  (nav_msgs/Odometry, odom->base_link, DRIFTING VO)
                                                          rtabmap_slam/rtabmap  (graph + loop closure)
                                                              -> map->odom TF correction (the loop-closure feedback)
                                                              -> /slam/odom  (map-frame pose, see "THE /slam/odom SEAM")

The Godot images are pre-rectified pinhole (``camera_info.D == [0,0,0,0,0]``, contract §2.2), so
rectification is a PASS-THROUGH: rtabmap's stereo front-end consumes ``image_raw`` directly and no
``stereo_image_proc`` disparity/rectify node is required (Dockerfile note, lines 30-32). The same
``image_rect := image_raw`` remap discipline as ``apriltag_bringup.launch.py`` is used because raw
IS rectified here.

THE /slam/odom SEAM (read me before merge -- M2-slam.md "integration_notes"):
  rtabmap_slam's main node connects `map` -> `base_link` through the TF tree (it publishes the
  `map->odom` *correction* TF; the VO node publishes `odom->base_link`). Its typed map-frame pose
  message is ``localization_pose`` (``geometry_msgs/PoseWithCovarianceStamped``, frame_id==map) --
  the stock node does NOT advertise a ``nav_msgs/Odometry`` on a renamable map-frame topic
  (verified against introlab/rtabmap_ros CoreWrapper: the only nav_msgs/Odometry publisher in the
  pipeline is the rtabmap_odom VO node, which is the `odom` frame). To honour contract §5's
  ``/slam/odom`` (``nav_msgs/Odometry``, frame_id==map) the orchestrator picks ONE at merge:
    (A) point lane C's subscriber at ``localization_pose`` (PoseWithCovarianceStamped, frame==map)
        and have it assert frame_id=='map' there -- same loud-fail guarantee, different msg type; OR
    (B) add a ~12-line typed adapter that restamps ``localization_pose`` (or composes map->odom @
        /odom) into ``nav_msgs/Odometry`` on ``/slam/odom``. The contract calls /slam/odom "no
        republisher", so the adapter (if chosen) is a TYPE bridge, not an aliasing node, and must
        carry frame_id=='map'/child=='base_link' verbatim.
  This launch therefore configures rtabmap correctly for the map-frame pose and remaps
  ``localization_pose -> /slam/odom`` so the topic NAME already matches; the message TYPE
  reconciliation (Odometry vs PoseWithCovarianceStamped) is the orchestrator's one-line call above.
  Either way the FRAME is honest: this lane never mislabels the drifting `odom` VO as `map`.

Run (in the container, alongside ``ros2 bag play`` of a bag_seq_writer.py MCAP -- see
``docs/lanes/M2-slam.md``):
    ros2 launch slam_bringup.launch.py
    ros2 bag play -s mcap --clock bags/<scene>_seq      # in a second shell

Args (``ros2 launch slam_bringup.launch.py --show-args``):
    approx_sync   (default true)  -- approx-time stereo sync (Godot stamps L/R identically, so
                                     true is safe and tolerant of any sub-ns skew)
    use_sim_time  (default true)  -- consume the bag's /clock (play with --clock)
    queue_size    (default 30)
    rtabmap_viz   (default false) -- headless by default (no GUI in the container)

CC0-1.0 (see ../../LICENSE).
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    approx_sync = LaunchConfiguration("approx_sync")
    use_sim_time = LaunchConfiguration("use_sim_time")
    queue_size = LaunchConfiguration("queue_size")
    rtabmap_viz = LaunchConfiguration("rtabmap_viz")

    # Front stereo topics (contract §2.3). Raw == rectified (D=0), so feed image_raw directly into
    # rtabmap's stereo subscriber (it expects left/right image_rect + camera_info).
    stereo_remaps = [
        ("left/image_rect", "/front_left/image_raw"),
        ("left/camera_info", "/front_left/camera_info"),
        ("right/image_rect", "/front_right/image_raw"),
        ("right/camera_info", "/front_right/camera_info"),
    ]

    common_params = {
        "frame_id": "base_link",          # contract §2.3 child_frame_id
        "map_frame_id": "map",            # contract §2.3 header.frame_id (loop-closed/global)
        "odom_frame_id": "odom",          # the intermediate DRIFTING VO frame (never /slam/odom)
        "approx_sync": approx_sync,
        "use_sim_time": use_sim_time,
        "queue_size": queue_size,
        "subscribe_stereo": True,
        "subscribe_depth": False,
        "subscribe_rgbd": False,
        "wait_imu_to_init": False,
    }

    # --- 1. stereo visual odometry (rtabmap_odom) -> /odom (odom -> base_link, DRIFTING) -------
    #     This is the ONLY nav_msgs/Odometry publisher in the stock pipeline; it is the `odom`
    #     frame, NOT the loop-closed map frame. rtabmap (below) corrects it via map->odom TF.
    stereo_odometry = Node(
        package="rtabmap_odom",
        executable="stereo_odometry",
        name="stereo_odometry",
        output="screen",
        parameters=[common_params],
        remappings=stereo_remaps + [("odom", "/odom")],
    )

    # --- 2. graph SLAM (rtabmap_slam) -> map->odom TF + localization_pose (map frame) ----------
    #     publish_tf=True => map->odom loop-closure correction. Its typed map-frame pose is
    #     `localization_pose` (PoseWithCovarianceStamped, frame_id=map). We remap that to the
    #     contract topic NAME /slam/odom (see "THE /slam/odom SEAM" in the module docstring for the
    #     Odometry-vs-PoseWithCovarianceStamped type reconciliation the orchestrator owns).
    rtabmap_params = dict(common_params)
    rtabmap_params.update({
        "subscribe_odom_info": True,
        "publish_tf": True,                          # map -> odom loop-closure correction TF
        "Rtabmap/DetectionRate": "1.0",
        "RGBD/NeighborLinkRefining": "true",
        "RGBD/ProximityBySpace": "true",
        "Reg/Strategy": "0",                         # 0 = Vis (visual) registration for stereo
        "Mem/IncrementalMemory": "true",
        "Rtabmap/PublishOdometry": "true",           # include odom in the Info/graph output
        "database_path": "",                         # in-memory DB; no on-disk rtabmap.db artifact
    })
    rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[rtabmap_params],
        remappings=stereo_remaps + [
            ("odom", "/odom"),                       # consume the VO odom-frame pose
            ("localization_pose", "/slam/odom"),     # map-frame pose -> contract topic NAME §5
        ],
        arguments=["--delete_db_on_start"],
    )

    # --- 3. optional rtabmap GUI (needs a display) -- default OFF for the headless container ----
    rtabmap_viz_node = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        name="rtabmap_viz",
        output="screen",
        parameters=[rtabmap_params],
        remappings=stereo_remaps + [("odom", "/odom")],
        condition=IfCondition(rtabmap_viz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("approx_sync", default_value="true",
                              description="approx-time stereo synchronizer"),
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="use the bag /clock (play with --clock)"),
        DeclareLaunchArgument("queue_size", default_value="30",
                              description="subscriber queue size"),
        DeclareLaunchArgument("rtabmap_viz", default_value="false",
                              description="launch the rtabmap GUI (needs a display; off by default)"),
        stereo_odometry,
        rtabmap,
        rtabmap_viz_node,
    ])
