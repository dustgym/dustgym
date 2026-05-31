# foss_ipex Track C1 -- ROS2 side of the M1 front-stereo -> AprilTag -> pose-vs-truth bridge.
# sensor_bridge_contract.md §2-§3. Buildable & testable WITHOUT the Godot track (via fixtures/).
#
# Base: osrf/ros:jazzy-desktop (ROS2 Jazzy on Ubuntu 24.04 Noble).
#
# FALLBACK NOTE (2026-05-30): the contract prefers `osrf/ros:jazzy-perception`, but that tag's
# manifest does NOT resolve from this environment's registry (osrf/ros publishes only
# jazzy-desktop / jazzy-desktop-full / jazzy-simulation for Jazzy; the official library/ros
# jazzy* tags were also unreachable here). `jazzy-desktop` is a clean superset of -perception
# (it bundles image_pipeline/stereo_image_proc + rclpy + the rosbag2 stack) and was verified to
# have valid Jazzy/Noble apt candidates for every package below (apriltag_ros 3.3.0,
# stereo_image_proc 5.0.11, rosbag2_storage_mcap 0.26.10). No humble/Jammy fallback was needed.
FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-lc"]

# --- apt layer: the detector + stereo + MCAP storage (cached unless this line changes) -----
#   ros-jazzy-apriltag-ros          christianrauch/apriltag_ros -- node `apriltag_node`
#   ros-jazzy-apriltag-msgs         AprilTagDetectionArray msg (pulled in, pinned explicit)
#   ros-jazzy-stereo-image-proc     M2 disparity (perception tier usually has it; explicit)
#   ros-jazzy-rosbag2-storage-mcap  MCAP storage plugin for `ros2 bag play/info`
#   ros-jazzy-rosbag2-transport     `ros2 bag` CLI verbs
#   python3-pip / python3-venv      to install the pure-python `rosbags` writer
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-apriltag-ros \
        ros-jazzy-apriltag-msgs \
        ros-jazzy-stereo-image-proc \
        ros-jazzy-rosbag2-storage-mcap \
        ros-jazzy-rosbag2-transport \
        ros-jazzy-rosbag2-py \
        python3-pip \
        python3-venv \
    && rm -rf /var/lib/apt/lists/*

# --- pip layer: pure-python rosbag2 writer (no rclpy needed to WRITE the bag) --------------
# Installed into a venv with --system-site-packages so it sits ALONGSIDE the apt rclpy/numpy
# (PEP 668 "externally managed" guard on Noble forbids a bare system pip install). This is the
# CONTAINER's python env -- the repo .venv is never touched (contract constraint).
ENV BRIDGE_VENV=/opt/bridge-venv
RUN python3 -m venv --system-site-packages "${BRIDGE_VENV}" \
    && "${BRIDGE_VENV}/bin/pip" install --no-cache-dir "rosbags>=0.10,<1.0"
ENV PATH="${BRIDGE_VENV}/bin:${PATH}"

# --- bridge sources ------------------------------------------------------------------------
WORKDIR /bridge
COPY frames.py bag_writer.py compare_pose.py depth_map.py fiducial_overlay.py \
     apriltag_bringup.launch.py tags_36h11.yaml test_frames.py ./
COPY fixtures/ ./fixtures/

# Source the ROS overlay for every interactive/`docker compose run` shell.
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /etc/bash.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
