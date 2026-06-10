# CCSDS/ROS rover nav stack — ROS 2 Jazzy + the NumPy terramechanics authority.
# Follows the scripts/ros2_bridge convention: base on osrf/ros, keep deps in the image, and MOUNT the
# repo (terrain_authority, samples, scripts) at /work rather than COPY, so the conserved physics
# authority is the single source of truth and code edits reflect without a rebuild.
FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-c"]

# terrain_authority hot path is NumPy/SciPy; matplotlib for the ground-station artifact. APT keeps these
# ABI-matched to the system python ROS uses.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-numpy python3-scipy python3-matplotlib python3-pip \
    && rm -rf /var/lib/apt/lists/*

# rosbags = pure-python MCAP writer (same lib the ros2_bridge uses) for optional telemetry bagging.
RUN pip3 install --no-cache-dir --break-system-packages "rosbags>=0.10,<1" || true

# The repo is mounted at /work (see docker-compose.yml). Make terrain_authority + the package importable.
ENV PYTHONPATH=/work:/work/scripts/ccsds_ros_nav
ENV MPLBACKEND=Agg
WORKDIR /work/scripts/ccsds_ros_nav

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
