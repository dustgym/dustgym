#!/usr/bin/env bash
# Source the ROS 2 overlay, then exec the container command (mirrors osrf/ros' own entrypoint).
set -e
source /opt/ros/jazzy/setup.bash
exec "$@"
