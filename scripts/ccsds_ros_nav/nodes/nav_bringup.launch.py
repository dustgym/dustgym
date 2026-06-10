"""Launch the rover nav stack inside the container: CCSDS bridge + rover executive.

    ros2 launch nav_bringup.launch.py body:=moon win:=160 light_time_s:=1.28

The nodes are plain python files run via ``python3 ... --ros-args`` (no colcon install / entry point
needed). The ground station runs outside (host or its own container) and speaks CCSDS/UDP to the bridge.
"""
from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

_HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description() -> LaunchDescription:
    body = LaunchConfiguration("body")
    win = LaunchConfiguration("win")
    light = LaunchConfiguration("light_time_s")
    time_factor = LaunchConfiguration("time_factor")
    return LaunchDescription([
        DeclareLaunchArgument("body", default_value="moon"),
        DeclareLaunchArgument("win", default_value="160"),
        # 0.0: for the HITL console the operator-side owns the (adjustable) latency model; set >0 only
        # for the batch ground_station path if you want bridge-side downlink delay.
        DeclareLaunchArgument("light_time_s", default_value="0.0"),
        DeclareLaunchArgument("time_factor", default_value="1.0"),
        ExecuteProcess(
            cmd=["python3", os.path.join(_HERE, "ccsds_bridge_node.py"),
                 "--ros-args", "-p", ["light_time_s:=", light]],
            output="screen"),
        ExecuteProcess(
            cmd=["python3", os.path.join(_HERE, "rover_executive_node.py"),
                 "--ros-args", "-p", ["body:=", body], "-p", ["win:=", win],
                 "-p", ["time_factor:=", time_factor]],
            output="screen"),
    ])
