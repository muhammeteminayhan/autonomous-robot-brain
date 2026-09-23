"""Bring up occupancy grid mapping.

The map is built from the ESTIMATED pose, never from ground truth. That makes
it self-referential -- an error in the estimate smears the map -- which is the
honest configuration: a map built from ground truth is a beautiful picture that
proves nothing about the stack.

Usage::

    ros2 launch arb_mapping mapping.launch.py
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "arb_mapping"


def generate_launch_description() -> LaunchDescription:
    default_params = os.path.join(
        get_package_share_directory(PKG), "config", "mapping.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("params", default_value=default_params),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package=PKG,
            executable="mapping_node",
            name="mapping",
            output="screen",
            parameters=[
                LaunchConfiguration("params"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
        ),
    ])
