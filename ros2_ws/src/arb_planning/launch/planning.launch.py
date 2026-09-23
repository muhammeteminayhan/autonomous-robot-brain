"""Bring up the global planner.

Usage::

    ros2 launch arb_planning planning.launch.py
    ros2 launch arb_planning planning.launch.py heuristic:=dijkstra
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "arb_planning"


def generate_launch_description() -> LaunchDescription:
    default_params = os.path.join(
        get_package_share_directory(PKG), "config", "planning.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("params", default_value=default_params),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "heuristic", default_value="octile",
            description="octile | euclidean | manhattan | dijkstra. "
                        "manhattan is inadmissible on an 8-connected grid and "
                        "is offered so the cost of that can be measured.",
        ),
        Node(
            package=PKG,
            executable="global_planner_node",
            name="global_planner",
            output="screen",
            parameters=[
                LaunchConfiguration("params"),
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "heuristic": LaunchConfiguration("heuristic"),
                },
            ],
        ),
    ])
