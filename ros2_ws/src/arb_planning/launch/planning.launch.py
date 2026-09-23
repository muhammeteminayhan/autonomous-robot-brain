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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

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
        DeclareLaunchArgument(
            "unknown_is_lethal", default_value="true",
            description="Treat never-observed cells as obstacles. True is the "
                        "safe reading and confines the robot to what it has "
                        "mapped. False plans optimistically THROUGH unknown "
                        "space and relies on replanning plus the local planner "
                        "to deal with what is actually there -- which is what "
                        "navigating while still exploring requires.",
        ),
        DeclareLaunchArgument(
            "local_planner", default_value="true",
            description="Start the DWA local planner, which publishes "
                        "/cmd_vel. Set false when using the PID follower in "
                        "arb_control instead: two publishers on a command "
                        "topic is a robot that does neither thing.",
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
                    "unknown_is_lethal": ParameterValue(
                        LaunchConfiguration("unknown_is_lethal"),
                        value_type=bool),
                },
            ],
        ),
        Node(
            package=PKG,
            executable="local_planner_node",
            name="local_planner",
            output="screen",
            parameters=[
                LaunchConfiguration("params"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            condition=IfCondition(LaunchConfiguration("local_planner")),
        ),
    ])
