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
        DeclareLaunchArgument(
            "predict_obstacle_motion", default_value="true",
            description="Advance tracked obstacles along their measured "
                        "velocity when scoring a rollout. False reproduces "
                        "the Phase 11 planner, which treats every obstacle "
                        "as stationary -- the baseline the prediction is "
                        "measured against.",
        ),
        DeclareLaunchArgument(
            "log_tracks", default_value="false",
            description="Log every tracked mover once a second: position, "
                        "speed, extent, hit count. Diagnostic only; the "
                        "count on its own cannot tell a phantom from a "
                        "real object.",
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
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    # An undeclared `name:=value` on the command line is
                    # accepted and silently discarded by ros2 launch, so a
                    # benchmark arm can appear to be configured while every
                    # run uses the YAML default. That happened: the "no
                    # prediction" arm ran WITH prediction for its whole
                    # sweep. tests/integration/test_launch_arguments.py now
                    # fails if a script passes an argument no launch file
                    # declares.
                    "predict_obstacle_motion": ParameterValue(
                        LaunchConfiguration("predict_obstacle_motion"),
                        value_type=bool),
                    "log_tracks": ParameterValue(
                        LaunchConfiguration("log_tracks"), value_type=bool),
                },
            ],
            condition=IfCondition(LaunchConfiguration("local_planner")),
        ),
    ])
