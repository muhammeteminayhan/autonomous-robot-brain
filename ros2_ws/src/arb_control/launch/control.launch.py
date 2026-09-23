"""Bring up the PID path follower.

This is the alternative to the DWA local planner, not a companion to it: both
publish /cmd_vel, and two publishers on a command topic is a robot that does
neither thing. The PID follower tracks the global path precisely and has NO
obstacle avoidance, which makes it the baseline for measuring what DWA's
avoidance costs in tracking accuracy.

Usage::

    ros2 launch arb_control control.launch.py
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "arb_control"


def generate_launch_description() -> LaunchDescription:
    default_params = os.path.join(
        get_package_share_directory(PKG), "config", "control.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("params", default_value=default_params),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package=PKG,
            executable="controller_node",
            name="controller",
            output="screen",
            parameters=[
                LaunchConfiguration("params"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
        ),
    ])
