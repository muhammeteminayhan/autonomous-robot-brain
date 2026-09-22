"""Bring up wheel odometry.

Phase 3 publishes dead reckoning only: no IMU, no scan matching, no
correction. That is deliberate. The drift this produces is the measurement
that justifies the Phase 4 filter, and fusing anything into it now would hide
exactly the quantity worth showing.

Usage::

    ros2 launch arb_localization odometry.launch.py
    ros2 launch arb_localization odometry.launch.py publish_tf:=false
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "arb_localization"


def generate_launch_description() -> LaunchDescription:
    default_params = os.path.join(
        get_package_share_directory(PKG), "config", "localization.yaml"
    )
    params = LaunchConfiguration("params")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_tf = LaunchConfiguration("publish_tf")

    return LaunchDescription([
        DeclareLaunchArgument("params", default_value=default_params),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Follow /clock. Must be true whenever Gazebo is running.",
        ),
        DeclareLaunchArgument(
            "publish_tf", default_value="true",
            description="Publish odom -> base_link. Set false to compare "
                        "against another odometry source without a TF conflict: "
                        "two publishers of the same transform produce a tree "
                        "that flickers between them and is worse than neither.",
        ),
        Node(
            package=PKG,
            executable="wheel_odometry_node",
            name="wheel_odometry",
            output="screen",
            parameters=[
                params,
                {"use_sim_time": use_sim_time, "publish_tf": publish_tf},
            ],
        ),
    ])
