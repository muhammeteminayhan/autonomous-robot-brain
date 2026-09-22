"""Bring up wheel odometry and, optionally, the EKF.

The odometry node publishes dead reckoning only: no IMU, no correction. The
EKF fuses that odometry with the IMU gyroscope and publishes the map -> odom
correction on top.

Running the odometry alone is useful and supported: its drift is the baseline
the filter is measured against, and Phase 5 compares the two directly.

Usage::

    ros2 launch arb_localization odometry.launch.py
    ros2 launch arb_localization odometry.launch.py ekf:=false
    ros2 launch arb_localization odometry.launch.py ekf_use_imu:=false
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
        DeclareLaunchArgument(
            "ekf", default_value="true",
            description="Also start the EKF. Set false to measure raw "
                        "dead-reckoning drift as a baseline.",
        ),
        DeclareLaunchArgument(
            "ekf_use_imu", default_value="true",
            description="Let the EKF fuse the gyroscope. Set false to isolate "
                        "the contribution of the IMU in an ablation.",
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
        Node(
            package=PKG,
            executable="ekf_node",
            name="ekf",
            output="screen",
            parameters=[
                params,
                {
                    "use_sim_time": use_sim_time,
                    "use_imu": LaunchConfiguration("ekf_use_imu"),
                },
            ],
            condition=IfCondition(LaunchConfiguration("ekf")),
        ),
    ])
