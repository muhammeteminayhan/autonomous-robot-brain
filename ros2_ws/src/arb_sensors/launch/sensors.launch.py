"""Bring up the three sensor preprocessing nodes.

Each sensor gets its own process. That costs a little memory and buys three
things worth more than the memory: a crash is contained to one sensor, the
per-node timing reported by `ros2 topic hz` is honest rather than shared with
two other callbacks, and Phase 10 can inject a fault into one sensor by name
without any cross-talk.

Usage::

    ros2 launch arb_sensors sensors.launch.py
    ros2 launch arb_sensors sensors.launch.py params:=/path/to/other.yaml
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "arb_sensors"

#: Node executable -> node name. The name must match the key in the parameter
#: file, otherwise the node starts with defaults and silently ignores the file.
NODES = {
    "imu_preprocessor_node": "imu_preprocessor",
    "encoder_preprocessor_node": "encoder_preprocessor",
    "lidar_preprocessor_node": "lidar_preprocessor",
}


def generate_launch_description() -> LaunchDescription:
    default_params = os.path.join(
        get_package_share_directory(PKG), "config", "sensors.yaml"
    )

    params = LaunchConfiguration("params")
    use_sim_time = LaunchConfiguration("use_sim_time")

    actions = [
        DeclareLaunchArgument(
            "params", default_value=default_params,
            description="ROS parameter file for the sensor preprocessing nodes",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Follow /clock. Must be true whenever Gazebo is running, "
                        "or timestamps come from the wall clock and every dt is wrong.",
        ),
    ]

    for executable, name in NODES.items():
        actions.append(
            Node(
                package=PKG,
                executable=executable,
                name=name,
                output="screen",
                parameters=[params, {"use_sim_time": use_sim_time}],
                # Restart on crash rather than taking the whole stack down: a
                # sensor node dying should degrade the system, not kill it.
                respawn=True,
                respawn_delay=2.0,
            )
        )

    return LaunchDescription(actions)
