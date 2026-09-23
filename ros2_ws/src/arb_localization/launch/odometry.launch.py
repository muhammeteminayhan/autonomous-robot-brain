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

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "arb_localization"


def _launch_setup(context, *args, **kwargs):
    """Resolve the initial pose by name from the shared floor plan.

    The map frame has no absolute anchor: encoders and a gyroscope measure
    change, never position. Something has to tell the robot where it starts,
    and taking it from the same named pose the simulator spawns at keeps the
    two from drifting apart. On hardware this is the AMCL initial pose, a
    docking station, or a fiducial.
    """
    params = LaunchConfiguration("params").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context) == "true"
    publish_tf = LaunchConfiguration("publish_tf").perform(context) == "true"
    run_ekf = LaunchConfiguration("ekf").perform(context) == "true"
    use_imu = LaunchConfiguration("ekf_use_imu").perform(context) == "true"
    run_matcher = LaunchConfiguration("scan_match").perform(context) == "true"
    spawn_name = LaunchConfiguration("initial_pose").perform(context)

    world_cfg = yaml.safe_load(
        open(os.path.join(
            get_package_share_directory("arb_description"), "config", "world.yaml"))
    )["world"]
    poses = world_cfg["poses"]
    if spawn_name not in poses:
        raise RuntimeError(
            f"initial_pose '{spawn_name}' not found in world.yaml; "
            f"available: {sorted(poses)}")
    start = poses[spawn_name]

    actions = [
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
    ]
    if run_ekf:
        actions.append(
            Node(
                package=PKG,
                executable="ekf_node",
                name="ekf",
                output="screen",
                parameters=[
                    params,
                    {
                        "use_sim_time": use_sim_time,
                        "use_imu": use_imu,
                        "initial.x": float(start["x"]),
                        "initial.y": float(start["y"]),
                        "initial.yaw": float(start["yaw"]),
                        "use_scan_match": run_matcher,
                    },
                ],
            )
        )
    if run_matcher:
        actions.append(
            Node(
                package=PKG,
                executable="scan_matcher_node",
                name="scan_matcher",
                output="screen",
                parameters=[params, {"use_sim_time": use_sim_time}],
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    default_params = os.path.join(
        get_package_share_directory(PKG), "config", "localization.yaml"
    )

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
        DeclareLaunchArgument(
            "scan_match", default_value="true",
            description="Match scans against the map and feed the result to "
                        "the EKF as an absolute pose. Set false to see what "
                        "dead reckoning alone does to the map.",
        ),
        DeclareLaunchArgument(
            "initial_pose", default_value="spawn",
            description="Named pose from configs/world.yaml where the robot "
                        "starts. Anchors the map frame; must match the "
                        "simulator's spawn argument.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
