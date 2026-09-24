"""Start the fault scheduler against a scenario file.

    ros2 launch arb_evaluation scenario.launch.py \
        scenario:=configs/scenarios/case_c_encoder_drift.yaml

`use_sim_time` defaults to true and should stay that way: the scenario's times
are SIMULATED seconds, and this world runs well above real time.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "scenario",
            description="Path to a configs/scenarios/*.yaml file. Required.",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rate_hz", default_value="20.0"),
        DeclareLaunchArgument(
            "shutdown_at_end", default_value="true",
            description="Exit when the scenario's duration elapses. False "
                        "leaves the scheduler up for manual poking.",
        ),
        Node(
            package="arb_evaluation",
            executable="fault_scheduler",
            name="fault_scheduler",
            output="screen",
            parameters=[{
                "scenario": LaunchConfiguration("scenario"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "rate_hz": LaunchConfiguration("rate_hz"),
                "shutdown_at_end": LaunchConfiguration("shutdown_at_end"),
            }],
        ),
    ])
