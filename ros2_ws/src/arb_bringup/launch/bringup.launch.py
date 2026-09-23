"""The whole stack in one command.

    simulation -> sensors -> localization -> mapping -> planning -> wheels

This is the file `arb_bringup` was created for. Everything it starts is an
`IncludeLaunchDescription` of a per-package launch file, never a re-declaration
of the nodes: a bringup that redefines what it composes drifts away from the
packages it is supposed to compose, and the parameters diverge silently.

WHY THE DELAYS
Each stage is started behind a `TimerAction`, which is a blunt instrument and
deliberately so. The clean alternative is to chain each stage to an event from
the previous one, but the events available here do not mean what they would
need to mean: `OnProcessStart` for `gz sim` fires when the process exists, not
when physics is stepping and the model has been spawned. The nodes downstream
tolerate a late start -- they simply publish nothing until their inputs arrive
-- so the delays buy determinism in the logs rather than correctness.

Usage::

    # everything, with RViz
    ros2 launch arb_bringup bringup.launch.py

    # headless, no RViz: for batch runs and for machines without a GPU
    ros2 launch arb_bringup bringup.launch.py headless:=true rviz:=false

    # the PID path follower instead of the DWA local planner
    ros2 launch arb_bringup bringup.launch.py controller:=pid

    # start somewhere else; the name comes from configs/world.yaml
    ros2 launch arb_bringup bringup.launch.py spawn:=control_test
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#: Seconds after launch at which each stage starts. Gazebo needs the longest:
#: it loads the world, spawns the model and lets the contact solver settle the
#: wheels before any sensor produces anything worth reading.
DELAY_SENSORS = 12.0
DELAY_LOCALIZATION = 15.0
DELAY_MAPPING = 18.0
DELAY_PLANNING = 20.0


def _include(package: str, launch_file: str, arguments: dict):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), "launch", launch_file)
        ),
        launch_arguments=arguments.items(),
    )


def _setup(context, *args, **kwargs):
    spawn = LaunchConfiguration("spawn").perform(context)
    headless = LaunchConfiguration("headless").perform(context)
    use_rviz = LaunchConfiguration("rviz").perform(context).lower() == "true"
    controller = LaunchConfiguration("controller").perform(context).lower()
    unknown_is_lethal = LaunchConfiguration("unknown_is_lethal").perform(context)

    if controller not in ("dwa", "pid", "none"):
        raise RuntimeError(
            f"controller must be dwa, pid or none, not '{controller}'")

    actions = [
        # --- 1. simulation ---------------------------------------------------
        # RViz is started from here rather than by arb_simulation, because the
        # view worth looking at is the one with the map and the plan in it, and
        # arb_simulation cannot depend on packages that come after it.
        _include("arb_simulation", "simulation.launch.py",
                 {"headless": headless, "rviz": "false", "spawn": spawn}),

        # --- 2. sensors ------------------------------------------------------
        TimerAction(period=DELAY_SENSORS, actions=[
            _include("arb_sensors", "sensors.launch.py", {}),
        ]),

        # --- 3. localization -------------------------------------------------
        # `initial_pose` MUST match the simulator's `spawn`. It anchors the map
        # frame; a mismatch does not fail, it silently offsets the entire map
        # from the world by the difference.
        TimerAction(period=DELAY_LOCALIZATION, actions=[
            _include("arb_localization", "odometry.launch.py",
                     {"initial_pose": spawn}),
        ]),

        # --- 4. mapping ------------------------------------------------------
        TimerAction(period=DELAY_MAPPING, actions=[
            _include("arb_mapping", "mapping.launch.py", {}),
        ]),
    ]

    # --- 5. planning and control ---------------------------------------------
    # The DWA local planner and the PID follower both publish /cmd_vel, so
    # exactly one of them may run. `controller:=none` starts the global planner
    # alone, which is what you want when driving by hand with teleop.
    planning = [
        _include("arb_planning", "planning.launch.py",
                 {"local_planner": "true" if controller == "dwa" else "false",
                  "unknown_is_lethal": unknown_is_lethal}),
    ]
    if controller == "pid":
        planning.append(_include("arb_control", "control.launch.py", {}))
    actions.append(TimerAction(period=DELAY_PLANNING, actions=planning))

    if use_rviz:
        actions.append(Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", os.path.join(
                get_package_share_directory("arb_bringup"),
                "config", "arb_navigation.rviz")],
            parameters=[{"use_sim_time": True}],
        ))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "spawn", default_value="spawn",
            description="Named pose from configs/world.yaml. Used for BOTH the "
                        "simulator's spawn point and the EKF's initial pose, "
                        "so they cannot drift apart.",
        ),
        DeclareLaunchArgument(
            "headless", default_value="false",
            description="Run Gazebo without its GUI. RViz is a separate "
                        "argument and is usually the view you actually want.",
        ),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "controller", default_value="dwa",
            description="dwa | pid | none. Both publish /cmd_vel, so only one "
                        "may run: two publishers on a command topic is a robot "
                        "that does neither thing. 'none' starts the global "
                        "planner alone, for driving by hand.",
        ),
        DeclareLaunchArgument(
            "unknown_is_lethal", default_value="false",
            description="Default false here, unlike arb_planning's own default. "
                        "From a cold start the robot has explored nothing, and "
                        "a planner that refuses to enter unknown space cannot "
                        "produce any plan at all. Planning optimistically and "
                        "replanning at 1 Hz is how a robot navigates while "
                        "still exploring.",
        ),
        OpaqueFunction(function=_setup),
    ])
