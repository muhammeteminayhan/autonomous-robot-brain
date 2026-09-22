"""Bring up the Gazebo Harmonic simulation with the robot spawned in it.

This is the Phase 1 entry point. It starts five things:

1. ``gz sim``                -- physics, sensors and rendering
2. ``robot_state_publisher`` -- turns the URDF into the static part of the TF tree
3. ``ros_gz_sim create``     -- spawns the robot model into the running world
4. ``ros_gz_bridge``         -- the only path between gz-transport and ROS 2 DDS
5. ``rviz2``                 -- optional visualisation

Usage::

    ros2 launch arb_simulation simulation.launch.py
    ros2 launch arb_simulation simulation.launch.py headless:=true rviz:=false
    ros2 launch arb_simulation simulation.launch.py spawn:=goal_room_c
"""

from __future__ import annotations

import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#: Extra height above the wheels-on-ground pose at spawn time. A small positive
#: drop lets the contact solver settle the wheels instead of starting from an
#: interpenetrating state; too large and the robot bounces.
SPAWN_DROP_M = 0.02

PKG_SIM = "arb_simulation"
PKG_DESC = "arb_description"
WORLD_NAME = "arb_indoor"


def _launch_setup(context, *args, **kwargs):
    """Resolve launch arguments and build the node list.

    An OpaqueFunction is used rather than plain substitutions because the spawn
    pose is looked up by NAME in configs/world.yaml, and that lookup needs the
    resolved string value of the ``spawn`` argument, not a Substitution object.
    """
    sim_share = get_package_share_directory(PKG_SIM)
    desc_share = get_package_share_directory(PKG_DESC)

    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    use_rviz = LaunchConfiguration("rviz").perform(context).lower() == "true"
    spawn_name = LaunchConfiguration("spawn").perform(context)
    world_file = LaunchConfiguration("world").perform(context)

    # --- spawn pose from the floor plan -------------------------------------
    world_cfg = yaml.safe_load(
        open(os.path.join(desc_share, "config", "world.yaml"))
    )["world"]
    poses = world_cfg["poses"]
    if spawn_name not in poses:
        raise RuntimeError(
            f"spawn pose '{spawn_name}' not found in world.yaml; "
            f"available: {sorted(poses)}"
        )
    pose = poses[spawn_name]

    # Spawn height. `-z` positions the MODEL ORIGIN, which for this robot is
    # base_link at the chassis centre, NOT the lowest point of the model. The
    # wheels only touch the ground when base_link sits at
    # ground_clearance + chassis_height/2 above it.
    #
    # Passing a small literal like 0.05 "to drop it gently" actually spawns the
    # robot 5 cm INTO the floor: the solver then ejects it, the wheels scrape on
    # the way out, and the first seconds of every run contain slip that looks
    # like an encoder fault. Measured cost of that bug: 14 % disagreement
    # between wheel arc length and distance actually travelled.
    robot_cfg = yaml.safe_load(
        open(os.path.join(desc_share, "config", "robot.yaml"))
    )["robot"]
    base_z = (
        robot_cfg["chassis"]["ground_clearance_m"]
        + robot_cfg["chassis"]["height_m"] / 2.0
    )
    spawn_z = base_z + SPAWN_DROP_M

    # --- robot description ---------------------------------------------------
    # xacro is run by the launch file (via the xacro command substitution inside
    # robot_state_publisher) so that a change to configs/robot.yaml takes effect
    # on the next launch without a rebuild.
    xacro_file = os.path.join(desc_share, "urdf", "arb_robot.urdf.xacro")
    robot_description_xml = xacro.process_file(
        xacro_file,
        mappings={
            "robot_config": os.path.join(desc_share, "config", "robot.yaml"),
            "sensor_config": os.path.join(desc_share, "config", "sensors.yaml"),
            "use_gazebo": "true",
        },
    ).toxml()

    # --- 1. Gazebo -----------------------------------------------------------
    # -r starts the world unpaused; -s runs server only (no GUI); -v 3 keeps the
    # log readable (4 is firehose).
    gz_args = f"-r -v 3 {os.path.join(sim_share, 'worlds', world_file)}"
    if headless:
        gz_args = f"-s {gz_args}"

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items(),
    )

    # --- 2. robot_state_publisher -------------------------------------------
    # Publishes /tf_static for every fixed joint and /tf for the wheel joints,
    # driven by /joint_states. This is what makes lidar_link and imu_link exist
    # as TF frames even though Gazebo lumped them into base_link internally.
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_xml, "use_sim_time": True}
        ],
    )

    # --- 3. spawn ------------------------------------------------------------
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_arb_robot",
        output="screen",
        arguments=[
            "-name", "arb_diffdrive",
            "-topic", "robot_description",
            "-x", str(pose["x"]),
            "-y", str(pose["y"]),
            "-z", str(spawn_z),
            "-Y", str(pose["yaw"]),
        ],
        parameters=[{"use_sim_time": True}],
    )

    # --- 4. bridge -----------------------------------------------------------
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[
            {
                "config_file": os.path.join(sim_share, "config", "ros_gz_bridge.yaml"),
                "use_sim_time": True,
            }
        ],
    )

    # --- 5. rviz -------------------------------------------------------------
    # RViz starts alongside the rest rather than being chained to the spawner
    # finishing. It reads /robot_description, which robot_state_publisher
    # latches with TRANSIENT_LOCAL durability, so RViz receives the model
    # whenever it happens to connect; ordering is not the launch file's problem.
    #
    # An earlier version chained it with an event handler whose on_exit list was
    # EMPTY when rviz was disabled. That is not a valid handler: launch raised an
    # AttributeError during description construction and then tore the whole
    # session down, Gazebo included. The failure mode is worth remembering,
    # because the traceback names an internal attribute and says nothing about
    # the empty list. Conditional composition belongs in plain Python.
    actions = [gz_sim, rsp, bridge, spawn]
    if use_rviz:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", os.path.join(desc_share, "rviz", "arb_simulation.rviz")],
                parameters=[{"use_sim_time": True}],
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world", default_value=f"{WORLD_NAME}.sdf",
                description="SDF world file name inside arb_simulation/worlds/",
            ),
            DeclareLaunchArgument(
                "spawn", default_value="spawn",
                description="Named pose from configs/world.yaml to spawn at",
            ),
            DeclareLaunchArgument(
                "headless", default_value="false",
                description="Run gz server only, no GUI (for batch experiments)",
            ),
            DeclareLaunchArgument(
                "rviz", default_value="true",
                description="Start RViz2",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
