"""Spawn an obstacle into the running world and drive it across the corridor.

Phase 11's fifth experiment. Everything before it degrades a SENSOR; this
degrades the WORLD, and it is the only experiment that tests the local planner
rather than the estimator.

WHY IT CANNOT BE A WALL IN THE FLOOR PLAN
The point is that the obstacle is not in the map. The global planner routes
through space the occupancy grid says is free, and something walks into it. If
the obstacle were part of the world file the robot would map it on the way past
and A* would simply route around it, which tests the planner's costmap and
nothing about reacting.

WHY IT IS NOT A Gazebo <actor>
Actors are the obvious tool and are visual-only by default: no collision
geometry, so the LiDAR sees straight through them. An obstacle a sensor cannot
detect tests nothing.

WHY set_pose AND NOT A VELOCITY PLUGIN
Teleporting the model each tick makes its trajectory exactly reproducible,
which matters when the same run is repeated over seeds. A velocity-controlled
model would accumulate its own integration differences between runs, and the
robot would meet the obstacle at a slightly different place each time --
turning the one thing that must be identical across repeats into another source
of variance.

The obstacle's true pose is published on a ROS topic as INSTRUMENTATION, so a
run can be scored on how close the robot came to it. No planner may subscribe:
a local planner told where an obstacle is, rather than seeing it, is solving a
different problem. tests/integration/test_ground_truth_isolation.py enforces
that for ground truth and the fault state; this topic is named to fall under
the same rule.
"""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseStamped
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node as GzNode
import rclpy
from rclpy.node import Node

#: SDF for the obstacle. A cylinder, because a curved return stresses the
#: LiDAR's ray casting far more than an axis-aligned box: every beam hits at a
#: different obliquity and the despeckle filter has to cope.
#:
#: `<static>true</static>` with a collision: it must be visible to the LiDAR
#: and immovable by contact, so a robot that touches it does not push it away
#: and quietly turn a collision into a near miss.
OBSTACLE_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <pose>{x} {y} {z} 0 0 0</pose>
    <link name="link">
      <collision name="collision">
        <geometry><cylinder><radius>{radius}</radius>
          <length>{height}</length></cylinder></geometry>
      </collision>
      <visual name="visual">
        <geometry><cylinder><radius>{radius}</radius>
          <length>{height}</length></cylinder></geometry>
        <material>
          <ambient>0.9 0.2 0.2 1</ambient>
          <diffuse>0.9 0.2 0.2 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


class MovingObstacle(Node):
    """Drives one obstacle back and forth along a segment, on sim time."""

    def __init__(self) -> None:
        super().__init__("moving_obstacle")

        self.world = self.declare_parameter("world", "arb_indoor").value
        self.name = self.declare_parameter("name", "arb_pedestrian").value
        self.radius = self.declare_parameter("radius_m", 0.20).value
        self.height = self.declare_parameter("height_m", 1.20).value
        # A segment crossing the corridor, and the speed along it. Walking pace
        # is about 1.3 m/s; 0.45 m/s is a slow shuffle, chosen so the robot at
        # 0.5 m/s actually meets it rather than passing behind it.
        self.x = self.declare_parameter("x_m", 0.0).value
        self.y_min = self.declare_parameter("y_min_m", -1.1).value
        self.y_max = self.declare_parameter("y_max_m", 1.1).value
        self.speed = self.declare_parameter("speed_m_s", 0.45).value
        self.rate_hz = self.declare_parameter("rate_hz", 20.0).value

        self.gz = GzNode()
        self.t0: float | None = None
        self.spawned = False
        self.set_pose_failures = 0

        self.pose_pub = self.create_publisher(
            PoseStamped, "/arb/ground_truth/obstacle_pose", 10)
        self.timer = self.create_timer(1.0 / float(self.rate_hz), self._tick)
        self.get_logger().info(
            f"moving obstacle '{self.name}' r={self.radius} m at x={self.x} m, "
            f"y in [{self.y_min}, {self.y_max}] at {self.speed} m/s")

    # ------------------------------------------------------------------ gz --
    def _spawn(self) -> bool:
        req = EntityFactory()
        req.sdf = OBSTACLE_SDF.format(
            name=self.name, x=self.x, y=self.y_min,
            z=self.height / 2.0, radius=self.radius, height=self.height)
        req.allow_renaming = False
        ok, reply = self.gz.request(
            f"/world/{self.world}/create", req, EntityFactory, Boolean, 3000)
        if not ok or not reply.data:
            self.get_logger().error(
                f"could not spawn '{self.name}' into world '{self.world}' -- "
                f"is the simulator running?")
            return False
        self.get_logger().info(f"spawned '{self.name}'")
        return True

    def _set_pose(self, y: float) -> None:
        req = Pose()
        req.name = self.name
        req.position.x = float(self.x)
        req.position.y = float(y)
        req.position.z = float(self.height / 2.0)
        req.orientation.w = 1.0
        ok, reply = self.gz.request(
            f"/world/{self.world}/set_pose", req, Pose, Boolean, 200)
        if not ok or not reply.data:
            self.set_pose_failures += 1
            if self.set_pose_failures in (1, 50, 500):
                self.get_logger().warn(
                    f"set_pose rejected ({self.set_pose_failures} so far)")

    # ---------------------------------------------------------------- time --
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self) -> None:
        now = self._now()
        if now <= 0.0:
            self.get_logger().warn("waiting for /clock", throttle_duration_sec=5.0)
            return
        if not self.spawned:
            self.spawned = self._spawn()
            if not self.spawned:
                return
            self.t0 = now
            return

        # A triangle wave: constant speed out and back, so the obstacle's
        # position is an exact function of elapsed simulated time and repeats
        # identically in every run.
        span = abs(self.y_max - self.y_min)
        period = 2.0 * span / max(1e-6, self.speed)
        phase = math.fmod(now - self.t0, period)
        travelled = self.speed * phase
        y = (self.y_min + travelled if travelled <= span
             else self.y_max - (travelled - span))
        self._set_pose(y)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(self.x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(self.height / 2.0)
        msg.pose.orientation.w = 1.0
        self.pose_pub.publish(msg)


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    node = None
    try:
        node = MovingObstacle()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
