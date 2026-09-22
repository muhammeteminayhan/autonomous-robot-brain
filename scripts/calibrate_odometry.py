#!/usr/bin/env python3
"""Calibrate the two systematic odometry parameters against ground truth.

WHY THIS SCRIPT EXISTS
======================
Differential-drive odometry has exactly two systematic error parameters, and
neither of them can be measured reliably with a ruler:

* the effective WHEEL RADIUS r, which sets every distance, and
* the effective TRACK WIDTH L, which sets every heading.

The URDF says the wheels are 0.350 m apart, because that is where their centre
planes are. The physics disagrees. A cylindrical wheel resting on a plane
touches it along a LINE, not at a point, and the traction that turns the robot
acts somewhere inside that line rather than at the centre plane. Measured on
this robot the effective track width is about 0.318 m -- close to the distance
between the wheels' INNER edges (0.315 m) -- a 10 % error that produces no
distance error at all and 9 degrees of heading error on every 90 degree turn.

Over two laps of a rectangle that accumulated to 72 degrees of heading error
and a metre of position error, while the odometry cheerfully reported closing
the loop to within 5 cm. Every number was self-consistent and every number was
wrong.

So L is a CALIBRATION PARAMETER, not a geometric measurement. Real robots are
calibrated the same way, and skipping it is one of the most common reasons a
mobile robot "drifts" in a way no amount of filter tuning can fix: a systematic
error is not noise, and no Kalman filter removes a bias it has not been told
about.

METHOD
======
Two decoupled experiments, because the two parameters affect different things:

1. Rotation: spin in place through several full turns, in BOTH directions, and
   compare the total ground-truth rotation with what the encoders imply.
   Averaging the two directions cancels any left/right asymmetry, which would
   otherwise be absorbed into L and reappear as a curve on straight runs.

       L_effective = L_nominal * (odometry rotation / true rotation)

2. Translation: drive straight and compare distances.

       r_effective = r_nominal * (true distance / odometry distance)

On real hardware ground truth is not available, and the standard substitute is
UMBmark (Borenstein & Feng, 1996): drive a 4 x 4 m square clockwise and
anticlockwise, measure the two end-point errors with a tape measure, and solve
the same two unknowns from the pattern they make. The arithmetic below is the
same; only the source of truth differs.

Usage::

    ros2 launch arb_simulation simulation.launch.py headless:=true \
        rviz:=false spawn:=odometry_test
    ros2 launch arb_sensors sensors.launch.py
    ros2 launch arb_localization odometry.launch.py
    python3 scripts/calibrate_odometry.py
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=200,
)
COMMAND_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

SPIN_SPEED = 0.8          # rad/s
SPIN_TURNS = 3.0          # full revolutions per direction
STRAIGHT_SPEED = 0.30     # m/s
STRAIGHT_DISTANCE = 1.2   # m, kept short so it fits the clearance disc


def yaw_of(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Calibrator(Node):
    def __init__(self) -> None:
        super().__init__("arb_odometry_calibrator")
        self.odom: Odometry | None = None
        self.truth: Odometry | None = None
        self.create_subscription(Odometry, "/arb/odom", self._on_odom, SENSOR_QOS)
        self.create_subscription(
            Odometry, "/arb/ground_truth/odom", self._on_truth, SENSOR_QOS)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", COMMAND_QOS)

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def _on_truth(self, msg: Odometry) -> None:
        self.truth = msg

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

    def publish(self, v: float, w: float) -> None:
        m = Twist()
        m.linear.x = v
        m.angular.z = w
        self.cmd.publish(m)

    def run(self, v: float, w: float, duration: float):
        """Command a twist and return the UNWRAPPED rotation and the distance.

        Rotation is accumulated from wrapped per-sample increments. A single
        end-minus-start subtraction cannot represent three full turns: it wraps
        to something in [-pi, pi) and silently discards the revolutions, which
        is exactly the quantity being measured here.
        """
        assert self.odom is not None and self.truth is not None
        o_prev, t_prev = yaw_of(self.odom), yaw_of(self.truth)
        ox0, oy0 = self.odom.pose.pose.position.x, self.odom.pose.pose.position.y
        tx0, ty0 = self.truth.pose.pose.position.x, self.truth.pose.pose.position.y
        o_total = t_total = 0.0

        end = time.monotonic() + duration
        while time.monotonic() < end and rclpy.ok():
            self.publish(v, w)
            self.spin_for(0.02)
            o_cur, t_cur = yaw_of(self.odom), yaw_of(self.truth)
            o_total += math.atan2(math.sin(o_cur - o_prev), math.cos(o_cur - o_prev))
            t_total += math.atan2(math.sin(t_cur - t_prev), math.cos(t_cur - t_prev))
            o_prev, t_prev = o_cur, t_cur

        self.publish(0.0, 0.0)
        self.spin_for(1.5)
        o_cur, t_cur = yaw_of(self.odom), yaw_of(self.truth)
        o_total += math.atan2(math.sin(o_cur - o_prev), math.cos(o_cur - o_prev))
        t_total += math.atan2(math.sin(t_cur - t_prev), math.cos(t_cur - t_prev))

        o_dist = math.hypot(self.odom.pose.pose.position.x - ox0,
                            self.odom.pose.pose.position.y - oy0)
        t_dist = math.hypot(self.truth.pose.pose.position.x - tx0,
                            self.truth.pose.pose.position.y - ty0)
        return o_total, t_total, o_dist, t_dist


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--apply", action="store_true",
                    help="write the calibrated values into configs/robot.yaml")
    args = ap.parse_args(argv)

    robot_cfg_path = REPO_ROOT / "configs" / "robot.yaml"
    drive = yaml.safe_load(robot_cfg_path.read_text())["robot"]["drive"]
    L_nominal = float(drive["wheel_separation_m"])
    r_nominal = float(drive["wheel_radius_m"])

    rclpy.init()
    node = Calibrator()
    try:
        print("waiting for /arb/odom and /arb/ground_truth/odom ...")
        deadline = time.monotonic() + 20.0
        while (node.odom is None or node.truth is None) and \
                time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.odom is None or node.truth is None:
            print("timed out; is the stack running?", file=sys.stderr)
            return 1

        spin_time = SPIN_TURNS * 2.0 * math.pi / SPIN_SPEED
        print(f"\n[1/3] spinning +{SPIN_TURNS} turns ({spin_time:.0f} s) ...")
        o_ccw, t_ccw, _, _ = node.run(0.0, SPIN_SPEED, spin_time)
        print(f"      odometry {math.degrees(o_ccw):8.2f} deg | "
              f"truth {math.degrees(t_ccw):8.2f} deg")

        print(f"[2/3] spinning -{SPIN_TURNS} turns ({spin_time:.0f} s) ...")
        o_cw, t_cw, _, _ = node.run(0.0, -SPIN_SPEED, spin_time)
        print(f"      odometry {math.degrees(o_cw):8.2f} deg | "
              f"truth {math.degrees(t_cw):8.2f} deg")

        straight_time = STRAIGHT_DISTANCE / STRAIGHT_SPEED
        print(f"[3/3] driving straight {STRAIGHT_DISTANCE} m ...")
        _, _, o_dist, t_dist = node.run(STRAIGHT_SPEED, 0.0, straight_time)
        print(f"      odometry {o_dist:.4f} m | truth {t_dist:.4f} m")

        node.publish(0.0, 0.0)
        node.spin_for(0.5)

        # Average the magnitudes of the two directions: a left/right asymmetry
        # cancels, leaving only the symmetric track-width error.
        odom_rot = 0.5 * (abs(o_ccw) + abs(o_cw))
        true_rot = 0.5 * (abs(t_ccw) + abs(t_cw))
        if true_rot < 1e-6 or o_dist < 1e-6:
            print("the robot did not move enough to calibrate", file=sys.stderr)
            return 1

        L_eff = L_nominal * (odom_rot / true_rot)
        r_eff = r_nominal * (t_dist / o_dist)

        # Asymmetry between the two spin directions, as a fraction. A large
        # value means one wheel is doing more than the other, which a single
        # track-width number cannot represent.
        asym = abs(abs(t_ccw) - abs(t_cw)) / true_rot

        print()
        print("=" * 68)
        print("ODOMETRY CALIBRATION")
        print("=" * 68)
        print(f"  rotation ratio (odom/true)   {odom_rot / true_rot:8.4f}")
        print(f"  direction asymmetry          {100 * asym:8.2f} %")
        print(f"  distance ratio (true/odom)   {t_dist / o_dist:8.4f}")
        print()
        print(f"  wheel_separation_m  {L_nominal:.4f}  ->  {L_eff:.4f}"
              f"   ({100 * (L_eff / L_nominal - 1):+.2f} %)")
        print(f"  wheel_radius_m      {r_nominal:.5f} ->  {r_eff:.5f}"
              f"   ({100 * (r_eff / r_nominal - 1):+.2f} %)")
        print()
        if asym > 0.02:
            print("  WARNING: the two spin directions disagree by more than 2 %.")
            print("  A single track width cannot capture that; it points to a")
            print("  left/right asymmetry (unequal wheel radii or friction).")
        print("  NOTE: the URDF geometry is NOT wrong. base_link to wheel centre")
        print("  really is half of", f"{L_nominal:.3f} m.", "The odometry's L is an")
        print("  EFFECTIVE parameter describing where traction actually acts on a")
        print("  line contact, and it belongs in the localisation config, not in")
        print("  the robot model.")
        print()
        print("  Apply with:")
        print("    ros2 param set /wheel_odometry wheel_separation_m "
              f"{L_eff:.4f}")
        print("  or permanently in "
              "ros2_ws/src/arb_localization/config/localization.yaml")
        return 0
    finally:
        try:
            node.publish(0.0, 0.0)
            node.spin_for(0.3)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
