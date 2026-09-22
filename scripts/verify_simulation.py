#!/usr/bin/env python3
"""Verify that the Phase 1 simulation actually produces usable data.

A topic existing is not evidence that a sensor works. This script subscribes to
every bridged topic, collects messages for a fixed window, and checks the
*content*: publication rate, frame ids, value ranges, and whether the noise
model is actually injecting noise. It then drives the robot with an open-loop
command and confirms the platform moved the way the kinematics predict.

Run it with the simulation already up::

    ros2 launch arb_simulation simulation.launch.py headless:=true rviz:=false
    python3 scripts/verify_simulation.py

Exit status is 0 only if every check passes, so it can gate CI.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass, field

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, JointState, LaserScan

#: Sensor streams are published BEST_EFFORT by the bridge; a RELIABLE subscriber
#: would be QoS-incompatible and silently receive nothing. This is the single
#: most common "my topic exists but echo shows nothing" cause in ROS 2.
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=50,
)
RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=100,
)

#: Distance from the LiDAR origin to the front face of the chassis. The scan
#: measures from the sensor, the robot collides with its bumper.
FRONT_OVERHANG_M = 0.06

#: How much clear space to leave in front of the robot after the test drive.
SAFETY_MARGIN_M = 0.40

#: Half-width of the forward sector averaged to estimate clearance, in beams.
#: A single beam can slip past a thin obstacle; a narrow sector cannot.
FORWARD_SECTOR_BEAMS = 4


@dataclass
class Stream:
    """Accumulator for one topic."""

    name: str
    expected_hz: float
    stamps: list[float] = field(default_factory=list)
    frames: set[str] = field(default_factory=set)
    count: int = 0

    def note(self, stamp_s: float, frame: str) -> None:
        self.stamps.append(stamp_s)
        self.frames.add(frame)
        self.count += 1

    @property
    def measured_hz(self) -> float:
        if len(self.stamps) < 2:
            return 0.0
        span = self.stamps[-1] - self.stamps[0]
        return (len(self.stamps) - 1) / span if span > 0 else 0.0


class Verifier(Node):
    """Collects sensor data, then runs an open-loop motion check."""

    def __init__(self, window_s: float, wheel_radius: float) -> None:
        super().__init__("arb_simulation_verifier")
        self.window_s = window_s

        self.imu = Stream("/imu/data_raw", 200.0)
        self.joints = Stream("/joint_states", 50.0)
        self.scan = Stream("/scan_raw", 10.0)
        self.truth = Stream("/arb/ground_truth/odom", 100.0)

        self.gyro_z: list[float] = []
        self.accel_x: list[float] = []
        self.wheel_pos: dict[str, float] = {}
        self.joint_names: list[str] = []
        self.last_scan: LaserScan | None = None
        self.last_truth: Odometry | None = None
        self.wheel_radius = wheel_radius

        self.create_subscription(Imu, "/imu/data_raw", self._on_imu, SENSOR_QOS)
        self.create_subscription(JointState, "/joint_states", self._on_joints, SENSOR_QOS)
        self.create_subscription(LaserScan, "/scan_raw", self._on_scan, SENSOR_QOS)
        self.create_subscription(Odometry, "/arb/ground_truth/odom", self._on_truth, SENSOR_QOS)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", RELIABLE_QOS)

    # --- callbacks ---------------------------------------------------------
    @staticmethod
    def _stamp(msg) -> float:
        h = msg.header.stamp
        return h.sec + h.nanosec * 1e-9

    def _on_imu(self, msg: Imu) -> None:
        self.imu.note(self._stamp(msg), msg.header.frame_id)
        self.gyro_z.append(msg.angular_velocity.z)
        self.accel_x.append(msg.linear_acceleration.x)

    def _on_joints(self, msg: JointState) -> None:
        self.joints.note(self._stamp(msg), msg.header.frame_id or "(none)")
        self.joint_names = list(msg.name)
        for name, pos in zip(msg.name, msg.position):
            self.wheel_pos[name] = pos

    def _on_scan(self, msg: LaserScan) -> None:
        self.scan.note(self._stamp(msg), msg.header.frame_id)
        self.last_scan = msg

    def _on_truth(self, msg: Odometry) -> None:
        self.truth.note(self._stamp(msg), msg.header.frame_id)
        self.last_truth = msg

    # --- helpers -----------------------------------------------------------
    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def publish_cmd(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def forward_clearance(self) -> float:
        """Minimum range in a narrow sector straight ahead of the robot.

        The scan starts at ``angle_min`` and steps by ``angle_increment``, so
        the beam pointing along +x (the robot's forward axis) is at index
        ``-angle_min / angle_increment``. Hard-coding "index 0 is forward" is a
        common bug: for a scan spanning -pi..+pi, index 0 points BACKWARDS.
        """
        scan = self.last_scan
        assert scan is not None
        forward_idx = int(round(-scan.angle_min / scan.angle_increment))
        n = len(scan.ranges)
        window = [
            scan.ranges[(forward_idx + k) % n]
            for k in range(-FORWARD_SECTOR_BEAMS, FORWARD_SECTOR_BEAMS + 1)
        ]
        finite = [r for r in window if math.isfinite(r) and r >= scan.range_min]
        return min(finite) if finite else float(scan.range_max)

    def truth_pose(self) -> tuple[float, float, float]:
        assert self.last_truth is not None
        p = self.last_truth.pose.pose.position
        q = self.last_truth.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return p.x, p.y, yaw


def _load_wheel_radius() -> float:
    """Read the wheel radius from configs/robot.yaml, the single source of truth.

    Hard-coding 0.06 here would make this script agree with a stale value after
    someone changes the robot, which is the exact class of bug the config file
    exists to prevent.
    """
    import pathlib

    import yaml

    cfg_path = pathlib.Path(__file__).resolve().parents[1] / "configs" / "robot.yaml"
    return float(yaml.safe_load(cfg_path.read_text())["robot"]["drive"]["wheel_radius_m"])


def _stdev(xs: list[float]) -> float:
    if len(xs) < 2:
        return 0.0
    mean = sum(xs) / len(xs)
    return math.sqrt(sum((x - mean) ** 2 for x in xs) / (len(xs) - 1))


class Report:
    """Collects pass/fail lines and renders them."""

    def __init__(self) -> None:
        self.rows: list[tuple[bool, str, str]] = []

    def check(self, ok: bool, label: str, detail: str) -> bool:
        self.rows.append((ok, label, detail))
        return ok

    def render(self) -> bool:
        width = max(len(label) for _, label, _ in self.rows)
        for ok, label, detail in self.rows:
            mark = "PASS" if ok else "FAIL"
            print(f"  [{mark}] {label.ljust(width)}  {detail}")
        return all(ok for ok, _, _ in self.rows)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--window", type=float, default=6.0,
                    help="seconds of passive data collection")
    ap.add_argument("--drive-time", type=float, default=4.0,
                    help="seconds of open-loop driving")
    ap.add_argument("--rate-tolerance", type=float, default=0.15,
                    help="allowed fractional deviation from the configured rate")
    args = ap.parse_args(argv)

    wheel_radius = _load_wheel_radius()

    rclpy.init()
    node = Verifier(args.window, wheel_radius)
    rep = Report()
    try:
        print(f"\ncollecting for {args.window:.0f} s ...")
        node.spin_for(args.window)

        print("\n=============== SENSOR STREAMS ===============")
        for stream, want_frame in (
            (node.imu, "imu_link"),
            (node.joints, None),
            (node.scan, "lidar_link"),
            (node.truth, None),
        ):
            hz = stream.measured_hz
            lo = stream.expected_hz * (1 - args.rate_tolerance)
            hi = stream.expected_hz * (1 + args.rate_tolerance)
            rep.check(
                stream.count > 0 and lo <= hz <= hi,
                f"{stream.name} rate",
                f"{hz:7.2f} Hz (expected {stream.expected_hz:.0f}, n={stream.count})",
            )
            if want_frame is not None:
                rep.check(
                    stream.frames == {want_frame},
                    f"{stream.name} frame_id",
                    f"{sorted(stream.frames)} (expected ['{want_frame}'])",
                )

        print("\n=============== IMU CONTENT ===============")
        gz_sd = _stdev(node.gyro_z)
        ax_mean = sum(node.accel_x) / len(node.accel_x) if node.accel_x else float("nan")
        rep.check(gz_sd > 1e-5, "gyro z noise present",
                  f"stdev = {gz_sd:.6f} rad/s (noise model active)")
        rep.check(abs(ax_mean) < 0.5, "accel x at rest",
                  f"mean = {ax_mean:+.4f} m/s^2 (robot stationary)")

        print("\n=============== WHEEL ENCODERS ===============")
        rep.check(
            set(node.joint_names) == {"left_wheel_joint", "right_wheel_joint"},
            "encoder joints",
            f"{sorted(node.joint_names)}",
        )

        print("\n=============== LIDAR CONTENT ===============")
        scan = node.last_scan
        if scan is None:
            rep.check(False, "scan received", "no LaserScan arrived")
        else:
            finite = [r for r in scan.ranges if math.isfinite(r)]
            rep.check(len(scan.ranges) == 360, "beam count",
                      f"{len(scan.ranges)} beams")
            rep.check(len(finite) > 0.5 * len(scan.ranges), "finite returns",
                      f"{len(finite)}/{len(scan.ranges)} beams hit geometry")
            if finite:
                rep.check(
                    min(finite) >= scan.range_min - 1e-3
                    and max(finite) <= scan.range_max + 1e-3,
                    "ranges within limits",
                    f"[{min(finite):.3f}, {max(finite):.3f}] m "
                    f"vs [{scan.range_min:.2f}, {scan.range_max:.2f}]",
                )
            # The span is deliberately one increment short of a full circle so
            # that the increment is exactly 2*pi/samples and no two beams point
            # the same way. See configs/sensors.yaml for the reasoning.
            span = scan.angle_max - scan.angle_min
            n = len(scan.ranges)
            expected_span = 2 * math.pi * (n - 1) / n
            rep.check(abs(span - expected_span) < 1e-4, "angular span",
                      f"{math.degrees(span):.3f} deg "
                      f"(expected {math.degrees(expected_span):.3f} = 360*{n-1}/{n})")
            rep.check(abs(scan.angle_increment - 2 * math.pi / n) < 1e-9,
                      "beam spacing",
                      f"{math.degrees(scan.angle_increment):.5f} deg "
                      f"(expected exactly {360.0 / n:.5f})")

        print("\n=============== OPEN-LOOP MOTION ===============")
        if node.last_truth is None or node.last_scan is None:
            rep.check(False, "ground truth / scan", "missing data, cannot drive")
        else:
            # How far can we actually drive? The spawn pose sits in a corridor
            # with a pillar 1.5 m ahead, so a fixed "drive for 4 seconds" runs
            # the robot into it: the wheels keep turning while the chassis is
            # blocked, and the resulting wheel-arc-versus-distance mismatch
            # looks exactly like 15 % wheel slip. It is not slip, it is a wall.
            #
            # The clearance is therefore measured from the LiDAR first and the
            # command sized to fit, which also makes this check independent of
            # where in the world the robot happens to be.
            clearance = node.forward_clearance()
            usable = clearance - FRONT_OVERHANG_M - SAFETY_MARGIN_M
            rep.check(usable > 0.2, "forward clearance",
                      f"{clearance:.2f} m to the nearest obstacle ahead, "
                      f"{usable:.2f} m usable")

            v_cmd = 0.30
            distance = min(v_cmd * args.drive_time, max(usable, 0.0))
            drive_s = distance / v_cmd if v_cmd > 0 else 0.0

            x0, y0, th0 = node.truth_pose()
            left0 = node.wheel_pos.get("left_wheel_joint", 0.0)
            end = time.monotonic() + drive_s
            while time.monotonic() < end and rclpy.ok():
                node.publish_cmd(v_cmd, 0.0)
                node.spin_for(0.05)
            node.publish_cmd(0.0, 0.0)
            node.spin_for(1.5)
            x1, y1, th1 = node.truth_pose()
            left1 = node.wheel_pos.get("left_wheel_joint", 0.0)

            travelled = math.hypot(x1 - x0, y1 - y0)
            rep.check(travelled > 0.5 * distance, "robot translated",
                      f"{travelled:.3f} m (commanded ~{distance:.2f} m "
                      f"over {drive_s:.1f} s)")
            heading_drift = math.atan2(math.sin(th1 - th0), math.cos(th1 - th0))
            rep.check(abs(heading_drift) < 0.15, "straight line held",
                      f"heading drift {math.degrees(heading_drift):+.2f} deg")

            # Wheel arc length must agree with the distance actually travelled.
            # This is the first real cross-check between the encoder model and
            # the physics, and it is precisely the relation Phase 3 relies on:
            # if these two disagree, dead reckoning cannot work no matter how
            # the equations are written.
            wheel_r = node.wheel_radius
            arc = abs(left1 - left0) * wheel_r
            err = abs(arc - travelled) / travelled if travelled > 0 else 1.0
            rep.check(err < 0.05, "encoder arc vs truth",
                      f"wheel arc {arc:.3f} m vs truth {travelled:.3f} m "
                      f"({err * 100:.1f}% disagreement)")

            # 2) Rotate in place.
            #
            # Total rotation is ACCUMULATED from wrapped increments rather than
            # taken as one end-minus-start difference. Heading lives on the
            # circle S^1: a 4 s turn at 0.8 rad/s is 3.2 rad = 183 deg, and a
            # single subtraction wraps that to -177 deg, making a perfectly
            # correct counter-clockwise turn look clockwise. This check failed
            # exactly that way the first time it ran.
            #
            # Summing per-sample wrapped deltas recovers the true unwrapped
            # angle, valid as long as each individual step stays below pi, which
            # at 20 Hz sampling and 2 rad/s it always does. Same failure and same
            # fix as arb_core::normalizeAngle.
            w_cmd = 0.8
            _, _, prev = node.truth_pose()
            turned = 0.0
            end = time.monotonic() + args.drive_time
            while time.monotonic() < end and rclpy.ok():
                node.publish_cmd(0.0, w_cmd)
                node.spin_for(0.05)
                _, _, cur = node.truth_pose()
                turned += math.atan2(math.sin(cur - prev), math.cos(cur - prev))
                prev = cur
            node.publish_cmd(0.0, 0.0)
            node.spin_for(1.0)
            _, _, cur = node.truth_pose()
            turned += math.atan2(math.sin(cur - prev), math.cos(cur - prev))

            expected_turn = w_cmd * args.drive_time
            rep.check(turned > 0.5 * expected_turn, "robot rotated (CCW)",
                      f"{math.degrees(turned):+.1f} deg accumulated "
                      f"(open-loop expectation ~{math.degrees(expected_turn):+.0f} deg)")

        print("\n=============== RESULT ===============")
        ok = rep.render()
        print()
        print("ALL CHECKS PASSED" if ok else "SOME CHECKS FAILED")
        return 0 if ok else 1
    finally:
        try:
            node.publish_cmd(0.0, 0.0)
            node.spin_for(0.3)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
