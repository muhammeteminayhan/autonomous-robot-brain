#!/usr/bin/env python3
"""Verify the Phase 2 sensor preprocessing chain, including live fault injection.

Checks the *content* of the preprocessed streams, not merely their existence:

* rates and frame ids survive preprocessing
* the IMU carries an honest measurement covariance, and declares that it has
  no orientation estimate
* encoder positions lie on the encoder's count grid and are consistent with
  the reported velocities
* the scan contains exactly one sentinel for "no reading", never NaN or -inf
* setting a fault parameter at runtime actually changes the data, and the
  reported uncertainty changes with it

The last point is the one that matters for Phase 10. A fault model that
degrades the signal without widening the reported covariance teaches the
estimator nothing, and the filter keeps trusting a sensor that has failed.

Run with the simulation and the sensor nodes already up::

    ros2 launch arb_simulation simulation.launch.py headless:=true rviz:=false
    ros2 launch arb_sensors sensors.launch.py
    python3 scripts/verify_sensors.py
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys
import time

import rclpy
import yaml
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Imu, JointState, LaserScan

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=200,
)


class SensorVerifier(Node):
    """Collects the preprocessed streams and can set fault parameters."""

    def __init__(self) -> None:
        super().__init__("arb_sensor_verifier")
        self.imu: list[Imu] = []
        self.wheels: list[JointState] = []
        self.scans: list[LaserScan] = []

        self.create_subscription(Imu, "/arb/imu", self.imu.append, SENSOR_QOS)
        self.create_subscription(
            JointState, "/arb/wheel_states", self.wheels.append, SENSOR_QOS)
        self.create_subscription(LaserScan, "/arb/scan", self.scans.append, SENSOR_QOS)

        # NOTE: not `self._clients`. rclpy.node.Node keeps its own list of
        # service clients in that attribute, and shadowing it with a dict makes
        # the executor iterate over strings and destroy_node() index a dict by
        # integer. Both failures surface far from the assignment.
        self._param_clients = {
            name: self.create_client(SetParameters, f"/{name}/set_parameters")
            for name in ("imu_preprocessor", "encoder_preprocessor", "lidar_preprocessor")
        }

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def clear(self) -> None:
        self.imu.clear()
        self.wheels.clear()
        self.scans.clear()

    def set_param(self, node: str, name: str, value) -> bool:
        """Set one parameter on a running node and wait for the result."""
        client = self._param_clients[node]
        if not client.wait_for_service(timeout_sec=5.0):
            return False
        pv = ParameterValue()
        if isinstance(value, bool):
            pv.type = ParameterType.PARAMETER_BOOL
            pv.bool_value = value
        else:
            pv.type = ParameterType.PARAMETER_DOUBLE
            pv.double_value = float(value)
        req = SetParameters.Request(parameters=[Parameter(name=name, value=pv)])
        future = client.call_async(req)
        end = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done():
            return False
        return all(r.successful for r in future.result().results)


def rate_of(msgs) -> float:
    if len(msgs) < 2:
        return 0.0
    def stamp(m):
        return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    span = stamp(msgs[-1]) - stamp(msgs[0])
    return (len(msgs) - 1) / span if span > 0 else 0.0


def stdev(xs) -> float:
    if len(xs) < 2:
        return 0.0
    m = sum(xs) / len(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


class Report:
    def __init__(self) -> None:
        self.rows: list[tuple[bool, str, str]] = []

    def check(self, ok: bool, label: str, detail: str) -> bool:
        self.rows.append((bool(ok), label, detail))
        return bool(ok)

    def render(self) -> bool:
        width = max(len(r[1]) for r in self.rows)
        for ok, label, detail in self.rows:
            print(f"  [{'PASS' if ok else 'FAIL'}] {label.ljust(width)}  {detail}")
        return all(r[0] for r in self.rows)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--window", type=float, default=6.0)
    args = ap.parse_args(argv)

    cfg = yaml.safe_load((REPO_ROOT / "configs" / "sensors.yaml").read_text())
    robot = yaml.safe_load((REPO_ROOT / "configs" / "robot.yaml").read_text())["robot"]
    counts = cfg["wheel_encoder"]["counts_per_revolution"] * \
        cfg["wheel_encoder"]["quadrature_multiplier"]
    resolution = 2.0 * math.pi / counts

    rclpy.init()
    node = SensorVerifier()
    rep = Report()
    try:
        print(f"\ncollecting nominal data for {args.window:.0f} s ...")
        node.spin_for(args.window)

        # ------------------------------------------------ streams
        print("\n=============== PREPROCESSED STREAMS ===============")
        for msgs, name, want_hz, want_frame in (
            (node.imu, "/arb/imu", cfg["imu"]["rate_hz"], "imu_link"),
            (node.wheels, "/arb/wheel_states", cfg["wheel_encoder"]["rate_hz"], None),
            (node.scans, "/arb/scan", cfg["lidar"]["rate_hz"], "lidar_link"),
        ):
            hz = rate_of(msgs)
            rep.check(len(msgs) > 0 and abs(hz - want_hz) < 0.15 * want_hz,
                      f"{name} rate",
                      f"{hz:7.2f} Hz (expected {want_hz:.0f}, n={len(msgs)})")
            if want_frame and msgs:
                frames = {m.header.frame_id for m in msgs}
                rep.check(frames == {want_frame}, f"{name} frame_id",
                          f"{sorted(frames)}")

        # ------------------------------------------------ IMU content
        print("\n=============== IMU ===============")
        if node.imu:
            m = node.imu[-1]
            gyro_var = m.angular_velocity_covariance[0]
            sigma = math.sqrt(gyro_var) if gyro_var > 0 else 0.0
            expected = cfg["imu"]["gyroscope"]["noise_density_rad_s_sqrt_hz"] * \
                math.sqrt(cfg["imu"]["rate_hz"])
            rep.check(gyro_var > 0.0, "gyro covariance filled",
                      f"var={gyro_var:.3e} -> sigma={sigma:.6f} rad/s")
            rep.check(abs(sigma - expected) < 1e-6, "covariance matches the spec",
                      f"{sigma:.6f} vs noise_density*sqrt(rate)={expected:.6f}")
            rep.check(m.orientation_covariance[0] == -1.0,
                      "orientation declared absent",
                      f"orientation_covariance[0]={m.orientation_covariance[0]}")
            measured = stdev([x.angular_velocity.z for x in node.imu])
            rep.check(measured > 0.0, "gyro noise present",
                      f"measured stdev={measured:.6f} rad/s (robot stationary)")

        # ------------------------------------------------ encoder content
        print("\n=============== WHEEL ENCODERS ===============")
        if len(node.wheels) > 2:
            off_grid = 0
            for w in node.wheels:
                for p in w.position:
                    if abs(p / resolution - round(p / resolution)) > 1e-6:
                        off_grid += 1
            rep.check(off_grid == 0, "positions lie on the count grid",
                      f"{off_grid} of {2 * len(node.wheels)} samples off the "
                      f"{resolution * 1e3:.3f} mrad grid")

            worst = 0.0
            for a, b in zip(node.wheels[:-1], node.wheels[1:]):
                ta = a.header.stamp.sec + a.header.stamp.nanosec * 1e-9
                tb = b.header.stamp.sec + b.header.stamp.nanosec * 1e-9
                dt = tb - ta
                if dt <= 0:
                    continue
                for i in range(2):
                    predicted = a.position[i] + b.velocity[i] * dt
                    worst = max(worst, abs(predicted - b.position[i]))
            rep.check(worst < 1e-6, "velocity integrates to position",
                      f"worst mismatch {worst:.3e} rad")

            rep.check(all(e > 0.0 for w in node.wheels for e in w.effort),
                      "velocity sigma published",
                      f"effort[0]={node.wheels[-1].effort[0]:.4f} rad/s")

        # ------------------------------------------------ scan content
        print("\n=============== LIDAR ===============")
        if node.scans:
            bad = 0
            for s in node.scans:
                for r in s.ranges:
                    if math.isnan(r) or (math.isinf(r) and r < 0):
                        bad += 1
            rep.check(bad == 0, "one sentinel for no-reading",
                      f"{bad} NaN or -inf values across {len(node.scans)} scans")
            finite = sum(1 for r in node.scans[-1].ranges if math.isfinite(r))
            rep.check(finite > 0.8 * len(node.scans[-1].ranges),
                      "beams survive preprocessing",
                      f"{finite}/{len(node.scans[-1].ranges)} finite")

        # ------------------------------------------------ fault injection
        print("\n=============== RUNTIME FAULT INJECTION ===============")
        baseline_gyro = stdev([x.angular_velocity.z for x in node.imu])

        ok = node.set_param("imu_preprocessor", "fault.noise_scale", 10.0)
        rep.check(ok, "set imu fault.noise_scale=10", "parameter accepted" if ok
                  else "service call failed")
        node.clear()
        node.spin_for(4.0)
        if node.imu:
            degraded = stdev([x.angular_velocity.z for x in node.imu])
            ratio = degraded / baseline_gyro if baseline_gyro > 0 else 0.0
            rep.check(6.0 < ratio < 15.0, "noise actually increased 10x",
                      f"stdev {baseline_gyro:.6f} -> {degraded:.6f} ({ratio:.1f}x)")
            var = node.imu[-1].angular_velocity_covariance[0]
            expected_var = (cfg["imu"]["gyroscope"]["noise_density_rad_s_sqrt_hz"] *
                            math.sqrt(cfg["imu"]["rate_hz"]) * 10.0) ** 2
            rep.check(abs(var - expected_var) / expected_var < 1e-6,
                      "reported covariance widened too",
                      f"var={var:.3e} (expected {expected_var:.3e}, 100x nominal)")
        node.set_param("imu_preprocessor", "fault.noise_scale", 1.0)

        # encoder drift
        node.clear()
        node.spin_for(1.5)
        start = node.wheels[-1] if node.wheels else None
        ok = node.set_param("encoder_preprocessor", "fault.right_bias_rad_s", 0.20)
        rep.check(ok, "set encoder fault.right_bias_rad_s=0.2", "parameter accepted")
        node.clear()
        node.spin_for(4.0)
        if start and len(node.wheels) > 1:
            t0 = start.header.stamp.sec + start.header.stamp.nanosec * 1e-9
            t1 = node.wheels[-1].header.stamp.sec + \
                node.wheels[-1].header.stamp.nanosec * 1e-9
            d_left = node.wheels[-1].position[0] - start.position[0]
            d_right = node.wheels[-1].position[1] - start.position[1]
            expected = 0.20 * (t1 - t0)
            rep.check(abs((d_right - d_left) - expected) < 0.25 * expected,
                      "drift accumulates on one wheel only",
                      f"right-left = {d_right - d_left:+.4f} rad over "
                      f"{t1 - t0:.2f} s (expected {expected:+.4f})")
        node.set_param("encoder_preprocessor", "fault.right_bias_rad_s", 0.0)

        # lidar dropout
        ok = node.set_param("lidar_preprocessor", "fault.dropout_ratio", 0.6)
        rep.check(ok, "set lidar fault.dropout_ratio=0.6", "parameter accepted")
        node.clear()
        node.spin_for(3.0)
        if node.scans:
            kept = sum(1 for s in node.scans for r in s.ranges if math.isfinite(r))
            total = sum(len(s.ranges) for s in node.scans)
            frac = kept / total if total else 0.0
            rep.check(0.3 < frac < 0.5, "60 % of beams dropped",
                      f"{frac * 100:.1f}% of beams survived across "
                      f"{len(node.scans)} scans")
        node.set_param("lidar_preprocessor", "fault.dropout_ratio", 0.0)

        # outage
        ok = node.set_param("lidar_preprocessor", "fault.outage", True)
        rep.check(ok, "set lidar fault.outage=true", "parameter accepted")
        node.clear()
        node.spin_for(2.5)
        rep.check(len(node.scans) == 0, "outage stops the stream",
                  f"{len(node.scans)} scans received during the outage")
        node.set_param("lidar_preprocessor", "fault.outage", False)
        node.clear()
        node.spin_for(2.0)
        rep.check(len(node.scans) > 5, "stream recovers after the outage",
                  f"{len(node.scans)} scans after clearing the fault")

        # a rejected parameter must not silently apply
        bad_ok = node.set_param("lidar_preprocessor", "fault.dropout_ratio", 5.0)
        rep.check(not bad_ok, "invalid fault value rejected",
                  "dropout_ratio=5.0 refused" if not bad_ok else "ACCEPTED, should not be")

        print("\n=============== RESULT ===============")
        passed = rep.render()
        print()
        print("ALL CHECKS PASSED" if passed else "SOME CHECKS FAILED")
        return 0 if passed else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
