#!/usr/bin/env python3
"""Measure how fast wheel odometry drifts, and plot it.

This is the point of Phase 3. Dead reckoning is trivially easy to write and
trivially easy to believe; what matters is knowing how quickly it stops being
true, and that is a number you have to measure rather than assume.

The script drives an open-loop manoeuvre, records the odometry estimate
alongside ground truth, and reports:

* position error against distance travelled (the honest x-axis for odometry;
  error grows with distance, not with elapsed time)
* heading error, which is the error that actually kills the estimate, because
  a heading error turns into a position error that grows *after* it happens
* whether the reported covariance is consistent with the observed error, i.e.
  whether the filter's own claim of uncertainty can be trusted

Ground truth is used here, and only here: this is an evaluation tool under
``scripts/``, not part of the estimation pipeline. See docs/architecture.md
section 4.

Run with the simulation, the sensor nodes and wheel odometry up::

    ros2 launch arb_simulation simulation.launch.py headless:=true rviz:=false
    ros2 launch arb_sensors sensors.launch.py
    ros2 launch arb_localization odometry.launch.py
    python3 scripts/evaluate_odometry.py
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
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

#: Command speeds for the evaluation route.
LINEAR_SPEED = 0.25          # m/s
ANGULAR_SPEED = 0.60         # rad/s
LONG_LEG_M = 1.5
SHORT_LEG_M = 1.0
LAPS = 2

#: Minimum forward clearance, in metres, below which the run is ABORTED.
#:
#: An open-loop route that drives into something does not measure odometry
#: drift: the wheels keep turning against a stationary chassis, the encoders
#: keep counting, and the estimate runs away. The first version of this script
#: did exactly that from the default spawn pose -- 1.5 m from a corridor pillar
#: -- and reported a 35 % drift that was entirely collision. An evaluation that
#: silently measures a crash is worse than one that refuses to run.
MIN_CLEARANCE_M = 0.45

#: Distance from the LiDAR origin to the front bumper.
FRONT_OVERHANG_M = 0.06


def build_route(laps: int = LAPS):
    """A closed rectangle, driven open loop, repeated `laps` times.

    A rectangle rather than a straight line because straight-line driving
    flatters odometry enormously: it exercises only the sum of the wheel
    arcs and leaves the heading, which is where the compounding error lives,
    almost untouched. Turns are what make dead reckoning fail.

    Returning to the start also gives a free second measurement: the gap
    between where the robot believes it ended and where it began is the
    loop-closure error, and it is independent of any frame alignment.
    """
    turn = (math.pi / 2.0) / ANGULAR_SPEED
    long_leg = LONG_LEG_M / LINEAR_SPEED
    short_leg = SHORT_LEG_M / LINEAR_SPEED
    route = [(0.0, 0.0, 2.0)]                       # settle
    for _ in range(laps):
        route += [
            (LINEAR_SPEED, 0.0, long_leg),
            (0.0, ANGULAR_SPEED, turn),
            (LINEAR_SPEED, 0.0, short_leg),
            (0.0, ANGULAR_SPEED, turn),
            (LINEAR_SPEED, 0.0, long_leg),
            (0.0, ANGULAR_SPEED, turn),
            (LINEAR_SPEED, 0.0, short_leg),
            (0.0, ANGULAR_SPEED, turn),
        ]
    route.append((0.0, 0.0, 2.0))                   # settle
    return route


DEFAULT_ROUTE = build_route()


def yaw_of(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class OdometryEvaluator(Node):
    """Drives the robot and records both estimates."""

    def __init__(self) -> None:
        super().__init__("arb_odometry_evaluator")
        self.odom: Odometry | None = None
        self.truth: Odometry | None = None
        self.scan: LaserScan | None = None
        self.samples: list[dict] = []
        self.aborted_reason: str | None = None

        self.create_subscription(Odometry, "/arb/odom", self._on_odom, SENSOR_QOS)
        self.create_subscription(
            Odometry, "/arb/ground_truth/odom", self._on_truth, SENSOR_QOS)
        self.create_subscription(LaserScan, "/arb/scan", self._on_scan, SENSOR_QOS)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", COMMAND_QOS)

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def _on_truth(self, msg: Odometry) -> None:
        self.truth = msg

    def _on_scan(self, msg: LaserScan) -> None:
        self.scan = msg

    def forward_clearance(self) -> float:
        """Smallest range in a narrow sector straight ahead, minus the bumper.

        The forward beam is at index -angle_min / angle_increment. Index 0
        points BACKWARDS for a scan spanning -pi..+pi, which is a mistake worth
        not making in a safety check.
        """
        scan = self.scan
        if scan is None or not scan.ranges:
            return float("inf")
        idx = int(round(-scan.angle_min / scan.angle_increment))
        n = len(scan.ranges)
        window = [scan.ranges[(idx + k) % n] for k in range(-4, 5)]
        finite = [r for r in window if math.isfinite(r) and r >= scan.range_min]
        return (min(finite) if finite else float(scan.range_max)) - FRONT_OVERHANG_M

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

    def publish(self, v: float, w: float) -> None:
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd.publish(msg)

    def record(self) -> None:
        if self.odom is None or self.truth is None:
            return
        o, t = self.odom, self.truth
        stamp = o.header.stamp.sec + o.header.stamp.nanosec * 1e-9
        if self.samples and stamp <= self.samples[-1]["t"]:
            return
        self.samples.append({
            "t": stamp,
            "ox": o.pose.pose.position.x,
            "oy": o.pose.pose.position.y,
            "oth": yaw_of(o),
            "tx": t.pose.pose.position.x,
            "ty": t.pose.pose.position.y,
            "tth": yaw_of(t),
            "sxx": o.pose.covariance[0],
            "syy": o.pose.covariance[7],
            "sthth": o.pose.covariance[35],
        })

    def drive_route(self, route, settle_hz: float = 50.0) -> bool:
        """Drive the route. Returns False if it was aborted for safety."""
        for v, w, duration in route:
            end = time.monotonic() + duration
            while time.monotonic() < end and rclpy.ok():
                # Only forward motion can collide; a turn in place cannot, and
                # refusing to turn because a wall is ahead would deadlock the
                # route exactly where turning is the way out.
                if v > 0.0:
                    clearance = self.forward_clearance()
                    if clearance < MIN_CLEARANCE_M:
                        self.publish(0.0, 0.0)
                        self.spin_for(1.0)
                        self.aborted_reason = (
                            f"forward clearance fell to {clearance:.2f} m "
                            f"(limit {MIN_CLEARANCE_M:.2f} m)")
                        return False
                self.publish(v, w)
                self.spin_for(1.0 / settle_hz)
                self.record()
        self.publish(0.0, 0.0)
        self.spin_for(1.5)
        self.record()
        return True


def analyse(samples: list[dict]) -> dict:
    """Errors are computed in the odometry's OWN starting frame.

    Odometry starts at its own origin; ground truth starts wherever the robot
    was spawned. Comparing them directly would report the spawn offset as
    error. Aligning only the FIRST sample -- a rigid transform, no fitting --
    keeps the comparison honest: any later divergence is real drift, not a
    coordinate mismatch. Fitting the whole trajectory instead (Umeyama
    alignment) would quietly absorb part of the drift into the fit and
    flatter the estimator.
    """
    t = np.array([s["t"] for s in samples])
    t -= t[0]
    ox = np.array([s["ox"] for s in samples])
    oy = np.array([s["oy"] for s in samples])
    oth = np.array([s["oth"] for s in samples])
    tx = np.array([s["tx"] for s in samples])
    ty = np.array([s["ty"] for s in samples])
    tth = np.array([s["tth"] for s in samples])

    # Rigid alignment from the first sample only.
    dth = wrap(oth[0] - tth[0])
    c, s = math.cos(dth), math.sin(dth)
    tx_a = c * (tx - tx[0]) - s * (ty - ty[0]) + ox[0]
    ty_a = s * (tx - tx[0]) + c * (ty - ty[0]) + oy[0]
    tth_a = np.array([wrap(a + dth) for a in tth])

    ex = ox - tx_a
    ey = oy - ty_a
    eth = np.array([wrap(a - b) for a, b in zip(oth, tth_a)])
    epos = np.hypot(ex, ey)

    # Distance travelled, measured on GROUND TRUTH: the odometry's own notion
    # of distance is one of the things under test.
    steps = np.hypot(np.diff(tx), np.diff(ty))
    distance = np.concatenate([[0.0], np.cumsum(steps)])

    return {
        "t": t, "ox": ox, "oy": oy, "oth": oth,
        "tx": tx_a, "ty": ty_a, "tth": tth_a,
        "ex": ex, "ey": ey, "eth": eth, "epos": epos,
        "distance": distance,
        "sigma_pos": np.sqrt(np.maximum(
            0.0, np.array([s["sxx"] for s in samples]) +
            np.array([s["syy"] for s in samples]))),
        "sigma_th": np.sqrt(np.maximum(
            0.0, np.array([s["sthth"] for s in samples]))),
    }


def plot(a: dict, out: pathlib.Path) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))

    ax = axes[0][0]
    ax.plot(a["tx"], a["ty"], color="#2f9e44", lw=2.0, label="ground truth")
    ax.plot(a["ox"], a["oy"], color="#e03131", lw=1.6, ls="--", label="wheel odometry")
    ax.plot(a["tx"][0], a["ty"][0], "o", color="#1971c2", ms=9, label="start")
    ax.plot(a["tx"][-1], a["ty"][-1], "s", color="#2f9e44", ms=8)
    ax.plot(a["ox"][-1], a["oy"][-1], "s", color="#e03131", ms=8)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3, ls=":")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Trajectory: dead reckoning vs ground truth")
    ax.legend()

    ax = axes[0][1]
    ax.plot(a["distance"], a["epos"], color="#e03131", lw=1.8, label="position error")
    ax.plot(a["distance"], a["sigma_pos"], color="#1971c2", lw=1.4, ls="--",
            label=r"reported $1\sigma$")
    ax.plot(a["distance"], 2 * a["sigma_pos"], color="#1971c2", lw=1.0, ls=":",
            label=r"reported $2\sigma$")
    ax.grid(alpha=0.3, ls=":")
    ax.set_xlabel("distance travelled [m]")
    ax.set_ylabel("error [m]")
    ax.set_title("Position error grows with DISTANCE, not time")
    ax.legend()

    ax = axes[1][0]
    ax.plot(a["t"], np.degrees(a["eth"]), color="#e03131", lw=1.8,
            label="heading error")
    ax.plot(a["t"], np.degrees(a["sigma_th"]), color="#1971c2", lw=1.4, ls="--",
            label=r"reported $1\sigma$")
    ax.plot(a["t"], -np.degrees(a["sigma_th"]), color="#1971c2", lw=1.4, ls="--")
    ax.axhline(0.0, color="#888", lw=0.8)
    ax.grid(alpha=0.3, ls=":")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("heading error [deg]")
    ax.set_title("Heading error: the one that compounds")
    ax.legend()

    ax = axes[1][1]
    ax.plot(a["t"], a["ex"], color="#1971c2", lw=1.5, label="x error")
    ax.plot(a["t"], a["ey"], color="#f08c00", lw=1.5, label="y error")
    ax.axhline(0.0, color="#888", lw=0.8)
    ax.grid(alpha=0.3, ls=":")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("error [m]")
    ax.set_title("Per-axis error")
    ax.legend()

    fig.suptitle(
        "Phase 3: wheel odometry drift, measured against ground truth", fontsize=13)
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--output", type=pathlib.Path,
                    default=REPO_ROOT / "results" / "phase3" / "odometry_drift.png")
    ap.add_argument("--csv", type=pathlib.Path,
                    default=REPO_ROOT / "results" / "phase3" / "odometry_drift.csv")
    args = ap.parse_args(argv)

    rclpy.init()
    node = OdometryEvaluator()
    try:
        print("waiting for /arb/odom and /arb/ground_truth/odom ...")
        deadline = time.monotonic() + 20.0
        while (node.odom is None or node.truth is None) and \
                time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.odom is None or node.truth is None:
            print("timed out; is the stack running?", file=sys.stderr)
            return 1

        total = sum(d for _, _, d in DEFAULT_ROUTE)
        expected_m = LAPS * 2 * (LONG_LEG_M + SHORT_LEG_M)
        print(f"driving the open-loop route: {LAPS} laps of a "
              f"{LONG_LEG_M} x {SHORT_LEG_M} m rectangle, "
              f"{expected_m:.1f} m over {total:.0f} s ...")
        completed = node.drive_route(DEFAULT_ROUTE)
        print(f"collected {len(node.samples)} samples")
        if not completed:
            print(f"\nABORTED: {node.aborted_reason}", file=sys.stderr)
            print("The route ran out of room. Spawn the robot in open floor:",
                  file=sys.stderr)
            print("  ros2 launch arb_simulation simulation.launch.py "
                  "spawn:=odometry_test", file=sys.stderr)
            return 2
        if len(node.samples) < 50:
            print("too few samples to analyse", file=sys.stderr)
            return 1

        a = analyse(node.samples)
        plot(a, args.output)

        args.csv.parent.mkdir(parents=True, exist_ok=True)
        with open(args.csv, "w") as f:
            f.write("t,distance,odom_x,odom_y,odom_yaw,truth_x,truth_y,truth_yaw,"
                    "err_x,err_y,err_pos,err_yaw,sigma_pos,sigma_yaw\n")
            for i in range(len(a["t"])):
                f.write(
                    f"{a['t'][i]:.4f},{a['distance'][i]:.4f},"
                    f"{a['ox'][i]:.5f},{a['oy'][i]:.5f},{a['oth'][i]:.5f},"
                    f"{a['tx'][i]:.5f},{a['ty'][i]:.5f},{a['tth'][i]:.5f},"
                    f"{a['ex'][i]:.5f},{a['ey'][i]:.5f},{a['epos'][i]:.5f},"
                    f"{a['eth'][i]:.5f},{a['sigma_pos'][i]:.5f},"
                    f"{a['sigma_th'][i]:.5f}\n")

        d = a["distance"][-1]
        final = a["epos"][-1]
        print()
        print("=" * 66)
        print("PHASE 3 -- WHEEL ODOMETRY DRIFT")
        print("=" * 66)
        print(f"  distance travelled          {d:8.2f} m")
        print(f"  duration                    {a['t'][-1]:8.2f} s")
        print(f"  final position error        {final:8.3f} m"
              f"   ({100 * final / d:.2f} % of distance)")
        print(f"  max position error          {a['epos'].max():8.3f} m")
        print(f"  RMS position error          "
              f"{math.sqrt(float(np.mean(a['epos'] ** 2))):8.3f} m")
        print(f"  final heading error         "
              f"{math.degrees(a['eth'][-1]):8.2f} deg")
        print(f"  max |heading error|         "
              f"{math.degrees(np.abs(a['eth']).max()):8.2f} deg")
        print()
        print(f"  reported final 1-sigma pos  {a['sigma_pos'][-1]:8.3f} m")
        print(f"  reported final 1-sigma yaw  "
              f"{math.degrees(a['sigma_th'][-1]):8.2f} deg")
        # Loop closure: the route is a closed rectangle, so the robot ends
        # where it started. The distance between the odometry's first and last
        # pose is therefore pure accumulated error, and unlike the aligned
        # comparison above it needs no reference frame at all.
        closure = math.hypot(a["ox"][-1] - a["ox"][0], a["oy"][-1] - a["oy"][0])
        truth_closure = math.hypot(
            a["tx"][-1] - a["tx"][0], a["ty"][-1] - a["ty"][0])
        print(f"  odometry loop closure       {closure:8.3f} m"
              f"   (ground truth actually closed to {truth_closure:.3f} m)")
        print()

        inside = float(np.mean(a["epos"] <= 2 * a["sigma_pos"]))
        print(f"  samples within reported 2-sigma  {100 * inside:5.1f} %")
        if inside < 0.6:
            print("  -> the filter is OVERCONFIDENT: it claims less uncertainty "
                  "than it has")
        elif inside > 0.995:
            print("  -> the filter is CONSERVATIVE: it claims more uncertainty "
                  "than it needs")
        print()
        print(f"  wrote {args.output.relative_to(REPO_ROOT)}")
        print(f"  wrote {args.csv.relative_to(REPO_ROOT)}")
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
