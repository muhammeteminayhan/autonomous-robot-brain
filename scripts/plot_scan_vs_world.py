#!/usr/bin/env python3
"""Overlay a live LiDAR scan on the known floor plan.

This is the strongest end-to-end check available in Phase 1. It exercises the
whole chain at once:

    world SDF -> physics -> GPU LiDAR -> gz-transport -> ros_gz_bridge
              -> sensor extrinsics (base_link -> lidar_link) -> world frame

If the plotted scan points land on the walls of the floor plan, then the world
was generated correctly, the sensor is mounted where the config says it is, the
beam angles are indexed the right way round, and the bridge is not mangling the
message. If any one of those is wrong the scan is visibly rotated, offset or
mirrored, and the error is obvious at a glance rather than buried in numbers.

Ground truth is used to place the scan in the world. That is legitimate here:
this is an offline visualisation tool under scripts/, not part of the estimation
pipeline (see docs/architecture.md section 4).

Usage::

    ros2 launch arb_simulation simulation.launch.py headless:=true rviz:=false
    python3 scripts/plot_scan_vs_world.py -o results/phase1/scan_vs_world.png
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys

import matplotlib
matplotlib.use("Agg")  # headless: no display needed
import matplotlib.pyplot as plt  # noqa: E402
import rclpy  # noqa: E402
import yaml  # noqa: E402
from matplotlib.patches import Circle, Rectangle  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan  # noqa: E402

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)


class ScanCollector(Node):
    """Grabs one scan and the pose it was taken from."""

    def __init__(self) -> None:
        super().__init__("arb_scan_plotter")
        self.scan: LaserScan | None = None
        self.odom: Odometry | None = None
        self.create_subscription(LaserScan, "/scan_raw", self._on_scan, SENSOR_QOS)
        self.create_subscription(
            Odometry, "/arb/ground_truth/odom", self._on_odom, SENSOR_QOS
        )

    def _on_scan(self, msg: LaserScan) -> None:
        self.scan = msg

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def ready(self) -> bool:
        return self.scan is not None and self.odom is not None


def _yaw_of(odom: Odometry) -> float:
    q = odom.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def draw_floor_plan(ax, world: dict) -> None:
    """Draw walls and obstacles exactly as generate_world.py laid them out."""
    thickness = world["wall"]["thickness_m"]
    for wall in world["walls"]:
        x1, y1, x2, y2 = wall["segment"]
        length = math.hypot(x2 - x1, y2 - y1) + thickness
        yaw = math.degrees(math.atan2(y2 - y1, x2 - x1))
        rect = Rectangle(
            (-length / 2.0, -thickness / 2.0), length, thickness,
            facecolor="#3a3a3a", edgecolor="none", zorder=1,
        )
        trans = (
            matplotlib.transforms.Affine2D()
            .rotate_deg(yaw)
            .translate((x1 + x2) / 2.0, (y1 + y2) / 2.0)
            + ax.transData
        )
        rect.set_transform(trans)
        ax.add_patch(rect)

    for obs in world["obstacles"]:
        ox, oy = obs["pose"]
        if obs["type"] == "cylinder":
            ax.add_patch(Circle((ox, oy), obs["radius"],
                                facecolor="#4a6fa5", edgecolor="none", zorder=2))
        else:
            sx, sy, _ = obs["size"]
            rect = Rectangle((-sx / 2, -sy / 2), sx, sy,
                             facecolor="#a5703f", edgecolor="none", zorder=2)
            trans = (
                matplotlib.transforms.Affine2D()
                .rotate(obs.get("yaw", 0.0))
                .translate(ox, oy)
                + ax.transData
            )
            rect.set_transform(trans)
            ax.add_patch(rect)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--output", type=pathlib.Path,
                    default=REPO_ROOT / "results" / "phase1" / "scan_vs_world.png")
    ap.add_argument("--timeout", type=float, default=20.0)
    args = ap.parse_args(argv)

    world = yaml.safe_load((REPO_ROOT / "configs" / "world.yaml").read_text())["world"]
    robot = yaml.safe_load((REPO_ROOT / "configs" / "robot.yaml").read_text())["robot"]
    lidar_x, lidar_y, _ = robot["sensor_mounts"]["lidar"]["xyz_m"]

    rclpy.init()
    node = ScanCollector()
    try:
        import time

        deadline = time.monotonic() + args.timeout
        while not node.ready() and time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.ready():
            print("timed out waiting for /scan_raw and /arb/ground_truth/odom",
                  file=sys.stderr)
            return 1

        scan, odom = node.scan, node.odom
        rx = odom.pose.pose.position.x
        ry = odom.pose.pose.position.y
        ryaw = _yaw_of(odom)

        # LiDAR origin in the world frame: base_link pose composed with the
        # sensor's mount offset. This is exactly Pose2D::compose from arb_core,
        # written out by hand for a script with no ROS message dependency.
        c, s = math.cos(ryaw), math.sin(ryaw)
        lx = rx + c * lidar_x - s * lidar_y
        ly = ry + s * lidar_x + c * lidar_y

        xs, ys = [], []
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
                continue
            # Beam i points at angle_min + i * angle_increment IN THE SENSOR
            # FRAME; add the robot yaw to get the world bearing.
            a = scan.angle_min + i * scan.angle_increment + ryaw
            xs.append(lx + r * math.cos(a))
            ys.append(ly + r * math.sin(a))

        fig, ax = plt.subplots(figsize=(13, 9))
        draw_floor_plan(ax, world)
        ax.scatter(xs, ys, s=7, c="#e03131", zorder=4,
                   label=f"LiDAR returns (n={len(xs)})")
        ax.plot([lx], [ly], marker="o", ms=9, color="#2f9e44", zorder=5,
                label="LiDAR origin (from ground truth)")
        ax.arrow(lx, ly, 0.7 * math.cos(ryaw), 0.7 * math.sin(ryaw),
                 head_width=0.18, color="#2f9e44", zorder=5, length_includes_head=True)

        b = world["bounds"]
        ax.set_xlim(b["x_min"] - 0.6, b["x_max"] + 0.6)
        ax.set_ylim(b["y_min"] - 0.6, b["y_max"] + 0.6)
        ax.set_aspect("equal")
        ax.grid(alpha=0.25, linestyle=":")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_title(
            "Phase 1 verification: LiDAR scan overlaid on the generated floor plan\n"
            f"robot at ({rx:.2f}, {ry:.2f}) heading {math.degrees(ryaw):.1f} deg",
            fontsize=11,
        )
        ax.legend(loc="upper right", framealpha=0.9)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        fig.tight_layout()
        fig.savefig(args.output, dpi=130)
        print(f"wrote {args.output.relative_to(REPO_ROOT)}  ({len(xs)} points)")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
