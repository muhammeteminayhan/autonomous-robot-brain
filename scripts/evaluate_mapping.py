#!/usr/bin/env python3
"""Explore the world, build a map, and score it against the known floor plan.

Phase 6's deliverable. The robot wanders reactively using only the LiDAR --
no ground truth, no prior map -- while `arb_mapping` integrates each scan at
the EKF's estimated pose. Afterwards the resulting occupancy grid is compared
cell by cell against the floor plan the world was generated from.

The comparison is the point. A map that looks plausible is easy; a map that
agrees with the world to within a cell is the thing worth claiming. Reported:

* precision  -- of the cells the map calls occupied, how many really are
* recall     -- of the surfaces the robot could see, how many the map found
* free-space accuracy -- of the cells the map calls free, how many really are
* explored fraction -- how much of the reachable world stopped being unknown

Ground truth is used to SCORE the map, never to build it. The robot's own
estimate places every scan, so an error in the estimate smears the map and the
score falls -- which is exactly what should happen.

Usage::

    ros2 launch arb_simulation simulation.launch.py headless:=true rviz:=false
    ros2 launch arb_sensors sensors.launch.py
    ros2 launch arb_localization odometry.launch.py
    ros2 launch arb_mapping mapping.launch.py
    python3 scripts/evaluate_mapping.py --duration 180
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
import yaml  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry  # noqa: E402
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
    history=QoSHistoryPolicy.KEEP_LAST, depth=10)
MAP_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)
COMMAND_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=10)

CRUISE_SPEED = 0.35
TURN_SPEED = 0.60
#: Clearance below which the explorer stops going forward and turns.
STOP_DISTANCE = 0.70
#: Clearance above which it is happy to cruise again.
GO_DISTANCE = 1.00
FRONT_SECTOR_DEG = 22.0
SIDE_SECTOR_DEG = 55.0

#: Which wall to hug. Wall following is what turns a random wanderer into
#: something that actually covers a floor plan: keeping one wall at a fixed
#: distance traces the boundary of every connected region and, at a doorway,
#: follows the wall through it. A purely reactive "turn away from obstacles"
#: explorer instead settles into a loop in whichever room it started -- measured
#: on the first attempt, 54 m of driving confined to one room and 29.5 % of
#: cells ever observed.
WALL_SIDE = 1.0               # +1 hug the left wall, -1 the right
WALL_TARGET_M = 0.9
WALL_GAIN = 1.6
#: Past this the wall is considered lost and the robot turns to find it again,
#: which is how it gets around outside corners and through doorways.
WALL_LOST_M = 1.8


class Explorer(Node):
    """A reactive wanderer that uses only the LiDAR.

    Deliberately not a planner. Phase 6 needs the robot to cover ground so the
    map fills in; it does not need the coverage to be optimal, and using a
    planner here would make the mapping result depend on Phase 7 code that does
    not exist yet.
    """

    def __init__(self) -> None:
        super().__init__("arb_mapping_evaluator")
        self.scan: LaserScan | None = None
        self.map: OccupancyGrid | None = None
        self.truth: Odometry | None = None
        self.track: list[tuple[float, float]] = []
        self.turn_sign = 1.0

        self.create_subscription(LaserScan, "/arb/scan", self._on_scan, SENSOR_QOS)
        self.create_subscription(OccupancyGrid, "/arb/map", self._on_map, MAP_QOS)
        self.create_subscription(
            Odometry, "/arb/ground_truth/odom", self._on_truth, SENSOR_QOS)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", COMMAND_QOS)

    def _on_scan(self, m): self.scan = m
    def _on_map(self, m): self.map = m

    def _on_truth(self, m):
        self.truth = m
        p = m.pose.pose.position
        if not self.track or math.hypot(p.x - self.track[-1][0],
                                        p.y - self.track[-1][1]) > 0.05:
            self.track.append((p.x, p.y))

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

    def publish(self, v: float, w: float) -> None:
        t = Twist()
        t.linear.x = v
        t.angular.z = w
        self.cmd.publish(t)

    def sector_min(self, centre_deg: float, half_width_deg: float) -> float:
        """Smallest range in a sector, measured in the SENSOR frame.

        The forward beam is at index -angle_min / angle_increment. Index 0
        points backwards for a scan spanning -pi..+pi.
        """
        s = self.scan
        if s is None or not s.ranges:
            return float("inf")
        n = len(s.ranges)
        centre = int(round((math.radians(centre_deg) - s.angle_min) /
                           s.angle_increment))
        half = max(1, int(round(math.radians(half_width_deg) / s.angle_increment)))
        vals = [s.ranges[(centre + k) % n] for k in range(-half, half + 1)]
        finite = [r for r in vals if math.isfinite(r) and r >= s.range_min]
        return min(finite) if finite else float(s.range_max)

    def step(self) -> None:
        """One control cycle of a wall-following explorer.

        Three regimes, in priority order:

        1. Something close ahead -> turn away from the wall being hugged. This
           is what gets the robot around inside corners.
        2. The wall has been lost -> turn towards it. This is what gets the
           robot around outside corners and, crucially, through doorways: the
           wall "disappears" at a door and following it turns the robot in.
        3. Otherwise -> cruise, steering proportionally to hold the wall at a
           fixed distance.
        """
        front = self.sector_min(0.0, FRONT_SECTOR_DEG)
        side = self.sector_min(WALL_SIDE * 75.0, SIDE_SECTOR_DEG)
        diag = self.sector_min(WALL_SIDE * 40.0, 30.0)

        if front < STOP_DISTANCE or diag < STOP_DISTANCE * 0.8:
            # Inside corner: turn AWAY from the followed wall.
            self.publish(0.0, -WALL_SIDE * TURN_SPEED)
            return

        if side > WALL_LOST_M:
            # Outside corner or a doorway: turn TOWARDS the wall to find it
            # again, while still creeping forward so the robot does not pivot
            # on the spot forever in an open room.
            self.publish(0.45 * CRUISE_SPEED, WALL_SIDE * 0.55 * TURN_SPEED)
            return

        # Proportional wall following. The error is signed so that being too
        # far from the wall steers towards it and too close steers away.
        error = side - WALL_TARGET_M
        turn = WALL_SIDE * WALL_GAIN * math.tanh(error)
        speed = CRUISE_SPEED * (0.5 if front < GO_DISTANCE else 1.0)
        self.publish(speed, max(-TURN_SPEED, min(TURN_SPEED, turn)))


def build_truth_grid(world: dict, info) -> np.ndarray:
    """Rasterise the known floor plan onto the published map's exact grid."""
    res = info.resolution
    w, h = info.width, info.height
    ox, oy = info.origin.position.x, info.origin.position.y

    xs = ox + (np.arange(w) + 0.5) * res
    ys = oy + (np.arange(h) + 0.5) * res
    gx, gy = np.meshgrid(xs, ys)
    occupied = np.zeros_like(gx, dtype=bool)

    th = world["wall"]["thickness_m"] / 2.0
    for wall in world["walls"]:
        x1, y1, x2, y2 = wall["segment"]
        cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        length = math.hypot(x2 - x1, y2 - y1) + world["wall"]["thickness_m"]
        yaw = math.atan2(y2 - y1, x2 - x1)
        c, s = math.cos(-yaw), math.sin(-yaw)
        lx = c * (gx - cx) - s * (gy - cy)
        ly = s * (gx - cx) + c * (gy - cy)
        occupied |= (np.abs(lx) <= length / 2.0) & (np.abs(ly) <= th)

    for obs in world["obstacles"]:
        ox_, oy_ = obs["pose"]
        if obs["type"] == "cylinder":
            occupied |= (gx - ox_) ** 2 + (gy - oy_) ** 2 <= obs["radius"] ** 2
        else:
            sx, sy, _ = obs["size"]
            yaw = obs.get("yaw", 0.0)
            c, s = math.cos(-yaw), math.sin(-yaw)
            lx = c * (gx - ox_) - s * (gy - oy_)
            ly = s * (gx - ox_) + c * (gy - oy_)
            occupied |= (np.abs(lx) <= sx / 2.0) & (np.abs(ly) <= sy / 2.0)
    return occupied


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--duration", type=float, default=180.0)
    ap.add_argument("--tag", default="nominal")
    ap.add_argument("--out", type=pathlib.Path,
                    default=REPO_ROOT / "results" / "phase6")
    #: How far a map cell may be from a true surface and still count as correct.
    #: One cell of tolerance, because a wall boundary genuinely falls somewhere
    #: inside a cell and both roundings are defensible.
    ap.add_argument("--tolerance-cells", type=int, default=1)
    args = ap.parse_args(argv)

    world = yaml.safe_load(
        (REPO_ROOT / "configs" / "world.yaml").read_text())["world"]

    rclpy.init()
    node = Explorer()
    try:
        print("waiting for /arb/scan and /arb/map ...")
        deadline = time.monotonic() + 40.0
        while (node.scan is None or node.map is None) and \
                time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.scan is None:
            print("no scan; is the stack running?", file=sys.stderr)
            return 1
        if node.map is None:
            print("no map yet; is arb_mapping running?", file=sys.stderr)
            return 1

        print(f"exploring for {args.duration:.0f} s ...")
        end = time.monotonic() + args.duration
        last_report = time.monotonic()
        while time.monotonic() < end and rclpy.ok():
            node.step()
            node.spin_for(0.05)
            if time.monotonic() - last_report > 30.0:
                last_report = time.monotonic()
                if node.map is not None:
                    d = np.array(node.map.data)
                    known = int(np.sum(d >= 0))
                    print(f"  {end - time.monotonic():5.0f} s left, "
                          f"{100.0 * known / d.size:5.1f} % of cells known")
        node.publish(0.0, 0.0)
        node.spin_for(2.0)

        grid = node.map
        info = grid.info
        data = np.array(grid.data, dtype=np.int16).reshape(info.height, info.width)
        truth = build_truth_grid(world, info)

        occupied = data >= 65
        free = (data >= 0) & (data <= 35)
        known = data >= 0

        # A cell counts as correctly occupied if a true surface lies within the
        # tolerance window. A wall boundary genuinely falls somewhere inside a
        # cell, so demanding an exact match would penalise a correct map for a
        # rounding that could have gone either way.
        from scipy.ndimage import binary_dilation
        k = args.tolerance_cells
        struct = np.ones((2 * k + 1, 2 * k + 1), dtype=bool)
        truth_near = binary_dilation(truth, structure=struct)
        map_near = binary_dilation(occupied, structure=struct)

        tp = int(np.sum(occupied & truth_near))
        fp = int(np.sum(occupied & ~truth_near))
        # Only surfaces the robot could actually SEE are recallable. A wall in
        # a room it never entered is not a mapping failure.
        seen_truth = truth & known
        fn = int(np.sum(seen_truth & ~map_near))

        precision = tp / (tp + fp) if (tp + fp) else float("nan")
        recall = tp / (tp + fn) if (tp + fn) else float("nan")
        f1 = (2 * precision * recall / (precision + recall)
              if precision + recall > 0 else float("nan"))
        free_correct = int(np.sum(free & ~truth_near))
        free_wrong = int(np.sum(free & truth))
        free_accuracy = free_correct / max(1, free_correct + free_wrong)
        explored = float(np.mean(known))

        report = {
            "tag": args.tag,
            "duration_s": args.duration,
            "cells": int(data.size),
            "resolution_m": float(info.resolution),
            "tolerance_cells": args.tolerance_cells,
            "occupied_cells": int(np.sum(occupied)),
            "free_cells": int(np.sum(free)),
            "unknown_cells": int(np.sum(~known)),
            "explored_fraction": explored,
            "occupied_precision": precision,
            "occupied_recall": recall,
            "occupied_f1": f1,
            "free_accuracy": free_accuracy,
            "false_positive_cells": fp,
            "false_negative_cells": fn,
            "path_length_m": float(sum(
                math.hypot(b[0] - a[0], b[1] - a[1])
                for a, b in zip(node.track, node.track[1:]))),
        }

        args.out.mkdir(parents=True, exist_ok=True)
        (args.out / f"metrics_{args.tag}.json").write_text(json.dumps(report, indent=2))

        # --- figure ---
        extent = [info.origin.position.x,
                  info.origin.position.x + info.width * info.resolution,
                  info.origin.position.y,
                  info.origin.position.y + info.height * info.resolution]

        fig, axes = plt.subplots(1, 3, figsize=(20, 7))

        shown = np.full(data.shape, 0.5)
        shown[free] = 1.0
        shown[occupied] = 0.0
        axes[0].imshow(shown, origin="lower", extent=extent, cmap="gray",
                       vmin=0, vmax=1, interpolation="nearest")
        if node.track:
            tx = [p[0] for p in node.track]
            ty = [p[1] for p in node.track]
            axes[0].plot(tx, ty, color="#e03131", lw=1.2, label="path (ground truth)")
            axes[0].legend(loc="upper right", fontsize=8)
        axes[0].set_title("Built map: black occupied, white free, grey unknown")

        axes[1].imshow(truth, origin="lower", extent=extent, cmap="gray_r",
                       interpolation="nearest")
        axes[1].set_title("Known floor plan (ground truth)")

        overlay = np.zeros(data.shape + (3,))
        overlay[..., :] = 1.0
        overlay[~known] = 0.85
        overlay[truth] = [0.35, 0.35, 0.35]              # true surface, grey
        overlay[occupied & truth_near] = [0.0, 0.55, 0.2]  # correct, green
        overlay[occupied & ~truth_near] = [0.9, 0.1, 0.1]  # false positive, red
        overlay[seen_truth & ~map_near] = [0.1, 0.3, 0.95]  # missed, blue
        axes[2].imshow(overlay, origin="lower", extent=extent,
                       interpolation="nearest")
        axes[2].set_title(
            f"green correct / red false / blue missed\n"
            f"precision {precision:.3f}  recall {recall:.3f}  F1 {f1:.3f}")

        for ax in axes:
            ax.set_xlabel("x [m]")
            ax.set_ylabel("y [m]")
        fig.suptitle(
            f"Phase 6: occupancy grid built from the ESTIMATED pose  ({args.tag})",
            fontsize=13)
        fig.tight_layout()
        fig.savefig(args.out / f"map_{args.tag}.png", dpi=120)

        print()
        print("=" * 60)
        print(f"PHASE 6 -- OCCUPANCY GRID  [{args.tag}]")
        print("=" * 60)
        print(f"  grid                 {info.width} x {info.height} cells "
              f"at {info.resolution:.3f} m")
        print(f"  path length          {report['path_length_m']:8.2f} m")
        print(f"  explored             {100 * explored:8.1f} % of cells")
        print(f"  occupied cells       {report['occupied_cells']:8d}")
        print(f"  free cells           {report['free_cells']:8d}")
        print(f"  unknown cells        {report['unknown_cells']:8d}")
        print()
        print(f"  occupied precision   {precision:8.3f}"
              f"   ({fp} false positives)")
        print(f"  occupied recall      {recall:8.3f}"
              f"   ({fn} visible surfaces missed)")
        print(f"  occupied F1          {f1:8.3f}")
        print(f"  free-space accuracy  {free_accuracy:8.3f}"
              f"   ({free_wrong} cells called free that are wall)")
        print()
        print(f"  wrote {(args.out / f'map_{args.tag}.png').relative_to(REPO_ROOT)}")
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
