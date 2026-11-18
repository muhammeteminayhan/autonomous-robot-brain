#!/usr/bin/env python3
# Copyright (c) 2026 Emin Ayhan. MIT License.
"""Record the running stack as an animated GIF: the map as the robot builds it.

Run it against a live stack, in a second terminal::

    ros2 launch arb_bringup bringup.launch.py headless:=true rviz:=false
    python3 scripts/make_demo_gif.py --duration 70 --out docs/images/demo.gif

It renders from the TOPICS, not from a window, so it needs no GUI, no screen
recorder and no ffmpeg -- matplotlib draws each frame and Pillow writes the
GIF. That also means the result is reproducible on a headless machine and in
CI, which a screen capture would not be.

What it draws is what the robot believes: the occupancy grid it has built so
far, the live LiDAR return, the A* plan through the grid, and the EKF pose.
Ground truth is not subscribed to -- the same rule the rest of the stack
follows, and here it also keeps the picture honest, because a demo drawn at
the true pose would hide exactly the drift the project is about.
"""
from __future__ import annotations

import argparse
import io
import pathlib
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt          # noqa: E402
import numpy as np                       # noqa: E402
from PIL import Image                    # noqa: E402

import rclpy                             # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped   # noqa: E402
from nav_msgs.msg import OccupancyGrid, Path              # noqa: E402
from rclpy.node import Node                               # noqa: E402
from rclpy.qos import (DurabilityPolicy, QoSProfile,      # noqa: E402
                       ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan                     # noqa: E402

#: Occupancy grid shades. Deliberately the same reading as the RViz screenshot
#: in the README: pale free, dark occupied, olive never observed.
FREE, OCCUPIED, UNKNOWN = "#f2f0e9", "#2b2b2b", "#8a8457"


def yaw_of(q) -> float:
    return np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Recorder(Node):
    def __init__(self, every_s: float, width_px: int):
        super().__init__("demo_gif_recorder")
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.Parameter.Type.BOOL, True)])

        # The map is latched, so a late subscriber still gets the current one.
        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        # The estimator and the LiDAR publish with SensorDataQoS, which is
        # BEST_EFFORT. A RELIABLE subscriber is INCOMPATIBLE with that and
        # receives nothing at all -- rclpy says so once, in a warning that
        # scrolls past, and then the recorder simply draws no frames. Matching
        # it here is not a detail; it is the difference between a GIF and an
        # empty one.
        self.create_subscription(OccupancyGrid, "/arb/map", self._map, latched)
        self.create_subscription(PoseWithCovarianceStamped, "/arb/pose",
                                 self._pose, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/arb/scan", self._scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Path, "/arb/global_path", self._path, 10)

        self.map_: OccupancyGrid | None = None
        self.pose: tuple[float, float, float] | None = None
        self.path: list[tuple[float, float]] = []
        self.scan: LaserScan | None = None
        self.frames: list[Image.Image] = []
        self.width_px = width_px
        # Recording stops when the ROBOT stops, not when the clock runs out.
        # The run reached its goal at 36.8 s of an 85 s recording, and the
        # rest was a photograph of a stationary robot, which a viewer reads
        # as a hang rather than as an arrival. Pixel differences cannot see
        # this -- the LiDAR keeps jittering and the map keeps settling long
        # after the wheels stop -- but the pose says it exactly.
        self.still_since: float | None = None
        self.previous: tuple[float, float] | None = None
        self.still_s = 2.0
        self.arrived = False
        self.create_timer(every_s, self._snap)

    def _map(self, m): self.map_ = m
    def _scan(self, s): self.scan = s

    def _pose(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _path(self, m):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in m.poses]

    # ----------------------------------------------------------------- draw
    def _snap(self) -> None:
        if self.map_ is None or self.pose is None:
            return
        m = self.map_
        grid = np.asarray(m.data, dtype=np.int8).reshape(
            m.info.height, m.info.width)
        ox, oy, res = (m.info.origin.position.x, m.info.origin.position.y,
                       m.info.resolution)
        extent = (ox, ox + m.info.width * res, oy, oy + m.info.height * res)

        # The grid carries a PROBABILITY in 0..100, with -1 for unknown --
        # not the three-valued {free, occupied, unknown} it looks like. A free
        # cell is a low probability, rarely exactly zero, so testing `== 0`
        # for free and `> 0` for occupied paints every swept cell black and
        # produces a photographic negative of the map. It did, the first time.
        rgb = np.empty(grid.shape + (3,), dtype=np.uint8)
        for value, colour in ((grid < 0, UNKNOWN),
                              ((grid >= 0) & (grid < 50), FREE),
                              (grid >= 50, OCCUPIED)):
            rgb[value] = [int(colour[i:i + 2], 16) for i in (1, 3, 5)]

        fig, ax = plt.subplots(figsize=(self.width_px / 100, 0.66 * self.width_px / 100),
                               dpi=100)
        ax.imshow(rgb, origin="lower", extent=extent, interpolation="nearest")

        x, y, th = self.pose
        if self.scan is not None:
            a = (self.scan.angle_min
                 + np.arange(len(self.scan.ranges)) * self.scan.angle_increment)
            r = np.asarray(self.scan.ranges)
            good = np.isfinite(r) & (r > self.scan.range_min) & (r < self.scan.range_max)
            ax.scatter(x + r[good] * np.cos(th + a[good]),
                       y + r[good] * np.sin(th + a[good]),
                       s=1.2, c="#d93025", linewidths=0, zorder=3)
        if len(self.path) > 1:
            px, py = zip(*self.path)
            ax.plot(px, py, "-", c="#00b8d4", lw=1.8, zorder=4)
        ax.plot([x, x + 0.6 * np.cos(th)], [y, y + 0.6 * np.sin(th)],
                "-", c="#1a73e8", lw=2.2, zorder=5)
        ax.scatter([x], [y], s=46, c="#1a73e8", zorder=6, edgecolors="white",
                   linewidths=1.0)

        ax.set_xlim(extent[0], extent[1])
        ax.set_ylim(extent[2], extent[3])
        ax.set_xticks([]); ax.set_yticks([])
        for s in ax.spines.values():
            s.set_visible(False)
        ax.set_title(
            f"t = {self.get_clock().now().nanoseconds * 1e-9:5.1f} s   "
            f"map {int((grid >= 0).sum() * res * res):d} m²   "
            f"pose ({x:+.1f}, {y:+.1f})",
            fontsize=8, family="monospace", pad=6)
        fig.tight_layout(pad=0.4)

        buf = io.BytesIO()
        fig.savefig(buf, format="png")
        plt.close(fig)
        buf.seek(0)
        self.frames.append(Image.open(buf).convert("RGB"))

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.previous is None or np.hypot(x - self.previous[0],
                                             y - self.previous[1]) > 0.05:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = now
        elif now - self.still_since > self.still_s:
            self.arrived = True
        self.previous = (x, y)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--duration", type=float, default=70.0,
                    help="seconds of simulated time to record")
    ap.add_argument("--every", type=float, default=0.7,
                    help="seconds between frames")
    ap.add_argument("--fps", type=float, default=12.0)
    ap.add_argument("--width", type=int, default=760)
    ap.add_argument("--out", default="docs/images/demo.gif")
    args = ap.parse_args(argv)

    rclpy.init()
    node = Recorder(args.every, args.width)
    start = node.get_clock().now()
    # A wall-clock deadline as well as the simulated one. `use_sim_time` means
    # this node's clock is frozen whenever the simulator is paused or wedged,
    # and a frozen clock makes the loop below immortal: it waits for a time
    # that will never arrive, printing nothing. Measured the hard way against
    # a stack whose /clock had stopped. A recorder that hangs in silence is
    # worse than one that fails, so it gets a deadline it cannot argue with
    # and a message that names which of the two happened.
    wall_start = time.monotonic()
    wall_budget = max(120.0, args.duration * 8.0)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed = (node.get_clock().now() - start).nanoseconds * 1e-9
            if elapsed >= args.duration or node.arrived:
                break
            if time.monotonic() - wall_start > wall_budget:
                print(f"gave up after {wall_budget:.0f} s of real time having "
                      f"recorded only {elapsed:.1f} s of simulated time. The "
                      f"simulator is paused or not stepping -- check that "
                      f"/clock is publishing.", file=sys.stderr)
                break
    except KeyboardInterrupt:
        pass

    frames = node.frames
    node.destroy_node()
    rclpy.shutdown()

    if not frames:
        print("no frames: the stack published no map or no pose", file=sys.stderr)
        return 1

    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    # Palettised, because a GIF is 256 colours anyway and letting Pillow pick
    # the palette once keeps the file a third of the size.
    frames = [f.convert("P", palette=Image.ADAPTIVE, colors=128) for f in frames]
    frames[0].save(out, save_all=True, append_images=frames[1:],
                   duration=int(1000 / args.fps), loop=0, optimize=True)
    print(f"wrote {out} -- {len(frames)} frames, "
          f"{out.stat().st_size / 1e6:.2f} MB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
