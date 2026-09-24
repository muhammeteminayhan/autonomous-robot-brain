#!/usr/bin/env python3
"""Measure how well each controller follows a path, and whether the stack navigates.

This is the Phase 9 deliverable, and it answers two different questions with
two modes.

``--mode tracking`` (the controller benchmark)
    Publishes a KNOWN reference path straight onto ``/arb/global_path`` and
    records what the robot actually does. The global planner and the map are
    deliberately out of the loop: a controller measured behind a planner is
    measured together with the planner's replanning jitter, and the numbers
    stop being about the controller.

    The robot starts 0.6 m OFF that path, on purpose. That offset is a step
    input, and a step is the only input for which "settling time" and
    "overshoot" have agreed definitions. Reported:

    * cross-track error   -- RMSE, mean, max, and the same over the steady
                             portion only, because the deliberate initial
                             error would otherwise dominate every average
    * settling time       -- time from the start until |cross-track| enters
                             the settling band and STAYS there
    * overshoot           -- furthest excursion past the path on the far side,
                             as a percentage of the initial 0.6 m offset
    * control smoothness  -- RMS of dv/dt and domega/dt of the COMMAND. A
                             controller that tracks perfectly by chattering
                             the wheels is not a controller worth shipping,
                             and nothing in the error metrics would show it
    * heading error, speed, distance, time, and the closest the robot came to
      anything (from the LiDAR, so a collision cannot be scored as success)

``--mode goal`` (the end-to-end check)
    Waits for the map the stack has built, publishes one goal on ``/arb/goal``,
    and watches the whole chain -- EKF, scan matcher, mapping, A*, local
    planner -- drive the robot there. Reports arrival, time, path length versus
    straight-line distance, and minimum clearance.

Ground truth is read here and only here, to SCORE the run. It never reaches a
controller, an estimator or a planner; see docs/architecture.md section 4.

Usage::

    # terminal 1 -- simulator, sensors, localization
    ros2 launch arb_simulation simulation.launch.py headless:=true rviz:=false \\
        spawn:=control_test
    ros2 launch arb_sensors sensors.launch.py
    ros2 launch arb_localization odometry.launch.py initial_pose:=control_test

    # terminal 2 -- ONE controller, never both: they share /cmd_vel
    ros2 launch arb_control control.launch.py
    python3 scripts/evaluate_navigation.py --mode tracking --controller pid

    ros2 launch arb_planning planning.launch.py local_planner:=true
    python3 scripts/evaluate_navigation.py --mode tracking --controller dwa
"""

from __future__ import annotations

import argparse
import csv
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
from geometry_msgs.msg import (  # noqa: E402
    PoseStamped,
    PoseWithCovarianceStamped,
    Twist,
)
from nav_msgs.msg import Odometry, Path  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan  # noqa: E402
from std_msgs.msg import Float64MultiArray  # noqa: E402

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=200,
)
#: The path is latched: a controller that starts after the evaluator must still
#: receive it, and a Path published into the void is the most annoying possible
#: way for this script to appear to hang.
LATCHED_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

#: Spacing of the reference path's vertices. The controllers consume a polyline
#: and interpolate nothing, so a coarse path is a path with corners the robot
#: is asked to cut.
PATH_RESOLUTION_M = 0.05

#: |cross-track| must fall below this and stay there for the run to count as
#: settled. 0.05 m is a quarter of the robot's 0.20 m goal tolerance -- tight
#: enough to be a real claim, loose enough that sensor noise alone does not
#: prevent it being reached.
SETTLING_BAND_M = 0.05

#: Distance travelled after which the error is called "steady", and equally the
#: window in which the step response is measured.
#:
#: Two constraints fix it at 3.0 m, and both were found by measurement.
#:
#: WIDE ENOUGH for the slowest controller to settle in. At 1.5 m the DWA planner
#: had not yet reached the settling band -- it has no in-place turn and arcs onto
#: the path -- so its settling time came out as "never" while the PID follower's
#: came out as 2.0 s. A window one controller cannot reach is not a comparison.
#:
#: NARROW ENOUGH to end before the robot starts reacting to the first corner.
#: The first leg is 4.6 m, but DWA looks 1.0 m along the path and 2.0 s into the
#: future, so it begins turning for the corner from about 3.5 m. At a 4.0 m
#: window its error left the band again in the final sample -- corner
#: anticipation, correctly, being scored as a failure to settle.
TRANSIENT_DISTANCE_M = 3.0

#: Robot circumscribed radius and the moving obstacle's radius, for the
#: geometric collision test. Both come from configs -- robot.yaml's chassis is
#: 0.40 x 0.30 m so half the diagonal is 0.25 m, and the obstacle node's
#: `radius_m` defaults to 0.20 m.
ROBOT_RADIUS_M = 0.25
OBSTACLE_RADIUS_M = 0.20

#: Below this the run is a collision, not a lap. The robot's circumscribed
#: radius is 0.25 m and the LiDAR sits 0.14 m forward of base_link, so a
#: reading of 0.11 m in front means contact.
COLLISION_RANGE_M = 0.12

STYLE = {
    "pid": ("#1971c2", "-", "PID + pure pursuit"),
    "dwa": ("#e8590c", "-", "DWA local planner"),
}


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# ---------------------------------------------------------------- the course
def densify(corners, resolution: float = PATH_RESOLUTION_M):
    pts: list[tuple[float, float]] = [corners[0]]
    for (x0, y0), (x1, y1) in zip(corners, corners[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(round(seg / resolution)))
        for i in range(1, n + 1):
            f = i / n
            pts.append((x0 + f * (x1 - x0), y0 + f * (y1 - y0)))
    return pts


def benchmark_course(start_x: float, start_y: float,
                     width: float, height: float,
                     tail: float) -> list[tuple[float, float]]:
    """Three square corners, densified, ending somewhere the start is not.

    Four square corners is the point. A smooth course flatters a path follower:
    on a straight line its tracking error is whatever the odometry noise is,
    which says nothing about the controller. The error at a 90 degree corner is
    a property of the lookahead distance and the angular gains, which is what is
    being measured.

    The course is deliberately NOT CLOSED. A controller's goal is the last pose
    of its path, and on a closed loop that pose is also where the robot starts:
    it is inside the goal tolerance at t = 0, declares itself arrived, and the
    run measures nothing. Ending 0.6 m short of the first leg keeps the goal
    unambiguous while staying inside Room B.
    """
    corners = [
        (start_x, start_y),
        (start_x + width, start_y),
        (start_x + width, start_y + height),
        (start_x, start_y + height),
        (start_x, start_y + tail),
    ]
    return densify(corners)


def polyline_length(pts) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


def signed_cross_track(pts: np.ndarray, x: float,
                       y: float) -> tuple[float, float, float]:
    """Signed distance to the polyline, arc length, and the path's heading there.

    The heading is returned so the heading error can be measured the same way for
    every controller. Taking it from a controller's own diagnostics topic makes
    the metric depend on what that controller chooses to publish -- and the DWA
    planner publishes nothing of the kind, so half the comparison came out as
    "not a number".

    Signed, not absolute, because overshoot is defined by CHANGING SIDE. An
    absolute error curve that dips to zero and rises again looks identical
    whether the robot crossed the path or merely approached and retreated, and
    those are opposite verdicts on a controller.

    The sign is taken from the cross product of the segment direction with the
    vector to the robot: positive means the robot is to the LEFT of the path as
    the path is travelled.
    """
    a = pts[:-1]
    b = pts[1:]
    d = b - a
    seg_len2 = np.einsum("ij,ij->i", d, d)
    seg_len2 = np.where(seg_len2 > 0.0, seg_len2, 1.0)
    w = np.array([x, y]) - a
    t = np.clip(np.einsum("ij,ij->i", w, d) / seg_len2, 0.0, 1.0)
    proj = a + t[:, None] * d
    diff = np.array([x, y]) - proj
    dist = np.hypot(diff[:, 0], diff[:, 1])
    k = int(np.argmin(dist))
    cross = d[k, 0] * w[k, 1] - d[k, 1] * w[k, 0]
    sign = 1.0 if cross >= 0.0 else -1.0
    seg_starts = np.concatenate(([0.0], np.cumsum(np.hypot(d[:, 0], d[:, 1]))))
    arc = seg_starts[k] + t[k] * math.sqrt(seg_len2[k])
    tangent = math.atan2(d[k, 1], d[k, 0])
    return sign * float(dist[k]), float(arc), tangent


# ------------------------------------------------------------------ recorder
class NavigationEvaluator(Node):
    """Publishes the reference, records the response, drives nothing itself."""

    def __init__(self, path_topic: str) -> None:
        super().__init__("arb_navigation_evaluator")
        self.truth: Odometry | None = None
        self.odom: Odometry | None = None
        self.scan: LaserScan | None = None
        self.pose: PoseWithCovarianceStamped | None = None
        self.cmd: Twist | None = None
        self.cmd_time: float = 0.0
        self.diag: list[float] | None = None
        self.planned: Path | None = None
        self.obstacle: PoseStamped | None = None

        self.create_subscription(
            Odometry, "/arb/ground_truth/odom", self._on_truth, SENSOR_QOS)
        # /arb/odom is RAW WHEEL ODOMETRY in the odom frame, which starts at the
        # origin by construction. Comparing it to ground truth in the map frame
        # reports the distance between two different frames' origins, not an
        # estimation error. The EKF's map-frame estimate is on /arb/pose.
        self.create_subscription(Odometry, "/arb/odom", self._on_odom, SENSOR_QOS)
        self.create_subscription(
            PoseWithCovarianceStamped, "/arb/pose", self._on_pose, SENSOR_QOS)
        self.create_subscription(LaserScan, "/arb/scan", self._on_scan, SENSOR_QOS)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_subscription(
            Float64MultiArray, "/arb/control_diagnostics", self._on_diag, 10)
        self.create_subscription(
            Path, "/arb/global_path", self._on_planned, 10)
        # Instrumentation only, and only present when the moving-obstacle
        # experiment is running. Never read by anything in the control path.
        self.create_subscription(
            PoseStamped, "/arb/ground_truth/obstacle_pose",
            self._on_obstacle, 10)

        self.path_pub = self.create_publisher(Path, path_topic, LATCHED_QOS)
        self.goal_pub = self.create_publisher(PoseStamped, "/arb/goal", 10)
        self.stop_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.samples: list[dict] = []

    # --- callbacks ---
    def _on_truth(self, m): self.truth = m
    def _on_odom(self, m): self.odom = m
    def _on_pose(self, m): self.pose = m
    def _on_scan(self, m): self.scan = m
    def _on_planned(self, m): self.planned = m
    def _on_obstacle(self, m): self.obstacle = m
    def _on_diag(self, m): self.diag = list(m.data)

    def _on_cmd(self, m):
        self.cmd = m
        self.cmd_time = self._now()

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_for(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for(self, predicate, timeout: float, what: str) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        print(f"timed out after {timeout:.0f}s waiting for {what}", file=sys.stderr)
        return False

    def publish_path(self, pts, frame: str = "map") -> Path:
        msg = Path()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for i, (x, y) in enumerate(pts):
            nxt = pts[min(i + 1, len(pts) - 1)]
            yaw = math.atan2(nxt[1] - y, nxt[0] - x)
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = float(x)
            p.pose.position.y = float(y)
            p.pose.orientation.z = math.sin(yaw / 2.0)
            p.pose.orientation.w = math.cos(yaw / 2.0)
            msg.poses.append(p)
        self.path_pub.publish(msg)
        return msg

    def publish_goal(self, x: float, y: float, yaw: float = 0.0) -> None:
        g = PoseStamped()
        g.header.frame_id = "map"
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x = float(x)
        g.pose.position.y = float(y)
        g.pose.orientation.z = math.sin(yaw / 2.0)
        g.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pub.publish(g)

    def halt(self) -> None:
        """Stop the robot. The controller under test owns /cmd_vel, so this is
        only ever sent after the controller has been told the run is over."""
        for _ in range(10):
            self.stop_pub.publish(Twist())
            self.spin_for(0.05)

    def min_range(self) -> float:
        """Nearest MEASURABLE return, which is not the same as nearest thing.

        Readings at or below `range_min` are excluded because the sensor cannot
        measure there -- but excluding them silently means this number bottoms
        out at range_min and stops falling however close something gets. Three
        obstacle runs all reported 0.120 m to three decimals, which is the
        sensor's floor rather than a measurement, and `collision_free` returned
        True for every one of them. Use `blind_spot_hits()` alongside this, and
        for the obstacle runs use the geometric clearance from ground truth.
        """
        if self.scan is None:
            return float("inf")
        r = np.asarray(self.scan.ranges, dtype=float)
        good = r[np.isfinite(r) & (r > self.scan.range_min)]
        return float(good.min()) if good.size else float("inf")

    def blind_spot_hits(self) -> int:
        """Beams reporting at or below range_min: something is too close to measure."""
        if self.scan is None:
            return 0
        r = np.asarray(self.scan.ranges, dtype=float)
        return int(np.count_nonzero(np.isfinite(r) & (r <= self.scan.range_min)))

    def obstacle_clearance(self, robot_x: float, robot_y: float) -> float:
        """Surface-to-surface distance to the moving obstacle, from GROUND TRUTH.

        The only honest collision test available here. The LiDAR cannot see
        inside its own minimum range, which is exactly where a collision is, so
        a scan-based verdict is blind at the moment it matters most. The
        obstacle publishes its true pose as instrumentation precisely so this
        can be computed; nothing in the control path may read it.
        """
        if self.obstacle is None:
            return float("inf")
        centre = math.hypot(self.obstacle.pose.position.x - robot_x,
                            self.obstacle.pose.position.y - robot_y)
        return centre - (ROBOT_RADIUS_M + OBSTACLE_RADIUS_M)

    def ready(self) -> bool:
        return (self.truth is not None and self.odom is not None
                and self.pose is not None and self.scan is not None)

    def record(self, reference: np.ndarray, t0: float) -> dict:
        t = self.truth
        gx = t.pose.pose.position.x
        gy = t.pose.pose.position.y
        gyaw = yaw_of_quat(t.pose.pose.orientation)
        xt, arc, tangent = signed_cross_track(reference, gx, gy)
        cmd_age = self._now() - self.cmd_time if self.cmd is not None else float("inf")
        fresh = self.cmd is not None and cmd_age < 0.5
        s = {
            "t": self._now() - t0,
            "gx": gx, "gy": gy, "gyaw": gyaw,
            "ex": self.pose.pose.pose.position.x,
            "ey": self.pose.pose.pose.position.y,
            "eyaw": yaw_of_quat(self.pose.pose.pose.orientation),
            "cross_track": xt,
            "arc": arc,
            "v_true": t.twist.twist.linear.x,
            "w_true": t.twist.twist.angular.z,
            "v_cmd": self.cmd.linear.x if fresh else 0.0,
            "w_cmd": self.cmd.angular.z if fresh else 0.0,
            "min_range": self.min_range(),
            "blind_spot_hits": self.blind_spot_hits(),
            "obstacle_clearance": self.obstacle_clearance(gx, gy),
        }
        # The controller's own view of the error, when it publishes one. Its
        # disagreement with the ground-truth cross-track error is the
        # LOCALIZATION error leaking into the control loop -- a controller can
        # only be as accurate as its estimate of where it is.
        # The controller's own view of its cross-track error, when it publishes
        # one. Its disagreement with the ground-truth value is the LOCALIZATION
        # error leaking into the control loop.
        s["cross_track_believed"] = self.diag[0] if self.diag else float("nan")
        # Heading error against the PATH TANGENT, from ground truth, so it means
        # the same for a controller that publishes diagnostics and one that does
        # not.
        s["heading_error"] = wrap(gyaw - tangent)
        # How wrong the estimate the controller is steering on actually is. A
        # controller can only be as accurate as its idea of where it is, so this
        # is the floor under every tracking number below.
        s["localization_error"] = math.hypot(s["ex"] - gx, s["ey"] - gy)
        self.samples.append(s)
        return s


# ------------------------------------------------------------------- metrics
def controller_metrics(samples: list[dict], initial_offset: float,
                       course_length: float) -> dict:
    """Turn the recorded response into the four numbers Phase 9 asks for."""
    t = np.array([s["t"] for s in samples])
    xt = np.array([s["cross_track"] for s in samples])
    gx = np.array([s["gx"] for s in samples])
    gy = np.array([s["gy"] for s in samples])
    v_cmd = np.array([s["v_cmd"] for s in samples])
    w_cmd = np.array([s["w_cmd"] for s in samples])
    v_true = np.array([s["v_true"] for s in samples])
    heading = np.array([s["heading_error"] for s in samples])

    step = np.hypot(np.diff(gx), np.diff(gy))
    travelled = np.concatenate(([0.0], np.cumsum(step)))

    # --- settling time -------------------------------------------------------
    # Settling time is a property of the STEP RESPONSE, so it is measured only
    # over the transient -- the first TRANSIENT_DISTANCE_M of travel, while the
    # robot is removing the deliberate initial offset on a straight leg.
    #
    # Measured over the whole run instead it reports the last corner: a corner
    # legitimately throws the error outside the band, so "the last time it left
    # the band" lands at the final corner and the number is just the run length
    # minus a few seconds. Measured that way this controller "settled" at
    # 22.3 s of a 25.8 s run, which says nothing about anything.
    #
    # Within the window the definition is the strict one: the last time the
    # error left the band, not the first time it entered. A response that
    # touches the band, leaves it and comes back has not settled at the touch.
    transient = travelled <= TRANSIENT_DISTANCE_M
    settling = float("nan")
    if transient.any():
        xt_t = xt[transient]
        t_t = t[transient]
        inside = np.abs(xt_t) <= SETTLING_BAND_M
        if inside.all():
            settling = 0.0
        elif inside.any():
            outside_idx = np.flatnonzero(~inside)
            if outside_idx[-1] + 1 < inside.size:
                settling = float(t_t[outside_idx[-1] + 1] - t_t[0])

    # --- overshoot -----------------------------------------------------------
    # The robot starts on the negative side of the path (it spawns south of an
    # eastward path). Overshoot is the largest excursion to the POSITIVE side
    # during the transient, as a fraction of the step it was asked to remove.
    sign0 = -1.0 if xt[0] < 0.0 else 1.0
    opposite = -sign0 * xt[transient]
    overshoot_m = float(max(0.0, opposite.max())) if opposite.size else 0.0
    overshoot_pct = 100.0 * overshoot_m / abs(initial_offset) if initial_offset else 0.0

    # --- steady-state tracking ----------------------------------------------
    steady = travelled > TRANSIENT_DISTANCE_M
    xt_steady = xt[steady] if steady.any() else xt

    # --- smoothness ----------------------------------------------------------
    # RMS of the command's time derivative. Normalising by the limit makes the
    # two channels comparable: 1.0 would mean the command sweeps its entire
    # range every second, on average.
    dt = np.diff(t)
    dt = np.where(dt > 1e-6, dt, np.nan)
    dv = np.abs(np.diff(v_cmd)) / dt
    dw = np.abs(np.diff(w_cmd)) / dt
    jerk_v = float(np.sqrt(np.nanmean(dv ** 2))) if dv.size else 0.0
    jerk_w = float(np.sqrt(np.nanmean(dw ** 2))) if dw.size else 0.0

    # Reversals: how often the angular command changes sign. A weaving
    # controller and a smooth one can have the same RMS error, and this is what
    # separates them.
    sgn = np.sign(w_cmd[np.abs(w_cmd) > 0.05])
    reversals = int(np.count_nonzero(np.diff(sgn) != 0)) if sgn.size > 1 else 0

    heading_steady = heading[steady] if steady.any() else heading
    finite_heading = heading[np.isfinite(heading)]
    return {
        "samples": len(samples),
        "duration_s": float(t[-1] - t[0]),
        "distance_m": float(travelled[-1]),
        "course_fraction": float(travelled[-1] / course_length) if course_length else 0.0,
        "cross_track_rmse_m": float(np.sqrt(np.mean(xt ** 2))),
        "cross_track_mean_abs_m": float(np.mean(np.abs(xt))),
        "cross_track_max_m": float(np.max(np.abs(xt))),
        "cross_track_steady_rmse_m": float(np.sqrt(np.mean(xt_steady ** 2))),
        "cross_track_steady_max_m": float(np.max(np.abs(xt_steady))),
        "settling_time_s": settling,
        "settling_band_m": SETTLING_BAND_M,
        "settling_window_m": TRANSIENT_DISTANCE_M,
        "initial_offset_m": float(abs(initial_offset)),
        "overshoot_m": overshoot_m,
        "overshoot_pct": overshoot_pct,
        "heading_error_rmse_rad": (
            float(np.sqrt(np.mean(finite_heading ** 2))) if finite_heading.size
            else float("nan")),
        "heading_error_steady_rmse_rad": float(np.sqrt(np.mean(
            heading_steady[np.isfinite(heading_steady)] ** 2))),
        "heading_error_max_rad": float(np.max(np.abs(finite_heading))),
        "smoothness_dv_rms_m_s2": jerk_v,
        "smoothness_dw_rms_rad_s2": jerk_w,
        "angular_reversals": reversals,
        "localization_error_rmse_m": float(np.sqrt(np.mean(
            np.array([s["localization_error"] for s in samples]) ** 2))),
        "localization_error_max_m": float(np.max(
            [s["localization_error"] for s in samples])),
        "mean_speed_m_s": float(np.mean(np.abs(v_true))),
        "max_speed_m_s": float(np.max(np.abs(v_true))),
        "min_lidar_range_m": float(np.nanmin([s["min_range"] for s in samples])),
        "collision_free": bool(
            np.nanmin([s["min_range"] for s in samples]) > COLLISION_RANGE_M),
    }


# --------------------------------------------------------------------- plots
def draw_floor_plan(ax, world: dict) -> None:
    th = world["wall"]["thickness_m"]
    for w in world["walls"]:
        x1, y1, x2, y2 = w["segment"]
        ax.plot([x1, x2], [y1, y2], color="#495057", lw=th * 18, solid_capstyle="butt",
                zorder=1, alpha=0.55)
    for o in world["obstacles"]:
        x, y = o["pose"]
        if o["type"] == "cylinder":
            ax.add_patch(plt.Circle((x, y), o["radius"], color="#868e96",
                                    zorder=1, alpha=0.7))
        else:
            sx, sy = o["size"][0], o["size"][1]
            ax.add_patch(plt.Rectangle((x - sx / 2, y - sy / 2), sx, sy,
                                       angle=math.degrees(o.get("yaw", 0.0)),
                                       rotation_point="center",
                                       color="#868e96", zorder=1, alpha=0.7))


def plot_tracking(results: dict, reference: np.ndarray, world: dict,
                  out_dir: pathlib.Path) -> list[pathlib.Path]:
    written = []

    # --- 1. trajectories over the floor plan ---
    fig, ax = plt.subplots(figsize=(9, 7))
    draw_floor_plan(ax, world)
    ax.plot(reference[:, 0], reference[:, 1], color="#212529", ls="--", lw=1.6,
            label="reference path", zorder=3)
    for name, data in results.items():
        colour, ls, label = STYLE[name]
        s = data["samples"]
        ax.plot([p["gx"] for p in s], [p["gy"] for p in s], color=colour, ls=ls,
                lw=1.8, label=f"{label} (truth)", zorder=4)
    ax.set_xlim(-3.4, 3.4)
    ax.set_ylim(1.2, 7.4)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Phase 9 -- path following, Room B rectangle")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(alpha=0.25)
    p = out_dir / "trajectory.png"
    fig.tight_layout()
    fig.savefig(p, dpi=140)
    plt.close(fig)
    written.append(p)

    # --- 2. cross-track error against distance ---
    fig, ax = plt.subplots(figsize=(10, 4.2))
    for name, data in results.items():
        colour, ls, label = STYLE[name]
        s = data["samples"]
        gx = np.array([q["gx"] for q in s])
        gy = np.array([q["gy"] for q in s])
        d = np.concatenate(([0.0], np.cumsum(np.hypot(np.diff(gx), np.diff(gy)))))
        ax.plot(d, [q["cross_track"] for q in s], color=colour, lw=1.4, label=label)
    ax.axhspan(-SETTLING_BAND_M, SETTLING_BAND_M, color="#40c057", alpha=0.15,
               label=f"settling band +/-{SETTLING_BAND_M} m")
    ax.axvline(TRANSIENT_DISTANCE_M, color="#adb5bd", ls=":",
               label="end of transient")
    ax.axhline(0.0, color="#212529", lw=0.8)
    ax.set_xlabel("distance travelled [m]")
    ax.set_ylabel("signed cross-track error [m]")
    ax.set_title("Cross-track error (positive = left of the path)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.25)
    p = out_dir / "cross_track.png"
    fig.tight_layout()
    fig.savefig(p, dpi=140)
    plt.close(fig)
    written.append(p)

    # --- 3. commands over time ---
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    for name, data in results.items():
        colour, ls, label = STYLE[name]
        s = data["samples"]
        t = [q["t"] for q in s]
        axes[0].plot(t, [q["v_cmd"] for q in s], color=colour, lw=1.2,
                     label=f"{label} commanded")
        axes[0].plot(t, [q["v_true"] for q in s], color=colour, lw=0.8, alpha=0.4,
                     label=f"{label} achieved")
        axes[1].plot(t, [q["w_cmd"] for q in s], color=colour, lw=1.2, label=label)
    axes[0].set_ylabel("v [m/s]")
    axes[0].set_title("Commanded vs achieved velocity, and the steering command")
    axes[0].legend(fontsize=7, ncol=2)
    axes[0].grid(alpha=0.25)
    axes[1].set_ylabel("omega [rad/s]")
    axes[1].set_xlabel("time [s]")
    axes[1].axhline(0.0, color="#212529", lw=0.8)
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=0.25)
    p = out_dir / "commands.png"
    fig.tight_layout()
    fig.savefig(p, dpi=140)
    plt.close(fig)
    written.append(p)

    # --- 4. the step response, on its own axes ---
    fig, ax = plt.subplots(figsize=(9, 4.0))
    for name, data in results.items():
        colour, ls, label = STYLE[name]
        s = data["samples"]
        gx = np.array([q["gx"] for q in s])
        gy = np.array([q["gy"] for q in s])
        d = np.concatenate(([0.0], np.cumsum(np.hypot(np.diff(gx), np.diff(gy)))))
        keep = d <= 3.0
        sign0 = -1.0 if s[0]["cross_track"] < 0.0 else 1.0
        ax.plot(np.array([q["t"] for q in s])[keep],
                sign0 * np.array([q["cross_track"] for q in s])[keep],
                color=colour, lw=1.6, label=label)
        m = data["metrics"]
        if math.isfinite(m["settling_time_s"]):
            ax.axvline(m["settling_time_s"], color=colour, ls=":", lw=1.2)
    ax.axhspan(-SETTLING_BAND_M, SETTLING_BAND_M, color="#40c057", alpha=0.15)
    ax.axhline(0.0, color="#212529", lw=0.8)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("remaining offset [m]")
    ax.set_title("Step response: converging onto a path 0.6 m away "
                 "(dotted = settling time)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.25)
    p = out_dir / "step_response.png"
    fig.tight_layout()
    fig.savefig(p, dpi=140)
    plt.close(fig)
    written.append(p)
    return written


def plot_goal(samples: list[dict], metrics: dict, world: dict,
              out_dir: pathlib.Path, tag: str) -> list[pathlib.Path]:
    """What the whole stack did, on one page."""
    written = []
    gx = np.array([s["gx"] for s in samples])
    gy = np.array([s["gy"] for s in samples])
    ex = np.array([s["ex"] for s in samples])
    ey = np.array([s["ey"] for s in samples])
    t = np.array([s["t"] for s in samples])

    fig, ax = plt.subplots(figsize=(11, 8))
    draw_floor_plan(ax, world)
    ax.plot(gx, gy, color="#1971c2", lw=2.0, label="ground truth", zorder=4)
    ax.plot(ex, ey, color="#e8590c", lw=1.2, ls="--",
            label="EKF estimate (what the robot steered on)", zorder=5)
    sx, sy = metrics["start_xy"]
    tx, ty = metrics["goal_xy"]
    ax.plot(sx, sy, "o", color="#2f9e44", ms=11, label="start", zorder=6)
    ax.plot(tx, ty, "*", color="#c2255c", ms=20, label="goal", zorder=6)
    ax.set_xlim(world["bounds"]["x_min"] - 0.5, world["bounds"]["x_max"] + 0.5)
    ax.set_ylim(world["bounds"]["y_min"] - 0.5, world["bounds"]["y_max"] + 0.5)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(
        f"Phase 9 end-to-end: {metrics['goal']} in {metrics['time_s']:.1f} s, "
        f"{metrics['distance_m']:.1f} m driven, closest approach "
        f"{metrics['min_lidar_range_m']:.2f} m")
    ax.legend(loc="lower left", fontsize=8)
    ax.grid(alpha=0.2)
    path = out_dir / f"navigation_{tag}.png"
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    written.append(path)

    # Speed and clearance over the run: where it had to slow down, and why.
    fig, axes = plt.subplots(2, 1, figsize=(11, 5.5), sharex=True)
    axes[0].plot(t, [s["v_true"] for s in samples], color="#1971c2", lw=1.2,
                 label="achieved")
    axes[0].plot(t, [s["v_cmd"] for s in samples], color="#adb5bd", lw=1.0,
                 label="commanded")
    axes[0].set_ylabel("v [m/s]")
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.25)
    axes[0].set_title("Speed, and the clearance that constrained it")
    clear = np.array([s["min_range"] for s in samples])
    axes[1].plot(t, clear, color="#e8590c", lw=1.2)
    axes[1].axhline(COLLISION_RANGE_M, color="#c2255c", ls=":",
                    label=f"contact ({COLLISION_RANGE_M} m)")
    axes[1].set_ylabel("nearest LiDAR return [m]")
    axes[1].set_xlabel("time [s]")
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=0.25)
    path = out_dir / f"navigation_{tag}_profile.png"
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    written.append(path)
    return written


def write_csv(path: pathlib.Path, samples: list[dict]) -> None:
    keys = list(samples[0].keys())
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        for s in samples:
            w.writerow(s)


# ----------------------------------------------------------------------- run
def run_tracking(node: NavigationEvaluator, args, world: dict) -> dict:
    pts = benchmark_course(args.course_x, args.course_y, args.course_w,
                           args.course_h, args.course_tail)
    reference = np.asarray(pts)
    lap_length = polyline_length(pts)

    if not node.wait_for(node.ready, 40.0,
                         "ground truth, the EKF pose and a scan"):
        raise SystemExit(2)

    x0 = node.truth.pose.pose.position.x
    y0 = node.truth.pose.pose.position.y
    offset, _, _ = signed_cross_track(reference, x0, y0)
    print(f"start ({x0:+.3f}, {y0:+.3f}), {abs(offset):.3f} m off the reference; "
          f"lap {lap_length:.2f} m")

    node.publish_path(pts)
    # Republish a few times: a controller that subscribed a moment ago with a
    # volatile reader would otherwise miss it, and the run would record a robot
    # standing perfectly still with zero tracking error.
    for _ in range(3):
        node.spin_for(0.3)
        node.publish_path(pts)

    t0 = node._now()
    period = 1.0 / args.rate
    deadline = time.time() + args.duration
    last = 0.0
    stalled_since = None
    finished = False
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.01)
        now = node._now()
        if now - last < period:
            continue
        last = now
        s = node.record(reference, t0)
        if s["min_range"] < COLLISION_RANGE_M:
            print(f"  COLLISION at t={s['t']:.1f}s, min range {s['min_range']:.3f} m",
                  file=sys.stderr)
            break
        travelled = 0.0
        if len(node.samples) > 1:
            gx = np.array([q["gx"] for q in node.samples])
            gy = np.array([q["gy"] for q in node.samples])
            travelled = float(np.sum(np.hypot(np.diff(gx), np.diff(gy))))
        # The course is finished when the robot reaches its LAST POSE, judged on
        # ground truth. Judging it on the controller's own "at_goal" flag would
        # score the controller against its own opinion of where it is.
        end = pts[-1]
        if math.hypot(s["gx"] - end[0], s["gy"] - end[1]) < args.goal_tolerance:
            print(f"  course complete at t={s['t']:.1f}s after {travelled:.2f} m")
            finished = True
            break
        if abs(s["v_true"]) < 0.02 and abs(s["w_true"]) < 0.05:
            stalled_since = stalled_since or s["t"]
            if s["t"] - stalled_since > args.stall_timeout and travelled > 0.5:
                print(f"  stalled for {args.stall_timeout:.0f}s at t={s['t']:.1f}s")
                break
        else:
            stalled_since = None

    node.halt()
    metrics = controller_metrics(node.samples, offset, lap_length)
    metrics["completed_course"] = finished
    metrics["course"] = {
        "start": [args.course_x, args.course_y],
        "width_m": args.course_w, "height_m": args.course_h,
        "tail_m": args.course_tail, "length_m": lap_length,
        "corners": 3,
    }
    return {"samples": list(node.samples), "metrics": metrics, "reference": pts}


def run_goal(node: NavigationEvaluator, args, world: dict) -> dict:
    goal_name = args.goal
    poses = world["poses"]
    if goal_name not in poses:
        raise SystemExit(f"goal '{goal_name}' not in world.yaml; have {sorted(poses)}")
    goal = poses[goal_name]

    if not node.wait_for(node.ready, 40.0,
                         "ground truth, the EKF pose and a scan"):
        raise SystemExit(2)
    x0 = node.truth.pose.pose.position.x
    y0 = node.truth.pose.pose.position.y
    straight = math.hypot(goal["x"] - x0, goal["y"] - y0)
    print(f"goal '{goal_name}' at ({goal['x']:+.2f}, {goal['y']:+.2f}); "
          f"straight-line {straight:.2f} m")

    # Published repeatedly, not once. A publisher created moments before it is
    # used has not necessarily finished DDS discovery, and the first message
    # goes nowhere: measured, the planner logged no goal at all and this script
    # timed out waiting for a plan that was never asked for.
    def goal_sent() -> bool:
        node.publish_goal(goal["x"], goal["y"], goal.get("yaw", 0.0))
        return node.planned is not None and len(node.planned.poses) > 1

    if not node.wait_for(goal_sent, 30.0, "a global plan"):
        raise SystemExit(3)
    print(f"  plan received: {len(node.planned.poses)} poses, "
          f"{polyline_length([(p.pose.position.x, p.pose.position.y) for p in node.planned.poses]):.2f} m")

    t0 = node._now()
    period = 1.0 / args.rate
    deadline = time.time() + args.duration
    last = 0.0
    arrived = False
    gave_up = False
    stopped_since = None
    reference = np.asarray(
        [(p.pose.position.x, p.pose.position.y) for p in node.planned.poses])
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.01)
        now = node._now()
        if now - last < period:
            continue
        last = now
        # The plan is replanned as the map grows, so the reference the robot is
        # being scored against is refreshed too.
        if node.planned is not None and len(node.planned.poses) > 1:
            reference = np.asarray(
                [(p.pose.position.x, p.pose.position.y) for p in node.planned.poses])
        s = node.record(reference, t0)
        # Arrival is judged on GROUND TRUTH, not on the robot's own estimate. A
        # robot that believes it arrived and is two metres away has not arrived,
        # and scoring it on its own estimate would hide exactly that failure.
        true_error = math.hypot(s["gx"] - goal["x"], s["gy"] - goal["y"])
        if true_error < args.goal_tolerance:
            arrived = True
            print(f"  ARRIVED at t={s['t']:.1f}s, {true_error:.3f} m from the goal")
            break
        if s["min_range"] < COLLISION_RANGE_M:
            print(f"  COLLISION at t={s['t']:.1f}s, min range {s['min_range']:.3f} m",
                  file=sys.stderr)
            break
        # The stack stops itself when IT believes it has arrived, and its belief
        # is only as good as the EKF. Waiting out the full duration after that
        # adds two minutes of a parked robot to every average -- one run came out
        # at a mean speed of 0.085 m/s having actually driven at 0.46 m/s.
        if abs(s["v_true"]) < 0.02 and abs(s["w_true"]) < 0.05:
            stopped_since = stopped_since if stopped_since is not None else s["t"]
            if s["t"] - stopped_since > args.stall_timeout:
                print(f"  stack stopped itself at t={stopped_since:.1f}s, "
                      f"{true_error:.3f} m from the goal "
                      f"(it believed {math.hypot(s['ex'] - goal['x'], s['ey'] - goal['y']):.3f} m)")
                gave_up = True
                break
        else:
            stopped_since = None

    node.halt()
    gx = np.array([q["gx"] for q in node.samples])
    gy = np.array([q["gy"] for q in node.samples])
    travelled = float(np.sum(np.hypot(np.diff(gx), np.diff(gy))))
    final = node.samples[-1]
    metrics = {
        "goal": goal_name,
        "goal_xy": [goal["x"], goal["y"]],
        "start_xy": [x0, y0],
        "arrived": arrived,
        "goal_tolerance_m": args.goal_tolerance,
        # The stack halted on its own, believing it had arrived. Whether it had
        # is `final_error_m`; the gap between the two is `final_estimate_error_m`
        # and it is the localization error, not a control error.
        "stopped_itself_short": gave_up,
        "time_s": float(final["t"]),
        "distance_m": travelled,
        "straight_line_m": straight,
        "path_efficiency": straight / travelled if travelled > 0 else 0.0,
        "final_error_m": math.hypot(final["gx"] - goal["x"], final["gy"] - goal["y"]),
        "final_estimate_error_m": math.hypot(final["ex"] - final["gx"],
                                             final["ey"] - final["gy"]),
        "min_lidar_range_m": float(np.nanmin([q["min_range"] for q in node.samples])),
        # Beams reporting inside range_min: something was closer than the
        # sensor can measure. The LiDAR-only verdict is blind there, which is
        # exactly where a collision is.
        "blind_spot_samples": int(sum(
            1 for q in node.samples if q.get("blind_spot_hits", 0) > 0)),
        "min_obstacle_clearance_m": float(np.nanmin(
            [q.get("obstacle_clearance", float("inf")) for q in node.samples])),
        # Geometric, from ground truth, and therefore the only verdict that
        # holds when the obstacle is inside the LiDAR's minimum range.
        "touched_obstacle": bool(np.nanmin(
            [q.get("obstacle_clearance", float("inf")) for q in node.samples]) <= 0.0),
        "collision_free": bool(
            np.nanmin([q["min_range"] for q in node.samples]) > COLLISION_RANGE_M),
        "mean_speed_m_s": float(np.mean(np.abs([q["v_true"] for q in node.samples]))),
    }
    return {"samples": list(node.samples), "metrics": metrics}


def load_samples(path: pathlib.Path) -> list[dict]:
    with open(path) as fh:
        rows = []
        for row in csv.DictReader(fh):
            out = {}
            for k, v in row.items():
                if v in ("True", "False"):
                    out[k] = v == "True"
                else:
                    try:
                        out[k] = float(v)
                    except ValueError:
                        out[k] = v
            rows.append(out)
    return rows


def recompute(out_dir: pathlib.Path, args, world: dict) -> int:
    """Re-derive every metric and plot from the saved samples.

    The CSVs are the record of what the robot did; the metrics are a reading of
    them. Changing how a metric is defined -- and the settling window changed
    three times before it meant anything -- should not require driving the robot
    again, both because a rerun is a different run and because a metric that can
    only be checked by re-running cannot be checked at all.
    """
    pts = benchmark_course(args.course_x, args.course_y, args.course_w,
                           args.course_h, args.course_tail)
    reference = np.asarray(pts)
    course_length = polyline_length(pts)
    runs: dict[str, dict] = {}
    for name in ("pid", "dwa"):
        csv_path = out_dir / f"samples_tracking_{name}.csv"
        if not csv_path.exists():
            continue
        samples = load_samples(csv_path)
        offset = samples[0]["cross_track"]
        metrics = controller_metrics(samples, offset, course_length)
        old_path = out_dir / f"metrics_tracking_{name}.json"
        if old_path.exists():
            previous = json.load(open(old_path))
            for key in ("completed_course", "course"):
                if key in previous:
                    metrics[key] = previous[key]
        with open(old_path, "w") as fh:
            json.dump(metrics, fh, indent=2)
        runs[name] = {"samples": samples, "metrics": metrics}
        print(f"recomputed {name} from {len(samples)} saved samples")
    if not runs:
        print("no saved samples to recompute", file=sys.stderr)
        return 1
    for p in plot_tracking(runs, reference, world, out_dir):
        print(f"  wrote {p}")
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", choices=["tracking", "goal", "recompute"],
                    default="tracking",
                    help="recompute re-derives metrics and plots from the saved "
                         "samples without touching the simulator.")
    ap.add_argument("--controller", choices=["pid", "dwa"], default="pid",
                    help="Which controller is running. This script does not "
                         "start it; the label decides where results are filed.")
    ap.add_argument("--tag", default=None, help="Result file suffix (default: controller)")
    ap.add_argument("--duration", type=float, default=180.0)
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--stall-timeout", type=float, default=8.0)
    ap.add_argument("--goal", default="goal_room_c")
    ap.add_argument("--goal-tolerance", type=float, default=0.3)
    ap.add_argument("--course-x", type=float, default=-2.3)
    ap.add_argument("--course-y", type=float, default=2.8)
    ap.add_argument("--course-w", type=float, default=4.6)
    ap.add_argument("--course-h", type=float, default=3.5)
    ap.add_argument("--course-tail", type=float, default=0.6,
                    help="How far down the last leg the course ends, so the "
                         "final pose is not also the first.")
    ap.add_argument("--out", default=None)
    args = ap.parse_args(argv)
    tag = args.tag or args.controller

    world = yaml.safe_load(open(REPO_ROOT / "configs" / "world.yaml"))["world"]
    out_dir = pathlib.Path(args.out) if args.out else REPO_ROOT / "results" / "phase9"
    out_dir.mkdir(parents=True, exist_ok=True)

    if args.mode == "recompute":
        return recompute(out_dir, args, world)

    rclpy.init()
    node = NavigationEvaluator("/arb/global_path")
    try:
        if args.mode == "tracking":
            result = run_tracking(node, args, world)
        else:
            result = run_goal(node, args, world)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    write_csv(out_dir / f"samples_{args.mode}_{tag}.csv", result["samples"])
    with open(out_dir / f"metrics_{args.mode}_{tag}.json", "w") as fh:
        json.dump(result["metrics"], fh, indent=2)

    m = result["metrics"]
    print()
    print(f"==================== {args.mode} / {tag} ====================")
    for k, v in m.items():
        if isinstance(v, float):
            print(f"  {k:32s} {v:12.4f}")
        elif isinstance(v, (int, bool, str)):
            print(f"  {k:32s} {str(v):>12s}")
    print(f"  wrote {out_dir}/metrics_{args.mode}_{tag}.json")

    if args.mode == "goal":
        for p in plot_goal(result["samples"], m, world, out_dir, tag):
            print(f"  wrote {p}")

    if args.mode == "tracking":
        # Plot whatever runs exist on disk, so pid and dwa end up on the same
        # axes without needing to be run in the same process.
        runs = {}
        for name in ("pid", "dwa"):
            csv_path = out_dir / f"samples_tracking_{name}.csv"
            json_path = out_dir / f"metrics_tracking_{name}.json"
            if not (csv_path.exists() and json_path.exists()):
                continue
            with open(csv_path) as fh:
                rows = [{k: float(v) if v not in ("", "True", "False") else v
                         for k, v in row.items()} for row in csv.DictReader(fh)]
            runs[name] = {"samples": rows, "metrics": json.load(open(json_path))}
        if runs:
            written = plot_tracking(runs, np.asarray(result["reference"]), world, out_dir)
            for p in written:
                print(f"  wrote {p}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
