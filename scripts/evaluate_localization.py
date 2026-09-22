#!/usr/bin/env python3
"""Compare every pose estimator against ground truth, with metrics and plots.

This is the Phase 5 deliverable. It drives a repeatable route, records each
estimator alongside ground truth, and reports:

* position RMSE, mean, median and maximum error
* heading RMSE and maximum error
* error as a fraction of distance travelled (the honest normalisation for dead
  reckoning: error grows with distance, not with elapsed time)
* estimation latency, measured as the age of the data each published estimate
  is based on
* covariance consistency: how much of the time the actual error falls inside
  the uncertainty the estimator claims

and writes five plots:

    1. ground truth vs estimated trajectory
    2. x error over time
    3. y error over time
    4. heading error over time
    5. covariance evolution against the errors it is supposed to bound

Ground truth is used here, and only here: this is an evaluation tool under
``scripts/``, never part of the estimation pipeline. See docs/architecture.md
section 4.

Usage::

    ros2 launch arb_simulation simulation.launch.py headless:=true \\
        rviz:=false spawn:=odometry_test
    ros2 launch arb_sensors sensors.launch.py
    ros2 launch arb_localization odometry.launch.py
    python3 scripts/evaluate_localization.py --tag nominal
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
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist  # noqa: E402
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
sys.path.insert(0, str(REPO_ROOT / "scripts"))

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

LINEAR_SPEED = 0.25
ANGULAR_SPEED = 0.60
LONG_LEG_M = 1.5
SHORT_LEG_M = 1.0
LAPS = 2
MIN_CLEARANCE_M = 0.45
FRONT_OVERHANG_M = 0.06

#: Colour per estimator, kept consistent across every plot and every run.
STYLE = {
    "odometry": ("#e03131", "--"),
    "ekf": ("#1971c2", "-"),
}


def build_route(laps: int = LAPS):
    """Two laps of a rectangle. Turns are what make dead reckoning fail, so an
    evaluation that only drives straight flatters every estimator."""
    turn = (math.pi / 2.0) / ANGULAR_SPEED
    long_leg = LONG_LEG_M / LINEAR_SPEED
    short_leg = SHORT_LEG_M / LINEAR_SPEED
    route = [(0.0, 0.0, 2.0)]
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
    route.append((0.0, 0.0, 2.0))
    return route


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LocalizationEvaluator(Node):
    """Drives the route and records every estimator plus ground truth."""

    def __init__(self) -> None:
        super().__init__("arb_localization_evaluator")
        self.truth: Odometry | None = None
        self.odom: Odometry | None = None
        self.ekf: PoseWithCovarianceStamped | None = None
        self.scan: LaserScan | None = None
        self.samples: list[dict] = []
        self.latencies: list[float] = []
        self.aborted_reason: str | None = None

        self.create_subscription(
            Odometry, "/arb/ground_truth/odom", self._on_truth, SENSOR_QOS)
        self.create_subscription(Odometry, "/arb/odom", self._on_odom, SENSOR_QOS)
        self.create_subscription(
            PoseWithCovarianceStamped, "/arb/pose", self._on_ekf, SENSOR_QOS)
        self.create_subscription(LaserScan, "/arb/scan", self._on_scan, SENSOR_QOS)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", COMMAND_QOS)

    def _on_truth(self, m): self.truth = m
    def _on_odom(self, m): self.odom = m
    def _on_scan(self, m): self.scan = m

    def _on_ekf(self, m: PoseWithCovarianceStamped) -> None:
        self.ekf = m
        # Estimation latency: how old the data behind this estimate is by the
        # time it is published. The EKF deliberately buffers measurements so
        # that a 200 Hz and a 50 Hz stream can be fused in timestamp order, and
        # that buffer is a real, quantifiable lag -- not a defect, but a cost
        # that has to be reported rather than hidden.
        if self.truth is not None:
            stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            latest = (self.truth.header.stamp.sec +
                      self.truth.header.stamp.nanosec * 1e-9)
            self.latencies.append(latest - stamp)

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

    def publish(self, v: float, w: float) -> None:
        m = Twist()
        m.linear.x = v
        m.angular.z = w
        self.cmd.publish(m)

    def forward_clearance(self) -> float:
        s = self.scan
        if s is None or not s.ranges:
            return float("inf")
        idx = int(round(-s.angle_min / s.angle_increment))
        n = len(s.ranges)
        window = [s.ranges[(idx + k) % n] for k in range(-4, 5)]
        finite = [r for r in window if math.isfinite(r) and r >= s.range_min]
        return (min(finite) if finite else float(s.range_max)) - FRONT_OVERHANG_M

    def ready(self) -> bool:
        return self.truth is not None and self.odom is not None

    def record(self) -> None:
        if not self.ready():
            return
        t = self.truth
        stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
        if self.samples and stamp <= self.samples[-1]["t"]:
            return
        row = {
            "t": stamp,
            "truth": (t.pose.pose.position.x, t.pose.pose.position.y,
                      yaw_of_quat(t.pose.pose.orientation)),
        }
        o = self.odom
        row["odometry"] = (o.pose.pose.position.x, o.pose.pose.position.y,
                           yaw_of_quat(o.pose.pose.orientation))
        row["odometry_cov"] = (o.pose.covariance[0], o.pose.covariance[7],
                               o.pose.covariance[35])
        if self.ekf is not None:
            e = self.ekf
            row["ekf"] = (e.pose.pose.position.x, e.pose.pose.position.y,
                          yaw_of_quat(e.pose.pose.orientation))
            row["ekf_cov"] = (e.pose.covariance[0], e.pose.covariance[7],
                              e.pose.covariance[35])
        self.samples.append(row)

    def drive_route(self, route, hz: float = 50.0) -> bool:
        for v, w, duration in route:
            end = time.monotonic() + duration
            while time.monotonic() < end and rclpy.ok():
                if v > 0.0:
                    clearance = self.forward_clearance()
                    if clearance < MIN_CLEARANCE_M:
                        self.publish(0.0, 0.0)
                        self.spin_for(1.0)
                        self.aborted_reason = (
                            f"forward clearance fell to {clearance:.2f} m")
                        return False
                self.publish(v, w)
                self.spin_for(1.0 / hz)
                self.record()
        self.publish(0.0, 0.0)
        self.spin_for(1.5)
        self.record()
        return True


def align_and_diff(samples: list[dict], key: str):
    """Errors in the estimator's own starting frame.

    Each estimator starts at its own origin; ground truth starts wherever the
    robot was spawned. Only the FIRST sample is aligned, with a rigid
    transform and no fitting. Fitting the whole trajectory (Umeyama) would
    absorb part of the drift into the fit and flatter the estimator.
    """
    rows = [s for s in samples if key in s]
    if len(rows) < 10:
        return None

    t = np.array([s["t"] for s in rows])
    t -= t[0]
    ex_ = np.array([s[key][0] for s in rows])
    ey_ = np.array([s[key][1] for s in rows])
    eth = np.array([s[key][2] for s in rows])
    tx = np.array([s["truth"][0] for s in rows])
    ty = np.array([s["truth"][1] for s in rows])
    tth = np.array([s["truth"][2] for s in rows])

    dth = wrap(eth[0] - tth[0])
    c, s = math.cos(dth), math.sin(dth)
    tx_a = c * (tx - tx[0]) - s * (ty - ty[0]) + ex_[0]
    ty_a = s * (tx - tx[0]) + c * (ty - ty[0]) + ey_[0]
    tth_a = np.array([wrap(a + dth) for a in tth])

    cov = np.array([s.get(f"{key}_cov", (0.0, 0.0, 0.0)) for s in rows])
    distance = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(tx), np.diff(ty)))])

    return {
        "t": t, "x": ex_, "y": ey_, "th": eth,
        "tx": tx_a, "ty": ty_a, "tth": tth_a,
        "err_x": ex_ - tx_a,
        "err_y": ey_ - ty_a,
        "err_th": np.array([wrap(a - b) for a, b in zip(eth, tth_a)]),
        "err_pos": np.hypot(ex_ - tx_a, ey_ - ty_a),
        "sigma_pos": np.sqrt(np.maximum(0.0, cov[:, 0] + cov[:, 1])),
        "sigma_th": np.sqrt(np.maximum(0.0, cov[:, 2])),
        "distance": distance,
    }


def metrics(a: dict) -> dict:
    ep, eth = a["err_pos"], a["err_th"]
    d = float(a["distance"][-1])
    return {
        "distance_m": d,
        "duration_s": float(a["t"][-1]),
        "position_rmse_m": float(np.sqrt(np.mean(ep ** 2))),
        "position_mean_m": float(np.mean(ep)),
        "position_median_m": float(np.median(ep)),
        "position_max_m": float(np.max(ep)),
        "position_final_m": float(ep[-1]),
        "position_final_pct_of_distance": float(100.0 * ep[-1] / d) if d > 0 else 0.0,
        "heading_rmse_deg": float(np.degrees(np.sqrt(np.mean(eth ** 2)))),
        "heading_max_deg": float(np.degrees(np.max(np.abs(eth)))),
        "heading_final_deg": float(np.degrees(eth[-1])),
        "reported_final_sigma_pos_m": float(a["sigma_pos"][-1]),
        "reported_final_sigma_heading_deg": float(np.degrees(a["sigma_th"][-1])),
        "fraction_inside_2sigma_pos": float(np.mean(ep <= 2 * a["sigma_pos"]))
        if np.any(a["sigma_pos"] > 0) else float("nan"),
        "fraction_inside_2sigma_heading": float(
            np.mean(np.abs(eth) <= 2 * a["sigma_th"]))
        if np.any(a["sigma_th"] > 0) else float("nan"),
    }


def plot_all(results: dict, latency_ms, out_dir: pathlib.Path, tag: str) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)

    # --- 1. trajectory ---
    fig, ax = plt.subplots(figsize=(9, 8))
    first = next(iter(results.values()))
    ax.plot(first["tx"], first["ty"], color="#2f9e44", lw=2.4, label="ground truth")
    for name, a in results.items():
        colour, ls = STYLE.get(name, ("#666", "-"))
        ax.plot(a["x"], a["y"], color=colour, ls=ls, lw=1.7, label=name)
        ax.plot(a["x"][-1], a["y"][-1], "s", color=colour, ms=8)
    ax.plot(first["tx"][0], first["ty"][0], "o", color="#1971c2", ms=10, label="start")
    ax.plot(first["tx"][-1], first["ty"][-1], "s", color="#2f9e44", ms=9)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3, ls=":")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"Trajectory vs ground truth  ({tag})")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / f"1_trajectory_{tag}.png", dpi=130)
    plt.close(fig)

    # --- 2 & 3. per-axis error ---
    for idx, (axis, label) in enumerate((("err_x", "x"), ("err_y", "y")), start=2):
        fig, ax = plt.subplots(figsize=(11, 4.5))
        for name, a in results.items():
            colour, ls = STYLE.get(name, ("#666", "-"))
            ax.plot(a["t"], a[axis], color=colour, ls=ls, lw=1.6, label=name)
        ax.axhline(0.0, color="#888", lw=0.8)
        ax.grid(alpha=0.3, ls=":")
        ax.set_xlabel("time [s]")
        ax.set_ylabel(f"{label} error [m]")
        ax.set_title(f"{label} error over time  ({tag})")
        ax.legend()
        fig.tight_layout()
        fig.savefig(out_dir / f"{idx}_{label}_error_{tag}.png", dpi=130)
        plt.close(fig)

    # --- 4. heading error ---
    fig, ax = plt.subplots(figsize=(11, 4.5))
    for name, a in results.items():
        colour, ls = STYLE.get(name, ("#666", "-"))
        ax.plot(a["t"], np.degrees(a["err_th"]), color=colour, ls=ls, lw=1.6,
                label=name)
        if np.any(a["sigma_th"] > 0):
            ax.fill_between(a["t"], -np.degrees(a["sigma_th"]),
                            np.degrees(a["sigma_th"]), color=colour, alpha=0.10)
    ax.axhline(0.0, color="#888", lw=0.8)
    ax.grid(alpha=0.3, ls=":")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("heading error [deg]")
    ax.set_title(f"Heading error, shaded bands are the reported 1-sigma  ({tag})")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / f"4_heading_error_{tag}.png", dpi=130)
    plt.close(fig)

    # --- 5. covariance evolution ---
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    for name, a in results.items():
        colour, ls = STYLE.get(name, ("#666", "-"))
        axes[0].plot(a["distance"], a["err_pos"], color=colour, ls=ls, lw=1.7,
                     label=f"{name}: error")
        axes[0].plot(a["distance"], a["sigma_pos"], color=colour, ls=":", lw=1.3,
                     label=f"{name}: reported 1-sigma")
        axes[1].plot(a["t"], np.degrees(a["sigma_th"]), color=colour, ls=ls, lw=1.7,
                     label=f"{name}: reported 1-sigma")
        axes[1].plot(a["t"], np.degrees(np.abs(a["err_th"])), color=colour, ls=":",
                     lw=1.3, label=f"{name}: |error|")
    axes[0].set_xlabel("distance travelled [m]")
    axes[0].set_ylabel("position [m]")
    axes[0].set_title("Position error vs the uncertainty claimed")
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("heading [deg]")
    axes[1].set_title("Heading error vs the uncertainty claimed")
    for ax in axes:
        ax.grid(alpha=0.3, ls=":")
        ax.legend(fontsize=8)
    fig.suptitle(
        "Covariance evolution: an estimator that claims less uncertainty than "
        "it has is overconfident and will diverge", fontsize=11)
    fig.tight_layout()
    fig.savefig(out_dir / f"5_covariance_{tag}.png", dpi=130)
    plt.close(fig)

    # --- latency ---
    if len(latency_ms) > 10:
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.hist(latency_ms, bins=40, color="#1971c2", alpha=0.85)
        ax.axvline(float(np.median(latency_ms)), color="#e03131", lw=1.6,
                   label=f"median {np.median(latency_ms):.1f} ms")
        ax.set_xlabel("estimation latency [ms]")
        ax.set_ylabel("count")
        ax.set_title("Age of the data behind each published estimate")
        ax.legend()
        ax.grid(alpha=0.3, ls=":")
        fig.tight_layout()
        fig.savefig(out_dir / f"6_latency_{tag}.png", dpi=130)
        plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tag", default="nominal",
                    help="label for this run; becomes part of every filename")
    ap.add_argument("--out", type=pathlib.Path,
                    default=REPO_ROOT / "results" / "phase5")
    args = ap.parse_args(argv)

    rclpy.init()
    node = LocalizationEvaluator()
    try:
        print("waiting for ground truth and at least one estimator ...")
        deadline = time.monotonic() + 25.0
        while not node.ready() and time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.ready():
            print("timed out; is the stack running?", file=sys.stderr)
            return 1
        if node.ekf is None:
            print("NOTE: /arb/pose not seen; evaluating odometry only")

        route = build_route()
        total = sum(d for _, _, d in route)
        print(f"driving {LAPS} laps of a {LONG_LEG_M} x {SHORT_LEG_M} m rectangle "
              f"({total:.0f} s) ...")
        completed = node.drive_route(route)
        print(f"collected {len(node.samples)} samples")
        if not completed:
            print(f"\nABORTED: {node.aborted_reason}", file=sys.stderr)
            return 2

        results = {}
        for key in ("odometry", "ekf"):
            a = align_and_diff(node.samples, key)
            if a is not None:
                results[key] = a
        if not results:
            print("no estimator produced enough samples", file=sys.stderr)
            return 1

        latency_ms = np.array(node.latencies) * 1e3 if node.latencies else np.array([])
        plot_all(results, latency_ms, args.out, args.tag)

        report = {"tag": args.tag, "estimators": {}}
        for name, a in results.items():
            report["estimators"][name] = metrics(a)
        if latency_ms.size:
            report["latency_ms"] = {
                "median": float(np.median(latency_ms)),
                "mean": float(np.mean(latency_ms)),
                "p95": float(np.percentile(latency_ms, 95)),
                "max": float(np.max(latency_ms)),
            }

        args.out.mkdir(parents=True, exist_ok=True)
        (args.out / f"metrics_{args.tag}.json").write_text(
            json.dumps(report, indent=2))

        # --- console table ---
        names = list(results)
        w = 26
        print()
        print("=" * (w + 18 * len(names)))
        print(f"PHASE 5 -- LOCALIZATION EVALUATION  [{args.tag}]")
        print("=" * (w + 18 * len(names)))
        print(f"{'metric'.ljust(w)}" + "".join(n.rjust(18) for n in names))
        print("-" * (w + 18 * len(names)))
        rows = [
            ("distance travelled [m]", "distance_m", "{:.2f}"),
            ("position RMSE [m]", "position_rmse_m", "{:.4f}"),
            ("position mean [m]", "position_mean_m", "{:.4f}"),
            ("position max [m]", "position_max_m", "{:.4f}"),
            ("position final [m]", "position_final_m", "{:.4f}"),
            ("  as % of distance", "position_final_pct_of_distance", "{:.2f} %"),
            ("heading RMSE [deg]", "heading_rmse_deg", "{:.3f}"),
            ("heading max [deg]", "heading_max_deg", "{:.3f}"),
            ("heading final [deg]", "heading_final_deg", "{:+.3f}"),
            ("reported 1-sigma pos [m]", "reported_final_sigma_pos_m", "{:.4f}"),
            ("reported 1-sigma yaw [deg]", "reported_final_sigma_heading_deg",
             "{:.2f}"),
            ("inside 2-sigma (pos)", "fraction_inside_2sigma_pos", "{:.1%}"),
            ("inside 2-sigma (yaw)", "fraction_inside_2sigma_heading", "{:.1%}"),
        ]
        for label, key, fmt in rows:
            line = label.ljust(w)
            for n in names:
                line += fmt.format(report["estimators"][n][key]).rjust(18)
            print(line)

        if len(names) == 2 and "odometry" in report["estimators"] and \
                "ekf" in report["estimators"]:
            o = report["estimators"]["odometry"]
            e = report["estimators"]["ekf"]
            print("-" * (w + 18 * len(names)))
            for label, key in (("position RMSE", "position_rmse_m"),
                               ("heading RMSE", "heading_rmse_deg")):
                if o[key] > 0:
                    change = 100.0 * (e[key] / o[key] - 1.0)
                    verdict = "better" if change < 0 else "worse"
                    print(f"  EKF {label}: {change:+.1f} % ({verdict} than odometry)")

        if latency_ms.size:
            print()
            print(f"  estimation latency: median {np.median(latency_ms):.1f} ms, "
                  f"p95 {np.percentile(latency_ms, 95):.1f} ms, "
                  f"max {np.max(latency_ms):.1f} ms")

        print()
        print(f"  wrote {args.out.relative_to(REPO_ROOT)}/*_{args.tag}.png")
        print(f"  wrote {args.out.relative_to(REPO_ROOT)}/metrics_{args.tag}.json")
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
