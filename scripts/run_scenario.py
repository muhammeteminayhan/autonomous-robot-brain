#!/usr/bin/env python3
"""Play one fault scenario against the estimator and score what it did.

Phase 10's deliverable. The fault scheduler (``arb_evaluation.fault_scheduler``)
applies the scenario's knob timeline; this script drives a fixed route,
records ground truth alongside every estimate, and reports:

* **position RMSE** and **heading RMSE**, split into the healthy phase and the
  degraded phase, because a single number over a run that was healthy for a
  third of it says nothing about either
* **recovery time** -- how long after a fault clears the error takes to come
  back inside the band it held while healthy, and stay there
* whether the filter's own reported covariance still bounded the error, which
  is the difference between a filter that degrades and one that lies

WHY THE ROUTE IS OPEN LOOP
The robot is driven by timed velocity commands, not towards a goal. Fault
injection happens in the PREPROCESSING layer, downstream of the simulator, so
the wheels receive the same commands and the true trajectory is the same in
every scenario. That is the property the whole comparison rests on: any
difference between Case A and Case C is estimation, not a different drive.

A closed-loop route would destroy it. The controller would steer on the
corrupted estimate, the robot would physically go somewhere else, and the
resulting error would confound estimation quality with trajectory difference --
with no way to separate them afterwards.

Ground truth is used to SCORE, never to estimate. See docs/architecture.md
section 4.

Usage::

    # terminal 1 -- simulator, sensors, localization with scan matching
    ros2 launch arb_simulation simulation.launch.py headless:=true rviz:=false \\
        spawn:=odometry_test
    ros2 launch arb_sensors sensors.launch.py
    ros2 launch arb_localization odometry.launch.py initial_pose:=odometry_test
    ros2 launch arb_mapping mapping.launch.py

    # terminal 2
    python3 scripts/run_scenario.py --scenario configs/scenarios/case_c_encoder_drift.yaml
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import subprocess
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
import yaml  # noqa: E402
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
sys.path.insert(0, str(REPO_ROOT / "ros2_ws" / "src" / "arb_evaluation"))
from arb_evaluation import scenario as sc  # noqa: E402

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

#: The same rectangle Phase 5 used, at the same pose. Reusing it means the
#: nominal numbers here are directly comparable with the Phase 5 baseline.
LINEAR_SPEED = 0.25
ANGULAR_SPEED = 0.60
LONG_LEG_M = 1.5
SHORT_LEG_M = 1.0
MIN_CLEARANCE_M = 0.45

#: Recovery is declared when the position error has been back inside the
#: healthy band for this long. A single sample dipping under the threshold is
#: noise; half a lap of staying under it is recovery.
RECOVERY_HOLD_S = 3.0

#: The healthy band is `RECOVERY_BAND_FACTOR` times the error the SAME RUN held
#: before its first fault, floored at RECOVERY_BAND_FLOOR_M.
#:
#: Measured against the run's own healthy phase rather than against Case A: two
#: runs differ by more than the noise floor for reasons that have nothing to do
#: with the fault (physics seeds, where the map happened to be built), and a
#: threshold from a different run would make recovery time an artefact of that
#: difference.
RECOVERY_BAND_FACTOR = 2.0
RECOVERY_BAND_FLOOR_M = 0.05


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def build_route(duration_s: float):
    """Laps of a rectangle, enough of them to fill the scenario.

    Turns are what make dead reckoning fail, so a route that only drives
    straight flatters every estimator and would hide Case C entirely -- a
    one-wheel bias IS a turn as far as the encoders can tell.
    """
    turn = (math.pi / 2.0) / ANGULAR_SPEED
    long_leg = LONG_LEG_M / LINEAR_SPEED
    short_leg = SHORT_LEG_M / LINEAR_SPEED
    lap = [
        (LINEAR_SPEED, 0.0, long_leg),
        (0.0, ANGULAR_SPEED, turn),
        (LINEAR_SPEED, 0.0, short_leg),
        (0.0, ANGULAR_SPEED, turn),
        (LINEAR_SPEED, 0.0, long_leg),
        (0.0, ANGULAR_SPEED, turn),
        (LINEAR_SPEED, 0.0, short_leg),
        (0.0, ANGULAR_SPEED, turn),
    ]
    lap_time = sum(step[2] for step in lap)
    laps = max(1, int(math.ceil((duration_s - 2.0) / lap_time)))
    return [(0.0, 0.0, 2.0)] + lap * laps, lap_time, laps


class ScenarioRunner(Node):
    """Drives the route and records every estimate against ground truth."""

    def __init__(self) -> None:
        super().__init__("arb_scenario_runner")
        self.truth: Odometry | None = None
        self.odom: Odometry | None = None
        self.ekf: PoseWithCovarianceStamped | None = None
        self.scan: LaserScan | None = None
        self.fault = None
        self.samples: list[dict] = []
        self.aborted_reason: str | None = None

        self.create_subscription(
            Odometry, "/arb/ground_truth/odom", self._on_truth, SENSOR_QOS)
        self.create_subscription(Odometry, "/arb/odom", self._on_odom, SENSOR_QOS)
        self.create_subscription(
            PoseWithCovarianceStamped, "/arb/pose", self._on_ekf, SENSOR_QOS)
        self.create_subscription(LaserScan, "/arb/scan", self._on_scan, SENSOR_QOS)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", COMMAND_QOS)

    def _on_truth(self, m): self.truth = m
    def _on_odom(self, m): self.odom = m
    def _on_ekf(self, m): self.ekf = m
    def _on_scan(self, m): self.scan = m

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_until_ready(self, timeout: float = 60.0) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(x is not None for x in (self.truth, self.odom, self.ekf, self.scan)):
                return True
        print("timed out waiting for truth, odometry, the EKF pose and a scan",
              file=sys.stderr)
        return False

    def publish(self, v: float, w: float) -> None:
        t = Twist()
        t.linear.x = v
        t.angular.z = w
        self.cmd_pub.publish(t)

    def forward_clearance(self) -> float:
        """Nearest return in a +/-25 degree cone ahead, for the abort check."""
        if self.scan is None:
            return float("inf")
        r = np.asarray(self.scan.ranges, dtype=float)
        n = r.size
        if n == 0:
            return float("inf")
        half = max(1, int(round(math.radians(25.0) / self.scan.angle_increment)))
        cone = np.concatenate([r[:half], r[-half:]])
        good = cone[np.isfinite(cone) & (cone > self.scan.range_min)]
        return float(good.min()) if good.size else float("inf")

    def record(self, t0: float) -> None:
        if self.truth is None or self.odom is None:
            return
        tp = self.truth.pose.pose
        row = {
            "t": self._now() - t0,
            "truth": (tp.position.x, tp.position.y, yaw_of_quat(tp.orientation)),
            "odometry": (self.odom.pose.pose.position.x,
                         self.odom.pose.pose.position.y,
                         yaw_of_quat(self.odom.pose.pose.orientation)),
            "odometry_cov": (self.odom.pose.covariance[0],
                             self.odom.pose.covariance[7],
                             self.odom.pose.covariance[35]),
        }
        if self.ekf is not None:
            e = self.ekf
            row["ekf"] = (e.pose.pose.position.x, e.pose.pose.position.y,
                          yaw_of_quat(e.pose.pose.orientation))
            row["ekf_cov"] = (e.pose.covariance[0], e.pose.covariance[7],
                              e.pose.covariance[35])
        self.samples.append(row)

    def drive(self, route, t0: float, hz: float = 50.0) -> bool:
        for v, w, duration in route:
            end = self._now() + duration
            while self._now() < end and rclpy.ok():
                if v > 0.0 and self.forward_clearance() < MIN_CLEARANCE_M:
                    self.publish(0.0, 0.0)
                    self.spin_for(1.0)
                    self.aborted_reason = (
                        f"forward clearance fell to {self.forward_clearance():.2f} m")
                    return False
                self.publish(v, w)
                self.spin_for(1.0 / hz)
                self.record(t0)
        self.publish(0.0, 0.0)
        self.spin_for(1.0)
        self.record(t0)
        return True


def align(samples: list[dict], key: str):
    """Errors in the estimator's own starting frame.

    Only the FIRST sample is aligned, with a rigid transform and no fitting.
    Fitting the whole trajectory would absorb part of the drift into the fit
    and flatter the estimator -- which in a fault-injection experiment would
    absorb exactly the thing being measured.
    """
    rows = [s for s in samples if key in s]
    if len(rows) < 10:
        return None
    t = np.array([s["t"] for s in rows])
    ex = np.array([s[key][0] for s in rows])
    ey = np.array([s[key][1] for s in rows])
    eth = np.array([s[key][2] for s in rows])
    tx = np.array([s["truth"][0] for s in rows])
    ty = np.array([s["truth"][1] for s in rows])
    tth = np.array([s["truth"][2] for s in rows])

    dth = wrap(eth[0] - tth[0])
    c, s = math.cos(dth), math.sin(dth)
    tx_a = c * (tx - tx[0]) - s * (ty - ty[0]) + ex[0]
    ty_a = s * (tx - tx[0]) + c * (ty - ty[0]) + ey[0]
    tth_a = np.array([wrap(a + dth) for a in tth])
    cov = np.array([s.get(f"{key}_cov", (0.0, 0.0, 0.0)) for s in rows])
    return {
        "t": t,
        "err_pos": np.hypot(ex - tx_a, ey - ty_a),
        "err_th": np.array([wrap(a - b) for a, b in zip(eth, tth_a)]),
        "sigma_pos": np.sqrt(np.maximum(0.0, cov[:, 0] + cov[:, 1])),
        "sigma_th": np.sqrt(np.maximum(0.0, cov[:, 2])),
        "x": ex, "y": ey, "tx": tx_a, "ty": ty_a,
        "distance": np.concatenate(
            [[0.0], np.cumsum(np.hypot(np.diff(tx), np.diff(ty)))]),
    }


def phase_masks(t: np.ndarray, windows: list[dict]):
    """Split a run into healthy-before, degraded, and recovered-after."""
    if not windows:
        return {"healthy": np.ones_like(t, dtype=bool),
                "degraded": np.zeros_like(t, dtype=bool),
                "recovered": np.zeros_like(t, dtype=bool)}
    first = min(w["onset_s"] for w in windows)
    last = max(w["end_s"] for w in windows)
    degraded = np.zeros_like(t, dtype=bool)
    for w in windows:
        degraded |= (t >= w["onset_s"]) & (t < w["end_s"])
    return {
        "healthy": t < first,
        "degraded": degraded,
        # Everything after the last fault ends, whether or not a gap between
        # two faults also counts as healthy -- that gap is not recovery, it is
        # a lull, and averaging it in would flatter the recovered phase.
        "recovered": t >= last,
    }


def _rmse(x) -> float:
    return float(np.sqrt(np.mean(np.square(x)))) if np.size(x) else float("nan")


def recovery_time(t, err, windows, band) -> tuple[float, str]:
    """Seconds from the last fault clearing until the error is back in band.

    Returns NaN with a reason when there is nothing to measure -- no fault, no
    tail, or an error that never came back. Reporting 0.0 for "there was no
    fault" and 0.0 for "it recovered instantly" would make the two
    indistinguishable in the results table.
    """
    if not windows:
        return float("nan"), "no fault"
    last = max(w["end_s"] for w in windows)
    tail = (t >= last)
    if not tail.any():
        return float("nan"), "no post-fault window"
    tt, ee = t[tail], err[tail]
    if ee[0] <= band:
        return 0.0, "never left the band"
    inside = ee <= band
    for i in range(len(tt)):
        if not inside[i]:
            continue
        # Must STAY inside for RECOVERY_HOLD_S, or a single sample dipping
        # under the threshold counts as recovery.
        hold = (tt >= tt[i]) & (tt <= tt[i] + RECOVERY_HOLD_S)
        if hold.sum() >= 2 and inside[hold].all():
            return float(tt[i] - last), "recovered"
    return float("nan"), "did not recover"


def score(aligned: dict, windows: list[dict]) -> dict:
    t = aligned["t"]
    ep, eth = aligned["err_pos"], aligned["err_th"]
    m = phase_masks(t, windows)

    healthy_rmse = _rmse(ep[m["healthy"]])
    band = max(RECOVERY_BAND_FACTOR * healthy_rmse, RECOVERY_BAND_FLOOR_M) \
        if np.isfinite(healthy_rmse) else RECOVERY_BAND_FLOOR_M
    rec, rec_reason = recovery_time(t, ep, windows, band)

    out = {
        "duration_s": float(t[-1]),
        "distance_m": float(aligned["distance"][-1]),
        "position_rmse_m": _rmse(ep),
        "position_max_m": float(np.max(ep)),
        "position_final_m": float(ep[-1]),
        "heading_rmse_deg": float(np.degrees(_rmse(eth))),
        "heading_max_deg": float(np.degrees(np.max(np.abs(eth)))),
        "position_rmse_healthy_m": healthy_rmse,
        "position_rmse_degraded_m": _rmse(ep[m["degraded"]]),
        "position_rmse_recovered_m": _rmse(ep[m["recovered"]]),
        "heading_rmse_healthy_deg": float(np.degrees(_rmse(eth[m["healthy"]]))),
        "heading_rmse_degraded_deg": float(np.degrees(_rmse(eth[m["degraded"]]))),
        "heading_rmse_recovered_deg": float(np.degrees(_rmse(eth[m["recovered"]]))),
        "recovery_band_m": band,
        "recovery_time_s": rec,
        "recovery_outcome": rec_reason,
        # A filter that degrades honestly widens its covariance to match. One
        # that stays confident while its error grows is worse than a bad
        # filter: everything downstream believes it.
        "fraction_inside_2sigma_pos": float(np.mean(ep <= 2 * aligned["sigma_pos"]))
        if np.any(aligned["sigma_pos"] > 0) else float("nan"),
        "fraction_inside_2sigma_heading": float(
            np.mean(np.abs(eth) <= 2 * aligned["sigma_th"]))
        if np.any(aligned["sigma_th"] > 0) else float("nan"),
        "reported_final_sigma_pos_m": float(aligned["sigma_pos"][-1]),
    }
    return out


def plot_run(results: dict, windows: list[dict], scenario_id: str,
             out_dir: pathlib.Path) -> list[pathlib.Path]:
    written = []
    colours = {"odometry": "#e03131", "ekf": "#1971c2"}

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for name, a in results.items():
        axes[0].plot(a["t"], a["err_pos"], color=colours[name], lw=1.3, label=name)
        axes[1].plot(a["t"], np.degrees(a["err_th"]), color=colours[name], lw=1.3,
                     label=name)
    axes[2].plot(results["ekf"]["t"], results["ekf"]["err_pos"],
                 color=colours["ekf"], lw=1.3, label="EKF error")
    axes[2].plot(results["ekf"]["t"], 2 * results["ekf"]["sigma_pos"],
                 color="#868e96", ls="--", lw=1.2, label="EKF 2-sigma")

    for ax in axes:
        for w in windows:
            ax.axvspan(w["onset_s"], w["end_s"], color="#fa5252", alpha=0.10)
        ax.grid(alpha=0.25)
        ax.legend(fontsize=8)
    # Label each band once, on the top axes only.
    for w in windows:
        axes[0].annotate(w["type"].replace("_", " "),
                         xy=((w["onset_s"] + w["end_s"]) / 2.0, 0.94),
                         xycoords=("data", "axes fraction"),
                         ha="center", fontsize=7, color="#c92a2a")
    axes[0].set_ylabel("position error [m]")
    axes[0].set_title(f"{scenario_id} -- shaded spans are the injected faults")
    axes[1].set_ylabel("heading error [deg]")
    axes[2].set_ylabel("EKF error vs its own 2-sigma [m]")
    axes[2].set_xlabel("time [s]")
    p = out_dir / f"{scenario_id}_errors.png"
    fig.tight_layout()
    fig.savefig(p, dpi=140)
    plt.close(fig)
    written.append(p)

    fig, ax = plt.subplots(figsize=(7.5, 6.5))
    a = results["ekf"]
    ax.plot(a["tx"], a["ty"], color="#2f9e44", lw=2.0, label="ground truth")
    for name, r in results.items():
        ax.plot(r["x"], r["y"], color=colours[name], lw=1.3, ls="--", label=name)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"{scenario_id} -- trajectory")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.25)
    p = out_dir / f"{scenario_id}_trajectory.png"
    fig.tight_layout()
    fig.savefig(p, dpi=140)
    plt.close(fig)
    written.append(p)
    return written


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", required=True)
    ap.add_argument("--out", default=None)
    ap.add_argument("--no-scheduler", action="store_true",
                    help="Assume the fault scheduler is already running.")
    args = ap.parse_args(argv)

    path = pathlib.Path(args.scenario)
    if not path.is_absolute():
        path = REPO_ROOT / path
    spec_file = yaml.safe_load(path.read_text())
    sc.validate(spec_file)
    spec = spec_file["scenario"]
    scenario_id = spec["id"]
    duration = float(spec["duration_s"])
    windows = sc.windows(spec_file)

    out_dir = pathlib.Path(args.out) if args.out else REPO_ROOT / "results" / "phase10"
    out_dir.mkdir(parents=True, exist_ok=True)

    route, lap_time, laps = build_route(duration)
    print(f"=== {scenario_id} ===")
    print(f"  {spec.get('description', '')}")
    print(f"  {duration:.0f} s, {laps} laps of {lap_time:.1f} s, "
          f"{len(windows)} fault window(s)")

    rclpy.init()
    node = ScenarioRunner()
    scheduler = None
    try:
        if not node.wait_until_ready():
            return 2
        # Started only once the estimator is alive, so t = 0 in the scenario is
        # the instant the robot starts driving rather than some point during
        # bring-up. Every onset in the file is relative to that.
        if not args.no_scheduler:
            scheduler = subprocess.Popen(
                ["ros2", "run", "arb_evaluation", "fault_scheduler", "--ros-args",
                 "-p", "use_sim_time:=true", "-p", f"scenario:={path}"],
                cwd=REPO_ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        t0 = node._now()
        ok = node.drive(route, t0)
        if not ok:
            print(f"  ABORTED: {node.aborted_reason}", file=sys.stderr)
    finally:
        if scheduler is not None and scheduler.poll() is None:
            scheduler.terminate()
            try:
                scheduler.wait(timeout=10)
            except subprocess.TimeoutExpired:
                scheduler.kill()
        node.publish(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

    results = {}
    for key in ("odometry", "ekf"):
        a = align(node.samples, key)
        if a is not None:
            results[key] = a
    if "ekf" not in results:
        print("no EKF samples recorded", file=sys.stderr)
        return 3

    metrics = {
        "scenario": scenario_id,
        "description": spec.get("description", ""),
        "aborted": node.aborted_reason,
        "fault_windows": windows,
        "samples": len(node.samples),
        "estimators": {k: score(a, windows) for k, a in results.items()},
    }
    with open(out_dir / f"metrics_{scenario_id}.json", "w") as fh:
        json.dump(metrics, fh, indent=2)
    for p in plot_run(results, windows, scenario_id, out_dir):
        print(f"  wrote {p}")
    print(f"  wrote {out_dir}/metrics_{scenario_id}.json")

    for name, m in metrics["estimators"].items():
        print(f"  --- {name} ---")
        print(f"    position RMSE  {m['position_rmse_m']:.4f} m "
              f"(healthy {m['position_rmse_healthy_m']:.4f}, "
              f"degraded {m['position_rmse_degraded_m']:.4f}, "
              f"recovered {m['position_rmse_recovered_m']:.4f})")
        print(f"    heading RMSE   {m['heading_rmse_deg']:.3f} deg "
              f"(healthy {m['heading_rmse_healthy_deg']:.3f}, "
              f"degraded {m['heading_rmse_degraded_deg']:.3f}, "
              f"recovered {m['heading_rmse_recovered_deg']:.3f})")
        print(f"    recovery       {m['recovery_time_s']:.2f} s "
              f"({m['recovery_outcome']}, band {m['recovery_band_m']:.4f} m)")
        print(f"    inside 2-sigma {m['fraction_inside_2sigma_pos'] * 100:.1f} %")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
