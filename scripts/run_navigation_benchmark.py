#!/usr/bin/env python3
"""Navigate the same route with and without an obstacle that is not on the map.

Phase 11's fifth experiment, and the only one that tests the LOCAL PLANNER
rather than the estimator. Everything else in this phase degrades a sensor and
measures an estimate; this degrades the world and measures whether the robot
still gets there without hitting anything.

THE CONTROL IS THE POINT
Each seed is run twice — once with the obstacle and once without — and the
pairs are compared. An absolute arrival time means very little on its own
because the route is replanned as the map fills in and no two runs take quite
the same path; the DIFFERENCE between a paired run with and without the
obstacle is what the obstacle cost.

WHY THE OBSTACLE IS NOT IN THE WORLD FILE
It has to be absent from the map. The global planner routes through space the
occupancy grid calls free, and something walks into it. An obstacle present at
startup would simply be mapped on the way past and A* would route around it,
which tests the costmap and nothing about reacting.

Usage::

    python3 scripts/run_navigation_benchmark.py
    python3 scripts/run_navigation_benchmark.py --seeds 1 2 3 4 5
    python3 scripts/run_navigation_benchmark.py --aggregate-only
"""

from __future__ import annotations

import argparse
import json
import math
import os
import pathlib
import shutil
import signal
import subprocess
import sys
import tempfile
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
SENSOR_PARAMS = REPO_ROOT / "ros2_ws" / "src" / "arb_sensors" / "config" / "sensors.yaml"
SEED_NODES = ("imu_preprocessor", "encoder_preprocessor", "lidar_preprocessor")

#: The robot drives the corridor west to east; the obstacle crosses it at x = 0,
#: which is far enough in that the robot is at full speed and the map of the
#: corridor is already built.
OBSTACLE_X = 0.0
OBSTACLE_SPEED = 0.45
STARTUP_S = 30.0
RUN_TIMEOUT_S = 420.0
GOAL = "goal_far_east"


def sensor_params_with_seeds(seed: int, out_path: pathlib.Path) -> pathlib.Path:
    """Offset every sensor seed, keeping the three different from each other."""
    cfg = yaml.safe_load(SENSOR_PARAMS.read_text())
    for key, block in cfg.items():
        node = key.split("/")[-1]
        if node in SEED_NODES and "seed" in block["ros__parameters"]:
            block["ros__parameters"]["seed"] = (
                int(block["ros__parameters"]["seed"]) + 1000 * int(seed))
    out_path.write_text(yaml.safe_dump(cfg, sort_keys=False))
    return out_path


class NavStack:
    """The full navigation stack, optionally with a moving obstacle."""

    def __init__(self, sensor_params: pathlib.Path, log: pathlib.Path,
                 with_obstacle: bool, predict: bool = True):
        self.sensor_params = sensor_params
        self.log = log
        self.with_obstacle = with_obstacle
        # Whether the local planner advances obstacles along their measured
        # velocity. False reproduces the Phase 11 planner exactly, so the two
        # arms differ by one parameter and nothing else.
        self.predict = predict
        self.procs: list[subprocess.Popen] = []

    def _spawn(self, *args: str) -> subprocess.Popen:
        # start_new_session so the whole launch tree can be signalled as a
        # group: `ros2 launch` children outlive a terminate() on the parent and
        # an orphaned gz sim holds the port the next run needs.
        return subprocess.Popen(
            args, cwd=REPO_ROOT, stdout=self.log.open("a"),
            stderr=subprocess.STDOUT, start_new_session=True)

    def up(self) -> None:
        self.procs.append(self._spawn(
            "ros2", "launch", "arb_simulation", "simulation.launch.py",
            "headless:=true", "rviz:=false", "spawn:=spawn"))
        time.sleep(12)
        self.procs.append(self._spawn(
            "ros2", "launch", "arb_sensors", "sensors.launch.py",
            f"params:={self.sensor_params}"))
        time.sleep(3)
        self.procs.append(self._spawn(
            "ros2", "launch", "arb_localization", "odometry.launch.py",
            "initial_pose:=spawn"))
        time.sleep(3)
        self.procs.append(self._spawn(
            "ros2", "launch", "arb_mapping", "mapping.launch.py"))
        time.sleep(2)
        self.procs.append(self._spawn(
            "ros2", "launch", "arb_planning", "planning.launch.py",
            "local_planner:=true", "unknown_is_lethal:=false",
            f"predict_obstacle_motion:={'true' if self.predict else 'false'}"))
        if self.with_obstacle:
            self.procs.append(self._spawn(
                "ros2", "run", "arb_evaluation", "moving_obstacle", "--ros-args",
                "-p", "use_sim_time:=true",
                "-p", f"x_m:={OBSTACLE_X}",
                "-p", f"speed_m_s:={OBSTACLE_SPEED}"))
        time.sleep(max(0.0, STARTUP_S - 20))

    def down(self) -> None:
        for p in self.procs:
            if p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGINT)
                except (ProcessLookupError, PermissionError):
                    pass
        deadline = time.time() + 12
        for p in self.procs:
            try:
                p.wait(timeout=max(0.5, deadline - time.time()))
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass
        self.procs.clear()
        time.sleep(3)


def run_one(seed: int, with_obstacle: bool, out_dir: pathlib.Path,
            work: pathlib.Path, predict: bool = True) -> dict | None:
    arm = "obstacle" if with_obstacle else "clear"
    if with_obstacle and not predict:
        arm = "obstacle_blind"
    tag = f"nav_{arm}_seed{seed}"
    params = sensor_params_with_seeds(seed, work / f"sensors_seed{seed}.yaml")
    stack = NavStack(params, work / f"{tag}.log", with_obstacle, predict)
    try:
        stack.up()
        result = subprocess.run(
            [sys.executable, "scripts/evaluate_navigation.py",
             "--mode", "goal", "--goal", GOAL, "--duration", "180",
             "--controller", "dwa", "--tag", tag, "--out", str(out_dir)],
            cwd=REPO_ROOT, capture_output=True, text=True, timeout=RUN_TIMEOUT_S)
        if result.returncode != 0:
            tail = result.stderr.strip().splitlines()[-1:] or ["no stderr"]
            print(f"    FAILED (rc={result.returncode}): {tail}")
            return None
    except subprocess.TimeoutExpired:
        print(f"    TIMED OUT after {RUN_TIMEOUT_S:.0f} s")
        return None
    finally:
        stack.down()

    path = out_dir / f"metrics_goal_{tag}.json"
    if not path.exists():
        print(f"    no metrics at {path}")
        return None
    metrics = json.loads(path.read_text())
    metrics["seed"] = seed
    metrics["with_obstacle"] = with_obstacle
    metrics["predict_obstacle_motion"] = predict
    path.write_text(json.dumps(metrics, indent=2))
    return metrics


# ------------------------------------------------------------------ aggregate
FIELDS = [
    ("arrived", "arrived", None),
    ("time_s", "time to goal [s]", ".1f"),
    ("distance_m", "distance driven [m]", ".2f"),
    ("path_efficiency", "path efficiency", ".3f"),
    # The LiDAR-derived number bottoms out at range_min and is reported for
    # continuity with the other phases, NOT as a collision verdict -- see
    # min_obstacle_clearance_m for that.
    ("min_lidar_range_m", "nearest measurable return [m]", ".3f"),
    ("min_obstacle_clearance_m", "true obstacle clearance [m]", ".3f"),
    ("mean_speed_m_s", "mean speed [m/s]", ".3f"),
    ("final_error_m", "final error [m]", ".3f"),
]


#: The three arms, in report order.
ARMS = ("clear", "obstacle_blind", "obstacle_predict")


def arm_of(m: dict) -> str:
    if not m.get("with_obstacle"):
        return "clear"
    return ("obstacle_predict" if m.get("predict_obstacle_motion", True)
            else "obstacle_blind")


def collect(out_dir: pathlib.Path) -> dict[str, list[dict]]:
    groups: dict[str, list[dict]] = {a: [] for a in ARMS}
    for path in sorted(out_dir.glob("metrics_goal_nav_*.json")):
        m = json.loads(path.read_text())
        if "with_obstacle" in m:
            groups[arm_of(m)].append(m)
    return groups


def summarise(runs: list[dict]) -> dict:
    out = {"runs": len(runs)}
    if not runs:
        return out
    out["arrived"] = sum(1 for r in runs if r.get("arrived"))
    # Two verdicts, deliberately separate. The LiDAR one is blind inside
    # range_min; the geometric one is computed from ground truth and is the one
    # to believe when an obstacle is present.
    out["collisions"] = sum(1 for r in runs if not r.get("collision_free", True))
    out["touched"] = sum(1 for r in runs if r.get("touched_obstacle", False))
    out["blind_spot"] = sum(1 for r in runs if r.get("blind_spot_samples", 0) > 0)
    for key, _, _ in FIELDS:
        if key == "arrived":
            continue
        values = [r[key] for r in runs
                  if r.get(key) is not None and math.isfinite(r[key])]
        if values:
            out[f"{key}_mean"] = float(np.mean(values))
            out[f"{key}_std"] = float(np.std(values))
            out[f"{key}_min"] = float(np.min(values))
    return out


LABELS = {
    "clear": "clear corridor",
    "obstacle_blind": "obstacle, no prediction",
    "obstacle_predict": "obstacle, predicted",
}
COLOURS = {"clear": "#1971c2", "obstacle_blind": "#e03131",
           "obstacle_predict": "#2f9e44"}


def plot(groups: dict[str, list[dict]], out_dir: pathlib.Path):
    present = [a for a in ARMS if groups.get(a)]
    if not present:
        return None
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.6))
    panels = [("min_obstacle_clearance_m", "true obstacle clearance [m]"),
              ("time_s", "time to goal [s]")]
    for ax, (key, label) in zip(axes, panels):
        for pos, arm in enumerate(present):
            values = [r[key] for r in groups[arm]
                      if r.get(key) is not None and math.isfinite(r[key])]
            if not values:
                continue
            ax.scatter([pos] * len(values), values, color=COLOURS[arm],
                       zorder=3, s=55)
            ax.plot([pos - 0.18, pos + 0.18], [np.mean(values)] * 2,
                    color=COLOURS[arm], lw=2.5, zorder=2)
        ax.set_xticks(range(len(present)))
        ax.set_xticklabels([LABELS[a] for a in present], fontsize=8)
        ax.set_ylabel(label)
        ax.grid(alpha=0.25, axis="y")
    # Zero clearance is contact; below it the two bodies overlap.
    axes[0].axhline(0.0, color="#c2255c", ls=":", lw=1.4, label="contact")
    axes[0].legend(fontsize=8)
    fig.suptitle("Phase 11 -- navigation past an obstacle that is not on the map")
    path = out_dir / "sweep_dynamic_obstacle.png"
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def report(groups: dict[str, list[dict]], out_dir: pathlib.Path) -> pathlib.Path:
    present = [a for a in ARMS if groups.get(a)]
    summaries = {a: summarise(groups[a]) for a in present}
    head = "| metric | " + " | ".join(LABELS[a] for a in present) + " |"
    lines = [head, "|" + "---|" * (len(present) + 1)]

    def row(name, fn):
        lines.append(f"| {name} | "
                     + " | ".join(fn(summaries[a]) for a in present) + " |")

    row("runs", lambda d: str(d.get("runs", 0)))
    row("arrived", lambda d: f"{d.get('arrived', 0)} / {d.get('runs', 0)}")
    row("touched the obstacle (ground truth)",
        lambda d: str(d.get("touched", 0)))
    row("runs with a beam inside range_min",
        lambda d: str(d.get("blind_spot", 0)))
    for key, label, spec in FIELDS:
        if key == "arrived":
            continue

        def cell(d, key=key, spec=spec):
            mean = d.get(f"{key}_mean")
            if mean is None:
                return "--"
            return f"{mean:{spec}} ± {d.get(f'{key}_std', 0.0):{spec}}"
        row(label, cell)

    path = out_dir / "dynamic_obstacle.md"
    path.write_text("\n".join(lines) + "\n")
    return path


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3])
    ap.add_argument("--out", default=None)
    ap.add_argument("--aggregate-only", action="store_true")
    args = ap.parse_args(argv)

    out_dir = pathlib.Path(args.out) if args.out else REPO_ROOT / "results" / "phase11"
    out_dir.mkdir(parents=True, exist_ok=True)

    if not args.aggregate_only:
        work = pathlib.Path(tempfile.mkdtemp(prefix="arb_nav_benchmark_"))
        print(f"working directory {work}")
        # Paired: the same seed with and without, back to back, so a drift in
        # machine load between the halves cannot masquerade as an effect.
        # Three arms per seed, run back to back so a drift in machine load
        # cannot masquerade as an effect:
        #   clear          -- no obstacle, the baseline
        #   obstacle_blind -- obstacle, prediction OFF (the Phase 11 planner)
        #   obstacle       -- obstacle, prediction ON
        plan = [(seed, flag, predict)
                for seed in args.seeds
                for flag, predict in ((False, True), (True, False), (True, True))]
        started = time.time()
        try:
            for i, (seed, flag, predict) in enumerate(plan, start=1):
                eta = ((time.time() - started) / (i - 1) * (len(plan) - i + 1) / 60
                       if i > 1 else 0)
                arm = ("clear" if not flag
                       else ("obstacle+predict" if predict else "obstacle+blind"))
                print(f"  [{i}/{len(plan)}] seed={seed} {arm} "
                      f"(eta {eta:.0f} min)", flush=True)
                run_one(seed, flag, out_dir, work, predict)
        finally:
            shutil.rmtree(work, ignore_errors=True)

    groups = collect(out_dir)
    path = plot(groups, out_dir)
    if path:
        print(f"wrote {path}")
    print(f"wrote {report(groups, out_dir)}")
    print()
    print((out_dir / "dynamic_obstacle.md").read_text())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
