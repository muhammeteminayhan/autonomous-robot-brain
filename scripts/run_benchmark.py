#!/usr/bin/env python3
"""Sweep one fault parameter across a range, repeated over seeds.

Phase 11's deliverable, and the answer to Phase 10's biggest weakness: every
number there was a SINGLE run at a SINGLE magnitude. That is enough to show a
failure mode does what the theory says and not enough to separate an effect
from run-to-run variation, nor to say how much of a fault the estimator can
take before it stops coping.

Each point here is repeated over seeds and reported as a mean with a spread.

WHAT ONE RUN IS
    fresh simulator -> fresh EKF -> fresh map -> fixed open-loop route
    -> one fault at one magnitude -> score against ground truth

The route is the same one Phase 10 used and is driven open loop, so the TRUE
trajectory is identical across every run in a sweep and any difference in the
result is estimation error. See docs/fault-injection.md section 1.

WHY EACH RUN RESTARTS EVERYTHING
Cheaper options were considered and are all wrong. Restarting only the sensor
nodes would leave the EKF holding the previous run's state and the mapping node
holding the previous run's map, so run N would inherit run N-1's error --
exactly the contamination the scheduler goes out of its way to avoid within a
run. Seeds cannot be changed at runtime either: the noise generators are
constructed with them, and the nodes reject the attempt rather than let the
parameter server disagree with the running model.

Usage::

    python3 scripts/run_benchmark.py                       # every experiment
    python3 scripts/run_benchmark.py --experiment encoder_bias
    python3 scripts/run_benchmark.py --dry-run             # print the plan
    python3 scripts/run_benchmark.py --aggregate-only      # re-derive the report
"""

from __future__ import annotations

import argparse
import copy
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
BENCHMARK_CFG = REPO_ROOT / "configs" / "benchmarks.yaml"
SENSOR_PARAMS = REPO_ROOT / "ros2_ws" / "src" / "arb_sensors" / "config" / "sensors.yaml"

#: Seconds to wait after launching before the estimator is expected to be alive.
#: run_scenario.py waits for its topics anyway; this only keeps the launch log
#: readable and avoids hammering a half-started graph.
STARTUP_S = 26.0

#: A run that hangs must not stall a two-hour sweep. Generous: a 75 s scenario
#: at a real-time factor below 1 can legitimately take several minutes.
RUN_TIMEOUT_S = 600.0

#: Which node's seed each sensor's noise comes from, and the parameter path.
SEED_NODES = ("imu_preprocessor", "encoder_preprocessor", "lidar_preprocessor")


def load_config() -> dict:
    return yaml.safe_load(BENCHMARK_CFG.read_text())["benchmark"]


def scenario_for(experiment: dict, level: float, duration_s: float) -> dict:
    """Build a one-fault scenario at the given magnitude.

    A level of zero produces a scenario with NO faults rather than a fault of
    magnitude zero. They are not the same thing: a zero-magnitude fault still
    drives the parameter round trip and still splits the run into phases, so
    its "degraded" numbers would be reported against a robot that was never
    degraded. The zero point is the control, and it has to be a real one.
    """
    spec = {
        "id": f"{experiment['id']}_{level:g}".replace(".", "p").replace("-", "neg"),
        "description": f"{experiment['title']} at {level:g} {experiment['unit']}",
        "duration_s": float(duration_s),
        "faults": [],
    }
    if level == 0.0:
        return {"scenario": spec}

    fault = copy.deepcopy(experiment["fault"])
    params = fault.setdefault("params", {})
    name = experiment["parameter"]
    if name == "outage_s":
        # The swept quantity IS the duration for this experiment.
        fault["duration_s"] = float(level)
    else:
        params[name] = float(level)
        fault["duration_s"] = float(fault["duration_s"])
    fault["onset_s"] = float(fault["onset_s"])
    spec["faults"] = [fault]
    return {"scenario": spec}


def sensor_params_with_seeds(seed: int, out_path: pathlib.Path) -> pathlib.Path:
    """Copy the checked-in sensor parameters with every seed offset by `seed`.

    Offset rather than replaced, so the three sensors keep DIFFERENT seeds from
    each other. Giving them all the same value would correlate their noise
    streams, and a correlated gyro and encoder is a physically different robot
    from the one every other phase measured.
    """
    cfg = yaml.safe_load(SENSOR_PARAMS.read_text())
    for key, block in cfg.items():
        node = key.split("/")[-1]
        if node in SEED_NODES and "seed" in block["ros__parameters"]:
            block["ros__parameters"]["seed"] = (
                int(block["ros__parameters"]["seed"]) + 1000 * int(seed))
    out_path.write_text(yaml.safe_dump(cfg, sort_keys=False))
    return out_path


class StackRunner:
    """Brings the estimation stack up and down for one run."""

    def __init__(self, spawn: str, sensor_params: pathlib.Path, log: pathlib.Path):
        self.spawn = spawn
        self.sensor_params = sensor_params
        self.log = log
        self.procs: list[subprocess.Popen] = []

    def _launch(self, *args: str) -> subprocess.Popen:
        # start_new_session so a whole launch tree can be signalled as a group:
        # `ros2 launch` spawns children that outlive a plain terminate() on the
        # parent, and an orphaned gz sim holds the port the next run needs.
        return subprocess.Popen(
            ["ros2", "launch", *args],
            cwd=REPO_ROOT, stdout=self.log.open("a"), stderr=subprocess.STDOUT,
            start_new_session=True)

    def up(self) -> None:
        self.procs.append(self._launch(
            "arb_simulation", "simulation.launch.py",
            "headless:=true", "rviz:=false", f"spawn:={self.spawn}"))
        time.sleep(12)
        self.procs.append(self._launch(
            "arb_sensors", "sensors.launch.py", f"params:={self.sensor_params}"))
        time.sleep(3)
        self.procs.append(self._launch(
            "arb_localization", "odometry.launch.py",
            f"initial_pose:={self.spawn}"))
        time.sleep(3)
        self.procs.append(self._launch("arb_mapping", "mapping.launch.py"))
        time.sleep(max(0.0, STARTUP_S - 18))

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
        # gz sim can outlive its launch parent; give the ports time to free.
        time.sleep(3)


def run_one(experiment: dict, level: float, seed: int, cfg: dict,
            raw_dir: pathlib.Path, work: pathlib.Path) -> dict | None:
    """One (level, seed) point. Returns its metrics, or None if it failed."""
    tag = f"{experiment['id']}_lvl{level:g}_seed{seed}"
    scenario = scenario_for(experiment, level, cfg["duration_s"])
    scenario_path = work / f"{tag}.yaml"
    scenario_path.write_text(yaml.safe_dump(scenario, sort_keys=False))
    params_path = sensor_params_with_seeds(seed, work / f"sensors_seed{seed}.yaml")
    log = work / f"{tag}.log"

    stack = StackRunner(cfg["spawn"], params_path, log)
    try:
        stack.up()
        result = subprocess.run(
            [sys.executable, "scripts/run_scenario.py",
             "--scenario", str(scenario_path), "--out", str(raw_dir)],
            cwd=REPO_ROOT, capture_output=True, text=True, timeout=RUN_TIMEOUT_S)
        if result.returncode != 0:
            print(f"    FAILED (rc={result.returncode}): "
                  f"{result.stderr.strip().splitlines()[-1:] or ['no stderr']}")
            return None
    except subprocess.TimeoutExpired:
        print(f"    TIMED OUT after {RUN_TIMEOUT_S:.0f} s")
        return None
    finally:
        stack.down()

    metrics_path = raw_dir / f"metrics_{scenario['scenario']['id']}.json"
    if not metrics_path.exists():
        print(f"    no metrics written at {metrics_path}")
        return None
    metrics = json.loads(metrics_path.read_text())
    metrics["level"] = level
    metrics["seed"] = seed
    metrics["experiment"] = experiment["id"]
    # Rename so repeats of the same level do not overwrite each other: the
    # scenario id carries the level but not the seed.
    metrics_path.rename(raw_dir / f"{tag}.json")
    (raw_dir / f"{tag}.json").write_text(json.dumps(metrics, indent=2))
    return metrics


# ------------------------------------------------------------------ aggregate
def aggregate(raw_dir: pathlib.Path, experiment: dict) -> list[dict]:
    """Mean and spread per level, over seeds."""
    by_level: dict[float, list[dict]] = {}
    for path in sorted(raw_dir.glob(f"{experiment['id']}_lvl*_seed*.json")):
        m = json.loads(path.read_text())
        by_level.setdefault(float(m["level"]), []).append(m)

    rows = []
    for level in sorted(by_level):
        runs = by_level[level]
        row = {"level": level, "runs": len(runs)}
        for estimator in ("ekf", "odometry"):
            for field in ("position_rmse_m", "heading_rmse_deg",
                          "position_rmse_degraded_m", "fraction_inside_2sigma_pos"):
                values = [r["estimators"][estimator][field] for r in runs
                          if estimator in r["estimators"]]
                values = [v for v in values if v is not None and math.isfinite(v)]
                if values:
                    row[f"{estimator}_{field}_mean"] = float(np.mean(values))
                    # Population spread over the repeats, not a standard error:
                    # three samples cannot support a confidence interval, and
                    # printing one would imply a precision that is not there.
                    row[f"{estimator}_{field}_std"] = float(np.std(values))
            outcomes = [r["estimators"][estimator].get("recovery_outcome")
                        for r in runs if estimator in r["estimators"]]
            row[f"{estimator}_recovered"] = sum(
                1 for o in outcomes if o in ("recovered", "never left the band"))
            times = [r["estimators"][estimator].get("recovery_time_s")
                     for r in runs if estimator in r["estimators"]]
            times = [t for t in times if t is not None and math.isfinite(t)]
            row[f"{estimator}_recovery_time_mean"] = (
                float(np.mean(times)) if times else float("nan"))
        rows.append(row)
    return rows


def plot_experiment(experiment: dict, rows: list[dict],
                    out_dir: pathlib.Path) -> pathlib.Path | None:
    if not rows:
        return None
    x = np.array([r["level"] for r in rows])
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.4))

    for ax, field, label in [
            (axes[0], "position_rmse_m", "position RMSE [m]"),
            (axes[1], "heading_rmse_deg", "heading RMSE [deg]")]:
        for estimator, colour in (("odometry", "#e03131"), ("ekf", "#1971c2")):
            mean = np.array([r.get(f"{estimator}_{field}_mean", np.nan) for r in rows])
            std = np.array([r.get(f"{estimator}_{field}_std", 0.0) for r in rows])
            ax.plot(x, mean, "o-", color=colour, lw=1.6, label=estimator)
            ax.fill_between(x, mean - std, mean + std, color=colour, alpha=0.18)
        ax.set_xlabel(f"{experiment['parameter']} [{experiment['unit']}]")
        ax.set_ylabel(label)
        ax.grid(alpha=0.25)
        ax.legend(fontsize=8)

    n = max(r["runs"] for r in rows)
    recovered = np.array([r.get("ekf_recovered", 0) for r in rows])
    axes[2].bar(x, recovered, width=(x[1] - x[0]) * 0.6 if len(x) > 1 else 0.5,
                color="#2f9e44")
    axes[2].axhline(n, color="#adb5bd", ls="--", lw=1.0, label=f"all {n} runs")
    axes[2].set_ylim(0, n + 0.4)
    axes[2].set_xlabel(f"{experiment['parameter']} [{experiment['unit']}]")
    axes[2].set_ylabel("EKF runs that recovered")
    axes[2].grid(alpha=0.25, axis="y")
    axes[2].legend(fontsize=8)

    fig.suptitle(f"{experiment['title']} -- mean over {n} seeds, band is 1 std")
    path = out_dir / f"sweep_{experiment['id']}.png"
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def markdown_table(experiment: dict, rows: list[dict]) -> str:
    def cell(row: dict, prefix: str, field: str, spec: str = ".4f") -> str:
        mean = row.get(f"{prefix}_{field}_mean")
        if mean is None:
            return "--"
        std = row.get(f"{prefix}_{field}_std")
        return f"{mean:{spec}} ± {std:{spec}}" if std is not None else f"{mean:{spec}}"

    out = [
        f"| {experiment['parameter']} [{experiment['unit']}] | runs "
        f"| EKF pos RMSE [m] | odom pos RMSE [m] | EKF heading RMSE [deg] "
        f"| EKF recovered | EKF inside 2-sigma |",
        "|---|---|---|---|---|---|---|",
    ]
    for r in rows:
        inside = r.get("ekf_fraction_inside_2sigma_pos_mean")
        inside_text = "--" if inside is None else f"{inside * 100:.0f} %"
        out.append(
            f"| {r['level']:g} | {r['runs']} "
            f"| {cell(r, 'ekf', 'position_rmse_m')} "
            f"| {cell(r, 'odometry', 'position_rmse_m')} "
            f"| {cell(r, 'ekf', 'heading_rmse_deg', '.2f')} "
            f"| {r.get('ekf_recovered', 0)} / {r['runs']} "
            f"| {inside_text} |")
    return "\n".join(out)


def write_report(cfg: dict, results: dict, out_dir: pathlib.Path) -> pathlib.Path:
    lines = ["# Phase 11 — benchmark sweeps", ""]
    lines.append(f"Every point is the mean over {len(cfg['seeds'])} seeds; "
                 f"`±` is the population standard deviation across them, not a "
                 f"standard error — three samples cannot support a confidence "
                 f"interval.")
    lines.append("")
    lines.append(f"Run length {cfg['duration_s']:g} s, spawn `{cfg['spawn']}`, "
                 f"open-loop route. These numbers are NOT directly comparable "
                 f"with Phase 10's 120 s runs: a dead-reckoning error grows "
                 f"with distance travelled.")
    lines.append("")
    for experiment in cfg["experiments"]:
        rows = results.get(experiment["id"], [])
        lines.append(f"## {experiment['title']}")
        lines.append("")
        lines.append(experiment["description"].strip())
        lines.append("")
        if not rows:
            lines.append("*not run*")
            lines.append("")
            continue
        lines.append(f"![{experiment['id']}](sweep_{experiment['id']}.png)")
        lines.append("")
        lines.append(markdown_table(experiment, rows))
        lines.append("")
    path = out_dir / "benchmark_report.md"
    path.write_text("\n".join(lines))
    return path


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--experiment", action="append", default=None,
                    help="Run only this experiment id; repeatable.")
    ap.add_argument("--out", default=None)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--aggregate-only", action="store_true",
                    help="Re-derive tables and plots from runs already on disk.")
    args = ap.parse_args(argv)

    cfg = load_config()
    experiments = cfg["experiments"]
    if args.experiment:
        wanted = set(args.experiment)
        experiments = [e for e in experiments if e["id"] in wanted]
        unknown = wanted - {e["id"] for e in cfg["experiments"]}
        if unknown:
            print(f"unknown experiment(s): {sorted(unknown)}", file=sys.stderr)
            return 2

    out_dir = pathlib.Path(args.out) if args.out else REPO_ROOT / "results" / "phase11"
    raw_dir = out_dir / "raw"
    raw_dir.mkdir(parents=True, exist_ok=True)

    total = sum(len(e["levels"]) for e in experiments) * len(cfg["seeds"])
    if args.dry_run:
        print(f"{total} runs, ~{total * (STARTUP_S + cfg['duration_s'] + 8) / 60:.0f} min")
        for e in experiments:
            print(f"  {e['id']:16s} {e['parameter']}: {e['levels']} "
                  f"x seeds {cfg['seeds']}")
        return 0

    if not args.aggregate_only:
        work = pathlib.Path(tempfile.mkdtemp(prefix="arb_benchmark_"))
        print(f"working directory {work}")
        done = 0
        started = time.time()
        try:
            for experiment in experiments:
                print(f"\n=== {experiment['title']} ({experiment['id']}) ===")
                for level in experiment["levels"]:
                    for seed in cfg["seeds"]:
                        done += 1
                        elapsed = time.time() - started
                        eta = (elapsed / done) * (total - done) / 60 if done else 0
                        print(f"  [{done}/{total}] level={level:g} seed={seed} "
                              f"(eta {eta:.0f} min)", flush=True)
                        run_one(experiment, level, seed, cfg, raw_dir, work)
        finally:
            shutil.rmtree(work, ignore_errors=True)

    results = {e["id"]: aggregate(raw_dir, e) for e in cfg["experiments"]}
    for experiment in cfg["experiments"]:
        path = plot_experiment(experiment, results[experiment["id"]], out_dir)
        if path:
            print(f"wrote {path}")
    report = write_report(cfg, results, out_dir)
    print(f"wrote {report}")
    with open(out_dir / "aggregate.json", "w") as fh:
        json.dump(results, fh, indent=2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
