#!/usr/bin/env python3
# Copyright (c) 2026 Emin Ayhan. MIT License.
"""Does predicting obstacle motion help? Measured against the same encounters.

The earlier three-arm sweep could not answer this, for two independent
reasons, and both are worth keeping in view:

1. The baseline arm was not a baseline. It passed
   `predict_obstacle_motion:=false` to a launch file that never declared the
   argument, so ros2 launch discarded it and every "blind" run predicted.
   Two arms of that sweep were one configuration sampled twice -- which does
   at least give an honest noise floor: 110 +- 36 s against 140 +- 56 s, for
   runs that differed in nothing at all.

2. Nothing controlled the encounter. The obstacle's phase was anchored to
   the instant its spawn call returned, which moved by seconds between runs,
   and whether the robot meets the obstacle at all depends on that phase.
   The same seed and the same binary produced both a clean 39.6 s arrival
   and a 180 s timeout with 0.39 m of overlap.

So this experiment fixes the phase deliberately and sweeps it. Each offset is
run twice -- once predicting, once not -- and the pairs are compared. A
paired design is what makes a dozen runs enough: the question is not "how
fast is the planner" but "at this same encounter, does prediction help?"
"""
from __future__ import annotations

import argparse
import json
import pathlib
import re
import statistics
import subprocess
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import run_navigation_benchmark as nav       # noqa: E402

REPO_ROOT = nav.REPO_ROOT
#: One full out-and-back of the obstacle, in simulated seconds.
PERIOD_S = 2.0 * 2.2 / 0.45


def run(phase: float, predict: bool, out_dir: pathlib.Path,
        work: pathlib.Path, seed: int) -> dict | None:
    arm = "predict" if predict else "blind"
    tag = f"phase{phase:04.1f}_{arm}"
    params = nav.sensor_params_with_seeds(seed, work / f"sensors{seed}.yaml")
    stack = nav.NavStack(params, work / f"{tag}.log", with_obstacle=True,
                         predict=predict, phase_offset_s=phase,
                         log_tracks=True)
    try:
        stack.up()
        r = subprocess.run(
            [sys.executable, "scripts/evaluate_navigation.py",
             "--mode", "goal", "--goal", nav.GOAL, "--duration", "180",
             "--controller", "dwa", "--tag", tag, "--out", str(out_dir)],
            cwd=REPO_ROOT, capture_output=True, text=True,
            timeout=nav.RUN_TIMEOUT_S)
        if r.returncode != 0:
            print(f"    FAILED rc={r.returncode}: {r.stderr.strip()[-200:]}")
            return None
    except subprocess.TimeoutExpired:
        print("    TIMED OUT")
        return None
    finally:
        stack.down()

    path = out_dir / f"metrics_goal_{tag}.json"
    if not path.exists():
        print(f"    no metrics at {path}")
        return None
    m = json.loads(path.read_text())
    m.update(phase_offset_s=phase, predict=predict, seed=seed)

    # How many movers the tracker reported, and what they were. A count on
    # its own cannot separate "six phantoms" from "one object split six ways".
    log = (work / f"{tag}.log").read_text() if (work / f"{tag}.log").exists() else ""
    counts = [int(c) for c in re.findall(r"moving obstacles (\d+)", log)]
    speeds = [float(s) for s in re.findall(r"v=(\d+\.\d+)", log)]
    m["tracked_mean"] = statistics.fmean(counts) if counts else None
    m["tracked_max"] = max(counts) if counts else None
    m["track_speed_median"] = statistics.median(speeds) if speeds else None
    path.write_text(json.dumps(m, indent=2))
    return m


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--phases", type=int, default=6,
                    help="evenly spaced offsets across one full sweep")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", default=None)
    args = ap.parse_args(argv)

    out_dir = (pathlib.Path(args.out) if args.out
               else REPO_ROOT / "results" / "phase12")
    out_dir.mkdir(parents=True, exist_ok=True)
    work = out_dir / "work"
    work.mkdir(exist_ok=True)

    offsets = [round(i * PERIOD_S / args.phases, 1) for i in range(args.phases)]
    plan = [(p, predict) for p in offsets for predict in (False, True)]
    print(f"{len(plan)} runs, offsets {offsets} s (period {PERIOD_S:.2f} s)",
          flush=True)

    rows: list[dict] = []
    for i, (phase, predict) in enumerate(plan, start=1):
        print(f"  [{i}/{len(plan)}] phase={phase}s "
              f"{'predict' if predict else 'blind'}", flush=True)
        m = run(phase, predict, out_dir, work, args.seed)
        if m:
            rows.append(m)

    report(rows, out_dir)
    return 0


def report(rows: list[dict], out_dir: pathlib.Path) -> None:
    if not rows:
        print("no runs completed")
        return
    by: dict[tuple[float, bool], dict] = {
        (r["phase_offset_s"], r["predict"]): r for r in rows}
    phases = sorted({r["phase_offset_s"] for r in rows})

    lines = ["| phase [s] | arm | arrived | time [s] | clearance [m] "
             "| movers (mean/max) |",
             "|---|---|---|---|---|---|"]
    for p in phases:
        for predict in (False, True):
            r = by.get((p, predict))
            if not r:
                continue
            lines.append(
                f"| {p} | {'predict' if predict else 'blind'} "
                f"| {'yes' if r.get('arrived') else 'no'} "
                f"| {r.get('time_s', 0):.1f} "
                f"| {r.get('min_obstacle_clearance_m', float('nan')):.3f} "
                f"| {r.get('tracked_mean') or 0:.1f} / {r.get('tracked_max') or 0} |")

    # Clearance from a run that never travelled is not a safety result: a
    # robot stuck at the start scores a perfect distance from an obstacle it
    # never approached. One run timed out 15.8 m from its goal and recorded
    # 6.03 m of clearance that way, which dragged an arm's mean to +1.39 m
    # and made a frozen planner look like the safest one measured.
    stuck = [r for r in rows if not r.get("arrived")
             and r.get("final_error_m", 0.0) > 2.0]
    if stuck:
        lines.append("")
        lines.append(f"**{len(stuck)} run(s) never travelled** (over 2 m from "
                     f"the goal at the end). Their clearance measures how far "
                     f"away they stopped, not how safely they passed, and is "
                     f"excluded from the means below.")
    scored = [r for r in rows if r not in stuck]

    paired = [(by[(p, False)], by[(p, True)]) for p in phases
              if (p, False) in by and (p, True) in by
              and by[(p, False)] in scored and by[(p, True)] in scored]
    lines.append("")
    lines.append(f"**{len(paired)} matched pairs** that both travelled.")
    if paired:
        better = sum(1 for b, pr in paired
                     if (pr.get("min_obstacle_clearance_m") or -9)
                     > (b.get("min_obstacle_clearance_m") or -9))
        arrived_b = sum(1 for b, _ in paired if b.get("arrived"))
        arrived_p = sum(1 for _, pr in paired if pr.get("arrived"))
        lines.append(f"- prediction gave more clearance in {better} of "
                     f"{len(paired)} encounters")
        lines.append(f"- arrived: blind {arrived_b}/{len(paired)}, "
                     f"predict {arrived_p}/{len(paired)}")
    text = "\n".join(lines)
    (out_dir / "obstacle_phase_sweep.md").write_text(text + "\n")
    print()
    print(text)


if __name__ == "__main__":
    raise SystemExit(main())
