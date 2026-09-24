"""Scenario timelines: what every fault knob should hold at a given instant.

This module is DELIBERATELY FREE OF ROS. `knobs_at` is a pure function of a
scenario dictionary and a time, which is what makes the timeline testable at
all: the alternative is asserting against a running simulator, where a wrong
ramp and a slow parameter service look identical.

A scenario file (configs/scenarios/*.yaml) is a list of faults, each with an
onset, a duration and its own parameters. Faults OVERLAP -- that is the whole
point of Case F -- so the knob set at time t is the composition of every fault
active at t, not the first one found.

See docs/fault-injection.md for what each case is testing.
"""

from __future__ import annotations

import dataclasses
import math

#: Every fault knob the sensor preprocessors expose, at its HEALTHY value.
#:
#: Written out rather than discovered, because the point of a scheduler is to
#: return the robot to a known-good state when a fault ends. A knob it does not
#: know about is a knob it cannot clear, and the contamination would carry into
#: the next scenario silently. tests/integration/test_config_consistency.py
#: checks this list against the checked-in parameter files.
NOMINAL: dict[str, dict[str, float | bool]] = {
    "imu_preprocessor": {
        "fault.noise_scale": 1.0,
        "fault.accel_noise_scale": 1.0,
        "fault.gyro_bias_offset": 0.0,
        "fault.outage": False,
    },
    "encoder_preprocessor": {
        "fault.left_bias_rad_s": 0.0,
        "fault.right_bias_rad_s": 0.0,
        "fault.noise_scale": 1.0,
        "fault.outage": False,
    },
    "lidar_preprocessor": {
        "fault.dropout_ratio": 0.0,
        "fault.noise_scale": 1.0,
        "fault.burst_mode": False,
        "fault.burst_active": False,
        "fault.outage": False,
    },
}

#: Fault type -> the FaultState constant that labels it, matching
#: arb_msgs/msg/FaultState.msg.
FAULT_CODES = {
    "imu_high_noise": 1,
    "encoder_drift": 2,
    "lidar_dropout": 3,
    "sensor_outage": 4,
}

KNOWN_FAULTS = frozenset(FAULT_CODES)


@dataclasses.dataclass
class Knobs:
    """A complete parameter set for the three preprocessors."""

    values: dict[str, dict[str, float | bool]]
    active: list[str] = dataclasses.field(default_factory=list)

    def get(self, node: str, name: str):
        return self.values[node][name]

    def changed_from(self, other: "Knobs") -> dict[str, dict[str, float | bool]]:
        """Only the knobs that differ, so the scheduler can avoid resending.

        A parameter service call per knob per cycle is 13 calls at 20 Hz, and
        every one of them logs a line in the node it hits. Sending only changes
        keeps the scenario's timeline visible in the log instead of buried.
        """
        delta: dict[str, dict[str, float | bool]] = {}
        for node, params in self.values.items():
            for name, value in params.items():
                if other.values[node][name] != value:
                    delta.setdefault(node, {})[name] = value
        return delta


def nominal_knobs() -> Knobs:
    return Knobs(values={n: dict(p) for n, p in NOMINAL.items()})


def validate(scenario: dict) -> None:
    """Reject a malformed scenario before it runs, not halfway through.

    A typo in a fault type is the failure worth catching here: the scheduler
    would otherwise apply nothing, the run would come out looking nominal, and
    the resulting row in the results table would be a lie rather than an error.
    """
    if "scenario" not in scenario:
        raise ValueError("file has no top-level 'scenario' key")
    s = scenario["scenario"]
    for key in ("id", "duration_s", "faults"):
        if key not in s:
            raise ValueError(f"scenario is missing '{key}'")
    if not isinstance(s["faults"], list):
        raise ValueError("scenario.faults must be a list")

    for i, fault in enumerate(s["faults"]):
        where = f"scenario.faults[{i}]"
        for key in ("type", "onset_s", "duration_s"):
            if key not in fault:
                raise ValueError(f"{where} is missing '{key}'")
        if fault["type"] not in KNOWN_FAULTS:
            raise ValueError(
                f"{where}.type = '{fault['type']}' is not one of "
                f"{sorted(KNOWN_FAULTS)}")
        if fault["onset_s"] < 0.0:
            raise ValueError(f"{where}.onset_s is negative")
        if fault["duration_s"] <= 0.0:
            raise ValueError(f"{where}.duration_s must be positive")

        params = fault.get("params") or {}
        if fault["type"] == "sensor_outage":
            sensor = params.get("sensor")
            if sensor not in ("imu", "lidar", "encoder"):
                raise ValueError(
                    f"{where}.params.sensor = '{sensor}' must be imu, lidar or "
                    f"encoder")
        if fault["type"] == "encoder_drift":
            wheel = params.get("wheel")
            if wheel not in ("left", "right"):
                raise ValueError(
                    f"{where}.params.wheel = '{wheel}' must be left or right")
        if fault["type"] == "lidar_dropout":
            ratio = params.get("dropout_ratio", 0.0)
            if not 0.0 <= ratio <= 1.0:
                raise ValueError(f"{where}.params.dropout_ratio must be in [0, 1]")


def _ramp(elapsed: float, ramp_time_s: float) -> float:
    """Return the fraction of a fault's magnitude reached after `elapsed`.

    A fault that steps instantaneously is easy to detect and rarely how hardware
    fails: a loosening wheel coupling or a warming MEMS element degrades over
    seconds. The ramp is what makes Case C hard -- there is no single instant at
    which something obviously changed.
    """
    if ramp_time_s <= 0.0:
        return 1.0
    return min(1.0, max(0.0, elapsed / ramp_time_s))


def knobs_at(scenario: dict, t: float) -> Knobs:
    """Return the complete knob set at scenario time `t`, seconds from start.

    Composes every active fault. Where two faults touch the same knob the
    stronger wins -- a multiplier takes the larger factor, a bias the larger
    magnitude, an outage wins outright -- rather than the later one in the file
    overwriting the earlier. File order is not a physical property and must not
    decide what the robot experiences.
    """
    knobs = nominal_knobs()
    s = scenario["scenario"]

    # Which knobs a fault has already written. The "stronger wins" rule below
    # resolves TWO OVERLAPPING FAULTS; it must not be applied between a fault
    # and the nominal baseline.
    #
    # Composing against nominal silently clamps any fault that asks for LESS
    # than nominal: `max(1.0, 0.5)` is 1.0, so a sweep point at half the
    # datasheet noise came out identical to the healthy robot and the
    # low-noise end of Phase 11's gyro experiment measured nothing. Found by
    # generating the sweep's scenarios and inspecting them, before running
    # them -- the run itself would just have produced a flat line.
    written: set[tuple[str, str]] = set()

    def put(node: str, key: str, value, stronger) -> None:
        """Write a knob, or compose with what another fault already wrote."""
        target = knobs.values[node]
        if (node, key) not in written:
            written.add((node, key))
            target[key] = value
        else:
            target[key] = stronger(target[key], value)

    for fault in s["faults"]:
        onset = float(fault["onset_s"])
        end = onset + float(fault["duration_s"])
        if not (onset <= t < end):
            continue
        elapsed = t - onset
        params = fault.get("params") or {}
        kind = fault["type"]
        knobs.active.append(kind)

        if kind == "imu_high_noise":
            scale = float(params.get("noise_scale", 1.0))
            put("imu_preprocessor", "fault.noise_scale", scale, max)
            if params.get("affect_accelerometer", False):
                put("imu_preprocessor", "fault.accel_noise_scale", scale, max)

        elif kind == "encoder_drift":
            bias = float(params.get("bias_rad_s", 0.0)) * _ramp(
                elapsed, float(params.get("ramp_time_s", 0.0)))
            put("encoder_preprocessor", f"fault.{params['wheel']}_bias_rad_s",
                bias, lambda a, b: a if abs(a) >= abs(b) else b)

        elif kind == "lidar_dropout":
            ratio = float(params.get("dropout_ratio", 0.0))
            put("lidar_preprocessor", "fault.dropout_ratio", ratio, max)
            lidar = knobs.values["lidar_preprocessor"]
            if params.get("burst_mode", False):
                lidar["fault.burst_mode"] = True
                # Bursts are a square wave over the fault's own elapsed time,
                # so the pattern is reproducible from the scenario file alone
                # and does not depend on when the run happened to start.
                on = float(params.get("burst_on_s", 1.0))
                off = float(params.get("burst_off_s", 1.0))
                period = on + off
                if period > 0.0:
                    lidar["fault.burst_active"] = (
                        math.fmod(elapsed, period) < on)

        elif kind == "sensor_outage":
            node = {
                "imu": "imu_preprocessor",
                "lidar": "lidar_preprocessor",
                "encoder": "encoder_preprocessor",
            }[params["sensor"]]
            knobs.values[node]["fault.outage"] = True

    return knobs


def timeline(scenario: dict) -> list[float]:
    """Return the times at which the knob set can change, for annotating logs.

    Onsets and ends, plus every burst edge. Used by the report to draw the fault
    bands on a plot; the scheduler itself polls, because a scenario is minutes
    long and a missed edge would be invisible in a precomputed list.
    """
    edges = {0.0, float(scenario["scenario"]["duration_s"])}
    for fault in scenario["scenario"]["faults"]:
        onset = float(fault["onset_s"])
        duration = float(fault["duration_s"])
        edges.add(onset)
        edges.add(onset + duration)
        params = fault.get("params") or {}
        if fault["type"] == "lidar_dropout" and params.get("burst_mode", False):
            on = float(params.get("burst_on_s", 1.0))
            off = float(params.get("burst_off_s", 1.0))
            period = on + off
            if period > 0.0:
                k = 0
                while k * period < duration:
                    edges.add(onset + k * period)
                    edges.add(min(onset + k * period + on, onset + duration))
                    k += 1
    return sorted(e for e in edges if e >= 0.0)


def windows(scenario: dict) -> list[dict]:
    """Each fault as a {type, onset, end, params} record, for plots and tables."""
    out = []
    for fault in scenario["scenario"]["faults"]:
        out.append({
            "type": fault["type"],
            "onset_s": float(fault["onset_s"]),
            "end_s": float(fault["onset_s"]) + float(fault["duration_s"]),
            "params": fault.get("params") or {},
        })
    return out
