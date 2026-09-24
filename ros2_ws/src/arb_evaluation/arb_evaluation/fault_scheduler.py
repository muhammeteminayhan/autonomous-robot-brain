"""Play a fault scenario against a running stack, and say so on a topic.

    ros2 run arb_evaluation fault_scheduler --ros-args \
        -p scenario:=configs/scenarios/case_c_encoder_drift.yaml

The timeline logic lives in `scenario.py` and is pure, tested and ROS-free.
This node is only the part that cannot be: reading the clock, calling parameter
services, and publishing what it did.

WHY SIM TIME AND NOT WALL TIME
Gazebo's real_time_factor is uncapped so batch experiments run fast, and this
world runs well above 1.0. A scheduler on wall time would apply a fault
declared for t = 20 s at whatever simulated instant happened to coincide, the
robot would be somewhere different every run, and Case A would not be a
baseline for anything. Every time in a scenario file is SIMULATED seconds since
the scheduler saw its first clock message.

WHY IT POLLS INSTEAD OF SCHEDULING TIMERS
A precomputed list of edges is tempting and wrong here: an encoder bias ramps
continuously over five seconds, so there is no finite set of instants at which
the correct knob value changes. Polling at a fixed rate makes the ramp's
resolution explicit (`rate_hz`) rather than hidden.

WHY IT PUBLISHES FaultState
Every plot and log downstream needs to be annotated with what was wrong at the
time, and only this node knows the whole truth -- the sensor nodes know their
current knob values but not the scenario, the fault type, or how long ago the
onset was. It is instrumentation: arb_msgs/FaultState.msg says no estimator may
consume it, and tests/integration/test_ground_truth_isolation.py enforces the
same rule for ground truth.
"""

from __future__ import annotations

import pathlib
import sys

from arb_evaluation import scenario as sc
from arb_msgs.msg import FaultState
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
import yaml

#: Latched, so a recorder or a plotter that attaches mid-run still learns the
#: scenario id and the current fault state instead of waiting for a change.
STATE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


def _parameter_msg(name: str, value) -> Parameter:
    p = Parameter()
    p.name = name
    p.value = ParameterValue()
    if isinstance(value, bool):
        p.value.type = ParameterType.PARAMETER_BOOL
        p.value.bool_value = value
    else:
        p.value.type = ParameterType.PARAMETER_DOUBLE
        p.value.double_value = float(value)
    return p


class FaultScheduler(Node):
    """Applies a scenario's knob timeline to the three sensor preprocessors."""

    def __init__(self) -> None:
        super().__init__("fault_scheduler")

        path = pathlib.Path(
            self.declare_parameter("scenario", "").get_parameter_value().string_value)
        if not str(path):
            raise SystemExit("the 'scenario' parameter is required")
        if not path.is_absolute():
            path = pathlib.Path.cwd() / path
        if not path.exists():
            raise SystemExit(f"scenario file not found: {path}")

        self.scenario = yaml.safe_load(path.read_text())
        # Validated BEFORE anything is applied. A typo in a fault type would
        # otherwise apply nothing, the run would come out looking nominal, and
        # the row in the results table would be a lie rather than an error.
        sc.validate(self.scenario)
        self.spec = self.scenario["scenario"]

        self.rate_hz = self.declare_parameter("rate_hz", 20.0).value
        # A run normally ends when the scenario does. Set false to leave the
        # scheduler up -- useful when driving it by hand.
        self.shutdown_at_end = self.declare_parameter("shutdown_at_end", True).value

        # NOT `self._clients`: rclpy.Node keeps its own list of service clients
        # in an attribute of that name, so assigning a dict to it makes the
        # executor iterate strings ("'str' object has no attribute
        # '_executor_event'") and destroy_node() index a dict by 0. The
        # traceback names rclpy internals and never mentions this file. Second
        # time in this project -- the same shadowing hit arb_sensors in Phase 2.
        self._param_clients = {
            node: self.create_client(SetParameters, f"/{node}/set_parameters")
            for node in sc.NOMINAL
        }
        self.state_pub = self.create_publisher(
            FaultState, "/arb/fault_state", STATE_QOS)

        self.t0: float | None = None
        self.applied = sc.nominal_knobs()
        self.pending: list = []
        self.finished = False
        self.sets_sent = 0
        self.sets_failed = 0

        # rclpy's logger takes ONE string -- there is no printf-style
        # formatting as there is in rclcpp, and passing extra positional
        # arguments raises TypeError at the call site rather than being
        # ignored. f-strings throughout.
        self.get_logger().info(
            f"scenario '{self.spec['id']}': {float(self.spec['duration_s']):.0f} s, "
            f"{len(self.spec['faults'])} fault(s) -- "
            f"{self.spec.get('description', '')}")
        for w in sc.windows(self.scenario):
            self.get_logger().info(
                f"  {w['type']:<16} {w['onset_s']:6.1f} -> {w['end_s']:6.1f} s  "
                f"{w['params']}")

        self.timer = self.create_timer(1.0 / float(self.rate_hz), self._tick)

    # ------------------------------------------------------------------ time --
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def elapsed(self) -> float | None:
        """Return simulated seconds since the first clock message, or None.

        None means no clock has arrived yet. The distinction matters: zero is a
        legitimate reading from an unconnected clock, so the scheduler must not
        confuse "no clock yet" with "t = 0" and start applying faults against a
        simulator that has not begun stepping.
        """
        now = self._now()
        if now <= 0.0:
            return None
        if self.t0 is None:
            self.t0 = now
            self.get_logger().info("clock acquired; scenario starts now")
        return now - self.t0

    # ------------------------------------------------------------------ apply --
    def _apply(self, knobs: sc.Knobs) -> None:
        delta = knobs.changed_from(self.applied)
        if not delta:
            return
        for node, params in delta.items():
            client = self._param_clients[node]
            if not client.service_is_ready():
                # Not an error worth failing on: the preprocessors may not be up
                # yet. The knob stays undelivered and the next tick retries,
                # because `self.applied` is only updated on success.
                self.get_logger().warn(
                    f"{node}/set_parameters not ready; retrying",
                    throttle_duration_sec=5.0)
                return
            request = SetParameters.Request()
            request.parameters = [_parameter_msg(n, v) for n, v in params.items()]
            future = client.call_async(request)
            future.add_done_callback(
                lambda f, n=node, p=dict(params): self._on_set_done(f, n, p))
            self.sets_sent += 1
            summary = ", ".join(
                f"{k.removeprefix('fault.')}={v}" for k, v in params.items())
            self.get_logger().info(f"{node} <- {summary}")
        # Recorded as applied optimistically; a rejected set is logged loudly by
        # the callback below, which is the only case where that matters and is
        # a bug in the scenario rather than something to paper over at runtime.
        self.applied = sc.Knobs(
            values={n: dict(p) for n, p in knobs.values.items()},
            active=list(knobs.active))

    def _on_set_done(self, future, node: str, params: dict) -> None:
        try:
            results = future.result().results
        except Exception as exc:                     # noqa: BLE001
            self.sets_failed += 1
            self.get_logger().error(f"{node} set_parameters failed: {exc}")
            return
        for name, result in zip(params, results):
            if not result.successful:
                self.sets_failed += 1
                self.get_logger().error(
                    f"{node} rejected {name}: {result.reason}")

    # ------------------------------------------------------------------ publish --
    def _publish(self, knobs: sc.Knobs, t: float) -> None:
        msg = FaultState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.scenario_id = self.spec["id"]
        codes = sorted({sc.FAULT_CODES[a] for a in knobs.active})
        if len(codes) > 1:
            # Case F's own label, so a plot legend does not have to enumerate.
            codes = [FaultState.FAULT_MULTI_DEGRADATION] + codes
        msg.active_faults = codes or [FaultState.FAULT_NONE]

        imu = knobs.values["imu_preprocessor"]
        enc = knobs.values["encoder_preprocessor"]
        lidar = knobs.values["lidar_preprocessor"]
        msg.imu_degraded = bool(
            imu["fault.noise_scale"] != 1.0 or imu["fault.accel_noise_scale"] != 1.0
            or imu["fault.gyro_bias_offset"] != 0.0 or imu["fault.outage"])
        msg.encoder_degraded = bool(
            enc["fault.left_bias_rad_s"] or enc["fault.right_bias_rad_s"]
            or enc["fault.noise_scale"] != 1.0 or enc["fault.outage"])
        msg.lidar_degraded = bool(
            lidar["fault.dropout_ratio"] or lidar["fault.noise_scale"] != 1.0
            or lidar["fault.outage"])
        msg.imu_noise_scale = float(imu["fault.noise_scale"])
        # The larger magnitude of the two wheels: the message carries one field
        # and a bias on either wheel curves the robot the same way.
        msg.encoder_bias_rad_s = max(
            (float(enc["fault.left_bias_rad_s"]), float(enc["fault.right_bias_rad_s"])),
            key=abs)
        msg.lidar_dropout_ratio = float(lidar["fault.dropout_ratio"])

        onsets = [w["onset_s"] for w in sc.windows(self.scenario)
                  if w["onset_s"] <= t < w["end_s"]]
        msg.elapsed_since_onset_s = float(t - min(onsets)) if onsets else 0.0
        self.state_pub.publish(msg)

    # ------------------------------------------------------------------ tick --
    def _tick(self) -> None:
        if self.finished:
            return
        t = self.elapsed()
        if t is None:
            self.get_logger().warn(
                "waiting for /clock -- is the simulator running with "
                "use_sim_time?", throttle_duration_sec=5.0)
            return

        duration = float(self.spec["duration_s"])
        if t >= duration:
            # Clear every knob, including ones no fault in this scenario used:
            # the scheduler's job at the end is a known-good robot, and a knob
            # it forgot is contamination carried into the next run.
            self._apply(sc.nominal_knobs())
            self._publish(sc.nominal_knobs(), t)
            self.get_logger().info(
                f"scenario '{self.spec['id']}' complete at t={t:.1f} s: "
                f"{self.sets_sent} parameter set(s), {self.sets_failed} rejected")
            self.finished = True
            if self.shutdown_at_end:
                raise SystemExit(0)
            return

        knobs = sc.knobs_at(self.scenario, t)
        self._apply(knobs)
        self._publish(knobs, t)


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    node = None
    try:
        node = FaultScheduler()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node is not None:
            # Hand the robot back healthy even on Ctrl-C. A scenario abandoned
            # halfway leaves a 0.05 rad/s encoder bias set, and the next run --
            # a nominal one, probably -- inherits it silently.
            try:
                node._apply(sc.nominal_knobs())
                for _ in range(20):
                    rclpy.spin_once(node, timeout_sec=0.05)
            except Exception:                        # noqa: BLE001
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
