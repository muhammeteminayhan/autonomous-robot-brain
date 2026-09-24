"""Tests for the scenario timeline.

`knobs_at` is a pure function, which is the only reason these tests can exist:
asserted against a running simulator, a wrong ramp and a slow parameter service
look exactly the same.

Every checked-in scenario in configs/scenarios/ is also run through the
validator and the timeline here, so a typo in a fault type fails the test suite
rather than producing a run that merely looks nominal.
"""

import math
import pathlib

from arb_evaluation import scenario as sc
import pytest
import yaml

SCENARIO_DIR = (pathlib.Path(__file__).resolve().parents[4]
                / "configs" / "scenarios")


def load(name: str) -> dict:
    return yaml.safe_load((SCENARIO_DIR / name).read_text())


def all_scenarios():
    return sorted(SCENARIO_DIR.glob("*.yaml"))


# ------------------------------------------------------------------ nominal --
def test_nominal_knobs_are_all_healthy():
    knobs = sc.nominal_knobs()
    for node, params in knobs.values.items():
        for name, value in params.items():
            expected = 1.0 if name.endswith("_scale") else (
                False if isinstance(value, bool) else 0.0)
            assert value == expected, f"{node}.{name}"


def test_nominal_knobs_are_independent_copies():
    """Check that each call returns a fresh dict.

    The scheduler mutates what knobs_at returns, so a shared dict would make
    every later call inherit the previous scenario's faults.
    """
    a = sc.nominal_knobs()
    a.values["imu_preprocessor"]["fault.outage"] = True
    b = sc.nominal_knobs()
    assert b.values["imu_preprocessor"]["fault.outage"] is False


def test_case_a_is_nominal_at_every_instant():
    s = load("case_a_nominal.yaml")
    for t in (0.0, 1.0, 60.0, 119.9):
        assert sc.knobs_at(s, t).values == sc.nominal_knobs().values
        assert sc.knobs_at(s, t).active == []


# --------------------------------------------------------------- validation --
@pytest.mark.parametrize("path", all_scenarios(), ids=lambda p: p.stem)
def test_every_checked_in_scenario_validates(path):
    sc.validate(yaml.safe_load(path.read_text()))


@pytest.mark.parametrize("path", all_scenarios(), ids=lambda p: p.stem)
def test_every_checked_in_scenario_returns_to_nominal_at_its_end(path):
    """Check that no knob is left set at the scenario's declared end.

    A scenario that leaves one set would contaminate whatever ran next. The
    check is at the declared duration, so a fault whose window extends past the
    end of the scenario fails here rather than silently outliving it.
    """
    s = yaml.safe_load(path.read_text())
    end = float(s["scenario"]["duration_s"])
    assert sc.knobs_at(s, end).values == sc.nominal_knobs().values


@pytest.mark.parametrize("bad, message", [
    ({"scenario": {}}, "missing 'id'"),
    ({"nope": {}}, "no top-level"),
    ({"scenario": {"id": "x", "duration_s": 1.0, "faults": {}}}, "must be a list"),
    ({"scenario": {"id": "x", "duration_s": 1.0, "faults": [
        {"type": "typo_fault", "onset_s": 0.0, "duration_s": 1.0}]}}, "not one of"),
    ({"scenario": {"id": "x", "duration_s": 1.0, "faults": [
        {"type": "sensor_outage", "onset_s": 0.0, "duration_s": 1.0,
         "params": {"sensor": "radar"}}]}}, "must be imu"),
    ({"scenario": {"id": "x", "duration_s": 1.0, "faults": [
        {"type": "encoder_drift", "onset_s": 0.0, "duration_s": 1.0,
         "params": {"wheel": "middle"}}]}}, "must be left or right"),
    ({"scenario": {"id": "x", "duration_s": 1.0, "faults": [
        {"type": "lidar_dropout", "onset_s": 0.0, "duration_s": 1.0,
         "params": {"dropout_ratio": 1.5}}]}}, "must be in \\[0, 1\\]"),
    ({"scenario": {"id": "x", "duration_s": 1.0, "faults": [
        {"type": "imu_high_noise", "onset_s": 0.0, "duration_s": 0.0}]}},
     "must be positive"),
])
def test_validate_rejects_malformed_scenarios(bad, message):
    with pytest.raises(ValueError, match=message):
        sc.validate(bad)


# ------------------------------------------------------------- case B, IMU ---
def test_imu_noise_applies_only_inside_its_window():
    s = load("case_b_imu_high_noise.yaml")
    assert sc.knobs_at(s, 19.9).get("imu_preprocessor", "fault.noise_scale") == 1.0
    assert sc.knobs_at(s, 20.0).get("imu_preprocessor", "fault.noise_scale") == 10.0
    assert sc.knobs_at(s, 94.9).get("imu_preprocessor", "fault.noise_scale") == 10.0
    # The fault clears 25 s before the scenario ends, so recovery has somewhere
    # to be measured; see the note at the top of the scenario file.
    assert sc.knobs_at(s, 95.0).get("imu_preprocessor", "fault.noise_scale") == 1.0
    assert sc.knobs_at(s, 119.9).get("imu_preprocessor", "fault.noise_scale") == 1.0


def test_case_b_degrades_the_accelerometer_and_case_f_does_not():
    """Check that `affect_accelerometer` is a setting, not a comment.

    It was the latter while a single knob scaled both IMU channels.
    """
    b = load("case_b_imu_high_noise.yaml")
    assert b["scenario"]["faults"][0]["params"]["affect_accelerometer"] is True
    assert sc.knobs_at(b, 50.0).get(
        "imu_preprocessor", "fault.accel_noise_scale") == 10.0

    f = load("case_f_multi_degradation.yaml")
    # The IMU fault in Case F runs 40-100 s and declares gyro only.
    knobs = sc.knobs_at(f, 50.0)
    assert knobs.get("imu_preprocessor", "fault.noise_scale") == 5.0
    assert knobs.get("imu_preprocessor", "fault.accel_noise_scale") == 1.0


# ------------------------------------------------------- case C, encoder -----
def test_encoder_bias_ramps_linearly_rather_than_stepping():
    s = load("case_c_encoder_drift.yaml")   # onset 15 s, ramp 5 s, 0.05 rad/s
    key = "fault.right_bias_rad_s"
    assert sc.knobs_at(s, 14.9).get("encoder_preprocessor", key) == 0.0
    assert sc.knobs_at(s, 15.0).get("encoder_preprocessor", key) == 0.0
    assert sc.knobs_at(s, 17.5).get(
        "encoder_preprocessor", key) == pytest.approx(0.025)
    assert sc.knobs_at(s, 20.0).get(
        "encoder_preprocessor", key) == pytest.approx(0.05)
    assert sc.knobs_at(s, 90.0).get(
        "encoder_preprocessor", key) == pytest.approx(0.05)
    # The ramp is monotonic and never overshoots the declared magnitude.
    previous = -1.0
    for i in range(0, 60):
        v = sc.knobs_at(s, 15.0 + i * 0.1).get("encoder_preprocessor", key)
        assert v >= previous - 1e-12
        assert v <= 0.05 + 1e-12
        previous = v


def test_encoder_bias_hits_the_correct_wheel():
    c = load("case_c_encoder_drift.yaml")       # right
    k = sc.knobs_at(c, 30.0)
    assert k.get("encoder_preprocessor", "fault.right_bias_rad_s") != 0.0
    assert k.get("encoder_preprocessor", "fault.left_bias_rad_s") == 0.0

    f = load("case_f_multi_degradation.yaml")   # left
    k = sc.knobs_at(f, 30.0)
    assert k.get("encoder_preprocessor", "fault.left_bias_rad_s") != 0.0
    assert k.get("encoder_preprocessor", "fault.right_bias_rad_s") == 0.0


def test_a_zero_ramp_time_is_an_instant_step():
    s = {"scenario": {"id": "x", "duration_s": 10.0, "faults": [
        {"type": "encoder_drift", "onset_s": 1.0, "duration_s": 5.0,
         "params": {"wheel": "left", "bias_rad_s": 0.1, "ramp_time_s": 0.0}}]}}
    assert sc.knobs_at(s, 1.0).get(
        "encoder_preprocessor", "fault.left_bias_rad_s") == pytest.approx(0.1)


# --------------------------------------------------------- case D, LiDAR -----
def test_lidar_bursts_alternate_on_the_declared_period():
    s = load("case_d_lidar_dropout.yaml")   # onset 25, burst 3 s on / 2 s off

    def active(t):
        return sc.knobs_at(s, t).get("lidar_preprocessor", "fault.burst_active")

    assert sc.knobs_at(s, 30.0).get(
        "lidar_preprocessor", "fault.burst_mode") is True
    assert active(25.0) is True       # 0.0 into the 5 s period
    assert active(27.9) is True       # 2.9 -> still in the 3 s on-phase
    assert active(28.1) is False      # 3.1 -> off-phase
    assert active(29.9) is False      # 4.9 -> still off
    assert active(30.1) is True       # 5.1 -> next period's on-phase

    # The pattern depends on elapsed time within the fault, so it is
    # reproducible from the file alone rather than on when the run started.
    for k in range(10):
        assert active(25.0 + k * 5.0 + 0.5) is True
        assert active(25.0 + k * 5.0 + 3.5) is False


def test_bursts_are_phased_from_the_onset_not_from_the_scenario_start():
    """The discriminating case, and Case D cannot provide it.

    Case D's onset is 25 s and its burst period is 5 s, so 25 is an exact
    multiple of the period and `fmod(t, period)` and `fmod(t - onset, period)`
    agree everywhere. A mutation swapping one for the other passed the whole
    suite. An onset that is NOT a multiple of the period separates them, and a
    real fault's onset is a coincidence away from being one.
    """
    s = {"scenario": {"id": "x", "duration_s": 40.0, "faults": [
        {"type": "lidar_dropout", "onset_s": 12.0, "duration_s": 20.0,
         "params": {"dropout_ratio": 0.5, "burst_mode": True,
                    "burst_on_s": 3.0, "burst_off_s": 2.0}}]}}

    def active(t):
        return sc.knobs_at(s, t).get("lidar_preprocessor", "fault.burst_active")

    # Phased from the onset: on for [12, 15), off for [15, 17), on again at 17.
    assert active(12.1) is True
    assert active(14.9) is True
    assert active(15.1) is False
    assert active(16.9) is False
    assert active(17.1) is True
    # Phased from the scenario start it would be off at 12.1 (fmod(12.1,5)=2.1
    # is inside the on-phase, so that instant agrees) and ON at 15.1
    # (fmod(15.1,5)=0.1) -- which is the assertion above that separates them.


def test_dropout_ratio_is_held_while_the_fault_is_active():
    s = load("case_d_lidar_dropout.yaml")
    assert sc.knobs_at(s, 24.9).get(
        "lidar_preprocessor", "fault.dropout_ratio") == 0.0
    assert sc.knobs_at(s, 60.0).get(
        "lidar_preprocessor", "fault.dropout_ratio") == pytest.approx(0.60)


def test_non_burst_dropout_leaves_burst_mode_off():
    s = load("case_f_multi_degradation.yaml")   # lidar_dropout, burst_mode false
    k = sc.knobs_at(s, 60.0)
    assert k.get("lidar_preprocessor", "fault.burst_mode") is False
    assert k.get("lidar_preprocessor", "fault.burst_active") is False
    assert k.get("lidar_preprocessor", "fault.dropout_ratio") == pytest.approx(0.30)


# --------------------------------------------------------- case E, outage ----
def test_outages_hit_one_sensor_each_and_then_clear():
    s = load("case_e_sensor_outage.yaml")   # imu 30-40, lidar 70-80
    assert sc.knobs_at(s, 29.9).get("imu_preprocessor", "fault.outage") is False
    assert sc.knobs_at(s, 35.0).get("imu_preprocessor", "fault.outage") is True
    assert sc.knobs_at(s, 35.0).get("lidar_preprocessor", "fault.outage") is False
    assert sc.knobs_at(s, 40.1).get("imu_preprocessor", "fault.outage") is False
    assert sc.knobs_at(s, 75.0).get("lidar_preprocessor", "fault.outage") is True
    assert sc.knobs_at(s, 75.0).get("imu_preprocessor", "fault.outage") is False
    assert sc.knobs_at(s, 80.1).get("lidar_preprocessor", "fault.outage") is False
    # Never both at once in this scenario -- the point is to isolate each.
    for i in range(1200):
        k = sc.knobs_at(s, i * 0.1)
        assert not (k.get("imu_preprocessor", "fault.outage")
                    and k.get("lidar_preprocessor", "fault.outage"))


# ------------------------------------------------------ case F, composition --
def test_overlapping_faults_compose_rather_than_overwrite():
    s = load("case_f_multi_degradation.yaml")
    # At 60 s all three are active: encoder 20-120, imu 40-100, lidar 50-120.
    k = sc.knobs_at(s, 60.0)
    assert k.get("encoder_preprocessor", "fault.left_bias_rad_s") == pytest.approx(0.04)
    assert k.get("imu_preprocessor", "fault.noise_scale") == 5.0
    assert k.get("lidar_preprocessor", "fault.dropout_ratio") == pytest.approx(0.30)
    assert sorted(set(k.active)) == [
        "encoder_drift", "imu_high_noise", "lidar_dropout"]


def test_the_stronger_of_two_faults_on_one_knob_wins_not_the_later_one():
    """Check that the stronger fault wins when two touch the same knob.

    File order is not a physical property and must not decide what the robot
    experiences, so a weak fault listed after a strong one must not undo it.
    """
    strong_first = {"scenario": {"id": "x", "duration_s": 10.0, "faults": [
        {"type": "imu_high_noise", "onset_s": 0.0, "duration_s": 10.0,
         "params": {"noise_scale": 10.0}},
        {"type": "imu_high_noise", "onset_s": 0.0, "duration_s": 10.0,
         "params": {"noise_scale": 2.0}}]}}
    weak_first = {"scenario": {"id": "x", "duration_s": 10.0, "faults": [
        strong_first["scenario"]["faults"][1],
        strong_first["scenario"]["faults"][0]]}}
    for s in (strong_first, weak_first):
        assert sc.knobs_at(s, 5.0).get(
            "imu_preprocessor", "fault.noise_scale") == 10.0

    # Same for a bias, by magnitude rather than by value, so a large negative
    # bias is not beaten by a small positive one.
    biases = {"scenario": {"id": "x", "duration_s": 10.0, "faults": [
        {"type": "encoder_drift", "onset_s": 0.0, "duration_s": 10.0,
         "params": {"wheel": "left", "bias_rad_s": -0.08}},
        {"type": "encoder_drift", "onset_s": 0.0, "duration_s": 10.0,
         "params": {"wheel": "left", "bias_rad_s": 0.01}}]}}
    assert sc.knobs_at(biases, 5.0).get(
        "encoder_preprocessor", "fault.left_bias_rad_s") == pytest.approx(-0.08)


def test_an_outage_wins_outright_over_a_noise_fault():
    s = {"scenario": {"id": "x", "duration_s": 10.0, "faults": [
        {"type": "imu_high_noise", "onset_s": 0.0, "duration_s": 10.0,
         "params": {"noise_scale": 3.0}},
        {"type": "sensor_outage", "onset_s": 2.0, "duration_s": 3.0,
         "params": {"sensor": "imu"}}]}}
    k = sc.knobs_at(s, 3.0)
    assert k.get("imu_preprocessor", "fault.outage") is True
    assert k.get("imu_preprocessor", "fault.noise_scale") == 3.0
    assert sc.knobs_at(s, 6.0).get("imu_preprocessor", "fault.outage") is False


# ----------------------------------------------------------------- deltas ----
def test_changed_from_reports_only_what_differs():
    a = sc.nominal_knobs()
    b = sc.knobs_at(load("case_b_imu_high_noise.yaml"), 50.0)
    delta = b.changed_from(a)
    assert delta == {"imu_preprocessor": {"fault.noise_scale": 10.0,
                                          "fault.accel_noise_scale": 10.0}}
    assert b.changed_from(b) == {}


@pytest.mark.parametrize("path", all_scenarios(), ids=lambda p: p.stem)
def test_every_scenario_with_a_fault_leaves_a_recovery_window(path):
    """Check there is a post-fault interval in which recovery can be measured.

    Recovery time is one of the three numbers this phase reports, and it is
    undefined without an interval in which the sensors are healthy again. The
    first version of these files ran every fault to the last second, so four of
    six cases could not produce the column at all.
    """
    s = yaml.safe_load(path.read_text())
    faults = s["scenario"]["faults"]
    if not faults:
        return                                   # Case A, nothing to recover from
    duration = float(s["scenario"]["duration_s"])
    last_end = max(f["onset_s"] + f["duration_s"] for f in faults)
    assert duration - last_end >= 20.0, (
        f"only {duration - last_end:.0f} s of healthy tail after the last fault")
    # And the knobs really are clear throughout that tail, not just at the end.
    for t in (last_end + 0.1, (last_end + duration) / 2.0, duration - 0.1):
        assert sc.knobs_at(s, t).values == sc.nominal_knobs().values


# --------------------------------------------------------------- timeline ----
@pytest.mark.parametrize("path", all_scenarios(), ids=lambda p: p.stem)
def test_timeline_is_sorted_and_within_the_scenario(path):
    s = yaml.safe_load(path.read_text())
    edges = sc.timeline(s)
    assert edges == sorted(edges)
    assert edges[0] == 0.0
    assert max(edges) <= float(s["scenario"]["duration_s"]) + 1e-9


def test_timeline_includes_every_burst_edge():
    s = load("case_d_lidar_dropout.yaml")   # 25 s onset, 95 s long, 3 on / 2 off
    edges = sc.timeline(s)
    for k in range(5):
        assert any(math.isclose(e, 25.0 + k * 5.0) for e in edges)
        assert any(math.isclose(e, 28.0 + k * 5.0) for e in edges)


@pytest.mark.parametrize("path", all_scenarios(), ids=lambda p: p.stem)
def test_windows_match_the_file(path):
    s = yaml.safe_load(path.read_text())
    w = sc.windows(s)
    assert len(w) == len(s["scenario"]["faults"])
    for record, fault in zip(w, s["scenario"]["faults"]):
        assert record["type"] == fault["type"]
        assert record["end_s"] > record["onset_s"]
