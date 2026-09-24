"""Keep the two configuration layers from drifting apart.

The project deliberately has configuration in two places:

* ``configs/`` at the repo root holds the **experiment-level** description --
  the robot's physical constants and the sensor specifications. It is read by
  the URDF (through ``xacro.load_yaml``), by the analysis scripts, and by the
  evaluation toolkit, none of which are ROS nodes.

* ``<package>/config/`` holds **node-level** ROS parameters, because that is
  where ``ament_index`` and ``launch_ros`` expect to find them, and because a
  ROS parameter file must be a flat per-node document.

The nominal numbers therefore appear twice. That is a deliberate trade, and it
is only safe if the duplication cannot drift silently -- so these tests assert
the two agree. Without them, editing ``configs/sensors.yaml`` would change the
simulated sensor while the preprocessing node kept publishing a covariance
derived from the old value, and the EKF would be tuned against a sensor that no
longer exists.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "ros2_ws" / "src" / "arb_evaluation"))
from arb_evaluation import scenario as sc  # noqa: E402
ROOT_SENSORS = REPO_ROOT / "configs" / "sensors.yaml"
ROOT_ROBOT = REPO_ROOT / "configs" / "robot.yaml"
NODE_SENSORS = (REPO_ROOT / "ros2_ws" / "src" / "arb_sensors" / "config" / "sensors.yaml")

TOL = 1e-12


@pytest.fixture(scope="module")
def root_cfg() -> dict:
    return yaml.safe_load(ROOT_SENSORS.read_text())


@pytest.fixture(scope="module")
def node_cfg() -> dict:
    raw = yaml.safe_load(NODE_SENSORS.read_text())
    return {key.split("/")[-1]: value["ros__parameters"] for key, value in raw.items()}


def test_both_config_files_exist():
    assert ROOT_SENSORS.is_file(), ROOT_SENSORS
    assert NODE_SENSORS.is_file(), NODE_SENSORS


def test_every_node_section_is_present(node_cfg: dict):
    expected = {"imu_preprocessor", "encoder_preprocessor", "lidar_preprocessor"}
    assert expected <= set(node_cfg), f"missing: {sorted(expected - set(node_cfg))}"


@pytest.mark.parametrize(
    "node,param_path,root_path",
    [
        # --- IMU ---
        ("imu_preprocessor", ("rate_hz",), ("imu", "rate_hz")),
        ("imu_preprocessor", ("gyro", "noise_density"),
         ("imu", "gyroscope", "noise_density_rad_s_sqrt_hz")),
        ("imu_preprocessor", ("gyro", "bias_stability"),
         ("imu", "gyroscope", "bias_stability_rad_s")),
        ("imu_preprocessor", ("gyro", "bias_random_walk"),
         ("imu", "gyroscope", "bias_random_walk_rad_s2")),
        ("imu_preprocessor", ("gyro", "scale_error"),
         ("imu", "gyroscope", "scale_factor_error")),
        ("imu_preprocessor", ("accel", "noise_density"),
         ("imu", "accelerometer", "noise_density_m_s2_sqrt_hz")),
        ("imu_preprocessor", ("accel", "bias_stability"),
         ("imu", "accelerometer", "bias_stability_m_s2")),
        ("imu_preprocessor", ("accel", "bias_random_walk"),
         ("imu", "accelerometer", "bias_random_walk_m_s3")),
        ("imu_preprocessor", ("accel", "scale_error"),
         ("imu", "accelerometer", "scale_factor_error")),
        # --- encoder ---
        ("encoder_preprocessor", ("rate_hz",), ("wheel_encoder", "rate_hz")),
        ("encoder_preprocessor", ("counts_per_revolution",),
         ("wheel_encoder", "counts_per_revolution")),
        ("encoder_preprocessor", ("quadrature_multiplier",),
         ("wheel_encoder", "quadrature_multiplier")),
        ("encoder_preprocessor", ("velocity_noise_sigma",),
         ("wheel_encoder", "velocity_noise_sigma_rad_s")),
        ("encoder_preprocessor", ("slip_ratio_mean",),
         ("wheel_encoder", "slip_ratio_mean")),
        ("encoder_preprocessor", ("slip_ratio_sigma",),
         ("wheel_encoder", "slip_ratio_sigma")),
        # --- lidar ---
        ("lidar_preprocessor", ("rate_hz",), ("lidar", "rate_hz")),
        ("lidar_preprocessor", ("range_noise_sigma",),
         ("lidar", "range_noise_sigma_m")),
    ],
)
def test_nominal_values_agree(node_cfg, root_cfg, node, param_path, root_path):
    """Each duplicated specification number must match its source of truth."""
    node_value = node_cfg[node]
    for key in param_path:
        node_value = node_value[key]
    root_value = root_cfg
    for key in root_path:
        root_value = root_value[key]
    assert math.isclose(float(node_value), float(root_value), rel_tol=TOL), (
        f"{node}.{'.'.join(param_path)} = {node_value} but "
        f"configs/sensors.yaml {'.'.join(root_path)} = {root_value}"
    )


@pytest.mark.parametrize(
    "node,topic_param,root_path",
    [
        ("imu_preprocessor", "input_topic", ("imu", "topic_raw")),
        ("imu_preprocessor", "output_topic", ("imu", "topic_out")),
        ("encoder_preprocessor", "output_topic", ("wheel_encoder", "topic_out")),
        ("lidar_preprocessor", "input_topic", ("lidar", "topic_raw")),
        ("lidar_preprocessor", "output_topic", ("lidar", "topic_out")),
    ],
)
def test_topic_names_agree(node_cfg, root_cfg, node, topic_param, root_path):
    """A mismatched topic name is a node that subscribes to nothing, silently."""
    root_value = root_cfg
    for key in root_path:
        root_value = root_value[key]
    assert node_cfg[node][topic_param] == root_value


def test_every_node_declares_a_fault_section(node_cfg: dict):
    """Phase 10 drives these at runtime; a missing section is a missing knob."""
    for node, params in node_cfg.items():
        assert "fault" in params, f"{node} has no fault section"
        assert "outage" in params["fault"], f"{node}.fault has no outage flag"


def _nominal_fault_value(key: str, value):
    """The value a fault knob must hold on a HEALTHY robot.

    Inferred from the name, not from a list: a MULTIPLIER is nominal at 1.0 and
    everything else is nominal at nothing-added, so anything ending in `_scale`
    is 1.0, a flag is False, and the rest are 0.0.
    """
    if isinstance(value, bool):
        return False
    if key.endswith("_scale"):
        return 1.0
    return 0.0


def test_faults_are_disabled_by_default(node_cfg: dict):
    """The checked-in configuration must describe a HEALTHY robot.

    A fault accidentally left enabled in the default parameter file would
    silently contaminate every nominal run, and the Case A baseline that every
    other scenario is measured against would be meaningless.
    """
    offenders: list[str] = []
    for node, params in node_cfg.items():
        for key, value in params["fault"].items():
            if value != _nominal_fault_value(key, value):
                offenders.append(
                    f"{node}.fault.{key} = {value} "
                    f"(expected {_nominal_fault_value(key, value)})")
    assert not offenders, "faults enabled in the default config:\n  " + "\n  ".join(offenders)


def test_every_fault_knob_follows_the_naming_convention(node_cfg: dict):
    """The healthy default is inferred from the NAME, so the name must be clear.

    `_nominal_fault_value` reads a knob's healthy value off its name rather than
    from a list of known keys. An earlier version special-cased the single name
    `noise_scale`, so adding `accel_noise_scale` made the check demand that a
    multiplier default to zero -- i.e. a sensor with no noise at all -- and the
    new knob looked like a fault left switched on.

    A knob whose name says nothing about its type would break the inference
    silently, so it is rejected here instead.
    """
    for node, params in node_cfg.items():
        for key, value in params["fault"].items():
            recognised = (
                isinstance(value, bool)
                or key.endswith("_scale")
                or key.endswith("_ratio")
                or key.endswith("_probability")
                or key.endswith("_rad_s")
                or key.endswith("_offset")
            )
            assert recognised, (
                f"{node}.fault.{key} does not follow the convention: a fault "
                f"knob must be a bool, or end in _scale (nominal 1.0) or one of "
                f"_ratio/_probability/_rad_s/_offset (nominal 0.0)")


def test_encoder_resolution_is_representable(root_cfg: dict):
    """Sanity-check the encoder specification itself."""
    enc = root_cfg["wheel_encoder"]
    counts = enc["counts_per_revolution"] * enc["quadrature_multiplier"]
    assert counts > 0
    resolution = 2.0 * math.pi / counts
    wheel_r = yaml.safe_load(ROOT_ROBOT.read_text())["robot"]["drive"]["wheel_radius_m"]
    arc_mm = resolution * wheel_r * 1000.0
    # A resolution coarser than a millimetre of arc would dominate every other
    # odometry error source and make Phase 3 a study of quantisation.
    assert arc_mm < 1.0, f"encoder resolves {arc_mm:.3f} mm of arc, too coarse"


def test_lidar_angular_span_leaves_no_duplicate_beam(root_cfg: dict):
    """Gazebo derives the increment as (max-min)/(samples-1).

    Spanning exactly 2*pi therefore makes the first and last beam point the same
    way and the spacing 2*pi/(N-1) instead of 2*pi/N.
    """
    lidar = root_cfg["lidar"]
    n = lidar["samples"]
    span = lidar["angle_max_rad"] - lidar["angle_min_rad"]
    increment = span / (n - 1)
    assert math.isclose(increment, 2.0 * math.pi / n, rel_tol=1e-9), (
        f"beam spacing is {increment:.8f} rad, expected exactly "
        f"2*pi/{n} = {2.0 * math.pi / n:.8f}"
    )


# ---------------------------------------------------------------- localisation

LOCALIZATION_CFG = (
    REPO_ROOT / "ros2_ws" / "src" / "arb_localization" / "config" / "localization.yaml"
)


@pytest.fixture(scope="module")
def localization_cfg() -> dict:
    raw = yaml.safe_load(LOCALIZATION_CFG.read_text())
    return {key.split("/")[-1]: value["ros__parameters"] for key, value in raw.items()}


def test_odometry_wheel_radius_matches_the_robot_model(localization_cfg: dict):
    """Radius is geometry and must agree with the URDF's source of truth."""
    robot = yaml.safe_load(ROOT_ROBOT.read_text())["robot"]["drive"]
    assert math.isclose(
        float(localization_cfg["wheel_odometry"]["wheel_radius_m"]),
        float(robot["wheel_radius_m"]), rel_tol=1e-12)


def test_odometry_encoder_spec_matches_the_sensor_config(
        localization_cfg: dict, root_cfg: dict):
    """The covariance model needs the same resolution the encoder actually has."""
    enc = root_cfg["wheel_encoder"]
    odo = localization_cfg["wheel_odometry"]
    assert odo["counts_per_revolution"] == enc["counts_per_revolution"]
    assert odo["quadrature_multiplier"] == enc["quadrature_multiplier"]


def test_odometry_track_width_is_calibrated_not_geometric(localization_cfg: dict):
    """The estimator's track width is a CALIBRATED value and must differ.

    This test looks backwards and is deliberate. The URDF's 0.350 m is the
    distance between the wheel centre planes and is correct as geometry. The
    odometry needs the EFFECTIVE track width, which is where traction acts on a
    line contact, measured at 0.3154 m.

    Using the geometric value loses 10 % of every rotation: 72 degrees of
    heading error over two laps of a rectangle, and a metre of position error
    after 10 m, with the odometry still reporting a 5 cm loop closure. Someone
    "fixing the inconsistency" by copying 0.350 across would reintroduce all of
    it, so the difference is asserted rather than merely commented.
    """
    geometric = float(
        yaml.safe_load(ROOT_ROBOT.read_text())["robot"]["drive"]["wheel_separation_m"])
    calibrated = float(localization_cfg["wheel_odometry"]["wheel_separation_m"])

    assert not math.isclose(calibrated, geometric, rel_tol=1e-6), (
        "wheel_separation_m in localization.yaml equals the geometric value "
        f"{geometric}. It must be the CALIBRATED effective track width; run "
        "scripts/calibrate_odometry.py."
    )
    ratio = calibrated / geometric
    assert 0.85 < ratio < 1.15, (
        f"calibrated track width is {100 * (ratio - 1):+.1f} % from geometry, "
        "which is too far to be a contact-patch effect; re-run the calibration"
    )
    # The physical bound: traction cannot act outside the wheel's own width.
    wheel_width = float(
        yaml.safe_load(ROOT_ROBOT.read_text())["robot"]["drive"]["wheel_width_m"])
    assert geometric - wheel_width <= calibrated <= geometric + wheel_width, (
        f"calibrated {calibrated} lies outside [inner edge, outer edge] = "
        f"[{geometric - wheel_width}, {geometric + wheel_width}]"
    )


# ===========================================================================
#  The fault scheduler's idea of "healthy" vs the parameter files' idea
# ===========================================================================
#
# arb_evaluation/scenario.py writes out every fault knob and its nominal value
# rather than discovering them, because the scheduler's job when a fault ends
# is to return the robot to a KNOWN-GOOD state. A knob it does not know about
# is a knob it cannot clear, and the contamination would carry silently into
# whatever scenario ran next -- most likely a nominal one, whose numbers every
# other case is then measured against.
#
# Writing the list out is only safe if it cannot drift from the parameter files
# it claims to mirror, which is what these two tests are for.


def test_scheduler_knows_every_fault_knob_the_nodes_declare(node_cfg: dict):
    """Every knob in the parameter files must appear in scenario.NOMINAL."""
    missing: list[str] = []
    for node, params in node_cfg.items():
        assert node in sc.NOMINAL, f"{node} has fault knobs the scheduler cannot clear"
        for key in params["fault"]:
            if f"fault.{key}" not in sc.NOMINAL[node]:
                missing.append(f"{node}.fault.{key}")
    assert not missing, (
        "knobs the fault scheduler would never reset:\n  " + "\n  ".join(missing))


def test_scheduler_invents_no_knob_the_nodes_do_not_have(node_cfg: dict):
    """And nothing the other way either.

    A knob in NOMINAL that no node declares makes the scheduler call
    set_parameters with a name the node rejects. The rejection is logged and
    the run continues, so the scenario silently applies less than it says.
    """
    extra: list[str] = []
    for node, params in sc.NOMINAL.items():
        assert node in node_cfg, f"scenario.NOMINAL names unknown node '{node}'"
        declared = {f"fault.{k}" for k in node_cfg[node]["fault"]}
        for key in params:
            if key not in declared:
                extra.append(f"{node}.{key}")
    assert not extra, (
        "knobs the scheduler would set that no node declares:\n  "
        + "\n  ".join(extra))


def test_scheduler_nominal_values_match_the_parameter_files(node_cfg: dict):
    """The healthy value must be the same in both places.

    If they disagree, a scenario would "restore nominal" to something the
    checked-in configuration does not consider nominal, and the difference
    would only show up as an unexplained offset in the next run's baseline.
    """
    mismatches: list[str] = []
    for node, params in sc.NOMINAL.items():
        for key, value in params.items():
            declared = node_cfg[node]["fault"][key.removeprefix("fault.")]
            if declared != value:
                mismatches.append(
                    f"{node}.{key}: config has {declared}, scheduler has {value}")
    assert not mismatches, "nominal values disagree:\n  " + "\n  ".join(mismatches)
