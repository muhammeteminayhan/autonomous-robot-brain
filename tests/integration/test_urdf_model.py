"""Check that the robot model matches its configuration and survives conversion.

Three classes of silent failure are covered:

1. **Config drift.** The URDF claims to read every dimension from
   ``configs/robot.yaml``. If a number is ever hard-coded back into the xacro,
   the simulated robot and the odometry equations stop agreeing, and the result
   is a constant scale error that looks exactly like sensor drift.

2. **Fixed-joint lumping.** Gazebo merges a link joined by a fixed joint into
   its parent when converting URDF to SDF. If ``base_link`` is lumped away,
   every gz system configured with ``<robot_base_frame>base_link</...>``
   silently publishes nothing. There is no error message; the topic just never
   appears.

3. **Unsupported geometry.** A malformed inertia or a missing collision only
   shows up as strange physics much later.
"""

from __future__ import annotations

import math
import shutil
import subprocess
from pathlib import Path
from xml.etree import ElementTree as ET

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
XACRO = REPO_ROOT / "ros2_ws" / "src" / "arb_description" / "urdf" / "arb_robot.urdf.xacro"
ROBOT_CFG = REPO_ROOT / "configs" / "robot.yaml"
SENSOR_CFG = REPO_ROOT / "configs" / "sensors.yaml"

TOL = 1e-9


@pytest.fixture(scope="module")
def robot_cfg() -> dict:
    return yaml.safe_load(ROBOT_CFG.read_text())["robot"]


@pytest.fixture(scope="module")
def urdf_root() -> ET.Element:
    """Run xacro and return the parsed URDF."""
    if shutil.which("xacro") is None:
        pytest.skip("xacro not on PATH; source the ROS 2 environment")
    proc = subprocess.run(
        [
            "xacro", str(XACRO),
            f"robot_config:={ROBOT_CFG}",
            f"sensor_config:={SENSOR_CFG}",
            "use_gazebo:=true",
        ],
        capture_output=True, text=True, timeout=120,
    )
    if proc.returncode != 0:
        pytest.fail(f"xacro failed:\n{proc.stderr}")
    return ET.fromstring(proc.stdout)


@pytest.fixture(scope="module")
def urdf_text() -> str:
    if shutil.which("xacro") is None:
        pytest.skip("xacro not on PATH; source the ROS 2 environment")
    return subprocess.run(
        [
            "xacro", str(XACRO),
            f"robot_config:={ROBOT_CFG}",
            f"sensor_config:={SENSOR_CFG}",
            "use_gazebo:=true",
        ],
        capture_output=True, text=True, timeout=120, check=True,
    ).stdout


def _joint(root: ET.Element, name: str) -> ET.Element:
    for j in root.findall("joint"):
        if j.get("name") == name:
            return j
    raise AssertionError(f"joint '{name}' not found")


def _origin_xyz(joint: ET.Element) -> tuple[float, float, float]:
    origin = joint.find("origin")
    assert origin is not None, f"joint {joint.get('name')} has no <origin>"
    return tuple(float(v) for v in origin.get("xyz").split())  # type: ignore[return-value]


# ------------------------------------------------------------------ structure

def test_base_link_is_the_root(urdf_root: ET.Element):
    """base_link must be the root, or SDF conversion lumps it away."""
    links = {link.get("name") for link in urdf_root.findall("link")}
    children = {j.find("child").get("link") for j in urdf_root.findall("joint")}  # type: ignore[union-attr]
    roots = links - children
    assert roots == {"base_link"}, (
        f"expected base_link to be the only root, got {roots}. "
        "See the header of arb_robot.urdf.xacro for why this matters."
    )


def test_expected_links_exist(urdf_root: ET.Element):
    links = {link.get("name") for link in urdf_root.findall("link")}
    expected = {
        "base_link", "base_footprint",
        "left_wheel_link", "right_wheel_link",
        "front_caster_link", "rear_caster_link",
        "imu_link", "lidar_link",
    }
    assert expected <= links, f"missing links: {sorted(expected - links)}"


def test_wheels_are_continuous_about_y(urdf_root: ET.Element):
    for name in ("left_wheel_joint", "right_wheel_joint"):
        joint = _joint(urdf_root, name)
        assert joint.get("type") == "continuous", f"{name} is {joint.get('type')}"
        axis = joint.find("axis")
        assert axis is not None
        xyz = [float(v) for v in axis.get("xyz").split()]  # type: ignore[union-attr]
        # Both wheels must share the same axis direction, otherwise a "drive
        # straight" command spins the robot on the spot.
        assert xyz == [0.0, 1.0, 0.0], f"{name} axis is {xyz}, expected [0, 1, 0]"


# ------------------------------------------------------ dimensions vs config

def test_wheel_placement_matches_config(urdf_root: ET.Element, robot_cfg: dict):
    drive = robot_cfg["drive"]
    chassis = robot_cfg["chassis"]
    base_z = chassis["ground_clearance_m"] + chassis["height_m"] / 2.0
    half_sep = drive["wheel_separation_m"] / 2.0
    wheel_z = drive["wheel_radius_m"] - base_z

    lx, ly, lz = _origin_xyz(_joint(urdf_root, "left_wheel_joint"))
    rx, ry, rz = _origin_xyz(_joint(urdf_root, "right_wheel_joint"))
    assert math.isclose(ly, half_sep, abs_tol=TOL), f"left wheel y={ly}"
    assert math.isclose(ry, -half_sep, abs_tol=TOL), f"right wheel y={ry}"
    assert math.isclose(lz, wheel_z, abs_tol=TOL)
    assert math.isclose(rz, wheel_z, abs_tol=TOL)
    assert math.isclose(lx, 0.0, abs_tol=TOL) and math.isclose(rx, 0.0, abs_tol=TOL)


def test_wheels_clear_the_chassis(robot_cfg: dict):
    """Overlapping collision shapes make the physics solver fight itself."""
    drive = robot_cfg["drive"]
    chassis = robot_cfg["chassis"]
    inner_face = drive["wheel_separation_m"] / 2.0 - drive["wheel_width_m"] / 2.0
    assert inner_face > chassis["width_m"] / 2.0, (
        f"wheel inner face at {inner_face:.4f} m overlaps the chassis "
        f"half-width {chassis['width_m'] / 2.0:.4f} m"
    )


def test_two_casters_straddle_the_wheel_axle(robot_cfg: dict, urdf_root: ET.Element):
    """One caster is not enough; see configs/robot.yaml for the measured failure."""
    offsets = robot_cfg["casters"]["x_offsets_m"]
    assert len(offsets) == 2, "expected exactly two casters"
    assert max(offsets) > 0.0 > min(offsets), (
        f"casters at {offsets} are all on one side of the wheel axle at x = 0; "
        "the robot will pitch onto the unsupported end"
    )
    front_x, _, _ = _origin_xyz(_joint(urdf_root, "front_caster_joint"))
    rear_x, _, _ = _origin_xyz(_joint(urdf_root, "rear_caster_joint"))
    assert math.isclose(front_x, max(offsets), abs_tol=TOL)
    assert math.isclose(rear_x, min(offsets), abs_tol=TOL)


def test_casters_fit_under_the_chassis(robot_cfg: dict):
    r = robot_cfg["casters"]["radius_m"]
    clearance = robot_cfg["chassis"]["ground_clearance_m"]
    assert 2 * r <= clearance + TOL, (
        f"caster diameter {2 * r} m exceeds ground clearance {clearance} m; "
        "the sphere would intersect the chassis"
    )


def test_sensor_mounts_match_config(urdf_root: ET.Element, robot_cfg: dict):
    for key, joint_name in (("imu", "imu_joint"), ("lidar", "lidar_joint")):
        want = robot_cfg["sensor_mounts"][key]["xyz_m"]
        got = _origin_xyz(_joint(urdf_root, joint_name))
        assert all(math.isclose(a, b, abs_tol=TOL) for a, b in zip(want, got)), (
            f"{joint_name} origin {got} does not match configs/robot.yaml {want}"
        )


def test_base_footprint_sits_on_the_ground(urdf_root: ET.Element, robot_cfg: dict):
    chassis = robot_cfg["chassis"]
    base_z = chassis["ground_clearance_m"] + chassis["height_m"] / 2.0
    _, _, z = _origin_xyz(_joint(urdf_root, "base_link_to_base_footprint"))
    assert math.isclose(z, -base_z, abs_tol=TOL), (
        f"base_footprint at z={z} relative to base_link, expected {-base_z}"
    )


def test_every_link_has_positive_mass_and_inertia(urdf_root: ET.Element):
    """A zero or placeholder inertia produces absurd dynamics."""
    for link in urdf_root.findall("link"):
        inertial = link.find("inertial")
        assert inertial is not None, f"link {link.get('name')} has no <inertial>"
        mass = float(inertial.find("mass").get("value"))  # type: ignore[union-attr]
        assert mass > 0.0, f"link {link.get('name')} has mass {mass}"
        inertia = inertial.find("inertia")
        assert inertia is not None
        for axis in ("ixx", "iyy", "izz"):
            val = float(inertia.get(axis))  # type: ignore[arg-type]
            assert val > 0.0, f"link {link.get('name')} has {axis}={val}"


# --------------------------------------------------------- SDF conversion

def test_sdf_conversion_keeps_base_link(urdf_text: str, tmp_path: Path):
    """The lumping check. This is the one that silently breaks ground truth."""
    if shutil.which("gz") is None:
        pytest.skip("gz not on PATH")
    urdf_file = tmp_path / "robot.urdf"
    urdf_file.write_text(urdf_text)
    proc = subprocess.run(["gz", "sdf", "-p", str(urdf_file)],
                          capture_output=True, text=True, timeout=120)
    assert proc.returncode == 0, f"gz sdf failed:\n{proc.stderr}"

    model = ET.fromstring(proc.stdout).find("model")
    assert model is not None
    sdf_links = {link.get("name") for link in model.findall("link")}
    assert "base_link" in sdf_links, (
        f"base_link was lumped away during URDF->SDF conversion; SDF links are "
        f"{sorted(sdf_links)}. Gazebo systems referencing base_link "
        "(OdometryPublisher for ground truth, DiffDrive) will publish nothing."
    )


def test_sdf_conversion_preserves_sensor_poses(urdf_text: str, robot_cfg: dict,
                                               tmp_path: Path):
    """Lumping relocates sensors into the parent link; the pose must survive."""
    if shutil.which("gz") is None:
        pytest.skip("gz not on PATH")
    urdf_file = tmp_path / "robot.urdf"
    urdf_file.write_text(urdf_text)
    proc = subprocess.run(["gz", "sdf", "-p", str(urdf_file)],
                          capture_output=True, text=True, timeout=120, check=True)
    model = ET.fromstring(proc.stdout).find("model")
    assert model is not None

    poses = {}
    for sensor in model.iter("sensor"):
        text = sensor.findtext("pose")
        assert text is not None, f"sensor {sensor.get('name')} lost its pose"
        poses[sensor.get("name")] = [float(v) for v in text.split()]

    for sensor_name, cfg_key in (("imu_sensor", "imu"), ("lidar_sensor", "lidar")):
        assert sensor_name in poses, f"{sensor_name} missing from SDF"
        want = robot_cfg["sensor_mounts"][cfg_key]["xyz_m"]
        got = poses[sensor_name][:3]
        assert all(math.isclose(a, b, abs_tol=1e-6) for a, b in zip(want, got)), (
            f"{sensor_name} at {got} after conversion, expected {want}"
        )
