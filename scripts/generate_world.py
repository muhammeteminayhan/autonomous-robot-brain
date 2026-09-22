#!/usr/bin/env python3
"""Generate the Gazebo Harmonic SDF world from the YAML floor plan.

Why generate instead of hand-writing SDF: see the header of ``configs/world.yaml``.

Usage::

    python3 scripts/generate_world.py                    # default paths
    python3 scripts/generate_world.py --check            # verify it is up to date
    python3 scripts/generate_world.py -o /tmp/other.sdf

The generator is deterministic -- the same YAML always produces byte-identical
SDF -- so ``--check`` can be wired into CI to catch a world file that was edited
by hand and has drifted from its source of truth.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Any
from xml.dom import minidom
from xml.etree import ElementTree as ET

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = REPO_ROOT / "configs" / "world.yaml"
DEFAULT_OUTPUT = REPO_ROOT / "ros2_ws" / "src" / "arb_simulation" / "worlds" / "arb_indoor.sdf"

#: SDF version shipped with Gazebo Harmonic (sdformat14).
SDF_VERSION = "1.10"

#: Mass is irrelevant for a static model -- the physics engine never integrates
#: it -- but SDF still requires a well-formed <inertial> block.
STATIC_MASS_KG = 1.0


def _sub(parent: ET.Element, tag: str, text: str | None = None, **attrs: str) -> ET.Element:
    el = ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})
    if text is not None:
        el.text = text
    return el


def _pose(x: float, y: float, z: float, yaw: float = 0.0) -> str:
    return f"{x:.6f} {y:.6f} {z:.6f} 0 0 {yaw:.6f}"


def _add_material(parent: ET.Element, rgba: tuple[float, float, float, float]) -> None:
    mat = _sub(parent, "material")
    r, g, b, a = rgba
    _sub(mat, "ambient", f"{r * 0.5:.3f} {g * 0.5:.3f} {b * 0.5:.3f} {a}")
    _sub(mat, "diffuse", f"{r:.3f} {g:.3f} {b:.3f} {a}")
    _sub(mat, "specular", "0.1 0.1 0.1 1")


def _static_model(
    world: ET.Element,
    name: str,
    pose: str,
    geometry: dict[str, Any],
    rgba: tuple[float, float, float, float],
) -> None:
    """Append one static model whose visual and collision share a geometry."""
    model = _sub(world, "model", name=name)
    _sub(model, "static", "true")
    _sub(model, "pose", pose)
    link = _sub(model, "link", name="link")

    inertial = _sub(link, "inertial")
    _sub(inertial, "mass", str(STATIC_MASS_KG))
    inertia = _sub(inertial, "inertia")
    for comp, val in (("ixx", 1.0), ("ixy", 0.0), ("ixz", 0.0),
                      ("iyy", 1.0), ("iyz", 0.0), ("izz", 1.0)):
        _sub(inertia, comp, str(val))

    for kind in ("collision", "visual"):
        el = _sub(link, kind, name=kind)
        geo = _sub(el, "geometry")
        if geometry["shape"] == "box":
            _sub(_sub(geo, "box"), "size", geometry["size"])
        else:
            cyl = _sub(geo, "cylinder")
            _sub(cyl, "radius", f"{geometry['radius']:.6f}")
            _sub(cyl, "length", f"{geometry['length']:.6f}")
        if kind == "visual":
            _add_material(el, rgba)


def build_world(cfg: dict[str, Any]) -> ET.Element:
    """Build the SDF element tree for the world described by ``cfg``."""
    w = cfg["world"]
    thickness = float(w["wall"]["thickness_m"])
    height = float(w["wall"]["height_m"])

    sdf = ET.Element("sdf", version=SDF_VERSION)
    world = _sub(sdf, "world", name=w["name"])

    physics = _sub(world, "physics", name="default_physics", type="ignored")
    _sub(physics, "max_step_size", str(w["physics_step_s"]))
    _sub(physics, "real_time_factor", "1.0")
    _sub(world, "gravity", " ".join(str(g) for g in w["gravity_m_s2"]))

    # --- systems -------------------------------------------------------------
    # Physics, UserCommands and SceneBroadcaster are the minimum for an
    # interactive world. Imu and Sensors are what make the robot's <sensor>
    # elements actually produce data: without them the URDF looks correct and
    # the topics simply never appear, which is a classic silent failure.
    for filename, cls in (
        ("gz-sim-physics-system", "gz::sim::systems::Physics"),
        ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
        ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
        ("gz-sim-imu-system", "gz::sim::systems::Imu"),
        ("gz-sim-contact-system", "gz::sim::systems::Contact"),
    ):
        _sub(world, "plugin", filename=filename, name=cls)
    sensors = _sub(world, "plugin",
                   filename="gz-sim-sensors-system",
                   name="gz::sim::systems::Sensors")
    # "ogre2" is the GPU renderer. On a machine without working GL the LiDAR
    # silently publishes nothing; see README "Known environment pitfalls".
    _sub(sensors, "render_engine", "ogre2")

    # --- scene / lighting ----------------------------------------------------
    scene = _sub(world, "scene")
    _sub(scene, "ambient", "0.6 0.6 0.6 1")
    _sub(scene, "background", "0.75 0.82 0.90 1")
    _sub(scene, "shadows", "true")

    light = _sub(world, "light", type="directional", name="sun")
    _sub(light, "cast_shadows", "true")
    _sub(light, "pose", _pose(0, 0, 12))
    _sub(light, "diffuse", "0.9 0.9 0.9 1")
    _sub(light, "specular", "0.2 0.2 0.2 1")
    _sub(light, "direction", "-0.4 0.3 -0.9")
    att = _sub(light, "attenuation")
    _sub(att, "range", "60")
    _sub(att, "constant", "0.9")
    _sub(att, "linear", "0.01")
    _sub(att, "quadratic", "0.001")

    # --- ground --------------------------------------------------------------
    b = w["bounds"]
    span_x = float(b["x_max"]) - float(b["x_min"])
    span_y = float(b["y_max"]) - float(b["y_min"])
    ground = _sub(world, "model", name="ground_plane")
    _sub(ground, "static", "true")
    glink = _sub(ground, "link", name="link")
    for kind in ("collision", "visual"):
        el = _sub(glink, kind, name=kind)
        plane = _sub(_sub(el, "geometry"), "plane")
        _sub(plane, "normal", "0 0 1")
        _sub(plane, "size", f"{span_x + 2:.1f} {span_y + 2:.1f}")
        if kind == "visual":
            _add_material(el, (0.55, 0.55, 0.58, 1.0))
        else:
            # Wheel traction. Without an explicit friction block the default is
            # mu = 1.0, which is fine, but stating it makes the odometry slip
            # behaviour reproducible across Gazebo versions.
            friction = _sub(_sub(el, "surface"), "friction")
            ode = _sub(friction, "ode")
            _sub(ode, "mu", "1.0")
            _sub(ode, "mu2", "1.0")

    # --- walls ---------------------------------------------------------------
    for wall in w["walls"]:
        x1, y1, x2, y2 = (float(v) for v in wall["segment"])
        dx, dy = x2 - x1, y2 - y1
        length = math.hypot(dx, dy)
        if length <= 0.0:
            raise ValueError(f"wall '{wall['name']}' has zero length")
        yaw = math.atan2(dy, dx)
        # Extend by one thickness so segments meeting at a corner overlap
        # instead of leaving a diagonal slit that LiDAR rays leak through.
        _static_model(
            world,
            f"wall_{wall['name']}",
            _pose((x1 + x2) / 2.0, (y1 + y2) / 2.0, height / 2.0, yaw),
            {"shape": "box", "size": f"{length + thickness:.6f} {thickness:.6f} {height:.6f}"},
            (0.82, 0.80, 0.76, 1.0),
        )

    # --- obstacles -----------------------------------------------------------
    for obs in w["obstacles"]:
        ox, oy = (float(v) for v in obs["pose"])
        if obs["type"] == "box":
            sx, sy, sz = (float(v) for v in obs["size"])
            geo = {"shape": "box", "size": f"{sx:.6f} {sy:.6f} {sz:.6f}"}
            z = sz / 2.0
            rgba = (0.70, 0.45, 0.25, 1.0)
        elif obs["type"] == "cylinder":
            h = float(obs["height"])
            geo = {"shape": "cylinder", "radius": float(obs["radius"]), "length": h}
            z = h / 2.0
            rgba = (0.35, 0.45, 0.65, 1.0)
        else:
            raise ValueError(f"unknown obstacle type '{obs['type']}' in '{obs['name']}'")
        _static_model(world, f"obstacle_{obs['name']}",
                      _pose(ox, oy, z, float(obs.get("yaw", 0.0))), geo, rgba)

    return sdf


def render(sdf: ET.Element) -> str:
    raw = ET.tostring(sdf, encoding="unicode")
    pretty = minidom.parseString(raw).toprettyxml(indent="  ")
    body = "\n".join(line for line in pretty.splitlines()[1:] if line.strip())
    return (
        "<?xml version='1.0' encoding='utf-8'?>\n"
        "<!--\n"
        "  GENERATED FILE — DO NOT EDIT BY HAND.\n"
        "  Source:    configs/world.yaml\n"
        "  Generator: scripts/generate_world.py\n"
        "  Regenerate with: python3 scripts/generate_world.py\n"
        "-->\n" + body + "\n"
    )


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-c", "--config", type=Path, default=DEFAULT_CONFIG)
    ap.add_argument("-o", "--output", type=Path, default=DEFAULT_OUTPUT)
    ap.add_argument("--check", action="store_true",
                    help="exit non-zero if the output file is missing or stale")
    args = ap.parse_args(argv)

    cfg = yaml.safe_load(args.config.read_text())
    text = render(build_world(cfg))

    if args.check:
        if not args.output.exists():
            print(f"MISSING: {args.output}", file=sys.stderr)
            return 1
        if args.output.read_text() != text:
            print(f"STALE: {args.output} does not match {args.config}", file=sys.stderr)
            return 1
        print(f"up to date: {args.output.relative_to(REPO_ROOT)}")
        return 0

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(text)
    n_walls = len(cfg["world"]["walls"])
    n_obs = len(cfg["world"]["obstacles"])
    print(f"wrote {args.output.relative_to(REPO_ROOT)} "
          f"({n_walls} walls, {n_obs} obstacles, {len(text.splitlines())} lines)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
