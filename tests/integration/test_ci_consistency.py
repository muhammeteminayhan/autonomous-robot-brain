"""Keep the container and CI from drifting apart on dependencies.

The Dockerfile and the GitHub Actions workflow install the same packages, and
they used to each carry their own copy of the list. That is the duplication
`configs/` exists to prevent everywhere else in this project: CI would keep
passing against a dependency set the container no longer had, and the
divergence would surface only for whoever tried to reproduce `results/` inside
the container -- the one person it matters most to.

Both now read `docker/apt-packages.txt`. These tests check that they still do.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
PACKAGES = REPO_ROOT / "docker" / "apt-packages.txt"
DOCKERFILE = REPO_ROOT / "docker" / "Dockerfile"
WORKFLOW = REPO_ROOT / ".github" / "workflows" / "ci.yml"


def parse_packages() -> list[str]:
    out = []
    for line in PACKAGES.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if line:
            out.append(line)
    return out


def test_the_package_list_exists_and_parses():
    assert PACKAGES.is_file(), f"{PACKAGES} missing"
    pkgs = parse_packages()
    assert len(pkgs) > 10, f"only {len(pkgs)} packages; the list looks truncated"
    # A stray shell fragment in the list would be passed to apt and fail the
    # build with a message about a package nobody ever named.
    for p in pkgs:
        assert re.fullmatch(r"[a-z0-9][a-z0-9+.-]*", p), f"not a package name: {p!r}"


def test_the_essentials_are_present():
    """A spot check, so a truncated list fails here rather than in a 20-minute job."""
    pkgs = set(parse_packages())
    for required in ("build-essential", "cmake", "libeigen3-dev",
                     "python3-colcon-common-extensions", "ros-jazzy-ros-gz",
                     "ros-jazzy-xacro", "python3-pytest", "python3-yaml"):
        assert required in pkgs, f"{required} missing from {PACKAGES.name}"


def test_gz_python_bindings_are_declared():
    """moving_obstacle.py imports gz.transport and gz.msgs directly.

    They arrive transitively via ros-gz, and a transitive dependency is one
    refactor in someone else's package away from disappearing -- the failure
    would be an ImportError in the one experiment that spawns the obstacle,
    long after the build and the tests have gone green.
    """
    pkgs = set(parse_packages())
    assert "ros-jazzy-gz-transport-vendor" in pkgs
    assert "ros-jazzy-gz-msgs-vendor" in pkgs


@pytest.mark.parametrize("path", [DOCKERFILE, WORKFLOW],
                         ids=lambda p: p.name)
def test_neither_consumer_inlines_its_own_package_list(path: Path):
    """Both must READ the shared file, not re-list the packages themselves."""
    text = path.read_text()
    assert "apt-packages.txt" in text, (
        f"{path.name} no longer reads docker/apt-packages.txt")
    # An inlined list shows up as several `ros-jazzy-*` or `python3-*` names
    # sitting in the file itself. A couple of mentions in prose is fine.
    inlined = re.findall(r"^\s*(?:ros-jazzy-|python3-)[a-z0-9+.-]+\s*\\?\s*$",
                         text, re.M)
    assert len(inlined) < 3, (
        f"{path.name} looks like it inlines a package list again: {inlined[:5]}")


def test_workflow_is_valid_yaml_and_runs_the_three_suites():
    spec = yaml.safe_load(WORKFLOW.read_text())
    # PyYAML parses the bare key `on:` as the boolean True, which is a YAML 1.1
    # quirk and not a mistake in the workflow.
    triggers = spec.get("on", spec.get(True))
    assert triggers, "workflow has no triggers"
    steps = spec["jobs"]["build-and-test"]["steps"]
    run_text = "\n".join(s.get("run", "") for s in steps)
    assert "colcon build" in run_text
    assert "colcon test" in run_text
    assert "tests/integration" in run_text, (
        "CI builds and unit-tests but never runs the integration suite")
