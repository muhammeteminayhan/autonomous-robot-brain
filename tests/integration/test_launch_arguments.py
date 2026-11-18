# Copyright (c) 2026 Emin Ayhan. MIT License.
"""Every `name:=value` a script passes to `ros2 launch` must be declared.

ros2 launch accepts an argument no launch file declares, discards it, and
exits zero. Nothing warns. The run proceeds on the YAML default and looks
exactly like a run that was configured.

That is not hypothetical: the navigation benchmark passed
`predict_obstacle_motion:=false` to build its baseline arm. The launch file
never declared it, so every "no prediction" run used prediction, and two
arms of the sweep were the same configuration measured twice. The numbers
were reported before anyone noticed, because there was nothing to notice.
"""
from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "ros2_ws" / "src"
SCRIPTS = sorted((REPO_ROOT / "scripts").glob("*.py"))


def _literal_prefix(node: ast.AST) -> str | None:
    """The leading literal text of a string or f-string argument.

    `f"predict_obstacle_motion:={x}"` is an f-string whose first piece is the
    part that matters -- the argument name.
    """
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    if isinstance(node, ast.JoinedStr):
        first = node.values[0] if node.values else None
        if isinstance(first, ast.Constant) and isinstance(first.value, str):
            return first.value
    return None


def launch_invocations() -> list[tuple[Path, str, str, str]]:
    """(script, package, launch file, argument name) for every call found."""
    found: list[tuple[Path, str, str, str]] = []
    for script in SCRIPTS:
        tree = ast.parse(script.read_text())
        for call in ast.walk(tree):
            if not isinstance(call, ast.Call):
                continue
            parts = [_literal_prefix(a) for a in call.args]
            if parts[:2] != ["ros2", "launch"] or len(parts) < 4:
                continue
            pkg, launch_file = parts[2], parts[3]
            if not pkg or not launch_file:
                continue
            for raw in parts[4:]:
                if raw and ":=" in raw:
                    found.append((script, pkg, launch_file,
                                  raw.split(":=", 1)[0]))
    return found


INVOCATIONS = launch_invocations()


def test_the_scan_found_something():
    """A regex that matches nothing passes every assertion below it."""
    assert INVOCATIONS, (
        "no `ros2 launch ... name:=value` calls were found in scripts/, so "
        "this file is asserting nothing. Either the scripts stopped "
        "launching things or the AST walk no longer recognises how they do "
        "it.")


@pytest.mark.parametrize(
    "script,pkg,launch_file,arg", INVOCATIONS,
    ids=[f"{s.name}:{p}:{a}" for s, p, _, a in INVOCATIONS])
def test_launch_argument_is_declared(script: Path, pkg: str,
                                     launch_file: str, arg: str):
    path = SRC_ROOT / pkg / "launch" / launch_file
    assert path.exists(), f"{script.name} launches {pkg}/{launch_file}, which does not exist"
    declared = set(re.findall(
        r"DeclareLaunchArgument\(\s*[\"']([A-Za-z0-9_]+)[\"']", path.read_text()))
    assert arg in declared, (
        f"{script.name} passes `{arg}:=...` to {pkg}/{launch_file}, which "
        f"does not declare it. ros2 launch will discard it silently and the "
        f"run will use the default. Declared there: {sorted(declared)}")
