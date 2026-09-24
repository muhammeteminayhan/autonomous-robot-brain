"""Enforce the hard architectural rule: ground truth never enters the estimator.

Two layers of defence:

* :func:`test_no_estimation_source_references_ground_truth` is a *static* check.
  It greps the source of every estimation package for any ground-truth topic or
  frame name. It runs without ROS installed, runs in milliseconds, and catches
  the violation at review time.

* :func:`test_running_graph_has_no_ground_truth_subscribers` is a *runtime*
  check. It inspects the live node graph and fails if any estimation node has
  actually subscribed to a ground-truth topic. It catches indirect leaks a grep
  cannot see -- a topic name built by string concatenation, or a remap in a
  launch file. It is skipped when ROS 2 is not available so the static check
  still runs in a bare CI container.

Why this is worth a dedicated test: an estimator that secretly reads ground
truth produces a flawless trajectory plot and a meaningless result. The bug is
silent, and every downstream number in ``results/`` inherits it.
"""

from __future__ import annotations

import re
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "ros2_ws" / "src"

#: Packages that form the live estimation/decision pipeline. ``arb_evaluation``
#: is deliberately absent -- it is *supposed* to read ground truth, and it is
#: also what publishes the fault state.
ESTIMATION_PACKAGES = (
    "arb_sensors",
    "arb_localization",
    "arb_mapping",
    "arb_planning",
    "arb_control",
)

#: Anything matching these is ground truth, however it is spelled.
FORBIDDEN_PATTERNS = (
    r"ground[_\-]?truth",
    r"/gt[_/]",
    r"\bgt_pose\b",
    r"\btrue_pose\b",
    r"\bperfect_odom\b",
    # Phase 10's fault state is instrumentation, and privileged in exactly the
    # same way ground truth is: it says which sensor is currently lying. An
    # estimator that reads it could down-weight the bad sensor by being TOLD
    # rather than by noticing, which is a different and much easier problem
    # than the one this project claims to solve. arb_msgs/FaultState.msg states
    # the rule; this is what enforces it.
    #
    # Matched via `arb_msgs` or the topic name, NOT on "FaultState" alone: the
    # sensor nodes have their own local ImuFaultState / LidarFaultState /
    # EncoderFaultState structs holding the knob values they were configured
    # with, which is exactly what they are supposed to have.
    # `fault_?state`, not `FaultState`: ROS generates the C++ header as
    # arb_msgs/msg/fault_state.hpp, and re.IGNORECASE does not bridge an
    # underscore. The CamelCase-only pattern let that include through.
    r"arb_msgs.{0,20}fault_?state",
    r"/arb/fault",
)

SOURCE_SUFFIXES = {".cpp", ".hpp", ".h", ".cc", ".py", ".yaml", ".yml", ".xml", ".launch"}

#: Lines that merely *say* the rule are fine, so comments are skipped -- but
#: what counts as a comment DEPENDS ON THE LANGUAGE, and getting that wrong
#: opened a hole in this very check.
#:
#: A single list containing "#" treated every C++ `#include` as a comment, so
#: the static check had never once inspected an include line. An estimation
#: node could `#include "arb_msgs/msg/fault_state.hpp"` -- or any ground-truth
#: header -- and this test would pass. Verified by injecting exactly that.
_COMMENT_PREFIXES_BY_SUFFIX = {
    ".cpp": ("//", "*", "/*"),
    ".hpp": ("//", "*", "/*"),
    ".h": ("//", "*", "/*"),
    ".cc": ("//", "*", "/*"),
    ".xml": ("<!--", "-->"),
    ".launch": ("<!--", "-->"),
    ".py": ("#",),
    ".yaml": ("#",),
    ".yml": ("#",),
}


def _iter_source_files():
    for package in ESTIMATION_PACKAGES:
        package_dir = SRC_ROOT / package
        if not package_dir.is_dir():
            continue
        for path in package_dir.rglob("*"):
            if path.is_file() and path.suffix in SOURCE_SUFFIXES:
                yield package, path


def _is_comment(line: str, suffix: str) -> bool:
    stripped = line.strip()
    # An unknown suffix is treated as having NO comment syntax, so a new file
    # type is over-reported rather than silently exempted.
    prefixes = _COMMENT_PREFIXES_BY_SUFFIX.get(suffix, ())
    return any(stripped.startswith(p) for p in prefixes)


def test_comment_detection_is_language_aware():
    """A C++ `#include` is not a comment, whatever Python thinks of `#`.

    This is the property that a single shared prefix list got wrong, and the
    consequence was that the static check below never inspected an include
    line in any C++ file in the project.
    """
    assert _is_comment("// a note", ".cpp")
    assert not _is_comment('#include "arb_msgs/msg/fault_state.hpp"', ".cpp")
    assert not _is_comment('#include "x.hpp"', ".hpp")
    assert _is_comment("# a note", ".py")
    assert _is_comment("# a note", ".yaml")
    assert not _is_comment("topic: /arb/ground_truth/odom", ".yaml")
    assert _is_comment("<!-- a note -->", ".xml")
    # An unrecognised suffix must not be silently exempt.
    assert not _is_comment("# anything", ".txt")


def test_estimation_packages_exist():
    """Guard against the static check silently passing because paths moved."""
    missing = [p for p in ESTIMATION_PACKAGES if not (SRC_ROOT / p).is_dir()]
    assert not missing, f"estimation packages not found under {SRC_ROOT}: {missing}"


def test_no_estimation_source_references_ground_truth():
    """No estimation source file may name a ground-truth topic or frame."""
    compiled = [re.compile(p, re.IGNORECASE) for p in FORBIDDEN_PATTERNS]
    violations: list[str] = []

    for package, path in _iter_source_files():
        try:
            text = path.read_text(encoding="utf-8", errors="replace")
        except OSError:  # pragma: no cover - unreadable file is a real problem
            continue
        for lineno, line in enumerate(text.splitlines(), start=1):
            if _is_comment(line, path.suffix):
                continue
            for pattern in compiled:
                if pattern.search(line):
                    rel = path.relative_to(REPO_ROOT)
                    violations.append(f"{package}: {rel}:{lineno}: {line.strip()}")

    assert not violations, (
        "Ground truth referenced inside the estimation pipeline.\n"
        "Ground truth is evaluation-only (see docs/architecture.md section 4).\n"
        + "\n".join(violations)
    )


def _ros2(*args: str) -> str:
    return subprocess.run(
        ["ros2", *args], capture_output=True, text=True, timeout=30, check=True
    ).stdout


@pytest.mark.integration
def test_running_graph_has_no_ground_truth_subscribers():
    """Inspect the live graph; skip cleanly when nothing is running.

    Covers the fault state as well as ground truth: both are privileged
    knowledge that no estimator is allowed to have.
    """
    try:
        node_list = _ros2("node", "list").split()
    except (FileNotFoundError, subprocess.SubprocessError):
        pytest.skip("ROS 2 not available or no daemon running")

    estimation_nodes = [
        n for n in node_list
        if any(pkg.replace("arb_", "") in n for pkg in ESTIMATION_PACKAGES)
    ]
    if not estimation_nodes:
        pytest.skip("no estimation nodes running; start the stack first")

    compiled = [re.compile(p, re.IGNORECASE) for p in FORBIDDEN_PATTERNS]
    violations: list[str] = []

    for node in estimation_nodes:
        info = _ros2("node", "info", node)
        # Only the "Subscribers:" block matters; publishing is harmless.
        in_subs = False
        for line in info.splitlines():
            if line.strip().endswith("Subscribers:"):
                in_subs = True
                continue
            if line.strip().endswith(("Publishers:", "Service Servers:",
                                      "Service Clients:", "Action Servers:",
                                      "Action Clients:")):
                in_subs = False
                continue
            if in_subs and any(p.search(line) for p in compiled):
                violations.append(f"{node} subscribes to {line.strip()}")

    assert not violations, (
        "Live estimation node subscribed to ground truth:\n" + "\n".join(violations)
    )
