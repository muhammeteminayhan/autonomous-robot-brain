"""Every directory a package installs must exist in a fresh clone.

CMake's `install(DIRECTORY x ...)` fails the whole build when `x` is missing,
and git does not track empty directories. The combination is a build that
works on the machine where the directory happens to exist and fails for
everybody else -- including CI, which is where this was finally caught, on its
first run, weeks after the directory stopped having anything in it.

`arb_description/CMakeLists.txt` installed a `launch/` that was empty and
therefore untracked. A fresh clone could not build the project at all.
"""

from __future__ import annotations

import re
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "ros2_ws" / "src"

#: `install(DIRECTORY a b c DESTINATION ...)` -- capture everything between.
INSTALL_DIRECTORY = re.compile(
    r"install\s*\(\s*DIRECTORY\s+(.*?)\s*DESTINATION", re.S | re.I)


def tracked_files() -> set[str]:
    out = subprocess.run(["git", "ls-files"], cwd=REPO_ROOT,
                         capture_output=True, text=True, check=True).stdout
    return set(out.split())


def cmake_files() -> list[Path]:
    return sorted(SRC_ROOT.rglob("CMakeLists.txt"))


@pytest.mark.parametrize("cmake", cmake_files(),
                         ids=lambda p: p.parent.name)
def test_installed_directories_are_tracked_by_git(cmake: Path):
    """A directory with no tracked file in it does not survive a clone."""
    tracked = tracked_files()
    package = cmake.parent
    missing: list[str] = []
    for block in INSTALL_DIRECTORY.findall(cmake.read_text()):
        for token in block.split():
            # Skip CMake variables and keywords; only bare relative names are
            # directories this check can reason about.
            if token.startswith("$") or token.upper() == token and token.isalpha():
                continue
            rel = (package / token.rstrip("/")).relative_to(REPO_ROOT)
            if not any(t.startswith(f"{rel}/") for t in tracked):
                local = "exists locally but is empty" \
                    if (REPO_ROOT / rel).is_dir() else "does not exist"
                missing.append(f"{rel} ({local})")
    assert not missing, (
        f"{cmake.relative_to(REPO_ROOT)} installs directories that a fresh "
        f"clone will not have:\n  " + "\n  ".join(missing))


def test_the_check_covers_every_package():
    """Guard against the parametrisation silently finding nothing."""
    assert len(cmake_files()) >= 8, (
        f"only {len(cmake_files())} CMakeLists.txt found under {SRC_ROOT}")
