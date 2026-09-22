#!/usr/bin/env bash
# Run every unit test in the workspace and print a per-test summary.
# Exits non-zero on any failure so CI gates correctly.
# `set -u` is deliberately omitted: ROS 2 setup.bash dereferences
# AMENT_TRACE_SETUP_FILES before defining it and would abort the script.
set -eo pipefail

ROS_DISTRO_NAME="jazzy"
# `pwd -P` resolves symlinks. This matters: if the repo is reached through a
# symlink whose path contains non-ASCII characters (e.g. the Turkish Desktop
# "Masaüstü"), colcon inherits that path and ROS 2's rosidl generator silently
# truncates it, producing "Target dependency '/home/user/Masa' does not exist".
# Resolving to the real path here makes the build work from either location.
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"

if printf '%s' "${REPO_ROOT}" | LC_ALL=C grep -q '[^ -~]'; then
  echo "ERROR: workspace path contains non-ASCII characters:" >&2
  echo "         ${REPO_ROOT}" >&2
  echo "       ROS 2's rosidl generator cannot handle this. Move the repo to" >&2
  echo "       an ASCII-only path (see README, 'Known environment pitfalls')." >&2
  exit 1
fi

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
cd "${REPO_ROOT}/ros2_ws"

colcon test --event-handlers console_direct+ "$@"

echo
echo "================ TEST RESULTS ================"
colcon test-result --verbose
