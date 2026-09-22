#!/usr/bin/env bash
# Run every unit test in the workspace and print a per-test summary.
# Exits non-zero on any failure so CI gates correctly.
# `set -u` is deliberately omitted: ROS 2 setup.bash dereferences
# AMENT_TRACE_SETUP_FILES before defining it and would abort the script.
set -eo pipefail

ROS_DISTRO_NAME="jazzy"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
cd "${REPO_ROOT}/ros2_ws"

colcon test --event-handlers console_direct+ "$@"

echo
echo "================ TEST RESULTS ================"
colcon test-result --verbose
