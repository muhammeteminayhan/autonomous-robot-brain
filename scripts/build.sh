#!/usr/bin/env bash
# Build the workspace. Defaults to Release; pass `debug` for a debug build.
# `set -u` is deliberately omitted: ROS 2 setup.bash dereferences
# AMENT_TRACE_SETUP_FILES before defining it and would abort the script.
set -eo pipefail

ROS_DISTRO_NAME="jazzy"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_TYPE="Release"
[ "${1:-}" = "debug" ] && BUILD_TYPE="Debug"

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
cd "${REPO_ROOT}/ros2_ws"

colcon build \
  --symlink-install \
  --cmake-args "-DCMAKE_BUILD_TYPE=${BUILD_TYPE}" \
               "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON" \
  --event-handlers console_cohesion+

# Expose compile_commands.json at the workspace root for clangd / IDEs.
if [ -f build/compile_commands.json ]; then
  ln -sf build/compile_commands.json compile_commands.json
fi

echo
echo "Build finished (${BUILD_TYPE}). Activate with:"
echo "  source ${REPO_ROOT}/ros2_ws/install/setup.bash"
