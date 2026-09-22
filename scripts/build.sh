#!/usr/bin/env bash
# Build the workspace. Defaults to Release; pass `debug` for a debug build.
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
BUILD_TYPE="Release"
[ "${1:-}" = "debug" ] && BUILD_TYPE="Debug"

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
cd "${REPO_ROOT}/ros2_ws"

# CMAKE_EXPORT_COMPILE_COMMANDS is passed via the ENVIRONMENT, not --cmake-args.
# As a -D flag it is "manually specified but unused" for every package that has
# no compiled target (pure install / interface packages), and CMake emits a
# warning on stderr for each one -- 8 spurious warnings in this workspace. As an
# environment variable CMake consumes it silently and still writes the file.
export CMAKE_EXPORT_COMPILE_COMMANDS=ON

colcon build \
  --symlink-install \
  --cmake-args "-DCMAKE_BUILD_TYPE=${BUILD_TYPE}" \
  --event-handlers console_cohesion+

# Expose compile_commands.json at the workspace root for clangd / IDEs.
if [ -f build/compile_commands.json ]; then
  ln -sf build/compile_commands.json compile_commands.json
fi

echo
echo "Build finished (${BUILD_TYPE}). Activate with:"
echo "  source ${REPO_ROOT}/ros2_ws/install/setup.bash"
