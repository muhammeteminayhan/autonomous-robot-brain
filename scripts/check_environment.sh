#!/usr/bin/env bash
# Verify that every tool the project needs is present, and report the versions.
# Exits non-zero if anything required is missing, so CI can gate on it.
# NOTE: no `set -u` here. /opt/ros/<distro>/setup.bash dereferences
# AMENT_TRACE_SETUP_FILES and COLCON_TRACE before defining them, so sourcing it
# under `set -u` aborts the script with "unbound variable". Every ROS 2 setup
# script has this property -- sourcing them always needs nounset off.
set -o pipefail

REQUIRED_ROS_DISTRO="jazzy"
missing=0
warn=0

blue()  { printf '\033[1;34m%s\033[0m\n' "$*"; }
ok()    { printf '  \033[0;32m[ OK ]\033[0m %-22s %s\n' "$1" "${2:-}"; }
bad()   { printf '  \033[0;31m[FAIL]\033[0m %-22s %s\n' "$1" "${2:-}"; missing=$((missing+1)); }
soft()  { printf '  \033[0;33m[WARN]\033[0m %-22s %s\n' "$1" "${2:-}"; warn=$((warn+1)); }

need() {  # need <name> <command> <version-cmd>
  if command -v "$2" >/dev/null 2>&1; then
    ok "$1" "$(eval "$3" 2>/dev/null | head -1)"
  else
    bad "$1" "not found -- see README Installation"
  fi
}

blue "== Operating system =="
if [ -r /etc/os-release ]; then
  . /etc/os-release
  if [ "${VERSION_ID:-}" = "24.04" ]; then
    ok "Ubuntu" "$PRETTY_NAME"
  else
    soft "Ubuntu" "${PRETTY_NAME:-unknown} (project is validated on 24.04)"
  fi
else
  soft "OS" "cannot read /etc/os-release"
fi

blue "== Toolchain =="
need "C++ compiler"  g++     "g++ --version"
need "CMake"         cmake   "cmake --version"
need "git"           git     "git --version"
need "Python 3"      python3 "python3 --version"

blue "== ROS 2 =="
if [ -d "/opt/ros/${REQUIRED_ROS_DISTRO}" ]; then
  ok "ROS 2 ${REQUIRED_ROS_DISTRO}" "/opt/ros/${REQUIRED_ROS_DISTRO}"
  # shellcheck disable=SC1090
  set +u
  source "/opt/ros/${REQUIRED_ROS_DISTRO}/setup.bash"
  set +u
else
  bad "ROS 2 ${REQUIRED_ROS_DISTRO}" "/opt/ros/${REQUIRED_ROS_DISTRO} missing"
fi
need "ros2 CLI" ros2   "ros2 --help >/dev/null && echo available"
need "colcon"   colcon "colcon version-check 2>/dev/null | head -1 || echo installed"

blue "== Gazebo =="
if command -v gz >/dev/null 2>&1; then
  ok "Gazebo (gz)" "$(gz sim --version 2>/dev/null | head -1)"
else
  bad "Gazebo (gz)" "not found -- expected Gazebo Harmonic"
fi
if [ -d "/opt/ros/${REQUIRED_ROS_DISTRO}/share/ros_gz_sim" ]; then
  ok "ros_gz bridge" "installed"
else
  bad "ros_gz bridge" "ros-${REQUIRED_ROS_DISTRO}-ros-gz missing"
fi

blue "== C++ libraries =="
if [ -d /usr/include/eigen3 ]; then
  ver=$(grep -h -E '#define EIGEN_(WORLD|MAJOR|MINOR)_VERSION' \
        /usr/include/eigen3/Eigen/src/Core/util/Macros.h 2>/dev/null \
        | awk '{print $3}' | paste -sd. -)
  ok "Eigen3" "${ver:-present}"
else
  bad "Eigen3" "libeigen3-dev missing"
fi

blue "== Python packages =="
for mod in numpy scipy matplotlib pytest; do
  if python3 -c "import ${mod}" 2>/dev/null; then
    ok "$mod" "$(python3 -c "import ${mod};print(getattr(${mod},'__version__','?'))" 2>/dev/null)"
  else
    bad "$mod" "not importable"
  fi
done

blue "== Workspace path =="
# ROS 2's rosidl CMake code splits paths on non-ASCII bytes, so a workspace at
# e.g. /home/user/Masaüstü/... fails to build with a confusing
# "Target dependency '/home/user/Masa' does not exist". Catch it here instead.
REPO_ROOT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
if printf '%s' "${REPO_ROOT_PATH}" | LC_ALL=C grep -q '[^ -~]'; then
  bad "Workspace path" "non-ASCII characters in ${REPO_ROOT_PATH} -- rosidl will fail"
else
  ok "Workspace path" "ASCII-clean: ${REPO_ROOT_PATH}"
fi

blue "== Graphics =="
[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ] && ok "Display" "DISPLAY=${DISPLAY:-none} WAYLAND=${WAYLAND_DISPLAY:-none}" \
                                          || soft "Display" "no display -- GUI (Gazebo/RViz) will not open"

echo
if [ "$missing" -eq 0 ]; then
  printf '\033[0;32mEnvironment OK\033[0m (%d warning(s))\n' "$warn"
  exit 0
else
  printf '\033[0;31m%d required component(s) missing.\033[0m See README "Installation".\n' "$missing"
  exit 1
fi
