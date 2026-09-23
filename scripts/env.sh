# Source this in every shell that talks to the robot.
#
#     source scripts/env.sh
#
# Not executable and with no shebang on purpose: running it in a subshell would
# set the environment of a process that then exits, which is the single most
# common way to spend ten minutes wondering why `ros2` cannot find a package.
#
# WHY `pwd -P`
# It resolves symlinks. If the repo is reached through a symlink whose path
# contains non-ASCII characters -- the Turkish Desktop is `Masaüstü` -- then
# `rosidl` truncates the path at the first non-ASCII byte and the build fails
# with "Target dependency '/home/<user>/Masa' does not exist". The guard below
# catches it before it can waste an afternoon; see README, "Known environment
# pitfalls".
#
# WHY `set -e` IS NOT USED
# ROS's own setup.bash dereferences AMENT_TRACE_SETUP_FILES while it may be
# unset, so a shell with `set -u` aborts halfway through sourcing it and leaves
# the environment half-configured, which is worse than not sourcing it at all.

_arb_env() {
  local repo
  repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"

  case "$repo" in
    *[![:ascii:]]*)
      echo "ERROR: repo path contains non-ASCII characters:" >&2
      echo "       $repo" >&2
      echo "       rosidl truncates such paths. Move the repo somewhere ASCII." >&2
      return 1
      ;;
  esac

  local distro="${ARB_ROS_DISTRO:-jazzy}"
  if [ ! -f "/opt/ros/$distro/setup.bash" ]; then
    echo "ERROR: /opt/ros/$distro/setup.bash not found." >&2
    echo "       Set ARB_ROS_DISTRO to your distro, or install ROS 2." >&2
    return 1
  fi
  # shellcheck disable=SC1090
  source "/opt/ros/$distro/setup.bash"

  if [ -f "$repo/ros2_ws/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$repo/ros2_ws/install/setup.bash"
  else
    echo "note: ros2_ws/install not found -- run ./scripts/build.sh first." >&2
  fi

  # Every pane, and every script, must agree on this or they will not see each
  # other's topics. It also keeps this robot off the default domain, so a
  # colleague on the same network does not receive its /cmd_vel.
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
  export ARB_ROOT="$repo"

  echo "ROS 2 $distro | domain $ROS_DOMAIN_ID | $repo"
}

_arb_env
unset -f _arb_env
