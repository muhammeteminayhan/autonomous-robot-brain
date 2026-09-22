#!/usr/bin/env bash
# Source the ROS 2 underlay, then the project overlay if it has been built.
set -e
source /opt/ros/jazzy/setup.bash
if [ -f /workspace/ros2_ws/install/setup.bash ]; then
  source /workspace/ros2_ws/install/setup.bash
fi
exec "$@"
