#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

if [ -f /opt/ros/noetic/setup.bash ]; then
  # Load the base ROS environment first.
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
fi

if [ ! -f "${WORKSPACE_DIR}/devel/setup.bash" ]; then
  echo "Workspace setup not found: ${WORKSPACE_DIR}/devel/setup.bash"
  echo "Please build the workspace first with: catkin_make"
  exit 1
fi

# shellcheck disable=SC1091
source "${WORKSPACE_DIR}/devel/setup.bash"

exec roslaunch my_class_pkg robot_dynamic_speed.launch "$@"
