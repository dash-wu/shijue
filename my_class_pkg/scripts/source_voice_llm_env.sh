#!/usr/bin/env bash
# 同时加载 upros_chat + my_class_pkg。ros_class_ws 必须用 --extend，否则会覆盖 upros 包路径，
# 导致 Resource not found: upros_chat。
ROS_DISTRO="${ROS_DISTRO:-noetic}"
WS_UROS="${WS_UROS:-/home/bcsh/upros_class_code}"
WS_CLASS="${WS_CLASS:-/home/bcsh/ros_class_ws}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WS_UROS}/devel/setup.bash"
source "${WS_CLASS}/devel/setup.bash" --extend
