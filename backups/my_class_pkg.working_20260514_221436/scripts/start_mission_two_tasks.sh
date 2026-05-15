#!/usr/bin/env bash
# 与仓库 README「实验六」一键方式一致：bringup 另开终端后执行本脚本。
# 换地图：MAP_NAME=xxx.yaml bash 本脚本
set -euo pipefail
ROS_DISTRO_SRC="/opt/ros/noetic/setup.bash"
UPROS_WS="${UPROS_WS:-$HOME/upros_class_code/devel/setup.bash}"
CLASS_WS="${CLASS_WS:-$HOME/ros_class_ws/devel/setup.bash}"
MAP_NAME="${MAP_NAME:-my_lab.yaml}"

[[ -f "$UPROS_WS" ]] || { echo "[错误] 找不到 $UPROS_WS"; exit 1; }
[[ -f "$CLASS_WS" ]] || { echo "[错误] 找不到 $CLASS_WS"; exit 1; }

# shellcheck source=/dev/null
source "$ROS_DISTRO_SRC"
# shellcheck source=/dev/null
source "$UPROS_WS"
# shellcheck source=/dev/null
source "$CLASS_WS" --extend

exec roslaunch my_class_pkg mission_four_points.launch "map_name:=${MAP_NAME}"
