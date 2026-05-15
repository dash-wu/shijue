#!/usr/bin/env bash
# 实验六 - 终端 A：直接运行本脚本即可，无需再 source 环境
set -euo pipefail
_ROS="/opt/ros/noetic/setup.bash"
_UPROS="${UPROS_WS:-$HOME/upros_class_code/devel/setup.bash}"
_CLASS="${CLASS_WS:-$HOME/ros_class_ws/devel/setup.bash}"
for _f in "$_ROS" "$_UPROS" "$_CLASS"; do [[ -f "$_f" ]] || { echo "[exp6] 找不到: $_f"; exit 1; }; done
# shellcheck source=/dev/null
source "$_ROS"
# shellcheck source=/dev/null
source "$_UPROS"
# shellcheck source=/dev/null
source "$_CLASS" --extend
exec roslaunch upros_bringup bringup_w2a.launch
