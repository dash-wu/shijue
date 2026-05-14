#!/usr/bin/env bash
# 实验六 - 单入口：bash exp6_run.sh <a|b|c|d|e|one>（无需事先 source）
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

usage() {
    echo "用法: $0 <a|b|c|d|e|one>" >&2
    exit 1
}

cmd="${1:-}"
case "$cmd" in
    a|A|1) exec roslaunch upros_bringup bringup_w2a.launch ;;
    b|B|2) MAP_NAME="${MAP_NAME:-my_lab.yaml}"; exec roslaunch upros_navigation navigation.launch "map_name:=${MAP_NAME}" ;;
    c|C|3)
        [[ "${RVIZ_USE_SW_GL:-}" == "1" ]] && export LIBGL_ALWAYS_SOFTWARE=1
        exec roslaunch upros_navigation view_nav.launch
        ;;
    d|D|4) exec roslaunch my_class_pkg mission_four_points_node_only.launch ;;
    e|E|5) exec rosrun upros_move_linear teleop_twist_keyboard.py ;;
    one|ONE|all) MAP_NAME="${MAP_NAME:-my_lab.yaml}"; exec roslaunch my_class_pkg mission_four_points.launch "map_name:=${MAP_NAME}" ;;
    *) usage ;;
esac
