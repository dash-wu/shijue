# shellcheck shell=bash
# 由 exp6_*.sh 同目录 source；勿用「source 本文件」代替整套启动。

exp6_setup_env() {
    local ros_distro="${ROS_DISTRO:-/opt/ros/noetic/setup.bash}"
    local upros="${UPROS_WS:-$HOME/upros_class_code/devel/setup.bash}"
    local cls="${CLASS_WS:-$HOME/ros_class_ws/devel/setup.bash}"
    if [[ ! -f "$ros_distro" ]]; then
        echo "[exp6] 找不到 ROS: $ros_distro" >&2
        return 1
    fi
    if [[ ! -f "$upros" ]]; then
        echo "[exp6] 找不到 UPROS_WS=$upros （请 export UPROS_WS=你的 upros_class_code/devel/setup.bash）" >&2
        return 1
    fi
    if [[ ! -f "$cls" ]]; then
        echo "[exp6] 找不到 CLASS_WS=$cls （请 export CLASS_WS=你的 ros_class_ws/devel/setup.bash）" >&2
        return 1
    fi
    # shellcheck source=/dev/null
    source "$ros_distro"
    # shellcheck source=/dev/null
    source "$upros"
    # shellcheck source=/dev/null
    source "$cls" --extend
    return 0
}
