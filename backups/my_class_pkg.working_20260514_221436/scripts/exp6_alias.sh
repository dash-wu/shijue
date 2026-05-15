# 实验六：缩短手打 —— 先在本机执行一次下面「加载」再打开多个终端。
#   source /path/to/exp6_alias.sh
# 或写入 ~/.bashrc：
#   source $HOME/ros_class_ws/src/shijue/my_class_pkg/scripts/exp6_alias.sh
# 之后每个新终端只输入一行：  s6

s6() {
    # shellcheck source=/dev/null
    . /opt/ros/noetic/setup.bash
    # shellcheck source=/dev/null
    . "${UPROS_WS:-$HOME/upros_class_code/devel/setup.bash}"
    # shellcheck source=/dev/null
    . "${CLASS_WS:-$HOME/ros_class_ws/devel/setup.bash}" --extend
    echo "[s6] 环境已就绪，可运行 rosrun my_class_pkg exp6_*.sh"
}
