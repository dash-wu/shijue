#!/usr/bin/env bash
# 语音离线识别（upros_chat）+ Kimi 大模型（my_class_pkg）联调启动脚本
# 使用前请按需修改下面两个工作空间路径。

set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
WS_UROS="${WS_UROS:-/home/bcsh/upros_class_code}"
WS_CLASS="${WS_CLASS:-/home/bcsh/ros_class_ws}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"
# 顺序：先 upros，再 ros_class_ws；课堂工作空间必须用 --extend，否则会丢失 upros_chat
source "${WS_UROS}/devel/setup.bash"
source "${WS_CLASS}/devel/setup.bash" --extend

usage() {
  cat <<'EOF'
用法（命令之间必须有空格）:
  ./start_voice_llm.sh env      # 只打印下面两条 export/source，便于粘贴到手写教程里
  ./start_voice_llm.sh speech   # 终端1：仅启动语音识别
  ./start_voice_llm.sh llm      # 终端2：仅启动大模型（需 speech 或已有 roscore）
  ./start_voice_llm.sh all      # 单终端：后台 speech + 前台 llm（演示用）
  ./start_voice_llm.sh echo     # 可选终端3：监听识别结果 rostopic echo /speech/result

首次使用前:
  1) 在 my_class_pkg/scripts/ 下放置 moonshot_api_key.txt（见实验手册）
  2) 在工作空间执行: cd <ros_class_ws> && catkin_make && source devel/setup.bash
EOF
}

case "${1:-}" in
  env)
    cat <<EOF
# 复制到每个要跑 ROS 的终端最前面（顺序勿改）：
export ROS_DISTRO=${ROS_DISTRO}
source /opt/ros/\${ROS_DISTRO}/setup.bash
source ${WS_UROS}/devel/setup.bash
source ${WS_CLASS}/devel/setup.bash --extend

# 终端1 — 语音识别：
roslaunch upros_chat speech_to_word.launch

# 终端2 — Kimi 大模型（与语音联动须加 --ros）：
rosrun my_class_pkg llm_chat.py --ros

# 可选终端3 — 查看识别字符串：
rostopic echo /speech/result
EOF
    ;;
  speech)
    exec roslaunch upros_chat speech_to_word.launch
    ;;
  llm)
    exec rosrun my_class_pkg llm_chat.py --ros
    ;;
  echo)
    exec rostopic echo /speech/result
    ;;
  all)
    KEY_FILE="${WS_CLASS}/src/my_class_pkg/scripts/moonshot_api_key.txt"
    if [[ ! -s "${KEY_FILE}" ]]; then
      echo "缺少密钥文件: ${KEY_FILE}"
      echo "请先创建 moonshot_api_key.txt 并写入 Moonshot API Key（一行，无多余空格）。"
      exit 1
    fi
    roslaunch upros_chat speech_to_word.launch &
    _SPID=$!
    cleanup() { kill "${_SPID}" 2>/dev/null || true; }
    trap cleanup EXIT INT TERM
    sleep 4
    rosrun my_class_pkg llm_chat.py --ros
    ;;
  -h|--help|help|"")
    usage
    ;;
  *)
    echo "未知子命令: $1"
    usage
    exit 1
    ;;
esac
