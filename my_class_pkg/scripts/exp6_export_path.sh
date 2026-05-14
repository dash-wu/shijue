#!/usr/bin/env bash
# 集成本包「scripts」路径为环境变量 SCR，供 exp6_a / exp6_b 等使用。
#
# 必须这样用（二选一）：
#   source /你的路径/my_class_pkg/scripts/exp6_export_path.sh
#   . /你的路径/my_class_pkg/scripts/exp6_export_path.sh
#
# 不要用 bash exp6_export_path.sh（子 shell 里 export 不会回到当前终端）。
#
# 编译安装后（已 source 工作空间 devel/setup.bash），也可：
#   source "$(rospack find my_class_pkg)/../../lib/my_class_pkg/exp6_export_path.sh"
#
if [[ -n "${BASH_SOURCE[0]:-}" ]]; then
  export SCR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
else
  export SCR="$(cd "$(dirname "$0")" && pwd)"
fi
echo "[exp6] SCR=$SCR"
