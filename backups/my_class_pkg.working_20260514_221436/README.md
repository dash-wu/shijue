# my_class_pkg

ROS 课堂示例包：**动态重配置（`dynamic_reconfigure`）** 与 **按动态参数发布 `cmd_vel` 线速度**，并含日志、巡线、手势、AprilTag 跟随与 **逆解抓取（`tag_grab_node`）** 等脚本/节点。

**六个实验的完整启动指令（逐项可复制）见仓库根目录 [`../README.md`](../README.md)；含 **实验六**（三任务点导航 + AprilTag 语音 + 可选键盘遥控）。**

> **完整「参数与动态参数」练习**（`ros_param`、`ros_param.py`、`parameter.launch`、`dynamic_reconfigure_node` 等）在独立工作空间  
> **`~/workspace/ex7pcyy/repo`**，请参阅该目录下的 `README.md`。

## 目录结构（本包实际内容）

```text
my_class_pkg/
├── CMakeLists.txt
├── package.xml
├── cfg/
│   └── Tutorials.cfg          # 动态参数定义（生成 TutorialsConfig）
├── launch/
│   └── robot_dynamic_speed.launch
├── scripts/
│   ├── ros_log.py
│   ├── follow_line.py
│   ├── get_ros_image.py
│   ├── upros_gesture.py
│   ├── gesture_movement.py
│   ├── apriltag_follow.py
│   └── start_robot_speed_demo.sh
└── src/
    ├── ros_log.cpp
    ├── ros_dynamic_speed.cpp
    └── tag_grab.cpp           # 编译为 tag_grab_node
```

## 编译

在任意已将该包放入 `src/` 的 catkin 工作空间下：

```bash
cd ~/ros_class_ws   # 或你的 ws 路径
catkin_make
source devel/setup.bash
```

## `ros_dynamic_speed_node` 行为

- 订阅 **动态重配置** 服务端，将配置项 **`double_param`** 作为线速度 `linear.x`。
- 向 **`cmd_vel_topic`** 发布 `geometry_msgs/Twist`（默认话题 `/cmd_vel`；可通过 ROS 参数或 launch 参数覆盖）。
- 默认 `robot_dynamic_speed.launch` 会尝试启动 W2A 底盘相关的 `bringup_w2a.launch`（包名 `upros_bringup`），并可选启动 `rqt_reconfigure`。

### 小乌龟联调示例（不启动底盘时）

```bash
roscore
rosrun turtlesim turtlesim_node
rosrun my_class_pkg ros_dynamic_speed_node _cmd_vel_topic:=/turtle1/cmd_vel
rosrun rqt_reconfigure rqt_reconfigure
```

在 `rqt_reconfigure` 中调节 **`double_param`** 观察小乌龟线速度变化（需在 `Tutorials.cfg` 给定范围内）。

### 与 launch 文件

```bash
roslaunch my_class_pkg robot_dynamic_speed.launch
# 或
roslaunch my_class_pkg robot_dynamic_speed.launch cmd_vel_topic:=/你的话题
```

## 其他可执行目标

| 目标 | 说明 |
|------|------|
| `ros_log` | C++ 日志示例 |
| `ros_log.py` | Python 日志示例 |
| `follow_line.py` | 巡线 |
| `get_ros_image.py` | 取图示例 |
| `upros_gesture.py` / `gesture_movement.py` | 手势识别与机械臂联动 |
| `apriltag_follow.py` | AprilTag 跟随 |
| `tag_grab_node` | AprilTag 位姿 + 逆解抓取（C++） |

## 与 `~/ros_class_ws` 的关系

若 `src/my_class_pkg` 为本目录的符号链接，在工作空间根目录执行 `catkin_make` 即编译本包。
