# shijue（视觉与机械臂课程代码）

本仓库为智行 W2A 相关实验代码，核心包为 [`my_class_pkg`](my_class_pkg/)。以下为 **课程实验与期末三任务点的完整启动方式**（每个实验均包含环境 `source` 与所需终端中的全部命令）。

---

## 环境与编译（先做一次）

将 `my_class_pkg` 放入 catkin 工作空间的 `src/`（例如通过符号链接指向本仓库目录），并确保能链到课程包 **`upros_class_code`**（含 `upros_bringup`、`upros_arm`、`upros_message` 等）。

**环境（每个新终端先执行）：**

```bash
source /opt/ros/noetic/setup.bash
source ~/upros_class_code/devel/setup.bash
source ~/ros_class_ws/devel/setup.bash --extend
```

**编译（修改 C++ 后需要）：**

```bash
cd ~/ros_class_ws
catkin_make
source ~/ros_class_ws/devel/setup.bash --extend
```

---

## 实验一：底盘与机械臂硬件通信（bringup）

本实验仅需要 **一个终端**。

**终端 A：**

```bash
source /opt/ros/noetic/setup.bash
source ~/upros_class_code/devel/setup.bash
source ~/ros_class_ws/devel/setup.bash --extend
roslaunch upros_bringup bringup_w2a.launch
```

---

## 实验二：巡线

需要先启动 **实验一**（`bringup_w2a.launch` 保持运行），再开新终端执行巡线节点。

**终端 A（硬件，占住终端）：**

```bash
source /opt/ros/noetic/setup.bash
source ~/upros_class_code/devel/setup.bash
source ~/ros_class_ws/devel/setup.bash --extend
roslaunch upros_bringup bringup_w2a.launch
```

**终端 B（巡线；须等待终端 A 已正常运行）：**

```bash
source /opt/ros/noetic/setup.bash
source ~/upros_class_code/devel/setup.bash
source ~/ros_class_ws/devel/setup.bash --extend
rosrun my_class_pkg follow_line.py
```

---

## 实验三：手势控制机械臂

需要先启动 **实验一**。手势识别与手势映射需 **两个独立终端**（可与 bringup 并行）。

**终端 A（硬件）：**

```bash
source /opt/ros/noetic/setup.bash
source ~/upros_class_code/devel/setup.bash
source ~/ros_class_ws/devel/setup.bash --extend
roslaunch upros_bringup bringup_w2a.launch
```

**终端 B（手势识别，摄像头/MediaPipe）：**

```bash
source /opt/ros/noetic/setup.bash
source ~/upros_class_code/devel/setup.bash
source ~/ros_class_ws/devel/setup.bash --extend
rosrun my_class_pkg upros_gesture.py
```

**终端 C（手势到机械臂/夹爪动作映射）：**

```bash
source /opt/ros/noetic/setup.bash
source ~/upros_class_code/devel/setup.bash
source ~/ros_class_ws/devel/setup.bash --extend
rosrun my_class_pkg gesture_movement.py
```

---

## 实验四：AprilTag 跟随

需要先启动 **实验一**。若课堂要求单独启动相机或识别 launch，请按讲义在下列命令前补充；以下为与当前包内脚本一致的跟随时启动方式。

**终端 A（硬件）：**

```bash
source /opt/ros/noetic/setup.bash
source ~/upros_class_code/devel/setup.bash
source ~/ros_class_ws/devel/setup.bash --extend
roslaunch upros_bringup bringup_w2a.launch
```

**终端 B（AprilTag 跟随节点；须等待终端 A 已正常运行）：**

```bash
source /opt/ros/noetic/setup.bash
source ~/upros_class_code/devel/setup.bash
source ~/ros_class_ws/devel/setup.bash --extend
rosrun my_class_pkg apriltag_follow.py
```

---

## 实验五：AprilTag 感知 + 逆解抓取（tag_grab）

需要先启动 **实验一**，再启动 **`upros_arm` 下的 AprilTag 识别 launch**，最后在 **TF 可用且有人看管机械臂** 时运行抓取节点。

**终端 A（硬件）：**

```bash
source /opt/ros/noetic/setup.bash
source ~/upros_class_code/devel/setup.bash
source ~/ros_class_ws/devel/setup.bash --extend
roslaunch upros_bringup bringup_w2a.launch
```

**终端 B（AprilTag 识别与 TF；须等待终端 A 已正常运行）：**

```bash
source /opt/ros/noetic/setup.bash
source ~/upros_class_code/devel/setup.bash
source ~/ros_class_ws/devel/setup.bash --extend
roslaunch upros_arm recognize_apriltag.launch
```

**终端 C（逆解抓取节点；请在相机稳定看到标签、且存在 `arm_base_link` → `tag_1` 的 TF 后再运行）：**

```bash
source /opt/ros/noetic/setup.bash
source ~/upros_class_code/devel/setup.bash
source ~/ros_class_ws/devel/setup.bash --extend
rosrun my_class_pkg tag_grab_node
```

---

## 实验六：地图三任务点（TASK1→TASK2→END）+ AprilTag 语音

与 **实验五** 相同，依赖 `upros_arm` 的 **`recognize_apriltag.launch`** 与 `upros_chat` 语音；本实验在 **`mission_four_points_node_only.launch` / `mission_four_points.launch` 内已 include**，无需再单独起一个识别终端。航点配置：[`my_class_pkg/config/mission_start_two_tasks_end.yaml`](my_class_pkg/config/mission_start_two_tasks_end.yaml)。

**推荐顺序**：**实验一 bringup** → **导航栈** → **RViz 2D Pose Estimate** → **航点任务**。

**终端 A（硬件）：** 同 **实验一**。

**终端 B（地图 + AMCL + move_base）：**

```bash
source /opt/ros/noetic/setup.bash
source ~/upros_class_code/devel/setup.bash
source ~/ros_class_ws/devel/setup.bash --extend
roslaunch upros_navigation navigation.launch map_name:=my_lab.yaml
```

**终端 C（RViz 初定位）：**

```bash
source /opt/ros/noetic/setup.bash
source ~/upros_class_code/devel/setup.bash
source ~/ros_class_ws/devel/setup.bash --extend
roslaunch upros_navigation view_nav.launch
```

**终端 D（Apriltag + 语音 + 三航点；与实验五一致先 source 三行，不重复起 B 中 navigation）：**

```bash
source /opt/ros/noetic/setup.bash
source ~/upros_class_code/devel/setup.bash
source ~/ros_class_ws/devel/setup.bash --extend
roslaunch my_class_pkg mission_four_points_node_only.launch
```

**终端 E（可选，键盘手动遥控）：** 需终端 A 已 bringup；与 `move_base` 同时运行会争抢 `/cmd_vel`，需要遥控时先停航点任务或暂时不要同时下发导航目标。

```bash
source /opt/ros/noetic/setup.bash
source ~/upros_class_code/devel/setup.bash
source ~/ros_class_ws/devel/setup.bash --extend
rosrun upros_move_linear teleop_twist_keyboard.py
```

**可选一键（单终端起导航 + 识别 + 航点，仍须终端 A 为 bringup）：**

```bash
source /opt/ros/noetic/setup.bash
source ~/upros_class_code/devel/setup.bash
source ~/ros_class_ws/devel/setup.bash --extend
roslaunch my_class_pkg mission_four_points.launch map_name:=my_lab.yaml
```

**同上「一键」的脚本写法**（bringup 仍须单独终端；可用 `UPROS_WS`、`CLASS_WS`、`MAP_NAME` 覆盖默认路径与地图）：

```bash
bash /path/to/shijue/my_class_pkg/scripts/start_mission_two_tasks.sh
```

**标记任务点坐标（与第六周 `tutorial_read_pose_map` 用法一致）：** bringup + navigation + RViz 初定位后，遥控到目标位停稳，执行：

```bash
bash ~/upros_class_code/scripts/tutorial_read_pose_map.sh
```

将输出的平移 x、y 与 RPY 中 yaw（度）写入 `mission_start_two_tasks_end.yaml` 对应航点。

---

## 安全提示

- 实验二、四、**六** 涉及 **底盘运动**；实验三、五涉及 **机械臂与夹爪**。运行前 Clear 场地，并在有人看管下操作。
- 若 `roslaunch`/`rosrun` 报找不到包，请确认已按顺序 `source` 三个 setup，且 **`ros_class_ws` 使用 `--extend` 叠在 `upros_class_code` 之上。

## 仓库与链接

- GitHub：<https://github.com/dash-wu/shijue>
