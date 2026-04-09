# shijue（视觉与机械臂课程代码）

本仓库为智行 W2A 相关实验代码，核心包为 [`my_class_pkg`](my_class_pkg/)。以下为 **五个实验的完整启动方式**（每个实验均包含环境 `source` 与所需终端中的全部命令）。

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

## 安全提示

- 实验二、四涉及 **底盘运动**；实验三、五涉及 **机械臂与夹爪**。运行前 Clear 场地，并在有人看管下操作。
- 若 `roslaunch`/`rosrun` 报找不到包，请确认已按顺序 `source` 三个 setup，且 **`ros_class_ws` 使用 `--extend` 叠在 `upros_class_code` 之上。

## 仓库与链接

- GitHub：<https://github.com/dash-wu/shijue>
