#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""期末综合实战航点节点（按第六周实验教程 5.4 风格写）。

流程：bringup（终端 A） → navigation.launch（终端 B） → view_nav.launch（终端 C）→ 本节点。

依次给 move_base 发 MoveBaseGoal（actionlib 阻塞等结果），到识别点时短时订阅
/tag_detections，识别/未识别都向 /talk 发布播报字符串：
  - 识别到：     "已找到目标"
  - 未识别到：    "未找到目标"

任务点和起点终点全部在 YAML 里配置（参考考核：起点→房间1入口→房间1识别→
房间2入口→房间2识别→返程中区→终点）。坐标按第六周实验教程 5.3 节示教：
  rosrun upros_transform tf_echo_node
读出 map→base_link 的 (X, Y, YAW°) 填进 YAML。
"""
from __future__ import annotations

import math
import sys
import threading
from typing import Dict, List, Set

import actionlib
import rospy
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf_conversions import transformations

try:
    from apriltag_ros.msg import AprilTagDetectionArray
except ImportError:
    AprilTagDetectionArray = None  # type: ignore


def call_clear_costmaps() -> bool:
    try:
        rospy.wait_for_service("/move_base/clear_costmaps", rospy.Duration(2.0))
        rospy.ServiceProxy("/move_base/clear_costmaps", Empty)()
        return True
    except Exception as ex:
        rospy.logwarn("clear_costmaps 失败: %s", ex)
        return False


def current_xy(tf_buf: tf2_ros.Buffer, map_f: str, base_f: str) -> "tuple[float, float] | None":
    try:
        t = tf_buf.lookup_transform(map_f, base_f, rospy.Time(0), rospy.Duration(0.4))
        return t.transform.translation.x, t.transform.translation.y
    except Exception:
        return None


def make_goal(map_frame: str, x: float, y: float, yaw_deg: float) -> MoveBaseGoal:
    g = MoveBaseGoal()
    g.target_pose.header.frame_id = map_frame
    g.target_pose.header.stamp = rospy.Time.now()
    g.target_pose.pose.position.x = float(x)
    g.target_pose.pose.position.y = float(y)
    yaw = math.radians(float(yaw_deg))
    q = transformations.quaternion_from_euler(0.0, 0.0, yaw)
    g.target_pose.pose.orientation.x = q[0]
    g.target_pose.pose.orientation.y = q[1]
    g.target_pose.pose.orientation.z = q[2]
    g.target_pose.pose.orientation.w = q[3]
    return g


def status_name(st: int) -> str:
    table = {
        GoalStatus.PENDING: "PENDING",
        GoalStatus.ACTIVE: "ACTIVE",
        GoalStatus.PREEMPTED: "PREEMPTED",
        GoalStatus.SUCCEEDED: "SUCCEEDED",
        GoalStatus.ABORTED: "ABORTED",
        GoalStatus.REJECTED: "REJECTED",
        GoalStatus.PREEMPTING: "PREEMPTING",
        GoalStatus.RECALLED: "RECALLED",
        GoalStatus.LOST: "LOST",
    }
    return table.get(st, "UNKNOWN(%d)" % st)


def detections_contain_id(msg, tid: int) -> bool:
    for d in msg.detections:
        for i in d.id:
            if int(i) == tid:
                return True
    return False


def main() -> None:
    rospy.init_node("goto_waypoints_named", anonymous=False)

    map_frame = str(rospy.get_param("~map_frame", "map"))
    wp: Dict = rospy.get_param("~waypoints")
    order: List[str] = list(rospy.get_param("~visit_order", []))

    tag_at = set(str(x) for x in rospy.get_param("~tag_check_at", []))
    tag_id = int(rospy.get_param("~tag_target_id", 1))
    tag_scan_sec = float(rospy.get_param("~tag_scan_duration_sec", 1.0))
    tag_topic = str(rospy.get_param("~tag_detections_topic", "/tag_detections"))

    talk_topic = str(rospy.get_param("~tts_talk_topic", "/talk"))
    tts_hit = str(rospy.get_param("~tts_hit", "已找到目标"))
    tts_miss = str(rospy.get_param("~tts_miss", "未找到目标"))
    tts_nonblocking = bool(rospy.get_param("~tts_nonblocking", True))
    tts_pause = float(rospy.get_param("~tts_pause_sec", 0.4))

    goal_timeout_sec = float(rospy.get_param("~goal_timeout_sec", 60.0))
    skip_first = bool(rospy.get_param("~skip_first_waypoint", True))
    base_frame = str(rospy.get_param("~base_frame", "base_footprint"))
    # 进入"区域成功"的判定半径：单点超时但车已经离目标很近时，按成功处理。
    arrive_tol_m = float(rospy.get_param("~arrive_tolerance_m", 0.55))
    # 单点最多重试次数（每次重试前清代价图）
    point_retries = int(rospy.get_param("~point_retries", 1))
    # 每个点开始前先清一次代价图，减少残留紫色膨胀挡路
    clear_before_each = bool(rospy.get_param("~clear_costmaps_before_each", True))

    startup_delay = float(rospy.get_param("~startup_delay_sec", 0.0))
    if startup_delay > 0:
        rospy.sleep(startup_delay)

    rospy.loginfo("等 move_base action server…")
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    if not ac.wait_for_server(rospy.Duration(60.0)):
        rospy.logfatal("未连接到 move_base，退出")
        return
    rospy.loginfo("move_base 已连接")

    tf_buf = tf2_ros.Buffer()
    _tf_lst = tf2_ros.TransformListener(tf_buf)

    talk_pub = rospy.Publisher(talk_topic, String, queue_size=4, latch=False)
    rospy.sleep(0.3)

    def speak(text: str) -> None:
        talk_pub.publish(String(data=text))
        rospy.loginfo("TTS: %s", text)
        if not tts_nonblocking and tts_pause > 0:
            rospy.sleep(tts_pause)

    def speak_async(text: str) -> None:
        threading.Thread(target=speak, args=(text,), daemon=True).start()

    def do_recognition(name: str) -> None:
        if AprilTagDetectionArray is None:
            rospy.logerr("[%s] 未安装 apriltag_ros，按未识别播报", name)
            speak_async(tts_miss)
            return

        class _Seen:
            ok = False

        seen = _Seen()

        def cb(msg) -> None:
            if detections_contain_id(msg, tag_id):
                seen.ok = True

        sub = rospy.Subscriber(tag_topic, AprilTagDetectionArray, cb, queue_size=10)
        t_end = rospy.Time.now() + rospy.Duration.from_sec(tag_scan_sec)
        rate = rospy.Rate(20)
        while rospy.Time.now() < t_end and not rospy.is_shutdown():
            if seen.ok:
                break
            rate.sleep()
        sub.unregister()
        speak_async(tts_hit if seen.ok else tts_miss)

    timeout = rospy.Duration.from_sec(goal_timeout_sec)
    for idx, name in enumerate(order):
        if name not in wp:
            rospy.logerr("YAML 缺少航点 %s，跳过", name)
            continue
        row = wp[name]
        if len(row) < 3:
            rospy.logerr("航点 %s 格式应为 [x,y,yaw_deg]，跳过：%s", name, row)
            continue
        x, y, yaw_deg = float(row[0]), float(row[1]), float(row[2])

        # 起点不需要发 goal（机器人已经在起点）
        if idx == 0 and skip_first:
            rospy.loginfo("[%s] 起点 (%.3f, %.3f, %.1f°) — 不发 goal", name, x, y, yaw_deg)
            if name in tag_at:
                do_recognition(name)
            continue

        rospy.loginfo("→ %s  (%.3f, %.3f) yaw=%.1f°", name, x, y, yaw_deg)
        succeeded = False
        for attempt in range(point_retries + 1):
            if clear_before_each:
                call_clear_costmaps()
                rospy.sleep(0.3)

            ac.send_goal(make_goal(map_frame, x, y, yaw_deg))
            ok_wait = ac.wait_for_result(timeout)
            st = ac.get_state()
            if ok_wait and st == GoalStatus.SUCCEEDED:
                rospy.loginfo("[%s] SUCCEEDED", name)
                succeeded = True
                break

            if not ok_wait:
                ac.cancel_goal()
                rospy.sleep(0.2)
                rospy.logwarn("[%s] 第 %d 次：%.0fs 超时未到达 (st=%s)",
                              name, attempt + 1, goal_timeout_sec, status_name(st))
            else:
                rospy.logwarn("[%s] 第 %d 次：%s", name, attempt + 1, status_name(st))

            # 容差自检：尽管 move_base 没 SUCCEEDED，但车若已经离目标很近，认为通过。
            pose = current_xy(tf_buf, map_frame, base_frame)
            if pose is not None:
                dxy = math.hypot(pose[0] - x, pose[1] - y)
                rospy.loginfo("[%s] 当前距目标 %.2fm（容差 %.2fm）", name, dxy, arrive_tol_m)
                if dxy <= arrive_tol_m:
                    rospy.loginfo("[%s] 已进入容差区，按到达处理", name)
                    succeeded = True
                    break

            if attempt < point_retries:
                rospy.logwarn("[%s] 清代价图后重试…", name)
                call_clear_costmaps()
                rospy.sleep(0.6)

        if not succeeded:
            rospy.logwarn("[%s] 多次尝试仍未到达，跳过本点继续下一航点", name)

        if name in tag_at:
            do_recognition(name)

    rospy.loginfo("全部航点流程结束")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        sys.exit(0)
