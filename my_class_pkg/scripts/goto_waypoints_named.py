#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
三任务点巡逻：TASK1 → TASK2 → END（顺序见 YAML visit_order）。

- 导航：move_base，位姿在 map 下；航点见 config/mission_start_two_tasks_end.yaml。
- 识别：TASK1、TASK2 到点后短时看 /tag_detections，出现过指定 AprilTag id → 播「识别到目标」，否则「未识别到目标」；
  END 不做识别。实验细则以课程 PDF / 现场要求为准（本机未必附带 PDF）。

航点第三项：
  * 数字 — 地图绝对朝向 yaw（度）；
  * 四元组 — [x,y, trigger|departure, 偏移度]：目标 yaw = 参考 yaw + 偏移；
      trigger=发该点前瞬时车头；departure=连上 move_base 后记录的出发车头。
"""
from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

import actionlib
import rospy
import tf2_ros
from actionlib_msgs.msg import GoalStatus

try:
    from apriltag_ros.msg import AprilTagDetectionArray
except ImportError:
    AprilTagDetectionArray = None  # type: ignore

from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from tf_conversions import transformations

# ---------------------------------------------------------------------------
# 几何
# ---------------------------------------------------------------------------


def norm_deg(d: float) -> float:
    x = d % 360.0
    if x > 180.0:
        x -= 360.0
    if x < -180.0:
        x += 360.0
    return x


def yaw_deg_from_tf(buf: tf2_ros.Buffer, map_f: str, base_f: str) -> float:
    t = buf.lookup_transform(map_f, base_f, rospy.Time(0))
    q = t.transform.rotation
    _, _, yr = transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    return norm_deg(math.degrees(yr))


def resolve_yaw_deg(
    row: List,
    name: str,
    departure_deg: float,
    buf: tf2_ros.Buffer,
    map_f: str,
    base_f: str,
) -> float:
    """row 为 [x,y,yaw] 或 [x,y,trigger|departure,offset_deg]。"""
    n = len(row)
    if n == 3:
        return float(row[2])
    if n != 4:
        rospy.logerr("航点 %s 长度须为 3 或 4：%s", name, row)
        return 0.0
    ref = str(row[2]).strip().lower()
    off = float(row[3])
    if ref == "trigger":
        b = yaw_deg_from_tf(buf, map_f, base_f)
        y = norm_deg(b + off)
        rospy.loginfo("[%s] yaw = 触发 %.1f° + %.1f° → %.1f°", name, b, off, y)
        return y
    if ref == "departure":
        y = norm_deg(departure_deg + off)
        rospy.loginfo("[%s] yaw = 出发 %.1f° + %.1f° → %.1f°", name, departure_deg, off, y)
        return y
    rospy.logerr("[%s] 第三项应为 trigger 或 departure，得到 %r", name, row[2])
    return 0.0


# ---------------------------------------------------------------------------
# AprilTag
# ---------------------------------------------------------------------------


def detections_contain_id(msg, tid: int) -> bool:
    for d in msg.detections:
        for i in d.id:
            if int(i) == tid:
                return True
    return False


@dataclass
class TagSeen:
    ok: bool = False

    def make_cb(self, tid: int):
        def _cb(msg) -> None:
            if detections_contain_id(msg, tid):
                self.ok = True

        return _cb


# ---------------------------------------------------------------------------
# move_base
# ---------------------------------------------------------------------------


def status_name(st: int) -> str:
    m = {
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
    return m.get(st, "UNKNOWN(%d)" % st)


def send_nav_goal(
    ac: actionlib.SimpleActionClient,
    x: float,
    y: float,
    yaw_deg: float,
    timeout: rospy.Duration,
    label: str,
) -> bool:
    g = MoveBaseGoal()
    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.header.stamp = rospy.Time.now()
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = 0.0
    yaw = yaw_deg / 180.0 * math.pi
    q = transformations.quaternion_from_euler(0.0, 0.0, yaw)
    ps.pose.orientation.x = q[0]
    ps.pose.orientation.y = q[1]
    ps.pose.orientation.z = q[2]
    ps.pose.orientation.w = q[3]
    g.target_pose = ps
    ac.send_goal(g)
    ok_wait = ac.wait_for_result(timeout)
    st = ac.get_state()
    if not ok_wait:
        ac.cancel_goal()
        rospy.logwarn("[%s] 超时/%s", label, status_name(st))
        return False
    if st != GoalStatus.SUCCEEDED:
        rospy.logwarn("[%s] %s", label, status_name(st))
        return False
    return True


# ---------------------------------------------------------------------------
# TF / 启动等待
# ---------------------------------------------------------------------------


def wait_map_tf(sec: float, map_f: str, base_f: str) -> bool:
    if sec <= 0:
        return True
    buf = tf2_ros.Buffer()
    _ = tf2_ros.TransformListener(buf)
    rospy.loginfo("等待 TF %s→%s（%.0fs），请在 RViz 做 2D Pose Estimate", map_f, base_f, sec)
    t_end = rospy.Time.now() + rospy.Duration.from_sec(sec)
    rate = rospy.Rate(4)
    while rospy.Time.now() < t_end and not rospy.is_shutdown():
        try:
            buf.lookup_transform(map_f, base_f, rospy.Time(0), rospy.Duration(0.5))
            rospy.loginfo("TF 就绪")
            return True
        except Exception:
            pass
        rate.sleep()
    rospy.logfatal("无 TF %s→%s", map_f, base_f)
    return False


def wait_move_base(ac: actionlib.SimpleActionClient, total_sec: float) -> bool:
    deadline = rospy.Time.now() + rospy.Duration.from_sec(total_sec)
    while rospy.Time.now() < deadline:
        if ac.wait_for_server(rospy.Duration(5.0)):
            return True
        rospy.logwarn("等待 move_base …")
    rospy.logfatal("未连接 move_base")
    return False


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------


def main() -> None:
    rospy.init_node("goto_waypoints_named", anonymous=False)

    to_sec = float(rospy.get_param("~goal_timeout_sec", 120.0))
    wp: Dict = rospy.get_param("~waypoints")
    order: List[str] = rospy.get_param("~visit_order", ["TASK1", "TASK2", "END"])

    tc = rospy.get_param("~tag_check_at", ["TASK1", "TASK2"])
    tag_on: Set[str] = set(str(x) for x in tc) if isinstance(tc, list) else set()

    tag_id = int(rospy.get_param("~tag_target_id", 1))
    tag_dt = float(rospy.get_param("~tag_scan_duration_sec", 5.0))
    tag_topic = rospy.get_param("~tag_detections_topic", "/tag_detections")
    talk_t = rospy.get_param("~tts_talk_topic", "/talk")
    tts_pause = float(rospy.get_param("~tts_pause_sec", 2.5))
    t_ok = rospy.get_param("~tts_hit", "识别到目标")
    t_bad = rospy.get_param("~tts_miss", "未识别到目标")

    sdelay = float(rospy.get_param("~startup_delay_sec", 0.0))
    if sdelay > 0:
        rospy.sleep(sdelay)

    map_f = rospy.get_param("~map_frame", "map")
    base_f = rospy.get_param("~base_frame", "base_footprint")
    if not wait_map_tf(float(rospy.get_param("~wait_map_tf_sec", 120.0)), map_f, base_f):
        return

    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    if not wait_move_base(ac, float(rospy.get_param("~wait_move_base_sec", 300.0))):
        return

    buf = tf2_ros.Buffer()
    _lst = tf2_ros.TransformListener(buf)
    rospy.sleep(0.5)
    try:
        depart = yaw_deg_from_tf(buf, map_f, base_f)
    except Exception:
        rospy.logwarn("首次读出发航向失败，departure 相对角按 0")
        depart = 0.0
    rospy.loginfo("出发朝向 ≈ %.1f°", depart)

    talk = rospy.Publisher(talk_t, String, queue_size=2, latch=False)

    def speak(txt: str) -> None:
        talk.publish(String(data=txt))
        rospy.loginfo("TTS: %s", txt)
        rospy.sleep(tts_pause)

    timeout = rospy.Duration(to_sec)

    for name in order:
        if name not in wp:
            rospy.logerr("无航点 %s", name)
            continue
        row = wp[name]
        if len(row) not in (3, 4):
            rospy.logerr("航点格式错误 %s: %s", name, row)
            continue
        x, y = float(row[0]), float(row[1])
        yaw = resolve_yaw_deg(row, name, depart, buf, map_f, base_f)

        rospy.loginfo("→ %s  (%.3f, %.3f) yaw=%.1f°", name, x, y, yaw)
        ok = send_nav_goal(ac, x, y, yaw, timeout, name)
        if not ok:
            rospy.logwarn("跳过 %s 后 AprilTag", name)
            continue

        if name not in tag_on:
            continue
        if AprilTagDetectionArray is None:
            rospy.logerr("未安装 apriltag_ros，无法识别，跳过 %s", name)
            continue
        seen = TagSeen()
        sub = rospy.Subscriber(
            tag_topic,
            AprilTagDetectionArray,
            seen.make_cb(tag_id),
            queue_size=20,
        )
        rospy.sleep(tag_dt)
        sub.unregister()
        speak(t_ok if seen.ok else t_bad)

    rospy.loginfo("全部航点流程结束")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        sys.exit(0)
