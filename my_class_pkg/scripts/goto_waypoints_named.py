#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
多航点巡逻：顺序见 YAML visit_order（如 TASK1 → TASK2 → TASK3 → END）。

- 导航：move_base，位姿在 map 下；航点见 config/mission_start_two_tasks_end.yaml。
- 识别：在 `tag_check_at` 点短时订阅 /tag_detections，**识别到即结束等待**；播 TTS 可用 `tts_nonblocking` 避免长时间停车。
- 匀速：`speed_boost.boost_at_start: true` 时，出发前即通过 dynamic_reconfigure 统一 `max_vel_x/max_vel_trans`，各段上限与「出发→首点」一致（仍受障碍物与 DWA 其它项约束）。

航点第三项：
  * 数字 — 地图绝对朝向 yaw（度）；
  * 字符串 `bearing` — 车头朝向为「当前位置 → 该航点」的方位角（终点常用，避免在窄道强扭到固定绝对角导致 ABORT）；
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
    from dynamic_reconfigure.client import Client as DynReconfClient
except ImportError:
    DynReconfClient = None  # type: ignore

try:
    from apriltag_ros.msg import AprilTagDetectionArray
except ImportError:
    AprilTagDetectionArray = None  # type: ignore

from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from std_srvs.srv import Empty
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
    """row 为 [x,y,yaw|bearing] 或 [x,y,trigger|departure,offset_deg]。"""
    n = len(row)
    if n == 3:
        gx, gy = float(row[0]), float(row[1])
        third = row[2]
        if isinstance(third, str) and third.strip().lower() == "bearing":
            try:
                t = buf.lookup_transform(map_f, base_f, rospy.Time(0))
                rx = t.transform.translation.x
                ry = t.transform.translation.y
            except Exception as ex:
                rospy.logwarn("[%s] bearing 读 TF 失败 %s，暂用出发航向", name, ex)
                return departure_deg
            dx = gx - rx
            dy = gy - ry
            if dx * dx + dy * dy < 0.0025:
                y = yaw_deg_from_tf(buf, map_f, base_f)
                rospy.loginfo("[%s] yaw = bearing 已在点附近 → 当前车头 %.1f°", name, y)
                return y
            y = norm_deg(math.degrees(math.atan2(dy, dx)))
            rospy.loginfo("[%s] yaw = bearing（指向航点）→ %.1f°", name, y)
            return y
        return float(third)
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
    tf_buf: Optional[tf2_ros.Buffer] = None,
    map_f: Optional[str] = None,
    base_f: Optional[str] = None,
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

    use_tf = (
        bool(rospy.get_param("~stuck_recovery_enable", True))
        and tf_buf is not None
        and map_f is not None
        and base_f is not None
    )
    if not use_tf:
        ok_wait = ac.wait_for_result(timeout)
        st = ac.get_state()
        if not ok_wait:
            ac.cancel_goal()
            rospy.logwarn("[%s] 超时/%s", label, status_name(st))
            return False
        if st != GoalStatus.SUCCEEDED:
            rospy.logwarn("[%s] %s", label, status_name(st))
            if st == GoalStatus.ABORTED:
                rospy.logwarn(
                    "[%s] ABORTED 常见原因：①初定位不准/车在地图黑障里→RViz 重新 2D Pose Estimate；"
                    "②导航终端未正常或代价地图未稳→确认 B 中 navigation 无报错；"
                    "③可用 rostopic echo /cmd_vel 查是否长期全 0。",
                    label,
                )
            return False
        return True

    chunks = rospy.Duration.from_sec(max(2.5, float(rospy.get_param("~stuck_check_chunk_sec", 5.5))))
    min_eps = float(rospy.get_param("~stuck_min_goal_progress_per_chunk_m", 0.04))
    streak_need = max(2, int(rospy.get_param("~stuck_consecutive_chunks", 5)))
    near_stop = float(rospy.get_param("~stuck_disable_near_goal_planar_m", 0.52))
    clear_stuck = bool(rospy.get_param("~clear_costmaps_on_stuck", True))

    elapsed = rospy.Duration(0)
    streak_no_gain = 0

    try:
        t0 = tf_buf.lookup_transform(map_f, base_f, rospy.Time(0), rospy.Duration(0.4))
        r0 = t0.transform.translation
        last_best_xy = math.hypot(x - r0.x, y - r0.y)
    except Exception:
        rospy.logwarn("[%s] TF 不可用，跳过卡死探测", label)
        ok_wait = ac.wait_for_result(timeout)
        st = ac.get_state()
        if not ok_wait:
            ac.cancel_goal()
            return False
        return st == GoalStatus.SUCCEEDED

    warmup_sec = float(rospy.get_param("~stuck_min_elapsed_before_check_sec", 14.0))

    while elapsed < timeout and not rospy.is_shutdown():
        wd = rospy.Duration(min(chunks.to_sec(), (timeout - elapsed).to_sec()))
        if wd <= rospy.Duration(0):
            break
        fin = ac.wait_for_result(wd)
        elapsed += wd
        st = ac.get_state()
        if fin:
            break
        if st not in (
            GoalStatus.ACTIVE,
            GoalStatus.PENDING,
            GoalStatus.PREEMPTING,
        ):
            break
        try:
            t = tf_buf.lookup_transform(map_f, base_f, rospy.Time(0), rospy.Duration(0.35))
            r = t.transform.translation
            dxy = math.hypot(x - r.x, y - r.y)
        except Exception:
            continue
        if elapsed.to_sec() < warmup_sec:
            continue
        if dxy <= near_stop:
            streak_no_gain = 0
            continue
        if dxy < last_best_xy - min_eps:
            last_best_xy = dxy
            streak_no_gain = 0
        else:
            streak_no_gain += 1
            if streak_no_gain >= streak_need:
                rospy.logwarn(
                    "[%s] 约在固定位置卡死 ≈ %.0fs（距目标平面距离无明显缩短）→ 取消并重清代价地图后由外层重试",
                    label,
                    elapsed.to_sec(),
                )
                ac.cancel_goal()
                rospy.sleep(0.4)
                if clear_stuck:
                    try_clear_costmaps("[%s]-卡死 " % label)
                return False

    st = ac.get_state()
    if st == GoalStatus.SUCCEEDED:
        return True

    if st in (
        GoalStatus.ACTIVE,
        GoalStatus.PENDING,
        GoalStatus.PREEMPTING,
    ):
        ac.cancel_goal()
        rospy.sleep(0.05)
        st2 = ac.get_state()
        rospy.logwarn("[%s] 未在时限内抵达或仍进行中→已取消/%s", label, status_name(st2))
        return False

    rospy.logwarn("[%s] %s", label, status_name(st))
    if st == GoalStatus.ABORTED:
        rospy.logwarn(
            "[%s] ABORTED 常见原因：①初定位不准/车在地图黑障里→RViz 重新 2D Pose Estimate；"
            "②导航终端未正常或代价地图未稳→确认 B 中 navigation 无报错；"
            "③可用 rostopic echo /cmd_vel 查是否长期全 0。",
            label,
        )
    return False


def try_clear_costmaps(tag: str = "") -> bool:
    """清除局部+全局代价地图；窄道/起点卡住时可缓解。"""
    try:
        rospy.wait_for_service("/move_base/clear_costmaps", rospy.Duration(5.0))
        rospy.ServiceProxy("/move_base/clear_costmaps", Empty)()
        rospy.loginfo("%s已调用 /move_base/clear_costmaps", tag)
        return True
    except rospy.ROSException:
        rospy.logwarn("%s未找到服务 /move_base/clear_costmaps（move_base 未起？）", tag)
    except Exception as ex:
        rospy.logwarn("%sclear_costmaps 失败: %s", tag, ex)
    return False


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
    tts_pause = float(rospy.get_param("~tts_pause_sec", 0.5))
    tts_nonblocking = bool(rospy.get_param("~tts_nonblocking", False))
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
    settle = float(rospy.get_param("~post_tf_settle_sec", 4.0))
    if settle > 0:
        rospy.loginfo("TF 就绪后等待 %.1fs，让 AMCL/代价地图稳定后再发目标", settle)
        rospy.sleep(settle)
    if bool(rospy.get_param("~clear_costmaps_before_mission", True)):
        try_clear_costmaps("出发前 ")
        ac_after = float(rospy.get_param("~after_clear_settle_sec", 2.0))
        if ac_after > 0:
            rospy.sleep(ac_after)
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
        if tts_nonblocking:
            return
        if tts_pause > 1e-6:
            rospy.sleep(tts_pause)

    speed_boost: Dict = rospy.get_param("~speed_boost", {})
    dwa_backup: Optional[Tuple[float, float]] = None
    boost_active = False

    def dwa_apply_segment_speed(vx: float, vt: float, label: str) -> bool:
        nonlocal dwa_backup
        if DynReconfClient is None:
            rospy.logwarn("无 dynamic_reconfigure，跳过 %s", label)
            return False
        try:
            if dwa_backup is None:
                dwa_backup = (
                    float(rospy.get_param("/move_base/DWAPlannerROS/max_vel_x", 0.78)),
                    float(rospy.get_param("/move_base/DWAPlannerROS/max_vel_trans", 0.78)),
                )
            client = DynReconfClient("move_base/DWAPlannerROS", timeout=3.0)
            client.update_configuration({"max_vel_x": float(vx), "max_vel_trans": float(vt)})
            rospy.loginfo("%s：DWA max_vel_x/max_vel_trans → %.2f / %.2f m/s", label, vx, vt)
            return True
        except Exception as ex:
            rospy.logwarn("%s 失败：%s", label, ex)
            return False

    def dwa_restore_speed() -> None:
        nonlocal dwa_backup
        if dwa_backup is None:
            return
        vx, vt = dwa_backup
        try:
            if DynReconfClient is not None:
                client = DynReconfClient("move_base/DWAPlannerROS", timeout=3.0)
                client.update_configuration({"max_vel_x": float(vx), "max_vel_trans": float(vt)})
                rospy.loginfo("DWA 限速已恢复为 %.2f / %.2f m/s", vx, vt)
        except Exception as ex:
            rospy.logwarn("DWA 恢复限速失败：%s", ex)
        dwa_backup = None

    timeout = rospy.Duration(to_sec)
    abort_retries = int(rospy.get_param("~goal_abort_retries", 2))
    retry_clear = bool(rospy.get_param("~retry_clear_costmap_on_abort", True))
    retry_sleep = float(rospy.get_param("~goal_retry_sleep_sec", 2.0))

    # 任务一开始就统一 DWA 上限，使各段与「出发→首点」同一 cap（仍受障碍与局部规划约束）
    if (
        bool(speed_boost.get("enable", False))
        and bool(speed_boost.get("boost_at_start", False))
        and DynReconfClient is not None
    ):
        vx0 = float(speed_boost.get("max_vel_x", 0.78))
        vt0 = float(speed_boost.get("max_vel_trans", vx0))
        if dwa_apply_segment_speed(vx0, vt0, "任务开始：全程统一 max_vel_x/max_vel_trans"):
            boost_active = True

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
        ok = False
        attempts = 0
        max_attempts = abort_retries + 1
        while attempts < max_attempts and not ok and not rospy.is_shutdown():
            ok = send_nav_goal(ac, x, y, yaw, timeout, name, tf_buf=buf, map_f=map_f, base_f=base_f)
            attempts += 1
            if ok or attempts >= max_attempts:
                break
            rospy.logwarn(
                "[%s] 第 %d/%d 次未成功，%s",
                name,
                attempts,
                max_attempts,
                "清除代价地图后重试 …" if retry_clear else "等待后重试 …",
            )
            if retry_clear:
                try_clear_costmaps("重试前 ")
                rospy.sleep(float(rospy.get_param("~after_clear_settle_sec", 2.0)))
            if retry_sleep > 0:
                rospy.sleep(retry_sleep)
        if not ok:
            if boost_active:
                dwa_restore_speed()
                boost_active = False
            rospy.logwarn("跳过 %s 的 AprilTag，继续下一航点", name)
            continue

        if (
            bool(speed_boost.get("enable", False))
            and DynReconfClient is not None
            and str(speed_boost.get("boost_after_waypoint", "")).strip() == name
        ):
            vx_b = float(speed_boost.get("max_vel_x", 0.82))
            vt_b = float(speed_boost.get("max_vel_trans", vx_b))
            if dwa_apply_segment_speed(vx_b, vt_b, "TASK2 后提速（至 END 后恢复）"):
                boost_active = True

        if (
            bool(speed_boost.get("enable", False))
            and boost_active
            and str(speed_boost.get("restore_after_waypoint", "")).strip() == name
        ):
            dwa_restore_speed()
            boost_active = False

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
        t_end = rospy.Time.now() + rospy.Duration.from_sec(tag_dt)
        rate = rospy.Rate(30)
        while rospy.Time.now() < t_end and not rospy.is_shutdown():
            if seen.ok:
                break
            rate.sleep()
        sub.unregister()
        speak(t_ok if seen.ok else t_bad)

    if boost_active:
        dwa_restore_speed()

    rospy.loginfo("全部航点流程结束")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        sys.exit(0)
