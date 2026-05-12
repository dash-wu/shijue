#!/usr/bin/env python3
"""期末智能家居实战：多段多点导航 + move_base路径规划 + AprilTag① + TTS。
导航栈：全局 GlobalPlanner(Dijkstra/A*) + 局部 DWA，每发一个子目标都会对当前点做一次全局路径规划，
并在局部频率下跟踪路径、避障；本节点按段的航点链表依次发往 move_base。

依赖：bringup_w2a、navigation.launch 已启动；本包 arena_mission.launch 起 apriltag + TTS + 本节点。"""
from __future__ import annotations

import math
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from std_msgs.msg import String
from tf_conversions import transformations

Waypoint = Tuple[float, float, float]


class Phase(Enum):
    NAV_LEAVE_START = auto()
    NAV_ROOM1_ENTRY = auto()
    NAV_ROOM1_TABLE = auto()
    TAG_ROOM1 = auto()
    NAV_ROOM2_ENTRY = auto()
    NAV_ROOM2_TABLE = auto()
    TAG_ROOM2 = auto()
    NAV_RETURN_MIDDLE = auto()
    NAV_FINISH = auto()
    DONE = auto()


def _as_triplet(v) -> Waypoint:
    if len(v) != 3:
        raise ValueError("waypoint must be [x,y,yaw_deg], got %r" % (v,))
    return float(v[0]), float(v[1]), float(v[2])


def _waypoints_from_rosparam(waypoints_key: str, legacy_key: str) -> List[Waypoint]:
    """支持 *_waypoints: [[x,y,yaw],...] 或旧版 legacy_key 单点 [x,y,yaw]。"""
    if rospy.has_param(waypoints_key):
        raw = rospy.get_param(waypoints_key)
        out: List[Waypoint] = []
        for row in raw:
            out.append(_as_triplet(row))
        if not out:
            raise ValueError("%s is empty" % waypoints_key)
        return out
    if rospy.has_param(legacy_key):
        raw = rospy.get_param(legacy_key)
        if raw and isinstance(raw[0], (list, tuple)):
            return [_as_triplet(row) for row in raw]
        return [_as_triplet(raw)]
    raise KeyError("need %s or %s" % (waypoints_key, legacy_key))


def _has_tag_id(msg: AprilTagDetectionArray, tid: int) -> bool:
    for det in msg.detections:
        for i in det.id:
            if int(i) == tid:
                return True
    return False


class ArenaMissionNode:
    def __init__(self):
        rospy.init_node("arena_mission", anonymous=False)

        self._tag_id = int(rospy.get_param("~tag_id", 1))
        tag_topic = rospy.get_param("~tag_topic", "/tag_detections")
        self._confirm = int(rospy.get_param("~tag_confirm_frames", 8))
        self._tag_timeout = float(rospy.get_param("~tag_search_timeout_sec", 14.0))
        self._goal_timeout = float(rospy.get_param("~goal_timeout_sec", 120.0))
        self._start_delay = float(rospy.get_param("~start_delay_sec", 3.0))
        self._tts_pause = float(rospy.get_param("~tts_pause_sec", 2.5))
        self._tts_found = rospy.get_param("~tts_found", "已找到目标")
        self._tts_lost = rospy.get_param("~tts_lost", "未找到目标")
        talk_topic = rospy.get_param("~tts_talk_topic", "/talk")
        self._nav_retry_max = int(rospy.get_param("~nav_retry_per_waypoint", 5))

        log_plan = rospy.get_param("~log_global_plan", True)
        plan_topic = rospy.get_param(
            "~global_plan_topic",
            "/move_base/GlobalPlanner/plan",
        )
        waypoint_step_topic = rospy.get_param(
            "~waypoint_goal_topic",
            "/arena_mission/current_goal",
        )

        self._talk = rospy.Publisher(talk_topic, String, queue_size=2, latch=False)
        self._tag_sub = rospy.Subscriber(tag_topic, AprilTagDetectionArray, self._on_tags, queue_size=5)
        self._goal_dbg_pub = rospy.Publisher(
            waypoint_step_topic,
            PoseStamped,
            queue_size=10,
            latch=False,
        )
        self._last_plan_log_ts = rospy.Time(0)
        if log_plan:
            self._plan_sub = rospy.Subscriber(
                plan_topic,
                Path,
                self._on_global_plan,
                queue_size=3,
            )
            rospy.loginfo("监听全局路径: %s (GlobalPlanner)", plan_topic)
        self._wp_table: Dict[Phase, List[Waypoint]] = {
            Phase.NAV_LEAVE_START: _waypoints_from_rosparam(
                "~leave_start_waypoints", "~leave_start"
            ),
            Phase.NAV_ROOM1_ENTRY: _waypoints_from_rosparam(
                "~room1_entry_waypoints", "~room1_entry"
            ),
            Phase.NAV_ROOM1_TABLE: _waypoints_from_rosparam(
                "~room1_table_waypoints", "~room1_table"
            ),
            Phase.NAV_ROOM2_ENTRY: _waypoints_from_rosparam(
                "~room2_entry_waypoints", "~room2_entry"
            ),
            Phase.NAV_ROOM2_TABLE: _waypoints_from_rosparam(
                "~room2_table_waypoints", "~room2_table"
            ),
            Phase.NAV_RETURN_MIDDLE: _waypoints_from_rosparam(
                "~return_middle_waypoints", "~return_middle"
            ),
            Phase.NAV_FINISH: _waypoints_from_rosparam(
                "~finish_waypoints", "~finish"
            ),
        }

        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("等待 move_base ...")
        self.ac.wait_for_server(rospy.Duration(120.0))
        rospy.loginfo("move_base 已连接（全局规划 + DWA局部避障由导航栈执行）")

        self.phase = Phase.NAV_LEAVE_START
        self._nav_chain_idx = 0
        self._current_segment: Optional[List[Waypoint]] = None
        self._wp_idx = 0
        self._nav_deadline: Optional[rospy.Time] = None
        self._tag_deadline = None
        self._tag_streak = 0
        self._nav_fail_ctr = 0

    def _on_global_plan(self, msg: Path):
        now = rospy.Time.now()
        if (now - self._last_plan_log_ts).to_sec() >= 5.0:
            self._last_plan_log_ts = now
            n = len(msg.poses)
            rospy.loginfo(
                "全局路径更新：共 %d 个路径点 (GlobalPlanner + 后续 DWA 局部跟踪)",
                n,
            )

    def _on_tags(self, msg: AprilTagDetectionArray):
        if _has_tag_id(msg, self._tag_id):
            self._tag_streak += 1
        else:
            self._tag_streak = 0

    def _speak(self, text: str):
        self._talk.publish(String(data=text))
        rospy.loginfo("TTS: %s", text)
        rospy.sleep(self._tts_pause)

    def _send_pose(self, x: float, y: float, yaw_deg: float):
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
        self._goal_dbg_pub.publish(ps)

        g = MoveBaseGoal()
        g.target_pose = ps
        self.ac.send_goal(g)
        self._nav_deadline = rospy.Time.now() + rospy.Duration(self._goal_timeout)

    def _send_current_waypoint(self):
        x, y, yaw = self._current_segment[self._wp_idx]
        rospy.loginfo(
            "[%s] 下发子目标 [%d/%d] (%.3f, %.3f, %.1f°) → move_base 将做全局路径规划",
            self.phase.name,
            self._wp_idx + 1,
            len(self._current_segment),
            x,
            y,
            yaw,
        )
        self._send_pose(x, y, yaw)

    def _begin_navigation_phase(self, ph: Phase):
        self.phase = ph
        self._current_segment = list(self._wp_table[ph])
        self._wp_idx = 0
        self._nav_fail_ctr = 0
        rospy.loginfo(
            "[%s] 进入路段，含 %d 个中继导航点（顺序执行）",
            ph.name,
            len(self._current_segment),
        )
        self._send_current_waypoint()

    def _poll_nav_done(self) -> Optional[bool]:
        """None 进行中 / True 成功 / False 失败超时或 ABORT"""
        state = self.ac.get_state()
        if state == GoalStatus.SUCCEEDED:
            return True
        if state in (
            GoalStatus.ABORTED,
            GoalStatus.REJECTED,
            GoalStatus.PREEMPTED,
        ):
            return False
        if self._nav_deadline and rospy.Time.now() > self._nav_deadline:
            rospy.logwarn("导航超时，取消当前子目标")
            self.ac.cancel_goal()
            return False
        return None

    def run(self):
        rospy.loginfo("arena_mission 启动；AprilTag 目标 id=%s", self._tag_id)
        rospy.sleep(max(0.0, self._start_delay))

        self._begin_navigation_phase(Phase.NAV_LEAVE_START)

        rate = rospy.Rate(15)
        while not rospy.is_shutdown() and self.phase != Phase.DONE:
            if not self._tick():
                break
            rate.sleep()

        if self.phase == Phase.DONE:
            rospy.loginfo("任务结束（DONE：不再发往 move_base）")

    def _tick(self) -> bool:
        if self.phase in (
            Phase.NAV_LEAVE_START,
            Phase.NAV_ROOM1_ENTRY,
            Phase.NAV_ROOM1_TABLE,
            Phase.NAV_ROOM2_ENTRY,
            Phase.NAV_ROOM2_TABLE,
            Phase.NAV_RETURN_MIDDLE,
            Phase.NAV_FINISH,
        ):
            return self._tick_navigation()

        if self.phase in (Phase.TAG_ROOM1, Phase.TAG_ROOM2):
            if self._tag_streak >= self._confirm:
                self._speak(self._tts_found)
                self._tag_streak = 0
                self._advance_after_tag()
                return True
            if rospy.Time.now() > self._tag_deadline:
                self._speak(self._tts_lost)
                self._tag_streak = 0
                self._advance_after_tag()
                return True

        if self.phase == Phase.DONE:
            return False
        return True

    def _tick_navigation(self) -> bool:
        assert self._current_segment is not None
        r = self._poll_nav_done()
        if r is None:
            return True
        if not r:
            self._nav_fail_ctr += 1
            rospy.logwarn(
                "[%s] 当前子路径失败 retry %d/%d",
                self.phase.name,
                self._nav_fail_ctr,
                self._nav_retry_max,
            )
            if self._nav_fail_ctr >= self._nav_retry_max:
                rospy.logerr(
                    "[%s] 超过重试上限，跳过该子点到下一点（请检查禁区/膨胀/坐标）",
                    self.phase.name,
                )
                self._nav_fail_ctr = 0
                self._wp_idx += 1
                if self._wp_idx < len(self._current_segment):
                    self._send_current_waypoint()
                else:
                    self._finish_navigation_segment_advance_fsm()
                return True
            self._send_current_waypoint()
            return True

        # 本子目标达成
        self._nav_fail_ctr = 0
        self._wp_idx += 1
        if self._wp_idx < len(self._current_segment):
            self._send_current_waypoint()
            return True
        self._finish_navigation_segment_advance_fsm()
        return True

    def _finish_navigation_segment_advance_fsm(self):
        if self.phase == Phase.NAV_LEAVE_START:
            self._begin_navigation_phase(Phase.NAV_ROOM1_ENTRY)

        elif self.phase == Phase.NAV_ROOM1_ENTRY:
            self._begin_navigation_phase(Phase.NAV_ROOM1_TABLE)

        elif self.phase == Phase.NAV_ROOM1_TABLE:
            self.phase = Phase.TAG_ROOM1
            self.ac.cancel_goal()
            self._tag_streak = 0
            self._tag_deadline = rospy.Time.now() + rospy.Duration(self._tag_timeout)
            rospy.loginfo("房间1：搜寻 AprilTag id=%s", self._tag_id)

        elif self.phase == Phase.NAV_ROOM2_ENTRY:
            self._begin_navigation_phase(Phase.NAV_ROOM2_TABLE)

        elif self.phase == Phase.NAV_ROOM2_TABLE:
            self.phase = Phase.TAG_ROOM2
            self.ac.cancel_goal()
            self._tag_streak = 0
            self._tag_deadline = rospy.Time.now() + rospy.Duration(self._tag_timeout)
            rospy.loginfo("房间2：搜寻 AprilTag id=%s", self._tag_id)

        elif self.phase == Phase.NAV_RETURN_MIDDLE:
            self._begin_navigation_phase(Phase.NAV_FINISH)

        elif self.phase == Phase.NAV_FINISH:
            rospy.logwarn("已到终点路段末端，切断 move_base")
            self.ac.cancel_all_goals()
            self.phase = Phase.DONE

    def _advance_after_tag(self):
        self.ac.cancel_goal()
        if self.phase == Phase.TAG_ROOM1:
            self._begin_navigation_phase(Phase.NAV_ROOM2_ENTRY)
        elif self.phase == Phase.TAG_ROOM2:
            self._begin_navigation_phase(Phase.NAV_RETURN_MIDDLE)


def main():
    try:
        ArenaMissionNode().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
