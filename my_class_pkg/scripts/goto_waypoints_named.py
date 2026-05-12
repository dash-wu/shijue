#!/usr/bin/env python3
"""按名字访问导航点（如 A,B,C）；依赖 move_base 已运行。"""
import math
from typing import Dict, List

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf_conversions import transformations


def _goal_status_name(st: int) -> str:
    names = {
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
    return names.get(st, "UNKNOWN(%d)" % st)


def goto(
    client: actionlib.SimpleActionClient,
    xyw: List[float],
    timeout: rospy.Duration,
    label: str,
) -> bool:
    x, y, yaw_deg = float(xyw[0]), float(xyw[1]), float(xyw[2])
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
    client.send_goal(g)
    waited = client.wait_for_result(timeout)
    st = client.get_state()
    if not waited:
        client.cancel_goal()
        rospy.logwarn(
            "[%s] move_base 等待结果超时(%.0fs)，上次状态=%s",
            label,
            timeout.to_sec(),
            _goal_status_name(st),
        )
        return False
    if st != GoalStatus.SUCCEEDED:
        rospy.logwarn(
            "[%s] move_base 未成功，终端状态=%s；常见：定位飘/目标在障碍内/规划无解",
            label,
            _goal_status_name(st),
        )
        return False
    return True


def main():
    rospy.init_node("goto_waypoints_named", anonymous=False)
    timeout_sec = float(rospy.get_param("~goal_timeout_sec", 90.0))
    wpd: Dict = rospy.get_param("~waypoints")
    order: List[str] = rospy.get_param("~visit_order", ["A", "B", "C"])

    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("等待 move_base ...")
    ac.wait_for_server(rospy.Duration(120.0))

    to = rospy.Duration(timeout_sec)
    for name in order:
        if name not in wpd:
            rospy.logerr("waypoints 里没有名字: %s", name)
            continue
        pt = wpd[name]
        if len(pt) != 3:
            rospy.logerr("点 %s 必须是 [x,y,yaw_deg]: %s", name, pt)
            continue
        rospy.loginfo("驶向任务点 [%s]: %s", name, pt)
        ok = goto(ac, pt, to, name)
        if ok:
            rospy.loginfo("[%s] 到达", name)
        else:
            rospy.logwarn("[%s] 失败或超时，继续下一个", name)
    rospy.loginfo("visit_order 执行完毕")


if __name__ == "__main__":
    main()
