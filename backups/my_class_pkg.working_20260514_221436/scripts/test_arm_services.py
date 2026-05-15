#!/usr/bin/env python3
"""不经过语音/视觉，直接调用机械臂服务，用于确认「臂能否动」。

用法（已 source 环境）:
  rosrun my_class_pkg test_arm_services.py
  rosrun my_class_pkg test_arm_services.py _x:=0 _y:=300 _z:=150

若此处臂仍不动：查电源、急停、/upros_arm_control 是否在跑、串口权限。
"""
import sys

import rospy
from std_srvs.srv import Empty
from upros_message.srv import ArmPosition, ArmPositionRequest


def main():
    rospy.init_node("test_arm_services", anonymous=True)
    x = float(rospy.get_param("~x", 0.0))
    y = float(rospy.get_param("~y", 300.0))
    z = float(rospy.get_param("~z", 150.0))
    rospy.loginfo("等待 /upros_arm_control/arm_pos_service_open ...")
    rospy.wait_for_service("/upros_arm_control/arm_pos_service_open", timeout=30.0)
    rospy.wait_for_service("/upros_arm_control/zero_service", timeout=10.0)
    move = rospy.ServiceProxy("/upros_arm_control/arm_pos_service_open", ArmPosition)
    zero = rospy.ServiceProxy("/upros_arm_control/zero_service", Empty)
    req = ArmPositionRequest(x=x, y=y, z=z)
    rospy.loginfo("调用 arm_pos_service_open: x=%.1f y=%.1f z=%.1f (与 inverse_move 测试同量级)", x, y, z)
    try:
        r = move(req)
        rospy.loginfo("arm_pos_service_open 返回 status=%s", getattr(r, "status", "?"))
    except rospy.ServiceException as e:
        rospy.logerr("调用失败: %s", e)
        sys.exit(1)
    rospy.sleep(4.0)
    rospy.loginfo("调用 zero_service 归零")
    try:
        zero()
    except rospy.ServiceException as e:
        rospy.logerr("归零失败: %s", e)


if __name__ == "__main__":
    main()
