#!/usr/bin/env python3
import os
import sys

# rosrun 安装路径下需能 import 同目录 upros_gesture
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import Empty, EmptyRequest

from upros_gesture import UP_Gesture


class ImageSubscriberNode:
    def __init__(self):
        rospy.init_node("gesture_control_node", anonymous=True)
        self.bridge = CvBridge()
        self.up_gesture = UP_Gesture()
        self.image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.image_callback, queue_size=1
        )
        self.image_pub = rospy.Publisher("/image_result", Image, queue_size=10)
        self.last_result_number = -1

        self.grab_proxy = None
        self.release_proxy = None
        self.empty_req = EmptyRequest()
        try:
            rospy.wait_for_service("/upros_arm_control/grab_service", timeout=5.0)
            rospy.wait_for_service("/upros_arm_control/release_service", timeout=5.0)
            self.grab_proxy = rospy.ServiceProxy("/upros_arm_control/grab_service", Empty)
            self.release_proxy = rospy.ServiceProxy(
                "/upros_arm_control/release_service", Empty
            )
            rospy.loginfo("夹爪服务已连接: grab / release")
        except rospy.ROSException as e:
            rospy.logwarn("夹爪服务未就绪（可先起 bringup）: %s", e)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            result = cv_image.copy()
            hands = self.up_gesture.findHind(result)
            if hands:
                result_number = self.up_gesture.detectNumber(hands, result)
                if result_number >= 0:
                    cv2.putText(
                        result,
                        str(result_number),
                        (150, 150),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        3.0,
                        (255, 0, 255),
                        5,
                        cv2.LINE_AA,
                    )
                    if result_number == 0 and self.grab_proxy is not None:
                        rospy.loginfo_throttle(2.0, "手势 0 -> 闭合夹爪")
                        if self.last_result_number != result_number:
                            self.grab_proxy.call(self.empty_req)
                    elif result_number == 5 and self.release_proxy is not None:
                        rospy.loginfo_throttle(2.0, "手势 5 -> 张开夹爪")
                        if self.last_result_number != result_number:
                            self.release_proxy.call(self.empty_req)
                    self.last_result_number = result_number
                else:
                    cv2.putText(
                        result,
                        "NO NUMBER",
                        (80, 150),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.2,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )
            ros_image = self.bridge.cv2_to_imgmsg(result, "bgr8")
            self.image_pub.publish(ros_image)
        except CvBridgeError as e:
            rospy.logerr("%s", e)


if __name__ == "__main__":
    try:
        ImageSubscriberNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
