#!/usr/bin/env python3
# 黑色电工胶布巡线：黑线在 HSV 里主要靠「暗」（低 V），H 不要用窄区间。

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from cv_bridge import CvBridge, CvBridgeError


class ImageSubscriberNode:
    def __init__(self):

        rospy.init_node("image_subscriber_node", anonymous=True)

        # --- 黑色胶布推荐初值（可通过参数覆盖，勿把 H 缩成窄带）---
        # rosrun … follow_line.py _vmax:=80 _min_m00:=800
        self.hmin = int(rospy.get_param("~hmin", 0))
        self.hmax = int(rospy.get_param("~hmax", 180))
        self.smin = int(rospy.get_param("~smin", 0))
        self.smax = int(rospy.get_param("~smax", 255))
        self.vmin = int(rospy.get_param("~vmin", 0))
        self.vmax = int(rospy.get_param("~vmax", 90))  # 仅保留较暗像素；反光强可提高到 100～115
        self.min_m00 = float(rospy.get_param("~min_m00", 800.0))

        # 线速度略降、角增益略降，减轻左右摇摆
        self.linear_x = float(rospy.get_param("~linear_x", 0.12))
        self.angular_gain = float(rospy.get_param("~angular_gain", 0.006))
        # 像素误差死区：中心附近不当成偏转，减少来回抖
        self.err_deadband_px = float(rospy.get_param("~err_deadband_px", 32.0))
        self.err_alpha = float(rospy.get_param("~err_alpha", 0.22))
        self.w_alpha = float(rospy.get_param("~w_alpha", 0.38))
        self.max_angular_z = float(rospy.get_param("~max_angular_z", 0.22))
        self.w_step = float(rospy.get_param("~w_step", 0.025))
        self.min_linear_scale = float(rospy.get_param("~min_linear_scale", 0.55))

        self._err_filt = 0.0
        self._w_smooth = 0.0
        self._w_prev = 0.0

        self.enable_move = False

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.image_callback, queue_size=1
        )
        self.enable_sub = rospy.Subscriber(
            "/enable_move", Int16, self.enable_callback, queue_size=1
        )
        self.image_mask_pub = rospy.Publisher("/image_mask", Image, queue_size=10)
        self.image_result_pub = rospy.Publisher("/image_result", Image, queue_size=10)

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.loginfo(
            "follow_line 黑胶布 H[%d,%d] S[%d,%d] V[%d,%d] m00>=%.0f | deadband=%.0fpx gain=%.4f max_w=%.2f | /enable_move:=1",
            self.hmin,
            self.hmax,
            self.smin,
            self.smax,
            self.vmin,
            self.vmax,
            self.min_m00,
            self.err_deadband_px,
            self.angular_gain,
            self.max_angular_z,
        )

    def enable_callback(self, msg):
        self.enable_move = msg.data == 1
        if not self.enable_move:
            self._err_filt = 0.0
            self._w_smooth = 0.0
            self._w_prev = 0.0
            self.move_up(0.0, 0.0, 0.0)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            src = cv_image.copy()
            self.update_frame(
                src, self.hmin, self.hmax, self.smin, self.smax, self.vmin, self.vmax
            )
        except CvBridgeError as e:
            rospy.logerr(e)
            return

    def update_frame(self, img, h_min, h_max, s_min, s_max, v_min, v_max):

        result = img

        hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        low_color = np.array([h_min, s_min, v_min])
        high_color = np.array([h_max, s_max, v_max])
        mask_color = cv2.inRange(hsv_frame, low_color, high_color)
        mask_color = cv2.medianBlur(mask_color, 5)
        # 略膨胀，让断裂的细黑线连起来
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask_color = cv2.morphologyEx(mask_color, cv2.MORPH_CLOSE, k)
        h, w, _ = img.shape

        search_top = 5 * h // 6
        mask_color[0:search_top, 0:w] = 0

        ros_mask_image = self.bridge.cv2_to_imgmsg(mask_color, "8UC1")
        self.image_mask_pub.publish(ros_mask_image)

        M = cv2.moments(mask_color)
        if M["m00"] > self.min_m00:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            err = float(cx - w / 2)
            if abs(err) < self.err_deadband_px:
                err = 0.0
            self._err_filt = self.err_alpha * err + (1.0 - self.err_alpha) * self._err_filt

            w_cmd = -self._err_filt * self.angular_gain
            w_cmd = float(np.clip(w_cmd, -self.max_angular_z, self.max_angular_z))

            self._w_smooth = self.w_alpha * w_cmd + (1.0 - self.w_alpha) * self._w_smooth
            dw = self._w_smooth - self._w_prev
            if abs(dw) > self.w_step:
                self._w_smooth = self._w_prev + float(np.sign(dw)) * self.w_step
            self._w_prev = self._w_smooth

            turn_ratio = abs(self._w_smooth) / max(self.max_angular_z, 1e-6)
            lin_scale = float(np.clip(1.0 - 0.75 * turn_ratio, self.min_linear_scale, 1.0))
            linear_x = self.linear_x * lin_scale

            self.move_up(linear_x, 0.0, self._w_smooth)
            cv2.circle(result, (cx, cy), 20, (0, 0, 255), -1)
        else:
            self._err_filt = 0.0
            self._w_smooth = 0.0
            self._w_prev = 0.0
            self.move_up(0.0, 0.0, 0.0)

        ros_result_image = self.bridge.cv2_to_imgmsg(result, "bgr8")
        self.image_result_pub.publish(ros_result_image)

    def move_up(self, x, y, th):
        t = Twist()
        t.linear.x = x
        t.linear.y = y
        t.angular.z = th
        if self.enable_move:
            self.cmd_pub.publish(t)

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = ImageSubscriberNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
