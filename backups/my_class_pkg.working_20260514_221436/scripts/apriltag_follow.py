#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import apriltag


class ImageSubscriberNode:
    def __init__(self):
        rospy.init_node("apriltag_follow_node", anonymous=True)
        self.tag_detector = apriltag.Detector(
            apriltag.DetectorOptions(families="tag36h11")
        )
        self.follow_tag_id = int(rospy.get_param("~follow_tag_id", 1))
        self.deadband_px = int(rospy.get_param("~deadband_px", 20))
        self.min_area = int(rospy.get_param("~min_area", 300))
        self.angular_speed = float(rospy.get_param("~angular_speed", 0.35))
        self.linear_speed = float(rospy.get_param("~linear_speed", 0.2))

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.image_callback, queue_size=1
        )
        self.image_pub = rospy.Publisher("/image_result", Image, queue_size=10)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.loginfo(
            "AprilTag 跟随: family=tag36h11 id=%d min_area=%d",
            self.follow_tag_id,
            self.min_area,
        )

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = cv_image.copy()
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            height, width = gray.shape[:2]
            tags = self.tag_detector.detect(gray)

            twist = Twist()
            found = False

            for tag in tags:
                if tag.tag_id != self.follow_tag_id:
                    continue
                found = True
                corners = np.asarray(tag.corners, dtype=np.float32)
                center_x = int(np.mean(corners[:, 0]))
                center_y = int(np.mean(corners[:, 1]))
                w_box = float(np.max(corners[:, 0]) - np.min(corners[:, 0]))
                h_box = float(np.max(corners[:, 1]) - np.min(corners[:, 1]))
                area = int(max(w_box * h_box, 1))

                cv2.line(
                    frame,
                    (center_x - 20, center_y),
                    (center_x + 20, center_y),
                    (0, 0, 255),
                    2,
                )
                cv2.line(
                    frame,
                    (center_x, center_y - 20),
                    (center_x, center_y + 20),
                    (0, 0, 255),
                    2,
                )
                cv2.putText(
                    frame,
                    "id %d area %d" % (tag.tag_id, area),
                    (max(center_x - 40, 5), max(center_y - 30, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )

                mid = width / 2.0
                if center_x > mid + self.deadband_px:
                    twist.angular.z = -self.angular_speed
                elif center_x < mid - self.deadband_px:
                    twist.angular.z = self.angular_speed
                else:
                    twist.angular.z = 0.0

                if area < self.min_area:
                    twist.linear.x = self.linear_speed
                break

            if not found:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            self.vel_pub.publish(twist)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)
            return

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        ImageSubscriberNode().spin()
    except rospy.ROSInterruptException:
        pass
