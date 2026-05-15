#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImageProcessor:
    def __init__(self):
        rospy.init_node("image_processor_node", anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/camera/color/image_raw",
            Image,
            self.image_callback,
            queue_size=1,
        )
        self.result_pub = rospy.Publisher("/image_result", Image, queue_size=1)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.circle(cv_image, (50, 50), 30, (0, 0, 255), 2)
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.result_pub.publish(ros_image)
        except CvBridgeError as e:
            rospy.logerr("CV Bridge转换失败: {}".format(e))

    def shutdown_hook(self):
        rospy.loginfo("节点已关闭")


if __name__ == "__main__":
    try:
        processor = ImageProcessor()
        rospy.on_shutdown(processor.shutdown_hook)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
