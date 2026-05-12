#!/usr/bin/env python3
"""
集中输出「语音识别」与「AprilTag 检测」的可读反馈。

若 rostopic echo 一直无输出：确认本节点已启动（rosnode list | grep voice_tag_feedback），
并 echo /voice_tag_feedback/combined（默认 1Hz 刷新，便于看见文字）。
"""
import sys
import time

import rospy
from std_msgs.msg import String

try:
    from apriltag_ros.msg import AprilTagDetectionArray
except ImportError as e:
    print("voice_tag_feedback: 无法导入 apriltag_ros.msg，请 apt/rosdep 安装 apriltag_ros:", e, file=sys.stderr)
    AprilTagDetectionArray = None

from upros_message.msg import TagCommand


def _fmt_pose(det):
    try:
        p = det.pose
        pos = p.pose.pose.position
        return "x=%.3f y=%.3f z=%.3f (相机系, m)" % (pos.x, pos.y, pos.z)
    except Exception:
        return "pose=?"


class VoiceTagFeedback:
    def __init__(self):
        rospy.init_node("voice_tag_feedback", anonymous=True)
        self._last_tag_summary = "[AprilTag] 尚未订阅到检测话题或尚无数据"
        self._last_speech = "（尚未收到语音识别）"
        self._last_command = "（尚未解析抓取指令）"
        self._tag_throttle_sec = float(rospy.get_param("~tag_feedback_throttle_sec", 0.4))
        self._last_tag_pub_time = 0.0
        tag_topic = rospy.get_param("~tag_detections_topic", "/tag_detections")
        self._combined_hz = float(rospy.get_param("~combined_refresh_hz", 1.0))

        self.pub_speech = rospy.Publisher(
            "/voice_tag_feedback/speech", String, queue_size=20, latch=True
        )
        self.pub_tag = rospy.Publisher(
            "/voice_tag_feedback/apriltag", String, queue_size=20, latch=True
        )
        self.pub_combined = rospy.Publisher(
            "/voice_tag_feedback/combined", String, queue_size=20, latch=True
        )
        self.pub_command = rospy.Publisher(
            "/voice_tag_feedback/command", String, queue_size=20, latch=True
        )

        rospy.Subscriber("/speech/result", String, self._on_speech, queue_size=10)
        rospy.Subscriber("/voice_control", TagCommand, self._on_command, queue_size=10)
        if AprilTagDetectionArray is not None:
            rospy.Subscriber(tag_topic, AprilTagDetectionArray, self._on_tags, queue_size=5)
        else:
            self._last_tag_summary = "[AprilTag] 未安装 apriltag_ros Python 消息，跳过标签订阅"

        rospy.loginfo(
            "voice_tag_feedback 已启动: 订阅 /speech/result、/voice_control、%s；"
            "发布 /voice_tag_feedback/{speech,command,apriltag,combined}",
            tag_topic if AprilTagDetectionArray else "(无)",
        )
        print(
            "[voice_tag_feedback] 已启动。请另开终端: rostopic echo /voice_tag_feedback/combined",
            flush=True,
        )

        rospy.sleep(0.3)
        self._publish_combined()
        self.pub_speech.publish(String(data="[语音识别] （等待说话…）"))
        self.pub_command.publish(String(data="[指令解析] （等待语音解析…）"))
        self.pub_tag.publish(String(data=self._last_tag_summary))

        if self._combined_hz > 0:
            rospy.Timer(rospy.Duration(1.0 / self._combined_hz), self._timer_combined)

    def _timer_combined(self, _evt=None):
        self._publish_combined()

    def _on_speech(self, msg):
        text = (msg.data or "").strip()
        if not text:
            return
        self._last_speech = text
        line = "[语音识别] %s" % text
        rospy.loginfo(line)
        print(line, flush=True)
        self.pub_speech.publish(String(data=line))
        self._publish_combined()

    def _on_command(self, msg):
        line = "[指令解析] intent=%s target=%d → TF 帧 tag_%d" % (
            msg.intent,
            msg.target,
            msg.target,
        )
        rospy.loginfo(line)
        print(line, flush=True)
        self._last_command = line
        self.pub_command.publish(String(data=line))
        self._publish_combined()

    def _publish_combined(self):
        sp = (
            self._last_speech
            if self._last_speech.startswith("[")
            else "[语音识别] " + self._last_speech
        )
        body = "%s\n%s\n%s" % (sp, self._last_command, self._last_tag_summary)
        self.pub_combined.publish(String(data=body))

    def _on_tags(self, msg):
        if not msg.detections:
            s = "[AprilTag] 当前帧未检测到标签"
        else:
            parts = []
            for i, det in enumerate(msg.detections):
                ids = list(det.id) if det.id else []
                sizes = list(det.size) if det.size else []
                pose_s = _fmt_pose(det)
                parts.append(
                    "标签#%d: id=%s 边长=%s m | %s"
                    % (i, ids, sizes, pose_s)
                )
            s = "[AprilTag] 本帧共 %d 个: %s" % (
                len(msg.detections),
                " ; ".join(parts),
            )
        now = time.time()
        self._last_tag_summary = s
        if now - self._last_tag_pub_time >= self._tag_throttle_sec:
            self._last_tag_pub_time = now
            rospy.loginfo(s)
            print(s, flush=True)
            self.pub_tag.publish(String(data=s))
            self._publish_combined()


if __name__ == "__main__":
    try:
        VoiceTagFeedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
