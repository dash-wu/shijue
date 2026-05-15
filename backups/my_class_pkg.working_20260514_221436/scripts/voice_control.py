#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
《语音交互与智能问答》实验：语音控制节点

订阅语音识别结果 /speech/result，经 tokenizer 解码后发布 /voice_control（TagCommand）。

教材步骤对应：
  rosrun my_class_pkg voice_control.py

抓取实验（抓取一号/二号）仅使用 intent=pick 且 target∈{1,2}；其它意图可扩展。
与教材一致：话题名无空格，应为 /voice_control、/speech/result。
"""
import rospy
from std_msgs.msg import String
from upros_message.msg import TagCommand

from grab_intent import parse_pick_target_fallback
from tokenizer import Tokenizer


class VoiceControlNode:
    def __init__(self):
        rospy.init_node("tokenizer_publisher")
        self.tokenizer = Tokenizer()
        self.tag_cmd_pub = rospy.Publisher("/voice_control", TagCommand, queue_size=10)
        self.text_fb_pub = rospy.Publisher(
            "/voice_tag_feedback/text", String, queue_size=20, latch=True
        )
        rospy.sleep(0.2)
        self._emit_text("[voice_control] 就绪，等待语音识别…")
        self.talker_sub = rospy.Subscriber(
            "/speech/result", String, self.speech_result_callback
        )

    def _emit_text(self, line):
        rospy.loginfo("%s", line)
        print(line, flush=True)
        self.text_fb_pub.publish(String(data=line))

    def speech_result_callback(self, msg):
        """教材：从语音字符串经 tokenizer 得到意图列表；本实验对抓取只处理 pick+1/2。"""
        user_input = (msg.data or "").strip()
        if not user_input:
            return
        self._emit_text("[ASR] %s" % user_input)
        rospy.loginfo("收到语音识别: %s", user_input)

        filtered_input = self.tokenizer.pre_process(user_input)
        intent_string = self.tokenizer.extract_intent(filtered_input)

        intent = None
        target = None
        if intent_string:
            intent = intent_string[0]["intent"]
            target = int(intent_string[0]["target"])

        # 抓取实验：优先 tokenizer；失败则用 grab_intent 正则兜底（应对 ASR 错字）
        if intent == "pick" and target in (1, 2):
            pass
        else:
            fb_intent, fb_target = parse_pick_target_fallback(user_input)
            if fb_intent == "pick" and fb_target in (1, 2):
                intent = "pick"
                target = fb_target
                rospy.loginfo("正则兜底抓取 target=%d（原文: %s）", target, user_input)
            else:
                warn = "未识别到抓取一号/二号: %s" % user_input
                self._emit_text("[拒] " + warn)
                rospy.logwarn(
                    "未识别到抓取一号/二号（需含抓/拿/取等 + 一号或二号）: %s",
                    user_input,
                )
                return

        cmd = TagCommand()
        cmd.intent = "pick"
        cmd.target = target
        self.tag_cmd_pub.publish(cmd)
        self._emit_text(
            "[指令] 抓取 tag_%d（将使用 TF 帧 tag_%d）" % (target, target)
        )
        rospy.loginfo("发布 TagCommand intent=%s target=%d", cmd.intent, cmd.target)


if __name__ == "__main__":
    try:
        VoiceControlNode()
        rospy.spin()
    except KeyboardInterrupt:
        print("\nCaught Ctrl+C. Exiting")
