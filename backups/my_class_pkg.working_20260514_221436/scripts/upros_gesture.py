#!/usr/bin/env python3
"""MediaPipe 手部 21 关键点 + 实验讲义中的数字 0~6 规则判断。"""

import cv2
import mediapipe as mp
import math


class UP_Gesture:
    def __init__(self):
        self.draw = mp.solutions.drawing_utils
        self.hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.75,
            min_tracking_confidence=0.75,
        )

    def findHind(self, img):
        """检测手并绘制骨架，返回 multi_hand_landmarks（与课程命名一致）。"""
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        handlmsstyle = self.draw.DrawingSpec(color=(0, 0, 255), thickness=5)
        handconstyle = self.draw.DrawingSpec(color=(0, 255, 0), thickness=5)
        results = self.hands.process(img_rgb)
        if results.multi_hand_landmarks:
            for hand_lms in results.multi_hand_landmarks:
                self.draw.draw_landmarks(
                    img,
                    hand_lms,
                    mp.solutions.hands.HAND_CONNECTIONS,
                    handlmsstyle,
                    handconstyle,
                )
        return results.multi_hand_landmarks

    def detectNumber(self, hand_landmarks, img):
        if not hand_landmarks:
            return -1
        h, w, _ = img.shape
        lm = hand_landmarks[0].landmark

        thumb_tip_id = 4
        index_tip_id = 8
        middle_tip_id = 12
        ring_tip_id = 16
        pinky_tip_id = 20
        pinky_mcp_id = 17
        wrist_id = 0

        thumb_tip_y = lm[thumb_tip_id].y * h
        index_tip_y = lm[index_tip_id].y * h
        middle_tip_y = lm[middle_tip_id].y * h
        ring_tip_y = lm[ring_tip_id].y * h
        pinky_tip_y = lm[pinky_tip_id].y * h
        pinky_mcp_y = lm[pinky_mcp_id].y * h
        wrist_y = lm[wrist_id].y * h

        thumb_tip_x = lm[thumb_tip_id].x * w
        index_tip_x = lm[index_tip_id].x * w
        middle_tip_x = lm[middle_tip_id].x * w
        ring_tip_x = lm[ring_tip_id].x * w
        pinky_tip_x = lm[pinky_tip_id].x * w
        pinky_mcp_x = lm[pinky_mcp_id].x * w
        wrist_x = lm[wrist_id].x * w

        dist_thumb2wrist = math.hypot(thumb_tip_x - wrist_x, thumb_tip_y - wrist_y)
        if dist_thumb2wrist < 1e-6:
            return -1

        dist_index2wrist = math.hypot(index_tip_x - wrist_x, index_tip_y - wrist_y)
        dist_middle2wrist = math.hypot(middle_tip_x - wrist_x, middle_tip_y - wrist_y)
        dist_ring2wrist = math.hypot(ring_tip_x - wrist_x, ring_tip_y - wrist_y)
        dist_pinky2wrist = math.hypot(pinky_tip_x - wrist_x, pinky_tip_y - wrist_y)
        # 与讲义代码一致：拇指尖到小指掌指关节的距离
        dist_pinky_mcp2wrist = math.hypot(
            thumb_tip_x - pinky_mcp_x, thumb_tip_y - pinky_mcp_y
        )

        dist_index2wrist_ratio = dist_index2wrist / dist_thumb2wrist
        dist_middle2wrist_ratio = dist_middle2wrist / dist_thumb2wrist
        dist_ring2wrist_ratio = dist_ring2wrist / dist_thumb2wrist
        dist_pinky2wrist_ratio = dist_pinky2wrist / dist_thumb2wrist
        dist_pinky_mcp2wrist_ratio = dist_pinky_mcp2wrist / dist_thumb2wrist

        if (
            dist_index2wrist_ratio < 1.9
            and dist_middle2wrist_ratio < 1.8
            and dist_ring2wrist_ratio < 1.6
            and dist_pinky2wrist_ratio < 1.4
            and dist_pinky_mcp2wrist_ratio < 0.8
        ):
            return 0
        if (
            2.0 < dist_index2wrist_ratio
            and dist_middle2wrist_ratio < 1.8
            and dist_ring2wrist_ratio < 1.6
            and dist_pinky2wrist_ratio < 1.4
            and dist_pinky_mcp2wrist_ratio < 0.8
        ):
            return 1
        if (
            2.0 < dist_index2wrist_ratio
            and 1.9 < dist_middle2wrist_ratio
            and dist_ring2wrist_ratio < 1.6
            and dist_pinky2wrist_ratio < 1.4
            and dist_pinky_mcp2wrist_ratio < 0.8
        ):
            return 2
        if (
            2.0 < dist_index2wrist_ratio
            and 1.9 < dist_middle2wrist_ratio
            and 1.75 < dist_ring2wrist_ratio
            and dist_pinky2wrist_ratio < 1.4
            and dist_pinky_mcp2wrist_ratio < 0.8
        ):
            return 3
        if (
            2.0 < dist_index2wrist_ratio
            and 1.9 < dist_middle2wrist_ratio
            and 1.75 < dist_ring2wrist_ratio
            and 1.5 < dist_pinky2wrist_ratio
            and dist_pinky_mcp2wrist_ratio < 0.8
        ):
            return 4
        if (
            dist_index2wrist_ratio > 0.5
            and dist_middle2wrist_ratio > 0.5
            and dist_ring2wrist_ratio > 0.5
            and 0.9 < dist_pinky_mcp2wrist_ratio < 1.2
        ):
            return 5
        if (
            dist_index2wrist_ratio < 0.5
            and dist_middle2wrist_ratio < 0.5
            and dist_ring2wrist_ratio < 0.5
        ):
            return 6
        return -1
