#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
《语音交互与智能问答》实验：语音指令解码（tokenizer）

流程概要（与教材一致）：
  1. 预处理 pre_process：去空白、中文数字转阿拉伯数字、jieba 分词、去停用词、水果名映射
  2. 意图抽取 extract_intent：在词序列中匹配「移动/抓取/放下」等触发词，并解析目标编号

说明：教材印刷稿中部分代码存在笔误（键名空格、正则写法等），本文件在保持相同结构的前提下
做了语法修正，并补充「一号/二号」分词歧义处理，供 /speech/result → /voice_control 使用。
"""
import re

import jieba


class Tokenizer:
    """语音指令分词与意图抽取（jieba + 规则）。"""

    # 停用词表
    stopwords = set(["的", "是", "啊"])

    # 与教材 json_template 对应：intent + target（目标编号，未知时为 -1）
    json_template = {"intent": "", "target": -1}

    # 中文数字 → 数值（用于 replace_chinese_numbers / chinese_to_arabic_number）
    chinese_to_arabic = {
        "零": 0,
        "一": 1,
        "二": 2,
        "三": 3,
        "四": 4,
        "五": 5,
        "六": 6,
        "七": 7,
        "八": 8,
        "九": 9,
        "十": 10,
        "百": 100,
        "千": 1000,
        "万": 10000,
        "亿": 100000000,
    }

    # 水果中英文对照（教材示例；便于分词后统一成英文 token）
    fruit_name_mapping = {
        "苹果": "apple",
        "香蕉": "banana",
        "橙子": "orange",
        "橘子": "orange",
        "草莓": "strawberry",
        "西瓜": "watermelon",
        "菠萝": "pineapple",
    }

    def __init__(self):
        print("Init Tokenizer!!!!")

    def chinese_to_arabic_number(self, chinese_num):
        """由汉字数字串转整数（实验用，支持常见组合如十一、二十三等）。"""
        if not chinese_num:
            return 0
        if chinese_num == "十":
            return 10
        if chinese_num.startswith("十") and len(chinese_num) == 2:
            return 10 + self.chinese_to_arabic.get(chinese_num[1], 0)
        if chinese_num.endswith("十") and len(chinese_num) == 2:
            return self.chinese_to_arabic.get(chinese_num[0], 0) * 10
        if "十" in chinese_num and len(chinese_num) == 3:
            return (
                self.chinese_to_arabic.get(chinese_num[0], 0) * 10
                + self.chinese_to_arabic.get(chinese_num[2], 0)
            )
        total = 0
        r = 1
        for i in range(len(chinese_num) - 1, -1, -1):
            val = self.chinese_to_arabic.get(chinese_num[i], 0)
            if val in (10000, 100000000):
                if r < val:
                    r *= val
                else:
                    r //= val
            elif val in (10, 100, 1000):
                r *= val
            else:
                total += r * val
        return total

    def replace_chinese_numbers(self, text):
        """将中文数字替换为阿拉伯数字；并先把「一号」「二号」等换成 1号、2号 便于分词。"""
        for cn, ar in [
            ("一", "1"),
            ("二", "2"),
            ("三", "3"),
            ("四", "4"),
            ("五", "5"),
            ("六", "6"),
            ("七", "7"),
            ("八", "8"),
            ("九", "9"),
            ("十", "10"),
        ]:
            text = text.replace(cn + "号", ar + "号")
        pattern = re.compile(r"零?[一二三四五六七八九十百千万亿]+")
        for match in pattern.findall(text):
            try:
                arabic_number = self.chinese_to_arabic_number(match)
                text = text.replace(match, str(arabic_number), 1)
            except Exception:
                continue
        return text

    def pre_process(self, text):
        """命令字符串预处理：去空白、数字规范化、分词、去停用词、水果名替换。"""
        text = re.sub(r"\s+", "", text or "")
        text = self.replace_chinese_numbers(text)
        text = text.lower()
        tokens = list(jieba.lcut(text))
        filtered_tokens = [t for t in tokens if t not in self.stopwords and t.strip()]
        for i, tok in enumerate(filtered_tokens):
            if tok in self.fruit_name_mapping:
                filtered_tokens[i] = self.fruit_name_mapping[tok]
        return filtered_tokens

    @staticmethod
    def _token_to_id(tok):
        if tok.isdigit():
            return int(tok)
        m = re.match(r"^(\d+)号$", tok)
        if m:
            return int(m.group(1))
        return None

    def extract_intent(self, tokens):
        """
        从分词结果中提取意图与目标 ID，返回字典列表（教材中为 JSON 列表）。
        意图键名：go_to / pick / release，与 TagCommand + 抓取实验一致。
        """
        intents = {
            "go_to": ["移动", "去", "前往"],
            "pick": ["抓取", "拿起", "抓", "拿", "取"],
            "release": ["放下", "放到", "放", "放置"],
        }
        result = []
        i = 0
        while i < len(tokens):
            matched_intent = None
            for intent, triggers in intents.items():
                if tokens[i] in triggers:
                    matched_intent = intent
                    break
            if matched_intent:
                target_index = -1
                for j in range(i + 1, len(tokens)):
                    tid = self._token_to_id(tokens[j])
                    if tid is not None:
                        target_index = tid
                        break
                # 「抓取一号」常被切成「抓取」「一」「号」——在全句中补目标
                if matched_intent == "pick" and target_index == -1:
                    for tok in tokens:
                        tid = self._token_to_id(tok)
                        if tid in (1, 2):
                            target_index = tid
                            break
                if matched_intent == "pick" and target_index == -1:
                    joined = "".join(tokens)
                    m = re.search(r"(?:抓取|拿起|抓|拿|取).*?([12])\s*号", joined)
                    if m:
                        target_index = int(m.group(1))
                obj = self.json_template.copy()
                obj["intent"] = matched_intent
                obj["target"] = target_index
                result.append(obj)
            i += 1
        return result

    def get_intent_from_text(self, text):
        """整句文本 → 预处理 → 意图列表（教材中的组合接口）。"""
        filtered_input = self.pre_process(text)
        return self.extract_intent(filtered_input)


if __name__ == "__main__":
    tokenizer = Tokenizer()
    user_input = input("请输入指令:")
    filtered_input = tokenizer.pre_process(user_input)
    intent_string = tokenizer.extract_intent(filtered_input)
    print("Output:", intent_string)
