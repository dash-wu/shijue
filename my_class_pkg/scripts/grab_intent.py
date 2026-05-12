#!/usr/bin/env python3
"""从语音识别原文中解析「抓取一号/二号」，不依赖 jieba 分词（应对 ASR 错字、连读）。"""
import re


def normalize_asr_grab(text):
    """常见误识 → 规整写法。"""
    t = re.sub(r"\s+", "", text or "")
    repl = [
        ("抓住", "抓取"),
        ("抓去", "抓取"),
        ("专取", "抓取"),
        ("转取", "抓取"),
        ("装取", "抓取"),
        ("爪取", "抓取"),
    ]
    for a, b in repl:
        t = t.replace(a, b)
    return t


def parse_pick_target_fallback(text):
    """
    若返回 (intent, target) 则 intent 恒为 'pick'，target 为 1 或 2；
    无法解析则返回 (None, None)。
    """
    t = normalize_asr_grab(text)
    if not t:
        return None, None

    # 抓取类动词（含口语）
    verb = re.compile(r"抓|拿|取|拾|要|帮|给")
    if not verb.search(t):
        return None, None

    # 二号优先于「一」+「二」歧义（用更具体的模式）
    if re.search(r"(二号|两号|2号|第\s*二|第二个)", t) and not re.search(
        r"(一号|1号|第一|第一个)", t
    ):
        return "pick", 2
    if re.search(r"(一号|1号|第\s*一|第一个)", t):
        return "pick", 1
    if re.search(r"(二号|两号|2号|第\s*二|第二个)", t):
        return "pick", 2

    # 「抓一号」被识别成「抓一」或带空格
    if re.search(r"抓.{0,2}一", t) and "二" not in t and "2" not in t:
        return "pick", 1
    if re.search(r"抓.{0,2}二", t):
        return "pick", 2

    return None, None
