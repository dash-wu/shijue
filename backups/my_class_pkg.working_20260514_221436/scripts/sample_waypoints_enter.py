#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""交互式航点标定：按一次 Enter 记录当前 map→base 的 (X, Y, Yaw°)。

用法：
  rosrun my_class_pkg sample_waypoints_enter.py

按顺序采 6 个点：TASK1 → TASK2 → TASK3 → TASK4 → TASK5 → END。
每按一次回车，就把当前位姿存进下一个名字。

操作键（在脚本终端里输入后按回车）：
  <Enter>  采当前点（默认动作）
  b        撤回上一个采的点
  s        跳过当前点（保留原 yaml 的占位）
  q        退出（不写文件）

采完 6 个点后，脚本会：
  1) 在终端打印完整 yaml 段落，方便复制；
  2) 把生成的 yaml 写到：
     ~/workspace/ex6pcyy/my_class_pkg/config/sampled_waypoints.yaml
  3) 询问是否直接覆盖更新主任务 yaml 里 waypoints + visit_order：
     mission_start_two_tasks_end.yaml
     输入 y 即覆盖；其他键则保留原文件不动。
"""
from __future__ import annotations

import math
import os
import sys
from typing import Dict, List, Optional, Tuple

import rospy
import tf2_ros
from tf_conversions import transformations


WAYPOINT_NAMES: List[str] = ["TASK1", "TASK2", "MID", "TASK3", "TASK4", "TASK5", "END"]

PKG_DIR = "/home/bcsh/workspace/ex6pcyy/my_class_pkg"
CONFIG_DIR = os.path.join(PKG_DIR, "config")
MAIN_YAML = os.path.join(CONFIG_DIR, "mission_start_two_tasks_end.yaml")
OUT_YAML = os.path.join(CONFIG_DIR, "sampled_waypoints.yaml")


def norm_deg(d: float) -> float:
    x = d % 360.0
    if x > 180.0:
        x -= 360.0
    if x < -180.0:
        x += 360.0
    return x


def lookup_xy_yaw(buf: tf2_ros.Buffer, map_f: str, base_f: str) -> Tuple[float, float, float]:
    t = buf.lookup_transform(map_f, base_f, rospy.Time(0), rospy.Duration(0.6))
    p = t.transform.translation
    q = t.transform.rotation
    _, _, yr = transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    return p.x, p.y, norm_deg(math.degrees(yr))


def load_existing_waypoints() -> Dict[str, Tuple[float, float, float]]:
    """从主 yaml 里读出当前已有的 waypoints，方便跳过时保留原值。"""
    if not os.path.isfile(MAIN_YAML):
        return {}
    try:
        import yaml  # 延迟导入
        with open(MAIN_YAML, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        wp = data.get("waypoints", {}) or {}
        out: Dict[str, Tuple[float, float, float]] = {}
        for k, v in wp.items():
            if isinstance(v, (list, tuple)) and len(v) >= 3:
                try:
                    out[str(k)] = (float(v[0]), float(v[1]), float(v[2]))
                except Exception:
                    pass
        return out
    except Exception as ex:
        rospy.logwarn("读现有 yaml 失败: %s", ex)
        return {}


def format_yaml(points: Dict[str, Tuple[float, float, float]],
                existing: Optional[Dict[str, Tuple[float, float, float]]] = None) -> str:
    existing = existing or {}
    lines: List[str] = []
    lines.append("# 自动生成 —— 由 sample_waypoints_enter.py 采样")
    lines.append("waypoints:")
    for name in WAYPOINT_NAMES:
        if name in points:
            x, y, yaw = points[name]
            lines.append("  %s: [%.3f, %.3f, %.2f]" % (name, x, y, yaw))
        elif name in existing:
            x, y, yaw = existing[name]
            lines.append("  %s: [%.3f, %.3f, %.2f]   # 保留原值（本次未采）" % (name, x, y, yaw))
        else:
            lines.append("  # %s: 未采集" % name)
    lines.append("")
    lines.append("visit_order: [%s]" % ", ".join(WAYPOINT_NAMES))
    lines.append("")
    return "\n".join(lines)


def overwrite_main_yaml(points: Dict[str, Tuple[float, float, float]],
                        existing: Optional[Dict[str, Tuple[float, float, float]]] = None) -> bool:
    """覆盖 mission_start_two_tasks_end.yaml 中的 waypoints 与 visit_order，保留其它行。

    简单实现：按行扫描，找到 'waypoints:' 与 'visit_order:'，分别替换它们及其紧随的缩进块。
    """
    if not os.path.isfile(MAIN_YAML):
        rospy.logwarn("未找到主任务 yaml: %s", MAIN_YAML)
        return False

    existing = existing or {}
    with open(MAIN_YAML, "r", encoding="utf-8") as f:
        original = f.read()
    lines = original.splitlines(keepends=False)

    new_lines: List[str] = []
    i = 0
    n = len(lines)
    replaced_waypoints = False
    replaced_order = False
    while i < n:
        line = lines[i]
        stripped = line.lstrip()
        # 处理 waypoints: 段（顶层键）
        if not line.startswith(" ") and stripped.startswith("waypoints:") and not replaced_waypoints:
            new_lines.append("waypoints:")
            for name in WAYPOINT_NAMES:
                if name in points:
                    x, y, yaw = points[name]
                    new_lines.append("  %s: [%.3f, %.3f, %.2f]" % (name, x, y, yaw))
                elif name in existing:
                    x, y, yaw = existing[name]
                    new_lines.append("  %s: [%.3f, %.3f, %.2f]" % (name, x, y, yaw))
            replaced_waypoints = True
            # 跳过原 waypoints 块（顶层下的所有缩进 / 注释行）
            i += 1
            while i < n:
                ln = lines[i]
                if ln.strip() == "":
                    new_lines.append(ln)
                    i += 1
                    continue
                if ln.startswith(" ") or ln.startswith("\t") or ln.lstrip().startswith("#"):
                    i += 1
                    continue
                break
            continue
        # 处理 visit_order: 段
        if not line.startswith(" ") and stripped.startswith("visit_order:") and not replaced_order:
            new_lines.append("visit_order: [%s]" % ", ".join(WAYPOINT_NAMES))
            replaced_order = True
            i += 1
            # visit_order 可能是单行 list 也可能多行，跳过紧随的缩进行
            while i < n:
                ln = lines[i]
                if ln.strip() == "":
                    new_lines.append(ln)
                    i += 1
                    continue
                if ln.startswith(" ") or ln.startswith("\t"):
                    i += 1
                    continue
                break
            continue
        new_lines.append(line)
        i += 1

    if not replaced_waypoints:
        new_lines.append("")
        new_lines.append("waypoints:")
        for name in WAYPOINT_NAMES:
            if name in points:
                x, y, yaw = points[name]
                new_lines.append("  %s: [%.3f, %.3f, %.2f]" % (name, x, y, yaw))
            elif name in existing:
                x, y, yaw = existing[name]
                new_lines.append("  %s: [%.3f, %.3f, %.2f]" % (name, x, y, yaw))
    if not replaced_order:
        new_lines.append("")
        new_lines.append("visit_order: [%s]" % ", ".join(WAYPOINT_NAMES))

    with open(MAIN_YAML, "w", encoding="utf-8") as f:
        f.write("\n".join(new_lines))
        if not new_lines[-1].endswith("\n"):
            f.write("\n")
    return True


def main() -> None:
    rospy.init_node("sample_waypoints_enter", anonymous=False)
    map_f = str(rospy.get_param("~map_frame", "map"))
    base_f = str(rospy.get_param("~base_frame", "base_footprint"))

    buf = tf2_ros.Buffer()
    _lst = tf2_ros.TransformListener(buf)
    rospy.loginfo("等 TF %s → %s（最多 30 秒，请确认 navigation + 初定位 OK）…", map_f, base_f)
    end = rospy.Time.now() + rospy.Duration(30.0)
    while rospy.Time.now() < end and not rospy.is_shutdown():
        try:
            buf.lookup_transform(map_f, base_f, rospy.Time(0), rospy.Duration(0.5))
            break
        except Exception:
            rospy.sleep(0.3)
    else:
        rospy.logfatal("等不到 TF，退出")
        return

    existing = load_existing_waypoints()
    if existing:
        print(">> 检测到主 yaml 已有 %d 个点：%s" % (len(existing), ", ".join(sorted(existing.keys()))))
        print(">> 按 s 跳过的点将保留原值（不会被清掉）")

    points: Dict[str, Tuple[float, float, float]] = {}
    idx = 0

    print("")
    print("=" * 60)
    print(" 航点采集（顺序：%s）" % " → ".join(WAYPOINT_NAMES))
    print("   <Enter> 采当前点    b 撤回    s 跳过(保留原值)    q 退出")
    print("=" * 60)
    print("")

    while idx < len(WAYPOINT_NAMES) and not rospy.is_shutdown():
        name = WAYPOINT_NAMES[idx]
        try:
            x, y, yaw = lookup_xy_yaw(buf, map_f, base_f)
            print(">> 即将记录 [%s]，当前位姿 X=%.3f Y=%.3f Yaw=%.2f°" % (name, x, y, yaw))
        except Exception as ex:
            print(">> [%s] 读 TF 失败：%s（按 Enter 重试）" % (name, ex))

        try:
            cmd = input("[%s] (Enter=采，b=回退，s=跳过，q=退出): " % name).strip().lower()
        except EOFError:
            cmd = "q"
        except KeyboardInterrupt:
            cmd = "q"

        if cmd == "q":
            print(">> 用户退出，不写文件")
            return
        if cmd == "b":
            if idx > 0:
                idx -= 1
                prev = WAYPOINT_NAMES[idx]
                if prev in points:
                    print(">> 撤回到 [%s]（之前值已丢弃）" % prev)
                    del points[prev]
                else:
                    print(">> 撤回到 [%s]" % prev)
            else:
                print(">> 已是第一个点，无法再退")
            continue
        if cmd == "s":
            print(">> 跳过 [%s]" % name)
            idx += 1
            continue

        # 默认（包括空回车）：采当前点
        try:
            x, y, yaw = lookup_xy_yaw(buf, map_f, base_f)
        except Exception as ex:
            print(">> 读 TF 失败：%s，请重试" % ex)
            continue
        points[name] = (x, y, yaw)
        print(">> [%s] = [%.3f, %.3f, %.2f]" % (name, x, y, yaw))
        idx += 1

    print("")
    print("=" * 60)
    yaml_text = format_yaml(points, existing)
    print(yaml_text)
    print("=" * 60)

    try:
        with open(OUT_YAML, "w", encoding="utf-8") as f:
            f.write(yaml_text)
        print("已写出：%s" % OUT_YAML)
    except Exception as ex:
        print("写副本失败：%s" % ex)

    try:
        ans = input("是否直接覆盖更新主任务 yaml (%s)？[y/N]: " % MAIN_YAML).strip().lower()
    except EOFError:
        ans = "n"
    if ans == "y":
        if overwrite_main_yaml(points, existing):
            print("已更新主任务 yaml：%s" % MAIN_YAML)
        else:
            print("覆盖失败，主任务 yaml 未变；可手动从副本复制")
    else:
        print("未覆盖主任务 yaml。需要时把副本里的 waypoints 与 visit_order 复制过去即可")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        sys.exit(0)
