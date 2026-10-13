#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""yolo11n.py 阶段门控契约（纯文本断言 + 抽函数行为核对；不需要 ROS/GPU/Gazebo）。

2026-09-28：ROAD 段原先完全不做推理（`if mission_stage not in ("PATROL","TRACKING")`）。
上一轮实测 4 号无人机 ROAD 占 179s（其 62% 时间）那一段全盲，是"人物从前面过却看不到"
的原因之一。本文件把下面三件事钉成契约，防止再被改回去：
  ① 阶段列表是**可配置参数** ~yolo_stages，且默认值含 ROAD；
  ② 门控**没有被削弱成"无条件推理"**：不在列表里的阶段仍然直接 return（不推理、不发框）；
  ③ 有 1Hz 的 `stage=… frames=… boxes=… infer_ms=…` 汇总日志，下一轮可从日志直接确认
     "ROAD 段确实出框了"，并用 infer_ms 评估 GPU 负载。

用法：python3 src/yolo/test/yolo_stage_gate_static_test.py
"""
from pathlib import Path
import re

root = Path(__file__).parents[1]                       # .../src/yolo
source = (root / "yolo11n.py").read_text()

# ---------------- ① 可配置参数 + 默认含 ROAD ----------------
assert 'YOLO_STAGES_DEFAULT = "PATROL,TRACKING,ROAD"' in source, \
    "阶段默认值必须是 'PATROL,TRACKING,ROAD'（ROAD 是本轮新增的）"
assert "def parse_yolo_stages(raw):" in source, "阶段串解析必须抽成可测函数"
assert "rospy.get_param('~yolo_stages', YOLO_STAGES_DEFAULT)" in source, \
    "必须用 ~yolo_stages 参数覆盖默认值（旧行为可一键恢复）"
assert "YOLO_STAGES = parse_yolo_stages(rospy.get_param('~yolo_stages'" in source, \
    "运行时必须把参数解析结果赋给 YOLO_STAGES（而不是只读不用）"
# 注释里必须写清 ROAD 原先全盲的来龙去脉 + 恢复旧行为的写法（给下一轮的人看）
assert "ROAD 段完全不做推理" in source
assert 'rosparam set /<vehicle_type>_<id>/yolo_stages "PATROL,TRACKING"' in source, \
    "必须留一行『如何恢复旧行为』的说明"

# 抽 parse_yolo_stages 函数体独立执行（它不依赖 ROS），做真行为核对而不只是文本匹配。
_match = re.search(r"(?ms)^def parse_yolo_stages\(raw\):.*?(?=^\S)", source)
assert _match, "没能抽出 parse_yolo_stages 函数体"
_ns = {}
exec(_match.group(0), _ns)
parse_yolo_stages = _ns["parse_yolo_stages"]
_default = re.search(r'YOLO_STAGES_DEFAULT = "([^"]*)"', source).group(1)
assert "ROAD" in [s.strip() for s in _default.split(",")], "默认值里必须有 ROAD"
assert parse_yolo_stages(_default) == {"PATROL", "TRACKING", "ROAD"}, "默认解析结果错"
# 旧行为可恢复：显式给 "PATROL,TRACKING"（不含 ROAD）→ ROAD 不再推理。
_old = parse_yolo_stages("PATROL,TRACKING")
assert _old == {"PATROL", "TRACKING"} and "ROAD" not in _old, "旧行为无法通过参数恢复"
# 容错：空格/空项/大小写；全空 = 关闭检测（不是悄悄回落到默认）。
assert parse_yolo_stages(" patrol , road ,,") == {"PATROL", "ROAD"}
assert parse_yolo_stages("") == set()

# ---------------- ② 门控没被削弱（不在列表仍然直接 return） ----------------
# 注意：修复说明的注释里会引用旧写法作为"修复前"对照，所以判"旧写法是否还在"时
# 必须先去掉注释（与 area_mission_static_test.py 的 _strip_cpp_comments 同一思路）。
_code = "\n".join(line.split("#", 1)[0] for line in source.splitlines())
assert 'mission_stage not in ("PATROL", "TRACKING")' not in _code, \
    "旧的硬编码门控必须已被参数化替换（否则 ROAD 仍然全盲）"
assert "if mission_stage not in YOLO_STAGES:\n        return\n" in source, \
    "门控必须是『不在列表就直接 return』（不能改成无条件推理）"
_cb = source[source.index("def image_callback"):source.index("# ROS 节点初始化入口")]
assert "if mission_stage not in YOLO_STAGES:" in _cb, "门控必须在 image_callback 内"
assert _cb.index("if mission_stage not in YOLO_STAGES:") < _cb.index("model.predict("), \
    "门控必须在推理之前（不允许先推理再丢弃）"
assert "ros_image_to_cv2(msg)" not in _cb[:_cb.index("if mission_stage not in YOLO_STAGES:")], \
    "门控必须在图像转换/推理之前"
# 阶段名归一化：只取第一个冒号前的段名 + 去空格 + 大写，否则 "STAGE:ROAD:seq" /
# "STAGE:road" 这类报文会匹配不上 → 阶段明明是 ROAD 却仍然不推理（哑火）。
assert 'stage = stage.split(":", 1)[0].strip().upper()' in source, \
    "mission_state_cb 必须把阶段名归一化（取首段 + 大写）"

# ---------------- ③ 1Hz 汇总日志（stage/frames/boxes + 推理耗时） ----------------
assert "import time" in source
assert "def note_inference(stage, n_boxes, infer_ms):" in source
assert "rospy.loginfo_throttle(1.0," in source, \
    "1Hz 限流日志（C++ 的 ROS_INFO_THROTTLE(1.0) 在 rospy 里是 loginfo_throttle）"
assert '"[yolo11n] stage=%s frames=%d boxes=%d infer_ms=%.1f (window=%.1fs)"' in source, \
    "汇总日志必须同时含 stage/frames/boxes/infer_ms"
# 每帧都统计：推理耗时计时 + 两条发布路径（无框早退 / 正常发布）都要记账。
assert "t_infer_start = time.time()" in source
assert "infer_ms = (time.time() - t_infer_start) * 1000.0" in source
assert source.count("note_inference(mission_stage, n_pub, infer_ms)") == 2, \
    "无框早退与正常发布两条路径都必须记 1Hz 统计"
assert "n_pub = publish_boxes_with_hold(boxes_msg)" in source
assert "return len(boxes_msg.bounding_boxes)" in source and \
       "return len(last_valid_boxes_msg.bounding_boxes)" in source, \
    "publish_boxes_with_hold 必须返回实际发布框数（保持帧也要算）"
_hold = source[source.index("def publish_boxes_with_hold("):source.index("def add_synthetic_tracking_box(")]
assert "for box in last_valid_boxes_msg.bounding_boxes:" in _hold and \
       "box.probability = 1.0" in _hold, \
    "漏检保持帧不能被 tracker 误算为新的真实 YOLO 检测"
assert "if box_w < 8 or box_h < 12:" in _cb
assert "if box_w > box_h * 2.5:" not in _cb, \
    "近距离从机头掠过时，宽幅裁切人物框不得按高宽比丢弃"
assert "if distance > 45.0:" in _cb and "skip %s: actor %.1fm" in _cb, \
    "启用真值身份校正时，必须拒绝距离本机过远的错误人物标签"

# ---------------- 跨文件：阶段名必须与任务节点发布的一致 ----------------
_area = root.parents[1] / "src" / "auto_explore" / "src" / "area_mission_node.cpp"
if _area.exists():
    _area_src = _area.read_text()
    assert 'publishStage("ROAD")' in _area_src, \
        "任务节点必须仍然发布 STAGE:ROAD（否则 yolo_stages 里的 ROAD 永远匹配不到）"
    assert '"STAGE:" + stage' in _area_src, "stage 报文前缀变了，mission_state_cb 的解析要同步"

print("yolo 阶段门控契约检查通过（~yolo_stages 默认含 ROAD / 不在列表仍直接 return / "
      "1Hz stage-frames-boxes-infer_ms 汇总日志）")
