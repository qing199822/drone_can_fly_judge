#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""裁判镜像（referee_mirror.py）的纯逻辑回归测试。

不需要 ROS/Gazebo：只测 RefereeCore 的判定规则，并交叉校验红色映射与
真正的裁判系统 score_cal.py 一致（如果裁判改了映射，这个测试会失败提醒我们）。

运行：
    python3 src/yolo/test/referee_mirror_test.py
"""

import importlib.util
import os
import re
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
MIRROR = os.path.join(REPO, "src", "yolo", "referee_mirror.py")
SCORE_CAL = os.path.join(REPO, "score_cal.py")


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


MIRROR_MODULE = load_module(MIRROR, "referee_mirror")


class FindRuleTest(unittest.TestCase):
    """找到并删除人物：误差<1m、间隔<=1s、连续 15s（score_cal.py 规则）。"""

    def setUp(self):
        self.core = MIRROR_MODULE.RefereeCore()
        self.core.actors_pos[0] = (10.0, 20.0)

    def test_found_and_deleted_after_15s(self):
        # 每 0.5s 一条、误差 0.3m，持续 16s
        t = 100.0
        while t < 116.0:
            self.core.process_actor_detection(t, [0], 10.3, 19.8)
            t += 0.5
        self.assertNotIn(0, self.core.left_actors, "连续 15s 后应判定找到并删除")
        self.assertNotIn(0, self.core.find_actors, "应触发过 find")
        # find 50 + target 100（不含传感器扣分）
        self.assertAlmostEqual(self.core.progress_score(sensor_cost=0.0), 150.0, places=3)

    def test_reset_when_error_exceeds_1m(self):
        t = 100.0
        for _ in range(10):                      # 先积累 5s
            self.core.process_actor_detection(t, [0], 10.2, 20.1)
            t += 0.5
        self.assertTrue(self.core.count_flag[0])
        self.core.process_actor_detection(t, [0], 11.7, 20.0)   # 误差 1.7m -> 重置
        self.assertFalse(self.core.count_flag[0])
        self.assertIn("error", self.core.last_reset_reason[0])

    def test_reset_when_message_gap_exceeds_1s(self):
        self.core.process_actor_detection(100.0, [0], 10.1, 20.0)
        self.assertTrue(self.core.count_flag[0])
        self.core.process_actor_detection(102.0, [0], 10.1, 20.0)   # 间隔 2s -> 重置
        self.assertFalse(self.core.count_flag[0])
        self.assertIn("gap", self.core.last_reset_reason[0])

    def test_no_truth_resets_detection(self):
        self.core.actors_pos[0] = None
        self.core.process_actor_detection(100.0, [0], 10.0, 20.0)
        self.assertFalse(self.core.count_flag[0])


class TrackingRuleTest(unittest.TestCase):
    """跟踪分：任一无人机真值在人物 10m 内连续 20s。"""

    def setUp(self):
        self.core = MIRROR_MODULE.RefereeCore()

    def test_tracking_ok_after_20s_within_10m(self):
        self.core.actors_pos[2] = (0.0, 0.0)
        uavs = [None] * 6
        uavs[3] = (9.0, 0.0)          # 9m < 10m
        t = 200.0
        while t < 222.0:
            self.core.update_tracking(t, uavs)
            t += 0.5
        self.assertIn(2, self.core.tracked_actors)

    def test_tracking_resets_when_leaving_range(self):
        self.core.actors_pos[2] = (0.0, 0.0)
        near = [None] * 6
        near[3] = (9.0, 0.0)
        far = [None] * 6
        far[3] = (12.0, 0.0)
        t = 300.0
        for _ in range(20):           # 10s 在范围内
            self.core.update_tracking(t, near)
            t += 0.5
        self.assertIsNotNone(self.core.tracking_start_time[2])
        self.core.update_tracking(t, far)      # 离开 -> 重新计时
        self.assertIsNone(self.core.tracking_start_time[2])
        self.assertNotIn(2, self.core.tracked_actors)


class ScoreTest(unittest.TestCase):
    def test_progress_score_composition(self):
        core = MIRROR_MODULE.RefereeCore()
        # 找到 1 个（find 50）、删除 1 个（target 100）、跟踪 1 个（track 80）
        core.find_actors.remove(0)
        core.left_actors.remove(0)
        core.tracked_actors.add(0)
        score = core.progress_score(sensor_cost=1700.0, uav_loss_count=0)
        self.assertAlmostEqual(score, 50 + 100 + 80 - 1700 * 3e-3, places=3)

    def test_final_score_formula(self):
        core = MIRROR_MODULE.RefereeCore()
        score = core.final_score(120.0, sensor_cost=1700.0, uav_loss_count=1, uav_loss_penalty=100.0)
        self.assertAlmostEqual(score, 2580.0 - 120.0 - 1700 * 3e-3 - 100.0, places=3)


class RedMappingTest(unittest.TestCase):
    """红色映射必须与真正的裁判 score_cal.py 一致（否则红色永远判不到）。"""

    def test_mirror_uses_same_red_mapping_as_score_cal(self):
        with open(SCORE_CAL, encoding='utf-8') as handle:
            cal = handle.read()
        red1 = re.search(r'actor_info1_callback\(msg\):\s*\n\s*_process_actor_detection\(msg,\s*\[(\d+)\]',
                         cal)
        red2 = re.search(r'actor_info2_callback\(msg\):\s*\n\s*_process_actor_detection\(msg,\s*\[(\d+)\]',
                         cal)
        self.assertIsNotNone(red1, "没能从 score_cal.py 解析出 red1 映射")
        self.assertIsNotNone(red2, "没能从 score_cal.py 解析出 red2 映射")
        expected_red1, expected_red2 = int(red1.group(1)), int(red2.group(1))

        with open(MIRROR, encoding='utf-8') as handle:
            mirror = handle.read()
        got_red1 = int(re.search(r"'/actor_red1_info'.*?\[(\d+)\]", mirror, re.S).group(1))
        got_red2 = int(re.search(r"'/actor_red2_info'.*?\[(\d+)\]", mirror, re.S).group(1))
        self.assertEqual((got_red1, got_red2), (expected_red1, expected_red2),
                         "裁判镜像与 score_cal.py 的红色映射不一致")

    def test_bbox2coord_default_mapping_matches_referee(self):
        """发布端默认必须让 actor_5 -> red1、actor_4 -> red2。"""
        with open(os.path.join(REPO, "src", "yolo", "bbox2coord_node.py"),
                  encoding='utf-8') as handle:
            source = handle.read()
        self.assertIn("red_4_topic, red_5_topic = '/actor_red2_info', '/actor_red1_info'", source)
        self.assertIn('rospy.get_param("~red_mapping", "referee")', source)


if __name__ == "__main__":
    sys.exit(0 if unittest.main(verbosity=2, exit=False).result.wasSuccessful() else 1)
