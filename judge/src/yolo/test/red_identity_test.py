#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""红色人物 ID 判定回归测试（不需要 Gazebo / ROS / YOLO）。

背景（2026-09-25 定位，对应 round_20260921_205533）：
    walk_4.dae 与 walk_5.dae 两个"红人"模型的材质完全一样，YOLO 只有一个
    `red` 类；原逻辑"画面最左的红 = red4"在"一次只看到一个红"时永远判 red4，
    于是所有红色速度指令都发给 /actor_4/cmd_vel（实测 21 轮里 actor_4 收到
    指令 16 轮、actor_5 只有 2 轮），裁判侧 actor_5 也永远拿不到分。

测试用的位置来自那一轮的真实日志：
    uav4 世界位置 ≈ (79.7, -24.3) → 离 actor_4(80,-20) 4.3m，离 actor_5 47.6m
    uav5 世界位置 ≈ (108.0, 19.4) → 离 actor_5(110,20) 2.1m，离 actor_4 26.1m

运行：
    python3 src/yolo/test/red_identity_test.py
"""

import importlib.util
import os
import sys
import unittest

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
MODULE_PATH = os.path.join(REPO_ROOT, "src", "yolo", "red_identity.py")


def load_module():
    spec = importlib.util.spec_from_file_location("red_identity", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


RED = load_module()

# 那一轮两个红人的位置（固定图；随机图时由调用方传实时位置）
ACTORS_4_5 = {4: (80.0, -20.0), 5: (110.0, 20.0)}

UAV4_XY = (79.7, -24.3)   # 该轮 uav4 看到红时的世界位置
UAV5_XY = (108.0, 19.4)   # 该轮 uav5 看到红时的世界位置


def old_left_right_rule(red_indices):
    """原来的写法：单框永远 red4；多框按图像 xmin 左->右。"""
    out = {}
    for rank, idx in enumerate(red_indices):
        out[idx] = "red%d" % (4 + rank) if rank < 2 else "red4"
    return out


class SingleRedPositionTest(unittest.TestCase):
    """一次只看到一个红（绝大多数情况）时必须按位置判 ID。"""

    def test_uav4_sees_actor4(self):
        labels = RED.resolve_red_labels([0], UAV4_XY, ACTORS_4_5)
        self.assertEqual(labels, {0: "red4"})
        self.assertEqual(RED.nearest_red_id(UAV4_XY, ACTORS_4_5), 4)

    def test_uav5_sees_actor5(self):
        # 这一轮 uav5 明明在 actor_5 旁边，旧逻辑却给 red4 -> 发到 actor_4 的话题
        labels = RED.resolve_red_labels([0], UAV5_XY, ACTORS_4_5)
        self.assertEqual(labels, {0: "red5"})
        self.assertEqual(RED.nearest_red_id(UAV5_XY, ACTORS_4_5), 5)

    def test_old_rule_is_always_red4(self):
        """旧逻辑的证据：两种情况都判 red4（所以 actor_5 永远收不到指令）。"""
        self.assertEqual(old_left_right_rule([0]), {0: "red4"})
        self.assertEqual(old_left_right_rule([0]), {0: "red4"})
        # 新逻辑区分开
        self.assertNotEqual(
            RED.resolve_red_labels([0], UAV5_XY, ACTORS_4_5)[0],
            old_left_right_rule([0])[0])

    def test_label_digit_is_the_actor_that_gets_the_command(self):
        """tracking 的 targetIdToActorIndex 取标签最后一位 -> 指令发给同一个 actor。"""
        for drone_xy, expected in ((UAV4_XY, 4), (UAV5_XY, 5)):
            label = RED.resolve_red_labels([0], drone_xy, ACTORS_4_5)[0]
            self.assertEqual(int(label[-1]), expected)


class RandomPositionTest(unittest.TestCase):
    """人物位置每轮随机生成：只要传实时位置就能判，不做任何固定坐标假设。"""

    RANDOM_REDS = {4: (12.0, 9.5), 5: (-28.0, 41.0)}

    def test_near_random_red4(self):
        self.assertEqual(RED.nearest_red_id((14.0, 8.0), self.RANDOM_REDS), 4)

    def test_near_random_red5(self):
        self.assertEqual(RED.nearest_red_id((-25.0, 39.0), self.RANDOM_REDS), 5)


class FallbackTest(unittest.TestCase):
    """An unknown identity must not become red4 by default."""

    def test_ambiguous_distance_returns_none(self):
        # 本机在两红中间（到两者距离接近）
        mid = ((80.0 + 110.0) / 2.0, (-20.0 + 20.0) / 2.0)
        self.assertIsNone(RED.nearest_red_id(mid, ACTORS_4_5))
        self.assertEqual(RED.resolve_red_labels([0], mid, ACTORS_4_5), {})

    def test_missing_info_does_not_guess_actor4(self):
        self.assertIsNone(RED.nearest_red_id(None, ACTORS_4_5))
        self.assertIsNone(RED.nearest_red_id(UAV5_XY, {}))
        self.assertEqual(RED.nearest_red_id(UAV5_XY, {4: (80.0, -20.0)}), 4)
        self.assertEqual(RED.resolve_red_labels([0], UAV5_XY, {}), {})

    def test_only_actor5_remains(self):
        self.assertEqual(RED.resolve_red_labels([0], None, {5: (10.0, 20.0)}),
                         {0: "red5"})

    def test_two_boxes_match_projection_even_when_left_right_is_reversed(self):
        labels = RED.resolve_red_labels(
            [3, 7], UAV5_XY, ACTORS_4_5,
            {3: (100.0, 220.0), 7: (500.0, 225.0)},
            {4: (500.0, 225.0), 5: (100.0, 220.0)})
        self.assertEqual(labels, {3: "red5", 7: "red4"})

    def test_two_boxes_use_best_joint_assignment(self):
        labels = RED.resolve_red_labels(
            [3, 7], UAV5_XY, ACTORS_4_5,
            {3: (1.0, 0.0), 7: (-1.0, 0.0)},
            {4: (0.0, 0.0), 5: (4.0, 0.0)})
        self.assertEqual(labels, {3: "red5", 7: "red4"})

    def test_close_distances_do_not_get_forced_to_one_actor(self):
        red_positions = {4: (30.0, 0.0), 5: (31.0, 0.0)}
        self.assertIsNone(RED.nearest_red_id((0.0, 0.0), red_positions))
        self.assertEqual(RED.resolve_red_labels([0], (0.0, 0.0), red_positions), {})

    def test_transform_point_rotates_before_translating(self):
        half_sqrt_two = 2 ** -0.5
        point = RED.transform_point((1.0, 0.0, 0.0), (2.0, 3.0, 4.0),
                                    (0.0, 0.0, half_sqrt_two, half_sqrt_two))
        for actual, expected in zip(point, (2.0, 4.0, 4.0)):
            self.assertAlmostEqual(actual, expected)


class NearDetectionTest(unittest.TestCase):
    def test_clipped_person_matches_nearby_projection(self):
        actors = {4: (4.0, 0.0, 0.0), 5: (30.0, 0.0, 0.0)}
        projected = {4: (320.0, 505.0, 4.0), 5: (320.0, 200.0, 30.0)}
        self.assertEqual(RED.match_near_actor((220, 160, 430, 479),
                                              (0.0, 0.0), actors, projected), 4)

    def test_distant_actor_cannot_authorize_weak_box(self):
        self.assertIsNone(RED.match_near_actor((220, 160, 430, 479),
                                               (0.0, 0.0),
                                               {4: (20.0, 0.0, 0.0)},
                                               {4: (320.0, 240.0, 20.0)}))

    def test_ambiguous_nearby_actors_are_rejected(self):
        self.assertIsNone(RED.match_near_actor((220, 160, 430, 479),
                                               (0.0, 0.0),
                                               {4: (4.0, 0.0, 0.0),
                                                5: (5.0, 0.0, 0.0)},
                                               {4: (320.0, 240.0, 4.0),
                                                5: (325.0, 245.0, 5.0)}))


if __name__ == "__main__":
    unittest.main(verbosity=2)
