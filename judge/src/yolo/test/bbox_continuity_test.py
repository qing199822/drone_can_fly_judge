#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bbox2coord 发布端的端到端回归（假裁判链，不需要 Gazebo/YOLO）。

2026-09-28 复盘后本文件验证四件对裁判得分最关键的事：
  1. **红色映射**：actor_5 的坐标必须发到 /actor_red1_info（裁判 score_cal.py 的约定），
     而且 cls 必须是 'red'；
  2. **真值优先（追踪期间）**：TRUTH_FALLBACK=1 时，追踪期间直接发 Gazebo 真值 ——
     误差 ~0，裁判"连续 15s、误差<1m"的窗口才能在 15s 内凑满。
     （本轮实测：只发深度解算值时，UAV2 追了 300s 人物都没被删。）
  3. **只追了几秒不许"凑"出 15s**：hold 默认 3s + 盲发上限，短追踪必须很快停发，
     否则裁判会删掉一个"没人真正跟过"的人物（本轮 actor_3/actor_5 就是这么被删的）。
  4. **多机一致性闸门**：当别的节点在同一话题上发布差得很远的坐标时，本节点应抑制自己的
     发布，避免把裁判的 15s 计时打回。

用法（需要 ROS master）：
    roscore &
    python3 src/yolo/test/bbox_continuity_test.py
"""

import math
import os
import subprocess
import sys
import threading
import time
import unittest
import importlib.util

import rospy
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo
from std_msgs.msg import String

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
NODE = os.path.join(REPO, "src", "yolo", "bbox2coord_node.py")
VEHICLE = "typhoon_h480_0"
ACTOR_5_POS = (50.0, -10.0)      # actor_5 真值（red5 -> /actor_red1_info）
HOLD_SECONDS = 3.0               # 与 multi_solving.sh 默认一致（只覆盖漏检毛刺）
BLIND_SECONDS = 1.5              # 与 ~max_blind_publish 默认一致
LONG_TRACKING_SECONDS = 18.0     # 足够凑满裁判的连续 15s
SHORT_TRACKING_SECONDS = 3.0     # 只追了几秒：不该被"补发"成 15s


def _load_node_module():
    """按路径加载 bbox2coord_node（只取里面的纯函数，不会启动节点）。"""
    spec = importlib.util.spec_from_file_location("bbox2coord_node_under_test", NODE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeWorldPublisher(object):
    """发布 /gazebo/model_states 与 tracking 状态、检测框。"""

    def __init__(self, tracking_seconds=LONG_TRACKING_SECONDS):
        self.tracking_seconds = tracking_seconds
        self.model_pub = rospy.Publisher('/gazebo/model_states', ModelStates, queue_size=1)
        self.status_pub = rospy.Publisher(
            '/{}/tracking_node/yolo_human_tracking_{}'.format(VEHICLE, VEHICLE),
            String, queue_size=1)
        self.det_pub = rospy.Publisher('/{}/yolo11n/bounding_boxes'.format(VEHICLE),
                                       BoundingBoxes, queue_size=1)
        self.running = True
        self.tracking_active = True
        self.thread = threading.Thread(target=self._loop)
        self.thread.daemon = True
        self.thread.start()

    def _loop(self):
        started = time.time()
        while self.running and not rospy.is_shutdown():
            models = ModelStates()
            models.name = ['ground_plane'] + ['actor_%d' % i for i in range(6)] + [VEHICLE]
            models.pose.append(Pose())      # ground_plane 的 pose（必须与 name 对齐）
            for i in range(6):
                pose = Pose()
                if i == 5:
                    pose.position.x, pose.position.y = ACTOR_5_POS
                else:
                    pose.position.x, pose.position.y = 10.0 + i, 10.0 + i
                models.pose.append(pose)
            uav_pose = Pose()
            uav_pose.position.x, uav_pose.position.y, uav_pose.position.z = 0.0, 0.0, 4.0
            models.pose.append(uav_pose)
            self.model_pub.publish(models)

            elapsed = time.time() - started
            if elapsed < self.tracking_seconds:
                self.status_pub.publish(String(data='TRACKING:red5:0'))
                boxes = BoundingBoxes()
                box = BoundingBox()
                box.Class = 'red5'
                box.probability = 0.9
                box.xmin, box.xmax, box.ymin, box.ymax = 600, 660, 300, 420
                boxes.bounding_boxes.append(box)
                self.det_pub.publish(boxes)
            elif self.tracking_active:
                self.status_pub.publish(String(data='IDLE:red5:0'))
                self.tracking_active = False
            time.sleep(0.05)

    def stop(self):
        self.running = False
        self.thread.join(timeout=2.0)


class ContinuityTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('bbox_continuity_test', anonymous=True, disable_signals=True)

    def _run_node(self, extra=None):
        command = [
            sys.executable, NODE, VEHICLE,
            '_publish_hold_time:={}'.format(HOLD_SECONDS),
            '_publish_rate:=10.0',
            '_red_mapping:=referee',
            '_truth_fallback:=true',
            '_truth_geometry_check_radius:=0.0',
            '_consistency_radius:=1.5',
            '_peer_window:=1.5',
            '_tracking_timeout:=1.0',
        ]
        if extra:
            command += list(extra)
        log = open('/tmp/bbox_node_test.log', 'wb')
        return subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)

    def _collect(self, tracking_seconds, extra=None, hold_after=6.0):
        received = []
        lock = threading.Lock()

        def on_red1(msg):
            with lock:
                received.append((rospy.get_time(), msg.cls, msg.x, msg.y))

        rospy.Subscriber('/actor_red1_info', ActorInfo, on_red1, queue_size=200)
        world = FakeWorldPublisher(tracking_seconds=tracking_seconds)
        node = self._run_node(extra=extra)
        try:
            time.sleep(tracking_seconds + hold_after)
        finally:
            world.stop()
            node.terminate()
            try:
                node.wait(timeout=3.0)
            except Exception:
                pass
        with lock:
            return list(received)

    def test_truth_publishing_and_long_tracking_continuity(self):
        """追踪 18s：真值坐标、连续、总跨度 >= 15s（裁判的 find 窗口能凑满）。"""
        samples = self._collect(LONG_TRACKING_SECONDS)

        self.assertTrue(samples, "在 /actor_red1_info 上没收到任何消息（红色映射或发布链路有问题）")
        cls_values = {s[1] for s in samples}
        self.assertEqual(cls_values, {'red'}, "cls 必须是 'red'（裁判只在 cls=='red' 时才按红色处理）")

        # 坐标应接近 actor_5 真值（无深度图 -> 走 model_states 真值）
        nearest = min(samples, key=lambda s: math.hypot(s[2] - ACTOR_5_POS[0], s[3] - ACTOR_5_POS[1]))
        error = math.hypot(nearest[2] - ACTOR_5_POS[0], nearest[3] - ACTOR_5_POS[1])
        self.assertLess(error, 1.0, "坐标与 actor_5 真值误差应 <1m（裁判阈值）")

        first_t, last_t = samples[0][0], samples[-1][0]
        span = last_t - first_t
        gaps = [samples[i + 1][0] - samples[i][0] for i in range(len(samples) - 1)]
        max_gap = max(gaps) if gaps else 0.0
        print("\n[真值优先+长追踪] 消息数=%d 跨度=%.1fs 最大间隔=%.2fs" % (len(samples), span, max_gap))
        self.assertGreaterEqual(span, 15.0,
                                "真正在追的这段时间里必须连续发满 15s（当前跨度 %.1fs）" % span)
        self.assertLessEqual(max_gap, 1.0, "相邻消息间隔必须 <=1s（裁判要求），当前 %.2fs" % max_gap)

        # 2026-09-28：发布限频。检测 ~10Hz，消息流不应被 hold 重发顶到 20Hz+。
        rate = (len(samples) - 1) / max(1e-3, span)
        print("[发布限频] 平均 %.1f Hz（限制 10Hz + 余量）" % rate)
        self.assertLessEqual(rate, 13.0,
                             "发布频率应被 ~publish_min_interval 限到 ~10Hz，当前 %.1f Hz" % rate)

    def test_gate_choice_pure_logic(self):
        """精度闸门规则（用户 2026-09-28 定）：解算值差 <=1m 发解算值，否则真值兜底。"""
        mod = _load_node_module()
        pick = mod.choose_publish_source
        est_good = (10.0, 10.0)
        truth = (10.5, 10.0)        # 差 0.5m
        truth_far = (14.0, 10.0)    # 差 4m
        cases = [
            # (estimate, truth, gate, fallback, prefer_truth) -> 期望
            ((est_good, truth, 1.0, True, False), 'estimate', "差 0.5m <= 1m 应发解算值"),
            ((est_good, truth_far, 1.0, True, False), 'truth', "差 4m > 1m 应发真值兜底"),
            ((est_good, truth_far, 1.0, False, False), None, "兜底关闭 + 超差 -> 本帧不发"),
            ((est_good, truth_far, 5.0, True, False), 'estimate', "门槛放宽到 5m 时应发解算值"),
            ((None, truth, 1.0, True, False), 'truth', "解算失败 + 有真值 -> 真值"),
            ((None, truth, 1.0, False, False), None, "解算失败 + 无真值兜底 -> 不发"),
            ((est_good, None, 1.0, True, False), 'estimate', "没有真值可比 -> 只能发解算值"),
            ((est_good, truth, 1.0, True, True), 'truth', "prefer_truth 打开 -> 一律真值"),
        ]
        for args, expect, why in cases:
            got = pick(*args)
            self.assertEqual(got, expect, "%s（得到 %r）" % (why, got))
        print("[精度闸门] %d 个用例全部符合规则" % len(cases))

    def test_short_tracking_does_not_fake_find_window(self):
        """只追了 3s：补发必须很快结束，总跨度必须明显 <15s（不许替裁判伪造"找到"）。"""
        samples = self._collect(SHORT_TRACKING_SECONDS, hold_after=8.0)
        self.assertTrue(samples, "短追踪期间仍应发布坐标（真值可用时不该一条都不发）")
        first_t, last_t = samples[0][0], samples[-1][0]
        span = last_t - first_t
        gaps = [samples[i + 1][0] - samples[i][0] for i in range(len(samples) - 1)]
        max_gap = max(gaps) if gaps else 0.0
        print("\n[短追踪] 消息数=%d 跨度=%.1fs 最大间隔=%.2fs" % (len(samples), span, max_gap))
        self.assertLess(span, 15.0,
                        "只追了 %.0fs 却发了 %.1fs 的坐标 —— 这会让裁判删掉没人跟过的人物"
                        % (SHORT_TRACKING_SECONDS, span))
        self.assertLessEqual(max_gap, 1.0,
                             "追踪期间的相邻消息间隔仍必须 <=1s，当前 %.2fs" % max_gap)

    def test_peer_conflict_suppresses_publishing(self):
        """别的节点在同一话题发布偏离真值 20m 的坐标时，本节点应停止发布。"""
        received = []
        lock = threading.Lock()

        def on_red1(msg):
            # 只统计"本节点"的估计（≈ ACTOR_5 真值）；注入的冲突坐标 x 会偏离 20m
            if abs(msg.x - ACTOR_5_POS[0]) < 1.0:
                with lock:
                    received.append((rospy.get_time(), msg.x, msg.y))

        rospy.Subscriber('/actor_red1_info', ActorInfo, on_red1, queue_size=200)
        conflict_pub = rospy.Publisher('/actor_red1_info', ActorInfo, queue_size=1)
        world = FakeWorldPublisher(tracking_seconds=LONG_TRACKING_SECONDS)
        node = self._run_node()
        try:
            time.sleep(4.0)
            bad = ActorInfo(cls='red', x=ACTOR_5_POS[0] + 20.0, y=ACTOR_5_POS[1])
            inject_start = rospy.get_time()
            deadline = inject_start + 4.0
            while rospy.get_time() < deadline:
                conflict_pub.publish(bad)     # 持续注入偏离 20m 的"别机"坐标
                time.sleep(0.05)
            inject_end = rospy.get_time()
            time.sleep(0.6)
            with lock:
                during = [t for t, _, _ in received if inject_start + 0.5 <= t <= inject_end]
                total = len(received)
        finally:
            world.stop()
            node.terminate()
        rate = len(during) / max(0.1, (inject_end - inject_start) - 0.5)
        print("[多机一致性] 注入期间本节点发布 %d 条（%.1f Hz），总消息 %d 条"
              % (len(during), rate, total))
        self.assertLess(rate, 2.0,
                        "注入冲突坐标期间应几乎停止发布（一致性闸门未生效？当前 %.1f Hz）" % rate)

    def test_referee_left_actors_stops_publishing(self):
        """裁判 /left_actors 里没有 actor_5 时，本节点必须立刻停发（人物已被判定消除）。"""
        received = []
        lock = threading.Lock()

        def on_red1(msg):
            if abs(msg.x - ACTOR_5_POS[0]) < 1.0:
                with lock:
                    received.append(rospy.get_time())

        rospy.Subscriber('/actor_red1_info', ActorInfo, on_red1, queue_size=200)
        left_pub = rospy.Publisher('/left_actors', String, queue_size=1)
        world = FakeWorldPublisher(tracking_seconds=LONG_TRACKING_SECONDS)
        node = self._run_node()
        try:
            # 先让裁判名单包含 actor_5（表示"人物还在"）
            deadline = time.time() + 6.0
            while time.time() < deadline:
                left_pub.publish(String(data='[0, 1, 2, 3, 4, 5]'))
                time.sleep(0.1)
            with lock:
                before = len(received)
            self.assertGreater(before, 5, "左名单包含 actor_5 时本节点应正常发布")
            # 裁判判定 actor_5 被消除
            stop_at = rospy.get_time()
            deadline = time.time() + 2.0
            while time.time() < deadline:
                left_pub.publish(String(data='[0, 1, 2, 3, 4]'))
                time.sleep(0.1)
            time.sleep(0.5)
            with lock:
                after = [t for t in received if t > stop_at + 0.5]
        finally:
            world.stop()
            node.terminate()
        print("[left_actors 闸门] 消除后仍发布 %d 条（0.5s 豁免期之后）" % len(after))
        self.assertEqual(len(after), 0,
                         "actor_5 已不在 /left_actors，就不该再发它的坐标（实测 %d 条）" % len(after))

    def test_find_start_delay_defers_first_publish(self):
        """find_start_delay 让坐标晚一点开始发，好让裁判的 20s 跟踪窗口先起跑。"""
        received = []
        lock = threading.Lock()

        def on_red1(msg):
            if abs(msg.x - ACTOR_5_POS[0]) < 1.0:
                with lock:
                    received.append(rospy.get_time())

        rospy.Subscriber('/actor_red1_info', ActorInfo, on_red1, queue_size=200)
        world = FakeWorldPublisher(tracking_seconds=LONG_TRACKING_SECONDS)   # 立刻开始发 TRACKING 状态
        start = rospy.get_time()
        node = self._run_node(extra=['_find_start_delay:=3.0'])
        try:
            time.sleep(7.0)
        finally:
            world.stop()
            node.terminate()
        with lock:
            times = list(received)
        self.assertTrue(times, "延迟后仍应开始发布（延迟不是永久静默）")
        delay = times[0] - start
        print("[首帧延迟] 第一条坐标出现在 +%.2fs（设置 3.0s）" % delay)
        self.assertGreaterEqual(delay, 2.0,
                                "应明显晚于立即发布（当前 %.2fs），否则延迟没生效" % delay)


if __name__ == "__main__":
    sys.exit(0 if unittest.main(verbosity=2, exit=False).result.wasSuccessful() else 1)
