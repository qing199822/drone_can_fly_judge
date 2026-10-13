#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_upper_stack.py 的回归测试（需要可用的 ROS master）。

做法：本脚本自己扮演"上层栈"——发布假的 scan_filtered / map / 静态 TF，
然后以子进程方式运行 check_upper_stack.py，验证：
    1. 三样都齐 -> 退出码 0；
    2. 停掉发布后 -> 超时退出码 2（不会假通过）。

运行（需要 roscore）：
    roscore &                      # 或用已有的 master
    python3 src/xtdrone_mavros_bridge/test/check_upper_stack_test.py
"""

import os
import subprocess
import sys
import threading
import time
import unittest

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
CHECKER = os.path.join(REPO_ROOT, "src", "xtdrone_mavros_bridge", "scripts",
                       "check_upper_stack.py")
DRONE_INDEX = 2


def make_scan():
    message = LaserScan()
    message.header.frame_id = "laser_2d_{}".format(DRONE_INDEX)
    message.header.stamp = rospy.Time.now()
    message.angle_min = -3.14
    message.angle_max = 3.14
    message.angle_increment = 0.01
    message.range_min = 0.08
    message.range_max = 20.0
    message.ranges = [3.0] * 628
    return message


def make_map():
    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.header.stamp = rospy.Time.now()
    message.info.width = 16
    message.info.height = 16
    message.info.resolution = 0.5
    message.data = [0] * (16 * 16)
    return message


def run_checker(timeout_seconds):
    command = [
        sys.executable, CHECKER,
        "--num-drones", "1",
        "--start-index", str(DRONE_INDEX),
        "--timeout", str(timeout_seconds),
    ]
    return subprocess.run(command, capture_output=True, text=True, timeout=timeout_seconds + 30)


class CheckerTest(unittest.TestCase):
    """每个用例独立启停发布者，避免用例间状态互相污染。"""

    @classmethod
    def setUpClass(cls):
        rospy.init_node("check_upper_stack_test", anonymous=True, disable_signals=True)
        cls.broadcaster = tf2_ros.StaticTransformBroadcaster()
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link_{}".format(DRONE_INDEX)
        transform.transform.rotation.w = 1.0
        cls.broadcaster.sendTransform(transform)
        time.sleep(1.0)

    def setUp(self):
        self.running = True
        self.scan_publisher = rospy.Publisher(
            "/typhoon_h480_{}/scan_filtered".format(DRONE_INDEX), LaserScan, queue_size=2)
        self.map_publisher = rospy.Publisher("/map", OccupancyGrid, queue_size=2, latch=True)
        self.thread = threading.Thread(target=self._publish_loop)
        self.thread.daemon = True
        self.thread.start()
        time.sleep(2.0)

    def tearDown(self):
        self.running = False
        self.thread.join(timeout=3.0)
        self.scan_publisher.unregister()
        self.map_publisher.unregister()
        time.sleep(1.0)

    def _publish_loop(self):
        try:
            while self.running and not rospy.is_shutdown():
                self.scan_publisher.publish(make_scan())
                self.map_publisher.publish(make_map())
                time.sleep(0.1)
        except rospy.exceptions.ROSException:
            pass  # 用例结束时的 unregister 竞态，忽略

    def test_passes_when_upper_stack_is_healthy(self):
        result = run_checker(30)
        self.assertEqual(result.returncode, 0, msg=result.stdout + result.stderr)
        self.assertIn("健康门通过", result.stdout)

    def test_fails_when_scan_is_missing(self):
        # 停掉发布者，模拟"激光过滤挂掉"，检查器必须超时判失败而不是假通过。
        self.running = False
        self.thread.join(timeout=3.0)
        self.scan_publisher.unregister()
        self.map_publisher.unregister()
        time.sleep(1.0)
        result = run_checker(5)
        self.assertEqual(result.returncode, 2, msg=result.stdout + result.stderr)
        self.assertIn("FAIL", result.stdout)


if __name__ == "__main__":
    sys.exit(0 if unittest.main(verbosity=2, exit=False).result.wasSuccessful() else 1)
