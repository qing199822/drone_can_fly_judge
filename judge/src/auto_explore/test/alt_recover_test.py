#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""S10 双向高度控制 + ALT_RECOVER 运行验证（假位姿 + 真 area_mission_node，需要 ROS master）。

复现问题 6：巡逻中持续掉高 -> 跌到 min_safe_altitude 以下 -> 旧代码整机硬停且永不
重新基线化 -> 6 台机全部锁死在 stop=3（S7' 300s 长测 6/6 复现）。

四个用例：
  T1 双向高度控制：pose_z 低于 target-alt_climb_deadband 时必须输出正 z（旧代码恒 0）
  T2 ALT_RECOVER 切入：pose_z 跌破 alt_recover_enter_z 时必须 stop=10 / alt_recover=1，
     且水平指令为 0（先停前向再爬升），不得再出现旧的 permanent stop=3(low_altitude)
  T3 ALT_RECOVER 退出：pose_z 回到 alt_recover_exit_z 以上后 alt_recover 必须回到 0
  T4 低位不再永久锁死：再次把 pose_z 打到 min_safe_altitude 以下，仍应回到 ALT_RECOVER
     （旧代码在这里永久 stop=3，直到测试结束）

用法（先起 roscore）：
    source devel/setup.bash
    python3 src/auto_explore/test/alt_recover_test.py
"""

import argparse
import math
import subprocess
import sys
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

VEHICLE = "s10_alt_check"       # 独立命名空间，避免和用户正在跑的六机仿真互相干扰
TARGET_ALT = 4.5                # 与夹具下发的 _target_altitude 一致
ENTER_Z = 3.5                   # alt_recover_enter_z 默认
EXIT_Z = 4.2                    # target 4.5 时 altRecoverExitZ() = max(3.8, min(4.2, 4.3))
CLIMB_DEADBAND = 0.3


def topics(vehicle):
    return {
        "pose": "/{}/mavros/local_position/pose".format(vehicle),
        # 节点默认订阅 scan_filtered（见 area_mission_node.cpp 的 scan_topic 默认值）
        "scan": "/{}/scan_filtered".format(vehicle),
        "cmd": "/{}/mux_inputs/navigator/cmd_vel".format(vehicle),
        "diag": "/{}/area_mission/diag".format(vehicle),
    }


class PublishLoop(object):
    """持续发布假位姿 + 干净的 2D 激光，z 可被用例随时改写。"""

    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.lock = threading.Lock()
        self.z = TARGET_ALT
        self.stop_event = threading.Event()
        self.thread = None

    def set_z(self, value):
        with self.lock:
            self.z = float(value)

    def get_z(self):
        with self.lock:
            return self.z

    def _run(self):
        topic = topics(self.vehicle)
        pub_pose = rospy.Publisher(topic["pose"], PoseStamped, queue_size=2)
        pub_scan = rospy.Publisher(topic["scan"], LaserScan, queue_size=2)
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            with self.lock:
                z = self.z
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = 0.0
            pose.pose.position.y = 0.0
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            pub_pose.publish(pose)

            scan = LaserScan()
            scan.header.stamp = rospy.Time.now()
            scan.header.frame_id = "laser_2d"
            scan.angle_min = -math.pi
            scan.angle_max = math.pi
            scan.angle_increment = 2.0 * math.pi / 640.0
            scan.range_min = 0.08
            scan.range_max = 20.0
            scan.ranges = [12.0] * 640      # 四周 12m 净空：不触发任何硬停
            pub_scan.publish(scan)
            time.sleep(0.05)

    def start(self):
        self.thread = threading.Thread(target=self._run)
        self.thread.daemon = True
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=2.0)


class DiagSample(object):
    def __init__(self, text):
        self.raw = {}
        self.num = {}
        for token in text.split():
            if "=" not in token:
                continue
            key, _, value = token.partition("=")
            self.raw[key] = value
            try:
                self.num[key] = float(value)
            except ValueError:
                pass

    def __repr__(self):
        return ("mode=%s stop=%s z=%.2f vz=%.2f alt_climb=%s alt_recover=%s reason=%s"
                % (self.raw.get("mode"), self.raw.get("stop"),
                   self.num.get("pose_z", 0.0), self.num.get("vz_pub", 0.0),
                   self.raw.get("alt_climb"), self.raw.get("alt_recover"),
                   self.raw.get("safety_reason")))


def wait_until(samples, predicate, timeout, description, start_index=0):
    deadline = time.time() + timeout
    while time.time() < deadline and not rospy.is_shutdown():
        for sample in samples[start_index:]:
            if predicate(sample):
                return True
        time.sleep(0.05)
    tail = samples[start_index:][-3:]
    print("  [超时] %s（等待 %.1fs）" % (description, timeout))
    for sample in tail:
        print("        最近样本: %s" % sample)
    return False


def main(argv):
    global VEHICLE
    parser = argparse.ArgumentParser()
    parser.add_argument("--vehicle", default=VEHICLE)
    parser.add_argument("--startup", type=float, default=2.0)
    parser.add_argument("--diag-rate", type=float, default=20.0)
    args = parser.parse_args(argv)
    VEHICLE = args.vehicle

    subprocess.call(["rosparam", "delete", "/{}/area_mission_node".format(VEHICLE)],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rospy.init_node("alt_recover_test", anonymous=True, disable_signals=True)
    topic = topics(VEHICLE)

    samples = []
    cmds = []

    def on_diag(message):
        samples.append(DiagSample(message.data))

    def on_cmd(message):
        cmds.append((time.time(), message.linear.x, message.linear.z))

    rospy.Subscriber(topic["diag"], String, on_diag)
    rospy.Subscriber(topic["cmd"], Twist, on_cmd)

    loop = PublishLoop(VEHICLE)
    loop.start()
    time.sleep(args.startup)

    node = subprocess.Popen([
        "rosrun", "auto_explore", "area_mission_node",
        "__ns:=" + VEHICLE,
        "_vehicle:=" + VEHICLE,
        "_target_altitude:=%.1f" % TARGET_ALT,
        "_diag_rate:=%.1f" % args.diag_rate,
    ])

    failures = []

    def check(condition, message):
        print("  [%s] %s" % ("PASS" if condition else "FAIL", message))
        if not condition:
            failures.append(message)

    try:
        # 新 diag 字段必须存在，否则整套断言没有依据
        if not wait_until(samples, lambda s: "pose_z" in s.num, 10.0, "diag 出现 pose_z 字段"):
            failures.append("diag 缺少 pose_z 字段（S10 未生效）")
            raise SystemExit(1)

        # 起跳完成（z=3.0 -> 立即 |zerr|<0.3 进入 MISSION，mode 不再等于 0）
        if not wait_until(samples, lambda s: s.num.get("mode") != 0.0, 15.0,
                          "进入 MISSION（mode!=0）"):
            failures.append("无法进入 MISSION，后续用例无意义")
            raise SystemExit(1)
        print("  已进入 MISSION，起点 z=%.2f" % loop.get_z())

        # ---------------- T1：双向高度控制 ----------------
        print("\n[T1] z 降到 %.2f（< target-%.1f）应输出正 z" % (TARGET_ALT - 0.4, CLIMB_DEADBAND))
        t1 = len(samples)
        loop.set_z(TARGET_ALT - 0.4)          # 2.6
        check(wait_until(samples, lambda s: s.num.get("alt_climb") == 1.0
                         and s.num.get("vz_pub", 0.0) > 0.05, 6.0,
                         "T1 低于目标时输出正 z（alt_climb=1）", t1),
              "T1 低位没有爬升输出（旧代码症状：vz_pub 恒 0）")

        # ---------------- T2：ALT_RECOVER 切入 ----------------
        print("\n[T2] z 打到 %.2f（< enter %.1f）必须进入 ALT_RECOVER" % (2.0, ENTER_Z))
        t2 = len(samples)
        loop.set_z(2.0)
        check(wait_until(samples, lambda s: s.num.get("stop") == 10.0
                         and s.num.get("alt_recover") == 1.0
                         and s.raw.get("safety_reason") == "alt_recover", 8.0,
                         "T2 进入 ALT_RECOVER（stop=10 / alt_recover=1）", t2),
              "T2 没有进入 ALT_RECOVER，或仍然报旧的 low_altitude 硬停")
        # 只给 z、不给前向：v_pub（实际发布的前向速度）=0 且 vz_pub>0
        check(wait_until(samples, lambda s: s.num.get("alt_recover") == 1.0
                         and abs(s.num.get("v_pub", 0.0)) < 1e-3
                         and s.num.get("vz_pub", 0.0) > 0.05, 8.0,
                         "T2 ALT_RECOVER 原地爬升（v_pub=0 且 vz_pub>0）", t2),
              "T2 ALT_RECOVER 没有做到'先停前向、再原地爬升'")

        # ---------------- T3：ALT_RECOVER 退出 ----------------
        print("\n[T3] z 回到 %.2f（>= exit %.1f）应退出 ALT_RECOVER" % (3.0, EXIT_Z))
        t3 = len(samples)
        loop.set_z(3.0)
        check(wait_until(samples, lambda s: s.num.get("alt_recover") == 0.0
                         and s.num.get("stop") != 10.0, 8.0,
                         "T3 退出 ALT_RECOVER、恢复前向", t3),
              "T3 ALT_RECOVER 退不出来（可能变成只会爬升的新死循环）")

        # ---------------- T4：低位不再永久锁死 ----------------
        print("\n[T4] 再次把 z 打到 0.8（< min_safe_altitude=1.0）")
        t4 = len(samples)
        loop.set_z(0.8)
        check(wait_until(samples, lambda s: s.num.get("alt_recover") == 1.0, 8.0,
                         "T4 低位仍然进入 ALT_RECOVER（而不是永久 stop=3）", t4),
              "T4 低位又变成永久停机（旧 bug 回归）")
        loop.set_z(3.0)
        check(wait_until(samples, lambda s: s.num.get("alt_recover") == 0.0, 8.0,
                         "T4 重新爬升后能再次退出恢复", t4),
              "T4 进入 ALT_RECOVER 后出不来")
    finally:
        loop.stop()
        node.terminate()
        time.sleep(0.5)

    if not samples:
        print("FAIL 没有收到 diag（节点没起来或话题不对）")
        return 2
    if failures:
        print("\n结果：FAIL（%d 项）" % len(failures))
        for item in failures:
            print("  - %s" % item)
        return 1
    print("\n结果：PASS（S10 双向高度控制 + ALT_RECOVER 行为符合规格）")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
