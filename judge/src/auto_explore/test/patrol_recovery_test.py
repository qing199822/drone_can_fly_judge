#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PATROL 恢复契约运行验证（假传感器 + 真 area_mission_node，不需要 Gazebo，需要 ROS master）。

复现 round_20260920_181208（300s 六机实机回归）里的失败模式并验证 P0 v3.1 修复。
本夹具把"无人机 + 房屋"简化成一个合成墙角模型：前方净空随侧移增加而打开
（正是"侧移绕开房屋后视野打开"的物理情形），位姿是一阶运动学积分器，
真正消费节点发布的 /mux_inputs/navigator/cmd_vel。

三个用例：
  T1 恢复空转（旧代码净侧移 <0.1m、从不完成）
     -> 期望：进入 ACTIVE、方向锁定、v_lat_pub 真的非零、净侧移达到 min_progress、成功退出
  T2 TIMEOUT_HOLD 必须保留侧移（旧代码在 state=2 把三个轴全部清零）
     -> 净空打开很慢，4s 超时进入 TIMEOUT_HOLD；期望仍然继续侧移直到净空打开并退出
  T3 位姿异常锁死 + 归因污染（旧代码 stop=3 永久保持、safety_reason 残留上一周期）
     -> 注入 +5m 碰撞类位移；期望 stop=3/abnormal_pose=2 期间保留侧移，
        且位姿稳定 pose_rebaseline_time 后自动重新基线化（不再锁死到结束）
     且全程不允许出现 "stop=0 但 safety_reason != none" 的残留归因。

用法（先起 roscore）：
    source devel/setup.bash
    python3 src/auto_explore/test/patrol_recovery_test.py
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

VEHICLE = "typhoon_h480_0"
SPAWN_X, SPAWN_Y = -17.0, -3.0
# region0 目标在世界系 (-35,-35)；节点内部用 odom（世界 - 出生点）。
TARGET_ODOM = (-35.0 - SPAWN_X, -35.0 - SPAWN_Y)          # (-18, -32)
ALT = 3.0

# 节点默认值（与 area_mission_node.cpp 保持一致，夹具只断言行为，不硬编码内部常量）
MIN_PROGRESS = 0.4
REBASELINE_TIME = 1.0


def topics(vehicle):
    return {
        "pose": "/{}/mavros/local_position/pose".format(vehicle),
        "scan": "/{}/scan".format(vehicle),
        "scan_filtered": "/{}/scan_filtered".format(vehicle),
        "cmd": "/{}/mux_inputs/navigator/cmd_vel".format(vehicle),
        "diag": "/{}/area_mission/diag".format(vehicle),
    }


class WallModel(object):
    """合成墙角：front = base + gain * max(0, y - y_ref)。

    y_ref 是"本次机动起点"的侧向位置；gain 越大，侧移一点净空就打开（= 房屋边缘很近）。
    """

    def __init__(self, y_ref, gain, base=2.5):
        self.lock = threading.Lock()
        self.y_ref = y_ref
        self.gain = gain
        self.base = base

    def set_phase(self, gain, y_ref):
        with self.lock:
            self.gain = gain
            self.y_ref = y_ref

    def front(self, y):
        with self.lock:
            gain, y_ref, base = self.gain, self.y_ref, self.base
        return min(12.0, base + gain * max(0.0, y - y_ref))


def build_scan(front):
    message = LaserScan()
    message.header.stamp = rospy.Time.now()
    message.header.frame_id = "laser_2d_0"
    message.angle_min = -math.pi
    message.angle_max = math.pi
    message.angle_increment = 2.0 * math.pi / 640.0
    message.range_min = 0.08
    message.range_max = 20.0
    ranges = []
    for index in range(640):
        angle = message.angle_min + index * message.angle_increment
        degrees = abs(math.degrees(angle))
        if degrees <= 15.0:
            ranges.append(front)          # 正前方：房屋面
        elif 70.0 <= degrees <= 110.0:
            ranges.append(8.0)            # 左右 ±90° 扇区：净空（两个逃逸方向都可用）
        else:
            ranges.append(12.0)
    message.ranges = ranges
    return message


class Plant(object):
    """一阶运动学：真正消费节点发布的 cmd_vel（只积分 vx/vy，忽略 yaw 力矩）。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.x, self.y, self.z = TARGET_ODOM[0], TARGET_ODOM[1], ALT
        self.vx = self.vy = 0.0
        self.offset_x = 0.0            # 注入的碰撞位移（模拟被撞出 5m）
        self.last_cmd = 0.0

    def on_cmd(self, message):
        with self.lock:
            self.vx = message.linear.x
            self.vy = message.linear.y
            self.last_cmd = time.time()

    def step(self, dt):
        with self.lock:
            if time.time() - self.last_cmd > 0.3:
                self.vx = self.vy = 0.0         # 指令超时保底，避免"幽灵运动"
            self.x += self.vx * dt
            self.y += self.vy * dt
            return self.x + self.offset_x, self.y, self.z


def publish_loop(plant, wall, stop_event):
    pub_pose = rospy.Publisher(topics(VEHICLE)["pose"], PoseStamped, queue_size=2)
    pub_scan = rospy.Publisher(topics(VEHICLE)["scan"], LaserScan, queue_size=2)
    pub_filtered = rospy.Publisher(topics(VEHICLE)["scan_filtered"], LaserScan, queue_size=2)
    last = time.time()
    while not stop_event.is_set() and not rospy.is_shutdown():
        now = time.time()
        dt = min(0.2, now - last)
        last = now
        x, y, z = plant.step(dt)
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        pub_pose.publish(pose)
        scan = build_scan(wall.front(y))
        pub_scan.publish(scan)
        pub_filtered.publish(scan)
        time.sleep(0.05)


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
        return ("rec=%s dir=%s prog=%.2f lat=%.2f stop=%s reason=%s abn=%s/%s" % (
            self.raw.get("recovery_state"), self.raw.get("escape_dir"),
            self.num.get("side_progress", 0.0), self.num.get("v_lat_pub", 0.0),
            self.raw.get("stop"), self.raw.get("safety_reason"),
            self.raw.get("abnormal_pose"), self.raw.get("abnormal_cycles")))


def wait_until(samples, predicate, timeout, description, start_index=0):
    deadline = time.time() + timeout
    while time.time() < deadline and not rospy.is_shutdown():
        for sample in samples[start_index:]:
            if predicate(sample):
                return True
        time.sleep(0.1)
    print("  [超时] %s（等待 %.1fs）" % (description, timeout))
    tail = samples[start_index:][-4:]
    for sample in tail:
        print("        最近样本: %s" % sample)
    return False


def main(argv):
    global VEHICLE
    parser = argparse.ArgumentParser()
    parser.add_argument("--vehicle", default=VEHICLE)
    parser.add_argument("--startup", type=float, default=2.0,
                        help="先喂几秒假传感器再起节点（与 road_speed_floor_test 一致）")
    parser.add_argument("--diag-rate", type=float, default=10.0)
    args = parser.parse_args(argv)

    VEHICLE = args.vehicle

    # rosrun 的 _param:= 会残留在参数服务器上，跨进程污染后续运行。
    subprocess.call(["rosparam", "delete", "/{}/area_mission_node".format(VEHICLE)],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    rospy.init_node("patrol_recovery_test", anonymous=True, disable_signals=True)
    topic = topics(VEHICLE)

    plant = Plant()
    wall = WallModel(plant.y, gain=4.0)
    samples = []

    def on_diag(message):
        samples.append(DiagSample(message.data))

    rospy.Subscriber(topic["diag"], String, on_diag)
    rospy.Subscriber(topic["cmd"], Twist, plant.on_cmd)

    stop_event = threading.Event()
    thread = threading.Thread(target=publish_loop, args=(plant, wall, stop_event))
    thread.daemon = True
    thread.start()
    time.sleep(args.startup)

    node = subprocess.Popen([
        "rosrun", "auto_explore", "area_mission_node",
        "__ns:=" + VEHICLE,
        "_vehicle:=" + VEHICLE,
        "_target_altitude:=%.1f" % ALT,
        "_diag_rate:=%.1f" % args.diag_rate,
        "_wp_reach_tolerance:=3.0",
    ])

    failures = []

    def check(condition, message):
        print("  [%s] %s" % ("PASS" if condition else "FAIL", message))
        if not condition:
            failures.append(message)

    try:
        # ---------------- 先确认真的进了 PATROL ----------------
        if not wait_until(samples, lambda s: s.num.get("mode") == 2.0, 15.0,
                          "节点进入 PATROL（mode=2）"):
            failures.append("无法进入 PATROL，后续用例无意义")
            raise SystemExit(1)
        print("  已进入 PATROL，起点 odom=(%.1f,%.1f)" % (plant.x, plant.y))

        # ---------------- T1：恢复必须真的侧移并完成 ----------------
        print("\n[T1] 前方 2.5m 房屋、侧移 1m 后净空打开（gain=4.0）")
        wall.set_phase(gain=4.0, y_ref=plant.y)
        t1_start = len(samples)
        check(wait_until(samples, lambda s: s.num.get("patrol_rec") == 1.0, 8.0,
                         "T1 恢复进入 ACTIVE", t1_start), "T1 恢复未触发")
        check(wait_until(samples, lambda s: abs(s.num.get("v_lat_pub", 0.0)) > 0.2, 6.0,
                         "T1 真实侧向输出非零", t1_start),
              "T1 侧向速度没有真的发出去（v_lat_pub 恒 0）")
        check(wait_until(samples, lambda s: s.num.get("side_progress", 0.0) >= MIN_PROGRESS, 8.0,
                         "T1 净侧移达到 min_progress=%.1fm" % MIN_PROGRESS, t1_start),
              "T1 净侧移不足（旧版本症状：恒在 ±0.1m 抖动）")
        check(wait_until(samples, lambda s: s.num.get("patrol_rec") == 0.0, 12.0,
                         "T1 恢复成功退出（ACTIVE->IDLE）", t1_start),
              "T1 恢复没有退出（旧版本症状：IDLE/ACTIVE/TIMEOUT_HOLD 空转）")
        dirs = {int(s.num.get("escape_dir", 0)) for s in samples[t1_start:]}
        check(dirs.issubset({-1, 0, 1}), "T1 逃逸方向取值合法 %s" % dirs)

        # ---------------- T2：TIMEOUT_HOLD 必须保留侧移 ----------------
        print("\n[T2] 净空打开很慢（gain=1.0）-> 4s 超时进 TIMEOUT_HOLD")
        wall.set_phase(gain=1.0, y_ref=plant.y)
        t2_start = len(samples)
        check(wait_until(samples, lambda s: s.num.get("patrol_rec") == 1.0, 8.0,
                         "T2 恢复进入 ACTIVE", t2_start), "T2 恢复未触发")
        in_hold = wait_until(samples, lambda s: s.num.get("recovery_state") == 2.0, 10.0,
                             "T2 进入 TIMEOUT_HOLD", t2_start)
        check(in_hold, "T2 没有观察到 TIMEOUT_HOLD（净空打开太快，用例无效）")
        if in_hold:
            lat_in_hold = [abs(s.num.get("v_lat_pub", 0.0)) for s in samples[t2_start:]
                           if s.num.get("recovery_state") == 2.0]
            check(max(lat_in_hold or [0.0]) > 0.2,
                  "T2 TIMEOUT_HOLD 期间仍在侧移（旧版本在此把三轴全部清零）")
        check(wait_until(samples, lambda s: s.num.get("patrol_rec") == 0.0
                         and s.num.get("recovery_state") == 0.0, 20.0,
                         "T2 净空打开后退出", t2_start), "T2 恢复没有退出")

        # ---------------- T3：位姿异常不再锁死 + 保留侧移 ----------------
        print("\n[T3] 恢复进行中注入 +5m 碰撞类位姿跳变")
        wall.set_phase(gain=0.5, y_ref=plant.y)
        t3_start = len(samples)
        check(wait_until(samples, lambda s: s.num.get("patrol_rec") == 1.0, 8.0,
                         "T3 恢复进入 ACTIVE（用于注入跳变）", t3_start), "T3 恢复未触发")
        plant.offset_x = 5.0
        jump_seen = wait_until(samples, lambda s: s.num.get("stop") == 3.0
                               and s.num.get("abnormal_pose") == 2.0, 5.0,
                               "T3 出现 stop=3 + abnormal_pose=2", t3_start)
        check(jump_seen, "T3 没有观察到碰撞类位姿异常归因")
        if jump_seen:
            escaping = [abs(s.num.get("v_lat_pub", 0.0)) for s in samples[t3_start:]
                        if s.num.get("stop") == 3.0 and s.num.get("recovery_state") == 1.0]
            check(max(escaping or [0.0]) > 0.2,
                  "T3 位姿异常期间保留恢复侧移（旧版本被 publishImmediateStop 抢占）")
            check(wait_until(samples, lambda s: s.num.get("stop") != 3.0, 6.0,
                             "T3 位姿稳定后自动重新基线化、不再锁死", t3_start),
                  "T3 stop=3 仍然锁死（旧版本症状：一直保持到测试结束）")

        # ---------------- 归因污染（全时段） ----------------
        print("\n[T4] safety_reason 每周期归零（全时段不变量）")
        polluted = [s for s in samples
                    if s.num.get("stop") == 0.0 and s.raw.get("safety_reason") not in (None, "none", "")]
        check(not polluted, "没有 'stop=0 却 safety_reason!=none' 的残留归因（找到 %d 条）"
              % len(polluted))
        check(all(s.raw.get("abnormal_cycles") is not None for s in samples[-5:]),
              "新诊断字段 abnormal_cycles 已在发布")

        # ---------------- 汇总 ----------------
        changes = [s.num.get("recovery_dir_changes", 0.0) for s in samples]
        print("\n汇总：样本 %d 条；escape_dir 取值 %s；recovery_dir_changes 峰值 %d"
              % (len(samples),
                 sorted({s.raw.get("escape_dir") for s in samples}),
                 int(max(changes or [0]))))
        print("      v_lat_pub 样本数(非零) = %d，最大 |v_lat_pub| = %.2f"
              % (sum(1 for s in samples if abs(s.num.get("v_lat_pub", 0.0)) > 0.05),
                 max([abs(s.num.get("v_lat_pub", 0.0)) for s in samples] or [0.0])))
    finally:
        stop_event.set()
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
    print("\n结果：PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
