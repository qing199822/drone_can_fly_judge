#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PATROL hover-skip + 线推进 运行验证（假位姿 + 真 area_mission_node，需要 ROS master）。

复现问题 10：MPPI 与蛇形探测航点冲突 → 5 号在 round_20260921_212904 进入
PATROL 后 v_pub 持续≈0 长达 134s，stall_count 全程=0，recovery 在 state=1↔2
之间反复打转。修复后要求：

  - 显式开启跳点时，悬停超过 effective_hover_deadline 后跳过当前点；
  - wp_idx_ 按四航点固定队列推进：(cur + 1) % 4；
  - 已到达当前点（dist < wp_reach_tolerance_）不触发；
  - 前方净空充足（无障碍）不触发。

用法（先起 roscore）：
    source devel/setup.bash
    python3 src/auto_explore/test/patrol_hover_line_advance_test.py
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

VEHICLE = "hover_skip_check"          # 独立命名空间，避免和用户正在跑的六机仿真互相干扰
TARGET_ALT = 4.5                      # 与 1.sh launch 一致
# ROAD 阶段的目标（drone_id=0 时）—— 用它进 PATROL 比钉 patrol_waypoints_[0] 更稳更快。
ROAD_TARGET_X_ODOM = -18.0            # drone0 spawn offset (-17,-3) + world target (-35,-35)
ROAD_TARGET_Y_ODOM = -32.0
WP_REACH_TOLERANCE = 2.0
ENTER_FAR_X = ROAD_TARGET_X_ODOM + 15.0   # 距 ROAD 目标 >10m
ENTER_FAR_Y = ROAD_TARGET_Y_ODOM


def topics(vehicle):
    return {
        "pose": "/{}/mavros/local_position/pose".format(vehicle),
        "scan": "/{}/scan_filtered".format(vehicle),
        "cmd":  "/{}/mux_inputs/navigator/cmd_vel".format(vehicle),
        "diag": "/{}/area_mission/diag".format(vehicle),
    }


class PublishLoop(object):
    """持续发布假位姿 + 假 scan，pose 与 scan pattern 可被用例随时改写。"""

    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.lock = threading.Lock()
        self.pose_x = ROAD_TARGET_X_ODOM
        self.pose_y = ROAD_TARGET_Y_ODOM
        self.pose_z = TARGET_ALT
        # scan 模式："clear"=四周 12m；"wall"=前方 ~60° 内 1m（触发 front_inhibit）
        self.scan_mode = "clear"
        self.stop_event = threading.Event()
        self.thread = None

    def set_pose(self, x, y):
        with self.lock:
            self.pose_x = float(x)
            self.pose_y = float(y)

    def set_scan(self, mode):
        with self.lock:
            self.scan_mode = mode

    def _run(self):
        topic = topics(self.vehicle)
        pub_pose = rospy.Publisher(topic["pose"], PoseStamped, queue_size=2)
        pub_scan = rospy.Publisher(topic["scan"], LaserScan, queue_size=2)
        N = 640
        front_indices = set()
        # 前方 ±30°：bin 范围 [ (π-π/6)/Δ , (π+π/6)/Δ ]，frame 0 = +x（机体前向）
        delta = 2.0 * math.pi / N
        for i in range(N):
            ang = -math.pi + (i + 0.5) * delta
            if abs(ang) < math.radians(30.0):
                front_indices.add(i)
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            with self.lock:
                px, py, pz, mode = self.pose_x, self.pose_y, self.pose_z, self.scan_mode
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.position.z = pz
            pose.pose.orientation.w = 1.0
            pub_pose.publish(pose)

            scan = LaserScan()
            scan.header.stamp = rospy.Time.now()
            scan.header.frame_id = "laser_2d"
            scan.angle_min = -math.pi
            scan.angle_max = math.pi
            scan.angle_increment = delta
            scan.range_min = 0.08
            scan.range_max = 20.0
            if mode == "wall":
                # 前方 60° 内 1m（触发 front_inhibit_laser_enter=2.5），
                # 其余扇区 12m 净空。
                scan.ranges = [1.0 if i in front_indices else 12.0 for i in range(N)]
            else:
                scan.ranges = [12.0] * N
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
        return ("mode=%s wp=%s hover_hold=%.2f hover_skip=%s stop=%s rec_state=%s"
                % (self.raw.get("mode"), self.raw.get("patrol_wp_idx"),
                   self.num.get("hover_hold_s", 0.0),
                   self.raw.get("hover_skip"),
                   self.raw.get("stop"),
                   self.raw.get("recovery_state")))


def wait_until(samples, predicate, timeout, description, start_index=0,
               poll_interval=0.05):
    deadline = time.time() + timeout
    while time.time() < deadline and not rospy.is_shutdown():
        for sample in samples[start_index:]:
            if predicate(sample):
                return True
        start_index = 0  # only skip the leading chunk once
        time.sleep(poll_interval)
    tail = samples[-3:]
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
    parser.add_argument("--enter-timeout", type=float, default=20.0)
    args = parser.parse_args(argv)
    VEHICLE = args.vehicle

    subprocess.call(["rosparam", "delete", "/{}/area_mission_node".format(VEHICLE)],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rospy.init_node("patrol_hover_line_advance_test", anonymous=True,
                    disable_signals=True)
    topic = topics(VEHICLE)

    samples = []

    def on_diag(message):
        samples.append(DiagSample(message.data))

    rospy.Subscriber(topic["diag"], String, on_diag)

    loop = PublishLoop(VEHICLE)
    loop.start()
    time.sleep(args.startup)

    node = subprocess.Popen([
        "rosrun", "auto_explore", "area_mission_node",
        "__ns:=" + VEHICLE,
        "_vehicle:=" + VEHICLE,
        "_target_altitude:=%.1f" % TARGET_ALT,
        "_diag_rate:=%.1f" % args.diag_rate,
        "_patrol_skip_enabled:=true",  # 本用例验证显式开启的跳点模式
        "_patrol_recovery_obstacle_clearance:=0.5",
        "_patrol_fast_stuck_timeout:=100.0",
        "_patrol_segment_timeout:=100.0",
        "_patrol_hover_window:=100.0",
        "_patrol_stall_window:=100.0",
        "_patrol_advance_interval:=100.0",
        # 缩短 recovery_timeout 让 effective_deadline = max(5, 2*2) = 5s
        "_patrol_recovery_timeout:=2.0",
        "_patrol_hover_deadline:=5.0",
        "_patrol_hover_skip_blacklist:=30.0",
    ])

    failures = []

    def check(condition, message):
        print("  [%s] %s" % ("PASS" if condition else "FAIL", message))
        if not condition:
            failures.append(message)

    try:
        # ==== T0：先进入 PATROL ====
        print("\n[T0] 把 pose 钉在 ROAD 目标 (%.1f, %.1f)，等进入 PATROL（mode==2）"
              % (ROAD_TARGET_X_ODOM, ROAD_TARGET_Y_ODOM))
        loop.set_pose(ROAD_TARGET_X_ODOM, ROAD_TARGET_Y_ODOM)
        loop.set_scan("clear")
        t0 = len(samples)
        entered = wait_until(
            samples,
            lambda s: s.num.get("mode") == 2.0,
            args.enter_timeout,
            "进入 PATROL（mode==2）",
            t0,
        )
        check(entered, "T0 未能进入 PATROL（无人机不会进入 hover 块）")
        if not entered:
            raise SystemExit(1)
        # 记录进入 PATROL 时的 wp_idx 作为基线
        before_wp = samples[-1].num.get("patrol_wp_idx", 0.0)
        before_skip = samples[-1].num.get("hover_skip", 0.0)
        print("  已进入 PATROL；wp_idx=%.0f hover_skip=%.0f"
              % (before_wp, before_skip))

        # ==== T1：造悬停 ====
        # 把 pose 拉远到 ROAD 目标 15m 外；scan 改为 wall 触发 front_inhibit +
        # recovery，确保 smooth_x_ 持续 ≈0。
        print("\n[T1] pose 拉远 + 前方 60° 内 1m 墙，造悬停")
        loop.set_pose(ENTER_FAR_X, ENTER_FAR_Y)
        loop.set_scan("wall")
        t1 = len(samples)
        # hover_hold_s 单调上升到 ≥4.5 后必然触发（effective_deadline=5）。
        check(wait_until(
            samples,
            lambda s: s.num.get("hover_hold_s", 0.0) >= 4.5,
            12.0,
            "T1 hover_hold_s 单调上升到 ≥4.5",
            t1,
        ), "T1 hover_hold_s 没涨到 4.5（hover 块未进入）")
        if not failures:
            print("  hover 块已进入死线累积阶段（≥4.5s）")

        # ==== T2：触发 hover-skip ====
        # DIAG_HOVER_SKIP_COUNT 在 hover 块内赋值，是当周期值；
        # DIAG_WP_IDX 在 controlCb 开头（≈line 1322）赋值，滞后 1 个周期。
        # 轮询 ≤1.5s（≈30 个控制周期）等到索引变化。
        print("\n[T2] 等待 hover-skip 触发，DIAG_HOVER_SKIP_COUNT 应 ≥ 1")
        t2 = len(samples)
        # hover_skip 计数：在 hover 块里 ++hover_skip_count_ 然后 diag_[DIAG_HOVER_SKIP_COUNT]
        # = hover_skip_count_，是当周期值。
        triggered = wait_until(
            samples,
            lambda s: s.num.get("hover_skip", 0.0) > before_skip,
            8.0,
            "T2 hover_skip 计数 +1",
            t2,
        )
        check(triggered, "T2 hover_skip 没增加（hover 块没触发）")
        if not triggered:
            print("  [ABORT] hover-skip 没触发，后续用例无效")
            raise SystemExit(1)
        # wp_idx 轮询（DIAG_WP_IDX 滞后 1 周期）
        check(wait_until(
            samples,
            lambda s: s.num.get("patrol_wp_idx", before_wp) != before_wp,
            1.5,
            "T2 wp_idx 变化（DIAG_WP_IDX 滞后 1 周期，需轮询）",
            t2,
        ), "T2 wp_idx 没变化")
        # 固定四航点队列每次仅推进一个索引。
        new_wp = int(samples[-1].num.get("patrol_wp_idx", -1.0))
        expected_next_line = (int(before_wp) + 1) % 4
        check(new_wp == expected_next_line,
              "T2 新 wp_idx=%d ≠ (before_wp + 1) %% 4 = %d"
              % (new_wp, expected_next_line))

        # ==== T3：黑名单生效（间接验证）====
        # 跳点时只把当前点加入黑名单；1.5s 内索引不应回到旧点。
        print("\n[T3] hover_skip 触发后，before_wp=%d 在 1.5s 内不再出现"
              % int(before_wp))
        t3 = len(samples)
        time.sleep(1.5)
        blacklisted_appearances = sum(
            1 for s in samples[t3:]
            if int(s.num.get("patrol_wp_idx", -1)) == int(before_wp)
        )
        check(blacklisted_appearances == 0,
              "T3 黑名单点 wp_idx=%d 在 1.5s 内仍被选中 %d 次（黑名单未生效）"
              % (int(before_wp), blacklisted_appearances))
        # sanity：hover_skip 至少涨了 1
        final_skip = samples[-1].num.get("hover_skip", 0.0)
        check(final_skip >= before_skip + 1,
              "T3 hover_skip 仅从 %.0f 涨到 %.0f（hover-skip 未触发）"
              % (before_skip, final_skip))

        # ==== T4：到达分支内不触发 ====
        print("\n[T4] pose 拉回 ROAD 目标 ≤2m + 全净空，应不触发 hover-skip")
        loop.set_pose(ROAD_TARGET_X_ODOM, ROAD_TARGET_Y_ODOM)
        loop.set_scan("clear")
        # 等一段让选择器换点 → 进入到达分支
        time.sleep(2.5)
        skip_before = samples[-1].num.get("hover_skip", 0.0)
        time.sleep(4.0)
        skip_after = samples[-1].num.get("hover_skip", 0.0)
        check(skip_after == skip_before,
              "T4 在到达分支里也触发了 hover-skip（before=%.0f after=%.0f）"
              % (skip_before, skip_after))

        # ==== T5：远距离无障碍不触发 ====
        print("\n[T5] pose 拉远 + 全净空，应不触发 hover-skip")
        loop.set_pose(ENTER_FAR_X, ENTER_FAR_Y)
        loop.set_scan("clear")
        time.sleep(2.0)   # 让节点切到远段 / 出到达分支
        skip_before = samples[-1].num.get("hover_skip", 0.0)
        time.sleep(4.0)
        skip_after = samples[-1].num.get("hover_skip", 0.0)
        check(skip_after == skip_before,
              "T5 在无障碍远距离也触发了 hover-skip（before=%.0f after=%.0f）"
              % (skip_before, skip_after))
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
    print("\n结果：PASS（PATROL hover-skip + 线推进行为符合规格）")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
