#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROAD 前进速度下限 / 诊断字段的运行验证（不需要 Gazebo，需要 ROS master）。

背景：2~5 号在 ROAD 阶段"原地旋转 / 几乎不前进"——道路分割正常、激光新鲜，
但 MPPI + 安全层把 linear.x 压到 ≈0 却保留 angular.z。
本脚本用"假传感器 + 真 area_mission_node"复现并验证修复：

  profile=side   : 前方净空 10m，两侧 2.2m 有墙（典型窄走廊，MPPI 会保守降速）
                   -> 期望：速度下限生效，v_published 明显 > 0
  profile=blocked: 前方 2.0m 就是墙（真正的硬障碍）
                   -> 期望：不允许兜底，v_published ≈ 0（安全优先）

同时打印别名字段，便于确认"速度在哪一步掉下去"。

用法（先起 roscore）：
    rosrun auto_explore road_speed_floor_test.py --profile side
"""

import argparse
import math
import os
import subprocess
import sys
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32MultiArray, String

VEHICLE = "typhoon_h480_0"
SCAN_TOPIC = ""
POSE_TOPIC = ""
STEER_TOPIC = ""
CAND_TOPIC = ""
DIAG_TOPIC = ""
CMD_TOPIC = ""


def set_vehicle(name):
    """测试夹具可以驱动任意编号的无人机（例如 4 号用来验证区域回归）。"""
    global VEHICLE, SCAN_TOPIC, POSE_TOPIC, STEER_TOPIC, CAND_TOPIC, DIAG_TOPIC, CMD_TOPIC
    VEHICLE = name
    SCAN_TOPIC = "/{}/scan".format(name)
    POSE_TOPIC = "/{}/mavros/local_position/pose".format(name)
    STEER_TOPIC = "/{}/road_follow/steering".format(name)
    CAND_TOPIC = "/{}/road_follow/candidates".format(name)
    DIAG_TOPIC = "/{}/area_mission/diag".format(name)
    CMD_TOPIC = "/{}/mux_inputs/navigator/cmd_vel".format(name)


set_vehicle("typhoon_h480_0")

DIAG_FIELDS = [
    "mppi_valid", "mppi_v", "mppi_w", "v_after_mppi", "v_after_depth",
    "v_after_ttc", "v_after_peer", "v_after_region", "v_after_floor",
    "v_published", "w_published", "road_valid", "road_held", "road_confidence",
    "guidance_norm", "fwd_lidar", "fwd_depth", "dist_to_goal", "stop_code", "mode",
]


def build_scan(profile):
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
        forward = math.cos(angle)
        if profile == "side":
            # 前方 ±40° 净空 10m，其余方向 2.2m（窄走廊两侧墙）
            ranges.append(10.0 if forward > math.cos(math.radians(40.0)) else 2.2)
        elif profile == "open":
            # 开阔地带：量程内完全没有回波（真实 2D 雷达在开阔地就是这种）
            ranges.append(float("inf"))
        elif profile == "broken":
            # 传感器坏数据：全是 NaN（必须判为不可用并停车，不能当"无障碍"）
            ranges.append(float("nan"))
        elif profile == "narrow":
            # 前方 ±40° 净空 10m，两侧仅 1.6m：MPPI 会因障碍代价想减速，
            # 但前方净空充足 -> 正是速度下限该兜底的场景
            ranges.append(10.0 if forward > math.cos(math.radians(40.0)) else 1.6)
        else:
            # 前方 ±20° 只有 2.0m（真正的硬障碍），其余 3.5m
            ranges.append(2.0 if forward > math.cos(math.radians(20.0)) else 3.5)
    message.ranges = ranges
    return message


CLOCK_JUMP_AT = 6.0
CLOCK_JUMP_SIZE = 2.5


def publish_clock(stop_event):
    """模拟 Gazebo 的 /clock：正常推进，中途突然 +2.5s（暂停恢复/时钟抖动）。"""
    clock_pub = rospy.Publisher("/clock", Clock, queue_size=10)
    started = time.time()
    while not stop_event.is_set() and not rospy.is_shutdown():
        elapsed = time.time() - started
        sim = elapsed + (CLOCK_JUMP_SIZE if elapsed >= CLOCK_JUMP_AT else 0.0)
        message = Clock()
        message.clock = rospy.Time.from_sec(sim)
        clock_pub.publish(message)
        time.sleep(0.01)


def publish_fakes(profile, stop_event):
    pose_pub = rospy.Publisher(POSE_TOPIC, PoseStamped, queue_size=2)
    scan_pub = rospy.Publisher(SCAN_TOPIC, LaserScan, queue_size=2)
    # 真系统里 scan_filtered 由 scan_self_filter 生成；本夹具直接两路同发。
    filtered_pub = rospy.Publisher("/{}/scan_filtered".format(VEHICLE), LaserScan, queue_size=2)
    steer_pub = rospy.Publisher(STEER_TOPIC, Vector3, queue_size=2)
    cand_pub = rospy.Publisher(CAND_TOPIC, Float32MultiArray, queue_size=2)
    started = time.time()
    while not stop_event.is_set() and not rospy.is_shutdown():
        now = rospy.Time.now()
        # 位姿必须真的在移动：否则 updateRoadProgress 会判定 ROAD stalled，
        # 进入 crawl recovery（按设计跳过速度下限），测不到要验证的行为。
        moved = -0.8 * (time.time() - started)  # 朝 -x 移动：避开区域边界(xmax=0)钳制
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = "map"
        pose.pose.position.x = moved
        pose.pose.position.y = 0.0
        pose.pose.position.z = min(3.0, (time.time() - started) * 2.0)  # 模拟爬升到 target_altitude
        pose.pose.orientation.w = 1.0
        pose_pub.publish(pose)

        scan = build_scan(profile)
        scan_pub.publish(scan)
        filtered_pub.publish(scan)

        steering = Vector3()
        steering.x = 0.0    # 横向偏差
        steering.y = 0.0    # 方向偏差（对准正前方）
        steering.z = 0.9    # 置信度
        steer_pub.publish(steering)

        candidates = Float32MultiArray()
        candidates.data = [0.0, 0.0, 0.9, 0.5]  # 单分支：[heading, lateral, conf, width]
        cand_pub.publish(candidates)
        time.sleep(0.05)


def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", choices=("side", "narrow", "blocked", "open", "broken"),
                        default="side")
    parser.add_argument("--vehicle", default="typhoon_h480_0",
                        help="被测无人机编号（4 号可验证区域回归）")
    parser.add_argument("--seconds", type=float, default=12.0)
    parser.add_argument("--launch", action="store_true",
                        help="用 roslaunch auto_explore_swarm.launch num_drones:=1 启动真节点（推荐）")
    parser.add_argument("--old-params", action="store_true",
                        help="用修复前的 ROAD-MPPI 保守代价 + 关闭速度下限，用于复现原地旋转")
    parser.add_argument("--old-cost-only", action="store_true",
                        help="保留修复前的保守代价，但开启速度下限：验证下限单独能否兜住")
    parser.add_argument("--force-low-mppi", action="store_true",
                        help="把 ROAD-MPPI 的期望速度设为 0：确定性验证速度下限的兜底与门控")
    parser.add_argument("--clock-jump", action="store_true",
                        help="自建 /clock 并在中途制造 +2.5s 仿真时间跳变，验证 scan stale 误判修复")
    parser.add_argument("--clean-params", action="store_true",
                        help="启动前删除本节点私有参数（rosrun _param:= 会残留在参数服务器上）")
    args = parser.parse_args(argv)
    set_vehicle(args.vehicle)

    if args.clean_params:
        # rosrun 的 _param:= 会留在参数服务器上，跨进程污染后续运行（曾导致"新参数"
        # 跑出旧行为）。测试前先清掉本节点的私有参数。
        subprocess.call(["rosparam", "delete", "/{}/area_mission_node".format(VEHICLE)],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # 关键顺序：先设 /use_sim_time -> init_node -> 再发布 /clock，否则时钟线程
    # 会在 rospy 未初始化时发布而报错。
    if args.clock_jump:
        rospy.set_param("/use_sim_time", True)
    rospy.init_node("road_speed_floor_test", anonymous=True, disable_signals=True)

    stop_event = threading.Event()
    if args.clock_jump:
        clock_thread = threading.Thread(target=publish_clock, args=(stop_event,))
        clock_thread.daemon = True
        clock_thread.start()
        time.sleep(0.5)

    thread = threading.Thread(target=publish_fakes, args=(args.profile, stop_event))
    thread.daemon = True
    thread.start()
    time.sleep(2.0)

    node = None
    if not args.launch:
        command = [
            "rosrun", "auto_explore", "area_mission_node",
            "__ns:=" + VEHICLE,
            "_vehicle:=" + VEHICLE,
            "_target_altitude:=3.0",
            "_diag_rate:=10.0",
        ]
        if args.old_params or args.old_cost_only:
            # 修复前的 ROAD-MPPI 保守代价（沿用 PATROL 的 80/4.0 等）
            command += [
                "_road_mppi_obstacle_weight:=80.0",
                "_road_mppi_safe_radius:=4.0",
                "_road_mppi_speed_weight:=0.4",
                "_road_mppi_goal_weight:=1.2",
                "_road_mppi_yaw_weight:=0.05",
            ]
        if args.old_params:
            command += ["_road_min_speed_floor:=0.0"]  # 一并关掉下限 = 修复前状态
        if args.force_low_mppi:
            command += ["_road_mppi_v_des:=0.0"]      # 让 MPPI 想停在原地
        node = subprocess.Popen(command)

    samples = []

    def on_diag(message):
        # 订阅人读版 String 诊断（避免数组消息在某些环境下握手异常）
        parsed = {}
        for token in message.data.split():
            if "=" in token:
                key, _, value = token.partition("=")
                try:
                    parsed[key] = float(value)
                except ValueError:
                    pass
        if parsed:
            samples.append(parsed)

    def on_cmd(message):
        pass

    rospy.Subscriber(DIAG_TOPIC, String, on_diag)
    rospy.Subscriber(CMD_TOPIC, Twist, on_cmd)

    deadline = time.time() + args.seconds
    while time.time() < deadline and not rospy.is_shutdown():
        time.sleep(0.2)

    stop_event.set()
    if node is not None:
        node.terminate()
    time.sleep(0.5)

    if not samples:
        print("FAIL 没有收到 diag（节点没起来或话题不对）")
        return 2

    # 只看进入 ROAD 且 MPPI 有效的样本
    # 注意：开阔无回波时 mppi_valid 本来就是 0（走 open-space 直飞），不能因此排除样本。
    road = [s for s in samples if s.get("mode") == 1.0]
    if not road:
        print("FAIL 没有 ROAD 阶段样本（mode 分布：%s；样例：%s）" %
              (sorted({s.get("mode") for s in samples}), samples[:1]))
        return 2

    def median(values):
        values = sorted(values)
        return values[len(values) // 2] if values else 0.0

    mppi_v = median([s["mppi_v"] for s in road])
    after_floor = median([s["v_floor"] for s in road])
    published_v = median([s["v_pub"] for s in road])
    forward = median([s["fwd_lidar"] for s in road])
    w_pub = median([abs(s["w_pub"]) for s in road])
    stop_codes = sorted({int(s["stop"]) for s in road})
    print("profile=%s  样本=%d（ROAD 有效 %d）" % (args.profile, len(samples), len(road)))
    stale_stops = [s for s in samples if int(s.get("stop", 0)) == 1]
    jumps = max([int(s.get("clock_jumps", 0)) for s in samples] or [0])
    def valid_age(key):
        values = [s.get(key, 0.0) for s in samples if s.get(key, 0.0) < 1.0e8]
        return max(values) if values else 0.0
    age_sim = valid_age("scan_age")
    age_wall = valid_age("scan_age_wall")
    floor_states = sorted({int(s.get("floor_state", -1)) for s in road})
    recovery = sorted({int(s.get("recovery", -1)) for s in road})
    print("  mppi_v=%.2f  v_after_floor=%.2f  v_published=%.2f  |w|=%.2f  fwd_lidar=%.2f  stop_code=%s"
          "  floor_state=%s  recovery=%s"
          % (mppi_v, after_floor, published_v, w_pub, forward, stop_codes, floor_states, recovery))

    if args.profile == "open":
        clear_flags = sorted({int(s.get("scan_clear", -1)) for s in samples})
        ok = published_v >= 0.6 and not stale_stops and 1 in clear_flags
        print("  期望（开阔无回波）：判为 scan_clear 并继续前进，不能因 MPPI valid()=false 停车")
        print("  scan_clear=%s" % clear_flags)
    elif args.profile == "broken":
        bad_flags = sorted({int(s.get("scan_bad", -1)) for s in samples})
        ok = published_v < 0.2 and len(stale_stops) > 0 and 2 in bad_flags
        print("  期望（全 NaN 坏数据）：判为不可用并停车（scan_bad=2），不能当无障碍乱飞")
        print("  scan_bad=%s" % bad_flags)
    elif args.force_low_mppi:
        if args.profile == "blocked":
            ok = published_v < 0.5
            print("  期望（下限机制 + 硬障碍）：MPPI 想停 + 前方 2m 墙 -> 下限必须不生效")
        else:
            ok = after_floor >= 0.9 and published_v >= 0.6
            print("  期望（下限机制 + 净空充足）：MPPI 想停，但下限兜到 %.1f m/s" % 1.0)
    elif args.profile in ("side", "narrow"):
        if args.old_params:
            ok = published_v < 0.5
            print("  期望（旧参数复现）：保守代价把 v 压到 ≈0、|w| 变大 -> 原地旋转")
        elif args.old_cost_only:
            ok = published_v >= 0.6 and after_floor >= 0.6
            print("  期望（仅开下限）：保守代价仍想停在原地，但速度下限把它兜到 ≥%.1f m/s"
                  % 0.6)
        else:
            ok = published_v >= 0.6
            print("  期望（新参数）：前方净空充足 -> 维持前进，v_published 明显 > 0")
    else:
        # 硬障碍：不允许兜底抬速（阈值放宽到 0.5，但下限必须没有生效）
        floor_raised = after_floor > mppi_v + 0.05 and after_floor > 0.6
        ok = published_v < 0.5 and not floor_raised
        print("  期望：前方 2m 硬障碍 -> 速度下限不生效，v_published ≈ 0（安全优先）")
    if args.clock_jump:
        print("  [时钟跳变] clock_jumps=%d  max(scan_age_sim)=%.2fs  max(scan_age_wall)=%.2fs  "
              "scan_stale 硬停样本=%d" % (jumps, age_sim, age_wall, len(stale_stops)))
        # 关键判据：仿真时钟跳了，但墙钟仍新鲜 -> 不应因为 stale 而停车
        ok = ok and jumps >= 1 and age_wall < 0.5 and len(stale_stops) == 0
    print("  结果：%s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
