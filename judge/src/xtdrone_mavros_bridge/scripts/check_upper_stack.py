#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""上层栈健康门：激光过滤 + 融合地图 + TF 树是否真正可用。

`1.sh` 在启动视觉/追踪（YOLO、tracking）之前调用本脚本，避免在
Gazebo/PX4/MAVROS 或建图链路还没健康时就把重负载拉起来。

检查项（每架）：
    1. /typhoon_h480_<id>/scan_filtered 有 LaserScan 数据；
    2. TF 能查到 map -> base_link_<id>（说明 odom_broadcaster / fuser 静态 tf 正常）；
    3. 全局：/map（OccupancyGrid）有数据。

用 rospy 直接收消息与查 tf，比 shell 里反复 `rostopic echo` 更可靠、也更容易诊断。
退出码：0=全部就绪；2=超时仍有缺失项；1=参数错误。

用法：
    rosrun xtdrone_mavros_bridge check_upper_stack.py --num-drones 6
"""

import argparse
import sys
import time

import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan


def parse_args(argv):
    parser = argparse.ArgumentParser(description="上层栈健康门检查")
    parser.add_argument("--num-drones", type=int, default=1)
    parser.add_argument("--start-index", type=int, default=0)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--base-frame-prefix", default="base_link_")
    return parser.parse_args(argv)


def main(argv):
    args = parse_args(argv)
    if args.num_drones < 1:
        sys.stderr.write("--num-drones 必须 >= 1\n")
        return 1

    drone_ids = list(range(args.start_index, args.start_index + args.num_drones))
    rospy.init_node("check_upper_stack", anonymous=True, disable_signals=True)

    buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
    tf2_ros.TransformListener(buffer)

    scan_ok = {index: None for index in drone_ids}
    tf_ok = {index: None for index in drone_ids}
    map_ok = [None]
    map_subscriber = [None]  # 保持订阅，避免被回收

    def on_map(message):
        if map_ok[0] is None:
            map_ok[0] = (message.info.width, message.info.height)

    map_subscriber[0] = rospy.Subscriber("/map", OccupancyGrid, on_map)

    for index in drone_ids:
        topic = "/typhoon_h480_{}/scan_filtered".format(index)
        rospy.Subscriber(topic, LaserScan, lambda msg, i=index: scan_ok.__setitem__(i, len(msg.ranges)))

    deadline = time.time() + args.timeout
    last_report = 0.0
    while time.time() < deadline and not rospy.is_shutdown():
        for index in drone_ids:
            frame = "{}{}".format(args.base_frame_prefix, index)
            if tf_ok[index] is None:
                try:
                    transform = buffer.lookup_transform("map", frame, rospy.Time(0), rospy.Duration(0.5))
                    point = transform.transform.translation
                    tf_ok[index] = (round(point.x, 2), round(point.y, 2))
                except Exception:
                    pass

        missing = []
        for index in drone_ids:
            if scan_ok[index] is None:
                missing.append("scan_filtered[{}]".format(index))
            if tf_ok[index] is None:
                missing.append("tf map->base_link_{}".format(index))
        if map_ok[0] is None:
            missing.append("/map")

        if not missing:
            break

        now = time.time()
        if now - last_report > 20.0:
            last_report = now
            print("[check-upper-stack] 仍缺少：{}（剩 {:.0f}s）".format(
                ", ".join(missing), deadline - now))
            sys.stdout.flush()
        time.sleep(1.0)

    # 汇总结果
    failures = []
    for index in drone_ids:
        if scan_ok[index] is None:
            failures.append("drone{}: 无 scan_filtered".format(index))
        else:
            print("[check-upper-stack] drone{} OK  scan_filtered={} 条  tf map->base_link_{}={}".format(
                index, scan_ok[index], index, tf_ok[index]))
    if map_ok[0] is None:
        failures.append("/map 无数据")
    else:
        print("[check-upper-stack] /map OK  {}x{}".format(map_ok[0][0], map_ok[0][1]))
    sys.stdout.flush()

    if failures:
        for item in failures:
            print("[check-upper-stack] FAIL  {}".format(item))
        sys.stdout.flush()
        return 2

    print("[check-upper-stack] 上层栈健康门通过：激光过滤 + TF + /map 均正常")
    sys.stdout.flush()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
