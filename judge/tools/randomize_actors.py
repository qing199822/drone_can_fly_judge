#!/usr/bin/env python3
# tools/randomize_actors.py
# 启动 gzserver 后，重置 actor_0..actor_5 的初始位置到指定区域。
# 与 start_referee.sh 流程一致：地图文件本身不动，actor 通过 SetModelState 重定位。
#
# 用法：
#   python3 tools/randomize_actors.py \
#       --x-min -50 --x-max 50 --y-min -50 --y-max 50 --z 1.25 \
#       [--seed N] [--actor-prefix actor_] [--count 6]

import argparse
import math
import os
import random
import sys
import time

import rospy
from gazebo_msgs.srv import SetModelState, SetModelStateRequest


def build_state(model_name, x, y, z):
    state = SetModelStateRequest()
    state.model_state.model_name = model_name
    state.model_state.pose.position.x = x
    state.model_state.pose.position.y = y
    state.model_state.pose.position.z = z
    state.model_state.pose.orientation.x = 0.0
    state.model_state.pose.orientation.y = 0.0
    state.model_state.pose.orientation.z = 0.0
    state.model_state.pose.orientation.w = 1.0
    state.model_state.twist.linear.x = 0.0
    state.model_state.twist.linear.y = 0.0
    state.model_state.twist.linear.z = 0.0
    state.model_state.twist.angular.x = 0.0
    state.model_state.twist.angular.y = 0.0
    state.model_state.twist.angular.z = 0.0
    return state


def wait_for_service(client, name, timeout=120.0):
    deadline = time.time() + timeout
    while not rospy.is_shutdown() and time.time() < deadline:
        try:
            # rospy.ServiceProxy.wait_for_service returns None on success.
            client.wait_for_service(timeout=min(2.0, max(0.1, deadline - time.time())))
            return True
        except rospy.ROSException:
            # A timed-out wait is expected while Gazebo is still starting.
            time.sleep(0.2)
    return False


def main():
    parser = argparse.ArgumentParser(
        description="Gazebo SetModelState：随机重置 actor_0..actor_N 初始位置。"
    )
    parser.add_argument('--x-min', type=float, default=-50.0,
                        help='actor x 最小值（含），默认 -50')
    parser.add_argument('--x-max', type=float, default=50.0,
                        help='actor x 最大值（含），默认 50')
    parser.add_argument('--y-min', type=float, default=-50.0,
                        help='actor y 最小值（含），默认 -50')
    parser.add_argument('--y-max', type=float, default=50.0,
                        help='actor y 最大值（含），默认 50')
    parser.add_argument('--z', type=float, default=1.25,
                        help='actor z 高度，默认 1.25（与 base0.world 一致）')
    parser.add_argument('--seed', type=int, default=None,
                        help='随机种子，复现同一分布')
    parser.add_argument('--actor-prefix', default='actor_',
                        help='actor 名前缀，默认 actor_')
    parser.add_argument('--count', type=int, default=6,
                        help='actor 数量，默认 6（actor_0..actor_5）')
    parser.add_argument('--timeout', type=float, default=120.0,
                        help='等 /gazebo/set_model_state 服务的超时（秒），默认 120')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('randomize_actors', anonymous=True)

    client = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    if not wait_for_service(client, '/gazebo/set_model_state', timeout=args.timeout):
        rospy.logerr("timeout: /gazebo/set_model_state")
        sys.exit(1)

    rng = random.Random(args.seed)
    placed = 0
    for i in range(args.count):
        x = rng.uniform(args.x_min, args.x_max)
        y = rng.uniform(args.y_min, args.y_max)
        name = f'{args.actor_prefix}{i}'
        req = build_state(name, x, y, args.z)
        try:
            resp = client(req)
            ok = getattr(resp, 'success', None)
            status = getattr(resp, 'status_message', '')
            print(f'actor {name} -> ({x:.2f}, {y:.2f}, {args.z:.2f}) success={ok} {status}')
            if ok:
                placed += 1
        except Exception as exc:
            print(f'actor {name} 异常: {exc}')

    print(f'placed {placed}/{args.count}')
    sys.exit(0 if placed == args.count else 2)


if __name__ == '__main__':
    main()