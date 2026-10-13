#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""裁判端自测：不依赖参赛队算法，直接向 /actor_*_info 注入报点，验证计分全链路。

映射表取自**官方** `robocup/test.py`（权威，勿改）：
    green -> actor_0   /actor_green_info   cls='green'
    blue  -> actor_1   /actor_blue_info    cls='blue'
    brown -> actor_2   /actor_brown_info   cls='brown'
    white -> actor_3   /actor_white_info   cls='white'
    red2  -> actor_4   /actor_red2_info    cls='red'
    red1  -> actor_5   /actor_red1_info    cls='red'     ← 两个红人都发 cls='red'

三种注入模式：
    truth   发人物**真实坐标**（走完"发现(+50) → 连续≥15s → 消除(+100)"全链路）
    offset  发真实坐标 + 偏移（默认 5 m；应判无效，用于验证 1 m 误差门限）
    nan     发 NaN（验证官方 NaN 缺陷 / 启动器的 --nan-guard）

用法：
    python3 tools/referee_selftest.py --list                     # 打印映射表
    python3 tools/referee_selftest.py                            # truth / 全部目标 / 20s / 10Hz
    python3 tools/referee_selftest.py --targets blue,green --duration 40
    python3 tools/referee_selftest.py --targets white --mode offset --offset 5
    python3 tools/referee_selftest.py --targets blue --mode nan --duration 10
    python3 tools/referee_selftest.py --watch                    # 只盯分，不注入
前置：`source /opt/ros/noetic/setup.bash && source devel/setup.bash`，ROS_MASTER_URI 指向裁判端。
"""
import argparse
import sys
import threading
import time

MAP = [
    # (名字, actor_id, topic, cls)
    ('green', 0, '/actor_green_info', 'green'),
    ('blue',  1, '/actor_blue_info',  'blue'),
    ('brown', 2, '/actor_brown_info', 'brown'),
    ('white', 3, '/actor_white_info', 'white'),
    ('red2',  4, '/actor_red2_info',  'red'),
    ('red1',  5, '/actor_red1_info',  'red'),
]


def _print_map():
    print('  %-8s %-8s %-22s %s' % ('名字', 'actor', '话题', 'cls'))
    for n, i, t, c in MAP:
        print('  %-8s %-8s %-22s %s' % (n, 'actor_%d' % i, t, c))


def _watch(stop, interval=2.0):
    import rospy
    from std_msgs.msg import Int16, String
    last = None
    while not stop.is_set():
        try:
            s = rospy.wait_for_message('/score', Int16, timeout=3).data
            l = rospy.wait_for_message('/left_actors', String, timeout=3).data
            finds = []
            from std_msgs.msg import Float32
            for n, i, t, c in MAP:
                try:
                    rospy.wait_for_message('/find_actor_%d' % i, Float32, timeout=0.3)
                    finds.append(n)
                except Exception:
                    pass
            line = '  [%s] score=%-6s left=%s  本轮有上报的 find: %s' % (
                time.strftime('%H:%M:%S'), s, l, ','.join(finds) or '—')
            if line != last:
                print(line, flush=True)
                last = line
        except Exception:
            pass
        time.sleep(interval)


def main():
    ap = argparse.ArgumentParser(description='裁判端自测（注入 /actor_*_info 报点）')
    ap.add_argument('--list', action='store_true', help='打印映射表后退出')
    ap.add_argument('--targets', default='all', help='green,blue,brown,white,red1,red2 或 all')
    ap.add_argument('--mode', default='truth', choices=['truth', 'offset', 'nan'])
    ap.add_argument('--duration', type=float, default=20.0, help='每个目标的注入时长（秒）')
    ap.add_argument('--rate', type=float, default=10.0, help='注入频率（Hz）')
    ap.add_argument('--offset', type=float, default=5.0, help='offset 模式的坐标偏移（米）')
    ap.add_argument('--watch', action='store_true', help='只盯分，不注入')
    ap.add_argument('--watch-only', action='store_true', help=argparse.SUPPRESS)
    args = ap.parse_args()

    if args.list:
        _print_map()
        return 0

    import rospy
    from gazebo_msgs.srv import GetModelState
    from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo
    from std_msgs.msg import Int16, String

    rospy.init_node('referee_selftest', anonymous=True)
    stop = threading.Event()
    threading.Thread(target=_watch, args=(stop,), daemon=True).start()

    if args.watch or args.watch_only:
        print('  只盯分模式（Ctrl-C 退出）')
        try:
            while not rospy.is_shutdown():
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        stop.set()
        return 0

    want = [m for m in MAP if args.targets == 'all' or m[0] in
            [x.strip() for x in args.targets.split(',')]]
    if not want:
        print('错误：--targets 没匹配到任何目标（可用 %s）' %
              ','.join(m[0] for m in MAP), file=sys.stderr)
        return 2

    get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    pubs = {n: rospy.Publisher(t, ActorInfo, queue_size=10) for n, i, t, c in want}

    def snapshot():
        try:
            s = rospy.wait_for_message('/score', Int16, timeout=4).data
            l = rospy.wait_for_message('/left_actors', String, timeout=4).data
            return s, l
        except Exception:
            return '?', '?'

    s0, l0 = snapshot()
    print('  注入前: /score=%s  /left_actors=%s' % (s0, l0))
    print('  模式=%s  目标=%s  时长=%.0fs  频率=%.0fHz%s'
          % (args.mode, ','.join(n for n, *_ in want), args.duration, args.rate,
             '  偏移=%.1fm' % args.offset if args.mode == 'offset' else ''))

    t0 = time.time()
    period = 1.0 / args.rate
    sent = 0
    while not rospy.is_shutdown() and time.time() - t0 < args.duration:
        for n, i, t, c in want:
            m = ActorInfo()
            m.cls = c
            if args.mode == 'nan':
                m.x = float('nan'); m.y = float('nan')
            else:
                try:
                    p = get_state('actor_%d' % i, 'ground_plane').pose.position
                except Exception as exc:
                    print('  取 actor_%d 位姿失败：%s' % (i, exc), file=sys.stderr)
                    m.x = m.y = float('nan')
                else:
                    sign = 1.0 if i % 2 == 0 else -1.0
                    m.x = p.x + (sign * args.offset if args.mode == 'offset' else 0.0)
                    m.y = p.y + (sign * args.offset if args.mode == 'offset' else 0.0)
            pubs[n].publish(m)
            sent += 1
        time.sleep(period)

    time.sleep(3)
    s1, l1 = snapshot()
    print('  注入后: /score=%s  /left_actors=%s   （共发 %d 条）' % (s1, l1, sent))
    try:
        print('  Δscore = %+d' % (int(s1) - int(s0)))
    except Exception:
        pass
    print('  提示：truth 模式若持续 ≥15 秒，裁判应把该目标"消除"（+100 且 /left_actors 少一个）；')
    print('        offset 模式（偏移 5 m > 1 m 门限）应完全不计分；nan 模式用于验证官方 NaN 缺陷。')
    stop.set()
    return 0


if __name__ == '__main__':
    sys.exit(main())
