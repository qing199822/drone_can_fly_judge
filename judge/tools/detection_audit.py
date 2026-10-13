#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""detection_audit —— 独立审计观察者（**不改官方裁判代码**）

目的：在"完全对齐官方"的前提下，回答"为什么没消除"。
它不参与计分，只是旁路订阅 + 用官方常量复算判定：

    err_threshold    = 1.0   m      （官方 score_cal.py:19）
    DETECTION_INTERVAL = 1.0 s      （官方 :25）
    DETECTION_DURATION = 15.0 s     （官方 :26）

对每个目标输出两类记录：
    [detection_audit] event=report  actor=N error=.. gap=.. elapsed=.. decision=accept|reset reason=..
    [detection_audit] event=found / deletion_predicted actor=N duration=.. reports=.. max_gap=.. max_error=..

用法：
    python3 tools/detection_audit.py                     # 持续监听，打印判定
    python3 tools/detection_audit.py --targets blue,red1 # 只看某几个
    python3 tools/detection_audit.py --summary-only      # 只打印每 5 秒的汇总
    python3 tools/detection_audit.py --log /tmp/a.log    # 同时落盘
前置：source /opt/ros/noetic/setup.bash && source devel/setup.bash，ROS_MASTER_URI 指向裁判端。
"""
import argparse
import sys
import threading
import time

ERR_THRESHOLD = 1.0          # 官方 score_cal.py:19
DETECTION_INTERVAL = 1.0     # 官方 :25
DETECTION_DURATION = 15.0    # 官方 :26

MAP = [  # 与官方 robocup/test.py 一致
    ('green', 0, '/actor_green_info', 'green'),
    ('blue',  1, '/actor_blue_info',  'blue'),
    ('brown', 2, '/actor_brown_info', 'brown'),
    ('white', 3, '/actor_white_info', 'white'),
    ('red2',  4, '/actor_red2_info',  'red'),
    ('red1',  5, '/actor_red1_info',  'red'),
]


def main():
    ap = argparse.ArgumentParser(description='独立审计观察者（不改官方裁判代码）')
    ap.add_argument('--targets', default='all')
    ap.add_argument('--summary-only', action='store_true')
    ap.add_argument('--log', default='')
    args = ap.parse_args()

    import rospy
    from gazebo_msgs.srv import GetModelState
    from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo
    from std_msgs.msg import Int16, String

    rospy.init_node('detection_audit', anonymous=True)
    want = [m for m in MAP if args.targets == 'all' or m[0] in
            [x.strip() for x in args.targets.split(',')]]
    get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    state = {i: dict(last=0.0, count=0, max_gap=0.0, max_err=0.0,
                     find_time=None, elapsed=0.0) for _, i, _, _ in want}
    lock = threading.Lock()
    logf = open(args.log, 'a', encoding='utf-8') if args.log else None

    def emit(line):
        print(line, flush=True)
        if logf:
            logf.write(line + '\n')

    def make_cb(name, aid, cls_expected):
        def _cb(msg):
            if getattr(msg, 'cls', '') != cls_expected:
                return                      # 官方按 cls 精确匹配
            try:
                p = get_state('actor_%d' % aid, 'ground_plane').pose.position
            except Exception:
                return
            now = time.time()
            err = ((msg.x - p.x) ** 2 + (msg.y - p.y) ** 2) ** 0.5
            with lock:
                st = state[aid]
                gap = 0.0 if st['last'] == 0.0 else now - st['last']
                st['last'] = now
                continuous = (st['count'] == 0) or (gap <= DETECTION_INTERVAL)
                ok = (err < ERR_THRESHOLD) and continuous
                st['count'] += 1
                st['max_gap'] = max(st['max_gap'], gap)
                st['max_err'] = max(st['max_err'], err)
                if not ok:
                    reason = ('coordinate_error' if err >= ERR_THRESHOLD else 'report_gap')
                    if not args.summary_only:
                        emit('[detection_audit] event=report actor=%d topic=%s error=%.3f '
                             'gap=%.3f decision=reset reason=%s' % (aid, name, err, gap, reason))
                    st['find_time'] = None
                    st['count'] = 0
                    return
                if st['find_time'] is None:
                    st['find_time'] = now
                    if not args.summary_only:
                        emit('[detection_audit] event=report actor=%d topic=%s error=%.3f '
                             'gap=%.3f decision=accept reason=found' % (aid, name, err, gap))
                st['elapsed'] = now - st['find_time']
                if st['elapsed'] >= DETECTION_DURATION:
                    emit('[detection_audit] event=deletion_predicted actor=%d topic=%s '
                         'duration=%.2f reports=%d max_gap=%.3f max_err=%.3f'
                         % (aid, name, st['elapsed'], st['count'], st['max_gap'], st['max_err']))
                    st['find_time'] = None
                    st['count'] = 0
        return _cb

    subs = [rospy.Subscriber(t, ActorInfo, make_cb(n, i, c)) for n, i, t, c in want]
    emit('[detection_audit] 监视 %s；官方常量 err=%.1fm interval=%.1fs duration=%.1fs'
         % (','.join(n for n, *_ in want), ERR_THRESHOLD, DETECTION_INTERVAL, DETECTION_DURATION))

    if args.summary_only:
        while not rospy.is_shutdown():
            time.sleep(5)
            with lock:
                parts = []
                for n, i, _, _ in want:
                    st = state[i]
                    parts.append('%s: 报点%d 最长间隔%.2f 最大误差%.2f 连续%.1fs'
                                 % (n, st['count'], st['max_gap'], st['max_err'], st['elapsed']))
            emit('[detection_audit] 汇总 | ' + ' | '.join(parts))
    else:
        rospy.spin()
    return 0


if __name__ == '__main__':
    sys.exit(main())
