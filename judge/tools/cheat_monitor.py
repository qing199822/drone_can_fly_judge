#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""裁判端监测：① 禁用真值话题订阅  ② 非法坐标（NaN/Inf）报点

① 官方口径（组委会 2026-10-10）：
    "/xtdrone/typhoon_h480_N/ground_truth/odom 会发布，但你们不能订阅，
     一订阅就会被裁判系统判断为作弊终止比赛。"
    → 本工具持续扫描 ROS 图里所有含 `ground_truth` 的话题的**订阅者**并告警。

② 官方 score_cal.py:119-121 的 NaN 缺陷（已向组委会反馈）：
    报点坐标是 NaN 时 `distance_sq >= err_threshold**2` 恒为 False → 不清零 →
    该目标被计为"发现"并**直接从 /left_actors 删除**，实测每个目标虚增
    50（发现）+100（消除）= 150 分，且一条报文即触发。
    在组委会修复前，本监测用于**赛前自查**：一眼看出自己有没有发非法坐标。

用法：
    python3 tools/cheat_monitor.py                      # 持续监测（2s 一次 + 实时听报点）
    python3 tools/cheat_monitor.py --once; echo $?      # 只查一次：1=有问题，0=干净
    python3 tools/cheat_monitor.py --interval 1 --nan-grace 5
    python3 tools/cheat_monitor.py --no-nan             # 只做真值订阅监测
    python3 tools/cheat_monitor.py --log /tmp/x.log

退出码：0=干净；1=发现违规；2=连不上 master。
"""
import argparse
import os
import sys
import threading
import time

try:
    import rosgraph
except ImportError:
    print('错误：找不到 rosgraph。请先 source /opt/ros/noetic/setup.bash', file=sys.stderr)
    sys.exit(2)

KEY = 'ground_truth'
# 裁判端自身的节点必须放行：
#  - control_actor.py（人物控制器）按官方实现会订阅 /xtdrone/*/ground_truth/odom
#    用于"人物逃跑"判定 —— 6 个人物 × 6 架 = 36 条订阅，属合法行为，不是作弊。
#  - score_cal / referee / mirror 同理。
DEFAULT_ALLOW_PATTERNS = (
    'score_cal', 'referee', 'cheat_monitor', 'mirror',
    '/actor_0', '/actor_1', '/actor_2', '/actor_3', '/actor_4', '/actor_5',
)
DEFAULT_ACTOR_TOPICS = (
    '/actor_blue_info', '/actor_green_info', '/actor_white_info',
    '/actor_brown_info', '/actor_red1_info', '/actor_red2_info',
)

_lock = threading.Lock()
_nan_events = []      # [(topic, cls, x, y, ts)]
_nan_topics_seen = set()


def _log(path, line):
    if not path:
        return
    with open(path, 'a', encoding='utf-8') as fh:
        fh.write(line + '\n')


def _make_actor_callback(topic):
    def _cb(msg):
        x = getattr(msg, 'x', 0.0)
        y = getattr(msg, 'y', 0.0)
        bad = (x != x) or (y != y) or _isinf(x) or _isinf(y)
        if not bad:
            return
        with _lock:
            _nan_events.append((topic, getattr(msg, 'cls', ''), x, y, time.time()))
            first = topic not in _nan_topics_seen
            _nan_topics_seen.add(topic)
        if first:
            print('\n' + '!' * 72, flush=True)
            print('[非法报点告警] 话题 %s 上报了非法坐标：cls=%r x=%r y=%r'
                  % (topic, getattr(msg, 'cls', ''), x, y), flush=True)
            print('    官方 score_cal.py:119 的 NaN 缺陷会把它计为"发现"并删除该目标，'
                  '实测虚增 150 分/目标。', flush=True)
            print('    请立刻让上报端停发 NaN/Inf。', flush=True)
            print('!' * 72, flush=True)
    return _cb


def _isinf(v):
    try:
        return v == float('inf') or v == float('-inf')
    except Exception:
        return False


def scan_ground_truth(master):
    try:
        _, subs, _ = master.getSystemState()
    except Exception as exc:
        raise RuntimeError(str(exc))
    return {t: sorted(n) for t, n in subs if KEY in t}


def violations(found, allow_patterns):
    bad = []
    for topic, nodes in found.items():
        for n in nodes:
            if any(p in n for p in allow_patterns):
                continue
            bad.append((topic, n))
    return bad


def main():
    ap = argparse.ArgumentParser(description='裁判端监测：真值订阅 + 非法报点')
    ap.add_argument('--interval', type=float, default=2.0, help='扫描间隔秒（默认 2）')
    ap.add_argument('--once', action='store_true', help='只查一次（会等 --nan-grace 秒收集报点）')
    ap.add_argument('--nan-grace', type=float, default=3.0, help='--once 时收集报点的等待秒数')
    ap.add_argument('--log', default='', help='告警追加写入该文件')
    ap.add_argument('--allow', default='', help='额外允许的节点名，逗号分隔')
    ap.add_argument('--no-nan', action='store_true', help='不监听 /actor_*_info 的 NaN/Inf')
    ap.add_argument('--nan-topics', default=','.join(DEFAULT_ACTOR_TOPICS),
                    help='要监听报点的话题，逗号分隔')
    args = ap.parse_args()

    allow = list(DEFAULT_ALLOW_PATTERNS)
    if args.allow:
        allow += [x.strip() for x in args.allow.split(',') if x.strip()]

    nan_subs = []
    if not args.no_nan:
        try:
            import rospy
            from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo
            # 不在 --once 下也允许没有 master 的空图；init_node 失败就跳过 NaN 监听
            try:
                rospy.init_node('cheat_monitor', anonymous=True, disable_signals=True)
            except Exception:
                pass
            for t in [s.strip() for s in args.nan_topics.split(',') if s.strip()]:
                nan_subs.append(rospy.Subscriber(t, ActorInfo, _make_actor_callback(t)))
            print('[cheat_monitor] 非法报点监测已开启：%s' % ', '.join(
                s.strip() for s in args.nan_topics.split(',') if s.strip()), flush=True)
        except ImportError as exc:
            print('[cheat_monitor] 跳过非法报点监测（%s）' % exc, file=sys.stderr)

    master = rosgraph.Master('/cheat_monitor')
    seen = set()
    warned = False
    first_pass = True
    while True:
        # ---- ① 真值订阅 ----
        try:
            found = scan_ground_truth(master)
        except RuntimeError as exc:
            print('[cheat_monitor] 连不上 ROS master：%s' % exc, file=sys.stderr)
            return 2
        bad = violations(found, allow)
        for topic, node in bad:
            if (topic, node) in seen:
                continue
            seen.add((topic, node))
            warned = True
            print('\n' + '!' * 72, flush=True)
            print('[作弊告警] 节点 %s 订阅了禁用的真值话题：%s' % (node, topic), flush=True)
            print('           官方口径：一订阅即判作弊、终止比赛。请立刻停掉该节点。', flush=True)
            print('!' * 72, flush=True)
            _log(args.log, '[%s] VIOLATION node=%s topic=%s'
                 % (time.strftime('%F %T'), node, topic))

        # ---- ② 非法报点 ----
        with _lock:
            n_nan = len(_nan_events)
            topics_nan = sorted(_nan_topics_seen)
        if n_nan:
            warned = True
            if first_pass or not hasattr(main, '_nan_count'):
                pass
            if getattr(main, '_nan_count', 0) != n_nan:
                main._nan_count = n_nan
                _log(args.log, '[%s] NAN_REPORT topics=%s count=%d last=%r'
                     % (time.strftime('%F %T'), ','.join(topics_nan), n_nan,
                        _nan_events[-1][:4]))

        print('[cheat_monitor] %s 真值话题 %d 个/订阅者 %d 个%s | 非法报点 %d 条%s'
              % (time.strftime('%T'), len(found), sum(len(v) for v in found.values()),
                 '（有违规！见下方告警）' if bad else '（干净：仅裁判端自身节点）',
                 n_nan, '（有违规！）' if n_nan else '（干净）'), flush=True)

        if args.once:
            if first_pass and not args.no_nan:
                time.sleep(args.nan_grace)      # 给报点一点时间进来
                first_pass = False
                continue
            return 1 if warned else 0
        first_pass = False
        time.sleep(args.interval)


if __name__ == '__main__':
    sys.exit(main())
