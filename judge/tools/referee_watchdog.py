#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
裁判计分节点看门狗（不改官方 score_cal.py）

背景（2026-10-12 复跑实测）：
    官方 score_cal.py 在本机跑到中途会退出（referee.log 末尾出现
    `Warning: UAV 3 is higher than 6 meters` / `score: 0` 后进程消失），
    之后 `/score` 话题变空，整轮"看起来还在跑、其实已经不算分"。
    这是**裁判端工具链**问题，不是选手代码问题，所以用一个外挂看门狗兜住。

做法：每 `--interval` 秒扫一次 ps，若计分进程不在，就用与 launcher **完全相同的
方式**重新拉起（source ROS + 本工作区 devel ⇒ 才有 ros_actor_cmd_pose_plugin_msgs；
带 SCORE_OUTPUT_PATH 指向本轮目录）。

用法：
    python3 tools/referee_watchdog.py --round-dir logs/official_round_XXXX [--interval 5]

脚本优先级（与 launcher 同序）：
    <round-dir>/score_cal_limit.py      ← --referee-limit 生成的副本（含 MISSION_TIMEOUT 改值）
    <round-dir>/score_cal_nan_guard.py  ← --nan-guard 生成的副本
    <round-dir>/score_cal_audit.py      ← 日志移植副本（若存在）
    <root>/XTDrone/robocup/score_cal.py ← 官方原版
"""
import argparse
import os
import subprocess
import sys
import time

VEHICLE = 'typhoon_h480'

CANDIDATES = [
    'score_cal_limit.py',
    'score_cal_nan_guard.py',
    'score_cal_audit.py',
]


def ps_text():
    try:
        return subprocess.run(['ps', '-eo', 'args='], capture_output=True,
                              text=True, errors='replace').stdout
    except Exception:
        return ''


def score_running(text, py_name):
    """计分进程是否在跑（排除自身/看门狗自身命中的情况）。"""
    for line in text.splitlines():
        if 'watchdog' in line:
            continue
        if 'score_cal' not in line:
            continue
        if py_name in line or 'score_cal' in line:
            return True
    return False


def pick_score_py(rd, root):
    for name in CANDIDATES:
        p = os.path.join(rd, name)
        if os.path.isfile(p):
            return p
    p = os.path.join(root, 'XTDrone', 'robocup', 'score_cal.py')
    return p if os.path.isfile(p) else None


def main():
    ap = argparse.ArgumentParser(description='裁判计分节点看门狗（不改官方 score_cal.py）')
    ap.add_argument('--round-dir', required=True, help='本轮目录（score.txt / referee.log 所在处）')
    ap.add_argument('--interval', type=float, default=5.0, help='巡检间隔秒（默认 5）')
    ap.add_argument('--script-root', default=os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    help='仓库根（默认由本脚本位置推断）')
    ap.add_argument('--vehicle', default=VEHICLE, help='机型名（默认 typhoon_h480）')
    ap.add_argument('--once', action='store_true', help='只巡检一次（自测用）')
    args = ap.parse_args()

    rd = os.path.abspath(args.round_dir)
    root = os.path.abspath(args.script_root)
    score_py = pick_score_py(rd, root)
    if score_py is None:
        print('错误：找不到任何 score_cal*.py（round=%s / root=%s）' % (rd, root), file=sys.stderr)
        return 2

    wlog = os.path.join(rd, 'referee_watchdog.log')

    def log(msg):
        line = '[%s] %s' % (time.strftime('%F %T'), msg)
        print(line, flush=True)
        try:
            with open(wlog, 'a', encoding='utf-8') as fh:
                fh.write(line + '\n')
        except Exception:
            pass

    log('看门狗启动：round=%s interval=%.0fs score_py=%s（官方 score_cal.py 未修改）'
        % (rd, args.interval, os.path.basename(score_py)))
    log('重启命令：bash -lc "source /opt/ros/noetic/setup.bash && source %s/devel/setup.bash && '
        'SCORE_OUTPUT_PATH=%s/score.txt python3 -u %s %s"'
        % (root, rd, score_py, args.vehicle))

    restarts = 0
    while True:
        if not score_running(ps_text(), os.path.basename(score_py)):
            restarts += 1
            outlog = os.path.join(rd, 'referee_restart.log')
            cmd = ('source /opt/ros/noetic/setup.bash && '
                   'source %s/devel/setup.bash && '
                   'export SCORE_OUTPUT_PATH=%s/score.txt && '
                   'exec python3 -u %s %s' % (root, rd, score_py, args.vehicle))
            try:
                with open(outlog, 'a', encoding='utf-8') as fh:
                    fh.write('\n===== [referee_watchdog] 第 %d 次拉起计分节点（%s）=====\n'
                             % (restarts, time.strftime('%F %T')))
                    subprocess.Popen(['bash', '-lc', cmd], cwd=rd,
                                     stdout=fh, stderr=subprocess.STDOUT,
                                     start_new_session=True)
                log('检测到计分节点不在运行 → 已第 %d 次拉起（日志 %s）' % (restarts, outlog))
            except Exception as exc:
                log('拉起计分节点失败：%s' % exc)
        if args.once:
            break
        time.sleep(args.interval)
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print('\n[referee_watchdog] 退出')
