#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""actor_watchdog —— 人物节点看门狗（**不修改官方 control_actor.py**）

背景（官方缺陷，已核实）：
    control_actor.py:668 在 except 分支里写成
        print("Gazebo model state service"+self.id+"  call failed: %s") % e
    `% e` 落在 print(...) 之外 → print() 返回 None → None % e 抛 TypeError，
    该 TypeError 又发生在 except 内部 → 直接冲出 loop() → **人物节点整个退出**，
    表现就是"人物被看到后突然不动了"（负载抖动时 /gazebo/get_model_state 超时即触发）。
    本地与官方 GitHub master 的该文件逐字节相同（sha256 03d13c7c331cef8b），确为官方 bug。

本看门狗只做一件事：**发现某个 control_actor.py 进程不在了，就把它重新拉起来**
（cwd 与 PYTHONPATH 与官方启动方式一致，日志追加写入同一文件，崩溃栈保留）。
它不修改官方代码、不改参数、不改行为，只是让"应有的 6 个人物"继续在跑。

用法：
    python3 tools/actor_watchdog.py --round-dir logs/official_round_XXXX [--interval 5]
退出：Ctrl-C

注意：这是裁判端的**额外进程**（补偿官方缺陷），是否在正式演练/比赛启用需组委会认可。
"""
import argparse
import os
import subprocess
import sys
import time

ACTORS = range(6)


def running_ids(text):
    """从 ps 输出里解析出正在运行的 control_actor.py 的 id 集合。"""
    ids = set()
    for line in text.splitlines():
        if 'control_actor.py' not in line or 'watchdog' in line:
            continue
        parts = line.split()
        for tok in reversed(parts):
            if tok.isdigit():
                ids.add(int(tok))
                break
    return ids


def ps_text():
    try:
        return subprocess.run(['ps', '-eo', 'args='], capture_output=True,
                              text=True, errors='replace').stdout
    except Exception:
        return ''


def main():
    ap = argparse.ArgumentParser(description='人物节点看门狗（不改官方代码）')
    ap.add_argument('--round-dir', required=True, help='本轮目录（actor_<id>.log 所在处）')
    ap.add_argument('--interval', type=float, default=5.0, help='巡检间隔秒（默认 5）')
    ap.add_argument('--script-root', default=os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    help='仓库根（默认由本脚本位置推断）')
    args = ap.parse_args()

    rd = os.path.abspath(args.round_dir)
    root = os.path.abspath(args.script_root)
    actor_py = os.path.join(root, 'XTDrone', 'robocup', 'control_actor.py')
    module_dir = os.path.join(root, 'XTDrone', 'robocup')
    if not os.path.isfile(actor_py):
        print('错误：找不到 %s' % actor_py, file=sys.stderr)
        return 2

    wlog = os.path.join(rd, 'actor_watchdog.log')

    def log(msg):
        line = '[%s] %s' % (time.strftime('%F %T'), msg)
        print(line, flush=True)
        with open(wlog, 'a', encoding='utf-8') as fh:
            fh.write(line + '\n')

    log('看门狗启动：round=%s interval=%.0fs（官方 control_actor.py 未修改）' % (rd, args.interval))
    env = os.environ.copy()
    env['PYTHONPATH'] = module_dir + (':' + env['PYTHONPATH'] if env.get('PYTHONPATH') else '')

    while True:
        alive = running_ids(ps_text())
        for i in ACTORS:
            if i in alive:
                continue
            logfile = os.path.join(rd, 'actor_%d.log' % i)
            with open(logfile, 'a', encoding='utf-8') as fh:
                fh.write('\n===== [watchdog] 检测到 actor_%d 不在运行，重新拉起（%s）=====\n'
                         % (i, time.strftime('%F %T')))
            try:
                subprocess.Popen(['python3', '-u', actor_py, str(i)], cwd=rd, env=env,
                                 stdout=open(logfile, 'a', encoding='utf-8'),
                                 stderr=subprocess.STDOUT, start_new_session=True)
                log('已重新拉起 actor_%d（日志 %s）' % (i, logfile))
            except Exception as exc:
                log('拉起 actor_%d 失败：%s' % (i, exc))
        time.sleep(args.interval)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print('\n[watchdog] 退出')
