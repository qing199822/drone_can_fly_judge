#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""字节级保真测试：把 referee.py 的 gen-map（忠实模式）与真实的上游 map_generator.py 对比。

做法（每个种子跑一轮）：
  1. 在临时目录里准备一个"上游工作目录"：拷入 base1/2/3.world + rover_static.world；
  2. 用 `random.seed(S)` 注入后，直接 runpy 跑 **XTDrone/robocup/map_generator.py 本体**
     （HOME 重定向到临时目录，让它写死的 ~/PX4_Firmware/... 落到沙箱里）；
  3. 用 `referee.py` 的 MapGenerator 以同样的 seed 跑**忠实模式**（不加 --fix-known-bugs）；
  4. 逐字节比较 house.world / rover.world / black_box.txt / obstacle.txt / robocup.world。

这验证的是"默认忠实复现"这句话到底成不成立。
用法：
    python3 robocup_referee/selftest_fidelity.py
    python3 robocup_referee/selftest_fidelity.py --seeds 1234,7,42 --keep
退出码 0 = 全部逐字节一致。
"""

import argparse
import contextlib
import io
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
WORKSPACE = os.path.dirname(HERE)
UPSTREAM_ROBOCOP = os.path.join(WORKSPACE, 'XTDrone', 'robocup')
UPSTREAM_SCRIPT = os.path.join(UPSTREAM_ROBOCOP, 'map_generator.py')
OUTPUT_TAIL = os.path.join('PX4_Firmware', 'Tools', 'sitl_gazebo', 'worlds')

sys.path.insert(0, HERE)
import referee as _referee   # noqa: E402

FILES = ('house.world', 'rover.world', 'black_box.txt', 'obstacle.txt')


RUNNER = """import random, runpy, sys
random.seed(int(sys.argv[1]))
runpy.run_path('map_generator.py', run_name='__main__')
"""


def first_diff(a, b, context=1):
    """返回 (是否相同, 描述, 上下文行)。"""
    if a == b:
        return True, '逐字节相同（%d B）' % len(a), None
    n = min(len(a), len(b))
    i = 0
    while i < n and a[i] == b[i]:
        i += 1
    line_no = a[:i].count(b'\n') + 1
    la = a.split(b'\n')
    lb = b.split(b'\n')
    detail = None
    if line_no - 1 < len(la) and line_no - 1 < len(lb):
        detail = ('第 %d 行：\n      上游: %s\n      本程序: %s'
                  % (line_no,
                     la[line_no - 1][:160].decode('utf-8', 'replace'),
                     lb[line_no - 1][:160].decode('utf-8', 'replace')))
    return False, ('第 %d 字节起不同（约第 %d 行），大小 %d vs %d'
                   % (i, line_no, len(a), len(b))), detail


def run_upstream(seed, root, index):
    cwd = os.path.join(root, 'up_%d' % index)
    home = os.path.join(root, 'home_%d' % index)
    os.makedirs(os.path.join(home, OUTPUT_TAIL), exist_ok=True)
    os.makedirs(cwd, exist_ok=True)
    for name in ('base1.world', 'base2.world', 'base3.world', 'rover_static.world'):
        shutil.copy(os.path.join(UPSTREAM_ROBOCOP, name), os.path.join(cwd, name))
    shutil.copy(UPSTREAM_SCRIPT, os.path.join(cwd, 'map_generator.py'))
    with open(os.path.join(cwd, '_run.py'), 'w') as fh:
        fh.write(RUNNER)
    env = dict(os.environ)
    env['HOME'] = home
    proc = subprocess.run([sys.executable, '_run.py', str(seed)], cwd=cwd, env=env,
                          capture_output=True, text=True)
    produced = {}
    for name in FILES:
        path = os.path.join(cwd, name)
        produced[name] = path if os.path.isfile(path) else None
    world = os.path.join(home, OUTPUT_TAIL, 'robocup.world')
    produced['robocup.world'] = world if os.path.isfile(world) else None
    return produced, proc


def run_mine(seed, root, index, src_dir):
    out = os.path.join(root, 'mine_%d' % index)
    gen = _referee.MapGenerator(_referee.Config(base='random', seed=seed, out=out,
                                               src_dir=src_dir))
    with contextlib.redirect_stdout(io.StringIO()):
        res = gen.generate()
    produced = {name: os.path.join(out, name) for name in FILES}
    produced['robocup.world'] = os.path.join(out, 'robocup.world')
    return produced, res


def main():
    parser = argparse.ArgumentParser(description='gen-map 与上游 map_generator.py 的字节级对比')
    parser.add_argument('--seeds', default='1234,7,20260928,1,2,3',
                        help='逗号分隔的随机种子列表')
    parser.add_argument('--keep', action='store_true', help='保留临时目录以便查看')
    args = parser.parse_args()
    seeds = [int(s) for s in args.seeds.split(',') if s.strip()]

    if not os.path.isfile(UPSTREAM_SCRIPT):
        print('找不到上游脚本：%s' % UPSTREAM_SCRIPT)
        return 2

    root = tempfile.mkdtemp(prefix='referee_fidelity_')
    print('保真测试：上游脚本 = %s' % UPSTREAM_SCRIPT)
    print('临时目录 = %s\n' % root)
    all_ok = True
    try:
        for index, seed in enumerate(seeds):
            print('=== seed=%d ===' % seed)
            up, proc = run_upstream(seed, root, index)
            mine, res = run_mine(seed, root, index, src_dir=os.path.dirname(up['house.world']))
            up_base = [ln.strip() for ln in proc.stdout.splitlines() if ln.strip()]
            up_map = up_base[0] if up_base else '?'
            print('  底图：上游=%s  本程序=base%d %s'
                  % (up_map, res['map_num'],
                     'OK' if str(res['map_num']) == up_map else '**不一致**'))
            if str(res['map_num']) != up_map:
                all_ok = False
            for name in ('house.world', 'rover.world', 'black_box.txt',
                         'obstacle.txt', 'robocup.world'):
                pa, pb = up.get(name), mine.get(name)
                if pa is None:
                    print('  [FAIL] %-16s 上游没产出该文件（可能走了回退/异常）' % name)
                    all_ok = False
                    continue
                if pb is None or not os.path.isfile(pb):
                    print('  [FAIL] %-16s 本程序没产出该文件' % name)
                    all_ok = False
                    continue
                ok, desc, detail = first_diff(open(pa, 'rb').read(), open(pb, 'rb').read())
                print('  [%s] %-16s %s' % ('PASS' if ok else 'FAIL', name, desc))
                if not ok:
                    all_ok = False
                    if detail:
                        print('      %s' % detail)
            print()

        print('保真测试结果：%s（%d 个种子）'
              % ('全部逐字节一致' if all_ok else '存在差异', len(seeds)))
        return 0 if all_ok else 1
    finally:
        if args.keep:
            print('（--keep：临时目录保留在 %s）' % root)
        else:
            shutil.rmtree(root, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(main())
