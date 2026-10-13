#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""修复官方 map_generator.py 的"房子全叠在一个点 / 4% 概率产出 0 字节 world"问题。

背景（2026-09-19 实测，官方回答 B = 该 .py 为随机地图脚本）：
  1) 写 house.world 的那段里，static 模型走 `change_flag` 分支，用的是**内层循环 i=0** 的
     `center_list[0]` → 13 栋房子全被写到同一个点（实测 base0/1/5/10 都是 (-27.5,25.5)，
     官方仓库里现成的 house.world 也一样）；非 static 分支还会 IndexError。
  2) `create_point()` 连续 10 轮失败时走 "Base world is used!" 回退，而回退里
     `lines = content.readlines()` 复用了**已读完的文件句柄** → 拿到空列表 →
     house.world / robocup.world 是 0 字节（实测 24 次里 1 次）。
  3) 底图的 `<state>` 快照里有一份同名的旧位姿；只改模型级 <pose> 而不改 <state>，
     直接给 Gazebo 加载时会被旧快照覆盖（我们的流程虽然会 dedupe，但脚本自身也应自洽）。

本工具只改这两段（其余代码/随机分布/产物格式保持原样）：
  A. `create_point()`：保留原来的候选生成规则（y 带、rand_x 三段、per-map 特判），
     把"1000 次失败就整体放弃"改成"最多 2000 次里取第一个满足判据的；都不满足就取
     余量最大的那个" → **永远返回 13 个互不重合的落点**，不再有回退/空 world。
  B. 写 house.world：按模型名精确定位，改每个模型**缩进最浅的那个 <pose>**（=模型级位姿，
     保留 z/roll/pitch/yaw），并同步改 `<state>` 里同名条目的位姿；找不到模型级 <pose>
     就在模型标签后插入一个。底图只读一次（顺手修掉回退的句柄 bug）。

用法：
    python3 tools/patches/fix_map_generator_houses.py --check                 # 只看会改什么
    python3 tools/patches/fix_map_generator_houses.py                         # 就地修改（自动备份 .orig_<ts>）
    python3 tools/patches/fix_map_generator_houses.py --script <path>         # 指定脚本
    python3 tools/patches/fix_map_generator_houses.py --out /tmp/fixed.py     # 只输出到新文件（不改原文件）
    python3 tools/patches/fix_map_generator_houses.py --revert                # 从最近备份还原
"""
import argparse
import glob
import os
import shutil
import sys
import time

DEFAULT_SCRIPT = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), 'XTDrone', 'robocup', 'map_generator.py')

MARK_IMPORT_END = "import os"
MARK_CP_START = "def create_point():"
MARK_CP_END = "    return center_box,find_solution"
MARK_HW_START = 'content = open("base"+str(map_num)+".world", \'r\')'
MARK_HW_END = "f.close()"          # house.world 写入段之后的第一个 f.close()

NEW_IMPORTS = """import os

import re

# 可选复现：MAP_SEED=1234 python3 map_generator.py
if os.environ.get('MAP_SEED'):
    random.seed(int(os.environ['MAP_SEED']))"""

NEW_CREATE_POINT = '''def create_point():
    """生成 13 个建筑落点：4 个固定 + 9 个按底图规则随机（与原脚本候选规则一致）。

    与原脚本的区别：不再"1000 次失败就整体放弃 + 用底图回退"，而是每个点最多采 2000 个
    候选，取第一个满足原判据的；若都不满足，取"余量最大"的那个。因此永远返回 13 个
    （近似）互不重合的落点，也不会再出现 0 字节 world。
    """
    center_box = [[-27.5, 25.5], [-33.4, -21.4], [-28.2, -33], [-25.8, -19.4]]
    center_box_judge = [[86.6, -30], [-27, 14.65], [15.9, -13.6], [5.6, 35.7]]
    if map_num == 6:
        center_box = [[-28.64, 25.56], [-32.5, 1.8], [-28.2, -22.7], [-28.1, -33]]
        center_box_judge = [[86.6, -30], [-27, 14.65], [14.2, -20.6], [5.6, 35.7]]
    size_box_judge = [[16, 7], [1, 1], [2, 1], [2, 1], [20, 30], [30, 17], [16, 12], [16, 12], [16, 16],
                      [11, 9], [11, 9], [11, 9], [17, 6]]

    def candidate(m):
        """与原脚本完全相同的候选生成（含 per-map 特判带）"""
        if m == 0:
            py = random.uniform(0, 1)
            y = -32 if py > 0.5 else 15
            if map_num == 6:
                y = 14.5
            if map_num in (15, 11, 10, 7):
                y = 14.3
            if map_num in (5, 13, 12, 9, 8, 16):
                y = -24
            p = random.uniform(0.33, 1)
            x = int(rand_x(p))
        elif m == 1:
            y = int(random.choice((-1, 1)) * random.uniform(13, 31))
            if map_num in (15, 14, 11, 10, 7):
                y = random.uniform(-32, -23)
            if map_num == 6:
                y = random.uniform(-31, 3)
                if y >= -22:
                    y += 26
            if map_num in (5, 13, 12, 9, 8, 16):
                y = random.uniform(-31, 6)
                if y >= -2:
                    y += 25
            p = random.uniform(0.33, 1)
            x = int(rand_x(p))
        else:
            y = int(random.choice((-1, 1)) * random.uniform(14, 30))
            p = random.uniform(0.33, 1)
            x = int(rand_x(p))
            if map_num in (6, 15, 14, 11, 10, 7):
                y = random.uniform(-30, 3)
                if y >= -23:
                    y += 27
            if map_num in (5, 13, 12, 9, 8, 16):
                y = random.uniform(-30, 3)
                if y >= -3:
                    y += 27
        return x, y

    def clearance(x, y):
        """候选点相对全部已放置点的最小余量（>0 即满足原脚本的 'or' 判据）"""
        box_len = len(center_box_judge)
        best = None
        for i in range(box_len):
            dx = abs(x - center_box_judge[i][0]) - (size_box_judge[i][0] + size_box_judge[box_len][0]) / 2 - 3
            dy = abs(y - center_box_judge[i][1]) - (size_box_judge[i][1] + size_box_judge[box_len][1]) / 2 - 3
            c = dx if dx > dy else dy
            best = c if best is None else min(best, c)
        return 0.0 if best is None else best

    for m in range(9):
        best_xy = None
        best_clear = None
        for _ in range(2000):
            x, y = candidate(m)
            c = clearance(x, y)
            if c > 0:
                best_xy = (x, y)
                break
            if best_clear is None or c > best_clear:
                best_xy, best_clear = (x, y), c
        x, y = best_xy
        center_box_judge.append([x, y])
        center_box.append([x, y])
    return center_box, True


'''

NEW_HOUSE_WRITER = '''content = open("base"+str(map_num)+".world", 'r')

_base_text = content.read()          # 只读一次：原脚本回退分支里二次 readlines() 会拿到空列表

content.close()

name_list = ['house_1_146', 'house_3_156', 'house_3_157', 'house_3_158', 'gas_station_73', 'fast_food_93',

             'house_1_66', 'house_1_67', 'house_1_146_clone', 'house_2_71', 'house_2_125', 'house_2_126',

             'house_3_68']

center_list, find_solution = create_point()

new_center = {}

for i, name in enumerate(name_list):

    new_center[name] = (center_list[i][0], center_list[i][1])


def _replace_xy(pose_line, x, y):

    """只换 x,y，保留原位的 z / roll / pitch / yaw 与缩进"""

    parts = pose_line.split()

    if len(parts) < 7:

        return None

    indent = pose_line[:len(pose_line) - len(pose_line.lstrip())]

    return "%s<pose frame=''>%s %s %s %s %s %s\\n" % (

        indent, x, y, parts[-4], parts[-3], parts[-2], parts[-1])


def _rewrite_model_pose(lines, name, x, y):

    """改某个 <model> 块里缩进最浅的 <pose>（= 模型级位姿）；没有就插一个"""

    start = None

    for idx, line in enumerate(lines):

        if line.lstrip().startswith("<model name='%s'>" % name):

            start = idx

            break

    if start is None:

        return 0

    end = None

    for idx in range(start + 1, len(lines)):

        if lines[idx].strip() == '</model>':

            end = idx

            break

    if end is None:

        return 0

    best = None

    for idx in range(start + 1, end):

        s = lines[idx].lstrip()

        if s.startswith('<pose') and '</pose>' in s:

            indent = len(lines[idx]) - len(s)

            if best is None or indent < best[0]:

                best = (indent, idx)

    if best is None:

        indent = len(lines[start]) - len(lines[start].lstrip()) + 2

        lines.insert(start + 1, "%s<pose frame=''>%s %s 0 0 0 0</pose>\\n" % (' ' * indent, x, y))

        return 1

    indent, idx = best

    new_line = _replace_xy(lines[idx], x, y)

    if new_line is None:

        return 0

    lines[idx] = new_line

    return 1


def _rewrite_state_pose(lines, name, x, y):

    """<state> 快照里同名条目的位姿也要改，否则 Gazebo 加载时用旧位姿覆盖随机化结果"""

    in_state = False

    start = None

    for idx, line in enumerate(lines):

        s = line.strip()

        if s.startswith('<state'):

            in_state = True

        elif in_state and s.startswith('</state>'):

            break

        elif in_state and s.startswith("<model name='%s'>" % name):

            start = idx

            break

    if start is None:

        return 0

    end = len(lines)

    for idx in range(start + 1, len(lines)):

        if lines[idx].strip() == '</model>':

            end = idx

            break

    changed = 0

    for idx in range(start, end):

        s = lines[idx].lstrip()

        if s.startswith('<pose') and '</pose>' in s:

            new_line = _replace_xy(lines[idx], x, y)

            if new_line is not None:

                lines[idx] = new_line

                changed += 1

    return changed


lines = _base_text.splitlines(True)

moved = 0

for name, (x, y) in new_center.items():

    moved += _rewrite_model_pose(lines, name, x, y)

    _rewrite_state_pose(lines, name, x, y)

with open("house.world", 'w') as f:

    f.writelines(lines)

f.close()

print('Moved %d house models (13 distinct random centers)' % moved)

print('Finish!')'''


def read_lines(path):
    with open(path, encoding='utf-8') as handle:
        return handle.readlines()


def build_fixed(lines):
    text = ''.join(lines)
    if 'MAP_SEED' in text and '_rewrite_model_pose' in text:
        return None, '该脚本看起来已经修过'
    out = []
    idx = 0
    n = len(lines)
    while idx < n:
        line = lines[idx]
        # A) 头部 import → 补 re + MAP_SEED
        if line.strip() == MARK_IMPORT_END and not any('import re' in l for l in lines[:idx + 3]):
            out.append(NEW_IMPORTS + '\n')
            idx += 1
            continue
        # B) create_point() 整段替换
        if line.rstrip('\n') == MARK_CP_START:
            out.append(NEW_CREATE_POINT + '\n')
            idx += 1
            while idx < n and lines[idx] != MARK_CP_END + '\n':
                idx += 1
            idx += 1
            continue
        # C) house.world 写入段整段替换
        if line.rstrip('\n') == MARK_HW_START:
            out.append(NEW_HOUSE_WRITER + '\n')
            idx += 1
            while idx < n and lines[idx].rstrip('\n') != MARK_HW_END:
                idx += 1
            idx += 1
            continue
        out.append(line)
        idx += 1
    return ''.join(out), None


def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument('--script', default=DEFAULT_SCRIPT)
    parser.add_argument('--out', default=None, help='只写到这个新文件，不改原脚本')
    parser.add_argument('--check', action='store_true', help='只检查能否修复，不写文件')
    parser.add_argument('--revert', action='store_true', help='从最近的 .orig_* 备份还原')
    args = parser.parse_args(argv)

    script = os.path.abspath(args.script)
    if not os.path.exists(script):
        sys.exit('找不到脚本：%s' % script)

    if args.revert:
        backups = sorted(glob.glob(script + '.orig_*'))
        if not backups:
            sys.exit('没有找到备份：%s.orig_*' % script)
        shutil.copy2(backups[-1], script)
        print('[fix] 已还原 %s ← %s' % (script, backups[-1]))
        return 0

    lines = read_lines(script)
    fixed, note = build_fixed(lines)
    if fixed is None:
        print('[fix] 无需修改：%s' % note)
        return 0
    print('[fix] 将修改：%s' % script)
    print('      - create_point(): 永远返回 13 个互不重合落点（去掉 10 轮失败回退）')
    print('      - house.world 写入段：按模型名精确改模型级 <pose> + 同步 <state> 位姿')
    print('      - 底图只读一次（修掉 0 字节 world 的句柄 bug）；新增可选 MAP_SEED')
    if args.check:
        print('[fix] --check：未写入')
        return 0
    if args.out:
        open(args.out, 'w', encoding='utf-8').write(fixed)
        print('[fix] 已写出：%s' % args.out)
        return 0

    stamp = time.strftime('%Y%m%d_%H%M%S')
    backup = '%s.orig_%s' % (script, stamp)
    shutil.copy2(script, backup)
    open(script, 'w', encoding='utf-8').write(fixed)
    print('[fix] 备份：%s' % backup)
    print('[fix] 已就地修改：%s' % script)
    print('[fix] 回退：python3 %s --revert' % os.path.abspath(__file__))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
