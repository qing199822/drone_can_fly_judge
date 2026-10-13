#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""world 清理：删除 <state> 世界状态快照 + 顶层重复模型名。

背景（2026-09-15 实测）：
    waypoint/world/base0..16.world 与 map_generator 产出的 robocup.world 里
    **每个模型名都出现 2 次**：
      - 第 1 份在 <world> 顶层：真正的模型定义（static + link + collision/visual + pose）
      - 第 2 份在 <world><state> 里：只有 pose/scale/velocity 的"世界状态快照"
    危害有两个：
      1. Gazebo 报 `Non-unique names detected in type model`（评审实测），
         模型可能被空壳顶替 → 建筑不碰撞/不可见；
      2. **加载时会把 <state> 里的旧位姿应用回去**，把随机地图的随机化结果覆盖掉
         （随机生成的地图等于白做）。

处理：
    - 默认删除整个 <state> 块（保留文本其余部分不变，不做 XML 重排）；
    - 再检查顶层模型名是否仍有重复，若有则用 ElementTree 删掉"内容较差"的重复项；
    - 输出统计：删除的 state 条目数、去重的模型数、以及最终唯一名数量。

用法：
    python3 tools/dedupe_world.py <输入.world> [输出.world] [--keep-state]
        path_out 省略时原地覆盖（先备份 .bak）
"""

import os
import re
import shutil
import sys
import xml.etree.ElementTree as ET

STATE_RE = re.compile(r'[ \t]*<state\b.*?</state>\s*\n?', re.S)
STATE_ENTRY_RE = re.compile(r"<model\s+name=['\"]([^'\"]+)['\"]")


def _top_level_names(text):
    """顶层 model/actor/include 名字（用 ET 解析，避免把 state 里的算进来）。"""
    root = ET.fromstring(text)
    world = root if root.tag == 'world' else root.find('world')
    names = []
    for child in list(world):
        if child.tag in ('model', 'actor'):
            names.append(child.get('name'))
        elif child.tag == 'include':
            name_child = child.find('name')
            if name_child is not None and (name_child.text or '').strip():
                names.append(name_child.text.strip())
    return [n for n in names if n]


def _drop_top_level_duplicates(text):
    """兜底：用 ET 删除顶层重复模型（保留含 collision/visual、块更大的一份）。"""
    root = ET.fromstring(text)
    world = root if root.tag == 'world' else root.find('world')
    groups = {}
    for child in list(world):
        if child.tag in ('model', 'actor'):
            name = child.get('name')
        elif child.tag == 'include':
            name_child = child.find('name')
            name = name_child.text.strip() if (name_child is not None and name_child.text) else None
        else:
            continue
        if not name:
            continue
        groups.setdefault(name, []).append(child)

    dropped = []
    for name, elements in groups.items():
        if len(elements) < 2:
            continue
        def score(el):
            s = len(ET.tostring(el, encoding='unicode'))
            if el.find('.//collision') is not None:
                s += 1_000_000
            if el.find('.//visual') is not None:
                s += 500_000
            return s
        best = max(elements, key=score)
        for el in elements:
            if el is not best:
                world.remove(el)
                dropped.append(name)
    if dropped:
        text = ET.tostring(root, encoding='unicode')
    return text, dropped


def clean_world_file(path_in, path_out=None, backup=True, keep_state=False):
    path_in = os.path.abspath(path_in)
    with open(path_in, encoding='utf-8') as handle:
        text = handle.read()

    state_entries, state_blocks = 0, 0
    if not keep_state:
        state_blocks = len(STATE_RE.findall(text))
        for block in STATE_RE.findall(text):
            state_entries += len(STATE_ENTRY_RE.findall(block))
        text = STATE_RE.sub('', text)

    duplicates_before = [n for n in dict.fromkeys(_top_level_names(text))
                         if _top_level_names(text).count(n) > 1]
    text, dropped_by_et = _drop_top_level_duplicates(text) if duplicates_before else (text, [])

    if path_out is None:
        if backup and not os.path.exists(path_in + '.bak'):
            shutil.copy2(path_in, path_in + '.bak')
        path_out = path_in
    else:
        path_out = os.path.abspath(path_out)
        os.makedirs(os.path.dirname(path_out) or '.', exist_ok=True)
    with open(path_out, 'w', encoding='utf-8') as handle:
        handle.write(text)

    remaining = _top_level_names(text)
    duplicates_after = sorted({n for n in remaining if remaining.count(n) > 1})
    return {
        'state_blocks': state_blocks,
        'state_entries': state_entries,
        'dropped_models': dropped_by_et,
        'duplicates_before': sorted(set(duplicates_before)),
        'duplicates_after': duplicates_after,
        'unique_models': len(set(remaining)),
        'out': path_out,
    }


def main(argv):
    if not argv:
        print(__doc__)
        return 2
    keep_state = '--keep-state' in argv
    args = [a for a in argv if not a.startswith('--')]
    if not args:
        print(__doc__)
        return 2
    path_in = args[0]
    path_out = args[1] if len(args) > 1 else None
    if not os.path.exists(path_in):
        print('错误：找不到 %s' % path_in)
        return 1
    report = clean_world_file(path_in, path_out, keep_state=keep_state)
    print('=== world 清理：%s -> %s ===' % (path_in, report['out']))
    print('  删除 <state> 块 %d 个（含 %d 个模型状态条目）'
          % (report['state_blocks'], report['state_entries']))
    if report['duplicates_before']:
        print('  顶层重复模型名 %d 个（示例 %s）'
              % (len(report['duplicates_before']), report['duplicates_before'][:3]))
        print('  ElementTree 兜底删除 %d 个重复块' % len(report['dropped_models']))
    print('  剩余唯一顶层模型名：%d' % report['unique_models'])
    if report['duplicates_after']:
        print('  FAIL 仍有重复模型名：%s' % report['duplicates_after'][:5])
        return 1
    print('  OK 模型名唯一，且不会再被旧世界状态覆盖')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
