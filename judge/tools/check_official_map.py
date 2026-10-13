#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""校验"官方 map_generator.py 产物"在结构上是不是一张能用的 world。

背景（2026-09-18 实测，官方回答 B = 用 XTDrone/robocup/map_generator.py 作为随机地图脚本）：
  - 官方 .py 的 `create_point()` 有概率连续 10 轮失败 → 走 "Base world is used!" 回退；
    该回退里 `lines = content.readlines()` 复用了已经读完的文件句柄 → 拿到空列表 →
    写出的 house.world / robocup.world 是 **0 字节**（实测 24 次里 1 次）。
  - 所以调用官方脚本后必须先做结构校验，不通过就重跑（换随机数），而不是直接把空 world 喂给 Gazebo。

本脚本只查"结构"，不查占位（占位交给 tools/check_referee_world.py，且要先 dedupe）：
  * 文件存在且 > 10 KB
  * XML 可解析且含 <world>
  * <road> 至少 1 条；顶层唯一模型名 ≥ 100；actor == 6；唯一 rover 名 == 20

用法：python3 tools/check_official_map.py <robocup.world> [--rovers 20]
退出码：0 = 结构有效；1 = 无效
"""
import argparse
import os
import sys
import xml.etree.ElementTree as ET

MIN_BYTES = 10 * 1024
MIN_UNIQUE_MODELS = 100
EXPECT_ACTOR = 6


def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument('world')
    parser.add_argument('--rovers', type=int, default=20)
    args = parser.parse_args(argv)

    path = args.world
    problems = []
    if not os.path.exists(path):
        print('结果：无效 —— 产物不存在：%s' % path)
        return 1
    size = os.path.getsize(path)
    if size < MIN_BYTES:
        print('结果：无效 —— 产物只有 %d 字节（< %d）；官方脚本大概率走了 "Base world is used!" 空回退'
              % (size, MIN_BYTES))
        return 1
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as exc:
        print('结果：无效 —— XML 解析失败：%s' % exc)
        return 1
    world = root if root.tag == 'world' else root.find('world')
    if world is None:
        print('结果：无效 —— 没有 <world> 元素')
        return 1

    roads = world.findall('road')
    top = [c.get('name') for c in world if c.tag in ('model', 'actor')]
    unique = {n for n in top if n}
    actors = list(world.iter('actor'))
    rovers = {n for n in top if (n or '').startswith('rover')}
    if not roads:
        problems.append('没有 <road>（道路网丢失）')
    if len(unique) < MIN_UNIQUE_MODELS:
        problems.append('顶层唯一模型名只有 %d 个（< %d）' % (len(unique), MIN_UNIQUE_MODELS))
    if len(actors) != EXPECT_ACTOR:
        problems.append('actor 数 = %d（期望 %d）' % (len(actors), EXPECT_ACTOR))
    if args.rovers >= 0 and len(rovers) != args.rovers:
        problems.append('唯一 rover 名 = %d 个（期望 %d）' % (len(rovers), args.rovers))

    print('=== 官方地图产物结构校验：%s ===' % path)
    print('  大小 %.2f MB / road %d 条 / 顶层条目 %d（唯一 %d）/ actor %d / 唯一 rover %d'
          % (size / 1e6, len(roads), len(top), len(unique), len(actors), len(rovers)))
    dups = len(top) - len(unique)
    if dups:
        print('  提示：顶层有 %d 个重名（官方 .py 的 rover.world 每架写两份 + 底图 <state> 同名）'
              '→ 交给 tools/dedupe_world.py 处理' % dups)
    for p in problems:
        print('  FAIL  %s' % p)
    if problems:
        print('结果：无效')
        return 1
    print('结果：结构有效')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
