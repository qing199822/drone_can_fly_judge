#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 robocup.world 画成 2D 俯视图（PNG），headless 也能用。

用法：
    python3 tools/plot_world.py <world 文件> [--out 输出.png] [--no-sidewalk] [--title 标题]

画的内容（全部来自 world 文件本身，不依赖 Gazebo/ROS）：
    - <road> 路面（宽 7.4m 的样条，按宽度画成粗带）
    - 建筑/障碍 AABB（<collision><box><size> 精确几何；mesh 用 check_referee_world 的尺寸表）
    - rover（2x2，来自 map_generator 的布置规则）
    - 人物 actor（init_pose 点 + 名字）
    - 场地边界（x∈[-50,130], y∈[-50,50]）
"""
import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_referee_world import DERIVED_SIZE_TABLE, DERIVED_SKIP_PREFIXES, _pose_xy_yaw  # noqa: E402

import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import PolyCollection  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

ARENA = (-50.0, 130.0, -50.0, 50.0)


def _world(path):
    root = ET.parse(path).getroot()
    return root if root.tag == 'world' else root.find('world')


def _road_polys(world):
    """把每条 <road> 按宽度画成若干四边形（不依赖 matplotlib 的线宽换算）。"""
    polys = []
    for road in world.findall('road'):
        width_el = road.find('width')
        try:
            width = float(width_el.text) if width_el is not None else 3.0
        except ValueError:
            width = 3.0
        pts = []
        for p in road.findall('point'):
            try:
                v = [float(x) for x in p.text.split()]
            except (ValueError, AttributeError):
                continue
            if len(v) >= 2:
                pts.append((v[0], v[1]))
        for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
            dx, dy = x2 - x1, y2 - y1
            length = math.hypot(dx, dy)
            if length < 1e-6:
                continue
            nx, ny = -dy / length * width / 2.0, dx / length * width / 2.0
            polys.append([(x1 + nx, y1 + ny), (x2 + nx, y2 + ny),
                          (x2 - nx, y2 - ny), (x1 - nx, y1 - ny)])
    return polys


def _sidewalk_lines(world):
    lines = []
    for model in world.findall('model'):
        if not (model.get('name') or '').startswith('sidewalk'):
            continue
        for col in model.iter('collision'):
            geo = col.find('geometry')
            if geo is None:
                continue
            for poly in geo.findall('polyline'):
                pts = []
                for p in poly.findall('point'):
                    try:
                        v = [float(x) for x in p.text.split()]
                    except (ValueError, AttributeError):
                        continue
                    if len(v) >= 2:
                        pts.append((v[0], v[1]))
                if len(pts) >= 2:
                    lines.append(pts)
    return lines


def _boxes(world):
    """建筑/障碍 AABB：box 几何精确，mesh 查尺寸表。"""
    out = []
    for model in world.findall('model'):
        name = model.get('name') or ''
        if name.startswith(DERIVED_SKIP_PREFIXES):
            continue
        pose = _pose_xy_yaw(model)
        if pose is None:
            continue
        exact = False
        for col in model.iter('collision'):
            size_el = col.find('geometry/box/size')
            if size_el is None or not (size_el.text or '').strip():
                continue
            try:
                v = [float(x) for x in size_el.text.split()]
            except ValueError:
                continue
            x, y, yaw = pose
            c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
            hx = (v[0] * c + v[1] * s) / 2.0
            hy = (v[0] * s + v[1] * c) / 2.0
            out.append((name, x - hx, y - hy, 2 * hx, 2 * hy, 'obstacle'))
            exact = True
        if exact:
            continue
        for prefix, dims in DERIVED_SIZE_TABLE:
            if name.startswith(prefix):
                x, y, yaw = pose
                c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
                hx = (dims[0] * c + dims[1] * s) / 2.0
                hy = (dims[0] * s + dims[1] * c) / 2.0
                out.append((name, x - hx, y - hy, 2 * hx, 2 * hy, 'building'))
                break
    return out


def _actors(world):
    out = []
    for actor in world.iter('actor'):
        ip = actor.find('.//init_pose')
        if ip is None or not (ip.text or '').strip():
            continue
        v = [float(x) for x in ip.text.split()]
        if len(v) >= 2:
            out.append((actor.get('name'), v[0], v[1]))
    return out


def _rovers(world):
    out = []
    seen = set()
    for model in world.findall('model'):
        name = model.get('name') or ''
        if not name.startswith('rover'):
            continue
        pose = _pose_xy_yaw(model)
        if not pose:
            continue
        key = (name, round(pose[0], 2), round(pose[1], 2))
        if key in seen:
            continue
        seen.add(key)
        out.append((name, pose[0], pose[1]))
    return out


def main(argv):
    parser = argparse.ArgumentParser(description='把 world 画成 2D 俯视图 PNG')
    parser.add_argument('world')
    parser.add_argument('--out', default=None)
    parser.add_argument('--title', default=None)
    parser.add_argument('--no-sidewalk', action='store_true')
    args = parser.parse_args(argv)

    world = _world(args.world)
    roads = _road_polys(world)
    sidewalks = [] if args.no_sidewalk else _sidewalk_lines(world)
    boxes = _boxes(world)
    actors = _actors(world)
    rovers = _rovers(world)

    fig, ax = plt.subplots(figsize=(16, 8.5), dpi=110)
    ax.set_facecolor('#f3f0e7')
    if roads:
        ax.add_collection(PolyCollection(roads, facecolors='#8d8d8d', edgecolors='none', zorder=1))
    for line in sidewalks:
        xs, ys = zip(*line)
        ax.plot(xs, ys, color='#b9b2a3', lw=0.8, zorder=2)
    for name, x, y, w, h, kind in boxes:
        color = '#c98a5b' if kind == 'building' else '#8fb98f'
        ax.add_patch(Rectangle((x, y), w, h, facecolor=color, edgecolor='#5b4a3a',
                               lw=0.6, alpha=0.9, zorder=3))
    for name, x, y in rovers:
        ax.add_patch(Rectangle((x - 1, y - 1), 2, 2, facecolor='#3f6fb5', edgecolor='#17325e',
                               lw=0.6, zorder=4))
    if actors:
        ax.scatter([a[1] for a in actors], [a[2] for a in actors], s=70, marker='o',
                   facecolor='#d33', edgecolor='white', zorder=6, label='人物 actor')
        for name, x, y in actors:
            ax.annotate(name, (x, y), textcoords='offset points', xytext=(6, 6),
                        fontsize=8, color='#8b0000', zorder=6)
    if rovers:
        ax.scatter([], [], marker='s', s=40, facecolor='#3f6fb5', edgecolor='#17325e', label='rover(%d)' % len(rovers))
    x0, x1, y0, y1 = ARENA
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, edgecolor='#333', lw=1.6))
    ax.set_xlim(x0 - 3, x1 + 3); ax.set_ylim(y0 - 3, y1 + 3)
    ax.set_aspect('equal'); ax.grid(alpha=0.15, lw=0.4)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.set_title(args.title or '%s\n路 %d 条 / 建筑障碍 %d / rover %d / 人物 %d' % (
        os.path.basename(args.world), len(roads), len(boxes), len(rovers), len(actors)))
    ax.legend(loc='upper right', fontsize=9)
    out = args.out or os.path.splitext(args.world)[0] + '_top.png'
    fig.tight_layout()
    fig.savefig(out)
    print('已生成：%s' % out)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
