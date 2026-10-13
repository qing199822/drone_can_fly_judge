#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Copy a Gazebo world and randomize actor init_pose values before launch.

When --black-box is not provided (e.g. fixed base0.world without companion
black_box.txt), this script falls back to deriving obstacle AABBs from the
world XML itself, mirroring tools/check_referee_world.py:_derive_boxes_from_world.
"""
import argparse
import ast
import math
import os
import random
import sys
import xml.etree.ElementTree as ET

ARENA_X = (-50.0, 130.0)
ARENA_Y = (-50.0, 50.0)
ACTOR_NAMES = tuple('actor_%d' % i for i in range(6))
MIN_SEPARATION = 1.0

# Prefix table for mesh-collision models whose footprint is not embedded in SDF.
# Order matters: longer prefixes first.
_DERIVED_SIZE_TABLE = (
    ('house_1_146_clone', (18.0, 18.0)),
    ('house_1_146', (16.0, 12.0)),
    ('house_1_66', (16.0, 12.0)),
    ('house_1_67', (18.0, 18.0)),
    ('house_2_71', (11.0, 9.0)),
    ('house_2_125', (11.0, 9.0)),
    ('house_2_126', (11.0, 9.0)),
    ('house_3_156', (5.0, 17.0)),
    ('house_3_157', (17.0, 5.0)),
    ('house_3_158', (5.0, 17.0)),
    ('house_3_68', (14.0, 5.0)),
    ('gas_station', (20.0, 15.0)),
    ('fast_food', (30.0, 25.0)),
    ('lamp_post', (1.0, 1.0)),
    ('dumpster', (3.0, 2.0)),
    ('Shelves high', (2.0, 2.0)),
    ('cessna_c172', (12.0, 10.0)),
)
_DERIVED_SKIP_PREFIXES = ('ground_plane', 'city_terrain', 'ocean', 'sidewalk', 'rover')


def _inside_boxes(x, y, boxes):
    for box in boxes:
        try:
            (xa, xb), (ya, yb) = box
        except (TypeError, ValueError):
            continue
        if xa <= x <= xb and ya <= y <= yb:
            return True
    return False


def _load_boxes(path):
    if not path or not os.path.exists(path):
        return []
    try:
        value = ast.literal_eval(open(path, encoding='utf-8').read().strip())
        return value if isinstance(value, list) else []
    except (OSError, ValueError, SyntaxError):
        return []


def _pose_xy_yaw(model):
    for source in [model] + list(model.findall('link')):
        for child in source:
            if child.tag != 'pose' or not (child.text or '').strip():
                continue
            try:
                values = [float(v) for v in child.text.split()]
            except ValueError:
                continue
            if len(values) >= 2:
                yaw = values[5] if len(values) >= 6 else 0.0
                return values[0], values[1], yaw
    return None


def _aabb(pose, length, width):
    x, y, yaw = pose
    cos_yaw, sin_yaw = abs(math.cos(yaw)), abs(math.sin(yaw))
    half_x = (length * cos_yaw + width * sin_yaw) / 2.0
    half_y = (length * sin_yaw + width * cos_yaw) / 2.0
    return [[x - half_x, x + half_x], [y - half_y, y + half_y]]


def _derive_boxes_from_world(world_el):
    boxes, resolved, skipped = [], [], []
    for model in world_el.findall('model'):
        name = model.get('name') or ''
        if name.startswith(_DERIVED_SKIP_PREFIXES):
            continue
        pose = _pose_xy_yaw(model)
        if pose is None:
            skipped.append('%s(无位姿)' % name)
            continue
        exact = False
        for collision in model.iter('collision'):
            size_el = collision.find('geometry/box/size')
            if size_el is None or not (size_el.text or '').strip():
                continue
            try:
                values = [float(v) for v in size_el.text.split()]
            except ValueError:
                continue
            if len(values) >= 2 and values[0] > 0 and values[1] > 0:
                boxes.append(_aabb(pose, values[0], values[1]))
                exact = True
        if exact:
            resolved.append(name)
            continue
        size = None
        for prefix, dims in _DERIVED_SIZE_TABLE:
            if name.startswith(prefix):
                size = dims
                break
        if size:
            boxes.append(_aabb(pose, size[0], size[1]))
            resolved.append(name)
        else:
            skipped.append(name)
    return boxes, resolved, skipped


def _collect_boxes(args, world_el):
    if args.black_box and os.path.exists(args.black_box):
        boxes = _load_boxes(args.black_box)
        if boxes:
            return boxes
    derived, _, _ = _derive_boxes_from_world(world_el)
    return derived


def _choose_positions(rng, x_min, x_max, y_min, y_max, boxes,
                      max_attempts_per_actor=20000):
    if x_min > x_max or y_min > y_max:
        raise ValueError('invalid x/y bounds')
    positions = []
    for actor_idx, _name in enumerate(ACTOR_NAMES):
        for _attempt in range(max_attempts_per_actor):
            point = (rng.uniform(x_min, x_max), rng.uniform(y_min, y_max))
            if _inside_boxes(point[0], point[1], boxes):
                continue
            if all(math.hypot(point[0] - old[0], point[1] - old[1]) >= MIN_SEPARATION
                   for old in positions):
                positions.append(point)
                break
        else:
            raise RuntimeError(
                'unable to place %d actors inside [%g,%g]x[%g,%g] avoiding %d buildings '
                '(last point tried: (%.2f, %.2f))'
                % (actor_idx + 1, x_min, x_max, y_min, y_max, len(boxes),
                   point[0], point[1]))
    return positions


def main(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True, help='source .world XML')
    parser.add_argument('--dest', required=True, help='destination .world XML')
    parser.add_argument('--seed', type=int, default=None)
    parser.add_argument('--x-min', type=float, default=ARENA_X[0])
    parser.add_argument('--x-max', type=float, default=ARENA_X[1])
    parser.add_argument('--y-min', type=float, default=ARENA_Y[0])
    parser.add_argument('--y-max', type=float, default=ARENA_Y[1])
    parser.add_argument('--actor-z', '--z', dest='actor_z', type=float, default=1.25)
    parser.add_argument('--black-box', default=None,
                        help='optional black_box.txt with obstacle AABBs')
    args = parser.parse_args(argv)

    root = ET.parse(args.source).getroot()
    world = root if root.tag == 'world' else root.find('world')
    if world is None:
        raise ValueError('world XML has no <world> element')
    actors = {actor.get('name'): actor for actor in world.iter('actor')}
    missing = [name for name in ACTOR_NAMES if name not in actors]
    if missing:
        raise ValueError('missing actors: %s' % ', '.join(missing))
    boxes = _collect_boxes(args, world)
    print('obstacle boxes: %d (source: %s)'
          % (len(boxes), 'black_box.txt' if args.black_box and os.path.exists(args.black_box)
             else 'derived from world XML'), file=sys.stderr)
    positions = _choose_positions(random.Random(args.seed), args.x_min, args.x_max,
                                  args.y_min, args.y_max, boxes)
    for name, (x, y) in zip(ACTOR_NAMES, positions):
        init_pose = actors[name].find('.//init_pose')
        if init_pose is None or not (init_pose.text or '').strip():
            raise ValueError('%s has no usable <init_pose>' % name)
        values = init_pose.text.split()
        if len(values) < 3:
            raise ValueError('%s <init_pose> has fewer than 3 values' % name)
        values[0], values[1], values[2] = ('%.9g' % x, '%.9g' % y, '%.9g' % args.actor_z)
        init_pose.text = ' '.join(values)
        print('actor %s -> (%.2f, %.2f, %.2f)' % (name, x, y, args.actor_z))

    parent = os.path.dirname(os.path.abspath(args.dest))
    if parent:
        os.makedirs(parent, exist_ok=True)
    ET.register_namespace('', 'http://sdformat.org/sdf/1.6')
    ET.ElementTree(root).write(args.dest, encoding='utf-8', xml_declaration=True)
    print('wrote %s' % args.dest)
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main(sys.argv[1:]))
    except (ET.ParseError, OSError, ValueError, RuntimeError) as exc:
        print('error: %s' % exc, file=sys.stderr)
        sys.exit(1)