#!/usr/bin/env python3
"""Prepare per-round obstacle files consumed by control_actor.py."""

import argparse
import ast
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from check_referee_world import _derive_boxes_from_world


def load_boxes(world_path, black_box_path=None):
    if black_box_path is not None:
        boxes = ast.literal_eval(Path(black_box_path).read_text().strip())
    else:
        root = ET.parse(world_path).getroot()
        world = root if root.tag == "world" else root.find("world")
        if world is None:
            raise ValueError("world XML has no <world> element")
        boxes, _, _ = _derive_boxes_from_world(world)
    if not boxes:
        raise ValueError("no obstacle boxes available for actor control")
    return boxes


def obstacle_points(boxes):
    points = set()
    for (xmin, xmax), (ymin, ymax) in boxes:
        for xi in range(math.floor(xmin), math.ceil(xmax)):
            for yi in range(math.floor(ymin), math.ceil(ymax)):
                x, y = xi + 0.5, yi + 0.5
                if xmin <= x <= xmax and ymin <= y <= ymax:
                    points.add((x, y))
        points.add(((xmin + xmax) / 2.0, (ymin + ymax) / 2.0))

    for xi in range(-50, 130):
        points.add((xi + 0.5, -50.0))
        points.add((xi + 0.5, 50.0))
    for yi in range(-50, 50):
        points.add((-50.0, yi + 0.5))
        points.add((130.0, yi + 0.5))
    return sorted(points)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--black-box")
    args = parser.parse_args()

    boxes = load_boxes(args.world, args.black_box)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "black_box.txt").write_text(repr(boxes) + "\n")
    points = obstacle_points(boxes)
    with (out / "obstacle.txt").open("w") as output:
        for x, y in points:
            output.write("{:.6f} {:.6f}\n".format(x, y))
    print("actor control map: {} boxes, {} obstacle points".format(len(boxes), len(points)))


if __name__ == "__main__":
    main()
