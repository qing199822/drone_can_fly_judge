#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Associate visually identical red detections with actor_4 and actor_5."""

import math


def _dist2(a, b):
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2


def transform_point(position, translation, quaternion):
    """Apply a TF translation and quaternion to a 3-D point."""
    x, y, z, w = quaternion
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm == 0.0:
        return None
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    px, py, pz = position
    return (
        translation[0] + (1 - 2*(y*y + z*z))*px + 2*(x*y - z*w)*py + 2*(x*z + y*w)*pz,
        translation[1] + 2*(x*y + z*w)*px + (1 - 2*(x*x + z*z))*py + 2*(y*z - x*w)*pz,
        translation[2] + 2*(x*z - y*w)*px + 2*(y*z + x*w)*py + (1 - 2*(x*x + y*y))*pz,
    )


def nearest_red_id(drone_xy, red_positions, min_gap=5.0):
    """Return the sole surviving red actor, or the clearly nearer one."""
    available = [actor_id for actor_id in (4, 5) if actor_id in red_positions]
    if len(available) == 1:
        return available[0]
    if drone_xy is None or len(available) != 2:
        return None
    d4 = math.sqrt(_dist2(drone_xy, red_positions[4]))
    d5 = math.sqrt(_dist2(drone_xy, red_positions[5]))
    if abs(d4 - d5) < float(min_gap):
        return None
    return 4 if d4 < d5 else 5


def match_near_actor(box, drone_xy, actor_positions, projected_positions,
                     max_world_distance=10.0, max_pixel_error=70.0,
                     min_pixel_gap=25.0):
    """Associate a weak detection with one nearby actor or return None."""
    if drone_xy is None:
        return None
    x1, y1, x2, y2 = box
    candidates = []
    for actor_id, position in actor_positions.items():
        projection = projected_positions.get(actor_id)
        if projection is None or projection[2] <= 0:
            continue
        if math.hypot(position[0] - drone_xy[0],
                      position[1] - drone_xy[1]) > max_world_distance:
            continue
        u, v = projection[:2]
        pixel_error = math.hypot(max(x1 - u, 0.0, u - x2),
                                 max(y1 - v, 0.0, v - y2))
        if pixel_error <= max_pixel_error:
            candidates.append((pixel_error, actor_id))
    candidates.sort()
    if not candidates or (len(candidates) > 1 and
                          candidates[1][0] - candidates[0][0] < min_pixel_gap):
        return None
    return candidates[0][1]


def resolve_red_labels(red_indices, drone_xy, red_positions,
                       red_centers=None, projected_positions=None,
                       max_pixel_error=90.0):
    """Map red box indices to actor IDs; omit boxes whose identity is uncertain."""
    if not red_indices:
        return {}

    red_centers = red_centers or {}
    projected_positions = projected_positions or {}
    available = {actor_id for actor_id in (4, 5) if actor_id in red_positions}
    candidates = []
    for idx in red_indices:
        if idx not in red_centers:
            continue
        cx, cy = red_centers[idx]
        for actor_id in available:
            projection = projected_positions.get(actor_id)
            if projection is None:
                continue
            error = math.hypot(cx - projection[0], cy - projection[1])
            if error <= max_pixel_error:
                candidates.append((error, idx, actor_id))

    labels = {}
    best_pair = None
    for first in candidates:
        for second in candidates:
            if first[1] == second[1] or first[2] == second[2]:
                continue
            score = first[0] + second[0]
            if best_pair is None or score < best_pair[0]:
                best_pair = (score, first, second)
    if best_pair is not None:
        for _, idx, actor_id in best_pair[1:]:
            labels[idx] = "red%d" % actor_id
    elif candidates:
        _, idx, actor_id = min(candidates)
        labels[idx] = "red%d" % actor_id

    if len(red_indices) == 1 and red_indices[0] not in labels:
        actor_id = nearest_red_id(drone_xy, red_positions)
        if actor_id is not None:
            labels[red_indices[0]] = "red%d" % actor_id
    return labels
