"""Place the 13 movable buildings in a base SDF world."""

import random
import xml.etree.ElementTree as ET


BUILDING_NAMES = (
    'house_1_146', 'house_3_156', 'house_3_157', 'house_3_158',
    'gas_station_73', 'fast_food_93', 'house_1_66', 'house_1_67',
    'house_1_146_clone', 'house_2_71', 'house_2_125', 'house_2_126',
    'house_3_68',
)

# Placement clearance sizes, in the same order as BUILDING_NAMES.
PLACEMENT_SIZES = (
    (16, 7), (1, 1), (2, 1), (2, 1), (20, 30), (30, 17),
    (16, 12), (16, 12), (16, 16), (11, 9), (11, 9), (11, 9), (17, 6),
)


def _candidate(index, map_num, rand_x):
    if index == 0:
        y = -32 if random.random() > 0.5 else 15
        if map_num == 6:
            y = 14.5
        if map_num in (15, 11, 10, 7):
            y = 14.3
        if map_num in (5, 13, 12, 9, 8, 16):
            y = -24
    elif index == 1:
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
    else:
        y = int(random.choice((-1, 1)) * random.uniform(14, 30))
        if map_num in (6, 15, 14, 11, 10, 7):
            y = random.uniform(-30, 3)
            if y >= -23:
                y += 27
        if map_num in (5, 13, 12, 9, 8, 16):
            y = random.uniform(-30, 3)
            if y >= -3:
                y += 27
    return int(rand_x(random.uniform(0.33, 1))), y


def generate_centers(map_num, rand_x, attempts=10):
    """Return 13 positions, or fail rather than emit an overlapping layout."""
    fixed = [[-27.5, 25.5], [-33.4, -21.4], [-28.2, -33], [-25.8, -19.4]]
    obstacles = [[86.6, -30], [-27, 14.65], [15.9, -13.6], [5.6, 35.7]]
    if map_num == 6:
        fixed = [[-28.64, 25.56], [-32.5, 1.8], [-28.2, -22.7], [-28.1, -33]]
        obstacles = [[86.6, -30], [-27, 14.65], [14.2, -20.6], [5.6, 35.7]]

    for _ in range(attempts):
        centers = [point[:] for point in fixed]
        placed = [point[:] for point in obstacles]
        for index in range(4, len(BUILDING_NAMES)):
            for _ in range(1000):
                x, y = _candidate(index - 4, map_num, rand_x)
                width, height = PLACEMENT_SIZES[index]
                if all(abs(x - px) > (width + PLACEMENT_SIZES[j][0]) / 2 + 3 or
                       abs(y - py) > (height + PLACEMENT_SIZES[j][1]) / 2 + 3
                       for j, (px, py) in enumerate(placed)):
                    centers.append([x, y])
                    placed.append([x, y])
                    break
            else:
                break
        if len(centers) == len(BUILDING_NAMES):
            return centers
    raise RuntimeError('Could not place all 13 buildings after %d layouts' % attempts)


def place_buildings(base_text, centers):
    """Change actual model/link poses and discard stale Gazebo state snapshots."""
    if len(centers) != len(BUILDING_NAMES):
        raise ValueError('Expected %d building centers' % len(BUILDING_NAMES))
    root = ET.fromstring(base_text)
    world = root if root.tag == 'world' else root.find('world')
    if world is None:
        raise ValueError('Base map has no world element')
    models = {model.get('name'): model for model in world.findall('model')}
    for name, (x, y) in zip(BUILDING_NAMES, centers):
        model = models.get(name)
        if model is None:
            raise ValueError('Base map is missing building %s' % name)
        pose = model.find('pose')
        if pose is None:
            pose = model.find('link/pose')
        if pose is None or not pose.text:
            raise ValueError('Building %s has no model or link pose' % name)
        values = pose.text.split()
        if len(values) != 6:
            raise ValueError('Building %s has an invalid pose' % name)
        pose.text = '%s %s %s' % (x, y, ' '.join(values[2:]))
    for state in world.findall('state'):
        world.remove(state)
    return ET.tostring(root, encoding='unicode')
