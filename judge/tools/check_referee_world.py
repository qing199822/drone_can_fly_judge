#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""裁判就绪性检查：这份 world 能不能直接喂给 score_cal.py 跑一轮。

背景：随机地图（tools/map_gen/gen_round_world.py，来自 XTDrone 旧模块）会随机化建筑与人物
位置。若人物被放进建筑、模型名重复、或 world 里带 <state> 旧状态快照，会直接导致：
    - Gazebo 报 Non-unique names detected in type model，或模型被空壳顶替（不碰撞）
    - 加载时用 <state> 里的旧位姿覆盖随机化结果（随机图白做）
    - 人物被建筑/rover 困住、六机出生点被建筑压住 → 一轮 600s 白跑
本脚本把这些前提一次查完，任何 FAIL 都不要拿去跑。

用法：
    python3 tools/check_referee_world.py <world 文件> [black_box.txt]
    python3 tools/check_referee_world.py <world> --keep-state     # 允许 state 存在（仅调试）

退出码：0 通过；1 有致命问题
"""

import ast
import math
import os
import sys
import xml.etree.ElementTree as ET

ARENA_X = (-50.0, 130.0)
ARENA_Y = (-50.0, 50.0)
ACTOR_NUM = 6
DRONE_SPAWN_ORIGIN = (-17.0, -3.0)
DRONE_SPAWN_STEP = 3.0
DRONE_CLEARANCE = 1.5          # 出生点与建筑/rover 的最小距离（m）
ACTOR_MIN_SEPARATION = 1.0     # 人物之间的最小距离（m）
ROVER_RADIUS = 3.0             # rover 碰撞半径（保守估计）
PLUGIN_NAME = 'libros_actor_cmd_pose_plugin.so'

# 固定图（waypoint/world/baseN.world）通常**没有** black_box.txt，而它的建筑是 mesh 碰撞，
# 没法直接读 <box><size>。这里按型号前缀查"占地尺寸"表，把固定图的建筑/障碍也纳入
# "人物/出生点是否被压住"的检查，避免"没有 black_box.txt 就当检查通过"。
# 表来源：XTDrone 随机生成器里那张 size_box 表（按 name_list 顺序）+ 实测常见障碍外形。
# 顺序必须"长前缀在前"，否则 house_1_146 会先匹配掉 house_1_146_clone。
DERIVED_SIZE_TABLE = (
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
    # 注意：stop_sign / fire_hydrant 这类"细杆/小件"故意不查表 —— 实测 base0 里
    # stop_sign_186 距 drone0 出生点只有 1.9m，用 1x1 的估算框会误判成 FAIL；
    # 它们会出现在下面的 WARN 里（"无法判定占地尺寸，未纳入检查"），既不假放行也不误杀。
    ('dumpster', (3.0, 2.0)),
    ('Shelves high', (2.0, 2.0)),
    ('cessna_c172', (12.0, 10.0)),
)
# 地面/可走区域：不能当建筑，否则整张图都"在建筑里"
DERIVED_SKIP_PREFIXES = ('ground_plane', 'city_terrain', 'ocean', 'sidewalk', 'rover')


def _pose_xy(element):
    for child in element:
        if child.tag == 'pose' and (child.text or '').strip():
            try:
                values = [float(v) for v in child.text.split()]
            except ValueError:
                return None
            if len(values) >= 2:
                return values[0], values[1]
    return None


def _pose_xy_yaw(model):
    """模型的世界位姿 (x, y, yaw)：优先顶层 <pose>，没有就看第一个 <link> 的 <pose>。"""
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
    """从 world 里推导建筑/障碍 AABB，返回 (boxes, resolved_names, skipped_names)。

    优先用真实几何 <collision><box><size>（精确）；mesh 碰撞查 DERIVED_SIZE_TABLE。
    查不到尺寸的模型进 skipped（只告警，不据此判 FAIL，避免误杀）。
    """
    boxes, resolved, skipped = [], [], []
    for model in world_el.findall('model'):
        name = model.get('name') or ''
        if name.startswith(DERIVED_SKIP_PREFIXES):
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
        for prefix, dims in DERIVED_SIZE_TABLE:
            if name.startswith(prefix):
                size = dims
                break
        if size:
            boxes.append(_aabb(pose, size[0], size[1]))
            resolved.append(name)
        else:
            skipped.append(name)
    return boxes, resolved, skipped


def _dist(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def _find_plugin_library():
    candidates = []
    for path in (os.environ.get('GAZEBO_PLUGIN_PATH') or '').split(':'):
        if path:
            candidates.append(path)
    workspace = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    candidates += [os.path.join(workspace, 'devel', 'lib'),
                   os.path.join(workspace, 'gazebo_map_assets', 'plugins')]
    for directory in candidates:
        if os.path.isfile(os.path.join(directory, PLUGIN_NAME)):
            return os.path.join(directory, PLUGIN_NAME)
    return None


def main(argv):
    if not argv:
        print(__doc__)
        return 2
    world_path = os.path.abspath(argv[0])
    keep_state = '--keep-state' in argv
    args = [a for a in argv if not a.startswith('--')]
    black_box_path = args[1] if len(args) > 1 else os.path.join(os.path.dirname(world_path),
                                                                'black_box.txt')
    if not os.path.exists(world_path):
        print('错误：找不到 world 文件 %s' % world_path)
        return 1

    problems, warnings, infos = [], [], []

    try:
        root = ET.parse(world_path).getroot()
    except ET.ParseError as exc:
        print('错误：world XML 解析失败：%s' % exc)
        return 1
    world_el = root if root.tag == 'world' else root.find('world')
    if world_el is None:
        print('错误：world 文件里没有 <world> 元素')
        return 1

    # ---------- 1) <state> 旧状态快照 ----------
    if any(c.tag == 'state' for c in world_el) and not keep_state:
        state_models = [m.get('name') for s in world_el.findall('state') for m in s.iter('model')]
        problems.append('world 里存在 <state> 旧状态快照（%d 个模型条目）；加载时会覆盖随机化位姿，'
                        '且与顶层模型同名。请先执行：python3 tools/dedupe_world.py %s'
                        % (len(state_models), os.path.basename(world_path)))

    # ---------- 2) 模型名唯一 ----------
    top_names = []
    for child in list(world_el):
        if child.tag in ('model', 'actor'):
            top_names.append(child.get('name'))
        elif child.tag == 'include':
            name_child = child.find('name')
            if name_child is not None and (name_child.text or '').strip():
                top_names.append(name_child.text.strip())
    top_names = [n for n in top_names if n]
    duplicates = sorted({n for n in top_names if top_names.count(n) > 1})
    if duplicates:
        problems.append('顶层模型名重复 %d 个（示例 %s）→ Gazebo 会报 Non-unique names'
                        % (len(duplicates), duplicates[:3]))

    # ---------- 3) 地面与人物 ----------
    if 'ground_plane' not in top_names:
        problems.append('缺少 ground_plane（/gazebo/get_model_state(..., ground_plane) 依赖它）')

    actors = [a for a in world_el.iter('actor')]
    actor_names = [a.get('name') for a in actors]
    expected = {'actor_%d' % i for i in range(ACTOR_NUM)}
    missing = sorted(expected - set(actor_names))
    if missing:
        problems.append('缺少人物模型：%s（裁判按 actor_0..%d 读真值）' % (missing, ACTOR_NUM - 1))
    extra = sorted(n for n in actor_names if n not in expected)
    if extra:
        warnings.append('多出人物模型 %s（裁判不读，但可能干扰视觉）' % extra)

    poses = {}
    for actor in actors:
        name = actor.get('name')
        plugins = [p.get('filename', '') for p in actor.iter('plugin')]
        if not any('libros_actor_cmd_pose_plugin' in f for f in plugins):
            problems.append('%s 缺少 %s（tracking 的 /actor_*/cmd_vel 与 cmd_motion 由它提供）'
                            % (name, PLUGIN_NAME))
        init_pose = actor.find('.//init_pose')   # 在 plugin 内部
        if init_pose is None or not (init_pose.text or '').strip():
            problems.append('%s 缺少 <init_pose>' % name)
            continue
        try:
            values = [float(v) for v in init_pose.text.split()]
        except ValueError:
            problems.append('%s <init_pose> 无法解析：%r' % (name, init_pose.text))
            continue
        if len(values) < 3:
            problems.append('%s <init_pose> 少于 3 个数值' % name)
            continue
        x, y, z = values[0], values[1], values[2]
        poses[name] = (x, y, z)
        if not (ARENA_X[0] <= x <= ARENA_X[1] and ARENA_Y[0] <= y <= ARENA_Y[1]):
            problems.append('%s 初始位置 (%.1f, %.1f) 在场地外' % (name, x, y))
        if abs(z - 1.25) > 0.6:
            warnings.append('%s init_pose z=%.2f（常规 1.25）' % (name, z))

    names_sorted = sorted(poses)
    for i in range(len(names_sorted)):
        for j in range(i + 1, len(names_sorted)):
            a, b = names_sorted[i], names_sorted[j]
            distance = _dist(poses[a], poses[b])
            if distance < ACTOR_MIN_SEPARATION:
                problems.append('%s 与 %s 相距仅 %.2fm（<%.1fm），视觉/深度会互相干扰'
                                % (a, b, distance, ACTOR_MIN_SEPARATION))

    if not _find_plugin_library():
        problems.append('找不到 %s（GAZEBO_PLUGIN_PATH / devel/lib / gazebo_map_assets/plugins 都没有）'
                        % PLUGIN_NAME)

    # ---------- 4) 建筑/障碍 black_box（人物/出生点占位）----------
    # 随机图：用生成器产出的 black_box.txt（权威）。固定图：旁边通常没有 black_box.txt，
    # 就从 world 自己推导建筑框 —— 绝不允许"没有 black_box.txt = 检查通过"。
    boxes = []
    box_source = None
    black_box_used = False
    if black_box_path and os.path.exists(black_box_path):
        try:
            parsed = ast.literal_eval(open(black_box_path, encoding='utf-8').read().strip())
            if parsed:
                boxes = parsed
                box_source = 'black_box.txt'
                black_box_used = True
        except (ValueError, SyntaxError) as exc:
            warnings.append('black_box.txt 解析失败（%s），改为从 world 推导建筑框' % exc)
    if not boxes:
        derived, resolved, skipped = _derive_boxes_from_world(world_el)
        if derived:
            boxes = derived
            box_source = 'world 推导（本图没有 black_box.txt）'
            infos.append('从 world 推导出 %d 个建筑/障碍框（命中 %d 个模型）'
                         % (len(boxes), len(resolved)))
            if skipped:
                warnings.append('有 %d 个模型无法判定占地尺寸，未纳入检查（前 5 个：%s）'
                                % (len(skipped), skipped[:5]))
        else:
            problems.append('无法确定建筑/障碍占位：本图既没有可用的 black_box.txt，'
                            '也无法从 world 推导出任何框 → 检查不完整，不能拿去跑一轮')
    if box_source:
        infos.append('占位检查依据：%s（%d 个框）' % (box_source, len(boxes)))

    def hits_box(x, y, margin=0.5):
        for box in boxes:
            try:
                (xa, xb), (ya, yb) = box
            except (TypeError, ValueError):
                continue
            if xa - margin <= x <= xb + margin and ya - margin <= y <= yb + margin:
                return (xa, xb, ya, yb)
        return None

    for name, (x, y, _z) in poses.items():
        box = hits_box(x, y)
        if box:
            problems.append('%s 初始位置 (%.1f, %.1f) 落在建筑 [%.1f,%.1f]×[%.1f,%.1f] 内'
                            % (name, x, y, box[0], box[1], box[2], box[3]))

    # ---------- 5) rover 占位 ----------
    rovers = {}
    for model in world_el.findall('model'):
        name = model.get('name') or ''
        if name.startswith('rover'):
            xy = _pose_xy(model)
            if xy:
                rovers[name] = xy
    for name, (x, y, _z) in poses.items():
        for rover, (rx, ry) in rovers.items():
            if _dist((x, y), (rx, ry)) < ROVER_RADIUS:
                problems.append('%s (%.1f,%.1f) 与 %s (%.1f,%.1f) 距离 <%.1fm，人物可能卡在 rover 上'
                                % (name, x, y, rover, rx, ry, ROVER_RADIUS))

    # ---------- 6) 六机出生点 ----------
    spawns = {}
    for drone_id in range(6):
        sx = DRONE_SPAWN_ORIGIN[0] + DRONE_SPAWN_STEP * (drone_id % 2)
        sy = DRONE_SPAWN_ORIGIN[1] + DRONE_SPAWN_STEP * (drone_id // 2)
        spawns['typhoon_h480_%d' % drone_id] = (sx, sy)
    for name, (x, y) in spawns.items():
        box = hits_box(x, y, margin=DRONE_CLEARANCE)
        if box:
            problems.append('%s 出生点 (%.1f,%.1f) 距建筑 [%.1f,%.1f]×[%.1f,%.1f] <%.1fm，'
                            'spawn 会失败/开局撞墙'
                            % (name, x, y, box[0], box[1], box[2], box[3], DRONE_CLEARANCE))
        for rover, (rx, ry) in rovers.items():
            if _dist((x, y), (rx, ry)) < ROVER_RADIUS + DRONE_CLEARANCE:
                problems.append('%s 出生点 (%.1f,%.1f) 距 %s 太近（<%.1fm）'
                                % (name, x, y, rover, ROVER_RADIUS + DRONE_CLEARANCE))

    # ---------- 7) 资源路径 ----------
    # 搜索路径必须与启动器（start_referee.sh / start_official_judge.sh）实际使用的
    # GAZEBO_MODEL_PATH 一致，否则会误报"缺资源"。
    # 历史误报（2026-10-10）：cardboard_box / cessna 在 ~/robocup_fly/gazebo_models，
    # casual_female 在 $ZZU_PX4_DIR/Tools/sitl_gazebo/models，而这里原来只找
    # ~/PX4_Firmware（本机那是 v1.13.2 的软链，不是实际用的树）→ 三条假 WARN。
    # 假 WARN 的危害不只是噪音：它和"真缺模型"长得一样，而真缺模型会让 gzserver
    # 去连在线模型库并永久卡死（SYN-SENT），所以必须让两者可区分。
    model_paths = [p for p in (os.environ.get('GAZEBO_MODEL_PATH') or '').split(':') if p]
    workspace = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    home = os.path.expanduser('~')
    px4_candidates = [
        os.environ.get('ZZU_PX4_DIR'),          # 启动器实际用的 PX4 树
        os.path.join(home, 'robocup_fly', 'PX4_Firmware'),
        os.path.expanduser('~/PX4_Firmware'),
        os.path.expanduser('~/PX4-Autopilot'),
    ]
    extra_dirs = [
        os.path.join(workspace, 'gazebo_map_assets', 'models'),
        os.path.join(workspace, 'XTDrone', 'sitl_config', 'models'),
        os.path.join(workspace, 'XTDrone', 'sitl_config', 'gazebo_plugin', 'citysim', 'models'),
        os.path.join(home, 'robocup_fly', 'gazebo_models'),
        os.path.join(home, 'gazebo_models'),
        os.path.join(home, '.gazebo', 'models'),
        os.path.join(home, 'XTDrone', 'sitl_config', 'models'),
    ]
    for px4_root in px4_candidates:
        if px4_root:
            extra_dirs.append(os.path.join(px4_root, 'Tools', 'sitl_gazebo', 'models'))
    for d in extra_dirs:
        if os.path.isdir(d):
            model_paths.append(d)
    missing_resources = []
    for uri in set(u for u in (el.text or '' for el in world_el.iter('uri')) if u.startswith('model://')):
        relative = uri[len('model://'):].split('/')
        if not relative:
            continue
        resolved = any(os.path.exists(os.path.join(base, *relative)) for base in model_paths)
        if not resolved:
            missing_resources.append(uri)
    if missing_resources:
        # 升级为硬失败：真缺 model:// 资源时，gzserver 会转去连 Gazebo 在线模型库；
        # 本机国际直连是 SYN-SENT 挂死（不是快速失败），表现为"gzserver 占用 100% CPU、
        # 零输出、gazebo 服务不注册、整轮超时"。宁可开跑前就拦下来。
        problems.append('❗真缺 model:// 资源（Gazebo 会转去连在线模型库并可能永久卡死）：%s'
                        % sorted(missing_resources)[:5])
        problems.append('  已搜索目录：%s' % ':'.join(model_paths))

    # ---------- 输出 ----------
    print('=== 裁判就绪性检查：%s ===' % world_path)
    for name in sorted(poses):
        x, y, z = poses[name]
        print('  %-8s init_pose=(%7.2f, %7.2f, %5.2f)' % (name, x, y, z))
    print('  顶层模型 %d 个 / 唯一 %d 个；rover %d 个；建筑框 %d 个'
          % (len(top_names), len(set(top_names)), len(rovers), len(boxes)))
    for info in infos:
        print('  INFO  %s' % info)
    for warning in warnings:
        print('  WARN  %s' % warning)
    for problem in problems:
        print('  FAIL  %s' % problem)

    if problems:
        print('结果：不通过（%d 个问题）——不要拿去跑一轮；可重新生成或用 tools/dedupe_world.py 清理'
              % len(problems))
        return 1
    print('结果：通过（模型唯一 / 无 state / 人物与出生点占位 / 插件与资源均正常）')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
