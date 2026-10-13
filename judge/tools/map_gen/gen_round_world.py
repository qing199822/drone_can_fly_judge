#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""随机地图生成器（内化自 XTDrone/robocup/map_generator.py，2026-09-15）。

新裁判 score_cal.py **不涉及**地图生成，而比赛规则要求"房子/灯杆等位置每次尝试前随机生成"，
因此这一块沿用 XTDrone 旧模块的逻辑：
  - 从 waypoint/world/base<N>.world 取基础地图（道路布局固定）
  - 随机化建筑（house_*/gas_station/fast_food）的位置与朝向
  - 随机布置 rover（rover_static.world）
  - 随机化 6 个人物的 init_pose（避开建筑 black_box ±3m）
  - 输出 robocup.world + black_box.txt + obstacle.txt

与原脚本相比，支持工作空间路径、参数化输出及建筑落点校验：
  1. base<N>.world 从工作空间读取（原脚本读 CWD 下的副本）；
  2. rover_static.world 从本目录 assets/ 读取；
  3. 产物写到 --out 目录，base 可由 --base 指定（原脚本写死 random.randint(1,3)，
     输出硬编码到作者机器的 ~/PX4_Firmware/Tools/sitl_gazebo/worlds/）。

用法：
    python3 tools/map_gen/gen_round_world.py                    # 随机 base(1..3)
    python3 tools/map_gen/gen_round_world.py --base 5 --out /tmp/x
    python3 tools/map_gen/gen_round_world.py --seed 1234        # 复现同一张图
"""
import argparse
import math
import os
import random
import re
import sys
import time

from house_placement import generate_centers, place_buildings

# 人物插件行：兼容单引号/双引号两种写法（见下方 random_actor_pose 段落注释）
_ACTOR_PLUGIN_RE = re.compile(r"filename=['\"]libros_actor_cmd_pose_plugin\.so['\"]\s*>")

# 六机出生点（与 serial_swarm_bringup.py 的 spawn_pose()/check_referee_world.py 保持一致）
DRONE_SPAWN_ORIGIN = (-17.0, -3.0)
DRONE_SPAWN_STEP = 3.0
SPAWN_SAFE_RADIUS = 5.0


def _drone_spawn_points():
    return [(DRONE_SPAWN_ORIGIN[0] + DRONE_SPAWN_STEP * (i % 2),
             DRONE_SPAWN_ORIGIN[1] + DRONE_SPAWN_STEP * (i // 2)) for i in range(6)]


def _near_spawn(x, y, radius=SPAWN_SAFE_RADIUS):
    """是否落在六机出生点安全圈内（rover/人物压出生点 → spawn 失败或开局撞）"""
    return any((x - sx) ** 2 + (y - sy) ** 2 < radius * radius
               for sx, sy in _drone_spawn_points())


def _find_workspace(start):
    directory = os.path.abspath(start)
    for _ in range(8):
        if os.path.exists(os.path.join(directory, '.catkin_workspace')):
            return directory
        parent = os.path.dirname(directory)
        if parent == directory:
            break
        directory = parent
    return os.path.abspath(os.path.join(start, '..', '..'))


_parser = argparse.ArgumentParser(description='随机地图生成（XTDrone robocup 逻辑内化版）')
_parser.add_argument('--base', default='random',
                     help='基础地图编号（0~16）或 random（默认：从 base1/base2/base3 随机抽一张）')
_parser.add_argument('--out', default=None, help='输出目录（默认 my_maps/rounds/<时间戳>）')
_parser.add_argument('--seed', type=int, default=None, help='随机种子（复现同一张图）')
_parser.add_argument('--rovers', type=int, default=20,
                     help='rover 数量（默认 20，与原脚本一致；设 0 可显著降低 Gazebo 负载）')
_args, _unknown = _parser.parse_known_args()

_WS = _find_workspace(os.path.dirname(os.path.abspath(__file__)))
BASE_DIR = os.path.join(_WS, 'waypoint', 'world')
ASSET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'assets')
# 注意：脚本中间会 os.chdir(OUT_DIR)，之后再用 abspath(__file__) 就算错了，
# 所以这里先把 tools/ 目录记下来（去重工具 dedupe_world.py 在 tools/ 下）。
TOOLS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# 同理：os.chdir(OUT_DIR) 之后 abspath(__file__) 就指向 OUT_DIR 了，必须提前记住自身路径
# （布局自检不合格时要用它重新 exec 自己换种子重生成）。
_SELF_SCRIPT = os.path.abspath(__file__)
if _args.out:
    OUT_DIR = os.path.abspath(_args.out)
else:
    OUT_DIR = os.path.join(_WS, 'my_maps', 'rounds', time.strftime('%Y%m%d_%H%M%S'))
os.makedirs(OUT_DIR, exist_ok=True)

if _args.seed is not None:
    random.seed(_args.seed)


# order:1_66,1_67,1_146,1_146_clone,2_71,2_125,2_126,3_68,3_156,3_157,3_158,73(gas-station),93(fast-food)
# 'gas_station_73','fast_food_93','house_1_66','house_1_67','house_1_146','house_1_146_clone','house_2_71','house_2_125','house_2_126','house_3_68','house_3_156','house_3_157','house_3_158']
# 'house_1_146','house_3_156','house_3_157','house_3_158','gas_station_73','fast_food_93','house_1_66','house_1_67','house_1_146_clone','house_2_71','house_2_125','house_2_126','house_3_68']
# size[x,y] of them

os.chdir(OUT_DIR)
## file path
output_path = OUT_DIR + os.sep
rover_num = max(0, int(_args.rovers))


def rand_x(num):
    correct=[[0,0],[2,2],[5,5],[-15,15],[15,-15],[0,0],[2,2],[5,0],[0,0],[24,-15],[17.5,-15],[15,-15],[18,-15],[3.7,5],[5,5],[17.5,-15],[15,-15]]
    if num > 0.33 and num < 0.66:
        x = random.uniform(61.3+correct[map_num][0], 95.5)
    elif num > 0.66:
        x = random.uniform(3.55, 30.8-correct[map_num][1])
        if map_num==8 or map_num==7:
            x = random.uniform(2.55, 27.8)
    else:
        x = random.uniform(-30.7, -29.7)
    return x


def create_point():
    return generate_centers(map_num, rand_x)


def obstacle_list(center_list):
    # generate obstacle list for human avoidance
    size_box = [[16, 12], [5, 17], [17, 5], [5, 17], [20, 15], [30, 25], [16, 12], [18, 18], [18, 18], [11, 9], [11, 9],
                [11, 9], [14, 5], [2, 2]]
    black_box = [[[-35.375, -34.825], [-4, -3]], [[-3.275, -2.725], [-4, -3]], [[20.725, 21.275], [-4, -3]], \
                 [[57.725, 58.275], [-4, -3]], [[83.725, 84.275], [-4, -3]], [[-25.475, -24.925], [3, 4]],
                 [[8.725, 9.275], [3, 4]], \
                 [[26.725, 27.275], [3, 4]], [[70.725, 71.275], [3, 4]], [[90.225, 90.775], [3, 4]],\
                [[79.4, 92.5], [-29.8, -28.6]], [[79, 85.2], [-33.7, -32.4]], [[88, 94.2], [-33.7, -32.4]],[[2.52 , 8.73],[34.5 , 35.7]]]
    for i in range(len(center_list)):
        j = i
        if i > len(size_box) - 1:
            j = len(size_box) - 1
        x_min = center_list[i][0] - size_box[j][0] / 2
        x_max = center_list[i][0] + size_box[j][0] / 2
        y_min = center_list[i][1] - size_box[j][1] / 2
        y_max = center_list[i][1] + size_box[j][1] / 2
        black_box.append([[x_min, x_max], [y_min, y_max]])
    return black_box


def change_list(list_o, d):
    list_result = []
    k = 1.0 * d / 2.0
    count = 0
    for item in list_o:
        count += 1
        y = item[1][0] + k
        while (y + k) <= item[1][1]:
            x = item[0][0] + k
            if count == 1:
                print('obstacle.txt generated!')
            # list_result.append([round(xx, 4),round(yy, 4)])
            list_result.append([x, y])
            while (x + k) <= item[0][1]:
                x += d
                # list_result.append([round(x, 4),round(yy, 4)])
                list_result.append([x, y])
            y += d
    return list_result


def create_human_point(black_box):
    count = 1
    while count < 1e5:
        a = random.uniform(-40, 110)
        b = random.uniform(-40, 40)
        in_obstacle = False
        
        for box in black_box:
            xmin, xmax = box[0]
            ymin, ymax = box[1]
            if (xmin - 3) < a < (xmax + 3) and (ymin - 3) < b < (ymax + 3):
                in_obstacle = True
                break
        
        if not in_obstacle:
            return int(a), int(b)
        
        count += 1
    
    print('ERROR: Actor generation failed after 100,000 attempts!')
    return None
    # in_obstacle = True
    # count=1
    # while in_obstacle and count<500:
    #     a = random.uniform(-40, 110)
    #     b = random.uniform(-40, 40)
    #     for i in range(int(len(black_box))):
    #         if (a > (black_box[i][0][0] - 3)) and (a < (black_box[i][0][1] + 3)) or (b > (black_box[i][1][0] - 3)) and (
    #                 b < (black_box[i][1][1] + 3)):
    #             in_obstacle = True
    #             count=count+1
    #             break
    #         else:
    #             in_obstacle = False
    # if count==100:
    #     a= random.choice((-40, 110))
    #     b= random.choice((-40, 40))
    #     print('AAA')
    # return int(a), int(b)


def rand_rover(p):
    if p == 0:
        x = random.uniform(-48, 122)
        y = random.uniform(-48, -42)
    elif p == 1:
        x = random.uniform(-48, -20) or random.uniform(-11, 122)
        y = random.uniform(-3, 3)
    elif p == 2:
        x = random.uniform(-48, 122)
        y = random.uniform(48, 42)
    elif p == 3:
        x = random.uniform(117, 123)
        y = random.uniform(-48, 48)
    elif p == 4:
        x = random.uniform(107, 113)
        y = random.uniform(-48, 48)
    elif p == 5:
        x = random.uniform(42, 48)
        y = random.uniform(-48, 48)
    elif p == 6:
        x = random.uniform(-11, -17)
        y = random.uniform(-48, -10) or random.uniform(10, 48)
    else:
        x = random.uniform(-41, -47)
        y = random.uniform(-48, 48)
    return [int(x), int(y)]


def create_rover_point(rover_num):
    in_obstacle = True
    rover_size = 2
    center_box = []
    for i in range(rover_num):
        in_obstacle = True
        while in_obstacle:
            p = math.floor(random.uniform(0, 8))
            [x, y] = rand_rover(p)
            # 与其它 rover 保持距离（原逻辑），且不能压住六机出生点（否则 spawn 即撞，
            # 见 tools/check_referee_world.py 的 FAIL 判据；实测 rovers=20 时约 8% 概率）
            in_obstacle = _near_spawn(x, y)
            for i in range(len(center_box)):
                if abs(x - center_box[i][0]) > rover_size + 2 or abs(y - center_box[i][1]) > rover_size + 2:
                    in_obstacle = False
                else:
                    in_obstacle = True
                    break
        center_box.append([x, y])
    return center_box

# 默认底图池：base1 / base2 / base3 随机抽一张（2026-09-28 按需求恢复随机池，
# start_referee.sh --world=random 走的就是这里）。
# 历史：曾因"base1/2/3 上六机 MAVROS 起不来"收窄为 [0]；该现象的真根因是
# PX4↔Gazebo lockstep 死锁（交接文档 §15.7），已在 serial_swarm_bringup.py 里
# 默认改写 <enable_lockstep>false 修掉，故这三张图重新参与随机。
# 想固定某张底图：--base=N（0~16）；想回到"只用 base0"：--base=0。
_base_choices = ([int(_args.base)] if _args.base not in ('random', '')
                 else [1, 2, 3])
map_num = random.choice(_base_choices)
print('Using map base%d' % map_num)
print('output dir: %s' % OUT_DIR)
# map_num=
print(map_num)

with open(os.path.join(BASE_DIR, "base%d.world" % map_num), encoding='utf-8') as source:
    base_text = source.read()
center_list = create_point()
with open('house.world', 'w', encoding='utf-8') as output:
    output.write(place_buildings(base_text, center_list))
print('Finish!')
## random_rover
rover_center_list = create_rover_point(rover_num)
rover_content = open(os.path.join(ASSET_DIR, "rover_static.world"), 'r')
with open("rover.world", 'w') as f:
    lines = rover_content.readlines()
    for i in range(rover_num):
        count = 0
        line_i = "<model name='rover_static_clone_clone_10_clone_clone_" + str(i + 1) + "'>\n"
        for num, line in enumerate(lines):
            if count == 1 and num + 1 < len(lines):
                if "</model>" in lines[num + 1]:
                    pose_ori = line.split()
                    line = " <pose frame=''>" + str(rover_center_list[i][0]) + ' ' + str(
                        rover_center_list[i][1]) + ' ' + str(pose_ori[-4]) + ' ' + str(pose_ori[-3]) + ' ' + str(
                        pose_ori[-2]) + ' ' + str(pose_ori[-1]) + '\n'
                    if count == 2 and num > 0:
                        if "<model name='rover_static" in line:
                            line = line_i
                        elif "<model name='rover_static" in lines[num - 1]:
                            pose_ori = line.split()
                            line = " <pose frame=''>" + str(rover_center_list[i][0]) + ' ' + str(
                                rover_center_list[i][1]) + ' ' + str(pose_ori[-4]) + ' ' + str(
                                pose_ori[-3]) + ' ' + str(pose_ori[-2]) + ' ' + str(pose_ori[-1]) + '\n'
                        elif "<link name='base_link'>" in lines[num - 1]:
                            pose_ori = line.split()
                            line = " <pose frame=''>" + str(rover_center_list[i][0]) + ' ' + str(
                                rover_center_list[i][1]) + ' ' + str(pose_ori[-4]) + ' ' + str(
                                pose_ori[-3]) + ' ' + str(pose_ori[-2]) + ' ' + str(pose_ori[-1]) + '\n'
                        elif "<link name='front_left_whe or map_num==10el_steering_block'>" in lines[num - 1]:
                            pose_ori = line.split()
                            line = " <pose frame=''>" + str(rover_center_list[i][0] - 1.2189) + ' ' + str(
                                rover_center_list[i][1] - 0.4431) + ' ' + str(pose_ori[-4]) + ' ' + str(
                                pose_ori[-3]) + ' ' + str(pose_ori[-2]) + ' ' + str(pose_ori[-1]) + '\n'
                        elif "<link name='front_right_wheel'>" in lines[num - 1]:
                            pose_ori = line.split()
                            line = " <pose frame=''>" + str(rover_center_list[i][0] - 1.1614) + ' ' + str(
                                rover_center_list[i][1] + 0.7712) + ' ' + str(pose_ori[-4]) + ' ' + str(
                                pose_ori[-3]) + ' ' + str(pose_ori[-2]) + ' ' + str(pose_ori[-1]) + '\n'
                        elif "<link name='front_right_wheel_steering_block'>" in lines[num - 1]:
                            pose_ori = line.split()
                            line = " <pose frame=''>" + str(rover_center_list[i][0] - 1.1715) + ' ' + str(
                                rover_center_list[i][1] + 0.5558) + ' ' + str(pose_ori[-4]) + ' ' + str(
                                pose_ori[-3]) + ' ' + str(pose_ori[-2]) + ' ' + str(pose_ori[-1]) + '\n'
                        elif "<link name='rear_left_wheel'>" in lines[num - 1]:
                            pose_ori = line.split()
                            line = " <pose frame=''>" + str(rover_center_list[i][0] + 0.964) + ' ' + str(
                                rover_center_list[i][1] - 0.7626) + ' ' + str(pose_ori[-4]) + ' ' + str(
                                pose_ori[-3]) + ' ' + str(pose_ori[-2]) + ' ' + str(pose_ori[-1]) + '\n'
                        elif "<link name='rear_right_wheel'>" in lines[num - 1]:
                            pose_ori = line.split()
                            line = " <pose frame=''>" + str(rover_center_list[i][0] + 1.0319) + ' ' + str(
                                rover_center_list[i][1] + 0.667) + ' ' + str(pose_ori[-4]) + ' ' + str(
                                pose_ori[-3]) + ' ' + str(pose_ori[-2]) + ' ' + str(pose_ori[-1]) + '\n'
                        elif "<link name='rover/imu_link'>" in lines[num - 1]:
                            pose_ori = line.split()
                            line = " <pose frame=''>" + str(rover_center_list[i][0]) + ' ' + str(
                                rover_center_list[i][1]) + ' ' + str(pose_ori[-4]) + ' ' + str(
                                pose_ori[-3]) + ' ' + str(pose_ori[-2]) + ' ' + str(pose_ori[-1]) + '\n'
                        else:
                            line = line
            if "<model name='rover_static_clone_clone_10_clone_clone" in line:
                line = line_i
                count = count + 1
            f.write(line)
f.close()
for i in range(len(rover_center_list)):
    center_list.append(rover_center_list[i])
black_box = obstacle_list(center_list)
with open("black_box.txt", 'w') as f:
    line = str(black_box)
    f.write(line)
f.close()
## random_actor_pose
print('Obstacle List generated!')
content = open("house.world", 'r')
rover_content = open("rover.world", 'r')
with open(output_path + "robocup.world", 'w') as f:
    lines = content.readlines()
    lines_rover = rover_content.readlines()
    count = 2
    for num, line in enumerate(lines):
        if count == 1:
            line = line_1
        # 注意：原脚本只匹配单引号写法（base1/2/3 是单引号），base*.world 里
        # 有的底图用双引号（如 base0），不兼容会导致人物 init_pose 根本没被随机化
        # → 人物留在底图默认点，随机建筑/rover 压上来（§15.7）。
        if _ACTOR_PLUGIN_RE.search(line):
            count = 0
            a, b = create_human_point(black_box)
            # 人物也不要压在六机出生点上（人物是实体，开局会挡住起飞）
            _guard = 0
            while _near_spawn(a, b) and _guard < 200:
                a, b = create_human_point(black_box)
                _guard += 1
            line_1 = "        <init_pose>" + str(a) + ' ' + str(b) + " 1.25 1.57 0 0</init_pose>"
            print(line_1)
        count = count + 1
        f.write(line)
        if num + 1 < len(lines):
            if "</world>\n" in lines[num + 1]:
                f.writelines(lines_rover)
f.close()
print('human initial pose generated!')

## obstacle.txt
result_list = change_list(black_box, 1.0)
length = len(result_list)
ARRS = []
f = open('obstacle.txt', 'w+')

for i in range(-50,130):
    f.write(str(i+0.5))
    f.write(' ')
    f.write("50")
    f.write(' ')
    f.write('\n')
    

for i in range(-50,130):
    f.write(str(i+0.5))
    f.write(' ')
    f.write("-50")
    
    f.write(' ')
    f.write('\n')

for i in range(-50,50):
    f.write("-50")
    f.write(' ')
    f.write(str(i+0.5))
    f.write(' ')
    f.write('\n')

for i in range(-50,50):
    f.write("130")
    f.write(' ')
    f.write(str(i+0.5))
    f.write(' ')
    f.write('\n')
  

for i in range(length):
    jointsFrame = result_list[i]
    ARRS.append(jointsFrame)
    for Ji in range(2):
        strNum = str(jointsFrame[Ji])
        f.write(strNum)
        f.write(' ')
    f.write('\n')
f.close()

# ---- 关键收尾：删除 <state> 世界状态快照 + 顶层模型去重 ----
# base<N>.world 每个模型名都出现 2 次（第 2 份在 <world><state> 里），会导致：
#   1) Gazebo 报 non-unique names；2) 加载时用旧状态覆盖随机化后的位姿（随机图白做）。
sys.path.insert(0, TOOLS_DIR)
from dedupe_world import clean_world_file  # noqa: E402

_world_path = os.path.join(OUT_DIR, 'robocup.world')
_report = clean_world_file(_world_path)
print('[clean_world] 删除 state 块 %d 个（%d 个模型状态）、顶层去重 %d 个，剩余唯一模型 %d'
      % (_report['state_blocks'], _report['state_entries'],
         len(_report['dropped_models']), _report['unique_models']))
if _report['duplicates_after']:
    print('ERROR: 输出 world 仍有重复模型名: %s' % _report['duplicates_after'][:5])
    sys.exit(1)

# 产出后自检：XML 必须可解析；顶层模型数/唯一数/rover 数/actor 数/state 都必须正确
import xml.etree.ElementTree as _ET
_root = _ET.parse(_world_path).getroot()
_world = _root if _root.tag == 'world' else _root.find('world')
_top = [c.get('name') for c in _world if c.tag in ('model', 'actor')]
_rover_n = len([n for n in _top if (n or '').startswith('rover')])
_actor_n = len([c for c in _world.iter('actor')])
_state_n = len([c for c in _world if c.tag == 'state'])
print('[postcheck] 顶层模型 %d / 唯一 %d / rover %d / actor %d / state 块 %d'
      % (len(_top), len(set(_top)), _rover_n, _actor_n, _state_n))
if len(_top) != len(set(_top)) or _state_n != 0 or _actor_n != 6 or _rover_n != rover_num:
    print('ERROR: 产出自检失败（重复模型/state/actor 数/rover 数不符）')
    sys.exit(1)

print('world : %s' % _world_path)
print('black_box/obstacle : %s' % OUT_DIR)

# ---- 布局就绪性自检（直接复用 tools/check_referee_world.py，单一判据来源）----
# 随机布局下"人物/rover/建筑互相压住"仍可能发生（例如 rover 停在六机出生点 3m 内），
# 而这类地图会被 起点脚本 判为不可用。这里直接调用同一个校验器，不合格就换种子重生成，
# 最多 GEN_MAX_ATTEMPTS 次（默认 8；显式给了 --seed 则不重试，保证可复现）。
import subprocess  # noqa: E402

_CHECKER = os.path.join(TOOLS_DIR, 'check_referee_world.py')
_attempt = int(os.environ.get('GEN_ROUND_ATTEMPT', '1'))
_max_attempts = max(1, int(os.environ.get('GEN_MAX_ATTEMPTS', '8')))
print('[layout] 布局就绪性检查（%s）...' % _CHECKER)
_proc = subprocess.run([sys.executable, _CHECKER, _world_path],
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
print((_proc.stdout or b'').decode('utf-8', 'replace').rstrip())
if _proc.returncode != 0:
    # _attempt > 1 说明是"自检不合格后自己 re-exec 出来的重试进程"，其 --seed 是内部注入的，
    # 不能当成"用户显式指定 seed，必须可复现、不许换种子"。
    if (_args.seed is None or _attempt > 1) and _attempt < _max_attempts:
        os.environ['GEN_ROUND_ATTEMPT'] = str(_attempt + 1)
        _next_seed = random.randrange(1, 10 ** 6)
        print('[layout] 第 %d 张布局不合格 → 换种子(%d)重新生成（第 %d/%d 次）'
              % (_attempt, _next_seed, _attempt + 1, _max_attempts))
        _argv = [sys.executable, _SELF_SCRIPT,
                 '--out', OUT_DIR, '--rovers', str(rover_num), '--seed', str(_next_seed)]
        if _args.base not in ('random', ''):
            _argv += ['--base', _args.base]
        # execv 会直接换掉进程镜像、不 flush 缓冲区，日志被重定向到文件时（块缓冲）
        # 上面所有输出都会丢，所以必须先 flush。
        sys.stdout.flush()
        sys.stderr.flush()
        os.execv(sys.executable, _argv)
    print('ERROR: 随机地图未通过布局就绪性检查（已试 %d 次）' % _attempt)
    sys.exit(2)
print('[layout] OK：随机地图可以拿去跑一轮')
