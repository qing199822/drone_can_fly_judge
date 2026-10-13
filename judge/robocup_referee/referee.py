#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""robocup_referee —— XTDrone/robocup 全链路集成裁判程序（单文件）

把开源仓库 XTDrone 在 robocup/ 目录下的四个部件集成成一个程序：

    ┌──────────────────────────────────────────────────────────────────────┐
    │ 本程序                          ← 对应 XTDrone/robocup 下的原件       │
    ├──────────────────────────────────────────────────────────────────────┤
    │ MapGenerator  (gen-map 子命令)   ← map_generator.py  (@5171c58)      │
    │   base 底图选择 / 13 栋建筑随机   ←   + base0..base3.world           │
    │   rover 随机 (20)                 ←   + rover_static.world           │
    │   black_box.txt / obstacle.txt    ←   + obstacle_list/change_list     │
    │   robocup.world 组装              ←   + house.world/rover.world       │
    │ HumanSpawner                      ← create_human_point()             │
    │   开局 6 个人物 init_pose          ←   （map_generator 内）            │
    │   运行期传送/重生落点              ←   （control_actor 内）            │
    │ ObstacleAvoid                     ← ObstacleAvoid.py                 │
    │ ActorBrain ×6  (run 子命令)       ← control_actor.py (@5171c58)      │
    │   巡逻 / 被追逃跑 / 防重叠避让      ←   + control_actors.sh(6 进程)     │
    │   传送 / cmd_motion                ←                                 │
    │ RefereeScorer  (run 子命令)       ← score_cal.py (@5171c58)          │
    │   发现/消除/跟踪/高度/超时/失联     ←                                 │
    └──────────────────────────────────────────────────────────────────────┘

与原版的差别（都是"集成"带来的，不改算法语义）：
  1. 原版要 1 个 map_generator.py + 6 个 control_actor.py 进程 + 1 个 score_cal.py
     一共 8 个进程；本程序是 1 个文件、2 个子命令（gen-map 在 gzserver 之前跑一次，
     run 在 gzserver 之后常驻）。
  2. 6 个 ActorBrain 共享同一份 /gazebo/model_states 缓存与 black_box/obstacle
     （原版每个进程各读一遍、各订阅一遍）。
  3. 原版 control_actor.py 依赖 /xtdrone/<type>_<i>/ground_truth/odom 判断"被追"；
     该话题在部分部署里不存在。本程序 --uav-odom=auto 会先订阅它，3 秒内收不到就
     自动降级为用 /gazebo/model_states 的位姿差分估算 UAV 速度。
  4. 若已有别的东西在发 /actor_<i>/cmd_vel（例如你自己的 tracking 节点），
     --tracker-yield（默认 2 秒）期间本程序不再对该人物发 cmd_motion，避免两边互抢；
     设 --tracker-yield 0 即为"裁判独占人物控制"。
  5. ~/.ros/log 只读的容器里 rospy.init_node 会直接崩，本程序在 init_node 之前
     自动把 ROS_LOG_DIR 换到可写目录（与 start_referee.sh 的做法一致）。

用法：
    # ① 生成这一局的世界（在起 gzserver 之前跑；产物直接喂给 gzserver 的 world 参数）
    python3 robocup_referee/referee.py gen-map --out /tmp/round01 --seed 1234
    #    可选 --base 1|2|3（默认跟上游一样随机 1..3）、--rovers 20、
    #         --src-dir <含 base*.world 与 rover_static.world 的目录>

    # ② 起完 gzserver + PX4 + MAVROS 之后，跑裁判（人物运动 + 评分）
    python3 robocup_referee/referee.py run \
        --black-box /tmp/round01/black_box.txt \
        --obstacle   /tmp/round01/obstacle.txt

    # 只读干跑（订阅/计算全做，但不发 /score、不发 cmd_motion、不删模型、不传送）
    python3 robocup_referee/referee.py run --dry-run --duration 30

    # 无 ROS 的纯逻辑自检（含真地图生成，写临时目录）
    python3 robocup_referee/referee.py selftest

忠实性说明 / 已知遗留（都能用开关切换）：
  * 上游 gen-map 有三个已知缺陷，本程序默认**保持上游行为**（--fix-known-bugs 才修）：
      (a) create_point() 连试 10 轮失败后走 "Base world is used!" 回退，而回退里
          content.readlines() 复用了已读完的句柄 → 产出 0 字节 world；
      (b) 13 栋建筑固定走 center_list[i] 的既有索引逻辑，实测房屋会大量重叠；
      (c) 只改模型级 <pose>、不同步底图 <state> 里的旧位姿。
    --fix-known-bugs 打开后：(a) 底图只读一次；(b) 按模型名精确定位每个模型的
    模型级 <pose> 并同步 <state>；(c) 每点最多 2000 个候选、取余量最大者，不再回退。
    这三条等价于你本地 9/19 对 XTDrone/robocup/map_generator.py 打过的那份补丁。
  * 评分公式两套：--score-mode repo（默认，= 上游 5171c58 原式）
    / extended（在上游基础上加"起飞飞机数×10 + 碰撞×30"，即你根目录 score_cal.py 的口径）。
  * 上游 control_actor.py 里的 get_moving/"shot" 分支从未被置 True，是死代码，
    本程序保留但默认关闭（--enable-shot-detection 打开）。

依赖：python3 + rospy + gazebo_msgs + ros_actor_cmd_pose_plugin_msgs
      （gen-map / selftest 只用标准库，不需要 ros master）
"""

import argparse
import atexit
import copy
import math
import os
import random
import shutil
import sys
import tempfile
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE_DIR = os.path.dirname(SCRIPT_DIR)

# ---------------------------------------------------------------------------
# 默认值（尽量与 XTDrone/robocup 各原件的硬编码值一致）
# ---------------------------------------------------------------------------
DEFAULTS = dict(
    # ---- 地图生成 ----
    base='random',          # 上游 map_num = random.randint(1, 3)
    rovers=20,              # 上游 rover_num = 20
    actor_num=6,            # 世界里的 6 个人物
    # ---- 人物（actor）----
    uav_type='typhoon_h480',
    drones=6,
    vehicle_type_aliases=('typhoon_h480',),
    actor_patrol_x=(-50.0, 130.0),      # 上游 x_min/x_max
    actor_patrol_y=(-60.0, 60.0),       # 上游 y_min/y_max
    actor_normal_speed=1.0,             # 上游 normal_speed
    actor_tracked_speed=2.0,            # 上游 tracked_speed
    actor_separation_speed=2.0,         # 上游 separation_speed
    actor_avoidance_radius=4.0,
    actor_min_distance=2.0,
    actor_spawn_distance=6.0,           # 运行期传送落点与别人的最小距离
    uav_safety_radius=7.0,
    uav_spawn_distance=8.0,
    actor_rate=10.0,                    # 上游 self.f = 10
    actor_black_box_inflate=5.0,        # 上游 self.spon_dis = 5
    teleport_interval=25.0,             # 上游 teleportation_interval = 25
    debug_teleport=False,               # 打印传送倒计时（排障用）
    tracker_yield=2.0,                  # 本程序新增：见文件头 ④
    uav_odom='auto',                    # auto|on|off
    enable_shot_detection=False,        # 上游死代码，默认关闭
    no_actor_control=False,             # 只评分、不驱动人物
    dump_score=True,                    # 把当前分写到 --score-output（上游没有这个功能）
    # ---- 评分 ----
    score_mode='repo',                  # repo|extended
    err_threshold=1.0,                  # 上游
    detection_interval=1.0,             # 上游
    detection_duration=15.0,            # 上游
    mission_timeout=600.0,              # 上游
    max_uav_altitude=6.0,               # 上游 MAX_UAV_ALTITUDE
    uav_miss_limit=3,                   # 上游 UAV_MISS_LIMIT
    uav_loss_penalty=100.0,             # 上游 DEFAULT_UAV_LOSS_PENALTY
    tracking_duration=20.0,             # 上游 TRACKING_DURATION
    tracking_distance=10.0,             # 上游 TRACKING_DISTANCE
    takeoff_altitude=0.5,               # extended 模式：z > 0.5 记一次起飞
    takeoff_score=10.0,
    collision_penalty=30.0,             # extended 模式
    collision_quota_per_uav=5,
    collision_low_v=0.1,
    collision_height_drop=0.5,
    collision_low_v_dwell=1.0,
    # ---- 传感器费用（上游 rosparam 口径）----
    mono_cam=1, stereo_cam=0, laser1d=0, laser2d=1,
    laser3d=0, gimbal=1, bino_cam=0,
)


class Config(object):
    """扁平配置对象：CLI / selftest 都能直接构造。"""

    def __init__(self, **kw):
        for key, val in DEFAULTS.items():
            setattr(self, key, val)
        for key, val in kw.items():
            if not hasattr(self, key) and key not in ('out', 'src_dir', 'seed',
                                                      'black_box', 'obstacle',
                                                      'dry_run', 'duration',
                                                      'world', 'fix_known_bugs',
                                                      'score_output', 'no_display'):
                raise KeyError('未知配置项: %s' % key)
            setattr(self, key, val)
        # 非 DEFAULTS 的键也允许存在
        self.out = getattr(self, 'out', None)
        self.src_dir = getattr(self, 'src_dir', None)
        self.seed = getattr(self, 'seed', None)
        self.black_box = getattr(self, 'black_box', None)
        self.obstacle = getattr(self, 'obstacle', None)
        self.dry_run = getattr(self, 'dry_run', False)
        self.duration = getattr(self, 'duration', 0.0)
        self.fix_known_bugs = getattr(self, 'fix_known_bugs', False)
        self.score_output = getattr(self, 'score_output', None)
        self.no_display = getattr(self, 'no_display', False)

    def sensor_cost(self):
        return (self.mono_cam * 5e2 + self.stereo_cam * 1e3 + self.laser1d * 2e2
                + self.laser2d * 1e3 + self.laser3d * 2e3 + self.gimbal * 2e2
                + self.bino_cam * 1e3)

    def __repr__(self):
        keys = sorted(DEFAULTS.keys())
        return 'Config(%s)' % ', '.join('%s=%r' % (k, getattr(self, k)) for k in keys)


# ===========================================================================
# 第 1 块：地图生成  —— 移植自 XTDrone/robocup/map_generator.py @ 5171c58
# ===========================================================================
class MapGenerator(object):
    """生成一局的世界文件。输入 base*.world / rover_static.world，输出：

        house.world     13 栋建筑随机后的底图
        rover.world     20 个 rover 随机后的模型
        black_box.txt   建筑包围盒（人物避障/落点判定用）
        robocup.world   最终世界（house + rover + 6 个人物 init_pose）
        obstacle.txt    栅格化障碍点（人物路径检查用；含 4 条边界）
        manifest.json   本次生成的指纹（seed/底图/sha256/人物落点）

    上游把产物写死到 ~/PX4_Firmware/Tools/sitl_gazebo/worlds/ 且从 CWD 读输入，
    本类改成 --out / --src-dir，算法与数值表保持原样。
    """

    # 上游 map_generator.py 的 building 顺序（13 个）
    NAME_LIST = ['house_1_146', 'house_3_156', 'house_3_157', 'house_3_158',
                 'gas_station_73', 'fast_food_93', 'house_1_66', 'house_1_67',
                 'house_1_146_clone', 'house_2_71', 'house_2_125',
                 'house_2_126', 'house_3_68']
    # 上游 obstacle_list() 里的 size_box（多一格 [2,2] 给 rover 用）
    SIZE_BOX = [[16, 12], [5, 17], [17, 5], [5, 17], [20, 15], [30, 25], [16, 12],
                [18, 18], [18, 18], [11, 9], [11, 9], [11, 9], [14, 5], [2, 2]]

    def __init__(self, cfg):
        self.cfg = cfg
        if cfg.seed is not None:
            random.seed(int(cfg.seed))
        self.map_num = self._pick_base()
        # create_point() 里原来用全局 map_num，这里改为实例属性，行为一致
        self.last_create_point = None

    # ---------------------------------------------------------------- 工具
    def _pick_base(self):
        base = self.cfg.base
        if base in (None, 'random', ''):
            return random.randint(1, 3)          # 上游：random.randint(1, 3)
        return int(base)

    def _candidates(self, name):
        """输入文件候选路径：--src-dir 优先，其次工作区里常见的位置。"""
        cands = [os.path.join(self.cfg.src_dir, name)]
        if name.startswith('base'):
            cands.append(os.path.join(WORKSPACE_DIR, 'waypoint', 'world', name))
        if name == 'rover_static.world':
            cands.append(os.path.join(WORKSPACE_DIR, 'tools', 'map_gen', 'assets', name))
            cands.append(os.path.join(WORKSPACE_DIR, 'XTDrone', 'robocup', name))
        return cands

    def _src(self, name):
        for path in self._candidates(name):
            if os.path.isfile(path):
                return path
        raise SystemExit(
            '[gen-map] 找不到输入文件 %s，已尝试：\n  %s\n'
            '         用 --src-dir 指定含 base*.world 与 rover_static.world 的目录'
            '（上游放在 XTDrone/robocup/）'
            % (name, '\n  '.join(self._candidates(name))))

    def _read(self, name):
        with open(self._src(name), 'r') as fh:
            return fh.read()

    def _write(self, name, text):
        path = os.path.join(self.cfg.out, name)
        with open(path, 'w') as fh:
            fh.write(text)
        return path

    # ------------------------------------------------- 上游 rand_x()
    def rand_x(self, num):
        correct = [[0, 0], [2, 2], [5, 5], [-15, 15], [15, -15], [0, 0], [2, 2],
                   [5, 0], [0, 0], [24, -15], [17.5, -15], [15, -15], [18, -15],
                   [3.7, 5], [5, 5], [17.5, -15], [15, -15]]
        map_num = self.map_num
        if 0.33 < num < 0.66:
            x = random.uniform(61.3 + correct[map_num][0], 95.5)
        elif num > 0.66:
            x = random.uniform(3.55, 30.8 - correct[map_num][1])
            if map_num in (7, 8):
                x = random.uniform(2.55, 27.8)
        else:
            x = random.uniform(-30.7, -29.7)
        return x

    # ------------------------------------------------- 上游 create_point()
    def create_point(self):
        """返回 (center_box, find_solution)。

        忠实移植：默认仍是"每个点最多 1000 次候选，10 轮失败就整体放弃"；
        --fix-known-bugs 时改为"每点最多 2000 个候选，取第一个满足判据的，
        都不满足就取余量最大的那个"，因此永远返回 13 个落点、不再有回退。
        """
        map_num = self.map_num
        fix = self.cfg.fix_known_bugs
        center_box = [[-27.5, 25.5], [-33.4, -21.4], [-28.2, -33], [-25.8, -19.4]]
        center_box_judge = [[86.6, -30], [-27, 14.65], [15.9, -13.6], [5.6, 35.7]]
        if map_num == 6:
            center_box = [[-28.64, 25.56], [-32.5, 1.8], [-28.2, -22.7], [-28.1, -33]]
            center_box_judge = [[86.6, -30], [-27, 14.65], [14.2, -20.6], [5.6, 35.7]]
        size_box_judge = [[16, 7], [1, 1], [2, 1], [2, 1], [20, 30], [30, 17],
                          [16, 12], [16, 12], [16, 16], [11, 9], [11, 9], [11, 9],
                          [17, 6]]
        in_obstacle = True
        count = 0
        find_solution = True
        candidate_limit = 2000 if fix else 1000
        for m in range(9):
            best = None
            best_margin = -1.0
            while in_obstacle and count < candidate_limit:
                # ---- 上游的候选生成规则（y 带 / rand_x 三段 / per-map 特判）----
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
                    x = int(self.rand_x(p))
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
                    x = int(self.rand_x(p))
                else:
                    y = int(random.choice((-1, 1)) * random.uniform(14, 30))
                    p = random.uniform(0.33, 1)
                    x = int(self.rand_x(p))
                    if map_num in (6, 15, 14, 11, 10, 7):
                        y = random.uniform(-30, 3)
                        if y >= -23:
                            y += 27
                    if map_num in (5, 13, 12, 9, 8, 16):
                        y = random.uniform(-30, 3)
                        if y >= -3:
                            y += 27
                # ---- 上游判据 ----
                box_len = len(center_box_judge)
                clearance = None
                for i in range(box_len):
                    dx = abs(x - center_box_judge[i][0]) - (
                        (size_box_judge[i][0] + size_box_judge[box_len][0]) / 2 + 3)
                    dy = abs(y - center_box_judge[i][1]) - (
                        (size_box_judge[i][1] + size_box_judge[box_len][1]) / 2 + 3)
                    if dx > 0 or dy > 0:
                        in_obstacle = False
                    else:
                        in_obstacle = True
                        count = count + 1
                        margin = max(dx, dy)          # 最接近"不重叠"的余量
                        if margin > best_margin:
                            best_margin = margin
                            best = (x, y)
                        break
                if fix and in_obstacle and count >= candidate_limit:
                    if best is not None:              # 取余量最大者，永不放弃
                        x, y = best
                        in_obstacle = False
            if not fix and count >= 1000:
                find_solution = False
            center_box_judge.append([x, y])
            center_box.append([x, y])
            in_obstacle = True
            # 上游的 count 是**跨 9 个点累积**的（从不按点清零），所以一旦累计到
            # 1000，后续点的 while 会立刻退出、直接复用上一轮的 x/y；忠实模式必须
            # 保持这个语义，否则回退概率与落点都会和上游不一样。
            # 只有 --fix-known-bugs 时才是"每点独立计数 + 取余量最大者"。
            if fix:
                count = 0
        self.last_create_point = (center_box, find_solution)
        return center_box, find_solution

    # --------------------------------------------- 上游 obstacle_list()
    def obstacle_list(self, center_list):
        black_box = [[[-35.375, -34.825], [-4, -3]], [[-3.275, -2.725], [-4, -3]],
                     [[20.725, 21.275], [-4, -3]], [[57.725, 58.275], [-4, -3]],
                     [[83.725, 84.275], [-4, -3]], [[-25.475, -24.925], [3, 4]],
                     [[8.725, 9.275], [3, 4]], [[26.725, 27.275], [3, 4]],
                     [[70.725, 71.275], [3, 4]], [[90.225, 90.775], [3, 4]],
                     [[79.4, 92.5], [-29.8, -28.6]], [[79, 85.2], [-33.7, -32.4]],
                     [[88, 94.2], [-33.7, -32.4]], [[2.52, 8.73], [34.5, 35.7]]]
        size_box = self.SIZE_BOX
        for i in range(len(center_list)):
            j = i if i <= len(size_box) - 1 else len(size_box) - 1
            x_min = center_list[i][0] - size_box[j][0] / 2.0
            x_max = center_list[i][0] + size_box[j][0] / 2.0
            y_min = center_list[i][1] - size_box[j][1] / 2.0
            y_max = center_list[i][1] + size_box[j][1] / 2.0
            black_box.append([[x_min, x_max], [y_min, y_max]])
        return black_box

    # ----------------------------------------------- 上游 change_list()
    @staticmethod
    def change_list(list_o, d):
        list_result = []
        k = 1.0 * d / 2.0
        for item in list_o:
            y = item[1][0] + k
            while (y + k) <= item[1][1]:
                x = item[0][0] + k
                list_result.append([x, y])
                while (x + k) <= item[0][1]:
                    x += d
                    list_result.append([x, y])
                y += d
        return list_result

    # --------------------------------- 上游 create_human_point()（开局落点）
    def create_human_point(self, black_box, occupied_points=None,
                           inflate=3.0, x_range=(-40.0, 110.0),
                           y_range=(-40.0, 40.0)):
        """人物开局 init_pose 落点：避开建筑(±inflate)、与已放置人物 ≥8m、
        与 6 个 UAV 起飞点 ≥8m（这三条是 5171c58 新增）。"""
        occupied_points = occupied_points or []
        count = 0
        while count < 100000:
            a = random.uniform(*x_range)
            b = random.uniform(*y_range)
            candidate_x = int(a)
            candidate_y = int(b)
            unsafe = False
            for box in black_box:
                xmin, xmax = box[0]
                ymin, ymax = box[1]
                if ((xmin - inflate) < candidate_x < (xmax + inflate)
                        and (ymin - inflate) < candidate_y < (ymax + inflate)):
                    unsafe = True
                    break
            if not unsafe:
                for actor_x, actor_y in occupied_points:
                    if math.hypot(candidate_x - actor_x,
                                  candidate_y - actor_y) < self.cfg.actor_spawn_distance:
                        unsafe = True
                        break
            if not unsafe:
                for uav_x, uav_y in self._uav_takeoff_points():
                    if math.hypot(candidate_x - uav_x,
                                  candidate_y - uav_y) < self.cfg.uav_spawn_distance:
                        unsafe = True
                        break
            if not unsafe:
                return candidate_x, candidate_y
            count += 1
        print('[gen-map] ERROR: 人物落点 100000 次都没找到（沿用上游行为）')
        return None

    @staticmethod
    def _uav_takeoff_points():
        # 上游 uav_takeoff_points（六机起飞点）
        return [(0.0, -3.0), (3.0, -3.0), (0.0, 0.0), (3.0, 0.0),
                (0.0, 3.0), (3.0, 3.0)]

    # ------------------------------------------------ 上游 rand_rover()
    @staticmethod
    def rand_rover(p):
        if p == 0:
            x = random.uniform(-48, 122)
            y = random.uniform(-48, -42)
        elif p == 1:
            # 上游写法 (random.uniform(-48, -20) or random.uniform(-11, 122))：
            # 前者几乎总是真值，因此这里等价于 random.uniform(-48, -20)
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

    # --------------------------------------- 上游 create_rover_point()
    def create_rover_point(self, rover_num):
        rover_size = 2
        center_box = []
        for _ in range(rover_num):
            in_obstacle = True
            attempts = 0
            cand = [0, 0]
            while in_obstacle:
                # [safety guard] 上游是 while True，理论上可能不收敛；这里加最大尝试次数
                attempts += 1
                p = math.floor(random.uniform(0, 8))
                cand = self.rand_rover(p)
                x, y = cand
                in_obstacle = False
                for i in range(len(center_box)):
                    if (abs(x - center_box[i][0]) > rover_size + 2
                            or abs(y - center_box[i][1]) > rover_size + 2):
                        in_obstacle = False
                    else:
                        in_obstacle = True
                        break
                if attempts > 20000:
                    print('[gen-map] WARN: rover 落点不收敛，沿用最后一个候选')
                    break
            center_box.append([cand[0], cand[1]])
        return center_box

    # ---------------------------------------------------- 主流程
    def generate(self):
        cfg = self.cfg
        if not cfg.out:
            raise SystemExit('[gen-map] 必须给 --out <输出目录>')
        os.makedirs(cfg.out, exist_ok=True)

        base_name = 'base%d.world' % self.map_num
        base_text = self._read(base_name)
        print('[gen-map] 底图 %s（上游 random.randint(1,3) 口径）' % base_name)

        house_text, used_fallback, centers = self._build_house_world(base_text)
        if used_fallback:
            print('[gen-map] Base world is used!  （上游回退路径）')

        rover_text = self._build_rover_world()
        center_list = list(centers)
        rover_centers = self._rover_centers
        center_list.extend(rover_centers)
        black_box = self.obstacle_list(center_list)
        with open(os.path.join(cfg.out, 'black_box.txt'), 'w') as fh:
            fh.write(str(black_box))
        print('[gen-map] black_box.txt 生成（%d 个包围盒）' % len(black_box))

        world_text, actor_points = self._assemble_world(house_text, rover_text,
                                                        black_box)
        self._write('house.world', house_text)
        self._write('rover.world', rover_text)
        self._write('robocup.world', world_text)
        self._write_obstacle_txt(black_box)
        self._write_manifest(base_name, base_text, actor_points, black_box,
                             used_fallback)
        print('[gen-map] 完成：%s' % cfg.out)
        for name in ('robocup.world', 'house.world', 'rover.world',
                     'black_box.txt', 'obstacle.txt'):
            path = os.path.join(cfg.out, name)
            print('           %-16s %10d B' % (name, os.path.getsize(path)))
        return dict(out=cfg.out, map_num=self.map_num,
                    actor_points=actor_points, black_box=black_box,
                    fallback=used_fallback)

    # ------------------------------- house.world（13 栋建筑随机）
    def _build_house_world(self, base_text):
        cfg = self.cfg
        fix = cfg.fix_known_bugs
        used_fallback = False
        if fix:
            # [fix-known-bugs a] 底图只读一次；上游回退分支会二次 readlines() 得到空表
            centers, find_solution = self.create_point()
            count_loop = 0
            while (not find_solution) and count_loop < 10:
                centers, find_solution = self.create_point()
                count_loop += 1
            if not find_solution:
                print('[gen-map] WARN: create_point 仍无解，沿用底图（但不会产出 0 字节）')
                return base_text, True, self.last_create_point[0]
            text = self._rewrite_model_poses(base_text, centers)
            text = self._sync_state_poses(text, centers)
            return text, False, centers

        # ---- 忠实移植上游逻辑（含 line_1 / count 的原有写法）----
        lines = base_text.splitlines(keepends=True)
        centers, find_solution = self.create_point()
        count_loop = 0
        while (not find_solution) and count_loop < 10:
            centers, find_solution = self.create_point()
            count_loop += 1
            print(' Try again!')
        out_lines = []
        if count_loop >= 10:
            # 上游：lines = content.readlines() 复用了已读完的句柄 → 这里等价于空表
            # （默认忠实复现上游的 0 字节产物；--fix-known-bugs 才换成底图本身）
            out_lines = []
            print(self.map_num)
            print('Base world is used!')
        else:
            count = 5
            change_flag = False
            line_1 = None                        # [safety guard] 上游此处可能未绑定
            for num, line in enumerate(lines):
                for i in range(len(self.NAME_LIST)):
                    if count == 1 or count == 4:
                        if line_1 is not None:
                            line = line_1
                    if (change_flag and "<pose frame=''>" in line
                            and (num == 0 or "<link name='link'>" not in lines[num - 1])):
                        pose_ori = line.split()
                        line = ("<pose frame=''>" + str(centers[i][0]) + ' '
                                + str(centers[i][1]) + ' '
                                + str(pose_ori[-4]) + ' ' + str(pose_ori[-3]) + ' '
                                + str(pose_ori[-2]) + ' ' + str(pose_ori[-1]) + '\n')
                        change_flag = False
                    if ("<model name='" + self.NAME_LIST[i] + "'>\n") in line:
                        if num + 1 < len(lines) and "<static>1</static>\n" in lines[num + 1]:
                            change_flag = True
                        else:
                            count = 0
                            pose_ori = lines[num + 1].split() if num + 1 < len(lines) else line.split()
                            line_1 = ("<pose frame=''>" + str(centers[i][0]) + ' '
                                      + str(centers[i][1]) + ' '
                                      + str(pose_ori[-4]) + ' ' + str(pose_ori[-3]) + ' '
                                      + str(pose_ori[-2]) + ' ' + str(pose_ori[-1]) + '\n')
                count = count + 1
                out_lines.append(line)
            used_fallback = False
        return ''.join(out_lines), used_fallback, centers

    # [fix-known-bugs b] 按模型名精确定位"模型级 <pose>"并改 x,y（跳过 <state> 区）
    def _rewrite_model_poses(self, text, centers):
        lines = text.splitlines(keepends=True)
        center_by_name = dict(zip(self.NAME_LIST, centers))
        ranges = {}
        current = None
        in_state = False
        for idx, line in enumerate(lines):
            stripped = line.strip()
            if stripped.startswith('<state '):
                in_state = True
                continue
            if stripped.startswith('</state>'):
                in_state = False
                continue
            if in_state:
                continue
            if stripped.startswith('<model name='):
                for name in self.NAME_LIST:
                    if ("<model name='%s'>" % name) in line:
                        current = name
                        ranges[name] = [idx, None]
                        break
            elif current is not None and stripped.startswith('</model>'):
                ranges[current][1] = idx
                current = None
        changed = 0
        for name, (start, end) in ranges.items():
            if end is None or name not in center_by_name:
                continue
            for idx in range(start + 1, end):
                stripped = lines[idx].lstrip()
                if stripped.startswith('<link'):
                    break                       # 已进入 link 内部，模型级 pose 不在后面
                if stripped.startswith('<pose'):
                    new = self._replace_xy(lines[idx], *center_by_name[name])
                    if new:
                        lines[idx] = new
                        changed += 1
                    break
        print('[gen-map] 模型级 <pose> 改写 %d 个（house 随机化补丁）' % changed)
        return ''.join(lines)

    # [fix-known-bugs c] 同步底图 <state> 里同名模型的位姿（否则旧快照会覆盖）
    def _sync_state_poses(self, text, centers):
        lines = text.splitlines(keepends=True)
        center_by_name = dict(zip(self.NAME_LIST, centers))
        in_state = False
        depth = 0
        current = None
        changed = 0
        for idx, line in enumerate(lines):
            stripped = line.lstrip()
            if stripped.startswith('<state '):
                in_state = True
                depth = 0
                continue
            if not in_state:
                continue
            if stripped.startswith('<model name='):
                for name in self.NAME_LIST:
                    if ("<model name='%s'>" % name) in line:
                        current = name
                        break
                continue
            if current is not None and stripped.startswith('<pose'):
                new = self._replace_xy(lines[idx], *center_by_name[current])
                if new:
                    lines[idx] = new
                    changed += 1
                current = None
                continue
            if stripped.startswith('</state>'):
                in_state = False
            if stripped.startswith('</model>'):
                current = None
            depth += 1
        if changed:
            print('[gen-map] <state> 位姿同步 %d 个' % changed)
        return ''.join(lines)

    @staticmethod
    def _replace_xy(pose_line, x, y):
        parts = pose_line.split()
        if len(parts) < 7:
            return None
        indent = pose_line[:len(pose_line) - len(pose_line.lstrip())]
        return '%s<pose frame=\'\'>%s %s %s %s %s %s\n' % (
            indent, x, y, parts[-4], parts[-3], parts[-2], parts[-1])

    # ------------------------------- rover.world（20 个 rover 随机）
    def _build_rover_world(self):
        """逐字移植上游 rover.world 的生成循环。

        上游那段的关键点（我第一版照"意图"补全过，是错的，这里按真实行为重写）：
          * 只有 `count == 1`（= 已见到第一个 model 名之后）时，才会把
            "紧跟着 </model> 的那一行 pose" 换成该 rover 的中心坐标；
          * 上游 `if count == 2 and num > 0:` 那一大段按 link 加偏移的代码
            **永远不会执行**（外层已被 count == 1 限死），等价于死代码；
          * 每个 `<model name='rover_static_clone_clone_10_clone_clone...'>`
            行都会被改名成 `..._<i+1>`。
        """
        rover_num = self.cfg.rovers
        centers = self.create_rover_point(rover_num)
        self._rover_centers = centers
        lines = self._read('rover_static.world').splitlines(keepends=True)
        out = []
        for i in range(rover_num):
            count = 0
            line_i = ("<model name='rover_static_clone_clone_10_clone_clone_"
                      + str(i + 1) + "'>\n")
            for num, line in enumerate(lines):
                if count == 1 and num + 1 < len(lines):
                    if "</model>" in lines[num + 1]:
                        pose_ori = line.split()
                        line = (" <pose frame=''>" + str(centers[i][0]) + ' '
                                + str(centers[i][1]) + ' '
                                + str(pose_ori[-4]) + ' ' + str(pose_ori[-3]) + ' '
                                + str(pose_ori[-2]) + ' ' + str(pose_ori[-1]) + '\n')
                if "<model name='rover_static_clone_clone_10_clone_clone" in line:
                    line = line_i
                    count = count + 1
                out.append(line)
        return ''.join(out)

    # --------------------------- robocup.world（拼装 + 6 个人物 init_pose）
    def _assemble_world(self, house_text, rover_text, black_box):
        lines = house_text.splitlines(keepends=True)
        out = []
        count = 2
        line_1 = None
        actor_points = []
        for num, line in enumerate(lines):
            if count == 1 and line_1 is not None:
                line = line_1
            if "filename='libros_actor_cmd_pose_plugin.so'>" in line:
                count = 0
                point = self.create_human_point(black_box, actor_points)
                if point is None:                # [safety guard]
                    point = (0, 0)
                a, b = point
                actor_points.append((a, b))
                line_1 = ("        <init_pose>" + str(a) + ' ' + str(b)
                          + " 1.25 1.57 0 0</init_pose>")
                print('[gen-map] ' + line_1.strip())
            count = count + 1
            out.append(line)
            if num + 1 < len(lines) and "</world>\n" in lines[num + 1]:
                out.append(rover_text)
        print('[gen-map] human initial pose generated!')
        return ''.join(out), actor_points

    # --------------------------------------- obstacle.txt（栅格 + 4 条边界）
    def _write_obstacle_txt(self, black_box):
        result_list = self.change_list(black_box, 1.0)
        path = os.path.join(self.cfg.out, 'obstacle.txt')
        with open(path, 'w') as f:
            for i in range(-50, 130):
                f.write(str(i + 0.5)); f.write(' '); f.write("50"); f.write(' '); f.write('\n')
            for i in range(-50, 130):
                f.write(str(i + 0.5)); f.write(' '); f.write("-50"); f.write(' '); f.write('\n')
            for i in range(-50, 50):
                f.write("-50"); f.write(' '); f.write(str(i + 0.5)); f.write(' '); f.write('\n')
            for i in range(-50, 50):
                f.write("130"); f.write(' '); f.write(str(i + 0.5)); f.write(' '); f.write('\n')
            for joints in result_list:
                for ji in range(2):
                    f.write(str(joints[ji])); f.write(' ')
                f.write('\n')
        print('[gen-map] obstacle.txt generated!（%d 个栅格点）' % len(result_list))

    def _write_manifest(self, base_name, base_text, actor_points, black_box,
                        used_fallback):
        import hashlib
        import json
        meta = dict(
            generated_at=time.strftime('%Y-%m-%d %H:%M:%S'),
            seed=self.cfg.seed,
            map_num=self.map_num,
            base_file=base_name,
            base_sha256=hashlib.sha256(base_text.encode('utf-8')).hexdigest(),
            rovers=self.cfg.rovers,
            actor_num=self.cfg.actor_num,
            actor_init_poses=[list(p) for p in actor_points],
            black_box_count=len(black_box),
            used_base_world_fallback=bool(used_fallback),
            fix_known_bugs=bool(self.cfg.fix_known_bugs),
        )
        path = os.path.join(self.cfg.out, 'manifest.json')
        with open(path, 'w') as fh:
            json.dump(meta, fh, indent=2, ensure_ascii=False)
        return meta


# ===========================================================================
# 第 2 块：人物避障工具 —— 移植自 XTDrone/robocup/ObstacleAvoid.py
# ===========================================================================
def xy(point):
    """把 geometry_msgs/Point、元组、带 x/y 的对象统一成 (x, y)。"""
    if point is None:
        return None
    if isinstance(point, (tuple, list)):
        return float(point[0]), float(point[1])
    return float(point.x), float(point.y)


class ObstacleAvoid(object):
    """把"当前点 → 目标点"的直线按障碍点切成若干子目标，绕开障碍。

    原版用 numpy.loadtxt('obstacle.txt') 从 CWD 读，并在 __init__ 里建 Point 列表；
    本实现改为直接传入 [(x, y), ...]，算法逐行等价（含 2*safeDis 聚类、左右侧判定、
    asin 外扩角等常数）。
    """

    def __init__(self, obstacle_points):
        self.obstList = [(float(p[0]), float(p[1])) for p in obstacle_points]
        self.obstInUAVCoorSys = []
        self.indexOfObstInSp = []
        self.indexOfFirstObst = 0
        self.useOriginalCurPos = False
        self.subTarg = None
        self.flag = True

    # ---- 上游 GetPointList()
    def GetPointList(self, cur_pos, target_pos, safe_dis):
        start = xy(cur_pos)
        target = xy(target_pos)
        sub_targ_list = []
        self.obstInUAVCoorSys = [(0.0, 0.0) for _ in self.obstList]
        self.flag = True
        self.useOriginalCurPos = False
        guard = 0
        while self.flag:
            # [safety guard] 上游是 while self.flag，极端几何下可能不收敛
            guard += 1
            if guard > 200:
                break
            self.subTarg = None
            if self.GetSubTarget(start, target, safe_dis):
                start = (self.subTarg[0], self.subTarg[1])
                sub_targ_list.append(start)
            else:
                self.flag = False
        sub_targ_list.append(target)
        return sub_targ_list

    # ---- 上游 GetSubTarget()
    def GetSubTarget(self, startPos, targetPos, safeDis):
        curposToTarget = self.VectDiff(targetPos, startPos)
        if curposToTarget[0] == 0.0 and curposToTarget[1] == 0.0:
            return False
        lengthCurToTarget = math.sqrt(curposToTarget[0] ** 2 + curposToTarget[1] ** 2)
        for k in range(len(self.obstList)):
            temp1 = self.VectDiff(self.obstList[k], startPos)
            self.obstInUAVCoorSys[k] = (
                self.VectDot(curposToTarget, temp1) / lengthCurToTarget,
                self.VectCross(curposToTarget, temp1) / lengthCurToTarget)
        if not self.useOriginalCurPos:
            self.indexOfObstInSp = []
            for j in range(len(self.obstInUAVCoorSys)):
                ox, oy = self.obstInUAVCoorSys[j]
                if 0 < ox < lengthCurToTarget and (abs(oy) + abs(ox) < safeDis):
                    self.indexOfObstInSp.append(j)
        if len(self.indexOfObstInSp) > 0:
            self.indexOfFirstObst = self.indexOfObstInSp[0]
            minDisInSpline = self.obstInUAVCoorSys[self.indexOfFirstObst][0]
            for i in range(len(self.indexOfObstInSp)):
                if minDisInSpline > self.obstInUAVCoorSys[self.indexOfObstInSp[i]][0]:
                    self.indexOfFirstObst = self.indexOfObstInSp[i]
                    minDisInSpline = self.obstInUAVCoorSys[self.indexOfObstInSp[i]][0]
            G = [self.indexOfFirstObst]
            numOfG = 1
            for i in range(120):
                if i < numOfG:
                    for j in range(len(self.obstList)):
                        if j == G[i]:
                            continue
                        elif self.VectNorm(self.obstList[j], self.obstList[G[i]]) <= 2 * safeDis:
                            if j not in G:
                                G.append(j)
                                numOfG = numOfG + 1
            maxPosEd = 0.0
            maxNegEd = 0.0
            for i in range(numOfG):
                oy = self.obstInUAVCoorSys[G[i]][1]
                if oy > 0:
                    maxPosEd = max(maxPosEd, oy)
                elif oy < 0:
                    maxNegEd = max(maxNegEd, abs(oy))
            sign_side = 1 if maxPosEd < maxNegEd else -1
            alpha = [0.0 for _ in range(numOfG)]
            maxAlpha = 0.0
            minAlpha = 10000.0
            maxAlphaIndex = 0
            minAlphaIndex = 0
            for i in range(numOfG):
                norm = self.VectNorm(self.obstList[G[i]], startPos)
                if norm <= 1e-9:
                    continue
                ratio = (safeDis + 0.1) / norm
                ratio = max(-1.0, min(1.0, ratio))
                alpha[i] = (math.atan2(self.obstInUAVCoorSys[G[i]][1],
                                       self.obstInUAVCoorSys[G[i]][0])
                            + sign_side * (math.asin(ratio) + 0.1))
                if alpha[i] > maxAlpha:
                    maxAlpha = alpha[i]
                    maxAlphaIndex = int(i)
                if alpha[i] < minAlpha:
                    minAlpha = alpha[i]
                    minAlphaIndex = int(i)
            if sign_side == 1:
                angle = maxAlpha
                obstIndex = maxAlphaIndex
            else:
                angle = minAlpha
                obstIndex = minAlphaIndex
            resu = self.VectNorm(self.obstList[G[obstIndex]], startPos) / lengthCurToTarget
            self.subTarg = (
                startPos[0] + (curposToTarget[0] * math.cos(angle)
                               - math.sin(angle) * curposToTarget[1]) * resu,
                startPos[1] + (curposToTarget[1] * math.cos(angle)
                               + math.sin(angle) * curposToTarget[0]) * resu)
            self.useOriginalCurPos = False
            return True
        return False

    @staticmethod
    def VectDiff(endPt, startPt):
        return (endPt[0] - startPt[0], endPt[1] - startPt[1])

    @staticmethod
    def VectDot(Pt1, Pt2):
        return Pt1[0] * Pt2[0] + Pt1[1] * Pt2[1]

    @staticmethod
    def VectCross(vect1, vect2):
        return vect1[0] * vect2[1] - vect1[1] * vect2[0]

    @staticmethod
    def VectNorm(Pt1, Pt2):
        return math.sqrt((Pt1[0] - Pt2[0]) ** 2 + (Pt1[1] - Pt2[1]) ** 2)


# ===========================================================================
# 第 3 块：世界状态（人物/UAV 位姿与速度）
# ===========================================================================
class WorldState(object):
    """共享一份 /gazebo/model_states 缓存（原版 6 个进程各订阅一遍）。

    UAV 速度来源（供"被追"判定）：
      --uav-odom on   只用 /xtdrone/<type>_<i>/ground_truth/odom 的 twist
      --uav-odom off  只用 model_states 位姿差分
      --uav-odom auto 优先 odom；某个 UAV 超过 3 秒没有 odom 就单独降级为位姿差分
    """

    ODOM_STALE = 3.0

    def __init__(self, cfg, clock=None):
        self.cfg = cfg
        self.clock = clock or time.time
        self.actor_pos = {}
        self.uav_pos = {}
        self.actor_yaw = {}
        self._last_pos = {}
        self._last_t = {}
        self._vel = {}
        self._odom = {}
        self._odom_t = {}

    # ---- /gazebo/model_states
    def on_model_states(self, msg):
        now = self.clock()
        actors = {}
        uavs = {}
        prefixes = tuple(self.cfg.vehicle_type_aliases) + (self.cfg.uav_type,)
        for idx, name in enumerate(msg.name):
            pose = msg.pose[idx].position
            if name.startswith('actor_'):
                actors[name] = (pose.x, pose.y, pose.z)
            elif name.startswith(prefixes):
                uavs[name] = (pose.x, pose.y, pose.z)
        for name, pos in actors.items():
            self.actor_pos[name] = pos
        for name, pos in uavs.items():
            self.uav_pos[name] = pos
        # 位姿差分算速度（只对 UAV）
        for name, pos in uavs.items():
            prev = self._last_pos.get(name)
            prev_t = self._last_t.get(name, now)
            dt = now - prev_t
            if prev is not None and dt > 1e-3:
                vx = (pos[0] - prev[0]) / dt
                vy = (pos[1] - prev[1]) / dt
                old = self._vel.get(name, (0.0, 0.0))
                # 指数平滑，避免单帧抖动误判"被追"
                self._vel[name] = (0.5 * old[0] + 0.5 * vx, 0.5 * old[1] + 0.5 * vy)
            self._last_pos[name] = pos
            self._last_t[name] = now

    # ---- /xtdrone/<type>_<i>/ground_truth/odom
    def on_odom(self, index, msg):
        self._odom[index] = (msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self._odom_t[index] = self.clock()

    def actor_xy(self, actor_id):
        pos = self.actor_pos.get('actor_%d' % actor_id)
        return None if pos is None else (pos[0], pos[1])

    def uav_xy(self, index):
        name = '%s_%d' % (self.cfg.uav_type, index)
        pos = self.uav_pos.get(name)
        return None if pos is None else (pos[0], pos[1])

    def uav_velocity(self, index):
        """返回 (vx, vy)。auto 模式下逐架 UAV 独立降级。"""
        mode = self.cfg.uav_odom
        if mode in ('on', 'auto'):
            if index in self._odom:
                fresh = (self.clock() - self._odom_t.get(index, 0.0)) <= self.ODOM_STALE
                if fresh or mode == 'on':
                    return self._odom[index]
        name = '%s_%d' % (self.cfg.uav_type, index)
        return self._vel.get(name, (0.0, 0.0))

    def uav_positions(self):
        out = {}
        for index in range(self.cfg.drones):
            p = self.uav_xy(index)
            if p is not None:
                out['%s_%d' % (self.cfg.uav_type, index)] = p
        return out

    def actor_positions(self, exclude_id=None):
        out = {}
        for name, pos in self.actor_pos.items():
            if exclude_id is not None and name == 'actor_%d' % exclude_id:
                continue
            out[name] = (pos[0], pos[1])
        return out


# ===========================================================================
# 第 4 块：人物落点（运行期传送/重生）—— 移植自 control_actor.create_human_point()
# ===========================================================================
class HumanSpawner(object):
    """运行期落点。与开局落点(gen-map)的区别：这里的建筑外扩用 control_actor 的
    spon_dis=5，并且落点范围是 (-40..110, -40..40)，与上游一致。"""

    def __init__(self, cfg, black_box, world):
        self.cfg = cfg
        self.black_box = black_box
        self.world = world

    def point_for(self, actor_id):
        cfg = self.cfg
        count = 0
        while count < 100000:
            candidate_x = int(random.uniform(-40, 110))
            candidate_y = int(random.uniform(-40, 40))
            unsafe = False
            for box in self.black_box:
                xmin, xmax = box[0]
                ymin, ymax = box[1]
                if ((xmin - cfg.actor_black_box_inflate) < candidate_x
                        < (xmax + cfg.actor_black_box_inflate)
                        and (ymin - cfg.actor_black_box_inflate) < candidate_y
                        < (ymax + cfg.actor_black_box_inflate)):
                    unsafe = True
                    break
            if not unsafe:
                for name, pos in self.world.actor_positions(exclude_id=actor_id).items():
                    if math.hypot(candidate_x - pos[0], candidate_y - pos[1]) < cfg.actor_spawn_distance:
                        unsafe = True
                        break
            if not unsafe:
                uav_points = list(MapGenerator._uav_takeoff_points())
                uav_points.extend(self.world.uav_positions().values())
                for uav_x, uav_y in uav_points:
                    if math.hypot(candidate_x - uav_x, candidate_y - uav_y) < cfg.uav_spawn_distance:
                        unsafe = True
                        break
            if not unsafe:
                return candidate_x, candidate_y
            count += 1
        print('[actor] WARN: actor_%d 落点 100000 次都没找到' % actor_id)
        return None


# ===========================================================================
# 第 5 块：人物运动大脑 —— 移植自 XTDrone/robocup/control_actor.py @ 5171c58
# ===========================================================================
class ActorBrain(object):
    """一个人物的行为：巡逻 → 遇障碍绕行 → 被 UAV 追上就逃 → 与其它人物/UAV 防重叠。

    与上游 control_actor.py 的差异（都是"6 进程合 1 进程"必须做的）：
      1. 上游每个进程自己订阅 /gazebo/model_states 与 6 路 odom；这里统一用共享
         WorldState。
      2. 上游在 /find_actor_i 回调里 **忙等** 25 秒（会把整个进程卡死）；这里改为
         在 tick 里判到期后传送一次，语义相同（"被发现后 25 秒换个落点"）但不阻塞。
      3. 上游用循环内局部变量 middd_pos 存子目标；这里显式用 subtarget_pos + 下标，
         行为等价（已逐行核对）。
    """

    def __init__(self, actor_id, cfg, world, spawner, obstacle_avoid,
                 black_box, publish_cmd=None, teleport_cmd=None, clock=None):
        self.id = actor_id
        self.cfg = cfg
        self.world = world
        self.spawner = spawner
        self.avoid_tool = obstacle_avoid
        self.black_box = black_box
        self.publish_cmd = publish_cmd
        self.teleport_cmd = teleport_cmd
        self.clock = clock or time.time
        self.active = True
        self.reset()

    def reset(self):
        cfg = self.cfg
        self.current_pose = (0.0, 0.0)
        self.last_pose = (0.0, 0.0)
        self.patrol_target = (0.0, 0.0)
        self.avoid = (0.0, 0.0)
        self.subtarget_pos = []
        self.subtarget_index = 0
        self.arrive_count = 0
        self.avoid_finish_flag = True
        self.distance_flag = True
        self.suitable_point = True
        self.escape_suce_flag = False
        self.get_moving = False
        self.shooting_count = 0
        self.f = cfg.actor_rate
        self.spon_dis = cfg.actor_black_box_inflate
        self.x_min, self.x_max = cfg.actor_patrol_x
        self.y_min, self.y_max = cfg.actor_patrol_y
        self.uav_num = cfg.drones
        self.actor_num = cfg.actor_num
        self.normal_speed = cfg.actor_normal_speed
        self.tracked_speed = cfg.actor_tracked_speed
        self.separation_speed = cfg.actor_separation_speed
        self.actor_avoidance_radius = cfg.actor_avoidance_radius
        self.actor_min_distance = cfg.actor_min_distance
        self.uav_safety_radius = cfg.uav_safety_radius
        self.dis_actor_uav = [0.0] * self.uav_num
        self.tracking_flag = [0] * self.uav_num
        self.catching_flag = 0
        self.catching_uav_num = 10
        self.teleport_request_time = None
        self.teleport_done = False
        self.show_log = False

    # ------------------------------------------------------------- 工具
    @staticmethod
    def pos2ang(deltax, deltay):
        if deltax != 0:
            angle = math.atan2(deltay, deltax)
            if deltay > 0 and angle < 0:
                angle = angle + math.pi
            elif deltay < 0 and angle > 0:
                angle = angle - math.pi
            elif deltay == 0:
                angle = 0.0 if deltax > 0 else math.pi
        else:
            if deltay > 0:
                angle = math.pi / 2
            elif deltay < 0:
                angle = -math.pi / 2
            else:
                angle = 0.0
        if angle < 0:
            angle = angle + 2 * math.pi
        return angle

    def _point_safe(self, x, y):
        for box in self.black_box:
            if (box[0][0] - self.spon_dis) < x < (box[0][1] + self.spon_dis):
                if (box[1][0] - self.spon_dis) < y < (box[1][1] + self.spon_dis):
                    return False
        return True

    def _path_safe(self, target, collision_threshold=1.5):
        """上游：在当前点到目标点之间采 101 个点，任何一点离 obstacle.txt 里的
        栅格点 < 1.5m 就重新选点。"""
        if not self.avoid_tool.obstList:
            return True
        for t in range(101):
            alpha = t / 100.0
            x = (1 - alpha) * self.current_pose[0] + alpha * target[0]
            y = (1 - alpha) * self.current_pose[1] + alpha * target[1]
            for ox, oy in self.avoid_tool.obstList:
                if math.hypot(x - ox, y - oy) < collision_threshold:
                    return False
        return True

    # ------------------------------------------- 巡逻目标（上游 loop 内那段）
    def choose_patrol_target(self):
        attempts = 0
        while True:
            attempts += 1
            if attempts > 20000:                 # [safety guard] 上游是 while True
                print('[actor_%d] WARN: 找不到可用巡逻点，退回场地中心' % self.id)
                self.patrol_target = (0.0, 0.0)
                break
            shres = 2
            x = random.uniform(self.x_min - shres, self.x_max + shres)
            y = random.uniform(self.y_min - shres, self.y_max + shres)
            if not self._point_safe(x, y):
                continue
            if not self._path_safe((x, y)):
                continue
            self.patrol_target = (x, y)
            break
        # 上游越界回拉
        tx, ty = self.patrol_target
        if tx < self.x_min:
            tx = self.x_min + 5
        elif tx > self.x_max:
            tx = self.x_max - 5
        if ty < self.y_min:
            ty = self.y_min + 5
        elif ty > self.y_max:
            ty = self.y_max - 5
        self.patrol_target = (tx, ty)
        try:
            self.subtarget_pos = self.avoid_tool.GetPointList(
                self.current_pose, self.patrol_target, 0.5)
        except Exception as exc:                 # 上游 except 分支的等价物
            self.subtarget_pos = self._edge_escape_list()
            if self.subtarget_pos is None:
                self.subtarget_pos = [self.patrol_target]
            print('[actor_%d] GetPointList 失败，改用就近脱离点：%s' % (self.id, exc))
        self.subtarget_index = 0
        self.avoid = self.subtarget_pos[0]
        self.avoid_finish_flag = False
        self.distance_flag = False
        self.suitable_point = True
        self.escape_suce_flag = False

    def _edge_escape_list(self):
        """上游 except 分支：被压在建筑里时，朝最近的建筑边线外侧挪 1m。"""
        dis_list = [0.0, 0.0, 0.0, 0.0]
        cx, cy = self.current_pose
        for box in self.black_box:
            if (box[0][0] - self.spon_dis) < cx < (box[0][1] + 1.0):
                if (box[1][0] - 1.0) < cy < (box[1][1] + 1.0):
                    dis_list[0] = abs(cx - (box[0][0] - 1.0))
                    dis_list[1] = abs(cx - (box[0][1] + 1.0))
                    dis_list[2] = abs(cy - (box[1][0] - 1.0))
                    dis_list[3] = abs(cy - (box[1][1] + 1.0))
                    dis_min = dis_list.index(min(dis_list))
                    if dis_min == 0:
                        return [(box[0][0] - 1.0, cy)]
                    elif dis_min == 1:
                        return [(box[0][1] + 1.0, cy)]
                    elif dis_min == 2:
                        return [(cx, box[1][0] - 1.0)]
                    else:
                        return [(cx, box[1][1] + 1.0)]
        return None

    # ------------------------------------------- 被追判定（上一轮修复后的版本）
    def update_tracking_state(self):
        for i in range(self.uav_num):
            vx, vy = self.world.uav_velocity(i)
            uav_speed_squared = vx ** 2 + vy ** 2
            uav_xy = self.world.uav_xy(i)
            if uav_xy is None:
                self.tracking_flag[i] = 0
                continue
            self.dis_actor_uav[i] = math.hypot(self.current_pose[0] - uav_xy[0],
                                               self.current_pose[1] - uav_xy[1])
            if (self.catching_flag == 0 and uav_speed_squared > 1.0
                    and self.dis_actor_uav[i] < 20.0):
                self.tracking_flag[i] += 1
                if self.tracking_flag[i] > 20:           # 2 秒（10 Hz）
                    self.catching_flag = 1
                    self.tracking_flag[i] = 0
                    self.catching_uav_num = i
                    print('[actor_%d] catch (UAV %d)' % (self.id, i))
                    break
            else:
                self.tracking_flag[i] = 0
        # 只有触发追踪的那架 UAV 能解除
        if self.catching_flag != 0 and self.catching_uav_num < self.uav_num:
            uav_xy = self.world.uav_xy(self.catching_uav_num)
            if uav_xy is None:
                return
            catching_distance = math.hypot(self.current_pose[0] - uav_xy[0],
                                           self.current_pose[1] - uav_xy[1])
            if catching_distance >= 20.0:
                self.catching_flag = 0
                self.catching_uav_num = 10

    # ------------------------------------------------- 逃跑目标（上游那段几何）
    def escape_target(self):
        i = self.catching_uav_num
        uav_xy = self.world.uav_xy(i)
        vx, vy = self.world.uav_velocity(i)
        if uav_xy is None:
            return (self.x_max, self.current_pose[1])
        angle = self.pos2ang(vx, vy)
        tar_angle = angle - math.pi / 2
        flag_k = 0
        k = 0.0
        if tar_angle == math.pi / 2:
            flag_k = 1
        elif tar_angle in (-math.pi / 2, 3 * math.pi / 2):
            flag_k = 2
        else:
            k = math.tan(tar_angle)
        tx, ty = self.patrol_target
        if tar_angle < 0 and flag_k == 0:
            y = k * (self.x_max - uav_xy[0]) + uav_xy[1]
            if y < self.y_min:
                ty, tx = self.y_min, (self.y_min - uav_xy[1]) / k + uav_xy[0]
            else:
                ty, tx = y, self.x_max
        elif 0 < tar_angle < math.pi / 2 and flag_k == 0:
            y = k * (self.x_max - uav_xy[0]) + uav_xy[1]
            if y > self.y_max:
                ty, tx = self.y_max, (self.y_max - uav_xy[1]) / k + uav_xy[0]
            else:
                ty, tx = y, self.x_max
        elif math.pi / 2 < tar_angle < math.pi and flag_k == 0:
            y = k * (self.x_min - uav_xy[0]) + uav_xy[1]
            if y > self.y_max:
                ty, tx = self.y_max, (self.y_max - uav_xy[1]) / k + uav_xy[0]
            else:
                ty, tx = y, self.x_min
        elif math.pi < tar_angle < 3 * math.pi / 2 and flag_k == 0:
            y = k * (self.x_min - uav_xy[0]) + uav_xy[1]
            if y < self.y_min:
                ty, tx = self.y_min, (self.y_min - uav_xy[1]) / k + uav_xy[0]
            else:
                ty, tx = y, self.x_min
        elif flag_k == 1:
            tx, ty = uav_xy[0], self.y_max
        elif flag_k == 2:
            tx, ty = uav_xy[0], self.y_min
        tx = max(self.x_min, min(self.x_max, tx))
        ty = max(self.y_min, min(self.y_max, ty))
        return (tx, ty)

    # --------------------------------- 防重叠转向（上游 build_motion_command()）
    def build_motion_command(self):
        command = (self.avoid[0], self.avoid[1], self.avoid_v())
        desired_x = command[0] - self.current_pose[0]
        desired_y = command[1] - self.current_pose[1]
        desired_length = math.sqrt(desired_x ** 2 + desired_y ** 2)
        if desired_length > 1e-6:
            steer_x, steer_y = desired_x / desired_length, desired_y / desired_length
        else:
            steer_x, steer_y = 0.0, 0.0
        separation_x = separation_y = 0.0
        nearby_actor = False
        nearby_uav = False
        emergency_separation = False
        for _, pos in self.world.actor_positions(exclude_id=self.id).items():
            away_x = self.current_pose[0] - pos[0]
            away_y = self.current_pose[1] - pos[1]
            distance = math.sqrt(away_x ** 2 + away_y ** 2)
            if distance >= self.actor_avoidance_radius:
                continue
            nearby_actor = True
            if distance < 1e-6:
                angle = (int(self.id) + 1) * 2.0 * math.pi / (self.actor_num + 1)
                unit_x, unit_y = math.cos(angle), math.sin(angle)
            else:
                unit_x, unit_y = away_x / distance, away_y / distance
            strength = 2.0 * (self.actor_avoidance_radius - distance) / self.actor_avoidance_radius
            steer_x += unit_x * strength
            steer_y += unit_y * strength
            separation_x += unit_x
            separation_y += unit_y
            if distance < self.actor_min_distance:
                emergency_separation = True
        for _, pos in self.world.uav_positions().items():
            away_x = self.current_pose[0] - pos[0]
            away_y = self.current_pose[1] - pos[1]
            distance = math.sqrt(away_x ** 2 + away_y ** 2)
            if distance >= self.uav_safety_radius:
                continue
            nearby_actor = True
            nearby_uav = True
            if distance < 1e-6:
                angle = (int(self.id) + 1) * 2.0 * math.pi / (self.actor_num + 1)
                unit_x, unit_y = math.cos(angle), math.sin(angle)
            else:
                unit_x, unit_y = away_x / distance, away_y / distance
            strength = 3.0 * (self.uav_safety_radius - distance) / self.uav_safety_radius
            steer_x += unit_x * strength
            steer_y += unit_y * strength
            separation_x += unit_x * 2.0
            separation_y += unit_y * 2.0
            if distance < self.actor_min_distance:
                emergency_separation = True
        if not nearby_actor:
            return command
        speed = command[2]
        if emergency_separation:
            steer_x, steer_y = separation_x, separation_y
            speed = max(speed, self.separation_speed)
        steer_length = math.sqrt(steer_x ** 2 + steer_y ** 2)
        if steer_length < 1e-6:
            angle = (int(self.id) + 1) * 2.0 * math.pi / (self.actor_num + 1)
            steer_x, steer_y = math.cos(angle), math.sin(angle)
            steer_length = 1.0
        target_distance = self.uav_safety_radius if nearby_uav else self.actor_avoidance_radius
        cx = self.current_pose[0] + target_distance * steer_x / steer_length
        cy = self.current_pose[1] + target_distance * steer_y / steer_length
        cx = max(self.x_min, min(self.x_max, cx))
        cy = max(self.y_min, min(self.y_max, cy))
        return (cx, cy, speed)

    def avoid_v(self):
        if self.catching_flag in (1, 2):
            return self.tracked_speed
        return self.normal_speed

    # ------------------------------------------------------ 传送（/find_actor_i）
    def on_found(self, reference_time):
        """记下"被找到"的基准时刻，teleport_interval 秒后换落点。

        注意：上游用 /find_actor_i（**Float32**）传 rospy.get_time()，而 1.79e9 量级
        的时间戳在 float32 下会被量化到 128 秒一档（尾数只有 24 位），于是上游
        control_actor.py 的"25 秒后传送"实际会随机偏移 ±128 秒；偏到未来时那个
        actor 进程会一直忙等（整个进程卡死）。
        所以这里**不使用消息里的值**，改由调用方传"收到消息的本地时刻"作基准。
        """
        self.teleport_request_time = reference_time
        self.teleport_done = False
        print('[actor_%d] 被发现在 t=%.1f，%.0fs 后传送'
              % (self.id, reference_time, self.cfg.teleport_interval))

    def _maybe_teleport(self, now):
        if self.teleport_request_time is None or self.teleport_done:
            return
        if now - self.teleport_request_time < self.cfg.teleport_interval:
            if self.cfg.debug_teleport and now >= getattr(self, '_tp_next_log', 0.0):
                self._tp_next_log = now + 1.0
                print('[actor_%d] 传送倒计时 %.1fs (now=%.1f req=%.1f)'
                      % (self.id, self.cfg.teleport_interval
                         - (now - self.teleport_request_time), now,
                         self.teleport_request_time))
            return
        self.teleport_done = True
        if self.teleport_cmd is None:            # dry-run
            print('[actor_%d] (dry-run) 该传送了，跳过' % self.id)
            return
        point = self.spawner.point_for(self.id)
        if point is None:
            return
        ok = self.teleport_cmd(self.id, point[0], point[1])
        print('[actor_%d] 传送 -> (%d, %d) ok=%s' % (self.id, point[0], point[1], ok))

    # ------------------------------------------------------------- 主 tick
    def tick(self, now):
        """返回要发布的 (x, y, v)；不发布时返回 None。"""
        if not self.active:
            return None
        pos = self.world.actor_xy(self.id)
        if pos is not None:
            self.last_pose = self.current_pose
            self.current_pose = pos
        # ---- 传送 ----
        self._maybe_teleport(now)
        # ---- 选新巡逻目标（上游：avoid_finish_flag and distance_flag）----
        if self.avoid_finish_flag and self.distance_flag:
            self.choose_patrol_target()
        # ---- 到点判定 ----
        distance = ((self.current_pose[0] - self.avoid[0]) ** 2
                    + (self.current_pose[1] - self.avoid[1]) ** 2)
        if distance < 0.01:
            self.arrive_count += 1
            if self.arrive_count > 5:
                self.distance_flag = True
                self.arrive_count = 0
                if self.catching_flag == 2:
                    self.catching_flag = 0
            else:
                self.distance_flag = False
        else:
            self.arrive_count = 0
            self.distance_flag = False
        # ---- 被追判定 ----
        self.update_tracking_state()
        # ---- 逃跑 ----
        if self.catching_flag == 1:
            target = self.escape_target()
            try:
                self.subtarget_pos = self.avoid_tool.GetPointList(self.current_pose, target, 1)
                self.avoid = self.subtarget_pos[0]
                self.subtarget_index = 0
                self.catching_flag = 2
            except Exception:
                self.avoid = target
                self.catching_flag = 1
        # ---- 子目标推进 ----
        if not self.avoid_finish_flag:
            if self.distance_flag:
                self.subtarget_index += 1
                self.distance_flag = False
                if self.subtarget_index >= len(self.subtarget_pos):
                    self.avoid_finish_flag = True
                    self.subtarget_index = 0
                else:
                    self.avoid = self.subtarget_pos[self.subtarget_index]
        # ---- 上游的 "被卡住=被击中" 死代码（默认关闭）----
        if self.cfg.enable_shot_detection and self.get_moving:
            distance_change = ((self.last_pose[0] - self.current_pose[0]) ** 2
                               + (self.last_pose[1] - self.current_pose[1]) ** 2)
            if distance_change < 0.00001:
                self.shooting_count += 1
                if self.shooting_count > 1000:
                    print('[actor_%d] shot' % self.id)
                    self.distance_flag = False
                    self.suitable_point = True
                    self.avoid_finish_flag = True
                    self.shooting_count = 0
            else:
                self.shooting_count = 0
        return self.build_motion_command()


# ===========================================================================
# 第 6 块：裁判评分 —— 移植自 XTDrone/robocup/score_cal.py @ 5171c58
# ===========================================================================
class RefereeScorer(object):
    """计分与规则。与 ROS 解耦：所有外部动作都通过构造参数注入的回调完成，
    因此 selftest 能用假时钟假服务把整套状态机跑一遍。

    评分口径（2026-09-28 用 selftest_scorer_equiv.py 与上游本体逐步比对过）：
      --score-mode repo     = 上游 score_cal.py @5171c58 原式：
          progress_score()：发现×50 + 跟踪×80 + 消除×100 − 传感器×0.003 − 失联×100
                            （该函数里**没有**用时项）
          六个人物全被消除时：inline 用 2580 − 用时 − 传感器×0.003 − 失联×100
      --score-mode extended = 在上式基础上再加 起飞飞机数×10 − 碰撞次数×30，
          且"完成"走 progress_score 里的 2580 − 用时 分支
          （= 你根目录 score_cal.py 现在的口径，对齐 2026 官方规则文字）
    """

    def __init__(self, cfg, publish_score=None, publish_time=None, publish_left=None,
                 find_publishers=None, delete_model=None, get_model_state=None,
                 clock=None, wall_clock=None, dump_path=None):
        self.cfg = cfg
        self.publish_score_cb = publish_score or (lambda value: None)
        self.publish_time_cb = publish_time or (lambda value: None)
        self.publish_left_cb = publish_left or (lambda text: None)
        self.find_publishers = find_publishers or []
        self.delete_model_cb = delete_model
        self.get_model_state_cb = get_model_state
        self.clock = clock or time.time
        self.wall_clock = wall_clock or time.time
        self.dump_path = dump_path
        self.reset()

    # ---------------------------------------------------------------- 状态
    def reset(self):
        cfg = self.cfg
        n_a, n_u = cfg.actor_num, cfg.drones
        self.left_actors = list(range(n_a))
        self.find_actors = list(range(n_a))
        self.actors_pos = [None] * n_a
        self.count_flag = [False] * n_a
        self.topic_arrive_time = [0.0] * n_a
        self.find_time = [0.0] * n_a
        self.tracking_start_time = [None] * n_a
        self.tracked_actors = set()
        self.uav_positions = [None] * n_u
        self.uav_seen = [False] * n_u
        self.uav_lost = [False] * n_u
        self.uav_miss_count = [0] * n_u
        self.uav_took_off = [False] * n_u
        self.collision_low_v_since = [None] * n_u
        self.collision_last_height = [None] * n_u
        self.collision_count_per_uav = [0] * n_u
        self.mission_finished = False
        self.mission_completed = False
        self.uav_loss_count = 0
        self.collision_count = 0
        self.target_finish = 0
        self.find_finish = 0
        self.score = 0.0
        self.time_usage = 0.0
        self.start_time = self.clock()
        self.takeoff_count = 0
        self.sensor_cost = self.cfg.sensor_cost()

    def start(self, sim_time=None):
        self.start_time = self.clock() if sim_time is None else sim_time

    # ---------------------------------------------------------------- 计分
    def progress_score(self):
        """repo 模式 = 上游 score_cal.py 的 _progress_score() 原式。

        注意（2026-09-28 用 selftest_scorer_equiv.py 与上游本体逐步比对后修正）：
          * 上游这个函数里**没有**"完成时 2580 − 用时"分支——那个 2580 是上游在
            "6 个人物全被消除"那一处 **inline** 算的（见 process_actor_detection）；
          * 上游也**没有** _dump_score_file/atexit（分数落盘是本程序的附加功能）。
        extended 模式才保留"完成 → 2580 − 用时"分支（那是你根目录 score_cal.py 的口径）。
        """
        cfg = self.cfg
        track_finish = len(self.tracked_actors)
        extended = (cfg.score_mode == 'extended')
        if extended and self.mission_completed:
            elapsed = max(0.0, self.clock() - self.start_time)
            return (2580.0 - elapsed - self.sensor_cost * 3e-3
                    - self.collision_count * cfg.collision_penalty
                    - self.uav_loss_count * cfg.uav_loss_penalty)
        score = (self.find_finish * 50
                 + track_finish * 80
                 + self.target_finish * 100
                 - self.sensor_cost * 3e-3
                 - self.uav_loss_count * cfg.uav_loss_penalty)
        if extended:
            score += sum(1 for x in self.uav_took_off if x) * cfg.takeoff_score
            score -= self.collision_count * cfg.collision_penalty
        return score

    def publish_score(self):
        value = int(round(max(-32768, min(32767, self.score))))
        self.publish_score_cb(value)
        if self.cfg.dump_score:
            self.dump_score_file(value)

    def dump_score_file(self, final_value=None):
        if not self.dump_path:
            return
        try:
            value = (final_value if final_value is not None
                     else int(round(max(-32768, min(32767, self.score)))))
        except Exception:
            return
        try:
            with open(self.dump_path, 'w') as fh:
                fh.write(str(value) + '\n')
        except Exception as exc:
            print('[score] 落盘失败: %s' % exc)

    def finish(self, reason, final_score=None):
        if self.mission_finished:
            return True
        self.mission_finished = True
        if final_score is not None:
            self.score = final_score
        print('[score] %s' % reason)
        print('[score] score: %s' % self.score)
        self.publish_score()
        return True

    # ------------------------------------------------------------ 发现/消除
    def reset_detection(self, actor_id):
        self.count_flag[actor_id] = False
        self.find_time[actor_id] = 0.0
        self.topic_arrive_time[actor_id] = 0.0

    def delete_actor(self, actor_id):
        name = 'actor_%d' % actor_id
        if self.delete_model_cb is not None:
            try:
                ok = self.delete_model_cb(name)
            except Exception as exc:
                print('[score] 删除 %s 失败: %s' % (name, exc))
                return False
            if not ok:
                print('[score] 删除 %s 失败' % name)
                return False
        if actor_id in self.left_actors:
            self.left_actors.remove(actor_id)
        self.target_finish = self.cfg.actor_num - len(self.left_actors)
        self.score = self.progress_score()
        return True

    def process_actor_detection(self, msg, actor_ids):
        if self.mission_finished or not getattr(msg, 'cls', None):
            return
        now = self.clock()
        cfg = self.cfg
        for actor_id in actor_ids:
            if actor_id not in self.left_actors:
                continue
            position = self.actors_pos[actor_id]
            previous = self.topic_arrive_time[actor_id]
            self.topic_arrive_time[actor_id] = now
            if position is None:
                self.reset_detection(actor_id)
                continue
            distance_sq = (msg.x - position.x) ** 2 + (msg.y - position.y) ** 2
            continuous = previous == 0.0 or now - previous <= cfg.detection_interval
            if distance_sq >= cfg.err_threshold ** 2 or not continuous:
                self.reset_detection(actor_id)
                continue
            if not self.count_flag[actor_id]:
                self.count_flag[actor_id] = True
                self.find_time[actor_id] = now
                if actor_id in self.find_actors:
                    self.find_actors.remove(actor_id)
                self.find_finish = cfg.actor_num - len(self.find_actors)
                if actor_id < len(self.find_publishers):
                    self.find_publishers[actor_id](now)
                self.score = self.progress_score()
                print('[score] find actor_%d' % actor_id)
                self.publish_score()
                continue
            if now - self.find_time[actor_id] < cfg.detection_duration:
                continue
            if not self.delete_actor(actor_id):
                self.reset_detection(actor_id)
                continue
            self.reset_detection(actor_id)
            print('[score] actor_%d is OK' % actor_id)
            print('[score] Time usage: %s' % self.time_usage)
            if self.target_finish == cfg.actor_num:
                self.mission_completed = True
                if self.cfg.score_mode == 'extended':
                    self.score = self.progress_score()
                else:
                    # 上游就是在这里 inline 算的：2580 − 用时 − 传感器×0.003 − 失联×扣分
                    elapsed = max(0.0, now - self.start_time)
                    self.score = (2580.0 - elapsed - self.sensor_cost * 3e-3
                                  - self.uav_loss_count * cfg.uav_loss_penalty)
                self.finish('Mission finished', self.score)
            else:
                print('[score] score: %s' % self.score)
                self.publish_score()

    # ---------------------------------------------------------------- 跟踪
    def update_tracking(self, now):
        cfg = self.cfg
        for actor_id in list(self.left_actors):
            actor_pos = self.actors_pos[actor_id]
            if actor_pos is None:
                self.tracking_start_time[actor_id] = None
                continue
            in_range = False
            for uav_id in range(cfg.drones):
                if self.uav_lost[uav_id]:
                    continue
                uav_pos = self.uav_positions[uav_id]
                if uav_pos is None:
                    continue
                dx = uav_pos.x - actor_pos.x
                dy = uav_pos.y - actor_pos.y
                if dx * dx + dy * dy <= cfg.tracking_distance ** 2:
                    in_range = True
                    break
            if in_range:
                if self.tracking_start_time[actor_id] is None:
                    self.tracking_start_time[actor_id] = now
                elif now - self.tracking_start_time[actor_id] >= cfg.tracking_duration:
                    if actor_id not in self.tracked_actors:
                        self.tracked_actors.add(actor_id)
                        print('[score] actor_%d tracking success' % actor_id)
                        self.score = self.progress_score()
                        self.publish_score()
            else:
                self.tracking_start_time[actor_id] = None

    # -------------------------------------------------- UAV / 人物状态刷新
    def _probe_uav(self, index):
        """返回 (resp, z, vx, vy)；resp 为 None 表示本 tick 拿不到该机状态。

        优先用 tick() 批量探测好的结果（每 tick 每架 UAV 只调一次服务），
        保证与上游 score_cal.py 每 tick 每架一次调用的开销一致。
        """
        cache = getattr(self, '_probe_cache', None) or {}
        resp = cache.get(index)
        if resp is None:
            if self.get_model_state_cb is None:
                return None, None, None, None
            name = '%s_%d' % (self.cfg.uav_type, index)
            try:
                resp = self.get_model_state_cb(name)
            except Exception:
                return None, None, None, None
            if resp is None or not getattr(resp, 'success', False):
                return None, None, None, None
        pos = resp.pose.position
        twist = getattr(resp, 'twist', None)
        vx = vy = 0.0
        if twist is not None:
            vx, vy = twist.linear.x, twist.linear.y
        return resp, pos.z, vx, vy

    def update_players(self, now):
        """返回 True 表示本 tick 已判定结束。"""
        cfg = self.cfg
        extended = (cfg.score_mode == 'extended')
        for i in range(cfg.drones):
            resp, z, vx, vy = self._probe_uav(i)
            if resp is not None:
                self.uav_positions[i] = resp.pose.position
                self.uav_seen[i] = True
                self.uav_miss_count[i] = 0
                if not math.isfinite(z):
                    continue
                if extended and not self.uav_took_off[i] and z > cfg.takeoff_altitude:
                    self.uav_took_off[i] = True
                    self.score = self.progress_score()
                    self.publish_score()
                    print('[score] UAV %d took off at z=%.2f; +%g'
                          % (i, z, cfg.takeoff_score))
                if extended and math.isfinite(vx) and math.isfinite(vy):
                    self._collision_heuristic(i, z, vx, vy)
                if z > cfg.max_uav_altitude:
                    self.finish('Warning: UAV %d is higher than %g meters'
                                % (i, cfg.max_uav_altitude), 0)
                    return True
            elif self.uav_seen[i] and not self.uav_lost[i]:
                self.uav_miss_count[i] += 1
                if self.uav_miss_count[i] >= cfg.uav_miss_limit:
                    self.uav_lost[i] = True
                    self.uav_loss_count += 1
                    self.uav_positions[i] = None
                    self.score = self.progress_score()
                    print('[score] UAV %d lost; penalty %g' % (i, cfg.uav_loss_penalty))
        # ---- 人物真实位置 ----
        for i in list(self.left_actors):
            if self.get_model_state_cb is None:
                break
            try:
                resp = self.get_model_state_cb('actor_%d' % i)
            except Exception:
                continue
            if resp is None or not getattr(resp, 'success', False):
                continue
            pos = resp.pose.position
            if pos.x ** 2 + pos.y ** 2 != 0:
                self.actors_pos[i] = pos
        return False

    def _collision_heuristic(self, index, z, vx, vy):
        """extended 模式：水平速度 <0.1 m/s 持续 1s 且高度掉 ≥0.5m 记一次碰撞。"""
        cfg = self.cfg
        speed = math.hypot(vx, vy)
        now_wall = self.wall_clock()
        if speed < cfg.collision_low_v:
            if self.collision_low_v_since[index] is None:
                self.collision_low_v_since[index] = now_wall
                self.collision_last_height[index] = z
            elif (now_wall - self.collision_low_v_since[index] >= cfg.collision_low_v_dwell
                  and self.collision_last_height[index] - z >= cfg.collision_height_drop
                  and self.collision_count_per_uav[index] < cfg.collision_quota_per_uav):
                drop = self.collision_last_height[index] - z
                self.collision_count += 1
                self.collision_count_per_uav[index] += 1
                self.collision_low_v_since[index] = None
                self.collision_last_height[index] = None
                self.score = self.progress_score()
                self.publish_score()
                print('[score] UAV %d collision: height_drop=%.2f; total=%d'
                      % (index, drop, self.collision_count))
        else:
            self.collision_low_v_since[index] = None
            self.collision_last_height[index] = None

    # ---------------------------------------------------------------- tick
    def tick(self, now, probe_positions=None):
        """probe_positions: {uav_index: GetModelState 响应} 由节点批量取好，
        避免每架 UAV 每 tick 重复调服务。"""
        if self.mission_finished:
            return True
        self._probe_cache = probe_positions or {}
        if self.update_players(now):
            return True
        self.time_usage = now - self.start_time
        self.update_tracking(now)
        if self.time_usage > self.cfg.mission_timeout:
            self.finish('Time out, mission failed')
            return True
        self.score = self.progress_score()
        self.publish_score()
        self.publish_time_cb(int(self.time_usage))
        self.publish_left_cb(str(self.left_actors))
        return False

    def summary(self):
        return dict(score=self.score, time_usage=self.time_usage,
                    left_actors=list(self.left_actors),
                    find_finish=self.find_finish, target_finish=self.target_finish,
                    tracked=sorted(self.tracked_actors),
                    uav_lost=[i for i, v in enumerate(self.uav_lost) if v],
                    uav_took_off=[i for i, v in enumerate(self.uav_took_off) if v],
                    collision_count=self.collision_count,
                    mission_finished=self.mission_finished,
                    mission_completed=self.mission_completed)


# ===========================================================================
# 第 7 块：ROS 集成（一个节点 = 6 个人物大脑 + 裁判评分）
# ===========================================================================
class _RosBundle(object):
    pass


def ensure_ros_log_dir():
    """~/.ros/log 只读时换到工作区可写目录（否则 init_node 直接崩）。"""
    base = os.path.expanduser('~/.ros/log')
    writable = False
    try:
        os.makedirs(base, exist_ok=True)
        probe = os.path.join(base, '.write_test')
        with open(probe, 'w') as fh:
            fh.write('ok')
        os.remove(probe)
        writable = True
    except Exception:
        writable = False
    if not writable:
        alt = os.path.join(WORKSPACE_DIR, 'logs', 'ros')
        os.makedirs(alt, exist_ok=True)
        os.environ['ROS_LOG_DIR'] = alt
        # roscore/roslaunch 还要往 ~/.ros 写 *.pid，只读时同样会崩
        if not os.environ.get('ROS_HOME'):
            ros_home = os.path.join(WORKSPACE_DIR, 'logs', 'ros_home')
            os.makedirs(ros_home, exist_ok=True)
            os.environ['ROS_HOME'] = ros_home
        print('[ros] %s 不可写，ROS_LOG_DIR 改用 %s，ROS_HOME 改用 %s'
              % (base, alt, os.environ.get('ROS_HOME', '(未设置)')))
    return os.environ.get('ROS_LOG_DIR')


def bootstrap_catkin_paths():
    """把 catkin 工作区的 devel python 路径接进 sys.path。

    ros_actor_cmd_pose_plugin_msgs / actor_msgs 这类自己 build 出来的消息包只存在于
    <ws>/devel/lib/python3/dist-packages，不 source devel/setup.bash 就 import 不到。
    这里自动挂上，省掉"必须先 source"这一步（也可用 $REFEREE_WS 指定别的工作区）。
    """
    added = []
    paths = []
    for ws in (os.environ.get('REFEREE_WS'), WORKSPACE_DIR):
        if not ws:
            continue
        paths.append(os.path.join(ws, 'devel', 'lib', 'python3', 'dist-packages'))
        paths.append(os.path.join(ws, 'devel', 'lib', 'python3', 'site-packages'))
    for prefix in os.environ.get('CMAKE_PREFIX_PATH', '').split(os.pathsep):
        if prefix:
            paths.append(os.path.join(prefix, 'lib', 'python3', 'dist-packages'))
    for path in paths:
        if os.path.isdir(path) and path not in sys.path:
            sys.path.insert(0, path)
            added.append(path)
    if added:
        print('[ros] 已自动挂上 catkin python 路径：%s' % ', '.join(added))
    return added


def import_ros():
    """按需导入 ROS（gen-map / selftest 不需要 ros master，也就不会导入）。"""
    ensure_ros_log_dir()
    bootstrap_catkin_paths()
    ros = _RosBundle()
    import rospy
    from std_msgs.msg import Float32, Float64, Int16, String
    from geometry_msgs.msg import Twist
    from gazebo_msgs.msg import ModelStates
    from gazebo_msgs.srv import (DeleteModel, GetModelState, SetModelState,
                                 SetModelStateRequest)
    try:
        from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo, ActorMotion
    except ImportError:                      # 备用：你工作区里的 actor_msgs
        from actor_msgs.msg import ActorInfo
        from ros_actor_cmd_pose_plugin_msgs.msg import ActorMotion
    try:
        from nav_msgs.msg import Odometry
    except ImportError:
        Odometry = None
    ros.Odometry = Odometry
    ros.rospy = rospy
    ros.Int16, ros.String, ros.Float32, ros.Twist = Int16, String, Float32, Twist
    ros.Float64 = Float64
    ros.ModelStates = ModelStates
    ros.DeleteModel, ros.GetModelState = DeleteModel, GetModelState
    ros.SetModelState, ros.SetModelStateRequest = SetModelState, SetModelStateRequest
    ros.ActorInfo, ros.ActorMotion = ActorInfo, ActorMotion
    return ros


def load_black_box(path):
    import ast
    if not path:
        raise SystemExit('[run] 必须给 --black-box（gen-map 产出的 black_box.txt）')
    with open(path) as fh:
        text = fh.read().strip()
    try:
        data = ast.literal_eval(text)
    except Exception as exc:
        raise SystemExit('[run] black_box.txt 解析失败（%s）：%s' % (path, exc))
    return [([float(b[0][0]), float(b[0][1])], [float(b[1][0]), float(b[1][1])])
            for b in data]


def load_obstacle_points(path):
    """obstacle.txt 每行两个数（x y）。上游用 numpy.loadtxt 读全部列，这里取前两列。"""
    points = []
    if not path or not os.path.isfile(path):
        return points
    with open(path) as fh:
        for line in fh:
            parts = line.split()
            if len(parts) < 2:
                continue
            try:
                points.append((float(parts[0]), float(parts[1])))
            except ValueError:
                continue
    return points


class RefereeNode(object):
    def __init__(self, cfg):
        self.cfg = cfg
        self.ros = import_ros()
        rospy = self.ros.rospy
        cfg.black_box = cfg.black_box or os.path.join(os.getcwd(), 'black_box.txt')
        cfg.obstacle = cfg.obstacle or os.path.join(os.getcwd(), 'obstacle.txt')
        self.black_box = load_black_box(cfg.black_box)
        self.obstacle_points = load_obstacle_points(cfg.obstacle)
        print('[run] black_box: %s（%d 盒）' % (cfg.black_box, len(self.black_box)))
        print('[run] obstacle : %s（%d 点）' % (cfg.obstacle, len(self.obstacle_points)))
        rospy.init_node('robocup_referee', anonymous=False, disable_signals=False)
        self.clock = rospy.get_time
        self.wall_clock = time.time

        # ---- 服务 / 发布者 ----
        self.delete_model_srv = rospy.ServiceProxy('/gazebo/delete_model',
                                                   self.ros.DeleteModel)
        self.get_state_srv = rospy.ServiceProxy('/gazebo/get_model_state',
                                                self.ros.GetModelState)
        self.set_state_srv = rospy.ServiceProxy('/gazebo/set_model_state',
                                                self.ros.SetModelState)
        self.score_pub = rospy.Publisher('/score', self.ros.Int16, queue_size=1)
        self.time_pub = rospy.Publisher('/time_usage', self.ros.Int16, queue_size=1)
        self.left_pub = rospy.Publisher('/left_actors', self.ros.String, queue_size=1)
        self.find_pubs = [rospy.Publisher('/find_actor_%d' % i, self.ros.Float32,
                                          queue_size=5)
                          for i in range(cfg.actor_num)]
        # 高精度版本：Float32 装不下 epoch 时间戳（会被量化到 128s 一档），
        # 这里额外发一份 Float64，供需要精确时间的消费者使用（上游话题保持不变）。
        self.find_pubs_f64 = [rospy.Publisher('/find_actor_%d_f64' % i,
                                              self.ros.Float64, queue_size=5)
                              for i in range(cfg.actor_num)]
        self.cmd_pubs = [rospy.Publisher('/actor_%d/cmd_motion' % i,
                                         self.ros.ActorMotion, queue_size=10)
                         for i in range(cfg.actor_num)]

        # ---- 世界状态 ----
        self.world = WorldState(cfg, clock=self.wall_clock)
        rospy.Subscriber('/gazebo/model_states', self.ros.ModelStates,
                         self.world.on_model_states, queue_size=1)
        if cfg.uav_odom in ('on', 'auto'):
            if self.ros.Odometry is None:
                print('[run] 未找到 nav_msgs/Odometry，回退为 model_states 位姿差分')
            else:
                for i in range(cfg.drones):
                    topic = '/xtdrone/%s_%d/ground_truth/odom' % (cfg.uav_type, i)
                    rospy.Subscriber(
                        topic, self.ros.Odometry,
                        (lambda idx: lambda msg: self.world.on_odom(idx, msg))(i),
                        queue_size=1)
                print('[run] 已订阅 /xtdrone/<type>_<i>/ground_truth/odom'
                      '（uav-odom=%s；收不到就自动降级为位姿差分）' % cfg.uav_odom)

        # ---- 谁在发 cmd_vel（tracker 让位判定）----
        self.yield_until = {}
        self.cmd_vel_seen = {}
        if cfg.tracker_yield > 0:
            for i in range(cfg.actor_num):
                rospy.Subscriber('/actor_%d/cmd_vel' % i, self.ros.Twist,
                                 (lambda idx: lambda msg: self._on_cmd_vel(idx))(i),
                                 queue_size=1)
            print('[run] tracker 让位 %.1fs：/actor_*/cmd_vel 期间不发 cmd_motion'
                  % cfg.tracker_yield)

        # ---- 人物大脑 ----
        self.spawner = HumanSpawner(cfg, self.black_box, self.world)
        self.brains = {}
        for i in range(cfg.actor_num):
            avoid_tool = ObstacleAvoid(self.obstacle_points)
            self.brains[i] = ActorBrain(
                i, cfg, self.world, self.spawner, avoid_tool, self.black_box,
                publish_cmd=None,
                teleport_cmd=(None if cfg.dry_run else self._teleport),
                clock=self.wall_clock)
            rospy.Subscriber('/find_actor_%d' % i, self.ros.Float32,
                             (lambda idx: lambda msg: self._on_found(idx, msg))(i),
                             queue_size=1)

        # ---- 评分 ----
        dump_path = (cfg.score_output
                     or os.environ.get('SCORE_OUTPUT_PATH')
                     or os.path.join(os.getcwd(), 'referee_score.txt'))
        self.scorer = RefereeScorer(
            cfg,
            publish_score=(None if cfg.dry_run else self.score_pub.publish),
            publish_time=(None if cfg.dry_run else self.time_pub.publish),
            publish_left=(None if cfg.dry_run else self.left_pub.publish),
            find_publishers=([] if cfg.dry_run
                             else [self._make_find_publisher(i)
                                   for i in range(cfg.actor_num)]),
            delete_model=(None if cfg.dry_run else self._delete_model),
            get_model_state=self._get_model_state,
            clock=rospy.get_time,
            wall_clock=time.time,
            dump_path=dump_path)
        if cfg.dump_score:
            atexit.register(self.scorer.dump_score_file)
        print('[run] 评分口径 %s；score.txt -> %s；dry_run=%s'
              % (cfg.score_mode, dump_path, cfg.dry_run))

        # ---- 人物话题映射（与你的 bbox2coord_node 一致）----
        for topic, ids in (('green', [0]), ('blue', [1]), ('brown', [2]),
                           ('white', [3]), ('red1', [5]), ('red2', [4])):
            rospy.Subscriber('/actor_%s_info' % topic, self.ros.ActorInfo,
                             (lambda a: lambda msg: self.scorer.process_actor_detection(msg, a))(ids),
                             queue_size=1)
        print('[run] 已订阅 /actor_{green,blue,brown,white,red1,red2}_info')

    # ------------------------------------------------------------ ROS 动作
    def _make_find_publisher(self, index):
        def publish(t):
            self.find_pubs[index].publish(t)
            self.find_pubs_f64[index].publish(t)
        return publish

    def _on_found(self, actor_id, msg):
        """用**收到消息的本地时刻**当基准（理由见 ActorBrain.on_found 的注释）。"""
        self.brains[actor_id].on_found(self.clock())

    def _on_cmd_vel(self, actor_id):
        self.yield_until[actor_id] = self.wall_clock() + self.cfg.tracker_yield
        self.cmd_vel_seen[actor_id] = self.cmd_vel_seen.get(actor_id, 0) + 1

    def _delete_model(self, name):
        try:
            resp = self.delete_model_srv(name)
        except Exception as exc:
            print('[run] delete_model(%s) 异常: %s' % (name, exc))
            return False
        return bool(getattr(resp, 'success', True))

    def _get_model_state(self, name):
        try:
            return self.get_state_srv(name, 'ground_plane')
        except Exception:
            return None

    def _teleport(self, actor_id, x, y):
        req = self.ros.SetModelStateRequest()
        req.model_state.model_name = 'actor_%d' % actor_id
        req.model_state.pose.position.x = float(x)
        req.model_state.pose.position.y = float(y)
        req.model_state.pose.position.z = 1.25
        try:
            resp = self.set_state_srv(req)
        except Exception as exc:
            print('[run] set_model_state(actor_%d) 异常: %s' % (actor_id, exc))
            return False
        return bool(getattr(resp, 'success', True))

    def _publish_cmd(self, actor_id, cmd):
        cfg = self.cfg
        if cfg.dry_run:
            return
        if cfg.tracker_yield > 0 and self.wall_clock() < self.yield_until.get(actor_id, 0.0):
            return
        msg = self.ros.ActorMotion()
        msg.x, msg.y, msg.v = float(cmd[0]), float(cmd[1]), float(cmd[2])
        self.cmd_pubs[actor_id].publish(msg)

    # ---------------------------------------------------------------- 主循环
    def run(self):
        cfg = self.cfg
        rospy = self.ros.rospy
        rospy.sleep(1.0)                      # 上游 score_cal 里也有这一步
        self.scorer.start(rospy.get_time())
        rate = rospy.Rate(cfg.actor_rate)
        t0 = time.time()
        last_log = 0.0
        print('[run] 开始：%d 架 %s，%d 个人物，控制=%s，传送间隔=%.0fs'
              % (cfg.drones, cfg.uav_type, cfg.actor_num,
                 'on' if not cfg.no_actor_control else 'off',
                 cfg.teleport_interval))
        while not rospy.is_shutdown():
            now = rospy.get_time()
            left = set(self.scorer.left_actors)
            for actor_id, brain in self.brains.items():
                brain.active = actor_id in left
                cmd = brain.tick(now)
                if cmd is not None and not cfg.no_actor_control:
                    self._publish_cmd(actor_id, cmd)
            probe = {}
            for i in range(cfg.drones):
                resp = self._get_model_state('%s_%d' % (cfg.uav_type, i))
                if resp is not None and getattr(resp, 'success', False):
                    probe[i] = resp
            if self.scorer.tick(now, probe_positions=probe):
                break
            if time.time() - last_log >= 5.0:
                last_log = time.time()
                s = self.scorer.summary()
                extra = ''
                if cfg.tracker_yield > 0:
                    yielding = [i for i, t in self.yield_until.items()
                                if self.wall_clock() < t]
                    extra = ' yielding=%s' % yielding
                print('[run] t=%.0fs score=%s left=%s tracked=%s%s'
                      % (s['time_usage'], s['score'], s['left_actors'],
                         s['tracked'], extra))
            if cfg.duration and (time.time() - t0) >= cfg.duration:
                print('[run] 达到 --duration %.0fs，主动退出' % cfg.duration)
                break
            rate.sleep()
        self.scorer.dump_score_file()
        print('[run] 结束：%s' % self.scorer.summary())
        return self.scorer.summary()


# ===========================================================================
# 第 8 块：命令行
# ===========================================================================
def default_src_dir():
    cand = os.environ.get('REFEREE_SRC_DIR')
    if cand and os.path.isdir(cand):
        return cand
    guess = os.path.join(WORKSPACE_DIR, 'XTDrone', 'robocup')
    if os.path.isfile(os.path.join(guess, 'rover_static.world')):
        return guess
    return guess


def cmd_gen_map(args):
    cfg = Config(base=args.base, rovers=args.rovers, seed=args.seed,
                 out=os.path.abspath(args.out), src_dir=os.path.abspath(args.src_dir),
                 fix_known_bugs=args.fix_known_bugs, actor_num=args.actor_num)
    gen = MapGenerator(cfg)
    result = gen.generate()
    print('[gen-map] seed=%s base=base%d 人物落点=%s'
          % (cfg.seed, result['map_num'], result['actor_points']))
    return 0


def cmd_run(args):
    cfg = Config(drones=args.drones, uav_type=args.uav_type,
                 score_mode=args.score_mode, tracker_yield=args.tracker_yield,
                 uav_odom=args.uav_odom, dry_run=args.dry_run,
                 duration=args.duration, no_actor_control=args.no_actor_control,
                 enable_shot_detection=args.enable_shot_detection,
                 black_box=args.black_box, obstacle=args.obstacle,
                 score_output=args.score_output,
                 dump_score=(not args.no_score_dump),
                 teleport_interval=args.teleport_interval,
                 debug_teleport=args.debug_teleport,
                 actor_num=args.actor_num)
    node = RefereeNode(cfg)
    node.run()
    return 0


def cmd_info(args):
    cfg = Config()
    print('robocup_referee 配置默认值：')
    for key in sorted(DEFAULTS):
        print('  %-26s %r' % (key, getattr(cfg, key)))
    print('\n默认 src_dir（gen-map 输入）: %s' % default_src_dir())
    for name in ('base1.world', 'base2.world', 'base3.world', 'rover_static.world'):
        path = os.path.join(default_src_dir(), name)
        print('  %-22s %s' % (name, 'OK' if os.path.isfile(path) else '缺失'))
    print('\n场景文件解析顺序（--src-dir 未给时）：')
    print('  1) $REFEREE_SRC_DIR')
    print('  2) %s' % os.path.join(WORKSPACE_DIR, 'XTDrone', 'robocup'))
    print('  3) %s' % os.path.join(WORKSPACE_DIR, 'waypoint', 'world') + '（只有 base 底图）')
    return 0


# ===========================================================================
# 第 9 块：selftest（不需要 ROS master）
# ===========================================================================
class _FakePoint(object):
    def __init__(self, x, y, z=1.25):
        self.x, self.y, self.z = x, y, z


class _FakeResp(object):
    def __init__(self, x, y, z=1.25, vx=0.0, vy=0.0, ok=True):
        self.success = ok
        self.pose = type('P', (), {'position': _FakePoint(x, y, z)})()
        self.twist = type('T', (), {'linear': _FakePoint(vx, vy, 0.0)})()


class _FakeMsg(object):
    def __init__(self, cls=None, x=0.0, y=0.0):
        self.cls, self.x, self.y = cls, x, y


class _Clock(object):
    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t


def _parse_first_model_poses(text, names):
    """取每个 <model name='X'> 之后的第一个 <pose>（= 模型级位姿），跳过 <state>。"""
    import re
    poses = {}
    cur = None
    in_state = False
    for line in text.splitlines():
        s = line.strip()
        if s.startswith('<state '):
            in_state = True
            continue
        if s.startswith('</state>'):
            in_state = False
            continue
        if in_state:
            continue
        if s.startswith('<model name='):
            cur = None
            for n in names:
                if ("<model name='%s'>" % n) in line:
                    cur = n
                    break
            continue
        if cur is not None and s.startswith('<pose'):
            m = re.search(r'<pose[^>]*>\s*(-?[\d.]+)\s+(-?[\d.]+)', s)
            if m:
                poses[cur] = (m.group(1), m.group(2))
    return poses


def _check(name, cond, detail=''):
    print('  [%s] %-46s %s' % ('PASS' if cond else 'FAIL', name, detail))
    return bool(cond)


def cmd_selftest(args):
    failures = []
    tmp = tempfile.mkdtemp(prefix='referee_selftest_')
    src = default_src_dir()
    print('selftest：src_dir=%s  tmp=%s' % (src, tmp))

    # ---------------------------------------------------------- 1. 配置
    print('\n[1] 配置')
    cfg = Config()
    failures.append(not _check('sensor_cost()==1700（上游默认传感器组合）',
                               abs(cfg.sensor_cost() - 1700.0) < 1e-9,
                               'value=%s' % cfg.sensor_cost()))

    # ------------------------------------------------- 2. 地图生成（修 bug）
    print('\n[2] 地图生成 --fix-known-bugs（等价你 9/19 的补丁）')
    out_fix = os.path.join(tmp, 'fixed')
    cfg_fix = Config(base=1, seed=1234, out=out_fix, src_dir=src,
                     fix_known_bugs=True)
    try:
        res = MapGenerator(cfg_fix).generate()
        ok = True
        detail = ''
    except Exception as exc:
        res, ok = None, False
        detail = '异常: %s' % exc
    failures.append(not _check('生成不抛异常', ok, detail))
    if res:
        world = open(os.path.join(out_fix, 'robocup.world')).read()
        house = open(os.path.join(out_fix, 'house.world')).read()
        obstacle_lines = open(os.path.join(out_fix, 'obstacle.txt')).read().splitlines()
        poses = _parse_first_model_poses(world, MapGenerator.NAME_LIST)
        distinct = set(poses.values())
        failures.append(not _check('house.world 非空', len(house) > 1000,
                                   '%d B' % len(house)))
        failures.append(not _check('13 栋建筑都改写了位姿', len(poses) == 13,
                                   '%d/13' % len(poses)))
        failures.append(not _check('13 栋落在 13 个不同点（修掉"全叠一起"）',
                                   len(distinct) == 13, '%d 个不同' % len(distinct)))
        failures.append(not _check('robocup.world 含 6 个人物 init_pose',
                                   world.count('<init_pose>') == 6,
                                   '%d 个' % world.count('<init_pose>')))
        failures.append(not _check('obstacle.txt 非空', len(obstacle_lines) > 1000,
                                   '%d 行' % len(obstacle_lines)))
        bb = load_black_box(os.path.join(out_fix, 'black_box.txt'))
        failures.append(not _check('black_box.txt 可解析', len(bb) >= 30,
                                   '%d 个包围盒' % len(bb)))

    # ------------------------------------------- 3. 地图生成（忠实上游行为）
    print('\n[3] 地图生成（默认 = 忠实上游 5171c58，不加补丁）')
    out_raw = os.path.join(tmp, 'upstream')
    try:
        res_raw = MapGenerator(Config(base=1, seed=1234, out=out_raw,
                                      src_dir=src)).generate()
        sizes = {n: os.path.getsize(os.path.join(out_raw, n))
                 for n in ('house.world', 'robocup.world', 'obstacle.txt')}
        poses_raw = _parse_first_model_poses(
            open(os.path.join(out_raw, 'robocup.world')).read(),
            MapGenerator.NAME_LIST)
        failures.append(not _check('不抛异常（复现上游的三个缺陷也无所谓）', True,
                                   'house=%dB 建筑不同点=%d 回退=%s'
                                   % (sizes['house.world'],
                                      len(set(poses_raw.values())),
                                      res_raw['fallback'])))
    except Exception as exc:
        failures.append(not _check('不抛异常', False, '异常: %s' % exc))

    # ------------------------------------------------------- 4. 避障绕行
    print('\n[4] ObstacleAvoid（上游 ObstacleAvoid.py）')
    wall = [(float(x), 0.0) for x in range(-10, 11)]
    path = ObstacleAvoid(wall).GetPointList((0.0, -10.0), (0.0, 10.0), 15.0)
    failures.append(not _check('正前方有障碍时会插入绕行子目标', len(path) >= 2,
                               '子目标数=%d' % len(path)))
    failures.append(not _check('路径终点仍等于目标点',
                               abs(path[-1][0]) < 1e-9 and abs(path[-1][1] - 10.0) < 1e-9,
                               'end=%s' % (path[-1],)))

    # ------------------------------------------------------- 5. 人物大脑
    print('\n[5] ActorBrain（上游 control_actor.py）')
    cfg_b = Config(actor_num=2, drones=1)
    world = WorldState(cfg_b, clock=_Clock())
    world.actor_pos = {'actor_0': (0.0, 0.0, 1.25), 'actor_1': (0.0, 0.0, 1.25)}
    spawner = HumanSpawner(cfg_b, [], world)
    b0 = ActorBrain(0, cfg_b, world, spawner, ObstacleAvoid([]), [], clock=_Clock())
    b1 = ActorBrain(1, cfg_b, world, spawner, ObstacleAvoid([]), [], clock=_Clock())
    b0.current_pose = b1.current_pose = (0.0, 0.0)
    b0.avoid = b1.avoid = (10.0, 0.0)
    c0 = b0.build_motion_command()
    c1 = b1.build_motion_command()
    failures.append(not _check('两人完全重叠时给出不同脱离方向',
                               abs(c0[0] - c1[0]) > 1e-6 or abs(c0[1] - c1[1]) > 1e-6,
                               'cmd0=(%.2f,%.2f) cmd1=(%.2f,%.2f)'
                               % (c0[0], c0[1], c1[0], c1[1])))
    world2 = WorldState(cfg_b, clock=_Clock())
    world2.actor_pos = {'actor_0': (5.0, 5.0, 1.25)}
    b2 = ActorBrain(0, cfg_b, world2, HumanSpawner(cfg_b, [], world2),
                    ObstacleAvoid([]), [], clock=_Clock())
    cmd = b2.tick(0.0)
    in_bounds = (cfg_b.actor_patrol_x[0] <= b2.patrol_target[0] <= cfg_b.actor_patrol_x[1]
                 and cfg_b.actor_patrol_y[0] <= b2.patrol_target[1] <= cfg_b.actor_patrol_y[1])
    failures.append(not _check('tick() 返回 (x,y,v) 且速度=常态速度',
                               cmd is not None and len(cmd) == 3
                               and abs(cmd[2] - cfg_b.actor_normal_speed) < 1e-9,
                               'cmd=%s' % (cmd,)))
    failures.append(not _check('巡逻点在场地范围内', in_bounds,
                               'patrol=%s' % (b2.patrol_target,)))

    # ------------------------------------------------------- 6. 计分公式
    print('\n[6] RefereeScorer 计分公式')
    cfg_s = Config()
    clock_s = _Clock(0.0)
    sc = RefereeScorer(cfg_s, clock=clock_s)
    sc.find_finish, sc.target_finish = 2, 1
    sc.tracked_actors = {0, 1}
    sc.uav_took_off[0] = True
    sc.collision_count = 1
    repo = sc.progress_score()
    cfg_x = Config(score_mode='extended')
    scx = RefereeScorer(cfg_x, clock=_Clock(0.0))
    scx.find_finish, scx.target_finish = 2, 1
    scx.tracked_actors = {0, 1}
    scx.uav_took_off[0] = True
    scx.collision_count = 1
    ext = scx.progress_score()
    failures.append(not _check('repo 口径 = 2*50+2*80+1*100-5.1 = 354.9',
                               abs(repo - 354.9) < 1e-6, 'value=%.4f' % repo))
    failures.append(not _check('extended 口径 = 354.9+10-30 = 334.9',
                               abs(ext - 334.9) < 1e-6, 'value=%.4f' % ext))
    clock_s.t = 100.0
    sc.mission_completed = True
    repo_done = sc.progress_score()
    scx.mission_completed = True
    scx.clock = _Clock(100.0)
    scx.start_time = 0.0
    ext_done = scx.progress_score()
    # 2026-09-28 修正：上游 _progress_score() 里**没有**"完成→2580−用时"分支
    # （那个 2580 是上游在"全消除"那处 inline 算的，见 selftest_scorer_equiv.py 场景1b），
    # 所以 repo 模式下 mission_completed 不影响该函数；extended 模式才走 2580 分支。
    failures.append(not _check('完成时 repo 仍等于未完成式 354.9（上游无 2580 分支）',
                               abs(repo_done - 354.9) < 1e-6,
                               'value=%.4f' % repo_done))
    failures.append(not _check('完成时 extended = 2580-100-5.1-30 = 2444.9',
                               abs(ext_done - 2444.9) < 1e-6, 'value=%.4f' % ext_done))

    # ------------------------------------------------ 7. 发现→保持→消除
    print('\n[7] 发现 / 保持 15 秒 / 消除')
    clock_d = _Clock(0.0)
    deleted = []
    scd = RefereeScorer(Config(actor_num=6), clock=clock_d,
                        delete_model=lambda name: (deleted.append(name) or True),
                        find_publishers=[(lambda i: lambda t: None)(i) for i in range(6)])
    scd.actors_pos[0] = _FakePoint(10.0, 10.0)
    t = 0.0
    while t <= 16.0:
        clock_d.t = t
        scd.process_actor_detection(_FakeMsg('green', 10.0, 10.0), [0])
        t += 0.5
    failures.append(not _check('发现后 find_finish=1', scd.find_finish == 1,
                               'find_finish=%d' % scd.find_finish))
    failures.append(not _check('保持 15s 后被删除',
                               'actor_0' in deleted and 0 not in scd.left_actors,
                               'deleted=%s left=%s' % (deleted, scd.left_actors)))
    failures.append(not _check('消除后 target_finish=1', scd.target_finish == 1,
                               'target_finish=%d' % scd.target_finish))

    # ------------------------------------------------------- 8. 跟踪得分
    print('\n[8] 连续跟踪 20s → +80')
    clock_t = _Clock(0.0)
    sct = RefereeScorer(Config(actor_num=6, drones=6), clock=clock_t)
    sct.actors_pos[0] = _FakePoint(10.0, 10.0)
    sct.uav_positions[0] = _FakePoint(12.0, 10.0)
    sct.update_tracking(0.0)
    clock_t.t = 20.5
    sct.update_tracking(20.5)
    failures.append(not _check('20.5s 后进入 tracked_actors',
                               0 in sct.tracked_actors,
                               'tracked=%s' % sorted(sct.tracked_actors)))
    clock_t.t = 21.0
    sct.update_tracking(21.0)
    failures.append(not _check('同一目标只记一次', len(sct.tracked_actors) == 1,
                               'tracked=%s' % sorted(sct.tracked_actors)))

    # ------------------------------------------------------------ 汇总
    shutil.rmtree(tmp, ignore_errors=True)
    bad = [f for f in failures if f]
    print('\nselftest：%d 项通过，%d 项失败' % (len(failures) - len(bad), len(bad)))
    return 1 if bad else 0


def build_parser():
    p = argparse.ArgumentParser(
        prog='referee.py',
        description='XTDrone/robocup 全链路集成裁判程序（地图生成 + 人物运动 + 评分）',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='典型流程：\n'
               '  referee.py gen-map --out <DIR> [--base 1|2|3|random] [--seed N]\n'
               '  referee.py run --black-box <DIR>/black_box.txt '
               '--obstacle <DIR>/obstacle.txt\n'
               '  referee.py selftest / selftest_ros.py / info\n'
               '详细说明见同目录 README.md（含与 XTDrone/robocup 各原件的对应关系）。')
    sub = p.add_subparsers(dest='command')

    g = sub.add_parser('gen-map', help='生成一局的世界（gzserver 之前跑一次）')
    g.add_argument('--out', required=True, help='输出目录')
    g.add_argument('--base', default='random', help='底图编号 1|2|3，或 random（上游默认随机 1..3）')
    g.add_argument('--seed', type=int, default=None, help='固定随机种子（复现同一张图）')
    g.add_argument('--rovers', type=int, default=DEFAULTS['rovers'])
    g.add_argument('--actor-num', type=int, default=DEFAULTS['actor_num'])
    g.add_argument('--src-dir', default=default_src_dir(),
                   help='含 base*.world 与 rover_static.world 的目录')
    g.add_argument('--fix-known-bugs', action='store_true',
                   help='修上游三个已知缺陷（0 字节 world / 房屋重叠 / <state> 不同步）')
    g.set_defaults(func=cmd_gen_map)

    r = sub.add_parser('run', help='常驻裁判节点（人物运动 + 评分；gzserver 之后跑）')
    r.add_argument('--black-box', default=None, help='gen-map 产出的 black_box.txt')
    r.add_argument('--obstacle', default=None, help='gen-map 产出的 obstacle.txt')
    r.add_argument('--drones', type=int, default=DEFAULTS['drones'])
    r.add_argument('--uav-type', default=DEFAULTS['uav_type'])
    r.add_argument('--actor-num', type=int, default=DEFAULTS['actor_num'])
    r.add_argument('--score-mode', choices=('repo', 'extended'),
                   default=DEFAULTS['score_mode'])
    r.add_argument('--score-output', default=None, help='最终分落盘路径')
    r.add_argument('--no-score-dump', action='store_true',
                   help='关掉分数落盘（上游 score_cal.py 没有落盘功能，这是本程序附加的）')
    r.add_argument('--tracker-yield', type=float, default=DEFAULTS['tracker_yield'],
                   help='/actor_*/cmd_vel 出现后让位秒数；0 = 裁判独占人物控制')
    r.add_argument('--uav-odom', choices=('auto', 'on', 'off'),
                   default=DEFAULTS['uav_odom'])
    r.add_argument('--no-actor-control', action='store_true',
                   help='只评分，不驱动人物')
    r.add_argument('--enable-shot-detection', action='store_true',
                   help='打开上游那段"卡住=被击中"死代码')
    r.add_argument('--dry-run', action='store_true',
                   help='只订阅与计算，不发 /score、不发 cmd_motion、不删模型、不传送')
    r.add_argument('--duration', type=float, default=0.0,
                   help='跑满 N 秒后主动退出（0 = 不退出）')
    r.add_argument('--teleport-interval', type=float,
                   default=DEFAULTS['teleport_interval'],
                   help='人物被"发现"后多少秒换落点（上游 25s）')
    r.add_argument('--debug-teleport', action='store_true',
                   help='打印人物传送倒计时（排障用）')
    r.set_defaults(func=cmd_run)

    s = sub.add_parser('selftest', help='无 ROS 的纯逻辑自检（含真地图生成）')
    s.set_defaults(func=cmd_selftest)

    i = sub.add_parser('info', help='打印配置默认值与输入文件查找结果')
    i.set_defaults(func=cmd_info)
    return p


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if not getattr(args, 'command', None):
        parser.print_help()
        return 2
    return args.func(args)


if __name__ == '__main__':
    sys.exit(main())
