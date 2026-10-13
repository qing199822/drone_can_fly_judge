#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""裁判系统镜像（本地预判工具）。

严格按 `/home/lishu2/2025_ZZU_FLY/score_cal.py` 的判定逻辑复刻，用来在**正式跑之前**
预判裁判会给多少分、以及每个目标卡在哪一步（坐标精度 / 消息连续性 / 距离）。

它做两件事：
    1. 订阅裁判系统同样的话题、用同样的红色映射与消息类型（ros_actor_cmd_pose_plugin_msgs）；
    2. 用 /gazebo/get_model_state 读人物与无人机真值，按 score_cal.py 的规则实时判定：
         - 找到并删除人物：坐标误差 < 1m、相邻消息间隔 <= 1s、连续 15s（之后裁判会删模型）
         - 跟踪分：任一无人机真值在人物 10m 内连续 20s
         - 越限/丢失：无人机真值高度 > 6m 立即 0 分；无人机连续 3 次取不到则罚 100 分
    并在终端打印进度表（哪一步差多少秒），不影响正式裁判。

用法（与 1.sh 起的仿真配合，单独一个终端）：
    rosrun ros_actor_cmd_pose_plugin_msgs 没有可执行文件，直接跑脚本即可：
    python3 src/yolo/referee_mirror.py typhoon_h480 6 6

可选参数（rosparam）：
    ~mono_cam ~stereo_cam ~laser1d ~laser2d ~laser3d ~gimbal ~bino_cam   # 传感器分，默认与 score_cal 一致
    ~uav_loss_penalty                                                    # 单机丢失罚分，默认 100
"""

import math
import sys
import threading
import time

import rospy
from gazebo_msgs.srv import GetModelState
from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo

# ---- 与 score_cal.py 保持一致的常量 ----
ACTOR_NUM = 6
UAV_NUM = 6
ERR_THRESHOLD = 1.0            # 坐标误差阈值（m）
DETECTION_INTERVAL = 1.0       # 相邻消息最大间隔（s）
DETECTION_DURATION = 15.0      # 连续满足后删除人物（s）
TRACKING_DURATION = 20.0       # 跟踪分需要的持续时间（s）
TRACKING_DISTANCE = 10.0       # 跟踪分距离阈值（m）
MISSION_TIMEOUT = 600.0        # 任务总时长（s）
MAX_UAV_ALTITUDE = 6.0         # 超过即 0 分
UAV_MISS_LIMIT = 3


class RefereeCore(object):
    """纯逻辑部分（可单测）：不依赖 ROS，时间与真值都由外部注入。"""

    def __init__(self, actor_num=ACTOR_NUM, uav_num=UAV_NUM,
                 err_threshold=ERR_THRESHOLD, detection_interval=DETECTION_INTERVAL,
                 detection_duration=DETECTION_DURATION, tracking_duration=TRACKING_DURATION,
                 tracking_distance=TRACKING_DISTANCE):
        self.actor_num = actor_num
        self.uav_num = uav_num
        self.err_threshold = err_threshold
        self.detection_interval = detection_interval
        self.detection_duration = detection_duration
        self.tracking_duration = tracking_duration
        self.tracking_distance = tracking_distance

        self.left_actors = list(range(actor_num))     # 还没被删除的人物
        self.find_actors = list(range(actor_num))     # 还没触发过"找到"的
        self.actors_pos = [None] * actor_num          # 真值
        self.count_flag = [False] * actor_num
        self.topic_arrive_time = [0.0] * actor_num
        self.find_time = [0.0] * actor_num
        self.tracking_start_time = [None] * actor_num
        self.tracked_actors = set()
        self.last_reset_reason = {}
        self.events = []

    # --- 人物坐标消息（等价于 _process_actor_detection）---
    def process_actor_detection(self, now, actor_ids, x, y):
        for actor_id in actor_ids:
            if actor_id not in self.left_actors:
                continue
            position = self.actors_pos[actor_id]
            previous = self.topic_arrive_time[actor_id]
            self.topic_arrive_time[actor_id] = now
            if position is None:
                self._reset(actor_id, 'no truth')
                continue
            distance = math.hypot(x - position[0], y - position[1])
            continuous = (previous == 0.0) or (now - previous <= self.detection_interval)
            if distance >= self.err_threshold or not continuous:
                reason = ('error %.2fm >= %.2fm' % (distance, self.err_threshold)
                          if distance >= self.err_threshold
                          else 'message gap %.2fs' % (now - previous))
                self._reset(actor_id, reason)
                continue
            if not self.count_flag[actor_id]:
                self.count_flag[actor_id] = True
                self.find_time[actor_id] = now
                if actor_id in self.find_actors:
                    self.find_actors.remove(actor_id)
                self.events.append((now, 'find actor_%d (%.2fm)' % (actor_id, distance)))
                continue
            if now - self.find_time[actor_id] < self.detection_duration:
                continue
            # 裁判此时会调用 /gazebo/delete_model 删除该人物
            self.left_actors.remove(actor_id)
            self._reset(actor_id, 'deleted')
            self.events.append((now, 'actor_%d FOUND & DELETED' % actor_id))

    def _reset(self, actor_id, reason):
        self.count_flag[actor_id] = False
        self.find_time[actor_id] = 0.0
        self.topic_arrive_time[actor_id] = 0.0
        self.last_reset_reason[actor_id] = reason

    # --- 跟踪分（等价于 _update_tracking）---
    def update_tracking(self, now, uav_positions):
        for actor_id in list(self.left_actors):
            actor_pos = self.actors_pos[actor_id]
            if actor_pos is None:
                self.tracking_start_time[actor_id] = None
                continue
            in_range = False
            for uav_id in range(self.uav_num):
                uav_pos = uav_positions[uav_id]
                if uav_pos is None:
                    continue
                if math.hypot(uav_pos[0] - actor_pos[0], uav_pos[1] - actor_pos[1]) \
                        <= self.tracking_distance:
                    in_range = True
                    break
            if in_range:
                if self.tracking_start_time[actor_id] is None:
                    self.tracking_start_time[actor_id] = now
                elif now - self.tracking_start_time[actor_id] >= self.tracking_duration:
                    if actor_id not in self.tracked_actors:
                        self.tracked_actors.add(actor_id)
                        self.events.append((now, 'actor_%d TRACKING OK' % actor_id))
            else:
                self.tracking_start_time[actor_id] = None

    # --- 分数（等价于 _progress_score / 终局分）---
    def progress_score(self, sensor_cost=1700.0, uav_loss_count=0, uav_loss_penalty=100.0):
        find_finish = self.actor_num - len(self.find_actors)
        target_finish = self.actor_num - len(self.left_actors)
        return (find_finish * 50 + len(self.tracked_actors) * 80 + target_finish * 100
                - sensor_cost * 3e-3 - uav_loss_count * uav_loss_penalty)

    def final_score(self, elapsed, sensor_cost=1700.0, uav_loss_count=0, uav_loss_penalty=100.0):
        return (2580.0 - elapsed - sensor_cost * 3e-3 - uav_loss_count * uav_loss_penalty)


class RefereeMirror(object):
    def __init__(self, uav_type, actor_num, uav_num):
        self.uav_type = uav_type
        self.core = RefereeCore(actor_num=actor_num, uav_num=uav_num)
        self.uav_positions = [None] * uav_num
        self.uav_seen = [False] * uav_num
        self.uav_lost = [False] * uav_num
        self.uav_miss_count = [0] * uav_num
        self.uav_loss_count = 0
        self.uav_loss_penalty = float(rospy.get_param('~uav_loss_penalty', 100.0))
        def sensor_param(name, default):
            # 官方裁判用 ~xxx（即 /score_cal/xxx）；这里优先跟随它，保证两边扣分一致
            return int(rospy.get_param('/score_cal/' + name, rospy.get_param('~' + name, default)))
        self.sensor_cost = (sensor_param('mono_cam', 1) * 5e2
                            + sensor_param('stereo_cam', 0) * 1e3
                            + sensor_param('laser1d', 0) * 2e2
                            + sensor_param('laser2d', 1) * 1e3
                            + sensor_param('laser3d', 0) * 2e3
                            + sensor_param('gimbal', 1) * 2e2
                            + sensor_param('bino_cam', 0) * 1e3)
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        # 注意：rospy.get_time() 在 /use_sim_time 生效前后会切换基准（墙钟 <-> 仿真时钟），
        # 用它算"本轮耗时"会出现巨大/负数（曾误报 600s 超时）。耗时统计一律用墙钟单调时间。
        self.wall_start = time.monotonic()
        self.lock = threading.Lock()

        # 与 score_cal.py 完全一致的映射：red1 -> actor_5，red2 -> actor_4
        self.actor_id_dict = {'green': [0], 'blue': [1], 'brown': [2], 'white': [3], 'red': [4, 5]}
        rospy.Subscriber('/actor_green_info', ActorInfo, self._cb_color, queue_size=1)
        rospy.Subscriber('/actor_blue_info', ActorInfo, self._cb_color, queue_size=1)
        rospy.Subscriber('/actor_brown_info', ActorInfo, self._cb_color, queue_size=1)
        rospy.Subscriber('/actor_white_info', ActorInfo, self._cb_color, queue_size=1)
        rospy.Subscriber('/actor_red1_info', ActorInfo,
                         lambda msg: self._cb(msg, [5] if msg.cls == 'red' else []), queue_size=1)
        rospy.Subscriber('/actor_red2_info', ActorInfo,
                         lambda msg: self._cb(msg, [4] if msg.cls == 'red' else []), queue_size=1)

    def _cb_color(self, msg):
        self._cb(msg, self.actor_id_dict.get(msg.cls, []))

    def _cb(self, msg, actor_ids):
        if not msg.cls:
            return
        with self.lock:
            self.core.process_actor_detection(rospy.get_time(), actor_ids, msg.x, msg.y)

    def _update_truth(self):
        for i in range(self.core.uav_num):
            try:
                response = self.get_model_state('%s_%d' % (self.uav_type, i), 'ground_plane')
                if getattr(response, 'success', False):
                    position = response.pose.position
                    if math.isnan(position.z) or math.isinf(position.z):
                        continue
                    self.uav_positions[i] = (position.x, position.y)
                    self.uav_seen[i] = True
                    self.uav_miss_count[i] = 0
                    if position.z > MAX_UAV_ALTITUDE:
                        print('!! UAV %d 高度 %.2fm > %.0fm，正式裁判会立即判 0 分'
                              % (i, position.z, MAX_UAV_ALTITUDE))
                elif self.uav_seen[i] and not self.uav_lost[i]:
                    self.uav_miss_count[i] += 1
                    if self.uav_miss_count[i] >= UAV_MISS_LIMIT:
                        self.uav_lost[i] = True
                        self.uav_loss_count += 1
                        self.uav_positions[i] = None
                        print('!! UAV %d lost: -%.1f 分' % (i, self.uav_loss_penalty))
            except Exception:
                pass
        for actor_id in list(self.core.left_actors):
            try:
                response = self.get_model_state('actor_%d' % actor_id, 'ground_plane')
                if not getattr(response, 'success', True):
                    continue
                position = response.pose.position
                if position.x ** 2 + position.y ** 2 != 0:
                    self.core.actors_pos[actor_id] = (position.x, position.y)
            except Exception:
                continue

    def report(self):
        now = rospy.get_time()
        elapsed = time.monotonic() - self.wall_start
        with self.lock:
            score = self.core.progress_score(self.sensor_cost, self.uav_loss_count,
                                             self.uav_loss_penalty)
            lines = []
            for actor_id in self.core.left_actors:
                if self.core.count_flag[actor_id]:
                    held = now - self.core.find_time[actor_id]
                    find_txt = 'find %5.1fs/%.0fs' % (held, self.core.detection_duration)
                else:
                    reason = self.core.last_reset_reason.get(actor_id, '-')
                    find_txt = 'waiting (last reset: %s)' % reason
                track = self.core.tracking_start_time[actor_id]
                track_txt = ('track %5.1fs/%.0fs' % (now - track, self.core.tracking_duration)
                             if track is not None else 'track --')
                if actor_id in self.core.tracked_actors:
                    track_txt += ' OK'
                lines.append('  actor_%d: %-46s %s' % (actor_id, find_txt, track_txt))
            events = [e[1] for e in self.core.events[-3:]]
            return elapsed, score, lines, events


def main():
    uav_type = sys.argv[1] if len(sys.argv) > 1 else 'typhoon_h480'
    actor_num = int(sys.argv[2]) if len(sys.argv) > 2 else ACTOR_NUM
    uav_num = int(sys.argv[3]) if len(sys.argv) > 3 else UAV_NUM
    rospy.init_node('referee_mirror')
    mirror = RefereeMirror(uav_type, actor_num, uav_num)
    print('=== 裁判镜像已启动（与 score_cal.py 同规则）uav_type=%s actors=%d uavs=%d ==='
          % (uav_type, actor_num, uav_num))
    # 真值/跟踪判定按 10Hz 更新，与官方 score_cal.py 的 rospy.Rate(10) 完全一致；
    # 终端打印降频到 ~1Hz，避免刷屏。
    rate = rospy.Rate(10)
    cycle = 0
    while not rospy.is_shutdown():
        mirror._update_truth()
        with mirror.lock:
            mirror.core.update_tracking(rospy.get_time(), mirror.uav_positions)
        if cycle % 10 == 0:
            elapsed, score, lines, events = mirror.report()
            print('\n[t=%6.1fs] 预估分数 %.1f' % (elapsed, score))
            for line in lines:
                print(line)
            if events:
                print('  最近事件: %s' % ' | '.join(events))
            if elapsed > MISSION_TIMEOUT:
                print('!! 超过 %ds，正式裁判会判 Time out mission failed' % MISSION_TIMEOUT)
                break
        cycle += 1
        rate.sleep()


if __name__ == '__main__':
    main()
