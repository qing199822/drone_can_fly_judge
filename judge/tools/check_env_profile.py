#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""zzu_fly 环境画像自检（只读）—— 起算法栈之前，先确认"仿真环境到底是哪一种"。

背景（为什么需要这个脚本）
--------------------------------------------------------------------------
比赛时组委会启动仿真环境，我们只订阅；组委会用的是官方 XTDrone
``sitl_config/launch/robocup.launch``：

    世界   robocup.world
    机型   typhoon_h480（**基础型只有 CGO3 单目相机**，没有 2D 激光、没有深度相机）
    6 架   typhoon_h480_0 .. typhoon_h480_5，出生点在 (0,-3) 系

我们自己本地调试用的机型是 ``typhoon_h480_zzufly``（2D 激光 + Realsense 深度），
出生点在 (-17,-3) 系。我们的算法栈对传感器话题是**硬依赖**，环境不匹配时会
**静默失效**（不报错、不崩，只是没结果）：

    /map 恒空、tracking 永不接管、播报给裁判的全局坐标整体偏 17 m（误差<1m 判据必失败）

所以本脚本在 ``bash start_sim_only.sh`` 之前跑一次：探测真实 ROS 图 → 判定画像 →
列出缺什么、谁失效、后果是什么 → 校验出生点布局是否与我们的默认值一致。

用法
--------------------------------------------------------------------------
    python3 tools/check_env_profile.py                        # 人类可读报告
    python3 tools/check_env_profile.py --json                 # 机器可读 JSON
    python3 tools/check_env_profile.py --num-drones=6 --vehicle-type=typhoon_h480
    python3 tools/check_env_profile.py --timeout=10           # 放宽收帧窗口
    python3 tools/check_env_profile.py --profile=official     # 明确声明"我知道是官方环境"

    ROS_MASTER_URI=http://127.0.0.1:11999 python3 tools/check_env_profile.py   # 测"连不上"

退出码
--------------------------------------------------------------------------
    0   必需话题齐全，且出生点布局 = zzu_fly 本地默认（(-17,-3) 系）→ 可用默认参数起栈
    1   缺失关键话题，或出生点布局与本地默认不一致（含布局无法判定）→ 不要盲目起栈
    2   连不上 ROS master（含 ROS Python 库缺失、ROS_MASTER_URI 非法）
        —— 这种情况下**不会抛栈**，只打印清晰错误，便于 start_sim_only.sh / CI 判失败

设计意图与取舍
--------------------------------------------------------------------------
* **只读**：不发布任何话题、不调 gazebo 服务、不改参数、不写任何文件（除 ROS 自己的
  日志目录回退）。唯一的"副作用"是临时注册一个匿名节点去订阅话题，进程退出即消失。
* **"有发布者" 与 "能收到帧" 分开报告**：前者来自 master 的 getSystemState（判定画像
  和必需性的依据），后者来自真订阅一帧（`rospy.AnyMsg`，不需要消息包即可收）。仿真
  paused 时 gzserver 仍有发布者注册但一帧都不发，两者分开才不会误判画像。
* **收帧窗口共享**：所有待验证话题共用一个 ``--timeout`` 窗口（总耗时 ≈ timeout），
  而不是每话题各等 timeout（59 个话题 × 5s = 5 分钟，不可接受）。
* **不依赖 source devel/setup.bash**：脚本自己把 ``devel/lib/python3/dist-packages``、
  ``CMake`` 前缀、``/opt/ros/*/lib/python3/dist-packages`` 挂进 sys.path，然后才 import
  rospy；``~/.ros`` 不可写时按 ``../robocup_referee/referee.py`` 的做法把 ROS_LOG_DIR /
  ROS_HOME 指到工作区 ``logs/``。
* **不 import 业务消息包**：话题类型从 master 的 getTopicTypes 拿，收帧统一用
  ``rospy.AnyMsg``，只有在需要读 ``/gazebo/model_states`` 的位姿时才按类型名反序列化
  （``roslib.message.get_message_class``）。缺消息包只会让对应项降级为"未知"，不崩。
* **画像判定按"存在性"**（有没有发布者），不看帧数；帧数是辅助信息（paused 时会全 0）。
* **退出码口径固定**：``--profile`` 只决定"哪些话题算关键"，不改变 "exit 0 必须
  布局 == 本地默认" 这一条。因此在官方环境下即使用 ``--profile=official`` 也会
  返回 1 —— 这正是想要的：它告诉你"感知栈会失效 + 出生点偏移 17 m，先改参数"。

作者：zzu_fly 工具链（只新增本文件，不改仓库其它文件）
"""

import argparse
import json
import math
import os
import re
import sys
import threading
import time

# ---------------------------------------------------------------------------
# 常量：工作区、布局、画像、超时
# ---------------------------------------------------------------------------

# tools/check_env_profile.py -> <ws>/tools/check_env_profile.py -> <ws>
WORKSPACE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

SPAWN_STRIDE_X = 3.0
SPAWN_STRIDE_Y = 3.0
LOCAL_ORIGIN = (-17.0, -3.0)      # zzu_fly 本地调试布局
OFFICIAL_ORIGIN = (0.0, -3.0)     # 官方 XTDrone robocup.launch 布局
SPAWN_TOL_M = 1.0                 # 布局判定容差（local 与 official 相差 17 m，极易区分）

MASTER_CALL_TIMEOUT = 4.0         # 单次 master API 调用的上限（防止挂死）
TF_DESER_CAP = 200                # /tf 最多反序列化多少帧做 frame 采样（避免拖慢收帧）

PROFILE_OFFICIAL = 'official'
PROFILE_LOCAL = 'local-lidar-depth'
PROFILE_LIDAR = 'lidar-only'
PROFILE_UNKNOWN = 'unknown'
ALL_PROFILES = (PROFILE_OFFICIAL, PROFILE_LOCAL, PROFILE_LIDAR, PROFILE_UNKNOWN)

PROFILE_ZH = {
    PROFILE_OFFICIAL: 'official（官方 robocup.launch 基础型：CGO3 单目，无 2D 激光、无深度）',
    PROFILE_LOCAL: 'local-lidar-depth（本地 typhoon_h480_zzufly：2D 激光 + Realsense 深度）',
    PROFILE_LIDAR: 'lidar-only（对应官方 typhoon_h480_2d_lidar：只有 2D 激光）',
    PROFILE_UNKNOWN: 'unknown（无法匹配任何已知画像组合）',
}

ROLE_ZH = {
    'scan': '2D 激光',
    'scan_filtered': '自滤波激光（我们的节点产出）',
    'realsense_color': 'Realsense RGB',
    'realsense_depth': 'Realsense 深度',
    'cgo3_camera': 'CGO3 单目（官方基础型唯一相机）',
    'mavros_pose': 'MAVROS 局部位姿',
    'mavros_state': 'MAVROS 状态',
    'mavros_setpoint': 'MAVROS 位置设定值',
    'mission_state': '任务状态（我们发布的）',
    'gazebo_model_states': 'Gazebo 模型真值',
    'left_actors': '裁判剩余人物名单',
    'score': '裁判分数',
    'tf': 'TF 树',
    'clock': '仿真时钟',
}

# 每架机的话题。probe:
#   'frame' = 判"有发布者" + 尝试收一帧
#   'both'  = 只报"有发布者/有订阅者"（这类话题是"我们发、别人订阅"，收帧无意义）
#   'exist' = 只报存在性
DRONE_TOPICS = (
    dict(role='scan', suffix='scan', expected_type='sensor_msgs/LaserScan', probe='frame',
         desc='2D 激光'),
    dict(role='scan_filtered', suffix='scan_filtered', expected_type='sensor_msgs/LaserScan',
         probe='frame', desc='scan_self_filter 输出（可选）'),
    dict(role='realsense_color', suffix='realsense/depth_camera/color/image_raw',
         expected_type='sensor_msgs/Image', probe='frame', desc='Realsense RGB'),
    dict(role='realsense_depth', suffix='realsense/depth_camera/depth/image_raw',
         expected_type='sensor_msgs/Image', probe='frame', desc='Realsense 深度'),
    dict(role='cgo3_camera', suffix='cgo3_camera/image_raw', expected_type='sensor_msgs/Image',
         probe='frame', desc='官方基础型唯一相机'),
    dict(role='mavros_pose', suffix='mavros/local_position/pose',
         expected_type='geometry_msgs/PoseStamped', probe='frame', desc='MAVROS 局部位姿'),
    dict(role='mavros_state', suffix='mavros/state', expected_type='mavros_msgs/State',
         probe='frame', desc='MAVROS 状态'),
    dict(role='mavros_setpoint', suffix='mavros/setpoint_raw/local',
         expected_type='mavros_msgs/PositionTarget', probe='both',
         desc='我们发布 / mavros 订阅'),
    dict(role='mission_state', suffix='mission/state', expected_type='std_msgs/String',
         probe='exist', desc='area_mission_node 发布，tracking 订阅'),
)

GLOBAL_TOPICS = (
    dict(role='gazebo_model_states', topic='/gazebo/model_states',
         expected_type='gazebo_msgs/ModelStates', probe='frame', desc='模型真值位姿'),
    dict(role='left_actors', topic='/left_actors', expected_type='std_msgs/String',
         probe='frame', desc='裁判权威消除名单'),
    dict(role='score', topic='/score', expected_type='std_msgs/Int16', probe='frame',
         desc='裁判分数'),
    dict(role='tf', topic='/tf', expected_type='tf2_msgs/TFMessage', probe='frame',
         desc='TF 树'),
    dict(role='clock', topic='/clock', expected_type='rosgraph_msgs/Clock', probe='frame',
         desc='仿真时钟'),
)

# 画像 -> 关键话题集合（--profile 只影响这里）。
# clock 会在 use_sim_time=true 时动态加入。
_CRIT_SENSOR_LOCAL = {'scan', 'realsense_color', 'realsense_depth'}
_CRIT_MAVROS = {'mavros_pose', 'mavros_state'}
_CRIT_TRUTH = {'gazebo_model_states', 'left_actors', 'tf'}
CRITICAL_ROLES = {
    PROFILE_OFFICIAL: set(_CRIT_MAVROS) | {'cgo3_camera'} | set(_CRIT_TRUTH),
    PROFILE_LOCAL: set(_CRIT_MAVROS) | set(_CRIT_SENSOR_LOCAL) | set(_CRIT_TRUTH),
    PROFILE_LIDAR: set(_CRIT_MAVROS) | set(_CRIT_SENSOR_LOCAL) | set(_CRIT_TRUTH),
    # unknown 全都要，缺哪个都报，宁可吵也不要静默失效
    PROFILE_UNKNOWN: (set(_CRIT_MAVROS) | set(_CRIT_SENSOR_LOCAL) | {'cgo3_camera'}
                      | set(_CRIT_TRUTH)),
}

# 影响面诊断表：role -> (失效的节点/链路, 后果)。覆盖任务书要求的全部条目。
IMPACTS = (
    ('scan', 'scan_self_filter → scan_map_fuser → /map',
     '/map 恒空；area_mission_node 巡逻 A* 全 NO PATH（无路可走）；'
     'start_sim_only.sh 阶段 C 健康门 240s 超时 exit 1'),
    ('scan_filtered', 'scan_self_filter（我们自己的节点）',
     '可选：缺 = 自滤波节点没起；area_mission_node 的 scan_topic 默认 /<V>_i/scan_filtered '
     '会拿不到输入（不影响画像判定）'),
    ('realsense_color', 'yolo11n.py → multi_yolo_detecting_gpu → bounding_boxes',
     'yolo11n.py 无检测 → bbox2coord 无 bbox → tracking 不接管（跟踪永不启动）；'
     'road_follow.py 无道路先验'),
    ('realsense_depth', 'bbox2coord_node.py / area_mission 深度急停',
     '解不出世界坐标 → 裁判拿不到 /actor_*_info → 消除分 0；area_mission 深度急停失效'),
    ('cgo3_camera', '官方基础型唯一相机 / yolo11n.py（需 remap 到该话题）',
     '官方 typhoon_h480 只有这一路相机；我们的 yolo11n.py 默认订阅 realsense 路径，'
     '不 remap 就永远没有检测结果'),
    ('mavros_pose', 'tracking_node.cpp 的 waitForPose() / area_mission_node',
     'tracking_node.cpp 的 waitForPose() 永久阻塞（不超时、不打日志）；area_mission 停车'),
    ('mavros_state', 'xtdrone_mavros_bridge.py',
     '拿不到 connected/armed → 不解锁 → 飞机不起飞'),
    ('mavros_setpoint', 'xtdrone_mavros_bridge.py（发布者）',
     '仅提示：本话题是我们发布、mavros 订阅；"无订阅者"说明 mavros 没起'),
    ('mission_state', 'area_mission_node（发布）/ tracking state_machine.cpp（订阅）',
     '仅提示存在性：缺 = area_mission_node 没起，tracking 的状态机收不到 mission/state'),
    ('gazebo_model_states', '裁判真值链 / tracking 高度真值 / actor 消除判定',
     '高度真值缺失（可能真飞 >6 m 被裁判判 0）；actor 消除判定 fail-open'),
    ('left_actors', 'tracking state_machine.cpp（裁判权威消除名单）',
     '追踪归还延迟、抱人不放（只能等 model_states 超时兜底）'),
    ('score', '裁判 referee.py',
     '仅提示：缺 = 裁判端没起，本轮拿不到分数'),
    ('tf', 'scan_map_fuser（map→odom_<id>）/ bbox2coord（外参）/ transform_tree',
     'TF 链断 → scan_map_fuser 全部 continue（一条激光都用不上）、bbox2coord 外参解算失败'),
    ('clock', '所有用 rospy.Time / ros::Time 定时器的节点（use_sim_time=true 时）',
     '定时器与超时全部失序（卡死或瞬间触发）'),
)
IMPACT_BY_ROLE = {r: (nodes, cons) for r, nodes, cons in IMPACTS}

# 报告里展示顺序（关键项在前）
ROLE_ORDER = [r for r, _n, _c in IMPACTS]


# ---------------------------------------------------------------------------
# 环境准备：ROS 日志目录 / sys.path（都在 import rospy 之前）
# ---------------------------------------------------------------------------

def ensure_ros_log_dir():
    """~/.ros/log 只读时换到工作区可写目录（否则 init_node 直接崩）。

    与 ../robocup_referee/referee.py 的 ensure_ros_log_dir() 同样做法。
    返回 (ROS_LOG_DIR 或 None, 说明文字或 None)。
    """
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
    if writable:
        return os.environ.get('ROS_LOG_DIR'), None
    alt = os.path.join(WORKSPACE_DIR, 'logs', 'ros')
    try:
        os.makedirs(alt, exist_ok=True)
    except Exception as exc:
        return None, '警告：%s 与 %s 都不可写（%s），ROS 日志配置可能失败' % (base, alt, exc)
    os.environ['ROS_LOG_DIR'] = alt
    note = '%s 不可写，ROS_LOG_DIR 改用 %s' % (base, alt)
    # roscore/roslaunch 还要往 ~/.ros 写 *.pid，只读时同样会崩
    if not os.environ.get('ROS_HOME'):
        ros_home = os.path.join(WORKSPACE_DIR, 'logs', 'ros_home')
        try:
            os.makedirs(ros_home, exist_ok=True)
            os.environ['ROS_HOME'] = ros_home
            note += '，ROS_HOME 改用 %s' % ros_home
        except Exception:
            pass
    return alt, note


def ensure_ros_python_logging():
    """不 source setup.bash 时 rospy 会打印 'cannot load logging configuration file'。

    这里在环境变量缺失时补一个存在的配置文件，纯降噪，失败无所谓。
    """
    if os.environ.get('ROS_PYTHON_LOG_CONFIG_FILE'):
        return
    candidates = []
    etc = os.environ.get('ROS_ETC_DIR')
    if etc:
        candidates.append(os.path.join(etc, 'python_logging.conf'))
    distro = os.environ.get('ROS_DISTRO')
    if distro:
        candidates.append('/opt/ros/%s/etc/ros/python_logging.conf' % distro)
    try:
        for name in sorted(os.listdir('/opt/ros')):
            candidates.append('/opt/ros/%s/etc/ros/python_logging.conf' % name)
    except Exception:
        pass
    for path in candidates:
        if os.path.isfile(path):
            os.environ['ROS_PYTHON_LOG_CONFIG_FILE'] = path
            return


def bootstrap_ros_paths():
    """把 ROS / catkin 的 python 路径挂进 sys.path，省掉"必须先 source"。

    优先级：devel（我们自己的消息包） > /opt/ros/<distro>（rospy 等）。
    返回实际新增的路径列表。
    """
    dirs = []
    # 1) 本工作区 devel（ros_actor_cmd_pose_plugin_msgs / actor_msgs 等只在这里）
    for sub in (('devel', 'lib', 'python3', 'dist-packages'),
                ('devel', 'lib', 'python3', 'site-packages')):
        dirs.append(os.path.join(WORKSPACE_DIR, *sub))
    # 2) CMAKE_PREFIX_PATH 里的前缀（catkin install 空间）
    for prefix in os.environ.get('CMAKE_PREFIX_PATH', '').split(os.pathsep):
        if prefix:
            dirs.append(os.path.join(prefix, 'lib', 'python3', 'dist-packages'))
    # 3) 系统 ROS：rospy/rosgraph/roslib 在这里，不 source 就没有
    ros_root = os.environ.get('ROS_ROOT')
    if ros_root:
        # /opt/ros/noetic/share/ros -> /opt/ros/noetic
        dirs.append(os.path.join(os.path.dirname(os.path.dirname(ros_root)),
                                 'lib', 'python3', 'dist-packages'))
    distro = os.environ.get('ROS_DISTRO')
    if distro:
        dirs.append('/opt/ros/%s/lib/python3/dist-packages' % distro)
    try:
        for name in sorted(os.listdir('/opt/ros'), reverse=True):
            dirs.append('/opt/ros/%s/lib/python3/dist-packages' % name)
    except Exception:
        pass

    added = []
    for path in reversed(dirs):            # 反向插入 => 保持 dirs 的先后优先级
        if not path:
            continue
        path = os.path.abspath(path)
        if os.path.isdir(path) and path not in sys.path:
            sys.path.insert(0, path)
            added.append(path)
    return added


class _Timeout(Exception):
    pass


def _call_with_timeout(fn, timeout):
    """在守护线程里调用 fn，超时抛 _Timeout（防止 master 不可达时挂死）。"""
    box = {}

    def runner():
        try:
            box['value'] = fn()
        except BaseException as exc:        # noqa: BLE001 - 原样带回来
            box['error'] = exc

    thread = threading.Thread(target=runner)
    thread.daemon = True
    thread.start()
    thread.join(timeout)
    if thread.is_alive():
        raise _Timeout('调用超过 %.1fs 未返回' % timeout)
    if 'error' in box:
        raise box['error']
    return box.get('value')


def load_ros():
    """按需 import ROS 库，返回 bundle（缺什么都不抛，交给调用方降级）。

    只用标准库 + rospy/rosgraph/roslib；业务消息包一概不 import。
    """
    added = bootstrap_ros_paths()
    ensure_ros_python_logging()
    bundle = {
        'rospy': None, 'rosgraph': None, 'roslib_message': None,
        'py_paths_added': added, 'errors': [],
    }
    try:
        import rosgraph as _rosgraph
        bundle['rosgraph'] = _rosgraph
    except Exception as exc:
        bundle['errors'].append('rosgraph: %s' % exc)
        try:
            import rosmaster as _rosmaster        # noqa: F401 - rosmaster 会带上 rosgraph
            import rosgraph as _rosgraph
            bundle['rosgraph'] = _rosgraph
        except Exception as exc2:
            bundle['errors'].append('rosmaster: %s' % exc2)
    try:
        import rospy as _rospy
        bundle['rospy'] = _rospy
    except Exception as exc:
        bundle['errors'].append('rospy: %s' % exc)
    try:
        import roslib.message as _roslib_message
        bundle['roslib_message'] = _roslib_message
    except Exception as exc:
        bundle['errors'].append('roslib.message: %s' % exc)
    return bundle


def resolve_message_class(roslib_message, topic_type):
    """按 'pkg/Msg' 字符串拿消息类；拿不到返回 None（调用方降级为只看帧数）。"""
    if not roslib_message or not topic_type:
        return None
    try:
        return roslib_message.get_message_class(topic_type)
    except Exception:
        return None


def decode_anymsg(anymsg, message_class):
    """把 rospy.AnyMsg（只有 _buff）反序列化成真消息；失败返回 None。"""
    if anymsg is None or message_class is None:
        return None
    try:
        obj = message_class()
        obj.deserialize(anymsg._buff)
        return obj
    except Exception:
        return None


# ---------------------------------------------------------------------------
# 探测：话题清单 / master 图 / 收帧
# ---------------------------------------------------------------------------

def build_entries(num_drones, vehicle_type):
    """构造待探测话题清单（逐机 + 全局）。"""
    entries = []
    for drone in range(num_drones):
        for spec in DRONE_TOPICS:
            entries.append({
                'topic': '/%s_%d/%s' % (vehicle_type, drone, spec['suffix']),
                'role': spec['role'],
                'drone': drone,
                'probe': spec['probe'],
                'desc': spec['desc'],
                'expected_type': spec['expected_type'],
                'publishers': [], 'subscribers': [],
                'registered_type': None, 'wire_type': None,
                'msg_type': spec['expected_type'],
                'exists': False, 'known': False,
                'frames': None, 'frame_ok': None, 'frame_attempted': False,
                'note': '',
            })
    for spec in GLOBAL_TOPICS:
        entries.append({
            'topic': spec['topic'],
            'role': spec['role'],
            'drone': None,
            'probe': spec['probe'],
            'desc': spec['desc'],
            'expected_type': spec['expected_type'],
            'publishers': [], 'subscribers': [],
            'registered_type': None, 'wire_type': None,
            'msg_type': spec['expected_type'],
            'exists': False, 'known': False,
            'frames': None, 'frame_ok': None, 'frame_attempted': False,
            'note': '',
        })
    return entries


def get_system_state(master):
    """返回 (pub_map, sub_map)；master 不支持时给空表。"""
    pub_map, sub_map = {}, {}
    pubs, subs, _services = master.getSystemState()
    for topic, nodes in pubs:
        pub_map[topic] = sorted(nodes)
    for topic, nodes in subs:
        sub_map[topic] = sorted(nodes)
    return pub_map, sub_map


def get_topic_types(master):
    """返回 {topic: 'pkg/Msg'}；老 master 不支持时给空表。"""
    types = {}
    raw = master.getTopicTypes()
    for item in raw:
        if isinstance(item, (list, tuple)) and len(item) >= 2:
            types[item[0]] = item[1]
    return types


def annotate_entries(entries, pub_map, sub_map, type_map):
    """把 master 侧的发布者/订阅者/类型填进清单。"""
    for entry in entries:
        topic = entry['topic']
        entry['publishers'] = pub_map.get(topic, [])
        entry['subscribers'] = sub_map.get(topic, [])
        entry['registered_type'] = type_map.get(topic)
        entry['known'] = bool(entry['publishers'] or entry['subscribers']
                              or entry['registered_type'])
        entry['exists'] = bool(entry['publishers'])       # 画像判定只看"有发布者"
        if entry['registered_type']:
            entry['msg_type'] = entry['registered_type']


def probe_frames(rospy, entries, timeout, roslib_message):
    """真订阅一帧：所有待验证话题共享一个 timeout 窗口。

    返回 extra 信息（tf 采样结果）。任何异常都吞掉——自检脚本不能因为收帧失败而崩。
    """
    extra = {'probe_window_s': timeout, 'tf_child_frames': [], 'tf_parent_frames': [],
             'tf_decoded': 0, 'tf_note': ''}
    wait_list = [e for e in entries if e['probe'] == 'frame' and e['exists']]
    for entry in entries:
        if entry['probe'] == 'frame' and not entry['exists']:
            entry['frames'] = 0
            entry['frame_ok'] = False
            entry['frame_attempted'] = False
            entry['note'] = '无发布者，未建订阅'
    if not wait_list:
        return extra
    if rospy is None:
        for entry in wait_list:
            entry['frame_attempted'] = False
            entry['note'] = 'rospy 不可用，未做收帧验证'
        return extra

    counts = {}
    raw_first = {}
    tf_child = set()
    tf_parent = set()
    lock = threading.Lock()
    state = {'tf_decoded': 0}

    tf_class = resolve_message_class(roslib_message, 'tf2_msgs/TFMessage')

    def make_callback(topic):
        def _callback(msg):
            with lock:
                counts[topic] = counts.get(topic, 0) + 1
                if topic not in raw_first:
                    raw_first[topic] = msg
                if topic == '/tf' and state['tf_decoded'] < TF_DESER_CAP:
                    state['tf_decoded'] += 1
                    if tf_class is not None:
                        obj = decode_anymsg(msg, tf_class)
                        if obj is not None:
                            for tr in obj.transforms:
                                tf_child.add(tr.child_frame_id)
                                tf_parent.add(tr.header.frame_id)
        return _callback

    subs = []
    try:
        for entry in wait_list:
            subs.append(rospy.Subscriber(entry['topic'], rospy.AnyMsg,
                                         make_callback(entry['topic']), queue_size=1))
    except Exception as exc:
        for entry in wait_list:
            entry['note'] = '订阅失败：%s' % exc
        return extra

    deadline = time.time() + timeout
    while time.time() < deadline:
        with lock:
            if all(counts.get(e['topic'], 0) > 0 for e in wait_list):
                break
        time.sleep(0.02)                     # 用墙上时钟，避免 use_sim_time + paused 卡死

    for sub in subs:
        try:
            sub.unregister()
        except Exception:
            pass

    for entry in wait_list:
        topic = entry['topic']
        entry['frame_attempted'] = True
        entry['frames'] = counts.get(topic, 0)
        entry['frame_ok'] = entry['frames'] > 0
        if topic in raw_first:
            header = getattr(raw_first[topic], '_connection_header', None) or {}
            entry['wire_type'] = header.get('type')
            if entry['wire_type'] and entry['wire_type'] != entry['msg_type']:
                entry['note'] = '实际线上类型 %s' % entry['wire_type']
        if not entry['frame_ok']:
            entry['note'] = (entry['note'] + '; ' if entry['note'] else '') + \
                '有发布者但 %gs 内 0 帧' % timeout

    extra['tf_child_frames'] = sorted(tf_child)[:40]
    extra['tf_parent_frames'] = sorted(tf_parent)[:40]
    extra['tf_decoded'] = state['tf_decoded']
    if tf_class is None:
        extra['tf_note'] = 'tf2_msgs 不可用，未解析 /tf 内容'
    # 首帧原文留给 /gazebo/model_states 反序列化用（不放进 json，AnyMsg 不可序列化）
    extra['raw_first'] = raw_first
    return extra


# ---------------------------------------------------------------------------
# 画像判定 / 缺失统计 / 布局校验
# ---------------------------------------------------------------------------

def drones_with(entries, role):
    return sorted(e['drone'] for e in entries if e['role'] == role and e['exists'])


def classify_profile(entries):
    """按"存在性"判定环境画像，返回 (profile, criteria 列表)。

    判定优先级（本地画像优先，因为它严格包含官方能力）：
      1) 有 scan 且 有 realsense color+depth        -> local-lidar-depth
      2) 有 scan 且 无 realsense color/depth        -> lidar-only
      3) 无 scan/realsense 且 有 cgo3               -> official
      4) 其它                                        -> unknown
    """
    has_scan = bool(drones_with(entries, 'scan'))
    has_color = bool(drones_with(entries, 'realsense_color'))
    has_depth = bool(drones_with(entries, 'realsense_depth'))
    has_cgo3 = bool(drones_with(entries, 'cgo3_camera'))

    ok_official = (not has_scan) and (not has_color) and (not has_depth) and has_cgo3
    ok_local = has_scan and has_color and has_depth
    ok_lidar = has_scan and not has_color and not has_depth
    if ok_local:
        profile = PROFILE_LOCAL
    elif ok_lidar:
        profile = PROFILE_LIDAR
    elif ok_official:
        profile = PROFILE_OFFICIAL
    else:
        profile = PROFILE_UNKNOWN

    total = _drone_total(entries)
    counts = lambda role: len(drones_with(entries, role))          # noqa: E731
    mark = lambda flag: '✓' if flag else '✗'                        # noqa: E731
    verdict = lambda matched: ('成立  <- 判定结果' if matched else '不成立')   # noqa: E731

    criteria = [
        'official          cgo3有 %s(%d/%d) | scan无 %s(%d/%d有) | color无 %s(%d/%d有)'
        ' | depth无 %s(%d/%d有)  -> %s'
        % (mark(has_cgo3), counts('cgo3_camera'), total,
           mark(not has_scan), counts('scan'), total,
           mark(not has_color), counts('realsense_color'), total,
           mark(not has_depth), counts('realsense_depth'), total, verdict(ok_official)),
        'local-lidar-depth scan %s(%d/%d) | color %s(%d/%d) | depth %s(%d/%d)'
        '                                     -> %s'
        % (mark(has_scan), counts('scan'), total,
           mark(has_color), counts('realsense_color'), total,
           mark(has_depth), counts('realsense_depth'), total, verdict(ok_local)),
        'lidar-only        scan %s(%d/%d) | color %s(%d/%d) | depth %s(%d/%d)'
        '                                     -> %s'
        % (mark(has_scan), counts('scan'), total,
           mark(has_color), counts('realsense_color'), total,
           mark(has_depth), counts('realsense_depth'), total, verdict(ok_lidar)),
    ]
    if profile == PROFILE_UNKNOWN:
        criteria.append('unknown           以上三组判据都不成立 -> 判定结果')
    return profile, criteria


def _drone_total(entries):
    total = 0
    for entry in entries:
        if entry['drone'] is not None and entry['drone'] >= total:
            total = entry['drone'] + 1
    return total


def critical_roles_for(profile, use_sim_time):
    roles = set(CRITICAL_ROLES.get(profile, CRITICAL_ROLES[PROFILE_UNKNOWN]))
    if use_sim_time:
        roles.add('clock')
    return roles


def collect_missing(entries, critical_roles):
    """返回 (missing_critical 扁平列表, by_role 明细)。"""
    flat = []
    by_role = {}
    for role in ROLE_ORDER:
        role_entries = [e for e in entries if e['role'] == role]
        if not role_entries:
            continue
        present = [e for e in role_entries if e['exists']]
        missing = [e for e in role_entries if not e['exists']]
        is_critical = role in critical_roles
        by_role[role] = {
            'critical': is_critical,
            'total': len(role_entries),
            'present_count': len(present),
            'present_indices': sorted(e['drone'] for e in present if e['drone'] is not None),
            'missing_indices': sorted(e['drone'] for e in missing if e['drone'] is not None),
            'missing_topics': [e['topic'] for e in missing],
            'exists': bool(present),
            'nodes_failing': IMPACT_BY_ROLE.get(role, ('', ''))[0],
            'consequence': IMPACT_BY_ROLE.get(role, ('', ''))[1],
        }
        if is_critical and missing:
            flat.extend(e['topic'] for e in missing)
    return sorted(set(flat)), by_role


def expected_spawn(layout, index):
    origin = LOCAL_ORIGIN if layout == 'local' else OFFICIAL_ORIGIN
    return (origin[0] + SPAWN_STRIDE_X * (index % 2),
            origin[1] + SPAWN_STRIDE_Y * (index // 2))


def extract_model_xy(model_states_msg):
    """gazebo_msgs/ModelStates -> {name: (x, y, z)}；解析失败返回 {}。"""
    result = {}
    try:
        names = list(model_states_msg.name)
        poses = list(model_states_msg.pose)
    except Exception:
        return result
    for idx, name in enumerate(names):
        if idx >= len(poses):
            break
        pos = poses[idx].position
        result[str(name)] = (float(pos.x), float(pos.y), float(pos.z))
    return result


def check_spawn_layout(entries, model_xy, num_drones, vehicle_type, tol):
    """把 /gazebo/model_states 的实际坐标与本地/官方两套布局比对。"""
    info = {
        'checked': False,
        'available': bool(model_xy),
        'source': '/gazebo/model_states',
        'tolerance_m': tol,
        'num_drones': num_drones,
        'layout': PROFILE_UNKNOWN,
        'matches_local': False,
        'matches_official': False,
        'max_error_local_m': None,
        'max_error_official_m': None,
        'drones_found': [],
        'drones_expected': list(range(num_drones)),
        'actual': {},
        'expected_local': {},
        'expected_official': {},
        'matched_model_names': {},
        'source_topic_exists': bool([e for e in entries
                                     if e['topic'] == '/gazebo/model_states' and e['exists']]),
    }
    if not model_xy:
        info['note'] = ('未拿到 /gazebo/model_states 数据：'
                        '无发布者，或仿真 paused / 无真值插件 → 出生点布局无法判定')
        return info

    # 机型名匹配：优先 <vehicle_type>_<i>，再兜底官方/本地 SDF 名
    used_names = set()
    for index in range(num_drones):
        for name in ('%s_%d' % (vehicle_type, index),
                     'typhoon_h480_%d' % index,
                     'typhoon_h480_zzufly_%d' % index):
            if name in model_xy and name not in used_names:
                info['actual'][index] = model_xy[name]
                info['matched_model_names'][index] = name
                used_names.add(name)
                break
    if not info['actual']:
        # 兜底：扫描形如 typhoon_h480[_xxx]_<i> 的模型名
        pattern = re.compile(r'^(?:typhoon_h480(?:_[A-Za-z0-9]+)?)_(\d+)$')
        for name in sorted(model_xy):
            match = pattern.match(name)
            if match:
                index = int(match.group(1))
                if index < num_drones and index not in info['actual']:
                    info['actual'][index] = model_xy[name]
                    info['matched_model_names'][index] = name
    if not info['actual']:
        info['note'] = ('/gazebo/model_states 里没有 typhoon_h480_* 模型'
                        '（当前模型：%s）' % ', '.join(sorted(model_xy)[:8]))
        return info

    info['checked'] = True
    info['drones_found'] = sorted(info['actual'])
    local_err, official_err = [], []
    for index in sorted(info['actual']):
        x, y, _z = info['actual'][index]
        exp_local = expected_spawn('local', index)
        exp_official = expected_spawn('official', index)
        info['expected_local'][index] = exp_local
        info['expected_official'][index] = exp_official
        local_err.append(math.hypot(x - exp_local[0], y - exp_local[1]))
        official_err.append(math.hypot(x - exp_official[0], y - exp_official[1]))
    info['max_error_local_m'] = round(max(local_err), 3)
    info['max_error_official_m'] = round(max(official_err), 3)
    info['matches_local'] = info['max_error_local_m'] <= tol
    info['matches_official'] = info['max_error_official_m'] <= tol
    if info['matches_local']:
        info['layout'] = 'local'
    elif info['matches_official']:
        info['layout'] = 'official'
    else:
        info['layout'] = PROFILE_UNKNOWN
    info['complete'] = len(info['actual']) == num_drones
    if not info['complete']:
        info['note'] = ('只匹配到 %d/%d 架模型，布局判定基于已找到的架次'
                        % (len(info['actual']), num_drones))
    return info


# ---------------------------------------------------------------------------
# 报告渲染
# ---------------------------------------------------------------------------

def _fmt_actual(xyz):
    return '(%.3f, %.3f, %.3f)' % xyz


def _fmt_xy(xy):
    return '(%.3f, %.3f)' % xy


def build_actions(result):
    """根据结果给可执行的建议动作。"""
    actions = []
    profile = result['profile']
    layout = result['spawn_layout']
    missing_roles = [r for r, d in result['missing_by_role'].items()
                     if d['critical'] and d['present_count'] < d['total']]
    use_sim_time = result['ros'].get('use_sim_time')

    if layout.get('layout') == 'official':
        actions.append(
            '出生点是官方 (0,-3) 系：起栈前必须把 spawn_origin_x 覆盖为 0.0（三处保持一致：'
            'pose_init 的 pose_transformer_node、area_mission_node、scan_map_fuser；'
            'launch 里传 spawn_origin_x:=0.0，例如 pose.launch / auto_explore_swarm.launch）。'
            '不改就会整体偏 -17 m，裁判"误差<1m"判据必失败。')
    elif layout.get('layout') == PROFILE_UNKNOWN and layout.get('checked'):
        actions.append('出生点既不匹配本地 (-17,-3) 系也不匹配官方 (0,-3) 系：'
                       '请人工确认坐标（见上面 actual 列），再决定 spawn_origin_x/y 该取什么。')
    elif not layout.get('checked'):
        actions.append('出生点布局未能判定（/gazebo/model_states 无数据）：'
                       '先确认 gzserver + gazebo_ros 真值插件在跑、仿真没暂停；'
                       '拿不到真值就按官方/本地布局各自试一次再定参数。')

    if 'scan' in missing_roles:
        actions.append('缺 2D 激光：不要起 area_mission 巡逻（A* 会全 NO PATH）、'
                       'start_sim_only.sh 健康门会 240s 超时 exit 1；'
                       '先确认加载的 SDF 带 2D 激光（本地 typhoon_h480_zzufly）。')
    if 'realsense_color' in missing_roles or 'realsense_depth' in missing_roles:
        actions.append('缺 Realsense：yolo11n.py / bbox2coord_node.py 链路不可用'
                       '（无检测、无世界坐标、裁判拿不到 /actor_*_info）。'
                       '官方基础型只有 CGO3 相机，需把 yolo 订阅 remap 到 '
                       '<V>_i/cgo3_camera/image_raw 并接受"没有深度"这一限制。')
    if 'mavros_pose' in missing_roles or 'mavros_state' in missing_roles:
        actions.append('缺 MAVROS 位姿/状态：等官方 PX4+MAVROS 起来'
                       '（connected=True）再起栈；tracking_node 会在 waitForPose() 永久阻塞。')
    if 'gazebo_model_states' in missing_roles:
        actions.append('缺 /gazebo/model_states：高度真值与 actor 消除判定 fail-open，'
                       '建议开 TRUTH_FALLBACK 并接受精度下降。')
    if 'left_actors' in missing_roles:
        actions.append('缺 /left_actors：确认裁判端（bash start_referee.sh）已启动，'
                       '否则追踪会抱人不放。')
    if 'tf' in missing_roles:
        actions.append('缺 /tf：scan_map_fuser 会全 continue、bbox2coord 外参解算失败；'
                       '先起 transform_tree / pose_init 再起感知。')
    if result['ros'].get('clock_missing_with_sim_time'):
        actions.append('use_sim_time=true 但没有 /clock：定时器与超时全部失序。'
                       '要么让 gazebo_ros 发 /clock（确认 gzserver 在跑），'
                       '要么把 /use_sim_time 设为 false。')
    if result.get('sim_paused_suspect'):
        actions.append('多个高 freq 话题"有发布者但 0 帧"且 /clock 无数据：'
                       '仿真很可能处于 paused —— 在 Gazebo 里按播放，或检查 '
                       'empty_world.launch 的 paused 参数。')
    if not use_sim_time and result['ros'].get('clock_topic_exists'):
        actions.append('有 /clock 但 /use_sim_time 不是 true：定时器走墙上时钟，'
                       '与仿真时间不同步，建议统一为 true。')
    if not actions:
        actions.append('环境与 zzu_fly 本地默认一致：可以直接 '
                       'bash start_sim_only.sh（或用你自己的接线方式）起算法栈。')
    return actions


def render_text(result):
    lines = []
    add = lines.append
    ros = result['ros']
    profile = result['profile']

    add('=' * 78)
    add('zzu_fly 环境画像自检（只读）  ——  check_env_profile.py')
    add('=' * 78)

    # ---- [0] 环境 ----
    add('[0] ROS 环境')
    add('  ROS master     : %s%s' % (ros.get('master_uri'),
                                     ('（pid %s）' % ros.get('master_pid'))
                                     if ros.get('master_pid') else ''))
    add('  python         : %s %s' % (result['python']['executable'],
                                      result['python']['version']))
    add('  rospy/rosgraph : %s / %s'
        % ('OK' if ros.get('rospy') else '缺失',
           'OK' if ros.get('rosgraph') else '缺失'))
    add('  use_sim_time   : %s   /clock 有发布者: %s'
        % ('(未知)' if ros.get('use_sim_time') is None else ros.get('use_sim_time'),
           ros.get('clock_topic_exists')))
    add('  探测参数       : num_drones=%d  vehicle_type=%s  timeout=%gs  画像来源=%s'
        % (result['num_drones'], result['vehicle_type'], result['timeout'],
           {'auto': 'auto（自动判定）', 'forced': 'forced（--profile 强制）'}.get(
               result['profile_source'], result['profile_source'])))
    if ros.get('py_paths_added'):
        add('  自动挂载的 python 路径: %s' % ', '.join(ros['py_paths_added']))
    for note in ros.get('notes') or []:
        add('  提示           : %s' % note)
    if ros.get('degraded'):
        add('  降级           : %s' % ros['degraded'])
    add('')

    # ---- [1] 话题探测 ----
    add('[1] 话题探测（pub=有发布者的节点数, sub=订阅者数, frames=%gs 内收到的帧数）'
        % result['timeout'])
    add('  %-5s %-19s %-62s %-4s %-4s %-6s %s'
        % ('drone', 'role', 'topic', 'pub', 'sub', 'frames', 'type'))
    for entry in result['topic_list']:
        if entry['probe'] == 'both' or not entry['frame_attempted']:
            frames_s = '-'
        elif entry['frames'] is None:
            frames_s = 'n/a'
        else:
            frames_s = str(entry['frames'])
        add('  %-5s %-19s %-62s %-4d %-4d %-6s %s'
            % ('-' if entry['drone'] is None else entry['drone'],
               entry['role'], entry['topic'], len(entry['publishers']),
               len(entry['subscribers']), frames_s,
               entry['msg_type'] or '?'))
    add('')
    summary = result['roles']
    add('  汇总（有发布者/应收架数，frames 为收到帧的架数）：')
    for role in ROLE_ORDER:
        info = summary.get(role)
        if not info:
            continue
        frames_ok = info['frames_ok_count']
        add('    %-19s 发布者 %d/%d   收到帧 %-3s   %s'
            % (role, info['present_count'], info['total'],
               ('%d' % frames_ok) if info['frame_attempted'] else 'n/a',
               ROLE_ZH.get(role, '')))
    add('')

    # ---- [2] 画像判定 ----
    add('[2] 环境画像判定')
    add('  画像：%s' % PROFILE_ZH.get(profile, profile))
    if result['profile_source'] == 'forced' and result['profile_detected'] != profile:
        add('  注意：画像被 --profile 强制为 %s，探测结果是 %s'
            % (profile, result['profile_detected']))
    add('  判据（✓=条件满足；括号内是"有发布者"的架数）：')
    for line in result['profile_criteria']:
        add('    %s' % line)
    if profile == PROFILE_OFFICIAL:
        add('  → 只订阅模式：官方基础型只有 CGO3 单目相机，我们的激光/深度感知栈会静默失效')
    add('')

    # ---- [3] 影响面诊断 ----
    add('[3] 影响面诊断（缺失的必需话题 → 谁失效 → 后果）')
    for role in ROLE_ORDER:
        info = summary.get(role)
        if not info:
            continue
        if info['present_count'] >= info['total']:
            add('  ✓ %-19s %d/%d 架有发布者%s'
                % (role, info['present_count'], info['total'],
                   '  [关键]' if info['critical'] else ''))
            continue
        if info['critical']:
            tag = '[关键]'
        else:
            tag = '[提示]'
        miss = info['missing_indices']
        miss_s = ('缺机号 %s' % ','.join(str(i) for i in miss)) if miss else '本话题无发布者'
        add('  ✗ %-19s %d/%d 架有发布者  %s  %s'
            % (role, info['present_count'], info['total'], tag, miss_s))
        if role in ('scan', 'realsense_color', 'realsense_depth') and \
                not info['critical'] and profile == PROFILE_OFFICIAL:
            add('      注：官方画像下属于结构性缺失（官方机型就没这路传感器），'
                '但我们的算法栈仍然依赖它')
        add('      失效节点：%s' % info['nodes_failing'])
        add('      后果    ：%s' % info['consequence'])
    add('')

    # ---- [4] 出生点布局 ----
    layout = result['spawn_layout']
    add('[4] 出生点布局校验（%s）' % layout['source'])
    if not layout.get('checked'):
        add('  ✗ 未能判定：%s' % layout.get('note', '无数据'))
    else:
        add('  %-5s %-24s %-24s %-17s %-17s %s'
            % ('drone', 'model', 'actual(x,y,z)', 'local期望', 'official期望',
               'err(local/official)'))
        for index in layout['drones_found']:
            actual = layout['actual'][index]
            exp_l = layout['expected_local'][index]
            exp_o = layout['expected_official'][index]
            err_l = math.hypot(actual[0] - exp_l[0], actual[1] - exp_l[1])
            err_o = math.hypot(actual[0] - exp_o[0], actual[1] - exp_o[1])
            add('  %-5d %-24s %-24s %-17s %-17s %.2f / %.2f'
                % (index, layout['matched_model_names'].get(index, '?'),
                   _fmt_actual(actual), _fmt_xy(exp_l), _fmt_xy(exp_o), err_l, err_o))
        add('  判定：%s   （最大误差 local=%.3f m / official=%.3f m，容差 %.1f m）'
            % (layout['layout'], layout['max_error_local_m'] or 0.0,
               layout['max_error_official_m'] or 0.0, layout['tolerance_m']))
        if layout.get('note'):
            add('  注：%s' % layout['note'])
        if layout['layout'] == 'local':
            add('  ✓ 与 zzu_fly 本地默认布局一致（spawn_origin_x=-17.0 可用）')
        elif layout['layout'] == 'official':
            add('')
            add('  ' + '!' * 74)
            add('  !! 高亮警告：出生点 = 官方 robocup.launch 布局（(0,-3) 系），'
                '不是本地 (-17,-3) 系')
            add('  !! zzu_fly 侧出生点偏移默认值是本地布局：')
            add('  !!   src/pose_init/src/pose_init.cpp:98            spawn_origin_x = -17.0')
            add('  !!   src/auto_explore/src/area_mission_node.cpp:75 spawn_origin_x 默认 -17.0')
            add('  !!   src/auto_explore/src/scan_map_fuser.cpp:50    spawn_origin_x 默认 -17.0')
            add('  !! 不覆盖就会让播报给裁判的全局坐标整体偏 -17 m，'
                '裁判"误差 < 1 m"判据必失败。')
            add('  !! 处置：三处一致地把 spawn_origin_x 覆盖为 0.0'
                '（launch 传 spawn_origin_x:=0.0 或 rosparam set .../spawn_origin_x 0）。')
            add('  ' + '!' * 74)
    add('')

    # ---- [5] 结论 ----
    add('[5] 结论')
    add('  结论：%s' % result['conclusion'])
    add('  建议动作：')
    for idx, action in enumerate(result['suggested_actions'], 1):
        add('    %d) %s' % (idx, action))
    add('')
    add('  退出码口径：0 = 关键话题齐全 且 出生点布局 == 本地默认；'
        '1 = 缺关键话题或布局不一致；2 = 连不上 ROS master')
    add('  本次退出码：%d（ok=%s，耗时 %.2fs）'
        % (result['exit_code'], result['ok'], result['elapsed_s']))
    return '\n'.join(lines)


def render_json(result):
    return json.dumps(result, ensure_ascii=False, indent=2, sort_keys=False)


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------

def parse_args(argv):
    epilog = (
        '画像说明：\n'
        '  official          有 cgo3_camera/image_raw，且没有 scan / realsense\n'
        '  local-lidar-depth scan 与 realsense/depth_camera/{color,depth} 都有\n'
        '  lidar-only        只有 scan（官方 typhoon_h480_2d_lidar）\n'
        '  unknown           其它组合\n'
        '\n'
        '退出码：0 关键话题齐全且布局=本地默认；1 缺关键话题或布局不一致；2 连不上 ROS master\n'
    )
    parser = argparse.ArgumentParser(
        prog='check_env_profile.py',
        description='zzu_fly 环境画像自检（只读）：探测 ROS 图 → 判定 official/local 画像 → '
                    '报告缺失话题的失效面 → 校验出生点布局。',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=epilog)
    parser.add_argument('--num-drones', type=int, default=6, metavar='N',
                        help='无人机架数（默认 6）')
    parser.add_argument('--vehicle-type', default='typhoon_h480', metavar='V',
                        help='机型 / 命名空间前缀（默认 typhoon_h480）')
    parser.add_argument('--timeout', type=float, default=5.0, metavar='SEC',
                        help='收帧窗口秒数，全部话题共享（默认 5）')
    parser.add_argument('--json', action='store_true',
                        help='输出 JSON（stdout 只有 JSON，人类提示走 stderr）')
    parser.add_argument('--profile', choices=['auto', 'official', 'local'], default='auto',
                        help='auto=自动判定（默认）；official/local=强制画像，'
                             '只影响"哪些话题算关键"，不改变退出码口径')
    args = parser.parse_args(argv)
    if args.num_drones < 1 or args.num_drones > 64:
        parser.error('--num-drones 必须在 1..64')
    if args.timeout <= 0 or args.timeout > 600:
        parser.error('--timeout 必须在 (0, 600]')
    if not args.vehicle_type or '/' in args.vehicle_type:
        parser.error('--vehicle-type 非法：%r' % args.vehicle_type)
    return args


def json_skeleton(args):
    """即使失败也要给全字段（profile/topics/missing_critical/spawn_layout/ok）。"""
    return {
        'ok': False,
        'exit_code': 2,
        'profile': None,
        'profile_detected': None,
        'profile_source': 'auto' if args.profile == 'auto' else 'forced',
        'num_drones': args.num_drones,
        'vehicle_type': args.vehicle_type,
        'timeout': args.timeout,
        'python': {
            'executable': sys.executable,
            'version': '%d.%d.%d' % sys.version_info[:3],
        },
        'ros': {
            'master_uri': os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'),
            'master_pid': None,
            'rospy': False,
            'rosgraph': False,
            'roslib_message': False,
            'use_sim_time': None,
            'clock_topic_exists': False,
            'clock_missing_with_sim_time': False,
            'py_paths_added': [],
            'notes': [],
            'degraded': None,
        },
        'topics': {},
        'topic_list': [],
        'roles': {},
        'missing_critical': [],
        'missing_by_role': {},
        'spawn_layout': {
            'checked': False,
            'available': False,
            'source': '/gazebo/model_states',
            'layout': PROFILE_UNKNOWN,
            'matches_local': False,
            'matches_official': False,
            'tolerance_m': SPAWN_TOL_M,
        },
        'impacts': [{'role': r, 'nodes': n, 'consequence': c} for r, n, c in IMPACTS],
        'conclusion': '',
        'suggested_actions': [],
        'elapsed_s': 0.0,
        'error': None,
    }


def render_failure_text(result):
    """连不上 master / ROS 库缺失时的简短失败报告（不打印空表）。"""
    ros = result['ros']
    lines = [
        '=' * 78,
        'zzu_fly 环境画像自检（只读）  ——  check_env_profile.py',
        '=' * 78,
        '[失败] exit=%d' % result['exit_code'],
        '  ROS master     : %s' % ros.get('master_uri'),
        '  python         : %s %s' % (result['python']['executable'],
                                      result['python']['version']),
        '  rospy/rosgraph : %s / %s' % ('OK' if ros.get('rospy') else '缺失',
                                       'OK' if ros.get('rosgraph') else '缺失'),
        '  ROS_LOG_DIR    : %s' % (os.environ.get('ROS_LOG_DIR') or '(默认 ~/.ros/log)'),
        '',
        '  %s' % (result.get('error') or '未知错误'),
        '',
        '  结论：%s' % (result.get('error') or '自检失败'),
        '  退出码口径：0 = 关键话题齐全 且 出生点布局 == 本地默认；'
        '1 = 缺关键话题或布局不一致；2 = 连不上 ROS master',
    ]
    return '\n'.join(lines)


def finish(result, args):
    """统一出口：算退出码、渲染、打印。"""
    result['ok'] = (result['exit_code'] == 0)
    if args.json:
        sys.stdout.write(render_json(result) + '\n')
    elif result.get('error'):
        sys.stdout.write(render_failure_text(result) + '\n')
    else:
        sys.stdout.write(render_text(result) + '\n')
    sys.stdout.flush()
    if result.get('error'):
        sys.stderr.write('[check_env_profile] %s\n' % result['error'].replace('\n', ' '))
    return result['exit_code']


def _fail(result, args, exit_code, message):
    result['exit_code'] = exit_code
    result['ok'] = False
    result['error'] = message
    result['conclusion'] = message
    result['suggested_actions'] = []
    return finish(result, args)


def main(argv=None):
    args = parse_args(argv)
    started = time.time()
    result = json_skeleton(args)

    # ---- 0) 环境准备（必须在 import rospy 之前）----
    _log_dir, log_note = ensure_ros_log_dir()
    if log_note:
        result['ros']['notes'].append(log_note)

    bootstrap_started = time.time()
    ros = load_ros()
    result['ros']['rospy'] = ros['rospy'] is not None
    result['ros']['rosgraph'] = ros['rosgraph'] is not None
    result['ros']['roslib_message'] = ros['roslib_message'] is not None
    result['ros']['py_paths_added'] = ros['py_paths_added']
    result['ros']['bootstrap_s'] = round(time.time() - bootstrap_started, 3)

    if ros['rosgraph'] is None:
        return _fail(
            result, args, 2,
            '未找到 ROS Python 库（rosgraph / rospy 都 import 不到）：%s\n'
            '  → 先 source 系统 ROS（例如 source /opt/ros/noetic/setup.bash），'
            '或确认 /opt/ros/<distro>/lib/python3/dist-packages 存在且 python3 版本匹配\n'
            '  → 已尝试自动挂载的路径：%s'
            % ('; '.join(ros['errors']) or '(none)',
               ', '.join(ros['py_paths_added']) or '(无)'))

    master_uri = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
    result['ros']['master_uri'] = master_uri

    # ---- 1) master 连通性（不可达 → 2，绝不抛栈）----
    master = None
    try:
        master = ros['rosgraph'].Master('/check_env_profile')
        master_pid = _call_with_timeout(master.getPid, MASTER_CALL_TIMEOUT)
        result['ros']['master_pid'] = master_pid
    except _Timeout as exc:
        return _fail(result, args, 2,
                     '连不上 ROS master（%s）：%s。\n'
                     '  → 确认 roscore / 组委会仿真已启动，且 ROS_MASTER_URI 指向正确的主机端口'
                     % (master_uri, exc))
    except Exception as exc:
        return _fail(result, args, 2,
                     '连不上 ROS master（%s）：%s: %s。\n'
                     '  → 确认 roscore / 组委会仿真已启动，且 ROS_MASTER_URI 指向正确的主机端口'
                     % (master_uri, type(exc).__name__, exc))

    # ---- 2) 读 ROS 图 ----
    try:
        pub_map, sub_map = get_system_state(master)
    except Exception as exc:
        return _fail(result, args, 2,
                     'master 可达但 getSystemState() 失败（%s: %s）：ROS 图读取不了，'
                     '无法判定画像' % (type(exc).__name__, exc))
    try:
        type_map = get_topic_types(master)
    except Exception as exc:
        type_map = {}
        result['ros']['notes'].append('getTopicTypes 不可用（%s），消息类型将显示为期望值或 ?'
                                      % exc)
    try:
        use_sim_time = bool(master.getParam('/use_sim_time'))
    except Exception:
        use_sim_time = None
    result['ros']['use_sim_time'] = use_sim_time

    entries = build_entries(args.num_drones, args.vehicle_type)
    annotate_entries(entries, pub_map, sub_map, type_map)

    clock_entries = [e for e in entries if e['role'] == 'clock']
    result['ros']['clock_topic_exists'] = bool(clock_entries and clock_entries[0]['exists'])

    # ---- 3) 收帧（可选；rospy 缺失则降级）----
    extra = {'probe_window_s': args.timeout}
    if ros['rospy'] is not None:
        try:
            ros['rospy'].init_node('check_env_profile', anonymous=True, disable_signals=True)
        except Exception as exc:
            return _fail(result, args, 2,
                         'rospy.init_node 失败（%s: %s）：master 可能刚断开，'
                         '或 ROS_MASTER_URI 不可用' % (type(exc).__name__, exc))
        try:
            extra = probe_frames(ros['rospy'], entries, args.timeout,
                                 ros['roslib_message'])
        except Exception as exc:
            result['ros']['notes'].append('收帧阶段异常（已忽略）：%s: %s'
                                          % (type(exc).__name__, exc))
        finally:
            try:
                ros['rospy'].signal_shutdown('check_env_profile done')
            except Exception:
                pass
    else:
        result['ros']['degraded'] = ('rospy 不可用（%s）：只做"有发布者/有订阅者"判定，'
                                     '跳过收帧验证' % ('; '.join(ros['errors']) or 'import 失败'))
        for entry in entries:
            if entry['probe'] == 'frame':
                entry['note'] = 'rospy 不可用，未做收帧验证'

    # ---- 4) 画像 / 缺失 / 布局 ----
    detected, criteria = classify_profile(entries)
    if args.profile == 'auto':
        profile = detected
        profile_source = 'auto'
    else:
        profile = PROFILE_LOCAL if args.profile == 'local' else PROFILE_OFFICIAL
        profile_source = 'forced'
    result['profile'] = profile
    result['profile_detected'] = detected
    result['profile_source'] = profile_source
    result['profile_criteria'] = criteria

    critical_roles = critical_roles_for(profile, use_sim_time)
    missing_critical, by_role = collect_missing(entries, critical_roles)
    result['missing_critical'] = missing_critical
    result['missing_by_role'] = by_role

    # 每架的 role 汇总 + 逐话题明细
    roles = {}
    for role in ROLE_ORDER:
        role_entries = [e for e in entries if e['role'] == role]
        if not role_entries:
            continue
        frame_attempted = any(e['frame_attempted'] for e in role_entries)
        roles[role] = {
            'total': len(role_entries),
            'present_count': sum(1 for e in role_entries if e['exists']),
            'critical': role in critical_roles,
            'frames_ok_count': sum(1 for e in role_entries if e['frame_ok']),
            'frame_attempted': frame_attempted,
            'missing_indices': sorted(e['drone'] for e in role_entries
                                      if not e['exists'] and e['drone'] is not None),
            'nodes_failing': IMPACT_BY_ROLE.get(role, ('', ''))[0],
            'consequence': IMPACT_BY_ROLE.get(role, ('', ''))[1],
        }
    result['roles'] = roles

    model_xy = {}
    model_entry = [e for e in entries if e['topic'] == '/gazebo/model_states']
    model_entry = model_entry[0] if model_entry else None
    # 收帧阶段留下的首帧原文里取真值（反序列化不需要节点还活着，但必须在进程内）
    raw_first = extra.get('raw_first') or {}
    raw_model_states = raw_first.get('/gazebo/model_states')
    if raw_model_states is not None:
        message_class = resolve_message_class(
            ros['roslib_message'], model_entry.get('wire_type') or 'gazebo_msgs/ModelStates')
        decoded = decode_anymsg(raw_model_states, message_class)
        if decoded is not None:
            model_xy = extract_model_xy(decoded)
        if model_entry is not None:
            model_entry['model_states_decoded'] = bool(model_xy)
    layout = check_spawn_layout(entries, model_xy, args.num_drones, args.vehicle_type,
                                SPAWN_TOL_M)
    result['spawn_layout'] = layout

    # ---- 5) 结论 / 退出码 ----
    result['ros']['clock_missing_with_sim_time'] = bool(
        use_sim_time and not result['ros']['clock_topic_exists'])

    # 仿真 paused 疑似：高 freq 话题有发布者但全部 0 帧
    hot_roles = ('realsense_color', 'scan', 'tf', 'clock')
    hot_zero = 0
    hot_total = 0
    for role in hot_roles:
        info = roles.get(role)
        if info and info['present_count'] > 0 and info['frame_attempted']:
            hot_total += info['present_count']
            hot_zero += info['present_count'] - info['frames_ok_count']
    result['sim_paused_suspect'] = bool(hot_total >= 3 and hot_zero == hot_total)

    missing_role_names = [r for r, d in by_role.items() if d['critical'] and d['missing_topics']]
    layout_ok = bool(layout.get('checked') and layout.get('layout') == 'local')
    if missing_critical or not layout_ok:
        result['exit_code'] = 1
    else:
        result['exit_code'] = 0

    concl = ['画像=%s' % profile]
    if missing_critical:
        concl.append('关键话题缺失 %d 类 / %d 个'
                     % (len(missing_role_names), len(missing_critical)))
    else:
        concl.append('关键话题齐全')
    if layout.get('checked'):
        concl.append('出生点布局=%s（local 最大误差 %s m）'
                     % (layout['layout'], layout.get('max_error_local_m')))
    else:
        concl.append('出生点布局=未能判定')
    if result['exit_code'] == 0:
        concl.append('→ 可以用默认参数起栈')
    else:
        reasons = []
        if missing_critical:
            reasons.append('缺 %s' % '/'.join(missing_role_names))
        if not layout_ok:
            reasons.append('布局与本地默认%s'
                           % ('不一致(%s)' % layout.get('layout')
                              if layout.get('checked') else '无法判定'))
        concl.append('→ 不要盲目起栈（%s）' % '；'.join(reasons))
    if result['sim_paused_suspect']:
        concl.append('（仿真疑似 paused）')
    result['conclusion'] = '；'.join(concl)

    result['suggested_actions'] = build_actions(result)
    result['topics'] = {e['topic']: {
        'role': e['role'],
        'drone': e['drone'],
        'exists': e['exists'],
        'publishers': e['publishers'],
        'subscribers': e['subscribers'],
        'frames': e['frames'],
        'frame_ok': e['frame_ok'],
        'frame_attempted': e['frame_attempted'],
        'msg_type': e['msg_type'],
        'wire_type': e['wire_type'],
        'critical': e['role'] in critical_roles,
        'note': e['note'],
    } for e in entries}
    result['topic_list'] = [{
        'topic': e['topic'], 'role': e['role'], 'drone': e['drone'],
        'probe': e['probe'], 'exists': e['exists'], 'known': e['known'],
        'publishers': e['publishers'], 'subscribers': e['subscribers'],
        'frames': e['frames'], 'frame_ok': e['frame_ok'],
        'frame_attempted': e['frame_attempted'],
        'msg_type': e['msg_type'], 'wire_type': e['wire_type'],
        'critical': e['role'] in critical_roles, 'note': e['note'],
    } for e in entries]
    result['frame_probe'] = {
        'window_s': args.timeout,
        'tf_decoded_msgs': extra.get('tf_decoded'),
        'tf_parent_frames': extra.get('tf_parent_frames'),
        'tf_child_frames': extra.get('tf_child_frames'),
        'tf_note': extra.get('tf_note', ''),
        'base_link_seen': any('base_link' in f for f in extra.get('tf_child_frames', [])),
    }
    result['elapsed_s'] = round(time.time() - started, 2)
    return finish(result, args)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.stderr.write('\n[check_env_profile] 被中断\n')
        sys.exit(130)
