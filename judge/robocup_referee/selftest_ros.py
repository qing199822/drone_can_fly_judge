#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""referee.py 的 ROS 集成冒烟测试 —— 在**私有 ROS master** 上跑，绝不碰你的仿真。

做法：
  1. 在本地端口 11400（可用 $REFEREE_TEST_PORT 改）起一个临时 roscore；
  2. 起一个"假 Gazebo"节点：发 /gazebo/model_states、发 /actor_green_info、
     提供 /gazebo/{get,set}_model_state 与 /gazebo/delete_model 服务，
     并订阅 /score /time_usage /left_actors /find_actor_* /actor_*/cmd_motion
     把收到的东西记到 JSON 里；
  3. 跑两个阶段（每段之间用 /referee_test/reset 清空记录）：

     phase-A  9 秒，--teleport-interval 3
        验证：发现目标 → 3 秒后人物被传送（set_model_state）
     phase-B  26 秒，--teleport-interval 60
        验证：发现后连续保持 15 秒 → 人物被消除（delete_model）、
              /left_actors 少一个、连续跟踪 20 秒拿到 +80、被追时会逃跑

为什么要分两段：消除要求"连续保持 15 秒"，而传送会让坐标跳变、判定按规则清零。
若把传送间隔设得比 15 秒还短，人物就永远消除不掉（这是规则行为，不是 bug）。

用法：
    python3 robocup_referee/selftest_ros.py
环境变量：
    REFEREE_TEST_PORT=11400      私有 master 端口
    REFEREE_TEST_SECONDS         覆盖每个阶段时长（调试用，不建议）
    REFEREE_TEST_DEBUG_TELEPORT=1 让 referee 打印传送倒计时
退出码 0 = 全过。
"""

import argparse
import json
import os
import shutil
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
WORKSPACE = os.path.dirname(HERE)
REFEREE = os.path.join(HERE, 'referee.py')
PORT = int(os.environ.get('REFEREE_TEST_PORT', '11400'))
MASTER = 'http://localhost:%d' % PORT

# 让本进程（包括 --fake-sim 子进程）也能 import 到工作区 devel 里的消息包，
# 并且避开只读的 ~/.ros（与 referee.py 用同一套兜底）。
sys.path.insert(0, HERE)
try:
    import referee as _referee
    _referee.ensure_ros_log_dir()
    _referee.catkin_paths = _referee.bootstrap_catkin_paths()
except Exception as _exc:                    # noqa: BLE001
    print('[selftest_ros] 预加载 referee 模块失败（可忽略）: %s' % _exc)

PHASES = [
    dict(name='phase-A 发现→传送', duration=9.0, teleport=3.0,
         checks=('find', 'teleport')),
    dict(name='phase-B 保持15s→消除 + 跟踪 + 逃跑', duration=26.0, teleport=60.0,
         checks=('find', 'delete', 'left', 'tracking', 'catch')),
]
if os.environ.get('REFEREE_TEST_SECONDS'):
    _s = float(os.environ['REFEREE_TEST_SECONDS'])
    for _p in PHASES:
        _p['duration'] = _s


# --------------------------------------------------------------------------
# 假 Gazebo / 假视觉输入
# --------------------------------------------------------------------------
def fake_sim(obs_file, seconds):
    import math
    import rospy
    from gazebo_msgs.msg import ModelStates
    from gazebo_msgs.srv import (DeleteModel, DeleteModelResponse, GetModelState,
                                 GetModelStateResponse, SetModelState,
                                 SetModelStateResponse)
    from geometry_msgs.msg import Point, Pose, Quaternion, Twist
    from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo, ActorMotion
    from std_msgs.msg import Empty, Float32, Header, Int16, String

    KEYS = ('score', 'left', 'time', 'find_actor', 'delete_model',
            'set_model_state')
    obs = {k: ({} if k == 'cmd_motion' else []) for k in KEYS}
    obs['cmd_motion'] = {}
    actors = {0: (20.0, 0.0), 1: (5.0, 0.0), 2: (-10.0, 8.0),
              3: (30.0, -5.0), 4: (-20.0, -10.0), 5: (40.0, 20.0)}
    uavs = {i: [0.0, 0.0, 1.2] for i in range(6)}

    def flush():
        with open(obs_file, 'w') as fh:
            json.dump(obs, fh)

    def reset_obs(_msg=None):
        for k in KEYS:
            obs[k] = {} if k == 'cmd_motion' else []
        flush()

    rospy.init_node('fake_sim', anonymous=True, disable_signals=True)
    ms_pub = rospy.Publisher('/gazebo/model_states', ModelStates, queue_size=1)
    info_pub = rospy.Publisher('/actor_green_info', ActorInfo, queue_size=1)

    rospy.Subscriber('/score', Int16, lambda m: obs['score'].append(m.data))
    rospy.Subscriber('/left_actors', String, lambda m: obs['left'].append(m.data))
    rospy.Subscriber('/time_usage', Int16, lambda m: obs['time'].append(m.data))
    rospy.Subscriber('/referee_test/reset', Empty, reset_obs, queue_size=1)

    def on_find(idx):
        def cb(msg):
            obs['find_actor'].append([idx, float(msg.data)])
        return cb

    def on_cmd(idx):
        def cb(_msg):
            key = str(idx)
            obs['cmd_motion'][key] = obs['cmd_motion'].get(key, 0) + 1
        return cb

    for i in range(6):
        rospy.Subscriber('/find_actor_%d' % i, Float32, on_find(i), queue_size=5)
        rospy.Subscriber('/actor_%d/cmd_motion' % i, ActorMotion, on_cmd(i),
                         queue_size=10)

    # ---- 假 gazebo 服务 ----
    # 注意：gazebo_msgs/GetModelState 的响应带 Header
    # （header, pose, twist, success, status_message），只能属性赋值构造。
    def make_get_response(x, y, z, ok=True, message='ok'):
        resp = GetModelStateResponse()
        resp.header = Header()
        resp.pose = Pose(Point(x, y, z), Quaternion(0, 0, 0, 1))
        resp.twist = Twist()
        resp.success = ok
        resp.status_message = message
        return resp

    def handle_get(req):
        name = req.model_name
        if name.startswith('typhoon_h480_'):
            idx = int(name.rsplit('_', 1)[-1])
            x, y, z = uavs[idx]
            return make_get_response(x, y, z)
        if name.startswith('actor_'):
            idx = int(name.rsplit('_', 1)[-1])
            x, y = actors.get(idx, (0.0, 0.0))
            return make_get_response(x, y, 1.25)
        return make_get_response(0.0, 0.0, 0.0, False, 'not found')

    def handle_set(req):
        name = req.model_state.model_name
        obs['set_model_state'].append(name)
        if name.startswith('actor_'):
            idx = int(name.rsplit('_', 1)[-1])
            actors[idx] = (req.model_state.pose.position.x,
                           req.model_state.pose.position.y)
        flush()
        return SetModelStateResponse(True, 'ok')

    def handle_delete(req):
        obs['delete_model'].append(req.model_name)
        flush()
        return DeleteModelResponse(True, 'ok')

    rospy.Service('/gazebo/get_model_state', GetModelState, handle_get)
    rospy.Service('/gazebo/set_model_state', SetModelState, handle_set)
    rospy.Service('/gazebo/delete_model', DeleteModel, handle_delete)

    rate = rospy.Rate(20)
    t0 = time.time()
    last_flush = 0.0
    while not rospy.is_shutdown() and (time.time() - t0) < seconds:
        t = time.time() - t0
        # UAV 0 以 2 m/s 绕 actor_1（半径 5 m）转圈 → 触发"被追"与"连续跟踪"
        uavs[0] = [5.0 + 5.0 * math.cos(0.4 * t), 5.0 * math.sin(0.4 * t), 1.2]
        msg = ModelStates()
        names, poses, twists = [], [], []
        for i in range(6):
            x, y, z = uavs[i]
            names.append('typhoon_h480_%d' % i)
            poses.append(Pose(Point(x, y, z), Quaternion(0, 0, 0, 1)))
            twists.append(Twist())
        for idx in sorted(actors):
            x, y = actors[idx]
            names.append('actor_%d' % idx)
            poses.append(Pose(Point(x, y, 1.25), Quaternion(0, 0, 0, 1)))
            twists.append(Twist())
        msg.name, msg.pose, msg.twist = names, poses, twists
        ms_pub.publish(msg)
        # 视觉输入：green = actor_0 的真实位置，20 Hz 连续给（满足 <1s 连续判据）
        info = ActorInfo()
        info.cls, info.x, info.y = 'green', actors[0][0], actors[0][1]
        info_pub.publish(info)
        if time.time() - last_flush > 1.0:
            last_flush = time.time()
            flush()
        rate.sleep()
    flush()
    print('[fake_sim] 结束')
    rospy.signal_shutdown('fake sim done')


# --------------------------------------------------------------------------
# 私有 master 启停 / 阶段编排
# --------------------------------------------------------------------------
def port_open(port, timeout=0.3):
    s = socket.socket()
    s.settimeout(timeout)
    try:
        s.connect(('127.0.0.1', port))
        return True
    except Exception:
        return False
    finally:
        s.close()


def wait_port(port, timeout=40.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if port_open(port):
            return True
        time.sleep(0.5)
    return False


def reset_observations(env):
    try:
        subprocess.run(['rostopic', 'pub', '-1', '/referee_test/reset',
                        'std_msgs/Empty'],
                       env=env, timeout=10, capture_output=True)
    except Exception as exc:                 # noqa: BLE001
        print('  （reset 话题发布失败，忽略: %s）' % exc)


def run_phase(env, run_dir, phase, index):
    log_path = os.path.join(run_dir, 'referee_%s.log' % phase['name'].split()[0])
    cmd = [sys.executable, REFEREE, 'run',
           '--black-box', os.path.join(run_dir, 'black_box.txt'),
           '--obstacle', os.path.join(run_dir, 'obstacle.txt'),
           '--score-mode', 'extended',
           '--teleport-interval', str(phase['teleport']),
           '--score-output', os.path.join(run_dir, 'score_%d.txt' % index),
           '--duration', str(phase['duration'])]
    if os.environ.get('REFEREE_TEST_DEBUG_TELEPORT'):
        cmd.append('--debug-teleport')
    print('  → 跑 %s：--duration %g --teleport-interval %g'
          % (phase['name'], phase['duration'], phase['teleport']))
    t0 = time.time()
    with open(log_path, 'w') as log:
        rc = subprocess.call(cmd, env=env, stdout=log, stderr=subprocess.STDOUT)
    return rc, log_path, time.time() - t0


def phase_checks(phase, obs, log_text, rc):
    """返回 [(name, ok, detail), ...]"""
    out = [
        ('referee 正常退出', rc == 0, 'rc=%d' % rc),
        ('/score 有发布', len(obs.get('score', [])) > 0,
         '末值=%s' % ((obs.get('score') or [None])[-1])),
        ('/time_usage 有发布', len(obs.get('time', [])) > 0,
         '末值=%s' % ((obs.get('time') or [None])[-1])),
        ('/left_actors 有发布', len(obs.get('left', [])) > 0,
         '末值=%s' % ((obs.get('left') or [None])[-1])),
        ('人物在巡逻（/actor_*/cmd_motion）',
         len([v for v in obs.get('cmd_motion', {}).values() if v]) >= 3,
         '收到了指令的人物=%s'
         % sorted(k for k, v in obs.get('cmd_motion', {}).items() if v)),
    ]
    if 'find' in phase['checks']:
        out.append(('/find_actor_* 有发布（发现目标）',
                    len(obs.get('find_actor', [])) > 0,
                    'find=%s' % obs.get('find_actor')))
    if 'teleport' in phase['checks']:
        out.append(('人物被传送 set_model_state(actor_0)',
                    'actor_0' in obs.get('set_model_state', []),
                    'set=%s' % len(obs.get('set_model_state', []))))
    if 'delete' in phase['checks']:
        out.append(('人物被消除 delete_model(actor_0)',
                    'actor_0' in obs.get('delete_model', []),
                    'delete=%s' % obs.get('delete_model')))
    if 'left' in phase['checks']:
        last_left = (obs.get('left') or [''])[-1]
        out.append(('/left_actors 里少了 actor_0', '0' not in last_left,
                    '末值=%s' % last_left))
    if 'tracking' in phase['checks']:
        out.append(('连续跟踪 20s 拿分', 'tracking success' in log_text))
    if 'catch' in phase['checks']:
        out.append(('被追时会逃跑', 'catch (UAV' in log_text))
    return out


def main():
    parser = argparse.ArgumentParser(description='referee.py 的 ROS 集成冒烟测试')
    parser.add_argument('--fake-sim', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--obs-file', default=None, help=argparse.SUPPRESS)
    parser.add_argument('--seconds', type=float, default=0.0, help=argparse.SUPPRESS)
    args = parser.parse_args()

    if args.fake_sim:                        # 作为子进程被调用
        fake_sim(args.obs_file, args.seconds)
        return 0

    env = dict(os.environ)
    env['ROS_MASTER_URI'] = MASTER
    env.setdefault('ROS_LOG_DIR', os.path.join(WORKSPACE, 'logs', 'ros'))
    env.setdefault('ROS_HOME', os.path.join(WORKSPACE, 'logs', 'ros_home'))
    os.makedirs(env['ROS_LOG_DIR'], exist_ok=True)
    os.makedirs(env['ROS_HOME'], exist_ok=True)

    run_dir = os.path.join(WORKSPACE, 'logs', 'referee_ros_test')
    if os.path.isdir(run_dir):
        shutil.rmtree(run_dir)
    os.makedirs(run_dir)
    obs_file = os.path.join(run_dir, 'fake_obs.json')
    fake_log = open(os.path.join(run_dir, 'fake_sim.log'), 'w')

    if port_open(PORT):
        print('端口 %d 已被占用，请用 $REFEREE_TEST_PORT 换一个' % PORT)
        return 2

    procs = []
    print('[1/4] 起私有 roscore（:%d）…' % PORT)
    roscore = subprocess.Popen(['roscore', '-p', str(PORT)],
                               stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL,
                               env=env, preexec_fn=os.setsid)
    procs.append(roscore)
    if not wait_port(PORT, 40):
        print('roscore 起不来（检查 $ROS_HOME 是否可写）')
        return 2
    time.sleep(2.0)
    fake = None
    try:
        print('[2/4] 生成测试地图 + 起假 Gazebo / 假视觉输入 …')
        gen = subprocess.run([sys.executable, REFEREE, 'gen-map',
                              '--out', run_dir, '--base', '1', '--seed', '7',
                              '--fix-known-bugs'],
                             env=env, capture_output=True, text=True)
        if gen.returncode != 0:
            print('gen-map 失败：\n%s' % gen.stderr)
            return 2
        fake_seconds = sum(p['duration'] for p in PHASES) + 20
        fake = subprocess.Popen([sys.executable, os.path.abspath(__file__),
                                 '--fake-sim', '--obs-file', obs_file,
                                 '--seconds', str(fake_seconds)],
                                stdout=fake_log, stderr=subprocess.STDOUT,
                                env=env, preexec_fn=os.setsid)
        procs.append(fake)
        time.sleep(2.0)

        print('[3/4] 分阶段跑 referee.py run …')
        all_checks = []
        for index, phase in enumerate(PHASES):
            reset_observations(env)
            time.sleep(0.5)
            rc, log_path, elapsed = run_phase(env, run_dir, phase, index)
            print('      %s：退出码=%d 用时 %.1fs' % (phase['name'], rc, elapsed))
            time.sleep(1.0)
            try:
                obs = json.load(open(obs_file))
            except Exception as exc:         # noqa: BLE001
                print('      读不到观测文件：%s' % exc)
                obs = {}
            log_text = open(log_path).read()
            print('  【%s】' % phase['name'])
            for name, ok, *detail in phase_checks(phase, obs, log_text, rc):
                print('    [%s] %-40s %s' % ('PASS' if ok else 'FAIL', name,
                                             detail[0] if detail else ''))
                all_checks.append((ok, name))

        print('\n[4/4] 汇总')
        failed = [n for ok, n in all_checks if not ok]
        print('集成冒烟：%d 通过 / %d 失败（日志目录: %s）'
              % (len(all_checks) - len(failed), len(failed), run_dir))
        for name in failed:
            print('  失败项：%s' % name)
        return 1 if failed else 0
    finally:
        for p in reversed(procs):
            try:
                os.killpg(os.getpgid(p.pid), 15)
            except Exception:
                pass
        time.sleep(1.5)
        for p in reversed(procs):
            try:
                os.killpg(os.getpgid(p.pid), 9)
            except Exception:
                pass
        fake_log.close()


if __name__ == '__main__':
    sys.exit(main())
