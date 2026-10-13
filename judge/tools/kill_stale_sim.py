#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""清理"上一轮残留的仿真进程"——只按我们自己的特征匹配，不用宽泛 pkill。

背景（2026-09-16 实测）：`serial_swarm_bringup` 故意让 PX4/MAVROS 在正常退出后继续跑
（keep_children_alive），方便复用；但如果上一轮是被 Ctrl-C / 工具会话结束 / kill 父进程
等"非正常收尾"打断的，PX4 daemon 会活下来。下一轮再起时：

    px4_0.log:  INFO [px4] PX4 daemon already running for instance 0 (Success)

六架 PX4 全部秒退 → 12s 内没有任何 MAVROS connected → 干等 120s 超时 → 健康门拒绝启动
上层（YOLO/坐标解算/tracking 全部不会启动），收尾时 gzserver 再段错误一次。
表面看像"lockstep 又坏了"，其实是**端口/工作目录被残留 PX4 占着**。

用法：
    python3 tools/kill_stale_sim.py            # 清理（先 TERM 再 KILL），打印杀了谁
    python3 tools/kill_stale_sim.py --dry-run  # 只列出，不杀
    python3 tools/kill_stale_sim.py --quiet    # 干净时一声不吭

退出码：0 = 清理后没有残留；1 = 仍有残留（或 --dry-run 时发现残留）
"""

import argparse
import glob
import os
import signal
import sys
import time

# 只匹配"我们这套仿真栈"的特征串（要尽量具体，避免误杀别人的进程）
SIGNATURES = (
    'sitl_typhoon_h480_zzufly', 'typhoon_h480_zzufly',
    'px4-mavlink', 'px4-simulator', 'px4-navigator', 'px4-battery',
    'px4-tone', 'px4-rc', 'init.d-posix/rcS', 'bin/px4',
    'gzserver', 'gzclient',
    'rosmaster', 'roscore', 'rosout',
    'mavros_node', 'xtdrone_mavros_bridge', 'serial_swarm_bringup',
    'src/auto_explore', 'area_mission_node', 'road_follow',
    'scan_map_fuser', 'scan_self_filter', 'coordinate_estimator',
    'target_lookup_service', 'pose_transformer_node', 'odom_broadcaster',
    'pose_cmd_mux', 'gimbal_control', 'yolo11n.py', 'bbox2coord_node',
    'tracking_node', 'start_6g_auto_explore.sh', 'kill_sim.sh',
    # 2026-10-09 补：官方链路的人物与裁判进程。原来漏了这三个，导致上一轮的
    # control_actor.py 残留没被清掉 → 新一轮启动时 ROS 报
    # "new node registered with same name"，6 个人物互相把对方踢死。
    'control_actor.py', 'control_actors.sh', 'score_cal.py', 'cheat_monitor.py',
    'score_watch.sh', 'control_actor_city.py', 'actor_watchdog.py', 'static_transform_publisher',
    # 打补丁的计分副本（文件名不含 score_cal.py，旧特征匹配不到）
    'score_cal_limit.py', 'score_cal_long.py', 'score_cal_nan_guard.py', 'score_cal_patched.py',
    'referee_mirror.py', 'start_official_judge.sh',
    'robocup_judge_drone.launch', 'robocup_judge_gazebo.launch',
)
SELF_MARKERS = ('kill_stale_sim',)


def _ancestors(pid):
    """自身 + 所有祖先 PID（绝不能杀自己或自己的调用者链）。"""
    chain = set()
    while pid and pid > 1:
        chain.add(pid)
        try:
            with open('/proc/%d/stat' % pid) as handle:
                fields = handle.read().rsplit(')', 1)[1].split()
            pid = int(fields[1])
        except (OSError, IndexError, ValueError):
            break
    chain.add(1)
    return chain


def _cmdline(pid):
    try:
        with open('/proc/%d/cmdline' % pid, 'rb') as handle:
            return handle.read().replace(b'\0', b' ').decode('utf-8', 'replace').strip()
    except OSError:
        return ''


# 我们这套仿真栈固定会用的端口（PX4 rcS + MAVROS 约定，见 交接文档 §12）
#   TCP 4560+i        : PX4 <-> Gazebo 的 mavlink 通道（模型插件里监听）
#   UDP 14280+i       : 负载/onboard 辅助
#   UDP 14580+i       : **offboard 本地端口**（MAVROS fcu_url 用的就是它，绑不上必然连不通）
#   UDP 18570+i       : GCS/常规 mavlink
#   UDP 14540+i       : offboard 远端
PORT_BASES = ((4560, 'tcp', 'PX4<->Gazebo'),
              (14280, 'udp', 'payload'),
              (14580, 'udp', 'offboard-local(MAVROS)'),
              (18570, 'udp', 'gcs'),
              (14540, 'udp', 'offboard-remote'))
INSTANCES = range(6)


def _bound_ports(proto):
    """返回 {port: inode}：当前已绑定的端口（/proc/net/{tcp,udp}）。"""
    path = '/proc/net/%s' % proto
    result = {}
    try:
        lines = open(path).read().splitlines()[1:]
    except OSError:
        return result
    for line in lines:
        fields = line.split()
        if len(fields) < 10:
            continue
        try:
            port = int(fields[1].split(':')[1], 16)
        except (IndexError, ValueError):
            continue
        result[port] = fields[9]
    return result


def _owners_of_inodes(inodes):
    """反查持有这些 socket inode 的进程；返回 {inode: 'pid cmd'}。"""
    owners = {}
    for fd in glob.glob('/proc/[0-9]*/fd/*'):
        try:
            link = os.readlink(fd)
        except OSError:
            continue
        if not link.startswith('socket:['):
            continue
        inode = link[8:-1]
        if inode not in inodes:
            continue
        pid = fd.split('/')[2]
        owners[inode] = 'pid=%s %s' % (pid, _cmdline(int(pid))[:100])
    return owners


def port_conflicts():
    """返回 (占用列表, 无法归因的占用列表)。

    占用列表元素: (port, proto, 说明, 持有者或 None)
    无法归因 = 端口被 bound 但是在我们的 PID 命名空间里找不到持有进程
    （沙箱/容器里常见：进程在别的 PID namespace，共享同一个网络 namespace）。
    """
    occupied, unattributed = [], []
    for base, proto, label in PORT_BASES:
        bound = _bound_ports(proto)
        for index in INSTANCES:
            port = base + index
            if port not in bound:
                continue
            owner = _owners_of_inodes({bound[port]}).get(bound[port])
            item = (port, proto, '%s (instance %d)' % (label, index), owner)
            occupied.append(item)
            if owner is None:
                unattributed.append(item)
    return occupied, unattributed


def collect():
    """返回 [(pid, cmdline)]，已排除自身/祖先/本工具自身。"""
    protected = _ancestors(os.getpid())
    found = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid in protected:
            continue
        cmd = _cmdline(pid)
        if not cmd:
            continue
        if any(marker in cmd for marker in SELF_MARKERS):
            continue
        if any(sig in cmd for sig in SIGNATURES):
            found.append((pid, cmd))
    return sorted(found)


def kill_all(procs, grace=2.0):
    for pid, _ in procs:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
    deadline = time.time() + grace
    while time.time() < deadline:
        if not any(_alive(pid) for pid, _ in procs):
            return
        time.sleep(0.2)
    for pid, _ in procs:
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass


def _alive(pid):
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    # 僵尸进程不算"活着"
    try:
        with open('/proc/%d/stat' % pid) as handle:
            state = handle.read().rsplit(')', 1)[1].split()[0]
        return state != 'Z'
    except (OSError, IndexError):
        return False



def clean_px4_locks(verbose=False):
    """清 PX4 SITL 的实例锁与 socket。

    不清的话，被中断的 PX4 会在 /tmp 留下 px4_lock-<i> / px4-sock-<i>，
    下一次启动时 PX4 直接报 "PX4 daemon already running for instance N (Success)"
    并退出 —— 表现为 MAVROS 永远 connected:False、整轮一架都连不上。
    """
    import glob as _glob
    n = 0
    for pat in ('/tmp/px4_lock-*', '/tmp/px4-sock-*'):
        for f in _glob.glob(pat):
            try:
                os.remove(f); n += 1
            except OSError:
                pass
    if verbose and n:
        print('  [px4] 已清实例锁/socket: %d 个' % n)
    return n

def main(argv):
    clean_px4_locks(True)
    parser = argparse.ArgumentParser(description='清理残留仿真进程（只认我们自己的特征）')
    parser.add_argument('--dry-run', action='store_true', help='只列出，不杀')
    parser.add_argument('--quiet', action='store_true', help='干净时什么都不打印')
    args = parser.parse_args(argv)

    procs = collect()
    occupied, unattributed = port_conflicts()
    if not procs and not occupied:
        if not args.quiet:
            print('[preflight] 没有残留仿真进程，环境干净（端口也未占用）')
        return 0

    if occupied:
        print('[preflight] 检测到 %d 个本栈专用端口已被占用：' % len(occupied))
        for port, proto, label, owner in occupied:
            print('  %s %-5d %-26s %s' % (proto, port, label, owner or '**持有进程不可见**'))
    if unattributed:
        print('[preflight] 注意：以下端口被 bind 但在当前 PID 命名空间里找不到持有进程')
        print('           （常见于沙箱/容器：进程在别的命名空间；bg 里 kill 不到）')
        print('           → 请到"能看到这些进程的那个终端"执行 bash kill_sim.sh，或稍后重试。')
        return 1

    if not procs:
        print('[preflight] 端口占用可归因，但没有匹配到可清理的进程签名')
        return 1

    print('[preflight] 检测到 %d 个残留仿真进程：' % len(procs))
    for pid, cmd in procs:
        print('  pid=%-7d %s' % (pid, cmd[:140]))
    if args.dry_run:
        print('[preflight] --dry-run：不清理。清理请执行 bash kill_sim.sh 或去掉 --dry-run')
        return 1

    print('[preflight] 清理中（SIGTERM → 2s → SIGKILL）...')
    kill_all(procs)
    left = collect()
    if left:
        print('[preflight] 仍有 %d 个残留：' % len(left))
        for pid, cmd in left:
            print('  pid=%-7d %s' % (pid, cmd[:140]))
        return 1
    print('[preflight] 已清理干净（%d 个）' % len(procs))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
