#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""严格串行的多机仿真启动编排器（Gazebo 世界 -> 逐架 模型/PX4/MAVROS）。

设计目标（对应"六机启动稳定性优先"）：
    1. Gazebo 世界完全就绪后才开始生成模型；
    2. 每架无人机严格串行，前一架未通过门控不会启动下一架：

       生成模型 i（阻塞式 spawn_sdf_model 调用）
         -> get_model_properties 确认模型真的在 Gazebo 里
         -> 启动 PX4 SITL(i)
         -> 启动 MAVROS(i)
         -> 等待 /typhoon_h480_i/mavros/state connected=True 且位姿话题有数据
         -> 生成模型 i+1 ...

    3. 六架全部就绪后，启动 xtdrone 控制桥（cmd_vel_flu -> MAVROS 桥）；
    4. 输出健康门报告（模型/位姿/原始激光/PX4 存活/端口冲突/段错误关键字），
       有 FAIL 项时以非 0 退出，便于 1.sh 拒绝启动上层算法栈。

用法：
    rosrun xtdrone_mavros_bridge serial_swarm_bringup.py --num-drones 6
    rosrun xtdrone_mavros_bridge serial_swarm_bringup.py --num-drones 2 --settle 3

前置条件：Gazebo 已在运行（roscore 可用），本脚本不负责启动 gzserver。
"""

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import rospy
from gazebo_msgs.srv import (
    GetModelProperties,
    GetWorldProperties,
    SpawnModel,
    SpawnModelRequest,
)
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty


# --- 路径（与 1.sh / spawn_drones.launch 保持一致的约定） -------------------
# PX4 树可用 ZZU_PX4_DIR 覆盖，默认 $HOME/PX4_Firmware。
# 本仓库的 px4_romfs 覆盖层是 v1.11 布局，必须配 v1.11 的 PX4 二进制，
# 否则 v1.13 会静默忽略 airframe / 端口脚本，MAVROS 永远连不上。
PX4_ROOT = os.path.expanduser(os.environ.get("ZZU_PX4_DIR", "~/PX4_Firmware"))
PX4_BIN = os.path.join(PX4_ROOT, "build", "px4_sitl_default", "bin", "px4")
PX4_BUILD = os.path.join(PX4_ROOT, "build", "px4_sitl_default")
PX4_ROMFS = os.environ.get(
    "ZZU_FLY_PX4_ROMFS",
    os.path.join(PX4_ROOT, "ROMFS", "px4fmu_common"),
)
AIRFRAME_DIR = os.path.join(PX4_ROMFS, "init.d-posix")
PX4_SIM_MODEL = "typhoon_h480_zzufly"


def find_workspace(start_dir):
    """向上查找 .catkin_workspace，定位工作空间根目录。"""
    directory = os.path.abspath(start_dir)
    for _ in range(8):
        if os.path.exists(os.path.join(directory, ".catkin_workspace")):
            return directory
        parent = os.path.dirname(directory)
        if parent == directory:
            break
        directory = parent
    return os.path.abspath(os.path.join(start_dir, "..", "..", ".."))


WORKSPACE = os.environ.get("ZZU_FLY_WORKSPACE") or find_workspace(
    os.path.dirname(os.path.realpath(__file__))
)
MODEL_DIR = os.path.join(WORKSPACE, "typhoon_h480_zzufly")
LOG_DIR = os.path.join(WORKSPACE, "logs", "serial_bringup")

# ---- PX4 <-> Gazebo lockstep 开关 -------------------------------------------------
# 2026-09-15 实测（六机 + 随机地图里带 rover）：
#   保持 SDF 里的 <enable_lockstep>true</enable_lockstep> 时，Gazebo 会在 sim_time≈4.0s
#   死锁：gzserver 所有线程挂在 futex_wait/hrtimer_nanosleep（不是 CPU 打满），/clock 不再
#   推进，六机 MAVROS 永远 connected=False，最后 gzserver 退出时段错误(139)。
#   把它改成 false（PX4 SITL 常规的非 lockstep 模式）后同一张图 RTF≈1.03、六机正常
#   connected=True。rover=0 的图两种设置都能跑，所以这个坑只在"图里还有 rover"时暴露。
# 想恢复旧行为：SDF_LOCKSTEP=1。
LOCKSTEP = os.environ.get("SDF_LOCKSTEP", "0").strip().lower() in ("1", "true", "yes", "on")
LOCKSTEP_RE = re.compile(r"<enable_lockstep>\s*(?:true|false|1|0)\s*</enable_lockstep>", re.IGNORECASE)

# ---- 残留 PX4 daemon 检测（2026-09-16）--------------------------------------------
# 上一轮被 Ctrl-C / kill 父进程等方式打断时，PX4 daemon 会活下来（keep_children_alive 的设计
# 使然）。下一轮再起 PX4 会直接退出并只留一行：
#     INFO  [px4] PX4 daemon already running for instance 0 (Success)
# 结果：六机 MAVROS 永远 connected=False → 干等 120s → 健康门拒绝启动上层。
# 这里把这种"秒退"当场识别出来，立刻给出可执行的原因，不再白等 120s。
STALE_PX4_RE = re.compile(r"PX4 daemon already running for instance (\d+)")


def px4_stale_daemon_reason(log_text):
    """PX4 日志里出现 'daemon already running' 时返回人类可读的原因，否则 None。"""
    match = STALE_PX4_RE.search(log_text or "")
    if not match:
        return None
    return ("检测到残留 PX4 daemon（instance {}）：上一轮 PX4 没退干净，端口/工作目录仍被占用，"
            "本次 PX4 已秒退。先执行 `python3 tools/kill_stale_sim.py`"
            "（或 `bash kill_sim.sh`）清掉残留，再重新起仿真。".format(match.group(1)))


# ---- gzserver 控制台日志：Queue limit 检测（2026-09-25）----------------------------
# 现象：健康门报 `/<drone>/scan` 无数据，但 ROS 侧发布者注册正常、Laser Plugin 也加载了。
# 真因通常是 GUI（gzclient）卡顿：它订阅了 <visualize>true</visualize> 的激光可视化话题，
# gzserver 的 Publisher 队列是"所有连接共享"的，GUI 不读 -> 队列满 -> 丢消息
# （gazebo 控制台打印 "Queue limit reached for topic .../laser/scan, deleting message"），
# 同队列上的 gazebo_ros_laser 桥接也被饿死。这里把该日志翻译成健康门告警，
# 下次一眼就能看出来，不用再翻 ~/.gazebo。
def find_gzserver_console_log(home=None):
    """返回最近写入的 gzserver 控制台日志路径（~/.gazebo/server-<port>/default.log）。"""
    import glob
    base = os.path.join(home or os.path.expanduser("~"), ".gazebo")
    paths = glob.glob(os.path.join(base, "server-*", "default.log"))
    if not paths:
        return None
    try:
        return max(paths, key=os.path.getmtime)
    except OSError:
        return paths[0]


def gz_queue_limit_topics(log_path, limit=6):
    """解析 gzserver 控制台日志，返回出现过 'Queue limit reached' 的话题（去重保序）。"""
    if not log_path or not os.path.exists(log_path):
        return []
    topics = []
    try:
        with open(log_path, "r", errors="replace") as handle:
            for line in handle:
                if "Queue limit reached" not in line:
                    continue
                match = re.search(r"Queue limit reached for topic (\S+)", line)
                if match:
                    topics.append(match.group(1).rstrip(","))
    except (IOError, OSError):
        return []
    unique = []
    for topic in topics:
        if topic not in unique:
            unique.append(topic)
    return unique[:limit]


def apply_lockstep(model_xml):
    """按 LOCKSTEP 改写 SDF 里的 <enable_lockstep>；返回 (新文本, 替换次数)。"""
    want = "<enable_lockstep>%s</enable_lockstep>" % ("true" if LOCKSTEP else "false")
    new_xml, count = LOCKSTEP_RE.subn(want, model_xml)
    return new_xml, count


def remove_unused_stereo(model_xml):
    root = ET.fromstring(model_xml)
    removed = 0
    for link in root.iter("link"):
        for sensor in list(link.findall("sensor")):
            if sensor.get("name") == "stereo_right":
                link.remove(sensor)
                removed += 1
    return ET.tostring(root, encoding="unicode"), removed


def spawn_pose(index):
    """与 spawn_drones.launch 相同的 2x3 集群出生点（世界坐标）。"""
    return (-17.0 + 3.0 * (index % 2), -3.0 + 3.0 * (index // 2), 1.0)


def model_name(index):
    return "typhoon_h480_{}".format(index)


def sdf_path(index):
    candidate = os.path.join(MODEL_DIR, "typhoon_h480_{}.sdf".format(index))
    if os.path.exists(candidate):
        return candidate
    return os.path.join(MODEL_DIR, "typhoon_h480_zzufly.sdf")


def log(message):
    sys.stdout.write("[serial-bringup] {}\n".format(message))
    sys.stdout.flush()


class BringupError(Exception):
    pass


class SerialSwarmBringup(object):
    def __init__(self, args):
        self.args = args
        self.drone_ids = list(range(args.start_index, args.start_index + args.num_drones))
        self.children = []          # 本脚本启动的 Popen（PX4 / roslaunch）
        # 编排器正常退出时，底层进程（PX4/MAVROS/控制桥）必须继续存活，交给 1.sh/用户管理；
        # 只有失败中止或收到信号时才回收。
        self.keep_children_alive = False
        self._states = {}           # drone id -> mavros_msgs/State（band 模式并发等待用）
        self._poses = {}            # drone id -> PoseStamped
        self._subscribers = []
        self._report_entries = {}   # drone id -> report 条目
        self.report = {"drones": [], "started_at": time.time(),
                       "mode": getattr(args, "mode", "serial")}
        os.makedirs(LOG_DIR, exist_ok=True)
        for path in (PX4_BIN, PX4_ROMFS, AIRFRAME_DIR, MODEL_DIR):
            if not os.path.exists(path):
                raise BringupError("缺少必要路径：{}".format(path))

    # --- 进程/日志工具 -----------------------------------------------------
    def start_process(self, command, log_path, extra_env=None):
        environment = os.environ.copy()
        if extra_env:
            environment.update(extra_env)
        # 每次启动都新建日志，避免多次运行的内容混在一起（例如端口冲突行重复出现）。
        handle = open(log_path, "wb")
        handle.write(("===== {} =====\n".format(" ".join(command))).encode("utf-8"))
        handle.flush()
        process = subprocess.Popen(
            command,
            stdout=handle,
            stderr=subprocess.STDOUT,
            env=environment,
            cwd=PX4_BUILD,
            preexec_fn=os.setsid,
        )
        self.children.append(process)
        return process

    def shutdown_children(self, force=False):
        if self.keep_children_alive and not force:
            return
        for process in self.children:
            if process.poll() is None:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGINT)
                except OSError:
                    pass
        deadline = time.time() + 8.0
        for process in self.children:
            while process.poll() is None and time.time() < deadline:
                time.sleep(0.2)
            if process.poll() is None:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except OSError:
                    pass

    def grep_log(self, path, needles):
        if not os.path.exists(path):
            return []
        hits = []
        with open(path, "rb") as handle:
            content = handle.read().decode("utf-8", "replace")
        for needle in needles:
            if needle.lower() in content.lower():
                hits.append(needle)
        return hits

    # --- 阶段 0：Gazebo 世界就绪 -------------------------------------------
    def wait_for_world(self):
        log("等待 Gazebo 世界服务 /gazebo/get_world_properties ...")
        rospy.wait_for_service("/gazebo/get_world_properties", timeout=60.0)
        world = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
        deadline = time.time() + self.args.world_timeout
        started = time.time()
        last_count = -1
        last_report = 0.0
        stable = 0
        while time.time() < deadline and not rospy.is_shutdown():
            try:
                response = world()
            except rospy.ServiceException as exc:
                log("  世界服务暂时不可用（{}），重试中...".format(exc))
                time.sleep(2.0)
                continue
            if not response.success:
                time.sleep(1.0)
                continue
            count = len(response.model_names)
            now = time.time()
            if now - last_report > 20.0:
                last_report = now
                log("  世界加载中：已出现 {} 个模型（已等待 {:.0f}s）".format(
                    count, now - started))
            if count == last_count and count > 0:
                stable += 1
                if stable >= 4:
                    log("世界已就绪：{} 个模型（含 {}，耗时 {:.1f}s）".format(
                        count, ", ".join(sorted(response.model_names)[:6]), now - started))
                    break
            else:
                stable = 0
                last_count = count
            time.sleep(0.5)
        else:
            raise BringupError(
                "Gazebo 世界在 {}s 内未就绪".format(self.args.world_timeout))

        try:
            unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
            unpause()
            log("已确认物理未暂停（unpause_physics）")
        except rospy.ServiceException as exc:
            log("unpause_physics 调用失败（可忽略）：{}".format(exc))

    # --- 阶段 1：逐架门控 --------------------------------------------------
    def ensure_airframe_alias(self):
        """PX4 同时存在 4001/6011 typhoon 时按后缀查找会撞车，需六旋翼唯一别名。"""
        for suffix in ("", ".post"):
            link = os.path.join(AIRFRAME_DIR, "6011_typhoon_h480_zzufly" + suffix)
            target = os.path.join(AIRFRAME_DIR, "6011_typhoon_h480" + suffix)
            if not os.path.exists(target):
                raise BringupError("缺少 PX4 机型文件：{}".format(target))
            if not os.path.exists(link):
                os.symlink(target, link)
                log("创建 PX4 机型别名 {}".format(link))

    def spawn_model(self, index):
        name = model_name(index)
        path = sdf_path(index)
        with open(path, "r") as handle:
            model_xml = handle.read()
        if self.args.no_stereo_right:
            model_xml, removed = remove_unused_stereo(model_xml)
            log("[drone{}] disabled {} unused stereo_right depth sensor(s)".format(
                index, removed))
        if LOCKSTEP is False:
            model_xml, patched = apply_lockstep(model_xml)
            if patched == 0:
                raise BringupError("[drone{}] {} 里找不到 <enable_lockstep>，"
                                   "无法关闭 lockstep（六机+rover 会死锁）".format(index, path))
            log("[drone{}] SDF lockstep=false：改写 <enable_lockstep> {} 处".format(index, patched))
        else:
            log("[drone{}] SDF lockstep=true：保持 SDF 原值（SDF_LOCKSTEP=1）".format(index))
        x, y, z = spawn_pose(index)
        spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        request = SpawnModelRequest()
        request.model_name = name
        request.model_xml = model_xml
        request.robot_namespace = ""
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = z
        request.reference_frame = "world"
        log("[drone{}] 生成模型 {} <- {} (x={:.1f}, y={:.1f}, z={:.1f})".format(
            index, name, os.path.basename(path), x, y, z))
        started = time.time()
        response = spawn(request)
        if not response.success:
            raise BringupError("[drone{}] spawn_sdf_model 失败：{}".format(
                index, response.status_message))
        log("[drone{}] 模型生成请求返回成功（{:.1f}s）".format(index, time.time() - started))

        properties = rospy.ServiceProxy("/gazebo/get_model_properties", GetModelProperties)
        deadline = time.time() + self.args.model_timeout
        while time.time() < deadline and not rospy.is_shutdown():
            try:
                if properties(name).success:
                    log("[drone{}] get_model_properties 确认模型已在 Gazebo 中".format(index))
                    return
            except rospy.ServiceException:
                pass
            time.sleep(1.0)
        raise BringupError("[drone{}] 模型 {} 在 {}s 内未出现在 Gazebo 中".format(
            index, name, self.args.model_timeout))

    def start_px4(self, index):
        work_dir = "/tmp/sitl_{}_{}".format(PX4_SIM_MODEL, index)
        os.makedirs(work_dir, exist_ok=True)
        log_path = os.path.join(LOG_DIR, "px4_{}.log".format(index))
        command = [
            PX4_BIN, PX4_ROMFS,
            "-s", "etc/init.d-posix/rcS",
            "-i", str(index),
            "-w", work_dir,
            "-d",
        ]
        log("[drone{}] 启动 PX4 SITL（instance {}，日志 {}）".format(
            index, index, log_path))
        # 把高度相关 mavlink 参数通过环境变量传给 PX4 SITL：
        # MPC_Z_MAX           物理硬顶 5.0m（score_cal MAX_UAV_ALTITUDE=6m，留 1m 余量）
        # MPC_Z_VEL_MAX_UP    上行速度 ≤0.5 m/s（避免 wave 过冲）
        # MPC_Z_VEL_MAX_DN    下行速度 ≤0.7 m/s
        # MPC_LAND_SPEED      着陆速度 ≤0.6 m/s
        #
        # 2026-09-25 S15：放宽 EKF2 baro 超时容限，避免 RTF 抖动触发
        #   "baro hgt timeout - reset to baro/gps"，把 fake_gps geo_origin=苏黎世 408m
        #   灌进 local_position/pose.z 产生巨大偏差。
        # EKF2_BARO_GATE      baro 测量噪声门：4σ → 8σ（多吸收短时跳变）
        # EKF2_BARO_NOISE      baro 噪声：1.0 → 3.0 m（容忍 mavlink 抖动）
        # EKF2_GPS_P_GATE_NOISE/V_GATE_NOISE 同步放宽，否则 GPS 也复位
        # EKF2_AID_MASK        默认 1+2+8 (GPS+baro+range)，不变
        px4_env = {
            "PX4_SIM_MODEL": PX4_SIM_MODEL,
            "PX4_ESTIMATOR": "ekf2",
            "PX4_PARAM_MPC_Z_MAX": "5.0",
            "PX4_PARAM_MPC_Z_VEL_MAX_UP": "0.5",
            "PX4_PARAM_MPC_Z_VEL_MAX_DN": "0.7",
            "PX4_PARAM_MPC_LAND_SPEED": "0.6",
            "PX4_PARAM_MPC_TKO_SPEED": "0.5",
            "PX4_PARAM_EKF2_BARO_GATE": "8.0",
            "PX4_PARAM_EKF2_BARO_NOISE": "3.0",
            "PX4_PARAM_EKF2_GPS_P_GATE": "8.0",
            "PX4_PARAM_EKF2_GPS_V_GATE": "8.0",
            "PX4_PARAM_SENS_BARO_RATE": "50.0",
        }
        return self.start_process(command, log_path, extra_env=px4_env)

    def start_mavros(self, index):
        log_path = os.path.join(LOG_DIR, "mavros_{}.log".format(index))
        command = [
            "roslaunch", "xtdrone_mavros_bridge", "mavros_vehicle.launch",
            "vehicle:=typhoon_h480_{}".format(index),
            "drone_id:={}".format(index),
            "tgt_system:={}".format(index + 1),
        ]
        log("[drone{}] 启动 MAVROS（日志 {}）".format(index, log_path))
        return self.start_process(command, log_path, extra_env={"ROS_MASTER_URI": os.environ.get(
            "ROS_MASTER_URI", "http://localhost:11311")})

    def check_px4_startup(self, index):
        """PX4 秒退 / offboard 端口被占时立刻报错，不要干等 mavros_timeout。

        两类残留都会让"六机永远 connected=False"：
          1. 上一轮的 PX4 daemon 还活着 → 本次 PX4 秒退（daemon already running）；
          2. 上一轮的 UDP 端口没释放 → 本实例 offboard 端口 14580+id bind failed
             （MAVROS 就是靠这个端口跟 PX4 说话的，绑不上必然连不通）。
        """
        path = os.path.join(LOG_DIR, "px4_{}.log".format(index))
        try:
            with open(path, "r") as handle:
                text = handle.read()
        except OSError:
            return
        reason = px4_stale_daemon_reason(text)
        if reason:
            raise BringupError("[drone{}] {}".format(index, reason))
        critical, others = self.analyze_mavlink_binds(path, index)
        if critical:
            raise BringupError(
                "[drone{}] PX4 offboard 端口 {} 绑定失败（bind failed: Address already in use）"
                "：上一轮的 UDP 端口没释放或别的进程占着 → MAVROS 不可能 connected。"
                "先执行 `python3 tools/kill_stale_sim.py`，"
                "不行就在能看到残留进程的终端执行 `bash kill_sim.sh`。"
                "（其余辅助端口绑定失败是 airframe 里写死 14558 造成的，属正常：{}）".format(
                    index, ", ".join(str(p) for p in critical), others or "无"))

    def wait_mavros_connected(self, index):
        state_topic = "/typhoon_h480_{}/mavros/state".format(index)
        pose_topic = "/typhoon_h480_{}/mavros/local_position/pose".format(index)
        deadline = time.time() + self.args.mavros_timeout
        last_log = 0.0
        while time.time() < deadline and not rospy.is_shutdown():
            try:
                state = rospy.wait_for_message(state_topic, State, timeout=3.0)
                if state.connected:
                    pose = rospy.wait_for_message(pose_topic, PoseStamped, timeout=5.0)
                    log("[drone{}] MAVROS connected=True（mode={}）且位姿可用："
                        "x={:.2f} y={:.2f} z={:.2f}".format(
                            index, state.mode,
                            pose.pose.position.x, pose.pose.position.y,
                            pose.pose.position.z))
                    return state
            except rospy.ROSException:
                pass
            self.check_px4_startup(index)
            now = time.time()
            if now - last_log > 15.0:
                last_log = now
                log("[drone{}] 等待 MAVROS connected=True ...（剩 {:.0f}s）".format(
                    index, deadline - now))
            time.sleep(1.0)
        raise BringupError("[drone{}] MAVROS 在 {}s 内未 connected=True".format(
            index, self.args.mavros_timeout))

    def bringup_one(self, index):
        entry = {"id": index, "model": model_name(index), "seconds": {}}
        started = time.time()
        self.spawn_model(index)
        entry["seconds"]["spawn"] = round(time.time() - started, 1)

        started = time.time()
        px4 = self.start_px4(index)
        entry["px4_pid"] = px4.pid
        self.start_mavros(index)
        state = self.wait_mavros_connected(index)
        entry["seconds"]["px4_to_mavros"] = round(time.time() - started, 1)
        entry["connected"] = bool(state.connected)
        entry["px4_log"] = os.path.join(LOG_DIR, "px4_{}.log".format(index))

        if self.args.settle > 0:
            log("[drone{}] 稳定观察 {:.0f}s 后继续下一架".format(index, self.args.settle))
            time.sleep(self.args.settle)
        self.report["drones"].append(entry)

    # --- 条目与并发等待（batch 模式）--------------------------------------
    def report_entry(self, index):
        entry = self._report_entries.get(index)
        if entry is None:
            entry = {"id": index, "model": model_name(index), "seconds": {}}
            self._report_entries[index] = entry
            self.report["drones"].append(entry)
        return entry

    def ensure_subscribers(self):
        """为每架缓存 mavros/state 与位姿，避免逐架阻塞等待（batch 模式提速关键）。"""
        if self._subscribers:
            return
        for index in self.drone_ids:
            self._subscribers.append(rospy.Subscriber(
                "/typhoon_h480_{}/mavros/state".format(index), State,
                lambda message, i=index: self._states.__setitem__(i, message)))
            self._subscribers.append(rospy.Subscriber(
                "/typhoon_h480_{}/mavros/local_position/pose".format(index), PoseStamped,
                lambda message, i=index: self._poses.__setitem__(i, message)))

    def wait_all_connected(self):
        """并发等待全部无人机 connected=True 且位姿可用；只受一个总 deadline 限制。"""
        deadline = time.time() + self.args.mavros_timeout
        pending = list(self.drone_ids)
        last_report = 0.0
        last_px4_check = 0.0
        while pending and time.time() < deadline and not rospy.is_shutdown():
            for index in list(pending):
                state = self._states.get(index)
                pose = self._poses.get(index)
                if state is None or not state.connected or pose is None:
                    continue
                pending.remove(index)
                self.report_entry(index)["connected"] = True
                log("[drone{}] MAVROS connected=True（mode={}）且位姿可用："
                    "x={:.2f} y={:.2f} z={:.2f}".format(
                        index, state.mode, pose.pose.position.x,
                        pose.pose.position.y, pose.pose.position.z))
            if pending:
                now = time.time()
                if now - last_px4_check > 1.0:
                    last_px4_check = now
                    for index in list(pending):
                        self.check_px4_startup(index)
                if now - last_report > 5.0:
                    last_report = now
                    log("  等待 connected：{}（剩 {:.0f}s）".format(pending, deadline - now))
                time.sleep(0.2)
        if pending:
            raise BringupError("以下无人机在 {}s 内未 connected=True：{}".format(
                self.args.mavros_timeout, pending))
        log("全部 {} 架 MAVROS connected=True".format(len(self.drone_ids)))

    # --- batch 模式：串行生成 + 错峰并发启动 --------------------------------
    def bringup_batch(self):
        """保留关键保证，同时把"逐架等 connected"改为并发：

        1. 严格串行生成并确认每个模型（模型先于对应 PX4 的保证不变，且仍是
           一次只发一个生成请求，避免 Gazebo 生成服务被并发请求压住）；
        2. 按 drone 顺序错峰启动 PX4 + MAVROS（默认每架间隔 0.5s）；
        3. 一次性并发等待全部 connected=True + 位姿可用；
        4. 之后仍由健康门统一验收。
        """
        for index in self.drone_ids:
            started = time.time()
            self.spawn_model(index)
            self.report_entry(index)["seconds"]["spawn"] = round(time.time() - started, 1)

        last_index = self.drone_ids[-1]
        for index in self.drone_ids:
            entry = self.report_entry(index)
            started = time.time()
            px4 = self.start_px4(index)
            entry["px4_pid"] = px4.pid
            self.start_mavros(index)
            entry["seconds"]["launch"] = round(time.time() - started, 1)
            if index != last_index and self.args.px4_stagger > 0:
                time.sleep(self.args.px4_stagger)

        self.ensure_subscribers()
        self.wait_all_connected()

        if self.args.settle > 0:
            log("全部 connected，统一稳定观察 {:.0f}s".format(self.args.settle))
            time.sleep(self.args.settle)

    # --- 阶段 2：控制桥 ----------------------------------------------------
    def start_bridge(self):
        log("启动 xtdrone 控制桥（{} 架：{}）".format(
            self.args.num_drones, self.drone_ids))
        if self.args.start_index == 0:
            # 标准路径：bridge.launch 递归启动 0..N-1
            log_path = os.path.join(LOG_DIR, "bridge.log")
            command = [
                "roslaunch", "xtdrone_mavros_bridge", "bridge.launch",
                "num_drones:={}".format(self.args.num_drones),
            ]
            return self.start_process(command, log_path)

        # 非 0 起始 id（断点续启）：bridge.launch 只支持 0..N-1，这里逐机启动。
        processes = []
        for index in self.drone_ids:
            log_path = os.path.join(LOG_DIR, "bridge_{}.log".format(index))
            command = [
                "rosrun", "xtdrone_mavros_bridge", "xtdrone_mavros_bridge.py",
                "__ns:=typhoon_h480_{}".format(index),
                "_vehicle:=typhoon_h480_{}".format(index),
                "_setpoint_rate:=30.0",
                "_velocity_timeout:=0.5",
            ]
            processes.append(self.start_process(command, log_path))
        return processes[-1] if processes else None

    # --- 阶段 3：健康门 ----------------------------------------------------
    def health_gate(self):
        log("=" * 60)
        log("健康门检查")
        log("=" * 60)
        failures = []
        warnings = []

        try:
            world = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
            models = set(world().model_names)
        except rospy.ServiceException as exc:
            failures.append("Gazebo 世界服务无响应：{}".format(exc))
            models = set()

        for entry in self.report["drones"]:
            index = entry["id"]
            name = entry["model"]
            problems = []

            if name not in models:
                problems.append("模型不在 Gazebo 中")
            if entry["px4_pid"] and self._pid_alive(entry["px4_pid"]) is False:
                problems.append("PX4 进程已退出")

            try:
                state = rospy.wait_for_message(
                    "/typhoon_h480_{}/mavros/state".format(index), State, timeout=3.0)
                if not state.connected:
                    problems.append("MAVROS 未连接")
            except rospy.ROSException:
                problems.append("MAVROS 状态话题无数据")

            try:
                pose = rospy.wait_for_message(
                    "/typhoon_h480_{}/mavros/local_position/pose".format(index),
                    PoseStamped, timeout=3.0)
                if abs(pose.pose.position.z) > 50.0:
                    problems.append("位姿 z={:.1f} 异常（疑似 EKF 发散）".format(
                        pose.pose.position.z))
            except rospy.ROSException:
                problems.append("位姿话题无数据")

            # 原始激光最多等 3 次：模型 SDF 已把激光改成 <visualize>false</visualize>
            # （2026-09-25 排查：GUI 会订阅 visualize=true 的激光可视化话题，GUI 卡顿时
            #  gz-transport 发布队列溢出，server 日志出现
            #  "Queue limit reached for topic .../laser/scan, deleting message"，
            #  gazebo_ros_laser 桥接拿不到扫描 -> /<drone>/scan 静默）。
            # 这里再留重试兜住瞬时丢包/慢启动，避免整轮被健康门拒绝。
            for attempt in range(1, 4):
                try:
                    scan = rospy.wait_for_message(
                        "/typhoon_h480_{}/scan".format(index), LaserScan,
                        timeout=3.0)
                except rospy.ROSException:
                    if attempt < 3:
                        log("[drone{}] 原始激光话题暂无数据，重试 {}/3 ...".format(
                            index, attempt))
                        time.sleep(0.5)
                    else:
                        problems.append("原始激光话题无数据")
                    continue
                valid = [r for r in scan.ranges if 0.0 < r < scan.range_max]
                entry["scan_valid"] = len(valid)
                if len(valid) < 30:
                    warnings.append("[drone{}] 原始激光有效点仅 {} 个".format(index, len(valid)))
                break

            entry["problems"] = problems
            if problems:
                failures.append("[drone{}] {}".format(index, "；".join(problems)))
            else:
                log("[drone{}] OK  模型✓ 位姿✓ 激光✓（有效点 {}）".format(
                    index, entry.get("scan_valid", "?")))

        # gzserver 控制台里的 Queue limit 告警 = GUI 卡顿把共享发布队列打满，
        # 丢的正是扫描消息（见 gz_queue_limit_topics 注释）。只告警不判失败：
        # 模型侧已经 visualize=false，这里主要是把真因直接摊在报告里。
        console_log = find_gzserver_console_log()
        queue_topics = gz_queue_limit_topics(console_log)
        if queue_topics:
            warnings.append(
                "gzserver 发布队列溢出（GUI 卡顿丢消息，日志 {}）：{}".format(
                    console_log, ", ".join(queue_topics)))

        # MAVLink 端口冲突：只有本机 offboard 端口（14580+id，MAVROS 依赖）绑定失败
        # 才算致命；其余（airframe .post 里固定写死 14558 的辅助实例、GCS/payload）
        # 只告警——connected=True 已经证明主链路可用。
        for entry in self.report["drones"]:
            index = entry["id"]
            critical, others = self.analyze_mavlink_binds(entry.get("px4_log"), index)
            if critical:
                failures.append(
                    "[drone{}] PX4 offboard 端口 1458{} 绑定失败（Address already in use）".format(
                        index, index))
            if others:
                warnings.append(
                    "[drone{}] PX4 其他 mavlink 端口绑定失败（不影响导航主链路）：{}".format(
                        index, ", ".join(str(port) for port in others)))

        if not self._pid_alive_by_name("gzserver"):
            failures.append("gzserver 未在运行")

        self.report["failures"] = failures
        self.report["warnings"] = warnings
        for warning in warnings:
            log("WARN  {}".format(warning))
        for failure in failures:
            log("FAIL  {}".format(failure))

        report_path = os.path.join(LOG_DIR, "health_report.json")
        with open(report_path, "w") as handle:
            json.dump(self.report, handle, indent=2, sort_keys=True)
        log("健康门报告已写入 {}".format(report_path))
        return not failures

    def analyze_mavlink_binds(self, path, index):
        """返回 (critical_ports, other_ports)：绑定失败的 MAVLink UDP 端口。

        PX4 每个实例都会按 airframe 的 6011_typhoon_h480.post 尝试
        `mavlink start -u 14558 ...`，因此多机时只有一架能绑上，其余实例都会打印
        'bind failed'。这不影响导航：MAVROS 依赖的是 offboard 端口 14580+id。

        日志里的因果是相邻两行，例如：
            INFO [mavlink] mode: Onboard, ... on udp port 14582 remote port 14542
            WARN [mavlink] bind failed: Address already in use
        """
        if not path or not os.path.exists(path):
            return [], []
        with open(path, "rb") as handle:
            lines = handle.read().decode("utf-8", "replace").splitlines()
        failed = []
        for position, line in enumerate(lines):
            match = re.search(r"on udp port (\d+)", line)
            if not match:
                continue
            following = ""
            for offset in (1, 2):
                if position + offset < len(lines) and lines[position + offset].strip():
                    following = lines[position + offset]
                    break
            if "bind failed" in following.lower():
                failed.append(int(match.group(1)))
        critical = [port for port in failed if port == 14580 + index]
        others = [port for port in failed if port != 14580 + index]
        return critical, others

    def _pid_alive(self, pid):
        try:
            os.kill(pid, 0)
        except OSError:
            return False
        return True

    def _pid_alive_by_name(self, name):
        try:
            subprocess.check_output(["pgrep", "-x", name])
            return True
        except (subprocess.CalledProcessError, OSError):
            return False

    # --- 主流程 ------------------------------------------------------------
    def run(self):
        self.wait_for_world()
        self.ensure_airframe_alias()
        log("SDF lockstep = {}（SDF_LOCKSTEP=1 可恢复 true；见文件头注释）".format(
            "true" if LOCKSTEP else "false"))

        if self.args.mode == "batch":
            log("启动模式：batch（串行生成模型 + 错峰并发 PX4/MAVROS，stagger={}s）".format(
                self.args.px4_stagger))
            started = time.time()
            try:
                self.bringup_batch()
            except BringupError as exc:
                log("ERROR {}".format(exc))
                raise
            log("batch 启动全部完成，耗时 {:.1f}s（不含世界加载与健康门）".format(
                time.time() - started))
        else:
            for index in self.drone_ids:
                if rospy.is_shutdown():
                    raise BringupError("ROS 已关闭，停止启动流程")
                log("-" * 60)
                log("启动第 {}/{} 架无人机（drone{}，严格串行）".format(
                    index - self.args.start_index + 1, self.args.num_drones, index))
                log("-" * 60)
                started = time.time()
                try:
                    self.bringup_one(index)
                except BringupError as exc:
                    log("ERROR {}".format(exc))
                    if not self.args.continue_on_failure:
                        raise
                    self.report_entry(index)["problems"] = [str(exc)]
                log("[drone{}] 本架耗时 {:.1f}s".format(index, time.time() - started))

        if self.args.start_bridge:
            self.start_bridge()

        healthy = self.health_gate()
        log("=" * 60)
        if healthy:
            log("六机底层启动通过：模型 / PX4 / MAVROS 全部就绪，可以启动上层算法栈")
            # 关键：不要把 PX4/MAVROS/控制桥一起带走，它们要继续跑。
            self.keep_children_alive = True
        else:
            log("底层启动未通过健康门，上层算法栈不应启动")
        log("=" * 60)
        return 0 if healthy else 2


def parse_args(argv):
    parser = argparse.ArgumentParser(description="严格串行的多机仿真启动编排器")
    parser.add_argument("--num-drones", type=int, default=6)
    parser.add_argument("--no-stereo-right", action="store_true",
                        help="remove the unused right depth camera from spawned models")
    parser.add_argument("--start-index", type=int, default=0,
                        help="起始 drone id（断点续启用，默认 0）")
    parser.add_argument("--mode", choices=("batch", "serial"), default="batch",
                        help="batch=串行生成+错峰并发启动（快，默认）；serial=逐架严格串行（最保守）")
    parser.add_argument("--px4-stagger", type=float, default=0.5,
                        help="batch 模式相邻两架 PX4/MAVROS 的启动间隔秒数")
    parser.add_argument("--settle", type=float, default=0.0,
                        help="connected 后的稳定观察秒数（默认 0，健康门已做验收）")
    parser.add_argument("--world-timeout", type=float, default=600.0,
                        help="等待 Gazebo 世界就绪的最长秒数")
    parser.add_argument("--model-timeout", type=float, default=300.0,
                        help="单架模型生成确认的最长秒数")
    parser.add_argument("--mavros-timeout", type=float, default=120.0,
                        help="单架 MAVROS 等待 connected=True 的最长秒数")
    parser.add_argument("--no-bridge", dest="start_bridge", action="store_false",
                        help="不启动 xtdrone 控制桥")
    parser.add_argument("--continue-on-failure", action="store_true",
                        help="某架失败时继续下一架（默认失败即中止）")
    parser.set_defaults(start_bridge=True)
    return parser.parse_args(argv)


def main(argv):
    args = parse_args(argv)
    if not 1 <= args.num_drones <= 6:
        sys.stderr.write("--num-drones 必须在 1~6 之间\n")
        return 1
    if args.start_index < 0 or args.start_index + args.num_drones > 6:
        sys.stderr.write("--start-index + --num-drones 不能超过 6\n")
        return 1
    rospy.init_node("serial_swarm_bringup", anonymous=True, disable_signals=True)
    bringup = SerialSwarmBringup(args)
    rospy.on_shutdown(bringup.shutdown_children)

    def handle_signal(signum, _frame):
        log("收到信号 {}，清理子进程后退出".format(signum))
        bringup.shutdown_children(force=True)
        sys.exit(130)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        return bringup.run()
    except BringupError as exc:
        log("ERROR {}".format(exc))
        return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
