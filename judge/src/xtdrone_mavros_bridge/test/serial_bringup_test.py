#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""serial_swarm_bringup 的纯逻辑回归测试（不需要 Gazebo / ROS）。

覆盖：
    1. analyze_mavlink_binds 能从 PX4 日志里区分"本机 offboard 端口绑定失败"（致命）
       和 airframe .post 固定写死 14558 造成的辅助实例绑定失败（仅告警）；
    2. 端口约定与 PX4 rcS 一致（offboard local = 14580 + instance）；
    3. --start-index / --num-drones 的参数校验；
    4. SDF lockstep 开关（默认 false；SDF_LOCKSTEP=1 恢复 true）；
    5. 激光传感器不得 <visualize>true</visualize>（否则 GUI 订阅可视化话题，
       卡顿时 gz 发布队列溢出，/<drone>/scan 静默）+ 健康门激光重试。

运行：
    python3 src/xtdrone_mavros_bridge/test/serial_bringup_test.py
"""

import importlib.util
import os
import sys
import tempfile
import unittest

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
SCRIPT = os.path.join(REPO_ROOT, "src", "xtdrone_mavros_bridge", "scripts",
                      "serial_swarm_bringup.py")


def load_module():
    spec = importlib.util.spec_from_file_location("serial_swarm_bringup", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


BRINGUP = load_module()


class MavlinkBindTest(unittest.TestCase):
    """PX4 六个实例都会尝试绑定 14558（6011_typhoon_h480.post），
    只有第一架成功，其余打印 'bind failed'——不能据此判定启动失败。"""

    PORT_14558_ONLY = """INFO  [mavlink] mode: Normal, data rate: 4000000 B/s on udp port 18572 remote port 14550
INFO  [mavlink] mode: Onboard, data rate: 4000000 B/s on udp port 14582 remote port 14542
INFO  [mavlink] mode: Onboard, data rate: 4000 B/s on udp port 14282 remote port 14032
INFO  [mavlink] mode: Onboard, data rate: 4000 B/s on udp port 14558 remote port 14530
WARN  [mavlink] bind failed: Address already in use
"""

    OFFBOARD_FAIL = """INFO  [mavlink] mode: Onboard, data rate: 4000000 B/s on udp port 14582 remote port 14542
WARN  [mavlink] bind failed: Address already in use
INFO  [mavlink] mode: Normal, data rate: 4000000 B/s on udp port 18572 remote port 14550
"""

    def _analyze(self, content, index):
        with tempfile.NamedTemporaryFile("w", suffix=".log", delete=False) as handle:
            handle.write(content)
            path = handle.name
        try:
            instance = object.__new__(BRINGUP.SerialSwarmBringup)
            return instance.analyze_mavlink_binds(path, index)
        finally:
            os.unlink(path)

    def test_auxiliary_14558_is_not_fatal(self):
        critical, others = self._analyze(self.PORT_14558_ONLY, 2)
        self.assertEqual(critical, [])
        self.assertEqual(others, [14558])

    def test_offboard_port_failure_is_fatal(self):
        critical, others = self._analyze(self.OFFBOARD_FAIL, 2)
        self.assertEqual(critical, [14582])
        self.assertEqual(others, [])

    def test_missing_log_is_not_fatal(self):
        instance = object.__new__(BRINGUP.SerialSwarmBringup)
        self.assertEqual(instance.analyze_mavlink_binds("/nonexistent/px4.log", 0), ([], []))


class PortConventionTest(unittest.TestCase):
    """与 PX4 ROMFS/init.d-posix/rcS 的端口约定保持一致。"""

    def test_offboard_local_port_matches_rcs(self):
        # rcS: udp_offboard_port_local=$((14580+px4_instance))
        for index in range(6):
            self.assertEqual(14580 + index, 14580 + index)

    def test_mavros_fcu_url_matches_px4_ports(self):
        path = os.path.join(REPO_ROOT, "src", "xtdrone_mavros_bridge", "launch",
                            "mavros_vehicle.launch")
        with open(path) as handle:
            content = handle.read()
        self.assertIn("udp://:1454$(arg drone_id)@localhost:1458$(arg drone_id)", content)


class ArgValidationTest(unittest.TestCase):
    def test_start_index_plus_num_drones_within_six(self):
        args = BRINGUP.parse_args(["--num-drones", "2", "--start-index", "4"])
        self.assertEqual(args.num_drones, 2)
        self.assertEqual(args.start_index, 4)
        self.assertLessEqual(args.start_index + args.num_drones, 6)

    def test_batch_mode_is_default_and_stagger_configurable(self):
        args = BRINGUP.parse_args([])
        self.assertEqual(args.mode, "batch")
        self.assertEqual(args.settle, 0.0)
        args = BRINGUP.parse_args(["--mode", "serial", "--px4-stagger", "1.5"])
        self.assertEqual(args.mode, "serial")
        self.assertEqual(args.px4_stagger, 1.5)

    def test_drone_ids_are_contiguous_from_start_index(self):
        args = BRINGUP.parse_args(["--num-drones", "3", "--start-index", "2"])
        instance = object.__new__(BRINGUP.SerialSwarmBringup)
        instance.args = args
        instance.drone_ids = list(range(args.start_index, args.start_index + args.num_drones))
        self.assertEqual(instance.drone_ids, [2, 3, 4])


class LockstepSwitchTest(unittest.TestCase):
    """6 机 + 随机地图（含 rover）时，SDF 里的 <enable_lockstep>true</enable_lockstep>
    会让 Gazebo 在 sim_time≈4s 死锁（MAVROS 永远 connected=False，退出时 gzserver 139）。
    默认必须把它改写成 false；SDF_LOCKSTEP=1 时保持 true。"""

    SDF_TEMPLATE = ("<sdf version='1.6'><model name='m'>"
                    "<plugin name='mavlink_interface' filename='libgazebo_mavlink_interface.so'>"
                    "<enable_lockstep>true</enable_lockstep><use_tcp>true</use_tcp>"
                    "</plugin></model></sdf>")

    def test_default_disables_lockstep(self):
        xml, count = BRINGUP.apply_lockstep(self.SDF_TEMPLATE)
        self.assertEqual(count, 1)
        expected_false = "<enable_lockstep>false</enable_lockstep>"
        if BRINGUP.LOCKSTEP:      # 导入时若显式设了 SDF_LOCKSTEP=1，则保持 true
            self.assertIn("<enable_lockstep>true</enable_lockstep>", xml)
        else:
            self.assertIn(expected_false, xml)
            self.assertNotIn("<enable_lockstep>true</enable_lockstep>", xml)

    def test_no_lockstep_tag_is_reported(self):
        xml, count = BRINGUP.apply_lockstep("<sdf><model name='m'/></sdf>")
        self.assertEqual(count, 0)

    def test_sdf_files_on_disk_keep_lockstep_true(self):
        """磁盘上的 SDF 不被改写，改写只发生在 spawn 前的内存里。"""
        path = os.path.join(REPO_ROOT, "typhoon_h480_zzufly", "typhoon_h480_0.sdf")
        with open(path) as handle:
            content = handle.read()
        self.assertIn("<enable_lockstep>true</enable_lockstep>", content)


class LaserVisualizationTest(unittest.TestCase):
    """激光传感器不允许 <visualize>true</visualize>。

    2026-09-25 排查 `原始激光话题无数据`（1~4 号，0/5 号偶尔正常）：
    GUI（gzclient）会为 visualize=true 的 ray 传感器创建 LaserVisual 并订阅
    gz 话题 `/gazebo/default/<model>/<link>/laser/scan`；GUI 卡顿时 gzserver 的
    发布队列溢出（server 日志 `Queue limit reached for topic .../laser/scan,
    deleting message`），同一条 gz 发布队列上的 gazebo_ros_laser 桥接被饿死，
    ROS 侧 `/<drone>/scan` 静默 → 健康门拒绝启动上层栈。
    关闭可视化后 GUI 不再订阅该话题，队列只服务进程内本地订阅，不再丢包。
    """

    LASER_BLOCK = "        <visualize>false</visualize>"

    def test_mission_sdfs_do_not_visualize_laser(self):
        import xml.etree.ElementTree as ET
        for index in range(6):
            path = os.path.join(REPO_ROOT, "typhoon_h480_zzufly",
                                "typhoon_h480_{}.sdf".format(index))
            with self.subTest(sdf=os.path.basename(path)):
                with open(path) as handle:
                    content = handle.read()
                self.assertNotIn("<visualize>true</visualize>", content)
                root = ET.fromstring(content)
                lasers = [s for s in root.iter("sensor")
                          if s.get("type") == "ray"]
                self.assertEqual(len(lasers), 1)
                visualize = (lasers[0].findtext("visualize") or "false").strip()
                self.assertEqual(visualize, "false")
                # 传感器本体不能被误删/改名
                plugin = lasers[0].find("plugin")
                self.assertIsNotNone(plugin)
                self.assertEqual(plugin.findtext("topicName"), "scan")
                self.assertGreater(float(lasers[0].findtext("update_rate")), 0.0)

    def test_reference_sdf_also_not_visualized(self):
        path = os.path.join(REPO_ROOT, "typhoon_h480_zzufly",
                            "typhoon_h480_zzufly.sdf")
        with open(path) as handle:
            content = handle.read()
        self.assertNotIn("<visualize>true</visualize>", content)
        self.assertIn(self.LASER_BLOCK, content)

    def test_health_gate_retries_laser_check(self):
        """瞬时丢包时健康门要重试，而不是一次超时就判 FAIL。"""
        with open(SCRIPT) as handle:
            source = handle.read()
        self.assertIn("原始激光话题暂无数据，重试", source)
        self.assertIn("for attempt in range(1, 4):", source)

    def test_queue_limit_topics_are_reported(self):
        """gzserver 控制台出现 Queue limit reached 时要能解析出话题（GUI 卡顿丢消息）。"""
        content = (
            "(1790262355 579545898) [Wrn] [Publisher.cc:135] Queue limit reached for topic "
            "/gazebo/default/typhoon_h480_1/base_link/laser/scan, deleting message. "
            "This warning is printed only once.\n"
            "(1790262355 584090918) [Wrn] [Publisher.cc:135] Queue limit reached for topic "
            "/gazebo/default/typhoon_h480_2/base_link/laser/scan, deleting message. "
            "This warning is printed only once.\n"
            "(1790262355 584090919) [Wrn] [Publisher.cc:135] Queue limit reached for topic "
            "/gazebo/default/typhoon_h480_2/base_link/laser/scan, deleting message. "
            "This warning is printed only once.\n"
            "(1790262345 000000000) [Msg] Loading world file [x.world]\n")
        with tempfile.NamedTemporaryFile("w", suffix=".log", delete=False) as handle:
            handle.write(content)
            path = handle.name
        try:
            topics = BRINGUP.gz_queue_limit_topics(path)
        finally:
            os.unlink(path)
        self.assertEqual(topics, [
            "/gazebo/default/typhoon_h480_1/base_link/laser/scan",
            "/gazebo/default/typhoon_h480_2/base_link/laser/scan",
        ])

    def test_queue_limit_missing_log_is_empty(self):
        self.assertEqual(BRINGUP.gz_queue_limit_topics("/nonexistent/gz.log"), [])
        self.assertEqual(BRINGUP.gz_queue_limit_topics(None), [])


class LockstepEnvTest(unittest.TestCase):
    """子进程里验证环境变量语义（模块级常量在导入时求值）。"""

    def _lockstep_with_env(self, value):
        import subprocess
        env = dict(os.environ)
        env.pop("SDF_LOCKSTEP", None)
        if value is not None:
            env["SDF_LOCKSTEP"] = value
        env["ZZU_FLY_WORKSPACE"] = REPO_ROOT
        code = ("import importlib.util;"
                "spec=importlib.util.spec_from_file_location('m', r'%s');"
                "m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);"
                "print(m.LOCKSTEP)" % SCRIPT)
        out = subprocess.run([sys.executable, "-c", code], env=env,
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.assertEqual(out.returncode, 0, out.stderr.decode())
        return out.stdout.decode().strip() == "True"

    def test_env_semantics(self):
        self.assertFalse(self._lockstep_with_env(None))   # 默认：关闭 lockstep
        self.assertFalse(self._lockstep_with_env("0"))
        self.assertTrue(self._lockstep_with_env("1"))     # 恢复旧行为
        self.assertTrue(self._lockstep_with_env("true"))


class StalePx4Test(unittest.TestCase):
    """残留 PX4 daemon 必须被当场识别出来（否则要干等 120s mavros 超时）。"""

    STALE_LOG = ("===== px4 -i 0 ... =====\n"
                 "INFO  [px4] PX4 daemon already running for instance 0 (Success)\n")

    def test_detects_stale_daemon(self):
        reason = BRINGUP.px4_stale_daemon_reason(self.STALE_LOG)
        self.assertIsNotNone(reason)
        self.assertIn('instance 0', reason)
        self.assertIn('kill_stale_sim', reason)

    def test_detects_instance_number(self):
        self.assertIn('instance 3', BRINGUP.px4_stale_daemon_reason(
            'INFO  [px4] PX4 daemon already running for instance 3 (Success)'))

    def test_normal_log_is_fine(self):
        self.assertIsNone(BRINGUP.px4_stale_daemon_reason(
            'INFO  [px4] Startup script: etc/init.d-posix/rcS\n'
            'INFO  [simulator_mavlink] Simulator connected on TCP port 4560.\n'))
        self.assertIsNone(BRINGUP.px4_stale_daemon_reason(''))
        self.assertIsNone(BRINGUP.px4_stale_daemon_reason(None))

    OFFBOARD_BIND_FAIL = (
        "INFO  [mavlink] mode: Onboard, data rate: 4000000 B/s on udp port 14580 remote port 14540\n"
        "WARN  [mavlink] bind failed: Address already in use\n"
        "INFO  [mavlink] mode: Onboard, data rate: 4000 B/s on udp port 14558 remote port 14530\n"
        "WARN  [mavlink] bind failed: Address already in use\n")

    def test_offboard_bind_failure_is_detected_fast(self):
        """offboard 端口 14580 绑不上 → MAVROS 永远连不通，必须当场报错（别等 120s）。"""
        import tempfile
        tmp = tempfile.mkdtemp()
        path = os.path.join(tmp, 'px4_0.log')
        with open(path, 'w') as handle:
            handle.write(self.OFFBOARD_BIND_FAIL)
        instance = object.__new__(BRINGUP.SerialSwarmBringup)
        old_dir = BRINGUP.LOG_DIR
        BRINGUP.LOG_DIR = tmp
        try:
            with self.assertRaises(BRINGUP.BringupError) as ctx:
                instance.check_px4_startup(0)
            message = str(ctx.exception)
            self.assertIn('offboard', message)
            self.assertIn('14580', message)
            self.assertIn('kill_stale_sim', message)
        finally:
            BRINGUP.LOG_DIR = old_dir

    def test_auxiliary_14558_bind_failure_is_not_fatal_for_startup(self):
        """只有写死 14558 的辅助端口失败（真实多机场景的常态）不能误报。"""
        import tempfile
        tmp = tempfile.mkdtemp()
        with open(os.path.join(tmp, 'px4_1.log'), 'w') as handle:
            handle.write("INFO  [mavlink] mode: Onboard, data rate: 4000 B/s on udp port 14558 remote port 14530\n"
                         "WARN  [mavlink] bind failed: Address already in use\n")
        instance = object.__new__(BRINGUP.SerialSwarmBringup)
        old_dir = BRINGUP.LOG_DIR
        BRINGUP.LOG_DIR = tmp
        try:
            instance.check_px4_startup(1)      # 不应抛异常
        finally:
            BRINGUP.LOG_DIR = old_dir

    def test_check_px4_startup_raises_with_real_log(self):
        import tempfile
        with tempfile.NamedTemporaryFile('w', suffix='.log', delete=False) as handle:
            handle.write(self.STALE_LOG)
            path = handle.name
        try:
            instance = object.__new__(BRINGUP.SerialSwarmBringup)
            old_dir = BRINGUP.LOG_DIR
            BRINGUP.LOG_DIR = os.path.dirname(path)
            os.rename(path, os.path.join(BRINGUP.LOG_DIR, 'px4_9.log'))
            try:
                with self.assertRaises(BRINGUP.BringupError) as ctx:
                    instance.check_px4_startup(9)
                self.assertIn('残留 PX4 daemon', str(ctx.exception))
            finally:
                BRINGUP.LOG_DIR = old_dir
                os.unlink(os.path.join(old_dir if False else os.path.dirname(path), 'px4_9.log'))
        finally:
            if os.path.exists(path):
                os.unlink(path)


if __name__ == "__main__":
    unittest.main(verbosity=2)
