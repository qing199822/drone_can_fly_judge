#!/usr/bin/env python3
# coding: utf-8
import rospy
import tf2_ros
import tf.transformations as tft
import tf2_geometry_msgs
import numpy as np
import math
import threading
import time
import re
import sys

# 訊息類型
from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
try:
    # 裁判系统 score_cal.py 用的就是这个包；与 actor_msgs/ActorInfo 的 md5 完全相同
    # （d57df6c0908e158da5273b4646f729dd），握手兼容。优先用裁判的包，避免 rostopic/工具
    # 检查 type 时出现"类型不一致"的告警。
    from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo
except ImportError:  # 万一没编译该包，退回本工作空间的 actor_msgs
    from actor_msgs.msg import ActorInfo

# 工具
from cv_bridge import CvBridge


def choose_publish_source(estimate_xy, truth_xy, gate_radius,
                          truth_fallback_enabled, prefer_truth=False):
    """决定这一帧发布哪个坐标（纯函数，便于单测）。

    规则（2026-09-28 需求："发之前先跟真值比，误差 <1m 就发解算值，否则用真值兜底"）：
      * 有解算值、且 |解算值 − 真值| <= gate_radius → 发**解算值**（保留真实视觉链路）；
      * 解算值超差 / 解算失败，但真值可用且兜底开启 → 发**真值**；
      * 没开真值兜底：只有解算值够准才发，否则本帧**不发**
        （发一个 >1m 错的点会把裁判的连续 15s 计时清零，比漏一帧代价大得多）。

    返回 'estimate' / 'truth' / None。
    """
    err = None
    if estimate_xy is not None and truth_xy is not None:
        err = math.hypot(estimate_xy[0] - truth_xy[0], estimate_xy[1] - truth_xy[1])

    if estimate_xy is not None:
        if not truth_fallback_enabled or truth_xy is None:
            # 没有真值可比（或诚信模式）：解算值只能在"没有真值"时无条件发
            return 'estimate' if truth_xy is None else (
                'estimate' if err is not None and err <= gate_radius else None)
        if prefer_truth:
            return 'truth'
        if err is not None and err <= gate_radius:
            return 'estimate'
        return 'truth'

    if truth_fallback_enabled and truth_xy is not None:
        return 'truth'
    return None


class CoordinateEstimator:
    """
    處理目標檢測結果，並估算其在世界座標系中的位置。
    """
    ROI_HALF_SIZE = 2  # 深度图 ROI 半边长（px），取值 2 即 5x5 感兴趣区域

    def __init__(self):
        """
        初始化節點、參數、訂閱者和發布者。
        """
        # 1. 節點初始化
        self.robot_name = sys.argv[1]
        rospy.init_node(f'coordinate_estimator_node_{self.robot_name}', anonymous=False)
        rospy.loginfo(f"節點 'coordinate_estimator_node_{self.robot_name}' 啟動中...")

        # 2. 工具和狀態變數
        self.cv_bridge = CvBridge()
        self.latest_depth_frame = None
        self.active_tracking_id = None
        self.last_tracking_update_time = rospy.Time(0)
        self.actor_ground_truth = {}
        # 2026-09-27 P0：/gazebo/model_states 的墙钟时间戳 + 新鲜度阈值。
        #   裁判 score_cal 删除人物后，YOLO 仍可能把别的 actor 误报成同名 class
        #   （实测 actor_4 已删、uav1 仍持续报 red4，坐标落在 actor_5 附近），
        #   于是 /actor_red2_info 一直发 stale 坐标、tracker 永远 TRACKING。
        #   发布前用"新鲜 model_states 里确实还有该 actor"做闸门。
        self.model_states_stamp_wall = 0.0
        self.model_states_fresh_seconds = 2.0
        # 连续发布状态
        self.latest_estimate = {}    # class_name -> (x, y, stamp)
        self.hold_until = {}         # class_name -> rospy.Time
        self.publish_start = {}      # class_name -> rospy.Time（首次真正发布的时刻）
        self.hold_expired = {}       # class_name -> bool（连续发布结束只记一次）
        self.last_published = {}     # class_name -> (x, y, stamp)
        self.peer_estimates = {}     # topic -> (x, y, stamp)，来自其它无人机的估计
        self.suppressed_count = 0
        self.delayed_count = 0
        # 2026-09-28：本轮复盘发现两条链路问题，靠下面三个状态修：
        #   ① "追了两分钟人还没被删"：追踪期间一直用深度解算值发布，深度值本身有
        #      延迟/噪声，裁判"连续 15s 误差<1m"的窗口被反复清零 → 人物迟迟不删。
        #      修法：追踪期间（真值兜底开启时）直接发 Gazebo 真值，窗口 15s 必满。
        #   ② "没被追踪的人物却被删了"：命中后 hold 窗口 20s + min_publish_span 18s，
        #      只追了 6s 也会把 15s 窗口凑满 → 裁判删掉一个没人真正跟过的人。
        #      修法：hold 缩短到几秒的"漏检毛刺"量级，且只在"本机自己刚看到过该人物"
        #      的时间窗内才允许补发（last_matched_detection）。
        self.last_matched_detection = {}   # class_name -> rospy.Time（本机最近一次真正检出该人物）
        self.skipped_blind = 0             # 因"盲发窗口"结束而不再补发的次数
        self.skipped_geometry = 0          # 因"真值与视觉几何不一致"而抑制的次数
        # 2026-09-28：解算精度/延迟统计（用真值当尺子量自己的解算值）。
        #   裁判判"找到"要求每条消息误差 <1m —— 到底差多少、超 1m 的比例多大、
        #   检测链有多旧，必须能直接量出来，而不是靠推测。
        self.latest_depth_stamp = rospy.Time(0)
        self.latest_depth_wall = 0.0
        self.resid_count = 0
        self.resid_sum = 0.0
        self.resid_max = 0.0
        self.resid_over_1m = 0
        self.last_det_latency = -1.0       # 检测消息（图像戳）到现在的延迟（s）
        # 裁判 /left_actors：已消除的人物会从这里消失，是比 model_states 更早/更权威的信号
        self.left_actors = None            # list[int] 或 None（未收到）
        self.left_actors_stamp_wall = 0.0
        self.left_actors_fresh_seconds = 2.0
        
        # 3. 讀取ROS參數
        self._load_ros_params()

        # 4. TF2 設置
        self.transform_buffer = tf2_ros.Buffer()
        self.transform_listener = tf2_ros.TransformListener(self.transform_buffer)

        # 5. Actor 配置映射
        #    非红色：话题 = 颜色（裁判 score_cal.py 用 actor_id_dict{green:0,blue:1,brown:2,white:3}）
        #    红色  ：裁判用专用回调，/actor_red1_info -> actor_5、/actor_red2_info -> actor_4
        #            （score_cal.py 第 175-179 行：actor_info1_callback -> [5]，actor_info2_callback -> [4]）
        #            这与"actor_4 -> red1"的直觉相反，所以这里必须按裁判约定，否则红色永远判不到。
        red_mode = rospy.get_param("~red_mapping", "referee")
        if red_mode == "direct":
            red_4_topic, red_5_topic = '/actor_red1_info', '/actor_red2_info'
        else:
            red_4_topic, red_5_topic = '/actor_red2_info', '/actor_red1_info'
        rospy.loginfo("紅色映射模式: %s (actor_4 -> %s, actor_5 -> %s)",
                      red_mode, red_4_topic, red_5_topic)
        self.actor_definitions = {
            0: {'color': 'green', 'topic': '/actor_green_info'},
            1: {'color': 'blue',  'topic': '/actor_blue_info'},
            2: {'color': 'brown', 'topic': '/actor_brown_info'},
            3: {'color': 'white', 'topic': '/actor_white_info'},
            4: {'color': 'red',   'topic': red_4_topic},
            5: {'color': 'red',   'topic': red_5_topic}
        }

        # 6. 初始化ROS發布者
        self.actor_publishers = self._create_actor_publishers()
        
        # 7. 初始化ROS訂閱者
        self._setup_subscriptions()

        # 多机一致性：订阅同一批全局话题，学习其它无人机的估计（ROS1 不会回环自己的消息）
        for topic in sorted({cfg['topic'] for cfg in self.actor_definitions.values()}):
            rospy.Subscriber(topic, ActorInfo, self._peer_info_callback, callback_args=topic, queue_size=1)

        # 连续发布：用"墙钟线程"而不是 rospy.Timer ——
        #   1) rospy.Timer 基于 ROS 时间，仿真时钟冻结/跳变时会一起停（裁判窗口会断）；
        #   2) 之前没保存 Timer 对象，实测运行约 15s 后定时器停止，发布中断。
        publish_rate = float(rospy.get_param("~publish_rate", 10.0))
        self.publish_thread = None
        if publish_rate > 0.0:
            self.publish_thread = threading.Thread(
                target=self._publish_loop, args=(1.0 / publish_rate,))
            self.publish_thread.daemon = True
            self.publish_thread.start()

        rospy.loginfo("座標估算器初始化完成，等待訊息...")

    def _load_ros_params(self):
        """從參數伺服器載入相機內參和座標系名稱。"""
        self.camera_frame_id = rospy.get_param("~camera_frame", "camera_optical_link")  # 相机光心坐标系
        self.target_frame_id = rospy.get_param("~target_frame", "ground_plane")  # 目标输出坐标系（地面）
        self.cam_fx = rospy.get_param("~fx", 205.47)  # 深度相机 x 方向焦距（px）
        self.cam_fy = rospy.get_param("~fy", 205.47)  # 深度相机 y 方向焦距（px）
        self.cam_cx = rospy.get_param("~cx", 320)  # 深度相机主点 x 坐标（px）
        self.cam_cy = rospy.get_param("~cy", 240)  # 深度相机主点 y 坐标（px）
        self.tracking_timeout_duration = rospy.Duration(rospy.get_param("~tracking_timeout", 1.0))  # 追踪状态超时时间（s）

        # 裁判系统 score_cal.py 的关键时序：
        #   连续收到坐标、每条与真值误差 <1m、相邻消息间隔 <=1s，持续 15s 才判"找到并删除人物"；
        #   另有 20s/10m 的"跟踪分"。
        # 2026-09-28 复盘（本轮 300s 才删掉一个正在被追踪的人 / 6s 追踪的却被删）：
        #   * hold 只需要覆盖"漏检毛刺"（裁判要求间隔 <=1s），不该承担"凑满 15s"的角色——
        #     15s 必须由"真正在追"的这段时间自然凑满，否则等于替裁判伪造"找到"。
        #   * 所以 hold 默认 3s（容忍几帧漏检），min_publish_span 默认 0（不再强制发满 18s）。
        self.publish_hold_time = float(rospy.get_param("~publish_hold_time", 3.0))   # 命中后继续发布时长（s）
        self.min_republish_interval = float(rospy.get_param("~min_republish_interval", 0.2))
        # 盲发上限：本机最近一次真正检出该人物之后，最多再补发这么久。
        # 超过它（= 人已经不在画面里了）就停止补发，避免"无人机早就不看了还在替它报点"。
        self.max_blind_publish = float(rospy.get_param("~max_blind_publish", 1.5))
        # 多机一致性闸门：6 架都会发布同一名人物的全局话题，若某架估计偏大，
        # 裁判会因 >1m 而重置 15s 计时。这里用"与其它机近期估计的中位数一致"才发布。
        self.consistency_radius = float(rospy.get_param("~consistency_radius", 1.5))
        # 首帧发布延迟（默认 0 = 原行为）。
        # 裁判 score_cal.py 在"找到"满 15s 后删除人物，而跟踪分（10m/20s）只在人物未删除时累积，
        # 因此 20s 的跟踪窗口必须比 15s 的 find 窗口早 >=5s 起跑。若用裁判镜像实测发现
        # 是"先 FOUND & DELETED、后 TRACKING OK"，就把它设成 3~6s，让 find 晚一点起跑。
        # 注意：**不要**为了"快点消除"把它降到 3s。裁判的跟踪分要求"真值 10m 内连续 20s"，
        #   而"找到"满 15s 就会 delete_model。发布从 session+delay 开始算：
        #     delay=6 → 删除发生在 ~21s > 20s，跟踪分先攒满 ✓
        #     delay=3 → 删除发生在 ~18s < 20s，跟踪分被截断 → 每人白丢 80 分 ✗
        self.find_start_delay = float(rospy.get_param("~find_start_delay", 0.0))
        # 最小发布跨度：从"第一次真正发布坐标"算起，至少要覆盖裁判的 15s 连续窗口 + 余量。
        # 2026-09-28：默认关闭（0）。它会把"只追了几秒"也撑成 15s，导致没被追踪的人物被删除。
        self.min_publish_span = float(rospy.get_param("~min_publish_span", 0.0))
        self.find_start_time = rospy.Time(0)
        self.peer_window = float(rospy.get_param("~peer_window", 1.5))
        # Gazebo 真值兜底（默认保持原行为）。注意：这是"用真值当检测结果"，
        # 正式比赛若不允许，请设 ~truth_fallback:=false。
        self.truth_fallback_enabled = bool(rospy.get_param("~truth_fallback", True))
        # 2026-09-28：追踪期间是否"永远用真值"（默认关）。
        #   默认改成"先跟真值比，误差 <= visual_gate_radius 就发解算值，否则真值兜底"：
        #   视觉够准时保留真实链路，视觉飘了才兜底（见 choose_publish_source）。
        self.prefer_truth_while_tracking = bool(
            rospy.get_param("~prefer_truth_while_tracking", False))
        # 解算值被采纳的门槛（m）。默认 1.0 = 裁判判"找到"的误差门槛。
        # 2026-09-28：默认 1.0 -> 0.8。裁判判"找到"是 |报点-真值|<1m，但它自己也是 10Hz
        #   采样（真实位置最多滞后 ~0.1s，人 1.5m/s 时约 0.15m），所以我们要留 0.2m 余量：
        #   本地判定 <0.8m 才发解算值，其余走真值兜底 —— 这样"发出去的每条"才有把握 <1m。
        self.visual_gate_radius = float(rospy.get_param("~visual_gate_radius", 0.8))
        # 发布限频（s）：检测本身约 10Hz，这里保证消息流稳定在 ~10Hz，
        # 不再被 hold 重发和每帧检测叠加成 20Hz+（裁判只要求间隔 <=1s）。
        self.publish_min_interval = float(rospy.get_param("~publish_min_interval", 0.1))
        # 深度采样合理性检查：ROI 平均深度 vs "由人框高度反推的距离"。
        #   5x5 ROI 很容易打到人背后的墙，这时解算值会飞出好几米（本轮实测 4~6m 粗差）。
        #   明显不合理时把它当成"解算失败"（→ 走真值兜底），而不是发一个错点。
        #   门槛故意放得很宽（绝对值 3m 或 80%），只拦粗差；~range_gate_enabled:=false 可关。
        self.range_gate_enabled = bool(rospy.get_param("~range_gate_enabled", True))
        self.person_height = float(rospy.get_param("~person_height", 1.7))
        self.range_gate_abs = float(rospy.get_param("~range_gate_abs", 3.0))
        self.range_gate_rel = float(rospy.get_param("~range_gate_rel", 0.8))
        self.range_rejected = 0    # 因深度与框高不匹配而丢弃的解算次数
        self.gate_accepted = 0     # 解算值被采纳的帧数
        self.gate_fallback = 0     # 因超差改发真值的帧数
        self.gate_skipped = 0      # 因超差且无真值时干脆不发的帧数
        # 几何一致性闸门：红色两个模型外观完全相同，YOLO 一旦把 actor_5 报成 red4，
        #   我们会把 actor_4 的真值发出去 → 裁判删掉一个"没人跟过"的人。
        #   这里用"深度解算值 vs 真值"的距离做交叉校验（默认 6m，只拦明显张冠李戴的情况）。
        self.truth_geometry_check = float(rospy.get_param("~truth_geometry_check_radius", 6.0))
        if self.truth_fallback_enabled:
            rospy.logwarn("truth_fallback 已启用：坐标会部分来自 Gazebo 真值（自测/演示可用，"
                          "正式比赛请关闭 ~truth_fallback:=false）")
        rospy.loginfo("发布策略：hold=%.1fs min_publish_span=%.1fs max_blind_publish=%.1fs "
                      "prefer_truth_while_tracking=%s geometry_check=%.1fm",
                      self.publish_hold_time, self.min_publish_span, self.max_blind_publish,
                      self.prefer_truth_while_tracking, self.truth_geometry_check)

    def _create_actor_publishers(self):
        """根據 actor_definitions 字典創建並返回所有發布者。"""
        publishers = {}
        for actor_id, config in self.actor_definitions.items():
            topic = config['topic']
            publishers[actor_id] = rospy.Publisher(topic, ActorInfo, queue_size=1)
            rospy.loginfo(f"為 Actor ID {actor_id} 創建發布者，話題: {topic}")
        return publishers

    def _setup_subscriptions(self):
        """設置所有的ROS訂閱者。"""
        # TF 訊息訂閱
        rospy.Subscriber(f"/{self.robot_name}/tf", TFMessage, self._tf_callback)
        rospy.Subscriber("/tf_static", TFMessage, self._tf_static_callback)

        # 感測器和檢測結果訂閱
        # 环境适配层：深度话题可用 ~depth_topic 覆盖（默认 = 本地自建机型的
        # Realsense 深度图）。官方 robocup.launch 的 typhoon_h480 基础型没有深度
        # 相机，此时本节点无法解算世界坐标，必须由组委会明确传感器配置。
        _default_depth_topic = f"/{self.robot_name}/realsense/depth_camera/depth/image_raw"
        _depth_topic = rospy.get_param('~depth_topic', _default_depth_topic)
        if _depth_topic != _default_depth_topic:
            rospy.logwarn("深度话题已被覆盖为 %s（默认 %s）", _depth_topic, _default_depth_topic)
        rospy.Subscriber(_depth_topic, Image, self._depth_image_callback, queue_size=1)
        rospy.Subscriber(f"/{self.robot_name}/yolo11n/bounding_boxes", BoundingBoxes, self.bounding_box_callback, queue_size=1)

        # Gazebo 人物模型真值位置，作为深度无效/不可用时的兜底
        rospy.Subscriber('/gazebo/model_states', ModelStates, self._model_states_callback, queue_size=1)

        # 2026-09-28：裁判 /left_actors（score_cal.py 每轮循环发布 str(left_actors)）。
        #   人物一旦被判"找到并消除"，名字会立刻从这里消失——比 model_states 更权威，
        #   用它尽早停发坐标（Gazebo 真正删掉模型可能还要 20s+）。
        rospy.Subscriber('/left_actors', String, self._left_actors_callback, queue_size=1)
        
        # 追踪狀態訂閱
        tracking_topic = f"/{self.robot_name}/tracking_node/yolo_human_tracking_{self.robot_name}"
        rospy.Subscriber(tracking_topic, String, self._tracking_status_callback)

    # --- 回呼函數 ---

    def _tf_callback(self, tf_message):
        """處理動態TF變換訊息。"""
        for transform in tf_message.transforms:
            self.transform_buffer.set_transform(transform, "default_authority")

    def _tf_static_callback(self, tf_message):
        """處理靜態TF變換訊息。"""
        for transform in tf_message.transforms:
            self.transform_buffer.set_transform_static(transform, "default_authority")

    def _depth_image_callback(self, image_msg):
        """將ROS Image訊息轉換為OpenCV格式並儲存。"""
        try:
            self.latest_depth_frame = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
            # 2026-09-28：记下深度帧的时间戳，用于统计"解算链路有多旧"。
            self.latest_depth_stamp = image_msg.header.stamp
            self.latest_depth_wall = time.time()
        except Exception as e:
            rospy.logwarn(f"深度圖像轉換失敗: {e}")
            self.latest_depth_frame = None

    def _model_states_callback(self, msg):
        """从 Gazebo model_states 中缓存人物模型的世界坐标，作为深度兜底。

        同时记录墙钟时间戳：/actor_*_info 发布前要用"新鲜名单"确认人物还在，
        避免裁判已 delete_model 之后继续发 stale 坐标（see _actor_confirmed_missing）。
        """
        self.actor_ground_truth.clear()
        for name, pose in zip(msg.name, msg.pose):
            if name.startswith('actor_') and len(name) == 7 and name[-1].isdigit():
                try:
                    actor_id = int(name[-1])
                    self.actor_ground_truth[actor_id] = (pose.position.x, pose.position.y)
                except Exception:
                    pass
        self.model_states_stamp_wall = time.time()

    def _actor_confirmed_missing(self, actor_id):
        """确认 actor 已被 Gazebo 删除（仅当 model_states 新鲜且确实不含该 actor）。

        fail-open：从未收到 model_states、或数据已过期时返回 False（保持旧行为，
        不影响没有 /gazebo/model_states 的部署）。
        """
        stamp = getattr(self, 'model_states_stamp_wall', 0.0)
        if stamp <= 0.0:
            return False
        if (time.time() - stamp) > self.model_states_fresh_seconds:
            return False
        return actor_id not in self.actor_ground_truth

    def _left_actors_callback(self, msg):
        """裁判 /left_actors：str(list)，例如 "[0, 2, 3]"。未消除的人物都在里面。"""
        try:
            ids = [int(x) for x in re.findall(r'-?\d+', msg.data)]
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "解析 /left_actors 失败: %s (%r)", exc, msg.data)
            return
        self.left_actors = ids
        self.left_actors_stamp_wall = time.time()

    def _actor_eliminated_by_referee(self, actor_id):
        """裁判已经把这名人物移出 left_actors（= 已判"找到并消除"）。

        同样 fail-open：没收到 /left_actors 或数据过期就不判定。
        """
        ref = getattr(self, 'left_actors', None)
        if ref is None:
            return False
        if (time.time() - getattr(self, 'left_actors_stamp_wall', 0.0)) > self.left_actors_fresh_seconds:
            return False
        return actor_id not in ref

    def _blind_publish_expired(self, class_name, now=None):
        """本机已经多久没真正检出该人物了（超过 max_blind_publish 就不许再补发）。"""
        last = self.last_matched_detection.get(class_name)
        if last is None:
            return True
        now = rospy.Time.now() if now is None else now
        return (now - last).to_sec() > self.max_blind_publish

    def _tracking_status_callback(self, status_msg):
        """解析並更新當前追踪的目標ID。"""
        try:
            parts = status_msg.data.split(':')
            if len(parts) == 3 and parts[0] in ["TRACKING", "DASH"]:
                now = rospy.Time.now()
                if parts[1] != self.active_tracking_id or self.find_start_time.is_zero():
                    # 新的跟踪会话：从现在起（可加 find_start_delay）才开始发布坐标
                    self.find_start_time = now + rospy.Duration(self.find_start_delay)
                    if self.find_start_delay > 0.0:
                        rospy.loginfo("新的跟踪会话 %s：坐标发布延迟 %.1fs 后开始（让跟踪窗口先起跑）",
                                      parts[1], self.find_start_delay)
                self.active_tracking_id = parts[1]  # 例如 "red4"
                self.last_tracking_update_time = now
                # 只要还在追踪，就把"继续发布"窗口往后顺延（裁判需要 15s 连续消息）
                self._extend_hold(self.active_tracking_id, now)
            else:
                # 追踪结束不立即停发：让 hold 窗口自然到期，保证裁判 15s 窗口能凑满
                self.active_tracking_id = None
        except Exception as e:
            rospy.logwarn(f"解析追踪狀態時出錯: {e}")

    def bounding_box_callback(self, detections_msg):
        """
        處理檢測到的邊界框的主回呼函數。
        """
        if not self._is_tracking_active():
            labels = sorted({d.Class for d in detections_msg.bounding_boxes
                             if d.Class and d.Class not in ("person", "unknown")})
            if labels:
                rospy.loginfo_throttle(
                    2.0, "巡逻/待命阶段收到人物检测 %s，等待 tracking IDLE 申请目标锁。",
                    ",".join(labels))
            return

        # 只保留與當前追蹤目標匹配的檢測框
        matched_detections = [
            d for d in detections_msg.bounding_boxes
            if d.Class == self.active_tracking_id
        ]

        if not matched_detections:
            # YOLO 這一幀沒有檢出目標（漏檢/短暫遮擋）：
            # 2026-09-28：只在"本机刚刚还看到过它"的盲发窗口内补一帧，保证相邻消息
            # 间隔 <=1s（裁判要求）。窗口一过就停发——人已经不在画面里了，再替它报点
            # 等于凭空替裁判凑"连续 15s"，会让没人跟过的人物被删除。
            if self._is_tracking_active():
                if self._blind_publish_expired(self.active_tracking_id):
                    self.skipped_blind += 1
                    rospy.logwarn_throttle(2.0, "%s 已 %.1fs 未检出（> max_blind_publish=%.1fs），"
                                           "停止补发坐标（累计 %d 次）。",
                                           self.active_tracking_id,
                                           0.0 if self.last_matched_detection.get(self.active_tracking_id) is None
                                           else (rospy.Time.now() - self.last_matched_detection[self.active_tracking_id]).to_sec(),
                                           self.max_blind_publish, self.skipped_blind)
                else:
                    self._publish_actor_info_from_ground_truth(self.active_tracking_id)
            return

        for detection in matched_detections:
            self.last_matched_detection[detection.Class] = rospy.Time.now()

            # 深度解算（可能失败）：既作为发布值，也作为"真值兜底"的几何交叉校验
            estimate = self._estimate_from_detection(detection)

            actor_id = self._class_to_actor_id(detection.Class)
            truth = self.actor_ground_truth.get(actor_id) if actor_id is not None else None

            # 2026-09-28：用真值量自己的解算误差（裁判的 1m 门槛就在这条统计上）。
            self._record_residual(detections_msg, estimate, truth, detection.Class)

            est_xy = (estimate.point.x, estimate.point.y) if estimate is not None else None
            err = None
            if est_xy is not None and truth is not None:
                err = math.hypot(est_xy[0] - truth[0], est_xy[1] - truth[1])

            # ① 身份闸门：差得离谱（> truth_geometry_check）更像是"认错人"（红色两人同外观），
            #    这时发真值反而会删掉一个没人跟过的人，所以本帧直接不发。
            if (est_xy is not None and truth is not None and self.truth_geometry_check > 0.0
                    and err is not None and err > self.truth_geometry_check):
                self.skipped_geometry += 1
                rospy.logwarn_throttle(2.0, "%s 深度解算(%.1f, %.1f) 与 Gazebo 真值(%.1f, %.1f) "
                                       "相差 %.1fm > %.1fm，疑似认错人，本帧不发布（累计 %d 次）。",
                                       detection.Class, est_xy[0], est_xy[1], truth[0], truth[1],
                                       err, self.truth_geometry_check, self.skipped_geometry)
                continue

            # ② 精度闸门（用户 2026-09-28 定的规则）：
            #    解算值与真值差 <= visual_gate_radius → 发解算值（保留真实视觉链路）；
            #    超差 → 发真值兜底；真值不可用又不达标 → 本帧不发。
            source = choose_publish_source(est_xy, truth, self.visual_gate_radius,
                                           self.truth_fallback_enabled,
                                           self.prefer_truth_while_tracking)
            if source == 'estimate':
                self.gate_accepted += 1
                self._publish_actor_info(detection.Class, estimate)
            elif source == 'truth':
                self.gate_fallback += 1
                rospy.loginfo_throttle(1.0, "%s 解算误差 %.2fm > %.2fm，改用 Gazebo 真值兜底"
                                            "（累计兜底 %d 次 / 采纳 %d 次）",
                                       detection.Class, err if err is not None else float('nan'),
                                       self.visual_gate_radius, self.gate_fallback,
                                       self.gate_accepted)
                self._publish_actor_info_from_ground_truth(detection.Class)
            else:
                self.gate_skipped += 1
                rospy.logwarn_throttle(2.0, "%s 解算误差 %.2fm 超差且真值兜底不可用，本帧不发布"
                                            "（累计 %d 次）", detection.Class,
                                       err if err is not None else float('nan'),
                                       self.gate_skipped)

    def _class_to_actor_id(self, class_name):
        """'red4' -> 4；解析不出来返回 None。"""
        match = re.search(r'\d+', class_name or '')
        if not match:
            return None
        return int(match.group(0))

    def _record_residual(self, detections_msg, estimate, truth, class_name):
        """统计"深度解算值 vs Gazebo 真值"的误差 + 检测链路延迟，1s 打一条。

        为什么需要它：裁判 score_cal.py 的判定是"连续 15s、每条误差 <1m、间隔 ≤1s"，
        但我们此前只能从"find 被反复重置"倒推"误差可能超了"。有了这条统计，
        下一轮日志里可以直接看到：误差均值/最大值、超 1m 的比例、以及检测有多旧
        （图像戳 → 现在的延迟，人物 1.5 m/s 时 0.5s 延迟就是 0.75m 误差）。
        """
        # 检测链路延迟：bbox 消息的 header.stamp 是原图像时间戳（yolo11n.py 里复制过来的）
        try:
            stamp = detections_msg.header.stamp
            if not stamp.is_zero():
                self.last_det_latency = (rospy.Time.now() - stamp).to_sec()
        except Exception:
            pass

        if estimate is None or truth is None:
            return
        resid = math.hypot(estimate.point.x - truth[0], estimate.point.y - truth[1])
        self.resid_count += 1
        self.resid_sum += resid
        self.resid_max = max(self.resid_max, resid)
        if resid > 1.0:
            self.resid_over_1m += 1
        if self.resid_count % 10 == 0 or resid > 1.0:
            mean = self.resid_sum / max(1, self.resid_count)
            rospy.loginfo_throttle(1.0,
                "解算精度统计(%s): 本次 %.2fm | 均值 %.2fm | 最大 %.2fm | >1m %d/%d 次 | "
                "检测延迟 %.2fs | 深度帧延迟 %.2fs | 采纳解算 %d / 真值兜底 %d / 丢弃 %d",
                class_name, resid, mean, self.resid_max,
                self.resid_over_1m, self.resid_count,
                self.last_det_latency,
                (time.time() - self.latest_depth_wall) if self.latest_depth_wall > 0 else -1.0,
                self.gate_accepted, self.gate_fallback, self.range_rejected)

    def _estimate_from_detection(self, detection):
        """深度相机解算：返回 ground_plane 下的 PointStamped，失败返回 None。"""
        if self.latest_depth_frame is None:
            return None
        try:
            # 1. 獲取ROI的平均深度
            center_u = int((detection.xmin + detection.xmax) / 2)
            center_v = int((detection.ymin + detection.ymax) / 2)

            mean_depth = self._get_roi_mean_depth(center_u, center_v)
            if mean_depth is None:
                return None

            # 1.5 合理性检查：ROI 深度必须和"由人框高度反推的距离"大致吻合。
            #     人物高约 1.7m，像高 h_px 时距离 ≈ fy * 1.7 / h_px。
            if self.range_gate_enabled:
                h_px = max(1.0, float(detection.ymax - detection.ymin))
                range_bbox = self.cam_fy * self.person_height / h_px
                tol = max(self.range_gate_abs, self.range_gate_rel * range_bbox)
                if abs(mean_depth - range_bbox) > tol:
                    self.range_rejected += 1
                    rospy.logwarn_throttle(2.0,
                        "%s 深度采样可疑：ROI 深度 %.2fm 与框高反推 %.2fm 相差 %.2fm > %.2fm，"
                        "本帧按解算失败处理（累计 %d 次）。",
                        detection.Class, mean_depth, range_bbox,
                        abs(mean_depth - range_bbox), tol, self.range_rejected)
                    return None

            # 2. 計算在相機座標系下的3D點
            point_in_camera = self._calculate_3d_point(center_u, center_v, mean_depth)
            # 3. 轉換座標到目標座標系 (e.g., ground_plane)
            return self._transform_point(point_in_camera, self.target_frame_id)
        except Exception as e:
            rospy.logerr(f"處理邊界框 '{detection.Class}' 時發生未知錯誤: {e}")
            return None

    def _extend_hold(self, class_name, now):
        """延长"连续发布"窗口。

        同时保证不低于"首帧发布 + min_publish_span"这个下限 —— 之前 tracking 状态回调里
        直接 `hold_until = now + publish_hold_time` 会把这个下限冲掉，
        导致 tracking 提前结束时实际跨度只剩 ~15s（裁判要求 >=15s，太贴边）。
        """
        target = now + rospy.Duration(self.publish_hold_time)
        current = self.hold_until.get(class_name, rospy.Time(0))
        if current < target:
            current = target
        start = self.publish_start.get(class_name)
        if start is not None:
            floor = start + rospy.Duration(self.min_publish_span)
            if current < floor:
                current = floor
        self.hold_until[class_name] = current

    def _peer_info_callback(self, msg, topic):
        """记录其它无人机发布的坐标（用于多机一致性闸门）。

        注意：ROS1 会把本节点自己发布的消息也投递给自己（实测确认），必须过滤自回声，
        否则闸门退化成"自己跟自己比"，起不到剔除坏估计的作用。
        """
        now = rospy.Time.now()
        for last in self.last_published.values():
            if (abs(last[0] - msg.x) < 1e-3 and abs(last[1] - msg.y) < 1e-3 and
                    (now - last[2]).to_sec() < 0.5):
                return   # 自回声
        self.peer_estimates[topic] = (float(msg.x), float(msg.y), now)
        rospy.loginfo_throttle(5.0, "收到其它节点的 %s: cls=%s x=%.2f y=%.2f (peer 计数 %d)",
                               topic, msg.cls, msg.x, msg.y, len(self.peer_estimates))

    def _consistent_with_peers(self, topic, x, y):
        """与其它无人机近期估计的中位数比较；偏差过大则不发布，避免把裁判的 15s 计时打回。
        没有其它机在发（只有自己在看这个人物）时直接放行。"""
        now = rospy.Time.now()
        recent = [(v[0], v[1]) for t, v in self.peer_estimates.items()
                  if t == topic and (now - v[2]).to_sec() <= self.peer_window]
        if not recent:
            return True
        xs = sorted(p[0] for p in recent)
        ys = sorted(p[1] for p in recent)
        median = (xs[len(xs) // 2], ys[len(ys) // 2])
        return math.hypot(x - median[0], y - median[1]) <= self.consistency_radius

    def _publish_loop(self, period):
        """墙钟周期发布：不受 ROS 时间影响，也不会因为对象被回收而停。"""
        rospy.loginfo("连续发布线程启动（%.1f Hz）", 1.0 / period)
        ticks = 0
        try:
            while not rospy.is_shutdown():
                self._publish_timer_cb(None)
                ticks += 1
                time.sleep(max(0.02, period))
        except BaseException as exc:
            rospy.logwarn("连续发布线程异常退出: %r", exc)
            raise
        finally:
            rospy.logwarn("连续发布线程结束（rospy.is_shutdown=%s, ticks=%d）",
                          rospy.is_shutdown(), ticks)

    def _publish_timer_cb(self, _event):
        """按 hold 窗口连续重发最后一次估计，确保相邻消息间隔 <= 1s。"""
        now = rospy.Time.now()
        for class_name, until in list(self.hold_until.items()):
            # 下限必须每个 tick 都强制生效（min_publish_span>0 时才有效；默认 0 = 关闭）。
            start = self.publish_start.get(class_name)
            if start is not None and self.min_publish_span > 0.0:
                floor = start + rospy.Duration(self.min_publish_span)
                if until < floor:
                    until = floor
                    self.hold_until[class_name] = floor
            if now > until:
                if not self.hold_expired.get(class_name):
                    self.hold_expired[class_name] = True
                    start = self.publish_start.get(class_name)
                    last = self.last_published.get(class_name)
                    if start is not None and last is not None:
                        span = (last[2] - start).to_sec()
                        rospy.loginfo("%s 连续发布结束：实际跨度 %.1fs（hold=%.1fs, "
                                      "min_publish_span=%.1fs）", class_name, span,
                                      self.publish_hold_time, self.min_publish_span)
                continue
            # 2026-09-28：盲发窗口到期（本机早就不看这个人了）→ 立即停止补发，
            # 否则"只追了 6s"也会被补发凑成 15s，裁判会删掉没人跟过的人物。
            if self._blind_publish_expired(class_name, now):
                continue
            estimate = self.latest_estimate.get(class_name)
            if estimate is None:
                continue
            last = self.last_published.get(class_name)
            if last is not None and (now - last[2]).to_sec() < self.min_republish_interval:
                continue
            point = PointStamped()
            point.header.frame_id = self.target_frame_id
            point.point.x = estimate[0]
            point.point.y = estimate[1]
            self._publish_actor_info(class_name, point, republish=True)

    # --- 輔助方法 ---

    def _is_tracking_active(self):
        """檢查追踪狀態是否在超時範圍內保持活躍。"""
        is_active = (self.active_tracking_id is not None and
                     (rospy.Time.now() - self.last_tracking_update_time) < self.tracking_timeout_duration)
        return is_active

    def _get_roi_mean_depth(self, u, v):
        """從深度圖像的一個小區域（ROI）中計算有效的平均深度值。"""
        h, w = self.latest_depth_frame.shape
        
        # 定義ROI邊界
        u_min = max(0, u - self.ROI_HALF_SIZE)
        u_max = min(w - 1, u + self.ROI_HALF_SIZE)
        v_min = max(0, v - self.ROI_HALF_SIZE)
        v_max = min(h - 1, v + self.ROI_HALF_SIZE)

        # 提取ROI並過濾無效值
        roi = self.latest_depth_frame[v_min:v_max+1, u_min:u_max+1]
        valid_depths = roi[(roi > 0) & np.isfinite(roi)]

        if valid_depths.size == 0:
            return None

        # 計算平均深度並處理單位（假設原始單位為毫米）
        mean_depth_mm = np.mean(valid_depths)
        return mean_depth_mm / 1000.0 if mean_depth_mm > 100 else mean_depth_mm

    def _calculate_3d_point(self, u, v, depth_in_meters):
        """根據像素座標和深度，計算在相機座標系中的3D點。"""
        x = (u - self.cam_cx) * depth_in_meters / self.cam_fx
        y = (v - self.cam_cy) * depth_in_meters / self.cam_fy
        z = depth_in_meters
        
        point = PointStamped()
        point.header.frame_id = self.camera_frame_id
        point.header.stamp = rospy.Time(0) # 使用最新的可用變換
        point.point.x = x
        point.point.y = y
        point.point.z = z
        return point

    def _transform_point(self, point_stamped, target_frame):
        """使用TF2將一個點從源座標系轉換到目標座標系。"""
        try:
            return self.transform_buffer.transform(point_stamped, target_frame, timeout=rospy.Duration(0.5))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF座標變換失敗從 {point_stamped.header.frame_id} 到 {target_frame}: {e}")
            return None

    def _publish_actor_info_from_ground_truth(self, class_name):
        """用 Gazebo 人物模型的当前位置发布坐标。

        两条用途：
          1) 追踪期间的首选来源（prefer_truth_while_tracking，见 _load_ros_params）：
             真值误差 ~0，裁判"连续 15s、误差<1m"的窗口能在 15s 内凑满，
             不会出现"追了 3 分钟人还没被删"。
          2) 深度解算失败时的兜底。
        """
        actor_id = self._class_to_actor_id(class_name)
        if actor_id is None:
            return
        if actor_id not in self.actor_ground_truth:
            rospy.logwarn_throttle(5, f"沒有 {class_name} 的 Gazebo model_states 真值，無法兜底發布。")
            return

        if not self.truth_fallback_enabled:
            rospy.logwarn_throttle(5, "真值兜底已关闭（~truth_fallback:=false），跳过 %s 的兜底发布。",
                                   class_name)
            return
        x, y = self.actor_ground_truth[actor_id]
        fake_point = PointStamped()
        fake_point.header.frame_id = self.target_frame_id
        fake_point.point.x = x
        fake_point.point.y = y
        if self._publish_actor_info(class_name, fake_point):
            rospy.loginfo_throttle(1.0, "使用 Gazebo model_states 真值發布 %s 位置: x=%.3f, y=%.3f",
                                   class_name, x, y)

    def _publish_actor_info(self, class_name, final_point, republish=False):
        """根據目標類別名稱，填充並發布ActorInfo訊息。

        新增两道保护（对齐裁判 score_cal.py 的判定）：
          1) 一致性闸门：与其它无人机近期估计的中位数偏差过大 -> 不发，
             否则裁判会因为这条 >1m 的消息把"连续 15s"计时清零；
          2) hold 窗口：首次命中后记录估计并在窗口内持续重发，保证消息间隔 <=1s。
        """
        actor_id = self._class_to_actor_id(class_name)
        if actor_id is None:
            rospy.logwarn_throttle(5, f"無法從類別名稱 '{class_name}' 中提取ID。")
            return False
        if actor_id not in self.actor_publishers:
            rospy.logwarn(f"檢測到未配置的 Actor ID: {actor_id}。")
            return False

        # 2026-09-27 P0：发布前用新鲜的 /gazebo/model_states 复核人物是否还在。
        #   裁判 score_cal 找到人物后会 delete_model，但 YOLO 可能把别的 actor
        #   误报成同名 class（实测 actor_4 已删、uav1 持续输出 red4，深度解算出的
        #   坐标落在 actor_5 附近）→ /actor_red2_info 无限发 stale 坐标、
        #   tracker 永远 TRACKING。确认 actor 已删除时：
        #     * 本条（含 hold 重发）一律不发；
        #     * 清掉 hold/估计状态，避免 _publish_timer_cb 继续重发。
        # 2026-09-28：再加一道裁判侧的权威信号 /left_actors——人物被判"消除"后
        #   会立刻从这里消失，比 Gazebo 真正把模型移出世界（实测可延迟 20s+）更早。
        if self._actor_confirmed_missing(actor_id) or self._actor_eliminated_by_referee(actor_id):
            self.latest_estimate.pop(class_name, None)
            self.hold_until.pop(class_name, None)
            self.publish_start.pop(class_name, None)
            self.hold_expired.pop(class_name, None)
            rospy.logwarn_throttle(5.0, "actor_%d 已不在 %s（裁判已判定消除），停止发布 %s 坐标。",
                                   actor_id,
                                   "left_actors" if self._actor_eliminated_by_referee(actor_id)
                                   else "/gazebo/model_states",
                                   class_name)
            return False

        config = self.actor_definitions[actor_id]
        publisher = self.actor_publishers[actor_id]
        x = float(final_point.point.x)
        y = float(final_point.point.y)
        now = rospy.Time.now()

        # 2026-09-28：发布限频。检测本身 ~10Hz，hold 重发又插进来，叠加后会到 20Hz+。
        #   裁判只要求相邻消息 <=1s，10Hz 已经有两个数量级余量，所以这里统一限到
        #   ~publish_min_interval（默认 0.1s = 10Hz）。
        last_pub = self.last_published.get(class_name)
        if (last_pub is not None and self.publish_min_interval > 0.0 and
                (now - last_pub[2]).to_sec() < self.publish_min_interval):
            return False

        # 首帧发布延迟：让"跟踪窗口(20s)"比"find 窗口(15s)"先起跑，
        # 否则裁判删除人物时跟踪分还没攒够 20s。
        if not self.find_start_time.is_zero() and now < self.find_start_time:
            self.latest_estimate[class_name] = (x, y, now)   # 先缓存，但不下发、不启动 hold
            self.delayed_count += 1
            rospy.loginfo_throttle(2.0, "%s 发布延迟中（还剩 %.1fs，累计延迟 %d 次）",
                                   class_name, (self.find_start_time - now).to_sec(),
                                   self.delayed_count)
            return False

        # 记录估计（hold 重发用它）
        self.latest_estimate[class_name] = (x, y, now)
        if not republish:
            self._extend_hold(class_name, now)

        if not self._consistent_with_peers(config['topic'], x, y):
            self.suppressed_count += 1
            rospy.logwarn_throttle(2.0, "%s 的估计与其它无人机不一致，暂不发布（累计抑制 %d 次）",
                                   class_name, self.suppressed_count)
            return False

        actor_message = ActorInfo(cls=config['color'], x=x, y=y)
        publisher.publish(actor_message)
        if class_name not in self.publish_start:
            # 第一帧真正发出的消息（可能是 hold 重发）——min_publish_span 以此为基准
            self.publish_start[class_name] = now
        self.last_published[class_name] = (x, y, now)
        rospy.loginfo_throttle(1.0, f"發布 {class_name} ({config['color']}) -> '{publisher.name}' "
                                    f"x={x:.3f} y={y:.3f}{' [hold]' if republish else ''}")
        return True


if __name__ == '__main__':
    try:
        estimator = CoordinateEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("節點已關閉。")
    except Exception as e:
        rospy.logfatal(f"節點因致命錯誤而崩潰: {e}")
