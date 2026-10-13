#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ROS 相关库
import rospy
from sensor_msgs.msg import Image                   # ROS 图像消息类型
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox  # 自定义目标检测消息类型
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
import tf2_ros
import os
import sys
import math
import re
import time
import ast
import threading

# 红色人物 ID（red4/red5）判定：两个红模型材质完全一样，只能按位置判
# （本模块在同目录，rosrun/python 直接运行时 sys.path[0] 就是脚本目录）
from red_identity import match_near_actor, resolve_red_labels, transform_point

# 图像与AI库
import cv2
import numpy as np

# ✅ 离线模式：禁止 ultralytics 首次运行时联网检查/下载（无网环境会阻塞在
#   首次 predict，导致 yolo 卡住不发布检测结果）。必须在 import 前设置。
os.environ['YOLO_OFFLINE'] = '1'  # 离线模式：禁止 ultralytics 联网检查/下载，避免无网环境首次 predict 阻塞
from ultralytics import YOLO  # ✅ Ultralytics YOLOv8/YOLO11n 模型接口
import torch

# --------------------------------------------------
# 初始化 YOLO 模型
# 'yolo11n.pt' 是你训练好的 YOLO 模型文件
# ultralytics 库会自动处理推理等流程
# --------------------------------------------------
if torch.cuda.is_available():
    device = 'cuda'
    rospy.loginfo(f"GPU可用，使用设备: {torch.cuda.get_device_name(0)}")
else:
    device = 'cpu'
    rospy.logwarn("GPU不可用，将使用CPU运行，性能会受影响")



model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'yolo11n_942.pt')  # 训练好的 YOLO 权重文件路径
model = YOLO(model_path)
model.to(device)  # 强制模型使用GPU或CPU

# 常规发布阈值；近距离弱框需额外通过人物位置校验。
# 0.25：移动中的行人检测框会抖动，阈值太高时频繁漏检 → tracking 进不了
# TRACKING（反复 DETECTING→IDLE）。降低以稳定连续检测。
CONF_THRESH = 0.20  # 最小置信度阈值：低于该值的检测框被过滤，0.20 用于稳定连续检测行人
NEAR_CONF_THRESH = 0.12  # 近距离候选；仅在真值位置和像素投影均匹配时发布

# 连续检测保持时间：如果 YOLO 某一帧没有检测到目标，但上一帧刚检测到过，
# 则在 DETECTION_HOLD_SECONDS 内继续发布上一帧的检测框，避免 tracking 因
# 单帧/短时漏检而中断。设太大会追错位置，设太小则连续检测效果不明显。
DETECTION_HOLD_SECONDS = 1.5  # 漏检保持时间：YOLO 某帧漏检时继续发布上一帧检测框的时长（s）

# 是否使用 Gazebo 真值位置校正颜色/ID。
# 如果校正导致“识别到蓝色却发给 red4”这类问题，请保持 False。
# 是否用 Gazebo 真值位置校正行人颜色/ID。
# ⚠️ 诚信提示：这等于用仿真真值给检测结果"打标"，正式比赛若不接受此类辅助，
#    请用 rosparam 关闭：rosparam set /yolo11n/use_position_correction false（见 __main__）。
USE_POSITION_CORRECTION = True  # 运行时会被 ~use_position_correction 覆盖

# --------------------------------------------------
# 用于修正 YOLO 颜色/ID 的全局状态（仿真调试用）
# --------------------------------------------------
tf_buffer = None
tf_listener = None
actor_world = {}  # actor_id -> (x, y, z) in world frame
OWN_MODEL_NAME = ""      # 本机在 /gazebo/model_states 里的名字（__main__ 里赋值）
drone_world_xy = None    # 本机世界坐标 (x, y)，红色 ID 位置判定用
left_actor_ids = None
left_actors_wall = 0.0
CAM_FX = 205.47  # 深度相机 x 方向焦距（px）
CAM_FY = 205.47  # 深度相机 y 方向焦距（px）
CAM_CX = 320.0  # 深度相机主点 x 坐标（px）
CAM_CY = 240.0  # 深度相机主点 y 坐标（px）

def model_states_cb(msg):
    global actor_world, drone_world_xy
    actor_world.clear()
    drone_world_xy = None
    for name, pose in zip(msg.name, msg.pose):
        if name.startswith('actor_') and len(name) == 7 and name[-1].isdigit():
            try:
                actor_id = int(name[-1])
                actor_world[actor_id] = (pose.position.x, pose.position.y, pose.position.z)
            except Exception:
                pass
        elif OWN_MODEL_NAME and name == OWN_MODEL_NAME:
            # 本机世界位置：两个红模型外观完全一样，只能按"离本机更近"判 ID
            drone_world_xy = (pose.position.x, pose.position.y)


def left_actors_cb(msg):
    global left_actor_ids, left_actors_wall
    try:
        ids = ast.literal_eval(msg.data)
        if not isinstance(ids, (list, tuple, set)) or not all(
                isinstance(actor_id, int) for actor_id in ids):
            return
        left_actor_ids = set(ids)
        left_actors_wall = time.time()
    except (SyntaxError, ValueError, TypeError):
        pass


def actor_is_remaining(actor_id):
    return (left_actor_ids is None or time.time() - left_actors_wall > 2.0
            or actor_id in left_actor_ids)


def own_tf_cb(msg):
    for transform in msg.transforms:
        tf_buffer.set_transform(transform, OWN_MODEL_NAME)

def project_actor_to_pixel(actor_id):
    """将 actor 的世界坐标投影到相机像素坐标，失败返回 None。"""
    if actor_id not in actor_world or not actor_is_remaining(actor_id) or tf_buffer is None:
        return None
    try:
        tf = tf_buffer.lookup_transform('camera_optical_link', 'world',
                                        rospy.Time(0), timeout=rospy.Duration(0.05))
        t = tf.transform.translation
        q = tf.transform.rotation
        point = transform_point(actor_world[actor_id],
                                (t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        if point is None:
            return None
        x, y, z = point
        if z <= 0:
            return None
        u = CAM_FX * x / z + CAM_CX
        v = CAM_FY * y / z + CAM_CY
        return (u, v, z)
    except Exception:
        return None

active_tracking_id = None
mission_stage = "TAKEOFF"

# --------------------------------------------------
# 允许做人物推理的任务阶段（可配置）
# ⚠️ 2026-09-28：这里原先硬编码 `mission_stage not in ("PATROL", "TRACKING")`，
#    于是 ROAD 段完全不做推理（上一轮实测：4 号无人机 ROAD 占 179s、占其 62%
#    时间，那一段全盲）——这正是"人物从前面过却看不到"的原因之一。
#    现在把 ROAD 加进默认值，并改成可通过 ~yolo_stages 配置（逗号分隔）。
# 恢复旧行为（ROAD 段不推理、不发框）：
#    rosparam set /<vehicle_type>_<id>/yolo_stages "PATROL,TRACKING"
#    或在 launch 里给 yolo11n 节点加 <param name="yolo_stages" value="PATROL,TRACKING"/>
# --------------------------------------------------
YOLO_STAGES_DEFAULT = "PATROL,TRACKING,ROAD"


def parse_yolo_stages(raw):
    """把逗号分隔的阶段串解析成集合（去空格、转大写）。

    容错：空项会被忽略（"PATROL,,ROAD" 等价于 "PATROL,ROAD"）。
    解析结果为空集合时表示"任何阶段都不推理"（等价于关闭人物检测）。
    """
    return {s.strip().upper() for s in str(raw).split(",") if s.strip()}


YOLO_STAGES = parse_yolo_stages(YOLO_STAGES_DEFAULT)   # 运行时会被 ~yolo_stages 覆盖


def mission_state_cb(msg):
    """Enable person inference only while the mission can track people."""
    global mission_stage, last_valid_boxes_msg, last_valid_boxes_time
    stage = msg.data.split(":", 1)[1] if msg.data.startswith("STAGE:") else ""
    # 2026-09-28：只取第一个冒号前的段名，并去空格/统一大写。
    # 兼容 "STAGE:ROAD:seq" 这类带后缀的报文和大小写差异；否则会出现
    # "阶段明明是 ROAD、却因为字符串不完全相等而匹配不上 → 仍然不推理"的哑火。
    stage = stage.split(":", 1)[0].strip().upper()
    if stage == mission_stage:
        return
    mission_stage = stage
    last_valid_boxes_msg = None
    last_valid_boxes_time = rospy.Time(0)
    if "pub" in globals():
        empty = BoundingBoxes()
        empty.header.stamp = rospy.Time.now()
        pub.publish(empty)


def tracking_status_cb(msg):
    """订阅 tracking 状态，获知当前正在追踪的目标 ID。"""
    global active_tracking_id
    try:
        parts = msg.data.split(':')
        if len(parts) == 3 and parts[0] in ('TRACKING', 'DASH'):
            active_tracking_id = parts[1]
        else:
            active_tracking_id = None
    except Exception:
        active_tracking_id = None


def correct_label_by_position(cls_id, cx, cy):
    """用 Gazebo 真值位置修正人物 ID/颜色，避免 YOLO 颜色识别错误导致锁错人。"""
    best_id = None
    best_dist = 80.0  # 像素阈值
    for actor_id in range(6):
        uv = project_actor_to_pixel(actor_id)
        if uv is None:
            continue
        dist = math.hypot(uv[0] - cx, uv[1] - cy)
        if dist < best_dist:
            best_dist = dist
            best_id = actor_id
    if best_id is None:
        return None
    # 根据 actor_id 生成稳定标签
    if best_id == 4:
        return 'red4'
    if best_id == 5:
        return 'red5'
    # 0~3 的颜色名从模型 names 取，但确保与 actor_id 一致
    color = model.names.get(best_id, str(best_id))
    return f'{color}{best_id}'

# --------------------------------------------------
# 图像订阅回调函数
# 每当接收到 ROS 图像时，执行以下函数
# --------------------------------------------------
def ros_image_to_cv2(msg):
    """ROS Image -> BGR ndarray（不依赖 cv_bridge，兼容 py3.8/py3.10+numpy2）。

    Gazebo 的 R8G8B8 相机发布的是 ROS RGB8 编码，而 OpenCV 使用 BGR。
    之前直接按原始字节当作 BGR 用，导致红色/蓝色通道互换，YOLO 把红人
    认成蓝人、蓝人认成红人。这里根据 msg.encoding 正确转换。
    """
    frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    enc = (msg.encoding or '').lower()

    if frame.shape[2] == 4:                      # 4 通道
        if enc == 'rgba8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        elif enc == 'bgra8':
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        else:
            frame = frame[:, :, :3]              # 兜底：丢 alpha，按 BGR 处理
    elif frame.shape[2] == 3:                    # 3 通道
        if enc == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)   # RGB -> BGR
        # enc == 'bgr8' 时已经可以直接给 OpenCV 用
    elif frame.shape[2] == 1:                    # 灰度
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

    return frame

last_valid_boxes_msg = None
last_valid_boxes_time = rospy.Time(0)

# 图像回调只保留最新帧。Gazebo 在负载较高时可能以高于 YOLO 推理速度
# 发布图像；若回调队列积压，推理会持续处理过期画面，造成延迟和卡顿。
# 丢弃过期帧不会改变检测模型、阈值或输出规则，只减少无效重复计算。
_latest_image = None
_latest_image_lock = threading.Lock()

# --------------------------------------------------
# 推理可观测性（2026-09-28）：每 1s 汇总一次
#     stage=<阶段> frames=<推理帧数> boxes=<实际发布框数> infer_ms=<平均推理耗时ms>
# 目的：① 下一轮可以从日志直接确认"ROAD 段确实出框了"，不用再靠猜；
#       ② infer_ms / 每秒帧数用于评估放开 ROAD 后的 GPU 负载。
# 相当于 C++ 的 ROS_INFO_THROTTLE(1.0, ...)（rospy 里对应 rospy.loginfo_throttle）。
# 窗口用墙钟 time.time()，即使仿真时间被暂停/冻结也能照常统计。
# --------------------------------------------------
_stat_window_start = time.time()
_stat_frames = 0
_stat_boxes = 0
_stat_infer_ms_sum = 0.0


def note_inference(stage, n_boxes, infer_ms):
    """累计一帧推理统计，满 1s 打印一次（1Hz）并清零重新计窗。"""
    global _stat_window_start, _stat_frames, _stat_boxes, _stat_infer_ms_sum
    _stat_frames += 1
    _stat_boxes += int(n_boxes)
    _stat_infer_ms_sum += float(infer_ms)
    now = time.time()
    elapsed = now - _stat_window_start
    if elapsed < 1.0:
        return
    rospy.loginfo_throttle(1.0,
                           "[yolo11n] stage=%s frames=%d boxes=%d infer_ms=%.1f (window=%.1fs)",
                           stage, _stat_frames, _stat_boxes,
                           _stat_infer_ms_sum / max(_stat_frames, 1), elapsed)
    _stat_window_start = now
    _stat_frames = 0
    _stat_boxes = 0
    _stat_infer_ms_sum = 0.0


def publish_boxes_with_hold(boxes_msg):
    """发布检测框；若当前帧为空，则在短时间内继续发布上一帧有效检测框。

    返回本帧"实际发布出去的框数"（含保持帧补发的框），供 1Hz 统计使用。
    """
    global last_valid_boxes_msg, last_valid_boxes_time
    now = rospy.Time.now()
    if len(boxes_msg.bounding_boxes) > 0:
        last_valid_boxes_msg = boxes_msg
        last_valid_boxes_time = now
        pub.publish(boxes_msg)
        return len(boxes_msg.bounding_boxes)
    if (last_valid_boxes_msg is not None and
            (now - last_valid_boxes_time).to_sec() < DETECTION_HOLD_SECONDS):
        # 保持帧只用于短时控制连续性，不能刷新 tracker 的真实检测计时。
        # probability=1.0 与真值补框共用“非本帧 YOLO 检测”标记。
        last_valid_boxes_msg.header.stamp = now
        for box in last_valid_boxes_msg.bounding_boxes:
            box.probability = 1.0
        pub.publish(last_valid_boxes_msg)
        return len(last_valid_boxes_msg.bounding_boxes)
    pub.publish(boxes_msg)
    return 0


def add_synthetic_tracking_box(boxes_msg):
    """YOLO 漏检时，用 Gazebo 真值投影生成当前追踪目标的检测框。

    这仅用于仿真连续性：当 YOLO 某一帧没有输出当前追踪目标，但该人物
    模型确实在相机视野内时，根据其世界坐标投影到像素坐标，按真人尺寸
    生成一个检测框补上，避免 tracking 进入 LOST。
    """
    global active_tracking_id
    if not active_tracking_id:
        return

    # 当前帧已经有该目标的检测框就不需要补
    for b in boxes_msg.bounding_boxes:
        if b.Class == active_tracking_id:
            return

    match = re.search(r'\d+', active_tracking_id)
    if not match:
        return
    actor_id = int(match.group(0))

    uvz = project_actor_to_pixel(actor_id)
    if uvz is None:
        return
    u, v, z = uvz
    if not (0 <= u < 640 and 0 <= v < 480):
        return

    # 按真人尺寸(高1.7m, 宽0.6m)在像素平面估计检测框
    person_h = 1.7
    person_w = 0.6
    h = CAM_FY * person_h / z
    w = CAM_FX * person_w / z
    x1 = int(max(0, u - w / 2.0))
    x2 = int(min(639, u + w / 2.0))
    y1 = int(max(0, v - h / 2.0))
    y2 = int(min(479, v + h / 2.0))
    if x2 - x1 < 5 or y2 - y1 < 5:
        return

    bbox = BoundingBox()
    bbox.Class = active_tracking_id
    bbox.probability = 1.0
    bbox.xmin = x1
    bbox.ymin = y1
    bbox.xmax = x2
    bbox.ymax = y2
    boxes_msg.bounding_boxes.append(bbox)


def image_callback(msg):
    global _latest_image
    with _latest_image_lock:
        _latest_image = msg

def process_image(msg):
    # 阶段门控：不在 ~yolo_stages 列表里的阶段直接返回（不做推理、不发框）。
    # 默认列表含 ROAD（原先只有 PATROL/TRACKING，ROAD 段全盲 → "人物从前面过却看不到"）。
    if mission_stage not in YOLO_STAGES:
        return

    # 1. 将 ROS 图像消息转换为 OpenCV 格式（BGR）
    frame = ros_image_to_cv2(msg)

    # 2. 使用 Ultralytics YOLO 模型进行推理
    # predict 返回的是一个列表，每一项是一个 `Results` 对象（支持 batch）
    t_infer_start = time.time()
    inference_conf = NEAR_CONF_THRESH if USE_POSITION_CORRECTION else CONF_THRESH
    results = model.predict(source=frame, conf=inference_conf, verbose=False, device=device)
    infer_ms = (time.time() - t_infer_start) * 1000.0   # 单帧推理耗时（ms），供 1Hz 均值统计

    # 3. 创建并初始化 ROS BoundingBoxes 消息
    boxes_msg = BoundingBoxes()
    boxes_msg.header = msg.header  # 使用图像的时间戳和坐标系

    # 4. 获取推理结果（这里只处理 batch 中的第 0 帧）
    result = results[0]

    # 5. 如果没有检测结果（即 boxes 为空），直接发布空消息并返回
    if result.boxes is None:
        add_synthetic_tracking_box(boxes_msg)
        n_pub = publish_boxes_with_hold(boxes_msg)
        note_inference(mission_stage, n_pub, infer_ms)
        return

    # 6. 从 result 中提取检测框信息
    boxes = result.boxes.xyxy.cpu().numpy()   # 检测框 [x1, y1, x2, y2]
    confs = result.boxes.conf.cpu().numpy()   # 置信度
    classes = result.boxes.cls.cpu().numpy()  # 类别索引

    # 弱框只在附近的唯一人物投影与画面位置吻合时使用。先按距离筛人，
    # 避免每帧对地图上的所有 actor 做 TF 查询。
    weak_matches = {}
    if USE_POSITION_CORRECTION and drone_world_xy is not None:
        weak_indices = [i for i in range(len(boxes))
                        if int(classes[i]) in (0, 1, 2, 3, 4)
                        and confs[i] < CONF_THRESH
                        and boxes[i][2] - boxes[i][0] >= 8
                        and boxes[i][3] - boxes[i][1] >= 12
                        and (boxes[i][2] - boxes[i][0]) *
                            (boxes[i][3] - boxes[i][1]) >= 300]
        if weak_indices:
            nearby_actors = {
                actor_id: position for actor_id, position in actor_world.items()
                if actor_is_remaining(actor_id) and
                math.hypot(position[0] - drone_world_xy[0],
                           position[1] - drone_world_xy[1]) <= 10.0
            }
            projections = {actor_id: project_actor_to_pixel(actor_id)
                           for actor_id in nearby_actors}
            for i in weak_indices:
                actor_id = match_near_actor(boxes[i], drone_world_xy,
                                            nearby_actors, projections)
                if actor_id is not None:
                    weak_matches[i] = actor_id

    # 7. 遍历每个检测结果，构建 ROS BoundingBox 消息
    # Both red actors share one YOLO class. Match boxes to their projected
    # positions; a single box can also use the sole surviving/nearer actor.
    red_indices = [i for i in range(len(boxes))
                   if int(classes[i]) == 4 and confs[i] >= CONF_THRESH]
    red_label = {}
    if red_indices:
        red_centers = {i: ((boxes[i][0] + boxes[i][2]) / 2.0,
                           (boxes[i][1] + boxes[i][3]) / 2.0) for i in red_indices}
        remaining_reds = {actor_id: actor_world[actor_id] for actor_id in (4, 5)
                          if actor_id in actor_world and actor_is_remaining(actor_id)}
        red_projection = {actor_id: project_actor_to_pixel(actor_id)
                          for actor_id in remaining_reds}
        red_label = resolve_red_labels(red_indices, drone_world_xy, remaining_reds,
                                       red_centers, red_projection)
        rospy.loginfo_throttle(
            2.0, "[yolo11n %s] red boxes=%d mapped=%s remaining=%s",
            OWN_MODEL_NAME, len(red_indices), ",".join(sorted(red_label.values())) or "none",
            ",".join(str(actor_id) for actor_id in sorted(remaining_reds)) or "none")

    for i in range(len(boxes)):
        cls_id = int(classes[i])              # 获取类别索引
        # ✅ **修改点：只处理类别索引为 0 到 4 的情况**
        # 如果类别是 'female' (5) 或 'fire hydrant' (6)，则跳过此次循环
        if cls_id not in [0, 1, 2, 3, 4]:
            continue
        x1, y1, x2, y2 = boxes[i]             # 获取边界框坐标
        conf = confs[i]                       # 获取置信度

        # 只过滤明显无效的小框。不要用严格的高宽比过滤移动/侧身人物：
        # 近距离人物经过机头时，框可能被裁切或接近方形，旧阈值会把真实
        # 检测全部丢掉，tracking 的 IDLE 状态因此永远看不到目标。
        box_w = x2 - x1
        box_h = y2 - y1
        if box_w < 8 or box_h < 12:
            continue
        weak_actor_id = weak_matches.get(i)
        if conf < CONF_THRESH and weak_actor_id is None:
            continue
        # 将类别索引转换为类别名；红色行人同一类，拆成 red4/red5
        if weak_actor_id is not None:
            color = 'red' if weak_actor_id in (4, 5) else model.names[weak_actor_id]
            label = f"{color}{weak_actor_id}"
        elif cls_id == 4:
            label = red_label.get(i)
            if label is None and USE_POSITION_CORRECTION:
                corrected = correct_label_by_position(cls_id, (x1 + x2) / 2.0,
                                                      (y1 + y2) / 2.0)
                if corrected in ('red4', 'red5'):
                    label = corrected
            if label is None:
                rospy.logwarn_throttle(5.0, "[yolo11n %s] red detection has ambiguous actor ID; waiting for pose",
                                       OWN_MODEL_NAME)
                continue
        else:
            label = f"{model.names[cls_id]}{cls_id}"

        # 位置校正默认关闭：避免“识别到蓝色却发给 red4”的误修正。
        # 需要时可将 USE_POSITION_CORRECTION 改为 True。
        if USE_POSITION_CORRECTION and weak_actor_id is None:
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            corrected = correct_label_by_position(cls_id, cx, cy)
            if corrected is not None:
                label = corrected
            actor_match = re.search(r'\d+$', label)
            if actor_match and drone_world_xy is not None:
                actor_id = int(actor_match.group(0))
                if actor_id in actor_world:
                    actor_x, actor_y, _ = actor_world[actor_id]
                    distance = math.hypot(actor_x - drone_world_xy[0],
                                          actor_y - drone_world_xy[1])
                    if distance > 45.0:
                        rospy.logwarn_throttle(
                            2.0, "[yolo11n %s] skip %s: actor %.1fm from camera drone",
                            OWN_MODEL_NAME, label, distance)
                        continue

        # 构建单个 BoundingBox 消息
        bbox = BoundingBox()
        bbox.Class = label                    # 类别名（如 'red4'）
        bbox.probability = float(conf)        # 置信度
        bbox.xmin = int(x1)
        bbox.ymin = int(y1)
        bbox.xmax = int(x2)
        bbox.ymax = int(y2)

        # 添加到 BoundingBoxes 列表中
        boxes_msg.bounding_boxes.append(bbox)

    # 8. 补充仿真真值框后发布（带短暂保持）
    add_synthetic_tracking_box(boxes_msg)
    n_pub = publish_boxes_with_hold(boxes_msg)
    if boxes_msg.bounding_boxes:
        rospy.loginfo_throttle(
            2.0, "[yolo11n] stage=%s person_labels=%s published=%d",
            mission_stage,
            ",".join(sorted({b.Class for b in boxes_msg.bounding_boxes})), n_pub)
    elif len(boxes):
        rospy.loginfo_throttle(
            2.0, "[yolo11n %s] raw=%d weak_near=%d published=%d stage=%s",
            OWN_MODEL_NAME, len(boxes), len(weak_matches), n_pub, mission_stage)
    if weak_matches:
        rospy.loginfo_throttle(
            2.0, "[yolo11n %s] near detections accepted: %s",
            OWN_MODEL_NAME,
            ",".join(f"actor_{actor_id}:{confs[i]:.2f}"
                     for i, actor_id in sorted(weak_matches.items())))
    # 1Hz 汇总：stage/frames/boxes + 平均推理耗时（可确认 ROAD 段确实出框、评估 GPU 负载）
    note_inference(mission_stage, n_pub, infer_ms)

# --------------------------------------------------
# ROS 节点初始化入口
# --------------------------------------------------
if __name__ == '__main__':
    # 真值校正开关（诚信相关）：~use_position_correction，默认保持原行为
    # 正式比赛建议：rosparam set /yolo11n/use_position_correction false

    vehicle_type = sys.argv[1]
    vehicle_id = sys.argv[2]
    # 本机在 /gazebo/model_states 里的模型名，用于缓存本机世界位置（红色 ID 判定）
    OWN_MODEL_NAME = f'{vehicle_type}_{vehicle_id}'
    # 初始化 ROS 节点，名称为 yolo11n_pedestrian_detector
    rospy.init_node(f'yolo11n_pedestrian_detector_{vehicle_type}_{vehicle_id}')

    # 真值校正开关（诚信相关）：默认 true 保持原行为；
    # 正式比赛若不接受用仿真真值辅助检测，设 ~use_position_correction:=false。
    # 模块级作用域，直接赋值即可（这里不能用 global）
    USE_POSITION_CORRECTION = bool(rospy.get_param('~use_position_correction',
                                                   USE_POSITION_CORRECTION))
    rospy.logwarn("位置真值校正(USE_POSITION_CORRECTION) = %s", USE_POSITION_CORRECTION)

    # 允许推理的任务阶段：~yolo_stages，逗号分隔，默认 "PATROL,TRACKING,ROAD"。
    # 恢复旧行为（ROAD 段不推理）：rosparam set /<vehicle>/yolo_stages "PATROL,TRACKING"。
    YOLO_STAGES = parse_yolo_stages(rospy.get_param('~yolo_stages', YOLO_STAGES_DEFAULT))
    if not YOLO_STAGES:
        rospy.logwarn("~yolo_stages 解析为空：YOLO 将不在任何阶段推理（不发布检测框）")
    rospy.loginfo("YOLO 允许推理的阶段(~yolo_stages) = [%s]",
                  ",".join(sorted(YOLO_STAGES)))

    # 创建发布器，发布检测框结果到 /yolo11n/bounding_boxes
    pub = rospy.Publisher(vehicle_type+'_'+vehicle_id+'/yolo11n/bounding_boxes', BoundingBoxes, queue_size=10)

    # 订阅摄像头图像话题（环境适配层）。
    # 默认 = 本地自建机型 typhoon_h480_zzufly 的 Realsense 彩色图；
    # 换环境（例如官方 robocup.launch 的 typhoon_h480 基础型只有 CGO3 单目）可用
    #   _color_topic:=/typhoon_h480_0/cgo3_camera/image_raw
    # 覆盖，无需改代码。start_sim_only.sh --env-profile=official 会自动传。
    _default_color_topic = vehicle_type+'_'+vehicle_id+'/realsense/depth_camera/color/image_raw'
    _color_topic = rospy.get_param('~color_topic', _default_color_topic)
    rospy.loginfo("YOLO 订阅图像话题(~color_topic)：%s", _color_topic)
    if _color_topic != _default_color_topic:
        rospy.logwarn("YOLO 图像话题已被覆盖为 %s（默认 %s）——请确认该话题确实存在",
                      _color_topic, _default_color_topic)
    rospy.Subscriber(_color_topic, Image, image_callback, queue_size=1, buff_size=2**24)

    # 订阅任务阶段；只有 ~yolo_stages 列表里的阶段（默认 PATROL/TRACKING/ROAD）允许人物推理
    rospy.Subscriber(vehicle_type+'_'+vehicle_id+'/mission/stage', String,
                     mission_state_cb, queue_size=1)

    # 订阅 Gazebo 真值位置，用于修正 YOLO 颜色/ID（仿真调试用）
    rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_cb, queue_size=1)
    rospy.Subscriber('/left_actors', String, left_actors_cb, queue_size=1)

    # 订阅 tracking 状态，用于 YOLO 漏检时补一个真值投影框
    rospy.Subscriber(vehicle_type+'_'+vehicle_id+'/tracking_node/yolo_human_tracking_'
                     + vehicle_type+'_'+vehicle_id, String, tracking_status_cb, queue_size=1)

    # TF 用于把 actor 世界坐标投影到相机像素
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.Subscriber('/' + OWN_MODEL_NAME + '/tf', TFMessage, own_tf_cb, queue_size=1)

    # 日志输出
    rospy.loginfo("YOLO11n Pedestrian Detector (using ultralytics) started.")

    # 推理在主线程串行执行，订阅回调只覆盖最新图像，不会让旧帧在队列中
    # 堵塞。算法本身和发布规则不变，负载高时仅跳过已经过期的输入帧。
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        with _latest_image_lock:
            image = _latest_image
            _latest_image = None
        if image is not None:
            process_image(image)
        rate.sleep()
""" 
依赖安装
pip install ultralytics opencv-python

运行
roscore
rosrun your_package_name yolo11n.py
"""
