#!/usr/bin/env python3
"""YOLO segmentation road detector for the simulated downward-looking camera.

The node publishes geometry only: Vector3.x is normalized road-center error,
Vector3.y is normalized road direction error, and Vector3.z is confidence.
"""
import cv2
import numpy as np
import os
import rospy

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PoseStamped, Vector3
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion


def mission_stage(value):
    """Return the stage token from a mission state message."""
    text = getattr(value, "data", value) or ""
    return text.split(":", 1)[1] if text.startswith("STAGE:") else ""


def image_to_bgr(msg):
    channels = 4 if msg.encoding.lower() in ("rgba8", "bgra8") else 3
    if msg.encoding.lower() in ("mono8", "8uc1"):
        channels = 1
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    expected = msg.height * msg.width * channels
    if arr.size < expected:
        raise ValueError("image buffer is shorter than its dimensions")
    arr = arr[:expected].reshape((msg.height, msg.width, channels))
    enc = msg.encoding.lower()
    if channels == 1:
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    if enc == "rgb8":
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    if enc == "rgba8":
        return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
    if enc == "bgra8":
        return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
    return arr[:, :, :3]


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class RoadDetector:
    def __init__(self):
        self.vehicle = rospy.get_param("~vehicle", "typhoon_h480_0")
        self.disable_for_vehicle = bool(rospy.get_param("~disable_for_vehicle", False))
        self.image_topic = rospy.get_param(
            "~image_topic",
            "/%s/realsense/depth_camera/color/image_raw" % self.vehicle,
        )
        self.output_topic = rospy.get_param(
            "~output_topic", "/%s/road_follow/steering" % self.vehicle
        )
        self.candidates_topic = rospy.get_param(
            "~candidates_topic", "/%s/road_follow/candidates" % self.vehicle
        )
        self.debug_topic = rospy.get_param(
            "~debug_topic", "/%s/road_follow/debug_image" % self.vehicle
        )
        self.publish_debug = rospy.get_param("~publish_debug", True)
        self.roi_top = float(rospy.get_param("~roi_top", 0.05))
        self.roi_bottom = float(rospy.get_param("~roi_bottom", 0.92))
        self.min_area_ratio = float(rospy.get_param("~min_area_ratio", 0.005))
        self.min_usable_pixels = int(rospy.get_param("~min_usable_pixels", 20))
        self.bottom_width_ratio = float(rospy.get_param("~bottom_width_ratio", 0.05))
        self.require_far = bool(rospy.get_param("~require_far", False))
        self.road_model = rospy.get_param("~road_model", "/home/lishu2/2025_ZZU_FLY/best.pt")
        self.road_class = int(rospy.get_param("~road_class", 0))
        self.infer_imgsz = int(rospy.get_param("~infer_imgsz", 640))
        self.infer_conf = float(rospy.get_param("~infer_conf", 0.25))
        self.infer_iou = float(rospy.get_param("~infer_iou", 0.7))
        self.infer_device = rospy.get_param("~infer_device", "cpu")
        self.infer_max_det = int(rospy.get_param("~infer_max_det", 10))
        if YOLO is None:
            raise ImportError("ultralytics is required for YOLO road segmentation")
        if not os.path.isfile(self.road_model):
            raise IOError("road model does not exist: %s" % self.road_model)
        self.model = YOLO(self.road_model)
        self.debug_rate = float(rospy.get_param("~debug_rate", 5.0))
        self.process_every = max(1, int(rospy.get_param("~process_every", 2)))
        self.frame_count = 0
        self.pose_topic = rospy.get_param(
            "~pose_topic", "/%s/mavros/local_position/pose" % self.vehicle
        )
        self.pose_timeout = float(rospy.get_param("~pose_timeout", 0.5))
        self.hold_seconds = float(rospy.get_param("~hold_seconds", 1.2))
        self.pitch_roi_gain = float(rospy.get_param("~pitch_roi_gain", 0.45))
        self.roll_roi_gain = float(rospy.get_param("~roll_roi_gain", 0.25))
        self.pitch = 0.0
        self.roll = 0.0
        self.pose_stamp = rospy.Time(0)
        self.stage = "TAKEOFF"
        self.last_result = (0.0, 0.0, 0.0, None)
        self.last_valid = rospy.Time(0)
        self.last_heading = 0.0
        self.has_heading = False
        self.last_debug = rospy.Time(0)

        self.candidates_pub = rospy.Publisher(self.candidates_topic, Float32MultiArray, queue_size=1)
        self.mission_sub = rospy.Subscriber(
            rospy.get_param("~mission_topic", "/%s/mission/stage" % self.vehicle),
            String, self.mission_callback, queue_size=1)

        self.pub = rospy.Publisher(self.output_topic, Vector3, queue_size=1)
        self.debug_pub = rospy.Publisher(self.debug_topic, Image, queue_size=1)
        self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback, queue_size=1)
        self.sub = rospy.Subscriber(self.image_topic, Image, self.callback, queue_size=1,
                                    buff_size=2 ** 24)
        self.publish_stats = bool(rospy.get_param("~publish_stats", True))
        self.stats_topic = "/%s/road_follow/stats" % self.vehicle
        self.stats_pub = None
        self.frame_index = 0
        if self.publish_stats:
            self.stats_pub = rospy.Publisher(self.stats_topic, String, queue_size=1)
        rospy.loginfo("[road_follow] %s listening to %s", self.vehicle, self.image_topic)

    def mission_callback(self, msg):
        stage = mission_stage(msg)
        if stage == self.stage:
            return
        self.stage = stage
        self.last_result = (0.0, 0.0, 0.0, None)
        self.last_valid = rospy.Time(0)
        self.has_heading = False
        self.last_heading = 0.0
        if not self.disable_for_vehicle:
            self.publish_invalid()
            self.candidates_pub.publish(Float32MultiArray())

    def pose_callback(self, msg):
        q = msg.pose.orientation
        self.roll, self.pitch, _ = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose_stamp = rospy.Time.now()

    def build_candidates(self, mask, lateral, heading, confidence):
        """Return branch candidates as [heading, lateral, confidence, width]."""
        if mask is None or confidence <= 0.0:
            return []
        h, w = mask.shape[:2]
        band = mask[int(0.30 * h):int(0.55 * h), :]
        counts = np.count_nonzero(band, axis=0)
        active = counts >= max(2, int(0.15 * band.shape[0]))
        runs, start = [], None
        for i, on in enumerate(active.tolist() + [False]):
            if on and start is None:
                start = i
            elif not on and start is not None:
                if i - start >= max(6, int(0.02 * w)):
                    runs.append((start, i - 1))
                start = None
        if not runs:
            return [[float(heading), float(lateral), float(confidence), 1.0]]
        near_y0, near_y1 = int(0.68 * h), int(0.90 * h)
        near_y, near_x = np.nonzero(mask[near_y0:near_y1, :])
        global_near_x = float(np.median(near_x)) if near_x.size else 0.5 * w
        result = []
        for x0, x1 in runs[:3]:
            center = 0.5 * (x0 + x1)
            margin = max(12.0, 1.5 * (x1 - x0 + 1))
            local = near_x[(near_x >= center - margin) & (near_x <= center + margin)]
            near_center = float(np.median(local)) if local.size else global_near_x
            heading_norm = clamp((near_center - center) / max(0.5 * w, 1.0) * 2.0, -1.0, 1.0)
            lateral_norm = clamp((center - 0.5 * w) / max(0.5 * w, 1.0), -1.0, 1.0)
            width_norm = clamp((x1 - x0 + 1) / float(w), 0.0, 1.0)
            result.append([float(heading_norm), float(lateral_norm),
                           float(confidence * min(1.0, width_norm * 5.0)),
                           float(width_norm)])
        return result

    def callback(self, msg):
        if self.disable_for_vehicle or self.stage not in ("TRANSIT_TO_REGION", "ROAD"):
            return
        self.frame_count += 1
        if self.frame_count % self.process_every != 0:
            return
        try:
            image = image_to_bgr(msg)
            lateral, heading, confidence, mask, stats = self.detect(image)
            now = rospy.Time.now()
            if stats.get('accepted', False):
                self.last_result = (lateral, heading, confidence, mask)
                self.last_valid = now
                if stats.get('heading_available', False):
                    self.last_heading = heading
                    self.has_heading = True
            elif self.last_valid != rospy.Time(0) and (now - self.last_valid).to_sec() <= self.hold_seconds:
                lateral, heading, old_conf, mask = self.last_result
                confidence = old_conf * max(0.05, 1.0 - (now - self.last_valid).to_sec() / max(self.hold_seconds, 1e-3))
            else:
                lateral, heading, confidence, mask = 0.0, 0.0, 0.0, mask
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "[road_follow] image processing failed: %s", exc)
            self.publish_invalid()
            return

        out = Vector3()
        out.x = float(lateral)
        out.y = float(heading)
        out.z = float(confidence)
        self.pub.publish(out)
        candidates = Float32MultiArray()
        if confidence > 0.0:
            candidates.data = [v for c in self.build_candidates(mask, lateral, heading, confidence) for v in c]
        self.candidates_pub.publish(candidates)
        if self.stats_pub is not None:
            self.frame_index += 1
            payload = "frame=%d yolo_conf=%.3f area=%.3f usable=%d pass_min=%d pass_usable=%d pass_bottom=%d pass_near_far=%d bottom=%d/%d near=%d far=%d accepted=%d reason=%s heading_available=%d final=%.3f steer=%.3f" % (
                self.frame_index,
                stats['yolo_conf'], stats['area_ratio'], stats['usable_pixels'],
                int(stats['pass_min_area']), int(stats['pass_usable']), int(stats['pass_bottom']),
                int(stats['pass_near_far']), stats['bottom_count'], stats['bottom_required'],
                stats['near_count'], stats['far_count'], int(stats['accepted']), stats['reason'],
                int(stats['heading_available']), stats['final_conf'], confidence,
            )
            self.stats_pub.publish(String(data=payload))
        if self.publish_debug and (rospy.Time.now() - self.last_debug).to_sec() >= 1.0 / max(self.debug_rate, 1.0):
            self.last_debug = rospy.Time.now()
            self.publish_debug_image(image, mask, lateral, heading, confidence)

    def publish_invalid(self):
        out = Vector3()
        out.z = 0.0
        self.pub.publish(out)

    def detect(self, image):
        h, w = image.shape[:2]
        pose_age = (rospy.Time.now() - self.pose_stamp).to_sec()
        pitch = self.pitch if pose_age <= self.pose_timeout else 0.0
        roi_top = clamp(self.roi_top + self.pitch_roi_gain * pitch / np.pi, 0.12, 0.55)
        roi_bottom = clamp(self.roi_bottom + self.pitch_roi_gain * pitch / np.pi, roi_top + 0.2, 1.0)
        y0, y1 = int(roi_top * h), int(roi_bottom * h)
        roi = image[y0:y1, :]
        kwargs = {"imgsz": self.infer_imgsz, "conf": self.infer_conf,
                  "iou": self.infer_iou, "max_det": self.infer_max_det, "verbose": False}
        if self.infer_device:
            kwargs["device"] = self.infer_device
        results = self.model.predict(roi, **kwargs)
        mask = np.zeros(roi.shape[:2], dtype=np.uint8)
        best_conf = 0.0
        if results and results[0].masks is not None:
            result = results[0]
            classes = result.boxes.cls.cpu().numpy() if result.boxes is not None else []
            confs = result.boxes.conf.cpu().numpy() if result.boxes is not None else []
            for index, (cls, conf) in enumerate(zip(classes, confs)):
                if int(cls) != self.road_class:
                    continue
                candidate = result.masks.data[index].cpu().numpy()
                candidate = cv2.resize(candidate, (roi.shape[1], roi.shape[0]), interpolation=cv2.INTER_NEAREST)
                if conf > best_conf:
                    mask = (candidate > 0.5).astype(np.uint8) * 255
                    best_conf = float(conf)
        mask[: int(0.05 * mask.shape[0]), :] = 0
        area_ratio = float(np.count_nonzero(mask)) / float(max(1, mask.size))
        ys, xs = np.nonzero(mask)
        usable_pixels = int(xs.size)
        bottom = (ys >= int(0.20 * mask.shape[0])) & (ys <= int(0.80 * mask.shape[0]))
        bottom_count = int(np.count_nonzero(bottom))
        bottom_required = max(8, int(self.bottom_width_ratio * mask.shape[1]))
        stats = {
            'yolo_conf': float(best_conf), 'area_ratio': float(area_ratio),
            'usable_pixels': usable_pixels, 'pass_min_area': area_ratio >= self.min_area_ratio,
            'pass_usable': usable_pixels >= self.min_usable_pixels,
            'pass_bottom': bottom_count >= bottom_required, 'pass_near_far': False,
            'bottom_required': bottom_required, 'bottom_count': bottom_count,
            'near_count': 0, 'far_count': 0, 'final_conf': 0.0,
            'accepted': False, 'reason': 'no_mask', 'heading_available': False,
        }
        if not stats['pass_min_area']:
            stats['reason'] = 'area'
            return 0.0, self.last_heading if self.has_heading else 0.0, 0.0, mask, stats
        if not stats['pass_usable']:
            stats['reason'] = 'usable_pixels'
            return 0.0, self.last_heading if self.has_heading else 0.0, 0.0, mask, stats
        stats['pass_near_far'] = bool(bottom_count >= bottom_required)
        if self.require_far:
            far = ys <= int(0.42 * mask.shape[0])
            stats['far_count'] = int(np.count_nonzero(far))
        middle_bottom = ((ys >= int(0.20 * mask.shape[0])) & (ys <= int(0.80 * mask.shape[0])))
        geometry_x = xs[middle_bottom] if np.any(middle_bottom) else xs
        center_x = float(np.median(geometry_x))
        lateral = clamp((center_x - 0.5 * mask.shape[1]) / max(0.5 * mask.shape[1], 1.0), -1.0, 1.0)
        lateral = clamp(lateral + self.roll_roi_gain * self.roll / np.pi, -1.0, 1.0)
        heading = self.last_heading if self.has_heading else 0.0
        heading_available = False
        if np.count_nonzero(middle_bottom) >= 12 and np.ptp(ys[middle_bottom]) >= max(4, int(0.08 * mask.shape[0])):
            slope = np.polyfit(ys[middle_bottom].astype(float), xs[middle_bottom].astype(float), 1)[0]
            heading = clamp(slope * mask.shape[0] / max(0.5 * mask.shape[1], 1.0), -1.0, 1.0)
            heading_available = True
        confidence = clamp(best_conf * min(1.0, area_ratio / 0.02) * min(1.0, usable_pixels / 100.0), 0.0, 1.0)
        stats.update({'accepted': confidence > 0.20, 'reason': 'accepted' if confidence > 0.20 else 'confidence',
                      'heading_available': heading_available, 'final_conf': float(confidence),
                      'near_count': int(np.count_nonzero(ys >= int(0.68 * mask.shape[0])))})
        return lateral, heading, confidence if stats['accepted'] else 0.0, mask, stats

    def publish_debug_image(self, image, mask, lateral, heading, confidence):
        overlay = image.copy()
        h, w = image.shape[:2]
        y0 = int(self.roi_top * h)
        colored = np.zeros_like(overlay[y0:y0 + mask.shape[0]])
        colored[:, :, 1] = mask
        overlay[y0:y0 + mask.shape[0]] = cv2.addWeighted(
            overlay[y0:y0 + mask.shape[0]], 0.65, colored, 0.35, 0.0
        )
        cx = int(w * 0.5 + lateral * w * 0.5)
        cv2.line(overlay, (w // 2, h - 1), (cx, int(0.55 * h)), (0, 255, 255), 3)
        text = "road conf=%.2f lat=%.2f head=%.2f" % (confidence, lateral, heading)
        cv2.putText(overlay, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                    (0, 255, 0) if confidence > 0.5 else (0, 0, 255), 2)
        out = Image()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = "camera_optical_link"
        out.height, out.width = overlay.shape[:2]
        out.encoding = "bgr8"
        out.step = overlay.shape[1] * 3
        out.data = overlay.tobytes()
        self.debug_pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("road_follow")
    RoadDetector()
    rospy.spin()
