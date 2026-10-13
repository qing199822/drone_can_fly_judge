#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
裁判端只读核对脚本：从任一提交 SDF 打印
  (1) 传感器安装位姿表 (pose / link / 所属插件 / frameName / topic)
  (2) 关键参数对照表 (与 XTDrone 内置基准逐项比对)
用法:
  python3 sdf_sensor_audit.py <提交.sdf> [--ref <基准.sdf>]
只读，不写任何文件。
"""
import sys, os, math, argparse
import xml.etree.ElementTree as ET

# ---------------- XTDrone 平台内置基准（文件实际内容） ----------------
# 来源: XTDrone/sitl_config/models/realsense_camera/realsense_camera.sdf  (depth)
#       XTDrone/sitl_config/models/typhoon_h480_2d_lidar/typhoon_h480_2d_lidar.sdf (laser)
#       XTDrone/sitl_config/models/hokuyo_lidar/model.sdf (laser, 备选)
#       XTDrone/sitl_config/models/typhoon_h480/typhoon_h480.sdf (camera / cgo3_camera_joint)
REF_DEPTH = {
    "pose": "0 0 0 0 0 0", "update_rate": "20", "horizontal_fov": "1.047198",
    "width": "640", "height": "480", "near": "0.05", "far": "100",
    "frameName": "depth_camera_base", "cameraName": "realsense/depth_camera",
    "pointCloudCutoff": "0.05", "pointCloudCutoffMax": "10",
}
REF_LASER_TYPHOON = {
    "pose": "0 0 0.2 0 -0 0", "update_rate": "500", "samples": "640",
    "resolution": "1", "min_angle": "-3.14", "max_angle": "3.14",
    "range_min": "0.08", "range_max": "10", "range_resolution": "0.01",
    "frameName": "laser_2d", "topicName": "scan",
    "plugin_name": "gazebo_ros_head_hokuyo_controller",
}
REF_LASER_HOKUYO = {
    "pose": "0 0 0.0175 0 -0 0", "update_rate": "500", "samples": "512",
    "resolution": "1", "min_angle": "-3.14", "max_angle": "3.14",
    "range_min": "0.5", "range_max": "20", "range_resolution": "0.01",
    "frameName": "laser_2d", "topicName": "scan",
    "plugin_name": "gazebo_ros_head_hokuyo_controller",
}
REF_CAM_RGB_TYPHOON = {
    "pose": "0.0 0 -0.162 0 0 0", "update_rate": "10", "horizontal_fov": "2.0",
    "width": "640", "height": "360", "near": "0.05", "far": "15000",
    "frameName": "cgo3_camera_optical_frame",
}
REF_GIMBAL_LIMITS = {
    "cgo3_mount_joint": ("0", "0"),
    "cgo3_vertical_arm_joint": ("-1e+16", "1e+16"),
    "cgo3_horizontal_arm_joint": ("-0.785398", "0.785398"),
    "cgo3_camera_joint": ("-1.05", "2.09"),   # 官方 typhoon_h480
}
GIMBAL_JOINTS = ["cgo3_mount_joint", "cgo3_vertical_arm_joint",
                 "cgo3_horizontal_arm_joint", "cgo3_camera_joint"]


def s(t):
    return (t or "").strip()


def txt(node, path, default=""):
    if node is None:
        return default
    e = node.find(path)
    return s(e.text) if e is not None and e.text is not None else default


def rad2deg(v):
    try:
        return "%.2f" % (float(v) * 180.0 / math.pi)
    except Exception:
        return "n/a"


def pose6(p):
    """返回 (xyz, rpy, pitch_deg, 说明)"""
    parts = p.split()
    if len(parts) != 6:
        return p, "-", "n/a", "文件中未找到/格式异常"
    xyz = " ".join(parts[0:3]); rpy = " ".join(parts[3:6])
    return xyz, rpy, rad2deg(parts[4]), ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sdf")
    ap.add_argument("--ref", default=None, help="可选：同时比对的基准 SDF")
    a = ap.parse_args()

    tree = ET.parse(a.sdf); root = tree.getroot()
    model = root.find("model")
    print("=" * 100)
    print("文件: %s   (%d bytes)" % (a.sdf, os.path.getsize(a.sdf)))
    print("sha256: %s" % __import__("hashlib").sha256(open(a.sdf, "rb").read()).hexdigest())
    print("<model name='%s'>   <sdf version='%s'>" % (model.get("name"), root.get("version")))
    print("=" * 100)

    # ---------- 1. 传感器安装位姿表 ----------
    print("\n### 1. 传感器安装位姿表\n")
    hdr = ("%-22s %-22s %-24s %-24s %-20s %-9s" %
           ("所属 <link>", "sensor name", "type", "<pose> (x y z r p y)",
            "frameName", "pitch(deg)"))
    print(hdr); print("-" * len(hdr))
    for link in model.findall("link"):
        for sen in link.findall("sensor"):
            p = txt(sen, "pose", "0 0 0 0 0 0")
            plug = sen.find("plugin")
            fn = txt(plug, "frameName")
            xyz, rpy, pdeg, _ = pose6(p)
            print("%-22s %-22s %-24s %-24s %-20s %-9s" %
                  (link.get("name"), sen.get("name"), sen.get("type"),
                   " ".join(p.split()[:6]), fn, pdeg))
    print("\n注意: 无 <pose> 的 sensor 取所属 link 原点 (0 0 0 0 0 0)。")
    print("      <pose> 顺序为 x y z roll pitch yaw；上表 pitch 列为第 5 个分量。")

    # ---------- 2. 插件/话题/关节 ----------
    print("\n### 2. 传感器插件与话题\n")
    for link in model.findall("link"):
        for sen in link.findall("sensor"):
            for plug in sen.findall("plugin"):
                print("link=%-18s sensor=%-14s plugin=%-22s lib=%s" %
                      (link.get("name"), sen.get("name"),
                       plug.get("name"), plug.get("filename")))
                for k in ("robotNamespace", "topicName", "cameraName", "frameName",
                          "updateRate", "alwaysOn", "baseline", "pointCloudCutoff",
                          "pointCloudCutoffMax", "hackBaseline"):
                    v = txt(plug, k)
                    if v != "" or plug.find(k) is not None:
                        print("      %-22s = %s" % (k, v if v != "" else "(空)"))
    print("\n### 2b. 云台四连杆关节\n")
    print("%-30s %-14s %-26s %-10s %-14s %-14s" %
          ("joint", "type", "<pose>", "axis xyz", "lower", "upper"))
    print("-" * 112)
    for j in model.findall("joint"):
        if j.get("name") in GIMBAL_JOINTS:
            ax = j.find("axis"); lim = ax.find("limit") if ax is not None else None
            print("%-30s %-14s %-26s %-10s %-14s %-14s" %
                  (j.get("name"), j.get("type"), txt(j, "pose", "(无)"),
                   txt(ax, "xyz"), txt(lim, "lower"), txt(lim, "upper")))

    # ---------- 3. 参数对照 ----------
    print("\n### 3. 参数对照 (提交值 vs XTDrone 内置基准)\n")
    POSE_KEYS = ("<pose>",)

    def cmp_rows(title, pairs):
        print("--- %s ---" % title)
        print("%-26s %-22s %-22s %s" % ("参数", "提交值", "XTDrone 基准", "判定"))
        print("-" * 92)
        for k, (got, ref) in pairs.items():
            got = got if got != "" else "文件中未找到"
            ref = ref if ref != "" else "基准未定义"
            if k in POSE_KEYS:
                flag = "🟡 安装位姿(规则允许自由)"
            elif ref in ("基准未定义",):
                flag = "⚠️ 基准未定义"
            elif got == ref or got.replace("'", "") == ref:
                flag = "✅ 一致"
            else:
                flag = "🔴 不一致"
            print("%-26s %-22s %-22s %s" % (k, got, ref, flag))
        print()

    # 找 depth / stereo_right / camera / laser
    def find_sensor(name):
        for link in model.findall("link"):
            for sen in link.findall("sensor"):
                if sen.get("name") == name:
                    return sen
        return None

    dep = find_sensor("depth")
    if dep is not None:
        dp = dep.find("plugin")
        cmp_rows("深度/RGB-D 相机 (sensor name='depth')", {
            "<pose>": (txt(dep, "pose"), REF_DEPTH["pose"]),
            "update_rate(传感器)": (txt(dep, "update_rate"), REF_DEPTH["update_rate"]),
            "horizontal_fov": (txt(dep, "camera/horizontal_fov"), REF_DEPTH["horizontal_fov"]),
            "width": (txt(dep, "camera/image/width"), REF_DEPTH["width"]),
            "height": (txt(dep, "camera/image/height"), REF_DEPTH["height"]),
            "clip/near": (txt(dep, "camera/clip/near"), REF_DEPTH["near"]),
            "clip/far": (txt(dep, "camera/clip/far"), REF_DEPTH["far"]),
            "frameName": (txt(dp, "frameName"), REF_DEPTH["frameName"]),
            "cameraName": (txt(dp, "cameraName"), REF_DEPTH["cameraName"]),
            "pointCloudCutoff": (txt(dp, "pointCloudCutoff"), REF_DEPTH["pointCloudCutoff"]),
            "pointCloudCutoffMax": (txt(dp, "pointCloudCutoffMax"), REF_DEPTH["pointCloudCutoffMax"]),
            "plugin filename": (s(dp.get("filename")),
                                "libgazebo_ros_openni_kinect.so"),
        })
        try:
            got = float(txt(dep, "camera/horizontal_fov")); ref = float(REF_DEPTH["horizontal_fov"])
            print("  FOV 倍数 = %.3f  (提交 %.4f rad = %.2f°  vs 基准 %.6f rad = %.2f°)\n"
                  % (got / ref, got, got * 180 / math.pi, ref, ref * 180 / math.pi))
        except Exception:
            pass

    for extra in ("stereo_right", "stereo_left", "stereo", "camera"):
        sen = find_sensor(extra)
        if sen is None:
            print("--- sensor name='%s': 文件中未找到 ---\n" % extra); continue
        p = sen.find("plugin")
        cmp_rows("附加相机 (sensor name='%s')" % extra, {
            "<pose>": (txt(sen, "pose"), REF_DEPTH["pose"]),
            "update_rate": (txt(sen, "update_rate"), REF_DEPTH["update_rate"]),
            "horizontal_fov": (txt(sen, "camera/horizontal_fov"), REF_DEPTH["horizontal_fov"]),
            "width": (txt(sen, "camera/image/width"), REF_DEPTH["width"]),
            "height": (txt(sen, "camera/image/height"), REF_DEPTH["height"]),
            "frameName": (txt(p, "frameName"), REF_DEPTH["frameName"]),
            "cameraName": (txt(p, "cameraName"), REF_DEPTH["cameraName"]),
            "pointCloudCutoffMax": (txt(p, "pointCloudCutoffMax"), REF_DEPTH["pointCloudCutoffMax"]),
        })

    las = find_sensor("laser")
    if las is not None:
        lp = las.find("plugin")
        cmp_rows("2D 激光雷达 (sensor name='laser')", {
            "<pose>": (txt(las, "pose"), REF_LASER_TYPHOON["pose"]),
            "update_rate": (txt(las, "update_rate"), REF_LASER_TYPHOON["update_rate"]),
            "samples": (txt(las, "ray/scan/horizontal/samples"), REF_LASER_TYPHOON["samples"]),
            "resolution": (txt(las, "ray/scan/horizontal/resolution"), REF_LASER_TYPHOON["resolution"]),
            "min_angle": (txt(las, "ray/scan/horizontal/min_angle"), REF_LASER_TYPHOON["min_angle"]),
            "max_angle": (txt(las, "ray/scan/horizontal/max_angle"), REF_LASER_TYPHOON["max_angle"]),
            "range/min": (txt(las, "ray/range/min"), REF_LASER_TYPHOON["range_min"]),
            "range/max": (txt(las, "ray/range/max"), REF_LASER_TYPHOON["range_max"]),
            "range/resolution": (txt(las, "ray/range/resolution"), REF_LASER_TYPHOON["range_resolution"]),
            "frameName": (txt(lp, "frameName"), REF_LASER_TYPHOON["frameName"]),
            "topicName": (txt(lp, "topicName"), REF_LASER_TYPHOON["topicName"]),
            "plugin name": (s(lp.get("name")), REF_LASER_TYPHOON["plugin_name"]),
            "plugin filename": (s(lp.get("filename")), "libgazebo_ros_laser.so"),
        })
        try:
            got = float(txt(las, "ray/range/max")); ref = float(REF_LASER_TYPHOON["range_max"])
            print("  range/max 倍数 = %.2f  (提交 %s m vs 基准 %s m)\n" % (got / ref, got, ref))
        except Exception:
            pass

    # 云台关节限位对照
    print("--- 云台关节限位 ---")
    print("%-30s %-22s %-22s %s" % ("joint", "提交 lower/upper", "官方 lower/upper", "判定"))
    print("-" * 92)
    for jn in GIMBAL_JOINTS:
        for j in model.findall("joint"):
            if j.get("name") == jn:
                lim = j.find("axis/limit")
                lo, up = txt(lim, "lower"), txt(lim, "upper")
                rlo, rup = REF_GIMBAL_LIMITS.get(jn, ("-", "-"))
                same = (lo == rlo and up == rup)
                print("%-30s %-22s %-22s %s" % (jn, "%s / %s" % (lo, up),
                                                "%s / %s" % (rlo, rup),
                                                "✅ 一致" if same else "🔴 不一致"))
    print()

    # 其它插件与端口
    print("--- 其它：端口 / 插件 / 模型名 ---")
    for plug in model.findall("plugin"):
        n = plug.get("name")
        if n == "mavlink_interface":
            for k in ("mavlink_tcp_port", "mavlink_udp_port", "qgc_udp_port",
                      "sdk_udp_port", "gpsSubTopic", "use_tcp", "enable_lockstep"):
                print("  mavlink_interface/%-20s = %s" % (k, txt(plug, k, "文件中未找到")))
        elif n == "gimbal_controller":
            print("  gimbal_controller: udp_gimbal_port_remote=%s joint_yaw=%s joint_roll=%s joint_pitch=%s gimbal_imu=%s"
                  % (txt(plug, "udp_gimbal_port_remote"), txt(plug, "joint_yaw"),
                     txt(plug, "joint_roll"), txt(plug, "joint_pitch"), txt(plug, "gimbal_imu")))
        elif n in ("magnetometer_plugin", "barometer_plugin", "gazebo_imu_plugin",
                   "groundtruth_plugin", "gps_plugin"):
            print("  %-22s lib=%s" % (n, plug.get("filename")))
    inc = [(s(i.find("uri").text) if i.find("uri") is not None else "?",
            txt(i, "pose", "(无)")) for i in model.findall("include")]
    print("  <include> 传感器模型: %s" % (inc if inc else "无"))
    print("  模型名冲突检查: <model name='%s'> (官方内置同名, 多机同世界需改名)"
          % model.get("name"))


if __name__ == "__main__":
    main()
