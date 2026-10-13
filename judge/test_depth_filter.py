#!/usr/bin/env python3
"""
2026-09-26：depthCb 过滤逻辑单测（BlackBox 复现 src/auto_explore/src/area_mission_node.cpp:741-766）。

按你的修法：
  - depth_extrinsic_z: -0.162 → -0.586（修正 +0.42 m 系统偏差）
  - cam_h > 0.6 → cam_h > 0.15（低空也做地面判定，spawn 处 pose_z=1.0+t_cam.z=-0.586 → cam_h≈0.414）
  - depth_ground_clearance: 0.5 → 0.85（吸收 ±0.3 m 噪声 + 0.42 m 外参误差）
  - 安全阈值不动：front_inhibit_depth_enter=4 / depth_obstacle_threshold=2.5 / hard_stop=2.5*(1.2+gain*v)
"""

from dataclasses import dataclass


@dataclass
class DepthPoint:
    xb: float  # 相机光心 → 机体系下的 x（机身左右）
    yb: float  # 相机光心 → 机体系下的 y（机头前后，+ = 前）
    zb: float  # 相机光心 → 机体系下的 z（光轴上下，地面点为负）
    d: float   # 沿光轴的测距（m）

    @property
    def horiz(self):
        return (self.xb ** 2 + self.yb ** 2) ** 0.5


@dataclass
class DepthFilter:
    depth_self_near: float = 1.5
    depth_self_radius: float = 1.0
    depth_ground_clearance_old: float = 0.5
    depth_ground_clearance_new: float = 0.85
    cam_h_ground_min_old: float = 0.6
    cam_h_ground_min_new: float = 0.15

    def accept_old(self, p: DepthPoint, cam_h: float):
        if p.d < self.depth_self_near and p.horiz < self.depth_self_radius:
            return False
        ground_ok = self.cam_h_ground_min_old <= cam_h < 20.0
        if ground_ok and (cam_h + p.zb) < self.depth_ground_clearance_old:
            return False
        return True

    def accept_new(self, p: DepthPoint, cam_h: float):
        if p.d < self.depth_self_near and p.horiz < self.depth_self_radius:
            return False
        ground_ok = self.cam_h_ground_min_new <= cam_h < 20.0
        if ground_ok and (cam_h + p.zb) < self.depth_ground_clearance_new:
            return False
        return True


def expected_filter_decisions():
    return [
        {"name": "spawn 低空 + 地面点（光轴正下，旧过滤也能滤掉）",
         "cam_h": 0.838,
         "points": [DepthPoint(xb=0, yb=0, zb=-2.0, d=1.93)],
         "old_keep": [False], "new_keep": [False]},
        {"name": "spawn 水平方向 0.6 m（其他 UAV 桨叶）—— 自机过滤应滤掉",
         "cam_h": 0.838,
         "points": [DepthPoint(xb=0.0, yb=0.6, zb=0.5, d=0.6)],
         "old_keep": [False], "new_keep": [False]},
        {"name": "目标高度 4.5 m + 远距地面点 zb=-4",
         "cam_h": 4.5,
         "points": [DepthPoint(xb=0, yb=4.0, zb=-4.0, d=4.0)],
         "old_keep": [True], "new_keep": [False]},
        {"name": "自机部件 (桨/脚架)",
         "cam_h": 4.5,
         "points": [DepthPoint(xb=0.0, yb=0.0, zb=0.3, d=0.8)],
         "old_keep": [False], "new_keep": [False]},
        {"name": "真障碍：人在前方 2 m",
         "cam_h": 4.5,
         "points": [DepthPoint(xb=0.0, yb=2.0, zb=-2.8, d=2.0)],
         "old_keep": [True], "new_keep": [True]},
        {"name": "墙紧贴：前方 0.4 m，地面附近 0.5 m 高（被自机过滤先吃掉）",
         "cam_h": 4.5,
         "points": [DepthPoint(xb=0.0, yb=0.4, zb=-0.5, d=0.4)],
         "old_keep": [False], "new_keep": [False]},
        {"name": "低空 (cam_h=0.5) + 远处地面点（光轴正下）",
         "cam_h": 0.5,
         "points": [DepthPoint(xb=0, yb=0, zb=-1.93, d=1.93)],
         "old_keep": [True], "new_keep": [False]},
        {"name": "低空 (cam_h=0.5) + 较近地面点 (zb=-1.0)",
         "cam_h": 0.5,
         "points": [DepthPoint(xb=0, yb=1.0, zb=-1.0, d=1.12)],
         "old_keep": [True], "new_keep": [False]},
        {"name": "低空 + 远处地面 + 横向偏移 (zb=-0.2)",
         "cam_h": 0.5,
         "points": [DepthPoint(xb=0.5, yb=2.0, zb=-0.2, d=2.05)],
         "old_keep": [True], "new_keep": [False]},
        {"name": "目标高度 + 远距地面 (zb=-2.0)",
         "cam_h": 4.5,
         "points": [DepthPoint(xb=0, yb=2.0, zb=-2.0, d=2.83)],
         "old_keep": [True], "new_keep": [True]},
    ]


def run_tests():
    f = DepthFilter()
    cases = expected_filter_decisions()
    failures = []
    print("=" * 78)
    print("深度过滤单测：旧 launch 默认 vs 新建议（外参+0.42 m / 判别带 0.85）")
    print("=" * 78)
    for case in cases:
        print(f"\n[场景] {case['name']}  cam_h={case['cam_h']}m")
        all_pass = True
        for i, p in enumerate(case["points"]):
            old_keep = f.accept_old(p, case["cam_h"])
            new_keep = f.accept_new(p, case["cam_h"])
            cam_h_zb = case["cam_h"] + p.zb
            print(f"  pt{i} d={p.d:.2f} xb={p.xb:+.2f} yb={p.yb:+.2f} zb={p.zb:+.2f} "
                  f"| cam_h+zb={cam_h_zb:+.2f}")
            print(f"    旧 launch 接受={old_keep}（期望 {case['old_keep'][i]}）")
            print(f"    新建议 接受={new_keep}（期望 {case['new_keep'][i]}）")
            if old_keep != case["old_keep"][i]:
                failures.append((case["name"], "旧", i, old_keep, case["old_keep"][i]))
                all_pass = False
            if new_keep != case["new_keep"][i]:
                failures.append((case["name"], "新", i, new_keep, case["new_keep"][i]))
                all_pass = False
        if all_pass:
            print("    ✓ 通过")
    print("\n" + "=" * 78)
    if failures:
        print(f"❌ 失败 {len(failures)} 项：")
        for name, who, idx, got, expected in failures:
            print(f"  {name} ({who}#{idx}): got={got} expected={expected}")
        return 1
    print("✓ 全部通过")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(run_tests())