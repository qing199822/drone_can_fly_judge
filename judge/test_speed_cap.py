#!/usr/bin/env python3
"""
2026-09-26：speed_cap 限制 + latch 不抹零 v_pub 的单测。

复现 src/auto_explore/src/area_mission_node.cpp:2928-2932 + 3174-3182：
  - env_clearance 越小，speed_cap 越小；
  - latch 不再把 cap 拉成 0（用户已识别为设计 bug）—— 改为 cap = speed_cap，
    让 MPPI cost map 仍能用 0.3 m/s 的最小速度绕障；
  - patrol_speed_envelope_min_=0.3 作为 speed_cap 的下限（env_clearance < 1.5 时仍给 0.3）。
"""

from dataclasses import dataclass


@dataclass
class SpeedCapConfig:
    patrol_speed_envelope_min_: float = 0.3
    patrol_speed_limit_: float = 1.5
    cap_overlap_min_: float = 1.5
    cap_overlap_max_: float = 6.0


def compute_speed_cap(env_clearance: float, cfg: SpeedCapConfig):
    """复现 area_mission_node.cpp:2929-2930。"""
    raw = 1.5 * (env_clearance - cfg.cap_overlap_min_) / (cfg.cap_overlap_max_ - cfg.cap_overlap_min_)
    return max(cfg.patrol_speed_envelope_min_, min(cfg.patrol_speed_limit_, raw))


def apply_latch_cap(cmd_linear_x: float, speed_cap: float):
    """复现 area_mission_node.cpp:3174-3182（修法版本）。

    旧版本：const double cap = front_inhibit_ ? 0.0 : speed_cap;（latch 期间 cap=0，v_pub=0）
    新版本：const double cap = speed_cap;（latch 仅限速，不抹零）
    """
    if cmd_linear_x > speed_cap:
        return speed_cap
    return cmd_linear_x


def expected_speed_caps():
    return [
        (1.5, 0.3),
        (2.5, 0.33),
        (3.0, 0.5),
        (4.5, 1.0),
        (6.0, 1.5),
        (10.0, 1.5),
        (1.0, 0.3),
    ]


def run_tests():
    cfg = SpeedCapConfig()
    failures = []

    print("=" * 78)
    print("speed_cap 计算 vs env_clearance")
    print("=" * 78)
    for ec, expected in expected_speed_caps():
        got = compute_speed_cap(ec, cfg)
        ok = abs(got - expected) < 0.02
        marker = "✓" if ok else "❌"
        print(f"  {marker} env_clearance={ec:.2f} m → cap={got:.2f} m/s (期望 {expected:.2f})")
        if not ok:
            failures.append((f"speed_cap({ec})", got, expected))

    print()
    print("=" * 78)
    print("latch cap 应用：cmd.linear_x 不会因 latch 被抹成 0")
    print("=" * 78)
    test_cases = [
        (1.5, 1.5, 1.5),
        (0.8, 1.5, 0.8),
        (0.5, 0.3, 0.3),    # env_close = 0.3 时 cap=0.3，cmd=0.5 被压到 0.3（不归零）
        (0.1, 0.3, 0.1),
        (-0.5, 0.3, -0.5),
        (1.0, 0.3, 0.3),    # MPPI 输出 1.0 m/s 但 cap=0.3 → 被压到 0.3（之前会被抹 0）
    ]
    for cmd_x, cap, expected in test_cases:
        got = apply_latch_cap(cmd_x, cap)
        ok = abs(got - expected) < 1e-6
        marker = "✓" if ok else "❌"
        print(f"  {marker} cmd={cmd_x:.2f} cap={cap:.2f} → {got:.2f}（期望 {expected:.2f}）")
        if not ok:
            failures.append((f"apply_latch_cap({cmd_x},{cap})", got, expected))

    print()
    print("=" * 78)
    if failures:
        print(f"❌ 失败 {len(failures)} 项：")
        for name, got, expected in failures:
            print(f"  {name}: got={got} expected={expected}")
        return 1
    print("✓ 全部通过")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(run_tests())