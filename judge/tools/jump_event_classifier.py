#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单步位移异常 + 同刻 stop_code 分类脚本（2026-09-23）。

判定"撞墙 vs 定位跳变"的口径：

  1) 取每台 UAV 的 pose.log，按时间顺序算相邻两个 sample 的位移 step_disp；
  2) 同一个 sample 的 timestamp 差 dt；
  3) 单步异常定义为 step_disp > JUMP_THRESHOLD（默认 3.0m）；
  4) 把同刻的 diag 文本解析，得到 stop_code/mode/altitude；
  5) 分类规则：
     - 跳变时刻前后 1s 内 stop_code == 10 (ALT_RECOVER) 或 mode 切换 → **定位/EKF 跳变**；
     - 跳变前后速度近 0（|v_pub| < 0.5）→ **物理上不可能撞墙**（撞墙必有减速过程），归到 EKF 跳变；
     - 跳变时刻前向速度高（|v_pub| > 1.5）+ 同刻有 contact → 真撞墙（但 sim.log 没记 contact，跳不到这条）；
     - 跳变前后速度高、front 小、stop_code == 5/6（depth/laser hard stop）→ **疑似撞墙**；
     - 其他 → 存疑，需要看 sim.log 里是否真有 collision 记录。

输出：每个 round 一份表格 + 一段总结。
"""

import argparse
import re
import sys
import yaml
from collections import Counter, defaultdict
from pathlib import Path

STOP_CODE_NAME = {
    0: "无", 1: "激光无效", 2: "激光短时过期", 3: "位姿过期/异常",
    4: "MPPI 无效", 5: "深度硬停", 6: "TTC/激光硬停", 7: "区域边界",
    8: "暂停", 9: "PATROL 恢复超时", 10: "ALT_RECOVER",
}

JUMP_THRESHOLD_DEFAULT = 3.0


def parse_pose_yaml(path):
    rows = []
    raw = Path(path).read_text(errors='replace')
    for d in raw.split('\n---\n'):
        try:
            y = yaml.safe_load(d)
        except Exception:
            continue
        if not y or 'pose' not in y:
            continue
        try:
            t = y['header']['stamp']['secs'] + y['header']['stamp']['nsecs'] * 1e-9
            x = float(y['pose']['position']['x'])
            yy = float(y['pose']['position']['y'])
            z = float(y['pose']['position']['z'])
            rows.append((t, x, yy, z))
        except Exception:
            continue
    rows.sort(key=lambda r: r[0])
    return rows


def parse_diag(path):
    raw = Path(path).read_text(errors='replace')
    samples = []
    buf = []
    for line in raw.splitlines():
        if line.startswith('#'):
            continue
        if line.startswith('data:'):
            if buf:
                samples.append(' '.join(buf))
            buf = [line]
        else:
            buf.append(line)
    if buf:
        samples.append(' '.join(buf))
    out = []
    for s in samples:
        s = s.replace('\\', ' ')
        d = {}
        for k in ('mode', 'stop', 'mppi_valid', 'v_pub', 'v_mppi', 'recovery_state',
                  'region_clamp', 'region_return', 'front_inhibit', 'fwd_lidar',
                  'dist', 'pose_z', 'recovery', 'hover_skip'):
            m = re.search(r'\b' + k + r'=([^\s\\]+)', s)
            if m:
                v = m.group(1)
                if v in ('true', 'false', 'none'):
                    d[k] = v
                else:
                    try:
                        d[k] = float(v)
                    except ValueError:
                        pass
        if d:
            # diag 文本里没有时间戳，先按 diag_rate 推 _t（diag_rate 约 5~10Hz，间隔 0.1~0.2s）。
            # 若需要更精确的 _t，把日志改成 ROS bag 回放即可。本脚本只用于粗筛。
            if '_t' not in d:
                # 用 pose 的时间序可能更准，但 pose 与 diag 不同步；
                # 这里取 diag 的下标 * 估计间隔。粗筛够用。
                pass
            out.append(d)
    # 估算 _t：用累计样本数 / 平均频率（≈9Hz）。
    # 因为每轮 diag 间隔是 ~0.107s（9.33Hz），直接用 idx * 0.107 即可。
    avg_dt = 0.107
    for i, d in enumerate(out):
        d['_t'] = i * avg_dt
    return out


def diag_at(diags, t, window=5.0):
    """返回最接近 t 的 diag 样本（窗口 ±window 秒内），找不到返回 None。

    diag 是按 (idx * 0.107s) 推算 _t 的，~5s 窗口对 9.33Hz 来说能覆盖 ~47 个样本，
    足够稳。"""
    if not diags:
        return None
    best = None
    best_dt = 1e9
    for d in diags:
        if '_t' not in d:
            continue
        dt = abs(d['_t'] - t)
        if dt < best_dt:
            best_dt = dt
            best = d
    return best if best_dt <= window else None


def detect_jumps(pose_rows, threshold):
    """返回 [(t_prev, t_curr, x_prev, y_prev, x_curr, y_curr, step_disp, dt)] 列表。"""
    out = []
    for i in range(1, len(pose_rows)):
        t0, x0, y0, _ = pose_rows[i-1]
        t1, x1, y1, _ = pose_rows[i]
        d = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
        dt = t1 - t0
        if d >= threshold:
            out.append((t0, t1, x0, y0, x1, y1, d, dt))
    return out


def classify_jump(step_disp, dt, diag_prev, diag_curr, diag_next):
    """根据跳变前后的 diag 样本分类。

    返回 (label, reason)
    label ∈ {ekf_jump, hover_jump, collision_likely, unknown}
    """
    if dt <= 0:
        return ('unknown', 'dt=0（同时间戳多帧）')

    speed_implied = step_disp / dt  # 物理上"瞬移速度"

    # 拿前后 ±1 样本
    samples = [s for s in (diag_prev, diag_curr, diag_next) if s is not None]

    # 1) 跳变时刻前后 ALT_RECOVER（stop=10）或位姿异常（stop=3）
    if samples:
        if any(s.get('stop') == 10 for s in samples):
            return ('ekf_jump', f"stop=10 ALT_RECOVER 同步出现，瞬时速度 {speed_implied:.1f} m/s")
        if any(s.get('stop') == 3 for s in samples):
            return ('ekf_jump', f"stop=3 位姿过期/异常 同步出现，瞬时速度 {speed_implied:.1f} m/s")

    # 2) 跳变前后 v_pub ≈ 0（物理上不可能撞墙）
    vpubs = [abs(s.get('v_pub', 0)) for s in samples if 'v_pub' in s]
    if vpubs and max(vpubs) < 0.5:
        return ('ekf_jump', f"跳变前后 |v_pub|<0.5（悬停状态不可能撞墙），瞬时速度 {speed_implied:.1f} m/s")

    # 3) 跳变前后 v_pub 很高 + 同刻有 contact/collision（需要 sim.log，目前无）
    # sim.log 没记 contact，所以跳不到这条

    # 4) 跳变前后 stop_code == 5/6（depth/laser hard stop）+ 前向速度高 + front 极小 → 疑似撞墙
    if samples:
        if any(s.get('stop') in (5, 6) for s in samples):
            fronts = [s.get('fwd_lidar', 1e9) for s in samples]
            if min(fronts) < 1.5:
                return ('collision_likely',
                        f"同刻 stop=5/6 + fwd_lidar<1.5m，可能真撞墙；瞬时速度 {speed_implied:.1f} m/s")

    # 5) 看 mppi_valid=0（mppi 失效）→ EKF/GPS 数据无效的概率更高
    if samples and any(s.get('mppi_valid') == 0 for s in samples):
        return ('ekf_jump', f"mppi_valid=0（传感器读数无效），瞬时速度 {speed_implied:.1f} m/s")

    return ('unknown', f"无 stop_code 共现；瞬时速度 {speed_implied:.1f} m/s")


def analyze_round(round_dir, threshold, drones=6):
    """分析一轮 round，返回每台机子的 jump 列表 + 分类统计。"""
    per_uav = {}
    summary = Counter()
    for d in range(drones):
        pose_path = round_dir / f'uav_{d}_pose.log'
        diag_path = round_dir / f'uav_{d}_diag.log'
        if not pose_path.exists():
            continue
        poses = parse_pose_yaml(pose_path)
        diags = parse_diag(diag_path) if diag_path.exists() else []
        if not poses:
            continue
        jumps = detect_jumps(poses, threshold)
        rows = []
        for (t0, t1, x0, y0, x1, y1, d, dt) in jumps:
            d_curr = diag_at(diags, t1, window=5.0)
            d_prev = diag_at(diags, t0, window=5.0)
            d_next = diag_at(diags, t1 + 1.0, window=5.0) if diags else None
            label, reason = classify_jump(d, dt, d_prev, d_curr, d_next)
            summary[label] += 1
            rows.append({
                't0': t0, 't1': t1, 'x0': x0, 'y0': y0, 'x1': x1, 'y1': y1,
                'disp': d, 'dt': dt, 'speed': d / dt if dt > 0 else float('inf'),
                'label': label, 'reason': reason,
                'stop_curr': d_curr.get('stop') if d_curr else None,
                'stop_prev': d_prev.get('stop') if d_prev else None,
                'mode_curr': d_curr.get('mode') if d_curr else None,
                'v_pub_prev': d_prev.get('v_pub') if d_prev else None,
                'v_pub_curr': d_curr.get('v_pub') if d_curr else None,
            })
        per_uav[d] = rows
    return per_uav, summary


def print_round_report(round_dir, per_uav, summary):
    print(f'\n=== {round_dir.name} ===')
    total_jumps = sum(summary.values())
    print(f'  jumps (>= threshold): {total_jumps}')
    for label, n in summary.most_common():
        print(f'    {label}: {n}')
    for d, rows in sorted(per_uav.items()):
        if not rows:
            continue
        for r in rows:
            d_curr = r['stop_curr']
            d_prev = r['stop_prev']
            stop_str = f"stop_prev={d_prev} stop_curr={d_curr}" if d_curr is not None or d_prev is not None else "无 diag"
            print(f"  uav{d} t={r['t1']:6.1f}s  ({r['x0']:6.1f},{r['y0']:6.1f})→({r['x1']:6.1f},{r['y1']:6.1f})  "
                  f"disp={r['disp']:6.1f}m  dt={r['dt']:.3f}s  {r['speed']:6.1f}m/s  "
                  f"[{r['label']}]  {stop_str}  →  {r['reason']}")


def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument('--threshold', type=float, default=JUMP_THRESHOLD_DEFAULT,
                        help='单步位移阈值 (m)，默认 3.0')
    parser.add_argument('--rounds', nargs='*', default=None,
                        help='要分析的 round 子目录名（默认最近 6 个）')
    parser.add_argument('--logs-root', default='/home/lishu2/2025_ZZU_FLY/logs/referee_round')
    args = parser.parse_args(argv)

    logs_root = Path(args.logs_root)
    if args.rounds:
        rounds = [logs_root / r for r in args.rounds]
    else:
        all_rounds = sorted(logs_root.iterdir(), key=lambda p: p.name)
        rounds = [r for r in all_rounds if r.is_dir()][-6:]

    grand_summary = Counter()
    for rd in rounds:
        per_uav, summary = analyze_round(rd, args.threshold)
        print_round_report(rd, per_uav, summary)
        grand_summary.update(summary)

    print('\n========== 总计 ==========')
    total = sum(grand_summary.values())
    print(f'  6 轮瞬移事件总数: {total}')
    for label, n in grand_summary.most_common():
        pct = 100 * n / max(1, total)
        print(f'    {label}: {n} ({pct:.0f}%)')


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]) or 0)