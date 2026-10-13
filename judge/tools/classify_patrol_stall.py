#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""归类 area_mission diag 里"前向不动（v_pub≈0）"的时段，指出真正的机制。

为什么要这个工具：
    无人机"原地悬停 / 左右横移 / 不巡逻"可能来自完全不同的机制，只看 v_pub=0 分不清：
      1. MPPI 无效硬停：`mppi_valid=0` 且 `stop=4`（mppi_planner.hpp 里
         `if (!field_valid_ || !scan_valid_) return Control()` 的后果）；
      2. PATROL 调 recovery：`recovery_state=1/2` 或 `stop=9/11`（前向被清、只留侧移）；
      3. 根本不在 PATROL：`mode!=2`（TRANSIT/ROAD，wp_idx 一直停在 0）；
      4. 传感器失效：`stop=1/2/3`（scan/pose）；
      5. 深度/激光硬停：`stop=5/6`；
      6. 区域边界：`stop=7`；
      7. 其它：MPPI 有效但最优解就是 v≈0（前方被堵时的局部极小）。

用法：
    python3 tools/classify_patrol_stall.py uav_3_diag.log [uav_0_diag.log ...]
    python3 tools/classify_patrol_stall.py --dir logs/referee_round/round_xxx --rate 10
    python3 tools/classify_patrol_stall.py --dir logs/xxx --min-seconds 2 --top 5

diag 采样没有时间戳，用 --rate（默认 10 Hz，本仓库 diag_rate=5 时传 5）折算秒数。
"""

import argparse
import glob
import os
import re
import sys


def parse_samples(path):
    """把 diag 文本解析成 [{key: float/str}, ...]。"""
    raw = open(path, errors="replace").read()
    # 合并行尾 '\' 续行
    raw = raw.replace("\\\n", " ")
    samples = []
    for line in raw.splitlines():
        if "data:" not in line:
            continue
        m = re.search(r'data:\s*"?(.*?)"?\s*$', line)
        if not m:
            continue
        body = m.group(1)
        rec = {}
        for k, v in re.findall(r"([a-zA-Z_][a-zA-Z0-9_]*)=([^\s\"]+)", body):
            try:
                rec[k] = float(v)
            except ValueError:
                rec[k] = v
        if "v_pub" in rec:
            samples.append(rec)
    return samples


def classify(rec):
    """给单个样本定一个"为什么不动"的标签。"""
    mode = int(rec.get("mode", -1))
    stop = int(rec.get("stop", -1))
    mppi_valid = int(rec.get("mppi_valid", -1))
    recov = int(rec.get("recovery_state", 0))
    if mode != 2:
        return "非PATROL(mode=%d)" % mode
    if mppi_valid == 0 and stop == 4:
        return "MPPI无效硬停(stop=4)"
    if stop in (1, 2, 3):
        return "scan/pose失效(stop=%d)" % stop
    if stop in (5, 6):
        return "深度/激光硬停(stop=%d)" % stop
    if stop == 7:
        return "区域边界(stop=7)"
    if stop == 8:
        return "PAUSE(stop=8)"
    if recov in (1, 2) or stop in (9, 11):
        return "PATROL recovery(侧移/超时)"
    if int(rec.get("front_inhibit", 0)) == 1:
        return "front_inhibit(前向抑制)"
    if mppi_valid == 0:
        return "MPPI无效(其它 stop=%d)" % stop
    return "MPPI有效但选停(v=0)"


def stuck_episodes(samples, rate, min_seconds):
    """把连续 v_pub≈0 的采样切成时段。"""
    eps = []
    cur = []
    for rec in samples:
        if float(rec.get("v_pub", 0.0)) < 0.05:
            cur.append(rec)
        else:
            if cur:
                eps.append(cur)
                cur = []
    if cur:
        eps.append(cur)
    out = []
    for ep in eps:
        dur = len(ep) / rate
        if dur < min_seconds:
            continue
        from collections import Counter
        cause = Counter(classify(r) for r in ep).most_common(1)[0][0]
        lateral = sum(1 for r in ep if abs(float(r.get("v_lat_pub", 0.0))) > 0.1)
        last = ep[-1]
        out.append({
            "cycles": len(ep), "seconds": dur, "cause": cause,
            "lateral_ratio": lateral / len(ep),
            "fwd_lidar": last.get("fwd_lidar"), "fwd_depth": last.get("fwd_depth"),
            "scan_valid": last.get("scan_valid"), "mppi_valid": last.get("mppi_valid"),
            "stop": last.get("stop"), "recovery_state": last.get("recovery_state"),
            "pose_z": last.get("pose_z"), "wp": last.get("patrol_wp_idx"),
            "env_clearance": last.get("env_clearance"),
        })
    return out


def report(path, rate, min_seconds, top):
    samples = parse_samples(path)
    if not samples:
        print("  (没有解析到 diag 采样)")
        return
    eps = stuck_episodes(samples, rate, min_seconds)
    stuck = sum(e["cycles"] for e in eps)
    print("== %s ==" % os.path.basename(path))
    print("   采样 %d，其中 v_pub≈0 时段 %d 段 / 共 %.1f s（占 %.0f%%）"
          % (len(samples), len(eps), stuck / rate, 100.0 * stuck / max(1, len(samples))))
    from collections import Counter
    cnt = Counter(e["cause"] for e in eps)
    for cause, n in cnt.most_common():
        t = sum(e["seconds"] for e in eps if e["cause"] == cause)
        print("     %-28s %2d 段 / %6.1f s" % (cause, n, t))
    for e in sorted(eps, key=lambda x: -x["seconds"])[:top]:
        print("   最长: %5.1fs  %-26s wp=%-4s mppi_valid=%s stop=%s recov=%s "
              "fwd_lidar=%s fwd_depth=%s scan_valid=%s env=%s z=%s 侧移占比=%.0f%%"
              % (e["seconds"], e["cause"], e["wp"], e["mppi_valid"], e["stop"],
                 e["recovery_state"], e["fwd_lidar"], e["fwd_depth"], e["scan_valid"],
                 e["env_clearance"], e["pose_z"], 100 * e["lateral_ratio"]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="*", help="diag 日志文件")
    ap.add_argument("--dir", help="目录（自动找 *_diag.log）")
    ap.add_argument("--rate", type=float, default=10.0, help="diag 采样率 Hz（默认 10）")
    ap.add_argument("--min-seconds", type=float, default=2.0, help="只报 >= 该时长的停顿时段")
    ap.add_argument("--top", type=int, default=3, help="每个文件打印最长的几段")
    args = ap.parse_args()

    files = list(args.files)
    if args.dir:
        files += sorted(glob.glob(os.path.join(args.dir, "*_diag.log")))
    if not files:
        ap.print_help()
        return 1
    for f in files:
        report(f, args.rate, args.min_seconds, args.top)
    return 0


if __name__ == "__main__":
    sys.exit(main())
