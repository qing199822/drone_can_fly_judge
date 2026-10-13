#!/bin/bash
# S16 短跑验证脚本：跑 3~5 分钟后统计 baro timeout / Warning 计数
# 用法：
#   bash tools/smoke_baro_check.sh <round_dir> [<min_duration_sec>]
# 默认 round_dir = logs/referee_only_<latest>
# 默认 min_duration_sec = 180
set -e
ROUND="${1:-$(ls -td logs/referee_only_* 2>/dev/null | head -1)}"
MIN_DURATION="${2:-180}"
if [ -z "$ROUND" ] || [ ! -d "$ROUND" ]; then
    echo "错误：找不到 round dir，请传 $0 <round_dir>" >&2
    exit 2
fi

echo "==> Round: $ROUND"

START=$(stat -c %Y "$ROUND"/gzserver.log 2>/dev/null || echo 0)
NOW=$(date +%s)
DURATION=$(( NOW - START ))
echo "==> Round 已运行 ${DURATION}s（最低要求 ${MIN_DURATION}s）"
if [ "$DURATION" -lt "$MIN_DURATION" ]; then
    echo "警告：运行不足 ${MIN_DURATION}s，结论仅供参考"
fi

echo
echo "=== 1) baro hgt timeout 统计（按 PX4 实例）==="
TOTAL_BARO=0
for f in "$ROUND"/../serial_bringup/px4_*.log "$ROUND"/serial_bringup/px4_*.log; do
    [ -f "$f" ] || continue
    COUNT=$(grep -ac 'baro hgt timeout' "$f" 2>/dev/null || echo 0)
    BASENAME=$(basename "$f")
    echo "  $BASENAME : $COUNT"
    TOTAL_BARO=$(( TOTAL_BARO + COUNT ))
done
echo "  TOTAL : $TOTAL_BARO"

echo
echo "=== 2) EKF reset position / reset to gps 统计 ==="
for f in "$ROUND"/../serial_bringup/px4_*.log "$ROUND"/serial_bringup/px4_*.log; do
    [ -f "$f" ] || continue
    RESET_POS=$(grep -ac 'reset position to' "$f" 2>/dev/null || echo 0)
    RESET_GPS=$(grep -ac 'reset to gps' "$f" 2>/dev/null || echo 0)
    BASENAME=$(basename "$f")
    echo "  $BASENAME : reset_position=$RESET_POS reset_to_gps=$RESET_GPS"
done

echo
echo "=== 3) Warning: UAV .* is higher than 6 meters ==="
HIGH=0
for f in "$ROUND"/referee.log; do
    [ -f "$f" ] || continue
    HIGH=$(grep -ac 'Warning: UAV .* is higher than 6 meters' "$f" 2>/dev/null || echo 0)
    echo "  $(basename "$f") : $HIGH"
done
echo "  TOTAL Warning: $HIGH"

echo
echo "=== 4) pose_z=-1761 sentinel（应大幅减少） ==="
for f in "$ROUND"/../sim_only_*/sim.log "$ROUND"/sim.log; do
    [ -f "$f" ] || continue
    SENTINEL=$(grep -ac 'pose_z=-1761' "$f" 2>/dev/null || echo 0)
    BASENAME=$(basename "$f")
    echo "  $BASENAME : pose_z=-1761 出现 $SENTINEL 次"
done

echo
echo "=== 5) final score.txt ==="
cat "$ROUND"/score.txt 2>/dev/null || echo "（未写出）"

echo
echo "==> 判定（不区分 hard failure，仅供观察）："
if [ "$HIGH" -gt 0 ]; then
    echo "  [FAIL] 触发 $HIGH 次 UAV .* is higher than 6 meters"
else
    echo "  [OK] 没有触发 UAV 高度警告"
fi
if [ "$TOTAL_BARO" -gt 5 ]; then
    echo "  [WARN] baro hgt timeout 总计 $TOTAL_BARO 次（建议继续修 EKF 容限）"
else
    echo "  [OK] baro hgt timeout 总计 $TOTAL_BARO 次（< 5）"
fi