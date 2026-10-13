#!/bin/bash
# 分数进展采样器：每 INTERVAL 秒把裁判端关键指标追加到日志。
# 用法：bash tools/score_watch.sh [间隔秒] [日志路径]
INTERVAL="${1:-20}"
LOG="${2:-logs/score_watch.log}"
mkdir -p "$(dirname "$LOG")"
source /opt/ros/noetic/setup.bash >/dev/null 2>&1
[ -f devel/setup.bash ] && source devel/setup.bash >/dev/null 2>&1
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"

echo "# 起始 $(date '+%F %T')  间隔 ${INTERVAL}s  master=$ROS_MASTER_URI" >> "$LOG"
while true; do
    S=$(timeout 5 rostopic echo -n1 /score 2>/dev/null | awk '{print $2}')
    T=$(timeout 5 rostopic echo -n1 /time_usage 2>/dev/null | awk '{print $2}')
    L=$(timeout 5 rostopic echo -n1 /left_actors 2>/dev/null | sed 's/.*data: //')
    N=$(echo "$L" | tr -cd ',' | wc -c)          # 剩余目标数-1
    F=0
    for i in 0 1 2 3 4 5; do
        timeout 3 rostopic echo -n1 /find_actor_$i 2>/dev/null | grep -q . && F=$((F+1))
    done
    printf '%s  score=%-6s time=%-6s left=%s  剩余目标=%s  find最近上报=%s\n' \
        "$(date '+%H:%M:%S')" "${S:-?}" "${T:-?}" "${L:-?}" "$((N+1))" "$F" >> "$LOG"
    sleep "$INTERVAL"
done
