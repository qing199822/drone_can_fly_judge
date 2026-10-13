#!/bin/bash
# ============================================================================
# 一轮双机联调：一键起裁判端（+可选 GUI +采分器），并打印算法机的接入命令
#
#   bash tools/round_start.sh [选项]
#
# 选项（透传给 start_official_judge.sh 的还会原样转发）：
#   --seed=N              地图种子（默认按时间生成）
#   --model=zzufly|official   机型（默认 zzufly = 带我们声明的传感器）
#   --sensors=zzu|...     传感器申报口径（默认 zzu = 13200 → 扣 39.6）
#   --actor-in-city       把人物活动边界收进城区（**仅取证用**；比赛演练不要加）
#   --round-limit=SEC     本轮运行上限（默认 3600）
#   --referee-limit=SEC   裁判任务超时（官方 600，默认 7200 便于长轮联调）
#   --gui                 自动挂 Gazebo GUI（带全套 GAZEBO_*_PATH，不会卡启动画面）
#   --no-wait             起完就返回，不等就绪
#
# 说明：默认 **不加 --actor-in-city**，即与比赛一致（组委会已明确人物边界不修）。
# ============================================================================
set -u
SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$SCRIPT_DIR" || exit 1

SEED="$(date +%m%d%H%M)"
MODEL="zzufly"
SENSORS="zzu"
ROUND_LIMIT=3600
REFEREE_LIMIT=7200
GUI=0; WAIT=1; PASS_ARGS=()

while [ $# -gt 0 ]; do
    case "$1" in
        --seed=*)          SEED="${1#*=}" ;;
        --model=*)         MODEL="${1#*=}" ;;
        --sensors=*)       SENSORS="${1#*=}" ;;
        --round-limit=*)   ROUND_LIMIT="${1#*=}" ;;
        --referee-limit=*) REFEREE_LIMIT="${1#*=}" ;;
        --gui)             GUI=1 ;;
        --no-wait)         WAIT=0 ;;
        --actor-in-city)   PASS_ARGS+=("$1") ;;          # 仅取证用
        --strict-sensors)  PASS_ARGS+=("$1") ;;
        --nan-guard)       PASS_ARGS+=("$1") ;;
        --no-cheat-monitor) PASS_ARGS+=("$1") ;;
        -h|--help)         sed -n '2,22p' "$0"; exit 0 ;;
        *) echo "未知参数：$1（-h 看用法）" >&2; exit 2 ;;
    esac
    shift
done

# 本机对外 IP（优先有线口 192.168.12.x）
IP="$(ip -4 -o addr show 2>/dev/null | awk '$2!="lo"{print $4}' | cut -d/ -f1 | grep '^192\.168\.12\.' | head -1)"
[ -z "$IP" ] && IP="$(hostname -I | awk '{print $1}')"

echo "================================================"
echo " 起一轮裁判端（双机联调）"
echo "   seed=$SEED  model=$MODEL  sensors=$SENSORS"
echo "   round-limit=${ROUND_LIMIT}s  referee-limit=${REFEREE_LIMIT}s  gui=$GUI"
echo "   人物边界：$( [ " ${PASS_ARGS[*]:-} " = *" --actor-in-city "* ] && echo '收进城区（取证用）' || echo '官方（= 比赛口径）' )"
echo "   本机 IP=$IP"
echo "================================================"

echo ">>> [1/3] 清理残留"
python3 tools/kill_stale_sim.py 2>&1 | tail -2 | sed 's/^/    /'

echo ">>> [2/3] 启动裁判端（后台）"
LOG="/tmp/round_${SEED}.log"
setsid bash start_official_judge.sh --display=headless \
    --model="$MODEL" --sensors="$SENSORS" \
    --map=random --seed="$SEED" \
    --round-limit="$ROUND_LIMIT" --referee-limit="$REFEREE_LIMIT" \
    ${PASS_ARGS[@]+"${PASS_ARGS[@]}"} > "$LOG" 2>&1 &   # 空数组必须这样展开，否则会多传一个空参数
echo "    日志：$LOG"

_start_gui() {
    local PX4=/home/wangtao/robocup_fly/PX4_Firmware; local Z="$SCRIPT_DIR"
      setsid env DISPLAY="${DISPLAY:-:1}" GAZEBO_MASTER_URI=http://localhost:11345 \
        GAZEBO_MODEL_PATH="$PX4/Tools/sitl_gazebo/models:$Z/gazebo_map_assets/models:$HOME/robocup_fly/gazebo_models:$HOME/.gazebo/models:$Z/XTDrone/sitl_config/models" \
        GAZEBO_RESOURCE_PATH="$PX4/Tools/sitl_gazebo:/usr/share/gazebo-11:$Z/gazebo_map_assets" \
        GAZEBO_PLUGIN_PATH="$Z/devel/lib:$PX4/Tools/sitl_gazebo/build:$PX4/build/px4_sitl_default/build_gazebo" \
        LD_LIBRARY_PATH="$PX4/Tools/sitl_gazebo/build:$Z/devel/lib:${LD_LIBRARY_PATH:-}" \
        gzclient > /tmp/gzclient_${SEED}.log 2>&1
}
if [ "$GUI" = "1" ] && [ "$WAIT" = "0" ]; then
    echo ">>> GUI 将在 90 秒后挂上（--no-wait 模式）"
    ( sleep 90; _start_gui ) &
fi

if [ "$WAIT" = "0" ]; then
    echo ">>> 不等就绪，已返回。看进度： tail -f $LOG"
    exit 0
fi

echo ">>> [3/3] 等待裁判端就绪（最多 8 分钟）"
for i in $(seq 1 96); do
    if grep -aq '\[4/5\] 结果' "$LOG" 2>/dev/null; then break; fi
    sleep 5
done
grep -aE '\[0c\]|机型定义|传感器规格核对|\[0d\]|费用合计|世界就绪|\[4/5\] 结果|人物边界已收进' "$LOG" 2>/dev/null | sed 's/^/    /'

# 采分器
RD="$SCRIPT_DIR/$(ls -dt logs/official_round_* 2>/dev/null | head -1)"
[ -n "$RD" ] && setsid bash tools/score_watch.sh 20 "$RD/score_watch.log" >/dev/null 2>&1 &
sleep 20
. /opt/ros/noetic/setup.bash >/dev/null 2>&1; export ROS_MASTER_URI="http://localhost:11311"
n=0; for i in 0 1 2 3 4 5; do timeout 4 rostopic echo -n1 /typhoon_h480_$i/mavros/state 2>/dev/null | grep -q 'connected: True' && n=$((n+1)); done
echo "    已连接：$n/6   人物进程：$(pgrep -c -f 'control_actor' 2>/dev/null || echo 0)"
for t in /score /left_actors; do echo -n "    $t = "; timeout 5 rostopic echo -n1 $t 2>/dev/null | head -1; done

if [ "$GUI" = "1" ]; then
    echo ">>> 就绪，现在挂 GUI（带全套 GAZEBO_*_PATH）"
    ( sleep 20; _start_gui ) &
fi

cat <<EOF

================================================
 裁判端就绪。**算法机**这样接入（先确认自己的旧栈已全停）：

  export ROS_MASTER_URI=http://$IP:11311
  export ROS_IP=<算法机自己的IP>
  source /opt/ros/noetic/setup.bash && source devel/setup.bash
  bash start_sim_only.sh --master-uri=http://$IP:11311 \\
       --ros-ip=<算法机自己的IP> --num-drones=6 --no-rviz --no-score --truth-fallback=0

 看分： rostopic echo /score       剩余目标： rostopic echo /left_actors
 采分日志： logs/official_round_*/score_watch.log
 清场： python3 tools/kill_stale_sim.py
================================================
EOF
