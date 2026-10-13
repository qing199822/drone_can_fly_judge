#!/bin/bash
# 一键启动 N 机 Gazebo + auto_explore_swarm
#
# 环境变量：
#   WORLD=base0|base5|/path/xxx.world   地图（默认 base0）
#   DISPLAY_MODE=gui|headless           是否显示 Gazebo（默认 gui）
#   NUM_DRONES=6                        无人机数量（默认 6）
#   SKIP_CLEANUP=1                      跳过清理（由调用方负责；start_referee_round.sh 用）
#   CLEAN_ALL=1                         用 kill_sim.sh 彻底清场（会杀所有仿真相关进程）
#
# 清理策略：默认只停止"上一次由本脚本启动"的进程组（PID 文件 logs/6g_auto_explore.pid），
# 不再使用宽泛的 pkill -f（避免误杀其它终端里的 tracking / 坐标节点 / 裁判进程）。

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

WORLD="${WORLD:-base0}"
DISPLAY_MODE="${DISPLAY_MODE:-gui}"
NUM_DRONES="${NUM_DRONES:-6}"
PID_FILE="$SCRIPT_DIR/logs/6g_auto_explore.pid"

echo "=============================================="
echo " 一键启动 $NUM_DRONES 机 Gazebo（$DISPLAY_MODE）+ 自主探索"
echo " 地图: $WORLD"
echo "=============================================="

if [ "${SKIP_CLEANUP:-0}" = "1" ]; then
    echo ">>> SKIP_CLEANUP=1：跳过旧进程清理（由调用方负责）"
elif [ "${CLEAN_ALL:-0}" = "1" ]; then
    echo ">>> CLEAN_ALL=1：调用 kill_sim.sh 彻底清理"
    bash "$SCRIPT_DIR/kill_sim.sh" >/dev/null 2>&1 || true
elif [ -f "$PID_FILE" ]; then
    OLD_PID="$(cat "$PID_FILE" 2>/dev/null || true)"
    if [ -n "$OLD_PID" ] && kill -0 "$OLD_PID" 2>/dev/null; then
        echo ">>> 停止上一次本脚本启动的进程组（PGID $OLD_PID）"
        kill -TERM "-$OLD_PID" 2>/dev/null || kill -TERM "$OLD_PID" 2>/dev/null || true
        sleep 2
        kill -KILL "-$OLD_PID" 2>/dev/null || kill -KILL "$OLD_PID" 2>/dev/null || true
    else
        echo ">>> PID 文件存在但进程已不在（$PID_FILE）"
    fi
else
    echo ">>> 未发现上一次的 PID 文件；如需彻底清场请用 CLEAN_ALL=1 或 bash kill_sim.sh"
fi

# 不管走哪条分支，都再扫一遍"我们的残留仿真进程"（典型是上一轮被打断后活下来的 PX4 daemon：
# 六架新 PX4 会全部秒退 "PX4 daemon already running"，MAVROS 永远连不上，白等 120s）。
# 只按我们自己的特征匹配，不用宽泛 pkill；详见 tools/kill_stale_sim.py。
if [ "${NO_PREFLIGHT_CLEAN:-0}" != "1" ]; then
    echo ">>> 预检：清理残留仿真进程（PX4 daemon / gzserver / rosmaster ...）"
    python3 "$SCRIPT_DIR/tools/kill_stale_sim.py" || {
        echo "错误：仍有残留仿真进程没清掉，拒绝启动（可先 bash kill_sim.sh）" >&2
        exit 1
    }
else
    echo ">>> NO_PREFLIGHT_CLEAN=1：跳过残留进程预检"
fi
mkdir -p "$(dirname "$PID_FILE")"

# ---- 崩溃现场保留（2026-09-17 加）------------------------------------------
# 目的：gzserver 以 SIGABRT/SEGV 死掉时，一定要留下"原生"证据，而不是只剩 roslaunch 摘要。
#   1) 允许写 core（若系统 core_pattern 允许）；
#   2) GZ_DEBUG=1 时把 /usr/bin/gzserver 包在 gdb 里（PATH 注入），崩溃瞬间打印所有线程 backtrace；
#   3) 模拟结束后把 ~/.gazebo/server-*/default.log 与本次 ~/.ros/log/<run_id>/ 归档到 logs/gz_debug/。
ulimit -c unlimited 2>/dev/null || true
DEBUG_ARCHIVE=""
if [ "${GZ_DEBUG:-0}" = "1" ]; then
    export PATH="$SCRIPT_DIR/tools/gz_debug:$PATH"
    export GZ_DEBUG_LOG="${GZ_DEBUG_LOG:-$SCRIPT_DIR/logs/gz_debug/$(date +%Y%m%d_%H%M%S)_gzserver_gdb.log}"
    mkdir -p "$(dirname "$GZ_DEBUG_LOG")"
    echo ">>> GZ_DEBUG=1：gzserver 跑在 gdb 下，崩溃时打印 backtrace → $GZ_DEBUG_LOG"
fi

echo ">>> 启动 Gazebo 仿真（$NUM_DRONES 机, $DISPLAY_MODE, 地图 $WORLD）..."
export AUTO_EXPLORE_SWARM=1
bash 1.sh "$NUM_DRONES" mission_base0_all_roads.json sim "$DISPLAY_MODE" "$WORLD" &
SIM_PID=$!
echo "$$" > "$PID_FILE"

echo ">>> 等待仿真就绪（最长 180 秒，日志见 1.sh 输出）..."
READY=false
for _ in $(seq 1 90); do
    if rosnode list 2>/dev/null | grep -q '/gazebo' && \
       rosnode list 2>/dev/null | grep -q '/typhoon_h480_0/mavros'; then
        READY=true
        break
    fi
    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        echo "错误：仿真进程已退出（可能是 gzserver 段错误）。" >&2
        exit 1
    fi
    sleep 2
done

if [ "$READY" != true ]; then
    echo "错误：仿真未在预期时间内就绪，请检查 Gazebo 日志。" >&2
    kill "$SIM_PID" 2>/dev/null || true
    exit 1
fi

echo ">>> 仿真已就绪，auto_explore_swarm 已由 1.sh 自动启动。"
echo "    仿真 PID: $SIM_PID（PID 文件：$PID_FILE）"
echo "    按 Ctrl-C 结束本脚本（不会自动杀仿真；清理用 CLEAN_ALL=1 或 bash kill_sim.sh）"

wait "$SIM_PID" || true

# ---- 归档 Gazebo/ROS 原生日志（不论是否崩溃）--------------------------------
DEBUG_ARCHIVE="$SCRIPT_DIR/logs/gz_debug/$(date +%Y%m%d_%H%M%S)_native"
mkdir -p "$DEBUG_ARCHIVE"
[ -n "${GZ_DEBUG_LOG:-}" ] && [ -f "$GZ_DEBUG_LOG" ] && cp -f "$GZ_DEBUG_LOG" "$DEBUG_ARCHIVE/gzserver_gdb.log" 2>/dev/null || true
for f in "$HOME"/.gazebo/server-*/default.log "$HOME"/.gazebo/server-*/gzserver.log "$HOME"/.gazebo/ogre.log; do
    [ -f "$f" ] || continue
    cp -f "$f" "$DEBUG_ARCHIVE/$(basename "$(dirname "$f")")_$(basename "$f")" 2>/dev/null || true
done
RUN_ID_DIR="$(ls -td "$HOME"/.ros/log/*/ 2>/dev/null | head -1)"
if [ -n "$RUN_ID_DIR" ]; then
    cp -r "$RUN_ID_DIR" "$DEBUG_ARCHIVE/ros_log_$(basename "$RUN_ID_DIR")" 2>/dev/null || true
fi
echo ">>> Gazebo/ROS 原生日志已归档：$DEBUG_ARCHIVE"
ls -l "$DEBUG_ARCHIVE" 2>/dev/null | head -10
