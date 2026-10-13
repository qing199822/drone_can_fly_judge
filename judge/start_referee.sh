#!/bin/bash
# 裁判端独立启动脚本：与仿真脚本解耦（方向 A）。
# 裁判端独占：gzserver + 6 架 typhoon_h480_zzufly + 6 个 PX4 SITL + 6 个 MAVROS
#              + actor 模型 + control_actor.py + score_cal + 可选 referee_mirror。
# 仿真端（start_sim_only.sh）只起上层算法节点，订阅裁判端发布的 UAV 话题。
#
# 用法：
#   bash start_referee.sh [--world=base0|random|<file>] [--display=gui|headless]
#                         [--drones=N] [--rovers=N] [--seed=N] [--no-mirror]
#                         [--x-min/--x-max/--y-min/--y-max] [--actor-z=Z]
#                         [--score|--no-score] [--round-limit=SEC]
#                         [--startup-mode=batch|serial] [--px4-stagger=SEC]
#                         [--settle=SEC]
#
# 默认：
#   --world=base0  --display=gui  --drones=6  --rovers=0  --no-mirror
#   --startup-mode=batch  --px4-stagger=0.5  --settle=0
#
# 流程（与 1.sh SERIAL_SWARM_START=1 严格串行链路对齐）：
#   [1] 选择地图（random 调 tools/map_gen/gen_round_world.py；fixed 复制 waypoint/world/<X>.world）
#   [2] world_clean.world 就绪性校验（tools/check_referee_world.py）
#   [3] 配置 Gazebo 环境与 PX4 / Typhoon symlink，启动 gzserver
#   [4] 串行 spawn 6 架 UAV + PX4 + MAVROS（serial_swarm_bringup.py，复用 1.sh 入口，不耦合 1.sh）
#   [5] health_report.json 验证 spawn 成功
#   [6] 启动 score_cal.py、control_actor.py 与可选 referee_mirror.py
#   [7] 等待 Ctrl-C 退出

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

WORLD="base0"
DISPLAY_MODE="${DISPLAY_MODE:-gui}"   # 默认 GUI；需要无界面运行时显式传 --display=headless
NUM_DRONES="6"
ROVERS="0"
SEED=""
WITH_MIRROR="0"
ENABLE_SCORE="1"
ROUND_LIMIT="600"
X_MIN="-50"
X_MAX="50"
Y_MIN="-50"
Y_MAX="50"
ACTOR_Z="1.25"
STARTUP_MODE="batch"
PX4_STAGGER="0.5"
SETTLE="0"
# ===== ROS 组网（裁判端 = master 所在机；算法在另一台机器上订阅）=====
# 留空 = 自动探测本机第一个非回环 IPv4；也可用 --ros-ip=192.168.x.y 显式指定。
ROS_IP_ARG=""
MASTER_PORT="11311"

# ===== PX4 树（裁判端自己起飞控，所以这里由本机决定用哪份 PX4）=====
# 默认 $HOME/PX4_Firmware；可用 ZZU_PX4_DIR 覆盖（例如指向仓库内随附的 v1.11：
#   ZZU_PX4_DIR=/home/wangtao/robocup_fly/PX4_Firmware bash start_referee.sh ...）。
# 注意：本仓库的 px4_romfs 覆盖层是 v1.11 布局（airframe 放 init.d-posix 顶层、
# 用 MPC_XY_VEL_* 旧参数名），v1.13 会把它们静默忽略 → MAVROS 永远 connected=false。
PX4_DIR="${ZZU_PX4_DIR:-$HOME/PX4_Firmware}"
PX4_BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
PX4_ROMFS_OVERLAY="$SCRIPT_DIR/px4_romfs/px4fmu_common"
export ZZU_FLY_PX4_ROMFS="$PX4_ROMFS_OVERLAY"
ACTORS_ONLY_GZ="false"   # 未来扩展位（当前为 false）

PID_FILE="$SCRIPT_DIR/logs/referee_only.pid"

while [ $# -gt 0 ]; do
    case "$1" in
        --world=*)        WORLD="${1#*=}" ;;
        --display=*)      DISPLAY_MODE="${1#*=}" ;;
        --drones=*)       NUM_DRONES="${1#*=}" ;;
        --rovers=*)       ROVERS="${1#*=}" ;;
        --seed=*)         SEED="${1#*=}" ;;
        --no-mirror)      WITH_MIRROR="0" ;;
        --score)          ENABLE_SCORE="1" ;;
        --no-score)       ENABLE_SCORE="0" ;;
        --round-limit=*)  ROUND_LIMIT="${1#*=}" ;;
        --x-min=*)        X_MIN="${1#*=}" ;;
        --x-max=*)        X_MAX="${1#*=}" ;;
        --y-min=*)        Y_MIN="${1#*=}" ;;
        --y-max=*)        Y_MAX="${1#*=}" ;;
        --actor-z=*)      ACTOR_Z="${1#*=}" ;;
        --startup-mode=*) STARTUP_MODE="${1#*=}" ;;
        --px4-stagger=*)  PX4_STAGGER="${1#*=}" ;;
        --settle=*)       SETTLE="${1#*=}" ;;
        --ros-ip=*)       ROS_IP_ARG="${1#*=}" ;;
        --master-port=*)  MASTER_PORT="${1#*=}" ;;
        *)                echo "未知参数：$1" >&2; exit 2 ;;
    esac
    shift
done

if [[ "$STARTUP_MODE" != "batch" && "$STARTUP_MODE" != "serial" ]]; then
    echo "错误：--startup-mode 必须是 batch|serial，当前：$STARTUP_MODE" >&2; exit 2
fi
if ! [[ "$NUM_DRONES" =~ ^[1-6]$ ]]; then
    echo "错误：--drones 必须是 1..6，当前：$NUM_DRONES" >&2; exit 2
fi
if [[ "$DISPLAY_MODE" != "gui" && "$DISPLAY_MODE" != "headless" ]]; then
    echo "错误：--display 必须是 gui|headless，当前：$DISPLAY_MODE" >&2; exit 2
fi

# ===== ROS 组网：让另一台机器上的算法能加入本机的 ROS 图 =====
# 必须导出 ROS_IP（而不是依赖主机名），否则另一台机器解析不到
# "wangtao-System-Product-Name" 这类本机 hostname，会连不上发布者。
if [ -z "$ROS_IP_ARG" ]; then
    ROS_IP_ARG="$(hostname -I 2>/dev/null | awk '{print $1}')"
fi
if [ -z "$ROS_IP_ARG" ]; then
    echo "错误：无法自动探测局域网 IP，请用 --ros-ip=<本机IP> 指定" >&2; exit 2
fi
if ! [[ "$MASTER_PORT" =~ ^[0-9]+$ ]]; then
    echo "错误：--master-port 必须是端口号，当前：$MASTER_PORT" >&2; exit 2
fi
export ROS_IP="$ROS_IP_ARG"
unset ROS_HOSTNAME
export ROS_MASTER_URI="http://${ROS_IP_ARG}:${MASTER_PORT}"

echo "================================================"
echo " 裁判端独立启动（含 gzserver + PX4 + MAVROS + actor + score_cal）"
echo " world=$WORLD display=$DISPLAY_MODE drones=$NUM_DRONES"
echo " rovers=$ROVERS seed='${SEED:-auto}' mirror=$([ "$WITH_MIRROR" = "1" ] && echo on || echo off) score=$([ "$ENABLE_SCORE" = "1" ] && echo on || echo off)"
echo " actor-region x=[$X_MIN,$X_MAX] y=[$Y_MIN,$Y_MAX] z=$ACTOR_Z"
echo " startup=$STARTUP_MODE px4-stagger=$PX4_STAGGER settle=$SETTLE round-limit=${ROUND_LIMIT}s"
echo " ROS_MASTER_URI=$ROS_MASTER_URI"
echo " ROS_IP=$ROS_IP"
echo "------------------------------------------------"
echo " 另一台机器（算法端）这样加入本机 ROS 图："
echo "   export ROS_MASTER_URI=$ROS_MASTER_URI"
echo "   export ROS_IP=<那台机器自己的局域网IP>"
echo "   bash start_sim_only.sh --master-uri=$ROS_MASTER_URI --num-drones=$NUM_DRONES --no-rviz"
echo " 连通自检：rostopic list | head    （应能看到 /typhoon_h480_0/... 与 /score）"
echo "================================================"

# 上一轮进程清理（不杀 gzserver / Xorg；只清 PGID）
if [ -f "$PID_FILE" ]; then
    OLD_PID="$(cat "$PID_FILE" 2>/dev/null || true)"
    if [ -n "$OLD_PID" ] && kill -0 "$OLD_PID" 2>/dev/null; then
        echo ">>> 停止上一次本脚本启动的进程组（PGID $OLD_PID）"
        kill -TERM "-$OLD_PID" 2>/dev/null || kill -TERM "$OLD_PID" 2>/dev/null || true
        sleep 2
        kill -KILL "-$OLD_PID" 2>/dev/null || kill -KILL "$OLD_PID" 2>/dev/null || true
    fi
fi
if [ "${NO_PREFLIGHT_CLEAN:-0}" != "1" ]; then
    echo ">>> 预检：清理残留仿真进程"
    python3 "$SCRIPT_DIR/tools/kill_stale_sim.py" || {
        echo "错误：仍有残留仿真进程没清掉" >&2; exit 1; }
fi

# 预检：PX4 树与 ROMFS 覆盖层必须配套。本仓库的 px4_romfs 覆盖层是 v1.11 布局
# （airframe 放 init.d-posix 顶层），配 v1.13 二进制会被静默忽略 → MAVROS 连不上。
if [ ! -x "$PX4_BUILD_DIR/bin/px4" ]; then
    echo "错误：找不到 PX4 SITL 可执行文件：$PX4_BUILD_DIR/bin/px4" >&2
    echo "      PX4_DIR=$PX4_DIR（可用 ZZU_PX4_DIR=/path/to/PX4_Firmware 指定其它树）" >&2
    exit 1
fi
if [ ! -f "$PX4_ROMFS_OVERLAY/init.d-posix/6011_typhoon_h480" ]; then
    echo "错误：ROMFS 覆盖层缺少机型文件 $PX4_ROMFS_OVERLAY/init.d-posix/6011_typhoon_h480" >&2
    exit 1
fi
if [ ! -f "$SCRIPT_DIR/XTDrone/robocup/control_actor.py" ]; then
    echo "警告：缺少 $SCRIPT_DIR/XTDrone/robocup/control_actor.py —— 6 个人物不会运动。" >&2
    echo "      获取（Gitee 官方，实测可用；约 30 秒）：" >&2
    echo "        git clone --depth 1 https://gitee.com/robin_shaun/XTDrone.git /tmp/XTDrone" >&2
    echo "        mkdir -p $SCRIPT_DIR/XTDrone && cp -r /tmp/XTDrone/robocup $SCRIPT_DIR/XTDrone/" >&2
    echo "        # 注意：不要用 --filter=blob:none --sparse，本机 git 2.25.1 会直接报错" >&2
    echo "      替代方案：robocup_referee/referee.py run --black-box ... --obstacle ...（见其 README）" >&2
fi
mkdir -p "$(dirname "$PID_FILE")"

ROUND_DIR="$SCRIPT_DIR/logs/referee_only_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$ROUND_DIR"
ACTOR_PIDS=()

# ===== [0/7] 显式先起 ROS master，不要依赖 roslaunch 自动拉起 =====
# 实测（2026-10-09）：当 ROS_MASTER_URI 指向局域网 IP 时，"让 roslaunch 自动
# 起 master"这条路径会让 gzserver 的 gazebo_ros_api_plugin 只注册日志服务、
# 不注册 spawn_sdf_model / get_model_state → 后续"等 /gazebo/get_model_state"
# 永远超时。显式先 roscore 再 roslaunch 则 30 个服务全部正常（局域网 URI 也行）。
# 这也是裁判端作为 master 所在机的正确做法：master 生命周期由裁判端掌握。
# 注意：此步骤早于后面的 PX4/Gazebo 环境加载，必须自己先把 ROS 挂上 PATH。
if [ -f /opt/ros/noetic/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
fi
if ! command -v roscore >/dev/null 2>&1; then
    echo "错误：找不到 roscore（请确认已安装 ROS Noetic：/opt/ros/noetic/setup.bash）" >&2
    exit 1
fi
MASTER_UP=false
if timeout 5 bash -c "cat < /dev/null > /dev/tcp/${ROS_IP_ARG}/${MASTER_PORT}" 2>/dev/null; then
    echo "[0/7] ROS master 已在 ${ROS_MASTER_URI} 运行（复用现有 master）"
    MASTER_UP=true
else
    echo "[0/7] 启动 ROS master：roscore -p ${MASTER_PORT}（监听 0.0.0.0，局域网可达）"
    setsid roscore -p "$MASTER_PORT" >"$ROUND_DIR/roscore.log" 2>&1 &
    for _ in $(seq 1 30); do
        if timeout 2 bash -c "cat < /dev/null > /dev/tcp/${ROS_IP_ARG}/${MASTER_PORT}" 2>/dev/null; then
            MASTER_UP=true; break
        fi
        sleep 1
    done
    if [ "$MASTER_UP" != "true" ]; then
        echo "错误：ROS master 未能在 ${ROS_MASTER_URI} 就绪（见 $ROUND_DIR/roscore.log）" >&2
        exit 1
    fi
    echo "      master 就绪：$ROS_MASTER_URI（roscore 日志 $ROUND_DIR/roscore.log）"
fi

# 链路符号链接标志（cleanup 时按创建标志删除）
MODEL_LINK_CREATED=false
AIRFRAME_LINK_CREATED=false
AIRFRAME_POST_LINK_CREATED=false

abort_round() {
    echo "错误：$1" >&2
    cleanup_all
    exit 1
}

cleanup_all() {
    # 正常跑完 round-limit 时（ZZU_KEEP_RUNNING=1）保留仿真进程，方便仿真端继续接入；
    # 只有被打断/失败才收尾清理。
    if [ "${ZZU_KEEP_RUNNING:-0}" = "1" ]; then
        echo ">>> 按设计保留仿真进程（gzserver/PX4/MAVROS/人物/裁判仍在运行）"
        echo "    要清场：python3 tools/kill_stale_sim.py"
        return 0
    fi
    local _exit_code=$?
    echo ">>> 退出清理（exit=$_exit_code）..."

    # 2026-09-26：在杀 gzserver 之前，先尝试取最后一个 /score 值并把 score_cal 优雅停下。
    # score_cal 的 atexit + _dump_score_file 会在 score_cal 收到 SIGTERM 后自然退出
    # 时把最终分写到 SCORE_OUTPUT_PATH（start_referee.sh 已经把它指向本端 round dir）。
    # 这里我们再额外把"退出时刻 + 最终 score + left_actors"写到 score.txt，带时间戳，
    # 保证 Ctrl-C 后 cat score.txt 一定能拿到最终分。
    if [ -n "${SCORE_PID:-}" ] && kill -0 "$SCORE_PID" 2>/dev/null; then
        FINAL_SCORE=""
        FINAL_LEFT=""
        TIMESTAMP="$(date '+%Y-%m-%d %H:%M:%S %z')"
        if [ -n "${SCORE_OUTPUT_PATH:-}" ] && [ -f "${SCORE_OUTPUT_PATH}" ]; then
            FINAL_SCORE="$(cat "${SCORE_OUTPUT_PATH}" 2>/dev/null | head -1 | tr -dc '0-9-' )"
        fi
        if [ -z "$FINAL_SCORE" ]; then
            FINAL_SCORE="$(timeout 3 bash -c 'export ROS_MASTER_URI='"${ROS_MASTER_URI:-http://localhost:11311}"'; rostopic echo -n 1 /score' 2>/dev/null \
                | awk '/data:/{print \$2}' | tr -dc '0-9-')"
        fi
        if [ -z "$FINAL_LEFT" ]; then
            FINAL_LEFT="$(timeout 3 bash -c 'export ROS_MASTER_URI='"${ROS_MASTER_URI:-http://localhost:11311}"'; rostopic echo -n 1 /left_actors' 2>/dev/null \
                | sed -n 's/^data: \"\\(.*\\)\"/\\1/p')"
        fi
        if [ -n "$FINAL_SCORE" ]; then
            {
                echo "# exit at ${TIMESTAMP}"
                echo "score: ${FINAL_SCORE}"
                [ -n "$FINAL_LEFT" ] && echo "left_actors: ${FINAL_LEFT}"
            } >"${SCORE_OUTPUT_PATH:-$ROUND_DIR/score.txt}"
            echo "  最终 score=${FINAL_SCORE} 已写入 ${SCORE_OUTPUT_PATH:-$ROUND_DIR/score.txt}"
        fi
    fi

    # score_cal 先 SIGTERM，让 atexit 自然跑 _dump_score_file（可能更新一次）。
    if [ -n "${SCORE_PID:-}" ] && kill -0 "$SCORE_PID" 2>/dev/null; then
        kill -TERM "$SCORE_PID" 2>/dev/null || true
        for _ in 1 2 3 4 5 6 7 8; do
            kill -0 "$SCORE_PID" 2>/dev/null || break
            sleep 0.3
        done
        kill -KILL "$SCORE_PID" 2>/dev/null || true
    fi
    if [ -n "${MIRROR_PID:-}" ] && kill -0 "$MIRROR_PID" 2>/dev/null; then
        kill -TERM "$MIRROR_PID" 2>/dev/null || true
    fi
    for ACTOR_PID in "${ACTOR_PIDS[@]}"; do
        kill -TERM "$ACTOR_PID" 2>/dev/null || true
    done
    if [ -n "${GZSERVER_PID:-}" ] && kill -0 "$GZSERVER_PID" 2>/dev/null; then
        kill -INT "$GZSERVER_PID" 2>/dev/null || true
    fi
    sleep 2
    for HPID in "${BRINGUP_HELPER_PIDS[@]:-}"; do
        [ -n "$HPID" ] && kill -KILL -- "-$HPID" 2>/dev/null || kill -KILL "$HPID" 2>/dev/null || true
    done
    [ -n "${SCORE_PID:-}" ] && kill -KILL -- "-$SCORE_PID" 2>/dev/null || kill -KILL "$SCORE_PID" 2>/dev/null || true
    [ -n "${MIRROR_PID:-}" ] && kill -KILL -- "-$MIRROR_PID" 2>/dev/null || kill -KILL "$MIRROR_PID" 2>/dev/null || true
    for ACTOR_PID in "${ACTOR_PIDS[@]}"; do
        kill -KILL -- "-$ACTOR_PID" 2>/dev/null || kill -KILL "$ACTOR_PID" 2>/dev/null || true
    done
    [ -n "${GZSERVER_PID:-}" ] && kill -KILL -- "-$GZSERVER_PID" 2>/dev/null || kill -KILL "$GZSERVER_PID" 2>/dev/null || true
    # 仅在 cleanup 退出码非零时删除由本脚本创建的符号链接
    [ "${MODEL_LINK_CREATED}" = "true" ] && rm -f "$PX4_DIR/Tools/sitl_gazebo/models/typhoon_h480_zzufly" 2>/dev/null || true
    [ "${AIRFRAME_LINK_CREATED}" = "true" ] && rm -f "$AIRFRAME_DIR/6011_typhoon_h480_zzufly" 2>/dev/null || true
    [ "${AIRFRAME_POST_LINK_CREATED}" = "true" ] && rm -f "$AIRFRAME_DIR/6011_typhoon_h480_zzufly.post" 2>/dev/null || true
    rm -f "$PID_FILE"
    return $_exit_code
}

trap 'cleanup_all; exit 0' INT TERM EXIT

# ----------------------------------------------------------------------
# [1/7] 地图选择
# ----------------------------------------------------------------------
WORLD_FOR_SIM=""
if [ "$WORLD" = "random" ]; then
    echo "[1/7] 随机地图生成（最多 8 次尝试）..."
    GENERATED="$ROUND_DIR/generated"
    mkdir -p "$GENERATED"
    OK=1
    GEN_ARGS=(--out "$GENERATED" --rovers "$ROVERS")
    [ -n "$SEED" ] && GEN_ARGS+=(--seed "$SEED")
    for attempt in 1 2 3 4 5 6 7 8; do
        if python3 "$SCRIPT_DIR/tools/map_gen/gen_round_world.py" "${GEN_ARGS[@]}" \
                >"$GENERATED/gen_$attempt.log" 2>&1; then
            WORLD_FOR_SIM="$GENERATED/robocup.world"
            BLACK_BOX="$GENERATED/black_box.txt"
            if python3 "$SCRIPT_DIR/tools/check_referee_world.py" "$WORLD_FOR_SIM" "$BLACK_BOX" \
                    >"$ROUND_DIR/world_check.log" 2>&1; then
                cp -f "$WORLD_FOR_SIM" "$ROUND_DIR/world_clean.world"
                OK=1
                echo "  采用第 $attempt 次尝试"
                break
            else
                echo "  world_check 失败，重试"
                OK=0
            fi
        else
            echo "  gen_round_world 失败，重试"
            OK=0
        fi
    done
    [ "$OK" = "1" ] || abort_round "随机地图 8 次都未通过检查"
    echo "$WORLD_FOR_SIM" >"$ROUND_DIR/world_path.txt"
elif [ -f "waypoint/world/${WORLD}.world" ]; then
    echo "[1/7] 使用固定地图 waypoint/world/${WORLD}.world"
    WORLD_FOR_SIM="waypoint/world/${WORLD}.world"
elif [ -f "$WORLD" ]; then
    echo "[1/7] 使用用户指定地图 $WORLD"
    WORLD_FOR_SIM="$WORLD"
else
    abort_round "找不到地图：$WORLD（既不是 random/base0..16，也不是有效路径）"
fi

# 固定图与用户图：复制并预先随机化 actor_0..5（重试 5 次，随机 seed 避开撞墙）
if [ "$WORLD" != "random" ]; then
    echo "[1b/7] 复制并预先随机化 actor_0..5..."
    RANDOMIZED_WORLD="$ROUND_DIR/world_randomized.world"
    ACTOR_OK=0
    for try in 1 2 3 4 5 6 7 8; do
        if [ -z "$SEED" ]; then
            RAND_SEED=$((RANDOM * 100000 + RANDOM))
        else
            RAND_SEED=$((SEED + try - 1))
        fi
        RAND_ARGS=(--source "$WORLD_FOR_SIM" --dest "$RANDOMIZED_WORLD" \
            --x-min "$X_MIN" --x-max "$X_MAX" --y-min "$Y_MIN" --y-max "$Y_MAX" \
            --actor-z "$ACTOR_Z" --seed "$RAND_SEED")
        [ -f "$(dirname "$WORLD_FOR_SIM")/black_box.txt" ] && RAND_ARGS+=(--black-box "$(dirname "$WORLD_FOR_SIM")/black_box.txt")
        if python3 "$SCRIPT_DIR/tools/randomize_world_actors.py" "${RAND_ARGS[@]}" \
                >"$ROUND_DIR/actor_randomize_$try.log" 2>&1; then
            # 落点必须同时过 check_referee_world 才算成功。
            # 只判"生成成功"的话，人物偶尔会落进建筑，[2/7] 就会直接中止整轮
            # （实测每几轮就撞一次）。这里换种子自动重试。
            if python3 "$SCRIPT_DIR/tools/check_referee_world.py" "$RANDOMIZED_WORLD" \
                    >"$ROUND_DIR/world_check_$try.log" 2>&1; then
                ACTOR_OK=1
                echo "  actor randomize 第 $try 次成功（seed=$RAND_SEED）" | tee -a "$ROUND_DIR/actor_randomize.log"
                cp -f "$ROUND_DIR/actor_randomize_$try.log" "$ROUND_DIR/actor_randomize.log"
                cp -f "$ROUND_DIR/world_check_$try.log" "$ROUND_DIR/world_check.log"
                break
            else
                echo "  actor randomize 第 $try 次落点未过校验（seed=$RAND_SEED），换种子重试"
                grep -aE '^  FAIL|^结果' "$ROUND_DIR/world_check_$try.log" 2>/dev/null | head -2 | sed 's/^/      /'
            fi
        else
            echo "  actor randomize 第 $try 次撞墙（seed=$RAND_SEED），重试"
        fi
    done
    [ "$ACTOR_OK" = "1" ] || abort_round "actor 随机化 + 落点校验 8 次都失败（见 $ROUND_DIR/actor_randomize_*.log / world_check_*.log）"
    WORLD_FOR_SIM="$RANDOMIZED_WORLD"
fi

# ----------------------------------------------------------------------
# [2/7] world_clean.world 就绪性校验
# ----------------------------------------------------------------------
echo "[2/7] 地图就绪性校验..."
python3 "$SCRIPT_DIR/tools/check_referee_world.py" "$WORLD_FOR_SIM" \
    >"$ROUND_DIR/world_check.log" 2>&1 || abort_round "world 未通过 check_referee_world（见 $ROUND_DIR/world_check.log）"
cp -f "$WORLD_FOR_SIM" "$ROUND_DIR/world_clean.world"
echo "  world_clean.world 检查通过：$(grep -aE '通过' "$ROUND_DIR/world_check.log" | head -1)"

# control_actor.py 从工作目录读取这两个文件；必须与本轮 world 对应。
CONTROL_MAP_ARGS=(--world "$ROUND_DIR/world_clean.world" --out "$ROUND_DIR")
if [ "$WORLD" = "random" ]; then
    CONTROL_MAP_ARGS+=(--black-box "$GENERATED/black_box.txt")
elif [ -f "$(dirname "$WORLD_FOR_SIM")/black_box.txt" ]; then
    CONTROL_MAP_ARGS+=(--black-box "$(dirname "$WORLD_FOR_SIM")/black_box.txt")
fi
python3 "$SCRIPT_DIR/tools/prepare_actor_control_map.py" "${CONTROL_MAP_ARGS[@]}" \
    >"$ROUND_DIR/actor_map.log" 2>&1 || abort_round "人物控制地图准备失败（见 $ROUND_DIR/actor_map.log）"

# ----------------------------------------------------------------------
# [3/7] Gazebo 环境 + PX4 / Typhoon symlink + 启动 gzserver
# ----------------------------------------------------------------------
echo "[3/7] 配置 Gazebo 环境与 PX4 / Typhoon 符号链接..."
# 1.sh:334-358 三件套：source ROS + devel + PX4 Gazebo + ROS_PACKAGE_PATH + 地理数据
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
fi
source "$SCRIPT_DIR/devel/setup.bash" 2>/dev/null || true
source "$PX4_DIR/Tools/setup_gazebo.bash" "$PX4_DIR" "$PX4_BUILD_DIR"
export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:+${ROS_PACKAGE_PATH}:}$PX4_DIR:$PX4_DIR/Tools/sitl_gazebo"
# ROS 日志目录兜底（与 start_sim_only.sh 一致）：
#   沙盒/容器里 $HOME/.ros/log 常常是只读挂载，rosmaster/roslaunch 建日志目录失败会
#   直接自杀（表现为 "gzserver 进程已退出"）。这里检测不可写就改用工作区内目录。
if [ -z "${ROS_LOG_DIR:-}" ]; then
    ROS_LOG_BASE="$HOME/.ros/log"
    if ! mkdir -p "$ROS_LOG_BASE" 2>/dev/null || [ ! -w "$ROS_LOG_BASE" ]; then
        export ROS_LOG_DIR="$SCRIPT_DIR/logs/ros"
        mkdir -p "$ROS_LOG_DIR"
        echo "提示：$ROS_LOG_BASE 不可写，ROS 日志改用 $ROS_LOG_DIR"
    fi
fi

export GAZEBO_MODEL_DATABASE_URI=""
# 组装 GAZEBO_MODEL_PATH：只加入“真实存在”的目录。
# 历史 bug（2026-10-09 定位）：这里原来写的是 $HOME/gazebo_models，而本机模型真实
# 位置是 $HOME/robocup_fly/gazebo_models → cardboard_box / cessna / casual_female
# 等 model:// 解析不到 → gzserver 转去连 Gazebo 在线模型库（实测 SYN-SENT 到
# 54.208.189.95:80）并永久阻塞 → gazebo_ros_api_plugin 不注册 spawn_sdf_model /
# get_model_state → 整轮卡在“等待 Gazebo 服务”直到超时。
_GAZEBO_MODEL_DIRS=(
    "$SCRIPT_DIR/gazebo_map_assets/models"
    "$SCRIPT_DIR/XTDrone/sitl_config/models"
    "$SCRIPT_DIR/XTDrone/sitl_config/gazebo_plugin/citysim/models"
    "$HOME/robocup_fly/gazebo_models"
    "$HOME/gazebo_models"
    "$HOME/.gazebo/models"
    "$HOME/XTDrone/sitl_config/models"
    "$HOME/XTDrone/sitl_config/gazebo_plugin/citysim/models"
)
GAZEBO_MODEL_PATH_NEW=""
for _d in "${_GAZEBO_MODEL_DIRS[@]}"; do
    if [ -d "$_d" ]; then
        GAZEBO_MODEL_PATH_NEW="${GAZEBO_MODEL_PATH_NEW:+$GAZEBO_MODEL_PATH_NEW:}$_d"
    fi
done
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH_NEW}${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
# 防回归：确认世界用到的三个模型能被本地解析（找不到就会去联网并卡死）
for _m in cardboard_box cessna casual_female; do
    if ! find ${GAZEBO_MODEL_PATH//:/ } -maxdepth 1 -name "$_m" -print -quit 2>/dev/null | grep -q .; then
        echo "警告：GAZEBO_MODEL_PATH 里找不到模型 '$_m'，gzserver 可能卡在联网下载。" >&2
        echo "      当前 GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH" >&2
    fi
done
export GAZEBO_PLUGIN_PATH="$SCRIPT_DIR/devel/lib:$PX4_DIR/Tools/sitl_gazebo/build:$SCRIPT_DIR/gazebo_map_assets/plugins:$PX4_DIR/build/px4_sitl_default/build_gazebo${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
# 关键：v1.11 的 Gazebo 插件构建在 Tools/sitl_gazebo/build/，而
# build/px4_sitl_default/build_gazebo/ 是空的。少了前者，
# libgazebo_mavlink_interface.so 加载失败 → PX4 永远卡在
# "Waiting for simulator to accept connection on TCP port 4560" →
# 6 架全部 connected=false。
export LD_LIBRARY_PATH="$PX4_DIR/Tools/sitl_gazebo/build:${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
if [ -r "$SCRIPT_DIR/geographiclib/geoids/egm96-5.pgm" ]; then
    export GEOGRAPHICLIB_DATA="$SCRIPT_DIR/geographiclib"
fi

# 1.sh:361-401 Typhoon 模型 / 机型符号链接
MODEL_LINK="$PX4_DIR/Tools/sitl_gazebo/models/typhoon_h480_zzufly"
if [ -L "$MODEL_LINK" ] && [ ! -e "$MODEL_LINK" ]; then
    abort_round "PX4 自定义模型路径是损坏的符号链接：$MODEL_LINK"
elif [ ! -e "$MODEL_LINK" ]; then
    if [ ! -w "$(dirname "$MODEL_LINK")" ]; then
        abort_round "无权创建 PX4 自定义模型链接：$(dirname "$MODEL_LINK")"
    fi
    ln -s "$SCRIPT_DIR/typhoon_h480_zzufly" "$MODEL_LINK"
    MODEL_LINK_CREATED=true
elif [ ! -d "$MODEL_LINK" ] || [ ! -f "$MODEL_LINK/typhoon_h480_zzufly.sdf" ]; then
    abort_round "PX4 自定义模型路径无效或缺少 typhoon_h480_zzufly.sdf：$MODEL_LINK"
fi

AIRFRAME_DIR="$PX4_ROMFS_OVERLAY/init.d-posix"
AIRFRAME_LINK="$AIRFRAME_DIR/6011_typhoon_h480_zzufly"
AIRFRAME_POST_LINK="$AIRFRAME_LINK.post"
if [ ! -e "$AIRFRAME_LINK" ]; then
    if [ ! -w "$AIRFRAME_DIR" ]; then
        abort_round "无权在 $AIRFRAME_DIR 创建机型别名"
    fi
    ln -s "$AIRFRAME_DIR/6011_typhoon_h480" "$AIRFRAME_LINK"
    AIRFRAME_LINK_CREATED=true
elif [ ! "$AIRFRAME_LINK" -ef "$AIRFRAME_DIR/6011_typhoon_h480" ]; then
    abort_round "PX4 六旋翼机型别名指向了其他文件：$AIRFRAME_LINK"
fi
if [ ! -e "$AIRFRAME_POST_LINK" ]; then
    if [ ! -w "$AIRFRAME_DIR" ]; then
        abort_round "无权在 $AIRFRAME_DIR 创建机型别名 post"
    fi
    ln -s "$AIRFRAME_DIR/6011_typhoon_h480.post" "$AIRFRAME_POST_LINK"
    AIRFRAME_POST_LINK_CREATED=true
fi

echo "[3b/7] 启动 gzserver（$DISPLAY_MODE）..."
GUI_FLAG="false"
[ "$DISPLAY_MODE" = "gui" ] && GUI_FLAG="true"
# PYTHONUNBUFFERED=1：roslaunch 是 Python，默认块缓冲会把输出憋在管道里
# （gzserver.log 长时间 0 字节，卡住时看不到任何线索）。
PYTHONUNBUFFERED=1 setsid roslaunch "$SCRIPT_DIR/robocup_zzufly_gazebo.launch" \
    world:="$ROUND_DIR/world_clean.world" gui:="$GUI_FLAG" \
    >"$ROUND_DIR/gzserver.log" 2>&1 &
GZSERVER_PID=$!
echo "  gzserver PID=$GZSERVER_PID"

echo "  等待 /gazebo/get_model_state 服务（最长 240s）..."
WAITED=0
while [ "$WAITED" -lt 240 ]; do
    if ! kill -0 "$GZSERVER_PID" 2>/dev/null; then
        abort_round "gzserver 进程已退出，见 $ROUND_DIR/gzserver.log"
    fi
    if rosservice list 2>/dev/null | grep -qx '/gazebo/get_model_state'; then
        echo "  /gazebo/get_model_state 已就绪"
        break
    fi
    # 每 20s 回显一次 Gazebo 侧输出，卡住时能直接看到原因
    if [ $((WAITED % 20)) -eq 0 ] && [ "$WAITED" -gt 0 ]; then
        echo "  [等待中 ${WAITED}s] gazebo 侧最近输出："
        tail -6 "$ROUND_DIR/gzserver.log" 2>/dev/null | sed 's/^/      /'
    fi
    sleep 2
    WAITED=$((WAITED + 2))
done
if ! rosservice list 2>/dev/null | grep -qx '/gazebo/get_model_state'; then
    abort_round "Gazebo 启动失败或 240s 内未暴露 /gazebo/get_model_state"
fi

# ----------------------------------------------------------------------
# [4/7] 串行 spawn 6 架 typhoon + PX4 + MAVROS（复用 1.sh 的编排器入口）
# ----------------------------------------------------------------------
echo "[4/7] 串行 spawn $NUM_DRONES 架 typhoon_h480 + PX4 + MAVROS（mode=$STARTUP_MODE）..."
BRINGUP_HELPER_PIDS=()
SERIAL_LOG="$ROUND_DIR/serial_bringup.log"
python3 -u "$SCRIPT_DIR/src/xtdrone_mavros_bridge/scripts/serial_swarm_bringup.py" \
    --num-drones "$NUM_DRONES" --start-index 0 \
    --no-stereo-right \
    --no-bridge \
    --mode "$STARTUP_MODE" --px4-stagger "$PX4_STAGGER" \
    --settle "$SETTLE" \
    >"$SERIAL_LOG" 2>&1 || abort_round "spawn/PX4/MAVROS 串行编排失败（见 $SERIAL_LOG 与 $SCRIPT_DIR/logs/serial_bringup/health_report.json）"

HEALTH_REPORT="$SCRIPT_DIR/logs/serial_bringup/health_report.json"
if [ -f "$HEALTH_REPORT" ]; then
    cp -f "$HEALTH_REPORT" "$ROUND_DIR/health_report.json"
    if python3 -c "import json,sys; d=json.load(open('$HEALTH_REPORT')); sys.exit(1 if d.get('failures') else 0)" 2>/dev/null; then
        echo "  serial_bringup health_report 无 failures"
    else
        abort_round "serial_bringup health_report 有 failures（见 $ROUND_DIR/health_report.json）"
    fi
else
    echo "  警告：$HEALTH_REPORT 不存在，跳过健康报告校验"
fi

# 健康门：每架 mavros state connected
echo "[4b/7] 等待所有 typhoon_h480_<i>/mavros/state connected（最长 240s）..."
WAITED=0
ALL_OK=0
while [ "$WAITED" -lt 240 ]; do
    CONNECTED_COUNT=0
    for ID in $(seq 0 $((NUM_DRONES - 1))); do
        if timeout 2 rostopic echo -n 1 "/typhoon_h480_${ID}/mavros/state" 2>/dev/null \
            | grep -q 'connected: True'; then
            CONNECTED_COUNT=$((CONNECTED_COUNT + 1))
        fi
    done
    if [ "$CONNECTED_COUNT" -ge "$NUM_DRONES" ]; then
        ALL_OK=1
        echo "  ${NUM_DRONES}/${NUM_DRONES} 架 MAVROS connected"
        break
    fi
    sleep 5
    WAITED=$((WAITED + 5))
done
if [ "$ALL_OK" != "1" ]; then
    abort_round "${CONNECTED_COUNT}/${NUM_DRONES} 架 MAVROS 在 240s 内未 connected"
fi

# ----------------------------------------------------------------------
# [6/7] score_cal + control_actor.py + 可选 referee_mirror
# ----------------------------------------------------------------------
UAV_TYPE="typhoon_h480"
SCORE_PID=""
MIRROR_PID=""
if [ "$ENABLE_SCORE" = "1" ]; then
    echo "[6/7] 启动 score_cal.py（UAV_TYPE=$UAV_TYPE）..."
    export SCORE_OUTPUT_PATH="$ROUND_DIR/score.txt"
    setsid python3 -u "$SCRIPT_DIR/score_cal.py" "$UAV_TYPE" \
        >"$ROUND_DIR/referee.log" 2>&1 &
    SCORE_PID=$!
    echo "  score_cal PID=$SCORE_PID  score.txt=$ROUND_DIR/score.txt"
fi

echo "[6b/7] 启动 6 个 control_actor.py 人物运动节点..."
for ID in 0 1 2 3 4 5; do
    (cd "$ROUND_DIR" && exec setsid python3 -u "$SCRIPT_DIR/XTDrone/robocup/control_actor.py" "$ID") \
        >"$ROUND_DIR/actor_${ID}.log" 2>&1 &
    ACTOR_PIDS+=("$!")
done
sleep 2
for ID in 0 1 2 3 4 5; do
    if ! kill -0 "${ACTOR_PIDS[$ID]}" 2>/dev/null; then
        abort_round "actor_$ID 控制进程启动失败（见 $ROUND_DIR/actor_${ID}.log）"
    fi
done

if [ "$WITH_MIRROR" = "1" ]; then
    echo "  启 referee_mirror.py（裁判镜像）"
    setsid python3 -u "$SCRIPT_DIR/src/yolo/referee_mirror.py" \
        "$UAV_TYPE" "$NUM_DRONES" "$NUM_DRONES" \
        >"$ROUND_DIR/mirror.log" 2>&1 &
    MIRROR_PID=$!
    echo "  referee_mirror PID=$MIRROR_PID"
fi

echo "$$" > "$PID_FILE"

# ----------------------------------------------------------------------
# [7/7] 进入等待
# ----------------------------------------------------------------------
echo "[7/7] 裁判端就绪；仿真端可启动"
echo "  round dir: $ROUND_DIR"
echo "  score:     $ROUND_DIR/score.txt（裁判端独占）"
echo "  终端 B: bash start_sim_only.sh"

# 注意：本脚本的子进程都用 setsid 起了独立会话，$GZSERVER_PID 那个 setsid
# 进程 fork 完就退出了 —— 直接 `wait $GZSERVER_PID` 会立刻返回，脚本随即走
# 收尾清理，把刚起好的 gzserver/PX4/MAVROS/人物/裁判全部杀掉（表现为：
# 日志刚打印「[7/7] 裁判端就绪」就立刻「退出清理」，仿真一秒都没留下）。
# 所以这里改成显式按 round-limit 倒计时，并在正常结束时保留进程。
WAITED=0
while [ "$WAITED" -lt "$ROUND_LIMIT" ]; do
    if ! pgrep -f 'gzserver -e od[e]' >/dev/null 2>&1; then
        echo "警告：gzserver 已不在运行，提前结束等待" >&2
        break
    fi
    # 每 60s 回显一次分数，方便盯盘
    if [ $((WAITED % 60)) -eq 0 ] && [ "$WAITED" -gt 0 ]; then
        echo "  [${WAITED}s/${ROUND_LIMIT}s] score=$(timeout 3 rostopic echo -n1 /score 2>/dev/null | head -1 | awk '{print $2}')"
    fi
    sleep 5; WAITED=$((WAITED + 5))
done
echo ">>> 等待结束（round-limit=${ROUND_LIMIT}s，已等 ${WAITED}s）"
echo "    进程按设计保留：清场用 python3 tools/kill_stale_sim.py"
ZZU_KEEP_RUNNING=1
exit 0
