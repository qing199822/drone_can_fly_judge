#!/bin/bash
# 仿真端独立启动脚本：与裁判脚本完全解耦（方向 A）。
# 不启 Gazebo / PX4 SITL / MAVROS；只起上层飞行算法节点（bbox2coord / area_mission / road_follow /
# tracking / yolo 等），由裁判端在另一终端（bash start_referee.sh）发布 mavros pose / depth
# / actor_info / model_states。
#
# 用法：
#   bash start_sim_only.sh [--master-uri=http://host:11311] [--rviz|--no-rviz]
#                          [--num-drones=N] [--score|--no-score]
#                          [--truth-fallback=0|1]
#
# 默认：
#   --master-uri=http://localhost:11311   （与裁判端共用本机 master）
#   --no-rviz                              （仿真端默认不起 rviz，避免与裁判端 GUI 争 X）
#   --no-score                             （裁判端独占计分，本端不重复发布 /score）
#   --num-drones=6                         （与裁判端默认对齐）
#   --truth-fallback=1                     （multi_solving.sh 的 TRUTH_FALLBACK 默认）
#
# 启动顺序：先在终端 A 跑 bash start_referee.sh，本端在终端 B 启动。
# 官方 XTDrone 模式：官方必须先启动 px4/robocup.launch（Gazebo + PX4 + MAVROS）。
# 本脚本不加载模型、不启动 PX4；它只启动算法节点并检查官方 MAVROS 接口。
#
#   * 启动顺序：static_tf / pose / tf_tree / target_lookup_service / spawn_mux_swarm
#                → auto_explore_swarm
#                → wait_for_upper_stack_ready（per-drone 检查）
#                → multi_yolo_detecting_gpu / multi_solving / multi_gimbal_control / tracking
#   * 仿真端不启 multi_yolo_detecting*(GPU 之外的版本) / spawn_mux_swarm 是上层控制链必需，
#     必须启动；multi_yolo_detecting_gpu 订阅裁判端 realsense 图像，发布 bounding_boxes。

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
WITH_RVIZ="0"
ENABLE_SCORE="0"
NUM_DRONES="6"
TRUTH_FALLBACK="1"
CHECK_ONLY="0"
# ===== ROS 组网（算法端）=====
# 本机发布的话题（/actor_*_info、/score 等）要被裁判端订阅，必须导出 ROS_IP，
# 否则裁判端解析不到本机 hostname。留空 = 自动探测第一个非回环 IPv4。
ROS_IP_ARG=""
INFER_DEVICE="cuda:0"
# ===== 环境适配层 =====
# 组委会（只订阅）环境 vs 本地调试环境，差异集中在三处：出生点布局、
# 传感器话题名、相机分辨率/内参。用 --env-profile 一键切换，避免改代码。
#   local    : 本地自建机型 typhoon_h480_zzufly（2D 激光 + Realsense 深度）
#              出生点 (-17,-3) 系
#   official : 官方 XTDrone sitl_config/launch/robocup.launch
#              出生点 (0,-3) 系；机型只有 CGO3 单目（无 scan / 无深度）
ENV_PROFILE="local"
SPAWN_ORIGIN_X=""
SPAWN_ORIGIN_Y=""
SPAWN_STRIDE_X="3.0"
SPAWN_STRIDE_Y="3.0"
# 传感器话题后缀（挂在 /<vehicle>_<id>/ 之后），可被 --color-topic= 等覆盖，
# 也会导出给 yolo / bbox2coord 启动器使用。
COLOR_TOPIC_SUFFIX="realsense/depth_camera/color/image_raw"
DEPTH_TOPIC_SUFFIX="realsense/depth_camera/depth/image_raw"
SCAN_TOPIC_SUFFIX="scan"
PID_FILE="$SCRIPT_DIR/logs/6g_sim_only.pid"

while [ $# -gt 0 ]; do
    case "$1" in
        --master-uri=*)      MASTER_URI="${1#*=}" ;;
        --rviz)              WITH_RVIZ="1" ;;
        --no-rviz)           WITH_RVIZ="0" ;;
        --score)             ENABLE_SCORE="1" ;;
        --no-score)          ENABLE_SCORE="0" ;;
        --num-drones=*)      NUM_DRONES="${1#*=}" ;;
        --truth-fallback=*)  TRUTH_FALLBACK="${1#*=}" ;;
        --infer-device=*)    INFER_DEVICE="${1#*=}" ;;
        --env-profile=*)     ENV_PROFILE="${1#*=}" ;;
        --spawn-origin-x=*)  SPAWN_ORIGIN_X="${1#*=}" ;;
        --spawn-origin-y=*)  SPAWN_ORIGIN_Y="${1#*=}" ;;
        --spawn-stride-x=*)  SPAWN_STRIDE_X="${1#*=}" ;;
        --spawn-stride-y=*)  SPAWN_STRIDE_Y="${1#*=}" ;;
        --color-topic=*)     COLOR_TOPIC_SUFFIX="${1#*=}" ;;
        --depth-topic=*)     DEPTH_TOPIC_SUFFIX="${1#*=}" ;;
        --scan-topic=*)      SCAN_TOPIC_SUFFIX="${1#*=}" ;;
        --check-only)        CHECK_ONLY="1" ;;
        --ros-ip=*)          ROS_IP_ARG="${1#*=}" ;;
        *)                   echo "未知参数：$1" >&2; exit 2 ;;
    esac
    shift
done

if ! [[ "$MASTER_URI" =~ ^https?://[^:/?]+:[0-9]+/?$ ]]; then
    echo "错误：--master-uri 必须是 http(s)://host:port 形式，当前：$MASTER_URI" >&2
    exit 2
fi
if ! [[ "$NUM_DRONES" =~ ^[1-6]$ ]]; then
    echo "错误：--num-drones 必须是 1..6，当前：$NUM_DRONES" >&2
    exit 2
fi
if ! [[ "$TRUTH_FALLBACK" =~ ^[01]$ ]]; then
    echo "错误：--truth-fallback 必须是 0 或 1，当前：$TRUTH_FALLBACK" >&2
    exit 2
fi

# ===== 环境画像解析（出生点布局默认值）=====
case "$ENV_PROFILE" in
    local)
        [ -n "$SPAWN_ORIGIN_X" ] || SPAWN_ORIGIN_X="-17.0"
        [ -n "$SPAWN_ORIGIN_Y" ] || SPAWN_ORIGIN_Y="-3.0"
        ;;
    official)
        # 组委会裁判机实际在跑的 robocup.launch（docs/reference/official_judge_robocup.launch）：
        #   0:(-17,-3) 1:(-14,-3) 2:(-17,0) 3:(-14,0) 4:(-17,3) 5:(-14,3)   z=1
        #   （注意：stock XTDrone 的 (0,-3) 系布局不是比赛用的那套）
        [ -n "$SPAWN_ORIGIN_X" ] || SPAWN_ORIGIN_X="-17.0"
        [ -n "$SPAWN_ORIGIN_Y" ] || SPAWN_ORIGIN_Y="-3.0"
        ;;
    *)
        echo "错误：--env-profile 只能是 local 或 official，当前：$ENV_PROFILE" >&2
        exit 2
        ;;
esac
[ -n "$SPAWN_ORIGIN_X" ] && [ -n "$SPAWN_ORIGIN_Y" ] || {
    echo "错误：无法确定出生点布局（$ENV_PROFILE）" >&2; exit 2; }

# 导出给子脚本（yolo / bbox2coord 启动器会读取这些后缀拼话题名）
export ZZU_ENV_PROFILE="$ENV_PROFILE"
export ZZU_COLOR_TOPIC_SUFFIX="$COLOR_TOPIC_SUFFIX"
export ZZU_DEPTH_TOPIC_SUFFIX="$DEPTH_TOPIC_SUFFIX"
export ZZU_SCAN_TOPIC_SUFFIX="$SCAN_TOPIC_SUFFIX"
export ZZU_SPAWN_ORIGIN_X="$SPAWN_ORIGIN_X"
export ZZU_SPAWN_ORIGIN_Y="$SPAWN_ORIGIN_Y"

SPAWN_ARGS=(
    "spawn_origin_x:=$SPAWN_ORIGIN_X"
    "spawn_origin_y:=$SPAWN_ORIGIN_Y"
    "spawn_stride_x:=$SPAWN_STRIDE_X"
    "spawn_stride_y:=$SPAWN_STRIDE_Y"
)

echo "================================================"
echo " 仿真端（方向 A：只起上层算法节点）"
echo " master=$MASTER_URI  rviz=$([ "$WITH_RVIZ" = "1" ] && echo on || echo off)  score=$([ "$ENABLE_SCORE" = "1" ] && echo on || echo off)"
echo " num_drones=$NUM_DRONES  truth_fallback=$TRUTH_FALLBACK  infer_device=$INFER_DEVICE"
echo " env_profile=$ENV_PROFILE  spawn=($SPAWN_ORIGIN_X,$SPAWN_ORIGIN_Y) 步长 ($SPAWN_STRIDE_X,$SPAWN_STRIDE_Y)"
echo " 话题后缀: scan=$SCAN_TOPIC_SUFFIX"
echo "           color=$COLOR_TOPIC_SUFFIX"
echo "           depth=$DEPTH_TOPIC_SUFFIX"
if [ "$ENV_PROFILE" = "official" ]; then
    echo " 注意：组委会裁判机实际机型是 typhoon_h480_2d_lidar_nudt（**带 2D 激光**），"
    echo "       所以 /scan 可用；是否带 Realsense 深度取决于该机型（待组委会确认）。"
    echo "       若话题名与默认不同，请用 --scan-topic= / --color-topic= / --depth-topic= 覆盖。"
fi
echo "================================================"

if [ "$CHECK_ONLY" = "1" ]; then
    echo "check-only：参数与环境画像解析通过，未启动任何节点。"
    exit 0
fi

# 上一轮进程清理（仅清本脚本启动的，不杀 gzserver / gzclient / Xorg）
if [ -f "$PID_FILE" ]; then
    OLD_PID="$(cat "$PID_FILE" 2>/dev/null || true)"
    if [ -n "$OLD_PID" ] && kill -0 "$OLD_PID" 2>/dev/null; then
        echo ">>> 停止上一次本脚本启动的进程组（PGID $OLD_PID）"
        kill -TERM "-$OLD_PID" 2>/dev/null || kill -TERM "$OLD_PID" 2>/dev/null || true
        sleep 2
        kill -KILL "-$OLD_PID" 2>/dev/null || kill -KILL "$OLD_PID" 2>/dev/null || true
    fi
fi

mkdir -p "$(dirname "$PID_FILE")"
ROUND_DIR="$SCRIPT_DIR/logs/sim_only_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$ROUND_DIR"
echo "$$" > "$PID_FILE"

# 1.sh:334-335：先 source 系统 ROS，再 source 工作空间
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
fi
source "$SCRIPT_DIR/devel/setup.bash" 2>/dev/null || true

# 1.sh:351-358：~/.ros/log 不可写时回退到工作空间内
if [ -z "${ROS_LOG_DIR:-}" ]; then
    ROS_LOG_BASE="$HOME/.ros/log"
    if ! mkdir -p "$ROS_LOG_BASE" 2>/dev/null || [ ! -w "$ROS_LOG_BASE" ]; then
        export ROS_LOG_DIR="$SCRIPT_DIR/logs/ros"
        mkdir -p "$ROS_LOG_DIR"
        echo "提示：$ROS_LOG_BASE 不可写，ROS 日志改用 $ROS_LOG_DIR"
    fi
fi

# ===== ROS 组网：导出本机可达 IP，并把 master 指向裁判端 =====
if [ -z "$ROS_IP_ARG" ]; then
    ROS_IP_ARG="$(hostname -I 2>/dev/null | awk '{print $1}')"
fi
if [ -z "$ROS_IP_ARG" ]; then
    echo "错误：无法自动探测本机局域网 IP，请用 --ros-ip=<本机IP> 指定" >&2
    exit 2
fi
export ROS_IP="$ROS_IP_ARG"
unset ROS_HOSTNAME
export ROS_MASTER_URI="$MASTER_URI"
echo "ROS_MASTER_URI=$ROS_MASTER_URI  ROS_IP=$ROS_IP"
export TRUTH_FALLBACK="$TRUTH_FALLBACK"

# 等官方仿真端提供 MAVROS 位姿。官方 XTDrone 不保证提供 /gazebo/set_model_state。
echo ">>> 等待官方 XTDrone MAVROS 位姿话题（最多 240 秒）..."
WAITED=0
while [ "$WAITED" -lt 240 ]; do
    if rostopic list 2>/dev/null | grep -qx '/typhoon_h480_0/mavros/local_position/pose'; then
        echo "  官方 MAVROS 位姿接口已就绪"
        break
    fi
    sleep 2
    WAITED=$((WAITED + 2))
done
if ! rostopic list 2>/dev/null | grep -qx '/typhoon_h480_0/mavros/local_position/pose'; then
    echo "错误：240 秒内未找到官方 MAVROS 位姿接口。" >&2
    echo "请先由官方启动 roslaunch px4 robocup.launch，再启动本算法脚本。" >&2
    exit 1
fi

# 探测 rosmaster（仿真端永远不启 roscore，必须共用裁判端 master）
if ! rostopic list >/dev/null 2>&1; then
    echo "错误：rosmaster 不可达（$MASTER_URI）。请确认裁判端已启动，或先手动执行 roscore -p 11311" >&2
    exit 1
fi

# 官方 robocup.launch 的 MAVROS 与模型由官方负责；算法仍需要每架机的
# 位姿、激光和相机输入。只检查话题是否注册，不依赖裁判私有服务。
MISSING_IFACES=""
# 必需输入按 --env-profile 解析出的后缀拼，避免"话题名写死 → 环境一换就静默无数据"。
REQUIRED_TOPIC_SUFFIXES=(
    "mavros/local_position/pose"
    "$SCAN_TOPIC_SUFFIX"
    "$COLOR_TOPIC_SUFFIX"
    "$DEPTH_TOPIC_SUFFIX"
)
for ID in $(seq 0 $((NUM_DRONES - 1))); do
    for SUFFIX in "${REQUIRED_TOPIC_SUFFIXES[@]}"; do
        TOPIC="/typhoon_h480_${ID}/${SUFFIX}"
        if ! rostopic list 2>/dev/null | grep -qx "$TOPIC"; then
            MISSING_IFACES="$MISSING_IFACES $TOPIC"
        fi
    done
done
if [ -n "$MISSING_IFACES" ]; then
    echo "错误：官方仿真未提供算法所需输入话题：$MISSING_IFACES" >&2
    echo "当前环境画像 env_profile=$ENV_PROFILE（scan=$SCAN_TOPIC_SUFFIX）" >&2
    echo "                       color=$COLOR_TOPIC_SUFFIX" >&2
    echo "                       depth=$DEPTH_TOPIC_SUFFIX" >&2
    if [ -f "$SCRIPT_DIR/tools/check_env_profile.py" ]; then
        echo ">>> 完整环境自检（tools/check_env_profile.py）：" >&2
        python3 "$SCRIPT_DIR/tools/check_env_profile.py" --num-drones="$NUM_DRONES" >&2 || true
    fi
    echo "排查建议：" >&2
    echo "  1) 官方 robocup.launch 用的是 typhoon_h480 基础型，只有 CGO3 单目相机，" >&2
    echo "     不含 /scan 与 /realsense/depth_camera/*；请与组委会确认实际机型与话题名。" >&2
    echo "  2) 若官方确实提供别的话题名，用 --scan-topic=/--color-topic=/--depth-topic= 覆盖。" >&2
    echo "  3) 若官方只有 CGO3 单目，本栈的建图/深度解算无法工作，需要单目降级方案。" >&2
    exit 1
fi

# ----------------------------------------------------------------------
# 阶段 A：纯算法 helper（与 1.sh start_sim_helpers 对齐）
# 仿真端不启 multi_yolo_detecting*（缺相机图像、cv_bridge 会卡住）。
# ----------------------------------------------------------------------
HELPER_PIDS=()

# 提前注册 trap，确保任何早期 exit 都会回收 helper
cleanup_sim_only() {
    local _exit_code=$?
    echo ">>> 退出清理（exit=$_exit_code）..."
    for HPID in "${HELPER_PIDS[@]:-}"; do
        [ -n "$HPID" ] && kill -INT -- "-$HPID" 2>/dev/null || kill -INT "$HPID" 2>/dev/null || true
    done
    [ -n "${SCORE_PID:-}" ] && kill -INT -- "-$SCORE_PID" 2>/dev/null || kill -INT "$SCORE_PID" 2>/dev/null || true
    [ -n "${MIRROR_PID:-}" ] && kill -INT -- "-$MIRROR_PID" 2>/dev/null || kill -INT "$MIRROR_PID" 2>/dev/null || true
    [ -n "${SIM_PID:-}" ] && kill -INT -- "-$SIM_PID" 2>/dev/null || kill -INT "$SIM_PID" 2>/dev/null || true
    sleep 2
    for HPID in "${HELPER_PIDS[@]:-}"; do
        [ -n "$HPID" ] && kill -KILL -- "-$HPID" 2>/dev/null || kill -KILL "$HPID" 2>/dev/null || true
    done
    [ -n "${SCORE_PID:-}" ] && kill -KILL -- "-$SCORE_PID" 2>/dev/null || kill -KILL "$SCORE_PID" 2>/dev/null || true
    [ -n "${MIRROR_PID:-}" ] && kill -KILL -- "-$MIRROR_PID" 2>/dev/null || kill -KILL "$MIRROR_PID" 2>/dev/null || true
    [ -n "${SIM_PID:-}" ] && kill -KILL -- "-$SIM_PID" 2>/dev/null || kill -KILL "$SIM_PID" 2>/dev/null || true
    rm -f "$PID_FILE"
    return $_exit_code
}
trap 'cleanup_sim_only' INT TERM
trap 'cleanup_sim_only; exit $?' EXIT

echo ">>> [helper] 启动 static_tf.launch（map/world/ground_plane/camera_link 静态变换）"
setsid roslaunch simple_navigator static_tf.launch \
    >"$ROUND_DIR/static_tf.log" 2>&1 &
HELPER_PIDS+=("$!")

echo ">>> [helper] 启动 pose.launch（多无人机位姿转换，出生点布局 $SPAWN_ORIGIN_X,$SPAWN_ORIGIN_Y）"
setsid roslaunch "$SCRIPT_DIR/src/pose_init/launch/pose.launch" \
    "num_drones:=$NUM_DRONES" "${SPAWN_ARGS[@]}" \
    >"$ROUND_DIR/pose_init.log" 2>&1 &
HELPER_PIDS+=("$!")

echo ">>> [helper] 启动 transform_tree/tf.launch num_drones=$NUM_DRONES（base_link 动态 TF）"
setsid roslaunch "$SCRIPT_DIR/src/transform_tree/launch/tf.launch" "num_drones:=$NUM_DRONES" \
    >"$ROUND_DIR/tf_tree.log" 2>&1 &
HELPER_PIDS+=("$!")

echo ">>> [helper] 启动 target_lookup_service.launch"
setsid roslaunch "$SCRIPT_DIR/src/look_up/launch/target_lookup_service.launch" \
    >"$ROUND_DIR/target_lookup.log" 2>&1 &
HELPER_PIDS+=("$!")

echo ">>> [helper] 启动 spawn_mux_swarm.launch num_drones=$NUM_DRONES（area_mission cmd_vel 必需）"
setsid roslaunch "$SCRIPT_DIR/src/look_up/launch/spawn_mux_swarm.launch" "num_drones:=$NUM_DRONES" \
    >"$ROUND_DIR/spawn_mux.log" 2>&1 &
HELPER_PIDS+=("$!")

echo ">>> [helper] 启动 xtdrone_mavros_bridge（官方 MAVROS 控制适配）"
setsid roslaunch "$SCRIPT_DIR/src/xtdrone_mavros_bridge/launch/bridge.launch" \
    "num_drones:=$NUM_DRONES" \
    >"$ROUND_DIR/bridge.log" 2>&1 &
HELPER_PIDS+=("$!")

# ----------------------------------------------------------------------
# 阶段 B：auto_explore_swarm.launch（必须在 health gate 之前起，作为上游生产者）
# ----------------------------------------------------------------------
if [ "$WITH_RVIZ" = "1" ]; then
    RVIZ_ARG="rviz:=true"
else
    RVIZ_ARG="rviz:=false"
fi

echo ">>> 启动 auto_explore_swarm.launch num_drones=$NUM_DRONES ($RVIZ_ARG, spawn=$SPAWN_ORIGIN_X,$SPAWN_ORIGIN_Y)..."
setsid roslaunch "$SCRIPT_DIR/src/auto_explore/launch/auto_explore_swarm.launch" \
    "num_drones:=$NUM_DRONES" "infer_device:=$INFER_DEVICE" "$RVIZ_ARG" "${SPAWN_ARGS[@]}" \
    >"$ROUND_DIR/sim.log" 2>&1 &
SIM_PID=$!
echo "  auto_explore_swarm PID=$SIM_PID"

# 早死探测：2 秒后看 roslaunch 父进程是否还活着
sleep 2
if ! kill -0 "$SIM_PID" 2>/dev/null; then
    echo "错误：auto_explore_swarm.launch 在启动后 2 秒内已退出，详情见 $ROUND_DIR/sim.log" >&2
    tail -30 "$ROUND_DIR/sim.log" >&2 || true
    exit 1
fi

# ----------------------------------------------------------------------
# 阶段 C：上层栈健康门（逐机检查位姿、激光、相机和 /map）
# 1.sh:521-530 调 check_upper_stack.py；此处改用轻量级 rostopic + tf2 检查，
# 因为 check_upper_stack.py 会检查 /mavros/state，而仿真端无 mavros。
# ----------------------------------------------------------------------
echo ">>> 上层栈健康门：6 架位姿/激光/相机 + /map（最长 240 秒）..."
HEALTH_TIMEOUT=240
HEALTH_ELAPSED=0
HEALTH_OK=0
HEALTH_FAIL_REASON=""
while [ "$HEALTH_ELAPSED" -lt "$HEALTH_TIMEOUT" ]; do
    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        echo "错误：auto_explore_swarm 父进程已退出，详情见 $ROUND_DIR/sim.log" >&2
        tail -30 "$ROUND_DIR/sim.log" >&2 || true
        exit 1
    fi
    TOPIC_LIST="$(rostopic list 2>/dev/null)" || TOPIC_LIST=""
    HAS_MAP=0; READY_COUNT=0
    if echo "$TOPIC_LIST" | grep -q '^/map$'; then HAS_MAP=1; fi
    for ID in $(seq 0 $((NUM_DRONES - 1))); do
        if echo "$TOPIC_LIST" | grep -qx "/typhoon_h480_${ID}/scan_filtered" \
            && echo "$TOPIC_LIST" | grep -qx "/typhoon_h480_${ID}/mavros/local_position/pose" \
            && echo "$TOPIC_LIST" | grep -qx "/typhoon_h480_${ID}/realsense/depth_camera/color/image_raw" \
            && echo "$TOPIC_LIST" | grep -qx "/typhoon_h480_${ID}/realsense/depth_camera/depth/image_raw"; then
            READY_COUNT=$((READY_COUNT + 1))
        fi
    done
    if [ "$HAS_MAP" = "1" ] && [ "$READY_COUNT" -eq "$NUM_DRONES" ]; then
        HEALTH_OK=1
        echo "  上层栈健康（${READY_COUNT}/${NUM_DRONES} 架位姿/激光/相机 + /map 就绪）"
        break
    fi
    if [ "$HEALTH_ELAPSED" = "0" ]; then
        HEALTH_FAIL_REASON="缺少："
        [ "$HAS_MAP" = "0" ] && HEALTH_FAIL_REASON="$HEALTH_FAIL_REASON /map"
        [ "$READY_COUNT" -lt "$NUM_DRONES" ] && HEALTH_FAIL_REASON="$HEALTH_FAIL_REASON ${NUM_DRONES}-${READY_COUNT} 架传感器/位姿"
    fi
    sleep 5
    HEALTH_ELAPSED=$((HEALTH_ELAPSED + 5))
done
if [ "$HEALTH_OK" != "1" ]; then
    echo "错误：${HEALTH_TIMEOUT}s 内上层栈健康门未通过${HEALTH_FAIL_REASON:+（$HEALTH_FAIL_REASON）}" >&2
    echo "  这通常意味着：官方未加载带传感器的 SDF，或 PX4/MAVROS/scan/realsense 未发布" >&2
    echo "  请确认另一终端的 start_referee.sh / start_referee_fast.sh 已正确启动并等" >&2
    echo "  所有 typhoon_h480_<i>/mavros/state connected=True 后再追 check_zone.py 验证" >&2
    exit 1
fi

# ----------------------------------------------------------------------
# 阶段 D：感知 + 追踪（multi_yolo / multi_solving / multi_gimbal / tracking）
# yolo11n.py 订阅裁判端 gzserver 发布的 realsense 图像，发布 bounding_boxes 给 bbox2coord
# ----------------------------------------------------------------------
echo ">>> [helper] 启动 multi_yolo_detecting_gpu.sh $NUM_DRONES（订阅裁判端 realsense 图）"
cd "$SCRIPT_DIR/src/yolo" && setsid bash multi_yolo_detecting_gpu.sh "$NUM_DRONES" \
    >"$ROUND_DIR/multi_yolo.log" 2>&1 &
HELPER_PIDS+=("$!")

echo ">>> [helper] 启动 multi_solving.sh $NUM_DRONES（TRUTH_FALLBACK=$TRUTH_FALLBACK via env）"
cd "$SCRIPT_DIR/src/yolo" && setsid bash multi_solving.sh "$NUM_DRONES" \
    >"$ROUND_DIR/multi_solving.log" 2>&1 &
HELPER_PIDS+=("$!")

echo ">>> [helper] 启动 multi_gimbal_control.sh $NUM_DRONES"
cd "$SCRIPT_DIR/src/gimbal" && setsid bash multi_gimbal_control.sh "$NUM_DRONES" \
    >"$ROUND_DIR/multi_gimbal.log" 2>&1 &
HELPER_PIDS+=("$!")
cd "$SCRIPT_DIR"

echo ">>> [helper] 启动 tracking.launch num_drones=$NUM_DRONES"
setsid roslaunch "$SCRIPT_DIR/src/tracking/launch/tracking.launch" "num_drones:=$NUM_DRONES" \
    >"$ROUND_DIR/tracking.log" 2>&1 &
HELPER_PIDS+=("$!")

SCORE_PID=""
MIRROR_PID=""
if [ "$ENABLE_SCORE" = "1" ]; then
    echo ">>> --score 模式：启动工作空间 score_cal.py + referee_mirror.py（本机独占调试）"
    # 把 score_cal 的 score.txt 写到本端 round dir，避免与裁判端 score.txt 冲突
    export SCORE_OUTPUT_PATH="$ROUND_DIR/score.txt"
    setsid python3 -u "$SCRIPT_DIR/score_cal.py" typhoon_h480 >"$ROUND_DIR/referee.log" 2>&1 &
    SCORE_PID=$!
    setsid python3 -u "$SCRIPT_DIR/src/yolo/referee_mirror.py" typhoon_h480 "$NUM_DRONES" "$NUM_DRONES" \
        >"$ROUND_DIR/mirror.log" 2>&1 &
    MIRROR_PID=$!
    echo "    score_cal PID=$SCORE_PID, referee_mirror PID=$MIRROR_PID"
    echo "    score_cal score.txt → $ROUND_DIR/score.txt"
else
    echo ">>> --no-score 模式：不启 score_cal / mirror，裁判端独占计分"
fi

echo ">>> 仿真端就绪；Ctrl-C 退出（裁判端仍在另一终端跑）"
echo "    round dir: $ROUND_DIR"
echo "    sim.log: $ROUND_DIR/sim.log"

wait "$SIM_PID" 2>/dev/null || true
exit 0
