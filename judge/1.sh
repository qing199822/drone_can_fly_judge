#!/bin/bash

# ===================================================================
# ROS / PX4 单机仿真启动脚本
# 用法: bash 1.sh [无人机数量] [任务文件名] [sim|mission|full] [gui|headless] [地图]
# base0 单机全道路与视觉跟踪（默认）: bash 1.sh
# 只启动 base0 仿真: bash 1.sh 1 mission_base0_all_roads.json sim gui base0
# 不启动 YOLO 的航点任务: bash 1.sh 1 mission_base0_all_roads.json mission gui base0
# 无界面完整任务: bash 1.sh 1 mission_base0_all_roads.json full headless base0
# 空世界基础仿真: bash 1.sh 1 mission_base0_all_roads.json sim gui empty
# ===================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"
PX4_DIR="$HOME/PX4_Firmware"
PX4_BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
SIMULATION_LAUNCH="$WORKSPACE_DIR/robocup_zzufly.launch"
GAZEBO_ONLY_LAUNCH="$WORKSPACE_DIR/robocup_zzufly_gazebo.launch"
MAP_ASSETS_DIR="$WORKSPACE_DIR/gazebo_map_assets"
READY_TIMEOUT_SECONDS=180
ACTOR_PLUGIN_LIBRARY_NAME="libros_actor_cmd_pose_plugin.so"

NUM_DRONES=${1:-1}
MISSION_FILE=${2:-mission_base0_all_roads.json}
RUN_MODE=${3:-full}
DISPLAY_MODE=${4:-gui}
WORLD_INPUT=${5:-base0}
# 设为 1 时，sim 模式会自动启动 auto_explore_swarm。
AUTO_EXPLORE_SWARM="${AUTO_EXPLORE_SWARM:-0}"
# 设为 1（默认）时使用严格串行门控启动：Gazebo 世界就绪 -> 逐架
#   spawn 模型 -> 确认模型在 Gazebo -> 启动 PX4 -> 等 MAVROS connected=True -> 下一架。
# 设为 0 时回退到旧的 robocup_zzufly.launch 一次性并行启动方式。
SERIAL_SWARM_START="${SERIAL_SWARM_START:-1}"
# 底层启动模式（传给 serial_swarm_bringup.py）：
#   batch  = 串行生成模型 + 错峰并发启动 PX4/MAVROS（快，默认，目标 30s 内）
#   serial = 逐架严格串行（最保守，每架等 connected 才动下一架）
STARTUP_MODE="${STARTUP_MODE:-batch}"
# batch 模式相邻两架 PX4/MAVROS 的启动间隔秒数。
PX4_STAGGER="${PX4_STAGGER:-0.5}"
# connected 后的稳定观察秒数（默认 0：健康门已做验收，要更保守可设 2~4）。
SERIAL_SETTLE="${SERIAL_SETTLE:-0}"
# 起始 drone id（断点续启用；0 表示从 drone0 开始）。
SERIAL_START_INDEX="${SERIAL_START_INDEX:-0}"
# 设为 0 时 sim 模式只启动底层 + 建图/区域任务，不启动 YOLO/tracking，
# 便于分层排查"到底是 Gazebo/PX4 层还是算法层的问题"。
START_PERCEPTION="${START_PERCEPTION:-1}"
# 设为 0 时连"阶段 A"辅助节点（静态 TF/坐标解算/MUX/查找服务）也不启动，
# 只保留 Gazebo + PX4 + MAVROS + 控制桥，用于把 gzserver 段错误与上层节点解耦。
START_SIM_HELPERS="${START_SIM_HELPERS:-1}"
GAZEBO_GUI=true
SIMULATION_PID=""
AUTO_EXPLORE_SWARM_PID=""
HELPER_PIDS=()
ACTOR_NAMES=()
ACTOR_PLUGIN_DIR=""
CLEANUP_DONE=false

cleanup() {
    if "$CLEANUP_DONE"; then
        return
    fi
    CLEANUP_DONE=true

    echo
    echo "正在停止本脚本启动的节点..."

    for pid in "${HELPER_PIDS[@]}"; do
        # Helper scripts spawn several Python processes then exit. They remain
        # in the setsid-created process group whose ID is the saved PID.
        kill -INT -- "-$pid" 2>/dev/null || true
    done

    if [ -n "$SIMULATION_PID" ] && kill -0 "$SIMULATION_PID" 2>/dev/null; then
        kill -INT "$SIMULATION_PID" 2>/dev/null || true
    fi

    if [ "${MODEL_LINK_CREATED:-false}" = true ]; then
        rm -f "$MODEL_LINK"
    fi

    if [ "${AIRFRAME_LINK_CREATED:-false}" = true ]; then
        rm -f "$AIRFRAME_LINK"
    fi
    if [ "${AIRFRAME_POST_LINK_CREATED:-false}" = true ]; then
        rm -f "$AIRFRAME_POST_LINK"
    fi

    for pid in "${HELPER_PIDS[@]}" "$SIMULATION_PID"; do
        if [ -n "$pid" ]; then
            wait "$pid" 2>/dev/null || true
        fi
    done
}

on_interrupt() {
    cleanup
    exit 130
}

trap cleanup EXIT
trap on_interrupt INT TERM

require_file() {
    if [ ! -e "$1" ]; then
        echo "错误：找不到 $2：$1" >&2
        exit 1
    fi
}

require_ros_package() {
    if ! rospack find "$1" >/dev/null 2>&1; then
        echo "错误：未找到 ROS 软件包 '$1'。请确认环境和依赖已正确安装。" >&2
        exit 1
    fi
}

require_command() {
    if ! command -v "$1" >/dev/null 2>&1; then
        echo "错误：找不到命令 '$1'，请先安装 $2。" >&2
        exit 1
    fi
}

resolve_actor_plugin_dir() {
    local directory

    for directory in \
        "$WORKSPACE_DIR/devel/lib" \
        "$WORKSPACE_DIR/build/ros_actor_cmd_pose_plugin"; do
        if [ -r "$directory/$ACTOR_PLUGIN_LIBRARY_NAME" ]; then
            realpath "$directory"
            return 0
        fi
    done

    return 1
}

resolve_world_file() {
    local input="$1"
    local candidate

    case "$input" in
        empty|empty.world)
            candidate="/usr/share/gazebo-11/worlds/empty.world"
            ;;
        base[0-9]|base1[0-6])
            candidate="$WORKSPACE_DIR/waypoint/world/${input}.world"
            ;;
        base[0-9].world|base1[0-6].world)
            candidate="$WORKSPACE_DIR/waypoint/world/$input"
            ;;
        *)
            if [ -f "$input" ]; then
                candidate="$input"
            else
                candidate="$WORKSPACE_DIR/$input"
            fi
            ;;
    esac

    if [ ! -f "$candidate" ]; then
        echo "错误：找不到 Gazebo 地图：$input" >&2
        echo "可使用 empty、base0 到 base16，或传入 .world 文件路径。" >&2
        return 1
    fi

    realpath "$candidate"
}

project_simulator_running() {
    rosnode list 2>/dev/null | grep -Eq '^/(gazebo|typhoon_h480_0/mavros)$'
}

# 本次启动涉及的 drone id 序列（支持 SERIAL_START_INDEX 断点续启）。
drone_id_seq() {
    seq "$SERIAL_START_INDEX" $((SERIAL_START_INDEX + NUM_DRONES - 1))
}

all_vehicles_ready() {
    local topic_list
    local id state_topic pose_topic scan_topic

    rosservice list 2>/dev/null | grep -qx '/gazebo/get_world_properties' || return 1
    topic_list="$(rostopic list 2>/dev/null)" || return 1

    for id in $(drone_id_seq); do
        state_topic="/typhoon_h480_${id}/mavros/state"
        pose_topic="/typhoon_h480_${id}/mavros/local_position/pose"
        scan_topic="/typhoon_h480_${id}/scan"

        grep -qx "$pose_topic" <<< "$topic_list" || return 1
        grep -qx "$scan_topic" <<< "$topic_list" || return 1
        timeout 2s rostopic echo -n 1 "$state_topic" 2>/dev/null | grep -q 'connected: True' || return 1
        timeout 2s rostopic echo -n 1 "$pose_topic" >/dev/null 2>&1 || return 1
        timeout 2s rostopic echo -n 1 "$scan_topic" >/dev/null 2>&1 || return 1
    done

    for actor_name in "${ACTOR_NAMES[@]}"; do
        grep -qx "/${actor_name}/cmd_motion" <<< "$topic_list" || return 1
        grep -qx "/${actor_name}/cmd_vel" <<< "$topic_list" || return 1
    done
}

# 等待 Gazebo 的世界服务可用（串行编排器启动前的快速前置检查）。
# base0 这类大地图在软件渲染/慢机器上可能需要数分钟才能加载完。
wait_for_gazebo_service() {
    local deadline=$(( $(date +%s) + 600 ))
    local started=$(( $(date +%s) ))
    local last_report=$started

    while [ "$(date +%s)" -lt "$deadline" ]; do
        if rosservice list 2>/dev/null | grep -qx '/gazebo/get_world_properties'; then
            return 0
        fi
        if [ -n "$SIMULATION_PID" ] && ! kill -0 "$SIMULATION_PID" 2>/dev/null; then
            echo "错误：Gazebo 启动进程已退出（可能是 gzserver 段错误）。" >&2
            return 1
        fi
        if [ $(( $(date +%s) - last_report )) -ge 30 ]; then
            last_report=$(date +%s)
            echo "  仍在等待 Gazebo 世界服务（已等待 $(( $(date +%s) - started ))s）..."
        fi
        sleep 2
    done
    return 1
}

# 上层栈健康门：激光过滤 + TF 树 + /map 可用后，才允许启动视觉/追踪节点。
# 具体检查由 check_upper_stack.py 用 rospy 完成（比在 shell 里反复 rostopic echo 更可靠）。
wait_for_upper_stack_ready() {
    echo "等待上层栈健康门：scan_filtered + TF + /map（最长 240 秒）..."
    if rosrun xtdrone_mavros_bridge check_upper_stack.py \
            --num-drones "$NUM_DRONES" --start-index "$SERIAL_START_INDEX" \
            --timeout 210; then
        return 0
    fi
    echo "错误：上层栈健康门未通过（scan_filtered / TF / /map）。" >&2
    return 1
}

# 串行模式下底层已由编排器逐架验证过，这里只需补验地图人物话题（纯 list，无需订阅）。
wait_for_actor_topics() {
    local deadline=$(( $(date +%s) + 120 ))
    local topic_list actor_name ready

    if [ "${#ACTOR_NAMES[@]}" -eq 0 ]; then
        return 0
    fi

    echo "等待地图人物话题（${#ACTOR_NAMES[@]} 个 actor，最长 120 秒）..."
    while [ "$(date +%s)" -lt "$deadline" ]; do
        topic_list="$(rostopic list 2>/dev/null)"
        ready=true
        for actor_name in "${ACTOR_NAMES[@]}"; do
            grep -qx "/${actor_name}/cmd_motion" <<< "$topic_list" || ready=false
            grep -qx "/${actor_name}/cmd_vel" <<< "$topic_list" || ready=false
        done
        if [ "$ready" = true ]; then
            echo "地图人物话题已就绪（${#ACTOR_NAMES[@]} 个）。"
            return 0
        fi
        sleep 2
    done

    echo "错误：地图人物话题未在 120 秒内就绪。" >&2
    return 1
}

start_helper() {
    local directory="$1"
    local script_name="$2"
    local title="$3"
    shift 3

    echo "启动${title}..."
    setsid bash -c 'cd "$1" && shift && exec bash "$@"' -- \
        "$directory" "./$script_name" "$@" &
    HELPER_PIDS+=("$!")
}

# --- ROS、PX4 与 Gazebo 环境设置 ---
if [ "$NUM_DRONES" -lt 1 ] || [ "$NUM_DRONES" -gt 6 ]; then
    echo "错误：无人机数量仅支持 1~6 架（当前: $NUM_DRONES）。" >&2
    exit 1
fi

case "$RUN_MODE" in
    sim|mission|full) ;;
    *)
        echo "错误：运行模式必须是 sim、mission 或 full。" >&2
        exit 1
        ;;
esac

case "$DISPLAY_MODE" in
    gui) GAZEBO_GUI=true ;;
    headless) GAZEBO_GUI=false ;;
    *)
        echo "错误：显示模式必须是 gui 或 headless。" >&2
        exit 1
        ;;
esac

require_file /opt/ros/noetic/setup.bash "ROS Noetic 环境脚本"
require_file "$PX4_DIR/Tools/setup_gazebo.bash" "PX4 Gazebo 环境脚本"
require_file "$PX4_BUILD_DIR/bin/px4" "PX4 SITL 编译产物"
require_file "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/6011_typhoon_h480" "PX4 Typhoon 六旋翼配置"
require_file "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/6011_typhoon_h480.post" "PX4 Typhoon 六旋翼附加配置"
require_file "$WORKSPACE_DIR/devel/setup.bash" "Catkin 工作空间环境脚本"
require_file "$SIMULATION_LAUNCH" "单机仿真 launch 文件"
require_file "$WORKSPACE_DIR/typhoon_h480_zzufly/typhoon_h480_zzufly.sdf" "自定义 Typhoon 模型文件"
require_file "$MAP_ASSETS_DIR/models/walker/walk_0.dae" "Gazebo 地图模型"
require_file "$MAP_ASSETS_DIR/plugins/libActorCollisionsPlugin.so" "Gazebo 人物碰撞插件"
require_command xmlstarlet "xmlstarlet"
WORLD_FILE="$(resolve_world_file "$WORLD_INPUT")" || exit 1
mapfile -t ACTOR_NAMES < <(
    xmlstarlet sel -t \
        -m "//*[local-name()='actor'][*[local-name()='plugin'][@filename='$ACTOR_PLUGIN_LIBRARY_NAME']]" \
        -v '@name' -n "$WORLD_FILE"
)
if [ "${#ACTOR_NAMES[@]}" -gt 0 ]; then
    if ! ACTOR_PLUGIN_DIR="$(resolve_actor_plugin_dir)"; then
        echo "错误：地图需要人物运动插件，但找不到 $ACTOR_PLUGIN_LIBRARY_NAME。" >&2
        echo "请先运行：catkin build ros_actor_cmd_pose_plugin" >&2
        exit 1
    fi
fi
if [ "$RUN_MODE" = "full" ]; then
    require_file "$WORKSPACE_DIR/src/yolo/multi_yolo_detecting.sh" "YOLO 检测脚本"
    require_file "$WORKSPACE_DIR/src/yolo/multi_solving.sh" "坐标计算脚本"
fi

source /opt/ros/noetic/setup.bash
source "$WORKSPACE_DIR/devel/setup.bash"
source "$PX4_DIR/Tools/setup_gazebo.bash" "$PX4_DIR" "$PX4_BUILD_DIR"
export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:+${ROS_PACKAGE_PATH}:}$PX4_DIR:$PX4_DIR/Tools/sitl_gazebo"
# GPU 渲染（RTX 5070 已装驱动；如遇 Gazebo 黑屏/崩溃可临时改回软渲染：
#   export LIBGL_ALWAYS_SOFTWARE=1; export GALLIUM_DRIVER=llvmpipe)
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH="$MAP_ASSETS_DIR/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export GAZEBO_PLUGIN_PATH="$MAP_ASSETS_DIR/plugins${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
if [ -n "$ACTOR_PLUGIN_DIR" ]; then
    export GAZEBO_PLUGIN_PATH="$ACTOR_PLUGIN_DIR:$GAZEBO_PLUGIN_PATH"
fi
if [ -r "$WORKSPACE_DIR/geographiclib/geoids/egm96-5.pgm" ]; then
    export GEOGRAPHICLIB_DATA="$WORKSPACE_DIR/geographiclib"
fi
# 若 ~/.ros/log 不可写（只读挂载的 HOME、受限沙箱等），rosmaster/roslaunch 会因为
# 无法创建日志目录而直接退出。此时把 ROS 日志目录改到工作空间内。
if [ -z "${ROS_LOG_DIR:-}" ]; then
    ROS_LOG_BASE="$HOME/.ros/log"
    if ! mkdir -p "$ROS_LOG_BASE" 2>/dev/null || [ ! -w "$ROS_LOG_BASE" ]; then
        export ROS_LOG_DIR="$WORKSPACE_DIR/logs/ros"
        mkdir -p "$ROS_LOG_DIR"
        echo "提示：$ROS_LOG_BASE 不可写，ROS 日志改用 $ROS_LOG_DIR"
    fi
fi
# single_vehicle_spawn_xtd.launch 从 PX4 的模型目录读取 SDF；本项目的自定义
# Typhoon 模型保存在工作空间根目录，因此为本次启动临时链接到该目录。
MODEL_LINK="$PX4_DIR/Tools/sitl_gazebo/models/typhoon_h480_zzufly"
MODEL_LINK_CREATED=false
if [ -L "$MODEL_LINK" ] && [ ! -e "$MODEL_LINK" ]; then
    echo "错误：PX4 自定义模型路径是损坏的符号链接：$MODEL_LINK" >&2
    exit 1
elif [ ! -e "$MODEL_LINK" ]; then
    if [ ! -w "$(dirname "$MODEL_LINK")" ]; then
        echo "错误：无权创建 PX4 自定义模型链接：$(dirname "$MODEL_LINK")" >&2
        exit 1
    fi
    ln -s "$WORKSPACE_DIR/typhoon_h480_zzufly" "$MODEL_LINK"
    MODEL_LINK_CREATED=true
elif [ ! -d "$MODEL_LINK" ] || [ ! -f "$MODEL_LINK/typhoon_h480_zzufly.sdf" ]; then
    echo "错误：PX4 自定义模型路径无效或缺少 typhoon_h480_zzufly.sdf：$MODEL_LINK" >&2
    exit 1
fi

# 该 PX4 版本同时包含 4001_typhoon_h480（四旋翼）和
# 6011_typhoon_h480（六旋翼）。PX4 按后缀查找时会同时命中，因此为
# 6011 创建唯一别名，确保六电机 SDF 使用 hexa_x 混控器。
AIRFRAME_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix"
AIRFRAME_LINK="$AIRFRAME_DIR/6011_typhoon_h480_zzufly"
AIRFRAME_POST_LINK="$AIRFRAME_LINK.post"
AIRFRAME_LINK_CREATED=false
AIRFRAME_POST_LINK_CREATED=false

if [ ! -e "$AIRFRAME_LINK" ]; then
    ln -s "$AIRFRAME_DIR/6011_typhoon_h480" "$AIRFRAME_LINK"
    AIRFRAME_LINK_CREATED=true
elif [ ! "$AIRFRAME_LINK" -ef "$AIRFRAME_DIR/6011_typhoon_h480" ]; then
    echo "错误：PX4 六旋翼机型别名指向了其他文件：$AIRFRAME_LINK" >&2
    exit 1
fi

if [ ! -e "$AIRFRAME_POST_LINK" ]; then
    ln -s "$AIRFRAME_DIR/6011_typhoon_h480.post" "$AIRFRAME_POST_LINK"
    AIRFRAME_POST_LINK_CREATED=true
elif [ ! "$AIRFRAME_POST_LINK" -ef "$AIRFRAME_DIR/6011_typhoon_h480.post" ]; then
    echo "错误：PX4 六旋翼附加配置别名指向了其他文件：$AIRFRAME_POST_LINK" >&2
    exit 1
fi

for package in px4 mavlink_sitl_gazebo mavros gazebo_ros look_up; do
    require_ros_package "$package"
done

if [ "$AUTO_EXPLORE_SWARM" = "1" ]; then
    require_ros_package "auto_explore"
    require_file "$WORKSPACE_DIR/src/auto_explore/launch/auto_explore_swarm.launch" "auto_explore swarm 启动文件"
fi

if project_simulator_running; then
    echo "错误：检测到已有 Gazebo 或 typhoon_h480_0 MAVROS 仿真正在运行。" >&2
    echo "请先在原来的终端按 Ctrl-C 停止它，再运行本脚本。" >&2
    exit 1
fi

echo "============================================"
if [ "$GAZEBO_GUI" = true ]; then
    echo "  启动 PX4/Gazebo 集群仿真（可视化）"
else
    echo "  启动 PX4/Gazebo 集群仿真（无界面）"
fi
echo "  无人机数量: $NUM_DRONES"
echo "  运行模式:   $RUN_MODE"
echo "  任务文件:   $MISSION_FILE"
echo "  Gazebo 地图: $WORLD_FILE"
echo "============================================"

echo "启动 Gazebo、PX4 SITL 和 MAVROS..."

# 启动阶段不自动重试：gzserver 段错误或启动失败时直接退出，由用户手动重新启动。
launch_simulation() {
    if [ "$SERIAL_SWARM_START" = "1" ]; then
        # 严格串行门控模式：先只启动 Gazebo 世界，无人机模型/PX4/MAVROS
        # 交给 serial_swarm_bringup.py 逐架启动（见该脚本头部说明）。
        roslaunch "$GAZEBO_ONLY_LAUNCH" gui:="$GAZEBO_GUI" world:="$WORLD_FILE" &
        SIMULATION_PID=$!
    else
        roslaunch "$SIMULATION_LAUNCH" gui:="$GAZEBO_GUI" world:="$WORLD_FILE" num_drones:="$NUM_DRONES" &
        SIMULATION_PID=$!
    fi
}

launch_simulation

if [ "$SERIAL_SWARM_START" = "1" ]; then
    # 先确认 Gazebo 世界服务起来，再交给编排器（编排器内部也会再等一次世界就绪）。
    if ! wait_for_gazebo_service; then
        echo "错误：Gazebo 世界服务未就绪，串行启动中止。" >&2
        exit 1
    fi

    if [ "$STARTUP_MODE" = "batch" ]; then
        echo "按 batch 模式启动 $NUM_DRONES 架无人机（串行生成模型 -> 错峰并发 PX4/MAVROS -> 统一等 connected）："
        echo "  关键保证：世界就绪后才生成；每个模型确认存在后才起它自己的 PX4；全部 connected + 健康门通过才起上层"
    else
        echo "按严格串行门控启动 $NUM_DRONES 架无人机："
        echo "  每架：spawn 模型 -> 确认模型在 Gazebo -> PX4 SITL -> MAVROS connected=True -> 下一架"
    fi
    BRINGUP_STARTED_AT=$(date +%s)
    if ! rosrun xtdrone_mavros_bridge serial_swarm_bringup.py \
            --num-drones "$NUM_DRONES" --start-index "$SERIAL_START_INDEX" \
            --mode "$STARTUP_MODE" --px4-stagger "$PX4_STAGGER" \
            --settle "$SERIAL_SETTLE"; then
        echo "错误：底层启动未通过健康门，拒绝启动上层算法栈。" >&2
        echo "  详情见 $WORKSPACE_DIR/logs/serial_bringup/health_report.json" >&2
        exit 1
    fi
    echo "底层启动总耗时（含世界加载等待 + 健康门）：$(( $(date +%s) - BRINGUP_STARTED_AT ))s"
fi

if [ "$SERIAL_SWARM_START" = "1" ]; then
    # 串行编排器已逐架验证 model / connected / 位姿 / 原始激光，这里只补验人物话题。
    if ! wait_for_actor_topics; then
        echo "错误：地图人物话题未就绪，串行启动中止。" >&2
        exit 1
    fi
elif ! wait_for_simulator; then
    echo "错误：仿真启动失败，不自动重试，请检查日志后手动重启。" >&2
    exit 1
fi

# 感知 + 追踪栈（相机行人识别 -> mux 控制权切换 -> 追踪 -> 位置广播）。
# sim 模式：explore 导航经 mux navigator 输入；yolo 检测行人；bbox2coord
# 广播坐标；tracking 识别后接管控制权。mission/full 由 down_resume 统一启动。
if [ "$RUN_MODE" = "sim" ] && [ "$START_SIM_HELPERS" = "0" ]; then
    echo "START_SIM_HELPERS=0：只保留底层（Gazebo/PX4/MAVROS/控制桥），不启动辅助节点。"
fi

if [ "$RUN_MODE" = "sim" ] && [ "$START_SIM_HELPERS" = "1" ]; then
    # --- 阶段 A：静态 TF / 坐标解算基础 / MUX 仲裁（轻量，无算法负载）---
    echo "启动静态 TF（人物位置坐标解算依赖）..."
    roslaunch simple_navigator static_tf.launch &
    STATIC_TF_PID=$!
    HELPER_PIDS+=("$STATIC_TF_PID")
    sleep 1

    echo "启动位姿转换与 TF 树（人物坐标解算依赖）..."
    roslaunch pose_init pose.launch &
    POSE_INIT_PID=$!
    HELPER_PIDS+=("$POSE_INIT_PID")
    roslaunch transform_tree tf.launch num_drones:="$NUM_DRONES" &
    TF_TREE_PID=$!
    HELPER_PIDS+=("$TF_TREE_PID")
    sleep 1

    echo "启动 MUX 控制权切换..."
    roslaunch look_up spawn_mux_swarm.launch num_drones:="$NUM_DRONES" &
    MUX_PID=$!
    sleep 2

    echo "启动目标查找服务（tracking 依赖）..."
    roslaunch look_up target_lookup_service.launch &
    LOOKUP_PID=$!
    sleep 1

    # --- 阶段 B：激光过滤 + 建图/融合 + 区域任务 ---
    # 视觉与追踪属于重负载，必须在激光过滤和 TF 健康之后才启动，
    # 否则 Gazebo/PX4 层的问题会被误判成算法问题。
    if [ "$AUTO_EXPLORE_SWARM" = "1" ]; then
        echo "启动 auto_explore_swarm（激光过滤 + TF + 区域任务）..."
        roslaunch auto_explore auto_explore_swarm.launch num_drones:="$NUM_DRONES" rviz:=false &
        AUTO_EXPLORE_SWARM_PID=$!
        HELPER_PIDS+=("$AUTO_EXPLORE_SWARM_PID")

        if ! wait_for_upper_stack_ready; then
            echo "错误：激光过滤/TF 未就绪，拒绝启动 YOLO 与 tracking。" >&2
            exit 1
        fi
    fi

    # --- 阶段 C：感知（道路分割 + 行人检测）与追踪接管 ---
    if [ "$START_PERCEPTION" = "1" ]; then
        if [ -x "$WORKSPACE_DIR/src/yolo/multi_yolo_detecting_gpu.sh" ]; then
            start_helper "$WORKSPACE_DIR/src/yolo" "multi_yolo_detecting_gpu.sh" "YOLO 检测(GPU)" "$NUM_DRONES"
        else
            start_helper "$WORKSPACE_DIR/src/yolo" "multi_yolo_detecting.sh" "YOLO 检测(CPU)" "$NUM_DRONES"
        fi
        start_helper "$WORKSPACE_DIR/src/yolo" "multi_solving.sh" "坐标计算" "$NUM_DRONES"

        echo "启动云台控制（俯视角由 SDF 深度相机 pose 提供，云台 pitch 置 0）..."
        start_helper "$WORKSPACE_DIR/src/gimbal" "multi_gimbal_control.sh" "云台控制" "$NUM_DRONES"

        echo "启动 tracking 追踪节点..."
        roslaunch tracking tracking.launch num_drones:="$NUM_DRONES" &
        TRACKING_PID=$!
    else
        echo "START_PERCEPTION=0：跳过 YOLO / tracking（只验证底层 + 建图/区域任务）。"
    fi
fi

if [ "$RUN_MODE" = "mission" ] || [ "$RUN_MODE" = "full" ]; then
    # down_resume.launch 里 YOLO / 坐标解算默认是注释状态，这里统一补上，
    # 保证 mission/full 模式也有人物检测和人物模型位置解算。
    echo "启动 YOLO 检测..."
    if [ -x "$WORKSPACE_DIR/src/yolo/multi_yolo_detecting_gpu.sh" ]; then
        start_helper "$WORKSPACE_DIR/src/yolo" "multi_yolo_detecting_gpu.sh" "YOLO 检测(GPU)" "$NUM_DRONES"
    else
        start_helper "$WORKSPACE_DIR/src/yolo" "multi_yolo_detecting.sh" "YOLO 检测(CPU)" "$NUM_DRONES"
    fi

    echo "启动人物位置坐标解算..."
    start_helper "$WORKSPACE_DIR/src/yolo" "multi_solving.sh" "坐标计算" "$NUM_DRONES"

    echo "启动云台控制（俯视角由 SDF 深度相机 pose 提供，云台 pitch 置 0）..."
    start_helper "$WORKSPACE_DIR/src/gimbal" "multi_gimbal_control.sh" "云台控制" "$NUM_DRONES"

    echo "启动 down_resume 任务节点..."
    roslaunch look_up down_resume.launch num_drones:="$NUM_DRONES" mission_filename:="$MISSION_FILE"
else
    # sim 模式：只监控 gzserver 状态，崩溃后不自动重启（按需手动重新启动）。
    echo "基础仿真已就绪。监控中（gzserver 崩溃后不自动重启，脚本将退出）。"
    while true; do
        if ! pgrep -x gzserver >/dev/null 2>&1; then
            echo "⚠ gzserver 已退出，已关闭自动重启，脚本退出。"
            exit 0
        fi
        sleep 3
    done
fi
