#!/bin/bash
# ============================================================================
# start_official_judge.sh —— 「官方链路」模拟裁判端启动器
#
# 完全按官方《RoboCup 无人机仿真赛双机通信官方指导文档》的主机流程：
#     roscore                      ← 必须单独先起（文档 §3.1；也是本机实测的硬要求）
#     roslaunch robocup.launch     ← 官方那份：6×typhoon_h480 + robocup.world(6 人物)
#     6× control_actor.py          ← 人物运动（官方 robocup/control_actor.py）
#     score_cal.py                 ← 裁判评分（发 /score /time_usage /left_actors /find_actor_i）
#
# 算法在另一台机器上通过 ROS 组网加入，只订阅话题。
#
# 用法：
#   bash start_official_judge.sh [--display=headless|gui] [--ros-ip=IP]
#                                [--no-actors] [--no-score] [--no-mirror]
#                                [--round-limit=SEC] [--inspect-only]
# 默认：headless、起 actors、起 score、round-limit=600s
#
# 前置：先跑过一次 tools/setup_official_chain.sh（把 XTDrone sitl_config 铺进 PX4 树）
# ============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PX4_DIR="${PX4_DIR:-$HOME/robocup_fly/PX4_Firmware}"
GZ="$PX4_DIR/Tools/sitl_gazebo"
DISPLAY_MODE="headless"
ROS_IP_ARG=""
MASTER_PORT="11311"
WITH_ACTORS=1
WITH_SCORE=1
WITH_MIRROR=0
ROUND_LIMIT="600"
# 地图：random = 每轮随机生成（默认，比赛口径）；stock = 用官方自带 robocup.world
MAP_MODE="random"
SEED=""
# 机型：official = 官方 typhoon_h480（只有 CGO3 单目，最忠实）
#       zzufly   = 使用我们已申报的传感器组合（2D 激光 + Realsense 深度）spawn 无人机，
#                  但**命名空间/模型名/model 数量/world/actor/裁判全部保持官方口径**
#                  （算法栈依赖 /typhoon_h480_i/scan 与 /typhoon_h480_i/realsense/...）
MODEL_MODE="official"
PER_DRONE_SDF=0
NAN_GUARD=0
SENSORS_ARG=""
STRICT_SENSORS=0
ALLOW_SPEC=""
REFEREE_LIMIT=""
WITH_CHEAT_MONITOR=1
ACTOR_IN_CITY=0
INSPECT_ONLY=0
NUM_DRONES=6

while [ $# -gt 0 ]; do
    case "$1" in
        --display=*)      DISPLAY_MODE="${1#*=}" ;;
        --ros-ip=*)       ROS_IP_ARG="${1#*=}" ;;
        --master-port=*)  MASTER_PORT="${1#*=}" ;;
        --actor-in-city)  ACTOR_IN_CITY=1 ;;   # 人物边界收进城区（打补丁副本，官方文件不动）
        --no-actors)      WITH_ACTORS=0 ;;
        --score)          WITH_SCORE=1 ;;   # 显式开启（默认就是开）
        --nan-guard)      NAN_GUARD=1 ;;    # 给官方裁判打 NaN 报点防护补丁（见 score_cal.py:119）
        --referee-limit=*) REFEREE_LIMIT="${1#*=}" ;;  # 改官方裁判任务超时（默认 600s）
        --strict-sensors) STRICT_SENSORS=1 ;;
        --allow-spec=*)   ALLOW_SPEC="${1#*=}" ;;   # 已裁定放行的规格项（逗号分隔）   # 传感器规格与官方基准不一致即拒绝开赛
        --sensors=*)      SENSORS_ARG="${1#*=}" ;;   # 申报传感器：名字或七位 0/1，如 mono,laser2d,gimbal
        --no-cheat-monitor) WITH_CHEAT_MONITOR=0 ;;
        --no-score)       WITH_SCORE=0 ;;
        --no-mirror)      WITH_MIRROR=0 ;;
        --mirror)         WITH_MIRROR=1 ;;
        --round-limit=*)  ROUND_LIMIT="${1#*=}" ;;
        --map=*)          MAP_MODE="${1#*=}" ;;
        --seed=*)         SEED="${1#*=}" ;;
        --model=*)        MODEL_MODE="${1#*=}" ;;
        --inspect-only)   INSPECT_ONLY=1 ;;
        -h|--help)        sed -n '2,22p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
    shift
done

[ -f /opt/ros/noetic/setup.bash ] && source /opt/ros/noetic/setup.bash
source "$SCRIPT_DIR/devel/setup.bash" 2>/dev/null || true

# ---------- 环境（官方链路） ----------
export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:+$ROS_PACKAGE_PATH:}$PX4_DIR:$GZ"
export GAZEBO_MODEL_PATH="$GZ/models:$HOME/robocup_fly/gazebo_models:$HOME/.gazebo/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
# 关键：v1.11 的 Gazebo 插件构建在 Tools/sitl_gazebo/build/，不是
# build/px4_sitl_default/build_gazebo/（后者存在但是空的）。指错会导致
# libgazebo_mavlink_interface.so 加载失败 → PX4 永远卡在
# "Waiting for simulator to accept connection on TCP port 4560"。
# （自研链路没这问题，因为它 source 了 PX4 的 setup_gazebo.bash。）
export GAZEBO_PLUGIN_PATH="$SCRIPT_DIR/devel/lib:$PX4_DIR/Tools/sitl_gazebo/build:$PX4_DIR/build/px4_sitl_default/build_gazebo:$SCRIPT_DIR/gazebo_map_assets/plugins${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
export LD_LIBRARY_PATH="$PX4_DIR/Tools/sitl_gazebo/build:$SCRIPT_DIR/devel/lib:${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export GAZEBO_RESOURCE_PATH="$GZ/worlds${GAZEBO_RESOURCE_PATH:+:$GAZEBO_RESOURCE_PATH}"
export GAZEBO_MODEL_DATABASE_URI=""
export PYTHONUNBUFFERED=1

# ---------- ROS 组网 ----------
if [ -z "$ROS_IP_ARG" ]; then
    ROS_IP_ARG="$(hostname -I 2>/dev/null | awk '{print $1}')"
fi
[ -n "$ROS_IP_ARG" ] || { echo "错误：无法探测局域网 IP，请用 --ros-ip= 指定" >&2; exit 2; }
export ROS_IP="$ROS_IP_ARG"
unset ROS_HOSTNAME
export ROS_MASTER_URI="http://${ROS_IP_ARG}:${MASTER_PORT}"

ROUND_DIR="$SCRIPT_DIR/logs/official_round_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$ROUND_DIR"
ACTOR_PIDS=()

cleanup_all() {
    # 幂等：INT/TERM 与 EXIT 都可能触发，避免重复清理与重复报分。
    [ "${_CLEANED:-0}" = "1" ] && return 0
    _CLEANED=1
    echo ">>> 退出清理 ..."
    # 官方 score_cal.py 不认 SCORE_OUTPUT_PATH、也不写任何文件，只把
    # "score: <值>" 打到 stdout（我们的 referee.log）。这里回收最终分数落盘，
    # 保证每轮都有 score.txt 存档（否则换用官方原版后分数就丢了）。
    if [ "${WITH_SCORE:-1}" = "1" ] && [ ! -s "${ROUND_DIR:-}/score.txt" ] \
            && [ -f "${ROUND_DIR:-}/referee.log" ]; then
        FINAL_SCORE=$(grep -a '^score:' "$ROUND_DIR/referee.log" | tail -1 | awk '{print $2}')
        if [ -n "$FINAL_SCORE" ]; then
            printf '%s\n' "$FINAL_SCORE" > "$ROUND_DIR/score.txt"
            echo "    已从 referee.log 回收最终分数：$FINAL_SCORE → $ROUND_DIR/score.txt"
        fi
    fi
    [ -n "${CHEAT_PID:-}" ] && kill "$CHEAT_PID" 2>/dev/null || true
    for p in "${ACTOR_PIDS[@]:-}"; do kill "$p" 2>/dev/null || true; done
    for p in "${DRONE_PIDS[@]:-}"; do kill -TERM "-$p" 2>/dev/null || true; done
    [ -n "${GZ_PID:-}" ] && kill -TERM "-$GZ_PID" 2>/dev/null || true
    echo "    提示：gzserver / PX4 / roscore 仍在运行；要清场用 python3 tools/kill_stale_sim.py"
}
trap 'cleanup_all' INT TERM EXIT

echo "================================================"
echo " 官方链路 · 模拟裁判端"
echo " PX4_DIR   = $PX4_DIR"
echo " display   = $DISPLAY_MODE    actors=$WITH_ACTORS score=$WITH_SCORE"
echo " ROS_MASTER_URI = $ROS_MASTER_URI"
echo " ROS_IP         = $ROS_IP"
echo " 轮次目录  = $ROUND_DIR"
echo "------------------------------------------------"
echo " 算法机（另一台电脑）这样加入："
echo "   export ROS_MASTER_URI=$ROS_MASTER_URI"
echo "   export ROS_IP=<那台机器的局域网IP>"
echo "   rostopic list                 # 应看到 /typhoon_h480_0/... 与 /score"
echo "   bash start_sim_only.sh --master-uri=$ROS_MASTER_URI --num-drones=6 --no-rviz"
echo "================================================"

# ---------- [0] 前置检查 ----------
for p in "$PX4_DIR/px4" "$PX4_DIR/launch/robocup.launch" "$GZ/worlds/robocup.world"; do
    [ -e "$p" ] || { echo "错误：缺少 $p —— 请先运行 bash tools/setup_official_chain.sh" >&2; exit 1; }
done
if ! rospack find px4 >/dev/null 2>&1 || ! rospack find mavlink_sitl_gazebo >/dev/null 2>&1; then
    echo "错误：rospack 找不到 px4 / mavlink_sitl_gazebo 包（ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH）" >&2
    exit 1
fi
echo "[0/5] 前置检查通过（px4 包、mavlink_sitl_gazebo 包、robocup.launch、robocup.world）"

# ---------- [0b] 机型选择 ----------
# zzufly：只换 SDF（加 2D 激光 + Realsense 深度），vehicle 名保持 typhoon_h480，
# 这样模型名/命名空间仍是 typhoon_h480_0..5，算法栈无需改任何话题名。
if [ "$MODEL_MODE" = "zzufly" ]; then
    SDF_NAME="typhoon_h480_zzufly"
    VEHICLE_NAME="typhoon_h480"
    ZZUFLY_DIR="$SCRIPT_DIR/typhoon_h480_zzufly"
    [ -f "$ZZUFLY_DIR/typhoon_h480_zzufly.sdf" ] || {
        echo "错误：缺少 $ZZUFLY_DIR/typhoon_h480_zzufly.sdf" >&2; exit 1; }
    MODEL_LINK="$GZ/models/typhoon_h480_zzufly"
    if [ ! -e "$MODEL_LINK" ]; then
        ln -s "$ZZUFLY_DIR" "$MODEL_LINK"
        echo "[0b] 已把传感器机型链接进 PX4 模型库：$MODEL_LINK -> $ZZUFLY_DIR"
    else
        echo "[0b] 传感器机型链接已存在：$MODEL_LINK"
    fi
    echo "      机型：sdf=$SDF_NAME（含 2D 激光 + Realsense 深度），命名空间仍为 ${VEHICLE_NAME}_0..5"

    # 逐架 SDF：自研链路用"逐架 SDF"能 6/6 连上 PX4，而通用 SDF 在官方链路里
    # mavlink 插件不监听 4560。这里为 6 架各建一个模型目录（meshes 用软链，不复制）。
    for _i in 0 1 2 3 4 5; do
        _d="$GZ/models/${SDF_NAME}_$_i"
        mkdir -p "$_d"
        for _f in "$ZZUFLY_DIR"/*; do
            _b="$(basename "$_f")"
            [ "$_b" = "${SDF_NAME}.sdf" ] && continue
            ln -sfn "$_f" "$_d/$_b"
        done
        if [ -f "$ZZUFLY_DIR/typhoon_h480_$_i.sdf" ]; then
            cp -f "$ZZUFLY_DIR/typhoon_h480_$_i.sdf" "$_d/${SDF_NAME}_$_i.sdf"
        else
            cp -f "$ZZUFLY_DIR/${SDF_NAME}.sdf" "$_d/${SDF_NAME}_$_i.sdf"
        fi
        cat > "$_d/model.config" <<EOFCFG
<?xml version="1.0"?>
<model>
  <name>${SDF_NAME}_$_i</name>
  <version>1.0</version>
  <sdf version="1.5">${SDF_NAME}_$_i.sdf</sdf>
  <author><name>zzu_fly</name><email>zzu_fly@example.com</email></author>
  <description>${SDF_NAME} per-drone SDF (sensors as declared)</description>
</model>
EOFCFG
    done
    PER_DRONE_SDF=1
    echo "      已生成 6 个逐架模型目录：$GZ/models/${SDF_NAME}_0..5"
else
    SDF_NAME="typhoon_h480"
    VEHICLE_NAME="typhoon_h480"
    echo "[0b] 机型：官方 $SDF_NAME（只有 CGO3 单目相机；算法栈的 /scan 与深度话题不存在）"
fi

# ---------- [0c] 模型预检（提交模型上线前的断言）----------
# 触发背景：提交文档自述"vehicle=typhoon_h480、PX4 用官方安装"，但实际需要
# PX4_SIM_MODEL=<新机型> + ROMFS 里存在 <编号>_<新机型>(.post)；照文档只 cp 模型
# 会让 6 架 SITL 全起不来。这里把"部署即失败"提前到开赛前拦下。
echo "[0c] 模型预检"
SDF_PATH="$GZ/models/$SDF_NAME/$SDF_NAME.sdf"
if [ ! -f "$SDF_PATH" ]; then
    echo "错误：找不到模型 SDF：$SDF_PATH" >&2
    echo "      提交模型请先拷进 \$PX4/Tools/sitl_gazebo/models/<模型名>/<模型名>.sdf" >&2
    exit 1
fi
echo "      SDF: $SDF_PATH  ($(wc -c < "$SDF_PATH") B, sha256=$(sha256sum "$SDF_PATH" | cut -c1-12))"
python3 -c "import xml.dom.minidom,sys; xml.dom.minidom.parse(sys.argv[1])" "$SDF_PATH" \
    || { echo "错误：SDF 不是合法 XML：$SDF_PATH" >&2; exit 1; }

ROMFS_POSIX="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix"
if [ ! -d "$ROMFS_POSIX" ]; then
    echo "错误：PX4 ROMFS 目录不存在：$ROMFS_POSIX（PX4_DIR=$PX4_DIR）" >&2; exit 1
fi
AIRFRAME=$(ls "$ROMFS_POSIX" 2>/dev/null | grep -E "^[0-9]+_${VEHICLE_NAME}$" | head -1)
AIRFRAME_POST=$(ls "$ROMFS_POSIX" 2>/dev/null | grep -E "^[0-9]+_${VEHICLE_NAME}\.post$" | head -1)
if [ -z "$AIRFRAME" ]; then
    echo "错误：ROMFS 里没有机型定义 <编号>_${VEHICLE_NAME}（目录 $ROMFS_POSIX）" >&2
    echo "      PX4_SIM_MODEL=${VEHICLE_NAME} 时 rcS 会报 'Error: Unknown model' 并以 256 退出。" >&2
    echo "      若用新机型名，请按官方流程把 airframe 与 .post 放进 ROMFS 并申报。" >&2
    exit 1
fi
echo "      机型定义: $AIRFRAME${AIRFRAME_POST:+ + $AIRFRAME_POST}"

# ---------- 传感器规格门禁（赛前检查，可硬拦）----------
# 规则："相机的视场角、分辨率、焦距，激光雷达的扫描范围等参数必须与 XTDrone
# 平台自带传感器保持一致，不能修改"（只有安装位姿自由）。这里用
# tools/sdf_sensor_audit.py 对提交件逐项比对官方基准并留证；
# 加 --strict-sensors 时只要有不一致项就拒绝开赛。
# 逐架模式下 6 架实际加载的是 <SDF>_<i>.sdf（由编号 SDF 生成），
# 审计必须看"真正被加载的那一份"，否则会审到通用件（内容可能不同）→ 结论失真。
if [ "${PER_DRONE_SDF:-0}" = "1" ] && [ -f "$GZ/models/${SDF_NAME}_0/${SDF_NAME}_0.sdf" ]; then
    SDF_PATH="$GZ/models/${SDF_NAME}_0/${SDF_NAME}_0.sdf"
    echo "      审计对象：逐架模式取 0 号实际加载件 $SDF_PATH"
fi

AUDIT="$SCRIPT_DIR/tools/sdf_sensor_audit.py"
if [ -f "$AUDIT" ]; then
    AUDIT_OUT="$ROUND_DIR/sdf_sensor_audit.txt"
    python3 "$AUDIT" "$SDF_PATH" > "$AUDIT_OUT" 2>&1 || true
    PAT=""
    if [ -n "${ALLOW_SPEC:-}" ]; then PAT="$(echo "$ALLOW_SPEC" | tr ',' '|')"; fi
    if [ -n "$PAT" ]; then
        SPEC_ALLOWED=$(grep -a '🔴' "$AUDIT_OUT" 2>/dev/null | grep -acE "$PAT" || echo 0)
        SPEC_BAD=$(grep -a '🔴' "$AUDIT_OUT" 2>/dev/null | grep -avE "$PAT" | wc -l)
    else
        SPEC_ALLOWED=0
        SPEC_BAD=$(grep -c '🔴' "$AUDIT_OUT" 2>/dev/null || echo 0)
    fi
    echo "      传感器规格核对：不一致 ${SPEC_BAD} 项（另 ${SPEC_ALLOWED} 项按 --allow-spec 放行）（明细 $AUDIT_OUT）"
    [ -n "$PAT" ] && echo "      放行规则：$ALLOW_SPEC"
    if [ "${SPEC_BAD:-0}" != "0" ]; then
        if [ -n "$PAT" ]; then
            grep -a '🔴' "$AUDIT_OUT" | grep -avE "$PAT" | head -10 | sed 's/^/        /'
        else
            grep -a '🔴' "$AUDIT_OUT" | head -10 | sed 's/^/        /'
        fi
    fi
    # 官方 stock 机型只作信息展示、不硬拦：audit 的基准是 realsense_camera /
    # typhoon_h480_2d_lidar，而官方 robocup 机型本身是 CGO3 单目（FOV 2.0/640x360），
    # 本来就不等于那两个基准，硬拦会把官方口径也拦掉。strict 只用于"提交模型"。
    if [ "$MODEL_MODE" = "official" ] && [ "$SPEC_BAD" != "0" ]; then
        echo "      （官方 stock 机型：以上差异源于 audit 基准选择，仅供参考，不作为拒绝依据）"
    fi
    if [ "${STRICT_SENSORS:-0}" = "1" ] && [ "$SPEC_BAD" != "0" ] && [ "$MODEL_MODE" != "official" ]; then
        {
            echo "错误：--strict-sensors 已启用，但该模型有 $SPEC_BAD 项参数与 XTDrone 官方基准不一致。"
            echo "      规则要求「参数必须与 XTDrone 平台自带传感器保持一致，不能修改」。拒绝开赛。"
            echo "      逐项明细：$AUDIT_OUT"
            echo "      若组委会已书面放行其中某几项，请改用不带 --strict-sensors 的方式并留档。"
        } >&2
        exit 1
    fi
else
    echo "      提示：缺 tools/sdf_sensor_audit.py，跳过传感器规格核对" >&2
fi

# 端口一致性：官方单机 block 的 xmlstarlet 只改 mavlink_tcp_port，UDP 只能靠 SDF。
# 通用（非逐架）SDF 里写死的 UDP 端口会让 6 架共用同一端口 → 冲突。这里只告警不拦。
SDF_UDP=$(grep -oE '<mavlink_udp_port>[0-9]+' "$SDF_PATH" 2>/dev/null | head -1 | grep -oE '[0-9]+' || true)
SDF_TCP=$(grep -oE '<mavlink_tcp_port>[0-9]+' "$SDF_PATH" 2>/dev/null | head -1 | grep -oE '[0-9]+' || true)
echo "      SDF 声明端口: tcp=${SDF_TCP:-未声明(launch 注入 4560+i)}  udp=${SDF_UDP:-未声明}"
if [ -n "$SDF_UDP" ] && [ "$SDF_UDP" = "18570" ]; then
    echo "      ⚠️ 通用 SDF 写死 udp=18570：6 架会共用同一端口（应逐架 18570+i）。" >&2
fi

# ---------- [0d] 传感器费用参数（显式入参 + 留证）----------
# 裁判软件用 7 个私有参数决定扣费：mono 500 / stereo 1000 / laser1d 200 /
# laser2d 1000 / laser3d 2000 / gimbal 200 / bino_cam 1000，求和 ×0.003。
# 官方默认 1/0/0/1/0/1/0 = 1700 → 扣 5.1 分；过去没有任何 launch 显式传参，
# 属隐式契约。这里把申报集合变成显式入参并留证（"赛前检查传感器参数"的依据）。
SENSOR_KEYS=(mono_cam stereo_cam laser1d laser2d laser3d gimbal bino_cam)
SENSOR_PRICE=(500 1000 200 1000 2000 200 1000)
SENSOR_VALS=(1 0 0 1 0 1 0)          # 官方默认（1 架的量）
SENSOR_NOTE="官方默认"
if [ -n "$SENSORS_ARG" ]; then
    if [ "$SENSORS_ARG" = "zzu" ]; then
        # 本队申报口径：6 架 × (RGB-D 1000 + 2D 激光 1000 + 云台 200) = 13200 → 扣 39.6
        SENSOR_VALS=(0 6 0 6 0 6 0); SENSOR_NOTE="本队申报（zzu 预设）"
    else
        IFS=',' read -r -a _want <<< "$SENSORS_ARG"
        if [ "${#_want[@]}" -eq 7 ] && [[ "${_want[0]}" =~ ^[0-9]+$ ]]; then
            SENSOR_VALS=("${_want[@]}")                  # 七个数（计数，非仅 0/1）
            SENSOR_NOTE="显式七元组"
        else
            SENSOR_VALS=(0 0 0 0 0 0 0)                  # 按名字[:计数]
            SENSOR_NOTE="按名称"
            for _w in "${_want[@]}"; do
                _w="$(echo "$_w" | tr -d ' ')"
                _n="${_w##*:}"; _name="${_w%%:*}"
                [[ "$_n" =~ ^[0-9]+$ ]] || _n=1
                case "$_name" in
                    mono|mono_cam)       SENSOR_VALS[0]=$_n ;;
                    stereo|stereo_cam)   SENSOR_VALS[1]=$_n ;;
                    laser1d)             SENSOR_VALS[2]=$_n ;;
                    laser2d|lidar2d)     SENSOR_VALS[3]=$_n ;;
                    laser3d)             SENSOR_VALS[4]=$_n ;;
                    gimbal)              SENSOR_VALS[5]=$_n ;;
                    bino|bino_cam)       SENSOR_VALS[6]=$_n ;;
                    '')                  ;;
                    *) echo "警告：未知传感器名 '$_name'（可用 mono/stereo/laser1d/laser2d/laser3d/gimbal/bino，可带 :数量）" >&2 ;;
                esac
            done
        fi
    fi
fi
SENSOR_COST=0; SENSOR_LIST=""
for i in "${!SENSOR_KEYS[@]}"; do
    _v="${SENSOR_VALS[$i]}"
    [ "$_v" = "0" ] && continue
    SENSOR_COST=$((SENSOR_COST + SENSOR_PRICE[i] * _v))
    if [ "$_v" = "1" ]; then
        SENSOR_LIST="${SENSOR_LIST:+$SENSOR_LIST,}${SENSOR_KEYS[$i]}"
    else
        SENSOR_LIST="${SENSOR_LIST:+$SENSOR_LIST,}${SENSOR_KEYS[$i]}x${_v}"
    fi
done
SENSOR_DEDUCT=$(python3 -c "print('%.3f' % ($SENSOR_COST * 3e-3))")
echo "[0d] 传感器申报（${SENSOR_NOTE}）：${SENSOR_LIST:-（无）}"
echo "      参数 mono/stereo/laser1d/laser2d/laser3d/gimbal/bino = ${SENSOR_VALS[*]}"
echo "      费用合计 $SENSOR_COST → 每轮扣 $SENSOR_DEDUCT 分"
{
    echo "sensors=$SENSOR_LIST"
    echo "params=${SENSOR_KEYS[*]} = ${SENSOR_VALS[*]}"
    echo "cost=$SENSOR_COST"
    echo "deduction_per_round=$SENSOR_DEDUCT"
    echo "recorded_at=$(date '+%F %T')"
    echo "sdf=$SDF_PATH"
    echo "sdf_sha256=$(sha256sum "$SDF_PATH" | awk '{print $1}')"
    echo "airframe=$AIRFRAME"
    echo "spec_allow=$ALLOW_SPEC"
    echo "spec_bad=$SPEC_BAD"
    echo "spec_allowed=$SPEC_ALLOWED"
} > "$ROUND_DIR/sensors.txt"
echo "      已留证：$ROUND_DIR/sensors.txt"
sensors_args=()
for i in "${!SENSOR_KEYS[@]}"; do
    sensors_args+=("_${SENSOR_KEYS[$i]}:=${SENSOR_VALS[$i]}")
done

if [ "$INSPECT_ONLY" = "1" ]; then
    echo ">>> --inspect-only：只做检查，不启动任何进程。"
    roslaunch --nodes px4 robocup.launch
    exit 0
fi

# ---------- [1] 清残留 ----------
echo "[1/5] 清理残留仿真进程"
NO_PREFLIGHT_CLEAN="${NO_PREFLIGHT_CLEAN:-0}"
if [ "$NO_PREFLIGHT_CLEAN" != "1" ]; then
    python3 "$SCRIPT_DIR/tools/kill_stale_sim.py" || { echo "错误：残留未清净" >&2; exit 1; }
fi

# ---------- [2] 显式先起 roscore ----------
echo "[2/5] 启动 ROS master（roscore -p $MASTER_PORT，监听 0.0.0.0）"
if timeout 5 bash -c "cat < /dev/null > /dev/tcp/${ROS_IP_ARG}/${MASTER_PORT}" 2>/dev/null; then
    echo "      已有 master，复用 $ROS_MASTER_URI"
else
    setsid roscore -p "$MASTER_PORT" >"$ROUND_DIR/roscore.log" 2>&1 &
    ok=0
    for _ in $(seq 1 30); do
        if timeout 2 bash -c "cat < /dev/null > /dev/tcp/${ROS_IP_ARG}/${MASTER_PORT}" 2>/dev/null; then ok=1; break; fi
        sleep 1
    done
    [ "$ok" = "1" ] || { echo "错误：master 未就绪（见 $ROUND_DIR/roscore.log）" >&2; exit 1; }
    echo "      master 就绪"
fi

# ---------- [2b] 生成随机地图（官方随机化算法） ----------
# 用 robocup_referee/referee.py gen-map：它是官方 robocup/map_generator.py 的
# 逐字节等价移植（selftest_fidelity 6 种子 × 5 产物全一致），并带 --fix-known-bugs
# 修掉官方三个缺陷。官方原件在本机实测会产出 0 字节 house.world（回退 base world，
# 等于房子完全没随机化），所以这里默认走修好的版本。
if [ "$MAP_MODE" = "random" ]; then
    MAP_DIR="$ROUND_DIR/mapgen"
    SEED_ARG=""
    [ -n "$SEED" ] && SEED_ARG="--seed=$SEED"
    echo "[2b] 生成随机地图（referee.py gen-map --fix-known-bugs，seed=${SEED:-random}）"
    if python3 "$SCRIPT_DIR/robocup_referee/referee.py" gen-map --out "$MAP_DIR" --fix-known-bugs $SEED_ARG \
            >"$ROUND_DIR/mapgen.log" 2>&1; then
        cp -f "$MAP_DIR/robocup.world" "$GZ/worlds/robocup.world"
        cp -f "$MAP_DIR/black_box.txt" "$MAP_DIR/obstacle.txt" "$ROUND_DIR/" 2>/dev/null || true
        grep -a 'seed=' "$ROUND_DIR/mapgen.log" | tail -1 | sed 's/^/      /'
        echo "      已写入 $GZ/worlds/robocup.world（人物地图复制到 $ROUND_DIR）"
    else
        echo "错误：随机地图生成失败（见 $ROUND_DIR/mapgen.log）" >&2
        tail -10 "$ROUND_DIR/mapgen.log" >&2 || true
        exit 1
    fi
elif [ "$MAP_MODE" = "official" ]; then
    # 逐字复刻官方：直接跑 Gitee robocup/map_generator.py（**不打补丁、保留其缺陷**）。
    # 已知缺陷：create_point 重试耗尽后回退 base world，house.world 为 0 字节 →
    # 房子完全不随机化。这里如实报告，不修。
    MAP_DIR="$ROUND_DIR/mapgen"; mkdir -p "$MAP_DIR"
    echo "[2b] 生成随机地图（**官方原件** map_generator.py，含其已知缺陷，seed=${SEED:-random}）"
    for f in base1.world base2.world base3.world rover_static.world map_generator.py; do
        [ -f "$SCRIPT_DIR/XTDrone/robocup/$f" ] && cp -f "$SCRIPT_DIR/XTDrone/robocup/$f" "$MAP_DIR/"
    done
    # 官方脚本把 robocup.world 写死到 ~/PX4_Firmware/Tools/sitl_gazebo/worlds/
    OFFICIAL_OUT="$(python3 -c 'import os;print(os.path.expanduser("~/PX4_Firmware/Tools/sitl_gazebo/worlds/robocup.world"))')"
    rm -f "$OFFICIAL_OUT" 2>/dev/null || true
    (
        cd "$MAP_DIR" || exit 1
        if [ -n "$SEED" ]; then
            # 官方脚本用全局 random、不接 seed；这里先 seed 再 exec，保证可复现
            python3 -c "import random;random.seed($SEED);exec(open('map_generator.py').read())"
        else
            python3 map_generator.py
        fi
    ) >"$ROUND_DIR/mapgen.log" 2>&1 || { echo "错误：官方 map_generator.py 失败（见 mapgen.log）" >&2; exit 1; }
    HOUSESZ=$(wc -c < "$MAP_DIR/house.world" 2>/dev/null || echo 0)
    if [ ! -s "$OFFICIAL_OUT" ]; then
        echo "错误：官方 map_generator.py 产出了 **0 字节** 的 robocup.world —— 这是它的已知缺陷" >&2
        echo "      （create_point 重试耗尽 → 回退 base world → house.world 为 0 字节 → 组装出空世界）。" >&2
        echo "      空世界会让 Gazebo 直接起不来，故拒绝使用。要正常跑一轮请用 --map=random。" >&2
        echo "      官方原文件与日志保留在：$MAP_DIR 与 $ROUND_DIR/mapgen.log" >&2
        exit 1
    fi
    cp -f "$OFFICIAL_OUT" "$GZ/worlds/robocup.world"
    cp -f "$MAP_DIR/black_box.txt" "$MAP_DIR/obstacle.txt" "$ROUND_DIR/" 2>/dev/null || true
    echo "      house.world = ${HOUSESZ} B$( [ "$HOUSESZ" -lt 1000 ] && echo '  ⚠️ 触发了官方已知缺陷（房子未随机化，回退 base world）' )"
    grep -aE 'Base world|Finish|generated' "$ROUND_DIR/mapgen.log" | tail -3 | sed 's/^/      /'
    echo "      已写入 $GZ/worlds/robocup.world"
else
    echo "[2b] 使用官方自带 robocup.world（--map=stock，不做随机化）"
fi

# ---------- [3] 只起 Gazebo 世界，等它真正加载完 ----------
GUI_FLAG="false"; [ "$DISPLAY_MODE" = "gui" ] && GUI_FLAG="true"
WORLD_FILE="$GZ/worlds/robocup.world"
echo "[3/5] 起 Gazebo 世界（只起世界，不 spawn 无人机）：$WORLD_FILE"
PYTHONUNBUFFERED=1 setsid roslaunch "$SCRIPT_DIR/tools/launch/robocup_judge_gazebo.launch" \
    world:="$WORLD_FILE" gui:="$GUI_FLAG" \
    >"$ROUND_DIR/gazebo.log" 2>&1 &
GZ_PID=$!
echo "      gzserver roslaunch PID=$GZ_PID  日志 $ROUND_DIR/gazebo.log"

echo "      等待世界加载完成（模型数连续 3 次稳定；最长 600s）"
WORLD_WAIT=0; STABLE=0; LAST=-1
while [ "$WORLD_WAIT" -lt 600 ]; do
    if ! kill -0 "$GZ_PID" 2>/dev/null; then
        echo "错误：Gazebo 启动进程已退出，见 $ROUND_DIR/gazebo.log" >&2
        tail -20 "$ROUND_DIR/gazebo.log" >&2 || true
        exit 1
    fi
    N="$(timeout 25 python3 "$SCRIPT_DIR/tools/world_probe.py" 10 2>/dev/null | tail -1)"
    N="${N:-0}"
    if [ "$N" -gt 1 ] && [ "$N" -eq "$LAST" ]; then
        STABLE=$((STABLE + 1))
    else
        STABLE=0
    fi
    LAST="$N"
    if [ "$STABLE" -ge 3 ]; then
        echo "      世界就绪：$N 个模型（连续 3 次稳定）"
        break
    fi
    if [ $((WORLD_WAIT % 30)) -eq 0 ]; then
        echo "      [${WORLD_WAIT}s] 当前模型数=$N（稳定计数 $STABLE/3）"
    fi
    sleep 10; WORLD_WAIT=$((WORLD_WAIT + 10))
done
if [ "$STABLE" -lt 3 ]; then
    echo "错误：600s 内世界未就绪（最后模型数=$LAST）" >&2
    exit 1
fi

# zzu_fly: SITL 无遥控/无地面站心跳 ⇒ PX4 的 rc_signal_lost/data_link_lost 恒为真，
# 一旦 failsafe 置位就永久封锁模式切换（OFFBOARD 永远切不过去）。
# 本地 PX4 树的 commander 打了开关（Commander.cpp: zzu_fly_skip_rc_failsafe），
# 这里默认打开；外部显式设 PX4_SKIP_RC_FAILSAFE=0 可关掉。
export PX4_SKIP_RC_FAILSAFE="${PX4_SKIP_RC_FAILSAFE:-1}"
echo "      PX4_SKIP_RC_FAILSAFE=$PX4_SKIP_RC_FAILSAFE（1=跳过无 RC/无数据链 failsafe，本地演练用）"

# ---------- [4] 逐架 spawn + MAVROS（官方单机 block） ----------
# 官方 robocup.launch 的 6 个出生点：
#   0:(0,-3) 1:(3,-3) 2:(0,0) 3:(3,0) 4:(0,3) 5:(3,3)
SPAWN_X=(-17 -14 -17 -14 -17 -14)
SPAWN_Y=(-3 -3 0 0 3 3)   # 组委会裁判机实际布局（见 docs/reference/official_judge_robocup.launch）
DRONE_PIDS=()
echo "[4/5] 逐架拉起 6 架（官方 single_vehicle_spawn_xtd + mavros，端口 2454i/3458i）"
CONNECTED=0
for i in 0 1 2 3 4 5; do
    X="${SPAWN_X[$i]}"; Y="${SPAWN_Y[$i]}"
    SDF_USE="$SDF_NAME"
    [ "${PER_DRONE_SDF:-0}" = "1" ] && SDF_USE="${SDF_NAME}_${i}"
    echo "      [drone$i] spawn @ ($X,$Y) ..."
    # PORT_RACE_RETRY: PX4 监听 3458i、MAVROS 监听 2454i；若上一轮或竞态残留占用，
    # PX4 会 bind failed 而整架连不上 → spawn 前先释放本架这两个端口。
    if command -v fuser >/dev/null 2>&1; then
        fuser -k -n udp "$((24540 + i))" >/dev/null 2>&1 || true
        fuser -k -n udp "$((34580 + i))" >/dev/null 2>&1 || true
    fi
    PYTHONUNBUFFERED=1 setsid roslaunch "$SCRIPT_DIR/tools/launch/robocup_judge_drone.launch" \
        id:="$i" x:="$X" y:="$Y" z:=1 \
        sdf:="$SDF_USE" vehicle:="$VEHICLE_NAME" \
        mavlink_tcp_port:="$((4560 + i))" \
        mavlink_udp_port:="$((14560 + i))" \
        fcu_url:="udp://:$((24540 + i))@localhost:$((34580 + i))" \
        tgt_system:="$((1 + i))" \
        >"$ROUND_DIR/drone_${i}.log" 2>&1 &
    DRONE_PIDS+=("$!")
    # 等这一架 connected（最多 90s）
    W=0; OK=0
    while [ "$W" -lt 90 ]; do
        if timeout 3 rostopic echo -n1 "/typhoon_h480_${i}/mavros/state" 2>/dev/null | grep -q 'connected: True'; then
            OK=1; break
        fi
        sleep 3; W=$((W + 3))
    done
    if [ "$OK" = "1" ]; then
        CONNECTED=$((CONNECTED + 1))
        echo "      [drone$i] MAVROS connected ✓（${W}s）"
    else
        echo "      [drone$i] 90s 内未 connected ✗（见 $ROUND_DIR/drone_${i}.log）" >&2
        grep -aiE 'error|died|Spawn status|failed' "$ROUND_DIR/drone_${i}.log" 2>/dev/null | tail -3 | sed 's/^/        /' >&2
    fi
    sleep 3   # 错峰，给 Gazebo 喘息
done
# PORT_RACE_RETRY: 对未连上的架再重试一遍（端口竞态是常见原因，重试通常能抢回）
for i in 0 1 2 3 4 5; do
    if timeout 3 rostopic echo -n1 "/typhoon_h480_${i}/mavros/state" 2>/dev/null | grep -q 'connected: True'; then
        continue
    fi
    X="${SPAWN_X[$i]}"; Y="${SPAWN_Y[$i]}"; SDF_USE="$SDF_NAME"
    [ "${PER_DRONE_SDF:-0}" = "1" ] && SDF_USE="${SDF_NAME}_${i}"
    echo "      [drone$i] 未连上 → 重试（清端口后重新 spawn）"
    if command -v fuser >/dev/null 2>&1; then
        fuser -k -n udp "$((24540 + i))" "$((34580 + i))" >/dev/null 2>&1 || true
    fi
    pkill -f "robocup_judge_drone.launch.*id:=${i}" 2>/dev/null || true
    sleep 3
    PYTHONUNBUFFERED=1 setsid roslaunch "$SCRIPT_DIR/tools/launch/robocup_judge_drone.launch" \
        id:="$i" x:="$X" y:="$Y" z:=1 \
        sdf:="$SDF_USE" vehicle:="$VEHICLE_NAME" \
        mavlink_tcp_port:="$((4560 + i))" \
        mavlink_udp_port:="$((14560 + i))" \
        fcu_url:="udp://:$((24540 + i))@localhost:$((34580 + i))" \
        tgt_system:="$((1 + i))" \
        >"$ROUND_DIR/drone_${i}_retry.log" 2>&1 &
    W=0
    while [ "$W" -lt 90 ]; do
        if timeout 3 rostopic echo -n1 "/typhoon_h480_${i}/mavros/state" 2>/dev/null | grep -q 'connected: True'; then
            CONNECTED=$((CONNECTED + 1)); echo "      [drone$i] 重试后 connected ✓（${W}s）"; break
        fi
        sleep 3; W=$((W + 3))
    done
    [ "$W" -ge 90 ] && echo "      [drone$i] 重试仍失败 ✗（见 $ROUND_DIR/drone_${i}_retry.log）" >&2
done

echo "[4/5] 结果：$CONNECTED/6 架 MAVROS connected"
if [ "$CONNECTED" -ne 6 ]; then
    echo "警告：未达 6/6，继续启动人物与裁判（可用 rostopic echo /typhoon_h480_i/mavros/state 复查）" >&2
fi

# ---------- [5] 人物 + 裁判 ----------
if [ "$WITH_ACTORS" = "1" ]; then
    ACTOR_PY="$SCRIPT_DIR/XTDrone/robocup/control_actor.py"
    if [ "${ACTOR_IN_CITY:-0}" = "1" ]; then
        # 官方 control_actor.py 的活动边界 x[-50,130] y[-60,60] 大于城区实测范围
        # x[-35,106] y[-45,77]，且逃逸路径不检查边界（实测人物停在 x=150）。
        # 在**副本**上收边界 + 发布前钳制；官方原文件不动，便于 A/B。
        ACTOR_PY="$ROUND_DIR/control_actor_city.py"
        python3 - "$SCRIPT_DIR/XTDrone/robocup/control_actor.py" "$ACTOR_PY" <<'PYACT'
import sys
src, dst = sys.argv[1], sys.argv[2]
s = open(src, encoding='utf-8').read()
for a, b in (('self.x_max = 130.0', 'self.x_max = 106.0'),
             ('self.x_min = -50.0', 'self.x_min = -35.0'),
             ('self.y_max = 60.0',  'self.y_max = 77.0'),
             ('self.y_min = -60.0', 'self.y_min = -45.0')):
    assert a in s, a
    s = s.replace(a, b, 1)
old = "self.cmd_pub.publish(self.build_motion_command())"
new = ("_cmd = self.build_motion_command()\n"
       "            _cmd.x = min(max(_cmd.x, self.x_min), self.x_max)\n"
       "            _cmd.y = min(max(_cmd.y, self.y_min), self.y_max)\n"
       "            self.cmd_pub.publish(_cmd)")
assert old in s, 'publish site'
s = s.replace(old, new, 1)
open(dst, 'w', encoding='utf-8').write(s)
PYACT
        echo "      人物边界已收进城区（x[-35,106] y[-45,77] + 发布前钳制；副本 $ACTOR_PY）"
    fi

    echo "[5/5] 启动 6 个人物控制（官方 control_actor.py）"
    ACTOR_DIR="$ROUND_DIR"
    # 优先用本轮随机地图生成的人物控制图（black_box/obstacle）；
    # 只有 --map=stock 时才会缺这两个文件，才回退到官方自带的那份。
    if [ -f "$ACTOR_DIR/black_box.txt" ] && [ -f "$ACTOR_DIR/obstacle.txt" ]; then
        echo "      使用本轮随机地图的人物地图（$ACTOR_DIR/black_box.txt, obstacle.txt）"
    else
        cp -f "$SCRIPT_DIR/XTDrone/robocup/black_box.txt" "$SCRIPT_DIR/XTDrone/robocup/obstacle.txt" "$ACTOR_DIR/" 2>/dev/null || true
        echo "      使用官方自带人物地图（未做随机化的回退）"
    fi
    for ID in 0 1 2 3 4 5; do
        (cd "$ACTOR_DIR" && PYTHONPATH="$SCRIPT_DIR/XTDrone/robocup:${PYTHONPATH:-}" exec setsid python3 -u "${ACTOR_PY:-$SCRIPT_DIR/XTDrone/robocup/control_actor.py}" "$ID") \
            >"$ACTOR_DIR/actor_${ID}.log" 2>&1 &
        ACTOR_PIDS+=("$!")
    done
    sleep 3
    for ID in 0 1 2 3 4 5; do
        if ! kill -0 "${ACTOR_PIDS[$ID]}" 2>/dev/null; then
            echo "警告：actor_$ID 启动失败（见 $ACTOR_DIR/actor_${ID}.log）" >&2
        fi
    done
    echo "      actor 日志：$ACTOR_DIR/actor_*.log"
else
    echo "[5/5] 跳过人物控制（--no-actors）"
fi

if [ "$WITH_SCORE" = "1" ]; then
    # 官方口径（老师 2026-10-10 明确）：裁判软件由组委会提供并启动，就是
    # Gitee XTDrone/robocup/score_cal.py。默认用官方原版
    # （公式 2580 − 用时 − 传感器×0.003 − 丢失无人机×30）。
    # 本仓库根目录的 score_cal.py 是团队改造版（按 2026 规则文本加了
    # 起飞×10 / 碰撞×30，与官方差 338 行），只用于本地对照 —— 用 OWN_SCORE=1 才启用。
    if [ "${OWN_SCORE:-0}" = "1" ]; then
        SCORE_PY="$SCRIPT_DIR/score_cal.py"
        echo "      计分用【本地改造版】score_cal.py（OWN_SCORE=1，仅自测对照）"
    elif [ -f "$SCRIPT_DIR/XTDrone/robocup/score_cal.py" ]; then
        SCORE_PY="$SCRIPT_DIR/XTDrone/robocup/score_cal.py"
        echo "      计分用【官方原版】XTDrone/robocup/score_cal.py（与组委会一致）"
    else
        SCORE_PY="$SCRIPT_DIR/score_cal.py"
        echo "      警告：缺官方 score_cal.py，回退本地改造版（见 §7 拉取命令）" >&2
    fi

    # --nan-guard：官方 score_cal.py:119 在报点坐标为 NaN 时，
    # distance_sq 也是 NaN，而 `NaN >= x` 恒为 False → 不触发 _reset_detection，
    # 非法坐标反而被计入"发现"（已向组委会反馈，他们回复"我们看一下"）。
    # 官方原文件保持不动，这里生成一份打了补丁的副本再跑。
    if [ "${NAN_GUARD:-0}" = "1" ]; then
        PATCHED="$ROUND_DIR/score_cal_nan_guard.py"
        if python3 - "$SCORE_PY" "$PATCHED" <<'PYEOF'
import sys
src, dst = sys.argv[1], sys.argv[2]
s = open(src, encoding='utf-8').read()
old = "        distance_sq = ((msg.x - position.x) ** 2 + (msg.y - position.y) ** 2)"
new = ("        if msg.x != msg.x or msg.y != msg.y:   # [nan-guard] NaN 坐标必须清零\n"
       "            _reset_detection(actor_id)\n"
       "            continue\n" + old)
if old not in s:
    print("未找到目标行，官方文件可能已改版", file=sys.stderr); sys.exit(1)
open(dst, 'w', encoding='utf-8').write(s.replace(old, new, 1))
PYEOF
        then
            SCORE_PY="$PATCHED"
            echo "      已启用 --nan-guard：用补丁副本计分（NaN 报点会被清零）"
        else
            echo "      警告：--nan-guard 打补丁失败，继续用官方原版（NaN 报点仍会被当成合格）" >&2
        fi
    else
        echo "      提示：官方原版对 NaN 报点不拦截（bug，已反馈）；加 --nan-guard 可启用本地防护"
    fi

    # --referee-limit=SEC：官方 MISSION_TIMEOUT=600（规则"每次尝试 600 秒"）。
    # 联调/长轮要跑更久时只改这一行常量，其余与官方逐字一致。
    if [ -n "${REFEREE_LIMIT:-}" ]; then
        PATCHED_LIM="$ROUND_DIR/score_cal_limit.py"
        if python3 - "$SCORE_PY" "$PATCHED_LIM" "$REFEREE_LIMIT" <<'PYLIM'
import re, sys
src, dst, lim = sys.argv[1], sys.argv[2], sys.argv[3]
s = open(src, encoding='utf-8').read()
s2, n = re.subn(r'MISSION_TIMEOUT\s*=\s*[\d.]+', 'MISSION_TIMEOUT = %s.0' % lim, s, count=1)
if n != 1:
    sys.exit(1)
open(dst, 'w', encoding='utf-8').write(s2)
PYLIM
        then
            SCORE_PY="$PATCHED_LIM"
            echo "      裁判任务超时改为 ${REFEREE_LIMIT}s（MISSION_TIMEOUT）"
        else
            echo "      警告：--referee-limit 打补丁失败，继续用官方 600s" >&2
        fi
    fi

    SCORE_OUTPUT_PATH="$ROUND_DIR/score.txt" setsid python3 -u "$SCORE_PY" typhoon_h480 \
        "${sensors_args[@]}" >"$ROUND_DIR/referee.log" 2>&1 &
    echo "      分数输出：$ROUND_DIR/score.txt（日志 referee.log）"
fi
if [ "$WITH_MIRROR" = "1" ]; then
    echo "      启动 referee_mirror.py（裁判镜像，实时看进度）"
    setsid python3 -u "$SCRIPT_DIR/src/yolo/referee_mirror.py" typhoon_h480 6 6 \
        >"$ROUND_DIR/mirror.log" 2>&1 &
fi

# ---------- 真值话题订阅监测（防作弊）----------
# 官方口径：/xtdrone/typhoon_h480_N/ground_truth/odom 会发布但禁止订阅，
# 一订阅即判作弊终止比赛。监测器把这件事变成可执行的自查/裁判端告警。
if [ "${WITH_CHEAT_MONITOR:-1}" = "1" ] && [ -f "$SCRIPT_DIR/tools/cheat_monitor.py" ]; then
    echo "      启动 ground_truth 订阅监测（违规写入 cheat_monitor.log）"
    setsid python3 -u "$SCRIPT_DIR/tools/cheat_monitor.py" \
        --interval 2 --log "$ROUND_DIR/cheat_monitor.log" \
        >"$ROUND_DIR/cheat_monitor_stdout.log" 2>&1 &
    CHEAT_PID=$!
fi

# ---------- 裁判计分看门狗（2026-10-12 加；WITH_REFEREE_WATCHDOG=0 可关）----------
# 实测官方 score_cal 会中途退出（referee.log 末尾 `score: 0` 后进程消失），之后 /score
# 变空、整轮"看起来还在跑其实已不算分"。看门狗按 launcher 完全相同的方式重新拉起
# （source ROS + 本工作区 devel + SCORE_OUTPUT_PATH），官方 score_cal.py 一个字不改。
if [ "${WITH_REFEREE_WATCHDOG:-1}" = "1" ] && [ -f "$SCRIPT_DIR/tools/referee_watchdog.py" ]; then
    echo "      启动裁判计分看门狗（退出即自动拉起；日志 referee_watchdog.log）"
    setsid python3 -u "$SCRIPT_DIR/tools/referee_watchdog.py" \
        --round-dir "$ROUND_DIR" --interval 5 \
        >"$ROUND_DIR/referee_watchdog_stdout.log" 2>&1 &
fi

# ---------- 人物看门狗（同模式；此前只有文档说明、没自动挂，这里一并挂上）----------
if [ "${WITH_ACTOR_WATCHDOG:-1}" = "1" ] && [ -f "$SCRIPT_DIR/tools/actor_watchdog.py" ]; then
    echo "      启动人物看门狗（官方 control_actor.py 崩溃即拉起；日志 actor_watchdog.log）"
    setsid python3 -u "$SCRIPT_DIR/tools/actor_watchdog.py" \
        --round-dir "$ROUND_DIR" --interval 5 \
        >"$ROUND_DIR/actor_watchdog_stdout.log" 2>&1 &
fi

echo
echo "================================================"
echo " 模拟裁判端已就绪（官方链路）"
echo "   运行上限：${ROUND_LIMIT}s（到点请手动结束或用 Ctrl-C）"
echo "   实时看分： rostopic echo /score"
echo "   看剩余人物：rostopic echo /left_actors"
echo "   看人物进度：rostopic echo /find_actor_0"
echo "   本轮目录：$ROUND_DIR"
echo " Ctrl-C 结束（gzserver/PX4 会保留；清场用 tools/kill_stale_sim.py）"
echo "================================================"
wait
