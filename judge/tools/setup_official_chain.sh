#!/bin/bash
# ============================================================================
# setup_official_chain.sh —— 把官方 XTDrone 的 sitl_config 铺进 PX4 树，
# 使「官方链路」可以直接跑：
#
#     roscore
#     roslaunch robocup.launch          # 6× typhoon_h480 + robocup.world(6 人物)
#     bash control_actors.sh            # 6 个人物运动
#     python3 score_cal.py              # 裁判出分
#
# 适用：本机作为「模拟裁判机」，参赛算法在另一台机器上通过 ROS 组网订阅。
#
# 为什么需要铺：官方 robocup.launch 依赖
#   $(find mavlink_sitl_gazebo)/worlds/robocup.world   ← PX4 树里没有
#   $(find px4)/launch/single_vehicle_spawn_xtd.launch ← PX4 树里没有
#   XTDrone 版 rcS（offboard 端口 34580+i / 24540+i）  ← PX4 原版是 14580/14540
#
# 安全性：只“新增”，不覆盖既有模型目录；会被覆盖的 rcS / airframe 一律先备份为
#         *.orig_<时间戳>。--check 只打印将要做什么，不落盘。
#
# 用法：
#   bash tools/setup_official_chain.sh --check          # 只检查、列出动作
#   bash tools/setup_official_chain.sh                  # 执行
#   PX4_DIR=/path/to/PX4 bash tools/setup_official_chain.sh
#   XTDRONE_DIR=/path/to/XTDrone bash tools/setup_official_chain.sh
# ============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

PX4_DIR="${PX4_DIR:-$HOME/robocup_fly/PX4_Firmware}"
XTDRONE_DIR="${XTDRONE_DIR:-$HOME/robocup_fly/XTDrone}"
SITL="$XTDRONE_DIR/sitl_config"
STAMP="$(date +%Y%m%d_%H%M%S)"
CHECK_ONLY=0

for a in "$@"; do
    case "$a" in
        --check) CHECK_ONLY=1 ;;
        --px4-dir=*)     PX4_DIR="${a#*=}" ;;
        --xtdrone-dir=*) XTDRONE_DIR="${a#*=}"; SITL="$XTDRONE_DIR/sitl_config" ;;
        -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
        *) echo "未知参数：$a" >&2; exit 2 ;;
    esac
done

GZ="$PX4_DIR/Tools/sitl_gazebo"
POSIX="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix"

echo "================================================"
echo " 官方链路铺设（XTDrone sitl_config → PX4 树）"
echo " PX4_DIR     = $PX4_DIR"
echo " XTDRONE_DIR = $XTDRONE_DIR"
echo " 模式        = $([ "$CHECK_ONLY" = "1" ] && echo '仅检查(--check)' || echo '执行')"
echo "================================================"

# ---------- [0] 前置校验 ----------
fail=0
need_dir() { [ -d "$1" ] || { echo "  缺目录: $1" >&2; fail=1; }; }
need_file() { [ -f "$1" ] || { echo "  缺文件: $1" >&2; fail=1; }; }
echo "[0] 前置校验"
need_dir  "$PX4_DIR";             echo "  PX4 树            ✓"
need_dir  "$GZ";                  echo "  Tools/sitl_gazebo ✓"
need_dir  "$POSIX";               echo "  ROMFS/init.d-posix ✓"
need_dir  "$SITL";                echo "  XTDrone/sitl_config ✓"
need_file "$PX4_DIR/package.xml";                 echo "  px4 package.xml   ✓"
need_file "$GZ/package.xml";                      echo "  mavlink_sitl_gazebo package.xml ✓"
need_file "$SITL/launch/robocup.launch";          echo "  robocup.launch    ✓"
need_file "$SITL/launch/single_vehicle_spawn_xtd.launch"; echo "  single_vehicle_spawn_xtd.launch ✓"
need_file "$SITL/worlds/robocup.world";           echo "  robocup.world     ✓"
need_file "$SITL/init.d-posix/rcS";               echo "  XTDrone 版 rcS    ✓"
need_file "$SITL/init.d-posix/6011_typhoon_h480"; echo "  XTDrone 版 airframe ✓"
[ "$fail" = "0" ] || { echo "错误：前置校验未通过，已中止。" >&2; exit 1; }

echo
echo "[1] 将执行的动作"
cat <<EOF
  a) 复制 worlds/*.world            → $GZ/worlds/            （新增）
  b) 复制 models/*                  → $GZ/models/             （只补缺失，不覆盖）
  c) 复制 init.d-posix/rcS          → $POSIX/rcS              （先备份 *.orig_$STAMP）
  d) 复制 init.d-posix/6011_typhoon_h480(.post) → $POSIX/     （先备份）
  e) 复制 launch/{robocup,single_vehicle_spawn_xtd}.launch → $PX4_DIR/launch/
  f) 生成可执行包装 $PX4_DIR/px4（rosrun px4 px4 需要）
EOF

if [ "$CHECK_ONLY" = "1" ]; then
    echo
    echo "（--check 模式：未做任何修改）"
    # 顺便报告当前是否已铺过
    echo
    echo "[2] 当前状态"
    for p in "$GZ/worlds/robocup.world" "$POSIX/rcS" "$PX4_DIR/launch/robocup.launch" "$PX4_DIR/launch/single_vehicle_spawn_xtd.launch" "$PX4_DIR/px4"; do
        if [ -e "$p" ]; then echo "  已存在: $p"; else echo "  待生成: $p"; fi
    done
    exit 0
fi

# ---------- [2] 备份被覆盖的文件 ----------
echo
echo "[2] 备份将被覆盖的文件（后缀 .orig_$STAMP）"
backup() {
    if [ -e "$1" ] && [ ! -e "$1.orig_$STAMP" ]; then
        cp -a "$1" "$1.orig_$STAMP"
        echo "  备份 $1 → $(basename "$1").orig_$STAMP"
    fi
}

# ---------- [3] worlds ----------
echo
echo "[3] 铺 worlds/"
mkdir -p "$GZ/worlds"
n=0
for f in "$SITL"/worlds/*.world; do
    [ -e "$f" ] || continue
    cp -a "$f" "$GZ/worlds/"; n=$((n + 1))
done
echo "  复制 $n 个 world → $GZ/worlds/"

# ---------- [4] models（只补缺失） ----------
echo
echo "[4] 铺 models/（已存在的目录不覆盖）"
mkdir -p "$GZ/models"
added=0; skipped=0
for d in "$SITL"/models/*; do
    [ -e "$d" ] || continue
    b="$(basename "$d")"
    if [ -e "$GZ/models/$b" ]; then skipped=$((skipped + 1)); else cp -a "$d" "$GZ/models/"; added=$((added + 1)); fi
done
echo "  新增 $added 个模型，跳过已存在 $skipped 个"

# ---------- [5] init.d-posix（rcS + airframe，备份后覆盖） ----------
echo
echo "[5] 铺 init.d-posix/（rcS 与 airframe 用 XTDrone 版本，端口 34580+i/24540+i）"
for b in rcS 6011_typhoon_h480 6011_typhoon_h480.post 2033_plane_gimbal 2033_plane_gimbal.post; do
    [ -e "$SITL/init.d-posix/$b" ] || continue
    backup "$POSIX/$b"
    cp -a "$SITL/init.d-posix/$b" "$POSIX/$b"
    echo "  安装 $b"
done

# ---------- [6] launch 文件 ----------
echo
echo "[6] 铺 launch/"
mkdir -p "$PX4_DIR/launch"
for b in robocup.launch robocup_indoor.launch single_vehicle_spawn_xtd.launch; do
    [ -f "$SITL/launch/$b" ] || continue
    if [ -e "$PX4_DIR/launch/$b" ]; then backup "$PX4_DIR/launch/$b"; fi
    cp -a "$SITL/launch/$b" "$PX4_DIR/launch/$b"
    echo "  安装 launch/$b"
done

# ---------- [6b] 修补 XTDrone 的 single_vehicle_spawn_xtd.launch ----------
# XTDrone 原件把 <env name="PX4_SIM_MODEL"> / <env name="PX4_ESTIMATOR"> 写在
# <node> 外面（顶层 <env> 被 roslaunch 忽略）→ PX4 拿不到 PX4_SIM_MODEL →
# rcS 报 `Error: Unknown model ''` 并以 256 退出 → airframe 不加载、offboard
# mavlink 实例不启动 → MAVROS 永远 connected=false。这里挪进 <node>（幂等）。
echo
echo "[6b] 修补 launch/single_vehicle_spawn_xtd.launch 的 <env> 位置"
python3 - "$PX4_DIR/launch/single_vehicle_spawn_xtd.launch" <<'PYEOF'
import re, sys
path = sys.argv[1]
src = open(path, encoding='utf-8').read()
if 'zzu_fly 修补' in src:
    print("  已是修补版（带 zzu_fly 修补标记），跳过")
    raise SystemExit(0)
if 'zzu_fly' in src and 'PX4_SIM_MODEL' in src and '<env name="PX4_SIM_MODEL"' in src:
    node_re0 = re.compile(r'<node name="sitl_\$\(arg ID\)"[^>]*>.*?<env name="PX4_SIM_MODEL"', re.S)
    if node_re0.search(src):
        print("  已是修补版，跳过")
        raise SystemExit(0)
envs = re.findall(r'^\s*<env name="(?:PX4_SIM_MODEL|PX4_ESTIMATOR)"[^>]*/>\s*$', src, re.M)
if not envs:
    print("  未发现顶层 <env>（可能已修好或版本不同），跳过")
    raise SystemExit(0)
for e in envs:
    src = src.replace(e + "\n", "", 1)
block = "\n".join("        " + e.strip() for e in envs)
node_re = re.compile(r'(<node name="sitl_\$\(arg ID\)"[^>]*>)\s*(</node>)', re.S)
src2, n = node_re.subn(lambda m: m.group(1) + "\n" + block + "\n    " + m.group(2), src)
if n == 0:
    print("  警告：未找到空的 sitl node，未能自动修补；请手工把 <env> 移入 <node>")
    raise SystemExit(1)
open(path, 'w', encoding='utf-8').write(src2)
print("  已把 %d 个 <env> 移入 <node>（PX4_SIM_MODEL / PX4_ESTIMATOR）" % len(envs))
PYEOF

# ---------- [6c] 修正 XTDrone rcS 的 SITL 陀螺标定 ID ----------
# XTDrone 的 rcS 把 CAL_GYRO0_ID/CAL_GYRO_PRIME 写成 2294028，与 PX4 v1.11 SITL
# 仿真 IMU 的器件 ID 不匹配 → PreFlightCheck 报 "Gyro #0 uncalibrated" /
# "Primary gyro not found" → 解锁被拒（MAV_RESULT_TEMPORARILY_REJECTED）。
# 官方 v1.11 的值是 1311244。这里改回官方值（eeprom 里的旧值需另删工作目录）。
echo
echo "[6c] 修正 rcS 的 CAL_GYRO0_ID / CAL_GYRO_PRIME（XTDrone 值 2294028 → 官方 1311244）"
RCS="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/rcS"
if [ -f "$RCS" ] && grep -aq 'CAL_GYRO0_ID 2294028' "$RCS"; then
    cp -f "$RCS" "$RCS.bak_gyrofix_$(date +%Y%m%d_%H%M%S)"
    sed -i 's/CAL_GYRO0_ID 2294028/CAL_GYRO0_ID 1311244/; s/CAL_GYRO_PRIME 2294028/CAL_GYRO_PRIME 1311244/' "$RCS"
    echo "  已改回官方值（备份 $RCS.bak_gyrofix_*）"
    echo "  注意：请同时删除 PX4 工作目录，否则 eeprom 里的旧标定值会覆盖："
    echo "    rm -rf ~/.ros/sitl_typhoon_h480_*"
else
    echo "  无需修改（已是官方值或文件不存在）"
fi

# ---------- [7] px4 可执行包装 ----------
# single_vehicle_spawn_xtd.launch 用 pkg="px4" type="px4"，rosrun 需要在 px4 包里
# 找到名为 px4 的可执行文件；PX4 源码树里没有，这里生成一个薄包装。
echo
echo "[7] 生成 $PX4_DIR/px4 包装（rosrun px4 px4）"
cat > "$PX4_DIR/px4" <<'EOS'
#!/bin/sh
# 由 zzu_fly/tools/setup_official_chain.sh 生成。
# rosrun px4 px4 <ROMFS> -s etc/init.d-posix/rcS -i <ID> -w <dir> -d
# 直接把参数透传给 PX4 SITL 真身。
SELF_DIR="$(cd "$(dirname "$0")" && pwd)"
BIN="$SELF_DIR/build/px4_sitl_default/bin/px4"
[ -x "$BIN" ] || { echo "找不到 PX4 SITL 二进制：$BIN" >&2; exit 1; }
exec "$BIN" "$@"
EOS
chmod +x "$PX4_DIR/px4"
echo "  已生成并 chmod +x"

# ---------- [8] 运行所需环境变量 ----------
echo
echo "================================================"
echo " 铺设完成。运行官方链路前需要这些环境变量"
echo " （start_official_judge.sh 会自动设置）："
echo "================================================"
cat <<EOF
export ROS_PACKAGE_PATH="\$ROS_PACKAGE_PATH:$PX4_DIR:$GZ"
export GAZEBO_MODEL_PATH="$GZ/models\${GAZEBO_MODEL_PATH:+:\$GAZEBO_MODEL_PATH}"
export GAZEBO_PLUGIN_PATH="$PX4_DIR/build/px4_sitl_default/build_gazebo\${GAZEBO_PLUGIN_PATH:+:\$GAZEBO_PLUGIN_PATH}"
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_RESOURCE_PATH="$GZ/worlds\${GAZEBO_RESOURCE_PATH:+:\$GAZEBO_RESOURCE_PATH}"
EOF
echo
echo "验证："
echo "  source /opt/ros/noetic/setup.bash && source $SCRIPT_DIR/devel/setup.bash"
echo "  export ROS_PACKAGE_PATH=\"\$ROS_PACKAGE_PATH:$PX4_DIR:$GZ\""
echo "  rospack find px4 && rospack find mavlink_sitl_gazebo"
echo "  roslaunch --nodes px4 robocup.launch | head"
