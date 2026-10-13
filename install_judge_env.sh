#!/usr/bin/env bash
# =============================================================================
# install_judge_env.sh —— 把"裁判端仿真环境"迁到另一台机器
#
# 配套文档：README_迁移说明.md（先读它）
# 设计原则：**每一步都先检查、能跳过就跳过**；不静默失败；危险动作前打印命令。
#
# 用法：
#   bash install_judge_env.sh                 # 全流程（缺什么装什么）
#   DRY=1 bash install_judge_env.sh           # 只打印将执行的命令，不真的执行
#   SKIP_APT=1 bash install_judge_env.sh      # 跳过 apt（依赖已装好）
#   SKIP_PX4=1 bash install_judge_env.sh      # 跳过 PX4 克隆/编译（已有树）
#   PX4_SRC=<git url> ...                     # 自定义 PX4 源（内网镜像）
#   XTDRONE_SRC=<git url> ...                 # 自定义 XTDrone 源
#
# 需要的输入（默认按包内目录推断）：
#   PKG_DIR = 本脚本所在目录（迁移包根）
#   WORK    = $HOME/robocup_fly        （裁判端代码落位处）
#   PX4_DIR = $HOME/PX4_Firmware_v1.13.2
#   XTDRONE = $HOME/XTDrone_full
# =============================================================================
set -uo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK="${WORK:-$HOME/robocup_fly}"
PX4_DIR="${PX4_DIR:-$HOME/PX4_Firmware_v1.13.2}"
XTDRONE="${XTDRONE:-$HOME/XTDrone_full}"
PX4_SRC="${PX4_SRC:-https://github.com/PX4/PX4-Autopilot.git}"
XTDRONE_SRC="${XTDRONE_SRC:-https://gitee.com/robin_shaun/XTDrone.git}"
DRY="${DRY:-0}"

say()  { printf '\n\033[1m== %s ==\033[0m\n' "$*"; }
info() { printf '   %s\n' "$*"; }
warn() { printf '   \033[33m⚠ %s\033[0m\n' "$*"; }
die()  { printf '\n\033[31m✗ %s\033[0m\n' "$*" >&2; exit 1; }
run()  { if [ "$DRY" = "1" ]; then printf '   [dry] %s\n' "$*"; else eval "$@"; fi; }
have() { command -v "$1" >/dev/null 2>&1; }

say "0. 环境自检"
info "PKG_DIR = $PKG_DIR"
info "WORK    = $WORK     PX4_DIR = $PX4_DIR     XTDRONE = $XTDRONE"
[ -d "$PKG_DIR/judge" ] || die "包里缺少 judge/ 目录（请确认在迁移包根目录运行）"

. /etc/os-release 2>/dev/null || true
info "OS      = ${PRETTY_NAME:-未知}"
case "${VERSION_ID:-}" in
  20.04) info "✓ 版本匹配（ROS Noetic 需要 20.04）" ;;
  *)     warn "本包在 Ubuntu 20.04 上验证；当前 ${VERSION_ID:-未知}，ROS Noetic 可能装不上" ;;
esac
if have roscore || [ -f /opt/ros/noetic/setup.bash ]; then
  info "✓ ROS 已安装：$(ls /opt/ros/ 2>/dev/null | tr '\n' ' ')"
  [ -f /opt/ros/noetic/setup.bash ] && info "   （新终端别忘了 source /opt/ros/noetic/setup.bash）"
else warn "未检测到 ROS（下一步会尝试安装 ros-noetic-desktop-full）"; fi
have gzserver && info "✓ Gazebo：$(gzserver --version 2>/dev/null | head -1)" || warn "未检测到 gazebo"
info "磁盘可用：$(df -h "$HOME" | awk 'NR==2{print $4}')（建议 ≥40G）"

# ---------------------------------------------------------------- 1) 依赖
if [ "${SKIP_APT:-0}" != "1" ]; then
  say "1. 安装系统依赖（apt）"
  run "sudo apt-get update"
  # ROS 源（若未装 ROS）
  if ! have roscore; then
    info "安装 ROS Noetic（清华源）"
    run "sudo sh -c 'echo \"deb http://packages.ros.org/ros/ubuntu focal main\" > /etc/apt/sources.list.d/ros1-latest.list'"
    run "sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 || true"
    run "sudo apt-get update"
    run "sudo apt-get install -y ros-noetic-desktop-full"
    run "echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc"
  fi
  run "sudo apt-get install -y python-is-python3 python3-pip python3-dev tmux git nano vim \
        build-essential cmake xmlstarlet python3-catkin-tools pkg-config libjsoncpp-dev \
        ros-noetic-cv-bridge ros-noetic-tf ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs \
        ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs \
        ros-noetic-gazebo-ros ros-noetic-gazebo-plugins ros-noetic-gazebo-msgs \
        ros-noetic-image-transport ros-noetic-nav-msgs ros-noetic-geometry-msgs ros-noetic-xacro \
        gazebo11 libgazebo11-dev"
  # mavros 地理数据集
  if [ -f "$PKG_DIR/judge/install_geographiclib_datasets.sh" ]; then
    info "安装 mavros 地理数据集（否则 mavros 起不来）"
    run "bash '$PKG_DIR/judge/install_geographiclib_datasets.sh'"
  fi
else
  say "1. 跳过 apt（SKIP_APT=1）"
fi

# ---------------------------------------------------------------- 2) XTDrone
say "2. XTDrone（提供 sitl_config/models 等大资产）"
if [ -d "$XTDRONE/sitl_config" ]; then
  info "✓ 已存在：$XTDRONE"
else
  info "克隆 XTDrone → $XTDRONE"
  run "git clone --depth 1 '$XTDRONE_SRC' '$XTDRONE'"
  warn "若仓库不含 sitl_config/models（287MB 城市模型），需另行获取并放到 $XTDRONE/sitl_config/models"
fi

# ---------------------------------------------------------------- 3) PX4
say "3. PX4 v1.13.2 + 补丁 + 编译"
if [ "${SKIP_PX4:-0}" = "1" ]; then
  info "跳过（SKIP_PX4=1）"
else
  if [ -d "$PX4_DIR/.git" ]; then
    info "✓ PX4 树已存在：$PX4_DIR"
  else
    info "克隆 PX4 v1.13.2（含 submodule，约 11G，耗时较长）"
    run "git clone -b v1.13.2 --recursive --depth 1 '$PX4_SRC' '$PX4_DIR' || \
         git clone -b v1.13.2 --recursive '$PX4_SRC' '$PX4_DIR'"
  fi
  info "应用本地补丁 px4-v1.13.2-zzufly.patch"
  if [ "$DRY" != "1" ] && git -C "$PX4_DIR" apply --check "$PKG_DIR/patches/px4-v1.13.2-zzufly.patch" 2>/dev/null; then
    run "git -C '$PX4_DIR' apply '$PKG_DIR/patches/px4-v1.13.2-zzufly.patch'"
    info "✓ 补丁已应用"
  else
    warn "补丁无法干净应用（可能已打过或版本不同）——请人工核对：git -C $PX4_DIR apply --check <补丁>"
  fi
  info "复制未被 git 跟踪但必需的文件（机型定义 / robocup launch）"
  run "cp -a '$PKG_DIR/patches/untracked/.' '$PX4_DIR/'"
  info "编译 PX4 SITL（首次 15~30 分钟）"
  if [ "$DRY" = "1" ] || [ -f "$PX4_DIR/build/px4_sitl_default/bin/px4" ]; then
    [ -f "$PX4_DIR/build/px4_sitl_default/bin/px4" ] && info "✓ 已编译产物存在，跳过"
    [ "$DRY" = "1" ] && run "make -C '$PX4_DIR' px4_sitl_default"
  else
    run "make -C '$PX4_DIR' px4_sitl_default"
  fi
fi

# ---------------------------------------------------------------- 4) 比赛材料
say "4. 比赛世界与地图（组委会材料）"
run "mkdir -p '$XTDRONE/robocup' '$XTDRONE/sitl_config/worlds'"
run "cp -a '$PKG_DIR/worlds/robocup/.'     '$XTDRONE/robocup/'"
run "cp -a '$PKG_DIR/worlds/sitl_worlds/.' '$XTDRONE/sitl_config/worlds/'"
if [ "$DRY" != "1" ]; then
  info "地图数：$(ls "$XTDRONE/robocup"/*.world 2>/dev/null | wc -l)　robocup.world：$([ -f "$XTDRONE/sitl_config/worlds/robocup.world" ] && echo ✓ || echo ✗)"
fi

# ---------------------------------------------------------------- 5) 裁判端代码 + 机型
say "5. 裁判端代码与机型"
run "mkdir -p '$WORK'"
if [ -d "$WORK/zzu_fly" ]; then
  warn "$WORK/zzu_fly 已存在 —— 已备份为 zzu_fly.bak.$(date +%m%d_%H%M%S)"
  run "mv '$WORK/zzu_fly' '$WORK/zzu_fly.bak.$(date +%m%d_%H%M%S)'"
fi
run "cp -a '$PKG_DIR/judge' '$WORK/zzu_fly'"
if [ "$DRY" != "1" ] && [ ! -f "$WORK/zzu_fly/typhoon_h480_zzufly/typhoon_h480_zzufly.sdf" ]; then
  info "包内机型缺失，用提交 zip 解出"
  run "cd '$WORK/zzu_fly' && unzip -o '$PKG_DIR/model/submit_zzufly_model.zip'"
fi
if [ -f "$WORK/zzu_fly/typhoon_h480_zzufly/typhoon_h480_zzufly.sdf" ]; then
  info "✓ 机型就位：$WORK/zzu_fly/typhoon_h480_zzufly（$(ls "$WORK/zzu_fly/typhoon_h480_zzufly"/*.sdf 2>/dev/null | wc -l) 份 SDF）"
else
  warn "机型目录不完整，请检查"
fi

# ---------------------------------------------------------------- 6) 模型库链接 + 自检
say "6. 模型库链接与自检"
if [ "$DRY" != "1" ]; then
  # 链接机型进 PX4 模型库（round_start.sh 也会自动做，这里提前做一遍便于手工核对）
  MODELS="$PX4_DIR/Tools/sitl_gazebo/models"
  if [ -d "$MODELS" ]; then
    [ -e "$MODELS/typhoon_h480_zzufly" ] || ln -s "$WORK/zzu_fly/typhoon_h480_zzufly" "$MODELS/typhoon_h480_zzufly"
    info "模型库链接：$MODELS/typhoon_h480_zzufly → $WORK/zzu_fly/typhoon_h480_zzufly"
    info "（逐架目录 typhoon_h480_zzufly_0..5 由 round_start.sh --model=zzufly 自动生成）"
  else
    warn "未找到 $MODELS —— PX4 未编译或路径不同"
  fi
fi

say "7. 下一步（手工执行，验证整链）"
cat <<EOF
   cd $WORK/zzu_fly
   PX4_DIR=$PX4_DIR bash tools/round_start.sh --model=zzufly --referee-limit=3600
   # 期望：6/6 connected=True；/typhoon_h480_0/scan ≈50Hz、range_max=20.0
   for i in 0 1 2 3 4 5; do rostopic echo -n1 /typhoon_h480_\$i/mavros/state | grep connected; done
   # 常见问题见 README_迁移说明.md 第 5 节
EOF
say "完成"
info "若某步失败，请把该步的命令与报错发给维护者；脚本可重复执行（幂等）"
