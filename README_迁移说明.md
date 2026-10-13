# 裁判端仿真环境 · 迁移包说明

> 项目：2026 中国机器人大赛「多旋翼无人机集群协同搜索仿真赛项」· 郑州大学"无人机能飞"队
> 内容：把**本地裁判端（仿真侧）**完整迁移到另一台机器的材料与步骤
> 打包日期：2026-10-13　适用源环境：Ubuntu 20.04.6 / ROS Noetic / Gazebo 11.15.1 / PX4 v1.13.2 / Python 3.8.10

---

## 1. 这套"裁判端"是什么

裁判端 = **仿真 + 裁判程序**（不含我们的算法）。它负责：

```
roscore → Gazebo 世界(robocup.world) → 加载 6 架无人机机型 → PX4 SITL ×6 → MAVROS ×6
        → 人物运动(control_actor.py ×6) → 计分(score_cal.py)
```

算法侧（我们的 Docker 容器，见 `docker/`）只通过 ROS 话题与它交互：
订阅 `/typhoon_h480_<i>/{scan,realsense/...}`、`/left_actors`、`/clock`，发布 `/actor_*_info`。

---

## 2. 包内清单

| 目录 | 内容 | 大小 |
|---|---|---|
| `judge/` | 我们自研的裁判端代码：`start_official_judge.sh`（主启动器）、`tools/round_start.sh`、`tools/kill_stale_sim.py`、`tools/referee_watchdog.py`、`tools/actor_watchdog.py`、`tools/launch/robocup_judge_{gazebo,drone}.launch`、`1.sh/2.sh/3.sh/bp.sh`、`px4_romfs/`、`typhoon_h480_zzufly/`（机型）、`scripts`、`docs/` 等 | ~120 MB |
| `patches/px4-v1.13.2-zzufly.patch` | PX4 v1.13.2 的本地改动（**12 文件 / +191 −140**）：`rcS`、`6011_typhoon_h480.post`、`px4-rc.mavlink`、`Commander.cpp`（本地 unlock 开关）、若干 launch | 40 KB |
| `patches/untracked/` | PX4 树里**未被 git 跟踪但必需**的文件：`ROMFS/.../6011_typhoon_h480`（机型定义）、`.post`、`launch/robocup.launch`、`launch/single_vehicle_spawn_xtd.launch` | 24 KB |
| `worlds/robocup/` | 组委会比赛材料：`base*.world`、`house.world`、`map_generator.py`、`control_actor.py`、`score_cal.py`、`control_actors.sh` 等 | 39 MB |
| `worlds/sitl_worlds/` | `robocup.world`（比赛世界）等 XTDrone 世界文件 | 5.6 MB |
| `model/submit_zzufly_model.zip` | **我们提交给裁判的机型包**（6 份逐架 SDF + 10 网格 + model.config + launch + README） | 6.4 MB |
| `docker/` | 算法侧镜像定义与启动脚本（`Dockerfile.player`、`run_player.sh`）—— 迁移裁判端时不一定需要，留作对照 | 小 |
| `docs/` | 本次联调的全部结论文档（方案审查、迁移记录、比赛当天流程、排障手册等） | 小 |
| `install_judge_env.sh` | 引导脚本（检查环境 → 装依赖 → 克隆并打补丁 → 放资源 → 自检） | — |

### 不随包的大资产（按第 4 节从官方仓库获取）

| 资产 | 大小 | 来源 |
|---|---|---|
| PX4 Firmware v1.13.2 树（含 submodule） | ~11 GB | `git clone -b v1.13.2 --recursive https://github.com/PX4/PX4-Autopilot.git`（或 Gitee 镜像）|
| XTDrone `sitl_config/models`（城市模型） | 287 MB | XTDrone 官方仓库 `sitl_config/` |
| XTDrone `sitl_config/gazebo_plugin` 等 | ~100 MB | 同上 |
| Gazebo 模型库 `~/.gazebo/models`（可选） | — | 首次运行自动下载 |

---

## 3. 目标机器要求

- **Ubuntu 20.04**（Noetic 支持到 20.04；本环境实测 20.04.6）
- **ROS Noetic**（desktop-full 或至少 ros-base + gazebo_ros）
- **Gazebo 11**
- 磁盘：**≥ 40 GB 空闲**（PX4 树 11 GB + 编译产物 + 世界资源）
- 内存 ≥ 8 GB；CPU ≥ 6 核（6 架 PX4 SITL + Gazebo 较重）
- 显卡：**非必需**（仿真用 CPU/软渲染即可；若要跑我们的算法容器才需要 NVIDIA GPU）
- 网络：能访问 GitHub/Gitee、清华镜像源

---

## 4. 迁移步骤（详细命令见 `install_judge_env.sh`，可整体执行）

### 4.1 依赖
```bash
sudo apt update && sudo apt install -y \
  python-is-python3 python3-pip python3-dev tmux git nano vim build-essential cmake \
  ros-noetic-cv-bridge ros-noetic-tf ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs \
  ros-noetic-mavros-msgs ros-noetic-gazebo-msgs ros-noetic-image-transport \
  ros-noetic-nav-msgs ros-noetic-geometry-msgs ros-noetic-gazebo-ros ros-noetic-gazebo-plugins \
  ros-noetic-xacro python3-catkin-tools pkg-config libjsoncpp-dev \
  gazebo11 libgazebo11-dev xmlstarlet
# MAVROS 若未装： sudo apt install -y ros-noetic-mavros ros-noetic-mavros-extras
# 地理数据集（mavros 需要）： bash judge/install_geographiclib_datasets.sh
```

### 4.2 PX4 树 + 补丁 + 编译
```bash
git clone -b v1.13.2 --recursive <PX4 仓库> ~/PX4_Firmware_v1.13.2
cd ~/PX4_Firmware_v1.13.2
git apply /path/to/migrate_judge_env/patches/px4-v1.13.2-zzufly.patch
cp -a /path/to/migrate_judge_env/patches/untracked/. .        # 机型定义与 robocup launch
make px4_sitl_default          # 首次约 15~30 分钟
```

### 4.3 XTDrone 大资产 + 比赛材料
```bash
git clone <XTDrone 仓库> ~/XTDrone_full      # 或用已有的 XTDrone 目录
# 放入比赛材料（本包 worlds/）：
mkdir -p ~/XTDrone_full/robocup
cp -a /path/to/migrate_judge_env/worlds/robocup/.        ~/XTDrone_full/robocup/
cp -a /path/to/migrate_judge_env/worlds/sitl_worlds/.    ~/XTDrone_full/sitl_config/worlds/
```

### 4.4 裁判端代码 + 机型
```bash
cp -a /path/to/migrate_judge_env/judge   ~/robocup_fly/zzu_fly     # 或放到任意路径，脚本内用绝对路径
# 机型包解到启动器期望的位置（$SCRIPT_DIR/typhoon_h480_zzufly）——本包已带该目录，
# 若要换成最新提交版：
cd ~/robocup_fly/zzu_fly && rm -rf typhoon_h480_zzufly && \
  unzip /path/to/migrate_judge_env/model/submit_zzufly_model.zip
```

### 4.5 起一轮自检（**关键验证**）
```bash
cd ~/robocup_fly/zzu_fly
PX4_DIR=~/PX4_Firmware_v1.13.2 bash tools/round_start.sh --model=zzufly --referee-limit=3600
# 期望：6/6 connected=True，且下列话题存在
for i in 0 1 2 3 4 5; do rostopic echo -n1 /typhoon_h480_$i/mavros/state | grep connected; done
rostopic hz /typhoon_h480_0/scan            # 期望 ~50 Hz，range_max=20.0
rostopic hz /typhoon_h480_0/realsense/depth_camera/depth/image_raw   # ~20 Hz
```
> `--model=zzufly` 时启动器会自动：把 `judge/typhoon_h480_zzufly` 软链进 PX4 模型库，
> 并生成 6 个逐架模型目录（`typhoon_h480_zzufly_0..5`）⇒ 无需手工摆放机型 ✓

---

## 5. 已知坑（都在本次联调中踩过，务必注意）

| 症状 | 根因 | 处置 |
|---|---|---|
| 容器/节点连不上 master | `ROS_MASTER_URI` 用了**过时 IP**（本机从 192.168.12.101 变成过 192.168.1.196） | 每次先 `ip -4 addr` 确认；同机自测用 `127.0.0.1` |
| 飞机停 POSCTL 不起飞 | XTDrone **通信桥**窗格没起 ⇒ 无 setpoint 流 ⇒ PX4 拒 OFFBOARD | 确认 `/xtdrone/<veh>/cmd_vel_flu` 有数据（桥接在跑） |
| 官方 `robocup.launch` 加载不了我们的机型 | 其 `<arg name="sdf" value="typhoon_h480"/>` 是**固定值**，命令行不可覆盖 | 用逐架方式（`tools/round_start.sh --model=zzufly` 已封装，或提交的 `zzu_fly_zzufly_model.launch`） |
| 单机 launch 报 `RLException: not a launch file name` | 容器内 `roslaunch <file>` 必须**带包名或先 cd 到 launch 目录** | 统一走 `1.sh` / 带包名 |
| 镜像/磁盘写满 ENOSPC | `~/.ros/log`、PX4 ulog 涨得快 | 见 `docs/TROUBLESHOOTING.md` 清理手册 |
| 误删交付镜像 | 用了 `docker image prune -a` | 只用 `docker image prune -f`（不带 `-a`）；交付镜像另有 `docker save` 归档 |
| 清理脚本自杀 | `pkill -f <自己的命令行>` 会匹配自己 | 用 `pgrep -x <名字>` |
| 人物遇不到 | 本轮实测 6 个人物**全被钉在 x=150**（边界钳制，组委会赛前不修） | 巡逻/区域需覆盖到 x≈150；或接受这几个人拿不到分 |

---

## 6. 与算法侧的关系（避免混淆）

- **本包不含算法**。算法在我们发布的镜像 `zzu_fly_algo:v2`（含 `auto_explore`、`best.pt` 等）里。
- 算法侧启动（详见 `docs/比赛当天流程与检查清单.md`）：
  ```bash
  docker run -d --name player_algo --network host \
    -e ROS_MASTER_URI=http://<裁判机IP>:11311 -e ROS_IP=<本机IP> \
    --device /dev/nvidia0 --device /dev/nvidiactl --device /dev/nvidia-uvm --device /dev/nvidia-modeset \
    -v /usr/lib/x86_64-linux-gnu/libcuda.so.1:/usr/lib/x86_64-linux-gnu/libcuda.so.1:ro \
    -v /usr/lib/x86_64-linux-gnu/libnvidia-ml.so.1:/usr/lib/x86_64-linux-gnu/libnvidia-ml.so.1:ro \
    -v /usr/lib/x86_64-linux-gnu/libnvidia-ptxjitcompiler.so.1:/usr/lib/x86_64-linux-gnu/libnvidia-ptxjitcompiler.so.1:ro \
    zzu_fly_algo:v2 sleep infinity
  docker exec -it player_algo bash -lc 'cd /root/ZZU_FLY && bash 1.sh'
  ```
- **顺序**：裁判端起世界+模型+MAVROS → 6/6 `connected=True` → 再起算法容器。

---

## 7. 目录树（包内）

```
migrate_judge_env/
├── install_judge_env.sh              引导脚本
├── README_迁移说明.md                本文件
├── judge/                            裁判端代码（启动器/工具/launch/机型/docs）
├── patches/
│   ├── px4-v1.13.2-zzufly.patch      PX4 改动补丁（git apply 可用）
│   └── untracked/                    未跟踪但必需的文件（机型定义 + robocup launch）
├── worlds/
│   ├── robocup/                      比赛材料（地图/脚本/计分）
│   └── sitl_worlds/                  robocup.world 等
├── model/submit_zzufly_model.zip      提交机型包
├── docker/                           算法侧镜像定义（对照）
└── docs/                             全部结论文档
```
