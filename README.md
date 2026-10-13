# drone_can_fly_judge —— 裁判端仿真环境（迁移用）

2026 中国机器人大赛「多旋翼无人机集群协同搜索仿真赛项」· 郑州大学"无人机能飞"队
**裁判端（仿真侧）** 的代码、补丁与资源，用于**迁移到另一台机器**。

> 📖 **先读 [`README_迁移说明.md`](README_迁移说明.md)**：目标机要求、7 步迁移流程、已知坑速查表。
> 🚀 **再跑 [`install_judge_env.sh`](install_judge_env.sh)**：建议先 `DRY=1 bash install_judge_env.sh` 干跑一遍。

---

## 这套环境负责什么

```
roscore → Gazebo 世界(robocup.world) → 加载 6 架无人机机型 → PX4 SITL ×6 → MAVROS ×6
        → 人物运动(control_actor.py ×6) → 计分(score_cal.py)
```

算法侧（我们的 Docker 镜像 `zzu_fly_algo`）只通过 ROS 话题交互：
订阅 `/typhoon_h480_<i>/...`、`/left_actors`、`/clock`，发布 `/actor_*_info`。

## 目录

| 路径 | 内容 |
|---|---|
| `judge/` | 裁判端自研代码：`start_official_judge.sh`（主启动器）、`tools/round_start.sh`、`tools/kill_stale_sim.py`、`tools/referee_watchdog.py`、`tools/actor_watchdog.py`、`tools/launch/robocup_judge_{gazebo,drone}.launch`、`1.sh/2.sh/3.sh/bp.sh`、`px4_romfs/`、`typhoon_h480_zzufly/`（机型）、`docs/` |
| `patches/px4-v1.13.2-zzufly.patch` | PX4 v1.13.2 本地改动（12 文件，+191/−140）：`rcS`、机型 airframe、`px4-rc.mavlink`、`Commander.cpp`、若干 launch |
| `patches/untracked/` | PX4 树里**未被 git 跟踪但必需**的文件：`ROMFS/.../6011_typhoon_h480(.post)`、`launch/robocup.launch`、`launch/single_vehicle_spawn_xtd.launch` |
| `worlds/robocup/` | 组委会比赛材料：地图、`robocup.world`、`control_actor.py`、`score_cal.py`、`map_generator.py` 等 |
| `worlds/sitl_worlds/` | XTDrone 世界文件（含 `robocup.world`）|
| `model/submit_zzufly_model.zip` | 提交给裁判的机型包（6 份逐架 SDF + 10 网格 + model.config）|
| `docker/` | 算法侧镜像定义（`Dockerfile.player`、`run_player.sh`），供对照 |
| `docs/` | 本次联调全部结论文档（方案审查、迁移记录、比赛当天流程、排障手册等）|

## 环境版本（实测）

Ubuntu 20.04.6 LTS · ROS Noetic · Gazebo 11.15.1 · PX4 v1.13.2 · Python 3.8.10

## 不随仓库的大件（脚本会自动获取）

| 资产 | 大小 | 来源 |
|---|---|---|
| PX4 Firmware v1.13.2 树 | ~11 GB | `git clone -b v1.13.2 --recursive <PX4 仓库>` |
| XTDrone `sitl_config/models`（城市模型） | 287 MB | XTDrone 官方仓库 |
| Gazebo 模型库（可选） | — | 首次运行自动下载 |

## 起一轮自检（迁移后必做）

```bash
cd ~/robocup_fly/zzu_fly
PX4_DIR=~/PX4_Firmware_v1.13.2 bash tools/round_start.sh --model=zzufly --referee-limit=3600
# 期望：6/6 connected=True
for i in 0 1 2 3 4 5; do rostopic echo -n1 /typhoon_h480_$i/mavros/state | grep connected; done
rostopic hz /typhoon_h480_0/scan                                  # ~50 Hz, range_max=20.0
rostopic hz /typhoon_h480_0/realsense/depth_camera/depth/image_raw # ~20 Hz
```

## 注意（高发坑，细节见 README_迁移说明 §5）

- 官方 `robocup.launch` 的 `<arg name="sdf" value="typhoon_h480"/>` 是**固定值**，命令行无法覆盖
  ⇒ 加载我们机型必须用**逐架**方式（`--model=zzufly` 已封装）。
- 容器内 `roslaunch <file>` 必须**带包名或先 cd 到 launch 目录**，否则报 `RLException`。
- 每次先确认 **当前 IP**（踩过：`192.168.12.101` → `192.168.1.196`，写死必炸）。
- 清理磁盘**不要**用 `docker image prune -a`（会删掉交付镜像）。
