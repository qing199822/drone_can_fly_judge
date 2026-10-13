# 调试与常见问题

## 已修复：3 号机撞加油站后失稳

Gazebo 接触流确认，原 `mission_down.json` 航线会让 `typhoon_h480_3::base_link` 撞到 `gas_station_73::link`。一次捕获中接触发生在 `x=53.20, y=-6.96, z=3.87`，随后俯仰角从 23.6 度升到 34.7 度；另一次运行中相同航段导致持续抬头并坠落。

加油站碰撞网格范围约为 `x=51.59..72.17`、`y=-36.63..-6.64`，最高约 8.98 米，不能在 6 米限高内从上方绕过。当前航线保持 3.5 米高度，先沿 `y=0` 飞到 `x=75`，越过加油站和 `lamp_post_195` 后再向南转。

修复后的 180 秒六机回归中，3 号机没有接触加油站或路灯，最大倾角为 43.2 度且保持飞行。六机最高高度分别为 5.45、4.95、4.48、3.93、4.45、5.49 米。

已确认并修复过一个独立问题：跟踪节点不应直接发布到 `/xtdrone/typhoon_h480_N/cmd_vel_flu`，否则会和 MUX 同时发布。当前正确关系应为：

```text
/typhoon_h480_N/tracking_node
  -> /typhoon_h480_N/mux_inputs/external/pose_cmd
  -> /typhoon_h480_N/pose_cmd_mux
  -> /xtdrone/typhoon_h480_N/cmd_vel_flu
```

用下面命令确认最终速度话题只有 MUX 一个发布者：

```bash
rostopic info /xtdrone/typhoon_h480_3/cmd_vel_flu
rostopic info /typhoon_h480_3/mux_inputs/external/pose_cmd
```

如果仍然坠落，请在 issue 中附上首次异常前后的仿真时间、Gazebo 位姿、Gazebo 接触对、MUX 选中话题和跟踪/任务节点日志。不要把 ROS 日志、PX4 构建目录或完整固件包直接提交到 Git；请压缩后作为 issue 附件或只粘贴相关片段。

## 六路相机没有图像

先检查 Gazebo 日志中是否有：

```text
Can't open display
Unable to create DepthCameraSensor. Rendering is disabled.
```

这表示 Gazebo 没有继承桌面图形会话，不是 YOLO 订阅话题写错。`1.sh` 会在启动 Gazebo 前调用 `scripts/graphics_environment.sh` 自动补齐环境变量，并打印：

```text
Gazebo 图形显示：:1
```

如果仍失败：

```bash
echo "$DISPLAY"
test -r "$XAUTHORITY" && echo readable
pgrep -a gnome-shell
```

不要使用 PX4 1.13；本项目固定使用 `v1.11.0-beta1`。

## 确认 RGB 和深度消息

仿真启动后：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
for id in 0 1 2 3 4 5; do
  timeout 5s rostopic echo -n 1 "/typhoon_h480_${id}/realsense/depth_camera/color/image_raw/header"
  timeout 5s rostopic echo -n 1 "/typhoon_h480_${id}/realsense/depth_camera/depth/image_raw/header"
done
```

只看到话题名不等于相机正常，必须实际收到消息。

## 六机节点检查

```bash
rosnode list | sort
```

应至少包含每架无人机对应的：

- `/typhoon_h480_N/mavros`
- `/typhoon_h480_N_communication`
- `/yolo11n_pedestrian_detector_typhoon_h480_N`
- `/coordinate_estimator_node_typhoon_h480_N`
- `/typhoon_h480_N/tracking_node`
- `/gimbal_control_typhoon_h480_N`
- `/waypoint_navigator_typhoon_h480_N`

## MAVROS 连接失败

```bash
for id in 0 1 2 3 4 5; do
  rostopic echo -n 1 "/typhoon_h480_${id}/mavros/state"
done
```

每架都应有 `connected: True`。如果只有部分实例连接，先停止整个启动脚本，确认没有残留 PX4、MAVROS、Gazebo 或 roslaunch 进程，再重新启动。不要单独重复启动失败编号，因为端口分配是按六机整体配置的。

## 起飞后不进入 OFFBOARD

PX4 接受 OFFBOARD 前必须先持续收到控制设定值。`1.sh` 会先启动六个 XTDrone 通信桥并等待节点出现，然后额外等待两秒，再启动任务节点。手工启动时也必须保持同样顺序。

## 起飞时超过 6 米

起飞器必须在每架飞机到达目标高度后立即停止向该机发布爬升速度，不能等待最慢的一架。修复前实测 1、4、5 号机曾分别达到约 7.62、8.01、9.09 米；修复后同一六机任务的最高值均低于 5.5 米。

## 无人机撞建筑或路灯

任务高度必须低于 6 米，因此不能通过提高到建筑上方来规避。修改航点后先运行：

```bash
python3 src/mix_nav/task_manager/test/test_mission_clearance.py
```

测试按无人机尺寸和控制偏差保留 2 米水平余量，并检查四份任务文件的所有航点严格低于 6 米。它只覆盖已经登记的静态障碍，新增地图或路线仍需要在 Gazebo 中做完整飞行回归。

## 退出后仍有进程

正常退出只需在 `1.sh` 所在终端按 `Ctrl-C`。检查残留：

```bash
pgrep -af 'roslaunch|gzserver|gzclient|px4|mavros|yolo11n|tracking_node'
```

不要在脚本仍运行时直接关闭终端。若桌面或终端异常退出，应先定位确切 PID/进程组，再停止对应仿真，避免误伤其他 ROS 项目。

`1.sh` 会保存任务、辅助节点和底层仿真的 PID，并用可终止后台 Bash 的信号清理。任务节点启动后的自动退出回归耗时约 16 秒。

## 测试说明

项目自身的导航、任务、启动和图形环境测试应通过。`src/gazebo_ros_pkgs` 带有完整上游 Gazebo 测试套件；其中部分测试依赖额外权限、独占端口或专用场景，不适合作为本项目的单次验收标准。提交修改时至少运行 README 中列出的项目测试，并注明未运行的集成测试。

## 磁盘写满（ENOSPC）清理手册（2026-10-12 实战，一次释放 ~13G）

**症状**：工具报 `Error: ENOSPC: no space left on device`；`df -h /` 显示系统盘 96%。

**根因（本机实测）**：`/` 上真正的可弃大头是 **ROS 运行数据**，不是代码：

| 路径 | 实测占用 | 说明 | 处理 |
|---|---|---|---|
| `~/.ros/log/` | **8.1 G**（11174 个目录）| 每轮 roslaunch 的日志目录，**全都当天的**（`-mtime +0` 匹配不到 ✗）| 按 mtime 保留最新 20 个，其余删 ✓ |
| `~/.ros/sitl_typhoon_h480_*` | **3.2 G** | PX4 SITL 工作目录（每轮重建）| 直接删 ✓ |
| `PX4_Firmware_*/build/**/log/*/*.ulg` | ~3.7 G | PX4 飞行日志 | 每个 SITL 目录保留最新 3 个 ✓ |
| `~/.gazebo/server-*` | 160 M | Gazebo 服务端日志 | 删 ✓ |
| `/var/lib/docker` | 15 G | **在数据盘**（`/dev/nvme0n1p5`）不在 `/` ✓ | `docker container/image prune -f` ✓ |

**标准清理命令**（先停仿真，避免文件被占用）：
```bash
python3 tools/kill_stale_sim.py                     # 停 gzserver/PX4/actor
ls -1t ~/.ros/log | tail -n +21 | while read d; do rm -rf ~/.ros/log/$d; done
rm -rf ~/.ros/sitl_typhoon_h480_*
for d in PX4_Firmware_*/build/px4_sitl_default/bin/sitl_*/log/*/; do
  ls -1t "$d"*.ulg 2>/dev/null | tail -n +4 | while read f; do rm -f "$f"; done
done
rm -rf ~/.gazebo/server-*
docker container prune -f && docker image prune -f
```
**不要删**：`~/.cache/codex-*`、`~/codex-desktop-linux`、`~/.zcode`、`~/文档`、`/var/lib/snapd`
（都很大但**不是本项目数据** ✗）

**清理前后**：`/` 可用空间 **4.2 G → 17 G** ✓

## ⚠️ 惨痛教训：不要用 `docker image prune -a`（2026-10-12 实战）

**事故**：清理磁盘时执行 `docker image prune -af`，把交付镜像 **`zzu_fly_algo:v1` 也删了** ✗
—— 因为当时没有"运行中的容器"引用它 ⇒ Docker 认为它是**未使用**镜像 ⇒ 归入 `-a`（全部未使用）清掉 ✓
（基础镜像 `ros:noetic-robot` 同时被删 ✗）

**恢复**：幸好交付归档 `zzu_fly_algo_v1.tar.gz`（3.6G）是 `docker save` 产物（含 `manifest.json` ✓）：
```bash
docker load -i zzu_fly_algo_v1.tar.gz     # 1 分 8 秒恢复 ✓
```
恢复出的是**打包时那一版** ✓（此后重建的层——例如 tf2 构建期校验——不会回来 ✗，需要重新 build ✓）

**正确做法**：
```bash
docker container prune -f      # 只清已退出容器 ✓
docker image prune -f          # 只清 dangling（<none>）镜像 ✓ —— 注意：**不带 -a**！
docker builder prune -f        # 只清构建缓存 ✓
```
**并且**：交付镜像务必**同时保留 `docker save` 归档**（本项目：`zzu_fly_algo_v1.tar.gz`，另存一份在数据盘 ✓）

