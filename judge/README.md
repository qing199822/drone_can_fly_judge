# 2025 RoboCup 多旋翼无人机集群协同搜索仿真

这是 2025 中国机器人大赛无人机挑战赛“多旋翼无人机集群协同搜索仿真”项目的调试版本。仓库包含六架 Typhoon H480 的 ROS 节点、Gazebo 插件、自定义模型、定点巡航任务、YOLO 检测、坐标解算、目标跟踪和一键启动脚本。

当前目标是让更多开发者能在相同版本组合上复现问题并共同调试。请先严格使用下列版本，尤其不要把 PX4 换成 1.13。

## 当前调试状态

截至 2026-07-26，六个 PX4/MAVROS 实例可以连接，六路 RGB 与六路深度图像均能持续发布。跟踪节点已改为通过 `topic_tools` MUX 的外部输入发布控制命令，避免与 MUX 同时向最终 XTDrone 速度话题写入。

已确认 3 号机失稳的直接原因是原航段擦入 `gas_station_73` 的碰撞网格，现改为从加油站和 `lamp_post_195` 北侧绕行。起飞器也会在每架飞机到达目标高度后单独停止爬升。修复后的 180 秒六机回归中没有加油站或路灯接触，六机最高高度为 3.93 至 5.49 米，全部低于 6 米。

当前版本仍是调试快照，不是稳定比赛版本。上述回归只覆盖已登记的静态障碍和一次完整运行；复现与采集建议见 [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md)。

## 已验证环境

- Ubuntu 20.04.6 LTS
- ROS Noetic 1.5.0
- Gazebo 11.15.1
- PX4 `v1.11.0-beta1`
- MAVROS 1.20.1
- Python 3.8.10
- NVIDIA 535.230.02，RTX 3060 12GB
- PyTorch 2.1.2 + CUDA 12.1 wheel
- Ultralytics 8.3.40

完整安装和目录说明见 [docs/ENVIRONMENT.md](docs/ENVIRONMENT.md)，常见故障见 [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md)。

## 仓库内容

```text
2025_ZZU_FLY/
|-- 1.sh                         # 六机一键启动和进程清理
|-- robocup_zzufly.launch        # Gazebo、PX4 SITL、MAVROS 六机启动
|-- typhoon_h480_zzufly/         # 自定义无人机模型和相机
|-- src/
|   |-- gimbal/                  # 六机云台控制
|   |-- look_up/                 # 目标占用管理和任务组合 launch
|   |-- mix_nav/                 # 起飞、导航、任务管理
|   |-- pose_init/               # 本地坐标转全局地图坐标
|   |-- tracking/                # 目标跟踪状态机和控制器
|   |-- transform_tree/          # TF 广播
|   |-- yolo/                    # YOLO 检测和深度坐标解算
|   `-- gazebo_ros_pkgs/         # 与本项目共同构建的 Gazebo ROS 插件
|-- tests/                       # 启动脚本和图形环境回归测试
`-- waypoint/                    # 地图浏览和航点辅助工具
```

PX4、XTDrone、Gazebo 模型库和 Python 虚拟环境不在本仓库中，默认与本项目放在同一个父目录。这样可以避免上传约 1.2GB 的 PX4 环境和本机编译结果。

## 快速开始

先完成 [环境安装](docs/ENVIRONMENT.md)，并确认目录如下：

```text
robocup_fly/
|-- 2025_ZZU_FLY/
|-- PX4_Firmware/
|-- XTDrone/
|-- gazebo_models/
|-- .venv-yolo/
`-- .xtdrone-python/
```

构建项目：

```bash
cd ~/robocup_fly/2025_ZZU_FLY
source /opt/ros/noetic/setup.bash
catkin_init_workspace src
catkin_make -DCMAKE_BUILD_TYPE=Release
```

启动六机 `down` 地图任务：

```bash
cd ~/robocup_fly/2025_ZZU_FLY
bash 1.sh 6 mission_down.json
```

启动脚本会依次检查并启动 Gazebo、六个 PX4 SITL、六个 MAVROS、XTDrone 通信、云台、YOLO、坐标解算、跟踪和任务节点。退出时在启动终端按 `Ctrl-C`，脚本会清理其创建的所有进程。

## 运行检查

静态和脚本测试：

```bash
cd ~/robocup_fly/2025_ZZU_FLY
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 -m unittest discover -s tests -p 'test_*.py'
python3 src/mix_nav/fly/test/test_fly_launch.py
python3 src/mix_nav/task_manager/test/test_mission_clearance.py
rostest simple_navigator velocity_continuity.test
rostest pose_init pose_namespace.test
```

六机运行后，至少应出现六个 MAVROS、六个通信节点、六个 YOLO 节点和 12 路 RGB/深度图像话题。详细检查命令和已知限制见 [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md)。

## 设计概述

比赛地图按主干道位置分为 `up`、`middle` 和 `down` 三类。任务管理器发布预设航点，导航器控制巡航；识别到目标后，跟踪节点通过 MUX 接管控制，目标丢失后再交还任务管理器继续航线。

本调试分支重点处理：

- Gazebo 图形会话缺失导致六路深度相机不发布图像。
- 六机启动顺序、MAVROS 就绪等待和退出清理。
- 航向对准时的速度连续性和紧急制动。
- 低于 6 米的任务航线及建筑、加油站、路灯碰撞余量。
- ROS Noetic 下重复消息包和命名空间冲突。

## 参与调试

请阅读 [CONTRIBUTING.md](CONTRIBUTING.md)。提交问题时附上环境版本、启动命令、首次错误日志，以及受影响的无人机编号和仿真时间。不要提交 `build/`、`devel/`、ROS 日志、PX4 固件包或 Python 虚拟环境。

本仓库包含来自多个上游项目的代码和资源，各目录继续遵循其原有许可。没有明确许可的比赛代码不得被推定为采用某个新的统一许可证。
