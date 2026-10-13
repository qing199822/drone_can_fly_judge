# 仿真环境安装与版本说明

## 1. 固定版本

这套工程已经在以下组合中跑通构建、六机 MAVROS 连接和相机图像发布：

| 组件 | 版本 |
| --- | --- |
| 操作系统 | Ubuntu 20.04.6 LTS (Focal) |
| ROS | Noetic 1.5.0 |
| Gazebo Classic | 11.15.1 |
| PX4 | `v1.11.0-beta1` |
| MAVROS | 1.20.1 |
| Python | 3.8.10 |
| GCC / CMake | 9.4.0 / 3.16.3 |
| NVIDIA 驱动 | 535.230.02 |
| GPU | GeForce RTX 3060 12GB |
| PyTorch | 2.1.2 + CUDA 12.1 wheel |
| Ultralytics | 8.3.40 |

`nvidia-smi` 显示的 CUDA 12.2 是驱动支持的最高 CUDA 版本，不要求另外安装完整 CUDA Toolkit。PyTorch wheel 自带所需 CUDA 运行库。

PX4 1.13 在本项目的模型和相机链路中会出现没有摄像机图像的问题。请使用 `v1.11.0-beta1`，不要为了“版本更新”替换它。

## 2. 安装顺序

先按照 XTDrone 中文教程完成 PX4 配置之前的基础部分：

- [XTDrone 基础配置入口](https://www.yuque.com/xtdrone/manual_cn/basic_config)
- [Ubuntu 20.04 / ROS Noetic 基础配置](https://www.yuque.com/xtdrone/manual_cn/basic_config_13)

Gazebo 11 软件源或下载页面打不开时，可参考：

- [Ubuntu 安装 Gazebo 的备用步骤](https://www.cnblogs.com/iwehdio/p/12756241.html)

推荐顺序：

1. 安装 Ubuntu 20.04、ROS Noetic desktop-full 和 ROS 开发工具。
2. 安装 Gazebo 11 和 `libgazebo11-dev`。
3. 安装 MAVROS、MAVROS extras 和 GeographicLib 数据集。
4. 准备 XTDrone、Gazebo 模型库和 Python 依赖。
5. 解压教程提供的约 1.2GB PX4 完整包，不要用 `git clone` 替代。
6. 确认 PX4 版本为 `v1.11.0-beta1`，完成依赖和目录配置后再编译。
7. 构建本仓库，最后启动六机仿真。

基础 ROS/Gazebo 包示例：

```bash
sudo apt update
sudo apt install ros-noetic-desktop-full gazebo11 libgazebo11-dev \
  ros-noetic-mavros ros-noetic-mavros-extras \
  geographiclib-tools python3-rosdep python3-catkin-tools
```

MAVROS 首次安装后执行 GeographicLib 数据集安装脚本：

```bash
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

上述操作会修改系统软件包，执行前应自行确认当前机器的软件源和已有 ROS 安装。

## 3. 工作目录

默认目录布局如下：

```text
~/robocup_fly/
|-- 2025_ZZU_FLY/       # 本仓库
|-- PX4_Firmware/       # 教程提供的完整 PX4 包
|-- XTDrone/            # XTDrone，当前验证提交 8e88116
|-- gazebo_models/      # 场景使用的 Gazebo 模型集合
|-- .venv-yolo/         # YOLO Python 虚拟环境
`-- .xtdrone-python/    # XTDrone 的 Python 补充依赖
```

如果目录不同，可以在启动前覆盖环境变量：

```bash
export PX4_DIR=/absolute/path/to/PX4_Firmware
export XTDRONE_DIR=/absolute/path/to/XTDrone
export GAZEBO_MODELS_DIR=/absolute/path/to/gazebo_models
export XTDRONE_PYTHON=/usr/bin/python3
export XTDRONE_PYTHONPATH=/absolute/path/to/xtdrone-python
```

## 4. PX4 1.11

不要用浅克隆或新版 `main` 分支替代教程给出的完整包。完整包应包含子模块内容、`Tools/sitl_gazebo`、`launch/single_vehicle_spawn_xtd.launch` 和比赛世界文件。

版本检查：

```bash
cd ~/robocup_fly/PX4_Firmware
git describe --tags --always
```

期望输出：

```text
v1.11.0-beta1
```

在 ROS、Gazebo、MAVROS 和编译依赖安装完成后构建：

```bash
cd ~/robocup_fly/PX4_Firmware
make px4_sitl_default gazebo
```

本项目启动前会检查：

```text
PX4_Firmware/build/px4_sitl_default/bin/px4
PX4_Firmware/Tools/sitl_gazebo/worlds/robocup.world
PX4_Firmware/launch/single_vehicle_spawn_xtd.launch
```

缺少任何一项时，先修复 PX4 完整包或编译，不要绕过 `1.sh` 的检查。

## 5. YOLO Python 环境

虚拟环境必须能访问系统 ROS Python 包，因此使用 `--system-site-packages`：

```bash
cd ~/robocup_fly
python3 -m venv --system-site-packages .venv-yolo
source .venv-yolo/bin/activate
python -m pip install --upgrade pip
python -m pip install -r 2025_ZZU_FLY/requirements-yolo.txt
```

验证 GPU：

```bash
source ~/robocup_fly/.venv-yolo/bin/activate
python -c 'import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))'
```

期望第一项为 `True`。CPU 也能启动，但六路 YOLO 推理性能会明显下降。

模型权重已保存在：

```text
src/yolo/yolo11n_942.pt
```

## 6. 构建工作空间

```bash
cd ~/robocup_fly/2025_ZZU_FLY
source /opt/ros/noetic/setup.bash
catkin_init_workspace src
catkin_make -DCMAKE_BUILD_TYPE=Release
```

不要提交或复制别人的 `build/`、`devel/`。它们包含绝对路径和本机生成文件，换机器后必须重新构建。

## 7. 启动

从图形桌面会话中运行：

```bash
cd ~/robocup_fly/2025_ZZU_FLY
bash 1.sh 6 mission_down.json
```

脚本会自动读取当前 GNOME/KDE 会话的 `DISPLAY`、`XAUTHORITY` 和 `XDG_RUNTIME_DIR`。相机传感器依赖 Gazebo 的渲染上下文，即使只需要 ROS 图像话题，也不能让 `gzserver` 在完全没有可用显示会话的环境中启动。

退出时在同一终端按 `Ctrl-C`。不要同时启动两份六机仿真，否则端口、ROS 节点名和 Gazebo 模型名会冲突。

