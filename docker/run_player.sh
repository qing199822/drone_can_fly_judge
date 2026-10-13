#!/bin/bash
# 起选手容器。官方流程要点：
#   --network host   ROS 1 必需（默认 bridge 网络下节点注册的内网 IP 对端连不上）
#   --gpus all       用 GPU 时才加；若宿主机没装 nvidia-container-toolkit 就删掉这行
#   -e ROS_MASTER_URI/ROS_IP  指向裁判机与自己的真实 IP
set -e
JUDGE_IP="${1:-192.168.12.101}"
MY_IP="${2:-192.168.12.102}"
# 【2026-10-13 改动】不再挂载宿主源码 ✗
#   原因：镜像内已自带 /root/ZZU_FLY/src **以及已编译好的 devel/**；
#         若把宿主 src 挂进去，会出现"源码是新的、devel 是旧的"⇒ 现场必崩 ✗
#         （曾踩过：挂载后 CMake 缓存与源码不一致，编译产物对不上）
#   现场只有镜像、没有宿主源码，所以默认不挂 ✓
#   本地演练确需热改源码时，再手动追加这一行：
#     -v /home/wangtao/robocup_fly/ws2025/src:/root/ZZU_FLY/src:ro
docker run -it --rm \
  --name player_algo \
  --network host \
  -e ROS_MASTER_URI="http://${JUDGE_IP}:11311" \
  -e ROS_IP="${MY_IP}" \
  zzu_fly_algo:v1 bash

# ---------------------------------------------------------------------------
# GPU 实测结论（2026-10-12）：本机没装 nvidia-container-toolkit，`--gpus all` 会报
# `could not select device driver "" with capabilities: [[gpu]]` ✗。
# 用「--device + 挂驱动库」手工透传**已验证可用**（无需 sudo）：
#   cuda_available: True | dev: 1 | NVIDIA GeForce RTX 3060（driver 535.230.02）
# 起容器时追加下面这组参数即可（检测必须带 GPU，否则 YOLO 退化为 CPU，几乎出不来框）：
#   --device /dev/nvidia0 --device /dev/nvidiactl --device /dev/nvidia-uvm --device /dev/nvidia-modeset \
#   -v /usr/lib/x86_64-linux-gnu/libcuda.so.1:/usr/lib/x86_64-linux-gnu/libcuda.so.1:ro \
#   -v /usr/lib/x86_64-linux-gnu/libnvidia-ml.so.1:/usr/lib/x86_64-linux-gnu/libnvidia-ml.so.1:ro \
#   -v /usr/lib/x86_64-linux-gnu/libnvidia-ptxjitcompiler.so.1:/usr/lib/x86_64-linux-gnu/libnvidia-ptxjitcompiler.so.1:ro
