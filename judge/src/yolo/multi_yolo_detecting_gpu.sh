#!/bin/bash
# GPU 版 YOLO 检测启动器（RTX 5070 / CUDA 12.8 / torch 2.11+cu128）
# 用法：bash multi_yolo_detecting_gpu.sh [无人机数量]
# 依赖：~/miniconda3/envs/yolo_gpu（python3.10 + torch cu128 + ultralytics）
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
typhoon_h480_num=${1:-1}

GPU_PY="$HOME/miniconda3/envs/yolo_gpu/bin/python"
if [ ! -x "$GPU_PY" ]; then
    echo "错误：找不到 GPU 环境 $GPU_PY，请先创建：
  conda create -n yolo_gpu python=3.10 -y -c conda-forge
  \$HOME/miniconda3/envs/yolo_gpu/bin/pip install torch --index-url https://download.pytorch.org/whl/cu128
  \$HOME/miniconda3/envs/yolo_gpu/bin/pip install ultralytics pyyaml rospkg defusedxml netifaces numpy" >&2
    exit 1
fi

# ROS Python 路径（rospy/消息包为纯 Python，py3.10 可直接复用）
export PYTHONPATH="/opt/ros/noetic/lib/python3/dist-packages:${WORKSPACE_DIR:-$HOME/2025_ZZU_FLY}/devel/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="/opt/ros/noetic/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

echo "GPU YOLO: $("$GPU_PY" -c 'import torch; print(torch.__version__, "| cuda:", torch.cuda.is_available(), "|", torch.cuda.get_device_name(0) if torch.cuda.is_available() else "NO")')"

# 诚信开关：1 = 用 Gazebo 真值位置校正检测框颜色/ID（自测/演示用）；
# 正式比赛若不允许用真值辅助，设为 0（USE_POSITION_CORRECTION=0 bash multi_yolo_detecting_gpu.sh 6）
USE_POSITION_CORRECTION="${USE_POSITION_CORRECTION:-1}"
echo "YOLO 位置真值校正=$USE_POSITION_CORRECTION（正式赛建议 0）"

for ((vehicle_num = 0; vehicle_num < typhoon_h480_num; vehicle_num++)); do
    "$GPU_PY" "$SCRIPT_DIR/yolo11n.py" typhoon_h480 "$vehicle_num" \
        "_use_position_correction:=$USE_POSITION_CORRECTION" \
        "_color_topic:=/typhoon_h480_${vehicle_num}/${ZZU_COLOR_TOPIC_SUFFIX:-realsense/depth_camera/color/image_raw}" &
done

wait
