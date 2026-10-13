#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
typhoon_h480_num=${1:-1}

# 诚信开关：1 = 用 Gazebo 真值位置校正检测框颜色/ID（自测/演示用）；
# 正式比赛若不允许用真值辅助，设为 0（USE_POSITION_CORRECTION=0 bash multi_yolo_detecting.sh 6）
USE_POSITION_CORRECTION="${USE_POSITION_CORRECTION:-1}"
echo "YOLO 位置真值校正=$USE_POSITION_CORRECTION（正式赛建议 0）"

for ((vehicle_num = 0; vehicle_num < typhoon_h480_num; vehicle_num++)); do
    python3 "$SCRIPT_DIR/yolo11n.py" typhoon_h480 "$vehicle_num" \
        "_use_position_correction:=$USE_POSITION_CORRECTION" \
        "_color_topic:=/typhoon_h480_${vehicle_num}/${ZZU_COLOR_TOPIC_SUFFIX:-realsense/depth_camera/color/image_raw}" &
done

wait
