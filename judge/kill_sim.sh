#!/bin/bash
# 完全停止 6 机 Gazebo 仿真及相关 ROS/PX4/YOLO/RViz 进程
# 用法: bash kill_sim.sh

echo ">>> 停止所有仿真相关进程..."

pkill -9 -f start_6g_auto_explore.sh 2>/dev/null || true
pkill -9 -f 'bash 1.sh' 2>/dev/null || true
pkill -9 -f roslaunch 2>/dev/null || true
pkill -9 -f rosmaster 2>/dev/null || true
pkill -9 -f roscore 2>/dev/null || true
pkill -9 -f rosout 2>/dev/null || true
pkill -9 -f gzserver 2>/dev/null || true
pkill -9 -f gzclient 2>/dev/null || true
pkill -9 -f 'px4' 2>/dev/null || true
pkill -9 -f mavros 2>/dev/null || true
pkill -9 -f yolo11n.py 2>/dev/null || true
pkill -9 -f bbox2coord_node 2>/dev/null || true
pkill -9 -f tracking_node 2>/dev/null || true
pkill -9 -f gimbal_control 2>/dev/null || true
pkill -9 -f explore_node 2>/dev/null || true
pkill -9 -f scan_self_filter 2>/dev/null || true
pkill -9 -f scan_map_fuser 2>/dev/null || true
pkill -9 -f odom_broadcaster 2>/dev/null || true
pkill -9 -f pose_cmd_mux 2>/dev/null || true
pkill -9 -f static_transform_publisher 2>/dev/null || true
pkill -9 -f tf_broadcaster_node 2>/dev/null || true
pkill -9 -f pose_transformer_node 2>/dev/null || true
pkill -9 -f coordinate_estimator 2>/dev/null || true
pkill -9 -f target_lookup_service 2>/dev/null || true
pkill -9 -f xtdrone_mavros_bridge 2>/dev/null || true
pkill -9 -f spawn_model 2>/dev/null || true
pkill -9 -f serial_swarm_bringup 2>/dev/null || true
pkill -9 -f wait_for_model 2>/dev/null || true
pkill -9 -f rviz 2>/dev/null || true
pkill -9 -f down_resume 2>/dev/null || true
pkill -9 -f auto_explore 2>/dev/null || true
pkill -9 -f 'rcS' 2>/dev/null || true

sleep 2

# 释放 ROS master / Gazebo master 端口
fuser -k 11311/tcp 2>/dev/null || true
fuser -k 11345/tcp 2>/dev/null || true

sleep 1

echo ">>> 检查剩余进程..."
ps aux | grep -iE 'gzserver|gzclient|ros|px4|mavros|yolo|tracking|rviz|spawn' | grep -v grep || true

echo ">>> 检查端口..."
for p in 11311 11345; do
    if timeout 1 bash -c "echo > /dev/tcp/127.0.0.1/$p" 2>/dev/null; then
        echo "端口 $p 仍被占用"
    else
        echo "端口 $p 已释放"
    fi
done

echo ">>> 完成。"
