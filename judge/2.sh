#!/bin/bash

# ===================================================================
# tmux 多窗格自动化启动脚本 - down_resume 版本
# 适用于 docker 环境
# ===================================================================

# --- 配置 ---
SESSION_NAME="ds_down"
WINDOW_NAME="Simulation"

# --- 警告 ---
echo "警告：正在尝试在单个窗口中创建大量窗格。"
echo "此操作的成功与否高度依赖于您的终端窗口大小。"
echo "如果窗口太小，部分窗格可能无法被创建。"
echo "--------------------------------------------------------"
sleep 2

# --- 步骤 1: 清理并创建会话 ---

echo "正在清理旧的 tmux 会话 '$SESSION_NAME'..."
tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true

echo "正在创建新的 tmux 会话 '$SESSION_NAME'..."
tmux new-session -d -s "$SESSION_NAME" -n "$WINDOW_NAME" "sleep infinity"

# 关键修复：等待一小段时间，确保 tmux 服务器完全准备好
sleep 1

# --- 步骤 2: 定义函数以创建新窗格 (带错误处理) ---

pane_index=0
continue_creating_panes=true

function run_in_new_pane() {
    if ! $continue_creating_panes; then
        return
    fi
    
    local title="$1"
    local command="$2"
    
    if [ $(($pane_index % 2)) -eq 0 ]; then
        split_direction="-h"
    else
        split_direction="-v"
    fi

    echo "在新窗格 '$title' 中启动 (分割方向: $split_direction)..."
    tmux split-window "$split_direction" -t "$SESSION_NAME:$WINDOW_NAME" "echo '--- $title ---'; $command; exec bash" 2>/dev/null
    
    if [ $? -ne 0 ]; then
        echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
        echo "!!! 错误: 无法为 '$title' 创建新窗格。终端空间不足。 !!!"
        echo "!!! 将停止创建更多窗格以防止 tmux 服务器崩溃。       !!!"
        echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
        continue_creating_panes=false
        return
    fi
    
    pane_index=$(($pane_index + 1))
    tmux select-layout -t "$SESSION_NAME:$WINDOW_NAME" tiled
}

# --- 步骤 3: 依次在新窗格中运行所有命令 ---
# 保持原有的命令内容不变

echo "定位"
run_in_new_pane "true_position" "source $HOME/ZZU_FLY/devel/setup.bash && roslaunch pose_init pose.launch"
sleep 2

#echo "建立多机通信"
#run_in_new_pane "comunication" "cd $HOME/XTDrone/communication && bash multi_vehicle_communication.sh"
#sleep 2

echo "云台控制"
run_in_new_pane "gimble_control" "cd $HOME/ZZU_FLY/src/gimbal && bash multi_gimbal_control.sh"
sleep 3

echo "启动静态变换"
run_in_new_pane "static_tf" "cd $HOME/ZZU_FLY/src/mix_nav/simple_navigator/launch && roslaunch static_tf.launch"
sleep 2

echo "tf"
run_in_new_pane "tf" "source $HOME/ZZU_FLY/devel/setup.bash && roslaunch transform_tree tf.launch num_drones:=2"
sleep 2

echo "目标管理"
run_in_new_pane "target_management" "source $HOME/ZZU_FLY/devel/setup.bash && roslaunch look_up target_lookup_service.launch"
sleep 2

echo "总控节点"
run_in_new_pane "commander" "cd $HOME/ZZU_FLY/src/look_up/launch && roslaunch spawn_mux_swarm.launch num_drones:=2"
sleep 2

echo "执行起飞命令..."
run_in_new_pane "Takeoff" "cd $HOME/ZZU_FLY/src/mix_nav/fly/launch && roslaunch fly.launch"
sleep 5

echo "启动导航"
run_in_new_pane "nav_swarm" "source $HOME/ZZU_FLY/devel/setup.bash && roslaunch simple_navigator nav.launch num_drones:=2"
sleep 2

echo "定点巡航"
run_in_new_pane "flight_points" "source $HOME/ZZU_FLY/devel/setup.bash && roslaunch task_manager task.launch num_drones:=2 mission_filename:=mission_middle.json"
sleep 2

echo "启动检测节点"
run_in_new_pane "browsing" "cd $HOME/ZZU_FLY/src/yolo && bash multi_yolo_detecting.sh"
sleep 2

echo "启动追踪节点"
run_in_new_pane "tracking" "source $HOME/ZZU_FLY/devel/setup.bash && roslaunch tracking tracking.launch num_drones:=2"
sleep 2

echo "启动坐标计算节点"
run_in_new_pane "solving" "cd $HOME/ZZU_FLY/src/yolo && bash multi_solving.sh"
sleep 2

# --- 步骤 4: 最终布局与连接 ---

echo "正在完成窗格布局..."
tmux kill-pane -t "$SESSION_NAME:$WINDOW_NAME.0"
tmux select-layout -t "$SESSION_NAME:$WINDOW_NAME" tiled

echo "脚本执行完毕。"
if $continue_creating_panes; then
    echo "所有进程已在 tmux 会话 '$SESSION_NAME' 的平铺窗格中启动。"
else
    echo "部分进程因终端空间不足未能启动。已启动的进程在下方会话中。"
fi

echo "正在连接到 tmux 会话..."
tmux attach-session -t "$SESSION_NAME"
