#!/bin/bash
# bbox2coord 发布端回归测试的一键运行脚本（私有 roscore，端口可配，不碰用户环境）。
#
#   bash src/yolo/test/run_bbox_tests.sh              # 默认端口 11461
#   ROS_TEST_PORT=11470 bash src/yolo/test/run_bbox_tests.sh
#
# 背景：沙箱里 ~/.ros 只读、且每个命令的 /tmp 互相隔离，所以这里一次性起 roscore、
# 跑测试、再收尾，并把 ROS_HOME/ROS_LOG_DIR 指到工作区 logs/ 下。
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/../../.." && pwd)"
PORT="${ROS_TEST_PORT:-11461}"

export ROS_MASTER_URI="http://127.0.0.1:${PORT}"
export ROS_HOME="$REPO/logs/ros_home"
export ROS_LOG_DIR="$REPO/logs/ros"
export PYTHONPATH="$REPO/devel/lib/python3/dist-packages:${PYTHONPATH:-}"
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash

echo ">>> 启动私有 roscore（端口 $PORT）"
roscore -p "$PORT" >"$ROS_LOG_DIR/roscore_bbox_test.log" 2>&1 &
ROSCORE_PID=$!
trap 'kill $ROSCORE_PID 2>/dev/null; wait $ROSCORE_PID 2>/dev/null' EXIT

for _ in $(seq 1 40); do
    if rostopic list >/dev/null 2>&1; then break; fi
    sleep 0.5
done
if ! rostopic list >/dev/null 2>&1; then
    echo "错误：私有 roscore 未就绪（$ROS_MASTER_URI）" >&2
    exit 2
fi

echo ">>> 运行 bbox_continuity_test.py"
python3 -u "$SCRIPT_DIR/bbox_continuity_test.py"
RC=$?
echo ">>> 测试退出码：$RC"
echo "    节点日志：/tmp/bbox_node_test.log（每次运行被覆盖）"
exit $RC
