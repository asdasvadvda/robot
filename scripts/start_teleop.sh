#!/usr/bin/env bash
# start_teleop.sh — 一键启动"键盘 + 直行自动纠偏"
#
# 用法(容器内):
#     bash /home/ubuntu/my_ros2_ws/scripts/start_teleop.sh
#
# 行为:
#   1. 停掉可能残留的旧纠偏节点 / 旧键盘(避免双发布者)
#   2. 后台启动 straighten_teleop.py --frame odom (反馈=odom, 不依赖定位)
#   3. 前台启动 teleop_twist_keyboard, 自动 remap cmd_vel -> cmd_vel_raw
#   4. 键盘退出(Ctrl+C / 关终端)时自动停掉纠偏节点
#
# 提示: 按 w/s 直行自动纠偏; 转向键/空格原样透传; 倒车同样纠偏。
# 纠偏节点日志: /tmp/straighten_teleop.log

set -e

source /opt/ros/humble/setup.bash
source /home/ubuntu/my_ros2_ws/ros2_ws/install/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 清理可能残留的旧实例, 防止双发布者抢 /cmd_vel
pkill -f 'straighten_teleop.py' 2>/dev/null || true
pkill -f 'teleop_twist_keyboard' 2>/dev/null || true
sleep 0.5

# 后台启动纠偏节点 (日志落盘, 排查方便)
python3 "$SCRIPT_DIR/straighten_teleop.py" --frame odom \
    > /tmp/straighten_teleop.log 2>&1 &
CORRECT_PID=$!
sleep 1.0
if ! kill -0 "$CORRECT_PID" 2>/dev/null; then
    echo "[start_teleop] 错误: 纠偏节点启动失败, 见 /tmp/straighten_teleop.log" >&2
    exit 1
fi
echo "[start_teleop] 纠偏节点已启动 (PID $CORRECT_PID, 反馈=odom, 日志 /tmp/straighten_teleop.log)"

cleanup() {
    echo ""
    echo "[start_teleop] 停止纠偏节点 (PID $CORRECT_PID)..."
    kill "$CORRECT_PID" 2>/dev/null || true
    echo "[start_teleop] 已退出"
}
trap cleanup EXIT INT TERM

echo "[start_teleop] 启动键盘 (remap: cmd_vel -> cmd_vel_raw), Ctrl+C 退出"
echo "[start_teleop] 提示: w/s 直行自动纠偏 | 转向键/空格透传 | 倒车同样纠偏"
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=cmd_vel_raw
