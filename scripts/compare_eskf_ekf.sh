#!/usr/bin/env bash
# compare_eskf_ekf.sh — ESKF vs robot_localization EKF 融合对比(路线 A: 并行/离线)
#
# 思路: 只录两个滤波器的输入(/odom + /imu/data)和参考输出(/odometry/filtered)。
#   offline 重放时 ESKF 现算 → 两个滤波器吃同一份数据, 严格同条件对比。
#   对比结果 = compare_odom.py 的统计 + 轨迹/误差图。
#
# 用法:
#   1) 实车巡检时录数据(起着 nav_bringup, 车按正常流程跑, Ctrl+C 结束):
#      bash /home/ubuntu/my_ros2_ws/scripts/compare_eskf_ekf.sh record run1
#      # 想实时看对比, 另开终端直接跑 compare_odom.py --plot live.png
#
#   2) 同数据重放对比(不需要车):
#      bash /home/ubuntu/my_ros2_ws/scripts/compare_eskf_ekf.sh offline run1 --plot out.png
#
# 说明: bag 存在 /home/ubuntu/my_ros2_ws/bags/ 下; 也可以传完整路径。
#   巡检后对比: 跑完回到原点, 两个滤波器"距原点漂移"都应在 0 附近 ——
#   这是比轨迹贴合更硬的指标(闭环误差)。
set -euo pipefail

WS=/home/ubuntu/my_ros2_ws
BAGS=$WS/bags

# 确保 ROS 环境(容器内路径; 已 source 过就跳过)
if ! command -v ros2 >/dev/null 2>&1; then
    source /opt/ros/humble/setup.bash
    source "$WS/ros2_ws/install/setup.bash"
fi

usage() {
    sed -n '1,20p' "$0" | sed 's/^# //; s/^#$//'
}

mode="${1:-}"
[ -z "$mode" ] && { usage; exit 1; }
shift

case "$mode" in
    record)
        bag="${1:?用法: compare_eskf_ekf.sh record <bag名>}"
        mkdir -p "$BAGS"
        echo ">> 录制 /odom /imu/data /odometry/filtered -> $BAGS/$bag"
        echo "   按正常巡检流程跑(回原点后 Ctrl+C 结束录制)"
        ros2 bag record -o "$BAGS/$bag" /odom /imu/data /odometry/filtered
        ;;

    offline)
        bag="${1:?用法: compare_eskf_ekf.sh offline <bag目录> [--plot 图] [--csv 文件]}"
        shift
        [ -d "$bag" ] || bag="$BAGS/$bag"
        [ -d "$bag" ] || { echo "!! 找不到 bag: $bag"; exit 1; }
        # 默认存一张对比图
        case "$*" in
            *--plot*) ;;
            *) set -- "$@" --plot "$BAGS/$(basename "$bag")_compare.png" ;;
        esac

        echo ">> 回放 $bag, ESKF 按实车参数现算, 与录好的 EKF 输出对比"
        ros2 bag play "$bag" --clock & PLAY_PID=$!
        # 直接复用实车 launch(程序化 remap /imu->/imu.data + 标定参数)。
        # 不要用 `ros2 run ... --ros-args -r /imu:=/imu.data`: 本环境 rcl 的
        # remap rule 解析器不接受带点的话题名(实测崩), 只有 launch 的程序化
        # remap 能过; ESKF 只读消息时间戳, 不需要 use_sim_time//clock。
        ros2 launch eskf_ros eskf_ros_car.launch.py & ESKF_PID=$!
        trap 'kill $PLAY_PID $ESKF_PID 2>/dev/null || true' EXIT

        sleep 2   # 等话题起来
        python3 "$WS/scripts/compare_odom.py" --timeout 8 "$@"
        echo ">> 对比完成(统计见上, 图/CSV 路径见上方输出)"
        ;;
    *)
        usage
        exit 1
        ;;
esac
