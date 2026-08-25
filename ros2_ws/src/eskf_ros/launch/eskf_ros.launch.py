# ============================================================
#  eskf_ros 一键端到端：node + record + play
#
#  跑完：data/sim_bag 播放（10s 仿真圆周）驱动节点，
#        /odom_filtered 录进 data/odom_filtered_bag，
#        play 播完自动触发 Shutdown，整个 launch 优雅退出。
#  评估：python3 scripts/eval_ros.py --bag data/odom_filtered_bag --source sim
#
#  用法：
#    cd /home/ubuntu/my_ros2_ws/ros2_ws/ESKF && source /opt/ros/humble/setup.bash
#    cd /home/ubuntu/my_ros2_ws/ros2_ws && source install/setup.bash   # 主工作空间 colcon 构建输出
#    ros2 launch eskf_ros eskf_ros.launch.py
#
#  参数说明：sim_bag/out_bag 可用 ros2 launch ... sim_bag:=<路径> 覆盖。
#  噪声参数写死匹配 data/sim_bag（100Hz 仿真：每样本σ·√dt 换算，
#  见 LEARNING_NOTES.md 踩坑1 —— 参数不对 ATE 会漂到 6m）。
# ============================================================
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            RegisterEventHandler, Shutdown, TimerAction,
                            LogInfo)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 仓库根：不能用 __file__ 推导！launch 运行时的 __file__ 指向 install 副本
    # （ros2_ws/install/eskf_ros/share/...），相对推导会错成 install/ 前缀。
    # 学习项目仓库位置固定，直接硬编码（挪仓库时改这一行即可）。
    repo_root = '/home/ubuntu/my_ros2_ws/ros2_ws/ESKF'
    sim_bag = os.path.join(repo_root, 'data', 'sim_bag')
    out_bag = os.path.join(repo_root, 'data', 'odom_filtered_bag')

    # 清掉上次录的 bag（rosbag2 不能覆盖已存在目录）
    clean = ExecuteProcess(
        cmd=['rm', '-rf', out_bag],
        name='clean_old_bag')

    # 节点：参数匹配 sim_bag（100Hz：sigma_g=0.01·√0.01, sigma_a=0.1·√0.01）
    node = Node(
        package='eskf_ros',
        executable='eskf_node',
        parameters=[{
            'sigma_g': 0.001,
            'sigma_a': 0.01,
            'sigma_obs': 0.3,
        }],
    )

    # 录制滤波结果
    record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', out_bag, '/odom_filtered'],
        name='record_filtered')

    # 延迟 3s 播放，等 record 完成 topic 发现。
    # 路径用绝对字符串 + cwd 固定到仓库根 —— 否则 launch 会把绝对路径当
    # "可执行文件"重定位到 install/ 前缀下（ros2 bag 子命令的经典坑）。
    play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', sim_bag],
        name='play_sim_bag',
        cwd=repo_root)
    play_delayed = TimerAction(
        period=3.0,
        actions=[play])

    # play 播完 → 延迟 2s 让 record 把尾部写完 → 优雅 shutdown。
    # 不能 play 一结束就立刻 Shutdown，否则 record 丢尾部、bag 少最后几十点，
    # eval 对齐错位 → ATE 虚高（实测 launch 958 点 vs 手动 998 点，差了 0.4s）。
    shutdown_after_play = RegisterEventHandler(
        OnProcessExit(target_action=play,
                      on_exit=[TimerAction(period=2.0, actions=[Shutdown()])]))

    return LaunchDescription([
        clean,
        node,
        record,
        play_delayed,
        shutdown_after_play,
    ])
