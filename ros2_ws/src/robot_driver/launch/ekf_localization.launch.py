#!/usr/bin/env python3
# 一键启动底盘传感栈: 底盘驱动 + EKF 融合 (odom + imu) + 激光雷达(可选)
#
# 用法:
#   ros2 launch robot_driver ekf_localization.launch.py
#   ros2 launch robot_driver ekf_localization.launch.py include_lidar:=false
#
# 启动内容:
#   1. robot_driver_node  - 串口驱动, 发布 /odom 和 /imu/data, publish_tf=false
#      (TF odom->base_link 由 ekf_node 统一广播, 避免双源冲突)
#   2. ekf_node           - robot_localization 融合节点
#      输入 /odom + /imu/data, 输出 /odometry/filtered, 广播 odom->base_link
#   3. MS200 激光雷达     - 发布 /MS200/scan + base_link->laser_frame 静态 TF
#      (默认启动, 供 slam_toolbox 建图使用; include_lidar:=false 可关掉)
#
# 建图: 在上述基础上再跑
#   ros2 run slam_toolbox async_slam_toolbox_node \
#     --ros-args -p use_sim_time:=false \
#     --params-file /home/ubuntu/my_ros2_ws/config/mapper_params_online_sync.yaml
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 1. 底盘驱动: 只发 odom + imu, 关闭 TF 广播 (由 EKF 接管)
    driver_node = Node(
        package='robot_driver',
        executable='robot_driver_node',
        name='robot_driver_node',
        output='screen',
        parameters=[{'publish_tf': False}]
    )

    # 2. EKF 融合节点
    ekf_config = os.path.join(
        get_package_share_directory('robot_driver'),
        'config', 'ekf.yaml'
    )
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[ekf_config]
    )

    # 3. 激光雷达 (可选): MS200/scan + base_link->laser_frame 静态 TF
    include_lidar = LaunchConfiguration('include_lidar')
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('oradar_lidar'),
                'launch', 'ms200_scan.launch.py'
            )
        ),
        condition=IfCondition(include_lidar)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'include_lidar',
            default_value='true',
            description='是否同时启动激光雷达 (true/false)'
        ),
        driver_node,
        ekf_node,
        lidar_launch,
    ])
