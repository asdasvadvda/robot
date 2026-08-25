#!/usr/bin/env python3
# 一键建图: 底盘传感栈(驱动 + EKF + 雷达) + slam_toolbox 建图
#
# 用法(容器内):
#   ros2 launch robot_driver mapping.launch.py
#
# 建图流程:
#   1. 起本 launch (等 /map 话题有数据、map->odom TF 出现)
#   2. 跑建图驱动脚本出图:
#        python3 /home/ubuntu/my_ros2_ws/scripts/map_drive_roundtrip.py --dist 3.0
#   3. 记下保存的地图路径(默认 /home/ubuntu/my_ros2_ws/maps/my_map_<时间戳>)
#
# slam_toolbox 配置: /home/ubuntu/my_ros2_ws/config/mapper_params_online_sync.yaml
# (odom_frame:odom, map_frame:map, base_frame:base_link, scan_topic:/MS200/scan, mode:mapping)
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 1. 底盘传感栈: 驱动 + EKF + 雷达 (复用现有一键 launch)
    driver_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_driver'),
                'launch', 'ekf_localization.launch.py'
            )
        ),
        launch_arguments={'include_lidar': 'true'}.items()
    )

    # 2. slam_toolbox 建图节点
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=['/home/ubuntu/my_ros2_ws/config/mapper_params_online_sync.yaml']
    )

    return LaunchDescription([
        driver_stack,
        slam_toolbox,
    ])
