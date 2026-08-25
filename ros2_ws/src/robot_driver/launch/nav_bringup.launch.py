#!/usr/bin/env python3
# 一键定位 + 导航: 底盘传感栈(驱动 + EKF + 雷达) + AMCL 定位 + Nav2 导航
#
# 用法(容器内):
#   ros2 launch robot_driver nav_bringup.launch.py
#   ros2 launch robot_driver nav_bringup.launch.py map:=/home/ubuntu/my_ros2_ws/maps/我的图.yaml
#
# 默认用最近一次建图的地图; 建议先跑 mapping.launch.py 建一张新图再传进来。
#
# 启动后:
#   1. 在 RViz 设初始位姿 (2D Pose Estimate), 或跑巡检脚本时用 --initial-pose 给
#   2. 发目标点: RViz 2D Nav Goal, 或 python3 /home/ubuntu/my_ros2_ws/scripts/waypoint_patrol.py
#
# Nav2 参数: /home/ubuntu/my_ros2_ws/config/nav2_params.yaml
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 地图 yaml 路径 (容器内路径)
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='/home/ubuntu/my_ros2_ws/maps/my_map.yaml',
        description='AMCL 定位用地图的 yaml 文件路径 (容器内路径)'
    )

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

    # 2. Nav2: map_server + AMCL + planner + controller + BT navigator + waypoint_follower + smoother
    #    复用 nav2_bringup 官方 bringup_launch.py (内部含 lifecycle 管理)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'bringup_launch.py'
            )
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': '/home/ubuntu/my_ros2_ws/config/nav2_params.yaml',
            'slam': 'False',
            'use_sim_time': 'False',
            'autostart': 'True',
            'use_composition': 'True',
        }.items()
    )

    return LaunchDescription([
        map_arg,
        driver_stack,
        nav2,
    ])
