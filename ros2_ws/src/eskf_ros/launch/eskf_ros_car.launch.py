#!/usr/bin/env python3
# ============================================================
#  eskf_ros 实车 launch：把 eskf_node 接到小车真实话题
#
#  输入：/imu/data (sensor_msgs/Imu, ~50Hz, robot_driver 发布)
#       /odom     (nav_msgs/Odometry, ~50Hz, STM32 轮式里程计)
#  输出：/odom_filtered  (ESKF 融合位姿, IMU 频率)
#
#  与仿真 launch 的差异：
#   ① remap /imu -> /imu/data   (小车话题名不同, 不改代码)
#   ② 噪声参数按小车实测标定 (静止采集 455 样本, 50Hz):
#      - sigma_g = 0.001  (gyro_z 每样本 std≈0.007  ×√0.02)
#      - sigma_a = 0.013  (accel x/z 每样本 std≈0.09 ×√0.02)
#      - sigma_obs= 0.1   (odom 里程 ~4% 误差, 1m≈4cm)
#   ③ frame_world=odom   与底盘 /odom 同 frame, RViz 直接对比
#
#  用法:
#    source /opt/ros/humble/setup.bash
#    source /home/ubuntu/my_ros2_ws/ros2_ws/install/setup.bash
#    ros2 launch eskf_ros eskf_ros_car.launch.py
# ============================================================
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    node = Node(
        package='eskf_ros',
        executable='eskf_node',
        name='eskf_node',
        output='screen',
        remappings=[
            ('/imu', '/imu/data'),   # 小车 IMU 话题名
        ],
        parameters=[{
            'sigma_g': 0.001,
            'sigma_a': 0.013,
            'sigma_obs': 0.1,
            'frame_world': 'odom',
            'frame_body': 'base_link',
        }],
    )
    return LaunchDescription([node])
