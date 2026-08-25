#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
航点记录工具: 读取当前 /amcl_pose 定位位姿, 追加到 waypoints.yaml 供巡检使用。

用法(容器内, 先起 nav_bringup.launch.py 且 AMCL 已收敛):
    车搬到位后, 每跑一次记录一个点:
        python3 /home/ubuntu/my_ros2_ws/scripts/record_waypoint.py
    # 上电后首次, 车在建图起点时设初始位姿 (只设, 不记录):
        python3 /home/ubuntu/my_ros2_ws/scripts/record_waypoint.py --initial-pose 0 0 0
    # 查看已记录航点:
        python3 /home/ubuntu/my_ros2_ws/scripts/record_waypoint.py --list
    # 清空重录:
        python3 /home/ubuntu/my_ros2_ws/scripts/record_waypoint.py --clear

记录的是地图(map)系下的 (x, y, 朝向角°)。朝向取多帧平均, 你搬车时怎么摆就怎么记。
"""
import argparse
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener

DEFAULT_FILE = '/home/ubuntu/my_ros2_ws/scripts/waypoints.yaml'

# 每帧间隔 / 采样帧数: 在定位位姿上采几帧求平均, 抗抖动
SAMPLE_INTERVAL = 0.25
SAMPLE_COUNT = 5
# 首帧超时(秒): 无 /amcl_pose 发布则报错退出
FIRST_MSG_TIMEOUT = 8.0


def quat_to_yaw(q):
    """四元数 -> 偏航角(弧度)"""
    t3 = 2.0 * (q.w * q.z + q.x * q.y)
    t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(t3, t4)


def yaw_to_quat(yaw):
    """偏航角(弧度) -> 四元数"""
    return dict(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def mean_yaw(yaws):
    """圆形平均, 避免 -179/179 跳变"""
    s = sum(math.sin(y) for y in yaws)
    c = sum(math.cos(y) for y in yaws)
    return math.atan2(s, c)


def load_waypoints(path):
    if not os.path.exists(path):
        return []
    import yaml
    with open(path, 'r') as f:
        data = yaml.safe_load(f) or {}
    wps = data.get('waypoints') or []
    return [{'x': float(w['x']), 'y': float(w['y']), 'yaw': float(w['yaw'])} for w in wps]


def save_waypoints(path, waypoints):
    import yaml
    with open(path, 'w') as f:
        yaml.safe_dump({'waypoints': waypoints}, f, allow_unicode=True, default_flow_style=False)
    print(f'已写入 {len(waypoints)} 个航点 -> {path}')


def set_initial_pose(x, y, yaw_deg):
    """向 /initialpose 发布初始位姿, 供 AMCL 初始化(只设不记录)"""
    rclpy.init()
    node = rclpy.create_node('record_waypoint_set_initial')
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    yaw = math.radians(yaw_deg)
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    for i in range(10):  # 发几帧确保 AMCL 收到
        pub.publish(msg)
        node.get_logger().info(f'发布初始位姿 ({x}, {y}, {yaw_deg}°) @ map')
        time.sleep(0.2)
    node.destroy_node()
    rclpy.shutdown()


def sample_current_pose(node, tf_buffer):
    """取当前定位位姿: 优先 /amcl_pose; 车静止时 AMCL 不发 pose, 兜底用 tf map->base_link"""
    samples = []
    received = []

    def cb(msg):
        samples.append(msg.pose.pose)
        received.append(True)

    sub = node.create_subscription(
        PoseWithCovarianceStamped, '/amcl_pose', cb, 10)

    # 车在动时 AMCL 会发布 pose, 短等一帧即可; 静止时它不发, 快速转 tf 方案
    deadline = time.time() + 2.0
    node.get_logger().info('等待 /amcl_pose 首帧 (2s, 车静止则转 tf 采样)...')
    while not received and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if received:
        node.destroy_subscription(sub)
        node.get_logger().info('收到 /amcl_pose, 采样取平均...')
        while len(samples) < SAMPLE_COUNT and time.time() - deadline < 3.0:
            n0 = len(samples)
            rclpy.spin_once(node, timeout_sec=SAMPLE_INTERVAL)
            if len(samples) == n0:
                time.sleep(0.05)
        n = len(samples)
        x = sum(s.position.x for s in samples) / n
        y = sum(s.position.y for s in samples) / n
        yaw = mean_yaw([quat_to_yaw(s.orientation) for s in samples])
        return x, y, math.degrees(yaw)

    # 车静止: AMCL 不更新不发 pose (Nav2 设计), 从 tf 采 map->base_link (等价于定位位姿)
    node.get_logger().info('/amcl_pose 无更新 (车静止), 改用 tf map->base_link 采样...')
    node.destroy_subscription(sub)
    tf_samples = []
    deadline = time.time() + FIRST_MSG_TIMEOUT
    while len(tf_samples) < SAMPLE_COUNT and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)  # 喂 tf listener
        try:
            t = tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
            p = t.transform.translation
            q = t.transform.rotation
            tf_samples.append({'x': p.x, 'y': p.y, 'yaw': quat_to_yaw(q)})
        except Exception:
            pass
        time.sleep(SAMPLE_INTERVAL)
    if not tf_samples:
        node.get_logger().error('tf 采样失败, 检查 AMCL 是否已设初始位姿且收敛')
        return None
    x = sum(s['x'] for s in tf_samples) / len(tf_samples)
    y = sum(s['y'] for s in tf_samples) / len(tf_samples)
    yaw = mean_yaw([s['yaw'] for s in tf_samples])
    return x, y, math.degrees(yaw)


def main():
    p = argparse.ArgumentParser(description='记录当前 /amcl_pose 为巡检航点')
    p.add_argument('--file', default=DEFAULT_FILE, help='航点文件路径')
    p.add_argument('--initial-pose', type=float, nargs=3, metavar=('X', 'Y', 'YAW_DEG'),
                   default=None, help='只设初始位姿(不记录), 车在建图起点时传 0 0 0')
    p.add_argument('--list', action='store_true', help='打印已记录航点')
    p.add_argument('--clear', action='store_true', help='清空航点文件')
    args = p.parse_args()

    if args.list:
        for i, w in enumerate(load_waypoints(args.file), 1):
            print(f'航点 {i}: x={w["x"]:.3f}  y={w["y"]:.3f}  yaw={w["yaw"]:.1f}°')
        return

    if args.clear:
        save_waypoints(args.file, [])
        return

    if args.initial_pose is not None:
        set_initial_pose(*args.initial_pose)
        return

    rclpy.init()
    node = rclpy.create_node('record_waypoint')
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    try:
        pose = sample_current_pose(node, tf_buffer)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if pose is None:
        sys.exit(1)

    x, y, yaw = pose
    waypoints = load_waypoints(args.file)
    # 提醒: 与上一个点太近, 可能重复点了
    if waypoints:
        dx = x - waypoints[-1]['x']
        dy = y - waypoints[-1]['y']
        dist = math.hypot(dx, dy)
        if dist < 0.2:
            print(f'警告: 距上一航点仅 {dist:.2f}m, 可能误触重复记录')
    waypoints.append({'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 1)})
    save_waypoints(args.file, waypoints)
    print(f'>>> 已记录 航点{len(waypoints)}: x={x:.3f}  y={y:.3f}  yaw={yaw:.1f}°')


if __name__ == '__main__':
    main()
