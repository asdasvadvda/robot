#!/usr/bin/env python3
"""
global_localize.py — 让 AMCL 在全图范围撒粒子(此版本 AMCL 的"全局定位")

Humble 版 nav2_amcl(1.1.13)没有 /global_localization 服务,也不支持
"零协方差触发全局定位"。它唯一的初始化途径是 /initialpose 的位姿+协方差,
粒子云 = 以位姿为中心、以协方差为散布的高斯云。

所以"全局定位"的等价做法:把协方差设得足够大(盖住整张地图的已探索
区域),粒子从开局就撒满全图。车一动,雷达扫描会淘汰错误假设,粒子云
自己收拢到真实位置——效果和官方全局定位一样,只是初始散布是高斯形。

用法(起导航之后):
    python3 /home/ubuntu/my_ros2_ws/scripts/global_localize.py
然后遥控车:原地转一圈 + 走 1~2 米(到特征多的地方),再用
check_localization.py 看粒子云是否收敛。

如果之后再确认过位置(比如点过初始位姿),想重新全局撒粒子,重跑本脚本即可。
"""
import math
import os
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

MAP_YAML = '/home/ubuntu/my_ros2_ws/maps/my_map.yaml'


def load_map_center():
    """从地图 yaml + pgm 计算已探索区域中心,用来放粒子云的均值。"""
    d = {}
    with open(MAP_YAML) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            key, _, val = line.partition(':')
            key = key.strip()
            val = val.strip().strip('"\'').strip()
            if key == 'image':
                d['image'] = val
            elif key == 'resolution':
                d['resolution'] = float(val)
            elif key == 'origin':
                d['origin'] = [float(x.strip()) for x in val.strip('[]').split(',')]
    res = d['resolution']
    ox, oy, _ = d['origin']
    pgm = os.path.join(os.path.dirname(MAP_YAML), d['image'])
    with open(pgm, 'rb') as f:
        raw = f.read()
    pos = 0

    def tok():
        nonlocal pos
        while pos < len(raw) and raw[pos] in b' \t\r\n':
            pos += 1
        start = pos
        while pos < len(raw) and raw[pos] not in b' \t\r\n':
            pos += 1
        return raw[start:pos]

    magic = tok()
    w, h, maxv = int(tok()), int(tok()), int(tok())
    pos += 1
    data = raw[pos:pos + w * h]
    # 自由 = 值 >= 240;行 0 在图片顶部(北)。世界坐标: x=ox+(c+0.5)*res,
    # y=oy+(H-1-r+0.5)*res。只统计自由格,避免未知区把中心带偏。
    n = 0
    sx = sy = 0.0
    xs, ys = [], []
    for r in range(h):
        for c in range(w):
            if data[r * w + c] >= 240:
                x = ox + (c + 0.5) * res
                y = oy + (h - 1 - r + 0.5) * res
                sx += x
                sy += y
                n += 1
                xs.append(x)
                ys.append(y)
    cx, cy = sx / n, sy / n
    hx = max(abs(max(xs) - cx), abs(cx - min(xs)), 6.0)  # 半宽,保底 6m
    hy = max(abs(max(ys) - cy), abs(cy - min(ys)), 6.0)
    return cx, cy, hx, hy


class GlobalLocalizer(Node):
    def __init__(self):
        super().__init__('global_localize')
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        cx, cy, hx, hy = load_map_center()
        self.get_logger().info(
            f'地图已探索区中心 ({cx:.2f}, {cy:.2f}), 半宽 {hx:.2f}m / {hy:.2f}m')
        # 3σ 盖住整张图: stddev = 半宽, 协方差 = stddev^2; 朝向 ±180°
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = cx
        msg.pose.pose.position.y = cy
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = hx * hx       # x 方差
        msg.pose.covariance[7] = hy * hy       # y 方差
        msg.pose.covariance[35] = math.pi ** 2  # yaw 方差(±180°)
        self.msg = msg
        self.timer = self.create_timer(0.5, self.send)
        self.n = 0
        self.get_logger().info('开始发布大协方差 /initialpose(连发 5 次)→ 粒子撒满全图')

    def send(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)
        self.n += 1
        if self.n >= 5:
            self.get_logger().info('已发送 5 次。现在遥控车:原地转一圈 + 走 1~2 米。')
            rclpy.shutdown()


def main():
    rclpy.init()
    node = GlobalLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
