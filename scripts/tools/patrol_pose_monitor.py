#!/usr/bin/env python3
"""临时监视器: 记录 /amcl_pose 与目标航点的距离, 验证到达精度。"""
import csv
import math
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

# 目标点: 与 waypoints.yaml 一致; None 表示"回起点阶段"
TARGETS = [
    (3.143, 0.032, 'wp1'),
    (2.046, -0.538, 'wp2'),
    (0.0, 0.0, 'home'),
]


class Monitor(Node):
    def __init__(self):
        super().__init__('patrol_pose_monitor')
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.cb, 10)
        self.t = time.time()
        self.stage = 0
        self.log = open('/tmp/patrol_pose_log.csv', 'w', newline='')
        self.w = csv.writer(self.log)
        self.w.writerow(['t', 'stage', 'target_x', 'target_y', 'pose_x', 'pose_y', 'dist', 'yaw_deg'])

    def cb(self, msg):
        now = time.time()
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        tx, ty, name = TARGETS[min(self.stage, 2)]
        if now - self.t > 30:
            self.t = now
        d = math.hypot(x - tx, y - ty)
        q = msg.pose.pose.orientation
        yaw = math.degrees(math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        if d < 0.10 and name != 'home':
            self.stage += 1
            self.get_logger().info(f'=== 进入阶段 {name} 范围(10cm) ===')
        self.w.writerow([f'{now:.2f}', self.stage, tx, ty, f'{x:.3f}', f'{y:.3f}', f'{d:.3f}', f'{yaw:.1f}'])
        self.log.flush()


def main():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.log.close()


if __name__ == '__main__':
    main()
