#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
极简阶梯测试: 发一条 vx=0.2 (驱动会保持该命令) -> 等 5s -> 发 vx=0 -> 等停稳
期望位移 = 0.2 × 5 = 1.000 m, 对比 odom 实际位移。
"""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class Step(Node):
    def __init__(self):
        super().__init__('step_tester')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.sub = self.create_subscription(Odometry, '/odom', self.cb, 10)
        self.x = None

    def cb(self, m):
        self.x = m.pose.pose.position.x

    def spin(self, secs):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    rclpy.init()
    n = Step()

    # 等第一个 odom
    t0 = time.time()
    while n.x is None and time.time() - t0 < 2:
        rclpy.spin_once(n, timeout_sec=0.05)
    if n.x is None:
        print("ERROR: 收不到 /odom")
        return

    print("3 秒后开始, 请确认前方有 >=1.5m 空间!", flush=True)
    time.sleep(3)

    x0 = n.x
    print(f"[t0] 发送 vx=0.2 m/s, x0 = {x0:+7.3f}", flush=True)
    tw = Twist()
    tw.linear.x = 0.2
    n.pub.publish(tw)

    n.spin(5.0)
    x1 = n.x
    print(f"[5s] x1 = {x1:+7.3f}  (途中位移 {x1-x0:+7.3f} m)", flush=True)

    tw0 = Twist()          # 默认全 0
    n.pub.publish(tw0)
    print("[5s] 发送 vx=0, 等停稳 1.5s ...", flush=True)
    n.spin(1.5)
    x2 = n.x
    print(f"[停稳] x2 = {x2:+7.3f}", flush=True)

    print("\n======== 结果 ========", flush=True)
    print(f"  5s 名义位移: 0.2 × 5 = 1.000 m", flush=True)
    print(f"  odom 实际位移: {x2-x0:+7.3f} m   (偏差 {(x2-x0-1.0):+.3f} m)", flush=True)

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
