#!/usr/bin/env python3
"""实测 STM32 旋转死区: 发 1.5s 旋转指令, 看 odom yaw 是否变化。"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class DeadZoneTest(Node):
    def __init__(self):
        super().__init__('deadzone_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom = None
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

    def odom_cb(self, msg):
        self.odom = msg

    def yaw(self):
        q = self.odom.pose.pose.orientation
        return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def wait_odom(self, sec):
        t0 = time.time()
        while self.odom is None and time.time() - t0 < 3:
            rclpy.spin_once(self, timeout_sec=0.2)

    def test(self, wz, dur=1.5):
        self.wait_odom(2)
        if self.odom is None:
            print('无 odom, 放弃'); return
        y0 = self.yaw()
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = wz
        t0 = time.time()
        while time.time() - t0 < dur:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.03)
        # 停
        msg.angular.z = 0.0
        for _ in range(10):
            self.pub.publish(msg)
            time.sleep(0.03)
        time.sleep(0.5)
        y1 = self.yaw()
        d = math.degrees(y1 - y0)
        print(f'wz={wz:+.2f} rad/s -> yaw {math.degrees(y0):+.1f}° -> {math.degrees(y1):+.1f}°, 变化 {d:+.2f}°')


def main():
    rclpy.init()
    node = DeadZoneTest()
    for wz in [0.15, 0.3, 0.5, 0.7]:
        node.test(wz)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
