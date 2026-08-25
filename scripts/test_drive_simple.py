#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
极简前进/后退测试：测小车能不能动起来
  前进 1m -> 停 1s -> 后退 1m -> 停
用法:
    python3 test_drive_simple.py              # 默认 0.25 m/s × 4s ≈ 1m
    python3 test_drive_simple.py --speed 0.2  # 自定义速度
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class MoveTest(Node):
    def __init__(self, speed, duration):
        super().__init__('move_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.cb, 10)
        self.speed = speed
        self.duration = duration
        self.x = None

    def cb(self, m):
        self.x = m.pose.pose.position.x

    def wait_odom(self, timeout=3.0):
        t0 = time.time()
        while self.x is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.x is None:
            print("ERROR: 收不到 /odom。先启动底盘驱动："
                  "ros2 launch robot_driver ekf_localization.launch.py", flush=True)
            sys.exit(1)

    def drive(self, secs, vx):
        deadline = time.time() + secs
        while time.time() < deadline:
            tw = Twist()
            tw.linear.x = vx
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.stop()

    def stop(self, secs=0.5):
        end = time.time() + secs
        while time.time() < end:
            self.pub.publish(Twist())      # 全零 = 停车
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    p = argparse.ArgumentParser(description='前进/后退 1m 测试')
    p.add_argument('--speed', type=float, default=0.25, help='线速度 m/s')
    p.add_argument('--dur', type=float, default=4.0, help='单向时长 s')
    args = p.parse_args()

    rclpy.init()
    n = MoveTest(args.speed, args.dur)

    print(f"3 秒后开始：前进 {args.speed} m/s × {args.dur}s ≈ {args.speed*args.dur:.2f}m，"
          f"请确认小车前方有 >=1.5m 空间！", flush=True)
    time.sleep(3)

    n.wait_odom()
    x0 = n.x
    print(f"[start]   x = {x0:+7.3f} m", flush=True)

    print("[forward] 前进中...", flush=True)
    n.drive(args.dur, +args.speed)
    x1 = n.x
    print(f"[pause]   停 1s. x = {x1:+7.3f}  前进 {x1-x0:+.3f} m", flush=True)
    time.sleep(1)

    print("[backward] 后退中...", flush=True)
    n.drive(args.dur, -args.speed)
    x2 = n.x
    print(f"[done]    停. x = {x2:+7.3f}  后退 {x2-x1:+.3f} m", flush=True)

    print("\n======== 结果 ========", flush=True)
    print(f"  前进位移: {x1-x0:+.3f} m  (期望 +{args.speed*args.dur:.2f})", flush=True)
    print(f"  后退位移: {x2-x1:+.3f} m  (期望 -{args.speed*args.dur:.2f})", flush=True)
    print(f"  净位移  : {x2-x0:+.3f} m  (期望 ≈0，即回到原点)", flush=True)

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
