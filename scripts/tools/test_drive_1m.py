#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
前进 1m / 后退 1m 往返测试
用法:
    python3 test_drive_1m.py                 # 默认: 0.25 m/s × 4.0 s ≈ 1m
    python3 test_drive_1m.py --speed 0.2 --dur 5.0
流程: 前进(speed) -> 停 -> 后退(-speed) -> 停
通过 /odom 记录实际位移, 用于校验底盘速度标定与 odom 一致性。
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class DriveTester(Node):
    def __init__(self, speed, duration, settle):
        super().__init__('drive_tester')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10)
        self.odom_x = None
        self.odom_y = None
        self.odom_yaw = None
        self.odom_y_start = None
        self.speed = speed
        self.duration = duration
        self.settle = settle

    def odom_cb(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        # yaw 从四元数取
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.odom_yaw = math.atan2(siny_cosp, cosy_cosp)

    def spin_loop(self, secs, label, vx):
        """发布 vx 持续 secs 秒, 期间按 5Hz 打印 odom"""
        deadline = time.time() + secs
        rate = 0.05
        last_log = 0.0
        while time.time() < deadline:
            tw = Twist()
            tw.linear.x = vx
            tw.linear.y = 0.0
            tw.angular.z = 0.0
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.02)
            if time.time() - last_log >= 0.2:
                last_log = time.time()
                if self.odom_x is not None:
                    print(f"  [{label}] t={time.time()-self.t0:5.2f}s  "
                          f"odom x={self.odom_x:+7.3f}  y={self.odom_y:+7.3f}  "
                          f"yaw={math.degrees(self.odom_yaw):+7.1f}°", flush=True)
        # 停止
        self.stop(0.3)

    def stop(self, secs):
        tw = Twist()
        self.pub.publish(tw)
        deadline = time.time() + secs
        while time.time() < deadline:
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.02)

    def read_x(self, label):
        for _ in range(30):  # 最多等 1.5s 拿到新 odom
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom_x is not None:
                return self.odom_x
        print("ERROR: 收不到 /odom", flush=True)
        sys.exit(1)

    def run(self):
        print(f"=== 前进 {self.speed} m/s × {self.duration}s ≈ {self.speed*self.duration:.2f}m ===",
              flush=True)
        print(f"3 秒后开始, 请确认小车前方有 >=1.5m 空间!", flush=True)
        time.sleep(3)
        self.t0 = time.time()

        x0 = self.read_x('start')
        self.odom_y_start = self.odom_y
        print(f"[start] odom x = {x0:+7.3f}  (y={self.odom_y:+7.3f})", flush=True)

        print("[forward] 前进中...", flush=True)
        self.spin_loop(self.duration, 'FWD', +self.speed)
        x_fwd = self.read_x('after-forward')
        d_fwd = x_fwd - x0
        print(f"[forward-done] odom x = {x_fwd:+7.3f}  前进位移 Δx = {d_fwd:+7.3f} m", flush=True)

        print("[pause] 停顿 1s...", flush=True)
        self.stop(1.0)

        print(f"[backward] 后退 {self.speed} m/s × {self.duration}s ...", flush=True)
        self.spin_loop(self.duration, 'BWD', -self.speed)
        x_bwd = self.read_x('after-backward')
        d_bwd = x_bwd - x_fwd
        print(f"[backward-done] odom x = {x_bwd:+7.3f}  后退位移 Δx = {d_bwd:+7.3f} m", flush=True)

        print("\n=========== 结果汇总 ===========", flush=True)
        print(f"  起点 x0      : {x0:+7.3f}", flush=True)
        print(f"  前进位移 Δfwd : {d_fwd:+7.3f} m  (期望 +1.000)", flush=True)
        print(f"  后退位移 Δbwd : {d_bwd:+7.3f} m  (期望 -1.000)", flush=True)
        print(f"  净位移        : {x_bwd-x0:+7.3f} m  (期望 0.000, 越小越好)", flush=True)
        print(f"  终点 y 偏差   : {self.odom_y - self.odom_y_start:+7.3f} m (期望 0.000)", flush=True)
        self.stop(0.5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--speed', type=float, default=0.25, help='线速度 m/s')
    parser.add_argument('--dur', type=float, default=4.0, help='单向时长 s')
    parser.add_argument('--settle', type=float, default=0.5, help='停止停留 s')
    args = parser.parse_args()

    rclpy.init()
    node = DriveTester(args.speed, args.dur, args.settle)
    try:
        node.run()
    except KeyboardInterrupt:
        print("\n中断, 已尝试停车", flush=True)
    finally:
        node.stop(0.3)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
