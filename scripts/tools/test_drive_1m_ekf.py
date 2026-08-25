#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
前进 1m / 后退 1m 往返测试 — 同时记录 /odom 和 /odometry/filtered (EKF)
用法:
    python3 test_drive_1m_ekf.py                 # 默认: 0.25 m/s × 4.0 s ≈ 1m
    python3 test_drive_1m_ekf.py --speed 0.2 --dur 5.0
流程: 前进(speed) -> 停 -> 后退(-speed) -> 停
对比 odom 与 EKF 融合输出的位移/航向, 验证 vx 与 x 一致性、yaw 跟随。
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
        self.sub_f = self.create_subscription(
            Odometry, '/odometry/filtered', self.filtered_cb, 10)
        self.odom_x = self.odom_y = self.odom_yaw = None
        self.f_x = self.f_y = self.f_yaw = None
        self.odom_y_start = None
        self.f_y_start = None
        self.speed = speed
        self.duration = duration
        self.settle = settle

    @staticmethod
    def yaw_of(msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_cb(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_yaw = self.yaw_of(msg)

    def filtered_cb(self, msg):
        self.f_x = msg.pose.pose.position.x
        self.f_y = msg.pose.pose.position.y
        self.f_yaw = self.yaw_of(msg)

    def spin_loop(self, secs, label, vx):
        """发布 vx 持续 secs 秒, 期间按 5Hz 打印 odom + filtered"""
        deadline = time.time() + secs
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
                          f"odom x={self.odom_x:+7.3f} | f x={self.f_x:+7.3f}  "
                          f"odom yaw={math.degrees(self.odom_yaw):+6.1f}° | "
                          f"f yaw={math.degrees(self.f_yaw):+6.1f}°", flush=True)
        self.stop(0.3)

    def stop(self, secs):
        tw = Twist()
        deadline = time.time() + secs
        while time.time() < deadline:
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.02)

    def read_poses(self, label):
        for _ in range(30):
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom_x is not None and self.f_x is not None:
                return
        print("ERROR: 收不到 /odom 或 /odometry/filtered", flush=True)
        sys.exit(1)

    def run(self):
        print(f"=== EKF 对比测试: 前进 {self.speed} m/s × {self.duration}s ≈ "
              f"{self.speed*self.duration:.2f}m ===", flush=True)
        print("3 秒后开始, 请确认小车前方有 >=1.5m 空间!", flush=True)
        time.sleep(3)
        self.t0 = time.time()

        self.read_poses('start')
        x0, f0 = self.odom_x, self.f_x
        self.odom_y_start, self.f_y_start = self.odom_y, self.f_y
        print(f"[start] odom x={x0:+7.3f} y={self.odom_y:+7.3f} | "
              f"f x={f0:+7.3f} y={self.f_y:+7.3f}", flush=True)

        print("[forward] 前进中...", flush=True)
        self.spin_loop(self.duration, 'FWD', +self.speed)
        self.read_poses('after-forward')
        x_fwd, f_fwd = self.odom_x, self.f_x
        print(f"[forward-done] odom Δx={x_fwd-x0:+7.3f} | EKF Δx={f_fwd-f0:+7.3f} m",
              flush=True)

        print("[pause] 停顿 1s...", flush=True)
        self.stop(1.0)

        print(f"[backward] 后退 {self.speed} m/s × {self.duration}s ...", flush=True)
        self.spin_loop(self.duration, 'BWD', -self.speed)
        self.read_poses('after-backward')
        x_bwd, f_bwd = self.odom_x, self.f_x
        print(f"[backward-done] odom Δx={x_bwd-x_fwd:+7.3f} | "
              f"EKF Δx={f_bwd-f_fwd:+7.3f} m", flush=True)

        print("\n=========== 结果汇总 ===========", flush=True)
        print(f"  前进位移  odom: {x_fwd-x0:+7.3f}   EKF: {f_fwd-f0:+7.3f}   (期望 +1.000)",
              flush=True)
        print(f"  后退位移  odom: {x_bwd-x_fwd:+7.3f}   EKF: {f_bwd-f_fwd:+7.3f}   (期望 -1.000)",
              flush=True)
        print(f"  净位移    odom: {x_bwd-x0:+7.3f}   EKF: {f_bwd-f0:+7.3f}   (期望 0.000)",
              flush=True)
        print(f"  终点 y 差  odom: {self.odom_y - self.odom_y_start:+7.3f}   "
              f"EKF: {self.f_y - self.f_y_start:+7.3f}", flush=True)
        print(f"  终点 yaw   odom: {math.degrees(self.odom_yaw):+7.2f}°   "
              f"EKF: {math.degrees(self.f_yaw):+7.2f}°   (期望 ~0°)", flush=True)
        self.stop(0.5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--speed', type=float, default=0.25)
    parser.add_argument('--dur', type=float, default=4.0)
    parser.add_argument('--settle', type=float, default=0.5)
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
