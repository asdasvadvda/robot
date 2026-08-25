#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
3m 往返直行漂移诊断: 前进 3m -> 停 -> 后退 3m -> 停
重点观察过程中小车的横向偏移 (y) 与航向 (yaw) 漂移, 量化"走偏"到底有多少。

用法(容器内):
    python3 /home/ubuntu/my_ros2_ws/scripts/test_drive_3m_drift.py                  # 默认 0.3 m/s × 3m
    python3 /home/ubuntu/my_ros2_ws/scripts/test_drive_3m_drift.py --speed 0.2 --dist 3.0

输出:
    - 行驶全程每 0.2s 记录 odom 与 EKF(/odometry/filtered) 的 (x, y, yaw)
    - 终端实时打印 t / odom(x,y) / yaw / 横向偏移
    - 结束打印: 前进段横向偏移+航向漂移, 后退段回程偏移, 最终净位移
    - 完整数据存 CSV: /tmp/drift_3m.csv

横向偏移定义: 以起点航向为基准的坐标系里, 垂直于行驶方向的位移 = 偏离直线的距离
    y_lat = -sin(yaw0)*(x-x0) + cos(yaw0)*(y-y0)
"""
import argparse
import csv
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class DriftTest(Node):
    def __init__(self, dist, speed):
        super().__init__('drift_test_3m')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.f_cb, 10)
        self.dist = dist
        self.speed = speed

        self.x = self.y = self.yaw = None
        self.fx = self.fy = self.fyaw = None
        self.t0 = 0.0
        self.samples = []   # (t, phase, ox, oy, oyaw_deg, fx, fy, fyaw_deg)

    @staticmethod
    def yaw_of(m):
        q = m.pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def odom_cb(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        self.yaw = self.yaw_of(m)

    def f_cb(self, m):
        self.fx = m.pose.pose.position.x
        self.fy = m.pose.pose.position.y
        self.fyaw = self.yaw_of(m)

    def wait_pose(self, timeout=5.0):
        t0 = time.time()
        while (self.x is None or self.fx is None) and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.x is None or self.fx is None:
            self.get_logger().error('收不到 /odom 或 /odometry/filtered, 先启动驱动栈: '
                                    'ros2 launch robot_driver ekf_localization.launch.py')
            sys.exit(1)

    def stop(self, secs=0.5):
        end = time.time() + secs
        while time.time() < end:
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)

    def log(self, phase):
        self.samples.append((time.time() - self.t0, phase,
                             self.x, self.y, math.degrees(self.yaw) if self.yaw is not None else 0.0,
                             self.fx, self.fy, math.degrees(self.fyaw) if self.fyaw is not None else 0.0))

    @staticmethod
    def lateral(x, y, x0, y0, yaw0):
        """相对起点航向的横向偏移 (m), 正=向左偏, 负=向右偏"""
        return -math.sin(yaw0) * (x - x0) + math.cos(yaw0) * (y - y0)

    def drive_path(self, target, direction, label, x0, y0, yaw0):
        """按 odom 路径长度行驶到 target, 全程 0.2s 采样; 返回 (实际里程, 末横向偏移, 末航向偏移deg)"""
        px, py = self.x, self.y
        traveled = 0.0
        last = 0.0
        while traveled < target:
            tw = Twist()
            tw.linear.x = direction * self.speed
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.x is None:
                continue
            traveled += math.hypot(self.x - px, self.y - py)
            px, py = self.x, self.y
            if time.time() - last >= 0.2:
                last = time.time()
                lat = self.lateral(self.x, self.y, x0, y0, yaw0)
                self.log(label)
                print(f'  [{label}] t={time.time()-self.t0:5.2f}s  '
                      f'odom x={self.x:+6.3f} y={self.y:+6.3f} | '
                      f'y_lat={lat:+6.3f} | yaw={math.degrees(self.yaw):+6.2f}°', flush=True)
        self.stop(0.5)
        self.log(label + '_stop')
        lat = self.lateral(self.x, self.y, x0, y0, yaw0)
        return traveled, lat, math.degrees(self.yaw) - math.degrees(yaw0)

    def run(self):
        self.get_logger().info('=== 3m 往返漂移测试: 前进 3m -> 停 -> 后退 3m 回起点 ===')
        self.get_logger().info('3 秒后开始! 请确认小车前方 >= 5m 无障碍空间, 需要中止请 Ctrl+C')
        time.sleep(3)

        self.wait_pose()
        x0, y0, yaw0 = self.x, self.y, self.yaw
        self.t0 = time.time()
        self.get_logger().info(
            f'[start] odom=({x0:+.3f},{y0:+.3f}) yaw={math.degrees(yaw0):+.2f}°')
        self.log('start')

        # ---- 前进 3m ----
        f_dist, f_lat, f_yaw = self.drive_path(self.dist, +1.0, 'fwd', x0, y0, yaw0)
        xf, yf, yawf = self.x, self.y, self.yaw
        print(f'  [fwd-stop] 行驶 {f_dist:.3f} m | 横向偏移 {f_lat:+.3f} m | '
              f'航向漂移 {f_yaw:+.2f}°', flush=True)

        # ---- 停 1s 采样 ----
        self.stop(1.0)
        self.log('pause')
        print(f'  [pause] 停 1s. yaw={math.degrees(self.yaw):+.2f}°', flush=True)

        # ---- 后退 3m ----
        b_dist, b_lat, b_yaw = self.drive_path(self.dist, -1.0, 'bwd', x0, y0, yaw0)
        xb, yb, yawb = self.x, self.y, self.yaw
        print(f'  [bwd-stop] 行驶 {b_dist:.3f} m | 相对起点的横向偏移 {b_lat:+.3f} m | '
              f'航向偏差 {b_yaw:+.2f}°', flush=True)
        self.log('end')

        # ---- 写 CSV ----
        csv_path = '/tmp/drift_3m.csv'
        with open(csv_path, 'w', newline='') as fp:
            w = csv.writer(fp)
            w.writerow(['t', 'phase', 'odom_x', 'odom_y', 'odom_yaw_deg',
                        'ekf_x', 'ekf_y', 'ekf_yaw_deg'])
            w.writerows(self.samples)
        print(f'\n[CSV] 已保存 {len(self.samples)} 个采样 -> {csv_path}', flush=True)

        # ---- 汇总 ----
        net = math.hypot(xb - x0, yb - y0)
        net_lat = self.lateral(xb, yb, x0, y0, yaw0)
        # EKF 同样指标
        ef_lat = self.lateral(self.fx, self.fy, x0, y0, yaw0)  # 末点(odom 与 ekf 末点几乎同步)
        print('\n=========== 结果汇总 (odom / EKF) ===========', flush=True)
        print(f'  前进段  横向偏移 : odom {f_lat:+6.3f} m      (向右为负, 向左为正)', flush=True)
        print(f'  前进段  航向漂移 : odom {f_yaw:+6.2f}°', flush=True)
        print(f'  后退段  相对起点: odom {b_lat:+6.3f} m  (期望≈0 = 回到出发直线)', flush=True)
        print(f'  最终    航向偏差 : odom {b_yaw:+6.2f}°', flush=True)
        print(f'  终点    净位移   : {net:+6.3f} m  (期望≈0 = 回到原点)', flush=True)
        print(f'  终点    横向偏移 : odom {net_lat:+6.3f} m', flush=True)
        # EKF 对比
        f_fyaw_deg = math.degrees(self.fyaw) - math.degrees(yaw0)
        self.get_logger().info('对比 EKF: 前进段横向 EKF 见 CSV; '
                               '终点横向(≈) {:.3f} m / 航向偏差 {:.2f}°'.format(ef_lat, f_fyaw_deg))
        self.stop(0.5)


def main():
    p = argparse.ArgumentParser(description='3m 往返直行漂移诊断')
    p.add_argument('--dist', type=float, default=3.0, help='单向往返距离 m')
    p.add_argument('--speed', type=float, default=0.3, help='线速度 m/s')
    args = p.parse_args()

    rclpy.init()
    node = DriftTest(args.dist, args.speed)
    try:
        node.run()
    except KeyboardInterrupt:
        print('\n中断, 已尝试停车', flush=True)
    finally:
        node.stop(0.3)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
