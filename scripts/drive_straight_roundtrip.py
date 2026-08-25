#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双段往返直行精度测量: 闭环直行前进 --distance 米 -> 停 --pause 秒 -> 闭环直行
回退同样距离(沿同一参考直线) -> 停。全程测算 x/y 误差, 用于验收
"前进 3m y 值左右偏差不超过 3cm"。

背景: 键盘直行是开环的(只发 vx, 无反馈), 左右轮速差/打滑必然导致漂移;
本脚本用 odom 实时反馈做 航向保持 PID + 横向纠偏(cross-track),
把侧向偏差压到厘米级, 再用"前进-回退"闭环量化 x/y 误差。

测量内容:
  odom 系(车头参考系, 参考直线 = 原始起点直线, 回退沿同一条线退回):
    前进段: 路程(积分) / 沿直线距离(应≈dist) / 法向偏差(应≈0) / 航向漂移
    回退段: 同样
    回环:   回退结束回到起点后, odom x/y 相对原始起点的误差
  map 系(若 Nav2 运行, map->base_link 存在才测; 用户视角验收用):
    起点/终点 map 位姿, Δx / Δy / 法向偏差

用法(容器内, 驱动栈运行中; Nav2 可选):
    python3 /home/ubuntu/my_ros2_ws/scripts/drive_straight_roundtrip.py
    python3 /home/ubuntu/my_ros2_ws/scripts/drive_straight_roundtrip.py --distance 3 --speed 0.2
    python3 /home/ubuntu/my_ros2_ws/scripts/drive_straight_roundtrip.py --yaw-src ekf

安全: 起步前激光预检(前进方向空闲 >= dist+margin, 后退方向 >= margin);
行驶中行驶方向半球内 <0.25m 立即急停。Ctrl+C 发零速。
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
from sensor_msgs.msg import LaserScan


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class RoundtripMeasure(Node):
    def __init__(self, dist, speed, yaw_src, kp, ki, kd, max_wz, k_xt, max_xt,
                 pause, margin=0.4, force=False):
        super().__init__('drive_roundtrip_measure')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.f_cb, 10)
        self.create_subscription(LaserScan, '/MS200/scan', self.scan_cb, 10)
        self.yaw_src = yaw_src   # 'odom' 或 'ekf'
        self.dist = dist
        self.speed = speed
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_wz = max_wz
        self.k_xt = k_xt         # 横向纠偏增益 (1/m)
        self.max_xt = max_xt     # 横向纠偏最大航向偏转 (rad)
        self.pause = pause
        self.margin = margin
        self.force = force
        self.x = self.y = self.yaw = None
        self.fyaw = None
        self.scan = None
        self.samples = []
        self.t0 = time.time()

    # ---------------- 话题回调 ----------------
    def yaw_of(self, m):
        q = m.pose.pose.orientation
        return 2.0 * math.atan2(q.z, q.w)

    def odom_cb(self, m):
        self.x, self.y = m.pose.pose.position.x, m.pose.pose.position.y
        self.yaw = self.yaw_of(m)

    def f_cb(self, m):
        self.fyaw = self.yaw_of(m)

    def scan_cb(self, m):
        self.scan = m

    def cur_yaw(self):
        return self.fyaw if self.yaw_src == 'ekf' else self.yaw

    # ---------------- 激光预检 / 安全刹 ----------------
    def wait_scan(self, timeout=5.0):
        t0 = time.time()
        while self.scan is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.scan is not None

    def cone_min(self, ang_min, ang_max):
        s = self.scan
        if s is None:
            return None
        best = None
        for i, r in enumerate(s.ranges):
            a = s.angle_min + i * s.angle_increment
            if not (ang_min <= a <= ang_max):
                continue
            if 0.0 < r < s.range_max:
                best = r if best is None else min(best, r)
        return best

    def scan_summary(self):
        """前/后/左/右 四锥最近障碍; 前/后用 ±8° 窄锥防侧墙斜向误判"""
        deg = math.radians
        fwd = self.cone_min(-deg(8), deg(8))
        bwd = self.cone_min(math.pi - deg(8), math.pi + deg(8))
        lft = self.cone_min(deg(65), deg(115))
        rgt = self.cone_min(-deg(115), -deg(65))
        return {'forward': fwd, 'backward': bwd, 'left': lft, 'right': rgt}

    def semicircle_min(self, center_angle):
        s = self.scan
        if s is None:
            return None
        best = None
        for i, r in enumerate(s.ranges):
            a = s.angle_min + i * s.angle_increment
            if abs(wrap_pi(a - center_angle)) > math.pi / 2:
                continue
            if 0.0 < r < s.range_max:
                best = r if best is None else min(best, r)
        return best

    def precheck(self):
        """起步前检查: 前进方向空闲 >= dist+margin, 后退方向 >= margin"""
        if not self.wait_scan():
            return False, '收不到 /MS200/scan, 无法预检, 拒绝起步'
        sm = self.scan_summary()
        need = self.dist + self.margin
        issues = []
        if sm['forward'] is not None and sm['forward'] < need:
            issues.append(f'前进方向最近障碍 {sm["forward"]:.2f} m < 需要 {need:.2f} m')
        if sm['backward'] is not None and sm['backward'] < self.margin:
            issues.append(f'后退方向最近障碍 {sm["backward"]:.2f} m < 需要 {self.margin:.2f} m')
        side_vals = [d for d in (sm['left'], sm['right']) if d is not None]
        side = min(side_vals) if side_vals else None
        if side is not None and side < 0.25:
            issues.append(f'侧面空间过窄 {side:.2f} m')
        ok = not issues

        def fmt(d):
            return '∞' if d is None else f'{d:.2f}'

        msg = ('通过' if ok else '未通过: ' + '; '.join(issues)) + \
              f'  (前 {fmt(sm["forward"])} / 后 {fmt(sm["backward"])} / ' \
              f'左 {fmt(sm["left"])} / 右 {fmt(sm["right"])} m)'
        return ok, msg

    # ---------------- 工具 ----------------
    def wait_pose(self, timeout=5.0):
        t0 = time.time()
        while (self.x is None or self.cur_yaw() is None) and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.x is None or self.cur_yaw() is None:
            self.get_logger().error('收不到 /odom 或 /odometry/filtered, 先启动驱动栈')
            sys.exit(1)

    def stop(self, secs=0.5):
        end = time.time() + secs
        while time.time() < end:
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)

    @staticmethod
    def lateral(x, y, x0, y0, yaw0):
        """相对起点直线 (x0,y0,yaw0) 的法向偏差(左手系下右偏为正)"""
        return -math.sin(yaw0) * (x - x0) + math.cos(yaw0) * (y - y0)

    @staticmethod
    def along(x, y, x0, y0, yaw0):
        """相对起点直线 (x0,y0,yaw0) 的纵向距离"""
        return math.cos(yaw0) * (x - x0) + math.sin(yaw0) * (y - y0)

    # ---------------- 核心: 单段闭环直行 ----------------
    def drive_leg(self, target, direction, ref, label):
        """沿参考直线 ref=(x0,y0,yaw0) 直行 target 米(前进 direction=+1 / 回退 -1)。
        航向保持 PID + 横向纠偏。返回 (路程, 纵向距离, 法向偏差, 航向漂移)"""
        x0, y0, yaw0 = ref
        px, py = self.x, self.y
        traveled = 0.0
        int_err = 0.0
        prev_yaw = self.cur_yaw()
        last_yaw_rate = 0.0
        last_log = 0.0
        while traveled < target:
            # 运行时安全刹: 行驶方向半球内 < 0.25m 急停
            danger = self.semicircle_min(0.0 if direction > 0 else math.pi)
            if danger is not None and danger < 0.25:
                self.stop(0.5)
                print(f'  [SAFETY-STOP] {label}: {danger:.2f} m 太近, 已急停', flush=True)
                break
            yaw = self.cur_yaw()
            # 横向纠偏: 法向偏差 -> 航向设定点偏转向回线方向 (限幅 ±max_xt)
            e_xt = self.lateral(self.x, self.y, x0, y0, yaw0)
            xt_ang = max(-self.max_xt, min(self.max_xt, self.k_xt * e_xt))
            yaw_setpoint = yaw0 - direction * xt_ang
            err = wrap_pi(yaw_setpoint - yaw)
            dt = 0.02
            yaw_rate = wrap_pi(yaw - prev_yaw) / dt
            prev_yaw = yaw
            last_yaw_rate = 0.8 * last_yaw_rate + 0.2 * yaw_rate  # 平滑
            int_err += err * dt
            int_err = max(-0.3, min(0.3, int_err))                # 抗饱和
            cmd_wz = self.kp * err + self.ki * int_err - self.kd * last_yaw_rate
            cmd_wz = max(-self.max_wz, min(self.max_wz, cmd_wz))
            tw = Twist()
            tw.linear.x = direction * self.speed
            tw.angular.z = cmd_wz
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.x is None:
                continue
            traveled += math.hypot(self.x - px, self.y - py)
            px, py = self.x, self.y
            if time.time() - last_log >= 0.2:
                last_log = time.time()
                lat = self.lateral(self.x, self.y, x0, y0, yaw0)
                lon = self.along(self.x, self.y, x0, y0, yaw0)
                self.samples.append((time.time() - self.t0, label, lon, lat,
                                     math.degrees(yaw), cmd_wz))
                print(f'  [{label}] t={time.time()-self.t0:5.2f}s  '
                      f'沿直线={lon:+6.3f} m  法向={lat:+6.3f} m  '
                      f'yaw={math.degrees(yaw):+6.2f}°  wz={cmd_wz:+6.3f} rad/s', flush=True)
        self.stop(0.5)
        lat = self.lateral(self.x, self.y, x0, y0, yaw0)
        lon = self.along(self.x, self.y, x0, y0, yaw0)
        yaw_drift = math.degrees(wrap_pi(self.cur_yaw() - yaw0))
        return traveled, lon, lat, yaw_drift

    # ---------------- map 系采样(可选, Nav2 运行才有) ----------------
    def sample_map_pose(self, timeout=3.0):
        """tf map->base_link, 成功返回 (x, y, yaw_deg), 失败返回 None"""
        from tf2_ros import Buffer, TransformListener
        from rclpy.duration import Duration
        buf = Buffer()
        lst = TransformListener(buf, self)
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                t = buf.lookup_transform('map', 'base_link', rclpy.time.Time(),
                                         timeout=Duration(seconds=0.3))
                p = t.transform.translation
                q = t.transform.rotation
                yaw = 2.0 * math.atan2(q.z, q.w)
                return (p.x, p.y, math.degrees(yaw))
            except Exception:
                pass
        return None

    # ---------------- 主流程 ----------------
    def run(self):
        self.get_logger().info(
            f'=== 往返直行测量: 前进 {self.dist:.0f}m (闭环) -> 停 {self.pause:.0f}s '
            f'-> 回退 {self.dist:.0f}m (闭环) ===')
        self.get_logger().info(f'速度 {self.speed} m/s, PID: Kp={self.kp} Ki={self.ki} '
                               f'Kd={self.kd} max_wz={self.max_wz}, yaw源={self.yaw_src}, '
                               f'横向纠偏 k_xt={self.k_xt} max_xt={math.degrees(self.max_xt):.1f}°')

        # 起步前激光预检
        ok, msg = self.precheck()
        self.get_logger().info(f'[precheck] 激光预检{msg}')
        if not ok:
            if not self.force:
                self.get_logger().error('预检未通过, 拒绝起步 (--force 可放行, 运行时安全刹仍生效)')
                sys.exit(1)
            self.get_logger().warn('--force 放行起步')

        self.wait_pose()
        self.stop(0.5)

        # 锁存参考直线 = 原始起点位姿 (回退段也沿这条线, 才能测回环误差)
        x0, y0 = self.x, self.y
        yaw0 = self.cur_yaw()
        ref = (x0, y0, yaw0)
        self.get_logger().info(
            f'参考直线: 起点 odom ({x0:.3f}, {y0:.3f}) 航向 {math.degrees(yaw0):.2f}°')
        m0 = self.sample_map_pose(2.0)  # map 系起点(可选)
        if m0:
            self.get_logger().info(f'map 系起点: ({m0[0]:.3f}, {m0[1]:.3f}, {m0[2]:.1f}°)')

        print('3 秒后起步! Ctrl+C 中止', flush=True)
        time.sleep(3.0)

        try:
            # 段1: 前进
            self.get_logger().info(f'--- 段1: 前进 {self.dist:.0f} m ---')
            t1, lon1, lat1, yaw1 = self.drive_leg(self.dist, +1, ref, '前进')

            # 段间停
            self.get_logger().info(f'停 {self.pause:.0f}s ...')
            time.sleep(self.pause)

            # 段2: 回退 (沿同一参考直线, 退回原点)
            self.get_logger().info(f'--- 段2: 回退 {self.dist:.0f} m (退回起点) ---')
            t2, lon2, lat2, yaw2 = self.drive_leg(self.dist, -1, ref, '回退')
        finally:
            self.stop(1.0)

        self.stop(1.0)
        # 回环终点 (odom)
        xe, ye = self.x, self.y
        dx_loop, dy_loop = xe - x0, ye - y0
        # 回环终点 (map, 可选)
        m2 = self.sample_map_pose(3.0)
        if m2:
            dmx, dmy = m2[0] - m0[0], m2[1] - m0[1]
            yawm0 = math.radians(m0[2])
            lat_m = self.lateral(m2[0], m2[1], m0[0], m0[1], yawm0)
        else:
            dmx = dmy = lat_m = None

        # ---------- 汇总打印 ----------
        print('\n=========== 往返直行测量总结 (odom 系, 车头参考) ===========', flush=True)
        print(f'  参考直线: 起点 ({x0:+.3f}, {y0:+.3f})  航向 {math.degrees(yaw0):+.2f}°', flush=True)
        print(f'  前进段: 路程 {t1:.3f} m  沿直线 {lon1:+.3f} m  法向偏差 {lat1:+.3f} m  '
              f'航向漂移 {yaw1:+.2f}°', flush=True)
        print(f'  回退段: 路程 {t2:.3f} m  沿直线 {lon2:+.3f} m  法向偏差 {lat2:+.3f} m  '
              f'航向漂移 {yaw2:+.2f}°', flush=True)
        print(f'  回环误差(回到起点后):  Δx={dx_loop:+.3f} m   Δy={dy_loop:+.3f} m   '
              f'|Δ|={math.hypot(dx_loop, dy_loop):.3f} m', flush=True)

        if m0 and m2:
            print('\n=========== map 系 (用户视角, Nav2/AMCL) ===========', flush=True)
            print(f'  起点 map: ({m0[0]:+.3f}, {m0[1]:+.3f}, {m0[2]:+.1f}°)', flush=True)
            print(f'  回退后 map: ({m2[0]:+.3f}, {m2[1]:+.3f}, {m2[2]:+.1f}°)', flush=True)
            print(f'  回环误差:  Δx={dmx:+.3f} m   Δy={dmy:+.3f} m   '
                  f'法向偏差 {lat_m:+.3f} m   (目标: 前进3m y偏差 ≤ 0.03m)', flush=True)
        else:
            print('\n(Nav2 未运行或无 map->base_link, 不测 map 系; '
                  '需要时先起 nav_bringup 再测)', flush=True)

        # 落盘 CSV 供复盘
        path = '/tmp/drive_roundtrip.csv'
        try:
            with open(path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['t', 'phase', 'along', 'lat', 'yaw_deg', 'cmd_wz'])
                w.writerows(self.samples)
            print(f'\n采样已存 {path}', flush=True)
        except Exception as e:
            print(f'\n写 {path} 失败: {e}', flush=True)


def main():
    p = argparse.ArgumentParser(description='双段往返直行精度测量 (闭环直行前进-回退)')
    p.add_argument('--distance', type=float, default=3.0, help='单段距离 m (默认3)')
    p.add_argument('--speed', type=float, default=0.2, help='线速度 m/s (默认0.2)')
    p.add_argument('--pause', type=float, default=1.0, help='前进/回退之间停留秒数')
    p.add_argument('--yaw-src', choices=['odom', 'ekf'], default='odom',
                   help='航向反馈源 (默认 odom)')
    p.add_argument('--kp', type=float, default=1.8)
    p.add_argument('--ki', type=float, default=0.15)
    p.add_argument('--kd', type=float, default=0.25)
    p.add_argument('--max-wz', type=float, default=0.5, help='最大角速度 rad/s')
    p.add_argument('--k-xt', type=float, default=1.5, help='横向纠偏增益 1/m')
    p.add_argument('--max-xt', type=float, default=0.15, help='横向纠偏最大航向偏转 rad')
    p.add_argument('--margin', type=float, default=0.4, help='预检安全余量 m')
    p.add_argument('--force', action='store_true', help='预检未通过也放行起步')
    args = p.parse_args()

    rclpy.init()
    node = RoundtripMeasure(args.distance, args.speed, args.yaw_src, args.kp, args.ki,
                            args.kd, args.max_wz, args.k_xt, args.max_xt,
                            args.pause, args.margin, args.force)
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warn('手动中断')
        node.stop(1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
