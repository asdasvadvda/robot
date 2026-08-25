#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
直行横向纠偏对比: 同一趟 4 段直行, 开环 vs 闭环(航向保持 + 横向纠偏) A/B 对比横向偏移。
默认 2m/段 (房间窄, 3m 会偏出走廊撞墙)。起步前先做激光预检, 前进/后退方向
空闲距离不足会自动拒绝起步, 防止撞墙。

用法(容器内):
    python3 /home/ubuntu/my_ros2_ws/scripts/drive_straight_ctrl.py                          # 2m × 4段
    python3 /home/ubuntu/my_ros2_ws/scripts/drive_straight_ctrl.py --dist 2.0               # 指定距离
    python3 /home/ubuntu/my_ros2_ws/scripts/drive_straight_ctrl.py --yaw-src ekf            # 用 EKF 航向做反馈
    python3 /home/ubuntu/my_ros2_ws/scripts/drive_straight_ctrl.py --k-xt 1.5 --max-xt 0.15 # 横向纠偏增益
    python3 /home/ubuntu/my_ros2_ws/scripts/drive_straight_ctrl.py --precheck-only          # 只读预检, 不动小车

流程:
    段1 前进 dist 开环 (wz=0) -> 段2 后退 dist 开环
    段3 前进 dist 闭环 (PID)  -> 段4 后退 dist 闭环
每段以"本段起点"为基准, 报告该段横向偏移(y_lat) 与 航向漂移 (不继承上一段偏移)。

闭环控制分两层:
  1) 横向纠偏 (Cross-track): 用 odom 位置算偏离本段起点直线的横向误差 e_xt,
     把航向设定点拉向回线方向: yaw_setpoint = yaw0 - dir * k_xt * e_xt (限幅 ±max_xt)。
  2) 航向保持: 对航向误差做 PD+积分: angular.z = Kp*err + Ki*∫err + Kd*(0 - yaw_rate)。
积分带限幅(抗饱和), 自动学出抵消机械偏置所需的常数 wz —— 即"前馈修正量"。

输出: /tmp/straight_ctrl.csv, 含每段样本 (t, phase, ctrl, x, y, yaw, err, cmd_wz)
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


class StraightCtrl(Node):
    def __init__(self, dist, speed, yaw_src, kp, ki, kd, max_wz, k_xt, max_xt,
                 margin=0.4, force=False):
        super().__init__('straight_ctrl')
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
        self.margin = margin
        self.force = force

        self.x = self.y = self.yaw = None
        self.fyaw = None
        self.scan = None          # 最新一帧 LaserScan
        self.t0 = 0.0
        self.samples = []

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
        self.fyaw = self.yaw_of(m)

    def scan_cb(self, m):
        self.scan = m

    def cur_yaw(self):
        return self.fyaw if self.yaw_src == 'ekf' else self.yaw

    # ---------- 激光预检 (只读, 不动小车) ----------
    def wait_scan(self, timeout=3.0):
        t0 = time.time()
        while self.scan is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.scan is not None

    def cone_min(self, ang_min, ang_max):
        """返回 [ang_min,ang_max] 范围内最近的有效障碍距离; 无有效点返回 None(视为无碍)"""
        s = self.scan
        best = None
        for i, r in enumerate(s.ranges):
            a = s.angle_min + i * s.angle_increment
            if not (ang_min <= a <= ang_max):
                continue
            if 0.0 < r < s.range_max:
                best = r if best is None else min(best, r)
        return best

    def scan_summary(self):
        """返回 前/后/左/右 四个锥形的最近障碍距离 dict。
        前/后只用 ±8° 直行窄锥, 避免走廊/墙角里侧墙的斜向投影误判。"""
        deg = math.radians
        fwd = self.cone_min(-deg(8), deg(8))
        bwd = self.cone_min(math.pi - deg(8), math.pi + deg(8))
        lft = self.cone_min(deg(65), deg(115))
        rgt = self.cone_min(-deg(115), -deg(65))
        return {'forward': fwd, 'backward': bwd, 'left': lft, 'right': rgt}

    def semicircle_min(self, center_angle):
        """[center±90°] 半球内最近有效障碍; 无有效点返回 None"""
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
        """起步前检查前进/后退方向空闲距离 >= dist+margin。返回 (ok, 说明str)"""
        if not self.wait_scan():
            return False, '收不到 /MS200/scan, 无法预检, 拒绝起步'
        sm = self.scan_summary()
        need = self.dist + self.margin
        issues = []
        for key, nm in [('forward', '前进'), ('backward', '后退')]:
            d = sm[key]
            if d is None:
                continue
            if d < need:
                issues.append(f'{nm}方向最近障碍 {d:.2f} m < 需要 {need:.2f} m')
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

    def log(self, phase, ctrl, err, cmd_wz):
        self.samples.append((time.time() - self.t0, phase, ctrl,
                             self.x, self.y,
                             math.degrees(self.yaw) if self.yaw is not None else 0.0,
                             math.degrees(err) if err is not None else 0.0,
                             cmd_wz))

    @staticmethod
    def lateral(x, y, x0, y0, yaw0):
        return -math.sin(yaw0) * (x - x0) + math.cos(yaw0) * (y - y0)

    def drive_leg(self, target, direction, closed_loop, x0, y0, yaw0, label):
        """跑一段 target 米。closed_loop=True 时 PID 保持航向 + 横向纠偏;
        返回 (里程, 横向偏移, 航向漂移, 末修正wz)"""
        # 本段参考基准 = 段起点位姿, 不继承上一段遗留偏移 (上一版误用全局起点, 混入了前段偏移)
        x0, y0 = self.x, self.y
        yaw0 = self.cur_yaw()
        px, py = self.x, self.y
        traveled = 0.0
        last = 0.0
        int_err = 0.0
        prev_yaw = self.cur_yaw()
        last_yaw_rate = 0.0
        cmd_wz = 0.0
        while traveled < target:
            # 运行时安全刹: 行驶方向的半球内最近障碍 < 0.25m 立即急停
            danger = self.semicircle_min(0.0 if direction > 0 else math.pi)
            if danger is not None and danger < 0.25:
                self.stop(0.5)
                self.log(label + '_SAFETY_STOP', 'ctrl' if closed_loop else 'open', 0.0, 0.0)
                print(f'  [SAFETY-STOP] {label}: {danger:.2f} m 太近, 已急停', flush=True)
                traveled = target  # 结束本段
                break
            yaw = self.cur_yaw()
            err = wrap_pi(yaw0 - yaw) if yaw is not None else 0.0
            if closed_loop and yaw is not None:
                dt = 0.02
                # 横向纠偏: 相对本段起点直线的横向误差 -> 航向设定点偏转向回线方向
                e_xt = self.lateral(self.x, self.y, x0, y0, yaw0)
                xt_ang = max(-self.max_xt, min(self.max_xt, self.k_xt * e_xt))
                yaw_setpoint = yaw0 - direction * xt_ang
                err = wrap_pi(yaw_setpoint - yaw)
                yaw_rate = wrap_pi(yaw - prev_yaw) / dt
                prev_yaw = yaw
                last_yaw_rate = 0.8 * last_yaw_rate + 0.2 * yaw_rate  # 平滑
                int_err += err * dt
                int_err = max(-0.3, min(0.3, int_err))               # 抗饱和限幅
                cmd_wz = self.kp * err + self.ki * int_err - self.kd * last_yaw_rate
                cmd_wz = max(-self.max_wz, min(self.max_wz, cmd_wz))
            else:
                cmd_wz = 0.0
            tw = Twist()
            tw.linear.x = direction * self.speed
            tw.angular.z = cmd_wz
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.x is None:
                continue
            traveled += math.hypot(self.x - px, self.y - py)
            px, py = self.x, self.y
            if time.time() - last >= 0.2:
                last = time.time()
                lat = self.lateral(self.x, self.y, x0, y0, yaw0)
                self.log(label, 'ctrl' if closed_loop else 'open', err, cmd_wz)
                print(f'  [{label}] t={time.time()-self.t0:5.2f}s  '
                      f'y_lat={lat:+6.3f} m  yaw={math.degrees(yaw):+6.2f}°  '
                      f'wz={cmd_wz:+6.3f} rad/s', flush=True)
        self.stop(0.5)
        self.log(label + '_stop', 'ctrl' if closed_loop else 'open', 0.0, 0.0)
        lat = self.lateral(self.x, self.y, x0, y0, yaw0)
        yaw_drift = math.degrees(self.cur_yaw()) - math.degrees(yaw0)
        return traveled, lat, yaw_drift, cmd_wz

    def run(self):
        self.get_logger().info(
            f'=== 航向闭环对比: 开环前{self.dist:.0f}m → 开环退{self.dist:.0f}m → '
            f'闭环前{self.dist:.0f}m → 闭环退{self.dist:.0f}m ===')
        self.get_logger().info(f'PID: Kp={self.kp} Ki={self.ki} Kd={self.kd} max_wz={self.max_wz}, '
                               f'yaw源={self.yaw_src}')
        self.get_logger().info(f'横向纠偏: k_xt={self.k_xt} /m, 航向偏转限幅 ±{math.degrees(self.max_xt):.1f}°'
                               ' (闭环段用 odom 位置反推横向偏差)')
        self.get_logger().info('3 秒后开始! Ctrl+C 中止')

        # 起步前激光预检 (只读): 前进/后退方向空闲距离不足则拒绝起步(--force 可放行, 运行时安全刹仍生效)
        ok, msg = self.precheck()
        self.get_logger().info(f'[precheck] 激光预检{msg}')
        if not ok:
            if self.force:
                self.get_logger().warn('--force: 预检未通过但放行, 靠运行时激光安全刹保护')
            else:
                self.get_logger().error('预检未通过, 拒绝起步。请先挪开小车或减小 --dist, 或加 --force')
                self.stop(0.3)
                return
        time.sleep(3)

        self.wait_pose()
        x0, y0 = self.x, self.y
        self.t0 = time.time()
        self.get_logger().info(f'[start] odom=({x0:+.3f},{y0:+.3f})')

        results = {}
        # 段1/2: 开环
        for label, direc in [('leg1_fwd_open', +1.0), ('leg2_bwd_open', -1.0)]:
            yaw0 = self.cur_yaw()
            results[label] = self.drive_leg(self.dist, direc, False, x0, y0, yaw0, label)
        # 段3/4: 闭环
        for label, direc in [('leg3_fwd_ctrl', +1.0), ('leg4_bwd_ctrl', -1.0)]:
            yaw0 = self.cur_yaw()
            results[label] = self.drive_leg(self.dist, direc, True, x0, y0, yaw0, label)

        # CSV
        csv_path = '/tmp/straight_ctrl.csv'
        with open(csv_path, 'w', newline='') as fp:
            w = csv.writer(fp)
            w.writerow(['t', 'phase', 'ctrl', 'odom_x', 'odom_y', 'odom_yaw_deg', 'yaw_err_deg', 'cmd_wz'])
            w.writerows(self.samples)
        print(f'\n[CSV] 已保存 {len(self.samples)} 个采样 -> {csv_path}', flush=True)

        # 汇总
        print('\n=========== 每段结果 (行驶距离 | 横向偏移 | 航向漂移) ===========', flush=True)
        names = [('leg1_fwd_open',  '段1 前进 开环'), ('leg2_bwd_open',  '段2 后退 开环'),
                 ('leg3_fwd_ctrl',  '段3 前进 闭环'), ('leg4_bwd_ctrl',  '段4 后退 闭环')]
        for key, nm in names:
            d, lat, yawd, wz = results[key]
            tag = '' if 'open' in key else f'  末修正wz={wz:+6.3f}'
            print(f'  {nm}: 行驶 {d:5.3f} m | 横向偏移 {lat:+6.3f} m | 航向漂移 {yawd:+6.2f}°{tag}', flush=True)

        open_lats = [abs(results['leg1_fwd_open'][1]), abs(results['leg2_bwd_open'][1])]
        ctrl_lats = [abs(results['leg3_fwd_ctrl'][1]), abs(results['leg4_bwd_ctrl'][1])]
        print('\n=========== 对比 ===========', flush=True)
        print(f'  开环 平均 |横向偏移|: {sum(open_lats)/2:.3f} m', flush=True)
        print(f'  闭环 平均 |横向偏移|: {sum(ctrl_lats)/2:.3f} m', flush=True)
        self.stop(0.5)


def main():
    p = argparse.ArgumentParser(description='航向闭环 vs 开环 直行 A/B 对比 (含激光预检)')
    p.add_argument('--dist', type=float, default=2.0, help='单向往返距离 m (默认2m)')
    p.add_argument('--speed', type=float, default=0.3)
    p.add_argument('--yaw-src', choices=['odom', 'ekf'], default='odom', help='反馈航向来源')
    p.add_argument('--kp', type=float, default=1.8, help='比例增益')
    p.add_argument('--ki', type=float, default=0.15, help='积分增益(学机械偏置)')
    p.add_argument('--kd', type=float, default=0.25, help='微分增益')
    p.add_argument('--max-wz', type=float, default=0.5, help='最大角速度 rad/s')
    p.add_argument('--k-xt', type=float, default=1.5, help='横向纠偏增益 1/m (闭环段, 传0=只做航向保持)')
    p.add_argument('--max-xt', type=float, default=0.15, help='横向纠偏最大航向偏转 rad')
    p.add_argument('--margin', type=float, default=0.4, help='预检安全余量 m')
    p.add_argument('--force', action='store_true',
                   help='预检未通过也放行起步(运行时激光安全刹仍生效)')
    p.add_argument('--precheck-only', action='store_true',
                   help='只读跑激光预检并打印周围空间, 不动小车')
    args = p.parse_args()

    rclpy.init()
    node = StraightCtrl(args.dist, args.speed, args.yaw_src,
                        args.kp, args.ki, args.kd, args.max_wz,
                        args.k_xt, args.max_xt, args.margin, args.force)
    try:
        if args.precheck_only:
            ok, msg = node.precheck()
            print(f'[precheck] 激光预检{msg}', flush=True)
            print('仅预检, 未动小车。通过与否: ', '通过' if ok else '未通过', flush=True)
        else:
            node.run()
    except KeyboardInterrupt:
        print('\n中断, 已尝试停车', flush=True)
    finally:
        node.stop(0.3)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
