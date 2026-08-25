#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
键盘直行纠偏助手 (Nav2 场景): 让 teleop_twist_keyboard 的"纯直行"也带闭环纠偏,
达到"前进 3m 侧向偏差 <= 3cm"。

原理: 键盘是开环的(按 w 只发 vx, 无反馈), 左右轮速差/打滑必然漂移。
本节点插在键盘与底盘之间:
    键盘(remap) --> /cmd_vel_raw --> [本节点] --> /cmd_vel --> 底盘
检测"直行意图"(|vx|>=0.05 且 |wz|<=0.05)时, 锁存参考直线(进入直行瞬间的
位姿), 此后以 10Hz 持续输出: vx=键盘速度 + wz=航向保持PID + 横向纠偏;
键盘转向(|wz|>0.05)或停止(vx<0.05)时退出直行模式, 原样透传。
按住/点按 w 均可: 底盘"执行最后一条命令", 第一条直行命令后节点持续补发
纠偏速度, 松开(收到零速)即停。

反馈默认 map 系(tf map->base_link, Nav2/AMCL 运行), 无 map 帧自动回落
odom 系(tf odom->base_link)。参考直线与偏差都在反馈系里算, 所以
"地图上看到的 y 直" 就是闭环目标。

用法(容器内):
  1) (可选)起 Nav2: ros2 launch robot_driver nav_bringup.launch.py map:=<图>
  2) 起纠偏节点: python3 /home/ubuntu/my_ros2_ws/scripts/straighten_teleop.py
  3) 起键盘(必须 remap 到 cmd_vel_raw):
       ros2 run teleop_twist_keyboard teleop_twist_keyboard \
           --ros-args -r cmd_vel:=cmd_vel_raw
  4) 按 w 直行(自动纠偏); 转向键/空格/倒车原样透传。

参数: --frame map|odom (默认 map)  --rate 10
      --kp 1.8 --ki 0.15 --kd 0.25 --max-wz 0.5 --k-xt 1.5 --max-xt 0.15
      --safe-stop   可选: 行驶方向半球内 <0.25m 时急停并退出直行
      --log-every 2.0   直行中状态打印间隔秒数

注意: Nav2 有活动导航目标时别按键盘(会抢 /cmd_vel); 本节点只在键盘
消息到来后的直行期间发布, Nav2 空闲时互不干扰。
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class StraightenTeleop(Node):
    def __init__(self, frame, rate, kp, ki, kd, max_wz, k_xt, max_xt,
                 safe_stop, log_every):
        super().__init__('straighten_teleop')
        self.frame = frame
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_wz = max_wz
        self.k_xt, self.max_xt = k_xt, max_xt
        self.safe_stop = safe_stop
        self.log_every = log_every

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel_raw', self.cmd_cb, 10)
        if safe_stop:
            self.create_subscription(LaserScan, '/MS200/scan', self.scan_cb, 10)

        # tf 反馈 (map 或 odom -> base_link)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0 / rate, self.tick)

        # 直行状态
        self.straight = False     # 直行模式激活?
        self.ref = None           # 参考直线 (x0, y0, yaw0)
        self.dir = 1              # 前进/后退方向
        self.vx = 0.0             # 键盘直行速度
        self.int_err = 0.0
        self.prev_yaw = None
        self.last_yaw_rate = 0.0
        self.last_log = 0.0
        self.scan = None
        self.pose = None          # 反馈系最新位姿 (x, y, yaw)
        self.last_cmd_time = 0.0

    # ---------------- 回调 ----------------
    def scan_cb(self, m):
        self.scan = m

    def cmd_cb(self, msg):
        self.last_cmd_time = time.time()
        vx, wz = msg.linear.x, msg.angular.z
        straight_now = abs(vx) >= 0.05 and abs(wz) <= 0.05
        if straight_now:
            if not self.straight:
                self.get_logger().info(
                    f'进入直行模式: vx={vx:+.2f} (参考直线将在下个控制周期锁存)')
                self.straight = True
                self.int_err = 0.0
                self.last_yaw_rate = 0.0
            self.vx = vx
            self.dir = 1 if vx > 0 else -1
        else:
            if self.straight:
                self.get_logger().info(f'退出直行模式 (vx={vx:+.2f} wz={wz:+.2f}), 透传')
            self.straight = False
            self.ref = None
            self.pub.publish(msg)   # 转向 / 停止: 原样透传

    # ---------------- 反馈位姿 ----------------
    def sample_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.frame, 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
            p = t.transform.translation
            q = t.transform.rotation
            self.pose = (p.x, p.y, 2.0 * math.atan2(q.z, q.w))
            return True
        except Exception:
            return False

    # ---------------- 控制周期 (10Hz) ----------------
    def tick(self):
        if not self.straight:
            return
        if not self.sample_pose():
            return
        x, y, yaw = self.pose

        # 进入直行的第一个周期: 锁存参考直线
        if self.ref is None:
            self.ref = (x, y, yaw)
            self.prev_yaw = yaw
            self.get_logger().info(
                f'参考直线锁存: ({x:+.3f}, {y:+.3f}, {math.degrees(yaw):+.1f}°)  '
                f'反馈系={self.frame}')

        # 安全刹(可选): 行驶方向半球内 <0.25m 急停
        if self.safe_stop and self.scan is not None:
            center = 0.0 if self.dir > 0 else math.pi
            danger = self.semicircle_min(center)
            if danger is not None and danger < 0.25:
                self.pub.publish(Twist())
                self.straight = False
                self.ref = None
                self.get_logger().warn(f'[SAFETY-STOP] {danger:.2f} m 太近, 急停退出直行')
                return

        x0, y0, yaw0 = self.ref
        # 横向纠偏: 法向偏差 -> 航向设定点拉回参考直线 (限幅 ±max_xt)
        e_xt = -math.sin(yaw0) * (x - x0) + math.cos(yaw0) * (y - y0)
        xt_ang = max(-self.max_xt, min(self.max_xt, self.k_xt * e_xt))
        yaw_setpoint = yaw0 - self.dir * xt_ang
        err = wrap_pi(yaw_setpoint - yaw)
        dt = 0.1
        yaw_rate = wrap_pi(yaw - self.prev_yaw) / dt
        self.prev_yaw = yaw
        self.last_yaw_rate = 0.8 * self.last_yaw_rate + 0.2 * yaw_rate
        self.int_err += err * dt
        self.int_err = max(-0.3, min(0.3, self.int_err))
        cmd_wz = self.kp * err + self.ki * self.int_err - self.kd * self.last_yaw_rate
        cmd_wz = max(-self.max_wz, min(self.max_wz, cmd_wz))

        tw = Twist()
        tw.linear.x = self.dir * abs(self.vx)
        tw.angular.z = cmd_wz
        self.pub.publish(tw)

        if time.time() - self.last_log >= self.log_every:
            self.last_log = time.time()
            self.get_logger().info(
                f'  直行中: 法向偏差 {e_xt:+.3f} m  航向误差 {math.degrees(err):+.2f}°  '
                f'wz={cmd_wz:+.3f} rad/s')

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


def main():
    p = argparse.ArgumentParser(description='键盘直行纠偏助手 (Nav2 场景)')
    p.add_argument('--frame', choices=['map', 'odom'], default='map',
                   help='反馈坐标系 (默认 map, 无 map 帧自动回落 odom)')
    p.add_argument('--rate', type=float, default=10.0, help='控制频率 Hz')
    p.add_argument('--kp', type=float, default=1.8)
    p.add_argument('--ki', type=float, default=0.15)
    p.add_argument('--kd', type=float, default=0.25)
    p.add_argument('--max-wz', type=float, default=0.5, help='最大纠偏角速度 rad/s')
    p.add_argument('--k-xt', type=float, default=1.5, help='横向纠偏增益 1/m')
    p.add_argument('--max-xt', type=float, default=0.15, help='横向纠偏最大航向偏转 rad')
    p.add_argument('--safe-stop', action='store_true', help='行驶方向 <0.25m 急停')
    p.add_argument('--log-every', type=float, default=2.0, help='直行状态打印间隔 s')
    args = p.parse_args()

    rclpy.init()
    node = StraightenTeleop(args.frame, args.rate, args.kp, args.ki, args.kd,
                            args.max_wz, args.k_xt, args.max_xt,
                            args.safe_stop, args.log_every)
    node.get_logger().info(
        f'键盘直行纠偏助手启动: 反馈={args.frame}(无 map 自动回落 odom), '
        f'直行判定 |vx|>=0.05 且 |wz|<=0.05, '
        f'PID Kp={args.kp} Ki={args.ki} Kd={args.kd} k_xt={args.k_xt}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.pub.publish(Twist())
        node.get_logger().info('退出, 已发零速')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
