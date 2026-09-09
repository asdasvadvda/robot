#!/usr/bin/env python3
"""
compare_odom.py — 对比 robot_localization EKF 与自己的 ESKF 融合输出

两个滤波器吃同样的输入(/odom + /imu/data),输出对比:
    /odometry/filtered  - robot_localization EKF(生产链路, Nav2 在用)
    /odom_filtered      - eskf_ros 自己写的 ESKF(候选替代)

两者同坐标系(odom)、同起点(原点)。**必须按消息时间戳对齐比较**,
不能按墙钟对齐:两个滤波器启动时刻不同,同一墙钟时刻累计的里程就不同;
同一时间戳 t 处,两个滤波器各自对"t 时刻车在哪"的估计才是公平对比。

用法(实车在线, 和 nav_bringup + eskf_node 同时跑):
    python3 /home/ubuntu/my_ros2_ws/scripts/compare_odom.py --plot out.png
车跑完(回到原点后)Ctrl+C,打印统计 + 存图。

用法(离线回放, 同一份数据喂两个滤波器, 最公平):
    bash /home/ubuntu/my_ros2_ws/scripts/compare_eskf_ekf.sh offline <bag> --plot out.png
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def yaw_from_quat(q):
    """Z 轴 yaw(2D, x/y 分量≈0)"""
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class OdomCompare(Node):
    def __init__(self, max_dt, timeout, plot_path, csv_path):
        super().__init__('compare_odom')
        self.max_dt = max_dt          # 时间戳对齐窗口(s)
        self.timeout = timeout        # 墙上秒数内无新数据 → 自动收尾退出
        self.plot_path = plot_path
        self.csv_path = csv_path
        self.seen = {'ekf': 0, 'eskf': 0, 'raw': 0}
        self.latest = {'ekf': None, 'eskf': None}   # (t, x, y, yaw)
        self.raw = None                               # (t, x, y)
        self.raw_len = 0.0
        self.rows = []  # [t, ekf_x, ekf_y, ekf_yaw, eskf_x, eskf_y, eskf_yaw, raw_len]
        self.last_wall = time.monotonic()
        self.finalized = False

        self.create_subscription(Odometry, '/odometry/filtered', self.on_ekf, 10)
        self.create_subscription(Odometry, '/odom_filtered', self.on_eskf, 10)
        self.create_subscription(Odometry, '/odom', self.on_raw, 10)
        self.create_timer(1.0, self.check_timeout)

    # ---- 各话题回调 ----
    def on_ekf(self, msg):
        self.seen['ekf'] += 1
        t = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        self.latest['ekf'] = (t, msg.pose.pose.position.x, msg.pose.pose.position.y,
                              yaw_from_quat(msg.pose.pose.orientation))
        self.try_pair(t)

    def on_eskf(self, msg):
        self.seen['eskf'] += 1
        t = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        self.latest['eskf'] = (t, msg.pose.pose.position.x, msg.pose.pose.position.y,
                               yaw_from_quat(msg.pose.pose.orientation))
        self.try_pair(t)

    def on_raw(self, msg):
        self.seen['raw'] += 1
        t = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if self.raw is not None:
            d = math.hypot(x - self.raw[1], y - self.raw[2])
            if d > 0.001:
                self.raw_len += d
        self.raw = (t, x, y)

    # ---- 配对: 同一时间戳附近的 ekf/eskf 各取最新一帧 ----
    def try_pair(self, t):
        self.last_wall = time.monotonic()
        a = self.latest['ekf']
        b = self.latest['eskf']
        if a is None or b is None:
            return
        dt = abs(a[0] - b[0])
        if dt > self.max_dt:
            return  # 两边还没对齐上(启动不同步), 等下一帧
        t = max(a[0], b[0])
        # 不重复记同一帧: 时间戳单调
        if self.rows and t - self.rows[-1][0] < 1e-4:
            return
        self.rows.append([t, a[1], a[2], a[3], b[1], b[2], b[3], self.raw_len])

    def check_timeout(self):
        if self.finalized:
            return
        if self.timeout > 0 and time.monotonic() - self.last_wall > self.timeout:
            self.get_logger().info(f'{self.timeout}s 没有新数据, 自动收尾')
            # 从 timer 回调里 rclpy.shutdown() 不会让 spin 返回(实测挂死),
            # 只抛异常让 spin 退出, 收尾(finish)由 main 的 finally 执行
            raise SystemExit

    # ---- 收尾: 统计 + CSV + 图 ----
    def finish(self):
        if self.finalized:
            return
        self.finalized = True
        self.print_summary()
        if self.csv_path:
            self.write_csv()
        if self.plot_path and self.rows:
            self.write_plot()
        rclpy.shutdown()

    def print_summary(self):
        print()
        print('=' * 62)
        print('EKF(robot_localization) vs ESKF(自己写) — 对比结果')
        print('=' * 62)
        s = self.seen
        print(f'收到帧数: EKF {s["ekf"]} | ESKF {s["eskf"]} | 原始 /odom {s["raw"]}')
        if not self.rows:
            miss = [t for t, c in s.items() if c == 0]
            print(f'!! 没有配到样本, 缺话题: {miss or "无(时间戳没对齐? 检查 max_dt)"}')
            print('   检查: nav_bringup 在跑吗? eskf_node 启动了吗?')
            return
        t0 = self.rows[0][0]
        ts = [r[0] - t0 for r in self.rows]
        pos_err = [math.hypot(r[1] - r[4], r[2] - r[5]) for r in self.rows]
        yaw_err = [abs(wrap_pi(r[3] - r[6])) * 180 / math.pi for r in self.rows]
        n = len(self.rows)
        mean_e = sum(pos_err) / n
        rms_e = math.sqrt(sum(e * e for e in pos_err) / n)
        print(f'配对样本 {n} 对 | 时间跨度 {ts[-1]:.1f}s | 对齐窗口 {self.max_dt * 1000:.0f}ms')
        print(f'位置误差: 均值 {mean_e * 100:.1f} cm | RMS {rms_e * 100:.1f} cm | 最大 {max(pos_err) * 100:.1f} cm')
        print(f'航向误差: 均值 {sum(yaw_err) / n:.1f}° | 最大 {max(yaw_err):.1f}°')
        last = self.rows[-1]
        print(f'--- 终点位姿(odom 系)与回原点漂移 ---')
        print(f'EKF : ({last[1]:.3f}, {last[2]:.3f})  距原点 {math.hypot(last[1], last[2]) * 100:.1f} cm')
        print(f'ESKF: ({last[4]:.3f}, {last[5]:.3f})  距原点 {math.hypot(last[4], last[5]) * 100:.1f} cm')
        print(f'原始里程计路径长 {self.raw_len:.1f} m(往返巡检后距原点应≈0, 越接近越不漂)')
        print('=' * 62)

    def write_csv(self):
        with open(self.csv_path, 'w') as f:
            f.write('t,ekf_x,ekf_y,ekf_yaw,eskf_x,eskf_y,eskf_yaw,pos_err_m,yaw_err_deg,raw_len_m\n')
            t0 = self.rows[0][0]
            for r in self.rows:
                pe = math.hypot(r[1] - r[4], r[2] - r[5])
                ye = abs(wrap_pi(r[3] - r[6])) * 180 / math.pi
                f.write(f'{r[0] - t0:.3f},{r[1]:.4f},{r[2]:.4f},{r[3]:.4f},'
                        f'{r[4]:.4f},{r[5]:.4f},{r[6]:.4f},{pe:.4f},{ye:.3f},{r[7]:.4f}\n')
        print(f'CSV 已存: {self.csv_path}')

    def write_plot(self):
        t0 = self.rows[0][0]
        ts = [r[0] - t0 for r in self.rows]
        pos_err = [math.hypot(r[1] - r[4], r[2] - r[5]) for r in self.rows]
        yaw_err = [abs(wrap_pi(r[3] - r[6])) * 180 / math.pi for r in self.rows]

        fig, axs = plt.subplots(3, 1, figsize=(10, 9), sharex=False)
        # 1) 轨迹叠加
        axs[0].plot([r[1] for r in self.rows], [r[2] for r in self.rows],
                    '-', color='#1f77b4', label='EKF /odometry/filtered')
        axs[0].plot([r[4] for r in self.rows], [r[5] for r in self.rows],
                    '--', color='#ff7f0e', label='ESKF /odom_filtered')
        axs[0].plot(0, 0, 'k+', ms=10, label='origin (0,0)')
        axs[0].set_aspect('equal')
        axs[0].set_title('trajectory in odom frame (same inputs, stamp-aligned)')
        axs[0].legend(loc='best', fontsize=9)
        axs[0].grid(alpha=0.3)
        # 2) 位置误差
        axs[1].plot(ts, pos_err, '-', color='k')
        axs[1].set_ylabel('pos error [m]')
        axs[1].set_title('EKF vs ESKF: position error')
        axs[1].grid(alpha=0.3)
        # 3) 航向误差
        axs[2].plot(ts, yaw_err, '-', color='#d62728')
        axs[2].set_ylabel('yaw error [deg]')
        axs[2].set_xlabel('time [s]')
        axs[2].set_title('EKF vs ESKF: yaw error')
        axs[2].grid(alpha=0.3)

        fig.tight_layout()
        fig.savefig(self.plot_path, dpi=120)
        print(f'对比图已存: {self.plot_path}')


def main():
    p = argparse.ArgumentParser(description='EKF vs ESKF 融合输出对比(按时间戳对齐)')
    p.add_argument('--max-dt', type=float, default=0.06,
                   help='时间戳对齐窗口秒(两滤波器都 ~50Hz, 默认 60ms)')
    p.add_argument('--timeout', type=float, default=10.0,
                   help='墙上秒数内无新数据自动收尾(离线回放结束时靠它退出); 0=只等 Ctrl+C')
    p.add_argument('--plot', default='',
                   help='输出对比图 PNG 路径(默认不存图)')
    p.add_argument('--csv', default='',
                   help='输出配对数据 CSV 路径(默认不存)')
    args = p.parse_args()

    rclpy.init()
    node = OdomCompare(args.max_dt, args.timeout, args.plot, args.csv)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.finalized:
            node.finish()


if __name__ == '__main__':
    main()
