#!/usr/bin/env python3
"""
check_localization.py — 检查 AMCL 粒子云收敛程度(定位是否锁定)

这个 Nav2 版本(1.1.13)的 AMCL 在 /particle_cloud 上发布的是
nav2_msgs/msg/ParticleCloud(带权重的粒子),不是老的 PoseArray——
订阅错类型会一个字节都收不到。

打印粒子数、按权重统计的位置/朝向标准差。粒子云越集中 = 定位越确定:
    stddev < 0.2m  ≈ 已锁定,可以放心导航
    0.2~0.5m       ≈ 正在收敛,再动一动
    > 0.5m 或分布成多团 ≈ 还没想明白,继续走动

注意:AMCL 只有车在移动时才会更新并发布粒子云,静止时收不到是正常的。

用法:
    python3 /home/ubuntu/my_ros2_ws/scripts/check_localization.py
收到一帧粒子云后自动退出;多跑几次观察趋势。
"""
import argparse
import math

import rclpy
from rclpy.node import Node
from nav2_msgs.msg import ParticleCloud
from rclpy.qos import qos_profile_sensor_data  # AMCL 粒子云是 best_effort,默认可靠订阅收不到


class LocalizationCheck(Node):
    def __init__(self, keep_listening=0.0):
        super().__init__('check_localization')
        self.keep = keep_listening
        self.n_frames = 0
        self.sub = self.create_subscription(
            ParticleCloud, '/particle_cloud', self.on_cloud,
            qos_profile_sensor_data)
        if keep_listening > 0:
            self.create_timer(keep_listening, self.finish)

    def finish(self):
        if self.n_frames == 0:
            print(f'[{self.keep:.0f}s 内没收到粒子云——车要动起来 AMCL 才会发布]')
        rclpy.shutdown()

    def on_cloud(self, msg: ParticleCloud):
        ps = msg.particles
        if not ps:
            self.get_logger().warn('收到空粒子云?')
            return
        n = len(ps)
        wsum = sum(p.weight for p in ps)
        mx = sum(p.pose.position.x * p.weight for p in ps) / wsum
        my = sum(p.pose.position.y * p.weight for p in ps) / wsum
        # 朝向用加权圆周平均
        s = sum(math.sin(2 * math.atan2(p.pose.orientation.z, p.pose.orientation.w)) * p.weight
                for p in ps) / wsum
        c = sum(math.cos(2 * math.atan2(p.pose.orientation.z, p.pose.orientation.w)) * p.weight
                for p in ps) / wsum
        std_yaw = math.degrees(math.sqrt(max(0.0, -math.log(max(1e-9, math.hypot(s, c))))))
        vx = sum(((p.pose.position.x - mx) ** 2) * p.weight for p in ps) / wsum
        vy = sum(((p.pose.position.y - my) ** 2) * p.weight for p in ps) / wsum
        std_xy = math.sqrt(vx + vy)
        far = sum(1 for p in ps
                  if math.hypot(p.pose.position.x - mx, p.pose.position.y - my) > 1.0)
        verdict = ('已锁定 ✔' if std_xy < 0.2 else
                   '正在收敛,继续动一动' if std_xy < 0.5 else
                   '还没想明白,继续走动(先转圈再平移效果最好)')
        print(f'粒子 {n} 个 | 加权中心 ({mx:.2f}, {my:.2f}) | '
              f'位置 stddev {std_xy:.2f} m | 朝向 stddev {std_yaw:.0f}° | '
              f'离群粒子 {far} ({far * 100 // n}%) | {verdict}')
        self.n_frames += 1
        if self.keep <= 0:
            rclpy.shutdown()


def main():
    p = argparse.ArgumentParser(description='检查 AMCL 粒子云收敛程度')
    p.add_argument('--seconds', type=float, default=0.0,
                   help='持续监听秒数(车边动边看时用,如 --seconds 30);'
                        '默认收到一帧粒子云就退出')
    args = p.parse_args()
    rclpy.init()
    node = LocalizationCheck(keep_listening=args.seconds)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
