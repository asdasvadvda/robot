#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""轨迹记录器: 10Hz 采样 map->base_link 与 odom->base_link, 写 /tmp/traj_record.csv
用法: python3 traj_recorder.py [--seconds 300]  (Ctrl+C 停止)
CSV 列: t, mx, my, myaw_deg, ox, oy, oyaw_deg
"""
import csv
import math
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class Recorder(Node):
    def __init__(self, seconds):
        super().__init__('traj_recorder')
        self.buf = Buffer()
        self.lst = TransformListener(self.buf, self)
        self.t0 = time.time()
        self.seconds = seconds
        self.f = open('/tmp/traj_record.csv', 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'mx', 'my', 'myaw_deg', 'ox', 'oy', 'oyaw_deg'])
        self.timer = self.create_timer(0.1, self.tick)

    def tick(self):
        if time.time() - self.t0 > self.seconds:
            self.f.close()
            self.get_logger().info(f'记录完成, 共 {self.seconds}s, 存 /tmp/traj_record.csv')
            rclpy.shutdown()
            return
        row = [f'{time.time()-self.t0:.2f}']
        for frame in ('map', 'odom'):
            try:
                t = self.buf.lookup_transform(frame, 'base_link', rclpy.time.Time(),
                                              timeout=rclpy.duration.Duration(seconds=0.1))
                p = t.transform.translation
                q = t.transform.rotation
                yaw = math.degrees(2.0 * math.atan2(q.z, q.w))
                row += [f'{p.x:.3f}', f'{p.y:.3f}', f'{yaw:.2f}']
            except Exception:
                row += ['', '', '']
        self.w.writerow(row)


def main():
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('--seconds', type=float, default=300.0)
    args = p.parse_args()
    rclpy.init()
    node = Recorder(args.seconds)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.f.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
