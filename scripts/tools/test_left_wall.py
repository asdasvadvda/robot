#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
左侧玻璃墙贴纸测试 — 验证 MS200 能否把贴纸墙识别成一面"实线墙"。

用法(容器内, 先起雷达):
    ros2 launch oradar_lidar ms200_scan.launch.py
    # 另开一个终端:
    python3 /home/ubuntu/my_ros2_ws/scripts/test_left_wall.py
    python3 /home/ubuntu/my_ros2_ws/scripts/test_left_wall.py --center-deg 90 --width-deg 30 --sec 8

原理:
    玻璃会透红外, 雷达"看穿"到远处物体(返回 3~6m 甚至无回波);
    贴上白纸后应在原地返回一条距离一致的近墙线。
    因此判断白纸有没有用, 关键指标是"近墙占比":
        把窗口内所有点按中位距离分带, 落在 [0.8×中位, 1.25×中位] 内的算"近墙点"。
        若纸墙被看到, 近墙占比应接近 100% 且墙面平面残差 <= 3cm。
    (单看"有效点占比"不行——玻璃透过去也返回有效点, 但距离很远很散。)

判定标准:
    - 近墙占比 >= 85% 且 残差 <= 3cm  -> ✅ 白纸墙识别良好(整窗实线、平整)
    - 近墙占比低但残差小            -> 纸墙清晰, 只是纸比窗口窄(可缩小 --width-deg)
    - 近墙占比低且残差大            -> ❌ 窗口里没看到明显的近墙(纸没生效?)
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def ang_dist(a, b):
    """两角最短角距离(弧度, [0, pi])"""
    d = (a - b) % (2.0 * math.pi)
    return min(d, 2.0 * math.pi - d)


class WallTester(Node):
    def __init__(self, topic, center, width, sec, ring_bins):
        super().__init__('wall_tester')
        self.sub = self.create_subscription(LaserScan, topic, self.cb, 10)
        self.center = center
        self.width = width
        self.sec = sec
        self.ring_bins = ring_bins
        self.scan = None

    def cb(self, msg):
        self.scan = msg

    def wait_scan(self, timeout=10.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.scan is not None:
                return True
        return False

    def _window_points(self):
        """窗口内所有有效点 [(angle, range)]"""
        scan = self.scan
        half = self.width / 2.0
        rmin, rmax = scan.range_min, scan.range_max
        pts, total = [], 0
        for i, r in enumerate(scan.ranges):
            a = scan.angle_min + i * scan.angle_increment
            if ang_dist(a, self.center) <= half:
                total += 1
                if rmin < r < rmax:
                    pts.append((a, r))
        return pts, total

    def _plane_residual(self, pts, center):
        """墙面模型 r = d/cos(δ-δ0), 扫 δ0 求最小残差标准差"""
        best = None
        for k in range(-13, 14):
            d0 = k * 0.01                          # δ0: ±0.13 rad
            proj = [r * math.cos(a - center - d0) for a, r in pts]
            d = sorted(proj)[len(proj) // 2]       # 中位数抗噪
            if d <= 0:
                continue
            res = [r - d / math.cos(a - center - d0) for a, r in pts]
            sd = math.sqrt(sum(x * x for x in res) / len(res))
            if best is None or sd < best[0]:
                best = (sd, d)
        return best

    def window_stats(self):
        """返回: 有效占比 / 近墙占比 / 最大近墙连续簇(中心角·跨度·距离·残差)"""
        pts, total = self._window_points()
        valid_ratio = len(pts) / total if total else 0.0
        if not pts:
            return dict(valid_ratio=0.0, near_ratio=0.0, run=None, med_r=None)

        ranges = [r for _, r in pts]
        med = sorted(ranges)[len(ranges) // 2]
        lo, hi = 0.8 * med, 1.25 * med
        near = sorted([(a, r) for a, r in pts if lo <= r <= hi])
        near_ratio = len(near) / len(pts) if pts else 0.0

        # 近墙点按角度分组为连续簇(相邻 >5° 断开), 取跨度最长簇
        runs = []
        cur = []
        for a, r in near:
            if cur and math.degrees(a - cur[-1][0]) > 5.0:
                runs.append(cur)
                cur = []
            cur.append((a, r))
        if cur:
            runs.append(cur)
        if runs:
            best = max(runs, key=lambda r: r[-1][0] - r[0][0])
            center = (best[0][0] + best[-1][0]) / 2.0
            span = math.degrees(best[-1][0] - best[0][0])
            resid, dist = self._plane_residual(best, center)
            run = dict(center_deg=math.degrees(center), span_deg=span,
                       dist=dist, residual=resid)
        else:
            run = None

        return dict(valid_ratio=valid_ratio, near_ratio=near_ratio,
                    run=run, med_r=med)

    def verdict(self, w):
        if w['run'] is None or w['near_ratio'] < 0.5:
            a = "❌ 窗口内没有明显近墙(纸没被看见?)"
        elif w['near_ratio'] >= 0.85:
            a = "✅ 整窗都是近墙(白纸墙识别良好)"
        else:
            a = "⚠️ 近墙占比低 — 纸可能比窗口窄, 可缩小 --width-deg 再看"
        res = w['run']['residual'] if w['run'] else None
        if res is None:
            b = "近墙点太少"
        elif res <= 0.03:
            b = "墙面平整 ✅"
        elif res <= 0.08:
            b = "轻度噪点"
        else:
            b = "噪点明显"
        return a, b

    # ---------- 环带图 ----------
    def ring_map(self):
        """36 扇区×10° 最近距离图, 标出前后左右与当前窗口中心"""
        scan = self.scan
        bins = self.ring_bins
        span = 2.0 * math.pi / bins
        mn = [float('inf')] * bins
        cnt = [0] * bins
        for i, r in enumerate(scan.ranges):
            if scan.range_min < r < scan.range_max:
                a = scan.angle_min + i * scan.angle_increment
                b = int(((a + math.pi) % (2.0 * math.pi)) / span)
                mn[b] = min(mn[b], r)
                cnt[b] += 1

        cells = []
        for b in range(bins):
            cells.append(f"{mn[b]:4.2f}" if cnt[b] else "  .  ")
        ring = " ".join(cells)

        def pos_of(deg):
            return int(round((math.radians(deg) + math.pi)
                             % (2.0 * math.pi) / span))

        marks = {pos_of(0): "F", pos_of(90): "L", pos_of(180): "B",
                 pos_of(270): "R"}
        row1 = [" "] * bins
        row2 = [" "] * bins
        for b in marks:
            row1[b] = marks[b]
        row2[pos_of(math.degrees(self.center))] = "^"

        def render(row):
            return " ".join(row[b].center(5) for b in range(bins)).rstrip()

        return (f"环带图(每格10°, 数字=最近距离m, .=无回波)\n"
                f"  {render(row1)}\n"
                f"  {render(row2)}\n"
                f"  {ring}\n"
                f"  F=前(0°) L=左(90°) B=后(180°) R=右(270°)  ^=当前分析窗口")

    # ---------- 主循环 ----------
    def run(self):
        print(f"=== 左侧纸墙雷达测试: 窗口中心 {math.degrees(self.center):.0f}° ± "
              f"{math.degrees(self.width)/2:.0f}° ===", flush=True)
        print("请保持小车静止, 纸墙在雷达可视范围内。每 1 秒刷新一次。", flush=True)
        deadline = time.time() + self.sec
        while time.time() < deadline:
            if self.scan is None:
                rclpy.spin_once(self, timeout_sec=0.1)
                continue
            w = self.window_stats()
            a, b = self.verdict(w)
            print("\n" + self.ring_map(), flush=True)
            if w['run'] is not None:
                print(f"  有效点 {w['valid_ratio']*100:4.0f}% | 近墙占比 "
                      f"{w['near_ratio']*100:4.0f}% | 近墙簇: 中心 "
                      f"{w['run']['center_deg']:3.0f}° 跨度 {w['run']['span_deg']:3.0f}° "
                      f"@ {w['run']['dist']:4.2f}m | 残差 {w['run']['residual']:5.3f}m",
                      flush=True)
            else:
                print(f"  有效点 {w['valid_ratio']*100:4.0f}% | 近墙占比 "
                      f"{w['near_ratio']*100:4.0f}% | 无近墙簇", flush=True)
            print(f"  判定: {a} | {b}", flush=True)
            t = time.time()
            while time.time() - t < 1.0:
                rclpy.spin_once(self, timeout_sec=0.1)
                if time.time() > deadline:
                    break


def main():
    parser = argparse.ArgumentParser(description='左侧纸墙雷达识别测试')
    parser.add_argument('--topic', default='/MS200/scan')
    parser.add_argument('--center-deg', type=float, default=90.0,
                        help='纸墙所在角度(度), 0=前 90=左 180=后 270=右')
    parser.add_argument('--width-deg', type=float, default=30.0,
                        help='分析窗口宽度(度)')
    parser.add_argument('--sec', type=float, default=8.0,
                        help='测试时长(秒)')
    args = parser.parse_args()

    rclpy.init()
    node = WallTester(args.topic, math.radians(args.center_deg),
                      math.radians(args.width_deg), args.sec, ring_bins=36)
    try:
        if not node.wait_scan():
            print(f"ERROR: 10s 内收不到 {args.topic}, 雷达起来了吗?", flush=True)
            return
        print(f"已收到 {args.topic} (frame={node.scan.header.frame_id})", flush=True)
        node.run()
    except KeyboardInterrupt:
        print("\n中断", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
