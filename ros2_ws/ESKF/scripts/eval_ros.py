#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
#  评估 eskf_node 输出的 /odom_filtered（从录制 bag 读取）
#
#  与阶段5 的 ATE/RPE 定义一致：
#    ATE = 全程位置误差 RMSE（绝对轨迹误差）
#    RPE = 固定间隔 Δ 的位置增量误差 RMSE（相对位姿误差）
#
#  数据源 --source：
#    sim   = 仿真圆周真值（解析公式），对应 make_ros_bag.py --source sim
#    euroc = EuRoC GT CSV（数据到位后再启用）
#
#  用法：python3 scripts/eval_ros.py --bag data/odom_filtered_bag --source sim
# ============================================================
import argparse
import math
import os

import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import (SequentialReader, StorageOptions, ConverterOptions,
                        StorageFilter)
from nav_msgs.msg import Odometry


def read_odom_traj(bag_dir):
    """读回 bag 里的 /odom_filtered，返回 [(t, (x,y,z)), ...]（t 为相对起始秒）。"""
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_dir, storage_id="sqlite3"),
        ConverterOptions(input_serialization_format="cdr",
                         output_serialization_format="cdr"))
    reader.set_filter(StorageFilter(topics=["/odom_filtered"]))
    out = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, Odometry)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        out.append((t, (msg.pose.pose.position.x,
                        msg.pose.pose.position.y,
                        msg.pose.pose.position.z)))
    return out


def sim_truth(t):
    return (5.0 * math.cos(0.5 * t), 5.0 * math.sin(0.5 * t), 0.0)


def main():
    ap = argparse.ArgumentParser()
    base = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")
    ap.add_argument("--bag", default=os.path.join(base, "odom_filtered_bag"))
    ap.add_argument("--source", default="sim", choices=["sim", "euroc"])
    args = ap.parse_args()

    rclpy.init()
    try:
        traj = read_odom_traj(args.bag)
        if not traj:
            print("!! bag 里没有 /odom_filtered，先跑节点 + bag play + bag record")
            return
        t0 = traj[0][0]
        rel = [(tt - t0, p) for tt, p in traj]
        # 对齐到真值：对每个点按最近时间取真值
        tru, est = [], []
        for tt, p in rel:
            if args.source == "sim":
                tr = sim_truth(tt)
            else:
                raise SystemExit("--source euroc 需实现 GT CSV 读取（数据到位后加）")
            est.append(p); tru.append(tr)

        # ATE
        ate = math.sqrt(sum((sum((e[i]-t[i])**2 for i in range(3))
                             for e, t in zip(est, tru))) / len(est))
        # RPE：固定间隔 Δ≈1s
        dt_avg = (rel[-1][0] - rel[0][0]) / (len(rel) - 1) if len(rel) > 1 else 0.0
        delta = max(1, int(round(1.0 / dt_avg))) if dt_avg > 0 else 1
        rpe_s = 0.0; m = 0
        for i in range(len(est) - delta):
            d_e = (est[i+delta][0]-est[i][0], est[i+delta][1]-est[i][1], est[i+delta][2]-est[i][2])
            d_t = (tru[i+delta][0]-tru[i][0], tru[i+delta][1]-tru[i][1], tru[i+delta][2]-tru[i][2])
            rpe_s += sum((d_e[k]-d_t[k])**2 for k in range(3)); m += 1
        rpe = math.sqrt(rpe_s / m) if m else 0.0

        print(f"/odom_filtered 共 {len(est)} 点，时长 {rel[-1][0]-rel[0][0]:.1f}s")
        print(f"ATE = {ate:.3f} m    RPE(Δ≈1s) = {rpe:.3f} m")
        print(f"(对照：独立版阶段5 仿真数据 ATE≈0.3~0.4m)")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
