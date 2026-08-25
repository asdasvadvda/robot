#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
#  把数据写成 rosbag，供 eskf_node 测试
#
#  数据源（--source）：
#    sim   = 仿真数据 imu_sim.bin（IMU 100Hz）+ 解析圆周真值
#            先拿这个跑通节点全链路；EuRoC 数据到位后再加 euroc 分支
#
#  输出的 bag 话题：
#    /imu  (sensor_msgs/Imu)      全部 IMU（陀螺+比力）
#    /odom (nav_msgs/Odometry)    位置观测（5Hz，真值 + 可选噪声）
#
#  用法：source /opt/ros/humble/setup.bash
#        python3 scripts/make_ros_bag.py --source sim --out data/sim_bag
# ============================================================
import argparse
import os
import struct
import math
import csv

import rclpy
from rclpy.serialization import serialize_message
from rosbag2_py import (SequentialWriter, StorageOptions, ConverterOptions,
                        TopicMetadata)
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3


def load_sim_bin(path):
    """读 imu_sim.bin：N(int), dt(double), 然后 N 组 (gyro 3×d, acc 3×d)。"""
    with open(path, "rb") as f:
        N = struct.unpack("<i", f.read(4))[0]
        dt = struct.unpack("<d", f.read(8))[0]
        gyro, acc = [], []
        for _ in range(N):
            g = struct.unpack("<3d", f.read(24))
            a = struct.unpack("<3d", f.read(24))
            gyro.append(g); acc.append(a)
    return N, dt, gyro, acc


def truth_p(t):
    """仿真真值位置：半径 5m 圆周。"""
    return (5.0 * math.cos(0.5 * t), 5.0 * math.sin(0.5 * t), 0.0)


def truth_v(t):
    """仿真真值速度（世界系）。"""
    return (-5.0 * 0.5 * math.sin(0.5 * t), 5.0 * 0.5 * math.cos(0.5 * t), 0.0)


def make_imu(t, gyro, acc):
    m = Imu()
    # 头部时间戳：基址 + t*1e9（ns）。只求递增，基址随意。
    m.header.stamp.sec = BASE_NS // 10**9 + int(t)
    m.header.stamp.nanosec = int((t - int(t)) * 10**9)
    m.header.frame_id = "imu_link"
    m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = gyro
    m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = acc
    return m


def make_odom(t, sigma_obs):
    m = Odometry()
    m.header.stamp.sec = BASE_NS // 10**9 + int(t)
    m.header.stamp.nanosec = int((t - int(t)) * 10**9)
    m.header.frame_id = "world"
    m.child_frame_id = "base_link"
    px, py, pz = truth_p(t)
    import random
    m.pose.pose.position.x = px + random.gauss(0, sigma_obs)
    m.pose.pose.position.y = py + random.gauss(0, sigma_obs)
    m.pose.pose.position.z = pz + random.gauss(0, sigma_obs)
    th = 0.5 * t
    m.pose.pose.orientation.z = math.sin(th / 2)
    m.pose.pose.orientation.w = math.cos(th / 2)
    vx, vy, vz = truth_v(t)
    m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z = vx, vy, vz
    return m


BASE_NS = 1_600_000_000_000_000_000   # 基址时间戳（ns）


def build_sim_bag(imu_csv_like_path, out, obs_interval, sigma_obs):
    # 参数里传的是 imu_sim.bin 的路径（脚本复用接口，后续 euroc 分支再换）
    N, dt, gyro, acc = load_sim_bin(imu_csv_like_path)

    writer = SequentialWriter()
    writer.open(
        StorageOptions(uri=out, storage_id="sqlite3"),
        ConverterOptions(input_serialization_format="cdr",
                         output_serialization_format="cdr"))
    writer.create_topic(TopicMetadata(
        name="/imu", type="sensor_msgs/msg/Imu", serialization_format="cdr"))
    writer.create_topic(TopicMetadata(
        name="/odom", type="nav_msgs/msg/Odometry", serialization_format="cdr"))

    # 先写 /odom（低频），再写 /imu（高频）；bag 按时间戳排序，顺序无所谓
    # 但同一时间戳要求严格递增，这里错开写保证时间序
    t_obs = 0.0
    while t_obs < (N - 1) * dt:
        stamp = BASE_NS + int(t_obs * 1e9)
        writer.write("/odom", serialize_message(make_odom(t_obs, sigma_obs)), stamp)
        t_obs += obs_interval

    for i in range(N):
        t = i * dt
        stamp = BASE_NS + int(t * 1e9)
        writer.write("/imu", serialize_message(make_imu(t, gyro[i], acc[i])), stamp)

    print(f"已写 bag 到 {out}：{N} 条 /imu，{int((N-1)*dt/obs_interval)} 条 /odom")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="sim", choices=["sim"],
                    help="数据源（当前只有 sim；EuRoC 后加 euroc）")
    ap.add_argument("--out", default=None, help="bag 输出路径")
    ap.add_argument("--obs-interval", type=float, default=0.2,
                    help="位置观测间隔秒（5Hz=0.2）")
    ap.add_argument("--sigma-obs", type=float, default=0.3,
                    help="位置观测噪声 std (m)")
    args = ap.parse_args()

    base = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")
    out = args.out or os.path.join(base, "sim_bag")

    rclpy.init()
    try:
        if args.source == "sim":
            build_sim_bag(os.path.join(base, "imu_sim.bin"), out,
                          args.obs_interval, args.sigma_obs)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
