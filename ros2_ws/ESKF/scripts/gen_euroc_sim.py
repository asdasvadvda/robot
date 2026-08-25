#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
#  生成"EuRoC 同款格式"的仿真数据，供 stage6_euroc 无改动测试
#
#  为什么需要它：stage6_euroc 是按 EuRoC 真实 CSV 的目录结构写的
#  （imu0/data.csv + sensor.yaml + state_groundtruth_estimate0/data.csv）。
#  在真实数据下载好之前，先用解析圆周轨迹 + 噪声 + 偏置
#  生成同格式数据，把 stage6 的"解析/调度/滤波/评估"全链路跑通。
#
#  与真实 EuRoC 的区别（诚实的仿真声明）：
#   ① 轨迹是解析圆周（stage0/4 同款），不是无人机机动
#   ② 时间戳带 ±10% 随机抖动，模拟真实 IMU 非均匀采样
#   ③ 噪声参数（sensor.yaml）与仿真注入的噪声同源，自洽
#   ④ GT 也是 200Hz（真实是 100Hz，但观测调度只看时间戳，无影响）
#
#  用法：python3 scripts/gen_euroc_sim.py
#        输出到 data/euroc_sim/（含 imu0/ 和 state_groundtruth_estimate0/）
# ============================================================
import os
import math
import random

OUT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "..", "data", "euroc_sim"))

# ---- 轨迹 / 采样参数（和 stage0 一致，便于和阶段4 结果对照） ----
DT_NOM = 0.005          # 标称采样周期 5ms = 200Hz
T      = 15.0           # 时长 15s
R_c    = 5.0            # 圆周半径
OMEGA  = 0.5            # 偏航角速度 rad/s
G      = 9.81

# ---- IMU 误差参数（注入到读数里，sensor.yaml 写同源"密度"） ----
b_g = (0.02, -0.03, 0.05)    # 陀螺偏置 rad/s（真实 EuRoC 里就有）
b_a = (0.10, -0.20, 0.10)    # 加速度计偏置 m/s²
sg_sample = 2e-3             # 陀螺白噪声每样本 std  rad/s
sa_sample = 5e-2             # 加速度计白噪声每样本 std  m/s²

random.seed(7)


def rand_normal(sigma):
    """Box-Muller：纯标准库实现高斯采样（避免依赖 numpy）。"""
    u = random.random()
    v = random.random()
    return sigma * math.sqrt(-2.0 * math.log(u)) * math.cos(2.0 * math.pi * v)


def rot_yaw(theta):
    """绕世界 z 轴的 body→world 旋转矩阵 R(θ)。"""
    c, s = math.cos(theta), math.sin(theta)
    return ((c, -s, 0.0),
            (s,  c, 0.0),
            (0.0, 0.0, 1.0))


def rot_vec(R, v):
    """旋转矩阵 R 作用到向量 v（手写 3x3 乘法，避免依赖）。"""
    return tuple(sum(R[i][k] * v[k] for k in range(3)) for i in range(3))


def quat_from_yaw(theta):
    """偏航 θ → 单位四元数 (w,x,y,z)。绕世界 z 轴，q = (cos θ/2, 0, 0, sin θ/2)。"""
    return (math.cos(theta / 2.0), 0.0, 0.0, math.sin(theta / 2.0))


def main():
    os.makedirs(os.path.join(OUT, "imu0"), exist_ok=True)
    os.makedirs(os.path.join(OUT, "state_groundtruth_estimate0"), exist_ok=True)

    # ---- 逐时刻生成：时间戳非均匀，轨迹按真实时间算 ----
    t = 0.0
    imu_rows, gt_rows = [], []
    while t < T:
        theta = OMEGA * t
        c, s = math.cos(theta), math.sin(theta)

        # 解析圆周：位置 / 速度 / 加速度（世界系）
        p = (R_c * c, R_c * s, 0.0)
        v = (-R_c * OMEGA * s, R_c * OMEGA * c, 0.0)
        a = (-R_c * OMEGA**2 * c, -R_c * OMEGA**2 * s, 0.0)

        # 姿态：机头沿切线（偏航 = 相位角），body→world 矩阵
        R_BW = rot_yaw(theta)
        R_WB = rot_yaw(-theta)                      # 转置：world→body
        q = quat_from_yaw(theta)

        # 理想 IMU 读数（机体系）
        #   陀螺：世界角速度 ω_W=(0,0,ω) 转到机体系（绕 z 转不变）
        #   比力：f_W = a - g，转到机体系
        g_W = (0.0, 0.0, -G)
        gyro = rot_vec(R_WB, (0.0, 0.0, OMEGA))
        f_B = rot_vec(R_WB, tuple(a[i] - g_W[i] for i in range(3)))

        # 注入偏置 + 白噪声
        gyro_noisy = tuple(gyro[i] + b_g[i] + rand_normal(sg_sample) for i in range(3))
        acc_noisy  = tuple(f_B[i]  + b_a[i] + rand_normal(sa_sample) for i in range(3))

        # 时间戳：200Hz 名义周期 + ±10% 随机抖动（模拟真实 IMU 非均匀）
        ns = int(round(t * 1e9))
        imu_rows.append((ns, gyro_noisy, acc_noisy))
        gt_rows.append((ns, p, q, v, b_g, b_a))

        t += DT_NOM * (1.0 + 0.1 * (2.0 * random.random() - 1.0))

    print(f"仿真样本：IMU {len(imu_rows)}，GT {len(gt_rows)}，时长 {t:.2f} s（200Hz 名义）")

    # ---- 写 IMU CSV（EuRoC 列布局） ----
    with open(os.path.join(OUT, "imu0", "data.csv"), "w") as f:
        f.write("#timestamp [ns],w_RS_S [rad s^-1],a_RS_S [m s^-2]\n")
        for ns, g, a in imu_rows:
            f.write(f"{ns},{g[0]:.9f},{g[1]:.9f},{g[2]:.9f},"
                    f"{a[0]:.6f},{a[1]:.6f},{a[2]:.6f}\n")

    # ---- 写 GT CSV（EuRoC 列布局：#timestamp,p,q(wxyz),v,b_g,b_a） ----
    with open(os.path.join(OUT, "state_groundtruth_estimate0", "data.csv"), "w") as f:
        f.write("#timestamp [ns],p_RS_R [m],q_RS [],v_RS_R [m s^-1],"
                "b_w_RS_S [rad s^-1],b_a_RS_S [m s^-2]\n")
        for ns, p, q, v, bg, ba in gt_rows:
            f.write(f"{ns},{p[0]:.6f},{p[1]:.6f},{p[2]:.6f},"
                    f"{q[0]:.9f},{q[1]:.9f},{q[2]:.9f},{q[3]:.9f},"
                    f"{v[0]:.6f},{v[1]:.6f},{v[2]:.6f},"
                    f"{bg[0]:.9f},{bg[1]:.9f},{bg[2]:.9f},"
                    f"{ba[0]:.6f},{ba[1]:.6f},{ba[2]:.6f}\n")

    # ---- 写 sensor.yaml（连续噪声"密度"，与注入每样本方差自洽） ----
    # 换算：连续白噪声 PSD = ρ²，Q 块 = ρ²·dt。
    # 仿真注入的是"每样本"标准差 σ_s，一步积分的速度/角度不确定度增量 = σ_s²·dt²，
    # 令 ρ²·dt = σ_s²·dt² → ρ = σ_s·√dt（乘 √dt，不是除！写反会放 Q 一万倍）
    dt = DT_NOM
    yaml_text = (
        f"# 仿真 IMU 噪声（由 gen_euroc_sim.py 生成，与注入值同源）\n"
        f"accelerometer_noise_density: {sa_sample * math.sqrt(dt):.9f}    # m/s^2/√Hz\n"
        f"accelerometer_random_walk: 0.0\n"
        f"gyroscope_noise_density: {sg_sample * math.sqrt(dt):.9f}        # rad/s/√Hz\n"
        f"gyroscope_random_walk: 0.0\n"
    )
    with open(os.path.join(OUT, "imu0", "sensor.yaml"), "w") as f:
        f.write(yaml_text)

    print(f"已生成到 {OUT}/")
    print("  imu0/data.csv, imu0/sensor.yaml, state_groundtruth_estimate0/data.csv")
    print("运行：build/stage6_euroc data/euroc_sim")


if __name__ == "__main__":
    main()
