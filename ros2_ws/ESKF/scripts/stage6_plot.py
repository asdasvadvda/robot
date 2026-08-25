#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
#  阶段6 画图：EuRoC（或同格式仿真）结果
#  读 stage6_euroc 导出的 data/euroc_traj.csv，画：
#    图1  3D 轨迹对比（真值 vs 纯积分 vs ESKF）
#    图2  位置误差随时间（漂移累积 vs 观测校正）
#    图3  偏置估计收敛（ESKF 估计 vs GT 真值）
#
#  用法：python3 scripts/stage6_plot.py
# ============================================================
import csv
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

BASE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")

# ---- 中文字体回退（Windows 黑体 / Linux Noto/Droid），避免中文变方块 ----
for fp in ["C:/Windows/Fonts/simhei.ttf",
           "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
           "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
           "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf"]:
    if os.path.exists(fp):
        try:
            font_manager.fontManager.addfont(fp)
        except Exception:
            pass   # .ttc 某些旧版 matplotlib 不支持，忽略
        break
# 注意：Linux Noto 用 ttc 注册后，matplotlib 里的字族名是 "Noto Sans CJK JP"（不是 SC）
plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei",
                                   "Noto Sans CJK JP", "Noto Sans CJK SC",
                                   "Droid Sans Fallback", "WenQuanYi Zen Hei",
                                   "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False   # 负号正常显示

csv_path = os.path.join(BASE, "euroc_traj.csv")
rows = []
with open(csv_path) as f:
    for r in csv.DictReader(f):
        rows.append(r)

t      = np.array([float(r["t"])       for r in rows])
gt     = np.array([[float(r[f"gt_{c}"])    for c in "xyz"] for r in rows])
pure   = np.array([[float(r[f"pure_{c}"])  for c in "xyz"] for r in rows])
eskf   = np.array([[float(r[f"eskf_{c}"])  for c in "xyz"] for r in rows])
bg_est = np.array([[float(r[f"bg_{c}"])    for c in "xyz"] for r in rows])
ba_est = np.array([[float(r[f"ba_{c}"])    for c in "xyz"] for r in rows])
# 真值偏置列（仿真数据里有；真实 EuRoC GT CSV 也带，但旧 CSV 可能没有 → 容错）
has_gt_bias = "gt_bg_x" in rows[0]
if has_gt_bias:
    bg_gt = np.array([[float(r[f"gt_bg_{c}"]) for c in "xyz"] for r in rows])
    ba_gt = np.array([[float(r[f"gt_ba_{c}"]) for c in "xyz"] for r in rows])

pure_err = np.linalg.norm(pure - gt, axis=1)
eskf_err = np.linalg.norm(eskf - gt, axis=1)

# ---- 图1：3D 轨迹对比 ----
fig = plt.figure(figsize=(8, 7))
ax = fig.add_subplot(111, projection="3d")
ax.plot(*gt.T,   "k-",  linewidth=2, label="真值轨迹")
ax.plot(*pure.T, "r--", linewidth=1.5, label="纯 IMU 积分 (漂移)")
ax.plot(*eskf.T, "b-",  linewidth=1.5, label="ESKF (IMU + 位置观测)")
ax.scatter(*gt[0],  color="k", marker="s", s=40, label="起点")
ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)")
ax.set_title("纯 IMU 积分 vs ESKF：3D 轨迹对比")
ax.legend(loc="best")
fig.tight_layout()
fig.savefig(os.path.join(BASE, "euroc_traj_compare.png"), dpi=150)
print("已保存 data/euroc_traj_compare.png")

# ---- 图2：位置误差随时间 ----
plt.figure(figsize=(8, 4.5))
plt.plot(t, pure_err, "r-", linewidth=1.5, label="纯积分误差 (越飘越大)")
plt.plot(t, eskf_err, "b-", linewidth=1.5, label="ESKF 误差 (被观测按住)")
plt.xlabel("时间 (s)")
plt.ylabel("位置误差 (m)")
plt.title("位置误差随时间：漂移累积 vs 观测校正")
plt.legend(loc="best")
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(BASE, "euroc_err_over_time.png"), dpi=150)
print("已保存 data/euroc_err_over_time.png")

# ---- 图3：偏置估计收敛（估计 vs 真值） ----
fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
for ax, est, gtt, title, lab in (
        (axes[0], bg_est, bg_gt, "陀螺偏置 b_g", "b_g"),
        (axes[1], ba_est, ba_gt, "加速度计偏置 b_a", "b_a")):
    for i, c in enumerate("xyz"):
        ax.plot(t, est[:, i], label=f"{lab}.{c} (估计)", linewidth=1.5)
        if has_gt_bias:
            ax.plot(t, gtt[:, i], "--", linewidth=1.0,
                    color=f"C{i}", alpha=0.6, label=f"{lab}.{c} (真值)")
    ax.set_xlabel("时间 (s)"); ax.set_ylabel("偏置")
    ax.set_title(f"{title}：ESKF 从 0 学起收敛到真值附近")
    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3)
fig.tight_layout()
fig.savefig(os.path.join(BASE, "euroc_bias.png"), dpi=150)
print("已保存 data/euroc_bias.png")
