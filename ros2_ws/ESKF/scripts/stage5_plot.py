# 阶段5 评估画图：轨迹对比 + 误差曲线
# 运行：python scripts/stage5_plot.py
# 依赖：matplotlib（本机已装）

import csv
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

# ---- 数据/输出目录：以脚本位置推断，不依赖当前工作目录 ----
BASE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")

# ---- 中文字体回退（Windows 黑体 / Linux Noto/WQY），避免中文变方块 ----
for fp in ["C:/Windows/Fonts/simhei.ttf",
           "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
           "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc"]:
    if os.path.exists(fp):
        font_manager.fontManager.addfont(fp)
        break
plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "Noto Sans CJK SC",
                                   "WenQuanYi Zen Hei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False   # 负号正常显示

# ---- 读数据 ----
rows = []
with open(os.path.join(BASE, "traj.csv")) as f:
    for r in csv.DictReader(f):
        rows.append(r)

t      = np.array([float(r["t"])              for r in rows])
tru_x  = np.array([float(r["true_x"])         for r in rows])
tru_y  = np.array([float(r["true_y"])         for r in rows])
pure_x = np.array([float(r["pure_x"])         for r in rows])
pure_y = np.array([float(r["pure_y"])         for r in rows])
eskf_x = np.array([float(r["eskf_x"])         for r in rows])
eskf_y = np.array([float(r["eskf_y"])         for r in rows])

# 误差随时间
pure_err = np.sqrt((pure_x - tru_x)**2 + (pure_y - tru_y)**2)
eskf_err = np.sqrt((eskf_x - tru_x)**2 + (eskf_y - tru_y)**2)

# ---- 图1：轨迹对比 ----
plt.figure(figsize=(7, 7))
plt.plot(tru_x,  tru_y,  "k-",  linewidth=2, label="真值轨迹 (半径 5m 圆周)")
plt.plot(pure_x, pure_y, "r--", linewidth=1.5, label="纯 IMU 积分 (漂移 31m)")
plt.plot(eskf_x, eskf_y, "b-",  linewidth=1.5, label="ESKF (IMU + 位置观测)")
plt.plot(tru_x[0],  tru_y[0],  "ks", markersize=8)
plt.plot(pure_x[0], pure_y[0], "r^", markersize=8)
plt.plot(eskf_x[0], eskf_y[0], "bs", markersize=8)
plt.text(tru_x[0] + 0.3, tru_y[0], "起点", fontsize=10)
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.title("纯 IMU 积分 vs ESKF：轨迹对比")
plt.legend(loc="best")
plt.axis("equal")
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(BASE, "traj_compare.png"), dpi=150)
print("已保存 data/traj_compare.png")

# ---- 图2：误差随时间 ----
plt.figure(figsize=(8, 4.5))
plt.plot(t, pure_err, "r-", linewidth=1.5, label="纯积分误差 (越飘越大)")
plt.plot(t, eskf_err, "b-", linewidth=1.5, label="ESKF 误差 (被观测按住)")
plt.xlabel("时间 (s)")
plt.ylabel("位置误差 (m)")
plt.title("位置误差随时间：漂移累积 vs 观测校正")
plt.legend(loc="best")
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(BASE, "err_over_time.png"), dpi=150)
print("已保存 data/err_over_time.png")
