# stage6_euroc.cpp 程序说明

> ESKF 在 EuRoC 真实数据（或同格式仿真数据）上运行。程序名对应 `src/stage6_euroc.cpp`。

---

## 一、程序调用图（函数级）

```
main()
 │
 ├── load_imu(imu_csv)                ──► std::vector<ImuSample>
 │      └── split_csv_line(line)     逐行切 CSV → 数字数组（表头/坏行返回空）
 │
 ├── load_gt(gt_csv)                  ──► std::vector<GtSample>
 │      └── split_csv_line(line)
 │
 ├── yaml_double(yaml, key, fallback) ──► 从 sensor.yaml 抠噪声密度 ×4
 │
 ├── 数据自检
 │      ├── 前 1s 比力 z 均值 ≈ ±9.81   （重力/坐标符号对不对）
 │      └── GT 前 1s 位置 z 几乎不变      （确认开头静止、可做初始化）
 │
 ├── eskf.reset(s0, np, 9.81)         ──► 初始化 ESKF（P 初值：偏置给大方差）
 │
 ├── build_measurements(gt, imu_t, 0.2, σ_obs, rng)
 │      └── lower_bound(imu_t, gt[j].t) ──► 按【时间戳】找最近 IMU 索引，造 5Hz 位置观测
 │
 └── for i in 1..imu.size():          ──► 主循环：每个 IMU 样本走一步
       │
       ├── dt = imu_t[i] - imu_t[i-1]    真实时间戳差值
       │      ├── dt <= 0   → 乱序/重复 → 拷贝上一步，continue
       │      └── dt > 0.1  → 大缺口封顶，防协方差爆掉
       │
       ├── eskf.predict(w, a, dt)     ①高频预测（名义积分 + P←F·P·Fᵀ+Q）
       │      └── [eskf.hpp] exp_map(δθ)、skew(·)（姿态指数映射、反对称阵）
       │
       ├── measures.find(i)           查本步有没有位置观测
       │      └── 有 → eskf.update(z) ②低频校正（新息→K→δx 叠回大状态→P收缩）
       │
       └── 纯积分基线（阶段1 对照，不除偏置不校正）
              ├── aW = qp * imu.a + (0,0,-9.81)
              ├── pp += vp*dt + ½·aW·dt²
              ├── vp += aW*dt
              └── qp = qp * exp_map(imu.w * dt)

       ↓  （循环结束后）

 sample_at_gt(eskf_traj, imu_t, gt)   ──► 估计轨迹重采样到 GT 时间轴
 sample_at_gt(pure_traj, imu_t, gt)        （两种轨迹/偏置对齐到同一时间轴）
 sample_at_gt(bg_hist,  imu_t, gt)
 sample_at_gt(ba_hist,  imu_t, gt)
       ↓
 ┌───────────────┐
 │   ate(est,tru) │ ──► ATE 绝对轨迹误差（逐点距离 RMSE）
 ├───────────────┤
 │ rpe(est,tru,Δ) │ ──► RPE 相对位姿误差（固定间隔 Δ≈1s 的增量误差）
 └───────────────┘
       ↓
 最大误差 = max ‖est - tru‖           （注意：是误差，不是离原点距离）
       ↓
 输出最终偏置估计 b_g / b_a
       ↓
 写 data/euroc_traj.csv（轨迹 + 偏置历史，供画图）
       ↓
 scripts/stage6_plot.py              读 CSV 画 3 张图（轨迹/误差/偏置收敛）
```

---

## 二、数据流（谁产生了什么，喂给谁）

```
 data/euroc/imu0/data.csv                     data/euroc/state_groundtruth_estimate0/data.csv
      │  7 字段：t, w(3), a(3)                             │  17 字段：t, p(3), q(4), v(3), b_g(3), b_a(3)
      ▼                                                    ▼
 ┌───────────┐                                        ┌───────────┐
 │ load_imu  │──► imu: [{t, w, a}]  (200Hz)           │ load_gt   │──► gt: [{t,p,q,v,b_g,b_a}]
 └───────────┘                                        └───────────┘
      │                                                     │
      │  提取时间轴 imu_t[i]                                  │
      │                                                     ▼
      │                                          build_measures(gt, imu_t, 0.2s)
      │                                                     │
      │                                              measures: map<IMU索引, 位置观测>
      │                                                     │
      ▼                                                     ▼
 ┌────────────────────────── 主循环 for i in 1..imu.size() ──────────────────────────┐
 │  imu[i].w, imu[i].a, dt ──► eskf.predict ──► eskf.state()                          │
 │                                  ▲                                                  │
 │                                  │         measures.find(i) ──有──► eskf.update(z) │
 │  imu[i].w, imu[i].a    ──► 纯积分（pp,vp,qp）   ← 两条轨迹各自维护                  │
 └───────────────────────────────────────────────────────────────────────────────────┘
      │                                   │
      ▼                                   ▼
 eskf_traj[i]                        pure_traj[i]
 bg_hist[i], ba_hist[i]
      │
      ▼ sample_at_gt（都对齐到 GT 时间轴）
 gt_p / eskf_at / pure_at / bg_at / ba_at
      │
      ▼
 ate / rpe / 最大误差  ──► 控制台
      │
      ▼
 euroc_traj.csv ──► stage6_plot.py ──► 三张 png
```

---

## 三、关键函数逐段说明

### 1. `split_csv_line(line)` → `vector<double>`
通用 CSV 行切分。剥 `#` 表头前缀；遇到非数字整行返回空，调用方跳过。**所有数据读取的地基。**

### 2. `load_imu(path)` → `vector<ImuSample>`
- 读 7 字段：`t(ns→s), w(3), a(3)`；`v.size() < 7` 的行跳过（表头/坏行防越界）
- 读完全量后检查时间戳**单调递增**（真实数据约定）

### 3. `load_gt(path)` → `vector<GtSample>`
- 读 17 字段：`t, p(3), q(4,wxyz), v(3), b_g(3), b_a(3)`；`v.size() < 17` 跳过
- `q` 是四元数 **4 个分量、w 在前**（写反会转反 —— 笔记里的经典坑）
- 真值里自带偏置 `b_g/b_a`，供画图对比"估计收敛到哪"

### 4. `yaml_double(path, key, fallback)` → `double`
手写 mini-yaml：按 `key:` 前缀取值。4 个噪声键各自读取，缺了用 `eskf.hpp` 默认值。

### 5. `build_measurements(gt, imu_t, obs_interval, σ_obs, rng)` → `map<size_t, Vector3d>`
**阶段6 的核心思想 —— 按时间戳对齐，不是按索引：**
- 每 `obs_interval=0.2s`（5Hz）从 GT 挑一个位置当观测
- `lower_bound(imu_t, gt[j].t)` 二分找**第一个 ≥ 该时刻的 IMU 索引**，再判断 `k-1` 是否更近，取最近的
- 观测 = `gt[j].p + N(0, σ_obs)`（位置传感器替身，σ=0.1m）
- 返回 `map<IMU索引, 位置>` —— 主循环里 `measures.find(i)` 直接查"本步有没有观测"
- 首样本已用于初始化，不再给观测

### 6. 主循环（`for i in 1..imu.size()`）
```
dt = imu_t[i] - imu_t[i-1]       真实 dt
if (dt <= 0)   拷贝上一步 continue      乱序/重复时间戳
if (dt > 0.1)  dt = 0.1                大缺口封顶，防 P 爆掉

eskf.predict(w, a, dt)                ① 预测（高频）
if (measures 里有 i) eskf.update(z)   ② 更新（低频 5Hz 校正）
eskf_traj[i] = eskf.state().p         记轨迹
bg_hist/ba_hist[i] = 偏置估计          记偏置历史
纯积分基线：aW→pp/vp→qp               阶段1 对照
```
- **ESKF 和纯积分吃同一套 `imu[i].w/a`**，区别只在 ESKF 除偏置 + 校正 → 差距全归功于 ESKF
- 纯积分是"最朴素 IMU 积分"，不除偏置不校正，用来演示漂移

### 7. `sample_at_gt(traj, imu_t, gt)` → `vector<Vector3d>`
把以 IMU 索引为下标的轨迹，按 GT 时间戳重采样成 GT 长度的数组，**让两条轨迹对齐到同一时间轴**再比较。用于：估计/纯积分轨迹 + 偏置历史（导出画图）。

### 8. `ate(est, tru)` / `rpe(est, tru, Δ)`
- **ATE**：`RMSE(est-tru)`，管"整条线离多远"，最敏感累计漂移
- **RPE**：`RMSE(Δ步增量差)`，Δ≈1s，管"局部步子稳不稳"
- 两者互补：平行偏移 1m → ATE 大、RPE 小

### 9. 导出 `euroc_traj.csv`
列：`t, gt轨迹(3), 纯积分轨迹(3), eskf轨迹(3), gt偏置(6), 估计偏置(6)`，全部对齐 GT 时间轴。`stage6_plot.py` 读它画图。

---

## 四、和 stage4 的对照（阶段6 的"真实数据"三处升级）

| 环节 | stage4（仿真二进制） | stage6（EuRoC CSV） |
|---|---|---|
| 数据 | `imu_sim.bin` 无时间戳 | CSV 带纳秒时间戳 |
| dt | 固定 `0.01` | 每步真实 `imu_t[i]-imu_t[i-1]` |
| 观测调度 | `obs_every` 数索引 | 按时间戳 `lower_bound` 找最近 IMU |
| 噪声参数 | 硬编码 | 从 `sensor.yaml` 读密度 |
| 姿态自检 | 无 | 静态 acc z ≈ +9.81 |
| 轨迹 | 圆形解析 | 任意真实飞行（数据到位后） |

**核心没变**：ESKF 的预测/更新数学、ATE/RPE 定义、纯积分对照 —— 全和 stage4/5 同源，只是"怎么拿 dt、怎么对齐观测"从"数索引"升级成"对时钟"。

---

## 五、运行方式

```bash
# 构建（CMake 里已注册 stage6_euroc 目标）
cmake --build build --target stage6_euroc

# 仿真测试（数据在 data/euroc_sim，EuRoC 同款格式）
python3 scripts/gen_euroc_sim.py      # 第一次先生成仿真数据
./build/stage6_euroc data/euroc_sim

# 真实数据（数据到位后）
./build/stage6_euroc data/euroc

# 画图
python3 scripts/stage6_plot.py
```

**仿真测试实测结果**（15s 圆周，5Hz 位置观测）：
| 指标 | 纯积分 | ESKF | 改善 |
|---|---|---|---|
| ATE | 30.20 m | 0.133 m | 227× |
| RPE | 5.27 m | 0.173 m | 30× |
| 最大误差 | 68.68 m | 0.334 m | — |
