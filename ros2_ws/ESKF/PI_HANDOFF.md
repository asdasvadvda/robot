# 树莓派端交接说明（给树莓派上的 Claude）

> 这是从电脑端（/home/wh/ESKF）交接过来的。**目标：把 ESKF ROS2 节点跑上真实小车。**

---

## 一、我在电脑端做了什么（背景）

完成了 ESKF 学习项目 stage0~6，最近完成的是 **stage6b：ROS2 节点端到端**。

- **共享核心** `include/eskf.hpp`：15 维误差状态 ESKF（δp δv δθ δb_g δb_a），`predict()` 接真实 dt，位置观测更新
- **ROS2 节点** `ros2_ws/src/eskf_ros/`：订阅 `/imu`（sensor_msgs/Imu）+ `/odom`（nav_msgs/Odometry）→ 发布 `/odom_filtered`
  - 用首条 `/odom` 的 位置+朝向+速度 初始化，之后只当位置观测
  - `predict` 在 `/imu` 回调、`update` 在 `/odom` 回调，按 header.stamp 时间序
- **launch** `ros2_ws/src/eskf_ros/launch/eskf_ros.launch.py`：一键端到端（node+record+play），已跑通
- **仿真验证结果**：ATE 0.33m / RPE 0.46m（纯积分 30m → ESKF 0.33m）

代码已全部推送到 GitHub：`https://github.com/asdasvadvda/ESKF.git`（**私有仓**）

## 二、当前任务

**把 eskf_ros 节点接到树莓派小车上跑真实数据。** 小车配置：树莓派主控 + 2D 激光 + IMU + 轮式里程计。

用户还没连过车，**话题布局未知**。第一步是排查小车。

## 三、部署步骤（电脑端已完成推送，树莓派端从这里开始）

### 第 1 步：clone + 认证
```bash
# 私有仓需要认证，推荐 SSH key（或 HTTPS + PAT）
git clone git@github.com:asdasvadvda/ESKF.git   # SSH
# 或
git clone https://github.com/asdasvadvda/ESKF.git  # HTTPS 需要 PAT
```

### 第 2 步：排查小车话题（第一优先，决定后续所有改动）
```bash
ros2 topic list                # 看有哪些话题
ros2 topic info /imu -v        # IMU 话题（类型、频率）
ros2 topic info /odom -v       # 里程计话题
ros2 topic echo /imu --once    # 看实际消息内容
```
**重点确认**：
- `/imu` 类型是不是 `sensor_msgs/Imu`（很多小车叫 `/imu/data`）
- `/odom` 类型是不是 `nav_msgs/Odometry`、位置稳不稳
- IMU 频率、噪声量级（决定节点参数）

### 第 3 步：对齐话题 + 部署
- 话题名不同 → launch 里 `remap`（推荐不改代码）或改节点
- 节点默认参数 `sigma_g=1.7e-4 sigma_a=2e-4` 是 200Hz EuRoC 密度，**小车要按实际 IMU 频率换算**（见踩坑1）
- 树莓派直接 `colcon build --packages-select eskf_ros`

### 第 4 步：验证
- RViz2 对比 `/odom`（纯里程计）vs `/odom_filtered`（ESKF），推车看平滑度
- **雷达 SLAM 当近似真值**（slam_toolbox 建图 → 位姿 vs ESKF 对比）—— 小车没有真值，这是评价方案

## 四、关键踩坑（务必看，全是血的教训）

1. **噪声密度换算（最狠）**：仿真/传感器给的往往是"每样本 σ"，ESKF 用 `Q = density²·dt`。**density = σ_s·√dt**（乘根号dt）。写反 Q 放大几万倍，滤波直接废。
   - 例：`imu_sim.bin` 是 100Hz，每样本 gyro σ=0.01 → density = 0.01×√0.01 = 0.001
2. **launch 的 `__file__` 坑**：launch 运行时 `__file__` 指向 install 副本，`../` 推导会错成 install 前缀 → 路径硬编码 repo_root
3. **play 播完立刻 shutdown 丢尾部**：record 会丢最后 0.4s，ATE 虚高。要延迟 2s 再 shutdown
4. **`ros2 bag play` 的绝对路径**会被 launch 重定位到 install 前缀，`ExecuteProcess` 要加 `cwd=repo_root`
5. **比力 vs 加速度**：IMU 的 `linear_acceleration` 是比力（静止时≈+9.81），节点里**不要**再减重力，ESKF 内部处理
6. 完整笔记在 `LEARNING_NOTES.md`，交接时也可以读

## 五、学习项目全貌

```
ESKF/
├── src/             独立版程序（stage0~6）
├── include/eskf.hpp 共享 ESKF 核心
├── scripts/         数据生成/评估/画图
├── ros2_ws/         ROS2 节点包（eskf_ros）
├── data/            数据（仿真 bag、euroc_sim）
└── LEARNING_NOTES.md 全部踩坑与结论
```

## 六、给树莓派端 Claude 的第一句

> "请帮我完成 ESKF ROS2 节点在小车上的实车验证。先看 PI_HANDOFF.md 了解背景，然后第一步是排查小车话题（ros2 topic list），我们一步步来。"
