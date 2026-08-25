# stage6b：ESKF 端到端小结

> ROS2 节点把 ESKF 从"算法函数"变成"可对接真实系统的模块"。
> 本文讲清楚：**端到端是什么意思**、**全链路是怎么跑起来的**、**每个环节干什么**。

---

## 一、端到端（end-to-end）是什么

**从"输入数据"到"最终结果"，中间不管多少环节，整条链路从头到尾真实走通一遍。**

两个"端"：
- **起点**：`data/sim_bag`（10s 仿真圆周数据）
- **终点**：`ATE = 0.33m / RPE = 0.46m`（滤波质量指标）

关键是：**这不是纸面分析，而是让真实进程把数据一步步传过去、算出来、录下来、再算指标。**

对比阶段4/5 的单机程序：

| | 阶段4/5 单机程序 | 阶段6b ROS2 端到端 |
|---|---|---|
| 数据怎么来 | 程序自己打开二进制文件 | `bag play` 按真实时间把消息发到网上 |
| 谁在滤波 | 程序内函数调用 | 独立节点，订阅话题 |
| 结果去哪 | 打印到控制台 | 发布成话题，谁都能订阅 |
| 评价 | 程序内算 | 独立 `eval_ros.py` 读录制结果算 |
| **真实感** | 分析上的 | **和真实机器人系统架构一模一样** |

> 真实机器人（无人机/自动驾驶）就是这样：IMU 和定位传感器各自发话题，一个滤波节点订阅它们、高频输出位姿，供导航/避障用。6b 就是真实"定位模块"的缩小版。

---

## 二、全链路图（4 个独立进程，话题通信）

```
[sim_bag] 播放进程              [eskf_node] 滤波进程              [record] 录制进程
   ┌──────────┐                    ┌──────────┐                    ┌──────────┐
   │ bag play │──/imu(100Hz)──►───│          │──/odom_filtered──►──│ bag record │
   │ (读bag)   │──/odom(5Hz)───►───│ ESKF 滤波 │                    │ (写bag)    │
   └──────────┘                    └──────────┘                    └──────────┘
       进程1                          进程2                           进程3
                                         │
                                         ▼
                                    eval_ros.py（读录制的 bag，算 ATE/RPE）
```

三个进程**独立运行、通过话题通信**，谁都不直接调用对方 —— ROS2 的松耦合。

---

## 三、每步具体做了什么

### 第 0 步：准备数据源（rosbag）
`scripts/make_ros_bag.py` 把仿真数据写成 **rosbag**（"录好的一堆带时间戳的消息"）：
- `/imu`（sensor_msgs/Imu）100Hz，1000 条
- `/odom`（nav_msgs/Odometry）5Hz，50 条，带位置噪声 σ=0.3、真值朝向/速度
- 每条带 `header.stamp` 时间戳

### 第 1 步：让数据"活"起来 —— `bag play`
`ros2 bag play data/sim_bag` **按真实时间**把消息一条条发到网络上：
- t=0.00s 发第一条 `/imu`，t=0.01s 发第二条……（真以 100Hz 间隔发）
- t=0.20s 发第一条 `/odom`，每 0.2s 一条

这模拟了一个真实 IMU + 一个真实位置传感器同时工作。

### 第 2 步：节点干活 —— `eskf_node`（`ros2_ws/src/eskf_ros/src/eskf_node.cpp`）
注册两个回调 + 一个发布者，消息一到自动触发：
```cpp
sub_imu_  = create_subscription<Imu>("imu", ...,
              [this](Imu::SharedPtr m) { on_imu(m); });     // /imu 一到 → 预测
sub_odom_ = create_subscription<Odometry>("odom", ...,
              [this](Odometry::SharedPtr m) { on_odom(m); }); // /odom 一到 → 更新
pub_      = create_publisher<Odometry>("odom_filtered", ...);
```
处理逻辑（和独立版 stage6 完全一样的数学，共用 `include/eskf.hpp`）：
1. **首条 `/odom`** → 用 位置+朝向+速度 初始化
2. **每条 `/imu`** → `eskf_.predict()` 预测一步 → 发布 `/odom_filtered`
3. **每条 `/odom`** → `eskf_.update()` 位置更新

所以 `/odom_filtered` 是 **100Hz** 的高频滤波结果（IMU 频率），比 5Hz 观测密 20 倍 —— 这就是"用低频观测按住高频 IMU 积分"。

### 第 3 步：录下结果 —— `bag record`
`ros2 bag record /odom_filtered` 订阅话题，把每条滤波结果存成新 bag（`data/odom_filtered_bag`，998 条 ≈ 10s）。

### 第 4 步：评价 —— `eval_ros.py`
读录制 bag → 还原轨迹 → 和真值（解析圆周）对比：
```
ATE = sqrt( mean( ||估计位置 - 真值位置||² ) )    绝对轨迹误差，管整条线离多远
RPE = 固定间隔Δ≈1s 的位置增量误差 RMSE           相对位姿误差，管局部步子稳不稳
```
结果：**ATE = 0.33m，RPE = 0.46m**，和独立版 stage6 一致（对照约 0.3~0.4m）。

---

## 四、为什么 launch 是一键的

四步是 4 个独立进程，手动要开几个终端。`launch`（`ros2_ws/src/eskf_ros/launch/eskf_ros.launch.py`）用 Python 编排进程启停：

```python
node    = Node(package='eskf_ros', executable='eskf_node', ...)       # 进程1：节点
record  = ExecuteProcess(cmd=['ros2','bag','record','-o',out_bag,'/odom_filtered'])  # 进程2
play    = ExecuteProcess(cmd=['ros2','bag','play', sim_bag])          # 进程3：播放
shutdown_after_play = RegisterEventHandler(
    OnProcessExit(target_action=play,
                  on_exit=[TimerAction(period=2.0, actions=[Shutdown()])]))  # 播完等2s退出
```

编排逻辑：
1. **先清旧 bag**（rosbag 不能覆盖已存在目录）
2. **启动节点 + record**
3. **延迟 3s** 播放（等 record 完成话题发现，否则漏录开头）
4. **play 播完 → 等 2s**（让 record 把尾部写完）→ **自动优雅退出**

> 这个 2s 是关键：play 一结束立刻 shutdown 会丢尾部（实测 958 点→ATE 虚高 1.12m；等 2s 后 998 点→ATE 0.33m）。

一行命令：
```bash
cd /home/wh/ESKF/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select eskf_ros
source install/setup.bash
ros2 launch eskf_ros eskf_ros.launch.py
# 结果：data/odom_filtered_bag，再 eval_ros.py 评估
```

---

## 五、端到端看什么画面（3 个终端手动跑）

- **终端A（play）**：`Opened database sim_bag... Press SPACE...` → 以 100Hz 往外发消息
- **终端B（node）**：`已初始化 p=[5.11,-0.32,-0.05]...` → 然后 `IMU=200 观测=10 ... IMU=400 观测=20 ...`，位置沿圆周走
- **终端C（record）**：`Recording...` → 收到 998 条
- **最后 eval**：`ATE = 0.33 m`

三个进程**各自独立**工作、靠话题互通 —— 这就是真实机器人系统的样子。

---

## 一句话总结

> 端到端 = 让数据从 `bag play` 真正流经 `eskf_node` 到 `record`，再算出指标，整条真实链路完整跑通。它比单机程序更接近真实系统（进程分离 + 话题通信），是 ESKF 从"算法"走向"可用模块"的一步。
