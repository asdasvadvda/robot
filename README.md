# 巡检机器人项目 (Inspection Robot)

基于 **树莓派 5 + ROS2 Humble(容器)** 的自主巡检小车:MS200 2D 激光雷达 + QMI8658 IMU + STM32 差分底盘。
定位用 EKF(robot_localization)/ ESKF 融合里程计,自主导航用 Nav2(AMCL 定位 + NavFn 规划 + DWB 控制),支持"记录航点 → 依次巡检 → 自动回原点"的完整闭环。

**2026-08-25 状态:完整巡检流程实测通过**(起点 → 航点1 22.6s → 停3s → 航点2 7.2s → 停3s → 回原点 45s,2/2 成功,停点误差 0.12m / 3.3°)。

## 系统架构

```
┌─────────────────────────── 树莓派 5 (容器: robot-dev:humble) ───────────────────────────┐
│                                                                                         │
│  /dev/ttyUSB0 ── MS200 雷达 ── oradar_lidar ──> /MS200/scan                            │
│  /dev/ttyACM0 ── STM32 底盘 ── robot_driver ──> /odom, /imu/data     订阅 /cmd_vel      │
│                          │                                                              │
│                          ▼                                                              │
│              robot_localization EKF (ekf.yaml)                                          │
│              轮式 vx/vy/wz + IMU yaw/wz(差分) ──> /odometry/filtered + odom->base_link  │
│                          │                                                              │
│                          ▼                                                              │
│  Nav2: AMCL(map) → NavFn(全局) → DWB(局部) → velocity_smoother → /cmd_vel              │
│  地图: /home/ubuntu/my_ros2_ws/maps/my_map.yaml                                        │
└─────────────────────────────────────────────────────────────────────────────────────────┘
```

## 目录结构

```
my_ros2_ws/
├── docker/               # 容器定义: Dockerfile + docker-compose.yml(整仓挂载 + USB 直通)
├── ros2_ws/              # ROS2 工作空间
│   ├── src/
│   │   ├── robot_driver/     # 底盘串口驱动 + EKF 融合 + 建图/导航 launch
│   │   ├── oradar_lidar/     # MS200 激光雷达驱动(厂商 SDK)
│   │   └── eskf_ros/         # ESKF 定位节点
│   └── ESKF/                 # ESKF 学习工程(stage0~6 + 算法核心 include/eskf.hpp)
├── scripts/              # 巡检工作流脚本(见下表)
│   └── tools/            # 一次性测试/调试工具
├── config/               # 全局配置(mapper_params_online_sync.yaml 建图、nav2_params.yaml 导航)
├── maps/                 # 地图(my_map.yaml 为正式图,backup_20260824/ 为历史备份)
└── docs/                 # 项目文档(PI_HANDOFF 交接、LEARNING_NOTES 学习札记)
```

## 功能模块

| 模块 | 位置 | 作用 |
|---|---|---|
| **robot_driver** | `ros2_ws/src/robot_driver` | STM32 串口驱动:发布 `/odom` + `/imu/data`,订阅 `/cmd_vel` |
| **oradar_lidar** | `ros2_ws/src/oradar_lidar` | MS200 雷达:发布 `/MS200/scan`(10Hz,360°) |
| **robot_localization** | 系统依赖 | EKF 融合(`ekf.yaml`):odom0(速度差分)+ imu0(yaw/wz 差分)→ `/odometry/filtered`,广播 odom→base_link |
| **eskf_ros / ESKF** | `ros2_ws/src/eskf_ros`、`ros2_ws/ESKF` | ESKF 定位节点(可选替代 EKF)+ 学习工程(stage0~6) |
| **slam_toolbox** | 系统依赖 | 激光 SLAM 建图(`config/mapper_params_online_sync.yaml`) |
| **Nav2** | 系统依赖 | 自主导航(AMCL + NavFn + DWB,`config/nav2_params.yaml`) |

## 巡检脚本(scripts/)

| 脚本 | 用途 |
|---|---|
| `waypoint_patrol.py` | **巡检主程序**:依次到达航点、每点停 N 秒、走完回原点(--dwell/--loops/--home/--waypoint-timeout/--list-waypoints) |
| `record_waypoint.py` | 记录航点:读当前 /amcl_pose 追加进 `waypoints.yaml`(--list 查看 / --clear 清空) |
| `waypoints.yaml` | 航点数据(地图系 x/y/yaw_deg) |
| `start_teleop.sh` | 一键键盘遥控:自动起 straighten_teleop 直行纠偏(见下) |
| `straighten_teleop.py` | 直行自动纠偏(odom 系反馈,与键盘联动) |
| `global_localize.py` | 全局定位:发大协方差 /initialpose 撒满粒子(AMCL 1.1.13 无全局定位服务,见故障排查) |
| `check_localization.py` | 定位收敛检查:粒子云加权标准差 <0.2m ≈ 已锁定 |
| `map_click_pose.py` | 在图上点选坐标,换算地图系位姿喂给 RViz/巡检脚本(需图形界面) |
| `map_drive_roundtrip.py` | 建图驱动:直行 dist 米 → 保存地图 → 退回起点 |
| `compare_eskf_ekf.sh` | **ESKF vs EKF 融合对比**:record 实车录数据 → offline 重放同一份数据喂两个滤波器,严格同条件对比(见下) |
| `compare_odom.py` | 对比节点:按时间戳对齐统计 EKF/ESKF 输出差异,出 CSV + 轨迹/误差 PNG(离线对比自动调用,也可单独跑) |
| `tools/` | 调试工具:test_drive_*(直线/漂移测试)、test_rot_deadzone.py(旋转死区实测)、patrol_pose_monitor.py(巡检位姿记录)、go_home.py(单点回原点)、traj_recorder.py 等 |

## ESKF vs EKF 融合对比(路线 A: 并行/离线)

自己写的 ESKF(`eskf_ros` 包, 输出 `/odom_filtered`)与生产链路的 robot_localization EKF(`/odometry/filtered`)吃同样输入、同在 odom 系、同起点,可直接对比;**必须按消息时间戳对齐**(两个滤波器启动时刻不同,不能按墙钟比)。

```bash
# 1. 实车巡检时录数据(起着 nav_bringup, 车跑完整流程回原点后 Ctrl+C)
bash /home/ubuntu/my_ros2_ws/scripts/compare_eskf_ekf.sh record run1

# 2. 离线重放: 同一份数据现算 ESKF, 与录好的 EKF 输出对比(不需要车)
bash /home/ubuntu/my_ros2_ws/scripts/compare_eskf_ekf.sh offline run1 --plot compare.png
#    bag 存 /home/ubuntu/my_ros2_ws/bags/run1; 对比图默认生成在同目录
```

硬指标:巡检回原点后,两个滤波器**距原点漂移**都应 ≈ 0(闭环误差);轨迹贴合看对比图的误差曲线。

## 硬件串口映射

| 设备 | 路径 | 波特率 | 用途 |
|---|---|---|---|
| STM32 底盘 | `/dev/ttyACM0` | 115200 | 轮式里程计/电机控制,协议 `CMD,vx,vy,wz` / `ODOM,...` / `IMU,...` |
| MS200 激光雷达 | `/dev/ttyUSB0` | 230400 | 2D 激光扫描 |

## 快速开始

```bash
# 1. 宿主机: 启动容器(容器内没有 docker CLI, 容器管理都在宿主机做)
cd /home/ubuntu/my_ros2_ws
docker compose -f docker/docker-compose.yml up -d
docker exec -it inspection_robot_ros2 bash

# 2. 容器内: 构建 + 起底盘传感栈(驱动 + EKF 融合 + 雷达)
cd /home/ubuntu/my_ros2_ws/ros2_ws
source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash
ros2 launch robot_driver ekf_localization.launch.py   # include_lidar:=false 可关雷达
```

## 巡检工作流(2026-08-25 实测通过)

```bash
# 1. 起导航(定位 + 避障 + 规划), map 默认 maps/my_map.yaml
source /opt/ros/humble/setup.bash && source /home/ubuntu/my_ros2_ws/ros2_ws/install/setup.bash
ros2 launch robot_driver nav_bringup.launch.py

# 2. RViz 设初始位姿: 2D Pose Estimate 在图上点车的位置和朝向
#    ⚠️ 不要用 --initial-pose: 它发的是零协方差 initialpose, 本版 AMCL(1.1.13)直接忽略
rviz2   # 用 2D Pose Estimate 对齐激光点云与墙, 然后等 AMCL 收敛

# 3. 记录航点(可选, 已记录过可跳过): 车开到巡检点, 跑一次记一个点
python3 /home/ubuntu/my_ros2_ws/scripts/record_waypoint.py
python3 /home/ubuntu/my_ros2_ws/scripts/record_waypoint.py --list   # 查看

# 4. 键盘遥控(需要把车摆回起点时用): 必须在自己的终端里跑, 交互式
bash /home/ubuntu/my_ros2_ws/scripts/start_teleop.sh   # w/s 直行(自动纠偏), a/d 转向
#    ⚠️⚠️ 开完车必须 Ctrl+C 完全退出键盘再跑巡检!
#        straighten_teleop 会和 Nav2 抢 /cmd_vel, 导致轮子冻结(2026-08-25 实测)

# 5. 跑完整巡检: 起点 → 航点1 → 停3s → 航点2 → 停3s → 自动回原点
python3 /home/ubuntu/my_ros2_ws/scripts/waypoint_patrol.py --dwell 3
```

## SLAM 建图(重做地图时用)

```bash
# 一键建图: 底盘传感栈 + slam_toolbox
ros2 launch robot_driver mapping.launch.py

# 沿墙走完一个闭环再保存(回环闭合后图才干净, 2026-08-24 实测经验)
# 保存地图(服务请求字段是 std_msgs/String, name 不含扩展名):
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/home/ubuntu/my_ros2_ws/maps/my_map_new'}}"

# 或用车自动驱动建图(直行 dist 米 → 保存 → 退回)
python3 /home/ubuntu/my_ros2_ws/scripts/map_drive_roundtrip.py --dist 4.0
```

## 实车调试工具

```bash
# 都在 scripts/tools/ 下, 一次一个:
python3 scripts/tools/test_drive_1m.py          # 前进/后退 1m 往返, /odom 校验位移
python3 scripts/tools/test_drive_1m_ekf.py      # 同上, 对比 /odometry/filtered
python3 scripts/tools/test_rot_deadzone.py      # 实测 STM32 旋转死区(2026-08-25: 0.15 不动, 0.30 转)
python3 scripts/tools/patrol_pose_monitor.py    # 记录 /amcl_pose 到 /tmp/patrol_pose_log.csv 供复盘
python3 scripts/tools/go_home.py                # 单点导航回原点(0,0,0), 150s 超时
```

## ESKF 定位(可选,替代 robot_localization 的 EKF)

```bash
# 实车: 把 eskf_ros 接到真实话题(/imu → /imu/data)
ros2 launch eskf_ros eskf_ros_car.launch.py
# 仿真评估: ros2 launch eskf_ros eskf_ros.launch.py
# 算法学习: 见 ros2_ws/ESKF/ 的 stage0~6 + LEARNING_NOTES.md
```

## 故障排查(2026-08-25 实测踩坑汇总)

| 现象 | 根因与修复 |
|---|---|
| 车到 0.15m 处停死,判不到达,recovery 反复 | **"0.15 双重锁死"**:① AMCL 更新门控 `update_min_d=0.25` 导致目标附近微调时定位冻结 → 调 0.05;② DWB 内部 `FollowPath.xy_goal_tolerance=0.15` 让车在 0.15m 停平移只旋转,永远进不了 goal_checker 的 0.15 判定圈 → 调 0.05 |
| 到点后朝向永远对不齐 | **旋转死区**:STM32 wz=0.15 rad/s 不动(+0.06°),0.30 才转。`FollowPath.min_speed_theta` 0.15→0.3 |
| 目标点附近微调时 AMCL 不发布位姿 | AMCL 运动门控设计:位移 < update_min_d 不更新。update_min_d: 0.25→0.05、update_min_a: 0.2→0.1 |
| 键盘开着时 Nav2 下轮子冻结 | straighten_teleop 与 Nav2 抢 /cmd_vel → 巡检前必须完全退出键盘(`ros2 topic info /cmd_vel -v` 确认发布者只剩 behavior_server/velocity_smoother/robot_driver_node) |
| 超时取消后下一个目标 0.0s 秒拒 | 取消后 Nav2 recovery 行为(Spin 等)残留,waypoint_patrol.py 已内置 3s 静置 |
| Nav2 取消操作挂起 | 调 **`/navigate_to_pose/_action/cancel_goal`**(ActionClient 底层服务名,不是 /navigate_to_pose/cancel_goal) |
| --initial-pose 无效 | AMCL 1.1.13 忽略零协方差 initialpose → 用 RViz 2D Pose Estimate(或 global_localize.py 大协方差) |
| AMCL 粒子云收不到 | 它是 best_effort QoS + nav2_msgs/ParticleCloud,订阅必须用 qos_profile_sensor_data;车静止时不发布,边动边看 |
| 车静止时记录航点等不到 pose | AMCL 静止不发布 /amcl_pose,record_waypoint.py 已加 tf 兜底采样 |
| STM32 固件执行残留指令(旋转不停) | 进程被杀前先发零速:`printf 'CMD,0.000,0.000,0.000\n' > /dev/ttyACM0` |

## 相关文档

- `docs/PI_HANDOFF.md` — 树莓派端交接说明(ESKF 上车背景与踩坑)
- `docs/LEARNING_NOTES.md` — ESKF 学习札记
- `ros2_ws/ESKF/STAGE6B_ROS2.md` — stage6b:ESKF 端到端小结
