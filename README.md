# 巡检机器人项目（Inspection Robot）

基于 **树莓派 5 + ROS2 Humble** 的巡检小车。整机由 2D 激光雷达（MS200）、IMU（QMI8658 + Madgwick 解算）、STM32 底盘（轮式里程计）组成，软件跑在 docker 容器内，定位使用 ESKF / EKF 融合，自主导航使用 Nav2。

## 目录结构

```
my_ros2_ws/
├── docker/            # 容器定义：Dockerfile + docker-compose.yml（含硬件挂载）
├── ros2_ws/           # ROS2 工作空间（核心代码）
│   ├── src/
│   │   ├── robot_driver/    # 底盘驱动（串口）
│   │   ├── oradar_lidar/    # MS200 激光雷达驱动（厂商 SDK）
│   │   └── eskf_ros/        # ESKF 定位节点
│   └── ESKF/                # ESKF 学习工程（stage0~6 + 算法核心 include/eskf.hpp）
├── scripts/           # 实车测试/建图/巡检脚本（test_drive_*.py、map_drive_roundtrip.py、waypoint_patrol.py）
├── config/            # 全局配置（slam_toolbox 建图、nav2_params.yaml 导航）
├── maps/              # 地图（*.pgm + *.yaml）
├── docs/              # 项目文档（PI_HANDOFF 交接说明、LEARNING_NOTES 学习札记）
└── firmware/          # STM32 固件工程（占位）
```

## 功能模块

| 模块 | 位置 | 作用 |
|---|---|---|
| **robot_driver** | `ros2_ws/src/robot_driver` | STM32 底盘串口驱动：发布 `/odom` + `/imu/data`，订阅 `/cmd_vel` |
| **oradar_lidar** | `ros2_ws/src/oradar_lidar` | MS200 雷达驱动：发布 `/MS200/scan`（10Hz，360°） |
| **eskf_ros** | `ros2_ws/src/eskf_ros` | ESKF 定位节点：订阅 `/imu` + `/odom` → 发布 `/odom_filtered`（IMU 频率） |
| **ESKF** | `ros2_ws/ESKF` | ESKF 学习工程：stage0~6 课程程序 + 共享算法核心 `include/eskf.hpp`（15 维误差状态） |
| **robot_localization** | 系统依赖 | EKF 融合（`ekf.yaml`）：odom0(速度) + imu0(yaw) → `/odometry/filtered` |
| **slam_toolbox** | 系统依赖 | 激光 SLAM 建图（`mapper_params_online_sync.yaml`） |
| **Nav2** | 系统依赖 | 自主导航（AMCL 定位 + NavFn 规划 + DWB 控制，`nav2_params.yaml`） |

## 硬件串口映射

| 设备 | 容器内路径 | 波特率 | 用途 |
|---|---|---|---|
| STM32 底盘 | `/dev/ttyACM0` | 115200 | 轮式里程计 / 电机控制，协议 `CMD,vx,vy,wz` / `ODOM,...` / `IMU,...` |
| MS200 激光雷达 | `/dev/ttyUSB0` | 230400 | 2D 激光扫描 |

## 快速开始

```bash
# 1. 启动容器（宿主机）
cd /home/pi/my_ros2_ws
docker compose -f docker/docker-compose.yml up -d
docker exec -it inspection_robot_ros2 bash

# 2. 构建（容器内）
cd /home/ubuntu/my_ros2_ws/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

# 3. 一键启动底盘传感栈：底盘驱动 + EKF 融合 + 雷达（默认带雷达，可 include_lidar:=false 关）
ros2 launch robot_driver ekf_localization.launch.py
```

## 实车测试

```bash
# 前进/后退 1m 往返，通过 /odom 校验位移（0.25 m/s × 4s ≈ 1m）
cd /home/ubuntu/my_ros2_ws/scripts
python3 test_drive_1m.py
# 对比 EKF 输出 /odometry/filtered 版本：
python3 test_drive_1m_ekf.py
# 极简阶梯测试（vx=0.2 阶跃 5s）：
python3 test_drive_step.py
```

## SLAM 建图

```bash
# 一键建图：底盘传感栈 + slam_toolbox（等价于起 ekf_localization + 手动跑 slam_toolbox）
ros2 launch robot_driver mapping.launch.py

# 等 /map 话题有数据、map->odom TF 出现后，跑建图驱动脚本出图（前进 dist 米 → 保存地图 → 后退回起点）
python3 /home/ubuntu/my_ros2_ws/scripts/map_drive_roundtrip.py --dist 3.0
# 记下保存的地图路径（默认 /home/ubuntu/my_ros2_ws/maps/my_map_<时间戳>.yaml），后续定位用
```

## Nav2 自主导航（定位 + 避障 + 航点巡检）

```bash
# 一键定位 + 导航：底盘传感栈 + AMCL(地图定位) + Nav2
# map 默认最近的一张图，建议传建图刚出的图
ros2 launch robot_driver nav_bringup.launch.py \
  map:=/home/ubuntu/my_ros2_ws/maps/my_map_<时间戳>.yaml
```

启动后两种玩法：

```bash
# 玩法 A（RViz 手点）：rviz2 里 2D Pose Estimate 设初始位姿 → 2D Nav Goal 发目标
#   容器内 DISPLAY 需可用，如 DISPLAY=:0 ros2 run rviz2 rviz2

# 玩法 B（巡检脚本，headless 可用）：依次到达地图上的航点
python3 /home/ubuntu/my_ros2_ws/scripts/waypoint_patrol.py --initial-pose 0 0 0
# 航点坐标在地图(map)系，先改脚本顶部 WAYPOINTS 列表为你的真实巡检点
# 参数：--loops 圈数  --waypoint-timeout 单点超时  --list-waypoints 只打印航点
```

限速在 `config/nav2_params.yaml`（`controller_server` / `velocity_smoother` 的
`max_vel_*`），首次实跑建议先调到 0.3 / 0.5 再逐步放开。

## ESKF 定位（可选替代 robot_localization 的 EKF）

```bash
# 实车：把 eskf_ros 接到小车真实话题（/imu → /imu/data）
ros2 launch eskf_ros eskf_ros_car.launch.py

# 仿真：端到端 node + record + play（数据在 ESKF/data/sim_bag）
ros2 launch eskf_ros eskf_ros.launch.py
# 评估：python3 /home/ubuntu/my_ros2_ws/ros2_ws/ESKF/scripts/eval_ros.py --bag <out_bag> --source sim
```

## 相关文档

- `docs/PI_HANDOFF.md` — 树莓派端交接说明（ESKF 节点上车的完整背景与踩坑）
- `docs/LEARNING_NOTES.md` — ESKF 学习札记（经典疑问与踩坑手册）
- `ros2_ws/ESKF/STAGE6B_ROS2.md` — stage6b：ESKF 端到端小结
