# STM32 巡检机器人底盘固件（STM32F407VET6 裸机）

自主巡检机器人**全链路的最底层**：接收树莓派（ROS2）下发的三轴目标速度 `CMD,vx,vy,wz`，完成
**麦克纳姆轮运动学 → 4 路电机速度环（前馈 + PI）→ PWM 输出**，并以 100Hz 回传
轮式里程计（`ODOM`）与 IMU 姿态（`IMU`）。

> 上位机（树莓派 5 + ROS2 / Docker / Nav2）部分见同仓库另一工程 `my_ros2_ws`。
> 本文档面向：看懂固件结构 → 上手改 / 加指令 → 上电自检 → 重新标定。

---

## 1. 硬件连接

| 外设 | 接口 | 说明 |
| ---- | ---- | ---- |
| 上位机串口 | USART1 / PA9(TX) PA10(RX)，115200 8N1 | 文本行协议收发 |
| 编码器 M1~M4 | TIM5 / TIM2 / TIM4 / TIM3（编码器模式） | 4 路 AB 相正交编码器 |
| 电机 PWM | TIM1(CH1~CH4)、TIM9(CH1/2)、TIM10/11(CH1) | 每路电机 = 2 路互补方向 PWM |
| IMU | I2C2 / PB10(SCL) PB11(SDA)，地址 0x6A/0x6B | QMI8658 六轴 |

电机 ↔ 定时器 / 编码器对应：M1→TIM1(3/4)+TIM5、M2→TIM1(1/2)+TIM2、M3→TIM9+TIM4、M4→TIM10/11+TIM3。

## 2. 软件模块

```
Core/
├── Src/main.c            100Hz 主循环调度（非阻塞，dt 按实测计算）
├── Src/protocol.c        串口协议层：指令注册表分发 + 状态打包（收 CMD/STOP，发 ODOM/IMU）
├── Src/chassis.c         麦轮逆运动学：vx/vy/wz → 4 轮目标速度（含最大速度归一化）
├── Src/motor_control.c   速度环：前馈(摩擦+比例) + PI + 启动kick + 主动刹车
├── Src/motor.c           双路互补 PWM 输出（方向切换）
├── Src/encoder.c         编码器测速（16/32 位计数器溢出处理，含方向修正）
├── Src/odom.c            麦轮正运动学 → 世界系积分里程计
├── Src/qmi8658.c         QMI8658 驱动：探测/软复位/配置回读校验/零偏校准/总线恢复
└── Src/Madgwick.c        姿态解算（beta=0.1）
```

### 主循环（main.c，约 100Hz）
每拍顺序执行：解析上位机指令 → `Chassis_Move` 逆运动学 → 4 路速度环 → PWM → `Odom_Update` 积分 → IMU 读取 + Madgwick 解算 → 串口回传 ODOM+IMU 两行。

循环频率不是定时器硬节拍，而是被阻塞式串口发送（115200）自然节流在 ~100Hz；
`dt` 每拍按 `HAL_GetTick()` 实测，因此 **PID 积分与里程计不依赖主频**，改主频/循环负载都不会漂。

### 时钟（重要设计决定）
板子**没有外部晶振**，`SystemClock_Config` **有意保持 HSI 16MHz、PLL 关闭**：
- 控制环 / PID / 里程计全部基于实测 `dt`，不依赖 CPU 主频，16MHz 足够 100Hz 巡检循环；
- 若日后升 168MHz：需启用 PLL **并** 重算 `tim.c` 分频（电机 PWM 载波）、核对 UART 波特率与
  FLASH 等待周期，且速度环 `VEL_FF / FRICTION_FF / BRAKE_GAIN` 等标定值需按新载波**重新标定**——
  不要只改时钟不动定时器。

## 3. 串口协议（文本行，`\n` 结尾）

**上位机 → STM32（接收）**

| 指令 | 含义 |
| ---- | ---- |
| `CMD,vx,vy,wz` | 三轴目标速度（m/s, m/s, rad/s） |
| `STOP` | 急停（三轴清零） |

**STM32 → 上位机（回传，约 100Hz）**

| 报文 | 内容 |
| ---- | ---- |
| `ODOM,vx,vy,wz,x,y,theta` | 速度 + 里程计（世界系，theta ∈ [-π,π]） |
| `IMU,ax,ay,az,gx,gy,gz,qw,qx,qy,qz` | 加速度(m/s²) + 角速度(rad/s) + Madgwick 四元数 |

**加一条新指令只需两步**（协议层用指令注册表分发，解析逻辑不用动）：
1. 写处理函数 `static void Cmd_Xxx(char *args)`；
2. 在 `Protocol_Init` 里注册 `Protocol_RegisterCmd("XXX", Cmd_Xxx);`

## 4. 速度环设计（实测标定，勿随意改数值）

电机模型按线性拟合 `pwm = FRICTION_FF + VEL_FF * v`（斜率 + 截距），PID 只负责修偏差：

| 参数 | 值 | 含义 / 历史 |
| ---- | ---- | ---- |
| `VEL_FF` | 250 | 每 m/s 的前馈 PWM（悬空/带负载基本一致） |
| `FRICTION_FF` | 16 | 克服摩擦的固定 PWM 截距。实测反推 ≈16(前进)/≈8(后退)；曾设 100 导致 1.7× 超跑 |
| `START_KICK` | 300 | 起步强推突破静摩擦（原 500 瞬态过冲过大） |
| `BRAKE_GAIN` | 400 | 停车反向制动，消除 ~0.4m 滑行 |
| PID | Kp=200, Ki=250, Kd=0 | Ki=250 → 积分时间常数 τ≈1s；Ki=20(τ≈12.5s) 时 4s 实测内环退化为开环 |

> 以上数值随**负载/地面**变化，换车换负载需按 `motor_control.c` 头注释的步骤地面重标。

## 5. 上电自检 & 调试

- **电机链路自检**：上电**按住 KEY1** 进入——逐电机输出 PWM=±300 并读编码器增量，
  快速锁定"底盘不动"断在 PWM 输出 / 驱动板 / 接线还是指令量纲。
- **IMU 校准**：正常上电后需保持小车静止 5s（陀螺仪零偏）+ 1s（加速度零偏）。
- **固件侧里程计**：悬空发 `vx=0.2`，`ODOM` 应稳定 ≈0.18m/s 无振荡（速度环闭环调通）。
- **系统级联调**：与树莓派端 `test_drive_1m.py`（往返 + 卷尺交叉验证）配合验收。

## 6. 构建

- **图形界面**：Keil MDK5 打开 `MDK-ARM/STM32_Template.uvprojx` 编译下载。
- **命令行**：`UV4 -j0 -b "MDK-ARM/STM32_Template.uvprojx" -o build.log`（已配置命令行编译）。
- 重新生成外设代码请用 STM32CubeMX 打开根目录 `.ioc`（`USER CODE` 区会被保留）。

## 7. 已知限制 / 待改进

- **电机 PWM 载波偏低**：TIM1 `PSC=839, ARR=999`，在 16MHz 下约 19Hz。低速能用但对绕组偏粗。
  最省事的改进是**保持 ARR≈999 不动（量纲 0~1000 不变、增益无需重标），把 PSC 降到最小**
  即可把载波提到 kHz 级；改动后须重跑一次 1m 往返验证。
- M2/M4 电机安装方向相反，采用"`encoder.c` direction + `main.c` PWM 取反"双保险修正，
  两处需同步维护（见代码注释）。

## License / 说明

个人学习 / 求职项目，代码为作者本人开发，注释保留实测过程供复现。
