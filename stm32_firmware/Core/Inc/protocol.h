#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "usart.h" // 获取 UART_HandleTypeDef
#include "odom.h"  // 获取 Odom_t

//底盘目标速度结构体
typedef struct
{
    float vx;
    float vy;
    float wz;
} ChassisCmd_t;

// 指令处理函数指针：args 指向参数部分（不含指令名和逗号），无参数时为空字符串 ""
typedef void (*CmdHandler_t)(char *args);

// 指令表项：一个名字 + 一个处理函数
typedef struct
{
    const char   *name;     // 指令名，如 "CMD"
    CmdHandler_t handler;   // 处理函数
} CmdEntry_t;

// 声明为外部变量，这样 main.c 也能直接使用 chassis_cmd.vx
extern ChassisCmd_t chassis_cmd;

// ================= API 接口 =================

// 初始化协议模块（注册内置指令 + 开启串口接收中断）
void Protocol_Init(UART_HandleTypeDef *huart);

// 注册一条新指令（供后续功能扩展）
void Protocol_RegisterCmd(const char *name, CmdHandler_t handler);

// 放在主循环中，负责解析上位机发来的指令并分发到对应处理函数
void Protocol_ParseCommand(void);

// 单独发送一帧 ODOM
void Protocol_SendOdom(Odom_t *odom);

// 单独发送一帧 IMU（含四元数）
void Protocol_SendImu(float ax, float ay, float az,
                      float gx, float gy, float gz,
                      float qw, float qx, float qy, float qz);

// 单独发送一帧 ANGLE（欧拉角，度，调试用）
void Protocol_SendAngle(float roll, float pitch, float yaw);

// 兼容接口：一帧发送 ODOM + IMU 两行（原有调用无需修改）
void Protocol_SendRobotState(float ax, float ay, float az,
                             float gx, float gy, float gz,
                             float qw, float qx, float qy, float qz,
                             Odom_t *odom);

#endif
