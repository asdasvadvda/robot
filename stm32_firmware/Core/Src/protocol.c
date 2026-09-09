#include "protocol.h"
#include <stdio.h>
#include <string.h>

/* ============================================================
 * 协议模块：解析上位机指令 + 打包发送机器人状态
 *
 * 【如何添加一条新指令】(以添加 "SETLED,50" 为例)
 *   1. 写一个处理函数：
 *        static void Cmd_SetLED(char *args) {
 *            uint8_t v; if (sscanf(args, "%hhu", &v) == 1) { ... }
 *        }
 *   2. 在 Protocol_Init 里注册：
 *        Protocol_RegisterCmd("SETLED", Cmd_SetLED);
 *    解析/分发逻辑完全不用改，加一条指令只需两步。
 * ============================================================ */

// 实例化对外共享的结构体
ChassisCmd_t chassis_cmd = {0.0f, 0.0f, 0.0f};

// ================= 内部私有变量 =================
static UART_HandleTypeDef *comm_huart;
static uint8_t rx_byte;          // 每次接收一个字节
static char rx_buffer[100];      // 存放整行字符串的缓存
static uint8_t rx_index = 0;     // 缓存当前索引
static uint8_t rx_complete = 0;  // 接收完成标志位

// ================= 指令注册表 =================
#define MAX_CMD_SLOTS 8
static CmdEntry_t cmd_table[MAX_CMD_SLOTS];
static uint8_t cmd_count = 0;

/* ---------- 内置指令处理函数 ---------- */

// CMD,vx,vy,wz : 底盘目标速度（原有指令，兼容）
static void Cmd_HandleMove(char *args)
{
    float vx, vy, wz;
    if (sscanf(args, "%f,%f,%f", &vx, &vy, &wz) == 3)
    {
        chassis_cmd.vx = vx;
        chassis_cmd.vy = vy;
        chassis_cmd.wz = wz;
    }
}

// STOP : 底盘急停（示例：展示如何加一条无参数指令）
static void Cmd_HandleStop(char *args)
{
    (void)args;
    chassis_cmd.vx = 0.0f;
    chassis_cmd.vy = 0.0f;
    chassis_cmd.wz = 0.0f;
}

// ================= 1. 初始化模块 =================
void Protocol_Init(UART_HandleTypeDef *huart)
{
    comm_huart = huart;
    rx_index = 0;
    rx_complete = 0;
    cmd_count = 0;

    // 注册内置指令（以后加新指令，在这里加一行即可）
    Protocol_RegisterCmd("CMD",  Cmd_HandleMove);
    Protocol_RegisterCmd("STOP", Cmd_HandleStop);

    // 开启第一次串口接收中断
    HAL_UART_Receive_IT(comm_huart, &rx_byte, 1);
}

// ================= 2. 指令注册 =================
void Protocol_RegisterCmd(const char *name, CmdHandler_t handler)
{
    if (cmd_count < MAX_CMD_SLOTS)
    {
        cmd_table[cmd_count].name    = name;
        cmd_table[cmd_count].handler = handler;
        cmd_count++;
    }
}

// ================= 3. 串口接收中断回调 =================
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == comm_huart->Instance)
    {
        // 遇到 \n 或 \r 认为一帧结束
        if (rx_byte != '\n' && rx_byte != '\r' && rx_index < sizeof(rx_buffer) - 1)
        {
            rx_buffer[rx_index++] = (char)rx_byte;
        }
        else
        {
            if (rx_index > 0)   // 防止 \r\n 连发时产生空包
            {
                rx_buffer[rx_index] = '\0';
                rx_complete = 1;
                rx_index = 0;
            }
        }
        // 重新开启中断接收下一个字节
        HAL_UART_Receive_IT(comm_huart, &rx_byte, 1);
    }
}

// ================= 4. 解析并分发指令 =================
void Protocol_ParseCommand(void)
{
    if (!rx_complete) return;
    rx_complete = 0;   // 清除标志位，准备接收下一帧

    // 分离指令名与参数：第一个逗号前是名字，其后是参数
    char *args = strchr(rx_buffer, ',');
    if (args) { *args = '\0'; args++; }

    // 在注册表中查找匹配的指令名，调用其处理函数
    for (uint8_t i = 0; i < cmd_count; i++)
    {
        if (strcmp(rx_buffer, cmd_table[i].name) == 0)
        {
            cmd_table[i].handler(args ? args : "");
            break;
        }
    }
}

// ================= 5. 发送助手 =================
static void Uart_SendLine(const char *line)
{
    if (comm_huart && line && line[0] != '\0')
    {
        HAL_UART_Transmit(comm_huart, (uint8_t *)line, strlen(line), 100);
    }
}

// ODOM,vx,vy,wz,x,y,theta
void Protocol_SendOdom(Odom_t *odom)
{
    char buf[80];
    snprintf(buf, sizeof(buf), "ODOM,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
             odom->vx, odom->vy, odom->wz, odom->x, odom->y, odom->theta);
    Uart_SendLine(buf);
}

// IMU,ax,ay,az,gx,gy,gz,qw,qx,qy,qz
void Protocol_SendImu(float ax, float ay, float az,
                      float gx, float gy, float gz,
                      float qw, float qx, float qy, float qz)
{
    char buf[120];
    snprintf(buf, sizeof(buf), "IMU,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
             ax, ay, az, gx, gy, gz, qw, qx, qy, qz);
    Uart_SendLine(buf);
}

// ANGLE,roll,pitch,yaw (度，调试用)
void Protocol_SendAngle(float roll, float pitch, float yaw)
{
    char buf[48];
    snprintf(buf, sizeof(buf), "ANGLE,%.1f,%.1f,%.1f\n", roll, pitch, yaw);
    Uart_SendLine(buf);
}

// ================= 6. 兼容接口（一帧发 ODOM + IMU 两行） =================
void Protocol_SendRobotState(float ax, float ay, float az,
                             float gx, float gy, float gz,
                             float qw, float qx, float qy, float qz,
                             Odom_t *odom)
{
    Protocol_SendOdom(odom);
    Protocol_SendImu(ax, ay, az, gx, gy, gz, qw, qx, qy, qz);
}
