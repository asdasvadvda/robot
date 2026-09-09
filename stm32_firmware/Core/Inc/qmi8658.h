#ifndef __QMI8658_H
#define __QMI8658_H

#include "main.h"

extern float gyro_bias_x;
extern float gyro_bias_y;
extern float gyro_bias_z;

void QMI8658_AccCalibration(void);
void QMI8658_GyroCalibration(void);

uint8_t QMI8658_ReadID(void);

uint8_t QMI8658_Init(void);


void QMI8658_ReadData(
    float *ax,
    float *ay,
    float *az,
    float *gx,
    float *gy,
    float *gz
);

// 返回连续读取失败次数（0 = 最近一次读取成功）
uint16_t QMI8658_GetReadFailCount(void);

// 打印当前 IMU 状态：WHO_AM_I + 配置寄存器回读 + 读取失败次数
// 用于主循环每 2 秒输出一次诊断，排查 IMU 是否在线/配置是否写入
void QMI8658_PrintStatus(void);


#endif