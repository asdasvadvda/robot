#ifndef __MADGWICK_H
#define __MADGWICK_H

#include <math.h>

// 算法参数初始化
// beta: 算法增益，默认0.1f。如果Yaw漂移快，可以适当调大；如果噪声大，适当调小。
void Madgwick_Init(float beta);

// 核心更新函数 (6DOF)
// 参数要求：
// gx, gy, gz: 陀螺仪数据，单位必须是 弧度/秒 (rad/s)
// ax, ay, az: 加速度数据，单位必须是 g (9.81m/s^2)
// dt: 采样周期，单位秒 (例如 100Hz 对应 0.01f)
void Madgwick_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

// 获取欧拉角
// 返回值：Roll, Pitch, Yaw，单位为 度 (°)
void Madgwick_GetAngle(float *roll, float *pitch, float *yaw);

// 获取四元数
void Madgwick_GetQuaternion(float *q_w, float *q_x, float *q_y, float *q_z);
#endif