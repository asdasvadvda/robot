#include "odom.h"
#include <math.h>  // 必须引入 math 库以使用 sinf 和 cosf

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define WHEEL_DISTANCE 0.30f

extern Motor_Control_t motor1_control;
extern Motor_Control_t motor2_control;
extern Motor_Control_t motor3_control;
extern Motor_Control_t motor4_control;

void Odom_Update(Odom_t *odom, float dt)
{
    float m1 = motor1_control.current_speed;
    float m2 = motor2_control.current_speed;
    float m3 = motor3_control.current_speed;
    float m4 = motor4_control.current_speed;

    //==========================
    // 1. 麦轮正运动学 (计算局部速度)
    //==========================
    odom->vx = (m1 + m2 + m3 + m4) / 4.0f;
    odom->vy = (-m1 + m2 + m3 - m4) / 4.0f;
    odom->wz = (-m1 + m2 - m3 + m4) / (4.0f * WHEEL_DISTANCE);

    //==========================
    // 2. 积分 (转换到世界坐标系)
    //==========================
    // 计算位移增量
    float delta_x = (odom->vx * cosf(odom->theta) - odom->vy * sinf(odom->theta)) * dt;
    float delta_y = (odom->vx * sinf(odom->theta) + odom->vy * cosf(odom->theta)) * dt;
    float delta_theta = odom->wz * dt;

    // 累加到全局坐标
    odom->x += delta_x;
    odom->y += delta_y;
    odom->theta += delta_theta;

    // 限制 theta 在 -PI 到 PI 之间 (可选，但对 ROS 很友好)
    if (odom->theta > M_PI)  odom->theta -= 2.0f * M_PI;
    if (odom->theta < -M_PI) odom->theta += 2.0f * M_PI;
}