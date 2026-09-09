#include "chassis.h"
#include "motor_control.h"
#include "motor.h"
#include <math.h>
#define MAX_WHEEL_SPEED 300.0f

// 轮距臂 (m)：wz 从 rad/s 换算到每个轮的线速度增量时用。
// 必须与 odom.c 里的 WHEEL_DISTANCE 保持一致，否则"命令转多少"
// 和"里程计测到转多少"会差好几倍。
#define CHASSIS_LEVER 0.30f


void Chassis_Move(float vx, float vy, float vw)
{

    float m1;
    float m2;
    float m3;
    float m4;


    // 麦克纳姆轮运动学
    // wz 是角速度(rad/s)，要乘轮距臂才是轮的线速度增量(m/s)，
    // 否则 wz=1.0 会被当成 1.0 m/s 的轮速，实际转 3.3 倍快。
    m1 = vx - vy - vw * CHASSIS_LEVER;
    m2 = vx + vy + vw * CHASSIS_LEVER;
    m3 = vx + vy - vw * CHASSIS_LEVER;
    m4 = vx - vy + vw * CHASSIS_LEVER;



    //==============================
    // 最大速度归一化
    //==============================

    float max_speed = fabs(m1);

    if(fabs(m2) > max_speed)
        max_speed = fabs(m2);

    if(fabs(m3) > max_speed)
        max_speed = fabs(m3);

    if(fabs(m4) > max_speed)
        max_speed = fabs(m4);



    if(max_speed > MAX_WHEEL_SPEED)
    {
        float scale = MAX_WHEEL_SPEED / max_speed;

        m1 *= scale;
        m2 *= scale;
        m3 *= scale;
        m4 *= scale;
    }



    //==============================
    // 转换成目标速度
    //==============================

    motor1_control.target_speed = m1;
    motor2_control.target_speed = m2;
    motor3_control.target_speed = m3;
    motor4_control.target_speed = m4;
}
void Chassis_Stop(void)
{

    motor1_control.target_speed = 0;
    motor2_control.target_speed = 0;
    motor3_control.target_speed = 0;
    motor4_control.target_speed = 0;
	
		Motor_Control_Update(&motor1_control, 0.02f);
    Motor_Control_Update(&motor2_control, 0.02f);
    Motor_Control_Update(&motor3_control, 0.02f);
    Motor_Control_Update(&motor4_control, 0.02f);


    Motor_SetSpeed(1,0);
    Motor_SetSpeed(2,0);
    Motor_SetSpeed(3,0);
    Motor_SetSpeed(4,0);


}