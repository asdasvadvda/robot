#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H


#include "main.h"
#include "pid.h"
#include "encoder.h"


typedef struct
{

    PID_t pid;                 // PID控制器

    Encoder_t *encoder;        // 对应编码器


    float target_speed;        //目标速度 rps

    float current_speed;       //实际速度 rps


    float pwm_output;          //输出PWM


}Motor_Control_t;



void Motor_Control_Init(void);


void Motor_Control_Update(Motor_Control_t *motor, float dt);



extern Motor_Control_t motor1_control;
extern Motor_Control_t motor2_control;
extern Motor_Control_t motor3_control;
extern Motor_Control_t motor4_control;



#endif