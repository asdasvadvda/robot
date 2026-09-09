#ifndef __PID_H
#define __PID_H

typedef struct
{

    float kp;
    float ki;
    float kd;


    float target;        //目标速度

    float feedback;      //实际速度


    float error;         //当前误差
    float last_error;    //上一次误差

    float integral;      //积分


    float output;        //PID输出


}PID_t;


void PID_Init(PID_t *pid,
              float kp,
              float ki,
              float kd);


float PID_Calc(PID_t *pid,
               float target,
               float feedback,
							 float dt);


#endif