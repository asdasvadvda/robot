#include "pid.h"


void PID_Init(PID_t *pid,
              float kp,
              float ki,
              float kd)
{

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;


    pid->target = 0;

    pid->feedback = 0;


    pid->error = 0;

    pid->last_error = 0;

    pid->integral = 0;

    pid->output = 0;

}



float PID_Calc(PID_t *pid,
               float target,
               float feedback,
               float dt)
{

    pid->target = target;
    pid->feedback = feedback;


    pid->error = target - feedback;


    //积分
    pid->integral += pid->error * dt;


    if(pid->integral > 1000)
        pid->integral = 1000;

    else if(pid->integral < -1000)
        pid->integral = -1000;



    float derivative =
        (pid->error - pid->last_error) / dt;



    pid->output =
        pid->kp * pid->error
        +
        pid->ki * pid->integral
        +
        pid->kd * derivative;



    if(pid->output > 1000)
        pid->output = 1000;

    else if(pid->output < -1000)
        pid->output = -1000;



    pid->last_error = pid->error;


    return pid->output;

}