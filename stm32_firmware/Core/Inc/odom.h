#ifndef __ODOM_H
#define __ODOM_H

#include "motor_control.h"


typedef struct
{

    //机器人速度
    float vx;
    float vy;
    float wz;


    //机器人位姿
    float x;
    float y;
    float theta;


}Odom_t;



void Odom_Update(Odom_t *odom, float dt);


#endif