#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

typedef struct
{
    TIM_HandleTypeDef *htim;   //定时器

		int32_t last_cnt;          //上一次计数值

    float ticks_per_circle;    //一圈对应多少计数
	
		int8_t direction;          //定义方向

}Encoder_t;

void Encoder_Init(void);
float Encoder_GetSpeed(Encoder_t *encoder, float dt);
int32_t Encoder_GetDelta(Encoder_t *encoder);
Encoder_t* GetEncoder1(void);
Encoder_t* GetEncoder2(void);
Encoder_t* GetEncoder3(void);
Encoder_t* GetEncoder4(void);

#endif