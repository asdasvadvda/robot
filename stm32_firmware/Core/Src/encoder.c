#include "encoder.h"
#include "tim.h"

#define WHEEL_DIAMETER 0.065f
#define WHEEL_CIRCUMFERENCE (3.1415926f * WHEEL_DIAMETER)

static Encoder_t encoder1;
static Encoder_t encoder2;
static Encoder_t encoder3;
static Encoder_t encoder4;

Encoder_t* GetEncoder1(void)
{
    return &encoder1;
}

Encoder_t* GetEncoder2(void)
{
    return &encoder2;
}

Encoder_t* GetEncoder3(void)
{
    return &encoder3;
}

Encoder_t* GetEncoder4(void)
{
    return &encoder4;
}
void Encoder_Init(void)
{
    encoder1.htim = &htim5;
    encoder1.last_cnt = 0;
    encoder1.ticks_per_circle = 1040.0f;
		encoder1.direction = -1;

    encoder2.htim = &htim2;
    encoder2.last_cnt = 0;
    encoder2.ticks_per_circle = 1040.0f;
		encoder2.direction = 1;    // 注意：主循环对 M2 发送的是 -pwm（main.c 里 Motor_SetSpeed(2,-pwm)），
		                           // 前进时 M2 计数器向上数，direction=+1 才能把前进报成正。
		                           // 不要只看自检原始计数，要结合主循环的取反。M4 同理。

    encoder3.htim = &htim4;
    encoder3.last_cnt = 0;
    encoder3.ticks_per_circle = 1040.0f;
		encoder3.direction = -1;

    encoder4.htim = &htim3;
    encoder4.last_cnt = 0;
    encoder4.ticks_per_circle = 1040.0f;
		encoder4.direction = 1;
}
int32_t Encoder_GetDelta(Encoder_t *encoder)
{
    int32_t now_cnt;
    int32_t delta_cnt;

    now_cnt = __HAL_TIM_GET_COUNTER(encoder->htim);

    delta_cnt = now_cnt - encoder->last_cnt;

    if(delta_cnt > 32768)
        delta_cnt -= 65536;
    else if(delta_cnt < -32768)
        delta_cnt += 65536;

    encoder->last_cnt = now_cnt;

    return delta_cnt;
}
float Encoder_GetSpeed(Encoder_t *encoder, float dt)
{
		int32_t now_cnt;
    int32_t delta_cnt;

    now_cnt = __HAL_TIM_GET_COUNTER(encoder->htim);

    delta_cnt = now_cnt - encoder->last_cnt;

    if(delta_cnt > 32768)
        delta_cnt -= 65536;
    else if(delta_cnt < -32768)
        delta_cnt += 65536;

    encoder->last_cnt = now_cnt;

    return ((float)delta_cnt /
        (float)encoder->ticks_per_circle /
        dt)
        *
        encoder->direction
        *
        WHEEL_CIRCUMFERENCE;
}