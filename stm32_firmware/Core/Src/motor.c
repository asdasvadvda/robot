#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>
#include "motor.h"
#include "chassis.h"

void Motor_SetSpeed(uint8_t motor_id, int speed) {
    if (motor_id == 1) {       // M1
        if(speed > 0) { __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0); __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed); }
        else if(speed < 0) { __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0); __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, -speed); }
        else { __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0); __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0); }

        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    } 
    else if (motor_id == 2) {  // M2
        if(speed > 0) { __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed); }
        else if(speed < 0) { __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0); __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -speed); }
        else { __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0); }

        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    }
    else if (motor_id == 3) {  // M3

        
        if(speed > 0) { __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0); __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, speed); }
        else if(speed < 0) { __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0); __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, -speed); }
        else { __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0); __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0); }
        
        HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
    }
    else if (motor_id == 4) {  // M4
 
        if(speed > 0) { __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0); __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, speed); }
        else if(speed < 0) { __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0); __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, -speed); }
        else { __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0); __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0); }
        
        HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
    }
}