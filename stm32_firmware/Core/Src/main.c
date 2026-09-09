/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "motor.h"
#include "encoder.h"
#include "motor_control.h"
#include "chassis.h"
#include "odom.h"
#include "qmi8658.h"
#include "Madgwick.h"
#include "protocol.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Odom_t odom = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// 传感器与状态变量
uint8_t imu_ok = 0;                          // IMU 是否检测成功
float ax = 0.0f, ay = 0.0f, az = 0.0f;       // 加速度 (m/s^2)
float gx = 0.0f, gy = 0.0f, gz = 0.0f;       // 角速度 (rad/s)
float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f; // 姿态四元数
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f; // 姿态欧拉角 (度)
uint32_t sys_tick = 0;                       // 系统滴答定时器基准
uint32_t led_tick = 0;                       // LED 心跳定时基准

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  // ================ 启动横幅：直接走 HAL 发送，绝对可靠 ================
  // 注意：本板无外部晶振，SystemClock_Config 有意保持 HSI 16MHz、PLL 关闭（见下）。
  //   控制/速度环/PID 全部基于实测 dt 计时，不依赖主频，16MHz 足够 100Hz 巡检循环。
  //   若日后要升 168MHz：需在 SystemClock_Config 启用 PLL，并重算 tim.c 分频
  //   （电机 PWM 载波）、核对 UART 波特率与 FLASH 等待周期，且 VEL_FF/FRICTION_FF
  //   等 PWM 标定值需按新载波重标——不要只改时钟不动定时器！
  HAL_UART_Transmit(&huart1, (uint8_t *)"STM32 BOOT OK (HSI 16MHz)\r\n",
                    strlen("STM32 BOOT OK (HSI 16MHz)\r\n"), 100);

  // ================ LED 指示灯 ================
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);   // 点亮表示进入运行

  // ================ 电机 PWM 启动 ================
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);

  // ================ 编码器 ================
  Encoder_Init();
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  Motor_Control_Init();

  // ================ 串口协议（树莓派收发） ================
  Protocol_Init(&huart1);

  // ================ IMU 初始化（可选：检测不到也不阻塞主程序） ================
  HAL_Delay(100);   // 给 IMU 一点上电稳定时间
  imu_ok = (QMI8658_Init() == 0);   // Init 内部会打印详细诊断信息
  uart_printf("QMI8658 %s\r\n", imu_ok ? "OK, IMU enabled" : "FAILED, IMU disabled");

  // ================ 陀螺仪零偏自动校准（可选：imu_ok 时才做） ================
  // 陀螺仪零偏会导致 yaw 静止时缓慢漂移（几十秒几度），校准后明显改善。
  // 注意：校准期间必须保持小车完全静止 5 秒！否则校准结果不准。
  // 如果不想开机校准，把下面这段改成：QMI8658_GyroCalibration();
  if (imu_ok)
  {
      uart_printf("Keep robot STILL for 5s (gyro calibration)...\r\n");
      HAL_Delay(500);   // 先给用户 0.5s 反应时间，松手让小车静止
      QMI8658_GyroCalibration();
      uart_printf("Gyro calibration DONE\r\n");
      QMI8658_AccCalibration();   // 加速度零偏校准：静止时把 ax/ay 拉到 0，az 拉到 9.8
  }

  // 初始化 Madgwick 算法，增益 beta 设为 0.1f 适用于大多数机器人
  Madgwick_Init(0.1f);

  // ================ 电机链路自检（上电按住 KEY1 进入） ================
  // 用途：绕过 CMD 协议和 PID，直接给每个电机 PWM=±300（占空比~30%），
  //       同时读编码器计数差值，锁定"底盘不动"断在哪一环：
  //   - 轮子不转 / 编码器无变化  => 该路 PWM输出->驱动板->接线 有问题
  //   - 轮子转、编码器有数值      => 硬件链全通，问题在 CMD 数值太小/量纲
  // 注意：测试期间请把车悬空，轮子会依次转动！测完自动进入主循环。
  if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
  {
      Encoder_t *test_enc[4] = {GetEncoder1(), GetEncoder2(), GetEncoder3(), GetEncoder4()};
      uart_printf("\r\n===== MOTOR SELF TEST (KEY1 pressed) =====\r\n");
      uart_printf(">>> KEEP ROBOT OFF GROUND, WHEELS FREE <<<\r\n");

      for (int id = 0; id < 4; id++)
      {
          Motor_SetSpeed(id + 1, 0);
          HAL_Delay(300);

          int32_t b = __HAL_TIM_GET_COUNTER(test_enc[id]->htim);
          Motor_SetSpeed(id + 1, 300);
          uart_printf("[TEST] M%d PWM=+300 ...\r\n", id + 1);
          HAL_Delay(2000);
          uart_printf("[TEST] M%d +300: enc_delta=%d\r\n",
                      id + 1, (int)(__HAL_TIM_GET_COUNTER(test_enc[id]->htim) - b));

          Motor_SetSpeed(id + 1, 0);
          HAL_Delay(300);

          b = __HAL_TIM_GET_COUNTER(test_enc[id]->htim);
          Motor_SetSpeed(id + 1, -300);
          uart_printf("[TEST] M%d PWM=-300 ...\r\n", id + 1);
          HAL_Delay(2000);
          uart_printf("[TEST] M%d -300: enc_delta=%d\r\n",
                      id + 1, (int)(__HAL_TIM_GET_COUNTER(test_enc[id]->htim) - b));

          Motor_SetSpeed(id + 1, 0);
      }
      uart_printf("===== MOTOR SELF TEST DONE =====\r\n");
  }

  uart_printf("System Ready, entering loop...\r\n");

  sys_tick = HAL_GetTick(); // 初始化系统基准时间
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* ================= 100Hz 主循环 =================
     * 注意：这段是 8/6 验证过、后被清空丢失的主循环，8/13 重建。
     * 若用 CubeMX 重新生成，这段在 USER CODE 3 区域会被保留，勿手动删除。 */

    // 1. 计算实际经过时间 dt（秒），非阻塞调度基准
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - sys_tick) / 1000.0f;
    if (dt < 0.001f) dt = 0.01f;   // 防御：首帧/异常时取 10ms
    sys_tick = now;

    // 2. 解析上位机指令（CMD,vx,vy,wz / STOP）
    Protocol_ParseCommand();

    // 3. 底盘运动学：目标速度 -> 4 路速度环 PID -> PWM
    Chassis_Move(chassis_cmd.vx, chassis_cmd.vy, chassis_cmd.wz);
    Motor_Control_Update(&motor1_control, dt);
    Motor_Control_Update(&motor2_control, dt);
    Motor_Control_Update(&motor3_control, dt);
    Motor_Control_Update(&motor4_control, dt);
    // M2/M4 电机安装方向相反，PWM 取反（见 encoder.c 注释，勿改）
    Motor_SetSpeed(1, (int)motor1_control.pwm_output);
    Motor_SetSpeed(2, -(int)motor2_control.pwm_output);
    Motor_SetSpeed(3, (int)motor3_control.pwm_output);
    Motor_SetSpeed(4, -(int)motor4_control.pwm_output);

    // 4. 里程计：编码器速度 -> 麦轮正运动学 -> 世界系积分
    Odom_Update(&odom, dt);

    // 5. IMU 读取 + Madgwick 姿态解算
    //    注意：QMI8658 输出加速度是 m/s²，Madgwick 要求 g，除以 9.80665。
    if (imu_ok)
    {
        QMI8658_ReadData(&ax, &ay, &az, &gx, &gy, &gz);
        Madgwick_Update(gx, gy, gz,
                        ax / 9.80665f, ay / 9.80665f, az / 9.80665f,
                        dt);
        Madgwick_GetQuaternion(&qw, &qx, &qy, &qz);
    }
    // imu_ok=0 时 ax..gz 保持 0、四元数保持 (1,0,0,0)，回传照发，树莓派端至少能拿到 ODOM

    // 6. 周期回传：ODOM + IMU 两行（约 100Hz）
    Protocol_SendRobotState(ax, ay, az, gx, gy, gz, qw, qx, qy, qz, &odom);

    // 7. LED 心跳：500ms 翻转一次，肉眼确认 MCU 存活
    if (now - led_tick >= 500)
    {
        led_tick = now;
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    }
    /* USER CODE END 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* 初始化失败时 LED1 快速闪烁，方便肉眼诊断（正常运行时 LED1 是 500ms 心跳） */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  {
    GPIO_InitTypeDef g = {0};
    g.Pin = LED1_Pin;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED1_GPIO_Port, &g);
  }
  while (1)
  {
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    for (volatile uint32_t i = 0; i < 200000; i++);   // 软件延时，不依赖 SysTick
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
