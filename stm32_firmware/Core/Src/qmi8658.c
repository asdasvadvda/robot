#include "qmi8658.h"
#include "usart.h"
#include "i2c.h"
#include <stdlib.h>

extern I2C_HandleTypeDef hi2c2;

/**
  * @brief  I2C 总线恢复：用 GPIO 模拟发 9 个时钟脉冲 + STOP，解开被卡死的总线
  *
  * 背景：如果上次断电时 MCU/器件停在"发送到一半"的中间状态，SDA 会被钳在低电平，
  *       导致这次上电时整个 I2C 总线表现为"忙"，所有器件都 NACK（读 WHO_AM_I 失败）。
  *       这是"昨天好好的、今天突然 I2C 全 NACK"的最常见原因，器件本身没坏。
  *
  * 原理：I2C 规定，从机只要看到 >=9 个 SCL 时钟脉冲，就必须释放 SDA。
  *       我们手动发 9 个脉冲 + 一个 STOP 条件，把总线恢复到空闲态。
  *
  * 注意：必须保证两根线上有上拉（外部 4.7k 或内部上拉），开漏模式下释放信号靠上拉。
  */
static void I2C_RecoverBus(void)
{
    GPIO_InitTypeDef g = {0};

    /* 把 PB10(SCL)/PB11(SDA) 临时改成开漏输出，用 GPIO 手动控制 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    g.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    g.Mode = GPIO_MODE_OUTPUT_OD;
    g.Pull = GPIO_PULLUP;               // 用内部上拉兜底（若模块无外接上拉）
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);

    /* 先释放 SDA 和 SCL（写 1 = 释放，靠上拉拉高） */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(1);

    /* 发 9 个时钟脉冲：卡死的从机看到 9 个时钟后会释放 SDA */
    for (int i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);   /* SCL 拉低 */
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);     /* SCL 拉高 */
        HAL_Delay(1);
    }

    /* 发一个 STOP 条件：SDA 在 SCL 为高时由低变高 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);       /* SDA 拉低 */
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);         /* SCL 拉高 */
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);         /* SDA 拉高 = STOP */
    HAL_Delay(1);

    /* 重新初始化 I2C2：会重新把引脚配回 I2C 复用功能 */
    HAL_I2C_DeInit(&hi2c2);
    MX_I2C2_Init();
}

// ==================== QMI8658 常量定义 ====================
#define QMI8658_ADDR      (0x6A << 1)   // 默认 I2C 地址（ADDR 引脚接低）
#define QMI8658_ADDR_ALT  (0x6B << 1)   // 备用 I2C 地址（ADDR 引脚接高）

#define QMI8658_WHO_AM_I  0x00          // 设备 ID，正常应为 0x05
#define QMI8658_VERSION   0x01          // 芯片固件版本
#define QMI8658_CTRL1     0x02
#define QMI8658_CTRL2     0x03
#define QMI8658_CTRL3     0x04
#define QMI8658_CTRL7     0x08
#define QMI8658_AX_L      0x35          // 加速度 X 低字节（从这里连续读 12 字节 = AX..GZ）

// 实际探测到的设备 I2C 地址（探测成功后确定）
static uint8_t g_dev_addr = QMI8658_ADDR;

// ==================== 校准偏置 ====================
float gyro_bias_x = 0.0f;
float gyro_bias_y = 0.0f;
float gyro_bias_z = 0.0f;

float acc_bias_x = 0.0f;
float acc_bias_y = 0.0f;
float acc_bias_z = 0.0f;

// "数据无效"警告只打印一次，避免刷屏
static uint8_t s_invalid_warned = 0;

// 连续读取失败计数（ReadData 每次失败 +1，成功清零）
static uint16_t s_read_fail_count = 0;

/**
  * @brief  写单个寄存器
  * @param  reg: 寄存器地址
  * @param  val: 要写入的值
  * @retval HAL 状态
  */
static HAL_StatusTypeDef QMI8658_WriteReg(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&hi2c2, g_dev_addr, reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, 20);
}

/**
  * @brief  读单个寄存器
  * @param  reg: 寄存器地址
  * @retval 寄存器值（读失败返回 0）
  */
static uint8_t QMI8658_ReadReg(uint8_t reg)
{
    uint8_t val = 0;
    HAL_I2C_Mem_Read(&hi2c2, g_dev_addr, reg,
                     I2C_MEMADD_SIZE_8BIT, &val, 1, 20);
    return val;
}

/**
  * @brief  探测 QMI8658 并读取 WHO_AM_I
  * @retval 返回 0x05 表示设备在线；0 表示没找到
  * @note   依次尝试 0x6A / 0x6B 两个地址，并只接受 ID=0x05 的结果，
  *         避免把其它 I2C 设备的 ACK 误判成 QMI8658
  */
uint8_t QMI8658_ReadID(void)
{
    uint8_t id = 0;

    g_dev_addr = QMI8658_ADDR;
    if (HAL_I2C_Mem_Read(&hi2c2, g_dev_addr, QMI8658_WHO_AM_I,
                         I2C_MEMADD_SIZE_8BIT, &id, 1, 20) == HAL_OK)
    {
        if (id == 0x05) return id;
        /* 读到了但 ID 不是 0x05 —— 打印出来诊断：可能是别的 I2C 器件或假 QMI8658 */
        uart_printf("[QMI8658] WHO_AM_I @0x6A = 0x%02X (expect 0x05)\r\n", id);
    }
    else
    {
        uart_printf("[QMI8658] I2C NACK @0x6A (no device answering)\r\n");
    }

    g_dev_addr = QMI8658_ADDR_ALT;
    if (HAL_I2C_Mem_Read(&hi2c2, g_dev_addr, QMI8658_WHO_AM_I,
                         I2C_MEMADD_SIZE_8BIT, &id, 1, 20) == HAL_OK)
    {
        if (id == 0x05) return id;
        uart_printf("[QMI8658] WHO_AM_I @0x6B = 0x%02X (expect 0x05)\r\n", id);
    }
    else
    {
        uart_printf("[QMI8658] I2C NACK @0x6B (no device answering)\r\n");
    }

    return 0;
}

/**
  * @brief  初始化 QMI8658（标准流程，顺序不可颠倒）
  * @retval 0: 成功; 1: 失败（主程序据此跳过 IMU，不阻塞）
  *
  * 流程：探测地址 -> 软复位(CTRL7 bit7) -> 等待15ms -> 再验证ID
  *       -> CTRL1(地址自增) -> CTRL2(acc ±8g) -> CTRL3(gyro ±512dps)
  *       -> CTRL7(使能 acc+gyro) -> 等待数据稳定 -> 回读验证配置
  */
uint8_t QMI8658_Init(void)
{
    /* 0. 先解开可能卡死的 I2C 总线（"昨天好今天 NACK"最常见原因） */
    I2C_RecoverBus();

    // ========== 探测 + 软复位（最多重试 3 次，消除上电时序偶发失败） ==========
    // 软复位时序很敏感：如果复位还没完成就去读 WHO_AM_I，会读到 0xFF 或 NACK，
    // 导致整个 IMU 被判定为"不存在"。所以这里加大延时到 50ms，并整段重试 3 次。
    uint8_t device_ok = 0;

    for (int attempt = 0; attempt < 3 && !device_ok; attempt++)
    {
        // 1. 探测设备地址（软复位要用正确的地址）
        if (QMI8658_ReadID() == 0x05)
        {
            // 2. 软复位：CTRL7 bit7=1，随后必须等待 >=10ms（手册要求，这里用 50ms 更保险）
            QMI8658_WriteReg(QMI8658_CTRL7, 0x80);
            HAL_Delay(50);

            // 3. 复位后再确认设备仍在
            if (QMI8658_ReadID() == 0x05)
            {
                device_ok = 1;
                break;
            }
        }
        HAL_Delay(50);   // 下一次重试前让 I2C 总线稳定
    }

    if (!device_ok)
    {
        uart_printf("[QMI8658] NOT FOUND (WHO_AM_I error)\r\n");
        return 1;
    }

    // 4. CTRL1: 开启地址自动递增（I2C 多字节连续读取的前提）
    QMI8658_WriteReg(QMI8658_CTRL1, 0x60);
    HAL_Delay(2);

    // 5. CTRL2: 加速度 ±8g
    QMI8658_WriteReg(QMI8658_CTRL2, 0x23);
    HAL_Delay(2);

    // 6. CTRL3: 陀螺仪 ±512dps
    QMI8658_WriteReg(QMI8658_CTRL3, 0x53);
    HAL_Delay(2);

    // 7. CTRL7: 使能加速度 + 陀螺仪（0x03 = acc + gyro 都开）
    QMI8658_WriteReg(QMI8658_CTRL7, 0x03);

    // 8. 等第一个数据就绪（内部滤波/抽取需要时间）
    HAL_Delay(50);

    // 9. 回读验证配置是否真正写入
    //    写进去的值能读回来 = I2C 正常、寄存器正常。任一不符说明写入失败。
    //    （写入失败常见原因：I2C 上拉缺失 / 时序不对 / VDD 电压不足）
    uint8_t rb_id = QMI8658_ReadReg(QMI8658_WHO_AM_I);
    uint8_t rb_c1 = QMI8658_ReadReg(QMI8658_CTRL1);
    uint8_t rb_c2 = QMI8658_ReadReg(QMI8658_CTRL2);
    uint8_t rb_c3 = QMI8658_ReadReg(QMI8658_CTRL3);
    uint8_t rb_c7 = QMI8658_ReadReg(QMI8658_CTRL7);

    uart_printf("[QMI8658] ID=0x%02X VER=0x%02X | CTRL1=0x%02X CTRL2=0x%02X CTRL3=0x%02X CTRL7=0x%02X\r\n",
                rb_id, QMI8658_ReadReg(QMI8658_VERSION),
                rb_c1, rb_c2, rb_c3, rb_c7);

    if (rb_id != 0x05 || rb_c1 != 0x60 || rb_c2 != 0x23 ||
        rb_c3 != 0x53 || rb_c7 != 0x03)
    {
        uart_printf("[QMI8658] CONFIG VERIFY FAILED (writes did not stick)\r\n");
        return 1;
    }

    return 0;
}

/**
  * @brief  返回连续读取失败次数（0 = 最近一次读取成功）
  */
uint16_t QMI8658_GetReadFailCount(void)
{
    return s_read_fail_count;
}

/**
  * @brief  打印 IMU 状态诊断（供主循环每 2 秒输出一次）
  *         重点看 ID 是否 0x05、C2/C3/C7 是否 0x23/0x53/0x03
  */
void QMI8658_PrintStatus(void)
{
    uart_printf("DIAG: ID=0x%02X C1=0x%02X C2=0x%02X C3=0x%02X C7=0x%02X readfails=%u\r\n",
                QMI8658_ReadReg(QMI8658_WHO_AM_I),
                QMI8658_ReadReg(QMI8658_CTRL1),
                QMI8658_ReadReg(QMI8658_CTRL2),
                QMI8658_ReadReg(QMI8658_CTRL3),
                QMI8658_ReadReg(QMI8658_CTRL7),
                (unsigned)s_read_fail_count);
}

/**
  * @brief  读取加速度 + 陀螺仪，换算成工程单位
  * @note   I2C 读取失败、或数据判定无效时直接返回，保持上一次的数据
  */
void QMI8658_ReadData(float *ax, float *ay, float *az,
                      float *gx, float *gy, float *gz)
{
    uint8_t buf[12];

    if (HAL_I2C_Mem_Read(&hi2c2, g_dev_addr, QMI8658_AX_L,
                         I2C_MEMADD_SIZE_8BIT, buf, 12, 20) != HAL_OK)
    {
        s_read_fail_count++;           // 统计连续读取失败，供主循环自愈判断
        if (s_read_fail_count > 60000) s_read_fail_count = 60000;   // 防溢出
        return;
    }
    s_read_fail_count = 0;

    int16_t ax_raw = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t ay_raw = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t az_raw = (int16_t)((buf[5] << 8) | buf[4]);
    int16_t gx_raw = (int16_t)((buf[7] << 8) | buf[6]);
    int16_t gy_raw = (int16_t)((buf[9] << 8) | buf[8]);
    int16_t gz_raw = (int16_t)((buf[11] << 8) | buf[10]);

    // ============ 数据有效性检查 ============
    // 加速度三轴全部接近满量程(±32000)：传感器未正常工作
    //（常见原因：VDD 没供电 / 芯片异常）。此时不更新数据、只警告一次，
    // 避免把满量程垃圾数据喂给 Madgwick 或发给上位机。
    if (abs(ax_raw) > 32000 && abs(ay_raw) > 32000 && abs(az_raw) > 32000)
    {
        if (!s_invalid_warned)
        {
            s_invalid_warned = 1;
            uart_printf("[QMI8658] WARNING: ACC full scale! raw=%d,%d,%d (check VDD/power)\r\n",
                        ax_raw, ay_raw, az_raw);
        }
        return;
    }

    // 加速度换算：±8g -> m/s^2
    *ax = ax_raw * 8.0f / 32768.0f * 9.80665f - acc_bias_x;
    *ay = ay_raw * 8.0f / 32768.0f * 9.80665f - acc_bias_y;
    *az = az_raw * 8.0f / 32768.0f * 9.80665f - acc_bias_z;

    // 陀螺仪换算：±512dps -> rad/s
    *gx = gx_raw * 512.0f / 32768.0f * 3.1415926f / 180.0f - gyro_bias_x;
    *gy = gy_raw * 512.0f / 32768.0f * 3.1415926f / 180.0f - gyro_bias_y;
    *gz = gz_raw * 512.0f / 32768.0f * 3.1415926f / 180.0f - gyro_bias_z;
}

/**
  * @brief  陀螺仪零偏校准（采样 1000 次取平均）
  */
void QMI8658_GyroCalibration(void)
{
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    float ax, ay, az, gx, gy, gz;

    uart_printf("Gyro calibration start...\r\n");

    for (int i = 0; i < 1000; i++)
    {
        QMI8658_ReadData(&ax, &ay, &az, &gx, &gy, &gz);
        sum_x += gx;
        sum_y += gy;
        sum_z += gz;
        HAL_Delay(5);
    }

    gyro_bias_x = sum_x / 1000.0f;
    gyro_bias_y = sum_y / 1000.0f;
    gyro_bias_z = sum_z / 1000.0f;

    uart_printf("Gyro Bias: X=%.6f Y=%.6f Z=%.6f\r\n",
                gyro_bias_x, gyro_bias_y, gyro_bias_z);
}

/**
  * @brief  加速度零偏校准（Z 轴保留重力分量）
  */
void QMI8658_AccCalibration(void)
{
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    float ax, ay, az, gx, gy, gz;

    uart_printf("ACC calibration start...\r\n");

    for (int i = 0; i < 1000; i++)
    {
        QMI8658_ReadData(&ax, &ay, &az, &gx, &gy, &gz);
        sum_x += ax;
        sum_y += ay;
        sum_z += az;
        HAL_Delay(5);
    }

    acc_bias_x = sum_x / 1000.0f;
    acc_bias_y = sum_y / 1000.0f;
    acc_bias_z = sum_z / 1000.0f - 9.80665f;   // Z 轴保留重力

    uart_printf("ACC Bias X=%.4f Y=%.4f Z=%.4f\r\n",
                acc_bias_x, acc_bias_y, acc_bias_z);
}
