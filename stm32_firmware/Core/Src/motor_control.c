#include "motor_control.h"
#include "math.h"
#include <stdio.h>


Motor_Control_t motor1_control;
Motor_Control_t motor2_control;
Motor_Control_t motor3_control;
Motor_Control_t motor4_control;



void Motor_Control_Init(void)
{

		// PID 增益：Kp=200 修正前馈偏差；Ki=250 让积分 ~1s 内收敛（τ = VEL_FF/Ki）。
		// 历史教训：Ki=20 时 τ≈12.5s，4s 的 1m 实测里积分几乎没动，环退化成"前馈+P"，
		// 前馈一标偏就兜不住（命令 0.25 实际 0.437）。Ki 太小=开着速度环但等于没环。
		// 若上车发现速度振荡，先降 Ki 到 150 再逐步加回；稳定就保持 250。
		PID_Init(&motor1_control.pid, 200.0f, 250.0f, 0.0f);
		PID_Init(&motor2_control.pid, 200.0f, 250.0f, 0.0f);
		PID_Init(&motor3_control.pid, 200.0f, 250.0f, 0.0f);
		PID_Init(&motor4_control.pid, 200.0f, 250.0f, 0.0f);


    motor1_control.encoder = GetEncoder1();
    motor2_control.encoder = GetEncoder2();
    motor3_control.encoder = GetEncoder3();
    motor4_control.encoder = GetEncoder4();



    motor1_control.target_speed = 0;
    motor2_control.target_speed = 0;
    motor3_control.target_speed = 0;
    motor4_control.target_speed = 0;


}
// ============ 速度环参数（电机模型实测标定） ============
// 电机模型：pwm = FRICTION_FF + VEL_FF*v（线性拟合：斜率 + 截距）
//   VEL_FF = 250：每 m/s 的 PWM 斜率，悬空/带负载基本一致，可靠。
//   FRICTION_FF = 截距（克服摩擦的固定 PWM），随负载/地面变化，必须实测！
//     历史：悬空≈60；带负载曾拟合出≈100 -> 定成 100。
//     【2026-08 两轮实测】
//      第一轮(ki=20, FRICTION_FF=100)：命令 0.25 实际 0.437 m/s（1.7× 超跑）。
//        反推真实截距 ≈16 PWM：FRICTION_FF=100 多灌 ~84 PWM 开环推力，
//        而 Ki=20 太弱(τ=12.5s)4s 内纠不回来 -> 物理超跑。
//      第二轮(ki=250, FRICTION_FF 仍 100)：4s 位移收到 1.303m(1.30×)，
//        残差 = ff 偏移瞬态 + 起步过冲，证实不是比例增益、是固定量。
//     -> FRICTION_FF 定为 16（暂定值）。正式值按下面步骤地面重标：
//     ① main.c 自检把测试 PWM 依次改成 150/200/300，放地跑 2s，记 enc_delta；
//     ② 换算 v = delta/1040/2.0 * 0.2042（米/秒），每台两个点拟合 pwm = k*v + b；
//     ③ VEL_FF=k，FRICTION_FF=b（正反向各拟合一次，取各自值）。
#define VEL_FF      250.0f   // 每 m/s 需要的前馈 PWM（斜率，悬空/负载一致）
#define FRICTION_FF 16.0f    // 克服摩擦的固定 PWM（截距）：2026-08 实测反推≈16（前进）/≈8（后退）
#define START_KICK  300.0f   // 启动 kick：静止且目标非零时强推该 PWM 突破静摩擦（原 500 起步瞬态过大）
#define BRAKE_GAIN  400.0f   // 刹车：目标为0、轮子还在转时，反向 PWM = -current*BRAKE_GAIN

void Motor_Control_Update(Motor_Control_t *motor, float dt)
{

    // 目标≈0（停车/换向间隙）时清零积分，防止往返测试里
    // 前进段积的负积分被带到后退段，导致后退起步过猛（2026-08 前后差 22%）。
    if (fabs(motor->target_speed) < 0.05f)
        motor->pid.integral = 0.0f;

    //读取编码器速度 (m/s)

    motor->current_speed =
        Encoder_GetSpeed(motor->encoder, dt);

    //PID计算（修正误差，主推力靠前馈）

    float pid_out =
        PID_Calc(&motor->pid,
                 motor->target_speed,
                 motor->current_speed,
								 dt);

    // ===== 速度前馈：摩擦补偿 + 速度比例 =====
    // pwm = 60*sign(target) + 250*target + PID
    // 命令 0.2 -> 110 PWM（不是 60），稳超摩擦阈值，轮子能匀速走。
    float ff = motor->target_speed * VEL_FF;
    if (motor->target_speed >  0.05f) ff += FRICTION_FF;
    if (motor->target_speed < -0.05f) ff -= FRICTION_FF;
    motor->pwm_output = ff + pid_out;

    // ===== 启动 kick：突破静摩擦（正反向对称） =====
    // 轮子静止(current_speed≈0)且目标非零时，强推到 START_KICK 突破静摩擦，
    // 转起来后自动让位给前馈+PID。旋转命令下一半轮子目标是负的，所以正反向都要。
    if (fabs(motor->current_speed) < 0.05f)
    {
        if (motor->target_speed >  0.05f && motor->pwm_output <  START_KICK)
            motor->pwm_output =  START_KICK;
        if (motor->target_speed < -0.05f && motor->pwm_output > -START_KICK)
            motor->pwm_output = -START_KICK;
    }

    // ===== 刹车：目标为0但轮子还在转 =====
    // 修停机滑行：发 0 后 2-PWM 驱动板两边都为低 = 自由滑行，靠 P 项(40*current)刹不住。
    // 这里用更猛的反向 PWM 制动，消除 ~0.4m 的滑行。
    if (fabs(motor->target_speed) < 0.05f && fabs(motor->current_speed) > 0.05f)
    {
        motor->pwm_output = -motor->current_speed * BRAKE_GAIN;
    }

    //PWM限制

    if(motor->pwm_output > 1000)
        motor->pwm_output = 1000;


    if(motor->pwm_output < -1000)
        motor->pwm_output = -1000;

}