/**
  ******************************************************************************
  * @file    servo_driver.c
  * @brief   TIM2 通道 2 舵机 PWM 驱动实现
  ******************************************************************************
  * TIM2 由 CubeMX 配置为 1 us 计数、20 ms 周期
  * 舵机控制夹爪的翻转，夹取到物体之后就翻转180°
  ******************************************************************************
  */

#include "servo_driver.h"

/**
  * @brief  初始化并启动 TIM2_CH2 的 PWM 输出。
  */
HAL_StatusTypeDef Servo_Init(void)
{
    return HAL_TIM_PWM_Start(SERVO_TIM_HANDLE, SERVO_TIM_CHANNEL);
}

/**
  * @brief  设置舵机脉宽。
  * @param  pulse_us 目标脉宽，单位 us。
  * @note   输入超出 500~2500 us 时先限幅，再写入 TIM2 的 CCR2。
  */
void Servo_SetPulseUs(uint32_t pulse_us)
{
    /* 低于最小脉宽时限制到 500 us，避免舵机收到无效控制信号。 */
    if (pulse_us < SERVO_MIN_PULSE_US)
    {
        pulse_us = SERVO_MIN_PULSE_US;
    }
    /* 高于最大脉宽时限制到 2500 us，避免舵机撞限位。 */
    else if (pulse_us > SERVO_MAX_PULSE_US)
    {
        pulse_us = SERVO_MAX_PULSE_US;
    }

    /* TIM2 计数单位为 1 us，所以 CCR2 的值直接等于脉宽微秒数。 */
    __HAL_TIM_SET_COMPARE(SERVO_TIM_HANDLE, SERVO_TIM_CHANNEL, pulse_us);
}

/**
  * @brief  设置舵机角度。
  * @param  angle_deg 目标角度，单位度。
  * @note   0 度映射为 500 us，180 度映射为 2500 us。
  */
void Servo_SetAngle(int32_t angle_deg)
{
    uint32_t pulse_us;

    /* 角度低于 0 度时限制到 0 度。 */
    if (angle_deg < 0)
    {
        angle_deg = 0;
    }
    /* 角度高于 180 度时限制到 180 度。 */
    else if (angle_deg > SERVO_MAX_ANGLE_DEG)
    {
        angle_deg = SERVO_MAX_ANGLE_DEG;
    }

    /*
     * 脉宽 = 500 + (2500 - 500) * 角度 / 180
     * 加上 SERVO_MAX_ANGLE_DEG / 2 用于整数除法四舍五入。
     */
    pulse_us = SERVO_MIN_PULSE_US +
        ((SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) *
         (uint32_t)angle_deg + SERVO_MAX_ANGLE_DEG / 2U) /
        SERVO_MAX_ANGLE_DEG;

    /* 复用脉宽接口，把换算结果写入 CCR2。 */
    Servo_SetPulseUs(pulse_us);
}

/**
  * @brief  停止 TIM2_CH2 的 PWM 输出。
  * @note   仅停止当前通道，不影响 TIM2 的其他通道。
  */
HAL_StatusTypeDef Servo_Stop(void)
{
    /* 停止 PWM 后，普通舵机通常不再主动保持位置。 */
    return HAL_TIM_PWM_Stop(SERVO_TIM_HANDLE, SERVO_TIM_CHANNEL);
}