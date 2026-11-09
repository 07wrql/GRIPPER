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
    if (pulse_us < SERVO_MIN_PULSE_US)
    {
        pulse_us = SERVO_MIN_PULSE_US;
    }
    else if (pulse_us > SERVO_MAX_PULSE_US)
    {
        pulse_us = SERVO_MAX_PULSE_US;
    }
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
    if (angle_deg < 0)
    {
        angle_deg = 0;
    }
    else if (angle_deg > SERVO_MAX_ANGLE_DEG)
    {
        angle_deg = SERVO_MAX_ANGLE_DEG;
    }
    pulse_us = SERVO_MIN_PULSE_US +
        (((SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) *
          (uint32_t)angle_deg +
          ((uint32_t)SERVO_MAX_ANGLE_DEG / 2U)) /
         (uint32_t)SERVO_MAX_ANGLE_DEG);

    Servo_SetPulseUs(pulse_us);
}

HAL_StatusTypeDef Servo_Stop(void)
{
    return HAL_TIM_PWM_Stop(SERVO_TIM_HANDLE, SERVO_TIM_CHANNEL);
}