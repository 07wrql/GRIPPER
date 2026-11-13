/**
  ******************************************************************************
  * @file    servo_driver.h
  * @brief   TIM2 通道 2 舵机 PWM 驱动
  ******************************************************************************
  * 硬件映射：
  *   PA1 -> TIM2_CH2
  *
  * CubeMX 中的 TIM2 配置：
  *   Prescaler = 83      -> 定时器计数频率 1 MHz，每格 1 us
  *   Period    = 19999   -> PWM 周期 20000 us，即 20 ms
  *   PWM Mode  = PWM1
  ******************************************************************************
  */

#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include "tim.h"

/* 舵机 PWM 硬件映射 */
#define SERVO_TIM_HANDLE          (&htim2)        
#define SERVO_TIM_CHANNEL         TIM_CHANNEL_2   

/* 舵机商品参数给出的脉宽范围和角度范围。 */
#define SERVO_MIN_PULSE_US        500U            /* 0 度对应 500 us */
#define SERVO_MAX_PULSE_US        2500U           /* 180 度对应 2500 us */
#define SERVO_MAX_ANGLE_DEG       180             /* 舵机最大机械角度，单位度 */

/**
  * @brief  初始化并启动 TIM2_CH2 的 PWM 输出。
  */
HAL_StatusTypeDef Servo_Init(void);

/**
  * @brief  直接设置舵机脉宽。
  * @param  pulse_us 目标脉宽，单位 us；超出 500~2500 us 时自动限幅。
  * @note   脉冲宽度最终写入 TIM2 的 CCR2 寄存器。
  */
void Servo_SetPulseUs(uint32_t pulse_us);

/**
  * @brief  设置舵机角度。
  * @param  angle_deg 目标角度，单位度；超出 0~180 度时自动限幅。
  * @note   角度与脉宽按以下关系线性换算：
  *         pulse_us = 500 + (2500 - 500) * angle_deg / 180。
  */
void Servo_SetAngle(int32_t angle_deg);



#endif /* SERVO_DRIVER_H */