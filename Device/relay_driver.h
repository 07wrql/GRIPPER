/**
  ******************************************************************************
  * @file           : relay_driver.h
  * @brief          : 单路 5V 继电器模块控制驱动
  ******************************************************************************
  * 硬件连接：
  *   PF0 (RELAY_IN) -> 继电器模块 IN2
  *   模块 VCC/GND     -> A板板载5v供电
  *   模块 GND         -> 与 单片机 共地
  *
  * 触发方式：
  *   由于继电器是接NC(2)和com，默认一上电就导通，所以当给到电平时吸合，是断电
  *
  * 使用说明：
  *   1. PF0 的推挽输出、下拉和低速配置由 CubeMX 在 MX_GPIO_Init() 中完成
  ******************************************************************************
  */

#ifndef RELAY_DRIVER_H
#define RELAY_DRIVER_H

#include "stm32f4xx_hal.h"

/* 继电器硬件映射 */
#define RELAY_IN_GPIO_PORT    GPIOF
#define RELAY_IN_GPIO_PIN     GPIO_PIN_0

void Relay_Init(void);

/**
  * @brief  设置继电器逻辑状态。
  */
void Relay_Set(uint8_t enabled);

/**
  * @brief  使继电器吸合。
  */
void Relay_On(void);

/**
  * @brief  使继电器断开。
  */
void Relay_Off(void);

/**
  * @brief  切换继电器状态。
  */
void Relay_Toggle(void);

/**
  * @brief  获取继电器当前逻辑状态。
  */
uint8_t Relay_GetState(void);


#endif /* RELAY_DRIVER_H */
