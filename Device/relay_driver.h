/**
  ******************************************************************************
  * @file           : relay_driver.h
  * @brief          : 单路 5V 继电器模块控制驱动
  ******************************************************************************
  * 硬件连接：
  *   PF0 (RELAY_IN) -> 继电器模块 IN1
  *   模块 VCC/GND     -> 独立 5V 电源
  *   模块 GND         -> 与 STM32 共地
  *
  * 触发方式：
  *   模块跳线帽连接 HIGH 与中间针，因此 PF0 输出高电平时继电器吸合，
  *   PF0 输出低电平时继电器断开。
  *
  * 使用说明：
  *   1. PF0 的推挽输出、下拉和低速配置由 CubeMX 在 MX_GPIO_Init() 中完成。
  *   2. 驱动不会修改 GPIO 模式，只负责输出电平和记录逻辑状态。
  *   3. 上电初始化后默认断开，避免负载在 MCU 复位期间意外动作。
  ******************************************************************************
  */

#ifndef RELAY_DRIVER_H
#define RELAY_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* 继电器硬件映射：更换控制引脚时只需要修改这两个宏。 */
#define RELAY_IN_GPIO_PORT    GPIOF
#define RELAY_IN_GPIO_PIN     GPIO_PIN_0

/**
  * @brief  初始化继电器控制输出。
  * @note   调用前必须先执行 MX_GPIO_Init()，确保 PF0 已配置为推挽输出。
  *         本函数会把 PF0 置为低电平，使高电平触发的继电器保持断开。
  * @retval 无
  */
void Relay_Init(void);

/**
  * @brief  设置继电器逻辑状态。
  * @param  enabled 非 0：继电器吸合；0：继电器断开。
  * @note   驱动按当前板子的高电平触发方式，将逻辑状态转换为 PF0 电平。
  * @retval 无
  */
void Relay_Set(uint8_t enabled);

/**
  * @brief  使继电器吸合。
  * @note   等价于 Relay_Set(1U)，PF0 输出高电平。
  * @retval 无
  */
void Relay_On(void);

/**
  * @brief  使继电器断开。
  * @note   等价于 Relay_Set(0U)，PF0 输出低电平。
  * @retval 无
  */
void Relay_Off(void);

/**
  * @brief  切换继电器状态。
  * @note   当前为吸合状态时断开，当前为断开状态时吸合。
  *         本函数通过读回 PF0 输出电平判断状态。
  * @retval 无
  */
void Relay_Toggle(void);

/**
  * @brief  获取继电器当前逻辑状态。
  * @note   返回值由 PF0 输出电平推导：1 表示吸合，0 表示断开。
  *         该接口不检测继电器触点或负载是否真正动作。
  * @retval 1：吸合；0：断开。
  */
uint8_t Relay_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* RELAY_DRIVER_H */
