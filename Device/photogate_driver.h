/**
  ******************************************************************************
  * @file           : photogate_driver.h
  * @brief          : STAB-13DTN 单路对射式光电门驱动
  ******************************************************************************
  * 适用传感器：
  *   STAB-13DTN，超薄正面检测对射式光电开关
  *   工作电压：DC12~24V
  *   输出类型：NPN 集电极开路
  *   输出模式：Light-on（入光动作），光束畅通时输出有效
  *
  * 本工程连接：
  *   传感器棕色线 -> DC12~24V 正极
  *   传感器蓝色线 -> DC12~24V 负极
  *   传感器黑色线 -> PF1 (PHOTO_GATE_IN)
  *   传感器电源负极 -> STM32 GND
  *
  * 原始电平关系：
  *   PF1 已由 CubeMX 配置为内部上拉输入。
  *   光束未遮挡：NPN 导通，PF1 被拉到 GND，
  *                为低电平 GPIO_PIN_RESET（0）。
  *   光束被遮挡：NPN 截止，PF1 被上拉为高电平，
  *                为高电平 GPIO_PIN_SET（1）。
  *
  * 注意：
  *   1. 传感器本体使用 DC12~24V 供电，不能接 5V 或 3.3V。
  *   2. NPN 集电极开路输出可以接 PF1，因为 PF1 只提供 3.3V 上拉。
  *   3. GPIO 的输入模式由 CubeMX 配置，驱动不会重新配置 GPIO。
  ******************************************************************************
  */

#ifndef PHOTOGATE_DRIVER_H
#define PHOTOGATE_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* 光电门硬件映射：更换信号引脚时只需要修改这两个宏。 */
#define PHOTO_GATE_IN_GPIO_PORT    GPIOF
#define PHOTO_GATE_IN_GPIO_PIN     GPIO_PIN_1

/**
  * @brief  读取光电门信号线的原始 GPIO 电平。
  * @note   调用前必须先执行 MX_GPIO_Init()。
  * @retval GPIO_PIN_RESET（0）表示光束畅通，没有检测到物体。
  * @retval GPIO_PIN_SET（1）表示光束被遮挡，检测到物体。
  */
GPIO_PinState PhotoGate_ReadRawLevel(void);

#ifdef __cplusplus
}
#endif

#endif /* PHOTOGATE_DRIVER_H */
