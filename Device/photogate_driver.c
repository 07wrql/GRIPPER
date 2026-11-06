/**
  ******************************************************************************
  * @file           : photogate_driver.c
  * @brief          : STAB-13DTN 单路对射式光电门驱动实现
  ******************************************************************************
  * 驱动只负责读取 GPIO 原始电平。
  * PF1 的输入模式和上拉配置由 CubeMX 生成在 gpio.c 中。
  ******************************************************************************
  */

#include "photogate_driver.h"

/**
  * @brief  读取光电门信号线的原始 GPIO 电平。
  * @note   原始电平的物理含义见 photogate_driver.h 中的实测说明。
  */
GPIO_PinState PhotoGate_ReadRawLevel(void)
{
    return HAL_GPIO_ReadPin(PHOTO_GATE_IN_GPIO_PORT,
                            PHOTO_GATE_IN_GPIO_PIN);
}
