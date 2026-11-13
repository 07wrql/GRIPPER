/**
  ******************************************************************************
  * @file           : relay_driver.c
  * @brief          : 单路 5V 继电器模块控制驱动
  ******************************************************************************
  * 光电门接com和nc(常闭)，一上电就给电磁阀通电，使电磁阀通气方向转换，夹爪张开，等待识别物体
  ******************************************************************************
  */

#include "relay_driver.h"

/* 继电器:高电平吸合，低电平释放 */
#define RELAY_ACTIVE_LEVEL   GPIO_PIN_SET
#define RELAY_INACTIVE_LEVEL GPIO_PIN_RESET


void Relay_Init(void)
{
    HAL_GPIO_WritePin(RELAY_IN_GPIO_PORT,
                      RELAY_IN_GPIO_PIN,
                      RELAY_INACTIVE_LEVEL);
}

void Relay_Set(uint8_t enabled)
{
    HAL_GPIO_WritePin(RELAY_IN_GPIO_PORT,
                      RELAY_IN_GPIO_PIN,
                      (enabled != 0U) ? RELAY_ACTIVE_LEVEL
                                     : RELAY_INACTIVE_LEVEL);
}
