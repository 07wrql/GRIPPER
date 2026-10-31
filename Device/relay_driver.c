/**
  ******************************************************************************
  * @file           : relay_driver.c
  * @brief          : 单路 5V 继电器模块控制驱动
  ******************************************************************************
  * 当前继电器模块的四路跳线均连接 HIGH 与中间针，因此每一路都是：
  *   控制端 = 高电平 -> 光耦导通 -> 三极管导通 -> 继电器线圈吸合
  *   控制端 = 低电平 -> 光耦截止 -> 三极管截止 -> 继电器线圈释放
  *
  * 本工程只使用第一路，控制端固定映射到 PF0。PF0 的电气配置由 CubeMX
  * 生成在 gpio.c 中，本文件只实现与业务无关的输出控制接口。
  ******************************************************************************
  */

#include "relay_driver.h"
#include "main.h"

/* 高电平触发板的有效和无效 GPIO 电平，修改硬件触发方式时只需调整这里。 */
#define RELAY_ACTIVE_LEVEL   GPIO_PIN_SET
#define RELAY_INACTIVE_LEVEL GPIO_PIN_RESET

/**
  * @brief  初始化继电器控制输出。
  * @note   MX_GPIO_Init() 已将 PF0 配置为推挽输出。这里再次写入无效电平，
  *         作为驱动层的安全初始化，确保应用开始运行前继电器处于断开状态。
  */
void Relay_Init(void)
{
    HAL_GPIO_WritePin(RELAY_IN_GPIO_Port, RELAY_IN_Pin, RELAY_INACTIVE_LEVEL);
}

/**
  * @brief  设置继电器逻辑状态。
  * @param  enabled 非 0 时吸合，为 0 时断开。
  * @note   参数按逻辑状态处理，不要求调用者关心高电平触发细节。
  */
void Relay_Set(uint8_t enabled)
{
    HAL_GPIO_WritePin(RELAY_IN_GPIO_Port,
                      RELAY_IN_Pin,
                      (enabled != 0U) ? RELAY_ACTIVE_LEVEL
                                     : RELAY_INACTIVE_LEVEL);
}

/**
  * @brief  使继电器吸合。
  * @note   当前为高电平触发，因此 PF0 输出高电平。
  */
void Relay_On(void)
{
    Relay_Set(1U);
}

/**
  * @brief  使继电器断开。
  * @note   当前为高电平触发，因此 PF0 输出低电平。
  */
void Relay_Off(void)
{
    Relay_Set(0U);
}

/**
  * @brief  切换继电器状态。
  * @note   先读取 PF0 当前输出逻辑，再写入相反状态。
  */
void Relay_Toggle(void)
{
    Relay_Set((uint8_t)!Relay_GetState());
}

/**
  * @brief  获取继电器当前逻辑状态。
  * @retval 1：PF0 为有效电平，继电器应处于吸合状态。
  * @retval 0：PF0 为无效电平，继电器应处于断开状态。
  * @note   该函数读取的是 GPIO 输出锁存结果，不提供继电器触点反馈。
  */
uint8_t Relay_GetState(void)
{
    return (HAL_GPIO_ReadPin(RELAY_IN_GPIO_Port, RELAY_IN_Pin) ==
            RELAY_ACTIVE_LEVEL)
               ? 1U
               : 0U;
}
