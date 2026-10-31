/**
  ******************************************************************************
  * @file           : photogate_driver.c
  * @brief          : STAB-13DTN 对射式光电门驱动实现
  ******************************************************************************
  * 驱动只负责读取 GPIO 电平并换算为业务逻辑状态。
  * PF1 的输入模式和上拉配置由 CubeMX 生成在 gpio.c 中。
  ******************************************************************************
  */

#include "photogate_driver.h"

/**
  * @brief  检查光电门句柄是否有效。
  * @note   一个句柄只能绑定一个 GPIO 引脚，因此引脚掩码中必须只有一个位为 1。
  */
static uint8_t PhotoGate_IsHandleValid(
    const PhotoGate_HandleTypeDef *sensor)
{
    if (sensor == NULL || sensor->port == NULL || sensor->pin == 0U)
    {
        return 0U;
    }

    /* 典型的非法值例如 GPIO_PIN_0 | GPIO_PIN_1。 */
    if ((sensor->pin & (uint16_t)(sensor->pin - 1U)) != 0U)
    {
        return 0U;
    }

    return (sensor->mode == PHOTO_GATE_LIGHT_ON ||
            sensor->mode == PHOTO_GATE_DARK_ON)
               ? 1U
               : 0U;
}

/**
  * @brief  初始化一路光电门。
  * @note   本函数不调用 HAL_GPIO_Init()，GPIO 的电气配置由 CubeMX 负责。
  */
HAL_StatusTypeDef PhotoGate_Init(PhotoGate_HandleTypeDef *sensor,
                                 GPIO_TypeDef *port,
                                 uint16_t pin,
                                 PhotoGate_Mode mode)
{
    if (sensor == NULL || port == NULL || pin == 0U ||
        (pin & (uint16_t)(pin - 1U)) != 0U ||
        (mode != PHOTO_GATE_LIGHT_ON && mode != PHOTO_GATE_DARK_ON))
    {
        return HAL_ERROR;
    }

    sensor->port = port;
    sensor->pin = pin;
    sensor->mode = mode;
    return HAL_OK;
}

/**
  * @brief  读取传感器信号线的原始 GPIO 电平。
  */
HAL_StatusTypeDef PhotoGate_ReadRawLevel(
    const PhotoGate_HandleTypeDef *sensor,
    GPIO_PinState *level)
{
    if (!PhotoGate_IsHandleValid(sensor) || level == NULL)
    {
        return HAL_ERROR;
    }

    *level = HAL_GPIO_ReadPin(sensor->port, sensor->pin);
    return HAL_OK;
}

/**
  * @brief  判断 NPN 开集输出是否导通。
  * @note   NPN 输出导通时把信号线拉到 GND，所以 GPIO 低电平代表输出有效。
  */
HAL_StatusTypeDef PhotoGate_ReadOutputActive(
    const PhotoGate_HandleTypeDef *sensor,
    uint8_t *active)
{
    GPIO_PinState level;

    if (active == NULL ||
        PhotoGate_ReadRawLevel(sensor, &level) != HAL_OK)
    {
        return HAL_ERROR;
    }

    *active = (level == GPIO_PIN_RESET) ? 1U : 0U;
    return HAL_OK;
}

/**
  * @brief  获取归一化后的光束状态。
  *
  * STAB-13DTN 使用 Dark-on：
  *   未遮挡 -> NPN 截止 -> PF1 高电平 -> 光束畅通
  *   遮挡   -> NPN 导通 -> PF1 低电平 -> 光束被挡
  *
  * Light-on 模式则相反：
  *   未遮挡 -> 输出有效 -> 光束畅通
  *   遮挡   -> 输出无效 -> 光束被挡
  */
HAL_StatusTypeDef PhotoGate_ReadBeamState(
    const PhotoGate_HandleTypeDef *sensor,
    PhotoGate_BeamState *state)
{
    uint8_t output_active;

    if (state == NULL ||
        PhotoGate_ReadOutputActive(sensor, &output_active) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (sensor->mode == PHOTO_GATE_DARK_ON)
    {
        *state = (output_active != 0U)
                     ? PHOTO_GATE_BEAM_BLOCKED
                     : PHOTO_GATE_BEAM_CLEAR;
    }
    else
    {
        *state = (output_active != 0U)
                     ? PHOTO_GATE_BEAM_CLEAR
                     : PHOTO_GATE_BEAM_BLOCKED;
    }

    return HAL_OK;
}

/**
  * @brief  以布尔值形式读取光束是否被遮挡。
  */
HAL_StatusTypeDef PhotoGate_ReadBeamBlocked(
    const PhotoGate_HandleTypeDef *sensor,
    uint8_t *blocked)
{
    PhotoGate_BeamState state;

    if (blocked == NULL ||
        PhotoGate_ReadBeamState(sensor, &state) != HAL_OK)
    {
        return HAL_ERROR;
    }

    *blocked = (state == PHOTO_GATE_BEAM_BLOCKED) ? 1U : 0U;
    return HAL_OK;
}
