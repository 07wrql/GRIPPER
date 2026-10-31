/**
  ******************************************************************************
  * @file           : photogate_driver.h
  * @brief          : STAB-13DTN 对射式光电门驱动
  ******************************************************************************
  * 适用传感器：
  *   STAB-13DTN，超薄正面检测对射式光电开关
  *   工作电压：DC12~24V
  *   输出类型：NPN 开集输出
  *   输出模式：Dark-on，光束被遮挡时输出有效
  *
  * 本工程连接：
  *   传感器棕色线 -> DC12~24V 正极
  *   传感器蓝色线 -> DC12~24V 负极
  *   传感器黑色线 -> PF1 (PHOTO_GATE_IN)
  *   传感器电源负极 -> STM32 GND
  *
  * 电平关系：
  *   PF1 已由 CubeMX 配置为内部上拉输入。
  *   光束未遮挡：NPN 截止，PF1 被上拉为高电平。
  *   光束被遮挡：NPN 导通，PF1 被拉到 GND，变为低电平。
  *
  * 注意：
  *   1. 传感器本体使用 DC12~24V 供电，不能接 5V 或 3.3V。
  *   2. NPN 开集输出可以接 PF1，因为 PF1 只提供 3.3V 上拉。
  *   3. GPIO 的输入模式由 CubeMX 配置，驱动不会重新配置 GPIO。
  ******************************************************************************
  */

#ifndef PHOTOGATE_DRIVER_H
#define PHOTOGATE_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/*
 * STAB-13DTN 的型号配置：
 *   D -> Dark-on
 *   N -> NPN 输出
 *
 * 初始化时统一使用该默认模式，避免调用者误传 Light-on。
 */
typedef enum
{
    PHOTO_GATE_LIGHT_ON = 0, /* 入光/光束畅通时输出有效。 */
    PHOTO_GATE_DARK_ON  = 1  /* 遮光/光束被挡时输出有效。 */
} PhotoGate_Mode;

#define PHOTO_GATE_STAB13DTN_MODE PHOTO_GATE_DARK_ON

/* 对外暴露的逻辑状态，屏蔽具体传感器电平。 */
typedef enum
{
    PHOTO_GATE_BEAM_CLEAR   = 0, /* 光束畅通，没有检测到物体。 */
    PHOTO_GATE_BEAM_BLOCKED = 1  /* 光束被遮挡，检测到物体。 */
} PhotoGate_BeamState;

/*
 * 一个句柄描述一路光电门。
 * 同一驱动可以创建多个句柄，分别绑定不同的 GPIO，例如四路光电门。
 */
typedef struct
{
    GPIO_TypeDef *port;  /* GPIO 端口，例如 GPIOF。 */
    uint16_t pin;        /* 单个 GPIO 引脚掩码，例如 GPIO_PIN_1。 */
    PhotoGate_Mode mode; /* 当前传感器的输出模式。 */
} PhotoGate_HandleTypeDef;

/**
  * @brief  初始化一路光电门。
  * @param  sensor 光电门句柄。
  * @param  port 已由 CubeMX 配置为输入上拉的 GPIO 端口。
  * @param  pin 已由 CubeMX 配置为输入上拉的单个 GPIO 引脚。
  * @param  mode 传感器输出模式；STAB-13DTN 使用 PHOTO_GATE_DARK_ON。
  * @note   调用前必须先执行 MX_GPIO_Init()。
  *         本函数只保存连接关系，不修改 GPIO 的输入/输出方向。
  * @retval HAL_OK 初始化成功。
  * @retval HAL_ERROR 句柄、端口、引脚或模式参数无效。
  */
HAL_StatusTypeDef PhotoGate_Init(PhotoGate_HandleTypeDef *sensor,
                                 GPIO_TypeDef *port,
                                 uint16_t pin,
                                 PhotoGate_Mode mode);

/**
  * @brief  读取 PF1 的原始电平。
  * @param  sensor 已初始化的光电门句柄。
  * @param  level 输出 GPIO_PIN_SET（高电平）或 GPIO_PIN_RESET（低电平）。
  * @note   该接口主要用于接线调试。正常业务判断建议使用
  *         PhotoGate_ReadBeamState()，它已经处理了 Dark-on/Light-on 模式。
  * @retval HAL_OK 读取成功。
  * @retval HAL_ERROR 句柄或输出指针无效。
  */
HAL_StatusTypeDef PhotoGate_ReadRawLevel(
    const PhotoGate_HandleTypeDef *sensor,
    GPIO_PinState *level);

/**
  * @brief  判断 NPN 输出是否处于有效状态。
  * @param  sensor 已初始化的光电门句柄。
  * @param  active 1 表示 NPN 输出导通，0 表示输出截止。
  * @note   NPN 开集输出导通时，信号线被拉到 GND，因此低电平为有效状态。
  * @retval HAL_OK 读取成功。
  * @retval HAL_ERROR 参数无效。
  */
HAL_StatusTypeDef PhotoGate_ReadOutputActive(
    const PhotoGate_HandleTypeDef *sensor,
    uint8_t *active);

/**
  * @brief  获取光束状态。
  * @param  sensor 已初始化的光电门句柄。
  * @param  state 输出 PHOTO_GATE_BEAM_CLEAR 或 PHOTO_GATE_BEAM_BLOCKED。
  * @note   驱动会根据初始化时传入的 Light-on/Dark-on 模式完成换算，
  *         调用者不需要再判断 GPIO 是高电平还是低电平。
  * @retval HAL_OK 读取成功。
  * @retval HAL_ERROR 参数无效。
  */
HAL_StatusTypeDef PhotoGate_ReadBeamState(
    const PhotoGate_HandleTypeDef *sensor,
    PhotoGate_BeamState *state);

/**
  * @brief  以布尔值形式读取光束是否被遮挡。
  * @param  sensor 已初始化的光电门句柄。
  * @param  blocked 1 表示光束被遮挡，0 表示光束畅通。
  * @note   这是 PhotoGate_ReadBeamState() 的兼容封装，适合直接用于条件判断。
  * @retval HAL_OK 读取成功。
  * @retval HAL_ERROR 参数无效。
  */
HAL_StatusTypeDef PhotoGate_ReadBeamBlocked(
    const PhotoGate_HandleTypeDef *sensor,
    uint8_t *blocked);

#ifdef __cplusplus
}
#endif

#endif /* PHOTOGATE_DRIVER_H */
