#include "servo_driver.h"

/* 仅支持 STM32 定时器的四个普通输出比较通道。 */
static uint8_t servo_channel_valid(uint32_t channel)
{
    switch (channel)
    {
    case TIM_CHANNEL_1:
    case TIM_CHANNEL_2:
    case TIM_CHANNEL_3:
    case TIM_CHANNEL_4:
        return 1U;
    default:
        return 0U;
    }
}

/* 检查实例是否已绑定有效定时器和通道。 */
static uint8_t servo_handle_valid(const Servo_HandleTypeDef *servo)
{
    return (servo != NULL && servo->htim != NULL &&
            servo_channel_valid(servo->channel));
}

HAL_StatusTypeDef Servo_Init(Servo_HandleTypeDef *servo,
                             TIM_HandleTypeDef *htim,
                             uint32_t channel)
{
    /* 防止无效参数及定时器周期不足以容纳最大舵机脉宽。 */
    if (servo == NULL || htim == NULL || !servo_channel_valid(channel) ||
        __HAL_TIM_GET_AUTORELOAD(htim) < SERVO_MAX_PULSE_US)
    {
        return HAL_ERROR;
    }

    servo->htim = htim;
    servo->channel = channel;

    /* 保留 CubeMX 设置的 CCR 初值；此处不发送中位或位置命令。 */
    return HAL_TIM_PWM_Start(servo->htim, servo->channel);
}

HAL_StatusTypeDef Servo_SetPulseUs(Servo_HandleTypeDef *servo,
                                   uint32_t pulse_us)
{
    /* 输入脉宽按舵机标称范围限幅，再写入对应通道的比较寄存器。 */
    if (!servo_handle_valid(servo))
    {
        return HAL_ERROR;
    }

    if (pulse_us < SERVO_MIN_PULSE_US)
    {
        pulse_us = SERVO_MIN_PULSE_US;
    }
    else if (pulse_us > SERVO_MAX_PULSE_US)
    {
        pulse_us = SERVO_MAX_PULSE_US;
    }

    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, pulse_us);
    return HAL_OK;
}

HAL_StatusTypeDef Servo_SetAngle(Servo_HandleTypeDef *servo,
                                 int32_t angle_deg)
{
    uint32_t pulse_us;

    /* 角度先限制在有效范围，再线性映射到脉宽范围。 */
    if (!servo_handle_valid(servo))
    {
        return HAL_ERROR;
    }

    if (angle_deg < 0)
    {
        angle_deg = 0;
    }
    {
        angle_deg = SERVO_MAX_ANGLE_DEG;
    }

    pulse_us = SERVO_MIN_PULSE_US +
        ((SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) *
         (uint32_t)angle_deg + SERVO_MAX_ANGLE_DEG / 2U) /
        SERVO_MAX_ANGLE_DEG;

    /* 本驱动要求计数单位为 1 us，因此比较值等于脉宽微秒数。 */
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, pulse_us);
    return HAL_OK;
}

HAL_StatusTypeDef Servo_Stop(Servo_HandleTypeDef *servo)
{
    /* 停止指定 PWM 通道，不影响同一定时器的其他通道。 */
    if (!servo_handle_valid(servo))
    {
        return HAL_ERROR;
    }

    return HAL_TIM_PWM_Stop(servo->htim, servo->channel);
}
