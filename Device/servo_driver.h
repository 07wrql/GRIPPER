#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include "stm32f4xx_hal.h"

/* 舵机商品参数给出的脉宽范围和角度范围。 */
#define SERVO_MIN_PULSE_US  500U
#define SERVO_MAX_PULSE_US  2500U
#define SERVO_MAX_ANGLE_DEG 180

/* 一个舵机实例关联一个定时器句柄和一个 PWM 输出通道。 */
typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
} Servo_HandleTypeDef;

/* 初始化并启动 PWM；要求定时器计数单位为 1 us，自动重装值至少为 2500。 */
/* 初始化只启动当前比较值，不会设置中位或其他目标位置。 */
HAL_StatusTypeDef Servo_Init(Servo_HandleTypeDef *servo,
                             TIM_HandleTypeDef *htim,
                             uint32_t channel);
/* 直接设置脉宽，超出 500~2500 us 时会限制到边界。 */
HAL_StatusTypeDef Servo_SetPulseUs(Servo_HandleTypeDef *servo,
                                   uint32_t pulse_us);
/* 将 0~180 度线性换算为脉宽，超出范围的角度会先限幅。 */
HAL_StatusTypeDef Servo_SetAngle(Servo_HandleTypeDef *servo,
                                 int32_t angle_deg);
/* 停止 PWM 输出；普通舵机停止脉冲后通常不再主动保持位置。 */
HAL_StatusTypeDef Servo_Stop(Servo_HandleTypeDef *servo);

#endif /* SERVO_DRIVER_H */
