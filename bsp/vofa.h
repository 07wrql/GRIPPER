#ifndef BSP_VOFA_H
#define BSP_VOFA_H

#include "stm32f4xx_hal.h"

/**
 * @brief 初始化VOFA并启动USART接收。
 * @param huart VOFA使用的串口，通常为&huart6。
 */
void Vofa_Init(UART_HandleTypeDef *huart);

/**
 * @brief VOFA周期任务。
 * @note 在GripperTask中每1ms调用一次，每100ms发送一帧4通道JustFloat：
 *       CH0=当前调试轴Kp，CH1=当前调试轴Kd，
 *       CH2=当前调试轴实际位置(rad)，CH3=当前调试轴目标位置(rad)。
 *       命令格式：P=50.0!、D=1.2!。
 *       调试轴由vofa.c中的VOFA_TUNE_AXIS选择：
 *       0U=大臂电机，1U=摩擦轮电机。
 */
void Vofa_Task1ms(void);


#endif /* BSP_VOFA_H */
