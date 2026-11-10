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
 * @note 在GripperTask中每1ms调用一次，每20ms发送一帧10通道JustFloat：
 *       CH0/CH1=轴0目标/实际位置，CH2/CH3=轴1目标/实际位置，
 *       CH4/CH5=轴0 Kp/Kd，CH6/CH7=轴1 Kp/Kd，
 *       CH8=命令计数，CH9=最近命令结果。
 *       命令格式：P8=15.0!、D8=0.8!、P2=12.0!、D2=0.8!。
 */
void Vofa_Task1ms(void);


#endif /* BSP_VOFA_H */
