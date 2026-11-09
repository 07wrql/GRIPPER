#ifndef BSP_VOFA_H
#define BSP_VOFA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/**
 * @brief 初始化VOFA并启动USART接收。
 * @param huart VOFA使用的串口，通常为&huart6。
 */
void Vofa_Init(UART_HandleTypeDef *huart);

/**
 * @brief VOFA周期任务。
 * @note 在GripperTask中每1ms调用一次。
 */
void Vofa_Task1ms(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_VOFA_H */
