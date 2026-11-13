#ifndef BSP_VOFA_H
#define BSP_VOFA_H

#include "stm32f4xx_hal.h"

/**
 * @brief  VOFA业务回调。
 * @note   本模块只负责VOFA协议和串口收发，不依赖任何具体控制模块。
 */
typedef struct
{
  /**
   * @brief  收到P=...!或D=...!命令后调用。
   * @param  parameter 命令参数，当前为'P'或'D'。
   * @param  value     解析后的浮点数值。
   */
  void (*on_command)(char parameter, float value);

  /**
   * @brief  填充一帧JustFloat遥测通道。
   * @param  values   输出通道数据。
   * @param  capacity values数组允许写入的最大通道数。
   * @return 实际填充的通道数；返回0表示本次不发送遥测。
   */
  uint8_t (*fill_telemetry)(float *values, uint8_t capacity);
} VofaPort_t;

/**
 * @brief  初始化VOFA并启动串口接收。
 * @param  huart VOFA使用的串口。
 * @param  port  业务回调；传入NULL时模块保持静默。
 */
void Vofa_Init(UART_HandleTypeDef *huart, const VofaPort_t *port);

/**
 * @brief  VOFA周期任务，建议每1ms调用一次。
 * @note   命令解析每1ms进行，遥测按100ms周期发送。
 */
void Vofa_Task1ms(void);

#endif /* BSP_VOFA_H */
