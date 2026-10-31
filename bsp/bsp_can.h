/**
  ******************************************************************************
  * @file    bsp_can.h
  * @brief   CAN1 板级驱动
  ******************************************************************************
  * 本层只负责 CAN 硬件收发的公共操作：
  *   1. 配置全接收滤波器、启动 CAN 和接收中断；
  *   2. 发送经典 CAN 标准数据帧；
  *   3. 从 FIFO0 取出报文并分发给已注册的设备回调。
  *
  * J60 的 ID、字节编码和反馈含义不属于本层。
  ******************************************************************************
  */

#ifndef BSP_CAN_H
#define BSP_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "can.h"

#define BSP_CAN_OK              0U
#define BSP_CAN_ERR_PARAM       1U
#define BSP_CAN_ERR_STATE       2U
#define BSP_CAN_ERR_TX_FULL     3U
#define BSP_CAN_ERR_TX          4U

typedef void (*BSP_CAN_RxCallback)(CAN_HandleTypeDef *hcan,
                                   const CAN_RxHeaderTypeDef *header,
                                   const uint8_t data[8]);

/* CAN 总线调试计数。 */
extern volatile uint32_t bsp_can_rx_count;
extern volatile uint32_t bsp_can_tx_count;
extern volatile uint32_t bsp_can_tx_fail_count;

/* 配置 CAN 滤波器、启动 CAN1 并开启 FIFO0 接收中断。 */
uint8_t BSP_CAN_Init(CAN_HandleTypeDef *hcan);

/* 发送一帧经典 CAN 标准数据帧。 */
uint8_t BSP_CAN_SendStd(CAN_HandleTypeDef *hcan,
                        uint16_t std_id,
                        const uint8_t *data,
                        uint8_t dlc);

/* 注册设备接收回调；同一句柄和回调重复注册会被忽略。 */
void BSP_CAN_RegisterRxCallback(CAN_HandleTypeDef *hcan,
                                BSP_CAN_RxCallback callback);

/* 从 FIFO0 取出全部报文并分发；可由前台周期调用。 */
void BSP_CAN_ProcessRx(CAN_HandleTypeDef *hcan);

#ifdef __cplusplus
}
#endif

#endif /* BSP_CAN_H */
