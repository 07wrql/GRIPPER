/**
 * @file    bsp_can.h
 * @brief   CAN1 板级驱动接口。
 * @details
 *   本层只负责 CAN 硬件收发的公共操作：
 *   1. 配置全接收滤波器、启动 CAN 和 FIFO0 接收中断；
 *   2. 发送经典 CAN 标准数据帧；
 *   3. 从 FIFO0 取出报文并分发给已注册的设备回调。
 *
 *   具体设备协议（例如 J60 的 ID、字节编码和反馈含义）
 *   不属于本层，由 Device 层解析。
 */

#ifndef BSP_CAN_H
#define BSP_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "can.h"

/* ========================== 返回值定义 ========================== */

#define BSP_CAN_OK              0U  /* 操作成功 */
#define BSP_CAN_ERR_PARAM       1U  /* 参数无效 */
#define BSP_CAN_ERR_STATE       2U  /* CAN 外设状态不允许当前操作 */
#define BSP_CAN_ERR_TX_FULL     3U  /* 发送邮箱已满，可稍后重试 */
#define BSP_CAN_ERR_TX          4U  /* HAL 发送入队失败 */

/* ========================== 回调类型 ========================== */

/**
 * @brief CAN 接收回调函数类型。
 * @param hcan   接收报文的 CAN 句柄。
 * @param header 接收帧头，包含标准 ID、DLC 等字段。
 * @param data   接收数据区，最多 8 字节。
 * @note  回调由 CAN 接收中断路径调用，应保持短小，不要阻塞。
 */
typedef void (*BSP_CAN_RxCallback)(CAN_HandleTypeDef *hcan,
                                   const CAN_RxHeaderTypeDef *header,
                                   const uint8_t data[8]);

/* ========================== 调试计数 ========================== */

/* 可在调试器中直接观察 CAN 总线收发状态。 */
extern volatile uint32_t bsp_can_rx_count;       /* 已分发的接收帧数 */
extern volatile uint32_t bsp_can_tx_count;       /* 成功入队的发送帧数 */
extern volatile uint32_t bsp_can_tx_fail_count;  /* 发送失败次数 */

/* ========================== 公共接口 ========================== */

/**
 * @brief  初始化 CAN 滤波器、启动 CAN1 并开启 FIFO0 接收中断。
 * @param  hcan 已完成 HAL 基础配置的 CAN 句柄。
 * @retval BSP_CAN_OK        初始化成功。
 * @retval BSP_CAN_ERR_PARAM hcan 为 NULL。
 * @retval BSP_CAN_ERR_STATE CAN 状态或 HAL 操作失败。
 * @note   若 CAN 已处于 LISTENING 状态，只补开接收通知，不重复启动外设。
 */
uint8_t BSP_CAN_Init(CAN_HandleTypeDef *hcan);

/**
 * @brief  发送一帧经典 CAN 标准数据帧。
 * @param  hcan   目标 CAN 句柄。
 * @param  std_id 11 位标准帧 ID。
 * @param  data   待发送数据区，不能为 NULL。
 * @param  dlc    有效数据长度，范围 0~8。
 * @retval BSP_CAN_OK        报文成功加入发送邮箱。
 * @retval BSP_CAN_ERR_PARAM 参数无效。
 * @retval BSP_CAN_ERR_TX_FULL 三个发送邮箱均已占用，可稍后重试。
 * @retval BSP_CAN_ERR_TX   HAL 发送入队失败。
 */
uint8_t BSP_CAN_SendStd(CAN_HandleTypeDef *hcan,
                        uint16_t std_id,
                        const uint8_t *data,
                        uint8_t dlc);

/**
 * @brief  注册设备层 CAN 接收回调。
 * @param  hcan     回调绑定的 CAN 句柄。
 * @param  callback 设备层接收回调。
 * @note   同一句柄和回调重复注册会被忽略；最多支持 4 个回调表项。
 */
void BSP_CAN_RegisterRxCallback(CAN_HandleTypeDef *hcan,
                                BSP_CAN_RxCallback callback);

/**
 * @brief  从 FIFO0 取出全部报文并分发给已注册回调。
 * @param  hcan 目标 CAN 句柄。
 * @note   本函数由接收中断调用，也可由前台周期调用。
 *         只向设备层分发标准数据帧，扩展帧和远程帧会被丢弃。
 */
void BSP_CAN_ProcessRx(CAN_HandleTypeDef *hcan);

#ifdef __cplusplus
}
#endif

#endif /* BSP_CAN_H */
