/**
 * @file    bsp_can.c
 * @brief   CAN1 板级驱动实现。
 * @details
 *   本层只处理 CAN 外设的公共收发流程：
 *   1. 配置全接收滤波器并启动 CAN；
 *   2. 发送经典 CAN 标准数据帧；
 *   3. 从 FIFO0 取出报文并分发给设备层回调。
 */

#include "bsp_can.h"

/* 每个 CAN 句柄最多允许注册的接收回调数量。 */
#define BSP_CAN_MAX_RX_CALLBACKS 4U

/* 接收回调注册表项。 */
typedef struct
{
  CAN_HandleTypeDef *hcan;       /* 回调绑定的 CAN 句柄 */
  BSP_CAN_RxCallback callback;   /* 设备层接收回调 */
  uint8_t used;                  /* 1：表项已占用；0：表项空闲 */
} BSP_CAN_CallbackEntry;

/* 接收回调注册表，静态分配，避免中断中使用动态内存。 */
static BSP_CAN_CallbackEntry s_rx_callbacks[BSP_CAN_MAX_RX_CALLBACKS];

/* CAN 总线调试计数，可在调试器中直接观察。 */
volatile uint32_t bsp_can_rx_count;       /* 已分发到设备层的接收帧数 */
volatile uint32_t bsp_can_tx_count;       /* 成功加入发送邮箱的帧数 */
volatile uint32_t bsp_can_tx_fail_count;  /* 发送邮箱满或入队失败的帧数 */

/* ========================== 初始化接口 ========================== */

uint8_t BSP_CAN_Init(CAN_HandleTypeDef *hcan)
{
  CAN_FilterTypeDef filter = {0};

  if (hcan == NULL)
  {
    return BSP_CAN_ERR_PARAM;
  }

  /*
   * READY：首次初始化，需要配置滤波器并启动 CAN。
   * LISTENING：CAN 已启动，只补充开启 FIFO0 接收中断。
   * 其他状态：BUSY/SLEEP/ERROR 等状态不在这里强行恢复，直接返回错误。
   */
  if (HAL_CAN_GetState(hcan) == HAL_CAN_STATE_READY)
  {
    /* FilterBank 0，32 位掩码模式，掩码全 0 表示接收所有标准/扩展帧。 */
    filter.FilterBank = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK)
    {
      return BSP_CAN_ERR_STATE;
    }
    if (HAL_CAN_Start(hcan) != HAL_OK)
    {
      return BSP_CAN_ERR_STATE;
    }
  }
  else if (HAL_CAN_GetState(hcan) != HAL_CAN_STATE_LISTENING)
  {
    return BSP_CAN_ERR_STATE;
  }

  /* FIFO0 收到新报文时触发 HAL_CAN_RxFifo0MsgPendingCallback。 */
  if (HAL_CAN_ActivateNotification(
          hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    return BSP_CAN_ERR_STATE;
  }

  return BSP_CAN_OK;
}

/* ========================== 发送接口 ========================== */

uint8_t BSP_CAN_SendStd(CAN_HandleTypeDef *hcan,
                        uint16_t std_id,
                        const uint8_t *data,
                        uint8_t dlc)
{
  CAN_TxHeaderTypeDef header = {0};
  uint32_t mailbox;

  /* 标准帧 ID 为 11 位，经典 CAN 单帧最多携带 8 字节数据。 */
  if ((hcan == NULL) || (data == NULL) ||
      (std_id > 0x07FFU) || (dlc > 8U))
  {
    return BSP_CAN_ERR_PARAM;
  }

  /* 先检查空闲邮箱，避免 HAL 调用失败后再区分具体错误原因。 */
  if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
  {
    bsp_can_tx_fail_count++;
    return BSP_CAN_ERR_TX_FULL;
  }

  /* 统一发送经典 CAN 标准数据帧，不使用远程帧和全局时间戳。 */
  header.StdId = std_id;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = dlc;
  header.TransmitGlobalTime = DISABLE;

  if (HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox) != HAL_OK)
  {
    bsp_can_tx_fail_count++;
    return BSP_CAN_ERR_TX;
  }

  bsp_can_tx_count++;
  return BSP_CAN_OK;
}

/* ======================== 接收回调管理 ======================== */

void BSP_CAN_RegisterRxCallback(CAN_HandleTypeDef *hcan,
                                BSP_CAN_RxCallback callback)
{
  uint32_t i;

  if ((hcan == NULL) || (callback == NULL))
  {
    return;
  }

  /* 同一 CAN 句柄和同一回调只注册一次，防止重复分发。 */
  for (i = 0U; i < BSP_CAN_MAX_RX_CALLBACKS; i++)
  {
    if ((s_rx_callbacks[i].used != 0U) &&
        (s_rx_callbacks[i].hcan == hcan) &&
        (s_rx_callbacks[i].callback == callback))
    {
      return;
    }
  }

  /* 找到第一个空闲表项；注册表满时静默返回。 */
  for (i = 0U; i < BSP_CAN_MAX_RX_CALLBACKS; i++)
  {
    if (s_rx_callbacks[i].used == 0U)
    {
      s_rx_callbacks[i].hcan = hcan;
      s_rx_callbacks[i].callback = callback;
      s_rx_callbacks[i].used = 1U;
      return;
    }
  }
}

/* ======================== 接收分发流程 ======================== */

/* 统计一帧接收数据，并依次通知绑定到当前 CAN 句柄的设备回调。 */
static void BSP_CAN_Dispatch(CAN_HandleTypeDef *hcan,
                             const CAN_RxHeaderTypeDef *header,
                             const uint8_t data[8])
{
  uint32_t i;

  bsp_can_rx_count++;
  for (i = 0U; i < BSP_CAN_MAX_RX_CALLBACKS; i++)
  {
    if ((s_rx_callbacks[i].used != 0U) &&
        (s_rx_callbacks[i].hcan == hcan) &&
        (s_rx_callbacks[i].callback != NULL))
    {
      s_rx_callbacks[i].callback(hcan, header, data);
    }
  }
}

void BSP_CAN_ProcessRx(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];

  if (hcan == NULL)
  {
    return;
  }

  /* 持续清空 FIFO0，避免接收缓冲区积压。 */
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) != 0U)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,
                             &header, data) != HAL_OK)
    {
      break;
    }

    /* 设备层只接收经典 CAN 标准数据帧，扩展帧和远程帧直接丢弃。 */
    if ((header.IDE == CAN_ID_STD) &&
        (header.RTR == CAN_RTR_DATA))
    {
      BSP_CAN_Dispatch(hcan, &header, data);
    }
  }
}

/* ========================== 中断回调 ========================== */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* HAL 在 FIFO0 有新报文时调用本函数，由公共接收流程完成分发。 */
  BSP_CAN_ProcessRx(hcan);
}
