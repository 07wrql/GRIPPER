/**
  ******************************************************************************
  * @file    bsp_can.c
  * @brief   CAN1 板级驱动实现
  ******************************************************************************
  */

#include "bsp_can.h"

#define BSP_CAN_MAX_RX_CALLBACKS 4U

typedef struct
{
  CAN_HandleTypeDef *hcan;
  BSP_CAN_RxCallback callback;
  uint8_t used;
} BSP_CAN_CallbackEntry;

static BSP_CAN_CallbackEntry s_rx_callbacks[BSP_CAN_MAX_RX_CALLBACKS];

volatile uint32_t bsp_can_rx_count;
volatile uint32_t bsp_can_tx_count;
volatile uint32_t bsp_can_tx_fail_count;

uint8_t BSP_CAN_Init(CAN_HandleTypeDef *hcan)
{
  CAN_FilterTypeDef filter = {0};

  if (hcan == NULL)
  {
    return BSP_CAN_ERR_PARAM;
  }

  /* 首次调用时配置滤波器并启动 CAN；恢复重试时总线可能已经处于 BUSY。 */
  if (HAL_CAN_GetState(hcan) == HAL_CAN_STATE_READY)
  {
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

  if (HAL_CAN_ActivateNotification(
          hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    return BSP_CAN_ERR_STATE;
  }

  return BSP_CAN_OK;
}

uint8_t BSP_CAN_SendStd(CAN_HandleTypeDef *hcan,
                        uint16_t std_id,
                        const uint8_t *data,
                        uint8_t dlc)
{
  CAN_TxHeaderTypeDef header = {0};
  uint32_t mailbox;

  if ((hcan == NULL) || (data == NULL) ||
      (std_id > 0x07FFU) || (dlc > 8U))
  {
    return BSP_CAN_ERR_PARAM;
  }

  if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
  {
    bsp_can_tx_fail_count++;
    return BSP_CAN_ERR_TX_FULL;
  }

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

void BSP_CAN_RegisterRxCallback(CAN_HandleTypeDef *hcan,
                                BSP_CAN_RxCallback callback)
{
  uint32_t i;

  if ((hcan == NULL) || (callback == NULL))
  {
    return;
  }

  for (i = 0U; i < BSP_CAN_MAX_RX_CALLBACKS; i++)
  {
    if ((s_rx_callbacks[i].used != 0U) &&
        (s_rx_callbacks[i].hcan == hcan) &&
        (s_rx_callbacks[i].callback == callback))
    {
      return;
    }
  }

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

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) != 0U)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,
                             &header, data) != HAL_OK)
    {
      break;
    }

    if ((header.IDE == CAN_ID_STD) &&
        (header.RTR == CAN_RTR_DATA))
    {
      BSP_CAN_Dispatch(hcan, &header, data);
    }
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  BSP_CAN_ProcessRx(hcan);
}
