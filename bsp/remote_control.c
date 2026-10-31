/**
  ******************************************************************************
  * @file           : remote_control.c
  * @brief          : DJI DT7/DR16 遥控器驱动
  ******************************************************************************
  * 本文件逐项移植原 R2_CHASSIS_ZGT 工程中的：
  *   Remote::parseSBUS()
  *   Remote::monitor()
  *   Remote::getRCSnapshot()
  *   UART_DMA_Channel::InitMultiBufferDMA()
  *   UART_DMA_Channel::RxEventCallback()
  *   UART_DMA_Channel::ReEnable()
  ******************************************************************************
  */

#include "remote_control.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* 当前工程只有一路遥控器，串口中断通过该指针找到运行对象。 */
static Remote_HandleTypeDef *s_active_remote;

static void Remote_ReEnable(Remote_HandleTypeDef *remote)
{
    UART_HandleTypeDef *huart = remote->huart;

    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    __HAL_DMA_ENABLE(huart->hdmarx);
}

/**
  * @brief  按旧工程方式手动开启硬件双缓冲 DMA 和 UART 空闲中断。
  */
static void Remote_InitMultiBufferDMA(Remote_HandleTypeDef *remote)
{
    UART_HandleTypeDef *huart = remote->huart;
    DMA_Stream_TypeDef *dma;

    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    huart->RxXferSize = (uint16_t)(REMOTE_DMA_BUFFER_SIZE * 2U);
    huart->RxXferCount = (uint16_t)(REMOTE_DMA_BUFFER_SIZE * 2U);

    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    dma = (DMA_Stream_TypeDef *)huart->hdmarx->Instance;
    __HAL_DMA_DISABLE(huart->hdmarx);
    while ((dma->CR & DMA_SxCR_EN) != 0U)
    {
        /* 等待 DMA 真正停止后再修改双缓冲地址。 */
    }

    dma->PAR = (uint32_t)&huart->Instance->DR;
    SET_BIT(dma->CR, DMA_SxCR_DBM);
    CLEAR_BIT(dma->CR, DMA_SxCR_CT);
    dma->M0AR = (uint32_t)remote->buffer0;
    dma->M1AR = (uint32_t)remote->buffer1;
    dma->NDTR = (uint32_t)(REMOTE_DMA_BUFFER_SIZE * 2U);

    __HAL_DMA_ENABLE(huart->hdmarx);
}

/**
  * @brief  将 11 bit 原始通道值转换为以 0 为中心的值。
  */
static int16_t Remote_NormalizeChannel(uint16_t raw_value)
{
    return (int16_t)raw_value - REMOTE_CH_VALUE_OFFSET;
}

/**
  * @brief  严格按旧工程逐字节解析 remote 的四个摇杆和两个拨杆。
  */
static void Remote_ParseFrame(Remote_HandleTypeDef *remote, const uint8_t *buf)
{
    uint16_t raw[REMOTE_CH_COUNT];
    uint8_t i;

    raw[0] = (uint16_t)((buf[0] | ((uint16_t)buf[1] << 8)) & 0x07FFU);
    raw[1] = (uint16_t)(((buf[1] >> 3) |
                         ((uint16_t)buf[2] << 5)) & 0x07FFU);
    raw[2] = (uint16_t)(((buf[2] >> 6) |
                         ((uint16_t)buf[3] << 2) |
                         ((uint16_t)buf[4] << 10)) & 0x07FFU);
    raw[3] = (uint16_t)(((buf[4] >> 1) |
                         ((uint16_t)buf[5] << 7)) & 0x07FFU);

    for (i = 0U; i < REMOTE_CH_COUNT; i++)
    {
        remote->data.ch[i] = Remote_NormalizeChannel(raw[i]);
    }

    /* 与旧工程一致：S1 = bit7~6，S2 = bit5~4。 */
    remote->data.s[REMOTE_SW_LEFT_S1] =
        (uint8_t)((buf[5] >> 6) & 0x03U);
    remote->data.s[REMOTE_SW_RIGHT_S2] =
        (uint8_t)((buf[5] >> 4) & 0x03U);

    remote->data.online = 1U;
    remote->data.last_rx_ms = HAL_GetTick();
    remote->data.frame_count++;

    /* 与旧工程一致：每收到有效帧就把在线计数重置为 250。 */
    remote->online_cnt = 0xFAU;
    remote->rc_lost = 0U;
}

/**
  * @brief  处理一次 UART 空闲事件，切换硬件双缓冲的当前目标块。
  */
static void Remote_RxEventCallback(Remote_HandleTypeDef *remote, uint16_t size)
{
    UART_HandleTypeDef *huart = remote->huart;
    DMA_Stream_TypeDef *dma;

    dma = (DMA_Stream_TypeDef *)huart->hdmarx->Instance;

    __HAL_DMA_DISABLE(huart->hdmarx);
    while ((dma->CR & DMA_SxCR_EN) != 0U)
    {
        /* 等待 DMA 真正停止，否则 CT 切换不会生效。 */
    }

    if ((dma->CR & DMA_SxCR_CT) == RESET)
    {
        SET_BIT(dma->CR, DMA_SxCR_CT);
        dma->NDTR = (uint32_t)(REMOTE_DMA_BUFFER_SIZE * 2U);

        if (size == REMOTE_FRAME_SIZE)
        {
            Remote_ParseFrame(remote, remote->buffer0);
        }
        else
        {
            remote->data.error_count++;
        }
    }
    else
    {
        CLEAR_BIT(dma->CR, DMA_SxCR_CT);
        dma->NDTR = (uint32_t)(REMOTE_DMA_BUFFER_SIZE * 2U);

        if (size == REMOTE_FRAME_SIZE)
        {
            Remote_ParseFrame(remote, remote->buffer1);
        }
        else
        {
            remote->data.error_count++;
        }
    }

    Remote_ReEnable(remote);
}

HAL_StatusTypeDef Remote_Init(Remote_HandleTypeDef *remote,
                            UART_HandleTypeDef *huart)
{
    if (remote == NULL || huart == NULL || huart->hdmarx == NULL)
    {
        return HAL_ERROR;
    }

    memset(remote, 0, sizeof(*remote));
    remote->huart = huart;
    remote->online_cnt = 0xFAU;
    remote->rc_lost = 1U;
    remote->initialized = 1U;
    s_active_remote = remote;

    Remote_InitMultiBufferDMA(remote);
    return HAL_OK;
}

void Remote_Reset(Remote_HandleTypeDef *remote)
{
    if (remote == NULL || remote->initialized == 0U)
    {
        return;
    }

    taskENTER_CRITICAL();
    memset(&remote->data, 0, sizeof(remote->data));
    remote->online_cnt = 0U;
    remote->rc_lost = 1U;
    taskEXIT_CRITICAL();
}

uint8_t Remote_Update(Remote_HandleTypeDef *remote)
{
    if (remote == NULL || remote->initialized == 0U)
    {
        return 0U;
    }

    /* 与原工程 Remote::monitor() 完全一致。 */
    if (remote->online_cnt > 0x32U)
    {
        remote->online_cnt--;
        return (remote->rc_lost == 0U) ? 1U : 0U;
    }

    taskENTER_CRITICAL();
    if ((remote->online_cnt <= 0x32U) && (remote->rc_lost == 0U))
    {
        memset(&remote->data, 0, sizeof(remote->data));
        remote->rc_lost = 1U;
    }
    taskEXIT_CRITICAL();

    return (remote->rc_lost == 0U) ? 1U : 0U;
}

uint8_t Remote_IsOnline(const Remote_HandleTypeDef *remote)
{
    if (remote == NULL || remote->initialized == 0U)
    {
        return 0U;
    }

    return (remote->rc_lost == 0U) ? 1U : 0U;
}

uint8_t Remote_GetSnapshot(const Remote_HandleTypeDef *remote,
                         Remote_Data *snapshot)
{
    uint8_t online;

    if (remote == NULL || snapshot == NULL || remote->initialized == 0U)
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    *snapshot = remote->data;
    online = (remote->rc_lost == 0U) ? 1U : 0U;
    snapshot->online = online;
    taskEXIT_CRITICAL();

    return online;
}

float Remote_NormalizeStick(int16_t channel, float deadband)
{
    float value;
    float magnitude;
    float scaled;

    if (deadband < 0.0f)
    {
        deadband = 0.0f;
    }
    else if (deadband > 1.0f)
    {
        deadband = 1.0f;
    }

    value = (float)channel / REMOTE_STICK_HALF_RANGE;
    if (value > 1.0f)
    {
        value = 1.0f;
    }
    else if (value < -1.0f)
    {
        value = -1.0f;
    }

    magnitude = (value < 0.0f) ? -value : value;
    if (magnitude <= deadband)
    {
        return 0.0f;
    }

    scaled = (magnitude - deadband) / (1.0f - deadband);
    return (value < 0.0f) ? -scaled : scaled;
}

/**
  * @brief  对应旧工程 BSP_USART::RxEventDispatch()。
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    Remote_HandleTypeDef *remote = s_active_remote;

    if (remote == NULL || huart != remote->huart)
    {
        return;
    }

    Remote_RxEventCallback(remote, size);
}
