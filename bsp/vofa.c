#include "vofa.h"

#include <stdlib.h>
#include <string.h>

#define VOFA_RX_RING_SIZE       128U
#define VOFA_CMD_SIZE            64U
#define VOFA_SEND_PERIOD_MS      100U
#define VOFA_MAX_CHANNELS         8U

static UART_HandleTypeDef *s_vofa_huart = NULL;
static VofaPort_t s_vofa_port;

static uint8_t s_rx_byte;

static uint8_t s_rx_ring[VOFA_RX_RING_SIZE];
static volatile uint16_t s_rx_head = 0U;
static volatile uint16_t s_rx_tail = 0U;

static char s_cmd[VOFA_CMD_SIZE];
static uint16_t s_cmd_len = 0U;
static uint8_t s_cmd_overflow = 0U;

static uint8_t s_tx_buf[(VOFA_MAX_CHANNELS * sizeof(float)) + 4U];
static volatile uint8_t s_tx_busy = 0U;

static uint32_t s_last_send_ms = 0U;

static void Vofa_ProcessRx(void);
static void Vofa_ApplyCommand(char *cmd);
static void Vofa_SendTelemetry(const float *values, uint8_t count);

/**
 * @brief  解析并转发一条命令。
 * @note   支持P和D，例如P=50.0!、D=1.2!。
 */
static void Vofa_ApplyCommand(char *cmd)
{
  char *equal_sign;
  char *end_ptr = NULL;
  float value;

  equal_sign = strchr(cmd, '=');
  if (equal_sign == NULL)
  {
    return;
  }

  *equal_sign = '\0';

  value = strtof(equal_sign + 1, &end_ptr);
  if ((end_ptr == (equal_sign + 1)) || (*end_ptr != '\0'))
  {
    return;
  }

  if (s_vofa_port.on_command != NULL)
  {
    s_vofa_port.on_command(cmd[0], value);
  }
}

/**
 * @brief  从环形缓冲区取出并解析命令。
 */
static void Vofa_ProcessRx(void)
{
  uint8_t ch;

  if (s_vofa_huart == NULL)
  {
    return;
  }

  while (s_rx_tail != s_rx_head)
  {
    ch = s_rx_ring[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) % VOFA_RX_RING_SIZE);

    if (ch == '\r')
    {
      continue;
    }

    if ((ch == '!') || (ch == '\n'))
    {
      if ((s_cmd_overflow == 0U) && (s_cmd_len > 0U))
      {
        s_cmd[s_cmd_len] = '\0';
        if (strchr(s_cmd, '=') != NULL)
        {
          Vofa_ApplyCommand(s_cmd);
        }
      }

      s_cmd_len = 0U;
      s_cmd_overflow = 0U;
      continue;
    }

    /* 丢弃二进制数据、空字节和其他非ASCII字节，避免污染命令缓冲。 */
    if ((ch < 0x20U) || (ch > 0x7EU))
    {
      s_cmd_len = 0U;
      s_cmd_overflow = 0U;
      continue;
    }

    /* 当前只接受P=或D=命令，其他前缀直接丢弃并重新同步。 */
    if (s_cmd_len == 0U)
    {
      if ((ch != 'P') && (ch != 'D'))
      {
        continue;
      }
    }
    else if (s_cmd_len == 1U)
    {
      if (ch != '=')
      {
        s_cmd_len = 0U;
        s_cmd_overflow = 0U;
        continue;
      }
    }

    if (s_cmd_len < (VOFA_CMD_SIZE - 1U))
    {
      s_cmd[s_cmd_len] = (char)ch;
      ++s_cmd_len;
    }
    else
    {
      s_cmd_overflow = 1U;
    }
  }
}

/**
 * @brief  发送一帧JustFloat遥测数据。
 * @param  values 浮点通道数组。
 * @param  count  通道数量，最多VOFA_MAX_CHANNELS。
 */
static void Vofa_SendTelemetry(const float *values, uint8_t count)
{
  static const uint8_t tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};
  uint8_t payload_size;

  if ((s_vofa_huart == NULL) || (values == NULL) ||
      (count == 0U) || (count > VOFA_MAX_CHANNELS) ||
      (s_tx_busy != 0U))
  {
    return;
  }

  payload_size = (uint8_t)(count * sizeof(float));
  memcpy(&s_tx_buf[0], values, payload_size);
  memcpy(&s_tx_buf[payload_size], tail, sizeof(tail));

  s_tx_busy = 1U;

  if (HAL_UART_Transmit_IT(s_vofa_huart,
                           s_tx_buf,
                           (uint16_t)(payload_size + sizeof(tail))) != HAL_OK)
  {
    s_tx_busy = 0U;
  }
}

void Vofa_Init(UART_HandleTypeDef *huart, const VofaPort_t *port)
{
  if (huart == NULL)
  {
    return;
  }

  s_vofa_huart = huart;

  if (port == NULL)
  {
    memset(&s_vofa_port, 0, sizeof(s_vofa_port));
  }
  else
  {
    memcpy(&s_vofa_port, port, sizeof(s_vofa_port));
  }

  s_rx_head = 0U;
  s_rx_tail = 0U;

  s_cmd_len = 0U;
  s_cmd_overflow = 0U;

  s_tx_busy = 0U;

  memset(s_rx_ring, 0, sizeof(s_rx_ring));
  memset(s_cmd, 0, sizeof(s_cmd));
  memset(s_tx_buf, 0, sizeof(s_tx_buf));

  s_last_send_ms = HAL_GetTick();

  (void)HAL_UART_Receive_IT(s_vofa_huart, &s_rx_byte, 1U);
}

void Vofa_Task1ms(void)
{
  uint32_t now;

  if (s_vofa_huart == NULL)
  {
    return;
  }

  Vofa_ProcessRx();

  now = HAL_GetTick();
  if ((uint32_t)(now - s_last_send_ms) >= VOFA_SEND_PERIOD_MS)
  {
    float values[VOFA_MAX_CHANNELS];
    uint8_t count = 0U;

    s_last_send_ms = now;

    if (s_vofa_port.fill_telemetry != NULL)
    {
      count = s_vofa_port.fill_telemetry(values, VOFA_MAX_CHANNELS);
      if (count > VOFA_MAX_CHANNELS)
      {
        count = VOFA_MAX_CHANNELS;
      }

      Vofa_SendTelemetry(values, count);
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint16_t next_head;

  if ((s_vofa_huart == NULL) || (huart != s_vofa_huart))
  {
    return;
  }

  next_head = (uint16_t)((s_rx_head + 1U) % VOFA_RX_RING_SIZE);

  if (next_head != s_rx_tail)
  {
    s_rx_ring[s_rx_head] = s_rx_byte;
    s_rx_head = next_head;
  }

  (void)HAL_UART_Receive_IT(s_vofa_huart, &s_rx_byte, 1U);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((s_vofa_huart != NULL) && (huart == s_vofa_huart))
  {
    s_tx_busy = 0U;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((s_vofa_huart != NULL) && (huart == s_vofa_huart))
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    (void)HAL_UART_Receive_IT(s_vofa_huart, &s_rx_byte, 1U);
  }
}
