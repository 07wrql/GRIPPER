#include "vofa.h"
#include "Grasp_sky_block.h"

#include <stdlib.h>
#include <string.h>

#define VOFA_RX_RING_SIZE       128U
#define VOFA_CMD_SIZE            64U
#define VOFA_SEND_PERIOD_MS      100U

/* JustFloat遥测同时包含两个电机轴的位置和PD参数。 */
#define VOFA_CHANNEL_COUNT        10U

static UART_HandleTypeDef *s_vofa_huart = NULL;

static uint8_t s_rx_byte;

static uint8_t s_rx_ring[VOFA_RX_RING_SIZE];
static volatile uint16_t s_rx_head = 0U;
static volatile uint16_t s_rx_tail = 0U;

static char s_cmd[VOFA_CMD_SIZE];
static uint16_t s_cmd_len = 0U;
static uint8_t s_cmd_overflow = 0U;

static uint8_t s_tx_buf[(VOFA_CHANNEL_COUNT * sizeof(float)) + 4U];
static volatile uint8_t s_tx_busy = 0U;

static uint32_t s_last_send_ms = 0U;
static uint32_t s_cmd_count = 0U;
static float s_last_cmd_status = 0.0f;

static void Vofa_ProcessRx(void);
static void Vofa_ApplyCommand(char *cmd);
static void Vofa_SendTelemetry(const float *values, uint8_t count);

/**
 * @brief 解析并执行一条命令。
 * @note 支持P8、D8、P2、D2，例如P8=15.0!。
 */
static void Vofa_ApplyCommand(char *cmd)
{
  char *equal_sign;
  char *end_ptr = NULL;
  float value;

  equal_sign = strchr(cmd, '=');
  if (equal_sign == NULL)
  {
    s_last_cmd_status = -1.0f;
    return;
  }

  *equal_sign = '\0';

  value = strtof(equal_sign + 1, &end_ptr);
  if ((end_ptr == (equal_sign + 1)) || (*end_ptr != '\0'))
  {
    s_last_cmd_status = -1.0f;
    return;
  }

  /* CH9状态：1=P8成功，2=D8成功，3=P2成功，4=D2成功，-1=失败。 */
  if (strcmp(cmd, "P8") == 0)
  {
    s_last_cmd_status =
        (GraspSkyBlock_SetKp(0U, value) == HAL_OK) ? 1.0f : -1.0f;
  }
  else if (strcmp(cmd, "D8") == 0)
  {
    s_last_cmd_status =
        (GraspSkyBlock_SetKd(0U, value) == HAL_OK) ? 2.0f : -1.0f;
  }
  else if (strcmp(cmd, "P2") == 0)
  {
    s_last_cmd_status =
        (GraspSkyBlock_SetKp(1U, value) == HAL_OK) ? 3.0f : -1.0f;
  }
  else if (strcmp(cmd, "D2") == 0)
  {
    s_last_cmd_status =
        (GraspSkyBlock_SetKd(1U, value) == HAL_OK) ? 4.0f : -1.0f;
  }
  else
  {
    s_last_cmd_status = -1.0f;
  }
}

/**
 * @brief 从环形缓冲区取出并解析命令。
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
          ++s_cmd_count;
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

    /*
     * 命令只可能是P8=、D8=、P2=或D2=。
     * 如果当前字节不符合命令前缀，直接重新同步，避免把噪声拼进命令。
     */
    if (s_cmd_len == 0U)
    {
      if ((ch != 'P') && (ch != 'D'))
      {
        continue;
      }
    }
    else if (s_cmd_len == 1U)
    {
      if ((ch != '8') && (ch != '2'))
      {
        s_cmd_len = 0U;
        s_cmd_overflow = 0U;
        continue;
      }
    }
    else if (s_cmd_len == 2U)
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
 * @brief 发送一帧JustFloat遥测数据。
 * @param values 浮点通道数组。
 * @param count  通道数量，最多VOFA_CHANNEL_COUNT。
 */
static void Vofa_SendTelemetry(const float *values, uint8_t count)
{
  static const uint8_t tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};
  uint8_t payload_size;

  if ((s_vofa_huart == NULL) || (values == NULL) ||
      (count == 0U) || (count > VOFA_CHANNEL_COUNT) ||
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

void Vofa_Init(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return;
  }

  s_vofa_huart = huart;

  s_rx_head = 0U;
  s_rx_tail = 0U;

  s_cmd_len = 0U;
  s_cmd_overflow = 0U;

  s_tx_busy = 0U;
  s_cmd_count = 0U;
  s_last_cmd_status = 0.0f;

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
    float target0_rad;
    float actual0_rad;
    float target1_rad;
    float actual1_rad;
    float kp0;
    float kd0;
    float kp1;
    float kd1;
    float values[VOFA_CHANNEL_COUNT];

    s_last_send_ms = now;

    if ((GraspSkyBlock_GetTelemetry(0U,
                                    &target0_rad,
                                    &actual0_rad) == HAL_OK) &&
        (GraspSkyBlock_GetTelemetry(1U,
                                    &target1_rad,
                                    &actual1_rad) == HAL_OK) &&
        (GraspSkyBlock_GetTuning(0U, &kp0, &kd0) == HAL_OK) &&
        (GraspSkyBlock_GetTuning(1U, &kp1, &kd1) == HAL_OK))
    {
      values[0] = target0_rad;        /* CH0：轴0目标位置(rad) */
      values[1] = actual0_rad;        /* CH1：轴0实际位置(rad) */
      values[2] = target1_rad;        /* CH2：轴1目标位置(rad) */
      values[3] = actual1_rad;        /* CH3：轴1实际位置(rad) */
      values[4] = kp0;                /* CH4：轴0当前Kp */
      values[5] = kd0;                /* CH5：轴0当前Kd */
      values[6] = kp1;                /* CH6：轴1当前Kp */
      values[7] = kd1;                /* CH7：轴1当前Kd */
      values[8] = (float)s_cmd_count; /* CH8：完整命令计数 */
      values[9] = s_last_cmd_status;  /* CH9：最近命令结果 */
      Vofa_SendTelemetry(values, VOFA_CHANNEL_COUNT);
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
