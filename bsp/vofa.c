#include "vofa.h"
#include "Grasp_sky_block.h"

#include <stdlib.h>
#include <string.h>

#define VOFA_RX_RING_SIZE       128U
#define VOFA_CMD_SIZE            64U
#define VOFA_SEND_PERIOD_MS      20U

/* 当前曲线监视的电机轴：0=8号大臂，1=2号传送轮。 */
#define VOFA_MONITOR_AXIS         0U

static UART_HandleTypeDef *s_vofa_huart = NULL;

static uint8_t s_rx_byte;

static uint8_t s_rx_ring[VOFA_RX_RING_SIZE];
static volatile uint16_t s_rx_head = 0U;
static volatile uint16_t s_rx_tail = 0U;

static char s_cmd[VOFA_CMD_SIZE];
static uint16_t s_cmd_len = 0U;
static uint8_t s_cmd_overflow = 0U;

static uint8_t s_tx_buf[12];
static volatile uint8_t s_tx_busy = 0U;

static uint32_t s_last_send_ms = 0U;

static void Vofa_ProcessRx(void);
static void Vofa_ApplyCommand(char *cmd);
static void Vofa_SendTwoFloats(float target, float actual);

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
    return;
  }

  *equal_sign = '\0';

  value = strtof(equal_sign + 1, &end_ptr);
  if (end_ptr == (equal_sign + 1))
  {
    return;
  }

  if (strcmp(cmd, "P8") == 0)
  {
    (void)GraspSkyBlock_SetKp(0U, value);
  }
  else if (strcmp(cmd, "D8") == 0)
  {
    (void)GraspSkyBlock_SetKd(0U, value);
  }
  else if (strcmp(cmd, "P2") == 0)
  {
    (void)GraspSkyBlock_SetKp(1U, value);
  }
  else if (strcmp(cmd, "D2") == 0)
  {
    (void)GraspSkyBlock_SetKd(1U, value);
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
        Vofa_ApplyCommand(s_cmd);
      }

      s_cmd_len = 0U;
      s_cmd_overflow = 0U;
      continue;
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
 * @brief 发送目标位置和实际位置两个JustFloat通道。
 */
static void Vofa_SendTwoFloats(float target, float actual)
{
  static const uint8_t tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};

  if ((s_vofa_huart == NULL) || (s_tx_busy != 0U))
  {
    return;
  }

  memcpy(&s_tx_buf[0], &target, sizeof(float));
  memcpy(&s_tx_buf[4], &actual, sizeof(float));
  memcpy(&s_tx_buf[8], tail, sizeof(tail));

  s_tx_busy = 1U;

  if (HAL_UART_Transmit_IT(s_vofa_huart,
                           s_tx_buf,
                           sizeof(s_tx_buf)) != HAL_OK)
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
    float target_rad;
    float actual_rad;

    s_last_send_ms = now;

    if (GraspSkyBlock_GetTelemetry(VOFA_MONITOR_AXIS,
                                   &target_rad,
                                   &actual_rad) == HAL_OK)
    {
      Vofa_SendTwoFloats(target_rad, actual_rad);
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
