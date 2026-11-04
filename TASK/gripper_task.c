/**
 * @file    gripper_task.c
 * @brief   夹爪任务入口。
 * @details
 *   任务层只负责遥控数据更新、调用唯一夹爪控制入口和 1 ms 周期延时。
 *   J60 初始化、故障重试、气泵、光电门和舵机流程均在控制层内部完成。
 */

#include "gripper_task.h"
#include "gripper_control.h"
#include "cmsis_os2.h"
#include "can.h"
#include "usart.h"

/* 保留为全局变量，便于在 Keil Watch 中查看遥控器原始数据。 */
Remote_HandleTypeDef s_remote;

void GRIPPERTask(void *argument)
{
  (void)argument;

  if (Remote_Init(&s_remote, &huart1) != HAL_OK)
  {
    for (;;)
    {
      osDelay(1000U);
    }
  }

  for (;;)
  {
    (void)Remote_Update(&s_remote);

    /* 夹爪唯一的控制入口。 */
    GripperControl_Run(&s_remote.data, &hcan1);

    osDelay(1U);
  }
}
