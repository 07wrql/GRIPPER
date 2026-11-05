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

  /*
   * 接收机初始化失败时，所有控制任务都不会进入
	 * 接收机初始化成功但无线电失联时，J60后台任务继续运行，气泵和舵机进入安全状态，等待遥控恢复。
   */
  if (Remote_Init(&s_remote, &huart1) != HAL_OK)
  {
    for (;;)
    {
      osDelay(1000U);
    }
  }

  for (;;)
  {
    /* 刷新在线计数和最新遥控数据。 */
    (void)Remote_Update(&s_remote);

    GripperControl_Run(&s_remote.data, &hcan1);
		
    /* 固定1ms控制周期。 */
    osDelay(1U);
  }
}
