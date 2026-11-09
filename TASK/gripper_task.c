/**
 * @file    gripper_task.c
 * @brief   夹爪任务入口。
 * @details
 *   任务层负责遥控更新、上电初始化、周期控制和约 1 ms 的周期延时。 */

#include "gripper_task.h"
#include "Grasp_sky_block.h"
#include "cmsis_os2.h"
#include "can.h"
#include "usart.h"
#include "vofa.h"

/* 遥控器数据 */
Remote_HandleTypeDef s_remote;

void GRIPPERTask(void *argument)
{
  (void)argument;

  /*
   * 遥控接收初始化失败时，不进入后续初始化和控制流程。
   */
  if (Remote_Init(&s_remote, &huart1) != HAL_OK)
  {
    for (;;)
    {
      osDelay(1000U);
    }
  }

  /* 上电顺序初始化：先夹爪本体，再J60电机组。 */
  (void)GraspSkyBlock_Init();
  (void)GraspSkyBlock_MotorInit(&hcan1);

  /* VOFA使用USART6，统一由vofa模块处理收发。 */
  Vofa_Init(&huart6);

  for (;;)
  {
    /* 刷新在线计数和最新遥控数据。 */
    (void)Remote_Update(&s_remote);

    GraspSkyBlock_Run(&s_remote.data, &hcan1);

    /* 解析VOFA命令并每20ms发送目标/实际位置曲线。 */
    Vofa_Task1ms();
		
    /* 约1ms周期延时。 */
    osDelay(1U);
  }
}
