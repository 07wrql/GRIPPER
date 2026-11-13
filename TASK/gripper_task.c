/**
 * @file    gripper_task.c
 * @brief   夹爪自动流程任务入口。
 * @details
 *   当前工程只运行自动流程。任务上电后完成夹爪和J60电机初始化，
 *   随后进入GraspSkyBlock_AutoRun()。
 */

#include "gripper_task.h"
#include "Grasp_sky_block.h"
#include "cmsis_os2.h"
#include "can.h"

void GRIPPERTask(void *argument)
{
  (void)argument;

  /* 上电顺序初始化：先夹爪本体，再J60电机组。 */
  if (GraspSkyBlock_Init() != HAL_OK)
  {
    for (;;)
    {
      osDelay(1000U);
    }
  }

  if (GraspSkyBlock_MotorInit(&hcan1) != HAL_OK)
  {
    /* 给短暂拥塞或总线恢复留出时间，再尝试一次。 */
    osDelay(20U);
    if (GraspSkyBlock_MotorInit(&hcan1) != HAL_OK)
    {
      for (;;)
      {
        osDelay(1000U);
      }
    }
  }

  for (;;)
  {
    GraspSkyBlock_AutoRun();
    osDelay(1U);
  }
}
