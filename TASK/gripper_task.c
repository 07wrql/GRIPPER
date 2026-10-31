/**
  ******************************************************************************
  * @file    gripper_task.c
  * @brief   天空块夹取任务
  ******************************************************************************
  * 本阶段只接入遥控器、CAN1、4 号传送轮 J60 和 8 号大臂 J60。
  * 舵机、光电门和继电器驱动继续保留，但本任务暂不调用。
  ******************************************************************************
  */

#include "gripper_task.h"
#include "gripper_control.h"
#include "cmsis_os2.h"
#include "can.h"
#include "usart.h"

//s_remote.data可以查看遥控器数据
//不使用 static，确保 Keil Watch 在任意调试位置都能解析该变量。
Remote_HandleTypeDef s_remote;

//初始化标志，0表示未初始化
volatile uint8_t g_gripper_task_ready = 0U;


void GRIPPERTask(void *argument)
{
  uint32_t last_init_attempt_ms;

  (void)argument;

	//遥控初始化
  if (Remote_Init(&s_remote, &huart1) != HAL_OK)
  {
    g_gripper_task_ready = 0U;
    for (;;)
    {
      osDelay(1000U);
    }
  }

	//初始化 CAN 和 J60、清错、使能、等待反馈、确认在线、设置当前位置为目标、建立软件限幅、允许控制。
  GripperControl_Init(&hcan1);
  last_init_attempt_ms = HAL_GetTick();

  for (;;)
  {
    (void)Remote_Update(&s_remote);

    if (GripperControl_Ready() == 0U)
    {
      /*
       * J60 在调试暂停、瞬时掉线或错误锁存后会进入 FailSafe。
       * 不永久停在初始化失败分支，而是每秒重试清错、使能和反馈确认。
       */
      g_gripper_task_ready = 0U;
      if ((uint32_t)(HAL_GetTick() - last_init_attempt_ms) >= 1000U)
      {
        last_init_attempt_ms = HAL_GetTick();
        GripperControl_Init(&hcan1);
      }
    }
    else
    {
      /* 两个电机均在线且状态正常，允许控制输出。 */
      g_gripper_task_ready = 1U;
      GripperControl_Step(&s_remote.data);
    }

    osDelay(1U);
  }
}
