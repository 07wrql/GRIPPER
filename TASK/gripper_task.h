/**
  ******************************************************************************
  * @file    gripper_task.h
  * @brief   夹爪任务接口
  ******************************************************************************
  */

#ifndef GRIPPER_TASK_H
#define GRIPPER_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "remote_control.h"

/* CubeMX 在 freertos.c 中生成了同名弱函数，本任务提供强实现。 */
void GRIPPERTask(void *argument);

/* 任务运行状态，可在 Keil Watch 中查看。 */
extern volatile uint8_t g_gripper_task_ready;

#ifdef __cplusplus
}
#endif

#endif /* GRIPPER_TASK_H */
