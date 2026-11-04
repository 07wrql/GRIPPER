/**
 * @file    gripper_task.h
 * @brief   夹爪任务接口。
 */

#ifndef GRIPPER_TASK_H
#define GRIPPER_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* CubeMX 在 freertos.c 中生成了同名弱函数，本任务提供强实现。 */
void GRIPPERTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* GRIPPER_TASK_H */
