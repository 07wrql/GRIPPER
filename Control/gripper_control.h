/**
 * @file    gripper_control.h
 * @brief   夹爪总控制接口。
 * @details
 *   本模块是任务层唯一需要调用的夹爪控制入口，内部完成：
 *   1. J60 大臂和传送轮的初始化、重试与周期控制；
 *   2. 遥控器掉线和 J60 故障保护；
 *   3. 气泵、光电门和舵机夹取流程的周期调度。
 *
 *   电机分配：
 *     8 号 J60 -> 大臂旋转
 *     4 号 J60 -> 顶端传送轮
 *
 *   摇杆分配：
 *     右摇杆 ch[1] -> 大臂目标位置
 *     左摇杆 ch[3] -> 传送轮目标位置
 */

#ifndef GRIPPER_CONTROL_H
#define GRIPPER_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "remote_control.h"

/**
 * @brief J60 位置控制增益。
 * @note  变量保留在 RAM 中，可在 Keil Watch 中直接修改。
 */
typedef struct
{
  float arm_kp;                  /* 8 号大臂电机位置刚度 */
  float arm_kd;                  /* 8 号大臂电机阻尼 */
  float belt_kp;                 /* 4 号传送轮电机位置刚度 */
  float belt_kd;                 /* 4 号传送轮电机阻尼 */
} Gripper_pid;

/**
 * @brief 夹爪两个 J60 的目标位置。
 */
typedef struct
{
  float arm_target_rad;          /* 大臂目标位置，单位：rad */
  float belt_target_rad;         /* 传送轮目标位置，单位：rad */
} GripperPosition_t;

extern volatile Gripper_pid gripper_pid;
extern volatile GripperPosition_t gripper_position;

/* 诊断变量，保留给 Keil Watch 使用，不提供额外调试接口。 */
extern volatile uint32_t g_gripper_init_count;
extern volatile uint32_t g_gripper_failsafe_count;
extern volatile uint16_t g_gripper_arm_status;
extern volatile uint16_t g_gripper_belt_status;
extern volatile uint32_t g_gripper_arm_rx_count;
extern volatile uint32_t g_gripper_belt_rx_count;

/**
 * @brief  夹爪总控制入口。
 * @param  remote 最新遥控数据，可以为 NULL。
 * @param  hcan   当前工程使用的 CAN 句柄，通常为 &hcan1。
 *
 * 任务层每 1 ms 调用一次即可。J60 初始化、失败重试、夹取状态机
 * 和所有安全处理均在本函数内部完成，任务层不再直接调用子模块。
 */
void GripperControl_Run(const Remote_Data *remote, CAN_HandleTypeDef *hcan);

#ifdef __cplusplus
}
#endif

#endif /* GRIPPER_CONTROL_H */
