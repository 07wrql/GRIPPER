/**
 * @file    gripper_control.h
 * @brief   夹爪总控制接口。
 * @details
 *   本模块是任务层唯一入口，内部同时维护两个互不阻塞的控制流程：
 *   1. 气泵、光电门和舵机控制；
 *   2. J60初始化、失败重试和周期控制。
 *
 *   气泵与舵机分别初始化、分别就绪、分别重试。J60的成功或失败
 *   不会影响气泵和舵机。只有遥控掉线会同时触发所有子系统的安全状态。
 *
 *   电机分配：
 *     8号J60 -> 大臂旋转
 *     2号J60 -> 顶端传送轮
 *
 *   摇杆分配：
 *     右摇杆ch[1] -> 大臂目标位置
 *     左摇杆ch[3] -> 传送轮目标位置
 */

#ifndef GRIPPER_CONTROL_H
#define GRIPPER_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "remote_control.h"

/**
 * @brief J60位置控制增益。
 * @note  变量保留在RAM中，可在Keil Watch中直接修改。
 */
typedef struct
{
  float arm_kp;                  /* 8号大臂电机位置刚度 */
  float arm_kd;                  /* 8号大臂电机阻尼 */
  float belt_kp;                 /* 2号传送轮电机位置刚度 */
  float belt_kd;                 /* 2号传送轮电机阻尼 */
} Gripper_pid;

/**
 * @brief 夹爪两个J60的目标位置。
 */
typedef struct
{
  float arm_target_rad;          /* 大臂目标位置，单位：rad */
  float belt_target_rad;         /* 传送轮目标位置，单位：rad */
} GripperPosition_t;

extern volatile Gripper_pid gripper_pid;
extern volatile GripperPosition_t gripper_position;

/* 诊断变量，保留给Keil Watch使用。 */
extern volatile uint32_t g_gripper_init_count;
extern volatile uint32_t g_gripper_failsafe_count;
extern volatile uint16_t g_gripper_arm_status;
extern volatile uint16_t g_gripper_belt_status;
extern volatile uint32_t g_gripper_arm_rx_count;
extern volatile uint32_t g_gripper_belt_rx_count;
extern volatile uint8_t g_gripper_motor_ready;
/* 0:IDLE 1:BOOT_WAIT 2:RESET 3:RESET_WAIT 4:ENABLE 5:ENABLE_WAIT
 * 6:FEEDBACK 7:READY 8:RETRY_WAIT */
extern volatile uint8_t g_gripper_motor_state;

/**
 * @brief  夹爪总控制入口。
 * @param  remote 最新遥控数据，可以为NULL。
 * @param  hcan   当前工程使用的CAN句柄，通常为&hcan1。
 *
 * 任务层每1ms调用一次。函数内部不使用长时间阻塞延时：
 * - 夹爪流程始终执行；
 * - J60初始化失败后自动在后台重试；
 * - J60故障不会关闭气泵或禁止舵机。
 */
void GripperControl_Run(const Remote_Data *remote, CAN_HandleTypeDef *hcan);

#ifdef __cplusplus
}
#endif

#endif /* GRIPPER_CONTROL_H */