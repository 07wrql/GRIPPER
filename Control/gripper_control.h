/**
  ******************************************************************************
  * @file    gripper_control.h
  * @brief   夹爪两个 J60 电机的遥控位置控制接口
  ******************************************************************************
  * 电机分配：
  *   8 号 -> 大臂旋转
  *   4 号 -> 顶端传送轮，驱动夹爪左右移动
  *
  * 控制方式：
  *   右摇杆上下 ch[1] -> 大臂目标位置增量
  *   左摇杆上下 ch[3] -> 传送轮目标位置增量
  *   摇杆回中后目标位置不再变化，电机保持在当前位置。
  ******************************************************************************
  */

#ifndef GRIPPER_CONTROL_H
#define GRIPPER_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "remote_control.h"

/**
  * @brief  J60 位置控制增益。
  *
  * 该结构体作为全局变量保留在 RAM 中，可在 Keil 调试的 Watch
  * 窗口添加 gripper_pid，运行期间直接修改 kp/kd，无需重新烧录。
  */
typedef struct
{
  float arm_kp;                  /* 8 号大臂电机位置刚度 */
  float arm_kd;                  /* 8 号大臂电机阻尼 */
  float belt_kp;                 /* 4 号传送轮电机位置刚度 */
  float belt_kd;                 /* 4 号传送轮电机阻尼 */
} Gripper_pid;

extern volatile Gripper_pid gripper_pid;

/**
  * @brief  夹爪目标位置。
  *
  * 在 Keil Watch 窗口添加 gripper_position，可查看两个电机的目标位置。
  * 实际反馈位置直接查看 J60 接收缓存 j60_motor_data。
  */
typedef struct
{
  float arm_target_rad;          /* 8 号大臂目标位置，单位 rad */
  float belt_target_rad;         /* 4 号传送轮目标位置，单位 rad */
} GripperPosition_t;

extern volatile GripperPosition_t gripper_position;

/* 初始化和故障诊断变量，可在 Keil Watch 中直接查看。 */
extern volatile uint32_t g_gripper_init_count;
extern volatile uint32_t g_gripper_failsafe_count;
extern volatile uint16_t g_gripper_arm_status;
extern volatile uint16_t g_gripper_belt_status;
extern volatile uint32_t g_gripper_arm_rx_count;
extern volatile uint32_t g_gripper_belt_rx_count;

/**
  * @brief  初始化夹爪两个 J60 电机。
  * @param  hcan 已完成 HAL 初始化的 CAN 句柄，当前工程为 hcan1。
  *
  * 初始化内容包括：
  *   - 启动 CAN 和 J60 接收回调；
  *   - 等待驱动器上电稳定；
  *   - 清除两个电机的历史错误；
  *   - 连续两次发送使能；
  *   - 读取位置反馈并建立初始保持目标；
  *   - 以启动位置为中心设置软件行程限制。
  *
  * @note 该函数内部包含约 2 秒初始化等待，必须在 FreeRTOS 任务中调用。
  */
/* 初始化 CAN、两个 J60，获取反馈并建立初始保持位置。 */
void GripperControl_Init(CAN_HandleTypeDef *hcan);

/**
  * @brief  执行一次夹爪电机控制更新。
  * @param  remote 最新遥控数据；可以传入 NULL。
  *
  * 正常情况下由 GripperTask 每 1 ms 调用一次。
  * 函数读取右摇杆上下和左摇杆上下，更新两个目标位置，
  * 然后分别向 8 号大臂电机和 4 号传送轮电机发送位置控制帧。
  *
  * 遥控数据离线或传入 NULL 时，不更新目标位置，
  * 电机继续使用原目标位置保持。
  */
/* 以 1 ms 周期调用，遥控快照可以为 NULL 或离线状态。 */
void GripperControl_Step(const Remote_Data *remote);

/**
  * @brief  让两个 J60 进入安全停止状态。
  *
  * 函数会先发送零刚度控制帧，再向两个节点发送 Disable，
  * 最后清除 s_ready，阻止后续周期继续输出遥控控制。
  */
/* 停止两个电机并关闭控制许可。 */
void GripperControl_FailSafe(void);

/**
  * @brief  查询夹爪控制是否已完成初始化。
  * @retval 1 两个电机均已完成初始化并允许周期控制。
  * @retval 0 尚未初始化或已经进入安全停止。
  */
/* 两个电机均已完成初始化和反馈同步时返回 1。 */
uint8_t GripperControl_Ready(void);

#ifdef __cplusplus
}
#endif

#endif /* GRIPPER_CONTROL_H */




