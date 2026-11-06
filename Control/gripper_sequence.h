/**
 * @file    gripper_sequence.h
 * @brief   气泵、光电门和舵机独立控制接口。
 * @details
 *   本模块内部的气泵和舵机分别初始化、分别就绪：
 *   1. 气泵初始化失败不会禁止舵机；
 *   2. 舵机初始化失败不会禁止气泵；
 *   3. 初始化只执行一次，失败后不自动重试；
 *   4. J60状态不参与本模块的任何判断。
 *
 *   拨杆控制规则：
 *   1. 3档无输出动作，只作为中位；
 *   2. 1档按电平打开气泵并清除气泵标志；
 *   3. 每次首次进入2档翻转一次舵机标志，在HOME/翻转角度之间切换；
 *   4. 光电门在3档检测到物体时关闭气泵并置位气泵标志。
 *
 *   2档采用档位变化检测，不做软件消抖。
 */

#ifndef GRIPPER_SEQUENCE_H
#define GRIPPER_SEQUENCE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "remote_control.h"


/**
 * @brief  夹爪运行状态。
 * @note   控制逻辑和Keil Watch共用这一份状态，不再维护两套镜像变量。
 */
typedef struct
{
  uint8_t pump_ready;              /* 1：继电器输出已初始化 */
  uint8_t servo_ready;             /* 1：舵机PWM已启动 */
  uint8_t pump_on;                 /* 1：气泵打开；0：气泵关闭 */
  uint8_t servo_flipped;           /* 1：舵机位于翻转角度；0：位于HOME角度 */
  uint8_t switch_position;         /* 左拨杆当前档位 */
  uint8_t switch_valid;            /* 左拨杆档位是否已取得有效值 */
  uint8_t photo_level;             /* 光电门检测状态：0=无物体，1=检测到物体 */
} GripperRuntime_t;

/* 运行状态，可在Keil Watch中直接查看。 */
extern volatile GripperRuntime_t g_gripper_runtime;

/**
 * @brief  初始化气泵和舵机的公共状态，并分别初始化两个子系统。
 * @note   气泵和舵机互不依赖。单个子系统初始化失败后保持未就绪，
 *         不会影响另一个子系统，也不会自动重试。
 */
void GripperSequence_Init(void);

/**
 * @brief  执行一次气泵、光电门和舵机控制更新。
 * @param  remote 最新遥控数据，可以为NULL。
 * @note   由控制层每1ms调用一次，不使用阻塞延时。
 *         气泵和舵机分别依据自己的就绪标志执行。
 */
void GripperSequence_Step(const Remote_Data *remote);

/**
 * @brief  进入夹爪安全状态：关闭气泵，舵机保持当前角度。
 * @note   仅由遥控掉线等夹爪自身安全条件调用，不由J60故障调用。
 */
void GripperSequence_FailSafe(void);

#ifdef __cplusplus
}
#endif

#endif /* GRIPPER_SEQUENCE_H */