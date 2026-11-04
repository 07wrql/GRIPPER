/**
 * @file    gripper_sequence.h
 * @brief   夹爪气泵、光电门和舵机顺序控制接口。
 * @details
 *   本模块负责一次完整夹取循环：
 *   1. 1 档打开气泵并使夹爪张开；
 *   2. 光电门检测到物体后关闭气泵；
 *   3. 第一次进入 3 档，舵机转到 180 度；
 *   4. 回到 1 档重新打开气泵；
 *   5. 第二次进入 3 档，舵机回到 0 度。
 *
 *   2 档不参与控制。拨杆按档位变化沿触发，光电门直接读取原始电平，
 *   所有处理均为非阻塞方式，由控制层按 1 ms 周期调用。
 */

#ifndef GRIPPER_SEQUENCE_H
#define GRIPPER_SEQUENCE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "remote_control.h"

/**
 * @brief  初始化气泵和舵机，并把舵机置于 0 度。
 * @retval HAL_OK    初始化成功。
 * @retval HAL_ERROR 舵机 PWM 启动失败。
 */
HAL_StatusTypeDef GripperSequence_Init(void);

/**
 * @brief  执行一次夹取状态机更新。
 * @param  remote 最新遥控数据，可以为 NULL。
 * @note   由控制层每 1 ms 调用一次，不使用阻塞延时。
 */
void GripperSequence_Step(const Remote_Data *remote);

/**
 * @brief  进入安全状态：关闭气泵并保持舵机当前角度。
 */
void GripperSequence_FailSafe(void);

#ifdef __cplusplus
}
#endif

#endif /* GRIPPER_SEQUENCE_H */
