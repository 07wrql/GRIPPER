/**
 * @file    Grasp_sky_block.h
 * @brief   夹爪统一控制接口。
 * @details
 *   夹爪统一控制模块，内部包含：
 *   1. 气泵、光电门和舵机控制；
 *   2. J60电机组的阻塞初始化、反馈判断和周期控制。
 *
 *   j60_driver提供总线初始化和通用电机组初始化；
 *   本模块负责节点选择、控制参数和周期性夹爪控制。
 *
 *   电机分配：
 *     8号J60 -> 大臂旋转
 *     2号J60 -> 顶端传送轮
 *
 *   摇杆分配：
 *     右摇杆ch[1] -> 大臂目标位置
 *     左摇杆ch[3] -> 传送轮目标位置
 *
 *   拨杆控制规则：
 *   1. 3档无输出动作，只作为中位；
 *   2. 1档按电平打开气泵并清除气泵标志；
 *   3. 每次首次进入2档翻转一次舵机标志，在HOME/翻转角度之间切换；
 *   4. 光电门在3档检测到物体时关闭气泵并置位气泵标志。
 */

#ifndef GRASP_SKY_BLOCK_H
#define GRASP_SKY_BLOCK_H

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
} GraspSkyBlockRuntime_t;

/* 运行状态，可在Keil Watch中直接查看。 */
extern volatile GraspSkyBlockRuntime_t g_grasp_sky_block_runtime;

/**
 * @brief  初始化气泵和舵机子系统。
 *
 * 在任务上电阶段调用一次。初始化完成后拨杆会从当前档位重新同步，
 * 不会补执行初始化期间的档位变化。
 * @retval HAL_OK    气泵和舵机均初始化成功。
 * @retval HAL_ERROR 至少一个子系统初始化失败。
 */
HAL_StatusTypeDef GraspSkyBlock_Init(void);

/**
 * @brief  阻塞初始化J60电机组。
 * @param  hcan 当前工程使用的CAN句柄，通常为&hcan1。
 *
 * 函数需在任务上下文调用。内部发送错误复位和使能命令，随后等待
 * 使能应答和首次控制反馈；等待均有超时上限，条件满足后立即返回。
 * 初始化失败时电机零输出并失能，不自动重试。
 * @retval HAL_OK    电机组初始化完成并可进入周期控制。
 * @retval HAL_ERROR 参数、CAN或反馈确认失败，电机保持失能。
 */
HAL_StatusTypeDef GraspSkyBlock_MotorInit(CAN_HandleTypeDef *hcan);

/**
 * @brief  夹爪周期控制入口。
 * @param  remote 最新遥控数据，可以为NULL。
 * @param  hcan   当前工程使用的CAN句柄，通常为&hcan1。
 *
 * 任务层每1ms调用一次。气泵、光电门和舵机流程始终执行；
 * 电机未初始化或初始化失败时保持零输出并失能。
 */
void GraspSkyBlock_Run(const Remote_Data *remote, CAN_HandleTypeDef *hcan);

/**
 * @brief  设置指定电机轴的运行时Kp。
 * @param  axis 电机轴索引：0=大臂，1=传送轮。
 * @param  kp   刚度系数。
 */
HAL_StatusTypeDef GraspSkyBlock_SetKp(uint8_t axis, float kp);

/**
 * @brief  设置指定电机轴的运行时Kd。
 * @param  axis 电机轴索引：0=大臂，1=传送轮。
 * @param  kd   阻尼系数。
 */
HAL_StatusTypeDef GraspSkyBlock_SetKd(uint8_t axis, float kd);

/**
 * @brief  获取指定电机轴当前运行时Kp/Kd。
 * @param  axis 电机轴索引：0=大臂，1=传送轮。
 * @param  kp   输出当前Kp。
 * @param  kd   输出当前Kd。
 * @retval HAL_OK    读取成功。
 * @retval HAL_ERROR 参数指针或轴索引无效。
 */
HAL_StatusTypeDef GraspSkyBlock_GetTuning(uint8_t axis, float *kp, float *kd);

/**
 * @brief  获取指定电机轴的目标位置和实际位置。
 * @param  axis        电机轴索引：0=大臂，1=传送轮。
 * @param  target_rad  输出目标位置，单位：rad。
 * @param  actual_rad  输出实际位置，单位：rad。
 */
HAL_StatusTypeDef GraspSkyBlock_GetTelemetry(uint8_t axis,
                                             float *target_rad,
                                             float *actual_rad);

#ifdef __cplusplus
}
#endif

#endif /* GRASP_SKY_BLOCK_H */
