#ifndef GRASP_SKY_BLOCK_H
#define GRASP_SKY_BLOCK_H          
             
#include "stm32f4xx_hal.h"

typedef struct
{
  uint8_t node_id;
  float min_rad;
  float max_rad;
  float max_speed_rad_s;
  float direction;
  float kp;
  float kd;
} GraspMotorAxisConfig_t;

typedef struct
{
  const GraspMotorAxisConfig_t *motors;

  float control_dt_s;             /* 周期控制积分步长，单位：s */
  uint32_t online_timeout_ms;     /* 运行阶段节点在线判定允许的反馈间隔 */
  uint32_t status_period_ms;      /* 运行阶段状态查询周期 */
  uint8_t send_fail_limit;        /* 连续发送失败达到此值后失能 */
} GraspMotorGroupConfig_t;

/**
 * @brief  夹爪运行状态。
 * @note   控制逻辑和Keil Watch共用这一份状态。
 */
typedef struct
{
  uint8_t pump_ready;              /* 1：继电器输出已初始化 */
  uint8_t servo_ready;                               /* 1：舵机PWM已启动 */
  uint8_t pump_on;                 /* 1：气泵打开；0：气泵关闭 */
  uint8_t photo_level;             /* 光电门检测状态：0=无物体，1=检测到物体 */

  uint8_t auto_start_flag;         /* 自动流程启动标志，写1启动一轮 */
  uint8_t auto_release_flag;       /* 放料许可标志，写1允许从起点下降到抓取位置 */
} GraspSkyBlockRuntime_t;

/* 运行状态，可在Keil Watch中直接查看和置位。 */
extern volatile GraspSkyBlockRuntime_t g_grasp_sky_block_runtime;

/**
 * @brief  初始化气泵和舵机子系统。
 * @retval HAL_OK    气泵和舵机均初始化成功。
 * @retval HAL_ERROR 至少一个子系统初始化失败。
 */
HAL_StatusTypeDef GraspSkyBlock_Init(void);

/**
 * @brief  阻塞初始化J60电机组。
 * @param  hcan 当前工程使用的CAN句柄，通常为&hcan1。
 * @retval HAL_OK    电机组初始化完成并可进入自动控制。
 * @retval HAL_ERROR 参数、CAN或反馈确认失败，电机保持失能。
 */
HAL_StatusTypeDef GraspSkyBlock_MotorInit(CAN_HandleTypeDef *hcan);

/**
 * @brief  执行一轮自动流程。
 * @note   函数内部按顺序完成扫描抓取、离地翻转回位、等待放料许可、
 *         原位置放料和第二次回位。控制周期约1 ms，2号按标定关系跟随大臂。
 */
void GraspSkyBlock_AutoRun(void);

#endif /* GRASP_SKY_BLOCK_H */
