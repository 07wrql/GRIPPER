/**
 * @file    Grasp_sky_block.c
 * @brief   夹爪自动流程控制实现。
 * @details
*/

#include "Grasp_sky_block.h"
#include "j60_driver.h"
#include "photogate_driver.h"
#include "relay_driver.h"
#include "servo_driver.h"
#include "cmsis_os2.h"

/* ======================== 自动流程配置 ======================== */

/* 大臂扫描和回位速度，单位：rad/s。 */
#define AUTO_SCAN_SPEED_RAD_S              0.50f
#define AUTO_RETURN_SPEED_RAD_S            0.50f


/* 夹住物块后，先抬离地面再开始翻转。该角度需按实机间隙标定。 */
#define AUTO_GROUND_CLEARANCE_RAD          0.15f
/* 第二次从起点下降到保存抓取位置的速度。 */
#define AUTO_PLACE_SPEED_RAD_S             0.50f

/* 大臂返回到起点的判定误差，单位：rad。 */
#define AUTO_RETURN_TOLERANCE_RAD          0.005f


/* 上电机械限位处的大臂反馈角度，单位：rad。实测有偏差时修改此值。 */
#define AUTO_ARM_HOME_RAD                  0.250f

/* 机械限位下，大臂与顶部夹爪组件保持90度时2号电机的反馈角度。 */
#define AUTO_BELT_HOME_RAD                 (-0.729f)

/*
 * 2号电机目标角相对大臂目标角的变化斜率：
 * theta2 = AUTO_BELT_HOME_RAD +
 *          AUTO_BELT_FOLLOW_SLOPE * (theta8 - AUTO_ARM_HOME_RAD)
 */
#define AUTO_BELT_FOLLOW_SLOPE             (-0.86f)

/* 上电低速对位参数。 */
#define AUTO_ALIGN_SPEED_RAD_S             1.0f        //对准速度
#define AUTO_ALIGN_POSITION_TOL_RAD        0.030f      //对准误差
#define AUTO_ALIGN_TIMEOUT_MS              3000U

/* 各执行动作的等待时间，单位：ms。 */
#define AUTO_DROP_WAIT_MS                  400U  /* 夹爪张开后等待物块落下 */
#define AUTO_SERVO_SETTLE_MS               700U  /* 舵机翻转或回零后等待到位 */
#define AUTO_PLACE_SETTLE_MS               200U  /* 第二次下降到抓取位置后等待稳定 */


/* ======================== 电机组配置定义 ======================== */

/*
 * J60电机组配置。
 * target_speed_rad_s[0] -> 大臂，target_speed_rad_s[1] -> 传送轮。
 */
static const GraspMotorAxisConfig_t s_gripper_motor_config[] =
{
  {
    .node_id = 8U,
    .min_rad = -2.10f,
    .max_rad = 0.260f,      //起始角度
    .max_speed_rad_s = 0.80f,
    .direction = -1.0f,
    .kp = 50.0f,
    .kd = 1.2f
  },
  {
    .node_id = 2U,
    .min_rad = -0.800f,
    .max_rad = 1.30f,
    .max_speed_rad_s = 2.00f,
    .direction = 1.0f,
    .kp = 50.0f,
    .kd = 0.8f
  }
};

#define GRIPPER_MOTOR_COUNT \
  ((uint8_t)(sizeof(s_gripper_motor_config) / \
             sizeof(s_gripper_motor_config[0])))

static const GraspMotorGroupConfig_t s_gripper_motor_group_config =
{
  .motors = s_gripper_motor_config,
  .control_dt_s = 0.001f,
  .online_timeout_ms = 200U,
  /* 每50ms轮流查询一个节点，每个节点仍约100ms查询一次。 */
  .status_period_ms = 50U,
  .send_fail_limit = 5U
};

static const J60_InitMotorsConfig_t s_gripper_motor_init_config =
{
  .reset_error = 1U,
  .online_timeout_ms = 200U,
  .enable_ack_timeout_ms = 30U,
  .feedback_timeout_ms = 300U,
  .status_request_period_ms = 20U
};

/*
 * 舵机角度定义，单位为度。
 */
#define GRIPPER_SERVO_HOME_ANGLE_DEG  (0)
#define GRIPPER_SERVO_FLIP_ANGLE_DEG  (180)

/*
 * 气泵逻辑状态到继电器状态的映射。
 * 当前接线：继电器吸合时气阀断电、夹爪夹紧；
 *           继电器释放时气阀得电、夹爪张开。
 */
#define GRIPPER_PUMP_OPEN_RELAY_STATE   (0U)
#define GRIPPER_PUMP_CLOSE_RELAY_STATE  (1U)

/* ======================== 全局运行状态 ======================== */

volatile GraspSkyBlockRuntime_t g_grasp_sky_block_runtime;

/* ======================== 电机组配置与内部状态 ======================== */

static CAN_HandleTypeDef *s_motor_group_hcan;
static GraspMotorGroupConfig_t s_motor_group_config;
/* 各电机控制积分目标，与静态配置分开保存。 */
static float s_motor_group_target_rad[GRIPPER_MOTOR_COUNT];
/* 本轮抓取位置，用于第二次下降放料。 */
static float s_grasp_arm_target_rad;
static uint8_t s_grasp_position_valid;
/* 运行时可调的位置刚度和阻尼，默认值来自静态电机配置。 */
static float s_motor_group_kp[GRIPPER_MOTOR_COUNT];
static float s_motor_group_kd[GRIPPER_MOTOR_COUNT];
static uint8_t s_motor_group_ready;
static uint8_t s_motor_group_initialized;
static uint8_t s_motor_group_send_fail_count;
static uint8_t s_motor_group_status_axis;
static uint32_t s_motor_group_last_status_ms;
/* 1：运行时强制2号目标角服从大臂-顶部夹爪组件的90度联动关系。 */
static uint8_t s_motor_group_belt_coupling_enabled;

/* ======================== 电机组内部辅助函数 ======================== */

/**
 * @brief  校验电机节点的关键配置参数。
 * @param  config 电机组配置。
 * @return 1：配置有效；0：存在非法参数。
 */
static uint8_t GraspSkyBlock_MotorGroupConfigValid(
    const GraspMotorGroupConfig_t *config)
{
  uint8_t i;

  /* 配置来自文件内静态常量，只检查最容易误写且影响电机安全的参数。 */
  for (i = 0U; i < GRIPPER_MOTOR_COUNT; ++i)
  {
    if ((config->motors[i].node_id > J60_NODE_MAX) ||
        (config->motors[i].min_rad >= config->motors[i].max_rad) ||
        (config->motors[i].max_speed_rad_s <= 0.0f) ||
        (config->motors[i].direction == 0.0f))
    {
      return 0U;
    }
  }

  return 1U;
}

/**
 * @brief  向全部电机发送零控制量。
 * @return 1：全部发送成功；0：至少一个节点发送失败。
 */
static uint8_t GraspSkyBlock_MotorGroupSendZeroControl(void)
{
  uint8_t all_ok = 1U;
  uint8_t i;

  for (i = 0U; i < GRIPPER_MOTOR_COUNT; ++i)
  {
    if (J60_SendControl(s_motor_group_config.motors[i].node_id,
                        0.0f, 0.0f, 0.0f, 0.0f, 0.0f) != HAL_OK)
    {
      all_ok = 0U;
    }
  }

  return all_ok;
}

/**
 * @brief  失能全部电机。
 * @return 1：全部失能成功；0：至少一个节点失能失败。
 */
static uint8_t GraspSkyBlock_MotorGroupDisableAll(void)
{
  uint8_t all_ok = 1U;
  uint8_t i;

  for (i = 0U; i < GRIPPER_MOTOR_COUNT; ++i)
  {
    if (J60_Disable(s_motor_group_config.motors[i].node_id) != HAL_OK)
    {
      all_ok = 0U;
    }
  }

  return all_ok;
}

/**
 * @brief  请求指定电机轴的状态反馈。
 * @param  axis 电机轴索引。
 * @return 1：状态请求发送成功；0：索引无效或发送失败。
 */
static uint8_t GraspSkyBlock_MotorGroupRequestStatus(uint8_t axis)
{
  if (axis >= GRIPPER_MOTOR_COUNT)
  {
    return 0U;
  }

  return (J60_GetStatus(s_motor_group_config.motors[axis].node_id) == HAL_OK)
             ? 1U
             : 0U;
}

/**
 * @brief  先发送零控制量，再失能全部电机，忽略底层返回值。
 */
static void GraspSkyBlock_MotorGroupStopOutputs(void)
{
  (void)GraspSkyBlock_MotorGroupSendZeroControl();
  (void)GraspSkyBlock_MotorGroupDisableAll();
}

/**
 * @brief  执行电机组失效保护并清除就绪状态。
 */
static void GraspSkyBlock_MotorGroupFailSafe(void)
{
  GraspSkyBlock_MotorGroupStopOutputs();
  s_motor_group_ready = 0U;
  s_motor_group_send_fail_count = 0U;
  s_motor_group_belt_coupling_enabled = 0U;
}

/**
 * @brief  检查全部电机是否有有效快照、在线且无故障。
 * @return 1：全部正常；0：任一电机异常。
 */
static uint8_t GraspSkyBlock_MotorGroupRuntimeStatusReady(void)
{
  J60_MotorData data;
  uint8_t i;

  for (i = 0U; i < GRIPPER_MOTOR_COUNT; ++i)
  {
    if (J60_GetMotorSnapshot(s_motor_group_config.motors[i].node_id,
                             &data) != HAL_OK)
    {
      return 0U;
    }

    if ((data.status != 0U) ||
        (J60_IsOnline(s_motor_group_config.motors[i].node_id,
                      s_motor_group_config.online_timeout_ms) == 0U))
    {
      return 0U;
    }
  }

  return 1U;
}

/**
 * @brief  读取电机反馈并将当前位置作为控制积分初值。
 * @return 1：全部位置有效；0：读取失败或位置超出配置范围。
 */
static uint8_t GraspSkyBlock_MotorGroupCaptureStartPosition(void)
{
  const GraspMotorAxisConfig_t *axis;
  J60_MotorData data;
  uint8_t i;

  for (i = 0U; i < GRIPPER_MOTOR_COUNT; ++i)
  {
    axis = &s_motor_group_config.motors[i];

    if (J60_GetMotorSnapshot(axis->node_id,
                             &data) != HAL_OK)
    {
      return 0U;
    }

    if ((data.position_rad < axis->min_rad) ||
        (data.position_rad > axis->max_rad))
    {
      return 0U;
    }

    s_motor_group_target_rad[i] = data.position_rad;
  }

  return 1U;
}

/* ======================== 电机组控制流程 ======================== */

/**
 * @brief  初始化夹爪使用的J60电机组。
 * @param  hcan   CAN 句柄。
 * @param  config 电机组运行配置。
 * @note   通用复位、使能、应答和反馈确认由J60_InitMotors()完成；
 *         本函数只提供节点列表、保存运行配置并捕获初始位置。
 */
static void GraspSkyBlock_MotorGroupInit(
    CAN_HandleTypeDef *hcan,
    const GraspMotorGroupConfig_t *config)
{
  uint8_t node_ids[GRIPPER_MOTOR_COUNT];
  uint8_t i;

  /* 配置非法时不启动电机组，保持未初始化和失能状态。 */
  if (GraspSkyBlock_MotorGroupConfigValid(config) == 0U)
  {
    s_motor_group_initialized = 0U;
    s_motor_group_ready = 0U;
    return;
  }

  /* 保存CAN句柄和运行配置，并清空上一轮运行留下的故障计数。 */
  s_motor_group_hcan = hcan;
  s_motor_group_config = *config;
  s_motor_group_send_fail_count = 0U;
  s_motor_group_ready = 0U;
  s_motor_group_initialized = 1U;
  s_motor_group_belt_coupling_enabled = 0U;
  s_motor_group_status_axis = 0U;
  s_motor_group_last_status_ms = HAL_GetTick();

  for (i = 0U; i < GRIPPER_MOTOR_COUNT; ++i)
  {
    node_ids[i] = s_motor_group_config.motors[i].node_id;
    s_motor_group_target_rad[i] = s_motor_group_config.motors[i].min_rad;
    s_motor_group_kp[i] = s_motor_group_config.motors[i].kp;
    s_motor_group_kd[i] = s_motor_group_config.motors[i].kd;
  }

  /* 先初始化CAN和J60协议层，再对当前夹爪节点执行通用电机初始化。 */
  if ((s_motor_group_hcan == NULL) ||
      (J60_BusInit(s_motor_group_hcan) != HAL_OK) ||
      (J60_InitMotors(node_ids,
                      GRIPPER_MOTOR_COUNT,
                      &s_gripper_motor_init_config) != HAL_OK))
  {
    GraspSkyBlock_MotorGroupFailSafe();
    return;
  }

  /* 通用驱动只确认反馈正常，位置范围由夹爪控制层检查。 */
  if (GraspSkyBlock_MotorGroupCaptureStartPosition() == 0U)
  {
    GraspSkyBlock_MotorGroupFailSafe();
    return;
  }

  /* 所有条件满足，记录状态查询时间并进入可控制状态。 */
  s_motor_group_last_status_ms = HAL_GetTick();
  s_motor_group_ready = 1U;
}

/**
 * @brief  将2号电机目标角强制约束到标定后的90度联动关系。
 * @return 1：目标在2号允许范围内；0：联动目标超限。
 */
static uint8_t GraspSkyBlock_ApplyBeltPositionCoupling(void)
{
  const GraspMotorAxisConfig_t *belt_axis =
      &s_motor_group_config.motors[1];
  float belt_target_rad =
      AUTO_BELT_HOME_RAD +
      AUTO_BELT_FOLLOW_SLOPE *
          (s_motor_group_target_rad[0] - AUTO_ARM_HOME_RAD);

  if ((belt_target_rad < belt_axis->min_rad) ||
      (belt_target_rad > belt_axis->max_rad))
  {
    return 0U;
  }

  s_motor_group_target_rad[1] = belt_target_rad;
  return 1U;
}

/**
 * @brief  电机组单周期控制核心。
 * @details 先更新并限幅各轴目标，再施加2号跟随大臂的位置约束，
 *          最后统一发送两台电机的位置指令，避免2号被独立限幅后
 *          大臂继续运动导致90度关系丢失。
 */
static uint8_t GraspSkyBlock_MotorGroupControlStep(
    const float *target_speed_rad_s,
    uint32_t now)
{
  const GraspMotorAxisConfig_t *axis;
  float speed_rad_s;
  uint8_t all_ok = 1U;
  uint8_t i;

  if (GraspSkyBlock_MotorGroupRuntimeStatusReady() == 0U)
  {
    return 0U;
  }

  /* 第一遍：更新并限幅各轴目标，暂不发送。 */
  for (i = 0U; i < GRIPPER_MOTOR_COUNT; ++i)
  {
    axis = &s_motor_group_config.motors[i];
    speed_rad_s = (target_speed_rad_s != NULL)
                      ? target_speed_rad_s[i]
                      : 0.0f;

    if (speed_rad_s > axis->max_speed_rad_s)
    {
      speed_rad_s = axis->max_speed_rad_s;
    }
    else if (speed_rad_s < -axis->max_speed_rad_s)
    {
      speed_rad_s = -axis->max_speed_rad_s;
    }

    s_motor_group_target_rad[i] +=
        speed_rad_s * s_motor_group_config.control_dt_s;

    if (s_motor_group_target_rad[i] < axis->min_rad)
    {
      s_motor_group_target_rad[i] = axis->min_rad;
    }
    else if (s_motor_group_target_rad[i] > axis->max_rad)
    {
      s_motor_group_target_rad[i] = axis->max_rad;
    }
  }

  /*
   * 运行时以8号大臂为主动轴，2号位置由大臂目标角直接计算。
   * 若联动目标超出2号允许范围，返回故障，不能只裁剪2号后继续运动。
   */
  if ((s_motor_group_belt_coupling_enabled != 0U) &&
      (GraspSkyBlock_ApplyBeltPositionCoupling() == 0U))
  {
    return 0U;
  }

  /* 第二遍：两台电机目标均确定后统一发送。 */
  for (i = 0U; i < GRIPPER_MOTOR_COUNT; ++i)
  {
    axis = &s_motor_group_config.motors[i];

    if (J60_SendControl(axis->node_id,
                        s_motor_group_target_rad[i],
                        0.0f,
                        0.0f,
                        s_motor_group_kp[i],
                        s_motor_group_kd[i]) != HAL_OK)
    {
      all_ok = 0U;
    }
  }

  /* 仅连续失败达到上限才要求失能，偶发发送失败可通过后续成功自动清零。 */
  if (all_ok == 0U)
  {
    ++s_motor_group_send_fail_count;
    if (s_motor_group_send_fail_count >=
        s_motor_group_config.send_fail_limit)
    {
      return 0U;
    }
  }
  else
  {
    s_motor_group_send_fail_count = 0U;
  }

  /*
   * 控制帧优先发送后，再按固定周期轮流查询一个节点。单次最多发送
   * 2个控制帧和1个状态请求，不超过CAN控制器的3个发送邮箱。
   */
  if ((uint32_t)(now - s_motor_group_last_status_ms) >=
      s_motor_group_config.status_period_ms)
  {
    if (GraspSkyBlock_MotorGroupRequestStatus(
            s_motor_group_status_axis) == 0U)
    {
      return 0U;
    }

    ++s_motor_group_status_axis;
    if (s_motor_group_status_axis >= GRIPPER_MOTOR_COUNT)
    {
      s_motor_group_status_axis = 0U;
    }
    s_motor_group_last_status_ms = now;
  }

  return 1U;
}

/**
 * @brief  电机组周期控制入口。
 * @param  target_speed_rad_s 各电机目标角速度，单位：rad/s，NULL 时按零速度处理。
 * @note   仅在初始化成功后运行；控制故障时立即零输出并失能。
 */
static void GraspSkyBlock_MotorGroupRun(const float *target_speed_rad_s)
{
  uint32_t now;

  if ((s_motor_group_initialized == 0U) ||
      (s_motor_group_ready == 0U))
  {
    return;
  }

  now = HAL_GetTick();
  if (GraspSkyBlock_MotorGroupControlStep(target_speed_rad_s, now) == 0U)
  {
    GraspSkyBlock_MotorGroupFailSafe();
  }
}

/* ======================== 气泵与舵机控制 ======================== */

/**
 * @brief  设置气泵状态。
 * @param  on 1：气泵打开、释放；0：气泵关闭、夹紧。
 * @note   当前接线与继电器逻辑相反，在这里统一完成极性转换。
 */
static void GraspSkyBlock_SetPumpOn(uint8_t on)
{
  uint8_t next_on = (on != 0U) ? 1U : 0U;

  if (g_grasp_sky_block_runtime.pump_on != next_on)
  {
    Relay_Set((next_on != 0U) ? GRIPPER_PUMP_OPEN_RELAY_STATE
                              : GRIPPER_PUMP_CLOSE_RELAY_STATE);
    g_grasp_sky_block_runtime.pump_on = next_on;
  }
}

/**
 * @brief  在舵机就绪时设置目标角度并更新PWM。
 * @param  angle_deg 目标角度，单位度。
 */
static void GraspSkyBlock_SetServo(int32_t angle_deg)
{
  if (g_grasp_sky_block_runtime.servo_ready != 0U)
  {
    Servo_SetAngle(angle_deg);
  }
}

/**
 * @brief  设置舵机逻辑状态并输出对应角度。
 * @param  flipped 1：翻转角度；0：HOME角度。
 */
static void GraspSkyBlock_SetServoState(uint8_t flipped)
{
  GraspSkyBlock_SetServo(
      (flipped != 0U) ? GRIPPER_SERVO_FLIP_ANGLE_DEG
                      : GRIPPER_SERVO_HOME_ANGLE_DEG);
}

/**
 * @brief  独立初始化气泵子系统。
 * @retval HAL_OK    继电器输出初始化成功。
 * @retval HAL_ERROR 继电器输出状态异常。
 */
static HAL_StatusTypeDef GraspSkyBlock_InitPump(void)
{
  /*
   * 初始化继电器为低电，继电器常闭，电磁阀得电，夹爪张开。
   */
  Relay_Init();

  /* 夹爪张开标志位。 */
  g_grasp_sky_block_runtime.pump_on = 1U;
  /* 气泵准备好标志位。 */
  g_grasp_sky_block_runtime.pump_ready = 1U;

  return HAL_OK;
}

/**
 * @brief  独立初始化舵机子系统。
 * @retval HAL_OK    PWM启动成功且HOME角度已输出。
 * @retval HAL_ERROR PWM启动失败或通道未进入运行状态。
 */
static HAL_StatusTypeDef GraspSkyBlock_InitServo(void)
{
  /*
   * 先设置HOME角度，再启动PWM，避免启动瞬间输出错误脉宽。
   */
  Servo_SetAngle(GRIPPER_SERVO_HOME_ANGLE_DEG);

  if (Servo_Init() != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* 通道启动成功，PWM已输出HOME角度。 */
  g_grasp_sky_block_runtime.servo_ready = 1U;
  return HAL_OK;
}

/* ======================== 自动流程辅助函数 ======================== */

/**
 * @brief  读取并转换光电门检测状态。
 * @note   运行状态中的photo_level是逻辑量：0=无物体，1=检测到物体。
 */
static void GraspSkyBlock_UpdatePhoto(void)
{
  g_grasp_sky_block_runtime.photo_level =
      (PhotoGate_ReadRawLevel() == PHOTO_GATE_OBJECT_LEVEL) ? 1U : 0U;
}

/**
 * @brief  每个控制周期驱动大臂，并由控制核心实时计算2号目标角。
 * @param  arm_speed_rad_s 大臂目标角速度，单位：rad/s。
 * @note   2号不再使用独立速度积分，其位置由大臂目标角按90度
 *         联动关系强制约束。
 */
static void GraspSkyBlock_AutoService(float arm_speed_rad_s)
{
  float target_speed_rad_s[GRIPPER_MOTOR_COUNT] = {arm_speed_rad_s, 0.0f};

  if ((s_motor_group_initialized == 0U) ||
      (s_motor_group_ready == 0U))
  {
    return;
  }

  GraspSkyBlock_MotorGroupRun(target_speed_rad_s);
}

/**
 * @brief  自动流程中的带服务延时。
 * @param  delay_ms 延时时间，单位：ms。
 * @return 1：延时完成且电机正常；0：电机组已失效。
 */
static uint8_t GraspSkyBlock_AutoDelayMs(uint32_t delay_ms)
{
  uint32_t start_ms = HAL_GetTick();

  while ((uint32_t)(HAL_GetTick() - start_ms) < delay_ms)
  {
    if (s_motor_group_ready == 0U)
    {
      return 0U;
    }

    GraspSkyBlock_AutoService(0.0f);
    osDelay(1U);
  }

  return (s_motor_group_ready != 0U) ? 1U : 0U;
}

/**
 * @brief  以给定速度将大臂平滑移动到指定目标角，并在到位后保持。
 */
static uint8_t GraspSkyBlock_AutoMoveArmTo(float target_rad,
                                           float speed_rad_s,
                                           uint32_t settle_ms)
{
  const GraspMotorAxisConfig_t *arm_axis = &s_gripper_motor_config[0];
  float current_rad;
  float error_rad;

  /* 速度必须是正数，目标角必须位于大臂软件限位内。 */
  if ((speed_rad_s <= 0.0f) ||
      (target_rad < arm_axis->min_rad) ||
      (target_rad > arm_axis->max_rad))
  {
    return 0U;
  }

  /*
   * 每一周期重新读取软件目标角，并判断是否已进入允许误差范围。
   * AutoService负责推进目标角、发送控制帧并维持2号电机联动。
   */
  while (s_motor_group_ready != 0U)
  {
    current_rad = s_motor_group_target_rad[0];
    error_rad = target_rad - current_rad;

    /* 统一转换为绝对误差，只用于判断距离目标还有多远。 */
    if (error_rad < 0.0f)
    {
      error_rad = -error_rad;
    }
    if (error_rad <= AUTO_RETURN_TOLERANCE_RAD)
    {
      break;
    }

    /* speed_rad_s只表示速度大小，正负方向由目标位置决定。 */
    if (target_rad > current_rad)
    {
      GraspSkyBlock_AutoService(speed_rad_s);
    }
    else
    {
      GraspSkyBlock_AutoService(-speed_rad_s);
    }
    osDelay(1U);
  }

  /* 移动途中电机组失效时，不能把本次移动报告为成功。 */
  if (s_motor_group_ready == 0U)
  {
    return 0U;
  }

  /* 消除容差内的小偏差，并发送零速度位置保持指令。 */
  s_motor_group_target_rad[0] = target_rad;
  GraspSkyBlock_AutoService(0.0f);

  /* 不需要稳定等待时直接返回。 */
  if (settle_ms == 0U)
  {
    return 1U;
  }

  return GraspSkyBlock_AutoDelayMs(settle_ms);
}

/**
 * @brief  检查自动流程所需的子系统是否全部就绪。
 * @return 1：可以运行自动流程；0：存在未就绪子系统。
 */
static uint8_t GraspSkyBlock_AutoReady(void)
{
  return ((s_motor_group_initialized != 0U) &&
          (s_motor_group_ready != 0U) &&
          (g_grasp_sky_block_runtime.pump_ready != 0U) &&
          (g_grasp_sky_block_runtime.servo_ready != 0U)) ? 1U : 0U;
}

/**
 * @brief  大臂向负方向扫描，直到检测到物块或到达扫描终点。
 * @return 1：检测到物块；0：到达限位、电机失效或未检测到物块。
 * @note   扫描期间2号目标角由大臂目标角实时计算，保持90度联动。
 */
static uint8_t GraspSkyBlock_AutoScan(void)
{
	//配置大臂电机的参数
  const GraspMotorAxisConfig_t *arm_axis = &s_gripper_motor_config[0];
  float scan_stop_rad;
  float scan_speed_rad_s;
  float current_target_rad;

  scan_stop_rad = arm_axis->min_rad;
	//扫描速度 = 方向符号 × 速度大小
  scan_speed_rad_s = arm_axis->direction * AUTO_SCAN_SPEED_RAD_S;

	//电机已经准备就绪
  while (s_motor_group_ready != 0U)
  {
		//参数决定电机扫描速度，控制电机向下扫描函数
    GraspSkyBlock_AutoService(scan_speed_rad_s);
		//更新 photo_level
    GraspSkyBlock_UpdatePhoto();

    /*  判断是否检测到物块
        → 返回 1，结束扫描并进入夹取流程。*/
    if (g_grasp_sky_block_runtime.photo_level != 0U)
    {
      s_grasp_arm_target_rad = s_motor_group_target_rad[0];
      s_grasp_position_valid = 1U;
      return 1U;
    }

    /* 到达软件限位前停止，避免目标长期顶在机械限位上。 */
    current_target_rad = s_motor_group_target_rad[0];
    if (current_target_rad <= scan_stop_rad)
    {
      break;
    }

    osDelay(1U);
  }

  return 0U;
}

/**
 * @brief  等待舵机动作剩余时间。
 * @param  start_ms 舵机指令发出时的 HAL_GetTick() 时间。
 * @return 1：等待完成；0：电机组失效。
 */
static uint8_t GraspSkyBlock_AutoWaitServoSettle(uint32_t start_ms)
{
  uint32_t elapsed_ms = (uint32_t)(HAL_GetTick() - start_ms);

  if (s_motor_group_ready == 0U)
  {
    return 0U;
  }
  if (elapsed_ms >= AUTO_SERVO_SETTLE_MS)
  {
    return 1U;
  }

  return GraspSkyBlock_AutoDelayMs(AUTO_SERVO_SETTLE_MS - elapsed_ms);
}

/**
 * @brief  将大臂目标移动到起点并保持稳定。
 * @return 1：回位完成；0：电机组失效。
 */
static uint8_t GraspSkyBlock_AutoReturnHome(void)
{
  return GraspSkyBlock_AutoMoveArmTo(AUTO_ARM_HOME_RAD,
                                     AUTO_RETURN_SPEED_RAD_S,
                                     0U);
}

/**
 * @brief  检测到物块后立即夹紧，并记录当前抓取位置。
 * @return 1：夹紧命令已发出。
 * @note   此处不翻转舵机。必须先回程抬离地面，避免物块与地面摩擦。
 */
static uint8_t GraspSkyBlock_AutoClampAndHold(void)
{
  J60_MotorData arm_data;
  const GraspMotorAxisConfig_t *arm_axis = &s_gripper_motor_config[0];

  GraspSkyBlock_SetPumpOn(0U);

  /* 记录当前大臂实际位置，作为第二次下降的目标。 */
  if (J60_GetMotorSnapshot(arm_axis->node_id, &arm_data) == HAL_OK)
  {
    s_grasp_arm_target_rad = arm_data.position_rad;
  }

  return 1U;
}

/**
 * @brief  先抬离地面，再在回起点过程中翻转物块。
 * @return 1：翻转并回位完成；0：参数、位置或电机异常。
 */
static uint8_t GraspSkyBlock_AutoLiftAndFlipReturn(void)
{
  float clearance_target_rad;
  uint32_t servo_start_ms;

  if ((s_grasp_position_valid == 0U) ||
      (s_grasp_arm_target_rad < s_gripper_motor_config[0].min_rad) ||
      (s_grasp_arm_target_rad > s_gripper_motor_config[0].max_rad))
  {
    return 0U;
  }

  clearance_target_rad =
      s_grasp_arm_target_rad + AUTO_GROUND_CLEARANCE_RAD;
  if (clearance_target_rad > AUTO_ARM_HOME_RAD)
  {
    clearance_target_rad = AUTO_ARM_HOME_RAD;
  }

  /* 舵机保持HOME角，先抬升到物块离开地面的位置。 */
  if (GraspSkyBlock_AutoMoveArmTo(clearance_target_rad,
                                  AUTO_RETURN_SPEED_RAD_S,
                                  0U) == 0U)
  {
    return 0U;
  }

  /* 物块已经离地，允许翻转；翻转与剩余回程同时进行。 */
  GraspSkyBlock_SetServoState(1U);
  servo_start_ms = HAL_GetTick();

  if (GraspSkyBlock_AutoReturnHome() == 0U)
  {
    return 0U;
  }

  return GraspSkyBlock_AutoWaitServoSettle(servo_start_ms);
}

/**
 * @brief  机械臂回到起点后等待放料许可。
 * @return 1：收到新的放料许可；0：电机组失效。
 * @note   必须先清除旧标志，只接受本次置1。
 */
static uint8_t GraspSkyBlock_AutoWaitPlaceFlag(void)
{
  g_grasp_sky_block_runtime.auto_release_flag = 0U;

  while ((g_grasp_sky_block_runtime.auto_release_flag == 0U) &&
         (s_motor_group_ready != 0U))
  {
    GraspSkyBlock_AutoService(0.0f);
    osDelay(1U);
  }

  if (s_motor_group_ready == 0U)
  {
    return 0U;
  }

  g_grasp_sky_block_runtime.auto_release_flag = 0U;
  return 1U;
}

/**
 * @brief  从起点下降到保存的抓取位置，放料后回位并让舵机回中。
 * @return 1：放置并回位完成；0：位置无效或电机组失效。
 * @note   第二次回程中舵机保持180度；大臂回到起点并稳定后，舵机再回0度。
 */
static uint8_t GraspSkyBlock_AutoPlaceAndReturn(void)
{
  uint32_t servo_start_ms;

  if (s_grasp_position_valid == 0U)
  {
    return 0U;
  }

  if (GraspSkyBlock_AutoMoveArmTo(s_grasp_arm_target_rad,
                                  AUTO_PLACE_SPEED_RAD_S,
                                  AUTO_PLACE_SETTLE_MS) == 0U)
  {
    return 0U;
  }

  GraspSkyBlock_SetPumpOn(1U);
  if (GraspSkyBlock_AutoDelayMs(AUTO_DROP_WAIT_MS) == 0U)
  {
    return 0U;
  }

  /* 第二次回程期间舵机保持180度，等待大臂先回到起点并稳定。 */
  if (GraspSkyBlock_AutoReturnHome() == 0U)
  {
    return 0U;
  }

  /* 大臂已经回中稳定，此时才让舵机回0度。 */
  GraspSkyBlock_SetServoState(0U);
  servo_start_ms = HAL_GetTick();

  if (GraspSkyBlock_AutoWaitServoSettle(servo_start_ms) == 0U)
  {
    return 0U;
  }

  s_grasp_position_valid = 0U;
  return 1U;
}

/**
 * @brief  建立大臂与2号传送带电机的90度联动基准。
 * @return 1：对位完成且联动已启用；0：电机异常或对位超时。
 * @note   该函数不强制大臂回到0.250 rad机械基准，而是保持上电时的实际
 *         大臂角，按标定关系计算2号目标角，再缓慢推进到该位置。
 */
static uint8_t GraspSkyBlock_MotorGroupAlignBeltToArm(void)
{
  const GraspMotorAxisConfig_t *arm_axis =
      &s_motor_group_config.motors[0];
  const GraspMotorAxisConfig_t *belt_axis =
      &s_motor_group_config.motors[1];
  J60_MotorData arm_data;
  J60_MotorData belt_data;
  float belt_target_rad;
  float belt_step_rad;
  float belt_delta_rad;
  float belt_error_rad;
  uint32_t start_ms;
  uint8_t ramp_target_reached = 0U;

  if ((s_motor_group_initialized == 0U) ||
      (s_motor_group_ready == 0U))
  {
    return 0U;
  }

  /* 对位期间由本函数手动推进2号目标，禁止控制核心提前覆盖。 */
  s_motor_group_belt_coupling_enabled = 0U;

  /*
   * 上电机械限位可能落在0.250 rad附近而不是绝对等于0.250 rad。
   * 因此保持当前实际大臂角，避免强行拉回基准导致对位超时。
   */
  if (J60_GetMotorSnapshot(arm_axis->node_id, &arm_data) != HAL_OK)
  {
    GraspSkyBlock_MotorGroupFailSafe();
    return 0U;
  }

  s_motor_group_target_rad[0] = arm_data.position_rad;

  /* 传送带目标角度:-0.729+-0.86（大臂当前角度-0.25），大臂转动的角度，小臂要转0.86×这个角度 */
  belt_target_rad = AUTO_BELT_HOME_RAD +AUTO_BELT_FOLLOW_SLOPE *(arm_data.position_rad - AUTO_ARM_HOME_RAD);

  if ((belt_target_rad < belt_axis->min_rad) ||
      (belt_target_rad > belt_axis->max_rad))
  {
    GraspSkyBlock_MotorGroupFailSafe();
    return 0U;
  }
  /*每一个周期移动的角度=速度×时间，调整对应的参数可以调整上电之后夹爪组件到90°的速度*/
  belt_step_rad = AUTO_ALIGN_SPEED_RAD_S * s_motor_group_config.control_dt_s;
  start_ms = HAL_GetTick();

  while (s_motor_group_ready != 0U)
  {
    /* 第一段：平滑推进2号目标角，最终精确落在目标值。 */
    belt_delta_rad =
        belt_target_rad - s_motor_group_target_rad[1];

    if (belt_delta_rad > belt_step_rad)
    {
      s_motor_group_target_rad[1] += belt_step_rad;
    }
    else if (belt_delta_rad < -belt_step_rad)
    {
      s_motor_group_target_rad[1] -= belt_step_rad;
    }
    else
    {
      s_motor_group_target_rad[1] = belt_target_rad;
      ramp_target_reached = 1U;
    }

    GraspSkyBlock_MotorGroupRun(NULL);

    /* 控制调用可能触发失效保护，失败后不再继续判断对准。 */
    if (s_motor_group_ready == 0U)
    {
      break;
    }

    /*
     * 第二段：软件目标到达后，再读取2号实际反馈。
     * 只有实际误差进入容差，才认为机械姿态真正对准。
     */
    if ((ramp_target_reached != 0U) &&
        (J60_GetMotorSnapshot(belt_axis->node_id, &belt_data) == HAL_OK))
    {
      belt_error_rad = belt_data.position_rad - belt_target_rad;
      if (belt_error_rad < 0.0f)
      {
        belt_error_rad = -belt_error_rad;
      }

      if (belt_error_rad <= AUTO_ALIGN_POSITION_TOL_RAD)
      {
        /* 第三段：锁定精确基准，并正式启用运行时90度联动。 */
        s_motor_group_target_rad[0] = arm_data.position_rad;
        s_motor_group_target_rad[1] = belt_target_rad;
        s_motor_group_belt_coupling_enabled = 1U;

        GraspSkyBlock_MotorGroupRun(NULL);
        return (s_motor_group_ready != 0U) ? 1U : 0U;
      }
    }

    /* 第四段：超时保护。差值写法支持HAL_GetTick()计数器回绕。 */
    if ((uint32_t)(HAL_GetTick() - start_ms) >=
        AUTO_ALIGN_TIMEOUT_MS)
    {
      break;
    }

    osDelay(1U);
  }

  /* 超时或控制失效统一关断输出并清除联动标志。 */
  GraspSkyBlock_MotorGroupFailSafe();
  return 0U;
}

/* ======================== 对外接口 ======================== */

/**
 * @brief  初始化气泵和舵机子系统。
 * @retval HAL_OK    两个子系统均初始化成功。
 * @retval HAL_ERROR 至少一个子系统初始化失败。
 */
HAL_StatusTypeDef GraspSkyBlock_Init(void)
{
  HAL_StatusTypeDef pump_status;
  HAL_StatusTypeDef servo_status;

  /* 两个子系统分别初始化，一个失败不会跳过另一个。 */
  pump_status = GraspSkyBlock_InitPump();
  servo_status = GraspSkyBlock_InitServo();

  return ((pump_status == HAL_OK) && (servo_status == HAL_OK))
             ? HAL_OK
             : HAL_ERROR;
}

/**
 * @brief  初始化夹爪 J60 电机组。
 * @param  hcan CAN 句柄。
 * @return HAL_OK：电机组就绪；HAL_ERROR：配置或初始化失败。
 */
HAL_StatusTypeDef GraspSkyBlock_MotorInit(CAN_HandleTypeDef *hcan)
{
  /* 顺序阻塞完成初始化，并低速建立90度基准姿态。 */
  GraspSkyBlock_MotorGroupInit(hcan, &s_gripper_motor_group_config);
  if (s_motor_group_ready == 0U)
  {
    return HAL_ERROR;
  }

  return (GraspSkyBlock_MotorGroupAlignBeltToArm() != 0U)
             ? HAL_OK
             : HAL_ERROR;
}

/**
 * @brief  执行一轮自动抓取流程。
 * @note   主函数负责扫描抓取、离地翻转回位、原位置放料和第二次回位。
 *         所有等待都会持续服务J60，2号始终按标定关系跟随大臂目标角。
 */
void GraspSkyBlock_AutoRun(void)
{
  uint8_t object_found;

  if (GraspSkyBlock_AutoReady() == 0U)
  {
    return;
  }

  /* 在没有给信号的时候，电机保持在原位置 */
  if (g_grasp_sky_block_runtime.auto_start_flag == 0U)
  {
    GraspSkyBlock_AutoService(0.0f);
    return;
  }

  /* 本轮开始，清除上次遗留的放置许可和抓取位置。 */
  g_grasp_sky_block_runtime.auto_start_flag = 0U;
  g_grasp_sky_block_runtime.auto_release_flag = 0U;
  s_grasp_position_valid = 0U;

  /* 扫描直到检测到物块或到达扫描终点。 */
  object_found = GraspSkyBlock_AutoScan();

  if (object_found == 0U)
  {
    /* 没有物块时直接回起点，不翻转、不等待放料许可。 */
    (void)GraspSkyBlock_AutoReturnHome();
    return;
  }

  /* 先夹紧并记录位置，原地稳定后再开始抬升。 */
  if (GraspSkyBlock_AutoClampAndHold() == 0U)
  {
    return;
  }

  /* 先抬离地面，再在回程中翻转物块。 */
  if (GraspSkyBlock_AutoLiftAndFlipReturn() == 0U)
  {
    return;
  }

  /* 已在起点，等待新的放料许可。 */
  if (GraspSkyBlock_AutoWaitPlaceFlag() == 0U)
  {
    return;
  }

  /* 下降到原抓取位置放料，并在第二次回程中翻回。 */
  (void)GraspSkyBlock_AutoPlaceAndReturn();
}
