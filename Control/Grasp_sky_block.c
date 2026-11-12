/**
 * @file    Grasp_sky_block.c
 * @brief   夹爪统一控制实现。
 * @details
 *   任务层依次调用GraspSkyBlock_Init()和GraspSkyBlock_MotorInit()，
 *   初始化结束后再周期调用GraspSkyBlock_Run()。
 *   流程编排：
 *   1. 气泵、光电门和舵机控制；
 *   2. J60电机组的阻塞初始化、反馈判断和周期控制。
 *
 *   j60_driver提供总线初始化、通用电机组初始化、协议和反馈查询；
 *   本文件负责夹爪业务、节点选择、控制参数和周期控制。
 *
 *   气泵与舵机使用独立就绪标志，任一子系统失败不影响另一子系统：
 *   - 气泵只依赖继电器和光电门接口；
 *   - 舵机只依赖PWM输出；
 *   - 初始化只执行一次，失败后不自动重试；
 *   - J60状态完全不影响气泵和舵机判断。
 */

#include "Grasp_sky_block.h"
#include "j60_driver.h"
#include "photogate_driver.h"
#include "relay_driver.h"
#include "servo_driver.h"

/* ======================== 通用配置 ======================== */

#define GRIPPER_STICK_DEADBAND             0.08f

/* ======================== 电机组配置定义 ======================== */

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

/*
 * J60电机组配置。
 * commands[0] -> 大臂，commands[1] -> 传送轮。
 */
static const GraspMotorAxisConfig_t s_gripper_motor_config[] =
{
  {
    .node_id = 8U,
    .min_rad = -1.625f,
    .max_rad = 0.270f,
    .max_speed_rad_s = 0.80f,
    .direction = -1.0f,
    .kp = 50.0f,
    .kd = 1.2f
  },
  {
    .node_id = 2U,
    .min_rad = -2.50f,
    .max_rad = -0.2925f,
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
 * 舵机角度定义，单位为度。修改后需要重新编译才能生效。
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
/* 运行时可调的位置刚度和阻尼，默认值来自静态电机配置。 */
static float s_motor_group_kp[GRIPPER_MOTOR_COUNT];
static float s_motor_group_kd[GRIPPER_MOTOR_COUNT];
static uint8_t s_motor_group_ready;
static uint8_t s_motor_group_initialized;
static uint8_t s_motor_group_send_fail_count;
static uint8_t s_motor_group_status_axis;
static uint32_t s_motor_group_last_status_ms;

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
 * @brief  执行一次 J60 电机组周期控制。
 * @param  commands 各电机归一化控制量，NULL 时按零指令处理。
 * @param  now      当前系统 tick，单位：ms。
 * @return 1：本周期正常；0：需要进入失能状态。
 * @note   目标角度按方向、速度和 control_dt_s 积分；返回 0 后由调用者执行失能。
 */
static uint8_t GraspSkyBlock_MotorGroupControlStep(const float *commands,
                                                   uint32_t now)
{
  const GraspMotorAxisConfig_t *axis;
  float command;
  uint8_t all_ok = 1U;
  uint8_t i;

  /* 每个控制周期都检查最新反馈，防止节点掉线或报错后继续下发控制。 */
  if (GraspSkyBlock_MotorGroupRuntimeStatusReady() == 0U)
  {
    return 0U;
  }

  for (i = 0U; i < GRIPPER_MOTOR_COUNT; ++i)
  {
    axis = &s_motor_group_config.motors[i];

    /* commands 为空表示本周期无控制输入，此时按零速度保持目标角度。 */
    command = (commands != NULL) ? commands[i] : 0.0f;

    /* 限制上层输入，避免异常的归一化指令导致目标角度跳变。 */
    if (command > 1.0f)
    {
      command = 1.0f;
    }
    else if (command < -1.0f)
    {
      command = -1.0f;
    }

    /* 根据方向、最大速度和固定步长积分目标角度。 */
    s_motor_group_target_rad[i] +=
        axis->direction * command *
        axis->max_speed_rad_s *
        s_motor_group_config.control_dt_s;

    /* 目标角度不得超过配置的机械范围。 */
    if (s_motor_group_target_rad[i] < axis->min_rad)
    {
      s_motor_group_target_rad[i] = axis->min_rad;
    }
    else if (s_motor_group_target_rad[i] > axis->max_rad)
    {
      s_motor_group_target_rad[i] = axis->max_rad;
    }

    /* 下发位置目标和 PD 参数；单个节点失败时继续尝试其余节点。 */
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
 * @param  commands 各电机归一化控制量，NULL 时按零指令处理。
 * @note   仅在初始化成功后运行；控制故障时立即零输出并失能。
 */
static void GraspSkyBlock_MotorGroupRun(const float *commands)
{
  uint32_t now;

  if ((s_motor_group_initialized == 0U) ||
      (s_motor_group_ready == 0U))
  {
    return;
  }

  now = HAL_GetTick();
  if (GraspSkyBlock_MotorGroupControlStep(commands, now) == 0U)
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
 * @brief  翻转舵机标志，并执行一次对应的HOME/翻转角度动作。
 */
static void GraspSkyBlock_ToggleServo(void)
{
  g_grasp_sky_block_runtime.servo_flipped =
      (g_grasp_sky_block_runtime.servo_flipped == 0U) ? 1U : 0U;

  GraspSkyBlock_SetServo(
      (g_grasp_sky_block_runtime.servo_flipped != 0U)
          ? GRIPPER_SERVO_FLIP_ANGLE_DEG
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

  /* 0表示位于0°。 */
  g_grasp_sky_block_runtime.servo_flipped = 0U;
  g_grasp_sky_block_runtime.servo_ready = 1U;
  return HAL_OK;
}

/* ======================== 遥控输入与业务步骤 ======================== */

/**
 * @brief  检测左拨杆是否刚刚进入新档位。
 * @param  remote 最新遥控数据。
 * @return 0表示没有档位变化；1/2/3表示刚进入对应档位。
 * @note   不做软件消抖。首次取得有效数据时只记录当前档位，不执行动作。
 */
static uint8_t GraspSkyBlock_UpdateSwitch(const Remote_Data *remote)
{
  uint8_t position = remote->s[REMOTE_SW_LEFT_S1];

  if ((position < 1U) || (position > 3U))
  {
    return 0U;
  }

  if (g_grasp_sky_block_runtime.switch_valid == 0U)
  {
    g_grasp_sky_block_runtime.switch_position = position;
    g_grasp_sky_block_runtime.switch_valid = 1U;
    return 0U;
  }

  if (position != g_grasp_sky_block_runtime.switch_position)
  {
    g_grasp_sky_block_runtime.switch_position = position;
    return position;
  }

  return 0U;
}

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
 * @brief  执行一次气泵、光电门和舵机控制更新。
 * @param  remote 最新遥控数据，可以为NULL。
 */
static void GraspSkyBlock_Step(const Remote_Data *remote)
{
  uint8_t switch_edge;
  uint8_t switch_position;

  /* 遥控无效时不执行拨杆动作，保持当前输出状态。 */
  if ((remote == NULL) || (remote->online == 0U))
  {
    return;
  }

  /* 获取左拨杆刚切换的状态。 */
  switch_edge = GraspSkyBlock_UpdateSwitch(remote);
  /* 获取左拨杆当前状态。 */
  switch_position = g_grasp_sky_block_runtime.switch_position;

  /* 进入2档实现舵机0和180度两种状态的翻转，以3档作为空挡。 */
  if ((switch_edge == 2U) &&
      (g_grasp_sky_block_runtime.servo_ready != 0U))
  {
    GraspSkyBlock_ToggleServo();
  }

  /* 如果播放1档且此时夹爪闭合，那夹爪就会打开。 */
  if (g_grasp_sky_block_runtime.pump_ready != 0U)
  {
    if ((switch_position == 1U) &&
        (g_grasp_sky_block_runtime.pump_on == 0U))
    {
      GraspSkyBlock_SetPumpOn(1U);
    }

    /* 每个气泵控制周期刷新一次光电门检测状态，供3档关泵判断使用。 */
    GraspSkyBlock_UpdatePhoto();

    /*
     * 仅当拨杆处于3档、气泵正在工作且光电门检测到物体时关闭气泵。
     * 该动作只改变气泵输出，不改变舵机当前角度。
     */
    if ((switch_position == 3U) &&
        (g_grasp_sky_block_runtime.pump_on != 0U) &&
        (g_grasp_sky_block_runtime.photo_level != 0U))
    {
      GraspSkyBlock_SetPumpOn(0U);
    }
  }
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

  /* 初始化完成后从当前拨杆状态重新同步，不复现上电前的档位动作。 */
  g_grasp_sky_block_runtime.switch_valid = 0U;

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
  /* 顺序阻塞完成初始化；条件满足提前结束，任一阶段超时则失能返回。 */
  GraspSkyBlock_MotorGroupInit(hcan, &s_gripper_motor_group_config);
  return (s_motor_group_ready != 0U) ? HAL_OK : HAL_ERROR;
}

/**
 * @brief  执行一次夹爪周期任务。
 * @param  remote 最新遥控数据，可以为 NULL。
 * @param  hcan  CAN 句柄，当前接口保留、未使用。
 */
void GraspSkyBlock_Run(const Remote_Data *remote, CAN_HandleTypeDef *hcan)
{
  float commands[GRIPPER_MOTOR_COUNT] = {0.0f, 0.0f};

  (void)hcan;

  /* 初始化已完成，本函数只执行气泵、光电门、舵机和电机周期控制。 */
  GraspSkyBlock_Step(remote);

  if ((remote != NULL) && (remote->online != 0U))
  {
    /* commands是归一化控制量，范围是闭区间[-1.0, 1.0]。 */
    commands[0] = Remote_NormalizeStick(remote->ch[1],
                                        GRIPPER_STICK_DEADBAND);
    commands[1] = Remote_NormalizeStick(remote->ch[3],
                                        GRIPPER_STICK_DEADBAND);
  }

  GraspSkyBlock_MotorGroupRun(commands);
}


/**
 * @brief  设置指定电机轴的运行时Kp。
 * @param  axis 电机轴索引：0=大臂，1=传送轮。
 * @param  kp   刚度系数，范围由J60协议限制。
 * @retval HAL_OK    设置成功。
 * @retval HAL_ERROR 轴索引或参数无效。
 */
HAL_StatusTypeDef GraspSkyBlock_SetKp(uint8_t axis, float kp)
{
  if ((axis >= GRIPPER_MOTOR_COUNT) ||
      (kp != kp) ||
      (kp < J60_KP_MIN) ||
      (kp > J60_KP_MAX))
  {
    return HAL_ERROR;
  }

  s_motor_group_kp[axis] = kp;
  return HAL_OK;
}

/**
 * @brief  设置指定电机轴的运行时Kd。
 * @param  axis 电机轴索引：0=大臂，1=传送轮。
 * @param  kd   阻尼系数，范围由J60协议限制。
 * @retval HAL_OK    设置成功。
 * @retval HAL_ERROR 轴索引或参数无效。
 */
HAL_StatusTypeDef GraspSkyBlock_SetKd(uint8_t axis, float kd)
{
  if ((axis >= GRIPPER_MOTOR_COUNT) ||
      (kd != kd) ||
      (kd < J60_KD_MIN) ||
      (kd > J60_KD_MAX))
  {
    return HAL_ERROR;
  }

  s_motor_group_kd[axis] = kd;
  return HAL_OK;
}

/**
 * @brief  获取指定电机轴当前运行时Kp/Kd。
 * @param  axis 电机轴索引：0=大臂，1=传送轮。
 * @param  kp   输出当前Kp。
 * @param  kd   输出当前Kd。
 * @retval HAL_OK    读取成功。
 * @retval HAL_ERROR 参数指针或轴索引无效。
 */
HAL_StatusTypeDef GraspSkyBlock_GetTuning(uint8_t axis, float *kp, float *kd)
{
  if ((axis >= GRIPPER_MOTOR_COUNT) || (kp == NULL) || (kd == NULL))
  {
    return HAL_ERROR;
  }

  *kp = s_motor_group_kp[axis];
  *kd = s_motor_group_kd[axis];
  return HAL_OK;
}

/**
 * @brief  获取指定电机轴的目标位置和实际位置。
 * @param  axis        电机轴索引：0=大臂，1=传送轮。
 * @param  target_rad  输出目标位置，单位：rad。
 * @param  actual_rad  输出实际位置，单位：rad。
 * @retval HAL_OK    读取成功。
 * @retval HAL_ERROR 参数无效、电机组未就绪或反馈离线。
 */
HAL_StatusTypeDef GraspSkyBlock_GetTelemetry(uint8_t axis,
                                             float *target_rad,
                                             float *actual_rad)
{
  J60_MotorData snapshot;
  uint8_t node_id;

  if ((axis >= GRIPPER_MOTOR_COUNT) ||
      (target_rad == NULL) ||
      (actual_rad == NULL) ||
      (s_motor_group_initialized == 0U) ||
      (s_motor_group_ready == 0U))
  {
    return HAL_ERROR;
  }

  node_id = s_motor_group_config.motors[axis].node_id;

  if ((J60_IsOnline(node_id, s_motor_group_config.online_timeout_ms) == 0U) ||
      (J60_GetMotorSnapshot(node_id, &snapshot) != HAL_OK))
  {
    return HAL_ERROR;
  }

  *target_rad = s_motor_group_target_rad[axis];
  *actual_rad = snapshot.position_rad;
  return HAL_OK;
}