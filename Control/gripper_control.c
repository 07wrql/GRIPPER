/**
 * @file    gripper_control.c
 * @brief   夹爪总控制实现。
 * @details
 *   本文件是夹爪控制层唯一入口。任务层只调用 GripperControl_Run()，
 *   由本文件统一完成 J60 初始化、失败重试、电机控制和夹取流程调度。
 */

#include "gripper_control.h"
#include "gripper_sequence.h"
#include "j60_driver.h"
#include "can.h"
#include "cmsis_os2.h"

/* ========================== 配置常量 ========================== */

/* J60 节点分配。 */
#define GRIPPER_ARM_ID                        8U
#define GRIPPER_BELT_ID                       2U

/* 右摇杆和左摇杆死区。 */
#define GRIPPER_STICK_DEADBAND                0.08f

/* 摇杆满量程时对应的目标速度。 */
#define GRIPPER_ARM_MAX_SPEED_RAD_S           0.20f
#define GRIPPER_BELT_MAX_SPEED_RAD_S          1.50f

/* 大臂和小臂均使用固定绝对软件行程限制。 */
#define GRIPPER_ARM_MIN_RAD                   (-1.590f)
#define GRIPPER_ARM_MAX_RAD                   ( 0.220f)
#define GRIPPER_BELT_MIN_RAD                   (-2.750f)
#define GRIPPER_BELT_MAX_RAD                   (-0.260f)

/* 右摇杆上推为正值时，取反后大臂顺时针运动。 */
#define GRIPPER_ARM_DIRECTION                 (-1.0f)
/* 左摇杆上推为正值时，传送轮逆时针运动。 */
#define GRIPPER_BELT_DIRECTION                1.0f

/* 控制周期和安全时序。 */
#define GRIPPER_CONTROL_PERIOD_MS             1U
#define GRIPPER_CONTROL_DT                    0.001f
#define GRIPPER_POWER_UP_WAIT_MS              1500U
#define GRIPPER_ERROR_RESET_WAIT_MS           100U
#define GRIPPER_ENABLE_SETTLE_MS              200U
#define GRIPPER_STARTUP_FEEDBACK_MS           300U
#define GRIPPER_FEEDBACK_TIMEOUT_MS           200U
#define GRIPPER_STATUS_PERIOD_MS              100U
#define GRIPPER_STATUS_REQUEST_PERIOD_MS      (GRIPPER_STATUS_PERIOD_MS / 2U)
#define GRIPPER_CAN_SEND_FAIL_LIMIT           5U
#define GRIPPER_REINIT_PERIOD_MS              500U

/* ========================== 对外变量 ========================== */

/* 位置环参数，可在 Keil Watch 中直接修改。 */
volatile Gripper_pid gripper_pid =
{
  .arm_kp = 12.0f,
  .arm_kd = 0.5f,
  .belt_kp = 12.0f,
  .belt_kd = 0.5f
};

/* 两个 J60 的当前目标位置。 */
volatile GripperPosition_t gripper_position = {0};

/* 现有诊断变量，保留给 Keil Watch 查看。 */
volatile uint32_t g_gripper_init_count = 0U;
volatile uint32_t g_gripper_failsafe_count = 0U;
volatile uint16_t g_gripper_arm_status = 0U;
volatile uint16_t g_gripper_belt_status = 0U;
volatile uint32_t g_gripper_arm_rx_count = 0U;
volatile uint32_t g_gripper_belt_rx_count = 0U;

/* ========================== 内部状态 ========================== */

static uint8_t s_ready;                       /* 1：J60 已完成初始化 */
static uint8_t s_controller_started;          /* 1：气泵/舵机流程已经初始化 */
static uint8_t s_motor_init_started;          /* 1：J60 首次初始化已经发起 */
static uint8_t s_sequence_ready;              /* 1：气泵/舵机流程初始化成功 */
static uint32_t s_last_init_attempt_ms;       /* 上一次 J60 初始化完成时间 */

static float s_arm_min_rad;
static float s_arm_max_rad;
static float s_belt_min_rad;
static float s_belt_max_rad;

static uint32_t s_last_status_request_ms;
static uint8_t s_next_status_node;
static uint8_t s_control_send_fail_count;

/* ========================== 电机安全控制 ========================== */

/**
 * @brief  发送零刚度控制帧并关闭两个 J60 输出。
 */
static void GripperControl_DisableMotors(void)
{
  (void)J60_SendControl(GRIPPER_ARM_ID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  (void)J60_SendControl(GRIPPER_BELT_ID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  (void)J60_Disable(GRIPPER_ARM_ID);
  (void)J60_Disable(GRIPPER_BELT_ID);
}

/**
 * @brief  进入 J60 安全停止状态。
 */
static void GripperControl_FailSafe(void)
{
  const J60_MotorData *arm = J60_GetMotorData(GRIPPER_ARM_ID);
  const J60_MotorData *belt = J60_GetMotorData(GRIPPER_BELT_ID);

  if (arm != NULL)
  {
    g_gripper_arm_status = arm->status;
    g_gripper_arm_rx_count = arm->rx_count;
  }
  if (belt != NULL)
  {
    g_gripper_belt_status = belt->status;
    g_gripper_belt_rx_count = belt->rx_count;
  }

  GripperControl_DisableMotors();
  s_control_send_fail_count = 0U;
  s_ready = 0U;
  g_gripper_failsafe_count++;
}

/* ========================== 电机状态检查 ========================== */

/**
 * @brief  检查两个 J60 是否在线且状态字正常。
 * @retval 1 可以执行周期控制。
 * @retval 0 至少一个节点不满足运行条件。
 */
static uint8_t GripperControl_MotorsReady(void)
{
  const J60_MotorData *arm = J60_GetMotorData(GRIPPER_ARM_ID);
  const J60_MotorData *belt = J60_GetMotorData(GRIPPER_BELT_ID);

  if ((arm == NULL) || (belt == NULL))
  {
    return 0U;
  }

  if ((J60_IsOnline(GRIPPER_ARM_ID,
                    GRIPPER_FEEDBACK_TIMEOUT_MS) == 0U) ||
      (J60_IsOnline(GRIPPER_BELT_ID,
                    GRIPPER_FEEDBACK_TIMEOUT_MS) == 0U))
  {
    return 0U;
  }

  return (arm->status == 0U && belt->status == 0U) ? 1U : 0U;
}

/**
 * @brief  清错、两次使能并等待两个 J60 返回控制反馈。
 * @retval 1 两个节点均在线。
 * @retval 0 初始化失败。
 */
static uint8_t GripperControl_EnableMotors(void)
{
  uint32_t start_ms;

  if ((J60_ResetError(GRIPPER_ARM_ID) != HAL_OK) ||
      (J60_ResetError(GRIPPER_BELT_ID) != HAL_OK))
  {
    return 0U;
  }

  osDelay(GRIPPER_ERROR_RESET_WAIT_MS);

  if ((J60_Enable(GRIPPER_ARM_ID) != HAL_OK) ||
      (J60_Enable(GRIPPER_BELT_ID) != HAL_OK))
  {
    return 0U;
  }

  osDelay(GRIPPER_ENABLE_SETTLE_MS);

  if ((J60_Enable(GRIPPER_ARM_ID) != HAL_OK) ||
      (J60_Enable(GRIPPER_BELT_ID) != HAL_OK))
  {
    return 0U;
  }

  /* 零刚度控制帧只用于获取反馈，不会让两个电机主动出力。 */
  start_ms = HAL_GetTick();
  while ((uint32_t)(HAL_GetTick() - start_ms) <
         GRIPPER_STARTUP_FEEDBACK_MS)
  {
    (void)J60_SendControl(GRIPPER_ARM_ID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    (void)J60_SendControl(GRIPPER_BELT_ID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    osDelay(GRIPPER_CONTROL_PERIOD_MS);
  }

  return GripperControl_MotorsReady();
}

/**
 * @brief  交替请求两个 J60 的状态字。
 * @param  now 当前系统毫秒时间。
 */
static void GripperControl_RequestStatus(uint32_t now)
{
  HAL_StatusTypeDef result;
  uint8_t node_id;

  if ((uint32_t)(now - s_last_status_request_ms) <
      GRIPPER_STATUS_REQUEST_PERIOD_MS)
  {
    return;
  }

  node_id = (s_next_status_node == 0U) ? GRIPPER_ARM_ID : GRIPPER_BELT_ID;
  result = J60_GetStatus(node_id);

  /* 邮箱满时保留当前节点，下个周期继续请求。 */
  if (result == HAL_BUSY)
  {
    return;
  }

  s_last_status_request_ms = now;
  if (result == HAL_OK)
  {
    s_next_status_node = (s_next_status_node == 0U) ? 1U : 0U;
  }
}

/* ========================== J60 初始化 ========================== */

/**
 * @brief  初始化 CAN、两个 J60 和软件行程限制。
 * @param  hcan 当前工程使用的 CAN 句柄。
 */
static void GripperControl_Initialize(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_StateTypeDef can_state;
  float arm_actual_rad;
  float belt_actual_rad;

  s_ready = 0U;
  g_gripper_init_count++;
  s_last_status_request_ms = HAL_GetTick();
  s_next_status_node = 0U;
  s_control_send_fail_count = 0U;

  J60_Init(hcan);

  /* 首次启动为 READY，自动恢复时 CAN 可能已经处于 LISTENING。 */
  can_state = HAL_CAN_GetState(hcan);
  if ((can_state != HAL_CAN_STATE_READY) &&
      (can_state != HAL_CAN_STATE_LISTENING))
  {
    GripperControl_FailSafe();
    return;
  }

  /* 等待驱动器上电稳定，并清除上一次运行锁存的故障。 */
  osDelay(GRIPPER_POWER_UP_WAIT_MS);

  if (GripperControl_EnableMotors() == 0U)
  {
    GripperControl_FailSafe();
    return;
  }

  /* 以启动时的实际位置作为目标，避免上电后突然运动。 */
  arm_actual_rad = J60_GetMotorData(GRIPPER_ARM_ID)->position_rad;
  belt_actual_rad = J60_GetMotorData(GRIPPER_BELT_ID)->position_rad;
  gripper_position.arm_target_rad = arm_actual_rad;
  gripper_position.belt_target_rad = belt_actual_rad;

  /* 大臂和小臂均使用固定绝对限位。 */
  s_arm_min_rad = GRIPPER_ARM_MIN_RAD;
  s_arm_max_rad = GRIPPER_ARM_MAX_RAD;
  s_belt_min_rad = GRIPPER_BELT_MIN_RAD;
  s_belt_max_rad = GRIPPER_BELT_MAX_RAD;

  if ((gripper_position.arm_target_rad < s_arm_min_rad) ||
      (gripper_position.arm_target_rad > s_arm_max_rad) ||
      (gripper_position.belt_target_rad < s_belt_min_rad) ||
      (gripper_position.belt_target_rad > s_belt_max_rad))
  {
    GripperControl_FailSafe();
    return;
  }

  g_gripper_arm_status = J60_GetMotorData(GRIPPER_ARM_ID)->status;
  g_gripper_belt_status = J60_GetMotorData(GRIPPER_BELT_ID)->status;
  g_gripper_arm_rx_count = J60_GetMotorData(GRIPPER_ARM_ID)->rx_count;
  g_gripper_belt_rx_count = J60_GetMotorData(GRIPPER_BELT_ID)->rx_count;
  s_ready = 1U;
}

/* ========================== 电机周期控制 ========================== */

/**
 * @brief  执行一次 J60 电机周期控制。
 * @param  remote 最新遥控数据，可以为 NULL。
 * @note   仅在 GripperControl_Run() 确认 s_ready 为 1 后调用。
 * @retval 1 本周期电机控制正常，可以继续执行夹取流程。
 * @retval 0 电机未就绪或已进入安全停止。
 */
static uint8_t GripperControl_Step(const Remote_Data *remote)
{
  float arm_command;
  float belt_command;
  uint32_t now;
  uint8_t remote_online;
  uint8_t control_send_ok;

  now = HAL_GetTick();
  remote_online =
      (remote != NULL && remote->online != 0U) ? 1U : 0U;

  GripperControl_RequestStatus(now);

  if (GripperControl_MotorsReady() == 0U)
  {
    GripperControl_FailSafe();
    return 0U;
  }

  /* 右摇杆 ch[1] 控制大臂，左摇杆 ch[3] 控制传送轮。 */
  if (remote_online != 0U)
  {
    arm_command = Remote_NormalizeStick(remote->ch[1],
                                        GRIPPER_STICK_DEADBAND);
    belt_command = Remote_NormalizeStick(remote->ch[3],
                                         GRIPPER_STICK_DEADBAND);
  }
  else
  {
    /* 遥控掉线时不再积分目标，电机保持当前位置。 */
    arm_command = 0.0f;
    belt_command = 0.0f;
  }

  gripper_position.arm_target_rad +=
      GRIPPER_ARM_DIRECTION * arm_command *
      GRIPPER_ARM_MAX_SPEED_RAD_S * GRIPPER_CONTROL_DT;
  gripper_position.belt_target_rad +=
      GRIPPER_BELT_DIRECTION * belt_command *
      GRIPPER_BELT_MAX_SPEED_RAD_S * GRIPPER_CONTROL_DT;

  /* 对两个目标位置应用软件行程限制。 */
  if (gripper_position.arm_target_rad < s_arm_min_rad)
  {
    gripper_position.arm_target_rad = s_arm_min_rad;
  }
  else if (gripper_position.arm_target_rad > s_arm_max_rad)
  {
    gripper_position.arm_target_rad = s_arm_max_rad;
  }

  if (gripper_position.belt_target_rad < s_belt_min_rad)
  {
    gripper_position.belt_target_rad = s_belt_min_rad;
  }
  else if (gripper_position.belt_target_rad > s_belt_max_rad)
  {
    gripper_position.belt_target_rad = s_belt_max_rad;
  }

  /* 每个周期同时向两个节点发送位置控制帧。 */
  control_send_ok = 1U;
  if (J60_SendControl(GRIPPER_ARM_ID, gripper_position.arm_target_rad,
                      0.0f, 0.0f,
                      gripper_pid.arm_kp, gripper_pid.arm_kd) != HAL_OK)
  {
    control_send_ok = 0U;
  }
  if (J60_SendControl(GRIPPER_BELT_ID, gripper_position.belt_target_rad,
                      0.0f, 0.0f,
                      gripper_pid.belt_kp, gripper_pid.belt_kd) != HAL_OK)
  {
    control_send_ok = 0U;
  }

  if (control_send_ok == 0U)
  {
    /* 单次邮箱拥塞允许下个周期重试，连续失败才进入安全停止。 */
    s_control_send_fail_count++;
    if (s_control_send_fail_count >= GRIPPER_CAN_SEND_FAIL_LIMIT)
    {
      GripperControl_FailSafe();
      return 0U;
    }
    return 1U;
  }

  s_control_send_fail_count = 0U;
  return 1U;
}

/* ========================== 电机初始化调度 ========================== */

/**
 * @brief  初始化 J60，并在失败后按固定周期重试。
 * @param  hcan 当前工程使用的 CAN 句柄。
 * @retval 1 两个 J60 已就绪。
 * @retval 0 尚未就绪或本次初始化失败。
 * @note   调用本函数前，气泵和舵机应已由调用者置于安全状态。
 */
static uint8_t GripperControl_TryInitializeMotors(CAN_HandleTypeDef *hcan)
{
  uint32_t now;

  if (s_motor_init_started == 0U)
  {
    s_motor_init_started = 1U;
		//初始化两个电机，返回s_ready=1
    GripperControl_Initialize(hcan);
		//上一次 J60 初始化结束时的系统时间
    s_last_init_attempt_ms = HAL_GetTick();
  }
  else if (s_ready == 0U)
  {
    now = HAL_GetTick();
    if ((uint32_t)(now - s_last_init_attempt_ms) >=
        GRIPPER_REINIT_PERIOD_MS)
    {
      GripperControl_Initialize(hcan);
      s_last_init_attempt_ms = HAL_GetTick();
    }
  }

  return s_ready;
}

/* ========================== 唯一任务入口 ========================== */

void GripperControl_Run(const Remote_Data *remote, CAN_HandleTypeDef *hcan)
{
  /* 第一次运行时初始化气泵和舵机，初始化后会立即关闭气泵。 */
  if (s_controller_started == 0U)
  {
    s_controller_started = 1U;
    s_sequence_ready =
        (GripperSequence_Init() == HAL_OK) ? 1U : 0U;
  }

  if (s_ready == 0U)
  {
    /*
     * J60 初始化包含约 2 秒阻塞等待。
     * 必须先关闭气泵并保持舵机角度，再进入初始化或重试流程。
     */
    if (s_sequence_ready != 0U)
    {
      GripperSequence_FailSafe();
    }

    if (GripperControl_TryInitializeMotors(hcan) == 0U)
    {
      return;
    }
  }

	
	//初始化成功之后的判断，保证运行过程中的安全
  /* 电机周期控制返回 0 表示已进入故障保护，不再执行夹取流程。 */
  if (GripperControl_Step(remote) == 0U)
  {
    if (s_sequence_ready != 0U)
    {
      GripperSequence_FailSafe();
    }
    return;
  }

  /* 只在两个 J60 正常时运行气泵、光电门和舵机状态机。 */
  if (s_sequence_ready != 0U)
  {
    GripperSequence_Step(remote);
  }
}
