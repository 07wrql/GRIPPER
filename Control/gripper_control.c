/**
  ******************************************************************************
  * @file    gripper_control.c
  * @brief   夹爪两个 J60 电机的简单遥控位置控制
  ******************************************************************************
  */

#include "gripper_control.h"
#include "j60_driver.h"
#include "can.h"
#include "cmsis_os2.h"

/* ---- 电机分配 ---- */
#define GRIPPER_ARM_ID                  8U
#define GRIPPER_BELT_ID                 4U

/* ---- 遥控输入 ---- */
#define GRIPPER_STICK_DEADBAND          0.08f

/* 右摇杆上推为正值时，-1 使大臂目标角度减小，即顺时针。 */
#define GRIPPER_ARM_DIRECTION           (-1.0f)
/* 左摇杆上推为正值时，+1 对应传送轮逆时针。 */
#define GRIPPER_BELT_DIRECTION          1.0f

/* 摇杆满量程时的目标速度，先使用保守值。 */
#define GRIPPER_ARM_MAX_SPEED_RAD_S     0.20f
#define GRIPPER_BELT_MAX_SPEED_RAD_S    0.80f

/* 与传送轮相同，均以本次上电位置为中心设置相对行程限制。 */
#define GRIPPER_ARM_TRAVEL_LIMIT_RAD     0.80f
#define GRIPPER_BELT_TRAVEL_LIMIT_RAD    2.00f

/*
  pid参数
 */
volatile Gripper_pid gripper_pid =
{
  .arm_kp = 0.0f,
  .arm_kd = 0.0f,
  .belt_kp = 0.0f,
  .belt_kd = 0.0f
};

/* 目标位置集中显示，便于在 Keil Watch 中查看。 */
volatile GripperPosition_t gripper_position = {0};

/* 启动与故障诊断变量，便于在 Keil Watch 中定位通信/初始化问题。 */
volatile uint32_t g_gripper_init_count = 0U;
volatile uint32_t g_gripper_failsafe_count = 0U;
volatile uint16_t g_gripper_arm_status = 0U;
volatile uint16_t g_gripper_belt_status = 0U;
volatile uint32_t g_gripper_arm_rx_count = 0U;
volatile uint32_t g_gripper_belt_rx_count = 0U;

/* ==================== 控制周期与安全时序 ==================== */

/* ---- 1. 运行周期 ---- */
#define GRIPPER_CONTROL_PERIOD_MS       1U      /* 控制循环周期，单位 ms */
#define GRIPPER_CONTROL_DT              0.001f  /* 位置积分步长，单位 s，应与控制周期一致 */

/* ---- 2. 上电初始化时序 ---- */
#define GRIPPER_POWER_UP_WAIT_MS        1500U   /* 上电后等待 J60 驱动器稳定，单位 ms */
#define GRIPPER_ERROR_RESET_WAIT_MS     100U    /* 清错后等待驱动器处理，单位 ms */
#define GRIPPER_ENABLE_SETTLE_MS        200U    /* 两次使能之间的等待时间，单位 ms */
#define GRIPPER_STARTUP_FEEDBACK_MS     300U    /* 使能后等待控制反馈，超时则初始化失败 */

/* ---- 3. 运行监控 ---- */
#define GRIPPER_FEEDBACK_TIMEOUT_MS     200U    /* 超过该时间未收到反馈则判离线，单位 ms */
#define GRIPPER_STATUS_PERIOD_MS        100U    /* J60 状态字查询周期，单位 ms */
#define GRIPPER_STATUS_REQUEST_PERIOD_MS (GRIPPER_STATUS_PERIOD_MS / 2U) /* 两个节点交替请求 */
#define GRIPPER_CAN_SEND_FAIL_LIMIT      5U     /* 连续发送失败达到该次数后进入安全停止 */

/* ==================== 控制器内部状态 ==================== */

/* ---- 1. 控制状态 ---- */
static uint8_t s_ready;                  /* 1：已完成初始化并允许周期控制；0：禁止输出 */

/* ---- 2. 软件行程限制 ---- */
static float s_arm_min_rad;              /* 大臂软件行程下限，单位 rad */
static float s_arm_max_rad;              /* 大臂软件行程上限，单位 rad */
static float s_belt_min_rad;             /* 传送轮软件行程下限，单位 rad */
static float s_belt_max_rad;             /* 传送轮软件行程上限，单位 rad */

/* ---- 3. CAN 状态查询与发送重试 ---- */
static uint32_t s_last_status_request_ms; /* 上一次状态字查询请求时间，单位 ms */
static uint8_t s_next_status_node;        /* 0：下一次查询大臂；1：下一次查询传送轮 */
static uint8_t s_control_send_fail_count; /* 控制帧连续发送失败次数 */


/**
  * @brief  向两个 J60 发送零刚度帧并关闭电机输出。
  *
  * 先发送零增益控制帧，可以让驱动器停止主动跟踪目标；
  * 随后再发送 Disable，避免电机继续保持位置闭环。
  */
static void GripperControl_DisableMotors(void)
{
  (void)J60_SendControl(GRIPPER_ARM_ID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  (void)J60_SendControl(GRIPPER_BELT_ID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  (void)J60_Disable(GRIPPER_ARM_ID);
  (void)J60_Disable(GRIPPER_BELT_ID);
}

/**
  * @brief  进入安全停止状态。
  */
void GripperControl_FailSafe(void)
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

/**
  * @brief  检查两个 J60 是否都满足运行条件。
  * @retval 1 两个节点在线、反馈未超时且状态字均为零。
  * @retval 0 任一条件不满足。
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
  * @brief  清除错误、使能两个 J60，并等待控制反馈。
  *
  * 初始化顺序与成功测试工程保持一致：
  *   1. 两个节点分别清错；
  *   2. 等待驱动器完成错误复位；
  *   3. 两个节点分别使能；
  *   4. 间隔一段时间后再次使能；
  *   5. 连续发送 300 ms 零刚度帧；
  *   6. 确认两个节点的反馈均在线。
  *
  * 零刚度帧不会让电机出力，只用于让驱动器返回位置、速度和力矩。
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
  *
  * 每个控制周期最多请求一个节点，两个节点轮流查询。
  * 发送邮箱满时保持当前节点和计时不变，下个周期继续重试。
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

  /* 邮箱拥塞时立即保留原计划，下个控制周期重试。 */
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

/**
  * @brief  初始化 CAN、两个 J60 和控制器内部目标。
  * @param  hcan 当前工程使用 hcan1。
  *
  * 初始化成功后，两个目标位置都同步为当前实际反馈位置。
  * 上电后电机保持在当前位置，不会主动向 0 rad 运动。
  */
void GripperControl_Init(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_StateTypeDef can_state;
  float arm_actual_rad;
  float belt_actual_rad;

	//允许电机控制标志
  s_ready = 0U;
  g_gripper_init_count++;
  s_last_status_request_ms = HAL_GetTick();
  s_next_status_node = 0U;
  s_control_send_fail_count = 0U;
	//j60电机初始化
  J60_Init(hcan);
	
  /* 首次启动为 READY，自动恢复重试时 CAN 已经处于 BUSY。 */
  can_state = HAL_CAN_GetState(hcan);
  if ((can_state != HAL_CAN_STATE_READY) &&
      (can_state != HAL_CAN_STATE_LISTENING))
  {
    GripperControl_FailSafe();
    return;
  }

  /* 给两个驱动器足够的上电时间，并清掉上一次运行锁存的超时故障。 */
  osDelay(GRIPPER_POWER_UP_WAIT_MS);

  /* 清错、双次使能并确认两个节点反馈在线。 */
  if (GripperControl_EnableMotors() == 0U)
  {
    GripperControl_FailSafe();
    return;
  }

  /* 记录上电时的实际位置，用于软件限位和启动安全检查。 */
  arm_actual_rad = J60_GetMotorData(GRIPPER_ARM_ID)->position_rad;
  belt_actual_rad = J60_GetMotorData(GRIPPER_BELT_ID)->position_rad;

  /* 使用当前位置作为目标，避免上电后突然向零位或旧目标运动。 */
  gripper_position.arm_target_rad = arm_actual_rad;
  gripper_position.belt_target_rad = belt_actual_rad;

  /* 大臂和传送轮都以本次上电位置为中心建立相对软件限位。 */
  s_arm_min_rad = arm_actual_rad - GRIPPER_ARM_TRAVEL_LIMIT_RAD;
  s_arm_max_rad = arm_actual_rad + GRIPPER_ARM_TRAVEL_LIMIT_RAD;
  s_belt_min_rad = belt_actual_rad - GRIPPER_BELT_TRAVEL_LIMIT_RAD;
  s_belt_max_rad = belt_actual_rad + GRIPPER_BELT_TRAVEL_LIMIT_RAD;

  /* 目标零点必须位于软件限位内，否则禁止启动运动。 */
  if ((gripper_position.arm_target_rad < s_arm_min_rad) ||
      (gripper_position.arm_target_rad > s_arm_max_rad))
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

/**
  * @brief  夹爪 1 ms 周期控制。
  * @param  remote 遥控器数据指针。
  *
  * 控制流程：
  *   1. 检查控制器是否已经完成初始化；
  *   2. 检查两个电机是否在线且状态正常；
  *   3. 每 50 ms 交替查询一个电机的状态字，每个电机约 100 ms 查询一次；
  *   4. 读取右摇杆 ch[1] 和左摇杆 ch[3]；
  *   5. 将摇杆速度积分成目标位置；
  *   6. 对目标位置应用软件行程限制；
  *   7. 每 1 ms 向两个 J60 发送位置控制帧。
  */
void GripperControl_Step(const Remote_Data *remote)
{
  float arm_command;
  float belt_command;
  uint32_t now;
  uint8_t remote_online;
  uint8_t control_send_ok;

  if (s_ready == 0U)
  {
    return;
  }

  /* 先检查电机在线状态，再根据遥控输入更新控制目标。 */
  now = HAL_GetTick();
  remote_online =
      (remote != NULL && remote->online != 0U) ? 1U : 0U;

  /* 两个节点交替查询，避免状态帧和控制帧在同一周期占满发送邮箱。 */
  GripperControl_RequestStatus(now);

  if (GripperControl_MotorsReady() == 0U)
  {
    GripperControl_FailSafe();
    return;
  }

  /* ch[1]：右摇杆上下；ch[3]：左摇杆上下。 */
  if (remote_online != 0U)
  {
    arm_command = Remote_NormalizeStick(remote->ch[1],
                                      GRIPPER_STICK_DEADBAND);
    belt_command = Remote_NormalizeStick(remote->ch[3],
                                       GRIPPER_STICK_DEADBAND);
  }
  else
  {
    /* 遥控掉线时停止更新目标，继续发送原目标以保持当前位置。 */
    arm_command = 0.0f;
    belt_command = 0.0f;
  }

  /* 摇杆值代表目标速度，乘以控制周期后积分成位置增量。 */
  gripper_position.arm_target_rad += GRIPPER_ARM_DIRECTION * arm_command *
                      GRIPPER_ARM_MAX_SPEED_RAD_S * GRIPPER_CONTROL_DT;
  gripper_position.belt_target_rad += GRIPPER_BELT_DIRECTION * belt_command *
                       GRIPPER_BELT_MAX_SPEED_RAD_S * GRIPPER_CONTROL_DT;

  /* 大臂和传送轮都必须限制在各自的软件安全行程内。 */
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

  /* 每 1 ms 分别向 8 号大臂和 4 号传送轮发送位置闭环指令。 */
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
    /*
     * 邮箱暂时占满等单次失败不应立即停机，下一毫秒继续重试。
     * 连续多个周期都无法入队才视为通信故障。
     */
    s_control_send_fail_count++;
    if (s_control_send_fail_count >= GRIPPER_CAN_SEND_FAIL_LIMIT)
    {
      GripperControl_FailSafe();
    }
    return;
  }

  s_control_send_fail_count = 0U;

}

/**
  * @brief  返回控制器当前是否可以执行周期控制。
  * @retval 1 已完成初始化。
  * @retval 0 未初始化或已进入安全停止。
  */
uint8_t GripperControl_Ready(void)
{
  return s_ready;
}















