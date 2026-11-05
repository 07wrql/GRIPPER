/**
 * @file    gripper_control.c
 * @brief   夹爪总控制实现。
 * @details
 *   任务层只调用GripperControl_Run()。本文件内部包含两个互不阻塞的流程：
 *   1. 气泵、光电门和舵机控制；
 *   2. J60初始化、失败重试和周期控制。
 *
 *   J60是否初始化成功不会阻止夹爪流程运行，J60故障也不会关闭气泵
 *   或禁止舵机。只有遥控掉线会同时触发两边各自的安全状态。
 */

#include "gripper_control.h"
#include "gripper_sequence.h"
#include "j60_driver.h"
#include "can.h"

/* ========================== 配置常量 ========================== */

/* J60节点分配。 */
#define GRIPPER_ARM_ID                        8U
#define GRIPPER_BELT_ID                       2U

/* 摇杆死区和满量程目标速度。 */
#define GRIPPER_STICK_DEADBAND                0.08f
#define GRIPPER_ARM_MAX_SPEED_RAD_S           0.50f
#define GRIPPER_BELT_MAX_SPEED_RAD_S          1.50f

/* J60软件行程限制。 */
#define GRIPPER_ARM_MIN_RAD                   (-1.625f)
#define GRIPPER_ARM_MAX_RAD                   ( 0.270f)
#define GRIPPER_BELT_MIN_RAD                   (-2.750f)
#define GRIPPER_BELT_MAX_RAD                   (-0.260f)

/* 摇杆方向。 */
#define GRIPPER_ARM_DIRECTION                 (-1.0f)
#define GRIPPER_BELT_DIRECTION                1.0f

/*
 * 控制周期和安全时序。
 *
 * 这些时间全部通过HAL_GetTick()配合状态机判断，不在控制任务里调用
 * osDelay()等待，因此J60初始化或重试期间，气泵和舵机流程仍然每1ms运行。
 *
 * GRIPPER_POWER_UP_WAIT_MS:
 *   J60上电后的最大等待时间。两个节点都提前反馈时可以提前结束。
 * GRIPPER_ERROR_RESET_WAIT_MS:
 *   发送清错命令后，等待驱动器处理命令的时间。
 * GRIPPER_ENABLE_SETTLE_MS:
 *   第一次使能后，等待驱动器进入使能状态的时间。
 * GRIPPER_STARTUP_FEEDBACK_MS:
 *   启动反馈阶段的总超时；两个电机提前就绪时立即结束。
 * GRIPPER_FEEDBACK_TIMEOUT_MS:
 *   正常运行时，超过该时间没有收到反馈就认为J60掉线。
 * GRIPPER_STATUS_REQUEST_PERIOD_MS:
 *   交替请求两个节点状态字的周期。
 * GRIPPER_CAN_SEND_FAIL_LIMIT:
 *   连续发送失败达到该次数后进入J60故障重试。
 * GRIPPER_REINIT_PERIOD_MS:
 *   J60初始化失败后，等待多久重新尝试。
 */
#define GRIPPER_CONTROL_DT                    0.001f
#define GRIPPER_POWER_UP_WAIT_MS              500U
#define GRIPPER_ERROR_RESET_WAIT_MS           30U
#define GRIPPER_ENABLE_SETTLE_MS              30U
#define GRIPPER_STARTUP_FEEDBACK_MS           300U
#define GRIPPER_FEEDBACK_TIMEOUT_MS           200U
#define GRIPPER_STATUS_PERIOD_MS              100U
#define GRIPPER_STATUS_REQUEST_PERIOD_MS      (GRIPPER_STATUS_PERIOD_MS / 2U)
#define GRIPPER_CAN_SEND_FAIL_LIMIT           5U
#define GRIPPER_REINIT_PERIOD_MS              500U

/* ========================== 对外变量 ========================== */

volatile Gripper_pid gripper_pid =
{
  .arm_kp = 15.0f,
  .arm_kd = 0.8f,
  .belt_kp = 12.0f,
  .belt_kd = 0.8f
};

volatile GripperPosition_t gripper_position = {0};

volatile uint32_t g_gripper_init_count = 0U;
volatile uint32_t g_gripper_failsafe_count = 0U;
volatile uint16_t g_gripper_arm_status = 0U;
volatile uint16_t g_gripper_belt_status = 0U;
volatile uint32_t g_gripper_arm_rx_count = 0U;
volatile uint32_t g_gripper_belt_rx_count = 0U;
volatile uint8_t g_gripper_motor_ready = 0U;
volatile uint8_t g_gripper_motor_state = 0U;

/* ========================== 内部类型 ========================== */

/*
 * J60非阻塞初始化状态机。
 *
 * 状态迁移全部在GripperControl_MotorStep()中完成，每次只执行当前状态
 * 对应的少量工作，不在任何一个状态里阻塞等待。
 */
typedef enum
{
  GRIPPER_MOTOR_IDLE = 0,       /* 尚未开始，等待第一次初始化 */
  GRIPPER_MOTOR_BOOT_WAIT,      /* 等待J60上电，同时发送状态查询 */
  GRIPPER_MOTOR_RESET,          /* 向两个节点发送错误复位 */
  GRIPPER_MOTOR_RESET_WAIT,     /* 等待驱动器处理错误复位 */
  GRIPPER_MOTOR_ENABLE,         /* 第一次发送使能命令 */
  GRIPPER_MOTOR_ENABLE_WAIT,    /* 等待第一次使能稳定 */
  GRIPPER_MOTOR_FEEDBACK,       /* 发送零刚度控制并等待有效反馈 */
  GRIPPER_MOTOR_READY,          /* 两个J60在线且状态正常 */
  GRIPPER_MOTOR_RETRY_WAIT      /* 初始化失败后等待重试 */
} GripperMotorState;

/* ========================== 内部状态 ========================== */

static uint8_t s_ready;                       /* 1：J60已就绪 */
static uint8_t s_controller_started;          /* 1：首次初始化已执行 */
static GripperMotorState s_motor_state;       /* 当前J60初始化状态 */
static uint32_t s_motor_state_ms;             /* 当前状态进入时间 */

/* J60软件行程限制，在J60首次就绪时装入。 */
static float s_arm_min_rad;
static float s_arm_max_rad;
static float s_belt_min_rad;
static float s_belt_max_rad;

/* 状态字请求交替选择节点，避免同一时刻占满CAN发送邮箱。 */
static uint32_t s_last_status_request_ms;
static uint8_t s_next_status_node;
static uint8_t s_control_send_fail_count;

/* ========================== 电机安全控制 ========================== */

/**
 * @brief  发送零刚度控制帧并关闭两个J60输出。
 */
static void GripperControl_DisableMotors(void)
{
  (void)J60_SendControl(GRIPPER_ARM_ID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  (void)J60_SendControl(GRIPPER_BELT_ID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  (void)J60_Disable(GRIPPER_ARM_ID);
  (void)J60_Disable(GRIPPER_BELT_ID);
}

/**
 * @brief  进入J60安全停止状态。
 * @note   只影响J60，不影响气泵和舵机。
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
  g_gripper_motor_ready = 0U;
  g_gripper_failsafe_count++;
}

/**
 * @brief  切换J60初始化状态机状态。
 */
static void GripperControl_SetMotorState(GripperMotorState state,
                                         uint32_t now)
{
  s_motor_state = state;
  s_motor_state_ms = now;
  g_gripper_motor_state = (uint8_t)state;
}

/**
 * @brief  进入失败等待，500ms后重新开始J60初始化。
 */
static void GripperControl_EnterRetry(uint32_t now)
{
  GripperControl_FailSafe();
  GripperControl_SetMotorState(GRIPPER_MOTOR_RETRY_WAIT, now);
}

/* ========================== 电机状态检查 ========================== */

/**
 * @brief  检查两个J60是否都有有效反馈。
 */
static uint8_t GripperControl_MotorsOnline(void)
{
  return ((J60_IsOnline(GRIPPER_ARM_ID,
                        GRIPPER_FEEDBACK_TIMEOUT_MS) != 0U) &&
          (J60_IsOnline(GRIPPER_BELT_ID,
                        GRIPPER_FEEDBACK_TIMEOUT_MS) != 0U))
             ? 1U
             : 0U;
}

/**
 * @brief  检查两个J60是否在线且状态字正常。
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

  return ((arm->status == 0U) && (belt->status == 0U)) ? 1U : 0U;
}

/**
 * @brief  交替请求两个J60的状态字。
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

/* ========================== 初始化与周期控制 ========================== */

/**
 * @brief  开始一次新的J60初始化流程。
 */
//它是 J60 非阻塞初始化状态机的入口，负责清理状态、
//检查 CAN、初始化驱动并启动后续初始化流程；函数本身不会等待初始化完成。
static void GripperControl_BeginInit(CAN_HandleTypeDef *hcan)
{
	//记录当前时间
  uint32_t now = HAL_GetTick();
  HAL_CAN_StateTypeDef can_state;
  //清楚电机准备就绪的状态
  s_ready = 0U;
  g_gripper_motor_ready = 0U;
	//统计初始化次数
  g_gripper_init_count++;
	//清空连续发送失败计数
  s_control_send_fail_count = 0U;
	//重置状态字请求调度
  s_last_status_request_ms = now;
  s_next_status_node = 0U;

  if (hcan == NULL)
  {
    GripperControl_EnterRetry(now);
    return;
  }

  J60_Init(hcan);

  can_state = HAL_CAN_GetState(hcan);
  if ((can_state != HAL_CAN_STATE_READY) &&
      (can_state != HAL_CAN_STATE_LISTENING))
  {
    GripperControl_EnterRetry(now);
    return;
  }

  GripperControl_SetMotorState(GRIPPER_MOTOR_BOOT_WAIT, now);
}

/**
 * @brief  在J60反馈建立后读取实际位置并进入READY。
 * @retval 1 位置有效并已进入READY。
 * @retval 0 位置超出软件限位。
 */
static uint8_t GripperControl_ActivateReady(uint32_t now)
{
  const J60_MotorData *arm = J60_GetMotorData(GRIPPER_ARM_ID);
  const J60_MotorData *belt = J60_GetMotorData(GRIPPER_BELT_ID);

  if ((arm == NULL) || (belt == NULL))
  {
    return 0U;
  }

  s_arm_min_rad = GRIPPER_ARM_MIN_RAD;
  s_arm_max_rad = GRIPPER_ARM_MAX_RAD;
  s_belt_min_rad = GRIPPER_BELT_MIN_RAD;
  s_belt_max_rad = GRIPPER_BELT_MAX_RAD;

  gripper_position.arm_target_rad = arm->position_rad;
  gripper_position.belt_target_rad = belt->position_rad;

  if ((gripper_position.arm_target_rad < s_arm_min_rad) ||
      (gripper_position.arm_target_rad > s_arm_max_rad) ||
      (gripper_position.belt_target_rad < s_belt_min_rad) ||
      (gripper_position.belt_target_rad > s_belt_max_rad))
  {
    return 0U;
  }

  g_gripper_arm_status = arm->status;
  g_gripper_belt_status = belt->status;
  g_gripper_arm_rx_count = arm->rx_count;
  g_gripper_belt_rx_count = belt->rx_count;
  s_ready = 1U;
  g_gripper_motor_ready = 1U;
  GripperControl_SetMotorState(GRIPPER_MOTOR_READY, now);
  return 1U;
}

/**
 * @brief  执行一次J60周期位置控制。
 * @retval 1 本周期正常。
 * @retval 0 J60故障或通信连续失败。
 */
static uint8_t GripperControl_Step(const Remote_Data *remote, uint32_t now)
{
  float arm_command;
  float belt_command;
  uint8_t remote_online;
  uint8_t control_send_ok;

  remote_online =
      ((remote != NULL) && (remote->online != 0U)) ? 1U : 0U;

  GripperControl_RequestStatus(now);

  if (GripperControl_MotorsReady() == 0U)
  {
    return 0U;
  }

  if (remote_online != 0U)
  {
    arm_command = Remote_NormalizeStick(remote->ch[1],
                                        GRIPPER_STICK_DEADBAND);
    belt_command = Remote_NormalizeStick(remote->ch[3],
                                         GRIPPER_STICK_DEADBAND);
  }
  else
  {
    arm_command = 0.0f;
    belt_command = 0.0f;
  }

  gripper_position.arm_target_rad +=
      GRIPPER_ARM_DIRECTION * arm_command *
      GRIPPER_ARM_MAX_SPEED_RAD_S * GRIPPER_CONTROL_DT;
  gripper_position.belt_target_rad +=
      GRIPPER_BELT_DIRECTION * belt_command *
      GRIPPER_BELT_MAX_SPEED_RAD_S * GRIPPER_CONTROL_DT;

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
    s_control_send_fail_count++;
    if (s_control_send_fail_count >= GRIPPER_CAN_SEND_FAIL_LIMIT)
    {
      return 0U;
    }
  }
  else
  {
    s_control_send_fail_count = 0U;
  }

  return 1U;
}

/**
 * @brief  非阻塞J60状态机，每1ms执行一次。
 */
static void GripperControl_MotorStep(const Remote_Data *remote,
                                     CAN_HandleTypeDef *hcan)
{
  uint32_t now = HAL_GetTick();
  uint32_t elapsed = (uint32_t)(now - s_motor_state_ms);

  switch (s_motor_state)
  {
    case GRIPPER_MOTOR_IDLE:
      GripperControl_BeginInit(hcan);
      break;

    case GRIPPER_MOTOR_BOOT_WAIT:
      /* 周期查询状态；两个节点都提前响应时无需等满上电时间。 */
      GripperControl_RequestStatus(now);
      if ((GripperControl_MotorsOnline() != 0U) ||
          (elapsed >= GRIPPER_POWER_UP_WAIT_MS))
      {
        GripperControl_SetMotorState(GRIPPER_MOTOR_RESET, now);
      }
      break;

    case GRIPPER_MOTOR_RESET:
      /* 清掉驱动器上一次运行可能锁存的故障。 */
      if ((J60_ResetError(GRIPPER_ARM_ID) != HAL_OK) ||
          (J60_ResetError(GRIPPER_BELT_ID) != HAL_OK))
      {
        GripperControl_EnterRetry(now);
      }
      else
      {
        GripperControl_SetMotorState(GRIPPER_MOTOR_RESET_WAIT, now);
      }
      break;

    case GRIPPER_MOTOR_RESET_WAIT:
      if (elapsed >= GRIPPER_ERROR_RESET_WAIT_MS)
      {
        GripperControl_SetMotorState(GRIPPER_MOTOR_ENABLE, now);
      }
      break;

    case GRIPPER_MOTOR_ENABLE:
      if ((J60_Enable(GRIPPER_ARM_ID) != HAL_OK) ||
          (J60_Enable(GRIPPER_BELT_ID) != HAL_OK))
      {
        GripperControl_EnterRetry(now);
      }
      else
      {
        GripperControl_SetMotorState(GRIPPER_MOTOR_ENABLE_WAIT, now);
      }
      break;

    case GRIPPER_MOTOR_ENABLE_WAIT:
      if (elapsed >= GRIPPER_ENABLE_SETTLE_MS)
      {
        /*
         * 保留第二次使能，提高上电阶段使能命令的可靠性。
         * 第二次使能成功后进入反馈等待，不再固定等待满300ms。
         */
        if ((J60_Enable(GRIPPER_ARM_ID) != HAL_OK) ||
            (J60_Enable(GRIPPER_BELT_ID) != HAL_OK))
        {
          GripperControl_EnterRetry(now);
        }
        else
        {
          GripperControl_SetMotorState(GRIPPER_MOTOR_FEEDBACK, now);
        }
      }
      break;

    case GRIPPER_MOTOR_FEEDBACK:
      /*
       * 零刚度控制帧只用于获取位置、速度和状态反馈，不会让电机主动出力。
       * 两个节点一旦就绪就立即进入READY，只有超时才判定本次初始化失败。
       */
      (void)J60_SendControl(GRIPPER_ARM_ID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      (void)J60_SendControl(GRIPPER_BELT_ID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      GripperControl_RequestStatus(now);

      if (GripperControl_MotorsReady() != 0U)
      {
        if (GripperControl_ActivateReady(now) == 0U)
        {
          GripperControl_EnterRetry(now);
        }
      }
      else if (elapsed >= GRIPPER_STARTUP_FEEDBACK_MS)
      {
        GripperControl_EnterRetry(now);
      }
      break;

    case GRIPPER_MOTOR_READY:
      /* 正常运行；反馈丢失或连续发送失败只让J60进入重试。 */
      if (GripperControl_Step(remote, now) == 0U)
      {
        GripperControl_EnterRetry(now);
      }
      break;

    case GRIPPER_MOTOR_RETRY_WAIT:
      /* 等待重试期间夹爪流程仍照常运行，不受J60影响。 */
      if (elapsed >= GRIPPER_REINIT_PERIOD_MS)
      {
        GripperControl_BeginInit(hcan);
      }
      break;

    default:
      GripperControl_BeginInit(hcan);
      break;
  }
}

/* ========================== 唯一任务入口 ========================== */

void GripperControl_Run(const Remote_Data *remote, CAN_HandleTypeDef *hcan)
{

 /**
	 * 舵机、舵机和电机的初始化
	 */
  if (s_controller_started == 0U)
  {
    s_controller_started = 1U;
		//进行气泵和舵机(夹爪)的初始化，且初始化不干涉
    GripperSequence_Init();
		//进行两个j60电机的初始化
    GripperControl_BeginInit(hcan);
  }

  /*
	 * 启动任务，并且夹爪和电机任务可以并发
   */
  GripperSequence_Step(remote);
  GripperControl_MotorStep(remote, hcan);
}