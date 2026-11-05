/**
 * @file    gripper_sequence.c
 * @brief   夹爪气泵、光电门和舵机顺序控制实现。
 * @details
 *   本模块实现固定夹取流程，并通过状态机保证舵机动作成对出现：
 *   HOME 角度 -> FLIP 角度 -> HOME 角度。只有回到 HOME 角度后，
 *   才允许下一轮翻转动作。
 *
 *   2 档不参与控制；1 档只负责打开气泵；3 档负责舵机翻转。
 *   拨杆按档位变化沿触发，光电门直接读取原始电平。所有输出只在状态迁移时
 *   更新，不使用阻塞延时。
 */

#include "gripper_sequence.h"
#include "photogate_driver.h"
#include "relay_driver.h"
#include "servo_driver.h"

/* ========================== 可调试配置 ========================== */

/*
 * 舵机角度配置。两个字段均可在 Keil Watch 中直接修改，单位为度。
 * 修改后从下一次对应角度切换开始生效；启动角度在复位初始化时生效。
 */
volatile GripperServoConfig_t gripper_servo_config =
{
  .home_angle_deg = 0,
  .flip_angle_deg = 180
};

/* ========================== 内部类型 ========================== */

/* 夹取流程状态。 */
typedef enum
{
  GRIPPER_SEQ_DISARMED = 0,   /* 安全等待，气泵关闭 */
  GRIPPER_SEQ_WAIT_OBJECT,    /* 气泵打开，等待光电门检测物体 */
  GRIPPER_SEQ_CLAMPED,        /* 气泵关闭，等待第一次进入 3 档 */
  GRIPPER_SEQ_FLIPPED,        /* 舵机位于 FLIP 角度，等待进入 1 档 */
  GRIPPER_SEQ_REOPENED,       /* 气泵打开，等待第二次进入 3 档 */
  GRIPPER_SEQ_WAIT_RELEASE    /* 舵机位于 HOME 角度，等待光电门恢复低电平 */
} GripperSequence_State;



/* ========================== 内部状态 ========================== */

static GripperSequence_State s_state = GRIPPER_SEQ_DISARMED;
static uint8_t s_servo_ready;
static uint8_t s_pump_on;
static int32_t s_servo_angle;

/* 左拨杆当前档位和变化沿检测状态。 */
static uint8_t s_switch_stable;
static uint8_t s_switch_valid;

/* 光电门当前电平。 */
static uint8_t s_photo_stable;

/* ========================== 输出控制 ========================== */

/**
 * @brief  设置气泵状态。
 * @param  enabled 1：气泵开，夹爪张开；0：气泵关，夹爪闭合。
 */
static void GripperSequence_SetPump(uint8_t enabled)
{
  if (s_pump_on != enabled)
  {
    Relay_Set(enabled);
    s_pump_on = enabled;
  }
}

/**
 * @brief  设置舵机角度，仅在目标角度变化时写 PWM。
 * @param  angle_deg 目标角度，单位度。
 */
static void GripperSequence_SetServo(int32_t angle_deg)
{
  if ((s_servo_ready != 0U) && (s_servo_angle != angle_deg))
  {
    Servo_SetAngle(angle_deg);
    s_servo_angle = angle_deg;
  }
}

/**
 * @brief  检查左拨杆档位，并在档位变化时返回当前档位。
 * @param  remote 最新遥控数据。
 * @return 0 表示档位没有变化；1/2/3 表示刚进入对应档位。
 */
static uint8_t GripperSequence_UpdateSwitch(const Remote_Data *remote)
{
  uint8_t position = remote->s[REMOTE_SW_LEFT_S1];

  /* 第一次取得有效遥控数据时，把当前档位作为初值。 */
  if (s_switch_valid == 0U)
  {
    s_switch_stable = position;
    s_switch_valid = 1U;
    return position;
  }

  /* 不做软件防抖，只在档位发生变化的周期触发一次。 */
  if (position != s_switch_stable)
  {
    s_switch_stable = position;
    return position;
  }

  return 0U;
}


/**
 * @brief  读取光电门当前电平。
 *
 * STAB-13DTN 在光束被遮挡时输出高电平，因此 s_photo_stable 为 1
 */
static void GripperSequence_UpdatePhoto(void)
{
  /* 高电平表示检测到物体，低电平表示光束畅通。 */
  s_photo_stable = (PhotoGate_ReadRawLevel() == GPIO_PIN_SET) ? 1U : 0U;
}

/* ========================== 公共接口 ========================== */

HAL_StatusTypeDef GripperSequence_Init(void)
{
  /* 初始化期间先保证气泵关闭，再启动舵机 PWM。 */
  Relay_Init();                    //继电器初始化
  s_pump_on = 0U;
  s_servo_ready = 0U;
  s_state = GRIPPER_SEQ_DISARMED;  //安全等待
  s_switch_valid = 0U;
  s_photo_stable = 0U;

  if (Servo_Init() != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* 启动时直接使用结构体中的 HOME 角度。 */
  Servo_SetAngle(gripper_servo_config.home_angle_deg);
  s_servo_angle = gripper_servo_config.home_angle_deg;
  s_servo_ready = 1U;
  return HAL_OK;
}

void GripperSequence_Step(const Remote_Data *remote)
{
  uint8_t switch_position;

  /* 遥控无效时立即进入安全状态，舵机保持当前角度。 */
  if ((remote == NULL) || (remote->online == 0U) ||
      (s_servo_ready == 0U))
  {
    GripperSequence_FailSafe();
    return;
  }

	//获取左拨杆的最新状态
  switch_position = GripperSequence_UpdateSwitch(remote);
	//获取光电门现在的状态
  GripperSequence_UpdatePhoto();

  switch (s_state)
  {
    case GRIPPER_SEQ_DISARMED:
      /* 进入 1 档后重新开泵；若舵机仍在 FLIP 角度，等待下一次 3 档回位。 */
      if (switch_position == 1U)
      {
				//设置气泵开，张开夹爪
        GripperSequence_SetPump(1U);
				//进入等待物体进来阶段
        s_state = (s_servo_angle == gripper_servo_config.flip_angle_deg)
                      ? GRIPPER_SEQ_REOPENED
                      : GRIPPER_SEQ_WAIT_OBJECT;
      }
      break;

    case GRIPPER_SEQ_WAIT_OBJECT:
      /* 光电门高电平表示检测到物体，关闭气泵夹紧。 */
      if (s_photo_stable != 0U)
      {
        GripperSequence_SetPump(0U);
        s_state = GRIPPER_SEQ_CLAMPED;
      }
      break;

    case GRIPPER_SEQ_CLAMPED:
      /* 第一次进入 3 档：舵机 HOME 角度 -> FLIP 角度。 */
      if (switch_position == 3U)
      {
        GripperSequence_SetServo(gripper_servo_config.flip_angle_deg);
        s_state = GRIPPER_SEQ_FLIPPED;
      }
      break;

    case GRIPPER_SEQ_FLIPPED:
      /* 回到 1 档重新开泵，舵机保持 FLIP 角度。 */
      if (switch_position == 1U)
      {
        GripperSequence_SetPump(1U);
        s_state = GRIPPER_SEQ_REOPENED;
      }
      break;

    case GRIPPER_SEQ_REOPENED:
      /* 第二次进入 3 档：舵机 FLIP 角度 -> HOME 角度。 */
      if (switch_position == 3U)
      {
        GripperSequence_SetServo(gripper_servo_config.home_angle_deg);
        s_state = GRIPPER_SEQ_WAIT_RELEASE;
      }
      break;

    case GRIPPER_SEQ_WAIT_RELEASE:
      /* 等待光电门恢复低电平，避免同一个物体重复触发下一轮夹紧。 */
      if (s_photo_stable == 0U)
      {
        s_state = GRIPPER_SEQ_WAIT_OBJECT;
      }
      break;

    default:
      GripperSequence_FailSafe();
      break;
  }
}

void GripperSequence_FailSafe(void)
{
  /* 停止气泵，舵机保持当前位置，等待遥控重新有效。 */
  GripperSequence_SetPump(0U);
  s_state = GRIPPER_SEQ_DISARMED;
  s_switch_valid = 0U;
}
