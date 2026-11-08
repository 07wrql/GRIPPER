/**
 * @file    gripper_sequence.c
 * @brief   气泵、光电门和舵机独立控制实现。
 * @details
 *   气泵与舵机使用独立就绪标志，互不阻塞：
 *   - 气泵只依赖继电器和光电门接口；
 *   - 舵机只依赖PWM输出；
 *   - 初始化只执行一次，失败后不自动重试；
 *   - J60状态完全不在本模块中判断。
 *
 *   拨杆规则：
 *   - 3档无输出动作，只作为中位；
 *   - 1档按电平打开气泵；
 *   - 每次首次进入2档时翻转一次舵机标志；
 *   - 光电门在3档检测到物体时关闭气泵。
 */

#include "gripper_sequence.h"
#include "photogate_driver.h"
#include "relay_driver.h"
#include "servo_driver.h"


/* ========================== 固定角度定义 ========================== */

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

/* ========================== 运行状态 ========================== */

volatile GripperRuntime_t g_gripper_runtime;

/* ========================== 输出控制 ========================== */

/**
 * @brief  设置气泵状态。
 * @param  on 1：气泵打开、释放；0：气泵关闭、夹紧。
 * @note   当前接线与继电器逻辑相反，在这里统一完成极性转换。
 */
static void GripperSequence_SetPumpOn(uint8_t on)
{
  uint8_t next_on = (on != 0U) ? 1U : 0U;

  if (g_gripper_runtime.pump_on != next_on)
  {
    Relay_Set((next_on != 0U) ? GRIPPER_PUMP_OPEN_RELAY_STATE
                              : GRIPPER_PUMP_CLOSE_RELAY_STATE);
    g_gripper_runtime.pump_on = next_on;
  }
}

/**
 * @brief  在舵机就绪时设置目标角度并更新PWM。
 * @param  angle_deg 目标角度，单位度。
 */
static void GripperSequence_SetServo(int32_t angle_deg)
{
  if (g_gripper_runtime.servo_ready != 0U)
  {
    Servo_SetAngle(angle_deg);
  }
}

/**
 * @brief  翻转舵机标志，并执行一次对应的HOME/翻转角度动作。
 */
static void GripperSequence_ToggleServo(void)
{
  g_gripper_runtime.servo_flipped =
      (g_gripper_runtime.servo_flipped == 0U) ? 1U : 0U;

  GripperSequence_SetServo(
      (g_gripper_runtime.servo_flipped != 0U)
          ? GRIPPER_SERVO_FLIP_ANGLE_DEG
          : GRIPPER_SERVO_HOME_ANGLE_DEG);
}

/**
 * @brief  独立初始化气泵子系统。
 * @retval HAL_OK    继电器输出初始化成功。
 * @retval HAL_ERROR 继电器输出状态异常。
 */
static HAL_StatusTypeDef GripperSequence_InitPump(void)
{
  /*
   * 上电默认张开：继电器常闭，气阀得电。
   */
  Relay_Init();

  g_gripper_runtime.pump_on = 1U;
  g_gripper_runtime.pump_ready = 1U;

  return HAL_OK;
}

/**
 * @brief  独立初始化舵机子系统。
 * @retval HAL_OK    PWM启动成功且HOME角度已输出。
 * @retval HAL_ERROR PWM启动失败或通道未进入运行状态。
 */
static HAL_StatusTypeDef GripperSequence_InitServo(void)
{
  /*
   * 舵机初始化只依赖TIM PWM，不检查气泵或J60。
   * 先设置HOME角度，再启动PWM，避免启动瞬间输出错误脉宽。
   */
  g_gripper_runtime.servo_ready = 0U;

  Servo_SetAngle(GRIPPER_SERVO_HOME_ANGLE_DEG);

  if (Servo_Init() != HAL_OK)
  {
    return HAL_ERROR;
  }

	//0表示位于0°
  g_gripper_runtime.servo_flipped = 0U;
  g_gripper_runtime.servo_ready = 1U;
  return HAL_OK;
}

/**
 * @brief  检测左拨杆是否刚刚进入新档位。
 * @param  remote 最新遥控数据。
 * @return 0表示没有档位变化；1/2/3表示刚进入对应档位。
 * @note   不做软件消抖。首次取得有效数据时只记录当前档位，不执行动作。
 */
static uint8_t GripperSequence_UpdateSwitch(const Remote_Data *remote)
{
  uint8_t position = remote->s[REMOTE_SW_LEFT_S1];

  if ((position < 1U) || (position > 3U))
  {
    return 0U;
  }

  if (g_gripper_runtime.switch_valid == 0U)
  {
    g_gripper_runtime.switch_position = position;
    g_gripper_runtime.switch_valid = 1U;
    return 0U;
  }

  if (position != g_gripper_runtime.switch_position)
  {
    g_gripper_runtime.switch_position = position;
    return position;
  }

  return 0U;
}

/**
 * @brief  读取并转换光电门检测状态。
 * @note   运行状态中的 photo_level 是逻辑量：0=无物体，1=检测到物体。
 */
static void GripperSequence_UpdatePhoto(void)
{
  g_gripper_runtime.photo_level =
      (PhotoGate_ReadRawLevel() == PHOTO_GATE_OBJECT_LEVEL) ? 1U : 0U;
}

/* ========================== 公共接口 ========================== */

void GripperSequence_Init(void)
{
  /* 彼此独立，初始化气泵和舵机 */
  (void)GripperSequence_InitPump();
  (void)GripperSequence_InitServo();
}



void GripperSequence_Step(const Remote_Data *remote)
{
  uint8_t switch_edge;
  uint8_t switch_position;

  /* 遥控无效时只进入夹爪自身安全状态，保留各子系统就绪信息。 */
  if ((remote == NULL) || (remote->online == 0U))
  {
    GripperSequence_FailSafe();
    return;
  }

	//获取左拨杆刚切换的状态
  switch_edge = GripperSequence_UpdateSwitch(remote);
	//获取左摇杆当前状态
  switch_position = g_gripper_runtime.switch_position;

  //进入2档实现舵机0和180度两种状态的翻转，以3档作为空挡
  if ((switch_edge == 2U) && (g_gripper_runtime.servo_ready != 0U))
  {
    GripperSequence_ToggleServo();
  }

	//如果播放1档且此时夹爪闭合，那夹爪就会打开
  if (g_gripper_runtime.pump_ready != 0U)
  {
    if ((switch_position == 1U) && (g_gripper_runtime.pump_on == 0U))
    {
      GripperSequence_SetPumpOn(1U);
    }

    /* 每个气泵控制周期刷新一次光电门检测状态，供3档关泵判断使用。 */
    GripperSequence_UpdatePhoto();

    /*
     * 仅当拨杆处于3档、气泵正在工作且光电门检测到物体时关闭气泵。
     * 该动作只改变气泵输出，不改变舵机当前角度。
     */
    if ((switch_position == 3U) &&
        (g_gripper_runtime.pump_on != 0U) &&
        (g_gripper_runtime.photo_level != 0U))
    {
      GripperSequence_SetPumpOn(0U);
    }
  }
}

void GripperSequence_FailSafe(void)
{
  /* 舵机PWM继续输出，保持当前角度。 */
  g_gripper_runtime.switch_valid = 0U;
  g_gripper_runtime.switch_position = 0U;
}