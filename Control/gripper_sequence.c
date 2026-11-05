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

/* ========================== 运行状态 ========================== */

volatile GripperRuntime_t g_gripper_runtime;

/* ========================== 输出控制 ========================== */

/**
 * @brief  设置气泵状态。
 * @param  on 1：气泵打开、释放；0：气泵关闭、夹紧。
 */
static void GripperSequence_SetPumpOn(uint8_t on)
{
  uint8_t next_on = (on != 0U) ? 1U : 0U;

  if (g_gripper_runtime.pump_on != next_on)
  {
    Relay_Set(next_on);
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
	* 继电器初始化，低电平，不给气
   */
  Relay_Init();

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
 * @brief  读取光电门当前电平。
 */
static void GripperSequence_UpdatePhoto(void)
{
  g_gripper_runtime.photo_level =
      (PhotoGate_ReadRawLevel() == GPIO_PIN_SET) ? 1U : 0U;
}

/* ========================== 公共接口 ========================== */

void GripperSequence_Init(void)
{
  /* 彼此独立，初始化气泵和舵机 */
  (void)GripperSequence_InitPump();
  (void)GripperSequence_InitServo();
}

/**
 * @brief  执行一次夹爪控制周期。
 * @param  remote 最新遥控数据；为NULL或离线时进入夹爪安全状态。
 * @details
 *   本函数由控制任务每1ms调用一次，只完成一次输入采样和输出更新，
 *   不使用阻塞延时，也不会等待气泵或舵机完成机械动作。
 *
 *   单次调用按以下顺序处理：
 *   1. 检查遥控数据，无效时关闭气泵并清空拨杆状态；
 *   2. 检测左拨杆是否发生档位变化，并保存当前有效档位；
 *   3. 气泵首次就绪后主动打开一次；
 *   4. 仅在首次进入2档的周期翻转一次舵机；
 *   5. 在1档按电平打开气泵，读取光电门，并在3档检测到物体时关闭气泵。
 *
 * @note
 *   - 气泵和舵机分别依据各自的ready标志执行，互不阻塞；
 *   - 档位动作采用变化检测，拨杆停留在2档期间不会重复翻转舵机；
 *   - 光电门只参与气泵控制，关闭气泵不会改变舵机当前角度。
 */
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


  /*
   * 初始化完成后的第一次正常控制主动打开气泵。
   * 这是初始化动作，不是3档动作；此后3档本身不会再主动开泵。
   */
  if ((g_gripper_runtime.pump_ready != 0U) && (g_gripper_runtime.pump_initial_open_done == 0U))
  {
    /*
     * 该标志置位后不会再次执行初始化开泵；
     * 之后气泵只由拨杆和光电门逻辑控制。
     */
    GripperSequence_SetPumpOn(1U);
    g_gripper_runtime.pump_initial_open_done = 1U;
  }

  /*
   * 只在档位变化返回2时翻转一次。
   * 拨杆停在2档期间不会重复触发；3档仅用于重新允许下一次进入2档。
   */
  if ((switch_edge == 2U) && (g_gripper_runtime.servo_ready != 0U))
  {
    /*
     * ToggleServo内部会同时更新servo_flipped标志，
     * 并输出HOME角度或翻转角度。
     */
    GripperSequence_ToggleServo();
  }

  /*
   * 气泵相关处理整体受pump_ready保护；
   * 气泵未就绪时跳过采样和输出，但不会阻止已经就绪的舵机继续工作。
   */
  if (g_gripper_runtime.pump_ready != 0U)
  {
    /* 1档按电平打开气泵，只在气泵当前关闭时执行一次打开。 */
    if ((switch_position == 1U) && (g_gripper_runtime.pump_on == 0U))
    {
      /* 拨杆保持在1档时，pump_on会保持为1，不会重复产生开泵动作。 */
      GripperSequence_SetPumpOn(1U);
    }

    /* 每个气泵控制周期刷新一次光电门原始电平，供3档关泵判断使用。 */
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
  /* 气泵关闭；舵机PWM继续输出，保持当前角度。 */
  GripperSequence_SetPumpOn(0U);
  g_gripper_runtime.switch_valid = 0U;
  g_gripper_runtime.switch_position = 0U;
}