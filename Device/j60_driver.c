/**
 * @file    j60_driver.c
 * @brief   J60 系列 CAN 驱动器协议与基础驱动实现。
 * @details
 *   本文件负责：
 *   1. 将物理量线性量化并打包成 J60 控制帧；
 *   2. 通过 BSP CAN 层发送标准帧；
 *   3. 在接收回调中解析反馈帧、配置帧和命令应答；
 *   4. 缓存节点反馈并提供带中断保护的状态查询接口。
 *
 * @note J60总线初始化、通用电机组初始化和反馈判断在本驱动中实现；
 *       节点选择、控制参数和业务故障处理属于 Control 层。
 *       实际 CAN 外设初始化、滤波器和 FIFO 分发由 bsp_can 模块完成。
 */

#include "j60_driver.h"
#include "bsp_can.h"
#include "cmsis_os2.h"
#include <string.h>

/* ========================== 内部变量 ========================== */

/* J60 协议层保存所绑定的总线句柄，实际收发由 BSP_CAN 完成。 */
static CAN_HandleTypeDef *s_hcan;

/* 每个节点的最新反馈数据和配置数据。 */
J60_MotorData j60_motor_data[J60_NODE_COUNT];
J60_ConfigData j60_config_data[J60_NODE_COUNT];

/* 成功发送和接收的报文计数，可用于通信状态监控。 */
volatile uint32_t j60_tx_count;
volatile uint32_t j60_rx_count;

/* ======================== 内部函数声明 ======================== */

/* CAN 接收回调：解析一帧 J60 反馈并更新节点缓存。 */
static void j60_parse_frame(CAN_HandleTypeDef *hcan,
                            const CAN_RxHeaderTypeDef *header,
                            const uint8_t data[8]);

/* ======================== 临界区保护 ========================== */

/**
 * @brief  进入短临界区。
 * @return 进入前的中断状态（PRIMASK）。
 * @note   接收回调会修改反馈缓存，读取快照或在线状态时需短暂屏蔽中断。
 */
static uint32_t j60_enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

/**
 * @brief  退出短临界区并恢复进入前的中断状态。
 * @param  primask j60_enter_critical() 返回的 PRIMASK。
 */
static void j60_exit_critical(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

/* ======================== 数值编解码 ========================== */

/**
 * @brief  将浮点量限制在协议范围内，并线性量化为无符号整数。
 * @param  value 待编码的物理量。
 * @param  min   协议允许的最小值。
 * @param  max   协议允许的最大值。
 * @param  bits  量化后的有效位数。
 * @return 量化结果，范围为 0~(2^bits - 1)。
 *
 * 超出范围的输入会被饱和到边界；其余输入按线性比例量化并四舍五入。
 * 例如 16 bit 结果范围为 0~65535。
 */
static uint32_t j60_clamp_encode(float value, float min, float max, uint8_t bits)
{
    float ratio;
    uint32_t max_code = (1UL << bits) - 1UL;

    if (value <= min) {
        return 0UL;
    }
    if (value >= max) {
        return max_code;
    }

    ratio = (value - min) / (max - min);
    return (uint32_t)(ratio * (float)max_code + 0.5f);
}

/**
 * @brief  将协议中的无符号整数按线性关系还原为浮点量。
 * @param  value 原始编码值。
 * @param  min   编码值 0 对应的物理量。
 * @param  max   满量程编码对应的物理量。
 * @param  bits  编码有效位数。
 * @return 解码后的物理量。
 */
static float j60_decode(uint32_t value, float min, float max, uint8_t bits)
{
    return ((float)value * (max - min) / (float)((1UL << bits) - 1UL)) + min;
}

/* ========================== CAN 收发 ========================== */

/**
 * @brief  按 J60 协议拼接 CAN 标准帧标识符。
 * @param  command 命令编号。
 * @param  node_id 节点 ID。
 * @return 标准帧 ID：命令号位于 bit5~bit10，节点 ID 位于 bit0~bit3。
 *         主机发送帧的 bit4（收发标志位）保持为 0。
 */
static uint16_t j60_can_id(uint8_t command, uint8_t node_id)
{
    return (uint16_t)(((uint16_t)command << 5) |
                            (node_id & J60_NODE_ID_MASK));
}

/**
 * @brief  检查节点 ID 是否在驱动器支持的范围内。
 * @param  node_id 待检查的节点 ID。
 * @return 1 表示有效，0 表示无效。
 */
static uint8_t j60_node_valid(uint8_t node_id)
{
    return node_id <= J60_NODE_MAX;
}

/**
 * @brief  发送一帧 J60 CAN 报文。
 * @param  command 命令编号。
 * @param  node_id 目标节点 ID。
 * @param  payload 数据区指针；为 NULL 时使用全 0 数据。
 * @param  dlc     有效数据长度，最大为 8 字节。
 * @retval HAL_OK    报文已成功提交到 BSP CAN 发送路径。
 * @retval HAL_BUSY  发送邮箱暂时占满，可在下一周期重发。
 * @retval HAL_ERROR 参数无效或发送失败。
 */
static HAL_StatusTypeDef j60_send(uint8_t command, uint8_t node_id,
                                  const uint8_t *payload, uint8_t dlc)
{
    uint8_t zero_payload[8] = {0};
    uint8_t result;

    /* 节点号非法或数据长度超限时拒绝发送。 */
    if (!j60_node_valid(node_id) || dlc > 8u) {
        return HAL_ERROR;
    }

    result = BSP_CAN_SendStd(
        s_hcan,
        j60_can_id(command, node_id),
        (payload != NULL) ? payload : zero_payload,
        dlc);

    if (result == BSP_CAN_OK) {
        ++j60_tx_count;
        return HAL_OK;
    }

    /* 发送邮箱暂时占满属于瞬时拥塞，下一周期可再次提交。 */
    if (result == BSP_CAN_ERR_TX_FULL) {
        return HAL_BUSY;
    }

    return HAL_ERROR;
}

/* ========================== 初始化接口 ========================== */

HAL_StatusTypeDef J60_BusInit(CAN_HandleTypeDef *hcan)
{
    /* 保存句柄并清空上一轮运行留下的反馈、配置和计数。 */
    s_hcan = hcan;
    memset(j60_motor_data, 0, sizeof(j60_motor_data));
    memset(j60_config_data, 0, sizeof(j60_config_data));
    j60_tx_count = 0;
    j60_rx_count = 0;

    if (hcan == NULL) {
        return HAL_ERROR;
    }

    /* BSP 层负责滤波器、启动、发送和 FIFO 分发，J60 只注册解析函数。 */
    BSP_CAN_RegisterRxCallback(hcan, j60_parse_frame);
    return (BSP_CAN_Init(hcan) == BSP_CAN_OK) ? HAL_OK : HAL_ERROR;
}
/* ========================== 命令接口 ========================== */

/* 以下接口是对常用 J60 命令的简单封装。 */

HAL_StatusTypeDef J60_Enable(uint8_t node_id)
{
    return j60_send(J60_CMD_ENABLE, node_id, NULL, 0);
}

HAL_StatusTypeDef J60_Disable(uint8_t node_id)
{
    return j60_send(J60_CMD_DISABLE, node_id, NULL, 0);
}

HAL_StatusTypeDef J60_SetHome(uint8_t node_id)
{
    return j60_send(J60_CMD_SET_HOME, node_id, NULL, 0);
}

HAL_StatusTypeDef J60_ResetError(uint8_t node_id)
{
    return j60_send(J60_CMD_ERROR_RESET, node_id, NULL, 0);
}

HAL_StatusTypeDef J60_GetStatus(uint8_t node_id)
{
    return j60_send(J60_CMD_GET_STATUS, node_id, NULL, 0);
}

HAL_StatusTypeDef J60_GetConfig(uint8_t node_id)
{
    return j60_send(J60_CMD_GET_CONFIG, node_id, NULL, 0);
}

HAL_StatusTypeDef J60_SaveConfig(uint8_t node_id)
{
    return j60_send(J60_CMD_SAVE_CONFIG, node_id, NULL, 0);
}

HAL_StatusTypeDef J60_SetCanTimeout(uint8_t node_id, uint8_t timeout_ms)
{
    /* 超时时间以 1 字节原始值发送，单位由 J60 协议定义为 ms。 */
    return j60_send(J60_CMD_SET_TIMEOUT, node_id, &timeout_ms, 1);
}

HAL_StatusTypeDef J60_SetBandwidth(uint8_t node_id, uint16_t bandwidth_hz)
{
    /* 配置值按小端格式发送：低字节在前，高字节在后。 */
    uint8_t payload[2] = {
        (uint8_t)bandwidth_hz,
        (uint8_t)(bandwidth_hz >> 8)
    };

    return j60_send(J60_CMD_SET_BANDWIDTH, node_id, payload, 2);
}

/* ========================== 控制接口 ========================== */

HAL_StatusTypeDef J60_SendControl(uint8_t node_id, float position_rad,
                                  float velocity_rad_s, float torque_nm,
                                  float kp, float kd)
{
    uint16_t position;
    uint16_t velocity;
    uint16_t stiffness;
    uint8_t damping;
    uint16_t torque;
    uint8_t payload[8];

    /* 将控制量映射到 J60 协议规定的位宽和数值范围。 */
    position = (uint16_t)j60_clamp_encode(
        position_rad, J60_POS_MIN, J60_POS_MAX, 16);
    velocity = (uint16_t)j60_clamp_encode(
        velocity_rad_s, J60_VEL_MIN, J60_VEL_MAX, 14);
    stiffness = (uint16_t)j60_clamp_encode(kp, J60_KP_MIN, J60_KP_MAX, 10);
    damping = (uint8_t)j60_clamp_encode(kd, J60_KD_MIN, J60_KD_MAX, 8);
    torque = (uint16_t)j60_clamp_encode(
        torque_nm, J60_TORQUE_MIN, J60_TORQUE_MAX, 16);

    /*
     * 8 字节控制帧布局：
     *   payload[0..1]：位置，16 bit，小端；
     *   payload[2..3]：速度，14 bit，小端；
     *   payload[3] bit7~6：刚度低 2 位；
     *   payload[4]：刚度高 8 位；
     *   payload[5]：阻尼，8 bit；
     *   payload[6..7]：力矩，16 bit，小端。
     */
    payload[0] = (uint8_t)position;
    payload[1] = (uint8_t)(position >> 8);
    payload[2] = (uint8_t)velocity;
    payload[3] = (uint8_t)((velocity >> 8) & 0x3Fu);
    payload[3] |= (uint8_t)((stiffness & 0x03u) << 6);
    payload[4] = (uint8_t)(stiffness >> 2);
    payload[5] = damping;
    payload[6] = (uint8_t)torque;
    payload[7] = (uint8_t)(torque >> 8);

    return j60_send(J60_CMD_CONTROL, node_id, payload, 8);
}

HAL_StatusTypeDef J60_SendVelocity(uint8_t node_id,
                                   float velocity_rad_s,
                                   float torque_nm,
                                   float kd)
{
    /* kp=0 时不需要位置保持，位置字段传 0 不影响速度控制。 */
    return J60_SendControl(node_id, 0.0f, velocity_rad_s,
                           torque_nm, 0.0f, kd);
}

/* ========================== 电机组初始化 ========================== */

typedef struct
{
    uint32_t ack_base;
    uint32_t control_base;
    uint32_t status_base;
} J60_InitFeedbackBase_t;

static uint8_t j60_init_config_valid(const uint8_t *node_ids,
                                     uint8_t node_count,
                                     const J60_InitMotorsConfig_t *config)
{
    uint8_t i;
    uint8_t j;

    if ((node_ids == NULL) || (node_count == 0U) ||
        (node_count > J60_NODE_COUNT) || (config == NULL) ||
        (config->online_timeout_ms == 0U) ||
        (config->enable_ack_timeout_ms == 0U) ||
        (config->feedback_timeout_ms == 0U) ||
        (config->status_request_period_ms == 0U)) {
        return 0U;
    }

    for (i = 0U; i < node_count; ++i) {
        if (node_ids[i] > J60_NODE_MAX) {
            return 0U;
        }

        for (j = 0U; j < i; ++j) {
            if (node_ids[j] == node_ids[i]) {
                return 0U;
            }
        }
    }

    return 1U;
}

static HAL_StatusTypeDef j60_init_send_zero_control(
    const uint8_t *node_ids, uint8_t node_count)
{
    uint8_t i;

    for (i = 0U; i < node_count; ++i) {
        if (J60_SendControl(node_ids[i], 0.0f, 0.0f, 0.0f,
                            0.0f, 0.0f) != HAL_OK) {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

static HAL_StatusTypeDef j60_init_disable(const uint8_t *node_ids,
                                          uint8_t node_count)
{
    uint8_t i;

    for (i = 0U; i < node_count; ++i) {
        if (J60_Disable(node_ids[i]) != HAL_OK) {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

static HAL_StatusTypeDef j60_init_reset(const uint8_t *node_ids,
                                        uint8_t node_count)
{
    uint8_t i;

    for (i = 0U; i < node_count; ++i) {
        if (J60_ResetError(node_ids[i]) != HAL_OK) {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

static HAL_StatusTypeDef j60_init_enable(const uint8_t *node_ids,
                                         uint8_t node_count)
{
    uint8_t i;

    for (i = 0U; i < node_count; ++i) {
        if (J60_Enable(node_ids[i]) != HAL_OK) {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

static HAL_StatusTypeDef j60_init_request_status(
    const uint8_t *node_ids, uint8_t node_count)
{
    uint8_t i;

    for (i = 0U; i < node_count; ++i) {
        if (J60_GetStatus(node_ids[i]) != HAL_OK) {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

static void j60_init_safe_stop(const uint8_t *node_ids, uint8_t node_count)
{
    (void)j60_init_send_zero_control(node_ids, node_count);
    (void)j60_init_disable(node_ids, node_count);
}

static HAL_StatusTypeDef j60_init_record_ack_base(
    const uint8_t *node_ids, uint8_t node_count,
    J60_InitFeedbackBase_t *bases)
{
    J60_MotorData data;
    uint8_t i;

    for (i = 0U; i < node_count; ++i) {
        if (J60_GetMotorSnapshot(node_ids[i], &data) != HAL_OK) {
            return HAL_ERROR;
        }

        bases[node_ids[i]].ack_base = data.command_ack_count;
    }

    return HAL_OK;
}

static uint8_t j60_init_enable_ack_ready(
    const uint8_t *node_ids, uint8_t node_count,
    const J60_InitFeedbackBase_t *bases)
{
    J60_MotorData data;
    uint8_t i;

    for (i = 0U; i < node_count; ++i) {
        if (J60_GetMotorSnapshot(node_ids[i], &data) != HAL_OK) {
            return 0U;
        }

        if ((data.command_ack_count <= bases[node_ids[i]].ack_base) ||
            (data.last_ack_command != J60_CMD_ENABLE) ||
            (data.last_command_ok == 0U)) {
            return 0U;
        }
    }

    return 1U;
}

static HAL_StatusTypeDef j60_init_record_feedback_base(
    const uint8_t *node_ids, uint8_t node_count,
    J60_InitFeedbackBase_t *bases)
{
    J60_MotorData data;
    uint8_t i;

    for (i = 0U; i < node_count; ++i) {
        if (J60_GetMotorSnapshot(node_ids[i], &data) != HAL_OK) {
            return HAL_ERROR;
        }

        bases[node_ids[i]].control_base = data.control_rx_count;
        bases[node_ids[i]].status_base = data.status_rx_count;
    }

    return HAL_OK;
}

static uint8_t j60_init_feedback_ready(
    const uint8_t *node_ids, uint8_t node_count,
    const J60_InitMotorsConfig_t *config,
    const J60_InitFeedbackBase_t *bases)
{
    J60_MotorData data;
    uint8_t i;

    for (i = 0U; i < node_count; ++i) {
        if (J60_GetMotorSnapshot(node_ids[i], &data) != HAL_OK) {
            return 0U;
        }

        if ((data.control_rx_count <= bases[node_ids[i]].control_base) ||
            (data.status_rx_count <= bases[node_ids[i]].status_base) ||
            (data.status != 0U) ||
            (J60_IsOnline(node_ids[i], config->online_timeout_ms) == 0U)) {
            return 0U;
        }
    }

    return 1U;
}

static HAL_StatusTypeDef j60_init_wait_enable_ack(
    const uint8_t *node_ids, uint8_t node_count,
    const J60_InitFeedbackBase_t *bases, uint32_t timeout_ms)
{
    uint32_t start_ms = HAL_GetTick();

    for (;;) {
        if (j60_init_enable_ack_ready(node_ids, node_count, bases) != 0U) {
            return HAL_OK;
        }

        if ((uint32_t)(HAL_GetTick() - start_ms) >= timeout_ms) {
            return HAL_ERROR;
        }

        osDelay(1U);
    }
}

static HAL_StatusTypeDef j60_init_wait_feedback(
    const uint8_t *node_ids, uint8_t node_count,
    const J60_InitMotorsConfig_t *config,
    const J60_InitFeedbackBase_t *bases)
{
    uint32_t start_ms = HAL_GetTick();
    uint32_t last_status_request_ms = start_ms;

    (void)j60_init_request_status(node_ids, node_count);

    for (;;) {
        uint32_t now;

        (void)j60_init_send_zero_control(node_ids, node_count);
        now = HAL_GetTick();

        if (j60_init_feedback_ready(node_ids, node_count,
                                    config, bases) != 0U) {
            return HAL_OK;
        }

        if ((uint32_t)(now - start_ms) >= config->feedback_timeout_ms) {
            return HAL_ERROR;
        }

        if ((uint32_t)(now - last_status_request_ms) >=
            config->status_request_period_ms) {
            (void)j60_init_request_status(node_ids, node_count);
            last_status_request_ms = now;
        }

        osDelay(1U);
    }
}

HAL_StatusTypeDef J60_InitMotors(const uint8_t *node_ids,
                                 uint8_t node_count,
                                 const J60_InitMotorsConfig_t *config)
{
    J60_InitFeedbackBase_t bases[J60_NODE_COUNT] = {0};

    if (j60_init_config_valid(node_ids, node_count, config) == 0U) {
        return HAL_ERROR;
    }

    /* 先零控制并失能，保证初始化期间电机不带输出。 */
    if ((j60_init_send_zero_control(node_ids, node_count) != HAL_OK) ||
        (j60_init_disable(node_ids, node_count) != HAL_OK)) {
        j60_init_safe_stop(node_ids, node_count);
        return HAL_ERROR;
    }

    /* 记录应答基准，随后发送错误复位和使能。 */
    if (j60_init_record_ack_base(node_ids, node_count, bases) != HAL_OK) {
        j60_init_safe_stop(node_ids, node_count);
        return HAL_ERROR;
    }

    if ((config->reset_error != 0U) &&
        (j60_init_reset(node_ids, node_count) != HAL_OK)) {
        j60_init_safe_stop(node_ids, node_count);
        return HAL_ERROR;
    }

    if (j60_init_enable(node_ids, node_count) != HAL_OK) {
        j60_init_safe_stop(node_ids, node_count);
        return HAL_ERROR;
    }

    /* 只等待使能应答，不单独等待复位应答。 */
    if (j60_init_wait_enable_ack(node_ids, node_count, bases,
                                 config->enable_ack_timeout_ms) != HAL_OK) {
        j60_init_safe_stop(node_ids, node_count);
        return HAL_ERROR;
    }

    /* 记录反馈基准，只接受使能之后产生的新反馈。 */
    if (j60_init_record_feedback_base(node_ids, node_count, bases) != HAL_OK) {
        j60_init_safe_stop(node_ids, node_count);
        return HAL_ERROR;
    }

    if (j60_init_wait_feedback(node_ids, node_count,
                               config, bases) != HAL_OK) {
        j60_init_safe_stop(node_ids, node_count);
        return HAL_ERROR;
    }

    return HAL_OK;
}
/* ========================== 反馈解析 ========================== */

/**
 * @brief  解析一帧来自 J60 的反馈报文，并更新对应节点的缓存。
 * @param  hcan   接收该帧的 CAN 句柄（本回调中不使用）。
 * @param  header 接收帧头，提供标准帧 ID 和 DLC。
 * @param  data   接收数据区，BSP 层保证至少 8 字节可读。
 */
static void j60_parse_frame(CAN_HandleTypeDef *hcan,
                            const CAN_RxHeaderTypeDef *header,
                            const uint8_t data[8])
{
    uint8_t command = (uint8_t)((header->StdId >> 5) & 0x3Fu);
    uint8_t node_id = (uint8_t)(header->StdId & J60_NODE_ID_MASK);
    J60_MotorData *motor;

    (void)hcan;

    /*
     * 手册规定反馈帧 bit4 为 1，但部分 J60 固件/参考工程并未严格置位。
     * 为兼容现有硬件，命令号仍按 (StdId >> 5) & 0x3F 提取，不强制校验 bit4；
     * BSP 层已保证进入此回调的报文是 CAN 标准数据帧。
     */
    if (!j60_node_valid(node_id)) {
        return;
    }
    if (header->DLC > 8u) {
        return;
    }

    /* 节点号和 DLC 检查通过后，更新该节点的通信统计。 */
    motor = &j60_motor_data[node_id];
    ++j60_rx_count;
    ++motor->rx_count;
    motor->last_rx_ms = HAL_GetTick();

    switch (command) {
    case J60_CMD_CONTROL: {
        /* 控制反馈：位置和速度各 20 bit，力矩 16 bit，温度 7 bit。 */
        uint32_t position;
        uint32_t velocity;
        uint16_t torque;

        if (header->DLC < 8u) {
            return;
        }

        ++motor->control_rx_count;

        /* bit0~19 为位置，bit20~39 为速度，bit40~55 为力矩。 */
        position = (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
                   (((uint32_t)data[2] & 0x0Fu) << 16);
        velocity = ((uint32_t)data[2] >> 4) | ((uint32_t)data[3] << 4) |
                   ((uint32_t)data[4] << 12);
        torque = (uint16_t)data[5] | ((uint16_t)data[6] << 8);

        motor->position_rad = j60_decode(
            position, J60_POS_MIN, J60_POS_MAX, 20);
        motor->velocity_rad_s = j60_decode(
            velocity, J60_VEL_MIN, J60_VEL_MAX, 20);
        motor->torque_nm = j60_decode(
            torque, J60_TORQUE_MIN, J60_TORQUE_MAX, 16);

        /* 温度字节 bit0 表示温度来源，bit1~7 为温度编码值。 */
        motor->temperature_is_motor = data[7] & 0x01u;
        motor->temperature_c = j60_decode(
            data[7] >> 1, J60_TEMP_MIN, J60_TEMP_MAX, 7);
        motor->online = 1;
        break;
    }

    case J60_CMD_GET_STATUS:
        /* J60 协议使用小端字节序，状态字低字节在 data[0]。 */
        if (header->DLC < 2u) {
            return;
        }
        motor->status = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
        ++motor->status_rx_count;
        motor->online = 1;
        break;

    case J60_CMD_GET_CONFIG:
        /* 配置反馈中的字段均按小端格式排列。 */
        if (header->DLC < 8u) {
            return;
        }
        j60_config_data[node_id].node_id = data[2];
        j60_config_data[node_id].can_timeout_ms = data[3];
        j60_config_data[node_id].current_bandwidth_hz =
            (uint16_t)data[4] | ((uint16_t)data[5] << 8);
        j60_config_data[node_id].current_limit_raw =
            (uint16_t)data[6] | ((uint16_t)data[7] << 8);
        motor->online = 1;
        break;

    case J60_CMD_ENABLE:
    case J60_CMD_DISABLE:
    case J60_CMD_SET_TIMEOUT:
    case J60_CMD_SET_BANDWIDTH:
    case J60_CMD_SAVE_CONFIG:
    case J60_CMD_ERROR_RESET:
        /* 这些命令返回 1 字节结果码，0 表示执行成功。 */
        if (header->DLC < 1u) {
            return;
        }
        ++motor->command_ack_count;
        motor->last_ack_command = command;
        motor->last_command_ok = (data[0] == 0u);
        motor->online = 1;
        break;

    default:
        /* 未定义的命令号不更新具体反馈字段，仅保留通信统计。 */
        break;
    }
}

/* ========================== 反馈查询 ========================== */

HAL_StatusTypeDef J60_GetMotorSnapshot(uint8_t node_id,
                                       J60_MotorData *snapshot)
{
    uint32_t primask;

    if (!j60_node_valid(node_id) || snapshot == NULL) {
        return HAL_ERROR;
    }

    primask = j60_enter_critical();
    *snapshot = j60_motor_data[node_id];
    j60_exit_critical(primask);

    return HAL_OK;
}

const J60_MotorData *J60_GetMotorData(uint8_t node_id)
{
    /* 返回缓存地址，调用者可直接读取，但不应修改内容。 */
    return j60_node_valid(node_id) ? &j60_motor_data[node_id] : NULL;
}

const J60_ConfigData *J60_GetConfigData(uint8_t node_id)
{
    /* 返回内部配置缓存的只读地址，避免复制整份配置结构体。 */
    return j60_node_valid(node_id) ? &j60_config_data[node_id] : NULL;
}

HAL_StatusTypeDef J60_GetConfigSnapshot(uint8_t node_id,
                                        J60_ConfigData *snapshot)
{
    uint32_t primask;

    if (!j60_node_valid(node_id) || snapshot == NULL) {
        return HAL_ERROR;
    }

    /* 复制期间屏蔽接收中断，避免结构体字段来自不同反馈帧。 */
    primask = j60_enter_critical();
    *snapshot = j60_config_data[node_id];
    j60_exit_critical(primask);

    return HAL_OK;
}

uint8_t J60_IsOnline(uint8_t node_id, uint32_t timeout_ms)
{
    uint32_t now;
    uint32_t primask;
    uint32_t last_rx_ms;
    uint32_t rx_count;

    if (!j60_node_valid(node_id)) {
        return 0U;
    }

    now = HAL_GetTick();

    /* 同时读取计数和时间戳，避免并发更新造成不一致判断。 */
    primask = j60_enter_critical();
    last_rx_ms = j60_motor_data[node_id].last_rx_ms;
    rx_count = j60_motor_data[node_id].rx_count;
    j60_exit_critical(primask);

    if (timeout_ms == 0U) {
        timeout_ms = J60_DEFAULT_RX_TIMEOUT_MS;
    }

    /* 无符号减法可正确处理 HAL_GetTick() 计数回绕。 */
    return (rx_count != 0U &&
            (uint32_t)(now - last_rx_ms) <= timeout_ms)
               ? 1U
               : 0U;
}
