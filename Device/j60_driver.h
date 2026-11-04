/**
 * @file    j60_driver.h
 * @brief   J60 系列 CAN 驱动器接口定义。
 * @details
 *   - 使用标准数据帧，建议 CAN 波特率为 1 Mbps。
 *   - CAN 标识符 bit0~bit3 为节点 ID，bit4 为收发标志位，bit5~bit10 为命令编号。
 *   - 主机发送的控制参数均为线性量化的无符号整数，本驱动负责物理量转换。
 *   - 所有节点共用一份反馈缓存，接收回调会在 CAN 中断中更新缓存。
 *
 * @note 本文件只定义协议接口和数据结构，不直接操作 STM32 CAN 外设。
 */

#ifndef J60_DRIVER_H
#define J60_DRIVER_H

#include "stm32f4xx_hal.h"

/* ========================== 协议常量 ========================== */

#define J60_NODE_MIN              0u          /* 节点 ID 最小值 */
#define J60_NODE_MAX              15u         /* 节点 ID 最大值（4 位） */
#define J60_NODE_COUNT            16u         /* 支持的节点数量 */
#define J60_CAN_BAUDRATE          1000000u    /* CAN 推荐波特率：1 Mbps */
#define J60_NODE_ID_MASK          0x0Fu       /* CAN ID 中的节点 ID 掩码 */
#define J60_CAN_FLAG_RX           0x10u       /* 反馈帧 bit4：驱动器 -> 主机 */
#define J60_DEFAULT_RX_TIMEOUT_MS 100u        /* 默认反馈超时时间 */

#define J60_CMD_DISABLE           1u          /* 关闭电机 */
#define J60_CMD_ENABLE            2u          /* 使能电机 */
#define J60_CMD_CONTROL           4u          /* 发送控制参数/接收电机状态 */
#define J60_CMD_SET_HOME          6u          /* 设置当前位置为零点 */
#define J60_CMD_SET_ID            8u          /* 设置节点 ID */
#define J60_CMD_SET_TIMEOUT       9u          /* 设置 CAN 超时时间 */
#define J60_CMD_SET_BANDWIDTH     10u         /* 设置电流环带宽 */
#define J60_CMD_SAVE_CONFIG       16u         /* 保存配置到非易失存储器 */
#define J60_CMD_ERROR_RESET       17u         /* 清除驱动器错误 */
#define J60_CMD_GET_STATUS        23u         /* 读取状态字 */
#define J60_CMD_GET_CONFIG        24u         /* 读取配置参数 */

/* ======================== 编码范围定义 ======================== */

#define J60_POS_MIN               (-40.0f)     /* 位置编码范围，单位：rad */
#define J60_POS_MAX               ( 40.0f)
#define J60_VEL_MIN               (-40.0f)     /* 速度编码范围，单位：rad/s */
#define J60_VEL_MAX               ( 40.0f)
#define J60_TORQUE_MIN            (-40.0f)     /* 力矩编码范围，单位：N·m */
#define J60_TORQUE_MAX            ( 40.0f)
#define J60_TEMP_MIN              (-20.0f)     /* 温度编码范围，单位：°C */
#define J60_TEMP_MAX              (200.0f)
#define J60_KP_MIN                (0.0f)       /* 刚度 Kp 编码范围 */
#define J60_KP_MAX                (1023.0f)
#define J60_KD_MIN                (0.0f)       /* 阻尼 Kd 编码范围 */
#define J60_KD_MAX                (51.0f)

/* ========================== 状态字位 ========================== */

#define J60_STATUS_OVERVOLTAGE     (1u << 0)   /* 电源过压 */
#define J60_STATUS_UNDERVOLTAGE    (1u << 1)   /* 电源欠压 */
#define J60_STATUS_OVERCURRENT     (1u << 2)   /* 过流 */
#define J60_STATUS_MOTOR_OVERTEMP  (1u << 3)   /* 电机过温 */
#define J60_STATUS_DRIVER_OVERTEMP (1u << 4)   /* 驱动器过温 */
#define J60_STATUS_CAN_TIMEOUT     (1u << 5)   /* CAN 通信超时 */
#define J60_STATUS_TEMP_SENSOR     (1u << 6)   /* 温度传感器故障 */

/* ========================== 数据结构 ========================== */

/**
 * @brief J60 节点的最新反馈与通信状态。
 */
typedef struct
{
    float position_rad;             /* 当前位置，单位：rad */
    float velocity_rad_s;           /* 当前速度，单位：rad/s */
    float torque_nm;                /* 当前力矩，单位：N·m */
    float temperature_c;            /* 当前温度，单位：°C */
    uint16_t status;                /* 状态字，具体含义见 J60_STATUS_* */
    uint32_t last_rx_ms;            /* 最近一次通过基础检查的帧时间，单位：ms */
    uint32_t rx_count;              /* 该节点累计收到的 CAN 帧数 */
    uint32_t control_rx_count;      /* 该节点累计收到的控制反馈帧数 */
    uint8_t temperature_is_motor;   /* 1：温度来自电机；0：温度来自驱动器 */
    uint8_t online;                 /* 收到过该节点可解析反馈或应答时为 1 */
    uint8_t last_command_ok;        /* 最近一次带应答命令是否执行成功 */
} J60_MotorData;

/**
 * @brief J60 驱动器的配置参数。
 */
typedef struct
{
    uint8_t node_id;                /* 驱动器当前节点 ID */
    uint8_t can_timeout_ms;         /* CAN 超时时间，单位：ms */
    uint16_t current_bandwidth_hz;  /* 电流环带宽，单位：Hz */
    uint16_t current_limit_raw;     /* 驱动器返回的原始电流限值 */
} J60_ConfigData;

/* 各节点最近一次收到的状态/配置缓存以及收发帧计数。 */
extern J60_MotorData j60_motor_data[J60_NODE_COUNT];
extern J60_ConfigData j60_config_data[J60_NODE_COUNT];
extern volatile uint32_t j60_tx_count;
extern volatile uint32_t j60_rx_count;

/* ========================== 初始化接口 ========================== */

/**
 * @brief  初始化 J60 驱动。
 * @param  hcan 已配置的 CAN 总线句柄。
 * @note   函数会清空全部反馈缓存和帧计数，并通过 BSP 层注册接收回调。
 */
void J60_Init(CAN_HandleTypeDef *hcan);

/* ========================== 命令接口 ========================== */

/*
 * 以下函数向指定节点发送不带数据或带少量配置数据的命令。
 * CAN 发送邮箱暂时占满时返回 HAL_BUSY，调用者可在后续周期重试。
 */

/** @brief 使能指定节点。 */
HAL_StatusTypeDef J60_Enable(uint8_t node_id);

/** @brief 关闭指定节点。 */
HAL_StatusTypeDef J60_Disable(uint8_t node_id);

/** @brief 将指定节点当前位置设置为零点。 */
HAL_StatusTypeDef J60_SetHome(uint8_t node_id);

/** @brief 清除指定节点的驱动器错误。 */
HAL_StatusTypeDef J60_ResetError(uint8_t node_id);

/** @brief 请求指定节点返回状态字。 */
HAL_StatusTypeDef J60_GetStatus(uint8_t node_id);

/** @brief 请求指定节点返回配置参数。 */
HAL_StatusTypeDef J60_GetConfig(uint8_t node_id);

/** @brief 将指定节点的当前配置保存到非易失存储器。 */
HAL_StatusTypeDef J60_SaveConfig(uint8_t node_id);

/**
 * @brief  设置 CAN 通信超时时间。
 * @param  node_id     目标节点 ID。
 * @param  timeout_ms  超时时间，单位：ms。
 */
HAL_StatusTypeDef J60_SetCanTimeout(uint8_t node_id, uint8_t timeout_ms);

/**
 * @brief  设置电流环带宽。
 * @param  node_id       目标节点 ID。
 * @param  bandwidth_hz  电流环带宽，单位：Hz。
 */
HAL_StatusTypeDef J60_SetBandwidth(uint8_t node_id, uint16_t bandwidth_hz);

/* ========================== 控制接口 ========================== */

/**
 * @brief  发送位置、速度、力矩控制帧。
 * @param  node_id          目标节点 ID。
 * @param  position_rad     目标位置，单位：rad。
 * @param  velocity_rad_s   目标速度，单位：rad/s。
 * @param  torque_nm        目标力矩，单位：N·m。
 * @param  kp               刚度系数。
 * @param  kd               阻尼系数。
 * @retval HAL_OK           控制帧已成功提交到 CAN 发送路径。
 * @retval HAL_BUSY         CAN 发送邮箱暂时占满，可稍后重试。
 * @retval HAL_ERROR        节点 ID 或参数无效，或发送失败。
 * @note   各参数超出协议范围时会自动限制到对应边界。
 */
HAL_StatusTypeDef J60_SendControl(uint8_t node_id,
                                  float position_rad,
                                  float velocity_rad_s,
                                  float torque_nm,
                                  float kp,
                                  float kd);

/**
 * @brief  发送速度控制帧。
 * @param  node_id          目标节点 ID。
 * @param  velocity_rad_s   目标速度，单位：rad/s。
 * @param  torque_nm        目标力矩，单位：N·m。
 * @param  kd               阻尼系数。
 * @note   位置增益 kp 固定为 0，位置字段为 0。
 */
HAL_StatusTypeDef J60_SendVelocity(uint8_t node_id,
                                   float velocity_rad_s,
                                   float torque_nm,
                                   float kd);

/* ========================== 反馈接口 ========================== */

/**
 * @brief  获取指定节点的最新电机反馈缓存。
 * @param  node_id 目标节点 ID。
 * @return 有效节点返回内部缓存地址；节点 ID 无效时返回 NULL。
 * @note   缓存会由 CAN 接收回调更新，调用者不得修改返回内容。
 */
const J60_MotorData *J60_GetMotorData(uint8_t node_id);

/**
 * @brief  获取指定节点的最新配置缓存。
 * @param  node_id 目标节点 ID。
 * @return 有效节点返回内部缓存地址；节点 ID 无效时返回 NULL。
 * @note   缓存会由 CAN 接收回调更新，调用者不得修改返回内容。
 */
const J60_ConfigData *J60_GetConfigData(uint8_t node_id);

/**
 * @brief  获取配置数据的一致快照。
 * @param  node_id   目标节点 ID。
 * @param  snapshot  输出缓冲区。
 * @retval HAL_OK    快照读取成功。
 * @retval HAL_ERROR 节点 ID 或输出指针无效。
 */
HAL_StatusTypeDef J60_GetConfigSnapshot(uint8_t node_id,
                                        J60_ConfigData *snapshot);

/**
 * @brief  根据最近一帧反馈时间判断节点是否在线。
 * @param  node_id     目标节点 ID。
 * @param  timeout_ms  在线超时时间，单位：ms；传 0 时使用默认值。
 * @return 1 表示在线，0 表示离线或节点 ID 无效。
 */
uint8_t J60_IsOnline(uint8_t node_id, uint32_t timeout_ms);

#endif /* J60_DRIVER_H */
