#ifndef J60_DRIVER_H
#define J60_DRIVER_H

#include "stm32f4xx_hal.h"

/*
 * J60 CAN 通信协议：使用标准帧，推荐波特率为 1 Mbps。
 * CAN 标识符的低 4 位为节点 ID，第 5~10 位为命令编号。
 * 反馈帧会在 bit4 置 1，用于和主机发送帧区分。
 */
#define J60_NODE_MIN              0u          /* 节点 ID 最小值 */
#define J60_NODE_MAX              15u         /* 节点 ID 最大值（4 位） */
#define J60_NODE_COUNT            16u         /* 支持的节点数量 */
#define J60_CAN_BAUDRATE          1000000u    /* CAN 推荐波特率：1 Mbps */
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

#define J60_STATUS_OVERVOLTAGE    (1u << 0)  /* 电源过压 */
#define J60_STATUS_UNDERVOLTAGE   (1u << 1)  /* 电源欠压 */
#define J60_STATUS_OVERCURRENT    (1u << 2)  /* 过流 */
#define J60_STATUS_MOTOR_OVERTEMP (1u << 3)  /* 电机过温 */
#define J60_STATUS_DRIVER_OVERTEMP (1u << 4) /* 驱动器过温 */
#define J60_STATUS_CAN_TIMEOUT    (1u << 5)  /* CAN 通信超时 */
#define J60_STATUS_TEMP_SENSOR    (1u << 6)  /* 温度传感器故障 */

typedef struct
{
    float position_rad;        /* 当前位置，单位：rad */
    float velocity_rad_s;      /* 当前速度，单位：rad/s */
    float torque_nm;           /* 当前力矩，单位：N·m */
    float temperature_c;       /* 当前温度，单位：°C */
    uint16_t status;           /* 状态字，具体含义见 J60_STATUS_* */
    uint32_t last_rx_ms;       /* 最近一次有效反馈的系统毫秒时间。 */
    uint32_t rx_count;         /* 该节点累计收到的有效帧数。 */
    uint32_t control_rx_count; /* 该节点累计收到的控制反馈帧数。 */
    uint8_t temperature_is_motor; /* 1：温度来自电机，0：来自驱动器 */
    uint8_t online;            /* 已收到该节点有效反馈时为 1 */
    uint8_t last_command_ok;   /* 最近一次带应答命令是否执行成功 */
} J60_MotorData;

typedef struct
{
    uint8_t node_id;                 /* 驱动器当前节点 ID */
    uint8_t can_timeout_ms;          /* CAN 超时时间，单位：ms */
    uint16_t current_bandwidth_hz;   /* 电流环带宽，单位：Hz */
    uint16_t current_limit_raw;      /* 驱动器返回的原始电流限值 */
} J60_ConfigData;

/* 各节点最近一次收到的状态/配置缓存以及收发帧计数。 */
extern J60_MotorData j60_motor_data[J60_NODE_COUNT];
extern J60_ConfigData j60_config_data[J60_NODE_COUNT];
extern volatile uint32_t j60_tx_count;
extern volatile uint32_t j60_rx_count;

/* 初始化 J60 驱动，绑定 BSP CAN 总线并注册 J60 接收解析回调。 */
void J60_Init(CAN_HandleTypeDef *hcan);

/*
 * 以下函数向指定节点发送不带数据或带少量配置数据的命令。
 * CAN 发送邮箱暂时占满时返回 HAL_BUSY，调用者可在后续周期重试。
 */
HAL_StatusTypeDef J60_Enable(uint8_t node_id);
HAL_StatusTypeDef J60_Disable(uint8_t node_id);
HAL_StatusTypeDef J60_SetHome(uint8_t node_id);
HAL_StatusTypeDef J60_ResetError(uint8_t node_id);
HAL_StatusTypeDef J60_GetStatus(uint8_t node_id);
HAL_StatusTypeDef J60_GetConfig(uint8_t node_id);
/* 设置 CAN 超时（ms）、电流环带宽（Hz），以及保存当前配置。 */
HAL_StatusTypeDef J60_SetCanTimeout(uint8_t node_id, uint8_t timeout_ms);
HAL_StatusTypeDef J60_SetBandwidth(uint8_t node_id, uint16_t bandwidth_hz);
HAL_StatusTypeDef J60_SaveConfig(uint8_t node_id);

/*
 * 发送位置、速度、力矩控制帧：
 * position_rad：目标位置，单位 rad；velocity_rad_s：目标速度，单位 rad/s；
 * torque_nm：目标力矩，单位 N·m；kp：刚度系数；kd：阻尼系数。
 * 各参数超出协议范围时会自动限制到对应边界。
 */
HAL_StatusTypeDef J60_SendControl(uint8_t node_id,
                                   float position_rad,
                                   float velocity_rad_s,
                                   float torque_nm,
                                   float kp,
                                   float kd);

/*
 * 速度控制快捷接口：位置增益设为 0，只给速度、力矩和阻尼。
 * 传送带轮适合先用该接口调试，确认方向后再调整 kd。
 */
HAL_StatusTypeDef J60_SendVelocity(uint8_t node_id,
                                   float velocity_rad_s,
                                   float torque_nm,
                                   float kd);

/* 获取指定节点的最新电机反馈/配置数据；节点 ID 无效时返回 NULL。 */
const J60_MotorData *J60_GetMotorData(uint8_t node_id);
/* 获取指定节点的最新驱动器配置；节点 ID 无效时返回 NULL。 */
const J60_ConfigData *J60_GetConfigData(uint8_t node_id);



/* 获取配置数据的一致快照。 */
HAL_StatusTypeDef J60_GetConfigSnapshot(uint8_t node_id,
                                        J60_ConfigData *snapshot);

/* 根据最近一帧反馈的时间判断节点是否在线。 */
uint8_t J60_IsOnline(uint8_t node_id, uint32_t timeout_ms);

#endif /* J60_DRIVER_H */


