/**
 * @file    remote_control.h
 * @brief   DJI DT7/DR16 遥控接收机驱动接口。
 * @details
 *   接收链路：
 *     USART1_RX (PB7) -> 板载硬件反相器 -> DMA2 Stream2 Channel4
 *     -> 18 字节遥控帧 -> 解析四个摇杆和两个拨杆。
 *
 *   UART 配置：
 *     100000 bps、9 Bits、Even、2 Stop Bits、Receive Only。
 */ 
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "stm32f4xx_hal.h"

/* ========================== 协议常量 ========================== */

/* 与原工程保持一致的遥控帧长度和中值。 */
#define REMOTE_FRAME_SIZE             18U

/* 每块 DMA 缓冲区可容纳两帧，硬件双缓冲共提供四帧空间。 */
#define REMOTE_DMA_BUFFER_SIZE        (REMOTE_FRAME_SIZE * 2U)

#define REMOTE_CH_COUNT                4U
#define REMOTE_SW_COUNT                2U
#define REMOTE_CH_VALUE_OFFSET         1024
#define REMOTE_STICK_HALF_RANGE       660.0f

/* ========================== 拨杆索引 ========================== */

/* 拨杆物理值：1=上档，3=中档，2=下档。 */
#define REMOTE_SW_LEFT_S1              0U
#define REMOTE_SW_RIGHT_S2             1U

/* ========================== 数据结构 ========================== */

/**
 * @brief 遥控数据快照。
 */
typedef struct
{
    int16_t ch[REMOTE_CH_COUNT]; /* 原始值减去 1024，回中约为 0 */
    uint8_t s[REMOTE_SW_COUNT];  /* 两个三段拨杆 */
    uint8_t online;              /* 当前数据是否在线 */
    uint32_t last_rx_ms;         /* 最近一次有效帧时间，单位：ms */
    uint32_t frame_count;        /* 累计有效帧数 */
    uint32_t error_count;        /* 长度错误累计数 */
} Remote_Data;

/**
 * @brief 遥控器运行对象。
 */
typedef struct
{
    UART_HandleTypeDef *huart;                 /* 遥控接收机串口句柄 */
    uint8_t buffer0[REMOTE_DMA_BUFFER_SIZE];   /* DMA 双缓冲内存块 0 */
    uint8_t buffer1[REMOTE_DMA_BUFFER_SIZE];   /* DMA 双缓冲内存块 1 */
    Remote_Data data;                          /* 最近一次解析结果 */

    /* 以下变量用于在线检测逻辑。 */
    volatile uint16_t online_cnt;              /* 在线倒计数，按 1 ms 递减 */
    volatile uint8_t rc_lost;                  /* 1：遥控器已掉线 */
    uint8_t initialized;                       /* 1：对象已完成初始化 */
} Remote_HandleTypeDef;

/* ========================== 公共接口 ========================== */

/**
 * @brief  初始化遥控器并启动双缓冲 DMA 接收。
 * @param  remote 遥控器对象。
 * @param  huart  已配置为 100000/9E2/Receive Only 的 USART1。
 * @retval HAL_OK    初始化成功。
 * @retval HAL_ERROR 参数错误。
 */
HAL_StatusTypeDef Remote_Init(Remote_HandleTypeDef *remote,
                            UART_HandleTypeDef *huart);

/**
 * @brief  清零遥控数据和在线状态。
 * @param  remote 遥控器对象。
 */
void Remote_Reset(Remote_HandleTypeDef *remote);

/**
 * @brief  执行旧工程 Remote::monitor() 的掉线计数。
 * @param  remote 遥控器对象。
 * @return 1 表示在线，0 表示离线或对象未初始化。
 * @note   建议在 GripperTask 中每 1 ms 调用一次。
 */
uint8_t Remote_Update(Remote_HandleTypeDef *remote);

/**
 * @brief  查询遥控器是否在线。
 * @param  remote 遥控器对象。
 * @return 1 表示在线，0 表示离线或对象未初始化。
 */
uint8_t Remote_IsOnline(const Remote_HandleTypeDef *remote);

/**
 * @brief  在临界区内复制一份完整遥控数据。
 * @param  remote   遥控器对象。
 * @param  snapshot 输出快照缓冲区。
 * @return 1 表示在线且复制成功，0 表示离线或参数错误。
 */
uint8_t Remote_GetSnapshot(const Remote_HandleTypeDef *remote,
                         Remote_Data *snapshot);

/**
 * @brief  把摇杆通道归一化到 [-1, 1] 并应用死区。
 * @param  channel  已减去中值的摇杆通道原始值。
 * @param  deadband 死区比例，内部限制到 0~1。
 * @return 归一化后的摇杆值。
 */
float Remote_NormalizeStick(int16_t channel, float deadband);


#endif /* REMOTE_CONTROL_H */
