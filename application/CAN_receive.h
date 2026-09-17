#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"
#include "struct_typedef.h"

/*
 * ============ CAN 协议：Yaw = GM6020 直驱电机（单电机工程） ============
 * 本工程只控制 Yaw 电机（水平轴 M3508 在 GUID_RAIL 工程里）。
 * GM6020：内置驱动器，无需外接电调。控制帧 0x1FF（管 ID 1~4），
 *         反馈帧 0x200+ID（ID=1 → 0x205），电压指令 ±25000。
 *         6020 直驱无减速箱，gear_ratio=1.0，转子转速 = 输出轴转速。
 *         最大空载转速 320rpm。
 * 反馈帧 8 字节 = ecd(2B) + speed_rpm(2B) + given_current(2B)
 *               + temperature(1B) + 保留(1B)，全部大端。
 * ========================================================================
 */

#define M6020_COMMAND_ID       0x1FFU            /* GM6020 电压控制帧（管 ID 1~4） */
#define M6020_CURRENT_CMD_ID   0x1FEU            /* GM6020 电流控制帧（管 ID 1~4，需固件开启电流环） */
#define YAW_ESC_ID            1U                /* Yaw 电机拨码 ID（GM6020） */
#define CAN_YAW_FEEDBACK_ID   (0x204U + YAW_ESC_ID)          /* = 0x205 */

/* 电机索引：与 motor_measure[] 数组下标对应（本工程只有一个电机） */
typedef enum
{
    YAW_MOTOR = 0,      /**< Yaw 轴（左右摆动） */
    GUIDE_MOTOR_COUNT   /**< 电机数量（数组大小） */
} guide_motor_index_e;

/* 一帧反馈解析后的数据（由 CAN 接收中断在后台更新） */
typedef struct
{
    uint16_t ecd;          /**< 编码器原始值 0~8191，对应机械角度 0~360°. */
    int16_t  speed_rpm;    /**< 当前转速 rpm，正负表示方向. */
    int16_t  given_current;/**< 实际电流，有符号. */
    uint8_t  temperature;  /**< 电机温度 °C. */
    uint8_t  received;     /**< 是否收到过反馈（在线标志）. */
    uint32_t last_rx_tick; /**< 最近一次收到反馈的时间戳，用于离线判断. */
} motor_measure_t;

extern volatile motor_measure_t motor_measure[GUIDE_MOTOR_COUNT];

/**
 * @brief 下发 Yaw 电机电压指令：走 0x1FF（GM6020 电压控制帧）。
 * @param yaw_voltage Yaw 电压指令（±25000）。
 * @return HAL_OK 发送成功；否则失败。
 */
HAL_StatusTypeDef CAN_cmd_yaw(int16_t yaw_voltage);

/**
 * @brief 下发 Yaw 电机电流指令：走 0x1FE（GM6020 电流控制帧，仅电流环开启时生效）。
 * @param yaw_current Yaw 电流指令（±16384 对应 ±3A）。
 * @return HAL_OK 发送成功；否则失败。
 */
HAL_StatusTypeDef CAN_cmd_yaw_current(int16_t yaw_current);

/**
 * @brief 判断 Yaw 电机反馈是否在线（100ms 内有新反馈）。
 * @return 1 = 在线可闭环，0 = 掉线需下电。
 */
uint8_t CAN_motor_feedback_ready(void);

#endif
