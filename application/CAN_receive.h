#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"
#include "struct_typedef.h"

/*
 * ==================== DJI GM6020 电机 CAN 协议 ====================
 * 控制帧：0x1FF 管拨码 ID 1~4（0x2FF 管 ID 5~8）。一帧 8 字节 =
 *         4 通道 × 2 字节，每个通道对应一个 ID 的电压（±30000）。
 *         本例两个电机拨 ID=1、2，故共用 0x1FF 一帧下发。
 * 反馈帧：GM6020 反馈 ID = 0x204 + 拨码ID（注意！C620 电调才是
 *         0x200 + ID，两者规则不同）。电机每 1ms 主动回传一帧。
 * 反馈帧 8 字节 = ecd(2B) + speed_rpm(2B) + given_current(2B)
 *               + temperature(1B) + 保留(1B)，全部大端。
 * ================================================================
 */

#define GM6020_COMMAND_ID       0x1FFU            /* 控制帧 ID（管 ID 1~4） */
#define YAW_ESC_ID              1U                /* Yaw 电调拨码 ID */
#define HORIZONTAL_ESC_ID       2U                /* 水平电调拨码 ID */
#define CAN_YAW_FEEDBACK_ID     (0x204U + YAW_ESC_ID)         /* = 0x205 */
#define CAN_HORIZONTAL_FEEDBACK_ID (0x204U + HORIZONTAL_ESC_ID) /* = 0x206 */

/* 电机索引：与 motor_measure[] 数组下标对应 */
typedef enum
{
    YAW_MOTOR = 0,      /**< Yaw 轴（左右摆动） */
    HORIZONTAL_MOTOR,   /**< 水平轴（移动） */
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
 * @brief 一帧 0x1FF 同时下发两个电机的电压。
 * @param yaw_voltage        Yaw 电机电压（±30000）。
 * @param horizontal_voltage 水平电机电压（±30000）。
 * @return HAL_OK 发送成功；否则发送失败（如邮箱满）。
 */
HAL_StatusTypeDef CAN_cmd_both(int16_t yaw_voltage, int16_t horizontal_voltage);

/**
 * @brief 判断两个电机反馈是否都在线（100ms 内有新反馈）。
 * @return 1 = 在线可闭环，0 = 掉线需下电。
 */
uint8_t CAN_motor_feedback_ready(void);

#endif
