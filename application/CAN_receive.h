#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"
#include "struct_typedef.h"

/*
 * ==================== CAN 协议：Yaw=GM6020，水平=M3508 ====================
 * GM6020：控制帧 0x1FF（管 ID 1~4），反馈帧 0x204+ID，电压 ±30000，直驱 1:1
 * M3508 ：控制帧 0x200（管 ID 1~4），反馈帧 0x200+ID，电流 ±16384，减速比 19.2
 * 两个电机用不同的控制帧，CAN_cmd_each 发两帧。
 * 反馈帧 8 字节 = ecd(2B) + speed_rpm(2B) + given_current(2B)
 *               + temperature(1B) + 保留(1B)，全部大端。
 * ========================================================================
 */

#define GM6020_COMMAND_ID      0x1FFU            /* GM6020 控制帧（管 ID 1~4） */
#define C620_COMMAND_ID        0x200U            /* C620 控制帧（管 ID 1~4） */
#define YAW_ESC_ID              1U                /* Yaw 电调拨码 ID（GM6020） */
#define HORIZONTAL_ESC_ID       2U                /* 水平电调拨码 ID（C620/M3508） */
#define CAN_YAW_FEEDBACK_ID     (0x204U + YAW_ESC_ID)          /* = 0x205 */
#define CAN_HORIZONTAL_FEEDBACK_ID (0x200U + HORIZONTAL_ESC_ID) /* = 0x202 */

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
 * @brief 分别下发两个电机的指令：Yaw 走 0x1FF（GM6020），水平走 0x200（C620）。
 * @param yaw_voltage        Yaw 电机电压（±30000，GM6020）。
 * @param horizontal_voltage 水平电机电流（±16384，C620/M3508）。
 * @return HAL_OK 两帧都发送成功；否则至少一帧失败。
 */
HAL_StatusTypeDef CAN_cmd_both(int16_t yaw_voltage, int16_t horizontal_voltage);

/**
 * @brief 判断两个电机反馈是否都在线（100ms 内有新反馈）。
 * @return 1 = 在线可闭环，0 = 掉线需下电。
 */
uint8_t CAN_motor_feedback_ready(void);

#endif
