#ifndef MOTOR_H
#define MOTOR_H

#include "pid.h"

/*
 * ==================== 电机控制结构（Yaw 单电机工程：GM6020 直驱） ====================
 * 本工程只控制 Yaw 轴 GM6020（水平轴 M3508 在 GUID_RAIL 工程里）。
 * GM6020：内置驱动器，无需外接电调，CAN 直连。
 * Yaw：速度环单环（匀速模式），速度环 PID → 电压 → CAN
 * 串级（角度环→速度环）分支已注释，当前无使用。
 *
 * 串口蓝牙指令控制 Yaw 转速（一行一个数字，回车/换行结尾）：
 *   "60\n" = 60rpm 正转，"-120\n" = 反转，"0\n" = 停止
 *   超出 ±YAW_SPIN_MAX_RPM 自动限幅；6020 直驱，输出轴转速 = 转子转速
 * 无 IMU：位置反馈用电机自带编码器（上电位置为 0° 参考）。
 * ======================================================
 */

/* ===== Yaw 电机参数 =====
 * Yaw 仅匀速模式（spin），只用速度环，角度环已注释不用。
 * 若以后需要角度定位（如步兵瞄准），取消下面角度环注释即可。 */
// #define YAW_ANGLE_KP          20.0f
// #define YAW_ANGLE_KI           0.0f
// #define YAW_ANGLE_KD           0.0f
// #define YAW_ANGLE_MAX_OUT    200.0f
// #define YAW_ANGLE_MAX_IOUT     0.0f

#define YAW_SPEED_KP          60.0f       /* GM6020 速度环 Kp：先从小值试 */
#define YAW_SPEED_KI           2.0f
#define YAW_SPEED_KD           0.0f
#define YAW_SPEED_MAX_OUT  25000.0f      /* GM6020 电压上限 ±25000（手册规格） */
#define YAW_SPEED_MAX_IOUT 20000.0f      /* 积分上限：电压模式下维持 150rpm 需要 ~11700，
                                            原 10000 会把积分卡死 → 稳态上不去目标转速 */

/* 速度前馈（可选，默认关闭）：
 * 电压模式下维持转速主要靠抵消反电动势，约 25000/320rpm ≈ 78 每 rpm。
 * 这部分由前馈直接给出，PID 只负责补摩擦和扰动，响应明显更快。
 * 想启用就改成 60.0f（保守值），实测稳态电压 ÷ 稳态转速 可精确标定。 */
#define YAW_SPEED_FF           0.0f

/* ===== Yaw 匀速转速限幅（安全范围） =====
 * GM6020 空载极限约 320rpm，限到 300 留余量。
 * 蓝牙发送的是转子转速；6020 直驱无减速箱，输出轴 = 转子转速。
 * 正值 = 正向转动，负值 = 反向转动，0 = 停止。 */
#define YAW_SPIN_MAX_RPM    300     /* 转速限幅上限（rpm） */

/* ===== 角度限位（相对上电位置） =====
 * Yaw：匀速连续转，限位不生效（角度环未启用）。 */
#define YAW_MIN_ANGLE_DEG          (-30.0f)
#define YAW_MAX_ANGLE_DEG            30.0f

/* 轴类型：motor_ctrl_init 用它选择对应轴的 PID 参数与限位
 * （本工程只有 Yaw 轴，保留枚举形式与 GUID_RAIL 工程接口一致） */
typedef enum
{
    MOTOR_AXIS_YAW = 0
} motor_axis_e;

/* 单轴电机控制器：速度环 + 编码器连续角度换算状态 */
typedef struct
{
    pid_type_def pid_speed;   /**< 速度环（内环）PID. */
    pid_type_def pid_angle;   /**< 角度环（外环）PID. */

    uint8_t  use_cascade;      /**< 是否启用串级 PID（角度环 → 速度环）. */

    fp32     spin_speed_rpm;   /**< 匀速模式转速（rpm）：非 0 时速度环直接跟踪该转速. */
    uint8_t  spin_use_limits;  /**< 匀速模式角度限位开关（当前未用，限位分支已注释）. */

    fp32     gear_ratio;      /**< 减速比：GM6020=1.0（直驱）. */
    int32_t  total_rounds;    /**< 累计圈数（过零时 ±1）. */
    uint16_t offset_ecd;      /**< 上电时记录的 ecd，作为 0° 参考点. */
    uint16_t last_ecd;        /**< 上一次 ecd，用于计算过零跳变. */
    fp32     current_angle;   /**< 当前连续角度（度，相对上电位置）. */
    fp32     min_angle;       /**< 角度下限. */
    fp32     max_angle;       /**< 角度上限. */
    uint8_t  initialized;     /**< 首帧反馈是否已捕获（用于记录 offset_ecd）. */
} motor_ctrl_t;
/**
 * @brief 初始化电机控制器：按轴类型选 PID 参数与限位，并清零状态。
 * @param motor 控制器实例。
 * @param axis  轴类型：本工程只有 MOTOR_AXIS_YAW。
 */
void motor_ctrl_init(motor_ctrl_t *motor, motor_axis_e axis);

/**
  * @brief  设置/取消匀速转动模式（仅串级电机生效）。
  * @param  motor 控制器实例。
  * @param  rpm         匀速目标转速（rpm）；传 0 取消，恢复角度控制。
  * @param  use_limits  1 = 到达角度限位自动反向（匀速往返）；0 = 不限位连续转。
  */
void motor_ctrl_set_spin(motor_ctrl_t *motor, fp32 rpm, uint8_t use_limits);

/**
 * @brief 执行一次 PID 控制（每个控制周期调用一次）。
 * @param motor      控制器实例。
 * @param target_angle 目标角度（度，匀速模式下被忽略）。
 * @param ecd        电机编码器反馈（0~8191）。
 * @param speed_rpm  电机转速反馈（rpm）。
 * @return 输出电压（±30000），给 CAN_cmd_yaw。
 */
int16_t motor_ctrl_update(motor_ctrl_t *motor, fp32 target_angle,
                          uint16_t ecd, int16_t speed_rpm);

/**
 * @brief 清零控制器状态（掉线重连/重新上电时调用）。
 *        下次 update 会把当前 ecd 记为新 0° 参考。
 */
void motor_ctrl_clear(motor_ctrl_t *motor);

#endif
