#ifndef MOTOR_H
#define MOTOR_H

#include "pid.h"

/*
 * ==================== 电机控制结构 ====================
 * Yaw：速度环单环（匀速模式），速度环 PID → 电压 → CAN
 * 水平轴：角度环单环，角度环 PID → 电压 → CAN
 * 串级（角度环→速度环）分支已注释，当前无电机使用。
 *
 * 两轴共用 pid.h 的位置式 PID，仅参数不同。
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

#define YAW_SPEED_KP          1000.0f   /* GM6020 速度环 Kp（DJI 官方参考值） */
#define YAW_SPEED_KI          0.0f      /* 纯 P，避免低速摩擦极限环 */
#define YAW_SPEED_KD          0.0f
#define YAW_SPEED_MAX_OUT  30000.0f     /* GM6020 电压上限 ±30000 */
#define YAW_SPEED_MAX_IOUT  5000.0f

/* ===== Yaw 持续匀速转动模式（默认启用） =====
 * 匀速模式下跳过角度环与 ±30° 限位，速度环直接跟踪恒定转速。
 * 正值 = 正向转动，负值 = 反向转动，转速按机械实际情况调。 */
#define YAW_SPIN_SPEED_RPM    60.0f    /* GM6020 直驱，60rpm 肉眼可见的慢转 */

/* ===== 水平电机 PID 参数（与 Yaw 相同，后续按机械结构单独调） ===== */
#define HORIZONTAL_ANGLE_KP         300.0f    /* 仿真调参：Kp=309 */
#define HORIZONTAL_ANGLE_KI         0.0f    /* 不用积分（避免三角波拐点 windup） */
#define HORIZONTAL_ANGLE_KD         0.0f    /* 仿真调参：Kd=6900，压拐点过冲 */
#define HORIZONTAL_ANGLE_MAX_OUT    16384.0f
#define HORIZONTAL_ANGLE_MAX_IOUT   5000.0f
/* 水平轴用单环（角度环）控制，速度环参数不需要，保持注释 */
// #define HORIZONTAL_SPEED_KP        1000.0f
// #define HORIZONTAL_SPEED_KI           1.0f
// #define HORIZONTAL_SPEED_KD           0.0f
// #define HORIZONTAL_SPEED_MAX_OUT  30000.0f
// #define HORIZONTAL_SPEED_MAX_IOUT  5000.0f

/* ===== 角度限位（相对上电位置） =====
 * Yaw：匀速连续转，限位不生效（角度环未启用）。
 * 水平轴：±30° 限位，靠 constrain_float 限制目标角度。 */
#define YAW_MIN_ANGLE_DEG          (-30.0f)
#define YAW_MAX_ANGLE_DEG            30.0f
#define HORIZONTAL_MIN_ANGLE_DEG   (-30.0f)
#define HORIZONTAL_MAX_ANGLE_DEG     30.0f

/* 轴类型：motor_ctrl_init 用它选择对应轴的 PID 参数与限位 */
typedef enum
{
    MOTOR_AXIS_YAW = 0,
    MOTOR_AXIS_HORIZONTAL
} motor_axis_e;

/* 单轴电机控制器：两个 PID + 编码器连续角度换算状态 */
typedef struct
{
    pid_type_def pid_speed;   /**< 速度环（内环）PID. */
    pid_type_def pid_angle;   /**< 角度环（外环）PID. */

    uint8_t  use_cascade;      /**< 是否启用串级 PID（角度环 → 速度环）. */

    fp32     spin_speed_rpm;   /**< 匀速模式转速（rpm）：非 0 时速度环直接跟踪该转速. */
    uint8_t  spin_use_limits;  /**< 匀速模式角度限位开关（当前未用，限位分支已注释）. */

    fp32     gear_ratio;      /**< 减速比：GM6020=1.0（直驱），M3508=19.2（有减速箱）. */
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
 * @param axis  轴类型：MOTOR_AXIS_YAW 或 MOTOR_AXIS_HORIZONTAL。
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
 * @brief 执行一次串级 PID 控制（每个控制周期调用一次）。
 * @param motor      控制器实例。
 * @param target_angle 目标角度（度，来自轨迹生成器）。
 * @param ecd        电机编码器反馈（0~8191）。
 * @param speed_rpm  电机转速反馈（rpm）。
 * @return 输出电压（±30000），给 CAN_cmd_both。
 */
int16_t motor_ctrl_update(motor_ctrl_t *motor, fp32 target_angle,
                          uint16_t ecd, int16_t speed_rpm);

/**
 * @brief 清零控制器状态（掉线重连/重新上电时调用）。
 *        下次 update 会把当前 ecd 记为新 0° 参考。
 */
void motor_ctrl_clear(motor_ctrl_t *motor);

#endif
