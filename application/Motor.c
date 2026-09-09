#include "Motor.h"
#include <stddef.h>

#define ECD_RANGE       8192              /* 编码器一圈 = 8192 计数 */
#define ECD_HALF_RANGE  4096              /* 半圈阈值：ecd 跳变超过它即判定过零 */
#define DEG_PER_ECD     (360.0f / 8192.0f) /* 每计数对应的角度 */
#define M3508_GEAR_RATIO   19.2032f   /* 3591/187，转子转19.2圈=输出轴1圈 */
/**
 * @brief 限幅：把 value 限制在 [min_value, max_value]。
 */
static fp32 constrain_float(fp32 value, fp32 min_value, fp32 max_value)
{
    if (value > max_value)
        return max_value;
    if (value < min_value)
        return min_value;
    return value;
}

void motor_ctrl_init(motor_ctrl_t *motor, motor_axis_e axis)
{
    fp32 angle_pid[3];
    fp32 speed_pid[3];

    if (motor == NULL)
        return;

    /* 按轴选择 PID 参数与角度限位 */
    if (axis == MOTOR_AXIS_HORIZONTAL)
    {
        angle_pid[0] = HORIZONTAL_ANGLE_KP;
        angle_pid[1] = HORIZONTAL_ANGLE_KI;
        angle_pid[2] = HORIZONTAL_ANGLE_KD;
        // speed_pid[0] = HORIZONTAL_SPEED_KP;
        // speed_pid[1] = HORIZONTAL_SPEED_KI;
        // speed_pid[2] = HORIZONTAL_SPEED_KD;
        Pid_init(&motor->pid_angle, angle_pid,
                 HORIZONTAL_ANGLE_MAX_OUT, HORIZONTAL_ANGLE_MAX_IOUT);
        // Pid_init(&motor->pid_speed, speed_pid,
        //          HORIZONTAL_SPEED_MAX_OUT, HORIZONTAL_SPEED_MAX_IOUT);
        motor->use_cascade = 0U; /* 单环（角度环）控制：输出直接当电压，靠三角波轨迹实现匀速往返 */
        motor->min_angle = HORIZONTAL_MIN_ANGLE_DEG;
        motor->max_angle = HORIZONTAL_MAX_ANGLE_DEG;
    }
    else
    {
        /* Yaw 仅匀速模式：角度环不用（spin 分支直接跳过），只初始化速度环 */
        // angle_pid[0] = YAW_ANGLE_KP;
        // angle_pid[1] = YAW_ANGLE_KI;
        // angle_pid[2] = YAW_ANGLE_KD;
        speed_pid[0] = YAW_SPEED_KP;
        speed_pid[1] = YAW_SPEED_KI;
        speed_pid[2] = YAW_SPEED_KD;
        // Pid_init(&motor->pid_angle, angle_pid,
        //          YAW_ANGLE_MAX_OUT, YAW_ANGLE_MAX_IOUT);
        Pid_init(&motor->pid_speed, speed_pid,
                 YAW_SPEED_MAX_OUT, YAW_SPEED_MAX_IOUT);
        motor->use_cascade = 1U; /* 保持 1：motor_ctrl_set_spin 据此允许进入匀速模式 */
        motor->min_angle = YAW_MIN_ANGLE_DEG;
        motor->max_angle = YAW_MAX_ANGLE_DEG;
    }

    motor_ctrl_clear(motor);
}

void motor_ctrl_clear(motor_ctrl_t *motor)
{
    if (motor == NULL)
        return;

    Pid_clear(&motor->pid_angle);

    if (motor->use_cascade)
        Pid_clear(&motor->pid_speed);        
    
    motor->spin_speed_rpm = 0.0f;  /* 掉线清零：恢复后由上层重新设置匀速模式 */
    motor->total_rounds = 0;
    motor->offset_ecd = 0U;
    motor->last_ecd = 0U;
    motor->current_angle = 0.0f;
    motor->initialized = 0U;
}

void motor_ctrl_set_spin(motor_ctrl_t *motor, fp32 rpm, uint8_t use_limits)
{
    if (motor == NULL)
        return;

    /* 只有串级（带速度环）的电机支持匀速模式 */
    motor->spin_speed_rpm = motor->use_cascade ? rpm : 0.0f;
    /* spin_use_limits 已注释：限位匀速分支不再使用，该字段暂不赋值 */
    // motor->spin_use_limits = use_limits;
}

int16_t motor_ctrl_update(motor_ctrl_t *motor, fp32 target_angle,
                          uint16_t ecd, int16_t speed_rpm)
{
    int32_t ecd_diff;
    int32_t relative_ecd;
    fp32 target_speed;
    fp32 voltage;

    if (motor == NULL)
        return 0;

    /* 首帧反馈：把当前 ecd 记为 0° 参考点（上电位置），
     * 并跳过本次过零判断（还没有 last_ecd 可比） */
    if (!motor->initialized)
    {
        motor->offset_ecd = ecd;
        motor->last_ecd = ecd;
        motor->initialized = 1U;
        return 0;
    }

    /* 过零处理：ecd 是 0~8191 的环形量，跨过 8191↔0 时差值会
     * 突变成 ±8000 量级。差值 > 半圈(4096) 说明反向跨零 → 圈数-1；
     * 差值 < -4096 说明正向跨零 → 圈数+1。用 total_rounds 记录圈数，
     * 把环形 ecd 展开成单调递增的连续角度。 */
    ecd_diff = (int32_t)ecd - (int32_t)motor->last_ecd;
    motor->last_ecd = ecd;

    if (ecd_diff > ECD_HALF_RANGE)
        motor->total_rounds--;
    else if (ecd_diff < -ECD_HALF_RANGE)
        motor->total_rounds++;

    /* 连续角度 = (累计圈数 × 一圈 + 当前ecd - 上电ecd) × 每计数角度 */
    relative_ecd = motor->total_rounds * ECD_RANGE +
                   (int32_t)ecd - (int32_t)motor->offset_ecd;
    motor->current_angle = relative_ecd * DEG_PER_ECD/M3508_GEAR_RATIO;

    /* 匀速模式：跳过角度环，速度环直接跟踪恒定转速（仅 yaw 使用） */
    if (motor->spin_speed_rpm != 0.0f)
    {
        /* 限位匀速分支已注释：当前 yaw 不限位连续转，水平轴不走匀速 */
        // if (motor->spin_use_limits)
        // {
        //     if (motor->current_angle >= motor->max_angle &&
        //         motor->spin_speed_rpm > 0.0f)
        //         motor->spin_speed_rpm = -motor->spin_speed_rpm;
        //     else if (motor->current_angle <= motor->min_angle &&
        //              motor->spin_speed_rpm < 0.0f)
        //         motor->spin_speed_rpm = -motor->spin_speed_rpm;
        // }
        voltage = Pid_calc(&motor->pid_speed, speed_rpm, motor->spin_speed_rpm);
        return (int16_t)voltage;
    }

    /* 目标角度限幅，防止轨迹超出机械行程 */
    target_angle = constrain_float(target_angle, motor->min_angle, motor->max_angle);

    /* 外环：角度误差 → 目标转速 */
    target_speed = Pid_calc(&motor->pid_angle,
                            motor->current_angle, target_angle);
    /* 串级分支已注释：当前无电机走串级（yaw 匀速提前 return，水平轴单环） */
    // if (motor->use_cascade){
    //      voltage = Pid_calc(&motor->pid_speed, speed_rpm, target_speed);
    // }
    // else{
    //     voltage = target_speed;
    // }
    voltage = target_speed; /* 单环：角度环输出直接当电压 */

    return (int16_t)voltage;
}
