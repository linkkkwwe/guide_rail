#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "main.h"

/* DR16 接收机 DBUS 协议解析。
 * 移植自 RoboMaster 标准车 application/remote_control.c/h，
 * 去掉了 detect_task（离线检测）和 sbus_to_usart1（转发给上位机）两个依赖：
 *   - 离线检测改用 HAL_GetTick() 时间戳，见 RC_is_offline()
 *   - 转发给上位机的功能本项目不需要
 *
 * 帧格式：18 字节，100000bps / 8 数据位 / 偶校验 / 1 停止位。
 */

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* 摇杆解析后的取值范围（已减去中值 1024），用于归一化 */
#define RC_CH_VALUE_MAX_ABS     ((int16_t)660)

/* ----------------------- 拨杆档位定义 ------------------------------------
 * 注意：中档是 3 不是 2，比较时用下面的 switch_is_xxx 宏，别直接比大小。 */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)

/* ----------------------- 键鼠位定义 -------------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)

/* 遥控器离线判定时间：DR16 约 70Hz（14ms 一帧），100ms 等于允许连丢 7 帧 */
#define RC_OFFLINE_TIME_MS      100u

/* ----------------------- 数据结构 ---------------------------------------- */
typedef __packed struct
{
    __packed struct
    {
        int16_t ch[5];      /* 已减去中值 1024，范围约 ±660 */
        char s[2];          /* s[0]=右拨杆, s[1]=左拨杆（实车实测，标准车注释标反了） */
    } rc;
    __packed struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;
    __packed struct
    {
        uint16_t v;
    } key;

} RC_ctrl_t;

/* ----------------------- 对外接口 ---------------------------------------- */

/* 初始化：把两块 DMA 缓冲区交给底层，并使能接收 */
extern void remote_control_init(void);

/* 获取遥控器数据指针 */
extern const RC_ctrl_t *get_remote_control_point(void);

/* 遥控器是否离线：上电后从未收到帧、或超过 RC_OFFLINE_TIME_MS 没收到合法帧 */
extern uint8_t RC_is_offline(void);

#endif
