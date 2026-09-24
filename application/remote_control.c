#include "remote_control.h"
#include "bsp_rc.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

/* 遥控器数据（中断里写，主循环读） */
RC_ctrl_t rc_ctrl;

/* 接收原始数据：一帧 18 字节，缓冲区给 36 字节，防止 DMA 传输越界 */
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/* 最近一次收到合法帧的时刻，以及"是否收到过帧"标志。
 * 用单独的标志而不是比较时间戳是否为 0：上电后 tick 很小，
 * 只比较时间戳会把"从未收到"误判成"刚收到"。 */
static volatile uint32_t rc_last_rx_tick;
static volatile uint8_t  rc_frame_received;

static void sbus_to_rc(const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

void remote_control_init(void)
{
    rc_last_rx_tick = 0U;
    rc_frame_received = 0U;
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

uint8_t RC_is_offline(void)
{
    if (!rc_frame_received)
        return 1U;

    return ((HAL_GetTick() - rc_last_rx_tick) > RC_OFFLINE_TIME_MS) ? 1U : 0U;
}

/* USART3 中断：本项目 CubeMX 里没有勾 USART3 中断，handler 定义在这里，
 * 绕过 HAL_UART_IRQHandler 自己处理——因为要在空闲中断里手动切换
 * DMA 双缓冲的目标缓冲区。 */
void USART3_IRQHandler(void)
{
    if (huart3.Instance->SR & UART_FLAG_RXNE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if (USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* 当前使用内存缓冲区 0 */

            /* 失效 DMA */
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            /* 接收长度 = 设定长度 - 剩余长度 */
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            /* 重新设定数据长度 */
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            /* 切到内存缓冲区 1 */
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

            /* 使能 DMA */
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
                rc_last_rx_tick = HAL_GetTick();
                rc_frame_received = 1U;
            }
        }
        else
        {
            /* 当前使用内存缓冲区 1 */

            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            /* 切回内存缓冲区 0 */
            hdma_usart3_rx.Instance->CR &= ~(DMA_SxCR_CT);

            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
                rc_last_rx_tick = HAL_GetTick();
                rc_frame_received = 1U;
            }
        }
    }
}

/**
 * @brief  DBUS 协议解析：18 字节 → RC_ctrl_t
 *
 * 字节布局：
 *   [0..4]   通道 0~2 的 11 位值（位域拼接）
 *   [5]      通道 3 + 两个拨杆
 *   [6..13]  鼠标 x/y/z + 左右键
 *   [14..15] 键盘位掩码
 *   [16..17] 通道 4（拨轮）
 */
static void sbus_to_rc(const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
        return;

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        /* 通道 0 */
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; /* 通道 1 */
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          /* 通道 2 */
                         (sbus_buf[4] << 10)) & 0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; /* 通道 3 */
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                       /* 右拨杆 */
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                  /* 左拨杆 */
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    /* 鼠标 X */
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    /* 鼠标 Y */
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  /* 鼠标 Z */
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  /* 左键 */
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  /* 右键 */
    rc_ctrl->key.v = (uint16_t)(sbus_buf[14] | (sbus_buf[15] << 8));        /* 键盘 */
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 /* 拨轮 */

    /* 通道值减中值 → 以中心为 0 */
    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}
