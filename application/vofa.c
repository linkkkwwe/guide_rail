#include "vofa.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

/* JustFloat 帧尾：00 00 80 7F（小端 float = NaN，上位机用它做帧同步） */
static const uint8_t VOFA_TAIL[4] = {0x00u, 0x00u, 0x80u, 0x7Fu};

/* 发送缓冲：通道数据 + 帧尾，静态分配避免栈上反复开大数组 */
static uint8_t vofa_tx_buf[VOFA_MAX_CH * 4u + 4u];

HAL_StatusTypeDef Vofa_JustFloat(UART_HandleTypeDef *huart,
                                 const fp32 *data, uint16_t num)
{
    uint16_t len;

    if (num == 0u || num > VOFA_MAX_CH)
        return HAL_ERROR;

    len = num * 4u;
    memcpy(vofa_tx_buf, data, len);              /* float 原样拷贝，小端无需转换 */
    memcpy(&vofa_tx_buf[len], VOFA_TAIL, 4u);    /* 追加帧尾 */

    return HAL_UART_Transmit(huart, vofa_tx_buf, len + 4u, 10u);
}

void Vofa_FireWater(UART_HandleTypeDef *huart, const fp32 *data, uint16_t num)
{
    char line[VOFA_MAX_CH * 16u + 8u];
    uint16_t pos = 0u;
    uint16_t i;

    if (num == 0u || num > VOFA_MAX_CH)
        return;

    for (i = 0u; i < num; i++)
    {
        /* 保留 3 位小数，够看又不至于太长 */
        pos += (uint16_t)snprintf(&line[pos], sizeof(line) - pos,
                                  "%.3f%c", (double)data[i],
                                  (i == num - 1u) ? '\n' : ',');
        if (pos >= sizeof(line) - 1u)
            break;
    }

    HAL_UART_Transmit(huart, (uint8_t *)line, pos, 10u);
}
