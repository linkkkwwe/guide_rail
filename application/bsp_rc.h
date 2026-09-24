#ifndef BSP_RC_H
#define BSP_RC_H

#include "main.h"

/* DR16 接收机底层配置：DMA 双缓冲 + 串口空闲中断。
 * 移植自 RoboMaster 标准车 bsp/boards/bsp_rc.c。 */

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void RC_unable(void);
void RC_restart(uint16_t dma_buf_num);

#endif
