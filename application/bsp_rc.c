#include "bsp_rc.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

/**
 * @brief  配置 USART3 接收 DBUS 数据：DMA 双缓冲 + 串口空闲中断。
 *         移植自标准车 bsp/boards/bsp_rc.c，唯一改动是 NVIC 在这里手动使能。
 *
 *         CubeMX 里刻意不勾 USART3 中断：handler 我们自己定义在
 *         remote_control.c 里（绕过 HAL，因为要手动切 DMA 双缓冲），
 *         若 CubeMX 也生成一个 USART3_IRQHandler 会重复定义链接报错。
 *
 * @param  rx1_buf      内存缓冲区 0
 * @param  rx2_buf      内存缓冲区 1
 * @param  dma_buf_num  每个缓冲区的字节数
 */
void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    /* 使能 USART3 中断（CubeMX 未生成，这里手动做） */
    HAL_NVIC_SetPriority(USART3_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);

    /* 使能 DMA 接收请求 */
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    /* 使能空闲中断 */
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    /* 失效 DMA */
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while (hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
    /* 内存缓冲区 0 */
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    /* 内存缓冲区 1 */
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    /* 数据长度 */
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;
    /* 使能双缓冲区 */
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

    /* 使能 DMA */
    __HAL_DMA_ENABLE(&hdma_usart3_rx);
}

void RC_unable(void)
{
    __HAL_UART_DISABLE(&huart3);
}

void RC_restart(uint16_t dma_buf_num)
{
    __HAL_UART_DISABLE(&huart3);
    __HAL_DMA_DISABLE(&hdma_usart3_rx);

    hdma_usart3_rx.Instance->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_usart3_rx);
    __HAL_UART_ENABLE(&huart3);
}
