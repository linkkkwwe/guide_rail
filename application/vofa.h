#ifndef VOFA_H
#define VOFA_H

#include "struct_typedef.h"
#include "usart.h"   /* 提供 UART_HandleTypeDef 与 HAL_StatusTypeDef */

/*
 * ==================== VOFA+ 上位机通信（JustFloat 协议） ====================
 * VOFA+ 是一款串口波形上位机，官网 https://www.vofa.plus/
 *
 * JustFloat 帧格式：
 *   [ch0:4B float][ch1:4B float]...[chN-1:4B float][0x00 0x00 0x80 0x7F]
 *   - 每个通道一个 IEEE754 单精度 float，小端（STM32 原生格式，直接 memcpy）
 *   - 帧尾 4 字节固定为 00 00 80 7F（小端读出来是 NaN，上位机靠它断帧）
 *
 * 用法：
 *   1) CubeMX/Keil 里把 vofa.c 加入工程
 *   2) 主循环里按固定降频（如 100Hz）调用 Vofa_JustFloat(&huart1, buf, n)
 *   3) VOFA+ 里选协议 JustFloat，拖一个波形控件，勾选要显示的通道
 * =======================================================================
 */

/* 单帧最多通道数（改大就改大内部缓冲） */
#ifndef VOFA_MAX_CH
#define VOFA_MAX_CH   8
#endif

/**
 * @brief 发送一帧 JustFloat 数据（阻塞式，适合降频调用）。
 * @param huart  串口句柄，本项目用 &huart1（PA9 发送）。
 * @param data   待发送的 float 数组。
 * @param num    通道数，必须 <= VOFA_MAX_CH。
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef Vofa_JustFloat(UART_HandleTypeDef *huart,
                                 const fp32 *data, uint16_t num);

/**
 * @brief 发送一帧 FireWater 数据（CSV 文本，如 "12.0,35.0,0\n"）。
 *        方便人眼直接看串口助手，但占带宽大、解析慢，调试时临时用。
 * @param huart  串口句柄。
 * @param data   待发送的 float 数组。
 * @param num    通道数。
 */
void Vofa_FireWater(UART_HandleTypeDef *huart, const fp32 *data, uint16_t num);

#endif
