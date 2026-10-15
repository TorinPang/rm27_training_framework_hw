#ifndef BSP_USART_HPP
#define BSP_USART_HPP

#include "main.h"
#include "stm32f4xx_hal_uart.h"

enum USART_Mode
{
    USART_MODE_BLOCK = 0,
    USART_MODE_DMA = 1,
    USART_MODE_IT = 2
  };

/**
 * @brief 初始化串口。
 */
void USART_Init(void);

/**
 * @brief 初始化串口1。
 * @note 串口1用于调试信息输出。使用DMA发送和接收。
 */
void USART1_Init(void);

/**
 * @brief 初始化串口6。
 * @note 串口6用于和裁判系统通信。使用DMA发送和接收。
 */
void USART6_Init(void);

/**
 * @brief 串口发送数据。
 * @param huart 指向串口句柄的指针
 * @param pData 发送的数据
 * @param Size 数据长度
 * @param mode 发送模式
 */
void USART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, enum USART_Mode mode);

/**
 * @brief 串口接收数据。
 * @param huart 指向串口句柄的指针
 * @param pData 接收数据的缓冲区
 * @param Size 数据长度
 */
void USART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

/**
 * @brief 串口接收数据出口（弱符号，上层可实现同名函数覆盖）。
 * @note 由 bsp_usart.cpp 的空闲中断回调（HAL_UARTEx_RxEventCallback）在每收到一段
 *       新数据后调用，用来把数据交给上层（如命令解析、裁判系统解包）。
 *       ⚠ 本符号在 bsp/ 内**只有声明、没有默认定义**（与 CAN_RxCallback 不同）：
 *       上层若不提供强符号，调用会被链接器替换成 NOP（实测 `nop.w`），即数据被静默丢弃 ——
 *       不会跳飞崩溃，但也无人处理收到的数据，用到它时请务必给出实现。
 * @param huart 触发接收的串口句柄（USART1 或 USART6）
 * @param pData 指向本次新增数据的指针（指向 bsp 的静态接收缓冲区，需要保留请自行拷贝）
 * @param Size  本次新增数据的长度（循环 DMA 的累计量与缓冲回绕已由 bsp 处理）
 */
void USART_RxCallback(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

#endif // BSP_USART_HPP
