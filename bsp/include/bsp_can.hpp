#ifndef BSP_CAN_HPP
#define BSP_CAN_HPP

#include "main.h"
#include "can.h"

/**
 * @brief 初始化CAN滤波器配置。
 * 设置CAN硬件的滤波器，用于优化接收数据的处理。
 * 更多信息，请参考原文，链接：https://blog.csdn.net/weixin_54448108/article/details/128570593
 */
void CAN_Init(void);

/**
 * @brief 发送CAN数据。
 * @param hcan 指向CAN句柄的指针，用于配置CAN传输。
 * @param StdId CAN消息的标准标识符
 * @param msg 发送的数据
 * @param len 数据长度
 * @note 该函数用于发送CAN数据，目前只支持标准帧
 */
void CAN_Transmit(CAN_HandleTypeDef *hcan, uint32_t Id, uint8_t *msg, uint16_t len);

/**
 * @brief CAN 接收数据出口（弱符号，上层可实现同名函数覆盖）。
 * @note 由 bsp_can.cpp 的 HAL_CAN_RxFifo0MsgPendingCallback 在收到数据标准帧后调用，
 *       用来把报文交给上层（本工程由 DJIMotorHandler.cpp 提供强符号做 ID→电机映射）。
 *       本符号在 bsp_can.cpp 内**有空实现**做默认，故上层未实现时调用也不会落到非法地址。
 * @param hcan  收到报文的 CAN 句柄（hcan1 / hcan2）
 * @param StdId 标准帧 ID
 * @param data  指向报文数据的指针（指向中断栈上的临时缓冲，需要保留请自行拷贝）
 */
void CAN_RxCallback(CAN_HandleTypeDef *hcan, uint32_t StdId, uint8_t *data);

#endif // BSP_CAN_HPP
