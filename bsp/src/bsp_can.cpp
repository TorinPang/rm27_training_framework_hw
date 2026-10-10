#include "bsp_can.hpp"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/**
 * @brief 初始化CAN滤波器配置。
 * 设置CAN硬件的滤波器，用于优化接收数据的处理。
 * 更多信息，请参考原文，链接：https://blog.csdn.net/weixin_54448108/article/details/128570593
 */
void CAN_Init(void)
{
    CAN_FilterTypeDef can_filter_st;                   ///< 定义过滤器结构体
    can_filter_st.FilterActivation = ENABLE;           ///< ENABLE使能过滤器
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;  ///< 设置过滤器模式--标识符屏蔽位模式
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT; ///< 过滤器的位宽 32 位
    can_filter_st.FilterIdHigh = 0x0000;               ///< ID高位
    can_filter_st.FilterIdLow = 0x0000;                ///< ID低位
    can_filter_st.FilterMaskIdHigh = 0x0000;           ///< 过滤器掩码高位
    can_filter_st.FilterMaskIdLow = 0x0000;            ///< 过滤器掩码低位

    can_filter_st.FilterBank = 0;                                      ///< 过滤器组-双CAN可指定0~27
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;                 ///< 与过滤器组管理的 FIFO
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);                      ///< HAL库配置过滤器函数
    HAL_CAN_Start(&hcan1);                                             ///< 使能CAN1控制器
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); ///< 使能CAN的各种中断

    can_filter_st.SlaveStartFilterBank = 14;                           ///< 双CAN模式下规定CAN的主从模式的过滤器分配，从过滤器为14
    can_filter_st.FilterBank = 14;                                     ///< 过滤器组-双CAN可指定0~27
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);                      ///< HAL库配置过滤器函数
    HAL_CAN_Start(&hcan2);                                             ///< 使能CAN2控制器
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); ///< 使能CAN的各种中断
}

void CAN_Transmit(CAN_HandleTypeDef *hcan, uint32_t Id, uint8_t *msg, uint16_t len)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t mailbox;

    // 参数保护：句柄、数据、长度（标准帧数据长度 0~8）都要合法
    if ((hcan == NULL) || (msg == NULL) || (len > 8U))
    {
        return;
    }
    // 只支持标准帧，11 位 ID 最大 0x7FF
    if (Id > 0x7FFU)
    {
        return;
    }
    // 邮箱满了就放弃本帧，避免覆盖尚未发出的报文（后续可改为等待/重试）
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
    {
        return;
    }

    tx_header.StdId = Id;
    tx_header.ExtId = 0U;
    tx_header.IDE   = CAN_ID_STD;    // 标准帧
    tx_header.RTR   = CAN_RTR_DATA;  // 数据帧
    tx_header.DLC   = len;
    tx_header.TransmitGlobalTime = DISABLE;

    (void)HAL_CAN_AddTxMessage(hcan, &tx_header, msg, &mailbox);
}

/**
 * @brief CAN 接收数据出口（弱符号）。
 * @note 与 USART_RxCallback 同一约定：上层可实现同名函数覆盖本弱符号，
 *       用来处理收到的 CAN 报文（如电机反馈解析）。这里只交出「哪条总线 + 标准帧 ID +
 *       8 字节数据」，具体的 ID→电机映射属于上层协议，不放进 bsp/，以免 bsp 反向依赖 modules/。
 * @param hcan 收到报文的 CAN 句柄
 * @param StdId 标准帧 ID
 * @param data 指向报文数据的指针（指向中断栈上的临时缓冲，需要保留请自行拷贝）
 */
__attribute__((weak)) void CAN_RxCallback(CAN_HandleTypeDef *hcan, uint32_t StdId, uint8_t *data)
{
    (void)hcan;
    (void)StdId;
    (void)data;
}

/**
 * @brief CAN RX FIFO0 收到报文的中断回调（HAL 弱函数）。
 * @note CAN_Init 里已对两个 CAN 使能 CAN_IT_RX_FIFO0_MSG_PENDING，
 *       HAL_CAN_IRQHandler 会在 FIFO0 有报文时调用本函数。中断上下文里只取帧、不发送、不阻塞。
 */
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
        return;
    }
    // 本工程只处理数据帧里的标准帧
    if ((rx_header.IDE != CAN_ID_STD) || (rx_header.RTR != CAN_RTR_DATA))
    {
        return;
    }

    CAN_RxCallback(hcan, rx_header.StdId, rx_data);
}
