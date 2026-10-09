#include "bsp_usart.hpp"

/* 串口句柄定义在 board/Core/Src/usart.c 中 */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

/* 接收缓冲区大小；循环模式下缓冲区越大会越晚回绕，但占用内存也更多 */
#define USART_RX_BUF_SIZE   128U
/* 阻塞发送的超时时间（ms），不用 HAL_MAX_DELAY 以免死等 */
#define USART_TX_TIMEOUT_MS 100U

/* 两个串口的默认接收缓冲区，必须是静态/全局：DMA 使用期间不能释放或复用 */
static uint8_t usart1_rx_buf[USART_RX_BUF_SIZE];
static uint8_t usart6_rx_buf[USART_RX_BUF_SIZE];

/* 上一次空闲中断时 DMA 的累计写入位置，用于在循环模式下算出本次新增的数据 */
static uint16_t usart1_rx_pos = 0U;
static uint16_t usart6_rx_pos = 0U;

/* 上层可实现同名函数覆盖本弱符号，用来处理收到的数据（如命令解析、裁判系统解包） */
__attribute__((weak)) void USART_RxCallback(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

/**
 * @brief 启动某个串口的“空闲中断 + DMA”接收。
 * @note 本工程 RX DMA 配置为循环模式（Circular）：IDLE 事件不会结束接收，
 *       所以只需启动一次；同时关掉半传输中断，避免每收满半个缓冲区就回调一次。
 */
static void USART_StartReceive(UART_HandleTypeDef *huart, uint8_t *buf)
{
  if ((huart == NULL) || (huart->hdmarx == NULL) || (buf == NULL))
  {
    return;
  }

  if (HAL_UARTEx_ReceiveToIdle_DMA(huart, buf, USART_RX_BUF_SIZE) == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }
}

void USART_Init(void)
{
  USART1_Init();
  USART6_Init();
}

void USART1_Init(void)
{
  usart1_rx_pos = 0U;
  USART_StartReceive(&huart1, usart1_rx_buf);
}

void USART6_Init(void)
{
  usart6_rx_pos = 0U;
  USART_StartReceive(&huart6, usart6_rx_buf);
}

void USART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, enum USART_Mode mode)
{
  if ((huart == NULL) || (pData == NULL) || (Size == 0U))
  {
    return;
  }

  switch (mode)
  {
  case USART_MODE_BLOCK:
    HAL_UART_Transmit(huart, pData, Size, USART_TX_TIMEOUT_MS);
    break;

  case USART_MODE_IT:
    HAL_UART_Transmit_IT(huart, pData, Size);
    break;

  case USART_MODE_DMA:
  default:
    /* DMA 发送要求缓冲区在发送完成前一直有效，调用方需保证这一点 */
    if (huart->hdmatx != NULL)
    {
      HAL_UART_Transmit_DMA(huart, pData, Size);
    }
    break;
  }
}

/**
 * @brief 标准库输出的底层出口（覆盖 board/Core/Src/syscalls.c 中的弱符号）。
 * @note syscalls.c 的 _write() 会逐字节调用本函数，这里把它接到调试串口 USART1。
 *       因为是被逐字节调用，只能用阻塞发送，不能改成 DMA。
 */
extern "C" int __io_putchar(int ch)
{
  uint8_t c = (uint8_t)ch;
  USART_Transmit(&huart1, &c, 1U, USART_MODE_BLOCK);
  return ch;
}

void USART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  if ((huart == NULL) || (pData == NULL) || (Size == 0U) || (huart->hdmarx == NULL))
  {
    return;
  }

  HAL_UARTEx_ReceiveToIdle_DMA(huart, pData, Size);
  __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

/**
 * @brief 串口接收事件回调（HAL 弱函数，空闲中断/传输完成都会走到这里）。
 * @note 循环 DMA 模式下，第二个参数 Size 是“本次接收窗口内的累计写入量”，
 *       不是本次新增的量；因此用 usartX_rx_pos 记录上次位置，算出真正新增的区间
 *       后再交给上层（USART_RxCallback），这样即使一直收、缓冲区回绕也不会丢数据。
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint8_t *buf;
  uint16_t buf_size;
  uint16_t *pos;

  if (huart->Instance == USART1)
  {
    pos = &usart1_rx_pos;
  }
  else if (huart->Instance == USART6)
  {
    pos = &usart6_rx_pos;
  }
  else
  {
    return;
  }

  buf      = huart->pRxBuffPtr;   /* 当前正在使用的接收缓冲区 */
  buf_size = huart->RxXferSize;   /* 该缓冲区的长度 */

  if ((buf == NULL) || (buf_size == 0U))
  {
    return;
  }

  if (Size > *pos)
  {
    /* 缓冲区尚未回绕：新增区间为 [*pos, Size) */
    USART_RxCallback(huart, &buf[*pos], (uint16_t)(Size - *pos));
  }
  else if (Size < *pos)
  {
    /* 缓冲区已回绕：先交尾部 [*pos, 末尾)，再交头部 [0, Size) */
    USART_RxCallback(huart, &buf[*pos], (uint16_t)(buf_size - *pos));
    if (Size > 0U)
    {
      USART_RxCallback(huart, buf, Size);
    }
  }
  /* Size == *pos：没有新增数据，忽略 */

  /* 收满一整圈后位置归零，与 DMA 计数器的回绕保持一致 */
  *pos = (Size >= buf_size) ? 0U : Size;
}
