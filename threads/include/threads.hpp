#ifndef THREADS_HPP
#define THREADS_HPP

#include "tx_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 创建并启动 control 线程（方向 B / TODO #10）。
 * @note 必须在 ThreadX 内核启动前调用 —— 即 tx_application_define() 的调用链内
 *       （本工程由 app_threadx.c 的 App_ThreadX_Init 触发）。ThreadX 不允许在
 *       tx_kernel_enter() 之前于 main() 里直接创建线程。
 *       线程用静态栈，不占用 CubeMX 的 byte pool。
 */
void ControlThread_Init(void);

#ifdef __cplusplus
}
#endif

#endif // THREADS_HPP
