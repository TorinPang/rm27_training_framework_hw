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

/**
 * @brief 创建并启动 imu 线程（方向 C / TODO #10）。
 * @note 与 ControlThread_Init 相同：必须在 tx_application_define() 的调用链内调用，
 *       线程用静态栈，不占用 CubeMX 的 byte pool。
 */
void ImuThread_Init(void);

/**
 * @brief 创建监控用的 3 个信号量 + alive 线程（方向 E / TODO #10、#11）。
 * @note 必须在 App_ThreadX_Init() 里**第一个**调用 —— 本函数负责建信号量，
 *       而 control / imu 线程一开始运行就会立刻向这些信号量上报，
 *       信号量晚建会丢上报。线程用静态栈（512 B）、优先级 20（最低）。
 */
void AliveThread_Init(void);

/**
 * @brief 上报"系统初始化成功"（一次性，由 imu 线程在 BMI088 初始化成功后调用一次）。
 */
void Alive_NotifyInitOk(void);

/**
 * @brief 上报"imu 线程存活"（由 imu 线程每轮调用一次）。
 */
void Alive_NotifyImuAlive(void);

/**
 * @brief 上报"control 线程存活"（由 control 线程每轮调用一次）。
 * @note 这报的是"线程在跑"，与"电机是否在线"无关。
 */
void Alive_NotifyMotorAlive(void);

#ifdef __cplusplus
}
#endif

#endif // THREADS_HPP
