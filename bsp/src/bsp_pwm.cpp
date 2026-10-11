//
// Created by cosmosmount on 2025/9/2.
//

#include "bsp_pwm.hpp"

/* ==================================================================== *
 * 方向 D · PWM 封装（TODO #2）
 *
 * 本工程用到两路 PWM，参数都由 CubeMX 定好（见 board/Core/Src/tim.c）：
 *   加热电阻：TIM10_CH1 = PF6，PSC=167 / ARR=999    -> 168 MHz / 168 / 1000 = 1 kHz
 *   舵机    ：TIM1_CH1  = PE9，PSC=167 / ARR=19999  -> 168 MHz / 168 / 20000 = 50 Hz
 * 两者都是 1 tick = 1 us：50 Hz 那路 20000 tick/周期，故 CCR 直接就是脉宽微秒数。
 * ==================================================================== */

/* 各功能的开关（§2 要求每个功能可单独验证）。
 * 加热与舵机都"启动通道"，但初值取安全侧：加热 0 %、舵机中位。*/
#define PWM_HEAT_ENABLE  1
#define PWM_SERVO_ENABLE 1

#define PWM_HEAT_TIM      (&htim10)
#define PWM_HEAT_CHANNEL  TIM_CHANNEL_1
#define PWM_SERVO_TIM     (&htim1)
#define PWM_SERVO_CHANNEL TIM_CHANNEL_1

/* 期望频率（Hz），与 CubeMX 里 TIMx 的 PSC/ARR 一致。
 * PWM_Init 里用 PWM_SetPeriod 再推导一遍周期，目的是让「秒 → ARR」这条换算
 * 每次上电都被真正执行一次：算对了 ARR 仍是 999 / 19999（行为不变），
 * 算错了 ARR 会跟着变，上板读寄存器立刻能看出来。*/
#define PWM_DERIVE_PERIOD   1
#define PWM_HEAT_FREQ_HZ    1000U
#define PWM_SERVO_FREQ_HZ   50U
#define PWM_SERVO_PERIOD_US (1000000U / PWM_SERVO_FREQ_HZ) ///< 50 Hz -> 20000 us
#define PWM_SERVO_CENTER_US 1500U                          ///< 中位脉宽 1.5 ms

/**
 * @brief 取定时器的实际输入时钟（Hz）。
 * @note APB 定时器在 APB 预分频不为 1 时，时钟是 PCLK 的 2 倍：
 *       本工程 APB1 = 42 MHz(/4) -> 定时器 84 MHz，APB2 = 84 MHz(/2) -> 定时器 168 MHz，
 *       两个域都要乘 2，故只需区分挂在哪条 APB 上。
 *       TIM2~7 / TIM12~14 挂 APB1，TIM1 / TIM8~11 挂 APB2。
 */
static uint32_t PWM_TimerClock(TIM_HandleTypeDef *htim)
{
    /* 用 *_BASE 常量做比较：TIMx 宏是 (TIM_TypeDef*) 指针，C++ 里不能用作 case 标签 */
    const uint32_t base = (uint32_t)htim->Instance;
    const bool on_apb2 = (base == TIM1_BASE) || (base == TIM8_BASE) ||
                         (base == TIM9_BASE) || (base == TIM10_BASE) || (base == TIM11_BASE);

    if (on_apb2)
    {
        const uint32_t div = ((RCC->CFGR & RCC_CFGR_PPRE2) == RCC_CFGR_PPRE2_DIV1) ? 1U : 2U;
        return HAL_RCC_GetPCLK2Freq() * div;
    }

    const uint32_t div = ((RCC->CFGR & RCC_CFGR_PPRE1) == RCC_CFGR_PPRE1_DIV1) ? 1U : 2U;
    return HAL_RCC_GetPCLK1Freq() * div;
}

void PWM_Init(void)
{
    /* 定时器本身的初始化由 CubeMX 的 MX_TIMx_Init 完成（main.c 里在我们之前调用）；
     * 这里只负责"启动要用的通道"，并显式给安全初值 —— 加热必须先摆到 0 再启动，
     * 否则会按 CubeMX 配的初始 Pulse 直接出功率。*/
#if PWM_DERIVE_PERIOD
    PWM_SetPeriod(PWM_HEAT_TIM, 1.0f / (float)PWM_HEAT_FREQ_HZ);
    PWM_SetPeriod(PWM_SERVO_TIM, 1.0f / (float)PWM_SERVO_FREQ_HZ);
#endif

#if PWM_HEAT_ENABLE
    PWM_SetDutyRatio(PWM_HEAT_TIM, 0.0f, PWM_HEAT_CHANNEL); // 0 % = 不加热
    PWM_Start(PWM_HEAT_TIM, PWM_HEAT_CHANNEL);
#endif

#if PWM_SERVO_ENABLE
    PWM_SetDutyRatio(PWM_SERVO_TIM, (float)PWM_SERVO_CENTER_US / (float)PWM_SERVO_PERIOD_US,
                     PWM_SERVO_CHANNEL); // 1.5 ms = 中位
    PWM_Start(PWM_SERVO_TIM, PWM_SERVO_CHANNEL);
#endif
}

void PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    if (htim == NULL)
    {
        return;
    }
    /* 启动失败（句柄未初始化 / 通道号非法）时把该通道停掉再返回。
     * 这里不调 Error_Handler()：PWM_Init 是在 main 的 bsp_Init() 里跑的，
     * 一旦死循环会把后面的自检和 ThreadX 全带走，故障反而更难定位；
     * 「退回安全状态」比「卡死」更符合本工程的一贯取舍（同 CAN_Transmit）。*/
    if (HAL_TIM_PWM_Start(htim, Channel) != HAL_OK)
    {
        (void)HAL_TIM_PWM_Stop(htim, Channel);
    }
}

void PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    if (htim == NULL)
    {
        return;
    }
    (void)HAL_TIM_PWM_Stop(htim, Channel);
}

void PWM_SetPeriod(TIM_HandleTypeDef *htim, float period_s)
{
    if ((htim == NULL) || !(period_s > 0.0f))
    {
        return;
    }

    const uint32_t psc = htim->Init.Prescaler;
    float ticks = period_s * (float)PWM_TimerClock(htim) / (float)(psc + 1U);

    /* 本工程两路定时器都是 16 位，ARR 必须落在 [0, 65535] */
    if (ticks < 1.0f)
    {
        ticks = 1.0f;
    }
    if (ticks > 65536.0f)
    {
        ticks = 65536.0f;
    }

    const uint32_t arr = (uint32_t)(ticks + 0.5f) - 1U;
    htim->Init.Period = arr;             // 句柄字段与寄存器保持一致，供后续换算使用
    __HAL_TIM_SET_AUTORELOAD(htim, arr); // AutoReloadPreload = DISABLE，立即生效
}

void PWM_SetDutyRatio(TIM_HandleTypeDef *htim, float dutyratio, uint32_t channel)
{
    if (htim == NULL)
    {
        return;
    }

    /* 先夹到 [0, 1]，把非法输入挡在寄存器之外 */
    if (!(dutyratio > 0.0f)) // 这样写同时挡住 NaN
    {
        dutyratio = 0.0f;
    }
    if (dutyratio > 1.0f)
    {
        dutyratio = 1.0f;
    }

    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    /* 满量程取 ARR+1：duty=1 时比较值大于 ARR，PWM1 模式下输出整周期为高，
     * 才是真正的 100%（若取 ARR，duty=1 仍会缺最后一个计数）。*/
    const uint32_t ccr = (uint32_t)(dutyratio * (float)(arr + 1U));
    __HAL_TIM_SET_COMPARE(htim, channel, ccr);
}
