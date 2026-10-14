//
// 方向 E · LED 模块（TODO #11）
//

#include "bsp_led.hpp"

#include "main.h" //< 板级生成：LED_R/G/B_Pin 与对应 GPIO 端口宏

/* ==================================================================== *
 * 方向 E · LED 模块（TODO #11）
 *
 * 三色灯是**板载**的，不需要任何外接设备，所以它一处承担两件事：
 *   ① 上电自检（main.c 的 BSP_SELF_TEST 开机闪一下，证明时钟/烧写/运行都通）；
 *   ② alive 线程的状态显示（初始化中 / 正常 / 初始化失败 / 线程死）。
 *
 * 引脚：PH12=红、PH11=绿、PH10=蓝，高电平点亮，已由 CubeMX 的 MX_GPIO_Init 配成
 * 推挽输出 —— 本模块**不再重复 HAL_GPIO_Init**，只负责写电平，避免生成代码与手写
 * 代码各配一份引脚配置。
 *
 * README 明确允许 LED 模块直接封装 HAL GPIO，故这里出现 HAL 调用不违反 AGENTS.md §2。
 * ==================================================================== */

/* 功能开关（§2：每个功能都要能单独验证）。置 0 = 灯完全关闭（所有写操作变成空操作），
 * 用于排查"灯效本身"干扰其它现象的情况。*/
#define LED_ENABLE 1

namespace
{
/* 颜色 -> 引脚 的查表（下标即 LED_Color_t 的值）。
 * GPIOH 是 ((GPIO_TypeDef *) GPIOH_BASE) 形式的地址常量，故这里是静态初始化。*/
GPIO_TypeDef *const kLedPort[3] = {LED_R_GPIO_Port, LED_G_GPIO_Port, LED_B_GPIO_Port};
const uint16_t kLedPin[3] = {LED_R_Pin, LED_G_Pin, LED_B_Pin};

/* 当前电平缓存：电平没变就不重复写 GPIO —— 少一次总线访问，也让"谁在驱动灯"更好追。*/
uint8_t LedState[3] = {0U, 0U, 0U};
} // namespace

void LED_Init(void)
{
#if LED_ENABLE
    /* 先摆全灭再交给别人驱动：避免上电瞬间残留电平让三灯齐亮。*/
    for (uint32_t i = 0U; i < 3U; ++i)
    {
        HAL_GPIO_WritePin(kLedPort[i], kLedPin[i], GPIO_PIN_RESET);
        LedState[i] = 0U;
    }
#endif
}

void LED_Set(LED_Color_t color, uint8_t on)
{
#if LED_ENABLE
    const uint32_t idx = static_cast<uint32_t>(color);
    if (idx >= 3U)
    {
        return; // 越界保护：宁可不动灯，也不要写错引脚
    }

    const uint8_t level = (on != 0U) ? 1U : 0U;
    if (LedState[idx] == level)
    {
        return;
    }

    LedState[idx] = level;
    HAL_GPIO_WritePin(kLedPort[idx], kLedPin[idx], level ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    (void)color;
    (void)on;
#endif
}

void LED_SetAll(uint8_t red, uint8_t green, uint8_t blue)
{
    LED_Set(LED_RED, red);
    LED_Set(LED_GREEN, green);
    LED_Set(LED_BLUE, blue);
}

void LED_Off(void)
{
    LED_SetAll(0U, 0U, 0U);
}
