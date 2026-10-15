//
// 方向 E · LED 模块（TODO #11）
//

#ifndef BSP_LED_HPP
#define BSP_LED_HPP

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 三色灯的颜色（索引即引脚查表下标）。
 * @note 引脚在 board/Core/Inc/main.h（CubeMX 生成）：PH12=红、PH11=绿、PH10=蓝，
 *       均为 GPIO 输出且**高电平点亮**（经三极管驱动）。
 */
typedef enum
{
    LED_RED = 0,
    LED_GREEN = 1,
    LED_BLUE = 2,
} LED_Color_t;

/**
 * @brief LED 模块初始化：三色灯全灭。
 * @note 引脚方向已由 CubeMX 的 MX_GPIO_Init 配好，本模块不重复 HAL_GPIO_Init
 *       （避免生成代码与手写代码各配一份配置），故只能在 MX_GPIO_Init 之后调用。
 *       本工程由 bsp_Init() 保证这个先后顺序。
 */
void LED_Init(void);

/**
 * @brief 点亮/熄灭单个颜色的灯。
 * @param color 颜色（LED_RED / LED_GREEN / LED_BLUE）
 * @param on    非 0 = 亮，0 = 灭
 */
void LED_Set(LED_Color_t color, uint8_t on);

/**
 * @brief 一次设置三个灯（alive 线程的状态显示用，避免多次函数调用把状态分两步刷新）。
 * @param red / green / blue 非 0 = 亮，0 = 灭
 */
void LED_SetAll(uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief 三色灯全灭。
 */
void LED_Off(void);

#ifdef __cplusplus
}
#endif

#endif //BSP_LED_HPP
