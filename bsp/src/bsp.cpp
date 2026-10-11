#include "bsp.hpp"

#include "bsp_can.hpp"
#include "bsp_pwm.hpp"
#include "bsp_usart.hpp"

void bsp_Init() {
    USART_Init();
    CAN_Init();
    PWM_Init(); // 启动加热(TIM10_CH1, duty=0) 与舵机(TIM1_CH1, 中位) 两路 PWM
}
