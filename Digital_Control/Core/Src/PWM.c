#include "PWM.h"
static void pwm_pin_configure();
void pwm_init(void){
    pwm_pin_configure();
    TIM3->PSC = 0;  // Set prescaler to 1 to have the timer clock at 16 MHz (no division)
    TIM3->ARR = 239;// 16 MHz / (15 µs * 16 MHz) = 240, so ARR = 240 - 1 = 239
    TIM3->CCR2 = 0; // Start with duty cycle 0
    // Configure Channel 2 for PWM mode 1 and enable preload
    TIM3->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;
    // Enable output for Channel 2
        TIM3->CCER |= TIM_CCER_CC2E;
}
void pwm_enable(void){
    // Enable Timer
    TIM3->CR1 |= TIM_CR1_CEN;
    TIM3->EGR |= TIM_EGR_UG;
}
void pwm_disable(void){
    // Disable Timer
    TIM3->CR1 &= ~(TIM_CR1_CEN);
}
void pwm_set_duty_cycle(uint32_t duty, Channels_e channel)
{
    switch (channel) {
        case CHANNEL2:
            TIM3->CCR2 = duty;
            break;
    }
}
static void pwm_pin_configure(){
	GPIO_InitTypeDef GPIO_InitStruct;
	    __HAL_RCC_TIM3_CLK_ENABLE();
	    __HAL_RCC_GPIOC_CLK_ENABLE();
	    // PC7 - TIMER3 - CH1
	    GPIO_InitStruct.Pin = GPIO_PIN_7;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}


