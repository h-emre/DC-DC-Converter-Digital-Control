#ifndef __PWM_H_
#define __PWM_H_

#include "stm32f4xx_hal.h"
#include "main.h"
typedef enum
{
    CHANNEL2,

} Channels_e;

void pwm_init(void);
void pwm_enable(void);
void pwm_disable(void);
void pwm_set_duty_cycle(uint32_t duty, Channels_e channel);

#endif
