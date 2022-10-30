#ifndef PWM_H
#define PWM_H

#include "gd32f30x_libopt.h"

#define PWM_TIM_PERIOD  100
#define PWM_TIM_PSC 120

void bsp_pwm_out_init(void);

#endif
