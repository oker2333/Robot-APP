#ifndef EXTI_H
#define EXTI_H

#include "gd32f30x_libopt.h"

#define  VL6180x_INT_Enable() exti_interrupt_enable(EXTI_1)
#define  VL6180x_INT_Disable() exti_interrupt_disable(EXTI_1)

void bsp_gpio_exti_init(void);

#endif
