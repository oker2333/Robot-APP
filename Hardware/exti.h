#ifndef EXTI_H
#define EXTI_H

#include "gd32f30x_libopt.h"

#define  VL6180x_INT_Enable() nvic_irq_enable(EXTI1_IRQn, 10U, 0U)
#define  VL6180x_INT_Disable() nvic_irq_disable(EXTI1_IRQn)

void bsp_gpio_exti_init(void);

#endif
