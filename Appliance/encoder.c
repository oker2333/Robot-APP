#include "encoder.h"
#include "gd32f30x_libopt.h"
 
#define ENCODER_TIM_PERIOD  0
#define ENCODER_TIM_PSC 0

static void encoderGPIOConfiguration(void)
{
    rcu_periph_clock_enable(RCU_TIMER2);
    rcu_periph_clock_enable(RCU_GPIOA);
		rcu_periph_clock_enable(RCU_AF);
	
		gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);
}

static void encoderTimerConfiguration(void)
{
	  timer_parameter_struct timer_initpara;
    timer_ic_parameter_struct timer_icinitpara;
	
    timer_deinit(TIMER2);

    timer_initpara.period = ENCODER_TIM_PERIOD;
    timer_initpara.prescaler = ENCODER_TIM_PSC;
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER2, &timer_initpara);

    timer_icinitpara.icfilter    = 0x05;
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;

    timer_input_capture_config(TIMER2, TIMER_CH_0, &timer_icinitpara);
    timer_input_capture_config(TIMER2, TIMER_CH_1, &timer_icinitpara);

    timer_quadrature_decoder_mode_config(TIMER2, TIMER_ENCODER_MODE2, TIMER_IC_POLARITY_RISING, TIMER_IC_POLARITY_FALLING);

		timer_slave_mode_select(TIMER2,TIMER_ENCODER_MODE2);

    timer_interrupt_enable(TIMER2, TIMER_INT_UP);
    nvic_irq_enable(TIMER2_IRQn, 1, 1);

		timer_auto_reload_shadow_enable(TIMER2);

    timer_enable(TIMER2);
}

void MotorEncoder_Init(void)
{
		encoderGPIOConfiguration();
		encoderTimerConfiguration();
}




