#include "encoder.h"
#include "gd32f30x_libopt.h"
#include "print.h"

/* https://blog.csdn.net/qq_48453068/article/details/126624658 */

#define ENCODER_TIM_PERIOD  (13*4*30-1)
#define ENCODER_TIM_PSC 0

static void encoderGPIOConfiguration(void)
{
    rcu_periph_clock_enable(RCU_TIMER3);
    rcu_periph_clock_enable(RCU_GPIOD);
		rcu_periph_clock_enable(RCU_AF);

		gpio_init(GPIOD, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_12 | GPIO_PIN_13);
}

static void encoderTimerConfiguration(void)
{
	  timer_parameter_struct timer_initpara;
    timer_ic_parameter_struct timer_icinitpara;
	
    timer_deinit(TIMER3);
		
		gpio_pin_remap_config(GPIO_TIMER3_REMAP, ENABLE);
	
    timer_initpara.period = ENCODER_TIM_PERIOD;
    timer_initpara.prescaler = ENCODER_TIM_PSC;
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER3, &timer_initpara);
		
		timer_channel_input_struct_para_init(&timer_icinitpara);
		/* TIMER1 CH0 input capture configuration */
    timer_icinitpara.icfilter    = 0x05;
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING | TIMER_IC_POLARITY_FALLING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;

		/*Configure PD12 PD13 (TIMER2 CH0 CH1) as alternate function*/
    timer_input_capture_config(TIMER3, TIMER_CH_0, &timer_icinitpara);
    timer_input_capture_config(TIMER3, TIMER_CH_1, &timer_icinitpara);

    timer_quadrature_decoder_mode_config(TIMER3, TIMER_ENCODER_MODE2, TIMER_IC_POLARITY_RISING | TIMER_IC_POLARITY_FALLING, TIMER_IC_POLARITY_FALLING | TIMER_IC_POLARITY_FALLING);

		timer_interrupt_flag_clear(TIMER3, TIMER_INT_FLAG_UP);
    timer_interrupt_enable(TIMER3, TIMER_INT_UP);
		
    nvic_irq_enable(TIMER3_IRQn, 10, 0);

    timer_enable(TIMER3);
}

void encoder_init(void)
{
		encoderGPIOConfiguration();
		encoderTimerConfiguration();
}

static uint32_t Timer_Update = 0;

void TIMER3_IRQHandler(void)
{
    if(SET == timer_interrupt_flag_get(TIMER3, TIMER_INT_FLAG_UP))
    {
				Timer_Update++;
        timer_interrupt_flag_clear(TIMER3, TIMER_INT_FLAG_UP);
				print_info("TIMER_CTL0_DIR = %d,TIMER_CNT = %d\n",(TIMER_CTL0(TIMER3)&TIMER_CTL0_DIR) == TIMER_CTL0_DIR,TIMER_CNT(TIMER3));
    }
}


