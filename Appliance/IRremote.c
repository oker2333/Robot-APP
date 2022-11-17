#include "IRremote.h"
#include "gd32f30x.h"

void ir_rx_init(void)
{
    timer_ic_parameter_struct timer_icinitpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_TIMER1);

    /*configure PA15 (TIMER1 CH0) as alternate function*/
    gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_15);

    timer_deinit(TIMER1);
	
		gpio_pin_remap_config(GPIO_TIMER1_FULL_REMAP, ENABLE);
	
    /* TIMER1 configuration */
    timer_initpara.prescaler         = 119;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 10000;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER1,&timer_initpara);

    /* TIMER1 CH3 input capture configuration */
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0;
    timer_input_capture_config(TIMER1,TIMER_CH_0,&timer_icinitpara);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER1);
    /* clear channel 0 interrupt bit */
    timer_interrupt_flag_clear(TIMER1,TIMER_INT_FLAG_CH0 | TIMER_INT_FLAG_UP);
    timer_interrupt_enable(TIMER1,TIMER_INT_CH0 | TIMER_INT_UP);
    /* channel 0 interrupt enable */
    nvic_irq_enable(TIMER1_IRQn, 10, 0);

    /* TIMER1 counter enable */
    timer_enable(TIMER1);
}

/**
  * @brief  This function handles TIMER1 interrupt request.
  * @param  None
  * @retval None
  */
void TIMER1_IRQHandler(void)
{
    if(SET == timer_interrupt_flag_get(TIMER1,TIMER_INT_FLAG_CH0))
    {
        if(gpio_input_bit_get(GPIOA, GPIO_PIN_15))
        {
						
        }

        timer_interrupt_flag_clear(TIMER1,TIMER_INT_FLAG_CH0);
    }
    if(SET == timer_interrupt_flag_get(TIMER1,TIMER_INT_FLAG_UP))
    {
        timer_interrupt_flag_clear(TIMER1,TIMER_INT_FLAG_UP);
    }
}
