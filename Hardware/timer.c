#include "timer.h"
#include "gd32f30x.h"
#include "print.h"

/**
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none
  */
void switch_timer_init(void)
{
    /* TIMER0CLK = SystemCoreClock / 120 = 1MHz */
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER1);
    timer_deinit(TIMER1);

    /* TIMER0 configuration */
    timer_initpara.prescaler         = 119;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 1000;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER1,&timer_initpara);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER1);
	
		nvic_irq_enable(TIMER1_IRQn, 10U, 0U);
		timer_interrupt_enable(TIMER1, TIMER_INT_CH0);

    timer_enable(TIMER1);
}

void TIMER1_IRQHandler(void)
{
    if (RESET != timer_interrupt_flag_get(TIMER1,TIMER_INT_FLAG_CH0)) {
			 timer_interrupt_flag_clear(TIMER1,TIMER_INT_FLAG_CH0);
    }
}


