#include "pwm.h"

void gpio_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);

    /*Configure PA6 PA7 (TIMER2 CH0 CH1) as alternate function*/
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_6);
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_7);
}

void timer_config(void)
{
    /* -----------------------------------------------------------------------
    TIMER1 configuration: generate 2 PWM signals with 2 different duty cycles:
    TIMER1CLK = SystemCoreClock / 120 = 1MHz

    TIMER1 channel0 duty cycle = (25/ 100)* 100%  = 25%
    TIMER1 channel1 duty cycle = (50/ 100)* 100%  = 50%
    ----------------------------------------------------------------------- */
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER2);

    timer_deinit(TIMER2);

    /* TIMER1 configuration */
    timer_initpara.prescaler         = PWM_TIM_PSC - 1;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = PWM_TIM_PERIOD - 1;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER2,&timer_initpara);

    /* CH0,CH1 and CH2 configuration in PWM mode */
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(TIMER2,TIMER_CH_0,&timer_ocintpara);
    timer_channel_output_config(TIMER2,TIMER_CH_1,&timer_ocintpara);

    /* CH0 configuration in PWM mode0,duty cycle 25% */
    timer_channel_output_pulse_value_config(TIMER2,TIMER_CH_0,0);
    timer_channel_output_mode_config(TIMER2,TIMER_CH_0,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER2,TIMER_CH_0,TIMER_OC_SHADOW_DISABLE);

    /* CH1 configuration in PWM mode0,duty cycle 50% */
    timer_channel_output_pulse_value_config(TIMER2,TIMER_CH_1,0);
    timer_channel_output_mode_config(TIMER2,TIMER_CH_1,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER2,TIMER_CH_1,TIMER_OC_SHADOW_DISABLE);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER2);
    /* auto-reload preload enable */
    timer_enable(TIMER2);
}

void bsp_pwm_out_init(void)
{
		gpio_config();
		timer_config();
}

