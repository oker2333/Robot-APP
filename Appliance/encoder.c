#include <limits.h>
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
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;

		/*Configure PD12 PD13 (TIMER2 CH0 CH1) as alternate function*/
    timer_input_capture_config(TIMER3, TIMER_CH_0, &timer_icinitpara);
    timer_input_capture_config(TIMER3, TIMER_CH_1, &timer_icinitpara);

    timer_quadrature_decoder_mode_config(TIMER3, TIMER_ENCODER_MODE2, TIMER_IC_POLARITY_RISING, TIMER_IC_POLARITY_RISING);

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
			  print_info("now it adds up to %d circles\n",Timer_Update);
        timer_interrupt_flag_clear(TIMER3, TIMER_INT_FLAG_UP);
    }
}

#define CIRCLE_PULSE 		(13*4*30)
#define WHEEL_DISTANCE 	(168)
#define WHEEL_DIAMETER	(65)

#define PI (3.141592)

/*unit:m/s*/
float left_velocity = 0;
float right_velocity = 0;

bool right_velocity_measurement(uint32_t time_interval)
{
	 bool ret = true;
	 static uint32_t Record_Right_Pulse = 0;
	 static int8_t Record_Right_Direction = 0;
	 
	 static uint32_t Current_Right_Pulse = 0;
	 static int8_t Current_Right_Direction = 0;	
	
	 Current_Right_Pulse = TIMER_CNT(TIMER3);
	 Current_Right_Direction = ((TIMER_CTL0(TIMER3)&TIMER_CTL0_DIR) == TIMER_CTL0_DIR)?-1:1;
	 
	 if(Record_Right_Direction != Current_Right_Direction)
	 {
		  ret = false;
	 }
	 else
	 {
		  uint32_t Pulse_Difference = 0;
		  if(Current_Right_Direction == 1)
			{
			   if(Current_Right_Pulse < Record_Right_Pulse)
				 {
					  Pulse_Difference = Current_Right_Pulse + CIRCLE_PULSE - Record_Right_Pulse;
				 }
				 else
				 {
						Pulse_Difference = Current_Right_Pulse - Record_Right_Pulse;
				 }
			}
			else if(Current_Right_Direction == -1)
			{
			   if(Current_Right_Pulse > Record_Right_Pulse)
				 {
					  Pulse_Difference = Record_Right_Pulse + CIRCLE_PULSE - Current_Right_Pulse;
				 }
				 else
				 {
						Pulse_Difference = Record_Right_Pulse - Current_Right_Pulse;
				 }
			}
			
			right_velocity = Current_Right_Direction * PI * WHEEL_DIAMETER * Pulse_Difference / CIRCLE_PULSE / time_interval;
		  print_info("Current_Right_Pulse = %d,Record_Right_Pulse = %d,right_velocity = %f\n",Current_Right_Pulse,Record_Right_Pulse,right_velocity);
	 }

		Record_Right_Pulse = Current_Right_Pulse;
		Record_Right_Direction = Current_Right_Direction;
	 
	 return ret;
}

bool left_velocity_measurement(uint32_t time_interval)
{

}

float get_left_velocity(void)
{
	 return left_velocity;
}

float get_right_velocity(void)
{
	 return right_velocity;
}

