#include "motor.h"
#include "pwm.h"

/*Configure PA6 PA7 as TIMER2 CH0(left) CH1(right),Configure PA5 as STBY */

/*
Left:E0 E1
		 0  0  stop
		 0  1	 backward
		 1  0	 forward
			
Right:E0 E1
			0  0  stop
			0  1	backward
			1  0	forward

*/

static void STBY_Init(void)
{
		rcu_periph_clock_enable(RCU_GPIOA);
		gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_5);
	  gpio_bit_set(GPIOA,GPIO_PIN_5);
}

static void Direction_Init(void)
{
		rcu_periph_clock_enable(RCU_GPIOE);
		gpio_init(GPIOE, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_0);
	  gpio_init(GPIOE, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_1);
	  
	  gpio_init(GPIOE, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_2);
	  gpio_init(GPIOE, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_3);

		/*left motor E0 E1*/
	  gpio_bit_reset(GPIOE,GPIO_PIN_0);
		gpio_bit_reset(GPIOE,GPIO_PIN_1);

		/*right motor E2 E3*/
	  gpio_bit_reset(GPIOE,GPIO_PIN_2);
		gpio_bit_reset(GPIOE,GPIO_PIN_3);
}

static void left_stop(void)
{
	  gpio_bit_reset(GPIOE,GPIO_PIN_0);
		gpio_bit_reset(GPIOE,GPIO_PIN_1);
}

static void right_stop(void)
{
	  gpio_bit_reset(GPIOE,GPIO_PIN_2);
		gpio_bit_reset(GPIOE,GPIO_PIN_3);
}

static void left_forward(void)
{
	  gpio_bit_set(GPIOE,GPIO_PIN_0);
		gpio_bit_reset(GPIOE,GPIO_PIN_1);
}

static void right_forward(void)
{
	  gpio_bit_set(GPIOE,GPIO_PIN_2);
		gpio_bit_reset(GPIOE,GPIO_PIN_3);
}

static void left_backward(void)
{
	  gpio_bit_reset(GPIOE,GPIO_PIN_0);
		gpio_bit_set(GPIOE,GPIO_PIN_1);
}

static void right_backward(void)
{
	  gpio_bit_reset(GPIOE,GPIO_PIN_2);
		gpio_bit_set(GPIOE,GPIO_PIN_3);
}

void motor_init(void)
{
	 bsp_pwm_out_init();
	 Direction_Init();
	 left_stop();
	 right_stop();
}

void motor_control(int32_t left,int32_t right)
{
	 
}

void motor_info(int32_t* left,int32_t* right)
{
	 
}
