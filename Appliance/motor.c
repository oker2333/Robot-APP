#include <stdio.h>
#include <limits.h>

#include "motor.h"
#include "encoder.h"
#include "pwm.h"
#include "app_config.h"
#include "pid.h"

/*Configure PA6 PA7 as TIMER2 CH0(left) CH1(right),Configure PA5 as STBY */

/*
  rated revolution:293rpm
	w = 293*2*pi/60   <----------->   duty cycle
	v = 2*pi*r/T
	v = w*r --------   pwm = f(w)	======	pwm = f(v/r)   === x = f(v/r) * 1000
*/

/* 
  Wheel Diameter:65mm
	Distance Between Wheels:123mm
*/

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

/*
	duty cycle = (x / 1000)* 100%  = 50%
*/

/* 
	rated speed:366 +- 26	rpm
	load speed:293 +- 21 rpm
*/

#define MAX_LOAD_VELOCITY (PI*65.0f*300.0f/60.0f)			/*unit:mm/s*/
#define MAX_RATED_VELOCITY (PI*65.0f*360.0f/60.0f)			/*unit:mm/s*/

#define MAX_VELOCITY MAX_RATED_VELOCITY

#define set_left_velocity(x)  timer_channel_output_pulse_value_config(TIMER2,TIMER_CH_0,(PWM_TIM_PERIOD * x / MAX_VELOCITY));
#define set_right_velocity(x) timer_channel_output_pulse_value_config(TIMER2,TIMER_CH_1,(PWM_TIM_PERIOD * x / MAX_VELOCITY));


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
}

static int32_t left_velocity = 0;
static int32_t right_velocity = 0;

void motor_control(int32_t left,int32_t right)
{
	 if(left != left_velocity)
	 {
		  left_stop();
		  if(left >= 0){
				 set_left_velocity(left);
				 left_forward();
			}else{
				 set_left_velocity(-left);
				 left_backward();
			}
			left_velocity = left;
	 }
	 
	 if(right != right_velocity)
	 {
		  right_stop();
		  if(right >= 0){
				 set_right_velocity(right);
				 right_forward();
			}else{
				 set_right_velocity(-right);
				 right_backward();
			}
			right_velocity = right;
	 }
	 printf("[motor_control]left_velocity = %d,right_velocity = %d\r\n",left_velocity,right_velocity);
}

static void Left_PID_Controller(int left_measure,int left_target)
{
	 int ret = IncPIDCalc(left_target,left_measure);
   if(ret != INT_MAX)
	 {
			if(ret >= 0){
				 set_left_velocity(ret);
				 left_forward();
			}else{
				 set_left_velocity(-ret);
				 left_backward();
			}
	 }
}

static void Right_PID_Controller(int right_measure,int right_target)
{
	 int ret = IncPIDCalc(right_target,right_measure);
   if(ret != INT_MAX)
	 {
			if(ret >= 0){
				 set_right_velocity(ret);
				 right_forward();
			}else{
				 set_right_velocity(-ret);
				 right_backward();
			}
	 }
}

static int left_wheel_target,right_wheel_target;

void pid_motor_control(int left_target,int right_target)
{
	 left_wheel_target = left_target;
	 right_wheel_target = right_target;
	 Left_PID_Controller(get_left_velocity()*1000,left_wheel_target);
	 Right_PID_Controller(get_right_velocity()*1000,right_wheel_target);
}

void PID_Controller(void)
{
	 Left_PID_Controller(get_left_velocity()*1000,left_wheel_target);
	 Right_PID_Controller(get_right_velocity()*1000,right_wheel_target);
}
