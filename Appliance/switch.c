#include <stdio.h>

#include "switch.h"
#include "print.h"
#include "gd32f30x.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#define SHAKING_TIME 10
#define LONG_PRESS_TIME 100

#define KEY_EXTI_OFF() exti_interrupt_disable(EXTI_9);
#define KEY_EXTI_ON()  exti_interrupt_enable(EXTI_9);

#define keyENTER_CRITICAL_FROM_ISR() 	uint32_t ulReturn = taskENTER_CRITICAL_FROM_ISR()
#define keyEXIT_CRITICAL_FROM_ISR() 	taskEXIT_CRITICAL_FROM_ISR(ulReturn)

int8_t key_type = 0;

void set_key_type(int8_t type)
{
	 key_type = type;
}

int8_t get_key_type(void)
{
	 return key_type;
}

typedef enum{
	DOWN,
	PRESS,
	LOW_UP,
	HIGH
}key_state;

TimerHandle_t	Key_Timer_Handle;

static key_state state = DOWN;
static uint32_t press_time = 0;

void Key_CallBack_FMS(void)
{ 
	 switch(state)
	 {
		 case PRESS:
			 state = LOW_UP;
			 KEY_EXTI_ON();
		 break;
		 
		 case LOW_UP:
			 if(gpio_input_bit_get(GPIOB,GPIO_PIN_9) == RESET){
				  KEY_EXTI_OFF();
					press_time++;
			 }
			 else{
				 xTimerStop(Key_Timer_Handle,0);
				 state = DOWN;
			 }
			 KEY_EXTI_ON();
				 
		 break;
		 
		 case HIGH:
			 state = DOWN;
		   xTimerStop(Key_Timer_Handle,0);
		   KEY_EXTI_ON();
		 break;
		 
		 default:
			 printf("[Key_CallBack_FMS]error state = %d\r\n",state);
		 break;
	 }
}

void Key_Interrupt_FMS(void)
{
	 BaseType_t pxHigherPriorityTaskWoken;
	 
	 switch(state)
	 {
		 case DOWN:
			 xTimerStartFromISR(Key_Timer_Handle, &pxHigherPriorityTaskWoken);
			 portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
			 KEY_EXTI_OFF();
			 state = PRESS;
		 break;
		 
		 case LOW_UP:
		   if(press_time >= LONG_PRESS_TIME)
			 {
				  printf("long press %d ms\r\n",press_time*10);
			 }
			 else
			 {
				  printf("short press %d ms\r\n",press_time*10);
			 }
			 press_time = 0;
			 xTimerResetFromISR(Key_Timer_Handle, &pxHigherPriorityTaskWoken);
		   portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
		   KEY_EXTI_OFF();
			 state = HIGH;
		 break;

		 default:
			 printf("[Key_Interrupt_FMS]error state = %d\r\n",state);
		 break;			 

	 }
}

void switch_exti_init(void)
{
		/* enable the key clock */
		rcu_periph_clock_enable(RCU_GPIOB);
		rcu_periph_clock_enable(RCU_AF);
		
		/* configure button pin as input */
		gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
		
		/* enable and set key EXTI interrupt to the lowest priority */
		nvic_irq_enable(EXTI5_9_IRQn, 10U, 0U);

		/* connect key EXTI line to key GPIO pin */
		gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOB, GPIO_PIN_SOURCE_9);

		/* configure key EXTI line */
		exti_init(EXTI_9, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
		exti_interrupt_flag_clear(EXTI_9);
		exti_interrupt_enable(EXTI_9);
}

void EXTI5_9_IRQHandler(void)
{
    if (RESET != exti_interrupt_flag_get(EXTI_9)) {
			Key_Interrupt_FMS();
			exti_interrupt_flag_clear(EXTI_9);
    }
}

void KeyCallback(TimerHandle_t xTimer)
{
	 Key_CallBack_FMS();
}

void switch_init(void)
{
	Key_Timer_Handle = xTimerCreate((const char* )"KeyTimer", (TickType_t)SHAKING_TIME, (UBaseType_t)pdTRUE, (void*)1, (TimerCallbackFunction_t)KeyCallback);
	switch_exti_init();
}