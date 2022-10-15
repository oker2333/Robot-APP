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

uint8_t key_type = 0;

typedef enum{
	DOWN,
	PRESS,
	LOW,
	UP,
	HIGH
}key_state;

TimerHandle_t	Key_Timer_Handle;

static key_state state = DOWN;

void key_state_update(void)
{
	if(state == LOW)
	{
		 state = UP;
	}
}

void Key_FMS(void)		//不可重入函数
{
	 static uint32_t press_time = 0;
	 BaseType_t pxHigherPriorityTaskWoken;
	 
	 switch(state)
	 {
		 case DOWN:
			 xTimerStartFromISR(Key_Timer_Handle, &pxHigherPriorityTaskWoken);
			 portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
			 KEY_EXTI_OFF();
			 state = PRESS;
//		   printf("DOWN\n");
		 break;
		 
		 case PRESS:
			 KEY_EXTI_ON();
			 state = LOW;
//			 printf("PRESS\n");
		 break;
		 
		 case LOW:
			 press_time++;
//		   printf("LOW %d\n",press_time);
		 break;
		 
		 case UP:
			 KEY_EXTI_OFF();
			 xTimerResetFromISR(Key_Timer_Handle, &pxHigherPriorityTaskWoken);
		   portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
			 state = HIGH;
//			 printf("UP\n");
		 break;
		 
		 case HIGH:
			 KEY_EXTI_ON();
		   xTimerStop(Key_Timer_Handle,0);
		   if(press_time >= LONG_PRESS_TIME)
			 {
				  printf("long press %d\r\n",press_time);
			 }
			 else
			 {
				  printf("short press %d\r\n",press_time);
			 }
			 press_time = 0;
			 state = DOWN;
//			 printf("HIGH\n\n");
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
			key_state_update();
			Key_FMS();
			exti_interrupt_flag_clear(EXTI_9);
    }
}

void KeyCallback(TimerHandle_t xTimer)
{
	 keyENTER_CRITICAL_FROM_ISR();
	 Key_FMS();
	 keyEXIT_CRITICAL_FROM_ISR();
}

void switch_init(void)
{
	Key_Timer_Handle = xTimerCreate((const char* )"KeyTimer", (TickType_t)SHAKING_TIME, (UBaseType_t)pdTRUE, (void*)1, (TimerCallbackFunction_t)KeyCallback);
	switch_exti_init();
}