#include "exti.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "vl6180x_api.h"
#include "vl6180x_sample_plat.h"
#include "app_config.h"

void bsp_gpio_exti_init(void)
{
		/* enable the key clock */
		rcu_periph_clock_enable(RCU_GPIOA);
		rcu_periph_clock_enable(RCU_AF);
		
		/* configure button pin as input */
		gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
		
		/* enable and set key EXTI interrupt to the lowest priority */
		nvic_irq_enable(EXTI1_IRQn, 10U, 0U);

		/* connect key EXTI line to key GPIO pin */
		gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOA, GPIO_PIN_SOURCE_1);

		/* configure key EXTI line */
		exti_init(EXTI_1, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
		exti_interrupt_flag_clear(EXTI_1);
		exti_interrupt_enable(EXTI_1);
	
}

void EXTI1_IRQHandler(void)
{
    if (RESET != exti_interrupt_flag_get(EXTI_1)) {
			BaseType_t pxHigherPriorityTaskWoken;
			xSemaphoreGiveFromISR(VL6180xSemaphore,&pxHigherPriorityTaskWoken);
			VL6180x_INT_Disable();
			exti_interrupt_flag_clear(EXTI_1);
			portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
    }
}
