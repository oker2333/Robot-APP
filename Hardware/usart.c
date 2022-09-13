#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "fifo.h"
#include "gd32f30x_libopt.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

static SemaphoreHandle_t UsartxSemaphore;

static void usart_config(uint32_t baudval)
{
		rcu_periph_clock_enable(RCU_GPIOA);	//enable GPIO clock, PA9/PA10
		rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_USART0);	//enable USART clock
	
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);	//PA9--TX0
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);	//PA10--RX0

    usart_deinit(USART0);
    usart_baudrate_set(USART0, baudval);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_interrupt_enable(USART0, USART_INT_IDLE);
		usart_interrupt_enable(USART0, USART_INT_TC);
    usart_enable(USART0);
}

uint8_t rxbuffer[RX_BUFFER_LEN];

static void usart_dma_config(void)
{
    dma_parameter_struct dma_init_struct;
    /* enable DMA0 clock */
    rcu_periph_clock_enable(RCU_DMA0);
    
    /* deinitialize DMA channel3(USART0 tx) */
    dma_deinit(DMA0, DMA_CH3);
    dma_struct_para_init(&dma_init_struct);

    dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_addr = NULL;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = 0;
    dma_init_struct.periph_addr = USART0_DATA_ADDRESS;
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA0, DMA_CH3, &dma_init_struct);
    
    /* deinitialize DMA channel4 (USART0 rx) */
    dma_deinit(DMA0, DMA_CH4);
    dma_struct_para_init(&dma_init_struct);

    dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_addr = (uint32_t)rxbuffer;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = ARRAYNUM(rxbuffer);
    dma_init_struct.periph_addr = USART0_DATA_ADDRESS;
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA0, DMA_CH4, &dma_init_struct);
  
    /* configure DMA mode */
    dma_circulation_disable(DMA0, DMA_CH3);
    dma_memory_to_memory_disable(DMA0, DMA_CH3);
    dma_circulation_disable(DMA0, DMA_CH4);
    dma_memory_to_memory_disable(DMA0, DMA_CH4);
    
    /* USART DMA0 enable for reception */
    usart_dma_receive_config(USART0, USART_DENR_ENABLE);
    /* enable DMA0 channel4 transfer complete interrupt */
    dma_interrupt_enable(DMA0, DMA_CH4, DMA_INT_FTF);
    /* enable DMA0 channel4 */
    dma_channel_enable(DMA0, DMA_CH4);
		
    /* USART DMA0 enable for transmission */
    usart_dma_transmit_config(USART0, USART_DENT_ENABLE);
    /* enable DMA0 channel3 transfer complete interrupt */
    dma_interrupt_enable(DMA0, DMA_CH3, DMA_INT_FTF);
		/* disable DMA0 channel3 */
    dma_channel_disable(DMA0, DMA_CH3);
}

static void nvic_config(void)
{
		nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);
		nvic_irq_enable(DMA0_Channel4_IRQn, 0, 0);
		nvic_irq_enable(DMA0_Channel3_IRQn, 0, 0);
		nvic_irq_enable(USART0_IRQn, 10, 0);
}

void bsp_usart_init(uint32_t baudval) 
{   
	  UsartxSemaphore = xSemaphoreCreateBinary();
    nvic_config();
    usart_config(baudval);
    usart_dma_config();
}

void usart0_dma_send(uint8_t *buffer,uint16_t len)
{
	dma_channel_disable(DMA0, DMA_CH3);
	
	dma_memory_address_config(DMA0, DMA_CH3,(uint32_t)buffer);
	dma_transfer_number_config(DMA0, DMA_CH3, len);
	
	dma_channel_enable(DMA0, DMA_CH3);
	xSemaphoreTake(UsartxSemaphore, portMAX_DELAY);
}

uint16_t usart0_dma_recv(uint8_t *buffer)
{
	dma_channel_disable(DMA0, DMA_CH4);
	
	uint16_t rx_len = ARRAYNUM(rxbuffer) - dma_transfer_number_get(DMA0, DMA_CH4);
	memcpy(buffer,rxbuffer,rx_len);
	
	dma_memory_address_config(DMA0, DMA_CH4,(uint32_t)rxbuffer);
	dma_transfer_number_config(DMA0, DMA_CH4, ARRAYNUM(rxbuffer));	
	
	dma_channel_enable(DMA0, DMA_CH4);
	return rx_len;
}

/*DMA TX*/
void DMA0_Channel3_IRQHandler(void)
{
    if(dma_interrupt_flag_get(DMA0, DMA_CH3, DMA_INT_FLAG_FTF)){     
        dma_interrupt_flag_clear(DMA0, DMA_CH3, DMA_INT_FLAG_G);
    }
}

/*DMA RX*/
extern FIFO_BUFFER *Queue_log;
void DMA0_Channel4_IRQHandler(void)
{
    if(dma_interrupt_flag_get(DMA0, DMA_CH4, DMA_INT_FLAG_FTF)){     
			 FIFO_Recv(Queue_log);
			 dma_interrupt_flag_clear(DMA0, DMA_CH4, DMA_INT_FLAG_G);
    }
}

/* USART RX */
void USART0_IRQHandler(void)
{
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_IDLE)){
			  FIFO_Recv(Queue_log);
				usart_interrupt_flag_clear(USART0,USART_INT_FLAG_IDLE);
        USART_STAT0(USART0);
			  USART_DATA(USART0);
    }
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_TC)){
				BaseType_t pxHigherPriorityTaskWoken;
				xSemaphoreGiveFromISR(UsartxSemaphore,&pxHigherPriorityTaskWoken);
				usart_interrupt_flag_clear(USART0,USART_INT_FLAG_TC);
				portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
    }
}
