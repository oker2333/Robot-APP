#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "fifo.h"
#include "gd32f30x_libopt.h"
#include "app_config.h"
#include "print.h"

uint8_t console_buffer[CONSOLE_BUFFER_LEN] = {0};
uint8_t console_header = 0;
uint8_t console_tail = 0;

#if FIFO_DEBUG
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
		nvic_irq_enable(USART0_IRQn, 10, 0);
		usart_interrupt_flag_clear(USART0,USART_INT_FLAG_TC);
		usart_interrupt_flag_clear(USART0,USART_INT_FLAG_RBNE);
		usart_interrupt_enable(USART0, USART_INT_TC);
	  usart_interrupt_enable(USART0, USART_INT_RBNE);
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
  
    /* configure DMA mode */
    dma_circulation_disable(DMA0, DMA_CH3);
    dma_memory_to_memory_disable(DMA0, DMA_CH3);
    
    /* USART DMA0 enable for reception */
    usart_dma_receive_config(USART0, USART_DENR_ENABLE);
		nvic_irq_enable(DMA0_Channel4_IRQn, 0, 0);
		
		dma_interrupt_flag_clear(DMA0, DMA_CH3, DMA_INT_FLAG_FTF);
		
    /* USART DMA0 enable for transmission */
    usart_dma_transmit_config(USART0, USART_DENT_ENABLE);
		nvic_irq_enable(DMA0_Channel3_IRQn, 0, 0);
    /* enable DMA0 channel3 transfer complete interrupt */
    dma_interrupt_enable(DMA0, DMA_CH3, DMA_INT_FTF);
		/* disable DMA0 channel3 */
    dma_channel_disable(DMA0, DMA_CH3);
}

void dma_usart0_init(uint32_t baudval) 
{   
    usart_config(baudval);
    usart_dma_config();
}

void usart0_dma_send(uint8_t *buffer,uint16_t len)
{
	dma_channel_disable(DMA0, DMA_CH3);
	
	dma_memory_address_config(DMA0, DMA_CH3,(uint32_t)buffer);
	dma_transfer_number_config(DMA0, DMA_CH3, len);
	
	dma_channel_enable(DMA0, DMA_CH3);
	xSemaphoreTake(Usar0TxSemaphore, portMAX_DELAY);
}

/*DMA TX*/
void DMA0_Channel3_IRQHandler(void)
{
    if(dma_interrupt_flag_get(DMA0, DMA_CH3, DMA_INT_FLAG_FTF)){
        dma_interrupt_flag_clear(DMA0, DMA_CH3, DMA_INT_FLAG_FTF);		
        dma_interrupt_flag_clear(DMA0, DMA_CH3, DMA_INT_FLAG_G);
    }
}

/* USART  */
void USART0_IRQHandler(void)
{
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE)){			//RX
			  if(CONSOLE_BUFFER_LEN > (console_header - console_tail))
				{
				   console_buffer[console_header%CONSOLE_BUFFER_LEN] = usart_data_receive(USART0);
					 console_header++;
				}
				usart_interrupt_flag_clear(USART0,USART_INT_FLAG_RBNE);
    }
		
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_TC)){   //TX
				BaseType_t pxHigherPriorityTaskWoken;
				xSemaphoreGiveFromISR(Usar0TxSemaphore,&pxHigherPriorityTaskWoken);
				usart_interrupt_flag_clear(USART0,USART_INT_FLAG_TC);
				portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
    }
}

#else

void usart0_init(uint32_t baudval)
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
		nvic_irq_enable(USART0_IRQn, 9, 0);
	  usart_interrupt_flag_clear(USART0, USART_INT_FLAG_RBNE);
		usart_interrupt_enable(USART0, USART_INT_RBNE);

    usart_enable(USART0);
}

void USART0_IRQHandler(void)
{
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE)){			//RX
			  if(CONSOLE_BUFFER_LEN > (console_header - console_tail))
				{
				   console_buffer[console_header%CONSOLE_BUFFER_LEN] = usart_data_receive(USART0);
					 console_header++;
				}
				usart_interrupt_flag_clear(USART0,USART_INT_FLAG_RBNE);
    }
}

#endif

/************************************IAP USART******************************************************/

static void iap_usart_config(uint32_t baudval)
{
		rcu_periph_clock_enable(RCU_GPIOA);
		rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_USART1);
	
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);	//PA2--TX
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_3);	//PA3--RX

    usart_deinit(USART1);
    usart_baudrate_set(USART1, baudval);
    usart_word_length_set(USART1, USART_WL_8BIT);
    usart_stop_bit_set(USART1, USART_STB_1BIT);
    usart_parity_config(USART1, USART_PM_NONE);
    usart_hardware_flow_rts_config(USART1, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART1, USART_CTS_DISABLE);
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
		nvic_irq_enable(USART1_IRQn, 9, 0);
	
	  usart_interrupt_flag_clear(USART1, USART_INT_FLAG_IDLE);
	  usart_interrupt_flag_clear(USART1, USART_INT_FLAG_TC);
    usart_interrupt_enable(USART1, USART_INT_IDLE);
		
		usart_interrupt_enable(USART1, USART_INT_TC);
    usart_enable(USART1);
}

static uint8_t IAP_Buffer[IAP_BUFFER_LEN];

static void isp_usart_dma_config(void)
{
    dma_parameter_struct dma_init_struct;
    /* enable DMA0 clock */
    rcu_periph_clock_enable(RCU_DMA0);
    
    /* deinitialize DMA channel6(USART1 tx) */
    dma_deinit(DMA0, DMA_CH6);
    dma_struct_para_init(&dma_init_struct);

    dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_addr = NULL;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = 0;
    dma_init_struct.periph_addr = USART1_DATA_ADDRESS;
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA0, DMA_CH6, &dma_init_struct);
    
    /* deinitialize DMA channel5 (USART1 rx) */
    dma_deinit(DMA0, DMA_CH5);
    dma_struct_para_init(&dma_init_struct);

    dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_addr = (uint32_t)IAP_Buffer;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = ARRAYNUM(IAP_Buffer);
    dma_init_struct.periph_addr = USART1_DATA_ADDRESS;
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA0, DMA_CH5, &dma_init_struct);
  
    /* configure DMA mode */
    dma_circulation_disable(DMA0, DMA_CH5);
    dma_memory_to_memory_disable(DMA0, DMA_CH5);
    dma_circulation_disable(DMA0, DMA_CH6);
    dma_memory_to_memory_disable(DMA0, DMA_CH6);
    
    /* USART DMA0 enable for reception */
    usart_dma_receive_config(USART1, USART_DENR_ENABLE);

		nvic_irq_enable(DMA0_Channel5_IRQn, 0, 0);
		
		dma_interrupt_flag_clear(DMA0, DMA_CH5, DMA_INT_FLAG_FTF);
		
    /* enable DMA0 channel5 transfer complete interrupt */
    dma_interrupt_enable(DMA0, DMA_CH5, DMA_INT_FTF);
    /* enable DMA0 channel5 */
    dma_channel_enable(DMA0, DMA_CH5);
		
    /* USART DMA0 enable for transmission */
    usart_dma_transmit_config(USART1, USART_DENT_ENABLE);
		
		nvic_irq_enable(DMA0_Channel6_IRQn, 0, 0);
		
		dma_interrupt_flag_clear(DMA0, DMA_CH6, DMA_INT_FLAG_FTF);
		
    /* enable DMA0 channel6 transfer complete interrupt */
    dma_interrupt_enable(DMA0, DMA_CH6, DMA_INT_FTF);
		/* disable DMA0 channel6 */
    dma_channel_disable(DMA0, DMA_CH6);
}

void usart1_dma_send(uint8_t *buffer,uint16_t len)
{
	dma_channel_disable(DMA0, DMA_CH6);
	
	dma_memory_address_config(DMA0, DMA_CH6,(uint32_t)buffer);
	dma_transfer_number_config(DMA0, DMA_CH6, len);
	
	dma_channel_enable(DMA0, DMA_CH6);
	xSemaphoreTake(Usart1TxSemaphore, portMAX_DELAY);
}

uint16_t usart1_dma_recv(uint8_t *buffer)
{
	dma_channel_disable(DMA0, DMA_CH5);
	
	uint16_t rx_len = ARRAYNUM(IAP_Buffer) - dma_transfer_number_get(DMA0, DMA_CH5);
	Mem_Copy(buffer,IAP_Buffer,rx_len);
	
	#if PRINT_HOST_FRAME
	print_info("[host]");
	for(int i = 0;i < rx_len;i++)
	{
		 printf("0x%x ",IAP_Buffer[i]);
	}
	printf("\n");
	#endif
	
	dma_memory_address_config(DMA0, DMA_CH5,(uint32_t)IAP_Buffer);
	dma_transfer_number_config(DMA0, DMA_CH5, ARRAYNUM(IAP_Buffer));	
	
	dma_channel_enable(DMA0, DMA_CH5);
	return rx_len;
}

void dma_usart1_init(uint32_t baudval)
{  
    iap_usart_config(baudval);
    isp_usart_dma_config();
}

/*DMA TX*/
void DMA0_Channel6_IRQHandler(void)
{
    if(dma_interrupt_flag_get(DMA0, DMA_CH6, DMA_INT_FLAG_FTF)){
        dma_interrupt_flag_clear(DMA0, DMA_CH6, DMA_INT_FLAG_FTF);			
        dma_interrupt_flag_clear(DMA0, DMA_CH6, DMA_INT_FLAG_G);
    }
}

/*DMA RX*/
void DMA0_Channel5_IRQHandler(void)
{
    if(dma_interrupt_flag_get(DMA0, DMA_CH5, DMA_INT_FLAG_FTF)){
       dma_interrupt_flag_clear(DMA0, DMA_CH5, DMA_INT_FLAG_FTF);
			 dma_interrupt_flag_clear(DMA0, DMA_CH5, DMA_INT_FLAG_G);
    }
}

/* USART */
void USART1_IRQHandler(void)
{
	  BaseType_t pxHigherPriorityTaskWoken;
    if(RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_IDLE)){		//RX
			  FIFO_Recv(Queue_Usart1_RX);
				usart_interrupt_flag_clear(USART1,USART_INT_FLAG_IDLE);
        USART_STAT0(USART1);
			  USART_DATA(USART1);
				xSemaphoreGiveFromISR(CommunicateSemaphore, &pxHigherPriorityTaskWoken);
				portYIELD_FROM_ISR((pxHigherPriorityTaskWoken));
    }
    if(RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_TC)){			//TX
				xSemaphoreGiveFromISR(Usart1TxSemaphore,&pxHigherPriorityTaskWoken);
				usart_interrupt_flag_clear(USART1,USART_INT_FLAG_TC);
				portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);			
    }
}
