#include "fms.h"
#include "fifo.h"
#include "app_config.h"
#include <stdint.h>

void Receive_FMS(uint8_t data_byte)
{
    fms_state_t state = FMS_RECEIVE_STATE_IDLE;
	  switch(state)
		{
			case FMS_RECEIVE_STATE_IDLE:
				
			break;
			
			case FMS_RECEIVE_STATE_PREAMBLE:
				
			break;
			
			case FMS_RECEIVE_STATE_LENGTH_H:
				
			break;
			
			case FMS_RECEIVE_STATE_LENGTH_L:
				
			break;
			
			case FMS_RECEIVE_STATE_CMD_H:
				
			break;
			
			case FMS_RECEIVE_STATE_CMD_L:
				
			break;
			
			case FMS_RECEIVE_STATE_DATA:
				
			break;
			
			case FMS_RECEIVE_STATE_CRC_H:
				
			break;
			
			case FMS_RECEIVE_STATE_CRC_L:
				
			break;
			
			case FMS_RECEIVE_STATE_TAIL:
				
			break;
			
			default:
				
			break;
		
		}
}

void DataFrame_Handle(void)
{
	 while(FIFO_Count(Queue_Usart1_RX))
	 {
		  Receive_FMS(FIFO_Get(Queue_Usart1_RX));
	 }
}


